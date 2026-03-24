"""Orchestrates calibration across one or more iC-MU encoders.

``EncoderCalibrator`` owns the motor movement logic and coordinates the
per-encoder calibration loop.  A single motor spin captures raw data from
all enrolled encoders simultaneously; each encoder's analog calibration
then proceeds independently.
"""

from __future__ import annotations

import json
import logging
import time
from pathlib import Path
from typing import TYPE_CHECKING

import mu_3sl_interface as mu_3sl
from ingenialink.exceptions import ILIOError

from .encoder import CalibrationResult, DriveFrameConfig, Encoder, ICMURegisterState
from .motor_control import DEFAULT_GEN_CURRENT, DEFAULT_GEN_FREQ, MotorControl
from .plotting import (
    _extract_residuals,
    plot_raw_waveforms,
    plot_residuals_bar,
    plot_residuals_trend,
)

if TYPE_CHECKING:
    from collections.abc import Iterable

    from ingeniamotion import MotionController

logger = logging.getLogger(__name__)

# Raw data bit widths for master/nonius tracks
_MASTER_WIDTH = 14
_NONIUS_WIDTH = 14
_MASTER_MASK = (1 << _MASTER_WIDTH) - 1
_NONIUS_MASK = (1 << _NONIUS_WIDTH) - 1

# Default data acquisition parameters
_DEFAULT_SAMPLING_TIME_S = 0.01
_DEFAULT_CAPTURE_DURATION_S = 30.0

# Convergence threshold (all 8 residuals must be <= this)
_RESIDUAL_THRESHOLD = 1.0


def _flatten(nested: Iterable[Iterable[int]]) -> Iterable[int]:
    """Flatten a nested iterable of integers.

    Yields:
        Each integer from the nested structure.
    """
    for row in nested:
        yield from row


def _split_raw_payload(payload: int) -> tuple[int, int]:
    """Extract 14-bit master and nonius from a packed BiSS payload.

    Returns:
        Tuple of (master, nonius) 14-bit values.
    """
    master = payload & _MASTER_MASK
    nonius = (payload >> _NONIUS_WIDTH) & _NONIUS_MASK
    return master, nonius


class EncoderCalibrator:
    """Orchestrates calibration for one or more iC-MU encoders.

    Args:
        mc: Connected MotionController instance.
        axis: Drive axis number.
        max_iterations: Maximum analog calibration iterations.
        gen_frequency: Saw-tooth generator frequency in Hz.
        gen_current: Quadrature current target in amps.
    """

    def __init__(
        self,
        mc: MotionController,
        *,
        axis: int = 1,
        max_iterations: int = 3,
        gen_frequency: float = DEFAULT_GEN_FREQ,
        gen_current: float = DEFAULT_GEN_CURRENT,
        output_dir: Path | None = None,
        interactive_plots: bool = False,
    ) -> None:
        self._mc = mc
        self._axis = axis
        self._max_iterations = max_iterations
        self._motor = MotorControl(
            mc, axis=axis, gen_frequency=gen_frequency, gen_current=gen_current
        )
        self._encoders: list[Encoder] = []
        self._output_dir = output_dir or Path("calibration_output")
        self._interactive_plots = interactive_plots

    # -- Encoder management --

    def add_encoder(self, encoder_number: int) -> Encoder:
        """Create and register an Encoder for the given channel.

        Args:
            encoder_number: 1 or 2.

        Returns:
            The newly created Encoder instance.
        """
        enc = Encoder(self._mc, encoder_number, axis=self._axis)
        self._encoders.append(enc)
        logger.info("Registered encoder %d for calibration.", encoder_number)
        return enc

    @property
    def encoders(self) -> list[Encoder]:
        """Return the list of enrolled encoders."""
        return list(self._encoders)

    # -- Motor control --

    def configure_internal_generator(self) -> None:
        """Configure the drive for internal generator mode."""
        self._motor.configure_internal_generator()

    # -- Data acquisition --

    def acquire_raw_data(
        self,
        *,
        duration_s: float = _DEFAULT_CAPTURE_DURATION_S,
        sampling_time_s: float = _DEFAULT_SAMPLING_TIME_S,
    ) -> dict[int, tuple[list[int], list[int]]]:
        """Capture raw master/nonius data from all enrolled encoders.

        The motor must already be spinning.

        Args:
            duration_s: Capture duration in seconds.
            sampling_time_s: Interval between SDO read cycles in seconds.

        Returns:
            Mapping of encoder number to (master_raw, nonius_raw) lists.
        """
        channels: list[list[int]] = [[] for _ in self._encoders]
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            for idx, enc in enumerate(self._encoders):
                try:
                    val = int(
                        self._mc.communication.get_register(
                            enc.regs.pos_value,
                            axis=self._axis,
                        )
                    )
                except ILIOError:
                    logger.debug("Skipped sample for encoder %d (SDO read error).", enc.number)
                    continue
                channels[idx].append(val)
            time.sleep(sampling_time_s)

        result: dict[int, tuple[list[int], list[int]]] = {}
        for enc, raw_values in zip(self._encoders, channels):
            masters: list[int] = []
            nonius: list[int] = []
            for val in raw_values:
                m, n = _split_raw_payload(val)
                masters.append(m)
                nonius.append(n)
            result[enc.number] = (masters, nonius)
            logger.info(
                "Encoder %d: captured %d samples (master range %d-%d, nonius range %d-%d).",
                enc.number,
                len(masters),
                min(masters) if masters else 0,
                max(masters) if masters else 0,
                min(nonius) if nonius else 0,
                max(nonius) if nonius else 0,
            )

        return result

    # -- Calibration orchestration --

    def calibrate(self) -> dict[int, CalibrationResult]:
        """Run the full calibration procedure for all enrolled encoders.

        Returns:
            Mapping of encoder number to CalibrationResult.

        Raises:
            RuntimeError: If no encoders have been registered.
        """
        if not self._encoders:
            msg = "No encoders registered. Call add_encoder() first."
            raise RuntimeError(msg)

        results: dict[int, CalibrationResult] = {}
        n_master_periods: dict[int, int] = {}
        saved_drive_configs: dict[int, DriveFrameConfig] = {}
        saved_ic_configs: dict[int, ICMURegisterState] = {}
        calibrations: dict[int, mu_3sl.Calibration] = {}

        try:
            # -- Setup phase (per-encoder) --
            for enc in self._encoders:
                enc.ensure_normal_mode()
                revision = enc.read_revision()
                saved_drive_configs[enc.number] = enc.get_drive_config()
                saved_ic_configs[enc.number] = enc.get_ic_config()
                n_periods = enc.configure_in_calibration_mode()
                n_master_periods[enc.number] = n_periods
                calibrations[enc.number] = mu_3sl.Calibration(revision)

            # -- Iterative analog calibration --
            converged: set[int] = set()
            iteration_count: dict[int, int] = {e.number: 0 for e in self._encoders}
            residual_history: dict[int, list[list[float]]] = {
                e.number: [] for e in self._encoders
            }
            iteration_log: dict[int, list[dict]] = {
                e.number: [] for e in self._encoders
            }
            self._output_dir.mkdir(parents=True, exist_ok=True)

            for iteration in range(1, self._max_iterations + 1):
                pending = [e for e in self._encoders if e.number not in converged]
                if not pending:
                    break

                logger.info("--- Iteration %d ---", iteration)

                with self._motor.running():
                    raw_data = self.acquire_raw_data()

                for enc in pending:
                    master_raw, nonius_raw = raw_data[enc.number]
                    iteration_count[enc.number] = iteration
                    cal = calibrations[enc.number]

                    # B1 fix: sync DLL with current chip state
                    master_adj, nonius_adj = enc.read_analog_adjustments()
                    cal.set_current_analog_track_adjustments(
                        master_adj,
                        nonius_adj,
                    )

                    analyze_result = cal.analyze_raw_data(
                        master_raw,
                        nonius_raw,
                    )

                    # Log residuals for convergence diagnostics
                    m_rel = analyze_result.relative_master_track_adjustments()
                    n_rel = analyze_result.relative_nonius_track_adjustments()
                    logger.info(
                        "Encoder %d iter %d residuals: "
                        "M(gx=%.2f, voss=%.2f, vosc=%.2f, ph=%.2f) "
                        "N(gx=%.2f, voss=%.2f, vosc=%.2f, ph=%.2f)",
                        enc.number,
                        iteration,
                        m_rel.cosine_gain_lsb,
                        m_rel.sine_offset_lsb,
                        m_rel.cosine_offset_lsb,
                        m_rel.phase_lsb,
                        n_rel.cosine_gain_lsb,
                        n_rel.sine_offset_lsb,
                        n_rel.cosine_offset_lsb,
                        n_rel.phase_lsb,
                    )

                    # -- Diagnostic data collection --
                    residuals = _extract_residuals(analyze_result)
                    residual_history[enc.number].append(residuals)
                    iteration_log[enc.number].append({
                        "iteration": iteration,
                        "master_raw": master_raw,
                        "nonius_raw": nonius_raw,
                        "analog_adjustments": {
                            "master": {
                                "cosine_gain": int(master_adj.cosine_gain),
                                "sine_offset": int(master_adj.sine_offset),
                                "cosine_offset": int(master_adj.cosine_offset),
                                "phase": int(master_adj.phase),
                            },
                            "nonius": {
                                "cosine_gain": int(nonius_adj.cosine_gain),
                                "sine_offset": int(nonius_adj.sine_offset),
                                "cosine_offset": int(nonius_adj.cosine_offset),
                                "phase": int(nonius_adj.phase),
                            },
                        },
                        "residuals": {
                            "master": {
                                "cosine_gain_lsb": float(m_rel.cosine_gain_lsb),
                                "sine_offset_lsb": float(m_rel.sine_offset_lsb),
                                "cosine_offset_lsb": float(m_rel.cosine_offset_lsb),
                                "phase_lsb": float(m_rel.phase_lsb),
                            },
                            "nonius": {
                                "cosine_gain_lsb": float(n_rel.cosine_gain_lsb),
                                "sine_offset_lsb": float(n_rel.sine_offset_lsb),
                                "cosine_offset_lsb": float(n_rel.cosine_offset_lsb),
                                "phase_lsb": float(n_rel.phase_lsb),
                            },
                        },
                        "converged": _is_converged(analyze_result),
                    })

                    # -- Diagnostic plots --
                    plot_raw_waveforms(
                        master_raw,
                        nonius_raw,
                        encoder=enc.number,
                        iteration=iteration,
                        output_dir=self._output_dir,
                        interactive=self._interactive_plots,
                    )
                    plot_residuals_bar(
                        analyze_result,
                        encoder=enc.number,
                        iteration=iteration,
                        output_dir=self._output_dir,
                        interactive=self._interactive_plots,
                    )
                    plot_residuals_trend(
                        residual_history,
                        encoder=enc.number,
                        output_dir=self._output_dir,
                        interactive=self._interactive_plots,
                    )

                    if _is_converged(analyze_result):
                        converged.add(enc.number)
                        logger.info("Encoder %d: converged at iteration %d.", enc.number, iteration)
                    else:
                        cal.adjust_analog_by_analyze_result(analyze_result)
                        new_master = cal.analog_master_track_adjustments()
                        new_nonius = cal.analog_nonius_track_adjustments()
                        enc.write_analog_adjustments(new_master, new_nonius)
                        logger.info("Encoder %d: analog params adjusted.", enc.number)

            # -- Export iteration data as JSON --
            for enc_num, log_entries in iteration_log.items():
                if log_entries:
                    json_path = self._output_dir / f"enc{enc_num}_calibration_data.json"
                    json_path.write_text(
                        json.dumps(log_entries, indent=2),
                        encoding="utf-8",
                    )
                    logger.info("Exported calibration data: %s", json_path)

            # -- Post-loop: nonius + EEPROM for converged encoders --
            for enc in self._encoders:
                if enc.number in converged:
                    results[enc.number] = self._finalize_encoder(
                        enc,
                        iteration_count[enc.number],
                        saved_ic_configs[enc.number],
                        calibrations[enc.number],
                    )
                else:
                    logger.warning(
                        "Encoder %d: did NOT converge after %d iterations. Skipping EEPROM save.",
                        enc.number,
                        self._max_iterations,
                    )
                    results[enc.number] = CalibrationResult(
                        success=False,
                        iterations=iteration_count[enc.number],
                    )

        finally:
            # -- Always restore drive encoder frame config --
            for enc in self._encoders:
                try:
                    enc.set_ic_config(saved_ic_configs[enc.number])
                except Exception:
                    logger.exception("Failed to restore iC-MU regs for encoder %d.", enc.number)
                try:
                    enc.set_drive_config(saved_drive_configs[enc.number])
                except Exception:
                    logger.exception("Failed to restore drive config for encoder %d.", enc.number)

        return results

    # -- Finalization helpers --

    def _finalize_encoder(
        self,
        enc: Encoder,
        iterations: int,
        ic_state: ICMURegisterState,
        cal: mu_3sl.Calibration,
    ) -> CalibrationResult:
        """Optimize nonius, save to EEPROM, and build the result for one encoder.

        Args:
            enc: The converged Encoder.
            iterations: How many iterations were used.
            ic_state: Saved iC-MU register state to restore before EEPROM save.
            cal: The mu_3sl Calibration object for this encoder.

        Returns:
            CalibrationResult for this encoder.
        """
        # Re-acquire data one last time with converged analog params
        # to get the best nonius table.
        with self._motor.running():
            raw_data = self.acquire_raw_data()

        master_raw, nonius_raw = raw_data[enc.number]
        master_adj, nonius_adj = enc.read_analog_adjustments()
        cal.set_current_analog_track_adjustments(master_adj, nonius_adj)
        analyze_result = cal.analyze_raw_data(master_raw, nonius_raw)

        # Nonius SPO optimization
        nonius_table = analyze_result.optimized_nonius_track_offset_table()
        cal.set_current_nonius_track_offset_table(nonius_table)
        table_params = mu_3sl.nonius_track_offset_table_parameters(nonius_table)
        enc.write_nonius_parameters(table_params)

        # Restore iC-MU config registers (exits raw mode) before EEPROM save
        enc.set_ic_config(ic_state)

        # Save to EEPROM
        success = enc.save_to_eeprom()

        final_master = cal.analog_master_track_adjustments()
        final_nonius = cal.analog_nonius_track_adjustments()

        spo_n = [table_params.spo_n[i] for i in range(15)]

        return CalibrationResult(
            success=success,
            iterations=iterations,
            master_adjustments=final_master,
            nonius_adjustments=final_nonius,
            spo_base=table_params.spo_base,
            spo_n=spo_n,
        )


def _is_converged(analyze_result: mu_3sl.AnalyzeResult) -> bool:
    """Check whether all 8 analog residual errors are within threshold.

    Args:
        analyze_result: Result from ``Calibration.analyze_raw_data()``.

    Returns:
        True if all residuals are <= 1.0 LSB.
    """
    master_rel = analyze_result.relative_master_track_adjustments()
    nonius_rel = analyze_result.relative_nonius_track_adjustments()

    return all(
        abs(v) <= _RESIDUAL_THRESHOLD
        for v in (
            master_rel.cosine_gain_lsb,
            master_rel.sine_offset_lsb,
            master_rel.cosine_offset_lsb,
            master_rel.phase_lsb,
            nonius_rel.cosine_gain_lsb,
            nonius_rel.sine_offset_lsb,
            nonius_rel.cosine_offset_lsb,
            nonius_rel.phase_lsb,
        )
    )
