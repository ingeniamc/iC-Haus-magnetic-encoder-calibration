"""Orchestrates calibration across one or more iC-MU encoders.

``EncoderCalibrator`` owns the motor movement logic and coordinates the
per-encoder calibration loop.  A single motor spin captures raw data from
all enrolled encoders simultaneously; each encoder's analog calibration
then proceeds independently.

Data acquisition uses EtherCAT PDOs (TPDO map on the encoder position
registers) for deterministic, high-rate sampling that runs in the same
PDO exchange thread as the FSoE safety protocol.
"""

import json
import logging
import shutil
import threading
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import mu_3sl_interface as mu_3sl
from ingenialink.pdo import RPDOMap, RPDOMapItem, TPDOMap
from ingeniamotion import MotionController
from ingeniamotion.enums import SensorType

from ic_haus_magnetic_encoder_calibration.config_loader import (
    EncoderRegisterConfig,
)

from .encoder import (
    CalibrationResult,
    DriveFrameConfig,
    Encoder,
    ICMURegisterState,
    split_raw_payload,
)
from .motor_control import DEFAULT_GEN_CURRENT, DEFAULT_GEN_FREQ, MotorControl
from .plotting import (
    RESIDUAL_THRESHOLD,
    _plot_nonius_track_offset_table,
    _plot_raw_waveforms,
    _plot_residuals_bar,
    _plot_residuals_trend,
    warm_matplotlib_cache,
)

logger = logging.getLogger(__name__)

# Default data acquisition parameters
DEFAULT_CAPTURE_DURATION_S = 30.0
DEFAULT_PDO_RATE_S = 0.001  # 1 ms
DEFAULT_PDO_EXCEPTION_INTERVAL_S = 0.5

# iC-MU AN3 "CALIBRATION" step 18 recommends keeping both Nonius "In Range"
# values below 60% (i.e. at least 40% margin to either side).
NONIUS_IN_RANGE_RECOMMENDED_MAX_PERCENT = 60.0


@dataclass
class NoniusInRangeResult:
    """Result of the Nonius "In Range" calculation.

    Attributes:
        range_limit: The Nonius phase range limit (LSB).
        margin_max: The Nonius phase margin max (LSB).
        margin_min: The Nonius phase margin min (LSB).
        in_range_max: The Nonius "In Range" Max value (percentage).
        in_range_min: The Nonius "In Range" Min value (percentage).
    """

    range_limit: int
    margin_max: int
    margin_min: int
    in_range_max: float
    in_range_min: float


def _extract_residuals(
    analyze_result: mu_3sl.AnalyzeResult,
) -> list[float]:
    """Return the 8 residual values as a flat list.

    Returns:
        List of 8 floats: [master gx, voss, vosc, ph, nonius gx, voss, vosc, ph].
    """
    m = analyze_result.relative_master_track_adjustments()
    n = analyze_result.relative_nonius_track_adjustments()
    return [
        m.cosine_gain_lsb,
        m.sine_offset_lsb,
        m.cosine_offset_lsb,
        m.phase_lsb,
        n.cosine_gain_lsb,
        n.sine_offset_lsb,
        n.cosine_offset_lsb,
        n.phase_lsb,
    ]


class _SingleEncoderCalibration:
    """Per-encoder calibration state and iteration logic.

    Each instance tracks the calibration progress for exactly one iC-MU
    encoder through setup, iterative analog calibration, and cleanup.
    """

    def __init__(self, enc: Encoder) -> None:
        self.enc = enc
        self._cal: Optional[mu_3sl.Calibration] = None
        self.n_master_periods: int = 0
        self.saved_drive_config: Optional[DriveFrameConfig] = None
        self.saved_ic_config: Optional[ICMURegisterState] = None
        self.converged: bool = False
        self.iteration_count: int = 0
        self.residual_history: list[list[float]] = []
        self.iteration_log: list[dict[str, object]] = []
        self.last_analyze_result: Optional[mu_3sl.AnalyzeResult] = None
        self.last_raw_data: Optional[tuple[list[int], list[int]]] = None

    @property
    def number(self) -> int:
        """Encoder channel number."""
        return self.enc.number

    @property
    def pending(self) -> bool:
        """True if this encoder has not yet converged."""
        return not self.converged

    @property
    def cal(self) -> mu_3sl.Calibration:
        """The mu_3sl Calibration object (available after :meth:`save_state`).

        Raises:
            RuntimeError: If :meth:`save_state` has not been called.
        """
        if self._cal is None:
            msg = "Calibration not initialized. Call save_state() first."
            raise RuntimeError(msg)
        return self._cal

    # -- Setup phases --

    def save_state(self) -> None:
        """Phase 1: Apply configuration, read revision, save configs.

        Raises:
            ValueError: If the encoder is not configured for BiSS-C protocol.

        """
        if not self.enc.is_bissc:
            raise ValueError(f"Encoder {self.number} is not set as a BiSS-C sensor.")
        self.enc.apply_config()
        revision = self.enc.read_revision()
        self.saved_drive_config = self.enc.get_drive_config()
        self.saved_ic_config = self.enc.get_ic_config()
        self._cal = mu_3sl.Calibration(revision)

    def enter_calibration_mode(self) -> None:
        """Phase 2: configure encoder + drive frame for calibration."""
        self.n_master_periods = self.enc.configure_in_calibration_mode()
        self.cal.preconfigure_number_of_master_periods(self.n_master_periods)

    def reset_analog(self) -> None:
        """Phase 3: reset analog parameters to factory defaults."""
        self.enc.reset_analog_to_factory_defaults()

    # -- Iteration --

    @staticmethod
    def is_converged(analyze_result: mu_3sl.AnalyzeResult) -> bool:
        """Check whether all 8 analog residual errors are within threshold.

        Args:
            analyze_result: Result from ``Calibration.analyze_raw_data()``.

        Returns:
            True if all residuals are <= 1.0 LSB.
        """
        master_rel = analyze_result.relative_master_track_adjustments()
        nonius_rel = analyze_result.relative_nonius_track_adjustments()

        return all(
            abs(v) <= RESIDUAL_THRESHOLD
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

    def get_nonius_in_range(self, analyze_result: mu_3sl.AnalyzeResult) -> NoniusInRangeResult:
        """Compute the Nonius In Range, matching the iC-Haus GUI.

        The GUI recommends keeping both Min and Max values below
        :data:`NONIUS_IN_RANGE_RECOMMENDED_MAX_PERCENT` (60%), i.e. at least
        40% margin to either side.

        Args:
            analyze_result: Result from ``Calibration.analyze_raw_data()``.

        Returns:
            Nonius 'In Range %' results.
        """
        range_limit = analyze_result.nonius_phase_range_limit()
        margin_max = analyze_result.nonius_phase_margin_max()
        margin_min = analyze_result.nonius_phase_margin_min()
        in_range_max = margin_max / range_limit * 100
        in_range_min = margin_min / -range_limit * 100

        return NoniusInRangeResult(
            range_limit=range_limit,
            margin_max=margin_max,
            margin_min=margin_min,
            in_range_max=in_range_max,
            in_range_min=in_range_min,
        )

    def is_in_range(
        self,
        analyze_result: mu_3sl.AnalyzeResult,
        in_range_threshold: Optional[float],
    ) -> bool:
        """Check whether the Nonius In Range Max & Min values are below the threshold.

        Args:
            analyze_result: Result from ``Calibration.analyze_raw_data()``.
            in_range_threshold: Threshold percentage (0-100) for In Range check.

        Returns:
            True if both the "max" and "min" In Range values are below the threshold,
                False otherwise.
        """
        if not in_range_threshold:
            in_range_threshold = NONIUS_IN_RANGE_RECOMMENDED_MAX_PERCENT
        nonius_in_range_result = self.get_nonius_in_range(analyze_result)
        in_range_max = nonius_in_range_result.in_range_max
        if in_range_max > in_range_threshold:
            logger.warning(
                f"Encoder {self.number} Nonius InRange Max {in_range_max:.1f}% exceeds threshold "
                f"{in_range_threshold:.1f}%."
            )
            return False
        in_range_min = nonius_in_range_result.in_range_min
        if in_range_min > in_range_threshold:
            logger.warning(
                f"Encoder {self.number} Nonius InRange Min {in_range_min:.1f}% exceeds threshold "
                f"{in_range_threshold:.1f}%."
            )
            return False
        return True

    def process_iteration(
        self,
        iteration: int,
        raw_data: list[int],
        output_dir: Path,
        interactive: bool,
        *,
        save_raw_plots: bool = False,
        save_residual_bar_plots: bool = False,
        save_trend_plot: bool = True,
        force_in_range_threshold: Optional[float] = None,
    ) -> None:
        """Run one calibration iteration: analyze, log, plot, correct.

        Updates :attr:`iteration_count`, :attr:`residual_history`,
        :attr:`iteration_log`, and potentially sets :attr:`converged`.

        Args:
            iteration: Current iteration number (1-based).
            raw_data: List of packed 32-bit register values from the drive.
            output_dir: Directory for diagnostic plot PNGs.
            interactive: If True, show plots interactively instead of saving.
            save_raw_plots: If True, save raw waveform plots for this iteration.
            save_residual_bar_plots: If True, save residual bar plots for this iteration.
            save_trend_plot: If True, save residuals trend plot (one per encoder).
            force_in_range_threshold: If not None, treat Nonius In Range > threshold as
                a calibration failure.

        Raises:
            RuntimeError: If monitoring data is empty or non-positive.

        """
        self.iteration_count = iteration

        if not raw_data:
            logger.warning(
                f"Encoder {self.number}: 0 samples captured at iteration"
                f" {iteration} — PDO exchange may have died. Stopping.",
            )
            raise RuntimeError("No samples captured. PDO exchange may have died.")

        # Unpack packed register values into master / nonius tracks.
        master_raw: list[int] = []
        nonius_raw: list[int] = []
        for val in raw_data:
            m, n = split_raw_payload(val)
            master_raw.append(m)
            nonius_raw.append(n)

        # B1 fix: sync DLL with current chip state
        master_adj, nonius_adj = self.enc.read_analog_adjustments()
        self.cal.set_current_analog_track_adjustments(master_adj, nonius_adj)

        # Reinforce MPC hint so the library doesn't re-detect a wrong count.
        self.cal.preconfigure_number_of_master_periods(self.n_master_periods)

        analyze_result = self.cal.analyze_raw_data(master_raw, nonius_raw)

        # -- Diagnostics: logging --
        ar = analyze_result
        logger.info(
            f"Encoder {self.number} iter {iteration} analysis: "
            f"valid={ar.is_analog_analyses_valid()}, "
            f"calc_periods={ar.number_of_calculated_master_periods()}, "
            f"revolutions={ar.number_of_revolutions():.2f}, "
            f"acquired_periods={ar.number_of_acquired_master_periods():.1f}, "
            f"avg_samples/period={ar.average_number_of_samples_per_master_period():.1f}, "
            f"min_samples/period={ar.minimal_number_of_samples_per_master_period():.1f}",
        )

        m_rel = analyze_result.relative_master_track_adjustments()
        n_rel = analyze_result.relative_nonius_track_adjustments()
        logger.info(
            f"Encoder {self.number} iter {iteration} residuals: "
            f"M(gx={m_rel.cosine_gain_lsb:.2f}, voss={m_rel.sine_offset_lsb:.2f}, "
            f"vosc={m_rel.cosine_offset_lsb:.2f}, ph={m_rel.phase_lsb:.2f}) "
            f"N(gx={n_rel.cosine_gain_lsb:.2f}, voss={n_rel.sine_offset_lsb:.2f}, "
            f"vosc={n_rel.cosine_offset_lsb:.2f}, ph={n_rel.phase_lsb:.2f})",
        )

        # -- Diagnostics: data collection --
        residuals = _extract_residuals(analyze_result)
        in_range = self.get_nonius_in_range(analyze_result)
        logger.info(
            f"Encoder {self.number} iter {iteration} nonius in range: "
            f"Max={in_range.in_range_max:.1f}%, Min={in_range.in_range_min:.1f}%",
        )
        self.residual_history.append(residuals)
        self.iteration_log.append({
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
            "nonius phase margin": {
                "InRange max %": in_range.in_range_max,
                "InRange min %": in_range.in_range_min,
                "phase margin max": in_range.margin_max,
                "phase margin min": in_range.margin_min,
                "phase range limit": in_range.range_limit,
            },
            "converged": self.is_converged(analyze_result),
        })

        # -- Diagnostics: plots --
        if save_raw_plots:
            _plot_raw_waveforms(
                master_raw,
                nonius_raw,
                encoder=self.number,
                iteration=iteration,
                output_dir=output_dir,
                interactive=interactive,
            )
        if save_residual_bar_plots:
            _plot_residuals_bar(
                residuals,
                encoder=self.number,
                iteration=iteration,
                output_dir=output_dir,
                interactive=interactive,
            )
        if save_trend_plot:
            _plot_residuals_trend(
                {self.number: self.residual_history},
                encoder=self.number,
                output_dir=output_dir,
                interactive=interactive,
            )

        # -- Convergence check / apply corrections --
        self.last_analyze_result = analyze_result
        self.last_raw_data = (master_raw, nonius_raw)
        if self.is_converged(analyze_result):
            self.converged = True
            logger.info(f"Encoder {self.number}: converged at iteration {iteration}.")

            # Check InRange
            if (
                not self.is_in_range(analyze_result, force_in_range_threshold)
                and force_in_range_threshold
            ):
                logger.warning("Force-in-range is enabled: treating this as a calibration failure.")
                self.converged = False

        else:
            self.cal.adjust_analog_by_analyze_result(analyze_result)
            new_master = self.cal.analog_master_track_adjustments()
            new_nonius = self.cal.analog_nonius_track_adjustments()
            self.enc.write_analog_adjustments(new_master, new_nonius)
            logger.info(f"Encoder {self.number}: analog params adjusted.")

    # -- Cleanup --

    def restore_state(self) -> None:
        """Restore drive and iC-MU config (always called, even on error)."""
        try:
            if self.saved_ic_config is None:
                logger.warning(f"Encoder {self.number}: no saved iC-MU config to restore.")
            else:
                # Restore iC-MU config registers (exits raw mode) before EEPROM save.
                self.enc.set_ic_config(self.saved_ic_config)
                try:
                    # Save to EEPROM (raises on failure)
                    self.enc.save_to_eeprom()

                except RuntimeError:
                    logger.error(f"Encoder {self.number}: could not save configuration to EEPROM.")

                # Perform an internal reset of the encoder
                self.enc.abs_reset()

            if self.saved_drive_config is None:
                logger.warning(f"Encoder {self.number}: no saved drive config to restore.")
            else:
                # Restore drive frame config (exits calibration mode) after EEPROM save.
                self.enc.set_drive_config(self.saved_drive_config)

        except Exception:
            logger.warning(
                f"Encoder {self.number}: could not restore state (drive may be offline).",
                exc_info=True,
            )

    def export_data(self, output_dir: Path) -> None:
        """Export iteration log as JSON."""
        if self.iteration_log:
            json_path = output_dir / f"enc{self.number}_calibration_data.json"
            json_path.write_text(
                json.dumps(self.iteration_log, indent=2),
                encoding="utf-8",
            )
            logger.info(f"Exported calibration data: {json_path}")

    def finalize(
        self,
    ) -> CalibrationResult:
        """Finalize calibration: optimize SPO, restore config, save to EEPROM.

        Returns:
            CalibrationResult with success status and iteration count.
        """
        enc = self.enc
        cal = self.cal

        # Use the last iteration's analysis result (no re-acquisition).
        assert self.last_analyze_result is not None
        analyze_result = self.last_analyze_result

        # Nonius SPO optimization
        nonius_table = analyze_result.optimized_nonius_track_offset_table()
        cal.set_current_nonius_track_offset_table(nonius_table)
        table_params = mu_3sl.nonius_track_offset_table_parameters(nonius_table)
        spo_n = [table_params.spo_n[i] for i in range(15)]
        logger.info(f"SPO base={table_params.spo_base} spo_n={spo_n}")
        enc.write_nonius_parameters(table_params)

        # Re-analyze the same raw data with the SPO table applied, so the nonius
        # curves and the InRange % reflect the final configuration.
        if self.last_raw_data is not None:
            master_raw, nonius_raw = self.last_raw_data
            cal.preconfigure_number_of_master_periods(self.n_master_periods)
            analyze_result = cal.analyze_raw_data(master_raw, nonius_raw)
            analyze_result.optimized_nonius_track_offset_table()  # populate curve buffers
            self.last_analyze_result = analyze_result

        # Get final analog adjustments and InRange values for the result.
        final_master = cal.analog_master_track_adjustments()
        final_nonius = cal.analog_nonius_track_adjustments()
        nonius_in_range = self.get_nonius_in_range(analyze_result)

        return CalibrationResult(
            success=True,
            iterations=self.iteration_count,
            master_adjustments=final_master,
            nonius_adjustments=final_nonius,
            spo_base=table_params.spo_base,
            spo_n=spo_n,
            nonius_in_range_max=nonius_in_range.in_range_max,
            nonius_in_range_min=nonius_in_range.in_range_min,
        )


class EncoderCalibrator:
    """Orchestrates calibration for one or more iC-MU encoders.

    Data acquisition uses a TPDO map registered alongside the FSoE
    safety PDO maps.  Both share the same PDO exchange thread, giving
    deterministic sampling at the PDO cycle rate.

    Args:
        mc: Connected MotionController instance.
        axis: Drive axis number.
        max_iterations: Maximum analog calibration iterations.
        gen_frequency: Saw-tooth generator frequency in Hz.
        gen_current: Quadrature current target in amps.
        pdo_rate: PDO cycle time in seconds.
        capture_duration: Data capture duration per iteration in seconds.
        output_dir: Directory for diagnostic plot PNGs.
        interactive_plots: If True, show plots interactively instead of saving.
        save_raw_plots: If True, save raw waveform plots for each iteration.
        save_residual_bar_plots: If True, save residual bar plots for each iteration.
        save_trend_plot: If True, save residuals trend plot (one per encoder).
        save_json: If True, save iteration logs as JSON files.
        force_in_range: If not None, treat Nonius In Range > force_in_range(%)
            as a calibration failure.
    """

    def __init__(
        self,
        mc: MotionController,
        *,
        axis: int = 1,
        max_iterations: int = 10,
        gen_frequency: float = DEFAULT_GEN_FREQ,
        gen_current: float = DEFAULT_GEN_CURRENT,
        pdo_rate: float = DEFAULT_PDO_RATE_S,
        capture_duration: float = DEFAULT_CAPTURE_DURATION_S,
        output_dir: Optional[Path] = None,
        interactive_plots: bool = False,
        save_raw_plots: bool = False,
        save_residual_bar_plots: bool = False,
        save_trend_plot: bool = True,
        save_json: bool = True,
        force_in_range: Optional[float] = None,
    ) -> None:
        self._mc = mc
        self._axis = axis
        self._max_iterations = max_iterations
        self._pdo_rate = pdo_rate
        self._capture_duration = capture_duration
        self._motor = MotorControl(
            mc, axis=axis, gen_frequency=gen_frequency, gen_current=gen_current
        )
        self._encoders: list[Encoder] = []
        self._output_dir = output_dir or Path("calibration_output")
        self._interactive_plots = interactive_plots
        self._save_raw_plots = save_raw_plots
        self._save_residual_bar_plots = save_residual_bar_plots
        self._save_trend_plot = save_trend_plot
        self._save_json = save_json
        # PDO state
        self._tpdo_map: Optional[TPDOMap] = None
        self._padding_rpdo: Optional[RPDOMap] = None
        self._pdo_buffer: deque[list[int]] = deque()
        self._pdo_lock = threading.Lock()
        self._pdo_collecting = False
        # InRange
        self._force_in_range = force_in_range

    # -- Encoder management --
    def add_encoder(self, sensor_type: SensorType, sensor_config: EncoderRegisterConfig) -> Encoder:
        """Create and register an Encoder for the given sensor type and configuration.

        The encoder channel number is derived from the sensor type.

        Args:
            sensor_type: Drive feedback sensor type for this encoder.
            sensor_config: Encoder register configuration for this encoder.

        Returns:
            The newly created Encoder instance.

        Raises:
            ValueError: If the sensor type has no valid config.

        """
        # Add the encoder to the calibrator's list and return it
        enc = Encoder(self._mc, sensor_type, axis=self._axis, config=sensor_config)
        self._encoders.append(enc)
        logger.info(f"Registered encoder {enc.number} for calibration.")
        return enc

    @property
    def encoders(self) -> list[Encoder]:
        """List of enrolled encoders.

        Returns:
            List of Encoder instances registered for calibration.

        """
        return list(self._encoders)

    # -- Motor control --

    def configure_drive_encoders(self) -> None:
        """Configure the drive for internal generator mode with enrolled encoders."""
        sensor_types = [enc.sensor_type for enc in self._encoders]
        self._motor.configure_encoders(sensor_types)

    # -- PDO data acquisition --

    def _setup_data_tpdo(self) -> None:
        """Create a TPDO map with encoder pos_value registers.

        Uses ``mc.capture.pdo.create_pdo_item()`` to build typed
        ``TPDOMapItem`` instances from the drive dictionary UIDs, then
        registers the map on the servo via
        ``mc.capture.pdo.set_pdo_maps_to_slave()``.

        For non-FSoE drives a 1-byte padding RPDO is included so that
        the slave has at least one RPDO mapping—required by the
        EtherCAT state machine to reach SafeOp.  When FSoE is present
        its handler provides its own RPDOs, so no padding is needed.
        """
        tpdo_map = TPDOMap()
        for enc in self._encoders:
            item = self._mc.capture.pdo.create_pdo_item(
                register_uid=enc.regs.pos_value,
                axis=self._axis,
            )
            tpdo_map.add_item(item)

        # Non-FSoE drives need at least one RPDO for SafeOp transition.
        rpdo_maps: list[RPDOMap] = []
        if not self._motor.has_fsoe:
            padding = RPDOMap()
            item = RPDOMapItem(size_bits=8)
            item.raw_data_bytes = int.to_bytes(0, 1, "little")
            padding.add_item(item)
            rpdo_maps.append(padding)
            self._padding_rpdo = padding
        else:
            self._padding_rpdo = None

        self._mc.capture.pdo.set_pdo_maps_to_slave(
            rpdo_maps=rpdo_maps,
            tpdo_maps=[tpdo_map],
        )
        tpdo_map.subscribe_to_process_data_event(self._on_pdo_data)
        self._tpdo_map = tpdo_map

        logger.info(f"Data TPDO configured ({len(self._encoders)} encoder registers mapped).")

    def _teardown_data_tpdo(self) -> None:
        """Unsubscribe and remove the data TPDO/RPDO maps from the servo."""
        if self._tpdo_map is None:
            return
        self._tpdo_map.unsubscribe_to_process_data_event()
        try:
            self._mc.capture.pdo.remove_tpdo_map(tpdo_map=self._tpdo_map)
        except Exception:
            logger.debug("Could not remove TPDO map (slave may be offline).")
        self._tpdo_map = None
        if self._padding_rpdo is not None:
            try:
                self._mc.capture.pdo.remove_rpdo_map(rpdo_map=self._padding_rpdo)
            except Exception:
                logger.debug("Could not remove padding RPDO map (slave may be offline).")
            self._padding_rpdo = None

    def _on_pdo_data(self) -> None:
        """TPDO process-data callback (runs in PDO exchange thread)."""
        if not self._pdo_collecting or self._tpdo_map is None:
            return
        values = [int(item.value) for item in self._tpdo_map.items]
        with self._pdo_lock:
            self._pdo_buffer.append(values)

    def _acquire_raw_data(self) -> dict[int, list[int]]:
        """Capture raw register data from all enrolled encoders.

        Enables PDO sample collection for the configured capture
        duration, then drains the buffer and transposes the row-major
        PDO data into per-encoder columns of packed register values.

        The PDO exchange must already be running.

        Returns:
            Mapping of encoder number to list of packed register values.

        Raises:
            RuntimeError: If the PDO exchange thread has raised an exception.

        """
        # Start collecting
        with self._pdo_lock:
            self._pdo_buffer.clear()
        self._pdo_collecting = True

        try:
            elapsed_time = 0.0
            pdo_exception_interval = min(DEFAULT_PDO_EXCEPTION_INTERVAL_S, self._capture_duration)
            while elapsed_time < self._capture_duration:
                if self._motor.pdo_exception is not None:
                    raise RuntimeError(f"PDO exchange died: {self._motor.pdo_exception}")
                time.sleep(pdo_exception_interval)
                elapsed_time += pdo_exception_interval

        finally:
            # Stop collecting
            self._pdo_collecting = False

        # Drain and clear buffer
        with self._pdo_lock:
            samples = list(self._pdo_buffer)
            self._pdo_buffer.clear()

        # Transpose row-major PDO data into per-encoder columns.
        result: dict[int, list[int]] = {}
        for idx, enc in enumerate(self._encoders):
            values = [row[idx] for row in samples if idx < len(row)]
            result[enc.number] = values
            logger.info(f"Encoder {enc.number}: captured {len(values)} raw samples.")

        return result

    # -- Calibration orchestration --

    def calibrate(self) -> dict[int, CalibrationResult]:
        """Run the full calibration procedure for all enrolled encoders.

        Lifecycle:

        1. Save encoder state, enter calibration mode, reset analog.
        2. Register data TPDO (must precede FSoE for correct PDO index order).
        3. Prepare FSoE (if applicable).
        4. Activate PDOs (FSoE + data start together).
        5. Start motor, run iterative calibration loop.
        6. Finalize converged encoders (nonius + EEPROM).
        7. Stop motor, stop PDOs/FSoE, restore encoder state.

        Returns:
            Mapping of encoder number to CalibrationResult.

        Raises:
            RuntimeError: If no encoders have been registered.
        """
        if not self._encoders:
            msg = "No encoders registered. Call add_encoder() first."
            raise RuntimeError(msg)

        encoders = [_SingleEncoderCalibration(enc) for enc in self._encoders]

        try:
            # -- Setup phase 1: save state --
            for enc in encoders:
                enc.save_state()

            # -- Setup phase 2: calibration mode --
            for enc in encoders:
                enc.enter_calibration_mode()

            # -- Setup phase 3: reset analog --
            for enc in encoders:
                enc.reset_analog()

            # -- Clean output directory --
            if self._output_dir.exists():
                shutil.rmtree(self._output_dir)
            self._output_dir.mkdir(parents=True, exist_ok=True)

            # -- Setup phase 4: data TPDO (register on servo) --
            # Must be registered BEFORE FSoE maps so that the TPDO dict
            # insertion order (0x1A00, then 0x1B00) matches the sorted
            # index order used by _process_tpdo() when parsing the
            # process data buffer.
            # https://novantamotion.atlassian.net/browse/INGK-1257
            warm_matplotlib_cache(interactive=self._interactive_plots)
            self._setup_data_tpdo()

            # -- Setup phase 5: FSoE (maps only, no PDO start) --
            if self._motor.has_fsoe:
                self._motor.prepare_fsoe()

            # -- Setup phase 6: activate all PDOs together --
            self._motor.activate_pdos(refresh_rate=self._pdo_rate)

            try:
                # -- Motor runs for the entire calibration session --
                with self._motor.motor_spinning():
                    # -- Iterative analog calibration --
                    for iteration in range(1, self._max_iterations + 1):
                        pending = [enc for enc in encoders if enc.pending]
                        if not pending:
                            break

                        logger.info(f"--- Iteration {iteration} ---")

                        raw_data = self._acquire_raw_data()

                        for enc in pending:
                            enc.process_iteration(
                                iteration,
                                raw_data[enc.number],
                                self._output_dir,
                                self._interactive_plots,
                                save_raw_plots=self._save_raw_plots,
                                save_residual_bar_plots=self._save_residual_bar_plots,
                                save_trend_plot=self._save_trend_plot,
                                force_in_range_threshold=self._force_in_range,
                            )

                    # -- Finalize converged encoders --
                    results: dict[int, CalibrationResult] = {}
                    for enc in encoders:
                        if enc.converged:
                            # Then in the loop:
                            results[enc.number] = enc.finalize()

                        else:
                            logger.warning(
                                f"Encoder {enc.number}: did NOT converge after"
                                f" {self._max_iterations} iterations."
                                f" Skipping EEPROM save.",
                            )
                            if enc.last_analyze_result is None:
                                in_range_max = in_range_min = 0.0
                            else:
                                in_range = enc.get_nonius_in_range(enc.last_analyze_result)
                                in_range_max = in_range.in_range_max
                                in_range_min = in_range.in_range_min
                            results[enc.number] = CalibrationResult(
                                success=False,
                                iterations=enc.iteration_count,
                                nonius_in_range_max=in_range_max,
                                nonius_in_range_min=in_range_min,
                            )

                        last_analyze_result = enc.last_analyze_result
                        # Plot nonius track offset table for the last iteration (even if not converged)
                        if last_analyze_result is not None:
                            phase_error = last_analyze_result.nonius_phase_errors()
                            track_offset_curve = last_analyze_result.nonius_track_offset_curve()
                            phase_margin = last_analyze_result.nonius_phase_margin()
                            single_turn_position = last_analyze_result.nonius_position(
                                mu_3sl.Unit.DEGREE, False
                            )
                            continuous_single_turn_position = last_analyze_result.nonius_position(
                                mu_3sl.Unit.DEGREE, True
                            )
                            # TODO: Optimize this
                            in_range = enc.get_nonius_in_range(last_analyze_result)
                            _plot_nonius_track_offset_table(
                                encoder=enc.number,
                                phase_error=phase_error,
                                track_offset_curve=track_offset_curve,
                                phase_margin=phase_margin,
                                single_turn_position=single_turn_position,
                                continuous_single_turn_position=continuous_single_turn_position,
                                nonius_phase_range_limit=in_range.range_limit,
                                nonius_phase_margin_max=in_range.margin_max,
                                nonius_phase_margin_min=in_range.margin_min,
                                output_dir=self._output_dir,
                            )

                return results

            finally:
                if self._save_json:
                    for enc in encoders:
                        enc.export_data(self._output_dir)
                # Stop PDOs first (returns slave to pre-op), then remove maps.
                self._motor.stop_pdos_and_fsoe()
                self._teardown_data_tpdo()

        finally:
            for enc in encoders:
                enc.restore_state()

    def calculate_in_range(self) -> None:
        """Calculate the Nonius InRange value for all enrolled encoders.

        Gathers a single batch of raw data and analyzes it to compute the Nonius InRange value
        for each encoder, but does not apply any analog adjustments or save to EEPROM,
        allowing for evaluation of the current encoder configuration
        without performing full calibration.


        Raises:
            RuntimeError: If no encoders have been registered.
        """
        if not self._encoders:
            msg = "No encoders registered. Call add_encoder() first."
            raise RuntimeError(msg)

        encoders = [_SingleEncoderCalibration(enc) for enc in self._encoders]

        try:
            # -- Setup phase 1: save state --
            for enc in encoders:
                enc.save_state()

            # -- Setup phase 2: calibration mode --
            for enc in encoders:
                enc.enter_calibration_mode()

            # -- Clean output directory --
            if self._output_dir.exists():
                shutil.rmtree(self._output_dir)
            self._output_dir.mkdir(parents=True, exist_ok=True)

            # -- Setup phase 4: data TPDO (register on servo) --
            # Must be registered BEFORE FSoE maps so that the TPDO dict
            # insertion order (0x1A00, then 0x1B00) matches the sorted
            # index order used by _process_tpdo() when parsing the
            # process data buffer.
            # https://novantamotion.atlassian.net/browse/INGK-1257
            warm_matplotlib_cache(interactive=self._interactive_plots)
            self._setup_data_tpdo()

            # -- Setup phase 5: FSoE (maps only, no PDO start) --
            if self._motor.has_fsoe:
                self._motor.prepare_fsoe()

            # -- Setup phase 6: activate all PDOs together --
            self._motor.activate_pdos(refresh_rate=self._pdo_rate)

            try:
                # -- Motor runs for the entire calibration session --
                with self._motor.motor_spinning():
                    raw_data = self._acquire_raw_data()
                    if not raw_data:
                        logger.warning(
                            "0 samples captured.",
                        )
                        raise RuntimeError("No samples captured. PDO exchange may have died.")

                    # -- Finalize converged encoders --
                    for enc in encoders:
                        samples = raw_data.get(enc.number, [])
                        if not samples:
                            logger.warning(f"Encoder {enc.number}: 0 samples captured.")
                            raise RuntimeError("No samples captured. PDO exchange may have died.")

                        # Unpack packed register values into master / nonius tracks.
                        master_raw: list[int] = []
                        nonius_raw: list[int] = []
                        for val in samples:
                            m, n = split_raw_payload(val)
                            master_raw.append(m)
                            nonius_raw.append(n)

                        # B1 fix: sync DLL with current chip state
                        master_adj, nonius_adj = enc.enc.read_analog_adjustments()
                        enc.cal.set_current_analog_track_adjustments(master_adj, nonius_adj)

                        # Reinforce MPC hint so the library doesn't re-detect a wrong count.
                        enc.cal.preconfigure_number_of_master_periods(enc.n_master_periods)

                        analyze_result = enc.cal.analyze_raw_data(master_raw, nonius_raw)
                        in_range = enc.get_nonius_in_range(analyze_result)
                        logger.info(
                            f"Encoder {enc.number} nonius in range: "
                            f"Max={in_range.in_range_max:.1f}%, Min={in_range.in_range_min:.1f}%, "
                            f"Phase margin max={in_range.margin_max}, "
                            f"Phase margin min={in_range.margin_min}, "
                            f"Phase range limit={in_range.range_limit}",
                        )
                        enc.iteration_log.append({
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
                            "nonius phase margin": {
                                "InRange max %": in_range.in_range_max,
                                "InRange min %": in_range.in_range_min,
                                "phase margin max": in_range.margin_max,
                                "phase margin min": in_range.margin_min,
                                "phase range limit": in_range.range_limit,
                            },
                        })

            finally:
                if self._save_json:
                    for enc in encoders:
                        enc.export_data(self._output_dir)
                # Stop PDOs first (returns slave to pre-op), then remove maps.
                self._motor.stop_pdos_and_fsoe()
                self._teardown_data_tpdo()

        finally:
            for enc in encoders:
                enc.restore_state()
