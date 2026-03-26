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
from pathlib import Path

import mu_3sl_interface as mu_3sl
from ingenialink.pdo import RPDOMap, RPDOMapItem, TPDOMap
from ingeniamotion import MotionController
from ingeniamotion.enums import SensorType

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
    _plot_raw_waveforms,
    _plot_residuals_bar,
    _plot_residuals_trend,
)

logger = logging.getLogger(__name__)

# Default data acquisition parameters
DEFAULT_CAPTURE_DURATION_S = 30.0
DEFAULT_PDO_RATE_S = 0.001  # 1 ms


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
        self._cal: mu_3sl.Calibration | None = None
        self.n_master_periods: int = 0
        self.saved_drive_config: DriveFrameConfig | None = None
        self.saved_ic_config: ICMURegisterState | None = None
        self.converged: bool = False
        self.iteration_count: int = 0
        self.residual_history: list[list[float]] = []
        self.iteration_log: list[dict[str, object]] = []
        self.last_analyze_result: mu_3sl.AnalyzeResult | None = None

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
        """Phase 1: ensure normal mode, read revision, save configs."""
        self.enc.ensure_normal_mode()
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
    ) -> None:
        """Run one calibration iteration: analyze, log, plot, correct.

        Updates :attr:`iteration_count`, :attr:`residual_history`,
        :attr:`iteration_log`, and potentially sets :attr:`converged`.
        """
        self.iteration_count = iteration

        if not raw_data:
            logger.warning(
                f"Encoder {self.number}: 0 samples captured at iteration"
                f" {iteration} — PDO exchange may have died. Skipping.",
            )
            return

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
        if self.is_converged(analyze_result):
            self.converged = True
            self.last_analyze_result = analyze_result
            logger.info(f"Encoder {self.number}: converged at iteration {iteration}.")
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
            if self.saved_ic_config is not None:
                self.enc.set_ic_config(self.saved_ic_config)
            self.enc.enable_all_errors()
            if self.saved_drive_config is not None:
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

    def finalize(self) -> CalibrationResult:
        """Optimize nonius, save to EEPROM, and build the result.

        Must only be called after convergence (:attr:`converged` is True).

        Returns:
            CalibrationResult for this encoder.
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
        enc.write_nonius_parameters(table_params)

        # Restore iC-MU config registers (exits raw mode) before EEPROM save.
        assert self.saved_ic_config is not None
        enc.set_ic_config(self.saved_ic_config)

        # Enable all error sources so real faults are visible.
        enc.enable_all_errors()

        # Save to EEPROM (raises on failure)
        enc.save_to_eeprom()

        # Perform an internal reset of the encoder
        enc.abs_reset()

        final_master = cal.analog_master_track_adjustments()
        final_nonius = cal.analog_nonius_track_adjustments()
        spo_n = [table_params.spo_n[i] for i in range(15)]

        return CalibrationResult(
            success=True,
            iterations=self.iteration_count,
            master_adjustments=final_master,
            nonius_adjustments=final_nonius,
            spo_base=table_params.spo_base,
            spo_n=spo_n,
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
        output_dir: Path | None = None,
        interactive_plots: bool = False,
        save_raw_plots: bool = False,
        save_residual_bar_plots: bool = False,
        save_trend_plot: bool = True,
        save_json: bool = True,
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
        self._tpdo_map: TPDOMap | None = None
        self._padding_rpdo: RPDOMap | None = None
        self._pdo_buffer: deque[list[int]] = deque()
        self._pdo_lock = threading.Lock()
        self._pdo_collecting = False

    # -- Encoder management --

    def add_encoder(self, sensor_type: SensorType) -> Encoder:
        """Create and register an Encoder for the given sensor type.

        The encoder channel number is derived from the sensor type.

        Args:
            sensor_type: Drive feedback sensor type for this encoder.

        Returns:
            The newly created Encoder instance.
        """
        enc = Encoder(self._mc, sensor_type, axis=self._axis)
        self._encoders.append(enc)
        logger.info(f"Registered encoder {enc.number} for calibration.")
        return enc

    @property
    def encoders(self) -> list[Encoder]:
        """Return the list of enrolled encoders."""
        return list(self._encoders)

    # -- Motor control --

    def configure_encoders(self) -> None:
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
        """
        # Start collecting
        with self._pdo_lock:
            self._pdo_buffer.clear()
        self._pdo_collecting = True

        time.sleep(self._capture_duration)

        # Stop collecting and drain buffer
        self._pdo_collecting = False
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
                            )

                    # -- Finalize converged encoders --
                    results: dict[int, CalibrationResult] = {}
                    for enc in encoders:
                        if enc.converged:
                            results[enc.number] = enc.finalize()
                        else:
                            logger.warning(
                                f"Encoder {enc.number}: did NOT converge after"
                                f" {self._max_iterations} iterations."
                                f" Skipping EEPROM save.",
                            )
                            results[enc.number] = CalibrationResult(
                                success=False,
                                iterations=enc.iteration_count,
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
