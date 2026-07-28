import mu_3sl_interface as mu_3sl
import pytest
from ingeniamotion.enums import SensorType

from ic_haus_magnetic_encoder_calibration.calibrator import (
    EncoderCalibrator,
    _SingleEncoderCalibration,
)
from ic_haus_magnetic_encoder_calibration.encoder import split_raw_payload


class TestSplitRawPayload:
    def test_extracts_master_and_nonius(self) -> None:
        # master = 14 bits low, nonius = 14 bits high
        master = 0x1234 & 0x3FFF
        nonius = 0x0ABC & 0x3FFF
        packed = master | (nonius << 14)
        m, n = split_raw_payload(packed)
        assert m == master
        assert n == nonius

    def test_max_values(self) -> None:
        packed = 0x3FFF | (0x3FFF << 14)
        m, n = split_raw_payload(packed)
        assert m == 0x3FFF
        assert n == 0x3FFF


@pytest.fixture
def calibrator(mock_mc, tmp_path):
    return EncoderCalibrator(mock_mc, axis=1, max_iterations=3, output_dir=tmp_path)


class TestCalibrateNoEncoders:
    def test_raises_if_no_encoders(self, calibrator) -> None:
        with pytest.raises(RuntimeError, match="No encoders registered"):
            calibrator.calibrate()


class TestIsConverged:
    def test_all_below_threshold(self, mocker) -> None:
        result = mocker.MagicMock()
        master_rel = mocker.MagicMock(
            cosine_gain_lsb=0.5,
            sine_offset_lsb=0.3,
            cosine_offset_lsb=0.1,
            phase_lsb=0.9,
        )
        nonius_rel = mocker.MagicMock(
            cosine_gain_lsb=0.2,
            sine_offset_lsb=0.4,
            cosine_offset_lsb=0.6,
            phase_lsb=1.0,
        )
        result.relative_master_track_adjustments.return_value = master_rel
        result.relative_nonius_track_adjustments.return_value = nonius_rel

        assert _SingleEncoderCalibration.is_converged(result) is True

    def test_one_above_threshold(self, mocker) -> None:
        result = mocker.MagicMock()
        master_rel = mocker.MagicMock(
            cosine_gain_lsb=0.5,
            sine_offset_lsb=0.3,
            cosine_offset_lsb=0.1,
            phase_lsb=0.9,
        )
        nonius_rel = mocker.MagicMock(
            cosine_gain_lsb=0.2,
            sine_offset_lsb=0.4,
            cosine_offset_lsb=0.6,
            phase_lsb=1.1,  # exceeds threshold
        )
        result.relative_master_track_adjustments.return_value = master_rel
        result.relative_nonius_track_adjustments.return_value = nonius_rel

        assert _SingleEncoderCalibration.is_converged(result) is False


# ---------------------------------------------------------------------------
#  Helpers for calibrate() tests
# ---------------------------------------------------------------------------


def _make_converged_analyze_result(mocker):
    """Create a mock AnalyzeResult where all residuals are within threshold.

    Returns:
        A mock AnalyzeResult with all residuals below 1.0.
    """
    result = mocker.MagicMock()
    master_rel = mocker.MagicMock(
        cosine_gain_lsb=0.5,
        sine_offset_lsb=0.3,
        cosine_offset_lsb=0.1,
        phase_lsb=0.2,
    )
    nonius_rel = mocker.MagicMock(
        cosine_gain_lsb=0.2,
        sine_offset_lsb=0.4,
        cosine_offset_lsb=0.6,
        phase_lsb=0.8,
    )
    result.relative_master_track_adjustments.return_value = master_rel
    result.relative_nonius_track_adjustments.return_value = nonius_rel
    result.minimal_number_of_samples_per_master_period.return_value = 10.0
    result.is_analog_analyses_valid.return_value = True
    result.number_of_calculated_master_periods.return_value = 32
    result.number_of_revolutions.return_value = 1.0
    result.number_of_acquired_master_periods.return_value = 32.0
    result.average_number_of_samples_per_master_period.return_value = 237.5
    return result


def _make_not_converged_analyze_result(mocker):
    """Create a mock AnalyzeResult where at least one residual exceeds threshold.

    Returns:
        A mock AnalyzeResult with master track residuals above 1.0.
    """
    result = mocker.MagicMock()
    master_rel = mocker.MagicMock(
        cosine_gain_lsb=3.0,
        sine_offset_lsb=2.0,
        cosine_offset_lsb=1.5,
        phase_lsb=0.9,
    )
    nonius_rel = mocker.MagicMock(
        cosine_gain_lsb=0.2,
        sine_offset_lsb=0.4,
        cosine_offset_lsb=0.6,
        phase_lsb=0.8,
    )
    result.relative_master_track_adjustments.return_value = master_rel
    result.relative_nonius_track_adjustments.return_value = nonius_rel
    result.minimal_number_of_samples_per_master_period.return_value = 10.0
    result.is_analog_analyses_valid.return_value = False
    result.number_of_calculated_master_periods.return_value = 28
    result.number_of_revolutions.return_value = 0.8
    result.number_of_acquired_master_periods.return_value = 28.0
    result.average_number_of_samples_per_master_period.return_value = 200.0
    return result


def _patch_encoder(enc, mocker):
    """Mock all Encoder methods used by calibrate() to avoid real BiSS calls."""
    mocker.patch.object(enc, "read_revision", return_value=mocker.MagicMock(name="REV"))
    mocker.patch.object(enc, "get_drive_config", return_value=mocker.MagicMock(name="DriveConfig"))
    mocker.patch.object(enc, "get_ic_config", return_value=mocker.MagicMock(name="ICConfig"))
    mocker.patch.object(enc, "configure_in_calibration_mode", return_value=32)
    mocker.patch.object(enc, "reset_analog_to_factory_defaults")
    mocker.patch.object(
        enc,
        "read_analog_adjustments",
        return_value=(mocker.MagicMock(name="master_adj"), mocker.MagicMock(name="nonius_adj")),
    )
    mocker.patch.object(enc, "write_analog_adjustments")
    mocker.patch.object(enc, "write_nonius_parameters")
    mocker.patch.object(enc, "set_drive_config")
    mocker.patch.object(enc, "set_ic_config")
    mocker.patch.object(enc, "enable_all_errors")
    mocker.patch.object(enc, "save_to_eeprom")
    mocker.patch.object(enc, "abs_reset")


def _setup_converging_calibration(cal, mocker, mu_mock, analyze_results):
    """Wire up a mu_3sl.Calibration mock and _acquire_raw_data for one encoder.

    Also mocks the PDO/motor lifecycle methods so calibrate() can run
    without real hardware.

    Args:
        analyze_results: List of mock AnalyzeResult objects returned by
            cal_obj.analyze_raw_data on successive calls.

    Returns:
        The mock Calibration object.
    """
    cal_obj = mocker.MagicMock(name="Calibration")
    mu_mock.Calibration.return_value = cal_obj
    cal_obj.analyze_raw_data.side_effect = list(analyze_results)

    mu_mock.nonius_track_offset_table_parameters.return_value = mocker.MagicMock(
        spo_base=0,
        spo_n=list(range(15)),
    )
    mocker.patch.object(cal, "_acquire_raw_data", return_value={1: [1 | (2 << 14)]})
    # Mock PDO/motor lifecycle
    mocker.patch.object(
        type(cal._motor),
        "has_fsoe",
        new_callable=mocker.PropertyMock,
        return_value=False,
    )
    mocker.patch.object(cal, "_setup_data_tpdo")
    mocker.patch.object(cal, "_teardown_data_tpdo")
    mocker.patch.object(cal._motor, "activate_pdos")
    mocker.patch.object(cal._motor, "stop_pdos_and_fsoe")
    mocker.patch.object(cal._motor, "motor_spinning")
    return cal_obj


@pytest.fixture
def mu_3sl_mock(mocker):
    """Patch the mu_3sl module used by the calibrator.

    Returns:
        The mock mu_3sl module.
    """
    return mocker.patch("ic_haus_magnetic_encoder_calibration.calibrator.mu_3sl")


# ---------------------------------------------------------------------------
#  calibrate() orchestration tests
# ---------------------------------------------------------------------------


class TestCalibrateConvergence:
    """Core calibration flow: convergence detection, iteration, analog adjustment."""

    def test_converges_first_iteration(
        self, mock_mc, mocker, mu_3sl_mock, tmp_path, mock_encoder_config
    ) -> None:
        """Single encoder converges on first try -> success, EEPROM saved."""

        cal = EncoderCalibrator(mock_mc, axis=1, max_iterations=3, output_dir=tmp_path)
        enc = cal.add_encoder(SensorType.ABS1, mock_encoder_config)
        _patch_encoder(enc, mocker)

        conv = _make_converged_analyze_result(mocker)
        conv.optimized_nonius_track_offset_table.return_value = mocker.MagicMock()
        _setup_converging_calibration(cal, mocker, mu_3sl_mock, [conv, conv])

        results = cal.calibrate()

        assert results[1].success is True
        assert results[1].iterations == 1

    def test_converges_after_adjustment(
        self, mock_mc, mocker, mu_3sl_mock, tmp_path, mock_encoder_config
    ) -> None:
        """Not converged -> adjusts analog params -> converges on iteration 2."""
        cal = EncoderCalibrator(mock_mc, axis=1, max_iterations=3, output_dir=tmp_path)
        enc = cal.add_encoder(SensorType.ABS1, mock_encoder_config)
        _patch_encoder(enc, mocker)

        not_conv = _make_not_converged_analyze_result(mocker)
        conv = _make_converged_analyze_result(mocker)
        conv.optimized_nonius_track_offset_table.return_value = mocker.MagicMock()
        cal_obj = _setup_converging_calibration(
            cal,
            mocker,
            mu_3sl_mock,
            [not_conv, conv, conv],
        )

        results = cal.calibrate()

        assert results[1].success is True
        assert results[1].iterations == 2
        enc.write_analog_adjustments.assert_called_once()
        cal_obj.adjust_analog_by_analyze_result.assert_called_once()

    def test_non_convergence_loads_config(
        self, mock_mc, mocker, mu_3sl_mock, tmp_path, mock_encoder_config
    ) -> None:
        """If never converges, configuration is loaded onto EEPROM."""
        cal = EncoderCalibrator(mock_mc, axis=1, max_iterations=3, output_dir=tmp_path)
        enc = cal.add_encoder(SensorType.ABS1, mock_encoder_config)
        _patch_encoder(enc, mocker)

        not_conv = _make_not_converged_analyze_result(mocker)
        _setup_converging_calibration(
            cal,
            mocker,
            mu_3sl_mock,
            [not_conv, not_conv, not_conv],
        )

        results = cal.calibrate()

        assert results[1].success is False
        enc.save_to_eeprom.assert_called_once()

    def test_syncs_dll_state_before_analysis(
        self, mock_mc, mocker, mu_3sl_mock, tmp_path, mock_encoder_config
    ) -> None:
        """B1 fix: read_analog_adjustments -> set_current before analyze_raw_data."""
        cal = EncoderCalibrator(mock_mc, axis=1, max_iterations=3, output_dir=tmp_path)
        enc = cal.add_encoder(SensorType.ABS1, mock_encoder_config)
        _patch_encoder(enc, mocker)
        master_adj = mocker.MagicMock(name="m_adj")
        nonius_adj = mocker.MagicMock(name="n_adj")
        enc.read_analog_adjustments.return_value = (master_adj, nonius_adj)

        conv = _make_converged_analyze_result(mocker)
        conv.optimized_nonius_track_offset_table.return_value = mocker.MagicMock()
        cal_obj = _setup_converging_calibration(
            cal,
            mocker,
            mu_3sl_mock,
            [conv, conv],
        )

        cal.calibrate()

        cal_obj.set_current_analog_track_adjustments.assert_called_with(
            master_adj,
            nonius_adj,
        )


class TestCalibrateBothEncoders:
    """Two encoders: both converge or mixed results."""

    def test_both_converge(self, mock_mc, mocker, tmp_path, mock_encoder_config) -> None:
        """Both encoder 1 and encoder 2 converge, EEPROM saved for both."""
        mu_mock = mocker.patch("ic_haus_magnetic_encoder_calibration.calibrator.mu_3sl")
        cal = EncoderCalibrator(mock_mc, axis=1, max_iterations=3, output_dir=tmp_path)

        enc1 = cal.add_encoder(SensorType.ABS1, mock_encoder_config)
        enc2 = cal.add_encoder(SensorType.SSI2, mock_encoder_config)
        _patch_encoder(enc1, mocker)
        _patch_encoder(enc2, mocker)

        mocker.patch.object(
            type(cal._motor),
            "has_fsoe",
            new_callable=mocker.PropertyMock,
            return_value=False,
        )
        mocker.patch.object(cal, "_setup_data_tpdo")
        mocker.patch.object(cal, "_teardown_data_tpdo")
        mocker.patch.object(cal._motor, "activate_pdos")
        mocker.patch.object(cal._motor, "stop_pdos_and_fsoe")
        mocker.patch.object(cal._motor, "motor_spinning")

        cal_obj1 = mocker.MagicMock(name="Cal1")
        cal_obj2 = mocker.MagicMock(name="Cal2")
        mu_mock.Calibration.side_effect = [cal_obj1, cal_obj2]

        conv = _make_converged_analyze_result(mocker)
        conv.optimized_nonius_track_offset_table.return_value = mocker.MagicMock()

        cal_obj1.analyze_raw_data.side_effect = [conv, conv]
        cal_obj2.analyze_raw_data.side_effect = [conv, conv]
        mu_mock.nonius_track_offset_table_parameters.return_value = mocker.MagicMock(
            spo_base=0,
            spo_n=list(range(15)),
        )
        mocker.patch.object(
            cal,
            "_acquire_raw_data",
            return_value={1: [1 | (2 << 14)], 2: [3 | (4 << 14)]},
        )

        results = cal.calibrate()

        assert results[1].success is True
        assert results[2].success is True
        enc1.save_to_eeprom.assert_called_once()
        enc2.save_to_eeprom.assert_called_once()

    def test_mixed_results(self, mock_mc, mocker, tmp_path, mock_encoder_config) -> None:
        """Encoder 1 converges, encoder 2 does not."""
        mu_mock = mocker.patch("ic_haus_magnetic_encoder_calibration.calibrator.mu_3sl")
        cal = EncoderCalibrator(mock_mc, axis=1, max_iterations=3, output_dir=tmp_path)

        enc1 = cal.add_encoder(SensorType.ABS1, mock_encoder_config)
        enc2 = cal.add_encoder(SensorType.SSI2, mock_encoder_config)
        _patch_encoder(enc1, mocker)
        _patch_encoder(enc2, mocker)

        mocker.patch.object(
            type(cal._motor),
            "has_fsoe",
            new_callable=mocker.PropertyMock,
            return_value=False,
        )
        mocker.patch.object(cal, "_setup_data_tpdo")
        mocker.patch.object(cal, "_teardown_data_tpdo")
        mocker.patch.object(cal._motor, "activate_pdos")
        mocker.patch.object(cal._motor, "stop_pdos_and_fsoe")
        mocker.patch.object(cal._motor, "motor_spinning")

        cal_obj1 = mocker.MagicMock(name="Cal1")
        cal_obj2 = mocker.MagicMock(name="Cal2")
        mu_mock.Calibration.side_effect = [cal_obj1, cal_obj2]

        conv = _make_converged_analyze_result(mocker)
        conv.optimized_nonius_track_offset_table.return_value = mocker.MagicMock()
        not_conv = _make_not_converged_analyze_result(mocker)

        cal_obj1.analyze_raw_data.side_effect = [conv, conv]
        cal_obj2.analyze_raw_data.return_value = not_conv
        mu_mock.nonius_track_offset_table_parameters.return_value = mocker.MagicMock(
            spo_base=0,
            spo_n=list(range(15)),
        )
        mocker.patch.object(
            cal,
            "_acquire_raw_data",
            return_value={1: [1 | (2 << 14)], 2: [3 | (4 << 14)]},
        )

        results = cal.calibrate()

        assert results[1].success is True
        assert results[2].success is False


class TestCalibrateRestore:
    """Config is always restored: on success, non-convergence, and exception."""

    def test_applies_config_before_calibration(
        self, mock_mc, mocker, mu_3sl_mock, tmp_path, mock_encoder_config
    ) -> None:
        """apply_config() is called during save_state (before calibration loop)."""
        cal = EncoderCalibrator(mock_mc, axis=1, max_iterations=3, output_dir=tmp_path)
        enc = cal.add_encoder(SensorType.ABS1, mock_encoder_config)
        _patch_encoder(enc, mocker)
        mocker.patch.object(enc, "apply_config")

        conv = _make_converged_analyze_result(mocker)
        conv.optimized_nonius_track_offset_table.return_value = mocker.MagicMock()
        _setup_converging_calibration(cal, mocker, mu_3sl_mock, [conv, conv])

        cal.calibrate()

        enc.apply_config.assert_called_once()

    def test_restores_on_success(
        self, mock_mc, mocker, mu_3sl_mock, tmp_path, mock_encoder_config
    ) -> None:
        cal = EncoderCalibrator(mock_mc, axis=1, max_iterations=3, output_dir=tmp_path)
        enc = cal.add_encoder(SensorType.ABS1, mock_encoder_config)
        _patch_encoder(enc, mocker)
        saved_drive = mocker.MagicMock(name="SavedDrive")
        saved_ic = mocker.MagicMock(name="SavedIC")
        enc.get_drive_config.return_value = saved_drive
        enc.get_ic_config.return_value = saved_ic

        conv = _make_converged_analyze_result(mocker)
        conv.optimized_nonius_track_offset_table.return_value = mocker.MagicMock()
        _setup_converging_calibration(cal, mocker, mu_3sl_mock, [conv, conv])

        cal.calibrate()

        enc.set_drive_config.assert_called_with(saved_drive)
        enc.set_ic_config.assert_called_with(saved_ic)

    def test_restores_on_non_convergence(
        self, mock_mc, mocker, mu_3sl_mock, tmp_path, mock_encoder_config
    ) -> None:
        """Drive and iC-MU config are restored even when calibration does not converge."""
        cal = EncoderCalibrator(mock_mc, axis=1, max_iterations=3, output_dir=tmp_path)
        enc = cal.add_encoder(SensorType.ABS1, mock_encoder_config)
        _patch_encoder(enc, mocker)
        saved_drive = mocker.MagicMock(name="SavedDrive")
        saved_ic = mocker.MagicMock(name="SavedIC")
        enc.get_drive_config.return_value = saved_drive
        enc.get_ic_config.return_value = saved_ic

        not_conv = _make_not_converged_analyze_result(mocker)
        _setup_converging_calibration(cal, mocker, mu_3sl_mock, [not_conv, not_conv, not_conv])

        results = cal.calibrate()

        assert results[1].success is False
        enc.set_ic_config.assert_called_with(saved_ic)
        enc.set_drive_config.assert_called_with(saved_drive)

    def test_restores_on_exception(
        self, mock_mc, mocker, mu_3sl_mock, tmp_path, mock_encoder_config
    ) -> None:
        cal = EncoderCalibrator(mock_mc, axis=1, max_iterations=3, output_dir=tmp_path)
        enc = cal.add_encoder(SensorType.ABS1, mock_encoder_config)
        _patch_encoder(enc, mocker)
        saved_drive = mocker.MagicMock(name="SavedDrive")
        saved_ic = mocker.MagicMock(name="SavedIC")
        enc.get_drive_config.return_value = saved_drive
        enc.get_ic_config.return_value = saved_ic

        mu_3sl_mock.Calibration.return_value = mocker.MagicMock()
        mocker.patch.object(
            type(cal._motor),
            "has_fsoe",
            new_callable=mocker.PropertyMock,
            return_value=False,
        )
        mocker.patch.object(cal, "_setup_data_tpdo")
        mocker.patch.object(cal, "_teardown_data_tpdo")
        mocker.patch.object(cal._motor, "activate_pdos")
        mocker.patch.object(cal._motor, "stop_pdos_and_fsoe")
        mocker.patch.object(cal._motor, "motor_spinning")
        mocker.patch.object(
            cal,
            "_acquire_raw_data",
            side_effect=RuntimeError("connection lost"),
        )

        with pytest.raises(RuntimeError, match="connection lost"):
            cal.calibrate()

        enc.set_ic_config.assert_called_with(saved_ic)
        enc.set_drive_config.assert_called_with(saved_drive)


class TestAddEncoder:
    """Verify that encoders are added correctly."""

    def test_add_encoder_with_config(self, mock_mc, tmp_path, mock_encoder_config) -> None:
        """add_encoder() assigns the loaded config to the encoder instance."""
        cal = EncoderCalibrator(mock_mc, axis=1, max_iterations=3, output_dir=tmp_path)
        enc = cal.add_encoder(SensorType.ABS1, mock_encoder_config)

        # Check register configuration
        assert enc._config is not None
        assert enc._config.out_msb == 0x05
        assert enc._config.out_lsb == 0x00
        assert enc._config.mode_st == 0x00
        assert enc._config.enac == 0x01
        assert enc._config.cfgew == 0xFF
        assert enc._config.filt == 0x03


# ---------------------------------------------------------------------------
#  Hardware integration tests (require a physical drive + encoder)
# ---------------------------------------------------------------------------


@pytest.mark.hardware
class TestCalibrationHardware:
    def test_calibrate_both_encoders(self, mc) -> None:
        """Both encoders converge from zero gains."""
        cal = EncoderCalibrator(mc, axis=1)
        enc1 = cal.add_encoder(SensorType.ABS1)
        enc2 = cal.add_encoder(SensorType.SSI2)

        # Write non-sensical analog adjustments
        # to validate the calibration can recover from
        # catastrophic starting parameters.
        # (Resets to factory defaults before finding optimal adjustments.)
        zeros = mu_3sl.AnalogTrackAdjustments(0, 0, 0, 0)
        enc1.write_analog_adjustments(zeros, zeros)
        enc2.write_analog_adjustments(zeros, zeros)

        cal.configure_drive_encoders()

        results = cal.calibrate()

        assert results[1].success is True
        assert results[1].iterations >= 1
        assert results[2].success is True
        assert results[2].iterations >= 1
