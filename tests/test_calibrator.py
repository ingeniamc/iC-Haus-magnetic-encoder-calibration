import pytest

from ic_haus_magnetic_encoder_calibration.calibrator import (
    EncoderCalibrator,
    _is_converged,
    _split_raw_payload,
)


class TestSplitRawPayload:
    def test_extracts_master_and_nonius(self) -> None:
        # master = 14 bits low, nonius = 14 bits high
        master = 0x1234 & 0x3FFF
        nonius = 0x0ABC & 0x3FFF
        packed = master | (nonius << 14)
        m, n = _split_raw_payload(packed)
        assert m == master
        assert n == nonius

    def test_zero(self) -> None:
        m, n = _split_raw_payload(0)
        assert m == 0
        assert n == 0

    def test_max_values(self) -> None:
        packed = 0x3FFF | (0x3FFF << 14)
        m, n = _split_raw_payload(packed)
        assert m == 0x3FFF
        assert n == 0x3FFF


@pytest.fixture
def calibrator(mock_mc):
    return EncoderCalibrator(mock_mc, axis=1, max_iterations=3)


class TestAddEncoder:
    def test_adds_encoder(self, calibrator) -> None:
        enc = calibrator.add_encoder(1)
        assert enc.number == 1
        assert len(calibrator.encoders) == 1

    def test_adds_multiple(self, calibrator) -> None:
        calibrator.add_encoder(1)
        calibrator.add_encoder(2)
        assert len(calibrator.encoders) == 2

    def test_encoders_returns_copy(self, calibrator) -> None:
        calibrator.add_encoder(1)
        encoders = calibrator.encoders
        encoders.clear()
        assert len(calibrator.encoders) == 1


class TestMotorControl:
    def test_configure_velocity_mode(self, calibrator, mock_mc) -> None:
        calibrator.configure_velocity_mode("DIGENC1")
        mock_mc.communication.set_register.assert_any_call(
            "FBK_DIGENC1_DIGENC2_SELECT",
            "DIGENC1",
            axis=1,
        )
        mock_mc.motion.set_operation_mode.assert_called_once_with(19, axis=1)

    def test_configure_internal_generator(self, calibrator, mock_mc) -> None:
        calibrator.configure_internal_generator()
        mock_mc.motion.set_operation_mode.assert_called_once_with(0, axis=1)

    def test_stop_motor(self, calibrator, mock_mc) -> None:
        calibrator.stop_motor()
        mock_mc.motion.set_velocity.assert_called_once_with(0, axis=1)
        mock_mc.motion.motor_disable.assert_called_once_with(axis=1)


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

        assert _is_converged(result) is True

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

        assert _is_converged(result) is False
