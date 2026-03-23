import time

import pytest

from ic_haus_magnetic_encoder_calibration.encoder import Encoder
from ic_haus_magnetic_encoder_calibration.ic_haus_registers import CFGEW
from ic_haus_magnetic_encoder_calibration.motor_control import MotorControl


@pytest.fixture
def motor(mock_mc):
    return MotorControl(mock_mc, axis=1)


class TestHasFsoe:
    def test_true_when_dict_safe(self, motor, mock_mc) -> None:
        mock_mc.servos = {"default": mock_mc}
        mock_mc.dictionary.is_safe = True
        assert motor.has_fsoe is True

    def test_false_when_dict_not_safe(self, motor, mock_mc) -> None:
        mock_mc.servos = {"default": mock_mc}
        mock_mc.dictionary.is_safe = False
        assert motor.has_fsoe is False

    def test_false_when_no_servo(self, motor, mock_mc) -> None:
        mock_mc.servos = {}
        assert motor.has_fsoe is False


class TestStartMotor:
    def test_enables_motor_and_starts_generator(self, motor, mock_mc) -> None:
        mock_mc.servos = {"default": mock_mc}
        mock_mc.dictionary.is_safe = False

        motor._start_motor()

        mock_mc.motion.motor_enable.assert_called_once_with(axis=1)
        mock_mc.motion.internal_generator_saw_tooth_move.assert_called_once()
        # Current ramp: 10 steps of set_current_quadrature
        assert mock_mc.motion.set_current_quadrature.call_count == 10

    def test_current_ramp_before_generator(self, motor, mock_mc) -> None:
        """Current must be ramped before the saw-tooth starts."""
        mock_mc.servos = {"default": mock_mc}
        mock_mc.dictionary.is_safe = False
        call_log: list[str] = []
        mock_mc.motion.set_current_quadrature.side_effect = lambda *_a, **_k: call_log.append(
            "current"
        )
        mock_mc.motion.internal_generator_saw_tooth_move.side_effect = lambda *_a, **_k: (
            call_log.append("saw_tooth")
        )

        motor._start_motor()

        saw_idx = call_log.index("saw_tooth")
        assert call_log.count("current") == 10
        assert all(c == "current" for c in call_log[:saw_idx])


class TestStartFsoe:
    def test_raises_when_no_fsoe(self, motor, mock_mc) -> None:
        mock_mc.servos = {"default": mock_mc}
        mock_mc.dictionary.is_safe = False

        with pytest.raises(RuntimeError, match="FSoE is not available"):
            motor._start_fsoe()

    def test_configures_fsoe_master(self, motor, mock_mc) -> None:
        mock_mc.servos = {"default": mock_mc}
        mock_mc.dictionary.is_safe = True

        motor._start_fsoe()

        handler = mock_mc.fsoe.create_fsoe_master_handler.return_value
        mock_mc.fsoe.create_fsoe_master_handler.assert_called_once_with(use_sra=True)
        handler.configure_pdo_maps.assert_called_once()
        handler.set_pdo_maps_to_slave.assert_called_once()
        handler.write_safe_parameters.assert_called_once()
        mock_mc.fsoe.start_master.assert_called_once_with(start_pdos=False)
        mock_mc.capture.pdo.start_pdos.assert_called_once()
        mock_mc.fsoe.wait_for_state_data.assert_called_once_with(timeout=15)
        handler.sto_deactivate.assert_called_once()
        handler.is_sto_active.assert_called_once()

    def test_sout_enabled_for_brake_release(self, motor, mock_mc) -> None:
        """SOut is enabled so the safety master controls brake release."""
        mock_mc.servos = {"default": mock_mc}
        mock_mc.dictionary.is_safe = True

        motor._start_fsoe()

        handler = mock_mc.fsoe.create_fsoe_master_handler.return_value
        sout_func = handler.sout_function.return_value
        handler.sout_function.assert_called()
        sout_func.command.set.assert_called_with(True)
        sout_func.sout_disable.set.assert_called_with(0)


class TestStopFsoe:
    def test_noop_when_not_active(self, motor, mock_mc) -> None:
        motor._stop_fsoe()
        mock_mc.fsoe.stop_master.assert_not_called()

    def test_stops_when_active(self, motor, mock_mc) -> None:
        mock_mc.servos = {"default": mock_mc}
        mock_mc.dictionary.is_safe = True
        motor._start_fsoe()

        motor._stop_fsoe()

        mock_mc.fsoe.stop_master.assert_called_once_with(stop_pdos=True)


class TestAutoFsoeOnStartMotor:
    def test_starts_fsoe_automatically(self, motor, mock_mc) -> None:
        mock_mc.servos = {"default": mock_mc}
        mock_mc.dictionary.is_safe = True

        motor._start_motor()

        mock_mc.fsoe.create_fsoe_master_handler.assert_called_once()

    def test_skips_fsoe_when_dict_not_safe(self, motor, mock_mc) -> None:
        mock_mc.servos = {"default": mock_mc}
        mock_mc.dictionary.is_safe = False

        motor._start_motor()

        mock_mc.fsoe.create_fsoe_master_handler.assert_not_called()


class TestRunningContextManager:
    def test_starts_and_stops_motor(self, motor, mock_mc) -> None:
        mock_mc.servos = {"default": mock_mc}
        mock_mc.dictionary.is_safe = False

        with motor.running():
            mock_mc.motion.motor_enable.assert_called_once()

        mock_mc.motion.motor_disable.assert_called_once()

    def test_stops_on_exception(self, motor, mock_mc) -> None:
        mock_mc.servos = {"default": mock_mc}
        mock_mc.dictionary.is_safe = False

        with pytest.raises(ValueError, match="test error"), motor.running():
            raise ValueError("test error")

        mock_mc.motion.motor_disable.assert_called_once()


# ---------------------------------------------------------------------------
#  Hardware integration tests (require a physical drive)
# ---------------------------------------------------------------------------


@pytest.fixture
def hw_motor(mc):
    """Create a MotorControl using the real MotionController.

    Returns:
        A MotorControl configured for axis 1.
    """
    return MotorControl(mc, axis=1)


@pytest.fixture
def hw_encoder(mc):
    """Prepare encoder for hardware motor tests.

    Ensures the encoder is in normal ABS mode (not leftover RAW from a
    crash) and suppresses ERR/WRN flags that an uncalibrated encoder
    asserts, which would otherwise cause the drive to fault with 0x7380.

    Returns:
        An Encoder with normal mode restored and error flags suppressed.
    """
    enc = Encoder(mc, encoder_number=1, axis=1)
    enc.ensure_normal_mode()
    enc._write_ic(CFGEW, 0xFF)
    return enc


@pytest.mark.hardware
class TestHasFsoeHardware:
    def test_detects_fsoe_from_dictionary(self, hw_motor) -> None:
        result = hw_motor.has_fsoe
        assert isinstance(result, bool)


@pytest.mark.hardware
class TestFsoeLifecycleHardware:
    def test_start_and_stop_fsoe(self, hw_motor) -> None:
        if not hw_motor.has_fsoe:
            pytest.skip("Drive does not have FSoE")
        with hw_motor._fsoe_session():
            assert hw_motor._fsoe_active is True
        assert hw_motor._fsoe_active is False


@pytest.mark.hardware
@pytest.mark.usefixtures(hw_encoder.__name__)
class TestInternalGeneratorHardware:
    def test_start_and_stop_with_generator(self, hw_motor) -> None:
        hw_motor.configure_internal_generator()
        with hw_motor.running():
            time.sleep(5)  # Let the motor spin
