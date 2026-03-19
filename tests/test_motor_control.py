import pytest

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
    def test_velocity_mode_enables_and_sets_velocity(self, motor, mock_mc) -> None:
        mock_mc.servos = {"default": mock_mc}
        mock_mc.dictionary.is_safe = False
        mock_mc.motion.get_operation_mode.return_value = 19

        motor.start_motor(velocity=2.0)

        mock_mc.motion.motor_enable.assert_called_once_with(axis=1)
        mock_mc.motion.set_velocity.assert_called_once_with(2.0, axis=1)

    def test_voltage_mode_ramps_frequency(self, motor, mock_mc) -> None:
        mock_mc.servos = {"default": mock_mc}
        mock_mc.dictionary.is_safe = False
        mock_mc.motion.get_operation_mode.return_value = 0

        motor.start_motor()

        mock_mc.motion.motor_enable.assert_called_once_with(axis=1)
        assert mock_mc.communication.set_register.call_count >= 1
        freq_calls = [
            c
            for c in mock_mc.communication.set_register.call_args_list
            if c.args[0] == "FBK_GEN_FREQ"
        ]
        assert len(freq_calls) == 8  # 0.5 to 4.0 in 0.5 steps


class TestStartFsoe:
    def test_raises_when_no_fsoe(self, motor, mock_mc) -> None:
        mock_mc.servos = {"default": mock_mc}
        mock_mc.dictionary.is_safe = False

        with pytest.raises(RuntimeError, match="FSoE is not available"):
            motor.start_fsoe()

    def test_fsoe_active_after_start(self, motor, mock_mc, mocker) -> None:
        mock_mc.servos = {"default": mock_mc}
        mock_mc.dictionary.is_safe = True
        mocker.patch.dict(
            "sys.modules",
            {"ingeniamotion.fsoe_master": mocker.MagicMock()},
        )

        motor.start_fsoe()

        handler = mock_mc.fsoe.create_fsoe_master_handler.return_value
        mock_mc.fsoe.create_fsoe_master_handler.assert_called_once_with(use_sra=True)
        handler.configure_pdo_maps.assert_called_once()
        handler.set_pdo_maps_to_slave.assert_called_once()
        handler.write_safe_parameters.assert_called_once()
        mock_mc.fsoe.start_master.assert_called_once_with(start_pdos=True)
        mock_mc.fsoe.wait_for_state_data.assert_called_once_with(timeout=15)
        assert motor.fsoe_active is True


class TestStopFsoe:
    def test_noop_when_not_active(self, motor, mock_mc) -> None:
        motor.stop_fsoe()
        mock_mc.fsoe.stop_master.assert_not_called()

    def test_stops_when_active(self, motor, mock_mc, mocker) -> None:
        mock_mc.servos = {"default": mock_mc}
        mock_mc.dictionary.is_safe = True
        mocker.patch.dict(
            "sys.modules",
            {"ingeniamotion.fsoe_master": mocker.MagicMock()},
        )
        motor.start_fsoe()

        motor.stop_fsoe()

        mock_mc.fsoe.stop_master.assert_called_once_with(stop_pdos=True)
        assert motor.fsoe_active is False


class TestAutoFsoeOnStartMotor:
    def test_starts_fsoe_automatically(self, motor, mock_mc, mocker) -> None:
        mock_mc.servos = {"default": mock_mc}
        mock_mc.dictionary.is_safe = True
        mock_mc.motion.get_operation_mode.return_value = 19
        mocker.patch.dict(
            "sys.modules",
            {"ingeniamotion.fsoe_master": mocker.MagicMock()},
        )

        motor.start_motor()

        mock_mc.fsoe.create_fsoe_master_handler.assert_called_once()
        assert motor.fsoe_active is True

    def test_skips_fsoe_when_dict_not_safe(self, motor, mock_mc) -> None:
        mock_mc.servos = {"default": mock_mc}
        mock_mc.dictionary.is_safe = False
        mock_mc.motion.get_operation_mode.return_value = 19

        motor.start_motor()

        mock_mc.fsoe.create_fsoe_master_handler.assert_not_called()
        assert motor.fsoe_active is False


class TestRunningContextManager:
    def test_starts_and_stops_motor(self, motor, mock_mc) -> None:
        mock_mc.servos = {"default": mock_mc}
        mock_mc.dictionary.is_safe = False
        mock_mc.motion.get_operation_mode.return_value = 19

        with motor.running(velocity=1.5):
            mock_mc.motion.motor_enable.assert_called_once()

        mock_mc.motion.set_velocity.assert_any_call(0, axis=1)
        mock_mc.motion.motor_disable.assert_called_once()

    def test_stops_on_exception(self, motor, mock_mc) -> None:
        mock_mc.servos = {"default": mock_mc}
        mock_mc.dictionary.is_safe = False
        mock_mc.motion.get_operation_mode.return_value = 19

        with pytest.raises(ValueError, match="test error"):
            with motor.running():
                raise ValueError("test error")

        mock_mc.motion.motor_disable.assert_called_once()


# ---------------------------------------------------------------------------
#  Hardware integration tests (require a physical drive)
# ---------------------------------------------------------------------------


@pytest.fixture
def hw_motor(mc):
    """Create a MotorControl using the real MotionController."""
    return MotorControl(mc, axis=1)


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
        with hw_motor.fsoe_session():
            assert hw_motor.fsoe_active is True
        assert hw_motor.fsoe_active is False


@pytest.mark.hardware
class TestInternalGeneratorHardware:
    def test_start_and_stop_with_generator(self, hw_motor) -> None:
        hw_motor.configure_internal_generator()
        with hw_motor.running():
            pass  # motor spins briefly then stops


@pytest.mark.hardware
class TestVelocityModeHardware:
    def test_start_and_stop_with_velocity(self, hw_motor) -> None:
        hw_motor.configure_velocity_mode(4)
        with hw_motor.running(velocity=1.0):
            pass  # motor spins briefly then stops
