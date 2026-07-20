import time

import pytest
from ingeniamotion.enums import SensorType

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


class TestStartMotor:
    def test_enables_motor_and_starts_generator(self, motor, mock_mc) -> None:
        motor._start_motor()

        mock_mc.motion.motor_enable.assert_called_once_with(axis=1)
        mock_mc.motion.internal_generator_saw_tooth_move.assert_called_once()
        assert mock_mc.motion.set_current_quadrature.call_count == 10

    def test_current_ramp_before_generator(self, motor, mock_mc) -> None:
        """Current must be ramped before the saw-tooth starts."""
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

    def test_saw_tooth_before_frequency_ramp(self, mock_mc) -> None:
        """Saw-tooth must start before any frequency-ramp register writes."""
        motor = MotorControl(mock_mc, axis=1, gen_frequency=5.0)
        call_log: list[str] = []
        mock_mc.motion.internal_generator_saw_tooth_move.side_effect = lambda *_a, **_k: (
            call_log.append("saw_tooth")
        )
        mock_mc.communication.set_register.side_effect = lambda *_a, **_k: call_log.append("ramp")

        motor._start_motor()

        assert call_log[0] == "saw_tooth"
        assert call_log.count("ramp") == 4

    def test_no_frequency_ramp_at_low_frequency(self, mock_mc) -> None:
        """At/below the min step, generator starts directly with no ramp."""
        motor = MotorControl(mock_mc, axis=1, gen_frequency=0.4)

        motor._start_motor()

        # Saw-tooth started once at the (single) step frequency, no ramp writes.
        mock_mc.motion.internal_generator_saw_tooth_move.assert_called_once()
        _, kwargs = mock_mc.motion.internal_generator_saw_tooth_move.call_args
        assert kwargs["frequency"] == 0.4
        mock_mc.communication.set_register.assert_not_called()

    def test_frequency_ramps_up_in_steps(self, mock_mc) -> None:
        """Above the min step, frequency is ramped via set_register."""
        motor = MotorControl(mock_mc, axis=1, gen_frequency=5.0)

        motor._start_motor()

        # First step: saw-tooth started at freq_step (1.0 Hz).
        mock_mc.motion.internal_generator_saw_tooth_move.assert_called_once()
        _, saw_kwargs = mock_mc.motion.internal_generator_saw_tooth_move.call_args
        assert saw_kwargs["frequency"] == 1.0

        # Remaining steps ramp the frequency: 2.0, 3.0, 4.0, then target 5.0.
        freqs = [c.args[1] for c in mock_mc.communication.set_register.call_args_list]
        assert freqs == [2.0, 3.0, 4.0, 5.0]

    @pytest.mark.parametrize(
        "target_freq,expected_steps",
        [
            (0.3, []),  # no step to target
            (0.8, []),  # no step to target
            (1.0, []),  # no step to target
            (1.5, [1.5]),  # one step to target
            (2.0, [2.0]),  # one step to target
            (3.4, [2.0, 3.0, 3.4]),  # multiple steps to target
            (3.8, [2.0, 3.0, 3.8]),  # multiple steps to target
            (5.0, [2.0, 3.0, 4.0, 5.0]),  # multiple steps to target
        ],
    )
    def test_frequency_ramp_ends_at_target(self, mock_mc, target_freq, expected_steps) -> None:
        """The last ramp step writes exactly the configured target frequency."""
        motor = MotorControl(mock_mc, axis=1, gen_frequency=target_freq)

        motor._start_motor()

        freqs = [c.args[1] for c in mock_mc.communication.set_register.call_args_list]
        if expected_steps:
            assert freqs[-1] == expected_steps[-1]  # exact target, not freq_step * ramp_steps
        assert freqs == expected_steps  # all steps match expected ramp sequence

    def test_rejects_non_positive_frequency(self, mock_mc) -> None:
        with pytest.raises(ValueError, match="gen_frequency must be positive"):
            MotorControl(mock_mc, axis=1, gen_frequency=0.0)


class TestPrepareFsoe:
    def test_raises_when_no_fsoe(self, motor, mock_mc) -> None:
        mock_mc.servos = {"default": mock_mc}
        mock_mc.dictionary.is_safe = False

        with pytest.raises(RuntimeError, match="FSoE is not available"):
            motor.prepare_fsoe()

    def test_configures_fsoe_master(self, motor, mock_mc) -> None:
        mock_mc.servos = {"default": mock_mc}
        mock_mc.dictionary.is_safe = True

        motor.prepare_fsoe()

        handler = mock_mc.fsoe.create_fsoe_master_handler.return_value
        mock_mc.fsoe.create_fsoe_master_handler.assert_called_once_with(use_sra=True)
        handler.configure_pdo_maps.assert_called_once()
        handler.set_pdo_maps_to_slave.assert_called_once()
        handler.write_safe_parameters.assert_called_once()
        mock_mc.fsoe.start_master.assert_called_once_with(start_pdos=False)

    def test_sout_enabled_for_brake_release(self, motor, mock_mc) -> None:
        """SOut is enabled so the safety master controls brake release."""
        mock_mc.servos = {"default": mock_mc}
        mock_mc.dictionary.is_safe = True

        motor.prepare_fsoe()

        handler = mock_mc.fsoe.create_fsoe_master_handler.return_value
        sout_func = handler.sout_function.return_value
        handler.sout_function.assert_called()
        sout_func.command.set.assert_called_with(True)
        sout_func.sout_disable.set.assert_called_with(0)


class TestActivatePdos:
    def test_starts_pdo_exchange(self, motor, mock_mc) -> None:
        motor.activate_pdos()
        mock_mc.capture.pdo.start_pdos.assert_called_once()

    def test_waits_for_data_state_when_fsoe_prepared(self, motor, mock_mc) -> None:
        mock_mc.servos = {"default": mock_mc}
        mock_mc.dictionary.is_safe = True
        motor.prepare_fsoe()

        motor.activate_pdos()

        mock_mc.fsoe.wait_for_state_data.assert_called_once_with(timeout=15)
        handler = mock_mc.fsoe.create_fsoe_master_handler.return_value
        handler.sto_deactivate.assert_called_once()


class TestStopPdosAndFsoe:
    def test_noop_when_not_active(self, motor, mock_mc) -> None:
        motor.stop_pdos_and_fsoe()
        mock_mc.fsoe.stop_master.assert_not_called()
        mock_mc.capture.pdo.stop_pdos.assert_called_once()

    def test_stops_fsoe_when_active(self, motor, mock_mc) -> None:
        mock_mc.servos = {"default": mock_mc}
        mock_mc.dictionary.is_safe = True
        motor.prepare_fsoe()
        motor.activate_pdos()

        motor.stop_pdos_and_fsoe()

        mock_mc.fsoe.stop_master.assert_called_once_with(stop_pdos=True)


class TestMotorSpinning:
    def test_starts_and_stops_motor(self, motor, mock_mc) -> None:
        with motor.motor_spinning():
            mock_mc.motion.motor_enable.assert_called_once()

        mock_mc.motion.motor_disable.assert_called_once()

    def test_stops_on_exception(self, motor, mock_mc) -> None:
        with pytest.raises(ValueError, match="test error"), motor.motor_spinning():
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
    enc = Encoder(mc, sensor_type=SensorType.ABS1, axis=1)
    enc.ensure_normal_mode()
    enc._write_ic(CFGEW, 0xFF)
    return enc


@pytest.mark.hardware
class TestHasFsoeHardware:
    def test_detects_fsoe_from_dictionary(self, hw_motor) -> None:
        result = hw_motor.has_fsoe
        assert isinstance(result, bool)


@pytest.mark.hardware
@pytest.mark.usefixtures(hw_encoder.__name__)
class TestInternalGeneratorHardware:
    def test_start_and_stop_with_generator(self, hw_motor) -> None:
        hw_motor.configure_encoders(
            encoder_sensor_types=[],
        )
        if hw_motor.has_fsoe:
            assert not hw_motor._fsoe_prepared
            hw_motor.prepare_fsoe()
            assert hw_motor._fsoe_prepared
            assert not hw_motor._fsoe_active

        hw_motor.activate_pdos()
        if hw_motor.has_fsoe:
            assert hw_motor._fsoe_active

        try:
            with hw_motor.motor_spinning():
                time.sleep(2)
        finally:
            hw_motor.stop_pdos_and_fsoe()

        if hw_motor.has_fsoe:
            assert not hw_motor._fsoe_active
            assert not hw_motor._fsoe_prepared
