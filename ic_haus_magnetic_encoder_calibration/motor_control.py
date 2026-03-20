"""Motor control with automatic FSoE detection.

Provides :class:`MotorController` which wraps ``ingeniamotion`` motor operations
and transparently handles FSoE (Functional Safety over EtherCAT) when detected.
"""

from __future__ import annotations

import logging
import time
from contextlib import contextmanager
from typing import TYPE_CHECKING

from ingeniamotion.enums import GeneratorMode, OperationMode, SensorType

if TYPE_CHECKING:
    from collections.abc import Generator

    from ingeniamotion import MotionController

logger = logging.getLogger(__name__)

# Internal generator defaults
_GEN_FREQ = 0.4  # Hz – saw-tooth frequency for internal generator
_GEN_CURRENT = 8.0  # Amps – 80% of MOT_RATED_CURRENT (10 A)
_GEN_CYCLES = 100  # number of generator cycles
_RAMP_STEPS = 10  # number of current ramp steps
_RAMP_INTERVAL = 0.2  # seconds between ramp steps


class MotorControl:
    """Motor control with automatic FSoE support.

    Auto-detects whether the drive has FSoE and starts the safety master
    before enabling the motor when required.

    Args:
        mc: Connected MotionController instance.
        axis: Drive axis number.
    """

    def __init__(self, mc: MotionController, *, axis: int = 1) -> None:
        self._mc = mc
        self._axis = axis
        self._fsoe_active = False

    @property
    def has_fsoe(self) -> bool:
        """True if the connected drive has FSoE (safety) capability.

        Checks that the drive dictionary declares safety support.
        """
        try:
            servo = self._mc.servos["default"]
            return bool(servo.dictionary.is_safe)
        except (KeyError, AttributeError):
            return False

    @property
    def fsoe_active(self) -> bool:
        """True if the FSoE master is currently running."""
        return self._fsoe_active

    # -- FSoE lifecycle --

    def start_fsoe(self) -> None:
        """Start the FSoE master in STO bypass mode and enable STO.

        Resets all safety parameters to dictionary defaults, configures PDO
        maps, writes parameters to the slave, and starts the master.

        Raises:
            RuntimeError: If FSoE is not available.
        """
        if not self.has_fsoe:
            raise RuntimeError("FSoE is not available on this drive.")

        from ingeniamotion.fsoe_master import STOFunction

        logger.info("Starting FSoE master (STO bypass)...")
        handler = self._mc.fsoe.create_fsoe_master_handler(use_sra=True)

        sto_func = handler.sto_function()
        ss1_func = handler.ss1_function()

        # Build minimal process image: STO command only
        if handler.process_image.editable:
            handler.process_image.inputs.clear()
            handler.process_image.outputs.clear()
            handler.process_image.inputs.add(sto_func.command)
            handler.process_image.outputs.add(sto_func.command)

        # Reset all safety parameters to dictionary defaults
        for uid, param in handler.safety_parameters.items():
            if uid.startswith("ETG_COMMS_"):
                continue
            default = param.register.default
            if default is not None:
                param.set_without_updating(default)

        # Deactivate STO and SS1 commands (initial values before start)
        sto_func.command.set(True)
        ss1_func.command.set(True)
        if handler.sout_function() is not None:
            handler.sout_disable()

        # Configure PDO maps and write safety params to slave
        handler.configure_pdo_maps()
        handler.set_pdo_maps_to_slave()
        handler.write_safe_parameters()

        # Start master and PDOs via the high-level API
        self._mc.fsoe.start_master(start_pdos=True)
        self._mc.fsoe.wait_for_state_data(timeout=15)

        # Exit failsafe mode and deactivate STO/SS1 after DATA state
        handler.sto_deactivate()
        handler.ss1_deactivate()
        time.sleep(2)

        logger.info(
            "FSoE in DATA state (state=%s), is_sto_active=%s",
            handler.state,
            handler.is_sto_active(),
        )

        self._handler = handler
        self._fsoe_active = True
        logger.info("FSoE master running, STO bypass active.")

    def stop_fsoe(self) -> None:
        """Stop the FSoE master if it is running."""
        if not self._fsoe_active:
            return
        try:
            self._mc.fsoe.stop_master(stop_pdos=True)
            self._handler.remove_pdo_maps_from_slave()
            self._handler.delete()
            logger.info("FSoE master stopped.")
        except Exception:
            logger.exception("Error stopping FSoE master.")
        self._fsoe_active = False

    @contextmanager
    def fsoe_session(self) -> Generator[None, None, None]:
        """Context manager that starts FSoE and stops it on exit.

        Raises:
            RuntimeError: If FSoE is not available.

        Yields:
            None — FSoE master is running with STO enabled.
        """
        self.start_fsoe()
        try:
            yield
        finally:
            self.stop_fsoe()

    # -- Motor control --

    def _set_all_feedback_sensors(self, source: SensorType) -> None:
        """Set all four feedback sensor registers to *source*."""
        for reg in (
            "CL_VEL_FBK_SENSOR",
            "CL_POS_FBK_SENSOR",
            "CL_AUX_FBK_SENSOR",
            "COMMU_ANGLE_SENSOR",
        ):
            self._mc.communication.set_register(reg, source, axis=self._axis)

    def configure_internal_generator(self) -> None:
        """Configure feedback sensors and current mode for the internal generator.

        Sets the commutation angle sensor to INTGEN so the saw-tooth
        generator drives the commutation.  Other feedback sensors are set
        to QEI to avoid CRC errors from the uncalibrated absolute encoder.
        Sets operation mode to current so the current loop drives the motor.
        """
        self._mc.configuration.set_commutation_feedback(
            SensorType.INTGEN, axis=self._axis
        )
        for setter in (
            self._mc.configuration.set_reference_feedback,
            self._mc.configuration.set_velocity_feedback,
            self._mc.configuration.set_position_feedback,
            self._mc.configuration.set_auxiliar_feedback,
        ):
            setter(SensorType.QEI, axis=self._axis)
        self._mc.motion.set_operation_mode(OperationMode.CURRENT, axis=self._axis)
        logger.info("Internal generator mode configured (current mode).")

    def start_motor(self) -> None:
        """Enable the motor and spin using the internal signal generator.

        If FSoE is available but not started, it will be started automatically.
        Uses a saw-tooth internal generator.  The current is ramped up in
        discrete steps with sleeps between them so FSoE PDO exchanges are
        not starved.
        """
        if self.has_fsoe and not self._fsoe_active:
            self.start_fsoe()

        self._mc.motion.motor_enable(axis=self._axis)

        # Configure saw-tooth generator movement and trigger it.
        self._mc.motion.internal_generator_saw_tooth_move(
            1, _GEN_CYCLES, _GEN_FREQ, axis=self._axis
        )

        # Ramp current until it reaches the target for calibration
        step = _GEN_CURRENT / _RAMP_STEPS
        for i in range(1, _RAMP_STEPS + 1):
            self._mc.motion.set_current_quadrature(
                step * i, axis=self._axis
            )
            time.sleep(_RAMP_INTERVAL)

        logger.info("Motor started.")

    def stop_motor(self) -> None:
        """Stop the motor and disable it."""
        self._mc.motion.motor_disable(axis=self._axis)
        logger.info("Motor stopped.")

    @contextmanager
    def running(self) -> Generator[None, None, None]:
        """Context manager that starts the motor and stops it on exit.

        Also handles FSoE lifecycle: starts before motor enable, stops after
        motor disable.

        Yields:
            None — motor is running.
        """
        self.start_motor()
        try:
            yield
        finally:
            self.stop_motor()
            self.stop_fsoe()
