"""Motor control with automatic FSoE detection.

Provides :class:`MotorController` which wraps ``ingeniamotion`` motor operations
and transparently handles FSoE (Functional Safety over EtherCAT) when detected.
"""

from __future__ import annotations

import logging
import time
from contextlib import contextmanager
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from collections.abc import Generator

    from ingeniamotion import MotionController

logger = logging.getLogger(__name__)

# Motor operation modes
_VOLTAGE_MODE = 0
_PROFILE_VELOCITY_MODE = 19

# Internal generator ramp parameters
_GEN_FREQ_START = 0.5
_GEN_FREQ_END = 4.0
_GEN_FREQ_STEP = 0.5


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

        # Deactivate STO and SS1 commands
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

        self._handler = handler
        self._fsoe_active = True
        logger.info("FSoE master running, STO bypass active.")

    def stop_fsoe(self) -> None:
        """Stop the FSoE master if it is running."""
        if not self._fsoe_active:
            return
        try:
            self._mc.fsoe.stop_master(stop_pdos=True)
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

    def configure_velocity_mode(self, feedback_source: int) -> None: # TODO use enum
        """Set the drive to profile velocity mode with the given feedback.

        Args:
            feedback_source: Integer value for ``CL_VEL_FBK_SENSOR`` register.
                Common values: 1 = Primary Absolute Slave 1, 3 = Internal generator,
                4 = Incremental encoder 1, 5 = Digital halls.
        """
        self._mc.communication.set_register(
            "CL_VEL_FBK_SENSOR",
            feedback_source,
            axis=self._axis,
        )
        self._mc.motion.set_operation_mode(_PROFILE_VELOCITY_MODE, axis=self._axis)
        logger.info("Velocity mode configured with feedback: %d.", feedback_source)

    def configure_internal_generator(self) -> None:
        """Set the drive to voltage mode for the internal signal generator."""
        self._mc.motion.set_operation_mode(_VOLTAGE_MODE, axis=self._axis)
        logger.info("Internal generator mode configured.")

    def start_motor(self, *, velocity: float = 1.0) -> None:
        """Enable the motor and bring it up to speed.

        If FSoE is available but not started, it will be started automatically.

        For internal-generator mode the frequency is ramped from 0.5 to 4 Hz.
        For velocity mode the target velocity is set directly.

        Args:
            velocity: Target velocity in rev/s (velocity mode only).
        """
        if self.has_fsoe and not self._fsoe_active:
            self.start_fsoe()

        mode = int(self._mc.motion.get_operation_mode(axis=self._axis))
        self._mc.motion.motor_enable(axis=self._axis)

        if mode == _VOLTAGE_MODE:
            freq = _GEN_FREQ_START
            while freq <= _GEN_FREQ_END:
                self._mc.motion.set_current_quadrature(freq + 1, axis=self._axis)
                time.sleep(0.2)
                self._mc.communication.set_register("FBK_GEN_FREQ", freq, axis=self._axis)
                time.sleep(0.2)
                freq += _GEN_FREQ_STEP
        else:
            self._mc.motion.set_velocity(velocity, axis=self._axis)

        time.sleep(0.5)
        logger.info("Motor started (mode=%d).", mode)

    def stop_motor(self) -> None:
        """Stop the motor and disable it."""
        self._mc.motion.set_velocity(0, axis=self._axis)
        self._mc.motion.motor_disable(axis=self._axis)
        logger.info("Motor stopped.")

    @contextmanager
    def running(self, *, velocity: float = 1.0) -> Generator[None, None, None]:
        """Context manager that starts the motor and stops it on exit.

        Also handles FSoE lifecycle: starts before motor enable, stops after
        motor disable.

        Args:
            velocity: Target velocity in rev/s (velocity mode only).

        Yields:
            None — motor is running.
        """
        self.start_motor(velocity=velocity)
        try:
            yield
        finally:
            self.stop_motor()
            self.stop_fsoe()
