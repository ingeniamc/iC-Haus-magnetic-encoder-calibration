"""Motor control with automatic FSoE detection.

Provides :class:`MotorController` which wraps ``ingeniamotion`` motor operations
and transparently handles FSoE (Functional Safety over EtherCAT) when detected.
"""

from __future__ import annotations

import logging
import time
from contextlib import contextmanager
from typing import TYPE_CHECKING

from ingeniamotion.enums import OperationMode, SensorType

if TYPE_CHECKING:
    from collections.abc import Generator

    from ingeniamotion import MotionController

logger = logging.getLogger(__name__)

# Internal generator defaults
DEFAULT_GEN_FREQ = 0.4  # Hz – saw-tooth frequency for internal generator
DEFAULT_GEN_CURRENT = 1.0  # Amps


_GEN_CYCLES = 100  # number of generator cycles
_RAMP_STEPS = 10  # number of current ramp steps
_RAMP_INTERVAL = 0.2  # seconds between ramp steps
_PDO_WATCHDOG_TIMEOUT = 0.3  # seconds


class MotorControl:
    """Motor control with automatic FSoE support.

    Auto-detects whether the drive has FSoE and starts the safety master
    before enabling the motor when required.

    Args:
        mc: Connected MotionController instance.
        axis: Drive axis number.
        gen_frequency: Saw-tooth generator frequency in Hz.
        gen_current: Quadrature current target in amps.
    """

    def __init__(
        self,
        mc: MotionController,
        *,
        axis: int = 1,
        gen_frequency: float = DEFAULT_GEN_FREQ,
        gen_current: float = DEFAULT_GEN_CURRENT,
    ) -> None:
        self._mc = mc
        self._axis = axis
        self._fsoe_active = False
        self._gen_frequency = gen_frequency
        self._gen_current = gen_current

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

    # -- FSoE lifecycle --

    def _start_fsoe(self) -> None:
        """Start the FSoE master in STO bypass mode and enable STO.

        Resets all safety parameters to dictionary defaults, configures PDO
        maps, writes parameters to the slave, and starts the master.

        Raises:
            RuntimeError: If FSoE is not available.
        """
        if not self.has_fsoe:
            raise RuntimeError("FSoE is not available on this drive.")

        logger.info("Starting FSoE master (STO bypass)...")
        handler = self._mc.fsoe.create_fsoe_master_handler(use_sra=True)

        sto_func = handler.sto_function()
        sout_func = handler.sout_function()

        # Build process image: STO + SOut commands
        if handler.process_image.editable:
            handler.process_image.inputs.clear()
            handler.process_image.outputs.clear()
            handler.process_image.inputs.add(sto_func.command)
            handler.process_image.outputs.add(sto_func.command)
            if sout_func is not None:
                handler.process_image.inputs.add(sout_func.command)
                handler.process_image.outputs.add(sout_func.command)

        # Reset all safety parameters to dictionary defaults
        for uid, param in handler.safety_parameters.items():
            if uid.startswith("ETG_COMMS_"):
                continue
            default = param.register.default
            if default is not None:
                param.set_without_updating(default)

        # Deactivate STO, and SOut commands (initial values before start)
        sto_func.command.set(True)
        if sout_func is not None:
            sout_func.command.set(True)
            sout_func.sout_disable.set(0)

        # Configure PDO maps and write safety params to slave
        handler.configure_pdo_maps()
        handler.set_pdo_maps_to_slave()
        handler.write_safe_parameters()

        # Start master and PDOs via the high-level API
        self._mc.fsoe.start_master(start_pdos=False)
        self._mc.capture.pdo.start_pdos(watchdog_timeout=_PDO_WATCHDOG_TIMEOUT)
        self._mc.fsoe.wait_for_state_data(timeout=15)

        # Exit failsafe mode and deactivate STO after DATA state
        handler.sto_deactivate()
        time.sleep(2)

        logger.info(
            "FSoE in DATA state (state=%s), is_sto_active=%s",
            handler.state,
            handler.is_sto_active(),
        )

        self._handler = handler
        self._fsoe_active = True
        logger.info("FSoE master running, STO bypass active.")

    def _stop_fsoe(self) -> None:
        """Stop the FSoE master if it is running."""
        if not self._fsoe_active:
            return
        try:
            self._mc.fsoe.stop_master(stop_pdos=True)
            self._handler.remove_pdo_maps_from_slave()
            self._handler.delete()
            logger.info("FSoE master stopped.")
        except Exception:
            logger.exception("Error stopping FSoE master")
        self._fsoe_active = False

    @contextmanager
    def _fsoe_session(self) -> Generator[None, None, None]:
        """Context manager that starts FSoE and stops it on exit.

        Raises:
            RuntimeError: If FSoE is not available.

        Yields:
            None — FSoE master is running with STO enabled.
        """
        self._start_fsoe()
        try:
            yield
        finally:
            self._stop_fsoe()

    # -- Motor control --

    def configure_internal_generator(self) -> None:
        """Configure feedback sensors and current mode for the internal generator.

        Sets commutation, velocity, and position feedback to INTGEN so the
        internal saw-tooth generator drives the motor independently of the
        absolute encoder.  Auxiliary feedback stays on ABS1 so the BiSS
        interface remains active for iC-MU register communication.

        The drive frame configuration must be updated separately (by the
        calibrator) to match whatever the encoder currently outputs so that
        CRC checks pass.
        """
        self._mc.configuration.set_commutation_feedback(SensorType.INTGEN, axis=self._axis)
        self._mc.configuration.set_velocity_feedback(SensorType.INTGEN, axis=self._axis)
        self._mc.configuration.set_position_feedback(SensorType.INTGEN, axis=self._axis)
        self._mc.configuration.set_auxiliar_feedback(SensorType.ABS1, axis=self._axis)
        self._mc.motion.set_operation_mode(OperationMode.CURRENT, axis=self._axis)
        logger.info("Internal generator mode configured (current mode).")

    def _start_motor(self) -> None:
        """Enable the motor and spin using the internal signal generator.

        If FSoE is available but not started, it will be started automatically.
        Uses a saw-tooth internal generator.  The current is ramped up first
        to lock the rotor magnetically, then the saw-tooth generator starts
        so the field advances from the locked position.  Current ramp uses
        discrete steps with sleeps to avoid starving FSoE PDO exchanges.
        """
        if self.has_fsoe and not self._fsoe_active:
            self._start_fsoe()

        self._mc.motion.fault_reset(axis=self._axis)

        self._mc.motion.motor_enable(axis=self._axis)

        # Step 1: Ramp current to full amplitude at a static angle.
        # This magnetically locks the rotor so it is in sync with the
        # stator field before we start moving.
        step = self._gen_current / _RAMP_STEPS
        for i in range(1, _RAMP_STEPS + 1):
            self._mc.motion.set_current_quadrature(
                step * i,
                axis=self._axis,
            )
            time.sleep(_RAMP_INTERVAL)

        # Step 2: Start the saw-tooth generator *after* the rotor is locked.
        # The field now advances smoothly from the locked position.
        self._mc.motion.internal_generator_saw_tooth_move(
            1,
            _GEN_CYCLES,
            self._gen_frequency,
            axis=self._axis,
        )

        logger.info("Motor started.")

    def _stop_motor(self) -> None:
        """Stop the motor, disable it, and stop FSoE if running."""
        self._mc.motion.motor_disable(axis=self._axis)
        self._stop_fsoe()
        logger.info("Motor stopped.")

    @contextmanager
    def running(self) -> Generator[None, None, None]:
        """Context manager that starts the motor and stops it on exit.

        Handles FSoE lifecycle automatically: starts before motor enable,
        stops after motor disable.

        Yields:
            None — motor is running.
        """
        self._start_motor()
        try:
            yield
        finally:
            self._stop_motor()
