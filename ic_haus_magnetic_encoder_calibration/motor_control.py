"""Motor control with automatic FSoE detection.

Provides :class:`MotorControl` which wraps ``ingeniamotion`` motor operations
and transparently handles FSoE (Functional Safety over EtherCAT) when detected.

The FSoE lifecycle is split into two phases so that callers can register
additional PDO maps (e.g. data TPDO for encoder position) any time before
``activate_pdos()`` is called, typically after ``prepare_fsoe()``.
"""

import logging
import math
import time
from collections.abc import Generator
from contextlib import contextmanager
from typing import Optional

from ingeniamotion import MotionController
from ingeniamotion.enums import OperationMode, SensorType
from ingeniamotion.fsoe_master.handler import FSoEMasterHandler  # noqa: TC002

logger = logging.getLogger(__name__)

# Internal generator defaults
DEFAULT_GEN_FREQ = 0.4  # Hz - saw-tooth frequency for internal generator
DEFAULT_GEN_CURRENT = 1.0  # Amps

# Current and frequency ramping parameters
_GEN_CYCLES = 10_000  # generous upper bound; calibration finishes well before exhaustion
_GEN_DIRECTION = 1  # saw-tooth direction (positive = forward)
_RAMP_STEPS = 10  # number of current ramp steps
_CURRENT_RAMP_INTERVAL = 0.2  # seconds between current ramp steps
_FREQUENCY_RAMP_INTERVAL = 0.8  # seconds between frequency ramp steps
_PDO_WATCHDOG_TIMEOUT = 6.0  # seconds — generous to tolerate GIL blocking (matplotlib, etc.)
_MIN_FREQ_STEP = 1.0  # Hz - starting frequency for the ramp (upper bound)


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
        self._fsoe_prepared = False
        self._handler: Optional[FSoEMasterHandler] = None
        if gen_frequency <= 0:
            raise ValueError(f"gen_frequency must be positive, got {gen_frequency}.")
        self._gen_frequency = gen_frequency
        self._gen_current = gen_current
        self._pdo_exception: Optional[Exception] = None

    @property
    def gen_frequency(self) -> float:
        """Internal generator frequency in Hz."""
        return self._gen_frequency

    @property
    def mc(self) -> MotionController:
        """The underlying MotionController instance."""
        return self._mc

    @property
    def has_fsoe(self) -> bool:
        """True if the connected drive has FSoE (safety) capability."""
        servo = self._mc.servos["default"]
        return servo.dictionary.is_safe

    def _on_pdo_exception(self, exc: Exception) -> None:
        """Callback for PDO exchange thread exceptions."""
        self._pdo_exception = exc

    # -- FSoE lifecycle (two-phase) --

    def prepare_fsoe(self) -> None:
        """Phase 1: configure FSoE handler, register safety PDO maps.

        After this call the safety maps are registered on the servo but
        PDOs are **not** yet started.  The caller should register any
        additional PDO maps (e.g. data TPDO) and then call
        :meth:`activate_pdos`.

        Raises:
            RuntimeError: If FSoE is not available.
        """
        if not self.has_fsoe:
            raise RuntimeError("FSoE is not available on this drive.")

        logger.info("Preparing FSoE master (STO bypass)...")
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

        # Start the FSoE master (but NOT the PDO thread)
        self._mc.fsoe.start_master(start_pdos=False)

        self._handler = handler
        self._fsoe_prepared = True
        logger.info("FSoE prepared (maps on slave, master started, PDOs pending).")

    def activate_pdos(self, *, refresh_rate: Optional[float] = None) -> None:
        """Phase 2: start the PDO exchange thread.

        All PDO maps (safety + data) must already be registered on the
        servo before calling this method.  If FSoE was prepared, this
        also waits for the DATA state and deactivates STO.

        Args:
            refresh_rate: PDO cycle time in seconds.  ``None`` uses the
                library default (10 ms).

        Raises:
            RuntimeError: If the PDO exchange fails to start.
        """
        self._pdo_exception = None
        self._mc.capture.pdo.subscribe_to_exceptions(self._on_pdo_exception)
        self._mc.capture.pdo.start_pdos(
            refresh_rate=refresh_rate,
            watchdog_timeout=_PDO_WATCHDOG_TIMEOUT,
        )
        # Give the PDO thread time to stabilise and detect failures.
        time.sleep(3)
        if self._pdo_exception is not None:
            msg = f"PDO exchange failed to start: {self._pdo_exception}"
            raise RuntimeError(msg)
        logger.info("PDO exchange started.")

        if self._fsoe_prepared:
            self._mc.fsoe.wait_for_state_data(timeout=15)

            assert self._handler is not None
            self._handler.sto_deactivate()
            time.sleep(2)

            logger.info(
                f"FSoE in DATA state (state={self._handler.state}),"
                f" is_sto_active={self._handler.is_sto_active()}",
            )
            self._fsoe_active = True
            logger.info("FSoE master running, STO bypass active.")

    def stop_pdos_and_fsoe(self) -> None:
        """Stop PDO exchange and FSoE if active."""
        self._mc.capture.pdo.unsubscribe_to_exceptions(self._on_pdo_exception)
        if self._fsoe_active:
            assert self._handler is not None
            self._mc.fsoe.stop_master(stop_pdos=True)
            self._handler.remove_pdo_maps_from_slave()
            self._handler.delete()
            logger.info("FSoE master stopped.")
            self._fsoe_active = False
            self._fsoe_prepared = False
        else:
            self._mc.capture.pdo.stop_pdos()
            logger.info("PDO exchange stopped.")

    # -- Motor control --

    def configure_encoders(
        self,
        encoder_sensor_types: list[SensorType],
    ) -> None:
        """Configure feedback sensors for encoders and internal generator.

        Args:
            encoder_sensor_types: Sensor types for each enrolled encoder.
                The first is set as auxiliary feedback, the second (if any)
                as reference feedback.
        """
        self._mc.configuration.set_commutation_feedback(SensorType.INTGEN, axis=self._axis)
        self._mc.configuration.set_velocity_feedback(SensorType.INTGEN, axis=self._axis)
        self._mc.configuration.set_position_feedback(SensorType.INTGEN, axis=self._axis)
        self._mc.configuration.set_auxiliar_feedback(SensorType.INTGEN, axis=self._axis)
        self._mc.configuration.set_reference_feedback(SensorType.INTGEN, axis=self._axis)
        if len(encoder_sensor_types) > 0:
            self._mc.configuration.set_auxiliar_feedback(
                encoder_sensor_types[0],
                axis=self._axis,
            )
        if len(encoder_sensor_types) > 1:
            self._mc.configuration.set_reference_feedback(
                encoder_sensor_types[1],
                axis=self._axis,
            )
        logger.info("Encoder feedback configured.")

    def _start_motor(self) -> None:
        """Enable the motor and spin using the internal signal generator.

        Assumes PDOs (and FSoE if applicable) are already running.
        """
        self._mc.motion.set_operation_mode(OperationMode.CURRENT, axis=self._axis)
        self._mc.motion.fault_reset(axis=self._axis)
        self._mc.motion.motor_enable(axis=self._axis)

        # Ramp current to full amplitude at a static angle.
        current_step = self._gen_current / _RAMP_STEPS
        for i in range(1, _RAMP_STEPS + 1):
            self._mc.motion.set_current_quadrature(
                current=current_step * i,
                axis=self._axis,
            )
            time.sleep(_CURRENT_RAMP_INTERVAL)

        # Start the saw-tooth generator after the rotor is locked.
        # Do it progressively to avoid a large current spike at the start.
        freq_step = min(_MIN_FREQ_STEP, self._gen_frequency)
        ramp_steps = max(1, math.ceil(self._gen_frequency / freq_step))
        for i in range(1, ramp_steps + 1):
            if i == 1:
                # First step: start the generator at a low frequency
                self._mc.motion.internal_generator_saw_tooth_move(
                    direction=_GEN_DIRECTION,
                    cycles=0,
                    frequency=freq_step,
                    axis=self._axis,
                )
            else:
                # Next steps: ramp the frequency up to the target value
                target_freq = self._gen_frequency if i == ramp_steps else freq_step * i
                self._mc.communication.set_register(
                    self._mc.motion.GENERATOR_FREQUENCY_REGISTER,
                    target_freq,
                    axis=self._axis,
                )
            time.sleep(_FREQUENCY_RAMP_INTERVAL)

    def _stop_motor(self) -> None:
        """Disable the motor."""
        self._mc.motion.motor_disable(axis=self._axis)
        logger.info("Motor stopped.")

    @contextmanager
    def motor_spinning(self) -> Generator[None, None, None]:
        """Context manager: start motor, yield, stop motor.

        Does NOT manage PDOs or FSoE — caller must handle those.
        """
        self._start_motor()
        try:
            yield
        finally:
            self._stop_motor()
