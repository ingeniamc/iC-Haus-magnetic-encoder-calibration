"""Single iC-MU encoder: BiSS operations, register management, calibration math.

Each ``Encoder`` instance wraps one physical iC-MU encoder connected via
a BiSS channel on a Novanta/Ingenia drive. It handles:

* Reading the chip revision
* Saving/restoring drive encoder frame configuration
* Saving/restoring iC-MU calibration-mode registers
* Reading/writing analog and nonius (SPO) parameters
* Saving calibration results to EEPROM
"""

import logging
import time
from dataclasses import dataclass, field
from typing import Optional, Union

import mu_3sl_interface as mu_3sl
from ingeniamotion import MotionController
from ingeniamotion.enums import SensorType

from ic_haus_magnetic_encoder_calibration.config_loader import (
    EncoderRegisterConfig,
)

from .drive_encoder_registers import (
    CALIB_ERROR_TOLERANCE,
    CALIB_FRAME_SIZE,
    CALIB_FRAME_TYPE,
    CALIB_POLARITY,
    CALIB_POS_BITS,
    CALIB_POS_OFFSET,
    CALIB_POS_ST_BITS,
    CALIB_POS_START_BIT,
    CALIB_PROTOCOL,
    DriveEncoderRegisters,
    get_encoder_registers,
)
from .ic_haus_registers import (
    CFGEW,
    CMD,
    ENAC,
    GX_M,
    GX_N,
    HARD_REV,
    MODEA_MODEB,
    MPC,
    OUT_LSB_ST,
    OUT_MSB_ZERO,
    PH_M,
    PH_N,
    SPO_REGISTERS,
    STATUS1,
    TEST,
    VOSC_M,
    VOSC_N,
    VOSS_M,
    VOSS_N,
    BissAction,
    ICHausRegister,
    ICHausRegisterField,
)

logger = logging.getLogger(__name__)

_BISS_SETTLE_S = 0.1

# Calibration field values for iC-MU registers
# Obtained from iC-Haus AN1 "Offline Calibration" application note, Table 1.
_CALIB_ENAC = 0x01
_CALIB_OUT_LSB = 0x00
_CALIB_OUT_MSB = 0x0E  # MSB = 13 + 14 = 27 (BiSS payload bits 0-27)
_CALIB_OUT_ZERO_BISS = 0x00  # No padding zeros for BiSS
_CALIB_MODE_ST = 0x02  # Raw analog data (required for calibration)
_CALIB_TEST = 0x00  # Deactivate testmodes
_CALIB_MPC = 0x0B  # MPC=11 (2^11=2048 master periods) is the minimum allowed for raw data capture
_CALIB_MODEA_BISS = 0x02  # Enable BiSS interface

# CFGEW value that suppresses all error/warning sources from the
# BiSS-C ERR and WRN status bits.  An uncalibrated encoder will
# otherwise assert ERR=0 (low-active) due to amplitude or internal
# CRC issues, causing the drive to fault with 0x7380.
_CALIB_CFGEW_SUPPRESS = 0xFF

# Raw data bit widths for master/nonius tracks in the BiSS payload.
_MASTER_WIDTH = 14
_NONIUS_WIDTH = 14
_MASTER_MASK = (1 << _MASTER_WIDTH) - 1
_NONIUS_MASK = (1 << _NONIUS_WIDTH) - 1


def split_raw_payload(payload: int) -> tuple[int, int]:
    """Extract 14-bit master and nonius from a packed BiSS payload.

    Returns:
        Tuple of (master, nonius) 14-bit values.
    """
    master = payload & _MASTER_MASK
    nonius = (payload >> _NONIUS_WIDTH) & _NONIUS_MASK
    return master, nonius


# Factory default analog parameters per iC-MU Series datasheet (Rev B1).
# These are the recommended starting values when no prior calibration exists.
_FACTORY_DEFAULT_GX = 0x00  # Cosine gain: 0% (no correction)
_FACTORY_DEFAULT_VOSS = 0x3F  # Sine offset: ~60-70 mV
_FACTORY_DEFAULT_VOSC = 0x3F  # Cosine offset: ~60-70 mV
_FACTORY_DEFAULT_PH = 0x3F  # Phase adjustment: baseline

# iC-MU commands (CMD register 0x75)
_CMD_WRITE_ALL = 0x01
_CMD_ABS_RESET = 0x03

# Mapping from drive feedback sensor type to physical encoder channel.
_SENSOR_TYPE_TO_ENCODER: dict[SensorType, int] = {
    SensorType.ABS1: 1,
    SensorType.SSI2: 2,
}


@dataclass(frozen=True)
class DriveFrameConfig:
    """Drive encoder BiSS frame configuration."""

    frame_size: int
    frame_type: int
    pos_bits: int
    pos_st_bits: int
    pos_start_bit: int
    pos_offset: int
    polarity: int
    error_tolerance: int


@dataclass(frozen=True)
class ICMURegisterState:
    """Snapshot of iC-MU configuration registers affected by calibration."""

    enac: int
    modea_modeb: int
    out_msb_zero: int
    out_lsb_st: int
    test: int
    mpc: int
    cfgew: int


@dataclass
class CalibrationResult:
    """Results of calibration for one encoder."""

    success: bool = True
    iterations: int = 0
    master_adjustments: Optional[mu_3sl.AnalogTrackAdjustments] = None
    nonius_adjustments: Optional[mu_3sl.AnalogTrackAdjustments] = None
    spo_base: int = 0
    spo_n: list[int] = field(default_factory=list)
    nonius_in_range_max: float = 0.0
    nonius_in_range_min: float = 0.0


# ---------------------------------------------------------------------------
# Encoder class
# ---------------------------------------------------------------------------


class Encoder:
    """Wraps a single iC-MU encoder connected via BiSS.

    Args:
        mc: Connected MotionController instance.
        sensor_type: Drive feedback sensor type for this encoder.
            The encoder channel (1 or 2) is derived from the sensor type.
        axis: Drive axis number.
        config: Register configuration.
    """

    def __init__(
        self,
        mc: MotionController,
        sensor_type: SensorType,
        *,
        axis: int = 1,
        config: EncoderRegisterConfig,
    ) -> None:
        self._mc = mc
        self._sensor_type = sensor_type
        self._number = _SENSOR_TYPE_TO_ENCODER[sensor_type]
        self._axis = axis
        self._regs = get_encoder_registers(self._number)
        self._config: EncoderRegisterConfig = config

    @property
    def number(self) -> int:
        """Encoder channel number (1 or 2)."""
        return self._number

    @property
    def sensor_type(self) -> SensorType:
        """Drive feedback sensor type for this encoder."""
        return self._sensor_type

    @property
    def regs(self) -> DriveEncoderRegisters:
        """Drive register names for this encoder."""
        return self._regs

    # -- Drive register helpers --

    def _read_drive(self, name: str) -> int:
        return int(self._mc.communication.get_register(name, axis=self._axis))

    def _write_drive(self, name: str, value: int) -> None:
        self._mc.communication.set_register(name, value, axis=self._axis)

    # -- iC-MU BiSS register helpers --

    def _read_ic(self, reg: ICHausRegister) -> int:
        """Read an 8-bit value from an iC-MU register via BiSS bidirectional.

        Returns:
            Register value (masked to 0xFF).
        """
        regs = self._regs
        ax = self._axis
        self._mc.communication.set_register(regs.itf_ctl, BissAction.NO_ACTION, axis=ax)
        self._mc.communication.set_register(regs.itf_addr, reg.address, axis=ax)
        self._mc.communication.set_register(regs.itf_ctl, BissAction.READ, axis=ax)
        # Let the BiSS transaction settle before reading the result register
        time.sleep(_BISS_SETTLE_S)
        raw = int(self._mc.communication.get_register(regs.itf_data, axis=ax)) & 0xFF
        logger.debug(
            f"Encoder {self._number}: _read_ic(0x{reg.address:02X}) -> 0x{raw:02X}",
        )
        return raw

    def _write_ic(self, reg: Union[ICHausRegister, ICHausRegisterField], value: int) -> None:
        """Write an 8-bit value to an iC-MU register or a register field via BiSS bidirectional.

        Args:
            reg: ICHausRegister or ICHausRegisterField to write.
            value: 8-bit value to write (0-255).

        Raises:
            ValueError: If a register field is provided without a parent register.

        """
        field = None
        if isinstance(reg, ICHausRegisterField):
            # If a field is passed, use it to mask/shift the value
            field = reg
            register = field.register
            if register is None:
                raise ValueError(f"Field {field.name} has no parent register. Cannot write.")
            value = field.insert(self._read_ic(register), value)
            reg = register
        regs = self._regs
        ax = self._axis
        self._mc.communication.set_register(regs.itf_ctl, BissAction.NO_ACTION, axis=ax)
        self._mc.communication.set_register(regs.itf_addr, reg.address, axis=ax)
        self._mc.communication.set_register(regs.itf_data, value & 0xFF, axis=ax)
        self._mc.communication.set_register(regs.itf_ctl, BissAction.WRITE, axis=ax)
        # Let the BiSS transaction settle before any subsequent operations.
        # They could potentially depend on the new register value, so it's safer to wait here
        time.sleep(_BISS_SETTLE_S)

    def _read_ic_field(
        self,
        register: ICHausRegister,
        register_field: ICHausRegisterField,
    ) -> int:
        """Read a single bit-field via read-modify-write.

        Args:
            register: ICHausRegister object to read from.
            register_field: ICHausRegisterField object specifying the field to extract.

        Returns:
            Integer value of the specified field.

        """
        raw = self._read_ic(register)
        return register_field.extract(raw)

    # -- Step 1: Revision --

    @property
    def is_bissc(self) -> bool:
        """Check if the encoder is configured for BiSS-C protocol.

        Returns:
            True if the drive encoder protocol is BiSS-C, False otherwise.

        """
        protocol_val = self._read_drive(self._regs.protocol)
        return protocol_val == CALIB_PROTOCOL

    def read_revision(self) -> mu_3sl.Revision:
        """Read the iC-MU hardware revision code.

        Returns:
            Revision enum value.

        Raises:
            RuntimeError: If the chip does not respond (NONE).
        """
        raw = self._read_ic(HARD_REV)
        logger.debug(
            f"Encoder {self._number}: HARD_REV(0x{HARD_REV.address:02X}) raw=0x{raw:02X}",
        )
        revision = mu_3sl.Revision(raw)
        if revision is mu_3sl.Revision.NONE:
            msg = f"Encoder {self._number}: unable to read revision (got 0x00)."
            raise RuntimeError(msg)
        logger.info(f"Encoder {self._number}: revision {revision.name} (0x{raw:02X})")
        return revision

    # -- Drive encoder frame config --

    def get_drive_config(self) -> DriveFrameConfig:
        """Read current drive encoder frame configuration.

        Returns:
            Current frame settings.
        """
        r = self._regs
        return DriveFrameConfig(
            frame_size=self._read_drive(r.frame_size),
            frame_type=self._read_drive(r.frame_type),
            pos_bits=self._read_drive(r.pos_bits),
            pos_st_bits=self._read_drive(r.pos_st_bits),
            pos_start_bit=self._read_drive(r.pos_start_bit),
            pos_offset=self._read_drive(r.pos_offset),
            polarity=self._read_drive(r.polarity),
            error_tolerance=self._read_drive(r.error_tolerance),
        )

    def set_drive_config(self, config: DriveFrameConfig) -> None:
        """Write drive encoder frame configuration.

        Args:
            config: Frame settings to apply.
        """
        r = self._regs
        self._write_drive(r.frame_size, config.frame_size)
        self._write_drive(r.frame_type, config.frame_type)
        self._write_drive(r.pos_bits, config.pos_bits)
        self._write_drive(r.pos_st_bits, config.pos_st_bits)
        self._write_drive(r.pos_start_bit, config.pos_start_bit)
        self._write_drive(r.pos_offset, config.pos_offset)
        self._write_drive(r.polarity, config.polarity)
        self._write_drive(r.error_tolerance, config.error_tolerance)
        logger.info(f"Encoder {self._number}: drive frame config applied.")

    # -- iC-MU register config --

    def get_ic_config(self) -> ICMURegisterState:
        """Read the iC-MU configuration registers affected by calibration.

        Returns:
            Current register values.
        """
        return ICMURegisterState(
            enac=self._read_ic(ENAC),
            modea_modeb=self._read_ic(MODEA_MODEB),
            out_msb_zero=self._read_ic(OUT_MSB_ZERO),
            out_lsb_st=self._read_ic(OUT_LSB_ST),
            test=self._read_ic(TEST),
            mpc=self._read_ic(MPC),
            cfgew=self._read_ic(CFGEW),
        )

    def set_ic_config(self, state: ICMURegisterState) -> None:
        """Write iC-MU configuration registers.

        Args:
            state: Register values to apply.
        """
        self._write_ic(ENAC, state.enac)
        self._write_ic(MODEA_MODEB, state.modea_modeb)
        self._write_ic(OUT_MSB_ZERO, state.out_msb_zero)
        self._write_ic(OUT_LSB_ST, state.out_lsb_st)
        self._write_ic(TEST, state.test)
        self._write_ic(MPC, state.mpc)
        self._write_ic(CFGEW, state.cfgew)
        logger.info(f"Encoder {self._number}: iC-MU config registers applied.")

    def apply_config(self) -> None:
        """Apply iC-MU configuration register values.

        Configuration is defined in the EncoderRegisterConfig object passed to the constructor.

        """
        for item, value in self._config.register_writes():
            self._write_ic(item, value)
            # item can be a field or a register; get the appropriate name for logging
            if isinstance(item, ICHausRegisterField) and item.register is not None:
                display_name = f"{item.register.name}.{item.name}"
            else:
                display_name = item.name
            logger.debug(
                f"Encoder {self._number}: {display_name} = 0x{value:02x}",
            )
        logger.info(f"Encoder {self._number}: configuration applied ({self._config})")

    # -- Calibration mode --
    def configure_in_calibration_mode(self) -> int:
        """Configure iC-MU registers and drive frame for calibration.

        Writes calibration values and sets the drive encoder frame for raw data capture.
        Use ``get_ic_config`` / ``get_drive_config`` before calling this
        method if you need to restore the original state later.

        Returns:
            Number of master periods (2^MPC).
        """
        # Enable analog calibration (ENAC bit)
        self._write_ic(ENAC.field("enac"), _CALIB_ENAC)

        # Set interface mode to BiSS
        self._write_ic(MODEA_MODEB.field("modea"), _CALIB_MODEA_BISS)

        # Configure output shift register length:
        # OUT_MSB=0x0E selects bit 27 as the MSB (14 master + 14 nonius = 28 bits)
        # OUT_ZERO=0x00 — no padding zeros (BiSS doesn't need them; SPI uses 0x04)
        self._write_ic(OUT_MSB_ZERO.field("out_msb"), _CALIB_OUT_MSB)
        self._write_ic(OUT_MSB_ZERO.field("out_zero"), _CALIB_OUT_ZERO_BISS)

        # Select raw master+nonius track output:
        # MODE_ST=0x02 selects raw analog data (required for calibration)
        # OUT_LSB=0x00 starts output from bit 0 (no truncation)
        self._write_ic(OUT_LSB_ST.field("mode_st"), _CALIB_MODE_ST)
        self._write_ic(OUT_LSB_ST.field("out_lsb"), _CALIB_OUT_LSB)

        # Clear test register
        self._write_ic(TEST, _CALIB_TEST)

        # Suppress all ERR/WRN sources so the drive doesn't fault
        # on error bits from the (still uncalibrated) encoder.
        self._write_ic(CFGEW, _CALIB_CFGEW_SUPPRESS)

        # MPC: if 0x0C set to 0x0B (per iC-Haus AN1 "Offline Calibration", Table 1)
        mpc_val = self._read_ic_field(MPC, MPC.field("mpc"))
        if mpc_val == 0x0C:
            # MPC = 12 not allowed for raw data
            self._write_ic(MPC.field("mpc"), _CALIB_MPC)
            mpc_val = _CALIB_MPC

        # Configure drive frame for calibration
        r = self._regs
        self._write_drive(r.frame_size, CALIB_FRAME_SIZE)
        self._write_drive(r.frame_type, CALIB_FRAME_TYPE)
        self._write_drive(r.pos_bits, CALIB_POS_BITS)
        self._write_drive(r.pos_st_bits, CALIB_POS_ST_BITS)
        self._write_drive(r.pos_start_bit, CALIB_POS_START_BIT)
        self._write_drive(r.pos_offset, CALIB_POS_OFFSET)
        self._write_drive(r.polarity, CALIB_POLARITY)
        # Raise error tolerance after the frame change to prevent the
        # drive from freezing POS_VALUE during transient CRC mismatches.
        self._write_drive(r.error_tolerance, CALIB_ERROR_TOLERANCE)

        n_master_periods = 1 << mpc_val
        logger.info(
            f"Encoder {self._number}: calibration mode configured"
            f" (MPC={mpc_val}, periods={n_master_periods}).",
        )
        return n_master_periods

    # -- Analog parameters --

    def read_analog_adjustments(
        self,
    ) -> tuple[mu_3sl.AnalogTrackAdjustments, mu_3sl.AnalogTrackAdjustments]:
        """Read current analog calibration parameters from chip.

        Returns:
            Tuple of (master, nonius) AnalogTrackAdjustments.
        """
        master = mu_3sl.AnalogTrackAdjustments(
            self._read_ic(GX_M),
            self._read_ic(VOSS_M),
            self._read_ic(VOSC_M),
            self._read_ic(PH_M),
        )
        nonius = mu_3sl.AnalogTrackAdjustments(
            self._read_ic(GX_N),
            self._read_ic(VOSS_N),
            self._read_ic(VOSC_N),
            self._read_ic(PH_N),
        )
        return master, nonius

    def write_analog_adjustments(
        self,
        master: mu_3sl.AnalogTrackAdjustments,
        nonius: mu_3sl.AnalogTrackAdjustments,
    ) -> None:
        """Write analog calibration parameters to chip.

        Args:
            master: Master track adjustments.
            nonius: Nonius track adjustments.
        """
        self._write_ic(GX_M, master.cosine_gain)
        self._write_ic(VOSS_M, master.sine_offset)
        self._write_ic(VOSC_M, master.cosine_offset)
        self._write_ic(PH_M, master.phase)
        self._write_ic(GX_N, nonius.cosine_gain)
        self._write_ic(VOSS_N, nonius.sine_offset)
        self._write_ic(VOSC_N, nonius.cosine_offset)
        self._write_ic(PH_N, nonius.phase)

    def reset_analog_to_factory_defaults(self) -> None:
        """Reset analog parameters to iC-MU Series factory defaults.

        Per the MU_Series datasheet (Rev B1), factory defaults are:
            GX = 0x00 (0% cosine gain correction)
            VOSS = 0x3F (sine offset ~60-70 mV)
            VOSC = 0x3F (cosine offset ~60-70 mV)
            PH = 0x3F (phase baseline)

        This is useful when the current chip state is unknown or corrupted,
        providing a sensible starting point for calibration iteration.
        """
        defaults = mu_3sl.AnalogTrackAdjustments(
            _FACTORY_DEFAULT_GX,
            _FACTORY_DEFAULT_VOSS,
            _FACTORY_DEFAULT_VOSC,
            _FACTORY_DEFAULT_PH,
        )
        self.write_analog_adjustments(defaults, defaults)
        logger.info(
            f"Encoder {self._number}: reset analog parameters to factory defaults"
            f" (GX=0x{_FACTORY_DEFAULT_GX:02X}, VOSS=0x{_FACTORY_DEFAULT_VOSS:02X},"
            f" VOSC=0x{_FACTORY_DEFAULT_VOSC:02X}, PH=0x{_FACTORY_DEFAULT_PH:02X})",
        )

    # -- Nonius (SPO) parameters --

    def write_nonius_parameters(
        self,
        table_params: mu_3sl.NoniusTrackOffsetTableParameters,
    ) -> None:
        """Write nonius track offset table to chip.

        Args:
            table_params: SPO_BASE + SPO_0..SPO_14 from the DLL.
        """
        spo_base = table_params.spo_base
        spo_n = [table_params.spo_n[i] for i in range(15)]

        # First register (0x52): spo_base[3:0] + spo_0[7:4]
        reg = SPO_REGISTERS[0]
        val = reg.field("spo_base").insert(0, spo_base)
        val = reg.field("spo_0").insert(val, spo_n[0])
        self._write_ic(reg, val)

        # Remaining registers (0x53-0x59): each packs two 4-bit SPO values --
        # odd-indexed in the low nibble [3:0], even-indexed in the high [7:4].
        for i, reg in enumerate(SPO_REGISTERS[1:]):
            odd = 2 * i + 1
            even = 2 * i + 2
            val = reg.field(f"spo_{odd}").insert(0, spo_n[odd])
            val = reg.field(f"spo_{even}").insert(val, spo_n[even])
            self._write_ic(reg, val)

    # -- EEPROM --

    def save_to_eeprom(self) -> None:
        """Issue WRITE_ALL to save configuration to iC-MU EEPROM.

        Raises:
            RuntimeError: If EEPROM write fails (EPR_ERR or CRC_ERR).
        """
        self._write_ic(CMD, _CMD_WRITE_ALL)
        time.sleep(1.0)
        if self._read_ic_field(STATUS1, STATUS1.field("epr_err")):
            msg = f"Encoder {self._number}: EEPROM write error (EPR_ERR)."
            raise RuntimeError(msg)
        if self._read_ic_field(STATUS1, STATUS1.field("crc_err")):
            msg = f"Encoder {self._number}: CRC error after EEPROM write."
            raise RuntimeError(msg)
        logger.info(f"Encoder {self._number}: EEPROM saved successfully.")

    def enable_all_errors(self) -> None:
        """Set CFGEW=0x00 so all error/warning sources are visible."""
        self._write_ic(CFGEW, 0x00)
        logger.info(f"Encoder {self._number}: all errors enabled (CFGEW=0x00).")

    def abs_reset(self) -> None:
        """Issue an ABS_RESET command to clear a startup NON_CTR error.

        After power-on the iC-MU may report a nonius consistency error
        (NON_CTR bit in STATUS1) because the period counter has not yet
        been synchronised with the nonius position.  ABS_RESET forces a
        fresh absolute-position calculation which clears the flag.
        """
        self._write_ic(CMD, _CMD_ABS_RESET)
        # Let the BiSS transaction settle and the encoder recalculate absolute
        # position before any further operations.
        time.sleep(_BISS_SETTLE_S)
        logger.info(f"Encoder {self._number}: ABS_RESET issued.")
