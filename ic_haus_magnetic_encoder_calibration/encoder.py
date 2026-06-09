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
from typing import Optional

import mu_3sl_interface as mu_3sl
from ingeniamotion import MotionController
from ingeniamotion.enums import SensorType

from .drive_encoder_registers import (
    CALIB_ERROR_TOLERANCE,
    CALIB_FRAME_SIZE,
    CALIB_POS_BITS,
    CALIB_POS_ST_BITS,
    CALIB_POS_START_BIT,
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
)

logger = logging.getLogger(__name__)

_BISS_SETTLE_S = 0.1

# Calibration field values for iC-MU registers
_CALIB_OUT_MSB = 0x0E
_CALIB_OUT_LSB = 0x00
_CALIB_OUT_ZERO_BISS = 0x00
_CALIB_MODE_ST_RAW = 0x02
_CALIB_MODEA_BISS = 0x02
_CALIB_TEST = 0x00

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
    pos_bits: int
    pos_st_bits: int
    pos_start_bit: int
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
    """

    def __init__(
        self,
        mc: MotionController,
        sensor_type: SensorType,
        *,
        axis: int = 1,
    ) -> None:
        self._mc = mc
        self._sensor_type = sensor_type
        self._number = _SENSOR_TYPE_TO_ENCODER[sensor_type]
        self._axis = axis
        self._regs = get_encoder_registers(self._number)

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

    def _write_ic(self, reg: ICHausRegister, value: int) -> None:
        """Write an 8-bit value to an iC-MU register via BiSS bidirectional."""
        regs = self._regs
        ax = self._axis
        self._mc.communication.set_register(regs.itf_ctl, BissAction.NO_ACTION, axis=ax)
        self._mc.communication.set_register(regs.itf_addr, reg.address, axis=ax)
        self._mc.communication.set_register(regs.itf_data, value & 0xFF, axis=ax)
        self._mc.communication.set_register(regs.itf_ctl, BissAction.WRITE, axis=ax)
        # Let the BiSS transaction settle before any subsequent operations.
        # They could potentially depend on the new register value, so it's safer to wait here
        time.sleep(_BISS_SETTLE_S)

    # -- Step 1: Revision --

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
            pos_bits=self._read_drive(r.pos_bits),
            pos_st_bits=self._read_drive(r.pos_st_bits),
            pos_start_bit=self._read_drive(r.pos_start_bit),
            error_tolerance=self._read_drive(r.error_tolerance),
        )

    def set_drive_config(self, config: DriveFrameConfig) -> None:
        """Write drive encoder frame configuration.

        Args:
            config: Frame settings to apply.
        """
        r = self._regs
        self._write_drive(r.frame_size, config.frame_size)
        self._write_drive(r.pos_bits, config.pos_bits)
        self._write_drive(r.pos_st_bits, config.pos_st_bits)
        self._write_drive(r.pos_start_bit, config.pos_start_bit)
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

    # -- Calibration mode --

    def ensure_normal_mode(self) -> bool:
        """Check if the encoder is in normal ABS mode and fix it if not.

        A previous interrupted calibration may leave the iC-MU in RAW
        mode with an enlarged output, causing the drive to receive
        frames that don't match its expected frame size.

        Returns:
            True if the encoder was already in normal mode, False if a
            fix was applied.
        """
        out_msb = OUT_MSB_ZERO.field("out_msb").extract(self._read_ic(OUT_MSB_ZERO))
        mode_st = OUT_LSB_ST.field("mode_st").extract(self._read_ic(OUT_LSB_ST))
        normal_out_msb = 0x06
        normal_mode_st = 0x00  # ABS

        if out_msb == normal_out_msb and mode_st == normal_mode_st:
            return True

        logger.warning(
            f"Encoder {self._number}: not in normal mode"
            f" (OUT_MSB=0x{out_msb:02X}, MODE_ST={mode_st});"
            f" restoring from previous interrupted calibration.",
        )
        # Restore to absolute mode with EEPROM-default output width
        lsb_raw = self._read_ic(OUT_LSB_ST)
        lsb_raw = OUT_LSB_ST.field("out_lsb").insert(lsb_raw, 0)
        lsb_raw = OUT_LSB_ST.field("mode_st").insert(lsb_raw, normal_mode_st)
        self._write_ic(OUT_LSB_ST, lsb_raw)

        msb_raw = self._read_ic(OUT_MSB_ZERO)
        msb_raw = OUT_MSB_ZERO.field("out_msb").insert(msb_raw, normal_out_msb)
        self._write_ic(OUT_MSB_ZERO, msb_raw)

        # Disable analog calibration enable bit
        enac_raw = self._read_ic(ENAC)
        enac_raw = ENAC.field("enac").insert(enac_raw, 0)
        self._write_ic(ENAC, enac_raw)
        # Suppress all errors during recovery
        self._write_ic(CFGEW, _CALIB_CFGEW_SUPPRESS)
        return False

    def configure_in_calibration_mode(self) -> int:
        """Configure iC-MU registers and drive frame for calibration.

        Reads the current iC-MU state, writes calibration values and
        sets the drive encoder frame for raw data capture.  Use
        ``get_ic_config`` / ``get_drive_config`` before calling this
        method if you need to restore the original state later.

        Returns:
            Number of master periods (2^MPC).
        """
        enac_orig = self._read_ic(ENAC)
        modea_orig = self._read_ic(MODEA_MODEB)
        out_orig = self._read_ic(OUT_MSB_ZERO)
        lsb_orig = self._read_ic(OUT_LSB_ST)
        test_orig = self._read_ic(TEST)
        mpc_orig = self._read_ic(MPC)

        # Enable analog calibration (ENAC bit)
        enac_new = ENAC.field("enac").insert(enac_orig, 1)
        if enac_new != enac_orig:
            self._write_ic(ENAC, enac_new)

        # Set interface mode to BiSS
        modea_new = MODEA_MODEB.field("modea").insert(modea_orig, _CALIB_MODEA_BISS)
        if modea_new != modea_orig:
            self._write_ic(MODEA_MODEB, modea_new)

        # Configure output shift register length:
        # OUT_MSB=0x0E selects bit 27 as the MSB (14 master + 14 nonius = 28 bits)
        # OUT_ZERO=0x00 — no padding zeros (BiSS doesn't need them; SPI uses 0x04)
        out = OUT_MSB_ZERO.field("out_zero").insert(out_orig, _CALIB_OUT_ZERO_BISS)
        out = OUT_MSB_ZERO.field("out_msb").insert(out, _CALIB_OUT_MSB)
        if out != out_orig:
            self._write_ic(OUT_MSB_ZERO, out)

        # Select raw master+nonius track output:
        # MODE_ST=0x02 selects raw analog data (required for calibration)
        # OUT_LSB=0x00 starts output from bit 0 (no truncation)
        lsb = OUT_LSB_ST.field("mode_st").insert(lsb_orig, _CALIB_MODE_ST_RAW)
        lsb = OUT_LSB_ST.field("out_lsb").insert(lsb, _CALIB_OUT_LSB)
        if lsb != lsb_orig:
            self._write_ic(OUT_LSB_ST, lsb)

        # Clear test register
        if test_orig != _CALIB_TEST:
            self._write_ic(TEST, _CALIB_TEST)

        # Suppress all ERR/WRN sources so the drive doesn't fault
        # on error bits from the (still uncalibrated) encoder.
        self._write_ic(CFGEW, _CALIB_CFGEW_SUPPRESS)

        # MPC: if 0x0C set to 0x0B (per iC-Haus AN1 "Offline Calibration", Table 1)
        mpc_val = MPC.field("mpc").extract(mpc_orig)
        if mpc_val == 0x0C:
            new_mpc = MPC.field("mpc").insert(mpc_orig, 0x0B)
            self._write_ic(MPC, new_mpc)
            mpc_val = 0x0B

        # Configure drive frame for calibration
        r = self._regs
        self._write_drive(r.frame_size, CALIB_FRAME_SIZE)
        self._write_drive(r.pos_bits, CALIB_POS_BITS)
        self._write_drive(r.pos_st_bits, CALIB_POS_ST_BITS)
        self._write_drive(r.pos_start_bit, CALIB_POS_START_BIT)

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
        status = self._read_ic(STATUS1)
        if STATUS1.field("epr_err").extract(status):
            msg = f"Encoder {self._number}: EEPROM write error (EPR_ERR)."
            raise RuntimeError(msg)
        if STATUS1.field("crc_err").extract(status):
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
