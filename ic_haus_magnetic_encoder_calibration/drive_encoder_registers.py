"""DR3256C drive encoder register name mappings.

Each ``DriveEncoderRegisters`` instance stores the dictionary register
name strings that the Ingenia drive uses to expose one BiSS encoder
channel.  These are the keys used with
``MotionController.communication.{get,set}_register``.
"""

from dataclasses import dataclass

# Drive encoder frame settings for calibration raw mode.
# In raw mode the iC-MU outputs 28 position bits (14 master + 14 nonius)
# followed by ERR + WRN + 6-bit CRC (BiSS-C, poly 0x43) = 36 total SCD bits.
# POS_START_BIT=8 skips the 8 trailing bits (CRC6 + ERR + WRN).
CALIB_PROTOCOL = 0  # BiSS-C
CALIB_FRAME_SIZE = 36
CALIB_FRAME_TYPE = 0  # Raw
CALIB_POS_BITS = 28
CALIB_POS_ST_BITS = 28
CALIB_POS_START_BIT = 8
CALIB_POS_OFFSET = 0
CALIB_POLARITY = 0  # Standard polarity (0=normal, 1=inverted)
# Maximum error tolerance during calibration.  Changing the drive frame
# geometry may cause transient CRC mismatches; a high tolerance prevents
# the drive from freezing POS_VALUE during the transition.
CALIB_ERROR_TOLERANCE = 0xFFFF


@dataclass(frozen=True)
class DriveEncoderRegisters:
    """Drive register names for a single BiSS encoder channel."""

    protocol: str
    itf_addr: str
    itf_data: str
    itf_ctl: str
    pos_value: str
    frame_size: str
    frame_type: str
    pos_bits: str
    pos_st_bits: str
    pos_start_bit: str
    pos_offset: str
    polarity: str
    error_tolerance: str


ENCODER_1_REGS = DriveEncoderRegisters(
    protocol="FBK_BISS1_SSI1_PROTOCOL",
    itf_addr="FBK_BISS1_SSI1_ITF_ADDR",
    itf_data="FBK_BISS1_SSI1_ITF_DATA",
    itf_ctl="FBK_BISS1_SSI1_ITF_CTL",
    pos_value="FBK_BISS1_SSI1_POS_VALUE",
    frame_size="FBK_BISS1_SSI1_FRAME_SIZE",
    frame_type="FBK_BISS1_SSI1_FRAME_TYPE",
    pos_bits="FBK_BISS1_SSI1_POS_BITS",
    pos_st_bits="FBK_BISS1_SSI1_POS_ST_BITS",
    pos_start_bit="FBK_BISS1_SSI1_POS_START_BIT",
    pos_offset="FBK_BISS1_SSI1_OFFSET",
    polarity="FBK_BISS1_SSI1_POS_POLARITY",
    error_tolerance="FBK_BISS1_SSI1_ERROR_TOLERANCE",
)

ENCODER_2_REGS = DriveEncoderRegisters(
    protocol="FBK_SSI2_PROTOCOL",
    itf_addr="FBK_BISS2_SSI2_ITF_ADDR",
    itf_data="FBK_BISS2_SSI2_ITF_DATA",
    itf_ctl="FBK_BISS2_SSI2_ITF_CTL",
    pos_value="FBK_SSI2_POS_VALUE",
    frame_size="FBK_SSI2_FRAME_SIZE",
    frame_type="FBK_SSI2_FRAME_TYPE",
    pos_bits="FBK_SSI2_POS_BITS",
    pos_st_bits="FBK_SSI2_POS_ST_BITS",
    pos_start_bit="FBK_SSI2_POS_START_BIT",
    pos_offset="FBK_SSI2_OFFSET",
    polarity="FBK_SSI2_POS_POLARITY",
    error_tolerance="FBK_SSI2_ERROR_TOLERANCE",
)


def get_encoder_registers(encoder: int) -> DriveEncoderRegisters:
    """Return the register set for the given encoder number.

    Args:
        encoder: 1 or 2.

    Returns:
        DriveEncoderRegisters instance for the requested encoder.

    Raises:
        ValueError: If encoder is not 1 or 2.
    """
    if encoder == 1:
        return ENCODER_1_REGS
    if encoder == 2:
        return ENCODER_2_REGS
    msg = f"encoder must be 1 or 2, got {encoder}"
    raise ValueError(msg)
