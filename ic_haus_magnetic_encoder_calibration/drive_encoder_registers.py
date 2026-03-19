"""DR3256C drive encoder register name mappings.

Each ``DriveEncoderRegisters`` instance stores the dictionary register
name strings that the Ingenia drive uses to expose one BiSS encoder
channel.  These are the keys used with
``MotionController.communication.{get,set}_register``.
"""

from __future__ import annotations

from dataclasses import dataclass

# Drive encoder frame settings for calibration mode
CALIB_FRAME_SIZE = 36
CALIB_POS_BITS = 28
CALIB_POS_ST_BITS = 28
CALIB_POS_START_BIT = 8


@dataclass(frozen=True)
class DriveEncoderRegisters:
    """Drive register names for a single BiSS encoder channel."""

    itf_addr: str
    itf_data: str
    itf_ctl: str
    pos_value: str
    frame_size: str
    pos_bits: str
    pos_st_bits: str
    pos_start_bit: str


ENCODER_1_REGS = DriveEncoderRegisters(
    itf_addr="FBK_BISS1_SSI1_ITF_ADDR",
    itf_data="FBK_BISS1_SSI1_ITF_DATA",
    itf_ctl="FBK_BISS1_SSI1_ITF_CTL",
    pos_value="FBK_BISS1_SSI1_POS_VALUE",
    frame_size="FBK_BISS1_SSI1_FRAME_SIZE",
    pos_bits="FBK_BISS1_SSI1_POS_BITS",
    pos_st_bits="FBK_BISS1_SSI1_POS_ST_BITS",
    pos_start_bit="FBK_BISS1_SSI1_POS_START_BIT",
)

ENCODER_2_REGS = DriveEncoderRegisters(
    itf_addr="FBK_BISS2_SSI2_ITF_ADDR",
    itf_data="FBK_BISS2_SSI2_ITF_DATA",
    itf_ctl="FBK_BISS2_SSI2_ITF_CTL",
    pos_value="FBK_BISS2_POS_VALUE",
    frame_size="FBK_BISS2_FRAME_SIZE",
    pos_bits="FBK_BISS2_POS_BITS",
    pos_st_bits="FBK_BISS2_POS_ST_BITS",
    pos_start_bit="FBK_BISS2_POS_START_BIT",
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
