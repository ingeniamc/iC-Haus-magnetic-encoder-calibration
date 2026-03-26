"""iC-Haus Magnetic Encoder Calibration library."""

from .calibrator import EncoderCalibrator
from .drive_encoder_registers import DriveEncoderRegisters, get_encoder_registers
from .encoder import (
    CalibrationResult,
    DriveFrameConfig,
    Encoder,
    ICMURegisterState,
    split_raw_payload,
)
from .ic_haus_registers import BissAction, ICHausRegister, ICHausRegisterField

__all__ = [
    "BissAction",
    "CalibrationResult",
    "DriveEncoderRegisters",
    "DriveFrameConfig",
    "Encoder",
    "EncoderCalibrator",
    "ICHausRegister",
    "ICHausRegisterField",
    "ICMURegisterState",
    "get_encoder_registers",
    "split_raw_payload",
]
