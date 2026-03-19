"""iC-Haus Magnetic Encoder Calibration library."""

from __future__ import annotations

from .calibrator import EncoderCalibrator
from .drive_encoder_registers import DriveEncoderRegisters, get_encoder_registers
from .encoder import CalibrationResult, DriveFrameConfig, Encoder, ICMURegisterState
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
]
