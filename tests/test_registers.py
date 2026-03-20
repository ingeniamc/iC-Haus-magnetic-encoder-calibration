import pytest

from ic_haus_magnetic_encoder_calibration.drive_encoder_registers import (
    ENCODER_1_REGS,
    ENCODER_2_REGS,
    get_encoder_registers,
)
from ic_haus_magnetic_encoder_calibration.ic_haus_registers import (
    ICHausRegisterField,
)


class TestICHausRegisterField:
    """Bit manipulation is the core of register access — must be correct."""

    def test_extract_isolates_field(self) -> None:
        f = ICHausRegisterField(mask=0x80, shift=7)
        assert f.extract(0xFF) == 1
        assert f.extract(0x7F) == 0

    def test_insert_preserves_other_bits(self) -> None:
        f = ICHausRegisterField(mask=0x0F, shift=0)
        assert f.insert(0xF0, 0x05) == 0xF5

    def test_insert_clears_old_value(self) -> None:
        f = ICHausRegisterField(mask=0x80, shift=7)
        assert f.insert(0xFF, 0) == 0x7F

    def test_from_bits_computes_mask_and_shift(self) -> None:
        f = ICHausRegisterField.from_bits(low=4, high=5)
        assert f.mask == 0x30
        assert f.shift == 4


class TestGetEncoderRegisters:
    def test_invalid_raises(self) -> None:
        with pytest.raises(ValueError, match="must be 1 or 2"):
            get_encoder_registers(3)


class TestDriveEncoderRegistersFields:
    def test_encoder_1_has_biss1_prefix(self) -> None:
        r = ENCODER_1_REGS
        assert "BISS1" in r.itf_addr
        assert "BISS1" in r.itf_data
        assert "BISS1" in r.itf_ctl
        assert "BISS1" in r.pos_value

    def test_encoder_2_has_biss2_prefix(self) -> None:
        r = ENCODER_2_REGS
        assert "BISS2" in r.itf_addr
        assert "BISS2" in r.itf_data
        assert "BISS2" in r.itf_ctl
        assert "BISS2" in r.pos_value
