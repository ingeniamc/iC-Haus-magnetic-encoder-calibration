import pytest

from ic_haus_magnetic_encoder_calibration.drive_encoder_registers import (
    ENCODER_1_REGS,
    ENCODER_2_REGS,
    get_encoder_registers,
)
from ic_haus_magnetic_encoder_calibration.ic_haus_registers import (
    BissAction,
    ICHausRegister,
    ICHausRegisterField,
)


class TestICHausRegisterField:
    def test_extract_single_bit(self) -> None:
        f = ICHausRegisterField(mask=0x80, shift=7)
        assert f.extract(0xFF) == 1
        assert f.extract(0x7F) == 0

    def test_extract_nibble(self) -> None:
        f = ICHausRegisterField(mask=0x0F, shift=0)
        assert f.extract(0xAB) == 0x0B

    def test_extract_high_nibble(self) -> None:
        f = ICHausRegisterField(mask=0xF0, shift=4)
        assert f.extract(0xAB) == 0x0A

    def test_insert_replaces_only_masked_bits(self) -> None:
        f = ICHausRegisterField(mask=0x0F, shift=0)
        assert f.insert(0xF0, 0x05) == 0xF5

    def test_insert_high_field(self) -> None:
        f = ICHausRegisterField(mask=0xE0, shift=5)
        assert f.insert(0x1F, 0x03) == 0x7F

    def test_insert_preserves_other_bits(self) -> None:
        f = ICHausRegisterField(mask=0x80, shift=7)
        assert f.insert(0x7F, 1) == 0xFF
        assert f.insert(0xFF, 0) == 0x7F

    def test_name_attribute(self) -> None:
        f = ICHausRegisterField(mask=0x80, shift=7, name="Test field")
        assert f.name == "Test field"

    def test_name_defaults_to_empty(self) -> None:
        f = ICHausRegisterField(mask=0x80, shift=7)
        assert f.name == ""

    def test_from_bits_single_bit(self) -> None:
        f = ICHausRegisterField.from_bits(low=7, high=7, name="bit 7")
        assert f.mask == 0x80
        assert f.shift == 7
        assert f.name == "bit 7"

    def test_from_bits_nibble(self) -> None:
        f = ICHausRegisterField.from_bits(low=0, high=3)
        assert f.mask == 0x0F
        assert f.shift == 0

    def test_from_bits_high_nibble(self) -> None:
        f = ICHausRegisterField.from_bits(low=4, high=7)
        assert f.mask == 0xF0
        assert f.shift == 4

    def test_from_bits_mid_range(self) -> None:
        f = ICHausRegisterField.from_bits(low=4, high=5)
        assert f.mask == 0x30
        assert f.shift == 4


class TestBissAction:
    def test_values(self) -> None:
        assert BissAction.NO_ACTION == 0
        assert BissAction.READ == 1
        assert BissAction.WRITE == 2

    def test_is_int(self) -> None:
        assert isinstance(BissAction.READ, int)


class TestICHausRegister:
    def test_address(self) -> None:
        r = ICHausRegister(address=0x42)
        assert r.address == 0x42

    def test_name_attribute(self) -> None:
        r = ICHausRegister(address=0x01, name="Cosine gain master")
        assert r.name == "Cosine gain master"

    def test_name_defaults_to_empty(self) -> None:
        r = ICHausRegister(address=0x01)
        assert r.name == ""

    def test_field_access(self) -> None:
        r = ICHausRegister(
            address=0x11,
            name="Output",
            out_msb=ICHausRegisterField(0x1F, 0),
            out_zero=ICHausRegisterField(0xE0, 5),
        )
        assert r.field("out_msb").mask == 0x1F
        assert r.field("out_zero").shift == 5

    def test_field_names(self) -> None:
        r = ICHausRegister(
            address=0x11,
            name="Output",
            out_msb=ICHausRegisterField(0x1F, 0),
            out_zero=ICHausRegisterField(0xE0, 5),
        )
        assert set(r.field_names) == {"out_msb", "out_zero"}

    def test_field_missing_raises(self) -> None:
        r = ICHausRegister(address=0x01)
        with pytest.raises(KeyError):
            r.field("nonexistent")


class TestGetEncoderRegisters:
    def test_encoder_1(self) -> None:
        assert get_encoder_registers(1) is ENCODER_1_REGS

    def test_encoder_2(self) -> None:
        assert get_encoder_registers(2) is ENCODER_2_REGS

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

    def test_frozen(self) -> None:
        with pytest.raises(AttributeError):
            ENCODER_1_REGS.itf_addr = "changed"  # type: ignore[misc]
