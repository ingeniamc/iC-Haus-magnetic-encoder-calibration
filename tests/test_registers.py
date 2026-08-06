import pytest

from ic_haus_magnetic_encoder_calibration.drive_encoder_registers import (
    CALIB_FRAME_SIZE,
    CALIB_POS_BITS,
    CALIB_POS_ST_BITS,
    CALIB_POS_START_BIT,
    ENCODER_1_REGS,
    ENCODER_2_REGS,
    get_encoder_registers,
)
from ic_haus_magnetic_encoder_calibration.ic_haus_registers import (
    OUT_MSB_ZERO,
    ICHausRegisterField,
)


class TestICHausRegisterField:
    """Bit manipulation is the core of register access -- must be correct."""

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
        assert "BISS1" in r.protocol
        assert "BISS1" in r.itf_addr
        assert "BISS1" in r.itf_data
        assert "BISS1" in r.itf_ctl
        assert "BISS1" in r.pos_value
        assert "BISS1" in r.frame_size
        assert "BISS1" in r.frame_type
        assert "BISS1" in r.pos_bits
        assert "BISS1" in r.pos_st_bits
        assert "BISS1" in r.pos_start_bit
        assert "BISS1" in r.pos_offset
        assert "BISS1" in r.polarity
        assert "BISS1" in r.error_tolerance

    def test_encoder_2_has_biss2_prefix(self) -> None:
        r = ENCODER_2_REGS
        assert "BISS2" in r.itf_addr
        assert "BISS2" in r.itf_data
        assert "BISS2" in r.itf_ctl

    def test_encoder_2_uses_ssi2_for_position(self) -> None:
        r = ENCODER_2_REGS
        assert "SSI2" in r.protocol
        assert "SSI2" in r.pos_value
        assert "SSI2" in r.frame_size
        assert "SSI2" in r.frame_type
        assert "SSI2" in r.pos_bits
        assert "SSI2" in r.pos_st_bits
        assert "SSI2" in r.pos_start_bit
        assert "SSI2" in r.pos_offset
        assert "SSI2" in r.polarity
        assert "SSI2" in r.error_tolerance

    def test_no_register_name_is_shared_between_channels(self) -> None:
        """A copy/paste slip between channels would silently calibrate the wrong encoder."""
        names_1 = set(vars(ENCODER_1_REGS).values())
        names_2 = set(vars(ENCODER_2_REGS).values())
        assert names_1 & names_2 == set()


class TestOutMsbZeroFields:
    def test_out_msb_and_out_zero_do_not_overlap(self) -> None:
        msb = OUT_MSB_ZERO.field("out_msb")
        zero = OUT_MSB_ZERO.field("out_zero")
        assert msb.mask == 0x1F  # bits 0:4
        assert zero.mask == 0xE0  # bits 5:7
        assert msb.mask & zero.mask == 0


class TestCalibrationFrameConstants:
    """The raw BiSS-C frame is 28 position bits + 8 trailing bits (CRC6 + ERR + WRN)."""

    def test_frame_geometry_is_consistent(self) -> None:
        assert CALIB_POS_BITS == CALIB_POS_ST_BITS == 28
        assert CALIB_FRAME_SIZE == CALIB_POS_ST_BITS + CALIB_POS_START_BIT
        assert CALIB_POS_START_BIT == 8
