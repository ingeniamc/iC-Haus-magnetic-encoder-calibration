import pytest
from ingeniamotion.enums import SensorType

from ic_haus_magnetic_encoder_calibration.config_loader import EncoderRegisterConfig
from ic_haus_magnetic_encoder_calibration.drive_encoder_registers import (
    CALIB_ERROR_TOLERANCE,
    CALIB_FRAME_SIZE,
    CALIB_FRAME_TYPE,
    CALIB_POLARITY,
    CALIB_POS_BITS,
    CALIB_POS_OFFSET,
    CALIB_POS_ST_BITS,
    CALIB_POS_START_BIT,
    CALIB_PROTOCOL,
    ENCODER_1_REGS,
)
from ic_haus_magnetic_encoder_calibration.encoder import (
    Encoder,
)
from ic_haus_magnetic_encoder_calibration.ic_haus_registers import (
    GX_M,
    HARD_REV,
    BissAction,
)


@pytest.fixture
def encoder_config():
    return EncoderRegisterConfig(
        out_msb=0x06,
        out_lsb=0x00,
        mode_st=0x00,
        enac=0x01,
        cfgew=0x00,
        filt=0x02,
    )


@pytest.fixture
def encoder(mock_mc, encoder_config):
    return Encoder(
        mock_mc,
        sensor_type=SensorType.ABS1,
        axis=1,
        config=encoder_config,
    )


class TestBissReadWrite:
    """BiSS protocol sequence must be exact: CTL=0, ADDR, CTL=READ/WRITE."""

    def test_read_ic_follows_biss_protocol(self, encoder, mock_mc) -> None:
        mock_mc.communication.get_register.return_value = 0x42
        regs = ENCODER_1_REGS

        result = encoder._read_ic(HARD_REV)

        assert result == 0x42
        calls = mock_mc.communication.set_register.call_args_list
        assert calls[0].args == (regs.itf_ctl, BissAction.NO_ACTION)
        assert calls[1].args == (regs.itf_addr, HARD_REV.address)
        assert calls[2].args == (regs.itf_ctl, BissAction.READ)

    def test_read_ic_masks_to_8_bits(self, encoder, mock_mc) -> None:
        mock_mc.communication.get_register.return_value = 0x1FF
        assert encoder._read_ic(GX_M) == 0xFF

    def test_write_ic_masks_value(self, encoder, mock_mc) -> None:
        encoder._write_ic(GX_M, 0x1FF)
        data_call = mock_mc.communication.set_register.call_args_list[2]
        assert data_call.args[1] == 0xFF


class TestReadRevision:
    def test_raises_for_none_revision(self, encoder, mock_mc, mocker) -> None:
        mu_3sl = mocker.patch("ic_haus_magnetic_encoder_calibration.encoder.mu_3sl")
        mock_mc.communication.get_register.return_value = 0x00
        none_rev = mocker.MagicMock()
        mu_3sl.Revision.return_value = none_rev
        mu_3sl.Revision.NONE = none_rev

        with pytest.raises(RuntimeError, match="unable to read revision"):
            encoder.read_revision()


class TestConfigureInCalibrationMode:
    @staticmethod
    def _drive_writes(mock_mc) -> dict:
        """Collect drive-register writes as {register_name: value}.

        iC-MU writes go through itf_data, so filter them out by register name.

        Returns:
            Mapping of drive register name to the value written.
        """
        ic_regs = {ENCODER_1_REGS.itf_ctl, ENCODER_1_REGS.itf_addr, ENCODER_1_REGS.itf_data}
        return {
            c.args[0]: c.args[1]
            for c in mock_mc.communication.set_register.call_args_list
            if c.args[0] not in ic_regs
        }

    def test_mpc_0x0c_reduced_to_0x0b(self, encoder, mock_mc) -> None:
        """MPC=0x0C is a special case that must be reduced to 0x0B."""
        mock_mc.communication.get_register.side_effect = [
            0x80,  # ENAC already set
            0x02,  # MODEA already BiSS
            0x0E,  # OUT_MSB_ZERO read (out_msb write)
            0x0E,  # OUT_MSB_ZERO read (out_zero write)
            0x20,  # OUT_LSB_ST read (mode_st write)
            0x20,  # OUT_LSB_ST read (out_lsb write)
            0x0C,  # MPC read (= 0x0C, triggers reduction)
            0x0C,  # MPC read-modify-write (reduce to 0x0B)
        ]

        n_periods = encoder.configure_in_calibration_mode()

        assert n_periods == 2048

    def test_writes_all_calibration_frame_registers(self, encoder, mock_mc) -> None:
        """All 8 frame registers must be programmed -- a missing one leaves the drive
        decoding the raw payload with the previous (non-raw) geometry. DR3256AC-937."""
        mock_mc.communication.get_register.return_value = 0x0B  # MPC already valid
        r = ENCODER_1_REGS

        encoder.configure_in_calibration_mode()

        writes = self._drive_writes(mock_mc)
        assert writes == {
            r.frame_size: CALIB_FRAME_SIZE,
            r.frame_type: CALIB_FRAME_TYPE,
            r.pos_bits: CALIB_POS_BITS,
            r.pos_st_bits: CALIB_POS_ST_BITS,
            r.pos_start_bit: CALIB_POS_START_BIT,
            r.pos_offset: CALIB_POS_OFFSET,
            r.polarity: CALIB_POLARITY,
            r.error_tolerance: CALIB_ERROR_TOLERANCE,
        }

    def test_error_tolerance_written_after_frame_geometry(self, encoder, mock_mc) -> None:
        """Tolerance must be raised *after* the geometry change, otherwise the drive
        can freeze POS_VALUE on the transient CRC mismatch it causes."""
        mock_mc.communication.get_register.return_value = 0x0B
        r = ENCODER_1_REGS

        encoder.configure_in_calibration_mode()

        order = list(self._drive_writes(mock_mc))
        assert order[-1] == r.error_tolerance

    def test_mpc_below_0x0c_is_left_untouched(self, encoder, mock_mc) -> None:
        """Only MPC=0x0C is illegal for raw capture; other values must be preserved."""
        mock_mc.communication.get_register.return_value = 0x0A  # MPC = 10

        n_periods = encoder.configure_in_calibration_mode()

        assert n_periods == 1024  # 2 ** 10, i.e. MPC was not rewritten


class TestSaveToEeprom:
    def test_success(self, encoder, mock_mc) -> None:
        mock_mc.communication.get_register.return_value = 0x00
        encoder.save_to_eeprom()  # should not raise

    def test_epr_error(self, encoder, mock_mc) -> None:
        mock_mc.communication.get_register.return_value = 0x40
        with pytest.raises(RuntimeError, match="EPR_ERR"):
            encoder.save_to_eeprom()

    def test_crc_error(self, encoder, mock_mc) -> None:
        mock_mc.communication.get_register.return_value = 0x80
        with pytest.raises(RuntimeError, match="CRC error"):
            encoder.save_to_eeprom()


class TestGetICConfig:
    """Tests for iC-MU config save/restore data integrity."""

    def test_roundtrip_preserves_values(self, encoder, mock_mc) -> None:
        """get_ic_config -> set_ic_config writes back the exact same register bytes."""
        mock_mc.communication.get_register.side_effect = [
            0x80,
            0x02,
            0xCE,
            0x20,
            0x00,
            0x05,
            0x00,
        ]
        state = encoder.get_ic_config()

        mock_mc.communication.set_register.reset_mock()
        encoder.set_ic_config(state)

        data_writes = [
            c.args[1]
            for c in mock_mc.communication.set_register.call_args_list
            if c.args[0] == ENCODER_1_REGS.itf_data
        ]
        assert data_writes == [0x80, 0x02, 0xCE, 0x20, 0x00, 0x05, 0x00]


class TestGetDriveConfig:
    """Drive frame save/restore data integrity (mocked -- runs in CI)."""

    def test_roundtrip_preserves_values(self, encoder, mock_mc) -> None:
        """get_drive_config -> set_drive_config writes back the exact same values,
        in the same register order, for all 8 fields."""
        # Distinct values so a field swapped with its neighbour is detectable.
        mock_mc.communication.get_register.side_effect = [
            36,  # frame_size
            0,  # frame_type
            28,  # pos_bits
            27,  # pos_st_bits
            8,  # pos_start_bit
            1,  # pos_offset
            1,  # polarity
            0x1234,  # error_tolerance
        ]
        config = encoder.get_drive_config()

        mock_mc.communication.set_register.reset_mock()
        encoder.set_drive_config(config)

        r = ENCODER_1_REGS
        writes = [(c.args[0], c.args[1]) for c in mock_mc.communication.set_register.call_args_list]
        assert writes == [
            (r.frame_size, 36),
            (r.frame_type, 0),
            (r.pos_bits, 28),
            (r.pos_st_bits, 27),
            (r.pos_start_bit, 8),
            (r.pos_offset, 1),
            (r.polarity, 1),
            (r.error_tolerance, 0x1234),
        ]


class TestIsBissc:
    def test_true_for_bissc_protocol(self, encoder, mock_mc) -> None:
        """Protocol register value 0 identifies BiSS-C."""
        mock_mc.communication.get_register.return_value = CALIB_PROTOCOL
        assert encoder.is_bissc is True

    def test_false_for_other_protocol(self, encoder, mock_mc) -> None:
        """Any other protocol (e.g. SSI) must not be treated as BiSS-C."""
        mock_mc.communication.get_register.return_value = CALIB_PROTOCOL + 1
        assert encoder.is_bissc is False


# ---------------------------------------------------------------------------
#  Hardware integration tests (require a physical drive)
# ---------------------------------------------------------------------------


@pytest.fixture
def hw_encoder(mc, encoder_config):
    """Create an Encoder using the real MotionController.

    Returns:
        An Encoder configured for encoder 1 on axis 1.
    """
    return Encoder(mc, sensor_type=SensorType.ABS1, axis=1, config=encoder_config)


@pytest.mark.hardware
class TestReadRevisionHardware:
    def test_returns_valid_revision(self, hw_encoder) -> None:
        revision = hw_encoder.read_revision()
        assert revision is not None


@pytest.mark.hardware
class TestDriveConfigHardware:
    def test_get_and_set_roundtrip(self, hw_encoder) -> None:
        original = hw_encoder.get_drive_config()
        hw_encoder.set_drive_config(original)
        restored = hw_encoder.get_drive_config()
        assert restored == original


@pytest.mark.hardware
class TestICConfigHardware:
    def test_get_and_set_roundtrip(self, hw_encoder) -> None:
        original = hw_encoder.get_ic_config()
        hw_encoder.set_ic_config(original)
        restored = hw_encoder.get_ic_config()
        assert restored == original


@pytest.mark.hardware
class TestCalibrationModeHardware:
    def test_configure_returns_positive_periods(self, hw_encoder) -> None:
        drive_config = hw_encoder.get_drive_config()
        ic_state = hw_encoder.get_ic_config()
        try:
            n_periods = hw_encoder.configure_in_calibration_mode()
            assert n_periods > 0
        finally:
            hw_encoder.set_ic_config(ic_state)
            hw_encoder.set_drive_config(drive_config)


@pytest.mark.hardware
class TestAnalogAdjustmentsHardware:
    def test_read_returns_two_adjustments(self, hw_encoder) -> None:
        master, nonius = hw_encoder.read_analog_adjustments()
        assert master is not None
        assert nonius is not None


class TestApplyConfig:
    def test_out_msb_preserves_out_zero_bits(self, encoder, mock_mc) -> None:
        """OUT_MSB write must only touch bits[0:4] of OUT_MSB_ZERO (0x11),
        leaving out_zero (bits[5:7]) intact -- regression for DR3256AC-844."""
        cfg = EncoderRegisterConfig.from_dict({
            "OUT_MSB": "0x05",
            "OUT_LSB": "0x00",
            "MODE_ST": "0x00",
            "ENAC": "0x00",
            "CFGEW": "0x00",
            "FILT": "0x00",
        })
        encoder._config = cfg
        # OUT_MSB_ZERO currently reads 0xE0 (out_zero = 0b111 set)
        mock_mc.communication.get_register.return_value = 0xE0

        encoder.apply_config()

        # Find the value written to OUT_MSB_ZERO
        writes = [
            c.args[1]
            for c in mock_mc.communication.set_register.call_args_list
            if c.args[0] == ENCODER_1_REGS.itf_data
        ]
        # out_zero (0xE0) preserved, out_msb set to 0x05 -> 0xE5
        assert 0xE5 in writes
