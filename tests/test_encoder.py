import pytest

from ic_haus_magnetic_encoder_calibration.drive_encoder_registers import (
    ENCODER_1_REGS,
)
from ic_haus_magnetic_encoder_calibration.encoder import (
    Encoder,
)
from ic_haus_magnetic_encoder_calibration.ic_haus_registers import (
    GX_M,
    HARD_REV,
    OUT_MSB_ZERO,
    BissAction,
)


@pytest.fixture
def encoder(mock_mc):
    return Encoder(mock_mc, encoder_number=1, axis=1)


class TestBissReadWrite:
    """Tests for the Encoder._read_ic / _write_ic BiSS protocol."""

    def test_read_calls_biss_sequence(self, encoder, mock_mc) -> None:
        mock_mc.communication.get_register.return_value = 0x42
        regs = ENCODER_1_REGS

        result = encoder._read_ic(HARD_REV)

        assert result == 0x42
        calls = mock_mc.communication.set_register.call_args_list
        assert len(calls) == 3
        assert calls[0].args == (regs.itf_ctl, BissAction.NO_ACTION)
        assert calls[1].args == (regs.itf_addr, HARD_REV.address)
        assert calls[2].args == (regs.itf_ctl, BissAction.READ)

    def test_read_masks_to_8_bits(self, encoder, mock_mc) -> None:
        mock_mc.communication.get_register.return_value = 0x1FF
        result = encoder._read_ic(GX_M)
        assert result == 0xFF

    def test_write_calls_biss_sequence(self, encoder, mock_mc) -> None:
        regs = ENCODER_1_REGS
        encoder._write_ic(OUT_MSB_ZERO, 0xAB)

        calls = mock_mc.communication.set_register.call_args_list
        assert len(calls) == 4
        assert calls[0].args == (regs.itf_ctl, BissAction.NO_ACTION)
        assert calls[1].args == (regs.itf_addr, OUT_MSB_ZERO.address)
        assert calls[2].args == (regs.itf_data, 0xAB)
        assert calls[3].args == (regs.itf_ctl, BissAction.WRITE)

    def test_write_masks_value(self, encoder, mock_mc) -> None:
        encoder._write_ic(GX_M, 0x1FF)
        data_call = mock_mc.communication.set_register.call_args_list[2]
        assert data_call.args[1] == 0xFF


class TestReadRevision:
    def test_returns_revision(self, encoder, mock_mc, mocker) -> None:
        mu_3sl = mocker.patch("ic_haus_magnetic_encoder_calibration.encoder.mu_3sl")
        mock_mc.communication.get_register.return_value = 0x03
        mu_3sl.Revision.return_value = mocker.MagicMock(name="REV_C")
        mu_3sl.Revision.NONE = mocker.sentinel.NONE

        result = encoder.read_revision()

        mu_3sl.Revision.assert_called_with(0x03)
        assert result is not mocker.sentinel.NONE

    def test_raises_for_none_revision(self, encoder, mock_mc, mocker) -> None:
        mu_3sl = mocker.patch("ic_haus_magnetic_encoder_calibration.encoder.mu_3sl")
        mock_mc.communication.get_register.return_value = 0x00
        none_rev = mocker.MagicMock()
        mu_3sl.Revision.return_value = none_rev
        mu_3sl.Revision.NONE = none_rev

        with pytest.raises(RuntimeError, match="unable to read revision"):
            encoder.read_revision()


class TestDriveConfig:
    def test_get_drive_config_reads_four_registers(self, encoder, mock_mc) -> None:
        mock_mc.communication.get_register.side_effect = [100, 16, 16, 4]

        config = encoder.get_drive_config()

        assert config.frame_size == 100
        assert config.pos_bits == 16
        assert config.pos_st_bits == 16
        assert config.pos_start_bit == 4

    def test_set_drive_config_writes_back(self, encoder, mock_mc) -> None:
        r = ENCODER_1_REGS
        mock_mc.communication.get_register.side_effect = [100, 16, 16, 4]
        config = encoder.get_drive_config()

        mock_mc.communication.set_register.reset_mock()
        encoder.set_drive_config(config)

        calls = mock_mc.communication.set_register.call_args_list
        assert any(c.args == (r.frame_size, 100) for c in calls)
        assert any(c.args == (r.pos_bits, 16) for c in calls)


class TestConfigureInCalibrationMode:
    def test_returns_master_periods(self, encoder, mock_mc) -> None:
        mock_mc.communication.get_register.side_effect = [
            0x00,  # ENAC
            0x00,  # MODEA_MODEB
            0x00,  # OUT_MSB_ZERO
            0x00,  # OUT_LSB_ST
            0x00,  # TEST
            0x05,  # MPC = 5 -> 2^5 = 32 periods
        ]

        n_periods = encoder.configure_in_calibration_mode()

        assert n_periods == 32

    def test_mpc_0x0c_changed_to_0x0b(self, encoder, mock_mc) -> None:
        mock_mc.communication.get_register.side_effect = [
            0x80,  # ENAC already set
            0x02,  # MODEA already BiSS
            0x0E,  # OUT_MSB_ZERO already correct
            0x20,  # OUT_LSB_ST = raw mode
            0x00,  # TEST
            0x0C,  # MPC = 0x0C (triggers reduction)
        ]

        n_periods = encoder.configure_in_calibration_mode()

        assert n_periods == 2048


class TestAnalogAdjustments:
    def test_read_calls_biss_read_8_times(self, encoder, mock_mc) -> None:
        mock_mc.communication.get_register.side_effect = [
            10,
            20,
            30,
            40,
            50,
            60,
            70,
            80,
        ]

        encoder.read_analog_adjustments()

        assert mock_mc.communication.get_register.call_count == 8

    def test_write_calls_8_registers(self, encoder, mock_mc, mocker) -> None:
        master = mocker.MagicMock(
            cosine_gain=1,
            sine_offset=2,
            cosine_offset=3,
            phase=4,
        )
        nonius = mocker.MagicMock(
            cosine_gain=5,
            sine_offset=6,
            cosine_offset=7,
            phase=8,
        )

        encoder.write_analog_adjustments(master, nonius)

        assert mock_mc.communication.set_register.call_count == 32


class TestWriteNoniusParameters:
    def test_writes_spo_registers(self, encoder, mock_mc, mocker) -> None:
        table_params = mocker.MagicMock()
        table_params.spo_base = 0x05
        table_params.spo_n = list(range(15))

        encoder.write_nonius_parameters(table_params)

        assert mock_mc.communication.set_register.call_count == 32


class TestSaveToEeprom:
    def test_success(self, encoder, mock_mc) -> None:
        mock_mc.communication.get_register.return_value = 0x00
        assert encoder.save_to_eeprom() is True

    def test_epr_error(self, encoder, mock_mc) -> None:
        mock_mc.communication.get_register.return_value = 0x40
        assert encoder.save_to_eeprom() is False

    def test_crc_error(self, encoder, mock_mc) -> None:
        mock_mc.communication.get_register.return_value = 0x80
        assert encoder.save_to_eeprom() is False


# ---------------------------------------------------------------------------
#  Hardware integration tests (require a physical drive)
# ---------------------------------------------------------------------------


@pytest.fixture
def hw_encoder(mc):
    """Create an Encoder using the real MotionController provided by summit_testing_framework."""
    return Encoder(mc, encoder_number=1, axis=1)


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

    def test_context_manager_restores_config(self, hw_encoder) -> None:
        drive_before = hw_encoder.get_drive_config()
        ic_before = hw_encoder.get_ic_config()

        with hw_encoder.in_calibration_mode() as n_periods:
            assert n_periods > 0

        drive_after = hw_encoder.get_drive_config()
        ic_after = hw_encoder.get_ic_config()
        assert drive_after == drive_before
        assert ic_after == ic_before


@pytest.mark.hardware
class TestAnalogAdjustmentsHardware:
    def test_read_returns_two_adjustments(self, hw_encoder) -> None:
        master, nonius = hw_encoder.read_analog_adjustments()
        assert master is not None
        assert nonius is not None
