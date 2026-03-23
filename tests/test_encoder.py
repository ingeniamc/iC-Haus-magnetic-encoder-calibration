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
    BissAction,
)


@pytest.fixture
def encoder(mock_mc):
    return Encoder(mock_mc, encoder_number=1, axis=1)


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
    def test_mpc_0x0c_reduced_to_0x0b(self, encoder, mock_mc) -> None:
        """MPC=0x0C is a special case that must be reduced to 0x0B."""
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


class TestGetICConfig:
    """Tests for iC-MU config save/restore data integrity."""

    def test_roundtrip_preserves_values(self, encoder, mock_mc) -> None:
        """get_ic_config → set_ic_config writes back the exact same register bytes."""
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


class TestInCalibrationModeContextManager:
    """Context manager must always restore config, even on exception."""

    def test_restores_on_exception(self, encoder, mocker) -> None:
        saved_drive = mocker.MagicMock(name="DriveConfig")
        saved_ic = mocker.MagicMock(name="ICConfig")
        mocker.patch.object(encoder, "get_drive_config", return_value=saved_drive)
        mocker.patch.object(encoder, "get_ic_config", return_value=saved_ic)
        mocker.patch.object(encoder, "configure_in_calibration_mode", return_value=32)
        mocker.patch.object(encoder, "set_ic_config")
        mocker.patch.object(encoder, "set_drive_config")

        with pytest.raises(ValueError, match="test error"), encoder.in_calibration_mode():
            raise ValueError("test error")

        encoder.set_ic_config.assert_called_once_with(saved_ic)
        encoder.set_drive_config.assert_called_once_with(saved_drive)


# ---------------------------------------------------------------------------
#  Hardware integration tests (require a physical drive)
# ---------------------------------------------------------------------------


@pytest.fixture
def hw_encoder(mc):
    """Create an Encoder using the real MotionController.

    Returns:
        An Encoder configured for encoder 1 on axis 1.
    """
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
