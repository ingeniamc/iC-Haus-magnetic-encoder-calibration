"""Test configuration for iC-Haus Magnetic Encoder Calibration."""

import pytest

from ic_haus_magnetic_encoder_calibration.config_loader import EncoderRegisterConfig

pytest_plugins = [
    "summit_testing_framework.pytest_addoptions",
    "summit_testing_framework.setup_fixtures",
]


@pytest.fixture
def mock_mc(mocker):
    """MagicMock MotionController for unit tests.

    Returns:
        A MagicMock instance.
    """
    return mocker.MagicMock()


@pytest.fixture
def mock_encoder_config(mocker):
    """Provide valid encoder configs so EncoderCalibrator can be instantiated."""
    valid_config = EncoderRegisterConfig(
        out_msb=0x05, out_lsb=0x00, mode_st=0x00, enac=0x01, cfgew=0xFF, filt=0x03
    )
    mocker.patch(
        "ic_haus_magnetic_encoder_calibration.calibrator.load_encoders_configuration_file",
        return_value={1: valid_config, 2: valid_config},
    )
