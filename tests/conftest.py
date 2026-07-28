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
def mock_encoder_config():
    """Provide a valid encoder config so Encoder can be instantiated.

    Returns:
        An EncoderRegisterConfig instance with valid register values.

    """
    valid_config = EncoderRegisterConfig(
        out_msb=0x05, out_lsb=0x00, mode_st=0x00, enac=0x01, cfgew=0xFF, filt=0x03
    )
    return valid_config
