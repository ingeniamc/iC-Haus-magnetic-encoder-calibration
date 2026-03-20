"""Test configuration for iC-Haus Magnetic Encoder Calibration."""

import pytest

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
