"""DR3256C-POC hardware test setup.

Run hardware tests with:
    pytest tests -m hardware --setup=tests.setups.dr3256c.setup.TESTS_SETUP

Set ECAT_IFNAME to override the EtherCAT interface for your machine:
    set ECAT_IFNAME=\\Device\\NPF_{YOUR-ADAPTER-GUID}
"""

import os
from pathlib import Path

from summit_testing_framework.setups.specifiers import (
    LocalDriveConfigSpecifier,
)

_DR3256C_DIR = Path(__file__).resolve().parent
IFNAME = os.environ.get("ECAT_IFNAME")
DICTIONARY = _DR3256C_DIR / "dr3256c-poc_0.1.0.xdf3"
CONFIG_FILE = _DR3256C_DIR / "dr3256c-poc_biss_config.xcf"

DR3256C_POC_SETUP = LocalDriveConfigSpecifier.from_ethercat_configuration(
    config_file=CONFIG_FILE,
    identifier="DR3256C-POC",
    ifname=IFNAME,
    slave=1,
    boot_in_app=True,
    dictionary=DICTIONARY,
    ignore_fw=True,
    extra_data={},
)

TESTS_SETUP = DR3256C_POC_SETUP
