"""DR3247B-30/48-E hardware test setup.

Run hardware tests with:
    pytest tests -m hardware --setup=tests.setups.dr3247b.setup.TESTS_SETUP

Set ECAT_IFNAME to override the EtherCAT interface for your machine:
    set ECAT_IFNAME=\\Device\\NPF_{YOUR-ADAPTER-GUID}
"""

import os
from pathlib import Path

from summit_testing_framework.setups.specifiers import (
    LocalDriveConfigSpecifier,
)

_DR3247_DIR = Path(__file__).resolve().parent
IFNAME = os.environ["ECAT_IFNAME"]
DICTIONARY = _DR3247_DIR / "dr3247b-30_48-e_1.1.0_v3.xdf"
CONFIG_FILE = _DR3247_DIR / "dr3247b-30_48-e_biss_config.xcf"

DR3247B_SETUP = LocalDriveConfigSpecifier.from_ethercat_configuration(
    config_file=CONFIG_FILE,
    identifier="DR3247B-30/48-E",
    ifname=IFNAME,
    slave=1,
    boot_in_app=True,
    dictionary=DICTIONARY,
    ignore_fw=True,
    extra_data={},
)

TESTS_SETUP = DR3247B_SETUP
