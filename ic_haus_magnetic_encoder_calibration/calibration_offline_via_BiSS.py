#!/usr/bin/env python3

"""Copyright 2022 iC-Haus GmbH.

Software and its documentation is provided by iC-Haus GmbH or contributors "AS IS" and is
subject to the ZVEI General Conditions for the Supply of Products and Services with iC-Haus
amendments and the ZVEI Software clause with iC-Haus amendments (http://www.ichaus.de/EULA).
"""

import math
import os.path
import time
from collections.abc import Iterable
from typing import Any

import mu_3sl_interface as mu_3sl
import numpy as np
from ingeniamotion import MotionController
from mu_3sl_calibration_adjustments import (
    optional_print_optimized_nonius_track_offset_parameters,
    optional_print_optimized_nonius_track_offset_table,
    print_analog_adjustments,
    print_analog_analyze_result_adjustable_log,
    print_analyze_result_log,
)

# --- Configuration ---
interface_name = "\\Device\\NPF_{82FB3479-1D1F-4252-A2D6-AE569A011CC3}"
dictionary_path = (
    r"C:\GitProjects\MU_3SL_interface_3.4.2\wrapper\python\examples\dr3247b-30_48-e_1.1.0_v3.xdf"
)
print(f"Library version: {mu_3sl.__version__}")

# Register addresses
ADDR_OUT_MSB_ZERO = 0x11  # OUT_ZERO[7:5], OUT_MSB[4:0]
ADDR_OUT_LSB_ST = 0x12  # MODE_ST[5:4], OUT_LSB[3:0]
ADDR_TEST = 0x18  # TEST[7:0]
ADDR_MPC = 0x0F  # MPC[3:0]
ADDR_HARD_REV = 0x74
ADDR_MODEA_MODEB = 0x0B


# Register masks
MASK_OUT_MSB = 0b1_1111
MASK_OUT_ZERO = 0b111 << 5
MASK_OUT_LSB = 0b1111
MASK_MODE_ST = 0b11 << 4
MASK_MPC = 0x0F
MODEA_MASK = 0b00000111

# Configuration values
out_msb_code: int = 0x0E  # -> MSB=bit 27 (Master[0..13] + Nonius[14..27])
out_zero_code_biss: int = 0x00
out_lsb_code: int = 0x00  # -> LSB=bit 0
MODE_ST_RAW = 0b10  # Master + Nonius raw (calibración)
set_test_value: int = 0x00

# Analog parameters that must be modified for calibration
GX_M = 0x01
VOSS_M = 0x02
VOSC_M = 0x03
PH_M = 0x04
GX_N = 0x07
VOSS_N = 0x08
VOSC_N = 0x09
PH_N = 0x0A

# Nonius parameters that must be modified for calibration
SPO_BASE_0 = 0x52
SPO_1_2 = 0x53
SPO_3_4 = 0x54
SPO_5_6 = 0x55
SPO_7_8 = 0x56
SPO_9_10 = 0x57
SPO_11_12 = 0x58
SPO_13_14 = 0x59

# ---- bit widths for raw mode ----
MASTER_WIDTH = 14
NONIUS_WIDTH = 14
RAW_TOTAL = MASTER_WIDTH + NONIUS_WIDTH  # 28 bits

MASTER_MASK = (1 << MASTER_WIDTH) - 1
NONIUS_MASK = (1 << NONIUS_WIDTH) - 1
ADDR_ENAC = 0x05 & 0xFF  # ENAC bit7
MASK_ENAC = 0b1 << 7

# Bidirectional operations
NO_ACTION = 0
READ_ACTION = 1
WRITE_ACTION = 2

# Operation modes
VOLTAGE_MODE = 0
PROFILE_VELOCITY_MODE = 19

# Array to store initial register values for restoration
initial_values: list[int] = []


# Functions
def _update_bits(orig: int, mask: int, value_shifted: int) -> int:
    return (orig & ~mask) | (value_shifted & mask)


def extract_master_nonius_from_payload(payload_val: int) -> tuple[int, int]:
    """Extract 14-bit master and nonius from a packed payload value.

    Handles either lower 28 bits (master+nonius) or full 32-bit payload
    (lower 28 bits carry master+nonius, upper 4 are padding).
    The integer is treated as LSB-aligned.

    Returns:
        Tuple of (master, nonius) 14-bit values.
    """
    # If the value is larger than 28 bits but within 32 bits, we still only use the lower 28.

    master = payload_val & MASTER_MASK
    nonius = (payload_val >> MASTER_WIDTH) & NONIUS_MASK
    return master, nonius


def flatten(nested: Iterable[Iterable[int]]) -> Iterable[int]:
    """Flatten a nested iterable of integers into a single iterable.

    Yields:
        Each integer from the nested structure.
    """
    for row in nested:
        yield from row


def split_channel_data(channel_data: list[list[int]]) -> list[tuple[int, int]]:
    """Split packed channel data into master and nonius pairs.

    Each element is a packed value as read from the interface that holds
    at least the lower 28 bits of (master+nonius).

    Returns:
        List of (master, nonius) tuples.
    """
    pairs = []
    for val in flatten(channel_data):
        m, n = extract_master_nonius_from_payload(val)
        pairs.append((m, n))
    return pairs


def configure_ic() -> int:
    """Configure the iC-MU encoder registers for calibration.

    Returns:
        Number of master periods derived from the MPC register.
    """
    # Configure ENAC
    mc.communication.set_register("FBK_BISS1_SSI1_ITF_CTL", NO_ACTION, axis=1)
    mc.communication.set_register("FBK_BISS1_SSI1_ITF_ADDR", ADDR_ENAC, axis=1)
    mc.communication.set_register("FBK_BISS1_SSI1_ITF_CTL", READ_ACTION, axis=1)
    time.sleep(0.1)
    initial_enac = int(mc.communication.get_register("FBK_BISS1_SSI1_ITF_DATA", axis=1)) & 0xFF
    initial_values.append(initial_enac)
    bit_changed_enac = initial_enac | MASK_ENAC  # Enable ENAC
    if initial_enac != bit_changed_enac:
        mc.communication.set_register("FBK_BISS1_SSI1_ITF_DATA", bit_changed_enac, axis=1)
        mc.communication.set_register("FBK_BISS1_SSI1_ITF_CTL", WRITE_ACTION, axis=1)
        mc.communication.set_register("FBK_BISS1_SSI1_ITF_CTL", READ_ACTION, axis=1)
        time.sleep(0.1)
        result = mc.communication.get_register("FBK_BISS1_SSI1_ITF_DATA", axis=1)
        print(f"Writing ENAC=1 on address 0x{ADDR_ENAC:02X}: result = {result}")

    # Set MODEA to BiSS
    mc.communication.set_register("FBK_BISS1_SSI1_ITF_CTL", NO_ACTION, axis=1)
    mc.communication.set_register("FBK_BISS1_SSI1_ITF_ADDR", ADDR_MODEA_MODEB, axis=1)
    mc.communication.set_register("FBK_BISS1_SSI1_ITF_CTL", READ_ACTION, axis=1)
    time.sleep(0.1)
    initial_modeab = int(mc.communication.get_register("FBK_BISS1_SSI1_ITF_DATA", axis=1)) & 0xFF
    initial_values.append(initial_modeab)
    mc.communication.set_register("FBK_BISS1_SSI1_ITF_CTL", NO_ACTION, axis=1)
    mc.communication.set_register("FBK_BISS1_SSI1_ITF_ADDR", ADDR_MODEA_MODEB, axis=1)
    mc.communication.set_register("FBK_BISS1_SSI1_ITF_DATA", 2, axis=1)
    mc.communication.set_register("FBK_BISS1_SSI1_ITF_CTL", WRITE_ACTION, axis=1)
    time.sleep(0.1)

    # Configure OUT_MSB, OUT_ZERO
    mc.communication.set_register("FBK_BISS1_SSI1_ITF_CTL", NO_ACTION, axis=1)
    mc.communication.set_register("FBK_BISS1_SSI1_ITF_ADDR", ADDR_OUT_MSB_ZERO, axis=1)
    mc.communication.set_register("FBK_BISS1_SSI1_ITF_CTL", READ_ACTION, axis=1)
    time.sleep(0.1)
    initial_out_msb_zero = (
        int(mc.communication.get_register("FBK_BISS1_SSI1_ITF_DATA", axis=1)) & 0xFF
    )
    initial_values.append(initial_out_msb_zero)
    val_zero = (out_zero_code_biss & 0x7) << 5
    val_msb = out_msb_code & 0x1F
    after = _update_bits(initial_out_msb_zero, MASK_OUT_ZERO, val_zero)
    final_out_msb_zero = _update_bits(after, MASK_OUT_MSB, val_msb)
    if final_out_msb_zero != initial_out_msb_zero:
        mc.communication.set_register("FBK_BISS1_SSI1_ITF_DATA", final_out_msb_zero, axis=1)
        mc.communication.set_register("FBK_BISS1_SSI1_ITF_CTL", WRITE_ACTION, axis=1)
        mc.communication.set_register("FBK_BISS1_SSI1_ITF_CTL", READ_ACTION, axis=1)
        result = mc.communication.get_register("FBK_BISS1_SSI1_ITF_DATA", axis=1)
        print(
            f"Writing OUT_ZERO=0x{out_zero_code_biss:02X} "
            f"and OUT_MSB=0x{out_msb_code:02X} "
            f"on address 0x{ADDR_OUT_MSB_ZERO:02X}: result = {result}"
        )

    # Configure OUT_LSB, MODE_ST=RAW
    mc.communication.set_register("FBK_BISS1_SSI1_ITF_CTL", NO_ACTION, axis=1)
    mc.communication.set_register("FBK_BISS1_SSI1_ITF_ADDR", ADDR_OUT_LSB_ST, axis=1)
    mc.communication.set_register("FBK_BISS1_SSI1_ITF_CTL", READ_ACTION, axis=1)
    time.sleep(0.1)
    initial_out_lsb_modest = int(mc.communication.get_register("FBK_BISS1_SSI1_ITF_DATA")) & 0xFF
    initial_values.append(initial_out_lsb_modest)
    val_mode_st = (MODE_ST_RAW & 0x3) << 4
    val_lsb = out_lsb_code & 0xF
    after = _update_bits(initial_out_lsb_modest, MASK_MODE_ST, val_mode_st)
    final_out_lsb_modest = _update_bits(after, MASK_OUT_LSB, val_lsb)
    if final_out_lsb_modest != initial_out_lsb_modest:
        mc.communication.set_register("FBK_BISS1_SSI1_ITF_DATA", final_out_lsb_modest, axis=1)
        mc.communication.set_register("FBK_BISS1_SSI1_ITF_CTL", WRITE_ACTION, axis=1)
        mc.communication.set_register("FBK_BISS1_SSI1_ITF_CTL", READ_ACTION, axis=1)
        time.sleep(0.1)
        result = mc.communication.get_register("FBK_BISS1_SSI1_ITF_DATA", axis=1)
        print(
            f"Writing MODE_ST=RAW and OUT_LSB=0x{out_lsb_code:02X} "
            f"on address 0x{ADDR_OUT_LSB_ST:02X}: result = {result}"
        )

    # Configure TEST
    mc.communication.set_register("FBK_BISS1_SSI1_ITF_CTL", NO_ACTION, axis=1)
    mc.communication.set_register("FBK_BISS1_SSI1_ITF_ADDR", ADDR_TEST, axis=1)
    mc.communication.set_register("FBK_BISS1_SSI1_ITF_CTL", READ_ACTION, axis=1)
    time.sleep(0.1)
    initial_test = int(mc.communication.get_register("FBK_BISS1_SSI1_ITF_DATA", axis=1)) & 0xFF
    initial_values.append(initial_test)
    after_test = set_test_value & 0xFF
    if initial_test != after_test:
        mc.communication.set_register("FBK_BISS1_SSI1_ITF_DATA", after_test, axis=1)
        mc.communication.set_register("FBK_BISS1_SSI1_ITF_CTL", WRITE_ACTION, axis=1)
        mc.communication.set_register("FBK_BISS1_SSI1_ITF_CTL", READ_ACTION, axis=1)
        time.sleep(0.1)
        result = mc.communication.get_register("FBK_BISS1_SSI1_ITF_DATA", axis=1)
        print(
            f"Writing TEST=0x{set_test_value:02X} on address 0x{ADDR_TEST:02X}: result = {result}"
        )

    # Configure MPC if initial_mpc_value == 0x0C -> set to 0x0B
    mc.communication.set_register("FBK_BISS1_SSI1_ITF_CTL", NO_ACTION, axis=1)
    mc.communication.set_register("FBK_BISS1_SSI1_ITF_ADDR", ADDR_MPC, axis=1)
    mc.communication.set_register("FBK_BISS1_SSI1_ITF_CTL", READ_ACTION, axis=1)
    time.sleep(0.1)
    initial_mpc = int(mc.communication.get_register("FBK_BISS1_SSI1_ITF_DATA", axis=1)) & 0xFF
    initial_values.append(initial_mpc)
    master_period_code = initial_mpc & MASK_MPC
    n_master_periods = 1 << master_period_code
    if (initial_mpc & MASK_MPC) == 0x0C:
        mpc_changed = _update_bits(initial_mpc, MASK_MPC, 0x0B)
        if mpc_changed != initial_mpc:
            mc.communication.set_register("FBK_BISS1_SSI1_ITF_DATA", mpc_changed, axis=1)
            mc.communication.set_register("FBK_BISS1_SSI1_ITF_CTL", WRITE_ACTION, axis=1)
            mc.communication.set_register("FBK_BISS1_SSI1_ITF_CTL", READ_ACTION, axis=1)
            time.sleep(0.1)
            result = mc.communication.get_register("FBK_BISS1_SSI1_ITF_DATA", axis=1)
            print(f"Writing MPC=0x0B on address 0x{ADDR_MPC:02X}: result = {result}")

    return n_master_periods


def adjust_analog_parameters(master_param: Any, nonius_param: Any) -> None:
    """Write optimized analog track parameters to the iC-MU registers."""
    # Write optimized parameters
    # Analog parameters
    #                  Master |   Nonius
    # Cosine gain:      GX_M  |    GX_N
    # Sine offset:     VOSS_M |   VOSS_N
    # Cosine offset:   VOSC_M |   VOSC_N
    # Phase adjust:     PH_M  |    PH_N

    mc.communication.set_register("FBK_BISS1_SSI1_ITF_CTL", NO_ACTION, axis=1)
    mc.communication.set_register("FBK_BISS1_SSI1_ITF_ADDR", GX_M, axis=1)
    mc.communication.set_register("FBK_BISS1_SSI1_ITF_DATA", master_param.cosine_gain, axis=1)
    mc.communication.set_register("FBK_BISS1_SSI1_ITF_CTL", WRITE_ACTION, axis=1)
    time.sleep(0.1)

    mc.communication.set_register("FBK_BISS1_SSI1_ITF_CTL", NO_ACTION, axis=1)
    mc.communication.set_register("FBK_BISS1_SSI1_ITF_ADDR", VOSS_M, axis=1)
    mc.communication.set_register("FBK_BISS1_SSI1_ITF_DATA", master_param.sine_offset, axis=1)
    mc.communication.set_register("FBK_BISS1_SSI1_ITF_CTL", WRITE_ACTION, axis=1)
    time.sleep(0.1)

    mc.communication.set_register("FBK_BISS1_SSI1_ITF_CTL", NO_ACTION, axis=1)
    mc.communication.set_register("FBK_BISS1_SSI1_ITF_ADDR", VOSC_M, axis=1)
    mc.communication.set_register("FBK_BISS1_SSI1_ITF_DATA", master_param.cosine_offset, axis=1)
    mc.communication.set_register("FBK_BISS1_SSI1_ITF_CTL", WRITE_ACTION, axis=1)
    time.sleep(0.1)

    mc.communication.set_register("FBK_BISS1_SSI1_ITF_CTL", NO_ACTION, axis=1)
    mc.communication.set_register("FBK_BISS1_SSI1_ITF_ADDR", PH_M, axis=1)
    mc.communication.set_register("FBK_BISS1_SSI1_ITF_DATA", master_param.phase, axis=1)
    mc.communication.set_register("FBK_BISS1_SSI1_ITF_CTL", WRITE_ACTION, axis=1)
    time.sleep(0.1)

    mc.communication.set_register("FBK_BISS1_SSI1_ITF_CTL", NO_ACTION, axis=1)
    mc.communication.set_register("FBK_BISS1_SSI1_ITF_ADDR", GX_N, axis=1)
    mc.communication.set_register("FBK_BISS1_SSI1_ITF_DATA", nonius_param.cosine_gain, axis=1)
    mc.communication.set_register("FBK_BISS1_SSI1_ITF_CTL", WRITE_ACTION, axis=1)
    time.sleep(0.1)

    mc.communication.set_register("FBK_BISS1_SSI1_ITF_CTL", NO_ACTION, axis=1)
    mc.communication.set_register("FBK_BISS1_SSI1_ITF_ADDR", VOSS_N, axis=1)
    mc.communication.set_register("FBK_BISS1_SSI1_ITF_DATA", nonius_param.sine_offset, axis=1)
    mc.communication.set_register("FBK_BISS1_SSI1_ITF_CTL", WRITE_ACTION, axis=1)
    time.sleep(0.1)

    mc.communication.set_register("FBK_BISS1_SSI1_ITF_CTL", NO_ACTION, axis=1)
    mc.communication.set_register("FBK_BISS1_SSI1_ITF_ADDR", VOSC_N, axis=1)
    mc.communication.set_register("FBK_BISS1_SSI1_ITF_DATA", nonius_param.cosine_offset, axis=1)
    mc.communication.set_register("FBK_BISS1_SSI1_ITF_CTL", WRITE_ACTION, axis=1)
    time.sleep(0.1)

    mc.communication.set_register("FBK_BISS1_SSI1_ITF_CTL", NO_ACTION, axis=1)
    mc.communication.set_register("FBK_BISS1_SSI1_ITF_ADDR", PH_N, axis=1)
    mc.communication.set_register("FBK_BISS1_SSI1_ITF_DATA", nonius_param.phase, axis=1)
    mc.communication.set_register("FBK_BISS1_SSI1_ITF_CTL", WRITE_ACTION, axis=1)
    time.sleep(0.1)


def read_master_nonius_parameters() -> tuple[Any, Any]:
    """Read current master and nonius analog parameters from the iC-MU.

    Returns:
        Tuple of (master_adjustments, nonius_adjustments).
    """
    mc.communication.set_register("FBK_BISS1_SSI1_ITF_CTL", NO_ACTION, axis=1)
    mc.communication.set_register("FBK_BISS1_SSI1_ITF_ADDR", GX_M, axis=1)
    mc.communication.set_register("FBK_BISS1_SSI1_ITF_CTL", READ_ACTION, axis=1)
    time.sleep(0.1)
    cosine_gain = mc.communication.get_register("FBK_BISS1_SSI1_ITF_DATA", axis=1)

    mc.communication.set_register("FBK_BISS1_SSI1_ITF_CTL", NO_ACTION, axis=1)
    mc.communication.set_register("FBK_BISS1_SSI1_ITF_ADDR", VOSS_M, axis=1)
    mc.communication.set_register("FBK_BISS1_SSI1_ITF_CTL", READ_ACTION, axis=1)
    time.sleep(0.1)
    sine_offset = mc.communication.get_register("FBK_BISS1_SSI1_ITF_DATA", axis=1)

    mc.communication.set_register("FBK_BISS1_SSI1_ITF_CTL", NO_ACTION, axis=1)
    mc.communication.set_register("FBK_BISS1_SSI1_ITF_ADDR", VOSC_M, axis=1)
    mc.communication.set_register("FBK_BISS1_SSI1_ITF_CTL", READ_ACTION, axis=1)
    time.sleep(0.1)
    cosine_offset = mc.communication.get_register("FBK_BISS1_SSI1_ITF_DATA", axis=1)

    mc.communication.set_register("FBK_BISS1_SSI1_ITF_CTL", NO_ACTION, axis=1)
    mc.communication.set_register("FBK_BISS1_SSI1_ITF_ADDR", PH_M, axis=1)
    mc.communication.set_register("FBK_BISS1_SSI1_ITF_CTL", READ_ACTION, axis=1)
    time.sleep(0.1)
    phase = mc.communication.get_register("FBK_BISS1_SSI1_ITF_DATA", axis=1)

    initial_master_adjustments = mu_3sl.AnalogTrackAdjustments(
        cosine_gain, sine_offset, cosine_offset, phase
    )

    mc.communication.set_register("FBK_BISS1_SSI1_ITF_CTL", NO_ACTION, axis=1)
    mc.communication.set_register("FBK_BISS1_SSI1_ITF_ADDR", GX_N, axis=1)
    mc.communication.set_register("FBK_BISS1_SSI1_ITF_CTL", READ_ACTION, axis=1)
    time.sleep(0.1)
    nonius_cosine_gain = mc.communication.get_register("FBK_BISS1_SSI1_ITF_DATA", axis=1)

    mc.communication.set_register("FBK_BISS1_SSI1_ITF_CTL", NO_ACTION, axis=1)
    mc.communication.set_register("FBK_BISS1_SSI1_ITF_ADDR", VOSS_N, axis=1)
    mc.communication.set_register("FBK_BISS1_SSI1_ITF_CTL", READ_ACTION, axis=1)
    time.sleep(0.1)
    nonius_sine_offset = mc.communication.get_register("FBK_BISS1_SSI1_ITF_DATA", axis=1)

    mc.communication.set_register("FBK_BISS1_SSI1_ITF_CTL", NO_ACTION, axis=1)
    mc.communication.set_register("FBK_BISS1_SSI1_ITF_ADDR", VOSC_N, axis=1)
    mc.communication.set_register("FBK_BISS1_SSI1_ITF_CTL", READ_ACTION, axis=1)
    time.sleep(0.1)
    nonius_cosine_offset = mc.communication.get_register("FBK_BISS1_SSI1_ITF_DATA", axis=1)

    mc.communication.set_register("FBK_BISS1_SSI1_ITF_CTL", NO_ACTION, axis=1)
    mc.communication.set_register("FBK_BISS1_SSI1_ITF_ADDR", PH_N, axis=1)
    mc.communication.set_register("FBK_BISS1_SSI1_ITF_CTL", READ_ACTION, axis=1)
    time.sleep(0.1)
    nonius_phase = mc.communication.get_register("FBK_BISS1_SSI1_ITF_DATA", axis=1)

    initial_nonius_adjustments = mu_3sl.AnalogTrackAdjustments(
        nonius_cosine_gain,
        nonius_sine_offset,
        nonius_cosine_offset,
        nonius_phase,
    )

    return initial_master_adjustments, initial_nonius_adjustments


def restore_values() -> None:
    """Restore all iC-MU register values to their initial state."""
    # Configure ENAC to initial value
    mc.communication.set_register("FBK_BISS1_SSI1_ITF_CTL", NO_ACTION, axis=1)
    mc.communication.set_register("FBK_BISS1_SSI1_ITF_ADDR", ADDR_ENAC, axis=1)
    mc.communication.set_register("FBK_BISS1_SSI1_ITF_DATA", initial_values[0], axis=1)
    mc.communication.set_register("FBK_BISS1_SSI1_ITF_CTL", WRITE_ACTION, axis=1)
    time.sleep(0.1)
    # Configure MODEA_MODEB to initial value
    mc.communication.set_register("FBK_BISS1_SSI1_ITF_CTL", NO_ACTION, axis=1)
    mc.communication.set_register("FBK_BISS1_SSI1_ITF_ADDR", ADDR_MODEA_MODEB, axis=1)
    mc.communication.set_register("FBK_BISS1_SSI1_ITF_DATA", initial_values[1], axis=1)
    mc.communication.set_register("FBK_BISS1_SSI1_ITF_CTL", WRITE_ACTION, axis=1)
    time.sleep(0.1)

    # Configure OUT_MSB, OUT_ZERO to initial value
    mc.communication.set_register("FBK_BISS1_SSI1_ITF_CTL", NO_ACTION, axis=1)
    mc.communication.set_register("FBK_BISS1_SSI1_ITF_ADDR", ADDR_OUT_MSB_ZERO, axis=1)
    mc.communication.set_register("FBK_BISS1_SSI1_ITF_DATA", initial_values[2], axis=1)
    mc.communication.set_register("FBK_BISS1_SSI1_ITF_CTL", WRITE_ACTION, axis=1)
    time.sleep(0.1)
    # Configure OUT_LSB, MODE_ST to initial value
    mc.communication.set_register("FBK_BISS1_SSI1_ITF_CTL", NO_ACTION, axis=1)
    mc.communication.set_register("FBK_BISS1_SSI1_ITF_ADDR", ADDR_OUT_LSB_ST, axis=1)
    mc.communication.set_register("FBK_BISS1_SSI1_ITF_DATA", initial_values[3], axis=1)
    mc.communication.set_register("FBK_BISS1_SSI1_ITF_CTL", WRITE_ACTION, axis=1)
    time.sleep(0.1)
    # Configure TEST to initial value
    mc.communication.set_register("FBK_BISS1_SSI1_ITF_CTL", NO_ACTION, axis=1)
    mc.communication.set_register("FBK_BISS1_SSI1_ITF_ADDR", ADDR_TEST, axis=1)
    mc.communication.set_register("FBK_BISS1_SSI1_ITF_DATA", initial_values[4], axis=1)
    mc.communication.set_register("FBK_BISS1_SSI1_ITF_CTL", WRITE_ACTION, axis=1)
    time.sleep(0.1)
    # Configure MPC to initial value
    mc.communication.set_register("FBK_BISS1_SSI1_ITF_CTL", NO_ACTION, axis=1)
    mc.communication.set_register("FBK_BISS1_SSI1_ITF_ADDR", ADDR_MPC, axis=1)
    mc.communication.set_register("FBK_BISS1_SSI1_ITF_DATA", initial_values[5], axis=1)
    mc.communication.set_register("FBK_BISS1_SSI1_ITF_CTL", WRITE_ACTION, axis=1)
    time.sleep(0.1)


def acquire_data() -> str:
    """Acquire raw encoder data by spinning the motor.

    Returns:
        Path to the output CSV file with master and nonius raw data.
    """
    sampling_time_s = 0.01
    polled_registers: list[dict[str, int | str]] = [
        {
            "name": "FBK_BISS1_SSI1_POS_VALUE",
            "axis": 1,
        }
    ]
    velocity_sp = 1  # rev/s
    min_test_duration = 30  # seconds

    print("\nStarting the process to capture raw data for calibration...")

    # Set drive to velocity mode
    print("Setting drive to velocity mode")

    # Move the motor at a defined velocity
    print(f"Set velocity operation mode and move at {velocity_sp} rev/s")
    mc.motion.motor_enable()

    generator_freq = 0.5
    while generator_freq <= 4:
        mc.motion.set_current_quadrature(generator_freq + 1)
        time.sleep(0.2)
        mc.communication.set_register("FBK_GEN_FREQ", generator_freq)
        time.sleep(0.2)
        generator_freq += 0.5

    time.sleep(0.5)
    # Create poller after enabling the motor to avoid initial static data
    poller = mc.capture.pdo.create_poller(polled_registers, sampling_time=sampling_time_s)

    print("Starting capture")
    channel_data: list[list[int]] = [[] for register in polled_registers]
    test_timeout = time.time() + min_test_duration
    try:
        while test_timeout > time.time():
            time.sleep(0.1)
            timestamp, data = poller.data
            for idx, channel in enumerate(data):
                channel_data[idx].extend(int(v) for v in channel)
    except KeyboardInterrupt:
        print("Stopping capture")

    # Stopping the motor
    mc.motion.set_velocity(0)
    mc.motion.motor_disable()

    # Stop the poller
    poller.stop()

    # Split the captured data into master and nonius raw data
    splitted_raw_data = split_channel_data(channel_data)

    # Save the captured data to a CSV file
    print("Capture finished, saving data to file")
    output_file_name = "firstprova" + ".csv"
    np.savetxt(
        output_file_name,
        np.asarray(splitted_raw_data),
        delimiter=",",
        fmt="%d",
        header="master_raw,nonius_raw",
        comments="",
    )
    print(f"Data saved to {output_file_name}")
    return output_file_name


# --- Step 1: Connect to the drive ---
# (comment this section if the communication is already done via ML3)
mc = MotionController()
mc.communication.connect_servo_ethercat(interface_name, slave_id=1, dict_path=dictionary_path)


# --- Step 2: Read and keep initial configuration registers ---
# Read revision code
mc.communication.set_register("FBK_BISS1_SSI1_ITF_CTL", NO_ACTION, axis=1)
mc.communication.set_register("FBK_BISS1_SSI1_ITF_ADDR", ADDR_HARD_REV, axis=1)
mc.communication.set_register("FBK_BISS1_SSI1_ITF_CTL", READ_ACTION, axis=1)
time.sleep(0.1)
revision_code_val = int(mc.communication.get_register("FBK_BISS1_SSI1_ITF_DATA", axis=1)) & 0xFF
revision_code = mu_3sl.Revision(revision_code_val)

if revision_code is mu_3sl.Revision.NONE:
    print("Error: Unable to read revision code from iC-MU device!")
    exit()


# Configure IC for calibration
n_master_periods = configure_ic()

# NOTE: Abs encoder should be configured as: frame size 36 bits,
# position and single turn bits 28, start bit 8 for the calibration.

# --- Step 3: Spin motor, read absolute position values from,
# convert into master and nonius raw data and calibrate
permissible_residual_errors_during_analog_calibration = 1.0
calibration_data_idx = 0
calibration = mu_3sl.Calibration(revision_code)

iteration = 1
# Ideally should be while True, and tolerated error should break the loop
while iteration <= 3:
    print(f"Iteration # {iteration}")
    output_file = acquire_data()
    if not os.path.isfile(output_file):
        print("Error: Calibration data file not found!")
        exit(1)

    print(
        "---------------------------------------------------------------\n"
        "---------------- Acquire raw data (from file) -----------------\n"
        "---------------------------------------------------------------"
    )

    print(f'Read master and nonius track raw data from file:\n"{output_file}"\n')

    print(
        "---------------------------------------------------------------\n"
        "---------------------- Analyze raw data  ----------------------\n"
        "---------------------------------------------------------------"
    )

    file_csv_data = np.genfromtxt("firstprova.csv", delimiter=",", names=["master", "nonius"])
    master_raw_data = [int(i) for i in file_csv_data["master"][1:].tolist()]
    nonius_raw_data = [int(i) for i in file_csv_data["nonius"][1:].tolist()]

    analyze_result = calibration.analyze_raw_data(master_raw_data, nonius_raw_data)
    print_analyze_result_log(analyze_result)

    print_analog_analyze_result_adjustable_log(calibration, analyze_result)

    relative_master_track_adjustments = analyze_result.relative_master_track_adjustments()
    relative_nonius_track_adjustments = analyze_result.relative_nonius_track_adjustments()

    if (
        abs(relative_master_track_adjustments.cosine_gain_lsb)
        <= permissible_residual_errors_during_analog_calibration
        and abs(relative_nonius_track_adjustments.cosine_gain_lsb)
        <= permissible_residual_errors_during_analog_calibration
        and abs(relative_master_track_adjustments.sine_offset_lsb)
        <= permissible_residual_errors_during_analog_calibration
        and abs(relative_nonius_track_adjustments.sine_offset_lsb)
        <= permissible_residual_errors_during_analog_calibration
        and abs(relative_master_track_adjustments.cosine_offset_lsb)
        <= permissible_residual_errors_during_analog_calibration
        and abs(relative_nonius_track_adjustments.cosine_offset_lsb)
        <= permissible_residual_errors_during_analog_calibration
        and abs(relative_master_track_adjustments.phase_lsb)
        <= permissible_residual_errors_during_analog_calibration
        and abs(relative_nonius_track_adjustments.phase_lsb)
        <= permissible_residual_errors_during_analog_calibration
    ):
        print(
            "\n"
            f'All residual errors (absolute relative changes in "LSB") '
            f"are smaller than {permissible_residual_errors_during_analog_calibration}.\n"
            "From this point, no further analog calibration step would be required.\n"
        )
        break

    print(
        "---------------------------------------------------------------\n"
        "------------ Adjust the analyzed analog parameters ------------\n"
        "---------------------------------------------------------------"
    )

    print("iC-MU analog parameters after adjustment:")
    calibration.adjust_analog_by_analyze_result(analyze_result)

    adjust_analog_parameters(
        calibration.analog_master_track_adjustments(),
        calibration.analog_nonius_track_adjustments(),
    )
    print_analog_adjustments(calibration)
    print("\n")

    iteration += 1

print(
    "\n"
    "---------------------------------------------------------------\n"
    "------------ Adjust the analyzed nonius parameters ------------\n"
    "---------------------------------------------------------------\n"
)

nonius_track_offset_table = analyze_result.optimized_nonius_track_offset_table()
nonius_track_offset_table_parameters = mu_3sl.nonius_track_offset_table_parameters(
    nonius_track_offset_table
)

# OPTIONAL - Generate a new analysis with the optimized nonius track offset table
calibration.set_current_nonius_track_offset_table(nonius_track_offset_table)
analyze_result = calibration.analyze_raw_data(master_raw_data, nonius_raw_data)
number_of_calculated_master_periods = analyze_result.number_of_calculated_master_periods()
calculated_master_period_code = round(math.log2(number_of_calculated_master_periods))
optional_print_optimized_nonius_track_offset_table(analyze_result, "new_curve.csv")
print("")
optional_print_optimized_nonius_track_offset_parameters(analyze_result)

# SPO_BASE
new_spo_base = nonius_track_offset_table_parameters.spo_base
# SPO_n
spo_array_n = []

for i in range(15):
    spo_array_i = nonius_track_offset_table_parameters.spo_n[i]
    spo_array_n.append(spo_array_i)

reg_52 = (spo_array_n[0] << 4) | new_spo_base & 0xFF
mc.communication.set_register("FBK_BISS1_SSI1_ITF_ADDR", SPO_BASE_0, axis=1)
mc.communication.set_register("FBK_BISS1_SSI1_ITF_DATA", reg_52, axis=1)
mc.communication.set_register("FBK_BISS1_SSI1_ITF_CTL", WRITE_ACTION, axis=1)
time.sleep(0.1)

addr = SPO_BASE_0 + 1
idx = 1
while addr <= SPO_13_14:
    even = spo_array_n[idx + 1] & 0xFF
    odd = spo_array_n[idx] & 0xFF
    addr_data = even << 4 | odd
    mc.communication.set_register("FBK_BISS1_SSI1_ITF_ADDR", addr, axis=1)
    mc.communication.set_register("FBK_BISS1_SSI1_ITF_DATA", addr_data, axis=1)
    mc.communication.set_register("FBK_BISS1_SSI1_ITF_CTL", WRITE_ACTION, axis=1)
    idx += 2
    addr += 1
    time.sleep(0.1)

# --- Step 7: Restore initial configuration registers
print("Restore initial iC-MU configuration")
restore_values()

# --- Step 8:
# Reset Encoder configuration

# --- Step 9: Safe configuration to the drive - WRITE_ALL (0x01) action in address 0x75
print("Safe calibration configuration")
mc.communication.set_register("FBK_BISS1_SSI1_ITF_CTL", NO_ACTION, axis=1)
mc.communication.set_register("FBK_BISS1_SSI1_ITF_ADDR", 0x75, axis=1)
mc.communication.set_register("FBK_BISS1_SSI1_ITF_DATA", 0x01, axis=1)
mc.communication.set_register("FBK_BISS1_SSI1_ITF_CTL", WRITE_ACTION, axis=1)
time.sleep(1)
mc.communication.set_register("FBK_BISS1_SSI1_ITF_CTL", NO_ACTION, axis=1)
mc.communication.set_register("FBK_BISS1_SSI1_ITF_ADDR", 0x77, axis=1)
mc.communication.set_register("FBK_BISS1_SSI1_ITF_CTL", READ_ACTION, axis=1)
time.sleep(0.1)
status1 = int(mc.communication.get_register("FBK_BISS1_SSI1_ITF_DATA", axis=1)) & 0xFF
if status1 & (1 << 6):  # EPR_ERR
    print("EEPROM write error (EPR_ERR=1)")
if status1 & (1 << 7):  # CRC_ERR
    print("CRC error (CRC_ERR=1)")
else:
    print("Calibration configuration saved successfully to the iC-MU EEPROM")
