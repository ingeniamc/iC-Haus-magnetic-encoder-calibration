"""Entry point for the iC-Haus Magnetic Encoder Calibration tool."""

import argparse


def parse_args() -> argparse.Namespace:
    """Parse command-line arguments for encoder calibration.

    Returns:
        Parsed arguments namespace.
    """
    parser = argparse.ArgumentParser(
        description="iC-Haus Magnetic Encoder Calibration via BiSS over EtherCAT",
    )
    parser.add_argument(
        "--interface",
        required=True,
        help=r"EtherCAT network interface name (e.g. \Device\NPF_{...})",
    )
    parser.add_argument(
        "--dictionary",
        required=True,
        help="Path to the XDF dictionary file for the drive",
    )
    parser.add_argument(
        "--slave-id",
        type=int,
        default=1,
        help="EtherCAT slave ID (default: 1)",
    )
    return parser.parse_args()


def main() -> None:
    """Run the iC-Haus magnetic encoder calibration."""
    _args = parse_args()


if __name__ == "__main__":
    main()
