"""Entry point for the iC-Haus Magnetic Encoder Calibration tool."""

import argparse
import logging
import sys

from ingeniamotion import MotionController

from ic_haus_magnetic_encoder_calibration.calibrator import EncoderCalibrator
from ic_haus_magnetic_encoder_calibration.motor_control import (
    DEFAULT_GEN_CURRENT,
    DEFAULT_GEN_FREQ,
)

logger = logging.getLogger("ic_haus_magnetic_encoder_calibration")


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
    parser.add_argument(
        "--encoder",
        choices=["1", "2", "both"],
        default="both",
        help="Which encoder(s) to calibrate (default: both)",
    )
    parser.add_argument(
        "--axis",
        type=int,
        default=1,
        help="Drive axis number (default: 1)",
    )
    parser.add_argument(
        "--gen-current",
        type=float,
        default=DEFAULT_GEN_CURRENT,
        help=f"Quadrature current for the internal generator in amps (default: {DEFAULT_GEN_CURRENT})",
    )
    parser.add_argument(
        "--gen-frequency",
        type=float,
        default=DEFAULT_GEN_FREQ,
        help=f"Saw-tooth generator frequency in Hz (default: {DEFAULT_GEN_FREQ})",
    )
    parser.add_argument(
        "--max-iterations",
        type=int,
        default=3,
        help="Maximum analog calibration iterations (default: 3)",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Enable debug logging",
    )
    return parser.parse_args()


def main() -> None:
    """Run the iC-Haus magnetic encoder calibration."""
    args = parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)-8s %(name)s: %(message)s",
    )

    mc = MotionController()
    mc.communication.connect_servo_ethercat(
        args.interface,
        slave_id=args.slave_id,
        dict_path=args.dictionary,
    )

    calibrator = EncoderCalibrator(
        mc,
        axis=args.axis,
        max_iterations=args.max_iterations,
        gen_frequency=args.gen_frequency,
        gen_current=args.gen_current,
    )

    encoder_numbers = [1, 2] if args.encoder == "both" else [int(args.encoder)]
    for num in encoder_numbers:
        calibrator.add_encoder(num)

    calibrator.configure_internal_generator()

    results = calibrator.calibrate()

    all_ok = True
    for enc_num, result in results.items():
        status = "SUCCESS" if result.success else "FAILED"
        logger.info(
            "Encoder %d: %s (%d iterations)",
            enc_num,
            status,
            result.iterations,
        )
        if not result.success:
            all_ok = False

    sys.exit(0 if all_ok else 1)


if __name__ == "__main__":
    main()
