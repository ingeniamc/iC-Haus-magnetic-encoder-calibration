"""Entry point for the iC-Haus Magnetic Encoder Calibration tool."""

import argparse
import logging
import sys
from pathlib import Path

from ingeniamotion import MotionController
from ingeniamotion.enums import SensorType

from ic_haus_magnetic_encoder_calibration.calibrator import (
    DEFAULT_CAPTURE_DURATION_S,
    DEFAULT_PDO_RATE_S,
    EncoderCalibrator,
)
from ic_haus_magnetic_encoder_calibration.motor_control import (
    DEFAULT_GEN_CURRENT,
    DEFAULT_GEN_FREQ,
)

logger = logging.getLogger("ic_haus_magnetic_encoder_calibration")


def _parse_bool(value: str) -> bool:
    """Parse a boolean CLI argument value.

    Args:
        value: The string value to parse.

    Returns:
        The parsed boolean value.

    Raises:
        argparse.ArgumentTypeError: If the value cannot be parsed as a boolean.
    """
    if value.lower() in ("true", "1", "yes"):
        return True
    if value.lower() in ("false", "0", "no"):
        return False
    msg = f"Expected true/false, got {value!r}"
    raise argparse.ArgumentTypeError(msg)


def _positive_float(value: str) -> float:
    """Parse a strictly-positive float CLI argument value.

    Args:
        value: The string value to parse.

    Returns:
        The parsed float value.

    Raises:
        argparse.ArgumentTypeError: If the value is not a strictly-positive float.
    """
    number = float(value)
    if number <= 0:
        msg = f"Expected a positive float number, got {value!r}"
        raise argparse.ArgumentTypeError(msg)
    return number


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
        help=(
            "Quadrature current for the internal generator in amps"
            f" (default: {DEFAULT_GEN_CURRENT})"
        ),
    )
    parser.add_argument(
        "--gen-frequency",
        type=_positive_float,
        default=DEFAULT_GEN_FREQ,
        help=f"Saw-tooth generator frequency in Hz (default: {DEFAULT_GEN_FREQ})",
    )
    parser.add_argument(
        "--max-iterations",
        type=int,
        default=10,
        help="Maximum analog calibration iterations (default: 10)",
    )
    parser.add_argument(
        "--pdo-rate-ms",
        type=float,
        default=DEFAULT_PDO_RATE_S * 1000,
        help=(f"PDO cycle time in milliseconds (default: {DEFAULT_PDO_RATE_S * 1000})"),
    )
    parser.add_argument(
        "--capture-duration",
        type=float,
        default=DEFAULT_CAPTURE_DURATION_S,
        help=(
            "Data capture duration per iteration in seconds"
            f" (default: {DEFAULT_CAPTURE_DURATION_S})"
        ),
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("calibration_output"),
        help="Directory for diagnostic plot PNGs (default: calibration_output)",
    )
    parser.add_argument(
        "--save-raw-plots",
        type=_parse_bool,
        default=False,
        metavar="BOOL",
        help="Save per-iteration raw waveform PNGs (default: false)",
    )
    parser.add_argument(
        "--save-residual-bar-plots",
        type=_parse_bool,
        default=False,
        metavar="BOOL",
        help="Save per-iteration residual bar chart PNGs (default: false)",
    )
    parser.add_argument(
        "--save-trend-plot",
        type=_parse_bool,
        default=True,
        metavar="BOOL",
        help="Save the residuals trend PNG (default: true)",
    )
    parser.add_argument(
        "--save-json",
        type=_parse_bool,
        default=True,
        metavar="BOOL",
        help="Save JSON export of calibration data (default: true)",
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=None,
        help="Path to an XCF configuration file to load onto the drive before calibration",
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

    if args.config is not None:
        mc.configuration.load_configuration(str(args.config))
        logger.info(f"Loaded configuration: {args.config}")

    calibrator = EncoderCalibrator(
        mc,
        axis=args.axis,
        max_iterations=args.max_iterations,
        gen_frequency=args.gen_frequency,
        gen_current=args.gen_current,
        pdo_rate=args.pdo_rate_ms / 1000.0,
        capture_duration=args.capture_duration,
        output_dir=args.output_dir,
        save_raw_plots=args.save_raw_plots,
        save_residual_bar_plots=args.save_residual_bar_plots,
        save_trend_plot=args.save_trend_plot,
        save_json=args.save_json,
    )

    encoder_sensor_types = {1: SensorType.ABS1, 2: SensorType.SSI2}
    encoder_numbers = [1, 2] if args.encoder == "both" else [int(args.encoder)]
    for num in encoder_numbers:
        calibrator.add_encoder(encoder_sensor_types[num])

    calibrator.configure_drive_encoders()

    results = calibrator.calibrate()

    all_ok = True
    for enc_num, result in results.items():
        status = "SUCCESS" if result.success else "FAILED"
        logger.info(f"Encoder {enc_num}: {status} ({result.iterations} iterations)")
        if not result.success:
            all_ok = False

    sys.exit(0 if all_ok else 1)


if __name__ == "__main__":
    main()
