import importlib.util
import subprocess
import sys
from pathlib import Path

from ic_haus_magnetic_encoder_calibration.calibrator import NONIUS_IN_RANGE_RECOMMENDED_MAX_PERCENT

# Load the CLI module from __main__.py to access the parse_args function for testing.
_MAIN_PATH = Path(__file__).resolve().parents[1] / "__main__.py"
_spec = importlib.util.spec_from_file_location("ic_haus_cli", _MAIN_PATH)
_cli = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_cli)
parse_args = _cli.parse_args


def test_cli_help_exits_successfully() -> None:
    result = subprocess.run(
        [sys.executable, "__main__.py", "--help"],
        capture_output=True,
        text=True,
        timeout=10,
    )
    assert result.returncode == 0
    assert "iC-Haus Magnetic Encoder Calibration" in result.stdout


def test_force_in_range_omitted_is_none(monkeypatch) -> None:
    """Verify that when the --force-in-range flag is omitted, the value is None."""
    monkeypatch.setattr(sys, "argv", ["__main__.py", "--interface", "x", "--dictionary", "y"])
    args = parse_args()
    assert args.force_in_range is None


def test_force_in_range_flag_without_value_uses_default(monkeypatch) -> None:
    """Verify that when the --force-in-range flag is provided without a value, default is used."""
    monkeypatch.setattr(
        sys, "argv", ["__main__.py", "--interface", "x", "--dictionary", "y", "--force-in-range"]
    )
    args = parse_args()
    assert args.force_in_range == NONIUS_IN_RANGE_RECOMMENDED_MAX_PERCENT


def test_force_in_range_with_explicit_value(monkeypatch) -> None:
    """Verify that when the --force-in-range flag is provided with an explicit value, it is used."""
    monkeypatch.setattr(
        sys,
        "argv",
        ["__main__.py", "--interface", "x", "--dictionary", "y", "--force-in-range", "70"],
    )
    args = parse_args()
    assert args.force_in_range == 70.0
