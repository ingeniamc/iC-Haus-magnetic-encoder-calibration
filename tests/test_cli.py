import importlib.util
import subprocess
import sys
from pathlib import Path

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
