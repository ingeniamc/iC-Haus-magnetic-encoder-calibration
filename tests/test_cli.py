import subprocess
import sys


def test_cli_help_exits_successfully() -> None:
    result = subprocess.run(
        [sys.executable, "__main__.py", "--help"],
        capture_output=True,
        text=True,
        timeout=10,
    )
    assert result.returncode == 0
    assert "iC-Haus Magnetic Encoder Calibration" in result.stdout
