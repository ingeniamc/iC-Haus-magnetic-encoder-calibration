import json
import logging
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional, Union

from .ic_haus_registers import (
    CFGEW,
    ENAC,
    FILT,
    OUT_LSB_ST,
    OUT_MSB_ZERO,
    ICHausRegister,
    ICHausRegisterField,
)

logger = logging.getLogger(__name__)

# JSON file path
DEFAULT_ENCODER_CONFIG_PATH = Path(__file__).resolve().parent.parent / "config" / "encoders.json"

# JSON versioning for encoder register config files
JSON_VERSION_KEY = "version"
JSON_VERSION = "1.0"

# JSON keys for encoder register config
JSON_OUT_MSB_KEY = "OUT_MSB"
JSON_OUT_LSB_KEY = "OUT_LSB"
JSON_MODE_ST_KEY = "MODE_ST"
JSON_ENAC_KEY = "ENAC"
JSON_CFGEW_KEY = "CFGEW"
JSON_FILT_KEY = "FILT"


@dataclass
class EncoderRegisterConfig:
    """Post-calibration iC-MU encoder register configuration.

    These values will be written to the encoder after calibration converges
    but before saving to EEPROM, allowing customization of output frame,
    error configuration, and filter settings per encoder.
    """

    out_msb: int  # Output frame MSB (OUT_MSB) register value
    out_lsb: int  # Output frame LSB (OUT_LSB) register value
    mode_st: int  # Mode and status (MODE_ST) register value
    enac: int  # Enable amplitude control (ENAC) register value
    cfgew: int  # Configuration word (CFGEW) register value
    filt: int  # Filter configuration (FILT) register value

    @classmethod
    def from_dict(cls, data: dict[Any, Any]) -> "EncoderRegisterConfig":
        """Create from JSON dict with hex string values.

        Args:
            data: Dictionary with keys: OUT_MSB, MODE_ST, ENAC, CFGEW, FILT
                  Values can be hex strings ("0x05") or integers.

        Returns:
            EncoderRegisterConfig instance.

        Raises:
            ValueError: If required keys are missing or values are invalid.
        """
        required_keys = {
            JSON_OUT_MSB_KEY,
            JSON_OUT_LSB_KEY,
            JSON_MODE_ST_KEY,
            JSON_ENAC_KEY,
            JSON_CFGEW_KEY,
            JSON_FILT_KEY,
        }
        # Missing keys
        missing_keys = required_keys - set(data.keys())
        if missing_keys:
            msg = f"Missing required register keys: {missing_keys}"
            raise ValueError(msg)

        try:
            return cls(
                out_msb=_parse_hex_or_int(data[JSON_OUT_MSB_KEY]),
                out_lsb=_parse_hex_or_int(data[JSON_OUT_LSB_KEY]),
                mode_st=_parse_hex_or_int(data[JSON_MODE_ST_KEY]),
                enac=_parse_hex_or_int(data[JSON_ENAC_KEY]),
                cfgew=_parse_hex_or_int(data[JSON_CFGEW_KEY]),
                filt=_parse_hex_or_int(data[JSON_FILT_KEY]),
            )
        except (ValueError, TypeError) as e:
            msg = f"Invalid register value in config: {e}"
            raise ValueError(msg) from e

    def register_writes(self) -> list[tuple[Union[ICHausRegisterField, ICHausRegister], int]]:
        """Return (field, value) for each configurable field.

        Each field knows its parent register via the backref, so register
        is not needed here.

        Returns:
        List of (field, value) tuples for writing to hardware.
        """
        return [
            (OUT_MSB_ZERO.field("out_msb"), self.out_msb),
            (OUT_LSB_ST.field("out_lsb"), self.out_lsb),
            (OUT_LSB_ST.field("mode_st"), self.mode_st),
            (ENAC.field("enac"), self.enac),
            (CFGEW, self.cfgew),  # whole register, no field
            (FILT.field("filt"), self.filt),
        ]

    def __str__(self) -> str:
        """Return formatted string of register values."""
        return (
            f"{JSON_OUT_MSB_KEY}={self.out_msb}, {JSON_OUT_LSB_KEY}={self.out_lsb}, "
            f"{JSON_MODE_ST_KEY}={self.mode_st}, {JSON_ENAC_KEY}={self.enac}, "
            f"{JSON_CFGEW_KEY}={self.cfgew}, {JSON_FILT_KEY}={self.filt}"
        )


def _parse_hex_or_int(value: Union[str, int]) -> int:
    """Parse a value that can be either a hex string or integer.

    Args:
        value: Either "0xNN" string, "NN" string, or integer.

    Returns:
        Parsed integer value.

    Raises:
        ValueError: If the value cannot be parsed.
    """
    if isinstance(value, int):
        return value

    if isinstance(value, str):
        value = value.strip()
        if not value:
            msg = "Cannot parse empty string"
            raise ValueError(msg)
        if value.lower().startswith("0x"):
            return int(value, 16)
        return int(value)

    msg = f"Cannot parse value: {value!r} (type: {type(value).__name__})"
    raise ValueError(msg)


def load_encoders_configuration_file(
    config_file: Optional[Path] = DEFAULT_ENCODER_CONFIG_PATH,
) -> dict[int, EncoderRegisterConfig]:
    """Load encoder configurations from JSON file.

    Loader validates the JSON format but does not enforce which encoders must be present.
    Caller must check that the required encoder configurations are available.
    If a configuration for an encoder is invalid, it will be skipped with a warning.

    Expected JSON structure:
    ```json
    {
        "version": "1.0",
        "1": {
            "OUT_MSB": "0x05",
            "OUT_LSB": "0x00",
            "MODE_ST": "0x00",
            "ENAC": "0x01",
            "CFGEW": "0xff",
            "FILT": "0x03"
        },
        "2": { ... }
    }
    ```

    Args:
        config_file: Path to JSON configuration file.

    Returns:
        Dictionary mapping encoder numbers (1, 2) to EncoderRegisterConfig objects.

    Raises:
        FileNotFoundError: If config file does not exist.
        json.JSONDecodeError: If config file is not valid JSON.
        ValueError: If configuration format is invalid or no valid configurations are found.
    """
    if not config_file:
        config_file = DEFAULT_ENCODER_CONFIG_PATH
    if not config_file.exists():
        msg = f"Encoder config file not found: {config_file}"
        raise FileNotFoundError(msg)

    try:
        with open(config_file) as f:
            data = json.load(f)
    except json.JSONDecodeError as e:
        msg = f"Invalid JSON in encoder config file: {e}"
        raise ValueError(msg) from e

    # Check version
    version = data.get(JSON_VERSION_KEY)
    if version != JSON_VERSION:
        msg = f"Unsupported config version: {version}, expected {JSON_VERSION}"
        raise ValueError(msg)

    configs: dict[int, EncoderRegisterConfig] = {}
    for enc_key in ["1", "2"]:
        if enc_key in data:
            enc_num = int(enc_key)
            try:
                configs[enc_num] = EncoderRegisterConfig.from_dict(data[enc_key])
            except ValueError as e:
                # Log the error but continue processing other encoders
                logger.warning(f"Invalid configuration for encoder {enc_num}: {e}")

    if not configs:
        msg = f"No valid encoder configs found in {config_file}"
        raise ValueError(msg)

    return configs  # may be empty — caller decides what's required
