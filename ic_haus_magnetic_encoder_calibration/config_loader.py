import json
import logging
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

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
DEFAULT_ENCODER_CONFIG_PATH = Path("config/encoders.json")

# JSON versioning for encoder register config files
JSON_VERSION_KEY = "version"
JSON_VERSION = "1.0"

# JSON keys for encoder register config
JSON_OUT_MSB_KEY = "OUT_MSB"
JSON_MODE_ST_KEY = "MODE_ST"
JSON_ENAC_KEY = "ENAC"
JSON_CFGEW_KEY = "CFGEW"
JSON_FILT_KEY = "FILT"


@dataclass(frozen=True)
class RegisterTarget:
    """Links a config key to a register and optional field."""

    register: ICHausRegister
    register_field: Optional[ICHausRegisterField] = (
        None  # Register bitfield, or None for whole register
    )


@dataclass
class EncoderRegisterConfig:
    """Post-calibration iC-MU encoder register configuration.

    These values will be written to the encoder after calibration converges
    but before saving to EEPROM, allowing customization of output frame,
    error configuration, and filter settings per encoder.
    """

    out_msb: int  # Output frame MSB (OUT_MSB) register value
    mode_st: int  # Mode and status (MODE_ST) register value
    enac: int  # Enable amplitude control (ENAC) register value
    cfgew: int  # Configuration word (CFGEW) register value
    filt: int  # Filter configuration (FILT) register value

    @classmethod
    def from_dict(cls, data: dict) -> "EncoderRegisterConfig":
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
            JSON_MODE_ST_KEY,
            JSON_ENAC_KEY,
            JSON_CFGEW_KEY,
            JSON_FILT_KEY,
        }
        missing_keys = required_keys - set(data.keys())
        if missing_keys:
            msg = f"Missing required register keys: {missing_keys}"
            raise ValueError(msg)

        try:
            return cls(
                out_msb=_parse_hex_or_int(data[JSON_OUT_MSB_KEY]),
                mode_st=_parse_hex_or_int(data[JSON_MODE_ST_KEY]),
                enac=_parse_hex_or_int(data[JSON_ENAC_KEY]),
                cfgew=_parse_hex_or_int(data[JSON_CFGEW_KEY]),
                filt=_parse_hex_or_int(data[JSON_FILT_KEY]),
            )
        except (ValueError, TypeError) as e:
            msg = f"Invalid register value in config: {e}"
            raise ValueError(msg) from e

    def register_writes(
        self,
    ) -> list[tuple[ICHausRegister, Optional[ICHausRegisterField], int]]:
        """Yield (register, field, value) for each configurable register.

        - register: ICHausRegister object to write to.
        - field: ICHausRegisterField object if writing a specific bit-field, or None for whole register.
        - value: Integer value to write to the register or field.

        Returns:
            List of tuples for each register to write.
        """
        return [
            (OUT_MSB_ZERO, OUT_MSB_ZERO.field("out_msb"), self.out_msb),
            (OUT_LSB_ST, OUT_LSB_ST.field("mode_st"), self.mode_st),
            (ENAC, ENAC.field("enac"), self.enac),
            (CFGEW, None, self.cfgew),
            (FILT, FILT.field("filt"), self.filt),
        ]


def _parse_hex_or_int(value: str | int) -> int:
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
        if value.lower().startswith("0x"):
            return int(value, 16)
        return int(value)

    msg = f"Cannot parse value: {value!r} (type: {type(value).__name__})"
    raise ValueError(msg)


def load_configuration_file(
    config_file: Optional[Path] = DEFAULT_ENCODER_CONFIG_PATH,
) -> dict[int, EncoderRegisterConfig]:
    """Load encoder configurations from JSON file.

    # TODO: Update to latest JSON format and validate against schema.
    Expected JSON structure:
    ```json
    {
        "version": "1.0",
        "1": {
            "OUT_MSB": "0x05",
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
        ValueError: If configuration format or values are invalid.
    """
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

    configs = {}
    for enc_key in ["1", "2"]:
        if enc_key in data:
            enc_num = int(enc_key)
            try:
                configs[enc_num] = EncoderRegisterConfig.from_dict(data[enc_key])
            except ValueError as e:
                msg = f"Invalid config for {enc_key}: {e}"
                raise ValueError(msg) from e

    if not configs:
        msg = f"No encoder configs found in {config_file}"
        raise ValueError(msg)

    return configs


def get_config_for_encoder(
    config_file: Optional[Path],
    encoder_num: int,
) -> Optional[EncoderRegisterConfig]:
    """Load configuration for a specific encoder from file.

    Args:
        config_file: Path to config file, or None to skip.
        encoder_num: Encoder number (1 or 2).

    Returns:
        EncoderRegisterConfig if found, None if file is not provided or
        encoder not found in file.

    Raises:
        ValueError: If file exists but has invalid format.
    """
    if config_file is None:
        return None

    try:
        configs = load_configuration_file(config_file)
        return configs.get(encoder_num)
    except (FileNotFoundError, json.JSONDecodeError):
        return None
