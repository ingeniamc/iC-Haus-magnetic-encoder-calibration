import json
import logging

import pytest

from ic_haus_magnetic_encoder_calibration.config_loader import (
    EncoderRegisterConfig,
    _parse_hex_or_int,
    load_encoders_configuration_file,
)
from ic_haus_magnetic_encoder_calibration.ic_haus_registers import (
    CFGEW,
    OUT_LSB_ST,
    OUT_MSB_ZERO,
)


def _valid_dict() -> dict:
    return {
        "OUT_MSB": "0x05",
        "OUT_LSB": "0x00",
        "MODE_ST": "0x00",
        "ENAC": "0x01",
        "CFGEW": "0x00",
        "FILT": "0x02",
    }


def _empty_dict() -> dict:
    return dict.fromkeys(_valid_dict(), "")


class TestParseHexOrInt:
    def test_hex_string(self) -> None:
        """Parse hex string."""
        assert _parse_hex_or_int("0x0E") == 14

    def test_decimal_string(self) -> None:
        """Parse decimal string."""
        assert _parse_hex_or_int("14") == 14

    def test_passthrough_int(self) -> None:
        """Return integer as-is."""
        assert _parse_hex_or_int(14) == 14

    def test_empty_string_raises(self) -> None:
        """Empty string is invalid."""
        with pytest.raises(ValueError, match="empty string"):
            _parse_hex_or_int("")


class TestFromDict:
    def test_parses_all_fields(self) -> None:
        """Parse all fields from valid dict."""
        cfg = EncoderRegisterConfig.from_dict(_valid_dict())
        assert cfg.out_msb == 0x05
        assert cfg.out_lsb == 0x00
        assert cfg.filt == 0x02

    def test_missing_key_raises(self) -> None:
        """Missing required key raises ValueError."""
        data = _valid_dict()
        del data["OUT_MSB"]
        with pytest.raises(ValueError, match="Missing required register keys"):
            EncoderRegisterConfig.from_dict(data)

    def test_invalid_value_raises(self) -> None:
        """Invalid value raises ValueError."""
        data = _valid_dict()
        data["OUT_MSB"] = "not-a-number"
        with pytest.raises(ValueError, match="Invalid register value"):
            EncoderRegisterConfig.from_dict(data)

    def test_empty_value_raises(self) -> None:
        """Empty string value raises ValueError."""
        data = _valid_dict()
        data["OUT_MSB"] = ""
        with pytest.raises(ValueError, match="Invalid register value"):
            EncoderRegisterConfig.from_dict(data)


class TestRegisterWrites:
    """Test that register_writes() returns correct register/field/value tuples."""

    def test_out_msb_targets_out_msb_zero_field(self) -> None:
        """OUT_MSB must target the out_msb field of OUT_MSB_ZERO (0x11)."""
        cfg = EncoderRegisterConfig.from_dict(_valid_dict())
        writes = cfg.register_writes()

        out_msb_write = next(w for w in writes if w[2] == cfg.out_msb and w[0] is OUT_MSB_ZERO)
        register, field, value = out_msb_write
        assert register is OUT_MSB_ZERO
        assert field is OUT_MSB_ZERO.field("out_msb")  # bits 0:4, not the whole reg
        assert field is not OUT_MSB_ZERO.field("out_zero")
        assert value == 0x05

    def test_out_lsb_and_mode_st_target_out_lsb_st(self) -> None:
        """OUT_LSB and MODE_ST must target the correct fields of OUT_LSB_ST."""
        cfg = EncoderRegisterConfig.from_dict(_valid_dict())
        targets = {(r, f) for r, f, _ in cfg.register_writes()}
        assert (OUT_LSB_ST, OUT_LSB_ST.field("out_lsb")) in targets
        assert (OUT_LSB_ST, OUT_LSB_ST.field("mode_st")) in targets

    def test_cfgew_writes_whole_register(self) -> None:
        """CFGEW must write the whole register, not a field."""
        cfg = EncoderRegisterConfig.from_dict(_valid_dict())
        cfgew_write = next(w for w in cfg.register_writes() if w[0] is CFGEW)
        assert cfgew_write[1] is None  # whole-register write


class TestLoadEncodersConfigurationFile:
    """Test loading encoder configurations from JSONfile with load_encoders_configuration_file()."""

    def test_missing_file_raises(self, tmp_path) -> None:
        """Missing config file raises FileNotFoundError."""
        with pytest.raises(FileNotFoundError):
            load_encoders_configuration_file(tmp_path / "nope.json")

    def test_wrong_version_raises(self, tmp_path) -> None:
        """Unsupported version raises ValueError."""
        path = tmp_path / "encoders.json"
        path.write_text(json.dumps({"version": "9.9", "1": _valid_dict()}))
        with pytest.raises(ValueError, match="Unsupported config version"):
            load_encoders_configuration_file(path)

    def test_loads_both_encoders(self, tmp_path) -> None:
        """Load valid config for both encoders."""
        path = tmp_path / "encoders.json"
        path.write_text(json.dumps({"version": "1.0", "1": _valid_dict(), "2": _valid_dict()}))
        configs = load_encoders_configuration_file(path)
        assert set(configs) == {1, 2}
        assert configs[1].out_msb == 0x05

    def test_empty_encoder_config_logs_warning(self, tmp_path, caplog) -> None:
        """If one encoder has all empty values, warning is logged, the other still loads."""
        path = tmp_path / "encoders.json"
        path.write_text(
            json.dumps({
                "version": "1.0",
                "1": _valid_dict(),
                "2": _empty_dict(),
            })
        )

        # Encoder 1 should load successfully, encoder 2 should log a warning and be skipped
        with caplog.at_level(
            logging.WARNING, logger="ic_haus_magnetic_encoder_calibration.config_loader"
        ):
            configs = load_encoders_configuration_file(path)

        assert set(configs) == {1}
        assert configs[1].out_msb == 0x05
        assert 2 not in configs
        assert any(
            "Invalid configuration for encoder 2" in record.message for record in caplog.records
        )

    def test_invalid_encoder_config_logs_warning(self, tmp_path, caplog) -> None:
        """If one encoder has one empty value, warning is logged, the other still loads."""
        path = tmp_path / "encoders.json"
        _one_empty_value_dict = _valid_dict()
        _one_empty_value_dict["OUT_MSB"] = ""
        path.write_text(
            json.dumps({
                "version": "1.0",
                "1": _valid_dict(),
                "2": _one_empty_value_dict,
            })
        )

        # Encoder 1 should load successfully, encoder 2 should log a warning and be skipped
        with caplog.at_level(
            logging.WARNING, logger="ic_haus_magnetic_encoder_calibration.config_loader"
        ):
            configs = load_encoders_configuration_file(path)

        assert set(configs) == {1}
        assert configs[1].out_msb == 0x05
        assert 2 not in configs
        assert any(
            "Invalid configuration for encoder 2" in record.message for record in caplog.records
        )

    def test_no_valid_encoder_configs_raises(self, tmp_path) -> None:
        """If no valid encoder configs are found, ValueError is raised."""
        path = tmp_path / "encoders.json"
        path.write_text(
            json.dumps({
                "version": "1.0",
                "1": _empty_dict(),
                "2": _empty_dict(),
            })
        )

        with pytest.raises(ValueError, match="No valid encoder configs found"):
            load_encoders_configuration_file(path)
