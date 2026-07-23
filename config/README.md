# config/

This folder holds the configuration file that defines register values to be applied to the encoders.

## How it works

Before calibration, the Calibrator loads the configuration file (`encoders.json`) and applies the register configuration to each encoder. After calibration completes (whether successful or not), the encoders will have this configuration applied.
The script will fail if the configuration is not present.


## Configuration file format

The configuration is stored in `encoders.json` in the following structure:

```json
{
  "version": "1.0",
  "1": {
    "OUT_MSB": "",
    "OUT_LSB": "",
    "MODE_ST": "",
    "ENAC": "",
    "CFGEW": "",
    "FILT": ""
  },
  "2": {
    "OUT_MSB": "",
    "OUT_LSB": "",
    "MODE_ST": "",
    "ENAC": "",
    "CFGEW": "",
    "FILT": ""
  }
}
```

**Notes:**

- The "1" and "2" keys correspond to encoder channels 1 and 2. Provide configurations for one or both encoders as needed.
- Register values can be specified as hex strings (e.g., "0x05") or decimal integers (e.g., 5).
- The "version" field must be "1.0" for the current format.

## Configurable registers

| Register | Address | Bits | Comment |
|------|---------|---------|---------|
| OUT_MSB | 0x11 | 4:0 | Output shift register configuration: MSB used bits |
| OUT_LSB | 0x12 | 3:0 | Output shift register configuration: LSB used bits |
| MODE_ST | 0x12 | 5:4 | Data output  |
| ENAC | 0x05 | 7 | Amplitude control unit activation (Activation of the automatic-gain-control) |
| CFGEW | 0x0C | 7:0 | Error and warning bit configuration |
| FILT | 0x0E | 2:0 | Digital filter settings |

For detailed register definitions and valid value ranges, consult the iC-MU encoder datasheet.