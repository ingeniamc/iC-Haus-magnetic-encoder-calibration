# config/

This folder holds the file that contains the default configuration for the encoders.

## How it works

The Calibrator loads the configuration file and applies the configuration to each encoder.

## Configurable registers

| Register | Address | Bits | |
|------|---------|---------|---------|
| OUT_MSB | 0x11 | 4:0 | Output shift register configuration: MSB used bits |
| OUT_LSB | 0x12 | 3:0 | Output shift register configuration: LSB used bits |
| MODE_ST | 0x12 | 5:4 | Data output  |
| ENAC | 0x05 | 7 | Amplitude control unit activation (Activation of the automatic-gain-control) |
| CFGEW | 0x0C | 7:0 | Error and warning bit configuration |
| FILT | 0x0E | 2:0 | Digital filter settings |

Check encoder manual for more information.