# Architecture: iC-Haus Magnetic Encoder Calibration

Calibration tool for iC-MU magnetic encoders on the DR3256C drive, using BiSS over EtherCAT.

---

## Table of Contents

- [0. How the iC-MU Encoder Works](#0-how-the-ic-mu-encoder-works)
  - [Physical Principle: Two-Track Magnetic Nonius](#physical-principle-two-track-magnetic-nonius)
  - [Analog Signal Conditioning](#analog-signal-conditioning)
  - [Sine → Digital Conversion](#sine--digital-conversion)
  - [Operating Modes](#operating-modes)
  - [BiSS-C Communication](#biss-c-communication)
  - [Error / Warning Bit Configuration (CFGEW)](#error--warning-bit-configuration-cfgew)
  - [Uncalibrated Encoder → Drive Fault Chain](#uncalibrated-encoder--drive-fault-chain)
  - [Internal CRC Checksums (EEPROM)](#internal-crc-checksums-eeprom)
- [1. Module Structure](#1-module-structure)
- [2. Calibration Flow](#2-calibration-flow)
- [3. Class Diagram](#3-class-diagram)
- [4. Design Notes](#4-design-notes)

---

## 0. How the iC-MU Encoder Works

### Physical Principle: Two-Track Magnetic Nonius

The iC-MU is a **magnetic off-axis position encoder** using integrated Hall sensors
to scan a ring magnet attached to the motor shaft. It uses the **Nonius principle**
with two sensing tracks of different pole-pair counts:

| Track | Pole pairs | Role |
|-------|-----------|------|
| **Master** | Higher (e.g. 11) | High resolution *within* one pole-pair period |
| **Nonius** | Lower (e.g. 8) | Determines *which* period → absolute position |

By combining both tracks the encoder resolves a unique absolute position across
one full mechanical revolution.

### Analog Signal Conditioning

Each track produces sine + cosine Hall signals. These are conditioned by 4
parameters per track (8 total), stored in EEPROM:

| Param | Master reg | Nonius reg | Purpose |
|-------|-----------|-----------|---------|
| GX    | 0x01      | 0x07      | Cosine gain (amplitude matching) |
| VOSS  | 0x02      | 0x08      | Sine DC offset removal |
| VOSC  | 0x03      | 0x09      | Cosine DC offset removal |
| PH    | 0x04      | 0x0A      | Phase / orthogonality correction |

Factory defaults are generic. **Calibration tunes them for the specific
motor + magnet + sensor assembly**.

### Sine → Digital Conversion

Internal 12-bit (14-bit filtered) converters turn the conditioned sine/cosine
into digital position words:

- **Master position** — 14 bits within one master period
- **Nonius position** — 14 bits within one nonius period

The Nonius calculation engine combines these into an absolute position.
A per-sector offset table (SPO_BASE + SPO\_0…SPO\_14) adds fine correction.

### Operating Modes

**Normal mode** — encoder outputs combined absolute position via BiSS-C:

```
SCD: [19-bit absolute position | nE | nW | CRC-6]  (27 bits)
```

**Raw / calibration mode** (`MODE_ST = RAW`, `OUT_MSB = 0x0E`):

```
SCD: [14-bit nonius | 14-bit master | nE | nW | CRC-6]  (36 bits)
```

Raw mode exposes each track independently so the calibration algorithm can
analyse and correct analog errors per-track.

### BiSS-C Communication

BiSS-C is a synchronous point-to-point serial protocol:

1. Drive (master) generates clock on **MA** line
2. Encoder asserts **ACK** on SCD when data is ready
3. Position data, MSB first
4. **nE** — error bit (active LOW)
5. **nW** — warning bit (active LOW)
6. **CRC** — 6-bit, inverted, polynomial 0x43

### Error / Warning Bit Configuration (CFGEW)

Register **CFGEW** (0x0C) selects which conditions appear on nE / nW.
Encoding: `0 = enabled`, `1 = disabled`.

| Bit | Visible on nE (error) | Condition |
|-----|----------------------|-----------|
| 7 | MT_ERR / MT_CTR | Multiturn errors |
| 6 | NON_CTR | Nonius period-counter error |
| **5** | **Ax_MAX, Ax_MIN** | **Signal-level errors** |
| 4 | EPR_ERR | EEPROM error |
| 3 | CRC_ERR | Internal RAM CRC error |
| 2 | CMD_EXE | Command executing |

| Bit | Visible on nW (warning) | Condition |
|-----|------------------------|-----------|
| 1 | FRQ_CNV / FRQ_ABZ | Frequency warning |
| 0 | Ax_MAX, Ax_MIN | Signal-level warning |

With **CFGEW.bit5 = 0** (default), signal-level errors
(`AM_MIN` / `AN_MIN` in STATUS0) will assert **nE = 0** on every BiSS frame.
An uncalibrated encoder whose signal amplitude is below threshold will
therefore report an error on every frame.

### Uncalibrated Encoder → Drive Fault Chain

When the calibration script enters raw mode and sets **FRAME_SIZE = 36** on
the drive, the full BiSS frame — including nE — is clocked. If the encoder is
uncalibrated:

1. Factory-default analog parameters → signal amplitude may be out of range
2. `AM_MIN` or `AN_MIN` fires (STATUS0), or `CRC_ERR` (STATUS1) asserts
   because the internal EEPROM CRC is invalid for an uncalibrated chip
3. `CFGEW.bit5 = 0` → error propagates to **nE = 0** (active low)
4. Drive reads nE = 0 on each frame → counts a BiSS error
5. After enough consecutive errors → **drive faults** with 0x7380 or 0x7382
6. `motor_enable()` fails

The calibration script mitigates this by writing **`CFGEW = 0xFF`** to the
iC-MU, which disables all error sources from asserting the BiSS nE/nW bits.
This is set in `configure_in_calibration_mode()` and restored to the original
value when calibration mode exits.

The drive-side **`ERROR_TOLERANCE`** register is also set to `0xFFFF` after
the frame geometry change.  Switching from the normal frame (27 bits) to the
calibration raw frame (36 bits) causes transient CRC mismatches; a high
tolerance prevents the drive from freezing POS_VALUE during the transition.

Additionally, `ensure_normal_mode()` is called at the start of every
calibration run to detect and recover an encoder left in RAW mode from a
previous interrupted calibration (e.g., `OUT_MSB = 0x0E` instead of `0x06`).

### Internal CRC Checksums (EEPROM)

The encoder also stores two internal CRCs protecting its configuration RAM:

| CRC | Polynomial | Protects | Address |
|-----|-----------|----------|---------|
| CRC-16 | 0x11021 | Config data (0x00–0x20, 0x30–0x3F) | 0x21–0x22 |
| CRC-8 | 0x197 | Offset/preset data (0x23–0x2E) | 0x2F |

These are checked at startup and optionally during operation (`NCHK_CRC`).
The `WRITE_ALL` command (0x01) automatically recalculates both CRCs.
`STATUS1.CRC_ERR` (bit 7) indicates an invalid internal checksum — this is
distinct from the BiSS frame CRC.

---

## 1. Module Structure

```mermaid
flowchart TD
    MAIN["__main__.py<br/>CLI entry point<br/>(argparse)"]
    ICREG["ic_haus_registers.py<br/>iC-MU register descriptors<br/>ICHausRegister / ICHausRegisterField<br/>BissAction enum"]
    DRVREG["drive_encoder_registers.py<br/>DriveEncoderRegisters dataclass<br/>Drive register name mappings"]
    ENC["encoder.py<br/>Encoder class<br/>Single encoder operations<br/>BiSS R/W, save/restore,<br/>CalibrationResult dataclass"]
    MOT["motor_control.py<br/>MotorControl class<br/>FSoE lifecycle, motor_spinning(),<br/>configure_encoders() + current ramp"]
    CAL["calibrator.py<br/>EncoderCalibrator class<br/>_SingleEncoderCalibration per-encoder state<br/>TPDO data acquisition, diagnostic plots"]
    PLOT["plotting.py<br/>Diagnostic plot functions<br/>raw waveforms, residual bars, trend"]

    MU([mu_3sl — DLL wrapper — External])
    IM([ingeniamotion — External])

    MAIN -->|"parses args, creates mc,<br/>configure_encoders()"| CAL
    CAL -->|"orchestrates"| ENC
    CAL -->|"delegates motor ops"| MOT
    CAL -->|"diagnostic plots"| PLOT
    MOT -->|"FSoE + motor control"| IM
    ENC -->|"uses chip register defs"| ICREG
    ENC -->|"uses drive register names"| DRVREG
    ENC -->|"calibration math"| MU
    ENC -->|"BiSS comms"| IM
```


---

## 2. Calibration Flow

```mermaid
flowchart TD
    CLI["**CLI** (__main__.py)<br/>--interface, --dictionary,<br/>--encoder 1|2|both, --axis,<br/>--gen-current, --gen-frequency,<br/>--max-iterations, --pdo-rate-ms,<br/>--capture-duration,<br/>--save-raw-plots, --save-residual-bar-plots,<br/>--save-trend-plot, --save-json"]

    CLI --> CONNECT["Connect to drive via EtherCAT"]
    CONNECT --> CREATE["Create EncoderCalibrator<br/>(wraps MotorControl internally)<br/>Add Encoder(sensor_type) for each encoder"]

    CREATE --> CONFIGURE["**configure_encoders()**<br/>Sensors: INTGEN (vel/pos/commu)<br/>+ enrolled encoder sensor types as<br/>auxiliary / reference feedback"]

    CONFIGURE --> CALIBRATE["**calibrator.calibrate()**"]

    CALIBRATE --> SETUP["**For each Encoder — setup phases:**<br/>1. ensure_normal_mode() (crash recovery)<br/>2. Read revision, save drive config, save iC-MU config<br/>3. configure_in_calibration_mode() (CFGEW=0xFF)<br/>4. reset_analog_to_factory_defaults()"]

    SETUP --> TPDO["**Setup data TPDO**<br/>Register TPDO map with<br/>encoder pos_value registers<br/>(before FSoE maps)"]
    TPDO --> FSOE["**Prepare FSoE** (if applicable)<br/>Safety PDO maps registered"]
    FSOE --> PDO_START["**Activate all PDOs**<br/>(FSoE + data start together)"]

    PDO_START --> MOTOR_START["**Start motor**<br/>motor_spinning() context manager<br/>(runs for entire calibration session)"]

    MOTOR_START --> LOOP{"iteration ≤ max_iterations?"}

    LOOP -- Yes --> ACQ["**Acquire raw data**<br/>_acquire_raw_data() collects<br/>TPDO samples for capture_duration"]

    ACQ --> PER_ENC["**For each pending Encoder:**<br/>process_iteration()"]
    PER_ENC --> READ_PARAMS["Read current analog params from chip<br/>set_current_analog_track_adjustments()<br/>(sync DLL with chip state)"]
    READ_PARAMS --> ANALYZE["analyze_raw_data(master, nonius)<br/>(split_raw_payload unpacks packed values)"]

    ANALYZE --> CHECK{"All 8 residuals<br/>≤ 1.0 LSB?"}
    CHECK -- Yes --> MARK["Mark encoder as converged<br/>Store last_analyze_result"]
    CHECK -- No --> ADJUST["adjust_analog_by_analyze_result()<br/>Write new params to chip"]

    ADJUST --> NEXT_ENC{"More encoders?"}
    MARK --> NEXT_ENC
    NEXT_ENC -- Yes --> PER_ENC
    NEXT_ENC -- No --> ALL_CONV{"All encoders<br/>converged?"}
    ALL_CONV -- Yes --> FINALIZE
    ALL_CONV -- No --> LOOP

    LOOP -- No --> FAIL["Non-converged encoders:<br/>CalibrationResult(success=False)"]
    FAIL --> MOTOR_STOP

    FINALIZE["**For each converged Encoder:**<br/>finalize():<br/>Optimize nonius SPO table using stored last_analyze_result<br/>Write SPO params to chip<br/>Restore iC-MU config (set_ic_config)<br/>Enable all errors (CFGEW=0x00)<br/>Save to EEPROM (WRITE_ALL)<br/>ABS_RESET (clear startup NON_CTR)"]
    FINALIZE --> MOTOR_STOP

    MOTOR_STOP["**Stop motor**<br/>(motor_spinning context exits)"]
    MOTOR_STOP --> TEARDOWN["Export JSON data (if --save-json)<br/>Teardown data TPDO<br/>Stop PDOs and FSoE"]
    TEARDOWN --> CLEANUP

    CLEANUP["**For all Encoders:**<br/>restore_state() per encoder:<br/>Restore iC-MU config (set_ic_config)<br/>Enable all errors (CFGEW=0x00)<br/>Restore drive config (set_drive_config)"]
    CLEANUP --> RESULT["Return dict[encoder_number, CalibrationResult]"]
```

---

## 3. Class Diagram

```mermaid
classDiagram
    class ICHausRegisterField {
        <<dataclass, frozen>>
        +mask: int
        +shift: int
        +name: str
        +from_bits(low, high, name) ICHausRegisterField
        +extract(raw) int
        +insert(raw, value) int
    }

    class ICHausRegister {
        +address: int
        +name: str
        +field(name) ICHausRegisterField
        +field_names: tuple
    }

    class DriveEncoderRegisters {
        <<dataclass, frozen>>
        +itf_addr: str
        +itf_data: str
        +itf_ctl: str
        +pos_value: str
        +frame_size: str
        +pos_bits: str
        +pos_st_bits: str
        +pos_start_bit: str
        +error_tolerance: str
    }

    class DriveFrameConfig {
        <<dataclass, frozen>>
        +frame_size: int
        +pos_bits: int
        +pos_st_bits: int
        +pos_start_bit: int
        +error_tolerance: int
    }

    class ICMURegisterState {
        <<dataclass, frozen>>
        +enac: int
        +modea_modeb: int
        +out_msb_zero: int
        +out_lsb_st: int
        +test: int
        +mpc: int
        +cfgew: int
    }

    class CalibrationResult {
        <<dataclass>>
        +success: bool
        +iterations: int
        +master_adjustments: AnalogTrackAdjustments?
        +nonius_adjustments: AnalogTrackAdjustments?
        +spo_base: int
        +spo_n: list~int~
    }

    class Encoder {
        -_mc: MotionController
        -_sensor_type: SensorType
        -_number: int
        -_axis: int
        -_regs: DriveEncoderRegisters
        +number: int
        +sensor_type: SensorType
        +regs: DriveEncoderRegisters
        +read_revision() Revision
        +get_drive_config() DriveFrameConfig
        +set_drive_config(config)
        +get_ic_config() ICMURegisterState
        +set_ic_config(state)
        +ensure_normal_mode() bool
        +configure_in_calibration_mode() int
        +read_analog_adjustments() tuple
        +write_analog_adjustments(master, nonius)
        +reset_analog_to_factory_defaults()
        +write_nonius_parameters(table_params)
        +save_to_eeprom() bool
        +enable_all_errors()
        +abs_reset()
        -_read_ic(reg) int
        -_write_ic(reg, value)
        -_read_drive(name) int
        -_write_drive(name, value)
    }

    class MotorControl {
        -_mc: MotionController
        -_axis: int
        -_fsoe_active: bool
        -_fsoe_prepared: bool
        -_handler: FSoEMasterHandler?
        -_gen_frequency: float
        -_gen_current: float
        +mc: MotionController
        +gen_frequency: float
        +has_fsoe: bool
        +configure_encoders(encoder_sensor_types)
        +prepare_fsoe()
        +activate_pdos(refresh_rate)
        +stop_pdos_and_fsoe()
        +motor_spinning() ContextManager
        -_start_motor()
        -_stop_motor()
    }

    class _SingleEncoderCalibration {
        +enc: Encoder
        +n_master_periods: int
        +saved_drive_config: DriveFrameConfig?
        +saved_ic_config: ICMURegisterState?
        +converged: bool
        +iteration_count: int
        +residual_history: list~list~float~~
        +iteration_log: list~dict~
        +last_analyze_result: AnalyzeResult?
        -_cal: Calibration?
        +number: int
        +pending: bool
        +cal: Calibration
        +save_state()
        +enter_calibration_mode()
        +reset_analog()
        +is_converged(analyze_result) bool$
        +process_iteration(iteration, raw_data, ...)
        +restore_state()
        +export_data(output_dir)
        +finalize() CalibrationResult
    }

    class EncoderCalibrator {
        -_mc: MotionController
        -_motor: MotorControl
        -_encoders: list~Encoder~
        -_axis: int
        -_max_iterations: int
        -_pdo_rate: float
        -_capture_duration: float
        -_output_dir: Path
        -_save_raw_plots: bool
        -_save_residual_bar_plots: bool
        -_save_trend_plot: bool
        -_save_json: bool
        -_tpdo_map: TPDOMap?
        -_pdo_buffer: deque
        -_pdo_lock: Lock
        -_pdo_collecting: bool
        +encoders: list~Encoder~
        +add_encoder(sensor_type) Encoder
        +configure_encoders()
        +calibrate() dict~int, CalibrationResult~
        -_setup_data_tpdo()
        -_teardown_data_tpdo()
        -_on_pdo_data()
        -_acquire_raw_data() dict~int, list~int~~
    }

    class plotting {
        <<module>>
        +RESIDUAL_THRESHOLD: float
        +_plot_raw_waveforms(master, nonius, ...)
        +_plot_residuals_bar(residuals, ...)
        +_plot_residuals_trend(history, ...)
    }

    ICHausRegister --> "*" ICHausRegisterField : contains
    Encoder --> DriveEncoderRegisters : uses
    Encoder --> ICHausRegister : reads/writes via BiSS
    _SingleEncoderCalibration --> Encoder : wraps
    EncoderCalibrator --> "*" _SingleEncoderCalibration : creates per encoder
    EncoderCalibrator --> "*" Encoder : registers
    EncoderCalibrator --> MotorControl : delegates motor ops
    EncoderCalibrator --> plotting : diagnostic plots
    MotorControl --> MotionController : FSoE + motor
    Encoder ..> DriveFrameConfig : get/set
    Encoder ..> ICMURegisterState : get/set
    _SingleEncoderCalibration ..> CalibrationResult : produces
```

> **Encoder**: Wraps a single iC-MU encoder — BiSS read/write, register save/restore via get/set pattern, analog parameter management, nonius SPO writes, factory default reset, EEPROM save. `ensure_normal_mode()` detects and recovers from interrupted calibration runs. State is not stored internally; the caller (`_SingleEncoderCalibration`) manages saved configs.
>
> **MotorControl**: Wraps motor operations with transparent FSoE support. Auto-detects drive safety capability, manages the full FSoE lifecycle (prepare/activate/stop), and handles internal generator configuration with current ramp-up to avoid FSoE PDO starvation. The `motor_spinning()` context manager starts and stops the motor; the motor runs continuously for the entire calibration session.
>
> **_SingleEncoderCalibration**: Per-encoder calibration state and iteration logic. Tracks calibration progress through setup (save state, enter calibration mode, reset analog), iterative analysis (`process_iteration()`), and cleanup (`restore_state()`, `finalize()`). Owns the mu_3sl `Calibration` object and stores residual history, iteration log, and the last analysis result.
>
> **EncoderCalibrator**: Orchestrates calibration across N encoders. Delegates all motor and FSoE operations to an internal `MotorControl` instance, manages TPDO data acquisition, and coordinates the calibration loop. Motor runs continuously for the entire session via `motor_spinning()`; data is captured from all encoders simultaneously via EtherCAT TPDOs.
>
> **plotting**: Module-level diagnostic plot functions for raw waveforms, per-iteration residual bar charts, and cumulative residual trend lines. Each figure is saved as PNG and optionally shown interactively.

---

## 4. Design Notes

- **Per-encoder state**: `_SingleEncoderCalibration` manages all per-encoder state (saved configs, residual history, iteration log, convergence flag). `EncoderCalibrator` creates one instance per enrolled encoder and orchestrates them collectively.
- **DLL sync**: `set_current_analog_track_adjustments()` is called before every `analyze_raw_data()` to keep the mu_3sl DLL in sync with chip state.
- **Convergence**: Configurable `max_iterations` (default=10). Stops early when all 8 residuals ≤ 1.0 LSB. Non-converged encoders get `CalibrationResult(success=False)`; converged ones proceed to EEPROM save.
- **Motor lifecycle**: The motor runs continuously for the entire calibration session via the `motor_spinning()` context manager. It starts once before the iteration loop and stops after finalization.
- **TPDO data acquisition**: Raw encoder data is captured via EtherCAT TPDOs (TPDO map on the encoder `pos_value` registers), not BiSS SDO reads. Sampling runs in the same PDO exchange thread as the FSoE safety protocol, giving deterministic capture at the PDO cycle rate.
- **Two-phase FSoE lifecycle**: `prepare_fsoe()` registers safety PDO maps on the servo, then the caller registers the data TPDO, then `activate_pdos()` starts the PDO thread. This ordering is critical because the TPDO dictionary insertion order must match the sorted index order expected by the process data parser. Uses STO bypass mode with `use_sra=True`. PDO watchdog raised to 1.0s. Current ramped in discrete steps with sleeps to avoid PDO starvation.
- **Factory-default analog reset**: `reset_analog_to_factory_defaults()` is called during setup (phase 3) to provide a sensible starting point when the current chip state is unknown or corrupted.
- **Nonius SPO finalization**: `finalize()` uses the stored `last_analyze_result` from the converging iteration to compute the optimized nonius offset table. No extra motor spin or data capture is needed.
- **EEPROM save sequence**: `finalize()` writes SPO params → restores iC-MU config (`set_ic_config()`) → enables all errors (`CFGEW=0x00`) → saves to EEPROM (`WRITE_ALL` command) → issues `ABS_RESET` to clear the startup NON_CTR error.
- **Guaranteed restore**: `restore_state()` runs in the outer `finally` block for all encoders. Each encoder restores its iC-MU config, enables all errors (`CFGEW=0x00`), and restores its drive config. Each restore is individually wrapped so one encoder's failure doesn't block another.
- **Multi-encoder**: `DriveEncoderRegisters` maps both encoder 1 and 2 register names. Data is captured simultaneously from all encoders via the shared TPDO map.
- **Save/restore pattern**: Caller-managed. `Encoder` exposes `get_/set_` methods returning frozen dataclasses.
- **Motor method**: Internal generator (current mode) with saw-tooth commutation. Configurable via `--gen-current` and `--gen-frequency`.
- **Crash recovery**: `ensure_normal_mode()` detects RAW mode left by interrupted calibrations and restores the encoder to ABS mode before re-entering calibration.
- **ERR/WRN suppression**: `CFGEW=0xFF` disables all iC-MU error sources from asserting the BiSS nE/nW bits during calibration, preventing drive faults on uncalibrated encoders. Restored to `CFGEW=0x00` (all errors enabled) both in `finalize()` (for converged encoders) and `restore_state()` (for all encoders).
- **Diagnostic plots and JSON export**: Configurable via CLI flags (`--save-raw-plots`, `--save-residual-bar-plots`, `--save-trend-plot`, `--save-json`). JSON export runs in the inner `finally` block so data is saved even if finalization fails.
- **Logging**: `logging` module throughout. `--verbose` enables DEBUG output.
