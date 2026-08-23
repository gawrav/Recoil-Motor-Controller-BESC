# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is STM32-based motor controller firmware for the B-G431B-ESC1 evaluation board, implementing Field-Oriented Control (FOC) for brushless motors. The firmware supports multiple control modes (position, velocity, torque, current) and communicates via CAN bus.

**Hardware**: STM32G431CBUx microcontroller on B-G431B-ESC1 board (https://www.st.com/en/evaluation-tools/b-g431b-esc1.html)

## Build System

This project uses STM32CubeIDE with GNU ARM toolchain. All build files are auto-generated.

### Build Commands

Build the project (from `Recoil-Motor-Controller-B-G431B-ESC1/` directory):
```bash
cd Recoil-Motor-Controller-B-G431B-ESC1/Debug
make clean
make -j$(nproc)
```

The build outputs:
- `Recoil-Motor-Controller-B-G431B-ESC1.elf` - Main firmware binary
- `Recoil-Motor-Controller-B-G431B-ESC1.map` - Memory map

### Build Configurations

- **Debug**: Located in `Debug/`, optimized for debugging (-Og, -g3)
- **Release**: Located in `Release/`, optimized for size/performance

## Project Structure

```
Recoil-Motor-Controller-B-G431B-ESC1/
├── Core/
│   ├── Inc/              # Application headers
│   ├── Src/              # Application source code
│   └── Startup/          # Startup assembly code
├── Drivers/
│   ├── STM32G4xx_HAL_Driver/  # STM32 HAL library
│   └── CMSIS/                  # ARM CMSIS library
├── Debug/                # Debug build directory with makefiles
├── Release/              # Release build directory
├── *.ioc                 # STM32CubeMX project file
└── STM32G431CBUX_FLASH.ld     # Linker script
```

## Architecture

### Control Loop Hierarchy

The firmware implements a cascaded control architecture:

1. **Current Control Loop** (10 kHz via TIM1)
   - Inner-most loop executing in `MotorController_update()`
   - Implements FOC with Clarke/Park transforms in dq reference frame
   - Controls phase currents via PWM through 3-phase inverter
   - Located in: `current_controller.c/h`

2. **Position/Velocity Control Loop** (2 kHz)
   - Outer control loop for position/velocity modes
   - Generates torque/current commands for inner loop
   - Located in: `position_controller.c/h`

### Key Components

**MotorController** (`motor_controller.c/h`):
- Top-level controller object aggregating all subsystems
- Handles mode switching, configuration, Flash persistence
- Entry point for CAN commands and periodic updates

**PowerStage** (`powerstage.c/h`):
- Manages 3-phase PWM generation via TIM1
- ADC sampling for current/voltage sensing using OPAMP amplifiers
- Hardware safety features (over-voltage, over-current protection)

**Encoder** (`encoder.c/h`):
- I2C-based absolute encoder interface
- Flux offset calibration table (128 entries) for motor commutation
- Position/velocity estimation with filtering

**CAN Interface** (`can.c/h`):
- FDCAN protocol implementation following CANopen-like structure
- Frame functions: NMT, PDO (Process Data Objects), SDO (Service Data Objects)
- Parameter read/write via memory-mapped access (see `Parameter` enum)

**FOC Math** (`foc_math.c/h`):
- Clarke and Park transformations
- Utility math functions (angle wrapping, clamping, fast min/max)
- Fixed-point and float conversions

**Motor Profiles** (`motor_profiles.h`):
- Motor-specific parameters (pole pairs, resistance, inductance, Kt)
- Selected via compile-time defines (e.g., `MOTORPROFILE_MAD_M6C12_150KV`)

### Configuration System

All configuration is centralized in `motor_controller_conf.h`:

- **Build-time Settings**:
  - Motor profile selection (MOTORPROFILE_*)
  - Control frequencies (COMMUTATION_FREQ, POSITION_UPDATE_FREQ)
  - Safety features (SAFETY_WATCHDOG_ENABLED)
  - Flash persistence flags (LOAD_CONFIG_FROM_FLASH, etc.)

- **Runtime Parameters**:
  - PID gains for position/velocity/current control
  - Limits (torque, velocity, current)
  - Encoder calibration
  - All accessible via CAN using `Parameter` enum addresses

### Operating Modes

Defined in `Mode` enum (motor_controller_conf.h:115):

- **Safe modes**: DISABLED, IDLE, DAMPING
- **Closed-loop**: CURRENT, TORQUE, VELOCITY, POSITION
- **Open-loop**: VABC_OVERRIDE, VALPHABETA_OVERRIDE, VQD_OVERRIDE
- **Special**: CALIBRATION (encoder alignment), DEBUG

Mode transitions handled by `MotorController_setMode()`.

### Peripheral Usage

- **TIM1**: PWM generation for 3-phase inverter (center-aligned, 10 kHz)
- **TIM2**: Watchdog timeout timer
- **TIM6**: Position controller update timer (2 kHz)
- **TIM8**: Fast CAN telemetry frame transmission
- **ADC1/ADC2**: Current sensing (phase A, B, C) and bus voltage
- **OPAMP1/2/3**: Current sense amplifiers (16x gain, 3mΩ shunt)
- **FDCAN1**: CAN bus communication
- **I2C1**: Absolute encoder interface
- **USART2**: Debug/logging (optional)

### Flash Memory

Configuration stored at address `0x0801F800` (Bank 1, Page 63):
- Device CAN ID
- Motor parameters
- Encoder flux offset calibration table
- Controller gains and limits

Save/load via `MotorController_saveConfig()` / `MotorController_loadConfig()`.

## Change Workflow (REQUIRED for all changes)

This is motor-controller firmware and the host tools that energize it. A bug here can destroy
hardware or injure someone. **Every** change follows this sequence — no exceptions for "small"
or "obvious" ones:

1. **Make the change.**
2. **Verify it.** Build the firmware, or exercise the Python against the stub CAN harness. Never
   report something as working on inspection alone.
3. **Independent review by a subagent.** Launch a fresh subagent to review the diff
   adversarially. Give it: the diff scope, what the change claims to do, and — critically — the
   specific firmware files that are *ground truth* for the claims, so it verifies against source
   rather than plausibility. Ask for confirmed-vs-speculative findings with severity, ranked.
4. **Act on the findings.** Verify each one against the firmware yourself before acting — reviews
   do produce false positives. Fix what is real; say plainly what was rejected and why.
5. **Repeat 3–4 until a round comes back clean.** One round is not enough — see below.
6. **Report, then wait for explicit user confirmation before committing.** Stop and hand the user
   a summary. Do not commit until they say to.

### What the summary must contain

Not a narrative — two explicit lists, so the user can audit the judgement calls without reading
the diff:

- **Fixed**: every finding acted on. Severity, what was wrong, and what the fix was. Say which
  round found it, and flag any finding that was a defect in an *earlier fix* rather than in the
  original work — that pattern is the reason the loop exists.
- **Dropped**: every finding NOT acted on, each with the reason. "Rejected — verified against
  `<file>`, the claim is wrong because X", or "accepted as a known limitation because Y". Silently
  omitting a finding is not allowed: the user cannot audit what they cannot see, and a finding
  dropped without a reason is indistinguishable from one that was missed.

Also state how many rounds ran and which round came back clean. If anything was verified
empirically (a measurement, a reproduction, a stub test), give the number — "max heartbeat gap
1.6 s → 0.22 s" carries the argument in a way "fixed the starvation" does not.

### Review until clean — not once

Every round reviews the code *as it now stands*, including the fixes the previous round prompted.
This is not belt-and-braces; fixes have repeatedly introduced worse bugs than the ones they
addressed. Real examples from this repo:

- Round 1 flagged a thread-safety concern that was waved off as having no concrete path. It was
  the concrete path: dropped CAN frames silently stalled the arm mid-jog.
- The fix for that (`can.ThreadSafeBus`) was a **no-op** — it holds separate send and recv locks.
- Its replacement (one lock over both) introduced a **hang-forever** in libusb, unkillable by
  Ctrl-C with the motor energized, plus the very heartbeat starvation it was written to prevent.
- A readback check added to catch lost commands would have told the operator a **moving** arm was
  dead, on ~30% of jogs, because of a float32 double-rounding mismatch with the MCU.

None of those were visible by inspection. Each was caught only because another round ran.

**Clean means:** the round produced no confirmed finding of medium severity or above that is
neither fixed nor consciously accepted with a stated reason. Low-severity nits you decide not to
act on do not block; say so explicitly rather than silently dropping them.

**Each round's prompt must say which round it is and what the previous rounds found**, so the
reviewer targets what changed instead of re-treading settled ground. Explicitly tell it that a
clean result is a valid outcome and it must not invent findings to appear thorough — otherwise
the loop never terminates.

If two consecutive rounds produce only findings you are rejecting, stop and put the disagreement
to the user rather than iterating further.

### What a good review prompt names as ground truth

- `Core/Inc/motor_controller_conf.h` — the `Parameter` enum is authoritative for every PARAM byte
  offset. The SDO handler does raw pointer arithmetic into `MotorController`, so a stale offset
  writes to the wrong field. Also `Mode`, `ErrorCode`, `FrameFunction`.
- `Core/Src/motor_controller.c` — `handleCANMessage` / `handleSDO` / `handleNMT`, the mode
  dispatch in `MotorController_update`, and the `_Static_assert` block pinning offsets.
- `Core/Src/position_controller.c` + `.h` — which limits clamp in which mode, and the
  host-vs-raw position frame (`position_offset`).
- `Core/Src/app.c` and `main.c` — the TIM2 safety watchdog and which frames reset it.

## Development Workflow

### Modifying Hardware Configuration

The `.ioc` file is the STM32CubeMX project. Opening it in STM32CubeMX or STM32CubeIDE allows graphical peripheral configuration. Code in `/* USER CODE BEGIN */` / `/* USER CODE END */` blocks is preserved during regeneration.

**Files auto-generated by CubeMX** (do not manually edit outside USER CODE sections):
- `Core/Src/main.c` - Peripheral initialization
- `Core/Src/stm32g4xx_hal_msp.c` - MSP initialization
- `Core/Src/stm32g4xx_it.c` - Interrupt handlers

### Adding User Application Code

User application logic goes in `app.c/h`:
- `APP_init()`: Called once during initialization
- `APP_main()`: Main application loop
- Callbacks like `HAL_TIM_PeriodElapsedCallback()` for interrupt handling

Example control loop customization in app.c:35:
```c
// Set position target from potentiometer:
controller.position_controller.position_target = APP_getUserPot() * M_PI;
```

### Motor Profile Configuration

To add a new motor:

1. Define profile in `motor_profiles.h` with measured parameters
2. Enable it in `motor_controller_conf.h:71` (uncomment appropriate #define)
3. Adjust NOMINAL_BUS_VOLTAGE for your power supply
4. Run calibration mode to generate encoder flux offset table

### CAN Communication

Frame ID format: `[Function:4][Device_ID:6]` (11-bit standard CAN)

Common commands:
- Set mode: Write to PARAM_MODE
- Read telemetry: Subscribe to PDO frames
- Save config: Use FUNC_FLASH

See `FrameFunction` and `Parameter` enums for full protocol definition.

## Important Notes

- **Coordinate frames**: Motor control uses rotating dq frame (direct/quadrature), stationary αβ frame (Clarke), and 3-phase abc
- **Control timing critical**: TIM1 ISR must complete in <100μs to avoid jitter
- **Encoder calibration required**: Run MODE_CALIBRATION once per motor before closed-loop operation
- **CAN ID range**: Valid device IDs are 1-63
- **Firmware version format**: `0xYYYYMMDD` (currently 0x20250226)
