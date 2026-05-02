# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a fork of ArduPilot focused on implementing **Field-Oriented Control (FOC)** for brushless DC motors on STM32 microcontrollers. The active development target is the `TEST-G4-FOC-ESC` board (STM32G431), integrated via the `AP_Periph` peripheral firmware framework. Active branch: `feat/ap_foc_main`.

## Build System

The build system is **Waf** (Python-based), invoked via `./waf`. A `Makefile` wrapper exists for convenience.

```bash
# Initial setup (required once)
git submodule update --init --recursive

# Configure for the FOC ESC test board
./waf configure --board TEST-G4-FOC-ESC

# Build AP_Periph firmware (main FOC target)
./waf AP_Periph

# Build bootloader
./waf --target tools/AP_Bootloader

# List all available boards
./waf list_boards

# Other vehicle builds (SITL, copter, plane, rover, etc.)
./waf configure --board sitl && ./waf copter
```

Build artifacts appear in `build/TEST-G4-FOC-ESC/` (or the configured board name).

## Running Tests

```bash
# Unit tests
make check          # Run only newly modified tests
make check-all      # Run all tests

# SITL-based integration tests
./Tools/autotest/autotest.py --vehicle=ArduCopter
```

## Code Style

- **Python:** Black (line length 120), isort (profile: black), Flake8 (max line length 127)
- **C++:** Clang-tidy (configured in pre-commit hooks for specific paths)
- **Pre-commit hooks:** `pre-commit run --all-files`

## Architecture

### HAL Abstraction

All hardware interaction goes through `AP_HAL`. The active implementation for embedded targets is `AP_HAL_ChibiOS` (ChibiOS RTOS on STM32). Key locations:

- `AP_HAL/` — Cross-platform interface definitions
- `AP_HAL_ChibiOS/` — STM32/ChibiOS implementation (contains the FOC motor control code)
- `AP_HAL_Linux/` — Linux/POSIX implementation
- `AP_HAL_SITL/` — Software-in-the-loop simulator

### FOC Motor Control (this branch)

The FOC implementation lives in `AP_HAL_ChibiOS/`:

| File | Purpose |
|------|---------|
| `MotorControl.h` / `.cpp` | C++ class: open-loop sine-wave control, current sensing, RMS calculation, ISR-based ADC sampling |
| `stm32_foc_motor_control.h` / `.cpp` | Low-level C driver: PWM (TIM1), ADC1/ADC2 with DMA, three OpAmps at 16× PGA gain, TIM2 sample trigger |

**Signal chain:** TIM1 generates three-phase complementary PWM → OpAmps sense phase currents → ADC1/ADC2 sample via DMA triggered by TIM2 → ISR accumulates samples for RMS calculation.

### TEST-G4-FOC-ESC Board

Hardware definition: `libraries/AP_HAL_ChibiOS/hwdef/TEST-G4-FOC-ESC/`

- `hwdef.dat` — Main board definition (STM32G431, 128 KB flash, 20 MHz PWM clock, 20 kHz PWM frequency)
- `hwdef-bl.dat` — Bootloader definition (26 KB reserved)

### Integration Point: AP_Periph

The FOC motor control is initialized in `Tools/AP_Periph/AP_Periph.cpp` (`AP_Periph::init()`), which configures PWM frequency, deadtime, and current sensing. `AP_Periph` is the DroneCAN peripheral firmware that hosts the motor control.

### Library Organization

153 shared libraries in `libraries/`, prefixed by subsystem:
- `AP_*` — Core autopilot libraries (HAL, motors, AHRS, GPS, battery, logging, etc.)
- `AC_*` — Copter-specific control libraries (attitude control, waypoint nav)

Vehicle-specific code lives in `ArduCopter/`, `ArduPlane/`, `Rover/`, `ArduSub/`, etc., each with a top-level class (`Copter`, `Plane`, `Rover`) that composes the shared libraries.
