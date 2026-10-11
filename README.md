# ECVT Code for Dingo

## Overview

This repository contains the embedded control firmware for the Dingo ECVT system. It runs on an ESP32 and manages a DRV8462-driven stepper motor to position the CVT sheave based on engine RPM and operator-selected modes. Telemetry is sent over a TX-only serial link to an ESP32-CAM, which logs it as CSV to its SD card.

## High-level architecture

- **Control loop**: A FreeRTOS timer drives the main controller tick that selects a mode and computes a sheave position setpoint.
- **Motor subsystem**: A dedicated motor timer applies acceleration/velocity limiting and commands the DRV8462 driver via RMT step pulses.
- **Sensing**: Hall-effect pulse counting provides engine RPM, and a quadrature encoder provides motor position feedback.
- **Telemetry**: CSV rows with time, engine RPM, motor setpoint, motor position, brake state, and control mode are sent over UART2 TX (GPIO32) to an ESP32-CAM (RX on GPIO13), which writes them to a new `log_NNNN.csv` file on its SD card each boot.

## Detailed breakdown

### Application core

- `src/main.cpp`: Arduino entry points, initializes the controller, and prints telemetry snapshots for debugging.
- `include/controller.h` / `src/controller.cpp`: Main control logic, mode selection, homing sequence, RPM-to-setpoint logic, and SD card logging.

### Motor control

- `include/motor.h` / `src/motor.cpp`: Motion planner with acceleration and velocity limiting, encoder feedback, and driver commands.
- `include/DRV8462.h` / `src/DRV8462.cpp`: DRV8462 driver SPI interface, auto-torque setup, RMT step pulse generation, and fault handling.
- `include/DRV8462_REGMAP.h`: Register addresses and bit masks used by the driver.

### Sensing and filtering

- `include/encoder.h` / `src/encoder.cpp`: Quadrature encoder reader using ESP32 PCNT hardware.
- `include/pulse_counter.h` / `src/pulse_counter.cpp`: Hall sensor pulse counter with RPM calculation and low-pass filtering.
- `include/filter.h`: Small low-pass filter utility used for RPM smoothing.

### SD card logger (ESP32-CAM)

- `sd_logger/`: Separate PlatformIO project for the ESP32-CAM. Receives telemetry on GPIO13 at 115200 baud and writes it to the SD card (1-bit mode), flushing every second. Build/upload with `pio run -d sd_logger -t upload`.

### Configuration and integration

- `include/config.h`: Pin mappings, timer rates, motor and controller constants, and debug flags.

## Repository structure

```
dingo_ecvt/
├─ platformio.ini           # PlatformIO project configuration (board, framework, build flags)
├─ README.md                # Project overview and documentation
├─ include/                 # Public headers for the main application
│  ├─ controller.h          # High-level control logic interface
│  ├─ motor.h               # Motor control interface
│  ├─ config.h              # Hardware pin mappings and constants
│  ├─ pulse_counter.h       # Hall sensor pulse counter interface
│  ├─ encoder.h             # Quadrature encoder interface
│  ├─ filter.h              # Simple low-pass filter utility
│  ├─ DRV8462_REGMAP.h      # Register map for DRV8462 motor driver
│  └─ DRV8462.h             # Motor driver interface
├─ sd_logger/               # ESP32-CAM serial-to-SD logger firmware
│  ├─ platformio.ini
│  └─ src/main.cpp
└─ src/                     # Main application sources
   ├─ controller.cpp        # Control logic implementation
   ├─ main.cpp              # Application entry point
   ├─ motor.cpp             # Motor control implementation
   ├─ pulse_counter.cpp     # Hall sensor pulse counter implementation
   ├─ encoder.cpp           # Encoder implementation
   └─ DRV8462.cpp           # Motor driver implementation
```
