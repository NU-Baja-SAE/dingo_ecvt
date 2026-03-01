# ECVT Code for Mr Ray

## Overview

This project implements a closed-loop control system for managing the CVT sheave position on a Baja SAE off-road vehicle. The system uses PID control to optimize engine RPM and vehicle performance by automatically adjusting the gear ratio based on sensor inputs.

## Features

- **PID Position Control**: Closed-loop control of sheave position with tunable PID gains
- **Engine RPM Monitoring**: Hall effect sensor input for primary and secondary RPM measurement
- **Wheel Speed Sensing**: Real-time vehicle speed calculation via wheel hall sensor
- **Brake Slam Detection**: Automatic downshift to low gear when brakes are applied suddenly
- **CAN Telemetry**: Real-time data logging via CAN Bus for debugging and analysis
- **FreeRTOS Tasks**: Multi-threaded operation for responsive control

## Hardware

- **Microcontroller**: ESP32 DOIT DevKit V1
- **Motor Driver**: DRV8462 Stepper motor driver
- **Sensors**:
  - Potentiometer for sheave position feedback
  - Rotary encoder for precise position tracking
  - Hall effect sensors for engine primary/secondary RPM
  - Hall effect sensor for wheel speed
  - Analog brake pedal sensor

## Repository structure

```
mr_ray_ecvt/
├─ platformio.ini           # PlatformIO project configuration (board, framework, build flags)
├─ README.md                # Project overview and documentation
├─ include/                 # Public headers for the main application
│  ├─ controller.h          # High-level control logic interface
│  ├─ motor.h               # Motor control interface
│  ├─ config.h              # Hardware pin mappings and constants
│  ├─ pulse_counter.h       # Hall sensor pulse counter interface
│  ├─ DRV8462_REGMAP.h      # Register map for DRV8462 motor driver
│  └─ DRV8462.h             # Motor driver interface
├─ lib/                     # Local libraries and submodules
│  └─ baja_can/             # CAN driver library (submodule)
│     ├─ platformio.ini     # Library-specific PlatformIO config
│     ├─ README.md          # Library documentation
│     ├─ include/           # Library headers (e.g., BajaCan.h)
│     └─ src/               # Library implementation (e.g., BajaCan.cpp)
└─ src/                     # Main application sources
   ├─ controller.cpp        # Control logic implementation
   ├─ main.cpp              # Application entry point
   ├─ motor.cpp             # Motor control implementation
   ├─ pulse_counter.cpp     # Hall sensor pulse counter implementation
   └─ DRV8462.cpp           # Motor drive implementation
```