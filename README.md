# ECVT Code for Mr Ray



## Repository structure

```
mr_ray_ecvt/
├─ platformio.ini           # PlatformIO project configuration (board, framework, build flags)
├─ README.md                # Project overview and documentation
├─ include/                 # Public headers for the main application
│  ├─ controller.h          # High-level control logic interface
│  ├─ motor.h               # Motor control interface
│  ├─ pins.h                # Hardware pin mappings
│  └─ pulse_counter.h       # Hall sensor pulse counter interface
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
   └─ pulse_counter.cpp     # Hall sensor pulse counter implementation
```