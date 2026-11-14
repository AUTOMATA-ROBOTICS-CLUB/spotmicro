# Firmware

Arduino firmware for the ESP32-S3, built up one step at a time. Each step is a separate PlatformIO
program, so it can be flashed and checked on the real robot before moving on.
[SpotMicroESP32-Leika](https://github.com/runeharlyk/SpotMicroESP32-Leika) is the reference
implementation for each step.

## Setup

Install [PlatformIO](https://platformio.org/) (VS Code extension, or `pipx install platformio` /
`uv tool install platformio`). Wiring and pins: `../hardware/README.md`.

```bash
pio run -e i2c_scan -t upload -t monitor      # step 1
pio run -e servo_tool -t upload -t monitor    # step 2
pio run -e calibrate -t upload -t monitor     # step 3
```

If the serial monitor stays empty and you're using the board's native USB port, uncomment
`-D ARDUINO_USB_CDC_ON_BOOT=1` in `platformio.ini`.

Shared settings (pins, PCA9685 address, 50 Hz, safe PWM range, channel names, measured leg
dimensions) live in `include/robot_config.h`.

## Code style

C++ follows the [LLVM coding style](https://llvm.org/docs/CodingStandards.html), set in
`../.clang-format`. Format before committing:

```bash
clang-format -i src/*.cpp include/*.h     # or: uvx clang-format -i ...
```

## Build order

| Step | Goal | Status | Leika reference |
|---|---|---|---|
| 1 | **I2C check**: the ESP32-S3 sees the PCA9685 at 0x40 | ✅ `i2c_scan` | `esp32/include/peripherals/i2c_bus.h` |
| 2 | **Servos by hand**: move any servo from the serial monitor at 50 Hz | ✅ `servo_tool` | `peripherals/drivers/pca9685.h` |
| 3 | **Calibration table**: channel, centre, direction, ticks/degree per servo, saved in flash (Preferences) | ✅ `calibrate` | `peripherals/servo_controller.h` |
| 4 | **One leg**: inverse kinematics puts a foot where you ask | | `include/kinematics.h` (`legIK`) |
| 5 | **Stand**: all four legs hold a level pose; rest/stand transitions | | `motion_states/stand_state.h`, `rest_state.h` |
| 6 | **Control link**: WiFi + web page / WebSocket for commands | | `src/main.cpp`, `app/` |
| 7 | **Walk**: trot gait (stance + Bezier swing) | | `motion_states/walk_state.h` |
| 8 | **Balance**: IMU levelling | | `peripherals/imu.h` |

Test the math of steps 4, 5 and 7 in `../simulation` first: it models this robot with the same
dimensions, servo strength and mass.

## servo_tool commands

| Command | Action |
|---|---|
| `c <0-15>` / `a` | select one channel (12–15 are spares) / all 12 robot servos |
| `p <pwm>` | set the pulse (100–600 ticks; 306 ≈ centre) |
| `+` `-` / `++` `--` | nudge by 1 / 10 |
| `s` | slow sweep between the limits (any command stops it) |
| `o` | outputs off (servos go limp) |
| `?` | help and current state |

Servos stay off after power-up until the first command. Remove the horns (or hold the robot up)
before sweeping: the full range can drive a leg into the body.

## calibrate commands

Every joint's centre pulse puts it in the **reference pose**, which is the standing pose: hips level
with the leg straight down, upper legs 58° back, knees bent 107° (the leg IK at the default height). Angles use the IK convention: hip 0 = straight down, femur 0 =
vertical (negative = back), knee 0 = straight.

| Command | Action |
|---|---|
| `j <0-11>` / `a` | select one joint / all joints |
| `h` | selection to the standing pose |
| `z` | selection to rest (body lowered to its minimum height) |
| `g <deg>` | selection to a joint angle (`h`, `z`, `g` move at 60°/s) |
| `+` `-` / `++` `--` | nudge the selected joint's centre by 1 / 10 ticks (the joint moves with it) |
| `k <deg>` | mark "the joint is really at `<deg>` now"; two marks ≥ 30° apart set direction, ticks/degree and centre |
| `d` | flip the selected joint's direction |
| `t <x>` | ticks per degree for the selection (MG996R ~2.2) |
| `m <0-15>` | move the selected joint to another PCA9685 channel |
| `l` | list the table, with the angle range each joint can reach inside 100–600 |
| `w` / `r` / `x` | write to flash / reload from flash / reset to `DEFAULT_CAL` in `robot_config.h` |
| `o` | outputs off |

The table is loaded from flash at boot (`include/servo_cal.h`), so later steps use the saved values.
