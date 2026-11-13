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
```

If the serial monitor stays empty and you're using the board's native USB port, uncomment
`-D ARDUINO_USB_CDC_ON_BOOT=1` in `platformio.ini`.

Shared settings (pins, PCA9685 address, 50 Hz, safe PWM range, channel names, measured leg
dimensions) live in `include/robot_config.h`.

## Build order

| Step | Goal | Status | Leika reference |
|---|---|---|---|
| 1 | **I2C check**: the ESP32-S3 sees the PCA9685 at 0x40 | ✅ `i2c_scan` | `esp32/include/peripherals/i2c_bus.h` |
| 2 | **Servos by hand**: move any servo from the serial monitor at 50 Hz | ✅ `servo_tool` | `peripherals/drivers/pca9685.h` |
| 3 | **Calibration table**: centre, direction, ticks/degree per servo, saved in flash (Preferences) | | `peripherals/servo_controller.h` |
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
| `p <pwm>` | set the pulse (125–600 ticks; 306 ≈ centre) |
| `+` `-` / `++` `--` | nudge by 1 / 10 |
| `s` | slow sweep between the limits (any command stops it) |
| `o` | outputs off (servos go limp) |
| `?` | help and current state |

Servos stay off after power-up until the first command. Remove the horns (or hold the robot up)
before sweeping: the full range can drive a leg into the body.
