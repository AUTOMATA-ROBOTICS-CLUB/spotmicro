# SpotMicro — Automata Robotics Club

A SpotMicro quadruped robot: 3D-printed SpotMicroESP32 body (Michael Kubina) with
robjk's reinforced shoulders, 12 × MG996R servos, ESP32-S3 + PCA9685, built
step by step with [SpotMicroESP32-Leika](https://github.com/runeharlyk/SpotMicroESP32-Leika) as the
reference design.

| Folder | What's in it |
|---|---|
| [`hardware/`](hardware/) | Print files, parts list, wiring, power, measured dimensions, servo calibration |
| [`firmware/`](firmware/) | Arduino (PlatformIO) firmware for the ESP32-S3, one build step at a time |
| [`simulation/`](simulation/) | MuJoCo simulation of this exact robot: printed parts, measured dimensions, MG996R servos; teleop, stairs, gait tuning and RL training |

## Getting started

1. **Build the hardware**: print and wire it following [`hardware/README.md`](hardware/README.md).
2. **Bring it up**: flash the firmware steps in order ([`firmware/README.md`](firmware/README.md)),
   starting with `i2c_scan` and `servo_tool`.
3. **Try things in simulation first**:

   ```bash
   cd simulation
   uv sync
   uv run python teleop.py      # drive it: arrow keys in the viewer, `help` in the terminal
   ```

## Credits and licences

- Code: MIT (see [`LICENSE`](LICENSE)). The simulation and the kinematics/gait it ports come from
  SpotMicroESP32-Leika by Rune Harlyk (MIT).
- Print files in `hardware/print/`: KDY0523 (SpotMicro, Thingiverse 3445283) and robjk (reinforced
  shoulders, Thingiverse 4937631), both CC BY.
