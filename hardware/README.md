# Hardware

SpotMicro quadruped printed from **Michael Kubina's SpotMicroESP32** with **robjk's reinforced
shoulders** and medium rear cover, with **12 × MG996R** servos, an **ESP32-S3** board and a
**PCA9685** servo driver.

## Printed parts (`print/`)

The folder also holds KDY0523's original SpotMicro, which Kubina's design is derived from but not
compatible with; those parts are not used. Counts are from `print/Kubina_parts.md` ("+ m" =
printed mirrored).

| Part | Qty | | Part | Qty |
|---|---|---|---|---|
| **Chassis** (Kubina) | | | **Legs** | |
| `Inner_Shoulder`, `Outer_Shoulder` | 2, 2 | | `Limb_Top_Shell_with_BB_hole` (robjk) | 2 + 2 m |
| `Top_Shoulder`, `Bottom_Shoulder` | 2, 2 | | `Limb_Bottom_Shell` | 2 + 2 m |
| `Chassis_Side` | 2 | | `Limb_Servohorn_Mount_long_v4` | 2 + 2 m |
| `Circuitry_Mountingplate_SZBK07` | 1 | | `Limb_Ball_Bearing_Mount_long_v4` | 2 + 2 m |
| `LiPo_Mountingbracket_Front` / `_Middle` / `_Rear` | 1 each | | `Wrist`, `Wrist _Counterpart` (lower leg) | 2 + 2 m each |
| **Shoulder joints** (robjk) | | | `Shoulder_Joint_Ball_Bearing_Pin` (knee) | 4 |
| `Shoulder_Joint_Servomount_extended_BB` | 2 + 2 m | | `Foot_Tip` (TPU) | 4 |
| `Shoulder_Joint_Servo_Cap_Extended` | 4 | | **Hull** (100% infill) | |
| `Shoulder_Cover` | 4 | | `Front_Cover_Optimized`, `sensormount_esp32_cam`, `cameramount_esp32_cam` | 1 each |
| `Shoulder_Joint_Ball_Bearing_Pin_M3x8` | 4 | | `Top_Cover_Split_Front` / `_Rear` | 1 / 1 |
| | | | `Bottom_Cover_Split_Front` / `_Rear` | 1 / 1 |
| | | | `Rear_Cover_Shell_Medium`, `Rear_Cover_Plate_Medium` (robjk) | 1 each |

Use the `_long_v4` knee mounts (Kubina's recommendation: the legs fold flat at rest). They are
61.5 mm long; the normal ones (46.5 mm) give a 3 mm shorter upper leg than the simulation uses.
The shoulder cover and extended servo cap are symmetric, so the same print fits both sides.

## Electronics and hardware

| Part | Qty | Notes |
|---|---|---|
| ESP32-S3 WROOM camera board (N8R8) | 1 | |
| PCA9685 16-channel servo driver | 1 | address 0x40 |
| MG996R servo, **180°**, metal gear | 12 (+2 spares) | not the "360°" continuous-rotation version |
| SZBK07 20 A buck converter | 1 | **servo power**, set to ~6.0 V (MG996R max is 7.2 V) |
| LM2596 / XL4015 buck converter | 1 | **5 V logic** only (too weak for 12 servos) |
| 2S LiPo, 2200 mAh, 25C+, XT60 | 1 | ~20–30 min walking; female XT60 on the battery |
| LiPo balance charger, voltage alarm, LiPo bag | 1 each | |
| Main switch, 12 V DC 20 A (SPST toggle) | 1 | |
| Inline fuse ~15–20 A | 1 | |
| 14–16 AWG silicone wire, XT60 pairs, Dupont wires | – | |
| Optional: GY-87 IMU, 2 × HC-SR04, 0.96" SSD1306, PAJ7620U2 | – | |

Screws and bearings (Kubina's bill of materials, plus robjk's shoulders): 84 × M2×8 + nuts (servo
horns, bearing pins), 80 × M3×8 + nuts, 64 × M3×20 + nuts, 12 × 625ZZ ball bearings (8 for Kubina's
legs, 4 for the shoulder covers).

## Wiring

**ESP32-S3 → PCA9685** (I2C):

| PCA9685 | ESP32-S3 |
|---|---|
| VCC | **3V3** (not 5 V: keeps SDA/SCL at 3.3 V) |
| GND | GND |
| SDA | **GPIO21** |
| SCL | **GPIO47** |
| OE, header V+ | not connected |

**Power:**

```
2S LiPo ──► fuse ──► switch ─┬─► SZBK07 (~6.0 V) ──► PCA9685 screw terminal V+ / GND  (servos)
                             └─► LM2596/XL4015 (5.0 V) ──► ESP32 5V / GND              (logic)
```

Set both converters with a multimeter **before** connecting anything. Connect all grounds together.
Add a 470–1000 µF capacitor across the PCA9685 screw terminal.

**Servos** (brown = GND, red = V+, orange = PWM) on PCA9685 channels:

| Channel | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 | 11 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| Leg | FL | FL | FL | FR | FR | FR | RL | RL | RL | RR | RR | RR |
| Joint | hip | femur | knee | hip | femur | knee | hip | femur | knee | hip | femur | knee |

Where the servos sit: each servo rides in the link it moves, with its horn on the link above. The
**hip** servo is in the shoulder joint servo mount (horn in the outer shoulder plate). The **femur**
servo is in the upper leg (horn in the shoulder joint's servo cap). The **knee** servo is in the
lower leg (horn in the knee servo-horn mount).

## Measured dimensions

Measured in Kubina's assembled model (servo gears, shoulder pins, foot-tip arc; averaged over the
four legs), which the print parts fit to within ~0.2–0.8 mm. These are the values the firmware and
simulation use:

| | mm | SpotMicroESP32-Leika firmware |
|---|---|---|
| Body length (front ↔ rear femur axes) | 205.9 | 207.5 |
| Body width (left ↔ right hip axes) | 78.0 | 78.0 |
| Coxa (hip axis → leg plane, sideways) | 60.9 | 60.5 |
| Coxa offset (hip axis → femur axis, down) | 10.0 | 10.0 |
| Femur (femur axis → knee axis) | 111.0 | 111.2 |
| Tibia (knee axis → foot-tip arc centre) | 118.0 | 118.5 |
| Foot tip radius (arc centre → ground) | 19.9 | – |

Estimated total mass ~2.13 kg (body 871 g, each shoulder 93 g, upper leg 125 g, lower leg 98 g).

## Servo calibration

Use the `servo_tool` firmware step (see `../firmware/README.md`). Always calibrate at **50 Hz**: the
same PWM value gives a ~20% different pulse at 60 Hz, so numbers found at another frequency don't
carry over.

Reference pose = **standing pose** (every servo at its centre pulse): hips level with the leg
straight down, upper legs **58° back** from vertical, knees bent **107°** (73° between upper and
lower leg). These were the leg IK angles at the default body height; with the measured Kubina
legs the IK gives 55° / 106° there, so standing at the default height is within ~3° of every
servo's centre. Keep the reference as it is: the calibration table is relative to it.

1. Horns off. `servo_tool`: `a` (all servos), `p 306`: every servo goes to its nominal centre.
2. Attach the horns and legs close to the standing pose (the 25-tooth spline allows ~14° steps).
   Hold the robot up so the legs hang free.
3. Per servo: `c <n>`, then `+` / `-` until the joint is in the standing pose. Write down the
   value: that is the servo's centre. Put the values in `DEFAULT_CAL` in
   `firmware/include/robot_config.h`.
4. Flash `calibrate`. Per joint (`j <n>`):
   - `h`, then check the angle with a protractor; fix it with `+` / `-` (this moves the centre).
   - femur: `g 0` should make the upper leg vertical. knee: `g 90` should make a right angle
     between the leg parts. hip: `g 20` should swing the foot 20° **outward**
     (away from the body), on every leg.
   - Wrong way: `d`. Right way but too far / not far enough: adjust `t`, then `h` again.
5. `l` to check each joint's reachable range, then `w` to save to flash.

Our hand-measured standing pose (2026-11-12):

| Leg | Hip | Femur | Knee |
|---|---|---|---|
| Front left | 306 | 180 | 400 |
| Front right | 470 | 432 | 194 |
| Rear left | 295 | 180 | 406 |
| Rear right | 310 | 490 | 194 |

## Credits

- SpotMicroESP32: **Michael Kubina** — https://github.com/michaelkubina/SpotMicroESP32 (GPL-3.0)
- Reinforced shoulders / medium rear cover: **robjk** — https://www.thingiverse.com/thing:4937631 (CC BY)
- Original SpotMicro: **KDY0523** — https://www.thingiverse.com/thing:3445283 (CC BY)
