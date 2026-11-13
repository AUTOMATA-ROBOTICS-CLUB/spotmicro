# Hardware

SpotMicro quadruped printed from **KDY0523's SpotMicro** (MG996R variants), with **12 × MG996R**
servos, an **ESP32-S3** board and a **PCA9685** servo driver.

## Printed parts (`print/`)

Print the `_mg` versions (sized for MG996R) and the `non-mega_` covers (any board other than an
Arduino Mega). Left (`L_`) and right (`R_`) parts are mirror images.

| Part | Qty | | Part | Qty |
|---|---|---|---|---|
| `I_shoulder_mg` (front/rear frame) | 2 | | `L_arm_joint_mg` / `R_arm_joint_mg` (shoulder bracket, holds hip servo) | 2 / 2 |
| `O_shoulder` (outer frame plate) | 2 | | `L_arm_mg` / `R_arm_mg` (upper leg, holds femur servo) | 2 / 2 |
| `non-mega_L_side_plate` / `R_side_plate` | 1 / 1 | | `L_arm_cover` / `R_arm_cover` (upper-leg shell) | 2 / 2 |
| `plate` (electronics plate, drilled for a Mega) | 1 | | `L_wrist_mg` / `R_wrist_mg` (lower leg, holds knee servo) | 2 / 2 |
| `non-mega_T_cover_mg` / `non-mega_B_cover_mg` | 1 / 1 | | `foot` | 4 |
| `F_cover` (head) / `R_cover` (rear) | 1 / 1 | | `L_ultra_sonic` / `R_ultra_sonic` (HC-SR04 mounts) | 1 / 1 |

Optional **reinforced-shoulder remix** by robjk: `Shoulder_*`, `Limb_Top_Shell_with_BB_hole`,
`Rear_Cover_*_Medium` (shorter rear cover with OLED + GX16 cut-outs). See `print/robjk_README.txt`.

Printed parts weigh ~510 g at 20% infill (~480–600 g for 15–30%).

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

Screws and bearings for the KDY design (from its README): 8 × M5×15, 40 × M4×20, 8 × M4×15,
48 × M4 nut, 4 × M3×20, 28 × M3×10, 16 × M3 nut, 8 × F625zz flange bearing. The remix adds M3×20
screws/nuts and 625ZZ bearings.

## Wiring

**ESP32-S3 → PCA9685** (I2C):

| PCA9685 | ESP32-S3 |
|---|---|
| VCC | **3V3** (not 5 V: keeps SDA/SCL at 3.3 V) |
| GND | GND |
| SDA | **GPIO47** |
| SCL | **GPIO21** |
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

Where the servos sit: the **hip** servo rides in the shoulder bracket (`arm_joint`), the **femur**
servo in the top of the upper leg (`arm`), the **knee** servo in the lower leg (`wrist`).

## Measured dimensions

Measured on the printed parts (pivot holes, bearing seats and servo mounting ears found in the
STLs; across-part distances from fitting the parts into an assembled model). These are the values
the firmware and simulation use:

| | mm | SpotMicroESP32-Leika firmware |
|---|---|---|
| Body length (front ↔ rear femur axes) | 185.5 | 207.5 |
| Body width (left ↔ right hip axes) | 78.0 | 78.0 |
| Coxa (hip axis → leg plane, sideways) | 55.0 | 60.5 |
| Coxa offset (hip axis → femur axis, down) | 10.75 | 10.0 |
| Femur (femur shaft → knee) | 108.3 | 111.2 |
| Tibia (knee → ground, leg straight) | 135.4 | 118.5 |

Estimated total mass ~1.78 kg (body 689 g, each shoulder 76 g, upper leg 110 g, lower leg 86 g).

## Servo calibration

Use the `servo_tool` firmware step (see `../firmware/README.md`). Always calibrate at **50 Hz**: the
same PWM value gives a ~20% different pulse at 60 Hz, so numbers found at another frequency don't
carry over.

1. Horns off. `a` (all servos), `p 306`: every servo goes to its nominal centre.
2. Attach the horns and legs so every leg hangs straight down, shoulders level (the 25-tooth spline
   allows ~14° steps; get close).
3. Per servo: `c <n>`, then `+` / `-` until the joint is exactly at its zero position. Write down the
   value — that is the servo's centre.
4. Find each servo's direction (which way a higher PWM turns the joint) and its ticks per degree
   (move 90° with a square: `(pwm_90 − pwm_0) / 90`, typically ~2.2 for MG996R).

## Credits

- SpotMicro design: **KDY0523** — https://www.thingiverse.com/thing:3445283 (CC BY)
- Reinforced shoulders / medium rear cover: **robjk** — https://www.thingiverse.com/thing:4937631 (CC BY)
