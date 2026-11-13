"""Generate the SpotMicro ESP32 MuJoCo scenes (scene.xml, scene_terrain.xml).

Kinematic skeleton: the leg/body dimensions measured on the printed parts (constants in
src.robot.firmware_gait), laid out so that each MuJoCo joint angle equals the firmware legIK angle
(theta1, theta2, theta3) in radians.

Visual meshes and mass model: the printed KDY0523 SpotMicro parts (MG996R variants; CC BY,
https://www.thingiverse.com/thing:3445283), placed by `fit_print_meshes.py` into
`print_parts.json`. Link masses = those parts at 20% infill + the non-printed components below
(servos, battery, electronics, hardware). Collision uses primitives (body box, leg capsules, foot
spheres).

    uv run python -m src.resources.spotmicro_esp32.generate_scene
"""

import json
import os

import mujoco
import numpy as np

from src.robot.firmware_gait import (
    COXA, COXA_OFFSET, FEMUR, TIBIA, LEG_NAMES, LEG_SIDE, HIP_POS, FOOT_RADIUS, STAND_Z,
    MIN_BODY_HEIGHT, MAX_BODY_HEIGHT,
)

HERE = os.path.dirname(os.path.abspath(__file__))
PARTS = json.load(open(os.path.join(HERE, "print_parts.json")))

# Non-printed components (kg). Servo positions come from print_parts.json (servo pockets).
SERVO_MASS = 0.055                       # MG996R
SERVO_SIZE = (0.0407, 0.0197, 0.0429)    # box used for its inertia
BATTERY = (0.125, (0.0, 0.0, -0.016), (0.034, 0.105, 0.018))   # 2S 2200 mAh LiPo: mass, centre, size
ELECTRONICS = (0.240, (0.0, 0.0, 0.012))  # ESP32, PCA9685, buck converters, switch, sensors, wiring
BODY_HARDWARE = 0.080                     # frame screws/nuts, hip bearings
LEG_HARDWARE = {"hip": 0.008, "femur": 0.020, "tibia": 0.013}  # screws, horns, knee bearing

# Servos: position-controlled MG996R, ~11 kg*cm (~1.0 N*m) stall torque at ~6 V from the SZBK07.
# For 20-35 kg*cm servos (DS3218, CLS6336HV) use 2.0-3.4.
SERVO_KP = 30.0         # N*m/rad
SERVO_TORQUE = 1.0      # N*m
JOINT_RANGE = {"hip": (-0.8, 0.8), "femur": (-2.4, 1.4), "tibia": (0.0, 2.7)}

COLORS = {"frame": "0.15 0.15 0.15 1", "shell": "0.95 0.75 0.2 1", "leg": "0.15 0.15 0.15 1",
          "foot": "0.3 0.3 0.3 1"}
PART_COLOR = {"cover": "shell", "foot": "foot", "wrist": "leg", "arm_joint": "frame", "shoulder": "frame"}


def _fmt(v):
    return " ".join(f"{x:.6g}" for x in v)


def _color(mesh):
    for key, c in PART_COLOR.items():
        if key in mesh:
            return COLORS[c]
    return COLORS["leg"] if "arm" in mesh else COLORS["frame"]


def mesh_geom(part):
    pose = np.array(part["pose"])
    quat = np.zeros(4)
    mujoco.mju_mat2Quat(quat, pose[:3, :3].flatten())
    return (f'<geom type="mesh" mesh="{part["mesh"]}" pos="{_fmt(pose[:3, 3])}" quat="{_fmt(quat)}" '
            f'rgba="{_color(part["mesh"])}" class="visual"/>')


def _box_inertia(m, size):
    x, y, z = size
    return np.diag([m / 12 * (y * y + z * z), m / 12 * (x * x + z * z), m / 12 * (x * x + y * y)])


def combine(items):
    """[(mass, com(3), inertia about com(3x3))] -> (mass, com, inertia about com)."""
    M = sum(m for m, _, _ in items)
    c = sum(m * np.asarray(p) for m, p, _ in items) / M
    I = np.zeros((3, 3))
    for m, p, Ic in items:
        d = np.asarray(p) - c
        I += Ic + m * (d @ d * np.eye(3) - np.outer(d, d))
    return M, c, I


def inertial_xml(body):
    """Mass model of one sim body: printed parts on it + the non-printed components."""
    items = [(p["mass"], p["com"], np.array(p["inertia"])) for p in PARTS["parts"] if p["body"] == body]
    if body == "base":
        items.append((sum(h["mass"] for h in PARTS["hidden"]) + BODY_HARDWARE, (0, 0, 0), np.zeros((3, 3))))
        items.append((BATTERY[0], BATTERY[1], _box_inertia(BATTERY[0], BATTERY[2])))
        items.append((ELECTRONICS[0], ELECTRONICS[1], np.zeros((3, 3))))
    else:
        lg, link = body.split("_")
        c = np.array(PARTS["servo_centers"][link]) * [LEG_SIDE[lg], 1, 1]  # stored for left legs
        items.append((SERVO_MASS, c, _box_inertia(SERVO_MASS, SERVO_SIZE)))
        printed = combine(items[:-1])
        items.append((LEG_HARDWARE[link], printed[1], np.zeros((3, 3))))
    m, com, I = combine(items)
    full = [I[0, 0], I[1, 1], I[2, 2], I[0, 1], I[0, 2], I[1, 2]]
    return f'<inertial pos="{_fmt(com)}" mass="{m:.4f}" fullinertia="{_fmt(full)}"/>'


def visuals(body, indent):
    return ("\n" + " " * indent).join(mesh_geom(p) for p in PARTS["parts"] if p["body"] == body)


def leg_xml(lg):
    side = LEG_SIDE[lg]
    # Hinge axes derived from the firmware legIK conventions (verified against leg_fk in the tests):
    # theta1 rolls the leg outward about the fore-aft axis; theta2/theta3 swing it forward.
    hip_axis = (0.0, -side, 0.0)
    pitch_axis = (-1.0, 0.0, 0.0)
    rng = {k: _fmt(v) for k, v in JOINT_RANGE.items()}
    return f"""
      <body name="{lg}_hip" pos="{_fmt(HIP_POS[lg])}">
        {inertial_xml(f"{lg}_hip")}
        <joint name="{lg}_hip_joint" axis="{_fmt(hip_axis)}" range="{rng['hip']}"/>
        {visuals(f"{lg}_hip", 8)}
        <body name="{lg}_femur" pos="{_fmt((side * COXA, 0, -COXA_OFFSET))}">
          {inertial_xml(f"{lg}_femur")}
          <joint name="{lg}_femur_joint" axis="{_fmt(pitch_axis)}" range="{rng['femur']}"/>
          {visuals(f"{lg}_femur", 10)}
          <geom type="capsule" fromto="0 0 0 0 0 {-FEMUR:.6g}" size="0.015" class="collision"/>
          <body name="{lg}_tibia" pos="0 0 {-FEMUR:.6g}">
            {inertial_xml(f"{lg}_tibia")}
            <joint name="{lg}_tibia_joint" axis="{_fmt(pitch_axis)}" range="{rng['tibia']}"/>
            {visuals(f"{lg}_tibia", 12)}
            <geom type="capsule" fromto="0 0 0 0 0 {-TIBIA + FOOT_RADIUS:.6g}" size="0.01" class="collision"/>
            <geom name="foot_{lg}" type="sphere" pos="0 0 {-TIBIA:.6g}" size="{FOOT_RADIUS}" class="collision"/>
            <site name="foot_{lg}" pos="0 0 {-TIBIA:.6g}" size="0.005" rgba="0 1 0 0.4"/>
          </body>
        </body>
      </body>"""


def scene_xml(terrain: bool) -> str:
    box = PARTS["base_box"]
    if terrain:
        floor_asset = '<hfield name="terrain" nrow="129" ncol="129" size="2.0 2.0 0.05 0.1"/>'
        floor = ('<geom name="floor" type="hfield" hfield="terrain" contype="1" conaffinity="1" '
                 'material="grid"/>')
    else:
        floor_asset = ""
        floor = '<geom name="floor" type="plane" size="0 0 0.05" contype="1" conaffinity="1" material="grid"/>'
    actuators = "\n".join(
        f'    <position name="{lg}_{j}_joint" joint="{lg}_{j}_joint" kp="{SERVO_KP}" dampratio="1" '
        f'ctrlrange="{_fmt(JOINT_RANGE[j])}" forcerange="{-SERVO_TORQUE} {SERVO_TORQUE}"/>'
        for lg in LEG_NAMES for j in ("hip", "femur", "tibia"))
    meshes = sorted({p["mesh"] for p in PARTS["parts"]})
    legs = "".join(leg_xml(lg) for lg in LEG_NAMES)
    return f"""<mujoco model="spotmicro_esp32">
  <!-- GENERATED by generate_scene.py from the measured SpotMicro dimensions and the printed parts
       (print_parts.json). Edit the generator, not this file.
       Base frame: +X left, -Y forward, +Z up; origin at hip height. Stand height {STAND_Z:.4f} m,
       firmware-formula body-height range {MIN_BODY_HEIGHT:.3f}-{MAX_BODY_HEIGHT:.3f} m. -->
  <option timestep="0.002" iterations="50" solver="Newton"/>
  <compiler angle="radian" meshdir="meshes/"/>

  <default>
    <geom conaffinity="0"/>
    <joint damping="0.05" frictionloss="0.05" armature="0.01"/>
    <default class="visual">
      <geom contype="0" conaffinity="0" group="1" density="0"/>
    </default>
    <default class="collision">
      <geom contype="1" group="3" rgba="0.8 0.2 0.2 0.5"/>
    </default>
  </default>

  <asset>
    <texture name="grid" type="2d" builtin="checker" rgb1="0.72 0.74 0.78" rgb2="0.62 0.64 0.68"
             width="300" height="300" mark="edge" markrgb="0.5 0.5 0.55"/>
    <material name="grid" texture="grid" texrepeat="8 8" texuniform="true"/>
{os.linesep.join(f'    <mesh name="{m}" file="{m}.stl" scale="0.001 0.001 0.001"/>' for m in meshes)}
    {floor_asset}
  </asset>

  <worldbody>
    <light pos="0 0 2" dir="0 0 -1" directional="true"/>
    {floor}
    <body name="base_link" pos="0 0 {STAND_Z:.4f}">
      <freejoint name="root"/>
      <site name="imu" pos="0 0 0" size="0.01" rgba="1 0 0 0.3"/>
      {inertial_xml("base")}
      {visuals("base", 6)}
      <geom type="box" pos="{_fmt(box["center"])}" size="{_fmt(box["half"])}" class="collision"/>{legs}
    </body>
  </worldbody>

  <actuator>
{actuators}
  </actuator>

  <sensor>
    <framequat name="imu_quat" objtype="site" objname="imu"/>
    <gyro name="imu_gyro" site="imu"/>
    <accelerometer name="imu_acc" site="imu"/>
  </sensor>
</mujoco>
"""


def main():
    for name, terrain in (("scene.xml", False), ("scene_terrain.xml", True)):
        path = os.path.join(HERE, name)
        with open(path, "w") as f:
            f.write(scene_xml(terrain))
        mujoco.MjModel.from_xml_path(path)  # fail loudly if the generated model doesn't compile
        print(f"wrote {path}")


if __name__ == "__main__":
    main()
