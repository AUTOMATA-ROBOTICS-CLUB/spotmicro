"""Staircase obstacle: a flight of steps up, a landing, and a flight down, in front of the robot.

Added to the scene at load time (MjSpec), so the rise/tread/count can be chosen per run
(`--stairs <rise_m>` on replay_gait.py, teleop.py, train_mj.py, eval_policy.py). The steps are
nested box slabs spanning the full width, so the robot can walk over them in either direction.
Robot forward is -Y, so the stairs start `start` metres ahead of the origin along -Y.

The baseline gait and the policy are blind (IMU only). With MG996R servos and the 20 mm swing the
zero-residual gait clears ~5 mm steps; 20-35 kg*cm servos with a 40 mm swing clear ~20 mm.
"""

from dataclasses import dataclass

import mujoco

STEP_COLORS = ([0.55, 0.45, 0.35, 1.0], [0.47, 0.38, 0.30, 1.0])  # alternate so step edges are visible


@dataclass
class Stairs:
    rise: float = 0.02      # step height (m)
    tread: float = 0.20     # step depth (m); about the robot's front-to-rear foot spacing (0.21 m)
    steps: int = 4          # steps up (and the same number down)
    landing: float = 0.40   # flat top between the two flights (m)
    width: float = 1.20     # across the direction of travel (m)
    start: float = 0.45     # distance from the origin to the first step edge (m)

    @property
    def length(self) -> float:
        return 2 * self.steps * self.tread + self.landing


def add_stairs(spec: mujoco.MjSpec, stairs: Stairs) -> None:
    """Add the staircase as stacked slabs: slab k is one rise thick and k treads shorter per end."""
    for k in range(stairs.steps):
        half_len = stairs.length / 2 - k * stairs.tread
        spec.worldbody.add_geom(
            name=f"stair_{k}",
            type=mujoco.mjtGeom.mjGEOM_BOX,
            size=[stairs.width / 2, half_len, stairs.rise / 2],
            pos=[0.0, -(stairs.start + stairs.length / 2), (k + 0.5) * stairs.rise],
            rgba=STEP_COLORS[k % 2],
            contype=1,
            conaffinity=1,
        )
