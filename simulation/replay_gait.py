"""Run the ported firmware gait (zero residual) on the SpotMicro ESP32 robot in MuJoCo.

Two uses:
  - GUI (default): launch the interactive viewer and walk with a fixed command, to visually
    validate the gait + IK on the real model.
  - --headless: no GUI; roll out a set of commands and report net displacement / achieved
    velocity. This is the numeric validation of the port (forward moves the robot forward, etc.).

Examples:
  python replay_gait.py --vx 0.1                  # viewer, walk forward
  python replay_gait.py --vx 0 --vy 0 --yaw 0.8   # viewer, turn in place
  python replay_gait.py --headless                # validate a battery of commands
  python replay_gait.py --stairs 0.005 --seconds 30  # viewer, walk over 5 mm stairs
"""

import argparse

import numpy as np

from src.sim.mj_runtime import SpotMicroSim, CONTROL_DT
from src.sim.stairs import Stairs
from src.robot.firmware_gait import GaitController, GaitState, BodyState, analytic_gait_action, set_mode, TROT, STAND_Z


def rollout(sim, cmd, seconds, viewer=None):
    """Run the zero-residual gait for `seconds`; return (dx, dy, dz) world base displacement."""
    gait = GaitState()
    set_mode(gait, TROT)
    gc = GaitController()
    body = BodyState()
    sim.reset_to_stand()
    x0 = sim.data.qpos[0:3].copy()
    n = int(seconds / CONTROL_DT)
    for _ in range(n):
        analytic_gait_action(cmd, gait)
        gc.advance_phase(gait, CONTROL_DT)
        gc.generate_feet(gait, body)
        sim.set_joint_targets(sim.body_targets_from_feet(body))
        sim.step_physics()
        if viewer is not None:
            if not viewer.is_running():
                break
            viewer.sync()
    return sim.data.qpos[0:3].copy() - x0, sim.base_height()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--vx", type=float, default=0.1, help="forward velocity command (m/s)")
    ap.add_argument("--vy", type=float, default=0.0, help="left velocity command (m/s)")
    ap.add_argument("--yaw", type=float, default=0.0, help="yaw-rate command (rad/s)")
    ap.add_argument("--seconds", type=float, default=8.0)
    ap.add_argument("--headless", action="store_true", help="no viewer; validate a battery of commands")
    ap.add_argument("--stairs", type=float, default=0.0, help="add a staircase ahead of the robot with this step height (m); with MG996R servos the baseline gait climbs ~0.005")
    args = ap.parse_args()

    sim = SpotMicroSim(stairs=Stairs(rise=args.stairs) if args.stairs > 0 else None)

    if args.headless:
        cmds = {
            "stand":   (0.0, 0.0, 0.0),
            "forward": (0.10, 0.0, 0.0),
            "back":    (-0.05, 0.0, 0.0),
            "left":    (0.0, 0.05, 0.0),
            "turn+":   (0.0, 0.0, 0.8),
        }
        print(f"{'command':9s} {'dx':>8s} {'dy(fwd=-)':>10s} {'dz':>8s} {'height':>8s}   note")
        for name, cmd in cmds.items():
            d, h = rollout(sim, cmd, args.seconds)
            note = "FELL" if h < 0.5 * STAND_Z else "ok"
            print(f"{name:9s} {d[0]:8.3f} {d[1]:10.3f} {d[2]:8.3f} {h:8.4f}   {note}")
        print(f"\nExpected: forward -> dy negative; back -> dy positive; left -> dx positive; upright height ~{STAND_Z:.2f}.")
        return

    import mujoco.viewer
    with mujoco.viewer.launch_passive(sim.model, sim.data) as viewer:
        d, h = rollout(sim, (args.vx, args.vy, args.yaw), args.seconds, viewer=viewer)
        print(f"displacement (world): {np.round(d, 3)}  final height {h:.4f}")


if __name__ == "__main__":
    main()
