"""Drive the SpotMicro ESP32 robot interactively in the MuJoCo viewer (firmware gait, zero residual).

Steer with the keyboard in the viewer window, and type commands in the terminal to change
gait, body pose, step height, push the robot, etc. Type `help` in the terminal for the list.

Viewer keys:
  Up / Down          forward / backward speed
  Left / Right       turn left / right
  Page Up / Down     strafe left / right
  Enter              stop (stand still)
  Delete             reset the robot to the origin

Examples:
  python teleop.py
  python teleop.py --terrain 0.02     # random bumps up to 20 mm
  python teleop.py --stairs 0.005     # staircase ahead with 5 mm steps
"""

import argparse
import math
import queue
import signal
import threading
import time

import glfw
import mujoco
import mujoco.viewer
import numpy as np

from src.sim.mj_runtime import SpotMicroSim, CONTROL_DT, MODEL_PATH, TERRAIN_MODEL_PATH
from src.sim.stairs import Stairs
from src.sim.terrain import randomize_hfield
from src.robot.firmware_gait import (GaitController, GaitState, BodyState, analytic_gait_action, set_mode, TROT,
                                     CRAWL, STAND_Z)
from src.leika.robot import MAX_VX, MAX_VY, MAX_YAW

VX_STEP, VY_STEP, YAW_STEP = 0.025, 0.015, 0.2
NARGS = {"vx": 1, "vy": 1, "turn": 1, "step": 1, "roll": 1, "pitch": 1, "twist": 1, "height": 1,
         "shift": 2, "push": 1}

HELP = """
commands (angles in degrees, distances in mm, speeds in m/s and rad/s):
  vx <v> | vy <v> | turn <w>   set forward / left speed or yaw rate
  stop                         zero the velocity command
  trot | crawl                 switch gait
  step <mm>                    swing step height (default from gait_coef.json)
  roll <deg> | pitch <deg> | twist <deg>     body orientation
  height <mm> | shift <x_mm> <y_mm>          body translation
  pose                         reset body pose to neutral
  push <vx> <vy>               kick the base sideways/forward (m/s)
  terrain [max_mm]             regenerate bumps (needs --terrain)
  follow                       toggle camera tracking
  reset                        put the robot back at the origin
  status                       print position / height / command
  quit
"""


class Teleop:
    def __init__(self, terrain: float, seed: int, stairs: float = 0.0):
        self.sim = SpotMicroSim(TERRAIN_MODEL_PATH if terrain > 0 else MODEL_PATH,
                                stairs=Stairs(rise=stairs) if stairs > 0 else None)
        self.terrain = terrain
        self.rng = np.random.default_rng(seed)
        if terrain > 0:
            randomize_hfield(self.sim.model, self.rng, terrain)

        self.cmd = np.zeros(3)  # [vx, vy, yaw]
        self.body = BodyState()
        self.gait = GaitState()
        self.gc = GaitController()
        self.mode = TROT
        self.step_height = None  # None: use the calibrated value
        self.follow = True
        self.running = True
        self.viewer = None
        # Keys and terminal commands run on other threads; the sim loop applies them.
        self.actions = queue.Queue()
        self.reset()

    # --- state changes (called from the sim loop only) -------------------------------
    def reset(self):
        self.sim.reset_to_stand()
        self.gait = GaitState()
        set_mode(self.gait, self.mode)
        self.gc = GaitController()

    def set_cmd(self, i, value, limit):
        self.cmd[i] = float(np.clip(value, -limit, limit))

    def push(self, vx, vy):
        a = self.sim.model.jnt_dofadr[mujoco.mj_name2id(self.sim.model, mujoco.mjtObj.mjOBJ_JOINT, "root")]
        # world frame: robot forward is -Y, left is +X
        self.sim.data.qvel[a] += vy
        self.sim.data.qvel[a + 1] -= vx

    def regen_terrain(self, max_height):
        if self.terrain <= 0:
            print("terrain: start with --terrain <metres> to use the heightfield scene")
            return
        self.terrain = max_height
        randomize_hfield(self.sim.model, self.rng, max_height)
        hid = mujoco.mj_name2id(self.sim.model, mujoco.mjtObj.mjOBJ_HFIELD, "terrain")
        self.viewer.update_hfield(hid)
        self.reset()

    def set_follow(self, on):
        self.follow = on
        cam = self.viewer.cam
        if on:
            cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
            cam.trackbodyid = self.sim.base_id
            cam.distance = 1.0
            cam.elevation = -25
        else:
            cam.type = mujoco.mjtCamera.mjCAMERA_FREE

    # --- input ------------------------------------------------------------------------
    def on_key(self, key):
        c = self.cmd
        bindings = {
            glfw.KEY_UP: lambda: self.set_cmd(0, c[0] + VX_STEP, MAX_VX),
            glfw.KEY_DOWN: lambda: self.set_cmd(0, c[0] - VX_STEP, MAX_VX),
            glfw.KEY_PAGE_UP: lambda: self.set_cmd(1, c[1] + VY_STEP, MAX_VY),
            glfw.KEY_PAGE_DOWN: lambda: self.set_cmd(1, c[1] - VY_STEP, MAX_VY),
            glfw.KEY_LEFT: lambda: self.set_cmd(2, c[2] + YAW_STEP, MAX_YAW),
            glfw.KEY_RIGHT: lambda: self.set_cmd(2, c[2] - YAW_STEP, MAX_YAW),
            glfw.KEY_ENTER: lambda: c.fill(0.0),
            glfw.KEY_DELETE: self.reset,
        }
        if key in bindings:
            self.actions.put(bindings[key])

    def parse(self, line):
        """Turn one terminal line into an action for the sim loop (or None)."""
        parts = line.split()
        if not parts:
            return None
        name, args = parts[0].lower(), [float(a) for a in parts[1:]]
        b = self.body
        simple = {
            "vx": lambda: self.set_cmd(0, args[0], MAX_VX),
            "vy": lambda: self.set_cmd(1, args[0], MAX_VY),
            "turn": lambda: self.set_cmd(2, args[0], MAX_YAW),
            "stop": lambda: self.cmd.fill(0.0),
            "trot": lambda: (setattr(self, "mode", TROT), set_mode(self.gait, TROT)),
            "crawl": lambda: (setattr(self, "mode", CRAWL), set_mode(self.gait, CRAWL)),
            "step": lambda: setattr(self, "step_height", args[0] / 1000),
            "roll": lambda: setattr(b, "omega", math.radians(args[0])),
            "pitch": lambda: setattr(b, "phi", math.radians(args[0])),
            "twist": lambda: setattr(b, "psi", math.radians(args[0])),
            "height": lambda: setattr(b, "zm", args[0] / 1000),
            "shift": lambda: (setattr(b, "xm", args[0] / 1000), setattr(b, "ym", args[1] / 1000)),
            "pose": lambda: [setattr(b, k, 0.0) for k in ("omega", "phi", "psi", "xm", "ym", "zm")],
            "push": lambda: self.push(args[0], args[1] if len(args) > 1 else 0.0),
            "terrain": lambda: self.regen_terrain(args[0] / 1000 if args else self.terrain),
            "follow": lambda: self.set_follow(not self.follow),
            "reset": self.reset,
            "status": self.print_status,
        }
        if name in ("help", "?"):
            print(HELP)
            return None
        if name in ("quit", "exit", "q"):
            self.running = False
            return None
        if name not in simple:
            print(f"unknown command {name!r} (type 'help')")
            return None
        if len(args) < NARGS.get(name, 0):
            print(f"{name} needs {NARGS[name]} number(s) (type 'help')")
            return None
        return simple[name]

    def read_terminal(self):
        print(HELP)
        while self.running:
            try:
                line = input("> ")
            except EOFError:  # no terminal attached: keep going with viewer keys only
                print("\n(no terminal input; use the viewer keys, close the window to quit)")
                break
            try:
                action = self.parse(line)
            except ValueError:
                print("arguments must be numbers (type 'help')")
                continue
            if action is not None:
                self.actions.put(action)

    # --- loop -------------------------------------------------------------------------
    def print_status(self):
        a = self.sim.base_qposadr
        x, y, z = self.sim.data.qpos[a : a + 3]
        print(f"pos x={x:+.3f} y={y:+.3f}  height={z:.4f} m  "
              f"cmd vx={self.cmd[0]:+.2f} vy={self.cmd[1]:+.2f} yaw={self.cmd[2]:+.2f}")

    def overlay(self):
        h = self.sim.base_height()
        left = "gait\nvx\nvy\nyaw\nheight"
        right = (f"{'trot' if self.mode == TROT else 'crawl'}\n{self.cmd[0]:+.2f} m/s\n"
                 f"{self.cmd[1]:+.2f} m/s\n{self.cmd[2]:+.2f} rad/s\n{h * 1000:.0f} mm"
                 + ("  FELL (Delete to reset)" if h < 0.5 * STAND_Z else ""))
        self.viewer.set_texts((None, mujoco.mjtGridPos.mjGRID_TOPLEFT, left, right))

    def overlay_loop(self):
        # set_texts() blocks until the next rendered frame, so keep it off the sim thread.
        while self.running and self.viewer.is_running():
            self.overlay()
            time.sleep(0.1)

    def step(self):
        while not self.actions.empty():
            try:
                self.actions.get_nowait()()
            except Exception as e:  # a bad command shouldn't kill the sim
                print(f"error: {e}")
        analytic_gait_action(self.cmd, self.gait)
        if self.step_height is not None:
            self.gait.step_height = self.step_height
        self.gc.advance_phase(self.gait, CONTROL_DT)
        self.gc.generate_feet(self.gait, self.body)
        self.sim.set_joint_targets(self.sim.body_targets_from_feet(self.body))
        self.sim.step_physics()

    def sim_loop(self, handle_return):
        viewer = self.viewer = handle_return.get()
        try:
            self.set_follow(True)
            threading.Thread(target=self.read_terminal, daemon=True).start()
            threading.Thread(target=self.overlay_loop, daemon=True).start()
            deadline = time.perf_counter()
            while self.running and viewer.is_running():
                with viewer.lock():
                    self.step()
                viewer.sync()
                # Fixed-rate schedule so short stalls are caught up; give up after 0.1 s behind.
                deadline += CONTROL_DT
                now = time.perf_counter()
                if deadline > now:
                    time.sleep(deadline - now)
                elif now - deadline > 0.1:
                    deadline = now
        finally:
            self.running = False
            viewer.close()

    def run(self):
        # launch_passive() runs the GUI on a background thread, where Wayland's GTK window
        # decorations (libdecor) fail to initialise. Instead run the GUI on the main thread and
        # the sim on a worker, using the same internal entry point launch_passive() wraps.
        handle_return = queue.Queue(1)
        threading.Thread(target=self.sim_loop, args=(handle_return,), daemon=True).start()
        mujoco.viewer._launch_internal(self.sim.model, self.sim.data, run_physics_thread=False,
                                       handle_return=handle_return, key_callback=self.on_key,
                                       show_left_ui=False, show_right_ui=False)
        self.running = False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--terrain", type=float, default=0.0, help="max bump height (m) of random terrain")
    ap.add_argument("--stairs", type=float, default=0.0, help="add a staircase ahead of the robot with this step height (m); with MG996R servos the baseline gait climbs ~0.005")
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()
    # Wayland can't report window positions; GLFW warns about it on every launch.
    glfw.ERROR_REPORTING = {glfw.FEATURE_UNAVAILABLE: "ignore", None: "warn"}
    # The main thread sits in the GUI's C++ loop, so let Ctrl+C kill the process directly.
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    Teleop(args.terrain, args.seed, args.stairs).run()


if __name__ == "__main__":
    main()
