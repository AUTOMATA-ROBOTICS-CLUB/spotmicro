"""Place the printed SpotMicro parts (KDY0523 SpotMicro, MG996R variants) on the simulation skeleton.

The print STLs are separate parts in print orientation. The SpotMicroESP32-Leika web controller's
model (`reference/web_model_stl.zip`, from its app/static/stl.zip) contains simplified copies of the same parts already
assembled, so each print part is fitted onto its web counterpart (point-to-point ICP, 24 axis
orientations), giving its assembled pose. Each part is then anchored on its own pivot feature
(servo shaft, bearing pin, ...) so it sits exactly on the simulated joint axes.

Writes `print_parts.json` (per part: link, mesh pose in the link frame, 20%-infill mass properties)
and copies the used STLs into `meshes/`. `generate_scene.py` reads that JSON; this script only
needs re-running if the parts or the skeleton change.

    uv run --with trimesh python -m src.resources.spotmicro_esp32.fit_print_meshes [PRINT_STL_DIR]

PRINT_STL_DIR defaults to the repo's `hardware/print` folder.
"""

import io
import itertools
import json
import math
import os
import shutil
import sys
import zipfile

import numpy as np
import trimesh
from scipy.spatial import cKDTree

from src.robot.firmware_gait import COXA, COXA_OFFSET, FEMUR, TIBIA, HIP_POS, LEG_SIDE

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", "..", "..", ".."))
WEB_ZIP = os.path.join(HERE, "reference", "web_model_stl.zip")
PRINT_DIR = sys.argv[1] if len(sys.argv) > 1 else os.path.join(REPO, "hardware", "print")

R_URDF = np.array([[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])  # URDF (fwd, left, up) -> sim
MIRROR_X = np.diag([-1.0, 1.0, 1.0])                                         # sim left <-> right
PLA_DENSITY, SHELL_CM, INFILL = 1.24, 0.10, 0.20                             # g/cm3, wall thickness, infill


def T(R=np.eye(3), t=(0, 0, 0)):
    M = np.eye(4); M[:3, :3] = R; M[:3, 3] = t
    return M


def rpy(r, p, y):
    Rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
    Ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
    Rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
    return Rz @ Ry @ Rx


# ---------------------------------------------------------------------------------------------
# Web model, front-left leg + body, assembled in the URDF frame (mm). Values from the xacro.
# ---------------------------------------------------------------------------------------------
_HIP = np.array([93.0, 39.0, 0.0]); _LEG = _HIP + [0, 55, 0]; _KNEE = _LEG + [0, 0, -107.5]; _TOE = _KNEE + [0, 0, -130]
WEB_POSE = {
    "lshoulder": (_HIP, (0, 0, math.pi), (0.135, 0.015, -0.01)),
    "larm": (_LEG, (0, -0.139, math.pi), (0.130, -0.040, -0.025)),
    "larm_cover": (_LEG, (0, -0.139, math.pi), (0.130, -0.040, -0.025)),
    "lfoot": (_KNEE, (0, 0, math.pi), (0.120, -0.04, 0.1)),
    "foot": (_TOE, (0, -0.4001, math.pi), (0.0, 0.01, 0.015)),
    "mainbody": (np.zeros(3), (0, 0, 0), (-0.042, -0.055, -0.010)),
    "backpart": (np.zeros(3), (0, 0, math.pi), (0.04, 0.055, -0.010)),
    "frontpart": (np.zeros(3), (0, 0, math.pi), (0.040, 0.055, -0.010)),
}


def web_mesh(name, zf):
    org, r, xyz = WEB_POSE[name]
    m = trimesh.load(io.BytesIO(zf.read(f"stl/{name}.stl")), file_type="stl", force="mesh")
    m.apply_transform(T(rpy(*r), np.array(xyz) * 1000 + org))
    return m


# ---------------------------------------------------------------------------------------------
# Rigid fitting (part surface points onto target surface points)
# ---------------------------------------------------------------------------------------------
ROTS = [R for R in (np.array(p) * np.array(s)[:, None] for p in itertools.permutations(np.eye(3))
                    for s in itertools.product((1, -1), repeat=3)) if np.linalg.det(R) > 0]


def _kabsch(P, Q):
    cp, cq = P.mean(0), Q.mean(0)
    U, _, Vt = np.linalg.svd((P - cp).T @ (Q - cq))
    D = np.diag([1, 1, np.sign(np.linalg.det(Vt.T @ U.T))])
    R = Vt.T @ D @ U.T
    return T(R, cq - R @ cp)


def _icp(pts, tree, tpts, M, iters):
    for _ in range(iters):
        d, i = tree.query(pts @ M[:3, :3].T + M[:3, 3])
        keep = d <= np.percentile(d, 90)  # trim: web meshes contain extra parts (servos, merged pieces)
        M = _kabsch(pts[keep], tpts[i[keep]])
    d, _ = tree.query(pts @ M[:3, :3].T + M[:3, 3])
    return M, d


def fit(src, tgt):
    """Best rigid transform taking `src` onto `tgt` (mm). Returns (4x4, rms mm)."""
    pts = trimesh.sample.sample_surface(src, 1500, seed=0)[0]
    tpts = trimesh.sample.sample_surface(tgt, 40000, seed=1)[0]
    tree = cKDTree(tpts)
    tc, slack = tgt.bounds.mean(0), np.maximum(tgt.extents - src.extents.min(), 0) / 4
    best = (np.inf, None)
    for R in ROTS:
        for off in [np.zeros(3)] + [s * slack[i] * np.eye(3)[i] for i in range(3) for s in (-1, 1)]:
            M, d = _icp(pts, tree, tpts, T(R, tc - R @ src.bounds.mean(0) + off), 25)
            score = np.sqrt(np.mean(np.sort(d)[: int(0.9 * len(d))] ** 2))
            if score < best[0]:
                best = (score, M)
    M, d = _icp(pts, tree, tpts, best[1], 100)
    return M, float(np.sqrt(np.mean(d ** 2)))


def load(name):
    return trimesh.load(os.path.join(PRINT_DIR, f"{name}.stl"), force="mesh")


def mirrored(m):
    m = m.copy(); m.apply_transform(T(MIRROR_X)); m.invert()
    return m


# ---------------------------------------------------------------------------------------------
# Pivot features measured on the parts (part coordinates, mm) -- see the measurement notes in
# the simulation README. "shaft" points lie on an axis; their position along that axis is free.
# ---------------------------------------------------------------------------------------------
ARM_JOINT_FEMUR_DISC = [36.5, 10.25, 20.25]  # L_arm_joint_mg: femur-servo horn disc centre (femur axis)
ARM_FEMUR_SHAFT = [10.0, 6.0, 9.5]           # L_arm_mg: femur servo shaft (pocket ears at Z -4 / 44.5)
ARM_KNEE_EYE = [-3.82, -6.6, 116.93]         # L_arm_mg: knee pivot eye
WRIST_KNEE_PIN = [9.5, -64.0, 10.0]          # L_wrist_mg: knee pin (coaxial with the knee servo shaft)
WRIST_FOOT_EYE = [132.01, -40.0, 12.49]      # L_wrist_mg: foot mount hole
# servo body centres (MG996R mounting-ear centre), for the mass model
SERVO_CENTERS = {"hip": ("L_arm_joint_mg", [10.0, 0.0, 20.25]), "femur": ("L_arm_mg", [10.0, 6.6, 20.25]),
                 "tibia": ("L_wrist_mg", [20.25, -40.0, 10.0])}


def place(M_fit, anchor_part, anchor_link_m, down_from=None):
    """Pose (4x4, metres) of a part in a sim link frame.

    M_fit: part -> URDF assembly (mm). The part keeps its fitted orientation (rotated into the sim
    frame) and is translated so `anchor_part` lands on `anchor_link_m` -- except the lateral (sim x)
    coordinate, which keeps the fitted value. `down_from`=(p0, p1) part points that must end up
    vertically aligned (p1 straight below p0): removes the small tilt of the web model's placement.
    """
    R = R_URDF @ M_fit[:3, :3]
    if down_from is not None:
        v = R @ (np.subtract(down_from[1], down_from[0]))
        ang = math.atan2(v[1], -v[2])                     # tilt in the sagittal (y-z) plane
        R = np.array([[1, 0, 0], [0, math.cos(-ang), -math.sin(-ang)], [0, math.sin(-ang), math.cos(-ang)]]) @ R
    a = R @ np.array(anchor_part, float) / 1000
    lateral = (R_URDF @ (M_fit[:3, :3] @ np.array(anchor_part, float) + M_fit[:3, 3]))[0] / 1000
    target = np.array(anchor_link_m, float); target[0] += lateral
    return T(R, target - a)


def mass_props(mesh, pose):
    """20%-infill PLA mass (kg), COM and inertia about the COM (kg m^2) in the link frame."""
    V, A = abs(mesh.volume) / 1000, mesh.area / 100                     # cm3, cm2
    shell = min(V, A * SHELL_CM / 2)
    mass = PLA_DENSITY * (shell + INFILL * (V - shell)) / 1000           # kg
    m = mesh.copy(); m.apply_transform(T(np.eye(3) * 0.001))             # mm -> m
    m.density = mass / abs(m.volume)
    R, t = pose[:3, :3], pose[:3, 3]
    com = R @ m.center_mass + t
    I = R @ m.moment_inertia @ R.T                                       # about COM
    return mass, com, I


def main():
    zf = zipfile.ZipFile(WEB_ZIP)
    parts, used = [], set()

    # --- left front leg: fit each part onto its web counterpart ------------------------------
    fits = {}
    for part, web in (("L_arm_joint_mg", "lshoulder"), ("L_arm_mg", "larm"), ("L_arm_cover", "larm_cover"),
                      ("L_wrist_mg", "lfoot"), ("foot", "foot")):
        fits[part], rms = fit(load(part), web_mesh(web, zf))
        print(f"fit {part:16s} -> {web:11s} rms {rms:.2f} mm")

    # leg poses in their sim link frames (front-left; x = left, y = rear, z = up)
    leg = {}
    leg["L_arm_joint_mg"] = ("hip", place(fits["L_arm_joint_mg"], ARM_JOINT_FEMUR_DISC, [0, 0, -COXA_OFFSET]))
    # the hip link frame sits on the hip axis; its disc anchor's lateral offset is relative to the
    # fitted hip-axis line, so re-express the lateral coordinate relative to the hip origin
    hip_axis_y = (fits["L_arm_joint_mg"][:3, :3] @ [10.0, 0, 9.5] + fits["L_arm_joint_mg"][:3, 3])[1] / 1000
    leg["L_arm_joint_mg"][1][0, 3] -= hip_axis_y
    leg_plane_y = (_LEG[1]) / 1000  # web model leg plane (y = 94 mm)
    arm = place(fits["L_arm_mg"], ARM_FEMUR_SHAFT, [0, 0, 0], down_from=(ARM_FEMUR_SHAFT, ARM_KNEE_EYE))
    arm[0, 3] -= leg_plane_y
    leg["L_arm_mg"] = ("femur", arm)
    # cover rides rigidly on the arm (same web link -> relative pose from the fit)
    rel = np.linalg.inv(fits["L_arm_mg"]) @ fits["L_arm_cover"]
    leg["L_arm_cover"] = ("femur", arm @ T(np.eye(3), [0, 0, 0]) @ _mm(rel))
    wrist = place(fits["L_wrist_mg"], WRIST_KNEE_PIN, [0, 0, 0], down_from=(WRIST_KNEE_PIN, WRIST_FOOT_EYE))
    wrist[0, 3] -= leg_plane_y
    leg["L_wrist_mg"] = ("tibia", wrist)
    rel = np.linalg.inv(fits["L_wrist_mg"]) @ fits["foot"]
    leg["foot"] = ("tibia", wrist @ _mm(rel))

    # --- right side: R_ parts are mirror images of L_ parts --------------------------------------
    right = {}
    for lpart, (link, pose) in leg.items():
        rpart = lpart.replace("L_", "R_", 1) if lpart.startswith("L_") else lpart
        G, rms = fit(load(rpart), mirrored(load(lpart)))   # R part -> mirrored L part coords
        right[rpart] = (link, T(MIRROR_X) @ pose @ _mirror_part(G))   # mirror(left pose of the mirrored part)
        print(f"mirror {rpart:16s} rms {rms:.2f} mm")

    for lg in ("fl", "rl", "fr", "rr"):
        for name, (link, pose) in (leg if LEG_SIDE[lg] > 0 else right).items():
            parts.append({"mesh": name, "body": f"{lg}_{link}", "pose": pose})

    # --- body ------------------------------------------------------------------------------------
    body = trimesh.util.concatenate([web_mesh(n, zf) for n in ("mainbody", "frontpart", "backpart")])
    c = body.triangles_center
    sel = {"front": c[:, 0] > 20, "rear": c[:, 0] < -20, "top": c[:, 2] > 0, "bottom": c[:, 2] < 0,
           "front_cover": c[:, 0] > 60, "rear_cover": c[:, 0] < -60}
    jobs = [("I_shoulder_mg", "front"), ("I_shoulder_mg", "rear"), ("O_shoulder", "front"),
            ("non-mega_T_cover_mg", "top"), ("non-mega_B_cover_mg", "bottom"), ("F_cover", "front_cover"),
            ("R_cover", "rear_cover")]
    bfit = {}
    for part, s in jobs:
        bfit[(part, s)], rms = fit(load(part), body.submesh([np.flatnonzero(sel[s])], append=True))
        print(f"fit {part:20s} ({s:11s}) rms {rms:.2f} mm")
    # body frame origin: midway between the front and rear hip bearings, at hip-axis height
    seat = lambda M: M[:3, :3] @ np.array([16.0, 2.0, 26.0]) + M[:3, 3]   # I_shoulder bearing seat
    origin = (seat(bfit[("I_shoulder_mg", "front")]) + seat(bfit[("I_shoulder_mg", "rear")])) / 2
    origin[1] = 0.0
    to_base = lambda M: T(R_URDF) @ T(np.eye(3), -origin / 1000) @ _mm(M)
    for (part, s), M in bfit.items():
        parts.append({"mesh": part, "body": "base", "pose": to_base(M)})
    # rear outer plate: front one turned 180 degrees about z (its own fit lands inside the body)
    parts.append({"mesh": "O_shoulder", "body": "base",
                  "pose": T(rpy(0, 0, math.pi)) @ to_base(bfit[("O_shoulder", "front")])})
    # hidden parts whose fit is ambiguous (flat plates): mass only, lumped at the body centre
    hidden = ["non-mega_L_side_plate", "R_side_plate", "plate", "L_ultra_sonic", "R_ultra_sonic"]

    # --- mass properties + copy meshes -------------------------------------------------------------
    out = {"parts": [], "hidden": [], "servo_centers": {}, "base_box": None}
    for link, (part, pt) in SERVO_CENTERS.items():   # left-leg link frames; mirror x for right legs
        out["servo_centers"][link] = (leg[part][1] @ np.r_[np.array(pt) / 1000, 1])[:3].tolist()
    # body collision box: bounds of the placed body shells (covers) in the base frame
    shells = [p for p in parts if p["body"] == "base" and "cover" in p["mesh"]]
    pts = np.vstack([trimesh.transformations.transform_points(load(p["mesh"]).vertices / 1000, p["pose"]) for p in shells])
    lo, hi = pts.min(0), pts.max(0)
    out["base_box"] = {"center": ((lo + hi) / 2).tolist(), "half": ((hi - lo) / 2).tolist()}
    for p in parts:
        m, com, I = mass_props(load(p["mesh"]), p["pose"])
        out["parts"].append({"mesh": p["mesh"], "body": p["body"], "pose": p["pose"].tolist(),
                             "mass": m, "com": com.tolist(), "inertia": I.tolist()})
        used.add(p["mesh"])
    for h in hidden:
        m, _, _ = mass_props(load(h), np.eye(4))
        out["hidden"].append({"mesh": h, "body": "base", "mass": m})
    mesh_dir = os.path.join(HERE, "meshes")
    for f in os.listdir(mesh_dir):
        os.remove(os.path.join(mesh_dir, f))
    for name in sorted(used):
        shutil.copy(os.path.join(PRINT_DIR, f"{name}.stl"), os.path.join(mesh_dir, f"{name}.stl"))
    with open(os.path.join(HERE, "print_parts.json"), "w") as f:
        json.dump(out, f, indent=1)
    total = sum(p["mass"] for p in out["parts"]) + sum(h["mass"] for h in out["hidden"])
    print(f"wrote print_parts.json ({len(out['parts'])} placed parts, printed mass {total * 1000:.0f} g)")


def _mm(M):
    """4x4 in mm -> same transform in metres."""
    M = np.array(M, float).copy(); M[:3, 3] /= 1000
    return M


def _mirror_part(G):
    """Mesh-coordinate map for a right part: R-part coords -> L-part coords, mirrored (metres)."""
    return T(MIRROR_X) @ _mm(G)


if __name__ == "__main__":
    main()
