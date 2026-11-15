"""Place the printed SpotMicroESP32 parts on the simulation skeleton.

The robot: Michael Kubina's SpotMicroESP32 (https://github.com/michaelkubina/SpotMicroESP32, GPL-3.0; long
knee mounts, SZBK07 electronics plate) with robjk's reinforced shoulders and medium rear cover
(https://www.thingiverse.com/thing:4937631, CC BY), and MG996R servos.

Kubina's assembled model and robjk's FreeCAD files (`reference/assemblies.zip`, made by
`extract_assembly.py`) hold the parts in their assembled poses. Each print STL is fitted onto its
counterpart there (point-to-point ICP, 24 axis orientations); robjk parts that replace a Kubina part are
fitted onto that part. The extended servo cap and shoulder cover have no Kubina counterpart: they keep their
pose relative to the limb shell from robjk's assembly, turned about the femur axis so the cap's arm points
up (onto the servo mount's extension), as on robjk's robot.

The joint axes and feet are measured in the same assembly (servo gears, shoulder pins, foot-tip arc) and
printed next to the skeleton constants in src.robot.firmware_gait, which they should match.

Writes `print_parts.json` (per part: link, mesh pose in the link frame, mass properties) and copies the used
STLs into `meshes/`, with mirrored copies for the right legs. `generate_scene.py` reads that JSON; this
script only needs re-running if the parts or the skeleton change.

    uv run --with trimesh python -m src.resources.spotmicro_esp32.fit_print_meshes [PRINT_STL_DIR]

PRINT_STL_DIR defaults to the repo's `hardware/print` folder.
"""

import io
import itertools
import json
import math
import os
import sys
import zipfile

import numpy as np
import trimesh
from scipy.spatial import ConvexHull, cKDTree

from src.robot import firmware_gait as fg

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", "..", "..", ".."))
REF_ZIP = os.path.join(HERE, "reference", "assemblies.zip")
PRINT_DIR = sys.argv[1] if len(sys.argv) > 1 else os.path.join(REPO, "hardware", "print")

R_ASM = np.array([[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])  # assembly (fwd, left, up) -> sim
MIRROR_X = np.diag([-1.0, 1.0, 1.0])                                     # sim left <-> right
PLA_DENSITY, SHELL_CM = 1.24, 0.10                                       # g/cm3, wall thickness
INFILL, HULL_INFILL = 0.20, 1.0                                          # Kubina prints the hull solid
HULL = ("Front_Cover", "Rear_Cover", "Top_Cover", "Bottom_Cover")

# Front-left leg: (print STL, sim link, assembly solid it replaces). Right legs use mirrored copies.
LEG_PARTS = [
    ("Shoulder_Joint_Servomount_extended_BB", "hip", "Shoulder_Joint_Servomount"),
    ("Limb_Top_Shell_with_BB_hole", "femur", "Limb_Top_Shell"),
    ("Limb_Bottom_Shell", "femur", "Limb_Bottom_Shell"),
    ("Limb_Servohorn_Mount_long_v4", "femur", "Limb_Servohorn_Mount_long_v005"),
    ("Limb_Ball_Bearing_Mount_long_v4", "femur", "Limb_Ball_Bearing_Mount_long_v4"),
    ("Wrist", "tibia", "Wrist"),
    ("Wrist _Counterpart", "tibia", "Wrist_Counterpart"),
    ("Foot_Tip", "tibia", "Foot_Tip"),
]
# robjk shoulder parts placed from robjk's assembly: (print STL, solid there); both ride on the hip link
ROBJK_SHOULDER = [("Shoulder_Joint_Servo_Cap_Extended", "Shoulder_Joint_Servohorn_Cap"),
                  ("Shoulder_Cover", "Shoulder Cover")]
# servo of each link: (assembly servo body, its output gear)
LEG_SERVOS = {"hip": ("MG996R004", "Gear005"), "femur": ("MG996R002", "Gear003"), "tibia": ("MG996R001", "Gear002")}
# Body: (print STL, assembly solid). The assembly's bottom cover halves are labelled the other way round.
BODY_PARTS = [
    ("Inner_Shoulder", "Inner_Shoulder001"), ("Inner_Shoulder", "Inner_Shoulder"),
    ("Outer_Shoulder", "Outer_Shoulder"), ("Outer_Shoulder", "Outer_Shoulder001"),
    ("Top_Shoulder", "Top_Shoulder001"), ("Top_Shoulder", "Top_Shoulder"),
    ("Bottom_Shoulder", "Bottom_Shoulder001"), ("Bottom_Shoulder", "Bottom_Shoulder"),
    ("Chassis_Side", "Chassis_Side"), ("Chassis_Side", "Chassis_Side001"),
    ("Front_Cover_Optimized", "Front_Cover_Optimized"),
    ("sensormount_esp32_cam", "Sensormount"), ("cameramount_esp32_cam", "Cameramount"),
    ("Top_Cover_Split_Front", "Top_Cover_Split_Front"), ("Top_Cover_Split_Rear", "Top_Cover_Split_Rear"),
    ("Bottom_Cover_Split_Front", "Bottom_Cover_Split_Rear"),
    ("Bottom_Cover_Split_Rear", "Bottom_Cover_Split_Front"),
    ("Rear_Cover_Shell_Medium", "Rear_Cover"),
]
# the assembly only has a flat template of the electronics plate: matched by its outline
PLATE = ("Circuitry_Mountingplate_SZBK07", "Circuitboard Mount (Template)")
# robjk's back plate: placed from robjk's rear assembly, relative to the shell
REAR_PLATE = ("Rear_Cover_Plate_Medium", "Rear_Cover_Plate_medium", "Rear_Cover_Shell_medium")
# not in the assembly (inside the body): mass only, lumped at the body centre
HIDDEN = ["LiPo_Mountingbracket_Front", "LiPo_Mountingbracket_Middle", "LiPo_Mountingbracket_Rear"]
LEGS = {"fl": "FL Leg", "fr": "FR Leg", "rl": "RL Leg", "rr": "RR LEG"}


def T(R=np.eye(3), t=(0, 0, 0)):
    M = np.eye(4); M[:3, :3] = R; M[:3, 3] = t
    return M


def _rot(axis, a):
    """Rotation by `a` about a unit `axis`."""
    x, y, z = axis
    K = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(a) * K + (1 - math.cos(a)) * K @ K


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
        keep = d <= np.percentile(d, 90)  # trim: remixed parts differ from their reference in places
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


class Reference:
    """The assembled solids of `reference/assemblies.zip`."""

    def __init__(self):
        self.zf = zipfile.ZipFile(REF_ZIP)
        self.index = json.loads(self.zf.read("index.json"))

    def find(self, label, source="kubina", within=None):
        hits = [e for e in self.index if e["source"] == source and e["label"] == label
                and (within is None or within in e["path"])]
        assert len(hits) == 1, f"{label!r} in {within!r}: {len(hits)} matches"
        return trimesh.load(io.BytesIO(self.zf.read(hits[0]["file"])), file_type="stl")

    def leg_solids(self, leg, prefix):
        return [(e["path"], trimesh.load(io.BytesIO(self.zf.read(e["file"])), file_type="stl"))
                for e in self.index if LEGS[leg] in e["path"] and e["label"].startswith(prefix)]


# ---------------------------------------------------------------------------------------------
# Skeleton measured in the assembly (mm, assembly frame)
# ---------------------------------------------------------------------------------------------
def _arc(mesh):
    """Centre (x, z) and radius of the foot tip's rolling arc: circle through its lower outline."""
    P = mesh.vertices[:, [0, 2]]
    h = P[ConvexHull(P).vertices]
    h = h[h[:, 1] < mesh.bounds[:, 2].mean()]
    cx, cz, k = np.linalg.lstsq(np.c_[2 * h, np.ones(len(h))], (h ** 2).sum(1), rcond=None)[0]
    return np.array([cx, cz]), math.sqrt(k + cx * cx + cz * cz)


def measure(ref):
    """Per leg: hip axis (y, z), femur and knee axes (x, z), foot arc centre (x, y, z) and radius."""
    legs = {}
    for lg in LEGS:
        gear = {g: m.bounds.mean(0) for g in ("Shoulder Joint Servo", "Upper Leg", "Lower Leg")
                for p, m in ref.leg_solids(lg, "Gear") if any(g in s for s in p)}
        pin = [m.bounds.mean(0) for p, m in ref.leg_solids(lg, "Shoulder_Joint_Ball_Bearing_Pin")
               if any("SHOULDER PIN" in s for s in p)][0]
        (_, foot), = ref.leg_solids(lg, "Foot_Tip")
        centre, radius = _arc(foot)
        legs[lg] = {"hip": (gear["Shoulder Joint Servo"][1:] + pin[1:]) / 2, "femur": gear["Upper Leg"][[0, 2]],
                    "knee": gear["Lower Leg"][[0, 2]], "foot": np.array([centre[0], foot.bounds[:, 1].mean(), centre[1]]),
                    "foot_radius": radius}
    return legs


def skeleton(legs):
    """The firmware_gait skeleton constants (mm), averaged over the four legs."""
    mean = lambda f, lgs=LEGS: float(np.mean([f(legs[lg]) for lg in lgs]))
    side = {"fl": 1, "rl": 1, "fr": -1, "rr": -1}
    return {
        "COXA": float(np.mean([side[lg] * (m["foot"][1] - m["hip"][0]) for lg, m in legs.items()])),
        "COXA_OFFSET": mean(lambda m: m["hip"][1] - m["femur"][1]),
        "FEMUR": mean(lambda m: np.linalg.norm(m["femur"] - m["knee"])),
        "TIBIA": mean(lambda m: np.linalg.norm(m["knee"] - m["foot"][[0, 2]])),
        "FOOT_RADIUS": mean(lambda m: m["foot_radius"]),
        "BODY_L": mean(lambda m: m["femur"][0], ("fl", "fr")) - mean(lambda m: m["femur"][0], ("rl", "rr")),
        "BODY_W": mean(lambda m: m["hip"][0], ("fl", "rl")) - mean(lambda m: m["hip"][0], ("fr", "rr")),
    }


def frames(legs):
    """Sim link frames in the assembly (4x4, mm): base, and the front-left leg's links at the assembly pose."""
    Rb = R_ASM.T
    pitch = lambda a: Rb @ _rot((-1.0, 0.0, 0.0), a)   # femur/tibia hinge: + swings the leg forward
    xs = [m["femur"][0] for m in legs.values()]
    ys = [m["hip"][0] for m in legs.values()]
    base = T(Rb, [(max(xs) + min(xs)) / 2, (max(ys) + min(ys)) / 2, np.mean([m["hip"][1] for m in legs.values()])])
    m = legs["fl"]
    leg_y = m["foot"][1]   # the leg plane
    (fx, fz), (kx, kz), (px, _, pz) = m["femur"], m["knee"], m["foot"]
    # the assembly's legs hang straight down (hip angle 0); femur/tibia angles from the axes
    t2, t23 = math.atan2(kx - fx, fz - kz), math.atan2(px - kx, kz - pz)
    out = {"base": base, "hip": T(Rb, [fx, m["hip"][0], m["hip"][1]]),
           "femur": T(pitch(t2), [fx, leg_y, fz]), "tibia": T(pitch(t23), [kx, leg_y, kz])}
    print(f"assembly pose: femur {math.degrees(t2):.1f} deg, knee {math.degrees(t23 - t2):.1f} deg")
    return out


def in_frame(F, P):
    """Assembly pose P (mm) -> pose in the frame F, metres."""
    M = np.linalg.inv(F) @ P
    M[:3, 3] /= 1000
    return M


def snap_onto(src, M, tgt):
    """`M` with its rotation snapped to the axes, the part's footprint centred on `tgt` and bottoms level."""
    R = min(ROTS, key=lambda Q: np.linalg.norm(Q - M[:3, :3]))
    v = src.vertices @ R.T
    t = tgt.bounds.mean(0) - (v.min(0) + v.max(0)) / 2
    t[2] = tgt.bounds[0, 2] - v[:, 2].min()
    return T(R, t)


def mass_props(mesh, pose, infill):
    """PLA mass (kg), COM and inertia about the COM (kg m^2) in the link frame."""
    V, A = abs(mesh.volume) / 1000, mesh.area / 100                     # cm3, cm2
    shell = min(V, A * SHELL_CM / 2)
    mass = PLA_DENSITY * (shell + infill * (V - shell)) / 1000           # kg
    m = mesh.copy(); m.apply_scale(0.001)
    m.density = mass / abs(m.volume)
    R, t = pose[:3, :3], pose[:3, 3]
    return mass, R @ m.center_mass + t, R @ m.moment_inertia @ R.T


def mirrored(pose):
    """Pose of a mirrored mesh on the mirrored link (left <-> right)."""
    return T(MIRROR_X) @ pose @ T(MIRROR_X)


def main():
    ref = Reference()
    legs = measure(ref)
    sk = skeleton(legs)
    print("skeleton (mm)  measured  firmware_gait")
    for k, v in sk.items():
        print(f"  {k:12s} {v:8.2f}  {getattr(fg, k) * 1000:8.2f}")
    F = frames(legs)
    parts = []   # (mesh name, sim body, pose (m), source mesh, infill)

    # --- front-left leg --------------------------------------------------------------------------
    leg = []
    for stl, link, solid in LEG_PARTS:
        M, rms = fit(load(stl), ref.find(solid, within="FL Leg"))
        print(f"fit {stl:40s} -> {solid:32s} rms {rms:.2f} mm")
        leg.append((stl, link, in_frame(F[link], M)))
    # robjk shoulder: robjk's assembly -> Kubina's, by the limb shell, then turned about the femur axis
    B, rms = fit(ref.find("Limb_Top_Shell", "robjk_limb"), ref.find("Limb_Top_Shell", within="FL Leg"))
    print(f"fit robjk limb shell -> Limb_Top_Shell rms {rms:.2f} mm")
    cap = ref.find("Shoulder_Joint_Servohorn_Cap", "robjk_limb")
    axis_pt = ref.find("Gear003", "robjk_limb").bounds.mean(0)
    arm = B[:3, :3] @ (cap.bounds.mean(0) - axis_pt)
    arm[1] = 0                                   # femur axis = assembly y
    turn = math.atan2(arm[0], arm[2])            # arm -> straight up
    Rt = _rot((0.0, 1.0, 0.0), -turn)
    pivot = B[:3, :3] @ axis_pt + B[:3, 3]
    A = T(Rt, pivot - Rt @ pivot) @ B
    for stl, solid in ROBJK_SHOULDER:
        M, rms = fit(load(stl), ref.find(solid, "robjk_limb"))
        print(f"fit {stl:40s} -> robjk {solid:26s} rms {rms:.2f} mm")
        leg.append((stl, "hip", in_frame(F["hip"], A @ M)))
    for lg in ("fl", "rl", "fr", "rr"):
        for stl, link, pose in leg:
            right = fg.LEG_SIDE[lg] < 0
            name = stl.replace(" ", "")
            parts.append((f"{name}_mirror" if right else name, f"{lg}_{link}", mirrored(pose) if right else pose,
                          stl, INFILL))

    # --- servos: one MG996R mesh, fitted onto the assembly's three --------------------------------
    servo = trimesh.util.concatenate([ref.find(s, within="FL Leg") for s in LEG_SERVOS["hip"]])
    servo.apply_translation(-servo.bounds.mean(0))
    servos = []
    for link, solids in LEG_SERVOS.items():
        M, rms = fit(servo, trimesh.util.concatenate([ref.find(s, within="FL Leg") for s in solids]))
        print(f"fit MG996R -> {solids[0]} rms {rms:.2f} mm")
        servos.append((link, in_frame(F[link], M)))

    # --- body ------------------------------------------------------------------------------------
    body = {}
    for stl, solid in BODY_PARTS:
        M, rms = fit(load(stl), ref.find(solid))
        print(f"fit {stl:40s} -> {solid:32s} rms {rms:.2f} mm")
        body.setdefault(stl, M)
        parts.append((stl, "base", in_frame(F["base"], M), stl, HULL_INFILL if stl.startswith(HULL) else INFILL))
    stl, solid = PLATE
    M, rms = fit(load(stl), ref.find(solid))
    M = snap_onto(load(stl), M, ref.find(solid))
    print(f"fit {stl:40s} -> {solid:32s} rms {rms:.2f} mm (outline only, snapped)")
    parts.append((stl, "base", in_frame(F["base"], M), stl, INFILL))
    plate, plate_solid, shell_solid = REAR_PLATE
    Mp, rms_p = fit(load(plate), ref.find(plate_solid, "robjk_rear"))
    Ms, rms_s = fit(load("Rear_Cover_Shell_Medium"), ref.find(shell_solid, "robjk_rear"))
    print(f"fit {plate} via robjk rear assembly rms {rms_p:.2f} / {rms_s:.2f} mm")
    M = body["Rear_Cover_Shell_Medium"] @ np.linalg.inv(Ms) @ Mp
    parts.append((plate, "base", in_frame(F["base"], M), plate, HULL_INFILL))

    # --- mass properties + meshes ------------------------------------------------------------------
    out = {"parts": [], "servos": [], "hidden": [], "base_box": None}
    meshes = {}
    for name, link, pose, stl, infill in parts:
        mesh = load(stl)
        if name.endswith("_mirror"):
            mesh.apply_transform(T(MIRROR_X))   # trimesh keeps the winding consistent
        meshes[name] = mesh
        m, com, I = mass_props(mesh, pose, infill)
        out["parts"].append({"mesh": name, "body": link, "pose": pose.tolist(), "mass": m, "com": com.tolist(),
                             "inertia": I.tolist()})
    meshes["MG996R"], meshes["MG996R_mirror"] = servo, servo.copy().apply_transform(T(MIRROR_X))
    ext = servo.extents / 1000
    box = np.diag([ext[1] ** 2 + ext[2] ** 2, ext[0] ** 2 + ext[2] ** 2, ext[0] ** 2 + ext[1] ** 2]) / 12
    for lg in ("fl", "rl", "fr", "rr"):
        for link, pose in servos:
            right = fg.LEG_SIDE[lg] < 0
            pose = mirrored(pose) if right else pose
            R = pose[:3, :3]
            out["servos"].append({"mesh": "MG996R_mirror" if right else "MG996R", "body": f"{lg}_{link}",
                                  "pose": pose.tolist(), "com": pose[:3, 3].tolist(),
                                  "inertia_per_kg": (R @ box @ R.T).tolist()})
    for h in HIDDEN:
        out["hidden"].append({"mesh": h, "body": "base", "mass": mass_props(load(h), np.eye(4), INFILL)[0]})
    # body collision box: bounds of the placed hull parts in the base frame
    hull = [p for p in out["parts"] if p["body"] == "base" and p["mesh"].startswith(HULL)]
    pts = np.vstack([trimesh.transformations.transform_points(meshes[p["mesh"]].vertices / 1000, p["pose"])
                     for p in hull])
    lo, hi = pts.min(0), pts.max(0)
    out["base_box"] = {"center": ((lo + hi) / 2).tolist(), "half": ((hi - lo) / 2).tolist()}

    mesh_dir = os.path.join(HERE, "meshes")
    for f in os.listdir(mesh_dir):
        os.remove(os.path.join(mesh_dir, f))
    for name, mesh in meshes.items():
        mesh.export(os.path.join(mesh_dir, f"{name}.stl"))
    with open(os.path.join(HERE, "print_parts.json"), "w") as f:
        json.dump(out, f, indent=1)
    total = sum(p["mass"] for p in out["parts"]) + sum(h["mass"] for h in out["hidden"])
    print(f"wrote print_parts.json ({len(out['parts'])} placed parts, printed mass {total * 1000:.0f} g)")


if __name__ == "__main__":
    main()
