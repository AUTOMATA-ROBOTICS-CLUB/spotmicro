"""Extract the assembled reference models that `fit_print_meshes.py` fits the print parts onto.

Sources (large, not in the repo):
- Michael Kubina's assembled SpotMicroESP32 (GPL-3.0),
  https://github.com/michaelkubina/SpotMicroESP32 -> assembly/spotmicroesp32_assembled_quick_and_dirty.FCStd
- robjk's FreeCAD files from https://www.thingiverse.com/thing:4937631 (CC BY): Limb_Top_Shell_mod.FCStd
  (shoulder cover + extended servo cap + limb top shell + femur servo, assembled) and
  Rear_Cover_Shell_Medium.FCStd (medium rear cover shell + back plate).

Every printed part, servo and joint feature (servo gears, shoulder pins, foot tips) is tessellated in
the assembly's global frame (mm; Kubina: +x forward, +y left, +z up), decimated and written to
`reference/assemblies.zip` with an `index.json` (source, label, container path). Fasteners and
electronics other than the servos are dropped.

    uv run --with cadquery-ocp --with trimesh --with fast-simplification \
        python -m src.resources.spotmicro_esp32.extract_assembly KUBINA.FCStd LIMB_TOP_SHELL_MOD.FCStd \
        REAR_COVER_SHELL_MEDIUM.FCStd
"""

import json
import os
import re
import sys
import tempfile
import zipfile
import xml.etree.ElementTree as ET

import numpy as np
import trimesh
from OCP.BRep import BRep_Builder, BRep_Tool
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.BRepTools import BRepTools
from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED
from OCP.TopExp import TopExp_Explorer
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS, TopoDS_Shape

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "reference", "assemblies.zip")
SOLID_TYPES = ("PartDesign::Body", "Part::Mirroring", "Part::Feature")
# fasteners, bearings, servo horns and electronics (by label, then by container)
DROP = re.compile(r"Nut|M[23]x|Screw|Dampener|wire|Pins|Futuba|Array|Kamera|Clock|^Body\d")
DROP_PATH = re.compile(r"HC-SR04|OV7670|TFT|Push Button|Pin Header|White LEDs")
# legs other than front-left only contribute their joint features
OTHER_LEGS = ("FR Leg", "RL Leg", "RR LEG")
FEATURES = ("Gear", "Shoulder_Joint_Ball_Bearing_Pin", "Foot_Tip")
MAX_FACES = 2500


def _placement(props):
    """FreeCAD Placement property -> 4x4."""
    M = np.eye(4)
    p = props.get("Placement")
    if p is None:
        return M
    a = {k: float(v) for k, v in p.find("PropertyPlacement").attrib.items()}
    x, y, z, w = a["Q0"], a["Q1"], a["Q2"], a["Q3"]
    M[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                 [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                 [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    M[:3, 3] = a["Px"], a["Py"], a["Pz"]
    return M


def _tessellate(shape):
    BRepMesh_IncrementalMesh(shape, 0.2, False, 0.3, True)
    V, F = [], []
    ex = TopExp_Explorer(shape, TopAbs_FACE)
    while ex.More():
        face = TopoDS.Face(ex.Current())
        loc = TopLoc_Location()
        tri = BRep_Tool.Triangulation_s(face, loc)
        if tri is not None:
            tr, base = loc.Transformation(), len(V)
            V += [(p.X(), p.Y(), p.Z()) for p in (tri.Node(i).Transformed(tr) for i in range(1, tri.NbNodes() + 1))]
            rev = face.Orientation() == TopAbs_REVERSED
            for i in range(1, tri.NbTriangles() + 1):
                a, b, c = (j + base - 1 for j in tri.Triangle(i).Get())
                F.append((a, c, b) if rev else (a, b, c))
        ex.Next()
    return trimesh.Trimesh(np.array(V, float).reshape(-1, 3), np.array(F, int).reshape(-1, 3))


def solids(fcstd, all_visible):
    """Yield (label, container path, mesh in the global frame) for the solids of a .FCStd file."""
    zf = zipfile.ZipFile(fcstd)
    root = ET.fromstring(zf.read("Document.xml"))
    types = {o.get("name"): o.get("type") for o in root.find("Objects")}
    props = {od.get("name"): {p.get("name"): p for p in od.find("Properties")} for od in root.find("ObjectData")}
    label = lambda n: props[n]["Label"].find("String").get("value")
    visible = lambda n: "Visibility" not in props[n] or props[n]["Visibility"].find("Bool").get("value") == "true"
    parent = {c.get("value"): n for n, t in types.items() if t == "App::Part"
              for c in props[n]["Group"].iter("Link") if c.get("value")}
    for n, t in types.items():
        if t not in SOLID_TYPES or DROP.search(label(n)):
            continue
        # global pose = the containers' placements; the stored shape already carries the solid's own
        G, path, p, shown = np.eye(4), [], parent.get(n), visible(n)
        while p is not None:
            G, shown = _placement(props[p]) @ G, shown and visible(p)
            path.insert(0, label(p))
            p = parent.get(p)
        if not (shown or all_visible) or any(DROP_PATH.search(s) for s in path):
            continue
        if any(leg in path for leg in OTHER_LEGS) and not label(n).startswith(FEATURES):
            continue
        part = props[n].get("Shape")
        part = part.find("Part") if part is not None else None
        if part is None or not part.get("file"):
            continue
        shape = TopoDS_Shape()
        with tempfile.TemporaryDirectory() as tmp:
            brp = zf.extract(part.get("file"), tmp)
            BRepTools.Read_s(shape, brp, BRep_Builder())
        if shape.IsNull():
            continue
        m = _tessellate(shape)
        if not len(m.faces):
            continue
        m.apply_transform(G)
        yield label(n), path, m


def main():
    kubina, robjk_limb, robjk_rear = sys.argv[1:4]
    index, files = [], {}
    sources = (("kubina", kubina, False), ("robjk_limb", robjk_limb, True), ("robjk_rear", robjk_rear, True))
    for source, path, all_visible in sources:
        for label, cpath, m in solids(path, all_visible):
            if len(m.faces) > MAX_FACES:
                m = m.simplify_quadric_decimation(face_count=MAX_FACES)
            name = f"{source}/{len(index):03d}.stl"
            files[name] = m.export(file_type="stl")
            index.append({"source": source, "label": label, "path": cpath, "file": name})
        print(f"{source}: {sum(e['source'] == source for e in index)} solids")
    with zipfile.ZipFile(OUT, "w", zipfile.ZIP_DEFLATED) as zf:
        zf.writestr("index.json", json.dumps(index, indent=1))
        for name, data in files.items():
            zf.writestr(name, data)
    print(f"wrote {OUT} ({os.path.getsize(OUT) / 1e6:.1f} MB)")


if __name__ == "__main__":
    main()
