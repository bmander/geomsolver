"""Create off-surface material probes from the actual STEP tooth-space export."""
import argparse
import hashlib
import json
from pathlib import Path

from OCP.BRep import BRep_Tool
from OCP.BRepClass3d import BRepClass3d_SolidClassifier
from OCP.BRepTools import BRepTools
from OCP.gp import gp_Pnt, gp_Vec
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPControl import STEPControl_Reader
from OCP.TopAbs import TopAbs_FACE, TopAbs_IN, TopAbs_OUT, TopAbs_SOLID
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS


def read(path):
    reader = STEPControl_Reader()
    if reader.ReadFile(str(path)) != IFSelect_RetDone or reader.TransferRoots() != 1:
        raise RuntimeError(f"unreadable single STEP root: {path}")
    explorer = TopExp_Explorer(reader.OneShape(), TopAbs_SOLID)
    shape = TopoDS.Solid_s(explorer.Current())
    explorer.Next()
    if explorer.More():
        raise RuntimeError("expected one solid")
    return shape


def generate(directory, output, displacement=.01):
    lines, provenance = [], []
    for member in range(2):
        space_path = directory/f"member{member}-space-16.step"
        cut_path = directory/f"member{member}-one-cut-16.step"
        space, cut = read(space_path), read(cut_path)
        classifier = BRepClass3d_SolidClassifier(cut)
        faces, index = TopExp_Explorer(space, TopAbs_FACE), 0
        while faces.More():
            face = TopoDS.Face_s(faces.Current())
            u0, u1, v0, v1 = BRepTools.UVBounds_s(face)
            surface = BRep_Tool.Surface_s(face)
            p, du, dv = gp_Pnt(), gp_Vec(), gp_Vec()
            surface.D1((u0+u1)/2, (v0+v1)/2, p, du, dv)
            normal = du.Crossed(dv).Normalized()
            for sign in (-1, 1):
                q = p.Translated(normal.Multiplied(sign*displacement))
                classifier.Perform(q, 1e-7)
                state = classifier.State()
                if state not in (TopAbs_IN, TopAbs_OUT):
                    raise RuntimeError("ambiguous CAD material probe")
                inside = state == TopAbs_IN
                label = f"face{index}_offset{sign}"
                lines.append(f"{member} {label} {int(inside)} {q.X():.17g} {q.Y():.17g} {q.Z():.17g}")
            # Tool closures at tip/toe/heel need not be exposed boundaries of the
            # remaining blank: both offsets can correctly be outside its material.
            index += 1
            faces.Next()
        provenance.append(dict(member=member, space_sha256=hashlib.sha256(space_path.read_bytes()).hexdigest(),
                               cut_sha256=hashlib.sha256(cut_path.read_bytes()).hexdigest(), faces=index))
    output.write_text("\n".join(lines)+"\n")
    output.with_suffix(".json").write_text(json.dumps(dict(provenance=provenance,
        displacement_mm=displacement, probe_sha256=hashlib.sha256(output.read_bytes()).hexdigest()),indent=2)+"\n")
    print(f"Wrote {len(lines)} CAD-classified material probes", flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    generate(args.directory, args.output)
