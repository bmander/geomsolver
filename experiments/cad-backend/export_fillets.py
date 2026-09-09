"""Bind actual STEP fillet coefficients to the bounded common-crown reference."""
import argparse
import json
import math
from pathlib import Path

from OCP.BRep import BRep_Tool
from OCP.TopAbs import TopAbs_FACE
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS

from export_supports import digest, extract
from material_probes import read
from tooth_space import sides_for_space


def run(source, references, spaces, output):
    data = json.loads(source.read_text())
    reference = json.loads(references.read_text())
    report = json.loads((spaces/"report.json").read_text())
    if (report["source_sha256"] != digest(source) or reference["module_mm"] != data["module_mm"]
            or reference["teeth"] != [m["teeth"] for m in data["members"]]):
        raise ValueError("source and reference disagree")
    records, inputs = [], []
    for member in data["members"]:
        m = member["member"]
        path = spaces/f"member{m}-space-16.step"
        checked = next(r for r in report["results"] if r["member"] == m and r["subdivisions"] == 16)
        if not checked["passes_local_checks"] or checked["step_sha256"] != digest(path):
            raise ValueError("changed or unchecked STEP input")
        sides = sides_for_space(member, next(l for l in member["levels"] if l["subdivisions"] == 16))
        faces = TopExp_Explorer(read(path), TopAbs_FACE)
        surfaces = []
        while faces.More():
            surfaces.append(BRep_Tool.Surface_s(TopoDS.Face_s(faces.Current())))
            faces.Next()
        if len(surfaces) != 10:
            raise ValueError("expected ten tooth-space faces")
        selected = set()
        for side in range(2):
            point = sides[side][8][8]
            distances = []
            for s in surfaces:
                u0, u1, v0, v1 = s.Bounds()
                p = s.Value((u0+u1)/2, (v0+v1)/2)
                distances.append(math.dist(point, [p.X(), p.Y(), p.Z()]))
            index = min(range(len(surfaces)), key=distances.__getitem__)
            if index in selected or distances[index] > 1e-7:
                raise ValueError("ambiguous or unmatched fillet")
            selected.add(index)
            records.append(dict(member=m, side=side, face_index=index,
                                surface=extract(surfaces[index]), match_error_mm=distances[index]))
        inputs.append(dict(member=m, step_file=str(path), step_sha256=digest(path)))
    output.write_text(json.dumps(dict(inputs=inputs, source_file=str(source), source_sha256=digest(source),
        references_file=str(references), references_sha256=digest(references), references=reference,
        surfaces=records, scope="Actual STEP fillet coefficients; whole-patch fidelity not checked"))+"\n")
    print(f"Extracted {len(records)} fillet surfaces", flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("source", "references", "spaces", "output"):
        parser.add_argument(name, type=Path)
    args = parser.parse_args()
    run(args.source, args.references, args.spaces, args.output)
