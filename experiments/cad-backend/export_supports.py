"""Extract actual STEP closure B-splines and their solved nominal supports."""
import argparse
import hashlib
import json
import math
from pathlib import Path

from OCP.BRep import BRep_Tool
from OCP.TopAbs import TopAbs_FACE
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS

from material_probes import read
from tooth_space import between, sides_for_space


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def extract(surface):
    if surface.IsUPeriodic() or surface.IsVPeriodic():
        raise ValueError("periodic closure not supported")
    nu, nv = surface.NbUPoles(), surface.NbVPoles()
    poles = [[[surface.Pole(i, j).Coord(k) for k in (1, 2, 3)]
              for j in range(1, nv+1)] for i in range(1, nu+1)]
    weights = [[surface.Weight(i, j) for j in range(1, nv+1)] for i in range(1, nu+1)]
    knots = [[surface.UKnot(i) for i in range(1, surface.NbUKnots()+1)
              for _ in range(surface.UMultiplicity(i))],
             [surface.VKnot(i) for i in range(1, surface.NbVKnots()+1)
              for _ in range(surface.VMultiplicity(i))]]
    return dict(degrees=[surface.UDegree(), surface.VDegree()], poles=poles, weights=weights, knots=knots)


def run(source, supports, spaces, output):
    data, nominal = [json.loads(p.read_text()) for p in (source, supports)]
    report = json.loads((spaces/"report.json").read_text())
    if report["source_sha256"] != digest(source) or nominal["module_mm"] != data["module_mm"]:
        raise ValueError("CAD candidates and supports use different sources")
    records, provenance = [], []
    for member, support in zip(data["members"], nominal["members"]):
        m = member["member"]
        if any(member[k] != support[k] for k in ("member", "teeth", "blank_meridian")):
            raise ValueError("solved supports disagree with original CAD source")
        level = next(v for v in member["levels"] if v["subdivisions"] == 16)
        sides = sides_for_space(member, level)
        expected = [("root", between(sides[0][8][0], sides[1][8][0], .5),
                     dict(kind="cone", meridian=support["cones_tip_root_back"][1])),
                    ("tip", between(sides[0][8][-1], sides[1][8][-1], .5),
                     dict(kind="cone", meridian=support["cones_tip_root_back"][0]))]
        for name, row, radius in zip(("toe", "heel"), (0, -1), nominal["sphere_radii_mm"]):
            for part, i in (("round", 8), ("flank", 24)):
                expected.append((f"{name}_{part}", between(sides[0][row][i], sides[1][row][i], .5),
                                 dict(kind="sphere", radius_mm=radius)))
        path = spaces/f"member{m}-space-16.step"
        checked = next(r for r in report["results"] if r["member"] == m and r["subdivisions"] == 16)
        if not checked["passes_local_checks"] or checked["step_sha256"] != digest(path):
            raise ValueError("unchecked or changed tooth-space STEP")
        shape = read(path)
        faces, surfaces = TopExp_Explorer(shape, TopAbs_FACE), []
        while faces.More():
            surfaces.append(BRep_Tool.Surface_s(TopoDS.Face_s(faces.Current())))
            faces.Next()
        if len(surfaces) != 10:
            raise ValueError("expected ten tooth-space faces")
        selected = set()
        for name, point, target in expected:
            distances = []
            for s in surfaces:
                u0, u1, v0, v1 = s.Bounds()
                p = s.Value((u0+u1)/2, (v0+v1)/2)
                distances.append(math.dist(point, [p.X(), p.Y(), p.Z()]))
            index = min(range(len(surfaces)), key=distances.__getitem__)
            if distances[index] > 1e-7 or index in selected:
                raise ValueError("closure face matching failed or ambiguous")
            selected.add(index)
            records.append(dict(member=m, role=name, face_index=index, support=target,
                                match_error_mm=distances[index], surface=extract(surfaces[index])))
        provenance.append(dict(member=m, step_file=str(path), step_sha256=digest(path)))
    output.write_text(json.dumps(dict(inputs=provenance, source_file=str(source), source_sha256=digest(source),
        supports_file=str(supports), supports_sha256=digest(supports), nominal_supports=nominal, surfaces=records,
        scope="Binary64 B-spline coefficients read from actual STEP closures; no bounds checked yet"))+"\n")
    print(f"Extracted {len(records)} complete closure support surfaces", flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("source", "supports", "spaces", "output"):
        parser.add_argument(name, type=Path)
    args = parser.parse_args()
    run(args.source, args.supports, args.spaces, args.output)
