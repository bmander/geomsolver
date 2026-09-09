"""Fit solved generated envelopes to CAD faces and test a closed STEP tooth space.

This is an isolated backend experiment, not a public gear constructor. The supplied
sections are local envelope witnesses, not proof of globally exposed swept material.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import time

from OCP.Approx import Approx_IsoParametric
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
from OCP.BRepBuilderAPI import (BRepBuilderAPI_MakeFace, BRepBuilderAPI_Sewing,
                               BRepBuilderAPI_MakeSolid, BRepBuilderAPI_MakeEdge, BRepBuilderAPI_MakeWire)
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.BRepGProp import BRepGProp
from OCP.BRepLib import BRepLib
from OCP.BRepPrimAPI import BRepPrimAPI_MakeRevol
from OCP.GC import GC_MakeArcOfCircle
from OCP.GeomAPI import GeomAPI_PointsToBSplineSurface
from OCP.GProp import GProp_GProps
from OCP.gp import gp_Pnt, gp_Ax1, gp_Dir
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPControl import STEPControl_Writer, STEPControl_Reader, STEPControl_AsIs
from OCP.TColgp import TColgp_Array2OfPnt
from OCP.TopAbs import TopAbs_SHELL, TopAbs_SOLID, TopAbs_FACE
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS


def count(shape, kind):
    explorer, total = TopExp_Explorer(shape, kind), 0
    while explorer.More():
        total += 1
        explorer.Next()
    return total


def volume(shape):
    props = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape, props, Eps=1e-10, OnlyClosed=True)
    return props.Mass()


def surface(grid):
    points = TColgp_Array2OfPnt(1, len(grid), 1, len(grid[0]))
    for i, row in enumerate(grid, 1):
        for j, p in enumerate(row, 1):
            points.SetValue(i, j, gp_Pnt(*p))
    fit = GeomAPI_PointsToBSplineSurface()
    # Uniform parameters make independently fitted shared boundaries agree.
    fit.Interpolate(points, Approx_IsoParametric, False)
    if not fit.IsDone():
        raise RuntimeError("surface interpolation failed")
    return fit.Surface()


def between(a, b, f):
    rho = math.sqrt(sum(x*x for x in a))
    pa, pb = (math.atan2(math.hypot(p[0], p[1]), p[2]) for p in (a, b))
    aa, ab = (math.atan2(p[1], p[0]) for p in (a, b))
    span = math.remainder(ab-aa, math.tau)
    angle, polar = aa+f*span, pa+f*(pb-pa)
    return [rho*math.sin(polar)*math.cos(angle), rho*math.sin(polar)*math.sin(angle),
            rho*math.cos(polar)]


def sides_for_space(member, level):
    sides = level["sides"]
    if member["member"] == 1:
        angle = math.tau/member["teeth"]
        c, s = math.cos(angle), math.sin(angle)
        sides = [sides[0], [[[c*x-s*y, s*x+c*y, z] for x, y, z in row]
                              for row in sides[1]]]
    return sides


def blank(member):
    (a, b), (c, d) = member["blank_meridian"]
    wire = BRepBuilderAPI_MakeWire()
    for p, q, circular in [(a, b, True), (b, d, False), (d, c, True), (c, a, False)]:
        if circular:
            curve = GC_MakeArcOfCircle(gp_Pnt(*p), gp_Pnt(*between(p, q, .5)), gp_Pnt(*q)).Value()
            edge = BRepBuilderAPI_MakeEdge(curve).Edge()
        else:
            edge = BRepBuilderAPI_MakeEdge(gp_Pnt(*p), gp_Pnt(*q)).Edge()
        wire.Add(edge)
    profile = BRepBuilderAPI_MakeFace(wire.Wire()).Face()
    body = BRepPrimAPI_MakeRevol(profile, gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1))).Shape()
    if not BRepCheck_Analyzer(body).IsValid() or volume(body) <= 0:
        raise RuntimeError("invalid blank from source meridian")
    return body


def roundtrip(shape, path):
    writer = STEPControl_Writer()
    if writer.Transfer(shape, STEPControl_AsIs) != IFSelect_RetDone or writer.Write(str(path)) != IFSelect_RetDone:
        raise RuntimeError("STEP export failed")
    reader = STEPControl_Reader()
    if reader.ReadFile(str(path)) != IFSelect_RetDone or reader.TransferRoots() <= 0:
        raise RuntimeError("STEP reimport failed")
    restored = reader.OneShape()
    return dict(step_file=str(path), step_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                roundtrip_valid=BRepCheck_Analyzer(restored).IsValid(),
                roundtrip_solids=count(restored, TopAbs_SOLID), roundtrip_volume=volume(restored))


def build(member, level, fine, tolerance):
    started = time.perf_counter()
    n = level["subdivisions"]
    sides = sides_for_space(member, level)
    surfaces, support = [], []
    for side in sides:
        for start in (0, n):
            patch = surface([row[start:start+n+1] for row in side])
            surfaces.append(patch)
            support.append(patch)
    for end in (0, -1):
        surfaces.append(surface([[between(a[end], b[end], j/n) for j in range(n+1)]
                                  for a, b in zip(*sides)]))
    for row in (0, -1):
        for start in (0, n):
            surfaces.append(surface([[between(sides[0][row][i], sides[1][row][i], j/n)
                                      for j in range(n+1)] for i in range(start, start+n+1)]))
    sewing = BRepBuilderAPI_Sewing(tolerance)
    for patch in surfaces:
        sewing.Add(BRepBuilderAPI_MakeFace(patch, tolerance).Face())
    sewing.Perform()
    shell = sewing.SewedShape()
    result = dict(faces=count(shell, TopAbs_FACE), shells=count(shell, TopAbs_SHELL),
                  free_edges=sewing.NbFreeEdges(), multiple_edges=sewing.NbMultipleEdges())
    if shell.ShapeType() != TopAbs_SHELL or result["free_edges"] or result["multiple_edges"]:
        raise RuntimeError(f"sewing did not produce one closed shell: {result}")
    solid = BRepBuilderAPI_MakeSolid(TopoDS.Shell_s(shell)).Solid()
    result["oriented"] = BRepLib.OrientClosedSolid_s(solid)
    result["valid"] = BRepCheck_Analyzer(solid).IsValid()
    result["volume"] = volume(solid)
    result["construction_seconds"] = time.perf_counter()-started
    fine_sides = sides_for_space(member, fine)
    fn = fine["subdivisions"]
    error = 0
    for si, side in enumerate(fine_sides):
        for half in range(2):
            patch = support[2*si+half]
            u0, u1, v0, v1 = patch.Bounds()
            for i, row in enumerate(side):
                for j in range(fn+1):
                    p = patch.Value(u0+(u1-u0)*i/fn, v0+(v1-v0)*j/fn)
                    error = max(error, p.Distance(gp_Pnt(*row[half*fn+j])))
    result["maximum_envelope_sample_error"] = error
    return solid, result


def run(source, directory, tolerance=1e-6, subdivisions=None):
    data = json.loads(source.read_text())
    directory.mkdir(parents=True, exist_ok=True)
    results = []
    for member in data["members"]:
        levels = [(level, fine) for level, fine in zip(member["levels"], member["levels"][1:])
                  if subdivisions is None or level["subdivisions"] == subdivisions]
        if not levels:
            raise ValueError("requested fitting grid or finer comparison grid missing")
        for level, fine in levels:
            row = dict(member=member["member"], subdivisions=level["subdivisions"], failures=[])
            try:
                solid, info = build(member, level, fine, tolerance)
                row.update(info)
                if not info["valid"] or not info["oriented"] or info["volume"] <= 0:
                    raise RuntimeError("invalid or non-positive candidate solid")
                path = directory/f"member{member['member']}-space-{level['subdivisions']}.step"
                row.update(roundtrip(solid, path))
                body = blank(member)
                started = time.perf_counter()
                operation = BRepAlgoAPI_Cut(body, solid)
                if not operation.IsDone():
                    raise RuntimeError("tooth-space subtraction failed")
                cut = operation.Shape()
                row["cut"] = dict(valid=BRepCheck_Analyzer(cut).IsValid(),
                                  solids=count(cut, TopAbs_SOLID), volume=volume(cut),
                                  blank_volume=volume(body), seconds=time.perf_counter()-started)
                cut_path = directory/f"member{member['member']}-one-cut-{level['subdivisions']}.step"
                row["cut"].update(roundtrip(cut, cut_path))
                for label, info in [("space", row), ("cut", row["cut"])]:
                    if not info["valid"] or info.get("solids", 1) != 1:
                        row["failures"].append(f"{label}: invalid or multiple native solids")
                    if not info["roundtrip_valid"] or info["roundtrip_solids"] != 1:
                        row["failures"].append(f"{label}: invalid or multiple STEP solids")
                    if abs(info["volume"]-info["roundtrip_volume"]) > 1e-8*abs(info["volume"]):
                        row["failures"].append(f"{label}: STEP volume mismatch")
                if row["maximum_envelope_sample_error"] > .001:
                    row["failures"].append("sampled envelope error exceeds 0.001 mm")
                if not 0 < row["cut"]["volume"] < row["cut"]["blank_volume"]:
                    row["failures"].append("cut did not remove a positive volume")
            except Exception as error:
                row["failures"].append(str(error))
            row["passes_local_checks"] = not row["failures"]
            results.append(row)
            print(json.dumps(row), flush=True)
    report = dict(source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                  source_scope=data["scope"], sewing_tolerance=tolerance, results=results,
                  volume_integration_relative_target=1e-10,
                  scope="CAD topology and sampled local-envelope fidelity; global material and pair validation pending")
    (directory/"report.json").write_text(json.dumps(report, indent=2)+"\n")
    return bool(results) and all(row["passes_local_checks"] for row in results)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--subdivisions", type=int)
    args = parser.parse_args()
    raise SystemExit(0 if run(args.source, args.output, subdivisions=args.subdivisions) else 1)
