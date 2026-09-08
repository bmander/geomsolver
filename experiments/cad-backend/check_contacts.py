"""Compare independent nominal contact positions with both exported STEP boundaries."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import time

from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeVertex
from OCP.BRepClass3d import BRepClass3d, BRepClass3d_SolidClassifier
from OCP.BRepExtrema import BRepExtrema_DistShapeShape
from OCP.gp import gp_Pnt, gp_Vec
from OCP.TopAbs import TopAbs_IN, TopAbs_OUT

from assembly import load_members, pose


def xyz(value):
    return [value.X(), value.Y(), value.Z()]


def run(source, reports, contacts_path, output, gear_offset=0.):
    if not math.isfinite(gear_offset):
        raise ValueError("expected finite gear offset")
    teeth, shapes, provenance, source_hash = load_members(source, reports)
    source_data = json.loads(source.read_text())
    data = json.loads(contacts_path.read_text())
    if (data["teeth"] != teeth or data["module_mm"] != source_data["module_mm"]
            or abs(data["mean_distance_mm"]-source_data["mean_distance"]) > 1e-10):
        raise ValueError("contact reference and CAD source disagree")
    contacts = data["contacts"]
    keys = {(c["side"], c["face_fraction"], c["sample"]) for c in contacts}
    expected = {(s, f, j) for s in range(2) for f in (.9, .95, 1., 1.05, 1.1) for j in range(25)}
    if keys != expected or len(contacts) != len(expected):
        raise ValueError("incomplete or duplicate contact grid")
    shells = [BRepClass3d.OuterShell_s(s) for s in shapes]
    classifiers = [BRepClass3d_SolidClassifier(s) for s in shapes]
    tolerance, displacement = .001, .001
    result = dict(inputs=provenance, source_sha256=source_hash,
                  contacts_sha256=hashlib.sha256(contacts_path.read_bytes()).hexdigest(),
                  gear_phase_offset_rad=gear_offset, distance_target_mm=tolerance,
                  offset_mm=displacement, max_distance_mm=[0., 0.], max_frame_error=0.,
                  samples=[], failures=[],
                  scope="Analytical contact positions and offset signs at listed samples; not a whole-surface or continuous-roll CAD certificate")
    started = time.perf_counter()
    for index, c in enumerate(contacts):
        probe = .9 < c["face_fraction"] < 1.1 and 0 < c["sample"] < 24
        if c["offset_probe"] != probe:
            raise ValueError("contact evidence changed the offset-probe coverage")
        world = gp_Pnt(*c["world_position_mm"])
        normal = gp_Vec(*c["world_normal"])
        distances, states = [], []
        for member in range(2):
            nominal = pose(teeth, member, c["tooth_fraction"])
            point = gp_Pnt(*c["local_positions_mm"][member]).Transformed(nominal)
            n = gp_Vec(*c["local_normals"][member]).Transformed(nominal)
            frame_error = max(point.Distance(world), math.dist(xyz(n),
                [v*(1 if member == 0 else -1) for v in xyz(normal)]))
            result["max_frame_error"] = max(result["max_frame_error"], frame_error)
            if not math.isfinite(frame_error) or frame_error > 1e-8:
                raise ValueError("analytical contact disagrees with independent assembly pose")
            inverse = pose(teeth, member, c["tooth_fraction"], gear_offset).Inverted()
            local = world.Transformed(inverse)
            vertex = BRepBuilderAPI_MakeVertex(local).Vertex()
            # Measure distance to the shell, so an interior point cannot report
            # zero merely because it is contained in a solid.
            query = BRepExtrema_DistShapeShape(vertex, shells[member], 1e-9)
            if not query.IsDone() or query.NbSolution() == 0:
                raise RuntimeError("CAD boundary distance unresolved")
            distance = query.Value()
            if not math.isfinite(distance) or distance < 0:
                raise RuntimeError("invalid CAD boundary distance")
            distances.append(distance)
            result["max_distance_mm"][member] = max(result["max_distance_mm"][member], distance)
            signs = []
            if c["offset_probe"]:
                for sign in (-1, 1):
                    q = world.Translated(normal.Multiplied(sign*displacement)).Transformed(inverse)
                    classifiers[member].Perform(q, 1e-7)
                    state = classifiers[member].State()
                    signs.append("in" if state == TopAbs_IN else "out" if state == TopAbs_OUT else "ambiguous")
            states.append(signs)
        row = dict(side=c["side"], face_fraction=c["face_fraction"], sample=c["sample"],
                   tooth_fraction=c["tooth_fraction"], distances_mm=distances, offset_states=states)
        if max(distances) > tolerance:
            result["failures"].append(dict(index=index, reason="contact misses CAD boundary", **row))
        if c["offset_probe"] and (states[0] not in (["in", "out"], ["out", "in"])
                or states[1] != states[0][::-1]):
            result["failures"].append(dict(index=index, reason="material sides are not complementary", **row))
        result["samples"].append(row)
        if (index+1) % 25 == 0:
            print(f"Contacts {index+1}/{len(contacts)}; max distances {result['max_distance_mm']}; failures {len(result['failures'])}", flush=True)
            output.write_text(json.dumps(result, indent=2)+"\n")
    result.update(seconds=time.perf_counter()-started, passes_sampled_checks=not result["failures"])
    output.write_text(json.dumps(result, indent=2)+"\n")
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("pinion_report", type=Path)
    parser.add_argument("gear_report", type=Path)
    parser.add_argument("contacts", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--gear-offset", type=float, default=0.)
    args = parser.parse_args()
    result = run(args.source, [args.pinion_report, args.gear_report], args.contacts,
                 args.output, args.gear_offset)
    raise SystemExit(0 if result["passes_sampled_checks"] else 1)
