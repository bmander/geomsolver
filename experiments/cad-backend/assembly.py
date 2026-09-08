"""Measure interference of actual STEP members in the documented assembly poses."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import time

from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.gp import gp_Ax1, gp_Dir, gp_Pnt, gp_Trsf
from OCP.TopAbs import TopAbs_SOLID

from material_probes import read
from tooth_space import count, volume


def rotation(axis, angle):
    transform = gp_Trsf()
    transform.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(*axis)), angle)
    return transform


def pose(teeth, member, fraction, gear_offset=0.):
    """One tooth period: +one pinion pitch and -one gear pitch, then shaft tilt."""
    sign = 1 if member == 0 else -1
    delta = math.atan2(teeth[member], teeth[1-member])
    spin = sign*math.tau*fraction/teeth[member] + (gear_offset if member == 1 else 0.)
    tilt = rotation((0, 1, 0), math.pi/2-sign*delta)
    return tilt.Multiplied(rotation((0, 0, 1), spin))


def load_members(source, reports):
    source_hash = hashlib.sha256(source.read_bytes()).hexdigest()
    data = json.loads(source.read_text())
    if [m["member"] for m in data["members"]] != [0, 1]:
        raise ValueError("expected ordered pinion and gear definitions")
    teeth = [int(m["teeth"]) for m in data["members"]]
    if any(t <= 0 or t != m["teeth"] for t, m in zip(teeth, data["members"])):
        raise ValueError("tooth counts must be positive integers")
    shapes, provenance = [], []
    for member, path in enumerate(reports):
        report = json.loads(path.read_text())
        if report["source_sha256"] != source_hash:
            raise ValueError("candidate report uses a different source")
        rows = [r for r in report["results"] if r["member"] == member]
        if len(rows) != 1 or not rows[0]["passes_local_checks"] or rows[0]["cuts"] != teeth[member]:
            raise ValueError("expected an accepted complete indexed member")
        row = rows[0]
        step = Path(row["step_file"])
        digest = hashlib.sha256(step.read_bytes()).hexdigest()
        if digest != row["step_sha256"]:
            raise ValueError("STEP changed since candidate check")
        shape = read(step)
        if not BRepCheck_Analyzer(shape).IsValid():
            raise ValueError("invalid STEP input")
        shapes.append(shape)
        provenance.append(dict(member=member, step_file=str(step), step_sha256=digest))
    return teeth, shapes, provenance, source_hash


def run(source, reports, output, fractions, gear_offset=0.):
    if not fractions or not all(math.isfinite(x) for x in [*fractions, gear_offset]):
        raise ValueError("expected finite, nonempty phase samples")
    teeth, shapes, provenance, source_hash = load_members(source, reports)
    result = dict(teeth=teeth, source_sha256=source_hash, inputs=provenance,
                  gear_phase_offset_rad=gear_offset, samples=[],
                  scope="Kernel intersection at listed phases only; no between-phase, contact or accuracy certificate")
    for fraction in fractions:
        placed = [BRepBuilderAPI_Transform(s, pose(teeth, m, fraction, gear_offset), True).Shape()
                  for m, s in enumerate(shapes)]
        print(f"Starting common at tooth fraction {fraction}, gear offset {gear_offset}", flush=True)
        started = time.perf_counter()
        operation = BRepAlgoAPI_Common()
        # Reuse of the input solids across phases must not mutate their tolerances.
        operation.SetNonDestructive(True)
        operation.SetArguments(_shapes(placed[:1]))
        operation.SetTools(_shapes(placed[1:]))
        operation.Build()
        row = dict(tooth_fraction=fraction, seconds=time.perf_counter()-started,
                   done=operation.IsDone())
        if operation.IsDone():
            common = operation.Shape()
            row.update(valid=BRepCheck_Analyzer(common).IsValid(),
                       solids=count(common, TopAbs_SOLID), volume_mm3=volume(common))
            if not math.isfinite(row["volume_mm3"]) or row["volume_mm3"] < 0:
                raise RuntimeError("invalid intersection volume")
        result["samples"].append(row)
        output.write_text(json.dumps(result, indent=2)+"\n")
        print(json.dumps(row), flush=True)
    return result


def _shapes(values):
    from OCP.TopTools import TopTools_ListOfShape
    result = TopTools_ListOfShape()
    for value in values:
        result.Append(value)
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("pinion_report", type=Path)
    parser.add_argument("gear_report", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--fractions", type=float, nargs="+", default=[0., .25, .5, .75])
    parser.add_argument("--gear-offset", type=float, default=0.)
    args = parser.parse_args()
    result = run(args.source, [args.pinion_report, args.gear_report], args.output,
                 args.fractions, args.gear_offset)
    raise SystemExit(0 if all(r["done"] and r["valid"] for r in result["samples"]) else 1)
