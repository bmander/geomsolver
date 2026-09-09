"""Repeat the checked tooth-space candidate with the CAD kernel's Boolean API."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import time

from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.gp import gp_Ax1, gp_Dir, gp_Pnt, gp_Trsf
from OCP.TopAbs import TopAbs_SOLID
from OCP.TopTools import TopTools_ListOfShape

from material_probes import read
from tooth_space import blank, count, roundtrip, volume


def run(source, spaces, output, full=False, selected=None):
    data = json.loads(source.read_text())
    space_report = json.loads((spaces/"report.json").read_text())
    if space_report["source_sha256"] != hashlib.sha256(source.read_bytes()).hexdigest():
        raise ValueError("tooth-space report uses a different source")
    output.mkdir(parents=True, exist_ok=True)
    results = []
    for member in data["members"]:
        index = member["member"]
        if selected is not None and index != selected:
            continue
        path = spaces/f"member{index}-space-16.step"
        rows = [r for r in space_report["results"]
                if r["member"] == index and r["subdivisions"] == 16]
        if (len(rows) != 1 or not rows[0]["passes_local_checks"]
                or rows[0]["step_sha256"] != hashlib.sha256(path.read_bytes()).hexdigest()):
            raise ValueError("missing, rejected or changed tooth-space input")
        tool = read(path)
        if not BRepCheck_Analyzer(tool).IsValid():
            raise ValueError("invalid tooth-space input")
        teeth = int(member["teeth"])
        copies = teeth if full else 2
        row = dict(member=index, cuts=copies, tool_sha256=hashlib.sha256(path.read_bytes()).hexdigest(), failures=[])
        body = blank(member)
        initial_volume = volume(body)
        arguments, tools = TopTools_ListOfShape(), TopTools_ListOfShape()
        arguments.Append(body)
        for i in range(copies):
            transform = gp_Trsf()
            transform.SetRotation(gp_Ax1(gp_Pnt(0,0,0), gp_Dir(0,0,1)), math.tau*i/teeth)
            tools.Append(BRepBuilderAPI_Transform(tool, transform, True).Shape())
        operation = BRepAlgoAPI_Cut()
        operation.SetArguments(arguments)
        operation.SetTools(tools)
        started = time.perf_counter()
        operation.Build()
        row["boolean_seconds"] = time.perf_counter()-started
        if not operation.IsDone():
            row["failures"].append("Boolean operation failed")
        else:
            shape = operation.Shape()
            result_volume = volume(shape)
            row.update(valid=BRepCheck_Analyzer(shape).IsValid(), solids=count(shape, TopAbs_SOLID),
                       volume=result_volume, blank_volume=initial_volume)
            if not row["valid"] or row["solids"] != 1 or not 0 < result_volume < initial_volume:
                row["failures"].append("invalid, multiple, or non-removing result")
            row.update(roundtrip(shape, output/f"member{index}-{copies}-cuts.step"))
            if not row["roundtrip_valid"] or row["roundtrip_solids"] != 1:
                row["failures"].append("STEP topology changed or invalid")
            if abs(result_volume-row["roundtrip_volume"]) > 1e-8*abs(result_volume):
                row["failures"].append("STEP volume mismatch")
        row["passes_local_checks"] = not row["failures"]
        results.append(row)
        print(json.dumps(row), flush=True)
        # Persist each completed member even if a later native operation is interrupted.
        (output/"report.json").write_text(json.dumps(dict(results=results,
            source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
            scope="Indexed CAD candidate and STEP checks only; complete surface and pair acceptance pending"),indent=2)+"\n")
    return bool(results) and all(row["passes_local_checks"] for row in results)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("spaces", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--full", action="store_true")
    parser.add_argument("--member", type=int, choices=(0, 1))
    args = parser.parse_args()
    raise SystemExit(0 if run(args.source, args.spaces, args.output, args.full, args.member) else 1)
