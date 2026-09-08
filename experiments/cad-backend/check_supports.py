"""Bound complete exported closure surfaces against solved sphere/cone supports."""
import argparse
from fractions import Fraction as F
import hashlib
import json
import math
from pathlib import Path
import time

from bernstein import patches, support_bound


def upper(value):
    result = float(value)
    return math.nextafter(result, math.inf) if F(result) < value else result


def run(path, output, selected=None):
    data = json.loads(path.read_text())
    references = {}
    for name in ("source", "supports"):
        raw = Path(data[f"{name}_file"]).read_bytes()
        if hashlib.sha256(raw).hexdigest() != data[f"{name}_sha256"]:
            raise ValueError(f"{name} changed since coefficient extraction")
        references[name] = json.loads(raw)
    if references["supports"] != data["nominal_supports"]:
        raise ValueError("embedded nominal supports changed")
    if [m["member"] for m in references["source"]["members"]] != [0, 1]:
        raise ValueError("expected ordered source members")
    if len(references["supports"]["members"]) != 2:
        raise ValueError("incomplete solved support members")
    for a, b in zip(references["source"]["members"], references["supports"]["members"]):
        if any(a[k] != b[k] for k in ("member", "teeth", "blank_meridian")):
            raise ValueError("supports disagree with the original CAD source")
    roles = {"root", "tip", "toe_round", "toe_flank", "heel_round", "heel_flank"}
    keys = {(s["member"], s["role"]) for s in data["surfaces"]}
    if len(data["surfaces"]) != 12 or keys != {(m, r) for m in (0, 1) for r in roles}:
        raise ValueError("incomplete or duplicate closure coverage")
    if [item["member"] for item in data["inputs"]] != [0, 1]:
        raise ValueError("incomplete or duplicate STEP member provenance")
    for item in data["inputs"]:
        if hashlib.sha256(Path(item["step_file"]).read_bytes()).hexdigest() != item["step_sha256"]:
            raise ValueError("STEP input changed since coefficient extraction")
    if selected is not None and not selected <= {f"{m}_{r}" for m, r in keys}:
        raise ValueError("unknown selected face")
    result = dict(evidence_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                  inputs=data["inputs"], source_sha256=data["source_sha256"],
                  supports_sha256=data["supports_sha256"], target_mm=.001, faces=[],
                  complete_closure_coverage=False,
                  scope="Exact rational whole-parameter bounds for the extracted binary64 surfaces to infinite nominal supports; trimmed coverage, STEP reader conversion, source solve and generated flank/fillet errors are separate")
    for record in data["surfaces"]:
        m, role = record["member"], record["role"]
        if selected is not None and f"{m}_{role}" not in selected:
            continue
        nominal = data["nominal_supports"]
        if role in ("root", "tip"):
            expected = dict(kind="cone", meridian=nominal["members"][m]["cones_tip_root_back"][role == "root"])
        else:
            expected = dict(kind="sphere", radius_mm=nominal["sphere_radii_mm"][role.startswith("heel")])
        if record["support"] != expected:
            raise ValueError("closure target changed from the solved support")
        started = time.perf_counter()
        maximum, count = F(0), 0
        for patch in patches(record["surface"]):
            maximum = max(maximum, support_bound(patch["poles"], expected))
            count += 1
        row = dict(member=m, role=role, bezier_patches=count, bound_mm=upper(maximum),
                   passes=maximum <= F("0.001"), seconds=time.perf_counter()-started)
        result["faces"].append(row)
        output.write_text(json.dumps(result, indent=2)+"\n")
        print(json.dumps(row), flush=True)
    result["passes_selected_bounds"] = bool(result["faces"]) and all(r["passes"] for r in result["faces"])
    result["complete_closure_coverage"] = selected is None and len(result["faces"]) == 12
    output.write_text(json.dumps(result, indent=2)+"\n")
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--faces", nargs="+")
    args = parser.parse_args()
    result = run(args.input, args.output, set(args.faces) if args.faces else None)
    raise SystemExit(0 if result["passes_selected_bounds"] else 1)
