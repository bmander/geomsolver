"""Bind CAD probes to the independent whole-roll material evidence audit."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]/"rust/gcs-core/tests/verification"))
import member_material


def check(probes, evidence):
    raw = probes.read_bytes()
    metadata = json.loads(probes.with_suffix(".json").read_text())
    if hashlib.sha256(raw).hexdigest() != metadata["probe_sha256"]:
        raise ValueError("CAD probe provenance mismatch")
    data = json.loads(evidence.read_text())
    if data["cad_failures"]:
        raise ValueError(f"CAD material disagreements or unresolved queries: {data['cad_failures']}")
    expected = {}
    for line in raw.decode().splitlines():
        member, label, inside, *point = line.split()
        key = (int(member), label)
        if key in expected:
            raise ValueError("duplicate CAD probe")
        expected[key] = (inside == "1", list(map(float, point)))
    seen, maximum_frame_error = set(), 0
    cases = {c["definition"]["member"]: c for c in data["cases"]}
    teeth = [cases[name]["definition"]["teeth"] for name in ("pinion", "gear")]
    for member, name in enumerate(("pinion", "gear")):
        angle = math.pi/2-(1 if member == 0 else -1)*math.atan2(teeth[member], teeth[1-member])
        c, s = math.cos(angle), math.sin(angle)
        for row in cases[name]["points"]:
            key = (member, row["label"])
            if key in seen or key not in expected:
                raise ValueError("unexpected or duplicate material evidence")
            seen.add(key)
            inside, (x, y, z) = expected[key]
            if inside != row["expected_inside"]:
                raise ValueError("material expectation was changed")
            world = (c*x+s*z, y, -s*x+c*z)
            error = math.dist(world, row["position_mm"])
            maximum_frame_error = max(maximum_frame_error, error)
            if error > 1e-10:
                raise ValueError("CAD and material coordinate frames disagree")
    if seen != expected.keys():
        raise ValueError("material evidence omitted CAD probes")
    report = member_material.check(data, progress=True)
    report.update(cad_probe_count=len(expected), maximum_coordinate_frame_difference_mm=maximum_frame_error,
                  probe_sha256=metadata["probe_sha256"], cad_provenance=metadata["provenance"],
                  evidence_sha256=hashlib.sha256(evidence.read_bytes()).hexdigest())
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("probes", type=Path)
    parser.add_argument("evidence", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    report = check(args.probes, args.evidence)
    args.output.write_text(json.dumps(report, indent=2)+"\n")
    print(json.dumps(report), flush=True)
