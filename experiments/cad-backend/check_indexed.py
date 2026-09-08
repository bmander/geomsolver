"""Check every indexed STEP tooth at the previously audited material probe locations."""
import argparse
import hashlib
import json
import math
from pathlib import Path

from OCP.BRepClass3d import BRepClass3d_SolidClassifier
from OCP.gp import gp_Pnt
from OCP.TopAbs import TopAbs_IN, TopAbs_OUT

from material_probes import read


def check(probes, evidence, report_path):
    audit = json.loads(evidence.read_text())
    if audit["probe_sha256"] != hashlib.sha256(probes.read_bytes()).hexdigest():
        raise ValueError("probes differ from independently audited inputs")
    report = json.loads(report_path.read_text())
    if not report["results"]:
        raise ValueError("no CAD candidates supplied")
    rows = []
    for candidate in report["results"]:
        if not candidate["passes_local_checks"]:
            raise ValueError("refused CAD candidate")
        member = candidate["member"]
        name = ("pinion", "gear")[member]
        teeth = audit["members"][name]["teeth"]
        if candidate["cuts"] != teeth:
            raise ValueError("this checker requires all indexed cuts")
        path = Path(candidate["step_file"])
        if candidate["step_sha256"] != hashlib.sha256(path.read_bytes()).hexdigest():
            raise ValueError("STEP artifact changed")
        provenance = {row["member"]: row for row in audit["cad_provenance"]}
        if candidate["tool_sha256"] != provenance[member]["space_sha256"]:
            raise ValueError("indexed cutter differs from audited tooth space")
        classifier = BRepClass3d_SolidClassifier(read(path))
        count, failures = 0, []
        for line in probes.read_text().splitlines():
            selected, label, expected, x, y, z = line.split()
            if int(selected) != member:
                continue
            x, y, z = map(float, (x, y, z))
            for index in range(teeth):
                angle = math.tau*index/teeth
                c, s = math.cos(angle), math.sin(angle)
                p = gp_Pnt(c*x-s*y, s*x+c*y, z)
                classifier.Perform(p, 1e-7)
                state = classifier.State()
                count += 1
                if state not in (TopAbs_IN, TopAbs_OUT) or (state == TopAbs_IN) != (expected == "1"):
                    failures.append(dict(label=label, index=index, expected_inside=expected == "1",
                                         state=str(state)))
        if count != audit["members"][name]["points"]*teeth:
            raise ValueError("missing indexed material probes")
        row = dict(member=member, teeth=teeth, points=count, failures=failures,
                   step_sha256=candidate["step_sha256"], passes=not failures)
        rows.append(row)
        print(json.dumps(row), flush=True)
    return dict(results=rows, audit_sha256=hashlib.sha256(evidence.read_bytes()).hexdigest(),
                scope="CAD classifications at rotated copies of independently audited probes; not whole-boundary certification")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("probes", type=Path)
    parser.add_argument("audit", type=Path)
    parser.add_argument("candidate", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    report = check(args.probes, args.audit, args.candidate)
    args.output.write_text(json.dumps(report, indent=2)+"\n")
    raise SystemExit(0 if all(row["passes"] for row in report["results"]) else 1)
