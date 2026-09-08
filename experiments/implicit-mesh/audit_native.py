"""Distinguish native mesh degeneracy from STL quantization failure.

Exact predicates on the JSON's binary64 coordinates. This diagnostic does not
check source fidelity or certify topology. It never repairs or converts a mesh.
"""
import hashlib
import json
from pathlib import Path
import sys

import audit


def check(path):
    data = path.read_bytes()
    source = json.loads(data)
    ratios = {x: x.as_integer_ratio() for p in source["vertices"] for x in p}
    exponent = max((d.bit_length()-1 for _, d in ratios.values()), default=0)
    scaled = {x: n << (exponent-d.bit_length()+1) for x, (n, d) in ratios.items()}
    vertices = [tuple(scaled[x] for x in p) for p in source["vertices"]]
    triangles = [tuple(vertices[i] for i in t) for t in source["triangles"]]
    degenerate = [i for i, (a, b, c) in enumerate(triangles)
                  if not any(audit.cross(audit.sub(b, a), audit.sub(c, a)))]
    intersections = None
    if not degenerate:
        intersections = [(i, j) for i, j in audit.stl_embedding.candidate_pairs(triangles)
                         if audit.stl_embedding.improper_intersection(triangles[i], triangles[j])]
    return dict(file=path.name, sha256=hashlib.sha256(data).hexdigest(),
                case=source["case"], triangles=len(triangles),
                native_binary64_degenerate_triangles=degenerate,
                native_binary64_improper_pairs=intersections,
                intersection_check="skipped for degenerate input" if degenerate else "complete",
                scope="Native triangle embedding diagnostic only; no source or topology certificate.")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        raise SystemExit("usage: python3 audit_native.py PRODUCER.json")
    path = Path(sys.argv[1])
    report = check(path)
    path.with_suffix(".native-audit.json").write_text(json.dumps(report, indent=2)+"\n")
    print(json.dumps(report))
    raise SystemExit(1 if report["native_binary64_degenerate_triangles"] or
                     report["native_binary64_improper_pairs"] else 0)
