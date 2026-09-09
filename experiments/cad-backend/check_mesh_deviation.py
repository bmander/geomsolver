"""Independently bound complete STL triangles against associated parameter surfaces.

Finite trimmed-face coverage and nominal-source accuracy remain separate checks.
"""
import argparse
from fractions import Fraction as F
import hashlib
import json
import math
from pathlib import Path
import struct
import time

from mesh_deviation import surface, triangle_bound, vertex_error, projected_witness


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def oriented_equal(a, b):
    return any(a == b[i:]+b[:i] for i in range(3))


def upper_float(x):
    value = float(x)
    return math.nextafter(value, math.inf) if F(value) < x else value


def run(source, output, tolerance=F(1, 50)):
    if tolerance <= 0 or output.resolve() == source.resolve():
        raise ValueError("positive tolerance and separate output required")
    data = json.loads(source.read_text())
    for role in ("source", "stl"):
        path = Path(data[f"{role}_file"])
        if digest(path) != data[f"{role}_sha256"] or output.resolve() == path.resolve():
            raise ValueError("changed input or attempted input overwrite")
    encoded = Path(data["stl_file"]).read_bytes()
    if len(encoded) < 84:
        raise ValueError("incomplete STL")
    count = struct.unpack_from("<I", encoded, 80)[0]
    if count == 0 or len(encoded) != 84+50*count:
        raise ValueError("invalid STL triangle count")
    result = dict(input_file=str(source), input_sha256=digest(source),
        stl_sha256=data["stl_sha256"], source_sha256=data["source_sha256"],
        surface_distance_target_mm=str(tolerance), faces=[],
        scope="Whole-triangle distance to associated support surfaces, including encoded vertex error. Finite trim coverage, reverse coverage and nominal source fidelity are not established.")
    seen, faces = set(), set()
    started = time.perf_counter()
    for face in data["faces"]:
        index = face["face_index"]
        if index in faces:
            raise ValueError("duplicate face index")
        faces.add(index)
        evaluator = surface(face["surface"])
        nodes = [tuple(map(F, p)) for p in face["nodes"]]
        maximum, cells, failures = F(0), 0, []
        if not face["triangles"]:
            raise ValueError("empty face triangulation")
        for identifier, *indices in face["triangles"]:
            if (type(identifier) is not int or identifier in seen or not 0 <= identifier < count
                    or len(indices) != 3 or any(type(i) is not int or not 0 <= i < len(nodes) for i in indices)):
                raise ValueError("invalid or duplicate triangle reference")
            points = [nodes[i] for i in indices]
            row = struct.unpack_from("<9f", encoded, 96+50*identifier)
            original = [tuple(map(F, row[k:k+3])) for k in (0, 3, 6)]
            if not oriented_equal([p[2:] for p in points], original):
                raise ValueError("associated triangle does not match encoded STL")
            seen.add(identifier)
            bound = triangle_bound(evaluator, points, tolerance)
            cells += bound["cells"]
            maximum = max(maximum, bound["bound"])
            if not bound["passes"]:
                failures.append(dict(triangle=identifier, upper_mm=upper_float(bound["bound"])))
        row = dict(face_index=index, kind=face["surface"]["kind"], triangles=len(face["triangles"]),
                   bound_mm=upper_float(maximum), cells=cells, failures=failures)
        result["faces"].append(row)
        evaluator.value.cache_clear()
        vertex_error.cache_clear()
        projected_witness.cache_clear()
        output.write_text(json.dumps(result, indent=2)+"\n")
        print(json.dumps(row), flush=True)
    if len(seen) != count:
        raise ValueError("omitted encoded triangles")
    result.update(triangles=count, seconds=time.perf_counter()-started,
                  maximum_bound_mm=max(r["bound_mm"] for r in result["faces"]),
                  passes_surface_distance_bound=all(not r["failures"] for r in result["faces"]))
    output.write_text(json.dumps(result, indent=2)+"\n")
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--tolerance-mm", type=F, default=F(1, 50))
    args = parser.parse_args()
    raise SystemExit(0 if run(args.source, args.output, args.tolerance_mm)["passes_surface_distance_bound"] else 1)
