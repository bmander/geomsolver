"""Audit the encoded STL, independently of Fidget. Python standard library only.

Topology and intersections are exact on encoded coordinates. Source deviation and
coverage are samples, not a Hausdorff bound or a whole-boundary certificate.
"""
import hashlib
import itertools
import json
import math
from pathlib import Path
import struct
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2] /
                       "rust/gcs-core/tests/verification"))
import stl_embedding
import stl_topology


def sub(a, b):
    return tuple(x-y for x, y in zip(a, b))


def dot(a, b):
    return sum(x*y for x, y in zip(a, b))


def cross(a, b):
    return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])


def norm(a):
    return math.sqrt(dot(a, a))


def rotate(p, angle):
    axis = tuple(x/math.sqrt(14) for x in (1, 2, 3))
    c, s = math.cos(angle), math.sin(angle)
    perpendicular = cross(axis, p)
    return tuple(c*p[k]+s*perpendicular[k]+(1-c)*dot(axis, p)*axis[k] for k in range(3))


def segment_distance(p, a, b):
    edge = sub(b, a)
    t = max(0, min(1, dot(sub(p, a), edge)/dot(edge, edge)))
    return norm(tuple(p[k]-a[k]-t*edge[k] for k in range(3)))


def triangle_distance(p, triangle):
    a, b, c = triangle
    n = cross(sub(b, a), sub(c, a))
    squared = dot(n, n)
    if squared:
        t = dot(sub(p, a), n)/squared
        q = tuple(p[k]-t*n[k] for k in range(3))
        if all(dot(cross(sub(v, u), sub(q, u)), n) >= 0
               for u, v in ((a, b), (b, c), (c, a))):
            return abs(t)*math.sqrt(squared)
    return min(segment_distance(p, u, v) for u, v in ((a, b), (b, c), (c, a))
               if u != v)


def reference(name):
    """Return distance to finite source boundary and coverage landmarks."""
    directions = [tuple(sign if k == axis else 0 for k in range(3))
                  for axis in range(3) for sign in (-1, 1)]
    if name == "sphere":
        return lambda p: abs(norm(p)-1), directions
    if name in ("cube", "rotated_cube", "thin_plate"):
        half = (.02, .7, .7) if name == "thin_plate" else (1, 1, 1)
        angle = 0 if name == "cube" else .47

        def distance(p):
            q = [abs(x)-h for x, h in zip(rotate(p, -angle), half)]
            return abs(norm([max(0, x) for x in q])+min(0, max(q)))

        corners = [rotate(tuple(x*h for x, h in zip(signs, half)), angle)
                   for signs in itertools.product((-1, 1), repeat=3)]
        return distance, corners
    if name == "torus":
        points = [((2+.6*math.cos(v))*math.cos(u),
                   (2+.6*math.cos(v))*math.sin(u), .6*math.sin(v))
                  for u in [k*math.pi/4 for k in range(8)]
                  for v in [k*math.pi/2 for k in range(4)]]
        return lambda p: abs(math.hypot(math.hypot(p[0], p[1])-2, p[2])-.6), points
    if name in ("spiky_tetrahedron", "rotated_tetrahedron"):
        angle = .47 if name == "rotated_tetrahedron" else 0
        points = [rotate(p, angle) for p in [(0, 0, 1.5), (.06, 0, -.5),
                  (-.03, .03*math.sqrt(3), -.5), (-.03, -.03*math.sqrt(3), -.5)]]
        faces = [tuple(points[i] for i in face) for face in
                 [(0, 1, 2), (0, 2, 3), (0, 3, 1), (1, 3, 2)]]
        return lambda p: min(triangle_distance(p, t) for t in faces), points
    if name == "disconnected":
        balls = [((-.4, 0, 0), .3), ((.53, .11, .07), .035)]
        points = [tuple(c[k]+r*d[k] for k in range(3)) for c, r in balls for d in directions]
        return lambda p: min(abs(norm(sub(p, c))-r) for c, r in balls), points
    raise ValueError(f"unknown reference {name}")


def components(triangles):
    parent = list(range(len(triangles)))

    def root(i):
        while i != parent[i]:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    owner = {}
    for i, triangle in enumerate(triangles):
        for vertex in triangle:
            if vertex in owner:
                parent[root(i)] = root(owner[vertex])
            owner[vertex] = i
    groups = {}
    for i in range(len(triangles)):
        groups.setdefault(root(i), []).append(i)
    return list(groups.values())


def audit(path, tolerance=.02):
    data = path.read_bytes()
    metadata = json.loads(path.with_suffix(".json").read_text())
    name = metadata["case"]
    count = struct.unpack_from("<I", data, 80)[0]
    report = {key: metadata[key] for key in
              ("case", "backend", "depth", "extraction_seconds", "setup_seconds")}
    for key in ("grid_divisions", "edge_length", "vertex_tolerance", "point_queries",
                "callback", "library_status", "min_feature", "max_err", "threads",
                "postprocessing", "postprocessing_seconds", "pipeline_seconds", "cgal_repair",
                "indexed_cleanup"):
        if key in metadata:
            report[key] = metadata[key]
    report.update(file=str(path), sha256=hashlib.sha256(data).hexdigest(), triangles=count,
                  sampled_tolerance=tolerance, failures=[])
    failures = report["failures"]
    if metadata.get("producer_failure", False):
        failures.append("producer reported a failed repair")
    if metadata.get("library_status", "Error.NoError") != "Error.NoError":
        failures.append("producer reported a library error")
    if not count:
        if len(data) != 84 or name != "zero_only":
            failures.append("unexpected empty mesh or malformed empty STL")
        report["passes_sampled_checks"] = not failures
        return report
    if name == "zero_only":
        failures.append("zero-only field produced material boundary")
    try:
        exact, exponent = stl_embedding.read_triangles(data)
    except ValueError as error:
        failures.append(str(error))
        report["passes_sampled_checks"] = False
        return report
    groups = components(exact)
    report["components"] = len(groups)
    if len(groups) != (2 if name == "disconnected" else 1):
        failures.append("unexpected component count")
    report["topology"] = []
    for group in groups:
        shell = bytes(80)+struct.pack("<I", len(group))
        shell += b"".join(data[84+50*i:134+50*i] for i in group)
        try:
            topology = stl_topology.check(shell)
            report["topology"].append(topology)
            volume_numerator = sum(dot(exact[i][0], cross(exact[i][1], exact[i][2]))
                                   for i in group)
            if volume_numerator <= 0 or topology["genus"] != (1 if name == "torus" else 0):
                failures.append("unexpected volume orientation or genus")
        except ValueError as error:
            failures.append(str(error))
    pair_count, intersections = 0, []
    for i, j in stl_embedding.candidate_pairs(exact):
        pair_count += 1
        if stl_embedding.improper_intersection(exact[i], exact[j]):
            intersections.append((i, j))
    report.update(candidate_pairs=pair_count, improper_intersection_count=len(intersections),
                  first_improper_intersections=intersections[:20])
    if intersections:
        failures.append("improper encoded triangle intersections")
    if name != "zero_only":
        triangles = [tuple(tuple(math.ldexp(x, -exponent) for x in p) for p in t) for t in exact]
        distance, landmarks = reference(name)
        maximum = 0
        for triangle in triangles:
            a, b, c = triangle
            samples = list(triangle)+[tuple((u[k]+v[k])/2 for k in range(3))
                                      for u, v in ((a, b), (b, c), (c, a))]
            samples.append(tuple((a[k]+b[k]+c[k])/3 for k in range(3)))
            maximum = max(maximum, *(distance(p) for p in samples))
        coverage = [min(triangle_distance(p, t) for t in triangles) for p in landmarks]
        report.update(sampled_source_deviation=maximum, landmark_distances=coverage,
                      maximum_landmark_distance=max(coverage))
        if maximum > tolerance or max(coverage) > tolerance:
            failures.append("sampled source deviation or landmark coverage exceeds tolerance")
    report["passes_sampled_checks"] = not failures
    return report


if __name__ == "__main__":
    if len(sys.argv) < 2:
        raise SystemExit("usage: python3 audit.py FILE.stl [FILE.stl ...]")
    failed = False
    for path in map(Path, sys.argv[1:]):
        report = audit(path)
        path.with_suffix(".audit.json").write_text(json.dumps(report, indent=2)+"\n")
        print(json.dumps(report), flush=True)
        failed |= not report["passes_sampled_checks"]
    raise SystemExit(1 if failed else 0)
