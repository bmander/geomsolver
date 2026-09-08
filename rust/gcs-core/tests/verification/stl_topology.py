"""Independent binary-STL topology check; Python standard library only.

This checks exact encoded coordinates, orientation, vertex fans and connectivity.
It does not certify self-intersection freedom or deviation from analytic geometry.
"""
import json
import math
import struct
import sys
from pathlib import Path


def check(data):
    if len(data) < 84:
        raise ValueError("incomplete binary STL header")
    faces = struct.unpack_from("<I", data, 80)[0]
    if len(data) != 84 + 50 * faces or not faces:
        raise ValueError("invalid triangle count")
    ids, halfedges, first, degree, parent = {}, {}, [], [], []

    def vertex(p):
        if p not in ids:
            ids[p] = len(parent)
            parent.append(len(parent))
            first.append(None)
            degree.append(0)
        return ids[p]

    def root(v):
        while parent[v] != v:
            parent[v] = parent[parent[v]]
            v = parent[v]
        return v

    volumes = []
    for f in range(faces):
        flat = struct.unpack_from("<9f", data, 84 + 50 * f + 12)
        if not all(map(math.isfinite, flat)):
            raise ValueError("nonfinite vertex")
        p, q, r = flat[:3], flat[3:6], flat[6:]
        u = [q[k] - p[k] for k in range(3)]
        v = [r[k] - p[k] for k in range(3)]
        cross = [u[1]*v[2]-u[2]*v[1], u[2]*v[0]-u[0]*v[2], u[0]*v[1]-u[1]*v[0]]
        if not any(cross):
            raise ValueError("degenerate triangle")
        a, b, c = map(vertex, (p, q, r))
        parent[root(a)] = root(b)
        parent[root(b)] = root(c)
        for a, b, c in ((a, b, c), (b, c, a), (c, a, b)):
            if (a, b) in halfedges:
                raise ValueError("repeated directed edge")
            halfedges[a, b] = c
            first[a] = b
            degree[a] += 1
        volumes.append(sum(p[k]*cross[k] for k in range(3))/6)
    for a, b in halfedges:
        if (b, a) not in halfedges:
            raise ValueError("open or inconsistently oriented edge")
    for a, start in enumerate(first):
        seen, b = set(), start
        while b not in seen:
            seen.add(b)
            b = halfedges[b, a]
        if b != start or len(seen) != degree[a]:
            raise ValueError("nonmanifold vertex fan")
    if len({root(v) for v in range(len(parent))}) != 1:
        raise ValueError("disconnected shell")
    edges = len(halfedges) // 2
    euler = len(parent) - edges + faces
    if euler > 2 or euler % 2:
        raise ValueError("invalid Euler characteristic")
    return dict(vertices=len(parent), edges=edges, triangles=faces,
                euler=euler, genus=(2-euler)//2, signed_volume=math.fsum(volumes))


def self_test():
    p = [(0., 0., 0.), (1., 0., 0.), (0., 1., 0.), (0., 0., 1.)]
    t = [(0, 2, 1), (0, 1, 3), (1, 2, 3), (2, 0, 3)]

    def encode(points, triangles):
        out = bytearray(80) + struct.pack("<I", len(triangles))
        for tri in triangles:
            out += bytes(12) + struct.pack("<9f", *(x for i in tri for x in points[i])) + bytes(2)
        return out

    if check(encode(p, t))["genus"] != 0:
        raise ValueError("sphere self-test failed")
    bad = [encode(p, t[:-1]),
           encode(p + [(x+3, y, z) for x, y, z in p], t + [tuple(i+4 for i in f) for f in t]),
           encode(p + [tuple(-x for x in v) for v in p[1:]],
                  t + [tuple(0 if i == 0 else i+3 for i in f) for f in t])]
    for data in bad:
        try:
            check(data)
        except ValueError:
            continue
        raise ValueError("invalid-shell self-test was accepted")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        raise SystemExit("usage: python3 stl_topology.py FILE.stl [FILE.stl ...]")
    self_test()
    print(json.dumps([dict(file=str(path), **check(path.read_bytes()))
                      for path in map(Path, sys.argv[1:])], indent=2))
