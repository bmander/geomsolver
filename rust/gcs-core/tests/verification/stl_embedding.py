"""Independent exact embedding audit of a closed binary-STL triangle mesh.

All encoded coordinates are dyadic rationals. One uniform power-of-two scale
turns them into integers; plane predicates, clipping and interval intersections
then use integers/Fractions only. No tolerance, projection of vertices, or weld
changes the geometry. See docs/stl-embedding-verification.md for the argument.
"""
import hashlib
import json
import math
import struct
import sys
import time
from fractions import Fraction
from pathlib import Path

import stl_topology


def sub(a, b):
    return tuple(x-y for x, y in zip(a, b))


def dot(a, b):
    return sum(x*y for x, y in zip(a, b))


def cross(a, b):
    return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])


def orient(a, b, p):
    return (b[0]-a[0])*(p[1]-a[1])-(b[1]-a[1])*(p[0]-a[0])


def read_triangles(data):
    if len(data) < 84:
        raise ValueError("incomplete binary STL header")
    count = struct.unpack_from('<I', data, 80)[0]
    if not count or len(data) != 84+50*count:
        raise ValueError("invalid binary STL length or triangle count")
    rows = [struct.unpack_from('<9f', data, 96+50*i) for i in range(count)]
    if not all(math.isfinite(x) for row in rows for x in row):
        raise ValueError("nonfinite encoded coordinate")
    # Every binary32 value is represented exactly by its Python binary64 value.
    ratios = {x: x.as_integer_ratio() for row in rows for x in row}
    exponent = max(d.bit_length()-1 for _, d in ratios.values())
    integers = {x: n << (exponent-(d.bit_length()-1)) for x, (n, d) in ratios.items()}
    triangles = [tuple(tuple(integers[row[3*j+k]] for k in range(3))
                       for j in range(3)) for row in rows]
    for i, t in enumerate(triangles):
        if not any(cross(sub(t[1], t[0]), sub(t[2], t[0]))):
            raise ValueError(f"exactly degenerate encoded triangle {i}")
    return triangles, exponent


def strictly_one_side(signs):
    return min(signs) > 0 or max(signs) < 0


def plane_section(triangle, signs, axis):
    """Coordinate interval of a noncoplanar triangle's intersection with a plane."""
    points = [p[axis] for p, s in zip(triangle, signs) if s == 0]
    for i in range(3):
        j = (i+1) % 3
        if signs[i]*signs[j] < 0:
            points.append(triangle[i][axis]+Fraction(signs[i], signs[i]-signs[j])
                          *(triangle[j][axis]-triangle[i][axis]))
    if not points:
        raise ValueError("invalid plane-section precondition")
    return min(points), max(points)


def clip(subject, triangle):
    """Closed convex polygon intersection in a nondegenerate plane projection."""
    sense = orient(*triangle)
    assert sense != 0
    for i in range(3):
        if not subject:
            break
        a, b = triangle[i], triangle[(i+1) % 3]
        output = []
        previous = subject[-1]
        before = sense*orient(a, b, previous)
        for current in subject:
            after = sense*orient(a, b, current)
            if (before < 0) != (after < 0):
                t = Fraction(before, before-after)
                output.append(tuple(p+t*(c-p) for p, c in zip(previous, current)))
            if after >= 0:
                output.append(current)
            previous, before = current, after
        subject = output
    return subject


def improper_intersection(a, b):
    """True unless the intersection is empty or precisely a common simplex.

    Inputs are nondegenerate integer triangles. An intended shared vertex/edge
    is recognized by exact coordinates, as in the encoded-STL topology audit.
    """
    na = cross(sub(a[1], a[0]), sub(a[2], a[0]))
    nb = cross(sub(b[1], b[0]), sub(b[2], b[0]))
    if not any(na) or not any(nb):
        raise ValueError("degenerate triangle in intersection predicate")
    sa = [dot(nb, sub(p, b[0])) for p in a]
    sb = [dot(na, sub(p, a[0])) for p in b]
    if strictly_one_side(sa) or strictly_one_side(sb):
        return False
    shared = set(a).intersection(b)
    if len(shared) == 3:
        return True
    if not any(sa):
        # Drop a coordinate with nonzero normal component: this projection is
        # one-to-one on the common plane, including its boundary segments.
        drop = max(range(3), key=lambda k: abs(na[k]))
        project = lambda p: tuple(p[k] for k in range(3) if k != drop)
        if len(shared) == 2:
            p, q = map(project, sorted(shared))
            ar = project(next(v for v in a if v not in shared))
            br = project(next(v for v in b if v not in shared))
            # Opposite half-planes meet only along the common edge. Same-side
            # triangles overlap in positive area next to the edge's interior.
            return orient(p, q, ar)*orient(p, q, br) > 0
        polygon = clip(list(map(project, a)), tuple(map(project, b)))
        if not polygon:
            return False
        return not (len(shared) == 1 and
                    all(p == project(next(iter(shared))) for p in polygon))
    if len(shared) == 2:
        # Distinct planes meet in the common edge's support line. Each
        # nondegenerate triangle meets that line in exactly its shared edge.
        return False
    direction = cross(na, nb)
    axis = next(k for k in range(3) if direction[k] != 0)
    alo, ahi = plane_section(a, sa, axis)
    blo, bhi = plane_section(b, sb, axis)
    lo, hi = max(alo, blo), min(ahi, bhi)
    if lo > hi:
        return False
    return not (len(shared) == 1 and lo == hi == next(iter(shared))[axis])


def candidate_pairs(triangles):
    """All closed-AABB overlaps, exactly once. Strict gaps alone prune pairs."""
    boxes = [(tuple(min(p[k] for p in t) for k in range(3)),
              tuple(max(p[k] for p in t) for k in range(3))) for t in triangles]
    active = []
    for i in sorted(range(len(boxes)), key=lambda i: (boxes[i][0][0], i)):
        lo, hi = boxes[i]
        active = [j for j in active if boxes[j][1][0] >= lo[0]]
        for j in active:
            a, b = boxes[j]
            if all(a[k] <= hi[k] and lo[k] <= b[k] for k in (1, 2)):
                yield j, i
        active.append(i)


def check(data):
    started = time.monotonic()
    triangles, exponent = read_triangles(data)
    topology = stl_topology.check(data)
    tested = 0
    for a, b in candidate_pairs(triangles):
        tested += 1
        if improper_intersection(triangles[a], triangles[b]):
            raise ValueError(f"encoded triangles {a} and {b} intersect beyond their shared simplex")
    # With embedded, connected, closed and consistently oriented topology, the
    # sign distinguishes the bounded region's outward from inward orientation.
    numerator = sum(dot(a, cross(b, c)) for a, b, c in triangles)
    if numerator <= 0:
        raise ValueError("encoded shell does not have positive exact signed volume")
    volume = Fraction(numerator, 6*(1 << exponent)**3)
    return dict(**topology, sha256=hashlib.sha256(data).hexdigest(),
                bytes=len(data), coordinate_scale_exponent=exponent,
                signed_volume_exact=str(volume), embedded=True,
                all_triangle_pairs=len(triangles)*(len(triangles)-1)//2,
                exact_pair_tests=tested, seconds=time.monotonic()-started,
                scope="exact embedding and outward orientation of this encoded closed mesh; "
                      "source geometry, material coverage, surface deviation and mating not certified")


if __name__ == '__main__':
    if len(sys.argv) < 2:
        raise SystemExit('usage: python3 stl_embedding.py FILE.stl [FILE.stl ...]')
    stl_topology.self_test()
    print(json.dumps([dict(file=str(path), **check(path.read_bytes()))
                      for path in map(Path, sys.argv[1:])], indent=2))
