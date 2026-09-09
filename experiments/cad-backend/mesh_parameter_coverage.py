"""Exact oriented-chain coverage of a rectangular parameter domain."""
from fractions import Fraction as F

from check_face_domains import rectangle


def cover(face):
    uv = [tuple(map(F, p[:2])) for p in face["nodes"]]
    if any(len(p) != 2 or any(not 0 <= x <= 1 for x in p) for p in uv):
        raise ValueError("UV vertex leaves the unit square")
    edges, orientation, area = {}, None, F(0)
    for row in face["triangles"]:
        indices = row[1:]
        if len(indices) != 3 or any(type(i) is not int or not 0 <= i < len(uv) for i in indices):
            raise ValueError("invalid triangle indices")
        a, b, c = [uv[i] for i in indices]
        twice = (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
        if not twice:
            raise ValueError("degenerate UV triangle")
        sign = 1 if twice > 0 else -1
        if orientation is not None and orientation != sign:
            raise ValueError("mixed UV triangle orientation")
        orientation = sign
        area += abs(twice)/2
        if sign < 0:
            b, c = c, b
        for p, q in ((a, b), (b, c), (c, a)):
            key = (p, q) if p < q else (q, p)
            edges[key] = edges.get(key, 0)+(1 if p < q else -1)
    if area != 1:
        raise ValueError("triangle area does not equal the unit square")
    boundary = {}
    for (p, q), count in edges.items():
        if not count:
            continue
        if abs(count) != 1:
            raise ValueError("overlapping boundary or duplicate triangle")
        if count < 0:
            p, q = q, p
        if p in boundary:
            raise ValueError("branched parameter boundary")
        boundary[p] = q
    if not boundary or len(set(boundary.values())) != len(boundary):
        raise ValueError("missing or disconnected parameter boundary")
    start = current = min(boundary)
    wire, seen = [], set()
    while current not in seen:
        seen.add(current)
        if current not in boundary:
            raise ValueError("open parameter boundary")
        following = boundary[current]
        wire.append(dict(location=current, direction=[b-a for a, b in zip(current, following)],
                         interval=[0, 1], reversed=False))
        current = following
    if current != start or len(seen) != len(boundary):
        raise ValueError("multiple parameter boundary cycles")
    domain = rectangle([wire])
    return dict(**domain, parameter_triangles=len(face["triangles"]),
                parameter_area_exact=str(area), parameter_coverage="complete_once")
