"""Negative-inside fields matching the Rust expression fixtures.

The independent audit does not import this module. These are max-of-plane fields,
not nearest-face distances, and no corners or crease curves enter the mesher API.
"""
import math

NAMES = ("sphere", "cube", "rotated_cube", "torus", "spiky_tetrahedron",
         "rotated_tetrahedron", "thin_plate", "disconnected", "zero_only")


def dot(a, b):
    return sum(x*y for x, y in zip(a, b))


def sub(a, b):
    return tuple(x-y for x, y in zip(a, b))


def cross(a, b):
    return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])


def rotate(p, angle):
    a = tuple(x/math.sqrt(14) for x in (1, 2, 3))
    s, c = math.sin(angle), math.cos(angle)
    q = cross(a, p)
    return tuple(c*p[k]+s*q[k]+(1-c)*dot(a, p)*a[k] for k in range(3))


def box(angle, half):
    axes = [rotate(v, angle) for v in [(1, 0, 0), (0, 1, 0), (0, 0, 1)]]
    return lambda p: max(abs(dot(p, axis))-h for axis, h in zip(axes, half))


def tetrahedron(angle):
    vertices = [rotate(p, angle) for p in [(0, 0, 1.5), (.06, 0, -.5),
                (-.03, .03*math.sqrt(3), -.5), (-.03, -.03*math.sqrt(3), -.5)]]
    center = tuple(sum(p[k] for p in vertices)/4 for k in range(3))
    planes = []
    for i, j, k in [(0, 1, 2), (0, 2, 3), (0, 3, 1), (1, 3, 2)]:
        a, b, c = vertices[i], vertices[j], vertices[k]
        n = cross(sub(b, a), sub(c, a))
        length = math.sqrt(dot(n, n))
        n = tuple(x/length for x in n)
        if dot(n, sub(center, a)) > 0:
            n = tuple(-x for x in n)
        planes.append((n, dot(n, a)))
    return lambda p: max(dot(n, p)-offset for n, offset in planes)


def build(name):
    if name == "sphere":
        return lambda p: math.sqrt(dot(p, p))-1, 1.25
    if name == "cube":
        return box(0, (1, 1, 1)), 1.25
    if name == "rotated_cube":
        return box(.47, (1, 1, 1)), 1.75
    if name == "thin_plate":
        return box(.47, (.02, .7, .7)), 1.25
    if name == "torus":
        return lambda p: math.hypot(math.hypot(p[0], p[1])-2, p[2])-.6, 3
    if name in ("spiky_tetrahedron", "rotated_tetrahedron"):
        return tetrahedron(.47 if name == "rotated_tetrahedron" else 0), 1.75
    if name == "disconnected":
        return lambda p: min(math.dist(p, (-.4, 0, 0))-.3,
                             math.dist(p, (.53, .11, .07))-.035), 1
    if name == "zero_only":
        return lambda p: abs(math.sqrt(dot(p, p))-1), 1.25
    raise ValueError(f"unknown fixture {name}")
