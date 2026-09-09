"""Whole-triangle correspondence to a parameter surface, using exact curvature bounds.

This bounds sag and encoded vertex displacement. It does not prove that the UV
triangle lies inside the finite CAD trim, or that the mesh covers that trim.
"""
from bisect import bisect_right
from fractions import Fraction as F
from functools import lru_cache
from math import comb, isqrt
import math

from bernstein import bounds, coordinate, derivative, patches
from curve_jets import analytical_surface, periodic_trig
from curve_taylor import Taylor
from spline_embedding import polynomial_surface


def sqrt_upper(x):
    if x < 0:
        raise ValueError("negative squared norm")
    scale = 1 << 80
    root = isqrt(x.numerator*scale*scale//x.denominator)
    return F(root if F(root, scale)**2 == x else root+1, scale)


def norm_upper(box):
    return sqrt_upper(sum(max(abs(lo), abs(hi))**2 for lo, hi in box))


def sqrt_lower(x):
    scale = 1 << 80
    return F(isqrt(x.numerator*scale*scale//x.denominator), scale)


def squared_distance_to_triangle(points):
    """Exact nearest point to the origin, including edges and degenerate triangles."""
    dot = lambda a, b: sum(x*y for x, y in zip(a, b))
    sub = lambda a, b: tuple(x-y for x, y in zip(a, b))
    a, b, c = points
    values = []
    for p, q in ((a, b), (b, c), (c, a)):
        d = sub(q, p)
        dd = dot(d, d)
        t = max(F(0), min(F(1), -dot(p, d)/dd)) if dd else F(0)
        v = tuple(x+t*y for x, y in zip(p, d))
        values.append(dot(v, v))
    ab, ac = sub(b, a), sub(c, a)
    aa, bb, cc = dot(ab, ab), dot(ab, ac), dot(ac, ac)
    determinant = aa*cc-bb*bb
    if determinant > 0:
        d, e = -dot(a, ab), -dot(a, ac)
        s, t = (cc*d-bb*e)/determinant, (aa*e-bb*d)/determinant
        if s >= 0 and t >= 0 and s+t <= 1:
            p = tuple(x+s*y+t*z for x, y, z in zip(a, ab, ac))
            values.append(dot(p, p))
    return min(values)


def sphere_triangle_bound(evaluator, nodes):
    points = [tuple(F(x)-o for x, o in zip(p[2:], evaluator.origin)) for p in nodes]
    minimum = sqrt_lower(squared_distance_to_triangle(points))
    maximum = sqrt_upper(max(sum(x*x for x in p) for p in points))
    # A Gram defect delta < 1 bounds each frame singular value's deviation
    # from 1 by delta. Thus the encoded ellipsoid is within radius*delta of
    # the exact sphere, regardless of chart seams or collapsed polar UVs.
    return max(maximum-evaluator.radius, evaluator.radius-minimum)+evaluator.radius*evaluator.frame_defect


def interpolation_bound(uv, hessian, vertex_error=F(0)):
    """Taylor remainder plus variance bounds, valid over the entire UV triangle."""
    du, dv = [max(p[k] for p in uv)-min(p[k] for p in uv) for k in (0, 1)]
    a, b, c = hessian
    if min(a, b, c, vertex_error) < 0:
        raise ValueError("bounds must be nonnegative")
    return vertex_error+(a*du*du+2*b*du*dv+c*dv*dv)/8


def tensor_value(nets, u, v):
    def basis(n, t):
        return [comb(n, i)*t**i*(1-t)**(n-i) for i in range(n+1)]
    a, b = basis(len(nets[0])-1, u), basis(len(nets[0][0])-1, v)
    weights = [x*y for x in a for y in b]
    return [sum(w*x for w, x in zip(weights, (x for row in net for x in row))) for net in nets]


class Polynomial:
    def __init__(self, record):
        degrees, knots, _ = polynomial_surface(record)
        if any(k.count(x) > d-1 for d, k in zip(degrees, knots)
               for x in set(k) if k[0] < x < k[-1]):
            raise ValueError("whole-triangle Taylor bound requires a C1 surface")
        self.breaks = [sorted(set(k)) for k in knots]
        self.cells = {}
        for patch in patches(record):
            domain = patch["domain"]
            spans = [b-a for a, b in domain]
            nets = [coordinate(patch["poles"], k) for k in range(3)]
            hessian = []
            for i, j in [(0, 0), (0, 1), (1, 1)]:
                boxes = [bounds(derivative(derivative(net, i, spans[i]), j, spans[j]))
                         for net in nets]
                hessian.append(norm_upper(boxes))
            index = tuple(self.breaks[k].index(domain[k][0]) for k in (0, 1))
            self.cells[index] = domain, nets, hessian

    def indices(self, box):
        ranges = []
        for (lo, hi), breaks in zip(box, self.breaks):
            if lo < breaks[0] or hi > breaks[-1] or hi < lo:
                raise ValueError("UV domain leaves the bounded spline")
            first = min(len(breaks)-2, max(0, bisect_right(breaks, lo)-1))
            last = min(len(breaks)-2, max(0, bisect_right(breaks, hi)-1))
            ranges.append(range(first, last+1))
        return [(i, j) for i in ranges[0] for j in ranges[1]]

    @lru_cache(8192)
    def value(self, uv):
        domain, nets, _ = self.cells[self.indices([(x, x) for x in uv])[0]]
        u, v = [(x-a)/(b-a) for x, (a, b) in zip(uv, domain)]
        values = tensor_value(nets, u, v)
        return [(x, x) for x in values]

    def hessian(self, box):
        cells = [self.cells[i][2] for i in self.indices(box)]
        return [max(c[k] for c in cells) for k in range(3)]

    def tangents(self, uv):
        domain, nets, _ = self.cells[self.indices([(x, x) for x in uv])[0]]
        u, v = [(x-a)/(b-a) for x, (a, b) in zip(uv, domain)]
        return [list(map(float, tensor_value(
            [derivative(net, k, domain[k][1]-domain[k][0]) for net in nets], u, v))) for k in (0, 1)]


class Analytical:
    def __init__(self, record):
        self.record = record
        self.kind = record["kind"]
        self.basis = [[F(x) for x in row] for row in record["basis"]]
        self.origin = list(map(F, record["origin"]))
        if (self.kind not in ("plane", "sphere", "cone") or len(self.origin) != 3
                or len(self.basis) != 3 or any(len(v) != 3 for v in self.basis)):
            raise ValueError("unsupported analytical surface")
        # Spectral norm bound for the encoded frame, without assuming that its
        # floating-point axes are exactly orthonormal.
        gram = [[sum(x*y for x, y in zip(a, b)) for b in self.basis] for a in self.basis]
        self.frame_norm = sqrt_upper(max(sum(map(abs, row)) for row in gram))
        self.frame_defect = max(sum(abs(x-(i == j)) for j, x in enumerate(row)) for i, row in enumerate(gram))
        if self.frame_defect >= 1:
            raise ValueError("analytical frame is not sufficiently orthonormal")
        self.radius = F(record.get("radius", 0))
        if self.kind == "sphere" and self.radius <= 0:
            raise ValueError("sphere radius must be positive")
        if self.kind == "cone":
            self.sine = Taylor(record["angle"]).sin_cos()[0].v

    @lru_cache(8192)
    def value(self, uv):
        if self.kind == "plane":
            values = [o+uv[0]*x+uv[1]*y for o, x, y in zip(self.origin, *self.basis[:2])]
            return [(x, x) for x in values]
        return [v.v for v in analytical_surface(self.record, [Taylor(x) for x in uv])]

    def hessian(self, box):
        if self.kind == "plane":
            return [F(0)]*3
        if self.kind == "sphere":
            sv, cv = periodic_trig(Taylor(box[1]))
            return [self.radius*self.frame_norm*x for x in
                    (min(F(1), max(map(abs, cv.v))), min(F(1), max(map(abs, sv.v))), F(1))]
        lo, hi = box[1]
        radius = max(abs(self.radius+v*s) for v in (lo, hi) for s in self.sine)
        return [radius*self.frame_norm, max(map(abs, self.sine))*self.frame_norm, F(0)]

    def tangents(self, uv):
        if self.kind == "plane":
            return [list(map(float, p)) for p in self.basis[:2]]
        return [[float(sum(c.c[1])/2) for c in analytical_surface(self.record,
                    [Taylor.variable(x, 1) if i == k else Taylor(x) for i, x in enumerate(uv)])]
                for k in (0, 1)]


def surface(record):
    return Polynomial(record) if record["kind"] == "polynomial" else Analytical(record)


@lru_cache(32768)
def vertex_error(evaluator, node):
    value = evaluator.value(tuple(node[:2]))
    return norm_upper([(x-hi, x-lo) for x, (lo, hi) in zip(node[2:], value)])


@lru_cache(8192)
def projected_witness(evaluator, node):
    """A floating-point proposal only; exact surface evaluation validates its error."""
    uv = tuple(node[:2])
    target = list(map(float, node[2:]))
    dot = lambda a, b: sum(x*y for x, y in zip(a, b))
    for _ in range(8):
        value = [float((a+b)/2) for a, b in evaluator.value(uv)]
        residual = [a-b for a, b in zip(target, value)]
        u, v = evaluator.tangents(uv)
        aa, ab, bb = dot(u, u), dot(u, v), dot(v, v)
        determinant = aa*bb-ab*ab
        if determinant <= 0:
            break
        a, b = dot(u, residual), dot(v, residual)
        delta = [(bb*a-ab*b)/determinant, (aa*b-ab*a)/determinant]
        improved = False
        for half in range(8):
            candidate = [float(x)+d/2**half for x, d in zip(uv, delta)]
            if not all(math.isfinite(x) for x in candidate):
                continue
            if isinstance(evaluator, Polynomial):
                candidate = [max(float(k[0]), min(float(k[-1]), x)) for x, k in zip(candidate, evaluator.breaks)]
            candidate = tuple(map(F, candidate))
            proposal = [float((a+b)/2) for a, b in evaluator.value(candidate)]
            if sum((a-b)**2 for a, b in zip(target, proposal)) < dot(residual, residual):
                uv, improved = candidate, True
                break
        if not improved or max(map(abs, delta)) < 1e-13:
            break
    return (*uv, *node[2:])


def triangle_bound(evaluator, nodes, target, max_depth=12):
    """Refine the proof over the existing triangle; never modify its geometry."""
    if len(nodes) != 3 or any(len(p) != 5 for p in nodes):
        raise ValueError("expected three UV/XYZ nodes")
    if isinstance(evaluator, Analytical) and evaluator.kind == "sphere":
        upper = sphere_triangle_bound(evaluator, nodes)
        return dict(passes=upper <= target, bound=upper, cells=1)
    stack = [(tuple(tuple(map(F, p)) for p in nodes), 0)]
    maximum, cells = F(0), 0
    while stack:
        points, depth = stack.pop()
        if len(points) != 3 or any(len(p) != 5 for p in points):
            raise ValueError("expected three UV/XYZ nodes")
        errors = [vertex_error(evaluator, p) for p in points]
        uv = [p[:2] for p in points]
        box = [(min(p[k] for p in uv), max(p[k] for p in uv)) for k in (0, 1)]
        upper = interpolation_bound(uv, evaluator.hessian(box), max(errors))
        cells += 1
        if upper <= target:
            maximum = max(maximum, upper)
            continue
        if depth == max_depth or max(errors) > target:
            return dict(passes=False, bound=upper, cells=cells, depth=depth)
        a, b, c = points
        ab, bc, ca = [projected_witness(evaluator, tuple((x+y)/2 for x, y in zip(p, q)))
                      for p, q in ((a, b), (b, c), (c, a))]
        stack.extend((t, depth+1) for t in ((a, ab, ca), (ab, b, bc), (ca, bc, c), (ab, bc, ca)))
    return dict(passes=True, bound=maximum, cells=cells)
