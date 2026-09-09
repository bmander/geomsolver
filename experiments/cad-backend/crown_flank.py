"""Straight generated flanks with a uniquely isolated, differentiated tip trim."""
from fractions import Fraction as F
from functools import lru_cache
import json

from crown_fillet import check_samples as check_common_samples
from crown_geometry import generated, radius, section
from interval_jet import Jet, f, point


def line(record, parameter):
    delta = [Jet(c) for c in record["delta"]]
    return [Jet(c)+d*parameter for c, d in zip(record["start"], delta)], delta


def tip_height(record, rho):
    a, b = [[F(x) for x in p] for p in record["tip_meridian"]]
    A, B = b[2]-a[2], b[0]-a[0]
    C = A*a[0]-B*a[2]
    if A <= 0 or a[1] != 0 or b[1] != 0:
        raise ValueError("unsupported tip-cone branch")
    norm2 = A*A+B*B
    z = (-B*C+A*(norm2*rho*rho-C*C).sqrt())/norm2
    if z.v[0] <= 0 or (B*z+C).v[0] <= 0:
        raise ValueError("unproved positive cone-sphere intersection")
    return z


def residual(data, record, q, parameter):
    rho = radius(data, q)
    profile, derivative = line(record, parameter)
    r, h, dr, dh, ct, st, x, y, A, B = section(data, profile, derivative, rho)
    if A.v[0] <= 0:
        raise ValueError("unproved positive characteristic branch")
    offset = h*dh/dr
    planar_square = rho*rho-h*h
    radial_dot = r+data["crown_center"][0]*ct+data["crown_center"][1]*st
    rotated_x = (planar_square+offset*radial_dot)/(planar_square+2*offset*radial_dot+offset*offset).sqrt()
    member = record["member"]
    n, g = data["teeth"][member], data["teeth"][1-member]
    size = (Jet(n)*n+Jet(g)*g).sqrt()
    z = (g*rotated_x+(1 if member == 0 else -1)*n*h)/size
    return z-tip_height(record, rho)


def sign(value):
    return -1 if value[1] < 0 else 1 if value[0] > 0 else 0


def residual_box(data, record, domain, parameter):
    """Mean-value enclosure removes cancellation between the two axial heights."""
    middle = sum(domain)/2
    center = residual(data, record, Jet(middle), Jet(parameter)).v
    if domain[0] == domain[1]:
        return center
    box = residual(data, record, Jet.variable(domain, 0), Jet(parameter))
    offset = domain[0]-middle, domain[1]-middle
    value = f.add(center, f.mul(box.d[0], offset))
    return max(value[0], box.v[0]), min(value[1], box.v[1])


@lru_cache(1024)
def tip_function(encoded, domain, order):
    data, record = json.loads(encoded)
    q = Jet(domain)
    middle = Jet(sum(domain)/2)
    lo, hi = F(0), F(1)
    slo = sign(residual(data, record, middle, Jet(lo)).v)
    shi = sign(residual(data, record, middle, Jet(hi)).v)
    if slo*shi != -1:
        raise ValueError("tip is not bracketed within the source line")
    for _ in range(26):
        mid = (lo+hi)/2
        sm = sign(residual(data, record, middle, Jet(mid)).v)
        if sm == 0:
            break
        if sm == slo:
            lo = mid
        else:
            hi = mid
    guess = (lo+hi)/2
    half = F(1, 64)
    for _ in range(7):
        lo, hi = max(F(0), guess-half), min(F(1), guess+half)
        left = sign(residual_box(data, record, domain, lo))
        right = sign(residual_box(data, record, domain, hi))
        if left*right == -1:
            slope = residual(data, record, q, Jet.variable((lo, hi), 1)).d[1]
            if sign(slope) == right:
                break
        half *= 2
    else:
        raise ValueError("whole-interval tip existence and uniqueness unproved")
    # Uniform endpoint signs and a nonzero partial derivative prove one root for
    # every q and coefficient choice. Interval Newton retains all those roots.
    for _ in range(12):
        mid = (lo+hi)/2
        slope = residual(data, record, q, Jet.variable((lo, hi), 1)).d[1]
        image = f.sub(point(mid), f.div(residual_box(data, record, domain, mid), slope))
        new = max(lo, image[0]), min(hi, image[1])
        if new[0] > new[1]:
            raise ValueError("tip interval contraction lost its proved root")
        old_width = hi-lo
        lo, hi = new
        if hi-lo >= old_width*F(99, 100):
            break
    if order == 0:
        return (lo, hi), None, None, None
    j = residual(data, record, Jet.variable(domain, 0, order), Jet.variable((lo, hi), 1, order))
    d = f.neg(f.div(j.d[0], j.d[1]))
    h = f.neg(f.div(f.add(j.h[0], f.add(f.mul(point(2), f.mul(j.h[1], d)),
                                                f.mul(j.h[2], f.square(d)))), j.d[1]))
    t = None
    if order == 3:
        numerator = j.t[0]
        for scale, value in ((3, f.mul(j.t[1], d)), (3, f.mul(j.t[2], f.square(d))),
                             (1, f.mul(j.t[3], f.mul(f.square(d), d))),
                             (3, f.mul(f.add(j.h[1], f.mul(j.h[2], d)), h))):
            numerator = f.add(numerator, f.mul(point(scale), value))
        t = f.neg(f.div(numerator, j.d[1]))
    return (lo, hi), d, h, t


def evaluate(data, record, u, v):
    # The key includes the full numeric definition, not object identity or q alone.
    encoded = json.dumps([data, record], sort_keys=True, separators=(",", ":"))
    value, d, h, t = tip_function(encoded, u.v, 0 if u.constant() else 3 if u.t is not None else 2)
    tip = Jet(value) if u.constant() else u.compose(value, d, h, t)
    parameter = record["join_parameter"]+v*(tip-record["join_parameter"])
    if parameter.v[0] < 0 or parameter.v[1] > 1:
        raise ValueError("unproved finite source-line trim")
    profile, derivative = line(record, parameter)
    return generated(data, record, profile, derivative, radius(data, u))


def check_samples(data, source, record):
    return check_common_samples(data, source, record, evaluate, 32)
