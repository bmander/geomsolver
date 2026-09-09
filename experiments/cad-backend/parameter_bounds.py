"""Finite line/spline parameter-curve bounds, including polynomial end extensions."""
from collections import Counter
from fractions import Fraction as F

from bernstein import refine
from interval_jet import f, point


def polynomial_box(poles, interval):
    t = tuple(F(x) for x in interval)
    other = f.sub(point(1),t)
    values = [[point(x) for x in p] for p in poles]
    while len(values) > 1:
        values = [[f.add(f.mul(other,x),f.mul(t,y)) for x,y in zip(a,b)]
                  for a,b in zip(values,values[1:])]
    return values[0]


def curve_bounds(curve):
    lo,hi = [F(x) for x in curve['interval']]
    if lo >= hi:
        raise ValueError('empty parameter-curve interval')
    if curve['kind'] == 'line':
        origin,direction = [[F(x) for x in curve[k]] for k in ('location','direction')]
        if len(origin) != 2 or len(direction) != 2:
            raise ValueError('expected a planar parameter line')
        return [tuple(sorted((a+lo*b,a+hi*b))) for a,b in zip(origin,direction)],0
    if curve['kind'] != 'bspline':
        raise ValueError('unsupported parameter curve')
    degree = curve['degree']
    knots = [F(x) for x in curve['knots']]
    poles = [tuple(F(x) for x in p) for p in curve['poles']]
    weights = [F(w) for w in curve['weights']]
    multiplicities = Counter(knots)
    if (type(degree) is not int or degree < 1 or len(poles) < degree+1
            or len(knots) != len(poles)+degree+1 or knots != sorted(knots)
            or knots[0] >= knots[-1] or any(len(p) != 2 for p in poles)
            or multiplicities[knots[0]] != degree+1 or multiplicities[knots[-1]] != degree+1
            or any(n > degree for k,n in multiplicities.items() if k not in (knots[0],knots[-1]))
            or len(weights) != len(poles) or any(w != 1 for w in weights)):
        raise ValueError('expected a clamped polynomial parameter spline')
    bounds = [(min(p[i] for p in poles),max(p[i] for p in poles)) for i in (0,1)]
    extended = int(lo < knots[0])+int(hi > knots[-1])
    if extended:
        points, refined = refine(poles,knots,degree)
        unique = sorted(set(refined))
        extensions = []
        if lo < unique[0]:
            extensions.append(polynomial_box(points[:degree+1],((lo-unique[0])/(unique[1]-unique[0]),F(0))))
        if hi > unique[-1]:
            extensions.append(polynomial_box(points[-degree-1:],(F(1),(hi-unique[-2])/(unique[-1]-unique[-2]))))
        for extension in extensions:
            bounds = [(min(a,c),max(b,d)) for (a,b),(c,d) in zip(bounds,extension)]
    return bounds,extended


def face_bounds(record):
    if not record['wires'] or any(not wire for wire in record['wires']):
        raise ValueError('face has no finite wire')
    curves = [curve_bounds(c) for w in record['wires'] for c in w]
    bounds = [(min(b[i][0] for b,_ in curves),max(b[i][1] for b,_ in curves)) for i in (0,1)]
    return bounds,sum(n for _,n in curves),len(curves)
