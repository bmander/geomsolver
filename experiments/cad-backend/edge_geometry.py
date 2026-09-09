"""Exact polynomial boundary curves extracted from clamped B-spline faces."""
from fractions import Fraction as F

from interval_jet import f, point


def iso_curve(surface, parameter):
    if parameter['kind'] != 'line':
        raise ValueError('expected a straight isoparametric boundary')
    origin,direction = [[F(x) for x in parameter[k]] for k in ('location','direction')]
    varying = [i for i,x in enumerate(direction) if x != 0]
    if len(origin) != 2 or len(direction) != 2 or len(varying) != 1:
        raise ValueError('parameter line is not isoparametric')
    axis, = varying
    fixed = 1-axis
    degree = surface['degrees'][fixed]
    endpoints = [F(surface['knots'][fixed][degree]),F(surface['knots'][fixed][-degree-1])]
    if any(surface['knots'][fixed].count(v)!=degree+1 for v in endpoints):
        raise ValueError('face boundary is not clamped')
    if origin[fixed] not in endpoints:
        raise ValueError('parameter line is not a natural face boundary')
    last = origin[fixed] == endpoints[1]
    index = -1 if last else 0
    poles = [row[index] for row in surface['poles']] if axis == 0 else surface['poles'][index]
    weights = [row[index] for row in surface['weights']] if axis == 0 else surface['weights'][index]
    knots = [(F(k)-origin[axis])/direction[axis] for k in surface['knots'][axis]]
    if direction[axis] < 0:
        poles,weights,knots = list(reversed(poles)),list(reversed(weights)),list(reversed(knots))
    return dict(kind='bspline',degree=surface['degrees'][axis],poles=poles,weights=weights,knots=knots,
                interval=parameter['interval'])


def same_basis_bound(a,b):
    if a['kind'] != 'bspline' or b['kind'] != 'bspline':
        raise ValueError('expected polynomial spline curves')
    for key in ('degree','knots','weights','interval'):
        if a[key] != b[key]:
            raise ValueError('boundary curves have different parameter bases')
    degree = a['degree']
    knots = [F(k) for k in a['knots']]
    if (type(degree) is not int or degree<1 or len(a['poles']) != len(b['poles']) or len(a['weights']) != len(a['poles'])
            or len(knots) != len(a['poles'])+degree+1 or any(w != 1 for w in a['weights'])
            or not knots[degree] <= F(a['interval'][0]) < F(a['interval'][1]) <= knots[-degree-1]
            or knots != sorted(knots) or knots.count(knots[0]) != degree+1
            or knots.count(knots[-1]) != degree+1):
        raise ValueError('invalid finite polynomial curve basis')
    squared = []
    for p,q in zip(a['poles'],b['poles']):
        if len(p) != 3 or len(q) != 3:
            raise ValueError('expected spatial poles')
        squared.append(sum((F(x)-F(y))**2 for x,y in zip(p,q)))
    return f.arithmetic.sqrt(point(max(squared)))[1]
