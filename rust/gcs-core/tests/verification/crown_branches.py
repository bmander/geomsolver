#!/usr/bin/env python3
"""Independently audit nominal circular-crown branch and regularity bounds.

The generating-profile enclosures are inputs. This checker does not re-solve
Solvent or prove the solved 3D assembly equals the ideal circular-crown system.
It verifies the complete rectangle cover and recomputes each branch inequality
using exact rational interval arithmetic, rational sqrt enclosures and a separate
high-degree Taylor enclosure. No Rust library or floating-point libm is used.
"""
from copy import deepcopy
from fractions import Fraction as F
from functools import lru_cache
from math import isqrt
from pathlib import Path
import json
import sys
from interval_arithmetic import reference_trig


def interval(values):
    a, b = map(F.from_float, map(float, values))
    assert a <= b
    return a, b


def point(value):
    x = F.from_float(float(value))
    return x, x


def add(a, b): return a[0]+b[0], a[1]+b[1]
def neg(a): return -a[1], -a[0]
def sub(a, b): return add(a, neg(b))
def mul(a, b):
    values = [x*y for x in a for y in b]
    return min(values), max(values)
def div(a, b):
    assert not b[0] <= 0 <= b[1]
    values = [x/y for x in a for y in b]
    return min(values), max(values)
def square(a):
    return (F(0) if a[0] <= 0 <= a[1] else min(x*x for x in a)), max(x*x for x in a)
def sqrt(a):
    assert a[0] >= 0
    scale = 1 << 200
    def lower(x): return F(isqrt((x.numerator*scale*scale)//x.denominator), scale)
    return lower(a[0]), lower(a[1])+F(1, scale)


@lru_cache(None)
def tangent(x):
    assert abs(x) < 1  # This proof uses the declared +/-35-degree roll interval.
    return div(reference_trig(x, False), reference_trig(x, True))


def surface_factor(profile, center, theta, cot):
    r, h, dr, dh, ddr, ddh = profile
    cx, cy = center
    ct, st = theta
    d = add(mul(dr, r), mul(h, dh))
    a, b = neg(add(mul(dr, cx), mul(d, ct))), neg(add(mul(dr, cy), mul(d, st)))
    norm2 = add(square(a), square(b))
    assert a[0] > 0 or a[1] < 0
    sign = point(1 if a[0] > 0 else -1)
    g, k = add(mul(cx, ct), mul(cy, st)), sub(mul(cy, ct), mul(cx, st))
    curvature = sub(mul(dr, ddh), mul(dh, ddr))
    e = add(mul(dr, add(square(dr), square(dh))), mul(h, curvature))
    numerator = add(mul(e, square(k)), mul(div(d, r), square(add(mul(dr, g), d))))
    return add(point(1), div(mul(mul(mul(cot, sign), h), numerator), mul(norm2, sqrt(norm2))))


def margins(center, roll, teeth, member, cell):
    cx, cy = map(point, center)
    profile = list(map(interval, cell['profile']))
    r, h, dr, dh, _, _ = profile
    rho = interval(cell['rho'])
    assert r[0] > 0
    c2 = add(square(cx), square(cy))
    c = sqrt(c2)
    q = div(sub(sub(sub(square(rho), square(h)), c2), square(r)), mul(point(2), mul(c, r)))
    discriminant = sub(point(1), square(q))
    root = sqrt(discriminant)
    ct = div(add(mul(cx, q), mul(cy, root)), c)
    st = div(sub(mul(cy, q), mul(cx, root)), c)
    other_st = div(add(mul(cy, q), mul(cx, root)), c)
    px, py = add(cx, mul(r, ct)), add(cy, mul(r, st))
    a = sub(mul(neg(dr), px), mul(mul(h, dh), ct))
    b = sub(mul(neg(dr), py), mul(mul(h, dh), st))
    ratio = div(neg(b), a)
    assert member in (0, 1) and all(isinstance(n, int) and n > 0 for n in teeth)
    # Keep the integer tooth ratio exact, including ratios not representable as a double.
    cot = (F((1 if member == 0 else -1)*teeth[1-member], teeth[member]),)*2
    jacobian = surface_factor(profile, [cx, cy], [ct, st], cot)
    declared = interval(cell['area_factor'])
    assert declared[0] <= jacobian[0] <= jacobian[1] <= declared[1], 'area-factor enclosure'
    assert not declared[0] <= 0 <= declared[1], 'area-factor enclosure contains a singularity'
    assert 0 < F.from_float(cell['margins'][6]) <= (declared[0] if declared[0] > 0 else -declared[1])
    return [discriminant[0], ct[0], min(-st[1], other_st[0]),
            a[0] if a[0] > 0 else -a[1], add(square(dr), square(dh))[0],
            min(ratio[0]-tangent(F.from_float(roll[0]))[1],
                tangent(F.from_float(roll[1]))[0]-ratio[1]),
            jacobian[0] if jacobian[0] > 0 else -jacobian[1]]


def cover(domain, cells):
    (u0, u1), (r0, r1) = map(interval, domain)
    assert u0 < u1 and r0 < r1 and cells
    rectangles = [(interval(c['u']), interval(c['rho'])) for c in cells]
    for (a, b), (c, d) in rectangles:
        assert u0 <= a < b <= u1 and r0 <= c < d <= r1
    # Exact sweep slabs, not an area sum: overlap cannot cancel a missing cell.
    cuts = sorted({u0, u1} | {x for u, _ in rectangles for x in u})
    for a, b in zip(cuts, cuts[1:]):
        spans = sorted(r for u, r in rectangles if u[0] <= a and b <= u[1])
        end = r0
        for c, d in spans:
            assert c == end, 'gap or overlapping cells'
            end = d
        assert end == r1, 'incomplete domain cover'


def check(records):
    assert records
    minimum = [None]*7
    cell_count = 0
    orientations = []
    for record in records:
        cover(record['domain'], record['cells'])
        signs = set()
        for cell in record['cells']:
            actual = margins(record['center'], record['roll'], record['teeth'], record['member'], cell)
            claimed = list(map(F.from_float, cell['margins']))
            assert len(claimed) == 7
            for i, (a, b) in enumerate(zip(actual, claimed)):
                assert 0 < b <= a, (record['patch'], i, float(a), float(b))
                minimum[i] = b if minimum[i] is None else min(minimum[i], b)
            cell_count += 1
            signs.add(1 if cell['area_factor'][0] > 0 else -1)
        assert len(signs) == 1, 'a connected regular patch must have one orientation'
        orientations.append(signs.pop())
    return {'domains': len(records), 'cells': cell_count,
            'minimum_reported_margins': [float(x) for x in minimum],
            'positive_orientation_domains': orientations.count(1),
            'negative_orientation_domains': orientations.count(-1)}


def selftest():
    base = {'u': [0., 1.], 'rho': [1., 2.]}
    cover([[0., 1.], [1., 2.]], [base])
    for cells in [[], [base, base], [{'u': [0., .5], 'rho': [1., 2.]}]]:
        try:
            cover([[0., 1.], [1., 2.]], cells)
        except AssertionError:
            pass
        else:
            raise AssertionError('invalid cover accepted')


if __name__ == '__main__':
    assert len(sys.argv) == 2, 'provide the exported branch cover JSON'
    selftest()
    records = json.loads(Path(sys.argv[1]).read_text())
    result = check(records)
    for field, index, value in [('margins', 0, 2.), ('margins', 6, 1e300),
                                ('area_factor', 0, 1e300), ('area_factor', 0, -1e300)]:
        bad = deepcopy(records[:1])
        bad[0]['cells'][0][field][index] = value
        try:
            check(bad)
        except AssertionError:
            pass
        else:
            raise AssertionError('overstated branch or regularity bound accepted')
    print(json.dumps(result, indent=2))
