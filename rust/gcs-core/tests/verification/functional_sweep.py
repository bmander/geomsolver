#!/usr/bin/env python3
"""Independent rational-interval audit of functional sweep upper-bound witnesses.

Reads the complete numeric generator/motion definitions from the experiment JSON.
No Rust, floating-point libm or source sketch is used. This checks the attained
upper bounds, NOT the global lower bounds or the source-solve error transfer.
"""
from copy import deepcopy
from functools import lru_cache
from pathlib import Path
import json
import sys
import crown_branches as arithmetic
from interval_arithmetic import reference_trig

F = arithmetic.F
point = arithmetic.point
interval = arithmetic.interval
SCALE = 1 << 200


def rounded(a):
    # Keep rational expressions bounded in size, rounding outward to a dyadic grid.
    def floor(x): return (x.numerator*SCALE)//x.denominator
    return F(floor(a[0]), SCALE), F(-floor(-a[1]), SCALE)


def add(a,b): return rounded(arithmetic.add(a,b))
def sub(a,b): return rounded(arithmetic.sub(a,b))
def mul(a,b): return rounded(arithmetic.mul(a,b))
def div(a,b): return rounded(arithmetic.div(a,b))
def square(a): return rounded(arithmetic.square(a))
def neg(a): return arithmetic.neg(a)
def minimum(a,b): return min(a[0],b[0]), min(a[1],b[1])
def maximum(a,b): return max(a[0],b[0]), max(a[1],b[1])
def dot(a,b):
    out = point(0)
    for x,y in zip(a,b): out = add(out,mul(x,y))
    return out
def norm(v):
    out = point(0)
    for x in v: out = add(out,square(x))
    return arithmetic.sqrt(out)
def unit(v):
    return normalized(tuple(v))


@lru_cache(512)
def normalized(v):
    length = norm(v)
    return tuple(div(x,length) for x in v)
def vec(v): return list(map(point,v))


@lru_cache(None)
def trig(angle):
    assert abs(angle) < 8
    return rounded(reference_trig(angle,False)), rounded(reference_trig(angle,True))


def rotation(p,definition,t,inverse=False):
    origin = vec(definition['origin'])
    axis = unit(vec(definition['axis']))
    q = [sub(x,o) for x,o in zip(p,origin)]
    theta = F.from_float(definition['phase'])+F.from_float(definition['ratio'])*F.from_float(t)
    if inverse: theta = -theta
    sine, cosine = trig(theta)
    cross = [sub(mul(axis[1],q[2]),mul(axis[2],q[1])),
             sub(mul(axis[2],q[0]),mul(axis[0],q[2])),
             sub(mul(axis[0],q[1]),mul(axis[1],q[0]))]
    axial = mul(dot(axis,q),sub(point(1),cosine))
    return [add(o,add(add(mul(cosine,x),mul(sine,c)),mul(a,axial)))
            for o,x,c,a in zip(origin,q,cross,axis)]


def half_plane(p,through,normal):
    return dot([sub(x,o) for x,o in zip(p,vec(through))],unit(vec(normal)))


def profile(p,definition):
    if 'disk' in definition:
        disk = definition['disk']
        assert disk['radius'] > 0
        return sub(norm([sub(x,o) for x,o in zip(p,vec(disk['center']))]),point(disk['radius']))
    if 'half_plane' in definition:
        plane = definition['half_plane']
        return half_plane(p,plane['through'],plane['normal'])
    if 'intersection' in definition:
        values = [profile(p,d) for d in definition['intersection']]
        assert values
        result = values[0]
        for value in values[1:]: result = maximum(result,value)
        return result
    if 'difference' in definition:
        assert len(definition['difference']) == 2
        a,b = definition['difference']
        return maximum(profile(p,a),neg(profile(p,b)))
    values = [half_plane(p,h['through'],h['normal']) for h in definition['half_planes']]
    for corner in definition['rounded_corners']:
        center = corner['center']
        disk = sub(norm([sub(x,o) for x,o in zip(p,vec(center))]),point(corner['radius']))
        for normal in corner['sector_normals']:
            disk = minimum(disk,half_plane(p,center,normal))
        values.append(disk)
    assert values
    result = values[0]
    for v in values[1:]: result = maximum(result,v)
    return result


def field(p,definition):
    if 'intersection' in definition:
        values = [field(p,d) for d in definition['intersection']]
        assert values, 'empty intersection'
        result = values[0]
        for value in values[1:]: result = maximum(result,value)
        return result
    if 'under' in definition:
        assert definition['under']['ratio'] == 0, 'field transform must be static'
        return field(rotation(p,definition['under'],0,inverse=True),definition['field'])
    axis = unit(vec(definition['axis']))
    q = [sub(x,o) for x,o in zip(p,vec(definition['origin']))]
    z = dot(q,axis)
    radial = norm([sub(x,mul(z,a)) for x,a in zip(q,axis)])
    if 'profile' in definition:
        return profile([radial,z],definition['profile'])
    # Schema 1 contains one profile directly; schema 2 explicitly lists the
    # profiles whose union revolves about this shared axis.
    profiles = definition.get('profiles',[definition])
    assert profiles, 'no generating profiles'
    values = [profile([radial,z],s) for s in profiles]
    result = values[0]
    for value in values[1:]: result = minimum(result,value)
    return result


def check_case(data):
    assert data['schema'] in [1,2,3] and data['units'] == {'length':'mm','angle':'rad'}
    rows = data['points']
    assert rows, 'no sweep witnesses'
    covered = []
    for row in rows:
        declared = interval(row['minimum'])
        t = row['roll_witness']
        lo, hi = row['roll_domain']
        assert lo <= t <= hi, 'witness outside declared motion'
        assert declared[1]-declared[0] <= point(data['value_tolerance_mm'])[0], 'enclosure width'
        # M=observer^-1*source, hence M^-1*x=source^-1*observer*x.
        p = rotation(vec(row['position_mm']),data['motion_definition']['observer'],t)
        p = rotation(p,data['motion_definition']['source'],t,inverse=True)
        actual = field(p,data['definition'])
        assert declared[0] <= actual[0] <= actual[1] <= declared[1], (
            'witness enclosure',row['surface'],row['offset_mm'],
            tuple(map(float,actual)),row['minimum'])
        if 'index' in row and row['offset_mm'] > 0 and actual[1] < 0:
            # One attained negative value proves this indexed candidate cuts
            # the point, independently of the global minimum's lower bound.
            covered.append({'member':data['member'],'surface':row['surface'],
                'index':row['index'],'roll':t,'attained':list(map(float,actual))})
    return {'witnesses':len(rows),'arithmetic':'outward rational intervals',
            'scope':'attained upper bounds; global lower bounds and source error not audited',
            'covered_material_witnesses':covered}


def cases(data):
    if data['schema'] == 1:
        return [data]
    assert data['schema'] in [2,3]
    members = [case['member'] for case in data['cases']]
    if data['schema'] == 2:
        assert sorted(members) == ['gear','pinion'], 'expected exactly one case for each member'
    else:
        assert members and len(set(members)) == len(members) and set(members) <= {'pinion','gear'}, 'invalid member cases'
    return data['cases']


def check(data):
    results = [check_case(case) for case in cases(data)]
    result = dict(results[0])
    result['witnesses'] = sum(r['witnesses'] for r in results)
    result['covered_material_witnesses'] = [w for r in results for w in r['covered_material_witnesses']]
    if data['schema'] in [2,3]:
        result['members'] = {c['member']:r['witnesses'] for c,r in zip(cases(data),results)}
    return result


def negative_controls(data):
    rejected = []
    for case in cases(data):
        member = case.get('member','pinion')
        # The false upper bound is still an ordered, sufficiently narrow
        # interval. Its independently evaluated witness must expose the error.
        for change in ['upper','domain']:
            bad = deepcopy(case)
            bad['points'] = [next(r for r in bad['points'] if r['offset_mm'] > 0)]
            if change == 'upper': bad['points'][0]['minimum'] = [-0.5001,-0.5]
            else: bad['points'][0]['roll_witness'] = bad['points'][0]['roll_domain'][1]+0.1
            try:
                check_case(bad)
            except AssertionError:
                rejected.append(member+':'+change)
            else:
                raise AssertionError('false witness accepted: '+member+':'+change)
        if member == 'gear' and case['schema'] < 3:
            for removed in ['gear_outer','gear_inner']:
                bad = deepcopy(case)
                bad['definition']['profiles'] = [p for p in bad['definition']['profiles'] if p['name'] != removed]
                bad['points'] = [next(r for r in bad['points'] if removed in r['surface'] and r['offset_mm'] > 0)]
                # Exercise field evaluation, not a schema check on names/count:
                # omitting either section must change an attained flank value.
                try:
                    check_case(bad)
                except AssertionError:
                    rejected.append('omitted:'+removed)
                else:
                    raise AssertionError('omitted generator accepted: '+removed)
        if case['schema'] == 3:
            for removed,side in [(0,'gear_outer_inner'),(1,'gear_inner_outer')]:
                bad = deepcopy(case)
                bad['definition'] = bad['definition']['intersection'][1-removed]
                bad['points'] = [next(r for r in bad['points'] if r['surface'] == 'pair.'+side and r['offset_mm'] > 0)]
                try:
                    check_case(bad)
                except AssertionError:
                    rejected.append('omitted_boundary:'+side)
                else:
                    raise AssertionError('omitted tooth-space boundary accepted: '+side)
    return rejected


if __name__ == '__main__':
    assert len(sys.argv) == 2, 'provide the functional sweep JSON'
    data = json.loads(Path(sys.argv[1]).read_text())
    result = check(data)
    result['rejected_controls'] = negative_controls(data)
    print(json.dumps(result,indent=2))
