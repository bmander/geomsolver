"""Exact finite parameter coverage for natural rectangular CAD support faces."""
import argparse
from fractions import Fraction as F
import hashlib
import json
from pathlib import Path


def rectangle(wires):
    if len(wires) != 1 or len(wires[0]) < 4:
        raise ValueError('expected one rectangular wire without holes')
    sides = {(axis,value):[] for axis in (0,1) for value in (F(0),F(1))}
    segments = []
    for edge in wires[0]:
        origin, direction = [[F(x) for x in edge[k]] for k in ('location','direction')]
        lo, hi = [F(x) for x in edge['interval']]
        if len(origin) != 2 or len(direction) != 2 or lo >= hi or type(edge['reversed']) is not bool:
            raise ValueError('invalid parameter line')
        ends = [tuple(o+d*t for o,d in zip(origin,direction)) for t in (lo,hi)]
        if edge['reversed']:
            ends.reverse()
        side = [(axis,ends[0][axis]) for axis in (0,1)
                if ends[0][axis] == ends[1][axis] and ends[0][axis] in (0,1)]
        if len(side) != 1:
            raise ValueError('edge is not on one rectangle side')
        axis, value = side[0]
        span = sorted(p[1-axis] for p in ends)
        if not 0 <= span[0] < span[1] <= 1:
            raise ValueError('edge leaves the rectangle or degenerates')
        sides[(axis,value)].append(span)
        segments.append(ends)
    for intervals in sides.values():
        cursor = F(0)
        for a,b in sorted(intervals):
            if a != cursor:
                raise ValueError('gap or overlap on rectangle side')
            cursor = b
        if cursor != 1:
            raise ValueError('incomplete rectangle side')
    if any(segments[i][1] != segments[(i+1)%len(segments)][0] for i in range(len(segments))):
        raise ValueError('wire is not a directed closed cycle')
    return dict(domain=[[0,1],[0,1]],boundary_sides=4,boundary_segments=len(segments),holes=0)


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def run(path, output):
    data = json.loads(path.read_text())
    if len(data['coefficients']) != 3 or len({r['file'] for r in data['coefficients']}) != 3:
        raise ValueError('expected three distinct coefficient inputs')
    definitions = {}
    for item in data['coefficients']:
        p = Path(item['file'])
        if digest(p) != item['sha256']:
            raise ValueError('changed coefficient input')
        d = json.loads(p.read_text())
        if d['inputs'] != data['inputs']:
            raise ValueError('coefficient and face STEP inputs disagree')
        definitions[str(p)] = d
    if len(data['inputs']) != 2 or {r['member'] for r in data['inputs']} != {0,1}:
        raise ValueError('expected both members')
    for item in data['inputs']:
        if digest(Path(item['step_file'])) != item['step_sha256']:
            raise ValueError('changed STEP input')
    expected = {(m,i) for m in (0,1) for i in range(10)}
    if len(data['faces']) != 20 or {(r['member'],r['face_index']) for r in data['faces']} != expected:
        raise ValueError('incomplete or duplicate face domains')
    results = []
    for face in data['faces']:
        key = face['member'],face['face_index']
        matches = [(p,r) for p,d in definitions.items() for r in d['surfaces']
                   if (r['member'],r['face_index']) == key]
        if len(matches) != 1 or matches[0][0] != face['coefficient_file']:
            raise ValueError('ambiguous bounded face association')
        surface = matches[0][1]['surface']
        if any((k[d],k[-d-1]) != (0,1) for k,d in zip(surface['knots'],surface['degrees'])):
            raise ValueError('bounded support has a different parameter domain')
        results.append(dict(member=key[0],face_index=key[1],**rectangle(face['wires'])))
    report = dict(evidence_sha256=digest(path),status='verified',faces=results,
                  scope='Exact unit-square parameter coverage of twenty extracted tooth-space faces; 3D edge consistency, global material and indexed-solid trimming remain separate')
    output.write_text(json.dumps(report,indent=2)+'\n')
    print(f'Verified {len(results)} complete finite parameter domains',flush=True)
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input',type=Path)
    parser.add_argument('output',type=Path)
    args = parser.parse_args()
    run(args.input,args.output)
