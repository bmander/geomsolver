"""Whole-support rotation transfer and finite domains for the indexed STEP splines."""
import argparse
from fractions import Fraction as F
from functools import lru_cache
import json
from pathlib import Path
import time

from check_face_domains import digest, rectangle, run as check_domains
from check_supports import upper
from interval_jet import atan_point, f, point, sin_cos


@lru_cache(128)
def rotation(index, teeth):
    if type(index) is not int or type(teeth) is not int or not 0 <= index < teeth:
        raise ValueError('invalid tooth index')
    signed = index if 2*index <= teeth else index-teeth
    # atan(1)=pi/4, with an independent rational remainder enclosure. This
    # bounds exact integer indexing, including the backend angle's rounding.
    return sin_cos(f.mul(atan_point(F(1)),point(F(8*signed,teeth))))


def rotation_bound(reference, surface, index, teeth):
    for key in ('degrees','knots','weights'):
        if reference[key] != surface[key]:
            raise ValueError('indexed surface has a changed basis')
    if any(w != 1 for row in surface['weights'] for w in row):
        raise ValueError('expected polynomial surfaces')
    if ([len(row) for row in reference['poles']] != [len(row) for row in surface['poles']]
            or [len(row) for row in surface['weights']] != [len(row) for row in surface['poles']]):
        raise ValueError('incompatible control nets')
    sine,cosine = rotation(index,teeth)
    maximum = F(0)
    for a,b in zip(reference['poles'],surface['poles']):
        for original,actual in zip(a,b):
            if len(original) != 3 or len(actual) != 3:
                raise ValueError('expected spatial poles')
            x,y,z = [point(v) for v in original]
            expected = (f.sub(f.mul(cosine,x),f.mul(sine,y)),
                        f.add(f.mul(sine,x),f.mul(cosine,y)),z)
            square = sum(max(abs(v) for v in f.sub(point(p),q))**2 for p,q in zip(actual,expected))
            maximum = max(maximum,square)
    # Equal B-spline bases form a nonnegative partition of unity. Their
    # difference is a convex combination of rotated pole differences.
    return f.arithmetic.sqrt(point(maximum))[1]


def run(path, domains, output):
    started = time.perf_counter()
    data, bounded = [json.loads(p.read_text()) for p in (path,domains)]
    check_domains(domains,output.with_suffix('.domains.json'))
    source = Path(data['source_file'])
    if digest(source) != data['source_sha256']:
        raise ValueError('changed source')
    members = json.loads(source.read_text())['members']
    if len(data['inputs']) != 2 or {r['member'] for r in data['inputs']} != {0,1}:
        raise ValueError('expected both indexed members')
    definitions = {r['file']:json.loads(Path(r['file']).read_text()) for r in bounded['coefficients']}
    if any(d['source_sha256'] != data['source_sha256'] for d in definitions.values()):
        raise ValueError('indexed and bounded surfaces have different sources')
    results = []
    for item in data['inputs']:
        m,n = item['member'],item['teeth']
        if members[m]['member'] != m or members[m]['teeth'] != n:
            raise ValueError('changed member indexing')
        for label in ('step','tool'):
            if digest(Path(item[f'{label}_file'])) != item[f'{label}_sha256']:
                raise ValueError('changed indexed STEP input')
        original, = [r for r in bounded['inputs'] if r['member']==m]
        if original['step_sha256'] != item['tool_sha256']:
            raise ValueError('indexed tool differs from bounded tooth space')
        if len(item['references']) != 10:
            raise ValueError('incomplete reference supports')
        retained = set()
        for face in bounded['faces']:
            if face['member'] != m:
                continue
            d = definitions[face['coefficient_file']]
            ref, = [r for r in d['surfaces'] if (r['member'],r['face_index']) == (m,face['face_index'])]
            if ref['surface'] != item['references'][face['face_index']]:
                raise ValueError('indexed reference differs from bounded coefficients')
            if 'side' in ref or ref.get('role') == 'root':
                retained.add(face['face_index'])
        if len(retained) != 5:
            raise ValueError('expected four generated supports and one root closure')
        actual = [r for r in data['surfaces'] if r['member']==m]
        expected = {(i,j) for i in range(n) for j in retained}
        if len(actual) != len(expected) or {(r['index'],r['reference_face']) for r in actual} != expected:
            raise ValueError('missing or duplicated indexed support')
        all_ids = [r['face_index'] for r in actual+item['other_faces']]
        if set(all_ids) != set(range(len(all_ids))) or len(set(all_ids)) != len(all_ids):
            raise ValueError('missing or duplicated indexed face inventory')
        for record in actual:
            bound = rotation_bound(item['references'][record['reference_face']],record['surface'],record['index'],n)
            domain = rectangle(record['wires'])
            if bound > F('0.00000001'):
                raise ValueError('indexed support transfer exceeds 1e-8 mm')
            results.append(dict(member=m,face_index=record['face_index'],index=record['index'],
                reference_face=record['reference_face'],rotation_transfer_bound_mm=upper(bound),**domain))
    if len(results) != len(data['surfaces']):
        raise ValueError('unrecognized member surfaces')
    result = dict(evidence_sha256=digest(path),domains_sha256=digest(domains),status='verified',
        surfaces=results,maximum_transfer_bound_mm=max(r['rotation_transfer_bound_mm'] for r in results),
        seconds=time.perf_counter()-started,
        scope='Complete indexed B-spline supports versus exact rotations of bounded tooth-space coefficients, plus full finite parameter coverage. Analytical blank faces, 3D edge consistency, source/reader error and global material/mating remain separate')
    output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='surfaces'}),flush=True)
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('input','domains','output'):
        parser.add_argument(name,type=Path)
    args = parser.parse_args()
    run(args.input,args.domains,args.output)
