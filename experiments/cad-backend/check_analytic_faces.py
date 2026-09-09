"""Independent complete support-distance bounds for the indexed blank faces."""
import argparse
from fractions import Fraction as F
import json
from pathlib import Path
import time

from check_face_domains import digest
from check_supports import upper
from interval_jet import f, point, sin_cos
from parameter_bounds import face_bounds


def sphere_bound(record, radius):
    basis = [[F(x) for x in p] for p in record['basis']]
    if len(record['origin']) != 3 or len(basis) != 3 or any(len(p) != 3 for p in basis):
        raise ValueError('invalid sphere frame')
    if any(sum(a*b for a,b in zip(basis[i],basis[j])) != int(i==j) for i in range(3) for j in range(3)):
        raise ValueError('sphere frame is not exactly orthonormal')
    actual,nominal = F(record['radius']),F(radius)
    if actual <= 0 or nominal <= 0:
        raise ValueError('nonpositive sphere radius')
    return f.arithmetic.sqrt(point(sum(F(x)**2 for x in record['origin'])))[1]+abs(actual-nominal)


def cone_bound(record, meridian, v):
    if (len(record['origin']) != 3 or record['basis'] != [[1,0,0],[0,1,0],[0,0,1]]
            or record['origin'][:2] != [0,0]):
        raise ValueError('expected an axial member-coordinate cone')
    a,b = [[F(x) for x in p] for p in meridian]
    if a[1] != 0 or b[1] != 0 or min(a[0],b[0]) <= 0:
        raise ValueError('invalid nominal cone meridian')
    A,B = b[2]-a[2],b[0]-a[0]
    C = A*a[0]-B*a[2]
    if A <= 0:
        raise ValueError('unsupported nominal cone branch')
    sine,cosine = sin_cos(point(record['angle']))
    radial = f.add(point(record['radius']),f.mul(v,sine))
    constant = A*F(record['radius'])-B*F(record['origin'][2])-C
    slope = f.sub(f.mul(point(A),sine),f.mul(point(B),cosine))
    residual = f.add(point(constant),f.mul(v,slope))
    magnitude = max(abs(x) for x in residual)
    norm2 = A*A+B*B
    # The nearest point on the rz line must retain positive radius, so the
    # meridian distance is a distance to the intended cone nappe.
    if radial[0]-abs(A)*magnitude/norm2 <= 0:
        raise ValueError('unproved positive projected cone radius')
    return magnitude/f.arithmetic.sqrt(point(norm2))[0]


def run(path, output):
    started = time.perf_counter()
    data = json.loads(path.read_text())
    loaded = {}
    for label in ('indexed','supports'):
        p = Path(data[f'{label}_file'])
        if digest(p) != data[f'{label}_sha256']:
            raise ValueError('changed analytical-face reference')
        loaded[label] = json.loads(p.read_text())
    indexed,nominal = loaded['indexed'],loaded['supports']
    source = Path(indexed['source_file'])
    if digest(source) != indexed['source_sha256']:
        raise ValueError('changed source')
    solved = json.loads(source.read_text())
    if (len(nominal['members']) != 2 or [m['member'] for m in solved['members']] != [0,1]
            or solved['module_mm'] != nominal['module_mm']
            or nominal['sphere_radii_mm'] != [.9*solved['mean_distance'],1.1*solved['mean_distance']]):
        raise ValueError('different nominal blank parameters')
    for a,b in zip(solved['members'],nominal['members']):
        if any(a[k] != b[k] for k in ('member','teeth','blank_meridian')):
            raise ValueError('different nominal blank meridian')
    expected = {}
    if len(indexed['inputs']) != 2 or {r['member'] for r in indexed['inputs']} != {0,1}:
        raise ValueError('incomplete indexed inputs')
    for item in indexed['inputs']:
        if digest(Path(item['step_file'])) != item['step_sha256']:
            raise ValueError('changed indexed STEP')
        for face in item['other_faces']:
            key = item['member'],face['face_index']
            if key in expected:
                raise ValueError('duplicate analytical face inventory')
            expected[key] = {'Geom_SphericalSurface':'sphere','Geom_ConicalSurface':'cone'}[face['kind']]
    if len(data['faces']) != len(expected) or {(r['member'],r['face_index']) for r in data['faces']} != set(expected):
        raise ValueError('incomplete or duplicate analytical faces')
    results = []
    for record in data['faces']:
        m = record['member']
        if expected[(m,record['face_index'])] != record['kind']:
            raise ValueError('changed analytical face type')
        domain,extensions,count = face_bounds(record)
        if record['kind'] == 'sphere':
            candidates = [(name,sphere_bound(record,r)) for name,r in zip(('toe','heel'),nominal['sphere_radii_mm'])]
        else:
            candidates = [(name,cone_bound(record,nominal['members'][m]['cones_tip_root_back'][i],domain[1]))
                          for name,i in (('tip',0),('back',2))]
        accepted = [(name,bound) for name,bound in candidates if bound <= F('.001')]
        if len(accepted) != 1:
            raise ValueError('analytical face lacks a unique nominal support within target')
        name,bound = accepted[0]
        results.append(dict(member=m,face_index=record['face_index'],role=name,bound_mm=upper(bound),
            parameter_bounds=[[str(x) for x in d] for d in domain],parameter_curves=count,polynomial_extensions=extensions))
    report = dict(evidence_sha256=digest(path),status='verified',target_mm=.001,faces=results,
        maximum_bound_mm=max(r['bound_mm'] for r in results),seconds=time.perf_counter()-started,
        scope='Complete analytical-support distance over the finite parameter slab enclosed by all trim curves. Bounded face-interior semantics are used; trim topology, separate 3D edges, source/reader error and global material/mating remain separate')
    output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k!='faces'}),flush=True)
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input',type=Path)
    parser.add_argument('output',type=Path)
    args = parser.parse_args()
    run(args.input,args.output)
