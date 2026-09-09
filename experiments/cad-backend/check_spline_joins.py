"""Consistent spline-face representatives with verified shared-edge joins."""
import argparse
from fractions import Fraction as F
import json
from pathlib import Path
import time

from check_face_domains import digest,rectangle
from check_spline_embedding import exact
from check_spline_separation import incidence
from check_supports import upper
from edge_data import load
from spline_embedding import check
from spline_joins import prepare,joined


TARGET=F('0.00000001')


def run(vertices_path,separation_path,output):
    started=time.perf_counter()
    vertices=json.loads(vertices_path.read_text());edge_path=Path(vertices['edges_file'])
    if digest(edge_path)!=vertices['edges_sha256']:raise ValueError('changed edge evidence')
    edges,faces=load(edge_path)
    fv,_=incidence(vertices,edges,faces)
    prior=json.loads(separation_path.read_text())
    if (prior['vertices_sha256']!=digest(vertices_path) or prior['edges_sha256']!=digest(edge_path)
            or prior['status']!='verified' or not prior['complete_nonincident_spline_pairs']):
        raise ValueError('different or incomplete nonincident separation evidence')
    surfaces,joins,movement=prepare(faces,edges)
    for pair,surface in surfaces.items():
        domain=rectangle(faces[pair]['wires'])['domain']
        if domain!=[[k[0],k[-1]] for k in surface['knots']]:raise ValueError('trim differs from complete spline domain')
    incident,nonincident=set(),set();keys=sorted(surfaces)
    for i,a in enumerate(keys):
        for b in keys[i+1:]:
            if a[0]!=b[0]:continue
            (incident if fv[a]&fv[b] else nonincident).add((a[0],a[1],b[1]))
    actual=[(j['member'],*sorted(j['faces'])) for j in joins]
    if len(actual)!=len(set(actual)) or set(actual)!=incident:
        raise ValueError('spline incident pairs are not single full-boundary joins')
    previous=[(r['member'],*r['faces']) for r in prior['pairs']]
    if len(previous)!=len(set(previous)) or set(previous)!=nonincident:
        raise ValueError('incomplete nonincident separation inventory')
    minimum=None
    for row in prior['pairs']:
        m=row['member'];a,b=row['faces']
        if not row['verified'] or row['unresolved_cells'] or row['tested_cells']!=2*row['accepted_cells']-1:
            raise ValueError('unverified source pair separation')
        gap=F(row['distance_lower_mm_exact'])-movement[m,a]-movement[m,b]
        if gap<=0:raise ValueError('representative movement consumes pair separation')
        minimum=gap if minimum is None else min(minimum,gap)
    results=[];covered=set()
    for join in joins:
        m=join['member'];a,b=join['faces']
        surface=joined(surfaces[m,a],surfaces[m,b],join['boundaries'])
        result=check(surface,[[0,1],[0,2]])
        result.update(join)
        results.append(exact(result))
        if result['verified']:covered|={(m,a),(m,b)}
        if len(results)%48==0:
            print(json.dumps(dict(joins=len(results),selected=len(joins),seconds=time.perf_counter()-started)),flush=True)
    bounded=all(d<=TARGET for d in movement.values()) and all(F(j['shared_curve_edge_error_exact'])<=TARGET for j in joins)
    report=dict(vertices_file=str(vertices_path),vertices_sha256=digest(vertices_path),
                edges_file=str(edge_path),edges_sha256=digest(edge_path),
                separation_file=str(separation_path),separation_sha256=digest(separation_path),
                status='verified' if covered==set(surfaces) and all(r['verified'] for r in results) and bounded else 'unresolved',
                complete_spline_join_inventory=True,joins=results,
                faces=[dict(member=m,face_index=i,displacement_bound_exact=str(d),displacement_bound_mm=upper(d))
                       for (m,i),d in sorted(movement.items())],
                nonincident_pairs_preserved=len(nonincident),
                minimum_remaining_separation_exact=str(minimum),
                maximum_face_displacement_mm=upper(max(movement.values())),
                maximum_shared_curve_edge_error_mm=upper(max(F(j['shared_curve_edge_error_exact']) for j in joins)),
                target_displacement_mm=float(TARGET),seconds=time.perf_counter()-started,
                scope='One consistent embedded representative of the complete spline-face subcomplex: '
                      'all intended spline joins and all nonincident spline separations. '
                      'Analytical-face attachment, complete closed 3D topology, global material, '
                      'continuous mating and source/reader accuracy remain separate.')
    output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k not in ('faces','joins')},indent=2))
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('vertices',type=Path);parser.add_argument('separation',type=Path)
    parser.add_argument('output',type=Path);args=parser.parse_args()
    run(args.vertices,args.separation,args.output)
