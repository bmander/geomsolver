"""Complete disjointness check for nonincident indexed spline-face pairs."""
import argparse
from collections import defaultdict
from fractions import Fraction as F
import json
from pathlib import Path
import time

from check_face_domains import digest, rectangle
from edge_data import load
from spline_separation import Node, WORLD, plane_gap, separate
from vertex_topology import check as topology


def incidence(vertices,edges,faces):
    if len(vertices['members'])!=2 or {m['member'] for m in vertices['members']}!={0,1}:
        raise ValueError('incomplete vertex member inventory')
    ends = {}
    for member in vertices['members']:
        m=member['member']
        actual=[r['edge_index'] for r in member['edges']]
        original,=[r for r in edges['members'] if r['member']==m]
        if len(actual)!=len(set(actual)) or set(actual)!={e['edge_index'] for e in original['edges']}:
            raise ValueError('different vertex edge inventory')
        for edge in member['edges']:
            if len(edge['vertices'])!=2 or any(type(v) is not int or not 1<=v<=len(member['vertices']) for v in edge['vertices']):
                raise ValueError('invalid edge endpoint vertex indices')
            ends[m,edge['edge_index']] = set(edge['vertices'])
    topology(vertices,edges,faces)
    fv,fe=defaultdict(set),defaultdict(set)
    for member in edges['members']:
        m=member['member']
        for edge in member['edges']:
            for use in edge['incidences']:
                pair=m,use['face_index']
                fv[pair]|=ends[m,edge['edge_index']];fe[pair].add(edge['edge_index'])
    return fv,fe


def run(path,output,member=None,face_ids=None,max_cells=1024):
    if max_cells<1:raise ValueError('positive pair budget required')
    if face_ids is not None and (member is None or len(set(face_ids))!=2):
        raise ValueError('two different face IDs require a selected member')
    started=time.perf_counter()
    vertices=json.loads(path.read_text());edge_path=Path(vertices['edges_file'])
    if digest(edge_path)!=vertices['edges_sha256']:raise ValueError('changed edge evidence')
    edges,faces=load(edge_path)
    fv,fe=incidence(vertices,edges,faces)
    selected={pair:f for pair,f in sorted(faces.items()) if f['kind']=='bspline'
              and (member is None or pair[0]==member) and (face_ids is None or pair[1] in face_ids)}
    if not selected or (face_ids is not None and len(selected)!=2):raise ValueError('missing selected spline faces')
    nodes={}
    for pair,face in selected.items():
        domain=rectangle(face['wires'])['domain']
        node=Node.surface(face['surface'])
        if domain!=[[k[0],k[-1]] for k in node.knots]:raise ValueError('trim differs from bounded spline domain')
        nodes[pair]=node
    results,incident=[],[];pairs=list(nodes)
    for i,a in enumerate(pairs):
        for b in pairs[i+1:]:
            if a[0]!=b[0]:continue
            common=fv[a]&fv[b]
            if common:
                incident.append(dict(member=a[0],faces=[a[1],b[1]],shared_vertices=sorted(common),shared_edges=sorted(fe[a]&fe[b])))
                continue
            gap=plane_gap(nodes[a],nodes[b],WORLD)
            if gap is not None:
                result=dict(verified=True,tested_cells=1,accepted_cells=1,unresolved_cells=0,distance_lower=gap)
                method='world_bounds'
            else:
                result=separate(nodes[a],nodes[b],max_cells)
                method='support_planes' if result['tested_cells']==1 else 'subdivision'
            gap=result.pop('distance_lower')
            result.update(member=a[0],faces=[a[1],b[1]],method=method,
                          distance_lower_mm_exact=str(gap) if gap is not None else None)
            results.append(result)
            if len(results)%5000==0:
                print(json.dumps(dict(pairs=len(results),seconds=time.perf_counter()-started)),flush=True)
    verified=all(r['verified'] for r in results)
    minimum=min((F(r['distance_lower_mm_exact']) for r in results),default=None) if verified else None
    report=dict(vertices_file=str(path),vertices_sha256=digest(path),
                edges_file=str(edge_path),edges_sha256=digest(edge_path),
                status='verified' if verified else 'unresolved',
                complete_nonincident_spline_pairs=member is None and face_ids is None,
                selected_spline_faces=len(selected),max_cells_per_pair=max_cells,
                pairs=results,incident_pairs=incident,
                minimum_separation_lower_mm_exact=str(minimum) if minimum is not None else None,
                seconds=time.perf_counter()-started,
                scope='Whole-surface separation of every selected spline-face pair with no common topological vertex. '
                      'Incident pairs are explicitly inventoried and remain unverified here. '
                      'Analytical-face pairs, common 3D topology realization, global material, '
                      'continuous mating and source/reader accuracy remain separate.')
    output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k not in ('pairs','incident_pairs')},indent=2))
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('vertices',type=Path);parser.add_argument('output',type=Path)
    parser.add_argument('--member',type=int,choices=(0,1));parser.add_argument('--faces',type=int,nargs=2)
    parser.add_argument('--max-cells',type=int,default=1024)
    args=parser.parse_args()
    run(args.vertices,args.output,args.member,args.faces,args.max_cells)
