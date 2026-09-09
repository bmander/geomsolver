"""Independent shared-edge endpoint versus topological vertex bounds."""
import argparse
from fractions import Fraction as F
import json
from pathlib import Path
import time

from check_face_domains import digest
from check_supports import upper
from curve_jets import Curve,norm_bound
from curve_taylor import Taylor
from edge_data import load
from vertex_topology import check as check_topology


def endpoint_bound(curve,t,vertex):
    if len(vertex)!=3:raise ValueError('expected a spatial vertex')
    point=curve.evaluate(Taylor(F(t)))
    return norm_bound([p-F(v) for p,v in zip(point,vertex)])


def run(path,output):
    started=time.perf_counter()
    data=json.loads(path.read_text());edges=Path(data['edges_file'])
    if digest(edges)!=data['edges_sha256']:raise ValueError('changed endpoint edge data')
    source,faces=load(edges)
    if len(data['members'])!=2 or {r['member'] for r in data['members']}!={0,1}:
        raise ValueError('incomplete vertex members')
    results=[]
    for member in data['members']:
        original,=[m for m in source['members'] if m['member']==member['member']]
        records={e['edge_index']:e for e in original['edges']}
        if len(member['edges'])!=len(records) or {e['edge_index'] for e in member['edges']}!=set(records):
            raise ValueError('missing or duplicated endpoint edge')
        used=set()
        for edge in member['edges']:
            if len(edge['vertices'])!=2:raise ValueError('expected two edge endpoints')
            record=records[edge['edge_index']]['curve'];curve=Curve(record)
            for end,(t,index) in enumerate(zip(record['interval'],edge['vertices'])):
                if type(index) is not int or not 1<=index<=len(member['vertices']):
                    raise ValueError('edge refers to unknown vertex')
                used.add(index)
                bound=endpoint_bound(curve,t,member['vertices'][index-1])
                if bound>F('0.000002'):raise ValueError('endpoint misses its topological vertex')
                results.append(dict(member=member['member'],edge_index=edge['edge_index'],end=end,
                                    vertex_index=index,bound_mm=upper(bound)))
        if used!=set(range(1,len(member['vertices'])+1)):
            raise ValueError('vertex inventory contains unaccounted points')
    topology=check_topology(data,source,faces)
    report=dict(evidence_sha256=digest(path),status='verified',endpoints=results,topology=topology,
        maximum_bound_mm=max(r['bound_mm'] for r in results),seconds=time.perf_counter()-started,
        scope='Every shared 3D edge endpoint agrees with its incident topological vertex; combinatorial wire closure and single-cycle vertex links are checked; geometric trim topology, source/reader error and global material/mating remain separate')
    output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k not in ('endpoints','topology')}),flush=True)
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input',type=Path);parser.add_argument('output',type=Path)
    args=parser.parse_args();run(args.input,args.output)
