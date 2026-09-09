"""Independent complete parameter-rectangle embedding check for indexed splines."""
import argparse
from fractions import Fraction as F
import json
from pathlib import Path
import time

from check_face_domains import digest, rectangle
from edge_data import load
from spline_embedding import check


def exact(value):
    if isinstance(value,F):return str(value)
    if isinstance(value,(tuple,list)):return [exact(x) for x in value]
    if isinstance(value,dict):return {k:exact(v) for k,v in value.items()}
    return value


def run(path,output,member=None,index=None):
    started = time.perf_counter()
    _,faces = load(path)
    selected = [(pair,face) for pair,face in sorted(faces.items()) if face['kind']=='bspline'
                and (member is None or pair[0]==member) and (index is None or face['index']==index)]
    if not selected:raise ValueError('no selected spline faces')
    results = []
    for pair,face in selected:
        domain = rectangle(face['wires'])
        result = check(face['surface'],domain['domain'])
        result.update(member=pair[0],face_index=pair[1],index=face['index'],
                      reference_face=face['reference_face'],trim_domain=domain)
        results.append(exact(result))
        if len(results)%60==0:
            print(json.dumps(dict(faces=len(results),selected=len(selected),seconds=time.perf_counter()-started)),flush=True)
    report = dict(edges_file=str(path),edges_sha256=digest(path),
                  status='verified' if all(r['verified'] and r['regular_everywhere'] for r in results) else 'unresolved',
                  complete_indexed_splines=member is None and index is None,
                  faces=results,seconds=time.perf_counter()-started,
                  scope='Injectivity and differential regularity of each actual indexed polynomial '
                        'spline on its complete clamped rectangle, with checked finite rectangular trims. '
                        'Inter-face embedding, common 3D topology realization, global material, '
                        'continuous mating and source/reader accuracy remain separate.')
    output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k!='faces'},indent=2))
    return report


if __name__=='__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('edges',type=Path)
    parser.add_argument('output',type=Path)
    parser.add_argument('--member',type=int,choices=(0,1))
    parser.add_argument('--index',type=int)
    args = parser.parse_args()
    run(args.edges,args.output,args.member,args.index)
