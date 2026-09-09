"""Verify bounded annular representatives for every repeated analytical seam edge."""
import argparse
from collections import Counter
from fractions import Fraction as F
import json
from pathlib import Path
import time

from check_face_domains import digest
from check_supports import upper
from edge_data import load
from trim_curves import pieces
from trim_seams import check


TARGET = F('0.00000001')


def run(path,output,member=None,face_index=None,max_cells=1024):
    if max_cells<1:raise ValueError('positive pair budget required')
    started = time.perf_counter()
    edges,faces = load(path)
    key = lambda c:json.dumps(c,sort_keys=True,separators=(',',':'))
    uses = {}
    for m in edges['members']:
        for edge in m['edges']:
            for use in edge['incidences']:
                identity = m['member'],use['face_index'],key(use['curve'])
                if identity in uses:raise ValueError('ambiguous trim edge association')
                uses[identity] = edge['edge_index'],use['reversed']
    results = []
    for pair,face in sorted(faces.items()):
        if face['kind']=='bspline':continue
        if member is not None and pair[0]!=member:continue
        if face_index is not None and pair[1]!=face_index:continue
        if len(face['wires'])!=1:raise ValueError('multiple-wire face requires a containment audit')
        wire = face['wires'][0]
        associations = [uses[(*pair,key(c))] for c in wire]
        indices = [edge for edge,_ in associations]
        if max(Counter(indices).values())<2:continue
        original = []
        for curve,(_,reverse) in zip(wire,associations):
            parts = pieces(curve)
            if reverse:parts = [list(reversed(p)) for p in reversed(parts)]
            original.append(parts)
        result = check(face,original,indices,max_cells)
        delta = F(result['displacement_bound_exact'])
        result.update(member=pair[0],face_index=pair[1],kind=face['kind'],edge_indices=indices,
                      displacement_bound_mm=upper(delta),within_displacement_target=delta<=TARGET)
        results.append(result)
        print(json.dumps({k:result[k] for k in ('member','face_index','embedded_annular_representative',
                                               'displacement_bound_mm','within_displacement_target')}),flush=True)
    if not results:raise ValueError('no selected periodic seam faces')
    report = dict(edges_file=str(path),edges_sha256=digest(path),
                  status='verified' if all(r['embedded_annular_representative'] and r['within_displacement_target'] for r in results) else 'unresolved',
                  complete_analytical_seams=member is None and face_index is None,
                  target_displacement_mm=float(TARGET),max_cells_per_pair=max_cells,faces=results,
                  maximum_displacement_bound_mm=max(r['displacement_bound_mm'] for r in results),
                  seconds=time.perf_counter()-started,
                  scope='Embedded annular representatives with exact periodic seam identification '
                        'and bounded displacement from raw analytical STEP boundaries. '
                        'Shared 3D topology realization, inter-face embedding, reader behavior, '
                        'global swept material and continuous mating remain separate.')
    output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k!='faces'},indent=2))
    return report


if __name__=='__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('edges',type=Path)
    parser.add_argument('output',type=Path)
    parser.add_argument('--member',type=int,choices=(0,1))
    parser.add_argument('--face',type=int)
    parser.add_argument('--max-cells',type=int,default=1024)
    args = parser.parse_args()
    run(args.edges,args.output,args.member,args.face,args.max_cells)
