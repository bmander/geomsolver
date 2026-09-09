"""Bound and verify explicit closed UV representatives of analytical STEP trims."""
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
from trim_loops import close_wire, displacement_bound, jordan, chart_domain


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
    selected = {pair:f for pair,f in faces.items() if f['kind']!='bspline'
                and (member is None or pair[0]==member)
                and (face_index is None or pair[1]==face_index)}
    if not selected:raise ValueError('no selected analytical faces')
    results = []
    for pair,face in sorted(selected.items()):
        if len(face['wires'])!=1:raise ValueError('multiple-wire face requires a containment audit')
        original,indices = [],[]
        for curve in face['wires'][0]:
            edge,reverse = uses[(*pair,key(curve))]
            parts = pieces(curve)
            if reverse:parts = [list(reversed(p)) for p in reversed(parts)]
            original.append(parts);indices.append(edge)
        changed,errors = close_wire(original)
        delta = displacement_bound(face,original,changed,errors)
        result = jordan(changed,max_cells)
        chart = chart_domain(face,changed)
        seams = [edge for edge,count in Counter(indices).items() if count>1]
        result.update(member=pair[0],face_index=pair[1],kind=face['kind'],edge_indices=indices,
                      displacement_bound_mm=upper(delta),displacement_bound_exact=str(delta),
                      within_displacement_target=delta<=TARGET,
                      representative_corners=[[str(x) for x in c[-1][-1]] for c in changed],
                      chart=chart,seam_edge_indices=seams,
                      embedded_representative=result['status']=='verified' and chart['injective_rectangle'] and not seams)
        results.append(result)
        if len(results)%8==0:
            print(json.dumps(dict(faces=len(results),selected=len(selected),seconds=time.perf_counter()-started)),flush=True)
    report = dict(edges_file=str(path),edges_sha256=digest(path),
                  status='verified' if all(r['status']=='verified' and r['within_displacement_target'] for r in results) else 'unresolved',
                  complete_analytical_pair=member is None and face_index is None,
                  target_displacement_mm=float(TARGET),max_cells_per_pair=max_cells,
                  faces=results,maximum_displacement_bound_mm=max(r['displacement_bound_mm'] for r in results),
                  embedded_representatives=sum(r['embedded_representative'] for r in results),
                  faces_requiring_seam_identification=sum(bool(r['seam_edge_indices']) for r in results),
                  seconds=time.perf_counter()-started,
                  scope='Existence of simple closed UV representatives within a bounded surface displacement. '
                        'The raw STEP pcurves are not modified. Periodic chart identification, '
                        'surface embedding, shared 3D topology realization, reader behavior, '
                        'global material and mating are separate.')
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
