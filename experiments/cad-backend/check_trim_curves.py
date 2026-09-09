"""Audit individual analytical trim injectivity and unsnapped UV corner gaps."""
import argparse
import json
from pathlib import Path
import time

from check_face_domains import digest
from edge_data import load
from trim_curves import pieces, monotone_axis, separated


def run(path, output, max_cells=1024):
    if max_cells<1:raise ValueError('positive separation budget required')
    started = time.perf_counter()
    edges,faces = load(path)
    key = lambda c:json.dumps(c,sort_keys=True,separators=(',',':'))
    uses = {}
    for member in edges['members']:
        for edge in member['edges']:
            for use in edge['incidences']:
                identity = member['member'],use['face_index'],key(use['curve'])
                if identity in uses:raise ValueError('ambiguous trim edge association')
                uses[identity] = edge['edge_index'],use['reversed']
    curves,corners,wires = [],[],[]
    for pair,face in sorted(faces.items()):
        if face['kind']=='bspline':continue
        for wi,wire in enumerate(face['wires']):
            ends,polynomials = [],[]
            for ci,curve in enumerate(wire):
                edge,reverse = uses[(*pair,key(curve))]
                parts = pieces(curve)
                polynomials.append(parts)
                endpoints = [parts[0][0],parts[-1][-1]]
                if reverse:endpoints.reverse()
                ends.append((edge,endpoints))
                curves.append(dict(member=pair[0],face_index=pair[1],wire=wi,curve=ci,
                                   edge_index=edge,pieces=len(parts),monotone=monotone_axis(parts)))
            for (incoming,a),(outgoing,b) in zip(ends,ends[1:]+ends[:1]):
                gap = [y-x for x,y in zip(a[1],b[0])]
                corners.append(dict(member=pair[0],face_index=pair[1],wire=wi,
                                    incoming=incoming,outgoing=outgoing,
                                    exact_uv_gap=list(map(str,gap))))
            tested,pairs,unresolved = 0,0,[]
            for i,a in enumerate(polynomials):
                for j in range(i+1,len(polynomials)):
                    if j==i+1 or (i==0 and j==len(polynomials)-1):continue
                    ok,cells = separated(a,polynomials[j],max_cells)
                    tested+=cells;pairs+=1
                    if not ok:unresolved.append([i,j])
            wires.append(dict(member=pair[0],face_index=pair[1],wire=wi,
                              nonadjacent_pairs=pairs,tested_cells=tested,unresolved=unresolved))
    unresolved = sum(c['monotone'] is None for c in curves)
    report = dict(edges_file=str(path),edges_sha256=digest(path),
                  individual_curves='verified' if not unresolved else 'unresolved',
                  curve_count=len(curves),unresolved_curves=unresolved,
                  exact_closed_corners=sum(all(x=='0' for x in c['exact_uv_gap']) for c in corners),
                  nonadjacent_pairs=sum(w['nonadjacent_pairs'] for w in wires),
                  unresolved_pairs=sum(len(w['unresolved']) for w in wires),
                  nonadjacent_uv_separation='verified' if all(not w['unresolved'] for w in wires) else 'unresolved',
                  max_cells_per_pair=max_cells,
                  curves=curves,corners=corners,wires=wires,seconds=time.perf_counter()-started,
                  scope='Individual pcurve injectivity, nonadjacent UV separation and raw UV endpoint differences; '
                        'no adjacent-curve separation, periodic chart identification, corner repair, '
                        'Jordan loop or surface embedding certificate')
    output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k not in ('curves','corners','wires')},indent=2))


if __name__=='__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('edges',type=Path)
    parser.add_argument('output',type=Path)
    parser.add_argument('--max-cells',type=int,default=1024)
    args = parser.parse_args()
    run(args.edges,args.output,args.max_cells)
