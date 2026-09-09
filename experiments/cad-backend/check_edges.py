"""Whole-interval 3D-edge versus incident-face correspondence on indexed STEP solids."""
import argparse
from fractions import Fraction as F
import json
from pathlib import Path
import time

from check_face_domains import digest
from check_supports import upper
from curve_jets import Curve, correspondence_bound, elementary_bound
from edge_data import load
from edge_geometry import iso_curve, same_basis_bound


TARGET = F('0.000002')


def analytical(curve, parameter, surface, max_cells):
    direct = elementary_bound(curve,parameter,surface)
    if direct is not None:
        return dict(method='shared_elementary_functions',bound_mm=upper(direct),passes=direct<=TARGET,cells_tested=0)
    a,b = Curve(curve),Curve(parameter)
    lo,hi = [F(x) for x in curve['interval']]
    knots = sorted({lo,hi}|{k for k in a.breaks+b.breaks if lo<k<hi})
    pending = list(reversed(list(zip(knots,knots[1:]))))
    accepted,violations,refusals = [],[],{}
    tested = 0
    while pending and tested<max_cells:
        domain = pending.pop();tested+=1
        try:
            lower,bound = correspondence_bound(a,b,surface,domain)
        except (ValueError,ZeroDivisionError) as error:
            lower,bound = F(0),None
            reason=str(error);refusals[reason]=refusals.get(reason,0)+1
        if lower>TARGET:
            violations.append(dict(domain=list(map(str,domain)),center_error_lower_bound_mm=-upper(-lower)))
            break
        if bound is not None and bound<=TARGET:
            accepted.append(dict(domain=list(map(str,domain)),bound_mm=upper(bound)))
        else:
            middle=sum(domain)/2
            pending.extend(((middle,domain[1]),(domain[0],middle)))
    passes=not pending and not violations
    if passes:
        cursor=lo
        for x,y in sorted(tuple(F(v) for v in r['domain']) for r in accepted):
            if x!=cursor:raise ValueError('edge cover has a gap or overlap')
            cursor=y
        if cursor!=hi:raise ValueError('edge cover ends early')
    return dict(method='cubic_taylor',passes=passes,cells_tested=tested,accepted=accepted,
                bound_mm=max((r['bound_mm'] for r in accepted),default=0) if passes else None,
                unresolved=[list(map(str,d)) for d in pending],violations=violations,refinement_refusals=refusals)


def run(path,output,member=None,edge_limit=None,max_cells=2048,edges=None):
    if max_cells<=0 or edge_limit is not None and edge_limit<=0:
        raise ValueError('expected positive work limits')
    data,faces=load(path)
    started=time.perf_counter()
    result=dict(evidence_sha256=digest(path),member=member,target_mm=upper(TARGET),results=[],
                complete_pair=False,complete_selected_members=False,status='running',
                scope='Shared 3D edges versus both incident parameterized faces. Finite intervals and exact parameter coverage are checked; vertex geometry, trim topology and global material/mating remain separate')
    all_pass=True
    for m in data['members']:
        if member is not None and m['member']!=member:continue
        selected=m['edges'][:edge_limit] if edge_limit is not None else m['edges']
        if edges is not None:selected=[e for e in selected if e['edge_index'] in edges]
        for edge in selected:
            for side,use in enumerate(edge['incidences']):
                face=faces[(m['member'],use['face_index'])]
                if face['kind']=='bspline':
                    bound=same_basis_bound(edge['curve'],iso_curve(face['surface'],use['curve']))
                    checked=dict(method='same_polynomial_basis',bound_mm=upper(bound),passes=bound<=TARGET,cells_tested=0)
                else:
                    checked=analytical(edge['curve'],use['curve'],face,max_cells)
                row=dict(member=m['member'],edge_index=edge['edge_index'],incidence=side,face_index=use['face_index'],**checked)
                result['results'].append(row)
                all_pass &= checked['passes']
            if len(result['results'])%32==0:
                print(f"checked {len(result['results'])} incidences",flush=True)
                output.write_text(json.dumps(result,indent=2)+'\n')
            if not all_pass:break
        if not all_pass:break
    complete=all_pass and bool(result['results']) and edge_limit is None and edges is None
    result.update(status='verified' if all_pass and result['results'] else 'unresolved_or_failed',
        complete_selected_members=complete,complete_pair=complete and member is None,
        seconds=time.perf_counter()-started)
    output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='results'}),flush=True)
    return result


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input',type=Path)
    parser.add_argument('output',type=Path)
    parser.add_argument('--member',type=int,choices=(0,1))
    parser.add_argument('--edge-limit',type=int)
    parser.add_argument('--edges',type=int,nargs='+')
    parser.add_argument('--max-cells',type=int,default=2048)
    args=parser.parse_args()
    result=run(args.input,args.output,args.member,args.edge_limit,args.max_cells,args.edges)
    raise SystemExit(0 if result['status']=='verified' else 1)
