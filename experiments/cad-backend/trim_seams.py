"""Explicit annular representatives of a single periodic analytical-face seam."""
from collections import Counter
from fractions import Fraction as F

from curve_jets import PI
from trim_loops import close_wire, displacement_bound, chart_domain, jordan


def periodic_wire(curves,indices):
    if len(curves)!=len(indices):raise ValueError('wire edge inventory mismatch')
    repeated = [edge for edge,count in Counter(indices).items() if count>1]
    if len(repeated)!=1 or indices.count(repeated[0])!=2:
        raise ValueError('expected exactly one twice-used seam edge')
    positions = [i for i,e in enumerate(indices) if e==repeated[0]]
    if (positions[1]-positions[0]) in (1,len(curves)-1):
        raise ValueError('periodic seam arcs must have disjoint endpoints')
    seam = [curves[i] for i in positions]
    if any(len(c)!=1 or len(c[0])!=2 or c[0][0][0]!=c[0][1][0] for c in seam):
        raise ValueError('expected straight constant-U seam curves')
    a,b = [c[0] for c in seam]
    if a[0][1]!=b[-1][1] or a[-1][1]!=b[0][1] or a[0][1]==a[-1][1]:
        raise ValueError('seam curves do not have identical reversed V intervals')
    left,right = sorted((a[0][0],b[0][0]))
    if left>=right:raise ValueError('empty periodic strip')
    changed,_ = close_wire(curves)
    corners = [c[-1][-1] for c in changed]
    # Use the common intrinsic seam V interval for both copies. This makes
    # every paired point, including endpoints, agree after periodic mapping.
    for i in positions:
        corners[i-1],corners[i] = curves[i][0][0],curves[i][0][-1]
    errors = []
    for i,c in enumerate(changed):
        c[0][0],c[-1][-1] = corners[i-1],corners[i]
        # A recorded, bounded representative correction, not a claim that
        # the original curve lies in this slab. Recheck all geometry afterward.
        changed[i] = [[(max(left,min(right,u)),v) for u,v in p] for p in c]
        errors.append([max(abs(p[k]-q[k]) for old,new in zip(curves[i],changed[i])
                           for p,q in zip(old,new)) for k in (0,1)])
    if any(changed[i]!=curves[i] for i in positions):
        raise ValueError('representative changed a seam meridian')
    return changed,errors,positions,(left,right)


def strict_strip(curves,seams,domain):
    """Only the named seam arcs and their four corners may touch strip sides."""
    left,right = domain
    corners = {k%len(curves) for s in seams for k in (s-1,s)}
    for i,curve in enumerate(curves):
        if i in seams:continue
        for j,p in enumerate(curve):
            u = [q[0] for q in p]
            if min(u)<left or max(u)>right or max(u)<=left or min(u)>=right:return False
            for end in (0,-1):
                if left<p[end][0]<right:continue
                corner = (i-1)%len(curves) if end==0 and j==0 else i if end==-1 and j==len(curve)-1 else None
                if corner not in corners:return False
    return True


def check(face,curves,indices,max_cells=1024):
    changed,errors,seams,(left,right) = periodic_wire(curves,indices)
    result = jordan(changed,max_cells)
    slab = strict_strip(changed,seams,(left,right))
    chart = chart_domain(face,changed)
    regular = F(chart['regular_margin_lower'])>0
    width = right-left
    period_error = max(abs(2*p-width) for p in PI)
    # The representative's actual angular coordinate is
    # left + 2*pi*(U-left)/width. Within the strip, its difference
    # from U is at most |2*pi-width|, throughout every curve piece.
    delta = displacement_bound(face,curves,changed,[[du+period_error,dv] for du,dv in errors])
    result.update(strict_strip=slab,regular_chart=regular,
                  embedded_annular_representative=result['status']=='verified' and slab and regular,
                  seam_positions=seams,seam_edge=indices[seams[0]],
                  rational_strip=list(map(str,(left,right))),
                  angular_map='left + 2*pi*(U-left)/(right-left)',
                  period_correction_bound=str(period_error),
                  displacement_bound_exact=str(delta),
                  representative_corners=[[str(x) for x in c[-1][-1]] for c in changed],chart=chart)
    return result
