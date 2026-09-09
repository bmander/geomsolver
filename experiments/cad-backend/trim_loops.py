"""Explicit closed representatives of tolerant UV wires and Jordan-loop checks."""
from fractions import Fraction as F

from curve_jets import PI
from interval_jet import f, point, sin_cos
from trim_curves import bounds, disjoint, split, monotone_axis, separated


def close_wire(curves):
    """Move each endpoint to its corner midpoint, retaining all other poles.

    Curves and pieces are already oriented in wire order. The returned error
    bounds apply to corresponding parameters on each original Bezier piece.
    This constructs a new representative, not an assertion of exact input closure.
    """
    if len(curves)<3:raise ValueError('expected at least three wire curves')
    corners = [tuple((F(x)+F(y))/2 for x,y in zip(a[-1][-1],b[0][0]))
               for a,b in zip(curves,curves[1:]+curves[:1])]
    result,errors = [],[]
    for i,curve in enumerate(curves):
        changed = [list(p) for p in curve]
        changed[0][0],changed[-1][-1] = corners[i-1],corners[i]
        error = [max(abs(a[k]-b[k]) for p,q in zip(curve,changed) for a,b in zip(p,q))
                 for k in (0,1)]
        result.append(changed);errors.append(error)
    return result,errors


def corner_separator(p,q):
    """Prove p and q meet only at p[-1] == q[0], using an exact separating line."""
    if p[-1]!=q[0]:return False
    corner = p[-1]
    a = [tuple(F(x)-F(y) for x,y in zip(v,corner)) for v in p]
    b = [tuple(F(x)-F(y) for x,y in zip(v,corner)) for v in q]
    first = next((v for v in reversed(a[:-1]) if any(v)),None)
    second = next((v for v in b[1:] if any(v)),None)
    if first is None or second is None:return False
    # Candidate choice is only a search heuristic. Every pole projection is
    # then checked exactly. L1 normalization keeps all coefficients rational.
    na,nb = sum(map(abs,first)),sum(map(abs,second))
    normals = [(1,0),(0,1),tuple(x/na-y/nb for x,y in zip(first,second)),
               (-first[1],first[0]),(-second[1],second[0])]
    for normal in normals:
        pa = [sum(x*y for x,y in zip(normal,v)) for v in a]
        pb = [sum(x*y for x,y in zip(normal,v)) for v in b]
        for sign in (1,-1):
            ca,cb = [sign*x for x in pa],[-sign*x for x in pb]
            # Each interior Bezier basis function is positive on (0,1).
            # Nonnegative projections with a positive non-shared endpoint
            # therefore put both open curves in strictly opposite halfplanes.
            if min(ca)>=0 and ca[0]>0 and min(cb)>=0 and cb[-1]>0:
                return True
    return False


def adjacent(a,b,max_cells=1024):
    if max_cells<1:raise ValueError('positive adjacency budget required')
    if a[-1][-1]!=b[0][0]:raise ValueError('adjacent representatives do not close')
    common = a[-1][-1]
    pending = [(p,q) for p in a for q in b]
    tested = 0
    while pending and tested<max_cells:
        p,q = pending.pop();tested+=1
        bp,bq = bounds([p]),bounds([q])
        if disjoint(bp,bq):continue
        if p[-1]==q[0]==common and corner_separator(p,q):continue
        if max(y-x for x,y in bp)>=max(y-x for x,y in bq):
            pending.extend((child,q) for child in split(p,F(1,2)))
        else:
            pending.extend((p,child) for child in split(q,F(1,2)))
    return not pending,tested


def jordan(curves,max_cells=1024):
    """Sufficient exact conditions for a piecewise-polynomial Jordan loop."""
    if len(curves)<3:raise ValueError('expected at least three wire curves')
    individual = [monotone_axis(c) for c in curves]
    unresolved,tests,pairs = [],0,0
    boxes = [bounds(c) for c in curves]
    for i,a in enumerate(curves):
        for j in range(i+1,len(curves)):
            b = curves[j];pairs+=1
            if j==i+1:
                ok,count = adjacent(a,b,max_cells)
            elif i==0 and j==len(curves)-1:
                ok,count = adjacent(b,a,max_cells)
            elif disjoint(boxes[i],boxes[j]):
                ok,count = True,1
            else:
                ok,count = separated(a,b,max_cells)
            tests+=count
            if not ok:unresolved.append([i,j])
    return dict(status='verified' if all(x is not None for x in individual) and not unresolved else 'unresolved',
                individual_monotonicity=individual,pairs=pairs,tested_cells=tests,unresolved_pairs=unresolved)


def analytical_radius(face):
    basis = [[F(x) for x in row] for row in face['basis']]
    if (len(basis)!=3 or any(len(row)!=3 for row in basis)
            or any(sum(x*y for x,y in zip(a,b))!=int(i==j)
                   for i,a in enumerate(basis) for j,b in enumerate(basis))):
        raise ValueError('expected an exactly orthonormal analytical frame')
    radius = F(face['radius'])
    if radius<=0:raise ValueError('expected positive reference radius')
    return radius


def displacement_bound(face,original,changed,errors):
    """World-distance bound for corresponding UV points on a sphere or cone."""
    radius = analytical_radius(face)
    du,dv = [max(e[i] for e in errors) for i in (0,1)]
    if face['kind']=='sphere':return radius*(du+dv)
    if face['kind']=='cone':
        v = max(abs(p[1]) for curve in original+changed for part in curve for p in part)
        return (radius+v)*du+dv
    raise ValueError('unsupported representative surface')


def chart_domain(face,curves):
    """Prove injectivity on a full UV rectangle, excluding periodic seams.

    A Jordan loop's bounded interior lies inside its boundary's coordinate box.
    This check can therefore transfer its embedding to the analytical surface,
    but does not identify separate chart edges as a periodic seam.
    """
    radius = analytical_radius(face)
    u,v = bounds([part for curve in curves for part in curve])
    period_margin = 2*PI[0]-(u[1]-u[0])
    if face['kind']=='sphere':
        regular_margin = min(v[0]+PI[0]/2,PI[0]/2-v[1])
    elif face['kind']=='cone':
        sine,cosine = sin_cos(point(face['angle']))
        radial = f.add(point(radius),f.mul(v,sine))
        regular_margin = min(radial[0],max(cosine[0],-cosine[1]))
    else:raise ValueError('unsupported representative surface')
    return dict(injective_rectangle=period_margin>0 and regular_margin>0,
                parameter_bounds=[[str(x) for x in d] for d in (u,v)],
                period_margin_lower=str(period_margin),regular_margin_lower=str(regular_margin))
