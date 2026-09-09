"""Exact finite polynomial pcurve pieces, without endpoint snapping or clamping."""
from fractions import Fraction as F

from bernstein import refine
from parameter_bounds import curve_bounds


def split(poles, t):
    rows = [poles]
    while len(rows[-1]) > 1:
        rows.append([tuple((1-t)*x+t*y for x,y in zip(a,b))
                     for a,b in zip(rows[-1],rows[-1][1:])])
    return [r[0] for r in rows], [r[-1] for r in reversed(rows)]


def restrict(poles, lo, hi):
    """Reparameterize a polynomial to [lo,hi], even outside its base span."""
    if lo >= hi:raise ValueError('empty polynomial restriction')
    if hi != 0:
        left,_ = split(poles,hi)
        return split(left,lo/hi)[1]
    _,right = split(poles,lo)
    return split(right,(hi-lo)/(1-lo))[0]


def pieces(curve):
    # Reuse the existing strict validation of dimensions, weights and knots.
    curve_bounds(curve)
    lo,hi = map(F,curve['interval'])
    if curve['kind']=='line':
        a,d = [[F(x) for x in curve[k]] for k in ('location','direction')]
        return [[tuple(x+t*y for x,y in zip(a,d)) for t in (lo,hi)]]
    degree = curve['degree']
    poles,knots = refine([tuple(F(x) for x in p) for p in curve['poles']],
                         list(map(F,curve['knots'])),degree)
    unique = sorted(set(knots))
    result = []
    for i,(a,b) in enumerate(zip(unique,unique[1:])):
        start = lo if i==0 else max(lo,a)
        end = hi if i==len(unique)-2 else min(hi,b)
        if start < end:
            result.append(restrict(poles[i*degree:i*degree+degree+1],
                                   (start-a)/(b-a),(end-a)/(b-a)))
    if not result:raise ValueError('missing finite polynomial pieces')
    if any(a[-1]!=b[0] for a,b in zip(result,result[1:])):
        raise ValueError('discontinuous polynomial pcurve')
    return result


def monotone_axis(parts):
    """A strict coordinate order proves injectivity of the entire finite curve.

    Nonnegative derivative coefficients suffice when each polynomial span has
    at least one positive coefficient. Isolated zero derivatives are allowed.
    Failure to find an axis is inconclusive, not a self-intersection witness.
    """
    if not parts or any(len(p)<2 for p in parts):
        raise ValueError('empty polynomial curve')
    if any(a[-1]!=b[0] for a,b in zip(parts,parts[1:])):
        raise ValueError('discontinuous polynomial curve')
    for axis in (0,1):
        for sign in (1,-1):
            differences = [[sign*(b[axis]-a[axis]) for a,b in zip(p,p[1:])] for p in parts]
            if all(d and min(d)>=0 and max(d)>0 for d in differences):
                return dict(axis=axis,sign=sign)
    return None


def bounds(parts):
    return [(min(p[i] for part in parts for p in part),max(p[i] for part in parts for p in part))
            for i in (0,1)]


def disjoint(a,b):
    return any(x[1]<y[0] or y[1]<x[0] for x,y in zip(a,b))


def separated(a,b,max_cells=1024):
    """Cover a full curve-product domain by strictly separated Bernstein boxes.

    Each split retains both children. A budget exit is inconclusive and cannot
    certify separation. Touching points are deliberately not accepted.
    """
    if max_cells<1:raise ValueError('positive separation budget required')
    if disjoint(bounds(a),bounds(b)):return True,1
    pending = [(p,q) for p in a for q in b]
    tested = 0
    while pending and tested<max_cells:
        p,q = pending.pop();tested+=1
        bp,bq = bounds([p]),bounds([q])
        if disjoint(bp,bq):continue
        # Split the larger bounding box; this affects cost, not acceptance.
        if max(y-x for x,y in bp)>=max(y-x for x,y in bq):
            pending.extend((child,q) for child in split(p,F(1,2)))
        else:
            pending.extend((p,child) for child in split(q,F(1,2)))
    return not pending,tested
