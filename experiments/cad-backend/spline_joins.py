"""Common spline boundaries and exact two-face rectangular representatives."""
from copy import deepcopy
from fractions import Fraction as F

from edge_geometry import iso_curve, same_basis_bound
from interval_jet import f, point
from spline_embedding import polynomial_surface


def boundary(parameter):
    if (parameter['kind']!='line' or parameter['interval']!=[0,1]
            or parameter['direction']!=[1,0] or parameter['location'][0]!=0
            or parameter['location'][1] not in (0,1)):
        raise ValueError('expected a complete unit-U natural boundary')
    return int(parameter['location'][1])


def prepare(faces,edges):
    surfaces={pair:deepcopy(face['surface']) for pair,face in faces.items() if face['kind']=='bspline'}
    used=set();joins=[]
    for member in edges['members']:
        m=member['member']
        for edge in member['edges']:
            uses=edge['incidences']
            pairs=[(m,u['face_index']) for u in uses]
            if not all(pair in surfaces for pair in pairs):continue
            if pairs[0]==pairs[1]:raise ValueError('self-incident spline seam is unsupported')
            ends=[boundary(u['curve']) for u in uses]
            a,b=[iso_curve(surfaces[p],u['curve']) for p,u in zip(pairs,uses)]
            same_basis_bound(a,b)
            shared=[tuple((F(x)+F(y))/2 for x,y in zip(p,q)) for p,q in zip(a['poles'],b['poles'])]
            edge_error=same_basis_bound(dict(a,poles=shared),edge['curve'])
            for pair,end in zip(pairs,ends):
                key=pair,end
                if key in used:raise ValueError('multiple joins on a complete boundary')
                used.add(key)
                for row,pole in zip(surfaces[pair]['poles'],shared):row[-1 if end else 0]=pole
            joins.append(dict(member=m,edge_index=edge['edge_index'],faces=[p[1] for p in pairs],
                              boundaries=ends,shared_curve_edge_error_exact=str(edge_error)))
    movement={}
    for pair,surface in surfaces.items():
        old=faces[pair]['surface']['poles']
        square=max(sum((F(x)-F(y))**2 for x,y in zip(p,q))
                   for a,b in zip(old,surface['poles']) for p,q in zip(a,b))
        movement[pair]=f.arithmetic.sqrt(point(square))[1]
    return surfaces,joins,movement


def orient(surface,reverse):
    degrees,knots,grid=polynomial_surface(surface)
    if any((k[0],k[-1])!=(0,1) for k in knots):raise ValueError('expected unit parameter rectangles')
    if reverse:
        grid=[list(reversed(row)) for row in grid]
        knots[1]=[1-x for x in reversed(knots[1])]
    return degrees,knots,grid


def joined(a,b,ends):
    # Orient A toward the shared edge and B away from it. Their U coordinates
    # already follow the same intrinsic edge parameter, checked by prepare().
    da,ka,ga=orient(a,ends[0]==0)
    db,kb,gb=orient(b,ends[1]==1)
    if da!=db or ka[0]!=kb[0] or len(ga)!=len(gb):raise ValueError('incompatible joined spline bases')
    if any(p[-1]!=q[0] for p,q in zip(ga,gb)):raise ValueError('joined boundary is not exactly shared')
    degree=da[1]
    grid=[p+q[1:] for p,q in zip(ga,gb)]
    # Degree-fold interior knot gives a continuous join. Neither half is
    # discarded, refitted or evaluated on an undeclared continuation.
    knots=[ka[0],ka[1][:-1]+[x+1 for x in kb[1][degree+1:]]]
    return dict(degrees=da,knots=knots,poles=grid,weights=[[1 for p in row] for row in grid])
