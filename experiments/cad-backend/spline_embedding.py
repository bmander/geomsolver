"""Whole-rectangle injectivity via one strongly monotone linear projection."""
from fractions import Fraction as F


def dot(a,b):return sum(x*y for x,y in zip(a,b))


def projection(surface):
    grid = [[tuple(F(x) for x in p) for p in row] for row in surface['poles']]
    nu,nv = len(grid),len(grid[0])
    # Secant directions select a candidate projection; only the subsequent
    # global Jacobian inequalities can establish injectivity.
    a = [y-x for x,y in zip(grid[0][nv//2],grid[-1][nv//2])]
    b = [y-x for x,y in zip(grid[nu//2][0],grid[nu//2][-1])]
    aa,ab,bb = dot(a,a),dot(a,b),dot(b,b)
    determinant = aa*bb-ab*ab
    if determinant<=0:raise ValueError('dependent projection secants')
    return [[(bb*x-ab*y)/determinant for x,y in zip(a,b)],
            [(aa*y-ab*x)/determinant for x,y in zip(a,b)]]


def jacobian_bounds(surface,rows):
    degrees = surface['degrees']
    knots = [[F(x) for x in k] for k in surface['knots']]
    grid = [[tuple(F(x) for x in p) for p in row] for row in surface['poles']]
    if (len(degrees)!=2 or any(type(d) is not int or d<1 for d in degrees)
            or not grid or not grid[0] or any(len(r)!=len(grid[0]) for r in grid)
            or any(len(p)!=3 for r in grid for p in r)
            or len(surface['weights'])!=len(grid)
            or any(len(w)!=len(p) or any(x!=1 for x in w) for w,p in zip(surface['weights'],grid))):
        raise ValueError('expected a rectangular polynomial spatial spline')
    for d,k,n in zip(degrees,knots,(len(grid),len(grid[0]))):
        if (len(k)!=n+d+1 or k!=sorted(k) or k[0]>=k[-1]
                or k.count(k[0])!=d+1 or k.count(k[-1])!=d+1
                or any(k.count(x)>d for x in set(k[1:-1]) if x not in (k[0],k[-1]))):
            raise ValueError('expected a continuous clamped spline basis')
    derivatives = []
    for axis in (0,1):
        d,k = degrees[axis],knots[axis]
        lines = list(zip(*grid)) if axis==0 else grid
        values = []
        for line in lines:
            for i,(a,b) in enumerate(zip(line,line[1:])):
                span = k[i+d+1]-k[i+1]
                if span<=0:raise ValueError('zero derivative knot span')
                values.append(tuple(d*(y-x)/span for x,y in zip(a,b)))
        derivatives.append(values)
    return [[(min(dot(row,p) for p in values),max(dot(row,p) for p in values))
             for values in derivatives] for row in rows]


def positive_symmetric(jacobian):
    a,b = jacobian[0]
    c,d = jacobian[1]
    cross = max(abs(b[0]+c[0]),abs(b[1]+c[1]))
    determinant4 = 4*a[0]*d[0]-cross*cross
    return dict(verified=a[0]>0 and d[0]>0 and determinant4>0,
                diagonal_lower=[a[0],d[0]],symmetric_determinant_times_four_lower=determinant4)


def check(surface,domain=None):
    rows = projection(surface)
    jacobian = jacobian_bounds(surface,rows)
    if domain is not None and domain!=[[k[0],k[-1]] for k in surface['knots']]:
        raise ValueError('trim rectangle differs from the complete clamped domain')
    result = positive_symmetric(jacobian)
    scale = F(1)
    if not result['verified']:
        b = max(map(abs,jacobian[0][1]))
        c = max(map(abs,jacobian[1][0]))
        if b>0 and c>0:
            # Fix one positive output scaling for the entire rectangle. This
            # balances the two off-diagonal bounds without changing the
            # projected map's injectivity. No cell-specific projection is used.
            scale = b/c
            rows[1] = [scale*x for x in rows[1]]
            jacobian[1] = [tuple(scale*x for x in interval) for interval in jacobian[1]]
            result = positive_symmetric(jacobian)
    c1_basis = all(k.count(x)<d for d,k in zip(surface['degrees'],surface['knots'])
                   for x in set(k) if x not in (k[0],k[-1]))
    return dict(**result,projection=rows,jacobian_bounds=jacobian,second_output_scale=scale,
                regular_everywhere=result['verified'] and c1_basis)
