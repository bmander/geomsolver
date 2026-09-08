"""Exact rational tensor B-spline refinement and Bernstein polynomial arithmetic."""
from fractions import Fraction as F
from math import comb


def insert(points, knots, degree, value):
    """One exact Boehm knot insertion on homogeneous vector coefficients."""
    k = max(i for i, knot in enumerate(knots) if knot <= value)
    multiplicity = knots.count(value)
    result = [None]*(len(points)+1)
    for i in range(k-degree+1):
        result[i] = points[i]
    for i in range(k-multiplicity, len(points)):
        result[i+1] = points[i]
    for i in range(k-degree+1, k-multiplicity+1):
        alpha = (value-knots[i])/(knots[i+degree]-knots[i])
        result[i] = tuple((1-alpha)*a+alpha*b for a, b in zip(points[i-1], points[i]))
    if any(p is None for p in result):
        raise ValueError("incomplete knot insertion")
    return result, knots[:k+1]+[value]+knots[k+1:]


def refine(points, knots, degree):
    if (degree < 1 or len(knots) != len(points)+degree+1
            or knots != sorted(knots) or knots[0] >= knots[-1]
            or knots.count(knots[0]) != degree+1 or knots.count(knots[-1]) != degree+1):
        raise ValueError("expected a clamped, nonperiodic B-spline")
    for knot in sorted(set(knots))[1:-1]:
        if knots.count(knot) > degree:
            raise ValueError("discontinuous interior knot")
        while knots.count(knot) < degree:
            points, knots = insert(points, knots, degree, knot)
    return points, knots


def patches(surface):
    """Partition the complete clamped parameter rectangle without floating conversion."""
    p, q = surface["degrees"]
    uk, vk = [[F(x) for x in knots] for knots in surface["knots"]]
    if (len(surface["poles"]) != len(surface["weights"])
            or any(len(row) != len(weights) for row, weights in zip(surface["poles"], surface["weights"]))):
        raise ValueError("control point and weight dimensions disagree")
    grid = [[tuple(F(w)*F(x) for x in point)+(F(w),)
             for point, w in zip(row, weights)]
            for row, weights in zip(surface["poles"], surface["weights"])]
    if (not grid or len(grid) != len(surface["poles"]) or not grid[0]
            or any(len(row) != len(grid[0]) for row in grid)
            or any(len(point) != 4 or point[3] <= 0 for row in grid for point in row)):
        raise ValueError("invalid rational control net")
    columns = []
    for j in range(len(grid[0])):
        column, new_uk = refine([row[j] for row in grid], uk, p)
        columns.append(column)
    grid = [list(row) for row in zip(*columns)]
    grid = [refine(row, vk, q)[0] for row in grid]
    us = sorted(set(new_uk))
    vs = sorted(set(vk))
    for i in range(len(us)-1):
        for j in range(len(vs)-1):
            yield dict(domain=[[us[i], us[i+1]], [vs[j], vs[j+1]]],
                       poles=[row[j*q:j*q+q+1] for row in grid[i*p:i*p+p+1]])


def product(a, b):
    """Coefficients of the product in the tensor Bernstein basis."""
    m, n, p, q = len(a)-1, len(a[0])-1, len(b)-1, len(b[0])-1
    result = [[F(0) for _ in range(n+q+1)] for _ in range(m+p+1)]
    for i, row in enumerate(a):
        for j, av in enumerate(row):
            for k, other in enumerate(b):
                u = F(comb(m, i)*comb(p, k), comb(m+p, i+k))
                for l, bv in enumerate(other):
                    v = F(comb(n, j)*comb(q, l), comb(n+q, j+l))
                    result[i+k][j+l] += av*bv*u*v
    return result


def coordinate(grid, axis):
    return [[p[axis] for p in row] for row in grid]


def combine(*terms):
    return [[sum(scale*net[i][j] for scale, net in terms)
             for j in range(len(terms[0][1][0]))] for i in range(len(terms[0][1]))]


def bounds(net):
    values = [x for row in net for x in row]
    return min(values), max(values)


def support_bound(grid, support):
    """Upper distance to the infinite support, from the Bernstein convex hull."""
    x, y, z, w = [coordinate(grid, i) for i in range(4)]
    wmin = bounds(w)[0]
    if wmin <= 0:
        raise ValueError("nonpositive rational denominator")
    xx, yy, zz = [product(v, v) for v in (x, y, z)]
    if support["kind"] == "sphere":
        radius = F(support["radius_mm"])
        if radius <= 0:
            raise ValueError("nonpositive sphere radius")
        residual = combine((1, xx), (1, yy), (1, zz), (-radius*radius, product(w, w)))
        # |norm(p)-r| = |norm(p)^2-r^2|/(norm(p)+r), denominator >= r.
        denominator = wmin*wmin*radius
    elif support["kind"] == "cone":
        a, b = [[F(v) for v in p] for p in support["meridian"]]
        if a[1] != 0 or b[1] != 0 or min(a[0], b[0]) <= 0:
            raise ValueError("expected positive-radius rz cone meridian")
        A, B = b[2]-a[2], b[0]-a[0]
        C = A*a[0]-B*a[2]
        if A < 0:
            A, B, C = -A, -B, -C
        line = combine((B, z), (C, w))
        lmin = bounds(line)[0]
        if A <= 0 or lmin <= 0:
            raise ValueError("unproved positive cone branch")
        residual = combine((A*A, xx), (A*A, yy), (-1, product(line, line)))
        # Projection onto the rz meridian line has positive radius because
        # A > 0 and B*z+C > 0. It therefore lies on the intended cone nappe.
        # sqrt(A^2+B^2) >= max(|A|,|B|); the other conjugate factor >= line.
        denominator = wmin*lmin*max(abs(A), abs(B))
    else:
        raise ValueError("unknown support")
    lo, hi = bounds(residual)
    return max(abs(lo), abs(hi))/denominator
