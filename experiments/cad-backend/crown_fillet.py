"""Independent common-crown fillet parameterization, including interval derivatives."""
from fractions import Fraction as F
from interval_jet import Jet, f, point


def check_samples(data, source, record):
    if (data["teeth"] != [m["teeth"] for m in source["members"]]
            or data["module_mm"] != source["module_mm"]
            or data["rho_range"] != [.9*source["mean_distance"], 1.1*source["mean_distance"]]):
        raise ValueError("nominal reference disagrees with the CAD source parameters")
    member = source["members"][record["member"]]
    level = next(l for l in member["levels"] if l["subdivisions"] == 32)
    maximum = F(0)
    # The source grids precede the gear-side indexing used to form its tooth space.
    raw = dict(record, index_angle=0.)
    for i in (0, 16, 32):
        for j in (0, 16, 32):
            expected = level["sides"][record["side"]][i][j]
            found = evaluate(data, raw, Jet(F(i, 32)), Jet(F(j, 32)))
            difference = [f.sub(value.v, point(x)) for value, x in zip(found, expected)]
            bound = f.arithmetic.sqrt(point(sum(max(abs(x) for x in d)**2 for d in difference)))[1]
            maximum = max(maximum, bound)
    if maximum > F("0.0000001"):
        raise ValueError("independent reference misses the original source samples")
    return maximum


def evaluate(data, record, u, v):
    rho = data["rho_range"][0]+u*(Jet(data["rho_range"][1])-data["rho_range"][0])
    parameter = record["u_range"][0]+v*(record["u_range"][1]-record["u_range"][0])
    sine, cosine = (parameter*record["sweep"]).sin_cos()
    a, b, center = [[Jet(c) for c in record[key]] for key in ("a", "b", "center")]
    profile = [c+x*cosine+y*sine for c, x, y in zip(center, a, b)]
    derivative = [(-x*sine+y*cosine)*record["sweep"] for x, y in zip(a, b)]
    cx, cy, _ = data["crown_center"]
    dx, dy = profile[0]-cx, profile[1]-cy
    if dx.v[0] <= 0:
        raise ValueError("unproved positive meridian radius branch")
    transverse = dy/dx
    factor = (1+transverse*transverse).sqrt()
    r, h = dx*factor, profile[2]
    dr, dh = (derivative[0]+transverse*derivative[1])/factor, derivative[2]
    c = (Jet(cx)*cx+Jet(cy)*cy).sqrt()
    q = (rho*rho-h*h-c*c-r*r)/(2*c*r)
    root = (1-q*q).sqrt()
    ct, st = (cx*q+cy*root)/c, (cy*q-cx*root)/c
    if ct.v[0] <= 0 or st.v[1] >= 0:
        raise ValueError("unproved source semicircle branch")
    x, y = cx+r*ct, cy+r*st
    # Cancel the common meridian-normal scale before enclosing the roll ratio.
    # Division refuses any cell in which the radial derivative could vanish.
    normal_intercept = r+h*dh/dr
    A, B = cx+normal_intercept*ct, cy+normal_intercept*st
    t = (-B/A).atan()
    if t.v[0] < record["roll_domain"][0] or t.v[1] > record["roll_domain"][1]:
        raise ValueError("unproved finite generating roll domain")
    sine, cosine = t.sin_cos()
    x, y = cosine*x-sine*y, sine*x+cosine*y
    member = record["member"]
    sign = 1 if member == 0 else -1
    n, g = data["teeth"][member], data["teeth"][1-member]
    size = (Jet(n)*n+Jet(g)*g).sqrt()
    sd, cd = Jet(n)/size, Jet(g)/size
    x, z = sign*sd*x-cd*h, cd*x+sign*sd*h
    sine, cosine = (-sign*t/sd+record["index_angle"]).sin_cos()
    return [cosine*x-sine*y, sine*x+cosine*y, z]
