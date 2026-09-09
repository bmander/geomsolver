"""Shared ideal common-crown characteristic and member-coordinate transforms."""
from interval_jet import Jet


def radius(data, u):
    return data["rho_range"][0]+u*(Jet(data["rho_range"][1])-data["rho_range"][0])


def section(data, profile, derivative, rho):
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
    return r, h, dr, dh, ct, st, x, y, A, B


def generated(data, record, profile, derivative, rho):
    r, h, dr, dh, ct, st, x, y, A, B = section(data, profile, derivative, rho)
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
