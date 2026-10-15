"""D0 (#339): the target programs' rate systems, integrated outside the core.

Each target is integrated the way the epic states a rate body: at every place its rates solve one
linear system, the body's algebraic rows differentiated along the formal and its rate rows, and
the instance is that flow integrated from the home. The rows are written out by hand from the
programs in `docs/rates/d0/` (nothing here reads Solvent), so this checks the *formulation*
against each closed form before any Rust exists; the gates of D1–D5 check the implementation.

    python3 docs/rates/d0/experiments.py

Needs numpy only. Integration is classical RK4 at a fixed step (the core's will be
Gragg–Bulirsch–Stoer); finite differences appear only in the stiffness estimates (E9), never in a
rate.
"""
import math
import numpy as np

np.set_printoptions(precision=6, suppress=True)


def rk4(f, y, t0, t1, n):
    h = (t1 - t0) / n
    t = t0
    for _ in range(n):
        k1 = f(t, y)
        k2 = f(t + h / 2, y + h / 2 * k1)
        k3 = f(t + h / 2, y + h / 2 * k2)
        k4 = f(t + h, y + h * k3)
        y = y + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        t += h
    return y


def cross2(a, b):
    return a[0] * b[1] - a[1] * b[0]


def stiffness(f, t, y, span):
    """The largest |λ| of the flow's Jacobian at (t, y), times the interval: how many e-folds a
    mode can make across it.  Explicit extrapolation is comfortable well below ~10³."""
    n = len(y)
    J = np.zeros((n, n))
    for j in range(n):
        h = 1e-6 * max(1.0, abs(y[j]))
        e = np.zeros(n)
        e[j] = h
        J[:, j] = (f(t, y + e) - f(t, y - e)) / (2 * h)
    lam = np.linalg.eigvals(J)
    return max(abs(lam)) * span


report = {}

# --- E1. tractrix ------------------------------------------------------------------------------
# Body: q level(up) o; o distance(a·u, along: right) q; length(a) string; rate(p over u) parallel
# string.  Unknown rates (p′, q′); four rows, rank 4 everywhere, the cusp at the home included.
a = 30.0


def tractrix(u, y):
    p, q = y[:2], y[2:]
    d = q - p
    A = np.array([
        [0, 0, 0, 1],                    # q level(up) o, differentiated
        [0, 0, 1, 0],                    # q.x = a·u, differentiated
        [-d[0], -d[1], d[0], d[1]],      # |q − p| = a, differentiated
        [d[1], -d[0], 0, 0],             # p′ × (q − p) = 0: the rate row
    ])
    b = np.array([0, a, 0, 0])
    return np.linalg.solve(A, b)


def tractrix_closed(u):
    return np.array([a * (u - math.tanh(u)), a / math.cosh(u)])


y0 = np.array([0, a, 0, 0.0])
for u in (1.2, 3.0):
    y = rk4(tractrix, y0, 0, u, 4000)
    report[f"tractrix p({u})"] = (y[:2], tractrix_closed(u), np.abs(y[:2] - tractrix_closed(u)).max())
# drift: the algebraic row |q − p| = a is only differentiated, so a coarse step leaves it
for n in (30, 300):
    y = rk4(tractrix, y0, 0, 3, n)
    report[f"tractrix drift, {n} steps"] = abs(np.linalg.norm(y[2:] - y[:2]) - a)
report["tractrix stiffness"] = stiffness(tractrix, 1.0, rk4(tractrix, y0, 0, 1, 1000), 3)

# --- E2. clothoid ------------------------------------------------------------------------------
# Body: tl = (p, q), nl = (p, k), each 10, tl angle(90deg) nl; rate rows: p′ ∥ tl, p′ along tl
# is 1, q′ along nl is 10·s/A².
A2 = 50.0


def clothoid(s, y):
    p, q, k = y[:2], y[2:4], y[4:]
    t, n = q - p, k - p
    M = np.zeros((6, 6))
    b = np.zeros(6)
    M[0, 0:2], M[0, 2:4] = -t, t                                  # |tl| = 10
    M[1, 0:2], M[1, 4:6] = -n, n                                  # |nl| = 10
    M[2, 0:2], M[2, 2:4], M[2, 4:6] = -(n + t), n, t              # tl · nl = 0
    M[3, 0:2] = [t[1], -t[0]]                                     # p′ ∥ tl
    M[4, 0:2] = t / 10; b[4] = 1                                  # unit speed
    M[5, 2:4] = n / 10; b[5] = 10 * s / A2**2                     # the tangent turns at s/A²
    return np.linalg.solve(M, b)


def fresnel(s, terms=30):
    w = s / A2
    x = sum((-1)**j * w**(4*j + 1) / (2**(2*j) * math.factorial(2*j) * (4*j + 1)) for j in range(terms))
    y = sum((-1)**j * w**(4*j + 3) / (2**(2*j + 1) * math.factorial(2*j + 1) * (4*j + 3)) for j in range(terms))
    return A2 * np.array([x, y])


y0 = np.array([0, 0, 10, 0, 0, 10.0])
y = rk4(clothoid, y0, 0, 80, 8000)
report["clothoid p(80)"] = (y[:2], fresnel(80), np.abs(y[:2] - fresnel(80)).max())
report["clothoid probe place.p (solventc)"] = np.array([67.8502, 30.3411])
y = rk4(clothoid, y0, 0, 100, 10000)
report["clothoid p(100)"] = (y[:2], fresnel(100), np.abs(y[:2] - fresnel(100)).max())
report["clothoid stiffness"] = stiffness(clothoid, 50, rk4(clothoid, y0, 0, 50, 5000), 100)

# --- E3. catenary_flow: the two-point problem by shooting ------------------------------------
# Body: q − p = (c, v + s); rate rows p′ ∥ (q − p), p′ along it is 1.  Home p = a; b reached at
# s = 150 settles c and v.  (The shooting's Jacobian here is differenced; D2's comes from the
# variational equations.)


def chain(c, v):
    def f(s, y):
        d = np.array([c, v + s])
        return d / np.linalg.norm(d)
    return f


def shoot(c, v):
    return rk4(chain(c, v), np.zeros(2), 0, 150, 3000) - np.array([100.0, 0.0])


cv = np.array([40.0, -60.0])
for _ in range(20):
    r = shoot(*cv)
    J = np.column_stack([(shoot(*(cv + e)) - shoot(*(cv - e))) / 2e-6 for e in np.eye(2) * 1e-6])
    cv = cv - np.linalg.solve(J, r)
c_closed = 30.823646973604035
report["catenary (c, v) shot"] = (cv, np.array([c_closed, -75.0]), np.abs(cv - [c_closed, -75]).max())
report["catenary stiffness"] = stiffness(lambda s, y: chain(*cv)(s, y), 75, np.array([50.0, -50.0]), 150)

# --- E4. rmf_helix ---------------------------------------------------------------------------
# Body: f.u, f.v unit, square to each other and to the tangent T(φ); rate row u′ · v = 0.
ah, bh = 10.0, 5 / (2 * math.pi)
L = math.hypot(ah, bh)


def T(phi):
    return np.array([-ah * math.sin(phi), ah * math.cos(phi), bh]) / L


def Tp(phi):
    return np.array([-ah * math.cos(phi), -ah * math.sin(phi), 0]) / L


def rmf(phi, y):
    u, v = y[:3], y[3:]
    t, tp = T(phi), Tp(phi)
    M = np.zeros((6, 6))
    b = np.zeros(6)
    M[0, :3] = t; b[0] = -u @ tp                 # u · T = 0
    M[1, 3:] = t; b[1] = -v @ tp                 # v · T = 0
    M[2, :3] = u                                 # |u| = 1
    M[3, 3:] = v                                 # |v| = 1
    M[4, :3], M[4, 3:] = v, u                    # u · v = 0
    M[5, :3] = v                                 # rate row: u′ · v = 0
    return np.linalg.solve(M, b)


u0 = np.array([1.0, 0, 0])
v0 = np.cross(T(0), u0)
y0 = np.concatenate([u0, v0])
phi1 = 1.5 * math.pi
y = rk4(rmf, y0, 0, phi1, 6000)


def frenet_angle(phi, u):
    radial = np.array([math.cos(phi), math.sin(phi), 0])
    return math.atan2(np.cross(radial, u) @ T(phi), radial @ u)


report["rmf f.u at 270deg"] = y[:3]
report["rmf probe place.f.u (solventc)"] = np.array([-0.0289679, -0.93094, 0.364022])
report["rmf turn from radius at 270deg"] = (frenet_angle(phi1, y[:3]), -phi1 * bh / L,
                                           abs(frenet_angle(phi1, y[:3]) + phi1 * bh / L))
y = rk4(rmf, y0, 0, 2 * math.pi, 8000)
report["rmf turn per turn"] = (frenet_angle(2 * math.pi, y[:3]), -2 * math.pi * bh / L)
report["rmf stiffness"] = stiffness(rmf, 1.0, rk4(rmf, y0, 0, 1, 1000), 2 * math.pi)

# --- E5. geodesic_cone -----------------------------------------------------------------------
# Body: p on the cone (gen at α to the axis), nl the cone's normal at p, tl square to it; rate
# rows p′ ∥ tl, unit speed along it, q′ parallel to the plane (tl, nl).  `w` (where the normal
# meets the axis) is algebraic and solved, not integrated; the state is p and T = (q − p)/10.
alpha = math.radians(30)


def normal(p):
    r = math.hypot(p[0], p[1])
    e = np.array([p[0] / r, p[1] / r, 0])
    return math.cos(alpha) * e - math.sin(alpha) * np.array([0, 0, 1.0])  # outward, ⟂ generator


def dnormal(p, dp):
    r = math.hypot(p[0], p[1])
    e = np.array([p[0] / r, p[1] / r, 0])
    de = np.array([dp[0], dp[1], 0]) / r - e * (e @ dp) / r
    return math.cos(alpha) * de


def geodesic(s, y):
    p, t = y[:3], y[3:]
    n = normal(p)
    dp = t                                       # p′ ∥ T, unit speed
    M = np.array([t, n, np.cross(t, n)])
    b = np.array([0, -t @ dnormal(p, dp), 0])    # |T| = 1; T · n = 0 differentiated; T′ in (T, n)
    return np.concatenate([dp, np.linalg.solve(M, b)])


r0 = 40.0
p0 = np.array([r0 * math.sin(alpha), 0, r0 * math.cos(alpha)])
y0 = np.concatenate([p0, [0, 1.0, 0]])
s1 = 30.0
y = rk4(geodesic, y0, 0, s1, 6000)
rho, psi = math.hypot(r0, s1), math.atan(s1 / r0)
ph = psi / math.sin(alpha)
closed = np.array([rho * math.sin(alpha) * math.cos(ph), rho * math.sin(alpha) * math.sin(ph), rho * math.cos(alpha)])
report["geodesic p(30)"] = (y[:3], closed, np.abs(y[:3] - closed).max())
report["geodesic T(30) against the probe's (q − p)/10"] = (y[3:], np.array([0.16 - 7, 29.12 - 24, 48.4974 - 43.3013]) / 10)
report["geodesic stiffness"] = stiffness(geodesic, 10, rk4(geodesic, y0, 0, 10, 2000), 60)

# --- E6. offset_front by the method of lines --------------------------------------------------
# Two formals: d marched, u sampled at nodes.  Rate rows at each node: x_d · x_u = 0 (the gauge)
# and x_d · n̂ = 1, n̂ = J·x_u/|x_u| the inward normal (x_u read off the nodes' polynomial).
ae, be = 40.0, 25.0
fold = be**2 / ae


def lobatto(n):
    """Chebyshev–Gauss–Lobatto nodes on [−1, 1] and their differentiation matrix."""
    k = np.arange(n)
    x = -np.cos(math.pi * k / (n - 1))
    c = np.ones(n); c[0] = c[-1] = 2
    c = c * (-1.0) ** k
    X = np.tile(x, (n, 1)).T
    D = np.outer(c, 1 / c) / (X - X.T + np.eye(n))
    D -= np.diag(D.sum(axis=1))
    return x, D


def legendre_lobatto(n):
    """Gauss–Legendre–Lobatto nodes (L2's six per piece) and their differentiation matrix."""
    P = np.polynomial.legendre.Legendre.basis(n - 1)
    x = np.concatenate([[-1], np.sort(P.deriv().roots().real), [1]])
    D = np.zeros((n, n))
    for i in range(n):
        for j in range(n):
            if i != j:
                D[i, j] = P(x[i]) / (P(x[j]) * (x[i] - x[j]))
    D[0, 0] = -n * (n - 1) / 4
    D[-1, -1] = n * (n - 1) / 4
    return x, D


def ellipse(u):
    return np.stack([ae * np.cos(u), be * np.sin(u)], axis=-1)


def inward(u):
    t = np.stack([-ae * np.sin(u), be * np.cos(u)], axis=-1)
    t /= np.linalg.norm(t, axis=-1, keepdims=True)
    return np.stack([-t[..., 1], t[..., 0]], axis=-1)


def front_flow(D):
    def f(d, y):
        x = y.reshape(-1, 2)
        xu = D @ x
        n = np.stack([-xu[:, 1], xu[:, 0]], axis=-1) / np.linalg.norm(xu, axis=1, keepdims=True)
        return n.reshape(-1)      # the 2×2 system x_d·x_u = 0, x_d·n̂ = 1 solves to x_d = n̂
    return f


u0, u1 = -math.pi / 3, math.pi / 3


def pieces_of(k):
    """L2's representation: k pieces of six Gauss–Lobatto nodes over (u0, u1), a joint node
    shared by two pieces whose derivatives there differ; the joint's rate is their mean."""
    s6, D6 = legendre_lobatto(6)
    edges = np.linspace(u0, u1, k + 1)
    nodes = []
    for i in range(k):
        un = edges[i] + (s6 + 1) / 2 * (edges[i + 1] - edges[i])
        nodes.extend(un if i == 0 else un[1:])
    nodes = np.array(nodes)
    D = np.zeros((len(nodes), len(nodes)))
    count = np.zeros(len(nodes))
    for i in range(k):
        idx = np.arange(5 * i, 5 * i + 6)
        D[np.ix_(idx, idx)] += D6 * 2 / (edges[i + 1] - edges[i])
        count[idx] += 1
    return nodes, D / count[:, None]


def one_polynomial(n):
    s, D = lobatto(n)
    return u0 + (s + 1) / 2 * (u1 - u0), D * 2 / (u1 - u0)


def periodic(n):
    """The whole ellipse, Fourier differentiation: no ends, no joints."""
    h = 2 * math.pi / n
    D = np.zeros((n, n))
    for i in range(n):
        for j in range(n):
            if i != j:
                D[i, j] = 0.5 * (-1)**(i - j) / math.tan((i - j) * h / 2)
    return np.arange(n) * h, D


def march(u, D, d, steps=1200):
    y = rk4(front_flow(D), ellipse(u).reshape(-1), 0, d, steps)
    return np.abs(y.reshape(-1, 2) - (ellipse(u) + d * inward(u))).max()


def growth(u, D, d):
    """The largest real part of the semi-discrete flow's Jacobian on the closed form at d, per mm:
    how fast its fastest mode grows.  The front's own is its greatest curvature, 1/(b²/a − d)."""
    f = front_flow(D)
    y = (ellipse(u) + d * inward(u)).reshape(-1)
    n = len(y)
    J = np.zeros((n, n))
    for j in range(n):
        e = np.zeros(n)
        e[j] = 1e-6
        J[:, j] = (f(d, y + e) - f(d, y - e)) / 2e-6
    return np.linalg.eigvals(J).real.max()


report["offset_front, quintic pieces: (pieces, nodes, error at d = 6, at d = 12)"] = [
    (k, len(pieces_of(k)[0]), march(*pieces_of(k), 6, 600), march(*pieces_of(k), 12))
    for k in (4, 8, 16, 32, 64)]
report["offset_front, one polynomial: (nodes, error at d = 6, at d = 12)"] = [
    (n, march(*one_polynomial(n), 6, 600), march(*one_polynomial(n), 12)) for n in (16, 24, 32, 48, 64)]
report["offset_front, fastest growth (per mm) at d = 0 and 12; the front's own"] = (
    [("front", 1 / fold, 1 / (fold - 12))]
    + [(f"pieces {k}", growth(*pieces_of(k), 0), growth(*pieces_of(k), 12)) for k in (4, 16, 64)]
    + [(f"one polynomial {n}", growth(*one_polynomial(n), 0), growth(*one_polynomial(n), 12)) for n in (32, 64)]
    + [(f"periodic {n}", growth(*periodic(n), 0), growth(*periodic(n), 12)) for n in (32, 64, 96)])
# the characteristics: each node carries its normal, x_d = n, n_d = 0 — no rate along u at all.
nodes, D = pieces_of(16)
xn = np.concatenate([ellipse(nodes), inward(nodes)], axis=1)
y = rk4(lambda d, y: np.concatenate([y.reshape(-1, 4)[:, 2:], np.zeros((len(nodes), 2))], axis=1).reshape(-1),
        xn.reshape(-1), 0, 12, 100).reshape(-1, 4)
report["offset_front by characteristics, error at d = 12"] = np.abs(y[:, :2] - (ellipse(nodes) + 12 * inward(nodes))).max()
# the fold: rank [x_d, x_u] = 2 is |x_u| ≠ 0 here (x_d ⟂ x_u, unit).  Marched on 32 pieces, the
# first d where a node's |x_u| falls below 1e-2 of its home value; the closed form's is
# b²/a · (1 − 1e-2) at u = 0.
nodes, D = pieces_of(32)
f = front_flow(D)
y = ellipse(nodes).reshape(-1)
home = np.linalg.norm(D @ ellipse(nodes), axis=1)
d, h = 0.0, 0.01
while np.min(np.linalg.norm(D @ y.reshape(-1, 2), axis=1) / home) > 1e-2 and d < 16:
    y = rk4(f, y, d, d + h, 1)
    d += h
report["offset_front, refused at d (a node's |x_u| below 1e-2); closed form"] = (d, fold * (1 - 1e-2))
report["offset_front stiffness (16 pieces, d = 0, over 15 mm)"] = stiffness(
    front_flow(pieces_of(16)[1]), 0, ellipse(pieces_of(16)[0]).reshape(-1), 15)
# a missing gauge: with only x_d · n̂ = 1 the rate system at a node is one row in two rates.
xu = (D @ ellipse(nodes))[3]
report["offset_front without its gauge: rank of one node's rate rows"] = np.linalg.matrix_rank(
    np.array([[-xu[1], xu[0]]]) / np.linalg.norm(xu))

# --- E7. helical_contact: conditions (2), (3) and a gauge, marched in the plane of action -----
rb1, rb2, alp, beta = 20.0, 40.0, math.radians(25), math.radians(15)
C = (rb1 + rb2) / math.cos(alp)
c1, c2 = np.zeros(3), np.array([C, 0, 0])
w1, w2 = 1.0, -rb1 / rb2
z = np.array([0, 0, 1.0])
nP = np.array([-math.cos(alp), -math.sin(alp), 0])       # the probe's P.n: c1 is rb1 along it
lP = np.array([math.sin(alp), -math.cos(alp), 0])        # the probe's P.u: the line of action
oP = c1 - rb1 * nP                                       # the probe's P.origin


def v(w, c, x):
    return w * np.cross(z, x - c)


def contact_flow(D, v1):
    def f(phi, y):
        x = y.reshape(-1, 3)
        xs = D @ x
        out = []
        for xi, si in zip(x, xs):
            n = np.cross(nP, si)
            n /= np.linalg.norm(n)               # the flank normal: in the plane of action, ⟂ x_σ
            M = np.array([nP, si, n])            # stay in P; the gauge; condition (3)
            b = np.array([0, 0, n @ v1(xi)])
            out.append(np.linalg.solve(M, b))
        return np.array(out).reshape(-1)
    return f


s, D = lobatto(6)
sig = 15 + 15 * s                                         # σ in (0, 30) along the line
g = math.cos(beta) * z + math.sin(beta) * lP
x0 = oP + np.outer(sig, g)
D = D / 15
phi1 = math.radians(20)
y = rk4(contact_flow(D, lambda x: v(w1, c1, x)), x0.reshape(-1), 0, phi1, 400).reshape(-1, 3)
# The nodes run square to the line (the gauge), so the line's own translation along the line of
# action is its normal displacement over n·l = cos βb.
nl = np.cross(nP, g)
nl /= np.linalg.norm(nl)
shift = (y - x0) @ nl / (nl @ lP)
report["helical: the line's translation along the line of action at 20deg"] = (shift, rb1 * phi1)
report["helical: off the plane / bent"] = (np.abs((y - oP) @ nP).max(),
                                           np.abs(np.cross(y - y[0], g)).max())
# condition (1) holds as a consequence, and v₁₂ vanishes where the line crosses the pitch line
x = y
n = np.cross(nP, D @ x); n /= np.linalg.norm(n, axis=1, keepdims=True)
v12 = np.array([v(w1, c1, xi) - v(w2, c2, xi) for xi in x])
report["helical: |n · v12| over the line (condition 1)"] = np.abs((n * v12).sum(axis=1)).max()
pitch_pt = c1 + (c2 - c1) * rb1 / (rb1 + rb2)
report["helical: |v12| on the pitch line"] = np.linalg.norm(v(w1, c1, pitch_pt) - v(w2, c2, pitch_pt))

# --- E8. spherical_contact --------------------------------------------------------------------
z1, z2, alp = 20, 40, math.radians(20)
d1_ = math.atan(z1 / z2)
base1 = math.asin(math.sin(d1_) * math.cos(alp))
ax1, ax2 = np.array([1.0, 0, 0]), np.array([0, 0, 1.0])
w1, w2 = 1.0, -math.tan(d1_)
pitch = np.array([math.cos(d1_), 0, math.sin(d1_)])
nP = np.array([0.420243, -0.34202, -0.840487])           # the probe's P.normal
nP -= (nP @ pitch) * pitch
nP /= np.linalg.norm(nP)


def contact_flow_sph(D):
    def f(phi, y):
        x = y.reshape(-1, 3)
        xs = D @ x
        out = []
        for xi, si in zip(x, xs):
            n = np.cross(nP, si); n /= np.linalg.norm(n)
            M = np.array([nP, si, n])
            b = np.array([0, 0, n @ (w1 * np.cross(ax1, xi))])
            out.append(np.linalg.solve(M, b))
        return np.array(out).reshape(-1)
    return f


s, D = lobatto(8)
sig = 40 + 10 * s                                         # σ from 30 to 50 from the apex
x0 = np.outer(sig, pitch)
D = D / 10
phi1 = math.radians(30)
y = rk4(contact_flow_sph(D), x0.reshape(-1), 0, phi1, 600).reshape(-1, 3)
ang = [math.atan2(np.cross(pitch, xi) @ nP, pitch @ xi) for xi in y]
report["spherical: the line's turn about the apex at 30deg"] = (np.array(ang), phi1 * math.sin(base1))
report["spherical: through the apex / straight"] = (
    np.abs(np.cross(y, y[-1] / np.linalg.norm(y[-1]))).max(), np.ptp(ang))
n_ = np.cross(nP, D @ y); n_ /= np.linalg.norm(n_, axis=1, keepdims=True)
v12 = np.array([w1 * np.cross(ax1, xi) - w2 * np.cross(ax2, xi) for xi in y])
report["spherical: |n · v12| over the line (condition 1)"] = np.abs((n_ * v12).sum(axis=1)).max()
report["spherical: |v12| along the pitch line (the home line)"] = max(
    np.linalg.norm(w1 * np.cross(ax1, k * pitch) - w2 * np.cross(ax2, k * pitch)) for k in (30, 40, 50))
report["spherical stiffness"] = stiffness(contact_flow_sph(D), 0, x0.reshape(-1), phi1)

for k, val in report.items():
    print(f"{k}:")
    print(f"    {val}")
