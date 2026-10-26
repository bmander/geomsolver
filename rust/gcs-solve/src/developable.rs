//! **A developable panel** (#645): the ruled set one plane touches along each ruling, between two
//! paths, built exactly — and its **development**, the panel laid flat in a plane.
//!
//! `std.panels.Developable(a, b)` says what a panel is: a point `m` on `a`, a point `n` on `b`, a
//! plane `T` touching `a` at `m` and `b` at `n`, `b` running through it the way `a` does, and the
//! set's point on the ruling between them.  The language reads that body as any set's; the core
//! recognises it (`gcs-core`'s recogniser, by what the body says) and builds the panel here.
//!
//! **The correspondence.**  A ruling joins `A = a(u)` to `B = b(w)` where one plane holds `a′(u)`,
//! `b′(w)` and `B − A`: `F(u, w) = det(a′, b′, B − A) = 0`.  The rulings are a curve in `(u, w)`,
//! the level set of `F`, marched as its own flow — `(u′, w′) ∝ (F_w, −F_u)`, which keeps `F`
//! where it started — in the panel's **running number** `σ`, the two rails' arc lengths together
//! (`|a′|u′ + |b′|w′ = 1`), so it never stalls where one rail stands still.  The flow is followed
//! by the core's integrator (`ode`), to its tolerance; a ruling read anywhere is one step from the
//! node before.
//!
//! **Which root.**  Two rails touched by one plane have, at a place on `a`, a ruling on each side
//! of the tangent: where `b` runs through the plane the way `a` does (`sense: same`, `a′ × r` and
//! `b′ × r` one way, `r = B − A`) and the crossing one, a bow-tie that cuts through itself.  The
//! home ruling is the shortest of the first kind from the point given on `a`; the march carries
//! that branch, and a place where `u` or `w` would turn back is a **fold**, refused.
//!
//! **Corners fan.**  At a kink of a path a plane touches it where it *supports* it: between its
//! two one-sided tangents.  So at a kink of `a` the ruling's foot stands at the corner while `b`
//! runs on, from where the plane holds the tangent the march came by to where it holds the one it
//! leaves by (`det(a′₊, b′, B − A) = 0`): a **fan**, a piece of a cone about the corner.  A rail's
//! **end** is a corner whose other side is open: where the regular rulings reach the end of one
//! rail before the other's, the end's fan takes the rest of the other.  A rectangle to a round is
//! four planar triangles (a straight side's rulings meet at one point of the round) and four fans.
//!
//! **Closed rails** run one way round from the home, and must come round to it (`E080`).
//!
//! **Sides.**  The panel's normal is `a′ × r` normalised — `b′ × r` on a fan about a corner of `a`
//! — so its **outside** is the order of the rails: the other order turns it.  A distance from the
//! panel is measured along that normal from a foot on it, and a band `0 ≤ s ≤ d` is square at its
//! edges.
//!
//! **The development.**  Along a ruling the panel's normal is one, so the strip between two
//! neighbouring rulings is planar and lays flat congruent.  The layer `d` along the normal has the
//! same rulings moved by `dN`; laid in a plane with the outside up, the image `f` of its foot on
//! `a` and its ruling's direction `e(φ)` run as
//!
//! `φ′ = N · (ĝ × ĝ′)`,   `f′ = (m′_d · ĝ) e(φ) + (m′_d · β) e⊥(φ)`,   `g = f + |n − m| e(φ)`,
//!
//! `ĝ` the ruling's direction, `β = N × ĝ`, `m′_d` the layer's foot's rate.  That is the exact
//! development (`Developable::develop`).  The **check** (`develop_generic`) marches the body's
//! own rows instead — each ruling as long as its image, each rail as fast as its image, `a` meeting
//! each ruling at its image's angle — solved by Newton for the rates at every evaluation: on every
//! smooth panel the two agree, and the rows holding at every node is the property.
use crate::ode;
use crate::path::{Path, V};
use crate::space::{add, cross, dot, norm, normalised, scale, sub};

/// Below this sine a ruling and a rail's tangent are one direction: the plane through them is
/// none.
const TANGENT: f64 = 1e-9;

/// Below this sine the home ruling runs along a rail's tangent: its root is found to a part in a
/// hundred million, and a double root of the correspondence — the ruling along the rail — only to
/// its square root.
const ALONG: f64 = 1e-6;

/// A fraction of the gauge either rail may run backward and still be read as standing.
const FOLD: f64 = 1e-7;

/// How far, relatively, a closed panel may miss its home coming round and still close.
const CLOSE: f64 = 1e-7;

/// A ruling this short, relative to the panel's size, is a point: where the rails' ends meet the
/// panel comes to one, and the march stops there.  The correspondence's level set crosses itself
/// at the meeting (a saddle of `F`), and nearer it a march's drift off the level set is no longer
/// small against the ruling — the laid march, which carries the feet again, would leave it.
const MEET: f64 = 1e-5;

/// Samples a rail's interval is searched at for the home's roots.
const ROOT_SAMPLES: usize = 720;

/// The most pieces a panel is marched in: a document is untrusted input.
const MAX_PIECES: usize = 4096;

/// Why a panel cannot be built, and where: said once, at the use (`E080`), naming the place on
/// `a` (`place`, in space; `None` where the refusal has no place).
#[derive(Clone, Debug, PartialEq)]
pub struct Refusal {
    pub why: String,
    pub place: Option<V>,
}

impl Refusal {
    fn at(why: impl Into<String>, place: V) -> Refusal {
        Refusal { why: why.into(), place: Some(place) }
    }

    fn whole(why: impl Into<String>) -> Refusal {
        Refusal { why: why.into(), place: None }
    }
}

impl std::fmt::Display for Refusal {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self.place {
            Some(p) => write!(f, "{} where `a` is at ({}, {}, {})", self.why,
                              crate::vocab::num(p[0]), crate::vocab::num(p[1]),
                              crate::vocab::num(p[2])),
            None => write!(f, "{}", self.why),
        }
    }
}

/// What a piece of the panel is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Piece {
    /// Both rails run: the regular rulings.
    Smooth,
    /// The foot on `a` stands at a corner or an end of `a` while `b` runs: a fan about it.
    FanA,
    /// The foot on `b` stands while `a` runs.
    FanB,
}

/// One ruling: where it is on the running number, its feet's places on the rails, its two ends,
/// the panel's unit normal along it, and the piece it is of.
#[derive(Clone, Copy, Debug)]
pub struct Ruling {
    pub sigma: f64,
    pub u: f64,
    pub w: f64,
    pub m: V,
    pub n: V,
    pub normal: V,
    pub piece: Piece,
}

/// A rail as the march reads it: a path, its interval, where its pieces meet, and — closed — its
/// period, read at places past its interval round again.
#[derive(Clone, Debug)]
struct Rail {
    path: Path,
    lo: f64,
    hi: f64,
    closed: bool,
    /// Its kinks and its spline's knots, in order: no step of a march crosses one, where the flow
    /// turns a corner in some order (a fan stands only at a kink, `needs_fan`).
    joins: Vec<f64>,
}

impl Rail {
    fn new(path: Path) -> Rail {
        let (lo, hi) = path.domain();
        let closed = path.closed();
        let mut joins = path.kinks();
        joins.extend(path.joins());
        joins.sort_by(f64::total_cmp);
        joins.dedup_by(|x, y| (*x - *y).abs() <= 1e-12 * (1.0 + x.abs()));
        Rail { path, lo, hi, closed, joins }
    }

    fn period(&self) -> f64 {
        self.hi - self.lo
    }

    /// The place `x` (unwrapped) moved into the interval, with the anchor `at` moved alike.
    fn wrapped(&self, x: f64, at: f64) -> (f64, f64) {
        if !self.closed {
            return (x, at);
        }
        let p = self.period();
        let shift = ((at - self.lo) / p).floor() * p;
        (x - shift, at - shift)
    }

    /// The point and its first two derivatives at `x`, on the smooth piece `at` stands on.
    fn jet(&self, x: f64, at: f64) -> Result<[V; 3], String> {
        let (x, at) = self.wrapped(x, at);
        let j = self.path.jet_near(x, at, false)?;
        Ok([j[0], j[1], j[2]])
    }

    /// Its breaks in order — kinks and knots, and an open rail's ends — over the unwrapped places
    /// `from` to `to` (either way round), each period's again.
    fn breaks(&self, from: f64, to: f64) -> Vec<f64> {
        let (lo, hi) = (from.min(to), from.max(to));
        let kinks = &self.joins;
        let mut out = Vec::new();
        if self.closed {
            let p = self.period();
            let k0 = ((lo - self.lo) / p).floor() as i64 - 1;
            let k1 = ((hi - self.lo) / p).ceil() as i64 + 1;
            for k in k0..=k1 {
                for &q in kinks {
                    out.push(q + k as f64 * p);
                }
            }
        } else {
            out.extend_from_slice(kinks);
            out.push(self.lo);
            out.push(self.hi);
        }
        out.retain(|&q| q >= lo - 1e-12 * (1.0 + q.abs()) && q <= hi + 1e-12 * (1.0 + q.abs()));
        out.sort_by(f64::total_cmp);
        out.dedup_by(|x, y| (*x - *y).abs() <= 1e-12 * (1.0 + x.abs()));
        out
    }

    /// The smooth piece's anchor ahead of `x` the way `dir` runs: the middle between the break at
    /// or behind it and the next ahead.
    fn anchor(&self, x: f64, dir: f64) -> f64 {
        let span = self.period().max(1e-300);
        let bs = self.breaks(x - 2.0 * span, x + 2.0 * span);
        let tol = 1e-12 * (1.0 + x.abs()) * span.max(1.0);
        let next = if dir > 0.0 {
            bs.iter().copied().find(|&q| q > x + tol)
        } else {
            bs.iter().rev().copied().find(|&q| q < x - tol)
        };
        let back = if dir > 0.0 {
            bs.iter().rev().copied().find(|&q| q <= x + tol)
        } else {
            bs.iter().copied().find(|&q| q >= x - tol)
        };
        match (back, next) {
            (Some(p), Some(q)) => 0.5 * (p + q),
            (None, Some(q)) => 0.5 * (x + q),
            (Some(p), None) => 0.5 * (x + p),
            (None, None) => x,
        }
    }

    /// The one-sided tangent leaving `x` the way `dir` runs (into the march), or arriving at it.
    fn tangent(&self, x: f64, dir: f64, leaving: bool) -> Result<V, String> {
        let side = if leaving == (dir > 0.0) { 1.0 } else { -1.0 };
        Ok(self.jet(x, self.anchor(x, side))?[1])
    }
}

/// Whether a corner of `fan` at `x` fans, `y` the place on `other`: whether the plane the march
/// came by fails to touch `fan` along the tangent it leaves by (a sign of `det` says nothing).
fn needs_fan(fan: &Rail, other: &Rail, x: f64, y: f64, dir: f64) -> Result<bool, Refusal> {
    let out = fan.tangent(x, dir, true).map_err(Refusal::whole)?;
    let [po, o1, _] = other.jet(y, other.anchor(y, dir)).map_err(Refusal::whole)?;
    let r = sub(po, fan.path.point(fan.wrapped(x, x).0));
    let (Some(t), Some(oo), Some(rr)) = (normalised(out), normalised(o1), normalised(r)) else {
        return Ok(false);
    };
    Ok(det(t, oo, rr).abs() > 1e-10)
}

/// `det(a, b, c)`.
fn det(a: V, b: V, c: V) -> f64 {
    dot(a, cross(b, c))
}

/// The correspondence's flow on one piece: the running number's rate of `u` and `w`, and — for a
/// development — of `φ` and `f` (`d` the layer), and — for the check — of `f` and `g` read off
/// the body's rows.
struct Flow<'r> {
    a: &'r Rail,
    b: &'r Rail,
    piece: Piece,
    /// The pieces of each rail the piece stands on.
    at_a: f64,
    at_b: f64,
    /// What else is carried: nothing, the exact development, or the check.
    carry: Carry,
    d: f64,
    /// The check's rates as last solved: the branch its Newton starts from.
    rates: [f64; 4],
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Carry {
    Nothing,
    Exact,
    Generic,
}

/// What the flow reads at a place: the feet, their rates, the normal and its rate, the ruling.
struct Local {
    m: V,
    n: V,
    dm: V,
    dn: V,
    normal: V,
    dnormal: V,
    du: f64,
    dw: f64,
}

impl<'r> Flow<'r> {
    /// The flow on the marched piece `p` of the panel between `a` and `b`, carrying nothing.
    fn on(a: &'r Rail, b: &'r Rail, p: &Marched) -> Flow<'r> {
        Flow { a, b, piece: p.piece, at_a: p.at_a, at_b: p.at_b, carry: Carry::Nothing, d: 0.0,
               rates: [0.0; 4] }
    }

    fn width(&self) -> usize {
        match self.carry {
            Carry::Nothing => 2,
            Carry::Exact => 5,
            Carry::Generic => 6,
        }
    }

    /// The panel at `(u, w)`: its feet, their rates in the running number, its normal and rate.
    fn local(&self, u: f64, w: f64) -> Result<Local, String> {
        let [pa, a1, a2] = self.a.jet(u, self.at_a)?;
        let [pb, b1, b2] = self.b.jet(w, self.at_b)?;
        let r = sub(pb, pa);
        let (du, dw) = match self.piece {
            Piece::Smooth => {
                let fu = det(a2, b1, r);
                let fw = det(a1, b2, r);
                let gauge = norm(a1) * fw - norm(b1) * fu;
                if gauge.abs() < 1e-300 {
                    return Err("fold".into());
                }
                (fw / gauge, -fu / gauge)
            }
            Piece::FanA => (0.0, 1.0 / norm(b1).max(1e-300)),
            Piece::FanB => (1.0 / norm(a1).max(1e-300), 0.0),
        };
        if du * norm(a1) < -FOLD || dw * norm(b1) < -FOLD {
            return Err("fold".into());
        }
        let (dm, dn) = (scale(a1, du), scale(b1, dw));
        let dr = sub(dn, dm);
        let (raw, draw) = match self.piece {
            Piece::FanA => (cross(b1, r), add(cross(scale(b2, dw), r), cross(b1, dr))),
            _ => (cross(a1, r), add(cross(scale(a2, du), r), cross(a1, dr))),
        };
        let len = norm(raw);
        if !(len > TANGENT * norm(r) * norm(if self.piece == Piece::FanA { b1 } else { a1 })) {
            return Err("tangent".into());
        }
        let normal = scale(raw, 1.0 / len);
        let dnormal = scale(sub(draw, scale(normal, dot(normal, draw))), 1.0 / len);
        Ok(Local { m: pa, n: pb, dm, dn, normal, dnormal, du, dw })
    }

    /// A march's node set back on the level set `F = 0` by a Newton step along `∇F`: a step's
    /// drift off it, small elsewhere, is not small against the ruling where the rails meet, the
    /// level set crossing itself there.  A fan has no level set: it runs along one rail.
    fn settle(&self, y: &mut [f64]) {
        if self.piece != Piece::Smooth {
            return;
        }
        let (Ok([pa, a1, a2]), Ok([pb, b1, b2])) = (self.a.jet(y[0], self.at_a),
                                                    self.b.jet(y[1], self.at_b)) else { return };
        let r = sub(pb, pa);
        let (f, fu, fw) = (det(a1, b1, r), det(a2, b1, r), det(a1, b2, r));
        let g2 = fu * fu + fw * fw;
        if g2 > 0.0 {
            y[0] -= f * fu / g2;
            y[1] -= f * fw / g2;
        }
    }
}

/// The layer `d` along the normal: its feet's rates, its ruling's direction and rate, and its
/// length (the panel's: the two feet move by one `d·N`).
struct Layer {
    dmo: V,
    dno: V,
    g: V,
    dg: V,
    len: f64,
}

fn layer(l: &Local, d: f64) -> Layer {
    let dmo = add(l.dm, scale(l.dnormal, d));
    let dno = add(l.dn, scale(l.dnormal, d));
    let r = sub(l.n, l.m);
    let len = norm(r);
    let g = scale(r, 1.0 / len);
    let dr = sub(l.dn, l.dm);
    let dg = scale(sub(dr, scale(g, dot(g, dr))), 1.0 / len);
    Layer { dmo, dno, g, dg, len }
}

impl ode::Rate for Flow<'_> {
    type Branch = [f64; 4];

    fn width(&self) -> usize {
        Flow::width(self)
    }

    fn rate(&mut self, _s: f64, y: &[f64], f: &mut [f64], jac: Option<&mut [f64]>) -> bool {
        if jac.is_some() {
            return false;
        }
        let Ok(l) = self.local(y[0], y[1]) else { return false };
        f[0] = l.du;
        f[1] = l.dw;
        match self.carry {
            Carry::Nothing => {}
            Carry::Exact => {
                let Layer { dmo, g, dg, .. } = layer(&l, self.d);
                let beta = cross(l.normal, g);
                let phi = y[2];
                let (sn, cs) = phi.sin_cos();
                let (along, across) = (dot(dmo, g), dot(dmo, beta));
                f[2] = dot(l.normal, cross(g, dg));
                f[3] = along * cs - across * sn;
                f[4] = along * sn + across * cs;
            }
            Carry::Generic => {
                let Some(r) = self.generic_rates(&l, y) else { return false };
                self.rates = r;
                f[2..6].copy_from_slice(&r);
            }
        }
        f.iter().all(|v| v.is_finite())
    }

    fn branch(&self) -> [f64; 4] {
        self.rates
    }

    fn set_branch(&mut self, b: [f64; 4]) {
        self.rates = b;
    }
}

impl Flow<'_> {
    /// **The check**: `f′` and `g′` solved from the body's own rows by Newton from the rates
    /// before — `|f′| = |m′|`, `|g′| = |n′|`, `f′` meeting the ruling's image at the angle `m′`
    /// meets the ruling, and the ruling's image keeping the ruling's length (`line(f, g) equal
    /// line(m, n)` differentiated) — four rows in the four rates.
    fn generic_rates(&self, l: &Local, y: &[f64]) -> Option<[f64; 4]> {
        let Layer { dmo, dno, g: rh, len, .. } = layer(l, self.d);
        let (f, g) = ([y[2], y[3]], [y[4], y[5]]);
        let e = [g[0] - f[0], g[1] - f[1]];
        let el = (e[0] * e[0] + e[1] * e[1]).sqrt();
        let eh = [e[0] / el, e[1] / el];
        let sm = norm(dmo);
        let cm = dot(dmo, rh);
        // the ruling's length kept, differentiated: `r·r′`, the layer's ruling the panel's moved
        let lhs4 = len * dot(rh, sub(dno, dmo));
        let speed_n = dot(dno, dno);
        let still = sm <= 1e-12 * norm(dno).max(1e-300);
        let mut x = self.rates;
        for _ in 0..30 {
            let (fx, fy, gx, gy) = (x[0], x[1], x[2], x[3]);
            let sf = (fx * fx + fy * fy).sqrt().max(1e-300);
            let mut res = [
                fx * fx + fy * fy - sm * sm,
                gx * gx + gy * gy - speed_n,
                (fx * eh[0] + fy * eh[1]) * sm - cm * sf,
                e[0] * (gx - fx) + e[1] * (gy - fy) - lhs4,
            ];
            let mut j = [
                [2.0 * fx, 2.0 * fy, 0.0, 0.0],
                [0.0, 0.0, 2.0 * gx, 2.0 * gy],
                [eh[0] * sm - cm * fx / sf, eh[1] * sm - cm * fy / sf, 0.0, 0.0],
                [-e[0], -e[1], e[0], e[1]],
            ];
            if still {
                // the foot stands (a fan about a corner of `a`, the layer at the panel): its
                // image stands too, and the angle row says nothing
                (res[0], res[2]) = (fx, fy);
                (j[0], j[2]) = ([1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]);
            }
            let mut a: Vec<f64> = j.iter().flatten().copied().collect();
            let mut dx = res;
            if !crate::linalg::lu_solve(4, &mut a, &mut dx) {
                return None;
            }
            let mut big = 0.0f64;
            for k in 0..4 {
                x[k] -= dx[k];
                big = big.max(dx[k].abs());
            }
            if big <= 1e-15 * (1.0 + x.iter().fold(0.0f64, |m, v| m.max(v.abs()))) {
                break;
            }
        }
        Some(x)
    }
}

/// One piece as marched: what it is, the pieces of each rail it stands on, and its nodes in the
/// running number — a ruling anywhere on it is one step from the node before.
#[derive(Clone, Debug)]
struct Marched {
    piece: Piece,
    at_a: f64,
    at_b: f64,
    /// `(σ, u, w)` at each node, in the order marched (σ decreasing on the march back).
    nodes: Vec<(f64, f64, f64)>,
}

impl Marched {
    fn span(&self) -> (f64, f64) {
        let (a, b) = (self.nodes[0].0, self.nodes[self.nodes.len() - 1].0);
        (a.min(b), a.max(b))
    }
}

/// **A panel, built**: its rails, its pieces in order of the running number, and where the home
/// ruling is on it (`σ = 0`).
#[derive(Clone, Debug)]
pub struct Developable {
    a: Rail,
    b: Rail,
    pieces: Vec<Marched>,
    /// The running number's interval: from the start's end to the finish's, the home at 0.
    span: (f64, f64),
}

impl Developable {
    /// **The panel between `a` and `b` whose home ruling stands at `s` on `a`**: its correspondence
    /// marched out from there both ways to the rails' ends, or round once where both are closed.
    pub fn build(a: Path, b: Path, s: V) -> Result<Developable, Refusal> {
        let (a, b) = (Rail::new(a), Rail::new(b));
        if a.closed != b.closed {
            return Err(Refusal::whole(format!("one rail is closed and the other open: a panel \
                between them has an end on one and none on the other (`{}` is closed)",
                if a.closed { "a" } else { "b" })));
        }
        if !(a.hi > a.lo) || !(b.hi > b.lo) {
            return Err(Refusal::whole("a rail runs over no interval"));
        }
        let u0 = a.path.nearest(s).map_err(Refusal::whole)?;
        let gap = norm(sub(a.path.point(u0), s));
        let size = a.path.size().max(b.path.size()).max(1.0);
        if gap > 1e-6 * size {
            return Err(Refusal::at(format!("the home ruling stands on `a`, and the point given \
                is {} from it", crate::vocab::num(gap)), s));
        }
        let (w0, pa, a1) = home_root(&a, &b, u0)?;
        // a ruling along a rail's tangent has no plane through the two: refused, not marched
        {
            let pb = b.path.point(b.wrapped(w0, w0).0);
            let b1 = b.jet(w0, b.anchor(w0, 1.0)).map_err(Refusal::whole)?[1];
            let r = sub(pb, pa);
            let along = |t: V| norm(cross(t, r)) <= ALONG * norm(t) * norm(r);
            if along(a1) || along(b1) {
                return Err(Refusal::at("a ruling is tangent to a rail: no plane holds the rail's \
                    tangent and the ruling as two directions", pa));
            }
        }
        let mut dev = Developable { a, b, pieces: Vec::new(), span: (0.0, 0.0) };
        if dev.a.closed {
            let ahead = dev.march(u0, w0, 1.0)?;
            let last = ahead.last().and_then(|p| p.nodes.last().copied()).unwrap_or((0.0, u0, w0));
            let miss = (last.2 - (w0 + dev.b.period())).abs();
            if (last.1 - (u0 + dev.a.period())).abs() > CLOSE * dev.a.period()
                || miss > CLOSE * dev.b.period()
            {
                return Err(Refusal::at("the rulings do not come round: marched once round `a` \
                    from the home, they do not come back to the home ruling", dev.a.path.point(u0)));
            }
            dev.span = (0.0, last.0);
            dev.pieces = ahead;
        } else {
            let mut back = dev.march(u0, w0, -1.0)?;
            let ahead = dev.march(u0, w0, 1.0)?;
            let lo = back.last().and_then(|p| p.nodes.last()).map_or(0.0, |n| n.0);
            let hi = ahead.last().and_then(|p| p.nodes.last()).map_or(0.0, |n| n.0);
            // the pieces in order of the running number, each its nodes as marched: a ruling is
            // read one step on from the node the march came by, never back from where it stopped
            back.reverse();
            back.extend(ahead);
            dev.pieces = back;
            dev.span = (lo, hi);
        }
        if dev.pieces.is_empty() || !(dev.span.1 > dev.span.0) {
            return Err(Refusal::whole("the panel has no rulings: its rails meet nowhere a plane \
                touches both"));
        }
        Ok(dev)
    }

    /// The march from `(u0, w0)` the way `dir` runs, piece after piece, to the rails' ends or —
    /// closed — once round.
    fn march(&self, u0: f64, w0: f64, dir: f64) -> Result<Vec<Marched>, Refusal> {
        let (a, b) = (&self.a, &self.b);
        let stop_u = if a.closed { Some(u0 + dir * a.period()) } else { None };
        let (mut u, mut w, mut sigma) = (u0, w0, 0.0);
        let mut out: Vec<Marched> = Vec::new();
        let size = a.path.size().max(b.path.size()).max(1.0);
        let at_end = |r: &Rail, x: f64| !r.closed && ((dir > 0.0 && x >= r.hi - 1e-12 * size)
            || (dir < 0.0 && x <= r.lo + 1e-12 * size));
        let round = |u: f64| stop_u.is_some_and(|s| dir * (u - s) >= -1e-12 * a.period());
        // where the rails' ends meet, the ruling shrinks to a point: the panel's end
        let ends_meet = !a.closed && !b.closed && {
            let end = |r: &Rail| r.path.point(if dir > 0.0 { r.hi } else { r.lo });
            norm(sub(end(b), end(a))) <= MEET * size
        };
        let met = |u: f64, w: f64| -> Result<bool, Refusal> {
            let (pa, pb) = (a.path.point(a.wrapped(u, u).0), b.path.point(b.wrapped(w, w).0));
            if norm(sub(pb, pa)) > 1.5 * MEET * size {
                Ok(false)
            } else if ends_meet {
                Ok(true)
            } else {
                Err(Refusal::at("the rails meet short of their ends: the panel pinches to a point \
                    there", pa))
            }
        };
        // a fan already taken at the corner the march stands on
        let (mut skip_a, mut skip_b) = (false, false);
        while out.len() < MAX_PIECES {
            let (ea, eb) = (at_end(a, u), at_end(b, w));
            if (ea && eb) || round(u) || met(u, w)? {
                break;
            }
            // which piece stands here: an end's fan, a corner's fan, or the regular rulings
            let on_break = |r: &Rail, x: f64| r.breaks(x - 1e-9, x + 1e-9).iter()
                .any(|&q| (q - x).abs() <= 1e-10 * (1.0 + x.abs()));
            let piece = if ea {
                Piece::FanA
            } else if eb {
                Piece::FanB
            } else if !skip_a && on_break(a, u) && needs_fan(a, b, u, w, dir)? {
                Piece::FanA
            } else if !skip_b && on_break(b, w) && needs_fan(b, a, w, u, dir)? {
                Piece::FanB
            } else {
                Piece::Smooth
            };
            let at_a = a.anchor(u, dir);
            let at_b = b.anchor(w, dir);
            let m = self.piece(piece, at_a, at_b, u, w, sigma, dir, ea || eb, stop_u)?;
            let &(s1, u1, w1) = m.nodes.last().expect("a piece has its start");
            let (moved_u, moved_w) = (u1 != u, w1 != w);
            if moved_u || moved_w {
                out.push(m);
            } else if piece == Piece::Smooth || ea || eb {
                // nothing moved: the march stands where it cannot go on
                return Err(Refusal::at("the rulings stop: no plane touches both rails past here",
                                       a.path.point(a.wrapped(u, u).0)));
            }
            if moved_u {
                skip_a = false;
            }
            if moved_w {
                skip_b = false;
            }
            skip_a |= piece == Piece::FanA;
            skip_b |= piece == Piece::FanB;
            u = u1;
            w = w1;
            sigma = s1;
        }
        Ok(out)
    }

    /// One piece marched from `(u, w)` at `sigma` the way `dir` runs, until a rail meets a break,
    /// a fan's plane touches its corner along the tangent it leaves by, or the march is round.
    #[allow(clippy::too_many_arguments)]
    fn piece(&self, piece: Piece, at_a: f64, at_b: f64, u: f64, w: f64, sigma: f64, dir: f64,
             end_fan: bool, stop_u: Option<f64>) -> Result<Marched, Refusal> {
        let (a, b) = (&self.a, &self.b);
        let size = a.path.size().max(b.path.size()).max(1.0);
        // the breaks ahead on each rail, within this piece's reach
        let next_break = |r: &Rail, x: f64| -> Option<f64> {
            let reach = if r.closed { 2.0 * r.period() } else { r.period() + 1.0 };
            let bs = r.breaks(x - reach, x + reach);
            let tol = 1e-10 * (1.0 + x.abs());
            if dir > 0.0 { bs.into_iter().find(|&q| q > x + tol) }
            else { bs.into_iter().rev().find(|&q| q < x - tol) }
        };
        let mut ua = match piece { Piece::FanA => None, _ => next_break(a, u) };
        if piece != Piece::FanA {
            if let Some(stop) = stop_u {
                ua = Some(match ua { Some(q) if dir * (q - stop) < 0.0 => q, _ => stop });
            }
        }
        let wb = match piece { Piece::FanB => None, _ => next_break(b, w) };
        // a fan about a corner ends where the plane holds the tangent the march leaves by
        let leave = match piece {
            Piece::FanA if !end_fan => Some(a.tangent(u, dir, true).map_err(Refusal::whole)?),
            Piece::FanB if !end_fan => Some(b.tangent(w, dir, true).map_err(Refusal::whole)?),
            _ => None,
        };
        let corner_a = a.path.point(a.wrapped(u, u).0);
        let corner_b = b.path.point(b.wrapped(w, w).0);
        let event = |y: &[f64]| -> Vec<f64> {
            let mut e = Vec::with_capacity(4);
            e.push(ua.map_or(1.0, |q| dir * (q - y[0])));
            e.push(wb.map_or(1.0, |q| dir * (q - y[1])));
            // the rails meeting: the ruling shrunk to a point
            let (pa, pb) = (a.jet(y[0], at_a), b.jet(y[1], at_b));
            e.push(match (pa, pb) {
                (Ok([pa, ..]), Ok([pb, ..])) => norm(sub(pb, pa)) - MEET * size,
                _ => f64::NAN,
            });
            if let Some(t) = leave {
                let v = match piece {
                    Piece::FanA => b.jet(y[1], at_b).ok().map(|[pb, b1, _]| {
                        det(t, b1, sub(pb, corner_a))
                    }),
                    _ => a.jet(y[0], at_a).ok().map(|[pa, a1, _]| det(a1, t, sub(corner_b, pa))),
                };
                e.push(v.unwrap_or(f64::NAN));
            }
            e
        };
        let mut flow = Flow { a, b, piece, at_a, at_b, carry: Carry::Nothing, d: 0.0,
                              rates: [0.0; 4] };
        let scale = [a.period().max(1e-9), b.period().max(1e-9)];
        let place = |u: f64| a.path.point(a.wrapped(u, u).0);
        let nodes = run(&mut flow, sigma, &[u, w], dir, size, &scale, event)
            .map_err(|(why, uu)| Refusal::at(why, place(uu)))?;
        let mut nodes: Vec<(f64, f64, f64)> = nodes.into_iter().map(|(s, y)| (s, y[0], y[1])).collect();
        // land a break exactly where the march met it
        if let Some(last) = nodes.last_mut() {
            if let Some(q) = ua.filter(|&q| (last.1 - q).abs() <= 1e-9 * (1.0 + q.abs())) {
                last.1 = q;
            }
            if let Some(q) = wb.filter(|&q| (last.2 - q).abs() <= 1e-9 * (1.0 + q.abs())) {
                last.2 = q;
            }
        }
        Ok(Marched { piece, at_a, at_b, nodes })
    }

    /// The running number's interval: the start's end to the finish's (a band's once round from
    /// its seam), the home at 0.
    pub fn span(&self) -> (f64, f64) {
        self.span
    }

    /// Whether both rails are closed, so the panel is a band.
    pub fn closed(&self) -> bool {
        self.a.closed
    }

    /// The pieces, in order of the running number, each its kind and its interval.
    pub fn pieces(&self) -> Vec<(Piece, f64, f64)> {
        self.pieces.iter().map(|p| {
            let (a, b) = p.span();
            (p.piece, a, b)
        }).collect()
    }

    /// The piece `sigma` is on: the first whose interval holds it.
    fn piece_at(&self, sigma: f64) -> &Marched {
        let s = sigma.clamp(self.span.0, self.span.1);
        self.pieces.iter().find(|p| {
            let (a, b) = p.span();
            s >= a && s <= b
        }).unwrap_or(&self.pieces[self.pieces.len() - 1])
    }

    /// The ruling at `sigma` on the running number.
    pub fn ruling(&self, sigma: f64) -> Result<Ruling, Refusal> {
        let p = self.piece_at(sigma);
        let mut flow = Flow::on(&self.a, &self.b, p);
        let y = dense(&mut flow, &p.nodes, sigma.clamp(self.span.0, self.span.1),
                      &[self.a.period(), self.b.period()])
            .ok_or_else(|| Refusal::whole("the panel cannot be read there"))?;
        let l = flow.local(y[0], y[1]).map_err(|_| {
            Refusal::at("the panel has no plane there", self.a.path.point(self.a.wrapped(y[0], y[0]).0))
        })?;
        Ok(Ruling { sigma, u: y[0], w: y[1], m: l.m, n: l.n, normal: l.normal, piece: p.piece })
    }

    /// Rulings enough to draw the panel to `tol`: each piece's ends, and between them halved
    /// where the middle ruling's feet, or its middle, stand off the chord between its neighbours'
    /// by more.
    pub fn stations(&self, tol: f64) -> Vec<Ruling> {
        let mut out: Vec<Ruling> = Vec::new();
        for p in &self.pieces {
            let (s0, s1) = p.span();
            if !(s1 > s0) {
                continue;
            }
            // each interval with its two end rulings, read once: a split reads its middle alone
            let n = 8;
            let marks: Vec<Ruling> = (0..=n)
                .filter_map(|k| self.ruling(s0 + (s1 - s0) * k as f64 / n as f64).ok())
                .collect();
            let mut seq: Vec<Ruling> = marks.first().copied().into_iter().collect();
            let mut stack: Vec<(Ruling, Ruling, usize)> =
                marks.windows(2).rev().map(|w| (w[0], w[1], 0)).collect();
            while let Some((rx, ry, depth)) = stack.pop() {
                let Ok(rm) = self.ruling(0.5 * (rx.sigma + ry.sigma)) else { continue };
                let off = |p: V, q: V, m: V| norm(sub(m, scale(add(p, q), 0.5)));
                let mid = |r: &Ruling| scale(add(r.m, r.n), 0.5);
                let bad = off(rx.m, ry.m, rm.m).max(off(rx.n, ry.n, rm.n))
                    .max(off(mid(&rx), mid(&ry), mid(&rm)));
                if bad > tol && depth < 16 {
                    stack.push((rm, ry, depth + 1));
                    stack.push((rx, rm, depth + 1));
                } else {
                    seq.push(ry);
                }
            }
            if let Some(last) = out.last() {
                if seq.first().is_some_and(|r| (r.sigma - last.sigma).abs() <= 1e-12) {
                    seq.remove(0);
                }
            }
            out.extend(seq);
        }
        out
    }

    /// The panel's area, by Gauss–Legendre over each piece in the running number of the
    /// rulings' strips: `(|m′ × r| + |n′ × r|) / 2` along it.
    pub fn area(&self) -> f64 {
        let mut total = 0.0;
        for p in &self.pieces {
            let (s0, s1) = p.span();
            if !(s1 > s0) {
                continue;
            }
            let flow = Flow::on(&self.a, &self.b, p);
            let strip = |s: f64| -> f64 {
                let Ok(r) = self.ruling(s) else { return 0.0 };
                let Ok(l) = flow.local(r.u, r.w) else { return 0.0 };
                let rr = sub(l.n, l.m);
                0.5 * (norm(cross(l.dm, rr)) + norm(cross(l.dn, rr)))
            };
            let k = 32;
            for j in 0..k {
                let (x, y) = (s0 + (s1 - s0) * j as f64 / k as f64,
                              s0 + (s1 - s0) * (j + 1) as f64 / k as f64);
                total += crate::path::gauss(&strip, x, y);
            }
        }
        total
    }

    /// **The development, exact**: the layer `d` along the normal laid flat, outside up, from its
    /// home ruling laid from `(0, 0)` along `+x`.
    pub fn develop(&self, d: f64) -> Result<Development, Refusal> {
        self.clone().laid(d, Carry::Exact)
    }

    /// **The check**: the same, marched by the body's own rows (Newton for the rates at every
    /// evaluation) rather than their closed form.
    pub fn develop_generic(&self, d: f64) -> Result<Development, Refusal> {
        self.clone().laid(d, Carry::Generic)
    }

    /// The panel laid, carrying `carry`: the development owns the panel it lays.
    fn laid(self, d: f64, carry: Carry) -> Result<Development, Refusal> {
        let home = self.ruling(0.0)?;
        let len = norm(sub(home.n, home.m));
        let scale = |w: usize| -> Vec<f64> {
            [self.a.period(), self.b.period(), 1.0, len.max(1.0), len.max(1.0), len.max(1.0)]
                [..w].to_vec()
        };
        let mut laid = Vec::new();
        // ahead from the home, then back from it, each piece from where the last left off: every
        // piece is on one side of the home, which is where the two marches met
        for dir in [1.0, -1.0] {
            let order: Vec<usize> = (0..self.pieces.len()).filter(|&k| {
                let (a, b) = self.pieces[k].span();
                b > a && if dir > 0.0 { a >= 0.0 } else { b <= 0.0 }
            }).collect();
            let order: Vec<usize> = if dir > 0.0 { order } else { order.into_iter().rev().collect() };
            let mut dev: Vec<f64> = match carry {
                Carry::Exact => vec![0.0, 0.0, 0.0],
                _ => vec![0.0, 0.0, len, 0.0],
            };
            let mut rates = [0.0; 4];
            for k in order {
                let p = &self.pieces[k];
                let (s0, s1) = p.span();
                let (from, to) = if dir > 0.0 { (s0, s1) } else { (s1, s0) };
                let &(_, u, w) = &p.nodes[0];
                let mut flow = Flow { carry, d, rates, ..Flow::on(&self.a, &self.b, p) };
                if carry == Carry::Generic && rates == [0.0; 4] {
                    // the check's first rates read off the rows from the outside-up guess
                    flow.rates = guess_rates(&flow, u, w, [dev[0], dev[1]], [dev[2], dev[3]])
                        .ok_or_else(|| Refusal::whole("the body's rows have no rates there"))?;
                }
                let mut y = vec![u, w];
                y.extend_from_slice(&dev);
                let path = ode::march(&mut flow, from, &y, 0, to - from, &scale(y.len()),
                                      &ode::Settings::default())
                    .map_err(|e| Refusal::whole(format!("the development cannot be followed \
                        past {}", crate::vocab::num(e.at()))))?;
                let last = path.nodes.last().expect("a march has its start");
                dev = last.y[2..].to_vec();
                rates = last.branch;
                laid.push(LaidPiece { lo: s0, hi: s1, piece: k, path });
            }
        }
        Ok(Development { panel: self, d, carry, laid, len })
    }

    /// The rail `which` (0 `a`, 1 `b`) read where the panel reaches it: its length over the panel.
    pub fn rail_length(&self, which: usize) -> f64 {
        let rail = if which == 0 { &self.a } else { &self.b };
        let place = |p: &Marched, n: usize| if which == 0 { p.nodes[n].1 } else { p.nodes[n].2 };
        self.pieces.iter().map(|p| {
            let (x0, x1) = (place(p, 0), place(p, p.nodes.len() - 1));
            if x0 == x1 { return 0.0 }
            unwrapped_length(rail, x0, x1, if which == 0 { p.at_a } else { p.at_b })
        }).sum()
    }
}

/// A rail's length between two unwrapped places on one smooth piece: an open rail's by the path's
/// own reading, cut at its spline's knots.
fn unwrapped_length(r: &Rail, x0: f64, x1: f64, at: f64) -> f64 {
    if !r.closed {
        return r.path.length(x0, x1);
    }
    let speed = |x: f64| r.jet(x, at).map_or(0.0, |j| norm(j[1]));
    let (lo, hi) = (x0.min(x1), x0.max(x1));
    let k = 16;
    (0..k).map(|j| crate::path::gauss(&speed, lo + (hi - lo) * j as f64 / k as f64,
                         lo + (hi - lo) * (j + 1) as f64 / k as f64)).sum()
}

/// The check's first rates: the body's rows solved from the guess that lays the layer's foot's
/// rate outside up — its part along the ruling along the image, its part across across it.
fn guess_rates(flow: &Flow, u: f64, w: f64, f: [f64; 2], g: [f64; 2]) -> Option<[f64; 4]> {
    let l = flow.local(u, w).ok()?;
    let Layer { dmo, dno, g: gh, .. } = layer(&l, flow.d);
    let beta = cross(l.normal, gh);
    let e = [g[0] - f[0], g[1] - f[1]];
    let el = (e[0] * e[0] + e[1] * e[1]).sqrt();
    let (c, s) = (e[0] / el, e[1] / el);
    let lay = |v: V| {
        let (al, ac) = (dot(v, gh), dot(v, beta));
        [al * c - ac * s, al * s + ac * c]
    };
    let (df, dg) = (lay(dmo), lay(dno));
    Some([df[0], df[1], dg[0], dg[1]])
}

/// One piece of a development: the panel's piece it lays, its interval and its march.
#[derive(Clone, Debug)]
struct LaidPiece {
    lo: f64,
    hi: f64,
    piece: usize,
    path: ode::Path<[f64; 4]>,
}

/// **A panel laid flat**: the layer `d` along its normal, outside up, its home ruling from
/// `(0, 0)` along `+x`.  `point(σ)` reads the images of the feet on `a` and `b`, and their rates.
#[derive(Clone, Debug)]
pub struct Development {
    panel: Developable,
    d: f64,
    /// What its marches carried: the exact development, or the check.
    carry: Carry,
    laid: Vec<LaidPiece>,
    len: f64,
}

/// The images of a ruling's ends at a place, and their rates in the running number.
#[derive(Clone, Copy, Debug)]
pub struct Laid {
    pub f: [f64; 2],
    pub g: [f64; 2],
    pub df: [f64; 2],
    pub dg: [f64; 2],
}

impl Development {
    /// The panel it lays.
    pub fn panel(&self) -> &Developable {
        &self.panel
    }

    /// The images at `sigma`: one step from the node before, on the piece it is on.
    pub fn at(&self, sigma: f64) -> Result<Laid, Refusal> {
        let (lo, hi) = self.panel.span;
        let s = sigma.clamp(lo, hi);
        let lp = self.laid.iter().find(|l| s >= l.lo && s <= l.hi)
            .or_else(|| self.laid.last()).ok_or_else(|| Refusal::whole("nothing is laid"))?;
        let p = &self.panel.pieces[lp.piece];
        let mut flow = Flow { carry: self.carry, d: self.d,
                              ..Flow::on(&self.panel.a, &self.panel.b, p) };
        let node = lp.path.at(&mut flow, s, false)
            .map_err(|_| Refusal::whole("the development cannot be read there"))?;
        let y = node.y;
        let mut f = vec![0.0; y.len()];
        flow.rates = node.branch;
        if !ode::Rate::rate(&mut flow, s, &y, &mut f, None) {
            return Err(Refusal::whole("the development has no rate there"));
        }
        Ok(match self.carry {
            Carry::Exact => {
                let l = flow.local(y[0], y[1]).map_err(|_| Refusal::whole("no plane there"))?;
                let (sn, cs) = y[2].sin_cos();
                let Layer { g, len, .. } = layer(&l, self.d);
                let dlen = dot(g, sub(l.dn, l.dm));
                let ff = [y[3], y[4]];
                let gg = [ff[0] + len * cs, ff[1] + len * sn];
                let dgg = [f[3] + dlen * cs - len * sn * f[2], f[4] + dlen * sn + len * cs * f[2]];
                Laid { f: ff, g: gg, df: [f[3], f[4]], dg: dgg }
            }
            _ => Laid { f: [y[2], y[3]], g: [y[4], y[5]], df: [f[2], f[3]], dg: [f[4], f[5]] },
        })
    }

    /// The ruling's length at the home.
    pub fn home_length(&self) -> f64 {
        self.len
    }

    /// `∫` of each of `g`'s components over each laid piece — no piece crosses a rail's knot, so
    /// its rates are smooth over it — by Gauss–Legendre on `k` cuts of it, each node read once.
    fn integral<const N: usize>(&self, g: &dyn Fn(&Laid) -> [f64; N]) -> [f64; N] {
        let read = |s: f64| self.at(s).map_or([0.0; N], |l| g(&l));
        let k = 32;
        let mut total = [0.0; N];
        for l in self.laid.iter().filter(|l| l.hi > l.lo) {
            for j in 0..k {
                let part = crate::path::gauss_n(&read, l.lo + (l.hi - l.lo) * j as f64 / k as f64,
                                                l.lo + (l.hi - l.lo) * (j + 1) as f64 / k as f64);
                for q in 0..N {
                    total[q] += part[q];
                }
            }
        }
        total
    }

    /// The laid rails' lengths, `∫ |f′|` and `∫ |g′|`, and the pattern's area — what its boundary
    /// (`a` from start to finish, the end ruling, `b` back, the start ruling) encloses, `½ ∮ (x dy
    /// − y dx)` — in one pass.
    pub fn measures(&self) -> [f64; 3] {
        let [la, lb, rails] = self.integral(&|l| [
            l.df[0].hypot(l.df[1]),
            l.dg[0].hypot(l.dg[1]),
            0.5 * ((l.f[0] * l.df[1] - l.f[1] * l.df[0]) - (l.g[0] * l.dg[1] - l.g[1] * l.dg[0])),
        ]);
        let (lo, hi) = self.panel.span;
        let chord = |p: [f64; 2], q: [f64; 2]| 0.5 * (p[0] * q[1] - q[0] * p[1]);
        let ends = match (self.at(lo), self.at(hi)) {
            (Ok(s), Ok(e)) => chord(e.f, e.g) + chord(s.g, s.f),
            _ => 0.0,
        };
        [la, lb, (rails + ends).abs()]
    }

    /// The length of the laid rail `which` (0 `a`, 1 `b`).
    pub fn length(&self, which: usize) -> f64 {
        self.measures()[which.min(1)]
    }

    /// The pattern's area.
    pub fn area(&self) -> f64 {
        self.measures()[2]
    }

    /// The pattern's points, in the canonical laying, as finely as `per` points a laid piece: its
    /// rails' images and the end rulings' ends.  What a box around it is read off.
    pub fn outline(&self, per: usize) -> Vec<[f64; 2]> {
        let mut out = Vec::new();
        for l in self.laid.iter().filter(|l| l.hi > l.lo) {
            for j in 0..=per {
                if let Ok(x) = self.at(l.lo + (l.hi - l.lo) * j as f64 / per as f64) {
                    out.push(x.f);
                    out.push(x.g);
                }
            }
        }
        out
    }
}

/// A march over one piece from `(s0, y0)` the way `dir` runs, stopping where an event function
/// of the state first crosses nothing (each read with the sign it starts with): the nodes
/// `(σ, y)`, the last on the event.  `Err` names why the flow stopped and the `u` it stopped at.
fn run(flow: &mut Flow, s0: f64, y0: &[f64], dir: f64, size: f64, scale: &[f64],
       raw: impl Fn(&[f64]) -> Vec<f64>) -> Result<Vec<(f64, Vec<f64>)>, (String, f64)> {
    let n = flow.width();
    let signs: Vec<f64> = raw(y0).iter().map(|e| e.signum()).collect();
    let event = |y: &[f64]| -> Vec<f64> { raw(y).iter().zip(&signs).map(|(e, s)| e * s).collect() };
    let st = ode::Settings::default();
    // the state's scales, each past the rails' a unit
    let scale: Vec<f64> = scale.iter().copied().chain(std::iter::repeat(1.0)).take(n).collect();
    let mut w = ode::Work::new(n, 0, &st);
    let mut nodes = vec![(s0, y0.to_vec())];
    let mut y = y0.to_vec();
    let mut out = vec![0.0; n];
    let mut s = s0;
    let mut h = 0.05 * size;
    let floor = 1e-12 * size;
    let e0 = event(&y);
    if e0.iter().any(|&e| !(e > 0.0)) {
        // already on an event: a piece of nothing
        return Ok(nodes);
    }
    let mut steps = 0usize;
    let why = |flow: &mut Flow, y: &[f64]| -> String {
        match flow.local(y[0], y[1]) {
            Err(e) if e == "tangent" => "a ruling is tangent to a rail".into(),
            _ => "the rulings fold".into(),
        }
    };
    loop {
        steps += 1;
        if steps > ode::STEPS_MAX {
            return Err(("the rulings cannot be followed in so many steps".into(), y[0]));
        }
        // no step longer than a quarter of the ruling, which shrinks no faster than the running
        // number runs: rails meeting are met, never stepped past
        let reach = match (flow.a.jet(y[0], flow.at_a), flow.b.jet(y[1], flow.at_b)) {
            (Ok([pa, ..]), Ok([pb, ..])) => 0.25 * norm(sub(pb, pa)),
            _ => h,
        };
        h = h.min(reach.max(floor));
        let err = ode::step(flow, &mut w, s, &y, dir * h, &scale, &mut out);
        let Some(err) = err else {
            if h <= floor {
                // where the flow stops: a fold, or a ruling along a rail
                let mut probe = y.clone();
                let mut f = vec![0.0; n];
                let tiny = dir * floor * 10.0;
                if ode::Rate::rate(flow, s + tiny, &y, &mut f, None) {
                    for k in 0..n {
                        probe[k] = y[k] + tiny * f[k];
                    }
                }
                return Err((why(flow, &probe), y[0]));
            }
            h *= 0.25;
            continue;
        };
        if err > st.tol && h > floor {
            h = st.next_h(h, err).min(h * 0.5);
            continue;
        }
        flow.settle(&mut out);
        let ev = event(&out);
        if ev.iter().any(|e| e.is_nan()) {
            return Err(("the panel cannot be read past here".into(), y[0]));
        }
        if ev.iter().any(|&e| e <= 0.0) {
            // an event lies inside this step: find each on the step by the secant of its value,
            // within what is left before the ones found, and stop at the first
            let e_here = event(&y);
            let (mut hi, mut best) = (h, out.clone());
            let mut tmp = vec![0.0; n];
            for k in 0..ev.len() {
                let mut fhi = event(&best)[k];
                if !(fhi <= 0.0) {
                    continue;
                }
                let (mut lo, mut flo) = (0.0, e_here[k]);
                for _ in 0..80 {
                    let t = if (fhi - flo).abs() > 0.0 { lo + (hi - lo) * flo / (flo - fhi) }
                            else { 0.5 * (lo + hi) };
                    let t = t.clamp(lo + 0.05 * (hi - lo), hi - 0.05 * (hi - lo));
                    if ode::step(flow, &mut w, s, &y, dir * t, &scale, &mut tmp).is_none() {
                        hi = t;
                        continue;
                    }
                    let ft = event(&tmp)[k];
                    if ft <= 0.0 {
                        hi = t;
                        fhi = ft;
                        best.copy_from_slice(&tmp);
                    } else {
                        lo = t;
                        flo = ft;
                    }
                    if (hi - lo) <= 1e-15 * size.max(h) || fhi.abs() <= 1e-15 {
                        break;
                    }
                }
            }
            flow.settle(&mut best);
            nodes.push((s + dir * hi, best));
            return Ok(nodes);
        }
        s += dir * h;
        std::mem::swap(&mut y, &mut out);
        nodes.push((s, y.clone()));
        h = st.next_h(h, err);
    }
}

/// The state at `sigma` on a piece's nodes: one step from the node before, the way they run.
fn dense(flow: &mut Flow, nodes: &[(f64, f64, f64)], sigma: f64, scale: &[f64]) -> Option<Vec<f64>> {
    let dir = if nodes.len() > 1 && nodes[nodes.len() - 1].0 < nodes[0].0 { -1.0 } else { 1.0 };
    let i = nodes.partition_point(|n| dir * (n.0 - sigma) <= 0.0).saturating_sub(1);
    let (s0, u0, w0) = nodes[i];
    let h = sigma - s0;
    if h == 0.0 || dir * h < 0.0 {
        return Some(vec![u0, w0]);
    }
    let st = ode::Settings::default();
    let mut w = ode::Work::new(2, 0, &st);
    let mut out = vec![0.0; 2];
    ode::step(flow, &mut w, s0, &[u0, w0], h, scale, &mut out)?;
    Some(out)
}

/// **The home's ruling**: the roots of `F(u0, ·)` along `b` — sign changes of it on samples,
/// refined by bisection — each corner of `b` where the plane through the ruling and `a′` supports
/// `b`, and `b`'s ends where it is open and nothing else is found; of those whose `b` runs through
/// the plane the way `a` does, the shortest — with `a`'s point and tangent there.
fn home_root(a: &Rail, b: &Rail, u0: f64) -> Result<(f64, V, V), Refusal> {
    // at a corner of `a`, the tangent the march arrives by: the home is where its fan starts
    let back = if !a.closed && u0 <= a.lo { 1.0 } else { -1.0 };
    let at_a = a.anchor(u0, back);
    let [pa, a1, _] = a.jet(u0, at_a).map_err(Refusal::whole)?;
    let f = |w: f64, at: f64| -> Option<(f64, V, V)> {
        let [pb, b1, _] = b.jet(w, at).ok()?;
        let r = sub(pb, pa);
        Some((det(a1, b1, r), b1, r))
    };
    let mut roots: Vec<(f64, V, V)> = Vec::new();
    let kinks = b.breaks(b.lo, b.hi);
    let mut cuts = vec![b.lo];
    cuts.extend(kinks.iter().copied().filter(|&q| q > b.lo && q < b.hi));
    cuts.push(b.hi);
    cuts.dedup();
    let per = (ROOT_SAMPLES / (cuts.len() - 1).max(1)).max(24);
    for win in cuts.windows(2) {
        let (lo, hi) = (win[0], win[1]);
        let at = 0.5 * (lo + hi);
        let xs: Vec<f64> = (0..=per).map(|k| lo + (hi - lo) * k as f64 / per as f64).collect();
        let vals: Vec<Option<f64>> = xs.iter().map(|&x| f(x, at).map(|v| v.0)).collect();
        for k in 0..per {
            let (Some(v0), Some(v1)) = (vals[k], vals[k + 1]) else { continue };
            if v0 == 0.0 {
                if let Some(r) = f(xs[k], at) {
                    roots.push((xs[k], r.1, r.2));
                }
                continue;
            }
            if v0.signum() == v1.signum() {
                continue;
            }
            let (mut x0, mut x1, mut f0) = (xs[k], xs[k + 1], v0);
            for _ in 0..200 {
                let xm = 0.5 * (x0 + x1);
                let Some((fm, ..)) = f(xm, at) else { break };
                if fm.signum() == f0.signum() {
                    x0 = xm;
                    f0 = fm;
                } else {
                    x1 = xm;
                }
                if (x1 - x0).abs() <= 1e-15 * (1.0 + xm.abs()) {
                    break;
                }
            }
            let x = 0.5 * (x0 + x1);
            if let Some(r) = f(x, at) {
                roots.push((x, r.1, r.2));
            }
        }
    }
    // a corner of `b` where the plane supports it
    for &q in kinks.iter().filter(|&&q| !(!b.closed && (q == b.lo || q == b.hi))) {
        let before = b.jet(q, q - 1e-9 * (1.0 + q.abs()) * b.period()).ok();
        let after = b.jet(q, q + 1e-9 * (1.0 + q.abs()) * b.period()).ok();
        if let (Some([pb, t0, _]), Some([_, t1, _])) = (before, after) {
            let r = sub(pb, pa);
            if det(a1, t0, r).signum() != det(a1, t1, r).signum() {
                let t = add(normalised(t0).unwrap_or(t0), normalised(t1).unwrap_or(t1));
                roots.push((q, t, r));
            }
        }
    }
    if roots.is_empty() && !b.closed {
        for q in [b.lo, b.hi] {
            if let Some(r) = f(q, q) {
                roots.push((q, r.1, r.2));
            }
        }
    }
    let same = |(_, b1, r): &(f64, V, V)| dot(cross(a1, *r), cross(*b1, *r)) > 0.0;
    let best = roots.iter().filter(|x| same(x)).min_by(|x, y| norm(x.2).total_cmp(&norm(y.2)));
    match best {
        Some(&(w, ..)) => Ok((w, pa, a1)),
        None => Err(Refusal::at("no ruling from the home has `b` running through its plane the \
            way `a` does (`sense: same`)", pa)),
    }
}


/* -- a laid rail as a curve of the drawing ------------------------------------------------- */
//
// `flat.a` and `flat.b` are curves drawn in the plane the development is laid in, over the
// curve parameter `t` from 0 to 1 — the running number from the panel's start to its finish,
// evenly — and placed by its home ruling's ends `f` and `g`, the curve's columns: the canonical
// development (home from `(0, 0)` along `+x`) turned to the bearing from `f` to `g` and moved to
// `f`.  The rails ride in the constants, so a contact's kernel is still a plain `fn`; the panel
// and its development are built from them once and remembered by their content, as a free
// curve's shape is.

/// The constants before the rails': which rail's image (0 `a`, 1 `b`), the layer, and the home
/// point on `a`.
pub const HEAD: usize = 5;

/// A laid rail's constants: which image, the layer `d`, the home point `s`, then the rails.
pub fn consts(a: &Path, b: &Path, s: V, d: f64, which: u8) -> Vec<f64> {
    let mut k = vec![which as f64, d, s[0], s[1], s[2]];
    a.encode(&mut k);
    b.encode(&mut k);
    k
}

thread_local! {
    /// The developments last built, by their constants past `which`: a solve reads the same one
    /// at every contact and every iteration.
    static BUILT: std::cell::RefCell<Vec<(Vec<f64>, std::rc::Rc<Result<Development, Refusal>>)>> =
        const { std::cell::RefCell::new(Vec::new()) };
}

/// How many developments are remembered: a drawing's every laid rail, past a few dozen panels
/// rebuilt as it is read.
const REMEMBERED: usize = 32;

/// The development a laid rail's constants say, built once and remembered by their content —
/// `None` where they are not a laid rail's.
pub fn developed(k: &[f64]) -> Option<std::rc::Rc<Result<Development, Refusal>>> {
    let key = k.get(1..)?;
    if let Some(hit) = BUILT.with(|b| b.borrow().iter().find(|(kk, _)| kk[..] == *key)
        .map(|(_, d)| d.clone())) {
        return Some(hit);
    }
    let d = *k.get(1)?;
    let s = [*k.get(2)?, *k.get(3)?, *k.get(4)?];
    let (a, na) = Path::decode(k.get(HEAD..)?)?;
    let (b, _) = Path::decode(k.get(HEAD + na..)?)?;
    let built = std::rc::Rc::new(Developable::build(a, b, s).and_then(|p| p.laid(d, Carry::Exact)));
    BUILT.with(|c| {
        let mut c = c.borrow_mut();
        if c.len() >= REMEMBERED {
            c.remove(0);
        }
        c.push((key.to_vec(), built.clone()));
    });
    Some(built)
}

/// The image at the curve parameter `t` in the canonical laying, and its derivatives in `t` to
/// `orders` (1 to 3, the rest zero): `F`, `F′` exactly, `F″` by a central difference of the exact
/// rate, `F‴` by a second difference of it — each order past the first two more readings.
fn canonical(dev: &Development, which: u8, t: f64, orders: u8) -> Option<[[f64; 2]; 4]> {
    let (lo, hi) = dev.panel.span;
    let span = hi - lo;
    let sigma = lo + span * t.clamp(0.0, 1.0);
    let rate = |s: f64| -> Option<([f64; 2], [f64; 2])> {
        let l = dev.at(s.clamp(lo, hi)).ok()?;
        Some(if which == 0 { (l.f, l.df) } else { (l.g, l.dg) })
    };
    let (x, dx) = rate(sigma)?;
    let mut out = [x, [dx[0] * span, dx[1] * span], [0.0; 2], [0.0; 2]];
    if orders >= 2 {
        let h = 1e-6 * span;
        let (a, b) = ((sigma - h).max(lo), (sigma + h).min(hi));
        let ((_, da), (_, db)) = (rate(a)?, rate(b)?);
        out[2] = [0, 1].map(|q| (db[q] - da[q]) / (b - a) * span * span);
    }
    if orders >= 3 {
        let h = 1e-4 * span;
        let ((_, da), (_, db)) = (rate(sigma - h)?, rate(sigma + h)?);
        out[3] = [0, 1].map(|q| (db[q] - 2.0 * dx[q] + da[q]) / (h * h) * span * span * span);
    }
    Some(out)
}

/// `(−y, x)`.
fn quarter(p: [f64; 2]) -> [f64; 2] {
    [-p[1], p[0]]
}

/// The laid rail at `t` placed by `θ = [f, g]`, its derivatives in `t` to `orders`, and the
/// gradient of each in `θ` (the bearing from `f` to `g` turns the canonical laying; `f` moves it).
type Placed = ([[f64; 2]; 4], [[[f64; 2]; 4]; 4]);

fn placed(k: &[f64], t: f64, theta: &[f64], orders: u8) -> Option<Placed> {
    let which = *k.first()? as u8;
    if theta.len() != 4 || which > 1 {
        return None;
    }
    let built = developed(k)?;
    // **until the panel can be built** (its rails at their seeds, a round of no radius): the
    // laid rail stands at its own end of the home ruling, a point, so a contact on it holds there
    // and the solve goes on to place the rails; the constants are read again where it stops
    // (`Sketch::development_of`, a refusal the report says)
    let Ok(dev) = built.as_ref() else {
        let at = [theta[2 * which as usize], theta[2 * which as usize + 1]];
        let mut grad = [[[0.0; 2]; 4]; 4];
        grad[0][2 * which as usize] = [1.0, 0.0];
        grad[0][2 * which as usize + 1] = [0.0, 1.0];
        return Some(([at, [0.0; 2], [0.0; 2], [0.0; 2]], grad));
    };
    let laid = canonical(dev, which, t, orders)?;
    let (dxv, dyv) = (theta[2] - theta[0], theta[3] - theta[1]);
    let l2 = dxv * dxv + dyv * dyv;
    if !(l2 > 0.0) {
        return None;
    }
    let l = l2.sqrt();
    let (c, s) = (dxv / l, dyv / l);
    let mut out = laid.map(|p| [c * p[0] - s * p[1], s * p[0] + c * p[1]]);
    // ∂φ/∂θ: the bearing from f to g
    let dphi = [dyv / l2, -dxv / l2, -dyv / l2, dxv / l2];
    let mut grad = [[[0.0; 2]; 4]; 4];
    for (order, p) in out.iter().enumerate() {
        let q = quarter(*p);
        for j in 0..4 {
            grad[order][j] = [q[0] * dphi[j], q[1] * dphi[j]];
        }
    }
    out[0] = [out[0][0] + theta[0], out[0][1] + theta[1]];
    grad[0][0][0] += 1.0;
    grad[0][1][1] += 1.0;
    Some((out, grad))
}

/// The evaluation a placing gives: `C` and its derivatives in `t` to `orders`, and the gradient of
/// `C` over `[t, f, g]`.
fn val_of((c, g): &Placed, orders: u8) -> crate::locus::Val {
    let mut v = crate::locus::Val::default();
    v.x = c[0][0];
    v.y = c[0][1];
    v.dx[0] = c[1][0];
    v.dy[0] = c[1][1];
    for j in 0..4 {
        v.dx[1 + j] = g[0][j][0];
        v.dy[1 + j] = g[0][j][1];
    }
    if orders >= 2 {
        v.d2 = c[2];
    }
    v.orders = orders.min(2);
    v.ok = true;
    v
}

/// A contact's evaluation (`kernels::body_val`'s for this body): `C` and its derivatives in `t`
/// to `need` (at most the second), and the gradient of `C` over `[t, f, g]`.  NaN where the panel
/// cannot be had.
pub fn kernel_eval(k: &[f64], t: f64, theta: &[f64], need: u8, _gradient: bool) -> crate::locus::Val {
    let orders = need.clamp(1, 2);
    placed(k, t, theta, orders).map_or_else(crate::locus::Val::default, |p| val_of(&p, orders))
}

/// A contact's frame (`kernels::curve_frame`'s): the evaluation, and the gradients of `C′` and
/// `C″` over `[t, f, g]` — along `t` the next order — from one placing.
pub fn kernel_frame(k: &[f64], t: f64, theta: &[f64], _need: u8) -> crate::locus::Frame {
    let mut d1 = [[0.0f64; crate::tape::MAX_VARS]; 2];
    let mut d2 = [[f64::NAN; crate::tape::MAX_VARS]; 2];
    let Some(p) = placed(k, t, theta, 3) else {
        return crate::locus::Frame { val: crate::locus::Val::default(), d1, d2 };
    };
    let (c, g) = &p;
    for q in 0..2 {
        d1[q][0] = c[2][q];
        d2[q][0] = c[3][q];
        for j in 0..4 {
            d1[q][1 + j] = g[1][j][q];
            d2[q][1 + j] = g[2][j][q];
        }
    }
    let mut val = val_of(&p, 2);
    (val.d3, val.orders) = (c[3], 3);
    crate::locus::Frame { val, d1, d2 }
}

/// The laid rail at `t` placed by `θ`, for drawing: `None` where the panel cannot be had.
pub fn point(k: &[f64], t: f64, theta: &[f64]) -> Option<[f64; 2]> {
    placed(k, t, theta, 1).map(|(c, _)| c[0])
}
