//! **X1 of #183: rounds and chamfers of a coaxial region, exactly in the meridian.**  A prototype,
//! kept in this file on purpose (the language step that would call it, L5, has not been decided):
//! a region of lines and arcs — a meridian, as `solid::region::meridian` folds one — rounded or
//! chamfered where a step of one group of faces meets a step of another, with nothing but the
//! planar kernel's own steps.  `A round(r) B` is the ball of radius `r` rolled into that corner:
//! its centre where the two faces' carriers, each offset `r` toward the ball, cross (the opening's
//! `−r` then `+r`, read at one corner), the corner replaced by the arc between the two feet.  The
//! ball lies inside the material at a convex corner and outside it at a concave one, read off the
//! turn of the boundary, never said.  `A chamfer(d) B` is the segment between a point on each face,
//! each leg measured one of two ways (`Leg`; #183 X0's finding 7).
//!
//! Turned about the axis the arcs are tori and spheres and the segments cones and planes: the
//! kernel already revolves such a profile exactly.  So the gates are volumes by Green's theorem
//! (`turned`), held to 1e-9 against the closed forms the Rust fillet stack (#79, #83) checks its
//! fixtures by, and the hypoid's drawn rounding and end chamfer against the prototype's.
use gcs_core::brep::planar::{boolean, Op, Region, Seg, Step};
use gcs_core::model::{RegionShape, SolidDef};
use gcs_core::program::Elaborated;
use gcs_core::space::{cross as cross3, dot as dot3, norm as norm3, sub as sub3};
use gcs_core::syntax::Cmp;
use std::f64::consts::{PI, TAU};

type P = [f64; 2];

fn sub(a: P, b: P) -> P { [a[0] - b[0], a[1] - b[1]] }
fn add(a: P, b: P) -> P { [a[0] + b[0], a[1] + b[1]] }
fn scale(a: P, k: f64) -> P { [a[0] * k, a[1] * k] }
fn dot(a: P, b: P) -> f64 { a[0] * b[0] + a[1] * b[1] }
fn cross(a: P, b: P) -> f64 { a[0] * b[1] - a[1] * b[0] }
fn norm(a: P) -> f64 { a[0].hypot(a[1]) }
fn dist(a: P, b: P) -> f64 { norm(sub(a, b)) }
fn unit(a: P) -> P { scale(a, 1.0 / norm(a)) }
fn left(a: P) -> P { [-a[1], a[0]] }

// -- the prototype -----------------------------------------------------------------------------

/// The line or circle a step lies on: what is offset and crossed.
#[derive(Clone, Copy, Debug)]
enum Carrier {
    Line { p: P, d: P },
    Circle { c: P, r: f64 },
}

impl Carrier {
    fn of(s: &Seg) -> Carrier {
        match *s {
            Seg::Line { a, b } => Carrier::Line { p: a, d: unit(sub(b, a)) },
            Seg::Arc { c, r, .. } => Carrier::Circle { c, r },
        }
    }
    /// The carrier moved `by` to the left of travel `t` at its point `x` (to the right where `by`
    /// is negative): a line shifted, a circle grown or shrunk; none where a circle shrinks away.
    fn offset(self, t: P, x: P, by: f64) -> Option<Carrier> {
        let n = unit(left(t));
        match self {
            Carrier::Line { p, d } => Some(Carrier::Line { p: add(p, scale(n, by)), d }),
            Carrier::Circle { c, r } => {
                let r = if dot(n, sub(c, x)) > 0.0 { r - by } else { r + by };
                (r > 0.0).then_some(Carrier::Circle { c, r })
            }
        }
    }
    /// The point of the carrier nearest `x`.
    fn foot(self, x: P) -> P {
        match self {
            Carrier::Line { p, d } => add(p, scale(d, dot(sub(x, p), d))),
            Carrier::Circle { c, r } => add(c, scale(unit(sub(x, c)), r)),
        }
    }
    /// Where two carriers cross, in closed form.
    fn meet(self, other: Carrier) -> Vec<P> {
        match (self, other) {
            (Carrier::Line { p, d }, Carrier::Line { p: q, d: e }) => {
                let den = cross(d, e);
                if den.abs() < 1e-15 {
                    return Vec::new();
                }
                vec![add(p, scale(d, cross(sub(q, p), e) / den))]
            }
            (Carrier::Line { p, d }, Carrier::Circle { c, r })
            | (Carrier::Circle { c, r }, Carrier::Line { p, d }) => {
                let f = sub(p, c);
                let (b, k) = (dot(f, d), dot(f, f) - r * r);
                let disc = b * b - k;
                if disc < 0.0 {
                    return Vec::new();
                }
                let w = disc.sqrt();
                vec![add(p, scale(d, -b - w)), add(p, scale(d, -b + w))]
            }
            (Carrier::Circle { c, r }, Carrier::Circle { c: c2, r: r2 }) => {
                let between = dist(c, c2);
                if between == 0.0 {
                    return Vec::new();
                }
                let along = (between * between + r * r - r2 * r2) / (2.0 * between);
                let h2 = r * r - along * along;
                if h2 < 0.0 {
                    return Vec::new();
                }
                let u = unit(sub(c2, c));
                let m = add(c, scale(u, along));
                let h = h2.sqrt();
                vec![add(m, scale(left(u), h)), add(m, scale(left(u), -h))]
            }
        }
    }
}

/// Where along a step a point of its carrier is (its fraction; an arc's measured the way it runs,
/// a point just before its start read as before it, as `planar` reads one).
fn param(s: &Seg, x: P) -> f64 {
    match *s {
        Seg::Line { a, b } => {
            let d = sub(b, a);
            dot(sub(x, a), d) / dot(d, d)
        }
        Seg::Arc { c, a0, sweep, .. } => {
            let phi = (x[1] - c[1]).atan2(x[0] - c[0]);
            let mut delta = ((phi - a0) * sweep.signum()).rem_euclid(TAU);
            if delta > PI + sweep.abs() / 2.0 {
                delta -= TAU;
            }
            delta / sweep.abs()
        }
    }
}

/// The stretch of a step from `t0` to `t1`.
fn part(s: &Seg, t0: f64, t1: f64) -> Seg {
    match *s {
        Seg::Line { .. } => Seg::Line { a: s.point(t0), b: s.point(t1) },
        Seg::Arc { c, r, a0, sweep } => {
            Seg::Arc { c, r, a0: a0 + sweep * t0, sweep: sweep * (t1 - t0) }
        }
    }
}

/// How a chamfer's leg is measured (#183 X0, finding 7).
#[derive(Clone, Copy, Debug)]
enum Leg {
    /// The point of the face `d` from the edge: a straight distance, along a line its length.
    FromEdge(f64),
    /// The point of the face `d` from the other face's surface, square to it.
    FromOther(f64),
}

/// What is put in a corner: where the step arriving at it ends and the step leaving it starts
/// (their fractions), and the step between.
type Cut = (f64, f64, Seg);

/// The ball of radius `r` rolled into the corner where `s` arrives and `n` leaves, on side `side`
/// (+1 the left: inside the material at a convex corner, outside it at a concave one).
fn ball(s: &Seg, n: &Seg, side: f64, r: f64) -> Result<Cut, String> {
    let x = s.end();
    let (ts, tn) = (unit(s.tangent(1.0)), unit(n.tangent(0.0)));
    let off = |c: Carrier, t: P| c.offset(t, x, side * r).ok_or("a face too curved for the ball");
    let (cs, cn) = (off(Carrier::of(s), ts)?, off(Carrier::of(n), tn)?);
    let centre = cs.meet(cn).into_iter().min_by(|a, b| dist(*a, x).total_cmp(&dist(*b, x)))
        .ok_or("the faces' offsets do not meet")?;
    let (fs, fn_) = (Carrier::of(s).foot(centre), Carrier::of(n).foot(centre));
    let a0 = (fs[1] - centre[1]).atan2(fs[0] - centre[0]);
    let a1 = (fn_[1] - centre[1]).atan2(fn_[0] - centre[0]);
    let mut sweep = (a1 - a0).rem_euclid(TAU);
    if side < 0.0 {
        sweep -= TAU;
    }
    Ok((param(s, fs), param(n, fn_), Seg::Arc { c: centre, r, a0, sweep }))
}

/// The flat across the corner where `s` arrives and `n` leaves, its leg on `s` measured by
/// `legs[0]` and on `n` by `legs[1]`.
fn flat(s: &Seg, n: &Seg, side: f64, legs: [Leg; 2]) -> Result<Cut, String> {
    let x = s.end();
    let (ts, tn) = (unit(s.tangent(1.0)), unit(n.tangent(0.0)));
    // the leg's end on `on`, by `leg`: of the crossings of `on`'s carrier, the one nearest the
    // corner along `on` (`before`: on the step arriving at the corner)
    let end = |on: &Seg, other: &Seg, t_other: P, leg: Leg, before: bool| -> Result<f64, String> {
        let crossing = match leg {
            Leg::FromEdge(d) => Carrier::Circle { c: x, r: d },
            Leg::FromOther(d) => Carrier::of(other).offset(t_other, x, side * d)
                .ok_or("a face too curved for the leg")?,
        };
        Carrier::of(on).meet(crossing).into_iter().map(|p| param(on, p))
            .filter(|&t| if before { t < 1.0 } else { t > 0.0 })
            .min_by(|a, b| {
                let off = |t: f64| if before { 1.0 - t } else { t };
                off(*a).total_cmp(&off(*b))
            })
            .ok_or_else(|| "the leg does not reach the face".to_string())
    };
    let t_s = end(s, n, tn, legs[0], true)?;
    let t_n = end(n, s, ts, legs[1], false)?;
    Ok((t_s, t_n, Seg::Line { a: s.point(t_s), b: n.point(t_n) }))
}

/// `region` with every corner where a step of `a` meets a step of `b` (either way round) cut by
/// `cut`, the new step tagged `tag`.  A seam (the steps meeting tangent) is no corner.  Refused
/// where the cut does not fit on a face: its foot past the far end of a step, or two cuts meeting
/// within one (a face the rounds would consume, #183 X0's finding 9).
fn corners(region: &Region, a: &[u32], b: &[u32], tag: u32,
           cut: &dyn Fn(&Step, &Step, f64) -> Result<Cut, String>) -> Result<Region, String> {
    let mut loops = Vec::new();
    for l in &region.loops {
        let k = l.len();
        // each step's stretch kept, and what follows it
        let mut keep: Vec<(f64, f64)> = vec![(0.0, 1.0); k];
        let mut after: Vec<Option<Seg>> = vec![None; k];
        for i in 0..k {
            let (s, n) = (&l[i], &l[(i + 1) % k]);
            let named = (a.contains(&s.tag) && b.contains(&n.tag))
                || (b.contains(&s.tag) && a.contains(&n.tag));
            if !named || s.chart.is_some() || n.chart.is_some() {
                continue;
            }
            let (ts, tn) = (unit(s.seg.tangent(1.0)), unit(n.seg.tangent(0.0)));
            let turn = cross(ts, tn);
            if turn.abs() <= 1e-12 && dot(ts, tn) > 0.0 {
                continue;
            }
            let (t_s, t_n, seg) = cut(s, n, turn.signum())?;
            if !(0.0..=1.0).contains(&t_s) || !(0.0..=1.0).contains(&t_n) {
                return Err(format!("the cut between faces {} and {} does not fit on them",
                    s.tag, n.tag));
            }
            keep[i].1 = t_s;
            keep[(i + 1) % k].0 = t_n;
            after[i] = Some(seg);
        }
        let mut walk = Vec::new();
        for i in 0..k {
            let (t0, t1) = keep[i];
            if t0 >= t1 {
                return Err(format!("the cuts at both ends of face {} meet within it", l[i].tag));
            }
            walk.push(Step { seg: part(&l[i].seg, t0, t1), ..l[i] });
            if let Some(seg) = after[i] {
                walk.push(Step { seg, chart: None, tag });
            }
        }
        loops.push(walk);
    }
    Ok(Region { loops })
}

/// `A round(r) B`: every corner between faces `a` and `b` rounded by the ball of radius `r`, filled
/// where concave and taken off where convex; a radius of zero the region as it was.
fn round(region: &Region, a: &[u32], b: &[u32], r: f64, tag: u32) -> Result<Region, String> {
    if r == 0.0 {
        return Ok(region.clone());
    }
    corners(region, a, b, tag, &|s, n, side| ball(&s.seg, &n.seg, side, r))
}

/// `A chamfer(…) B`: every corner between faces `a` and `b` cut flat, the leg on the face of `a`
/// by `legs[0]` and on the face of `b` by `legs[1]`.
fn chamfer(region: &Region, a: &[u32], b: &[u32], legs: [Leg; 2], tag: u32)
    -> Result<Region, String> {
    corners(region, a, b, tag, &|s, n, side| {
        let legs = if a.contains(&s.tag) { legs } else { [legs[1], legs[0]] };
        flat(&s.seg, &n.seg, side, legs)
    })
}

// -- exact measures ----------------------------------------------------------------------------

/// `∮ ρ²/2 dz` along a step, `(ρ, z)` its coordinates, in closed form.
fn flux(s: &Seg) -> f64 {
    match *s {
        Seg::Line { a: p, b: q } => {
            (q[1] - p[1]) / 2.0 * (p[0] * p[0] + p[0] * q[0] + q[0] * q[0]) / 3.0
        }
        Seg::Arc { c, r, a0, sweep } => {
            let prim = |phi: f64| {
                let (sn, cs) = phi.sin_cos();
                (c[0] * c[0] * r * sn + c[0] * r * r * (phi + sn * cs)
                    + r * r * r * (sn - sn.powi(3) / 3.0)) / 2.0
            };
            prim(a0 + sweep) - prim(a0)
        }
    }
}

/// The volume a meridian sweeps turned once about `ρ = 0` (Green's theorem, every step exact).
fn turned(region: &Region) -> f64 {
    2.0 * PI * region.loops.iter().flatten().map(|s| flux(&s.seg)).sum::<f64>()
}

fn close(got: f64, want: f64, what: &str) {
    let off = got - want;
    assert!(off.abs() <= 1e-9 * want.abs(), "{what}: {got} != {want} (off by {off:e})");
}

// -- meridians of the stack's turned fixtures ---------------------------------------------------

/// A polygon's meridian, its sides tagged in turn.
fn polygon(pts: &[P], tags: &[u32]) -> Region {
    let k = pts.len();
    Region::new(vec![(0..k).map(|i| Step {
        seg: Seg::Line { a: pts[i], b: pts[(i + 1) % k] },
        chart: None,
        tag: tags[i],
    }).collect()])
}

/// A disc of radius `r` from `z0` to `z1` about the axis: bottom, side, top, axis tagged
/// `tag`, `tag + 1`, `tag + 2`, `tag + 3`.
fn slab(r: f64, z0: f64, z1: f64, tag: u32) -> Region {
    polygon(&[[0.0, z0], [r, z0], [r, z1], [0.0, z1]], &[tag, tag + 1, tag + 2, tag + 3])
}

/// #79's frustum: 15 across its foot at 10, 5 across its top at 30; foot, slant, top, axis
/// tagged 11 to 14.
fn frustum() -> Region {
    polygon(&[[0.0, 10.0], [15.0, 10.0], [5.0, 30.0], [0.0, 30.0]], &[11, 12, 13, 14])
}

fn faces(tag: u32, n: u32) -> Vec<u32> { (tag..tag + n).collect() }

fn union(a: &Region, b: &Region) -> Region { boolean(a, b, Op::Union, 1e-9).unwrap() }

/// The stack's `root_ring`: the ring a ball of radius `r` fills where a cylinder of radius `big`
/// stands on a plane — the square at the corner less the quarter disc, turned (Pappus).
fn root_ring(big: f64, r: f64) -> f64 {
    let moment = r * r * (big + r / 2.0) - PI * r * r / 4.0 * (big + r - 4.0 * r / (3.0 * PI));
    2.0 * PI * moment
}

/// The stack's `turned_wedge`: the volume the ball of radius `r` in the meridian corner at `corner`
/// between unit directions `d` leaves turned, by its section in closed form.
fn turned_wedge(corner: P, d: [P; 2], r: f64) -> f64 {
    let half = dot(d[0], d[1]).acos() / 2.0;
    let s = r / half.tan();
    let t = d.map(|u| add(corner, scale(u, s)));
    let c = add(corner, scale(unit(add(d[0], d[1])), r / half.sin()));
    let angle = |p: P| (p[1] - c[1]).atan2(p[0] - c[0]);
    let (a0, mut a1) = (angle(t[0]), angle(t[1]));
    while a1 - a0 > PI { a1 -= TAU; }
    while a0 - a1 > PI { a1 += TAU; }
    let f = flux(&Seg::Line { a: corner, b: t[0] }) + flux(&Seg::Arc { c, r, a0, sweep: a1 - a0 })
        + flux(&Seg::Line { a: t[1], b: corner });
    2.0 * PI * f.abs()
}

/// #79's pad: a boss of radius 8 standing 20 on a plate 10 thick, its root filled to 3, a bore of
/// radius 4 drilled through after.  The plate is a 60 × 40 prism; in the meridian it is a disc
/// wide enough that the ring stands on its top, which is all the ring reads.
#[test]
fn a_boss_root_is_its_ring() {
    let (big, r, small) = (8.0, 3.0, 4.0);
    let plate = slab(20.0, -10.0, 0.0, 1);
    let boss = slab(big, 0.0, 20.0, 11);
    let pad = union(&plate, &boss);
    let rounded = round(&pad, &faces(11, 4), &faces(1, 4), r, 100).unwrap();
    close(turned(&rounded) - turned(&pad), root_ring(big, r), "the root ring");
    let bore = slab(small, -10.0, 20.0, 21);
    let drilled = boolean(&rounded, &bore, Op::Cut, 1e-9).unwrap();
    let want = PI * 20.0 * 20.0 * 10.0 + PI * big * big * 20.0 + root_ring(big, r)
        - PI * small * small * 30.0;
    close(turned(&drilled), want, "the drilled pad");
}

/// #79's turned fixture (`x0/turned.sv`): a frustum standing on a disc, its foot filled to 3 and
/// its top rim rounded to 1 — concave and convex by the same word.
#[test]
fn a_frustum_root_and_rim_are_tori() {
    let plate = slab(40.0, 0.0, 10.0, 1);
    let body = union(&plate, &frustum());
    let root = round(&body, &faces(11, 4), &faces(1, 4), 3.0, 100).unwrap();
    let rim = round(&root, &[13], &[12], 1.0, 101).unwrap();
    let slant = unit([-10.0, 20.0]);
    let want_root = turned_wedge([15.0, 10.0], [[1.0, 0.0], slant], 3.0);
    let want_rim = turned_wedge([5.0, 30.0], [[-1.0, 0.0], scale(slant, -1.0)], 1.0);
    close(turned(&root) - turned(&body), want_root, "the root");
    close(turned(&root) - turned(&rim), want_rim, "the rim");
    let frustum_v = PI * 20.0 / 3.0 * (15.0 * 15.0 + 15.0 * 5.0 + 5.0 * 5.0);
    close(turned(&rim), PI * 40.0 * 40.0 * 10.0 + frustum_v + want_root - want_rim, "the body");
    // the order of two rounds on different corners is no matter
    let rim_first = round(&body, &[13], &[12], 1.0, 101).unwrap();
    let other = round(&rim_first, &faces(11, 4), &faces(1, 4), 3.0, 100).unwrap();
    close(turned(&other), turned(&rim), "the rounds the other way about");
}

/// #83's knob (`x0/knob.sv`): a rod standing on a disc and entering a ball, its root filled to 2
/// (plane against cylinder) and its neck to 1.5 (cylinder against sphere, a section with a curved
/// side).
#[test]
fn a_knobs_root_and_neck_turn_their_sections() {
    let (s, big, rod, r): (P, f64, f64, f64) = ([0.0, 44.0], 8.0, 4.0, 1.5);
    let disc = slab(30.0, 0.0, 8.0, 1);
    let shaft = slab(rod, 8.0, 40.0, 11);
    let ball = Region::new(vec![vec![
        Step { seg: Seg::Arc { c: s, r: big, a0: -PI / 2.0, sweep: PI }, chart: None, tag: 21 },
        Step { seg: Seg::Line { a: [0.0, s[1] + big], b: [0.0, s[1] - big] }, chart: None,
               tag: 22 },
    ]]);
    let knob = union(&union(&disc, &shaft), &ball);
    let root = round(&knob, &faces(11, 4), &faces(1, 4), 2.0, 100).unwrap();
    let neck = round(&root, &faces(11, 4), &[21, 22], r, 101).unwrap();
    // the stack's closed forms: the neck's section is the corner, its foot on the rod, the arc of
    // the ball rolled, its foot on the ball and the ball's arc back
    let corner = [rod, s[1] - (big * big - rod * rod).sqrt()];
    let centre = [rod + r, s[1] - ((big + r).powi(2) - (rod + r).powi(2)).sqrt()];
    let (on_rod, on_ball) = ([rod, centre[1]], add(s, scale(unit(sub(centre, s)), big)));
    let at = |c: P, p: P| (p[1] - c[1]).atan2(p[0] - c[0]);
    let arc = |c: P, r: f64, from: P, to: P| {
        Seg::Arc { c, r, a0: at(c, from), sweep: at(c, to) - at(c, from) }
    };
    let neck_v = 2.0 * PI * (flux(&Seg::Line { a: corner, b: on_rod })
        + flux(&arc(centre, r, on_rod, on_ball)) + flux(&arc(s, big, on_ball, corner))).abs();
    let root_v = root_ring(rod, 2.0);
    close(turned(&root) - turned(&knob), root_v, "the root");
    close(turned(&neck) - turned(&root), neck_v, "the neck");
    let cap = { let h = corner[1] - (s[1] - big); PI * h * h * (3.0 * big - h) / 3.0 };
    let want = PI * 30.0 * 30.0 * 8.0 + PI * rod * rod * (corner[1] - 8.0)
        + 4.0 / 3.0 * PI * big.powi(3) - cap + root_v + neck_v;
    close(turned(&neck), want, "the knob");
}

/// A chamfer of the frustum's rim is a cone band: the triangle it takes, turned (Pappus).  Both
/// legs measured from the edge, then from the other face: at the rim's corner (a turn of 117°)
/// the two differ by `1 / sin`.
#[test]
fn a_rim_chamfer_takes_its_triangle() {
    let frustum = frustum();
    let (corner, slant, top) = ([5.0, 30.0], unit([10.0, -20.0]), [-1.0, 0.0]);
    // a triangle's area times the path of its centroid
    let pappus = |a: P, b: P, c: P| {
        2.0 * PI * (cross(sub(b, a), sub(c, a)) / 2.0).abs() * (a[0] + b[0] + c[0]) / 3.0
    };
    let d = 1.0;
    let by_edge = chamfer(&frustum, &[12], &[13], [Leg::FromEdge(d); 2], 100).unwrap();
    let want = pappus(corner, add(corner, scale(slant, d)), add(corner, scale(top, d)));
    close(turned(&frustum) - turned(&by_edge), want, "legs from the edge");
    let sin = cross(slant, top).abs();
    let by_other = chamfer(&frustum, &[12], &[13], [Leg::FromOther(d); 2], 100).unwrap();
    let want = pappus(corner, add(corner, scale(slant, d / sin)), add(corner, scale(top, d / sin)));
    close(turned(&frustum) - turned(&by_other), want, "legs from the other face");
    // and the two legs told apart: the slant's 2, the top's 1
    let two = chamfer(&frustum, &[12], &[13], [Leg::FromEdge(2.0), Leg::FromEdge(1.0)], 100)
        .unwrap();
    let want = pappus(corner, add(corner, scale(slant, 2.0)), add(corner, scale(top, 1.0)));
    close(turned(&frustum) - turned(&two), want, "two legs");
}

/// A size of zero is the identity, decided by value; a ball the faces cannot hold is refused,
/// naming the faces — under the definition it would consume one (#183 X0, finding 9).
#[test]
fn zero_is_the_identity_and_a_ball_too_big_is_refused() {
    let frustum = frustum();
    let same = round(&frustum, &[12], &[13], 0.0, 100).unwrap();
    assert_eq!(turned(&same), turned(&frustum));
    let why = round(&frustum, &[12], &[13], 20.0, 100).unwrap_err();
    assert!(why.contains("12") && why.contains("13"), "{why}");
    // a disc's side is 10 tall: rounds of 6 at both its ends meet within it
    let why = round(&slab(40.0, 0.0, 10.0, 1), &[2], &[1, 3], 6.0, 100).unwrap_err();
    assert!(why.contains("meet within it"), "{why}");
}

// -- the hypoid's drawn rounding and end chamfer -------------------------------------------------

fn entity(e: &Elaborated, name: &str) -> usize {
    e.map.ent_named(name).unwrap_or_else(|| panic!("no `{name}`")).i()
}

/// #145's crown tip rounding (`crown/rounding.sv`'s preview): a section's two tip corners rounded
/// by `TipRounding`'s drawn tangent arcs.  The sharp section, from the drawn lines' carriers,
/// rounded by the prototype: the same centres and feet.
#[test]
fn the_crown_tip_rounding_is_the_balls() {
    let e = fixtures::read(include_str!("../../examples/spiral_bevel/crown/rounding.sv"));
    let sk = &e.sketch;
    // std.front is the world's (x, z)
    let at = |name: &str| {
        let p = sk.world_point(entity(&e, name));
        assert!(p[1].abs() < 1e-12);
        [p[0], p[2]]
    };
    let carrier = |a: &str, b: &str| Carrier::Line { p: at(a), d: unit(sub(at(b), at(a))) };
    let meet = |a: Carrier, b: Carrier| a.meet(b)[0];
    let (outer, tip, inner) = (carrier("bo", "oj"), carrier("ot", "it"), carrier("ij", "bi"));
    let sharp = polygon(&[at("bi"), at("bo"), meet(outer, tip), meet(tip, inner)], &[1, 2, 3, 4]);
    let r = 0.3 * 2.0;
    let rounded = round(&round(&sharp, &[2], &[3], r, 10).unwrap(), &[3], &[4], r, 11).unwrap();
    let arc = |tag: u32| rounded.loops[0].iter().find(|s| s.tag == tag).unwrap().seg;
    let mut worst: f64 = 0.0;
    let arcs = [(10, "outer_round", ["oj", "ot"]), (11, "inner_round", ["it", "ij"])];
    for (tag, name, feet) in arcs {
        let Seg::Arc { c, r: got, .. } = arc(tag) else { panic!("a round is an arc") };
        let drawn = &sk.arcs[entity(&e, name)];
        let centre = sk.world_point(drawn.center as usize);
        worst = worst.max(dist(c, [centre[0], centre[2]])).max((got - r).abs());
        let s = arc(tag);
        let ends = [s.start(), s.end()];
        for f in feet {
            worst = worst.max(ends.iter().map(|p| dist(*p, at(f))).fold(f64::INFINITY, f64::min));
        }
    }
    assert!(worst < 1e-9, "the prototype's rounds leave the drawn ones by {worst:e}");
}

/// #145's end relief (`blank/member.sv`'s preview, `blank/ends.sv`'s `EndChamfer`): the blank's
/// meridian, folded by the core from its region's terms, chamfered at the tip's toe and heel
/// corners by the prototype, against the drawn chamfers' ends and the volume the body rule builds
/// (the region less the two drawn rings, by the kernel).  `EndChamfer` measures each leg from the
/// other face: its end on the tip `size` inside the sphere, its end on the sphere `size` below the
/// tip cone.  Measured from the edge instead, both legs move by `1 / sin` of the corner's angle.
#[test]
fn the_end_relief_is_a_chamfer_from_the_other_face() {
    let e = fixtures::gear::read(include_str!("../../examples/spiral_bevel/blank/member.sv"),
        &fixtures::gear::project());
    let sk = &e.sketch;
    let solid = entity(&e, "member.material");
    let m = gcs_core::solid::region::meridian(sk, solid).unwrap();
    // every term is named for the region (`m.names`), not for the set it came from, so a face is
    // told by its shape and side: the tip the cone it is inside, the heel the ball it is inside,
    // the toe the ball it is outside
    let SolidDef::Region { terms, .. } = &sk.solids[solid].def else { panic!("a region") };
    let tag = |ball: bool, cmp: Cmp| terms.iter().position(|t| {
        matches!(t.shape, RegionShape::Ball { .. }) == ball && t.cmp == cmp
            && matches!(t.shape, RegionShape::Ball { .. } | RegionShape::Cone { .. })
    }).unwrap() as u32 + 1;
    let (tip, toe, heel) = (tag(false, Cmp::Le), tag(true, Cmp::Ge), tag(true, Cmp::Le));
    let to_meridian = |p: [f64; 3]| {
        let d = sub3(p, m.origin);
        [norm3(cross3(d, m.axis)), dot3(d, m.axis)]
    };
    let size = 0.2;
    let legs = [Leg::FromOther(size); 2];
    let ends = |legs: [Leg; 2]| {
        let toe_end = chamfer(&m.region, &[tip], &[toe], legs, 100).unwrap();
        chamfer(&toe_end, &[tip], &[heel], legs, 101).unwrap()
    };
    let relieved = ends(legs);
    // the chamfers' ends, the drawn ones'
    let copied = |end: &str, part: &str| e.map.names.values().flatten()
        .find(|n| n.starts_with("member.") && n.ends_with(&format!("{end}_end.{part}")))
        .unwrap_or_else(|| panic!("no `{part}` of the {end} end")).clone();
    let mut worst: f64 = 0.0;
    for (end, t) in [("toe", 100), ("heel", 101)] {
        let flat = relieved.loops.iter().flatten().find(|s| s.tag == t).unwrap().seg;
        for part in ["along_tip", "down_end"] {
            let drawn = to_meridian(sk.world_point(entity(&e, &copied(end, part))));
            worst = worst.max(dist(flat.start(), drawn).min(dist(flat.end(), drawn)));
        }
    }
    assert!(worst < 1e-9, "the prototype's chamfers leave the drawn ones by {worst:e}");
    // the volumes: the sharp meridian turned against the region's exact solid, and the chamfered
    // one against the body the rings are cut from
    let volume = |name: &str| {
        let x = sk.exact_solid(entity(&e, name)).unwrap_or_else(|why| panic!("{name}: {why}"));
        gcs_core::brep::props::volume(&x.brep)
    };
    close(turned(&m.region), volume("member.material"), "the sharp blank");
    let body = volume("body");
    close(turned(&relieved), body, "the relieved blank");
    // legs measured from the edge instead
    let by_edge = ends([Leg::FromEdge(size); 2]);
    let (taken, short) = (turned(&m.region) - body, turned(&by_edge) - turned(&relieved));
    let (of_chamfers, of_blank) = (short / taken, short / body);
    eprintln!("end relief: the chamfers take {taken:.6} mm³; legs from the edge take {short:.3e} \
               mm³ less ({of_chamfers:.3e} of the chamfers', {of_blank:.3e} of the blank)");
    assert!(short > 0.0 && short / taken < 1e-2, "{short} of {taken}");
}
