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
use gcs_core::brep::corners::{chamfer, round, Carrier, Leg};
use gcs_core::brep::planar::{boolean, Op, Region, Seg, Step};
use gcs_core::model::{RegionShape, SolidDef};
use gcs_core::program::Elaborated;
use gcs_core::space::norm as norm3;
use gcs_core::syntax::Cmp;
use std::f64::consts::{PI, TAU};

use crate::fillet_definition::root_ring;

type P = [f64; 2];

fn sub(a: P, b: P) -> P { [a[0] - b[0], a[1] - b[1]] }
fn add(a: P, b: P) -> P { [a[0] + b[0], a[1] + b[1]] }
fn scale(a: P, k: f64) -> P { [a[0] * k, a[1] * k] }
fn dot(a: P, b: P) -> f64 { a[0] * b[0] + a[1] * b[1] }
#[allow(dead_code)]
fn cross(a: P, b: P) -> f64 { a[0] * b[1] - a[1] * b[0] }
fn norm(a: P) -> f64 { a[0].hypot(a[1]) }
fn dist(a: P, b: P) -> f64 { norm(sub(a, b)) }
fn unit(a: P) -> P { scale(a, 1.0 / norm(a)) }
#[allow(dead_code)]
fn left(a: P) -> P { [-a[1], a[0]] }

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
        // a rational quadratic: eight-point Gauss–Legendre over 16 stretches, to roundoff
        Seg::Conic { .. } => (0..16).map(|k| gcs_core::integral::GAUSS8.iter().map(|&(x, w)| {
            let t = (k as f64 + 0.5 + 0.5 * x) / 16.0;
            let (p, d) = (s.point(t), s.tangent(t));
            w * p[0] * p[0] / 2.0 * d[1]
        }).sum::<f64>() / 32.0).sum(),
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

/// The stack's closed form of #83's knob's neck: a rod of radius `rod` about the axis entering a
/// ball of radius `big` about `s`, filled to `r`.  Its section is the corner, its foot on the rod,
/// the arc of the ball rolled, its foot on the ball and the ball's arc back, turned.
pub(crate) fn knob_neck(s: P, big: f64, rod: f64, r: f64) -> f64 {
    let corner = [rod, s[1] - (big * big - rod * rod).sqrt()];
    let centre = [rod + r, s[1] - ((big + r).powi(2) - (rod + r).powi(2)).sqrt()];
    let (on_rod, on_ball) = ([rod, centre[1]], add(s, scale(unit(sub(centre, s)), big)));
    let at = |c: P, p: P| (p[1] - c[1]).atan2(p[0] - c[0]);
    let arc = |c: P, r: f64, from: P, to: P| {
        Seg::Arc { c, r, a0: at(c, from), sweep: at(c, to) - at(c, from) }
    };
    2.0 * PI * (flux(&Seg::Line { a: corner, b: on_rod })
        + flux(&arc(centre, r, on_rod, on_ball)) + flux(&arc(s, big, on_ball, corner))).abs()
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
    close(turned(&root) - turned(&knob), root_ring(rod, 2.0), "the root");
    close(turned(&neck) - turned(&root), knob_neck(s, big, rod, r), "the neck");
    close(turned(&neck), knob_rounded(s, big, rod, r), "the knob");
}

/// #83's knob rounded, whole (`x0/knob.sv`, `rounds/knob.sv`): the disc (30 by 8), the rod up to
/// the ball, the ball less the cap the rod holds, its root's ring (2) and its neck's (`knob_neck`).
pub(crate) fn knob_rounded(s: P, big: f64, rod: f64, r: f64) -> f64 {
    let corner = s[1] - (big * big - rod * rod).sqrt();
    let cap = { let h = corner - (s[1] - big); PI * h * h * (3.0 * big - h) / 3.0 };
    PI * 30.0 * 30.0 * 8.0 + PI * rod * rod * (corner - 8.0) + 4.0 / 3.0 * PI * big.powi(3) - cap
        + root_ring(rod, 2.0) + knob_neck(s, big, rod, r)
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

/// #145's crown tip rounding as it was drawn (`fixtures/tip_rounding.sv`'s preview): a section's
/// two tip corners rounded by `TipRounding`'s drawn tangent arcs.  The sharp section, from the drawn lines' carriers,
/// rounded by the prototype: the same centres and feet.
#[test]
fn the_crown_tip_rounding_is_the_balls() {
    let e = fixtures::read(include_str!("fixtures/tip_rounding.sv"));
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

/// `fixtures/member_blank.sv`'s preview (the spiral bevel's blank as its own step drew it) with an
/// end relief of `relief` mm, and its region solid's meridian.
fn member(relief: f64) -> (Elaborated, usize) {
    let text = include_str!("fixtures/member_blank.sv");
    assert!(text.contains("end_relief: 0.2mm"));
    let text = text.replace("end_relief: 0.2mm", &format!("end_relief: {relief}mm"));
    let e = fixtures::gear::read(&text);
    let solid = entity(&e, "member.material");
    (e, solid)
}

/// The tags of a blank's tip, toe and heel in its meridian: every term is named for the region
/// (`m.names`), not for the set it came from, so a face is told by its shape and side — the tip
/// the cone it is inside, the heel the ball it is inside, the toe the ball it is outside.
fn tip_toe_heel(e: &Elaborated, solid: usize) -> [u32; 3] {
    let SolidDef::Region { terms, .. } = &e.sketch.solids[solid].def else { panic!("a region") };
    let tag = |ball: bool, cmp: Cmp| terms.iter().position(|t| {
        matches!(t.shape, RegionShape::Ball { .. }) == ball && t.cmp == cmp
            && matches!(t.shape, RegionShape::Ball { .. } | RegionShape::Cone { .. })
    }).unwrap() as u32 + 1;
    [tag(false, Cmp::Le), tag(true, Cmp::Ge), tag(true, Cmp::Le)]
}

/// #145's end relief, `std.Chamfered` in the blank's own region (`fixtures/member_blank.sv`), **lowered**
/// (#196): the meridian's toe and heel corners where the tip cone meets each sphere are cut by a
/// conic's arc whose ends are the prototype's chamfer's, each leg measured from the other face —
/// on the tip `size` inside the sphere, on the sphere `size` below the tip cone — and the exact
/// solid turns it to the meridian's volume.  Between its ends it bows off the straight chamfer
/// (`EndChamfer`'s, retired) by the sphere's curvature, a little of the chamfer's material.
#[test]
fn the_end_relief_is_lowered_from_the_other_face() {
    let size = 0.2;
    let (sharp, s0) = member(0.0);
    let m0 = gcs_core::solid::region::meridian(&sharp.sketch, s0).unwrap();
    let (e, solid) = member(size);
    let sk = &e.sketch;
    assert!(!gcs_core::solid::region::in_general(sk, solid));
    let m = gcs_core::solid::region::meridian(sk, solid).unwrap();
    let [tip, toe, heel] = tip_toe_heel(&sharp, s0);
    let legs = [Leg::FromOther(size); 2];
    let straight = chamfer(&chamfer(&m0.region, &[tip], &[toe], legs, 100).unwrap(), &[tip], &[heel], legs, 101).unwrap();
    let conics: Vec<Seg> = m.region.loops.iter().flatten().map(|s| s.seg)
        .filter(|s| matches!(s, Seg::Conic { .. })).collect();
    assert_eq!(conics.len(), 2, "a flat at each end");
    let mut worst: f64 = 0.0;
    for t in [100, 101] {
        let flat = straight.loops.iter().flatten().find(|s| s.tag == t).unwrap().seg;
        let near = conics.iter().map(|c| dist(c.start(), flat.start()).min(dist(c.end(), flat.start()))
            + dist(c.start(), flat.end()).min(dist(c.end(), flat.end()))).fold(f64::INFINITY, f64::min);
        worst = worst.max(near);
    }
    assert!(worst < 1e-9, "the flats' ends leave the legs' by {worst:e}");
    let x = sk.exact_solid(solid).unwrap();
    close(gcs_core::brep::props::volume(&x.brep) / x.mm.powi(3), turned(&m.region), "the relieved blank");
    // the bow, against the material the chamfers take
    let (taken, bow) = (turned(&m0.region) - turned(&straight), turned(&straight) - turned(&m.region));
    eprintln!("end relief: the straight chamfers take {taken:.6} mm³; the bowed flats {bow:+.3e} mm³ \
               more ({:+.2e} of it)", bow / taken);
    assert!(taken > 0.0 && bow != 0.0 && bow.abs() < 2e-3 * taken, "{bow} of {taken}");
}

/// The conic the lowering builds is the set's own edge: on it the sum of the distances from the
/// tip cone and the sphere, each over the other's leg, is one (to roundoff) — and between its ends
/// it stands off its chord by the bow X3 measured, 7e-5 mm of a 0.2 mm leg.
#[test]
fn the_end_reliefs_conic_is_its_sum_of_distances() {
    let size = 0.2;
    let (sharp, s0) = member(0.0);
    let m0 = gcs_core::solid::region::meridian(&sharp.sketch, s0).unwrap();
    let (e, solid) = member(size);
    let m = gcs_core::solid::region::meridian(&e.sketch, solid).unwrap();
    let [tip, toe, heel] = tip_toe_heel(&sharp, s0);
    let step = |t: u32| m0.region.loops.iter().flatten().find(|s| s.tag == t).unwrap().seg;
    let Seg::Line { a: ta, b: tb } = step(tip) else { panic!("the tip is a line") };
    // signed distances, positive in the material (on a step's left)
    let to_tip = |q: P| cross(sub(tb, ta), sub(q, ta)) / dist(ta, tb);
    let (mut worst, mut widest): (f64, f64) = (0.0, 0.0);
    for end in [toe, heel] {
        let Seg::Arc { c, r, sweep, .. } = step(end) else { panic!("an end is an arc") };
        let to_sphere = |q: P| sweep.signum() * (r - dist(q, c));
        // the conic at this end: the one whose ends lie on this sphere and the tip
        let conic = m.region.loops.iter().flatten().map(|s| s.seg)
            .find(|s| matches!(s, Seg::Conic { .. }) && [s.start(), s.end()].iter().any(|&p| to_sphere(p).abs() < 1e-9))
            .unwrap();
        let (a, b) = (conic.start(), conic.end());
        let chord = |q: P| cross(sub(b, a), sub(q, a)) / dist(a, b);
        for k in 0..=200 {
            let q = conic.point(k as f64 / 200.0);
            worst = worst.max((to_tip(q) / size + to_sphere(q) / size - 1.0).abs());
            widest = widest.max(chord(q).abs());
        }
    }
    eprintln!("end relief: the conic's sum within {worst:.1e}; it bows {widest:.2e} mm off its chord");
    assert!(worst < 1e-10, "the conic leaves its sum by {worst:e}");
    assert!(widest > 1e-5 && widest < 1e-3 * size, "{widest}");
}

/// The lowering builds what the set says: about each flat, the field of the relieved blank's
/// meridian and the region read point by point agree wherever a point is not on the boundary.  A
/// relief of nothing weighs both distances by nothing: the bound holds everywhere and the region is
/// its four bounds.
#[test]
fn the_end_relief_lowered_is_the_region_read_point_by_point() {
    let (e, solid) = member(0.2);
    let sk = &e.sketch;
    let m = gcs_core::solid::region::meridian(sk, solid).unwrap();
    let lowered = gcs_core::solid::MaterialField::read(sk, solid, 1e-10).unwrap();
    let general = gcs_core::solid::region::general(sk, solid).unwrap();
    let lift = |q: P| -> [f64; 3] { std::array::from_fn(|k| m.origin[k] + q[0] * m.seam[k] + q[1] * m.axis[k]) };
    let mut checked = 0;
    for st in m.region.loops.iter().flatten().filter(|s| matches!(s.seg, Seg::Conic { .. })) {
        let c = st.seg.point(0.5);
        for i in 0..=30 {
            for j in 0..=30 {
                // off the vertices' own ordinates (the parity rule's edge, finding 9)
                let q = [c[0] + (i as f64 - 15.0) * 0.01 + 1.37e-4, c[1] + (j as f64 - 15.0) * 0.01 + 2.91e-4];
                let p = lift(q);
                let g = general.value(p);
                if g.abs() > 1e-9 {
                    assert_eq!(lowered.side(p) < 0.0, g < 0.0, "at {q:?}: {} against {g}", lowered.side(p));
                    checked += 1;
                }
            }
        }
    }
    assert!(checked > 1800, "{checked}");
    // and the lowered field is the distance from the meridian's boundary, the conic's included:
    // each step's nearest point by its own closed form, a conic's by a dense walk refined
    let nearest = |q: P| m.region.loops.iter().flatten().map(|st| match st.seg {
        Seg::Line { a, b } => {
            let d = sub(b, a);
            let t = (dot(sub(q, a), d) / dot(d, d)).clamp(0.0, 1.0);
            dist(q, add(a, scale(d, t)))
        }
        Seg::Arc { c, r, a0, sweep } => {
            let phi = (q[1] - c[1]).atan2(q[0] - c[0]);
            let along = ((phi - a0) * sweep.signum()).rem_euclid(TAU);
            if along <= sweep.abs() { (dist(q, c) - r).abs() } else { dist(q, st.seg.start()).min(dist(q, st.seg.end())) }
        }
        Seg::Conic { .. } => {
            let d = |t: f64| dist(q, st.seg.point(t));
            let k = (0..=4000).min_by(|&i, &j| d(i as f64 / 4000.0).total_cmp(&d(j as f64 / 4000.0))).unwrap();
            let (mut lo, mut hi) = ((k.max(1) - 1) as f64 / 4000.0, (k + 1).min(4000) as f64 / 4000.0);
            for _ in 0..200 {
                let (x, y) = (lo + (hi - lo) / 3.0, hi - (hi - lo) / 3.0);
                if d(x) < d(y) { hi = y } else { lo = x }
            }
            d(0.5 * (lo + hi))
        }
    }).fold(f64::INFINITY, f64::min);
    let mut worst: f64 = 0.0;
    for st in m.region.loops.iter().flatten().filter(|s| matches!(s.seg, Seg::Conic { .. })) {
        let c = st.seg.point(0.5);
        for i in 0..=10 {
            for j in 0..=10 {
                let q = [c[0] + (i as f64 - 5.0) * 0.03 + 1.37e-4, c[1] + (j as f64 - 5.0) * 0.03 + 2.91e-4];
                worst = worst.max((lowered.side(lift(q)).abs() - nearest(q)).abs());
            }
        }
    }
    assert!(worst < 1e-9, "the lowered field leaves the boundary's distance by {worst:e}");
    let (sharp, s0) = member(0.0);
    let SolidDef::Region { terms, .. } = &sharp.sketch.solids[s0].def else { panic!("a region") };
    assert_eq!(terms.len(), 4);
    let m0 = gcs_core::solid::region::meridian(&sharp.sketch, s0).unwrap();
    assert!(m0.region.loops.iter().flatten().all(|s| !matches!(s.seg, Seg::Conic { .. })));
}

/// **The crown's tip rounding by the word** (#183 step 6): the section drawn sharp from the same
/// carriers, swept a little either side of its view, its tip's two edges rounded by `round` — the
/// field's boundary passes through `TipRounding`'s drawn arcs, and a point a twentieth of the
/// radius either side of an arc is on that side.
#[test]
fn the_crown_tip_rounding_is_the_words() {
    let text = include_str!("fixtures/tip_rounding.sv").replacen("use std\n", "use std (round)\n", 1);
    let close = text.rfind('}').unwrap();
    let more = "  in std.front {\n\
                    c1 := point hint((6.5, 2.5))\n    c2 := point hint((-0.5, 2.5))\n\
                    c1 coincident outer\n    c1 coincident tip\n    c2 coincident tip\n    c2 coincident inner\n\
                    (sb := line(bi, bo)) -> (so := line(bo, c1)) -> (st := line(c1, c2)) -> (si := line(c2, bi)) -> close\n\
                    sharp := face(sb, so, st, si)\n  }\n\
                  tooth := solid(sharp, from: -1mm, to: 1mm)\n\
                  tooth.st round(r: 0.6mm) tooth.so\n\
                  tooth.st round(r: 0.6mm) tooth.si\n";
    let text = format!("{}{more}{}", &text[..close], &text[close..]);
    let e = fixtures::read(&text);
    let sk = &e.sketch;
    let field = gcs_core::solid::MaterialField::read(sk, entity(&e, "tooth"), 1e-10).unwrap();
    let r = 0.6;
    // what the sharp corners had, a little in from each toward its arc's centre, is taken away
    for (corner, name) in [("c1", "outer_round"), ("c2", "inner_round")] {
        let (p, c) = (sk.world_point(entity(&e, corner)), sk.world_point(sk.arcs[entity(&e, name)].center as usize));
        let q: [f64; 3] = std::array::from_fn(|k| p[k] + 0.05 * (c[k] - p[k]));
        assert!(field.side(q) > 0.0, "{corner}: {}", field.side(q));
    }
    for name in ["outer_round", "inner_round"] {
        let arc = &sk.arcs[entity(&e, name)];
        let c = sk.world_point(arc.center as usize);
        let (s, t) = (sk.world_point(arc.start as usize), sk.world_point(arc.end as usize));
        // the arc's middle, in the view (y = 0), and its outward normal there
        let mid: [f64; 3] = std::array::from_fn(|k| (s[k] + t[k]) / 2.0 - c[k]);
        let n = 1.0 / norm3(mid);
        let out: [f64; 3] = mid.map(|x| x * n);
        let on: [f64; 3] = std::array::from_fn(|k| c[k] + r * out[k]);
        let off = |d: f64| field.side(std::array::from_fn(|k| on[k] + d * out[k]));
        assert!(off(0.0).abs() < 1e-7, "{name}: the field at the drawn arc reads {}", off(0.0));
        assert!(off(-r / 20.0) < 0.0 && off(r / 20.0) > 0.0, "{name}");
    }
}

