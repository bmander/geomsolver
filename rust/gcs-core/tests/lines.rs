//! **The method of lines** (#339 D4): a body over two formals whose relations read a rate along
//! either — a first-order PDE — marched along the first and sampled along the second, at the
//! Gauss–Lobatto nodes of pieces of its interval.  The gate is an ellipse's offset front, moving
//! inward at unit speed square to itself (`rust/examples/rates/front.sv`): its points and both
//! its derivatives against `E(u) + d·n(u)`, the march's convergence in its pieces measured and
//! held to the march on twice the pieces, its fold refused short of `b²/a` where its least radius
//! of curvature runs out, a contact on the surface solved and its Jacobian held to differences,
//! the document's own point, claim and fold, and each statement the split refuses.

use crate::common::{build, parse, solved};
use gcs_core::program::elaborate;
use gcs_core::traced::Need;

const FRONT: &str = include_str!("../../examples/rates/front.sv");
/// The front's gauge, as the document states it.
const GAUGE: &str = "  rate(p over d) parallel nl                 // the gauge: square to itself\n";

/// The ellipse (40, 25) at eccentric angle `u` (degrees), moved `d` along its inward normal, and
/// that point's derivatives in `d` and in `u` (per degree): `x_d = n`, `x_u = E′ + d n′`.
fn front(d: f64, u: f64) -> [[f64; 3]; 3] {
    let (a, b) = (40.0, 25.0);
    let r = u.to_radians();
    let (x, y) = (a * r.cos(), b * r.sin());
    let (tx, ty) = (-a * r.sin(), b * r.cos());
    let l = (tx * tx + ty * ty).sqrt();
    let n = [-ty / l, tx / l];
    // n′ = −κ |E′| t̂ turned: the unit normal turns at the curvature times the speed
    let (ax, ay) = (-a * r.cos(), -b * r.sin());
    let kappa = (tx * ay - ty * ax) / (l * l * l);
    let dn = [-kappa * tx, -kappa * ty];
    let deg = std::f64::consts::PI / 180.0;
    [[x + d * n[0], y + d * n[1], 0.0], [n[0], n[1], 0.0],
     [(tx + d * dn[0]) * deg, (ty + d * dn[1]) * deg, 0.0]]
}

fn far(a: [f64; 3], b: [f64; 3]) -> f64 {
    (0..3).map(|k| (a[k] - b[k]).abs()).fold(0.0, f64::max)
}

fn surface(e: &gcs_core::program::Elaborated) -> usize {
    let r = e.map.ent_named("front").expect("no `front`");
    assert_eq!(r.kind, gcs_core::model::EntKind::Traced);
    r.i()
}

/// **An offset front is marched to its closed form.**  On the 32 pieces a surface is sampled on
/// by default, the front stands at `E(u) + d·n(u)` at every place read — between nodes too, where
/// it is the quintic through a piece's — and its derivatives are the closed form's: `x_d` the unit
/// inward normal (the law and the gauge), `x_u` the ellipse's tangent stretched as it focuses.
#[test]
fn an_offset_front_is_marched_to_its_closed_form() {
    let e = build(FRONT);
    let sk = solved(&e);
    let i = surface(&e);
    // the point, x_d and x_u (per degree): at the home the quintics' own error between nodes,
    // then the march's, growing as the front focuses
    let bounds = [(0.0, [1e-11, 5e-10, 1e-10]), (3.0, [2e-8, 2e-8, 2e-7]),
                  (6.0, [1.5e-7, 1e-7, 1.5e-6]), (9.0, [8e-7, 5e-7, 7e-6]),
                  (12.0, [6e-6, 5e-6, 6e-5])];
    for (d, tol) in bounds {
        let (mut at, mut along, mut across) = (0.0f64, 0.0f64, 0.0f64);
        for k in 0..=240 {
            let u = -60.0 + 0.5 * k as f64;
            let r = gcs_core::traced::reading(&sk, i, [d, u], Need::Tangent)
                .unwrap_or_else(|| panic!("no reading at ({d}, {u})"));
            let w = front(d, u);
            at = at.max(far(r.position, w[0]));
            along = along.max(far(r.d[0], w[1]));
            across = across.max(far(r.d[1], w[2]));
        }
        assert!(at < tol[0] && along < tol[1] && across < tol[2],
                "at d = {d}: the point {at:.1e}, x_d {along:.1e}, x_u {across:.1e}");
    }
    assert_eq!(gcs_core::report::traced_text(&sk, i),
               "front: a surface traced in space, marched from its home at 0 across (0, 15)");
}

/// **The march converges in its pieces, and is held to the march on twice them** (D0's
/// decision 4).  The error at d = 6 and 12 falls by more than sixteen each time the pieces are
/// doubled — about fifth order, the quintics' (on 64 pieces it is 1.7e-9 and 6.1e-8, the
/// record's table) — and the distance from the march on twice the pieces, all `lines::check`
/// can know, is the error's own size: it bounds it.
#[test]
fn the_march_converges_in_its_pieces() {
    let e = build(FRONT);
    let sk = solved(&e);
    let i = surface(&e);
    let error = |pieces: usize, d: f64| -> f64 {
        (0..=120).map(|k| {
            let u = -60.0 + k as f64;
            let r = gcs_core::traced::lines_at(&sk, i, pieces, 15.0, [d, u], Need::Point)
                .unwrap_or_else(|| panic!("{pieces} pieces: no reading at ({d}, {u})"));
            far(r.x, front(d, u)[0])
        }).fold(0.0, f64::max)
    };
    let mut table = Vec::new();
    for pieces in [8, 16, 32] {
        table.push((pieces, error(pieces, 6.0), error(pieces, 12.0)));
    }
    for w in table.windows(2) {
        let ((p, a6, a12), (_, b6, b12)) = (w[0], w[1]);
        assert!(a6 / b6 > 16.0 && a12 / b12 > 16.0,
                "from {p} pieces: {a6:.1e} → {b6:.1e} at d = 6, {a12:.1e} → {b12:.1e} at d = 12");
    }
    assert!(table[2].1 < 1e-7 && table[2].2 < 3e-6, "{table:?}");
    let held = gcs_core::traced::lines_check(&sk, i, 16, 15.0, &[6.0, 12.0]).unwrap();
    let (e6, e12) = (table[1].1, table[1].2);
    assert!(held >= 0.5 * e12.max(e6) && held <= 2.0 * e12.max(e6),
            "the march on 32 pieces is {held:.1e} from the one on 16, whose error is {e12:.1e}");
}

/// **The front folds where its least radius of curvature runs out, and is refused there.**  At
/// d = b²/a = 15.625 the ends of the major axis focus: rank [x_d, x_u] falls to one.  The march is
/// refused where a node's area `|x_d × x_u|` falls below a hundredth of its home's — b²/a·0.99 =
/// 15.47 for the closed form — at the node at u = 0, short of the fold, and nearer it on twice the
/// pieces: the march's own error grows as the front focuses, so the coarser one errs early (on
/// 16, 32 and 64 pieces 14.66, 15.11 and 15.35).
#[test]
fn the_front_folds_where_its_least_radius_runs_out() {
    let src = FRONT.replace("d in (0mm, 15mm)", "d in (0mm, 16mm)");
    let e = build(&src);
    let sk = solved(&e);
    let i = surface(&e);
    let fold = 25.0f64.powi(2) / 40.0 * (1.0 - gcs_core::lines::FOLD);
    let at = |pieces: usize| -> f64 {
        match gcs_core::traced::lines_reached(&sk, i, pieces, 16.0) {
            Some(Err(Some((refusal, gcs_core::lines::Why::Fold { sigma })))) => {
                assert!(sigma.abs() < 1e-9, "{pieces} pieces: folds at u = {sigma}");
                refusal.at()
            }
            other => panic!("{pieces} pieces: {other:?}"),
        }
    };
    let (coarse, fine) = (at(16), at(32));
    assert!(coarse > 14.5 && coarse < fine && fine > 15.0 && fine < fold,
            "refused at {coarse} on 16 pieces, {fine} on 32; the closed form's {fold}");
    let said = gcs_core::report::traced_text(&sk, i);
    assert!(said.starts_with("front: a surface traced in space, marched from its home at 0, \
        refused at 15.1") && said.ends_with(": it folds there, near 0 along its second formal"),
            "{said}");
    // short of the fold the surface is read; past where it was refused, not
    assert!(gcs_core::traced::reading(&sk, i, [15.0, 0.0], Need::Point).is_some());
    assert!(gcs_core::traced::reading(&sk, i, [15.5, 0.0], Need::Point).is_none());
}

/// **The document holds a point where the closed form is, and says where `folded` is refused.**
/// `held`, on the major axis 35 from the centre, is found on the front at its place (5, 0°);
/// the line along the surface's normal there is the plane's, the document's one claim a
/// theorem; and the same front run to 16mm is refused short of its fold, as `solventc` says.
#[test]
fn the_document_holds_its_point_and_reports_its_fold() {
    let mut e = crate::common::parsed::determined(FRONT);
    let contact = e.sketch.constraints.iter()
        .find(|c| c.kind == gcs_core::constraints::CKind::PointOnTraced).unwrap().clone();
    let place: Vec<f64> = contact.local_values(&e.sketch)[3..5].to_vec();
    assert!((place[0] - 5.0).abs() < 1e-8 && place[1].abs() < 1e-7, "{place:?}");
    let d = gcs_core::diagnose::diagnose(&mut e.sketch, Default::default());
    assert!(d.claims_violated.is_empty() && d.claims_consuming.is_empty()
            && d.claims_theorem.len() == 1, "{d:?}");
    let folded = e.map.ent_named("folded").unwrap().i();
    let said = gcs_core::report::traced_text(&e.sketch, folded);
    assert!(said.starts_with("folded: a surface traced in space, marched from its home at 0, \
        refused at 15.1") && said.ends_with(": it folds there, near 0 along its second formal"),
            "{said}");
}

/// **A contact owns its place on the marched surface.**  A point held where the closed form is at
/// (9, −25°) is found there; the contact's Jacobian — in the point, the place, and the geometry the
/// body is written over, the last through the variational equations carried by the march — is its
/// residual's own, read against central differences with a step a march's tolerance does not
/// drown.  A line along the surface's normal there is checked alike, reading the derivatives'
/// sensitivities, and so are the document's own two.
#[test]
fn a_contact_owns_its_place_on_the_marched_surface() {
    let w = front(9.0, -25.0)[0];
    let src = format!("{FRONT}
h := point hint(({}, {}, 0))
fix(({}, {}, {})) h
h coincident front
top := point hint(({}, {}, 10))
nl := line(h, top)
front perpendicular(at: h) nl
", w[0] + 0.3, w[1] - 0.2, w[0], w[1], w[2], w[0], w[1]);
    let e = build(&src);
    let mut sk = solved(&e);
    // the last: the document holds a point of its own
    let contact = sk.constraints.iter().rev()
        .find(|c| c.kind == gcs_core::constraints::CKind::PointOnTraced).unwrap().clone();
    let place: Vec<f64> = contact.local_values(&sk)[3..5].to_vec();
    assert!((place[0] - 9.0).abs() < 1e-7 && (place[1] + 25.0).abs() < 1e-6, "{place:?}");
    let top = sk.world_point(e.map.ent_named("top").unwrap().i());
    assert!((top[0] - w[0]).abs() < 1e-7 && (top[1] - w[1]).abs() < 1e-7, "{top:?}");
    // every column free and moved off the solution
    for p in sk.params.iter_mut() {
        p.fixed = false;
    }
    let mut rng = gcs_core::rng::Rng::new(7);
    for p in sk.params.iter_mut() {
        p.value += rng.uniform(-0.02, 0.02) * (1.0 + p.value.abs().min(10.0));
    }
    // a rate of the contact too, the normal and the tangent being rates (#365)
    for c in sk.constraints.iter()
        .filter(|c| c.kind.traced_contact().is_some() || c.rate_source().is_some()) {
        let v = c.local_values(&sk);
        let rows = c.n_residuals();
        let got = c.jacobian(&sk, &v);
        // the lead, the place, and two columns that move the ellipse — its centre, and the axis
        // its plane is square to — each a march of its own
        let lead = v.len() - 2 - 12;
        let cols: Vec<usize> = (0..lead + 2).chain([lead + 2, lead + 11]).collect();
        for k in cols {
            let h = 1e-4 * v[k].abs().max(1.0);
            let (mut lo, mut hi) = (v.clone(), v.clone());
            lo[k] -= h;
            hi[k] += h;
            let (a, b) = (c.residual(&sk, &lo), c.residual(&sk, &hi));
            for r in 0..rows {
                let fd = (b[r] - a[r]) / (2.0 * h);
                let j = got[r * v.len() + k];
                assert!((fd - j).abs() < 1e-6 * (1.0 + j.abs()),
                        "{:?}: row {r}, column {k}: {j} against {fd}", c.kind);
            }
        }
    }
}

/// The errors a document elaborates to.
fn errors(src: &str) -> Vec<String> {
    let (prog, errs) = parse(src);
    if !errs.is_empty() {
        return errs.into_iter().map(|e| e.message).collect();
    }
    elaborate(&prog).errors().map(|d| format!("{} {}", d.code.as_str(), d.message)).collect()
}

/// **A front without its gauge is refused, naming its freedom.**  The law alone, with the front
/// kept in its plane, leaves the rate along it free: a node may slide along the front as it moves,
/// one freedom among `p`'s rates.
#[test]
fn a_front_without_its_gauge_names_its_freedom() {
    let src = FRONT.replace(GAUGE, "  rate(p over d) perpendicular za\n");
    let errs = errors(&src);
    assert!(errs.iter().any(|e| e.starts_with("E103 a body that states a rate must determine its \
        rates along `d`: 3 unknowns against 0 rows and 2 rate rows — its rates leave 1 freedom \
        among those of `p`")), "{errs:?}");
}

/// **What the split refuses** (`lines.rs`): a rate along the sampled formal of a point no rate
/// row moves, a row reading one that places only points the rates move (it would tie a piece's
/// nodes together), `initially` placing a point the rows place, two rates along the marched
/// formal in a row, a rate along a formal the body does not run over, and a sampled formal's rate
/// in a body that states none along the marched one.  And a curvature contact on a marched
/// surface: its Jacobian is a third order the march does not carry.
#[test]
fn what_the_split_refuses() {
    let rows = [
        ("  nl perpendicular rate(p over u)            // ...square to the front\n",
         "  nl perpendicular rate(k over u)\n",
         "`rate(k over u)` is of a point no rate row moves along `d`"),
        ("  nl perpendicular rate(p over u)            // ...square to the front\n",
         "  nl perpendicular rate(p over u)\n  za perpendicular rate(p over u)\n",
         "this relation reads a rate along `u` and places only points the rates move"),
        ("    c distance(0mm, along: za) p\n",
         "    c distance(0mm, along: za) p\n    c distance(0mm, along: za) k\n",
         "`initially` says where the points the rates move start, and `k` is placed"),
        (GAUGE,
         "  rate(p over d) parallel rate(k over d)\n",
         "a relation reads one rate along `d`"),
        (GAUGE,
         "  rate(p over a) parallel nl\n",
         "is read along `a`, and the surface runs over `d` and `u`"),
    ];
    for (from, to, said) in rows {
        assert!(FRONT.contains(from), "{from}");
        let errs = errors(&FRONT.replace(from, to));
        assert!(errs.iter().any(|e| e.contains(said)), "{to}: {errs:?}");
    }
    let errs = errors(&FRONT
        .replace(GAUGE, "")
        .replace("  rate(p over d) distance(1, along: nl)      // the law: at unit speed\n", ""));
    assert!(errs.iter().any(|e| e.contains("this body states no rate along `d`: say how it moves")),
            "{errs:?}");
    let errs = errors(&format!("{FRONT}
h := point hint((30, 0, 0))
h coincident front
in std.front {{
  o := point hint((10, 0))
  k := circle(center: o) hint(r: 5)
}}
front curvature(at: h) k
"));
    assert!(errs.iter().any(|e| e.contains("`front` is marched by the method of lines, and its \
        curvature is not yet read")), "{errs:?}");
}

/// **A whole turn of an angle closes.**  The front over the whole ellipse, `u` in (0°, 360°): its
/// last node is its first, so no end stands where the front has none, and the surface reads the
/// same a turn on.  On the whole turn its 32 pieces are three times the arc's length, and the error
/// is theirs.
#[test]
fn a_whole_turn_closes() {
    let src = FRONT.replace("u in (-60deg, 60deg)", "u in (0deg, 360deg)")
        .replace("d in (0mm, 15mm)", "d in (0mm, 6mm)");
    let e = build(&src);
    let sk = solved(&e);
    let i = surface(&e);
    let mut worst = 0.0f64;
    for k in 0..=72 {
        let u = 5.0 * k as f64;
        let r = gcs_core::traced::reading(&sk, i, [6.0, u], Need::Point).unwrap();
        worst = worst.max(far(r.position, front(6.0, u)[0]));
    }
    assert!(worst < 3e-5, "{worst:.1e}");
    let (a, b) = (gcs_core::traced::reading(&sk, i, [4.0, 10.0], Need::Tangent).unwrap(),
                  gcs_core::traced::reading(&sk, i, [4.0, 370.0], Need::Tangent).unwrap());
    assert!(far(a.position, b.position) < 1e-12 && far(a.d[1], b.d[1]) < 1e-12);
}



/* -- Cauchy data a motion carries (#377) ------------------------------------------------------ */

const CIRCLES: &str = include_str!("../../examples/rates/geodesic_circles.sv");
const HOUSING: &str = include_str!("../../examples/rates/housing.sv");

/// `geodesic_circles.sv` as it was written before a rate body read a carried point: each starting
/// circle its own two ordinates, `r1·cos u` and `r1·sin u`, across the axis.
const CIRCLES_WRITTEN: &str = "unit mm
use std
R := 30mm
r1 := 15mm
half := 30deg
component OnSphere(c: point, x1: point, y1: point, R: Length, r1: Length, d: Length, u: Angle) {
  xa := line(c, x1)
  ya := line(c, y1)
  p := point hint((0, 0, 30))
  k := point hint((0, 0, -30))
  rad := line(c, p)
  nl := line(p, k)
  c distance(R) p
  length(10mm) nl
  nl perpendicular rad
  nl perpendicular rate(p over u)
  rate(p over d) parallel nl
  rate(p over d) distance(1, along: nl)
  initially {
    c distance(r1 * cos(u), along: xa) p
    c distance(r1 * sin(u), along: ya) p
  }
}
component OnCone(o: point, x1: point, y1: point, top: point, half: Angle, r1: Length, d: Length,
                 u: Angle) {
  xa := line(o, x1)
  ya := line(o, y1)
  ax := line(o, top)
  p := point hint((0, 0, 30))
  k := point hint((0, 0, 200))
  w := point hint((0, 0, 50))
  gen := line(o, p)
  cn := line(p, w)
  nl := line(p, k)
  gen angle(half) ax
  w coincident ax
  cn perpendicular gen
  length(10mm) nl
  nl perpendicular cn
  nl perpendicular rate(p over u)
  rate(p over d) parallel nl
  rate(p over d) distance(1, along: nl)
  initially {
    o distance(r1 * cos(u), along: xa) p
    o distance(r1 * sin(u), along: ya) p
  }
}
c := point hint((0, 0, 0))
fix((0, 0, 0)) c
x1 := point hint((10, 0, 0))
fix((10, 0, 0)) x1
y1 := point hint((0, 10, 0))
fix((0, 10, 0)) y1
top := point hint((0, 0, 10))
fix((0, 0, 10)) top
on_sphere := OnSphere(c, x1, y1, R: R, r1: r1).p over (d in (0mm, pi * R), u in (0deg, 360deg))
on_cone := OnCone(c, x1, y1, top, half: half, r1: r1).p over (d in (0mm, 30mm),
                                                            u in (0deg, 360deg))
";

/// `housing.sv` as it was written before: its Cauchy data the epitrochoid's two terms.
const HOUSING_WRITTEN: &str = "unit mm
use std
R := 75mm
e := 15mm
s := 3mm
component Bore(c: point, x1: point, y1: point, z1: point, R: Length, e: Length, d: Length,
               u: Angle) {
  xa := line(c, x1)
  ya := line(c, y1)
  za := line(c, z1)
  p := point hint((0, 0, 0))
  k := point hint((0, 0, 0))
  nl := line(p, k)
  length(10mm) nl
  c distance(0mm, along: za) k
  nl perpendicular rate(p over u)
  rate(p over d) parallel nl
  rate(p over d) distance(-1, along: nl)
  initially {
    c distance(e * cos(3 * u) + R * cos(u), along: xa) p
    c distance(e * sin(3 * u) + R * sin(u), along: ya) p
    c distance(0mm, along: za) p
  }
}
c := point hint((0, 0, 0))
fix((0, 0, 0)) c
x1 := point hint((10, 0, 0))
fix((10, 0, 0)) x1
y1 := point hint((0, 10, 0))
fix((0, 10, 0)) y1
z1 := point hint((0, 0, 10))
fix((0, 0, 10)) z1
bore := Bore(c, x1, y1, z1, R: R, e: e).p over (d in (0mm, s), u in (0deg, 180deg))
";

/// The greatest distance between two surfaces, `name` in each document, at `d` over `u` every
/// 2.5° of `(lo, hi)`: the point, `x_d` and `x_u`.
fn apart(a: &gcs_core::model::Sketch, ea: &gcs_core::program::Elaborated,
         b: &gcs_core::model::Sketch, eb: &gcs_core::program::Elaborated, name: &str, d: f64,
         lo: f64, hi: f64) -> [f64; 3] {
    let (i, j) = (ea.map.ent_named(name).unwrap().i(), eb.map.ent_named(name).unwrap().i());
    let mut out = [0.0f64; 3];
    let n = ((hi - lo) / 2.5) as usize;
    for k in 0..=n {
        let u = lo + 2.5 * k as f64;
        let (r, s) = (gcs_core::traced::reading(a, i, [d, u], Need::Tangent).unwrap(),
                      gcs_core::traced::reading(b, j, [d, u], Need::Tangent).unwrap());
        out[0] = out[0].max(far(r.position, s.position));
        out[1] = out[1].max(far(r.d[0], s.d[0]));
        out[2] = out[2].max(far(r.d[1], s.d[1]));
    }
    out
}

/// **Cauchy data a motion carries is the data written out** (#377).  `geodesic_circles.sv`'s
/// starting circles are a drawn point turned about the axis, and `housing.sv`'s trochoid the apex
/// the rotor's motion carries; each surface, and both its derivatives, is the one its Cauchy data
/// written as ordinates marches to — at the home to rounding, and after to the march's own
/// tolerance, under a hundredth of the error against the closed form
/// (`rate_examples::fronts_on_a_surface_are_geodesic_circles`).
#[test]
fn cauchy_data_a_motion_carries_is_the_data_written_out() {
    let (ea, eb) = (build(CIRCLES), build(CIRCLES_WRITTEN));
    let (a, b) = (solved(&ea), solved(&eb));
    for (name, d, tol) in [("on_sphere", 0.0, 1e-11), ("on_sphere", 20.0, 1e-10),
                           ("on_sphere", 40.0, 2e-10), ("on_sphere", 70.0, 1e-8),
                           ("on_cone", 0.0, 1e-11), ("on_cone", 30.0, 1e-9)] {
        let got = apart(&a, &ea, &b, &eb, name, d, 0.0, 360.0);
        assert!(got.iter().all(|&x| x < tol), "{name} at {d}: {got:?}");
    }
    let (ea, eb) = (build(HOUSING), build(HOUSING_WRITTEN));
    let (a, b) = (solved(&ea), solved(&eb));
    for (d, tol) in [(0.0, 1e-11), (3.0, 1e-10)] {
        let got = apart(&a, &ea, &b, &eb, "bore", d, 0.0, 180.0);
        assert!(got.iter().all(|&x| x < tol), "the bore at {d}: {got:?}");
    }
}

/// **A contact on a front from carried data reads its sensitivities exactly** (#377): the home's
/// implicit-function step reads the carried point's row over jets, so a point held on the bore,
/// off the solution, has a Jacobian in the apex and in the centres the motion turns about that is
/// its residual's own, read against central differences, and each of them moves it.
#[test]
fn a_contact_on_carried_data_is_exact_in_its_columns() {
    let w = [(2.0f64, 50.0f64)].map(|(d, u)| {
        // the epitrochoid moved `d` along its outward normal (`rate_examples::housing`)
        let (r, e, u) = (75.0f64, 15.0f64, u.to_radians());
        let (x, y) = (e * (3.0 * u).cos() + r * u.cos(), e * (3.0 * u).sin() + r * u.sin());
        let (tx, ty) = (-3.0 * e * (3.0 * u).sin() - r * u.sin(),
                        3.0 * e * (3.0 * u).cos() + r * u.cos());
        let l = tx.hypot(ty);
        [x + d * ty / l, y - d * tx / l, 0.0]
    })[0];
    let src = format!("{HOUSING}
h := point hint(({}, {}, 0))
fix(({}, {}, {})) h
h coincident bore
", w[0], w[1], w[0], w[1], w[2]);
    let e = build(&src);
    let mut sk = e.sketch.clone();
    let contact = sk.constraints.iter().rev()
        .find(|c| c.kind == gcs_core::constraints::CKind::PointOnTraced).unwrap().clone();
    let place: Vec<f64> = contact.local_values(&sk)[3..5].to_vec();
    assert!((place[0] - 2.0).abs() < 0.2 && (place[1] - 50.0).abs() < 2.0, "seeded at {place:?}");
    for p in sk.params.iter_mut() {
        p.fixed = false;
    }
    let mut rng = gcs_core::rng::Rng::new(7);
    for p in sk.params.iter_mut() {
        p.value += rng.uniform(-0.02, 0.02) * (1.0 + p.value.abs().min(10.0));
    }
    let v = contact.local_values(&sk);
    let rows = contact.n_residuals();
    let got = contact.jacobian(&sk, &v);
    // the lead (3), the place (2), then `θ`: `c`, `z1`, the apex, and the rotor's motion — the
    // shaft's centre and its view's two directions, then the hub and its view's.  Each column of
    // `θ` is a march of its own: the apex's three and each turn's centre along `x`
    let theta = 5;
    let cols: Vec<usize> =
        (0..theta).chain(theta + 6..theta + 9).chain([theta + 9, theta + 18]).collect();
    assert_eq!(v.len(), theta + 9 + 18);
    for k in cols {
        let h = 1e-4 * v[k].abs().max(1.0);
        let (mut lo, mut hi) = (v.clone(), v.clone());
        lo[k] -= h;
        hi[k] += h;
        let (a, b) = (contact.residual(&sk, &lo), contact.residual(&sk, &hi));
        for r in 0..rows {
            let fd = (b[r] - a[r]) / (2.0 * h);
            let j = got[r * v.len() + k];
            assert!((fd - j).abs() < 1e-6 * (1.0 + j.abs()),
                    "row {r}, column {k}: {j} against {fd}");
        }
        // the carried data moves the surface: each of these columns moves the point held on it
        assert!((0..rows).any(|r| got[r * v.len() + k].abs() > 1e-3), "column {k} moves nothing");
    }
}
