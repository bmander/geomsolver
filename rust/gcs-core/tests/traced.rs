//! **Bodies traced in space** (#326, R3 and R5): a point of a component as one formal runs (a
//! curve in space) or two (a surface), its body solved in space.  Each is held here to closed
//! forms worked out apart from `traced`: a helix (stated by its ordinates, and as a point a screw
//! carries), and a torus — its point, its normal and the osculating circle of its meridian.
use crate::common::build;
use gcs_core::constraints::CKind;
use gcs_core::fdcheck::check_constraint;
use gcs_core::model::{EntKind, Sketch};
use gcs_core::rng::Rng;
use gcs_core::solve;
use gcs_core::space::{dot, norm, sub, unit};
use gcs_core::traced::{self, Need};

/// Three axes through the origin, drawn in space as lines, for a body to measure along.
const AXES: &str = "\
unit mm
use std
o := point hint((0, 0, 0))
fix((0, 0, 0)) o
X1 := point hint((10, 0, 0))
fix((10, 0, 0)) X1
Y1 := point hint((0, 10, 0))
fix((0, 10, 0)) Y1
Z1 := point hint((0, 0, 10))
fix((0, 0, 10)) Z1
xl := line(o, X1)
yl := line(o, Y1)
zl := line(o, Z1)
";

/// A helix of radius `r` and lead `lead` about `z`, by the point's three ordinates; and the same
/// helix as the point a screw along `z` carries from `(5, 0, 0)`.
const HELIX: &str = "\
component Helix(o: point, x: line, y: line, z: line, r: Length, lead: Length, u: Angle) {
  p := point hint((r * cos(u), r * sin(u), lead * u / 360deg))
  o distance(r * cos(u), along: x) p
  o distance(r * sin(u), along: y) p
  o distance(lead * u / 360deg, along: z) p
}
k := trace(Helix(o, xl, yl, zl, r: 5mm, lead: 3mm).p over u in (0deg, 720deg))
s0 := point hint((5, 0, 0))
fix((5, 0, 0)) s0
screw := frame
std.Turn(screw, about: zl, ratio: 1, advance: 3mm)
component Screwed(p0: point, m: frame, u: Angle) {
  p := point hint((5, 0, 0))
  p coincident(under: m, t == u) p0
}
h := trace(Screwed(s0, screw).p over u in (0deg, 720deg))
";

/// A torus of major radius 30 and minor 8 about `z`, by its point's three ordinates over two
/// formals: `phi` round the axis and `s` round the tube.
const TORUS: &str = "\
component Torus(o: point, x: line, y: line, z: line, phi: Angle, s: Angle) {
  p := point hint(((30 + 8 * cos(s)) * cos(phi), (30 + 8 * cos(s)) * sin(phi), 8 * sin(s)))
  o distance((30mm + 8mm * cos(s)) * cos(phi), along: x) p
  o distance((30mm + 8mm * cos(s)) * sin(phi), along: y) p
  o distance(8mm * sin(s), along: z) p
}
T := trace(Torus(o, xl, yl, zl).p over (phi in (-180deg, 180deg), s in (-180deg, 180deg)))
";

fn helix(u: f64) -> [f64; 3] {
    let r = u.to_radians();
    [5.0 * r.cos(), 5.0 * r.sin(), 3.0 * u / 360.0]
}

fn solved(src: &str) -> gcs_core::program::Elaborated {
    let mut e = build(src);
    let r = solve::solve(&mut e.sketch, Default::default());
    assert!(r.success, "{r:?}");
    e
}

fn at(e: &gcs_core::program::Elaborated, name: &str) -> [f64; 3] {
    e.sketch.world_point(e.map.ent_named(name).unwrap_or_else(|| panic!("no `{name}`")).i())
}

fn traced_named(e: &gcs_core::program::Elaborated, name: &str) -> usize {
    let r = e.map.ent_named(name).unwrap_or_else(|| panic!("no `{name}`"));
    assert_eq!(r.kind, EntKind::Traced, "`{name}` is traced in space");
    r.i()
}

/// **A curve traced in space is a helix where its body says one is.**  The body's point, read at
/// places across two turns, is the closed form's, and its tangent too; written as the point a
/// screw carries, the same curve.  A point held to it with its height stated is where the helix
/// is that high, and a line held along its tangent there runs along the helix's.
#[test]
fn a_curve_traced_in_space_is_the_helix_its_body_states() {
    let src = format!("{AXES}{HELIX}\
q := point hint((-5, 0.5, 1.4))
q coincident k
o distance(1.5mm, along: zl) q
q2 := point hint((-5, -5, 2))
tl := line(q, q2)
k tangent(at: q) tl
q distance(10mm) q2
w := point hint((0, 5, 0.7))
w coincident h
o distance(0.75mm, along: zl) w
");
    let e = solved(&src);
    let sk = &e.sketch;
    for name in ["k", "h"] {
        let i = traced_named(&e, name);
        assert_eq!(sk.traced[i].formals(), 1);
        for u in [0.0, 37.0, 180.0, 333.3, 500.0, 719.0] {
            let r = traced::reading(sk, i, [u, 0.0], Need::Tangent).unwrap();
            assert!(norm(sub(r.position, helix(u))) < 1e-9, "{name} at {u}: {:?}", r.position);
            // per degree: the closed form's derivative
            let a = u.to_radians();
            let d = [-5.0 * a.sin(), 5.0 * a.cos(), 3.0 / std::f64::consts::TAU]
                .map(|x| x * std::f64::consts::PI / 180.0);
            assert!(norm(sub(r.d[0], d)) < 1e-9, "{name} at {u}: {:?} against {d:?}", r.d[0]);
        }
    }
    assert!(norm(sub(at(&e, "q"), helix(180.0))) < 1e-9, "{:?}", at(&e, "q"));
    let along = unit(sub(at(&e, "q2"), at(&e, "q")));
    let t = unit([0.0, -5.0, 3.0 / (2.0 * std::f64::consts::PI)]);
    assert!((dot(along, t).abs() - 1.0).abs() < 1e-10, "{along:?} against {t:?}");
    assert!(norm(sub(at(&e, "w"), helix(90.0))) < 1e-9, "{:?}", at(&e, "w"));
}

/// The torus's closed forms: its point at `(φ, s)`, its unit normal (outward from the tube), and
/// the tube's centre in the meridian — the centre of the meridian's osculating circle.
fn torus(phi: f64, s: f64) -> ([f64; 3], [f64; 3], [f64; 3]) {
    let (p, q) = (phi.to_radians(), s.to_radians());
    let rho = 30.0 + 8.0 * q.cos();
    let n = [q.cos() * p.cos(), q.cos() * p.sin(), q.sin()];
    ([rho * p.cos(), rho * p.sin(), 8.0 * q.sin()], n, [30.0 * p.cos(), 30.0 * p.sin(), 0.0])
}

/// **A surface traced in space is the torus its body states**, with the contacts a surface takes.
/// Its point and normal at places across it are the closed form's; a point held on it at a stated
/// place is there, a line along its normal there is along the closed form's, a plane touching it
/// there is square to it, and a circle osculating its section by the meridian plane there is the
/// tube's: its centre the tube's centre and its radius the tube's.
#[test]
fn a_surface_traced_in_space_is_the_torus_its_body_states() {
    let src = format!("{AXES}{TORUS}\
p := point hint((30, 9, 3))
p coincident(u == 20, s == 50) T
n1 := point hint((40, 14, 9))
nl := line(p, n1)
T perpendicular(at: p) nl
p distance(10mm) n1
// the meridian plane through p: along the radius at 20° and the axis
r1 := point hint((9.4, 3.4, 0))
fix((9.39692620786, 3.42020143326, 0)) r1
rl := line(o, r1)
meridian := plane(u: rl, v: zl)
in meridian {{
  c := point hint((28, 3))
  osc := circle(center: c) hint(r: 7)
}}
T curvature(at: p) osc
");
    let e = solved(&src);
    let sk = &e.sketch;
    let i = traced_named(&e, "T");
    assert_eq!(sk.traced[i].formals(), 2);
    let mut rng = Rng::new(326);
    for _ in 0..20 {
        let (phi, s) = (rng.uniform(-170.0, 170.0), rng.uniform(-170.0, 170.0));
        let r = traced::reading(sk, i, [phi, s], Need::Curvature).unwrap();
        let (x, n, _) = torus(phi, s);
        assert!(norm(sub(r.position, x)) < 1e-9, "({phi}, {s}): {:?} against {x:?}", r.position);
        let got = r.normal.unwrap();
        assert!((dot(got, n).abs() - 1.0).abs() < 1e-10, "({phi}, {s}): {got:?} against {n:?}");
        let t = &sk.traced[i];
        assert!(traced::regularity(&traced::consts_of(t), &traced::theta(sk, t), [phi, s]).unwrap()
                > 0.5);
    }
    let (x, n, centre) = torus(20.0, 50.0);
    assert!(norm(sub(at(&e, "p"), x)) < 1e-9, "{:?}", at(&e, "p"));
    let along = unit(sub(at(&e, "n1"), at(&e, "p")));
    assert!((dot(along, n).abs() - 1.0).abs() < 1e-10, "{along:?} against {n:?}");
    let k = e.map.ent_named("osc").unwrap();
    let c = sk.world_point(sk.round_center(k));
    let radius = sk.params[sk.round_radius(k)].value;
    assert!(norm(sub(c, centre)) < 1e-8, "{c:?} against {centre:?}");
    assert!((radius - 8.0).abs() < 1e-8, "{radius}");
}

/// A traced curve and surface's contacts against finite differences: at the solve, and at points
/// knocked off it, every column — the place, the body's geometry, and what the contact states
/// about.
#[test]
fn traced_contacts_agree_with_finite_differences() {
    let src = format!("{AXES}{HELIX}{TORUS}\
q := point hint((-5, 0.5, 1.4))
q coincident k
q2 := point hint((-5, -5, 2))
tl := line(q, q2)
k tangent(at: q) tl
cc := point hint((0, 0, 1))
k curvature(at: q) cc
w := point hint((0, 5, 0.7))
w coincident h
p := point hint((30, 9, 3))
p coincident T
n1 := point hint((40, 14, 9))
nl := line(p, n1)
T perpendicular(at: p) nl
m1 := point hint((25, 20, 9))
ml := line(p, m1)
T tangent(at: p) ml
r1 := point hint((9.4, 3.4, 0))
rl := line(o, r1)
meridian := plane(u: rl, v: zl)
in meridian {{
  c := point hint((28, 3))
  osc := circle(center: c) hint(r: 7)
}}
T curvature(at: p) osc
T tangent(at: p) meridian
rate(q over u) perpendicular nl
");
    let (prog, errs) = crate::common::parse(&src);
    assert!(errs.is_empty(), "{errs:?}");
    let el = gcs_core::program::elaborate(&prog);
    assert!(el.ok(), "{:?}", el.errors().map(|d| &d.message).collect::<Vec<_>>());
    let mut sk: Sketch = el.sketch;
    let traced = |c: &&gcs_core::constraints::Constraint| {
        c.kind.traced_contact().is_some() || c.rate_source().is_some()
    };
    assert!(sk.constraints.iter().any(|c| c.kind == CKind::PointOnTraced));
    // a curve's rate, held along a line and square to one, and a surface's frame, every word of
    // it: one kind, one kernel (#365)
    use gcs_core::contact_rate::Word;
    let words: Vec<Word> = sk.constraints.iter().filter(|c| c.kind == CKind::Rate)
        .map(|c| c.rate_word()).collect();
    for w in [Word::Parallel, Word::Square, Word::InPlane, Word::Normal, Word::Plane,
              Word::Curvature, Word::Centre] {
        assert!(words.contains(&w), "{w:?} in {words:?}");
    }
    // the fixed axes' points are free here, so the body's geometry is differentiated too
    for p in sk.params.iter_mut() {
        p.fixed = false;
    }
    let mut rng = Rng::new(7);
    for round in 0..3 {
        if round > 0 {
            for p in sk.params.iter_mut() {
                p.value += rng.uniform(-0.02, 0.02) * (1.0 + p.value.abs().min(10.0));
            }
        }
        for c in sk.constraints.iter().filter(traced) {
            let err = check_constraint(&sk, c, 1e-6, 1e-6)
                .unwrap_or_else(|e| panic!("round {round}, {:?}: {e}", c.kind));
            assert!(err.is_finite());
        }
    }
}

/// **A surface is reported regular, or where it folds** (#326 R5): the torus is regular over its
/// whole domain; a sheet whose second derivative vanishes along a line — its point squared in the
/// second formal — folds there, and the report says so, near where.
#[test]
fn a_traced_surface_is_reported_regular_or_where_it_folds() {
    let e = solved(&format!("{AXES}{TORUS}"));
    let i = traced_named(&e, "T");
    let said = gcs_core::report::traced_text(&e.sketch, i);
    assert_eq!(said, "T: a surface traced in space, regular over its domain");
    let folded = format!("{AXES}\
component Sheet(o: point, x: line, y: line, z: line, a: Length, b: Length) {{
  p := point hint((a, b * b / 10mm, 0))
  o distance(a, along: x) p
  o distance(b * b / 10mm, along: y) p
  o distance(0mm, along: z) p
}}
F := trace(Sheet(o, xl, yl, zl).p over (a in (-6, 6), b in (-6, 6)))
");
    let e = solved(&folded);
    let i = traced_named(&e, "F");
    let said = gcs_core::report::traced_text(&e.sketch, i);
    assert!(said.starts_with("F: a surface traced in space, folding near (")
            && said.ends_with(", 0)"), "{said}");
}
