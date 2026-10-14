//! What the gesture tests share (`edit`, `edit_solids`, `edit_rounds`, `extrude`, `revolve`,
//! `workspace`, …): a document built and solved, an edit taken or reconciled, a solid's volume and
//! where the eye sees a point.
use gcs_core::edit::{self, Kind};
use gcs_core::library;
use gcs_core::program::{elaborate, Elaborated};
use gcs_core::solid::ApproximationPolicy::Report;
use gcs_core::solve::{solve, SolveOpts};

/// `src` parsed and linked with no error, elaborated with none, and solved.
pub fn build(src: &str) -> Elaborated {
    let (p, errors, linked) = library::parse_linked(src);
    assert!(errors.is_empty() && linked.is_empty(), "{errors:?} {linked:?}\n{src}");
    let mut e = elaborate(&p);
    assert!(e.ok(), "{:?}\n{src}", e.errors().map(|d| d.message.clone()).collect::<Vec<_>>());
    assert!(solve(&mut e.sketch, SolveOpts::default()).success);
    e
}

/// Names as owned strings, as an edit takes them.
pub fn names(n: &[&str]) -> Vec<String> {
    n.iter().map(|s| s.to_string()).collect()
}

/// The volume of the solid the document calls `name`.
pub fn volume(e: &Elaborated, name: &str) -> f64 {
    e.sketch.evaluated_solid(e.map.ent_named(name).unwrap().i(), Report).unwrap().volume()
}

/// `a` within `rel` of `b`, relative (absolute below 1).
pub fn near(a: f64, b: f64, rel: f64) -> bool {
    (a - b).abs() <= rel * b.abs().max(1.0)
}

/// The eye's picture-plane coordinates of a point in space, written out from `overview::eye`'s
/// convention: the viewer at (cos az cos el, sin az cos el, sin el), looking at the origin.
pub fn seen(x: [f64; 3], az: f64, el: f64) -> (f64, f64) {
    let right = [-az.sin(), az.cos(), 0.0];
    let up = [-az.cos() * el.sin(), -az.sin() * el.sin(), el.cos()];
    let dot = |a: [f64; 3], b: [f64; 3]| a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    (dot(right, x), dot(up, x))
}

/// `reconcile` applies itself to the elaboration, which is the point of it — so a test that wants
/// both the text and the elaboration afterwards has to hand it the whole thing.
pub fn reconciled(e: &mut Elaborated) -> edit::Edit {
    let sk = std::mem::take(&mut e.sketch);
    let out = edit::reconcile(e, &sk);
    e.sketch = sk;
    out
}

/// The text of an edit taken: structural, and refused nothing.
pub fn taken(e: edit::Edit) -> String {
    assert_eq!(e.refused, None);
    assert_eq!(e.kind, Kind::Structural);
    e.text
}
