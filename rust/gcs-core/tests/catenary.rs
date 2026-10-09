//! The acceptance cases of #121 and #144: a rope stated as what it is — a curve of fixed length
//! between two points that minimises its potential energy — held to the closed-form catenary to
//! the integration's accuracy, reported a minimum, re-hung as an end is dragged, draped over a
//! peg, touched by a line; Dido's arc; and a free length found where the energy is stationary in it.

use gcs_core::model::{EntKind, EntRef, Sketch};
use gcs_core::program::{elaborate, Elaborated};
use gcs_core::solve::{solve, SolveOpts};
use crate::common::parse;

pub fn read(src: &str) -> (Elaborated, Vec<String>) {
    let (prog, errs) = parse(src);
    let e = elaborate(&prog);
    let mut all: Vec<String> = errs.iter().map(|x| format!("syntax: {}", x.message)).collect();
    all.extend(e.diags.iter().map(|d| format!("{}: {}", d.code.as_str(), d.message)));
    (e, all)
}

pub fn solved(src: &str) -> Elaborated {
    let (mut e, d) = read(src);
    assert!(e.ok(), "{d:?}");
    let r = solve(&mut e.sketch, SolveOpts::default());
    assert!(r.success, "{}", r.message);
    e
}

pub const ROPE: &str = "\
use std
in std.front {
  a := point
  b := point
  fix((0, 0)) a
  fix((100, 0)) b
  rope := curve(a, b)
  length(150) rope
}
rope minimizes integral(p.y over p)
";

/// Bisection on `f(x) = 0` over `[lo, hi]`, `f(lo)` and `f(hi)` of opposite signs.
pub fn bisect(mut lo: f64, mut hi: f64, f: impl Fn(f64) -> f64) -> f64 {
    let flo = f(lo);
    for _ in 0..300 {
        let mid = 0.5 * (lo + hi);
        if (f(mid) > 0.0) == (flo > 0.0) { lo = mid } else { hi = mid }
    }
    0.5 * (lo + hi)
}

/// The catenary through `p` and `q` of length `len`, never asked of the core: its height at `x`.
pub fn catenary(p: (f64, f64), q: (f64, f64), len: f64) -> impl Fn(f64) -> f64 {
    let (dx, dy) = (q.0 - p.0, q.1 - p.1);
    let flat = (len * len - dy * dy).sqrt();
    let a = bisect(1e-6, 1e6, |a| 2.0 * a * (dx / (2.0 * a)).sinh() - flat);
    let x0 = 0.5 * (p.0 + q.0) - a * (dy / len).atanh();
    let y0 = p.1 - a * ((p.0 - x0) / a).cosh();
    move |x: f64| y0 + a * ((x - x0) / a).cosh()
}

/// Curve `i` sampled along its length.
pub fn samples(sk: &Sketch, i: usize) -> Vec<(f64, f64)> {
    (0..=400).map(|k| sk.curve_point(i, k as f64 / 400.0)).collect()
}

/// How far curve `i` strays from `y(x)`.
pub fn off(sk: &Sketch, i: usize, y: impl Fn(f64) -> f64) -> f64 {
    samples(sk, i).into_iter().map(|(x, yy)| (yy - y(x)).abs()).fold(0.0, f64::max)
}

fn verdicts(e: &mut Elaborated) -> Vec<&'static str> {
    let d = gcs_core::diagnose::diagnose(&mut e.sketch, gcs_core::diagnose::DiagnoseOptions::default());
    d.extrema.iter().map(|(_, v)| v.name()).collect()
}

fn dof(e: &mut Elaborated) -> i64 {
    gcs_core::diagnose::diagnose(&mut e.sketch, gcs_core::diagnose::DiagnoseOptions::default()).dof
}

/// The free rope hangs as the catenary through its ends — between ends 100 apart, and 20 apart,
/// where its bottom bends at a radius of 2.4, two strands and a turn no discretisation spaced
/// along its length could draw.
#[test]
fn a_free_rope_hangs_as_the_catenary() {
    for d in [100.0, 20.0] {
        let e = solved(&ROPE.replace("fix((100, 0)) b", &format!("fix(({d}, 0)) b")));
        let o = off(&e.sketch, 0, catenary((0.0, 0.0), (d, 0.0), 150.0));
        assert!(o < 1e-9, "chord {d}: off the catenary by {o}");
    }
}

/// The verdict on the hanging rope is a minimum — and on the same rope stood on its head, a
/// standing arch where the height is greatest, a maximum, the rope turned over the chord.
#[test]
fn the_rope_is_a_minimum_and_the_arch_a_maximum() {
    let mut e = solved(ROPE);
    assert_eq!(verdicts(&mut e), ["minimum"]);
    let mut arch = solved(&ROPE.replace("minimizes", "maximizes"));
    let d = gcs_core::diagnose::diagnose(&mut arch.sketch, gcs_core::diagnose::DiagnoseOptions::default());
    assert_eq!(d.extrema.iter().map(|(_, v)| v.name()).collect::<Vec<_>>(), ["maximum"]);
    // and the report says each is what its statement asked for
    let json = gcs_core::report::diagnosis_json(&arch.sketch, &d).dump(None);
    assert!(json.contains(r#","maximum",true]"#), "{json}");
    for ((x, y), (ax, ay)) in samples(&e.sketch, 0).into_iter().zip(samples(&arch.sketch, 0)) {
        assert!((x - ax).abs() < 1e-9 && (y + ay).abs() < 1e-9, "({x}, {y}) against ({ax}, {ay})");
    }
}

/// The rope's ledger: a free curve adds its length, and the length row holds it — so the only
/// freedoms are the ends' own.
#[test]
fn the_ropes_only_freedoms_are_its_ends() {
    let mut held = solved(ROPE);
    assert_eq!(dof(&mut held), 0);
    let mut free = solved(&ROPE.replace("  fix((100, 0)) b\n", "").replace("  b := point\n", "  b := point hint((100, 0))\n"));
    assert_eq!(dof(&mut free), 2);
}

fn b_of(e: &Elaborated) -> usize {
    e.sketch.curves[0].args[1].i()
}

/// An end let go and dragged: every frame the rope hangs again from where the end is, as the
/// catenary through the new ends of the same length — exactly where a rope hung there from the
/// start comes to rest, so the drag carried nothing of where it began.
#[test]
fn dragging_an_end_rehangs_the_rope() {
    let src = ROPE.replace("  fix((100, 0)) b\n", "").replace("  b := point\n", "  b := point hint((100, 0))\n");
    let mut e = solved(&src);
    let b = b_of(&e);
    let mut drag = gcs_core::decompose::PlanDrag::new(&e.sketch, b, 100.0, 0.0, None, 0.05);
    for k in 1..=10 {
        let x = 100.0 - 2.0 * k as f64;
        let r = drag.move_to(&mut e.sketch, None, x, 0.0);
        assert!(r.success, "frame {k}: {r:?}");
        let (bx, by) = e.sketch.point_xy(b);
        assert!((bx - x).abs() < 1e-6 && by.abs() < 1e-6, "frame {k}: b at ({bx}, {by})");
    }
    drag.end();
    let o = off(&e.sketch, 0, catenary((0.0, 0.0), e.sketch.point_xy(b), 150.0));
    assert!(o < 1e-9, "off the catenary by {o}");
}

/// **Dido's problem**: of every curve of length 130 standing on a chord of 100, the one enclosing
/// the most area with it — `∮ (x dy − y dx) / 2`, the chord adding nothing on `y = 0` — is the
/// arc of the circle through the ends with that length, `R sin α = 50`, `2Rα = 130`.
#[test]
fn didos_curve_is_the_circular_arc() {
    let src = ROPE.replace("length(150) rope", "length(130) rope").replace(
        "rope minimizes integral(p.y over p)",
        "rope maximizes integral((p.x * t.y - p.y * t.x) / 2 over (p, t))",
    );
    let mut e = solved(&src);
    let al = bisect(1e-6, std::f64::consts::PI - 1e-9, |a| 130.0 * a.sin() / (2.0 * a) - 50.0);
    let r = 65.0 / al;
    // the arc bulges below the chord: walked from a to b, that is the side the area is counted on
    let cy = r * al.cos();
    let o = samples(&e.sketch, 0).into_iter().map(|(x, y)| ((x - 50.0).hypot(y - cy) - r).abs()).fold(0.0, f64::max);
    assert!(o < 1e-9, "off the arc by {o}");
    assert_eq!(verdicts(&mut e), ["maximum"]);
}

/// **A peg**: a held point the rope is stated to pass is part of its problem.  The rope drapes
/// over it in a corner, two catenaries of the one rope — here mirror images, each the catenary
/// through its ends of half the length — and nothing is left free.
#[test]
fn a_peg_drapes_the_rope_in_a_corner() {
    let src = ROPE.replace(
        "  length(150) rope\n",
        "  length(150) rope\n  peg := point\n  fix((50, -40)) peg\n  peg coincident rope\n",
    );
    let mut e = solved(&src);
    let left = catenary((0.0, 0.0), (50.0, -40.0), 75.0);
    let right = catenary((50.0, -40.0), (100.0, 0.0), 75.0);
    let o = off(&e.sketch, 0, |x| if x <= 50.0 { left(x) } else { right(x) });
    assert!(o < 1e-9, "off the two catenaries by {o}");
    assert_eq!(dof(&mut e), 0);
    assert_eq!(verdicts(&mut e), ["minimum"]);
}

/// **A line drawn tangent to the rope moves to touch it**: the line is free, so the tangency is
/// the drawing's — read through the rope as any curve's contact — and presses on nothing: the
/// rope hangs as the catenary, the line keeps the freedoms the tangency leaves it.
#[test]
fn a_free_line_drawn_tangent_to_the_rope_moves_to_it() {
    let src = ROPE.replace(
        "  length(150) rope\n",
        "  length(150) rope\n  f0 := point hint((20, -40))\n  f1 := point hint((90, -45))\n  floor := line(f0, f1)\n  rope tangent floor\n",
    );
    let mut e = solved(&src);
    let o = off(&e.sketch, 0, catenary((0.0, 0.0), (100.0, 0.0), 150.0));
    assert!(o < 1e-9, "off the catenary by {o}");
    // the line's four coordinates less the two the tangency takes (a touch and a direction), and
    // the contact's own place
    assert_eq!(dof(&mut e), 3);
    assert_eq!(verdicts(&mut e), ["minimum"]);
    // it touches: where the contact is on the rope, the rope is on the line
    let l = &e.sketch.lines[0];
    let (a, b) = (e.sketch.point_xy(l.p1 as usize), e.sketch.point_xy(l.p2 as usize));
    let c = e.sketch.constraints.iter().find(|c| c.kind == gcs_core::constraints::CKind::CurveTangentLine).unwrap();
    let t = e.sketch.params[c.aux_params()[0] as usize].value;
    let (x, y) = e.sketch.curve_point(0, t);
    let gap = (((b.0 - a.0) * (y - a.1) - (b.1 - a.1) * (x - a.0)) / (b.0 - a.0).hypot(b.1 - a.1)).abs();
    assert!(gap < 1e-9, "{gap}");
}

/// A held line pushed against the rope would meet it at a corner, so it cannot be tangent there:
/// refused where it is written, never solved to a wrong smooth answer.
#[test]
fn a_held_line_tangent_to_the_rope_is_refused() {
    let src = ROPE.replace(
        "  length(150) rope\n",
        "  length(150) rope\n  f0 := point\n  f1 := point\n  fix((20, -40)) f0\n  fix((90, -40)) f1\n  floor := line(f0, f1)\n  rope tangent floor\n",
    );
    let (_, d) = read(&src);
    assert!(d.iter().any(|m| m.starts_with("E040") && m.contains("corner")), "{d:?}");
}

/// **A free length**: with no length stated, the energy is stationary in the length too
/// (transversality, `H = 0`).  `∫ 2/(1 + |p|²) ds` is length on the sphere seen
/// stereographically, so the curve is a great circle — between two points 40° apart on the circle
/// of centre `(0, 2)` and radius `√5`, that arc of it, the short way round: a minimum.
#[test]
fn a_free_length_is_where_the_energy_is_stationary_in_it() {
    let (c, rho) = ((0.0, 2.0), 5f64.sqrt());
    let on = |phi: f64| (c.0 + rho * phi.cos(), c.1 + rho * phi.sin());
    let (pa, pb) = ((-2.0f64).atan2(-1.0), (-2.0f64).atan2(-1.0) + 40f64.to_radians());
    let (a, b) = (on(pa), on(pb));
    let src = format!(
        "use std\nin std.front {{\n  a := point\n  b := point\n  fix(({}, {})) a\n  fix(({}, {})) b\n  k := curve(a, b)\n}}\n\
         k minimizes integral(2 / (1 + p.x^2 + p.y^2) over p)\n",
        a.0, a.1, b.0, b.1
    );
    let mut e = solved(&src);
    let o = samples(&e.sketch, 0).into_iter().map(|(x, y)| ((x - c.0).hypot(y - c.1) - rho).abs()).fold(0.0, f64::max);
    assert!(o < 1e-9, "off the great circle by {o}");
    let len = e.sketch.params[e.sketch.curves[0].length.unwrap() as usize].value;
    assert!((len - rho * 40f64.to_radians()).abs() < 1e-9, "{len}");
    // the length's row, and its Jacobian the system's own derivative
    crate::common::fd_jacobian(&e.sketch, 1e-6);
    assert_eq!(dof(&mut e), 0);
    assert_eq!(verdicts(&mut e), ["minimum"]);
}

/// The contacts' and the length's rows are the system's own derivative: a free end, a free length
/// set by an unknown, a free line tangent to the rope and a point on it.
#[test]
fn the_rows_on_a_free_curve_differentiate_exactly() {
    let src = ROPE
        .replace("  fix((100, 0)) b\n", "")
        .replace("  b := point\n", "  b := point hint((100, 5))\n")
        .replace(
            "  length(150) rope\n",
            "  length(150) rope\n  f0 := point hint((20, -40))\n  f1 := point hint((90, -45))\n  floor := line(f0, f1)\n  rope tangent floor\n  q := point hint((30, -30))\n  q coincident rope\n",
        );
    let e = solved(&src);
    crate::common::fd_jacobian(&e.sketch, 1e-6);
}

/// `std.hangs(L) rope` is the longhand in one word: the same rope, the same verdict, and a rope
/// deleted takes the word's use with it.
#[test]
fn the_library_word_hangs_the_same_rope() {
    let longhand = solved(ROPE);
    let src = ROPE
        .replace("use std\n", "use std (hangs)\n")
        .replace("  length(150) rope\n", "  hangs(L: 150) rope\n")
        .replace("rope minimizes integral(p.y over p)\n", "");
    let mut e = solved(&src);
    for (p, q) in samples(&e.sketch, 0).into_iter().zip(samples(&longhand.sketch, 0)) {
        assert!((p.0 - q.0).hypot(p.1 - q.1) < 1e-9, "{p:?} against {q:?}");
    }
    assert_eq!(verdicts(&mut e), ["minimum"]);
    let out = gcs_core::edit::remove(&e, &e.program, &e.sketch, &[EntRef::new(EntKind::Curve, 0)], &[]);
    assert!(!out.text.contains("hangs(L: 150) rope"), "{}", out.text);
}

/// A free curve has nothing of its own to write: after a solve, a gesture's reconcile and a seed
/// writeback leave the source as it was.
#[test]
fn a_free_curve_writes_nothing_back() {
    let mut e = solved(ROPE);
    let sk = e.sketch.clone();
    let out = gcs_core::edit::reconcile(&mut e, &sk);
    assert_eq!(out.text, ROPE, "reconcile wrote {}", out.text);
    let seeds = gcs_core::edit::commit_seeds(&e, &e.sketch, &e.program);
    assert_eq!(seeds.text, ROPE, "a writeback wrote {}", seeds.text);
}

/// An end dragged is a seed like any other: its `hint(…)` is rewritten where it moved to, and
/// nothing else is written.
#[test]
fn a_dragged_end_writes_its_hint() {
    let src = ROPE.replace("  b := point\n", "  b := point hint((100, 0))\n").replace("  fix((100, 0)) b\n", "");
    let mut e = solved(&src);
    let b = b_of(&e);
    let mut drag = gcs_core::decompose::PlanDrag::new(&e.sketch, b, 100.0, 0.0, None, 0.05);
    let r = drag.move_to(&mut e.sketch, None, 90.0, -10.0);
    assert!(r.success, "{r:?}");
    drag.end();
    let seeds = gcs_core::edit::commit_seeds(&e, &e.sketch, &e.program);
    assert!(seeds.text.contains("b := point hint((90, -10))"), "{}", seeds.text);
    assert_eq!(seeds.text.matches("hint").count(), 1, "{}", seeds.text);
}

/// A copy of the drawing (a paste, a deletion's rebuild) carries the curve, its length and its
/// energy, and hangs the same rope.
#[test]
fn a_copied_rope_hangs_again() {
    let e = solved(ROPE);
    let all: Vec<EntRef> = (0..e.sketch.points.len())
        .map(EntRef::point)
        .chain((0..e.sketch.curves.len()).map(|i| EntRef::new(EntKind::Curve, i)))
        .collect();
    let mut copied = gcs_core::io::copy(&e.sketch, &all);
    let r = solve(&mut copied, SolveOpts::default());
    assert!(r.success, "{}", r.message);
    for (p, q) in samples(&copied, 0).into_iter().zip(samples(&e.sketch, 0)) {
        assert!((p.0 - q.0).hypot(p.1 - q.1) < 1e-9, "{p:?} against {q:?}");
    }
}



/// A rope shorter than the distance between its ends has no shape.  No row reads its shape — the
/// curve is a function of its ends and length, and here there is none — so the verdict says it:
/// the statement is answered `unsolved`, and the rope draws nothing.  Never a crash, never a shape
/// made up.
#[test]
fn a_rope_too_short_to_reach_is_unsolved() {
    let (mut e, d) = read(&ROPE.replace("length(150) rope", "length(90) rope"));
    assert!(e.ok(), "{d:?}");
    let _ = solve(&mut e.sketch, SolveOpts::default());
    assert_eq!(verdicts(&mut e), ["unsolved"]);
    assert!(e.sketch.curve_polyline(0).is_empty());
}


/// A line drawn tangent to the rope and the rope's free end are one figure, whichever is dragged:
/// dragging the end re-hangs the rope and carries the line with it, and dragging the line moves it
/// along the rope — every frame the line still touches the rope, and the rope is still a catenary.
#[test]
fn a_tangent_line_and_the_rope_drag_together_either_way() {
    let src = ROPE.replace("  fix((100, 0)) b\n", "").replace("  b := point\n", "  b := point hint((100, 0))\n").replace(
        "  length(150) rope\n",
        "  length(150) rope\n  f0 := point hint((20, -40))\n  f1 := point hint((90, -45))\n  floor := line(f0, f1)\n  rope tangent floor\n",
    );
    let touching = |e: &Elaborated| {
        let c = e.sketch.constraints.iter().find(|c| c.kind == gcs_core::constraints::CKind::CurveTangentLine).unwrap();
        let t = e.sketch.params[c.aux_params()[0] as usize].value;
        let (x, y) = e.sketch.curve_point(0, t);
        let l = &e.sketch.lines[0];
        let (a, b) = (e.sketch.point_xy(l.p1 as usize), e.sketch.point_xy(l.p2 as usize));
        (((b.0 - a.0) * (y - a.1) - (b.1 - a.1) * (x - a.0)) / (b.0 - a.0).hypot(b.1 - a.1)).abs()
    };
    let hangs = |e: &Elaborated| off(&e.sketch, 0, catenary((0.0, 0.0), e.sketch.point_xy(b_of(e)), 150.0));
    let mut e = solved(&src);
    // the rope's end, dragged
    let b = b_of(&e);
    let mut drag = gcs_core::decompose::PlanDrag::new(&e.sketch, b, 100.0, 0.0, None, 0.05);
    for k in 1..=5 {
        let r = drag.move_to(&mut e.sketch, None, 100.0 - 2.0 * k as f64, -2.0 * k as f64);
        assert!(r.success, "end, frame {k}: {r:?}");
        assert!(touching(&e) < 1e-6, "end, frame {k}: the line left the rope by {}", touching(&e));
        assert!(hangs(&e) < 1e-6, "end, frame {k}: off the catenary by {}", hangs(&e));
    }
    drag.end();
    // and the line's end
    let f1 = e.sketch.lines[0].p2 as usize;
    let (x, y) = e.sketch.point_xy(f1);
    let mut drag = gcs_core::decompose::PlanDrag::new(&e.sketch, f1, x, y, None, 0.05);
    for k in 1..=5 {
        let r = drag.move_to(&mut e.sketch, None, x + k as f64, y - k as f64);
        assert!(r.success, "line, frame {k}: {r:?}");
        assert!(touching(&e) < 1e-6, "line, frame {k}: the line left the rope by {}", touching(&e));
        assert!(hangs(&e) < 1e-6, "line, frame {k}: off the catenary by {}", hangs(&e));
    }
    drag.end();
}

/* -- a length the drawing determines ------------------------------------------------------- */

/// The rope through `(0, 0)` and `(100, 0)` hanging `sag` below them: its catenary's parameter
/// (`a·(cosh(50/a) − 1) = sag`), so the height at `x` and the length `2a·sinh(50/a)`.
fn sagging(sag: f64) -> (impl Fn(f64) -> f64, f64) {
    let a = bisect(1.0, 1e4, |a| a * ((50.0 / a).cosh() - 1.0) - sag);
    (move |x: f64| a * ((x - 50.0) / a).cosh() - a * (50.0 / a).cosh(), 2.0 * a * (50.0 / a).sinh())
}

const SAG: &str = "\
use std
in std.front {
  a := point
  b := point
  m := point hint((50, -30))
  fix((0, 0)) a
  fix((100, 0)) b
  a distance(50, along: right) m
  a distance(30, along: down) m
  rope := curve(a, b)
  m coincident rope
}
rope minimizes integral(p.y over p)
";

fn length_of(e: &Elaborated) -> f64 {
    e.sketch.params[e.sketch.curves[0].length.unwrap() as usize].value
}

/// **Span and sag**: nothing states the rope's length, but a point placed below the middle of the
/// span is on it — so the drawing determines the length, and it is solved for, not taken where
/// the energy is stationary in it (a hanging rope has no such length).
#[test]
fn a_rope_through_a_placed_point_takes_the_length_it_needs() {
    let mut e = solved(SAG);
    let (y, len) = sagging(30.0);
    let o = off(&e.sketch, 0, y);
    assert!(o < 1e-9, "off the catenary by {o}");
    assert!((length_of(&e) - len).abs() < 1e-9, "length {} against {len}", length_of(&e));
    assert_eq!(dof(&mut e), 0);
    assert_eq!(verdicts(&mut e), ["minimum"]);
}

/// The same with the far end let go: its two freedoms are the drawing's, and dragged, the rope
/// re-hangs through the placed point at whatever length that takes.
#[test]
fn a_dragged_end_keeps_the_sag_point() {
    let src = SAG.replace("  fix((100, 0)) b\n", "").replace("  b := point\n", "  b := point hint((100, 0))\n");
    let mut e = solved(&src);
    assert_eq!(dof(&mut e), 2);
    let b = b_of(&e);
    let m = e.sketch.constraints.iter().find(|c| c.kind == gcs_core::constraints::CKind::PointOnCurve).unwrap().clone();
    let mut drag = gcs_core::decompose::PlanDrag::new(&e.sketch, b, 100.0, 0.0, None, 0.05);
    for k in 1..=5 {
        let r = drag.move_to(&mut e.sketch, None, 100.0 + 2.0 * k as f64, 3.0 * k as f64);
        assert!(r.success, "frame {k}: {r:?}");
    }
    drag.end();
    let o = off(&e.sketch, 0, catenary((0.0, 0.0), e.sketch.point_xy(b), length_of(&e)));
    assert!(o < 1e-9, "off the catenary by {o}");
    let t = e.sketch.params[m.aux_params()[0] as usize].value;
    let (x, y) = e.sketch.curve_point(0, t);
    assert!((x - 50.0).hypot(y + 30.0) < 1e-9, "the rope left the sag point: ({x}, {y})");
}

/// **A deck the rope touches**: a held level line 30 below the ends, the rope tangent to it.  With
/// the length stated that would press the rope into a corner (refused); with nothing stating it,
/// the touch is what sets the length — the rope's lowest point on the deck.
#[test]
fn a_held_deck_the_rope_touches_sets_its_length() {
    let src = SAG.replace("  m := point hint((50, -30))\n", "").replace(
        "  a distance(50, along: right) m\n  a distance(30, along: down) m\n",
        "  f0 := point\n  f1 := point\n  fix((20, -30)) f0\n  fix((90, -30)) f1\n  deck := line(f0, f1)\n",
    ).replace("  m coincident rope\n", "  rope tangent deck\n");
    let mut e = solved(&src);
    let (y, len) = sagging(30.0);
    let o = off(&e.sketch, 0, y);
    assert!(o < 1e-9, "off the catenary by {o}");
    assert!((length_of(&e) - len).abs() < 1e-9, "length {} against {len}", length_of(&e));
    assert_eq!(verdicts(&mut e), ["minimum"]);
}

/// **The area under it**: `∫ y dx = ∫ y t.x ds`, the area between the chord and the curve, is
/// Dido's energy — so through the placed point, which sets its length, the curve is the arc of the
/// circle through the ends and that point: centre `(50, k)`, `50² + k² = (k + 30)²`.
#[test]
fn the_least_area_through_a_placed_point_is_the_circular_arc() {
    let e = solved(&SAG.replace("integral(p.y over p)", "integral(p.y * t.x over (p, t))"));
    let k = (2500.0 - 900.0) / 60.0;
    let o = samples(&e.sketch, 0).into_iter().map(|(x, y)| ((x - 50.0).hypot(y - k) - (k + 30.0)).abs()).fold(0.0, f64::max);
    assert!(o < 1e-9, "off the arc by {o}");
}

/// A free point drawn on a curve of free length moves with the curve and holds nothing: the
/// curve is still the great circle, its length still where the energy is stationary in it.
#[test]
fn a_free_point_on_a_free_length_curve_leaves_it_stationary() {
    let (c, rho) = ((0.0, 2.0), 5f64.sqrt());
    let on = |phi: f64| (c.0 + rho * phi.cos(), c.1 + rho * phi.sin());
    let pa = (-2.0f64).atan2(-1.0);
    let (a, b, q) = (on(pa), on(pa + 40f64.to_radians()), on(pa + 15f64.to_radians()));
    let src = format!(
        "use std\nin std.front {{\n  a := point\n  b := point\n  q := point hint(({}, {}))\n  fix(({}, {})) a\n  fix(({}, {})) b\n  \
         k := curve(a, b)\n  q coincident k\n}}\nk minimizes integral(2 / (1 + p.x^2 + p.y^2) over p)\n",
        q.0 + 0.01, q.1, a.0, a.1, b.0, b.1
    );
    let mut e = solved(&src);
    let o = samples(&e.sketch, 0).into_iter().map(|(x, y)| ((x - c.0).hypot(y - c.1) - rho).abs()).fold(0.0, f64::max);
    assert!(o < 1e-9, "off the great circle by {o}");
    assert!((length_of(&e) - rho * 40f64.to_radians()).abs() < 1e-9, "{}", length_of(&e));
    // the point slides along it
    assert_eq!(dof(&mut e), 1);
}

/// A rope with nothing holding its length and no length its energy is stationary in — a hanging
/// rope only lowers its energy as it lengthens — is said so where its energy is stated, and no
/// row sends the solve after the length that is not there: the length is left a freedom, the
/// rope hangs at the one it has, and the solve and the diagnosis come back at once (they once ran
/// on, the rope ever longer).
#[test]
fn a_rope_with_no_length_to_settle_on_is_said_so() {
    let (mut e, d) = read(&ROPE.replace("  length(150) rope\n", ""));
    assert!(d.iter().any(|m| m.starts_with("W114") && m.contains("nothing holds")), "{d:?}");
    let r = solve(&mut e.sketch, SolveOpts::default());
    assert!(r.success, "{}", r.message);
    assert_eq!(dof(&mut e), 1);
    // and the example with its length deleted, its far end free, as the app is left
    let (mut e, _) = read(&include_str!("../../examples/catenary.sv").replace("  length(150mm) rope\n", ""));
    let r = solve(&mut e.sketch, SolveOpts::default());
    assert!(r.success, "{}", r.message);
    assert_eq!(dof(&mut e), 3);
}



/* -- the library's energies ------------------------------------------------------------------ */

/// A library document solved, with its curve's ends.
fn library(key: &str) -> (Elaborated, (f64, f64), (f64, f64)) {
    let mut e = solved(gcs_core::examples::source(key).unwrap());
    let (a, b) = (e.sketch.curves[0].args[0].i(), e.sketch.curves[0].args[1].i());
    assert_eq!(verdicts(&mut e), ["minimum"], "{key}");
    let (a, b) = (e.sketch.point_xy(a), e.sketch.point_xy(b));
    (e, a, b)
}

/// `sag.sv`: no length stated, the low point placed — the catenary of that sag, and its length.
#[test]
fn the_library_sag_is_the_catenary_of_its_sag() {
    let (e, ..) = library("sag");
    let (y, len) = sagging(30.0);
    let o = off(&e.sketch, 0, y);
    assert!(o < 1e-9, "off the catenary by {o}");
    assert!((length_of(&e) - len).abs() < 1e-9, "{}", length_of(&e));
}

/// `fisheye.sv`: a ray in Maxwell's fish-eye, `n = 2 / (1 + r² / R²)`, is the circle through its
/// ends with `|c|² + R² = ρ²` — linear in the centre, `p·p − 2p·c = R²` at each end.
#[test]
fn the_library_fisheye_ray_is_a_circle() {
    let (e, a, b) = library("fisheye");
    let r2 = 50.0f64 * 50.0;
    let (m11, m12, m21, m22) = (2.0 * a.0, 2.0 * a.1, 2.0 * b.0, 2.0 * b.1);
    let (k1, k2) = (a.0 * a.0 + a.1 * a.1 - r2, b.0 * b.0 + b.1 * b.1 - r2);
    let det = m11 * m22 - m12 * m21;
    let c = ((k1 * m22 - m12 * k2) / det, (m11 * k2 - m21 * k1) / det);
    let rho = (r2 + c.0 * c.0 + c.1 * c.1).sqrt();
    let o = samples(&e.sketch, 0).into_iter().map(|(x, y)| ((x - c.0).hypot(y - c.1) - rho).abs()).fold(0.0, f64::max);
    assert!(o < 1e-9, "off the circle by {o}");
}

/// `projectile.sv`: speed `√(2g(h − y))` from `a`, so the range is `2h sin 2α`; 80 of a possible
/// 100 is `sin 2α = 0.8`, and the line drive, the lower angle, is the minimum.
#[test]
fn the_library_throw_is_the_line_drive_parabola() {
    let (e, ..) = library("projectile");
    let al = 0.5 * 0.8f64.asin();
    let o = off(&e.sketch, 0, |x| x * al.tan() - x * x / (4.0 * 50.0 * al.cos().powi(2)));
    assert!(o < 1e-9, "off the parabola by {o}");
}

/// `hyperbolic.sv`: in Poincaré's half-plane the shortest path is the circle through the ends
/// centred on `y = 0`.
#[test]
fn the_library_hyperbolic_path_is_a_semicircle_arc() {
    let (e, a, b) = library("hyperbolic");
    let x0 = (b.0 * b.0 + b.1 * b.1 - a.0 * a.0 - a.1 * a.1) / (2.0 * (b.0 - a.0));
    let rho = (a.0 - x0).hypot(a.1);
    let o = samples(&e.sketch, 0).into_iter().map(|(x, y)| ((x - x0).hypot(y) - rho).abs()).fold(0.0, f64::max);
    assert!(o < 1e-9, "off the arc by {o}");
}
