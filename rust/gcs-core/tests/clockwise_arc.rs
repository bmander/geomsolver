//! **An arc said to turn clockwise is the counter-clockwise arc from its end** (#573): `arc(center:
//! k, start: a, end: b, sense: cw)` drawn in a plane is drawn from `b`, in its own plane, so it
//! owns what `arc(center: k, start: b, end: a)` owns and nothing more — no plane of its own, with
//! an attitude and an origin the diagnosis would blame for the drawing's freedom — and a ring's
//! copy of it is a copy like any other.  What reads an end or a fraction of its turn reads it as
//! written: `bow.start` is `a`, `tangent(at: start)` touches at `a`, `t == 0.25` is a quarter of
//! the way clockwise from `a`, and a copy, the JSON and the lifted text keep it so.  Held against
//! the same arc written counter-clockwise from its other end, the issue's workaround.

use crate::common::at_path as ent;
use crate::common::parsed::read;
use gcs_core::diagnose::{diagnose, DiagnoseOptions, Diagnosis};
use gcs_core::model::Sketch;
use gcs_core::program::Elaborated;
use gcs_core::solve::{solve, SolveOpts};

/// The issue's arc: centre `k` 20 out along `u`, ends `a` below and `b` above 17 from the hub,
/// `BOW` the arc and `MORE` what else is stated.
const ONE: &str = "unit mm
use std
in std.top {
  hub := point
  hub coincident std.top.origin
  k := point hint((20, 0))
  a := point hint((17, -3))
  b := point hint((17, 3))
  hub distance(20mm, along: u) k
  hub level(v) k
  hub distance(17mm) a
  hub distance(17mm) b
  bow := BOW
  MORE
}
";

/// The same turned four times about the hub.
const RING: &str = "unit mm
use std
in std.top {
  hub := point
  hub coincident std.top.origin
  ring 4 about hub {
    k := point hint((20, 0))
    a := point hint((17, -3))
    b := point hint((17, 3))
    hub distance(20mm) k
    hub distance(17mm) a
    hub distance(17mm) b
    bow := BOW
    MORE
  }
  hub level(v) k[0]
}
";

/// Each drawing and what its arc is called there: the ring's representative is copy 0.
const BOTH: [(&str, &str); 2] = [(ONE, "bow"), (RING, "bow[0]")];

const CW: &str = "arc(center: k, start: a, end: b, sense: cw)";
const CCW: &str = "arc(center: k, start: b, end: a)";

fn doc(frame: &str, bow: &str, more: &str) -> String {
    frame.replace("BOW", bow).replace("MORE", more)
}

/// What reads the arc by an end and by its turn: a line from `a` touching it `at`, and a point
/// `t` of the way round.
fn touched(at: &str, t: &str) -> String {
    format!("radius(5mm) bow
  q := point hint((22, -9))
  l := line(a, q)
  bow tangent(at: {at}) l
  length(8mm) l
  m := point hint((16, -2))
  m coincident(t == {t}) bow")
}

fn solved(src: &str) -> (Elaborated, Diagnosis) {
    let mut e = read(src);
    let r = solve(&mut e.sketch, SolveOpts::default());
    assert!(r.success, "{r:?}\n{src}");
    let d = diagnose(&mut e.sketch, DiagnoseOptions::default());
    (e, d)
}

fn at(e: &Elaborated, name: &str) -> [f64; 3] {
    e.sketch.world_point(ent(e, name).i())
}

fn near(p: [f64; 3], q: [f64; 3], what: &str) {
    assert!((0..3).all(|k| (p[k] - q[k]).abs() <= 1e-9), "{what}: {p:?} against {q:?}");
}

/// The numbers the solve is over, the rows, the planes and the freedom: the counts the report
/// opens with.
fn counts(e: &Elaborated, d: &Diagnosis) -> (usize, usize, usize, i64) {
    (d.n_params, d.n_equations, e.sketch.frames.len(), d.dof)
}

/// **`sense: cw` costs nothing** (#573): the arc and the ring of four are solved over the 9
/// unknowns the counter-clockwise arc from the other end is — not 45 and 153, a plane minted for
/// each arc and each copy — and with no plane more.
#[test]
fn a_clockwise_arc_owns_what_the_arc_it_describes_owns() {
    for (frame, bow) in BOTH {
        let (cw, dc) = solved(&doc(frame, CW, "radius(5mm) bow"));
        let (ccw, dd) = solved(&doc(frame, CCW, "radius(5mm) bow"));
        assert_eq!(counts(&cw, &dc), counts(&ccw, &dd));
        assert_eq!((dc.n_params, dc.dof), (9, 0), "{dc:?}");
        // and it is the same arc: its ends where the other's are, the other way round
        let arc = |e: &Elaborated| e.sketch.arcs[ent(e, bow).i()].clone();
        let (p, q) = (arc(&cw), arc(&ccw));
        for (x, y) in [(p.start, q.start), (p.end, q.end), (p.center, q.center)] {
            near(cw.sketch.world_point(x as usize), ccw.sketch.world_point(y as usize), "an end");
        }
    }
}

/// **A freedom the drawing leaves is the arc's** (#573): with its radius unstated, the one freedom
/// left is the arc's radius, as it is the counter-clockwise arc's — in the ring too, where every
/// copy's radius is the representative's.
#[test]
fn a_free_radius_is_the_arcs_own() {
    for (frame, bow) in BOTH {
        let (cw, dc) = solved(&doc(frame, CW, ""));
        let (ccw, dd) = solved(&doc(frame, CCW, ""));
        assert_eq!(counts(&cw, &dc), counts(&ccw, &dd));
        assert_eq!(dc.dof, 1, "{dc:?}");
        let r = cw.sketch.round_radius(ent(&cw, bow)) as u32;
        assert!(dc.under_params.contains(&r), "{:?}", dc.under_params);
        let free = |sk: &Sketch, d: &Diagnosis| {
            let mut names: Vec<&str> =
                d.under_params.iter().map(|&p| sk.params[p as usize].name.as_str()).collect();
            names.sort();
            names.join(", ")
        };
        assert_eq!(free(&cw.sketch, &dc), free(&ccw.sketch, &dd));
    }
}

/// **It reads as written** (#573): `bow.start` is `a` and `bow.end` is `b` — named, claimed and
/// in the table of positions — a line touching at its start touches at `a`, and a quarter of its
/// turn is a quarter of the way clockwise from `a`: the counter-clockwise arc from `b` touched at
/// its end, three quarters round.
#[test]
fn a_clockwise_arc_reads_its_ends_and_its_turn_as_written() {
    let more = |at: &str, t: &str, (to_a, to_b): (&str, &str)| format!("{}
  claim bow.{to_a} coincident a
  claim bow.{to_b} coincident b", touched(at, t));
    let (cw, dc) = solved(&doc(ONE, CW, &more("start", "0.25", ("start", "end"))));
    assert!(dc.claims_violated.is_empty() && dc.claims_theorem.len() == 2, "{dc:?}");
    assert_eq!(dc.dof, 0, "{dc:?}");
    let (ccw, dd) = solved(&doc(ONE, CCW, &more("end", "0.75", ("end", "start"))));
    assert!(dd.claims_violated.is_empty() && dd.claims_theorem.len() == 2, "{dd:?}");
    for p in ["q", "m", "a", "b"] {
        near(at(&cw, p), at(&ccw, p), p);
    }
    // the quarter is below the centre's level, where `a` is, not above it by `b`
    assert!(at(&cw, "m")[1] < 0.0, "{:?}", at(&cw, "m"));
    let positions = gcs_core::report::positions(&cw.sketch, &cw.map);
    let listed = |n: String| positions.iter().find(|(k, _)| *k == n).map(|(_, v)| *v).unwrap();
    for (end, p) in [("start", "a"), ("end", "b")] {
        let w = at(&cw, p);
        let got = [listed(format!("bow.{end}.x")), listed(format!("bow.{end}.y")), w[2]];
        near(got, w, &format!("bow.{end}"));
    }
}

/// **A copy, the JSON and the lifted text keep it as written** (#573): each reads the fraction
/// and the end from `a`, so each holds where the drawing was solved, and the text says `sense:
/// cw` from `a` to `b` again.
#[test]
fn a_clockwise_arc_is_kept_as_written() {
    use gcs_core::{graft, io};
    let (e, _) = solved(&doc(ONE, CW, &touched("start", "0.25")));
    let sk = &e.sketch;
    assert_eq!(sk.reversed.len(), 1);
    // read the other way round, the end and the quarter would be off where the drawing stands
    for back in [io::from_json(&io::to_json(sk)).expect("reads back"),
                 graft::copy(sk, &sk.primitives())] {
        assert_eq!(back.reversed.len(), 1);
        let mut sys = gcs_core::system::System::new(&back);
        let z = sys.z0(&back);
        assert!(sys.max_hard_residual(&z) <= 1e-9 * sys.scale);
    }
    let mut lifted = gcs_core::program::to_program(sk);
    let text = gcs_core::syntax::render_flat(&mut lifted).unwrap().to_string();
    assert!(text.contains("arc(center: p1, start: p2, end: p3, sense: cw)"), "{text}");
    let mut again = read(&text);
    assert!(solve(&mut again.sketch, SolveOpts::default()).success, "{text}");
    // lifted, the points go by the names a program gives them, in the order they were made
    for (p, lifted) in [("a", "p2"), ("b", "p3"), ("q", "p4"), ("m", "p5")] {
        near(at(&again, lifted), at(&e, p), p);
    }
}

/// **A chain entering it at its start still turns right there** (#363 S4): such an arc is drawn
/// from its start in its plane turned over, not from its end — and a clockwise half turn is
/// refused either way it is drawn.
#[test]
fn a_clockwise_arc_a_chain_enters_at_its_start_is_drawn_from_there() {
    let (e, _) = solved(&doc(ONE, CW, "radius(5mm) bow
  p := point hint((10, -10))
  route := (l := line(p, a)) -> bow"));
    assert!(e.sketch.reversed.is_empty());
    for more in ["", "route := (l := line(p, q0)) -> k"] {
        let said: Vec<String> = crate::common::parsed::build(&format!("unit mm
use std
in std.top {{
  p := point hint((-5, -5))
  q0 := point hint((0, 0))
  q1 := point hint((5, 0))
  q2 := point hint((10, 0))
  k := arc(center: q1, start: q0, end: q2, sense: cw)
  {more}
}}
")).errors().map(|d| format!("{}: {}", d.code.as_str(), d.message)).collect();
        assert!(said.iter().any(|m| m.starts_with("E060") && m.contains("a half turn")),
                "{more}: {said:?}");
    }
}
