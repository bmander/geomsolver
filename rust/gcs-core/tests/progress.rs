//! **What a long run is doing** (#320): the solids a diagnosis builds and the claims it judges,
//! said to whoever listens (`gcs_core::progress`), nested as they ran, and timed.

use std::cell::RefCell;
use std::rc::Rc;

use gcs_core::progress::{self, At, Kind, Listener, Timings, Work};
use crate::common::parsed::read;

/// Everything said, in order: `Some(work)` begun, `None` the latest open one ended, and the
/// times each end was given.
#[derive(Default)]
struct Heard {
    said: RefCell<Vec<Option<Work>>>,
    times: RefCell<Vec<(Work, f64, f64)>>,
}

impl Listener for Heard {
    fn begin(&self, work: &Work) {
        self.said.borrow_mut().push(Some(work.clone()));
    }
    fn end(&self, work: &Work, total: f64, own: f64) {
        self.said.borrow_mut().push(None);
        self.times.borrow_mut().push((work.clone(), total, own));
    }
}

/// An arm on a slide and a wall it must clear: one claim over the slide's travel and one at the
/// pose the drawing stands in.
const SLIDE: &str = "\
unit mm
use std (horizontal, vertical)
in std.front {
o := point
fix((0, 0)) o
p := point hint((10, 0))
}
param reach: Length
in std.front {
o distance(reach, along: u) p
o horizontal p
q := point hint((14, 0))
p distance(4, along: u) q
p horizontal q
r := point hint((14, 4))
q vertical r
q distance(4, along: v) r
s := point hint((10, 4))
p vertical s
p distance(4, along: v) s
(e0 := line(p, q)) -> (e1 := line(q, r)) -> (e2 := line(r, s)) -> (e3 := line(s, p)) -> close
arm_f := face(e0, e1, e2, e3)
arm := solid(arm_f, depth: 3mm)
w0 := point
w1 := point hint((56, 0))
w2 := point hint((56, 4))
w3 := point hint((50, 4))
(g0 := line(w0, w1)) -> (g1 := line(w1, w2)) -> (g2 := line(w2, w3)) -> (g3 := line(w3, w0)) -> close
fix((50, 0)) w0
horizontal g0
vertical g1
w0 distance(6) w1
w1 distance(4) w2
horizontal g2
vertical g3
wall_f := face(g0, g1, g2, g3)
wall := solid(wall_f, depth: 3mm)
claim over reach in (10mm, 40mm) { arm clear(1mm) wall }
claim arm clear(1mm) wall
}
";

fn solved() -> gcs_core::model::Sketch {
    let mut sk = read(SLIDE).sketch;
    assert!(gcs_core::solve::solve(&mut sk, Default::default()).success);
    sk
}

#[test]
fn a_judgement_is_said_with_the_solids_it_built_inside_it() {
    let sk = solved();
    let heard = Rc::new(Heard::default());
    let verdicts = progress::listen(heard.clone(), || gcs_core::diagnose::judge_solids(&sk));
    assert!(!progress::listening(), "the listener goes when its work does");
    let said = heard.said.borrow();
    // every begin is ended, innermost first, and nothing is ended that was not begun; the claims
    // are the outermost work
    let (mut depth, mut claims) = (0usize, Vec::new());
    for s in said.iter() {
        match s {
            Some(w) => {
                if depth == 0 { claims.push(w.clone()); }
                depth += 1;
            }
            None => depth = depth.checked_sub(1).expect("an end closes a begin"),
        }
    }
    assert_eq!(depth, 0, "every begin is ended");
    // in the order written, worded as the report words them
    assert_eq!(claims.len(), 2, "{claims:?}");
    for (w, v) in claims.iter().zip(&verdicts) {
        assert_eq!(w.kind, Kind::Claim);
        assert_eq!(w.subject, v.text());
        assert_eq!(w.at, Some(At::Stmt(v.stmt())));
    }
    assert_eq!(claims[1].text(), "judged claim arm clear(1mm) wall");
    // and inside them, each solid evaluated by its own name and statement
    let arm = sk.solids.iter().position(|s| s.name == "arm").unwrap();
    assert!(said.iter().flatten().any(|w| w.kind == Kind::Evaluate && w.subject == "arm"
        && w.at == Some(At::Solid(arm))), "the arm evaluated: {said:?}");
    // a piece of work's own time is its time less what it started
    for (w, total, own) in heard.times.borrow().iter() {
        assert!(*own >= 0. && *own <= *total + 1e-12, "{w:?}: {own} of {total}");
    }
}

#[test]
fn work_nobody_listens_to_is_only_done() {
    let made = std::cell::Cell::new(false);
    let r = progress::during(|| { made.set(true); Work::elaborate("x.sv") }, || 7);
    assert_eq!(r, 7);
    assert!(!made.get(), "with nobody listening the work is not even named");
}

#[test]
fn timings_count_work_done_again_and_rank_by_own_time() {
    let sk = solved();
    let heard = Rc::new(Heard::default());
    progress::listen(heard.clone(), || gcs_core::diagnose::judge_solids(&sk));
    let mut timings = Timings::default();
    let mut most = 0;
    for (w, total, own) in heard.times.borrow().iter() {
        most = most.max(timings.add(w, *total, *own));
    }
    // the swept claim re-places the arm at each of its poses: one row, counted
    assert!(most > 1, "work done again is counted on its row");
    let lines = timings.lines(usize::MAX);
    assert!(lines.iter().any(|(l, _)| l.contains("judged claim over reach")), "{lines:?}");
    assert!(lines.iter().any(|(l, _)| l.contains(" (×")), "{lines:?}");
    assert!(timings.heading().starts_with("solids and claims, slowest first"));
}

#[test]
fn the_words_are_the_cores() {
    let w = Work { kind: Kind::Build, subject: "stator.lamination".into(), at: None,
                  gauge: Default::default() };
    assert_eq!(w.done(2.1, 2.1), "built stator.lamination in 2.1 s");
    assert_eq!(w.done(2.0, 0.5), "built stator.lamination in 2.0 s (0.5 s its own)");
    assert_eq!(w.running(10.), "still building stator.lamination, 10.0 s so far");
    assert_eq!(progress::seconds(0.0123), "12 ms");
    let sk = solved();
    assert_eq!(Work::solve(&sk).text(), format!("solved for {} params", sk.free_indices().len()));
}

/// **A march is said, and how far it has got** (#591): a drawing followed along its parameter is
/// work like any stage, so a long one is told from a hang, and its gauge says where it stands.
#[test]
fn a_march_is_said_with_how_far_it_has_got() {
    let src = "unit mm\nuse std\nparam t: Angle over (0deg, 90deg)\nin std.front {\n  \
        a := point\n  d := point\n  b := point hint((10, 0))\n  c := point hint((40, 25))\n  \
        fix((0, 0)) a\n  fix((40, 0)) d\n  a distance(10) b\n  b distance(35) c\n  \
        c distance(25) d\n  line(a, d) angle(t) line(a, b)\n}\n";
    let mut sk = read(src).sketch;
    assert!(gcs_core::solve::solve(&mut sk, Default::default()).success);
    let heard = Rc::new(Heard::default());
    let f = progress::listen(heard.clone(), || gcs_core::follow::followed(&sk, "t"));
    assert!(f.is_ok());
    let said = heard.said.borrow();
    let w = said.iter().flatten().find(|w| w.kind == Kind::Follow).expect("said");
    assert_eq!(w.text(), "followed t");
    assert_eq!(w.gauge.read().as_deref(), Some("now at 90deg of (0deg, 90deg)"));
    assert_eq!(w.running(3.1), "still following t, now at 90deg of (0deg, 90deg), 3.1 s so far");
}
