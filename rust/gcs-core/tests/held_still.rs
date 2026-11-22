//! **A solid built once per state of the drawing, whoever asks** (#994): a body rounded all over
//! is a region for each edge and corner, each lowered and gated against its field — the dear part
//! of the round — and every asker (the elaboration's passes, the reading where the drawing solves,
//! the refusals, the report's evaluation) reads the one made, so a run makes each once; an edit
//! makes again only what reads what it moved.  Held by counting what the core says it made
//! (`progress::Made`), never by the clock.
use std::cell::RefCell;
use std::collections::BTreeMap;
use std::rc::Rc;

use gcs_core::model::Sketch;
use gcs_core::progress::{self, Listener, Made, Work};
use gcs_core::solid::ApproximationPolicy;
use gcs_core::solve::{solve, SolveOpts};

use crate::common::lenient::read;

/// What was made of each solid, by what and the solid's name, and how many times.
#[derive(Default)]
struct Tally(RefCell<BTreeMap<(Made, String), usize>>);

impl Listener for Tally {
    fn begin(&self, _: &Work) {}
    fn end(&self, _: &Work, _: f64, _: f64) {}
    fn made(&self, what: Made, subject: &str) {
        *self.0.borrow_mut().entry((what, subject.to_string())).or_default() += 1;
    }
}

impl Tally {
    /// Each solid whose name `which` takes, and how many times `what` was made of it.
    fn of(&self, what: Made, which: impl Fn(&str) -> bool) -> BTreeMap<String, usize> {
        self.0.borrow().iter().filter(|((m, s), _)| *m == what && which(s))
            .map(|((_, s), n)| (s.clone(), *n)).collect()
    }
}

/// A plate as `whole` with `pockets` square pockets cut in its top, each pocket's own edges
/// rounded, then the whole rounded over as `all` — the documents the issue timed.
fn pocketed(pockets: usize) -> String {
    let mut s = String::from("unit mm\nuse std\nuse std.solids (cut, union)\n\
                              use std.rounds (round)\nin std.top {\n");
    let w = 10 * pockets + 4;
    s += &format!("a := point\nb := point\nc := point\nd := point\nfix((0, 0)) a\n\
                   fix(({w}, 0)) b\nfix(({w}, 14)) c\nfix((0, 14)) d\n");
    for k in 0..pockets {
        let x = 4 + 10 * k;
        s += &format!("q{k}a := point\nq{k}b := point\nq{k}c := point\nq{k}d := point\n\
                       fix(({x}, 4)) q{k}a\nfix(({}, 4)) q{k}b\nfix(({}, 10)) q{k}c\n\
                       fix(({x}, 10)) q{k}d\n", x + 6, x + 6);
    }
    s += "}\nplate := solid(std.solids.Prism(face(a, b, c, d, -> close), from: 0mm, to: 10mm))\n\
          whole := solid(plate)\n";
    for k in 0..pockets {
        s += &format!("pk{k} := solid(std.solids.Prism(face(q{k}a, q{k}b, q{k}c, q{k}d, -> close), \
                       from: 6mm, to: 11mm))\npk{k} cut whole\n");
    }
    for k in 0..pockets {
        s += &format!("p{k} := round(r: 0.5mm) whole.pk{k}\n");
    }
    s + "all := solid(whole)\nr := round(r: 0.5mm) all\n"
}

/// `sk`'s solid called `name`, evaluated as a report reads it: its volume.
fn volume(sk: &Sketch, name: &str) -> f64 {
    let i = sk.solid_called(name).unwrap();
    sk.evaluated_solid(i, ApproximationPolicy::Report).unwrap().volume()
}

/// A run of `src` as `solventc --where all.volume` makes it: elaborated, solved on a copy, its
/// refusals read, the report's solid evaluated — what each made, and the volume.
fn run(src: &str) -> (Tally, f64) {
    let tally = Rc::new(Tally::default());
    let v = progress::listen(tally.clone(), || {
        let (e, d) = read(src);
        assert!(e.ok(), "{d:?}");
        let mut sk = e.sketch.clone();
        assert!(solve(&mut sk, SolveOpts::default()).settled());
        let said = gcs_core::program::solid_diagnostics(&sk, &e.map, &e.program);
        assert!(said.is_empty(), "{said:?}");
        volume(&sk, "all")
    });
    (Rc::try_unwrap(tally).ok().unwrap(), v)
}

#[test]
fn every_region_of_a_body_rounded_all_over_is_lowered_once_a_run() {
    let (tally, v) = run(&pocketed(2));
    // the plate less two pockets, each pocket's edges and then every edge rounded: material less
    // than the sharp body's, and more than nothing
    let sharp = (24.0 * 14.0 * 10.0) - 2.0 * (6.0 * 6.0 * 4.0);
    assert!(v > 0.9 * sharp && v < sharp, "{v} against {sharp}");
    let lowered = tally.of(Made::Lowered, |_| true);
    // the pockets' rounds and the round over the whole both lower their regions …
    assert!(lowered.keys().any(|s| s.starts_with("p0")), "{lowered:?}");
    assert!(lowered.keys().any(|s| s.starts_with("r")), "{lowered:?}");
    // … each once: the elaboration's three passes, the reading where it solves, the refusals and
    // the report all read the one lowering (they made the pockets' four times, #994)
    let again: Vec<_> = lowered.iter().filter(|(_, &n)| n > 1).collect();
    assert!(again.is_empty(), "lowered again: {again:?}");
    // and the body whose faces both rounds read is built once a state of the drawing: in the
    // first pass, before its pockets' rounds are expanded, and once they are — not again for the
    // pass that runs the last round over its faces, nor where the drawing solves
    let built = tally.of(Made::Built, |s| s == "whole");
    assert!(built.get("whole").is_some_and(|&n| n <= 2), "{built:?}");
}

/// Two plates, `one` and `two`, each with a pocket, each rounded all over.
fn two_plates() -> String {
    let mut s = String::from("unit mm\nuse std\nuse std.solids (cut, union)\n\
                              use std.rounds (round)\nin std.top {\n");
    for (k, x) in [(1, 0), (2, 40)] {
        s += &format!("a{k} := point\nb{k} := point\nc{k} := point\nd{k} := point\n\
            fix(({x}, 0)) a{k}\nfix(({}, 0)) b{k}\nfix(({}, 14)) c{k}\nfix(({x}, 14)) d{k}\n\
            e{k} := point\nf{k} := point\ng{k} := point\nh{k} := point\n\
            fix(({}, 4)) e{k}\nfix(({}, 4)) f{k}\nfix(({}, 10)) g{k}\nfix(({}, 10)) h{k}\n",
            x + 14, x + 14, x + 4, x + 10, x + 10, x + 4);
    }
    s += "}\n";
    for (k, name) in [(1, "one"), (2, "two")] {
        s += &format!("slab{k} := solid(std.solids.Prism(face(a{k}, b{k}, c{k}, d{k}, -> close), \
                       from: 0mm, to: 10mm))\n{name} := solid(slab{k})\n\
                       hole{k} := solid(std.solids.Prism(face(e{k}, f{k}, g{k}, h{k}, -> close), \
                       from: 6mm, to: 11mm))\nhole{k} cut {name}\n");
    }
    s + "r1 := round(r: 0.5mm) one\nr2 := round(r: 0.5mm) two\n"
}

#[test]
fn an_edit_lowers_again_only_the_regions_that_read_what_it_moved() {
    let (e, d) = read(&two_plates());
    assert!(e.ok(), "{d:?}");
    let mut sk = e.sketch.clone();
    assert!(solve(&mut sk, SolveOpts::default()).settled());
    let before = [volume(&sk, "one"), volume(&sk, "two")];
    // the second plate's pocket made a millimetre wider: a corner of it moved
    let g2 = e.map.ent_named("g2").unwrap();
    let x = sk.own_params(g2)[0] as usize;
    sk.params[x].value += 1.0;
    let tally = Rc::new(Tally::default());
    let after = progress::listen(tally.clone(), || [volume(&sk, "one"), volume(&sk, "two")]);
    // the first plate is the solid it was, read from what was made; the second is made again
    assert_eq!(after[0], before[0]);
    assert!(after[1] < before[1], "{after:?} against {before:?}");
    let lowered = tally.of(Made::Lowered, |_| true);
    assert!(lowered.keys().any(|s| s.starts_with("r2")), "{lowered:?}");
    assert!(lowered.keys().all(|s| s.starts_with("r2")), "the first plate's again: {lowered:?}");
    let built = tally.of(Made::Built, |_| true);
    assert!(!built.contains_key("one") && built.contains_key("two"), "{built:?}");
}
