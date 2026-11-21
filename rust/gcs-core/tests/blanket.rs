//! **A round over a whole body** (#994): `round(r: …) X` copies the body's edges and corners —
//! a block over its faces copies only the faces meeting the one its `when` asks of — so what it
//! makes, and what it costs, grows with the edges, not with every pair and three of the faces;
//! what it runs over is said as it starts, and an expansion past the limit says what it was
//! expanding and what to do.
use std::cell::RefCell;
use std::rc::Rc;

use gcs_core::progress::{self, Kind, Listener, Work};

use crate::common::lenient::read;

/// A prism 10 thick whose outline is a regular polygon of `n` sides, radius 50, as `whole`, and
/// `round` after it.
fn polygon(n: usize, round: &str) -> String {
    let mut s = String::from("unit mm\nuse std\nuse std.solids (cut, union)\nuse std.rounds (round)\n\
                              in std.top {\n");
    for k in 0..n {
        let t = std::f64::consts::TAU * k as f64 / n as f64;
        s += &format!("v{k} := point\nfix(({:.9}, {:.9})) v{k}\n", 50.0 * t.cos(), 50.0 * t.sin());
    }
    let vs: Vec<String> = (0..n).map(|k| format!("v{k}")).collect();
    s += &format!("}}\nplate := solid(std.solids.Prism(face({}, -> close), from: 0mm, to: 10mm))\n\
                   whole := solid(plate)\n", vs.join(", "));
    s + round
}

/// A plate 10 thick on the top plane as `whole`, `pockets` square pockets 6 wide and 4 deep cut
/// from it on a grid 10 apart, and `round` after it.
fn pocketed(pockets: usize, round: &str) -> String {
    let cols = (pockets as f64).sqrt().ceil().max(1.0) as usize;
    let rows = pockets.div_ceil(cols).max(1);
    let (w, h) = (10 * cols + 4, 10 * rows + 4);
    let mut s = format!("unit mm\nuse std\nuse std.solids (cut, union)\nuse std.rounds (round)\n\
        in std.top {{\na := point\nb := point\nc := point\nd := point\nfix((0, 0)) a\n\
        fix(({w}, 0)) b\nfix(({w}, {h})) c\nfix((0, {h})) d\n");
    for k in 0..pockets {
        let (x, y) = (4 + 10 * (k % cols), 4 + 10 * (k / cols));
        s += &format!("q{k}a := point\nq{k}b := point\nq{k}c := point\nq{k}d := point\n\
            fix(({x}, {y})) q{k}a\nfix(({}, {y})) q{k}b\nfix(({}, {})) q{k}c\nfix(({x}, {})) q{k}d\n",
            x + 6, x + 6, y + 6, y + 6);
    }
    s += "}\nplate := solid(std.solids.Prism(face(a, b, c, d, -> close), from: 0mm, to: 10mm))\n\
          whole := solid(plate)\n";
    for k in 0..pockets {
        s += &format!("pk{k} := solid(std.solids.Prism(face(q{k}a, q{k}b, q{k}c, q{k}d, -> close), \
                       from: 6mm, to: 11mm))\npk{k} cut whole\n");
    }
    s + round
}

/// Each piece of work said while a document was read, as the core words it when done.
#[derive(Default)]
struct Heard(RefCell<Vec<Work>>);

impl Listener for Heard {
    fn begin(&self, _: &Work) {}
    fn end(&self, work: &Work, _: f64, _: f64) {
        self.0.borrow_mut().push(work.clone());
    }
}

/// `src` read with a listener: what it says it expanded, and how many solids it made.
fn expanded(src: &str) -> (Vec<String>, usize) {
    let heard = Rc::new(Heard::default());
    let (e, d) = progress::listen(heard.clone(), || read(src));
    assert!(d.is_empty(), "{d:?}");
    let said = heard.0.borrow().iter().filter(|w| w.kind == Kind::Expand).map(Work::text).collect();
    (said, e.sketch.solids.len())
}

/// **What a round runs over is said** as its first block over faces is expanded: the faces, and
/// how many pairs and threes of them meet (each two of the three: a corner's faces, or a
/// triangle's three sides) — a prism of `n` sides, four or more, has `3n` edges and `2n` corners,
/// and so many pairs and threes meet; a plate of `k` pockets has `12 + 12k` and `8 + 8k`, each
/// pocket's top listed too, though the cut took it in (it meets nothing).
#[test]
fn a_round_over_a_body_says_how_many_of_its_faces_meet() {
    for n in [4, 7] {
        let (said, _) = expanded(&polygon(n, "round(r: 0.5mm) whole\n"));
        let want = format!("expanded `round(r: 0.5mm) whole` over {} faces of `whole`, of which {} \
                            pairs meet and {} threes meet each other", n + 2, 3 * n, 2 * n);
        assert!(said.contains(&want), "{n} sides: {said:?}");
    }
    for k in [2] {
        let (said, _) = expanded(&pocketed(k, "round(r: 0.5mm) whole\n"));
        let want = format!("expanded `round(r: 0.5mm) whole` over {} faces of `whole`, of which {} \
                            pairs meet and {} threes meet each other", 6 + 6 * k, 12 + 12 * k,
                           8 + 8 * k);
        assert!(said.contains(&want), "{k} pockets: {said:?}");
    }
}

/// **A round is linear in the edges** (#994): what a prism's round makes grows by as much for
/// each side added however many it has, as its edges and corners do — a set and its fill for
/// each edge and corner, not a copy for every pair and three of its faces.  Each copy a `when`
/// leaves out is not walked at all (`Walk::adjacent`), so a body of many faces is read for its
/// edges as a block is.
#[test]
fn a_round_over_a_body_makes_as_much_as_its_edges() {
    let made = |n| expanded(&polygon(n, "round(r: 0.5mm) whole\n")).1;
    let (s4, s8, s16) = (made(4), made(8), made(16));
    let bare = expanded(&polygon(4, "")).1;
    assert_eq!(s16 - s8, 2 * (s8 - s4), "{s4}, {s8}, {s16}");
    // each side adds three edges and two corners, and each is at most a set and its fill
    assert!(s8 - s4 <= 4 * 2 * (3 + 2), "{s4}, {s8}");
    assert!(s4 > bare, "the round made nothing: {s4}");
}

/// **An expansion past its limit over faces says what it was expanding** (#994): the statement
/// it was walking, the faces it ran over and how many meet, and what to do — at the statement,
/// once, and nothing built.
#[test]
fn an_expansion_past_its_limit_over_faces_says_what_it_ran_over() {
    let src = polygon(4, "repeat a in whole {\n  repeat 1000 as i {\n    repeat 300 as j {\n      \
                          p := point hint((i, j))\n    }\n  }\n}\n");
    let (e, d) = read(&src);
    assert_eq!(d.len(), 1, "{d:?}");
    let want = "E103: more than 200000 statements once expanded, from `repeat a in whole { …` over \
                6 faces of `whole`, of which 12 pairs meet and 8 threes meet each other: the drawing \
                is cut short, and nothing of it is built — run it over fewer faces at once";
    assert!(d[0].starts_with(want), "{d:?}");
    assert_eq!(e.diags[0].span.lo as usize, src.find("repeat a in whole").unwrap());
    assert!(e.sketch.points.is_empty(), "{} points built", e.sketch.points.len());
}
