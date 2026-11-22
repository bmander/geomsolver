//! **A round that runs out against a face flush with its end** (#962 item 4).  A boss across an
//! arm whose end caps lie flush with the arm's sides, filleted to the arm's top and bottom: each
//! fill runs along the boss to where the top meets the side, and ends there on the side, square.
//! What a run ends on is a face's plane, read where the run's edge ends; a drawing states its
//! numbers to five or six places, so a side is square to the edge only to those, and the piece was
//! carried past it until its whole section had crossed it and cut back to it — along its own end,
//! by a plane a hundred-millionth of a millimetre off it, a sliver the Boolean could not classify
//! (#962's 122: "a run cut off at its end: a piece of `far` (a plane) touches `far` (a plane) …
//! along no face of either"; drawn a little further off square, "a shell of the arrangement is kept
//! on one side of a plane (`far`) and dropped on another").  A plane holding the end's section as
//! the body reads it is the piece's own end (`lowering::piece::holds_end`).
//!
//! The family, each drawn whole at DOF 0 and each volume its closed form's to 1e-9: an eye (a
//! cylinder across the arm) and a block boss, flush with both sides, a hair (a micron, half a
//! millimetre) short of one and proud of one, rounded and chamfered to the top and the bottom, the
//! boss's own faces or only its sides; the eye across the arm at a slant, cut flush by the sides; a
//! boss flush with the arm's end too, its fill running out where two flush faces meet; the slot,
//! the convex twin, open at both sides, its rim rounded and its floor filled; a cutter rounded
//! before it is cut from the arm (#962's 133), its ends square, or square only to the drawing's
//! numbers; the arm's sides turned off square by 1e-8 to 1e-3 of a radian; and #962's 122 as the
//! issue reduced it.  And a face along a run, or back to back with one of its faces, is no face it
//! ends on (#996 item 2): a flange's underside rimmed where the boxes it stands on end flush with
//! its side or run on past it.  Each is exported (STEP and STL, read back) with no face thinner
//! than the export's tolerance.
use gcs_core::brep::{export, props, sweep::Say};
use std::f64::consts::PI;

use crate::common::rounds::gauss_legendre;
use crate::common::{at, close, ent, text::{dof, solved}};

/// The arm: `[0, 60] × [0, 20] × [−5, 5]`, its sides turned by `tilt` (radians) about `x = 30`.
const LENGTH: f64 = 60.0;
const WIDTH: f64 = 20.0;
const HALF: f64 = 5.0;

/// A round or a chamfer, of radius or legs its number.
#[derive(Clone, Copy, Debug)]
enum Word {
    Round(f64),
    Chamfer(f64),
}

impl Word {
    fn text(self) -> String {
        match self {
            Word::Round(r) => format!("round(r: {r}mm)"),
            Word::Chamfer(d) => format!("chamfer(d: {d}mm)"),
        }
    }

    /// What it takes or fills along a corner of two planes square to one another, per length.
    fn square(self) -> f64 {
        match self {
            Word::Round(r) => (1.0 - PI / 4.0) * r * r,
            Word::Chamfer(d) => 0.5 * d * d,
        }
    }
}

/// A boss's section, in the plane square to its axis: an eye of radius `big` about the axis `zc`
/// above the arm's middle, or a block `2w` wide from `zb` to `zt`.
#[derive(Clone, Copy, Debug)]
enum Boss {
    Eye { big: f64, zc: f64 },
    Block { w: f64, zb: f64, zt: f64 },
}

/// The arm and a boss about the line through `(cx, ·, zc)` along y turned `skew` toward x, from
/// `y0` to `y1` (its caps the planes `y = y0`, `y = y1`), unioned; `word` between `faces` of the
/// boss (`eye`, all of them, else a path under it) and the arm's top and bottom.
#[derive(Clone, Debug)]
struct Arm {
    boss: Boss,
    cx: f64,
    ends: [f64; 2],
    skew: f64,
    tilt: f64,
    faces: &'static str,
    word: Option<Word>,
}

fn arm(boss: Boss, ends: [f64; 2], word: Option<Word>) -> Arm {
    Arm { boss, cx: 30.0, ends, skew: 0.0, tilt: 0.0, faces: "eye", word }
}

const HEAD: &str = "unit mm\nuse std\nuse std.solids (cut, union, common)\nuse std.rounds (round, chamfer)\n";

/// A closed outline of fixed points named `{p}0…`, its lines `{l}0…`, and the face `{f}`.
fn outline(p: &str, l: &str, f: &str, corners: &[[f64; 2]]) -> String {
    let mut s = String::new();
    for (k, c) in corners.iter().enumerate() {
        s.push_str(&format!("  {p}{k} := point\n  fix(({:.17}, {:.17})) {p}{k}\n", c[0], c[1]));
    }
    let n = corners.len();
    for k in 0..n {
        s.push_str(&format!("  {l}{k} := line({p}{k}, {p}{})\n", (k + 1) % n));
    }
    let lines: Vec<String> = (0..n).map(|k| format!("{l}{k}")).collect();
    s.push_str(&format!("  {f} := face({})\n", lines.join(", ")));
    s
}

/// The arm's outline, its sides turned `tilt` about `x = 30`: `s0` along y = 0, `s2` along y = 20.
fn arm_outline(tilt: f64) -> String {
    let h = 0.5 * LENGTH * tilt;
    format!("in std.top {{\n{}}}\npart := solid(std.solids.Prism(outline, from: -{HALF}mm, to: {HALF}mm))\n",
        outline("p", "s", "outline", &[[0.0, -h], [LENGTH, h], [LENGTH, WIDTH - h], [0.0, WIDTH + h]]))
}

impl Arm {
    fn document(&self) -> String {
        let mut s = String::from(HEAD);
        s.push_str(&arm_outline(self.tilt));
        // the boss's section in its own plane: (x − cx, z) square to its axis
        let section = match self.boss {
            Boss::Eye { big, zc } => format!("  ec := point\n  fix(({}, {zc})) ec\n  eo := circle(center: ec) hint(r: {big})\n  \
                radius({big}mm) eo\n  section := face(eo)\n", if self.skew == 0.0 { self.cx } else { 0.0 }),
            Boss::Block { w, zb, zt } => {
                let x = if self.skew == 0.0 { self.cx } else { 0.0 };
                outline("q", "b", "section", &[[x - w, zb], [x + w, zb], [x + w, zt], [x - w, zt]])
            }
        };
        let [y0, y1] = self.ends;
        if self.skew == 0.0 {
            s.push_str(&format!("in std.front {{\n{section}}}\n\
                eye := solid(std.solids.Prism(section, from: {}mm, to: {}mm))\n", -y1, -y0));
        } else {
            // drawn in a plane square to the turned axis, long, and kept between the caps' planes
            let (sn, cs) = self.skew.sin_cos();
            s.push_str(&format!("su := axis hint(dir: ({cs}, {}, 0))\nfix(dir == ({cs:.17}, {:.17}, 0), origin == ({}, 10, 0)) su\n\
                sv := axis hint(dir: (0, 0, 1))\nfix(dir == (0, 0, 1), origin == ({}, 10, 0)) sv\nsp := plane(u: su, v: sv)\n\
                in sp {{\n{section}}}\neye := solid(std.solids.Prism(section, from: -80mm, to: 80mm))\n\
                in std.top {{\n{}}}\nslab := solid(std.solids.Prism(between, from: -30mm, to: 30mm))\nslab common eye\n",
                -sn, -sn, self.cx, self.cx, outline("k", "m", "between", &[[-40.0, y0], [100.0, y0], [100.0, y1], [-40.0, y1]])));
        }
        s.push_str("eye union part\n");
        if let Some(w) = self.word {
            for f in ["near", "far"] {
                s.push_str(&format!("part.{} {} part.{f}\n", self.faces, w.text()));
            }
        }
        s
    }

    /// The closed form: the arm, the boss outside it, and four fills (each side of the boss, top
    /// and bottom) each along the boss's length within the arm.
    fn volume(&self) -> f64 {
        let slant = self.skew.cos();
        let inside = self.ends[1].min(WIDTH) - self.ends[0].max(0.0);
        let (section, within) = match self.boss {
            Boss::Eye { big, zc } => (PI * big * big, PI * big * big - segment(big, HALF - zc) - segment(big, HALF + zc)),
            Boss::Block { w, zb, zt } => (2.0 * w * (zt - zb), 2.0 * w * 2.0 * HALF),
        };
        // a block's side flush with the arm's end, or not named, meets the top in no edge
        let sides = match self.boss {
            Boss::Block { w, .. } if self.faces.ends_with(".b1") || self.cx + w >= LENGTH => 1.0,
            _ => 2.0,
        };
        let fills = 0.5 * sides * match (self.word, self.boss) {
            (None, _) => 0.0,
            (Some(w), Boss::Block { .. }) => 4.0 * w.square(),
            (Some(w), Boss::Eye { big, zc }) => [zc - HALF, -HALF - zc].into_iter().map(|c| beside_eye(w, big, c)).sum(),
        };
        LENGTH * WIDTH * 2.0 * HALF + (section * (self.ends[1] - self.ends[0]) - within * inside) / slant
            + fills * inside / slant
    }
}

/// The area of the disc of `big` past a line `b` from its centre.
fn segment(big: f64, b: f64) -> f64 { big * big * (b / big).acos() - b * (big * big - b * b).sqrt() }

/// What `word` fills between an eye of radius `big` and a plane its axis stands `c` (negative)
/// from, on the side away from it: both corners.  A round's in closed form — the quadrilateral of
/// the axis, where the eye crosses the plane, where the ball touches it and the ball's centre, less
/// the eye's sector and the ball's; a chamfer's (the points whose distances from the plane and the
/// eye sum to at most its leg) over the angle about the axis, `ρ` from the larger of the eye's
/// radius and the plane's distance to where the sum is the leg, `(d + R − c) / (1 + sin φ)`.
fn beside_eye(word: Word, big: f64, c: f64) -> f64 {
    match word {
        Word::Round(r) => {
            let a = (big * big - c * c).sqrt();
            let dx = ((big + r).powi(2) - (r - c).powi(2)).sqrt();
            let pts = [[0.0, c], [-a, 0.0], [-dx, 0.0], [-dx, r]];
            let quad = 0.5 * (0..4).map(|i| { let (p, q) = (pts[i], pts[(i + 1) % 4]); p[0] * q[1] - q[0] * p[1] }).sum::<f64>().abs();
            let angle = |u: [f64; 2], v: [f64; 2]| ((u[0] * v[0] + u[1] * v[1]) / (u[0].hypot(u[1]) * v[0].hypot(v[1]))).acos();
            let at_axis = angle([-a, -c], [-dx, r - c]);
            let at_ball = angle([0.0, -r], [dx, c - r]);
            2.0 * (quad - 0.5 * big * big * at_axis - 0.5 * r * r * at_ball)
        }
        Word::Chamfer(d) => {
            assert!(d - c < big, "a flat over the eye's crown");
            let hi = |s: f64| (d + big - c) / (1.0 + s);
            let pieces: [(f64, f64, &dyn Fn(f64) -> f64); 2] = [
                ((-c / (d + big)).asin(), (-c / big).asin(), &|s: f64| -c / s),
                ((-c / big).asin(), ((d - c) / big).asin(), &|_| big),
            ];
            let nodes = gauss_legendre(60);
            2.0 * pieces.iter().map(|&(a, b, lo)| nodes.iter().map(|&(x, w)| {
                let s = (0.5 * (a + b) + 0.5 * (b - a) * x).sin();
                0.5 * (b - a) * w * 0.5 * (hi(s).powi(2) - lo(s).powi(2))
            }).sum::<f64>()).sum::<f64>()
        }
    }
}

/// Solid `name` of `src` built exactly, exported as STEP and STL (each read back), no face of it
/// thinner than the export's tolerance; its volume.
fn built(src: &str, name: &str) -> f64 {
    let e = solved(src);
    let say = Say { stage: &|_: &str| {}, mark: &|_| {} };
    let body = ent(&e, name).i();
    let exact = export::exact(&e.sketch, body, None, None, &say).unwrap_or_else(|r| panic!("{name}: {r:?}\n{src}"));
    if let Some(s) = export::slivers(&e.sketch, body, &exact, None) {
        panic!("{}\n{src}", s.message);
    }
    export::step(&exact, name, None, &say).unwrap();
    let stl = export::stl(&e.sketch, body, &exact, None, &say).unwrap();
    gcs_core::mesh::stl_shells(&stl).unwrap();
    props::volume(&exact.solid)
}

fn holds(a: &Arm) -> f64 {
    let src = a.document();
    assert_eq!(dof(&src), 0, "{src}");
    let got = built(&src, "part");
    close(got, a.volume());
    got
}

/// **An eye flush with the arm's sides** (#962's 122), rounded and chamfered to the top and the
/// bottom: flush with both, proud of both, flush with one and proud of the other; its axis at the
/// arm's middle and off it.  Rounded only along its cylinder, it is a hair (50 µm, half a
/// millimetre) short of a side too, and the flush eye is the limit of the short and the proud: the
/// closed form is continuous there.  (Rounded over its caps too, a cap short of the side meets the
/// top in an edge of its own, filled however narrow the shelf it stands on: the set itself is not
/// continuous there.  A shelf narrower than the export's tolerance is the drawing's own sliver.)
#[test]
fn an_eye_flush_with_the_arms_sides_is_filled_to_its_top_and_bottom() {
    for zc in [0.0, 1.5] {
        let boss = Boss::Eye { big: 9.0, zc };
        for word in [Word::Round(3.0), Word::Chamfer(2.0)] {
            for ends in [[0.0, WIDTH], [-0.5, WIDTH + 0.5], [0.0, WIDTH + 0.5]] {
                holds(&arm(boss, ends, Some(word)));
            }
            let flush = holds(&Arm { faces: "eye.eo", ..arm(boss, [0.0, WIDTH], Some(word)) });
            for hair in [0.05, 0.5] {
                for ends in [[hair, WIDTH], [-hair, WIDTH], [0.0, WIDTH - hair], [-hair, WIDTH + hair]] {
                    holds(&Arm { faces: "eye.eo", ..arm(boss, ends, Some(word)) });
                }
            }
            // the closed form is continuous at flush: the short and the proud tend to it
            for ends in [[1e-12, WIDTH], [-1e-12, WIDTH + 1e-12]] {
                close(Arm { faces: "eye.eo", ..arm(boss, ends, Some(word)) }.volume(), flush);
            }
        }
    }
}

/// **A block boss flush with the arm's sides**, and flush with its end too (its fill running out
/// where the end and the side meet, two flush faces): rounded and chamfered, its own faces or its
/// sides alone, flush, short of a side and proud of it.
#[test]
fn a_block_boss_flush_with_the_arms_sides_is_filled_to_its_top_and_bottom() {
    let boss = Boss::Block { w: 6.0, zb: -12.0, zt: 10.0 };
    for word in [Word::Round(3.0), Word::Chamfer(2.0)] {
        for cx in [30.0, LENGTH - 6.0] {
            for ends in [[0.0, WIDTH], [-0.5, WIDTH + 0.5], [0.0, WIDTH + 0.5]] {
                holds(&Arm { cx, ..arm(boss, ends, Some(word)) });
            }
        }
        for ends in [[0.05, WIDTH], [0.5, WIDTH], [0.0, WIDTH - 0.5], [-0.05, WIDTH + 0.05]] {
            holds(&Arm { faces: "eye.b1", ..arm(boss, ends, Some(word)) });
        }
    }
}

/// **An eye across the arm at a slant**, kept between the planes of the arm's sides: its caps
/// flush with them and aslant its axis, so each fill ends on a side at a slant, cut off there.
#[test]
fn an_eye_across_the_arm_at_a_slant_is_cut_off_flush_with_its_sides() {
    for skew in [0.3, -0.3] {
        for word in [Word::Round(3.0), Word::Chamfer(2.0)] {
            holds(&Arm { skew, ..arm(Boss::Eye { big: 9.0, zc: 0.0 }, [0.0, WIDTH], Some(word)) });
            holds(&Arm { skew, ..arm(Boss::Block { w: 6.0, zb: -12.0, zt: 10.0 }, [0.0, WIDTH], Some(word)) });
        }
    }
}

/// **The arm's sides square only to the drawing's numbers** (#962's 122: a side through a solved
/// point eight tenths of a micron off its neighbour's line): turned 1e-8 to 1e-3 of a radian about
/// the arm's middle, the eye proud of both.  Each fill ends on a side the plane of whose end
/// section it holds as the body reads it, or one it is cut off at — the same volume either way,
/// the fills' lengths and the arm's along the eye summing to the square arm's.
#[test]
fn a_fill_ending_on_a_side_square_to_the_drawings_numbers_ends_there() {
    for tilt in [1e-8, -1e-8, 1e-7, 1e-6, 1e-5, 1e-4, 1e-3] {
        for word in [Word::Round(3.0), Word::Chamfer(2.0)] {
            holds(&Arm { tilt, ..arm(Boss::Eye { big: 9.0, zc: 0.0 }, [-2.0, WIDTH + 2.0], Some(word)) });
        }
    }
}

/// The arm less a slot `[25, 35]` wide from `z = 1` up through the top, from `y0` to `y1`: its
/// rim (the walls with the top) and its floor (with the walls) each taken by `rim` and `floor`
/// where given; or, `before`, the slot's cutter rounded along its floor before it is cut.
fn slot(ends: [f64; 2], rim: Option<Word>, floor: Option<Word>, before: Option<Word>) -> String {
    let [y0, y1] = ends;
    let mut s = String::from(HEAD);
    s.push_str(&arm_outline(0.0));
    s.push_str(&format!("in std.front {{\n{}}}\nslot := solid(std.solids.Prism(cs, from: {}mm, to: {}mm))\n",
        outline("q", "b", "cs", &[[25.0, 1.0], [35.0, 1.0], [35.0, 12.0], [25.0, 12.0]]), -y1, -y0));
    if let Some(w) = before {
        s.push_str(&format!("slot.b0 {} slot.b1\nslot.b0 {} slot.b3\n", w.text(), w.text()));
    }
    s.push_str("slot cut part\n");
    if let Some(w) = rim {
        s.push_str(&format!("part.slot {} part.near\n", w.text()));
    }
    if let Some(w) = floor {
        s.push_str(&format!("part.slot.b0 {} part.slot.b1\npart.slot.b0 {} part.slot.b3\n", w.text(), w.text()));
    }
    s
}

/// **The convex twin: a slot open at the arm's sides** (its cutter flush with them and proud of
/// them), its rim rounded or chamfered — each run ending on a side where the wall meets it — and
/// its floor filled; and **the cutter rounded before it is cut** (#962's 133), its floor's edges
/// running out at its own caps, flush with the arm's sides or past them.
#[test]
fn a_slot_open_at_the_arms_sides_is_rounded_along_its_rim_and_floor() {
    let block = LENGTH * WIDTH * 2.0 * HALF;
    let slot_volume = 10.0 * 4.0 * WIDTH;
    for ends in [[0.0, WIDTH], [-1.0, WIDTH + 1.0], [0.0, WIDTH + 1.0]] {
        for word in [Word::Round(1.5), Word::Chamfer(1.5)] {
            let along = 2.0 * word.square() * WIDTH;
            close(built(&slot(ends, Some(word), None, None), "part"), block - slot_volume - along);
            close(built(&slot(ends, None, Some(word), None), "part"), block - slot_volume + along);
            close(built(&slot(ends, Some(word), Some(word), None), "part"), block - slot_volume);
            close(built(&slot(ends, None, None, Some(word)), "part"), block - slot_volume + along);
        }
    }
}

/// A cutter drawn in the top plane, `[25, 35]` across and from `y0` to `y1`, its ends turned `tilt`
/// about its middle, from `z = 1` to 12, its floor's edges along y taken by `word` before it is cut
/// from the arm (#962's 133 as the issue tells it: a channel's cutter rounded first).
fn cutter(ends: [f64; 2], tilt: f64, word: Word) -> String {
    let [y0, y1] = ends;
    let h = 5.0 * tilt;
    let mut s = String::from(HEAD);
    s.push_str(&arm_outline(0.0));
    s.push_str(&format!("in std.top {{\n{}}}\nslot := solid(std.solids.Prism(cs, from: 1mm, to: 12mm))\n\
        slot.far {} slot.c1\nslot.far {} slot.c3\nslot cut part\n",
        outline("q", "c", "cs", &[[25.0, y0 - h], [35.0, y0 + h], [35.0, y1 - h], [25.0, y1 + h]]), word.text(), word.text()));
    s
}

/// **A cutter rounded before it is cut, its ends square only to the drawing's numbers** (#962's
/// 133): turned 1e-8 to 1e-3 of a radian, past the arm's sides; and square, flush with them.
#[test]
fn a_cutter_rounded_before_it_is_cut_ends_on_its_own_caps() {
    let block = LENGTH * WIDTH * 2.0 * HALF;
    for word in [Word::Round(1.5), Word::Chamfer(1.5)] {
        let want = block - (10.0 * 4.0 - 2.0 * word.square()) * WIDTH;
        close(built(&cutter([0.0, WIDTH], 0.0, word), "part"), want);
        for tilt in [0.0, 1e-8, -1e-7, 1e-6, 1e-4, 1e-3] {
            close(built(&cutter([-1.0, WIDTH + 1.0], tilt, word), "part"), want);
        }
    }
}

/// A half-round lug 6 tall on the arm's top, its flat on the arm's side `y = 0` from `x = 22` to
/// 38, its arc about `(30, off)`, the root taken by `word`.
fn lug(off: f64, word: Word) -> String {
    let mut s = String::from(HEAD);
    s.push_str(&arm_outline(0.0));
    s.push_str(&format!("in std.top {{\n  la := point\n  fix((38, 0)) la\n  lb := point\n  fix((22, 0)) lb\n  \
        lc := point\n  fix((30, {off:.17})) lc\n  lr := arc(center: lc, start: la, end: lb)\n  lf := line(lb, la)\n  \
        lface := face(lr, lf)\n}}\nlug := solid(std.solids.Prism(lface, from: {HALF}mm, to: 11mm))\nlug union part\n\
        part.lug {} part.near\n", word.text()));
    s
}

/// **A ring ending on a face through its axis only to the drawing's numbers**: a half-round lug
/// standing on the arm, its flat flush with the arm's side, its arc's centre `off` that side (a
/// hundred-millionth of a millimetre to a tenth, either way), its root rounded and chamfered.  The
/// ring's stretch ends on the side, a plane `off` its axis, so it is turned clear past it and cut
/// off there: the closed form is the fill's section turned, each point at `ρ` from the axis
/// through `π + 2 asin(off / ρ)`, over the arm and the lug (the disc's segment over the flat).
#[test]
fn a_ring_ending_on_a_side_through_its_axis_to_the_drawings_numbers_ends_there() {
    let block = LENGTH * WIDTH * 2.0 * HALF;
    let nodes = gauss_legendre(60);
    for off in [0.0, 1e-8, -1e-8, 1e-7, 1e-6, -1e-6, 1e-4, 0.1, -0.1] {
        let big = (64.0f64 + off * off).sqrt();
        let standing = 6.0 * segment(big, -off);
        for word in [Word::Round(2.0), Word::Chamfer(2.0)] {
            // the section's height `u` out from the lug's wall, by `t` over [0, π/2] where smooth
            let (reach, h): (f64, &dyn Fn(f64) -> (f64, f64)) = match word {
                Word::Round(r) => (0.5 * PI, &move |t: f64| (r * (1.0 - t.cos()), r * (1.0 - t.sin()) * r * t.sin())),
                Word::Chamfer(d) => (d, &move |u: f64| (u, d - u)),
            };
            let turned: f64 = nodes.iter().map(|&(x, w)| {
                let (u, height) = h(0.5 * reach * (1.0 + x));
                let rho = big + u;
                0.5 * reach * w * height * rho * (PI + 2.0 * (off / rho).asin())
            }).sum();
            let src = lug(off, word);
            assert_eq!(dof(&src), 0, "{src}");
            close(built(&src, "part"), block + standing + turned);
        }
    }
}

/// #962's 122 as the issue reduced it: the eye across the arm's bottom bar, whose side `e5` runs
/// through a point the solve leaves 0.8 µm off its other end's line (into an arc tangent to it at
/// `r5`, under the fill's section), the eye filled to `far` along its cylinder and its cap.  Its
/// fills are those of the same drawing with `r5` fixed on that line, the side square to the edge:
/// the part built with them less the part built without, each way, to 1e-9 (the runs a micron
/// longer or shorter at their ends are a part in 10¹¹ of it).
#[test]
fn the_issues_eye_is_filled_to_where_its_edge_meets_the_arms_side() {
    let round = "part.eyel round(r: 3mm) part.far\n";
    let square = FRAGMENT.replace("r5 := point hint((-11.5500, -36.3000))", "r5 := point\n  fix((-11.55, -36.3)) r5");
    assert_ne!(square, FRAGMENT);
    let e = solved(FRAGMENT);
    assert!((at(&e, "r5")[1] + 36.3).abs() > 1e-7, "the side off square, as the issue's solve leaves it");
    let [with, without, with_square, without_square] = [FRAGMENT.to_string(), FRAGMENT.replace(round, ""),
        square.clone(), square.replace(round, "")].map(|src| built(&src, "part"));
    assert!(with_square - without_square > 40.0);
    close(with, without + (with_square - without_square));
}

const FRAGMENT: &str = "use std
use std.solids (cut, union)
use std.rounds (round, chamfer)
in std.top {
  bc := point hint((-55.7500, 37.5000))
  cc := point hint((-11.5500, 1.1000))
  fc := point hint((24.8900, 57.8000))
  gc := point hint((-26.2260, -20.0000))
  t1 := point hint((-68.0004, 35.0143))
  g1 := point hint((-55.6268, -25.9658))
  g2 := point hint((-26.2260, -50.0000))
  r3 := point hint((55.7500, -50.0000))
  r4 := point hint((55.7500, -36.3000))
  r5 := point hint((-11.5500, -36.3000))
  f1 := point hint((8.6704, 32.5626))
  f2 := point hint((24.8900, 27.8000))
  l3 := point hint((55.7500, 27.8000))
  l4 := point hint((55.7500, 44.2000))
  l5 := point hint((15.2500, 44.2000))
  l6 := point hint((-3.7500, 50.0000))
  l7 := point hint((-55.7500, 50.0000))
  e1 := line(t1, g1)
  e2 := arc(center: gc, start: g1, end: g2)
  e3 := line(g2, r3)
  e4 := line(r3, r4)
  e5 := line(r4, r5)
  e6 := arc(center: cc, start: r5, end: f1, sense: cw)
  e7 := arc(center: fc, start: f1, end: f2)
  e8 := line(f2, l3)
  e9 := line(l3, l4)
  e10 := line(l4, l5)
  e11 := line(l5, l6)
  e12 := line(l6, l7)
  e13 := arc(center: bc, start: l7, end: t1)
  outline := face(e1, e2, e3, e4, e5, e6, e7, e8, e9, e10, e11, e12, e13)
}
in std.front {
  ec := point
  eo := circle(center: ec) hint(r: 12.5)
  radius(12.5mm) eo
}
part := solid(std.solids.Prism(outline, from: -5.0mm, to: 5.0mm))
eyel := solid(std.solids.Prism(face(eo), from: 27.8000mm, to: 44.2000mm))
eyel union part
part.eyel round(r: 3mm) part.far
";

/// Solid `name` of `src` built exactly and exported as STEP and STL (each read back); its volume,
/// and the export's sliver warning where it has one — which `bare` (the same body unrounded) must
/// have too: a strip the drawing's numbers leave is the body's own, never the round's.
fn built_beside(src: &str, bare: &str, name: &str) -> (f64, Option<String>) {
    let say = Say { stage: &|_: &str| {}, mark: &|_| {} };
    let read = |src: &str| {
        let e = solved(src);
        let body = ent(&e, name).i();
        let exact = export::exact(&e.sketch, body, None, None, &say).unwrap_or_else(|r| panic!("{name}: {r:?}\n{src}"));
        let slivers = export::slivers(&e.sketch, body, &exact, None).map(|s| s.message);
        (e, body, exact, slivers)
    };
    let (e, body, exact, slivers) = read(src);
    if let Some(s) = &slivers {
        assert!(read(bare).3.is_some(), "the round's own sliver: {s}\n{src}");
    }
    export::step(&exact, name, None, &say).unwrap();
    let stl = export::stl(&e.sketch, body, &exact, None, &say).unwrap();
    gcs_core::mesh::stl_shells(&stl).unwrap();
    (props::volume(&exact.solid), slivers)
}

/// The arm `[−50, 50] × [−15, 15] × [0, 10]`, its south side turned `tilt` about its west end, and a
/// boss standing on its top (`from: 10mm`, its base in the top's plane, as a boss is drawn): a
/// block `[x0, x1] × [y0, y1]` or a disc of radius 8 about `(0, y0)`, 12 tall; `word` between the
/// boss's faces named `faces` and the arm's top.
#[derive(Clone)]
struct Standing {
    block: Option<[f64; 4]>,
    disc: Option<f64>,
    tilt: f64,
    faces: &'static str,
    word: Option<Word>,
}

impl Standing {
    fn document(&self) -> String {
        let mut s = String::from(HEAD);
        s.push_str(&format!("in std.top {{\n{}", outline("a", "s", "arm_f",
            &[[-50.0, -15.0], [50.0, -15.0 + 100.0 * self.tilt], [50.0, 15.0], [-50.0, 15.0]])));
        if let Some([x0, x1, y0, y1]) = self.block {
            s.push_str(&outline("p", "b", "boss_f", &[[x0, y0], [x1, y0], [x1, y1], [x0, y1]]));
        }
        if let Some(y0) = self.disc {
            s.push_str(&format!("  bc := point\n  fix((0, {y0})) bc\n  rim := circle(center: bc) hint(r: 8)\n  \
                radius(8mm) rim\n  boss_f := face(rim)\n"));
        }
        s.push_str("}\narm := solid(std.solids.Prism(arm_f, from: 0mm, to: 10mm))\n\
            boss := solid(std.solids.Prism(boss_f, from: 10mm, to: 22mm))\npart := solid(arm)\nboss union part\n");
        if let Some(w) = self.word {
            for f in self.faces.split(' ') {
                s.push_str(&format!("part.{f} {} part.arm.near\n", w.text()));
            }
        }
        s
    }

    fn holds(&self, want: f64) -> Option<String> {
        let src = self.document();
        assert_eq!(dof(&src), 0, "{src}");
        let bare = Standing { word: None, ..self.clone() }.document();
        let (got, slivers) = built_beside(&src, &bare, "part");
        close(got, want);
        slivers
    }
}

/// The arm's volume, square.
const ARM: f64 = 100.0 * 30.0 * 10.0;

/// **A block boss standing on the arm's top, past its side** (its base in the top's plane, as a
/// boss is drawn): where it overhangs the arm's side its underside is a face the fill along its
/// sides ends on, lying along the run in the plane of the floor the fill stands on.  A face along a
/// run is no face it ends on (`BodyFaces::lies_along`), so it is no cut, and the fill ends on the
/// side, square.  Proud of one side, of both, flush with
/// one and proud of the other, flush with both; a hair (1 µm, 50 µm, half a millimetre) short of
/// flush or proud, rounded along its sides alone, the flush one their limit; rounded over all its
/// faces short of flush, its end's fill mitred with the sides' at its corners; standing at the
/// arm's corner, flush with its side and its end, the fill along its one free side running out
/// where the two flush faces meet the top; round and chamfer.
#[test]
fn a_boss_standing_on_the_top_past_its_side_is_filled_along_its_sides() {
    for word in [Word::Round(3.0), Word::Chamfer(2.0)] {
        let a = word.square();
        let boss = |y0: f64, y1: f64| 16.0 * 12.0 * (y1 - y0);
        let block = |y0: f64, y1: f64| Some([-8.0, 8.0, y0, y1]);
        for (y0, y1) in [(-17.0, 15.0), (-17.0, 17.0), (-15.0, 17.0), (-15.0, 15.0)] {
            let s = Standing { block: block(y0, y1), disc: None, tilt: 0.0, faces: "boss", word: Some(word) };
            assert!(s.holds(ARM + boss(y0, y1) + 2.0 * a * 30.0).is_none());
        }
        for hair in [1e-3, 0.05, 0.5] {
            for (y0, y1) in [(-15.0 + hair, 15.0), (-15.0 - hair, 15.0), (-15.0 + hair, 15.0 - hair)] {
                let s = Standing { block: block(y0, y1), disc: None, tilt: 0.0, faces: "boss.b1 boss.b3", word: Some(word) };
                let inside = y1.min(15.0) - y0.max(-15.0);
                s.holds(ARM + boss(y0, y1) + 2.0 * a * inside);
            }
            // over all its faces: the end's fill along its 16, whole however narrow the shelf of
            // the top it stands on, each run square at the boss's corner (a reflex corner of the
            // fill's: the wedge between the two ends is the set's own, unrounded)
            let s = Standing { block: block(-15.0 + hair, 15.0), disc: None, tilt: 0.0, faces: "boss", word: Some(word) };
            s.holds(ARM + boss(-15.0 + hair, 15.0) + a * (2.0 * (30.0 - hair) + 16.0));
        }
        let corner = Standing { block: Some([34.0, 50.0, -15.0, 15.0]), disc: None, tilt: 0.0, faces: "boss", word: Some(word) };
        assert!(corner.holds(ARM + boss(-15.0, 15.0) + a * 30.0).is_none());
    }
}

/// **The same with the arm's side square only to the drawing's numbers**: the south side turned
/// 1e-8 to 1e-3 of a radian about its west end (either way), the boss flush with where it was
/// drawn, or 2 proud.  Flush and turned outward, the side crosses under the boss, which overhangs
/// it by `(x + 50)·tilt` — a strip of its underside 4.2 nm to 58 µm wide, the body's own, which
/// the export warns of with or without the round — and each fill along a side is cut off by the
/// side's plane at its slant: their lengths at their sections' centroids sum to `60 − 100 tilt`.
/// Turned 1e-8, the body reads no strip (half a nanometre, under its tolerance), and the fill, cut
/// exactly at the side, ends half a nanometre short of the boss's wall: that end's corner a vertex
/// on the boss's edge within the reach of its end, and so that end (#994, `tolerant::at_end`), the
/// fill joined to the boss there — where the two were two vertices, and the Boolean could not
/// close the boss's wall past the fill's end: refused, naming the two faces flush only to the
/// drawing's numbers.
#[test]
fn a_boss_standing_on_a_side_turned_by_the_drawings_numbers_is_filled_to_it() {
    for word in [Word::Round(3.0), Word::Chamfer(2.0)] {
        let a = word.square();
        for tilt in [1e-8, 1e-7, 1e-6, 1e-4, 1e-3, -1e-6] {
            let arm = ARM - 0.5 * 100.0 * 100.0 * tilt * 10.0;
            let proud = Standing { block: Some([-8.0, 8.0, -17.0, 15.0]), disc: None, tilt, faces: "boss", word: Some(word) };
            proud.holds(arm + 16.0 * 12.0 * 32.0 + a * (60.0 - 100.0 * tilt));
            if tilt > 0.0 {
                let flush = Standing { block: Some([-8.0, 8.0, -15.0, 15.0]), disc: None, tilt, faces: "boss", word: Some(word) };
                let slivers = flush.holds(arm + 16.0 * 12.0 * 30.0 + a * (60.0 - 100.0 * tilt));
                // (turned under a tenth of a micron across the boss, the body reads no strip)
                assert_eq!(slivers.is_some(), 5e-8 <= tilt && 42.0 * tilt < 1e-2, "{tilt}: {slivers:?}");
            }
        }
    }
}

/// **A ring ending on a face square to its axis**: a disc boss standing on the arm's top past its
/// side, its root rounded or chamfered.  The ring's stretch ends on the side, carried round and
/// kept within it, and on the boss's underside where it overhangs — a plane square to the ring's
/// axis in the floor's plane, the section on the side it keeps: no cut.  Each point of the section
/// at `ρ` from the axis is turned through the circle's arc over the side, `2π − 2 acos(e / ρ)`,
/// `e` the axis's distance from the side.
#[test]
fn a_disc_boss_standing_past_the_side_is_filled_round_to_it() {
    let nodes = gauss_legendre(60);
    for word in [Word::Round(3.0), Word::Chamfer(2.0)] {
        for y0 in [-10.0, -12.0] {
            let e = y0 + 15.0;
            let (reach, h): (f64, Box<dyn Fn(f64) -> (f64, f64)>) = match word {
                Word::Round(r) => (0.5 * PI, Box::new(move |t: f64| (r * (1.0 - t.cos()), r * (1.0 - t.sin()) * r * t.sin()))),
                Word::Chamfer(d) => (d, Box::new(move |u: f64| (u, d - u))),
            };
            let turned: f64 = nodes.iter().map(|&(x, w)| {
                let (u, height) = h(0.5 * reach * (1.0 + x));
                let rho = 8.0 + u;
                0.5 * reach * w * height * rho * (2.0 * PI - 2.0 * (e / rho).acos())
            }).sum();
            let s = Standing { block: None, disc: Some(y0), tilt: 0.0, faces: "boss", word: Some(word) };
            assert!(s.holds(ARM + PI * 64.0 * 12.0 + turned).is_none());
        }
    }
}

/// **A flange's underside rimmed where the bodies it stands on end flush with its edge, or run on
/// past it** (#996 item 2).  A plate 80 × 68 with corners of 10 (its sides `s0` along y = −34,
/// `k1`, `s1` along x = 40, `k2`, `s2`, `k3`, `s3`, `k0`), 10 thick on the top plane, stands on
/// boxes under it, and `far`'s rim is rounded or chamfered against the plate's sides.  Where a box
/// runs out to or past the plate's side, its top lies in the underside's plane past the box's own
/// side, back to back with the underside: the two meet at the point where the box's side crosses
/// the rim.  The top lies along the run there and says nothing of where it ends — kept within it
/// the run was cut whole ("a piece of nothing cut off at a plane"), or, where it reached back over
/// a corner's seam, refused naming the underside as the face the run ends on.  The run ends on
/// the box's side, which crosses the corner's rounded side below it and makes no corner with it:
/// the ring is cut off by its plane.
const FLANGE: &str = "unit mm\nuse std\nuse std.solids (union)\nuse std.rounds (round, chamfer)\n\
    in std.top {\n  a := point\n  b := point\n  c := point\n  d := point\n  fix((-40, -34)) a\n  \
    fix((40, -34)) b\n  fix((40, 34)) c\n  fix((-40, 34)) d\n  \
    (s0 := line(a, b)) -> (k1 := round(r: 10mm)) (s1 := line(b, c)) -> (k2 := round(r: 10mm)) \
    (s2 := line(c, d)) -> (k3 := round(r: 10mm)) (s3 := line(d, a)) -> (k0 := round(r: 10mm)) \
    close\n  fl_f := face(s0, k1, s1, k2, s2, k3, s3, k0)\n}\n\
    flange := solid(std.solids.Prism(fl_f, from: 0mm, to: 10mm))\n";

/// The plate's volume.
const PLATE: f64 = (80.0 * 68.0 - 4.0 * (1.0 - PI / 4.0) * 100.0) * 10.0;

/// Solid `name`, the outline through `points` drawn in the front plane (`outline`: its points
/// `{name}0…`, its lines `{name}s0…`, its face `{name}f`) and stood over `[y0, y1]` — the front
/// plane's normal is −y: from −y1 to −y0, its `near` at y0 and `far` at y1.
fn drawn_front(name: &str, points: &[[f64; 2]], [y0, y1]: [f64; 2]) -> String {
    format!("in std.front {{\n{}}}\n{name} := solid(std.solids.Prism({name}f, from: {}mm, to: {}mm))\n",
        outline(name, &format!("{name}s"), &format!("{name}f"), points), -y1, -y0)
}

/// A box under the plate: `[x0, x1] × [y0, y1] × [−60, 0]`, its top past the plate's side `x = 40`
/// falling `fall` by its far end (a plane through that side's foot).
#[derive(Clone, Copy, Debug)]
struct Under {
    x: [f64; 2],
    y: [f64; 2],
    fall: f64,
}

impl Under {
    fn text(&self, name: &str) -> String {
        let [x0, x1] = self.x;
        let mut points = vec![[x0, 0.0], [x0, -60.0], [x1, -60.0], [x1, -self.fall]];
        if self.fall != 0.0 {
            points.push([40.0, 0.0]);
        }
        drawn_front(name, &points, self.y)
    }

    /// Its plan's area across x, times its depth along y.
    fn volume(&self) -> f64 {
        let across = (self.x[1] - self.x[0]) * 60.0 - 0.5 * (self.x[1] - 40.0) * self.fall;
        across * (self.y[1] - self.y[0])
    }

    /// Whether its plan holds `(x, y)`, its sides included.
    fn holds(&self, x: f64, y: f64) -> bool {
        self.x[0] <= x && x <= self.x[1] && self.y[0] <= y && y <= self.y[1]
    }
}

/// The plate standing on `under` (each named, unioned in the order given), and `stmt`.
fn flange_on(under: &[(&str, Under)], stmt: &str) -> String {
    let mut s = String::from(FLANGE);
    for (name, u) in under {
        s.push_str(&u.text(name));
    }
    s.push_str("part := solid(flange)\n");
    for (name, _) in under {
        s.push_str(&format!("{name} union part\n"));
    }
    s.push_str(stmt);
    s.push('\n');
    s
}

/// What `word` takes along the plate's rim at `z = 0` where no box stands under it: each side's
/// stretch of its line out of every box's plan, its section's area along it, and round each
/// corner's arc (about `(±30, ±24)`, radius 10) the section's points at `ρ` from the axis turned
/// through the arc's angles out of every box's plan — the run cut off by the box's side it ends
/// on, whatever of its section that side's plane leaves — integrated over `ρ` by Gauss–Legendre
/// (no box's corner stands within the section's reach of an arc, so each angle is smooth in `ρ`).
fn rim_taken(word: Word, under: &[Under]) -> f64 {
    let open = |x: f64, y: f64| !under.iter().any(|u| u.holds(x, y));
    // each side: a point at `s` along it, and its length
    let sides: [(Box<dyn Fn(f64) -> [f64; 2]>, f64); 4] = [
        (Box::new(|s| [-30.0 + s, -34.0]), 60.0),
        (Box::new(|s| [40.0, -24.0 + s]), 48.0),
        (Box::new(|s| [30.0 - s, 34.0]), 60.0),
        (Box::new(|s| [-40.0, 24.0 - s]), 48.0),
    ];
    // where a side's line crosses a box's plan's sides, and its stretches between, open or not
    let mut straight = 0.0;
    for (at, length) in &sides {
        let [a, b] = [at(0.0), at(1.0)];
        let mut cuts = vec![0.0, *length];
        for u in under {
            for (k, c) in [(0, u.x[0]), (0, u.x[1]), (1, u.y[0]), (1, u.y[1])] {
                let rate = b[k] - a[k];
                if rate.abs() > 0.0 {
                    let s = (c - a[k]) / rate;
                    if s > 0.0 && s < *length {
                        cuts.push(s);
                    }
                }
            }
        }
        cuts.sort_by(f64::total_cmp);
        let between = |w: &&[f64]| { let m = at(0.5 * (w[0] + w[1])); open(m[0], m[1]) };
        straight += cuts.windows(2).filter(between).map(|w| w[1] - w[0]).sum::<f64>();
    }
    // the section's points at `ρ` from a corner's axis and their height, at `s` over `[0, 1]`, and
    // `dρ/ds`: a round's over the ball's quarter turn (its height `r (1 − cos φ)` at `ρ = 10 − r +
    // r sin φ`, smooth where the arc meets the rim), a chamfer's along its leg
    let section: Box<dyn Fn(f64) -> (f64, f64, f64)> = match word {
        Word::Round(r) => Box::new(move |s: f64| {
            let (sin, cos) = (0.5 * PI * s).sin_cos();
            (10.0 - r + r * sin, r * (1.0 - cos), 0.5 * PI * r * cos)
        }),
        Word::Chamfer(d) => Box::new(move |s: f64| (10.0 - d + d * s, d * s, d)),
    };
    let corners = [([30.0, -24.0], -0.5 * PI), ([30.0, 24.0], 0.0), ([-30.0, 24.0], 0.5 * PI),
                   ([-30.0, -24.0], PI)];
    let open_turn = |c: [f64; 2], t0: f64, rho: f64| -> f64 {
        let at = |t: f64| [c[0] + rho * t.cos(), c[1] + rho * t.sin()];
        let mut cuts = vec![t0, t0 + 0.5 * PI];
        for u in under {
            for (k, v) in [(0, u.x[0]), (0, u.x[1]), (1, u.y[0]), (1, u.y[1])] {
                let q = (v - c[k]) / rho;
                if q.abs() >= 1.0 {
                    continue;
                }
                let base = if k == 0 { q.acos() } else { q.asin() };
                let roots = if k == 0 { [base, -base] } else { [base, PI - base] };
                for t in roots {
                    let t = t0 + (t - t0).rem_euclid(2.0 * PI);
                    if t < t0 + 0.5 * PI {
                        cuts.push(t);
                    }
                }
            }
        }
        cuts.sort_by(f64::total_cmp);
        cuts.windows(2).filter(|w| { let m = at(0.5 * (w[0] + w[1])); open(m[0], m[1]) })
            .map(|w| w[1] - w[0]).sum()
    };
    let turned: f64 = corners.iter().map(|&(c, t0)| {
        gauss_legendre(40).iter().map(|&(x, w)| {
            let (rho, height, rate) = section(0.5 * (1.0 + x));
            0.5 * w * rate * rho * height * open_turn(c, t0, rho)
        }).sum::<f64>()
    }).sum();
    word.square() * straight + turned
}

/// The plate and its boxes, unrounded: two overlapping boxes standing as deep, their common plan
/// counted once.
fn flange_and(under: &[Under]) -> f64 {
    let mut v = PLATE + under.iter().map(Under::volume).sum::<f64>();
    for (i, a) in under.iter().enumerate() {
        for b in &under[i + 1..] {
            let span = |p: [f64; 2], q: [f64; 2]| (p[1].min(q[1]) - p[0].max(q[0])).max(0.0);
            v -= span(a.x, b.x) * span(a.y, b.y) * 60.0;
        }
    }
    v
}

/// The plate on `under`, `far`'s rim rounded or chamfered against the plate's sides by `word`:
/// built, exported with no sliver, and its volume the closed form's.
fn rim_holds(under: &[(&str, Under)], word: Word) {
    let src = flange_on(under, &format!("part.flange.far {} part.flange", word.text()));
    assert_eq!(dof(&src), 0, "{src}");
    let boxes: Vec<Under> = under.iter().map(|&(_, u)| u).collect();
    close(built(&src, "part"), flange_and(&boxes) - rim_taken(word, &boxes));
}

/// **The issue's flange** (#996 item 2, as reported): a rib 12 wide and a wider block under it,
/// both drawn as pentagons ending flush with the plate's side `s1`, the rib's slope rising to the
/// plate's opposite side.  Its whole underside rim rounded 1: refused, "a run cut off at its end: a
/// piece of nothing cut off at a plane" — the block's top, past the plate's corner `k1`, read as a
/// face the corner's ring ends on.  Now the ring along `k1` ends on the block's side `y = −30`,
/// which crosses the arc 38 along, cut off by its plane; `s1`'s run ends square on the block's
/// other side, and `s3`'s on the rib's.  The flange's sides and the rib and the block, less the
/// rim's runs and rings (the closed form's, as `rim_taken` reads them).
#[test]
fn the_issues_flange_is_rimmed_where_its_bodies_end_flush() {
    let pentagon = |x0: f64| [[x0, 0.0], [0.0, -60.0], [30.0, -60.0], [40.0, -30.0], [40.0, 0.0]];
    let shoelace = |p: [[f64; 2]; 5]| {
        let cross = |k: usize| p[k][0] * p[(k + 1) % 5][1] - p[(k + 1) % 5][0] * p[k][1];
        0.5 * (0..5).map(cross).sum::<f64>().abs()
    };
    let (rib, block) = (pentagon(-40.0), pentagon(0.0));
    let drawn = |name: &str, p: [[f64; 2]; 5], y: [f64; 2]| drawn_front(name, &p, y);
    for word in [Word::Round(1.0), Word::Chamfer(1.0)] {
        let src = format!("{FLANGE}{}{}part := solid(flange)\nrib union part\nblock union part\n\
            part.flange.far {} part.flange\n", drawn("rib", rib, [-6.0, 6.0]),
            drawn("block", block, [-30.0, 6.0]), word.text());
        assert_eq!(dof(&src), 0, "{src}");
        // the rib's plan at the underside: `[−40, 40] × [−6, 6]`, its slope rising to `s3`'s foot
        let under = [Under { x: [-40.0, 40.0], y: [-6.0, 6.0], fall: 0.0 },
                     Under { x: [0.0, 40.0], y: [-30.0, 6.0], fall: 0.0 }];
        let whole = PLATE + 12.0 * shoelace(rib) + 24.0 * shoelace(block);
        close(built(&src, "part"), whole - rim_taken(word, &under));
    }
}

/// Boxes ending flush with the plate's side `s1`: crossing the corner `k1`'s arc and ending
/// mid-side (and mirrored, under `s3` and `k0`); reaching over a corner whole (`k2` stands on it,
/// `s2` ending on its end); its side through the seam where `k1` goes on into `s1` (the ring whole,
/// ending at its meridian); ending mid-side at both ends (`s1` in two runs).
const FLUSH: [Under; 5] = [
    Under { x: [0.0, 40.0], y: [-30.0, 6.0], fall: 0.0 },
    Under { x: [-40.0, 0.0], y: [-30.0, 6.0], fall: 0.0 },
    Under { x: [0.0, 40.0], y: [-30.0, 40.0], fall: 0.0 },
    Under { x: [0.0, 40.0], y: [-24.0, 6.0], fall: 0.0 },
    Under { x: [0.0, 40.0], y: [-10.0, 6.0], fall: 0.0 },
];

/// **The class, rounded**: each of the flush boxes (`FLUSH`) under the plate, its underside's rim
/// rounded 1.
#[test]
fn a_flanges_rim_is_rounded_where_a_box_under_it_ends_flush() {
    for u in FLUSH {
        rim_holds(&[("blk", u)], Word::Round(1.0));
    }
}

/// **Other radii**: the first box, the rim rounded 2 and 0.5, the block's side crossing the
/// corner's ring at another place in its section.
#[test]
fn a_flanges_rim_is_rounded_at_other_radii_where_a_box_under_it_ends_flush() {
    for word in [Word::Round(2.0), Word::Round(0.5)] {
        rim_holds(&[("blk", FLUSH[0])], word);
    }
}

/// **The class, chamfered**: the same runs and rings, each a flat's.
#[test]
fn a_flanges_rim_is_chamfered_where_a_box_under_it_ends_flush() {
    for u in FLUSH {
        rim_holds(&[("blk", u)], Word::Chamfer(1.0));
    }
}

/// **A rib under the box, in either order of the unions**: a rib 12 wide reaching on under the
/// plate, unioned before the box and after it.
#[test]
fn a_flanges_rim_over_a_rib_and_a_box_is_one_in_either_order() {
    let rib = Under { x: [-20.0, 40.0], y: [-6.0, 6.0], fall: 0.0 };
    rim_holds(&[("rib", rib), ("blk", FLUSH[0])], Word::Round(1.0));
    rim_holds(&[("blk", FLUSH[0]), ("rib", rib)], Word::Round(1.0));
}

/// **A box running on past the plate's side** (the issue's second variant): its top past `x = 40`
/// in the underside's plane, or falling 1 by its far end (a plane through the side's foot, along
/// the run of `s1`'s rim and back to back with no face).  Either lies along the run and ends
/// nothing: `s1`'s run ends square on the box's side, the corner's ring on the box's other side.
/// Refused before, the level top as the round reaching back over `k2`'s seam, "`flange.far`, which
/// its run along `flange.k2` ends on, reaches 1mm back over the section there … Move `flange.far`
/// clear of the seam, or over it" — the face rounded, named as the face its run ends on.  Over one
/// corner and over two, round and chamfer.
#[test]
fn a_flanges_rim_is_rounded_where_a_box_under_it_runs_on_past_its_edge() {
    for word in [Word::Round(1.0), Word::Chamfer(1.0)] {
        for u in [Under { x: [0.0, 50.0], y: [-30.0, 6.0], fall: 0.0 },
                  Under { x: [0.0, 50.0], y: [-30.0, 6.0], fall: 1.0 },
                  Under { x: [0.0, 50.0], y: [-30.0, 40.0], fall: 0.0 }] {
            rim_holds(&[("blk", u)], word);
        }
    }
}

/// **The other rim, and the concave edges beside the flush box**: the plate's top rim rounded
/// whole over the box flush with its side; and the underside filled into the box's sides, each
/// fill its section along its edge — along `y = 6` to the plate's side `x = 40`, where the box's
/// side goes on flush with the plate's, and along `y = −30` to where it meets the corner's arc at
/// `x = 38`, ending square there (the arc's cylinder, which the fill's own edge goes across, is no
/// face it ends on), the box's top past the arc back to back with the underside and no end of it.
#[test]
fn a_flanges_other_rim_and_its_fills_into_a_flush_box_are_built() {
    let flush = Under { x: [0.0, 40.0], y: [-30.0, 6.0], fall: 0.0 };
    let bare = flange_and(&[flush]);
    let src = flange_on(&[("blk", flush)], "part.flange.near round(r: 1mm) part.flange");
    close(built(&src, "part"), bare - rim_taken(Word::Round(1.0), &[]));
    for word in [Word::Round(1.0), Word::Chamfer(1.0)] {
        for (side, length) in [("far", 40.0), ("near", 38.0)] {
            let stmt = format!("part.flange.far {} part.blk.{side}", word.text());
            let src = flange_on(&[("blk", flush)], &stmt);
            close(built(&src, "part"), bare + word.square() * length);
        }
    }
}
