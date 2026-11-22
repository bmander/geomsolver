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
//! issue reduced it.  Each is exported (STEP and STL, read back) with no face thinner than the
//! export's tolerance.
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
/// run cuts it lengthwise or not at all; here the fill lies on the side it keeps (`along_run`), so
/// it is no cut, and the fill ends on the side, square.  Proud of one side, of both, flush with
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
