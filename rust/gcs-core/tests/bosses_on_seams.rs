//! **A boss or a cut across a rim at a tangent seam** (#994, #962's 127): an obround flange — two
//! half circles of 99.29 going on tangent into its two flat sides — its top rim rounded or
//! chamfered, and a boss standing flush on it (a cylinder or a square prism, unioned) or a hole
//! cut through it, crossing the rim near where a half circle goes on tangent into a flat side.
//! The rim is a smooth chain of edges from the boss's wall round the loop to its wall again; each
//! edge's piece is cut off by the wall where the chain ends on it, and so is the piece the chain
//! goes on into across the seam where the wall reaches back over the section there
//! (`BodyFaces::chain_ends`).  Slid through the seam — clear of it, its wall reaching back over
//! the section there by a few hundredths and by a few millimetres, over the seam itself, from the
//! half circle's side and from the flat's — each volume is held to an independent computation:
//! at each depth `w` under the top the rim takes the band between the outline and its offset in
//! by the section's width there, `δ(w)` (`r − √(r² − (r − w)²)` for a round, `d − w` for a
//! chamfer), less what of the band lies within the boss's footprint, so the flange less its rim
//! weighs `21·A(O ∪ D) − ∫ A(O \ O_δ) − A((O \ O_δ) ∩ D) dw` (`A(O \ D)` for a cut).  The areas
//! of regions bounded by lines and circles are Green's `½∮(x dy − y dx)` over the parts of each
//! boundary within the other, found by bisection, and the depth integrated by Simpson's rule
//! smooth in `t` (`w = r(1 − cos t)`).
use gcs_core::brep::props;
use std::f64::consts::PI;

use crate::common::rounds::exported;
use crate::common::text::solved;

/// The flange's half length between its half circles' centres, their centres' `y`, their radius
/// and its thickness.
const L: f64 = 384.62;
const YC: f64 = 10.55;
const R: f64 = 99.29;
const THICK: f64 = 21.0;

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
}

/// What stands across the rim: a disc about `at` of radius `r`, or a square about `at` of half side
/// `h` turned by `turn` radians.
#[derive(Clone, Copy, Debug)]
enum Foot {
    Disc { at: [f64; 2], r: f64 },
    Square { at: [f64; 2], h: f64, turn: f64 },
}

impl Foot {
    /// The square's corners, counter-clockwise.
    fn corners(at: [f64; 2], h: f64, turn: f64) -> [[f64; 2]; 4] {
        let (s, c) = turn.sin_cos();
        [[-h, -h], [h, -h], [h, h], [-h, h]].map(|[x, y]| [at[0] + c * x - s * y, at[1] + s * x + c * y])
    }
}

/// The flange, `foot` unioned onto it as a boss as thick (`cut`: cut through it), and its top rim
/// rounded or chamfered by `word` over the solid they leave.
fn document(foot: Foot, cut: bool, word: Word) -> String {
    let mut s = format!("use std\nuse std.solids (cut, union, common)\nuse std.rounds (round, chamfer)\nin std.top {{\n  \
        cr := point\n  fix(({L}, {YC})) cr\n  cl := point\n  fix((-{L}, {YC})) cl\n  \
        a := point\n  fix(({L}, {})) a\n  b := point\n  fix(({L}, {})) b\n  \
        c := point\n  fix((-{L}, {})) c\n  d := point\n  fix((-{L}, {})) d\n  \
        ar := arc(center: cr, start: a, end: b)\n  al := arc(center: cl, start: c, end: d)\n  \
        t := line(b, c)\n  s := line(d, a)\n  outline := face(ar, t, al, s)\n",
        YC - R, YC + R, YC + R, YC - R);
    let (from, to) = if cut { (-160.0, -120.0) } else { (-152.5, -131.5) };
    match foot {
        Foot::Disc { at, r } => s += &format!("  o := point\n  fix(({}, {})) o\n  disc := circle(center: o) hint(r: {r})\n  \
            radius({r}mm) disc\n  foot := face(disc)\n", at[0], at[1]),
        Foot::Square { at, h, turn } => {
            let p = Foot::corners(at, h, turn);
            for (k, q) in p.iter().enumerate() {
                s += &format!("  q{k} := point\n  fix(({}, {})) q{k}\n", q[0], q[1]);
            }
            s += "  k0 := line(q0, q1)\n  k1 := line(q1, q2)\n  k2 := line(q2, q3)\n  k3 := line(q3, q0)\n  \
                  foot := face(k0, k1, k2, k3)\n";
        }
    }
    s += &format!("}}\nflange := solid(std.solids.Prism(outline, from: -152.5mm, to: -131.5mm))\n\
        boss := solid(std.solids.Prism(foot, from: {from}mm, to: {to}mm))\nouter := solid(flange)\n\
        boss {} outer\npart := solid(outer)\npart.outer.flange.near {} part.outer.flange\n",
        if cut { "cut" } else { "union" }, word.text());
    s
}

/// A stretch of a region's boundary, counter-clockwise round it: a line, or an arc about `c` of
/// radius `r` from angle `from` to `to`.
#[derive(Clone, Copy)]
enum Curve {
    Line([f64; 2], [f64; 2]),
    Arc { c: [f64; 2], r: f64, from: f64, to: f64 },
}

impl Curve {
    /// Its point a fraction `s` along it.
    fn at(&self, s: f64) -> [f64; 2] {
        match *self {
            Curve::Line(p, q) => [p[0] + s * (q[0] - p[0]), p[1] + s * (q[1] - p[1])],
            Curve::Arc { c, r, from, to } => {
                let a = from + s * (to - from);
                [c[0] + r * a.cos(), c[1] + r * a.sin()]
            }
        }
    }

    /// Green's `½∫(x dy − y dx)` along it from `s0` to `s1`, in closed form.
    fn green(&self, s0: f64, s1: f64) -> f64 {
        match *self {
            Curve::Line(..) => {
                let (p, q) = (self.at(s0), self.at(s1));
                0.5 * (p[0] * q[1] - q[0] * p[1])
            }
            Curve::Arc { c, r, from, to } => {
                let (a, b) = (from + s0 * (to - from), from + s1 * (to - from));
                0.5 * (r * r * (b - a) + c[0] * r * (b.sin() - a.sin()) - c[1] * r * (b.cos() - a.cos()))
            }
        }
    }
}

/// A region of the plane: its boundary, and whether a point lies within it.
struct Region {
    bound: Vec<Curve>,
    inside: Box<dyn Fn([f64; 2]) -> bool>,
}

/// The flange's outline offset in by `delta`: its half circles' radius less `delta`.
fn obround(delta: f64) -> Region {
    let rr = R - delta;
    let (cl, cr) = ([-L, YC], [L, YC]);
    let bound = vec![
        Curve::Line([-L, YC - rr], [L, YC - rr]),
        Curve::Arc { c: cr, r: rr, from: -0.5 * PI, to: 0.5 * PI },
        Curve::Line([L, YC + rr], [-L, YC + rr]),
        Curve::Arc { c: cl, r: rr, from: 0.5 * PI, to: 1.5 * PI },
    ];
    Region { bound, inside: Box::new(move |p| (p[0].clamp(-L, L) - p[0]).hypot(p[1] - YC) <= rr) }
}

/// The footprint of `foot`.
fn footprint(foot: Foot) -> Region {
    match foot {
        Foot::Disc { at, r } => Region {
            bound: vec![Curve::Arc { c: at, r, from: 0.0, to: 2.0 * PI }],
            inside: Box::new(move |p| (p[0] - at[0]).hypot(p[1] - at[1]) <= r),
        },
        Foot::Square { at, h, turn } => {
            let q = Foot::corners(at, h, turn);
            let (s, c) = turn.sin_cos();
            Region {
                bound: (0..4).map(|k| Curve::Line(q[k], q[(k + 1) % 4])).collect(),
                inside: Box::new(move |p| {
                    let (x, y) = (p[0] - at[0], p[1] - at[1]);
                    (c * x + s * y).abs() <= h && (-s * x + c * y).abs() <= h
                }),
            }
        }
    }
}

/// Green's integral over the parts of `a`'s boundary within `b`: each boundary curve sampled at
/// 1000 points, each change of side found by bisection, each stretch between kept where its
/// middle is within.
fn part_within(a: &Region, b: &Region) -> f64 {
    let n = 1000;
    a.bound.iter().map(|cv| {
        let within = |s: f64| (b.inside)(cv.at(s));
        let mut cuts = vec![0.0];
        for k in 0..n {
            let (s0, s1) = (k as f64 / n as f64, (k + 1) as f64 / n as f64);
            if within(s0) != within(s1) {
                let (mut lo, mut hi) = (s0, s1);
                for _ in 0..60 {
                    let m = 0.5 * (lo + hi);
                    if within(m) == within(s0) { lo = m } else { hi = m }
                }
                cuts.push(0.5 * (lo + hi));
            }
        }
        cuts.push(1.0);
        cuts.windows(2).filter(|w| within(0.5 * (w[0] + w[1]))).map(|w| cv.green(w[0], w[1])).sum::<f64>()
    }).sum()
}

/// The area the two regions share.
fn common(a: &Region, b: &Region) -> f64 { part_within(a, b) + part_within(b, a) }

/// A region's area: Green's integral round its whole boundary.
fn area(a: &Region) -> f64 { a.bound.iter().map(|cv| cv.green(0.0, 1.0)).sum() }

/// What the flange with `foot` unioned onto it (`cut`: cut from it) weighs less its rim, `word`'s
/// section taken along it and kept out of the footprint.
fn expected(foot: Foot, cut: bool, word: Word) -> f64 {
    let (whole, d) = (obround(0.0), footprint(foot));
    let shared = common(&whole, &d);
    let plate = if cut { area(&whole) - shared } else { area(&whole) + area(&d) - shared };
    // what the rim takes at a depth where its section is `delta` wide
    let level = |delta: f64| {
        let inner = obround(delta);
        area(&whole) - area(&inner) - shared + common(&inner, &d)
    };
    let n = 2000;
    let simpson = |f: &dyn Fn(f64) -> f64, span: f64| {
        let step = span / n as f64;
        step / 3.0 * (0..=n).map(|k| {
            let w = if k == 0 || k == n { 1.0 } else if k % 2 == 1 { 4.0 } else { 2.0 };
            w * f(k as f64 * step)
        }).sum::<f64>()
    };
    let taken = match word {
        Word::Round(r) => simpson(&|t: f64| level(r - r * t.sin()) * r * t.sin(), 0.5 * PI),
        Word::Chamfer(c) => simpson(&|w: f64| level(c - w), c),
    };
    plate * THICK - taken
}

/// The volume of `part` built from `document(foot, cut, word)`, and the face count.
fn built(foot: Foot, cut: bool, word: Word) -> (f64, usize) {
    let e = solved(&document(foot, cut, word));
    assert!(e.diags.is_empty(), "{foot:?}: {:?}", e.diags);
    let b = exported(&e, "part");
    (props::volume(&b), b.faces.len())
}

/// Held to the independent computation to 1e-9.
fn holds(foot: Foot, cut: bool, word: Word) {
    let (got, _) = built(foot, cut, word);
    let want = expected(foot, cut, word);
    assert!((got - want).abs() <= 1e-9 * want, "{foot:?} {cut} {word:?}: {got} against {want}");
}

/// The independent computation is the closed form where the boss stands clear of both half
/// circles (`rim_seams::flange_and_boss_less_rim`'s: the boss's chord at each depth integrated in
/// closed form along the flat side).
#[test]
fn the_computation_is_the_closed_form_clear_of_the_seam() {
    let (r, rb, below): (f64, f64, f64) = (10.0, 46.76, 20.84);
    let obround = 4.0 * L * R + PI * R * R;
    let segment = rb * rb * (below / rb).acos() - below * (rb * rb - below * below).sqrt();
    let a = (1.0 - PI / 4.0) * r * r;
    let inset = (10.0 - 3.0 * PI) / (3.0 * (4.0 - PI)) * r;
    let chord = |t: f64| {
        let u = r * (1.0 - t.cos());
        r * (1.0 - t.sin()) * 2.0 * (rb * rb - (u - below).powi(2)).sqrt() * r * t.sin()
    };
    let n = 2000;
    let step = 0.5 * PI / n as f64;
    let kept_out = step / 3.0 * (0..=n).map(|k| {
        let w = if k == 0 || k == n { 1.0 } else if k % 2 == 1 { 4.0 } else { 2.0 };
        w * chord(k as f64 * step)
    }).sum::<f64>();
    let want = (obround + segment) * THICK - (a * (4.0 * L + 2.0 * PI * (R - inset)) - kept_out);
    let got = expected(Foot::Disc { at: [-300.0, -67.9], r: rb }, false, Word::Round(r));
    assert!((got - want).abs() <= 1e-10 * want, "{got} against {want}");
}

/// The boss of #962's 127, a disc of 46.76 about `(x, −67.9)`, slid along the flat side.
fn disc(x: f64) -> Foot { Foot::Disc { at: [x, -67.9], r: 46.76 } }

/// Clear of the seam at −384.62 (−300, −335): the half circle's ring untouched.
#[test]
fn a_disc_boss_clear_of_the_seam() {
    holds(disc(-300.0), false, Word::Round(10.0));
    holds(disc(-335.0), false, Word::Chamfer(6.0));
}

/// #962's 127 itself (−339.17): the seam on the rim, 3.6 short of the boss, and the boss's wall
/// reaching 0.035 back over the round's section there — refused before ("Move `outer.boss.disc`
/// clear of the seam, or over it"); the ring round the half circle is cut off by the wall too.
#[test]
fn the_fixtures_boss_reaching_back_over_the_seam() {
    holds(disc(-339.17), false, Word::Round(10.0));
    holds(disc(-339.17), false, Word::Chamfer(6.0));
}

/// Further on (−341): the seam 1.8 short of the boss, the wall 1.8 back over the section.
#[test]
fn a_disc_boss_reaching_further_back_over_the_seam() {
    holds(disc(-341.0), false, Word::Round(10.0));
    holds(disc(-341.0), false, Word::Chamfer(6.0));
}

/// The boss's wall crossing the rim exactly at the seam (`x = −384.62 + √(46.76² − 20.84²)`), and
/// a ten-thousandth short of it: the flat side's stretch there is of no length, or of a
/// ten-thousandth.  (A ten-thousandth past it, the seam inside the boss, builds too.)
#[test]
fn a_disc_boss_whose_wall_crosses_the_rim_at_the_seam() {
    let at = -L + (46.76f64 * 46.76 - 20.84 * 20.84).sqrt();
    for x in [at - 1e-4, at] {
        holds(disc(x), false, Word::Round(10.0));
    }
}

/// Over the seam (−342.9 just, −380 well): the seam inside the boss, the half circle's
/// ring ending on its wall.
#[test]
fn a_disc_boss_over_the_seam() {
    holds(disc(-342.9), false, Word::Round(10.0));
    holds(disc(-380.0), false, Word::Chamfer(6.0));
}
/// The boss crossing the half circle just short of the seam, its wall reaching over the flat
/// side's section there: the chain from the flat side's run goes on round the half circle into
/// the boss (a disc of 24 about (−400, −70): the seam 0.24 outside it, its section's inner corner
/// 6.3 inside).
#[test]
fn a_disc_boss_over_the_half_circle_reaching_back_over_the_flat() {
    for word in [Word::Round(10.0), Word::Chamfer(6.0)] {
        holds(Foot::Disc { at: [-400.0, -70.0], r: 24.0 }, false, word);
    }
}

/// A square boss, its sides square to the flat side and along it, clear of the seam: its wall, a
/// plane square to the rim, reaches back over no section, and the half circle's ring is whole.
#[test]
fn a_square_boss_beside_the_seam() {
    holds(Foot::Square { at: [-365.0, -80.0], h: 15.0, turn: 0.0 }, false, Word::Round(10.0));
    holds(Foot::Square { at: [-365.0, -80.0], h: 15.0, turn: 0.0 }, false, Word::Chamfer(6.0));
}

/// A round notch cut through the rim beside the seam, its wall reaching back over the section
/// there by 0.035 and by 1.8: the half circle's ring kept out of the hole.
#[test]
fn a_notch_cut_at_the_seam() {
    holds(disc(-339.17), true, Word::Round(10.0));
    holds(disc(-341.0), true, Word::Chamfer(6.0));
}



/// The square boss over the seam turned 0.04 rad, its wall `outer.boss.k3` crossing the half
/// circle a little off square: the ring ends on it, kept on its side (a plane not through the
/// ring's axis, carried round past it), and the plate less its rim is the band model's to 1e-9.
#[test]
fn a_square_boss_over_the_seam() {
    let foot = Foot::Square { at: [-372.0, -80.0], h: 15.0, turn: -0.04 };
    holds(foot, false, Word::Round(10.0));
    holds(foot, false, Word::Chamfer(6.0));
}

/// **What the definition gives where the ring's own edge crosses the wall's plane** (a defect held
/// as it stands, #994): the square over the seam not turned, its wall `x = −387` crossing the half
/// circle 1.4° off square at `V`.  The half circle's edge comes back across that plane at its far
/// end (`x = −384.62`), so the plane is no face it ends on (`BodyFaces::goes_across`: no side of
/// it holds the run), and the ring's region ends where `distance(p, crease) <= distance(p,
/// meeting)` puts it, at the meridian through `V` — which leans into the boss.  The body less
/// `taken` then loses the boss's material between the wall and that meridian: at a depth where
/// the section is `δ` wide, `∫ ρ |θ_wall(ρ) − θ_V| dρ` over `R − δ ≤ ρ ≤ R` in polar about the
/// half circle's centre, `θ_wall(ρ) = −acos((x_w + L) / ρ)`.  The field and the pieces agree with
/// that (the gate passes) and both are the band model less that wedge, to 1e-9: 1.149 mm³ at R10.
/// The shape meant is the band model's — a round never takes another body's material — and a
/// half-space cannot say it (the plane cuts the ring's far end); the boss's own solid, kept out
/// of as a boss's cylinder is (`kept_out_of`), would.
#[test]
fn a_wall_the_rings_own_edge_crosses_takes_the_bosss_wedge() {
    let (xw, r) = (-387.0f64, 10.0f64);
    let theta_v = (-(R * R - (xw + L).powi(2)).sqrt()).atan2(xw + L);
    let simpson = |f: &dyn Fn(f64) -> f64, a: f64, b: f64, n: usize| {
        let h = (b - a) / n as f64;
        h / 3.0 * (0..=n).map(|k| {
            let w = if k == 0 || k == n { 1.0 } else if k % 2 == 1 { 4.0 } else { 2.0 };
            w * f(a + k as f64 * h)
        }).sum::<f64>()
    };
    let wedge = |delta: f64| simpson(&|rho: f64| rho * (-((xw + L) / rho).acos() - theta_v).abs(), R - delta, R, 2000);
    let taken = simpson(&|t: f64| wedge(r - r * t.sin()) * r * t.sin(), 0.0, 0.5 * PI, 400);
    let foot = Foot::Square { at: [-372.0, -80.0], h: 15.0, turn: 0.0 };
    let (got, _) = built(foot, false, Word::Round(r));
    let want = expected(foot, false, Word::Round(r)) - taken;
    assert!((got - want).abs() <= 1e-9 * want, "{got} against {want} ({taken} taken from the boss)");
}

/// **Another body's wall turning back over the run is its end** (#994, `BodyFaces::another_body`):
/// a square boss turned 45° on the flat side, far from the seam, its wall meeting the rim at a
/// reflex corner of 45°.  A side of the plate's own there is no end — the run ends square — but
/// the boss's wall is: kept on its side, the round takes nothing of the boss (before, ending
/// square at the corner, it took 96 mm³ of it).  The chamfer of 6, whose section stays within
/// the wall's reach (the boss's side corners stand 8.74 in from the rim), is the band model's to
/// 1e-9.  The round of 10 reaches past those corners, and the wall's plane, a half-space, goes on
/// past them where the boss does not: the round is kept off it there too, and leaves 0.04 mm³
/// more than the band model; turned 30°, the corner nearer the rim, 21 mm³ more.  Never less: a
/// round takes no body's material but its own, and where a half-space cannot say how far the
/// boss goes, it keeps too much rather than taking the boss's.  The boss's own solid, kept out of
/// as a cylinder is (`kept_out_of`), is what would say it.
#[test]
fn another_bodys_wall_turning_back_over_the_run_ends_it() {
    let diamond = Foot::Square { at: [-200.0, -80.0], h: 15.0, turn: PI / 4.0 };
    holds(diamond, false, Word::Chamfer(6.0));
    let (got, _) = built(diamond, false, Word::Round(10.0));
    let want = expected(diamond, false, Word::Round(10.0));
    assert!(got > want && got < want + 0.1, "{got} against {want}");
    let turned = Foot::Square { at: [-200.0, -80.0], h: 15.0, turn: PI / 6.0 };
    let (got, _) = built(turned, false, Word::Round(10.0));
    let want = expected(turned, false, Word::Round(10.0));
    assert!(got > want + 1.0 && got < want + 30.0, "{got} against {want}");
}

/// **The refusals that remain** (`lowering::back_over_said`): where a face a run ends on reaches
/// back over the section at a seam and the chain gone on across it is not kept off it — a
/// half-space the half circle's own edge crosses (the 45° boss beside the seam: its wall's plane
/// would cut the half circle's far end), and a solid the run is kept within (a short flat side
/// ending at a corner on a small convex arc of its own plate, whose cylinder the half circle's
/// section at the seam stands partly outside).
#[test]
fn a_face_reaching_back_over_the_seam_that_keeps_no_chain_is_refused() {
    let diamond = Foot::Square { at: [-366.0, -80.0], h: 15.0, turn: PI / 4.0 };
    let why = refusal(&document(diamond, false, Word::Round(10.0)));
    assert!(why.starts_with("`part`'s round or chamfer `taken`: its round goes on tangent from `outer.flange.s` into \
        `outer.flange.al` across their seam at [-384.62, -88.74, -131.5]mm, and `outer.boss.k3`, which its run along \
        `outer.flange.s` ends on, reaches 2.7mm back over the section there"), "{why}");
    assert!(why.ends_with("Move `outer.boss.k3` clear of the seam, or over it"), "{why}");
    let src = "use std\nuse std.solids (cut, union)\nuse std.rounds (round)\nin std.top {\n  \
        o := point\n  fix((0, 20)) o\n  c := point\n  fix((-20, 20)) c\n  d := point\n  fix((0, 0)) d\n  \
        p := point\n  fix((1, 0)) p\n  k := point\n  fix((-1, -1)) k\n  q := point\n  fix((-1, 1.2360679775)) q\n  \
        al := arc(center: o, start: c, end: d)\n  s := line(d, p)\n  sm := arc(center: k, start: p, end: q)\n  \
        t := line(q, c)\n  outline := face(al, s, sm, t)\n}\n\
        plate := solid(std.solids.Prism(outline, from: 0mm, to: 5mm))\npart := solid(plate)\n\
        part.plate.near round(r: 1.5mm) part.plate\n";
    let why = refusal(src);
    assert!(why.contains("its round goes on tangent from `plate.s` into `plate.al` across their seam")
        && why.contains("and `plate.sm`, which its run along `plate.s` ends on, reaches 0.46mm back over the section there")
        && why.ends_with("Move `plate.sm` clear of the seam, or over it"), "{why}");
}

/// The export's refusal of `part` in document `src`, as it is printed.
fn refusal(src: &str) -> String {
    let e = solved(src);
    let say = gcs_core::brep::sweep::Say { stage: &|_: &str| {}, mark: &|_| {} };
    let body = crate::common::ent(&e, "part").i();
    gcs_core::brep::export::exact(&e.sketch, body, None, None, &say).err().expect("refused").message
}
