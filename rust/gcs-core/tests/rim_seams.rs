//! **A rim round or chamfer across tangent seams** (#962 item 2): a prism's rim rounded or
//! chamfered round an outline of lines and arcs meeting tangent, a run along each line and a
//! ring's stretch round each arc, each glued to the next at the seam's one section.  A seam the
//! body reads as tangent — its two faces' normals one to `faces::TANGENT` — is no end, so its two
//! pieces are one section there by the same reading: a drawing whose arcs meet tangent to its own
//! numbers (a line `1e-7` off its arc's tangent, two arcs `5e-7`) has pieces whose caps are a
//! millionth apart, glued at what they stand apart, never unioned (a union of two caps a hair
//! apart could not tell what touched what).  Each family drawn whole (every seam tangent, so the
//! rim is pieces end to end and nothing else): the step of #962's 117 — a concave arc into a
//! convex lobe, the lobe tangent into a line, the line into a concave arc — at every line length
//! down to none and every radius to past the smaller arc's; a plate of alternating lobes and
//! waists, four, six and twelve arcs (#962's 132); each mirrored and turned, round and chamfer, on
//! the top rim and the bottom; each volume the closed form's to 1e-9 (the outline's area by
//! Green's theorem, a run's section along each line and Pappus round each arc).
use gcs_core::brep::props;
use std::f64::consts::PI;

use crate::common::rounds::{exact, exported, square_run};
use crate::common::{close, text::solved};

/// A stroke of an outline on the top plane: a line, or an arc about `c` from `s` to `e`, turning
/// counter-clockwise or not.
#[derive(Clone, Copy, Debug)]
enum Seg {
    Line([f64; 2], [f64; 2]),
    Arc { c: [f64; 2], s: [f64; 2], e: [f64; 2], ccw: bool },
}

impl Seg {
    /// Its first and last points.
    fn ends(&self) -> ([f64; 2], [f64; 2]) {
        match *self {
            Seg::Line(a, b) => (a, b),
            Seg::Arc { s, e, .. } => (s, e),
        }
    }

    /// Moved by `f`, which turns or mirrors the plane (`mirror`: the sense of each arc flips).
    fn mapped(&self, f: &dyn Fn([f64; 2]) -> [f64; 2], mirror: bool) -> Seg {
        match *self {
            Seg::Line(a, b) => Seg::Line(f(a), f(b)),
            Seg::Arc { c, s, e, ccw } => Seg::Arc { c: f(c), s: f(s), e: f(e), ccw: ccw != mirror },
        }
    }

    /// Its sweep about its centre, signed counter-clockwise (an arc's).
    fn sweep(&self) -> f64 {
        let Seg::Arc { c, s, e, ccw } = *self else { return 0.0 };
        let (a0, a1) = ((s[1] - c[1]).atan2(s[0] - c[0]), (e[1] - c[1]).atan2(e[0] - c[0]));
        let turn = (a1 - a0).rem_euclid(2.0 * PI);
        if ccw { turn } else { turn - 2.0 * PI }
    }
}

/// An outline: its strokes end to end, each the last one's end on.
type Outline = Vec<Seg>;

/// The outline's area, signed (counter-clockwise positive): Green's `½∮(x dy − y dx)`, a line's
/// `½(x₀y₁ − x₁y₀)`, an arc's `½(R²φ + cₓ(e_y − s_y) − c_y(eₓ − sₓ))`.
fn area(o: &Outline) -> f64 {
    o.iter().map(|g| match *g {
        Seg::Line(a, b) => 0.5 * (a[0] * b[1] - b[0] * a[1]),
        Seg::Arc { c, s, e, .. } => {
            let r2 = (s[0] - c[0]).powi(2) + (s[1] - c[1]).powi(2);
            0.5 * (r2 * g.sweep() + c[0] * (e[1] - s[1]) - c[1] * (e[0] - s[0]))
        }
    }).sum()
}

/// A rim's section: its area, and how far in from each face its centroid stands.
#[derive(Clone, Copy)]
enum Word {
    Round(f64),
    Chamfer(f64),
}

impl Word {
    fn section(self) -> (f64, f64) {
        match self {
            Word::Round(r) => (square_run(r), (10.0 - 3.0 * PI) / (3.0 * (4.0 - PI)) * r),
            Word::Chamfer(d) => (0.5 * d * d, d / 3.0),
        }
    }

    fn text(self) -> String {
        match self {
            Word::Round(r) => format!("round(r: {r}mm)"),
            Word::Chamfer(d) => format!("chamfer(d: {d}mm)"),
        }
    }
}

/// What the rim takes, the outline tangent all the way round: the section along each line, and
/// round each arc its centroid turned about the arc's axis (Pappus) — `R − inset` where the arc
/// is convex (it turns the way the outline does), `R + inset` where it is concave.
fn taken(o: &Outline, word: Word) -> f64 {
    let (a, inset) = word.section();
    let way = area(o).signum();
    a * o.iter().map(|g| match *g {
        Seg::Line(p, q) => (q[0] - p[0]).hypot(q[1] - p[1]),
        Seg::Arc { c, s, .. } => {
            let radius = (s[0] - c[0]).hypot(s[1] - c[1]);
            let phi = g.sweep();
            let rho = if phi * way > 0.0 { radius - inset } else { radius + inset };
            rho * phi.abs()
        }
    }).sum::<f64>()
}

/// Outline `o` as an author states it, to a millionth of a millimetre: each stroke's ends rounded
/// to the micron grid, and each arc's centre one coordinate of it so rounded too (its x where its
/// chord runs more along y, else its y) and the other where the arc's ends are as far from it,
/// as the solve puts it — so each seam is tangent to the numbers, a ten-millionth of a turn or so
/// off, as #962's 132 and 142 were drawn.
fn stated(o: &Outline) -> Outline {
    let grid = |x: f64| (x * 1e6).round() / 1e6;
    let at = |p: [f64; 2]| [grid(p[0]), grid(p[1])];
    o.iter().map(|g| match *g {
        Seg::Line(a, b) => Seg::Line(at(a), at(b)),
        Seg::Arc { c, s, e, ccw } => {
            let (s, e) = (at(s), at(e));
            let m = [0.5 * (s[0] + e[0]), 0.5 * (s[1] + e[1])];
            let d = [e[0] - s[0], e[1] - s[1]];
            // on the chord's bisector, `(c − m)·d = 0`
            let c = if d[1].abs() > d[0].abs() {
                let x = grid(c[0]);
                [x, m[1] - (x - m[0]) * d[0] / d[1]]
            } else {
                let y = grid(c[1]);
                [m[0] - (y - m[1]) * d[1] / d[0], y]
            };
            Seg::Arc { c, s, e, ccw }
        }
    }).collect()
}

/// The plate drawn on the top plane through outline `o`, 10 thick, and its rim on `cap` (`near`,
/// z = 10, or `far`, z = 0) taken by `word`: every end fixed, and each arc's centre by the
/// coordinate `stated` rounds.
fn document(o: &Outline, cap: &str, word: Word) -> String {
    let mut body = String::new();
    let mut names = Vec::new();
    let point = |body: &mut String, name: &str, p: [f64; 2]| {
        body.push_str(&format!("  {name} := point\n  fix(({:.17}, {:.17})) {name}\n", p[0], p[1]));
    };
    for (k, g) in o.iter().enumerate() {
        point(&mut body, &format!("v{k}"), g.ends().0);
    }
    for (k, g) in o.iter().enumerate() {
        let (from, to) = (format!("v{k}"), format!("v{}", (k + 1) % o.len()));
        let name = format!("k{k}");
        match *g {
            Seg::Line(..) => body.push_str(&format!("  {name} := line({from}, {to})\n")),
            Seg::Arc { c, s, e, ccw } => {
                let along_y = (e[1] - s[1]).abs() > (e[0] - s[0]).abs();
                let (axis, value) = if along_y { ("x", c[0]) } else { ("y", c[1]) };
                body.push_str(&format!("  c{k} := point hint(({:.12}, {:.12}))\n  fix({axis} == {value:.12}mm) c{k}\n",
                    c[0], c[1]));
                let sense = if ccw { "" } else { ", sense: cw" };
                body.push_str(&format!("  {name} := arc(center: c{k}, start: {from}, end: {to}{sense})\n"));
            }
        }
        names.push(name);
    }
    format!("unit mm\nuse std\nuse std.solids\nuse std.rounds (round, chamfer)\nin std.top {{\n{body}  \
             outline := face({})\n}}\nplate := solid(std.solids.Prism(outline, from: 0mm, to: 10mm))\n\
             plate.{cap} {} plate\n", names.join(", "), word.text())
}

/// The plate's volume less its rim, drawn as an author states it (`stated`), built exactly and
/// held to the closed form of what is drawn.
fn holds(o: &Outline, cap: &str, word: Word, export: bool) { drawn(&stated(o), cap, word, export) }

/// The plate's volume less its rim, drawn through `o` to the last digit, built exactly and held to
/// the closed form.
fn drawn(o: &Outline, cap: &str, word: Word, export: bool) {
    let src = document(o, cap, word);
    let e = solved(&src);
    assert!(e.diags.is_empty(), "{:?}\n{src}", e.diags);
    let b = if export { exported(&e, "plate") } else { exact(&e, "plate") };
    close(props::volume(&b), area(o).abs() * 10.0 - taken(o, word));
}

/// **The step of #962's 117**: along the bottom from the left, a concave arc of `r1` turning down
/// into a convex lobe of 12 about (−12, 0), which runs up tangent into the line `x = 0` at the
/// origin; the line `length` long, running tangent into a concave arc of 8 that turns along the
/// plate's lower edge to the right; then the plate's corners each a quarter circle of 10.  The
/// bottom stands 4.25 below the lobe's centre, so the concave arc's centre stands where it is
/// `r1 + 12` from the lobe's and `r1` below the bottom.  `length` nothing, the lobe runs straight
/// into the arc of 8.
fn step(r1: f64, length: f64) -> Outline {
    let (r2, r3, rc) = (12.0, 8.0, 10.0);
    let lobe = [-r2, 0.0];
    let yb = -4.25;
    let c1 = [lobe[0] - ((r1 + r2).powi(2) - (yb - r1).powi(2)).sqrt(), yb - r1];
    let t = r1 / (r1 + r2);
    let v10 = [c1[0] + t * (lobe[0] - c1[0]), c1[1] + t * (lobe[1] - c1[1])];
    let (right, top, left) = (40.0, 50.0, -70.0);
    let lower = length + r3;
    let mut o = vec![
        Seg::Line([left, yb], [c1[0], yb]),
        Seg::Arc { c: c1, s: [c1[0], yb], e: v10, ccw: false },
        Seg::Arc { c: lobe, s: v10, e: [0.0, 0.0], ccw: true },
    ];
    if length > 0.0 {
        o.push(Seg::Line([0.0, 0.0], [0.0, length]));
    }
    o.extend([
        Seg::Arc { c: [r3, length], s: [0.0, length], e: [r3, lower], ccw: false },
        Seg::Line([r3, lower], [right - rc, lower]),
        Seg::Arc { c: [right - rc, lower + rc], s: [right - rc, lower], e: [right, lower + rc], ccw: true },
        Seg::Line([right, lower + rc], [right, top - rc]),
        Seg::Arc { c: [right - rc, top - rc], s: [right, top - rc], e: [right - rc, top], ccw: true },
        Seg::Line([right - rc, top], [left + rc, top]),
        Seg::Arc { c: [left + rc, top - rc], s: [left + rc, top], e: [left, top - rc], ccw: true },
        Seg::Line([left, top - rc], [left, yb + rc]),
        Seg::Arc { c: [left + rc, yb + rc], s: [left, yb + rc], e: [left + rc, yb], ccw: true },
    ]);
    // (the bottom line from the last corner on)
    o[0] = Seg::Line([left + rc, yb], [c1[0], yb]);
    o
}

/// Outline `o` mirrored across the y axis and turned by `turn` about the origin, each stroke in
/// the order it is met going round the same way as before (mirrored, the other way).
fn moved(o: &Outline, mirror: bool, turn: f64) -> Outline {
    let (s, c) = turn.sin_cos();
    let f = move |p: [f64; 2]| {
        let x = if mirror { -p[0] } else { p[0] };
        [c * x - s * p[1], s * x + c * p[1]]
    };
    o.iter().map(|g| g.mapped(&f, mirror)).collect()
}

/// **The step's rim, at every line length and radius** (#962's 117): the line between the lobe
/// and the arc of 8 13.75 long, 1.5 (shorter than the round), and none (the lobe straight into
/// the arc); the round 1, 2, 4.9 and 5.5 (past the concave arc's 5, which a ring outside its
/// cylinder holds at any radius); the chamfer; the top rim and the bottom; mirrored and turned.
/// Refused before: the run along the line, its direction a ten-millionth off the lobe's tangent
/// where the drawing's numbers left it, was glued to neither arc and unioned with both.
#[test]
fn a_step_of_tangent_arcs_and_a_line_is_rimmed_whole() {
    for length in [13.75, 1.5, 0.0] {
        for word in [Word::Round(2.0), Word::Chamfer(1.5)] {
            holds(&step(5.0, length), "near", word, length == 13.75);
        }
    }
    for r in [1.0, 4.9, 5.5] {
        holds(&step(5.0, 13.75), "near", Word::Round(r), false);
    }
    holds(&step(5.0, 1.5), "far", Word::Round(2.0), false);
    holds(&moved(&step(5.0, 13.75), true, 0.0), "near", Word::Round(2.0), false);
    holds(&moved(&step(5.0, 1.5), false, 0.7), "far", Word::Chamfer(1.0), false);
}

/// Lobes of `rl` about `n` centres `d` from the origin, the first on the x axis, and between each
/// two a concave waist of `rw` tangent to both, its centre on the bisector between them: `2n`
/// arcs, every seam tangent.
fn lobed(n: usize, d: f64, rl: f64, rw: f64) -> Outline {
    let half = PI / n as f64;
    // the waist's centre: `rl + rw` from both lobes' on the bisector
    let e = d * half.cos() + ((rl + rw).powi(2) - (d * half.sin()).powi(2)).sqrt();
    let at = |r: f64, a: f64| [r * a.cos(), r * a.sin()];
    let mut o = Vec::new();
    for k in 0..n {
        let (a, b) = (2.0 * half * k as f64, 2.0 * half * (k + 1) as f64);
        let (lobe, waist, next) = (at(d, a), at(e, a + half), at(d, b));
        let touch = |l: [f64; 2]| {
            let t = rl / (rl + rw);
            [l[0] + t * (waist[0] - l[0]), l[1] + t * (waist[1] - l[1])]
        };
        let (into, out) = (touch(lobe), touch(next));
        // the lobe from where the last waist left it to where this waist meets it
        let prev = at(e, a - half);
        let from = [lobe[0] + rl / (rl + rw) * (prev[0] - lobe[0]), lobe[1] + rl / (rl + rw) * (prev[1] - lobe[1])];
        o.push(Seg::Arc { c: lobe, s: from, e: into, ccw: true });
        o.push(Seg::Arc { c: waist, s: into, e: out, ccw: false });
    }
    o
}

/// **A plate of alternating lobes and waists** (#962's 132): two, three and six lobes, every seam
/// a convex arc meeting a concave one tangent; rounded and chamfered, on either cap, mirrored and
/// turned.  Refused before: each ring's caps lay in planes through the seam whose normals agree
/// to the rounding of the arcs' centres, short of the glue's reading of one plane, and two
/// stretches joined at the seam shared no face.
#[test]
fn a_plate_of_alternating_lobes_and_waists_is_rimmed_whole() {
    let plates = [lobed(2, 30.0, 15.0, 40.0), lobed(3, 45.0, 10.0, 40.0), lobed(6, 45.0, 10.0, 40.0)];
    for (k, o) in plates.iter().enumerate() {
        holds(o, "near", Word::Round(1.0), k == 2);
        holds(o, "far", Word::Chamfer(1.0), false);
    }
    holds(&moved(&plates[2], true, 0.3), "near", Word::Round(2.0), false);
    holds(&moved(&plates[1], false, 1.1), "far", Word::Round(3.0), false);
}

/// The step with the lobe's end turned `eps` on round its circle past the origin, so the line
/// from there to `(0, length)` leaves it `eps` off its tangent, and the arc of 8's start turned
/// `eps` on too: two seams the body reads as tangent that are not, to the drawing's numbers.
fn creased(eps: f64) -> Outline { creased_at(eps, eps) }

/// The step with the lobe's end turned `lobe_eps` on round its circle past the origin and the arc
/// of 8's start `root_eps` on round its own (either negative: turned back short of it).
fn creased_at(lobe_eps: f64, root_eps: f64) -> Outline {
    let mut o = step(5.0, 13.75);
    let lobe = [-12.0, 0.0];
    let v11 = [lobe[0] + 12.0 * lobe_eps.cos(), lobe[1] + 12.0 * lobe_eps.sin()];
    let c3 = [8.0, 13.75];
    let v12 = [c3[0] - 8.0 * root_eps.cos(), c3[1] + 8.0 * root_eps.sin()];
    for g in &mut o {
        match g {
            Seg::Arc { c, e, .. } if *c == lobe => *e = v11,
            Seg::Line(a, b) if *a == [0.0, 0.0] => (*a, *b) = (v11, v12),
            Seg::Arc { c, s, .. } if *c == c3 => *s = v12,
            _ => {}
        }
    }
    o
}

/// **Seams tangent to the drawing's numbers** (#962's 117 and 142): the line `eps` off the lobe's
/// tangent and the arc of 8's, at `1e-8`, `1e-7` and `5e-7` — all under the body's reading of
/// tangent — the round and the chamfer each the closed form of what is drawn.
#[test]
fn seams_tangent_to_the_drawings_numbers_are_glued() {
    for eps in [1e-8, 1e-7, 5e-7] {
        drawn(&creased(eps), "near", Word::Round(2.0), eps == 5e-7);
        drawn(&creased(eps), "near", Word::Chamfer(1.5), false);
    }
}

/// **`rounds/step.sv`, #962's 117 as its author drew it**: every number stated to five places, so
/// each seam is tangent to them alone — the lobe meets the arc of 5 half a millionth of a radian
/// off, the side the lobe and the arc of 8 a ten-millionth off.  Refused before: "a piece of `end`
/// (a plane) touches `near` (a plane) … along no face of either", the round's own construction
/// planes, which no document names.  The plate less its rim: the runs and the stretches the closed
/// form's, each convex corner of the frame the stack's mitre (`(5/3 − π/2) r³` at a right angle),
/// what is drawn read as the solve places it — the lobe's end where it meets the arc of 5, its
/// radius to there, and the side's foot where the lobe reaches `y = −55`.
#[test]
fn the_step_as_its_author_drew_it_is_rimmed() {
    let e = solved(include_str!("../../examples/rounds/step.sv"));
    assert!(e.diags.is_empty(), "{:?}", e.diags);
    let (c9, c10, c12) = ([-57.26315, -64.25], [-43.0, -55.0], [-23.0, -41.25]);
    let v10 = [c9[0] + (25.0f64 - (-61.52941f64 - c9[1]).powi(2)).sqrt(), -61.52941];
    let lobe = (v10[0] - c10[0]).hypot(v10[1] - c10[1]);
    let v11 = [c10[0] + lobe, -55.0];
    let (v9, v12, v13) = ([-57.26315, -59.25], [-31.0, -41.25], [-23.0, -33.25]);
    let (a, east, north, west) = ([-125.0, -59.25], [105.0, -33.25], [105.0, 72.0], [-125.0, 72.0]);
    let o = vec![
        Seg::Line(a, v9),
        Seg::Arc { c: c9, s: v9, e: v10, ccw: false },
        Seg::Arc { c: c10, s: v10, e: v11, ccw: true },
        Seg::Line(v11, v12),
        Seg::Arc { c: c12, s: v12, e: v13, ccw: false },
        Seg::Line(v13, east),
        Seg::Line(east, north),
        Seg::Line(north, west),
        Seg::Line(west, a),
    ];
    let r = 2.0;
    let mitres = 4.0 * (5.0 / 3.0 - PI / 2.0) * r * r * r;
    let b = exported(&e, "deck");
    close(props::volume(&b), area(&o).abs() * 30.0 - (taken(&o, Word::Round(r)) - mitres));
}

/// #962's 127: an obround flange 21 thick, its half circles of 99.29 about (±384.62, 10.55), and
/// a boss of 46.76 about `(x, -67.9)` as thick, their tops flush, unioned; the flange's top rim
/// rounded 10 over the solid they leave.
fn flange_and_boss(x: f64) -> String {
    format!("use std\nuse std.solids (cut, union, common)\nuse std.rounds (round)\nin std.top {{\n  \
             cr := point\n  fix((384.62, 10.55)) cr\n  cl := point\n  fix((-384.62, 10.55)) cl\n  \
             a := point\n  fix((384.62, -88.74)) a\n  b := point\n  fix((384.62, 109.84)) b\n  \
             c := point\n  fix((-384.62, 109.84)) c\n  d := point\n  fix((-384.62, -88.74)) d\n  \
             ar := arc(center: cr, start: a, end: b)\n  al := arc(center: cl, start: c, end: d)\n  \
             t := line(b, c)\n  s := line(d, a)\n  outline := face(ar, t, al, s)\n  \
             o := point\n  fix(({x}, -67.9)) o\n  disc := circle(center: o) hint(r: 46.76)\n  \
             radius(46.76mm) disc\n}}\n\
             flange := solid(std.solids.Prism(outline, from: -152.5mm, to: -131.5mm))\n\
             boss := solid(std.solids.Prism(face(disc), from: -152.5mm, to: -131.5mm))\n\
             outer := solid(flange)\nboss union outer\npart := solid(outer)\n\
             part.outer.flange.near round(r: 10mm) part.outer.flange\n")
}


/// What the flange less its rim and the boss weigh, the boss at `x` clear of both half circles:
/// the obround and the boss's segment below the flange's lower side, 21 thick; less the run along
/// the upper side, a stretch of a ring round each half circle (Pappus), and the run along the
/// lower side less what of it the boss's cylinder keeps it out of — at each depth `u` in from the
/// side, the section's depth there `r − √(r² − (r − u)²)` times the boss's chord at `u`,
/// integrated in `u = r(1 − cos t)`, smooth in `t` (Simpson's rule over 2000 steps).
fn flange_and_boss_less_rim() -> f64 {
    let (half, big, r, rb, below): (f64, f64, f64, f64, f64) = (384.62, 99.29, 10.0, 46.76, 20.84);
    let obround = 2.0 * half * 2.0 * big + PI * big * big;
    let segment = rb * rb * (below / rb).acos() - below * (rb * rb - below * below).sqrt();
    let (a, inset) = Word::Round(r).section();
    let chord = |t: f64| {
        let u = r * (1.0 - t.cos());
        let depth = r * (1.0 - t.sin());
        depth * 2.0 * (rb * rb - (u - below).powi(2)).sqrt() * r * t.sin()
    };
    let n = 2000;
    let step = 0.5 * PI / n as f64;
    let kept_out = step / 3.0 * (0..=n).map(|k| {
        let w = if k == 0 || k == n { 1.0 } else if k % 2 == 1 { 4.0 } else { 2.0 };
        w * chord(k as f64 * step)
    }).sum::<f64>();
    let taken = a * (2.0 * 2.0 * half + 2.0 * PI * (big - inset)) - kept_out;
    (obround + segment) * 21.0 - taken
}

/// **A flush boss slid along the rim through the tangent seam** (#962's 127): the round stops on
/// the boss at both ends of the stretch of the lower side it leaves, each run kept outside the
/// boss's cylinder.  Clear of the section where the side runs tangent into the half circle (the
/// boss at −300 and −335), the closed form's to 1e-9; the boss over the seam itself (−343, −360,
/// −380), the side's stretch and the seam gone, built.  Between (−339.17, #962's own, and −341),
/// the boss's side reaches back across that section, and the half circle's ring — its chain going
/// on across the seam into the side's stretch, which ends on the boss — is kept outside the boss
/// with it (`BodyFaces::chain_ends`, #994): built, the two glued at a section the boss cuts alike
/// (refused before, "Move `outer.boss.disc` clear of the seam, or over it"; its volumes are held
/// in `tests/bosses_on_seams.rs`).
#[test]
fn a_flush_boss_slid_through_the_seam_stops_the_round() {
    let built = |x: f64| {
        let e = solved(&flange_and_boss(x));
        assert!(e.diags.is_empty(), "{x}: {:?}", e.diags);
        props::volume(&exported(&e, "part"))
    };
    for x in [-300.0, -335.0] {
        close(built(x), flange_and_boss_less_rim());
    }
    // (the boss reaching back over the seam's section takes from the half circle's ring too: less
    // than the closed form clear of it, which takes the ring whole)
    for x in [-339.17, -341.0] {
        assert!(built(x) > flange_and_boss_less_rim(), "{x}");
    }
    for x in [-343.0, -360.0, -380.0] {
        built(x);
    }
}

/// The export's refusal of solid `name` of document `src`, as text.
fn refused(src: &str, name: &str) -> String {
    let e = solved(src);
    let say = gcs_core::brep::sweep::Say { stage: &|_: &str| {}, mark: &|_| {} };
    let body = crate::common::ent(&e, name).i();
    gcs_core::brep::export::exact(&e.sketch, body, None, None, &say).err().expect("refused").message
}

/// Whether solid `name` of document `src` is built exactly.
fn builds(src: &str, name: &str) -> bool {
    let e = solved(src);
    let say = gcs_core::brep::sweep::Say { stage: &|_: &str| {}, mark: &|_| {} };
    gcs_core::brep::export::exact(&e.sketch, crate::common::ent(&e, name).i(), None, None, &say).is_ok()
}

/// **A corner too nearly tangent to round, said by its faces** (#962): the step's side turned
/// `eps` off the lobe of 12 it leaves, a reflex corner — inside the body's reading of tangent
/// (`5e-7`) a seam the round goes on across, built; past it a corner, whose round meets the lobe
/// again `2R sin θ` on round it, which the kernel tells apart from the corner only where `sin² θ`
/// is over `ON_CURVE tol / R` (here `2.9e-4` rad: `tol` a billionth of the plate's diagonal of
/// 124.5): at `2e-6`, `1e-5`, `1e-4` and `2e-4` refused, naming the side `k3` and the lobe `k2`, the
/// turn, the bound, and the remedy (before, "a cylinder's pieces do not close"); at `3e-4` and
/// `1e-3` built as a corner, at a round of 2 and of 5.  A chamfer's flat crosses the side, and is
/// built at `1e-5`; so is the side turned off the concave arc of 8 it runs into.
#[test]
fn a_corner_too_nearly_tangent_to_round_is_said_by_its_faces() {
    let round = |eps: f64, r: f64| document(&creased_at(eps, 0.0), "near", Word::Round(r));
    assert!(builds(&round(5e-7, 2.0), "plate"));
    for (eps, turn) in [(2e-6, "2e-06"), (1e-5, "1e-05"), (1e-4, "0.0001"), (2e-4, "0.0002")] {
        let refusal = refused(&round(eps, 2.0), "plate");
        // (the witness, the lobe's end, to the tenth of a micron under the kernel's tolerance)
        let at = |x: f64| gcs_core::json::fmt_g((x / 1e-7).round() * 1e-7, 6);
        let said = format!("`k3` and `k2` meet at a turn of {turn} rad at [{}, {}, 10]mm: not tangent, so their \
                            edge is a corner and no seam a round goes on across, and too nearly tangent a corner \
                            for this kernel to round at 2mm against `k2`'s radius of 12mm",
                           at(12.0 * eps.cos() - 12.0), at(12.0 * eps.sin()));
        assert!(refusal.contains(&said), "{eps}: {refusal}");
        assert!(refusal.contains("(a turn of 0.00029 rad or more is built)"), "{refusal}");
        assert!(refusal.contains("State the tangency (`k3` tangent `k2`, or a chain's `-> tangent` joint)"), "{refusal}");
    }
    for r in [2.0, 5.0] {
        for eps in [3e-4, 1e-3] {
            assert!(builds(&round(eps, r), "plate"), "{eps} at {r}");
        }
    }
    assert!(builds(&document(&creased_at(1e-5, 0.0), "near", Word::Chamfer(1.5)), "plate"));
    assert!(builds(&document(&creased_at(0.0, 1e-5), "near", Word::Round(2.0)), "plate"));
}

/// **A body grown over seams tangent to the drawing's numbers** (#962's 132, `std.rounds.Grown`):
/// a vertex where two faces go on from one another across a seam is no corner — two of its faces'
/// normals one to the body's reading of tangent — so no ball's sector stands there, and the two
/// faces' prisms meet in the seam's section.  Read to a billionth, the normals of a lobe and a
/// waist met tangent only to the stated numbers were two, a corner's sector was cut at a seam, and
/// "a grown body's piece shares no face with the rest".  The plate of twelve arcs and the step,
/// each stated, grown 1: Steiner's `(A + Pd + πd²)h + 2(Ad + πPd²/4 + 2πd³/3)` over the outline's
/// area `A` and perimeter `P`, its reach past `d` everywhere.
#[test]
fn a_body_grown_over_stated_seams_stands_no_corner_there() {
    for o in [stated(&lobed(6, 45.0, 10.0, 40.0)), stated(&step(5.0, 13.75))] {
        let src = document(&o, "near", Word::Round(1.0))
            .replace("plate.near round(r: 1mm) plate\n", "g := std.rounds.Grown(plate, d: 1mm)\n");
        let e = solved(&src);
        assert!(e.diags.is_empty(), "{:?}", e.diags);
        let (a, d, h) = (area(&o).abs(), 1.0, 10.0);
        let p: f64 = o.iter().map(|g| match *g {
            Seg::Line(p, q) => (q[0] - p[0]).hypot(q[1] - p[1]),
            Seg::Arc { c, s, .. } => (s[0] - c[0]).hypot(s[1] - c[1]) * g.sweep().abs(),
        }).sum();
        let want = (a + p * d + PI * d * d) * h + 2.0 * (a * d + p * PI * d * d / 4.0 + 2.0 * PI * d * d * d / 3.0);
        close(props::volume(&exact(&e, "g.body")), want);
    }
}

