//! **A run that ends where its end face does not cut its whole section off** (#962 item 3).  What
//! `std.rounds` says of a run's end is `crease.ends` and the meeting: the run is carried past its
//! edge's end, cut off by the faces it ends on, and read against the meeting — the faces'
//! surfaces' meeting within those faces — wherever the crease is the nearest of it.  Round a
//! circle that meeting comes back past a flat (a disc's rim, the next arc of its circle), so a
//! ring's stretch is carried round to the meridian halfway to where it comes back, kept within the
//! flat, whatever of its section the flat leaves: the disc with flats of #962's 149, its rim
//! chamfered or rounded deeper than the flat stands in from the circle.  Each family drawn whole,
//! each volume its closed form's to 1e-9: at each depth under the cap the section the rim leaves
//! is the disc shrunk by the round's inset there, cut by each flat moved in as far, less the bore
//! grown as far with each key shrunk as far, integrated over the depth (Gauss–Legendre on a
//! smooth parametrisation).
use gcs_core::brep::props;
use std::f64::consts::PI;

use crate::common::rounds::{exact, exported, gauss_legendre};
use crate::common::{close, text::solved};

/// The disc's thickness: drawn from −12.5 to 12.5.
const THICK: f64 = 25.0;

/// A round or a chamfer of the rim, of radius or legs its number.
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

    /// `∫₀ᴰ F(g(u)) du` over the depth `u` under the cap, `g` how far in from the side the word
    /// takes at that depth: a chamfer's `d − u`; a round's `r − √(r² − (r − u)²)`, which in
    /// `u = r(1 − sin t)` is `r(1 − cos t)` with `du = r cos t dt`, smooth.
    fn over_depth(self, f: &dyn Fn(f64) -> f64) -> f64 {
        let nodes = gauss_legendre(60);
        let on = |a: f64, b: f64, h: &dyn Fn(f64) -> f64| {
            nodes.iter().map(|&(x, w)| 0.5 * (b - a) * w * h(0.5 * (a + b) + 0.5 * (b - a) * x)).sum::<f64>()
        };
        match self {
            Word::Chamfer(d) => on(0.0, d, &|g| f(g)),
            Word::Round(r) => on(0.0, 0.5 * PI, &|t| f(r * (1.0 - t.cos())) * r * t.cos()),
        }
    }
}

/// A disc of radius `big` about the origin, a flat at each `(angle, distance)` of `flats`, a bore
/// of `bore` with a key at each angle of `keys` (each `half` either side of its line, its tip
/// `tip` from the centre).
#[derive(Clone, Debug)]
struct Disc {
    big: f64,
    flats: Vec<(f64, f64)>,
    bore: Option<f64>,
    keys: Vec<f64>,
    half: f64,
    tip: f64,
}

/// `(x, y)` turned to `angle`.
fn at(angle: f64, x: f64, y: f64) -> [f64; 2] {
    let (s, c) = angle.sin_cos();
    [c * x - s * y, s * x + c * y]
}

/// The area of the disc of `rho` about the origin less what lies past the line `distance` out at
/// `angle` (a segment): `ρ² acos(b/ρ) − b√(ρ² − b²)`.
fn segment(rho: f64, b: f64) -> f64 { rho * rho * (b / rho).acos() - b * (rho * rho - b * b).sqrt() }

/// The area of the disc of `rho` within `half` of a line through its centre, from `from` out along
/// it: `c√(ρ² − c²) + ρ² asin(c/ρ) − 2c·from`.
fn strip(rho: f64, half: f64, from: f64) -> f64 {
    half * (rho * rho - half * half).sqrt() + rho * rho * (half / rho).asin() - 2.0 * half * from
}

impl Disc {
    /// The document: each point fixed, each arc's centre on the axis its chord's bisector crosses
    /// at the origin (one coordinate fixed), the bore a circle, the cap `cap` taken by `word`.
    fn document(&self, cap: &str, word: Word) -> String {
        let mut body = String::new();
        let mut names = Vec::new();
        let mut n = 0;
        let mut point = |body: &mut String, p: [f64; 2]| {
            n += 1;
            body.push_str(&format!("  p{n} := point\n  fix(({:.17}, {:.17})) p{n}\n", p[0], p[1]));
            format!("p{n}")
        };
        let mut arcs = 0;
        let mut arc = |body: &mut String, s: (&str, [f64; 2]), e: (&str, [f64; 2])| {
            arcs += 1;
            let d = [e.1[0] - s.1[0], e.1[1] - s.1[1]];
            let axis = if d[1].abs() > d[0].abs() { "x" } else { "y" };
            body.push_str(&format!("  c{arcs} := point hint((0, 0))\n  fix({axis} == 0mm) c{arcs}\n  \
                a{arcs} := arc(center: c{arcs}, start: {}, end: {})\n", s.0, e.0));
            format!("a{arcs}")
        };
        // the outline: each flat, then the arc on to the next
        let ends: Vec<([f64; 2], [f64; 2])> = self.flats.iter().map(|&(angle, a)| {
            let h = (self.big * self.big - a * a).sqrt();
            (at(angle, a, -h), at(angle, a, h))
        }).collect();
        let named: Vec<(String, String)> = ends.iter().map(|&(s, e)| (point(&mut body, s), point(&mut body, e))).collect();
        for k in 0..ends.len() {
            let next = (k + 1) % ends.len();
            body.push_str(&format!("  f{k} := line({}, {})\n", named[k].0, named[k].1));
            names.push(format!("f{k}"));
            names.push(arc(&mut body, (&named[k].1, ends[k].1), (&named[next].0, ends[next].0)));
        }
        let holes = match self.bore {
            None => String::new(),
            Some(rb) if self.keys.is_empty() => {
                body.push_str(&format!("  bc := point\n  fix((0, 0)) bc\n  rim := circle(center: bc) hint(r: {rb})\n  \
                    radius({rb}mm) rim\n  bore := face(rim)\n"));
                ", holes: bore".to_string()
            }
            Some(rb) => {
                let foot = (rb * rb - self.half * self.half).sqrt();
                let mut loop_: Vec<String> = Vec::new();
                let keys: Vec<[([f64; 2], String); 4]> = self.keys.iter().map(|&angle| {
                    [(foot, -self.half), (self.tip, -self.half), (self.tip, self.half), (foot, self.half)]
                        .map(|(x, y)| { let p = at(angle, x, y); (p, point(&mut body, p)) })
                }).collect();
                for (j, key) in keys.iter().enumerate() {
                    for s in 0..3 {
                        body.push_str(&format!("  k{j}_{s} := line({}, {})\n", key[s].1, key[s + 1].1));
                        loop_.push(format!("k{j}_{s}"));
                    }
                    let next = &keys[(j + 1) % keys.len()][0];
                    loop_.push(arc(&mut body, (&key[3].1, key[3].0), (&next.1, next.0)));
                }
                body.push_str(&format!("  bore := face({})\n", loop_.join(", ")));
                ", holes: bore".to_string()
            }
        };
        format!("unit mm\nuse std\nuse std.solids\nuse std.rounds (round, chamfer)\nin std.top {{\n{body}  \
                 outline := face({}{holes})\n}}\ndisc := solid(std.solids.Prism(outline, from: -12.5mm, to: 12.5mm))\n\
                 part := solid(disc)\npart.disc.{cap} {} part.disc\n", names.join(", "), word.text())
    }

    /// The section the rim leaves `g` in from the sides: the disc shrunk by `g` and cut by each flat
    /// moved in by `g`, less the bore grown by `g` and each key shrunk by `g` along its sides and
    /// its tip, the bore's ring kept off each key's sides — the notch where a key's side meets the
    /// bore left standing (a concave corner).
    fn section(&self, g: f64) -> f64 {
        let rho = self.big - g;
        let outer = PI * rho * rho - self.flats.iter().map(|&(_, a)| segment(rho, a - g)).sum::<f64>();
        let hole = match self.bore {
            None => 0.0,
            Some(rb) => {
                let (m, c) = (self.keys.len(), self.half);
                let grown = rb + g;
                // each side of a key: the notch the bore's ring leaves beside it — kept within the
                // side's plane, the half of the strip between the key's lines, where the arc beside
                // it turns less than half round; ending square at the key's meridian otherwise, the
                // side's plane crossing the arc (no face it ends on), the half of the sector between
                // the meridians through the key's feet
                let mut keys = self.keys.clone();
                keys.sort_by(|x, y| x.rem_euclid(2.0 * PI).total_cmp(&y.rem_euclid(2.0 * PI)));
                let notch = (0..m).map(|j| {
                    let gaps = [(keys[j] - keys[(j + m - 1) % m]), (keys[(j + 1) % m] - keys[j])]
                        .map(|t| if m == 1 { 2.0 * PI } else { t.rem_euclid(2.0 * PI) });
                    gaps.iter().map(|&gap| if gap > PI {
                        0.5 * (grown * grown - rb * rb) * (c / rb).asin()
                    } else {
                        0.5 * (strip(grown, c, 0.0) - strip(rb, c, 0.0))
                    }).sum::<f64>()
                }).sum::<f64>();
                let rest = m as f64 * strip(rb, c - g, self.tip + g);
                PI * grown * grown - notch - rest
            }
        };
        outer - hole
    }

    /// The disc less its rim on one cap taken by `word`.
    fn volume(&self, word: Word) -> f64 {
        let whole = self.section(0.0);
        whole * THICK - word.over_depth(&|g| whole - self.section(g))
    }

    /// Turned by `angle` about the origin, and mirrored across the x axis where `mirror`.
    fn moved(&self, angle: f64, mirror: bool) -> Disc {
        let f = |a: f64| if mirror { -a + angle } else { a + angle };
        Disc { flats: self.flats.iter().map(|&(a, d)| (f(a), d)).collect(), keys: self.keys.iter().map(|&a| f(a)).collect(),
            ..self.clone() }
    }
}

/// The disc built exactly (exported where `export`) and held to its closed form.
fn holds(disc: &Disc, cap: &str, word: Word, export: bool) {
    // (mirrored, the outline's order turns: drawn going round the same way again)
    let mut d = disc.clone();
    d.flats.sort_by(|x, y| x.0.rem_euclid(2.0 * PI).total_cmp(&y.0.rem_euclid(2.0 * PI)));
    d.keys.sort_by(|x, y| x.rem_euclid(2.0 * PI).total_cmp(&y.rem_euclid(2.0 * PI)));
    let src = d.document(cap, word);
    let e = solved(&src);
    assert!(e.diags.is_empty(), "{:?}\n{src}", e.diags);
    let b = if export { exported(&e, "part") } else { exact(&e, "part") };
    close(props::volume(&b), d.volume(word));
}

/// #962's 149's disc: 75 in radius, `n` flats 74.5 out.
fn flats(n: usize) -> Disc {
    Disc { big: 75.0, flats: (0..n).map(|k| (2.0 * PI * k as f64 / n as f64, 74.5)).collect(), bore: None,
        keys: Vec::new(), half: 2.5, tip: 42.5 }
}

/// **A rim chamfered deeper than its flat stands in** (#962's 149): four flats half a millimetre
/// in from the circle, the chamfer 0.4 (each point of the ring's section crossing the flat, as
/// before), 0.5 (the innermost touching it) and 0.6 and 1 (the inner part of the section passing
/// it by: carried round to the meridian halfway to the next arc, kept within the flat).
#[test]
fn a_rim_chamfered_deeper_than_its_flats_stand_in_is_built() {
    for d in [0.4, 0.5, 0.6, 1.0] {
        holds(&flats(4), "far", Word::Chamfer(d), d == 1.0);
    }
    holds(&flats(4), "near", Word::Round(1.0), false);
    holds(&flats(4).moved(0.3, true), "far", Word::Round(2.0), false);
}

/// **One and two flats** (a D, and #962's 149 with two): each stretch carried round to the
/// meridian halfway across what lies between it and the next — round the D's whole turn, from
/// both ends of its one arc to the meridian square to the flat, kept within it.
#[test]
fn a_rim_past_one_or_two_flats_is_built() {
    for n in [1, 2] {
        holds(&flats(n), "far", Word::Chamfer(1.0), false);
        holds(&flats(n).moved(0.7, false), "near", Word::Round(1.5), false);
    }
}

/// **A keyed bore beside the flats** (#962's 149 whole): a bore of 47.5 with four keys 5 wide to 42.5
/// from the centre, and with one, its rim chamfered 1 and rounded 1 with the disc's: each stretch of
/// the bore ending on a key's side (the section crossing it, as before), the key's runs kept within
/// the bore's cylinder, the notch where a key's side meets the bore left standing.
#[test]
fn a_keyed_bore_and_flats_are_rimmed_together() {
    let keyed = |keys: Vec<f64>| Disc { bore: Some(47.5), keys, ..flats(4) };
    holds(&keyed(vec![0.0, 0.5 * PI, PI, 1.5 * PI]), "far", Word::Chamfer(1.0), true);
    holds(&keyed(vec![0.0, 0.5 * PI, PI, 1.5 * PI]), "near", Word::Round(1.0), false);
    holds(&keyed(vec![0.25 * PI]), "far", Word::Chamfer(1.0), false);
    holds(&Disc { bore: Some(47.5), ..flats(2) }, "far", Word::Chamfer(1.0), false);
}

/// `∫ₐᵇ f` by the double-exponential rule (tanh–sinh), whose nodes crowd the ends: to the last few
/// digits for an `f` smooth inside and with square-root behaviour at either end (a chord through a
/// circle, a round's section, at their ends).
fn tanh_sinh(f: &dyn Fn(f64) -> f64, a: f64, b: f64) -> f64 {
    let (h, n) = (1.0 / 64.0, 6 * 64);
    let (mid, half) = (0.5 * (a + b), 0.5 * (b - a));
    (-n..=n).map(|k| {
        let t = k as f64 * h;
        let u = 0.5 * PI * t.sinh();
        let (x, w) = (u.tanh(), 0.5 * PI * t.cosh() / u.cosh().powi(2));
        if w < 1e-300 { 0.0 } else { h * w * half * f(mid + half * x) }
    }).sum()
}

/// `∫ₐᵇ f` by `tanh_sinh` over the stretches between `breaks` (where `f` has square-root
/// behaviour), each break within `(a, b)`.
fn split(f: &dyn Fn(f64) -> f64, a: f64, b: f64, breaks: &[f64]) -> f64 {
    let mut at: Vec<f64> = breaks.iter().copied().filter(|&x| x > a && x < b).collect();
    at.sort_by(f64::total_cmp);
    let mut points = vec![a];
    points.extend(at);
    points.push(b);
    points.windows(2).map(|w| tanh_sinh(f, w[0], w[1])).sum()
}

/// **Two discs whose rims meet at a corner**: a disc of `ra` about the origin and one of `rb` about
/// `(xb, 0)`, the plate their common part (each arc convex, meeting the other at a corner) or their
/// union (meeting at a reflex corner), its rim on `cap` taken by `word`.  Each ring's stretch ends
/// on the other's cylinder; where the other disc reaches over the first's further than the word's
/// depth, part of the stretch's section never leaves the other's solid however far it turns, and
/// the stretch is carried round to the meridian halfway to where its circle comes back within the
/// other — round its whole turn where it never does — kept within it (#962's 125).
fn lens(ra: f64, rb: f64, xb: f64, common: bool, cap: &str, word: Word) -> String {
    // where the circles cross
    let x = (xb * xb + ra * ra - rb * rb) / (2.0 * xb);
    // (`p` below `q` where the second disc stands to the right; above it, to the left)
    let y = (ra * ra - x * x).sqrt() * xb.signum();
    let arcs = if common {
        // the first disc's arc within the second (round the right, from below), then the second's
        // within the first
        format!("  ka := arc(center: ca, start: p, end: q)\n  kb := arc(center: cb, start: q, end: p)\n")
    } else {
        format!("  ka := arc(center: ca, start: q, end: p)\n  kb := arc(center: cb, start: p, end: q)\n")
    };
    format!("unit mm\nuse std\nuse std.solids\nuse std.rounds (round, chamfer)\nin std.top {{\n  \
             p := point\n  fix(({x:.17}, {:.17})) p\n  q := point\n  fix(({x:.17}, {y:.17})) q\n  \
             ca := point hint((0, 0))\n  fix(x == 0mm) ca\n  cb := point hint(({xb}, 0))\n  fix(x == {xb}mm) cb\n\
             {arcs}  outline := face(ka, kb)\n}}\n\
             disc := solid(std.solids.Prism(outline, from: -12.5mm, to: 12.5mm))\n\
             part := solid(disc)\npart.disc.{cap} {} part.disc\n", -y, word.text())
}

/// The area the discs of `r1` and `r2` with centres `d` apart have in common.
fn common_area(r1: f64, r2: f64, d: f64) -> f64 {
    if d >= r1 + r2 {
        return 0.0;
    }
    if d <= (r1 - r2).abs() {
        return PI * r1.min(r2).powi(2);
    }
    let a1 = ((d * d + r1 * r1 - r2 * r2) / (2.0 * d * r1)).acos();
    let a2 = ((d * d + r2 * r2 - r1 * r1) / (2.0 * d * r2)).acos();
    r1 * r1 * a1 + r2 * r2 * a2 - 0.5 * ((-d + r1 + r2) * (d + r1 - r2) * (d - r1 + r2) * (d + r1 + r2)).sqrt()
}

/// The section the rim of `lens` leaves `g` in from the sides: of the common part, the discs
/// shrunk by `g` in common; of the union, each disc shrunk by `g`, and their common part whole —
/// each ring kept outside the other disc leaves the reflex corner's notch.
fn lens_section(ra: f64, rb: f64, xb: f64, common: bool, g: f64) -> f64 {
    let (sa, sb, xb) = (ra - g, rb - g, xb.abs());
    if common {
        common_area(sa, sb, xb)
    } else {
        PI * (sa * sa + sb * sb) + common_area(ra, rb, xb) - common_area(sa, rb, xb) - common_area(ra, sb, xb)
    }
}

/// The lens or the union built exactly and held to its closed form.
fn lens_holds(ra: f64, rb: f64, xb: f64, common: bool, cap: &str, word: Word) {
    let src = lens(ra, rb, xb, common, cap, word);
    let e = solved(&src);
    assert!(e.diags.is_empty(), "{:?}\n{src}", e.diags);
    let whole = lens_section(ra, rb, xb, common, 0.0);
    let want = whole * THICK - word.over_depth(&|g| whole - lens_section(ra, rb, xb, common, g));
    close(props::volume(&exact(&e, "part")), want);
}

/// **A rim ending on another arc's face at a corner** (#962's 125, "a stretch of a circle its
/// curved end does not cut off"): a disc of 20 and one of 100 whose arc stands 1 in from the first
/// circle's far side, in common — the first's ring, rounded 0.5 (its section crossing the second's
/// cylinder), 1 (its innermost point touching it) and 1.5 and 3 (passing it by: the whole turn,
/// kept within the larger disc); and chamfered — and their union, each ring kept outside the other.
#[test]
fn a_rim_ending_on_another_arc_at_a_corner_is_built() {
    for r in [0.5, 0.999, 1.001, 1.5, 3.0] {
        lens_holds(20.0, 100.0, -81.0, true, "near", Word::Round(r));
    }
    lens_holds(20.0, 100.0, -81.0, true, "far", Word::Round(2.0));
    lens_holds(20.0, 100.0, 81.0, true, "near", Word::Round(1.5));
    for d in [0.5, 1.0] {
        lens_holds(20.0, 100.0, -81.0, true, "far", Word::Chamfer(d));
    }
    lens_holds(20.0, 100.0, -81.0, true, "near", Word::Chamfer(2.0));
    lens_holds(20.0, 15.0, 25.0, true, "near", Word::Round(2.0));
    lens_holds(20.0, 15.0, 25.0, false, "near", Word::Round(2.0));
    lens_holds(20.0, 15.0, 25.0, false, "far", Word::Chamfer(1.0));
}

/// The two discs' common part `height` tall standing on a flange of 70 about the origin, 10 thick,
/// and where they meet filled by `word`.
fn drum(ra: f64, rb: f64, xb: f64, word: Word) -> String {
    let x = (xb * xb + ra * ra - rb * rb) / (2.0 * xb);
    let y = (ra * ra - x * x).sqrt() * xb.signum();
    format!("unit mm\nuse std\nuse std.solids\nuse std.solids (union)\nuse std.rounds (round, chamfer)\n\
             in std.top {{\n  o := point\n  fix((0, 0)) o\n  rim := circle(center: o) hint(r: 70)\n  radius(70mm) rim\n  \
             p := point\n  fix(({x:.17}, {:.17})) p\n  q := point\n  fix(({x:.17}, {y:.17})) q\n  \
             ca := point hint((0, 0))\n  fix(x == 0mm) ca\n  cb := point hint(({xb}, 0))\n  fix(x == {xb}mm) cb\n  \
             ka := arc(center: ca, start: p, end: q)\n  kb := arc(center: cb, start: q, end: p)\n  \
             outline := face(ka, kb)\n}}\n\
             flange := solid(std.solids.Prism(face(rim), from: 0mm, to: 10mm))\n\
             drum := solid(std.solids.Prism(outline, from: 10mm, to: 40mm))\n\
             part := solid(flange)\ndrum union part\npart.drum {} part.flange.near\n", -y, word.text())
}

/// **The drum's foot filled** (#962's 125, R2 where a flange meets the drum): each arc's ring outside
/// it, ending on the other's cylinder at the drum's corners, kept within that disc — the notch
/// beyond each corner left standing.  At each height over the flange the fill is the first disc grown
/// by `g` in common with the second, and the second grown with the first, beyond the drum.
#[test]
fn a_drums_foot_filled_round_its_corners_is_built() {
    let (ra, rb) = (20.0, 100.0);
    for (xb, word) in [(-81.0, Word::Round(2.0)), (81.0, Word::Round(3.0)), (-81.0, Word::Chamfer(2.0))] {
        let src = drum(ra, rb, xb, word);
        let e = solved(&src);
        assert!(e.diags.is_empty(), "{:?}\n{src}", e.diags);
        let d = xb.abs();
        let base = common_area(ra, rb, d);
        let added = |g: f64| common_area(ra + g, rb, d) + common_area(ra, rb + g, d) - 2.0 * base;
        let want = PI * 70.0 * 70.0 * 10.0 + base * 30.0 + word.over_depth(&added);
        close(props::volume(&exact(&e, "part")), want);
    }
}

/// What cuts a pocket's floor-to-wall edge: a vertical hole through the block, `off` from the wall
/// into the pocket (negative into the wall) and `at` along it from its first end; or a trench, a
/// cylinder along y whose axis stands `below` the floor, crossing the edge `at` along it.
#[derive(Clone, Copy, Debug)]
enum Cut {
    Hole { at: f64, off: f64 },
    Trench { at: f64, below: f64 },
}

/// The block a pocket is cut into: 140 by 120, 30 tall, the pocket's floor at 10.
const FLOOR: f64 = 10.0;

/// A pocket `length` by 30 in a block, its wall `w0` along the way `turn` from the x axis through
/// `(20, −30)`, the pocket on its left; cut by `cut` (radius `rho`), or not; and the floor's edge
/// with `w0` filled by `word` where `word` is given.
fn pocket(length: f64, turn: f64, cut: Option<(Cut, f64)>, word: Option<Word>) -> String {
    let (s, c) = turn.sin_cos();
    let at = |t: f64, n: f64| [20.0 + c * t - s * n, -30.0 + s * t + c * n];
    let corners = [at(0.0, 0.0), at(length, 0.0), at(length, 30.0), at(0.0, 30.0)];
    let mut body = String::new();
    for (k, p) in corners.iter().enumerate() {
        body.push_str(&format!("  p{k} := point\n  fix(({:.17}, {:.17})) p{k}\n", p[0], p[1]));
    }
    body.push_str("  w0 := line(p0, p1)\n  w1 := line(p1, p2)\n  w2 := line(p2, p3)\n  w3 := line(p3, p0)\n");
    for (k, p) in [[-20.0, -60.0], [120.0, -60.0], [120.0, 60.0], [-20.0, 60.0]].iter().enumerate() {
        body.push_str(&format!("  b{k} := point\n  fix(({}, {})) b{k}\n", p[0], p[1]));
    }
    body.push_str("  bounds := face(b0, b1, b2, b3, -> close)\n");
    let mut rest = String::new();
    match cut {
        None => {}
        Some((Cut::Hole { at: t, off }, rho)) => {
            let [x, y] = at(t, off);
            body.push_str(&format!("  hc := point\n  fix(({x:.17}, {y:.17})) hc\n  hole := circle(center: hc) hint(r: {rho})\n  \
                radius({rho}mm) hole\n"));
            rest.push_str("drill := solid(std.solids.Prism(face(hole), from: -1mm, to: 31mm))\ndrill cut part\n");
        }
        Some((Cut::Trench { at: t, below }, rho)) => {
            let [x, _] = at(t, 0.0);
            rest.push_str(&format!("in std.front {{\n  tc := point\n  fix(({x:.17}, {:.17})) tc\n  \
                trench := circle(center: tc) hint(r: {rho})\n  radius({rho}mm) trench\n}}\n\
                drill := solid(std.solids.Prism(face(trench), from: -100mm, to: 100mm))\ndrill cut part\n", FLOOR - below));
        }
    }
    if let Some(w) = word {
        rest.push_str(&format!("part.pocket.far {} part.pocket.w0\n", w.text()));
    }
    format!("unit mm\nuse std\nuse std.solids\nuse std.solids (cut)\nuse std.rounds (round, chamfer)\nin std.top {{\n{body}  \
             pf := face(w0, w1, w2, w3)\n}}\nblock := solid(std.solids.Prism(bounds, from: 0mm, to: 30mm))\n\
             pocket := solid(std.solids.Prism(pf, from: {FLOOR}mm, to: 31mm))\npart := solid(block)\npocket cut part\n{rest}")
}

/// What a round of `r` fills along the pocket's edge, `length` long, less what `cut` (radius `rho`)
/// keeps it out of: at each offset `s` from the wall the fill stands `h(s) = r − √(r² − (r − s)²)`
/// tall, and each line along the edge through it is filled over the edge's span less the chord the
/// cut takes from it — a hole's `2√(ρ² − (s − off)²)` about its centre (the run carried to the
/// plane halfway across it where no edge goes on beyond), the trench's `2√(ρ² − (z − z_c)²) / cos`
/// at the line's height `z` over the floor.
fn filled(length: f64, turn: f64, cut: Option<(Cut, f64)>, r: f64) -> f64 {
    let h = |s: f64| r - (r * r - (r - s) * (r - s)).max(0.0).sqrt();
    let outside = |lo: f64, hi: f64| length.min(lo).max(0.0) + (length - hi.max(0.0)).max(0.0);
    match cut {
        None => (1.0 - PI / 4.0) * r * r * length,
        Some((Cut::Hole { at, off }, rho)) => split(&|s| {
            let half = (rho * rho - (s - off) * (s - off)).max(0.0).sqrt();
            h(s) * if half > 0.0 { outside(at - half, at + half) } else { length }
        }, 0.0, r, &[off - rho, off + rho]),
        Some((Cut::Trench { at, below }, rho)) => {
            // at height `z`, the fill `r − √(r² − (r − z)²)` wide
            let slant = turn.cos().abs();
            split(&|z| {
                let half = (rho * rho - (z + below) * (z + below)).max(0.0).sqrt() / slant;
                h(z) * if half > 0.0 { outside(at - half, at + half) } else { length }
            }, 0.0, r, &[rho - below])
        }
    }
}

/// The pocket's fill: the part built with it, exactly, held to the part built without it (planes and
/// cylinders, no round) and the fill's closed form.
fn pocket_holds(length: f64, turn: f64, cut: Option<(Cut, f64)>, r: f64) {
    let [with, without] = [Some(Word::Round(r)), None].map(|w| {
        let src = pocket(length, turn, cut, w);
        let e = solved(&src);
        assert!(e.diags.is_empty(), "{:?}\n{src}", e.diags);
        props::volume(&exact(&e, "part"))
    });
    close(with, without + filled(length, turn, cut, r));
}

/// **A pocket's floor-to-wall fill broken by a hole** (#962's 133): a vertical hole of 4 through the
/// floor 2 inside the wall, on it and 2 within it, square-on and turned; at the pocket's far end
/// (no edge beyond it: the run carried to the plane through the hole's axis square to the edge,
/// kept outside the hole); and a trench under the floor crossing the edge square-on and at a
/// slant, which cuts the floor in two — its two pieces one plane to the run across it.
#[test]
fn a_fill_broken_by_a_hole_or_a_trench_is_one_run_kept_outside_it() {
    for turn in [0.0, 0.5] {
        pocket_holds(60.0, turn, None, 5.0);
        for off in [2.0, 0.0, -2.0] {
            pocket_holds(60.0, turn, Some((Cut::Hole { at: 25.0, off }, 4.0)), 5.0);
        }
        pocket_holds(60.0, turn, Some((Cut::Hole { at: 60.0, off: 0.0 }, 4.0)), 5.0);
        pocket_holds(60.0, turn, Some((Cut::Trench { at: 25.0, below: 0.634 }, 4.0)), 5.0);
    }
}
