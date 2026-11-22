//! **A round beside a seam its edge is not along** (#994, fixture 126): a box drafted 2° from its
//! floor (or upright), its plan corners R25, so each side is a plane going on tangent into its
//! corner's cone (or cylinder) along a line; and a foot unioned onto it whose side meets the box's
//! north side a few millimetres short of that line, aslant to it — the foot's side upright and the
//! box's leaning in, or the foot's leaning and the box's upright.  The fill where they meet stands
//! against the box's smooth surface (`a.smooth`), so where its ball's touch passes the seam it
//! rolls on the corner's cone: short of there the run's prism, past it the ball rolled against the
//! cone (`brep::spill::round_beyond`), the two glued at the section where the touch crosses the
//! seam (`Beyond::seamed`) — or past it all along, the ball against the cone the edge's whole
//! length.  Each held to a quadrature of `std.rounds.Fill`'s own words, slice by slice, to a
//! millionth; one whose ball stays short of the seam to the prism's closed form.
use std::f64::consts::PI;

use crate::common::text::solved;
use crate::common::rounds::{bisect, dot, gauss_legendre, refusal, square_run, volume};

/// The box (`draft` degrees, or upright where it is nought, 200 square and 100 tall about the top
/// plane's origin, its corners R25) and a foot 40 tall unioned onto it, its corners `foot` in turn
/// (`p0` to `p3`, its sides `f0` to `f3`) drafted `foot_draft` degrees (upright at nought); then
/// `rest`.
fn part(draft: f64, foot: [(f64, f64); 4], foot_draft: f64, rest: &str) -> String {
    joined(draft, foot, foot_draft, "union", [0.0, 40.0], rest)
}

/// `part` with the foot from `zs[0]` to `zs[1]` and joined to the box by `how` (`union`, or `cut`:
/// a notch).
fn joined(draft: f64, foot: [(f64, f64); 4], foot_draft: f64, how: &str, zs: [f64; 2], rest: &str) -> String {
    let solid = |name: &str, d: f64, [from, to]: [f64; 2]| if d == 0.0 {
        format!("solid(std.solids.Prism({name}, from: {from}mm, to: {to}mm))")
    } else {
        format!("solid(std.solids.Drafted({name}, from: {from}mm, to: {to}mm, draft: {d}deg))")
    };
    let mut s = String::from("unit mm\nuse std\nuse std.solids\nuse std.solids (union, cut)\nuse std.rounds (round, chamfer)\n\
        in std.top {\n  a := point\n  fix((-100, -100)) a\n  b := point\n  fix((100, -100)) b\n  c := point\n  \
        fix((100, 100)) c\n  d := point\n  fix((-100, 100)) d\n  (e0 := line(a, b)) -> (k1 := round(r: 25mm)) \
        (e1 := line(b, c)) -> (k2 := round(r: 25mm)) (e2 := line(c, d)) -> (k3 := round(r: 25mm)) \
        (e3 := line(d, a)) -> (k0 := round(r: 25mm)) close\n  box_f := face(e0, k1, e1, k2, e2, k3, e3, k0)\n}\n");
    s += &format!("box := {}\nin std.top {{\n", solid("box_f", draft, [0.0, 100.0]));
    for (k, (x, y)) in foot.iter().enumerate() {
        s += &format!("  p{k} := point\n  fix(({x}, {y})) p{k}\n");
    }
    s += "  f0 := line(p0, p1)\n  f1 := line(p1, p2)\n  f2 := line(p2, p3)\n  f3 := line(p3, p0)\n  \
        foot_f := face(f0, f1, f2, f3)\n}\n";
    s + &format!("foot := {}\nbody := solid(box)\nfoot {how} body\n{rest}", solid("foot_f", foot_draft, zs))
}

/// The issue's foot, its side `f1` from (71.2, 95) to (55, 190): it meets the box's north side
/// 4.4 to 4.7 short of the seam at x = 75, aslant.
const FOOT: [(f64, f64); 4] = [(-20.0, 95.0), (71.2, 95.0), (55.0, 190.0), (-5.0, 190.0)];

/// The same foot mirrored across x = 0, beside the north-west corner: its side `f3`.
const MIRRORED: [(f64, f64); 4] = [(-71.2, 95.0), (20.0, 95.0), (5.0, 190.0), (-55.0, 190.0)];

/// What `round(r)` of the foot's side `side` against the box adds, built exactly.
fn added(draft: f64, foot: [(f64, f64); 4], foot_draft: f64, side: &str, r: f64) -> f64 {
    let e = solved(&part(draft, foot, foot_draft, &format!("body.foot.{side} round(r: {r}mm) body.box\n")));
    let bare = solved(&part(draft, foot, foot_draft, ""));
    volume(&e, "body") - volume(&bare, "body")
}

/// `∫ g` over `[a, z]` by Gauss–Legendre on `n` stretches, each read through `x = (1 − cos πs)/2`
/// over its own: a square root at either end of the stretch is smooth in `s`.
fn integral(a: f64, z: f64, n: usize, g: &dyn Fn(f64) -> f64) -> f64 {
    let rule = gauss_legendre(8);
    (0..n).map(|j| {
        let (lo, hi) = (a + (z - a) * j as f64 / n as f64, a + (z - a) * (j + 1) as f64 / n as f64);
        rule.iter().map(|&(x, w)| {
            let s = 0.5 * (1.0 + x);
            let t = lo + (hi - lo) * 0.5 * (1.0 - (PI * s).cos());
            0.5 * w * (hi - lo) * 0.5 * PI * (PI * s).sin() * g(t)
        }).sum::<f64>()
    }).sum()
}

/// What a round of `r` adds where the foot's side from `p` to `q` (the foot on its left, drafted
/// `foot_draft` degrees) meets the box's north side (drafted `draft`), beside its north-east corner
/// — worked out from `std.rounds.Fill`'s words alone.  The box's distance is its nearest face's: the
/// north side's plane short of x = 75, the corner's cone (a cylinder upright) past it; the foot's,
/// its side's plane.  A point outside both is the fill's where no ball of radius `r` whose centre
/// stands at least `r` from both reaches it: the nearest centre straight off one face where it
/// stands far enough from the other, or a point of the curve where both offsets meet (in each
/// horizontal plane, the foot's offset line across the box's offset outline), found by Newton's
/// steps.  Read in horizontal slices between the floor and the foot's top (the planes the run ends
/// on): about the point where the two faces meet in the slice, along each ray from it the fill's
/// reach by bisection — and below the north side's line, where the ball touches the cone past the
/// seam, from where the ray leaves the box — its area by Gauss–Legendre over the angle (its ends
/// square roots), split where the ray runs along the side; the slices over the height, split where
/// the touch crosses the seam.  Nothing of the core is asked.
fn fill(draft: f64, side: [(f64, f64); 2], foot_draft: f64, r: f64) -> f64 {
    region(draft, side, foot_draft, r, 1.0, [0.0, 40.0])
}

/// `fill` (`sigma` 1), or what a round takes within the box where a notch's side meets its north
/// side (`sigma` −1, the box's distance turned round: `std.rounds.Run`'s words), between the
/// heights `zs` (the planes the run ends on).  Within the box a ray from the corner may pass
/// through a ball and out again before the cone, so each slice is read about the ball's centre
/// there instead: the centres are a convex set, so what the balls reach is, and the material by the
/// corner is too, and along each ray from the centre the region is the one stretch between them,
/// over the angles the corner is seen in, split where the ray meets the corner and the seam.
fn region(draft: f64, [p, q]: [(f64, f64); 2], foot_draft: f64, r: f64, sigma: f64, zs: [f64; 2]) -> f64 {
    let (sb, cb) = draft.to_radians().sin_cos();
    let (sf, cf) = foot_draft.to_radians().sin_cos();
    let (dx, dy) = (q.0 - p.0, q.1 - p.1);
    let l = dx.hypot(dy);
    // the foot's side's outward normal (the foot on the left of p → q), leaning in by its draft
    let n = [dy / l * cf, -dx / l * cf, sf];
    let o = [p.0, p.1, 0.0];
    let foot = |x: [f64; 3]| dot(n, [x[0] - o[0], x[1] - o[1], x[2]]);
    let boxed = |x: [f64; 3]| -> (f64, [f64; 3]) {
        // (the north side short of the seam, the east side below the corner)
        let (d, g) = if x[0] <= 75.0 {
            ((x[1] - 100.0) * cb + x[2] * sb, [0.0, cb, sb])
        } else if x[1] <= 75.0 {
            ((x[0] - 100.0) * cb + x[2] * sb, [cb, 0.0, sb])
        } else {
            let (u, v) = (x[0] - 75.0, x[1] - 75.0);
            let rho = u.hypot(v);
            ((rho - 25.0) * cb + x[2] * sb, [cb * u / rho, cb * v / rho, sb])
        };
        (sigma * d, g.map(|c| sigma * c))
    };
    // where the offsets meet at height s: the foot's offset line across the box's offset outline
    let k = r + dot(n, o);
    let spine = |s: f64| -> [f64; 3] {
        let rhs = k - n[2] * s;
        let y = 100.0 + (sigma * r - s * sb) / cb;
        let x = (rhs - n[1] * y) / n[0];
        if x <= 75.0 {
            return [x, y, s];
        }
        let rho = 25.0 + (sigma * r - s * sb) / cb;
        // the line through a point of it along (−n_y, n_x), across the circle about (75, 75)
        let at = [(rhs - n[1] * 75.0) / n[0], 75.0];
        let w = [-n[1], n[0]];
        let ww = w[0] * w[0] + w[1] * w[1];
        let d = [at[0] - 75.0, at[1] - 75.0];
        let b = d[0] * w[0] + d[1] * w[1];
        let t = (-b + (b * b - ww * (d[0] * d[0] + d[1] * d[1] - rho * rho)).sqrt()) / ww;
        let t = if at[1] + t * w[1] >= 75.0 { t } else { (-b - (b * b - ww * (d[0] * d[0] + d[1] * d[1] - rho * rho)).sqrt()) / ww };
        [at[0] + t * w[0], at[1] + t * w[1], s]
    };
    // whether no ball reaches `x`: no centre within `r` of it
    let free = |x: [f64; 3]| -> bool {
        let (db, nb) = boxed(x);
        let df = foot(x);
        if db >= r && df >= r {
            return false;
        }
        let mut best = f64::INFINITY;
        if db < r && foot([0, 1, 2].map(|i| x[i] + (r - db) * nb[i])) >= r {
            best = r - db;
        }
        if df < r && boxed([0, 1, 2].map(|i| x[i] + (r - df) * n[i])).0 >= r {
            best = best.min(r - df);
        }
        let f = |s: f64| { let c = spine(s); ((x[0] - c[0]).powi(2) + (x[1] - c[1]).powi(2) + (x[2] - c[2]).powi(2)).sqrt() };
        let (mut s, h) = (x[2], 1e-4);
        for _ in 0..40 {
            let (d1, d2) = ((f(s + h) - f(s - h)) / (2.0 * h), (f(s + h) - 2.0 * f(s) + f(s - h)) / (h * h));
            if !(d2 > 0.0) { break }
            let step = d1 / d2;
            s -= step;
            if step.abs() < 1e-13 { break }
        }
        best.min(f(s)) >= r
    };
    let held = |x: [f64; 3]| boxed(x).0 > 0.0 && foot(x) > 0.0;
    let filled = |x: [f64; 3]| held(x) && free(x);
    // in the slice at height z: the point where the faces meet, and the angle of the foot's side
    let slice = |z: f64| -> f64 {
        let yw = 100.0 - z * sb / cb;
        let xe = (dot(n, o) - n[2] * z - n[1] * yw) / n[0];
        let at = |a: f64, rho: f64| [xe + rho * a.cos(), yw + rho * a.sin(), z];
        let side = (sigma * n[0]).atan2(-sigma * n[1]);
        // within the box: the centres are a convex set (the box shrunk by the ball, and a
        // half-space), so what the balls reach is too, and so is the material near the corner —
        // along each ray from the centre in this plane, the region is one stretch, from where the
        // balls stop to where the material does, over the angles the corner is seen in
        if sigma < 0.0 {
            let c = spine(z);
            let at = |a: f64, rho: f64| [c[0] + rho * a.cos(), c[1] + rho * a.sin(), z];
            let near = |a: f64| bisect(0.0, 3.0 * r, |rho| !free(at(a, rho)));
            let far = |a: f64| bisect(0.0, 3.0 * r, |rho| held(at(a, rho)));
            let open = |a: f64| far(a) > near(a);
            let corner = (yw - c[1]).atan2(xe - c[0]);
            let (lo, hi) = (bisect(corner, corner - 0.5 * PI, open), bisect(corner, corner + 0.5 * PI, open));
            let mut cuts = vec![lo, corner, hi];
            // (where the side goes on into the cone, its curvature's turn)
            let seam = (yw - c[1]).atan2(75.0 - c[0]);
            if seam > lo && seam < hi && (seam - corner).abs() > 1e-9 {
                cuts.push(seam);
                cuts.sort_by(f64::total_cmp);
            }
            let g = |a: f64| 0.5 * (far(a).powi(2) - near(a).powi(2)).max(0.0);
            return cuts.windows(2).map(|w| integral(w[0], w[1], 4, &g)).sum();
        }
        // outside, the fill along each ray is one stretch from the corner
        let out = |a: f64, from: f64| bisect(from, 4.0 * r, |rho| filled(at(a, rho)));
        let mut area = integral(0.0, side, 4, &|a| 0.5 * out(a, 0.0).powi(2));
        // below the side's line, where the ball touches the cone past the seam: from where the
        // ray leaves the box, as far round as the fill is there at all
        let leaves = |a: f64| bisect(0.0, 3.0 * r, |rho| boxed(at(a, rho)).0 <= 0.0);
        let below = |a: f64| { let lo = leaves(a); filled(at(a, lo * (1.0 + 1e-9) + 1e-9)) };
        if below(-1e-9) {
            let least = bisect(0.0, -0.5, below);
            area += integral(least, 0.0, 4, &|a| { let lo = leaves(a); 0.5 * (out(a, lo * (1.0 + 1e-12)).powi(2) - lo * lo) });
        }
        area
    };
    // the height where the touch crosses the seam (x = 75), if it does between the ends: the
    // spine's against the side's plane, the touch straight off it
    let x_at = |s: f64| (k - n[2] * s - n[1] * (100.0 + (sigma * r - s * sb) / cb)) / n[0];
    let (x0, x1) = (x_at(0.0) - 75.0, x_at(1.0) - 75.0);
    let cross = -x0 / (x1 - x0) - sigma * r * sb;
    if cross <= zs[0] || cross >= zs[1] {
        return integral(zs[0], zs[1], 4, &slice);
    }
    integral(zs[0], cross, 4, &slice) + integral(cross, zs[1], 4, &slice)
}

/// **The issue's foot, rounded across the seam** (#994): R10 and R7 touch the corner's cone the
/// edge's whole length, R5.4 crosses the seam partway up — the run's prism below, the ball against
/// the cone above, glued at the section where its touch crosses — each the quadrature's to a
/// millionth.
#[test]
fn a_round_across_a_seam_aslant_its_edge_is_built() {
    for r in [10.0, 7.0, 5.4] {
        let got = added(2.0, FOOT, 0.0, "f1", r);
        let want = fill(2.0, [FOOT[1], FOOT[2]], 0.0, r);
        assert!((got - want).abs() <= 1e-6 * want, "r = {r}: {got} against {want}");
    }
}

/// **Short of the seam, the run's prism** (#994): R4 on the issue's foot, whose ball's touch stays
/// short of the seam all along — what it built before — the section of a round in the corner
/// between the two planes swept between the floor and the foot's top, to 1e-9.
#[test]
fn a_round_short_of_a_seam_aslant_its_edge_is_the_prism() {
    let (sb, cb) = 2f64.to_radians().sin_cos();
    let ne = [0.0, cb, sb];
    let (dx, dy) = (FOOT[2].0 - FOOT[1].0, FOOT[2].1 - FOOT[1].1);
    let l = dx.hypot(dy);
    let nf = [dy / l, -dx / l, 0.0];
    // the corner the round stands in, between the faces, and the edge's slope from upright
    let alpha = PI - dot(ne, nf).acos();
    let along = [ne[1] * nf[2] - ne[2] * nf[1], ne[2] * nf[0] - ne[0] * nf[2], ne[0] * nf[1] - ne[1] * nf[0]];
    let rise = along[2].abs() / dot(along, along).sqrt();
    let r = 4.0;
    let want = r * r * (1.0 / (alpha / 2.0).tan() - (PI - alpha) / 2.0) * 40.0 / rise;
    let got = added(2.0, FOOT, 0.0, "f1", r);
    assert!((got - want).abs() <= 1e-9 * want, "{got} against {want}");
}

/// **Mirrored** (#994): the foot beside the north-west corner, its side `f3` — the seam the other
/// way along the edge — the first's image, so its fill the quadrature's of the first.
#[test]
fn a_round_across_a_seam_aslant_its_edge_is_built_mirrored() {
    for r in [10.0, 5.4] {
        let got = added(2.0, MIRRORED, 0.0, "f3", r);
        let want = fill(2.0, [FOOT[1], FOOT[2]], 0.0, r);
        assert!((got - want).abs() <= 1e-6 * want, "r = {r}: {got} against {want}");
    }
}

/// An upright box, so its corners are cylinders, and a foot drafted 10° whose side `f1` stands at
/// x = 70 (its south side deep within the box, which its draft leans north): the edge leans from 5 short of the seam at the floor to 12 short at the foot's top.
const LEANING: [(f64, f64); 4] = [(-20.0, 60.0), (70.0, 60.0), (70.0, 190.0), (-20.0, 190.0)];

/// **Into a cylinder** (#994): the leaning foot rounded 8 — its touch crosses the upright seam
/// partway up — and 14, past it all along, each the quadrature's to a millionth; and 4, short of
/// it, the right angle's prism between the floor and the foot's top, 40 / cos 10° long, to 1e-9.
#[test]
fn a_round_across_a_seam_aslant_its_edge_into_a_cylinder_is_built() {
    for r in [8.0, 14.0] {
        let got = added(0.0, LEANING, 10.0, "f1", r);
        let want = fill(0.0, [LEANING[1], LEANING[2]], 10.0, r);
        assert!((got - want).abs() <= 1e-6 * want, "r = {r}: {got} against {want}");
    }
    let got = added(0.0, LEANING, 10.0, "f1", 4.0);
    let want = square_run(4.0) * 40.0 / 10f64.to_radians().cos();
    assert!((got - want).abs() <= 1e-9 * want, "{got} against {want}");
}

/// The issue's foot reaching 40 into the box (its side `f1` the same line, from y = 60), cut from
/// it as a notch.
const NOTCH: [(f64, f64); 4] = [(-20.0, 60.0), (77.168421052632, 60.0), (55.0, 190.0), (-5.0, 190.0)];

/// **Convex** (#994): a notch the issue's foot's shape cut through the box's whole height, its side
/// `f1` meeting the box's north side `e2` within it; their rim rounded 10 (past the seam all along) and
/// 5.4 (crossing it partway up), each what the quadrature of `std.rounds.Run`'s words takes, to a
/// millionth.
#[test]
fn a_rim_across_a_seam_aslant_its_edge_is_built() {
    let notch = |rest: &str| joined(2.0, NOTCH, 0.0, "cut", [-1.0, 101.0], rest);
    let bare = volume(&solved(&notch("")), "body");
    for r in [10.0, 5.4] {
        let got = bare - volume(&solved(&notch(&format!("body.foot.f1 round(r: {r}mm) body.box.e2\n"))), "body");
        let want = region(2.0, [NOTCH[1], NOTCH[2]], 0.0, r, -1.0, [0.0, 100.0]);
        assert!((got - want).abs() <= 1e-6 * want, "r = {r}: {got} against {want}");
    }
}

/// **A chamfer across the seam is refused by name** (#994): the flat's run changes face along the
/// seam's normal plane, which crosses its section aslant, and no construction is made of it yet —
/// said so, the field reading it.
#[test]
fn a_chamfer_across_a_seam_aslant_its_edge_is_refused_by_name() {
    let e = solved(&part(2.0, FOOT, 0.0, "body.foot.f1 chamfer(d: 6mm) body.box\n"));
    let why = refusal(&e, "body");
    assert!(why.contains("a chamfer's flat beside a seam crossing its footprint aslant"), "{why}");
}
