//! **A standing ball whose stretch ends where a face cuts across it** (#994 item 5, fixture 133:
//! R10 along an arm's bottom edges on the finished part, "the ball … stands still, and their
//! meeting ends where `hub.far` cuts across it: no stretch of its dome is carried past and cut off
//! by a face yet").  A rim rounded to its own cylinder's radius has its ball's centre on the axis
//! alone, standing still over the stretch of the rim the body leaves (`fillet::dome`); where a
//! face cuts across that stretch, `std.rounds` carries the set on past the end, keeps it on the
//! face's side, and reads it against the faces' meeting only where its nearest point of that
//! meeting is on the crease — and every section of a still ball holds its centre and its foot, so
//! none is cut off whole by a face the centre stands within: the dome goes on round until the
//! meeting comes back (`fillet::dome_cut`).  Both ends cut across: the whole loop's dome, cut off
//! by the faces at both (a D-shaped post under a tilted lid, the chord a plane or a cylinder's
//! bite); one end where the side goes on tangent: half round from that section, cut off by the
//! face at the other (an arm's rounded end half inside a hub).  Each volume the closed form's to
//! 1e-9 where one is written out (below, by `∫∫ (lid − ball)` over the plan the round keeps), else
//! by Gauss–Legendre over exact inner integrals, and the field drawing the kernel's boundary.
use std::f64::consts::PI;

use gcs_core::brep::props;

use crate::common::{close, text::solved};
use crate::common::rounds::{exported, field_is_the_kernels, gauss_legendre, refused};

/// A post 14 high on the plan `plan` (in `std.top`, naming its rim `side` and its face
/// `outline`) cut by a lid tilted `tilt` degrees about the y axis through (0, 0, 10), its rim
/// along `side` treated by `word`.
fn lidded(plan: &str, tilt: f64, word: &str) -> String {
    let (c, s) = (tilt.to_radians().cos(), tilt.to_radians().sin());
    format!("unit mm\nuse std\nuse std.solids (cut, union, common)\nuse std.rounds (round, chamfer)\n\
        in std.top {{\n{plan}}}\n\
        post := solid(std.solids.Prism(outline, from: 0mm, to: 14mm))\n\
        across := axis hint(dir: (0, 1, 0))\nacross parallel std.y\n\
        u := axis hint(dir: ({c:.6}, 0, {:.6}))\nu perpendicular across\nu angle({tilt}deg, about: std.y) std.x\n\
        slant := plane(u: u, v: across)\nfix(origin == (0mm, 0mm, 10mm)) slant\n\
        in slant {{\n  p := point\n  q := point\n  r := point\n  s := point\n\
        \x20 fix((-20, -20)) p\n  fix((20, -20)) q\n  fix((20, 20)) r\n  fix((-20, 20)) s\n\
        \x20 (pq := line(p, q)) -> (qr := line(q, r)) -> (rs := line(r, s)) -> (sp := line(s, p)) -> close\n}}\n\
        lid := solid(std.solids.Prism(face(pq, qr, rs, sp), from: 0mm, to: 30mm))\n\
        drum := solid(post)\nlid cut drum\ndrum.lid {word} drum.post.side\n", -s)
}

/// A D: an arc of radius 5 about the origin closed by a chord at `x = a`.
fn d_plan(a: f64) -> String {
    let h = (25.0 - a * a).sqrt();
    format!("  o := point\n  fix((0, 0)) o\n  a := point\n  fix(({a}, {:.17})) a\n  b := point\n  fix(({a}, {h:.17})) b\n  \
        side := arc(center: o, start: b, end: a) hint(r: 5)\n  flat := line(a, b)\n  outline := face(side, flat)\n", -h)
}

/// The D under the lid, its rim rounded to its radius 5 (or not, `round` false).  The lid is
/// `z = 10 − x tan t`; the ball's centre on the axis 5 below it, `z = 10 − 5 / cos t`, and its
/// lower half `z_c − √(25 − ρ²)` under the lid everywhere over the disc (the ball touches the lid
/// once).  What the round takes is `∫∫_D (lid − z_c − √(25 − ρ²))`: the D's area `A` times
/// `5 / cos t`, less `tan t` times its moment `∫∫_D x = −⅔ h³` (the segment past the chord, `h` its
/// half chord, taken from the disc's nought), less the volume under the hemisphere over the D,
/// `⅔π5³` less the segment's `∫_a^5 π(25 − x²)/2`.
fn d_volume(a: f64, tilt: f64, round: bool) -> f64 {
    let r = 5.0f64;
    let t = tilt.to_radians();
    let h = (r * r - a * a).sqrt();
    let area = PI * r * r - (r * r * (a / r).acos() - a * h);
    let mx = -(2.0 / 3.0) * h * h * h;
    let post = 10.0 * area - t.tan() * mx;
    if !round { return post }
    let under = (2.0 / 3.0) * PI * r * r * r - (PI / 2.0) * (r * r * (r - a) - (r * r * r - a * a * a) / 3.0);
    post - (r / t.cos() * area - t.tan() * mx - under)
}

/// **Both ends cut across by the chord**: the dome round the whole loop, the D keeping it.  The
/// chord short of the axis, past it (the stretch under half a turn), and a hair off it; the lid
/// tilted 10°, 20° and 30°, and square (a ring there, which the lowering turns and has built since
/// #962 item 3 — the same shape, the same closed form).
#[test]
fn a_tilted_ds_rim_rounded_to_its_radius_is_the_whole_dome_its_chord_cuts() {
    for (a, tilt) in [(3.0, 20.0), (3.0, 10.0), (3.0, 30.0), (-2.0, 20.0), (0.5, 20.0), (3.0, 0.0)] {
        let e = solved(&lidded(&d_plan(a), tilt, "round(r: 5mm)"));
        assert!(e.diags.is_empty(), "{:?}", e.diags);
        assert_eq!(e.sketch.field_meshed_ahead(crate::common::ent(&e, "drum").i()), None);
        close(props::volume(&exported(&e, "drum")), d_volume(a, tilt, true));
    }
    let e = solved(&lidded(&d_plan(3.0), 20.0, "round(r: 5mm)"));
    field_is_the_kernels(&e, "drum", [-6.0, -6.0, 2.0], [6.0, 6.0, 13.0], 0.05, 3000);
}

/// **The chord a cylinder's bite**: an arc of radius 5 about the origin closed by one of radius 6
/// about (9, 0) — the dome kept outside that cylinder, cut off by a curved face's solid.  The
/// field draws the kernel's boundary.
#[test]
fn a_lids_rim_cut_across_by_a_cylinder_is_the_dome_kept_outside_it() {
    let x = 70.0f64 / 18.0;
    let y = (25.0 - x * x).sqrt();
    let plan = format!("  o := point\n  fix((0, 0)) o\n  k := point\n  fix((9, 0)) k\n  a := point\n  fix(({x:.17}, {:.17})) a\n  \
        b := point\n  fix(({x:.17}, {y:.17})) b\n  side := arc(center: o, start: b, end: a) hint(r: 5)\n  \
        bite := arc(center: k, start: a, end: b, sense: cw) hint(r: 6)\n  outline := face(side, bite)\n", -y);
    let e = solved(&lidded(&plan, 20.0, "round(r: 5mm)"));
    assert!(e.diags.is_empty(), "{:?}", e.diags);
    let b = exported(&e, "drum");
    assert!(props::volume(&b) < d_volume(x, 20.0, false));
    field_is_the_kernels(&e, "drum", [-6.0, -6.0, 2.0], [6.0, 6.0, 13.0], 0.05, 3000);
}

/// **What is refused, and by what words**: a ball a hair over the post's radius, which nothing
/// holds; a hair under it, which rolls, and whose run's ends against the chord no section is
/// carried past yet (as before); and a post with a chord each side, its rim two stretches, where
/// the meeting past one stretch's end comes back within both chords — the other stretch — and
/// halfway there is no section of a ball standing still.
#[test]
fn a_standing_ball_cut_across_is_refused_saying_why() {
    let why = refused(&solved(&lidded(&d_plan(3.0), 20.0, "round(r: 5.001mm)")), "drum");
    assert!(why.contains("of radius 5.001, is larger than `post.side` can hold"), "{why}");
    let why = refused(&solved(&lidded(&d_plan(3.0), 20.0, "round(r: 4.999mm)")), "drum");
    assert!(why.contains("cannot roll on past where `post.flat` and `post.flat` cut it off"), "{why}");
    let h = 4.0;
    let plan = format!("  o := point\n  fix((0, 0)) o\n  a := point\n  fix((3, {})) a\n  b := point\n  fix((3, {h})) b\n  \
        c := point\n  fix((-3, {h})) c\n  d := point\n  fix((-3, {})) d\n  \
        side := arc(center: o, start: b, end: c) hint(r: 5)\n  back := line(c, d)\n  \
        other := arc(center: o, start: d, end: a) hint(r: 5)\n  flat := line(a, b)\n  \
        outline := face(side, back, other, flat)\n", -h, -h);
    let why = refused(&solved(&lidded(&plan, 20.0, "round(r: 5mm)")), "drum");
    assert!(why.contains("comes back"), "{why}");
}

/// An arm 20 wide along x, its end at the origin a half circle of radius 10, from its slanted
/// bottom (`z = −10 − y tan slope`) to z = 20, unioned to a hub 70 tall that holds the arm where it
/// lies past the hub's face, the line through (0, `c`) at `skew` degrees to x (a hub 60 along x
/// and from y = −30 to that line), and the arm's bottom edges `edges` treated by `word` (none:
/// untreated).
fn arm(c: f64, slope: f64, skew: f64, word: Option<&str>, edges: &str) -> String {
    let (co, si) = (slope.to_radians().cos(), slope.to_radians().sin());
    let t = skew.to_radians().tan();
    let mut s = format!("unit mm\nuse std\nuse std.solids (cut, union)\nuse std.rounds (round)\n\
        in std.top {{\n  o := point\n  fix((0, 0)) o\n  a := point\n  fix((0, 10)) a\n  b := point\n  fix((0, -10)) b\n\
        \x20 c := point\n  fix((60, -10)) c\n  d := point\n  fix((60, 10)) d\n\
        \x20 tip := arc(center: o, start: a, end: b) hint(r: 10)\n  s0 := line(b, c)\n  s1 := line(c, d)\n  s2 := line(d, a)\n\
        \x20 h0 := point\n  fix((-30, -30)) h0\n  h1 := point\n  fix((30, -30)) h1\n  h2 := point\n  fix((30, {:.17})) h2\n\
        \x20 h3 := point\n  fix((-30, {:.17})) h3\n\
        \x20 (q0 := line(h0, h1)) -> (q1 := line(h1, h2)) -> (hf := line(h2, h3)) -> (q3 := line(h3, h0)) -> close\n}}\n\
        bar := solid(std.solids.Prism(face(tip, s0, s1, s2), from: -20mm, to: 20mm))\n\
        along := axis hint(dir: (1, 0, 0))\nalong parallel std.x\n\
        v := axis hint(dir: (0, {co:.6}, {:.6}))\nv perpendicular along\nv angle({slope}deg, about: std.x) std.y\n\
        slant := plane(u: along, v: v)\nfix(origin == (0mm, 0mm, -10mm)) slant\n\
        in slant {{\n  p := point\n  q := point\n  r := point\n  s := point\n\
        \x20 fix((-80, -80)) p\n  fix((80, -80)) q\n  fix((80, 80)) r\n  fix((-80, 80)) s\n\
        \x20 (pq := line(p, q)) -> (qr := line(q, r)) -> (rs := line(r, s)) -> (sp := line(s, p)) -> close\n}}\n\
        lid := solid(std.solids.Prism(face(pq, qr, rs, sp), from: -40mm, to: 0mm))\n\
        arm := solid(bar)\nlid cut arm\n\
        hub := solid(std.solids.Prism(face(q0, q1, hf, q3), from: -40mm, to: 30mm))\n\
        part := solid(arm)\nhub union part\n", c + 30.0 * t, c - 30.0 * t, -si);
    if let Some(w) = word { s += &format!("part.arm.lid {w} {edges}\n"); }
    s
}

/// What the arm's end rounded to its radius 10 takes, the hub's face the line `y = c + x tan skew`:
/// over the end's disc within `x ≤ 0` (the section where the side goes on tangent: the plane the
/// foot, on the bottom's normal, and the touch on the seam span) and past the line, the height
/// between the bottom `z = −10 − y tan s` and the ball's lower half, its centre on the axis
/// `10 / cos s` above the bottom's `−10`: `10 / cos s + y tan s − √(100 − ρ²)`.  About the end's
/// axis, along each ray exactly — `∫ (A + B ρ sin θ − √(100 − ρ²)) ρ dρ` over where the ray is past
/// the line within the disc — and round from `θ = π/2` to `3π/2` by Gauss–Legendre, split where the
/// line meets the circle and in `w` with `θ − θ_b ∝ w²` either side of it (the ray's reach there
/// goes as a power `3/2` of the turn).
fn arm_taken(c: f64, slope: f64, skew: f64) -> f64 {
    let (s, t) = (slope.to_radians(), skew.to_radians().tan());
    let (a, b) = (10.0 / s.cos(), s.tan());
    // where the line meets the circle, x < 0: (1 + t²) x² + 2ct x + c² − 100 = 0
    let (qa, qb, qc) = (1.0 + t * t, 2.0 * c * t, c * c - 100.0);
    let x0 = (-qb - (qb * qb - 4.0 * qa * qc).sqrt()) / (2.0 * qa);
    let tb = (c + t * x0).atan2(x0).rem_euclid(2.0 * PI);
    // along the ray at `θ`: where `ρ (sin θ − t cos θ) ≥ c` within the disc
    let ray = |th: f64| {
        let d = th.sin() - t * th.cos();
        let (lo, hi) = if d > 0.0 { ((c / d).max(0.0), 10.0) } else if d < 0.0 { (0.0, (c / d).min(10.0)) }
            else if c <= 0.0 { (0.0, 10.0) } else { (0.0, 0.0) };
        if hi <= lo { return 0.0 }
        let cube = |r: f64| (100.0 - r * r).max(0.0).powf(1.5);
        a * (hi * hi - lo * lo) / 2.0 + b * th.sin() * (hi.powi(3) - lo.powi(3)) / 3.0 + (cube(hi) - cube(lo)) / 3.0
    };
    let gl = gauss_legendre(96);
    let side = |far: f64| -> f64 {
        gl.iter().map(|&(u, w)| { let v = 0.5 * (1.0 + u); 0.5 * w * 2.0 * v * (far - tb) * ray(tb + (far - tb) * v * v) }).sum()
    };
    side(0.5 * PI).abs() + side(1.5 * PI).abs()
}

/// The closed form `arm_taken` reads by quadrature, the hub's face square at `y = c`: `A / cos s`
/// with `A` the quarter-disc segment's area `(100 acos(c/10) − c√(100 − c²)) / 2`, plus `tan s`
/// times its moment `(100 − c²)^{3/2} / 3`, less the volume under the hemisphere over it,
/// `π(100(10 − c) − (1000 − c³)/3) / 4`.
fn arm_taken_square(c: f64, slope: f64) -> f64 {
    let s = slope.to_radians();
    let w = (100.0 - c * c).sqrt();
    let area = (100.0 * (c / 10.0).acos() - c * w) / 2.0;
    10.0 / s.cos() * area + s.tan() * w * w * w / 3.0 - PI / 4.0 * (100.0 * (10.0 - c) - (1000.0 - c * c * c) / 3.0)
}

/// **The arm's end half inside the hub** (fixture 133's shape): its bottom's edge along the end
/// rounded to the end's radius, the ball standing on the end's axis — over the stretch from where
/// the side goes on tangent to where the hub's face cuts across it, the dome half round from the
/// side's section, kept on the face's side.  The face past the axis and short of it (the stretch
/// past a quarter turn), square to y, skewed either way, the bottom slanted 15° and 25°; each
/// the plain part's volume less what the round takes, the face square in closed form and its
/// quadrature agreeing.  (Further past the axis — `y = −6` — the field reads the set a few tenths
/// short of the ball just inside the face, where the crease's end is a point's nearest: the gate
/// refuses it, "disagrees with the field".)
#[test]
fn an_arms_end_rounded_to_its_radius_where_a_hub_cuts_across_it_is_the_dome_kept_out_of_it() {
    let tip = "part.arm.bar.tip";
    for (c, slope, skew) in [(4.0, 15.0, 0.0), (-3.0, 15.0, 0.0), (0.5, 25.0, 0.0),
                             (4.0, 15.0, -20.0), (4.0, 15.0, 5.0), (-3.0, 25.0, 10.0)] {
        let plain = props::volume(&exported(&solved(&arm(c, slope, skew, None, tip)), "part"));
        let e = solved(&arm(c, slope, skew, Some("round(r: 10mm)"), tip));
        assert!(e.diags.is_empty(), "{:?}", e.diags);
        let taken = arm_taken(c, slope, skew);
        if skew == 0.0 { close(taken, arm_taken_square(c, slope)); }
        close(props::volume(&exported(&e, "part")), plain - taken);
    }
    let e = solved(&arm(4.0, 15.0, -20.0, Some("round(r: 10mm)"), tip));
    field_is_the_kernels(&e, "part", [-11.0, -2.0, -16.0], [2.0, 11.0, 2.0], 0.05, 3000);
}

/// **The arm's bottom edges all rounded** (fixture 133's statement, the edges along its sides
/// too): the dome glued at the side's section to the run along it, as a standing ball's piece
/// always is.  Built, and the field draws the kernel's boundary round the end.
#[test]
fn an_arms_bottom_edges_rounded_where_a_hub_cuts_across_its_end_are_the_fields() {
    let e = solved(&arm(4.0, 15.0, 0.0, Some("round(r: 10mm)"), "part.arm.bar"));
    assert!(e.diags.is_empty(), "{:?}", e.diags);
    exported(&e, "part");
    field_is_the_kernels(&e, "part", [-11.0, -2.0, -16.0], [20.0, 11.0, 2.0], 0.05, 3000);
}

