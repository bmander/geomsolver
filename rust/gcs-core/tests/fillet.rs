//! **Fillets** (Solvent §6.9, issue #66, rung 1): a rolling ball's material along the edges where
//! two solids or faces meet, held against arithmetic — a straight round removes `(1 − π/4) r²` of
//! section along its edge, a ring adds or removes its section turned about the axis (Pappus) —
//! and every fillet this rung cannot round exactly refused with its reason.

use gcs_core::program::{elaborate, Code, Elaborated};
use std::f64::consts::PI;

use crate::common::unit;
use gcs_core::space::{dot, scale, sub};

/// A drawing written on the page, drawn in `std.front` (the page's own coordinates: x right, z
/// up, its depth toward −y); one already drawn in a plane of `std` as it is.
fn front(src: &str) -> String {
    if src.contains("in std.") { return src.to_string(); }
    let (head, body) = match src.strip_prefix("unit mm\n") { Some(rest) => ("unit mm\n", rest), None => ("", src) };
    format!("{head}use std\nin std.front {{\n{body}\n}}\n")
}

fn read(src: &str) -> Elaborated { crate::common::read(&front(src)) }

fn refused(src: &str, code: Code, needle: &str) {
    let src = &front(src);
    let (prog, errs, linked) = gcs_core::library::parse_linked(src);
    let mut saw: Vec<String> = errs.iter().map(|e| format!("E100: {}", e.message)).collect();
    let mut hit = code == Code::E100 && errs.iter().any(|d| d.message.contains(needle));
    if !hit && errs.is_empty() {
        // what a fillet cannot round is known once the drawing is solved: the documents here
        // are seeded where they solve, so the diagnosis after the solve reads them as they are
        let e = elaborate(&prog);
        let mut diags = linked;
        diags.extend(e.diags.iter().cloned());
        diags.extend(gcs_core::program::solid_diagnostics(&e.sketch, &e.map));
        saw.extend(diags.iter().map(|d| format!("{}: {}", d.code.as_str(), d.message)));
        hit = diags.iter().any(|d| d.code == code && d.message.contains(needle));
    }
    assert!(hit, "expected {} `{needle}`\n{src}\n{saw:#?}", code.as_str());
}

fn volume(e: &Elaborated, name: &str) -> f64 {
    let key = format!("{name}.volume");
    // the one solid asked of the report: every other solid of the document is left unevaluated
    gcs_core::report::positions_where(&e.sketch, &e.map, &|n| n == name)
        .into_iter()
        .find(|(n, _)| *n == key)
        .unwrap_or_else(|| panic!("no `{key}` in the report"))
        .1
}

fn assert_volume(got: f64, want: f64) {
    assert!((got - want).abs() <= 1e-9 * want.abs(), "{got} != {want} (off by {:e})", got - want);
}

/// A 60 × 40 rectangle on the page, fully dimensioned and grounded, as `sec`.
const RECT: &str = "\
unit mm
a := point
b := point hint(x: 60, y: 0)
c := point hint(x: 60, y: 40)
d := point hint(x: 0, y: 40)
(ab := line(a, b)) -> (bc := line(b, c)) -> (cd := line(c, d)) -> (da := line(d, a)) -> close
horizontal ab
vertical bc
a distance(60) b
a distance(40) d
fix(x == 0, y == 0) a
sec := face(ab, bc, cd, da)
";

/// A circle of radius `r` about (30, 20), as `f`, its circle `k` and centre `o`.
fn disc(f: &str, r: f64) -> String {
    format!(
        "{f}_o := point hint(x: 30, y: 20)\na distance(30, along: x) {f}_o\na distance(20, along: y) {f}_o\n\
         {f}_k := circle(center: {f}_o) hint(r: {r})\nradius({r}) {f}_k\n{f} := face({f}_k)\n"
    )
}

/// The ring a ball of radius `r` fills in the corner where a cylinder of radius `big` stands on a
/// plane: the square of side `r` at the corner less the quarter disc about the ball's centre,
/// turned about the axis (Pappus: area times the path of its centroid).
fn root_ring(big: f64, r: f64) -> f64 {
    let square = r * r;
    let quarter = PI * r * r / 4.0;
    // the first moment about the axis: the area times its centroid's radius
    let moment = square * (big + r / 2.0) - quarter * (big + r - 4.0 * r / (3.0 * PI));
    2.0 * PI * moment
}

#[test]
fn a_rounded_edge_takes_its_section_along_the_edge() {
    let r = 5.0;
    let e = read(&format!(
        "{RECT}block := solid(sec, depth: 30mm)\nlip := fillet(block.near, block.bc, r: {r}mm)\nlip cut block\n"
    ));
    assert_volume(volume(&e, "block"), 60.0 * 40.0 * 30.0 - (1.0 - PI / 4.0) * r * r * 40.0);
    // the fillet on its own is the material the ball rolls off: its caps meet the ball's arc in
    // cusps, which the mesher recovers (a hull corner's fan is walked both ways round)
    assert_volume(volume(&e, "lip"), (1.0 - PI / 4.0) * r * r * 40.0);
}

#[test]
fn a_boss_root_fills_its_ring() {
    let (big, r) = (8.0, 3.0);
    let e = read(&format!(
        "{RECT}{}plate := solid(sec, depth: 10mm)\nboss := solid(boss_f, from: 0mm, to: 20mm)\n\
         body := solid(plate)\nboss union body\nroot := fillet(boss, plate, r: {r}mm)\nroot union body\n",
        disc("boss_f", big)
    ));
    let want = 60.0 * 40.0 * 10.0 + PI * big * big * 20.0 + root_ring(big, r);
    assert_volume(volume(&e, "root"), root_ring(big, r));
    assert_volume(volume(&e, "body"), want);
}

#[test]
fn a_bore_drilled_after_the_root_passes_through_it() {
    let (big, r, small) = (8.0, 3.0, 4.0);
    let e = read(&format!(
        "{RECT}{}{}plate := solid(sec, depth: 10mm)\nboss := solid(boss_f, from: 0mm, to: 20mm)\n\
         bore := solid(bore_f, from: -10mm, to: 20mm)\n\
         body := solid(plate)\nboss union body\nroot := fillet(boss, plate, r: {r}mm)\nroot union body\nbore cut body\n",
        disc("boss_f", big),
        disc("bore_f", small)
    ));
    let want = 60.0 * 40.0 * 10.0 + PI * big * big * 20.0 + root_ring(big, r) - PI * small * small * 30.0;
    assert_volume(volume(&e, "body"), want);
}

/// A disc of radius 40 and thickness 10 and a frustum standing on it (radius 15 at its foot, 5 at
/// its top, 20 tall), both turned about one vertical axis on the page; the frustum's root filleted
/// with radius 3 and its top rim rounded with radius 1.
const TURNED: &str = include_str!("../../examples/solid_fillet_turned.sv");

/// The volume a ball of radius `r` in the meridian corner at `corner` between the unit
/// directions `d` sweeps when turned about the axis (`ρ = 0`): Green's theorem over its section —
/// the corner, the two touches, the arc between — `2π ∮ ρ²/2 dz`, each piece in closed form.
fn turned_wedge(corner: [f64; 2], d: [[f64; 2]; 2], r: f64) -> f64 {
    let cos = d[0][0] * d[1][0] + d[0][1] * d[1][1];
    let half = cos.acos() / 2.0;
    let s = r / half.tan();
    let t = d.map(|u| [corner[0] + s * u[0], corner[1] + s * u[1]]);
    let bis = [d[0][0] + d[1][0], d[0][1] + d[1][1]];
    let bl = bis[0].hypot(bis[1]);
    let c = [corner[0] + bis[0] / bl * r / half.sin(), corner[1] + bis[1] / bl * r / half.sin()];
    // ∫ ρ²/2 dz along a segment, and along the short arc from the first touch to the second
    let line = |p: [f64; 2], q: [f64; 2]| (q[1] - p[1]) / 2.0 * (p[0] * p[0] + p[0] * q[0] + q[0] * q[0]) / 3.0;
    let angle = |p: [f64; 2]| (p[1] - c[1]).atan2(p[0] - c[0]);
    let (a0, mut a1) = (angle(t[0]), angle(t[1]));
    while a1 - a0 > PI { a1 -= 2.0 * PI; }
    while a0 - a1 > PI { a1 += 2.0 * PI; }
    let prim = |phi: f64| {
        let (sn, cs) = phi.sin_cos();
        (c[0] * c[0] * r * sn + c[0] * r * r * (phi + sn * cs) + r * r * r * (sn - sn.powi(3) / 3.0)) / 2.0
    };
    let flux = line(corner, t[0]) + (prim(a1) - prim(a0)) + line(t[1], corner);
    2.0 * PI * flux.abs()
}

#[test]
fn a_frustum_root_and_its_rim_turn_their_sections() {
    let root = turned_wedge([15.0, 10.0], [[1.0, 0.0], [-1.0 / 5f64.sqrt(), 2.0 / 5f64.sqrt()]], 3.0);
    let rim = turned_wedge([5.0, 30.0], [[-1.0, 0.0], [1.0 / 5f64.sqrt(), -2.0 / 5f64.sqrt()]], 1.0);
    let e = read(TURNED);
    assert_volume(volume(&e, "root"), root);
    assert_volume(volume(&e, "rim"), rim);
    let frustum = PI * 20.0 / 3.0 * (15.0 * 15.0 + 15.0 * 5.0 + 5.0 * 5.0);
    assert_volume(volume(&e, "body"), PI * 40.0 * 40.0 * 10.0 + frustum + root - rim);
}

/// A rib 10 wide and 5 tall across the whole 60 × 40 plate, or stopping `inset` short of each side.
fn rib(inset: f64) -> String {
    let (lo, hi) = (inset, 40.0 - inset);
    format!(
        "{RECT}r0 := point hint(x: 20, y: {lo})\nr1 := point hint(x: 30, y: {lo})\n\
         r2 := point hint(x: 30, y: {hi})\nr3 := point hint(x: 20, y: {hi})\n\
         fix(x == 20, y == {lo}) r0\nfix(x == 30, y == {lo}) r1\nfix(x == 30, y == {hi}) r2\nfix(x == 20, y == {hi}) r3\n\
         (e0 := line(r0, r1)) -> (e1 := line(r1, r2)) -> (e2 := line(r2, r3)) -> (e3 := line(r3, r0)) -> close\n\
         rib_f := face(e0, e1, e2, e3)\nplate := solid(sec, depth: 10mm)\nrib := solid(rib_f, from: 0mm, to: 5mm)\n\
         body := solid(plate)\nrib union body\n"
    )
}

#[test]
fn a_rib_across_the_plate_is_filleted_along_both_sides() {
    let r = 2.0;
    // its ends lie in the plate's sides, which the fillets end in too: those seams are no edges
    let e = read(&format!("{}root := fillet(rib, plate, r: {r}mm)\nroot union body\n", rib(0.0)));
    let rounds = 2.0 * (1.0 - PI / 4.0) * r * r * 40.0;
    assert_volume(volume(&e, "root"), rounds);
    assert_volume(volume(&e, "body"), 60.0 * 40.0 * 10.0 + 10.0 * 40.0 * 5.0 + rounds);
}

#[test]
fn a_fillet_takes_the_words_its_edges_call_for() {
    refused(
        &format!("{RECT}block := solid(sec, depth: 30mm)\nlip := fillet(block.near, block.bc, r: 5mm)\nlip union block\n"),
        Code::E085,
        "write `lip cut block`",
    );
    refused(
        &format!(
            "{RECT}{}plate := solid(sec, depth: 10mm)\nboss := solid(boss_f, from: 0mm, to: 20mm)\n\
             body := solid(plate)\nboss union body\nroot := fillet(boss, plate, r: 3mm)\nroot cut body\n",
            disc("boss_f", 8.0)
        ),
        Code::E085,
        "write `root union body`",
    );
    refused(
        &format!(
            "{RECT}block := solid(sec, depth: 30mm)\nlip := fillet(block.near, block.bc, r: 5mm)\n\
             body := solid(block)\nlip bound body\n"
        ),
        Code::E085,
        "not as its stock or a bound",
    );
    refused(
        &format!(
            "{RECT}block := solid(sec, depth: 30mm)\nlip := fillet(block.near, block.bc, r: 5mm)\n\
             nick := solid(sec, depth: 1mm)\nnick cut lip\n"
        ),
        Code::E085,
        "`lip` is a fillet, which takes no `cut`",
    );
    // a fillet reads what it rounds, so two reading each other are made of themselves
    refused(
        &format!(
            "{RECT}block := solid(sec, depth: 30mm)\none := fillet(two, block, r: 1mm)\n\
             two := fillet(one, block, r: 1mm)\n"
        ),
        Code::E041,
        "made of itself",
    );
}

#[test]
fn what_rung_one_cannot_round_is_refused_with_its_reason() {
    // a ball wider than the plate's margin about the boss
    refused(
        &format!(
            "{RECT}{}plate := solid(sec, depth: 10mm)\nboss := solid(boss_f, from: 0mm, to: 20mm)\n\
             body := solid(plate)\nboss union body\nroot := fillet(boss, plate, r: 13mm)\nroot union body\n",
            disc("boss_f", 8.0)
        ),
        Code::E085,
        "larger than `plate.near` can hold",
    );
    // a ball taller than the boss
    refused(
        &format!(
            "{RECT}{}plate := solid(sec, depth: 10mm)\nboss := solid(boss_f, from: 0mm, to: 2mm)\n\
             body := solid(plate)\nboss union body\nroot := fillet(boss, plate, r: 3mm)\nroot union body\n",
            disc("boss_f", 8.0)
        ),
        Code::E085,
        "larger than `boss.boss_f_k` can hold",
    );
    // a rib ending on the plate: the fillet would have to turn its corners
    refused(
        &format!("{}root := fillet(rib, plate, r: 2mm)\nroot union body\n", rib(5.0)),
        Code::E085,
        "is rung 3",
    );
    // faces that do not meet
    refused(
        &format!("{RECT}block := solid(sec, depth: 30mm)\nlip := fillet(block.ab, block.cd, r: 1mm)\nlip cut block\n"),
        Code::E085,
        "meet at no edge",
    );
    // a face the solid does not have
    refused(
        &format!("{RECT}block := solid(sec, depth: 30mm)\nlip := fillet(block.top, block.cd, r: 1mm)\nlip cut block\n"),
        Code::E085,
        "has no face `top`",
    );
}

#[test]
fn a_fillet_of_concave_and_convex_edges_is_two_fillets() {
    // a stepped turn: a disc of radius 40 with a hub of radius 15 on it, one section; its deck meets
    // the hub's wall in a concave ring and the disc's rim in a convex one
    let src = "\
unit mm
a0 := point
a1 := point hint(x: 0, y: 40)
fix(x == 0, y == 0) a0
fix(x == 0, y == 40) a1
spindle := line(a0, a1)
s0 := point
s1 := point hint(x: 40, y: 0)
s2 := point hint(x: 40, y: 10)
s3 := point hint(x: 15, y: 10)
s4 := point hint(x: 15, y: 30)
s5 := point hint(x: 0, y: 30)
fix(x == 0, y == 0) s0
fix(x == 40, y == 0) s1
fix(x == 40, y == 10) s2
fix(x == 15, y == 10) s3
fix(x == 15, y == 30) s4
fix(x == 0, y == 30) s5
(base := line(s0, s1)) -> (rim := line(s1, s2)) -> (deck := line(s2, s3)) -> (hub := line(s3, s4)) -> (top := line(s4, s5)) -> (spine := line(s5, s0)) -> close
stepped := solid(face(base, rim, deck, hub, top, spine), about: spindle)
both := fillet(stepped.deck, stepped, r: 2mm)
both union stepped
";
    refused(src, Code::E085, "so write two fillets");
}

#[test]
fn a_fillet_is_written_as_it_was_read() {
    let src = "root := fillet(boss, plate.near, r: 3mm)\n";
    let (mut prog, errs) = gcs_core::syntax::parse(src);
    assert!(errs.is_empty(), "{errs:?}");
    assert_eq!(gcs_core::syntax::render_flat(&mut prog).unwrap().trim(), src.trim());
    // coloured as the declaration word it is, as `solid` is
    let word = gcs_core::syntax::highlight(src).into_iter()
        .find(|(_, s)| &src[s.lo as usize..s.hi as usize] == "fillet");
    assert!(matches!(word, Some((gcs_core::syntax::Tint::Word, _))), "{word:?}");
    refused("root := fillet(boss, plate)\n", Code::E100, "needs `r:`");
    refused("root := fillet(boss, plate, r: 3mm, depth: 2mm)\n", Code::E100, "takes its two operands and `r:`");
    refused("root := solid(boss, r: 3mm)\n", Code::E100, "`r:` is a fillet's radius");
}

/// The material field reads a fillet's pieces exactly as the kernels build them: probes either side
/// of every face of the body's exact mesh find material inside and none outside.
fn field_agrees(e: &Elaborated, body: &str) {
    use gcs_core::solid::{agreement, MaterialField};
    let i = e.map.ent_named(body).unwrap().i();
    let x = e.sketch.exact_solid(i).unwrap();
    // a chord's sag well under the probes' offset and the confirming distance (millimetres)
    let m = gcs_core::brep::mesh::mesh(&x.brep, 1e-3 * x.mm, std::f64::consts::TAU / 64.0).unwrap();
    let v: Vec<[f64; 3]> = m.pts.iter().map(|p| std::array::from_fn(|k| p[k] / x.mm + x.origin[k])).collect();
    let mut material = MaterialField::read(&e.sketch, i, 1e-10).unwrap().evaluator(4096);
    let options = agreement::Options { offset: 0.02, confirm: 0.005, ..agreement::Options::default() };
    let a = agreement::of_triangles(&v, &m.tris, &mut material, &options).unwrap();
    assert!(a.agrees() && a.unresolved == 0 && a.probed_triangles > 20, "`{body}`: {a:?}");
}

#[test]
fn the_field_reads_a_filleted_body_as_the_kernel_builds_it() {
    let e = read(TURNED);
    field_agrees(&e, "body");
    let e = read(&format!(
        "{RECT}block := solid(sec, depth: 30mm)\nlip := fillet(block.near, block.bc, r: 5mm)\nlip cut block\n"
    ));
    field_agrees(&e, "block");
    let e = read(&format!("{}root := fillet(rib, plate, r: 2mm)\nroot union body\n", rib(0.0)));
    field_agrees(&e, "body");
}

// -- rung 2: sections with a curved side ------------------------------------------------------

/// One stroke of a section's loop: a segment, or the short arc about `centre` through `r`.
#[derive(Clone, Copy)]
enum Stroke { Line([f64; 2], [f64; 2]), Arc { centre: [f64; 2], r: f64, from: [f64; 2], to: [f64; 2] } }

impl Stroke {
    /// The arc's start angle and signed sweep, the short way round.
    fn turn(c: [f64; 2], from: [f64; 2], to: [f64; 2]) -> (f64, f64) {
        let a = (from[1] - c[1]).atan2(from[0] - c[0]);
        let mut sweep = (to[1] - c[1]).atan2(to[0] - c[0]) - a;
        while sweep > PI { sweep -= 2.0 * PI; }
        while sweep < -PI { sweep += 2.0 * PI; }
        (a, sweep)
    }
    /// `½ ∮ (x dy − y dx)` along it.
    fn area(&self) -> f64 {
        match *self {
            Stroke::Line(p, q) => 0.5 * (p[0] * q[1] - q[0] * p[1]),
            Stroke::Arc { centre: c, r, from, to } => {
                let (a, s) = Stroke::turn(c, from, to);
                0.5 * (r * r * s + c[0] * r * ((a + s).sin() - a.sin()) - c[1] * r * ((a + s).cos() - a.cos()))
            }
        }
    }
    /// `∮ ρ²/2 dz` along it, `(ρ, z)` its coordinates: the volume it bounds turned, over 2π.
    fn flux(&self) -> f64 {
        match *self {
            Stroke::Line(p, q) => (q[1] - p[1]) / 2.0 * (p[0] * p[0] + p[0] * q[0] + q[0] * q[0]) / 3.0,
            Stroke::Arc { centre: c, r, from, to } => {
                let (a, s) = Stroke::turn(c, from, to);
                let prim = |phi: f64| {
                    let (sn, cs) = phi.sin_cos();
                    (c[0] * c[0] * r * sn + c[0] * r * r * (phi + sn * cs) + r * r * r * (sn - sn.powi(3) / 3.0)) / 2.0
                };
                prim(a + s) - prim(a)
            }
        }
    }
}

/// A section's area, and the volume it sweeps turned once about `ρ = 0`.
fn section_area(loop_: &[Stroke]) -> f64 { loop_.iter().map(Stroke::area).sum::<f64>().abs() }
fn turned(loop_: &[Stroke]) -> f64 { 2.0 * PI * loop_.iter().map(Stroke::flux).sum::<f64>().abs() }

fn toward(c: [f64; 2], p: [f64; 2], r: f64) -> [f64; 2] {
    let (dx, dy) = (p[0] - c[0], p[1] - c[1]);
    let l = dx.hypot(dy);
    [c[0] + r * dx / l, c[1] + r * dy / l]
}

const KNOB: &str = include_str!("../../examples/solid_fillet_knob.sv");
const RAIL: &str = include_str!("../../examples/solid_fillet_rail.sv");

#[test]
fn a_rods_neck_in_a_ball_turns_a_section_with_a_curved_side() {
    // the rod's wall (ρ = 4) meets the ball (centre z 44, radius 8) at the corner; the ball of
    // radius 1.5 is 5.5 from the axis and 9.5 from the ball's centre
    let (s, big, rod, r): ([f64; 2], f64, f64, f64) = ([0.0, 44.0], 8.0, 4.0, 1.5);
    let corner = [rod, s[1] - (big * big - rod * rod).sqrt()];
    let centre = [rod + r, s[1] - ((big + r).powi(2) - (rod + r).powi(2)).sqrt()];
    let (on_rod, on_ball) = ([rod, centre[1]], toward(s, centre, big));
    let neck = turned(&[
        Stroke::Line(corner, on_rod),
        Stroke::Arc { centre, r, from: on_rod, to: on_ball },
        Stroke::Arc { centre: s, r: big, from: on_ball, to: corner },
    ]);
    let root = turned(&[
        Stroke::Line([4.0, 8.0], [6.0, 8.0]),
        Stroke::Arc { centre: [6.0, 10.0], r: 2.0, from: [6.0, 8.0], to: [4.0, 10.0] },
        Stroke::Line([4.0, 10.0], [4.0, 8.0]),
    ]);
    let e = read(KNOB);
    assert_volume(volume(&e, "neck"), neck);
    assert_volume(volume(&e, "root"), root);
    // the disc, the rod up to where it enters the ball, the ball less its cap below there (inside
    // the rod), and the two rings
    let cap = { let h = corner[1] - (s[1] - big); PI * h * h * (3.0 * big - h) / 3.0 };
    let want = PI * 30.0 * 30.0 * 8.0 + PI * rod * rod * (corner[1] - 8.0) + 4.0 / 3.0 * PI * big.powi(3) - cap
        + root + neck;
    assert_volume(volume(&e, "knob"), want);
    field_agrees(&e, "knob");
}

#[test]
fn a_rod_sunk_in_a_plate_is_filleted_along_both_sides() {
    // the plate's top (y = 10) meets the rod (centre (30, 10), radius 6) at x = 36 and 24; the
    // ball of radius 2 stands on the plate and 8 from the rod's centre
    let (c, rod, r): ([f64; 2], f64, f64) = ([30.0, 10.0], 6.0, 2.0);
    let corner = [36.0, 10.0];
    let centre = [c[0] + ((rod + r).powi(2) - r * r).sqrt(), 10.0 + r];
    let (on_plate, on_rod) = ([centre[0], 10.0], toward(c, centre, rod));
    let side = section_area(&[
        Stroke::Line(corner, on_plate),
        Stroke::Arc { centre, r, from: on_plate, to: on_rod },
        Stroke::Arc { centre: c, r: rod, from: on_rod, to: corner },
    ]);
    let e = read(RAIL);
    assert_volume(volume(&e, "sides"), 2.0 * side * 40.0);
    assert_volume(volume(&e, "rail"), 60.0 * 10.0 * 40.0 + PI * rod * rod / 2.0 * 40.0 + 2.0 * side * 40.0);
    field_agrees(&e, "rail");
}

#[test]
fn two_round_bars_side_by_side_are_filleted_at_their_waist() {
    let bars = "\
unit mm
o1 := point
o2 := point hint(x: 8, y: 0)
fix(x == 0, y == 0) o1
fix(x == 8, y == 0) o2
k1 := circle(center: o1) hint(r: 6)
k2 := circle(center: o2) hint(r: 6)
radius(6) k1
radius(6) k2
bar1 := solid(face(k1), from: 0mm, to: 30mm)
bar2 := solid(face(k2), from: 0mm, to: 30mm)
bars := solid(bar1)
bar2 union bars
waist := fillet(bar1, bar2, r: 1.5mm)
waist union bars
";
    // the circles meet at (4, ±√20); the ball's centre is 7.5 from both
    let (c1, c2, big, r): ([f64; 2], [f64; 2], f64, f64) = ([0.0, 0.0], [8.0, 0.0], 6.0, 1.5);
    let corner = [4.0, 20f64.sqrt()];
    let centre = [4.0, ((big + r).powi(2) - 16.0).sqrt()];
    let (t1, t2) = (toward(c1, centre, big), toward(c2, centre, big));
    let side = section_area(&[
        Stroke::Arc { centre: c1, r: big, from: corner, to: t1 },
        Stroke::Arc { centre, r, from: t1, to: t2 },
        Stroke::Arc { centre: c2, r: big, from: t2, to: corner },
    ]);
    let lens = 2.0 * big * big * (8.0 / (2.0 * big)).acos() - 4.0 * (4.0 * big * big - 64.0).sqrt();
    let e = read(bars);
    assert_volume(volume(&e, "waist"), 2.0 * side * 30.0);
    assert_volume(volume(&e, "bars"), (2.0 * PI * big * big - lens + 2.0 * side) * 30.0);
    field_agrees(&e, "bars");
}

#[test]
fn what_rung_two_cannot_round_yet_is_refused_with_its_reason() {
    // a ball so large it would touch the rod below the disc it stands on
    refused(
        &KNOB.replace("neck := fillet(rod, ball, r: 1.5mm)", "neck := fillet(rod, ball, r: 200mm)"),
        Code::E085,
        "can hold",
    );
    // a rod stopping short of the plate's ends: the fillet would have to turn its corners
    refused(
        &RAIL.replace("rod := solid(face(k), from: 0mm, to: 40mm)", "rod := solid(face(k), from: 5mm, to: 35mm)"),
        Code::E085,
        "rung 3",
    );
    // a branch so short the ball would roll up past its end
    refused(&TEE.replace("from: 0mm, to: 25mm", "from: 0mm, to: 11mm"), Code::E085, "can hold");
    // a branch at the main pipe's end: its crotch runs out onto the end, where the branch's foot on
    // the end, rounded too, meets it
    refused(&TEE.replace("fix(x == 0, y == 0) c2", "fix(x == 27, y == 0) c2").replace("c2 := point\n", "c2 := point hint(x: 27)\n"),
        Code::E085, "meets a traced run of the same fillet where it ends");
    // a face swept from a spline has no offset in closed form
    refused(
        &format!("{}round := fillet(plate.lobe, plate.near, r: 1mm)\n", include_str!("../../examples/solid_spline.sv")),
        Code::E085,
        "no offset in closed form",
    );
}

/// The field reads a ball's material by the nearest point of its centre's path, so it refuses a
/// corner reaching past where that point is unique: a branch so thin the crotch's ball reaches
/// across the bend of its own path.
#[test]
fn a_canal_reaching_past_its_spines_bend_is_refused_by_the_field() {
    let e = read_linked(&TEE.replace("radius(6) stem_k", "radius(1) stem_k").replace("hint(r: 6)", "hint(r: 1)"));
    let i = e.map.ent_named("tee").unwrap().i();
    let err = gcs_core::solid::MaterialField::read(&e.sketch, i, 1e-10).err().expect("the field reads it");
    assert!(format!("{err:?}").contains("nearest point is unique"), "{err:?}");
}

// -- rung 2: balls rolled along traced loops ---------------------------------------------------

type V3 = [f64; 3];

/// The least positive root of `a t² + b t + c` (infinite where there is none).
fn first_root(a: f64, b: f64, c: f64) -> f64 {
    let disc = b * b - 4.0 * a * c;
    if disc < 0.0 { return f64::INFINITY; }
    [(-b - disc.sqrt()) / (2.0 * a), (-b + disc.sqrt()) / (2.0 * a)].into_iter().filter(|&t| t > 1e-12).fold(f64::INFINITY, f64::min)
}

/// Where a ray from `o` along `w` first meets the cylinder of `radius` about the line through `at`
/// along the axis `axis` (0, 1 or 2).
fn ray_cylinder(o: V3, w: V3, at: V3, axis: usize, radius: f64) -> f64 {
    let o = sub(o, at);
    let (i, j) = ((axis + 1) % 3, (axis + 2) % 3);
    first_root(w[i] * w[i] + w[j] * w[j], 2.0 * (o[i] * w[i] + o[j] * w[j]), o[i] * o[i] + o[j] * o[j] - radius * radius)
}

/// Where a ray from `o` along the unit `w` first meets the sphere of `radius` about the origin.
fn ray_sphere(o: V3, w: V3, radius: f64) -> f64 { first_root(1.0, 2.0 * dot(o, w), dot(o, o) - radius * radius) }

/// A ball of radius `r` rolled round a closed spine `c(φ)`, φ over a turn, independently of the
/// kernel: in each plane square to the spine the fillet's section is the cone between the
/// directions `toward` its two contacts, out from the ball to whichever face a ray meets first
/// (`hit`, the distance along a ray), swept with the spine's curvature (`dV = (1 − κ q) dA ds`,
/// `q` along its principal normal).
fn rolled(c: &dyn Fn(f64) -> V3, toward: [&dyn Fn(V3) -> V3; 2], hit: [&dyn Fn(V3, V3) -> f64; 2], r: f64) -> f64 {
    let (n_phi, n_psi) = (720, 96);
    // Gauss–Legendre on [−1, 1], by Newton on the Legendre polynomial (the kernel's own rule is not
    // borrowed: the reference reads nothing of it)
    let gauss: Vec<(f64, f64)> = (0..n_psi).map(|i| {
        let mut x = (PI * (i as f64 + 0.75) / (n_psi as f64 + 0.5)).cos();
        let mut dp = 0.0;
        for _ in 0..100 {
            let (mut p0, mut p1) = (1.0, x);
            for k in 2..=n_psi { let p2 = ((2 * k - 1) as f64 * x * p1 - (k - 1) as f64 * p0) / k as f64; p0 = p1; p1 = p2; }
            dp = n_psi as f64 * (x * p1 - p0) / (x * x - 1.0);
            let dx = p1 / dp;
            x -= dx;
            if dx.abs() < 1e-16 { break; }
        }
        (x, 2.0 / ((1.0 - x * x) * dp * dp))
    }).collect();
    let h = 1e-4;
    let mut total = 0.0;
    for k in 0..n_phi {
        let phi = 2.0 * PI * k as f64 / n_phi as f64;
        let p = c(phi);
        let (pa, pb) = (c(phi - h), c(phi + h));
        let d1 = scale(sub(pb, pa), 1.0 / (2.0 * h));
        let d2 = scale([pb[0] - 2.0 * p[0] + pa[0], pb[1] - 2.0 * p[1] + pa[1], pb[2] - 2.0 * p[2] + pa[2]], 1.0 / (h * h));
        let speed = dot(d1, d1).sqrt();
        let t = scale(d1, 1.0 / speed);
        // κ N, the turn of the tangent per unit length
        let kn = scale(sub(d2, scale(t, dot(d2, t))), 1.0 / (speed * speed));
        let (ea, eb) = (toward[0](p), toward[1](p));
        let span = dot(ea, eb).clamp(-1.0, 1.0).acos();
        let side = unit(sub(eb, scale(ea, dot(ea, eb))));
        let dir = |psi: f64| [0, 1, 2].map(|i| ea[i] * psi.cos() + side[i] * psi.sin());
        // the ray leaves through the first face from the first contact's side, the second from the
        // second's: the angle where the two meet splits the integral (a kink no rule integrates)
        let switch = {
            let gap = |psi: f64| hit[0](p, dir(psi)) - hit[1](p, dir(psi));
            let (mut lo, mut hi) = (0.0, span);
            for _ in 0..80 { let m = 0.5 * (lo + hi); if (gap(m) < 0.0) == (gap(lo) < 0.0) { lo = m } else { hi = m } }
            0.5 * (lo + hi)
        };
        let mut area = 0.0;
        for (a, b) in [(0.0, switch), (switch, span)] {
            for &(x, w) in &gauss {
                let psi = a + 0.5 * (b - a) * (x + 1.0);
                let d = dir(psi);
                let reach = hit[0](p, d).min(hit[1](p, d));
                let q = dot(d, kn);
                area += w * 0.5 * (b - a) * ((reach * reach - r * r) / 2.0 - q * (reach.powi(3) - r.powi(3)) / 3.0);
            }
        }
        total += area * speed * 2.0 * PI / n_phi as f64;
    }
    total
}

/// The tee's crotch fillet: its spine is where the cylinders offset by the ball's radius meet
/// (radii `big + r` about x, `small + r` about z), in closed form round the branch.
fn tee_crotch(big: f64, small: f64, r: f64) -> f64 {
    let (rb, rs) = (big + r, small + r);
    rolled(
        &|phi: f64| { let (s, co) = phi.sin_cos(); [rs * co, rs * s, (rb * rb - rs * rs * s * s).sqrt()] },
        [&|p: V3| unit([0.0, -p[1], -p[2]]), &|p: V3| unit([-p[0], -p[1], 0.0])],
        [&|o, w| ray_cylinder(o, w, [0.0; 3], 0, big), &|o, w| ray_cylinder(o, w, [0.0; 3], 2, small)],
        r,
    )
}

const TEE: &str = include_str!("../../examples/solid_fillet_tee.sv");

/// A document with `use std`, linked as the CLI links it.
fn read_linked(src: &str) -> Elaborated {
    let (prog, errs, _) = gcs_core::library::parse_linked(src);
    assert!(errs.is_empty(), "does not parse: {errs:?}");
    let e = elaborate(&prog);
    assert!(e.ok(), "does not elaborate: {:?}", e.diags.iter().map(|d| d.message.clone()).collect::<Vec<_>>());
    e
}

#[test]
fn a_pipe_tees_crotch_is_rolled_round_its_traced_loop() {
    let e = read_linked(TEE);
    let want = tee_crotch(10.0, 6.0, 2.0);
    let got = volume(&e, "crotch");
    // the canal is fitted, within a ten-millionth of the part
    assert!((got - want).abs() <= 1e-6 * want, "{got} != {want} (off by {:e})", got - want);
    // the body is the pipes' union and the fillet, exactly
    assert_volume(volume(&e, "tee"), volume(&e, "main") + volume(&e, "stem")
        - PI * 36.0 * 10.0 + volume(&e, "crotch") - stem_inside_main(10.0, 6.0));
    field_agrees(&e, "tee");
}

/// How much less than a cylinder of the main pipe's radius the branch's foot inside the main pipe
/// is: `∫∫ (√(big² − y²) − big)` over the branch's disc, by `y = small sin θ` (smooth and periodic
/// in θ, so the trapezoid rule converges at once).
fn stem_inside_main(big: f64, small: f64) -> f64 {
    let n = 2000;
    (0..n).map(|i| {
        let th = -PI / 2.0 + PI * i as f64 / n as f64;
        let (s, c) = th.sin_cos();
        let y = small * s;
        2.0 * small * c * ((big * big - y * y).sqrt() - big) * small * c
    }).sum::<f64>() * PI / n as f64
}

/// A pin of radius 2 through the knob's ball (radius 8) 3 off its centre, square to the knob's axis.
const PIN: &str = "\
in std.front {
  pc := point hint(x: 3, y: 44)
  fix(x == 3, y == 44) pc
  pin_k := circle(center: pc) hint(r: 2)
  radius(2) pin_k
}
pin := solid(face(pin_k), from: -20mm, to: 20mm)
";

/// One of the two fillets where the pin leaves the ball, independently: about the ball's centre,
/// the pin's axis along z at `off` in x, the spine is where the sphere of `big + r` meets the
/// cylinder of `small + r`.
fn pin_collar(big: f64, small: f64, off: f64, r: f64) -> f64 {
    let (rb, rs) = (big + r, small + r);
    rolled(
        &|phi: f64| {
            let (s, co) = phi.sin_cos();
            let (x, y) = (off + rs * co, rs * s);
            [x, y, (rb * rb - x * x - y * y).sqrt()]
        },
        [&|p: V3| unit(scale(p, -1.0)), &|p: V3| unit([off - p[0], -p[1], 0.0])],
        [&|o, w| ray_sphere(o, w, big), &|o, w| ray_cylinder(o, w, [off, 0.0, 0.0], 2, small)],
        r,
    )
}

#[test]
fn a_pin_through_a_ball_off_its_centre_is_rolled_round_both_loops() {
    let e = read(&format!(
        "{KNOB}{PIN}pinned := solid(knob)\npin union pinned\nnub := fillet(pin, ball, r: 0.5mm)\n\
         nubbed := solid(pinned)\nnub union nubbed\n"
    ));
    // one fillet of both loops, each a collar where the pin leaves the ball
    let want = 2.0 * pin_collar(8.0, 2.0, 3.0, 0.5);
    let got = volume(&e, "nub");
    assert!((got - want).abs() <= 1e-6 * want, "{got} != {want} (off by {:e})", got - want);
    assert_volume(volume(&e, "nubbed"), volume(&e, "pinned") + got);
    field_agrees(&e, "nubbed");
}

const BORE: &str = include_str!("../../examples/solid_fillet_bore.sv");

/// The rim's material where a bore of radius `small` is drilled square into a pipe of radius `big`:
/// the crotch's mirror, the ball rolling inside the pipe's wall and outside the bore, its centre on
/// the cylinders of radii `big − r` and `small + r`.
fn bore_rim(big: f64, small: f64, r: f64) -> f64 {
    rolled(
        &|phi: f64| {
            let (s, co) = phi.sin_cos();
            let (rb, rs) = (big - r, small + r);
            [rs * co, rs * s, (rb * rb - rs * rs * s * s).sqrt()]
        },
        [&|p: V3| unit([0.0, p[1], p[2]]), &|p: V3| unit([-p[0], -p[1], 0.0])],
        [&|o, w| ray_cylinder(o, w, [0.0; 3], 0, big), &|o, w| ray_cylinder(o, w, [0.0; 3], 2, small)],
        r,
    )
}

#[test]
fn a_bores_rim_is_rolled_off_round_its_saddle() {
    let e = read_linked(BORE);
    let want = bore_rim(10.0, 6.0, 1.0);
    let got = volume(&e, "rim");
    assert!((got - want).abs() <= 1e-6 * want, "{got} != {want} (off by {:e})", got - want);
    assert_volume(volume(&e, "pipe"), volume(&e, "drilled") - got);
    field_agrees(&e, "pipe");
}

// -- rung 3: a traced run that ends ------------------------------------------------------------

/// A rectangle in `std.side` (its own `y`, `z` as `x`, `y`), swept from the plane `x = 0` to
/// `x = 40`: the half of space past it, within reach of the pipes, as `half`.
const HALF: &str = "\
in std.side {
  h0 := point hint(x: -30, y: -30)
  h1 := point hint(x: 30, y: -30)
  h2 := point hint(x: 30, y: 40)
  h3 := point hint(x: -30, y: 40)
  (hs := line(h0, h1)) -> (he := line(h1, h2)) -> (hn := line(h2, h3)) -> (hw := line(h3, h0)) -> close
  half_f := face(hs, he, hn, hw)
}
fix(x == -30, y == -30) h0
fix(x == 30, y == -30) h1
fix(x == 30, y == 40) h2
fix(x == -30, y == 40) h3
half := solid(half_f, from: 0mm, to: 40mm)
";

/// `src` with `HALF` written before `at` and `then` in place of it.
fn halved(src: &str, at: &str, then: &str) -> String {
    assert!(src.contains(at), "no `{at}`");
    src.replacen(at, &format!("{HALF}{then}"), 1)
}

#[test]
fn half_a_tees_crotch_runs_out_onto_the_face_it_was_cut_by() {
    // the tee cut in half along the plane through both pipes' axes: its crotch an open run, ending
    // square at that plane at both ends, the ball rolled on past and cut off there
    let e = read_linked(&halved(TEE, "tee := solid(main)\nstem union tee\ncrotch := fillet(stem, main, r: 2mm)\n",
        "halved := solid(main)\nstem union halved\nhalf bound halved\ntee := solid(halved)\n\
         crotch := fillet(halved.stem, halved.main, r: 2mm)\n"));
    let want = tee_crotch(10.0, 6.0, 2.0) / 2.0;
    let got = volume(&e, "crotch");
    assert!((got - want).abs() <= 1e-6 * want, "{got} != {want} (off by {:e})", got - want);
    // the field's fillet stops at the plane too: in the crotch's corner on the kept side, not past it
    let crotch = gcs_core::solid::MaterialField::read(&e.sketch, e.map.ent_named("crotch").unwrap().i(), 1e-10).unwrap();
    // (a tenth from the corner where the cutting plane meets both pipes, toward the ball)
    assert!(crotch.side([0.3, 6.09, 8.04]) < 0.0 && crotch.side([-0.3, 6.09, 8.04]) > 0.0);
    field_agrees(&e, "tee");
}

#[test]
fn half_a_bores_rim_runs_out_into_the_void_past_the_face_it_was_cut_by() {
    // convex: the ball rolled on past the plane takes away nothing more there, and is cut off too
    let e = read_linked(&halved(BORE, "pipe := solid(drilled)\n", "half bound drilled\npipe := solid(drilled)\n"));
    let want = bore_rim(10.0, 6.0, 1.0) / 2.0;
    let got = volume(&e, "rim");
    assert!((got - want).abs() <= 1e-6 * want, "{got} != {want} (off by {:e})", got - want);
    assert_volume(volume(&e, "pipe"), volume(&e, "drilled") - got);
    field_agrees(&e, "pipe");
}

/// A wedge 40 long, its top sloping from 20 high at `x = 0` to 10 at `x = 40`, drawn in `std.front`
/// and swept 24 deep (`y` from −24 to 0), with a boss of radius 6 standing on the floor through the
/// slope at `(20, 0)`, centred on the wedge's near face, its root on the slope rounded to 2: `body`,
/// `root`.
const SLOPED: &str = "unit mm
use std
in std.front {
  w0 := point
  w1 := point hint(x: 40, y: 0)
  w2 := point hint(x: 40, y: 10)
  w3 := point hint(x: 0, y: 20)
  (wb := line(w0, w1)) -> (we := line(w1, w2)) -> (wt := line(w2, w3)) -> (ww := line(w3, w0)) -> close
  side_f := face(wb, we, wt, ww)
}
fix(x == 0, y == 0) w0
fix(x == 40, y == 0) w1
fix(x == 40, y == 10) w2
fix(x == 0, y == 20) w3
in std.top {
  bc := point hint(x: 20, y: 0)
  boss_k := circle(center: bc) hint(r: 6)
}
fix(x == 20, y == 0) bc
radius(6) boss_k
wedge := solid(side_f, from: 0mm, to: 24mm)
boss := solid(face(boss_k), from: 0mm, to: 30mm)
body := solid(wedge)
boss union body
root := fillet(boss, wedge.wt, r: 2mm)
root union body
";

/// The fillet a ball of radius `r` fills round a boss of radius `big` about `(cx, cy)` standing
/// through the plane `z = h0 − s x`, the whole way round, independently: its spine is where the
/// plane offset by `r` meets the cylinder of `big + r`, an ellipse.
fn sloped_root(big: f64, [cx, cy]: [f64; 2], [h0, s]: [f64; 2], r: f64) -> f64 {
    let n = unit([s, 0.0, 1.0]);
    let lift = r / n[2];
    rolled(
        &|phi: f64| {
            let (sn, co) = phi.sin_cos();
            let x = cx + (big + r) * co;
            [x, cy + (big + r) * sn, h0 - s * x + lift]
        },
        [&|_: V3| scale(n, -1.0), &|p: V3| unit([cx - p[0], cy - p[1], 0.0])],
        [&|o: V3, w: V3| { let d = dot(w, n); if d < 0.0 { -(dot(o, n) - h0 * n[2]) / d } else { f64::INFINITY } },
            &|o, w| ray_cylinder(o, w, [cx, cy, 0.0], 2, big)],
        r,
    )
}

#[test]
fn a_boss_on_a_slope_standing_off_its_edge_is_rounded_to_the_edge() {
    // the boss centred on the wedge's near face: its root an arc of an ellipse, ending square at
    // that face at both ends, half the root of a boss standing in the slope's middle
    let e = read_linked(SLOPED);
    let want = sloped_root(6.0, [20.0, 0.0], [20.0, 0.25], 2.0) / 2.0;
    let got = volume(&e, "root");
    assert!((got - want).abs() <= 1e-6 * want, "{got} != {want} (off by {:e})", got - want);
    field_agrees(&e, "body");
}

#[test]
fn what_a_run_that_ends_cannot_round_is_refused() {
    // the tee bound by a cylinder across the main pipe: its crotch cut off by a curved face
    let curved = TEE.replace("tee := solid(main)\nstem union tee\ncrotch := fillet(stem, main, r: 2mm)\n",
        "in std.front {\n  rc := point hint(x: -6, y: 0)\n  round_k := circle(center: rc) hint(r: 14)\n}\n\
         fix(x == -6, y == 0) rc\nradius(14) round_k\nround := solid(face(round_k), from: -30mm, to: 30mm)\n\
         halved := solid(main)\nstem union halved\nround bound halved\ntee := solid(halved)\n\
         crotch := fillet(halved.stem, halved.main, r: 2mm)\n");
    refused(&curved, Code::E085, "ending on a curved face is rung 3");
    // the half tee cut at the crotch's height too, both planes through its ends: each a corner
    let cornered = halved(TEE, "tee := solid(main)\nstem union tee\ncrotch := fillet(stem, main, r: 2mm)\n",
        "in std.front {\n  l0 := point hint(x: -40, y: 8)\n  l1 := point hint(x: 40, y: 8)\n  l2 := point hint(x: 40, y: 40)\n\
         l3 := point hint(x: -40, y: 40)\n  (ls := line(l0, l1)) -> (le := line(l1, l2)) -> (ln := line(l2, l3)) -> (lw := line(l3, l0)) -> close\n\
         lift_f := face(ls, le, ln, lw)\n}\nfix(x == -40, y == 8) l0\nfix(x == 40, y == 8) l1\nfix(x == 40, y == 40) l2\n\
         fix(x == -40, y == 40) l3\nlift := solid(lift_f, from: -40mm, to: 40mm)\n\
         halved := solid(main)\nstem union halved\nhalf bound halved\nlift bound halved\ntee := solid(halved)\n\
         crotch := fillet(halved.stem, halved.main, r: 2mm)\n");
    refused(&cornered, Code::E085, "a fillet ending at a corner is rung 3");
    // the boss's foot on the wedge's near face rounded too: two straight runs up the face meet the
    // root where it is cut off
    refused(&SLOPED.replace("fillet(boss, wedge.wt", "fillet(boss, wedge"), Code::E085,
        "meets a traced run of the same fillet where it ends");
}

const RUNOUT: &str = include_str!("../../examples/solid_fillet_runout.sv");

#[test]
fn a_fillet_cut_off_obliquely_by_the_face_it_runs_out_onto_reads_as_built() {
    // the boss off the near face and the hole through the far one: each run ends where the face
    // cuts it obliquely; the bodies are their operands and fillets, exactly, and the fields agree
    let e = read_linked(RUNOUT);
    let (root, rim) = (volume(&e, "root"), volume(&e, "rim"));
    assert!(root > 0.0 && rim > 0.0);
    assert_volume(volume(&e, "vent"), volume(&e, "drilled") - rim);
    field_agrees(&e, "lug");
    field_agrees(&e, "vent");
}

// -- rung 3: fillets carried along chains of tangent edges -------------------------------------

/// A rectangle `w` × `h` about `(cx, cy)` with its corners rounded to `rr`, every point fixed, as
/// face `{n}`: the sides straight runs between the corners' quarter circles, a side of no length
/// left out (an obround's ends are half circles).
fn rounded(n: &str, [cx, cy]: [f64; 2], [w, h]: [f64; 2], rr: f64) -> String {
    let (x0, x1, y0, y1) = (cx - w / 2.0, cx + w / 2.0, cy - h / 2.0, cy + h / 2.0);
    let mut out = String::new();
    let mut point = |p: &str, x: f64, y: f64| {
        out.push_str(&format!("{n}_{p} := point hint(x: {x}, y: {y})\nfix(x == {x}, y == {y}) {n}_{p}\n"));
    };
    // corners' centres, and each side's two ends, counter-clockwise from the bottom
    let centres = [("br", x1 - rr, y0 + rr), ("tr", x1 - rr, y1 - rr), ("tl", x0 + rr, y1 - rr), ("bl", x0 + rr, y0 + rr)];
    for (c, x, y) in centres { point(&format!("c{c}"), x, y); }
    let sides = [("b", [x0 + rr, y0], [x1 - rr, y0]), ("r", [x1, y0 + rr], [x1, y1 - rr]),
        ("t", [x1 - rr, y1], [x0 + rr, y1]), ("l", [x0, y1 - rr], [x0, y0 + rr])];
    for (s, a, b) in sides { point(&format!("{s}0"), a[0], a[1]); point(&format!("{s}1"), b[0], b[1]); }
    // the loop: each side that has a length, then the corner after it, from where the last ended
    // to where the next side starts
    let mut links: Vec<(String, String)> = Vec::new();
    let mut from = format!("{n}_b0");
    for (k, (s, a, b)) in sides.iter().enumerate() {
        if (a[0] - b[0]).abs() + (a[1] - b[1]).abs() > 1e-12 {
            links.push((format!("{n}_{s}"), format!("line({from}, {n}_{s}1)")));
            from = format!("{n}_{s}1");
        }
        let (c, _, _) = centres[k];
        let to = format!("{n}_{}0", sides[(k + 1) % 4].0);
        links.push((format!("{n}_a{c}"), format!("arc(center: {n}_c{c}, start: {from}, end: {to}) hint(r: {rr})")));
        from = to;
    }
    for (name, decl) in &links { out.push_str(&format!("{name} := {decl}\n")); }
    let names: Vec<&str> = links.iter().map(|(name, _)| name.as_str()).collect();
    out.push_str(&format!("{n} := face({})\n", names.join(", ")));
    out
}

/// The straight runs' share of a rounded outline's foot fillet: its perimeter less the corners'.
fn runs([w, h]: [f64; 2], rr: f64) -> f64 { 2.0 * (w - 2.0 * rr) + 2.0 * (h - 2.0 * rr) }

#[test]
fn a_rounded_bosss_foot_is_carried_round_its_corners() {
    let (size, rr, r) = ([30.0, 20.0], 4.0, 2.0);
    let e = read(&format!(
        "{RECT}{}plate := solid(sec, depth: 10mm)\nboss := solid(boss_f, from: 0mm, to: 20mm)\n\
         body := solid(plate)\nboss union body\nfoot := fillet(boss, plate, r: {r}mm)\nfoot union body\n",
        rounded("boss_f", [30.0, 20.0], size, rr)
    ));
    // the straight runs, and the four quarter rings: one whole ring about a corner's axis
    let want = (1.0 - PI / 4.0) * r * r * runs(size, rr) + root_ring(rr, r);
    assert_volume(volume(&e, "foot"), want);
    let boss = (size[0] * size[1] - (4.0 - PI) * rr * rr) * 20.0;
    assert_volume(volume(&e, "body"), 60.0 * 40.0 * 10.0 + boss + want);
    field_agrees(&e, "body");
}

#[test]
fn an_obround_bosss_foot_runs_round_its_half_circles() {
    // the ends' quarter circles meet at the end's middle, two to a half circle
    let (size, rr, r) = ([30.0, 12.0], 6.0, 2.0);
    let e = read(&format!(
        "{RECT}{}plate := solid(sec, depth: 10mm)\nboss := solid(boss_f, from: 0mm, to: 8mm)\n\
         body := solid(plate)\nboss union body\nfoot := fillet(boss, plate, r: {r}mm)\nfoot union body\n",
        rounded("boss_f", [30.0, 20.0], size, rr)
    ));
    let want = (1.0 - PI / 4.0) * r * r * runs(size, rr) + root_ring(rr, r);
    assert_volume(volume(&e, "foot"), want);
    let boss = (size[0] * size[1] - (4.0 - PI) * rr * rr) * 8.0;
    assert_volume(volume(&e, "body"), 60.0 * 40.0 * 10.0 + boss + want);
}

#[test]
fn a_rounded_pockets_rim_is_rolled_off_round_its_corners() {
    // the rim's section at a corner stands outside the corner's radius, as a boss's root does
    let (size, rr, r) = ([30.0, 20.0], 5.0, 1.0);
    let e = read(&format!(
        "{RECT}{}plate := solid(sec, depth: 10mm)\npocket := solid(pocket_f, from: 1mm, to: -6mm)\n\
         cupped := solid(plate)\npocket cut cupped\nbody := solid(cupped)\n\
         rim := fillet(cupped.plate, cupped.pocket, r: {r}mm)\nrim cut body\n",
        rounded("pocket_f", [30.0, 20.0], size, rr)
    ));
    let want = (1.0 - PI / 4.0) * r * r * runs(size, rr) + root_ring(rr, r);
    assert_volume(volume(&e, "rim"), want);
    let pocket = (size[0] * size[1] - (4.0 - PI) * rr * rr) * 6.0;
    assert_volume(volume(&e, "cupped"), 60.0 * 40.0 * 10.0 - pocket);
    assert_volume(volume(&e, "body"), 60.0 * 40.0 * 10.0 - pocket - want);
    field_agrees(&e, "body");
}

#[test]
fn a_chain_that_turns_a_corner_or_crowds_a_ring_is_refused() {
    let doc = |boss: &str, more: &str, plate: &str| format!(
        "{RECT}{boss}{more}plate := solid(sec, depth: 10mm)\nboss := solid(boss_f, from: 0mm, to: 8mm)\n\
         body := solid({plate})\nboss union body\nfoot := fillet(boss, {plate}, r: 2mm)\nfoot union body\n");
    // a square boss: its foot's runs meet at corners no ball rolls round without turning, the
    // boss's upright edge convex where the foot is concave
    let square = "q0 := point hint(x: 15, y: 10)\nq1 := point hint(x: 45, y: 10)\nq2 := point hint(x: 45, y: 30)\n\
        q3 := point hint(x: 15, y: 30)\nfix(x == 15, y == 10) q0\nfix(x == 45, y == 10) q1\nfix(x == 45, y == 30) q2\n\
        fix(x == 15, y == 30) q3\n(s0 := line(q0, q1)) -> (s1 := line(q1, q2)) -> (s2 := line(q2, q3)) -> (s3 := line(q3, q0)) -> close\n\
        boss_f := face(s0, s1, s2, s3)\n";
    refused(&doc(square, "", "plate"), Code::E085, "rung 3");
    // a hole drilled in the plate under a rounded corner's quarter ring, within the band it rolls on
    let hole = "h_o := point hint(x: 44.5, y: 10.5)\nfix(x == 44.5, y == 10.5) h_o\nh_k := circle(center: h_o) hint(r: 0.5)\n\
        radius(0.5) h_k\nhole := solid(face(h_k), from: 1mm, to: -11mm)\n";
    let drilled = doc(&rounded("boss_f", [30.0, 20.0], [30.0, 20.0], 4.0), hole, "holed")
        .replace("body := solid(holed)", "holed := solid(plate)\nhole cut holed\nbody := solid(holed)");
    refused(&drilled, Code::E085, "can hold");
}

#[test]
fn a_rib_built_in_two_halves_is_rounded_as_one() {
    // the halves' sides meet flat, so each foot is two straight edges end to end: two runs, their
    // sections one at the joint
    let r = 2.0;
    let half = |n: &str, lo: f64, hi: f64| format!(
        "{n}0 := point hint(x: 20, y: {lo})\n{n}1 := point hint(x: 30, y: {lo})\n{n}2 := point hint(x: 30, y: {hi})\n\
         {n}3 := point hint(x: 20, y: {hi})\nfix(x == 20, y == {lo}) {n}0\nfix(x == 30, y == {lo}) {n}1\n\
         fix(x == 30, y == {hi}) {n}2\nfix(x == 20, y == {hi}) {n}3\n\
         ({n}e0 := line({n}0, {n}1)) -> ({n}e1 := line({n}1, {n}2)) -> ({n}e2 := line({n}2, {n}3)) -> ({n}e3 := line({n}3, {n}0)) -> close\n\
         {n}_f := face({n}e0, {n}e1, {n}e2, {n}e3)\n{n} := solid({n}_f, from: 0mm, to: 5mm)\n");
    let e = read(&format!(
        "{RECT}{}{}plate := solid(sec, depth: 10mm)\nrib := solid(front)\nback union rib\n\
         body := solid(plate)\nrib union body\nroot := fillet(rib, plate, r: {r}mm)\nroot union body\n",
        half("front", 0.0, 20.0), half("back", 20.0, 40.0)
    ));
    let i = e.map.ent_named("root").unwrap().i();
    assert_eq!(e.sketch.fillet_blend(i).unwrap().pieces.len(), 4, "each foot split at the joint");
    let rounds = 2.0 * (1.0 - PI / 4.0) * r * r * 40.0;
    assert_volume(volume(&e, "root"), rounds);
    assert_volume(volume(&e, "body"), 60.0 * 40.0 * 10.0 + 10.0 * 40.0 * 5.0 + rounds);
    field_agrees(&e, "body");
}

// -- rung 3: spherical corners -----------------------------------------------------------------

/// A box `l × w × h` with every edge rounded to `r`: Steiner's formula, the inset box grown by a
/// ball — its volume, its faces carried out by `r`, its edges' quarter cylinders, the ball itself.
fn rounded_box([l, w, h]: [f64; 3], r: f64) -> f64 {
    let (a, b, c) = (l - 2.0 * r, w - 2.0 * r, h - 2.0 * r);
    a * b * c + 2.0 * r * (a * b + b * c + a * c) + PI * r * r * (a + b + c) + 4.0 * PI * r * r * r / 3.0
}

#[test]
fn a_block_rounded_all_over_has_a_ball_at_each_corner() {
    let r = 2.0;
    let e = read(&format!("{RECT}block := solid(sec, depth: 10mm)\nall := fillet(block, block, r: {r}mm)\n\
        body := solid(block)\nall cut body\n"));
    let i = e.map.ent_named("all").unwrap().i();
    let blend = e.sketch.fillet_blend(i).unwrap();
    assert_eq!((blend.pieces.len(), blend.corners.len(), blend.joins.len()), (12, 8, 24));
    // glued whole: each corner meets three runs, which meet one another only through it
    let built = gcs_core::brep::recipe::build(&gcs_core::solid::cad::recipe(&e.sketch, i).unwrap()).unwrap();
    built.check(1e-9).unwrap();
    assert_volume(volume(&e, "body"), rounded_box([60.0, 40.0, 10.0], r));
    assert_volume(volume(&e, "all"), 60.0 * 40.0 * 10.0 - rounded_box([60.0, 40.0, 10.0], r));
    field_agrees(&e, "body");
}

/// A polygon of fixed points as face `{n}`.
fn polygon(n: &str, pts: &[[f64; 2]]) -> String {
    let mut out = String::new();
    for (k, p) in pts.iter().enumerate() {
        out.push_str(&format!("{n}{k} := point hint(x: {}, y: {})\nfix(x == {}, y == {}) {n}{k}\n", p[0], p[1], p[0], p[1]));
    }
    let sides: Vec<String> = (0..pts.len()).map(|k| format!("({n}_s{k} := line({n}{k}, {n}{}))", (k + 1) % pts.len())).collect();
    out.push_str(&format!("{} -> close\n", sides.join(" -> ")));
    out.push_str(&format!("{n} := face({})\n", (0..pts.len()).map(|k| format!("{n}_s{k}")).collect::<Vec<_>>().join(", ")));
    out
}

#[test]
fn a_pocket_rounded_inside_fills_a_ball_at_each_floor_corner() {
    // the floor's edges and the walls' corners, concave: runs, and at each floor corner a ball's
    // patch filling the corner's cell; the corners' runs stop flush in the plate's top
    let (r, [l, w], d) = (1.5, [30.0, 20.0], 6.0);
    let e = read(&format!(
        "{RECT}{}plate := solid(sec, depth: 10mm)\npocket := solid(cut_f, from: 1mm, to: -{d}mm)\n\
         cupped := solid(plate)\npocket cut cupped\nbody := solid(cupped)\n\
         inside := fillet(cupped.pocket, cupped.pocket, r: {r}mm)\ninside union body\n",
        polygon("cut_f", &[[15.0, 10.0], [45.0, 10.0], [45.0, 30.0], [15.0, 30.0]])
    ));
    let run = (1.0 - PI / 4.0) * r * r;
    let want = 4.0 * run * (d - r) + run * (2.0 * (l - 2.0 * r) + 2.0 * (w - 2.0 * r)) + 4.0 * (r * r * r - PI * r * r * r / 6.0);
    assert_volume(volume(&e, "inside"), want);
    assert_volume(volume(&e, "body"), 60.0 * 40.0 * 10.0 - l * w * d + want);
    field_agrees(&e, "body");
}

#[test]
fn a_triangular_prism_rounded_all_over_meets_slanted_corners_with_the_ball() {
    // Steiner's formula over the prism inset by `r`: its volume, its area carried out by `r`, its
    // edges' wedges of cylinder (the caps' square, the sides' turning through the triangle's
    // exterior angles, 2π in all), and the whole ball
    let (r, h) = (2.0, 10.0);
    let tri = [[10.0, 5.0], [50.0, 5.0], [5.0, 20.0]];
    let side = |a: [f64; 2], b: [f64; 2]| (b[0] - a[0]).hypot(b[1] - a[1]);
    let perimeter = side(tri[0], tri[1]) + side(tri[1], tri[2]) + side(tri[2], tri[0]);
    let area = 0.5 * ((tri[1][0] - tri[0][0]) * (tri[2][1] - tri[0][1]) - (tri[2][0] - tri[0][0]) * (tri[1][1] - tri[0][1])).abs();
    let inradius = 2.0 * area / perimeter;
    let s = (inradius - r) / inradius;
    let (a2, p2, h2) = (s * s * area, s * perimeter, h - 2.0 * r);
    let rounded = a2 * h2 + (2.0 * a2 + p2 * h2) * r + PI * r * r / 2.0 * p2 + PI * r * r * h2 + 4.0 * PI * r * r * r / 3.0;
    let e = read(&format!("unit mm\n{}prism := solid(tri_f, depth: {h}mm)\nall := fillet(prism, prism, r: {r}mm)\n\
        body := solid(prism)\nall cut body\n", polygon("tri_f", &tri)));
    assert_volume(volume(&e, "body"), rounded);
    field_agrees(&e, "body");
}

/// Where two runs of a fillet cross at a corner whose third edge is left sharp (a mitre), each
/// height `z` within `r` of the face they share has its strips `δ = r − √(2rz − z²)` wide along both
/// edges; over a convex polygon's prism a cap rounded alone loses the polygon less its inset by
/// `δ`, `Pδ − δ² Σ cot(θᵢ/2)` (θ its angles), so `∫δ = (1 − π/4) r²` along the perimeter and
/// `∫δ² = (5/3 − π/2) r³` at each corner, counted twice in the runs.
fn mitred(perimeter: f64, cots: f64, r: f64) -> f64 {
    (1.0 - PI / 4.0) * r * r * perimeter - (5.0 / 3.0 - PI / 2.0) * r * r * r * cots
}

#[test]
fn a_blocks_top_rim_rounded_alone_is_mitred_at_its_sharp_corners() {
    let r = 2.0;
    let e = read(&format!("{RECT}block := solid(sec, depth: 10mm)
rim := fillet(block.near, block, r: {r}mm)
        body := solid(block)
rim cut body
"));
    let i = e.map.ent_named("rim").unwrap().i();
    let blend = e.sketch.fillet_blend(i).unwrap();
    assert_eq!((blend.pieces.len(), blend.corners.len(), blend.joins.len()), (4, 0, 0));
    assert_volume(volume(&e, "rim"), mitred(200.0, 4.0, r));
    assert_volume(volume(&e, "body"), 60.0 * 40.0 * 10.0 - mitred(200.0, 4.0, r));
    field_agrees(&e, "body");
}

#[test]
fn a_pockets_floor_rounded_alone_is_mitred_at_its_corners() {
    let (r, [l, w], d) = (1.5, [30.0, 20.0], 6.0);
    let e = read(&format!(
        "{RECT}{}plate := solid(sec, depth: 10mm)
pocket := solid(cut_f, from: 1mm, to: -{d}mm)
         cupped := solid(plate)
pocket cut cupped
body := solid(cupped)
         floor := fillet(cupped.pocket.far, cupped.pocket, r: {r}mm)
floor union body
",
        polygon("cut_f", &[[15.0, 10.0], [45.0, 10.0], [45.0, 30.0], [15.0, 30.0]])
    ));
    let fill = mitred(2.0 * (l + w), 4.0, r);
    assert_volume(volume(&e, "body"), 60.0 * 40.0 * 10.0 - l * w * d + fill);
    field_agrees(&e, "body");
}

#[test]
fn a_triangular_prisms_cap_rounded_alone_runs_each_side_on_to_the_next() {
    // an obtuse corner, where each run carries on past the vertex to the plane beyond, and two
    // acute ones, where each run crosses the other's edge in the cap
    let (r, h) = (1.5, 10.0);
    let tri = [[10.0, 5.0], [50.0, 5.0], [5.0, 20.0]];
    let side = |a: [f64; 2], b: [f64; 2]| (b[0] - a[0]).hypot(b[1] - a[1]);
    let perimeter = side(tri[0], tri[1]) + side(tri[1], tri[2]) + side(tri[2], tri[0]);
    let cots: f64 = (0..3).map(|k| {
        let (p, a, b) = (tri[k], tri[(k + 1) % 3], tri[(k + 2) % 3]);
        let (u, v) = ([a[0] - p[0], a[1] - p[1]], [b[0] - p[0], b[1] - p[1]]);
        let theta = (u[0] * v[1] - u[1] * v[0]).abs().atan2(u[0] * v[0] + u[1] * v[1]);
        1.0 / (theta / 2.0).tan()
    }).sum();
    let e = read(&format!("unit mm
{}prism := solid(tri_f, depth: {h}mm)
cap := fillet(prism.near, prism, r: {r}mm)
        body := solid(prism)
cap cut body
", polygon("tri_f", &tri)));
    let area = 0.5 * ((tri[1][0] - tri[0][0]) * (tri[2][1] - tri[0][1]) - (tri[2][0] - tri[0][0]) * (tri[1][1] - tri[0][1])).abs();
    field_agrees(&e, "body");
    assert_volume(volume(&e, "body"), area * h - mitred(perimeter, cots, r));
}

#[test]
fn what_a_corner_cannot_round_is_refused() {
    // an L-shaped plate's cap rounded alone: at its reflex corner the runs would part, not cross
    let ell = polygon("ell_f", &[[10.0, 5.0], [50.0, 5.0], [50.0, 20.0], [30.0, 20.0], [30.0, 35.0], [10.0, 35.0]]);
    refused(&format!("unit mm
{ell}ell := solid(ell_f, depth: 10mm)
cap := fillet(ell.near, ell, r: 2mm)
        body := solid(ell)
cap cut body
"), Code::E085, "turns the other way");
    // a ball too large for a short edge between two corners: their setbacks meet
    refused(&format!("{RECT}block := solid(sec, depth: 3mm)\nall := fillet(block, block, r: 2mm)\n\
        body := solid(block)\nall cut body\n"), Code::E085, "can hold");
    // a half disc's prism: where its flat side meets its round, a cap and a cylinder at a corner
    let half = "unit mm\nh0 := point hint(x: 10, y: 20)\nh1 := point hint(x: 50, y: 20)\nho := point hint(x: 30, y: 20)\n\
        fix(x == 10, y == 20) h0\nfix(x == 50, y == 20) h1\nfix(x == 30, y == 20) ho\n\
        (flat := line(h0, h1)) -> (bow := arc(center: ho) hint(r: 20)) -> close\nhalf_f := face(flat, bow)\n\
        half := solid(half_f, depth: 10mm)\nall := fillet(half, half, r: 2mm)\nbody := solid(half)\nall cut body\n";
    refused(half, Code::E085, "rung 3");
    // its cap alone: a run and a ring at each end of the flat side
    refused(&half.replace("fillet(half, half", "fillet(half.near, half"), Code::E085, "rung 3");
}

#[test]
fn a_many_sided_prism_rounded_all_over_reads_as_a_field() {
    // sixteen sides: forty-eight runs and thirty-two corners, their material unioned in pairs so the
    // field stands few unions deep (folded one by one, it stood past the depth a field allows)
    let (n, big, h, r) = (16, 20.0, 10.0, 2.0);
    let tri: Vec<[f64; 2]> = (0..n).map(|k| {
        let a = std::f64::consts::TAU * k as f64 / n as f64;
        [30.0 + big * a.cos(), 25.0 + big * a.sin()]
    }).collect();
    // Steiner over the prism inset by `r`: its sides' exterior angles turn through 2π in all
    let inset = big * (PI / n as f64).cos() - r;
    let p2 = 2.0 * n as f64 * inset * (PI / n as f64).tan();
    let (a2, h2) = (p2 * inset / 2.0, h - 2.0 * r);
    let rounded = a2 * h2 + (2.0 * a2 + p2 * h2) * r + PI * r * r / 2.0 * p2 + PI * r * r * h2 + 4.0 * PI * r * r * r / 3.0;
    let e = read(&format!("unit mm\n{}prism := solid(gon_f, depth: {h}mm)\nall := fillet(prism, prism, r: {r}mm)\n\
        body := solid(prism)\nall cut body\n", polygon("gon_f", &tri)));
    assert_volume(volume(&e, "body"), rounded);
    field_agrees(&e, "body");
}
