//! **Fillets** (Solvent §6.9, issue #66, rung 1): a rolling ball's material along the edges where
//! two solids or faces meet, held against arithmetic — a straight round removes `(1 − π/4) r²` of
//! section along its edge, a ring adds or removes its section turned about the axis (Pappus) —
//! and every fillet this rung cannot round exactly refused with its reason.

use gcs_core::program::{elaborate, Code, Elaborated};
use gcs_core::syntax::parse;
use std::f64::consts::PI;

fn read(src: &str) -> Elaborated {
    let (prog, errs) = parse(src);
    assert!(errs.is_empty(), "does not parse: {errs:?}\n{src}");
    let e = elaborate(&prog);
    assert!(
        e.ok(),
        "does not elaborate: {:?}\n{src}",
        e.diags.iter().map(|d| format!("{}: {}", d.code.as_str(), d.message)).collect::<Vec<_>>()
    );
    e
}

fn refused(src: &str, code: Code, needle: &str) {
    let (prog, errs) = parse(src);
    let mut saw: Vec<String> = errs.iter().map(|e| format!("E100: {}", e.message)).collect();
    let mut hit = code == Code::E100 && errs.iter().any(|d| d.message.contains(needle));
    if !hit && errs.is_empty() {
        // what a fillet cannot round is known once the drawing is solved: the documents here
        // are seeded where they solve, so the diagnosis after the solve reads them as they are
        let e = elaborate(&prog);
        let mut diags = e.diags.clone();
        diags.extend(gcs_core::program::solid_diagnostics(&e.sketch, &e.map));
        saw.extend(diags.iter().map(|d| format!("{}: {}", d.code.as_str(), d.message)));
        hit = diags.iter().any(|d| d.code == code && d.message.contains(needle));
    }
    assert!(hit, "expected {} `{needle}`\n{src}\n{saw:#?}", code.as_str());
}

fn volume(e: &Elaborated, name: &str) -> f64 {
    let key = format!("{name}.volume");
    gcs_core::report::positions(&e.sketch, &e.map)
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
/// its top, 20 tall), both turned about one vertical axis on the page.
const TURNED: &str = "\
unit mm
a0 := point
a1 := point hint(x: 0, y: 40)
fix(x == 0, y == 0) a0
fix(x == 0, y == 40) a1
axis := line(a0, a1)
p0 := point
p1 := point hint(x: 40, y: 0)
p2 := point hint(x: 40, y: 10)
p3 := point hint(x: 0, y: 10)
fix(x == 0, y == 0) p0
fix(x == 40, y == 0) p1
fix(x == 40, y == 10) p2
fix(x == 0, y == 10) p3
(pbot := line(p0, p1)) -> (pside := line(p1, p2)) -> (ptop := line(p2, p3)) -> (paxis := line(p3, p0)) -> close
pf := face(pbot, pside, ptop, paxis)
q0 := point hint(x: 0, y: 10)
q1 := point hint(x: 15, y: 10)
q2 := point hint(x: 5, y: 30)
q3 := point hint(x: 0, y: 30)
fix(x == 0, y == 10) q0
fix(x == 15, y == 10) q1
fix(x == 5, y == 30) q2
fix(x == 0, y == 30) q3
(cbot := line(q0, q1)) -> (slant := line(q1, q2)) -> (ctop := line(q2, q3)) -> (caxis := line(q3, q0)) -> close
cf := face(cbot, slant, ctop, caxis)
plate := solid(pf, about: axis)
frustum := solid(cf, about: axis)
body := solid(plate)
frustum union body
";

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
    let e = read(&format!(
        "{TURNED}root := fillet(frustum, plate, r: 3mm)\nroot union body\n\
         rim := fillet(frustum.ctop, frustum.slant, r: 1mm)\nrim cut body\n"
    ));
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
fn a_ball_on_a_plate_is_rung_two() {
    // a sphere sunk into the plate meets it on a circle no plane, cylinder or cone carries
    let src = format!(
        "{TURNED}c0 := point hint(x: 0, y: 10)\ns0 := point hint(x: 0, y: 4)\ns1 := point hint(x: 0, y: 16)\n\
         fix(x == 0, y == 10) c0\nfix(x == 0, y == 4) s0\nfix(x == 0, y == 16) s1\n\
         rim_arc := arc(center: c0, start: s0, end: s1) hint(r: 6)\nshut := line(s1, s0)\n\
         ball := solid(face(rim_arc, shut), about: axis)\nball union body\n\
         neck := fillet(ball, plate, r: 1mm)\nneck union body\n"
    );
    refused(&src, Code::E085, "rung 2");
}

#[test]
fn a_fillet_is_written_as_it_was_read() {
    let src = "root := fillet(boss, plate.near, r: 3mm)\n";
    let (mut prog, errs) = parse(src);
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
    let e = read(&format!(
        "{TURNED}root := fillet(frustum, plate, r: 3mm)\nroot union body\n\
         rim := fillet(frustum.ctop, frustum.slant, r: 1mm)\nrim cut body\n"
    ));
    field_agrees(&e, "body");
    let e = read(&format!(
        "{RECT}block := solid(sec, depth: 30mm)\nlip := fillet(block.near, block.bc, r: 5mm)\nlip cut block\n"
    ));
    field_agrees(&e, "block");
    let e = read(&format!("{}root := fillet(rib, plate, r: 2mm)\nroot union body\n", rib(0.0)));
    field_agrees(&e, "body");
}
