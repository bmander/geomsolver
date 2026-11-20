//! **Where a body is made, and a face's measures** (#953, §6.25): a solid kept within a
//! half-space is the body in common with it, built exactly (`solid::defined::trimmed`, the
//! kernel's `half` node); its plane's height may be an unknown a measure drives
//! (`interior::Sheet::Ordinate`); and the drawing states, in `std.measures`'s words, where a
//! body's volume or mass is centred, `g centroid B`, `g centerofmass B`, a face's area, `area(A)
//! F`, and its second moment about a line, `moment(i: I, about: l) F` — each an equation of
//! integrals, a row of the drawing over the body's filled mesh moved by its sheets, or a claim
//! judged on the solved drawing.  Each is held to its closed form: a box barge, a cylinder
//! floating at a draft, a wedge heeled about its apex.  The rules are
//! `docs/contract/analysis.md`'s.
use gcs_core::constraints::CKind;
use gcs_core::model::Sketch;
use gcs_core::program::Elaborated;

/// A box barge `L` long and `B` in beam, 100 deep, on the top plane its keel's middle at the
/// origin, kept within the water below the waterline `T` above it (an unknown), and after it
/// `lines` of the drawing's own.
pub fn barge(lines: &str) -> String {
    format!("\
unit mm
use std
use std.solids
use std.sets
use std.measures (mass, volume, area, moment, centroid, centerofmass)
param T: Length hint(30mm)
L := 200mm
B := 60mm
in std.top {{
  a := point
  b := point
  c := point
  d := point
  fix((0mm, -B / 2)) a
  fix((L, -B / 2)) b
  fix((L, B / 2)) c
  fix((0mm, B / 2)) d
  side := line(a, b)
  stern := line(b, c)
  other := line(c, d)
  bow := line(d, a)
  plan := face(side, stern, other, bow)
}}
hull := solid(std.solids.Prism(plan, from: 0mm, to: 100mm))
keel := point
fix((0mm, 0mm, 0mm)) keel
water := std.sets.HalfSpace(keel, std.z, d: T)
immersed := solid({{ p | p in hull; p in water }})
fore := point
aft := point
fix((0mm, 0mm, 0mm)) fore
fix((L, 0mm, 0mm)) aft
centreline := line(fore, aft)
{lines}
")
}

/// `barge`, its centreline drawn in the waterplane: its points `T` above the keel, so a second
/// moment about it is the waterplane's own.
pub fn floating(lines: &str) -> String {
    barge(lines).replace("fore := point\naft := point\nfix((0mm, 0mm, 0mm)) fore\n\
        fix((L, 0mm, 0mm)) aft", "fore := point hint((0, 0, 30))\naft := point hint((200, 0, \
        30))\nfix(x == 0mm, y == 0mm) fore\nfix(x == L, y == 0mm) aft\nkeel distance(T, along: \
        std.z) fore\nkeel distance(T, along: std.z) aft")
}

/// `src` elaborated with no error and solved.
fn sized(src: &str) -> (Elaborated, Sketch) {
    crate::common::read_solved(src)
}

/// What the unknown `name` came to.
fn unknown(sk: &Sketch, name: &str) -> f64 {
    sk.params[sk.free_vars[name] as usize].value
}

/// What the report says of each measure the drawing states, each its sentence.
fn said(e: &Elaborated, sk: &Sketch) -> Vec<String> {
    gcs_core::report::measure_answers(sk, &e.measures).into_iter().map(|a| a.text).collect()
}

/// What the report says of each measure of `src`, solved, and the sketch.
fn read(src: &str) -> (Vec<String>, Sketch) {
    let (e, sk) = sized(src);
    (said(&e, &sk), sk)
}

/// Where the point the document calls `n` stands on the solved drawing.
fn at(e: &Elaborated, sk: &Sketch, n: &str) -> [f64; 3] {
    sk.world_point(crate::common::ent(e, n).i())
}

fn close(got: f64, want: f64, tol: f64, what: &str) {
    assert!((got - want).abs() <= tol * want.abs().max(1.0), "{what}: {got} against {want}");
}

/// The errors `src` elaborates with, each its code and its message.
fn errors(src: &str) -> Vec<(String, String)> {
    let (p, errs, _) = gcs_core::library::parse_linked(src);
    assert!(errs.is_empty(), "{errs:?}");
    gcs_core::program::elaborate(&p).errors()
        .map(|d| (d.code.as_str().to_string(), d.message.clone())).collect()
}

/// **The hull in water is the hull kept within a half-space, built exactly** (#953, step 1): the
/// region `{ p | p in hull; p in water }` is the body over `hull` kept in common with a region of
/// the same name, so its faces are the hull's (`immersed.hull.low`) and the waterplane's
/// (`immersed.water`), and what it is made of is read on it — `L · B · T` and the waterplane's
/// `L · B`.
#[test]
fn the_hull_in_water_is_built_exactly() {
    let src = barge("claim volume(L * B * 30mm) immersed\nclaim area(L * B) immersed.water\n")
        .replace("param T: Length hint(30mm)", "T := 30mm");
    let (e, sk) = sized(&src);
    let immersed = crate::common::ent(&e, "immersed").i();
    assert!(matches!(sk.solids[immersed].def, gcs_core::model::SolidDef::Body { .. }));
    let faces: Vec<String> = gcs_core::solid::faces::face_paths(&sk, immersed).iter()
        .map(|f| f.text()).collect();
    assert!(faces.contains(&"water".to_string()) && faces.contains(&"hull.side".to_string()),
            "{faces:?}");
    let said = said(&e, &sk);
    assert!(said[0].ends_with("== 360000 mm^3: claimed, kept"), "{}", said[0]);
    assert!(said[1].ends_with("== 12000 mm^2: claimed, kept"), "{}", said[1]);
}

/// **A box barge, its waterline found** (#953, the issue's acceptance): `volume(L · B · T₀)
/// immersed` drives the half-space's height `T` to `T₀`, leaving no freedom; the immersed
/// centroid, driven onto a point, stands at `(L/2, 0, T/2)`; the waterplane's area is `LB`, its
/// centroid on the centreline, its second moment about the centreline `LB³/12` — each claimed
/// against its closed form, and each driven where it sizes an unknown: the waterplane's breadth
/// `w` by its area, `k = I / (L B)` by its moment.
#[test]
fn a_box_barge_floats_at_its_draft() {
    let lines = "\
volume(L * B * 40mm) immersed
g := point hint((100, 0, 20))
g centroid immersed
f := point hint((100, 0, 40))
claim f centroid immersed.water
claim area(L * B) immersed.water
claim moment(i: L * B^3 / 12, about: centreline) immersed.water
param w: Length hint(50mm)
area(L * w) immersed.water
param k: Length hint(10mm)
moment(i: k * L * B * 1mm, about: centreline) immersed.water
";
    let src = floating(lines);
    let (e, mut sk) = sized(&src);
    close(unknown(&sk, "T"), 40.0, 1e-9, "T");
    let g = at(&e, &sk, "g");
    for (got, want) in g.iter().zip([100.0, 0.0, 20.0]) {
        close(*got, want, 1e-9, "the immersed centroid");
    }
    close(unknown(&sk, "w"), 60.0, 1e-9, "the waterplane's breadth");
    close(unknown(&sk, "k"), 200.0 * 60f64.powi(3) / 12.0 / (200.0 * 60.0), 1e-9, "I / (L B)");
    assert_eq!(crate::common::dof(&mut sk), 3, "the claimed point `f` alone is free");
    // a volume, a centroid's three, an area and a second moment: six rows of one kind
    let rows = sk.constraints.iter().filter(|c| c.kind == CKind::Integral).count();
    assert_eq!(rows, 6);
    let said = said(&e, &sk);
    assert!(said[0].ends_with("== 480000 mm^3: driven, held"), "{}", said[0]);
    for s in &said[1..4] {
        assert!(s.starts_with("g centroid immersed: ") && s.ends_with("== 0 mm: driven, held"),
                "{s}");
    }
    for s in &said[4..7] {
        assert!(s.starts_with("f centroid immersed.water: ") && s.ends_with("claimed, kept"),
                "{s}");
    }
    assert!(said[7].ends_with("== 12000 mm^2: claimed, kept"), "{}", said[7]);
    assert!(said[8].ends_with("== 3.6e+06 mm^4: claimed, kept"), "{}", said[8]);
}

/// The cylinder's section: a rectangle `L` along x and `R` up from the axis in the front plane,
/// turned once about the x axis; its keel `R` below the axis, kept within the water up to `T`
/// above the keel, and after it `lines`.
fn cylinder(lines: &str) -> String {
    format!("\
unit mm
use std
use std.solids
use std.sets
use std.measures (mass, volume, area, moment, centroid, centerofmass)
param T: Length hint(20mm)
L := 200mm
R := 50mm
in std.front {{
  o := point
  a := point
  b := point
  c := point
  fix((0mm, 0mm)) o
  fix((L, 0mm)) a
  fix((L, R)) b
  fix((0mm, R)) c
  axis_line := line(o, a)
  end := line(a, b)
  top := line(b, c)
  front := line(c, o)
  section := face(axis_line, end, top, front)
}}
hull := solid(std.solids.Revolution(section, about: std.x))
keel := point
fix((0mm, 0mm, -R)) keel
water := std.sets.HalfSpace(keel, std.z, d: T)
immersed := solid({{ p | p in hull; p in water }})
{lines}
")
}

/// **A cylinder floating at a stated draft** (#953): curved faces, so the circular segment's
/// closed forms — at the half-angle α where the waterline cuts it, the draft `R (1 − cos α)`, the
/// volume `L R² (α − sin α cos α)`, the centroid `4 R sin³α / (3 (2α − sin 2α))` below the axis,
/// the waterplane `2 R sin α` wide.  The volume drives the draft and a point onto the centroid,
/// each within the quadratic mesh's error of its curved faces; the waterplane's area and second
/// moment about its centreline are claimed.
#[test]
fn a_cylinder_floats_at_its_draft() {
    let (r, l) = (50.0f64, 200.0f64);
    let alpha = std::f64::consts::FRAC_PI_3;
    let v = l * r * r * (alpha - alpha.sin() * alpha.cos());
    let draft = r * (1.0 - alpha.cos());
    let below = 4.0 * r * alpha.sin().powi(3) / (3.0 * (2.0 * alpha - (2.0 * alpha).sin()));
    let width = 2.0 * r * alpha.sin();
    let lines = format!("\
volume({v} * 1mm^3) immersed
g := point hint((100, 0, -30))
g centroid immersed
");
    let (e, sk) = sized(&cylinder(&lines));
    close(unknown(&sk, "T"), draft, 1e-4, "the draft");
    let g = at(&e, &sk, "g");
    close(g[0], l / 2.0, 1e-6, "the centroid along the axis");
    assert!(g[1].abs() <= 1e-6 * r, "the centroid's off the centreline: {}", g[1]);
    close(g[2], -below, 1e-4, "the centroid below the axis");
    // the waterplane claimed at the draft stated, its centreline in it
    let stated = cylinder(&format!("\
fore := point
aft := point
fix((0mm, 0mm, {z})) fore
fix((L, 0mm, {z})) aft
centreline := line(fore, aft)
claim area(<= {a_hi} * 1mm^2) immersed.water
claim area(>= {a_lo} * 1mm^2) immersed.water
claim moment(i: <= {i_hi} * 1mm^4, about: centreline) immersed.water
claim moment(i: >= {i_lo} * 1mm^4, about: centreline) immersed.water
", z = draft - r, a_hi = l * width * (1.0 + 1e-4), a_lo = l * width * (1.0 - 1e-4),
       i_hi = l * width.powi(3) / 12.0 * (1.0 + 1e-4), i_lo = l * width.powi(3) / 12.0 * (1.0 - 1e-4)))
        .replace("param T: Length hint(20mm)", &format!("T := {draft}mm"));
    let (said, _) = read(&stated);
    assert!(said.iter().all(|s| s.ends_with("kept")), "{said:#?}");
}

/// A V-wedge `L` long, its sides `β` either side of the vertical from its apex on the y axis, up
/// to `H`; the water heeled `φ` about the apex, its height `T` along the heeled vertical; and after
/// it `lines`.
fn wedge(beta: f64, phi: f64, lines: &str) -> String {
    let h = 100.0;
    let half = h * beta.tan();
    let (s, c) = phi.sin_cos();
    format!("\
unit mm
use std
use std.solids
use std.sets
use std.measures (mass, volume, area, moment, centroid, centerofmass)
param T: Length hint(35mm)
L := 100mm
in std.front {{
  apex := point
  a := point
  b := point
  fix((0mm, 0mm)) apex
  fix(({half}mm, {h}mm)) a
  fix(({neg}mm, {h}mm)) b
  side_a := line(apex, a)
  top := line(a, b)
  side_b := line(b, apex)
  section := face(side_a, top, side_b)
}}
hull := solid(std.solids.Prism(section, from: 0mm, to: L))
keel := point
fix((0mm, 0mm, 0mm)) keel
heel := axis
fix(dir == ({s}, 0, {c}), origin == (0, 0, 0)) heel
water := std.sets.HalfSpace(keel, heel, d: T)
immersed := solid({{ p | p in hull; p in water }})
{lines}
", neg = -half)
}

/// **A wedge heeled about its apex** (#953): an inclined waterline.  In the section the sides run
/// along `u₁ = (sin β, cos β)` and `u₂ = (−sin β, cos β)` from the apex, the waterline is where
/// `n · x = T`, `n = (sin φ, cos φ)` the heeled vertical, so it meets them at `tᵢ = T / (n · uᵢ)`:
/// the immersed triangle's area `½ t₁ t₂ sin 2β`, its centroid the mean of its corners, the
/// waterline `|t₁u₁ − t₂u₂|` long.  The volume drives `T` and a point onto the centroid; the
/// waterplane's area and second moment about its own centreline are claimed.
#[test]
fn a_wedge_heeled_about_its_apex() {
    let (beta, phi, l, t) = (30f64.to_radians(), 10f64.to_radians(), 100.0f64, 40.0f64);
    let n = [phi.sin(), phi.cos()];
    let (u1, u2) = ([beta.sin(), beta.cos()], [-beta.sin(), beta.cos()]);
    let dot = |a: [f64; 2], b: [f64; 2]| a[0] * b[0] + a[1] * b[1];
    let (t1, t2) = (t / dot(n, u1), t / dot(n, u2));
    let area = 0.5 * t1 * t2 * (2.0 * beta).sin();
    let (p1, p2) = ([t1 * u1[0], t1 * u1[1]], [t2 * u2[0], t2 * u2[1]]);
    let centroid = [(p1[0] + p2[0]) / 3.0, (p1[1] + p2[1]) / 3.0];
    let width = ((p1[0] - p2[0]).powi(2) + (p1[1] - p2[1]).powi(2)).sqrt();
    let lines = format!("volume({} * 1mm^3) immersed\ng := point hint((0, 50, 20))\n\
                         g centroid immersed\n", l * area);
    let (e, sk) = sized(&wedge(beta, phi, &lines));
    close(unknown(&sk, "T"), t, 1e-9, "the height along the heeled vertical");
    let g = at(&e, &sk, "g");
    close(g[0], centroid[0], 1e-9, "the centroid across");
    close(g[2], centroid[1], 1e-9, "the centroid up");
    close(g[1].abs(), l / 2.0, 1e-9, "the centroid along");
    let mid = [(p1[0] + p2[0]) / 2.0, (p1[1] + p2[1]) / 2.0];
    let i = l * width.powi(3) / 12.0;
    let stated = wedge(beta, phi, &format!("\
fore := point
aft := point
fix(({x}mm, 0mm, {z}mm)) fore
fix(({x}mm, -L, {z}mm)) aft
centreline := line(fore, aft)
claim area({a} * 1mm^2) immersed.water
claim moment(i: {i} * 1mm^4, about: centreline) immersed.water
", x = mid[0], z = mid[1], a = l * width))
        .replace("param T: Length hint(35mm)", &format!("T := {t}mm"));
    let (said, _) = read(&stated);
    assert!(said.iter().all(|s| s.ends_with("kept")), "{said:#?}");
}

/// A steel block 40 × 20 × 10 at the origin and an aluminium one 60 long beside it, as two
/// components of one instance (`pair := Pair(…)`) or as one component of both bodies (`both :=
/// Both(…)`), and `of` naming which.
fn blocks(of: &str) -> String {
    let block = |name: &str, x0: f64, x1: f64| format!("\
component {name}(m: group) {{
  in std.top {{
    a := point
    b := point
    c := point
    d := point
    fix(({x0}mm, 0mm)) a
    fix(({x1}mm, 0mm)) b
    fix(({x1}mm, 20mm)) c
    fix(({x0}mm, 20mm)) d
    s1 := line(a, b)
    s2 := line(b, c)
    s3 := line(c, d)
    s4 := line(d, a)
    sec := face(s1, s2, s3, s4)
  }}
  body := solid(std.solids.Prism(sec, from: 0mm, to: 10mm)) of m
}}
");
    format!("\
unit mm
use std
use std.solids
use std.measures (mass, volume, area, moment, centroid, centerofmass)
steel := material(E: 200GPa, nu: 0.3, rho: 7850kg / 1m^3)
alu := material(E: 70GPa, nu: 0.33, rho: 2700kg / 1m^3)
{}{}
component Pair(ms: group, ma: group) {{
  s := Steel(m: ms)
  a := Alu(m: ma)
}}
component Both(ms: group, ma: group) {{
  in std.top {{
    a := point
    b := point
    c := point
    d := point
    e := point
    f := point
    fix((0mm, 0mm)) a
    fix((40mm, 0mm)) b
    fix((40mm, 20mm)) c
    fix((0mm, 20mm)) d
    fix((100mm, 0mm)) e
    fix((100mm, 20mm)) f
    s1 := line(a, b)
    s2 := line(b, c)
    s3 := line(c, d)
    s4 := line(d, a)
    t1 := line(b, e)
    t2 := line(e, f)
    t3 := line(f, c)
    t4 := line(c, b)
    left := face(s1, s2, s3, s4)
    right := face(t1, t2, t3, t4)
  }}
  steel_body := solid(std.solids.Prism(left, from: 0mm, to: 10mm)) of ms
  alu_body := solid(std.solids.Prism(right, from: 0mm, to: 10mm)) of ma
}}
pair := Pair(ms: steel, ma: alu)
both := Both(ms: steel, ma: alu)
g := point hint((50, 10, 5))
g centerofmass {of}
c := point hint((50, 10, 5))
c centroid {of}
", block("Steel", 0.0, 40.0), block("Alu", 40.0, 100.0))
}

/// **A centre of mass does not depend on how bodies are grouped** (#953, the design bible's
/// totals): the same two blocks, steel and aluminium, as two components of one instance and as
/// one component of both bodies, put a point at `Σ ρᵢ Vᵢ xᵢ / Σ ρᵢ Vᵢ` alike — and their centroid,
/// by volume alone, at their middle.
#[test]
fn a_centre_of_mass_does_not_depend_on_grouping() {
    let (ms, ma) = (7850.0 * 8000.0, 2700.0 * 12000.0);
    let x = (ms * 20.0 + ma * 70.0) / (ms + ma);
    let mut found = Vec::new();
    for of in ["pair", "both"] {
        let (e, sk) = sized(&blocks(of));
        let (g, c) = (at(&e, &sk, "g"), at(&e, &sk, "c"));
        close(g[0], x, 1e-9, &format!("{of}: the centre of mass"));
        close(c[0], 50.0, 1e-9, &format!("{of}: the centroid"));
        for p in [g, c] {
            close(p[1], 10.0, 1e-9, "across");
            close(p[2].abs(), 5.0, 1e-9, "up");
        }
        found.push(g);
    }
    for k in 0..3 {
        close(found[0][k], found[1][k], 1e-12, "grouped and not");
    }
}

/// **A body floats where it weighs the water it displaces** (§6.25): one equation of two
/// integrals, the hull's mass against the water's density times the immersed volume —
/// `integral(density(p) over p in hull) == ρ_w * integral(1 over p in immersed)` — finds the
/// waterline with no intermediate unknown: a box 200 by 60 by 100 of 300 kg/m³ in water of 1000
/// floats at `0.3 · 100 = 30mm`, to 1e-9, leaving no freedom.
#[test]
fn a_body_floats_where_it_weighs_its_displacement() {
    let src = barge("integral(density(p) over p in hull) == 1000kg / 1m^3 * integral(1 over p in \
                     immersed)")
        .replace("hull := solid(std.solids.Prism(plan, from: 0mm, to: 100mm))",
                 "wood := material(E: 10GPa, nu: 0.3, rho: 300kg / 1m^3)\n\
                  hull := solid(std.solids.Prism(plan, from: 0mm, to: 100mm)) of wood")
        .replace("param T: Length hint(30mm)", "param T: Length hint(20mm)");
    assert!(src.contains("of wood"), "{src}");
    let (e, mut sk) = sized(&src);
    close(unknown(&sk, "T"), 30.0, 1e-9, "the waterline");
    assert_eq!(crate::common::dof(&mut sk), 0);
    let said = said(&e, &sk);
    assert!(said[0].ends_with("driven, held"), "{said:?}");
}

/// **A body not polynomial in its point is read by quadrature** (§6.25): `∫ r dV` of a cylinder
/// about its own axis, `r` the distance from it, is `2π L R³ / 3`; read over the cylinder's
/// filled mesh to its quadrature's order, within 1e-3, and claimed, kept within the error
/// estimated beside it.
#[test]
fn an_integral_not_polynomial_is_read_by_quadrature() {
    let (l, r) = (200.0f64, 50.0f64);
    let want = 2.0 * std::f64::consts::PI * l * r.powi(3) / 3.0;
    let src = cylinder(&format!("claim integral(distance(p, std.x) over p in hull) == {want} * \
                                 1mm^4\n"))
        .replace("param T: Length hint(20mm)", "T := 20mm");
    let (e, sk) = sized(&src);
    let m = &sk.measured[e.measures[0].index];
    assert!(!m.integrals[0].quadratic(), "a distance is no polynomial in the point");
    let a = gcs_core::measured::read(&sk, m).unwrap();
    close(a.lhs, want, 1e-3, "∫ r dV");
    let said = said(&e, &sk);
    assert!(said[0].ends_with("claimed, kept"), "{said:?}");
}

/// **What cannot be measured says why** (#953, §6.25), each where it is written: an area of a
/// solid (its integral a volume, against an area), a second moment with no line, a centre of mass
/// of a face (it weighs nothing), a claim reading an unknown nothing determines (a claim binds
/// none), a centre of mass of a body of no density, a word written without `std.measures`.
#[test]
fn what_cannot_be_measured_says_why() {
    let cases = [
        ("area(L * B) immersed", "its sides are of two dimensions"),
        ("moment(i: 1 * 1mm^4) immersed.water", "`moment` needs `about`"),
        ("g := point\ng centerofmass immersed.water", "a face weighs nothing"),
        ("param w: Length hint(5mm)\nclaim area(L * w) immersed.water", "a claim binds none"),
        ("g := point\ng centerofmass immersed", "made of no material"),
    ];
    for (line, want) in cases {
        let errs = errors(&barge(line));
        assert!(errs.iter().any(|(_, m)| m.contains(want)), "{line}: {errs:?}");
    }
    let errs = errors(&barge("volume(L * B * 40mm) immersed").replace(WORDS, ""));
    assert!(errs.iter().any(|(c, m)| c == "E102"
                            && m.contains("`std.measures` defines `volume`")), "{errs:?}");
}

/// The words of `std.measures` the fixtures import.
const WORDS: &str = "use std.measures (mass, volume, area, moment, centroid, centerofmass)\n";

/// A punt 1000 long and 300 deep: a flat bottom between two spline chines in the top plane, a
/// flat deck between the sheers in the plane 300 up, each sheer its chine moved 40 out and 300 up,
/// so each side is the developable panel between them — its rulings all along `(0, ±40, 300)`, a
/// cylinder over its chine — and flat ends; its shell `{ q | q in bottom or q in port or … }`, the
/// hull what it encloses, `solid({ p | p in shell.side })`; and after it `lines`.  `free` leaves a
/// chine's middle point to the solve.
pub fn punt(lines: &str, free: bool) -> String {
    let (xs, ys) = ([0.0, 250.0, 500.0, 750.0, 1000.0], [150.0, 260.0, 300.0, 260.0, 150.0]);
    let points = |prefix: &str, sign: f64, dy: f64| -> String {
        (0..5).map(|i| {
            let n = format!("{prefix}{i}");
            let fixed = !(free && prefix == "pc" && i == 2);
            let (x, y) = (xs[i], sign * (ys[i] + dy));
            if fixed { format!("  {n} := point\n  fix(({x}mm, {y}mm)) {n}\n") }
            else { format!("  {n} := point hint(({x}, {y}))\n") }
        }).collect()
    };
    let end = |name: &str, x: f64| format!("\
{name}_plane := plane
fix(origin == ({x}mm, 0mm, 0mm)) {name}_plane
fix(dir == (0, 1, 0)) {name}_plane.u
fix(dir == (0, 0, 1)) {name}_plane.v
in {name}_plane {{
  {name}1 := point
  {name}2 := point
  {name}3 := point
  {name}4 := point
  fix((-150mm, 0mm)) {name}1
  fix((150mm, 0mm)) {name}2
  fix((190mm, 300mm)) {name}3
  fix((-190mm, 300mm)) {name}4
  {name}_a := line({name}1, {name}2)
  {name}_b := line({name}2, {name}3)
  {name}_c := line({name}3, {name}4)
  {name}_d := line({name}4, {name}1)
  {name} := face({name}_a, {name}_b, {name}_c, {name}_d)
}}
");
    format!("\
unit mm
use std
use std.panels
use std.sets
use std.measures (mass, volume, area, moment, centroid, centerofmass)
deck_plane := plane
fix(origin == (0mm, 0mm, 300mm)) deck_plane
fix(dir == (1, 0, 0)) deck_plane.u
fix(dir == (0, 1, 0)) deck_plane.v
in std.top {{
{}{}  port_chine := spline(pc0, pc1, pc2, pc3, pc4)
  starboard_chine := spline(sc0, sc1, sc2, sc3, sc4)
  bow_foot := line(sc0, pc0)
  stern_foot := line(pc4, sc4)
  bottom := face(port_chine, stern_foot, starboard_chine, bow_foot)
}}
in deck_plane {{
{}{}  port_sheer := spline(ps0, ps1, ps2, ps3, ps4)
  starboard_sheer := spline(ss0, ss1, ss2, ss3, ss4)
  bow_head := line(ss0, ps0)
  stern_head := line(ps4, ss4)
  deck := face(port_sheer, stern_head, starboard_sheer, bow_head)
}}
{}{}port := std.panels.Developable(port_chine, port_sheer)
starboard := std.panels.Developable(starboard_sheer, starboard_chine)
shell := {{ q | q in bottom or q in port or q in starboard or q in deck or q in bow or q in stern }}
hull := solid({{ p | p in shell.side }})
keel := point
fix((0mm, 0mm, 0mm)) keel
{lines}
", points("pc", -1.0, 0.0), points("sc", 1.0, 0.0), points("ps", -1.0, 40.0),
       points("ss", 1.0, 40.0), end("bow", 0.0), end("stern", 1000.0))
}

/// **A hull of panels encloses a solid, and floats** (#953): the punt's shell — two developable
/// sides and four faces of the drawing — sewn into the solid it encloses, each face named by the
/// set it is in (`hull.port`, `hull.deck`), its volume `300 A + 300 · 40 · 1000`, `A` the bottom's
/// area; kept within the water, `volume(V) immersed` finds the waterline at the root of
/// `(40 / 300) · 1000 · T² + A T = V` — each side's section a trapezoid widening `40 / 300` a unit
/// up — and the waterplane's area is `A + 1000 · 80 T / 300`, to the tolerance its sides are
/// drawn to: through their rulings, each chord inside its rail (2e-4 of it here).
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, a minute: a hull of panels filled and \
                                             driven, then read again")]
fn a_hull_of_panels_floats() {
    let lines = "param T: Length hint(100mm)
water := std.sets.HalfSpace(keel, std.z, d: T)
immersed := solid({ p | p in hull; p in water })
volume(30000 * 1cm^3) immersed
";
    let (e, sk) = sized(&punt(lines, false));
    let hull = crate::common::ent(&e, "hull").i();
    let faces: Vec<String> = gcs_core::solid::faces::face_paths(&sk, hull).iter()
        .map(|f| f.text()).collect();
    assert_eq!(faces, ["bottom", "port", "starboard", "deck", "bow", "stern"]);
    let x = sk.exact_preimage(hull).expect("the hull is built exactly");
    let b = &x.brep;
    let bottom = b.faces.iter().filter(|f| f.name.ends_with("bottom"))
        .map(|f| gcs_core::brep::props::area(b, f)).sum::<f64>() / (x.mm * x.mm);
    let volume = gcs_core::brep::props::volume(b) / x.mm.powi(3);
    close(volume, 300.0 * bottom + 300.0 * 40.0 * 1000.0, 1e-6, "the hull's volume");
    let k = 40.0 / 300.0 * 1000.0;
    let t = (-bottom + (bottom * bottom + 4.0 * k * 3e7).sqrt()) / (2.0 * k);
    close(unknown(&sk, "T"), t, 1e-5, "the waterline");
    let (said, _) = read(&punt(&format!("T := {t}mm
water := std.sets.HalfSpace(keel, std.z, d: T)
immersed := solid({{ p | p in hull; p in water }})
claim area(<= {hi} * 1mm^2) immersed.water
claim area(>= {lo} * 1mm^2) immersed.water
", hi = (bottom + 1000.0 * 80.0 * t / 300.0) * (1.0 + 2e-4),
   lo = (bottom + 1000.0 * 80.0 * t / 300.0) * (1.0 - 2e-4)), false));
    assert!(said.iter().all(|s| s.ends_with("kept")), "{said:#?}");
}

/// **A panel the solve moves is said** (#953): a row over a hull whose chine a point the solve
/// moves shapes is refused where it is stated, naming the panel — a driving row moves a held
/// panel's nodes over its rulings' triangles, and does not move a panel by its rails yet.
#[test]
fn a_panel_the_solve_moves_is_said() {
    let lines = "pc2 coincident(t == 0.5) bottom_line
volume(30000 * 1cm^3) hull
";
    let errs = errors(&punt(lines, true).replace("  port_chine := spline", "  bottom_line := \
        line(pc1, pc3)\n  port_chine := spline"));
    assert!(errs.iter().any(|(c, m)| c == "E040" && m.contains("its panel `port`'s rails are \
        moved by the solve")), "{errs:?}");
}

/// **A claim over a body built exactly is read off its boundary** (#953): the log of
/// `cylinder.sv` stood at its draft, 25mm, its displacement, its centre of buoyancy's height over
/// its keel and its waterplane's second moment about the centreline each claimed within a
/// ten-billionth of its closed form — `L R² (α − sin α cos α)`, `R − 4 R sin³α / (3 (2α − sin
/// 2α))`, `L (2 R sin α)³ / 12` at α = 60° — and kept, each read through the ten moments of the
/// exact B-rep (`brep::props::moments`, `face_moments`), no error beside it, nothing meshed.
#[test]
fn a_claim_over_an_exact_body_is_read_off_its_boundary() {
    let (r, l, alpha) = (50.0f64, 200.0, std::f64::consts::FRAC_PI_3);
    let volume = l * r * r * (alpha - alpha.sin() * alpha.cos());
    let kb = r - 4.0 * r * alpha.sin().powi(3) / (3.0 * (2.0 * alpha - (2.0 * alpha).sin()));
    let i = l * (2.0 * r * alpha.sin()).powi(3) / 12.0;
    let within = |name: &str, v: f64, unit: &str, what: &str| format!(
        "claim {what} >= {lo} * {unit}\nclaim {what} <= {hi} * {unit}\n",
        lo = v * (1.0 - 1e-10), hi = v * (1.0 + 1e-10), what = what.replace("NAME", name));
    let src = example("cylinder.sv")
        .replace("param T: Length hint(20mm)", "T := 25mm")
        .replace("volume(L * R^2 * (pi / 3 - sqrt(3) / 4)) immersed\n", "")
        .replace("buoyancy := point hint((100, 0, -30))\nbuoyancy centroid immersed\n", "")
        + "fore := point\nfix((0mm, 0mm, -25mm)) fore\naft := point\nfix((200mm, 0mm, -25mm)) aft\n\
           centreline := line(fore, aft)\n"
        + &within("", volume, "1mm^3", "integral(1 over p in immersed)")
        + &within("", kb, "1mm", "integral(distance(keel, p, along: std.z) over p in immersed) \
                                   / integral(1 over p in immersed)")
        + &within("", i, "1mm^4", "integral(distance(p, centreline)^2 over p in immersed.water)");
    let (said, _) = read(&src);
    assert_eq!(said.len(), 6, "{said:#?}");
    assert!(said.iter().all(|s| s.ends_with("kept") && s.contains(" ± 0 ")), "{said:#?}");
}

/// **Two bodies kept within the water are two** (#953): the barge kept within a half-space 30
/// deep and within another 50 deep — the first's half-space's region made past it
/// (`defined::trimmed`), every solid declared after it as far further on — each claim reads its
/// own body and its own waterplane: `L B T` and `L B` at each draft, not the first's.
#[test]
fn two_bodies_kept_within_the_water_are_two() {
    let src = barge("T := 30mm\n").replace("param T: Length hint(30mm)\n", "")
        + "deeper := std.sets.HalfSpace(keel, std.z, d: 50mm)\n\
           wetter := solid({ p | p in hull; p in deeper })\n\
           claim volume(360000 * 1mm^3) immersed\n\
           claim area(12000 * 1mm^2) immersed.water\n\
           claim volume(600000 * 1mm^3) wetter\n\
           claim area(12000 * 1mm^2) wetter.deeper\n";
    let (said, _) = read(&src);
    assert_eq!(said.len(), 4, "{said:#?}");
    assert!(said.iter().all(|s| s.ends_with("kept")), "{said:#?}");
}

/// One of the hydrostatics examples (`rust/examples/hydrostatics/`), as its text stands.
fn example(name: &str) -> String {
    let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../examples/hydrostatics").join(name);
    std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{}: {e}", path.display()))
}

/// **The hydrostatics examples hold what they say** (#953): the log floats at 25mm, its centre of
/// buoyancy 35.25mm below its axis; the heeled wedge's waterline is 40mm along the heeled
/// vertical, its centre of buoyancy the immersed triangle's; the pine box floats at 45mm where it
/// weighs what it displaces, one equation of two integrals; the two blocks' centre of mass is
/// 37.02mm along whether grouped as two components or as one — each with no freedom left, its
/// rows held.
#[test]
fn the_hydrostatics_examples_hold_what_they_say() {
    let (r, alpha) = (50.0f64, std::f64::consts::FRAC_PI_3);
    let (e, mut sk) = sized(&example("cylinder.sv"));
    close(unknown(&sk, "T"), r * (1.0 - alpha.cos()), 1e-4, "the log's draft");
    let below = 4.0 * r * alpha.sin().powi(3) / (3.0 * (2.0 * alpha - (2.0 * alpha).sin()));
    close(at(&e, &sk, "buoyancy")[2], -below, 1e-4, "the log's centre of buoyancy");
    assert_eq!(crate::common::dof(&mut sk), 0);
    let (e, mut sk) = sized(&example("heeled_wedge.sv"));
    close(unknown(&sk, "T"), 40.0, 1e-5, "the wedge's waterline");
    let b = at(&e, &sk, "buoyancy");
    let (beta, phi) = (30f64.to_radians(), 10f64.to_radians());
    let (t1, t2) = (40.0 / (beta - phi).cos(), 40.0 / (beta + phi).cos());
    close(b[0], (t1 * beta.sin() - t2 * beta.sin()) / 3.0, 1e-4, "across");
    close(b[2], (t1 + t2) * beta.cos() / 3.0, 1e-4, "up");
    assert_eq!(crate::common::dof(&mut sk), 0);
    let (e, mut sk) = sized(&example("equilibrium.sv"));
    close(unknown(&sk, "T"), 45.0, 1e-9, "the pine box's waterline");
    let b = at(&e, &sk, "buoyancy");
    close(b[0], 100.0, 1e-9, "its centre of buoyancy along");
    close(b[2], 22.5, 1e-9, "its centre of buoyancy up");
    assert_eq!(crate::common::dof(&mut sk), 0);
    assert!(said(&e, &sk)[0].starts_with("integral(density(p) over p in box) == rho_w * "),
            "{:?}", said(&e, &sk));
    let (e, mut sk) = sized(&example("centre_of_mass.sv"));
    let (ms, ma) = (7850.0 * 8000.0, 2700.0 * 12000.0);
    let x = (ms * 20.0 + ma * 70.0) / (ms + ma);
    close(at(&e, &sk, "g")[0], x, 1e-9, "the pair's centre of mass");
    close(at(&e, &sk, "h")[0], x, 1e-9, "both bodies' centre of mass");
    close(at(&e, &sk, "c")[0], 50.0, 1e-9, "the pair's centroid");
    assert_eq!(crate::common::dof(&mut sk), 0);
    assert!(said(&e, &sk).iter().all(|s| s.ends_with("held")), "{:#?}", said(&e, &sk));
}

/// **The punt's example holds what it says** (#953): its hull of panels built, its displacement at
/// the stated waterline claimed and kept.
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, half a minute: a hull of panels filled \
                                             twice for its claim")]
fn the_punt_example_floats() {
    let (said, _) = read(&example("punt.sv"));
    assert_eq!(said.len(), 1);
    assert!(said[0].ends_with("kept"), "{said:#?}");
}
