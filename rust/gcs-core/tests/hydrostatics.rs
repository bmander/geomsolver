//! **Where a body is made, and a face's measures** (#953): a solid kept within a half-space is
//! the body in common with it, built exactly (`solid::defined::trimmed`, the kernel's `half`
//! node); its plane's height may be an unknown a measure drives (`interior::Sheet::Ordinate`); and
//! the drawing states where a body's volume or mass is centred, `g centroid B`, `g centerofmass
//! B`, a face's area, `area(A) F`, and its second moment about a line, `moment(I, about: l) F` —
//! each a row of the drawing over the body's filled mesh moved by its sheets, or a claim judged on
//! the solved drawing.  Each is held to its closed form: a box barge, a cylinder floating at a
//! draft, a wedge heeled about its apex.  The rules are `docs/contract/analysis.md`'s.
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

/// `src` elaborated with no error and solved.
fn sized(src: &str) -> (Elaborated, Sketch) {
    let e = crate::common::read(src);
    let sk = crate::common::solved(&e);
    (e, sk)
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
    assert!(said[0].ends_with("claimed == 360000mm³, kept"), "{}", said[0]);
    assert!(said[1].ends_with("claimed == 12000mm², kept"), "{}", said[1]);
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
claim moment(L * B^3 / 12, about: centreline) immersed.water
param w: Length hint(50mm)
area(L * w) immersed.water
param k: Length hint(10mm)
moment(k * L * B * 1mm, about: centreline) immersed.water
";
    let src = barge(lines).replace("fore := point\naft := point\nfix((0mm, 0mm, 0mm)) fore\n\
        fix((L, 0mm, 0mm)) aft", "fore := point hint((0, 0, 30))\naft := point hint((200, 0, \
        30))\nfix(x == 0mm, y == 0mm) fore\nfix(x == L, y == 0mm) aft\nkeel distance(T, along: \
        std.z) fore\nkeel distance(T, along: std.z) aft");
    let (e, mut sk) = sized(&src);
    close(unknown(&sk, "T"), 40.0, 1e-9, "T");
    let g = at(&e, &sk, "g");
    for (got, want) in g.iter().zip([100.0, 0.0, 20.0]) {
        close(*got, want, 1e-9, "the immersed centroid");
    }
    close(unknown(&sk, "w"), 60.0, 1e-9, "the waterplane's breadth");
    close(unknown(&sk, "k"), 200.0 * 60f64.powi(3) / 12.0 / (200.0 * 60.0), 1e-9, "I / (L B)");
    assert_eq!(crate::common::dof(&mut sk), 3, "the claimed point `f` alone is free");
    let rows = |k: CKind| sk.constraints.iter().filter(|c| c.kind == k).count();
    assert_eq!((rows(CKind::Volume), rows(CKind::Centroid), rows(CKind::Area),
                rows(CKind::Moment)), (1, 3, 1, 1));
    let said = said(&e, &sk);
    assert!(said[0].ends_with("driven == 480000mm³, held"), "{}", said[0]);
    assert!(said[1].starts_with("g centroid immersed: at (100, ") && said[1].ends_with("held"),
            "{}", said[1]);
    assert!(said[2].ends_with("claimed, kept"), "{}", said[2]);
    assert!(said[3].ends_with("claimed == 12000mm², kept"), "{}", said[3]);
    assert!(said[4].ends_with("claimed == 3.6e+06mm⁴, kept"), "{}", said[4]);
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
claim moment(<= {i_hi} * 1mm^4, about: centreline) immersed.water
claim moment(>= {i_lo} * 1mm^4, about: centreline) immersed.water
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
claim moment({i} * 1mm^4, about: centreline) immersed.water
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

/// **What cannot be measured says why** (#953), each where it is written: an area of a solid
/// (a face's), a second moment with no line, a centre of mass of a face (it weighs nothing), a
/// claim reading an unknown (a claim binds none), a centre of mass of a body of no density.
#[test]
fn what_cannot_be_measured_says_why() {
    let cases = [
        ("area(L * B) immersed", "an area is of a face of a solid"),
        ("moment(1 * 1mm^4) immersed.water", "a second moment is about a line"),
        ("g := point\ng centerofmass immersed.water", "a centerofmass is a solid's"),
        ("param w: Length hint(5mm)\nclaim area(L * w) immersed.water", "a claim binds none"),
        ("g := point\ng centerofmass immersed", "made of no material"),
    ];
    for (line, want) in cases {
        let errs = errors(&barge(line));
        assert!(errs.iter().any(|(_, m)| m.contains(want)), "{line}: {errs:?}");
    }
}

/// **The rows agree with their differences** (#953): a centroid's three, an area's and a second
/// moment's over the barge's moved mesh, its waterline an unknown and the moment's centreline in
/// the waterplane, at the solution and knocked off it.
#[test]
fn the_rows_agree_with_finite_differences() {
    let lines = "\
volume(L * B * 40mm) immersed
g := point hint((100, 0, 20))
g centroid immersed
param w: Length hint(50mm)
area(L * w) immersed.water
param k: Length hint(10mm)
moment(k * L * B * 1mm, about: centreline) immersed.water
";
    let src = barge(lines).replace("fore := point\naft := point\nfix((0mm, 0mm, 0mm)) fore\n\
        fix((L, 0mm, 0mm)) aft", "fore := point hint((0, 0, 30))\naft := point hint((200, 0, \
        30))\nfix(x == 0mm, y == 0mm) fore\nfix(x == L, y == 0mm) aft\nkeel distance(T, along: \
        std.z) fore\nkeel distance(T, along: std.z) aft");
    let mut sk = crate::common::solved(&crate::common::read(&src));
    let mut rng = gcs_core::rng::Rng::new(953);
    for round in 0..2 {
        if round > 0 {
            for p in sk.params.iter_mut().filter(|p| !p.fixed) {
                p.value += rng.uniform(-0.01, 0.01) * (1.0 + p.value.abs().min(10.0));
            }
        }
        for c in sk.constraints.iter().filter(|c| c.kind.interior()) {
            let v = c.local_values(&sk);
            let ja = c.jacobian(&sk, &v);
            let jn = gcs_core::fdcheck::fd_jacobian(|w| c.residual(&sk, w), &v, 1e-4);
            let scale = gcs_core::linalg::absmax(&jn.data);
            assert!(scale.is_finite() && scale > 0.0, "{:?}, round {round}", c.kind);
            for (col, (a, n)) in ja.iter().zip(&jn.data).enumerate() {
                assert!((a - n).abs() <= 1e-6 * scale,
                        "{:?}, round {round}: column {col}: {a} against {n}", c.kind);
            }
        }
    }
}
