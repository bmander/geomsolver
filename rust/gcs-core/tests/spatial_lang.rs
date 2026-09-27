//! Views whose attitude or position is solved for, in the language (§6.7;
//! `docs/spatial-constraints-plan.md`, P2a): the plane clauses that name unknowns, the hinges
//! they come to, their seeds, their refusals, and `project` between a stated and a solved view.
use gcs_core::constraints::{CKind, Constraint};
use gcs_core::diagnose::{diagnose, view_freedoms, DiagnoseOptions};
use gcs_core::edit;
use gcs_core::io;
use gcs_core::model::{EntRef, Sketch};
use gcs_core::plane::{fold_rotor, from_quat, quat_mul, to_quat, Basis};
use gcs_core::program::{elaborate, solid_diagnostics, Elaborated};
use gcs_core::solve::{solve, SolveOpts};
use gcs_core::space::{cross, dot, norm, sub};
use gcs_core::syntax::{parse, write_stmt_to};

fn read(src: &str) -> Elaborated {
    let (prog, errs) = parse(src);
    assert!(errs.is_empty(), "does not parse: {errs:?}\n{src}");
    let e = elaborate(&prog);
    assert!(
        e.ok(),
        "does not elaborate: {:?}\n{src}",
        e.errors().map(|d| format!("{} {}", d.code.as_str(), d.message)).collect::<Vec<_>>()
    );
    e
}

fn line(e: &Elaborated, n: &str) -> EntRef {
    e.map.ent_named(n).unwrap_or_else(|| panic!("no `{n}`"))
}

/// A line's two ends where they stand in space.
fn ends(sk: &Sketch, l: EntRef) -> ([f64; 3], [f64; 3]) {
    let l = &sk.lines[l.i()];
    (sk.world_point(l.p1 as usize), sk.world_point(l.p2 as usize))
}

/// The gear and pinion axes of a hypoid layout, each drawn in its own view: the gear's in the
/// front view, the pinion's in a view folded from it by a fold the document solves for.
const AXES: &str = "\
unit mm
point o hint(x: 0, y: 0)
point t hint(x: 40, y: 0)
plane front(origin: o, toward: t)
ground o
ground t
line gax(hint(x: 0, y: 0), hint(x: 0, y: 50)) in front
ground gax.p1
ground gax.p2
point o2 hint(x: 120, y: 0)
point t2 hint(x: 160, y: 0)
plane side(origin: o2, toward: t2, from: front, fold: beta) hint(fold: 30deg)
ground o2
ground t2
line pax(hint(x: 120, y: 10), hint(x: 180, y: 12)) in side
pax.p1 distance(0, along: u) side
pax.p2 distance(60, along: u) side
pax.p1 horizontal pax.p2
";

#[test]
fn the_shaft_angle_and_the_offset_solve_the_fold() {
    let e = read(AXES);
    // the fold is the document's free variable, and says so
    assert!(e.diags.iter().any(|d| d.code.as_str() == "W111" && d.message.contains("beta")),
            "{:?}", e.diags.iter().map(|d| &d.message).collect::<Vec<_>>());
    let mut sk = e.sketch.clone();
    let (gax, pax) = (line(&e, "gax"), line(&e, "pax"));
    let offset = 17.5;
    let angle = Constraint::in_space(&sk, CKind::Angle3, &[gax, pax], Some(90f64.to_radians()))
        .unwrap();
    sk.add(angle);
    let skew = Constraint::in_space(&sk, CKind::LineLine3, &[gax, pax], Some(offset)).unwrap();
    sk.add(skew);
    let r = solve(&mut sk, SolveOpts::default());
    assert!(r.success, "{}", r.message);
    let d = diagnose(&mut sk, DiagnoseOptions::default());
    assert_eq!(d.dof, 0, "{}", gcs_core::diagnose::summary(&d));
    assert!(d.over.is_empty() && d.conflicts.as_deref().unwrap_or(&[]).is_empty());
    // an independent reading: the two axes' ends in space, from the solved bases
    let ((a, b), (c, dd)) = (ends(&sk, gax), ends(&sk, pax));
    let (e1, e2) = (sub(b, a), sub(dd, c));
    let cosang = dot(e1, e2) / (norm(e1) * norm(e2));
    assert!(cosang.abs() < 1e-9, "the shaft angle is 90°: cos = {cosang}");
    let m = cross(e1, e2);
    let gap = dot(m, sub(c, a)).abs() / norm(m);
    assert!((gap - offset).abs() < 1e-9, "the offset is {offset}: {gap}");
    // and the fold came to a half turn's multiple, from a seed of 30°
    let beta = sk.params[sk.free_vars["beta"] as usize].value;
    assert!(beta.rem_euclid(180.0).min(180.0 - beta.rem_euclid(180.0)) < 1e-7, "beta = {beta}");
}

/// The gate fixture's statements in space, stated through the Rust API (the words are P2b's).
fn gate() -> (Elaborated, Sketch) {
    let e = read(AXES);
    let mut sk = e.sketch.clone();
    let (gax, pax) = (line(&e, "gax"), line(&e, "pax"));
    let angle = Constraint::in_space(&sk, CKind::Angle3, &[gax, pax], Some(90f64.to_radians()));
    sk.add(angle.unwrap());
    sk.add(Constraint::in_space(&sk, CKind::LineLine3, &[gax, pax], Some(17.5)).unwrap());
    (e, sk)
}

#[test]
fn the_gate_round_trips_through_json() {
    let (_, mut sk) = gate();
    assert!(solve(&mut sk, SolveOpts::default()).success);
    let text = io::dumps(&sk, None);
    let mut back = io::loads(&text).expect("loads");
    // the same unknowns and the same statements, the hinge among them and the view still hinged
    assert_eq!(back.params.len(), sk.params.len());
    assert_eq!(back.constraints.len(), sk.constraints.len());
    assert!(back.constraints.iter().any(|c| c.kind == CKind::Hinge && c.free.is_some()));
    let side = (0..back.planes.len()).find(|&i| back.plane_name(i) == "v1").unwrap_or(1);
    assert!(back.planes[side].att.as_ref().is_some_and(|a| a.hinged));
    // already solved: nothing moves, and the diagnosis reads the same
    let before: Vec<f64> = back.params.iter().map(|p| p.value).collect();
    assert!(solve(&mut back, SolveOpts::default()).success);
    for (a, p) in before.iter().zip(&back.params) {
        assert!((a - p.value).abs() < 1e-9, "{} moved", p.name);
    }
    assert_eq!(diagnose(&mut back, DiagnoseOptions::default()).dof, 0);
    // and written again, byte for byte
    assert_eq!(io::dumps(&back, None), text);
}

/// `plane::fold_rotor` is the turn `Basis::fold` makes, in the parent's own axes — which is what
/// lets a hinge hold a folded view to a solved parent by a product of quaternions.
#[test]
fn a_fold_is_a_turn_in_the_parents_axes() {
    let parents = [Basis::page(), Basis::page().fold(0.4), Basis::page().fold(-1.2).fold(2.1)];
    for p in parents {
        for theta in [-2.0, -0.5, 0.0, 0.3, 1.5707963267948966, 2.8] {
            let b = from_quat(quat_mul(to_quat(&p), fold_rotor(theta)), p.o).unwrap();
            let want = p.fold(theta);
            for k in 0..3 {
                assert!((b.u[k] - want.u[k]).abs() < 1e-14 && (b.v[k] - want.v[k]).abs() < 1e-14,
                        "{p:?} folded {theta}: {b:?} against {want:?}");
            }
        }
    }
}

/// Each clause prints back as it was written, and what it prints parses to the same print.
#[test]
fn the_clauses_print_back() {
    let src = "\
point o hint(x: 0, y: 0)
point t hint(x: 40, y: 0)
plane front(origin: o, toward: t)
line l(hint(x: 0, y: 0), hint(x: 30, y: 10)) in front
point m hint(x: 5, y: 7) in front
plane a(origin: o, toward: t, from: front, fold: beta) hint(fold: 30deg)
plane b(origin: o, toward: t, from: front, fold: along l)
plane c(origin: o, toward: t, from: front, fold: 0deg, through: m)
plane d(origin: o, toward: t, attitude: free, offset: free) hint(u: (0, 1, 0), v: (0, 0, 1), offset: 5)
plane f(origin: o, toward: t, from: front, offset: free) hint(offset: 12)
";
    let print = |text: &str| -> String {
        let (prog, errs) = parse(text);
        assert!(errs.is_empty(), "{errs:?}\n{text}");
        prog.root().body.iter().map(|st| {
            let mut out = String::new();
            write_stmt_to(&mut out, &st.kind).unwrap();
            out
        }).collect::<Vec<_>>().join("\n")
    };
    let once = print(src);
    for clause in ["fold: beta", "fold: 30deg", "fold: along l", "through: m", "attitude: free",
                   "offset: free", "u: (0, 1, 0)", "v: (0, 0, 1)", "offset: 5", "offset: 12"] {
        assert!(once.contains(clause), "`{clause}` is not in\n{once}");
    }
    assert_eq!(print(&once), once);
    read(src);
}

/// `fold: along l` stands the view square to its parent on the line's bearing and through it,
/// and follows the line when the line moves; it adds no freedom of its own.
#[test]
fn a_fold_along_a_line_follows_the_line() {
    let doc = |deg: f64| format!("\
unit mm
point o hint(x: 0, y: 0)
point t hint(x: 40, y: 0)
plane front(origin: o, toward: t)
ground o
ground t
line base(hint(x: 0, y: 0), hint(x: 50, y: 0)) in front
ground base.p1
ground base.p2
line l(hint(x: 10, y: 5), hint(x: 40, y: 20)) in front
ground l.p1
l.p1 distance(40) l.p2
base angle({deg}deg) l
point o2 hint(x: 120, y: 0)
point t2 hint(x: 160, y: 0)
plane side(origin: o2, toward: t2, from: front, fold: along l)
ground o2
ground t2
");
    for deg in [30.0, 55.0] {
        let e = read(&doc(deg));
        let mut sk = e.sketch.clone();
        assert!(solve(&mut sk, SolveOpts::default()).success);
        let d = diagnose(&mut sk, DiagnoseOptions::default());
        assert_eq!(d.dof, 0, "{}", gcs_core::diagnose::summary(&d));
        let side = e.map.ent_named("side").unwrap().i();
        let b = sk.basis(side);
        let (p1, p2) = ends(&sk, line(&e, "l"));
        let dir = sub(p2, p1);
        // the view contains the line: its direction and its first end
        assert!(dot(b.normal(), dir).abs() < 1e-9 * norm(dir), "{b:?}");
        assert!(dot(b.normal(), sub(p1, b.o)).abs() < 1e-9);
        // square to its parent, and the line at the stated bearing in space
        assert!(dot(b.normal(), Basis::page().normal()).abs() < 1e-12);
        let bearing = dir[2].atan2(dir[0]).to_degrees();
        assert!((bearing - deg).abs() < 1e-7, "{bearing}");
        assert!(view_freedoms(&sk, &d).is_empty());
    }
}

/// `through: M` stands the plane's offset where `M` is, and follows it.
#[test]
fn a_plane_through_a_point_follows_it() {
    let doc = |h: f64| format!("\
unit mm
point o hint(x: 0, y: 0)
point t hint(x: 40, y: 0)
plane front(origin: o, toward: t)
ground o
ground t
point m hint(x: 12, y: 3) in front
o distance(12, along: x) m
o distance({h}, along: y) m
point o3 hint(x: 0, y: 100)
point t3 hint(x: 40, y: 100)
plane top(origin: o3, toward: t3, from: front, fold: 0deg, through: m)
ground o3
ground t3
");
    for h in [8.0, -3.5] {
        let e = read(&doc(h));
        let mut sk = e.sketch.clone();
        assert!(solve(&mut sk, SolveOpts::default()).success);
        let d = diagnose(&mut sk, DiagnoseOptions::default());
        assert_eq!(d.dof, 0, "{}", gcs_core::diagnose::summary(&d));
        let top = e.map.ent_named("top").unwrap().i();
        let b = sk.basis(top);
        // the top view's normal is +z, and the front view's z is its drawn y
        assert!((b.normal()[2] - 1.0).abs() < 1e-12);
        assert!((b.along_normal() - h).abs() < 1e-9, "{} against {h}", b.along_normal());
    }
}

/// A free attitude and a free offset start where their seeds say and are four freedoms, named
/// by the ledger's words.
#[test]
fn a_free_view_starts_at_its_seed() {
    let e = read("\
unit mm
point o hint(x: 0, y: 0)
point t hint(x: 40, y: 0)
ground o
ground t
plane q(origin: o, toward: t, attitude: free, offset: free) hint(u: (0, 1, 0), v: (0, 0, 1), offset: 5)
");
    let mut sk = e.sketch.clone();
    let q = e.map.ent_named("q").unwrap().i();
    let b = sk.basis(q);
    assert_eq!((b.u, b.v, b.o), ([0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [5.0, 0.0, 0.0]));
    let d = diagnose(&mut sk, DiagnoseOptions::default());
    assert_eq!(d.dof, 4, "{}", gcs_core::diagnose::summary(&d));
    assert_eq!(view_freedoms(&sk, &d), vec!["q.attitude".to_string(), "q.offset".to_string()]);
    // held, the attitude alone is three
    let e = read("\
point o hint(x: 0, y: 0)
point t hint(x: 40, y: 0)
ground o
ground t
plane q(origin: o, toward: t, attitude: free)
");
    let mut sk = e.sketch.clone();
    let d = diagnose(&mut sk, DiagnoseOptions::default());
    assert_eq!(d.dof, 3, "{}", gcs_core::diagnose::summary(&d));
    assert_eq!(view_freedoms(&sk, &d), vec!["q.attitude".to_string()]);
    // and `--where q` reports where it stands in space
    let at = gcs_core::report::positions(&sk, &e.map);
    for k in ["q.u.x", "q.v.z", "q.n.y", "q.o.x"] {
        assert!(at.iter().any(|(n, _)| n == k), "no `{k}`");
    }
}

/// A stated view reports what it always did: no `.u`, no `.o`.
#[test]
fn a_stated_view_reports_as_before() {
    let e = read("point o hint(x: 0, y: 0)\npoint t hint(x: 40, y: 0)\nplane q(origin: o, toward: t)\n");
    let at = gcs_core::report::positions(&e.sketch, &e.map);
    assert!(!at.iter().any(|(n, _)| n.starts_with("q.u.") || n.starts_with("q.o.")));
}

/// `project` between a stated view and a solved one is the projector rule in space; the stated
/// view is given held unknowns, and the fold comes out where the two images agree.
#[test]
fn a_projection_between_a_stated_and_a_solved_view() {
    let e = read("\
unit mm
point o hint(x: 0, y: 0)
point t hint(x: 40, y: 0)
plane front(origin: o, toward: t)
ground o
ground t
point o2 hint(x: 120, y: 0)
point t2 hint(x: 160, y: 0)
plane side(origin: o2, toward: t2, from: front, fold: beta) hint(fold: 20deg)
ground o2
ground t2
point a hint(x: 30, y: 40) in front
o distance(30, along: x) a
o distance(40, along: y) a
point b hint(x: 160, y: 0) in side
o2 distance(0, along: y) b
o2 distance(50, along: x) b
a project b
");
    let mut sk = e.sketch.clone();
    assert!(sk.constraints.iter().any(|c| c.kind == CKind::ProjectSolved));
    assert!(solve(&mut sk, SolveOpts::default()).success);
    let d = diagnose(&mut sk, DiagnoseOptions::default());
    assert_eq!(d.dof, 0, "{}", gcs_core::diagnose::summary(&d));
    // b is on the fold line (its v is 0), 50 along it: a's image on the fold line is that far
    // along, so cos β·30 + sin β·40 = 50 — β = atan2(40, 30)
    let beta = sk.params[sk.free_vars["beta"] as usize].value;
    assert!((beta - 40f64.atan2(30.0).to_degrees()).abs() < 1e-4, "beta = {beta}");
    assert!(solid_diagnostics(&sk, &e.map).is_empty());
}

/// Two solved views a `project` relates that come out parallel are refused after the solve.
#[test]
fn views_that_come_out_parallel_are_said() {
    let e = read("\
point o hint(x: 0, y: 0)
point t hint(x: 40, y: 0)
plane front(origin: o, toward: t)
plane q(origin: o, toward: t, attitude: free) hint(u: (1, 0, 0), v: (0, 0, 1))
point a hint(x: 3, y: 4) in front
point b hint(x: 3, y: 4) in q
a project b
");
    let diags = solid_diagnostics(&e.sketch, &e.map);
    assert!(diags.iter().any(|d| d.code.as_str() == "E065" && d.message.contains("parallel")),
            "{diags:?}");
}

fn refused(src: &str, code: &str, needle: &str, at: &str) {
    let (prog, errs) = parse(src);
    assert!(errs.is_empty(), "{errs:?}");
    let e = elaborate(&prog);
    let saw: Vec<String> =
        e.diags.iter().map(|d| format!("{}: {}", d.code.as_str(), d.message)).collect();
    let d = e.diags.iter().find(|d| d.code.as_str() == code && d.message.contains(needle))
        .unwrap_or_else(|| panic!("expected {code} `{needle}`\n{src}\n{saw:#?}"));
    assert_eq!(d.span.slice(prog.text()), at, "{saw:?}");
}

const VIEWS: &str = "\
point o hint(x: 0, y: 0)
point t hint(x: 40, y: 0)
plane front(origin: o, toward: t)
point o2 hint(x: 100, y: 0)
point t2 hint(x: 140, y: 0)
";

#[test]
fn a_seed_for_a_stated_quantity_is_refused_at_its_key() {
    refused(&format!("{VIEWS}plane s(origin: o2, toward: t2, from: front, fold: 30deg) hint(fold: 10deg)\n"),
            "E040", "the fold is stated", "fold");
    refused(&format!("{VIEWS}plane s(origin: o2, toward: t2, from: front, fold: 0deg) hint(u: (1, 0, 0))\n"),
            "E040", "the attitude is stated", "u");
    refused(&format!("{VIEWS}plane s(origin: o2, toward: t2, from: front, offset: 4) hint(offset: 3)\n"),
            "E040", "the offset is stated", "offset");
    // a seed of the wrong kind of number, and half a direction
    refused(&format!("{VIEWS}plane s(origin: o2, toward: t2, from: front, fold: beta) hint(fold: 5mm)\n"),
            "E103", "5mm", "5mm");
    refused(&format!("{VIEWS}plane s(origin: o2, toward: t2, attitude: free) hint(u: (1, 0, 0))\n"),
            "E103", "both `u:` and `v:`", "u");
}

#[test]
fn a_fold_along_a_line_of_another_view_is_refused() {
    refused(&format!("{VIEWS}line l(hint(x: 0, y: 0), hint(x: 5, y: 5))\n\
                      plane s(origin: o2, toward: t2, from: front, fold: along l)\n"),
            "E064", "not drawn in `front`", "l");
}

#[test]
fn a_position_stated_twice_is_refused() {
    refused(&format!("{VIEWS}point m hint(x: 1, y: 2) in front\n\
                      plane s(origin: o2, toward: t2, from: front, offset: 4, through: m)\n"),
            "E064", "stated twice", "m");
    refused(&format!("{VIEWS}line l(hint(x: 0, y: 0), hint(x: 5, y: 5)) in front\n\
                      plane s(origin: o2, toward: t2, from: front, fold: along l, offset: free)\n"),
            "E064", "stated twice", "free");
    refused(&format!("{VIEWS}point m hint(x: 101, y: 2) in s\n\
                      plane s(origin: o2, toward: t2, from: front, fold: 0deg, through: m)\n"),
            "E064", "drawn in `s` itself", "m");
}

/// `against` places a plane by a number worked out before the solve, which a solved view's
/// offset is not: refused until P4.
#[test]
fn against_a_solved_view_is_refused() {
    let square = |tag: &str, plane: &str, lo: &str, hi: &str| format!(
        "point a{tag} hint(x: 0, y: 0) in {plane}\npoint b{tag} hint(x: 20, y: 0) in {plane}\n\
         point c{tag} hint(x: 20, y: 20) in {plane}\npoint d{tag} hint(x: 0, y: 20) in {plane}\n\
         line p{tag}(a{tag}, b{tag}) -> line q{tag}(b{tag}, c{tag}) -> \
         line r{tag}(c{tag}, d{tag}) -> line s{tag}(d{tag}, a{tag}) -> close\n\
         face f{tag}(p{tag}, q{tag}, r{tag}, s{tag})\n\
         solid {tag}(f{tag}, from: {lo}, to: {hi})\n");
    let src = format!("unit mm\npoint o hint(x: 0, y: 0)\npoint qq hint(x: 40, y: 0)\n\
        plane front(origin: o, toward: qq)\n\
        plane back(origin: o, toward: qq, from: front, offset: free) hint(offset: 20)\n\
        {}{}back_part.far against front_part.near\n",
        square("front_part", "front", "-6mm", "0mm"), square("back_part", "back", "-2mm", "2mm"));
    refused(&src, "E066", "solved view", "back_part.far against front_part.near");
}

/// A solve's fold goes back into the source as its seed, where the seed was written.
#[test]
fn the_solved_fold_is_written_back_to_its_seed() {
    let (e, mut sk) = gate();
    assert!(solve(&mut sk, SolveOpts::default()).success);
    let ed = edit::commit_seeds(&e, &sk, &e.program);
    let at = ed.text.rfind(", fold: ").expect("the fold's seed") + ", fold: ".len();
    let written = &ed.text[at..at + ed.text[at..].find("deg)").expect("in degrees, as written")];
    assert!(written.parse::<f64>().unwrap().abs() < 1e-7, "{}", ed.text);
    // and the text it wrote elaborates to the same drawing
    read(&ed.text);
}

/// A skew distance read from a document that left its side out has the side inferred from the
/// geometry, as a `null` there always did — not defaulted to one the drawing may not stand on.
#[test]
fn a_skew_side_left_out_of_a_document_is_read_off_the_geometry() {
    let mut sides = Vec::new();
    // the pinion's axis drawn above the gear's, and below it
    let below = AXES.replace("y: 10), hint(x: 180, y: 12)", "y: -10), hint(x: 180, y: -12)");
    for doc in [AXES.to_string(), below] {
        let e = read(&doc);
        let mut sk = e.sketch.clone();
        let ls = [line(&e, "gax"), line(&e, "pax")];
        let c = Constraint::in_space(&sk, CKind::LineLine3, &ls, Some(5.0)).unwrap();
        let side = c.args[3].clone();
        sk.add(c);
        let text = io::dumps(&sk, None);
        let written = format!("5.0,{}]}}", if side.num() < 0.0 { "-1" } else { "1" });
        assert!(text.contains(&written), "{text}");
        let back = io::loads(&text.replace(&written, "5.0]}")).expect("loads");
        let c = back.constraints.iter().find(|c| c.kind == CKind::LineLine3).unwrap();
        assert_eq!(c.args[3].num(), side.num(), "the side as the geometry has it");
        sides.push(side.num());
    }
    // the two stand on opposite sides, so one of them is the side a default would miss
    assert_eq!(sides[0], -sides[1]);
}

/// A view folded along a line, or stood through a point, is defined from nothing once that line
/// or point is deleted, and goes with it — as a view folded from a deleted view does.
#[test]
fn deleting_what_a_view_stands_on_deletes_the_view() {
    let src = format!("{VIEWS}line l(hint(x: 0, y: 0), hint(x: 5, y: 5)) in front\n\
                       point m hint(x: 1, y: 2) in front\n\
                       plane s(origin: o2, toward: t2, from: front, fold: along l)\n\
                       plane u(origin: o2, toward: t2, from: front, fold: 0deg, through: m)\n");
    let e = read(&src);
    let out = edit::remove(&e, &e.program, &e.sketch, &[line(&e, "l")], &[]);
    assert!(!out.text.contains("plane s(") && out.text.contains("plane u("), "{}", out.text);
    let m = e.map.ent_named("m").unwrap();
    let out = edit::remove(&e, &e.program, &e.sketch, &[m], &[]);
    assert!(out.text.contains("plane s(") && !out.text.contains("plane u("), "{}", out.text);
    read(&out.text);
}

/// A solved fold inside a component is that instance's own unknown, as any name its body leaves
/// undefined is: two instances fold independently.
#[test]
fn a_solved_fold_in_a_component_is_the_instances_own() {
    let e = read("\
point o hint(x: 0, y: 0)
point t hint(x: 40, y: 0)
plane front(origin: o, toward: t)
component Wing(f: plane) {
  point a hint(x: 100, y: 0)
  point b hint(x: 140, y: 0)
  plane w(origin: a, toward: b, from: f, fold: beta) hint(fold: 15deg)
}
one: Wing(front)
two: Wing(front)
");
    let names: Vec<&String> = e.sketch.free_vars.keys().collect();
    assert_eq!(names.len(), 2, "{names:?}");
    assert_eq!(e.sketch.constraints.iter().filter(|c| c.kind == CKind::Hinge).count(), 2);
}

/* -- P2b: the words across views ----------------------------------------------------------- */

/// The gate, written in words: the shaft angle and the offset are `angle` and `distance` between
/// two axes drawn in different views, and so are relations in space.
const AXES_IN_WORDS: &str = "\
gax angle(90deg) pax
gax distance(17.5) pax
";

#[test]
fn the_gate_in_words_solves_the_fold() {
    let e = read(&format!("{AXES}{AXES_IN_WORDS}"));
    let mut sk = e.sketch.clone();
    let kinds: Vec<CKind> = sk.user_constraints().iter().map(|c| c.kind).collect();
    assert!(kinds.contains(&CKind::Angle3) && kinds.contains(&CKind::LineLine3), "{kinds:?}");
    let r = solve(&mut sk, SolveOpts::default());
    assert!(r.success, "{}", r.message);
    let d = diagnose(&mut sk, DiagnoseOptions::default());
    assert_eq!(d.dof, 0, "{}", gcs_core::diagnose::summary(&d));
    let ((a, b), (c, dd)) = (ends(&sk, line(&e, "gax")), ends(&sk, line(&e, "pax")));
    let (e1, e2) = (sub(b, a), sub(dd, c));
    assert!((dot(e1, e2) / (norm(e1) * norm(e2))).abs() < 1e-9);
    let m = cross(e1, e2);
    assert!((dot(m, sub(c, a)).abs() / norm(m) - 17.5).abs() < 1e-9);
    // the same solve the Rust API's statements (P2a's gate) come to, point for point in space
    let (ge, mut gk) = gate();
    assert!(solve(&mut gk, SolveOpts::default()).success);
    let (gc, gd) = ends(&gk, line(&ge, "pax"));
    for (x, y) in [(c, gc), (dd, gd)] {
        assert!(norm(sub(x, y)) < 1e-9, "{x:?} against {y:?}");
    }
    // and the statements round-trip through JSON as the kinds they settled to
    let back = io::loads(&io::dumps(&sk, None)).expect("loads");
    let kinds: Vec<CKind> = back.user_constraints().iter().map(|c| c.kind).collect();
    assert!(kinds.contains(&CKind::Angle3) && kinds.contains(&CKind::LineLine3), "{kinds:?}");
    assert_eq!(io::dumps(&back, None), io::dumps(&sk, None));
    // and lifted to a program they are spelled in words, and read back in space
    let lifted = gcs_core::program::to_program(&sk);
    let text = lifted.text().to_string();
    assert!(text.contains("angle(90") && text.contains("distance(17.5)"), "{text}");
    let again = read(&text);
    let kinds: Vec<CKind> = again.sketch.user_constraints().iter().map(|c| c.kind).collect();
    assert!(kinds.contains(&CKind::Angle3) && kinds.contains(&CKind::LineLine3), "{kinds:?}\n{text}");
}

/// Two stated views, square to each other, with a point, a line and a circle drawn in each.
const TWO_VIEWS: &str = "\
unit mm
point o hint(x: 0, y: 0)
point t hint(x: 40, y: 0)
plane front(origin: o, toward: t)
point o2 hint(x: 120, y: 0)
point t2 hint(x: 160, y: 0)
plane side(origin: o2, toward: t2, from: front, fold: 90deg)
point a hint(x: 10, y: 20) in front
point b hint(x: 130, y: 5) in side
line la(hint(x: 0, y: 0), hint(x: 30, y: 10)) in front
line lb(hint(x: 125, y: 3), hint(x: 150, y: 20)) in side
circle cb(hint(x: 140, y: 10)) hint(r: 8) in side
";

/// The kind one statement over `TWO_VIEWS` settles to.
fn settles(stmt: &str) -> CKind {
    let e = read(&format!("{TWO_VIEWS}{stmt}\n"));
    let cs = e.sketch.user_constraints();
    assert_eq!(cs.len(), 1, "{stmt}: {:?}", cs.iter().map(|c| c.kind).collect::<Vec<_>>());
    cs[0].kind
}

/// Every word that has a meaning in space means it across views, and the same word within one
/// view is the 2D relation it always was.
#[test]
fn each_word_across_views_is_the_relation_in_space() {
    for (stmt, kind) in [
        ("a coincident b", CKind::Coincident3),
        ("a distance(30) b", CKind::Distance3),
        ("a distance(5) lb", CKind::PointLine3),
        ("la distance(5) lb", CKind::LineLine3),
        ("la angle(60deg) lb", CKind::Angle3),
        ("la perpendicular lb", CKind::Perpendicular3),
        ("la parallel lb", CKind::Parallel3),
        ("la equal lb", CKind::EqualLength3),
        ("a on lb", CKind::PointOnLine3),
        ("a on cb", CKind::PointOnCircle3Fixed),
        // a plane is a place in space whatever view the point is drawn in
        ("a on side", CKind::PointOnPlaneFixed),
        ("la on side", CKind::LineOnPlaneFixed),
        ("a distance(5, along: n) side", CKind::PointPlaneDistanceFixed),
        // within one view, the page's words
        ("a distance(30) la.p1", CKind::Distance),
        ("b on lb", CKind::PointOnLine),
        ("la angle(60deg) la", CKind::Angle),
    ] {
        assert_eq!(settles(stmt), kind, "{stmt}");
    }
}

/// A statement in space reads back as the word it was written with.
#[test]
fn a_relation_in_space_is_described_by_its_word() {
    let e = read(&format!("{TWO_VIEWS}la distance(5) lb\na distance(3, along: n) side\n"));
    let said: Vec<String> = e.sketch.user_constraints().iter()
        .map(|c| io::describe_with(c, &|r| e.map.name_of(r).cloned())).collect();
    assert_eq!(said, vec!["la distance(5) lb".to_string(), "a distance(3, along: n) side".to_string()]);
}

/// A plane's own datum points are read by their role: among themselves they are sheet layout, and
/// beside a view's points they are that view's — never a relation in space.
#[test]
fn datum_points_are_read_by_their_role() {
    // layout: two views' origins, on the page
    assert_eq!(settles("o distance(120) o2"), CKind::Distance);
    assert_eq!(settles("o distance(120, along: x) o2"), CKind::HorizontalDistance);
    // an ordinate from the datum a view is drawn from, in that view
    assert_eq!(settles("o2 distance(15) b"), CKind::Distance);
    assert_eq!(settles("o distance(10, along: x) a"), CKind::HorizontalDistance);
}

fn refused_as(stmt: &str, code: &str, needle: &str, at: &str) {
    refused(&format!("{TWO_VIEWS}{stmt}\n"), code, needle, at);
}

/// A word with no meaning in space, used across views, is refused and says so; a selector that
/// names a page direction says nothing in space and is refused at its key.
#[test]
fn a_word_with_no_meaning_in_space_is_refused_across_views() {
    refused_as("a horizontal b", "E062", "no meaning in space", "a horizontal b");
    refused_as("a distance(5, along: x) b", "E062", "no meaning in space", "a distance(5, along: x) b");
    refused_as("la tangent cb", "E062", "no meaning in space", "la tangent cb");
    refused_as("a midpoint lb", "E062", "no meaning in space", "a midpoint lb");
    refused_as("la angle(60deg, sense: cw) lb", "E040", "unsigned", "sense");
    refused_as("a distance(5, side: left) lb", "E040", "magnitude", "side");
    // a point on the page has no place in space, nor a datum point read beside another view
    refused_as("point pg hint(x: 1, y: 1)\npg distance(5) b", "E062", "on the page", "pg distance(5) b");
    refused_as("o2 distance(5) a", "E062", "on the page", "o2 distance(5) a");
    // a point on its own view, and a sphere against a circle, which is still to come
    refused_as("b on side", "E061", "every point of a view is on it", "b on side");
}

/// The gate's view points left ungrounded: a solved view's place on the sheet is held silently,
/// so the ledger is still 0 and counts no freedom of where the picture sits.
#[test]
fn a_solved_views_page_placement_is_held_silently() {
    let bare = AXES.replace("ground o\n", "").replace("ground t\n", "")
        .replace("ground o2\n", "").replace("ground t2\n", "");
    let e = read(&format!("{bare}{AXES_IN_WORDS}"));
    let mut sk = e.sketch.clone();
    assert_eq!(sk.page_held.len(), 4, "o, t, o2 and t2");
    assert!(solve(&mut sk, SolveOpts::default()).success);
    let d = diagnose(&mut sk, DiagnoseOptions::default());
    assert_eq!(d.dof, 0, "{}", gcs_core::diagnose::summary(&d));
    // the hold is the gauge's, not a `ground`: nothing is written back, and nothing is lifted
    let ed = edit::commit_seeds(&e, &sk, &e.program);
    assert!(!ed.text.contains("ground o"), "{}", ed.text);
    // a free view alone is its attitude and its offset, and nothing of where it is drawn
    let free = "point o hint(x: 0, y: 0)\npoint t hint(x: 40, y: 0)\n\
                plane q(origin: o, toward: t, attitude: free, offset: free)\n";
    let mut sk = read(free).sketch;
    let d = diagnose(&mut sk, DiagnoseOptions::default());
    assert_eq!(d.dof, 4, "{}", gcs_core::diagnose::summary(&d));
    // until the document says something about the datum points: then they are its own
    let mut sk = read(&format!("{free}o distance(40) t\n")).sketch;
    assert!(sk.page_held.is_empty());
    let d = diagnose(&mut sk, DiagnoseOptions::default());
    assert_eq!(d.dof, 4 + 3, "{}", gcs_core::diagnose::summary(&d));
}

/// A stated view a solved one reads is given held unknowns, and its unit row is no equation the
/// ledger counts: the gate reads as many equations as its rank.
#[test]
fn a_held_views_unit_row_is_not_counted() {
    let e = read(&format!("{AXES}{AXES_IN_WORDS}"));
    let mut sk = e.sketch.clone();
    assert!(solve(&mut sk, SolveOpts::default()).success);
    let d = diagnose(&mut sk, DiagnoseOptions::default());
    assert_eq!(d.n_equations, d.structural_rank, "{}", gcs_core::diagnose::summary(&d));
    assert_eq!(d.n_equations, d.n_params, "{}", gcs_core::diagnose::summary(&d));
}

/// A sphere about a centre drawn in one view: a point of another view on it, its radius, and its
/// tangency to a line and to a second sphere, each in space.
#[test]
fn a_sphere_takes_its_words_in_space() {
    let with = |stmt: &str| format!(
        "{TWO_VIEWS}sphere s(hint(x: 140, y: 10)) hint(r: 12) in side\n\
         sphere s2(hint(x: 10, y: 40)) hint(r: 5) in front\n{stmt}\n");
    let kind = |stmt: &str| read(&with(stmt)).sketch.user_constraints()[0].kind;
    assert_eq!(kind("a on s"), CKind::SphereOn);
    assert_eq!(kind("radius(12) s"), CKind::SphereRadius);
    assert_eq!(kind("s tangent la"), CKind::SphereTangentLine);
    assert_eq!(kind("s tangent s2"), CKind::SphereTangentSphere);
    refused(&with("s tangent cb"), "E040", "a sphere is tangent to a line or to another sphere", "tangent");
    // and solved: the centre held, a point of the other view on it, and a line tangent to it
    let e = read(&with("ground s.center\na on s\nground la.p1\nground la.p2\ns tangent la"));
    let mut sk = e.sketch.clone();
    let r = solve(&mut sk, SolveOpts::default());
    assert!(r.success, "{}", r.message);
    let s = e.map.ent_named("s").unwrap();
    let c = sk.world_point(sk.round_center(s));
    let rad = sk.radius_value(s);
    let pa = sk.world_point(e.map.ent_named("a").unwrap().i());
    assert!((norm(sub(pa, c)) - rad).abs() < 1e-9, "{} against {rad}", norm(sub(pa, c)));
    let (l1, l2) = ends(&sk, line(&e, "la"));
    let dl = sub(l2, l1);
    let gap = norm(cross(sub(c, l1), dl)) / norm(dl);
    assert!((gap - rad).abs() < 1e-9, "the line touches the sphere: {gap} against {rad}");
    // on the sphere is one equation: a point of the other view keeps one of its two freedoms
    let dof = |src: String| {
        let mut sk = read(&src).sketch;
        assert!(solve(&mut sk, SolveOpts::default()).success);
        diagnose(&mut sk, DiagnoseOptions::default()).dof
    };
    let tied = "ground s.center\nground la.p1\nground la.p2\ns tangent la";
    assert_eq!(dof(with(tied)) - dof(with(&format!("{tied}\na on s"))), 1);
}
