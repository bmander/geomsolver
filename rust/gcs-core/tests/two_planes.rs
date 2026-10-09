//! **A point drawn in two planes** (§6.7, #145): `O := point in P, G` is on both, so where they
//! meet, and drawn in each — by a twin in every further plane, tied to it in space.  What is
//! drawn in a plane reads the point there: an arc in `G` centred on it, a line in `G` through
//! it, a relation with a point of `G`.  The duplicated points and `project`s a descriptive
//! layout wrote to say so are gone.
use gcs_core::constraints::CKind;
use gcs_core::diagnose::{diagnose, DiagnoseOptions, State};
use gcs_core::edit;
use gcs_core::program::Elaborated;
use gcs_core::solve::{solve, SolveOpts};

use crate::common::{ent, read, refused};

/// The top plane and a plane `G` square to it along the x axis: they meet on that axis.
const FOLDED: &str =
    "unit mm\nuse std\nG := plane(u: std.x, v: std.z)\nfix(origin == (0, 0, 0)) G\n";

fn solved(src: &str) -> Elaborated {
    let mut e = read(src);
    let r = solve(&mut e.sketch, SolveOpts::default());
    assert!(r.success, "{}", r.message);
    e
}

fn kinds(e: &Elaborated) -> Vec<CKind> {
    e.sketch.user_constraints().iter().map(|c| c.kind).collect()
}

/// The point is on both planes, so on the line they meet on: one freedom, along it.
#[test]
fn a_point_in_two_planes_is_where_they_meet() {
    let mut e = solved(&format!("{FOLDED}O := point hint((10, 3)) in std.top, G\n"));
    let o = e.sketch.world_point(ent(&e, "O").i());
    assert!(o[1].abs() < 1e-9 && o[2].abs() < 1e-9, "on the x axis: {o:?}");
    let d = diagnose(&mut e.sketch, DiagnoseOptions::default());
    assert_eq!(d.dof, 1);
    assert!(d.implied.is_empty() && d.over.is_empty());
}

/// What is drawn in `G` reads the point there: an arc centred on it, a line through it, a
/// distance to a point of `G` — the plane's own 2D relation, not one in space.
#[test]
fn what_is_drawn_in_the_second_plane_reads_it_there() {
    let e = solved(&format!(
        "{FOLDED}O := point hint((10, 0)) in std.top, G\nin G {{\n  b := point hint((10, -8))\n  \
         t := point hint((10, 8))\n  k := arc(center: O, start: b, end: t)\n  \
         q := point hint((30, 5))\n}}\nO distance(25) q\n"
    ));
    assert_eq!(kinds(&e), vec![CKind::Distance]);
    let centre = e.sketch.arcs[0].center as usize;
    let g = ent(&e, "G").i();
    assert_eq!(e.sketch.plane_of(centre), Some(g));
    let (a, b) = (e.sketch.world_point(centre), e.sketch.world_point(ent(&e, "O").i()));
    assert!(a.iter().zip(&b).all(|(x, y)| (x - y).abs() < 1e-9), "{a:?} {b:?}");
}

/// A revolution drawn in `G` about a line through the point: its axis is in the face's plane.
#[test]
fn a_solid_in_the_second_plane_turns_about_a_line_through_it() {
    let e = solved(&format!(
        "{FOLDED}O := point hint((0, 0)) in std.top, G\nfix((0, 0)) O\nin G {{\n  \
         T := point hint((0, 20))\n  a := point hint((5, 2))\n  c := point hint((10, 2))\n  \
         h := point hint((10, 12))\n  ax := line(O, T)\n}}\nfix((0, 20)) T\nfix((5, 2)) a\n\
         fix((10, 2)) c\nfix((10, 12)) h\nf := face(a, c, h, -> close)\n\
         ring := solid(f, about: ax)\n"
    ));
    assert_eq!(e.sketch.solids.len(), 1);
}

/// A plane on two lines that meet at the point, each through its own image of it — here one drawn
/// in a plane, one standing in space — stands at it as at any shared end: one tie of its origin,
/// not the two axes' rows over it, which say the origin twice (the sundial's equator, on a ray
/// east along the dial and one toward the noon sun, both from the dial's centre).
#[test]
fn a_plane_on_two_lines_through_the_point_stands_at_it_once() {
    let src = "unit mm\nuse std\nO := point in std.top, std.side\nfix(y == 0) O\n\
               in std.side {\n  T := point hint((40, 50))\n  style := line(O, T)\n}\n\
               fix((40, 50)) T\nS := point hint((-20, -30, 30))\nfix((-20, -30, 30)) S\n\
               ray := line(O, S)\nH := plane(u: style, v: ray)\n";
    let mut e = solved(src);
    let (style, ray) = (ent(&e, "style").i(), ent(&e, "ray").i());
    let (a, b) = (e.sketch.lines[style].p1 as usize, e.sketch.lines[ray].p1 as usize);
    assert_ne!(a, b, "each line reads its own image of O");
    let d = diagnose(&mut e.sketch, DiagnoseOptions::default());
    assert_eq!((d.dof, d.status), (0, State::Well));
    assert_eq!(d.n_params, d.n_equations, "nothing said twice");
    let h = ent(&e, "H").i();
    let o = e.sketch.planes[h].o.map(|q| e.sketch.params[q as usize].value);
    assert!(o.iter().all(|x| x.abs() < 1e-9), "H stands at O: {o:?}");
}

/// A twin goes by its point's name: a relation read in the second plane names `O`, and a sheet
/// sketching that plane draws its dimension.
#[test]
fn a_twin_is_called_by_its_points_name() {
    let src = format!(
        "{FOLDED}O := point hint((0, 0)) in std.top, G\nfix(x == 0) O\n\
         F := point hint((20, 0)) in std.top, G\n\
         in G {{\n  base := line(O, F)\n  distance(20) base\n}}\n"
    );
    let e = solved(&src);
    let c = e.sketch.user_constraints().into_iter().find(|c| c.kind == CKind::Distance).unwrap();
    let said = gcs_core::io::describe_with(c, &|r| e.map.name_of(r).cloned());
    assert_eq!(said, "O distance(20) F");
    let doc = gcs_core::drawing::parse(
        "model m from \"x.sv\"\nsheet s {\n  sketch g(m) from m.G at (50mm, 50mm)\n  \
         dimensions in g\n}\n",
    )
    .unwrap();
    let models = std::collections::BTreeMap::from([(
        "m".into(),
        gcs_core::drawing::Model { sketch: &e.sketch, names: &e.map },
    )]);
    let svg = gcs_core::drawing::render(&doc, &models, None).unwrap();
    assert!(svg.contains(">20<"), "the dimension is drawn: {svg}");
}

/// A plane built through the point — folded along a line of the first plane through it — holds
/// it already: one of the tie's three rows is what the fold says, found by the rank and said
/// nowhere.  The drawing is as determined as it is.
#[test]
fn a_plane_built_through_the_point_says_nothing_more() {
    // O on F's line, by a relation, and O at its end: F's axis along `gen` already holds O on F,
    // so the tie says only where O meets the fold, and the structural count is square as the
    // rank is
    // F folded along `gen` as the gear's views are (`views.FoldedView`)
    let folded = "F := plane(u: gen, v: hint(dir: (0, 0, -1)))\nstd.top perpendicular F.v\n\
                  std.top.origin project F.origin\n";
    let on = format!(
        "unit mm\nuse std\nin std.top {{\n  o := point hint((0, 0))\n  M := point hint((40, 0))\n  \
         gen := line(o, M)\n}}\nfix((0, 0)) o\nfix((40, 0)) M\n{folded}\
         O := point hint((20, 0)) in std.top, F\nO distance(15) M\nO coincident gen\n"
    );
    let end = format!(
        "unit mm\nuse std\no := point hint((0, 0)) in std.top\nfix((0, 0)) o\n\
         O := point hint((40, 0)) in std.top, F\nfix((40, 0)) O\ngen := line(o, O)\n{folded}"
    );
    for text in [on, end] {
        let mut e = solved(&text);
        let mut sys = gcs_core::system::System::new(&e.sketch);
        let order = sys.block_order();
        assert!(order.over_rows.is_empty() && order.under_cols.is_empty(), "{text}");
        let d = diagnose(&mut e.sketch, DiagnoseOptions::default());
        assert_eq!((d.dof, d.status), (0, State::Well), "{text}: {:?}", d.warnings);
        assert!(d.implied.is_empty() && d.over.is_empty(), "{:?} {:?}", d.implied, d.over);
    }
}

#[test]
fn a_point_is_drawn_in_planes_that_meet() {
    let twice = "O := point in std.top, std.top";
    refused(&format!("{FOLDED}{twice}\n"), "E040", "once", twice);
    refused(&format!("{FOLDED}H := plane(u: std.x, v: std.y)\nfix(origin == (0, 0, 9)) H\n\
             O := point in std.top, H\n"), "E061", "parallel", "O := point in std.top, H");
}

#[test]
fn only_a_point_is_drawn_in_two_planes() {
    let text = format!("{FOLDED}l := line in std.top, G\n");
    let (_, errs, _) = gcs_core::library::parse_linked(&text);
    assert!(errs.iter().any(|e| e.message.contains("drawn in one")), "{errs:?}");
}

/// The source says it once: a solve writes the point's seed in its first plane and never a
/// twin's, the program prints `in P, G` and no twin, and a document keeps the twins.
#[test]
fn the_source_says_it_once() {
    let src = format!(
        "{FOLDED}O := point hint((10, 3)) in std.top, G\nfix(x == 12) O\n\
         q := point hint((5, 5)) in G\nO distance(9) q\n"
    );
    let e = read(&src);
    let mut sk = e.sketch.clone();
    assert!(solve(&mut sk, SolveOpts::default()).success);
    let out = edit::commit_seeds(&e, &sk, &e.program).text;
    let line = out.lines().find(|l| l.starts_with("O :=")).unwrap();
    assert!(line.starts_with("O := point hint(y: ") && line.ends_with(") in std.top, G"), "{out}");
    // printed and read back: the same drawing, the twin said by its point
    let printed = gcs_core::program::to_program(&e.sketch).text().to_string();
    let two = |l: &&str| {
        l.contains(":= point") && l.split(" in ").nth(1).is_some_and(|t| t.contains(", "))
    };
    assert_eq!(printed.lines().filter(two).count(), 1, "{printed}");
    let back = read(&printed);
    assert_eq!(back.sketch.twins.len(), 1);
    assert_eq!(back.sketch.topology_key(), e.sketch.topology_key());
    // and a document carries the twins, tied again on load (with no partial hold, which a
    // document does not keep, twin or no twin)
    let free = read(&src.replace("fix(x == 12) O\n", ""));
    let loaded = gcs_core::io::loads(&gcs_core::io::dumps(&free.sketch, None)).unwrap();
    assert_eq!(loaded.twins, free.sketch.twins);
    assert_eq!(loaded.topology_key(), free.sketch.topology_key());
}

/// Deleting a further plane takes it out of the clause and leaves the point drawn in the rest.
#[test]
fn deleting_a_plane_keeps_the_others() {
    let e = read(&format!("{FOLDED}O := point hint((10, 0)) in std.top, G\n"));
    let g = ent(&e, "G");
    let out = edit::remove(&e, &e.program, &e.sketch, &[g], &[]);
    let line = out.text.lines().find(|l| l.starts_with("O :=")).unwrap_or_default().to_string();
    assert_eq!(line, "O := point hint((10, 0)) in std.top", "{}", out.text);
}
