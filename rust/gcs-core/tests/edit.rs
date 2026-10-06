//! Editing the source, which is the document.
//!
//! The property under test is not that an edit produces *a* correct program — a reprint would do
//! that.  It is that an edit leaves everything it did not mean to touch **byte for byte** as it
//! was: the comments, the blank lines, the components, the formatting somebody chose.  That is
//! the difference between source that is the document and source that is a view of one, and it is
//! only visible if you check the characters.

use gcs_core::constraints::{CKind, Constraint};
use gcs_core::edit::{self, Kind};
use gcs_core::style::Classes;
use gcs_core::model::{EntKind, EntRef};
use gcs_core::program::elaborate;
use gcs_core::solve::{solve, SolveOpts};
use crate::common::{parse, STD_POINTS};

/// A little document with everything an edit could wreck: a comment at the top, a comment inside,
/// blank lines, alignment, and a trailing note after the last statement.
const DOC: &str = "\
use std
// a triangle, and this comment must survive every drag
in std.front {
a := point
b := point hint(x: 100, y: 0)
c := point hint(x: 40, y: 70)

ab := line(a, b)      // the base
bc := line(b, c)
ca := line(c, a)

horizontal ab
}
param w := 140
in std.front {
a distance(w) b
fix(x == 0, y == 0) a
}

// and this trailing note, too
";

/// `reconcile` applies itself to the elaboration, which is the point of it — so a test that wants
/// both the text and the elaboration afterwards has to hand it the whole thing.
fn reconciled(e: &mut gcs_core::program::Elaborated) -> edit::Edit {
    let sk = std::mem::take(&mut e.sketch);
    let out = edit::reconcile(e, &sk);
    e.sketch = sk;
    out
}

fn prog_of(src: &str) -> gcs_core::syntax::Program {
    let (p, errs) = parse(src);
    assert!(errs.is_empty(), "{:?}", errs.iter().map(|e| &e.message).collect::<Vec<_>>());
    p
}

/// **The whole point.**  Solve, write the coordinates back, and the only characters that changed
/// are the numbers inside `at (…)`.
#[test]
fn a_solve_writes_back_the_seeds_and_nothing_else() {
    let prog = prog_of(DOC);
    let mut e = elaborate(&prog);
    assert!(e.ok());
    assert!(solve(&mut e.sketch, SolveOpts::default()).success);
    let edit = edit::commit_seeds(&e, &e.sketch, &prog);
    assert_eq!(edit.kind, Kind::Numeric, "a seed is not a statement");

    // every line that is not a point declaration is untouched, character for character
    let before: Vec<&str> = DOC.lines().collect();
    let after: Vec<&str> = edit.text.lines().collect();
    assert_eq!(before.len(), after.len(), "no line was added or lost");
    for (b, a) in before.iter().zip(&after) {
        if b.contains(" := point ") {
            continue;
        }
        assert_eq!(b, a, "a line that is not a seed changed");
    }
    // the comments are all still there, in place
    assert!(edit.text.contains("// a triangle, and this comment must survive every drag"));
    assert!(edit.text.contains("ab := line(a, b)      // the base"));
    assert!(edit.text.contains("// and this trailing note, too"));
    // and it still says the same thing
    let back = elaborate(&prog_of(&edit.text));
    assert!(back.ok());
    assert_eq!(gcs_core::io::dumps(&back.sketch, Some(1)), gcs_core::io::dumps(&e.sketch, Some(1)));
}

/// A seed written over a component's parameter keeps its *name*.  Overwriting `r: Rr` with the
/// number it came to would be the solve editing what the author meant, not where it started.
#[test]
fn a_seed_written_as_an_expression_is_not_overwritten() {
    let src = "\
use std
component Ring(rad: Length) {
  o := point
  c := circle(center: o) hint(r: rad)
  radius(rad) c
  fix(x == 0, y == 0) o
}
in std.front {
g := Ring(rad: 25)
}
";
    let prog = prog_of(src);
    let mut e = elaborate(&prog);
    assert!(e.ok(), "{:?}", e.errors().map(|d| &d.message).collect::<Vec<_>>());
    assert!(solve(&mut e.sketch, SolveOpts::default()).success);
    let edit = edit::commit_seeds(&e, &e.sketch, &prog);
    assert!(edit.text.contains("r: rad"), "the parameter's name stayed:\n{}", edit.text);
}

/// A point inside a `cycle` has one statement and many poses, so there is no one pose to write.
#[test]
fn a_seed_inside_a_block_is_left_alone() {
    let src = "\
use std
in std.front {
o := point
fix(x == 0, y == 0) o
cycle 4 as i {
  p := point hint(x: 10, y: 0)
}
}
";
    let prog = prog_of(src);
    let mut e = elaborate(&prog);
    assert!(e.ok(), "{:?}", e.errors().map(|d| &d.message).collect::<Vec<_>>());
    assert_eq!(e.sketch.points.len(), 5 + STD_POINTS, "one centre and four copies");
    // move every copy somewhere different, then try to write back
    for i in 1..5 {
        let [x, y] = e.sketch.point_params(i);
        e.sketch.params[x as usize].value = 3.0 * i as f64;
        e.sketch.params[y as usize].value = 7.0 * i as f64;
    }
    let edit = edit::commit_seeds(&e, &e.sketch, &prog);
    assert_eq!(edit.kind, Kind::None, "four poses, one statement, nothing to record");
    assert_eq!(edit.text, src);
}

/// Drawing appends a statement, and appends it *before* a trailing comment rather than after the
/// end of the file.
#[test]
fn drawing_a_point_appends_one_statement() {
    let prog = prog_of(DOC);
    let e = edit::add_point(&prog, 12.5, -3.0);
    assert_eq!(e.kind, Kind::Structural);
    assert_eq!(e.names, vec!["p0"], "a name nothing had taken");
    assert!(e.text.contains("p0 := point hint(x: 12.5, y: -3)"), "{}", e.text);
    assert!(e.text.trim_end().ends_with("// and this trailing note, too"), "{}", e.text);
    let back = elaborate(&prog_of(&e.text));
    assert!(back.ok());
    assert_eq!(back.sketch.points.len(), 4 + STD_POINTS);
}

#[test]
fn drawing_into_a_preview_keeps_new_geometry_inside_it() {
    for src in [
        "component Part() {}\npreview {\n}\ntail := 1\n",
        "use std\npreview {\nin std.front {\nexisting := point\n}\n// keep this comment\n}\ntail := 1\n",
    ] {
        let e = edit::add_point(&prog_of(src), 12.5, -3.0);
        let p = prog_of(&e.text);
        let added = p.root().body.iter().find(|st| st.span.slice(p.text()).contains("12.5")).unwrap();
        assert!(p.preview.unwrap().contains(added.span.lo), "{}", e.text);
        assert!(elaborate(&p).ok());
        assert!(e.text.ends_with("tail := 1\n"));
        let rect = edit::add_rectangle(&p, 30.0, 20.0, None);
        let p = prog_of(&rect.text);
        assert!(elaborate(&p).ok());
        let definition = p.component("Rectangle").unwrap();
        assert!(definition.span.hi <= p.preview.unwrap().lo);
        let instance = p.root().body.iter().find(|st| matches!(st.kind,
            gcs_core::syntax::StmtKind::Instance(_))).unwrap();
        assert!(p.preview.unwrap().contains(instance.span.lo));
    }
}

#[test]
fn preview_gestures_keep_source_mappings_with_statements_after_the_block() {
    let mut e = elaborate(&prog_of("use std\npreview {\nin std.front {\nexisting := point\n}\n}\ntail := 1\n"));
    e.sketch.point(200.0, 20.0, false, "");
    let first = reconciled(&mut e);
    assert_eq!(first.kind, Kind::Structural);
    let added = e.map.ent_named("p0").expect("the drawn point has a source name");
    let site = e.map.site_of(added).unwrap();
    assert!(e.program.preview.unwrap().contains(site.span.lo));
    assert!(site.span.slice(e.text()).starts_with("p0 := point"), "{}", site.span.slice(e.text()));
    assert_eq!(reconciled(&mut e).kind, Kind::None);
    let removed = edit::remove(&e, &e.program, &e.sketch, &[added], &[]);
    assert!(removed.text.ends_with("tail := 1\n"));
    assert_eq!(elaborate(&prog_of(&removed.text)).sketch.points.len(), 1 + STD_POINTS);
}

/// **The Rect tool writes a component, once, and instances after.**  The first rectangle
/// brings the `Rectangle` definition with it — the chain of four lines welded at right
/// angles — and every rectangle is one statement, `rN := Rectangle(w: …, h: …)`, each a rigid
/// figure of its own (DOF 3: position and rotation).
#[test]
fn a_rectangle_is_a_component_instance() {
    let prog = prog_of("use std\nin std.front {\np9 := point hint(x: 5, y: 5)\n}\n");
    let e = edit::add_rectangle(&prog, 120.0, 60.0, Some("std.front"));
    assert_eq!(e.kind, Kind::Structural);
    assert_eq!(e.names, vec!["r0"]);
    assert!(e.text.contains("component Rectangle(w: Length, h: Length)"), "{}", e.text);
    assert!(e.text.contains("r0 := Rectangle(w: 120, h: 60)"), "{}", e.text);
    let prog = prog_of(&e.text);
    let back = elaborate(&prog);
    assert!(back.ok(), "{:?}", back.errors().map(|d| d.message.clone()).collect::<Vec<_>>());
    assert_eq!((back.sketch.points.len(), back.sketch.lines.len()), (5 + STD_POINTS, 4));

    // the second rectangle reuses the definition: one component, two instances
    let e = edit::add_rectangle(&prog, 40.0, 40.0, Some("std.front"));
    assert_eq!(e.names, vec!["r1"]);
    assert_eq!(e.text.matches("component Rectangle").count(), 1, "{}", e.text);
    let back = elaborate(&prog_of(&e.text));
    assert!(back.ok(), "{:?}", back.errors().map(|d| d.message.clone()).collect::<Vec<_>>());
    assert_eq!(back.sketch.lines.len(), 8);
    let mut sk = back.sketch.clone();
    let r = solve(&mut sk, SolveOpts::default());
    assert!(r.success, "{}", r.message);
    let d = gcs_core::diagnose::diagnose(&mut sk, gcs_core::diagnose::DiagnoseOptions::default());
    assert_eq!(d.dof, 8, "a loose point (2) and two rigid rectangles (3 each)");
}

/// A minted name never collides with one already written, whatever it is.
#[test]
fn a_minted_name_is_free() {
    let prog = prog_of("use std\nin std.front {\np0 := point hint(x: 0, y: 0)\np1 := point hint(x: 1, y: 0)\np3 := point hint(x: 3, y: 0)\n}\n");
    assert_eq!(edit::mint(&prog, EntKind::Point), "p2", "the first gap, not the next number");
    assert_eq!(edit::mint(&prog, EntKind::Line), "l0");
}

/// Deleting takes out the declaration and every statement that named it — the same rule
/// `io::without` follows on a sketch, said about text.
#[test]
fn deleting_a_point_takes_what_named_it() {
    let prog = prog_of(DOC);
    let e = elaborate(&prog);
    let d = edit::remove(&e, &prog, &e.sketch, &[EntRef::point(2)], &[]);
    assert_eq!(d.kind, Kind::Structural);
    // `c` and the two lines that named it are gone; the base and its dimension are not
    assert!(!d.text.contains("c := point"), "{}", d.text);
    assert!(!d.text.contains("bc := line"), "{}", d.text);
    assert!(!d.text.contains("ca := line"), "{}", d.text);
    assert!(d.text.contains("ab := line(a, b)      // the base"), "{}", d.text);
    assert!(d.text.contains("a distance(w) b"), "{}", d.text);
    assert!(d.text.contains("// a triangle"), "the comments stay");
    let back = elaborate(&prog_of(&d.text));
    assert!(back.ok(), "{:?}", back.errors().map(|x| &x.message).collect::<Vec<_>>());
    assert_eq!(back.sketch.points.len(), 2 + STD_POINTS);
    assert_eq!(back.sketch.lines.len(), 1);
}

/// Deleting something a component made is refused, and says why: the statement makes N of them,
/// and taking the component out is a far larger edit than the gesture asked for.
#[test]
fn deleting_what_a_component_made_is_refused() {
    let src = "\
use std
component Pair() {
  a := point
  b := point hint(x: 10, y: 0)
}
in std.front {
q := Pair()
fix(x == 0, y == 0) q.a
}
";
    let prog = prog_of(src);
    let e = elaborate(&prog);
    assert!(e.ok(), "{:?}", e.errors().map(|d| &d.message).collect::<Vec<_>>());
    let d = edit::remove(&e, &prog, &e.sketch, &[EntRef::point(0)], &[]);
    assert_eq!(d.kind, Kind::None);
    assert!(d.refused.is_some_and(|r| r.contains("component")), "it says why");
    assert_eq!(d.text, src, "and changes nothing");
}

/// Editing a number splices the number.  A plain one is `Numeric` — the topology cannot have
/// moved, so a compiled plan survives it — and one that names anything is not, because the name
/// may be an unknown, and that is a column.  A number typed over a dimension that reads a `param`
/// is the param's new value.
#[test]
fn editing_a_dimension_splices_the_number() {
    let distance = |e: &gcs_core::program::Elaborated| {
        e.sketch.user_constraints().iter()
            .find(|c| c.kind == gcs_core::constraints::CKind::Distance).unwrap().id
    };
    // the dimension reads `param w := 140`: what is typed over it is the param's value
    let prog = prog_of(DOC);
    let e = elaborate(&prog);
    let typed = edit::set_dimension(&e, &prog, distance(&e), "d", "150");
    assert_eq!(typed.kind, Kind::Structural, "a param may feed anything");
    assert!(typed.text.contains("param w := 150"), "{}", typed.text);
    assert!(typed.text.contains("a distance(w) b"), "{}", typed.text);
    assert!(typed.text.contains("// the base"), "everything else is untouched");

    let prog = prog_of(&DOC.replace("a distance(w) b", "a distance(140) b"));
    let e = elaborate(&prog);
    let cid = distance(&e);
    let plain = edit::set_dimension(&e, &prog, cid, "d", "150");
    assert_eq!(plain.kind, Kind::Numeric);
    assert!(plain.text.contains("a distance(150) b"), "{}", plain.text);

    let named = edit::set_dimension(&e, &prog, cid, "d", "w / 2");
    assert_eq!(named.kind, Kind::Structural, "a name may be an unknown, and that is a column");
    let back = elaborate(&prog_of(&named.text));
    assert!(back.ok());
}

/// Nothing here panics on a span that does not name a place in the text it is given, which is
/// what happens the moment two edits are computed against one program and applied in turn.
#[test]
fn a_stale_span_edits_nothing() {
    let prog = prog_of(DOC);
    let e = elaborate(&prog);
    let short = prog_of("use std\nin std.front {\na := point hint(x: 0, y: 0)\n}\n");
    // spans from the long document, applied to the short one
    let d = edit::remove(&e, &short, &e.sketch, &[EntRef::point(2)], &[]);
    let _ = d.text;
    let s = edit::commit_seeds(&e, &e.sketch, &short);
    let _ = s.text;
}

/// **The gear, dragged.**
///
/// A hundred and twenty points move, and the source stays what somebody wrote: the curve family,
/// the `Flank` component, the `cycle`, every comment.  Nothing writes back, because every point
/// in it comes from a statement that makes thirty of them — and that is the correct answer, not
/// a limitation.  A reprint would have replaced the whole file with a hundred and twenty `point`
/// declarations on the first drag.
#[test]
fn dragging_the_gear_does_not_rewrite_the_gear() {
    let prog = prog_of(gcs_core::examples::GEAR);
    let mut e = elaborate(&prog);
    assert!(e.ok());
    assert!(solve(&mut e.sketch, SolveOpts::default()).success);
    // shove every point somewhere else, as a drag would
    for i in 0..e.sketch.points.len() {
        let [x, y] = e.sketch.point_params(i);
        e.sketch.params[x as usize].value += 1.5;
        e.sketch.params[y as usize].value -= 0.5;
    }
    let edit = edit::commit_seeds(&e, &e.sketch, &prog);
    assert_eq!(edit.text, gcs_core::examples::GEAR, "the source is untouched");
    assert!(edit.text.contains("component Involute(c: circle, phase: Angle, u: Angle) {"));
    assert!(edit.text.contains("component Flank("));
    assert!(edit.text.contains("cycle N as i {"));
}

/// A drawing made *by drawing* round-trips: append, elaborate, append again, and each statement
/// is one line that says what the last gesture did.
#[test]
fn a_drawing_built_by_gestures_reads_as_a_program() {
    let mut prog = prog_of("");
    let mut names = Vec::new();
    for (x, y) in [(0.0, 0.0), (60.0, 0.0), (60.0, 40.0)] {
        let e = edit::add_point(&prog, x, y);
        names.push(e.names[0].clone());
        prog = prog_of(&e.text);
    }
    for (a, b) in [(0, 1), (1, 2), (2, 0)] {
        let e = edit::add_entity(&prog, EntKind::Line, &[names[a].clone(), names[b].clone()], &[]);
        prog = prog_of(&e.text);
    }
    let e = elaborate(&prog);
    assert!(e.ok(), "{:?}", e.errors().map(|d| &d.message).collect::<Vec<_>>());
    assert_eq!(e.sketch.points.len(), 3);
    assert_eq!(e.sketch.lines.len(), 3);
    assert_eq!(prog.text().lines().filter(|l| !l.trim().is_empty()).count(), 6);
    assert!(prog.text().contains("l0 := line(p0, p1)"), "{}", prog.text());
}

/* -- a gesture on the drawing, brought back into the source ------------------------ */

/// **Drawing is a way to edit the source.**  A tool mutates the elaborated sketch — that is how it
/// gets to snap and solve while the pointer is still down — and `reconcile` is what makes the
/// document say so afterwards.  It is a splice, so everything around it is left alone.
#[test]
fn a_line_drawn_beside_a_comment_leaves_the_comment_alone() {
    let prog = prog_of(DOC);
    let mut e = elaborate(&prog);
    assert!(e.ok());
    let n = e.sketch.points.len();
    let p = e.sketch.point(200.0, 20.0, false, "");
    let q = e.sketch.point(260.0, 20.0, false, "");
    // drawn on the front, as the app draws them
    let front = e.map.ent_named("std.front").unwrap().i();
    e.sketch.set_plane(p, Some(front));
    e.sketch.set_plane(q, Some(front));
    let l = e.sketch.line(p, q);
    e.sketch.add(Constraint::one_line(CKind::Horizontal, EntRef::line(l)));

    let edit = reconciled(&mut e);
    assert_eq!(edit.kind, Kind::Structural);
    assert_eq!(edit.names, vec!["p0", "p1", "l0"], "each new thing was named, the old kept");
    // everything somebody wrote is still there, character for character
    for line in DOC.lines().filter(|l| !l.trim().is_empty()) {
        assert!(edit.text.contains(line), "lost: {line}\n{}", edit.text);
    }
    assert!(edit.text.contains("l0 := line(p0, p1)"), "{}", edit.text);
    assert!(edit.text.contains("horizontal l0"), "{}", edit.text);

    let back = elaborate(&prog_of(&edit.text));
    assert!(back.ok(), "{:?}", back.errors().map(|d| &d.message).collect::<Vec<_>>());
    assert_eq!(back.sketch.points.len(), n + 2);
    assert_eq!(back.sketch.lines.len(), 4);
    // the source now says exactly what the gesture made: each point where it was, in the plane
    // it was drawn in (the drawn ones numbered before the standard datums now), and every relation
    for n in ["a", "b", "c", "p0", "p1"] {
        let (was, now) = (e.map.ent_named(n).unwrap().i(), back.map.ent_named(n).unwrap().i());
        assert_eq!(back.sketch.point_xy(now), e.sketch.point_xy(was), "{n}");
        assert_eq!(back.sketch.points[now].plane, e.sketch.points[was].plane, "{n}");
    }
    assert_eq!(back.sketch.user_constraints().len(), e.sketch.user_constraints().len());
}

/// A constraint added to the drawing gets a statement; one taken off it loses one.
#[test]
fn a_constraint_added_and_one_removed_are_both_splices() {
    let prog = prog_of(DOC);
    let mut e = elaborate(&prog);
    e.sketch.add(Constraint::one_line(CKind::Vertical, EntRef::line(1)));
    let edit = reconciled(&mut e);
    assert!(edit.text.contains("vertical bc"), "{}", edit.text);
    assert!(edit.text.contains("horizontal ab"), "and the one that was there stays");

    let prog2 = prog_of(&edit.text);
    let mut e2 = elaborate(&prog2);
    let id = e2.sketch.user_constraints().last().unwrap().id;
    e2.sketch.remove(id);
    let back = reconciled(&mut e2);
    assert!(!back.text.contains("vertical bc"), "{}", back.text);
    assert!(back.text.contains("horizontal ab"), "{}", back.text);
    assert!(back.text.contains("// a triangle, and this comment must survive every drag"));
}

/// A gesture beside a gear does not rewrite the gear.  This is the property that makes the source
/// the document rather than a print-out of the drawing.
#[test]
fn drawing_beside_a_component_leaves_the_component_written() {
    let prog = prog_of(gcs_core::examples::GEAR);
    let mut e = elaborate(&prog);
    assert!(e.ok());
    let p = e.sketch.point(200.0, 0.0, false, "");
    let q = e.sketch.point(260.0, 0.0, false, "");
    e.sketch.line(p, q);
    let edit = reconciled(&mut e);
    // every line of the gear is still there, in order: the new statements went in beside it
    let mut rest = edit.text.as_str();
    for line in gcs_core::examples::GEAR.lines() {
        let Some(i) = rest.find(line) else { panic!("lost: {line}") };
        rest = &rest[i + line.len()..];
    }
    assert!(edit.text.contains("cycle N as i {"));
    assert!(edit.text.contains("component Involute(c: circle, phase: Angle, u: Angle) {"));
    let back = elaborate(&prog_of(&edit.text));
    assert!(back.ok(), "{:?}", back.errors().map(|d| &d.message).collect::<Vec<_>>());
    assert_eq!(back.sketch.lines.len(), e.sketch.lines.len());
}

/// **The elaboration takes the edit, and the drawing is not rebuilt.**  That is what lets a tool
/// hold a proxy between two clicks: the sketch it is pointing into is still the sketch.  A second
/// pass finds nothing left to say, which is what "the source is in step" means.
#[test]
fn reconciling_extends_the_map_rather_than_rebuilding_the_drawing() {
    let prog = prog_of(DOC);
    let mut e = elaborate(&prog);
    let before = e.sketch.points.len();
    let p = e.sketch.point(200.0, 20.0, false, "");
    let q = e.sketch.point(260.0, 20.0, false, "");
    e.sketch.line(p, q);

    let first = reconciled(&mut e);
    assert_eq!(first.kind, Kind::Structural);
    assert_eq!(e.sketch.points.len(), before + 2, "the sketch was not rebuilt");
    assert_eq!(e.text(), first.text, "and the elaboration took the edit");
    // the map now names what was drawn, so the *next* edit can splice against it
    assert!(e.map.ent_named("p0").is_some(), "the new point has a name in the map");
    assert!(e.map.site_of(EntRef::line(3)).is_some(), "and the new line has a site");

    let again = reconciled(&mut e);
    assert_eq!(again.kind, Kind::None, "nothing left to say");
    assert_eq!(again.text, first.text);

    // and a deletion afterwards splices against the source it just wrote
    let d = edit::remove(&e, &e.program, &e.sketch, &[EntRef::point(before)], &[]);
    assert!(!d.text.contains("p0 := point"), "{}", d.text);
    assert!(!d.text.contains("l0 := line("), "{}", d.text);
    assert!(d.text.contains("// a triangle, and this comment must survive every drag"));
}

/// A gauge a component wrote is the component's, not the drawing's: `fix(…) center` inside
/// `Gear` says the same thing about `g.center` as a top-level `fix(…) g.center` would, and adding
/// the second is a statement the document did not need and nobody asked for.
#[test]
fn a_gauge_a_component_wrote_is_not_repeated() {
    let prog = prog_of(gcs_core::examples::GEAR);
    let mut e = elaborate(&prog);
    assert!(e.ok());
    assert!(e.sketch.point_fixed(0), "the gear grounds its own centre");
    let edit = reconciled(&mut e);
    assert!(!edit.text.contains(") g.center\n"), "{}", &edit.text[edit.text.len() - 300..]);
    assert_eq!(edit.text, gcs_core::examples::GEAR, "nothing to say, and nothing said");
}

/// **A held number is its own seed** — so fixing a point takes its numbers out of its `hint(…)`,
/// the whole clause where it holds them all, and letting it go writes them back: the source
/// never says one number twice, and never loses where a point it stops holding stands.
#[test]
fn a_fix_takes_the_numbers_it_holds_out_of_the_hint() {
    let prog = prog_of(DOC);
    let mut e = elaborate(&prog);
    assert!(e.ok());
    let b = e.map.ent_named("b").unwrap();
    let c = e.map.ent_named("c").unwrap();
    let x = e.sketch.points[c.i()].x as usize;
    e.sketch.params[x].fixed = true;
    e.sketch.fix_point(b.i(), true);
    let held = reconciled(&mut e);
    assert!(held.text.contains("b := point\n"), "{}", held.text);
    assert!(held.text.contains("c := point hint(y: 70)\n"), "{}", held.text);
    assert!(held.text.contains("fix(x == 40) c\n"), "{}", held.text);
    let back = elaborate(&prog_of(&held.text));
    assert!(back.ok(), "{:?}", back.errors().map(|d| &d.message).collect::<Vec<_>>());
    let at = |e: &gcs_core::program::Elaborated, r: EntRef| {
        let p = &e.sketch.points[r.i()];
        (e.sketch.params[p.x as usize].value, e.sketch.params[p.y as usize].value)
    };
    assert_eq!(at(&back, b), (100.0, 0.0), "held where the fix says, with no hint");
    assert_eq!(at(&back, c), (40.0, 70.0));

    // and let go, the seeds come back, so the drawing re-reads where it stood
    e.sketch.params[x].fixed = false;
    e.sketch.fix_point(b.i(), false);
    let free = reconciled(&mut e);
    assert_eq!(free.text, DOC, "both fixes gone and both hints back, byte for byte");
}

/// A construction flag and a gauge are neither an entity nor a constraint, so nothing about the
/// two counts notices them: they are read off the drawing and compared with what the source says.
#[test]
fn a_gauge_is_spliced_but_presentation_stays_out_of_model_source() {
    let prog = prog_of(DOC);
    let mut e = elaborate(&prog);
    assert!(e.ok());
    e.sketch.lines[1].class = Classes::one("construction");
    let a = e.map.ent_named("a").unwrap();
    let c = e.map.ent_named("c").unwrap();
    e.sketch.fix_point(a.i(), false);
    e.sketch.fix_point(c.i(), true);

    let edit = reconciled(&mut e);
    assert!(!edit.text.contains("class construction"), "{}", edit.text);
    assert!(edit.text.contains("ab := line(a, b)      // the base"), "the comment stayed");
    assert!(edit.text.contains("fix(x == 40, y == 70) c\n"), "{}", edit.text);
    assert!(!edit.text.contains(") a\n"), "the one that was let go is gone:\n{}", edit.text);
    assert!(edit.text.contains("c := point\n"), "the fix is its seed, and the hint goes:\n{}", edit.text);

    let back = elaborate(&prog_of(&edit.text));
    assert!(back.ok(), "{:?}", back.errors().map(|d| &d.message).collect::<Vec<_>>());
    assert!(!back.sketch.lines[1].class.has("construction"));
    assert!(back.sketch.point_fixed(2) && !back.sketch.point_fixed(0));

    // and taking the flag off again takes the word out, leaving the line as it was
    e.sketch.lines[1].class = Default::default();
    let off = reconciled(&mut e);
    assert!(off.text.contains("bc := line(b, c)\n"), "{}", off.text);
}

/// **A statement inside a `cycle` is one statement.**  Thirty instances come from one line of
/// source, and the line is what a span points at, what a caret lands on and what a splice edits.
/// Giving each expanded copy an identity of its own made every one of them name a statement no
/// source has — so a gear's entities could not be found in the text they were written in, and the
/// first gesture beside one lost the map for the whole drawing.
#[test]
fn an_expanded_statement_keeps_the_identity_of_the_one_it_came_from() {
    let prog = prog_of(gcs_core::examples::GEAR);
    let e = elaborate(&prog);
    assert!(e.ok());
    assert!(e.sketch.points.len() > 100);
    for (r, site) in e.map.of_entity.iter() {
        assert!(prog.stmt(site.stmt).is_some(), "{r:?} names a statement no source has");
    }
    for (id, site) in e.map.of_constraint.iter() {
        assert!(prog.stmt(site.stmt).is_some(), "constraint {id} names a statement no source has");
    }
    // and the same line really is reached many times, which is what stops a seed writeback
    let mut reached = std::collections::BTreeMap::new();
    for site in e.map.of_entity.values() {
        *reached.entry(site.stmt).or_insert(0) += 1;
    }
    assert!(reached.values().any(|&n| n >= 30), "one statement, thirty poses");
}

/// Two gestures in a row, on a document made of components.  The second is where the map had to
/// survive the first: an edit is computed against spans, and stale spans splice in the wrong
/// place — or, as this used to, give up and leave the drawing unwritten.
#[test]
fn a_second_gesture_beside_a_component_still_lands() {
    let prog = prog_of(gcs_core::examples::GEAR);
    let mut e = elaborate(&prog);
    let p = e.sketch.point(-95.0, 48.0, false, "");
    let first = reconciled(&mut e);
    assert_eq!(first.kind, Kind::Structural);

    let q = e.sketch.point(-40.0, 60.0, false, "");
    e.sketch.line(p, q);
    let second = reconciled(&mut e);
    assert_eq!(second.kind, Kind::Structural, "{:?}", second.refused);
    assert!(second.text.contains("p0 := point hint(x: -95, y: 48)"), "{}", second.text);
    assert!(second.text.contains("p1 := point hint(x: -40, y: 60)"), "{}", second.text);
    assert!(second.text.contains("l0 := line(p0, p1)"), "{}", second.text);
    assert!(second.text.contains("cycle N as i {"), "and the gear is still written");

    let back = elaborate(&prog_of(&second.text));
    assert!(back.ok(), "{:?}", back.errors().map(|d| &d.message).collect::<Vec<_>>());
    assert_eq!(back.sketch.points.len(), e.sketch.points.len());
    assert_eq!(back.sketch.lines.len(), e.sketch.lines.len());
}

/// **A callout dragged somewhere else is a source edit.**
///
/// Where a callout sits is document state saved on the statement it qualifies (spec §13.1), so
/// moving one has to reach the text — and as a splice of the two numbers, leaving the statement
/// around them alone.  Reaching for it is `reconcile`, the same seam a construction word uses.
#[test]
fn a_dragged_callout_does_not_rewrite_the_model() {
    let src = "use std\nin std.front {\na := point hint(x: 0, y: 0)\nb := point hint(x: 60, y: 0)\na distance(60) b\n}\n";
    let mut e = elaborate(&prog_of(src));
    let id = e.sketch.user_constraints()[0].id;
    for place in [(12.0, -4.0), (20.0, 8.0)] {
        e.sketch.placements.insert(id, place);
        assert_eq!(reconciled(&mut e).text, src);
    }
    e.sketch.placements.remove(&id);
    assert_eq!(reconciled(&mut e).text, src);
}

/// A dimension that is its line's one relation takes the dragged callout's clause at the end
/// of the line's statements, wherever the dimension fell among the links (§13.1).
#[test]
fn a_chained_dimensions_placement_stays_out_of_model_source() {
    let src = "\
use std
in std.front {
p1 := point hint(x: 0, y: 0)
p2 := point hint(x: 60, y: 0)
p3 := point hint(x: 60, y: 40)
p4 := point hint(x: 0, y: 40)
B := line(p3, p4)
(a := line(p1, p2)) angle(30deg) B
}
";
    let mut e = elaborate(&prog_of(src));
    let id = e.sketch.user_constraints().iter().find(|c| c.kind == CKind::Angle).unwrap().id;
    e.sketch.placements.insert(id, (2.0, 5.0));
    let out = reconciled(&mut e);
    assert!(out.refused.is_none(), "{:?}", out.refused);
    assert_eq!(out.text, src);
    // and read back, it is the angle's
    let again = elaborate(&prog_of(&out.text));
    let id = again.sketch.user_constraints().iter().find(|c| c.kind == CKind::Angle).unwrap().id;
    assert_eq!(again.sketch.placements.get(&id).copied(), None);
}

/// A dimension sharing its line with another relation has no spot a placement clause can
/// name, so a dragged callout is not written down — and the gesture still succeeds.
#[test]
fn a_callout_on_a_run_dimension_is_left_to_the_layout() {
    let src = "\
use std
in std.front {
p1 := point hint(x: 0, y: 0)
p2 := point hint(x: 60, y: 0)
p3 := point hint(x: 60, y: 40)
p4 := point hint(x: 0, y: 40)
A := line(p1, p2)
B := line(p3, p4)
A equal angle(30deg) B
}
";
    let mut e = elaborate(&prog_of(src));
    let id = e.sketch.user_constraints().iter().find(|c| c.kind == CKind::Angle).unwrap().id;
    e.sketch.placements.insert(id, (0.5, 10.0));
    let out = reconciled(&mut e);
    assert!(out.refused.is_none(), "{:?}", out.refused);
    assert!(!out.text.contains(" at ("), "nothing lands mid-joint: {}", out.text);
}

/// The same, for a placement on a relation that states no number — the clause stands alone
/// there, so it is written and removed on its own rather than after a `==`.
#[test]
fn a_placement_on_a_relation_is_refused_in_model_source() {
    let (_, errs) = parse("a := point\nb := point\nl := line(a,b)\nhorizontal l at (3,5)\n");
    assert!(errs.iter().any(|e| e.message.contains(".svd")));
}

/// **A seed the document never wrote is recorded where the clause would have gone.**
///
/// A radius is a seed now — `c := circle(center: o) hint(r: 25)` — so it is one a person may
/// perfectly well never write, and a solve still moves it.  There is then no span to splice, and
/// leaving it alone would mean a drawing whose pose the source cannot express.  So the clause is
/// written out whole, at the point the parser recorded for it, and the statement around it is
/// untouched — one splice, and every comment where it was.
#[test]
fn a_seed_the_source_never_wrote_is_appended() {
    let src = "use std\nin std.front {\no := point hint(x: 0, y: 0)\nc := circle(center: o)   // a hole, and this comment stays\n}\n";
    let mut e = elaborate(&prog_of(src));
    let mut sk = std::mem::take(&mut e.sketch);
    let r = sk.circles[0].radius as usize;
    sk.params[r].value = 12.5;
    let out = edit::commit_seeds(&e, &sk, &e.program);
    assert_eq!(out.kind, Kind::Numeric, "a seed is not a statement, wherever it is written");
    assert!(
        out.text.contains("c := circle(center: o) hint(r: 12.5)   // a hole"),
        "the clause is appended, and the comment is where it was: {}",
        out.text
    );

    // and once it is written it splices in place, like every other seed
    let mut e2 = elaborate(&prog_of(&out.text));
    let mut sk2 = std::mem::take(&mut e2.sketch);
    let r2 = sk2.circles[0].radius as usize;
    sk2.params[r2].value = 30.0;
    let again = edit::commit_seeds(&e2, &sk2, &e2.program);
    assert!(again.text.contains("hint(r: 30)"), "{}", again.text);
    assert_eq!(again.text.matches("hint(r:").count(), 1, "spliced, not appended twice");
}

/// A solve writes a seed back **inside** the clause: the spans point at the numbers, not at the
/// words in front of them, so the statement around them is never reprinted.
#[test]
fn a_seed_written_in_a_hint_clause_is_written_back() {
    let src = "use std\nin std.front {\na := point\nb := point hint(x: 10, y: 0)\nl := line(a, b)\nfix(x == 0, y == 0) a\n}\n";
    let mut e = elaborate(&prog_of(src));
    let mut sk = std::mem::take(&mut e.sketch);
    let bx = sk.points[1].x as usize;
    sk.params[bx].value = 42.5;     // drag `b` sideways
    let out = edit::commit_seeds(&e, &sk, &e.program);
    assert_eq!(out.kind, Kind::Numeric);
    assert!(out.text.contains("b := point hint(x: 42.5, y: 0)"), "{}", out.text);
    assert!(out.text.starts_with("use std\nin std.front {\na := point\nb"), "and nothing else moved: {}", out.text);
}

/// **A drag of an anonymous endpoint records itself.**
///
/// `line l(hint(x: 0, y: 0), …)` puts a point's seed in its parent's statement, one level down
/// from where `Decl::seed_spans` looks — so writeback has to find the slot, and splice inside
/// it.  A point named the ordinary way and one written in a slot are the same point, and a
/// gesture on either has to reach the source.
#[test]
fn a_drag_of_an_anonymous_child_is_written_back() {
    let src = "use std\nin std.front {\nl := line(hint(x: 0, y: 0), hint(x: 60, y: 20))   // one line, two points\n}\n";
    let mut e = elaborate(&prog_of(src));
    assert!(e.ok(), "{:?}", e.errors().map(|d| &d.message).collect::<Vec<_>>());
    let mut sk = std::mem::take(&mut e.sketch);
    let p2 = e.map.ent_named("l.p2").expect("the dotted path is its name");
    let [x, y] = sk.point_params(p2.i());
    sk.params[x as usize].value = 42.5;
    sk.params[y as usize].value = -3.0;
    let out = edit::commit_seeds(&e, &sk, &e.program);
    assert_eq!(out.kind, Kind::Numeric);
    assert!(
        out.text.contains("l := line(hint(x: 0, y: 0), hint(x: 42.5, y: -3))   // one line"),
        "spliced in the slot, comment intact: {}",
        out.text
    );
}

/// The same for a declaration that wrote no list at all: there is nothing to splice, so the
/// argument list is written out — the tail of the statement, in one edit, exactly as an omitted
/// scalar's clause is.
#[test]
fn a_drag_of_a_minted_child_writes_the_list() {
    let src = "use std\nin std.front {\nl := line   // two ends, and nothing names them\n}\n";
    let mut e = elaborate(&prog_of(src));
    assert!(e.ok(), "{:?}", e.errors().map(|d| &d.message).collect::<Vec<_>>());
    let mut sk = std::mem::take(&mut e.sketch);
    for (i, r) in ["l.p1", "l.p2"].iter().enumerate() {
        let p = e.map.ent_named(r).expect("named by its path");
        let [x, y] = sk.point_params(p.i());
        sk.params[x as usize].value = 10.0 * (i as f64 + 1.0);
        sk.params[y as usize].value = 0.0;
    }
    let out = edit::commit_seeds(&e, &sk, &e.program);
    assert!(
        out.text.contains("l := line(hint(x: 10, y: 0), hint(x: 20, y: 0))   // two ends"),
        "{}",
        out.text
    );
    // and reading it back is the same drawing, with the same names
    let back = elaborate(&prog_of(&out.text));
    assert!(back.ok());
    assert!(back.map.ent_named("l.p1").is_some());
}

/// The argument list belongs to the **name**, not to wherever the clause happens to sit.
///
/// A declaration's trailers are order-free, so `construction` may stand between the two; a list
/// written at the clause's position would land past it, where an argument list is not something
/// a declaration can say.  And what is written is the printer's spelling — an `arc`'s labels
/// included — since a statement spelled two ways is two spellings of one clause.
#[test]
fn a_written_argument_list_goes_on_the_name_and_is_spelled_once() {
    for (src, want) in [
        ("use std\nin std.front {\nc := circle hint(r: 25)\n}\n", "use std\nin std.front {\nc := circle(center: hint(x: 3, y: 4)) hint(r: 25)\n}\n"),
        ("use std\nin std.front {\nc := circle hint(r: 25)\n}\n",
         "use std\nin std.front {\nc := circle(center: hint(x: 3, y: 4)) hint(r: 25)\n}\n"),
    ] {
        let mut e = elaborate(&prog_of(src));
        assert!(e.ok(), "{src}: {:?}", e.errors().map(|d| &d.message).collect::<Vec<_>>());
        let mut sk = std::mem::take(&mut e.sketch);
        let [x, y] = sk.point_params(e.map.ent_named("c.center").unwrap().i());
        sk.params[x as usize].value = 3.0;
        sk.params[y as usize].value = 4.0;
        let out = edit::commit_seeds(&e, &sk, &e.program);
        assert!(out.text.contains(want), "{src} gave {}", out.text);
        // and what it wrote is a document again
        let back = elaborate(&prog_of(&out.text));
        assert!(back.ok(), "{}: {:?}", out.text,
            back.errors().map(|d| &d.message).collect::<Vec<_>>());
    }
}

/// A slot that keys one coordinate and omits the other still records both.
///
/// `hint(x: 3)` says y is 0, and gives it no span to splice — so the clause it is written in is
/// what gets rewritten, which is the whole reason `KidSeed` carries its own span.  Dropping the
/// coordinate instead would leave a drawing whose source cannot express its pose, which is the
/// case `commit_seeds` exists to prevent.
#[test]
fn a_slot_that_omits_a_coordinate_is_rewritten_whole() {
    let src = "use std\nin std.front {\nl := line(hint(x: 3), hint(x: 60, y: 20))   // one keyed, one not\n}\n";
    let mut e = elaborate(&prog_of(src));
    assert!(e.ok(), "{:?}", e.errors().map(|d| &d.message).collect::<Vec<_>>());
    let mut sk = std::mem::take(&mut e.sketch);
    let p1 = e.map.ent_named("l.p1").expect("the dotted path is its name");
    let [x, y] = sk.point_params(p1.i());
    sk.params[x as usize].value = 7.0;
    sk.params[y as usize].value = -5.0;
    let out = edit::commit_seeds(&e, &sk, &e.program);
    assert!(
        out.text.contains("l := line(hint(x: 7, y: -5), hint(x: 60, y: 20))   // one keyed"),
        "both coordinates, comment intact: {}",
        out.text
    );
}

#[test]
fn driving_radius_dimensions_do_not_acquire_redundant_hints() {
    for source in [
        "use std\nsize := 15\nin std.front {\no := point\nfix(x == 0, y == 0) o\nc := circle(center: o)\nradius(size) c\n}\n",
        "use std\nsize := 15\nin std.front {\no := point\nfix(x == 0, y == 0) o\nc := radius(size) circle(center: o)\n}\n",
        "use std\nsize := 15\nin std.front {\no := point\nfix(x == 0, y == 0) o\nradius(size) c\nc := circle(center: o)\n}\n",
    ] {
        let mut e = elaborate(&prog_of(source));
        assert!(e.ok(), "{:?}", e.diags);
        assert!(solve(&mut e.sketch, Default::default()).success);
        let out = edit::commit_seeds(&e, &e.sketch, &e.program);
        assert!(!out.text.contains("hint(r:"), "{}", out.text);
        let mut reloaded = elaborate(&prog_of(&out.text.replace("size := 15", "size := 32")));
        assert!(solve(&mut reloaded.sketch, Default::default()).success);
        assert!((reloaded.sketch.radius_value(EntRef::circle(0)) - 32.0).abs() < 1e-8);
    }
}

#[test]
fn dimensioned_radius_is_omitted_when_recording_an_unnamed_center() {
    let mut e = elaborate(&prog_of("use std\nin std.front {\nc := radius(12) circle\n}\n"));
    assert!(e.ok(), "{:?}", e.diags);
    assert!(solve(&mut e.sketch, Default::default()).success);
    let center = e.map.ent_named("c.center").unwrap().i();
    let [x, y] = e.sketch.point_params(center);
    e.sketch.params[x as usize].value = 7.0;
    e.sketch.params[y as usize].value = 9.0;
    let out = edit::commit_seeds(&e, &e.sketch, &e.program);
    assert!(!out.text.contains("hint(r:"), "{}", out.text);
    let mut back = elaborate(&prog_of(&out.text));
    assert!(back.ok(), "{:?}", back.diags);
    assert_eq!(back.sketch.point_xy(back.map.ent_named("c.center").unwrap().i()), (7.0, 9.0));
    assert!(solve(&mut back.sketch, Default::default()).success);
    assert!((back.sketch.radius_value(EntRef::circle(0)) - 12.0).abs() < 1e-8);
}

#[test]
fn free_claimed_and_soft_radius_dimensions_keep_pose_hints() {
    for (relation, soft) in [("param unknown: Length\nradius(unknown) c", false),
                             ("claim radius(12) c", false), ("radius(12) c", true)] {
        let e = elaborate(&prog_of(&format!(
            "use std\nin std.front {{\no := point hint(x: 0,y: 0)\nc := circle(center: o)\n}}\n{relation}")));
        assert!(e.ok(), "{:?}", e.diags);
        let mut moved = e.sketch.clone();
        moved.constraints.iter_mut().find(|c| c.kind == CKind::Radius).unwrap().soft = soft;
        let r = moved.circles[0].radius as usize;
        moved.params[r].value = 12.5;
        let out = edit::commit_seeds(&e, &moved, &e.program);
        assert!(out.text.contains("hint(r: 12.5)"), "{}", out.text);
    }
}

#[test]
fn dimensioned_arcs_omit_new_radius_hints_but_authored_hints_are_preserved() {
    let source = "use std\nin std.front {\no := point hint(x: 0,y: 0)\na := point hint(x: 12,y: 0)\nb := point hint(x: 0,y: 12)\nbend := arc(center: o, start: a, end: b)\nradius(12) bend\n}\n";
    let e = elaborate(&prog_of(source));
    assert!(e.ok(), "{:?}", e.diags);
    let out = edit::commit_seeds(&e, &e.sketch, &e.program);
    assert!(!out.text.contains("hint(r:"), "{}", out.text);
    let mut explicit = elaborate(&prog_of("use std\nin std.front {\no := point hint(x: 0,y: 0)\nc := circle(center: o) hint(r: 10)\nradius(12) c\n}\n"));
    assert!(solve(&mut explicit.sketch, Default::default()).success);
    let out = edit::commit_seeds(&explicit, &explicit.sketch, &explicit.program);
    assert!(out.text.contains("hint(r:"), "{}", out.text);
}

/// `use std` lands on a line of its own — after the document's last `use`, or before its first
/// statement, under a heading and `unit` — and nothing changes when the document already says it.
#[test]
fn a_use_is_added_once_on_a_line_of_its_own() {
    let after = edit::add_use(&prog_of("unit mm\nuse hardware\np := point\n"), "std");
    assert_eq!(after.text, "unit mm\nuse hardware\nuse std\np := point\n");
    assert_eq!(after.kind, Kind::Structural);
    let before = edit::add_use(&prog_of("// a heading\nunit mm\n\np := point\n"), "std");
    assert_eq!(before.text, "// a heading\nunit mm\n\nuse std\np := point\n");
    let empty = edit::add_use(&prog_of(""), "std");
    assert_eq!(empty.text, "use std\n");
    let again = edit::add_use(&prog_of(&after.text), "std");
    assert_eq!(again.kind, Kind::None);
    assert_eq!(again.text, after.text);
}

/// A module's statements keep their ids when the document grows.  They used to be numbered after
/// the document's, so a statement a gesture appended renumbered every one of the library's on the
/// next relink, and the map's sites for the standard datums named the wrong statements: the second
/// point drawn on `std.side` took the first one's `in` clause away, and the library's fixed origin
/// was copied into the document as a `fix`.
#[test]
fn appending_to_a_document_that_uses_a_module_keeps_the_module_in_the_map() {
    let (p, errs, linked) = gcs_core::library::parse_linked("use std\nw := 100\n");
    assert!(errs.is_empty() && linked.is_empty());
    let mut e = elaborate(&p);
    let mut sk = e.sketch.clone();
    let side = e.map.ent_named("std.side").unwrap().i();
    for (x, y) in [(5.0, 6.0), (7.0, 8.0), (9.0, 1.0)] {
        let p = sk.point(x, y, false, "");
        sk.set_plane(p, Some(side));
        let out = edit::reconcile(&mut e, &sk);
        assert!(out.refused.is_none(), "{:?}", out.refused);
    }
    let text = e.program.text().to_string();
    assert_eq!(text.matches(" in std.side").count(), 3, "{text}");
    assert!(!text.contains("fix("), "the library's datums are not the document's: {text}");
}
