//! Editing the source, which is the document.
//!
//! The property under test is not that an edit produces *a* correct program — a reprint would do
//! that.  It is that an edit leaves everything it did not mean to touch **byte for byte** as it
//! was: the comments, the blank lines, the components, the formatting somebody chose.  That is
//! the difference between source that is the document and source that is a view of one, and it is
//! only visible if you check the characters.

use gcs_core::constraints::{CKind, Constraint};
use gcs_core::edit::{self, Kind};
use gcs_core::model::{EntKind, EntRef};
use gcs_core::program::elaborate;
use gcs_core::solve::{solve, SolveOpts};
use gcs_core::syntax::parse;

/// A little document with everything an edit could wreck: a comment at the top, a comment inside,
/// blank lines, alignment, and a trailing note after the last statement.
const DOC: &str = "\
// a triangle, and this comment must survive every drag
point a hint(x: 0, y: 0)
point b hint(x: 100, y: 0)
point c hint(x: 40, y: 70)

line ab(a, b)      // the base
line bc(b, c)
line ca(c, a)

horizontal(ab)
distance(a, b) == w = 140
ground(a)

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
        if b.starts_with("point ") {
            continue;
        }
        assert_eq!(b, a, "a line that is not a seed changed");
    }
    // the comments are all still there, in place
    assert!(edit.text.contains("// a triangle, and this comment must survive every drag"));
    assert!(edit.text.contains("line ab(a, b)      // the base"));
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
component Ring(rad: Length) {
  point o hint(x: 0, y: 0)
  circle c(center: o) hint(r: rad)
  radius(c) == rad
  ground(o)
}
g: Ring(rad: 25)
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
point o hint(x: 0, y: 0)
ground(o)
cycle 4 as i {
  point p hint(x: 10, y: 0)
}
";
    let prog = prog_of(src);
    let mut e = elaborate(&prog);
    assert!(e.ok(), "{:?}", e.errors().map(|d| &d.message).collect::<Vec<_>>());
    assert_eq!(e.sketch.points.len(), 5, "one centre and four copies");
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
    assert!(e.text.contains("point   p0 hint(x: 12.5, y: -3)"), "{}", e.text);
    assert!(e.text.trim_end().ends_with("// and this trailing note, too"), "{}", e.text);
    let back = elaborate(&prog_of(&e.text));
    assert!(back.ok());
    assert_eq!(back.sketch.points.len(), 4);
}

/// A minted name never collides with one already written, whatever it is.
#[test]
fn a_minted_name_is_free() {
    let prog = prog_of("point p0 hint(x: 0, y: 0)\npoint p1 hint(x: 1, y: 0)\npoint p3 hint(x: 3, y: 0)\n");
    assert_eq!(edit::mint(&prog, EntKind::Point), "p2", "the first gap, not the next number");
    assert_eq!(edit::mint(&prog, EntKind::Line), "l0");
}

/// Deleting takes out the declaration and every statement that named it — the same rule
/// `io::without` follows on a sketch, said about text.
#[test]
fn deleting_a_point_takes_what_named_it() {
    let prog = prog_of(DOC);
    let e = elaborate(&prog);
    let d = edit::remove(&e, &prog, &[EntRef::point(2)], &[]);
    assert_eq!(d.kind, Kind::Structural);
    // `c` and the two lines that named it are gone; the base and its dimension are not
    assert!(!d.text.contains("point c at"), "{}", d.text);
    assert!(!d.text.contains("line bc"), "{}", d.text);
    assert!(!d.text.contains("line ca"), "{}", d.text);
    assert!(d.text.contains("line ab(a, b)      // the base"), "{}", d.text);
    assert!(d.text.contains("distance(a, b) == w = 140"), "{}", d.text);
    assert!(d.text.contains("// a triangle"), "the comments stay");
    let back = elaborate(&prog_of(&d.text));
    assert!(back.ok(), "{:?}", back.errors().map(|x| &x.message).collect::<Vec<_>>());
    assert_eq!(back.sketch.points.len(), 2);
    assert_eq!(back.sketch.lines.len(), 1);
}

/// Deleting something a component made is refused, and says why: the statement makes N of them,
/// and taking the component out is a far larger edit than the gesture asked for.
#[test]
fn deleting_what_a_component_made_is_refused() {
    let src = "\
component Pair() {
  point a hint(x: 0, y: 0)
  point b hint(x: 10, y: 0)
}
q: Pair()
ground(q.a)
";
    let prog = prog_of(src);
    let e = elaborate(&prog);
    assert!(e.ok(), "{:?}", e.errors().map(|d| &d.message).collect::<Vec<_>>());
    let d = edit::remove(&e, &prog, &[EntRef::point(0)], &[]);
    assert_eq!(d.kind, Kind::None);
    assert!(d.refused.is_some_and(|r| r.contains("component")), "it says why");
    assert_eq!(d.text, src, "and changes nothing");
}

/// Editing a number splices the number.  A plain one is `Numeric` — the topology cannot have
/// moved, so a compiled plan survives it — and one that names anything is not, because a name
/// nothing defines is a free variable and that is a column.
#[test]
fn editing_a_dimension_splices_the_number() {
    let prog = prog_of(DOC);
    let e = elaborate(&prog);
    let cid = e
        .sketch
        .user_constraints()
        .iter()
        .find(|c| c.kind == gcs_core::constraints::CKind::Distance)
        .unwrap()
        .id;

    let plain = edit::set_dimension(&e, &prog, cid, "d", "140");
    assert!(plain.text.contains("distance(a, b) == 140"), "{}", plain.text);
    assert!(plain.text.contains("// the base"), "everything else is untouched");

    let named = edit::set_dimension(&e, &prog, cid, "d", "w = 140");
    assert_eq!(named.kind, Kind::Structural, "a name may be a free variable, and that is a column");
    let back = elaborate(&prog_of(&named.text));
    assert!(back.ok());
}

/// Nothing here panics on a span that does not name a place in the text it is given, which is
/// what happens the moment two edits are computed against one program and applied in turn.
#[test]
fn a_stale_span_edits_nothing() {
    let prog = prog_of(DOC);
    let e = elaborate(&prog);
    let short = prog_of("point a hint(x: 0, y: 0)\n");
    // spans from the long document, applied to the short one
    let d = edit::remove(&e, &short, &[EntRef::point(2)], &[]);
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
    assert!(edit.text.contains("curve involute(c: circle, phase: Angle)(u) ="));
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
    assert!(prog.text().contains("line    l0(p0, p1)"), "{}", prog.text());
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
    let l = e.sketch.line(p, q);
    e.sketch.add(Constraint::one_line(CKind::Horizontal, EntRef::line(l)));

    let edit = reconciled(&mut e);
    assert_eq!(edit.kind, Kind::Structural);
    assert_eq!(edit.names, vec!["p0", "p1", "l0"], "each new thing was named, the old kept");
    // everything somebody wrote is still there, character for character
    for line in DOC.lines().filter(|l| !l.trim().is_empty()) {
        assert!(edit.text.contains(line), "lost: {line}\n{}", edit.text);
    }
    assert!(edit.text.contains("line    l0(p0, p1)"), "{}", edit.text);
    assert!(edit.text.contains("horizontal(l0)"), "{}", edit.text);

    let back = elaborate(&prog_of(&edit.text));
    assert!(back.ok(), "{:?}", back.errors().map(|d| &d.message).collect::<Vec<_>>());
    assert_eq!(back.sketch.points.len(), n + 2);
    assert_eq!(back.sketch.lines.len(), 4);
    assert_eq!(
        gcs_core::io::dumps(&back.sketch, Some(1)),
        gcs_core::io::dumps(&e.sketch, Some(1)),
        "the source now says exactly what the gesture made",
    );
}

/// A constraint added to the drawing gets a statement; one taken off it loses one.
#[test]
fn a_constraint_added_and_one_removed_are_both_splices() {
    let prog = prog_of(DOC);
    let mut e = elaborate(&prog);
    e.sketch.add(Constraint::one_line(CKind::Vertical, EntRef::line(1)));
    let edit = reconciled(&mut e);
    assert!(edit.text.contains("vertical(bc)"), "{}", edit.text);
    assert!(edit.text.contains("horizontal(ab)"), "and the one that was there stays");

    let prog2 = prog_of(&edit.text);
    let mut e2 = elaborate(&prog2);
    let id = e2.sketch.user_constraints().last().unwrap().id;
    e2.sketch.remove(id);
    let back = reconciled(&mut e2);
    assert!(!back.text.contains("vertical(bc)"), "{}", back.text);
    assert!(back.text.contains("horizontal(ab)"), "{}", back.text);
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
    assert!(edit.text.contains("curve involute(c: circle, phase: Angle)(u) ="));
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
    let d = edit::remove(&e, &e.program, &[EntRef::point(before)], &[]);
    assert!(!d.text.contains("point   p0 at"), "{}", d.text);
    assert!(!d.text.contains("line    l0("), "{}", d.text);
    assert!(d.text.contains("// a triangle, and this comment must survive every drag"));
}

/// A gauge a component wrote is the component's, not the drawing's: `ground(center)` inside
/// `Gear` says the same thing about `g.center` as a top-level `ground(g.center)` would, and adding
/// the second is a statement the document did not need and nobody asked for.
#[test]
fn a_gauge_a_component_wrote_is_not_repeated() {
    let prog = prog_of(gcs_core::examples::GEAR);
    let mut e = elaborate(&prog);
    assert!(e.ok());
    assert!(e.sketch.point_fixed(0), "the gear grounds its own centre");
    let edit = reconciled(&mut e);
    assert!(!edit.text.contains("ground(g.center)"), "{}", &edit.text[edit.text.len() - 300..]);
    assert_eq!(edit.text, gcs_core::examples::GEAR, "nothing to say, and nothing said");
}

/// A construction flag and a gauge are neither an entity nor a constraint, so nothing about the
/// two counts notices them: they are read off the drawing and compared with what the source says.
#[test]
fn a_flag_and_a_gauge_are_spliced_too() {
    let prog = prog_of(DOC);
    let mut e = elaborate(&prog);
    assert!(e.ok());
    e.sketch.lines[1].construction = true;
    let a = e.map.ent_named("a").unwrap();
    let c = e.map.ent_named("c").unwrap();
    e.sketch.fix_point(a.i(), false);
    e.sketch.fix_point(c.i(), true);

    let edit = reconciled(&mut e);
    assert!(edit.text.contains("line bc(b, c) construction"), "{}", edit.text);
    assert!(edit.text.contains("line ab(a, b)      // the base"), "the comment stayed");
    assert!(edit.text.contains("ground(c)"), "{}", edit.text);
    assert!(!edit.text.contains("ground(a)"), "the one that was let go is gone:\n{}", edit.text);

    let back = elaborate(&prog_of(&edit.text));
    assert!(back.ok(), "{:?}", back.errors().map(|d| &d.message).collect::<Vec<_>>());
    assert!(back.sketch.lines[1].construction);
    assert!(back.sketch.point_fixed(2) && !back.sketch.point_fixed(0));

    // and taking the flag off again takes the word out, leaving the line as it was
    e.sketch.lines[1].construction = false;
    let off = reconciled(&mut e);
    assert!(off.text.contains("line bc(b, c)\n"), "{}", off.text);
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
    assert!(second.text.contains("point   p0 hint(x: -95, y: 48)"), "{}", second.text);
    assert!(second.text.contains("point   p1 hint(x: -40, y: 60)"), "{}", second.text);
    assert!(second.text.contains("line    l0(p0, p1)"), "{}", second.text);
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
fn a_dragged_callout_is_written_down() {
    let src = "point a hint(x: 0, y: 0)\npoint b hint(x: 60, y: 0)\ndistance(a, b) == 60\n";
    let mut e = elaborate(&prog_of(src));
    let id = e.sketch.user_constraints()[0].id;

    // dragged where the layout would not have put it
    e.sketch.placements.insert(id, (12.0, -4.0));
    let out = reconciled(&mut e);
    assert!(out.text.contains("distance(a, b) == 60 at (12, -4)"), "{}", out.text);
    assert!(out.text.contains("point a hint(x: 0, y: 0)"), "the rest of the file is untouched");

    // dragged again: the two numbers are rewritten where they stand, not appended beside
    e.sketch.placements.insert(id, (20.0, 8.0));
    let out = reconciled(&mut e);
    assert!(out.text.contains("distance(a, b) == 60 at (20, 8)"), "{}", out.text);
    // `at (…)` is now the callout's alone: every seed in the language is in a `hint(…)` clause,
    // and a placement is the one inert number that is not a seed (spec §6.4)
    assert_eq!(out.text.matches(" at (").count(), 1, "the callout's, and nothing else's");

    // and put back where the layout would place it, the clause goes with its space
    e.sketch.placements.remove(&id);
    let out = reconciled(&mut e);
    assert!(out.text.contains("distance(a, b) == 60\n"), "{}", out.text);
}

/// The same, for a placement on a relation that states no number — the clause stands alone
/// there, so it is written and removed on its own rather than after a `==`.
#[test]
fn a_placement_without_a_dimension_round_trips() {
    let src = "point a hint(x: 0, y: 0)\npoint b hint(x: 60, y: 0)\nline l(a, b)\nhorizontal(l) at (3, 5)\n";
    let mut e = elaborate(&prog_of(src));
    let id = e.sketch.user_constraints().iter().find(|c| c.kind == CKind::Horizontal).unwrap().id;
    assert_eq!(e.sketch.placements.get(&id).copied(), Some((3.0, 5.0)), "read as written");

    e.sketch.placements.insert(id, (9.0, 1.0));
    let out = reconciled(&mut e);
    assert!(out.text.contains("horizontal(l) at (9, 1)"), "{}", out.text);

    e.sketch.placements.remove(&id);
    let out = reconciled(&mut e);
    assert!(out.text.contains("horizontal(l)\n"), "{}", out.text);
}

/// **A seed the document never wrote is recorded where the clause would have gone.**
///
/// A radius is a seed now — `circle c(center: o) hint(r: 25)` — so it is one a person may
/// perfectly well never write, and a solve still moves it.  There is then no span to splice, and
/// leaving it alone would mean a drawing whose pose the source cannot express.  So the clause is
/// written out whole, at the point the parser recorded for it, and the statement around it is
/// untouched — one splice, and every comment where it was.
#[test]
fn a_seed_the_source_never_wrote_is_appended() {
    let src = "point o hint(x: 0, y: 0)\ncircle c(center: o)   // a hole, and this comment stays\n";
    let mut e = elaborate(&prog_of(src));
    let mut sk = std::mem::take(&mut e.sketch);
    let r = sk.circles[0].radius as usize;
    sk.params[r].value = 12.5;
    let out = edit::commit_seeds(&e, &sk, &e.program);
    assert_eq!(out.kind, Kind::Numeric, "a seed is not a statement, wherever it is written");
    assert!(
        out.text.contains("circle c(center: o) hint(r: 12.5)   // a hole"),
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
    let src = "point a hint(x: 0, y: 0)\npoint b hint(x: 10, y: 0)\nline l(a, b)\nground(a)\n";
    let mut e = elaborate(&prog_of(src));
    let mut sk = std::mem::take(&mut e.sketch);
    let bx = sk.points[1].x as usize;
    sk.params[bx].value = 42.5;     // drag `b` sideways
    let out = edit::commit_seeds(&e, &sk, &e.program);
    assert_eq!(out.kind, Kind::Numeric);
    assert!(out.text.contains("point b hint(x: 42.5, y: 0)"), "{}", out.text);
    assert!(out.text.contains("point a hint(x: 0, y: 0)"), "and nothing else moved: {}", out.text);
}
