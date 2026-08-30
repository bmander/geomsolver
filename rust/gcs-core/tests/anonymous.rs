//! Anonymous elements: the name in a declaration is optional (issue #33).
//!
//! `line` alone is a valid statement — a line with no name, implicit children and no hint — and
//! the name is optional *independently* of the rest, so `line(p1, p2)`, `line hint(x: 0, y: 0)`
//! and `arc(center: c)` are all valid anonymous forms.  Internally the statement id suffices: an
//! anonymous element parses, elaborates, draws and deletes without ever being named.  The moment
//! the *source* must reference it — a constraint applied from the app, a gauge on a fixed point —
//! a real name is minted and spliced into the declaration, the same bargain `commit_seeds`
//! strikes with an unwritten `hint(…)` clause.  No hidden names in the *source*: an unnamed thing
//! has none until the source needs one.

use gcs_core::constraints::{CKind, Constraint};
use gcs_core::edit::{self, Kind};
use gcs_core::model::EntRef;
use gcs_core::program::{elaborate, Elaborated};
use gcs_core::syntax::parse;

fn read(src: &str) -> Elaborated {
    let (prog, errs) = parse(src);
    assert!(errs.is_empty(), "does not parse: {errs:?}");
    let e = elaborate(&prog);
    assert!(
        e.ok(),
        "does not elaborate: {:?}",
        e.errors().map(|d| d.message.clone()).collect::<Vec<_>>()
    );
    e
}

fn refuses(src: &str, needle: &str) {
    let (_, errs) = parse(src);
    let msgs: Vec<String> = errs.into_iter().map(|e| e.message).collect();
    assert!(msgs.iter().any(|m| m.contains(needle)), "expected `{needle}`\n{src}\n{msgs:?}");
}

/// `reconcile` applies itself to the elaboration, so a test that wants both the text and the
/// elaboration afterwards hands it the whole thing — `tests/edit.rs`'s device.
fn reconciled(e: &mut Elaborated) -> edit::Edit {
    let sk = std::mem::take(&mut e.sketch);
    let out = edit::reconcile(e, &sk);
    e.sketch = sk;
    out
}

/// The name is optional independently of the children, the clauses and the class — each written
/// form elaborates to exactly what its named twin does.
#[test]
fn every_anonymous_form_declares() {
    let e = read("line\n");
    assert_eq!((e.sketch.points.len(), e.sketch.lines.len()), (2, 1));

    let e = read("point hint(x: 3, y: 4)\n");
    assert_eq!(e.sketch.points.len(), 1);
    assert_eq!(e.sketch.point_xy(0), (3.0, 4.0), "the clause still seeds it");

    let e = read("point o hint(x: 0, y: 0)\ncircle(center: o) hint(r: 25)\n");
    assert_eq!(e.sketch.circles.len(), 1);
    let rp = e.sketch.circles[0].radius as usize;
    assert_eq!(e.sketch.params[rp].value, 25.0);

    // the children list without a name, over names declared further down
    let e = read("line(p1, p2)\npoint p1 hint(x: 0, y: 0)\npoint p2 hint(x: 8, y: 0)\n");
    assert_eq!((e.sketch.points.len(), e.sketch.lines.len()), (2, 1));

    // and a class, which is a trailing-clause word and so can no longer be a name
    let e = read("line class construction\n");
    assert!(e.sketch.class_of(EntRef::line(0)).0.contains(&"construction".to_string()));
}

/// Two anonymous declarations are two elements, not one name declared twice.
#[test]
fn two_anonymous_declarations_are_not_a_duplicate() {
    let e = read("line\nline\n");
    assert_eq!((e.sketch.points.len(), e.sketch.lines.len()), (4, 2));
}

/// The payoff (issue #33): a fully anonymous open contour.  The corner-minting rule names shared
/// points by the parent's dotted path, so a threaded corner between two anonymous links welds
/// exactly as a named chain's does — and each joint is the regular At-form, never the bare pair.
#[test]
fn a_fully_anonymous_chain_threads_its_corners() {
    let e = read("line -> tangent arc -> tangent line\n");
    assert_eq!(
        (e.sketch.points.len(), e.sketch.lines.len(), e.sketch.arcs.len()),
        (5, 2, 1),
        "three links, two shared corners"
    );
    let l0 = e.sketch.children(EntRef::line(0));
    let l1 = e.sketch.children(EntRef::line(1));
    let a = e.sketch.children(EntRef::arc(0));
    assert_eq!(a[1], l0[1], "the arc starts where the first line ends");
    assert_eq!(a[2], l1[0], "and ends where the second begins");
    let kinds: Vec<CKind> = e.sketch.user_constraints().iter().map(|c| c.kind).collect();
    assert_eq!(kinds, vec![CKind::TangentArcLine; 2], "the At-form, at each threaded corner");
}

/// The retired seed spellings still say where the number belongs, without inventing a name to
/// say it with.
#[test]
fn the_retired_spelling_errors_without_a_name() {
    refuses("point at (0, 0)\n", "a coordinate seed is keyed now: `point hint(x: …, y: …)`");
}

/// `curve` keeps requiring a name: its form is `curve name = family(…)`, and the name is what
/// the contact constraints address.
#[test]
fn a_curve_still_requires_a_name() {
    refuses("curve = involute(c)\n", "expected a name");
}

/// A solve writes an anonymous element's pose back the way it writes `line l`'s: the argument
/// list and the `hint(…)` clause the source never wrote, spliced after the keyword, and the
/// statement is still anonymous afterwards.
#[test]
fn an_anonymous_line_writes_its_pose_back() {
    let mut e = read("line\n");
    for i in 0..2 {
        let [x, y] = e.sketch.point_params(i);
        e.sketch.params[x as usize].value = 10.0 * (i + 1) as f64;
        e.sketch.params[y as usize].value = 5.0;
    }
    let edit = edit::commit_seeds(&e, &e.sketch, &e.program);
    assert_eq!(edit.kind, Kind::Numeric, "a seed is not a statement");
    assert_eq!(
        edit.text, "line(hint(x: 10, y: 5), hint(x: 20, y: 5))\n",
        "the pose, and no name"
    );
    let back = read(&edit.text);
    assert_eq!(back.sketch.point_xy(0), (10.0, 5.0));
    assert_eq!(back.sketch.point_xy(1), (20.0, 5.0));
}

/// **Identity is minted on demand.**  A constraint applied from the drawing must be written over
/// a name, so reconciling it splices one into the anonymous declaration — and only there: an
/// anonymous element nothing references stays unnamed.
#[test]
fn a_constraint_from_the_drawing_mints_a_name() {
    let mut e = read("line\npoint hint(x: 40, y: 40)\n");
    e.sketch.add(Constraint::one_line(CKind::Horizontal, EntRef::line(0)));
    let edit = reconciled(&mut e);
    assert_eq!(edit.kind, Kind::Structural);
    // the declaration gained its name — and, reconcile committing the seeds too, its pose
    assert!(edit.text.contains("line l0("), "the declaration gained its name:\n{}", edit.text);
    assert!(edit.text.contains("horizontal l0"), "{}", edit.text);
    assert!(edit.text.contains("point hint(x: 40, y: 40)"), "the unreferenced point stays");
    let back = read(&edit.text);
    assert_eq!(back.sketch.user_constraints().len(), 1);
    // and the elaboration that spliced it can keep editing: a second pass has nothing to say
    let again = reconciled(&mut e);
    assert_eq!(again.kind, Kind::None, "{}", again.text);
}

/// A dimension names the *child* of an anonymous element by its dotted path, so the mint renames
/// the parent once and the path follows it.
#[test]
fn a_dimension_on_anonymous_endpoints_mints_the_parent() {
    let mut e = read("line\n");
    let kids = e.sketch.children(EntRef::line(0));
    e.sketch.add(Constraint::distance(kids[0], kids[1], 80.0));
    let edit = reconciled(&mut e);
    assert!(edit.text.contains("line l0("), "{}", edit.text);
    assert!(edit.text.contains("l0.p1 distance(80) l0.p2"), "{}", edit.text);
    let back = read(&edit.text);
    assert_eq!(back.sketch.user_constraints().len(), 1);
}

/// A slot the source named beside a slot it left empty.  The minted child's dotted path is its
/// **position** among the parent's children — the same walk that named it in the first place —
/// so it comes out as the slot it actually sits in (`a0.end`), and the named ones are untouched.
/// Recovering the path by slicing the key it elaborated under happened to agree here and could
/// not say why: the key's own half is an offset, and only the marker it is spelled with told the
/// two halves apart.
#[test]
fn a_mint_names_a_child_by_the_slot_it_sits_in() {
    let src = "point c hint(x: 0, y: 0)\npoint s hint(x: 10, y: 0)\narc(center: c, start: s)\n";
    let mut e = read(src);
    let kids = e.sketch.children(EntRef::arc(0));
    assert_eq!(kids[0], EntRef::point(0), "the named slots are the points that were declared");
    e.sketch.add(Constraint::distance(EntRef::point(0), kids[2], 80.0));
    let edit = reconciled(&mut e);
    assert!(edit.text.contains("arc a0("), "{}", edit.text);
    assert!(edit.text.contains("c distance(80) a0.end"), "{}", edit.text);
    let back = read(&edit.text);
    assert_eq!(back.sketch.user_constraints().len(), 1);
    assert_eq!(back.sketch.points.len(), 3, "the named endpoints are not minted again");
}

/// The mint reaches into a chain: naming one link splices its name where it would stand, the
/// seeds are recorded where each link owns them, and a corner the thread filled with a name the
/// source cannot write is left an empty slot for the marker to thread again — never written out.
#[test]
fn a_mint_names_one_link_and_leaves_the_chain_threaded() {
    let mut e = read("line -> tangent arc -> tangent line\n");
    e.sketch.add(Constraint::one_line(CKind::Horizontal, EntRef::line(0)));
    let edit = reconciled(&mut e);
    assert!(edit.text.contains("line l0("), "{}", edit.text);
    assert!(edit.text.contains("horizontal l0"), "{}", edit.text);
    assert!(!edit.text.contains('#'), "no hidden name reaches the source:\n{}", edit.text);
    let back = read(&edit.text);
    assert_eq!(back.sketch.user_constraints().len(), 3, "two tangencies and the new relation");
    assert_eq!(back.sketch.points.len(), 5, "the corners are still shared");
}

/// A gauge names what it holds, so fixing an anonymous point names it too.
#[test]
fn a_fixed_anonymous_point_is_named_for_its_gauge() {
    let mut e = read("point hint(x: 3, y: 4)\n");
    e.sketch.fix_point(0, true);
    let edit = reconciled(&mut e);
    assert!(edit.text.contains("point p0 hint(x: 3, y: 4)"), "{}", edit.text);
    assert!(edit.text.contains("ground p0"), "{}", edit.text);
    let back = read(&edit.text);
    assert!(back.sketch.point_fixed(0));
}

/// A chain of *lines* writes its pose back through the same elision an arc's does — and a line's
/// children print positionally, so the slot the thread will fill again forces labels on, or the
/// kept end would count into the wrong slot on the next parse and quietly reseed.
#[test]
fn an_anonymous_line_chain_survives_a_reload() {
    let mut e = read("line -> line\n");
    assert_eq!(e.sketch.points.len(), 3, "one corner shared");
    let poses = [(0.0, 0.0), (20.0, 10.0), (30.0, 102.0)];
    for (i, (x, y)) in poses.iter().enumerate() {
        let [px, py] = e.sketch.point_params(i);
        e.sketch.params[px as usize].value = *x;
        e.sketch.params[py as usize].value = *y;
    }
    let edit = edit::commit_seeds(&e, &e.sketch, &e.program);
    assert!(!edit.text.contains('#'), "no hidden name reaches the source:\n{}", edit.text);
    assert!(edit.text.contains("p2: hint(x: 30, y: 102)"), "the kept slot is labelled:\n{}", edit.text);
    let back = read(&edit.text);
    assert_eq!(back.sketch.points.len(), 3, "the corner is still shared");
    assert_eq!(
        back.sketch.children(EntRef::line(1))[0],
        back.sketch.children(EntRef::line(0))[1],
        "and it is still the weld"
    );
    for (i, (x, y)) in poses.iter().enumerate() {
        assert_eq!(back.sketch.point_xy(i), (*x, *y), "point {i} kept its pose");
    }
}

/// The app never re-elaborates after a reconcile, so the *same* elaboration must take a second
/// gesture: the mint renames every entity its statement made, first in the map, and the next
/// statement writes the name and not the key.
#[test]
fn a_second_gesture_after_a_mint_still_writes() {
    let mut e = read("line\n");
    e.sketch.add(Constraint::one_line(CKind::Horizontal, EntRef::line(0)));
    let first = reconciled(&mut e);
    assert!(first.refused.is_none(), "{:?}", first.refused);
    let kids = e.sketch.children(EntRef::line(0));
    e.sketch.add(Constraint::distance(kids[0], kids[1], 80.0));
    let second = reconciled(&mut e);
    assert!(second.refused.is_none(), "{:?}", second.refused);
    assert!(second.text.contains("l0.p1 distance(80) l0.p2"), "{}", second.text);
    assert!(!second.text.contains('#'), "{}", second.text);
    let back = read(&second.text);
    assert_eq!(back.sketch.user_constraints().len(), 2);
}

/// A numeric edit above the declaration moves its offset, so the map's hidden keys go stale —
/// the rename follows the dotted path, which is the stable half, and never compares offsets.
#[test]
fn a_mint_survives_offsets_an_earlier_edit_moved() {
    let mut e = read("point q hint(x: 1, y: 2)\nline\n");
    let [px, py] = e.sketch.point_params(0);
    e.sketch.params[px as usize].value = 17.25;
    e.sketch.params[py as usize].value = -3.5;
    let seeds = edit::commit_seeds(&e, &e.sketch, &e.program);
    assert_eq!(seeds.kind, Kind::Numeric);
    assert!(e.retext(&seeds.text), "a numeric edit retexts in place");
    let kids = e.sketch.children(EntRef::line(0));
    e.sketch.add(Constraint::distance(kids[0], kids[1], 80.0));
    let edit = reconciled(&mut e);
    assert!(edit.refused.is_none(), "{:?}", edit.refused);
    assert!(edit.text.contains("l0.p1 distance(80) l0.p2"), "{}", edit.text);
    let back = read(&edit.text);
    assert_eq!(back.sketch.user_constraints().len(), 1);
}

/// An element a component made anonymously has no statement of the root's to put a name on, so
/// the gesture is refused **with the cause** — never by writing the hidden key and failing to
/// parse it back.
#[test]
fn a_gesture_on_a_component_made_anonymous_element_says_why_it_is_refused() {
    let mut e = read("component Strut() {\n  line\n}\ns1: Strut()\n");
    e.sketch.add(Constraint::one_line(CKind::Horizontal, EntRef::line(0)));
    let edit = reconciled(&mut e);
    assert_eq!(edit.kind, Kind::None);
    let why = edit.refused.expect("refused, and says why");
    assert!(why.contains("name"), "{why}");
}

/// **What the source calls a thing and what a statement can be written with are two questions**,
/// and a block prefix is what separates them: `#3.0.p` is published and selected by — it says
/// which copy, which an index cannot — and it carries a `#` no tokenizer will give back.  So a
/// gesture reaching for one copy of a block is refused with the cause, where it used to write
/// the prefix out and come back with `adopt`'s generic "could not be written down".
#[test]
fn a_gesture_on_one_copy_of_a_block_says_why_it_is_refused() {
    let mut e = read("point q hint(x: 5, y: 5)\ncycle 2 {\n  point p hint(x: 0, y: 0)\n}\n");
    assert_eq!(e.map.name_of(EntRef::point(1)).map(String::as_str), Some("#3.0.p"));
    e.sketch.add(Constraint::distance(EntRef::point(0), EntRef::point(1), 80.0));
    let edit = reconciled(&mut e);
    assert_eq!(edit.kind, Kind::None);
    let why = edit.refused.expect("refused, and says why");
    assert!(why.contains("block"), "{why}");
    assert!(!why.contains("could not be written down"), "{why}");
    assert!(!edit.text.contains('#'), "and nothing was written:\n{}", edit.text);
}

/// A reserved word written where a name would go is very possibly a name somebody meant, so a
/// line that then fails to parse says the reservation — beside the failure, never on a line
/// that parses (`line tangent arc` is a chain).
#[test]
fn a_reserved_word_meant_as_a_name_is_said_so() {
    refuses("point tangent hint(x: 0, y: 0)\n", "cannot be a declaration's name");
    refuses("point close hint(x: 0, y: 0)\n", "cannot be a declaration's name");
    let (_, errs) = parse("line -> tangent arc -> tangent line\n");
    assert!(errs.is_empty(), "a chain that parses gets no note: {errs:?}");
}

/// Deletion never needed the name: the statement goes by its site.
#[test]
fn deleting_an_anonymous_element_takes_its_statement() {
    let e = read("line\npoint a hint(x: 1, y: 2)\n");
    let d = edit::remove(&e, &e.program, &[EntRef::line(0)], &[]);
    assert_eq!(d.text, "point a hint(x: 1, y: 2)\n");
}

/// **Names in the sketch are display; names in the map are identity.**  A parameter carries its
/// entity's name into every place a parameter is listed (a DOF report, a mode's label), so an
/// anonymous element is called what the *drawing* calls it — never by the offset key it resolves
/// by, which is not a thing to show anybody.
#[test]
fn the_sketch_shows_a_name_and_never_the_key() {
    let e = read("point a hint(x: 1, y: 2)\nline\ncircle\n");
    for p in &e.sketch.params {
        assert!(!p.name.contains('#'), "a key reached a parameter: {}", p.name);
    }
    let names: Vec<&str> = e.sketch.params.iter().map(|p| p.name.as_str()).collect();
    assert!(names.contains(&"a.x"), "a written name is unchanged: {names:?}");
    assert!(names.contains(&"l0.p1.x"), "and an anonymous one reads positionally: {names:?}");
    assert!(names.contains(&"c0.r"), "{names:?}");
    // the map keeps the key all the same — that is what a chain's corner welds by — but files
    // it under *resolution* and never as a name, so nothing that reads what an entity is called
    // has to screen it out (issue #39)
    let key = format!("#a{}", "point a hint(x: 1, y: 2)\nline".len());
    assert_eq!(e.map.ent_named(&key), Some(EntRef::line(0)), "the key still resolves");
    assert_eq!(e.map.name_of(EntRef::line(0)), None, "and the source calls it nothing");
    assert!(e.map.names.values().flatten().all(|n| !n.starts_with("#a")), "a key was filed");

    // and a *block prefix* is not an anonymous name: the flattener wrote it, it says which
    // instance the thing belongs to, and it is shown as it always has been
    let e = read("point o hint(x: 0, y: 0)\nground o\ncycle 3 as i {\n  point p hint(x: 1, y: 0)\n}\n");
    let names: Vec<&str> = e.sketch.params.iter().map(|p| p.name.as_str()).collect();
    assert!(names.iter().any(|n| n.ends_with(".0.p.x")), "the instance path stands: {names:?}");
}

/// A diagnostic about an anonymous declaration spells the kind, never the hidden key — a `#`
/// and an offset are the elaboration's, and nothing a person wrote or can search for.
#[test]
fn a_trace_block_diagnostic_never_says_the_hidden_key() {
    let src = "\
point o hint(x: 0, y: 0)
circle c(center: o) hint(r: 10)
curve fam(c: circle)(u) = trace p where {
  point p hint(x: 1, y: 0)
  line
}
";
    let (prog, errs) = parse(src);
    assert!(errs.is_empty(), "{errs:?}");
    let e = elaborate(&prog);
    let msgs: Vec<&String> = e.diags.iter().map(|d| &d.message).collect();
    for m in &msgs {
        assert!(!m.contains('#'), "a hidden key leaked: {m}");
    }
    assert!(msgs.iter().any(|m| m.contains("needs its points named")), "{msgs:?}");
}

/// An anonymous declaration inside a component is one statement and many elements, like any
/// other — the instances do not collide.
#[test]
fn an_anonymous_declaration_in_a_component_instances_cleanly() {
    let e = read("component Strut() {\n  line\n}\ns1: Strut()\ns2: Strut()\n");
    assert_eq!((e.sketch.points.len(), e.sketch.lines.len()), (4, 2));
}
