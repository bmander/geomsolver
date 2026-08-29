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

/// Deletion never needed the name: the statement goes by its site.
#[test]
fn deleting_an_anonymous_element_takes_its_statement() {
    let e = read("line\npoint a hint(x: 1, y: 2)\n");
    let d = edit::remove(&e, &e.program, &[EntRef::line(0)], &[]);
    assert_eq!(d.text, "point a hint(x: 1, y: 2)\n");
}

/// An anonymous declaration inside a component is one statement and many elements, like any
/// other — the instances do not collide.
#[test]
fn an_anonymous_declaration_in_a_component_instances_cleanly() {
    let e = read("component Strut() {\n  line\n}\ns1: Strut()\ns2: Strut()\n");
    assert_eq!((e.sketch.points.len(), e.sketch.lines.len()), (4, 2));
}
