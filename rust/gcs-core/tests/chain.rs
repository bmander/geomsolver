//! Chains: the prefix and joint sugar (spec §6.6).
//!
//! A chain is a parser addition rather than a change of shape — `horizontal line bottom(b1, b2)
//! tangent arc a(center: c, r: 5) …` desugars into the declarations and relations it is sugar
//! for, each with its own id and a span into the chain's own text.  What is worth testing is
//! therefore the desugaring itself: that a contour written as a chain states *exactly* what the
//! longhand states (held against the shipped `rect_fillets` case), that threading fills the
//! boundary points one side named and refuses what neither did, that every joint is the regular
//! At-form and never the bare pair, and that the edits a chain makes awkward — removing one word
//! of somebody else's line — splice rather than break.

use gcs_core::constraints::CKind;
use gcs_core::diagnose::{diagnose, DiagnoseOptions, State};
use gcs_core::edit;
use gcs_core::examples;
use gcs_core::model::EntRef;
use gcs_core::program::{elaborate, Elaborated};
use gcs_core::syntax::{highlight, parse, Chained, Tint};

/// The shipped fillet case, written as one chain.  Same points, same numbers, same order of
/// elements — only the four arc declarations lose their endpoints (the chain threads them) and
/// the four levels and eight tangencies move into the chain's words.
const RECT_FILLETS_CHAIN: &str = "\
param w = 100
param h = 60
param r = 10

point b1 at (r, 0)
point b2 at (w - r, 0)
point r1 at (w, r)
point r2 at (w, h - r)
point t1 at (w - r, h)
point t2 at (r, h)
point l1 at (0, h - r)
point l2 at (0, r)

point c_br at (w - r, r)
point c_tr at (w - r, h - r)
point c_tl at (r, h - r)
point c_bl at (r, r)

horizontal line bottom(b1, b2) tangent
arc a_br(center: c_br, r: r) tangent
vertical line right(r1, r2) tangent
arc a_tr(center: c_tr, r: r) tangent
horizontal line top(t1, t2) tangent
arc a_tl(center: c_tl, r: r) tangent
vertical line left(l1, l2) tangent
arc a_bl(center: c_bl, r: r) tangent close

equal_radius(a_br, a_tr)
equal_radius(a_br, a_tl)
equal_radius(a_br, a_bl)
radius(a_bl) == r

distance(b1, b2) == w - 2 * r
distance(l1, l2) == h - 2 * r

ground(c_bl)
";

/// Five points and a centre, the cast every small chain below is drawn from.
const PTS: &str = "point p1 at (0, 0)\npoint p2 at (10, 0)\npoint p3 at (20, 10)\n\
                   point p4 at (20, 30)\npoint c at (10, 10)\n";

/// One line, one arc, one line, tangent at both joints — the smallest chain with a threaded
/// element in the middle, and what three of the edit tests below are written against.
const TANGENT_RUN: &str =
    "line a(p1, p2) tangent arc k(center: c, r: 10) tangent line b(p3, p4)\n";

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

/// Every error the parser had, for the documents that are supposed to have one.
fn errors(src: &str) -> Vec<String> {
    let (_, errs) = parse(src);
    errs.into_iter().map(|e| e.message).collect()
}

/// A document the parser must refuse, and the words its complaint must carry.  One parse, and
/// the failure prints the document that produced it.
fn refuses(src: &str, needle: &str) {
    let msgs = errors(src);
    assert!(msgs.iter().any(|m| m.contains(needle)), "expected `{needle}`\n{src}\n{msgs:?}");
}

/// What a sketch's constraints say, as `io::describe` writes them — which leaves out the hidden
/// unknowns, since a `Param` seed is not something a constraint states.  Statement order is the
/// one thing a chain legitimately changes, so equivalence is a sorted comparison.
fn said(sk: &gcs_core::model::Sketch) -> Vec<String> {
    let mut v: Vec<String> =
        sk.user_constraints().iter().map(|c| gcs_core::io::describe(c)).collect();
    v.sort();
    v
}

/// **The gate.**  The chain says exactly what the longhand case says: same entities at the same
/// indices, the same constraints with the same arguments, and the same drawing once solved.
#[test]
fn the_chain_states_the_longhand() {
    let e = read(RECT_FILLETS_CHAIN);
    let long = examples::case("rect_fillets").expect("the shipped case");
    let mut sk = e.sketch;
    assert_eq!(
        (sk.points.len(), sk.lines.len(), sk.arcs.len()),
        (long.points.len(), long.lines.len(), long.arcs.len()),
        "the chain draws something else"
    );
    assert_eq!(said(&sk), said(&long), "the chain states something else");
    let _ = gcs_core::solve::solve(&mut sk, gcs_core::solve::SolveOpts::default());
    let d = diagnose(&mut sk, DiagnoseOptions::default());
    assert_eq!((d.dof, d.status), (0, State::Well), "not the well-constrained contour");
}

/// A joint's point is named by one side and fills the other: the arc between two lines borrows
/// the line ends the chain threads to it.
#[test]
fn threading_fills_the_ends_nobody_wrote() {
    let e = read(&format!("{PTS}{TANGENT_RUN}"));
    let kids = e.sketch.children(EntRef::arc(0));
    assert_eq!(kids[1], EntRef::point(1), "the arc starts where `a` ends");
    assert_eq!(kids[2], EntRef::point(2), "and ends where `b` starts");
    // and both joints are the regular At-form, stated at the ends just threaded
    let ats: Vec<_> = e
        .sketch
        .user_constraints()
        .iter()
        .filter(|c| c.kind == CKind::TangentArcLine)
        .map(|c| format!("{:?}", c.args))
        .collect();
    assert_eq!(ats.len(), 2, "one tangency per joint");
    assert!(ats[0].contains("start") && ats[1].contains("end"), "{ats:?}");
}

/// `close` threads the last exit to the first entry, which is what makes a contour a loop.
#[test]
fn close_threads_back_to_the_first_link() {
    let e = read(
        "point p1 at (0, 0)\npoint p2 at (10, 0)\n\
         line a(p1, p2) to line b(p2) to close\n",
    );
    let kids = e.sketch.children(EntRef::line(1));
    assert_eq!(kids[0], EntRef::point(1), "b starts where a ends");
    assert_eq!(kids[1], EntRef::point(0), "and closes onto a's start");
}

/// Both sides may name the joint, in agreement; two different names are refused, and a joint
/// neither side names is refused, since an unnamed point has no seed and no statement.
#[test]
fn a_joint_is_named_by_exactly_one_side_or_by_both_in_agreement() {
    let agree = "point p1 at (0, 0)\npoint p2 at (10, 0)\npoint p3 at (5, 10)\n\
                 line a(p1, p2) to line b(p2, p3)\n";
    assert!(errors(agree).is_empty(), "{:?}", errors(agree));
    let disagree = "point p1 at (0, 0)\npoint p2 at (10, 0)\npoint p3 at (5, 10)\n\
                    point p4 at (9, 9)\nline a(p1, p2) to line b(p3, p4)\n";
    refuses(disagree, "names two points");
    let nobody = "point p1 at (0, 0)\npoint c at (5, 5)\n\
                  line a(p1) to arc k(center: c, r: 5)\n";
    refuses(nobody, "neither");
}

/// An open chain's first entry and last exit are not joints; left unnamed they are reported,
/// not quietly seeded at the origin.
#[test]
fn an_open_end_must_be_named() {
    let src = "point p3 at (20, 10)\npoint p4 at (20, 30)\npoint c at (10, 10)\n\
               arc k(center: c, r: 5) to line b(p3, p4)\n";
    refuses(src, "leaves `k`'s start unnamed");
}

/// The joint vocabulary maps to the regular forms and refuses the rest: two straight runs
/// meeting tangent are collinear (parallel over the shared point), `perpendicular` joins lines
/// only, two arcs have no regular tangency to state, and a circle has no ends at all.
#[test]
fn the_vocabulary_is_the_regular_forms_or_a_refusal() {
    let e = read(
        "point p1 at (0, 0)\npoint p2 at (10, 0)\npoint p3 at (20, 0)\n\
         line a(p1, p2) tangent line b(p2, p3)\n",
    );
    assert!(e.sketch.user_constraints().iter().any(|c| c.kind == CKind::Parallel));

    let perp = "point p1 at (0, 0)\npoint p2 at (10, 0)\npoint c at (5, 5)\n\
                arc k(center: c, start: p1, end: p2, r: 5) perpendicular line b(p2, p1)\n";
    refuses(perp, "does not join");

    let arcs = "point p1 at (0, 0)\npoint p2 at (10, 0)\npoint p3 at (20, 0)\n\
                point c at (5, 5)\npoint d at (15, 5)\n\
                arc k(center: c, start: p1, end: p2, r: 5) tangent \
                arc m(center: d, start: p2, end: p3, r: 5)\n";
    refuses(arcs, "does not join");

    let circle = "point p1 at (0, 0)\npoint p2 at (10, 0)\npoint c at (5, 5)\n\
                  circle q(center: c, r: 5) to line b(p1, p2)\n";
    refuses(circle, "no ends");

    let lone = "point p1 at (0, 0)\npoint p2 at (10, 0)\nline a(p1, p2) tangent close\n";
    refuses(lone, "at least two");
}

/// Any binary constraint whose spec is two entity slots is an infix word — the two-argument
/// counterpart of the prefix rule, derived from the same registry, and type-checked against the
/// pair it stands between before it desugars.
#[test]
fn a_binary_constraint_is_an_infix_word() {
    let e = read(
        "point p1 at (0, 0)\npoint p2 at (10, 0)\npoint p3 at (10, 10)\n\
         line a(p1, p2) equal_length line b(p2, p3)\n",
    );
    assert!(e.sketch.user_constraints().iter().any(|c| c.kind == CKind::EqualLength));

    let e = read(
        "point p1 at (0, 0)\npoint p2 at (10, 0)\npoint p3 at (20, 0)\n\
         point c at (5, 5)\npoint d at (15, 5)\n\
         arc k(center: c, start: p1, end: p2, r: 5) equal_radius \
         arc m(center: d, end: p3, r: 5)\n",
    );
    assert!(e.sketch.user_constraints().iter().any(|c| c.kind == CKind::EqualRadius));
    // and the joint still threads: m starts where k ends, whatever the joint says about them
    assert_eq!(e.sketch.children(EntRef::arc(1))[1], EntRef::point(1));

    // a word whose slots the pair does not fit is refused, not guessed at
    let unfit = "point p1 at (0, 0)\npoint p2 at (10, 0)\npoint c at (5, 5)\n\
                 line a(p1, p2) equal_radius arc k(center: c, end: p1, r: 5)\n";
    refuses(unfit, "does not join");
}

/// A prefix on a lone declaration is the smallest chain there is: two statements from one line.
#[test]
fn a_prefix_stands_alone() {
    let e = read("point p1 at (0, 0)\npoint p2 at (10, 3)\nhorizontal line a(p1, p2)\n");
    assert!(e.sketch.user_constraints().iter().any(|c| c.kind == CKind::Horizontal));
}

/// The same text parses to the same statements with the same ids, which is what `retext` and a
/// numeric splice rest on.
#[test]
fn reparsing_a_chain_mints_the_same_ids() {
    let a = parse(RECT_FILLETS_CHAIN).0;
    let b = parse(RECT_FILLETS_CHAIN).0;
    let sig = |p: &gcs_core::syntax::Program| {
        p.stmts()
            .map(|s| (s.id, s.span, std::mem::discriminant(&s.kind)))
            .collect::<Vec<_>>()
    };
    assert_eq!(sig(&a), sig(&b));
    assert!(sig(&a).len() > 30, "the chain expanded into its statements");
}

/// Removing a chain-borne relation is a word splice, not a line deletion: a joint steps down to
/// `to` — the corner stays, the claim goes — and a prefix word goes where it stands.  The text
/// still parses, and says one thing less.
#[test]
fn removing_a_chain_relation_splices_a_word() {
    let src = format!("{PTS}{}", TANGENT_RUN.replace("tangent line b", "tangent vertical line b"));
    let e = read(&src);
    let tangency = e
        .sketch
        .user_constraints()
        .iter()
        .find(|c| c.kind == CKind::TangentArcLine)
        .expect("a joint")
        .id;
    let out = edit::remove(&e, &e.program, &[], &[tangency]);
    assert!(out.refused.is_none(), "{:?}", out.refused);
    assert!(out.text.contains("line a(p1, p2) to arc"), "{}", out.text);
    let again = read(&out.text);
    assert_eq!(
        again.sketch.user_constraints().iter().filter(|c| c.kind == CKind::TangentArcLine).count(),
        1,
        "one joint stepped down to a plain corner"
    );

    let level = e
        .sketch
        .user_constraints()
        .iter()
        .find(|c| c.kind == CKind::Vertical)
        .expect("the prefix")
        .id;
    let out = edit::remove(&e, &e.program, &[], &[level]);
    assert!(out.text.contains("tangent line b(p3, p4)"), "{}", out.text);
    assert!(!read(&out.text).sketch.user_constraints().iter().any(|c| c.kind == CKind::Vertical));
}

/// Deleting a link would leave a chain no splice can repair, so it is refused with its reason —
/// the same bargain a component member strikes.
#[test]
fn removing_a_link_is_refused() {
    let e = read(&format!("{PTS}{TANGENT_RUN}"));
    let out = edit::remove(&e, &e.program, &[EntRef::arc(0)], &[]);
    assert!(out.refused.is_some(), "deleting a link went through");
    // and deleting a point a link is threaded through is the same refusal, found transitively
    let out = edit::remove(&e, &e.program, &[EntRef::point(1)], &[]);
    assert!(out.refused.is_some(), "deleting a threaded point went through");
}

/// A seed written as a literal inside a chain link writes back like any other: six characters of
/// one line, with the chain around them untouched.
#[test]
fn a_chain_seed_writes_back() {
    let e = read(&format!("{PTS}{TANGENT_RUN}"));
    let mut sk = e.sketch.clone();
    let r = sk.own_params(EntRef::arc(0))[0] as usize;
    sk.params[r].value = 12.5;
    let out = edit::commit_seeds(&e, &sk, &e.program);
    assert!(out.text.contains("arc k(center: c, r: 12.5) tangent"), "{}", out.text);
}

/// The chain's words are coloured by the parser's own scan, like every other word: joints read
/// as relations, `to` and `close` as structure, and a prefixed element still names itself.
#[test]
fn the_chain_is_coloured() {
    let src = "line a(p1, p2) tangent vertical arc k(center: c, r: 10) to close\n";
    let tint = |what: &str| {
        highlight(src)
            .into_iter()
            .find(|(_, s)| src[s.lo as usize..].starts_with(what))
            .map(|(t, _)| t)
    };
    assert_eq!(tint("tangent"), Some(Tint::Relation));
    assert_eq!(tint("vertical"), Some(Tint::Relation));
    assert_eq!(tint("arc"), Some(Tint::Word));
    assert_eq!(tint("k(center"), Some(Tint::Def));
    assert_eq!(tint("to"), Some(Tint::Word));
    assert_eq!(tint("close"), Some(Tint::Word));
}

/// **The colouring is the parser's own reading, not a second one.**  A prefix word qualifies an
/// element only when an element follows it — so `horizontal(bottom)` is the longhand statement
/// it always was, and `horizontal foo` is neither, and the colour says so in both cases.  Both
/// questions go through `opens_link`; asked twice, the two copies drifted on exactly this.
#[test]
fn a_prefix_word_is_only_a_prefix_where_the_parser_reads_one() {
    let longhand = "line a(p1, p2)\nhorizontal(a)\n";
    let e = read(&format!(
        "point p1 at (0, 0)\npoint p2 at (10, 3)\n{longhand}"
    ));
    assert!(e.sketch.user_constraints().iter().any(|c| c.kind == CKind::Horizontal));
    // the statement is a relation, so its name colours as one and nothing reads as a chain
    let runs = highlight(longhand);
    let at = |what: &str| {
        runs.iter()
            .find(|(_, s)| longhand[s.lo as usize..].starts_with(what))
            .map(|(t, _)| *t)
    };
    assert_eq!(at("horizontal"), Some(Tint::Relation));

    // and a prefix word before something that is not an element opens no chain either way
    let neither = "horizontal foo\n";
    assert!(!errors(neither).is_empty(), "`horizontal foo` is not a statement");
    let runs = highlight(neither);
    assert!(
        runs.iter().all(|(_, s)| !neither[s.lo as usize..].starts_with("foo")),
        "`foo` coloured as an element the parser never read"
    );
}

/// A chain of one — a plain declaration — is exactly what it always was: whole-line statements
/// carrying no chain provenance, deletable as lines.
#[test]
fn a_plain_declaration_is_untouched_by_the_sugar() {
    let (prog, errs) = parse("point p1 at (0, 0)\npoint p2 at (10, 0)\nline a(p1, p2)\n");
    assert!(errs.is_empty());
    let marks: Vec<Chained> = prog.stmts().map(|s| s.chained).collect();
    assert_eq!(marks, vec![Chained::No; 3]);
}

/// What the parser recorded about a chain's words is what `edit` splices on, so the provenance
/// is asserted where it is written rather than only through its consequences.
#[test]
fn a_chain_records_how_each_statement_is_spelled() {
    let (prog, errs) = parse(
        "horizontal line a(p1, p2) tangent arc k(center: c, r: 5) tangent close\n",
    );
    assert!(errs.is_empty(), "{errs:?}");
    let marks: Vec<Chained> = prog.stmts().map(|s| s.chained).collect();
    assert_eq!(
        marks,
        vec![Chained::Prefix, Chained::Link, Chained::Joint, Chained::Link, Chained::Close]
    );
}
