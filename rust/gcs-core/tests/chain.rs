//! Chains: the prefix and joint sugar (spec §6.6).
//!
//! A chain is a parser addition rather than a change of shape — `horizontal line bottom(b1, b2)
//! tangent arc a(center: c) hint(r: 5) …` desugars into the declarations and relations it is sugar
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

/// The shipped fillet case as it was written *longhand* — the independent witness the chain
/// spelling is held against.
///
/// `rect_fillets.sv` is the chain now, so comparing the case against a chain written here would
/// compare a thing with itself.  What has to be true is that the two *spellings* are one
/// drawing, and that needs both of them: this is the one the language always had.
const RECT_FILLETS_LONGHAND: &str = "\
param w = 100
param h = 60
param r = 10

point b1 hint(x: r, y: 0)
point b2 hint(x: w - r, y: 0)
point r1 hint(x: w, y: r)
point r2 hint(x: w, y: h - r)
point t1 hint(x: w - r, y: h)
point t2 hint(x: r, y: h)
point l1 hint(x: 0, y: h - r)
point l2 hint(x: 0, y: r)

point c_br hint(x: w - r, y: r)
point c_tr hint(x: w - r, y: h - r)
point c_tl hint(x: r, y: h - r)
point c_bl hint(x: r, y: r)

line bottom(b1, b2)
arc a_br(center: c_br, start: b2, end: r1) hint(r: r)
line right(r1, r2)
arc a_tr(center: c_tr, start: r2, end: t1) hint(r: r)
line top(t1, t2)
arc a_tl(center: c_tl, start: t2, end: l1) hint(r: r)
line left(l1, l2)
arc a_bl(center: c_bl, start: l2, end: b1) hint(r: r)

horizontal bottom
horizontal top
vertical left
vertical right

a_br tangent(at: start) bottom
a_br tangent(at: end) right
a_tr tangent(at: start) right
a_tr tangent(at: end) top
a_tl tangent(at: start) top
a_tl tangent(at: end) left
a_bl tangent(at: start) left
a_bl tangent(at: end) bottom

a_br equal a_tr
a_tr equal a_tl
a_tl equal a_bl
radius(r) a_bl

l1 distance(w) r2
t1 distance(h) b2

ground c_bl
";

/// Five points and a centre, the cast every small chain below is drawn from.
const PTS: &str = "point p1 hint(x: 0, y: 0)\npoint p2 hint(x: 10, y: 0)\npoint p3 hint(x: 20, y: 10)\n\
                   point p4 hint(x: 20, y: 30)\npoint c hint(x: 10, y: 10)\n";

/// One line, one arc, one line, tangent at both joints — the smallest chain with a threaded
/// element in the middle, and what three of the edit tests below are written against.
const TANGENT_RUN: &str =
    "line a(p1, p2) tangent arc k(center: c) hint(r: 10) tangent line b(p3, p4)\n";

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

/// The same for a complaint **elaboration** carries.  Which of the two a mistake lands in is not
/// arbitrary: what a word *means* is the kinds of its operands, and a name does not carry its
/// kind until elaboration — so `a equal q` is refused there, and a chain that declared its
/// elements is refused there too, because every statement now goes through one settling.
fn refuses_later(src: &str, needle: &str) {
    let (prog, errs) = parse(src);
    assert!(errs.is_empty(), "it parses: {errs:?}");
    let e = elaborate(&prog);
    let msgs: Vec<String> = e.errors().map(|d| d.message.clone()).collect();
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

/// **The gate.**  The shipped case — a chain — says exactly what the longhand says: same
/// entities at the same indices, the same constraints with the same arguments, and the same
/// drawing once solved.
#[test]
fn the_chain_states_the_longhand() {
    let chained = examples::case("rect_fillets").expect("the shipped case");
    let long = read(RECT_FILLETS_LONGHAND).sketch;
    let mut sk = chained;
    assert_eq!(
        (sk.points.len(), sk.lines.len(), sk.arcs.len()),
        (long.points.len(), long.lines.len(), long.arcs.len()),
        "the chain draws something else"
    );
    assert!(
        examples::source("rect_fillets").expect("its source").contains(" tangent\n"),
        "the shipped case is supposed to be the chain spelling"
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
        "point p1 hint(x: 0, y: 0)\npoint p2 hint(x: 10, y: 0)\n\
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
    let agree = "point p1 hint(x: 0, y: 0)\npoint p2 hint(x: 10, y: 0)\npoint p3 hint(x: 5, y: 10)\n\
                 line a(p1, p2) to line b(p2, p3)\n";
    assert!(errors(agree).is_empty(), "{:?}", errors(agree));
    let disagree = "point p1 hint(x: 0, y: 0)\npoint p2 hint(x: 10, y: 0)\npoint p3 hint(x: 5, y: 10)\n\
                    point p4 hint(x: 9, y: 9)\nline a(p1, p2) to line b(p3, p4)\n";
    refuses(disagree, "names two points");
    let nobody = "point p1 hint(x: 0, y: 0)\npoint c hint(x: 5, y: 5)\n\
                  line a(p1) to arc k(center: c) hint(r: 5)\n";
    refuses(nobody, "neither");
}

/// An open chain's first entry and last exit are not joints; left unnamed they are reported,
/// not quietly seeded at the origin.
#[test]
fn an_open_end_must_be_named() {
    let src = "point p3 hint(x: 20, y: 10)\npoint p4 hint(x: 20, y: 30)\npoint c hint(x: 10, y: 10)\n\
               arc k(center: c) hint(r: 5) to line b(p3, p4)\n";
    refuses(src, "leaves `k`'s start unnamed");
}

/// The joint vocabulary maps to the regular forms and refuses the rest: two straight runs
/// meeting tangent are collinear (parallel over the shared point), `perpendicular` joins lines
/// only, two arcs have no regular tangency to state, and a circle has no ends at all.
#[test]
fn the_vocabulary_is_the_regular_forms_or_a_refusal() {
    let e = read(
        "point p1 hint(x: 0, y: 0)\npoint p2 hint(x: 10, y: 0)\npoint p3 hint(x: 20, y: 0)\n\
         line a(p1, p2) tangent line b(p2, p3)\n",
    );
    assert!(e.sketch.user_constraints().iter().any(|c| c.kind == CKind::Parallel));

    let perp = "point p1 hint(x: 0, y: 0)\npoint p2 hint(x: 10, y: 0)\npoint c hint(x: 5, y: 5)\n\
                arc k(center: c, start: p1, end: p2) hint(r: 5) perpendicular line b(p2, p1)\n";
    refuses_later(perp, "does not relate");

    let arcs = "point p1 hint(x: 0, y: 0)\npoint p2 hint(x: 10, y: 0)\npoint p3 hint(x: 20, y: 0)\n\
                point c hint(x: 5, y: 5)\npoint d hint(x: 15, y: 5)\n\
                arc k(center: c, start: p1, end: p2) hint(r: 5) tangent \
                arc m(center: d, start: p2, end: p3) hint(r: 5)\n";
    refuses(arcs, "already meet there");

    let circle = "point p1 hint(x: 0, y: 0)\npoint p2 hint(x: 10, y: 0)\npoint c hint(x: 5, y: 5)\n\
                  circle q(center: c) hint(r: 5) to line b(p1, p2)\n";
    refuses(circle, "no ends");

    let lone = "point p1 hint(x: 0, y: 0)\npoint p2 hint(x: 10, y: 0)\nline a(p1, p2) tangent close\n";
    refuses(lone, "at least two");
}

/// **An operator between two links is the same operator it is between two names.**  A joint is
/// not a grammar of its own: `equal` is `EqualLength` between lines and `EqualRadius` between
/// arcs, settled by the kinds either way, and the chain contributes only the corner.
#[test]
fn a_binary_constraint_is_an_infix_word() {
    let e = read(
        "point p1 hint(x: 0, y: 0)\npoint p2 hint(x: 10, y: 0)\npoint p3 hint(x: 10, y: 10)\n\
         line a(p1, p2) equal line b(p2, p3)\n",
    );
    assert!(e.sketch.user_constraints().iter().any(|c| c.kind == CKind::EqualLength));

    let e = read(
        "point p1 hint(x: 0, y: 0)\npoint p2 hint(x: 10, y: 0)\npoint p3 hint(x: 20, y: 0)\n\
         point c hint(x: 5, y: 5)\npoint d hint(x: 15, y: 5)\n\
         arc k(center: c, start: p1, end: p2) hint(r: 5) equal \
         arc m(center: d, end: p3) hint(r: 5)\n",
    );
    assert!(e.sketch.user_constraints().iter().any(|c| c.kind == CKind::EqualRadius));
    // and the joint still threads: m starts where k ends, whatever the joint says about them
    assert_eq!(e.sketch.children(EntRef::arc(1))[1], EntRef::point(1));

    // a word whose operands it does not relate is refused, not guessed at
    let unfit = "point p1 hint(x: 0, y: 0)\npoint p2 hint(x: 10, y: 0)\npoint c hint(x: 5, y: 5)\n\
                 line a(p1, p2) perpendicular arc k(center: c, end: p1) hint(r: 5)\n";
    refuses_later(unfit, "does not relate");
}

/// A prefix on a lone declaration is the smallest chain there is: two statements from one line.
#[test]
fn a_prefix_stands_alone() {
    let e = read("point p1 hint(x: 0, y: 0)\npoint p2 hint(x: 10, y: 3)\nhorizontal line a(p1, p2)\n");
    assert!(e.sketch.user_constraints().iter().any(|c| c.kind == CKind::Horizontal));
}

/// The same text parses to the same statements with the same ids, which is what `retext` and a
/// numeric splice rest on.
#[test]
fn reparsing_a_chain_mints_the_same_ids() {
    let src = examples::source("rect_fillets").expect("its source");
    let a = parse(src).0;
    let b = parse(src).0;
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
    assert!(out.text.contains("arc k(center: c) hint(r: 12.5) tangent"), "{}", out.text);
}

/// The chain's words are coloured by the parser's own scan, like every other word: joints read
/// as relations, `to` and `close` as structure, and a prefixed element still names itself.
#[test]
fn the_chain_is_coloured() {
    let src = "line a(p1, p2) tangent vertical arc k(center: c) hint(r: 10) to close\n";
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

/// **The colouring is the parser's own reading, not a second one.**  A prefix word opens a
/// *link* only when an element keyword follows it; before a bare name it is the prefix operator
/// (`horizontal a`), which declares nothing.  Both questions go through `opens_link`; asked
/// twice, the two copies drifted on exactly this.
#[test]
fn a_prefix_word_is_only_a_prefix_where_the_parser_reads_one() {
    let longhand = "line a(p1, p2)\nhorizontal a\n";
    let e = read(&format!(
        "point p1 hint(x: 0, y: 0)\npoint p2 hint(x: 10, y: 3)\n{longhand}"
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

    // and a prefix word before a *name* is the prefix operator it is — `horizontal foo` is a
    // statement about `foo`, refused at elaboration where `foo` turns out to be nothing
    let neither = "horizontal foo\n";
    assert!(errors(neither).is_empty(), "`horizontal foo` parses: {:?}", errors(neither));
    refuses_later(neither, "needs to know what `foo` is");
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
    let (prog, errs) = parse("point p1 hint(x: 0, y: 0)\npoint p2 hint(x: 10, y: 0)\nline a(p1, p2)\n");
    assert!(errs.is_empty());
    let marks: Vec<Chained> = prog.stmts().map(|s| s.chained).collect();
    assert_eq!(marks, vec![Chained::No; 3]);
}

/// What the parser recorded about a chain's words is what `edit` splices on, so the provenance
/// is asserted where it is written rather than only through its consequences.
#[test]
fn a_chain_records_how_each_statement_is_spelled() {
    let (prog, errs) = parse(
        "horizontal line a(p1, p2) tangent arc k(center: c) hint(r: 5) tangent close\n",
    );
    assert!(errs.is_empty(), "{errs:?}");
    let marks: Vec<Chained> = prog.stmts().map(|s| s.chained).collect();
    assert_eq!(
        marks,
        vec![Chained::Prefix, Chained::Link, Chained::Joint, Chained::Link, Chained::Close]
    );
}

/* -- relation chains: operands that name rather than declare ---------------------------- */

/// **`equal` is polymorphic, and a chain over names states it.**  Between arcs it is a radius,
/// between lines a length, and which one is settled by what the names turn out to be.
#[test]
fn equal_chains_over_names() {
    let e = read(&format!(
        "{PTS}point d hint(x: 30, y: 10)\n\
         arc k(center: c, start: p1, end: p2) hint(r: 10)\n\
         arc m(center: d, start: p2, end: p3) hint(r: 10)\n\
         arc q(center: d, start: p3, end: p4) hint(r: 10)\n\
         k equal m equal q\n"
    ));
    let kinds: Vec<CKind> = e.sketch.user_constraints().iter().map(|c| c.kind).collect();
    assert_eq!(
        kinds.iter().filter(|&&k| k == CKind::EqualRadius).count(),
        2,
        "three arcs chained is two statements, not three: {kinds:?}"
    );

    // the same word between lines is the other equality
    let e = read(
        "point p1 hint(x: 0, y: 0)\npoint p2 hint(x: 10, y: 0)\npoint p3 hint(x: 10, y: 9)\n\
         line a(p1, p2)\nline b(p2, p3)\n\
         a equal b\n",
    );
    assert!(e.sketch.user_constraints().iter().any(|c| c.kind == CKind::EqualLength));
}

/// A relation chain does **not** thread.  Arcs have ends, so a chain that welded them would
/// quietly say the two are adjacent as well as the same size — which is not what was written.
#[test]
fn a_relation_chain_threads_nothing() {
    let e = read(&format!(
        "{PTS}point d hint(x: 30, y: 10)\n\
         arc k(center: c, start: p1, end: p2) hint(r: 10)\n\
         arc m(center: d, start: p3, end: p4) hint(r: 10)\n\
         k equal m\n"
    ));
    // each arc kept the ends it was given: nothing was threaded onto anything
    assert_eq!(e.sketch.children(EntRef::arc(0))[1], EntRef::point(0));
    assert_eq!(e.sketch.children(EntRef::arc(0))[2], EntRef::point(1));
    assert_eq!(e.sketch.children(EntRef::arc(1))[1], EntRef::point(2));
    assert_eq!(e.sketch.children(EntRef::arc(1))[2], EntRef::point(3));
    assert_eq!(e.sketch.user_constraints().iter().filter(|c| c.kind == CKind::Coincident).count(), 0);
}

/// Every binary constraint is an infix word over names too, not just `equal` — the same
/// registry rule the declaring form follows.
#[test]
fn any_binary_word_chains_over_names() {
    let e = read(
        "point p1 hint(x: 0, y: 0)\npoint p2 hint(x: 10, y: 0)\npoint p3 hint(x: 0, y: 9)\npoint p4 hint(x: 9, y: 9)\n\
         line a(p1, p2)\nline b(p3, p4)\n\
         a parallel b\n",
    );
    assert!(e.sketch.user_constraints().iter().any(|c| c.kind == CKind::Parallel));
}

/// A chain either declares every element or names every one.  Mixing them asks the two rules
/// about threading at once, so it is refused rather than answered arbitrarily.
#[test]
fn a_chain_may_not_mix_declarations_and_names() {
    let src = "point p1 hint(x: 0, y: 0)\npoint p2 hint(x: 10, y: 0)\npoint p3 hint(x: 10, y: 9)\n\
               line a(p1, p2)\n\
               line b(p2, p3) equal a\n";
    refuses(src, "declares every element or names every one");
}

/// **`to` is the only word that needs a corner**, and a relation chain has none.
///
/// `tangent` used to be here beside it.  It is not any more: between two names it is the
/// ordinary infix operator, and `a tangent k` is a statement about two things that touch — which
/// needs no corner to be one.  It is only *in a contour* that it means "and at the point they
/// share", which is the chain contributing what the operator cannot know.
#[test]
fn only_a_corner_word_is_refused_between_names() {
    let base = "point p1 hint(x: 0, y: 0)\npoint p2 hint(x: 10, y: 0)\npoint c hint(x: 5, y: 5)\n\
                line a(p1, p2)\narc k(center: c, start: p1, end: p2) hint(r: 5)\n";
    refuses(&format!("{base}a to k\n"), "corner");
    refuses(&format!("{base}a equal a to close\n"), "no loop");
    // and the tangency between two names states itself, at the end the drawing already shares
    let e = read(&format!("{base}k tangent a\n"));
    assert!(e.sketch.user_constraints().iter().any(|c| c.kind == CKind::TangentArcLine));
}

/// `equal` between kinds no constraint relates is an error, reported where the kinds become
/// known — which is elaboration, for a chain that declared its elements and for one that only
/// named them alike.
#[test]
fn equal_across_kinds_is_refused_either_way() {
    // declared or named, it is settled in one place now: what a word means is the kinds of its
    // operands, and asking that question twice is what let the two answers drift
    let declared = "point p1 hint(x: 0, y: 0)\npoint p2 hint(x: 10, y: 0)\npoint c hint(x: 5, y: 5)\n\
                    line a(p1, p2) equal arc k(center: c, start: p2, end: p1) hint(r: 5)\n";
    refuses_later(declared, "does not relate");

    // named: only elaboration knows, so the diagnosis carries it
    let named = "point p1 hint(x: 0, y: 0)\npoint p2 hint(x: 10, y: 0)\npoint c hint(x: 5, y: 5)\n\
                 line a(p1, p2)\ncircle q(center: c) hint(r: 5)\n\
                 a equal q\n";
    let (prog, errs) = parse(named);
    assert!(errs.is_empty(), "it parses: {errs:?}");
    let e = elaborate(&prog);
    assert!(
        e.errors().any(|d| d.message.contains("does not relate")),
        "{:?}",
        e.errors().map(|d| d.message.clone()).collect::<Vec<_>>()
    );
}

/// A name declared *after* the chain that reads it still resolves — which is the whole reason
/// `equal` over names cannot be settled as it parses.
#[test]
fn equal_reads_a_name_declared_further_down() {
    let e = read(
        "point p1 hint(x: 0, y: 0)\npoint p2 hint(x: 10, y: 0)\npoint p3 hint(x: 10, y: 9)\n\
         a equal b\n\
         line a(p1, p2)\nline b(p2, p3)\n",
    );
    assert!(e.sketch.user_constraints().iter().any(|c| c.kind == CKind::EqualLength));
}

/* -- the operator form (spec §9.1) ---------------------------------------------------------- */

/// **Every statement is a prefix or an infix operator, and `name(args…)` is retired.**
///
/// `radius(25) c` and `p1 distance(80) p2` — the word, whatever is not one of its two operands
/// in the parentheses, and nothing else.  This is the shape the library already had: every
/// user-facing constraint has one or two entity slots, always first in spec order, with
/// `Symmetric` the single exception the parentheses absorb.
#[test]
fn a_statement_is_a_prefix_or_an_infix_operator() {
    let e = read(
        "point p1 hint(x: 0, y: 0)\npoint p2 hint(x: 60, y: 0)\npoint p3 hint(x: 60, y: 40)\n\
         point c hint(x: 20, y: 20)\ncircle k(center: c) hint(r: 5)\n\
         line l(p1, p2)\nline m(p2, p3)\n\
         radius(25) k\n\
         p1 distance(80) p2\n\
         horizontal l\n\
         l perpendicular m\n\
         p1 symmetry(m) p3\n\
         p1 distance(20, along: y) p3\n",
    );
    let kinds: Vec<CKind> = e.sketch.user_constraints().iter().map(|c| c.kind).collect();
    for want in [
        CKind::Radius,
        CKind::Distance,
        CKind::Horizontal,
        CKind::Perpendicular,
        CKind::Symmetric,
        CKind::VerticalDistance,
    ] {
        assert!(kinds.contains(&want), "{want:?} not among {kinds:?}");
    }
    // `symmetry`'s third entity went into the parentheses and came out in the third spec slot
    let sym = e.sketch.user_constraints().into_iter().find(|c| c.kind == CKind::Symmetric).unwrap();
    assert_eq!(gcs_core::io::describe(sym), "P0 symmetry(L1) P2");
}

/// **The fixity does the work** for `horizontal` and `vertical`: a line prefixed, a pair of
/// points infixed — which is exactly the distinction `HorizontalPoints` was added to draw.
#[test]
fn one_word_two_constraints_by_fixity() {
    let e = read(
        "point p1 hint(x: 0, y: 0)\npoint p2 hint(x: 60, y: 3)\nline l(p1, p2)\n\
         horizontal l\np1 horizontal p2\n",
    );
    let kinds: Vec<CKind> = e.sketch.user_constraints().iter().map(|c| c.kind).collect();
    assert!(kinds.contains(&CKind::Horizontal) && kinds.contains(&CKind::HorizontalPoints));
}

/// `distance` before a line is sugar for the distance between its own ends, and states exactly
/// what naming them would.
#[test]
fn a_prefix_distance_is_the_distance_between_the_ends() {
    let a = read("point p1 hint(x: 0, y: 0)\npoint p2 hint(x: 6, y: 0)\nline l(p1, p2)\ndistance(6) l\n");
    let b = read(
        "point p1 hint(x: 0, y: 0)\npoint p2 hint(x: 6, y: 0)\nline l(p1, p2)\n\
         l.p1 distance(6) l.p2\n",
    );
    assert_eq!(
        gcs_core::io::describe(a.sketch.user_constraints()[0]),
        gcs_core::io::describe(b.sketch.user_constraints()[0]),
    );
}

/// **`on` is five constraints and one word**, told apart by the right operand's kind — including
/// a name that comes from a component, which is the case deferred settling exists for.
#[test]
fn on_resolves_across_every_kind_it_reaches() {
    let src = "\
component Holder() {
  port hub: point
  point o hint(x: 0, y: 0)
  circle k(center: o) hint(r: 20)
  port ring = k
}
g: Holder()
point p hint(x: 20, y: 0)
p on g.ring
point q hint(x: 5, y: 0)
point r hint(x: 30, y: 0)
line   l(q, r)
point s hint(x: 10, y: 1)
s on l
";
    let e = read(src);
    let kinds: Vec<CKind> = e.sketch.user_constraints().iter().map(|c| c.kind).collect();
    assert!(kinds.contains(&CKind::PointOnCircle), "a name from a component: {kinds:?}");
    assert!(kinds.contains(&CKind::PointOnLine));
}

/// Round-trip: parse → print → parse gives the same statements.
#[test]
fn the_operator_form_round_trips() {
    for key in ["rect_fillets", "truss", "pythagoras", "altitudes"] {
        let src = examples::source(key).expect("its source");
        let a = read(src);
        let mut p = gcs_core::program::to_program(&a.sketch);
        let text = gcs_core::syntax::render(&mut p).to_string();
        let b = read(&text);
        assert_eq!(
            gcs_core::io::dumps(&a.sketch, Some(1)),
            gcs_core::io::dumps(&b.sketch, Some(1)),
            "{key} did not come back the same:\n{text}"
        );
        assert!(!text.contains("=="), "no statement is written as a call: {text}");
    }
}

/// **No document, fixture or example writes a constraint as a call.**  The acceptance criterion,
/// as a test: a `name(` at the head of a statement is the retired form.
#[test]
fn no_document_writes_a_call() {
    for (_, key, _) in gcs_core::examples::CASES {
        let Some(src) = examples::source(key) else { continue };
        for (n, line) in src.lines().enumerate() {
            let code = line.split("//").next().unwrap_or("").trim();
            // the retired *names*, and every word that is infix-only opening a call — a
            // prefix operator's own parentheses (`radius(r) a_bl`) look like one and are not
            for w in [
                "point_on_", "equal_length", "equal_radius", "tangent_", "horizontal_points",
                "vertical_points", "horizontal_distance", "vertical_distance",
                "parallel_distance", "point_line_distance", "annular_distance", "symmetric(",
                "angle(", "parallel(", "perpendicular(", "coincident(", "midpoint(",
                "symmetry(", "equal(", "on(", "curvature(", "tangent(", "ground(", "fix(",
                "horizontal(", "vertical(",
            ] {
                assert!(!code.starts_with(w), "{key}:{}: a call — {line}", n + 1);
            }
        }
    }
}

/// **A prefix word carrying its own number opens a chain, exactly as a bare one does.**
///
/// `radius(25) circle base(center: o)` is the prefix form of a dimension (spec §9.1), and the
/// only thing between it and `horizontal line l(a, b)` is the parentheses.  The chain lookahead
/// read the token at `i + 1`, found `(` rather than the element keyword, and opened no link — so
/// the statement fell to `relation()`, whose `refr()` swallowed the keyword `circle` as an
/// operand and reported "no such entity: 'circle'".  The *same* form parsed perfectly well as a
/// later link, where `link` reads the arguments itself, which is the tell: one grammar was being
/// asked two questions.  Both spellings state the same two things here, in both positions.
#[test]
fn a_parenthesised_prefix_opens_a_chain() {
    let leading = read("point o hint(x: 0, y: 0)\nradius(25) circle base(center: o)\n");
    let kinds = |e: &Elaborated| -> Vec<CKind> {
        e.sketch.user_constraints().iter().map(|c| c.kind).collect()
    };
    assert_eq!(kinds(&leading), vec![CKind::Radius], "the leading link states its radius");
    assert_eq!(leading.sketch.circles.len(), 1, "and declares the circle it stands before");

    // the same word, one link along: what already worked, and what the leading link now matches
    let later = read(&format!(
        "{PTS}{}",
        TANGENT_RUN.replace("tangent arc k", "tangent radius(10) arc k")
    ));
    assert!(kinds(&later).contains(&CKind::Radius), "and so does the same word mid-chain");
}

/// **A slot is named where it is written, and the name is kept** (spec §4.3, §9.1).
///
/// `t == 0.4` and `hint(t: 0.4)` are the same number read the same way — a pin is one a solve may
/// not revise and a seed is where it begins — so an operator carries one `OpArg::Slot` for both,
/// holding the key the writer used and the ordinary `syntax::Arg` the elaborator wants.  Three
/// things went wrong when that key and that text were thrown away, and they are the three checked
/// here: a pin written as an *expression* kept only the parsed value, and `value_text` returns
/// none for an expression, so `t == t0` inside a component pinned the contact at 0 with no
/// diagnostic anywhere; a key naming no slot at all was accepted and quietly filled whichever
/// `Param` slot the settled kind had; and the printer guessed the name `t`, which is right on a
/// spline and wrong on a curve, so it disagreed with `operator_text` about the same statement.
///
/// The printed name is therefore stated over a **curve**, whose slot is `u`: on a spline the
/// guess and the truth are the same string, so a spline can witness nothing here.
#[test]
fn a_slot_keeps_the_name_and_the_number_it_was_written_with() {
    // a pin written over a component's parameters, settled by `flatten` and *pinned*
    let e = read(&format!(
        "component C(t0: Scalar) {{\n{}  a on(t == t0) s\n}}\nc: C(t0: 0.25)\n",
        SPLINE.replace('\n', "\n  ").trim_end_matches(' ')
    ));
    let cs = e.sketch.user_constraints();
    let c = cs.iter().find(|c| c.kind == CKind::PointOnSpline).expect("the contact");
    let gcs_core::constraints::Arg::Param(i) = c.args[2] else { panic!("a Param slot: {c:?}") };
    let p = &e.sketch.params[i as usize];
    assert!((p.value - 0.25).abs() < 1e-12, "the pin is what was written, not 0: {}", p.value);
    assert!(p.fixed, "and it is still a pin");

    // a key the kind has no slot for is a typo, not something to fill the first slot with —
    // reported on the key, which is what it is about, and in the word the writer typed
    refuses_later(&format!("{SPLINE}a on s hint(bogus: 0.4)\n"), "`on` has no slot `bogus`");

    // and the printer writes the name that was written.  `u` is the case: a curve's slot is not
    // called `t`, so the retired hard-code printed `hint(t: …)` for a statement that said `u`.
    for stated in ["p on flank hint(u: 20)", "p on(u == 20) flank"] {
        let src = format!("{CURVE}{stated}\n");
        let (mut prog, errs) = parse(&src);
        assert!(errs.is_empty(), "{stated} parses: {errs:?}");
        let text = gcs_core::syntax::render(&mut prog).to_string();
        assert!(text.contains(stated), "{stated} did not print back:\n{text}");
        assert!(!text.contains("(t:") && !text.contains("t =="), "no guessed `t`:\n{text}");
    }
}

/// A spline and a point beside it, for the statements above that need an owned slot called `t`.
const SPLINE: &str = "point s0 hint(x: 0, y: 0)\npoint s1 hint(x: 20, y: 10)\n\
                      point s2 hint(x: 40, y: 10)\npoint s3 hint(x: 60, y: 0)\n\
                      spline s(s0, s1, s2, s3)\npoint a hint(x: 30, y: 8)\n";

/// The same, for a curve family written in the document — whose slot is called `u`.
const CURVE: &str = "\
curve involute(c: circle, phase: Angle)(u) over (0, 90) =
  ( c.center.x + c.r * (cos(u + phase) + u / 1rad * sin(u + phase)),
    c.center.y + c.r * (sin(u + phase) - u / 1rad * cos(u + phase)) )
point o hint(x: 0, y: 0)
circle base(center: o) hint(r: 20)
curve flank = involute(base, phase: 0) over (0, 60)
point p hint(x: 40, y: 40)
";
