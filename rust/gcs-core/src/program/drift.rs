//! **The drift lint** (#462): six patterns that are drift from Solvent's style by definition.
//! They are not counts of an idiom, which a legitimate new example raises and a reshuffle onto
//! separate lines lowers, but places where a statement says something the language has a better
//! way to say (R10 of `docs/solvent-direction.md`).
//!
//! Advisory: each pattern is a warning of its own code, asked for (`solventc --lint`) and never
//! raised by elaboration, so nothing is refused on its word.  Asked of the document's own text —
//! its components and its root — and never of a module it reads, which is linted as a document of
//! its own.  Asked of the text, before anything is bound, so it holds of a component whether or
//! not anything instantiates it.

use super::{Code, Diag};
use crate::syntax::{hidden, name_uses, Arg, BlockKind, Chained, Component, CurveTarget, InstArg,
    InstVal, Instance, Name, OpArg, Program, Span, Stmt, StmtId, StmtKind, Sweep, Ty, Written};
use std::collections::{BTreeMap, BTreeSet};

/// The lint's codes, one per pattern: a pun, a bare component statement, an asserting component,
/// a deep path, a presentation flag, a group threaded unread.
pub const DRIFT: [Code; 6] =
    [Code::W201, Code::W202, Code::W203, Code::W204, Code::W205, Code::W206];

/// Every place the document's own text drifts, in the order a reader meets them.
pub fn drift(p: &Program) -> Vec<Diag> {
    let mut out = Vec::new();
    let mut inside = BTreeMap::new();
    for c in p.components.iter().filter(|c| c.module.is_none()) {
        let mut pieces = Vec::new();
        cut(&mut c.body.iter(), &mut pieces);
        let read = read(p.text(), &pieces);
        puns(p, &c.body, &mut out);
        bare(&c.body, &mut out);
        asserting(c, &mut out);
        deep(p, c, &read, &mut inside, &mut out);
        formals(c, &read, &mut out);
    }
    out.sort_by_key(|d| (d.span.lo, d.code.as_str()));
    out.dedup_by(|a, b| a.span.lo == b.span.lo && a.code == b.code);
    out
}

/// A piece of a body's text the lexical patterns read: where it is, the statement it is of, and
/// whether it is a `when`'s condition.
struct Piece {
    span: Span,
    stmt: StmtId,
    when: bool,
}

/// The body cut into pieces no two of which hold one name: a block is its head and its
/// statements, so a name in a block's body is read once and not again as part of the block.
fn cut<'a>(body: &mut dyn Iterator<Item = &'a Stmt>, out: &mut Vec<Piece>) {
    for st in body {
        let piece = |span: Span, when: bool| Piece { span, stmt: st.id, when };
        match &st.kind {
            StmtKind::Block(b) if b.image.is_none() => {
                out.push(piece(b.head, b.kind == BlockKind::When));
                cut(&mut b.stmts(), out);
            }
            StmtKind::ClaimOver(c) => {
                let head = c.body.first().map_or(c.span.hi, |s| s.span.lo);
                out.push(piece(Span { lo: c.span.lo, hi: head.max(c.span.lo) }, false));
                cut(&mut c.body.iter(), out);
            }
            StmtKind::Set(s) => cut(&mut s.lit.body.iter(), out),
            _ => out.push(piece(st.span, false)),
        }
    }
}

/// Every name the pieces read, its dotted parts each at its place in the document, beside the
/// piece it is read in.
fn read<'a>(text: &str, pieces: &'a [Piece]) -> Vec<(Vec<(String, Span)>, &'a Piece)> {
    let mut out = Vec::new();
    for piece in pieces {
        for u in name_uses(piece.span.slice(text)) {
            let parts = u.parts.into_iter().map(|(n, at)| (n, shift(at, piece.span))).collect();
            out.push((parts, piece));
        }
    }
    out
}

/// A span within a piece's text, as a place in the document.
fn shift(at: Span, piece: Span) -> Span {
    Span { lo: at.lo + piece.lo, hi: at.hi + piece.lo }
}

/// Every statement of a body, those inside its blocks and claims included.
fn each<'a>(body: &'a [Stmt], f: &mut dyn FnMut(&'a Stmt)) {
    for st in body {
        f(st);
        match &st.kind {
            StmtKind::Block(b) => b.stmts().for_each(|s| each(std::slice::from_ref(s), f)),
            StmtKind::ClaimOver(c) => each(&c.body, f),
            StmtKind::Set(s) => each(&s.lit.body, f),
            _ => {}
        }
    }
}

/// **W201: a label given the name it labels**, in a call: `teeth: teeth` to a component, `d: d`
/// to a word.  The value is handed on under the name it already has — a number threaded beside
/// the thing it measures, or context passed down a level at a time (direction N6, N7).  Only a
/// word a file defines takes labels a document chose: a built-in's own slots (`plane(u: u, v:
/// v)`, `distance(0mm, along: along)`) are no call, and a pin (`ratio == ratio`) no label.
fn puns(p: &Program, body: &[Stmt], out: &mut Vec<Diag>) {
    fn labelled<'a>(args: &'a [InstArg], found: &mut Vec<&'a Name>) {
        for a in args {
            match (&a.label, &a.value) {
                (Some(l), InstVal::Ref(r)) if r.path.is_empty() && r.root.text == l.text => {
                    found.push(l)
                }
                (Some(l), InstVal::Expr(t)) if t.trim() == l.text => found.push(l),
                (_, InstVal::Group(g)) => labelled(g, found),
                _ => {}
            }
        }
    }
    each(body, &mut |st| {
        let mut found = Vec::new();
        for i in calls(st) {
            labelled(&i.args, &mut found);
        }
        if let StmtKind::Relation(r) = &st.kind {
            let defined = |w: &&Written| p.words.iter().any(|d| d.word.text == w.word.text);
            for a in r.form.written().filter(defined).iter().flat_map(|w| &w.args) {
                let OpArg::Named(l, arg) = a else { continue };
                let same = match arg {
                    Arg::Ref(r) => r.path.is_empty() && r.root.text == l.text,
                    Arg::Word(w) => *w == l.text,
                    Arg::Dim { text, .. } => text.trim() == l.text,
                    _ => false,
                };
                if same {
                    found.push(l);
                }
            }
        }
        for l in found {
            out.push(Diag::new(Code::W201, l.span, Some(st.id), format!(
                "`{0}: {0}` hands `{0}` on under the name it already has: a value threaded \
                 through a call, where a number could travel with the thing it measures and a \
                 standard be context (direction N6, N7)",
                l.text
            )));
        }
    });
}

/// The components a statement calls: an instance, and one written in place for a curve to trace
/// or a face family to sweep.
fn calls(st: &Stmt) -> Vec<&Instance> {
    match &st.kind {
        StmtKind::Instance(i) => vec![i],
        StmtKind::Decl(d) => {
            let traced = d.curve.as_ref().and_then(|c| match &c.target {
                CurveTarget::Anon(i, _) => Some(i),
                CurveTarget::Drawn(_) => None,
            });
            let swept = match &d.sweep {
                Some(Sweep::Family(f)) => Some(&f.instance),
                _ => None,
            };
            traced.into_iter().chain(swept).collect()
        }
        _ => Vec::new(),
    }
}

/// **W202: a component call standing as a statement** for what it states (`std.Spin(f, …)`): a
/// relation written as a procedure call (direction R3).  A set written in place as an operand
/// (`q inside std.Cylinder(ax, r: 8mm)`) is an instance under the same key, but a link of its
/// relation's statement and not one of its own.
fn bare(body: &[Stmt], out: &mut Vec<Diag>) {
    each(body, &mut |st| {
        let StmtKind::Instance(i) = &st.kind else { return };
        if i.name.text.starts_with("#i") && st.chained == Chained::No {
            out.push(Diag::new(Code::W202, i.component.span, Some(st.id), format!(
                "`{}(…)` stands as a statement, called for what it states: a relation written \
                 as a procedure call (direction R3)",
                i.component.text
            )));
        }
    });
}

/// **W203: a component that makes nothing a parent can read** — everything it declares is
/// `private`, or a claim's witness — and states relations: a fact about what it is handed,
/// written as a component.  A verb (direction R3).  A bare call in it states, whatever the callee
/// makes, since nothing it makes has a name; a number worked out is the formals' arithmetic,
/// where an input nothing binds is an unknown the parent reads.
fn asserting(c: &Component, out: &mut Vec<Diag>) {
    let (Some(name), None) = (&c.name, &c.set) else { return };
    let (mut public, mut private, mut states) = (false, false, false);
    each(&c.body, &mut |st| match &st.kind {
        StmtKind::Decl(d) if d.annotations.private || d.annotations.roles.witness.is_some() => {
            private = true
        }
        StmtKind::Chain(ch) if ch.annotations.private => private = true,
        StmtKind::Instance(i) if hidden(&i.name.text) => states = true,
        StmtKind::Instance(i) if i.annotations.private => private = true,
        StmtKind::Relation(r) if r.name.is_some() => public = true,
        StmtKind::Param(d) if d.bound() => {}
        StmtKind::Block(b) if b.image.is_some() => public = true,
        StmtKind::Decl(_) | StmtKind::Chain(_) | StmtKind::Instance(_) | StmtKind::Param(_)
        | StmtKind::Set(_) | StmtKind::Derived(_) => public = true,
        StmtKind::Relation(_) | StmtKind::Measured(_) | StmtKind::Minimize(_)
        | StmtKind::Branch(_) | StmtKind::SolidRel(_) => states = true,
        StmtKind::Block(_) | StmtKind::ClaimOver(_) | StmtKind::Group(_) | StmtKind::Style(_)
        | StmtKind::Unit(_) => {}
    });
    if public || !states {
        return;
    }
    let makes = if private { "makes nothing a parent can read — all it declares is private —" }
        else { "declares nothing" };
    out.push(Diag::new(Code::W203, name.span, None, format!(
        "`{}` {makes} and only states relations of its formals: a fact about what it is handed, \
         written as a component.  It is a relation word (direction R3)",
        name.text
    )));
}

/// **W204: a path reaching more than one level into another instance** (`pair.gear_cone.ax`): a
/// statement coupled to the inside of a level that is not its own (direction N4).  An instance
/// is known by its statement in this body and the component it calls, read in a block's next or
/// previous copy too (`next.t.r.lo`); a path through an index (`cyl[0].small.p`) or a formal is
/// not followed.
fn deep(p: &Program, c: &Component, read: &[(Vec<(String, Span)>, &Piece)],
    inside: &mut BTreeMap<usize, BTreeSet<String>>, out: &mut Vec<Diag>) {
    let instances = |body: &[Stmt], from: Option<usize>| {
        let mut found = BTreeMap::new();
        each(body, &mut |st| {
            let StmtKind::Instance(i) = &st.kind else { return };
            if !hidden(&i.name.text) {
                let k = p.resolve_component(&i.component.text, from).ok();
                found.insert(i.name.text.clone(), k);
            }
        });
        found
    };
    let here = instances(&c.body, c.module);
    for (written, piece) in read {
        // a copy's sibling is the same body's instance, one copy on
        let copy = matches!(written.first(), Some((w, _)) if w == "next" || w == "prev");
        let parts = &written[copy as usize..];
        if parts.len() < 3 {
            continue;
        }
        let Some(&Some(k)) = here.get(&parts[0].0) else { continue };
        let callee = &p.components[k];
        let subs = inside.entry(k).or_insert_with(|| instances(&callee.body, callee.module)
            .into_keys().collect());
        if !subs.contains(&parts[1].0) {
            continue;
        }
        let path: Vec<&str> = written.iter().map(|(n, _)| n.as_str()).collect();
        let at = Span { lo: written[0].1.lo, hi: written[written.len() - 1].1.hi };
        out.push(Diag::new(Code::W204, at, Some(piece.stmt), format!(
            "`{}` reads through `{}` into its instance `{}`: a statement coupled to the inside of \
             another level.  Relate what `{}` itself makes (direction N4)",
            path.join("."), parts[0].0, parts[1].0, parts[0].0
        )));
    }
}

/// **W205: an `Int` formal read only by `when` conditions**, choosing which of the body is made
/// (`draw_end`) — presentation, where what it chooses is what is drawn, which belongs in the
/// drawing (`.svd`).  **W206: a `group` formal handed on whole and never read** through
/// (`dims`) — configuration threaded through a level that reads none of it (direction N6).
fn formals(c: &Component, read: &[(Vec<(String, Span)>, &Piece)], out: &mut Vec<Diag>) {
    // a group a solid is made of is read here whole, as a material (#510 P3)
    let mut materials = std::collections::BTreeSet::new();
    each(&c.body, &mut |st| {
        if let StmtKind::Decl(d) = &st.kind {
            materials.extend(d.material.as_ref().map(|m| m.root.text.clone()));
        }
    });
    for f in &c.formals {
        let uses: Vec<_> = read.iter().filter(|(parts, _)| parts[0].0 == f.name.text).collect();
        if uses.is_empty() {
            continue;
        }
        if f.ty == Ty::Int && uses.iter().all(|(_, piece)| piece.when) {
            out.push(Diag::new(Code::W205, f.name.span, None, format!(
                "`{}` is read only by `when` conditions: an `Int` choosing which of the body is \
                 made.  Where it chooses what is drawn, it is presentation, which belongs in the \
                 drawing (`.svd`), not the model",
                f.name.text
            )));
        }
        if f.ty == Ty::Group && !materials.contains(&f.name.text)
            && uses.iter().all(|(parts, _)| parts.len() == 1)
        {
            out.push(Diag::new(Code::W206, f.name.span, None, format!(
                "the group `{}` is handed on whole and never read here: configuration threaded \
                 through a level that reads none of it.  Let the component that reads it take it \
                 (direction N6)",
                f.name.text
            )));
        }
    }
}
