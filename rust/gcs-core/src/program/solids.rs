//! Build faces and solids, then apply solid claims and placement.

use super::resolve::Resolver;
use super::{Code, Diag, Made, SourceMap};
use crate::ir::{Decl, Kid, Operation as StmtKind, Relation, Statement as Stmt};
use crate::model::{EntKind, EntRef, Extent, Sense, Sketch, SolidDef};
use crate::style::Classes;
use crate::syntax::{Span, StmtId};
use std::collections::{BTreeMap, BTreeSet};

mod claims;
mod faces;
mod build;
pub(super) use claims::solid_claims;
use claims::place;
use faces::{chain_face,validate_chain,build_face};
use build::build_solid;

/// `boss on cyl`: an `on` whose two operands are both solids.  Asked of the *resolver*, which
/// has known every declaration's kind since phase 1 — so the question is answered the same way
/// before the solids are built and after.
pub(super) fn is_body_on(sk: &Sketch, res: &Resolver, r: &Relation) -> bool {
    let Some(w) = r.form.written() else { return false };
    if w.word.text != "on" || w.ops.len() != 2 {
        return false;
    }
    w.ops.iter().all(|o| res.lookup(o)
        .and_then(|e| super::resolve::follow_building(sk,res,e,o).ok())
        .is_some_and(|e| e.kind == EntKind::Solid))
}

/// Build faces and solids in dependency order, then fold body operations into stock solids.
pub(super) fn solids(
    sk: &mut Sketch,
    res: &mut Resolver,
    map: &mut SourceMap,
    body: &[&Stmt],
    skip: &BTreeSet<StmtId>,
    diags: &mut Vec<Diag>,
) {
    let has = body.iter().any(|st| {
        (matches!(&st.kind, StmtKind::Decl(d) if d.kind.spatial())
            || matches!(&st.kind, StmtKind::Chain(_))) && !skip.contains(&st.id)
    });
    if !has {
        return;
    }
    // -- faces --------------------------------------------------------------
    for st in body {
        let chain_decl;
        let d = match &st.kind {
            StmtKind::Decl(d) => d.as_ref(),
            StmtKind::Chain(c) if !skip.contains(&st.id) => {
                if !validate_chain(sk, res, c, st, diags) {
                    if c.closed {
                        res_forget(res, &c.name.key().text);
                    }
                    continue;
                }
                if !c.closed {
                    map.record(st, Made::Gauge);
                    continue;
                }
                chain_decl = chain_face(c);
                &chain_decl
            }
            _ => continue,
        };
        if d.kind != EntKind::Face || skip.contains(&st.id)
            || d.children.get(2).is_some_and(|g| !g.is_empty()) {
            continue;
        }
        let name = d.name.key().text.clone();
        let first_line = sk.lines.len();
        match build_face(sk, res, d, &d.name.key().text, st.id, st.span, diags) {
            Some(i) => {
                let e = EntRef::face(i);
                map.bind(&name, e, d.name.named());
                map.record(st, Made::Ent(e));
                // The face made these lines.  Record them after their parent so source
                // reconciliation does not mistake them for newly drawn geometry.
                for li in first_line..sk.lines.len() {
                    map.record(st, Made::Ent(EntRef::line(li)));
                }
            }
            None => {
                // A refused loop must not leave unowned closing lines in the sketch.
                sk.lines.truncate(first_line);
                res_forget(res, &name);
            }
        }
    }
    // -- solids -------------------------------------------------------------
    for st in body {
        let StmtKind::Decl(d) = &st.kind else { continue };
        if d.kind != EntKind::Solid || skip.contains(&st.id) {
            continue;
        }
        let name = d.name.key().text.clone();
        let first_face = sk.faces.len();
        let first_line = sk.lines.len();
        match build_solid(sk, res, d, st, diags) {
            Some(i) => {
                let e = EntRef::solid(i);
                map.bind(&name, e, d.name.named());
                map.record(st, Made::Ent(e));
                // The solid owns its inline section and that section's closing lines.
                // Record the solid first, as source editing expects for every declaration.
                for fi in first_face..sk.faces.len() {
                    map.record(st, Made::Ent(EntRef::face(fi)));
                }
                for li in first_line..sk.lines.len() {
                    map.record(st, Made::Ent(EntRef::line(li)));
                }
            }
            None => {
                sk.faces.truncate(first_face);
                sk.lines.truncate(first_line);
                res_forget(res, &name);
            }
        }
    }
    // -- the body rule --------------------------------------------------------
    // `bore cut cyl`, `boss on cyl`, folded into the body they name.  **Both are sets**, so
    // the order this walk meets them in cannot matter, and a document may write them anywhere.
    for st in body {
        if skip.contains(&st.id) {
            continue;
        }
        let (word, what, at, into) = match &st.kind {
            // `against` is not the body rule: it says where a part *stands*, and is read by the
            // placement walk after every solid is built
            StmtKind::SolidRel(r) if r.word != crate::syntax::BodyWord::Against => {
                (r.word, &r.what, r.span, &r.body)
            }
            StmtKind::Relation(r) if is_body_on(sk, res, r) => {
                let w = r.form.written().expect("`is_body_on` read the operands");
                (crate::syntax::BodyWord::On, &w.ops[0], st.span, &w.ops[1])
            }
            _ => continue,
        };
        let mut say = |span: Span, m: String| {
            diags.push(Diag { code: Code::E080, span, stmt: Some(st.id), message: m });
        };
        let (Some(a), Some(b)) = (res.lookup(what), res.lookup(into)) else {
            let miss = if res.lookup(what).is_none() { what } else { into };
            diags.push(Diag {
                code: Code::E101,
                span: miss.span,
                stmt: Some(st.id),
                message: format!("no such entity: `{}`", miss.root.text),
            });
            continue;
        };
        let resolved = super::resolve::follow_building(sk,res,a,what)
            .and_then(|a| super::resolve::follow_building(sk,res,b,into).map(|b| (a,b)));
        let (a,b) = match resolved {
            Ok(pair) => pair,
            Err(message) => { say(at,message); continue; }
        };
        if a.kind != EntKind::Solid || b.kind != EntKind::Solid {
            let bad = if a.kind != EntKind::Solid { (what, a) } else { (into, b) };
            say(
                at,
                format!(
                    "`{}` relates solids, and `{}` is a {}",
                    word.as_str(),
                    bad.0.root.text,
                    bad.1.kind.as_str()
                ),
            );
            continue;
        }
        if a.idx == b.idx {
            say(at, format!("`{}` is {} itself", into.root.text, word.as_str()));
            continue;
        }
        let Some(sol) = sk.solids.get_mut(b.i()) else { continue };
        match &mut sol.def {
            SolidDef::Body { on, through, bound, .. } => match word {
                crate::syntax::BodyWord::On => on.push(a.idx),
                crate::syntax::BodyWord::Cut => through.push(a.idx),
                crate::syntax::BodyWord::Bound => bound.push(a.idx),
                crate::syntax::BodyWord::Against => unreachable!("filtered above"),
            },
            _ => {
                // a swept solid is what its brackets say; a body is what its statements say.
                // Naming the first in the second would make one statement mean two things
                say(
                    at,
                    format!(
                        "`{}` is a face swept, and only a body takes features: give it a stock \
                         (`solid {name}({name}_stock)`) and write them there",
                        into.root.text,
                        name = into.root.text
                    ),
                );
            }
        }
    }
    // -- where the parts stand (§6.10) ----------------------------------------
    // **After the solids and before anything evaluates them**: a face's ordinate along its
    // plane's normal is the sweep's own number and does not depend on where the plane stands, so
    // the walk can be done on the statements alone — and every reader below (a view, a mesh, a
    // claim) resolves its term lazily and therefore sees the planes placed.
    place(sk, res, body, skip, diags);

    // -- the pictures the document asks for (§6.11) ---------------------------
    for st in body {
        let StmtKind::Derived(d) = &st.kind else { continue };
        if skip.contains(&st.id) {
            continue;
        }
        let mut say = |code: Code, span: Span, m: String| {
            diags.push(Diag { code, span, stmt: Some(st.id), message: m });
        };
        let find = |r: &crate::syntax::Ref, want: EntKind| match res.lookup(r) {
            Some(e) if e.kind == want => Ok(e.idx),
            Some(e) => Err((
                Code::E040,
                r.span,
                format!(
                    "a {} is asked of a {}, and `{}` is a {}",
                    if want == EntKind::Solid { "picture" } else { "view" },
                    want.as_str(),
                    r.root.text,
                    e.kind.as_str()
                ),
            )),
            None => Err((Code::E101, r.span, format!("no such entity: `{}`", r.root.text))),
        };
        let solid = match find(&d.solid, EntKind::Solid) {
            Ok(i) => i,
            Err((c, sp, m)) => {
                say(c, sp, m);
                continue;
            }
        };
        let plane = match find(&d.plane, EntKind::Plane) {
            Ok(i) => i,
            Err((c, sp, m)) => {
                say(c, sp, m);
                continue;
            }
        };
        let at = match &d.at {
            None => None,
            Some(r) => match find(r, EntKind::Plane) {
                Ok(i) => Some(i),
                Err((c, sp, m)) => {
                    say(c, sp, m);
                    continue;
                }
            },
        };
        // **a section is drawn in a view parallel to the cut**, or the true shape it shows is
        // not the shape it is a section of
        if let Some(a) = at {
            let (pa, pb) = (sk.basis(a as usize), sk.basis(plane as usize));
            if crate::plane::fold_line(&pa, &pb).is_some() {
                say(
                    Code::E084,
                    d.span,
                    "a section is drawn in a view parallel to the plane it is cut at".into(),
                );
                continue;
            }
        }
        sk.derived.push(crate::model::DerivedE {
            solid,
            plane: Some(plane),
            at,
            dims: d.dims,
            name: d.name.key().text.clone(),
            class: d.class.clone(),
        });
        map.record(st, Made::Gauge);
    }

    // **a body may not be made of itself** — the term walk would not terminate, and the
    // document says something that is not about any object (§6.9)
    for i in 0..sk.solids.len() {
        if reaches(sk, i as u32, i as u32) {
            let name = sk.solids[i].name.clone();
            let at = body
                .iter()
                .find(|st| {
                    matches!(&st.kind, StmtKind::Decl(d)
                                    if d.kind == EntKind::Solid && d.name.key().text == name)
                })
                .map(|st| st.span)
                .unwrap_or_default();
            diags.push(Diag {
                code: Code::E041,
                span: at,
                stmt: None,
                message: format!("`{name}` is made of itself"),
            });
            // left standing but emptied, so nothing below it walks the cycle
            sk.solids[i].def =
                SolidDef::Body { stock: i as u32, on: Vec::new(), through: Vec::new(), bound: Vec::new() };
        }
    }
}

/// Does `from` reach `goal` through its operands?  The guard on the term walk, and the one thing
/// a document can write that has no object behind it.
fn reaches(sk: &Sketch, from: u32, goal: u32) -> bool {
    let mut pending = vec![from];
    let mut seen = BTreeSet::new();
    while let Some(i) = pending.pop() {
        if !seen.insert(i) { continue; }
        if sk.solids.get(i as usize).is_some() {
            let Ok(operands) = crate::solid::evaluation_operands(sk, i as usize) else { return true };
            for o in operands {
                if o == goal { return true; }
                pending.push(o);
            }
        }
    }
    false
}

/// Unbind a failed declaration and names that resolve to the same entity, so later
/// references cannot address another entity that inherited its index.
fn res_forget(res: &mut Resolver, name: &str) {
    if let Some(gone) = res.of.remove(name) {
        for e in res.of.values_mut() {
            if e.kind == gone.kind && e.idx > gone.idx {
                e.idx -= 1;
            }
        }
    }
}
