//! Build faces and solids, then apply solid claims and placement.

use super::resolve::Resolver;
use super::{Code, Diag, Made, SourceMap};
use crate::ir::{Decl, Kid, Operation as StmtKind, Statement as Stmt};
use crate::model::{EntKind, EntRef, Extent, Sense, Sketch, SolidDef};
use crate::style::Classes;
use crate::syntax::{Span, StmtId};
use std::collections::{BTreeMap, BTreeSet};

mod claims;
mod faces;
mod build;
pub(super) use claims::solid_claims;
use faces::{chain_face,validate_chain,build_face};
use build::build_solid;
pub(super) use build::carry_legs;

/// Build faces and solids in dependency order, then fold body operations into stock solids.
pub(super) fn solids(
    sk: &mut Sketch,
    res: &mut Resolver,
    map: &mut SourceMap,
    body: &[&Stmt],
    skip: &BTreeSet<StmtId>,
    probes: &mut super::regions::Probes,
    diags: &mut Vec<Diag>,
) {
    let has = body.iter().any(|st| {
        (matches!(&st.kind, StmtKind::Decl(d) if d.kind.spatial())
            || matches!(&st.kind, StmtKind::Chain(_))) && !skip.contains(&st.id)
    });
    if !has {
        return;
    }
    faces(sk, res, map, body, skip, diags);
    build_solids(sk, res, map, body, skip, probes, diags);
    body_rule(sk, res, map, body, skip, diags);
    derived(sk, res, map, body, skip, diags);
    made_of_itself(sk, body, diags);
}

/// Every face, a chain closed into one included: bound and recorded with the lines it made,
/// or forgotten with them.
fn faces(
    sk: &mut Sketch,
    res: &mut Resolver,
    map: &mut SourceMap,
    body: &[&Stmt],
    skip: &BTreeSet<StmtId>,
    diags: &mut Vec<Diag>,
) {
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
        if d.kind != EntKind::Face || skip.contains(&st.id) {
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
}

/// Every solid, bound and recorded with the inline section it owns and that section's closing
/// lines, or forgotten with them.
fn build_solids(
    sk: &mut Sketch,
    res: &mut Resolver,
    map: &mut SourceMap,
    body: &[&Stmt],
    skip: &BTreeSet<StmtId>,
    probes: &mut super::regions::Probes,
    diags: &mut Vec<Diag>,
) {
    for st in body {
        let StmtKind::Decl(d) = &st.kind else { continue };
        if d.kind != EntKind::Solid || skip.contains(&st.id) {
            continue;
        }
        let name = d.name.key().text.clone();
        let first_face = sk.faces.len();
        let first_line = sk.lines.len();
        match build_solid(sk, res, d, st, probes, diags) {
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
}

/// **The body rule**: `bore cut cyl`, `boss union cyl`, folded into the body they name.
fn body_rule(
    sk: &mut Sketch,
    res: &mut Resolver,
    map: &mut SourceMap,
    body: &[&Stmt],
    skip: &BTreeSet<StmtId>,
    diags: &mut Vec<Diag>,
) {
    // `bore cut cyl`, `boss union cyl`, folded into the body they name.  **Both are sets**, so
    // the order this walk meets them in cannot matter, and a document may write them anywhere.
    let mut stated = Vec::new();
    for st in body {
        if skip.contains(&st.id) {
            continue;
        }
        let StmtKind::SolidRel(r) = &st.kind else { continue };
        match body_operands(sk, res, r) {
            Ok((a, b)) => stated.push((*st, r.word, r.span, a, b)),
            Err((code, span, m)) => diags.push(Diag::within(code, span, st.id, m)),
        }
    }
    // **an empty body** (`solid()`, its own stock) is its first `union` and the rest of them, so
    // a pattern joins every copy alike and none is the stock written apart
    let (mut taken, mut empty) = (BTreeSet::new(), BTreeSet::new());
    for i in 0..sk.solids.len() {
        let SolidDef::Body { stock, on, .. } = &sk.solids[i].def else { continue };
        if *stock as usize != i || !on.is_empty() {
            continue;
        }
        let first = stated.iter()
            .position(|(_, word, _, _, b)| *word == crate::syntax::BodyWord::Union && b.i() == i);
        match first {
            Some(k) => {
                let SolidDef::Body { stock, .. } = &mut sk.solids[i].def else { unreachable!() };
                *stock = stated[k].3.idx;
                taken.insert(k);
            }
            None => {
                empty.insert(i);
                let name = sk.solids[i].name.clone();
                diags.push(Diag::new(
                    Code::E080,
                    declared_at(body, &name),
                    None,
                    format!("`{name}` is an empty body, and nothing is in `union` with it: \
                             write `x union {name}` for its material"),
                ));
            }
        }
    }
    for (k, (st, word, at, a, b)) in stated.into_iter().enumerate() {
        // the stock taken, and a body already refused as empty, join nothing more
        if taken.contains(&k) || empty.contains(&b.i()) {
            continue;
        }
        // **a swept solid that takes a feature is the body over its own sweep**: the name keeps
        // its index, so everything already reading it (a `through:` extent, a body's operand, a
        // view) reads the whole object, and the sweep moves to a stock of the same name, so a
        // face is called what it was (`plate.near`) whether or not anything stands on it
        if let Some(stock) = sk.as_body(b.i()) {
            map.also_made(b, EntRef::solid(stock));
        }
        // **the statement that would close a cycle is the one refused**, where it is written —
        // so a gesture writing it is refused at the gesture (#162 F4) — and the body is left as
        // the statements before it made it
        if let Err(why) = sk.join_body(b.i(), word, a.i()) {
            diags.push(Diag::within(Code::E041, at, st.id, why));
        }
    }
}

/// The two solids a body statement names, `what` and the body it joins — or why it names none.
fn body_operands(sk: &Sketch, res: &Resolver, r: &crate::syntax::SolidRel)
    -> Result<(EntRef, EntRef), (Code, Span, String)> {
    let (what, into, at) = (&r.what, &r.body, r.span);
    let (Some(a), Some(b)) = (res.lookup(what), res.lookup(into)) else {
        let miss = if res.lookup(what).is_none() { what } else { into };
        return Err((Code::E101, miss.span, format!("no such entity: `{}`", miss.root.text)));
    };
    // `A.body`: the solid a face path is of (#183 F5), `pad.boss.body` is `pad`
    let follow = |e: EntRef, r: &crate::syntax::Ref| match res.operand(sk, r) {
        Ok(super::resolve::Operand::Body(solid)) => Ok(solid),
        _ => super::resolve::follow_building(sk,res,e,r),
    };
    let (a,b) = follow(a,what).and_then(|a| follow(b,into).map(|b| (a,b)))
        .map_err(|m| (Code::E080, at, m))?;
    if a.kind != EntKind::Solid || b.kind != EntKind::Solid {
        let bad = if a.kind != EntKind::Solid { (what, a) } else { (into, b) };
        return Err((Code::E080, at, format!("`{}` relates solids, and `{}` is {}",
            r.word.as_str(), bad.0.root.text, bad.1.kind.a())));
    }
    if a.idx == b.idx {
        return Err((Code::E080, at, format!("`{0} {1} {0}` joins a solid to itself", into.root.text,
                                            r.word.as_str())));
    }
    Ok((a, b))
}

/// Where the declaration of the solid `name` is written, or nowhere.
fn declared_at(body: &[&Stmt], name: &str) -> Span {
    body.iter()
        .find(|st| matches!(&st.kind, StmtKind::Decl(d)
                            if d.kind == EntKind::Solid && d.name.key().text == name))
        .map(|st| st.span)
        .unwrap_or_default()
}

/// The pictures the document asks for (§6.11): views and sections of a solid in a plane.
fn derived(
    sk: &mut Sketch,
    res: &mut Resolver,
    map: &mut SourceMap,
    body: &[&Stmt],
    skip: &BTreeSet<StmtId>,
    diags: &mut Vec<Diag>,
) {
    for st in body {
        let StmtKind::Derived(d) = &st.kind else { continue };
        if skip.contains(&st.id) {
            continue;
        }
        let mut say = |code: Code, span: Span, m: String| {
            diags.push(Diag::within(code, span, st.id, m));
        };
        let find = |r: &crate::syntax::Ref, want: EntKind| match res.lookup(r) {
            Some(e) if e.kind == want => Ok(e.idx),
            Some(e) => Err((
                Code::E040,
                r.span,
                format!(
                    "a {} is asked of {}, and `{}` is {}",
                    if want == EntKind::Solid { "picture" } else { "view" },
                    want.a(),
                    r.root.text,
                    e.kind.a()
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
        let plane = match find(&d.plane, EntKind::Frame) {
            Ok(i) => i,
            Err((c, sp, m)) => {
                say(c, sp, m);
                continue;
            }
        };
        let at = match &d.at {
            None => None,
            Some(r) => match find(r, EntKind::Frame) {
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
}

/// **A body may not be made of itself** — the term walk would not terminate, and the
/// document says something that is not about any object (§6.9).
fn made_of_itself(sk: &mut Sketch, body: &[&Stmt], diags: &mut Vec<Diag>) {
    for i in 0..sk.solids.len() {
        // an empty body nothing joined is said where it is written (`body_rule`)
        if matches!(&sk.solids[i].def, SolidDef::Body { stock, .. } if *stock as usize == i) {
            continue;
        }
        if crate::solid::reaches(sk, i as u32, i as u32) {
            let name = sk.solids[i].name.clone();
            let at = declared_at(body, &name);
            diags.push(Diag::new(
                Code::E041,
                at,
                None,
                format!("`{name}` is made of itself"),
            ));
            // left standing but emptied, so nothing below it walks the cycle
            sk.solids[i].def =
                SolidDef::Body { stock: i as u32, on: Vec::new(), through: Vec::new(), bound: Vec::new() };
        }
    }
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
