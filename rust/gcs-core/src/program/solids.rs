//! Build faces and solids, then apply solid claims and placement.

use super::resolve::Resolver;
use super::{Code, Diag, Made, SourceMap};
use crate::ir::{Decl, Kid, Operation as StmtKind, Relation, Statement as Stmt};
use crate::model::{EntKind, EntRef, Extent, Sense, Sketch, SolidDef};
use crate::style::Classes;
use crate::syntax::{Span, StmtId};
use std::collections::{BTreeMap, BTreeSet};

/// A named closed traversal uses the existing face representation. Its joints have
/// already shared endpoints, so face construction must not manufacture a closing edge.
fn chain_face(c: &crate::syntax::NamedChain) -> Decl {
    Decl {
        kind: EntKind::Face, name: c.name.clone(),
        children: vec![c.links.iter().cloned().map(Kid::Ref).collect()],
        seed: Vec::new(), seed_text: Vec::new(), seed_spans: Vec::new(),
        unseeded: false, seed_explicit: Vec::new(), closed: false, knots: None,
        curve: None, computed: None, class: Classes::default(), seed_at: None,
        seed_names: Vec::new(), attitude: crate::syntax::Attitude::Page,
        sweep: None, membership: crate::syntax::Membership::default(),
    }
}

fn validate_chain(
    sk: &Sketch,
    res: &Resolver,
    c: &crate::syntax::NamedChain,
    st: &Stmt,
    diags: &mut Vec<Diag>,
) -> bool {
    let mut previous = None;
    for r in &c.links {
        let e = res.lookup(r).filter(|e| e.i() < sk.count(e.kind))
            .and_then(|e| super::resolve::follow(sk, e, &r.path).ok());
        let pair = e.filter(|e| matches!(e.kind, EntKind::Line | EntKind::Arc))
            .and_then(|e| crate::model::edge_ends(sk, e));
        let Some((start, end)) = pair else {
            diags.push(Diag {
                code: Code::E080, span: r.span, stmt: Some(st.id),
                message: format!("a named chain traverses lines and arcs; `{}` is not one",
                    crate::syntax::ref_text(r)),
            });
            return false;
        };
        if previous.is_some_and(|p| p != start) {
            diags.push(Diag {
                code: Code::E080, span: r.span, stmt: Some(st.id),
                message: format!("`{}` does not share the preceding link's endpoint in chain `{}`",
                    crate::syntax::ref_text(r), c.name.key().text),
            });
            return false;
        }
        previous = Some(end);
    }
    true
}

/// Expanded references keep the source's last field as the boundary's public name.
fn boundary_name(r: &crate::syntax::Ref) -> &str {
    match r.path.last() {
        Some(crate::syntax::Seg::Field(n)) => &n.text,
        _ => r.root.text.rsplit('.').next().unwrap_or(&r.root.text),
    }
}

fn fresh_boundary_name(prefix: &str, next: &mut usize, reserved: &BTreeSet<&str>) -> String {
    loop {
        let name = format!("{prefix}{next}");
        *next += 1;
        if !reserved.contains(name.as_str()) {
            return name;
        }
    }
}

/// Apply solid claims after body construction, preserving statement spans for diagnostics.
pub(super) fn solid_claims(
    sk: &mut Sketch,
    res: &Resolver,
    map: &mut SourceMap,
    body: &[&Stmt],
    skip: &BTreeSet<StmtId>,
    diags: &mut Vec<Diag>,
) {
    // A claim over a sweep is the *same* claims with an interval attached, so both forms go
    // through one reader: the block says how its body is judged and asserts nothing itself.
    for st in body {
        match &st.kind {
            StmtKind::Relation(_) => solid_claim(sk, res, st, None, map, skip, diags),
            StmtKind::ClaimOver(c) if !skip.contains(&st.id) => {
                let over = match sweep_of_claim(sk, res, c, st, diags) {
                    Some(o) => o,
                    None => continue,
                };
                for inner in &c.body {
                    solid_claim(sk, res, inner, Some(&over), map, skip, diags);
                }
                map.record(st, Made::Gauge);
            }
            _ => {}
        }
    }
}

fn solid_claim(
    sk: &mut Sketch,
    res: &Resolver,
    st: &Stmt,
    over: Option<&crate::model::Sweep>,
    map: &mut SourceMap,
    skip: &BTreeSet<StmtId>,
    diags: &mut Vec<Diag>,
) {
    let StmtKind::Relation(r) = &st.kind else { return };
    if skip.contains(&st.id) {
        return;
    }
    let Some(w) = r.form.written() else { return };
    let Some(word) = crate::constraints::solid_word(&w.word.text) else { return };
    let mut say = |code: Code, span: Span, m: String| {
        diags.push(Diag { code, span, stmt: Some(st.id), message: m });
    };
    // **it is a claim whether or not it says so.**  These words cannot act — there is no row for
    // them — so a document that writes one without `claim` is saying the same thing, and refusing
    // it would be a rule about spelling rather than about meaning.  Written *with* `claim` is the
    // reading to prefer, and the printer spells it that way.
    if w.ops.len() != 2 {
        say(Code::E040, st.span, format!("`{}` relates two solids", word.as_str()));
        return;
    }
    let mut ends = Vec::new();
    for o in &w.ops {
        match res.lookup(o) {
            Some(e) if e.kind == EntKind::Solid => ends.push(e.idx),
            Some(e) => {
                say(
                    Code::E040,
                    o.span,
                    format!(
                        "`{}` relates solids, and `{}` is a {}",
                        word.as_str(),
                        o.root.text,
                        e.kind.as_str()
                    ),
                );
                return;
            }
            None => {
                say(Code::E101, o.span, format!("no such entity: `{}`", o.root.text));
                return;
            }
        }
    }
    let gap = match (word.takes_gap(), w.args.as_slice()) {
        (false, []) => Extent { text: String::new(), value: 0.0 },
        (true, [crate::syntax::OpArg::Dim(text, span)]) => {
            match crate::flatten::value_aff(text, &BTreeMap::new(), sk.units) {
                Ok(v) if v.c.is_finite()
                    && v.dim.require(crate::units::Dim::LENGTH, word.as_str()).is_ok() =>
                {
                    Extent { text: text.trim().to_string(), value: v.c }
                }
                _ => {
                    say(Code::E103, *span, format!("`{}` asks for a finite length", word.as_str()));
                    return;
                }
            }
        }
        (true, []) => {
            say(Code::E040, st.span,
                format!("`{}` asks for room: `{}(2mm)`", word.as_str(), word.as_str()));
            return;
        }
        _ => {
            say(Code::E040, st.span, format!("`{}` takes {}", word.as_str(),
                if word.takes_gap() { "exactly one length argument" } else { "no arguments" }));
            return;
        }
    };
    sk.solid_claims.push(crate::model::SolidClaim {
        word,
        a: ends[0],
        b: ends[1],
        gap,
        over: over.cloned(),
        stmt: st.id.0,
    });
    map.record(st, Made::Gauge);
}

/// Resolve stack placement from bearing faces (§6.10), diagnosing inconsistent or
/// missing placement of planes with unspecified offsets.
fn place(
    sk: &mut Sketch,
    res: &Resolver,
    body: &[&Stmt],
    skip: &BTreeSet<StmtId>,
    diags: &mut Vec<Diag>,
) {
    // every mate, resolved to (placed plane, datum plane, the offset it implies)
    struct Mate {
        stmt: StmtId,
        span: Span,
        placed: u32,
        datum: u32,
        /// The two faces' ordinates along their own planes' normals, and whether the normals
        /// agree.  Kept **as ordinates** and not as a finished offset: the datum's own offset may
        /// not be known yet — a washer between two parts stands on the first before the second
        /// stands on it — and a delta worked out at collection time reads a zero the walk was
        /// about to fill in.
        ordf: f64,
        ordg: f64,
        dot: f64,
        faces: [(u32, String); 2],
    }
    let mut mates: Vec<Mate> = Vec::new();
    for st in body {
        let StmtKind::SolidRel(r) = &st.kind else { continue };
        if r.word != crate::syntax::BodyWord::Against || skip.contains(&st.id) {
            continue;
        }
        let mut say = |code: Code, span: Span, m: String| {
            diags.push(Diag { code, span, stmt: Some(st.id), message: m });
        };
        let face = |rf: &crate::syntax::Ref| -> Result<(u32, f64, f64, String, u32), (Code, Span, String)> {
            let Some(e) = res.lookup(rf) else {
                return Err((Code::E101, rf.span, format!("no such entity: `{}`", rf.root.text)));
            };
            if e.kind != EntKind::Solid {
                return Err((
                    Code::E083,
                    rf.span,
                    format!(
                        "`against` mates faces of solids, and `{}` is a {}",
                        rf.root.text,
                        e.kind.as_str()
                    ),
                ));
            }
            let path: Vec<String> = rf
                .path
                .iter()
                .map(|seg| match seg {
                    crate::syntax::Seg::Field(n) => n.text.clone(),
                    other => format!("{other:?}"),
                })
                .collect();
            face_ordinate(sk, e.idx, &path).map(|(p, o, s, path)| (p, o, s, path, e.idx)).ok_or_else(|| {
                (
                    Code::E082,
                    rf.span,
                    format!(
                        "`{}` names no flat face of `{}` that a stack could bear on: a mate is \
                         between the caps a sweep makes",
                        path.join("."),
                        rf.root.text
                    ),
                )
            })
        };
        let (f, g) = match (face(&r.what), face(&r.body)) {
            (Ok(f), Ok(g)) => (f, g),
            (Err((c, sp, m)), _) | (_, Err((c, sp, m))) => {
                say(c, sp, m);
                continue;
            }
        };
        let (pf, ordf, sf, pathf, solidf) = f;
        let (pg, ordg, sg, pathg, solidg) = g;
        let (bf, bg) = (sk.planes[pf as usize].basis, sk.planes[pg as usize].basis);
        let _ = bg;
        // **parallel, and facing each other**: two faces in contact share a normal and point
        // opposite ways along it, which is what "against" means and what a stack needs
        let dot = crate::plane::dot(bf.normal(), bg.normal());
        if (dot.abs() - 1.0).abs() > 1e-9 {
            say(Code::E083, r.span, "`against` mates parallel faces".into());
            continue;
        }
        // each face's outward direction, as a sign along the *datum's* normal
        let out_f = sf * dot;
        if out_f * sg > 0.0 {
            say(
                Code::E083,
                r.span,
                "`against` mates faces that look at each other: these look the same way".into(),
            );
            continue;
        }
        mates.push(Mate {
            stmt: st.id, span: r.span, placed: pf, datum: pg, ordf, ordg, dot,
            faces: [(solidf, pathf), (solidg, pathg)],
        });
    }
    if mates.is_empty() && sk.placed_planes.is_empty() {
        return;
    }
    // **one mate places one plane**: none and it stands nowhere, two and the document says two
    // things about one number
    let mut by_plane: BTreeMap<u32, Vec<usize>> = BTreeMap::new();
    for (i, m) in mates.iter().enumerate() {
        by_plane.entry(m.placed).or_default().push(i);
    }
    for (p, ms) in &by_plane {
        if ms.len() > 1 {
            diags.push(Diag {
                code: Code::E083,
                span: mates[ms[1]].span,
                stmt: Some(mates[ms[1]].stmt),
                message: format!(
                    "`{}` is placed twice: a plane stands where one thing bears on it",
                    sk.plane_name(*p as usize)
                ),
            });
        }
        if !sk.placed_planes.contains(p) {
            diags.push(Diag {
                code: Code::E083,
                span: mates[ms[0]].span,
                stmt: Some(mates[ms[0]].stmt),
                message: format!(
                    "`{}` already says where it stands: a mate places a plane written \
                     `from: … ` with no `fold:` and no `offset:`",
                    sk.plane_name(*p as usize)
                ),
            });
        }
    }
    for p in sk.placed_planes.clone() {
        if !by_plane.contains_key(&p) {
            diags.push(Diag {
                code: Code::E083,
                span: Span::default(),
                stmt: None,
                message: format!(
                    "`{}` is a plane nothing places: write `offset:` or state one `against`",
                    sk.plane_name(p as usize)
                ),
            });
        }
    }
    // Both derivations and mates are placement dependencies. A child offset (or fold) must
    // inherit its parent's final origin, and a mate using that child must wait for it.
    let mut derived = BTreeMap::new();
    for st in body {
        let StmtKind::Decl(d) = &st.kind else { continue };
        if d.kind != EntKind::Plane || skip.contains(&st.id) { continue; }
        let parent = match &d.attitude {
            crate::syntax::Attitude::Offset { plane, .. } |
            crate::syntax::Attitude::From { plane, .. } => plane,
            _ => continue,
        };
        if let (Some(child), Some(parent)) = (res.of.get(&d.name.key().text), res.lookup(parent)) {
            if !sk.placed_planes.contains(&child.idx) {
                let cb = sk.planes[child.i()].basis;
                let pb = sk.planes[parent.i()].basis;
                derived.insert(child.idx, (parent.idx, [cb.o[0] - pb.o[0], cb.o[1] - pb.o[1], cb.o[2] - pb.o[2]]));
            }
        }
    }
    let mut done: BTreeSet<u32> = (0..sk.planes.len() as u32)
        .filter(|p| !sk.placed_planes.contains(p) && !derived.contains_key(p)).collect();
    let mut left: Vec<usize> = (0..mates.len()).collect();
    while !left.is_empty() || !derived.is_empty() {
        let children: Vec<_> = derived.iter().filter(|(_, (parent, _))| done.contains(parent))
            .map(|(&child, &value)| (child, value)).collect();
        for (child, (parent, delta)) in &children {
            let origin = sk.planes[*parent as usize].basis.o;
            sk.planes[*child as usize].basis.o = std::array::from_fn(|k| origin[k] + delta[k]);
            done.insert(*child);
            derived.remove(child);
        }
        let ready: Vec<usize> =
            left.iter().copied().filter(|&i| done.contains(&mates[i].datum)).collect();
        if ready.is_empty() && children.is_empty() {
            for i in &left {
                diags.push(Diag {
                    code: Code::E041,
                    span: mates[*i].span,
                    stmt: Some(mates[*i].stmt),
                    message: format!(
                        "`{}` stands on what stands on it",
                        sk.plane_name(mates[*i].placed as usize)
                    ),
                });
            }
            break;
        }
        for i in ready {
            let m = &mates[i];
            // f sits at `off(Pf) + ordf` along Pf's own normal; measured along Pg's that is
            // `dot·(off(Pf) + ordf)`, and contact makes it equal to `off(Pg) + ordg` — read
            // *now*, with the datum's own offset already written
            let datum = sk.planes[m.datum as usize].basis.along_normal();
            let want = (datum + m.ordg) * m.dot - m.ordf;
            let b = sk.planes[m.placed as usize].basis;
            sk.planes[m.placed as usize].basis = b.offset(want - b.along_normal());
            done.insert(m.placed);
            left.retain(|&k| k != i);
        }
    }
    // Keep the contact checks until after solving; a seed can have a vanished face which
    // the final constraints restore, or the other way around.
    for m in &mates {
        for (solid, path) in &m.faces {
            sk.solid_bearings.push(crate::model::SolidBearing {
                solid: *solid, path: path.clone(), stmt: m.stmt.0, span: m.span,
            });
        }
    }
}

/// Resolve a solid face to its plane, normal ordinate, and facing direction,
/// including named faces reached through body operations.
fn face_ordinate(
    sk: &Sketch,
    mut solid: u32,
    mut path: &[String],
) -> Option<(u32, f64, f64, String)> {
    let mut parity = 1.0;
    let mut seen = BTreeSet::new();
    loop {
        if !seen.insert(solid) {
            return None;
        }
        let s = sk.solids.get(solid as usize)?;
        match &s.def {
            SolidDef::Prism { face, from, to } => {
                if path.len() != 1 {
                    return None;
                }
                let last = path.last()?;
                let (ord, sign) = match last.as_str() {
                    "near" => (from.value.max(to.value), 1.0),
                    "far" => (from.value.min(to.value), -1.0),
                    _ => return None,
                };
                return Some((
                    sk.faces.get(*face as usize)?.plane?,
                    ord,
                    sign * parity,
                    format!("{}.{last}", s.name),
                ));
            }
            SolidDef::Revolve { .. } => return None,
            SolidDef::Body { stock, on, through } => {
                // a body's faces are its operands', reached through the operand that made them
                let (head, rest) = path.split_first()?;
                let operand =
                    std::iter::once(stock).chain(on.iter()).chain(through.iter()).copied().find(
                        |&o| {
                            sk.solids.get(o as usize).is_some_and(|x| {
                                &x.name == head || x.name.rsplit('.').next() == Some(head.as_str())
                            })
                        },
                    );
                if let Some(o) = operand {
                    if through.contains(&o) {
                        parity = -parity;
                    }
                    solid = o;
                    path = rest;
                } else if path.len() == 1 {
                    solid = *stock;
                } else {
                    return None;
                }
            }
        }
    }
}

/// The interval a swept claim runs over: a free variable of the drawing, and where it goes.
fn sweep_of_claim(
    sk: &Sketch,
    res: &Resolver,
    c: &crate::ir::ClaimOver,
    st: &Stmt,
    diags: &mut Vec<Diag>,
) -> Option<crate::model::Sweep> {
    let mut say = |code: Code, span: Span, m: String| {
        diags.push(Diag { code, span, stmt: Some(st.id), message: m });
    };
    let name = crate::syntax::ref_text(&c.formal);
    // **a free variable, and nothing else**: a `param` is a number the document already fixed,
    // and sweeping it would be sweeping a constant
    if !sk.free_vars.contains_key(&name) {
        if res.lookup(&c.formal).is_some() {
            say(Code::E040, c.formal.span, format!("`{name}` is geometry, not a free variable"));
        } else {
            say(
                Code::E040,
                c.formal.span,
                format!(
                    "`{name}` is not a free variable of this drawing: a swept claim runs \
                         along an unknown the solver answers for"
                ),
            );
        }
        return None;
    }
    let dimension = *sk.free_dimensions.get(&name)?;
    // Free parameters are stored in user units. In particular, an angular unknown is in
    // degrees; its readers' affine coefficients own the radians conversion into kernels.
    // Check the inferred variable dimension before assigning either endpoint.
    let mut num = |a: &crate::syntax::Arg, what: &str| -> Option<f64> {
        let crate::syntax::Arg::Dim { text, span } = a else { return None };
        match crate::flatten::value_aff(text, &BTreeMap::new(), sk.units) {
            Ok(v) if v.c.is_finite() && v.dim.require(dimension, &name).is_ok() => Some(v.c),
            Ok(_) => {
                say(Code::E103, *span,
                    format!("`{what}` for `{name}` must be a finite {}", dimension.name()));
                None
            }
            Err(e) => {
                say(Code::E103, *span, format!("`{what}`: {e}"));
                None
            }
        }
    };
    let (from, to) = (num(&c.from, "from")?, num(&c.to, "to")?);
    Some(crate::model::Sweep { name, from, to })
}

/// `boss on cyl`: an `on` whose two operands are both solids.  Asked of the *resolver*, which
/// has known every declaration's kind since phase 1 — so the question is answered the same way
/// before the solids are built and after.
pub(super) fn is_body_on(res: &Resolver, r: &Relation) -> bool {
    let Some(w) = r.form.written() else { return false };
    if w.word.text != "on" || w.ops.len() != 2 {
        return false;
    }
    w.ops.iter().all(|o| res.lookup(o).map(|e| e.kind == EntKind::Solid).unwrap_or(false))
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
        if d.kind != EntKind::Face || skip.contains(&st.id) {
            continue;
        }
        let name = d.name.key().text.clone();
        let first_line = sk.lines.len();
        match build_face(sk, res, d, st.id, st.span, diags) {
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
    // `bore through cyl`, `boss on cyl`, folded into the body they name.  **Both are sets**, so
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
            StmtKind::Relation(r) if is_body_on(res, r) => {
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
            SolidDef::Body { on, through, .. } => match word {
                crate::syntax::BodyWord::On => on.push(a.idx),
                crate::syntax::BodyWord::Through => through.push(a.idx),
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
            let (pa, pb) = (sk.planes[a as usize].basis, sk.planes[plane as usize].basis);
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
                SolidDef::Body { stock: i as u32, on: Vec::new(), through: Vec::new() };
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
        if let Some(s) = sk.solids.get(i as usize) {
            for o in s.operands() {
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

/// Build a face's ordered boundary on one plane (§6.8).
/// Points introduce straight runs; existing edges take their direction from a neighbour
/// they meet.  Only `-> close` permits a gap from the last item back to the first.  Generated
/// lines carry `.closure`, hidden by the base sheet, and introduce no points or unknowns.
fn build_face(
    sk: &mut Sketch,
    res: &Resolver,
    d: &Decl,
    stmt: StmtId,
    span: Span,
    diags: &mut Vec<Diag>,
) -> Option<usize> {
    let mut fail = |span: Span, m: String| {
        diags.push(Diag { code: Code::E080, span, stmt: Some(stmt), message: m });
    };
    let mut refs = Vec::new();
    for k in d.children.first().into_iter().flatten() {
        let Kid::Ref(r) = k else {
            fail(span, "a face names the edges it is bounded by; a seed places a point".into());
            return None;
        };
        let chain = res.chains.get(&r.root.text).filter(|_| r.path.is_empty());
        let count = chain.map_or(1, |c| c.links.len());
        if count > crate::flatten::MAX_FLAT.saturating_sub(refs.len()) {
            fail(span, "a face expands to too many chain edges".into());
            return None;
        }
        if let Some(c) = chain {
            refs.extend(&c.links);
        } else {
            refs.push(r);
        }
    }
    if refs.is_empty() {
        fail(span, "a face is a loop of edges: `face f(ab, bc, cd, da)`".into());
        return None;
    }
    // what one item of the walk is: an edge, or a corner the loop goes straight to
    struct Item {
        entity: EntRef,
        name: String,
    }
    let reserved: BTreeSet<&str> = refs.iter().map(|r| boundary_name(r)).collect();
    let mut anonymous = 0;
    let mut items: Vec<Item> = Vec::with_capacity(refs.len());
    for r in &refs {
        let Some(e) = res.lookup(r) else {
            diags.push(Diag {
                code: Code::E101,
                span: r.span,
                stmt: Some(stmt),
                message: format!("no such entity: `{}`", r.root.text),
            });
            return None;
        };
        // **the leaf, not the absolute name.**  By the time a face is built the flattener has
        // rewritten `lid` into `cyl.lid`, and a face path is already prefixed by the solid it
        // belongs to — so keeping the whole thing spells `cyl.body.block.cyl.lid`, saying
        // "cylinder" twice about one face.  The leaf is what the source wrote.
        let leaf = boundary_name(r);
        let name = if leaf.starts_with('#') {
            fresh_boundary_name("edge", &mut anonymous, &reserved)
        } else {
            leaf.to_string()
        };
        match e.kind {
            EntKind::Line | EntKind::Arc | EntKind::Circle | EntKind::Point => {
                items.push(Item { entity: e, name });
            }
            _ => {
                fail(
                    r.span,
                    format!(
                        "a face is bounded by lines, arcs and circles and turns at points, and \
                         `{}` is a {}",
                        r.root.text,
                        e.kind.as_str()
                    ),
                );
                return None;
            }
        }
    }
    // **a circle is a loop by itself, and may not stand in one**
    let lone_circle = items.len() == 1 && items[0].entity.kind == EntKind::Circle;
    if !lone_circle && items.iter().any(|i| i.entity.kind == EntKind::Circle) {
        fail(span, "a circle is a whole loop: it stands in a face by itself".into());
        return None;
    }
    // **the ends of each item**, which is what says whether two neighbours already meet.  A
    // point is both of its own.
    let mut ends = Vec::with_capacity(items.len());
    for it in items.iter().filter(|_| !lone_circle) {
        let e = it.entity;
        let pair = if e.kind == EntKind::Point {
            Some((e.idx, e.idx))
        } else {
            crate::model::edge_ends(sk, e)
        };
        let Some(pair) = pair else {
            fail(span, format!("`{}` has no ends: a face is a loop, walked in order", it.name));
            return None;
        };
        ends.push(pair);
    }
    let n = items.len();
    // **one item is a loop only when it is a circle** — said here, so that a lone point or a
    // lone line is refused for what it is rather than as a gap between an item and itself
    if n == 1 && !lone_circle {
        fail(
            span,
            format!(
                "`{}` is not a loop by itself: a face is a loop of edges and the corners \
                 between them",
                items[0].name
            ),
        );
        return None;
    }
    let contains = |i: usize, p: u32| ends[i].0 == p || ends[i].1 == p;
    // Whether each item shares an endpoint with its successor, including the wrap.
    let meets: Vec<bool> = (0..ends.len())
        .map(|i| contains((i + 1) % n, ends[i].0) || contains((i + 1) % n, ends[i].1))
        .collect();
    // An edge with no meeting neighbour has two unstated readings.  Refuse it before
    // choosing directions, so inserting closing lines cannot choose a shape by accident.
    let walked = ends.len();
    for i in 0..walked {
        if items[i].entity.kind != EntKind::Point && !meets[(i + n - 1) % n] && !meets[i] {
            fail(
                span,
                format!(
                    "`{}` meets neither of its neighbours: a face is a loop, walked in order",
                    items[i].name
                ),
            );
            return None;
        }
    }
    // Walk actual endpoints, not merely shared sets of endpoints.  Both ends can be shared
    // (an arc and its chord), so try either direction of the first item; each later direction
    // is fixed by the preceding exit or by the next neighbour when there is a gap.
    let orient = |reverse_first: bool| -> Result<Vec<(u32, u32)>, usize> {
        let mut walk: Vec<(u32, u32)> = Vec::with_capacity(walked);
        for i in 0..walked {
            let (a, b) = ends[i];
            let prev = (i + n - 1) % n;
            let next = (i + 1) % n;
            let candidates =
                if i == 0 && reverse_first { [(b, a), (a, b)] } else { [(a, b), (b, a)] };
            let (from, to) = candidates
                .into_iter()
                .find(|&(from, to)| {
                    let arrives = !meets[prev]
                        || if i == 0 { contains(prev, from) } else { walk[i - 1].1 == from };
                    let leaves = !meets[i] || contains(next, to);
                    arrives && leaves
                })
                .ok_or(i)?;
            walk.push((from, to));
        }
        if walked > 0 && meets[n - 1] && walk[n - 1].1 != walk[0].0 {
            return Err(n - 1);
        }
        Ok(walk)
    };
    let walk = match orient(false).or_else(|_| orient(true)) {
        Ok(walk) => walk,
        Err(i) => {
            fail(
                span,
                format!(
                    "`{}` and its neighbours share no point along the walk: a face must \
                     enter and leave each edge in order",
                    items[i].name
                ),
            );
            return None;
        }
    };
    // -- the walk, and the straight runs it mints ------------------------------------------
    let mut edges: Vec<EntRef> = Vec::with_capacity(n);
    let mut names: Vec<String> = Vec::with_capacity(n);
    let reserved: BTreeSet<&str> =
        items.iter().filter(|i| i.entity.kind != EntKind::Point).map(|i| i.name.as_str()).collect();
    let mut minted = 0usize;
    for i in 0..n {
        if items[i].entity.kind != EntKind::Point {
            edges.push(items[i].entity);
            names.push(items[i].name.clone());
        }
        if lone_circle {
            continue;
        }
        let j = (i + 1) % n;
        let (from, to) = (walk[i].1, walk[j].0);
        if from == to {
            continue;
        }
        // a gap.  The wrap is minted only where `-> close` says so, and an interior one only
        // where a *point* is one of its sides
        if j == 0 {
            if !d.closed {
                fail(
                    span,
                    format!(
                        "`{}` and `{}` share no point: a face is a loop, and one that does not \
                         come back to where it started closes with `-> close`",
                        items[i].name, items[j].name
                    ),
                );
                return None;
            }
        } else if items[i].entity.kind != EntKind::Point && items[j].entity.kind != EntKind::Point {
            fail(
                span,
                format!(
                    "`{}` and `{}` share no point: a face is a loop, walked in order",
                    items[i].name, items[j].name
                ),
            );
            return None;
        }
        let li = sk.line(from as usize, to as usize);
        sk.lines[li].class = Classes::one("closure");
        edges.push(EntRef::line(li));
        // A generated side must not merge with a named side in reports or face selection.
        names.push(fresh_boundary_name("close", &mut minted, &reserved));
    }
    // **three corners, or a curve.**  A loop of straight runs between two points is a line
    // drawn twice, and a face with no area is a solid with no volume — worth saying here,
    // where there is a span, rather than letting the boundary evaluation quietly find nothing.
    if !edges.iter().any(|e| matches!(e.kind, EntKind::Arc | EntKind::Circle)) {
        let mut corners: Vec<u32> = edges
            .iter()
            .filter_map(|e| crate::model::edge_ends(sk, *e))
            .flat_map(|(a, b)| [a, b])
            .collect();
        corners.sort_unstable();
        corners.dedup();
        if corners.len() < 3 {
            fail(span, "a face is a loop, and a straight one needs three corners".into());
            return None;
        }
    }
    // **one plane**, read off the memberships and never written on the face
    let mut plane: Option<Option<u32>> = None;
    for (e, n) in edges.iter().zip(&names) {
        for c in sk.children(*e).into_iter().chain([*e]) {
            if c.kind != EntKind::Point {
                continue;
            }
            let p = sk.plane_of(c.i()).map(|x| x as u32);
            match plane {
                None => plane = Some(p),
                Some(q) if q == p => {}
                Some(q) => {
                    let say = |x: Option<u32>| match x {
                        Some(i) => format!("view {i}"),
                        None => "the page".to_string(),
                    };
                    fail(
                        span,
                        format!(
                            "a face lies in one plane, and `{n}` is on {} where the loop is on {}",
                            say(p),
                            say(q)
                        ),
                    );
                    return None;
                }
            }
        }
    }
    let i = sk.face(edges, names, &d.name.key().text);
    sk.faces[i].plane = plane.flatten();
    sk.faces[i].class = d.class.clone();
    Some(i)
}

/// **A solid is a face swept, or a term over other solids** (§6.9).
fn build_solid(
    sk: &mut Sketch,
    res: &Resolver,
    d: &Decl,
    st: &Stmt,
    diags: &mut Vec<Diag>,
) -> Option<usize> {
    let kids = d.children.first().map(Vec::as_slice).unwrap_or(&[]);
    let mut ops: Vec<EntRef> = Vec::new();
    for k in kids {
        let e = match k {
            Kid::Ref(r) if !r.path.is_empty() && res.chains.contains_key(&r.root.text) => {
                diags.push(Diag {
                    code: Code::E080, span: r.span, stmt: Some(st.id),
                    message: format!("`{}` names a chain, not a member of that chain; \
                        its named links belong to the enclosing component", r.root.text),
                });
                return None;
            }
            Kid::Face { decl: face, span } => {
                EntRef::face(build_face(sk, res, face, st.id, *span, diags)?)
            }
            Kid::Ref(r) if res.chains.get(&r.root.text).is_some_and(|c| !c.closed) => {
                diags.push(Diag {
                    code: Code::E080, span: r.span, stmt: Some(st.id),
                    message: format!("`{}` is an open chain: a sweep needs a closed loop; \
                        finish the chain with `-> close` or write `face({}, -> close)`",
                        r.root.text, r.root.text),
                });
                return None;
            }
            Kid::Ref(r) => match res.lookup(r) {
                Some(e) => e,
                None => {
                    diags.push(Diag {
                        code: Code::E101,
                        span: r.span,
                        stmt: Some(st.id),
                        message: format!("no such entity: `{}`", r.root.text),
                    });
                    return None;
                }
            },
            Kid::Hint(_) => {
                diags.push(Diag {
                    code: Code::E080,
                    span: st.span,
                    stmt: Some(st.id),
                    message: "a solid is made of a face or of other solids".into(),
                });
                return None;
            }
        };
        ops.push(e);
    }
    let mut say = |code: Code, span: Span, m: String| {
        diags.push(Diag { code, span, stmt: Some(st.id), message: m });
    };
    let sweep = d.sweep.as_ref().unwrap_or(&crate::syntax::Sweep::Body);
    // every number a solid carries is settled here and is never an unknown — the `fold:` rule
    let ext =
        |a: &crate::syntax::Arg, what: &str, dim: crate::units::Dim| -> Result<Extent, String> {
            let crate::syntax::Arg::Dim { text, .. } = a else {
                return Err(format!("`{what}` is not a number"));
            };
            let v = crate::flatten::value_aff(text, &BTreeMap::new(), sk.units)
                .map_err(|e| format!("`{text}`: {e}"))?;
            v.dim.require(dim, what)?;
            if !v.c.is_finite() { return Err(format!("`{what}` must be finite")); }
            Ok(Extent { text: text.trim().to_string(), value: v.c })
        };
    let def = match sweep {
        crate::syntax::Sweep::Depth { depth } => {
            let face = one_face(&ops, st, &mut say)?;
            let d = match ext(depth, "depth", crate::units::Dim::LENGTH) {
                Ok(d) => d,
                Err(m) => { say(Code::E103, st.span, m); return None; }
            };
            if d.value <= 0.0 {
                say(Code::E080, st.span,
                    "depth is a positive magnitude; use signed `from:` / `to:` ordinates for direction".into());
                return None;
            }
            SolidDef::Prism {
                face,
                from: Extent { text: format!("-({})", d.text), value: -d.value },
                to: Extent::at(0.0),
            }
        }
        crate::syntax::Sweep::Prism { from, to } => {
            let Some(face) = one_face(&ops, st, &mut say) else { return None };
            let (a, b) = match (
                ext(from, "from", crate::units::Dim::LENGTH),
                ext(to, "to", crate::units::Dim::LENGTH),
            ) {
                (Ok(a), Ok(b)) => (a, b),
                (Err(m), _) | (_, Err(m)) => {
                    say(Code::E103, st.span, m);
                    return None;
                }
            };
            if (a.value - b.value).abs() <= 0.0 {
                say(Code::E080, st.span, "a prism swept nowhere is no solid".into());
                return None;
            }
            SolidDef::Prism { face, from: a, to: b }
        }
        crate::syntax::Sweep::Revolve { axis, sweep, sense } => {
            let Some(face) = one_face(&ops, st, &mut say) else { return None };
            let Some(ax) = res.lookup(axis) else {
                say(Code::E101, axis.span, format!("no such entity: `{}`", axis.root.text));
                return None;
            };
            if ax.kind != EntKind::Line {
                say(
                    Code::E081,
                    axis.span,
                    format!(
                        "a face turns about a line, and `{}` is a {}",
                        axis.root.text,
                        ax.kind.as_str()
                    ),
                );
                return None;
            }
            // **the axis lies in the face's own plane**: a line in another view names a
            // direction this face knows nothing about
            let fp = sk.faces[face as usize].plane;
            for p in [sk.lines[ax.i()].p1, sk.lines[ax.i()].p2] {
                if sk.plane_of(p as usize).map(|x| x as u32) != fp {
                    say(
                        Code::E081,
                        axis.span,
                        format!("`{}` is not in the face's own plane", axis.root.text),
                    );
                    return None;
                }
            }
            let turn = match sweep {
                None => Extent { text: String::new(), value: std::f64::consts::TAU },
                Some(a) => match ext(a, "sweep", crate::units::Dim::ANGLE) {
                    Ok(e) => Extent { text: e.text, value: e.value.to_radians() },
                    Err(m) => {
                        say(Code::E103, st.span, m);
                        return None;
                    }
                },
            };
            // **a selector is a word, never a sign**: which way it turns is `sense:`
            if turn.value <= 0.0 {
                say(
                    Code::E040,
                    st.span,
                    "a sweep is a magnitude: which way it turns is `sense: cw`".into(),
                );
                return None;
            }
            let sense = match sense {
                crate::syntax::Sense::Cw => Sense::Cw,
                crate::syntax::Sense::Ccw => Sense::Ccw,
            };
            SolidDef::Revolve { face, axis: ax.idx, sweep: turn, sense }
        }
        crate::syntax::Sweep::Body => {
            let mut solids = Vec::new();
            for (e, k) in ops.iter().zip(kids) {
                if e.kind != EntKind::Solid {
                    let at = match k {
                        Kid::Ref(r) => r.span,
                        _ => st.span,
                    };
                    say(
                        Code::E080,
                        at,
                        format!("a body is made of solids, and this is a {}", e.kind.as_str()),
                    );
                    return None;
                }
                solids.push(e.idx);
            }
            let Some((stock, on)) = solids.split_first() else {
                say(
                    Code::E080,
                    st.span,
                    "a solid is a face swept (`from:`/`to:`, `depth:`, `about:`) or a body over \
                     other solids"
                        .into(),
                );
                return None;
            };
            SolidDef::Body { stock: *stock, on: on.to_vec(), through: Vec::new() }
        }
    };
    let i = sk.solid(def, &d.name.key().text);
    sk.solids[i].class = d.class.clone();
    Some(i)
}

/// The one face a swept solid is written over.
fn one_face(
    ops: &[EntRef],
    st: &Stmt,
    say: &mut impl FnMut(Code, Span, String),
) -> Option<u32> {
    match ops.first() {
        Some(e) if e.kind == EntKind::Face && ops.len() == 1 => Some(e.idx),
        Some(e) if e.kind != EntKind::Face => {
            say(
                Code::E080,
                st.span,
                format!("a swept solid is written over a face, and this is a {}", e.kind.as_str()),
            );
            None
        }
        _ => {
            say(Code::E080, st.span, "a swept solid is written over one face".into());
            None
        }
    }
}
