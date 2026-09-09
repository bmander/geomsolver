//! Solid claims, face bearing and placement after construction.
use super::*;

/// Apply solid claims after body construction, preserving statement spans for diagnostics.
pub(in crate::program) fn solid_claims(
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
    let StmtKind::Relation(r) = &st.kind else {
        return;
    };
    if skip.contains(&st.id) {
        return;
    }
    let Some(w) = r.form.written() else { return };
    let Some(word) = crate::constraints::solid_word(&w.word.text) else {
        return;
    };
    let mut say = |code: Code, span: Span, m: String| {
        diags.push(Diag {
            code,
            span,
            stmt: Some(st.id),
            message: m,
        });
    };
    // **it is a claim whether or not it says so.**  These words cannot act — there is no row for
    // them — so a document that writes one without `claim` is saying the same thing, and refusing
    // it would be a rule about spelling rather than about meaning.  Written *with* `claim` is the
    // reading to prefer, and the printer spells it that way.
    if w.ops.len() != 2 {
        say(
            Code::E040,
            st.span,
            format!("`{}` relates two solids", word.as_str()),
        );
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
                say(
                    Code::E101,
                    o.span,
                    format!("no such entity: `{}`", o.root.text),
                );
                return;
            }
        }
    }
    use crate::model::{Length, SolidRequirement};
    let requirement = match (word.takes_gap(), w.args.as_slice()) {
        (false, []) => SolidRequirement::Inside,
        (true, [crate::syntax::OpArg::Dim(text, span)]) => {
            match crate::flatten::value_aff(text, &BTreeMap::new(), sk.units) {
                Ok(v)
                    if v.c.is_finite()
                        && v.dim
                            .require(crate::units::Dim::LENGTH, word.as_str())
                            .is_ok() =>
                {
                    let gap = Length::written(v.c, text.trim().to_string()).unwrap();
                    match word {
                        crate::constraints::SolidWord::Fits => SolidRequirement::Fits { gap },
                        crate::constraints::SolidWord::Clear => SolidRequirement::Clear { gap },
                        _ => unreachable!(),
                    }
                }
                _ => {
                    say(
                        Code::E103,
                        *span,
                        format!("`{}` asks for a finite length", word.as_str()),
                    );
                    return;
                }
            }
        }
        (true, []) => {
            say(
                Code::E040,
                st.span,
                format!(
                    "`{}` asks for room: `{}(2mm)`",
                    word.as_str(),
                    word.as_str()
                ),
            );
            return;
        }
        _ => {
            say(
                Code::E040,
                st.span,
                format!(
                    "`{}` takes {}",
                    word.as_str(),
                    if word.takes_gap() {
                        "exactly one length argument"
                    } else {
                        "no arguments"
                    }
                ),
            );
            return;
        }
    };
    sk.solid_claims.push(crate::model::SolidClaim {
        requirement,
        a: ends[0],
        b: ends[1],
        over: over.cloned(),
        stmt: st.id.0,
    });
    map.record(st, Made::Gauge);
}

/// Resolve stack placement from bearing faces (§6.10), diagnosing inconsistent or
/// missing placement of planes with unspecified offsets.
pub(super) fn place(
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
                    sk.faces.get(*face as usize)?.plane().ok()??,
                    ord,
                    sign * parity,
                    format!("{}.{last}", s.name),
                ));
            }
            SolidDef::Revolve { .. } | SolidDef::Through { .. } | SolidDef::Loft { .. }
                | SolidDef::Placed { .. } | SolidDef::Swept { .. } => return None,
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
        diags.push(Diag {
            code,
            span,
            stmt: Some(st.id),
            message: m,
        });
    };
    let name = crate::syntax::ref_text(&c.formal);
    // **a free variable, and nothing else**: a `param` is a number the document already fixed,
    // and sweeping it would be sweeping a constant
    if !sk.free_vars.contains_key(&name) {
        if res.lookup(&c.formal).is_some() {
            say(
                Code::E040,
                c.formal.span,
                format!("`{name}` is geometry, not a free variable"),
            );
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
        let crate::syntax::Arg::Dim { text, span } = a else {
            say(
                Code::E103,
                st.span,
                format!(
                    "`{what}` for `{name}` must be a finite {}",
                    dimension.name()
                ),
            );
            return None;
        };
        match crate::flatten::value_aff(text, &BTreeMap::new(), sk.units) {
            Ok(v) if v.c.is_finite() && v.dim.require(dimension, &name).is_ok() => Some(v.c),
            Ok(_) => {
                say(
                    Code::E103,
                    *span,
                    format!(
                        "`{what}` for `{name}` must be a finite {}",
                        dimension.name()
                    ),
                );
                None
            }
            Err(e) => {
                say(Code::E103, *span, format!("`{what}`: {e}"));
                None
            }
        }
    };
    let (from, to) = (num(&c.from, "from")?, num(&c.to, "to")?);
    Some(crate::model::Sweep {
        name,
        from,
        to,
        dimension,
    })
}
