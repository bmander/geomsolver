//! Lower a resolved solid declaration into its typed solid definition.
use super::*;

/// **A solid is a face swept, or a term over other solids** (§6.9).
pub(super) fn build_solid(
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
                EntRef::face(build_face(sk, res, face, &d.name.key().text, st.id, *span, diags)?)
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
        if e.kind == EntKind::Face && sk.faces.get(e.i()).is_none_or(|f| f.plane().is_err()) {
            diags.push(Diag {code:Code::E080,span:st.span,stmt:Some(st.id),
                message:"a sweep needs a built planar profile; a spatial face cannot be swept".into()});
            return None;
        }
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
        crate::syntax::Sweep::Along { guide } => {
            if !(1..=2).contains(&ops.len()) || ops.iter().any(|e| e.kind != EntKind::Face) {
                say(Code::E080, st.span, "`along:` requires one start face and an optional end face".into());
                return None;
            }
            let Some(g) = res.lookup(guide).and_then(|e| super::super::resolve::follow(sk, e, &guide.path).ok()) else {
                say(Code::E101, guide.span, "no such guide geometry".into());
                return None;
            };
            if !matches!(g.kind, EntKind::Line | EntKind::Arc) {
                say(Code::E081, guide.span, "`along:` requires a directed line or circular arc".into());
                return None;
            }
            SolidDef::Loft { face: ops[0].idx, end: ops.get(1).map(|e| e.idx), guide: g }
        }
        crate::syntax::Sweep::Through { body } => {
            let face = one_face(&ops, st, &mut say)?;
            let Some(target) = res.lookup(body) else {
                say(Code::E101, body.span, format!("no such entity: `{}`", body.root.text));
                return None;
            };
            if target.kind != EntKind::Solid {
                say(Code::E080, body.span, "`through:` requires a solid target".into());
                return None;
            }
            SolidDef::Through { face, body: target.idx }
        }
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
            let fp = sk.faces[face as usize].plane().ok()?;
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
