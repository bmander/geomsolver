//! Lower a resolved solid declaration into its typed solid definition.
use super::*;
use crate::syntax::Sweep;

/// **A solid is a face swept, or a term over other solids** (§6.9).
pub(super) fn build_solid(
    sk: &mut Sketch,
    res: &Resolver,
    d: &Decl,
    st: &Stmt,
    probes: &mut super::super::regions::Probes,
    diags: &mut Vec<Diag>,
) -> Option<usize> {
    let kids = d.children.first().map(Vec::as_slice).unwrap_or(&[]);
    let ops = operands(sk, res, d, kids, st, diags)?;
    let mut say = |code: Code, span: Span, m: String| {
        diags.push(Diag::within(code, span, st.id, m));
    };
    let sweep = d.sweep.as_ref().unwrap_or(&Sweep::Body);
    // every number a solid carries is settled here and is never an unknown (`Extent`)
    let units = sk.units;
    let ext = |a: &crate::syntax::Arg, what: &str, dim: crate::units::Dim| extent(units, a, what, dim);
    // a prism's draft (#245): an angle, leaning its sides less than square to the face
    let draft = match sweep.draft().map(|a| ext(a, "draft", crate::units::Dim::ANGLE)) {
        None => None,
        Some(Err(m)) => { say(Code::E103, st.span, m); return None; }
        Some(Ok(e)) if e.value.abs() >= 90.0 => {
            say(Code::E080, st.span, format!("a draft of `{}` leans a side flat: a draft is less \
                                              than a right angle", e.text));
            return None;
        }
        Some(Ok(e)) => (e.value != 0.0).then(|| Extent { text: e.text, value: e.value.to_radians() }),
    };
    let def = match sweep {
        // a region: the bounds its set put on the probe, gathered as they were stated (§6.21)
        Sweep::Region { probe } => region(sk, res, probe, st, probes, &mut say)?,
        Sweep::Placed { .. } | Sweep::Swept { .. } | Sweep::Rolled { .. } => {
            under_motion(sk, res, sweep, &ops, st, &mut say)?
        }
        Sweep::Along { guide } => {
            if !(1..=2).contains(&ops.len()) || ops.iter().any(|e| e.kind != EntKind::Face) {
                say(Code::E080, st.span, "`along:` requires one start face and an optional end face".into());
                return None;
            }
            let Some(g) = res.entity(sk, guide) else {
                say(Code::E101, guide.span, "no such guide geometry".into());
                return None;
            };
            if !matches!(g.kind, EntKind::Line | EntKind::Arc) {
                say(Code::E081, guide.span, "`along:` requires a directed line or circular arc".into());
                return None;
            }
            SolidDef::Loft { face: ops[0].idx, end: ops.get(1).map(|e| e.idx), guide: g }
        }
        Sweep::Through { body } => {
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
        Sweep::Depth { depth, .. } => {
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
                draft,
            }
        }
        Sweep::Prism { from, to, .. } => {
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
            SolidDef::Prism { face, from: a, to: b, draft }
        }
        Sweep::Revolve { axis, sweep, sense } => {
            let face = one_face(&ops, st, &mut say)?;
            revolve(sk, res, face, (axis, sweep.as_ref(), *sense), st, &mut say)?
        }
        Sweep::Body => body(&ops, kids, st, &mut say)?,
        Sweep::Family(f) => family(sk, res, f, &ops, st, &mut say)?,
    };
    let i = sk.solid(def, &d.name.key().text);
    sk.solids[i].class = d.class.clone();
    Some(i)
}

/// **A face family** (#243 L1): the face of the station the flattener drew, the running number it
/// held, and every entity the station is made of — what the family solves again at each station.
fn family(
    sk: &Sketch,
    res: &Resolver,
    f: &crate::syntax::FamilySpec,
    ops: &[EntRef],
    st: &Stmt,
    say: &mut impl FnMut(Code, Span, String),
) -> Option<SolidDef> {
    let face = one_face(ops, st, say)?;
    let held = f.held.clone()?;
    let ends = [&f.domain.0, &f.domain.1].map(|t| crate::expr::literal(t));
    let [Some(a), Some(b)] = ends else {
        say(Code::E103, st.span, "a face family's interval is two numbers".into());
        return None;
    };
    // the station: every entity its instance made, the face's edges (a closing edge the face
    // made itself among them), and what each is made of
    let prefix = format!("{}.", held.rsplit_once('.').map_or("", |(p, _)| p));
    let mut station: BTreeSet<EntRef> = res.of.iter()
        .filter(|(k, _)| k.starts_with(&prefix))
        .map(|(_, &e)| e)
        .chain(sk.faces[face as usize].boundaries().flat_map(|(edges, _)| edges.iter().copied()))
        .filter(|e| !matches!(e.kind, EntKind::Face | EntKind::Solid))
        .collect();
    let mut stack: Vec<EntRef> = station.iter().copied().collect();
    while let Some(e) = stack.pop() {
        for c in sk.children(e) {
            if !matches!(c.kind, EntKind::Face | EntKind::Solid) && station.insert(c) {
                stack.push(c);
            }
        }
    }
    let extent = |v: f64, t: &str| Extent { text: t.trim().to_string(), value: v };
    Some(SolidDef::Family {
        face,
        station: station.into_iter().collect(),
        held,
        from: extent(a, &f.domain.0),
        to: extent(b, &f.domain.1),
    })
}

/// What a solid is written over, resolved: faces (one written in place built here) or solids —
/// or `None`, refused.
fn operands(
    sk: &mut Sketch,
    res: &Resolver,
    d: &Decl,
    kids: &[Kid],
    st: &Stmt,
    diags: &mut Vec<Diag>,
) -> Option<Vec<EntRef>> {
    let mut ops: Vec<EntRef> = Vec::new();
    for k in kids {
        let e = match k {
            Kid::Ref(r) if !r.path.is_empty() && res.chains.contains_key(&r.root.text) => {
                diags.push(Diag::within(
                    Code::E080, r.span, st.id,
                    format!("`{}` names a chain, not a member of that chain; \
                        its named links belong to the enclosing component", r.root.text),
                ));
                return None;
            }
            Kid::Face { decl: face, span } => {
                EntRef::face(build_face(sk, res, face, &d.name.key().text, st.id, *span, diags)?)
            }
            Kid::Ref(r) if res.chains.get(&r.root.text).is_some_and(|c| !c.closed) => {
                diags.push(Diag::within(
                    Code::E080, r.span, st.id,
                    format!("`{}` is an open chain: a sweep needs a closed loop; \
                        finish the chain with `-> close` or write `face({}, -> close)`",
                        r.root.text, r.root.text),
                ));
                return None;
            }
            Kid::Ref(r) => match res.lookup(r).ok_or_else(||
                format!("no such entity: `{}`",r.root.text))
                .and_then(|e| super::super::resolve::follow_building(sk,res,e,r)) {
                Ok(e) => e,
                Err(message) => {
                    diags.push(Diag::within(
                        Code::E101,
                        r.span,
                        st.id,
                        message,
                    ));
                    return None;
                }
            },
            Kid::Hint(_) | Kid::Trim { .. } => {
                diags.push(Diag::on(
                    Code::E080,
                    st,
                    "a solid is made of a face or of other solids".into(),
                ));
                return None;
            }
        };
        if e.kind == EntKind::Face && sk.faces.get(e.i()).is_none_or(|f| f.plane().is_err()) {
            diags.push(Diag::on(Code::E080,st,
                "a sweep needs a built planar profile; a spatial face cannot be swept".into()));
            return None;
        }
        ops.push(e);
    }
    Some(ops)
}

/// A number a solid carries, settled in the document's units and checked to be of `dim`.
fn extent(
    units: crate::units::Units, a: &crate::syntax::Arg, what: &str, dim: crate::units::Dim,
) -> Result<Extent, String> {
    let crate::syntax::Arg::Dim { text, .. } = a else {
        return Err(format!("`{what}` is not a number"));
    };
    let v = crate::flatten::value_aff(text, &BTreeMap::new(), units)
        .map_err(|e| format!("`{text}`: {e}"))?;
    v.dim.require(dim, what)?;
    if !v.c.is_finite() { return Err(format!("`{what}` must be finite")); }
    Ok(Extent { text: text.trim().to_string(), value: v.c })
}

/// A region solid: the bounds its set put on the probe, gathered as they were stated (§6.21), and
/// the probe held.
fn region(
    sk: &mut Sketch,
    res: &Resolver,
    probe: &crate::syntax::Ref,
    st: &Stmt,
    probes: &mut super::super::regions::Probes,
    say: &mut impl FnMut(Code, Span, String),
) -> Option<SolidDef> {
    let Some(p) = res.lookup(probe).filter(|e| e.kind == EntKind::Point) else {
        let m = "a region solid lost the point its set was applied to";
        say(Code::E080, st.span, m.into());
        return None;
    };
    let mut terms = probes.terms.remove(&p.i()).unwrap_or_default();
    if let Err(m) = super::super::regions::fill_sets(sk, &mut terms, probes, p.i(), 0) {
        say(Code::E040, st.span, m);
        return None;
    }
    if terms.is_empty() {
        let m = "a solid is a region: a set whose body bounds a number, or a set its \
                 `inside` reads one of";
        say(Code::E080, st.span, m.into());
        return None;
    }
    for q in sk.own_params(p) {
        sk.params[q as usize].fixed = true;
    }
    Some(SolidDef::Region { probe: p.idx, terms })
}

/// A solid under a named motion: placed at one pose (`at:`), swept over an interval, or rolled
/// through a body's material (#271, found after the solve).
fn under_motion(
    sk: &Sketch,
    res: &Resolver,
    sweep: &Sweep,
    ops: &[EntRef],
    st: &Stmt,
    say: &mut impl FnMut(Code, Span, String),
) -> Option<SolidDef> {
    let (Sweep::Placed { motion, .. } | Sweep::Swept { motion, .. } | Sweep::Rolled { motion, .. }) =
        sweep else { unreachable!() };
    if ops.len() != 1 || ops[0].kind != EntKind::Solid {
        say(Code::E080,st.span,"a named motion needs one source solid".into());
        return None;
    }
    let found = res.lookup(motion).and_then(|e|
        super::super::resolve::follow_building(sk,res,e,motion).ok());
    let Some(m) = found.filter(|e| e.kind == EntKind::Motion) else {
        say(Code::E080,motion.span,"`under:` requires a named motion".into());
        return None;
    };
    let angle = |a,what| extent(sk.units,a,what,crate::units::Dim::ANGLE)
        .map(|e| Extent {text:e.text,value:e.value.to_radians()});
    Some(match sweep {
        Sweep::Placed {at,..} => {
            let at = match angle(at,"at") {
                Ok(at) => at,
                Err(message) => { say(Code::E103,st.span,message); return None; }
            };
            SolidDef::Placed {source:ops[0].idx,motion:m.idx,at}
        }
        Sweep::Swept {from,to,..} => {
            let (from,to) = match (angle(from,"from"),angle(to,"to")) {
                (Ok(a),Ok(b)) if a.value < b.value => (a,b),
                (Ok(_),Ok(_)) => {
                    say(Code::E080,st.span,"a continuous motion sweep needs increasing angular bounds; use `at:` for one pose".into());
                    return None;
                }
                (Err(message),_) | (_,Err(message)) => { say(Code::E103,st.span,message); return None; }
            };
            SolidDef::Swept {source:ops[0].idx,motion:m.idx,roll:crate::model::Roll::Stated {from,to}}
        }
        // the roll through a body's material (#271): found after the solve
        Sweep::Rolled {body,..} => {
            let Some(target) = res.lookup(body) else {
                say(Code::E101, body.span, format!("no such entity: `{}`", body.root.text));
                return None;
            };
            if target.kind != EntKind::Solid {
                say(Code::E080, body.span, "`through:` requires a solid to roll through".into());
                return None;
            }
            SolidDef::Swept {source:ops[0].idx,motion:m.idx,roll:crate::model::Roll::Through {body:target.idx}}
        }
        _ => unreachable!(),
    })
}

/// A face turned about a line in its own plane or an axis lying there (§6.9), through a sweep
/// that is a magnitude, which way a word (`sense:`).
fn revolve(
    sk: &Sketch,
    res: &Resolver,
    face: u32,
    (axis, sweep, sense): (&crate::syntax::Ref, Option<&crate::syntax::Arg>, crate::syntax::Sense),
    st: &Stmt,
    say: &mut impl FnMut(Code, Span, String),
) -> Option<SolidDef> {
    let found = res.entity(sk, axis);
    let Some(ax) = found else {
        say(Code::E101, axis.span, format!("no such entity: `{}`", axis.root.text));
        return None;
    };
    let fp = sk.faces[face as usize].plane().ok()?;
    match ax.kind {
        // **the line lies in the face's own plane**: a line in another view names a
        // direction this face knows nothing about
        EntKind::Line => {
            for p in [sk.lines[ax.i()].p1, sk.lines[ax.i()].p2] {
                if sk.plane_of(p as usize).map(|x| x as u32) != fp {
                    say(Code::E081, axis.span,
                        format!("`{}` is not in the face's own plane", axis.root.text));
                    return None;
                }
            }
        }
        // an axis in space lies in the plane where the plane's own axes say so, or where
        // what holds both says so now; one the solve moves is read where it is evaluated
        EntKind::Axis => {
            if !axis_may_lie_in(sk, fp, ax.i()) {
                say(Code::E081, axis.span,
                    format!("`{}` does not lie in the face's own plane", axis.root.text));
                return None;
            }
        }
        _ => {
            say(Code::E081, axis.span,
                format!("a face turns about a line or an axis, and `{}` is {}",
                    axis.root.text, ax.kind.a()));
            return None;
        }
    }
    let turn = match sweep {
        None => Extent { text: String::new(), value: std::f64::consts::TAU },
        Some(a) => match extent(sk.units, a, "sweep", crate::units::Dim::ANGLE) {
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
    Some(SolidDef::Revolve { face, axis: ax, sweep: turn, sense })
}

/// A body over other solids: the first its stock, the rest in `union` with it.
fn body(
    ops: &[EntRef],
    kids: &[Kid],
    st: &Stmt,
    say: &mut impl FnMut(Code, Span, String),
) -> Option<SolidDef> {
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
                format!("a body is made of solids, and this is {}", e.kind.a()),
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
    Some(SolidDef::Body { stock: *stock, on: on.to_vec(), through: Vec::new(), bound: Vec::new() })
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
                format!("a swept solid is written over a face, and this is {}", e.kind.a()),
            );
            None
        }
        _ => {
            say(Code::E080, st.span, "a swept solid is written over one face".into());
            None
        }
    }
}

/// Whether axis `ax` may lie in plane `plane` (a face's, `None` the page): it is one of the plane's
/// own axes; or nothing the solve moves places either, and it lies there now; or the solve may
/// put it there, which the evaluation reads (`Sketch::revolve_axis`).
fn axis_may_lie_in(sk: &Sketch, plane: Option<u32>, ax: usize) -> bool {
    let own = plane.map(|p| &sk.planes[p as usize]);
    if own.is_some_and(|pl| [pl.u, pl.v].contains(&(ax as u32))) {
        return true;
    }
    let (x, held) = (&sk.axes[ax], super::super::views::held);
    let still = held(sk, &x.d) && held(sk, &x.a)
        && plane.is_none_or(|p| sk.plane_fixed(p as usize));
    !still || sk.axis_in(plane, ax).is_some()
}
