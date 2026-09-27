//! Resolve plane bases, placement dependencies, and entity membership.

use super::relations::arg_span;
use super::resolve::Resolver;
use super::{Code, Diag, SourceMap};
use crate::constraints::SpecKind;
use crate::ir::{Decl, Operation as StmtKind, Statement as Stmt};
use crate::model::{EntKind, EntRef, Sketch};
use crate::syntax::{Arg, Attitude, Span, StmtId};
use crate::{expr, io};
use std::collections::{BTreeMap, BTreeSet};

/// Resolve plane bases with memoized recursion and cycle detection. Key by absolute
/// declaration name, since expanded copies share statement IDs but have distinct bases.
pub(super) fn plane_bases(
    body: &[&Stmt],
    res: &Resolver,
    skip: &BTreeSet<StmtId>,
    units: crate::units::Units,
    diags: &mut Vec<Diag>,
) -> BTreeMap<String, crate::plane::Basis> {
    let mut decls: BTreeMap<&str, (&Stmt, &Decl)> = BTreeMap::new();
    for st in body {
        let StmtKind::Decl(d) = &st.kind else { continue };
        if d.kind == EntKind::Plane && !skip.contains(&st.id) {
            decls.entry(&d.name.key().text).or_insert((st, d));
        }
    }
    let mut done: BTreeMap<String, Option<crate::plane::Basis>> = BTreeMap::new();
    let keys: Vec<&str> = decls.keys().copied().collect();
    for k in keys {
        basis_of(k, &decls, res, units, &mut done, &mut Vec::new(), diags);
    }
    done.into_iter().filter_map(|(k, b)| b.map(|b| (k, b))).collect()
}

/// The plane a derived one is *derived from*, shared by the fold and the offset: both say "this
/// one, turned or moved", and looking the parent up twice is two chances to disagree about what
/// `from:` may name.
fn parent_plane<'a>(
    plane: &'a crate::syntax::Ref,
    key: &'a str,
    decls: &BTreeMap<&'a str, (&'a Stmt, &'a Decl)>,
    res: &Resolver,
    units: crate::units::Units,
    done: &mut BTreeMap<String, Option<crate::plane::Basis>>,
    stack: &mut Vec<&'a str>,
    diags: &mut Vec<Diag>,
) -> Option<crate::plane::Basis> {
    let stmt = decls.get(key).map(|&(st, _)| st.id);
    let fail = |diags: &mut Vec<Diag>, code: Code, span: Span, message: String| {
        diags.push(Diag { code, span, stmt, message });
    };
    if !plane.path.is_empty() {
        fail(diags, Code::E040, plane.span, "`from` names a plane, not a part of one".into());
        None
    } else {
        match res.lookup(plane) {
            None => {
                fail(
                    diags,
                    Code::E101,
                    plane.span,
                    format!("no such entity: `{}`", plane.root.text),
                );
                None
            }
            Some(e) if e.kind != EntKind::Plane => {
                fail(
                    diags,
                    Code::E040,
                    plane.span,
                    format!(
                        "`{}` is a {}, and `from` names a plane",
                        plane.root.text,
                        e.kind.as_str()
                    ),
                );
                None
            }
            Some(_) if stack.contains(&key) || plane.root.text == key => {
                fail(
                    diags,
                    Code::E041,
                    plane.span,
                    format!("`{key}` is folded from itself, through `{}`", plane.root.text),
                );
                None
            }
            Some(_) => {
                stack.push(key);
                let p = basis_of(plane.root.text.as_str(), decls, res, units, done, stack, diags);
                stack.pop();
                p
            }
        }
    }
}

fn basis_of<'a>(
    key: &'a str,
    decls: &BTreeMap<&'a str, (&'a Stmt, &'a Decl)>,
    res: &Resolver,
    units: crate::units::Units,
    done: &mut BTreeMap<String, Option<crate::plane::Basis>>,
    stack: &mut Vec<&'a str>,
    diags: &mut Vec<Diag>,
) -> Option<crate::plane::Basis> {
    let &(st, d) = decls.get(key)?;
    if let Some(b) = done.get(key) {
        return *b;
    }
    let fail = |diags: &mut Vec<Diag>, code: Code, span: Span, message: String| {
        diags.push(Diag { code, span, stmt: Some(st.id), message });
    };
    // one number the attitude was written with, as the dimension its slot takes
    // what a written number comes to, asked of the one function that answers it everywhere —
    // an attitude is written in the same little language a dimension is, and read in the
    // document's own units.  Nothing is in scope: a plane's fold is settled per copy by the
    // flattener before this runs, so what arrives here is arithmetic over literals.
    let number = |a: &Arg, want: crate::units::Dim, what: &str| -> Result<f64, String> {
        let Arg::Dim { text, .. } = a else { return Err(format!("`{what}` is not a number")) };
        let v = crate::flatten::value_aff(text, &BTreeMap::new(), units)
            .map_err(|e| format!("`{text}`: {e}"))?;
        v.dim.require(want, what)?;
        Ok(v.c)
    };
    // a seed the `hint(…)` clause wrote for a quantity the brackets made an unknown; whether
    // they did is the views pass's question (`views::solve_planes`), which refuses one they did
    // not — here it is only where the solve begins
    let seed = |key: &str, want: crate::units::Dim| -> Option<Vec<f64>> {
        let h = d.plane.hint(key)?;
        h.args.iter().map(|a| number(a, want, key).ok()).collect()
    };
    let basis = match &d.attitude {
        Attitude::Page => Some(crate::plane::Basis::page()),
        // **a solved fold starts where its seed says** — `fold: beta` with `hint(fold: 30deg)`
        // is folded 30° until the solve moves it, and at the constant part of its expression
        // (`beta` at 0) where no seed is written
        Attitude::From { plane, fold } => {
            let parent = parent_plane(plane, key, decls, res, units, done, stack, diags);
            let theta = match fold_aff(fold, units) {
                Ok(a) if a.free.is_some() => {
                    let deg = seed("fold", crate::units::Dim::ANGLE).map_or(a.c, |v| v[0]);
                    Some(expr::to_arg_units(SpecKind::Angle, deg))
                }
                Ok(_) => match number(fold, crate::units::Dim::ANGLE, "fold") {
                    Ok(deg) => Some(expr::to_arg_units(SpecKind::Angle, deg)),
                    Err(m) => {
                        fail(diags, Code::E103, arg_span(fold).unwrap_or(st.span), m);
                        None
                    }
                },
                Err(m) => {
                    fail(diags, Code::E103, arg_span(fold).unwrap_or(st.span), m);
                    None
                }
            };
            match (parent, theta) {
                (Some(p), Some(t)) => Some(p.fold(t)),
                _ => None,
            }
        }
        // **parallel, and that far along the normal** (§6.7).  Only along the normal: an offset
        // in the plane would move the origin `project` measures both images from and put a
        // constant in a residual that has none.
        Attitude::Offset { plane, offset } => {
            let parent = parent_plane(plane, key, decls, res, units, done, stack, diags);
            let k = match offset {
                None => Some(0.0),
                Some(a) => match number(a, crate::units::Dim::LENGTH, "offset") {
                    Ok(v) => Some(v),
                    Err(m) => {
                        fail(diags, Code::E103, arg_span(a).unwrap_or(st.span), m);
                        None
                    }
                },
            };
            match (parent, k) {
                (Some(p), Some(k)) => Some(p.offset(k)),
                _ => None,
            }
        }
        // folded along a line, it stands where the line is: that is known only once the line is
        // built, so the parent's `fold(0)` stands in until the views pass reads the line
        Attitude::Along { plane, .. } => {
            parent_plane(plane, key, decls, res, units, done, stack, diags).map(|p| p.fold(0.0))
        }
        // a free attitude starts at the basis its seed gives, or the page's
        Attitude::Free { span } => {
            match (seed("u", crate::units::Dim::SCALAR), seed("v", crate::units::Dim::SCALAR)) {
                (Some(u), Some(v)) => {
                    let b = crate::plane::Basis::explicit([u[0], u[1], u[2]], [v[0], v[1], v[2]]);
                    if b.is_none() {
                        fail(diags, Code::E103, *span, "the seeded `u` and `v` span no plane".into());
                    }
                    b
                }
                _ => Some(crate::plane::Basis::page()),
            }
        }
        Attitude::Basis { u, v } => {
            let mut vals = [[0.0; 3]; 2];
            let mut ok = true;
            for (k, (name, triple)) in [("u", u), ("v", v)].into_iter().enumerate() {
                for (i, a) in triple.iter().enumerate() {
                    match number(a, crate::units::Dim::SCALAR, name) {
                        Ok(x) => vals[k][i] = x,
                        Err(m) => {
                            fail(diags, Code::E103, arg_span(a).unwrap_or(st.span), m);
                            ok = false;
                        }
                    }
                }
            }
            let b = ok.then(|| crate::plane::Basis::explicit(vals[0], vals[1])).flatten();
            if ok && b.is_none() {
                fail(
                    diags,
                    Code::E103,
                    arg_span(&u[0]).unwrap_or(st.span),
                    "`u` and `v` do not span a plane".into(),
                );
            }
            b
        }
    };
    // an offset solved for starts at its seed, along the normal of whatever the attitude is
    let basis = match (&d.plane.position, basis) {
        (crate::syntax::Position::Free(_), Some(b)) => {
            Some(seed("offset", crate::units::Dim::LENGTH).map_or(b, |k| b.offset(k[0])))
        }
        (_, b) => b,
    };
    done.insert(key.to_string(), basis);
    basis
}

/// A fold as its expression comes to: a number, or affine in the one name nothing defines —
/// `fold: beta`, the fold solved for (§6.7, P2a).  The views pass asks the same question the
/// basis did, so the two cannot disagree about which folds are unknowns.
pub(super) fn fold_aff(fold: &Arg, units: crate::units::Units) -> Result<expr::Aff, String> {
    let Arg::Dim { text, .. } = fold else { return Err("`fold` is not a number".into()) };
    let p = expr::parse_in(text.trim(), units)?;
    let a = expr::eval(&p.body, &BTreeMap::new()).map_err(|e| format!("`{text}`: {e}"))?;
    if a.free.is_none() && !a.c.is_finite() {
        return Err(format!("`{text}` comes to {}", a.c));
    }
    Ok(a)
}
pub(super) fn memberships(
    sk: &mut Sketch,
    res: &Resolver,
    map: &SourceMap,
    body: &[&Stmt],
    skip: &BTreeSet<StmtId>,
    diags: &mut Vec<Diag>,
) {
    for st in body {
        let StmtKind::Decl(d) = &st.kind else { continue };
        let Some(r) = d.membership.plane() else { continue };
        if skip.contains(&st.id) {
            continue;
        }
        let mut fail = |code: Code, span: Span, message: String| {
            diags.push(Diag { code, span, stmt: Some(st.id), message });
        };
        let plane = match res.lookup(r) {
            None => {
                fail(Code::E101, r.span, format!("no such entity: `{}`", r.root.text));
                continue;
            }
            Some(e) if e.kind != EntKind::Plane || !r.path.is_empty() => {
                fail(
                    Code::E040,
                    r.span,
                    format!("`{}` is a {}, and `in` names a plane", r.root.text, e.kind.as_str()),
                );
                continue;
            }
            Some(e) => e.i(),
        };
        // a declaration that could not be built made nothing to put anywhere
        let Some(me) = res.of.get(&d.name.key().text).copied() else { continue };
        let points: Vec<usize> = match me.kind {
            EntKind::Point => vec![me.i()],
            _ => sk.children(me).into_iter().map(|k| k.i()).collect(),
        };
        for p in points {
            match sk.plane_of(p) {
                Some(q) if q != plane => {
                    let who =
                        |e: EntRef| map.name_of(e).cloned().unwrap_or_else(|| io::entity_name(e));
                    fail(
                        Code::E060,
                        // the ref's own span: the clause's word, or the block header's
                        r.span,
                        format!(
                            "`{}` is already in `{}`",
                            who(EntRef::point(p)),
                            who(EntRef::plane(q))
                        ),
                    );
                }
                _ => sk.set_plane(p, Some(plane)),
            }
        }
    }
}

/// The one plane every point of an entity is on, or `None` — for a point, its own.
pub(crate) fn plane_of_entity(sk: &Sketch, e: EntRef) -> Option<usize> {
    plane_of_entity_by(sk, e, |p| sk.plane_of(p))
}

/// The same walk over any reading of where a point is — membership here, and the overview's
/// `view_of` (which also reads a datum's own points in the plane they place) — so the rule
/// "every point it is made of agrees" is written once.
pub(crate) fn plane_of_entity_by(
    sk: &Sketch,
    e: EntRef,
    at: impl Fn(usize) -> Option<usize>,
) -> Option<usize> {
    // a point answers for itself, and a datum or a curve has no points of its own to answer
    // for; everything else is its children, walked without collecting them twice
    if !e.kind.bears_points() {
        return None;
    }
    if e.kind == EntKind::Point {
        return at(e.i());
    }
    let kids = sk.children(e);
    let first = at(kids.first()?.i())?;
    kids.iter().all(|k| at(k.i()) == Some(first)).then_some(first)
}
