//! Views whose attitude or position is **solved for** (§6.7; `docs/spatial-constraints-plan.md`,
//! P2a): a plane is fixed unless its brackets name an unknown, and this pass is where the
//! unknowns a plane's brackets named are minted and tied down.
//!
//! It runs once every plane is built and every membership is in, and before any relation is
//! stated, so a relation over a solved view is stated against its unknowns from the first.  What
//! each clause comes to:
//!
//! - `attitude: free` — the quaternion free (three freedoms and a `quat_unit` row), the offset
//!   held where the seed stood it;
//! - `fold: E` over a name nothing defines — a hinge to the parent whose fold is that free
//!   variable (`CKind::Hinge`, the free twin), the parent given held unknowns if it is stated;
//! - `from: P, fold: θ` or `offset: k` where `P` is solved — a hinge whose turn is a constant;
//! - `fold: along l` — a hinge along `l`'s bearing in the parent, and the offset solved so `l`'s
//!   first end is in the view (`CKind::HingeAlong` and a `PointOnPlane`);
//! - `offset: free` — the offset free, the attitude whatever else says it is;
//! - `through: M` — the offset solved so `M` is in the plane (`PointOnPlane`).
//!
//! A plane that says none of these, folded from one that is stated, stays exactly as it was:
//! no parameter, no row, and the corpus compiles to the same bytes.

use super::planes::fold_aff;
use super::resolve::{follow, Resolver};
use super::{Code, Diag, Made, SourceMap};
use crate::constraints::{Arg as CArg, CKind, Constraint, SpecKind};
use crate::ir::{Decl, Operation as StmtKind, Statement as Stmt};
use crate::model::{EntKind, EntRef, Sketch};
use crate::syntax::{Attitude, Position, Span, StmtId};
use std::collections::{BTreeMap, BTreeSet};

/// A plane's in-plane origin this close to nothing is on its own normal through the shared
/// origin — which is what a fold turning about a line through it keeps.
const ON_NORMAL: f64 = 1e-9;

pub(super) fn solve_planes(
    sk: &mut Sketch,
    res: &Resolver,
    map: &mut SourceMap,
    body: &[&Stmt],
    skip: &BTreeSet<StmtId>,
    diags: &mut Vec<Diag>,
) {
    let mut decls: BTreeMap<&str, (&Stmt, &Decl)> = BTreeMap::new();
    for st in body {
        let StmtKind::Decl(d) = &st.kind else { continue };
        if d.kind == EntKind::Plane && !skip.contains(&st.id) {
            decls.entry(&d.name.key().text).or_insert((st, d));
        }
    }
    // parents before children: a hinge reads its parent's unknowns, so they are minted first.
    // A cycle was refused where the bases were worked out (E041); it is only cut short here.
    let mut order: Vec<&str> = Vec::new();
    let mut seen: BTreeSet<&str> = BTreeSet::new();
    fn visit<'a>(k: &'a str, decls: &BTreeMap<&'a str, (&'a Stmt, &'a Decl)>,
                 seen: &mut BTreeSet<&'a str>, order: &mut Vec<&'a str>) {
        if !seen.insert(k) {
            return;
        }
        if let Some((_, d)) = decls.get(k) {
            if let Some(p) = d.attitude.plane_ref() {
                if let Some((&pk, _)) = decls.get_key_value(p.root.text.as_str()) {
                    visit(pk, decls, seen, order);
                }
            }
        }
        order.push(k);
    }
    for &k in decls.keys() {
        visit(k, &decls, &mut seen, &mut order);
    }
    for k in order {
        let (st, d) = decls[k];
        let Some(e) = res.of.get(k).copied().filter(|e| e.kind == EntKind::Plane) else { continue };
        one(sk, res, map, st, d, e.i(), diags);
    }
}

/// What a solved view's plane statement comes to.
fn one(
    sk: &mut Sketch,
    res: &Resolver,
    map: &mut SourceMap,
    st: &Stmt,
    d: &Decl,
    pi: usize,
    diags: &mut Vec<Diag>,
) {
    let fail = |diags: &mut Vec<Diag>, code: Code, span: Span, message: String| {
        diags.push(Diag { code, span, stmt: Some(st.id), message });
    };
    let name = sk.plane_name(pi);
    let parent = d.attitude.plane_ref().and_then(|r| res.lookup(r)).filter(|e| e.kind == EntKind::Plane)
        .map(|e| e.i());
    let parent_solved = parent.is_some_and(|p| sk.planes[p].att.is_some());
    // is the fold an unknown: an expression over a name nothing defines
    let free_fold = match &d.attitude {
        Attitude::From { fold, .. } => fold_aff(fold, sk.units).ok().filter(|a| a.free.is_some()),
        _ => None,
    };
    // **a seed is for an unknown** (§4.3): a `hint(…)` key for a quantity the brackets state
    // is refused at the key, in the words that say which clause would make it one
    for h in &d.plane.hints {
        let why = match h.key.text.as_str() {
            "fold" if free_fold.is_none() => {
                Some("the fold is stated; a seed is for one solved for, `fold: beta`")
            }
            "u" | "v" if !matches!(d.attitude, Attitude::Free { .. }) => {
                Some("the attitude is stated; a seed is for one solved for, `attitude: free`")
            }
            "offset" if !matches!(d.plane.position, Position::Free(_)) => {
                Some("the offset is stated; a seed is for one solved for, `offset: free`")
            }
            _ => None,
        };
        if let Some(why) = why {
            fail(diags, Code::E040, h.key.span, format!("`{}` on `{name}`: {why}", h.key.text));
            continue;
        }
        // and a seed is a number of what it seeds: an angle, a length, a direction's component
        let want = match h.key.text.as_str() {
            "fold" => crate::units::Dim::ANGLE,
            "offset" => crate::units::Dim::LENGTH,
            _ => crate::units::Dim::SCALAR,
        };
        for a in &h.args {
            let crate::syntax::Arg::Dim { text, span } = a else { continue };
            let got = crate::flatten::value_aff(text, &BTreeMap::new(), sk.units)
                .and_then(|v| v.number().ok_or_else(|| "a seed is a number".to_string())
                    .and_then(|_| v.dim.require(want, &h.key.text).map_err(|e| e.to_string())));
            if let Err(m) = got {
                fail(diags, Code::E103, *span, format!("`{text}`: {m}"));
            }
        }
    }
    if matches!(d.attitude, Attitude::Free { .. })
        && d.plane.hint("u").is_some() != d.plane.hint("v").is_some()
    {
        let h = d.plane.hint("u").or(d.plane.hint("v")).expect("one of them");
        fail(diags, Code::E103, h.key.span, "a free attitude is seeded by both `u:` and `v:`".into());
    }
    // **a position is stated once**: a written offset, or a fold along a line (which stands the
    // view where the line is), beside `offset: free` or `through:` is two answers to one number
    let position_span = match &d.plane.position {
        Position::Stated => None,
        Position::Free(sp) => Some(*sp),
        Position::Through(r) => Some(r.span),
    };
    if let Some(sp) = position_span {
        let twice = match &d.attitude {
            Attitude::Offset { offset: Some(_), .. } => Some("`offset:` already says"),
            Attitude::Along { .. } => Some("a fold `along` a line already stands it where the line is"),
            _ => None,
        };
        if let Some(w) = twice {
            fail(diags, Code::E064, sp, format!("where `{name}` stands is stated twice: {w}"));
            return;
        }
    }
    // a view derived from a parent follows the parent's origin, which it can do only while that
    // origin is a constant of the parent's attitude — on its normal through the shared origin,
    // and held there — or the child's own constants would have to move with it
    let derived_from_solved = parent_solved
        || free_fold.is_some()
        || matches!(d.attitude, Attitude::Along { .. });
    if let (Some(p), true) = (parent, derived_from_solved) {
        if let Some(a) = sk.planes[p].att.as_ref() {
            if !sk.params[a.d as usize].fixed {
                let span = d.attitude.plane_ref().map_or(st.span, |r| r.span);
                fail(diags, Code::E064, span, format!(
                    "`{}`'s offset is solved, and a view derived from it would have to follow it \
                     — fold from a view whose offset is stated",
                    sk.plane_name(p)
                ));
                return;
            }
        }
        let turns = free_fold.is_some() || matches!(d.attitude, Attitude::Along { .. });
        let ab = sk.planes[p].att.as_ref().map_or_else(|| {
            let b = sk.basis(p);
            [crate::plane::dot(b.o, b.u), crate::plane::dot(b.o, b.v)]
        }, |a| a.ab);
        if turns && ab[0].hypot(ab[1]) > ON_NORMAL * (1.0 + sk.extent()) {
            let span = d.attitude.plane_ref().map_or(st.span, |r| r.span);
            fail(diags, Code::E064, span, format!(
                "`{}` stands off the shared origin in its own plane, and a solved fold turns \
                 about a line through that origin",
                sk.plane_name(p)
            ));
            return;
        }
    }
    // -- the seeded pose, before anything is minted: a fold along a line stands on the line's
    // bearing and through its first end, and a plane `through:` a point stands through it
    let mut hinges: Vec<Constraint> = Vec::new();
    let mut rows: Vec<Constraint> = Vec::new();
    let mut theta_along = None;
    if let Attitude::Along { line, .. } = &d.attitude {
        let p = parent.expect("an `along` fold names its parent");
        let l = match res.lookup(line).map(|e| follow(sk, e, &line.path)) {
            None => {
                fail(diags, Code::E101, line.span, format!("no such entity: `{}`", line.root.text));
                return;
            }
            Some(Err(m)) => {
                fail(diags, Code::E101, line.span, m);
                return;
            }
            Some(Ok(e)) if e.kind != EntKind::Line => {
                fail(diags, Code::E040, line.span, format!(
                    "`{}` is a {}, and a fold is taken along a line",
                    crate::syntax::ref_text(line), e.kind.as_str()
                ));
                return;
            }
            Some(Ok(e)) => e,
        };
        if super::planes::plane_of_entity(sk, l) != Some(p) {
            fail(diags, Code::E064, line.span, format!(
                "`{}` is not drawn in `{}`, and a fold is taken along a line of the view it \
                 folds from",
                crate::syntax::ref_text(line), sk.plane_name(p)
            ));
            return;
        }
        let ln = &sk.lines[l.i()];
        let (p1, p2) = (ln.p1 as usize, ln.p2 as usize);
        let f = &sk.planes[p].frame;
        let (c, s) = (sk.params[f.c as usize].value, sk.params[f.s as usize].value);
        let o = sk.point_xy(f.origin as usize);
        let (a1, b1) = crate::plane::in_view(c, s, o, sk.point_xy(p1));
        let (a2, b2) = crate::plane::in_view(c, s, o, sk.point_xy(p2));
        let theta = (b2 - b1).atan2(a2 - a1);
        let folded = sk.basis(p).fold(theta);
        sk.set_basis(pi, stand_through(folded, sk.world_point(p1)));
        theta_along = Some(theta);
        let (hs, hc) = (0.5 * theta).sin_cos();
        hinges.push(Constraint::new(CKind::HingeAlong, vec![
            CArg::Ent(EntRef::plane(pi)),
            CArg::Ent(EntRef::plane(p)),
            CArg::Ent(l),
            CArg::Seed { value: hc, pinned: false },
            CArg::Seed { value: hs, pinned: false },
        ]));
        rows.push(Constraint::new(CKind::PointOnPlane, vec![
            CArg::Ent(EntRef::point(p1)),
            CArg::Ent(EntRef::plane(pi)),
        ]));
    }
    if let Position::Through(r) = &d.plane.position {
        let m = match res.lookup(r).map(|e| follow(sk, e, &r.path)) {
            None => {
                fail(diags, Code::E101, r.span, format!("no such entity: `{}`", r.root.text));
                return;
            }
            Some(Err(m)) => {
                fail(diags, Code::E101, r.span, m);
                return;
            }
            Some(Ok(e)) if e.kind != EntKind::Point => {
                fail(diags, Code::E040, r.span, format!(
                    "`{}` is a {}, and a plane is stood `through:` a point",
                    crate::syntax::ref_text(r), e.kind.as_str()
                ));
                return;
            }
            Some(Ok(e)) => e.i(),
        };
        match sk.plane_of(m) {
            None => {
                fail(diags, Code::E040, r.span, format!(
                    "`{}` is on no view, so where it stands in space is not drawn anywhere",
                    crate::syntax::ref_text(r)
                ));
                return;
            }
            Some(v) if v == pi => {
                fail(diags, Code::E064, r.span, format!(
                    "`{}` is drawn in `{name}` itself, which puts it in the plane whatever the \
                     offset is: `through:` names a point of another view",
                    crate::syntax::ref_text(r)
                ));
                return;
            }
            Some(_) => {}
        }
        let b = sk.basis(pi);
        sk.set_basis(pi, stand_through(b, sk.world_point(m)));
        rows.push(Constraint::new(CKind::PointOnPlane, vec![
            CArg::Ent(EntRef::point(m)),
            CArg::Ent(EntRef::plane(pi)),
        ]));
    }
    // -- the attitude's unknowns
    let hold_parent = |sk: &mut Sketch, p: usize| {
        if sk.planes[p].att.is_none() {
            sk.free_attitude(p);
            sk.fix_attitude(p, true);
        }
    };
    let q_parent = |sk: &Sketch, p: usize| {
        let a = sk.planes[p].att.as_ref().expect("held or solved");
        a.q.map(|k| sk.params[k as usize].value)
    };
    match &d.attitude {
        Attitude::Free { .. } => sk.free_attitude(pi),
        Attitude::Along { .. } => {
            let p = parent.expect("an `along` fold names its parent");
            hold_parent(sk, p);
            let rel = crate::plane::fold_rotor(theta_along.expect("read above"));
            sk.hinge_attitude(pi, crate::plane::quat_mul(q_parent(sk, p), rel));
        }
        Attitude::From { fold, .. } if free_fold.is_some() || parent_solved => {
            let Some(p) = parent else { return };
            hold_parent(sk, p);
            let (text, deg) = match (&free_fold, fold) {
                (Some(a), crate::syntax::Arg::Dim { text, .. }) => {
                    // where the basis was folded to: the seed, or the expression at nothing
                    let deg = d.plane.hint("fold")
                        .and_then(|h| h.args.first())
                        .and_then(|a| match a {
                            crate::syntax::Arg::Dim { text, .. } => {
                                crate::flatten::value_aff(text, &BTreeMap::new(), sk.units).ok()
                            }
                            _ => None,
                        })
                        .and_then(|a| a.number())
                        .unwrap_or(a.c);
                    (Some(text.clone()), deg)
                }
                (_, _) => (None, fold_aff(fold, sk.units).map_or(0.0, |a| a.c)),
            };
            let theta = crate::expr::to_arg_units(SpecKind::Angle, deg);
            let rel = crate::plane::fold_rotor(theta);
            sk.hinge_attitude(pi, crate::plane::quat_mul(q_parent(sk, p), rel));
            let angle = match text {
                Some(t) => CArg::Expr(crate::expr::Expr::new(t, theta)),
                None => CArg::Num(theta),
            };
            hinges.push(Constraint::new(CKind::Hinge, vec![
                CArg::Ent(EntRef::plane(pi)),
                CArg::Ent(EntRef::plane(p)),
                angle,
            ]));
        }
        Attitude::Offset { .. } if parent_solved => {
            let p = parent.expect("an offset names its parent");
            sk.hinge_attitude(pi, q_parent(sk, p));
            hinges.push(Constraint::new(CKind::HingeParallel, vec![
                CArg::Ent(EntRef::plane(pi)),
                CArg::Ent(EntRef::plane(p)),
            ]));
        }
        _ => {}
    }
    // -- the offset's
    match (&d.plane.position, &d.attitude) {
        (Position::Stated, Attitude::Along { .. }) => sk.fix_offset(pi, false),
        (Position::Stated, _) => sk.fix_offset(pi, true),
        (Position::Free(_) | Position::Through(_), _) => {
            if sk.planes[pi].att.is_none() {
                sk.free_attitude(pi);
                sk.fix_turn(pi, true);
            }
            sk.fix_offset(pi, false);
        }
    }
    for c in hinges.into_iter().chain(rows) {
        let id = sk.add_quiet(c);
        map.record(st, Made::Con(id));
    }
}

/// `b` moved along its own normal until `x` is in it.
fn stand_through(b: crate::plane::Basis, x: [f64; 3]) -> crate::plane::Basis {
    let n = b.normal();
    let gap = crate::plane::dot(n, [x[0] - b.o[0], x[1] - b.o[1], x[2] - b.o[2]]);
    b.offset(gap)
}

/// **What a solve can make degenerate that no stated number said** (E065): two views a
/// `project` relates that came out parallel, which share no fold line and so say nothing, and
/// two lines whose skew distance is stated that came out parallel, which have no common
/// perpendicular to measure.  Asked of the solved sketch — where the views are stated, the first
/// is refused where it is written (E061) and nothing here can fire.
pub(crate) fn degenerate(sk: &Sketch, map: &SourceMap) -> Vec<Diag> {
    let mut out = Vec::new();
    for c in &sk.constraints {
        let message = match c.kind {
            CKind::ProjectSolved => {
                let (a, b) = (c.args[2].ent().i(), c.args[3].ent().i());
                let (na, nb) = (sk.basis(a).normal(), sk.basis(b).normal());
                (crate::space::norm(crate::space::cross(na, nb)) <= crate::plane::PARALLEL_TOL)
                    .then(|| format!(
                        "`{}` and `{}` came out parallel, so no fold line relates their views \
                         and the projection says nothing",
                        sk.plane_name(a), sk.plane_name(b)
                    ))
            }
            CKind::LineLine3 | CKind::CylinderTangentLine => {
                let dir = |i: usize| {
                    // a cylinder's line is its axis (P4)
                    let e = c.args[i].ent();
                    let e = if e.kind == crate::model::EntKind::Cylinder {
                        crate::model::EntRef::line(sk.axial(e).axis as usize)
                    } else {
                        e
                    };
                    let l = &sk.lines[e.i()];
                    crate::space::sub(sk.lifted(l.p2 as usize), sk.lifted(l.p1 as usize))
                };
                let (a, b) = (dir(0), dir(1));
                let m = crate::space::norm(crate::space::cross(a, b));
                (m <= crate::plane::PARALLEL_TOL * crate::space::norm(a) * crate::space::norm(b))
                    .then(|| "the two lines came out parallel in space, and parallel lines have \
                              no common perpendicular to measure"
                        .to_string())
            }
            _ => None,
        };
        if let Some(message) = message {
            let site = map.site_of_constraint(c.id);
            out.push(Diag {
                code: Code::E065,
                span: site.map(|s| s.span).unwrap_or_default(),
                stmt: site.map(|s| s.stmt),
                message,
            });
        }
    }
    out
}
