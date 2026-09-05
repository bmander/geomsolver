//! Build primitive entities and resolve their initial seeds.

use super::curves::build_curve;
use super::resolve::{follow, follow_building, Resolver};
use super::{Code, Diag};
use crate::ir::{Decl, Kid, Statement as Stmt};
use crate::model::{EntKind, EntRef, Field, Sketch};
use crate::rng::Rng;
use crate::style::Classes;
use crate::syntax::{AtRef, Name, Program, Seg, Span, StmtId};
use crate::{curve, expr, io};
use std::collections::BTreeMap;

/// Default radius for unseeded circles and degenerate arcs. Keep it nonzero so
/// the radius gradient can move the initial geometry.
const UNSEEDED_RADIUS: f64 = 1.0;

fn scatter(i: usize) -> (f64, f64) {
    // the bearing walks a fixed step per minted point, in creation order — which for a chain's
    // corners is traversal order, so a contour of implicit points seeds as a *simple polygon*
    // and not as a pile or a self-crossing quad, whose nearest solution is a collapsed side (a
    // zero-length line satisfies every direction constraint on it).  The step is irrational in
    // turns (half the golden angle), so no later point lands back on an earlier bearing.
    const STEP: f64 = 1.199982;
    let mut rng = Rng::new(0x5eed_u32 ^ (i as u32).wrapping_mul(2_654_435_761));
    let th = i as f64 * STEP + rng.uniform(-0.2, 0.2);
    let r = rng.uniform(0.8, 1.2);
    (r * th.cos(), r * th.sin())
}

/// Child display names under `base`, such as `l.p1` and `a.center`.
/// Anonymous declarations use their generated display name as the base.
pub(crate) fn child_names(kind: EntKind, base: &str) -> Vec<String> {
    kind.fields()
        .iter()
        .filter(|(_, f)| *f == Field::Child)
        .map(|(n, _)| format!("{base}.{n}"))
        .collect()
}

/// Use the declared display name, or a positional model name for an anonymous
/// declaration. Internal resolution keys must not appear in parameter labels.
fn shown(sk: &Sketch, d: &Decl) -> String {
    match d.name.shown() {
        None => crate::syntax::entity_name(EntRef::new(d.kind, sk.count(d.kind))),
        Some(n) => n.text.clone(),
    }
}
#[allow(clippy::too_many_arguments)]
pub(super) fn build(
    sk: &mut Sketch,
    res: &Resolver,
    d: &Decl,
    st: &Stmt,
    bases: &BTreeMap<String, crate::plane::Basis>,
    diags: &mut Vec<Diag>,
    anon: &mut Vec<(String, EntRef)>,
    deferred: &mut Vec<Deferred>,
    prog: &Program,
    insts: &[crate::flatten::InstanceInfo],
) -> Option<EntRef> {
    // a curve is the one kind whose arguments need not be points, so it is built before the
    // walk that insists they are
    if d.kind == EntKind::Curve {
        return build_curve(sk, res, d, st, diags, prog, insts);
    }
    // a seed named by a place (`hint(at: k, bearing: b)`) is a point's; every other kind has
    // a scalar of its own the clause seeds by name
    if d.seed_at.is_some() && d.kind != EntKind::Point {
        diags.push(Diag {
            code: Code::E103,
            span: st.span,
            stmt: Some(st.id),
            message: "only a point takes a geometric seed (`hint(at: …)`)".to_string(),
        });
        return None;
    }
    // every child a declaration names, flattened in field order and checked to be a Point —
    // which every other child of every other kind is, and which is what an alias class must
    // agree about.  A slot may hold a *seed* instead of a name, and a declaration may write no
    // list at all: both mint a point nothing names, reached as `l.p1` (spec §6.1, §6.2).
    let written: usize = d.children.iter().map(|g| g.len()).sum();
    // the dotted names, worked out only where one is needed: every child a document names is a
    // `Kid::Ref`, and formatting names nothing would read is a string per slot per elaboration
    let anonymous = written == 0
        || d.children.iter().any(|g| g.is_empty() || g.iter().any(|k| matches!(k, Kid::Hint(_))));
    // The child's **name**, which is its dotted path, and what to **call** it — the same string
    // for a declaration the source named, and two different ones for an anonymous declaration,
    // whose key is an offset nobody should be shown.
    let dotted = if anonymous { child_names(d.kind, &d.name.key().text) } else { Vec::new() };
    let label = if anonymous { child_names(d.kind, &shown(sk, d)) } else { Vec::new() };
    let mut kids: Vec<usize> = Vec::new();
    // `Some(0)` and `None` both mean there is nothing to mint, and so does a written list
    let mint = if written == 0 { d.kind.children_arity().unwrap_or(0) } else { 0 };
    for k in 0..mint {
        let (x, y) = scatter(sk.points.len());
        // the dotted path *is* the point's name — there is no other — so it is what the map
        // binds; the sketch carries what a reader is shown
        let i = sk.point(x, y, false, &label[k]);
        kids.push(i);
        anon.push((dotted[k].clone(), EntRef::point(i)));
    }
    let mut slot = 0usize;
    for group in &d.children {
        // a slot nothing names or seeds is an *implicit child*, minted exactly as a declaration
        // that writes no list at all mints them (spec §6.2) — which is what lets a chain's
        // thread fill only the slots it speaks for (`line l1 -> line l2`) and leave the rest
        // the drawing's own
        if group.is_empty() && written != 0 {
            // a `List` kind has no arity to mint from, and a slot with no dotted path has no
            // name to be reached by
            if let Some(name) = d.kind.children_arity().and(dotted.get(slot)) {
                let (x, y) = scatter(sk.points.len());
                let i = sk.point(x, y, false, label.get(slot).unwrap_or(name));
                kids.push(i);
                anon.push((name.clone(), EntRef::point(i)));
            }
            slot += 1;
            continue;
        }
        for kid in group {
            let r = match kid {
                Kid::Ref(r) => r,
                Kid::Face { .. } => return None, // only solids accept inline sections
                Kid::Hint(seed) => {
                    // a list slot has no arity, so it has no dotted path to be named by, and a
                    // point nothing can name is a point nothing can constrain or drag
                    let Some(name) = dotted.get(slot) else {
                        diags.push(Diag {
                            code: Code::E103,
                            span: st.span,
                            stmt: Some(st.id),
                            message: format!(
                                "a {}'s control points have no names to be reached by, so each \
                                 one is declared",
                                d.kind.as_str()
                            ),
                        });
                        return None;
                    };
                    let i = sk.point(seed.v[0], seed.v[1], false, label.get(slot).unwrap_or(name));
                    for (k, t) in seed.text.iter().enumerate() {
                        if let Some(t) = t {
                            deferred.push(Deferred::Text {
                                param: sk.point_params(i)[k],
                                text: t.clone(),
                                names: d.seed_names.clone(),
                                span: seed.spans[k],
                                stmt: st.id,
                            });
                        }
                    }
                    kids.push(i);
                    anon.push((name.clone(), EntRef::point(i)));
                    slot += 1;
                    continue;
                }
            };
            slot += 1;
            let Some(e) = res.lookup(r) else {
                diags.push(Diag {
                    code: Code::E101,
                    span: r.span,
                    stmt: Some(st.id),
                    message: format!("no such entity: `{}`", r.root.text),
                });
                return None;
            };
            let e = match follow_building(sk, res, e, r) {
                Ok(e) => e,
                Err(msg) => {
                    diags.push(Diag {
                        code: Code::E040,
                        span: r.span,
                        stmt: Some(st.id),
                        message: msg,
                    });
                    return None;
                }
            };
            if e.kind != EntKind::Point {
                diags.push(Diag {
                    code: Code::E040,
                    span: r.span,
                    stmt: Some(st.id),
                    message: format!(
                        "`{}` is a {}, and a {} is built from points",
                        r.root.text,
                        e.kind.as_str(),
                        d.kind.as_str()
                    ),
                });
                return None;
            }
            kids.push(e.i());
        }
    }
    // a slot carries a name, a seed, or nothing — an implicit child, minted above — so the one
    // thing left to refuse is *more* than the kind has slots for
    let want = d.kind.children_arity();
    if let Some(n) = want {
        if kids.len() != n {
            diags.push(Diag {
                code: Code::E103,
                span: st.span,
                stmt: Some(st.id),
                message: format!(
                    "a {} is built from {n} point(s), and {} were given",
                    d.kind.as_str(),
                    written
                ),
            });
            return None;
        }
    }
    let seed = |i: usize| d.seed.get(i).copied().unwrap_or(0.0);
    // what a reader is shown: the declaration's own name, or what the drawing calls it where
    // the source named nothing — a scalar carries this into every list of parameters
    let show = shown(sk, d);
    // A point whose source wrote no seed at all — no `hint(…)` clause (the empty span where
    // one would go) and no place — starts where a minted child does, not at the origin: two
    // such points on top of each other put every distance between them at a stationary point
    // of its own residual, and the first document anybody writes solved as a conflict (#43).
    // A declaration lifted from a sketch has no span (`None`) and carries its numbers.
    let unseeded = d.unseeded;
    // A scalar the source never wrote reads as 0, and for a radius 0 is a stationary point of
    // every on-circle row (∂/∂r of |p−c|² − r² is −2r): an `arc` with its ends grounded and no
    // `hint(r:)` could not solve at all, and a conflict elsewhere in the drawing was blamed on
    // the arc's own intrinsic, the first row a search from that pose could not satisfy (#45.6).
    // So a radius is *written or computed*, never defaulted: the constructor's geometric one for
    // an arc, `UNSEEDED_RADIUS` for a circle and wherever the
    // geometry gives nothing.  A declaration lifted from a sketch has no spans and carries its
    // numbers, as for a point above.
    let wrote = |i: usize| d.seed_explicit.get(i).copied().unwrap_or(false);
    let nonzero = |r: f64| if r.abs() > 1e-9 { r } else { UNSEEDED_RADIUS };
    let idx = match d.kind {
        // built by their own phase, after every other kind: a face is written over edges and a
        // solid over faces and solids, so neither can be minted by the walk that makes points
        EntKind::Face | EntKind::Solid => return None,
        EntKind::Point if unseeded => {
            let (x, y) = scatter(sk.points.len());
            sk.point(x, y, false, &show)
        }
        EntKind::Point => sk.point(seed(0), seed(1), false, &show),
        EntKind::Line => sk.line(kids[0], kids[1]),
        EntKind::Circle => {
            let r = if wrote(0) { seed(0) } else { UNSEEDED_RADIUS };
            sk.circle(kids[0], r, &show)
        }
        EntKind::Arc => {
            // `arc` adds the two intrinsic `PointOnCircle`s here and nowhere else, and computes a
            // radius from the geometry that a *written* seed then replaces
            let ai = sk.arc(kids[0], kids[1], kids[2], &show);
            let rp = sk.arcs[ai].radius as usize;
            sk.params[rp].value = if wrote(0) { seed(0) } else { nonzero(sk.params[rp].value) };
            ai
        }
        EntKind::Spline => {
            if kids.len() > io::MAX_CTRL {
                diags.push(Diag {
                    code: Code::E104,
                    span: st.span,
                    stmt: Some(st.id),
                    message: format!(
                        "a curve may not have more than {} control points",
                        io::MAX_CTRL
                    ),
                });
                return None;
            }
            match sk.spline_with(&kids, d.knots.clone()) {
                Some(si) => si,
                None => {
                    diags.push(Diag {
                        code: Code::E103,
                        span: st.span,
                        stmt: Some(st.id),
                        message: format!(
                            "a curve needs more than {} control points and a matching knot vector",
                            curve::DEGREE
                        ),
                    });
                    return None;
                }
            }
        }
        EntKind::Plane => {
            // `plane` adds the two intrinsics here and nowhere else, and computes a rotor from
            // the chord that a declared seed then replaces — except (0, 0), which is no rotor
            // at all and is what an unwritten seed reads as — over a basis the attitude pass
            // resolved before this walk; a plane whose basis was refused has no entry and is
            // not built
            let basis = *bases.get(&d.name.key().text)?;
            let pi = sk.plane(kids[0], kids[1], basis, &show);
            // **a plane written `from: P` with neither clause is one a mate places** (§6.10):
            // it says which plane it is parallel to and leaves where it stands to one `against`.
            // Recorded here, where the attitude as *written* is still in hand — after this the
            // basis is a number and says nothing about how it was arrived at.
            if matches!(&d.attitude, crate::syntax::Attitude::Offset { offset: None, .. }) {
                sk.placed_planes.insert(pi as u32);
            }
            if let Some(n) = d.name.shown() {
                sk.plane_names.insert(pi as u32, n.text.clone());
            }
            let (c, s) = (seed(0), seed(1));
            if c != 0.0 || s != 0.0 {
                let f = &sk.planes[pi].frame;
                let (cp, sp) = (f.c as usize, f.s as usize);
                sk.params[cp].value = c;
                sk.params[sp].value = s;
            }
            pi
        }
        EntKind::Curve => unreachable!("a curve is built before this walk"),
    };
    let e = EntRef::new(d.kind, idx);
    set_class(sk, e, d.class.clone());
    // the seeds this declaration wrote over geometry, settled once everything is built: a
    // point's place, and any scalar's text that read another entity's
    if let Some(at) = &d.seed_at {
        deferred.push(Deferred::At {
            point: idx,
            at: at.clone(),
            names: d.seed_names.clone(),
            span: st.span,
            stmt: st.id,
        });
    }
    // the declaration's own scalars are the last of `entity_params`, after its children's
    let params = sk.entity_params(e);
    let own = &params[params.len().saturating_sub(d.seed_text.len())..];
    for (k, t) in d.seed_text.iter().enumerate() {
        if let (Some(t), Some(param)) = (t, own.get(k).copied()) {
            let span = d.seed_spans.get(k).copied().unwrap_or(st.span);
            deferred.push(Deferred::Text {
                param,
                text: t.clone(),
                names: d.seed_names.clone(),
                span,
                stmt: st.id,
            });
        }
    }
    Some(e)
}

/// Defer geometric seed expressions until all declarations have initial values.
/// These initialize parameters without adding constraints.
pub(super) enum Deferred {
    Text { param: u32, text: String, names: Vec<(String, String)>, span: Span, stmt: StmtId },
    At { point: usize, at: AtRef, names: Vec<(String, String)>, span: Span, stmt: StmtId },
}

/// The seed a dotted name reads: `pin.x`, `k.center.y`, `base.r`, `e.b` — `dotted` as the
/// flattener resolved it, its root the entity's absolute name, and the last segment the scalar by
/// the kind's own `scalar_names`.
fn seed_read(sk: &Sketch, res: &Resolver, dotted: &str) -> Result<f64, String> {
    let (path, scalar) =
        dotted.rsplit_once('.').ok_or_else(|| format!("`{dotted}` is not a number here"))?;
    // an entity's absolute name is dotted itself (`t1.pin` under an instance), so the entity is
    // the longest head of the path that names one, and the rest is fields into it
    let segs: Vec<&str> = path.split('.').collect();
    let (e, fields) = (1..=segs.len())
        .rev()
        .find_map(|k| res.of.get(&segs[..k].join(".")).map(|e| (*e, &segs[k..])))
        .ok_or_else(|| format!("no such entity: `{}`", segs[0]))?;
    let fields: Vec<Seg> = fields.iter().map(|f| Seg::Field(Name::new(*f))).collect();
    let e = follow(sk, e, &fields)?;
    let names = e
        .kind
        .scalar_names(path)
        .ok_or_else(|| format!("a {} has no scalar to read by name", e.kind.as_str()))?;
    let at = names.iter().position(|n| *n == dotted).ok_or_else(|| {
        format!(
            "a {} has no `{scalar}`; its scalars are {}",
            e.kind.as_str(),
            names.iter().map(|n| format!("`{n}`")).collect::<Vec<_>>().join(", ")
        )
    })?;
    let p = *sk.entity_params(e).get(at).ok_or_else(|| format!("`{dotted}` has no seed yet"))?;
    Ok(sk.params[p as usize].value)
}

/// An expression over geometry's seeds, come to its number.  `names` is what each dotted name
/// in it resolved to (`Decl::seed_names`); one it does not list is read as written.
fn seed_eval(
    sk: &Sketch,
    res: &Resolver,
    text: &str,
    names: &[(String, String)],
) -> Result<f64, String> {
    let p = expr::parse_in(text, sk.units)?;
    let mut env: BTreeMap<String, expr::Aff> = BTreeMap::new();
    // a coordinate or a radius is a length, so it adds to `150mm` and not to a plain number —
    // where the document names a unit.  Where it does not, no literal can be a length, so the
    // read is the bare number everything else there is (§3.3).
    let dim = if sk.units.name().is_some() {
        crate::units::Dim::LENGTH
    } else {
        crate::units::Dim::SCALAR
    };
    for dep in p.body.deps() {
        if dep.contains('.') {
            let abs = names.iter().find(|(w, _)| *w == dep).map(|(_, a)| a.as_str());
            let v = seed_read(sk, res, abs.unwrap_or(&dep))?;
            env.insert(dep.clone(), expr::Aff::of_dim(v, dim));
        }
    }
    let a = expr::eval(&p.body, &env)?;
    match a.number() {
        Some(v) if v.is_finite() => Ok(v),
        Some(v) => Err(format!("comes to {v}")),
        None => Err(format!(
            "`{}` is not a number here — a component's parameters are, and the document's \
             dimensions are not",
            a.free.unwrap_or_default()
        )),
    }
}

pub(super) fn settle_deferred(
    sk: &mut Sketch,
    res: &Resolver,
    deferred: &[Deferred],
    diags: &mut Vec<Diag>,
) {
    for d in deferred {
        let (span, stmt, result) = match d {
            Deferred::Text { param, text, names, span, stmt } => {
                let r = seed_eval(sk, res, text, names).map(|v| {
                    sk.params[*param as usize].value = v;
                });
                (*span, *stmt, r.map_err(|e| format!("`{text}`: {e}")))
            }
            Deferred::At { point, at, names, span, stmt } => {
                let r = place_of(sk, res, at, names).map(|(x, y)| {
                    let [px, py] = sk.point_params(*point);
                    sk.params[px as usize].value = x;
                    sk.params[py as usize].value = y;
                });
                (*span, *stmt, r)
            }
        };
        if let Err(message) = result {
            diags.push(Diag { code: Code::E103, span, stmt: Some(stmt), message });
        }
    }
}

/// Where `hint(at: …)` puts a point on the sheet: the seed of the point it names, or the edge
/// of the circle it names at the bearing given — `at_seed`'s two places, read as numbers.
fn place_of(
    sk: &Sketch,
    res: &Resolver,
    a: &AtRef,
    names: &[(String, String)],
) -> Result<(f64, f64), String> {
    let e = *res
        .of
        .get(&a.what.root.text)
        .ok_or_else(|| format!("no such entity: `{}`", a.what.root.text))?;
    let e = follow(sk, e, &a.what.path)?;
    match (e.kind, &a.bearing) {
        (EntKind::Point, None) => Ok(sk.point_xy(e.i())),
        (EntKind::Point, Some(_)) => {
            Err("a point is already a place; a bearing needs a circle".to_string())
        }
        (EntKind::Circle, Some((text, _))) => {
            let c = &sk.circles[e.i()];
            let (cx, cy) = sk.point_xy(c.center as usize);
            let r = sk.params[c.radius as usize].value;
            // a bearing is an angle, so the text may say `90deg` or read a `param` already
            // written in; what it comes to is in the document's angle unit, which is degrees
            let b = seed_eval(sk, res, text, names).map_err(|m| format!("`{text}`: {m}"))?;
            let b = b.to_radians();
            Ok((cx + r * b.cos(), cy + r * b.sin()))
        }
        (EntKind::Circle, None) => {
            Err("where on the edge?  `hint(at: c, bearing: …)` says the bearing".to_string())
        }
        (k, _) => Err(format!("a seed cannot be at a {}", k.as_str())),
    }
}

fn set_class(sk: &mut Sketch, e: EntRef, c: Classes) {
    match e.kind {
        EntKind::Point => {}
        EntKind::Line => sk.lines[e.i()].class = c,
        EntKind::Curve => sk.curves[e.i()].class = c,
        EntKind::Circle => sk.circles[e.i()].class = c,
        EntKind::Arc => sk.arcs[e.i()].class = c,
        EntKind::Spline => sk.splines[e.i()].class = c,
        EntKind::Plane => sk.planes[e.i()].frame.class = c,
        EntKind::Face => sk.faces[e.i()].class = c,
        EntKind::Solid => sk.solids[e.i()].class = c,
    }
}
