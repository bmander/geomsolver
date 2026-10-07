//! A `ring`'s copies, turned (Solvent §12.3–12.4, issue #96).
//!
//! The flattener expands a ring as the cycle it is — every copy's declarations, so each copy is
//! an entity a face, a callout or `tip[2]` may name — and marks every statement of a copy past
//! the first `turned`: its relations, holds and claims are the representative's turned, and are
//! never stated.  What makes the copies turns is here: each entity a turned copy declared is
//! paired, by its name under the ring, with the representative's of the same name, and recorded
//! as that one turned `k` steps about the ring's centre (`model::Turn`).  Its numbers are then
//! derived, and the solve is over the representative's alone.

use super::resolve::Resolver;
use super::source_map::{public_path, SourceMap};
use super::{Code, Diag};
use crate::constraints::Arg;
use crate::flatten::RingInfo;
use crate::model::{EntKind, EntRef, Sketch, Turn};
use std::collections::BTreeSet;

fn refuse(diags: &mut Vec<Diag>, span: crate::syntax::Span, message: String) {
    if !diags.iter().any(|d| d.code == Code::E023 && d.span == span) {
        diags.push(Diag { code: Code::E023, span, stmt: None, message });
    }
}

/// What the source calls an entity, for a message: its public path (`tip[1]`), else its kind.
fn called(map: &SourceMap, e: EntRef) -> String {
    map.name_of(e).map(|n| format!("`{}`", public_path(n))).unwrap_or_else(|| e.kind.a())
}

/// Record every ring's turned copies — after the holds, so a hold on a copy can be told, and
/// before anything reads a copy's numbers.
pub(super) fn rings(
    sk: &mut Sketch,
    res: &Resolver,
    map: &SourceMap,
    rings: &[RingInfo],
    diags: &mut Vec<Diag>,
) {
    for ring in rings {
        let Some(about) = res.lookup(&ring.about)
            .and_then(|e| super::resolve::follow(sk, e, &ring.about.path).ok())
        else {
            continue;
        };
        let centre = public_path(&crate::flatten::written_name(&ring.about));
        match about.kind {
            EntKind::Point if sk.points[about.i()].z.is_some() => {
                refuse(diags, ring.about.span, format!(
                    "`{centre}` stands in space, where a turn about a point has no plane: turn \
                     about an axis, or about a point drawn in a view"));
                continue;
            }
            EntKind::Point => {}
            EntKind::Axis if !sk.axes[about.i()].d.iter().all(|&p| sk.params[p as usize].fixed) => {
                refuse(diags, ring.about.span, format!(
                    "a `ring` about an axis turns about its direction, and `{centre}`'s is not \
                     held: hold it (`fix(dir == (…)) {centre}`), or turn about a point drawn in \
                     a view"));
                continue;
            }
            EntKind::Axis => {}
            k => {
                refuse(diags, ring.about.span, format!(
                    "a `ring` turns about a point or an axis, and `{centre}` is {}", k.a()));
                continue;
            }
        }
        let view = sk.points.get(about.i()).map(|p| p.plane).filter(|_| about.kind == EntKind::Point);
        let mut seen: BTreeSet<EntRef> = BTreeSet::new();
        let mut pairs: Vec<(EntRef, EntRef, u32)> = Vec::new();
        for (key, copy) in map.keys() {
            let Some((k, tail)) = key.strip_prefix(&ring.prefix).and_then(|r| r.split_once('.')) else {
                continue;
            };
            let Ok(k) = k.parse::<u32>() else { continue };
            if k == 0 || k >= ring.n || !seen.insert(copy) {
                continue;
            }
            let Some(rep) = map.ent_named(&format!("{}0.{tail}", ring.prefix)) else { continue };
            if rep.kind == copy.kind && rep != copy {
                pairs.push((copy, rep, k));
            }
        }
        for (copy, rep, k) in pairs {
            let fits = match copy.kind {
                EntKind::Point => {
                    let at = |e: EntRef| (sk.points[e.i()].plane, sk.points[e.i()].z.is_some());
                    match view {
                        Some(v) => at(copy) == (v, false) && at(rep) == (v, false),
                        None => at(copy).1 && at(rep).1,
                    }
                }
                EntKind::Circle | EntKind::Arc => true,
                EntKind::Curve => about.kind == EntKind::Point,
                // made of points, which are turned themselves; or owning no number at all
                EntKind::Line | EntKind::Spline | EntKind::Face | EntKind::Solid => continue,
                _ => {
                    refuse(diags, ring.span, format!(
                        "{} is declared in a `ring`, which turns points, lines, circles, arcs and \
                         curves: declare it outside", called(map, rep)));
                    continue;
                }
            };
            if !fits {
                let why = match view {
                    Some(_) => format!("is not drawn in `{centre}`'s view, and a ring about a point \
                                        turns within it"),
                    None => "is not a point in space, which a ring about an axis turns".to_string(),
                };
                refuse(diags, ring.span, format!("{} {why}", called(map, rep)));
                continue;
            }
            // a copy's numbers are its representative's: holding one holds nothing of its own
            if copy.kind == EntKind::Point
                && sk.point_all_params(copy.i()).iter().any(|&p| sk.params[p as usize].fixed)
            {
                refuse(diags, ring.span, format!(
                    "{} is held, but it is {} turned: hold that instead",
                    called(map, copy), called(map, rep)));
                continue;
            }
            sk.turns.push(Turn { copy, rep, about, k, n: ring.n });
        }
    }
    sk.settle_turns();
}

/// A contact on a turned curve: the curve is evaluated as its representative turned, and no
/// kernel reads it so — state the contact on the representative (E023).
pub(super) fn contacts(sk: &Sketch, map: &SourceMap, diags: &mut Vec<Diag>) {
    if sk.turns.is_empty() {
        return;
    }
    for c in &sk.constraints {
        let turned = c.args.iter().find_map(|a| match a {
            Arg::Ent(e) if e.kind == EntKind::Curve && sk.turn_of(*e).is_some() => Some(*e),
            _ => None,
        });
        let (Some(e), Some(site)) = (turned, map.site_of_constraint(c.id)) else { continue };
        refuse(diags, site.span, format!(
            "{} is a `ring`'s turned copy of a curve, which a relation cannot read yet: state it \
             on the representative", called(map, e)));
    }
}
