//! **Corner rounds** (#576): the arc a chain's `A -> round(r: R) B` joint mints, settled once the
//! drawing is seeded.  The parser wrote the arc from the arriving line's end to the leaving
//! line's start, tangent to each there, and the corner the two were written meeting at onto both
//! (`syntax::parser::chains`); what only the corner as drawn can say is said here.
//!
//! - **The walk chooses the branch, no seed does.**  The round lies inside its corner, on the
//!   side the walk turns toward there: an arc runs counter-clockwise from its start, so at a
//!   corner the walk turns right at it is drawn from its end — start and end swapped, the names
//!   kept as written (`Sketch::walked_back`, which `resolve::follow` reads) — and the round's
//!   centre is held on the turn's side of both lines and of its own chord, three stated
//!   orientations (`Gauges::orient`), so of the four circles tangent to both lines the one inside
//!   the corner is the only root.
//! - **Seeds come from the corner**: each point the round touches a line at starts `R / tan(θ/2)`
//!   from the corner along it, the centre `R / sin(θ/2)` along the bisector, `θ` the corner's
//!   angle, its sides running toward the corners at the lines' far ends — so the walk's seeds
//!   carry across every round of it, and a point held at a fraction of its turn starts there.
//!   Only a point nothing seeded is written, and never a held one.
//! - A corner its lines run straight through, or fold back on, has nothing to round (E040).

use super::elaborate::Elab;
use super::entities::Deferred;
use super::source_map::public_path;
use super::{Code, Diag};
use crate::ir::{Operation, Statement};
use crate::model::{rounded, EntKind, EntRef, Round};
use std::collections::BTreeSet;

/// One round as the drawing has it, resolved: its statement, arc, lines and corner.
struct Found<'a> {
    st: &'a Statement,
    arc: usize,
    from: usize,
    to: usize,
    corner: Option<usize>,
}

/// Every round of the drawing settled (see the module): turned the way its corner turns, seeded
/// from that corner, its branch stated — a turned copy of a `ring` only turned and seeded, its
/// branch being its representative's.
pub(super) fn rounds(
    cx: &mut Elab<'_>,
    body: &[&Statement],
    deferred: &[Deferred],
    held: &BTreeSet<u32>,
) {
    let mut found = Vec::new();
    for &st in body {
        let Operation::Decl(d) = &st.kind else { continue };
        let Some(k) = d.round.as_deref() else { continue };
        let Some(arc) = cx.res.of.get(&d.name.key().text).copied() else { continue };
        let line = |r: &crate::syntax::Ref| {
            cx.res.entity(&cx.sk, r).filter(|e| e.kind == EntKind::Line).map(|e| e.i())
        };
        let (Some(from), Some(to)) = (line(&k.from), line(&k.to)) else {
            let what = if line(&k.from).is_none() { &k.from } else { &k.to };
            cx.diags.push(Diag::within(Code::E040, st.span, st.id, format!(
                "a round rounds the corner where two lines meet, and `{}` is none",
                crate::syntax::ref_text(what))));
            continue;
        };
        let corner = match &k.corner {
            Some(c) => match cx.res.entity(&cx.sk, c).filter(|e| e.kind == EntKind::Point) {
                Some(p) => Some(p.i()),
                None => continue,
            },
            None => None,
        };
        found.push(Found { st, arc: arc.i(), from, to, corner });
    }
    if found.is_empty() {
        return;
    }
    let sk = &cx.sk;
    let in_space = found.iter().find(|f| {
        let l = |i: usize| [sk.lines[i].p1, sk.lines[i].p2];
        l(f.from).into_iter().chain(l(f.to)).chain(f.corner.map(|c| c as u32))
            .any(|p| sk.points[p as usize].z.is_some() || sk.points[p as usize].chart.is_some())
    });
    if let Some(f) = in_space {
        cx.diags.push(Diag::within(Code::E040, f.st.span, f.st.id, "a round rounds a corner \
            drawn in a plane, and this one's lines stand in space: draw them `in` one"
            .to_string()));
        return;
    }
    // what each is called, for the messages said here and after a solve
    let called = |e: EntRef| {
        cx.map.name_of(e).filter(|n| !n.contains("#a")).map(|n| public_path(n))
    };
    cx.sk.rounds = found.iter().map(|f| {
        let (from, to) = (EntRef::line(f.from), EntRef::line(f.to));
        let lines = [called(from), called(to)].map(|n| n.unwrap_or_else(|| "a line".into()));
        let arc = called(EntRef::arc(f.arc)).unwrap_or_else(|| {
            format!("the round of `{}` into `{}`", lines[0], lines[1])
        });
        let [a, b] = lines;
        Round { arc: f.arc as u32, from: f.from as u32, to: f.to as u32,
                corner: f.corner.map(|c| c as u32), turn: 1, names: [arc, a, b] }
    }).collect();
    let scattered: BTreeSet<usize> = deferred.iter().filter_map(|d| match d {
        Deferred::Scattered { point } => Some(*point),
        _ => None,
    }).collect();
    for (n, f) in found.iter().enumerate() {
        let sk = &cx.sk;
        let k = &sk.rounds[n];
        let r = stated_radius(sk, f.arc);
        let (lf, lt) = (&sk.lines[f.from], &sk.lines[f.to]);
        let (start, end, far_a, far_b) = (lf.p2, lt.p1, lf.p1, lt.p2);
        let at = sk.round_corner(k).unwrap_or_else(|| {
            let (s, e) = (sk.point_xy(start as usize), sk.point_xy(end as usize));
            ((s.0 + e.0) / 2.0, (s.1 + e.1) / 2.0)
        });
        let Some(round) = rounded(at, sk.round_far(far_a), sk.round_far(far_b), r) else {
            cx.diags.push(Diag::within(Code::E040, f.st.span, f.st.id, format!(
                "`{}` rounds no corner: `{}` and `{}` run straight on there, or fold back on \
                 each other, as drawn — a round turns from one line onto the other",
                k.names[0], k.names[1], k.names[2])));
            continue;
        };
        let centre = sk.arcs[f.arc].center as usize;
        let radius = sk.arcs[f.arc].radius;
        // a point nothing seeded starts where the corner puts it; a held one stays
        let sk = &mut cx.sk;
        let seeds = [(start as usize, round.start), (end as usize, round.end),
                     (centre, round.center)];
        for (p, q) in seeds {
            if !scattered.contains(&p) {
                continue;
            }
            for (param, v) in sk.point_params(p).into_iter().zip([q.0, q.1]) {
                let x = &mut sk.params[param as usize];
                if !x.fixed && !held.contains(&param) {
                    x.value = v;
                }
            }
        }
        if !sk.params[radius as usize].fixed && !held.contains(&radius) {
            sk.params[radius as usize].value = r;
        }
        // at a right turn the arc is drawn from its end, counter-clockwise from there
        sk.rounds[n].turn = round.turn;
        if round.turn < 0 {
            let a = &mut sk.arcs[f.arc];
            std::mem::swap(&mut a.start, &mut a.end);
            sk.walked_back.insert(f.arc as u32);
        }
        if f.st.turned {
            continue;
        }
        // the centre on the turn's side of both lines and of the arc's own chord
        let ccw = round.turn > 0;
        let names = &sk.rounds[n].names;
        for (tri, what) in [
            ([far_a as usize, start as usize, centre], names[1].as_str()),
            ([end as usize, far_b as usize, centre], names[2].as_str()),
            ([start as usize, end as usize, centre], names[0].as_str()),
        ] {
            let (key, v) = crate::decompose::branch_record(tri, ccw);
            cx.map.chosen.push((key.clone(), super::InstPath(f.st.path.clone())));
            let side = if ccw { "left" } else { "right" };
            let what = format!("the centre of `{}` on the {side} of `{what}`", names[0]);
            cx.gauges.orient(key, v, what, f.st);
        }
    }
    // a point held at a fraction of a round's turn, nothing else seeding it, starts there — read
    // over the round as just seeded and turned
    let sk = &mut cx.sk;
    let rounds: BTreeSet<EntRef> = sk.rounds.iter().map(|k| EntRef::arc(k.arc as usize)).collect();
    let along: Vec<(usize, usize, f64)> = sk.constraints.iter()
        .filter(|c| c.kind == crate::constraints::CKind::PointAlongArc)
        .filter(|c| rounds.contains(&c.args[1].ent()) && scattered.contains(&c.args[0].ent().i()))
        .map(|c| (c.args[0].ent().i(), c.args[1].ent().i(), c.args[2].value(sk)))
        .collect();
    for (p, arc, t) in along {
        let a = &sk.arcs[arc];
        let (c, s, e) = (sk.point_xy(a.center as usize), sk.point_xy(a.start as usize),
                         sk.point_xy(a.end as usize));
        let r = sk.params[a.radius as usize].value;
        let from = (s.1 - c.1).atan2(s.0 - c.0);
        let sweep = ((e.1 - c.1).atan2(e.0 - c.0) - from).rem_euclid(std::f64::consts::TAU);
        let (sin, cos) = (from + t * sweep).sin_cos();
        for (param, v) in sk.point_params(p).into_iter().zip([c.0 + r * cos, c.1 + r * sin]) {
            let x = &mut sk.params[param as usize];
            if !x.fixed && !held.contains(&param) {
                x.value = v;
            }
        }
    }
    cx.sk.settle_turns();
}

/// The radius round `arc` states: its `radius` row's number — a `ring`'s turned copy's, its
/// representative's — or where it has none the arc's own.
fn stated_radius(sk: &crate::model::Sketch, arc: usize) -> f64 {
    let of = sk.turn_of(EntRef::arc(arc)).map_or(EntRef::arc(arc), |t| t.rep);
    sk.constraints.iter()
        .find(|c| c.kind == crate::constraints::CKind::Radius && c.args[0].ent() == of)
        .map(|c| c.args[1].value(sk))
        .unwrap_or(sk.params[sk.arcs[of.i()].radius as usize].value)
}
