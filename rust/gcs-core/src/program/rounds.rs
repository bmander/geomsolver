//! **Corner rounds** (#576): the arc a chain's `A -> round(r: R) B` joint mints, settled once the
//! drawing is seeded.  The parser wrote the arc from the arriving side's end to the leaving
//! side's start, tangent to each there, and the corner the two were written meeting at onto both
//! (`syntax::parser::chains`); what only the corner as drawn can say is said here.  A side is a
//! line or an arc (#962).
//!
//! - **The walk chooses the branch, no seed does.**  The round lies inside its corner, on the
//!   side the walk turns toward there: an arc runs counter-clockwise from its start, so at a
//!   corner the walk turns right at it is drawn from its end — start and end swapped, the names
//!   kept as written (`Sketch::reversed`, #573, which names, `at:` and `t` read) — and the round's
//!   centre is held there by three stated orientations (`Gauges::orient`): on the turn's side of
//!   its own chord, and for each side on the turn's side of a line, or, of an arc, the arc's
//!   centre on the chord's side the round's is where the round is inside the arc's circle (the
//!   walk turning toward its centre) and on the other where it is outside.  So of the circles
//!   tangent to both sides the one inside the corner is the only root.
//! - **Seeds come from the corner** (`model::rounded`): between two lines each point the round
//!   touches a line at starts `R / tan(θ/2)` from the corner along it, the centre `R / sin(θ/2)`
//!   along the bisector, `θ` the corner's angle, its sides running toward the corners at the
//!   lines' far ends; beside an arc, where the sides' offsets by `R` toward the turn cross —
//!   so the walk's seeds carry across every round of it, and a point held at a fraction of its
//!   turn starts there.  Only a point nothing seeded is written, and never a held one.
//! - A corner its sides run straight through, or fold back on, has nothing to round (E040).

use super::elaborate::Elab;
use super::entities::Deferred;
use super::source_map::public_path;
use super::{Code, Diag};
use crate::ir::{Operation, Statement};
use crate::model::{rounded, EntKind, EntRef, Round, RoundSide, Sketch};
use std::collections::BTreeSet;

/// Why a round is refused where `what`, one of its two sides, is `is` — no line or arc (#948
/// says it where the declaration names a curve declared apart from the chain, `names`; here,
/// where a side names nothing drawn).
pub(super) fn not_sides(what: &str, is: &str) -> String {
    format!("a round rounds the corner where two lines or arcs meet, and `{what}` is {is}")
}

/// Whether `st` declares a corner round's arc (#576).
pub(super) fn is_round(st: &Statement) -> bool {
    matches!(&st.kind, Operation::Decl(d) if d.round.is_some())
}

/// One round as the drawing has it, resolved: its statement, arc, sides and corner.
struct Found<'a> {
    st: &'a Statement,
    arc: usize,
    from: EntRef,
    to: EntRef,
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
        let side = |r: &crate::syntax::Ref| {
            cx.res.entity(&cx.sk, r).filter(|e| matches!(e.kind, EntKind::Line | EntKind::Arc))
                .filter(|e| e.i() < cx.sk.count(e.kind))
        };
        let (Some(from), Some(to)) = (side(&k.from), side(&k.to)) else {
            let what = if side(&k.from).is_none() { &k.from } else { &k.to };
            let what = crate::syntax::ref_text(what);
            cx.diags.push(Diag::within(Code::E040, st.span, st.id, not_sides(&what, "none")));
            continue;
        };
        // a corner that is no point is said by the `coincident`s holding it to the sides
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
        let ends = |e: EntRef| {
            let (s, t) = sk.side_ends(e);
            let centre = (e.kind == EntKind::Arc).then(|| sk.arcs[e.i()].center);
            [s, t].into_iter().chain(centre)
        };
        ends(f.from).chain(ends(f.to)).chain(f.corner.map(|c| c as u32))
            .any(|p| sk.points[p as usize].z.is_some() || sk.points[p as usize].chart.is_some())
    });
    if let Some(f) = in_space {
        cx.diags.push(Diag::within(Code::E040, f.st.span, f.st.id, "a round rounds a corner \
            drawn in a plane, and this one's sides stand in space: draw them `in` one"
            .to_string()));
        return;
    }
    // what each is called, for the messages said here and after a solve
    let called = |e: EntRef| cx.map.name_of(e).map(|n| public_path(n));
    cx.sk.rounds = found.iter().map(|f| {
        let what = |e: EntRef| match e.kind {
            EntKind::Arc => "an arc",
            _ => "a line",
        };
        let sides = [called(f.from).unwrap_or_else(|| what(f.from).into()),
                     called(f.to).unwrap_or_else(|| what(f.to).into())];
        let arc = called(EntRef::arc(f.arc)).unwrap_or_else(|| {
            format!("the round of `{}` into `{}`", sides[0], sides[1])
        });
        let [a, b] = sides;
        Round { arc: f.arc as u32, from: f.from, to: f.to,
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
        let ((far_a, start), (end, far_b)) = (sk.side_ends(f.from), sk.side_ends(f.to));
        let at = sk.round_corner(k).unwrap_or_else(|| {
            let (s, e) = (sk.point_xy(start as usize), sk.point_xy(end as usize));
            ((s.0 + e.0) / 2.0, (s.1 + e.1) / 2.0)
        });
        // an arc side read through the corner, which its circle passes through: its radius the
        // centre's distance from a corner written, else the one it states
        let side = |e: EntRef, first: bool| match e.kind {
            EntKind::Arc => {
                let a = &sk.arcs[e.i()];
                let c = sk.point_xy(a.center as usize);
                let radius = match f.corner {
                    Some(_) => (at.0 - c.0).hypot(at.1 - c.1),
                    None => stated_radius(sk, e.i()),
                };
                (RoundSide::Arc { center: c, radius, ccw: !sk.arc_reversed(e.i()) }, Some(radius))
            }
            _ => (sk.round_side(e, first), None),
        };
        let ((a, ra), (b, rb)) = (side(f.from, true), side(f.to, false));
        let Some(round) = rounded(at, a, b, r) else {
            cx.diags.push(Diag::within(Code::E040, f.st.span, f.st.id, format!(
                "`{}` rounds no corner: `{}` and `{}` run straight on there, or fold back on \
                 each other, as drawn — a round turns from one side onto the other",
                k.names[0], k.names[1], k.names[2])));
            continue;
        };
        let centre = sk.arcs[f.arc].center as usize;
        let radius = sk.arcs[f.arc].radius;
        // a point nothing seeded starts where the corner puts it; a held one stays
        let sk = &mut cx.sk;
        let seeds = [(start as usize, round.start), (end as usize, round.end),
                     (centre, round.center)];
        for (p, q) in seeds.into_iter().filter(|(p, _)| scattered.contains(p)) {
            seed(sk, &sk.point_params(p), &[q.0, q.1], held);
        }
        seed(sk, &[radius], &[round.most.map_or(r, |m| r.min(m))], held);
        // an arc side's own radius, as its circle passes through the corner
        for (e, rho) in [(f.from, ra), (f.to, rb)] {
            if let Some(rho) = rho {
                seed(sk, &[sk.arcs[e.i()].radius], &[rho], held);
            }
        }
        // at a right turn the arc is drawn from its end, counter-clockwise from there
        sk.rounds[n].turn = round.turn;
        if round.turn < 0 {
            let a = &mut sk.arcs[f.arc];
            std::mem::swap(&mut a.start, &mut a.end);
            sk.reversed.insert(f.arc as u32);
        }
        if f.st.turned {
            continue;
        }
        // the centre on the turn's side of its own chord, and of each side: of a line, the line
        // run from its far end; of an arc, its centre on the chord's same side where the round is
        // inside its circle and the other where it is outside
        let ccw = round.turn > 0;
        let side = if ccw { "left" } else { "right" };
        let names = sk.rounds[n].names.clone();
        let mut gauges = vec![([start as usize, end as usize, centre], ccw,
                               format!("the centre of `{}` on the {side} of its chord", names[0]))];
        for (x, e, line, name) in [(0, f.from, [far_a as usize, start as usize], &names[1]),
                                   (1, f.to, [end as usize, far_b as usize], &names[2])] {
            gauges.push(match e.kind {
                EntKind::Arc => {
                    let o = sk.arcs[e.i()].center as usize;
                    let inside = round.inside[x];
                    let how = if inside { "inside" } else { "outside" };
                    ([start as usize, end as usize, o], ccw == inside,
                     format!("`{}` {how} `{name}`", names[0]))
                }
                _ => ([line[0], line[1], centre], ccw,
                      format!("the centre of `{}` on the {side} of `{name}`", names[0])),
            });
        }
        for (tri, ccw, what) in gauges {
            let (key, v) = crate::decompose::branch_record(tri, ccw);
            cx.map.chosen.push((key.clone(), super::InstPath(f.st.path.clone())));
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
        // a round drawn from its end runs its `t` from the end written as its start
        let t = if sk.arc_reversed(arc) { 1.0 - t } else { t };
        let (a0, a1) = sk.arc_angles(arc);
        let c = sk.point_xy(sk.arcs[arc].center as usize);
        let r = sk.params[sk.arcs[arc].radius as usize].value;
        let (sin, cos) = (a0 + t * (a1 - a0)).sin_cos();
        seed(sk, &sk.point_params(p), &[c.0 + r * cos, c.1 + r * sin], held);
    }
    sk.settle_turns();
}

/// Each of `params` written its value in `to`, but a held one: fixed, or carried across in a pass
/// before.
fn seed(sk: &mut Sketch, params: &[u32], to: &[f64], held: &BTreeSet<u32>) {
    for (&p, &v) in params.iter().zip(to) {
        let x = &mut sk.params[p as usize];
        if !x.fixed && !held.contains(&p) {
            x.value = v;
        }
    }
}

/// The radius round `arc` states: its `radius` row's number — a `ring`'s turned copy's, its
/// representative's — or where it has none the arc's own.
fn stated_radius(sk: &Sketch, arc: usize) -> f64 {
    let of = sk.turn_of(EntRef::arc(arc)).map_or(EntRef::arc(arc), |t| t.rep);
    sk.constraints.iter()
        .find(|c| c.kind == crate::constraints::CKind::Radius && c.args[0].ent() == of)
        .map(|c| c.args[1].value(sk))
        .unwrap_or(sk.params[sk.arcs[of.i()].radius as usize].value)
}
