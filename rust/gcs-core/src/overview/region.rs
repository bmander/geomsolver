//! **The region under the cursor** (#162 F0): the smallest closed loop of drawn edges around a
//! place on a plane, and the loops inside it that are its holes — what a click inside a profile
//! asks before a face is written over it.
//!
//! The edges meet where they share a **point**, by identity, exactly as a face's walk
//! (`program::solids::faces::build_loop`) matches them: two distinct points held together by a
//! constraint do not join, since a face could not be written over them either.  The faces of
//! that graph are walked the usual way — at each corner the edge turning least to the right
//! after arriving — after the dangling edges, which bound nothing, are pruned; a circle or a
//! closed curve is a loop by itself.

use super::workspace::Views;
use super::drawable;
use crate::graph::UnionFind;
use crate::model::{edge_ends, EntKind, EntRef, Sketch};
use crate::program::SourceMap;
use crate::solid::{inside_ring as contains, ring_area as signed_area};
use std::collections::{BTreeMap, BTreeSet};

/// A region: its outer loop and its holes, each the edges in walk order, and their rings — the
/// outer first, then each hole's — in the plane's own coordinates, for a front end to wash.
#[derive(Clone, Debug, PartialEq)]
pub struct Region {
    pub outer: Vec<EntRef>,
    pub holes: Vec<Vec<EntRef>>,
    pub rings: Vec<Vec<(f64, f64)>>,
}

/// One loop of the graph: its edges in walk order, its ring and its signed area.
struct Cycle {
    edges: Vec<EntRef>,
    ring: Vec<(f64, f64)>,
    area: f64,
    /// Which connected piece of the drawing it belongs to: a point's root among the joined
    /// edges, or past every point for a loop standing alone.
    piece: usize,
}

/// A drawn edge one way along: from one point to the other, its polyline oriented so.
struct Half {
    edge: usize,
    from: u32,
    to: u32,
    pts: Vec<(f64, f64)>,
    /// The way it leaves `from`.
    angle: f64,
}

/// The region of the drawing on `plane` (`None` the page) around `at`, in that plane's
/// coordinates.  `Ok(None)` when no loop encloses it; `Err` with the cause when the loop that does
/// cannot be a face's — one running along an edge twice.  `unit` refines a round edge.
pub fn region_at(
    sk: &Sketch,
    plane: Option<usize>,
    at: (f64, f64),
    unit: f64,
) -> Result<Option<Region>, String> {
    let views = Views::new(sk);
    let place = views.place(plane);
    let mut edges: Vec<(EntRef, (u32, u32))> = Vec::new();
    let mut closed: Vec<Cycle> = Vec::new();
    for e in sk.drawn() {
        let drawable_edge = matches!(e.kind,
            EntKind::Line | EntKind::Arc | EntKind::Circle | EntKind::Spline | EntKind::Curve);
        if !drawable_edge || sk.class_of(e).has("closure") || sk.roles_of(e).construction {
            continue;
        }
        if views.entity_view(sk, e).ok().map(|v| views.place(v)) != Some(place) {
            continue;
        }
        let ends = edge_ends(sk, e);
        let whole = match e.kind {
            EntKind::Circle => true,
            EntKind::Curve => ends.is_none() && sk.curve_closed(e.i()),
            _ => ends.is_some_and(|(a, b)| a == b),
        };
        if whole {
            // a lone loop bounds what it encloses, whichever way it was drawn, and is its own
            // outline: counter-clockwise for the one, clockwise for the other
            let mut ring: Vec<(f64, f64)> = drawable(sk, e, unit).into_iter().flatten().collect();
            if signed_area(&ring) < 0.0 {
                ring.reverse();
            }
            let piece = sk.points.len() + closed.len();
            let back: Vec<(f64, f64)> = ring.iter().rev().copied().collect();
            closed.push(Cycle { edges: vec![e], area: signed_area(&ring), ring, piece });
            closed.push(Cycle { edges: vec![e], area: signed_area(&back), ring: back, piece });
        } else if let Some(ends) = ends {
            edges.push((e, ends));
        }
    }
    let mut cycles = walk(sk, &edges, unit);
    cycles.extend(closed);
    // the outer loop: the smallest bounded face holding `at`
    let Some(outer) = cycles.iter()
        .filter(|c| c.area > 0.0 && contains(&c.ring, at))
        .min_by(|a, b| a.area.total_cmp(&b.area))
    else {
        return Ok(None);
    };
    once_each(&outer.edges)?;
    // its holes: every other piece whose outline lies inside it, and inside no other such piece
    let outlines: Vec<&Cycle> = cycles.iter()
        .filter(|c| c.area < 0.0 && c.piece != outer.piece)
        .filter(|c| c.ring.first().is_some_and(|&p| contains(&outer.ring, p)))
        .collect();
    let mut holes = Vec::new();
    let mut rings = vec![outer.ring.clone()];
    for (k, c) in outlines.iter().enumerate() {
        let p = c.ring[0];
        let nested = outlines.iter().enumerate()
            .any(|(j, d)| j != k && contains(&d.ring, p) && !contains(&c.ring, d.ring[0]));
        if nested {
            continue;
        }
        once_each(&c.edges)?;
        // walked the other way round, so the hole reads the way the outer loop does
        holes.push(c.edges.iter().rev().copied().collect());
        rings.push(c.ring.iter().rev().copied().collect());
    }
    Ok(Some(Region { outer: outer.edges.clone(), holes, rings }))
}

/// The names a face statement writes a region's edges by, the outer loop then each hole: the
/// source's own (`ab`, `r0.l1`).  `Err` names an edge the source cannot write — a block copy's.
pub fn written(map: &SourceMap, r: &Region) -> Result<(Vec<String>, Vec<Vec<String>>), String> {
    let name = |e: &EntRef| {
        map.writable_name(*e).cloned().ok_or_else(|| match map.name_of(*e) {
            Some(n) => format!("`{n}` has no name a statement can write: name it in the source first"),
            None => format!("{} on the region's boundary has no name: name it first", e.kind.a()),
        })
    };
    let outer = r.outer.iter().map(name).collect::<Result<Vec<_>, _>>()?;
    let holes = r.holes.iter()
        .map(|h| h.iter().map(name).collect::<Result<Vec<_>, _>>())
        .collect::<Result<Vec<_>, _>>()?;
    Ok((outer, holes))
}

/// A loop that runs along one edge twice — a bridge between two loops, walked both ways — is no
/// face: the walk would have to cross itself.
fn once_each(edges: &[EntRef]) -> Result<(), String> {
    let mut seen = BTreeSet::new();
    if edges.iter().all(|e| seen.insert(*e)) {
        return Ok(());
    }
    Err("the loop around this place runs along an edge twice, so it bounds no face: a line joins \
         two loops"
        .into())
}

/// Every loop of the graph the edges make, pruned of what dangles: the bounded faces counter-
/// clockwise (positive area), each piece's outline clockwise (negative).
fn walk(sk: &Sketch, edges: &[(EntRef, (u32, u32))], unit: f64) -> Vec<Cycle> {
    let ends: Vec<(u32, u32)> = edges.iter().map(|&(_, ends)| ends).collect();
    // prune: an edge with an end nothing else meets bounds nothing, to a fixed point
    let mut alive = vec![true; edges.len()];
    loop {
        let mut degree: BTreeMap<u32, usize> = BTreeMap::new();
        for (&(a, b), _) in ends.iter().zip(&alive).filter(|(_, live)| **live) {
            *degree.entry(a).or_default() += 1;
            *degree.entry(b).or_default() += 1;
        }
        let mut pruned = false;
        for (k, &(a, b)) in ends.iter().enumerate() {
            if alive[k] && (degree[&a] < 2 || degree[&b] < 2) {
                alive[k] = false;
                pruned = true;
            }
        }
        if !pruned {
            break;
        }
    }
    // the pieces: points joined by a live edge are one
    let mut piece = UnionFind::new(sk.points.len());
    for (&(a, b), _) in ends.iter().zip(&alive).filter(|(_, live)| **live) {
        piece.union(a as usize, b as usize);
    }
    // two halves an edge, `2k` from its first end and `2k + 1` back
    let mut halves: Vec<Half> = Vec::new();
    for (k, &(e, _)) in edges.iter().enumerate() {
        let pts: Vec<(f64, f64)> = drawable(sk, e, unit).into_iter().flatten().collect();
        let (a, b) = ends[k];
        let back: Vec<(f64, f64)> = pts.iter().rev().copied().collect();
        halves.push(Half { edge: k, from: a, to: b, angle: leaving(&pts), pts });
        halves.push(Half { edge: k, from: b, to: a, angle: leaving(&back), pts: back });
    }
    // what leaves each corner, counter-clockwise by the way it leaves
    let mut out: BTreeMap<u32, Vec<usize>> = BTreeMap::new();
    for (h, half) in halves.iter().enumerate().filter(|(_, h)| alive[h.edge]) {
        out.entry(half.from).or_default().push(h);
    }
    for list in out.values_mut() {
        list.sort_by(|&x, &y| halves[x].angle.total_cmp(&halves[y].angle).then(x.cmp(&y)));
    }
    // after arriving along `h`, the edge leaving next clockwise from the way back
    let next = |h: usize| -> usize {
        let list = &out[&halves[h].to];
        let back = h ^ 1;
        let i = list.iter().position(|&x| x == back).unwrap();
        list[(i + list.len() - 1) % list.len()]
    };
    let mut seen = vec![false; halves.len()];
    let mut cycles = Vec::new();
    for start in 0..halves.len() {
        if seen[start] || !alive[halves[start].edge] {
            continue;
        }
        let (mut h, mut walked) = (start, Vec::new());
        while !seen[h] {
            seen[h] = true;
            walked.push(h);
            h = next(h);
        }
        if h != start {
            continue;   // a walk that joins a loop it did not start: not a face (cannot happen)
        }
        let mut ring: Vec<(f64, f64)> = Vec::new();
        for &w in &walked {
            let pts = &halves[w].pts;
            ring.extend(if ring.is_empty() { &pts[..] } else { &pts[1..] });
        }
        let area = signed_area(&ring);
        cycles.push(Cycle { edges: walked.iter().map(|&w| edges[halves[w].edge].0).collect(), ring,
            area, piece: piece.find(halves[start].from as usize) });
    }
    cycles
}

/// The direction a polyline leaves its first point, read off its first step of any length.
fn leaving(pts: &[(f64, f64)]) -> f64 {
    let p = pts[0];
    let q = pts.iter().skip(1).find(|q| q.0 != p.0 || q.1 != p.1).copied().unwrap_or(p);
    (q.1 - p.1).atan2(q.0 - p.0)
}
