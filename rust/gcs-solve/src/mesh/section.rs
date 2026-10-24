//! **A solid cut by a plane**: where the plane meets its surface, as closed loops, and the face
//! the cut leaves, as triangles.
//!
//! A section is read off the mesh a viewer is given (`Mesh`), not off the boundary pieces, so the
//! face it leaves meets the surface that is drawn beside it with no crack: the loops are made of
//! the very points where the drawn triangles are cut.  The plane takes away the side its normal
//! points to — `n·(p − o) > 0` — and keeps the rest, the material behind it as the plane's viewer
//! sees it, as a sheet's `section` keeps it.
//!
//! **Each crossing is a point of a mesh edge, computed once.**  The mesh is welded: two
//! triangles sharing a side share its corners' bits.  So a crossing is filed by the side it is on,
//! read from its lower corner to its higher whichever triangle asks, and the two triangles across
//! a side agree on it exactly; a closed surface's crossings then chain into closed loops with no
//! tolerance.  A corner within the mesh's resolution of the plane is *on* it, and on it counts as
//! taken away — a symbolic choice, so a corner is never half of each, and a face lying in the
//! plane is cut along its rim and nowhere else.
//!
//! **Which loop is a hole is how deep it is nested**, not which way the triangles wound it: a loop
//! inside an even number of others bounds material and is turned counter-clockwise about the
//! normal, one inside an odd number is a hole in the loop just outside it and is turned
//! clockwise.  The face is each outer loop and its holes, the holes bridged in (Eberly) and the
//! whole clipped of ears with the exact orientation predicate, its triangles counter-clockwise
//! about the normal — so the cut face faces the side taken away, as an outward face does.
//! Deterministic: ordered maps only, every choice by an index.
#[allow(unused_imports)]
use crate::fmath::Det;
use super::Mesh;
use crate::delaunay::predicates::orient2;
use crate::space::{across, cross, dot, normalised, sub};
use std::collections::BTreeMap;

type P = [f64; 2];

/// How near the plane a corner is, relative to the mesh's extent, to be on it: the weld's own
/// resolution (`WELD`), far below anything a drawing states.
const ON: f64 = 1e-9;

/// Where a plane meets a solid's surface, in the coordinates the mesh is in.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Section {
    /// Each closed loop of the cut: an outer loop counter-clockwise about the plane's normal, a
    /// hole clockwise.  Not repeated at its end.
    pub loops: Vec<Loop>,
    /// The face the cut leaves, nine doubles a triangle, counter-clockwise about the normal.
    pub triangles: Vec<f64>,
    /// Chains of the cut that do not close — a surface that is not closed — drawn and not
    /// filled.
    pub open: Vec<Vec<[f64; 3]>>,
    /// The plane: a point of it and its unit normal, the side taken away, and how near it a point
    /// is to be on it (and so taken away too).
    pub origin: [f64; 3],
    pub normal: [f64; 3],
    pub on: f64,
}

/// One closed loop of a section.
#[derive(Clone, Debug, PartialEq)]
pub struct Loop {
    pub pts: Vec<[f64; 3]>,
    /// Whether it bounds a hole in the loop around it rather than material.
    pub hole: bool,
}

impl Section {
    /// **How far `p` stands in front of the plane**, nought within `on` of it: what the cut takes
    /// away is where this is not negative.  The one rule the loops were cut by, so what a
    /// renderer leaves out of the surface ends where the face the cut leaves begins.
    pub fn height(&self, p: [f64; 3]) -> f64 {
        let h = dot(self.normal, sub(p, self.origin));
        if h.abs() <= self.on { 0.0 } else { h }
    }

    /// Whether the cut keeps `p`: behind the plane, and not on it.
    pub fn keeps(&self, p: [f64; 3]) -> bool {
        self.height(p) < 0.0
    }

    /// What the cut keeps of the segment `a → b`, or `None` where it keeps none of it: a segment
    /// on the plane is taken away with what stands in front of it.
    pub fn clip(&self, a: [f64; 3], b: [f64; 3]) -> Option<([f64; 3], [f64; 3])> {
        let (ha, hb) = (self.height(a), self.height(b));
        if ha >= 0.0 && hb >= 0.0 {
            return None;
        }
        if ha <= 0.0 && hb <= 0.0 {
            return Some((a, b));
        }
        let t = ha / (ha - hb);
        let at: [f64; 3] = std::array::from_fn(|k| a[k] + t * (b[k] - a[k]));
        Some(if ha > 0.0 { (at, b) } else { (a, at) })
    }

    /// Whether the plane meets the surface at all.
    pub fn is_empty(&self) -> bool {
        self.loops.is_empty() && self.open.is_empty()
    }

    /// The cut face's area: its triangles'.
    pub fn area(&self) -> f64 {
        self.triangles.as_chunks::<9>().0.iter().map(|t| {
            let (a, b, c) = ([t[0], t[1], t[2]], [t[3], t[4], t[5]], [t[6], t[7], t[8]]);
            crate::space::norm(cross(sub(b, a), sub(c, a))) / 2.0
        }).sum()
    }

    /// Every stretch of the cut, closed loops and open chains, as segments `[a, b]`: what is
    /// inked round the face.
    pub fn segments(&self) -> Vec<[[f64; 3]; 2]> {
        let closed = self.loops.iter().flat_map(|l| {
            (0..l.pts.len()).map(move |i| [l.pts[i], l.pts[(i + 1) % l.pts.len()]])
        });
        let open = self.open.iter().flat_map(|c| c.windows(2).map(|w| [w[0], w[1]]));
        closed.chain(open).collect()
    }
}

/// **The plane `section` cuts `m` by, and nothing cut**: its rule of what is kept (`height`,
/// `keeps`, `clip`) with no loops and no face — what a pick asks of a cut, which must not slice
/// the mesh to be told which side of the plane a point is on.  Where `n` has no direction it
/// takes everything away.
pub fn plane(m: &Mesh, o: [f64; 3], n: [f64; 3]) -> Section {
    let Some(n) = normalised(n).filter(|n| n.iter().all(|x| x.is_finite())) else {
        return Section { origin: o, ..Section::default() };
    };
    let (mut lo, mut hi) = ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]);
    for p in m.positions.as_chunks::<3>().0 {
        for k in 0..3 {
            lo[k] = lo[k].min(p[k]);
            hi[k] = hi[k].max(p[k]);
        }
    }
    let extent = (0..3).fold(0.0f64, |e, k| e.max(hi[k] - lo[k]));
    Section { origin: o, normal: n, on: extent * ON, ..Section::default() }
}

/// **`m` cut by the plane through `o` whose normal is `n`** (not necessarily unit): the side `n`
/// points to taken away.  Empty where the plane misses the surface; where `n` has no direction,
/// empty and taking everything away.
pub fn section(m: &Mesh, o: [f64; 3], n: [f64; 3]) -> Section {
    let mut out = plane(m, o, n);
    let n = out.normal;
    if n == [0.0; 3] {
        return out;
    }
    let on = out.on;
    // how far a corner stands out of the plane, snapped onto it within the mesh's resolution
    let height = |p: [f64; 3]| out.height(p);
    // taken away: in front of the plane, or on it
    let away = |h: f64| h >= 0.0;

    // -- every triangle cut: a segment between the crossings of its two cut sides, each crossing
    // filed by its side (its corners' bits, lower first) and then by its own bits, so crossings
    // at one corner on the plane are one point
    let mut by_side: BTreeMap<[u64; 6], usize> = BTreeMap::new();
    let mut by_point: BTreeMap<[u64; 3], usize> = BTreeMap::new();
    let mut points: Vec<[f64; 3]> = Vec::new();
    let mut segs: Vec<[usize; 2]> = Vec::new();
    let bits = |p: [f64; 3]| p.map(f64::to_bits);
    for t in m.positions.as_chunks::<9>().0 {
        let c = [[t[0], t[1], t[2]], [t[3], t[4], t[5]], [t[6], t[7], t[8]]];
        let h = c.map(height);
        let side = h.map(away);
        if side[0] == side[1] && side[1] == side[2] {
            continue;
        }
        let mut at = [usize::MAX; 2];
        let mut k = 0;
        for i in 0..3 {
            let j = (i + 1) % 3;
            if side[i] == side[j] {
                continue;
            }
            let (mut a, mut b, mut ha, mut hb) = (c[i], c[j], h[i], h[j]);
            if bits(b) < bits(a) {
                std::mem::swap(&mut a, &mut b);
                std::mem::swap(&mut ha, &mut hb);
            }
            let key = {
                let (x, y) = (bits(a), bits(b));
                [x[0], x[1], x[2], y[0], y[1], y[2]]
            };
            let index = match by_side.get(&key) {
                Some(&i) => i,
                None => {
                    // a corner on the plane is the crossing itself, exactly
                    let p = if ha == 0.0 { a } else if hb == 0.0 { b } else {
                        let s = ha / (ha - hb);
                        std::array::from_fn(|k| a[k] + s * (b[k] - a[k]))
                    };
                    let i = *by_point.entry(bits(p)).or_insert_with(|| {
                        points.push(p);
                        points.len() - 1
                    });
                    by_side.insert(key, i);
                    i
                }
            };
            at[k] = index;
            k += 1;
        }
        if k == 2 && at[0] != at[1] {
            segs.push(at);
        }
    }
    if segs.is_empty() {
        return out;
    }

    // -- the segments chained: from each end of an open chain first, then round each cycle
    let mut about: Vec<Vec<usize>> = vec![Vec::new(); points.len()];
    for (s, &[a, b]) in segs.iter().enumerate() {
        about[a].push(s);
        about[b].push(s);
    }
    let mut used = vec![false; segs.len()];
    let walk = |from: usize, used: &mut Vec<bool>| -> (Vec<usize>, bool) {
        let mut chain = vec![from];
        let mut here = from;
        loop {
            let Some(&s) = about[here].iter().find(|&&s| !used[s]) else { return (chain, false) };
            used[s] = true;
            here = if segs[s][0] == here { segs[s][1] } else { segs[s][0] };
            if here == from {
                return (chain, true);
            }
            chain.push(here);
        }
    };
    let (u, v) = across(n);
    let flat = |p: [f64; 3]| -> P { let d = sub(p, o); [dot(d, u), dot(d, v)] };
    let mut closed: Vec<Vec<usize>> = Vec::new();
    let odd = (0..points.len()).filter(|&p| about[p].len() % 2 == 1);
    let starts: Vec<usize> = odd.chain(segs.iter().map(|s| s[0])).collect();
    for p in starts {
        while about[p].iter().any(|&s| !used[s]) {
            let (chain, round) = walk(p, &mut used);
            if round {
                closed.push(chain);
            } else {
                // an open chain is drawn to the point it stopped at
                out.open.push(chain.iter().map(|&i| points[i]).collect());
            }
        }
    }

    // -- each loop read in the plane, nested, and turned by its depth
    let mut loops: Vec<(Vec<[f64; 3]>, Vec<P>, f64)> = closed.into_iter().filter_map(|c| {
        let flat_pts: Vec<P> = c.iter().map(|&i| flat(points[i])).collect();
        let a = signed_area(&flat_pts);
        let pts = c.iter().map(|&i| points[i]).collect();
        (c.len() >= 3 && a.abs() > on * on).then_some((pts, flat_pts, a))
    }).collect();
    let boxes: Vec<[f64; 4]> = loops.iter().map(|(_, q, _)| box_of(q)).collect();
    let within = |i: usize, j: usize, loops: &[(Vec<[f64; 3]>, Vec<P>, f64)]| -> bool {
        if i == j || !box_holds(boxes[j], boxes[i]) || loops[j].2.abs() <= loops[i].2.abs() {
            return false;
        }
        // the first of i's points not on j's boundary says which side of j the whole of i is on
        loops[i].1.iter().find_map(|&p| inside(p, &loops[j].1)).unwrap_or(false)
    };
    let count = loops.len();
    let depth: Vec<usize> =
        (0..count).map(|i| (0..count).filter(|&j| within(i, j, &loops)).count()).collect();
    for (i, (pts, q, a)) in loops.iter_mut().enumerate() {
        let hole = depth[i] % 2 == 1;
        if (*a > 0.0) == hole {
            pts.reverse();
            q.reverse();
            *a = -*a;
        }
    }
    // each hole's outer loop: the smallest of those it is nested in one level down
    let mut holes: Vec<Vec<usize>> = vec![Vec::new(); count];
    for i in (0..count).filter(|&i| depth[i] % 2 == 1) {
        let parent = (0..count).filter(|&j| depth[j] + 1 == depth[i] && within(i, j, &loops))
            .min_by(|&x, &y| loops[x].2.abs().total_cmp(&loops[y].2.abs()));
        if let Some(j) = parent {
            holes[j].push(i);
        }
    }
    for i in (0..count).filter(|&i| depth[i] % 2 == 0) {
        let rings: Vec<(&[[f64; 3]], &[P])> = std::iter::once(i).chain(holes[i].iter().copied())
            .map(|k| (&loops[k].0[..], &loops[k].1[..])).collect();
        for t in triangulate(&rings) {
            for p in t {
                out.triangles.extend(p);
            }
        }
    }
    out.loops = loops.into_iter().enumerate()
        .map(|(i, (pts, _, _))| Loop { pts, hole: depth[i] % 2 == 1 }).collect();
    out
}

/// Twice nothing: the shoelace area of a closed polygon, positive counter-clockwise.
fn signed_area(q: &[P]) -> f64 {
    let n = q.len();
    (0..n).map(|i| {
        let (a, b) = (q[i], q[(i + 1) % n]);
        a[0] * b[1] - a[1] * b[0]
    }).sum::<f64>() / 2.0
}

fn box_of(q: &[P]) -> [f64; 4] {
    q.iter().fold([f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY],
        |b, p| [b[0].min(p[0]), b[1].min(p[1]), b[2].max(p[0]), b[3].max(p[1])])
}

fn box_holds(outer: [f64; 4], inner: [f64; 4]) -> bool {
    outer[0] <= inner[0] && outer[1] <= inner[1] && outer[2] >= inner[2] && outer[3] >= inner[3]
}

/// Whether `p` is inside the closed polygon `q`, by the crossings of a ray along +x; `None` where
/// it is on its boundary, which says nothing.
fn inside(p: P, q: &[P]) -> Option<bool> {
    let n = q.len();
    let mut odd = false;
    for i in 0..n {
        let (a, b) = (q[i], q[(i + 1) % n]);
        let o = orient2(a, b, p);
        if o == 0 && p[0] >= a[0].min(b[0]) && p[0] <= a[0].max(b[0])
            && p[1] >= a[1].min(b[1]) && p[1] <= a[1].max(b[1]) {
            return None;
        }
        // a side crosses the ray where its ends straddle the ray's line and it passes right of p
        if (a[1] > p[1]) != (b[1] > p[1]) {
            let rising = b[1] > a[1];
            if (o > 0) == rising {
                odd = !odd;
            }
        }
    }
    Some(odd)
}

/// One corner of the polygon being clipped: its point in the plane and in space, and its
/// neighbours along the polygon.
#[derive(Clone, Copy)]
struct Node {
    q: P,
    p: [f64; 3],
    prev: usize,
    next: usize,
    alive: bool,
}

/// **An outer loop and its holes as triangles** — the outer counter-clockwise, the holes
/// clockwise, each given in the plane (`.1`) and in space (`.0`): each hole bridged to what is
/// already joined from its rightmost corner (Eberly, "Triangulation by Ear Clipping" §3), then
/// ears clipped, each counter-clockwise.  Where no ear is left (a polygon pinched to a point), the
/// first convex corner is clipped anyway, so the walk always ends.
fn triangulate(rings: &[(&[[f64; 3]], &[P])]) -> Vec<[[f64; 3]; 3]> {
    let mut nodes: Vec<Node> = Vec::new();
    let ring = |(pts, q): (&[[f64; 3]], &[P]), nodes: &mut Vec<Node>| -> usize {
        let first = nodes.len();
        let n = q.len();
        for i in 0..n {
            nodes.push(Node {
                q: q[i],
                p: pts[i],
                prev: first + (i + n - 1) % n,
                next: first + (i + 1) % n,
                alive: true,
            });
        }
        first
    };
    let outer = ring(rings[0], &mut nodes);
    // the holes from the rightmost in: each bridge then sees only what is already joined
    let mut holes: Vec<(usize, usize)> = rings[1..].iter().map(|&r| {
        let first = ring(r, &mut nodes);
        let right = (first..first + r.1.len()).max_by(|&a, &b| {
            let (x, y) = (nodes[a].q, nodes[b].q);
            x[0].total_cmp(&y[0]).then(y[1].total_cmp(&x[1]))
        }).expect("a ring has corners");
        (first, right)
    }).collect();
    holes.sort_by(|a, b| nodes[b.1].q[0].total_cmp(&nodes[a.1].q[0]).then(a.0.cmp(&b.0)));
    for (_, m) in holes {
        if let Some(p) = bridge(&nodes, outer, m) {
            // p → m → … round the hole … → m′ → p′ → what followed p
            let (pn, mp) = (nodes[p].next, nodes[m].prev);
            let m2 = nodes.len();
            nodes.push(Node { prev: mp, next: m2 + 1, ..nodes[m] });
            let p2 = nodes.len();
            nodes.push(Node { prev: m2, next: pn, ..nodes[p] });
            nodes[mp].next = m2;
            nodes[pn].prev = p2;
            nodes[p].next = m;
            nodes[m].prev = p;
        }
    }

    let mut out = Vec::new();
    let mut left = {
        let mut k = 1;
        let mut i = nodes[outer].next;
        while i != outer {
            k += 1;
            i = nodes[i].next;
        }
        k
    };
    let mut at = outer;
    // how many corners have been looked at since the last was clipped: once all have, none is
    // an ear
    let mut since = 0;
    let unlink = |nodes: &mut Vec<Node>, b: usize| {
        let (a, c) = (nodes[b].prev, nodes[b].next);
        nodes[a].next = c;
        nodes[c].prev = a;
        nodes[b].alive = false;
    };
    while left > 3 {
        let (a, b, c) = (nodes[at].prev, at, nodes[at].next);
        let turn = turn(nodes[a].q, nodes[b].q, nodes[c].q);
        let stuck = since > left;
        if turn == 0 && !stuck {
            // a corner on the line between its neighbours (a bridge's doubled corner, a spike)
            // bounds nothing: dropped, no triangle
            unlink(&mut nodes, b);
            left -= 1;
            at = c;
            since = 0;
            continue;
        }
        if turn > 0 && (stuck || ear(&nodes, a, b, c)) {
            out.push([nodes[a].p, nodes[b].p, nodes[c].p]);
            unlink(&mut nodes, b);
            left -= 1;
            at = c;
            since = 0;
            continue;
        }
        if stuck && since > 2 * left {
            // nothing convex is left: the polygon is no more than a sliver
            break;
        }
        at = c;
        since += 1;
    }
    if left == 3 {
        let (a, b, c) = (nodes[at].prev, at, nodes[at].next);
        if turn(nodes[a].q, nodes[b].q, nodes[c].q) > 0 {
            out.push([nodes[a].p, nodes[b].p, nodes[c].p]);
        }
    }
    out
}

/// Which way the polygon turns at `b`: the exact sign, but straight where it turns by less than
/// the rounding a crossing's point carries — three points along one straight side of a loop
/// otherwise make a sliver facing either way.
fn turn(a: P, b: P, c: P) -> i8 {
    let (u, v) = ([b[0] - a[0], b[1] - a[1]], [c[0] - b[0], c[1] - b[1]]);
    let area = u[0] * v[1] - u[1] * v[0];
    if area.abs() <= 1e-12 * u[0].dhypot(u[1]) * v[0].dhypot(v[1]) {
        return 0;
    }
    orient2(a, b, c)
}

/// Whether the convex corner `b` between `a` and `c` is an ear: no other corner of the polygon in
/// its triangle, its edges included, but those standing where one of its own corners does (a
/// bridge's doubles).
fn ear(nodes: &[Node], a: usize, b: usize, c: usize) -> bool {
    let (qa, qb, qc) = (nodes[a].q, nodes[b].q, nodes[c].q);
    let lo = [qa[0].min(qb[0]).min(qc[0]), qa[1].min(qb[1]).min(qc[1])];
    let hi = [qa[0].max(qb[0]).max(qc[0]), qa[1].max(qb[1]).max(qc[1])];
    let mut i = nodes[c].next;
    while i != a {
        let q = nodes[i].q;
        let own = q == qa || q == qb || q == qc;
        if !own && q[0] >= lo[0] && q[0] <= hi[0] && q[1] >= lo[1] && q[1] <= hi[1]
            && orient2(qa, qb, q) >= 0 && orient2(qb, qc, q) >= 0 && orient2(qc, qa, q) >= 0 {
            return false;
        }
        i = nodes[i].next;
    }
    true
}

/// The corner of the polygon joined so far (from `outer`) that hole corner `m` — the hole's
/// rightmost — can be bridged to: the ray from `m` along +x meets the nearest side at `i`; the
/// side's end farther right is seen from `m` unless a reflex corner stands in the triangle
/// `m, i, end`, and then the one of those nearest the ray's direction is.
fn bridge(nodes: &[Node], outer: usize, m: usize) -> Option<usize> {
    let qm = nodes[m].q;
    let mut best: Option<(f64, usize)> = None;
    let mut i = outer;
    loop {
        let j = nodes[i].next;
        let (a, b) = (nodes[i].q, nodes[j].q);
        // a side running down past m's height (the outer loop counter-clockwise, its right side
        // rises; seen from inside, a side of the hole's right rises too)
        if (a[1] <= qm[1] && b[1] >= qm[1] || a[1] >= qm[1] && b[1] <= qm[1]) && a[1] != b[1] {
            let x = a[0] + (qm[1] - a[1]) * (b[0] - a[0]) / (b[1] - a[1]);
            if x >= qm[0] && best.is_none_or(|(bx, _)| x < bx) {
                let end = if a[0] >= b[0] { i } else { j };
                // the ray through a corner meets that corner itself
                let end = if x == a[0] && qm[1] == a[1] { i }
                    else if x == b[0] && qm[1] == b[1] { j }
                    else { end };
                best = Some((x, end));
            }
        }
        i = j;
        if i == outer {
            break;
        }
    }
    let (x, p) = best?;
    let qi = [x, qm[1]];
    let qp = nodes[p].q;
    if qp == qi {
        return Some(p);
    }
    // a reflex corner inside m, i, p hides p: the one making the least angle with the ray
    let (t0, t1, t2) = if orient2(qm, qi, qp) > 0 { (qm, qi, qp) } else { (qm, qp, qi) };
    let mut chosen = p;
    let mut least = (f64::INFINITY, f64::INFINITY);
    let mut k = outer;
    loop {
        let q = nodes[k].q;
        if k != p && q[0] >= qm[0] {
            let reflex = orient2(nodes[nodes[k].prev].q, q, nodes[nodes[k].next].q) <= 0;
            let held = orient2(t0, t1, q) >= 0 && orient2(t1, t2, q) >= 0 && orient2(t2, t0, q) >= 0;
            if reflex && held {
                let (dx, dy) = (q[0] - qm[0], q[1] - qm[1]);
                let angle = (dy.abs() / dx.dhypot(dy).max(f64::MIN_POSITIVE), dx.dhypot(dy));
                if angle < least {
                    least = angle;
                    chosen = k;
                }
            }
        }
        k = nodes[k].next;
        if k == outer {
            break;
        }
    }
    Some(chosen)
}
