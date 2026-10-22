//! **A region meshed for analysis** (#510 S2): quadratic triangles over a region of the plane
//! bounded by curves, by Delaunay refinement.
//!
//! The region is its loops of sides, each side a curve read at a parameter on `[0, 1]` (a line, an
//! arc, a spline of the drawing), the sides of a loop running end to end.  The boundary is cut
//! into segments whose ends lie *on* the curves — a curved side first into pieces turning no more
//! than `Criteria::turn` — and the triangulation is Delaunay and conforms to them: every segment
//! is a union of its edges, held so by splitting any segment a vertex encroaches (lies inside its
//! diametral circle) at its parameter's middle, on the curve.  Then Ruppert's refinement: a
//! triangle of the region too long for its shortest side (`Criteria::ratio`, its circumradius
//! over it) or too big (`Criteria::size`) gets its circumcentre, unless the circumcentre
//! encroaches a segment, which is split instead.  Under a bound of √2 every triangle's least
//! angle is over 20.7°, and a small feature — a hole, a tight curve — grades its neighbours down
//! to its size and back up away from it, so the mesh is fine where the region is.
//!
//! Each triangle gets its sides' middles: on a segment the curve's point at the middle of its
//! parameters, so the element is curved as the side is; elsewhere the straight middle.  Every
//! node on the boundary knows its side and its parameter (`Mesh::on`) — what moves it with the
//! drawing — and each side knows its segments in order, the material on their left
//! (`Mesh::sides`).  Deterministic: no draw, and every choice in an order of indices.

use super::predicates::{in_circle, orient2};
use std::collections::{BTreeMap, BTreeSet};

type P = [f64; 2];

const NONE: u32 = u32::MAX;

/// One side of a region: a curve from its parameter 0 to 1, whether it is straight (a line,
/// cut only where refinement asks), and the parameters strictly between its ends that must be
/// nodes (where a point is named on it).
pub struct Side<'a> {
    pub at: Box<dyn Fn(f64) -> P + 'a>,
    pub straight: bool,
    pub cuts: Vec<f64>,
}

/// A region: its loops, the outer first, each its sides end to end, and the points inside it
/// that must be nodes.  Which way a loop runs is the mesher's to read: the outer one is turned
/// counter-clockwise and the holes clockwise, so the material is on every segment's left.
pub struct Region<'a> {
    pub loops: Vec<Vec<Side<'a>>>,
    pub points: Vec<P>,
}

/// What a mesh is held to.
#[derive(Clone, Copy, Debug)]
pub struct Criteria {
    /// The most a triangle's circumradius may be over its shortest side: √2 bounds every angle
    /// below at 20.7°.
    pub ratio: f64,
    /// The most a triangle's circumradius may be.
    pub size: f64,
    /// The most a piece of a curved side may turn, in radians.
    pub turn: f64,
    /// The most points the mesh may take, past which it is refused.
    pub most: usize,
}

/// **A mesh of quadratic triangles** over a region (`mesh`): its nodes, and for each on the
/// boundary its side (counted through the loops in order) and parameter; each element's six
/// nodes, corners counter-clockwise then the middles of its sides 0–1, 1–2, 2–0 (`interior::Tri6`'s
/// numbering); and for each side of the region its segments in the order its loop runs, each
/// `[first, middle, last]` with the material on its left.
#[derive(Clone, Debug, PartialEq)]
pub struct Mesh {
    pub points: Vec<P>,
    pub on: Vec<Option<(usize, f64)>>,
    pub tris: Vec<[usize; 6]>,
    pub sides: Vec<Vec<[usize; 3]>>,
}

/// A segment of the boundary: its ends (vertices), its side, and their parameters along it.
#[derive(Clone, Copy, Debug)]
struct Seg {
    a: u32,
    b: u32,
    side: usize,
    s: [f64; 2],
}

/// The triangulation being refined: points, triangles counter-clockwise with their neighbours
/// (`nbr[t][k]` across the side from corner `k` to `k + 1`), one triangle about each point, and
/// the triangles of the region (`inside`, read again after every split).
struct Tri {
    pts: Vec<P>,
    on: Vec<Option<(usize, f64)>>,
    tris: Vec<[u32; 3]>,
    nbr: Vec<[u32; 3]>,
    about: Vec<u32>,
    inside: Vec<bool>,
    last: usize,
    mark: Vec<u32>,
    stamp: u32,
}

impl Tri {
    fn p(&self, v: u32) -> P {
        self.pts[v as usize]
    }

    /// Writes triangle `t` (a slot, or one past the last) and points its corners at it.
    fn write(&mut self, t: usize, tri: [u32; 3], nbr: [u32; 3]) {
        if t == self.tris.len() {
            self.tris.push(tri);
            self.nbr.push(nbr);
            self.inside.push(false);
        } else {
            self.tris[t] = tri;
            self.nbr[t] = nbr;
        }
        for v in tri {
            self.about[v as usize] = t as u32;
        }
    }

    /// The triangle holding `p`, on its sides included, walked to from the last one reached.
    fn locate(&mut self, p: P) -> Option<usize> {
        let mut t = self.last;
        if t >= self.tris.len() {
            t = 0;
        }
        for step in 0..=self.tris.len() {
            let mut next = None;
            for j in 0..3 {
                let k = (j + step) % 3;
                let (a, b) = (self.tris[t][k], self.tris[t][(k + 1) % 3]);
                if orient2(self.p(a), self.p(b), p) < 0 {
                    next = Some(self.nbr[t][k]);
                    break;
                }
            }
            match next {
                None => {
                    self.last = t;
                    return Some(t);
                }
                Some(NONE) => return None,
                Some(u) => t = u as usize,
            }
        }
        (0..self.tris.len()).find(|&t| {
            (0..3).all(|k| {
                orient2(self.p(self.tris[t][k]), self.p(self.tris[t][(k + 1) % 3]), p) >= 0
            })
        })
    }

    /// Point `p` into the triangulation (Bowyer–Watson): every triangle whose circle holds it, a
    /// connected cavity about the one it lies in, replaced by the fan from it to the cavity's
    /// rim.  The new point, and whether the cavity was all of the region; `None` where `p` lies
    /// outside or on a point already there.
    fn insert(&mut self, p: P, on: Option<(usize, f64)>) -> Option<(u32, bool)> {
        let t0 = self.locate(p)?;
        if self.tris[t0].iter().any(|&v| self.p(v) == p) {
            return None;
        }
        let v = self.pts.len() as u32;
        self.stamp += 1;
        let stamp = self.stamp;
        self.mark.resize(self.tris.len(), 0);
        let holds = |s: &Tri, t: usize| {
            let [a, b, c] = s.tris[t].map(|x| s.p(x));
            in_circle(a, b, c, p) > 0
        };
        let mut cavity = vec![t0];
        self.mark[t0] = stamp;
        let mut i = 0;
        while i < cavity.len() {
            let t = cavity[i];
            i += 1;
            for k in 0..3 {
                let u = self.nbr[t][k];
                if u != NONE && self.mark[u as usize] != stamp && holds(self, u as usize) {
                    self.mark[u as usize] = stamp;
                    cavity.push(u as usize);
                }
            }
        }
        let inside = cavity.iter().all(|&t| self.inside[t]);
        let mut rim = Vec::new();
        for &t in &cavity {
            for k in 0..3 {
                let u = self.nbr[t][k];
                if u == NONE || self.mark[u as usize] != stamp {
                    rim.push((self.tris[t][k], self.tris[t][(k + 1) % 3], u));
                }
            }
        }
        // a disc of k triangles has k + 2 sides round it
        if rim.len() != cavity.len() + 2 {
            return None;
        }
        self.pts.push(p);
        self.on.push(on);
        self.about.push(t0 as u32);
        let mut slots = cavity;
        let n = self.tris.len();
        slots.extend(n..n + 2);
        for (&(a, b, u), &t) in rim.iter().zip(&slots) {
            self.write(t, [a, b, v], [u, NONE, NONE]);
            if u != NONE {
                let k = (0..3).find(|&k| {
                    self.tris[u as usize][k] == b && self.tris[u as usize][(k + 1) % 3] == a
                }).expect("a neighbour shares the side");
                self.nbr[u as usize][k] = t as u32;
            }
        }
        let made = slots[..rim.len()].to_vec();
        for (i, &(a, b, _)) in rim.iter().enumerate() {
            let after = rim.iter().position(|r| r.0 == b).expect("the rim closes");
            let before = rim.iter().position(|r| r.1 == a).expect("the rim closes");
            self.nbr[slots[i]][1] = slots[after] as u32;
            self.nbr[slots[i]][2] = slots[before] as u32;
        }
        for &t in &made {
            self.inside[t] = inside;
        }
        self.last = made[0];
        Some((v, inside))
    }

    /// The triangle with the side from `a` to `b` counter-clockwise (so on that side's left), and
    /// the side's index in it, walking the triangles about `a`.
    fn edge(&self, a: u32, b: u32) -> Option<(usize, usize)> {
        let start = self.about[a as usize] as usize;
        let mut t = start;
        for _ in 0..self.tris.len() {
            let k = (0..3).find(|&k| self.tris[t][k] == a)?;
            if self.tris[t][(k + 1) % 3] == b {
                return Some((t, k));
            }
            // the next triangle about `a`, across its side from the corner before `a`
            let u = self.nbr[t][(k + 2) % 3];
            if u == NONE || u as usize == start {
                break;
            }
            t = u as usize;
        }
        // the other way about, where a fan is cut by the outermost triangle's edge
        let mut t = start;
        for _ in 0..self.tris.len() {
            let k = (0..3).find(|&k| self.tris[t][k] == a)?;
            let u = self.nbr[t][k];
            if u == NONE || u as usize == start {
                return None;
            }
            t = u as usize;
            let k = (0..3).find(|&k| self.tris[t][k] == a)?;
            if self.tris[t][(k + 1) % 3] == b {
                return Some((t, k));
            }
        }
        None
    }

    /// Whether segment `s` is encroached: it is no edge, or a point across it (one of the two
    /// triangles' third corners) is strictly inside its diametral circle.
    fn encroached(&self, s: &Seg) -> bool {
        let (a, b) = (self.p(s.a), self.p(s.b));
        let inside = |v: u32| {
            let q = self.p(v);
            (q[0] - a[0]) * (q[0] - b[0]) + (q[1] - a[1]) * (q[1] - b[1]) < 0.0
        };
        let Some((t, k)) = self.edge(s.a, s.b) else { return true };
        if inside(self.tris[t][(k + 2) % 3]) {
            return true;
        }
        let u = self.nbr[t][k];
        u != NONE && {
            let u = u as usize;
            let j = (0..3).find(|&j| self.tris[u][j] == s.b).unwrap();
            inside(self.tris[u][(j + 2) % 3])
        }
    }

    /// The circumcentre and circumradius of triangle `t`, and its shortest side.
    fn circle(&self, t: usize) -> (P, f64, f64) {
        let [a, b, c] = self.tris[t].map(|v| self.p(v));
        let (bx, by, cx, cy) = (b[0] - a[0], b[1] - a[1], c[0] - a[0], c[1] - a[1]);
        let d = 2.0 * (bx * cy - by * cx);
        let (b2, c2) = (bx * bx + by * by, cx * cx + cy * cy);
        let (ux, uy) = ((cy * b2 - by * c2) / d, (bx * c2 - cx * b2) / d);
        let side = |p: P, q: P| (p[0] - q[0]).hypot(p[1] - q[1]);
        let least = side(a, b).min(side(b, c)).min(side(c, a));
        ([a[0] + ux, a[1] + uy], ux.hypot(uy), least)
    }
}

/// **A region meshed in quadratic triangles** (#510 S2), held to `criteria`, or why it is
/// refused: a loop that does not close, a side of no length, or more points than
/// `Criteria::most` (a corner too sharp for refinement to end).
pub fn mesh(region: &Region, criteria: &Criteria) -> Result<Mesh, String> {
    // every side, counted through the loops, and which way its loop runs
    let sides: Vec<&Side> = region.loops.iter().flatten().collect();
    let mut first = 0;
    let mut lo = [f64::INFINITY; 2];
    let mut hi = [f64::NEG_INFINITY; 2];
    // each loop's points, before any is inserted: its sides' ends, and a curved side's pieces
    let mut loops: Vec<Vec<(P, usize, f64)>> = Vec::new();
    let mut backward: Vec<bool> = Vec::new();
    for (li, lp) in region.loops.iter().enumerate() {
        let mut pts = Vec::new();
        for k in 0..lp.len() {
            let g = first + k;
            let side = &lp[k];
            let end = (side.at)(1.0);
            let next = (lp[(k + 1) % lp.len()].at)(0.0);
            let gap = (end[0] - next[0]).hypot(end[1] - next[1]);
            let across = {
                let (a, b) = ((side.at)(0.0), end);
                (a[0] - b[0]).hypot(a[1] - b[1]).max(1e-300)
            };
            if lp.len() > 1 && gap > 1e-6 * across {
                return Err("a loop of the region does not close: a side does not end where the \
                            next begins".to_string());
            }
            for s in pieces(side, criteria, lp.len() == 1) {
                let p = (side.at)(s);
                for c in 0..2 {
                    lo[c] = lo[c].min(p[c]);
                    hi[c] = hi[c].max(p[c]);
                }
                pts.push((p, g, s));
            }
        }
        if pts.len() < 3 {
            return Err("a loop of the region bounds nothing".to_string());
        }
        // the outer loop counter-clockwise, a hole clockwise
        let ring: Vec<(f64, f64)> = pts.iter().map(|&(p, _, _)| (p[0], p[1])).collect();
        backward.push((crate::solid::ring_area(&ring) > 0.0) != (li == 0));
        loops.push(pts);
        first += lp.len();
    }
    let extent = (hi[0] - lo[0]).max(hi[1] - lo[1]);
    if !(extent > 0.0) {
        return Err("the region has no extent".to_string());
    }
    let mid = [(lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0];
    let far = 64.0 * extent;
    let mut tri = Tri {
        pts: vec![[mid[0] - 2.0 * far, mid[1] - far], [mid[0] + 2.0 * far, mid[1] - far],
                  [mid[0], mid[1] + 2.0 * far]],
        on: vec![None; 3],
        tris: vec![[0, 1, 2]],
        nbr: vec![[NONE; 3]],
        about: vec![0, 0, 0],
        inside: vec![false],
        last: 0,
        mark: Vec::new(),
        stamp: 0,
    };
    // every boundary point, then the segments between them along each loop: each from a point to
    // the next, along the first's side — to the side's end where the next is the next side's
    // start — and turned where the loop runs the other way, so the material is on its left
    let mut segs: Vec<Seg> = Vec::new();
    for (lp, &back) in loops.iter().zip(&backward) {
        let ids: Vec<u32> = lp.iter().map(|&(p, g, s)| {
            tri.insert(p, Some((g, s))).map(|(v, _)| v)
                .ok_or_else(|| "two points of the region's boundary are one".to_string())
        }).collect::<Result<_, _>>()?;
        for i in 0..lp.len() {
            let j = (i + 1) % lp.len();
            let (side, s0) = (lp[i].1, lp[i].2);
            let s1 = if lp[j].1 == side && lp[j].2 > s0 { lp[j].2 } else { 1.0 };
            segs.push(match back {
                false => Seg { a: ids[i], b: ids[j], side, s: [s0, s1] },
                true => Seg { a: ids[j], b: ids[i], side, s: [s1, s0] },
            });
        }
    }
    // the points inside that must be nodes, before refinement grades about them
    for &p in &region.points {
        tri.insert(p, None)
            .ok_or_else(|| "a point named inside the region is on its boundary or outside it"
                .to_string())?;
    }
    refine(&mut tri, &mut segs, &sides, criteria)?;
    Ok(extract(&tri, &segs, &sides))
}

/// A side's first cut: its start, the parameters that must be nodes, and a curved side cut where
/// it turns more than `Criteria::turn` over a piece — a side alone in its loop (a closed curve)
/// in at least four.  Its end is the next side's start.
fn pieces(side: &Side, criteria: &Criteria, closed: bool) -> Vec<f64> {
    let mut out = vec![0.0];
    let mut cuts: Vec<f64> = side.cuts.iter().copied().filter(|&c| c > 0.0 && c < 1.0).collect();
    if closed && !side.straight {
        cuts.extend([0.25, 0.5, 0.75]);
    }
    cuts.push(1.0);
    cuts.sort_by(f64::total_cmp);
    cuts.dedup();
    if side.straight {
        out.extend(&cuts[..cuts.len() - 1]);
    } else {
        let mut stack: Vec<(f64, f64)> = Vec::new();
        let mut from = 0.0;
        for &c in &cuts {
            stack.push((from, c));
            from = c;
        }
        stack.reverse();
        while let Some((a, b)) = stack.pop() {
            let m = 0.5 * (a + b);
            let (pa, pm, pb) = ((side.at)(a), (side.at)(m), (side.at)(b));
            let (u, v) = ([pm[0] - pa[0], pm[1] - pa[1]], [pb[0] - pm[0], pb[1] - pm[1]]);
            let turn = (u[0] * v[1] - u[1] * v[0]).atan2(u[0] * v[0] + u[1] * v[1]).abs();
            let long = (pb[0] - pa[0]).hypot(pb[1] - pa[1]) > criteria.size;
            if (2.0 * turn > criteria.turn || long) && b - a > 1e-9 {
                stack.push((m, b));
                stack.push((a, m));
            } else {
                out.push(b);
            }
        }
        out.pop();
    }
    out
}

/// Ruppert's refinement, to `criteria`: every encroached segment split at its middle on the
/// curve, then every bad triangle of the region its circumcentre, or the segments it encroaches
/// split instead.
fn refine(tri: &mut Tri, segs: &mut Vec<Seg>, sides: &[&Side], criteria: &Criteria)
    -> Result<(), String>
{
    let mut stuck: BTreeSet<[u32; 3]> = BTreeSet::new();
    loop {
        // conform: no segment encroached, each split looked at again with every other, as a
        // split may encroach the segments about it
        while let Some(i) = (0..segs.len()).rev().find(|&i| tri.encroached(&segs[i])) {
            split(tri, segs, sides, i)?;
            if tri.pts.len() > criteria.most {
                return Err(too_many(criteria));
            }
        }
        classify(tri, segs);
        // the bad triangles of the region, by index
        let worse: Vec<usize> = (0..tri.tris.len()).filter(|&t| {
            tri.inside[t] && bad(tri, t, criteria)
                && !stuck.contains(&tri.tris[t])
        }).collect();
        if worse.is_empty() {
            return Ok(());
        }
        for t in worse {
            if !tri.inside[t] || !bad(tri, t, criteria) {
                continue;
            }
            let (c, _, _) = tri.circle(t);
            let near: Vec<usize> = (0..segs.len()).filter(|&i| {
                let (a, b) = (tri.p(segs[i].a), tri.p(segs[i].b));
                (c[0] - a[0]) * (c[0] - b[0]) + (c[1] - a[1]) * (c[1] - b[1]) < 0.0
            }).collect();
            if !near.is_empty() {
                for i in near {
                    split(tri, segs, sides, i)?;
                }
                // conform again before another centre is tried
                break;
            }
            match tri.insert(c, None) {
                Some((_, true)) => {}
                // a centre on a point, or one the region does not hold: left, never tried again
                _ => {
                    stuck.insert(tri.tris[t]);
                }
            }
            if tri.pts.len() > criteria.most {
                return Err(too_many(criteria));
            }
        }
    }
}

fn too_many(criteria: &Criteria) -> String {
    format!("its mesh would take more than {} points: a corner of the region is too sharp for \
             its refinement to end", criteria.most)
}

/// Whether triangle `t` is bad: too long for its shortest side, or too big.
fn bad(tri: &Tri, t: usize, criteria: &Criteria) -> bool {
    let (_, r, least) = tri.circle(t);
    r > criteria.ratio * least || r > criteria.size
}

/// Segment `i` split at its parameter's middle, on its curve: its first half in its place, its
/// second after the last.
fn split(tri: &mut Tri, segs: &mut Vec<Seg>, sides: &[&Side], i: usize) -> Result<(), String> {
    let s = segs[i];
    let m = 0.5 * (s.s[0] + s.s[1]);
    let p = (sides[s.side].at)(m);
    let (v, _) = tri.insert(p, Some((s.side, m))).ok_or_else(|| {
        "a segment of the region's boundary could not be split: its middle is a point already, or \
         lies outside the triangulation".to_string()
    })?;
    segs[i] = Seg { a: s.a, b: v, side: s.side, s: [s.s[0], m] };
    segs.push(Seg { a: v, b: s.b, side: s.side, s: [m, s.s[1]] });
    Ok(())
}

/// Which triangles are of the region: those on each segment's left, and every one reached from
/// them without crossing a segment.
fn classify(tri: &mut Tri, segs: &[Seg]) {
    let held: BTreeSet<(u32, u32)> =
        segs.iter().map(|s| (s.a.min(s.b), s.a.max(s.b))).collect();
    tri.inside.iter_mut().for_each(|v| *v = false);
    let mut stack: Vec<usize> = segs.iter()
        .filter_map(|s| tri.edge(s.a, s.b).map(|(t, _)| t)).collect();
    for &t in &stack {
        tri.inside[t] = true;
    }
    while let Some(t) = stack.pop() {
        for k in 0..3 {
            let (a, b) = (tri.tris[t][k], tri.tris[t][(k + 1) % 3]);
            let u = tri.nbr[t][k];
            if u == NONE || held.contains(&(a.min(b), a.max(b))) || tri.inside[u as usize] {
                continue;
            }
            tri.inside[u as usize] = true;
            stack.push(u as usize);
        }
    }
}

/// The region's triangles as quadratic elements, renumbered: the corners in the order made, then
/// the middles; each side's segments in its loop's order.
fn extract(tri: &Tri, segs: &[Seg], sides: &[&Side]) -> Mesh {
    let mut index: Vec<Option<usize>> = vec![None; tri.pts.len()];
    let mut out = Mesh { points: Vec::new(), on: Vec::new(), tris: Vec::new(),
                         sides: vec![Vec::new(); sides.len()] };
    let kept: Vec<usize> = (0..tri.tris.len()).filter(|&t| tri.inside[t]).collect();
    for &t in &kept {
        for v in tri.tris[t] {
            if index[v as usize].is_none() {
                index[v as usize] = Some(out.points.len());
                out.points.push(tri.p(v));
                out.on.push(tri.on[v as usize]);
            }
        }
    }
    let by_edge: BTreeMap<(u32, u32), &Seg> =
        segs.iter().map(|s| ((s.a.min(s.b), s.a.max(s.b)), s)).collect();
    let mut middles: BTreeMap<(u32, u32), usize> = BTreeMap::new();
    for &t in &kept {
        let c = tri.tris[t];
        let mut el = [0usize; 6];
        for k in 0..3 {
            el[k] = index[c[k] as usize].unwrap();
            let (a, b) = (c[k], c[(k + 1) % 3]);
            let key = (a.min(b), a.max(b));
            el[3 + k] = *middles.entry(key).or_insert_with(|| {
                let (p, on) = match by_edge.get(&key) {
                    Some(s) => {
                        let m = 0.5 * (s.s[0] + s.s[1]);
                        ((sides[s.side].at)(m), Some((s.side, m)))
                    }
                    None => {
                        let (p, q) = (tri.p(a), tri.p(b));
                        ([0.5 * (p[0] + q[0]), 0.5 * (p[1] + q[1])], None)
                    }
                };
                out.points.push(p);
                out.on.push(on);
                out.points.len() - 1
            });
        }
        out.tris.push(el);
    }
    // each side's segments, along its loop: chained from end to end
    let mut along: Vec<Vec<&Seg>> = vec![Vec::new(); sides.len()];
    for s in segs.iter() {
        along[s.side].push(s);
    }
    for (g, list) in along.iter().enumerate() {
        let mut ordered: Vec<&Seg> = Vec::with_capacity(list.len());
        let start = list.iter().find(|s| !list.iter().any(|o| o.b == s.a)).or(list.first());
        if let Some(start) = start {
            ordered.push(start);
            while ordered.len() < list.len() {
                let end = ordered.last().unwrap().b;
                match list.iter().find(|s| s.a == end) {
                    Some(s) => ordered.push(s),
                    None => break,
                }
            }
        }
        out.sides[g] = ordered.iter().map(|s| {
            let key = (s.a.min(s.b), s.a.max(s.b));
            [index[s.a as usize].unwrap(), middles[&key], index[s.b as usize].unwrap()]
        }).collect();
    }
    out
}
