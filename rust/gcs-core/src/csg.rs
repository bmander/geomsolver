//! **Boundary evaluation**: the surface of a term, found by classifying candidates against it.
//!
//! Requicha & Voelcker's route, and the reason there is no B-rep in this project.  The boundary
//! of `A ∪ B` or `A − B` is a *subset* of the boundaries of `A` and `B`, so nothing has to be
//! constructed: cut every primitive's facets by the planes of every other primitive that reaches
//! them, and ask of each piece whether the material is on one side of it.  Exactly one side
//! inside is a face of the solid; both or neither is interior or exterior, and is dropped.
//!
//! The same trick gives the *edges* a view draws, one dimension down: a candidate is a seam
//! between two facets or the meeting of two primitives' facets, and it is an edge of the solid
//! where the material around it forms a wedge rather than a slab or nothing.  A seam that is
//! merely tessellation — the flats of a bore's wall — is `smooth` and is drawn only where it is
//! a *silhouette*, which is what makes a cylinder two lines instead of sixty-four.
//!
//! Every walk here is culled by bounding box before it is paid for, and every container is
//! ordered, so the answer does not depend on how the drawing was written down.

#[allow(unused_imports)]
use crate::fmath::Det;
use crate::plane;
use crate::solid::{Box3, Csg, Facet, Prim, RayIndex};

/// A face of the solid: a planar convex piece of some primitive's facet that survived
/// classification, with its outward normal and the path the document reaches it by.
#[derive(Clone, Debug)]
pub struct Piece {
    pub pts: Vec<[f64; 3]>,
    pub n: [f64; 3],
    /// `body.bore.wall` — the solid, the primitive, the drawn edge it was swept from.
    pub path: String,
    pub prim: usize,
    pub smooth: bool,
}

impl Piece {
    pub fn centroid(&self) -> [f64; 3] {
        let k = 1.0 / self.pts.len() as f64;
        let mut c = [0.0; 3];
        for p in &self.pts {
            for i in 0..3 {
                c[i] += p[i] * k;
            }
        }
        c
    }

    pub fn bbox(&self) -> Box3 {
        self.bbox_of()
    }

    pub fn bbox_of(&self) -> Box3 {
        let mut b = Box3::empty();
        for p in &self.pts {
            b.add(*p);
        }
        b
    }

    /// The unsigned area, evaluated in a local frame.
    pub fn area(&self) -> f64 {
        plane::norm(crate::solid::area_vector(&self.pts)) / 2.0
    }
}

/// An edge of the solid, as a view would draw it: the segment, the two surfaces meeting along
/// it, and whether that meeting is a corner of the design or a chord of a tessellation.
#[derive(Clone, Debug)]
pub struct Edge {
    pub a: [f64; 3],
    pub b: [f64; 3],
    pub na: [f64; 3],
    pub nb: [f64; 3],
    /// A tessellation seam: drawn only where it is a silhouette in the view being taken.
    pub smooth: bool,
    pub path: String,
}

/// The cutting planes of a primitive, deduped — many facets of one cap share one plane, and
/// splitting a piece by the same plane sixty times is sixty times the pieces.
fn planes_of(p: &Prim) -> Vec<([f64; 3], f64, Box3)> {
    let mut out: Vec<([f64; 3], f64, Box3)> = Vec::new();
    for f in &p.facets {
        let (n, d) = (f.n, f.offset());
        let b = f.bbox();
        if let Some(e) = out.iter_mut().find(|(m, o, _)| same_plane(*m, *o, n, d)) {
            e.2.add(b.lo);
            e.2.add(b.hi);
        } else {
            out.push((n, d, b));
        }
    }
    out
}

fn same_plane(n1: [f64; 3], d1: f64, n2: [f64; 3], d2: f64) -> bool {
    let dot = plane::dot(n1, n2);
    let par = (dot.abs() - 1.0).abs() < 1e-9;
    if !par {
        return false;
    }
    let d2 = if dot > 0.0 { d2 } else { -d2 };
    (d1 - d2).abs() < 1e-9 * (1.0 + d1.abs().max(d2.abs()))
}

/// `body.bore.wall`: the solid the primitive came from, and the drawn edge its facet was swept
/// from.  A path and never an index — a boolean never renames, which is why the naming problem
/// every history-based kernel has does not arise here.
fn path_of(prim: &Prim, f: &Facet) -> String {
    match prim.faces.get(f.face) {
        Some(n) if prim.exact => n.clone(),
        Some(n) => format!("{}.{}", prim.of, n),
        None => prim.of.clone(),
    }
}

// -- boundary evaluation, by binary space partition ---------------------------------------------
//
// The first version of this cut every facet by every plane of every primitive that reached it and
// classified the pieces.  That is Requicha & Voelcker exactly, and on a *square* hole it is exact
// — but the pieces go as the square of the facets: a block's cap cut by the six hundred wall
// planes of a faceted bore is the arrangement of six hundred lines, twenty-five thousand cells of
// which the drawing needs about six hundred.  Bounding boxes do not save it, because the piece
// left outside a chord still stretches across the whole cap.
//
// A BSP prunes exactly what that loop could not.  Descending the tree, a piece that lands wholly
// in front of a node's plane is *decided* and goes no further, so a cap against a cylinder costs
// pieces in proportion to the facets and not to their square.  It is the same set of planes and
// the same answer; what changes is that the walk stops asking once it knows.

/// A facet on its way through the tree, carrying where it came from.
#[derive(Clone, Debug)]
struct Poly {
    pts: Vec<[f64; 3]>,
    n: [f64; 3],
    w: f64,
    path: String,
    prim: usize,
    smooth: bool,
    /// Which node of its tree the polygon came from, while `clip_to` clips a tree's polygons
    /// as one batch; its pieces keep it.
    tag: u32,
}

impl Poly {
    fn flip(&mut self) {
        self.pts.reverse();
        self.n = plane::scaled(self.n, -1.0);
        self.w = -self.w;
    }
}

/// Where a vertex falls against a plane.  The tolerance is relative to the piece, so a facet far
/// from the origin is judged as sharply as one at it.
const FRONT: u8 = 1;
const BACK: u8 = 2;

/// Which side of a plane a polygon lies on: on it (facing the plane's way or not), wholly in
/// front, wholly behind, or across it.
#[derive(Clone, Copy)]
enum Side {
    Coplanar(bool),
    Front,
    Back,
    Spanning,
}

fn side(poly: &Poly, n: [f64; 3], w: f64, tol: f64) -> Side {
    let mut kind = 0u8;
    for p in &poly.pts {
        let t = plane::dot(n, *p) - w;
        if t < -tol {
            kind |= BACK;
        } else if t > tol {
            kind |= FRONT;
        }
    }
    match kind {
        0 => Side::Coplanar(plane::dot(n, poly.n) > 0.0),
        FRONT => Side::Front,
        BACK => Side::Back,
        _ => Side::Spanning,
    }
}

/// Whether no polygon of a batch spans the plane and the ones off it all lie on one side:
/// `Some(true)` in front, `Some(false)` behind (or every polygon on the plane), `None` when
/// some polygon spans it or some lie on each side.
fn one_side(sides: &[Side]) -> Option<bool> {
    let (mut front, mut back) = (false, false);
    for s in sides {
        match s {
            Side::Coplanar(_) => {}
            Side::Front => front = true,
            Side::Back => back = true,
            Side::Spanning => return None,
        }
        if front && back {
            return None;
        }
    }
    Some(front)
}

/// A spanning polygon cut in two by the plane: the part in front and the part behind, either
/// dropped when the cut leaves it fewer than three vertices.
fn split(poly: Poly, n: [f64; 3], w: f64, tol: f64) -> (Option<Poly>, Option<Poly>) {
    let kind = |p: [f64; 3]| {
        let t = plane::dot(n, p) - w;
        if t < -tol {
            BACK
        } else if t > tol {
            FRONT
        } else {
            0
        }
    };
    let (mut f, mut b) = (Vec::new(), Vec::new());
    for i in 0..poly.pts.len() {
        let j = (i + 1) % poly.pts.len();
        let (vi, vj) = (poly.pts[i], poly.pts[j]);
        let (ti, tj) = (kind(vi), kind(vj));
        if ti != BACK {
            f.push(vi);
        }
        if ti != FRONT {
            b.push(vi);
        }
        if (ti | tj) == (FRONT | BACK) {
            let di = plane::dot(n, vi) - w;
            let dj = plane::dot(n, vj) - w;
            let t = di / (di - dj);
            let x = [
                vi[0] + t * (vj[0] - vi[0]),
                vi[1] + t * (vj[1] - vi[1]),
                vi[2] + t * (vj[2] - vi[2]),
            ];
            f.push(x);
            b.push(x);
        }
    }
    let front = (f.len() >= 3).then(|| Poly { pts: f, path: poly.path.clone(), ..poly });
    let back = (b.len() >= 3).then(|| Poly { pts: b, ..poly });
    (front, back)
}

#[derive(Debug, Default)]
struct Node {
    plane: Option<([f64; 3], f64)>,
    front: Option<Box<Node>>,
    back: Option<Box<Node>>,
    polys: Vec<Poly>,
}

// Every walk over the tree is a loop over an explicit stack, never a recursion: a convex
// solid's tree is a chain as deep as the solid has facets (every other facet lies behind each
// facet's plane), so a finely cut sphere is thousands of levels, and a recursive walk ran out
// of stack on one.

impl Node {
    fn new(polys: Vec<Poly>, tol: f64) -> Node {
        let mut n = Node::default();
        n.build(polys, tol);
        n
    }

    fn build(&mut self, polys: Vec<Poly>, tol: f64) {
        let mut work: Vec<(&mut Node, Vec<Poly>)> = vec![(self, polys)];
        // each polygon is classified once per level, into one buffer the whole build reuses
        let mut sides: Vec<Side> = Vec::new();
        while let Some((node, polys)) = work.pop() {
            if polys.is_empty() {
                continue;
            }
            if polys.len() >= SPINE {
                build_spine(node, polys, tol, &mut work);
                continue;
            }
            if node.plane.is_none() {
                node.plane = Some((polys[0].n, polys[0].w));
            }
            let (n, w) = node.plane.expect("just set");
            sides.clear();
            sides.extend(polys.iter().map(|p| side(p, n, w, tol)));
            let (fr, bk) = match one_side(&sides) {
                // a chain's usual step: the node keeps its own plane's polygons and the rest go
                // on as the batch they came in, in order, without a new allocation
                Some(front) => {
                    let mut rest = polys;
                    let mut at = sides.iter();
                    node.polys.extend(rest.extract_if(.., |_| matches!(at.next(), Some(Side::Coplanar(_)))));
                    if front { (rest, Vec::new()) } else { (Vec::new(), rest) }
                }
                None => {
                    let (mut fr, mut bk) = (Vec::new(), Vec::new());
                    // every polygon moves to where it belongs: one that is not cut is never copied
                    for (p, s) in polys.into_iter().zip(sides.iter().copied()) {
                        match s {
                            Side::Coplanar(_) => node.polys.push(p),
                            Side::Front => fr.push(p),
                            Side::Back => bk.push(p),
                            Side::Spanning => {
                                let (f, b) = split(p, n, w, tol);
                                fr.extend(f);
                                bk.extend(b);
                            }
                        }
                    }
                    (fr, bk)
                }
            };
            let Node { front, back, .. } = node;
            if !bk.is_empty() {
                work.push((back.get_or_insert_with(Default::default), bk));
            }
            if !fr.is_empty() {
                work.push((front.get_or_insert_with(Default::default), fr));
            }
        }
    }

    /// The parts of `polys` that lie **outside** this solid, in the order a front-first walk
    /// of the tree reaches them.
    fn clip(&self, polys: Vec<Poly>, tol: f64) -> Vec<Poly> {
        let mut out = Vec::new();
        let mut work: Vec<Item> = vec![Item::Plain(self, polys)];
        let mut sides: Vec<Side> = Vec::new();
        while let Some(item) = work.pop() {
            let (node, polys) = match item {
                Item::Spine(node, sp) => {
                    clip_spine(node, sp, tol, &mut out, &mut work);
                    continue;
                }
                Item::Plain(node, polys) => (node, polys),
            };
            let Some((n, w)) = node.plane else {
                out.extend(polys);
                continue;
            };
            if polys.is_empty() {
                continue;
            }
            if polys.len() >= SPINE {
                clip_spine(node, Spine::new(polys), tol, &mut out, &mut work);
                continue;
            }
            // a batch wholly on one side (coplanar pieces going the way they face) goes on
            // whole: descending a chain, that is every level but the last
            sides.clear();
            sides.extend(polys.iter().map(|p| side(p, n, w, tol)));
            let faces = |s: &Side| matches!(s, Side::Coplanar(true) | Side::Front);
            let backs = |s: &Side| matches!(s, Side::Coplanar(false) | Side::Back);
            let (fr, bk) = match () {
                _ if sides.iter().all(faces) => (polys, Vec::new()),
                _ if sides.iter().all(backs) => (Vec::new(), polys),
                _ => {
                    let (mut fr, mut bk) = (Vec::new(), Vec::new());
                    for (p, s) in polys.into_iter().zip(sides.iter().copied()) {
                        match s {
                            Side::Coplanar(true) | Side::Front => fr.push(p),
                            Side::Coplanar(false) | Side::Back => bk.push(p),
                            Side::Spanning => {
                                let (f, b) = split(p, n, w, tol);
                                fr.extend(f);
                                bk.extend(b);
                            }
                        }
                    }
                    (fr, bk)
                }
            };
            // nothing behind the deepest plane is outside the solid, so the walk stops there —
            // the pruning the flat loop could not do
            if let Some(b) = &node.back {
                if !bk.is_empty() {
                    work.push(Item::Plain(b, bk));
                }
            }
            // what lies in front is finished before what lies behind, so the front's pieces
            // leave first
            match &node.front {
                Some(f) => work.push(Item::Plain(f, fr)),
                None => out.extend(fr),
            }
        }
        out
    }

    /// Every polygon of this tree clipped to `other`, as one batch tagged by the node it came
    /// from in a fixed walk: what `other` keeps of a polygon owes nothing to the rest of its
    /// batch, and a node's own pieces come out in the order a clip of its polygons alone gives
    /// them, so one batch down `other`'s chains does the work of a walk per node.
    fn clip_to(&mut self, other: &Node, tol: f64) {
        let mut batch = Vec::new();
        let mut nodes = 0u32;
        let mut work: Vec<&mut Node> = vec![&mut *self];
        while let Some(node) = work.pop() {
            for mut p in std::mem::take(&mut node.polys) {
                p.tag = nodes;
                batch.push(p);
            }
            nodes += 1;
            let Node { front, back, .. } = node;
            if let Some(b) = back.as_deref_mut() {
                work.push(b);
            }
            if let Some(f) = front.as_deref_mut() {
                work.push(f);
            }
        }
        let mut pieces: Vec<Vec<Poly>> = (0..nodes).map(|_| Vec::new()).collect();
        for p in other.clip(batch, tol) {
            let at = p.tag as usize;
            pieces[at].push(p);
        }
        let mut at = 0;
        let mut work: Vec<&mut Node> = vec![self];
        while let Some(node) = work.pop() {
            node.polys = std::mem::take(&mut pieces[at]);
            at += 1;
            let Node { front, back, .. } = node;
            if let Some(b) = back.as_deref_mut() {
                work.push(b);
            }
            if let Some(f) = front.as_deref_mut() {
                work.push(f);
            }
        }
    }

    fn invert(&mut self) {
        let mut work: Vec<&mut Node> = vec![self];
        while let Some(node) = work.pop() {
            for p in node.polys.iter_mut() {
                p.flip();
            }
            if let Some((n, w)) = node.plane {
                node.plane = Some((plane::scaled(n, -1.0), -w));
            }
            std::mem::swap(&mut node.front, &mut node.back);
            let Node { front, back, .. } = node;
            if let Some(b) = back.as_deref_mut() {
                work.push(b);
            }
            if let Some(f) = front.as_deref_mut() {
                work.push(f);
            }
        }
    }

    /// Every polygon in the tree, each node's before its front subtree's before its back's.
    fn all(&self) -> Vec<Poly> {
        let mut v = Vec::new();
        let mut work: Vec<&Node> = vec![self];
        while let Some(node) = work.pop() {
            v.extend(node.polys.iter().cloned());
            if let Some(b) = &node.back {
                work.push(b);
            }
            if let Some(f) = &node.front {
                work.push(f);
            }
        }
        v
    }
}

/// A batch this large goes down a chain of the tree as `build_spine` and `clip_spine` walk it;
/// below it the plain step is cheaper than the spheres.
const SPINE: usize = 32;

/// Bounding spheres over a batch of polygons, in a tree of median splits: which of the batch
/// might be anything but strictly on one side of a plane, found without asking the rest.
struct Balls {
    balls: Vec<Ball>,
    order: Vec<usize>,
}

struct Ball {
    center: [f64; 3],
    radius: f64,
    lo: usize,
    hi: usize,
    kids: Option<(usize, usize)>,
}

impl Balls {
    fn new(polys: &[Option<Poly>]) -> Balls {
        let centroids: Vec<[f64; 3]> = polys
            .iter()
            .map(|p| {
                let p = p.as_ref().expect("a fresh batch");
                let k = 1.0 / p.pts.len() as f64;
                let mut c = [0.0; 3];
                for v in &p.pts {
                    for i in 0..3 {
                        c[i] += v[i] * k;
                    }
                }
                c
            })
            .collect();
        let mut out = Balls { balls: Vec::new(), order: (0..polys.len()).collect() };
        out.split(polys, &centroids, 0, polys.len());
        out
    }

    fn split(&mut self, polys: &[Option<Poly>], centroids: &[[f64; 3]], lo: usize, hi: usize) -> usize {
        let (mut min, mut max) = ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]);
        for &i in &self.order[lo..hi] {
            for v in &polys[i].as_ref().expect("a fresh batch").pts {
                for k in 0..3 {
                    min[k] = min[k].min(v[k]);
                    max[k] = max[k].max(v[k]);
                }
            }
        }
        let center: [f64; 3] = std::array::from_fn(|k| 0.5 * (min[k] + max[k]));
        let mut radius = 0.0f64;
        for &i in &self.order[lo..hi] {
            for v in &polys[i].as_ref().expect("a fresh batch").pts {
                radius = radius.max(plane::norm([v[0] - center[0], v[1] - center[1], v[2] - center[2]]));
            }
        }
        let at = self.balls.len();
        self.balls.push(Ball { center, radius: radius * (1.0 + 1e-12), lo, hi, kids: None });
        if hi - lo > 4 {
            // the longest axis of the centroids' box, split at its median
            let (mut cmin, mut cmax) = ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]);
            for &i in &self.order[lo..hi] {
                for k in 0..3 {
                    cmin[k] = cmin[k].min(centroids[i][k]);
                    cmax[k] = cmax[k].max(centroids[i][k]);
                }
            }
            let axis = (0..3).max_by(|&a, &b| (cmax[a] - cmin[a]).total_cmp(&(cmax[b] - cmin[b]))).unwrap_or(0);
            let mid = (lo + hi) / 2;
            self.order[lo..hi].select_nth_unstable_by(mid - lo, |&a, &b| {
                centroids[a][axis].total_cmp(&centroids[b][axis]).then(a.cmp(&b))
            });
            let left = self.split(polys, centroids, lo, mid);
            let right = self.split(polys, centroids, mid, hi);
            self.balls[at].kids = Some((left, right));
        }
        at
    }

    /// Every live polygon of which some vertex may lie within `tol` of the plane `m·x = e` or
    /// behind it; the rest lie strictly in front of it, beyond `tol`, and need no test.
    fn reaching(&self, polys: &[Option<Poly>], m: [f64; 3], e: f64, tol: f64, out: &mut Vec<usize>) {
        let mut stack = vec![0];
        while let Some(b) = stack.pop() {
            let ball = &self.balls[b];
            let s = plane::dot(m, ball.center) - e;
            // a margin far above the rounding of the per-vertex test the ball stands in for
            let slack = 1e-12 * (e.abs() + plane::norm(ball.center) + ball.radius);
            if s - ball.radius - slack > tol {
                continue;
            }
            match ball.kids {
                Some((l, r)) => {
                    stack.push(l);
                    stack.push(r);
                }
                None => out.extend(self.order[ball.lo..ball.hi].iter().copied().filter(|&i| polys[i].is_some())),
            }
        }
    }
}

/// A batch on its way down a tree: its polygons (each slot emptied as the polygon leaves), the
/// spheres over them, how many are left, the first still there, and the side it last went on.
struct Spine {
    polys: Vec<Option<Poly>>,
    balls: Balls,
    remaining: usize,
    first: usize,
    reached: Vec<usize>,
    forward: bool,
}

impl Spine {
    fn new(batch: Vec<Poly>) -> Spine {
        let polys: Vec<Option<Poly>> = batch.into_iter().map(Some).collect();
        let balls = Balls::new(&polys);
        Spine { remaining: polys.len(), polys, balls, first: 0, reached: Vec::new(), forward: false }
    }

    /// Into `reached`, in batch order: the live polygons not strictly on the side of the plane
    /// the batch is to go on, the only ones that side's step has to classify.
    fn reach(&mut self, pn: [f64; 3], w: f64, forward: bool, tol: f64) {
        let (m, e) = if forward { (pn, w) } else { (plane::scaled(pn, -1.0), -w) };
        self.reached.clear();
        self.balls.reaching(&self.polys, m, e, tol, &mut self.reached);
        self.reached.sort_unstable();
    }

    /// At a node where the batch could go on either way: the side it went on last, unless more
    /// than half of it reaches the plane from there. The smaller part is what leaves as a batch
    /// of its own, so no polygon is gathered into new spheres more than a logarithm of times.
    fn choose(&mut self, pn: [f64; 3], w: f64, tol: f64) -> bool {
        let first = self.forward;
        self.reach(pn, w, first, tol);
        let side = if 2 * self.reached.len() <= self.remaining {
            first
        } else {
            self.reach(pn, w, !first, tol);
            !first
        };
        self.forward = side;
        side
    }

    fn take(&mut self, i: usize) -> Poly {
        self.remaining -= 1;
        self.polys[i].take().expect("a live polygon")
    }

    fn first_live(&mut self) -> &Poly {
        while self.polys[self.first].is_none() {
            self.first += 1;
        }
        self.polys[self.first].as_ref().expect("a live polygon")
    }

    fn rest(self) -> Vec<Poly> {
        self.polys.into_iter().flatten().collect()
    }
}

/// A build down the tree from `node`: at every node the batch goes on along one side, and what
/// goes to the other leaves as a work item of its own, in order (the subtrees build
/// independently). The polygons reaching a node's plane are found by `Balls` and classified as
/// the plain step classifies them; the rest lie strictly on the side the batch goes on, where
/// the plain step would have put them. A convex solid's tree is one chain, so its build asks
/// each plane about its neighbours only, not about every polygon left.
fn build_spine<'a>(mut node: &'a mut Node, batch: Vec<Poly>, tol: f64, work: &mut Vec<(&'a mut Node, Vec<Poly>)>) {
    let mut sp = Spine::new(batch);
    loop {
        if sp.remaining == 0 {
            return;
        }
        if node.plane.is_none() {
            let p = sp.first_live();
            node.plane = Some((p.n, p.w));
        }
        let (pn, w) = node.plane.expect("just set");
        let forward = sp.choose(pn, w, tol);
        let mut leaving = Vec::new();
        let reached = std::mem::take(&mut sp.reached);
        for &i in &reached {
            match side(sp.polys[i].as_ref().expect("live"), pn, w, tol) {
                Side::Coplanar(_) => node.polys.push(sp.take(i)),
                Side::Front if !forward => leaving.push(sp.take(i)),
                Side::Back if forward => leaving.push(sp.take(i)),
                Side::Front | Side::Back => {}
                Side::Spanning => {
                    let (f, b) = split(sp.take(i), pn, w, tol);
                    let (stays, leaves) = if forward { (f, b) } else { (b, f) };
                    leaving.extend(leaves);
                    if let Some(q) = stays {
                        sp.polys[i] = Some(q);
                        sp.remaining += 1;
                    }
                }
            }
        }
        sp.reached = reached;
        let Node { front, back, .. } = node;
        let (on, off) = if forward { (front, back) } else { (back, front) };
        if !leaving.is_empty() {
            work.push((off.get_or_insert_with(Default::default), leaving));
        }
        if sp.remaining == 0 {
            return;
        }
        node = on.get_or_insert_with(Default::default);
    }
}

/// A step of `clip`'s walk: a batch at a node, plain or with its spheres.
enum Item<'a> {
    Plain(&'a Node, Vec<Poly>),
    Spine(&'a Node, Spine),
}

/// A clip down the tree from `node`: where the node has one child the batch goes on along it
/// and a polygon leaving by the other side leaves the solid (in front, kept) or its inside
/// (behind, dropped); at a node with both, it goes on along the side most of it takes, the rest
/// leaving as a batch of its own, and the front's pieces still come out before the back's (a
/// batch going on behind waits on the stack under the one sent in front). Classified as
/// `build_spine` finds them.
fn clip_spine<'a>(mut node: &'a Node, mut sp: Spine, tol: f64, out: &mut Vec<Poly>, work: &mut Vec<Item<'a>>) {
    loop {
        if sp.remaining == 0 {
            return;
        }
        let Some((pn, w)) = node.plane else {
            out.extend(sp.rest());
            return;
        };
        let (forward, next, branch) = match (&node.front, &node.back) {
            (Some(f), Some(b)) => {
                let forward = sp.choose(pn, w, tol);
                (forward, if forward { &**f } else { &**b }, true)
            }
            (Some(f), None) => {
                sp.forward = true;
                sp.reach(pn, w, true, tol);
                (true, &**f, false)
            }
            (None, Some(b)) => {
                sp.forward = false;
                sp.reach(pn, w, false, tol);
                (false, &**b, false)
            }
            (None, None) => {
                // a leaf: everything leaves, in front kept and behind dropped
                for p in sp.rest() {
                    match side(&p, pn, w, tol) {
                        Side::Coplanar(true) | Side::Front => out.push(p),
                        Side::Coplanar(false) | Side::Back => {}
                        Side::Spanning => out.extend(split(p, pn, w, tol).0),
                    }
                }
                return;
            }
        };
        // what leaves by the other side: kept or dropped with no child there, a batch for the
        // child with one
        let mut leaving = Vec::new();
        let reached = std::mem::take(&mut sp.reached);
        for &i in &reached {
            let s = side(sp.polys[i].as_ref().expect("live"), pn, w, tol);
            match s {
                Side::Spanning => {
                    let (f, b) = split(sp.take(i), pn, w, tol);
                    let (stays, leaves) = if forward { (f, b) } else { (b, f) };
                    leaving.extend(leaves);
                    if let Some(q) = stays {
                        sp.polys[i] = Some(q);
                        sp.remaining += 1;
                    }
                }
                _ if matches!(s, Side::Coplanar(true) | Side::Front) == forward => {}
                _ => leaving.push(sp.take(i)),
            }
        }
        sp.reached = reached;
        if !branch {
            if !forward {
                out.extend(leaving);
            }
            node = next;
            continue;
        }
        let (front, back) = (node.front.as_deref().expect("a branch"), node.back.as_deref().expect("a branch"));
        if forward {
            // the front goes on here and now; the back waits its turn
            work.push(Item::Plain(back, leaving));
            node = front;
        } else {
            work.push(Item::Spine(back, sp));
            work.push(Item::Plain(front, leaving));
            return;
        }
    }
}

impl Drop for Node {
    /// Taken apart one level at a time: the derived drop recurses down the chain.
    fn drop(&mut self) {
        let mut work: Vec<Box<Node>> = self.front.take().into_iter().chain(self.back.take()).collect();
        while let Some(mut node) = work.pop() {
            work.extend(node.front.take());
            work.extend(node.back.take());
        }
    }
}

fn union(a: Vec<Poly>, b: Vec<Poly>, tol: f64) -> Vec<Poly> {
    if a.is_empty() {
        return b;
    }
    if b.is_empty() {
        return a;
    }
    let mut na = Node::new(a, tol);
    let mut nb = Node::new(b, tol);
    na.clip_to(&nb, tol);
    nb.clip_to(&na, tol);
    nb.invert();
    nb.clip_to(&na, tol);
    nb.invert();
    na.build(nb.all(), tol);
    na.all()
}

fn difference(a: Vec<Poly>, b: Vec<Poly>, tol: f64) -> Vec<Poly> {
    if a.is_empty() || b.is_empty() {
        return a;
    }
    let mut na = Node::new(a, tol);
    let mut nb = Node::new(b, tol);
    na.invert();
    na.clip_to(&nb, tol);
    nb.clip_to(&na, tol);
    nb.invert();
    nb.clip_to(&na, tol);
    nb.invert();
    na.build(nb.all(), tol);
    na.invert();
    na.all()
}

fn intersection(a: Vec<Poly>, b: Vec<Poly>, tol: f64) -> Vec<Poly> {
    if a.is_empty() || b.is_empty() {
        return Vec::new();
    }
    let mut na = Node::new(a, tol);
    let mut nb = Node::new(b, tol);
    na.invert();
    nb.clip_to(&na, tol);
    nb.invert();
    na.clip_to(&nb, tol);
    nb.clip_to(&na, tol);
    na.build(nb.all(), tol);
    na.invert();
    na.all()
}

/// Common material, evaluated in one local frame. Boundary sampling cannot find all crossings.
pub(crate) fn common_boundary(a: &Csg, b: &Csg, eps: f64) -> Vec<Piece> {
    if !a.bbox().overlaps(&b.bbox()) {
        return Vec::new();
    }
    let origin = a
        .prims
        .iter()
        .flat_map(|p| &p.facets)
        .find_map(|f| f.pts.first())
        .copied()
        .unwrap_or([0.0; 3]);
    let tol = eps * 1e-3;
    intersection(polys_of(a, &a.term, tol, origin), polys_of(b, &b.term, tol, origin), tol)
        .into_iter()
        .filter(|p| p.pts.len() >= 3)
        .map(|p| Piece {
            pts: p.pts.iter().map(|v| std::array::from_fn(|k| v[k] + origin[k])).collect(),
            n: p.n,
            path: p.path,
            prim: p.prim,
            smooth: p.smooth,
        })
        .collect()
}

fn polys_of(csg: &Csg, t: &crate::solid::Term, tol: f64, origin: [f64; 3]) -> Vec<Poly> {
    use crate::solid::Term;
    match t {
        Term::Empty => Vec::new(),
        Term::Prim(i) => {
            let prim = &csg.prims[*i];
            prim
                .facets
                .iter()
                .map(|f| {
                    let pts: Vec<_> = f.pts.iter().map(|p| {
                        [p[0] - origin[0], p[1] - origin[1], p[2] - origin[2]]
                    }).collect();
                    Poly {
                        w: plane::dot(f.n, pts[0]),
                        pts,
                        n: f.n,
                        path: path_of(prim, f),
                        prim: *i,
                        smooth: f.smooth,
                        tag: 0,
                    }
                })
                .collect()
        }
        Term::Union(a, b) => {
            union(polys_of(csg, a, tol, origin), polys_of(csg, b, tol, origin), tol)
        }
        Term::Diff(a, b) => {
            difference(polys_of(csg, a, tol, origin), polys_of(csg, b, tol, origin), tol)
        }
        Term::Inter(a, b) => {
            intersection(polys_of(csg, a, tol, origin), polys_of(csg, b, tol, origin), tol)
        }
    }
}

/// **The boundary of a term**: the faces of the solid, each carrying the path the document
/// reaches it by.
pub fn boundary(csg: &Csg, eps: f64) -> Vec<Piece> {
    // Split in a local frame. At large world coordinates even a facet's own vertices can
    // fall off its rounded plane, so a BSP node repeatedly splits its first polygon forever.
    let origin = csg.prims.iter().flat_map(|p| &p.facets)
        .find_map(|f| f.pts.first()).copied().unwrap_or([0.0; 3]);
    let polys = polys_of(csg, &csg.term, eps * 1e-3, origin);
    let mut out: Vec<Piece> = polys
        .into_iter()
        .filter(|p| p.pts.len() >= 3)
        .map(|p| Piece {
            pts: p.pts.into_iter().map(|v| {
                [v[0] + origin[0], v[1] + origin[1], v[2] + origin[2]]
            }).collect(),
            n: p.n, path: p.path, prim: p.prim, smooth: p.smooth,
        })
        .collect();
    // a sliver with no area is no face: the cut leaves them wherever a plane grazes a corner,
    // and they carry no volume, no ink and no name anyone would ask for
    let big = out.iter().map(|p| p.area()).fold(0.0f64, f64::max);
    out.retain(|p| p.area() > big * 1e-12);
    out
}

/// Containment is the emptiness of A − B, including portions of B's voids enclosed by A.
/// Sampling only A's exterior misses both enclosed cavities and unsampled boundary crossings.
pub(crate) fn contains_boundary(b: &Csg, a: &[Piece], eps: f64) -> bool {
    let origin = a.iter().find_map(|p| p.pts.first()).copied().unwrap_or([0.0; 3]);
    let ap = a.iter().map(|p| {
        let pts: Vec<_> = p.pts.iter().map(|v| {
            [v[0] - origin[0], v[1] - origin[1], v[2] - origin[2]]
        }).collect();
        Poly { w: plane::dot(p.n, pts[0]), pts, n: p.n, path: p.path.clone(), prim: p.prim, smooth: p.smooth, tag: 0 }
    }).collect();
    let tol = eps * 1e-3;
    let remainder = difference(ap, polys_of(b, &b.term, tol, origin), tol);
    !remainder.iter().any(|p| plane::norm(crate::solid::area_vector(&p.pts)) > tol * tol)
}

// -- the edges a view draws ---------------------------------------------------------------------

/// How many directions the material around a candidate edge is sampled in.  Twelve is enough to
/// tell a wedge from a slab and cheap enough to pay per candidate.
const RING: usize = 12;

/// **The edges of a term.**  Candidates are the seams inside each primitive and the meetings of
/// facets from different ones; each is cut where any other primitive's plane crosses it, and a
/// piece survives where the material around it forms a wedge.
pub fn edges(csg: &Csg, eps: f64) -> Vec<Edge> {
    let indices: Vec<_> = csg.prims.iter().map(RayIndex::new).collect();
    edges_indexed(csg, eps, &indices)
}

pub(crate) fn edges_indexed(csg: &Csg, eps: f64, indices: &[RayIndex]) -> Vec<Edge> {
    // Asked for only where an edge meets another primitive: deduping one primitive's planes is
    // quadratic in its facets, and a lone primitive's seams never need them.
    let planes: Vec<std::cell::OnceCell<Vec<([f64; 3], f64, Box3)>>> =
        csg.prims.iter().map(|_| std::cell::OnceCell::new()).collect();
    // Each candidate with the primitive it is a seam of, if it is one: a primitive is a closed
    // surface that does not cross itself, so none of its own facets cuts the inside of one of its
    // own seams, and testing them all is a check per facet per seam — quadratic in the facets,
    // two seconds of a 25 000-facet torus. A crossing of two primitives is cut by every plane.
    let mut cand: Vec<(Edge, Option<usize>)> = Vec::new();
    for (i, prim) in csg.prims.iter().enumerate() {
        let mut own = Vec::new();
        seams(prim, &mut own);
        cand.extend(own.into_iter().map(|e| (e, Some(i))));
        for (j, other) in csg.prims.iter().enumerate() {
            if j <= i || !prim.bbox.overlaps(&other.bbox) {
                continue;
            }
            let mut crossing = Vec::new();
            crossings(prim, other, &mut crossing);
            cand.extend(crossing.into_iter().map(|e| (e, None)));
        }
    }
    // A solid that is one primitive is bounded by all of it: each of its seams is on the
    // boundary, and the ring of classifications that asks is not needed.
    let lone = matches!(csg.term, crate::solid::Term::Prim(0)) && csg.prims.len() == 1;
    let mut out = Vec::new();
    for (e, seam_of) in cand {
        let mut cuts: Vec<f64> = vec![0.0, 1.0];
        let d = [e.b[0] - e.a[0], e.b[1] - e.a[1], e.b[2] - e.a[2]];
        let len = plane::norm(d);
        if len <= 0.0 {
            continue;
        }
        let eb = {
            let mut b = Box3::empty();
            b.add(e.a);
            b.add(e.b);
            b
        };
        for (j, other) in csg.prims.iter().enumerate() {
            if seam_of == Some(j) || !other.bbox.grown(eps).overlaps(&eb) {
                continue;
            }
            for (n, dd, pb) in planes[j].get_or_init(|| planes_of(other)) {
                if !pb.grown(eps).overlaps(&eb) {
                    continue;
                }
                let denom = plane::dot(*n, d);
                if denom.abs() < 1e-12 {
                    continue;
                }
                let t = (dd - plane::dot(*n, e.a)) / denom;
                if t > 1e-9 && t < 1.0 - 1e-9 {
                    cuts.push(t);
                }
            }
        }
        cuts.sort_by(|a, b| a.partial_cmp(b).unwrap());
        for w in cuts.windows(2) {
            if w[1] - w[0] < 1e-9 {
                continue;
            }
            let at = |t: f64| {
                [e.a[0] + t * d[0], e.a[1] + t * d[1], e.a[2] + t * d[2]]
            };
            let m = at((w[0] + w[1]) / 2.0);
            if !(lone && seam_of.is_some()) && !on_boundary(csg, indices, m, d, eps) {
                continue;
            }
            out.push(Edge { a: at(w[0]), b: at(w[1]), ..e.clone() });
        }
    }
    out
}

/// Is the material around this point a *wedge* — a corner of the solid — rather than a slab, a
/// solid block or empty space?  Sampled on a ring perpendicular to the edge, counting the
/// changes: two or more transitions is a surface passing through, none is interior or exterior.
fn on_boundary(csg: &Csg, indices: &[RayIndex], m: [f64; 3], along: [f64; 3], eps: f64) -> bool {
    let Some(w) = plane::unit(along) else { return false };
    let helper = if w[0].abs() < 0.9 { [1.0, 0.0, 0.0] } else { [0.0, 1.0, 0.0] };
    let Some(p) = plane::unit(plane::cross(w, helper)) else { return false };
    let q = plane::cross(w, p);
    let mut ins = [false; RING];
    for (k, slot) in ins.iter_mut().enumerate() {
        let a = std::f64::consts::TAU * k as f64 / RING as f64;
        let (s, c) = a.dsin_cos();
        *slot = csg.inside_indexed([
            m[0] + eps * (c * p[0] + s * q[0]),
            m[1] + eps * (c * p[1] + s * q[1]),
            m[2] + eps * (c * p[2] + s * q[2]),
        ], indices);
    }
    let changes = (0..RING).filter(|k| ins[*k] != ins[(k + 1) % RING]).count();
    changes >= 2
}

/// The seams inside one primitive: every segment two of its facets share.
fn seams(prim: &Prim, out: &mut Vec<Edge>) {
    let mut map: std::collections::BTreeMap<([i64; 3], [i64; 3]), (usize, usize)> =
        std::collections::BTreeMap::new();
    let scale = {
        let b = &prim.bbox;
        (0..3).fold(1.0f64, |m, i| m.max((b.hi[i] - b.lo[i]).abs())).max(1.0)
    };
    let key = |p: [f64; 3]| {
        let g = scale * 1e-9;
        [(p[0] / g).round() as i64, (p[1] / g).round() as i64, (p[2] / g).round() as i64]
    };
    for (fi, f) in prim.facets.iter().enumerate() {
        for i in 0..f.pts.len() {
            let a = f.pts[i];
            let b = f.pts[(i + 1) % f.pts.len()];
            let (ka, kb) = (key(a), key(b));
            let k = if ka <= kb { (ka, kb) } else { (kb, ka) };
            match map.get_mut(&k) {
                Some(slot) => slot.1 = fi,
                None => {
                    map.insert(k, (fi, usize::MAX));
                }
            }
        }
    }
    for (k, (f1, f2)) in map {
        if f2 == usize::MAX {
            continue;
        }
        let g = scale * 1e-9;
        let a = [k.0[0] as f64 * g, k.0[1] as f64 * g, k.0[2] as f64 * g];
        let b = [k.1[0] as f64 * g, k.1[1] as f64 * g, k.1[2] as f64 * g];
        let (na, nb) = (prim.facets[f1].n, prim.facets[f2].n);
        // a seam whose two facets face the same way is a chord of a tessellation and not a
        // corner, and it is smooth if *either* facet says its sweep was one — within one face of an
        // exact B-rep, whose faces meet at its edges
        let flat = plane::dot(na, nb) > 0.999_999;
        let one_face = !prim.exact || prim.facets[f1].face == prim.facets[f2].face;
        let smooth = flat || (prim.facets[f1].smooth && prim.facets[f2].smooth && one_face);
        out.push(Edge {
            a,
            b,
            na,
            nb,
            smooth,
            path: path_of(prim, &prim.facets[f1]),
        });
    }
}

/// Where two primitives' facets meet: the intersection line of their planes, clipped to both.
fn crossings(p: &Prim, q: &Prim, out: &mut Vec<Edge>) {
    for f in &p.facets {
        let fb = f.bbox();
        if !q.bbox.overlaps(&fb) {
            continue;
        }
        for g in &q.facets {
            let gb = g.bbox();
            if !gb.overlaps(&fb) {
                continue;
            }
            let dir = plane::cross(f.n, g.n);
            let Some(dir) = plane::unit(dir) else { continue };
            // a point on both planes: solve the 3×3 with the direction as the third row
            let Some(x0) = meet(f.n, f.offset(), g.n, g.offset(), dir) else { continue };
            let Some((t0, t1)) = clip(&f.pts, f.n, x0, dir) else { continue };
            let Some((s0, s1)) = clip(&g.pts, g.n, x0, dir) else { continue };
            let (lo, hi) = (t0.max(s0), t1.min(s1));
            if hi - lo < 1e-9 {
                continue;
            }
            let at = |t: f64| [x0[0] + t * dir[0], x0[1] + t * dir[1], x0[2] + t * dir[2]];
            out.push(Edge {
                a: at(lo),
                b: at(hi),
                na: f.n,
                nb: g.n,
                smooth: false,
                path: path_of(p, f),
            });
        }
    }
}

/// A point on both planes, nearest the origin along the shared direction.
fn meet(n1: [f64; 3], d1: f64, n2: [f64; 3], d2: f64, dir: [f64; 3]) -> Option<[f64; 3]> {
    let m = [n1, n2, dir];
    let det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
        - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
        + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    if det.abs() < 1e-12 {
        return None;
    }
    let rhs = [d1, d2, 0.0];
    let mut x = [0.0; 3];
    for c in 0..3 {
        let mut a = m;
        for r in 0..3 {
            a[r][c] = rhs[r];
        }
        let d = a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
            - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
            + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);
        x[c] = d / det;
    }
    Some(x)
}

/// The stretch of the line `x0 + t·dir` that lies inside a convex facet.
fn clip(pts: &[[f64; 3]], n: [f64; 3], x0: [f64; 3], dir: [f64; 3]) -> Option<(f64, f64)> {
    let (mut lo, mut hi) = (f64::NEG_INFINITY, f64::INFINITY);
    for i in 0..pts.len() {
        let a = pts[i];
        let b = pts[(i + 1) % pts.len()];
        let e = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
        // inward normal of this side, in the facet's plane
        let ni = plane::cross(n, e);
        let denom = plane::dot(ni, dir);
        let num = plane::dot(ni, [x0[0] - a[0], x0[1] - a[1], x0[2] - a[2]]);
        if denom.abs() < 1e-15 {
            if num < -1e-12 * plane::norm(ni).max(1.0) {
                return None;
            }
            continue;
        }
        let t = -num / denom;
        if denom > 0.0 {
            lo = lo.max(t);
        } else {
            hi = hi.min(t);
        }
    }
    (hi > lo).then_some((lo, hi))
}
