//! The regular (weighted Delaunay) triangulation of weighted points, built incrementally by
//! Bowyer–Watson inside an enclosing tetrahedron far beyond the points' ball: every inserted
//! point is interior, so no insertion meets the hull and every tetrahedron is finite.
//!
//! A point is located by a remembering stochastic walk. Its conflict region is the connected
//! set of tetrahedra whose orthosphere it has negative power against; a point in conflict with
//! none is **hidden** (redundant: it has no cell in the power diagram), and a vertex whose every
//! tetrahedron is in the region becomes hidden when the point goes in. Where the region's
//! boundary would make a flat or inverted tetrahedron with the point (a tie: the point on an
//! orthosphere and in the plane of a boundary face), the region takes the tetrahedron beyond
//! that face too, so every new tetrahedron is positively oriented. Ties are otherwise resolved
//! arbitrarily: any regular triangulation of degenerate input is a correct one, and `check`
//! judges it by the empty-orthosphere property, not by comparison with another triangulation.
use super::predicates::{orient,power,Weighted};

pub const NONE: u32 = u32::MAX;

/// A tetrahedron: its vertices, positively oriented, and the tetrahedron across the face
/// opposite each (`NONE` outside the enclosing tetrahedron).
#[derive(Clone,Copy,Debug)]
pub struct Tet { pub v: [u32;4],pub n: [u32;4] }

/// What became of an inserted point: a vertex, or hidden.
#[derive(Clone,Copy,Debug,PartialEq,Eq)]
pub enum Inserted { Vertex(u32),Hidden(u32) }

/// The number of enclosing vertices, which come first and are never hidden.
pub const ENCLOSING: u32 = 4;

pub struct Regular {
    points: Vec<Weighted>,
    hidden: Vec<bool>,
    tets: Vec<Tet>,
    free: Vec<u32>,
    /// Per tetrahedron, for insertion `k`: `3k` in its conflict region, `3k + 1` tested strictly
    /// outside it, `3k + 2` tested and tied (the point on the orthosphere).
    stamp: Vec<u64>,
    /// Per vertex: the last insertion whose region's boundary it is on.
    kept: Vec<u64>,
    /// Per vertex: a tetrahedron that had it as a vertex when made (current for every unhidden
    /// vertex: a tetrahedron is only freed when a new one on the same vertices replaces it).
    incident: Vec<u32>,
    /// The tetrahedra the last insertion made.
    created: Vec<u32>,
    insertion: u64,
    last: u32,
    walk: u64,
    /// Tetrahedra added to a conflict region past a tie, over all insertions.
    pub ties: usize,
    /// Buffers an insertion reuses: the conflict region, the search stack and its boundary faces.
    cavity: Vec<u32>,
    stack: Vec<u32>,
    boundary: Vec<(u32,usize)>,
    /// Open-addressed table of those faces, by edge: (edge key, generation, tetrahedron, slot).
    open: Vec<(u64,u64,u32,u8)>,
}

impl Regular {
    /// An empty triangulation for points within `radius` of `center`: an enclosing tetrahedron
    /// whose insphere has sixteen times that radius.
    pub fn new(center: [f64;3],radius: f64) -> Self {
        let r = 3.*16.*radius.max(f64::MIN_POSITIVE); // circumradius = 3 × inradius
        let s = r/3f64.sqrt();
        let corner = |d: [f64;3]| Weighted {p:std::array::from_fn(|k| center[k]+s*d[k]),w:0.};
        let mut v = [[1.,1.,1.],[1.,-1.,-1.],[-1.,1.,-1.],[-1.,-1.,1.]].map(corner);
        if orient(v[0].p,v[1].p,v[2].p,v[3].p) < 0 { v.swap(0,1); }
        Self {points:v.to_vec(),hidden:vec![false;4],tets:vec![Tet {v:[0,1,2,3],n:[NONE;4]}],
            free:Vec::new(),stamp:vec![0],kept:vec![0;4],incident:vec![0;4],created:Vec::new(),
            insertion:0,last:0,walk:0x9e3779b97f4a7c15,ties:0,cavity:Vec::new(),stack:Vec::new(),
            boundary:Vec::new(),open:Vec::new()}
    }

    pub fn points(&self) -> &[Weighted] { &self.points }
    pub fn is_hidden(&self,v: u32) -> bool { self.hidden[v as usize] }
    pub fn tet(&self,t: u32) -> Tet { self.tets[t as usize] }
    /// The live tetrahedra's indices.
    pub fn tets(&self) -> impl Iterator<Item = u32> + '_ {
        (0..self.tets.len() as u32).filter(|&t| self.alive(t))
    }
    /// Whether a tetrahedron index is in use (a freed one has `NONE` for its first vertex).
    pub fn alive(&self,t: u32) -> bool { self.tets[t as usize].v[0] != NONE }
    /// Whether a tetrahedron has an enclosing vertex.
    pub fn touches_enclosure(&self,t: u32) -> bool {
        self.tets[t as usize].v.iter().any(|&v| v < ENCLOSING)
    }

    fn corners(&self,t: u32) -> [Weighted;4] {
        self.tets[t as usize].v.map(|v| self.points[v as usize])
    }

    /// Whether `q` is in conflict with tetrahedron `t`.
    fn conflict(&self,t: u32,q: Weighted) -> bool { self.side(t,q) > 0 }
    /// `power` of `q` against tetrahedron `t`: +1 in conflict, 0 tied, −1 strictly outside.
    fn side(&self,t: u32,q: Weighted) -> i8 {
        let [a,b,c,d] = self.corners(t);
        power(a,b,c,d,q)
    }

    /// Start the next walk at `hint`, when it is a live tetrahedron.
    fn hint(&mut self,hint: Option<u32>) {
        if let Some(h) = hint.filter(|&h| (h as usize) < self.tets.len() && self.alive(h)) {
            self.last = h;
        }
    }

    /// The tetrahedron containing `p`, by a remembering stochastic walk from the last one made.
    pub fn locate(&mut self,p: [f64;3]) -> Result<u32,String> {
        let mut t = self.last;
        if !self.alive(t) { t = self.tets().next().ok_or("an empty triangulation")?; }
        let limit = 64+4*self.tets.len();
        for _ in 0..limit {
            self.walk ^= self.walk << 13; self.walk ^= self.walk >> 7; self.walk ^= self.walk << 17;
            let start = (self.walk % 4) as usize;
            let tet = self.tets[t as usize];
            let corners = tet.v.map(|v| self.points[v as usize].p);
            let mut next = None;
            for k in 0..4 {
                let i = (start+k)%4;
                let mut q = corners;
                q[i] = p;
                if orient(q[0],q[1],q[2],q[3]) < 0 { next = Some(tet.n[i]); break; }
            }
            match next {
                None => return Ok(t),
                Some(NONE) => return Err(format!("{p:?} lies outside the enclosing tetrahedron")),
                Some(n) => t = n,
            }
        }
        Err("the walk did not reach the point's tetrahedron".into())
    }

    fn allocate(&mut self,tet: Tet) -> u32 {
        let t = if let Some(t) = self.free.pop() {
            self.tets[t as usize] = tet;
            t
        } else {
            self.tets.push(tet);
            self.stamp.push(0);
            (self.tets.len()-1) as u32
        };
        for v in tet.v { self.incident[v as usize] = t; }
        t
    }

    /// Insert a weighted point, walking from the last tetrahedron made.
    pub fn insert(&mut self,p: [f64;3],w: f64) -> Result<Inserted,String> {
        self.insert_near(p,w,None)
    }

    /// Insert a weighted point, walking from `hint` (a live tetrahedron near it) when given. On
    /// return, `created` and `removed` say what the insertion changed.
    pub fn insert_near(&mut self,p: [f64;3],w: f64,hint: Option<u32>) -> Result<Inserted,String> {
        if !p.iter().chain([&w]).all(|x| x.is_finite()) {
            return Err("a point must be finite".into());
        }
        let q = Weighted {p,w};
        self.hint(hint);
        let start = self.locate(p)?;
        let id = self.points.len() as u32;
        self.points.push(q);
        self.hidden.push(false);
        self.kept.push(0);
        self.incident.push(NONE);
        self.created.clear();
        self.cavity.clear();
        if !self.conflict(start,q) {
            self.hidden[id as usize] = true;
            return Ok(Inserted::Hidden(id));
        }
        self.insertion += 1;
        let (inside,outside) = (3*self.insertion,3*self.insertion+1);
        let mut cavity = std::mem::take(&mut self.cavity);
        let mut stack = std::mem::take(&mut self.stack);
        let mut boundary = std::mem::take(&mut self.boundary);
        let mut open = std::mem::take(&mut self.open);
        stack.clear(); boundary.clear();
        self.region(start,q,&mut cavity,&mut stack);
        // Every boundary face must make a positively oriented tetrahedron with the point. Across a
        // face whose far tetrahedron is strictly outside that holds already: both orthospheres are
        // orthogonal to the face's three weighted vertices, so their radical plane is the face's
        // plane, and a point of negative power against the near one and positive against the far
        // one lies strictly on the near side. Only a tie, or a tetrahedron taken past one (whose
        // neighbours were never tested), needs the orientation asked.
        loop {
            boundary.clear();
            let mut grown = false;
            for k in 0..cavity.len() {
                let c = cavity[k];
                for i in 0..4 {
                    let n = self.tets[c as usize].n[i];
                    if n != NONE && self.stamp[n as usize] == inside { continue; }
                    if n != NONE && self.stamp[n as usize] == outside {
                        boundary.push((c,i));
                        continue;
                    }
                    let mut v = self.tets[c as usize].v.map(|v| self.points[v as usize].p);
                    v[i] = p;
                    if orient(v[0],v[1],v[2],v[3]) > 0 { boundary.push((c,i)); continue; }
                    if n == NONE { return Err(format!("{p:?} lies on the enclosing tetrahedron")); }
                    self.stamp[n as usize] = inside;
                    cavity.push(n);
                    self.ties += 1;
                    grown = true;
                }
            }
            if !grown { break; }
        }
        // Vertices of the region on no boundary face are engulfed: hidden by the new point.
        for &(c,i) in &boundary {
            for j in 0..4 {
                if j != i { self.kept[self.tets[c as usize].v[j] as usize] = self.insertion; }
            }
        }
        for &c in &cavity {
            for v in self.tets[c as usize].v {
                if v >= ENCLOSING && self.kept[v as usize] != self.insertion {
                    self.hidden[v as usize] = true;
                }
            }
        }
        // One new tetrahedron per boundary face, linked across the faces through the new point:
        // three open faces per new tetrahedron wait in the table for their partner, keyed by the
        // edge they share with the boundary; a slot from an earlier insertion is empty.
        let generation = self.insertion;
        let wanted = (4*3*boundary.len()).next_power_of_two().max(64);
        if open.len() < wanted { open = vec![(0,0,0,0);wanted]; }
        let mask = open.len()-1;
        let mut waiting = 0usize;
        let mut first = NONE;
        for &(c,i) in &boundary {
            let old = self.tets[c as usize];
            let mut v = old.v;
            v[i] = id;
            let beyond = old.n[i];
            let mut n = [NONE;4];
            n[i] = beyond;
            let t = self.allocate(Tet {v,n});
            self.created.push(t);
            if first == NONE { first = t; }
            if beyond != NONE {
                let slot = self.tets[beyond as usize].n.iter().position(|&x| x == c)
                    .ok_or("a neighbour does not point back across its face")?;
                self.tets[beyond as usize].n[slot] = t;
            }
            for j in 0..4 {
                if j == i { continue; }
                // The face opposite v[j] holds the new point and the two vertices other than i, j.
                let mut pair = [0u32;2];
                let mut m = 0;
                for k in 0..4 { if k != i && k != j { pair[m] = v[k]; m += 1; } }
                let key = ((pair[0].min(pair[1]) as u64) << 32) | pair[0].max(pair[1]) as u64;
                let mut h = (key.wrapping_mul(0x9e37_79b9_7f4a_7c15) >> 20) as usize & mask;
                loop {
                    let slot = open[h];
                    if slot.1 != generation {
                        open[h] = (key,generation,t,j as u8);
                        waiting += 1;
                        break;
                    }
                    if slot.0 == key {
                        // Paired: the slot is spent (a key recurs only between two tetrahedra).
                        open[h].0 = u64::MAX;
                        self.tets[t as usize].n[j] = slot.2;
                        self.tets[slot.2 as usize].n[slot.3 as usize] = t;
                        waiting -= 1;
                        break;
                    }
                    h = (h+1) & mask;
                }
            }
        }
        if waiting != 0 { return Err("the new tetrahedra do not close around the point".into()); }
        for &c in &cavity {
            self.tets[c as usize].v[0] = NONE;
            self.free.push(c);
        }
        self.cavity = cavity; self.stack = stack; self.boundary = boundary; self.open = open;
        self.last = first;
        Ok(Inserted::Vertex(id))
    }

    /// The tetrahedra the last insertion made (none when its point was hidden).
    pub fn created(&self) -> &[u32] { &self.created }
    /// The tetrahedra the last insertion freed, its conflict region: their indices may already
    /// be reused by `created`, and are only a record of what changed.
    pub fn removed(&self) -> &[u32] { &self.cavity }

    /// A live tetrahedron with `v` as a vertex, for an unhidden vertex.
    pub fn incident(&self,v: u32) -> Option<u32> {
        if self.hidden[v as usize] { return None; }
        let t = self.incident[v as usize];
        (t != NONE && self.alive(t) && self.tets[t as usize].v.contains(&v)).then_some(t)
    }

    /// The facet `(t, i)` seen from the other side: the tetrahedron across it and the index of
    /// the vertex opposite the facet there, or `None` on the enclosing tetrahedron's hull.
    pub fn mirror(&self,t: u32,i: usize) -> Option<(u32,usize)> {
        let n = self.tets[t as usize].n[i];
        if n == NONE { return None; }
        let j = self.tets[n as usize].n.iter().position(|&x| x == t)?;
        Some((n,j))
    }

    /// The orthocentre of tetrahedron `t` and the square of its orthoradius, in floating point:
    /// the centre and radius of the sphere orthogonal to its four weighted vertices (their
    /// circumsphere for zero weights). A reading for construction, never a predicate.
    pub fn orthosphere(&self,t: u32) -> ([f64;3],f64) {
        let [a,b,c,d] = self.corners(t);
        let from_a = |q: Weighted| [0,1,2].map(|k| q.p[k]-a.p[k]);
        let (u,v,x) = (from_a(b),from_a(c),from_a(d));
        let dot = |p: [f64;3],q: [f64;3]| p[0]*q[0]+p[1]*q[1]+p[2]*q[2];
        let cross = |p: [f64;3],q: [f64;3]|
            [p[1]*q[2]-p[2]*q[1],p[2]*q[0]-p[0]*q[2],p[0]*q[1]-p[1]*q[0]];
        let (ru,rv,rx) = (dot(u,u)-(b.w-a.w),dot(v,v)-(c.w-a.w),dot(x,x)-(d.w-a.w));
        let (vx,xu,uv) = (cross(v,x),cross(x,u),cross(u,v));
        let det = 2.*dot(u,vx);
        let o: [f64;3] = std::array::from_fn(|k| (ru*vx[k]+rv*xu[k]+rx*uv[k])/det);
        ([a.p[0]+o[0],a.p[1]+o[1],a.p[2]+o[2]],dot(o,o)-a.w)
    }

    /// The tetrahedra a point would take if inserted (its conflict region), leaving the
    /// triangulation as it is: empty when the point would be hidden.
    pub fn conflicts(&mut self,p: [f64;3],w: f64,hint: Option<u32>) -> Result<Vec<u32>,String> {
        let q = Weighted {p,w};
        self.hint(hint);
        let start = self.locate(p)?;
        if !self.conflict(start,q) { return Ok(Vec::new()); }
        self.insertion += 1;
        let (mut region,mut stack) = (Vec::new(),Vec::new());
        self.region(start,q,&mut region,&mut stack);
        Ok(region)
    }

    /// The conflict region of `q` from `start` (which must be in it) for the current insertion
    /// number, into `region`: every tetrahedron reached is stamped in, tied or strictly out.
    fn region(&mut self,start: u32,q: Weighted,region: &mut Vec<u32>,stack: &mut Vec<u32>) {
        let (inside,outside,tied) = (3*self.insertion,3*self.insertion+1,3*self.insertion+2);
        region.clear();
        stack.clear();
        region.push(start);
        stack.push(start);
        self.stamp[start as usize] = inside;
        while let Some(c) = stack.pop() {
            for i in 0..4 {
                let n = self.tets[c as usize].n[i];
                if n == NONE { continue; }
                let seen = self.stamp[n as usize];
                if seen == inside || seen == outside || seen == tied { continue; }
                match self.side(n,q) {
                    1 => { self.stamp[n as usize] = inside; region.push(n); stack.push(n); }
                    0 => self.stamp[n as usize] = tied,
                    _ => self.stamp[n as usize] = outside,
                }
            }
        }
    }

    /// Check the triangulation against its definition, by brute force: every tetrahedron is
    /// positively oriented; neighbours share exactly their face and point back; only the
    /// enclosing tetrahedron's faces are open; no unhidden vertex has negative power against any
    /// tetrahedron's orthosphere; every unhidden vertex is a vertex of some tetrahedron; and
    /// every hidden point conflicts with no tetrahedron. Quadratic, for tests.
    pub fn check(&self) -> Result<(),String> {
        let mut used = vec![false;self.points.len()];
        for t in self.tets() {
            let tet = self.tets[t as usize];
            let [a,b,c,d] = self.corners(t);
            if super::predicates::orient_exact(a.p,b.p,c.p,d.p) <= 0 {
                return Err(format!("tetrahedron {t} is not positively oriented"));
            }
            for i in 0..4 {
                used[tet.v[i] as usize] = true;
                let n = tet.n[i];
                let face: Vec<u32> = (0..4).filter(|&k| k != i).map(|k| tet.v[k]).collect();
                if n == NONE {
                    if face.iter().any(|&v| v >= ENCLOSING) {
                        return Err(format!("tetrahedron {t} has an open inner face"));
                    }
                    continue;
                }
                if !self.alive(n) { return Err(format!("tetrahedron {t} neighbours a dead one")); }
                let other = self.tets[n as usize];
                let back = other.n.iter().position(|&x| x == t)
                    .ok_or(format!("tetrahedron {n} does not point back to {t}"))?;
                let mut theirs: Vec<u32> =
                    (0..4).filter(|&k| k != back).map(|k| other.v[k]).collect();
                let mut ours = face.clone();
                theirs.sort_unstable(); ours.sort_unstable();
                if theirs != ours { return Err(format!("tetrahedra {t} and {n} share no face")); }
            }
            for (v,&q) in self.points.iter().enumerate() {
                if self.hidden[v] || tet.v.contains(&(v as u32)) { continue; }
                if power(a,b,c,d,q) > 0 {
                    return Err(format!("vertex {v} conflicts with tetrahedron {t}"));
                }
            }
        }
        for v in 0..self.points.len() {
            if self.hidden[v] {
                let q = self.points[v];
                let conflicting = |t: &u32| {
                    let [a,b,c,d] = self.corners(*t);
                    power(a,b,c,d,q) > 0
                };
                if let Some(t) = self.tets().find(conflicting) {
                    return Err(format!("hidden point {v} conflicts with tetrahedron {t}"));
                }
            } else if !used[v] {
                return Err(format!("vertex {v} is in no tetrahedron"));
            }
        }
        Ok(())
    }
}

/// An insertion order that keeps each point near the last: biased randomized insertion
/// (Amenta, Choi and Rote) with every round in Hilbert-curve order. Rounds of doubling size keep
/// the randomized bound on work; within a round the curve keeps the walk from each point to the
/// next a few tetrahedra long. Deterministic for a given input.
pub fn spatial_order(points: &[[f64;3]]) -> Vec<usize> {
    if points.is_empty() { return Vec::new(); }
    let (mut lo,mut hi) = ([f64::INFINITY;3],[f64::NEG_INFINITY;3]);
    for p in points { for k in 0..3 { lo[k] = lo[k].min(p[k]); hi[k] = hi[k].max(p[k]); } }
    let span = (0..3).map(|k| hi[k]-lo[k]).fold(0.,f64::max).max(f64::MIN_POSITIVE);
    let key = |p: &[f64;3]|
        hilbert(std::array::from_fn(|k| (((p[k]-lo[k])/span)*65535.).clamp(0.,65535.) as u32));
    // Rounds: each point's round is set by a deterministic hash, the last round holding about half.
    let mut rounds: Vec<Vec<usize>> = Vec::new();
    for (i,_) in points.iter().enumerate() {
        let mut h = (i as u64).wrapping_mul(0x9e3779b97f4a7c15);
        h ^= h >> 31;
        let r = (h.trailing_zeros() as usize).min(24);
        if rounds.len() <= r { rounds.resize(r+1,Vec::new()); }
        rounds[r].push(i);
    }
    rounds.reverse();
    let mut order = Vec::with_capacity(points.len());
    for mut round in rounds {
        round.sort_by_cached_key(|&i| key(&points[i]));
        order.extend(round);
    }
    order
}

/// The Hilbert index of a point on a 2¹⁶ grid (Skilling, "Programming the Hilbert curve", 2004).
fn hilbert(mut x: [u32;3]) -> u64 {
    const BITS: u32 = 16;
    let m = 1u32 << (BITS-1);
    // Inverse undo of excess work.
    let mut q = m;
    while q > 1 {
        let p = q-1;
        for i in 0..3 {
            if x[i] & q != 0 { x[0] ^= p; } else { let t = (x[0]^x[i]) & p; x[0] ^= t; x[i] ^= t; }
        }
        q >>= 1;
    }
    // Gray encode.
    for i in 1..3 { x[i] ^= x[i-1]; }
    let mut t = 0;
    let mut q = m;
    while q > 1 { if x[2] & q != 0 { t ^= q-1; } q >>= 1; }
    for v in &mut x { *v ^= t; }
    // Interleave the transposed bits, most significant first.
    let mut index = 0u64;
    for b in (0..BITS).rev() { for v in &x { index = (index << 1) | ((v >> b) & 1) as u64; } }
    index
}
