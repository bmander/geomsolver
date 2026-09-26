//! Delaunay refinement of an implicit domain's boundary with protected features (Cheng, Dey and
//! Ramos; Dey and Levine's protecting balls; the scheme CGAL's Mesh_3 implements), over the
//! regular triangulation (`regular`). The domain need only say which side of its boundary a point
//! is on (`Domain`); the features are polylines the caller supplies (docs/field-meshing.md, F4).
//!
//! 1. **Protection** (`protect`). Each feature curve is sampled at a spacing no larger than
//!    `edge_size` and every sample becomes a weighted point, a ball of radius `BALL` × its spacing:
//!    consecutive balls on a curve overlap, and where two balls neither consecutive on one curve
//!    nor sharing a corner would meet, the curves involved are sampled more finely there (an eighth
//!    of `edge_size` beside a shared corner, down to `edge_size / LEAST` elsewhere); past that only
//!    balls near a corner their curves share (curves meeting tangentially there) may meet. The
//!    balls' centres stay edges of the regular triangulation, so the curve survives in the mesh.
//! 2. **Seeding** (`seed`). Rays from the centre, in evenly spread directions, are sampled; every
//!    change of side is bisected to a boundary point and inserted (unless inside a ball).
//! 3. **Restricted facets** (`crossing`). A facet lies on the boundary when the domain's side
//!    differs at its two tetrahedra's orthocentres, the ends of its dual edge in the power diagram;
//!    its surface centre is where that dual edge crosses the boundary, found by bisection.
//! 4. **Refinement.** Facets breaking a criterion (its surface ball larger than `facet_size`, its
//!    circumcentre farther than `facet_distance` from its surface centre, an angle below
//!    `facet_angle`) are refined worst first by inserting their surface centre, unless that lies
//!    in a protecting ball, which is counted and left; the facets an insertion makes are judged.
//! 5. **Manifold** (`manifold`). Where the extracted boundary has an edge not used twice, or a
//!    vertex whose triangles form more than one fan, the largest facet there is refined, until
//!    none remain.
//!
//! `Progressive` runs the five a step at a time (`progressive`). Outside the bounding sphere is
//! outside the domain, as in Mesh_3.
use super::predicates::orient;
use super::regular::{Regular,Inserted,NONE,ENCLOSING};
use crate::space::{sub,dot,distance_squared as dist2,circumcentre};
use std::collections::{BinaryHeap,HashMap};

mod crossing;
mod manifold;
mod progressive;
mod protect;
mod seed;
mod trace;

pub use progressive::{Progress,Progressive,Stage};
use protect::Place;

type P = [f64;3];

/// The domain a refinement meshes: `{p : value(p) < 0}` within the bounding ball. A plain
/// `FnMut(P) -> f64` is one that answers only by sign; a domain with a value and a gradient says
/// so by `readings`, and its crossings are then placed by Newton rather than by bisection
/// (`Refiner::newton_crossing`).
pub trait Domain {
    /// A value negative inside the domain and not negative outside it: only its sign is read.
    fn value(&mut self,p: P) -> f64;
    /// Whether `reading` may be asked, and how far its signs are to be trusted.
    fn readings(&self) -> Readings { Readings::Absent }
    /// The value at `p` and its gradient, the value with `value`'s sign. `continues` says `p`
    /// lies on the segment of the crossing just read, so a domain whose readings are searches may
    /// start from the last one's answer. Asked only when `readings` is not `Absent`.
    fn reading(&mut self,p: P,continues: bool) -> (f64,P) { let _ = continues; (self.value(p),[0.;3]) }
}

/// What a domain's readings are worth to a refinement.
#[derive(Clone,Copy,Debug,PartialEq,Eq)]
pub enum Readings {
    /// None: crossings are bisected on `value`'s sign.
    Absent,
    /// Readings whose sign may disagree with `value`'s near the boundary (a reading continued
    /// from the last may have followed another branch): the bracket Newton closes is checked by
    /// `value` a tolerance outside each end, two queries a crossing.
    Checked,
    /// Readings whose sign is `value`'s own at every point, and that never continue a search
    /// along another branch: no bracket needs checking.
    Agreeing,
}

impl<F: FnMut(P) -> f64> Domain for F {
    fn value(&mut self,p: P) -> f64 { self(p) }
}

/// A domain made of two functions, its sign and its reading (`Domain::reading`): for a caller
/// with closures in hand rather than a type of its own.
pub struct WithReadings<S,R> { pub value: S,pub reading: R,pub readings: Readings }

impl<S: FnMut(P) -> f64,R: FnMut(P,bool) -> (f64,P)> Domain for WithReadings<S,R> {
    fn value(&mut self,p: P) -> f64 { (self.value)(p) }
    fn readings(&self) -> Readings { self.readings }
    fn reading(&mut self,p: P,continues: bool) -> (f64,P) { (self.reading)(p,continues) }
}

/// What a mesh must satisfy, in the domain's units but the angle.
#[derive(Clone,Debug)]
pub struct Criteria {
    /// The largest radius a facet's surface ball may have.
    pub facet_size: f64,
    /// The largest distance from a facet's circumcentre to its surface centre.
    pub facet_distance: f64,
    /// The smallest angle a facet may have, in degrees.
    pub facet_angle: f64,
    /// The largest spacing of protecting balls along a feature curve.
    pub edge_size: f64,
    /// Where a crossing is placed: bisection stops once the bracket is this short.
    pub bisection: f64,
    /// Refinement refuses past this many inserted points.
    pub max_points: usize,
    /// The largest angle, in degrees, between the surface normals at a facet's vertices; 0 asks
    /// nothing of them. Where the domain gives readings (`Domain::readings`) this is what
    /// makes the mesh follow the local feature size: the two walls of a thin feature have opposed
    /// normals, so a facet reaching across it is refined until facets fit between them, and on a
    /// curved face a facet is held to a fraction of its radius. A facet with a protecting ball for a
    /// vertex is exempt — that vertex is on a sharp edge, where no one normal is the surface's.
    pub normal_angle: f64,
}

/// What the refinement did.
#[derive(Clone,Debug,Default)]
pub struct Report {
    pub queries: usize,
    pub balls: usize,
    pub seeds: usize,
    pub inserted: usize,
    /// Refinement points left because they fell inside a protecting ball.
    pub in_balls: usize,
    /// Facets left because their surface ball was too small to refine (see `Refiner::judge`).
    pub too_small: usize,
    pub manifold_rounds: usize,
    /// Facets with a protecting ball for a vertex standing off the surface by more than ten times
    /// `facet_distance` in the mesh returned (the balls shrink and the mesh is rebuilt until none
    /// do, or the rebuilds run out).
    pub coarse: usize,
    /// Refinement points in conflict with neither tetrahedron of their facet, left.
    pub off_dual: usize,
    /// Where the queries went: orthocentres' sides, crossings found (each bisected or by Newton),
    /// and the sign queries of bisection.
    pub orthocentre_queries: usize,
    pub crossings: usize,
    pub bisection_queries: usize,
    /// Crossings found by Newton on readings, those that fell back to bisection, and readings made.
    pub newton: usize,
    pub fallbacks: usize,
    pub readings: usize,
    /// Times the protecting balls were refined and the triangulation rebuilt.
    pub rebuilds: usize,
}

/// A closed, oriented boundary mesh: vertices and outward-facing triangles.
#[derive(Clone,Debug,Default)]
pub struct Mesh { pub vertices: Vec<P>,pub triangles: Vec<[u32;3]>,pub report: Report }

/// A facet waiting to be refined: how bad it was, where it is and what it is made of (to
/// recognise a stale entry, once its tetrahedron is gone or remade).
#[derive(PartialEq)]
struct Bad { badness: f64,t: u32,i: usize,key: [u32;3] }
impl Eq for Bad {}
impl PartialOrd for Bad { fn partial_cmp(&self,o: &Self) -> Option<std::cmp::Ordering> { Some(self.cmp(o)) } }
impl Ord for Bad { fn cmp(&self,o: &Self) -> std::cmp::Ordering { self.badness.total_cmp(&o.badness) } }

struct Refiner<'a> {
    domain: Box<dyn Domain + 'a>,
    centre: P,
    radius: f64,
    criteria: Criteria,
    tri: Regular,
    /// Per tetrahedron index: the domain's side at its orthocentre, 0 when not yet read or since
    /// remade.
    sign: Vec<i8>,
    balls: Vec<(P,f64)>,
    /// The curves each ball stands on.
    owners: Vec<Vec<Place>>,
    /// Every unweighted point inserted (seeds and refinement), to carry into a rebuild.
    kept: Vec<P>,
    // The four maps below are only ever looked up and filled, never iterated, so their order
    // cannot reach the mesh (the determinism rule is about iteration).
    /// The ball each protecting vertex is.
    ball_of: HashMap<u32,usize>,
    /// Every side read, by the point's bits, carried across rebuilds.
    memo: HashMap<[u64;3],i8>,
    /// The surface normal at each vertex asked about, by its bits (`normal`).
    normals: HashMap<[u64;3],Option<P>>,
    /// Every crossing found, by its segment's ends and the first end's side, carried across
    /// rebuilds with the sides: a rebuild inserts the same points, so most of its dual segments
    /// are the last build's to the bit, and a crossing by readings costs the domain a search each.
    crossings: HashMap<[u64;7],P>,
    /// Balls a refinement point needed to enter, since the last rebuild.
    blocking: Vec<usize>,
    /// Balls by grid cell, for the in-a-ball test: looked up by cell, never iterated.
    grid: HashMap<[i64;3],Vec<usize>>,
    cell: f64,
    queue: BinaryHeap<Bad>,
    report: Report,
}

impl Refiner<'_> {
    /// A ball `p` lies in, with the refinement margin.
    fn ball_at(&self,p: P) -> Option<usize> { self.ball_within(p,self.margin()) }

    /// A ball `p` lies within `margin` of.
    fn ball_within(&self,p: P,margin: f64) -> Option<usize> {
        let c = self.cell_of(p);
        let reach = (margin/self.cell).ceil().max(1.) as i64;
        for dx in -reach..=reach { for dy in -reach..=reach { for dz in -reach..=reach {
            if let Some(list) = self.grid.get(&[c[0]+dx,c[1]+dy,c[2]+dz]) {
                let inside = |k: &&usize| {
                    let reach = self.balls[**k].1+margin;
                    dist2(p,self.balls[**k].0) < reach*reach
                };
                if let Some(&k) = list.iter().find(inside) { return Some(k); }
            }
        } } }
        None
    }
    fn cell_of(&self,p: P) -> [i64;3] { p.map(|x| (x/self.cell).floor() as i64) }
    /// How near two refinement points, or a point and a ball, may come: a twentieth of the
    /// feature spacing. Refinement terminates because every point it inserts is a positive
    /// distance from those already there; a surface ball whose power radius is below this is
    /// one whose centre all but coincides with a vertex (a point left on a ball's boundary makes
    /// such facets), and refining it inserts that vertex again, forever.
    fn margin(&self) -> f64 { 0.05*self.criteria.edge_size }

    /// The facet `(t, i)` if restricted: its sorted vertices and its surface centre.
    fn restricted(&mut self,t: u32,i: usize) -> Option<([u32;3],P)> {
        if !self.tri.alive(t) { return None; }
        let n = self.tri.tet(t).n[i];
        if n == NONE { return None; }
        let (st,sn) = (self.tet_side(t),self.tet_side(n));
        if st == sn { return None; }
        let face = facet_vertices(self.tri.tet(t).v,i);
        if face.iter().any(|&v| v < ENCLOSING) { return None; }
        let (ot,_) = self.tri.orthosphere(t);
        let (on,_) = self.tri.orthosphere(n);
        // A tetrahedron flat in floating point has no finite orthocentre to bisect toward.
        if !ot.iter().chain(&on).all(|x| x.is_finite()) { return None; }
        let centre = self.dual_crossing(face,ot,st,on);
        let mut key = face;
        key.sort_unstable();
        Some((key,centre))
    }

    /// The surface normal at a vertex, from the domain's gradient there, made once: `None` without
    /// readings or where the gradient vanishes.
    fn normal(&mut self,p: P) -> Option<P> {
        let key = p.map(f64::to_bits);
        if let Some(&n) = self.normals.get(&key) { return n; }
        let n = (self.domain.readings() != Readings::Absent).then(|| self.domain.reading(p,false).1).and_then(|g| {
            let l = dot(g,g).sqrt();
            (l > 0. && l.is_finite()).then(|| g.map(|x| x/l))
        });
        self.normals.insert(key,n);
        n
    }

    /// How badly a restricted facet's vertex normals disagree (above 1 is bad): 1 at
    /// `normal_angle` between the two most different, 2 at twice it; 0 for a facet exempt.
    fn normal_badness(&mut self,key: [u32;3]) -> f64 {
        let limit = self.criteria.normal_angle;
        if !(limit > 0.) || self.domain.readings() == Readings::Absent { return 0.; }
        let q = key.map(|v| self.tri.points()[v as usize]);
        if q.iter().any(|w| w.w > 0.) { return 0.; }
        let mut ns = Vec::with_capacity(3);
        for w in q { match self.normal(w.p) { Some(n) => ns.push(n), None => return 0. } }
        let least = [(0,1),(1,2),(2,0)].iter().map(|&(i,j)| dot(ns[i],ns[j])).fold(1f64,f64::min);
        least.clamp(-1.,1.).acos().to_degrees()/limit
    }

    /// How badly a restricted facet breaks the criteria (above 1 is bad).
    fn badness(&mut self,key: [u32;3],centre: P) -> f64 {
        let turned = self.normal_badness(key);
        self.shape_badness(key,centre).max(turned)
    }

    fn shape_badness(&self,key: [u32;3],centre: P) -> f64 {
        let q = key.map(|v| self.tri.points()[v as usize]);
        let [a,b,c] = q.map(|w| w.p);
        let size2 = (dist2(centre,a)-q[0].w).max(0.);
        let mut bad = size2/(self.criteria.facet_size*self.criteria.facet_size);
        if let Some(cc) = circumcentre(a,b,c) {
            bad = bad.max(dist2(cc,centre).sqrt()/self.criteria.facet_distance);
            let angle = |p: P,q: P,r: P| {
                let (x,y) = (sub(q,p),sub(r,p));
                (dot(x,y)/(dot(x,x)*dot(y,y)).sqrt()).clamp(-1.,1.).acos().to_degrees()
            };
            let least = angle(a,b,c).min(angle(b,c,a)).min(angle(c,a,b));
            if least < self.criteria.facet_angle { bad = bad.max(1.+(self.criteria.facet_angle-least)/self.criteria.facet_angle); }
        } else {
            bad = bad.max(2.);
        }
        bad
    }

    fn judge(&mut self,t: u32,i: usize) {
        if let Some((key,centre)) = self.restricted(t,i) {
            let badness = self.badness(key,centre);
            if badness <= 1. { return; }
            let a = self.tri.points()[key[0] as usize];
            if dist2(centre,a.p)-a.w < self.margin()*self.margin() { self.report.too_small += 1; return; }
            self.queue.push(Bad {badness,t,i,key});
        }
    }

    /// Insert an unweighted point, unless it lies in a protecting ball; judge what it made.
    fn insert(&mut self,p: P) -> Result<bool,String> { self.insert_judging(p,true) }

    /// Insert without judging the facets made: a bulk insertion (seeds, a rebuild's kept points)
    /// judges every facet once at the end, where facets judged along the way belong to
    /// tetrahedra that do not last, and each costs the domain a bisection.
    fn insert_judging(&mut self,p: P,judging: bool) -> Result<bool,String> {
        if let Some(b) = self.ball_at(p) { self.report.in_balls += 1; self.blocking.push(b); return Ok(false); }
        if self.report.inserted >= self.criteria.max_points {
            return Err(format!("refinement passed {} points",self.criteria.max_points));
        }
        match self.tri.insert(p,0.)? {
            Inserted::Hidden(_) => Ok(false),
            Inserted::Vertex(_) => {
                self.report.inserted += 1;
                self.kept.push(p);
                let created = self.tri.created().to_vec();
                self.trace_inserted(p,&created,judging);
                for &t in &created {
                    if self.sign.len() <= t as usize { self.sign.resize(t as usize+1,0); }
                    self.sign[t as usize] = 0;
                }
                if judging { for &t in &created { for i in 0..4 { self.judge(t,i); } } }
                Ok(true)
            }
        }
    }

    /// Refine the queued facets, worst first. An entry is re-read when popped: its tetrahedron
    /// may be gone, and even when the facet stands the tetrahedron across it may have been
    /// remade, moving its dual edge and so its surface centre; the stored centre is then no
    /// centre of an empty ball, and can fall on a vertex inserted since.
    fn refine(&mut self) -> Result<(),String> { self.refine_some(usize::MAX).map(|_| ()) }

    /// Refine at most `budget` queued facets: whether the queue is empty.
    fn refine_some(&mut self,budget: usize) -> Result<bool,String> {
        let mut spent = 0;
        while spent < budget {
            let Some(bad) = self.queue.pop() else { return Ok(true) };
            spent += 1;
            if !self.tri.alive(bad.t) { continue; }
            let mut face = facet_vertices(self.tri.tet(bad.t).v,bad.i);
            face.sort_unstable();
            if face != bad.key { continue; }
            let Some((key,centre)) = self.restricted(bad.t,bad.i) else { continue };
            let badness = self.badness(key,centre);
            if badness <= 1. { continue; }
            let a = self.tri.points()[key[0] as usize];
            if dist2(centre,a.p)-a.w < self.margin()*self.margin() { self.report.too_small += 1; continue; }
            self.trace_refining(key,badness,centre);
            // A point that would not remake either tetrahedron of the facet does not refine it
            // (and the facet, judged from both sides, would ask for the same point again).
            let n = self.tri.tet(bad.t).n[bad.i];
            let region = self.tri.conflicts(centre,0.,Some(bad.t))?;
            if !region.iter().any(|&t| t == bad.t || t == n) { self.report.off_dual += 1; continue; }
            self.insert(centre)?;
        }
        Ok(self.queue.is_empty())
    }

    /// The restricted facets, oriented from the inside out.
    fn extract(&mut self) -> Vec<([u32;3],u32,usize)> {
        let tets: Vec<u32> = self.tri.tets().collect();
        let mut out = Vec::new();
        for t in tets {
            for i in 0..4 {
                let n = self.tri.tet(t).n[i];
                if n == NONE || n < t { continue; }
                let (st,sn) = (self.tet_side(t),self.tet_side(n));
                if st == sn { continue; }
                let face = facet_vertices(self.tri.tet(t).v,i);
                if face.iter().any(|&v| v < ENCLOSING) { continue; }
                // Outward: away from the inside tetrahedron's opposite vertex.
                let (inner,j) = if st < 0 { (t,i) } else { (n,self.tri.mirror(t,i).map_or(0,|m| m.1)) };
                let tet = self.tri.tet(inner);
                let mut f = facet_vertices(tet.v,j);
                let p = |v: u32| self.tri.points()[v as usize].p;
                if orient(p(f[0]),p(f[1]),p(f[2]),p(tet.v[j])) > 0 { f.swap(1,2); }
                out.push((f,t,i));
            }
        }
        out
    }
}

/// The three vertices of the face opposite vertex `i`, in tetrahedron order.
fn facet_vertices(v: [u32;4],i: usize) -> [u32;3] {
    match i { 0 => [v[1],v[2],v[3]], 1 => [v[0],v[2],v[3]], 2 => [v[0],v[1],v[3]], _ => [v[0],v[1],v[2]] }
}

/// The facets' vertices, renumbered from the triangulation's, and the triangles over them.
fn collect(r: &Refiner,facets: &[([u32;3],u32,usize)]) -> (Vec<P>,Vec<[u32;3]>) {
    // looked up and filled in facet order, never iterated
    let mut index: HashMap<u32,u32> = HashMap::new();
    let mut vertices = Vec::new();
    let mut triangles = Vec::with_capacity(facets.len());
    for (f,_,_) in facets {
        triangles.push(f.map(|v| *index.entry(v).or_insert_with(|| {
            vertices.push(r.tri.points()[v as usize].p);
            (vertices.len()-1) as u32
        })));
    }
    (vertices,triangles)
}

/// Mesh the boundary of `{p : value(p) < 0}` within `radius` of `centre`, to the end.
pub fn mesh<'a>(domain: impl Domain + 'a,centre: P,radius: f64,curves: &[Vec<P>],criteria: &Criteria)
    -> Result<Mesh,String> {
    let mut run = Progressive::new(Box::new(domain),centre,radius,curves.to_vec(),criteria.clone());
    while !run.step(usize::MAX)? {}
    run.finished()
}
