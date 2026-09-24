//! Delaunay refinement of an implicit domain's boundary with protected features (Cheng, Dey and
//! Ramos; Dey and Levine's protecting balls; the scheme CGAL's Mesh_3 implements), over the
//! regular triangulation (`regular`). The domain need only say which side of its boundary a point
//! is on; the features are polylines the caller supplies (docs/field-meshing.md, F4).
//!
//! 1. **Protection.** Each feature curve is sampled at a spacing no larger than `edge_size` and
//!    every sample becomes a weighted point, a ball of radius 0.6 × its spacing: consecutive
//!    balls on a curve overlap, and where two balls neither consecutive on one curve nor sharing
//!    a corner would meet, the curves involved are sampled twice as finely, down to an eighth of
//!    `edge_size`; past that only balls near a corner their curves share (curves meeting
//!    tangentially there) may meet. The balls' centres stay edges of the regular triangulation,
//!    so the curve survives in the mesh.
//! 2. **Seeding.** Rays from the centre, in evenly spread directions, are sampled; every change
//!    of side is bisected to a boundary point and inserted (unless inside a ball).
//! 3. **Restricted facets.** A facet lies on the boundary when the domain's side differs at its
//!    two tetrahedra's orthocentres, the ends of its dual edge in the power diagram; its surface
//!    centre is where that dual edge crosses the boundary, found by bisection.
//! 4. **Refinement.** Facets breaking a criterion (its surface ball larger than `facet_size`, its
//!    circumcentre farther than `facet_distance` from its surface centre, an angle below
//!    `facet_angle`) are refined worst first by inserting their surface centre, unless that lies
//!    in a protecting ball, which is counted and left; the facets an insertion makes are judged.
//! 5. **Manifold.** Where the extracted boundary has an edge not used twice, or a vertex whose
//!    triangles form more than one fan, the largest facet there is refined, until none remain.
//!
//! Outside the bounding sphere is outside the domain, as in Mesh_3.
use super::predicates::orient;
use super::regular::{Regular,Inserted,NONE,ENCLOSING};
use std::collections::{BTreeMap,BinaryHeap,HashMap};

type P = [f64;3];

fn sub(a: P,b: P) -> P { [a[0]-b[0],a[1]-b[1],a[2]-b[2]] }
fn dot(a: P,b: P) -> f64 { a[0]*b[0]+a[1]*b[1]+a[2]*b[2] }
fn cross(a: P,b: P) -> P { [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]] }
fn dist2(a: P,b: P) -> f64 { let d = sub(a,b); dot(d,d) }
fn lerp(a: P,b: P,t: f64) -> P { [a[0]+t*(b[0]-a[0]),a[1]+t*(b[1]-a[1]),a[2]+t*(b[2]-a[2])] }

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
    domain: Box<dyn FnMut(P) -> f64 + 'a>,
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
    /// The ball each protecting vertex is.
    ball_of: HashMap<u32,usize>,
    /// Every side read, by the point's bits, carried across rebuilds.
    memo: HashMap<[u64;3],i8>,
    /// Balls a refinement point needed to enter, since the last rebuild.
    blocking: Vec<usize>,
    /// Balls by grid cell, for the in-a-ball test.
    grid: HashMap<[i64;3],Vec<usize>>,
    cell: f64,
    queue: BinaryHeap<Bad>,
    report: Report,
}

impl Refiner<'_> {
    fn side(&mut self,p: P) -> i8 {
        if dist2(p,self.centre) > self.radius*self.radius { return 1; }
        // A rebuild asks again at the orthocentres and bisections it asked before.
        let key = p.map(f64::to_bits);
        if let Some(&s) = self.memo.get(&key) { return s; }
        self.report.queries += 1;
        let s = if (self.domain)(p) < 0. { -1 } else { 1 };
        self.memo.insert(key,s);
        s
    }

    fn tet_side(&mut self,t: u32) -> i8 {
        if self.sign.len() <= t as usize { self.sign.resize(t as usize+1,0); }
        if self.sign[t as usize] == 0 {
            let (o,_) = self.tri.orthosphere(t);
            let s = if o.iter().all(|x| x.is_finite()) { self.side(o) } else { 1 };
            self.sign[t as usize] = s;
        }
        self.sign[t as usize]
    }

    /// The boundary crossing between `a` (on side `sa`) and `b` (on the other side).
    fn crossing(&mut self,mut a: P,sa: i8,mut b: P) -> P {
        while dist2(a,b) > self.criteria.bisection*self.criteria.bisection {
            let m = lerp(a,b,0.5);
            if self.side(m) == sa { a = m } else { b = m }
        }
        lerp(a,b,0.5)
    }

    /// Where the dual edge of facet `face`, from orthocentre `ot` (on side `st`) to `on`,
    /// crosses the boundary. Bisected along the facet's own dual line — the points of equal power
    /// to its three weighted vertices, worked from them — between the orthocentres' projections
    /// onto it: a nearly flat tetrahedron's orthocentre is far off and inaccurate, and bisected
    /// toward it directly the crossing leaves the line, is in conflict with neither tetrahedron,
    /// and inserted leaves the facet standing to be refined again at the same place.
    fn dual_crossing(&mut self,face: [u32;3],ot: P,st: i8,on: P) -> P {
        let [a,b,c] = face.map(|v| self.tri.points()[v as usize]);
        let (u,v) = (sub(b.p,a.p),sub(c.p,a.p));
        let n = cross(u,v);
        let nn = dot(n,n);
        if !(nn > 0.) { return self.crossing(ot,st,on); }
        let (ru,rv) = (dot(u,u)-(b.w-a.w),dot(v,v)-(c.w-a.w));
        let (vn,nu) = (cross(v,n),cross(n,u));
        let c0: P = std::array::from_fn(|k| a.p[k]+(ru*vn[k]+rv*nu[k])/(2.*nn));
        let along = |q: P| dot(sub(q,c0),n)/nn;
        let (tt,tn) = (along(ot),along(on));
        let (x,y) = ([0,1,2].map(|k| c0[k]+tt*n[k]),[0,1,2].map(|k| c0[k]+tn*n[k]));
        if !x.iter().chain(&y).all(|z| z.is_finite()) { return self.crossing(ot,st,on); }
        // Where a projection is its orthocentre to within the bisection, the side is known.
        let sn = -st;
        let tol = self.criteria.bisection*self.criteria.bisection;
        let sx = if dist2(x,ot) <= tol { st } else { self.side(x) };
        let sy = if dist2(y,on) <= tol { sn } else { self.side(y) };
        if sx == sy { return self.crossing(ot,st,on); }
        self.crossing(x,sx,y)
    }

    /// A ball `p` lies in, with the refinement margin.
    fn ball_at(&self,p: P) -> Option<usize> {
        let c = self.cell_of(p);
        let margin = self.margin();
        for dx in -1..=1 { for dy in -1..=1 { for dz in -1..=1 {
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

    /// How badly a restricted facet breaks the criteria (above 1 is bad).
    fn badness(&self,key: [u32;3],centre: P) -> f64 {
        let q = key.map(|v| self.tri.points()[v as usize]);
        let [a,b,c] = q.map(|w| w.p);
        let size2 = (dist2(centre,a)-q[0].w).max(0.);
        let mut bad = size2/(self.criteria.facet_size*self.criteria.facet_size);
        let (u,v) = (sub(b,a),sub(c,a));
        let n = cross(u,v);
        let nn = dot(n,n);
        if nn > 0. {
            // Circumcentre of abc: a + (|u|² v × n + |v|² n × u) / (2 |n|²).
            let (vn,nu) = (cross(v,n),cross(n,u));
            let cc: P = std::array::from_fn(|k| a[k]+(dot(u,u)*vn[k]+dot(v,v)*nu[k])/(2.*nn));
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
    /// tetrahedra that do not last, and each costs the field a bisection.
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
                if trace() {
                    let near = created.iter().flat_map(|&t| self.tri.tet(t).v).filter(|&v| v >= ENCLOSING)
                        .map(|v| self.tri.points()[v as usize].p).filter(|&q| q != p)
                        .map(|q| dist2(p,q)).fold(f64::INFINITY,f64::min).sqrt();
                    if near <= 2.*self.criteria.bisection {
                        eprintln!("refine: inserted {p:?} {near:.3e} from a vertex (judging {judging}, {} inserted)",self.report.inserted);
                    }
                }
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
            if trace() && self.report.inserted % 250 == 0 {
                let [a,b,c] = key.map(|v| self.tri.points()[v as usize].p);
                let edges = [dist2(a,b).sqrt(),dist2(b,c).sqrt(),dist2(c,a).sqrt()];
                eprintln!("refine: {} inserted, {} in balls, queue {}: facet edges {:?}, badness {badness:.2} at {:?}",
                    self.report.inserted,self.report.in_balls,self.queue.len(),edges.map(|x| (x*1e4).round()/1e4),
                    centre.map(|x| (x*1e4).round()/1e4));
            }
            // A point that would not remake either tetrahedron of the facet does not refine it
            // (and the facet, judged from both sides, would ask for the same point again).
            let n = self.tri.tet(bad.t).n[bad.i];
            let region = self.tri.conflicts(centre,0.,Some(bad.t))?;
            if !region.iter().any(|&t| t == bad.t || t == n) { self.report.off_dual += 1; continue; }
            self.insert(centre)?;
        }
        Ok(self.queue.is_empty())
    }

    /// The restricted facets, oriented from the material side out.
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
                // Outward: away from the material tetrahedron's opposite vertex.
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

/// `SOLVENT_REFINE_TRACE=1`: a line every 2000 insertions naming the facet being refined.
fn trace() -> bool {
    static ON: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    *ON.get_or_init(|| std::env::var_os("SOLVENT_REFINE_TRACE").is_some())
}

/// The three vertices of the face opposite vertex `i`, in tetrahedron order.
fn facet_vertices(v: [u32;4],i: usize) -> [u32;3] {
    match i { 0 => [v[1],v[2],v[3]], 1 => [v[0],v[2],v[3]], 2 => [v[0],v[1],v[3]], _ => [v[0],v[1],v[2]] }
}

/// The finest a protecting ball's gaps may become, as a fraction of `edge_size`.
const LEAST: f64 = 1024.;
/// Rebuilds with shrunk balls before refusing.
const REBUILDS: usize = 12;

/// How finely a feature curve is sampled: a uniform grid no coarser than `base`, its gaps halved
/// wherever they exceed the target spacing — `h` at arc length `s` for each `(s, h)` in `local`,
/// growing by `GRADING` per unit of arc length away from it. Halving a fixed grid rather than
/// walking the target keeps every station a later refinement does not reach exactly where it
/// was, so a rebuild meets the same balls, tetrahedra and orthocentres there as before.
#[derive(Clone,Debug)]
struct Sizing { base: f64,local: Vec<(f64,f64)> }

const GRADING: f64 = 0.25;
/// A ball's radius over the shorter of its two gaps. The gaps are balanced — within a factor of
/// two of their neighbours, and none longer than both — so across every gap one ball has at least
/// the gap for its shorter one and the other at least half: 0.7 × 1.5 > 1, consecutive balls
/// overlap; next but one are a gap each apart and 0.7 < 1, they do not.
const BALL: f64 = 0.7;

impl Sizing {
    /// The finest target anywhere on the stretch `[a, b]`: a gap touching a constraint's
    /// place must come down to its spacing, where the target at the gap's middle would allow
    /// the grading's worth more and a station beside it would never be as fine as asked.
    fn over(&self,a: f64,b: f64,length: f64,closed: bool) -> f64 {
        self.local.iter().fold(self.base,|h,&(t,ht)| {
            let off = |t: f64| if t < a { a-t } else if t > b { t-b } else { 0. };
            let d = if closed { off(t).min(off(t-length)).min(off(t+length)) } else { off(t) };
            h.min(ht+GRADING*d)
        })
    }
    /// Arc-length stations from 0 to `length`; a closed curve's last, being its first, is left
    /// out.
    fn stations(&self,length: f64,closed: bool) -> Vec<f64> {
        let n = ((length/self.base).ceil() as usize).max(if closed { 3 } else { 1 });
        let mut gaps: Vec<(f64,f64)> = (0..n).map(|k| (length*k as f64/n as f64,length*(k+1) as f64/n as f64)).collect();
        let long = |a: f64,b: f64| a > b*1.0001;
        loop {
            let m = gaps.len();
            let size = |k: usize| gaps[k].1-gaps[k].0;
            let neighbours = |k: usize| -> Vec<f64> {
                let mut out = Vec::new();
                if k > 0 { out.push(size(k-1)); } else if closed { out.push(size(m-1)); }
                if k+1 < m { out.push(size(k+1)); } else if closed { out.push(size(0)); }
                out
            };
            let split: Vec<bool> = (0..m).map(|k| {
                let g = size(k);
                let near = neighbours(k);
                long(g,self.over(gaps[k].0,gaps[k].1,length,closed))
                    || near.iter().any(|&h| long(g,2.*h))
                    || (!near.is_empty() && near.iter().all(|&h| long(g,h)))
            }).collect();
            if !split.contains(&true) { break; }
            gaps = gaps.iter().zip(&split).flat_map(|(&(a,b),&cut)| {
                let mid = 0.5*(a+b);
                if cut { vec![(a,mid),(mid,b)] } else { vec![(a,b)] }
            }).collect();
        }
        let mut out: Vec<f64> = gaps.iter().map(|g| g.0).collect();
        if !closed { out.push(length); }
        out
    }
}

/// A protecting ball's place: its curve, its arc length along it, and the shorter of its gaps.
#[derive(Clone,Copy,Debug)]
struct Place { curve: usize,s: f64,gap: f64 }

/// Protecting balls along the curves: `(centre, radius)` and where each stands, with corners
/// (curve ends, and ends shared by curves) once each. A closed curve repeats its first point at
/// its end. Where balls neither consecutive on one curve nor sharing a corner meet, the curves
/// are refined there, down to an eighth of `edge_size`.
fn protect(curves: &[Vec<P>],edge_size: f64,sizing: &mut [Sizing]) -> Result<(Vec<(P,f64)>,Vec<Vec<Place>>),String> {
    let length = |c: &[P]| c.windows(2).map(|w| dist2(w[0],w[1]).sqrt()).sum::<f64>();
    let at = |c: &[P],s: f64| -> P {
        let mut left = s;
        for w in c.windows(2) {
            let l = dist2(w[0],w[1]).sqrt();
            if left <= l && l > 0. { return lerp(w[0],w[1],left/l); }
            left -= l;
        }
        *c.last().unwrap()
    };
    let lengths: Vec<f64> = curves.iter().map(|c| length(c)).collect();
    let closed: Vec<bool> = curves.iter().map(|c| c.len() >= 2 && dist2(c[0],*c.last().unwrap()) == 0.).collect();
    let finest = edge_size/8.;
    let least = edge_size/LEAST;
    for round in 0..16 {
        // Each ball: centre, radius, and the (curve, station) places it stands at; a corner
        // stands at several. `at_place` finds a ball by place.
        let mut balls: Vec<(P,f64)> = Vec::new();
        let mut places: Vec<Vec<(usize,usize)>> = Vec::new();
        let mut at_place: std::collections::BTreeMap<(usize,usize),usize> = Default::default();
        let stations: Vec<Vec<f64>> = (0..curves.len()).map(|k|
            if curves[k].len() < 2 { Vec::new() } else { sizing[k].stations(lengths[k],closed[k]) }).collect();
        // The shorter gap either side of station `j`.
        let gap = |k: usize,j: usize| -> f64 {
            let st = &stations[k];
            let n = st.len();
            let next = if j+1 < n { st[j+1]-st[j] } else if closed[k] { lengths[k]-st[j] } else { f64::INFINITY };
            let prev = if j > 0 { st[j]-st[j-1] } else if closed[k] { lengths[k]-st[n-1] } else { f64::INFINITY };
            next.min(prev)
        };
        for (k,c) in curves.iter().enumerate() {
            let n = stations[k].len();
            for j in 0..n {
                // A curve's ends are its own points exactly: an end is a corner another curve
                // must meet.
                let p = if j == 0 { c[0] } else if !closed[k] && j+1 == n { *c.last().unwrap() } else { at(c,stations[k][j]) };
                let r = BALL*gap(k,j);
                let end = j == 0 || (!closed[k] && j+1 == n);
                let same = |q: P| dist2(q,p) <= (1e-9*edge_size)*(1e-9*edge_size);
                let b = match balls.iter().position(|b| end && same(b.0)) {
                    Some(b) => { balls[b].1 = balls[b].1.min(r); b }
                    None => { balls.push((p,r)); places.push(Vec::new()); balls.len()-1 }
                };
                places[b].push((k,j));
                at_place.insert((k,j),b);
            }
        }
        // Neighbours along a curve, cyclically on a closed one.
        let next_to = |b: usize| -> Vec<usize> {
            let mut out = Vec::new();
            for &(k,j) in &places[b] {
                let n = stations[k].len();
                let mut near = Vec::new();
                if j > 0 { near.push(j-1); } else if closed[k] { near.push(n-1); }
                if j+1 < n { near.push(j+1); } else if closed[k] { near.push(0); }
                out.extend(near.into_iter().filter_map(|m| at_place.get(&(k,m)).copied()));
            }
            out
        };
        let neighbours: Vec<Vec<usize>> = (0..balls.len()).map(next_to).collect();
        let corner: Vec<bool> = places.iter().map(|p| p.len() > 1).collect();
        // Two balls may meet when consecutive, or both next to one corner.
        let may_meet = |x: usize,y: usize| {
            neighbours[x].contains(&y)
                || neighbours[x].iter().any(|&c| corner[c] && neighbours[y].contains(&c))
        };
        // Near a corner where two curves meet at a small angle they part only linearly (or
        // quadratically, tangent), and no spacing keeps their balls apart there: once a curve is
        // at its finest spacing (an eighth of `edge_size`), balls within `edge_size` along their
        // curves of a corner both share may meet. A distance and not a count of stations: a
        // rebuild samples a curve more finely than `finest`, and the region stays the same.
        let near_corner = |b: usize,c: usize| -> bool {
            places[b].iter().all(|&(k,j)| places[c].iter().any(|&(kc,jc)| {
                kc == k && {
                    let along = (stations[k][j]-stations[k][jc]).abs();
                    along <= edge_size*1.0001 || (closed[k] && lengths[k]-along <= edge_size*1.0001)
                }
            }))
        };
        let fine_gap = finest*1.0001;
        let fine = |b: usize| places[b].iter().all(|&(k,j)| gap(k,j) <= fine_gap);
        // Likewise a curve turning more tightly than its finest balls: stations within
        // `edge_size` of each other along it may meet, the turn being a corner at that scale.
        let beside = |x: usize,y: usize| (0..balls.len()).any(|c| corner[c] && near_corner(x,c) && near_corner(y,c)) || near_corner(x,y);
        let tangent = |x: usize,y: usize| fine(x) && fine(y) && beside(x,y);
        let mut tight: Vec<(usize,f64,f64)> = Vec::new();
        let mut example = None;
        for x in 0..balls.len() { for y in x+1..balls.len() {
            let reach = balls[x].1+balls[y].1;
            if dist2(balls[x].0,balls[y].0) >= reach*reach || may_meet(x,y) || tangent(x,y) { continue; }
            // Beside a corner both share, straight to the finest spacing (halving one station at
            // a time, the next round's stations stand elsewhere and the pair that met is a
            // different pair). Elsewhere two curves merely pass close, and their balls are sized
            // to the distance between them, however fine that is.
            let d = dist2(balls[x].0,balls[y].0).sqrt();
            let target = if beside(x,y) { finest } else { (0.9*d/(2.*BALL)).max(least) };
            for &(k,j) in places[x].iter().chain(&places[y]) {
                if gap(k,j) > target*1.0001 { tight.push((k,stations[k][j],target)); }
            }
            example.get_or_insert((x,y));
        } }
        if example.is_none() {
            let owners = places.iter().map(|p| p.iter().map(|&(k,j)| Place {curve:k,s:stations[k][j],gap:gap(k,j)}).collect()).collect();
            return Ok((balls,owners));
        }
        if tight.is_empty() || round == 15 {
            let (x,y) = example.unwrap();
            return Err(format!("the feature curves could not be protected: balls at {:?} and {:?} (places {:?} and {:?}) \
                still meet, finest {:.2e} (radii {:.3e} and {:.3e}, gaps {:?} and {:?}, round {round})",balls[x].0,balls[y].0,places[x],places[y],finest,
                balls[x].1,balls[y].1,places[x].iter().map(|&(k,j)| gap(k,j)).collect::<Vec<_>>(),places[y].iter().map(|&(k,j)| gap(k,j)).collect::<Vec<_>>()));
        }
        for (k,s,h) in tight { sizing[k].local.push((s,h)); }
    }
    unreachable!()
}

/// Mesh the boundary of `{p : side(p) < 0}` within `radius` of `centre`.
pub fn mesh(side: &mut dyn FnMut(P) -> f64,centre: P,radius: f64,curves: &[Vec<P>],criteria: &Criteria)
    -> Result<Mesh,String> {
    let mut run = Progressive::new(Box::new(|p| side(p)),centre,radius,curves.to_vec(),criteria.clone());
    while !run.step(usize::MAX)? {}
    run.finished()
}

/// Where a progressive refinement stands.
enum Stage {
    /// Protect, seed or re-insert, and judge every facet.
    Build,
    /// Refine the queued facets, worst first.
    Refine,
    /// Repair the manifold, check what the balls hold off the surface, and finish or rebuild.
    Repair,
    Done(Result<Mesh,String>),
}

/// **A refinement that can be stopped and looked at**: `step` does a bounded amount of work, and
/// `snapshot` gives the restricted facets as they stand — worst first means the first snapshots
/// are the coarse shape and every later one a finer version of it, which is what a preview shows
/// while the refinement goes on. A snapshot before the end may be open or not manifold.
///
/// Refinement that a ball blocks is the ball's to give way: the curves are sampled twice as
/// finely where balls blocked a manifold repair, graded back to their spacing, and the
/// triangulation rebuilt with the new balls and every point kept so far (Mesh_3 shrinks balls in
/// place; this triangulation removes no vertex, and a rebuild costs a few microseconds a point,
/// the field's answers being remembered).
pub struct Progressive<'a> {
    domain: Option<Box<dyn FnMut(P) -> f64 + 'a>>,
    refiner: Option<Refiner<'a>>,
    centre: P,
    radius: f64,
    curves: Vec<Vec<P>>,
    criteria: Criteria,
    sizing: Vec<Sizing>,
    least: f64,
    kept: Vec<P>,
    memo: HashMap<[u64;3],i8>,
    queries: usize,
    rebuild: usize,
    stage: Stage,
}

impl<'a> Progressive<'a> {
    pub fn new(side: Box<dyn FnMut(P) -> f64 + 'a>,centre: P,radius: f64,curves: Vec<Vec<P>>,criteria: Criteria) -> Self {
        let length = |c: &[P]| c.windows(2).map(|w| dist2(w[0],w[1]).sqrt()).sum::<f64>();
        let sizing = curves.iter().map(|c| Sizing {base:length(c).min(criteria.edge_size).max(criteria.edge_size*1e-3),local:Vec::new()}).collect();
        let least = criteria.edge_size/LEAST;
        Self {domain:Some(side),refiner:None,centre,radius,curves,criteria,sizing,least,kept:Vec::new(),
            memo:HashMap::new(),queries:0,rebuild:0,stage:Stage::Build}
    }

    /// Do at most about `budget` refinements (a build or a repair runs whole): whether the
    /// refinement has finished, well or not — `finished` says which.
    pub fn step(&mut self,budget: usize) -> Result<bool,String> {
        match self.stage {
            Stage::Done(_) => return Ok(true),
            Stage::Build => { self.build()?; self.stage = Stage::Refine; }
            Stage::Refine => {
                let r = self.refiner.as_mut().expect("a refiner while refining");
                if r.refine_some(budget)? { self.stage = Stage::Repair; }
            }
            Stage::Repair => {
                let outcome = self.repair();
                if let Some(result) = outcome { self.stage = Stage::Done(result); }
            }
        }
        Ok(matches!(self.stage,Stage::Done(_)))
    }

    /// Whether the refinement has finished, well or not.
    pub fn done(&self) -> bool { matches!(self.stage,Stage::Done(_)) }

    /// Why a finished refinement could not make a mesh, if it could not.
    pub fn done_error(&self) -> Option<String> {
        match &self.stage { Stage::Done(Err(e)) => Some(e.clone()), _ => None }
    }

    /// The mesh a finished refinement made, or why it could not.
    pub fn finished(self) -> Result<Mesh,String> {
        match self.stage {
            Stage::Done(result) => result,
            _ => Err("the refinement has not finished".into()),
        }
    }

    /// The restricted facets as they stand, oriented outward; empty before the first build.
    pub fn snapshot(&mut self) -> Mesh {
        if let Stage::Done(Ok(m)) = &self.stage { return m.clone(); }
        let Some(r) = self.refiner.as_mut() else { return Mesh {vertices:Vec::new(),triangles:Vec::new(),report:Report::default()} };
        let facets = r.extract();
        let (vertices,triangles) = collect(r,&facets);
        Mesh {vertices,triangles,report:r.report.clone()}
    }

    fn build(&mut self) -> Result<(),String> {
        let criteria = &self.criteria;
        let (balls,owners) = protect(&self.curves,criteria.edge_size,&mut self.sizing)?;
        let cell = balls.iter().map(|b| b.1).fold(criteria.edge_size,f64::max).max(self.radius*1e-9);
        let domain = match self.refiner.take() {
            Some(old) => { self.kept = old.kept; self.memo = old.memo; self.queries = old.report.queries; old.domain }
            None => self.domain.take().expect("the domain before the first build"),
        };
        let mut r = Refiner {domain,centre:self.centre,radius:self.radius,criteria:criteria.clone(),
            tri:Regular::new(self.centre,self.radius),sign:Vec::new(),balls:Vec::new(),owners:Vec::new(),
            ball_of:HashMap::new(),kept:Vec::new(),memo:std::mem::take(&mut self.memo),blocking:Vec::new(),
            grid:HashMap::new(),cell,queue:BinaryHeap::new(),report:Report {queries:self.queries,..Report::default()}};
        // Protecting balls first, in spatial order.
        let order = super::spatial_order(&balls.iter().map(|b| b.0).collect::<Vec<_>>());
        for &k in &order {
            let (p,rad) = balls[k];
            if let Inserted::Vertex(v) = r.tri.insert(p,rad*rad)? { r.ball_of.insert(v,r.balls.len()); }
            let key = r.cell_of(p);
            r.grid.entry(key).or_default().push(r.balls.len());
            r.balls.push((p,rad));
            r.owners.push(owners[k].clone());
        }
        r.report.balls = r.balls.len();
        r.report.rebuilds = self.rebuild;
        if self.rebuild == 0 {
            seed(&mut r)?;
        } else {
            let kept = std::mem::take(&mut self.kept);
            for &k in &super::spatial_order(&kept) { r.insert_judging(kept[k],false)?; }
            r.blocking.clear();
        }
        // Every facet now; refinement follows in steps.
        let tets: Vec<u32> = r.tri.tets().collect();
        for t in tets { for i in 0..4 { r.judge(t,i); } }
        self.refiner = Some(r);
        Ok(())
    }

    /// Repair and judge the refined surface: the finished mesh, a refusal, or `None` after
    /// arranging a rebuild with shrunk balls.
    fn repair(&mut self) -> Option<Result<Mesh,String>> {
        let rebuild = self.rebuild;
        let r = self.refiner.as_mut().expect("a refiner while repairing");
        let manifold = match repair(r) { Ok(m) => m, Err(e) => return Some(Err(e)) };
        if manifold {
            let facets = r.extract();
            // A facet with a protecting ball for a vertex may stand off the surface where no
            // refinement could reach it, its surface centre inside the ball: those balls are too
            // big for the surface's curvature there, and shrink as a repair's blocking balls do.
            let off = standing_off(r,&facets);
            r.report.coarse = off.len();
            if trace() && !off.is_empty() { eprintln!("refine: rebuild {rebuild}: {} facets stand off the surface",off.len()); }
            let blocking: Vec<usize> = off.iter().flatten().copied().collect();
            if blocking.is_empty() || rebuild+1 == REBUILDS || !shrink(r,&blocking,&mut self.sizing,self.least) {
                let (vertices,triangles) = collect(r,&facets);
                return Some(Ok(Mesh {vertices,triangles,report:r.report.clone()}));
            }
        } else {
            let mut blocking = r.blocking.clone();
            blocking.sort_unstable();
            blocking.dedup();
            let shrunk = shrink(r,&blocking,&mut self.sizing,self.least);
            if trace() {
                let faults = non_manifold(&r.extract());
                eprintln!("refine: rebuild {rebuild}: {} faults (first at {:?}), {} points, {} queries; {} blocking balls, {} balls",
                    faults.len(),faults.first().map(|f| r.tri.orthosphere(f.0).0),r.kept.len(),r.report.queries,blocking.len(),r.balls.len());
                for &b in blocking.iter().take(6) { eprintln!("refine:   ball {b} at {:?} r {:.3e}",r.balls[b].0,r.balls[b].1); }
                for &(t,i,key) in faults.iter().take(8) {
                    let n = r.tri.tet(t).n[i];
                    eprintln!("refine:   fault {key:?} sides {} {}: {:?}",r.tet_side(t),r.tet_side(n),
                        key.map(|v| { let q = r.tri.points()[v as usize]; (q.p.map(|x| (x*1e4).round()/1e4),(q.w.max(0.).sqrt()*1e4).round()/1e4) }));
                }
            }
            if !shrunk { return Some(Err("the boundary is not a manifold, and no ball blocking its repair can shrink".into())); }
            if rebuild+1 == REBUILDS {
                return Some(Err(format!("the boundary is not a manifold after refining the protecting balls {REBUILDS} times")));
            }
        }
        self.rebuild += 1;
        self.stage = Stage::Build;
        None
    }
}

/// The facets' vertices, renumbered from the triangulation's, and the triangles over them.
fn collect(r: &Refiner,facets: &[([u32;3],u32,usize)]) -> (Vec<P>,Vec<[u32;3]>) {
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

/// Halve the feature spacing at the places of balls `blocking`, down to `least`: whether any
/// could shrink.
fn shrink(r: &Refiner,blocking: &[usize],sizing: &mut [Sizing],least: f64) -> bool {
    let mut blocking = blocking.to_vec();
    blocking.sort_unstable();
    blocking.dedup();
    let mut shrunk = false;
    for &b in &blocking {
        for place in &r.owners[b] {
            if place.gap > least*1.0001 { sizing[place.curve].local.push((place.s,(place.gap*0.5).max(least))); shrunk = true; }
        }
    }
    shrunk
}

/// The facets with a protecting ball for a vertex whose surface does not cross the facet's normal
/// line within ten times `facet_distance` of its centroid, each as the balls at its vertices.
/// Judged by the side at the two ends of that stretch of the normal, both the same.
fn standing_off(r: &mut Refiner,facets: &[([u32;3],u32,usize)]) -> Vec<Vec<usize>> {
    let d = 10.*r.criteria.facet_distance;
    let mut out = Vec::new();
    for &(f,_,_) in facets {
        let held: Vec<usize> = f.iter().filter_map(|v| r.ball_of.get(v).copied()).collect();
        if held.is_empty() { continue; }
        let [a,b,c] = f.map(|v| r.tri.points()[v as usize].p);
        let n = cross(sub(b,a),sub(c,a));
        let l = dot(n,n).sqrt();
        if !(l > 0.) { continue; }
        let g: P = std::array::from_fn(|k| (a[k]+b[k]+c[k])/3.);
        let (x,y) = ([0,1,2].map(|k| g[k]+d*n[k]/l),[0,1,2].map(|k| g[k]-d*n[k]/l));
        if r.side(x) == r.side(y) { out.push(held); }
    }
    out
}

/// Seed the boundary along rays from the centre.
fn seed(r: &mut Refiner) -> Result<(),String> {
    let (rays,steps) = (128,96);
    let golden = std::f64::consts::PI*(3.-5f64.sqrt());
    let (centre,radius) = (r.centre,r.radius);
    for k in 0..rays {
        let z = 1.-2.*(k as f64+0.5)/rays as f64;
        let s = (1.-z*z).sqrt();
        let d = [s*(golden*k as f64).cos(),s*(golden*k as f64).sin(),z];
        let point = |j: usize| lerp(centre,[centre[0]+radius*d[0],centre[1]+radius*d[1],centre[2]+radius*d[2]],j as f64/steps as f64);
        let mut last = (point(0),r.side(point(0)));
        for j in 1..=steps {
            let p = point(j);
            let sp = r.side(p);
            if sp != last.1 {
                let c = r.crossing(last.0,last.1,p);
                if r.insert_judging(c,false)? { r.report.seeds += 1; }
            }
            last = (p,sp);
        }
    }
    Ok(())
}

/// Refine at the boundary's manifold faults until none is left (true), or until refining them
/// inserts nothing, every point needed having fallen in a ball (false: `blocking` says which).
fn repair(r: &mut Refiner) -> Result<bool,String> {
    for round in 0..32 {
        let facets = r.extract();
        let faults = non_manifold(&facets);
        if faults.is_empty() { return Ok(true); }
        r.report.manifold_rounds = round+1;
        r.blocking.clear();
        let mut any = false;
        for (t,i,key) in faults {
            // Each insertion may remake the facets after it in the list.
            if !r.tri.alive(t) { continue; }
            let mut face = facet_vertices(r.tri.tet(t).v,i);
            face.sort_unstable();
            if face != key { continue; }
            if let Some((key,centre)) = r.restricted(t,i) {
                // A crossing is placed only to `bisection`: a surface centre no farther than that
                // from its facet's vertices says nothing about where the boundary is, and inserted
                // it duplicates a vertex.
                let near = key.iter().map(|&v| dist2(centre,r.tri.points()[v as usize].p)).fold(f64::INFINITY,f64::min).sqrt();
                if near <= 2.*r.criteria.bisection { r.report.too_small += 1; continue; }
                any |= r.insert(centre)?;
                r.refine()?;
            }
        }
        if !any { return Ok(false); }
    }
    Ok(non_manifold(&r.extract()).is_empty())
}

/// The facets at a manifold fault: each edge used other than twice and each vertex whose
/// triangles form more than one fan, as the largest facet of each.
fn non_manifold(facets: &[([u32;3],u32,usize)]) -> Vec<(u32,usize,[u32;3])> {
    let mut edges: BTreeMap<(u32,u32),Vec<usize>> = BTreeMap::new();
    let mut around: BTreeMap<u32,Vec<usize>> = BTreeMap::new();
    for (k,(f,_,_)) in facets.iter().enumerate() {
        for j in 0..3 {
            let (a,b) = (f[j],f[(j+1)%3]);
            edges.entry((a.min(b),a.max(b))).or_default().push(k);
            around.entry(f[j]).or_default().push(k);
        }
    }
    let mut faults: Vec<usize> = edges.values().filter(|l| l.len() != 2).flatten().copied().collect();
    for (&v,list) in &around {
        // The triangles about v in one fan: connected through edges at v.
        let mut seen = vec![false;list.len()];
        let mut stack = vec![0];
        seen[0] = true;
        while let Some(x) = stack.pop() {
            let fx = facets[list[x]].0;
            for (y,&ky) in list.iter().enumerate() {
                if seen[y] { continue; }
                let fy = facets[ky].0;
                let shared = fx.iter().filter(|&&a| a != v && fy.contains(&a)).count();
                if shared > 0 { seen[y] = true; stack.push(y); }
            }
        }
        if seen.iter().any(|s| !s) { faults.extend(list.iter().copied()); }
    }
    faults.sort_unstable();
    faults.dedup();
    faults.into_iter().map(|k| { let mut key = facets[k].0; key.sort_unstable(); (facets[k].1,facets[k].2,key) }).collect()
}
