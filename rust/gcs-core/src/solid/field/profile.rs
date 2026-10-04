//! Planar material fields of closed profile loops of lines and circular arcs.
//! A convex loop is the exact intersection of its outward half-planes and
//! circular sectors, which is cheap and a true signed distance on each wall.
//! Any other simple loop is its signed boundary distance: the unsigned
//! distance to every wall is enclosed over a whole box, and a box that stays
//! clear of the boundary takes one sign, read by ray parity at its centre.
//! That field is one-Lipschitz by construction, whatever the winding, and a
//! hole is an ordinary loop subtracted. Snapshot construction reads solved
//! floating-point curves; interval evaluation does not certify that reading.
#[allow(unused_imports)]
use crate::fmath::Det;
use super::{norm,min,point,Error,I,Node,PlanarField,RevolvedField};
use crate::solid::surface::{Edge,RevolvedRegion};
use std::f64::consts::{PI,TAU};

type P = [f64;2];
fn cross(a: P,b: P) -> f64 { a[0]*b[1]-a[1]*b[0] }
fn delta(a: P,b: P) -> P { [a[0]-b[0],a[1]-b[1]] }
fn distance(a: P,b: P) -> f64 { let (x,y) = (a[0]-b[0],a[1]-b[1]); (x*x+y*y).sqrt() }
fn failure(e: Error) -> String { format!("profile field: {e:?}") }

impl Edge {
    // Connectivity comes from the source vertices. Reconstructing a constrained
    // arc endpoint with radius and trigonometry adds source-solve and rounding
    // residuals; those must not turn one shared vertex into two separate ones.
    fn endpoints(&self) -> [P;2] {
        match *self {
            Self::Line {a,b,..} => [a,b],
            Self::Arc {sweep,ends,..} if sweep.abs() < TAU => ends,
            Self::Arc {..} => [self.at(0.),self.at(1.)],
        }
    }
    fn reversed(mut self) -> Self {
        match &mut self {
            Self::Line {a,b,..} => std::mem::swap(a,b),
            Self::Arc {start,sweep,ends,..} => {
                *start += *sweep; *sweep = -*sweep; ends.swap(0,1);
            }
        }
        self
    }
    fn tangent(&self,t: f64) -> P {
        match *self {
            Self::Line {a,b,..} => delta(b,a),
            Self::Arc {start,sweep,..} => {
                let (s,c) = (start+t*sweep).dsin_cos();
                [-s*sweep.signum(),c*sweep.signum()]
            }
        }
    }
    fn area_twice(&self) -> f64 {
        match *self {
            Self::Line {a,b,..} => cross(a,b),
            Self::Arc {center,radius,sweep,..} =>
                cross(center,delta(self.at(1.),self.at(0.)))+radius*radius*sweep,
        }
    }
    // The farthest boundary point from planar zero; material lies within the
    // convex hull of its boundary, hence inside this disk.
    fn reach(&self) -> f64 {
        match *self {
            Self::Line {a,b,..} => a[0].dhypot(a[1]).max(b[0].dhypot(b[1])),
            Self::Arc {center,radius,..} => center[0].dhypot(center[1])+radius,
        }
    }
}

/// Boundary order and individual edge direction need not match a face's written
/// operand order. Reconstruct one loop before checking turning and convexity.
/// `join` is the distance an endpoint put on a revolution axis was moved by,
/// so the edges sharing that point join within it as well as within the
/// roundoff of the conversion.
fn ordered(edges: &[Edge],join: f64) -> Result<Vec<Edge>,String> {
    let scale = edges.iter().map(Edge::reach).fold(0_f64,f64::max);
    let tolerance = (scale*f64::EPSILON*128.).max(join);
    if edges.is_empty() { return Err("empty material profile".into()); }
    // a loop read off in order (a curve's chords, a face walked) needs no search, which is
    // quadratic: each edge ends where the next begins, and the last where the first does
    let n = edges.len();
    if (0..n).all(|i| distance(edges[i].endpoints()[1],edges[(i+1)%n].endpoints()[0]) <= tolerance) {
        return Ok(edges.to_vec());
    }
    let mut remaining = edges.to_vec();
    let mut out = vec![remaining.remove(0)];
    while !remaining.is_empty() {
        let end = out.last().unwrap().endpoints()[1];
        let mut next = None;
        for (i,e) in remaining.iter().enumerate() {
            for (p,reverse) in e.endpoints().into_iter().zip([false,true]) {
                if distance(end,p) <= tolerance {
                    if next.is_some() { return Err("ambiguous material profile junction".into()); }
                    next = Some((i,reverse));
                }
            }
        }
        let (i,reverse) = next.ok_or_else(|| {
            let gap = remaining.iter().flat_map(|e| e.endpoints().map(|p| distance(end,p)))
                .fold(f64::INFINITY,f64::min);
            format!("material profile endpoints do not meet: gap {gap:e}, tolerance {tolerance:e}")
        })?;
        let edge = remaining.remove(i);
        out.push(if reverse { edge.reversed() } else { edge });
    }
    if distance(out[0].endpoints()[0],out.last().unwrap().endpoints()[1]) > tolerance {
        return Err("material profile is not closed".into());
    }
    Ok(out)
}

/// The loop's orientation (+1 counter-clockwise) and whether every arc and every
/// junction turns toward the material. A simple loop's tangent turns exactly
/// once, whichever way its corners go; anything else is not one loop.
fn turning(edges: &[Edge]) -> Result<(f64,bool),String> {
    let area = edges.iter().map(Edge::area_twice).sum::<f64>();
    if !area.is_finite() || area == 0. { return Err("degenerate material profile".into()); }
    let direction = area.signum();
    let mut convex = true;
    let mut turning = 0.;
    for (i,e) in edges.iter().enumerate() {
        if let Edge::Arc {sweep,..} = *e {
            if sweep*direction <= 0. { convex = false; }
            turning += sweep*direction;
        }
        let a = e.tangent(1.); let b = edges[(i+1)%edges.len()].tangent(0.);
        let turn = (direction*cross(a,b)).datan2(a[0]*b[0]+a[1]*b[1]);
        // Tangent junctions inherit the solved source's floating-point residual.
        if turn < -1e-9 { convex = false; }
        turning += turn;
    }
    if (turning-TAU).abs() > 1e-8 {
        return Err("material profile does not make one turn".into());
    }
    Ok((direction,convex))
}

fn convex(edges: &[Edge],direction: f64) -> Result<PlanarField,String> {
    let mut constraints = Vec::new();
    let mut radius = 0_f64;
    let circular_section =
        edges.iter().filter(|e| !matches!(e,Edge::Line {axis:true,..})).count() == 1;
    for e in edges {
        radius = radius.max(e.reach());
        let field = match *e {
            Edge::Line {a,b,axis} => {
                // Cylindrical radius is already nonnegative. The construction
                // diameter of a sphere/cylinder must not become a material wall.
                if axis { continue; }
                PlanarField::half_plane(a,[direction*(b[1]-a[1]),direction*(a[0]-b[0])])
                    .map_err(failure)?
            }
            Edge::Arc {center,radius:r,sweep,..} => {
                let disk = PlanarField::disk(center,r).map_err(failure)?;
                if sweep.abs() >= TAU || circular_section { disk }
                else {
                    let (a,b) = if direction > 0. { (e.at(0.),e.at(1.)) }
                        else { (e.at(1.),e.at(0.)) };
                    let a = delta(a,center); let b = delta(b,center);
                    let before = PlanarField::half_plane(center,[-a[1],a[0]]).map_err(failure)?;
                    let after = PlanarField::half_plane(center,[b[1],-b[0]]).map_err(failure)?;
                    // The circle constrains its angular sector only. Extending
                    // the whole disk would cut away distant parts of a fillet's profile.
                    let outside = if sweep.abs() <= PI { before.union(after) }
                        else { before.intersection(after) }.map_err(failure)?;
                    disk.union(outside).map_err(failure)?
                }
            }
        };
        constraints.push(field);
    }
    // Convex material lies in the convex hull of its boundary, hence this disk
    // contains it strictly. It also bounds profiles made solely of half-planes.
    // Intersected pairwise, a balanced tree: a loop of many chords (a spline's) stays well
    // within a field's depth, where one after another it would not.
    let mut level = vec![PlanarField::disk([0.;2],radius*2.).map_err(failure)?];
    level.extend(constraints);
    while level.len() > 1 {
        let mut next = Vec::with_capacity(level.len().div_ceil(2));
        let mut it = level.into_iter();
        while let Some(a) = it.next() {
            next.push(match it.next() { Some(b) => a.intersection(b).map_err(failure)?,None => a });
        }
        level = next;
    }
    Ok(level.pop().expect("a convex field has its bounding disk"))
}

// One wall's distance, enclosed over a box by interval arithmetic. A segment is
// its perpendicular offset and the overshoot past either end; an arc is the
// ring distance where the box lies within its angular sector, the nearer
// endpoint where it lies wholly outside, and the ring's lower bound with the
// endpoints' upper bound between (the ring never exceeds an endpoint, which is
// on it). Sector membership is a pair of cross-product signs against the exact
// source vertices, so no trigonometry enters; a reflex sector is the complement
// of a convex one. Axis lines are no wall: they vanish in a full revolution.
#[derive(Clone,Debug)]
enum Wall {
    Segment {a:[I;2],direction:[I;2],length:I},
    Round {center:[I;2],radius:I,ends:[[I;2];2],from:[I;2],to:[I;2],reflex:bool},
    Circle {center:[I;2],radius:I},
}

fn nonnegative(x: I) -> I { I::new(x.bounds()[0].max(0.),x.bounds()[1].max(0.)).unwrap() }
fn magnitude(x: I) -> I {
    let [lo,hi] = x.bounds();
    if lo >= 0. { x } else if hi <= 0. { x.neg() } else { I::new(0.,(-lo).max(hi)).unwrap() }
}
fn wedge(a: [I;2],b: [I;2]) -> Result<I,Error> { a[0].mul(b[1])?.sub(a[1].mul(b[0])?) }
fn offset(p: [I;2],c: [I;2]) -> Result<[I;2],Error> { Ok([p[0].sub(c[0])?,p[1].sub(c[1])?]) }

impl Wall {
    fn of(edge: &Edge) -> Result<Option<Self>,Error> {
        Ok(Some(match *edge {
            Edge::Line {axis:true,..} => return Ok(None),
            Edge::Line {a,b,..} => {
                let (a,b) = (point(a)?,point(b)?);
                let chord = offset(b,a)?; let length = norm(chord)?;
                Self::Segment {a,direction:[chord[0].div(length)?,chord[1].div(length)?],length}
            }
            Edge::Arc {center,radius,sweep,..} if sweep.abs() >= TAU =>
                Self::Circle {center:point(center)?,radius:I::point(radius)?},
            Edge::Arc {center,radius,sweep,ends,..} => {
                let center = point(center)?; let radius = I::point(radius)?;
                let ccw = if sweep > 0. { [ends[0],ends[1]] } else { [ends[1],ends[0]] };
                let (from,to) = (offset(point(ccw[0])?,center)?,offset(point(ccw[1])?,center)?);
                // The arc's ends lie on its own circle along the vertices' directions.
                let on_circle = |v: [I;2]| -> Result<[I;2],Error> {
                    let scale = radius.div(norm(v)?)?;
                    Ok([center[0].add(v[0].mul(scale)?)?,center[1].add(v[1].mul(scale)?)?])
                };
                Self::Round {center,radius,ends:[on_circle(from)?,on_circle(to)?],from,to,
                    reflex:sweep.abs() > PI}
            }
        }))
    }

    fn distance(&self,p: [I;2]) -> Result<I,Error> {
        match self {
            Self::Segment {a,direction,length} => {
                let q = offset(p,*a)?;
                let along = q[0].mul(direction[0])?.add(q[1].mul(direction[1])?)?;
                let across = magnitude(wedge(q,*direction)?);
                let over = nonnegative(along.sub(*length)?);
                let under = nonnegative(along.neg());
                norm([across,over,under])
            }
            Self::Circle {center,radius} => Ok(magnitude(norm(offset(p,*center)?)?.sub(*radius)?)),
            Self::Round {center,radius,ends,from,to,reflex} => {
                let q = offset(p,*center)?;
                let ring = magnitude(norm(q)?.sub(*radius)?);
                let (first,second) = (wedge(*from,q)?.bounds(),wedge(q,*to)?.bounds());
                let (within,beyond) = if *reflex {
                    (first[0] >= 0. || second[0] >= 0.,first[1] < 0. && second[1] < 0.)
                } else {
                    (first[0] >= 0. && second[0] >= 0.,first[1] < 0. || second[1] < 0.)
                };
                if within { return Ok(ring); }
                let corners = min(norm(offset(p,ends[0])?)?,norm(offset(p,ends[1])?)?);
                if beyond { return Ok(corners); }
                let hi = corners.bounds()[1];
                I::new(ring.bounds()[0].min(hi),hi)
            }
        }
    }
}

/// A box about every edge, in a hierarchy (`crate::bvh`): what lets a loop of thousands of chords
/// (a curve's) answer a point in the logarithm of its edges. Each box is its edge's own, grown by a
/// hair of the loop's reach, so a box passed by for being farther than the nearest edge found, or
/// clear of a point's ray, holds no edge that would have changed the answer: a point reads exactly
/// what a walk over every edge reads, ties to the earlier edge included. Over a box the enclosure
/// skips walls whose box lies past the least upper bound found, so it may be tighter than the
/// walk's, and is as sound: none of them can be nearer than that bound anywhere in the box.
#[derive(Clone,Debug)]
struct Boxes(std::sync::Arc<crate::bvh::Bvh<2>>);

impl Boxes {
    fn new(edges: &[Edge],reach: f64) -> Boxes {
        let grow = 1e-9*(1.+reach);
        Boxes(std::sync::Arc::new(crate::bvh::Bvh::new(edges.iter().map(|e| {
            let (lo,hi) = match *e {
                Edge::Line {a,b,..} => ([a[0].min(b[0]),a[1].min(b[1])],[a[0].max(b[0]),a[1].max(b[1])]),
                Edge::Arc {center,radius,..} => ([center[0]-radius,center[1]-radius],[center[0]+radius,center[1]+radius]),
            };
            crate::bvh::Bounds {lo:[lo[0]-grow,lo[1]-grow],hi:[hi[0]+grow,hi[1]+grow]}
        }))))
    }

    /// The edge nearest `p` and its distance, the earlier edge where two tie.
    fn nearest(&self,edges: &[Edge],p: P) -> (f64,usize) {
        let best = std::cell::Cell::new((f64::INFINITY,usize::MAX));
        self.0.query_nearest(|b| b.gap(crate::bvh::Bounds {lo:p,hi:p}),|| best.get().0,|k| {
            let d = edges[k].distance(p);
            let (bd,bk) = best.get();
            if d < bd || (d == bd && k < bk) { best.set((d,k)) }
        });
        best.get()
    }

    /// How many of the edges `p`'s ray toward +x crosses.
    fn crossings(&self,edges: &[Edge],p: P) -> usize {
        let mut count = 0;
        self.0.query(|b| p[1] >= b.lo[1] && p[1] <= b.hi[1] && p[0] <= b.hi[0],|k| count += edges[k].crossings(p));
        count
    }

    /// Every wall's distance over box `p` folded by `min`, passing by a box farther than the least
    /// upper bound found: none of its walls could lower either end of the minimum.
    fn bounds(&self,walls: &[Option<Wall>],p: [I;2]) -> Result<Option<I>,Error> {
        let (lo,hi) = ([p[0].bounds()[0],p[1].bounds()[0]],[p[0].bounds()[1],p[1].bounds()[1]]);
        let nearest: std::cell::Cell<Option<I>> = std::cell::Cell::new(None);
        let mut failed = None;
        self.0.query_nearest(|b| b.gap(crate::bvh::Bounds {lo,hi})*(1.-1e-12),|| nearest.get().map_or(f64::INFINITY,|m| m.bounds()[1]),|k| {
            let Some(wall) = &walls[k] else { return };
            match wall.distance(p) {
                Ok(d) => nearest.set(Some(nearest.get().map_or(d,|m| min(m,d)))),
                Err(e) => failed = Some(e),
            }
        });
        match failed { Some(e) => Err(e),None => Ok(nearest.get()) }
    }
}

/// A simple closed loop as its signed boundary distance, interval-evaluable.
#[derive(Clone,Debug)]
pub(super) struct Profile { edges:Vec<Edge>,
    /// Per edge its wall, `None` for an axis line, which is no wall.
    walls:Vec<Option<Wall>>,reach:f64,boxes:Boxes,
    /// Per edge, whether the material lies on its left (a line) or inside its circle (an arc):
    /// read once off the loop a hair inside the edge's middle, for `carrier`.
    material:Vec<bool> }

impl Profile {
    fn new(edges: Vec<Edge>) -> Result<Self,String> {
        let walls = edges.iter().map(Wall::of).collect::<Result<Vec<_>,_>>().map_err(failure)?;
        if walls.iter().all(Option::is_none) { return Err("material profile has no wall".into()); }
        let reach = edges.iter().map(Edge::reach).fold(0_f64,f64::max);
        let boxes = Boxes::new(&edges,reach);
        let mut profile = Self {edges,walls,reach,boxes,material:Vec::new()};
        let eps = 1e-6*(1.+reach);
        profile.material = profile.edges.iter().map(|e| {
            let m = e.at(0.5);
            let probe = match *e {
                Edge::Line {a,b,..} => {
                    let d = [b[0]-a[0],b[1]-a[1]];
                    let l = (d[0]*d[0]+d[1]*d[1]).sqrt().max(f64::MIN_POSITIVE);
                    [m[0]-d[1]/l*eps,m[1]+d[0]/l*eps]
                }
                Edge::Arc {center,..} => {
                    let r = distance(m,center).max(f64::MIN_POSITIVE);
                    [m[0]+(center[0]-m[0])/r*eps,m[1]+(center[1]-m[1])/r*eps]
                }
            };
            profile.value(probe) < 0.
        }).collect();
        Ok(profile)
    }
    pub(super) fn reach(&self) -> f64 { self.reach }

    pub(super) fn edge_count(&self) -> usize { self.edges.len() }

    /// `value`, and the edge nearest the point, which decides it.
    pub(super) fn value_edge(&self,p: P) -> (f64,usize) {
        let (nearest,edge) = self.boxes.nearest(&self.edges,p);
        let inside = self.boxes.crossings(&self.edges,p)%2 == 1;
        (if inside { -nearest } else { nearest },if edge == usize::MAX { 0 } else { edge })
    }

    /// Edge `j`'s carrier, its whole line or circle, negative on the loop's material side of it
    /// (`Profile::material`).
    pub(super) fn carrier(&self,p: P,j: usize) -> f64 {
        let material = self.material[j];
        match self.edges[j] {
            Edge::Line {a,b,..} => {
                let d = [b[0]-a[0],b[1]-a[1]];
                let l = (d[0]*d[0]+d[1]*d[1]).sqrt().max(f64::MIN_POSITIVE);
                let left = (d[0]*(p[1]-a[1])-d[1]*(p[0]-a[0]))/l;
                if material { -left } else { left }
            }
            Edge::Arc {center,radius,..} => {
                let ring = distance(p,center)-radius;
                if material { ring } else { -ring }
            }
        }
    }

    /// The signed boundary distance at a point, in plain floating point.
    pub(super) fn value(&self,p: P) -> f64 { self.value_edge(p).0 }

    pub(super) fn bounds(&self,p: [I;2]) -> Result<I,Error> {
        let nearest = self.boxes.bounds(&self.walls,p)?.expect("a profile keeps at least one wall");
        let [lo,hi] = nearest.bounds();
        // Clear of every wall, a connected box lies on one side of the loop.
        if lo > 0. {
            let centre = p.map(|v| { let [a,b] = v.bounds(); a*0.5+b*0.5 });
            let inside = self.boxes.crossings(&self.edges,centre)%2 == 1;
            return Ok(if inside { nearest.neg() } else { nearest });
        }
        I::new(-hi,hi)
    }
}

impl PlanarField {
    /// Any simple loop of lines and circular arcs, in any winding: the exact
    /// convex construction where it applies, the signed boundary distance elsewhere.
    pub(crate) fn from_loop(edges: &[Edge],join: f64) -> Result<Self,String> {
        let edges = ordered(edges,join)?;
        let (direction,is_convex) = turning(&edges)?;
        if is_convex { return convex(&edges,direction); }
        let profile = Profile::new(edges)?;
        Ok(Self {node:Node::Profile(profile),depth:1,uses:[true;2]})
    }

    /// One outer loop with every further loop subtracted as a hole.
    pub(crate) fn from_loops(loops: &[Vec<Edge>],join: f64) -> Result<Self,String> {
        let mut loops = loops.iter();
        let mut field = Self::from_loop(loops.next().ok_or("empty material profile")?,join)?;
        for hole in loops { field = field.difference(Self::from_loop(hole,join)?).map_err(failure)?; }
        Ok(field)
    }
}

impl RevolvedRegion {
    /// Convert the solved profile to an interval-evaluable material field. Each
    /// outer/hole loop is made of lines and circular arcs; hole loops are
    /// subtracted, regardless of winding. Axis edges are no wall.
    /// The field represents this snapshot; source-solve and axis-snapping error
    /// are separate from interval evaluation and are not certified by this conversion.
    pub fn field(&self) -> Result<RevolvedField,String> {
        RevolvedField::new(PlanarField::from_loops(&self.loops,self.axis_tolerance)?,self.origin,self.axis)
            .map_err(failure)
    }
}
