//! The boundary of a swept tool as faces and edges, by surface family. The
//! language builds solids bounded by planes, cylinders, cones, spheres and
//! tori: revolved faces, and the planar and extruded faces of a prism. Each
//! family has an exact contact equation along one station of the face under
//! an instantaneous motion, which is what the tracer in `sweep_candidates`
//! walks; an edge is a shared boundary curve of two faces, charted on both.
use super::{RevolvedSurface,surface::contact::sinusoid_roots};
use crate::{envelope::{self,Error,Motion,SurfacePoint},plane::{cross,dot,scaled}};

type V3 = [f64;3];
fn add(a: V3,b: V3) -> V3 { std::array::from_fn(|k| a[k]+b[k]) }
fn sub(a: V3,b: V3) -> V3 { std::array::from_fn(|k| a[k]-b[k]) }

/// A straight or circular edge of a planar loop, in the plane's own
/// coordinates: what a prism's profile is made of.
#[derive(Clone,Copy,Debug)]
pub enum PlanarEdge {
    Line { start: [f64;2], end: [f64;2] },
    /// From `start` angle through `sweep` (signed) about `center`.
    Arc { center: [f64;2], radius: f64, start: f64, sweep: f64 },
}

impl PlanarEdge {
    /// The point at fraction `t` along the edge.
    pub fn at(&self,t: f64) -> [f64;2] {
        match *self {
            Self::Line {start,end} => [start[0]+t*(end[0]-start[0]),start[1]+t*(end[1]-start[1])],
            Self::Arc {center,radius,start,sweep} => {
                let a = start+t*sweep;
                [center[0]+radius*a.cos(),center[1]+radius*a.sin()]
            }
        }
    }

    /// Crossings of the ray from `p` along +x, for point-in-loop tests.
    fn crossings(&self,p: [f64;2]) -> usize {
        match *self {
            Self::Line {start,end} => {
                if (start[1] > p[1]) == (end[1] > p[1]) { return 0; }
                let x = start[0]+(p[1]-start[1])/(end[1]-start[1])*(end[0]-start[0]);
                usize::from(x > p[0])
            }
            Self::Arc {center,radius,start,sweep} => {
                let dy = p[1]-center[1];
                if dy.abs() >= radius { return 0; }
                let dx = (radius*radius-dy*dy).sqrt();
                let mut n = 0;
                for x in [center[0]-dx,center[0]+dx] {
                    if x <= p[0] { continue; }
                    let angle = (p[1]-center[1]).atan2(x-center[0]);
                    let mut t = (angle-start)/sweep;
                    // the angle is the same modulo a turn
                    let turn = std::f64::consts::TAU/sweep.abs();
                    while t < 0. { t += turn; }
                    while t > turn { t -= turn; }
                    if t <= 1. { n += 1; }
                }
                n
            }
        }
    }
}

/// A closed loop of planar edges in loop order, with the box it lies in.
#[derive(Clone,Debug)]
pub struct PlanarLoop {
    pub edges: Vec<PlanarEdge>,
    pub lo: [f64;2],
    pub hi: [f64;2],
}

impl PlanarLoop {
    pub fn new(edges: Vec<PlanarEdge>) -> Self {
        let (mut lo,mut hi) = ([f64::INFINITY;2],[f64::NEG_INFINITY;2]);
        for e in &edges {
            for i in 0..=16 {
                let p = e.at(i as f64/16.);
                for k in 0..2 { lo[k] = lo[k].min(p[k]); hi[k] = hi[k].max(p[k]); }
            }
            if let PlanarEdge::Arc {center,radius,..} = *e {
                // an arc may bulge past its sampled points; its circle bounds it
                for k in 0..2 { lo[k] = lo[k].min(center[k]-radius); hi[k] = hi[k].max(center[k]+radius); }
            }
        }
        Self {edges,lo,hi}
    }

    pub fn contains(&self,p: [f64;2]) -> bool {
        self.edges.iter().map(|e| e.crossings(p)).sum::<usize>()%2 == 1
    }
}

/// A planar face: a loop in the plane through `origin` spanned by the
/// orthonormal `u` and `v`, parametrized over the loop's box.
#[derive(Clone,Debug)]
pub struct PlanarFace {
    pub origin: V3,
    pub u: V3,
    pub v: V3,
    pub outer: PlanarLoop,
    pub holes: Vec<PlanarLoop>,
}

impl PlanarFace {
    fn coords(&self,u: f64,v: f64) -> [f64;2] {
        let (lo,hi) = (self.outer.lo,self.outer.hi);
        [lo[0]+u*(hi[0]-lo[0]),lo[1]+v*(hi[1]-lo[1])]
    }
    /// The plane point at in-plane coordinates `(a, b)`.
    pub fn lift(&self,a: f64,b: f64) -> V3 {
        add(self.origin,add(scaled(self.u,a),scaled(self.v,b)))
    }
}

/// A profile arc extruded along `direction`: a cylinder side. `u` runs along
/// the extrusion over `span`, `v` around the arc from `start` through `sweep`.
#[derive(Clone,Debug)]
pub struct ExtrudedFace {
    pub center: V3,
    pub a: V3,
    pub b: V3,
    pub radius: f64,
    pub start: f64,
    pub sweep: f64,
    pub direction: V3,
    pub span: [f64;2],
}

/// One face of the tool's boundary, in the tool's own frame.
#[derive(Clone,Debug)]
pub enum ToolFace {
    /// A profile edge revolved about an axis: `u` along the meridian, `v` the
    /// fraction of the revolution.
    Revolved(RevolvedSurface),
    Planar(PlanarFace),
    Extruded(ExtrudedFace),
}

/// The contact condition `n·v = 0` along one station of a face, with the
/// station's own chart in `v`.
#[derive(Clone,Copy,Debug)]
pub enum StationEquation {
    /// `a cos θ + b sin θ + c = 0`, `θ = v·sweep`, over `domain` in `v`.
    Sinusoid { a: f64, b: f64, c: f64, sweep: f64, domain: [f64;2] },
    /// `c + slope·(v − ½) = 0` over `v` in `[0, 1]`.
    Affine { c: f64, slope: f64 },
}

impl StationEquation {
    /// The amplitude of the equation's varying part and its constant, so that
    /// a root exists exactly when `|c| <= amplitude`: a zero amplitude makes
    /// the whole station stationary or nowhere in contact.
    pub fn amplitude_and_constant(&self) -> (f64,f64) {
        match *self {
            Self::Sinusoid {a,b,c,..} => (a.hypot(b),c),
            Self::Affine {c,slope} => (0.5*slope.abs(),c),
        }
    }

    /// Isolated roots as `(branch, v)`.
    pub fn roots(&self,tolerance: f64) -> Result<Vec<(usize,f64)>,Error> {
        match *self {
            Self::Sinusoid {a,b,c,sweep,domain} => sinusoid_roots(a,b,c,sweep,domain,tolerance),
            Self::Affine {c,slope} => {
                if !c.is_finite() || !slope.is_finite() { return Err(Error::NonFinite); }
                if 0.5*slope.abs() <= tolerance { return Err(Error::Degenerate); }
                let v = 0.5-c/slope;
                Ok(if (0. ..=1.).contains(&v) { vec![(0,v)] } else { Vec::new() })
            }
        }
    }
}

impl ToolFace {
    pub fn at(&self,u: f64,v: f64) -> Result<SurfacePoint,Error> {
        if !u.is_finite() || !v.is_finite() { return Err(Error::NonFinite); }
        match self {
            Self::Revolved(s) => s.at(u,v),
            Self::Planar(f) => {
                if !(0. ..=1.).contains(&u) || !(0. ..=1.).contains(&v) { return Err(Error::OutsideDomain); }
                let [a,b] = f.coords(u,v);
                let (lo,hi) = (f.outer.lo,f.outer.hi);
                Ok(SurfacePoint {position:f.lift(a,b),du:scaled(f.u,hi[0]-lo[0]),dv:scaled(f.v,hi[1]-lo[1])})
            }
            Self::Extruded(f) => {
                if !(0. ..=1.).contains(&u) || !(0. ..=1.).contains(&v) { return Err(Error::OutsideDomain); }
                let h = f.span[0]+u*(f.span[1]-f.span[0]);
                let angle = f.start+v*f.sweep;
                let radial = add(scaled(f.a,angle.cos()),scaled(f.b,angle.sin()));
                let tangent = add(scaled(f.a,-angle.sin()),scaled(f.b,angle.cos()));
                Ok(SurfacePoint {
                    position:add(add(f.center,scaled(f.direction,h)),scaled(radial,f.radius)),
                    du:scaled(f.direction,f.span[1]-f.span[0]),
                    dv:scaled(tangent,f.radius*f.sweep),
                })
            }
        }
    }

    /// The face's own domain in `(u, v)`.
    pub fn domain(&self) -> [[f64;2];2] {
        match self { Self::Revolved(s) => s.domain(),Self::Planar(_) | Self::Extruded(_) => [[0.,1.],[0.,1.]] }
    }

    /// Whether `(u, v)` is on the face proper: inside a planar face's loop and
    /// outside its holes; everywhere in the domain for the other families.
    pub fn contains(&self,u: f64,v: f64) -> bool {
        match self {
            Self::Planar(f) => {
                let p = f.coords(u,v);
                f.outer.contains(p) && !f.holes.iter().any(|h| h.contains(p))
            }
            _ => true,
        }
    }

    /// The contact equation along the station at `u` under `motion`.
    /// `Degenerate` where the station has no extent (a pole).
    pub fn station(&self,u: f64,motion: Motion) -> Result<StationEquation,Error> {
        // the velocity field pulled back into the tool's frame, affine in the point
        let inverse = motion.inverse();
        let velocity = |p: V3| inverse.vector(motion.velocity(p));
        match self {
            Self::Revolved(s) => {
                let (a,b,c) = s.contact_coefficients(u,motion)?;
                let [_,domain] = s.domain();
                Ok(StationEquation::Sinusoid {a,b,c,sweep:s.sweep(),domain})
            }
            Self::Planar(f) => {
                if !(0. ..=1.).contains(&u) { return Err(Error::OutsideDomain); }
                let n = cross(f.u,f.v);
                let (p0,p1) = (self.at(u,0.)?.position,self.at(u,1.)?.position);
                let (n0,n1) = (dot(n,velocity(p0)),dot(n,velocity(p1)));
                Ok(StationEquation::Affine {c:0.5*(n0+n1),slope:n1-n0})
            }
            Self::Extruded(f) => {
                if !(0. ..=1.).contains(&u) { return Err(Error::OutsideDomain); }
                // n·v(p) = cos θ (a·V) + sin θ (b·V) with V the velocity of the
                // station's centre: the skew part of the field drops out of a ring.
                let h = f.span[0]+u*(f.span[1]-f.span[0]);
                let centre = velocity(add(f.center,scaled(f.direction,h)));
                Ok(StationEquation::Sinusoid {a:dot(f.a,centre),b:dot(f.b,centre),c:0.,sweep:f.sweep,domain:[0.,1.]})
            }
        }
    }

    /// The outward-oriented unit normal at `(u, v)` given the face's sign.
    pub fn normal(&self,u: f64,v: f64,sign: f64) -> Result<(V3,V3),Error> {
        let s = self.at(u,v)?;
        let n = envelope::contact(s,Motion::identity())?.normal;
        Ok((s.position,n.map(|x| x*sign)))
    }

    /// Where a meridian end lies on the revolution axis the station there is a
    /// pole every branch meets: its position and the axis direction, which is
    /// the tangent plane's normal there.
    pub fn pole(&self,end: f64,scale: f64) -> Result<Option<(V3,V3)>,Error> {
        match self {
            Self::Revolved(s) => {
                let p = s.at(end,0.)?;
                let dv = p.dv[0].hypot(p.dv[1]).hypot(p.dv[2]);
                Ok((dv <= scale*1e-9).then(|| (p.position,s.axis_direction())))
            }
            _ => Ok(None),
        }
    }

    /// A point inside the face away from its boundary, for probing its side.
    pub fn interior(&self) -> Result<(f64,f64),Error> {
        match self {
            Self::Planar(f) => {
                // walk a grid of the box until a point is inside the loop
                for n in [4,8,16,32,64] {
                    for i in 1..n { for j in 1..n {
                        let (u,v) = (i as f64/n as f64,j as f64/n as f64);
                        if self.contains(u,v) {
                            // and not too near an edge: its neighbours are inside too
                            let d = 0.5/n as f64;
                            if [(u-d,v),(u+d,v),(u,v-d),(u,v+d)].iter().all(|&(a,b)| self.contains(a,b)) { return Ok((u,v)); }
                        }
                    } }
                }
                let _ = f; Err(Error::Degenerate)
            }
            _ => { let [_,d] = self.domain(); Ok((0.5,d[0]+0.25*(d[1]-d[0]))) }
        }
    }
}

/// Where along a face's boundary an edge runs: the chart from the edge's
/// parameter `t` in `[0, 1]` to the face's `(u, v)`.
#[derive(Clone,Copy,Debug)]
pub enum EdgeChart {
    /// `u` fixed, `v` running over the face's `v` domain.
    FixedU(f64),
    /// `v` fixed, `u` running over `[0, 1]`.
    FixedV(f64),
    /// Along the `k`th edge of a planar face's outer loop.
    Loop(usize),
    /// Along the `k`th edge of the `h`th hole of a planar face.
    Hole(usize,usize),
}

impl EdgeChart {
    pub fn at(self,t: f64,face: &ToolFace) -> (f64,f64) {
        let domain = face.domain();
        match self {
            Self::FixedU(u) => (u,domain[1][0]+t*(domain[1][1]-domain[1][0])),
            Self::FixedV(v) => (t,v),
            Self::Loop(_) | Self::Hole(..) => {
                let ToolFace::Planar(f) = face else { return (t,0.) };
                let p = self.planar_edge(f).at(t);
                let (lo,hi) = (f.outer.lo,f.outer.hi);
                ((p[0]-lo[0])/(hi[0]-lo[0]),(p[1]-lo[1])/(hi[1]-lo[1]))
            }
        }
    }

    /// The direction along the edge at `t`, from the face's own tangents.
    pub fn tangent(self,t: f64,face: &ToolFace,point: &SurfacePoint) -> V3 {
        match self {
            Self::FixedU(_) => point.dv,
            Self::FixedV(_) => point.du,
            Self::Loop(_) | Self::Hole(..) => {
                let ToolFace::Planar(f) = face else { return point.du };
                let e = self.planar_edge(f);
                let (p0,p1) = (e.at((t-1e-6).max(0.)),e.at((t+1e-6).min(1.)));
                sub(f.lift(p1[0],p1[1]),f.lift(p0[0],p0[1]))
            }
        }
    }

    fn planar_edge(self,f: &PlanarFace) -> PlanarEdge {
        match self {
            Self::Loop(k) => f.outer.edges[k],
            Self::Hole(h,k) => f.holes[h].edges[k],
            _ => unreachable!("a fixed chart has no planar edge"),
        }
    }
}

/// A boundary curve shared by two faces, charted on each so both incident
/// normals are read at one parameter.
#[derive(Clone,Copy,Debug)]
pub struct ToolEdge {
    pub faces: [usize;2],
    pub charts: [EdgeChart;2],
}

impl ToolFace {
    /// The face carried by a rigid pose.
    pub fn placed(&self,pose: Motion) -> Self {
        match self {
            Self::Revolved(s) => Self::Revolved(s.placed(pose)),
            Self::Planar(f) => Self::Planar(PlanarFace {
                origin:pose.point(f.origin),u:pose.vector(f.u),v:pose.vector(f.v),
                outer:f.outer.clone(),holes:f.holes.clone(),
            }),
            Self::Extruded(f) => Self::Extruded(ExtrudedFace {
                center:pose.point(f.center),a:pose.vector(f.a),b:pose.vector(f.b),
                direction:pose.vector(f.direction),..f.clone()
            }),
        }
    }
}

impl PlanarEdge {
    fn reversed(self) -> Self {
        match self {
            Self::Line {start,end} => Self::Line {start:end,end:start},
            Self::Arc {center,radius,start,sweep} => Self::Arc {center,radius,start:start+sweep,sweep:-sweep},
        }
    }
    fn ends(self) -> [[f64;2];2] { [self.at(0.),self.at(1.)] }
}

/// The edges of a loop walked head to tail from the first, each turned to
/// run the way the walk arrives at it.
fn ordered(edges: Vec<PlanarEdge>,tolerance: f64) -> Result<Vec<PlanarEdge>,String> {
    let mut remaining = edges;
    if remaining.is_empty() { return Err("an empty profile loop".into()); }
    let mut out = vec![remaining.remove(0)];
    while !remaining.is_empty() {
        let tail = out.last().unwrap().ends()[1];
        let near = |p: [f64;2]| (p[0]-tail[0]).hypot(p[1]-tail[1]) <= tolerance;
        let Some((i,reverse)) = remaining.iter().enumerate().find_map(|(i,e)| {
            let [a,b] = e.ends();
            if near(a) { Some((i,false)) } else if near(b) { Some((i,true)) } else { None }
        }) else { return Err("a profile loop's edges do not meet".into()); };
        let e = remaining.remove(i);
        out.push(if reverse { e.reversed() } else { e });
    }
    Ok(out)
}

/// The faces and edges of a prism read from its solved profile: two planar
/// caps at the extent's ends, a planar side per straight edge and an extruded
/// side per round edge of every loop, and the edges between them charted
/// consistently, so an edge's parameter runs the same way on both faces.
pub(super) fn prism_faces(sk: &crate::model::Sketch,solid: usize) -> Result<(Vec<ToolFace>,Vec<ToolEdge>),String> {
    use crate::{model::{EntKind,SolidDef},plane::{self,Basis}};
    let sol = sk.solids.get(solid).ok_or("no such solid")?;
    let SolidDef::Prism {face,from,to} = &sol.def else {
        return Err("a swept tool's prism needs its own extent (`from:`/`to:` or `depth:`)".into());
    };
    let (lo,hi) = (from.value.min(to.value),from.value.max(to.value));
    if !(lo.is_finite() && hi.is_finite()) || hi <= lo { return Err("a prism tool needs a finite positive extent".into()); }
    let face = sk.faces.get(*face as usize).ok_or("no such profile")?;
    let (basis,c,s,o) = if let Some(p) = face.plane()? {
        let p = sk.planes.get(p as usize).ok_or("no profile plane")?;
        (p.basis,sk.params[p.frame.c as usize].value,sk.params[p.frame.s as usize].value,sk.point_xy(p.frame.origin as usize))
    } else { (Basis::page(),1.,0.,(0.,0.)) };
    let n = basis.normal();
    let datum_angle = s.atan2(c);
    let view = |p: (f64,f64)| { let q = plane::in_view(c,s,o,p); [q.0,q.1] };
    let lift = |a: f64,b: f64,h: f64| add(basis.lift(a,b),scaled(n,h));
    let edge_of = |e: &crate::model::EntRef| -> Result<PlanarEdge,String> {
        Ok(match e.kind {
            EntKind::Line => {
                let l = sk.lines.get(e.i()).ok_or("no such profile line")?;
                PlanarEdge::Line {start:view(sk.point_xy(l.p1 as usize)),end:view(sk.point_xy(l.p2 as usize))}
            }
            EntKind::Arc => {
                let a = sk.arcs.get(e.i()).ok_or("no such profile arc")?;
                let (start,end) = sk.arc_angles(e.i());
                PlanarEdge::Arc {center:view(sk.point_xy(a.center as usize)),radius:sk.params[a.radius as usize].value,
                    start:start-datum_angle,sweep:end-start}
            }
            EntKind::Circle => {
                let k = sk.circles.get(e.i()).ok_or("no such profile circle")?;
                PlanarEdge::Arc {center:view(sk.point_xy(k.center as usize)),radius:sk.params[k.radius as usize].value,
                    start:0.,sweep:std::f64::consts::TAU}
            }
            _ => return Err("a prism tool's profile is made of lines, arcs and circles".into()),
        })
    };
    let scale = face.boundaries().flat_map(|(edges,_)| edges.iter()).filter_map(|e| edge_of(e).ok())
        .flat_map(|e| e.ends()).map(|p| p[0].hypot(p[1])).fold(1_f64,f64::max);
    let mut loops = Vec::new();
    for (edges,_) in face.boundaries() {
        loops.push(ordered(edges.iter().map(edge_of).collect::<Result<Vec<_>,_>>()?,1e-9*scale)?);
    }
    let outer = PlanarLoop::new(loops[0].clone());
    let holes: Vec<PlanarLoop> = loops[1..].iter().map(|l| PlanarLoop::new(l.clone())).collect();
    let mut faces = Vec::new();
    let mut edges = Vec::new();
    for h in [lo,hi] {
        faces.push(ToolFace::Planar(PlanarFace {origin:lift(0.,0.,h),u:basis.u,v:basis.v,outer:outer.clone(),holes:holes.clone()}));
    }
    for (l,edges_2d) in loops.iter().enumerate() {
        let first = faces.len();
        let count = edges_2d.len();
        let chart = |k: usize| if l == 0 { EdgeChart::Loop(k) } else { EdgeChart::Hole(l-1,k) };
        for (k,e) in edges_2d.iter().enumerate() {
            let side = match *e {
                PlanarEdge::Line {start,end} => {
                    let (p0,p1) = (lift(start[0],start[1],0.),lift(end[0],end[1],0.));
                    let length = plane::norm(sub(p1,p0));
                    let u = scaled(sub(p1,p0),1./length);
                    let rect = PlanarLoop::new(vec![
                        PlanarEdge::Line {start:[0.,lo],end:[length,lo]},PlanarEdge::Line {start:[length,lo],end:[length,hi]},
                        PlanarEdge::Line {start:[length,hi],end:[0.,hi]},PlanarEdge::Line {start:[0.,hi],end:[0.,lo]}]);
                    ToolFace::Planar(PlanarFace {origin:p0,u,v:n,outer:rect,holes:Vec::new()})
                }
                PlanarEdge::Arc {center,radius,start,sweep} => ToolFace::Extruded(ExtrudedFace {
                    center:lift(center[0],center[1],0.),a:basis.u,b:basis.v,radius,start,sweep,direction:n,span:[lo,hi]}),
            };
            let (bottom,top) = match side {
                ToolFace::Planar(_) => (EdgeChart::FixedV(0.),EdgeChart::FixedV(1.)),
                _ => (EdgeChart::FixedU(0.),EdgeChart::FixedU(1.)),
            };
            let index = faces.len();
            faces.push(side);
            edges.push(ToolEdge {faces:[0,index],charts:[chart(k),bottom]});
            edges.push(ToolEdge {faces:[1,index],charts:[chart(k),top]});
        }
        // the edges along the extrusion at each vertex, unless the loop is one full circle
        let full_circle = count == 1 && matches!(edges_2d[0],PlanarEdge::Arc {sweep,..} if (sweep.abs()-std::f64::consts::TAU).abs() < 1e-12);
        if full_circle { continue; }
        for k in 0..count {
            let (a,b) = (first+k,first+(k+1)%count);
            let end = |f: &ToolFace| match f { ToolFace::Planar(_) => EdgeChart::FixedU(1.),_ => EdgeChart::FixedV(1.) };
            let start = |f: &ToolFace| match f { ToolFace::Planar(_) => EdgeChart::FixedU(0.),_ => EdgeChart::FixedV(0.) };
            edges.push(ToolEdge {faces:[a,b],charts:[end(&faces[a]),start(&faces[b])]});
        }
    }
    Ok((faces,edges))
}
