//! What a sweep reads its tool and its blank by where no document hands it a field: a tool's
//! material from its plain-data description (`tool::Tool`) in closed form — a revolution's by its
//! profile in a half-plane through its axis (`surface::RevolvedRegion`), a prism's by its profile
//! and its two caps, a union's the least and an intersection's the greatest of its members', a
//! placement's its tool's read back through it — and a blank's inside and its signed distance read
//! off its boundary (`Blank`).
#[allow(unused_imports)]
use crate::util::fmath::Det;
use super::envelope::Motion;
use super::surface::{Meridian,RevolvedRegion,RevolvedSurface};
use super::tool::{Material,Tool};
use crate::build::{walks,Seg};
use crate::geom::{Curve,Frame};
use crate::planar::{Region,Seg as Step};
use crate::query::{Located,Place};
use crate::topo::Brep;
use crate::util::space::{add,dot,scale,sub,unit};
use std::f64::consts::TAU;
use std::sync::Arc;

type V = [f64;3];

/// A tool's material in closed form: its signed distance where that is a revolution's or a
/// square prism's, a bound on it otherwise (a union's, an intersection's, a drafted prism's) —
/// never more than the distance, its sign always right, so a ball of a radius it reads clear of
/// the tool is clear of it.
#[derive(Debug)]
enum Shape {
    Revolved(RevolvedRegion),
    /// A prism: its plane's frame, its profile's region in that frame's (x, y), its extent along
    /// the frame's z, and `tan draft` (each section moved that much into the region a unit away
    /// from its plane).
    Prism { frame: Frame,region: Region,extent: [f64;2],slope: f64 },
    Union(Vec<Shape>),
    Intersection(Vec<Shape>),
    /// Its shape read back through a placement's inverse.
    Placed(Motion,Box<Shape>),
}

impl Shape {
    fn value(&self,p: V) -> f64 {
        match self {
            Shape::Revolved(r) => r.classify(p,0.).map_or(f64::INFINITY,|s| s.signed_distance),
            Shape::Prism {frame,region,extent,slope} => {
                let l = frame.local(p);
                let q = [l[0],l[1]];
                let near = region.loops.iter().flatten().map(|s| s.seg.distance(q)).fold(f64::INFINITY,f64::min);
                let across = if region.contains(q) { -near } else { near }+slope*l[2].abs();
                let along = (extent[0]-l[2]).max(l[2]-extent[1]);
                let d = if across <= 0. && along <= 0. { across.max(along) } else { across.max(0.).hypot(along.max(0.)) };
                d/slope.hypot(1.)
            }
            Shape::Union(shapes) => shapes.iter().map(|s| s.value(p)).fold(f64::INFINITY,f64::min),
            Shape::Intersection(shapes) => shapes.iter().map(|s| s.value(p)).fold(f64::NEG_INFINITY,f64::max),
            Shape::Placed(m,shape) => shape.value(m.inverse().point(p)),
        }
    }
}

/// A tool's material, its `Shape` (`Tool::material`).
#[derive(Debug)]
struct Closed(Shape);

impl Material for Closed {
    fn value(&self,p: V) -> f64 { self.0.value(p) }
}

/// A walk's step in a plane's (x, y): a line, or an arc counter-clockwise about the plane's normal
/// or against it.
fn step(s: &Seg,frame: &Frame) -> Result<Step,String> {
    let at = |q: V| { let l = frame.local(q); [l[0],l[1]] };
    match &s.curve {
        Curve::Line {..} => Ok(Step::Line {a:at(s.start()),b:at(s.end())}),
        Curve::Circle(f,r) => {
            let c = at(f.o);
            let q = at(s.start());
            let a0 = (q[1]-c[1]).datan2(q[0]-c[0]);
            let turn = dot(f.z,frame.z).signum();
            Ok(Step::Arc {c,r:*r,a0,sweep:(s.t[1]-s.t[0])*turn})
        }
        _ => Err("a tool's profile is lines and arcs: a spline's sweep is not read".into()),
    }
}

fn shape(tool: &Tool) -> Result<Shape,String> {
    Ok(match tool {
        Tool::Revolved {name,profile,origin,axis} => {
            let a = unit(*axis);
            let normal = unit(profile.normal);
            let plane = Frame::about(profile.origin,normal);
            let coords = |q: V| { let l = plane.local(q); [l[0],l[1]] };
            // every step a face of the turn, a line on the axis too (the region marks it so), and
            // an arc short of a circle with its ends
            let loops = walks(profile,&coords)?.iter().map(|walk| walk.iter().map(|s| {
                let (meridian,ends) = match &s.curve {
                    Curve::Line {..} => (Meridian::Line {start:s.start(),delta:sub(s.end(),s.start())},None),
                    Curve::Circle(f,r) => {
                        let (t0,t1) = (s.t[0],s.t[1]);
                        let (sn,cs) = t0.dsin_cos();
                        let b = scale(add(scale(f.x,-r*sn),scale(f.y,r*cs)),(t1-t0).signum());
                        let whole = (t1-t0).abs() >= TAU*(1.-1e-12);
                        (Meridian::Round {center:f.o,a:add(scale(f.x,r*cs),scale(f.y,r*sn)),b,sweep:(t1-t0).abs()},
                            (!whole).then(|| [s.start(),s.end()]))
                    }
                    _ => return Err("a tool's profile is lines and arcs: a spline's sweep is not read".to_string()),
                };
                Ok((RevolvedSurface::new(name.clone(),meridian,*origin,a,TAU),ends))
            }).collect::<Result<Vec<_>,String>>()).collect::<Result<Vec<_>,_>>()?;
            Shape::Revolved(RevolvedRegion::of_profile(loops,normal,super::AXIS_TOLERANCE).map_err(|e| format!("`{name}`: {e}"))?)
        }
        Tool::Prism {name,profile,from,to,draft} => {
            let frame = Frame::about(profile.origin,unit(profile.normal));
            let coords = |q: V| { let l = frame.local(q); [l[0],l[1]] };
            let loops = walks(profile,&coords).map_err(|e| format!("`{name}`: {e}"))?.iter()
                .map(|walk| walk.iter().map(|s| step(s,&frame)).collect::<Result<Vec<_>,_>>()).collect::<Result<Vec<_>,_>>()?;
            Shape::Prism {frame,region:Region::plain(loops),extent:[from.min(*to),from.max(*to)],slope:draft.map_or(0.,|d| d.dtan())}
        }
        Tool::Union(tools) => Shape::Union(tools.iter().map(shape).collect::<Result<_,_>>()?),
        Tool::Intersection(tools) => Shape::Intersection(tools.iter().map(shape).collect::<Result<_,_>>()?),
        Tool::Placed(m,tool) => Shape::Placed(*m,Box::new(shape(tool)?)),
        Tool::Read(_) => return Err("a tool of faces a reader made has no material of its own: its reader's".into()),
    })
}

impl Tool {
    /// The tool's material in closed form (`Shape`): its signed distance, or a bound on it never
    /// more than the distance, its sign always right.
    pub fn material(&self) -> Result<Arc<dyn Material>,String> { Ok(Arc::new(Closed(shape(self)?))) }
}

/// **A blank read off its boundary**: whether a point is in it — farther in than `margin`, as a
/// sweep's rows ask — and its signed distance from the boundary, negative inside, read off a
/// mesh within a measured sag of it (good to that sag, which is all a sheet's fit asks of it).
pub struct Blank {
    located: Located<'static>,
    triangles: Vec<[V;3]>,
    boxes: crate::util::bvh::Bvh<3>,
    sag: f64,
}

impl Blank {
    /// `brep` as a blank: a point within `margin` of its boundary is not in it.
    pub fn new(brep: &Brep,margin: f64) -> Result<Blank,String> {
        let size = brep.size();
        let m = crate::mesh::mesh(brep,size*1e-4,0.2)?;
        let triangles: Vec<[V;3]> = m.tris.iter().map(|t| t.map(|i| m.pts[i as usize])).collect();
        let grow = 1e-9*(1.+size);
        let boxes = crate::util::bvh::Bvh::new(triangles.iter().map(|t| crate::util::bvh::Bounds {
            lo:std::array::from_fn(|a| t[0][a].min(t[1][a]).min(t[2][a])-grow),
            hi:std::array::from_fn(|a| t[0][a].max(t[1][a]).max(t[2][a])+grow),
        }));
        Ok(Blank {located:Located::owned(brep.clone(),margin),triangles,boxes,sag:m.sag})
    }

    /// Whether `p` is in the blank, farther than its margin from the boundary.
    pub fn inside(&self,p: V) -> bool { self.located.solid_place(p) == Place::In }

    /// The signed distance from the boundary at `p`, negative inside: to the mesh, within its sag.
    pub fn distance(&self,p: V) -> f64 {
        let d2 = std::cell::Cell::new(f64::INFINITY);
        self.boxes.query_nearest(|b| b.gap(crate::util::bvh::Bounds {lo:p,hi:p}),|| d2.get().sqrt(),
            |k| d2.set(d2.get().min(crate::query::triangle_distance2(p,&self.triangles[k]))));
        let d = d2.get().sqrt();
        match self.located.solid_place(p) { Place::In => -d,Place::Out => d,Place::On => 0. }
    }

    /// How far the mesh the distance is read off lies from the boundary at most.
    pub fn sag(&self) -> f64 { self.sag }

    /// The boundary.
    pub fn brep(&self) -> &Brep { &self.located.b }

    /// The box the blank lies in.
    pub fn bounds(&self) -> ([f64;3],[f64;3]) { self.located.b.bounds() }
}
