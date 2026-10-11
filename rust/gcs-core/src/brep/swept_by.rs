//! How a surface is swept, and its section. Every analytic surface here is a curve swept by a
//! **turn** about an axis (a revolution, cone, sphere, torus) or an **extrusion** along a direction
//! (an `Extrusion`); a cylinder and a plane are swept by both. As it is swept, each point of the
//! curve runs along a path — a circle about the axis, or a line along the extrusion — and the
//! surface is its **section** swept: the meridian half-plane `(ρ, z)` of a turn, the plane square to
//! an extrusion. Two surfaces swept by the same turn or extrusion meet in the paths through the
//! points where their sections cross, in closed form wherever the sections cross so
//! (`ssi::intersect`), and are one surface where their sections are one curve (`ssi::same`).
//!
//! A screw (a turn advancing along its axis) would sweep helices; no surface kind is swept by one
//! yet.
#[allow(unused_imports)]
use crate::fmath::Det;
use super::geom::{Curve,Frame,Surface,V};
use crate::space::{add,cross,dot,norm,scale,sub};
use std::sync::Arc;

/// What sweeps a surface, and the coordinates of its section plane.
#[derive(Clone,Copy,Debug,PartialEq)]
pub enum SweptBy {
    /// A turn about the axis through `o` along `z`: a point's path is a circle about the axis, its
    /// place in the section `(ρ, h)`, distance from the axis and height along it from `o`.
    Turn(Frame),
    /// An extrusion along `z`: a point's path is a line along `z`, its place in the section
    /// `(x, y)` in the plane through `o` square to `z`.
    Extrusion(Frame),
}

/// A surface's section, in its section plane's coordinates.
#[derive(Clone,Debug)]
pub enum Section {
    /// The line through `p` along the unit `d`, both ways.
    Line { p: [f64;2],d: [f64;2] },
    /// The circle about `c` of radius `r`.
    Circle { c: [f64;2],r: f64 },
    /// A B-spline that is swept, read in the section plane through `SweptBy::in_section`.
    Spline(Arc<super::nurbs::BSpline>),
}

impl SweptBy {
    pub fn frame(&self) -> &Frame { match self { SweptBy::Turn(f) | SweptBy::Extrusion(f) => f } }
    /// Where `p`'s path crosses the section plane.
    pub fn in_section(&self,p: V) -> [f64;2] {
        match self {
            SweptBy::Turn(f) => {
                let q = sub(p,f.o);
                let h = dot(q,f.z);
                [norm(sub(q,scale(f.z,h))),h]
            }
            SweptBy::Extrusion(f) => { let q = sub(p,f.o); [dot(q,f.x),dot(q,f.y)] }
        }
    }
    /// Whether the section plane holds `q` (`ρ ≥ −tol` for a turn).
    pub fn holds(&self,q: [f64;2],tol: f64) -> bool { !matches!(self,SweptBy::Turn(_)) || q[0] >= -tol }
    /// The path the section's point `q` is swept along: a circle about the axis (none within `tol`
    /// of it, where the path is a point), or a line along the extrusion.
    pub fn path(&self,[a,b]: [f64;2],tol: f64) -> Option<Curve> {
        match self {
            SweptBy::Turn(f) => (a > tol).then(|| Curve::Circle(Frame::about(add(f.o,scale(f.z,b)),f.z),a)),
            SweptBy::Extrusion(f) => Some(Curve::Line {p:add(f.o,add(scale(f.x,a),scale(f.y,b))),d:f.z}),
        }
    }
}

impl Section {
    /// A signed distance in the section plane whose zero set is this section (not a spline's).
    pub fn implicit(&self,[x,y]: [f64;2]) -> f64 {
        match *self {
            Section::Line {p,d} => (x-p[0])*d[1]-(y-p[1])*d[0],
            Section::Circle {c,r} => (x-c[0]).dhypot(y-c[1])-r,
            Section::Spline(..) => unreachable!("a spline section has no closed-form distance"),
        }
    }
}

impl Surface {
    /// What sweeps this surface that the surface alone places: the turn about its axis, the
    /// extrusion along its generators. A plane and a sphere are swept by more than they place, so
    /// they lend a turn only against another surface (`shared`).
    pub fn swept_by(&self) -> Vec<SweptBy> {
        let f = *self.frame();
        match self {
            Surface::Cylinder(..) => vec![SweptBy::Turn(f),SweptBy::Extrusion(f)],
            Surface::Cone(..) | Surface::Torus(..) | Surface::Revolution(..) => vec![SweptBy::Turn(f)],
            Surface::Extrusion(..) => vec![SweptBy::Extrusion(f)],
            Surface::Plane(_) | Surface::Sphere(..) | Surface::Blend(..) | Surface::BSpline(..) => vec![],
        }
    }
    /// This surface's section in `by`'s section plane, where `by` sweeps it and the section has a
    /// form here; none otherwise.
    pub fn section(&self,by: &SweptBy,tol: f64) -> Option<Section> {
        let (f,z) = (self.frame(),by.frame().z);
        let aligned = norm(cross(f.z,z)) <= 1e-12;
        match (by,self) {
            (SweptBy::Turn(a),_) => {
                let height = |p: V| dot(sub(p,a.o),z);
                let on_axis = |p: V| norm(cross(sub(p,a.o),z)) <= tol;
                match *self {
                    Surface::Plane(_) => aligned.then(|| Section::Line {p:[0.,height(f.o)],d:[1.,0.]}),
                    Surface::Sphere(_,r) => on_axis(f.o).then(|| Section::Circle {c:[0.,height(f.o)],r}),
                    _ if !(aligned && on_axis(f.o)) => None,
                    Surface::Cylinder(_,r) => Some(Section::Line {p:[r,height(f.o)],d:[0.,1.]}),
                    // (r + v sin α, v cos α) in the cone's own (ρ, z); its z may run against the axis
                    Surface::Cone(_,r,al) => Some(Section::Line {p:[r,height(f.o)],d:[al.dsin(),al.dcos()*dot(f.z,z).signum()]}),
                    Surface::Torus(_,big,r) => Some(Section::Circle {c:[big,height(f.o)],r}),
                    Surface::Revolution(_,ref c) => match &**c { Curve::BSpline(b) => Some(Section::Spline(b.clone())),_ => None },
                    Surface::Extrusion(..) | Surface::Blend(..) | Surface::BSpline(..) => None,
                }
            }
            (SweptBy::Extrusion(_),Surface::Plane(_)) => (dot(f.z,z).abs() <= 1e-12).then(|| {
                let w = cross(z,f.z);
                let w = scale(w,1./norm(w));
                let [p,q] = [f.o,add(f.o,w)].map(|x| by.in_section(x));
                Section::Line {p,d:[q[0]-p[0],q[1]-p[1]]}
            }),
            (SweptBy::Extrusion(_),Surface::Cylinder(_,r)) => aligned.then(|| Section::Circle {c:by.in_section(f.o),r:*r}),
            (SweptBy::Extrusion(_),Surface::Extrusion(_,c)) if aligned => match &**c {
                Curve::BSpline(b) => Some(Section::Spline(b.clone())),
                _ => None,
            },
            (SweptBy::Extrusion(_),_) => None,
        }
    }
}

/// What sweeps both surfaces, either's own first, then a turn one lends the other: about a plane's
/// normal through a sphere's centre, about the line through two spheres' centres. Each comes with
/// both sections.
pub fn shared(a: &Surface,b: &Surface,tol: f64) -> Vec<(SweptBy,Section,Section)> {
    let mut by = a.swept_by();
    by.extend(b.swept_by());
    let lent = |s: &Surface,t: &Surface| match (s,t) {
        (Surface::Plane(p),Surface::Sphere(c,_)) => Some(SweptBy::Turn(Frame::about(c.o,p.z))),
        (Surface::Sphere(c,_),Surface::Sphere(d,_)) => {
            let w = sub(d.o,c.o);
            (norm(w) > 0.).then(|| SweptBy::Turn(Frame::about(c.o,w)))
        }
        _ => None,
    };
    by.extend(lent(a,b).or_else(|| lent(b,a)));
    by.into_iter().filter_map(|t| Some((t,a.section(&t,tol)?,b.section(&t,tol)?))).collect()
}
