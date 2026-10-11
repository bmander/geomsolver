//! The motions a surface is carried onto itself by, and the plane of its orbits. Every analytic
//! surface here is a curve swept by a one-parameter group of rigid motions: turned about an axis
//! (a revolution, cone, sphere, torus), slid along a direction (an extrusion), or both (a cylinder,
//! a plane). Two surfaces one motion carries onto themselves meet in orbits of it — circles about
//! the axis, lines along the slide — through the points where their sections cross in the plane
//! of orbits: a meridian half-plane `(ρ, z)` for a turn, the plane square to a slide. So two
//! surfaces sharing a motion meet in closed form wherever their sections do (`ssi::intersect`),
//! and are one surface where their sections are one curve (`ssi::same`).
//!
//! A screw (a turn advancing along its axis) is the group between the two, whose orbits are
//! helices; no surface kind is swept by one yet.
#[allow(unused_imports)]
use crate::fmath::Det;
use super::geom::{Curve,Frame,Surface,V};
use crate::space::{add,cross,dot,norm,scale,sub};
use std::sync::Arc;

/// A one-parameter group of rigid motions, and the coordinates of the plane of its orbits.
#[derive(Clone,Copy,Debug,PartialEq)]
pub enum Twist {
    /// Turns about the axis through `o` along `z`: a point's orbit is a circle about the axis, its
    /// section `(ρ, h)`, distance from the axis and height along it from `o`.
    Turn(Frame),
    /// Slides along `z`: a point's orbit is a line along `z`, its section `(x, y)` in the plane
    /// through `o` square to `z`.
    Slide(Frame),
}

/// A surface's section in the plane of a twist's orbits, in its coordinates.
#[derive(Clone,Debug)]
pub enum Section {
    /// The line through `p` along the unit `d`, both ways.
    Line { p: [f64;2],d: [f64;2] },
    /// The circle about `c` of radius `r`.
    Circle { c: [f64;2],r: f64 },
    /// A B-spline swept by the twist, read in its section through the twist's coordinates.
    Spline(Arc<super::nurbs::BSpline>),
}

impl Twist {
    pub fn frame(&self) -> &Frame { match self { Twist::Turn(f) | Twist::Slide(f) => f } }
    /// Where `p`'s orbit crosses the plane of orbits.
    pub fn section(&self,p: V) -> [f64;2] {
        match self {
            Twist::Turn(f) => {
                let q = sub(p,f.o);
                let h = dot(q,f.z);
                [norm(sub(q,scale(f.z,h))),h]
            }
            Twist::Slide(f) => { let q = sub(p,f.o); [dot(q,f.x),dot(q,f.y)] }
        }
    }
    /// Whether the plane of orbits holds `q` (`ρ ≥ −tol` for a turn).
    pub fn reaches(&self,q: [f64;2],tol: f64) -> bool { !matches!(self,Twist::Turn(_)) || q[0] >= -tol }
    /// The orbit through the section's point `q`: a circle about the axis (none within `tol` of
    /// it, where the orbit is a point), or a line along the slide.
    pub fn orbit(&self,[a,b]: [f64;2],tol: f64) -> Option<Curve> {
        match self {
            Twist::Turn(f) => (a > tol).then(|| Curve::Circle(Frame::about(add(f.o,scale(f.z,b)),f.z),a)),
            Twist::Slide(f) => Some(Curve::Line {p:add(f.o,add(scale(f.x,a),scale(f.y,b))),d:f.z}),
        }
    }
}

impl Section {
    /// A signed distance in the plane of orbits whose zero set is this section (not a spline's).
    pub fn implicit(&self,[x,y]: [f64;2]) -> f64 {
        match *self {
            Section::Line {p,d} => (x-p[0])*d[1]-(y-p[1])*d[0],
            Section::Circle {c,r} => (x-c[0]).dhypot(y-c[1])-r,
            Section::Spline(..) => unreachable!("a spline section has no closed-form distance"),
        }
    }
}

impl Surface {
    /// The twists this surface alone fixes and is carried onto itself by: turns about its axis,
    /// slides along its generators. A plane and a sphere are carried by more than they fix, so
    /// they lend a twist only against another surface (`shared`).
    pub fn twists(&self) -> Vec<Twist> {
        let f = *self.frame();
        match self {
            Surface::Cylinder(..) => vec![Twist::Turn(f),Twist::Slide(f)],
            Surface::Cone(..) | Surface::Torus(..) | Surface::Revolution(..) => vec![Twist::Turn(f)],
            Surface::Extrusion(..) => vec![Twist::Slide(f)],
            Surface::Plane(_) | Surface::Sphere(..) | Surface::Blend(..) | Surface::BSpline(..) => vec![],
        }
    }
    /// This surface's section in the plane of `t`'s orbits, where `t` carries it onto itself and
    /// the section has a form here; none otherwise.
    pub fn section(&self,t: &Twist,tol: f64) -> Option<Section> {
        let (f,z) = (self.frame(),t.frame().z);
        let aligned = norm(cross(f.z,z)) <= 1e-12;
        match (t,self) {
            (Twist::Turn(a),_) => {
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
            (Twist::Slide(_),Surface::Plane(_)) => (dot(f.z,z).abs() <= 1e-12).then(|| {
                let w = cross(z,f.z);
                let w = scale(w,1./norm(w));
                let [p,q] = [f.o,add(f.o,w)].map(|x| t.section(x));
                Section::Line {p,d:[q[0]-p[0],q[1]-p[1]]}
            }),
            (Twist::Slide(_),Surface::Cylinder(_,r)) => aligned.then(|| Section::Circle {c:t.section(f.o),r:*r}),
            (Twist::Slide(_),Surface::Extrusion(_,c)) if aligned => match &**c {
                Curve::BSpline(b) => Some(Section::Spline(b.clone())),
                _ => None,
            },
            (Twist::Slide(_),_) => None,
        }
    }
}

/// The twists that carry both surfaces onto themselves, either's own first, then any one lends
/// the other: a plane's normal through a sphere's centre, the line through two spheres' centres.
/// Each comes with both sections.
pub fn shared(a: &Surface,b: &Surface,tol: f64) -> Vec<(Twist,Section,Section)> {
    let mut twists = a.twists();
    twists.extend(b.twists());
    let lent = |s: &Surface,t: &Surface| match (s,t) {
        (Surface::Plane(p),Surface::Sphere(c,_)) => Some(Twist::Turn(Frame::about(c.o,p.z))),
        (Surface::Sphere(c,_),Surface::Sphere(d,_)) => {
            let w = sub(d.o,c.o);
            (norm(w) > 0.).then(|| Twist::Turn(Frame::about(c.o,w)))
        }
        _ => None,
    };
    twists.extend(lent(a,b).or_else(|| lent(b,a)));
    twists.into_iter().filter_map(|t| Some((t,a.section(&t,tol)?,b.section(&t,tol)?))).collect()
}
