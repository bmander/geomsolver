//! A file's geometry read into the kernel's: a curve in space (`geom::Curve`), a curve in a
//! face's parameters, and a surface (`geom::Surface`), each scaled from the file's units
//! (`context::Context`) and placed as the kernel places it. What the kernel has no curve or
//! surface for is refused naming the entity (`Refusal::unsupported`).
//!
//! **Nothing is refitted.** A curve or a surface is the one the file gives, its numbers scaled
//! into millimetres and radians: a B-spline's poles and weights and knots, a conic's placement and
//! radii. Where the kernel holds the same surface placed another way it is placed so exactly: a
//! linear extrusion's curve is carried along its direction into the plane square to it (an affine
//! map of its poles, so the surface is the same set of points), a revolution's frame is turned so
//! its curve lies in the half-plane of its `x`. A surface the kernel holds no exact form of — an
//! extrusion of a conic slanted to its plane, a revolution of a curve off its axis's plane — is
//! refused, never approximated.
//!
//! **A parameter's rate.** The kernel reads a line by its length and a conic by its angle in
//! radians; the file reads a line by multiples of its vector's magnitude in the file's unit and a
//! conic by its angle in the file's unit. Each curve read carries `rate`, the kernel's parameter
//! per the file's, so a curve in a face's parameters written over the file's parameter of an edge
//! can be read over the kernel's (`Spatial::rate`).
#[allow(unused_imports)]
use crate::util::fmath::Det;
use super::context::Context;
use super::Refusal;
use crate::geom::{Curve,Frame,Surface,V};
use crate::nurbs::{BSpline,Net};
use crate::step_file::{File,Value};
use crate::util::space::{add,cross,dot,norm,normalised,scale,sub};
use std::sync::Arc;

/// A file's geometry, read in one representation's context.
pub struct Geometry<'f,'a> { pub file: &'f File<'a>,pub context: Context }

/// A curve in space as the kernel holds it, in millimetres, and the kernel's parameter per one of
/// the file's along it: 1 for a B-spline, the vector's magnitude in millimetres for a line, the
/// angle unit in radians for a conic.
#[derive(Clone,Debug)]
pub struct Spatial { pub curve: Curve,pub rate: f64 }

/// A surface as the kernel holds it, in millimetres and radians, and the kernel's parameters per
/// the file's along each (`scale`): a length's the unit of length, an angle's the unit of angle, a
/// cone's along its generator the unit of length over the cosine of its half-angle (the file reads
/// it along the axis), a spline's 1. Where the kernel's parameters are no such scaling of the
/// file's (an extrusion whose curve was carried into the plane square to it) a curve in the face's
/// parameters read through it lies off the edge, and is not used (`topology::file_pcurve`).
#[derive(Clone,Debug)]
pub struct Sheet { pub surface: Surface,pub scale: [f64;2] }

/// A curve in a face's parameters as the file writes it, over the file's own parameter and in
/// the file's parameters of the surface: a line `p + t d` (`d` carrying its magnitude), or a
/// conic or a B-spline held as the kernel's curve with `z = 0`.
#[derive(Clone,Debug)]
pub enum Flat { Line { p: [f64;2],d: [f64;2] },Curve(Curve) }

impl Flat {
    /// The point at the file's parameter `t`.
    pub fn at(&self,t: f64) -> [f64;2] {
        match self { Flat::Line {p,d} => [p[0]+t*d[0],p[1]+t*d[1]],Flat::Curve(c) => { let q = c.point(t); [q[0],q[1]] } }
    }
    /// The same curve with its parameters scaled by `s`, the file's to the kernel's (its own
    /// parameter kept): none for a conic scaled unevenly.
    pub fn scaled(&self,s: [f64;2]) -> Option<Flat> {
        let p = |q: V| [q[0]*s[0],q[1]*s[1],0.];
        Some(match self {
            Flat::Line {p:o,d} => Flat::Line {p:[o[0]*s[0],o[1]*s[1]],d:[d[0]*s[0],d[1]*s[1]]},
            Flat::Curve(Curve::BSpline(b)) => Flat::Curve(Curve::BSpline(Arc::new(BSpline {poles:b.poles.iter().map(|&q| p(q)).collect(),..(**b).clone()}))),
            Flat::Curve(Curve::Circle(f,r)) if s[0] == s[1] => Flat::Curve(Curve::Circle(Frame {o:p(f.o),..*f},r*s[0])),
            Flat::Curve(Curve::Ellipse(f,a,b)) if s[0] == s[1] => Flat::Curve(Curve::Ellipse(Frame {o:p(f.o),..*f},a*s[0],b*s[0])),
            Flat::Curve(_) => return None,
        })
    }
    /// Where its parameter is bounded (a B-spline's domain), if it is.
    pub fn domain(&self) -> Option<[f64;2]> { match self { Flat::Curve(Curve::BSpline(b)) => Some(b.domain()),_ => None } }
}

/// A parameter's number, finite.
pub fn number(v: Option<&Value>,from: u32,what: &str) -> Result<f64,Refusal> {
    v.and_then(Value::number).filter(|x| x.is_finite()).ok_or_else(|| Refusal::malformed(from,format!("{what}: a number expected")))
}

/// A parameter's list.
pub fn list<'v,'a>(v: Option<&'v Value<'a>>,from: u32,what: &str) -> Result<&'v [Value<'a>],Refusal> {
    v.and_then(Value::list).ok_or_else(|| Refusal::malformed(from,format!("{what}: a list expected")))
}

/// A parameter's logical, `.T.` or `.F.`.
pub fn logical(v: Option<&Value>,from: u32,what: &str) -> Result<bool,Refusal> {
    v.and_then(Value::logical).ok_or_else(|| Refusal::malformed(from,format!("{what}: .T. or .F. expected")))
}

/// A parameter's integer.
fn integer(v: Option<&Value>,from: u32,what: &str) -> Result<i64,Refusal> {
    v.and_then(Value::int).ok_or_else(|| Refusal::malformed(from,format!("{what}: an integer expected")))
}

/// A parameter that is a reference, or unset (`$`).
fn optional(v: Option<&Value>,from: u32,what: &str) -> Result<Option<u32>,Refusal> {
    match v {
        Some(Value::Unset) => Ok(None),
        Some(Value::Ref(r)) => Ok(Some(*r)),
        _ => Err(Refusal::malformed(from,format!("{what}: a reference or `$` expected"))),
    }
}

/// A knot vector from its distinct knots and their multiplicities.
fn knots(mults: &[Value],values: &[Value],from: u32) -> Result<Vec<f64>,Refusal> {
    if mults.len() != values.len() { return Err(Refusal::malformed(from,"as many multiplicities as knots expected")) }
    let mut out = Vec::new();
    for (m,k) in mults.iter().zip(values) {
        let m = integer(Some(m),from,"a knot's multiplicity")?;
        let k = number(Some(k),from,"a knot")?;
        if !(1..=64).contains(&m) { return Err(Refusal::malformed(from,format!("a knot's multiplicity of {m}"))) }
        out.extend(std::iter::repeat_n(k,m as usize));
    }
    Ok(out)
}

impl<'f,'a> Geometry<'f,'a> {
    /// The kinds `#r` is (a complex entity's parts joined by `+`).
    pub fn kinds(&self,r: u32) -> String { self.file.get(r).parts.iter().map(|p| p.0).collect::<Vec<_>>().join("+") }
    /// The simple entity of one of `kinds` that `from` refers to as `what`: its number, kind and
    /// parameters; refused naming `from` where it is of another kind.
    pub fn of(&self,from: u32,r: Option<u32>,kinds: &[&str],what: &str) -> Result<(u32,&'a str,&'f [Value<'a>]),Refusal> {
        let r = r.ok_or_else(|| Refusal::malformed(from,format!("{what}: a reference expected")))?;
        match self.file.get(r).simple() {
            Some((n,v)) if kinds.contains(&n) => Ok((r,n,v)),
            _ => Err(Refusal::malformed(from,format!("{what}: #{r} is {}, not {}",self.kinds(r),kinds.join(" or ")))),
        }
    }
    /// The `n` numbers of a point or a direction `#r` (`kind`).
    fn coordinates(&self,from: u32,r: Option<u32>,kind: &str,n: usize) -> Result<(u32,Vec<f64>),Refusal> {
        let (r,_,v) = self.of(from,r,&[kind],&format!("a {}",kind.to_lowercase().replace('_'," ")))?;
        let c = list(v.get(1),r,"its coordinates")?;
        if c.len() != n { return Err(Refusal::malformed(r,format!("{} coordinates where {n} are read",c.len()))) }
        Ok((r,c.iter().map(|x| number(Some(x),r,"a coordinate")).collect::<Result<_,_>>()?))
    }
    /// A point in space, in millimetres.
    pub fn point(&self,from: u32,r: Option<u32>) -> Result<V,Refusal> {
        let (_,c) = self.coordinates(from,r,"CARTESIAN_POINT",3)?;
        Ok([c[0],c[1],c[2]].map(|x| x*self.context.length))
    }
    /// A point in a surface's parameters, as the file writes it.
    fn flat_point(&self,from: u32,r: Option<u32>) -> Result<[f64;2],Refusal> {
        let (_,c) = self.coordinates(from,r,"CARTESIAN_POINT",2)?;
        Ok([c[0],c[1]])
    }
    /// A unit direction in space.
    pub fn direction(&self,from: u32,r: Option<u32>) -> Result<V,Refusal> {
        let (r,c) = self.coordinates(from,r,"DIRECTION",3)?;
        normalised([c[0],c[1],c[2]]).ok_or_else(|| Refusal::malformed(r,"a direction of no length"))
    }
    /// A unit direction in a surface's parameters.
    fn flat_direction(&self,from: u32,r: Option<u32>) -> Result<[f64;2],Refusal> {
        let (r,c) = self.coordinates(from,r,"DIRECTION",2)?;
        let l = c[0].dhypot(c[1]);
        if !(l > 0.) { return Err(Refusal::malformed(r,"a direction of no length")) }
        Ok([c[0]/l,c[1]/l])
    }
    /// A vector: its unit direction, and its magnitude as written.
    fn vector(&self,from: u32,r: Option<u32>,flat: bool) -> Result<(Vec<f64>,f64),Refusal> {
        let (r,_,v) = self.of(from,r,&["VECTOR"],"a vector")?;
        let d = if flat { self.flat_direction(r,v.get(1).and_then(Value::reference))?.to_vec() }
            else { self.direction(r,v.get(1).and_then(Value::reference))?.to_vec() };
        let m = number(v.get(2),r,"a vector's magnitude")?;
        if !(m > 0.) { return Err(Refusal::malformed(r,format!("a vector's magnitude of {m}"))) }
        Ok((d,m))
    }
    /// An `AXIS2_PLACEMENT_3D` as a right-handed frame (the standard's axes are right-handed by
    /// construction, `y = z × x`): its axis `[0, 0, 1]` and its reference direction the standard's
    /// default (`[1, 0, 0]`, or `[0, 0, 1]` about that axis) where unset, the reference made
    /// square to the axis; refused where it lies along the axis.
    pub fn placement(&self,from: u32,r: Option<u32>) -> Result<Frame,Refusal> {
        let (r,_,v) = self.of(from,r,&["AXIS2_PLACEMENT_3D"],"a placement")?;
        if v.len() != 4 { return Err(Refusal::malformed(r,"a placement of four parameters expected")) }
        let o = self.point(r,v[1].reference())?;
        let z = match optional(v.get(2),r,"a placement's axis")? { Some(a) => self.direction(r,Some(a))?,None => [0.,0.,1.] };
        let x = match optional(v.get(3),r,"a placement's reference direction")? {
            Some(a) => self.direction(r,Some(a))?,
            None => if z == [1.,0.,0.] { [0.,0.,1.] } else { [1.,0.,0.] },
        };
        if !(norm(sub(x,scale(z,dot(x,z)))) > 1e-12) { return Err(Refusal::malformed(r,"a placement whose reference direction lies along its axis")) }
        Ok(Frame::new(o,z,x))
    }
    /// An `AXIS2_PLACEMENT_2D` as a frame in the plane `z = 0`, in the file's parameters.
    fn flat_placement(&self,from: u32,r: Option<u32>) -> Result<Frame,Refusal> {
        let (r,_,v) = self.of(from,r,&["AXIS2_PLACEMENT_2D"],"a placement in a surface's parameters")?;
        let [x,y] = self.flat_point(r,v.get(1).and_then(Value::reference))?;
        let [a,b] = match optional(v.get(2),r,"a placement's reference direction")? { Some(d) => self.flat_direction(r,Some(d))?,None => [1.,0.] };
        Ok(Frame {o:[x,y,0.],x:[a,b,0.],y:[-b,a,0.],z:[0.,0.,1.]})
    }
    /// A positive length, scaled.
    fn length(&self,v: Option<&Value>,from: u32,what: &str) -> Result<f64,Refusal> {
        let x = number(v,from,what)?;
        if !(x > 0.) { return Err(Refusal::malformed(from,format!("{what} of {x}"))) }
        Ok(x*self.context.length)
    }

    /// A curve in space.
    pub fn curve(&self,from: u32,r: Option<u32>) -> Result<Spatial,Refusal> {
        let r = r.ok_or_else(|| Refusal::malformed(from,"a curve: a reference expected"))?;
        let e = self.file.get(r);
        if e.is("B_SPLINE_CURVE") || e.is("B_SPLINE_CURVE_WITH_KNOTS") {
            return Ok(Spatial {curve:Curve::BSpline(Arc::new(self.spline(r,false)?)),rate:1.})
        }
        let Some((kind,v)) = e.simple() else { return Err(Refusal::unsupported(r,format!("a curve of the kinds {}",self.kinds(r)))) };
        let (l,a) = (self.context.length,self.context.angle);
        Ok(match kind {
            "LINE" => {
                let p = self.point(r,v.get(1).and_then(Value::reference))?;
                let (d,m) = self.vector(r,v.get(2).and_then(Value::reference),false)?;
                Spatial {curve:Curve::Line {p,d:[d[0],d[1],d[2]]},rate:m*l}
            }
            "CIRCLE" => {
                let f = self.placement(r,v.get(1).and_then(Value::reference))?;
                Spatial {curve:Curve::Circle(f,self.length(v.get(2),r,"a circle's radius")?),rate:a}
            }
            "ELLIPSE" => {
                let f = self.placement(r,v.get(1).and_then(Value::reference))?;
                let (s1,s2) = (self.length(v.get(2),r,"an ellipse's semi-axis")?,self.length(v.get(3),r,"an ellipse's semi-axis")?);
                Spatial {curve:Curve::Ellipse(f,s1,s2),rate:a}
            }
            "POLYLINE" => Spatial {curve:Curve::BSpline(Arc::new(self.polyline(r,v,false)?)),rate:1.},
            // trimmed by its edge's vertices, not by its own
            "TRIMMED_CURVE" => self.curve(r,v.get(1).and_then(Value::reference))?,
            _ => return Err(Refusal::unsupported(r,format!("a curve of the kind {kind}"))),
        })
    }
    /// A curve in a surface's parameters, as the file writes it (`Flat`).
    pub fn flat(&self,from: u32,r: Option<u32>) -> Result<Flat,Refusal> {
        let r = r.ok_or_else(|| Refusal::malformed(from,"a curve: a reference expected"))?;
        let e = self.file.get(r);
        if e.is("B_SPLINE_CURVE") || e.is("B_SPLINE_CURVE_WITH_KNOTS") { return Ok(Flat::Curve(Curve::BSpline(Arc::new(self.spline(r,true)?)))) }
        let Some((kind,v)) = e.simple() else { return Err(Refusal::unsupported(r,format!("a curve of the kinds {}",self.kinds(r)))) };
        Ok(match kind {
            "LINE" => {
                let p = self.flat_point(r,v.get(1).and_then(Value::reference))?;
                let (d,m) = self.vector(r,v.get(2).and_then(Value::reference),true)?;
                Flat::Line {p,d:[d[0]*m,d[1]*m]}
            }
            "CIRCLE" => {
                let f = self.flat_placement(r,v.get(1).and_then(Value::reference))?;
                let rad = number(v.get(2),r,"a circle's radius")?;
                if !(rad > 0.) { return Err(Refusal::malformed(r,format!("a circle's radius of {rad}"))) }
                Flat::Curve(Curve::Circle(f,rad))
            }
            "ELLIPSE" => {
                let f = self.flat_placement(r,v.get(1).and_then(Value::reference))?;
                let (s1,s2) = (number(v.get(2),r,"an ellipse's semi-axis")?,number(v.get(3),r,"an ellipse's semi-axis")?);
                if !(s1 > 0. && s2 > 0.) { return Err(Refusal::malformed(r,"an ellipse's semi-axes must be positive")) }
                Flat::Curve(Curve::Ellipse(f,s1,s2))
            }
            "POLYLINE" => Flat::Curve(Curve::BSpline(Arc::new(self.polyline(r,v,true)?))),
            "TRIMMED_CURVE" => self.flat(r,v.get(1).and_then(Value::reference))?,
            _ => return Err(Refusal::unsupported(r,format!("a curve of the kind {kind}"))),
        })
    }
    /// A pole: a point in space, or one in a surface's parameters (`flat`) as `[u, v, 0]`.
    fn pole(&self,from: u32,r: Option<u32>,flat: bool) -> Result<V,Refusal> {
        if flat { let [u,v] = self.flat_point(from,r)?; Ok([u,v,0.]) } else { self.point(from,r) }
    }
    /// A polyline as the degree-one B-spline through its points, its parameter the standard's (the
    /// `i`th segment over `[i - 1, i]`).
    fn polyline(&self,r: u32,v: &[Value],flat: bool) -> Result<BSpline,Refusal> {
        let points = list(v.get(1),r,"a polyline's points")?;
        let poles: Vec<V> = points.iter().map(|p| self.pole(r,p.reference(),flat)).collect::<Result<_,_>>()?;
        let n = poles.len();
        if n < 2 { return Err(Refusal::malformed(r,"a polyline of fewer than two points")) }
        let mut knots = vec![0.];
        knots.extend((0..n).map(|k| k as f64));
        knots.push((n-1) as f64);
        BSpline::new(1,knots,poles).map_err(|m| Refusal::malformed(r,m))
    }
    /// A B-spline curve, simple or the complex entity with its weights; in space, or in a surface's
    /// parameters (`flat`).
    fn spline(&self,r: u32,flat: bool) -> Result<BSpline,Refusal> {
        let e = self.file.get(r);
        let (head,ks,weights) = match e.simple() {
            Some(("B_SPLINE_CURVE_WITH_KNOTS",v)) => {
                if v.len() != 9 { return Err(Refusal::malformed(r,"a B-spline curve of nine parameters expected")) }
                (&v[1..6],&v[6..8],None)
            }
            Some((kind,_)) => return Err(Refusal::unsupported(r,format!("a curve of the kind {kind}"))),
            None => match (e.part("B_SPLINE_CURVE"),e.part("B_SPLINE_CURVE_WITH_KNOTS")) {
                (Some(s),Some(k)) if s.len() == 5 && k.len() == 3 => (s,&k[..2],e.part("RATIONAL_B_SPLINE_CURVE")),
                (Some(_),None) => return Err(Refusal::unsupported(r,format!("a B-spline curve of the kinds {}",self.kinds(r)))),
                _ => return Err(Refusal::malformed(r,"a B-spline curve whose parts do not read")),
            },
        };
        let degree = integer(head.first(),r,"a B-spline's degree")?;
        if !(1..=crate::nurbs::MAX_DEGREE as i64).contains(&degree) { return Err(Refusal::unsupported(r,format!("a B-spline of degree {degree}"))) }
        let poles: Vec<V> = list(head.get(1),r,"a B-spline's poles")?.iter().map(|p| self.pole(r,p.reference(),flat)).collect::<Result<_,_>>()?;
        let knots = knots(list(ks.first(),r,"a B-spline's multiplicities")?,list(ks.get(1),r,"a B-spline's knots")?,r)?;
        let s = match weights {
            Some(w) => {
                let w = list(w.first(),r,"a B-spline's weights")?.iter().map(|x| number(Some(x),r,"a weight")).collect::<Result<Vec<_>,_>>()?;
                BSpline::rational(degree as usize,knots,poles,w)
            }
            None => BSpline::new(degree as usize,knots,poles),
        };
        s.map_err(|m| Refusal::malformed(r,m))
    }

    /// A surface.
    pub fn surface(&self,from: u32,r: Option<u32>) -> Result<Sheet,Refusal> {
        let r = r.ok_or_else(|| Refusal::malformed(from,"a surface: a reference expected"))?;
        let e = self.file.get(r);
        if e.is("B_SPLINE_SURFACE") || e.is("B_SPLINE_SURFACE_WITH_KNOTS") {
            return Ok(Sheet {surface:Surface::BSpline(Frame::WORLD,Arc::new(self.net(r)?)),scale:[1.,1.]})
        }
        let Some((kind,v)) = e.simple() else { return Err(Refusal::unsupported(r,format!("a surface of the kinds {}",self.kinds(r)))) };
        let (l,a) = (self.context.length,self.context.angle);
        let place = || self.placement(r,v.get(1).and_then(Value::reference));
        let (surface,scale) = match kind {
            "PLANE" => (Surface::Plane(place()?),[l,l]),
            "CYLINDRICAL_SURFACE" => (Surface::Cylinder(place()?,self.length(v.get(2),r,"a cylinder's radius")?),[a,l]),
            "CONICAL_SURFACE" => {
                let rad = number(v.get(2),r,"a cone's radius")?;
                let half = number(v.get(3),r,"a cone's semi-angle")?*a;
                if !(rad >= 0.) { return Err(Refusal::malformed(r,format!("a cone's radius of {rad}"))) }
                if !(half > 0. && half < std::f64::consts::FRAC_PI_2) { return Err(Refusal::malformed(r,format!("a cone's semi-angle of {half} rad"))) }
                (Surface::Cone(place()?,rad*l,half),[a,l/half.dcos()])
            }
            "SPHERICAL_SURFACE" => (Surface::Sphere(place()?,self.length(v.get(2),r,"a sphere's radius")?),[a,a]),
            "TOROIDAL_SURFACE" => (Surface::Torus(place()?,number(v.get(2),r,"a torus's major radius")?*l,
                self.length(v.get(3),r,"a torus's minor radius")?),[a,a]),
            "SURFACE_OF_LINEAR_EXTRUSION" => self.extrusion(r,v)?,
            "SURFACE_OF_REVOLUTION" => self.revolution(r,v)?,
            _ => return Err(Refusal::unsupported(r,format!("a surface of the kind {kind}"))),
        };
        Ok(Sheet {surface,scale})
    }
    /// A B-spline surface, simple or the complex entity with its weights.
    fn net(&self,r: u32) -> Result<Net,Refusal> {
        let e = self.file.get(r);
        let (head,ks,weights) = match e.simple() {
            Some(("B_SPLINE_SURFACE_WITH_KNOTS",v)) => {
                if v.len() != 13 { return Err(Refusal::malformed(r,"a B-spline surface of thirteen parameters expected")) }
                (&v[1..8],&v[8..12],None)
            }
            Some((kind,_)) => return Err(Refusal::unsupported(r,format!("a surface of the kind {kind}"))),
            None => match (e.part("B_SPLINE_SURFACE"),e.part("B_SPLINE_SURFACE_WITH_KNOTS")) {
                (Some(s),Some(k)) if s.len() == 7 && k.len() == 5 => (s,&k[..4],e.part("RATIONAL_B_SPLINE_SURFACE")),
                (Some(_),None) => return Err(Refusal::unsupported(r,format!("a B-spline surface of the kinds {}",self.kinds(r)))),
                _ => return Err(Refusal::malformed(r,"a B-spline surface whose parts do not read")),
            },
        };
        let degree = |k: usize| -> Result<usize,Refusal> {
            let d = integer(head.get(k),r,"a B-spline's degree")?;
            if !(1..=crate::nurbs::MAX_DEGREE as i64).contains(&d) { return Err(Refusal::unsupported(r,format!("a B-spline surface of degree {d}"))) }
            Ok(d as usize)
        };
        let (du,dv) = (degree(0)?,degree(1)?);
        let poles: Vec<Vec<V>> = list(head.get(2),r,"a B-spline surface's poles")?.iter().map(|row| {
            list(Some(row),r,"a row of poles")?.iter().map(|p| self.point(r,p.reference())).collect()
        }).collect::<Result<_,_>>()?;
        let uknots = knots(list(ks.first(),r,"multiplicities")?,list(ks.get(2),r,"knots")?,r)?;
        let vknots = knots(list(ks.get(1),r,"multiplicities")?,list(ks.get(3),r,"knots")?,r)?;
        let weights = match weights {
            Some(w) => Some(list(w.first(),r,"a B-spline surface's weights")?.iter().map(|row| {
                list(Some(row),r,"a row of weights")?.iter().map(|x| number(Some(x),r,"a weight")).collect()
            }).collect::<Result<Vec<Vec<f64>>,_>>()?),
            None => None,
        };
        let net = Net {du,dv,uknots,vknots,poles,weights};
        net.check().map_err(|m| Refusal::malformed(r,m))?;
        Ok(net)
    }
    /// A linear extrusion, `C(u) + v z`: its curve carried along `z` into the plane square to it
    /// through the curve's first point — the same surface, its curve an affine image (a spline's
    /// poles, a line's point and direction); a conic slanted to `z` is refused (its image is an
    /// ellipse of another frame, not read yet).
    fn extrusion(&self,r: u32,v: &[Value]) -> Result<(Surface,[f64;2]),Refusal> {
        let swept = self.curve(r,v.get(1).and_then(Value::reference))?;
        let (d,m) = self.vector(r,v.get(2).and_then(Value::reference),false)?;
        let rates = [swept.rate,m*self.context.length];
        let z = [d[0],d[1],d[2]];
        let o = swept.curve.point(match &swept.curve { Curve::BSpline(b) => b.domain()[0],_ => 0. });
        let flat = |p: V| sub(p,scale(z,dot(sub(p,o),z)));
        let square = |f: &Frame| norm(cross(f.z,z)) <= 1e-12;
        let curve = match swept.curve {
            Curve::BSpline(b) => Curve::BSpline(Arc::new(BSpline {poles:b.poles.iter().map(|&p| flat(p)).collect(),..(*b).clone()})),
            Curve::Line {p,d} => {
                let d = normalised(sub(d,scale(z,dot(d,z)))).ok_or_else(|| Refusal::malformed(r,"an extrusion of a line along itself"))?;
                Curve::Line {p:flat(p),d}
            }
            Curve::Circle(f,rad) if square(&f) => Curve::Circle(Frame {o:flat(f.o),..f},rad),
            Curve::Ellipse(f,s1,s2) if square(&f) => Curve::Ellipse(Frame {o:flat(f.o),..f},s1,s2),
            other => return Err(Refusal::unsupported(r,format!("an extrusion of a {} slanted to its direction",other.kind()))),
        };
        Ok((Surface::Extrusion(Frame::about(o,z),Arc::new(curve)),rates))
    }
    /// A surface of revolution, its curve turned about the axis: the frame's `x` toward the curve,
    /// which must lie in the half-plane of the axis it points to (the kernel turns a meridian);
    /// a curve leaving that half-plane is refused.
    fn revolution(&self,r: u32,v: &[Value]) -> Result<(Surface,[f64;2]),Refusal> {
        let swept = self.curve(r,v.get(1).and_then(Value::reference))?;
        let (a,_,axis) = self.of(r,v.get(2).and_then(Value::reference),&["AXIS1_PLACEMENT"],"a revolution's axis")?;
        let o = self.point(a,axis.get(1).and_then(Value::reference))?;
        let z = match optional(axis.get(2),a,"an axis's direction")? { Some(d) => self.direction(a,Some(d))?,None => [0.,0.,1.] };
        // points the curve lies within: a spline's poles (it lies in their hull), else samples
        let points: Vec<V> = match &swept.curve {
            Curve::BSpline(b) => b.poles.clone(),
            Curve::Line {p,d} => vec![*p,add(*p,*d)],
            c => (0..64).map(|k| c.point(std::f64::consts::TAU*k as f64/64.)).collect(),
        };
        let radial = |p: V| { let w = sub(p,o); sub(w,scale(z,dot(w,z))) };
        let far = points.iter().map(|&p| radial(p)).max_by(|a,b| norm(*a).total_cmp(&norm(*b))).unwrap_or([0.;3]);
        let x = normalised(far).filter(|_| norm(far) > 0.).ok_or_else(|| Refusal::unsupported(r,"a revolution of a curve on its axis"))?;
        let f = Frame::new(o,z,x);
        let size = points.iter().map(|&p| norm(sub(p,o))).fold(0.,f64::max);
        let within = 1e-9*size+self.context.uncertainty.unwrap_or(0.);
        for &p in &points {
            let l = f.local(p);
            if l[1].abs() > within || l[0] < -within {
                return Err(Refusal::unsupported(r,"a revolution of a curve leaving the half-plane of its axis"))
            }
        }
        Ok((Surface::Revolution(f,Arc::new(swept.curve)),[self.context.angle,swept.rate]))
    }
}
