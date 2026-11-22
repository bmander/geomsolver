//! A file's geometry read into the kernel's: a curve in space (`geom::Curve`), a curve in a
//! face's parameters, and a surface (`geom::Surface`), each scaled from the file's units
//! (`context::Context`) and placed as the kernel places it. What the kernel has no curve or
//! surface for is refused naming the entity (`Refusal::unsupported`).
//!
//! **Nothing is refitted.** A curve or a surface is the one the file gives, its numbers scaled
//! into millimetres and radians: a B-spline's poles and weights and knots, a conic's placement and
//! radii. Where the kernel holds the same set of points in another form it is read so, exactly: a
//! linear extrusion's curve is carried along its direction into the plane square to it (an affine
//! map of its poles), a revolution's frame is turned so its curve lies in the half-plane of its
//! `x`, a curve turned off that half-plane — a line turned about an axis it does not meet too
//! (`Seat::Throat`) — is the rational net of its revolution (`turned`), a parabola's arc is the quadratic over its own parameter and a
//! hyperbola's the rational quadratics of its `cosh` (`parabolic`, `hyperbolic`), a B-spline whose
//! knots are not clamped is clamped over its domain by inserting them (`clamped`), an offset of an
//! elementary surface is the elementary surface it is, and a spline sheet's offset the kernel's
//! `Surface::Offset`. A surface the kernel holds no exact form of is refused, never approximated.
//!
//! **A form that waits on its faces** (`Seat`). A net has no period, and a line or a parabola runs
//! on without end: a revolution read as a net has its seams put where no face of it crosses them,
//! and a surface swept from an unbounded curve is bounded past every face on it, once the faces'
//! edges are read (`Sheet::seated`, which `topology` calls with their points).
//!
//! **A parameter's rate.** The kernel reads a line by its length and a conic by its angle in
//! radians; the file reads a line by multiples of its vector's magnitude in the file's unit and a
//! conic by its angle in the file's unit. Each curve read carries `rate`, the kernel's parameter
//! per the file's, so a curve in a face's parameters written over the file's parameter of an edge
//! can be read over the kernel's (`Spatial::rate`). Where no rate maps the one onto the other (a
//! curve read the other way round, a hyperbola's rational parameter) a file's curve in a face's
//! parameters lies off its edge, and is not used (`topology::file_pcurve` checks each).
#[allow(unused_imports)]
use crate::util::fmath::Det;
use super::context::Context;
use super::Refusal;
use crate::geom::{Curve,Frame,Surface,V};
use crate::nurbs::{arc,BSpline,Net};
use crate::step_file::{File,Value};
use crate::util::space::{add,cross,distance,dot,norm,normalised,scale,sub};
use std::f64::consts::TAU;
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
/// `seat`, where the surface's form waits on where its faces lie (`Seat`): `surface` is then a
/// provisional one, and `seated` gives the form.
#[derive(Clone,Debug)]
pub struct Sheet { pub surface: Surface,pub scale: [f64;2],pub seat: Option<Arc<Seat>> }

/// **The scale of a sheet whose parameters are no scaling of the file's** (a revolution or a sweep
/// held as a rational net, its arcs' parameters meeting the file's angles only at their knots, its
/// seams put where its faces leave room): a file's curve in its faces' parameters is not read
/// through it (`Flat::scaled`), the edge inverted onto the sheet instead. A curve read through it
/// at a parameter off the net's domain was taken for on it where the net, clamped, held its point
/// at the domain's edge and its foot there too (springrep's coin turned about a line through its
/// rim read an edge 3.55 mm off its face).
pub const UNMAPPED: [f64;2] = [f64::NAN;2];

/// **A surface whose kernel form waits on where its faces lie**: one held as a net, which has no
/// period, whose seams are put where no face of it crosses them, or one swept from a curve without
/// end, bounded past every face on it (`Sheet::seated`). Each is the file's set of points exactly
/// over every face on it.
#[derive(Clone,Debug)]
pub enum Seat {
    /// A curve (a B-spline, a circle or an ellipse) turned about `f.z` as the rational net of its
    /// revolution (`turned`), standing `offset` off it along its normal (`Surface::Offset`) where
    /// that is not zero; its seams — where the turn starts, and where a closed conic starts — put
    /// in the widest gap its faces leave.
    Turned { f: Frame,curve: Curve,offset: f64 },
    /// A curve in the plane through `f.o` square to `f.z` (a B-spline, a circle or an ellipse)
    /// swept along `f.z` as a net, standing `offset` off it: bounded along `f.z` past its faces, a
    /// closed conic's seam put in the widest gap they leave.
    Swept { f: Frame,curve: Curve,offset: f64 },
    /// A line `p + t d` turned about `f.z`, which it neither meets nor parallels (a hyperboloid of
    /// one sheet): the rational net of its stretch past its faces turned (`turned`), its `u` and
    /// `t` the file's (so the line, where a file lays its seam, is one of the net's), its seam put
    /// as a `Turned` one's.
    Throat { f: Frame,p: V,d: V },
    /// A line `p + t d` turned about the axis through `apex` along `z`, which it meets at `apex` (a
    /// double cone, or a plane): the revolution of the half of it on the side its faces lie, its
    /// parameters the file's.
    Nappe { apex: V,z: V,p: V,d: V },
    /// An open conic in a plane holding the axis `f.z`, turned about it: the revolution of its arc
    /// in the half-plane of `f.x` — the one half it has there, or where it crosses the axis the
    /// half the file's curves in its faces' parameters lie on — bounded by its faces.
    Meridian { f: Frame,conic: Open },
    /// A closed conic in a plane holding the axis `f.z`, crossing it, turned about it: the
    /// revolution of its arc between two crossings on the side of the axis its faces lie on (the
    /// arc's points `f.at([±ρ, 0, h])`, the sign `f.x`'s or the other), its normal the file's at
    /// every point; where its faces lie on both sides or on neither, the whole conic turned once
    /// as a net from the file's start (`turned`), each point of it on one sheet or the other.
    Crossing { f: Frame,curve: Curve },
}

/// **A conic that runs without end**, in its frame `g`: a parabola `F (u² x + 2u y)`, a hyperbola
/// `a cosh u x + b sinh u y` (ISO 10303-42's parameters).
#[derive(Clone,Copy,Debug)]
pub enum Open { Parabola { g: Frame,focal: f64 },Hyperbola { g: Frame,a: f64,b: f64 } }

impl Open {
    fn frame(&self) -> &Frame { match self { Open::Parabola {g,..} | Open::Hyperbola {g,..} => g } }
    pub fn point(&self,u: f64) -> V {
        match *self { Open::Parabola {g,focal} => g.at([focal*u*u,2.*focal*u,0.]),Open::Hyperbola {g,a,b} => g.at([a*u.cosh(),b*u.sinh(),0.]) }
    }
    /// The parameter of its point `p`, by its coordinate along `y` (which it runs along one to one).
    pub fn parameter(&self,p: V) -> f64 {
        match *self { Open::Parabola {g,focal} => g.local(p)[1]/(2.*focal),Open::Hyperbola {g,b,..} => (g.local(p)[1]/b).asinh() }
    }
    /// Its arc over `span`, exactly (`parabolic`, `hyperbolic`).
    pub fn arc(&self,span: [f64;2]) -> BSpline {
        match *self { Open::Parabola {g,focal} => parabolic(&g,focal,span),Open::Hyperbola {g,a,b} => hyperbolic(&g,a,b,span) }
    }
    /// **The one sign its points' coordinate along the unit `e` from `o` takes**, where it takes
    /// one (zero allowed): `A u² + B u + C` a parabola's, of one sign where `A` leads and its
    /// discriminant is not positive; `α cosh u + β sinh u + C` a hyperbola's, bounded on one side
    /// where `|α| > |β|`, at `sign α √(α² − β²) + C`.
    fn one_sided(&self,o: V,e: V) -> Option<f64> {
        let g = self.frame();
        let c = dot(sub(g.o,o),e);
        match *self {
            Open::Parabola {focal,..} => {
                let (a,b) = (focal*dot(g.x,e),2.*focal*dot(g.y,e));
                if a.abs() <= 1e-12*focal { return (b.abs() <= 1e-12*focal && c != 0.).then(|| c.signum()) }
                (b*b-4.*a*c <= 1e-12*(b*b+(4.*a*c).abs())).then(|| a.signum())
            }
            Open::Hyperbola {a,b,..} => {
                let (al,be) = (a*dot(g.x,e),b*dot(g.y,e));
                if !(al.abs() > be.abs()) { return None }
                let least = al.signum()*(al*al-be*be).sqrt()+c;
                (least*al.signum() >= -1e-12*(a+b)).then(|| al.signum())
            }
        }
    }
}

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
        if !s.iter().all(|x| x.is_finite()) { return None }
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

/// The kinds of B-spline curve that say their knots (one with them, or one whose kind implies
/// them), and of surface; and the weights' part of a rational one.
const SPLINE_CURVES: [&str;5] = ["B_SPLINE_CURVE_WITH_KNOTS","UNIFORM_CURVE","QUASI_UNIFORM_CURVE","BEZIER_CURVE","RATIONAL_B_SPLINE_CURVE"];
const SPLINE_SURFACES: [&str;5] = ["B_SPLINE_SURFACE_WITH_KNOTS","UNIFORM_SURFACE","QUASI_UNIFORM_SURFACE","BEZIER_SURFACE","RATIONAL_B_SPLINE_SURFACE"];

/// Where a spline's knots come from: written, or implied by its kind (`implied`).
enum Spec<'v,'a> { Written(&'v [Value<'a>]),Implied(&'static str) }

/// **The knots a B-spline's kind implies** (ISO 10303-42), for `n` poles of degree `p`: a uniform
/// one's `−p, 1 − p, …, n` (its domain `[0, n − p]`); a quasi-uniform one's `0, 1, …, n − p`, its
/// ends `p + 1` times; a piecewise Bézier one's `0, 1, …, (n − 1)/p`, each inside `p` times and its
/// ends `p + 1` — refused where its poles are no whole number of Bézier pieces.
fn implied(kind: &str,p: usize,n: usize,r: u32) -> Result<Vec<f64>,Refusal> {
    if n <= p { return Err(Refusal::malformed(r,format!("a B-spline of degree {p} with {n} poles"))) }
    Ok(match kind {
        "UNIFORM" => (0..n+p+1).map(|i| i as f64-p as f64).collect(),
        "QUASI_UNIFORM" => {
            let mut k = vec![0.;p+1];
            k.extend((1..n-p).map(|i| i as f64));
            k.extend(std::iter::repeat_n((n-p) as f64,p+1));
            k
        }
        _ => {
            if (n-1)%p != 0 { return Err(Refusal::malformed(r,format!("a Bézier B-spline of degree {p} with {n} poles: no whole number of pieces"))) }
            let m = (n-1)/p;
            let mut k = vec![0.;p+1];
            for i in 1..m { k.extend(std::iter::repeat_n(i as f64,p)); }
            k.extend(std::iter::repeat_n(m as f64,p+1));
            k
        }
    })
}

/// The family of knots a kind of B-spline implies (`implied`).
fn family(kind: &str) -> &'static str {
    if kind.starts_with("QUASI_UNIFORM") { "QUASI_UNIFORM" } else if kind.starts_with("UNIFORM") { "UNIFORM" } else { "BEZIER" }
}

/// Whether knots of degree `p` over `n` poles are clamped: their first `p + 1` one, and their last.
fn is_clamped(k: &[f64],p: usize,n: usize) -> bool { k[..=p].iter().all(|&x| x == k[p]) && k[n..].iter().all(|&x| x == k[n]) }

/// **A B-spline clamped over its domain**: knots put in at its domain's ends until each stands
/// `p` times, the poles outside dropped — exact (Boehm's insertion: the same curve at the same
/// parameters, `Net::segment` over a net of two equal columns). One already clamped is itself.
fn clamped(b: BSpline) -> BSpline {
    let (p,n) = (b.degree,b.poles.len());
    if is_clamped(&b.knots,p,n) { return b }
    let net = Net {du:p,dv:1,uknots:b.knots.clone(),vknots:vec![0.,0.,1.,1.],poles:b.poles.iter().map(|&q| vec![q,q]).collect(),
        weights:b.weights.as_ref().map(|w| w.iter().map(|&x| vec![x,x]).collect())};
    let s = net.segment(b.domain(),[0.,1.]);
    BSpline {degree:p,knots:s.uknots,poles:s.poles.iter().map(|r| r[0]).collect(),weights:s.weights.map(|w| w.iter().map(|r| r[0]).collect())}
}

/// A curve read the other way round, `C(−t)` (a B-spline's `C(a + b − t)`): a line's direction
/// turned, a conic's frame turned over its `x`, a spline's poles and knots reversed.
fn reversed(c: &Curve) -> Option<Curve> {
    Some(match c {
        Curve::Line {p,d} => Curve::Line {p:*p,d:scale(*d,-1.)},
        Curve::Circle(f,r) => Curve::Circle(Frame {y:scale(f.y,-1.),z:scale(f.z,-1.),..*f},*r),
        Curve::Ellipse(f,a,b) => Curve::Ellipse(Frame {y:scale(f.y,-1.),z:scale(f.z,-1.),..*f},*a,*b),
        Curve::BSpline(s) => {
            let (a,z) = (s.knots[0],s.knots[s.knots.len()-1]);
            Curve::BSpline(Arc::new(BSpline {degree:s.degree,knots:s.knots.iter().rev().map(|k| a+z-k).collect(),
                poles:s.poles.iter().rev().copied().collect(),weights:s.weights.as_ref().map(|w| w.iter().rev().copied().collect())}))
        }
        _ => return None,
    })
}

/// What bounds an open conic: an edge's vertices, or a trimmed curve's two parameters.
#[derive(Clone,Copy,Debug)]
enum Ends { Points([V;2]),Parameters([f64;2]) }

/// **A parabola's arc `C + F (u² x + 2u y)` over `[u0, u1]`, exactly**: the quadratic B√©zier
/// over its own parameter, its ends the arc's and its middle pole where their tangents meet.
fn parabolic(f: &Frame,focal: f64,[u0,u1]: [f64;2]) -> BSpline {
    let at = |a: f64,b: f64| f.at([focal*a,2.*focal*b,0.]);
    BSpline {degree:2,knots:vec![u0,u0,u0,u1,u1,u1],poles:vec![at(u0*u0,u0),at(u0*u1,0.5*(u0+u1)),at(u1*u1,u1)],weights:None}
}

/// **A hyperbola's arc `C + a cosh t x + b sinh t y` over `[t0, t1]`, exactly**: rational
/// quadratics, a span to every unit of `t` or less, each span's middle pole where its ends'
/// tangents meet, `(a cosh m, b sinh m) / cosh h` about the span's middle `m` and half-width `h`,
/// weighed `cosh h` (the circle's arc with its cosines hyperbolic), its parameter meeting `t` at
/// every knot.
fn hyperbolic(f: &Frame,a: f64,b: f64,[t0,t1]: [f64;2]) -> BSpline {
    let n = ((t1-t0).ceil() as usize).max(1);
    let d = (t1-t0)/n as f64;
    let h = 0.5*d;
    let at = |c: f64,s: f64| f.at([a*c,b*s,0.]);
    let (mut poles,mut weights,mut knots) = (vec![at(t0.cosh(),t0.sinh())],vec![1.],vec![t0;3]);
    for k in 0..n {
        let (m,end) = (t0+(k as f64+0.5)*d,if k+1 == n { t1 } else { t0+(k+1) as f64*d });
        poles.push(at(m.cosh()/h.cosh(),m.sinh()/h.cosh()));
        poles.push(at(end.cosh(),end.sinh()));
        weights.extend([h.cosh(),1.]);
        knots.extend(std::iter::repeat_n(end,if k+1 == n { 3 } else { 2 }));
    }
    BSpline {degree:2,knots,poles,weights:Some(weights)}
}

/// The rational spline of a closed conic, its whole turn from the angle `from`.
fn conic_spline(c: &Curve,from: f64) -> Option<BSpline> {
    Some(match c {
        Curve::Circle(g,r) => arc(g,*r,[from,from+TAU]),
        Curve::Ellipse(g,a,b) => {
            let unit = arc(&Frame::WORLD,1.,[from,from+TAU]);
            BSpline {poles:unit.poles.iter().map(|p| g.at([a*p[0],b*p[1],0.])).collect(),..unit}
        }
        _ => return None,
    })
}

/// The angle in `[0, 2π)` a whole turn's rational arc from `from` reaches at its parameter `t`
/// (`nurbs::arc`'s parameter meets the angle only at its knots).
fn angle_at(t: f64,from: f64) -> f64 {
    let q = arc(&Frame::WORLD,1.,[from,from+TAU]).point(t);
    q[1].datan2(q[0]).rem_euclid(TAU)
}

/// **The widest gap angles leave round a turn**: where it starts and how wide it is; none where
/// there are none.
fn widest_gap(mut a: Vec<f64>) -> Option<(f64,f64)> {
    a.retain(|x| x.is_finite());
    a.sort_by(f64::total_cmp);
    let (first,last) = (*a.first()?,*a.last()?);
    let mut best = (first+TAU-last,last);
    for w in a.windows(2) { if w[1]-w[0] > best.0 { best = (w[1]-w[0],w[0]); } }
    Some((best.1,best.0))
}

/// `[lo, hi]` of numbers, widened each way by a tenth of its width and `pad`.
fn spread(x: impl Iterator<Item=f64>,pad: f64) -> Option<[f64;2]> {
    let (lo,hi) = x.filter(|x| x.is_finite()).fold((f64::INFINITY,f64::NEG_INFINITY),|(lo,hi),x| (lo.min(x),hi.max(x)));
    (lo <= hi).then(|| { let m = 0.1*(hi-lo)+pad; [lo-m,hi+m] })
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
    /// A vector: its unit direction, and its magnitude as written — not negative (the standard's
    /// rule), and zero only where `what` accepts none (a line's is refused by the line).
    fn vector(&self,from: u32,r: Option<u32>,flat: bool) -> Result<(Vec<f64>,f64),Refusal> {
        let (r,_,v) = self.of(from,r,&["VECTOR"],"a vector")?;
        let d = if flat { self.flat_direction(r,v.get(1).and_then(Value::reference))?.to_vec() }
            else { self.direction(r,v.get(1).and_then(Value::reference))?.to_vec() };
        let m = number(v.get(2),r,"a vector's magnitude")?;
        if !(m >= 0.) { return Err(Refusal::malformed(r,format!("a vector's magnitude of {m}"))) }
        Ok((d,m))
    }
    /// An `AXIS2_PLACEMENT_3D` as a right-handed frame (the standard's axes are right-handed by
    /// construction, `y = z × x`): its axis `[0, 0, 1]` and its reference direction the standard's
    /// default where unset (ISO 10303-42's `first_proj_axis`: `[1, 0, 0]`, or `[0, 1, 0]` about an
    /// axis along `±x`, CATIA's `AXIS2_PLACEMENT_3D('…',#p,(-1,0,0),$)`), the reference made square
    /// to the axis; refused where it lies along the axis.
    pub fn placement(&self,from: u32,r: Option<u32>) -> Result<Frame,Refusal> {
        let (r,_,v) = self.of(from,r,&["AXIS2_PLACEMENT_3D"],"a placement")?;
        if v.len() != 4 { return Err(Refusal::malformed(r,"a placement of four parameters expected")) }
        let o = self.point(r,v[1].reference())?;
        let z = match optional(v.get(2),r,"a placement's axis")? { Some(a) => self.direction(r,Some(a))?,None => [0.,0.,1.] };
        let x = match optional(v.get(3),r,"a placement's reference direction")? {
            Some(a) => self.direction(r,Some(a))?,
            // (the default's own part square to the axis, however small: an axis a rounding off
            // `±x` — CATIA's `(1.,-2.1862853408E-015,0.)` — leaves `[1, 0, 0]` a hair off it, and
            // the standard's projection is still a direction; where the turn starts about such an
            // axis is all it says)
            None => {
                let v = if z[1] == 0. && z[2] == 0. { [0.,1.,0.] } else { [1.,0.,0.] };
                normalised(sub(v,scale(z,dot(v,z)))).ok_or_else(|| Refusal::malformed(r,"a placement whose axis leaves its reference direction none"))?
            }
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
    /// **Whether a replica's transformation moves nothing**: its axes unset or the frame's own,
    /// its origin the origin and its scale unset or one (`CARTESIAN_TRANSFORMATION_OPERATOR_3D`,
    /// with or without the description later editions put after its name).
    fn identity(&self,from: u32,r: Option<u32>) -> Result<bool,Refusal> {
        let (r,_,v) = self.of(from,r,&["CARTESIAN_TRANSFORMATION_OPERATOR_3D","CARTESIAN_TRANSFORMATION_OPERATOR"],"a replica's transformation")?;
        let k = usize::from(v.get(1).and_then(Value::text).is_some());
        let axis = |i: usize,unit: V| -> Result<bool,Refusal> {
            Ok(match optional(v.get(i+k),r,"a transformation's axis")? { None => true,Some(a) => norm(sub(self.direction(r,Some(a))?,unit)) <= 1e-12 })
        };
        let origin = self.point(r,v.get(3+k).and_then(Value::reference))?;
        let unit = match v.get(4+k) { Some(Value::Unset) | None => true,s => (number(s,r,"a transformation's scale")?-1.).abs() <= 1e-12 };
        Ok(axis(1,[1.,0.,0.])? && axis(2,[0.,1.,0.])? && (v.len() <= 5+k || axis(5,[0.,0.,1.])?) && norm(origin) <= 1e-12 && unit)
    }

    /// A curve in space.
    pub fn curve(&self,from: u32,r: Option<u32>) -> Result<Spatial,Refusal> { self.curve_in(from,r,None,0) }
    /// An edge's curve, `ends` its vertices' points: as `curve`, a curve without end (a parabola,
    /// a hyperbola) read as its arc between them.
    pub fn edge_curve(&self,from: u32,r: Option<u32>,ends: [V;2]) -> Result<Spatial,Refusal> { self.curve_in(from,r,Some(Ends::Points(ends)),0) }
    fn curve_in(&self,from: u32,r: Option<u32>,ends: Option<Ends>,depth: usize) -> Result<Spatial,Refusal> {
        let r = r.ok_or_else(|| Refusal::malformed(from,"a curve: a reference expected"))?;
        if depth > 8 { return Err(Refusal::malformed(r,"a curve nested more than eight deep")) }
        let e = self.file.get(r);
        if e.is("B_SPLINE_CURVE") || SPLINE_CURVES.iter().any(|k| e.is(k)) {
            return Ok(Spatial {curve:Curve::BSpline(Arc::new(self.spline(r,false)?)),rate:1.})
        }
        let Some((kind,v)) = e.simple() else { return Err(Refusal::unsupported(r,format!("a curve of the kinds {}",self.kinds(r)))) };
        let (l,a) = (self.context.length,self.context.angle);
        Ok(match kind {
            "LINE" => {
                let p = self.point(r,v.get(1).and_then(Value::reference))?;
                let (d,m) = self.vector(r,v.get(2).and_then(Value::reference),false)?;
                if m == 0. { return Err(Refusal::malformed(r,"a line whose vector has no length: a point, which no edge runs along")) }
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
            "PARABOLA" | "HYPERBOLA" => {
                let f = self.placement(r,v.get(1).and_then(Value::reference))?;
                let parabola = kind == "PARABOLA";
                let (p,q) = if parabola { (self.length(v.get(2),r,"a parabola's focal distance")?,0.) }
                    else { (self.length(v.get(2),r,"a hyperbola's semi-axis")?,self.length(v.get(3),r,"a hyperbola's imaginary semi-axis")?) };
                // its parameter at a point: `y`'s coordinate over `2F`, or `asinh` of it over `b`
                let at = |x: V| { let y = f.local(x)[1]; if parabola { y/(2.*p) } else { (y/q).asinh() } };
                let [u0,u1] = match ends {
                    Some(Ends::Points([x,y])) => [at(x),at(y)],
                    Some(Ends::Parameters(t)) => t,
                    None => return Err(Refusal::unsupported(r,format!("a {} not bounded by an edge's vertices or a trimmed curve's trims, which runs without end",kind.to_lowercase()))),
                };
                let (u0,u1) = (u0.min(u1),u0.max(u1));
                if !(u1 > u0) || !(u1-u0).is_finite() { return Err(Refusal::malformed(r,format!("a {}'s arc between one point and itself",kind.to_lowercase()))) }
                if !parabola && u1-u0 > 64. { return Err(Refusal::unsupported(r,"a hyperbola's arc of more than 64 units of its parameter")) }
                let s = if parabola { parabolic(&f,p,[u0,u1]) } else { hyperbolic(&f,p,q,[u0,u1]) };
                Spatial {curve:Curve::BSpline(Arc::new(s)),rate:1.}
            }
            "POLYLINE" => Spatial {curve:Curve::BSpline(Arc::new(self.polyline(r,v,false)?)),rate:1.},
            "TRIMMED_CURVE" => {
                // bounded by its edge's vertices where it is an edge's curve (an edge is trimmed
                // by them), else by its own trims; read the other way where it runs against its
                // basis curve
                let sense = logical(v.get(4),r,"a trimmed curve's sense")?;
                let ends = match ends { Some(e) => Some(e),None => self.trims(r,v)? };
                let basis = self.curve_in(r,v.get(1).and_then(Value::reference),ends,depth+1)?;
                if sense { basis } else {
                    Spatial {curve:reversed(&basis.curve).ok_or_else(|| Refusal::unsupported(r,"a trimmed curve against a basis curve the reader cannot reverse"))?,rate:basis.rate}
                }
            }
            "COMPOSITE_CURVE" => {
                let (parent,same) = self.segment(r,v)?;
                let c = self.curve_in(r,Some(parent),ends,depth+1)?;
                if same { c } else { Spatial {curve:reversed(&c.curve).ok_or_else(|| Refusal::unsupported(r,"a composite curve's segment the reader cannot reverse"))?,rate:c.rate} }
            }
            "CURVE_REPLICA" => {
                if !self.identity(r,v.get(2).and_then(Value::reference))? {
                    return Err(Refusal::unsupported(r,"a curve's replica moved: not read yet"))
                }
                self.curve_in(r,v.get(1).and_then(Value::reference),ends,depth+1)?
            }
            "OFFSET_CURVE_3D" => return Err(Refusal::unsupported(r,"an offset curve in space, which the kernel holds no form of")),
            _ => return Err(Refusal::unsupported(r,format!("a curve of the kind {kind}"))),
        })
    }
    /// A composite curve of one segment: its parent curve, and whether along it; refused where
    /// it has more (an edge running along several curves is not one the kernel holds).
    fn segment(&self,r: u32,v: &[Value]) -> Result<(u32,bool),Refusal> {
        let segments = list(v.get(1),r,"a composite curve's segments")?;
        if segments.len() != 1 { return Err(Refusal::unsupported(r,format!("a composite curve of {} segments as one curve",segments.len()))) }
        let (s,_,sv) = self.of(r,segments[0].reference(),&["COMPOSITE_CURVE_SEGMENT","REPARAMETRISED_COMPOSITE_CURVE_SEGMENT"],"a composite curve's segment")?;
        let same = logical(sv.get(1),s,"a segment's sense")?;
        Ok((sv.get(2).and_then(Value::reference).ok_or_else(|| Refusal::malformed(s,"a segment's curve: a reference expected"))?,same))
    }
    /// A trimmed curve's trims, where it is bounded by them alone: a point's each, or a
    /// parameter's, as its master representation prefers (the point where it is unspecified and
    /// both are given). None where either trim gives neither.
    fn trims(&self,r: u32,v: &[Value]) -> Result<Option<Ends>,Refusal> {
        let cartesian = v.get(5).and_then(Value::word) == Some("CARTESIAN");
        let mut points = Vec::new();
        let mut parameters = Vec::new();
        for k in [2,3] {
            for t in list(v.get(k),r,"a trimmed curve's trim")? {
                match t {
                    Value::Ref(p) => points.push(self.point(r,Some(*p))?),
                    Value::Typed("PARAMETER_VALUE",x) => parameters.push(number(x.first(),r,"a trim's parameter")?),
                    _ => return Err(Refusal::malformed(r,"a trim: a point or a parameter value expected")),
                }
            }
        }
        Ok(match (points.len(),parameters.len()) {
            (2,2) if !cartesian => Some(Ends::Parameters([parameters[0],parameters[1]])),
            (2,_) => Some(Ends::Points([points[0],points[1]])),
            (_,2) => Some(Ends::Parameters([parameters[0],parameters[1]])),
            _ => None,
        })
    }
    /// A curve in a surface's parameters, as the file writes it (`Flat`).
    pub fn flat(&self,from: u32,r: Option<u32>) -> Result<Flat,Refusal> { self.flat_in(from,r,0) }
    fn flat_in(&self,from: u32,r: Option<u32>,depth: usize) -> Result<Flat,Refusal> {
        let r = r.ok_or_else(|| Refusal::malformed(from,"a curve: a reference expected"))?;
        if depth > 8 { return Err(Refusal::malformed(r,"a curve nested more than eight deep")) }
        let e = self.file.get(r);
        if e.is("B_SPLINE_CURVE") || SPLINE_CURVES.iter().any(|k| e.is(k)) { return Ok(Flat::Curve(Curve::BSpline(Arc::new(self.spline(r,true)?)))) }
        let Some((kind,v)) = e.simple() else { return Err(Refusal::unsupported(r,format!("a curve of the kinds {}",self.kinds(r)))) };
        Ok(match kind {
            "LINE" => {
                let p = self.flat_point(r,v.get(1).and_then(Value::reference))?;
                let (d,m) = self.vector(r,v.get(2).and_then(Value::reference),true)?;
                if m == 0. { return Err(Refusal::unsupported(r,"a line in a surface's parameters whose vector has no length")) }
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
            // (one running against its basis is over another parameter: not used)
            "TRIMMED_CURVE" if logical(v.get(4),r,"a trimmed curve's sense")? => self.flat_in(r,v.get(1).and_then(Value::reference),depth+1)?,
            "COMPOSITE_CURVE" => match self.segment(r,v)? {
                (parent,true) => self.flat_in(r,Some(parent),depth+1)?,
                _ => return Err(Refusal::unsupported(r,"a composite curve in a surface's parameters against its segment")),
            },
            _ => return Err(Refusal::unsupported(r,format!("a curve in a surface's parameters of the kind {kind}"))),
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
    /// **A B-spline curve**, in space or in a surface's parameters (`flat`): the simple entity of
    /// its knots (`B_SPLINE_CURVE_WITH_KNOTS`) or of a kind implying them (`UNIFORM_CURVE`,
    /// `QUASI_UNIFORM_CURVE`, `BEZIER_CURVE`, `implied`), or the complex entity of one of those
    /// with `B_SPLINE_CURVE` and its weights. Its form and its `closed_curve` are the writer's
    /// word about it, not read: the poles say what it is. Clamped over its domain (`clamped`).
    fn spline(&self,r: u32,flat: bool) -> Result<BSpline,Refusal> {
        let e = self.file.get(r);
        let abstract_ = || Refusal::malformed(r,"a B-spline curve of no kind of knots (`B_SPLINE_CURVE` is an abstract supertype)");
        let (head,spec,weights) = match e.simple() {
            Some(("B_SPLINE_CURVE_WITH_KNOTS",v)) => {
                if v.len() != 9 { return Err(Refusal::malformed(r,"a B-spline curve of nine parameters expected")) }
                (&v[1..6],Spec::Written(&v[6..8]),None)
            }
            Some((k @ ("UNIFORM_CURVE" | "QUASI_UNIFORM_CURVE" | "BEZIER_CURVE"),v)) => {
                if v.len() != 6 { return Err(Refusal::malformed(r,format!("a {k} of six parameters expected"))) }
                (&v[1..6],Spec::Implied(family(k)),None)
            }
            Some(("B_SPLINE_CURVE",_)) => return Err(abstract_()),
            Some((kind,_)) => return Err(Refusal::unsupported(r,format!("a curve of the kind {kind}"))),
            None => {
                let Some(s) = e.part("B_SPLINE_CURVE").filter(|s| s.len() == 5) else { return Err(Refusal::malformed(r,"a B-spline curve whose parts do not read")) };
                let spec = match (e.part("B_SPLINE_CURVE_WITH_KNOTS"),["UNIFORM_CURVE","QUASI_UNIFORM_CURVE","BEZIER_CURVE"].into_iter().find(|k| e.is(k))) {
                    (Some(k),_) if k.len() == 3 => Spec::Written(&k[..2]),
                    (Some(_),_) => return Err(Refusal::malformed(r,"a B-spline curve's knots of three parameters expected")),
                    (None,Some(k)) => Spec::Implied(family(k)),
                    (None,None) => return Err(abstract_()),
                };
                (s,spec,e.part("RATIONAL_B_SPLINE_CURVE"))
            }
        };
        let degree = integer(head.first(),r,"a B-spline's degree")?;
        if !(1..=crate::nurbs::MAX_DEGREE as i64).contains(&degree) { return Err(Refusal::unsupported(r,format!("a B-spline of degree {degree}"))) }
        let p = degree as usize;
        let mut poles: Vec<V> = list(head.get(1),r,"a B-spline's poles")?.iter().map(|q| self.pole(r,q.reference(),flat)).collect::<Result<_,_>>()?;
        let mut knots = match spec {
            Spec::Written(ks) => knots(list(ks.first(),r,"a B-spline's multiplicities")?,list(ks.get(1),r,"a B-spline's knots")?,r)?,
            Spec::Implied(k) => implied(k,p,poles.len(),r)?,
        };
        let mut weights = match weights {
            Some(w) => Some(list(w.first(),r,"a B-spline's weights")?.iter().map(|x| number(Some(x),r,"a weight")).collect::<Result<Vec<_>,_>>()?),
            None => None,
        };
        // an end knot repeated past the degree and one more weighs its end pole nothing over the
        // domain (a span of no length beyond it): that pole and knot dropped, the same curve
        if knots.len() == poles.len()+p+1 && weights.as_ref().is_none_or(|w| w.len() == poles.len()) {
            while poles.len() > p+1 && knots[poles.len()-1] == knots[poles.len()] {
                poles.pop(); knots.pop();
                if let Some(w) = &mut weights { w.pop(); }
            }
            while poles.len() > p+1 && knots[p] == knots[p+1] {
                poles.remove(0); knots.remove(0);
                if let Some(w) = &mut weights { w.remove(0); }
            }
        }
        let s = match weights {
            Some(w) => BSpline::rational(p,knots,poles,w),
            None => BSpline::new(p,knots,poles),
        };
        s.map(clamped).map_err(|m| Refusal::malformed(r,m))
    }

    /// A surface.
    pub fn surface(&self,from: u32,r: Option<u32>) -> Result<Sheet,Refusal> { self.surface_in(from,r,0) }
    fn surface_in(&self,from: u32,r: Option<u32>,depth: usize) -> Result<Sheet,Refusal> {
        let r = r.ok_or_else(|| Refusal::malformed(from,"a surface: a reference expected"))?;
        if depth > 8 { return Err(Refusal::malformed(r,"a surface nested more than eight deep")) }
        let e = self.file.get(r);
        if e.is("B_SPLINE_SURFACE") || SPLINE_SURFACES.iter().any(|k| e.is(k)) {
            return Ok(Sheet {surface:Surface::BSpline(Frame::WORLD,Arc::new(self.net(r)?)),scale:[1.,1.],seat:None})
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
                if !(half > 0. && half < std::f64::consts::FRAC_PI_2) { return Err(Refusal::malformed(r,format!("a cone's semi-angle of {half} rad"))) }
                (cone(place()?,rad*l,half),[a,l/half.dcos()])
            }
            "SPHERICAL_SURFACE" => (Surface::Sphere(place()?,self.length(v.get(2),r,"a sphere's radius")?),[a,a]),
            "TOROIDAL_SURFACE" => (Surface::Torus(place()?,number(v.get(2),r,"a torus's major radius")?*l,
                self.length(v.get(3),r,"a torus's minor radius")?),[a,a]),
            "DEGENERATE_TOROIDAL_SURFACE" => {
                // the apple (`select_outer`) or the lemon of a spindle torus: the kernel's torus
                // holds both, its faces' bounds the part they lie on
                let (big,small) = (self.length(v.get(2),r,"a torus's major radius")?,self.length(v.get(3),r,"a torus's minor radius")?);
                logical(v.get(4),r,"a degenerate torus's choice of its outer part")?;
                if !(big < small) { return Err(Refusal::malformed(r,"a degenerate torus whose major radius is not less than its minor")) }
                (Surface::Torus(place()?,big,small),[a,a])
            }
            "SURFACE_OF_LINEAR_EXTRUSION" => return self.extrusion(r,v),
            "SURFACE_OF_REVOLUTION" => return self.revolution(r,v),
            "OFFSET_SURFACE" => return self.offset(r,v,depth),
            // a face's surface trimmed or bounded: its basis surface, which the face's own bounds
            // trim (one running against its basis in one parameter alone, its normal turned, not)
            "RECTANGULAR_TRIMMED_SURFACE" => {
                let (us,vs) = (logical(v.get(6),r,"a trimmed surface's sense in u")?,logical(v.get(7),r,"a trimmed surface's sense in v")?);
                if us != vs { return Err(Refusal::unsupported(r,"a trimmed surface against its basis in one parameter, its normal turned: not read yet")) }
                return self.surface_in(r,v.get(1).and_then(Value::reference),depth+1)
            }
            "CURVE_BOUNDED_SURFACE" => return self.surface_in(r,v.get(1).and_then(Value::reference),depth+1),
            "SURFACE_REPLICA" => {
                if !self.identity(r,v.get(2).and_then(Value::reference))? { return Err(Refusal::unsupported(r,"a surface's replica moved: not read yet")) }
                return self.surface_in(r,v.get(1).and_then(Value::reference),depth+1)
            }
            _ => return Err(Refusal::unsupported(r,format!("a surface of the kind {kind}"))),
        };
        Ok(Sheet {surface,scale,seat:None})
    }
    /// **A B-spline surface**: the simple entity of its knots or of a kind implying them
    /// (`UNIFORM_SURFACE`, `QUASI_UNIFORM_SURFACE`, `BEZIER_SURFACE`), or the complex entity of
    /// one of those with `B_SPLINE_SURFACE` and its weights; clamped over its domain.
    fn net(&self,r: u32) -> Result<Net,Refusal> {
        let e = self.file.get(r);
        let abstract_ = || Refusal::malformed(r,"a B-spline surface of no kind of knots (`B_SPLINE_SURFACE` is an abstract supertype)");
        let (head,spec,weights) = match e.simple() {
            Some(("B_SPLINE_SURFACE_WITH_KNOTS",v)) => {
                if v.len() != 13 { return Err(Refusal::malformed(r,"a B-spline surface of thirteen parameters expected")) }
                (&v[1..8],Spec::Written(&v[8..12]),None)
            }
            Some((k @ ("UNIFORM_SURFACE" | "QUASI_UNIFORM_SURFACE" | "BEZIER_SURFACE"),v)) => {
                if v.len() != 8 { return Err(Refusal::malformed(r,format!("a {k} of eight parameters expected"))) }
                (&v[1..8],Spec::Implied(family(k)),None)
            }
            Some(("B_SPLINE_SURFACE",_)) => return Err(abstract_()),
            Some((kind,_)) => return Err(Refusal::unsupported(r,format!("a surface of the kind {kind}"))),
            None => {
                let Some(s) = e.part("B_SPLINE_SURFACE").filter(|s| s.len() == 7) else { return Err(Refusal::malformed(r,"a B-spline surface whose parts do not read")) };
                let spec = match (e.part("B_SPLINE_SURFACE_WITH_KNOTS"),["UNIFORM_SURFACE","QUASI_UNIFORM_SURFACE","BEZIER_SURFACE"].into_iter().find(|k| e.is(k))) {
                    (Some(k),_) if k.len() == 5 => Spec::Written(&k[..4]),
                    (Some(_),_) => return Err(Refusal::malformed(r,"a B-spline surface's knots of five parameters expected")),
                    (None,Some(k)) => Spec::Implied(family(k)),
                    (None,None) => return Err(abstract_()),
                };
                (s,spec,e.part("RATIONAL_B_SPLINE_SURFACE"))
            }
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
        let (nu,nv) = (poles.len(),poles.first().map_or(0,Vec::len));
        let (uknots,vknots) = match spec {
            Spec::Written(ks) => (knots(list(ks.first(),r,"multiplicities")?,list(ks.get(2),r,"knots")?,r)?,
                knots(list(ks.get(1),r,"multiplicities")?,list(ks.get(3),r,"knots")?,r)?),
            Spec::Implied(k) => (implied(k,du,nu,r)?,implied(k,dv,nv,r)?),
        };
        let weights = match weights {
            Some(w) => Some(list(w.first(),r,"a B-spline surface's weights")?.iter().map(|row| {
                list(Some(row),r,"a row of weights")?.iter().map(|x| number(Some(x),r,"a weight")).collect()
            }).collect::<Result<Vec<Vec<f64>>,_>>()?),
            None => None,
        };
        let net = Net {du,dv,uknots,vknots,poles,weights};
        net.check().map_err(|m| Refusal::malformed(r,m))?;
        // (its knots clamped over its domain where they are not, exactly)
        if is_clamped(&net.uknots,du,nu) && is_clamped(&net.vknots,dv,nv) { return Ok(net) }
        let [u,v] = net.domain();
        let net = net.segment(u,v);
        net.check().map_err(|m| Refusal::malformed(r,m))?;
        Ok(net)
    }
    /// A linear extrusion, `C(u) + v z`: its curve carried along `z` into the plane square to it
    /// through the curve's first point — the same surface, its curve an affine image (a spline's
    /// poles, a line's point and direction, a conic slanted to `z` the ellipse it projects to,
    /// `slanted`).
    fn extrusion(&self,r: u32,v: &[Value]) -> Result<Sheet,Refusal> {
        let swept = self.curve(r,v.get(1).and_then(Value::reference))?;
        let (d,m) = self.vector(r,v.get(2).and_then(Value::reference),false)?;
        if m == 0. { return Err(Refusal::malformed(r,"an extrusion along a vector of no length")) }
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
            Curve::Circle(f,rad) => slanted(&f,rad,rad,z,flat(f.o)).ok_or_else(|| along_plane(r))?,
            Curve::Ellipse(f,s1,s2) => slanted(&f,s1,s2,z,flat(f.o)).ok_or_else(|| along_plane(r))?,
            other => return Err(Refusal::unsupported(r,format!("an extrusion of a {} curve",other.kind()))),
        };
        Ok(Sheet {surface:Surface::Extrusion(Frame::about(o,z),Arc::new(curve)),scale:rates,seat:None})
    }
    /// A surface of revolution, its curve turned about the axis: the frame's `x` toward the curve,
    /// where it lies in the half-plane of the axis it points to (the kernel turns a meridian); a
    /// curve leaving that half-plane — a circle beside the axis, not about it — turned as a
    /// rational net (`turned`, `Seat::Turned`), the same surface exactly; a line leaving it the
    /// same, bounded past its faces (`Seat::Throat`), or where it meets the axis, the half of it its
    /// faces lie on (`Seat::Nappe`).
    fn revolution(&self,r: u32,v: &[Value]) -> Result<Sheet,Refusal> {
        let (a,_,axis) = self.of(r,v.get(2).and_then(Value::reference),&["AXIS1_PLACEMENT"],"a revolution's axis")?;
        let o = self.point(a,axis.get(1).and_then(Value::reference))?;
        let z = match optional(axis.get(2),a,"an axis's direction")? { Some(d) => self.direction(a,Some(d))?,None => [0.,0.,1.] };
        if let Some(c) = v.get(1).and_then(Value::reference) {
            if let Some((kind @ ("PARABOLA" | "HYPERBOLA"),cv)) = self.file.get(c).simple() {
                let g = self.placement(c,cv.get(1).and_then(Value::reference))?;
                let conic = if kind == "PARABOLA" { Open::Parabola {g,focal:self.length(cv.get(2),c,"a parabola's focal distance")?} }
                    else { Open::Hyperbola {g,a:self.length(cv.get(2),c,"a hyperbola's semi-axis")?,b:self.length(cv.get(3),c,"a hyperbola's imaginary semi-axis")?} };
                return self.meridian(r,o,z,conic)
            }
        }
        let swept = self.curve(r,v.get(1).and_then(Value::reference))?;
        // points the curve lies within: a spline's poles (it lies in their hull), else samples
        let points: Vec<V> = match &swept.curve {
            Curve::BSpline(b) => b.poles.clone(),
            Curve::Line {p,d} => vec![*p,add(*p,*d)],
            c => (0..64).map(|k| c.point(TAU*k as f64/64.)).collect(),
        };
        let radial = |p: V| { let w = sub(p,o); sub(w,scale(z,dot(w,z))) };
        let far = points.iter().map(|&p| radial(p)).max_by(|a,b| norm(*a).total_cmp(&norm(*b))).unwrap_or([0.;3]);
        let x = normalised(far).filter(|_| norm(far) > 0.).ok_or_else(|| Refusal::unsupported(r,"a revolution of a curve on its axis"))?;
        let f = Frame::new(o,z,x);
        let size = points.iter().map(|&p| norm(sub(p,o))).fold(0.,f64::max);
        let within = 1e-9*size+self.context.uncertainty.unwrap_or(0.);
        // (a line not along the axis meets its plane's every half-plane but one: each of its halves
        // the side its faces lie on, or the hyperboloid it sweeps off the plane)
        if let Curve::Line {p,d} = swept.curve {
            if norm(cross(d,z)) > 1e-12 { return line_turned(r,o,z,p,d,within,[self.context.angle,swept.rate]) }
        }
        if points.iter().all(|&p| { let l = f.local(p); l[1].abs() <= within && l[0] >= -within }) {
            return Ok(Sheet {surface:Surface::Revolution(f,Arc::new(swept.curve)),scale:[self.context.angle,swept.rate],seat:None})
        }
        // a closed conic crossing the axis in a plane of it (a lemon's circle, a ball swung across
        // its own centre line): the revolution of its arc on the side its faces lie on (`Seat::Crossing`)
        if points.iter().all(|&p| f.local(p)[1].abs() <= within) && matches!(swept.curve,Curve::Circle(..) | Curve::Ellipse(..)) {
            let mut sheet = Seat::Crossing {f,curve:swept.curve}.sheet(r)?;
            sheet.scale = UNMAPPED;
            return Ok(sheet)
        }
        // any other curve crossing the axis in a plane of it: its net kept as the file's, whole
        // turns from where its parameters start, never seated
        if points.iter().all(|&p| f.local(p)[1].abs() <= within) {
            let net = turned(&f,&swept.curve,[0.,TAU],0.).ok_or_else(|| Refusal::unsupported(r,"a revolution of a curve the reader cannot make a net of"))?;
            net.check().map_err(|m| Refusal::malformed(r,m))?;
            return Ok(Sheet {surface:Surface::BSpline(Frame::WORLD,Arc::new(net)),scale:UNMAPPED,seat:None})
        }
        // a curve leaving the axis's plane: the rational net that is its revolution exactly
        Seat::Turned {f,curve:swept.curve,offset:0.}.sheet(r)
    }
    /// **An open conic turned about the axis through `o` along `z`** (`Seat::Meridian`), refused
    /// where its plane does not hold the axis. Where it crosses the axis its two halves turn to
    /// the same points facing opposite ways: the half is the one the file's curves in its faces'
    /// parameters lie on (the sign of their `v`, the conic's parameter, at their ends and middle),
    /// refused where they say none or both.
    fn meridian(&self,r: u32,o: V,z: V,conic: Open) -> Result<Sheet,Refusal> {
        let g = *conic.frame();
        let within = 1e-9*(1.+norm(sub(g.o,o)))+self.context.uncertainty.unwrap_or(0.);
        if dot(g.z,z).abs() > 1e-9 || dot(sub(o,g.o),g.z).abs() > within {
            return Err(Refusal::unsupported(r,"a revolution of a parabola or a hyperbola off its axis's plane"))
        }
        let e = normalised(cross(z,g.z)).ok_or_else(|| Refusal::malformed(r,"a conic square to its axis"))?;
        let side = match conic.one_sided(o,e) {
            Some(s) => s,
            None => {
                let mut signs = Vec::new();
                for pc in self.file.all("PCURVE") {
                    let Some((_,pv)) = pc.simple() else { continue };
                    if pv.get(1).and_then(Value::reference) != Some(r) { continue }
                    let Ok((d,_,dv)) = self.of(pc.id,pv.get(2).and_then(Value::reference),&["DEFINITIONAL_REPRESENTATION"],"a pcurve's curve") else { continue };
                    let Ok(flat) = self.flat(d,list(dv.get(1),d,"a representation's items")?.first().and_then(Value::reference)) else { continue };
                    let ts = match flat.domain() { Some([a,b]) => [a,0.5*(a+b),b],None => [0.,0.5,1.] };
                    for t in ts {
                        let x = dot(sub(conic.point(flat.at(t)[1]),o),e);
                        if x.abs() > within { signs.push(x.signum()); }
                    }
                }
                match signs.first() {
                    Some(&s) if signs.iter().all(|&x| x == s) => s,
                    _ => return Err(Refusal::unsupported(r,"a revolution of a conic crossing its axis whose faces' half no curve in their parameters says")),
                }
            }
        };
        Seat::Meridian {f:Frame::new(o,z,scale(e,side)),conic}.sheet(r)
    }
    /// **An offset surface** `S + d N`, `N` its basis's unit normal (`S_u × S_v`): of a plane, a
    /// sphere, a cylinder, a cone or a torus the surface of the same kind it is; of a spline sheet
    /// the kernel's `Surface::Offset`; of a curve swept or turned, the kernel's offset of its net
    /// (`Seat`). Refused where its writer says it intersects itself, and where it reaches through
    /// its basis's axis or centre (its normal turned).
    fn offset(&self,r: u32,v: &[Value],depth: usize) -> Result<Sheet,Refusal> {
        let basis = self.surface_in(r,v.get(1).and_then(Value::reference),depth+1)?;
        let d = number(v.get(2),r,"an offset's distance")?*self.context.length;
        match v.get(3).and_then(Value::word) {
            Some("T") => return Err(Refusal::unsupported(r,"an offset surface its writer says intersects itself")),
            Some("F" | "U") => {}
            _ => return Err(Refusal::malformed(r,"an offset's self-intersection: .T., .F. or .U. expected")),
        }
        if d == 0. { return Ok(basis) }
        if let Some(seat) = &basis.seat {
            return match &**seat {
                Seat::Turned {f,curve,offset} => Seat::Turned {f:*f,curve:curve.clone(),offset:offset+d}.sheet(r),
                Seat::Swept {f,curve,offset} => Seat::Swept {f:*f,curve:curve.clone(),offset:offset+d}.sheet(r),
                Seat::Crossing {..} => Err(Refusal::unsupported(r,"an offset of a conic turned across its axis: not read yet")),
                _ => Err(Refusal::unsupported(r,"an offset of a line turned off its axis's plane, which the kernel holds no form of")),
            }
        }
        let through = || Refusal::unsupported(r,"an offset reaching through its basis's axis or centre, its normal turned");
        let surface = match basis.surface {
            Surface::Plane(f) => Surface::Plane(Frame {o:add(f.o,scale(f.z,d)),..f}),
            Surface::Cylinder(f,rad) if rad+d > 0. => Surface::Cylinder(f,rad+d),
            Surface::Sphere(f,rad) if rad+d > 0. => Surface::Sphere(f,rad+d),
            Surface::Torus(f,big,rad) if rad+d > 0. => Surface::Torus(f,big,rad+d),
            Surface::Cylinder(..) | Surface::Sphere(..) | Surface::Torus(..) => return Err(through()),
            // the cone's point `(ρ, v cos α)` carried `d` along `(cos α, −sin α)`: the cone of the
            // same frame and half-angle whose radius at its origin is `r + d / cos α`
            Surface::Cone(f,rad,half) => cone(f,rad+d/half.dcos(),half),
            Surface::BSpline(f,n) => Surface::Offset(f,n,d),
            Surface::Offset(f,n,e) => Surface::Offset(f,n,e+d),
            Surface::Extrusion(f,c) => match &*c {
                // a plane: its line carried along the normal `line × z`
                Curve::Line {p,d:t} => {
                    let n = normalised(cross(*t,f.z)).ok_or_else(|| Refusal::malformed(r,"an extrusion of a line along itself"))?;
                    Surface::Extrusion(f,Arc::new(Curve::Line {p:add(*p,scale(n,d)),d:*t}))
                }
                c => return Seat::Swept {f,curve:c.clone(),offset:d}.sheet(r),
            },
            Surface::Revolution(f,c) => match &*c {
                // a cone, a cylinder or a plane turned: its meridian carried along the normal
                // `y × line` (the turn's direction crossed with it), held to its half-plane
                Curve::Line {p,d:t} => {
                    let n = normalised(cross(f.y,*t)).ok_or_else(|| Refusal::malformed(r,"a revolution of a line along its turn"))?;
                    let q = add(*p,scale(n,d));
                    if f.local(q)[0] < 0. || f.local(add(q,*t))[0] < 0. { return Err(through()) }
                    Surface::Revolution(f,Arc::new(Curve::Line {p:q,d:*t}))
                }
                c => return Seat::Turned {f,curve:c.clone(),offset:d}.sheet(r),
            },
            other => return Err(Refusal::unsupported(r,format!("an offset of a {} surface",other.kind()))),
        };
        Ok(Sheet {surface,..basis})
    }
}

/// A conic swept along its own plane: the strip it sweeps there covered twice, its normal turning
/// over across it (`S_u × S_v` is `C′ × z`, along the plane's normal one way and then the other) —
/// which half a face lies on only its curves in the face's parameters could say: not read.
fn along_plane(r: u32) -> Refusal {
    Refusal::unsupported(r,"an extrusion of a conic along its own plane: a strip covered twice, facing both ways")
}

/// **A cone `o + (r + v sin α)(cos u x + sin u y) + v cos α z` as the kernel holds it**: as it
/// is where `r ≥ 0`; where `r < 0` (its apex past its origin along `z`), its origin moved to its
/// apex, radius 0 — the same points at the same `u`, `v` moved along itself, so its normal
/// `S_u × S_v` is kept. (The standard asks `r ≥ 0` of a `CONICAL_SURFACE`, but its points and their
/// normal are the formula's for any `r`: read as written.)
fn cone(f: Frame,r: f64,half: f64) -> Surface {
    if r >= 0. { return Surface::Cone(f,r,half) }
    let (s,c) = half.dsin_cos();
    Surface::Cone(Frame {o:add(f.o,scale(f.z,-r*c/s)),..f},0.,half)
}

/// **A line `p + t d` turned about the axis through `o` along `z`**, not along it: refused where it
/// is square to the axis (a plane or a ring, which each half of it turns to facing opposite ways:
/// which a face lies on, only its curves in the face's parameters could say); where it meets the
/// axis — its point nearest the axis on it — the half of it on the faces' side (`Seat::Nappe`), its
/// parameters the file's (`rates`); else the hyperboloid it sweeps, as a net (`Seat::Throat`).
fn line_turned(r: u32,o: V,z: V,p: V,d: V,within: f64,rates: [f64;2]) -> Result<Sheet,Refusal> {
    let c = dot(d,z);
    let perp = sub(d,scale(z,c));
    let s = norm(perp);
    if !(s > 1e-12) { return Err(Refusal::unsupported(r,"a revolution of a line parallel to its axis, off its axis's plane")) }
    let w = sub(p,o);
    let t = -dot(sub(w,scale(z,dot(w,z))),perp)/(s*s);
    let near = add(p,scale(d,t));
    let throat = sub(sub(near,o),scale(z,dot(sub(near,o),z)));
    let a = norm(throat);
    if c.abs() <= 1e-12 { return Err(Refusal::unsupported(r,"a revolution of a line square to its axis: a plane or a ring each half of the line turns to, facing opposite ways")) }
    if a <= within {
        let mut sheet = Seat::Nappe {apex:add(o,scale(z,dot(sub(near,o),z))),z,p,d}.sheet(r)?;
        sheet.scale = rates;
        return Ok(sheet)
    }
    Seat::Throat {f:Frame::new(o,z,throat),p:near,d}.sheet(r)
}

impl Seat {
    /// The sheet this seat gives before its faces are known: its seams where the file's are, its
    /// bounds a unit each way of where the file's parameters start.
    fn sheet(self,r: u32) -> Result<Sheet,Refusal> {
        let surface = self.surface([0.,TAU],0.,[-1.,1.]).ok_or_else(|| Refusal::unsupported(r,"a surface swept or turned from a curve the reader cannot make a net of"))?;
        if let Surface::BSpline(_,n) | Surface::Offset(_,n,_) = &surface { n.check().map_err(|m| Refusal::malformed(r,m))?; }
        Ok(Sheet {surface,scale:[1.,1.],seat:Some(Arc::new(self))})
    }
    /// Its surface turned over `turn`, a closed conic's spline starting at its angle `from`, and
    /// bounded over `span` (along a sweep or a turned line; a nappe's side the sign of its end).
    fn surface(&self,turn: [f64;2],from: f64,span: [f64;2]) -> Option<Surface> {
        let off = |n: Net,d: f64| if d == 0. { Surface::BSpline(Frame::WORLD,Arc::new(n)) } else { Surface::Offset(Frame::WORLD,Arc::new(n),d) };
        Some(match self {
            Seat::Turned {f,curve,offset} => off(turned(f,curve,turn,from)?,*offset),
            Seat::Swept {f,curve,offset} => off(swept(f,curve,from,span)?,*offset),
            Seat::Throat {f,p,d} => {
                let [t0,t1] = span;
                let line = BSpline {degree:1,knots:vec![t0,t0,t1,t1],poles:vec![add(*p,scale(*d,t0)),add(*p,scale(*d,t1))],weights:None};
                off(turned(f,&Curve::BSpline(Arc::new(line)),turn,0.)?,0.)
            }
            Seat::Meridian {f,conic} => Surface::Revolution(*f,Arc::new(Curve::BSpline(Arc::new(conic.arc(span))))),
            Seat::Crossing {f,curve} => off(turned(f,curve,[0.,TAU],0.)?,0.),
            Seat::Nappe {apex,z,p,d} => {
                let side = if span[1] < 0. { -1. } else { 1. };
                let x = scale(sub(*d,scale(*z,dot(*d,*z))),side);
                Surface::Revolution(Frame::new(*apex,*z,x),Arc::new(Curve::Line {p:*p,d:*d}))
            }
        })
    }
}

impl Sheet {
    /// **The sheet seated on its faces** (`Seat`): `points` their edges' and vertices' points,
    /// `seams` those of each edge a face uses twice. A net's seam, in each parameter it closes in,
    /// where the file's is if a face's seam runs along it; else a turn only over the angles the
    /// points take, a quarter of the widest gap they leave past each of its ends (a net of fewer
    /// spans, its feet found more surely), a closed conic's seam in the middle of that gap; a
    /// sweep's or a turned line's bounds a tenth past theirs each way; a nappe the half they lie
    /// on, refused where they lie on both.
    pub fn seated(&self,r: u32,points: &[V],seams: &[Vec<V>]) -> Result<Sheet,Refusal> {
        let Some(seat) = &self.seat else { return Ok(self.clone()) };
        if points.is_empty() { return Ok(self.clone()) }
        let s = &self.surface;
        // (a parameter's seam kept where a face's seam runs along it, the file's: every point of
        // the seam at an end of the parameter's span `ends`, `param` the point's parameter)
        let along_seam = |param: &dyn Fn(V) -> f64,[a,b]: [f64;2]| seams.iter().any(|seam| seam.iter().all(|&p| {
            let x = param(p);
            (x-a).abs().min((b-x).abs()) <= 1e-6*(b-a)
        }));
        let gap = |param: &dyn Fn(V) -> f64,ends: [f64;2],angle: &dyn Fn(f64) -> f64|
            if along_seam(param,ends) { None } else { widest_gap(points.iter().map(|&p| angle(param(p))).collect()) };
        let domain = match s { Surface::BSpline(_,n) | Surface::Offset(_,n,_) => n.domain(),_ => [[0.,TAU];2] };
        let turn = |g: Option<(f64,f64)>| g.map_or([0.,TAU],|(a,w)| [a+0.75*w,a+TAU+0.25*w]);
        let middle = |g: Option<(f64,f64)>| g.map_or(0.,|(a,w)| (a+0.5*w).rem_euclid(TAU));
        let conic = |c: &Curve| matches!(c,Curve::Circle(..) | Curve::Ellipse(..));
        let size = points.iter().map(|&p| norm(p)).fold(0.,f64::max);
        if let Seat::Crossing {f,curve} = &**seat {
            let scale = 1.+points.iter().map(|&p| norm(sub(p,f.o))).fold(0.,f64::max);
            let Some(arc) = crossing_side(f,curve,points,scale) else { return Ok(Sheet {seat:None,..self.clone()}) };
            return Ok(Sheet {surface:Surface::Revolution(arc.0,Arc::new(Curve::BSpline(Arc::new(arc.1)))),scale:UNMAPPED,seat:None})
        }
        let (turn,from,span) = match &**seat {
            Seat::Crossing {..} => unreachable!("seated above"),
            Seat::Turned {curve,..} => {
                let u = turn(gap(&|p| s.inverse(p)[0],domain[0],&|x| angle_at(x,0.)));
                (u,if conic(curve) { middle(gap(&|p| s.inverse(p)[1],domain[1],&|x| angle_at(x,0.))) } else { 0. },[-1.,1.])
            }
            Seat::Swept {f,curve,..} => {
                let along = |p: V| dot(sub(p,f.o),f.z);
                let u = if conic(curve) { middle(gap(&|p| curve.inverse(sub(p,scale(f.z,along(p)))).rem_euclid(TAU),[0.,TAU],&|x| x)) } else { 0. };
                ([0.,TAU],u,spread(points.iter().map(|&p| along(p)),1e-6*(1.+size)).unwrap_or([-1.,1.]))
            }
            Seat::Throat {f,p,d} => {
                // (a point's `t` its height along the axis over the line's rise, its turn from the
                // line's point there the difference of their bearings about the axis)
                let t = |q: V| dot(sub(q,*p),f.z)/dot(*d,f.z);
                let bearing = |q: V| { let l = f.local(q); l[1].datan2(l[0]) };
                let u = |q: V| (bearing(q)-bearing(add(*p,scale(*d,t(q))))).rem_euclid(TAU);
                (turn(gap(&u,[0.,TAU],&|x| x)),0.,spread(points.iter().map(|&q| t(q)),1e-6*(1.+size)).unwrap_or([-1.,1.]))
            }
            Seat::Meridian {f,conic} => {
                // each point carried round the axis into the half-plane, read on the arc; its
                // bounds a millionth past theirs, but not across the axis
                let u: Vec<f64> = points.iter().map(|&p| { let l = f.local(p); conic.parameter(f.at([l[0].dhypot(l[1]),0.,l[2]])) }).collect();
                let (lo,hi) = u.iter().filter(|x| x.is_finite()).fold((f64::INFINITY,f64::NEG_INFINITY),|(lo,hi),&x| (lo.min(x),hi.max(x)));
                if !(lo <= hi) { return Err(Refusal::unsupported(r,"a revolution's faces read on no point of its curve")) }
                let pad = 1e-6*(hi-lo)+1e-9;
                let held = |x: f64,y: f64| if f.local(conic.point(x))[0] >= 0. { x } else { y };
                ([0.,TAU],0.,[held(lo-pad,lo),held(hi+pad,hi)])
            }
            Seat::Nappe {apex,z,d,..} => {
                let t: Vec<f64> = points.iter().map(|&p| dot(sub(p,*apex),*z)/dot(*d,*z)).collect();
                let tiny = 1e-9*(1.+size);
                let side = if t.iter().all(|&x| x >= -tiny) { 1. } else if t.iter().all(|&x| x <= tiny) { -1. } else {
                    return Err(Refusal::unsupported(r,"faces of a line turned about an axis it meets, on both sides of where it meets it"))
                };
                ([0.,TAU],0.,[side,side])
            }
        };
        let surface = seat.surface(turn,from,span).ok_or_else(|| Refusal::unsupported(r,"a surface swept or turned from a curve the reader cannot make a net of"))?;
        if let Surface::BSpline(_,n) | Surface::Offset(_,n,_) = &surface { n.check().map_err(|m| Refusal::malformed(r,m))?; }
        // (a net's parameters are its rational arc's and its conic spline's, from where its seams
        // were put: no scaling of the file's, whose curves in its faces' parameters lie elsewhere)
        let scale = match &**seat { Seat::Turned {..} | Seat::Swept {..} | Seat::Throat {..} => UNMAPPED,_ => self.scale };
        Ok(Sheet {surface,scale,seat:None})
    }
}

/// **The arc of a closed conic crossing the axis `f.z` in a plane of it that its faces lie on**
/// (`Seat::Crossing`), and the frame it is turned in: each of `points` carried round the axis into
/// the conic's plane, `ρ` from the axis at its height, on the conic at `+ρ` along `f.x` or at `−ρ`
/// (whichever stands nearer it, by more than a millionth of `size` and ten times the other),
/// points on the axis saying nothing. Its arc between its two crossings on that side, exactly (a
/// circle's `nurbs::arc`, an ellipse's the unit arc stretched), run the way the conic runs, so the
/// revolution's normal is the file's; turned in the frame whose `x` points to that side. None
/// where the points lie on both sides or on neither, or the conic does not cross the axis.
fn crossing_side(f: &Frame,curve: &Curve,points: &[V],size: f64) -> Option<(Frame,BSpline)> {
    let (g,a,b) = match *curve { Curve::Circle(g,r) => (g,r,r),Curve::Ellipse(g,a,b) => (g,a,b),_ => return None };
    let mut side = 0.;
    for &p in points {
        let l = f.local(p);
        let rho = l[0].dhypot(l[1]);
        if rho <= 1e-6*size { continue }
        let off = |s: f64| { let q = f.at([s*rho,0.,l[2]]); distance(q,curve.point(curve.inverse(q))) };
        let (dp,dm) = (off(1.),off(-1.));
        let s = if dp+1e-6*size <= dm && 10.*dp <= dm { 1. } else if dm+1e-6*size <= dp && 10.*dm <= dp { -1. } else { continue };
        if side != 0. && side != s { return None }
        side = s;
    }
    if side == 0. { return None }
    // its signed reach along `f.x` at its angle `t`: `C + A cos t + B sin t`, nought at two angles
    let (c,ca,cb) = (dot(sub(g.o,f.o),f.x),a*dot(g.x,f.x),b*dot(g.y,f.x));
    let big = ca.dhypot(cb);
    if !(c.abs() < big) { return None }
    let (phi,half) = (cb.datan2(ca),(-c/big).clamp(-1.,1.).acos());
    // the arc from the one crossing on to the other whose middle lies on the faces' side
    let (t0,t1) = (phi+half,phi+TAU-half);
    let middle = 0.5*(t0+t1);
    let (t0,t1) = if (c+ca*middle.dcos()+cb*middle.dsin())*side > 0. { (t0,t1) } else { (phi-half,phi+half) };
    let unit = arc(&Frame::WORLD,1.,[t0,t1]);
    let spline = BSpline {poles:unit.poles.iter().map(|p| g.at([a*p[0],b*p[1],0.])).collect(),..unit};
    Some((Frame::new(f.o,f.z,scale(f.x,side)),spline))
}

/// **A curve turned once about `f.z` as the rational net that is exactly its surface of
/// revolution** (Piegl and Tiller A8.1): each of its poles turned by each of a whole turn's rational
/// arc's (`nurbs::arc`, its parameter meeting the angle at every knot) — a turn and a stretch in the
/// plane square to the axis, linear in the pole — weighed the product of the two. The turn runs
/// over the angles `turn` from the curve (a whole turn at most), a conic's own spline from its
/// angle `from`. A line, which runs on without end, has none.
pub fn turned(f: &Frame,c: &Curve,turn: [f64;2],from: f64) -> Option<Net> {
    let s = match c {
        Curve::BSpline(b) => (**b).clone(),
        c => conic_spline(c,from)?,
    };
    let round = arc(&Frame::WORLD,1.,turn);
    let (wu,wv) = (round.weights.clone()?,s.weights.clone().unwrap_or_else(|| vec![1.;s.poles.len()]));
    let poles = round.poles.iter().map(|q| s.poles.iter().map(|&p| { let l = f.local(p); f.at([l[0]*q[0]-l[1]*q[1],l[0]*q[1]+l[1]*q[0],l[2]]) }).collect()).collect();
    let weights = wu.iter().map(|a| wv.iter().map(|b| a*b).collect()).collect();
    Some(Net {du:2,dv:s.degree,uknots:round.knots.clone(),vknots:s.knots.clone(),poles,weights:Some(weights)})
}

/// **A curve in the plane through `f.o` square to `f.z` swept along it over `[v0, v1]`** as the
/// net that is exactly that stretch of its extrusion: its poles (a closed conic's spline from its
/// angle `from`) at either end, weighed alike, its `v` the length along `f.z`.
fn swept(f: &Frame,c: &Curve,from: f64,[v0,v1]: [f64;2]) -> Option<Net> {
    let s = match c { Curve::BSpline(b) => (**b).clone(),c => conic_spline(c,from)? };
    let poles = s.poles.iter().map(|&p| vec![add(p,scale(f.z,v0)),add(p,scale(f.z,v1))]).collect();
    let weights = s.weights.as_ref().map(|w| w.iter().map(|&x| vec![x,x]).collect());
    Some(Net {du:s.degree,dv:1,uknots:s.knots.clone(),vknots:vec![v0,v0,v1,v1],poles,weights})
}

/// **The ellipse a conic `o + a cos t x + b sin t y` of the frame `f` projects to along the unit
/// `z`**, centred at `centre`: the projection is linear, `M (cos t, sin t)` in the plane square to
/// `z`, and `M = U Σ Vᵀ` (its singular values the semi-axes, `U`'s columns the axes) is the same
/// ellipse turned by a constant in its parameter — `V` taken a turn, never a reflection, so it runs
/// the way the conic did and the surface's normal is kept. None where the conic's plane holds `z`.
fn slanted(f: &Frame,a: f64,b: f64,z: V,centre: V) -> Option<Curve> {
    let g = Frame::about([0.;3],z);
    let flat = |d: V| { let d = sub(d,scale(z,dot(d,z))); [dot(d,g.x),dot(d,g.y)] };
    let (p,q) = (flat(scale(f.x,a)),flat(scale(f.y,b)));
    // `M = [p q]`; `MᵀM`'s eigenvectors are `V`'s columns
    let (m11,m12,m22) = (p[0]*p[0]+p[1]*p[1],p[0]*q[0]+p[1]*q[1],q[0]*q[0]+q[1]*q[1]);
    let angle = 0.5*(2.*m12).datan2(m11-m22);
    let (s,c) = angle.dsin_cos();
    let (v1,v2) = ([c,s],[-s,c]);
    let image = |v: [f64;2]| [p[0]*v[0]+q[0]*v[1],p[1]*v[0]+q[1]*v[1]];
    let (u1,u2) = (image(v1),image(v2));
    let (s1,s2) = (u1[0].dhypot(u1[1]),u2[0].dhypot(u2[1]));
    if !(s1 > 1e-12*(a+b) && s2 > 1e-12*(a+b)) { return None }
    let x = normalised(add(scale(g.x,u1[0]),scale(g.y,u1[1])))?;
    let y = normalised(add(scale(g.x,u2[0]),scale(g.y,u2[1])))?;
    Some(Curve::Ellipse(Frame {o:centre,x,y,z:cross(x,y)},s1,s2))
}
