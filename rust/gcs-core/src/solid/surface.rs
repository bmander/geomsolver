//! Exact generating patches of existing declarative solids. These retain the solved
//! profile's lines and circles; no facets enter their position or derivative evaluation.
//! Boundary trimming and outward orientation remain separate questions.
#[allow(unused_imports)]
use crate::fmath::Det;
pub(super) mod contact;
pub use contact::RevolvedContact;
mod project;
pub use project::{SurfaceProjection,SurfaceProjector};
mod region;
pub use region::{RegionLocation,RegionSample,RevolvedRegion};
pub(crate) use region::Edge;

use crate::envelope::{Error, Motion, SurfacePoint};
use crate::model::{EntKind, EntRef, Sense, Sketch, SolidDef};
use crate::plane::{self, Basis};

use crate::space::{add, sub, scale};

type V = [f64;3];

#[derive(Clone, Debug)]
enum Meridian {
    Line { start: V, delta: V },
    Round { center: V, a: V, b: V, sweep: f64 },
    /// A conic's arc: the rational quadratic Bézier over `a`, `m`, `b`, the middle weighed `w`.
    Conic { a: V, m: V, b: V, w: f64 },
}

impl Meridian {
    /// An arc of radius `r` about `center`, from `start` counter-clockwise through `sweep` in the
    /// plane whose directions are `u` and `v`.
    fn round(center: V,u: V,v: V,r: f64,start: f64,sweep: f64) -> Meridian {
        let (s,c) = start.dsin_cos();
        let at = |x: f64,y: f64| add(scale(u,x),scale(v,y));
        Meridian::Round { center,a: at(r*c,r*s),b: at(-r*s,r*c),sweep }
    }
}

/// A snapshot of one named side of a revolution. `u` traverses the source edge from
/// 0 to 1; `v` traverses the declared revolution from 0 to 1. A named angular span
/// restricts this original chart; `domain()` gives its retained bounds. Tangents follow the source
/// edge and sweep directions; they are not necessarily the solid's outward orientation.
/// Re-read after changing the sketch: this object owns solved coordinates, not a live cache.
#[derive(Clone, Debug)]
pub struct RevolvedSurface {
    pub name: String,
    meridian: Meridian,
    origin: V,
    axis: V,
    sweep: f64,
    v_domain: [f64;2],
}

impl RevolvedSurface {
    /// Bounds for the generating profile before revolution, in solved world
    /// coordinates. The coefficients of this owned snapshot are treated as exact
    /// binary64 data; solve error and correspondence to the intended dimensions
    /// are separate. This does not bound the swept or generated envelope surface.
    pub fn generating_profile_bounds(&self,u: crate::interval::Interval)
        -> Result<([crate::interval::Interval;3],[crate::interval::Interval;3]),crate::interval::Error> {
        let [p,d,_] = self.profile_bounds(u,false)?;
        Ok((p,d))
    }

    /// Position, du and duu enclosures for the pre-revolution generating profile.
    /// This has the same snapshot and finite-domain contract as
    /// `generating_profile_bounds`; derivatives are with respect to normalized u.
    pub fn generating_profile_jet_bounds(&self,u: crate::interval::Interval)
        -> Result<[[crate::interval::Interval;3];3],crate::interval::Error> {
        self.profile_bounds(u,true)
    }

    fn profile_bounds(&self,u: crate::interval::Interval,second_order: bool)
        -> Result<[[crate::interval::Interval;3];3],crate::interval::Error> {
        use crate::interval::{Error,Interval as I};
        let [lo,hi] = u.bounds();
        if lo < 0. || hi > 1. { return Err(Error::OutsideDomain); }
        let (mut position,mut derivative,mut second) = ([I::ZERO;3],[I::ZERO;3],[I::ZERO;3]);
        match &self.meridian {
            Meridian::Line {start,delta} => {
                for i in 0..3 {
                    derivative[i] = I::point(delta[i])?;
                    position[i] = I::point(start[i])?.add(derivative[i].mul(u)?)?;
                }
            }
            Meridian::Round {center,a,b,sweep} => {
                let sweep = I::point(*sweep)?;
                let (s,c) = u.mul(sweep)?.sin_cos()?;
                for i in 0..3 {
                    let (a,b) = (I::point(a[i])?,I::point(b[i])?);
                    position[i] = I::point(center[i])?.add(a.mul(c)?.add(b.mul(s)?)?)?;
                    derivative[i] = a.neg().mul(s)?.add(b.mul(c)?)?.mul(sweep)?;
                    if second_order {
                        second[i] = a.mul(c)?.add(b.mul(s)?)?.neg().mul(sweep.square()?)?;
                    }
                }
            }
            Meridian::Conic {a,m,b,w} => {
                // the quotient rule over N = Σ kᵢ pᵢ and D = Σ kᵢ, the Bernstein weights' own
                // enclosures: C = N/D, C′ = (N′ − C D′)/D, C″ = (N″ − 2C′D′ − C D″)/D
                let one = I::point(1.)?;
                let (s,w) = (one.sub(u)?,I::point(*w)?);
                let k = [s.square()?,I::point(2.)?.mul(w)?.mul(u)?.mul(s)?,u.square()?];
                let dk = [I::point(-2.)?.mul(s)?,I::point(2.)?.mul(w)?.mul(one.sub(I::point(2.)?.mul(u)?)?)?,
                    I::point(2.)?.mul(u)?];
                let ddk = [I::point(2.)?,I::point(-4.*w.bounds()[0])?,I::point(2.)?]; // w is a point
                let sum = |x: [I;3]| -> Result<I,Error> { x[0].add(x[1])?.add(x[2]) };
                let (d,dd,ddd) = (sum(k)?,sum(dk)?,sum(ddk)?);
                for i in 0..3 {
                    let p = [I::point(a[i])?,I::point(m[i])?,I::point(b[i])?];
                    let dot = |x: [I;3]| -> Result<I,Error> { x[0].mul(p[0])?.add(x[1].mul(p[1])?)?.add(x[2].mul(p[2])?) };
                    position[i] = dot(k)?.div(d)?;
                    derivative[i] = dot(dk)?.sub(position[i].mul(dd)?)?.div(d)?;
                    if second_order {
                        second[i] = dot(ddk)?.sub(I::point(2.)?.mul(derivative[i])?.mul(dd)?)?
                            .sub(position[i].mul(ddd)?)?.div(d)?;
                    }
                }
            }
        }
        Ok([position,derivative,second])
    }

    /// Read a named model surface against the sketch's current solved state.
    pub fn named(sk: &Sketch, surface: usize) -> Result<Self, String> {
        let s = sk.surfaces.get(surface).ok_or("no such surface")?;
        let mut patch = Self::read(sk,s.solid as usize,s.edge)?;
        patch.name = s.name.clone();
        match s.span { Some(span) => patch.spanned(span),None => Ok(patch) }
    }

    /// Read a line, arc or circle of a revolution's profile, including hole boundaries.
    /// This checks the requested patch, not closure or validity of the entire solid.
    pub fn read(sk: &Sketch, solid: usize, edge: EntRef) -> Result<Self, String> {
        let solid = sk.solids.get(solid).ok_or("no such solid")?;
        let SolidDef::Revolve { face, axis, sweep, sense } = &solid.def else {
            return Err("an exact revolved patch requires a revolution".into());
        };
        if !sweep.value.is_finite() || sweep.value <= 0. {
            return Err("a revolution needs a finite positive sweep".into());
        }
        let (a2, b2) = sk.revolve_axis(*face, *axis)
            .ok_or("the revolution axis does not lie in the profile's plane")?;
        let face = sk.faces.get(*face as usize).ok_or("no such profile")?;
        let name = face.edges.iter().zip(&face.edge_names)
            .chain(face.holes.iter().flat_map(|h| h.edges.iter().zip(&h.edge_names)))
            .find_map(|(e,n)| (*e == edge).then_some(n))
            .ok_or("the requested edge is not a boundary of this revolution's profile")?;
        let basis = if let Some(p) = face.plane()? {
            sk.planes.get(p as usize).ok_or("no profile plane")?;
            sk.basis(p as usize)
        } else { Basis::page() };
        let lift = |q: (f64,f64)| basis.lift(q.0,q.1);
        let origin = lift(a2);
        let delta = sub(lift(b2),origin);
        let len = delta[0].dhypot(delta[1]).dhypot(delta[2]);
        if !len.is_finite() || len == 0. { return Err("degenerate revolution axis".into()); }
        let meridian = match edge.kind {
            EntKind::Line => {
                let line = sk.lines.get(edge.i()).ok_or("no such profile line")?;
                let start = lift(sk.point_xy(line.p1 as usize));
                Meridian::Line { start,delta: sub(lift(sk.point_xy(line.p2 as usize)),start) }
            }
            EntKind::Arc | EntKind::Circle => {
                let (center,radius,start,sweep) = if edge.kind == EntKind::Arc {
                    let a = sk.arcs.get(edge.i()).ok_or("no such profile arc")?;
                    let (start,end) = sk.arc_angles(edge.i());
                    (a.center,a.radius,start,end-start)
                } else {
                    let c = sk.circles.get(edge.i()).ok_or("no such profile circle")?;
                    (c.center,c.radius,0.,std::f64::consts::TAU)
                };
                let r = sk.params[radius as usize].value;
                if !r.is_finite() || r <= 0. || !sweep.is_finite() || sweep <= 0. {
                    return Err("a round profile edge needs positive finite radius and sweep".into());
                }
                Meridian::round(lift(sk.point_xy(center as usize)),basis.u,basis.v,r,start,sweep)
            }
            _ => return Err("exact revolved patches currently require lines, arcs or circles".into()),
        };
        let sweep = sweep.value.min(std::f64::consts::TAU)
            * if *sense == Sense::Cw { -1. } else { 1. };
        let patch = Self { name: format!("{}.{}",solid.name,name),meridian,origin,
            axis: scale(delta,1./len),sweep,v_domain:[0.,1.] };
        patch.at(0.,0.).map_err(|e| format!("invalid revolved surface: {e:?}"))?;
        Ok(patch)
    }

    /// A region solid's faces (§6.21): each step of its meridian turned once about its axis,
    /// named by the term it came from — but a line on the axis, which turns to no face.
    pub(crate) fn of_meridian(m: &super::region::Meridian,solid: &str) -> Vec<Self> {
        Self::meridian_loops(m,solid).into_iter().flatten().flatten().collect()
    }

    /// `of_meridian` loop by loop, in the meridian's order, a line on the axis `None`: the
    /// profile a region tool's sweep is read over, as a revolution's faces' loops are.
    pub(crate) fn meridian_loops(m: &super::region::Meridian,solid: &str)
        -> Vec<Vec<Option<Self>>> {
        use crate::brep::planar::Seg;
        let basis = Basis { u: m.seam,v: m.axis,o: m.origin };
        let at = |q: [f64;2]| basis.lift(q[0],q[1]);
        let on_axis = |q: [f64;2]| q[0].abs() <= crate::solid::cad::AXIS_TOLERANCE;
        m.region.loops.iter().map(|lp| lp.iter().map(|step| {
            let meridian = match step.seg {
                Seg::Line {a,b} if on_axis(a) && on_axis(b) => return None,
                Seg::Line {a,b} => Meridian::Line {start: at(a),delta: sub(at(b),at(a))},
                // a round edge runs counter-clockwise from its start
                Seg::Arc {c,r,a0,sweep} => Meridian::round(at(c),m.seam,m.axis,r,
                    if sweep < 0. { a0+sweep } else { a0 },sweep.abs()),
                Seg::Conic {a,m: c,b,w} => Meridian::Conic {a: at(a),m: at(c),b: at(b),w},
            };
            let name = format!("{solid}.{}",m.name(step.tag));
            Some(Self {name,meridian,origin: m.origin,axis: m.axis,sweep: std::f64::consts::TAU,
                v_domain: [0.,1.]})
        }).collect()).collect()
    }

    /// The surface over an angular span of its revolution (radians from its chart's start), as a
    /// declaration's `from:`/`to:` restricts it (#212).
    pub fn spanned(&self,span: [f64;2]) -> Result<Self,String> {
        let sweep = self.sweep.abs();
        if !span.iter().all(|v| v.is_finite()) || span[0] < 0. || span[0] >= span[1] || span[1] > sweep {
            return Err("a surface needs increasing angular bounds within its source sweep".into());
        }
        Ok(Self {v_domain:span.map(|a| a/sweep),..self.clone()})
    }

    /// Original source parameters: an angular span restricts v without renumbering it.
    pub fn domain(&self) -> [[f64;2];2] { [[0.,1.],self.v_domain] }

    /// The signed revolution angle the `v` chart spans.
    pub fn sweep(&self) -> f64 { self.sweep }

    pub fn at(&self, u: f64, v: f64) -> Result<SurfacePoint, Error> {
        if !u.is_finite() || !v.is_finite() { return Err(Error::NonFinite); }
        if !(0. ..=1.).contains(&u) || v < self.v_domain[0] || v > self.v_domain[1] {
            return Err(Error::OutsideDomain);
        }
        let (p,d) = match &self.meridian {
            Meridian::Line { start,delta } => (add(*start,scale(*delta,u)),*delta),
            Meridian::Round { center,a,b,sweep } => {
                let (s,c) = (u*sweep).dsin_cos();
                (add(*center,add(scale(*a,c),scale(*b,s))),
                    scale(add(scale(*a,-s),scale(*b,c)),*sweep))
            }
            Meridian::Conic { a,m,b,w } => crate::brep::planar::conic_at([*a,*m,*b],*w,u),
        };
        // Rodrigues' rotation of the two vectors about the unit axis, and the
        // angular velocity's cross product: what a pose would compute through
        // two matrices, on a path evaluated many thousand times per trace.
        let a = crate::envelope::normalized(self.axis).ok_or(Error::Degenerate)?;
        let (s,c) = (v*self.sweep).dsin_cos();
        use crate::space::cross;
        let turned = |x: V| { let along = (a[0]*x[0]+a[1]*x[1]+a[2]*x[2])*(1.-c); add(add(scale(x,c),scale(cross(a,x),s)),scale(a,along)) };
        let r = turned(sub(p,self.origin));
        let result = SurfacePoint { position: add(self.origin,r),du: turned(d),dv: scale(cross(a,r),self.sweep) };
        if !result.position.iter().chain(&result.du).chain(&result.dv).all(|v| v.is_finite()) {
            return Err(Error::NonFinite);
        }
        Ok(result)
    }
}
