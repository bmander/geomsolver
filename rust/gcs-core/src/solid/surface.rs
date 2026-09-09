//! Exact generating patches of existing declarative solids. These retain the solved
//! profile's lines and circles; no facets enter their position or derivative evaluation.
//! Boundary trimming and outward orientation remain separate questions.
mod contact;
pub use contact::RevolvedContact;
mod bounds;
pub use bounds::SurfaceBounds;
mod project;
pub use project::{SurfaceProjection,SurfaceProjector};
mod region;
pub use region::{RegionLocation,RegionSample,RevolvedRegion};

use crate::envelope::{Error, Motion, SurfacePoint};
use crate::model::{EntKind, EntRef, Sense, Sketch, SolidDef};
use crate::plane::{self, Basis};

type V = [f64;3];
fn add(a: V, b: V) -> V { std::array::from_fn(|i| a[i]+b[i]) }
fn sub(a: V, b: V) -> V { std::array::from_fn(|i| a[i]-b[i]) }
fn scale(a: V, s: f64) -> V { a.map(|x| x*s) }

#[derive(Clone, Debug)]
enum Meridian {
    Line { start: V, delta: V },
    Round { center: V, a: V, b: V, sweep: f64 },
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
        }
        Ok([position,derivative,second])
    }

    /// Read a named model surface against the sketch's current solved state.
    pub fn named(sk: &Sketch, surface: usize) -> Result<Self, String> {
        let s = sk.surfaces.get(surface).ok_or("no such surface")?;
        let mut patch = Self::read(sk,s.solid as usize,s.edge)?;
        patch.name = s.name.clone();
        if let Some(span) = s.span {
            let sweep = patch.sweep.abs();
            if !span.iter().all(|v| v.is_finite()) || span[0] < 0. || span[0] >= span[1]
                || span[1] > sweep {
                return Err("a surface needs increasing angular bounds within its source sweep".into());
            }
            patch.v_domain = span.map(|a| a/sweep);
        }
        Ok(patch)
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
        let face = sk.faces.get(*face as usize).ok_or("no such profile")?;
        let name = face.edges.iter().zip(&face.edge_names)
            .chain(face.holes.iter().flat_map(|h| h.edges.iter().zip(&h.edge_names)))
            .find_map(|(e,n)| (*e == edge).then_some(n))
            .ok_or("the requested edge is not a boundary of this revolution's profile")?;
        let (basis,c,s,o) = if let Some(p) = face.plane()? {
            let p = sk.planes.get(p as usize).ok_or("no profile plane")?;
            (p.basis,sk.params[p.frame.c as usize].value,
                sk.params[p.frame.s as usize].value,sk.point_xy(p.frame.origin as usize))
        } else { (Basis::page(),1.,0.,(0.,0.)) };
        let lift = |p| { let q = plane::in_view(c,s,o,p); basis.lift(q.0,q.1) };
        let vector = |p: (f64,f64)| {
            let q = (c*p.0+s*p.1,-s*p.0+c*p.1);
            add(scale(basis.u,q.0),scale(basis.v,q.1))
        };
        let axis = sk.lines.get(*axis as usize).ok_or("no revolution axis")?;
        let origin = lift(sk.point_xy(axis.p1 as usize));
        let delta = sub(lift(sk.point_xy(axis.p2 as usize)),origin);
        let len = delta[0].hypot(delta[1]).hypot(delta[2]);
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
                let (s,c) = start.sin_cos();
                Meridian::Round { center: lift(sk.point_xy(center as usize)),
                    a: vector((r*c,r*s)),b: vector((-r*s,r*c)),sweep }
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

    /// Original source parameters: an angular span restricts v without renumbering it.
    pub fn domain(&self) -> [[f64;2];2] { [[0.,1.],self.v_domain] }

    /// Whether this snapshot retains an entire revolution, including both sides
    /// of its coordinate seam. A restricted angular span is not periodic.
    pub fn is_periodic(&self) -> bool {
        self.sweep.abs() == std::f64::consts::TAU && self.v_domain == [0.,1.]
    }

    /// Allowed coordinates for a local angular chart. The quarter-turn overlap
    /// keeps interval trigonometry within its supported [-8,8] radian domain.
    /// This extends the coordinate chart, not the material or the motion domain.
    pub fn angular_chart_domain(&self) -> [f64;2] {
        if self.is_periodic() { [-0.25,1.25] } else { self.v_domain }
    }

    /// Restrict or locally continue the same angular parametrization across a
    /// full revolution's seam. Evaluate unwrapped angles directly: interval
    /// proofs do not assume that binary64 TAU gives exact periodic equality.
    /// Identifying coincident seam geometry during B-rep assembly is separate.
    pub fn angular_chart(&self,range: [f64;2]) -> Result<Self,Error> {
        let [a,b] = range; let [lo,hi] = self.angular_chart_domain();
        if !a.is_finite() || !b.is_finite() { return Err(Error::NonFinite); }
        if a > b || a < lo || b > hi || b-a > 1. { return Err(Error::OutsideDomain); }
        Ok(Self {v_domain:range,..self.clone()})
    }

    pub fn at(&self, u: f64, v: f64) -> Result<SurfacePoint, Error> {
        if !u.is_finite() || !v.is_finite() { return Err(Error::NonFinite); }
        if !(0. ..=1.).contains(&u) || v < self.v_domain[0] || v > self.v_domain[1] {
            return Err(Error::OutsideDomain);
        }
        let (p,d) = match &self.meridian {
            Meridian::Line { start,delta } => (add(*start,scale(*delta,u)),*delta),
            Meridian::Round { center,a,b,sweep } => {
                let (s,c) = (u*sweep).sin_cos();
                (add(*center,add(scale(*a,c),scale(*b,s))),
                    scale(add(scale(*a,-s),scale(*b,c)),*sweep))
            }
        };
        let rotation = Motion::rotation(self.axis,v*self.sweep,self.sweep)?;
        let r = sub(p,self.origin);
        let result = SurfacePoint { position: add(self.origin,rotation.vector(r)),
            du: rotation.vector(d),dv: rotation.velocity(r) };
        if !result.position.iter().chain(&result.du).chain(&result.dv).all(|v| v.is_finite()) {
            return Err(Error::NonFinite);
        }
        Ok(result)
    }
}
