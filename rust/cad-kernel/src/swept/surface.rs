//! A face of revolution of a tool, as the sweep reads it (#721 F2b): a meridian (a line, an arc or a
//! conic's arc) turned about an axis over a span, its points and their derivatives exact, its rings
//! and their contact under a twist in closed form (`RingFrame`), and its signed support equation.
//! The solver reads one off a drawing; nothing here knows how it was drawn.
#[allow(unused_imports)]
use crate::util::fmath::Det;
use crate::contact::{self,Contact,RingFrame,SurfacePoint,Sinusoid};
use crate::trajectory::{Error,Motion};
use crate::util::space::{add,cross,distance,dot,length,scale,sub};
use std::f64::consts::TAU;

type V = [f64;3];

/// A meridian, the profile edge a face of revolution turns: a line, an arc, or a conic's arc.
#[derive(Clone, Debug)]
pub enum Meridian {
    Line { start: V, delta: V },
    Round { center: V, a: V, b: V, sweep: f64 },
    /// A conic's arc: the rational quadratic Bézier over `a`, `m`, `b`, the middle weighed `w`.
    Conic { a: V, m: V, b: V, w: f64 },
}

impl Meridian {
    /// An arc of radius `r` about `center`, from `start` counter-clockwise through `sweep` in the
    /// plane whose directions are `u` and `v`.
    pub fn round(center: V,u: V,v: V,r: f64,start: f64,sweep: f64) -> Meridian {
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
    /// `meridian` turned about the line through `origin` along the unit `axis` by `sweep` radians
    /// (negative clockwise), over the whole of it, named `name`.
    pub fn new(name: String,meridian: Meridian,origin: V,axis: V,sweep: f64) -> Self {
        Self {name,meridian,origin,axis,sweep,v_domain:[0.,1.]}
    }

    /// The meridian it turns.
    pub fn meridian(&self) -> &Meridian { &self.meridian }
    /// A point of the axis it turns about.
    pub fn origin(&self) -> V { self.origin }

    /// Bounds for the generating profile before revolution, in solved world
    /// coordinates. The coefficients of this owned snapshot are treated as exact
    /// binary64 data; solve error and correspondence to the intended dimensions
    /// are separate. This does not bound the swept or generated envelope surface.
    pub fn generating_profile_bounds(&self,u: crate::util::interval::Interval)
        -> Result<([crate::util::interval::Interval;3],[crate::util::interval::Interval;3]),crate::util::interval::Error> {
        let [p,d,_] = self.profile_bounds(u,false)?;
        Ok((p,d))
    }

    /// Position, du and duu enclosures for the pre-revolution generating profile.
    /// This has the same snapshot and finite-domain contract as
    /// `generating_profile_bounds`; derivatives are with respect to normalized u.
    pub fn generating_profile_jet_bounds(&self,u: crate::util::interval::Interval)
        -> Result<[[crate::util::interval::Interval;3];3],crate::util::interval::Error> {
        self.profile_bounds(u,true)
    }

    fn profile_bounds(&self,u: crate::util::interval::Interval,second_order: bool)
        -> Result<[[crate::util::interval::Interval;3];3],crate::util::interval::Error> {
        use crate::util::interval::{Error,Interval as I};
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

    /// The same surface with its generating edge run the other way, `u` turned to `1 − u` (its
    /// chart's normal turned with it): a region's face walked as its neighbours are, where its
    /// meridian's step runs against the walk (#214).
    pub fn reversed(&self) -> Self {
        let meridian = match self.meridian {
            Meridian::Line {start,delta} => Meridian::Line {start: add(start,delta),delta: scale(delta,-1.)},
            // `c + a cos(s(1 − u)) + b sin(s(1 − u))`: the arc from its other end, the same way
            // round about its centre's other side
            Meridian::Round {center,a,b,sweep} => {
                let (sn,cs) = sweep.dsin_cos();
                Meridian::Round {center,a: add(scale(a,cs),scale(b,sn)),b: sub(scale(a,sn),scale(b,cs)),sweep}
            }
            Meridian::Conic {a,m,b,w} => Meridian::Conic {a: b,m,b: a,w},
        };
        Self {meridian,..self.clone()}
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

    /// Where the face is flat — its meridian a line square to the axis, turned to a disc or a ring
    /// in a plane square to it (#684 O) — that plane's centre on the axis, the unit axis, the
    /// radii of the meridian's two ends, and whether the revolution runs all the way round.
    pub fn flat(&self) -> Option<(V,V,[f64;2],bool)> {
        let Meridian::Line {start,delta} = self.meridian else { return None };
        let a = crate::trajectory::normalized(self.axis)?;
        let length = delta[0].dhypot(delta[1]).dhypot(delta[2]);
        if !(dot(delta,a).abs() <= 1e-12*length) { return None }
        let centre = add(self.origin,scale(a,dot(sub(start,self.origin),a)));
        let whole = self.v_domain == [0.,1.] && self.sweep.abs() >= std::f64::consts::TAU*(1.-1e-12);
        Some((centre,a,[distance(start,centre),distance(add(start,delta),centre)],whole))
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
            Meridian::Conic { a,m,b,w } => crate::planar::conic_at([*a,*m,*b],*w,u),
        };
        // Rodrigues' rotation of the two vectors about the unit axis, and the
        // angular velocity's cross product: what a pose would compute through
        // two matrices, on a path evaluated many thousand times per trace.
        let a = crate::trajectory::normalized(self.axis).ok_or(Error::Degenerate)?;
        let (s,c) = (v*self.sweep).dsin_cos();
        let turned = |x: V| { let along = (a[0]*x[0]+a[1]*x[1]+a[2]*x[2])*(1.-c); add(add(scale(x,c),scale(cross(a,x),s)),scale(a,along)) };
        let r = turned(sub(p,self.origin));
        let result = SurfacePoint { position: add(self.origin,r),du: turned(d),dv: scale(cross(a,r),self.sweep) };
        if !result.position.iter().chain(&result.du).chain(&result.dv).all(|v| v.is_finite()) {
            return Err(Error::NonFinite);
        }
        Ok(result)
    }
}

/// One isolated root of the contact equation. Branch labels are the two analytic
/// roots, not a sort by wrapped v; that sorting would exchange sheets at a seam.
#[derive(Clone,Copy,Debug)]
pub struct RevolvedContact {
    pub branch: usize,
    pub v: f64,
    pub contact: Contact,
}

impl RevolvedSurface {
    /// Apply a fixed placement to this snapshot. Only the pose is used; a motion's
    /// derivative does not turn this placement into another generating motion.
    pub fn placed(&self,pose: Motion) -> Self {
        let meridian = match self.meridian {
            Meridian::Line {start,delta} => Meridian::Line {
                start:pose.point(start),delta:pose.vector(delta)},
            Meridian::Round {center,a,b,sweep} => Meridian::Round {
                center:pose.point(center),a:pose.vector(a),b:pose.vector(b),sweep},
            Meridian::Conic {a,m,b,w} => Meridian::Conic {
                a:pose.point(a),m:pose.point(m),b:pose.point(b),w},
        };
        Self {meridian,origin:pose.point(self.origin),axis:pose.vector(self.axis),..self.clone()}
    }

    /// Solve n dot velocity = 0 around one meridian station, without a seed or
    /// angular march. For a surface of revolution this is A cos(theta) +
    /// B sin(theta) + C = 0 under any instantaneous rigid motion.
    ///
    /// This produces candidate contact points, not a trimmed swept boundary.
    /// Poles, stationary rings and near-double roots return Degenerate, so topology
    /// events cannot silently become an empty curve. Tolerance is normal velocity
    /// in model length per motion parameter; this is not an interval root proof.
    pub fn contacts(&self,u: f64,motion: Motion,tolerance: f64)
        -> Result<Vec<RevolvedContact>,Error> {
        contact::check_tolerance(tolerance)?;
        if tolerance == 0. { return Err(Error::InvalidOptions); }
        let k = self.contact_coefficients(u,motion)?;
        let mut roots = Vec::new();
        for (branch,v) in k.roots_over(self.sweep,self.v_domain,tolerance)? {
            let contact = contact::contact(self.at(u,v)?,motion)?;
            if contact.normal_velocity.abs() > tolerance { return Err(Error::NotConverged); }
            roots.push(RevolvedContact {branch,v,contact});
        }
        Ok(roots)
    }

    /// The ring's contact `a cos + b sin + c` at meridian station `u` under the motion's
    /// instantaneous twist, read on what it carries (`RingFrame::coefficients`). A
    /// zero amplitude with zero constant is a stationary ring, every point of which is in contact.
    pub fn contact_coefficients(&self,u: f64,motion: Motion) -> Result<Sinusoid,Error> {
        let s = self.full().at(u,0.)?;
        let ring = self.ring_at(s.position)?;
        let normal = contact::contact(s,Motion::identity())?.normal;
        Ok(ring.coefficients(dot(normal,ring.radial),dot(normal,ring.axis),motion))
    }

    /// The circle meridian station `u` turns to: its centre on the axis, its radius, and the
    /// radial direction at the chart's start (`v = 0`) the angle of a point on it is read from.
    pub fn ring(&self,u: f64) -> Result<RingFrame,Error> {
        // The ring uses the original meridian even for a restricted span.
        self.ring_at(self.full().at(u,0.)?.position)
    }

    /// The circle the point `p` turns to about the axis.
    fn ring_at(&self,p: V) -> Result<RingFrame,Error> {
        let q = sub(p,self.origin);
        let axial = dot(q,self.axis);
        let radial = sub(q,scale(self.axis,axial));
        let radius = radial[0].dhypot(radial[1]).dhypot(radial[2]);
        if radius == 0. || !radius.is_finite() { return Err(Error::Degenerate); }
        Ok(RingFrame {center:add(self.origin,scale(self.axis,axial)),radius,radial:scale(radial,1./radius),axis:self.axis})
    }

    fn full(&self) -> Self { Self {v_domain:[0.,1.],..self.clone()} }
    /// The revolution's unit axis direction.
    pub fn axis_direction(&self) -> [f64;3] { self.axis }
}

#[derive(Clone,Debug)]
enum Section {
    Line {start:[f64;2],delta:[f64;2],length:f64},
    Circle {center:[f64;2],a:[f64;2],b:[f64;2],radius:f64,sweep:f64,orientation:f64},
    /// A conic's arc in (radius, height), read by its nearest point (`Seg::nearest`).
    Conic {seg:crate::planar::Seg},
}

/// A support-surface projection and an actual point on the finite source patch.
#[derive(Clone,Copy,Debug)]
pub struct SurfaceProjection {
    /// Parameters on the continued supporting surface. They may be outside [0,1].
    pub parameters: [f64;2],
    /// Oriented meridian line/circle residual in length units. This is a local signed
    /// distance near a regular interior point, not a global closest-distance claim.
    pub signed_residual: f64,
    /// Gradient of signed_residual, following the source tangent orientation.
    pub normal: V,
    /// A point on the finite source patch, obtained by bounding the projected parameters.
    pub point: V,
    /// Distance from the query to `point`. This certifies incidence when small; it is
    /// not asserted to be the minimum distance to a finite patch with multiple edges.
    pub incidence_error: f64,
}

/// A solved snapshot for signed support equations and bounded incidence checks. The
/// meridian must lie on one side of the axis; an axis-crossing parameterization is refused.
#[derive(Clone,Debug)]
pub struct SurfaceProjector {
    source: RevolvedSurface,
    radial: V,
    section: Section,
}

// Select the periodic representative nearest the closed interval [0,sweep]. Do not
// silently turn an off-patch projection into an in-domain parameter by clamping it.
fn phase(angle: f64,sweep: f64) -> f64 {
    let a = angle.rem_euclid(TAU);
    let distance = |a: f64| (-a).max(a-sweep).max(0.);
    if distance(a-TAU) < distance(a) { a-TAU } else { a }
}

impl RevolvedSurface {
    pub fn projector(&self) -> Result<SurfaceProjector,String> {
        let perp = |p: V| sub(p,scale(self.axis,dot(p,self.axis)));
        let candidates = match self.meridian {
            Meridian::Line {start,delta} => vec![sub(start,self.origin),delta],
            Meridian::Round {center,a,b,..} => vec![sub(center,self.origin),a,b],
            Meridian::Conic {a,m,b,..} => vec![sub(a,self.origin),sub(m,self.origin),sub(b,self.origin)],
        };
        let radial = candidates.iter().map(|&p| perp(p))
            .find(|&p| length(p) > 1e-12*candidates.iter().map(|&p| length(p)).fold(0.,f64::max))
            .ok_or("a surface on its axis has no regular meridian")?;
        let radial = scale(radial,1./length(radial));
        let coords = |p: V| [dot(p,radial),dot(p,self.axis)];
        let section = match self.meridian {
            Meridian::Line {start,delta} => {
                let start = coords(sub(start,self.origin));
                let delta = coords(delta);
                let length = delta[0].dhypot(delta[1]);
                if !length.is_finite() || length == 0. {
                    return Err("a boundary meridian needs a nondegenerate line".into());
                }
                let rmin = start[0].min(start[0]+delta[0]);
                if rmin < -1e-12*length || start[0].max(start[0]+delta[0]) <= 0. {
                    return Err("a boundary meridian must stay on one side of its axis".into());
                }
                Section::Line {start,delta,length}
            }
            Meridian::Round {center,a,b,sweep} => {
                let center = coords(sub(center,self.origin));
                let a = coords(a); let b = coords(b);
                let radius = a[0].dhypot(a[1]);
                let det = a[0]*b[1]-a[1]*b[0];
                if !radius.is_finite() || radius <= 0. || det == 0. {
                    return Err("a boundary meridian needs a regular circle".into());
                }
                let r = |t: f64| center[0]+a[0]*t.dcos()+b[0]*t.dsin();
                let mut rmin = r(0.).min(r(sweep));
                let extreme = b[0].datan2(a[0]).rem_euclid(TAU);
                for t in [extreme,(extreme+std::f64::consts::PI).rem_euclid(TAU)] {
                    if t <= sweep { rmin = rmin.min(r(t)); }
                }
                if rmin < -1e-12*radius {
                    return Err("a boundary meridian must stay on one side of its axis".into());
                }
                Section::Circle {center,a,b,radius,sweep,orientation:-det.signum()}
            }
            Meridian::Conic {a,m,b,w} => {
                let at = |p: V| coords(sub(p,self.origin));
                let (a,m,b) = (at(a),at(m),at(b));
                if a[0].min(m[0]).min(b[0]) < -1e-12*a[0].abs().max(b[0].abs()) {
                    return Err("a boundary meridian must stay on one side of its axis".into());
                }
                Section::Conic {seg:crate::planar::Seg::Conic {a,m,b,w}}
            }
        };
        Ok(SurfaceProjector {source:self.clone(),radial,section})
    }
}

impl SurfaceProjector {
    pub fn surface(&self) -> &RevolvedSurface { &self.source }

    pub fn project(&self, p: V) -> Result<SurfaceProjection,Error> {
        if !p.iter().all(|x| x.is_finite()) { return Err(Error::NonFinite); }
        let s = &self.source;
        let q = sub(p,s.origin);
        let z = dot(q,s.axis);
        let rvec = sub(q,scale(s.axis,z));
        let r = length(rvec);
        let direction = if r > 0. { scale(rvec,1./r) } else { self.radial };
        let sign = s.sweep.signum();
        let angle = dot(s.axis,cross(self.radial,direction))
            .datan2(dot(self.radial,direction));
        let [from,to] = s.v_domain.map(|v| v*s.sweep.abs());
        let v = (from+phase(sign*angle-from,to-from))/s.sweep.abs();
        let (u,distance,n) = match self.section {
            Section::Line {start,delta,length} => {
                let q = [r-start[0],z-start[1]];
                let n = [-delta[1]/length,delta[0]/length];
                ((q[0]*delta[0]+q[1]*delta[1])/(length*length),
                    q[0]*n[0]+q[1]*n[1],n)
            }
            Section::Circle {center,a,b,radius,sweep,orientation} => {
                let q = [r-center[0],z-center[1]];
                let d = q[0].dhypot(q[1]);
                if d == 0. { return Err(Error::Degenerate); }
                let angle = (q[0]*b[0]+q[1]*b[1]).datan2(q[0]*a[0]+q[1]*a[1]);
                (phase(angle,sweep)/sweep,orientation*(d-radius),
                    [orientation*q[0]/d,orientation*q[1]/d])
            }
            // the left of travel, as a line's: the offset from the nearest point along its normal
            Section::Conic {seg} => {
                let u = seg.nearest([r,z]);
                let (c,t) = (seg.point(u),seg.tangent(u));
                let l = t[0].dhypot(t[1]);
                if l == 0. { return Err(Error::Degenerate); }
                let n = [-t[1]/l,t[0]/l];
                (u,(r-c[0])*n[0]+(z-c[1])*n[1],n)
            }
        };
        // At an axis point a nonzero radial normal has no unique world direction.
        if r == 0. && n[0].abs() > 1e-12 { return Err(Error::Degenerate); }
        let point = s.at(u.clamp(0.,1.),v.clamp(s.v_domain[0],s.v_domain[1]))?.position;
        let result = SurfaceProjection {parameters:[u,v],signed_residual:sign*distance,
            normal:scale(add(scale(direction,n[0]),scale(s.axis,n[1])),sign),
            point,incidence_error:length(sub(p,point))};
        if !result.parameters.iter().chain(&result.normal).chain(&result.point)
            .chain([&result.signed_residual,&result.incidence_error]).all(|x| x.is_finite()) {
            return Err(Error::NonFinite);
        }
        Ok(result)
    }
}

impl RevolvedSurface {
    /// Intersect a fixed revolution meridian with a sphere. A line meridian gives a
    /// quadratic; return every root on the finite source edge, in parameter order.
    /// This is an exact geometric section, independent of any gear convention.
    pub fn line_on_sphere(&self, center: V, radius: f64, v: f64)
        -> Result<Vec<SurfacePoint>,String> {
        if !matches!(self.meridian,Meridian::Line {..}) {
            return Err("a line/sphere section requires a straight meridian".into());
        }
        if !radius.is_finite() || radius <= 0. || !center.iter().all(|x| x.is_finite()) {
            return Err("a sphere section needs a finite center and positive radius".into());
        }
        let start = self.at(0.,v).map_err(|e| format!("invalid section: {e:?}"))?;
        let end = self.at(1.,v).map_err(|e| format!("invalid section: {e:?}"))?;
        let q = sub(start.position,center);
        let d = sub(end.position,start.position);
        let size = length(q).max(length(d)).max(radius);
        let q = scale(q,1./size); let d = scale(d,1./size); let r = radius/size;
        let a = dot(d,d); let b = dot(q,d); let c = dot(q,q)-r*r;
        if !a.is_finite() || a <= 0. { return Err("a line section needs a regular meridian".into()); }
        let disc = b*b-a*c;
        if !disc.is_finite() { return Err("a sphere section overflowed".into()); }
        if disc < 0. { return Ok(vec![]); }
        let k = -b-disc.sqrt().copysign(b);
        let mut roots = if disc == 0. { vec![-b/a] } else { vec![k/a,c/k] };
        roots.sort_by(f64::total_cmp);
        roots.retain(|&u| (0. ..=1.).contains(&u));
        roots.into_iter().map(|u| self.at(u,v).map_err(|e| format!("invalid section: {e:?}"))).collect()
    }
}
