//! Where a surface is **in contact** under a twist (#721 F2): the points whose normal is square to
//! their velocity, `n · v = 0`, the equation every sweep's boundary is read from. At each instant
//! a rigid motion is a screw (`trajectory::Twist`), so what a face contributes to a sweep's
//! boundary is its surface's contact under that screw, and the kinds are few (#684):
//!
//! * **a ring** of a surface turned about its frame's `z` (a cylinder, a cone, a sphere, a torus, a
//!   revolution): its point and its normal are turned with it, so on the ring at each `v` the
//!   contact is `A cos u + B sin u + C = 0`, two roots or none, under any twist (`Sinusoid`);
//! * **a ruling** along which the normal stands still (a plane, a cylinder, a cone, an extrusion,
//!   a revolution of a line): the contact is linear along it, `a + b v = 0`, one root (`Affine`);
//! * **a plane**: linear across it, `a + b u + c v = 0`, a line (`Surface::plane_contact`).
//!
//! Every other surface (a blend, a B-spline, a face family's wall) is read numerically: its
//! contact function evaluated (`Surface::normal_velocity`) and its zeros found along a line of
//! its parameters (`Surface::contact_along`). The closed forms are that function, exactly
//! rearranged, and are held to it.
//!
//! A twist is written in the coordinates the surface is written in: a tool's own surfaces under
//! the twist that carries the tool, read on the tool (`Twist::body`).
#[allow(unused_imports)]
use crate::util::fmath::Det;
use crate::geom::{Curve,Surface,Uv,V};
use crate::trajectory::{Error,Motion,Twist,normalized};
use crate::util::space::{add,cross,dot,length,normalised,scale,sub};

const TAU: f64 = std::f64::consts::TAU;

/// Which closed form a surface's contact has (`Surface::contact_form`).
#[derive(Clone,Copy,Debug,PartialEq,Eq)]
pub enum Form {
    /// Linear across the surface: a plane (`Surface::plane_contact`), whose rulings run both ways.
    Linear,
    /// Sinusoidal on each ring (`Surface::ring_contact`), and where `rulings`, linear along each
    /// ruling as well (`Surface::ruling_contact`): a cylinder, a cone, a revolution of a line.
    Rings { rulings: bool },
    /// Linear along each ruling (`Surface::ruling_contact`): an extrusion.
    Rulings,
    /// None: read numerically (`Surface::contact_along`).
    Numeric,
}

/// `a cos u + b sin u + c`, a ring's contact as its angle `u` runs.
#[derive(Clone,Copy,Debug,PartialEq)]
pub struct Sinusoid { pub a: f64,pub b: f64,pub c: f64 }

/// Where a ring is in contact.
#[derive(Clone,Copy,Debug,PartialEq)]
pub enum RingRoots {
    /// Nowhere.
    None,
    /// At two angles, each in `[0, 2π)`.
    Two([f64;2]),
    /// At one angle, where its two roots meet: the characteristic turns back on the ring there.
    Double(f64),
    /// All round: every point of the ring is in contact.
    All,
}

impl Sinusoid {
    pub fn at(&self,u: f64) -> f64 { let (s,c) = u.dsin_cos(); self.a*c+self.b*s+self.c }
    /// Its roots, `tolerance` read in the contact's own units (a velocity, along the normal): an
    /// amplitude within it is none, and a constant within it of the amplitude a double root.
    pub fn roots(&self,tolerance: f64) -> RingRoots {
        let amplitude = self.a.dhypot(self.b);
        if amplitude <= tolerance { return if self.c.abs() <= tolerance { RingRoots::All } else { RingRoots::None } }
        if self.c.abs() > amplitude+tolerance { return RingRoots::None }
        let phase = self.b.datan2(self.a);
        if (self.c.abs()-amplitude).abs() <= tolerance {
            // cos(u − phase) = −c / amplitude = ∓1
            let u = if self.c > 0. { phase+std::f64::consts::PI } else { phase };
            return RingRoots::Double(u.rem_euclid(TAU))
        }
        let spread = (-self.c/amplitude).dacos();
        RingRoots::Two([(phase-spread).rem_euclid(TAU),(phase+spread).rem_euclid(TAU)])
    }
}

/// `a + b v`, a ruling's contact as its parameter `v` runs.
#[derive(Clone,Copy,Debug,PartialEq)]
pub struct Affine { pub a: f64,pub b: f64 }

/// Where a ruling is in contact.
#[derive(Clone,Copy,Debug,PartialEq)]
pub enum RulingRoot {
    /// Nowhere.
    None,
    /// At one `v`.
    One(f64),
    /// All along it.
    All,
}

impl Affine {
    pub fn at(&self,v: f64) -> f64 { self.a+self.b*v }
    /// Its root, `tolerance` read in the contact's own units (`Sinusoid::roots`): a slope within it
    /// a unit of `v` is none, the ruling in contact all along where its constant is within it too.
    pub fn root(&self,tolerance: f64) -> RulingRoot {
        if self.b.abs() <= tolerance { return if self.a.abs() <= tolerance { RulingRoot::All } else { RulingRoot::None } }
        RulingRoot::One(-self.a/self.b)
    }
}

impl Surface {
    /// Which closed form its contact has.
    pub fn contact_form(&self) -> Form {
        match self {
            Surface::Plane(_) => Form::Linear,
            Surface::Cylinder(..) | Surface::Cone(..) => Form::Rings {rulings:true},
            Surface::Sphere(..) | Surface::Torus(..) => Form::Rings {rulings:false},
            Surface::Revolution(_,c) => Form::Rings {rulings:matches!(**c,Curve::Line {..})},
            Surface::Extrusion(..) => Form::Rulings,
            Surface::Blend(..) | Surface::BSpline(..) | Surface::Family(..) => Form::Numeric,
        }
    }

    /// **The contact function** at `uv` under `twist`: the unit normal (`normal`, of `S_u × S_v`)
    /// dotted with the velocity there. Nothing where the normal is not defined (a pole, an apex).
    pub fn normal_velocity(&self,uv: Uv,twist: &Twist) -> Option<f64> {
        let (p,n) = self.point_normal(uv)?;
        Some(dot(n,twist.velocity(p)))
    }

    /// The point at `uv` and its unit normal (`normal`'s), read in one evaluation.
    fn point_normal(&self,uv: Uv) -> Option<(V,V)> {
        let (p,su,sv) = self.d1(uv);
        Some((p,normalised(cross(su,sv))?))
    }

    /// The ring at `v`'s contact, `A cos u + B sin u + C`, where its surface is turned about its
    /// frame's `z` by `u` (`Form::Rings`): its point `q` and unit normal `m` at `u = 0`, in the
    /// frame, turned by `u`, so that with the velocity `V` of the frame's origin and `ω` in the frame,
    /// `n · v = (R m) · V + ω · (R (q × m))`, each term sinusoidal in `u`. Nothing for any other
    /// surface, or where the ring's normal is not defined (a pole, an apex).
    pub fn ring_contact(&self,v: f64,twist: &Twist) -> Option<Sinusoid> {
        if !matches!(self.contact_form(),Form::Rings {..}) { return None }
        let f = self.frame();
        let (q,m) = self.point_normal([0.,v])?;
        let (q,m) = (f.local(q),f.dir_local(m));
        let k = cross(q,m);
        let big = f.dir_local(twist.velocity(f.o));
        let w = f.dir_local(twist.omega);
        Some(Sinusoid {
            a: m[0]*big[0]+m[1]*big[1]+k[0]*w[0]+k[1]*w[1],
            b: m[0]*big[1]-m[1]*big[0]+k[0]*w[1]-k[1]*w[0],
            c: m[2]*big[2]+k[2]*w[2],
        })
    }

    /// The ruling at `u`'s contact, `a + b v`, where the surface is a line along `v` whose normal
    /// stands still along it (`Form::Rings {rulings: true}`, `Form::Rulings`, `Form::Linear`): with
    /// its point `P` and direction `d` at `v = 0` and its normal `n`, `n · v = n · v(P) + v n · (ω × d)`.
    /// `n` is **the ruling's normal**, read at `v = 0`, or at `v = 1` where it is not defined there
    /// (a cone's apex): the surface's own up to its sign, which turns where the ruling crosses an
    /// apex or the axis it is turned about, the root the same. Nothing for any other surface.
    pub fn ruling_contact(&self,u: f64,twist: &Twist) -> Option<Affine> {
        if !matches!(self.contact_form(),Form::Linear | Form::Rulings | Form::Rings {rulings:true}) { return None }
        let (p,su,d) = self.d1([u,0.]);
        let n = normalised(cross(su,d)).or_else(|| self.normal([u,1.]))?;
        Some(Affine {a:dot(n,twist.velocity(p)),b:dot(n,cross(twist.omega,d))})
    }

    /// A plane's contact, `[a, b, c]` of `a + b u + c v`: with its frame's origin `o` and axes `x`,
    /// `y` and its normal `n`, `n · v(o) + u n · (ω × x) + v n · (ω × y)`. Nothing for any other
    /// surface.
    pub fn plane_contact(&self,twist: &Twist) -> Option<[f64;3]> {
        let Surface::Plane(f) = self else { return None };
        let n = self.normal([0.,0.])?;
        Some([dot(n,twist.velocity(f.o)),dot(n,cross(twist.omega,f.x)),dot(n,cross(twist.omega,f.y))])
    }

    /// Where the contact function crosses nought along the line of parameters from `a` to `b`,
    /// as fractions of the way: sampled at `samples` steps and each change of sign closed on to
    /// `width` of the way, a sample where the normal is not defined passed over. The numerical
    /// reading every surface has, closed form or not; a root where the function touches nought
    /// without crossing is not seen.
    pub fn contact_along(&self,a: Uv,b: Uv,twist: &Twist,samples: usize,width: f64) -> Vec<f64> {
        let at = |x: f64| self.normal_velocity([a[0]+(b[0]-a[0])*x,a[1]+(b[1]-a[1])*x],twist);
        let samples = samples.max(1);
        let mut read = (0..=samples).filter_map(|i| { let x = i as f64/samples as f64; at(x).map(|g| (x,g)) });
        let (mut out,mut last) = (Vec::new(),read.next());
        for (x1,g1) in read {
            let Some((x0,g0)) = last.replace((x1,g1)) else { continue };
            if g0 == 0. { out.push(x0) }
            else if (g0 < 0.) != (g1 < 0.) && g1 != 0. { out.push(crate::util::roots::bracketed_root(at,x0,g0,x1,g1,width)) }
        }
        if let Some((x,0.)) = last { out.push(x) }
        out
    }
}

/// A smooth surface and its two exact first derivatives at one parameter pair.
#[derive(Clone, Copy, Debug)]
pub struct SurfacePoint {
    pub position: V,
    pub du: V,
    pub dv: V,
}

/// Boundary checks allow zero tolerance, but never negative or nonfinite controls.
pub fn check_tolerance(tolerance: f64) -> Result<(),Error> {
    if tolerance.is_finite() && tolerance >= 0. { Ok(()) } else { Err(Error::InvalidOptions) }
}

/// A moving surface point. `normal_velocity = 0` is the envelope equation.
#[derive(Clone, Copy, Debug)]
pub struct Contact {
    pub position: V,
    pub normal: V,
    pub velocity: V,
    pub normal_velocity: f64,
}

/// The surface point carried by `motion`: where it stands, its unit normal (of `du × dv`) and its
/// velocity, and the envelope's equation there, `n · v`.
pub fn contact(surface: SurfacePoint, motion: Motion) -> Result<Contact, Error> {
    // Normalize each tangent first: units or parameterization must not turn a small,
    // perfectly regular surface into a singular one (nor overflow the cross product).
    let u = normalized(surface.du).ok_or(Error::Degenerate)?;
    let v = normalized(surface.dv).ok_or(Error::Degenerate)?;
    let n = cross(u, v);
    if length(n) < 1e-12 { return Err(Error::Degenerate); }
    let normal = motion.vector(normalized(n).ok_or(Error::Degenerate)?);
    let position = motion.point(surface.position);
    let velocity = motion.velocity(surface.position);
    let normal_velocity = dot(normal, velocity);
    if !position.iter().chain(&normal).chain(&velocity)
        .chain(std::iter::once(&normal_velocity)).all(|x| x.is_finite()) {
        return Err(Error::NonFinite);
    }
    Ok(Contact { position, normal, velocity, normal_velocity })
}

/// A circle turned about an axis, and the frame a ring's contact is written in on it: `radial` at
/// angle 0, `axis × radial` at a quarter turn. A face of revolution's ring, or the circle a convex
/// corner of its profile turns to, whose fan carries every normal between its faces'.
#[derive(Clone,Copy,Debug)]
pub struct RingFrame { pub center: V,pub radius: f64,pub radial: V,pub axis: V }

impl RingFrame {
    /// The point at angle `theta` about the axis, and the direction away from the axis there.
    pub fn at(&self,theta: f64) -> (V,V) {
        let (s,c) = theta.dsin_cos();
        let e = add(scale(self.radial,c),scale(cross(self.axis,self.radial),s));
        (add(self.center,scale(e,self.radius)),e)
    }

    /// The angles about the ring of the roots of its contact `k` (`Sinusoid::roots_over`, the
    /// ring once round): none, two, or `Degenerate` where the ring is in contact all round or the
    /// two roots meet.
    pub fn roots(&self,k: Sinusoid,tolerance: f64) -> Result<Vec<f64>,Error> {
        Ok(k.roots_over(TAU,[0.,1.],tolerance)?.into_iter().map(|(_,v)| TAU*v).collect())
    }

    /// The ring's contact `A cos θ + B sin θ + C` under `motion`'s instantaneous twist, of the
    /// normal `nr` away from the axis and `nz` along it, the same at every angle: a face's normal at
    /// the ring, or any of the fan a convex corner carries. With `V` the velocity of the centre and
    /// `ω` the turn, both read on what the motion carries (the twist `Twist::body` is), the normal
    /// `nr e + nz a` dotted with `V + ω × R e` is `e · (nr V + nz R (a × ω)) + nz a · V`. (`V` and
    /// `ω` are read off the motion's velocity field itself, its points' velocities taken back into
    /// the carried frame, as the constant-twist walk has always read them.)
    pub fn coefficients(&self,nr: f64,nz: f64,motion: Motion) -> Sinusoid {
        let (e,axis,radius) = (self.radial,self.axis,self.radius);
        let f = cross(axis,e);
        // the carried frame's angular velocity is the axial vector of R^T R'
        let inverse = motion.inverse();
        let velocity = |p| inverse.vector(motion.velocity(p));
        let v0 = velocity([0.;3]);
        let columns: [V;3] = std::array::from_fn(|i| {
            let mut p = [0.;3]; p[i] = 1.; sub(velocity(p),v0)
        });
        let omega = [0.5*(columns[1][2]-columns[2][1]),
            0.5*(columns[2][0]-columns[0][2]),0.5*(columns[0][1]-columns[1][0])];
        let center_velocity = velocity(self.center);
        let h = add(scale(center_velocity,nr),scale(cross(axis,omega),radius*nz));
        Sinusoid {a:dot(h,e),b:dot(h,f),c:nz*dot(center_velocity,axis)}
    }
}

impl Sinusoid {
    /// Its isolated roots on a chart `v` of the ring that runs over `domain` of a span of `sweep`
    /// radians from its angle 0 (negative clockwise), `v` being the angle over `sweep`: each with
    /// its branch, 0 the root before the phase and 1 the one after, as the two analytic roots are
    /// labelled (never by a sort by `v`, which would exchange them at a seam). None where none lies
    /// in `domain`; `Degenerate` where the ring is in contact all round or its two roots meet, and
    /// `NonFinite` where the contact is no number. `tolerance` is `roots`'.
    pub fn roots_over(&self,sweep: f64,domain: [f64;2],tolerance: f64) -> Result<Vec<(usize,f64)>,Error> {
        let (a,b,c) = (self.a,self.b,self.c);
        let amplitude = a.dhypot(b);
        if ![a,b,c,amplitude].iter().all(|v| v.is_finite()) { return Err(Error::NonFinite); }
        if c.abs() > amplitude+tolerance { return Ok(Vec::new()); }
        if amplitude <= tolerance || (c.abs()-amplitude).abs() <= tolerance {
            return Err(Error::Degenerate);
        }
        let phase = b.datan2(a); let spread = (-c/amplitude).dacos();
        let mut roots = Vec::new();
        for (branch,theta) in [phase-spread,phase+spread].into_iter().enumerate() {
            let v = (theta*sweep.signum()).rem_euclid(TAU)/sweep.abs();
            // the endpoint at 1 stands for the seam at 0 of a full turn too
            let period = TAU/sweep.abs();
            let Some(v) = [v,v-period,v+period].into_iter().find(|v|
                *v >= domain[0] && *v <= domain[1]) else { continue };
            roots.push((branch,v));
        }
        Ok(roots)
    }
}
