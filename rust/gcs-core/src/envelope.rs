//! Differential geometry of the envelope of a moving surface.
//!
//! A surface S(u,v) moving by M(t) contributes to its envelope where
//! n · ∂(M S)/∂t = 0. This is the equation of meshing, independent of gears or of a
//! manufacturing process. The position, tangents and velocity come from the generating
//! geometry; a mesh is not used to estimate any of them.
//!
//! This module finds regular, local intersections. It does not certify a global envelope,
//! trim self-intersections, choose a material side, or turn patches into a closed solid.

mod named;
pub use named::GeneratedEnvelope;

use crate::plane::{cross, dot, scaled};

type V3 = [f64; 3];
type M3 = [[f64; 3]; 3];

fn add(a: V3, b: V3) -> V3 { std::array::from_fn(|i| a[i] + b[i]) }
fn mv(a: M3, b: V3) -> V3 { a.map(|r| dot(r, b)) }
fn mm(a: M3, b: M3) -> M3 {
    std::array::from_fn(|i| std::array::from_fn(|j| (0..3).map(|k| a[i][k]*b[k][j]).sum()))
}
fn ma(a: M3, b: M3) -> M3 {
    std::array::from_fn(|i| add(a[i], b[i]))
}
fn length(a: V3) -> f64 { a[0].hypot(a[1]).hypot(a[2]) }
fn normalized(a: V3) -> Option<V3> {
    let n = length(a);
    (n > 0.0 && n.is_finite()).then(|| a.map(|v| v/n))
}

/// A smooth surface and its two exact first derivatives at one parameter pair.
#[derive(Clone, Copy, Debug)]
pub struct SurfacePoint {
    pub position: V3,
    pub du: V3,
    pub dv: V3,
}

/// A rigid pose and its derivative with respect to a shared motion parameter.
/// Composition differentiates both moving frames by the product rule.
#[derive(Clone, Copy, Debug)]
pub struct Motion {
    r: M3,
    dr: M3,
    p: V3,
    dp: V3,
}

impl Motion {
    pub fn identity() -> Self {
        Self { r: [[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]], dr: [[0.;3];3],
            p: [0.;3], dp: [0.;3] }
    }

    /// Right-handed rotation about an axis through the origin. Angle is in radians;
    /// rate is radians per unit of the motion parameter. The axis need not be a unit vector.
    pub fn rotation(axis: V3, angle: f64, rate: f64) -> Result<Self, Error> {
        if !angle.is_finite() || !rate.is_finite() { return Err(Error::NonFinite); }
        let a = normalized(axis).ok_or(Error::Degenerate)?;
        let (s, c) = angle.sin_cos();
        let k = [[0.,-a[2],a[1]], [a[2],0.,-a[0]], [-a[1],a[0],0.]];
        let r = std::array::from_fn(|i| std::array::from_fn(|j|
            c * if i == j { 1. } else { 0. } + (1.-c)*a[i]*a[j] + s*k[i][j]));
        let dr = mm(k, r).map(|row| scaled(row, rate));
        Ok(Self { r, dr, ..Self::identity() })
    }

    pub fn translation(offset: V3, velocity: V3) -> Result<Self, Error> {
        if !offset.iter().chain(&velocity).all(|x| x.is_finite()) {
            return Err(Error::NonFinite);
        }
        Ok(Self { p: offset, dp: velocity, ..Self::identity() })
    }

    /// Apply this motion, then `next`: the resulting matrix is `next * self`.
    pub fn then(self, next: Self) -> Self {
        Self {
            r: mm(next.r, self.r),
            dr: ma(mm(next.dr, self.r), mm(next.r, self.dr)),
            p: add(mv(next.r, self.p), next.p),
            dp: add(add(mv(next.dr, self.p), mv(next.r, self.dp)), next.dp),
        }
    }

    pub(crate) fn is_finite(&self) -> bool {
        self.r.iter().flatten().chain(self.dr.iter().flatten()).chain(&self.p).chain(&self.dp)
            .all(|v| v.is_finite())
    }

    pub fn point(self, p: V3) -> V3 { add(mv(self.r, p), self.p) }
    pub fn vector(self, v: V3) -> V3 { mv(self.r, v) }
    pub fn velocity(self, p: V3) -> V3 { add(mv(self.dr, p), self.dp) }

    /// The moving inverse frame, including the derivative of its translated origin.
    pub fn inverse(self) -> Self {
        let r = std::array::from_fn(|i| std::array::from_fn(|j| self.r[j][i]));
        let dr = std::array::from_fn(|i| std::array::from_fn(|j| self.dr[j][i]));
        Self { r, dr, p: scaled(mv(r, self.p), -1.),
            dp: scaled(add(mv(dr, self.p), mv(r, self.dp)), -1.) }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    InvalidOptions,
    OutsideDomain,
    NonFinite,
    Degenerate,
    SingularIntersection,
    NotConverged,
}

/// Boundary checks allow zero tolerance, but never negative or nonfinite controls.
pub(crate) fn check_tolerance(tolerance: f64) -> Result<(),Error> {
    if tolerance.is_finite() && tolerance >= 0. { Ok(()) } else { Err(Error::InvalidOptions) }
}

pub(crate) fn check_iterations(max_iterations: u32) -> Result<(),Error> {
    if max_iterations == 0 || max_iterations > i32::MAX as u32/4 { Err(Error::InvalidOptions) }
    else { Ok(()) }
}

/// A moving surface point. `normal_velocity = 0` is the envelope equation.
#[derive(Clone, Copy, Debug)]
pub struct Contact {
    pub position: V3,
    pub normal: V3,
    pub velocity: V3,
    pub normal_velocity: f64,
}

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

/// Parameter scales condition the three unknowns [u,v,t]. Residual tolerances are in
/// the equation's own units: normal velocity, then each of the two section equations.
/// Finite bounds restrict the branch search; a seed and bounds do not prove uniqueness.
/// Equal bounds hold that parameter exactly, removing it from the numerical unknowns.
/// All three residuals must still satisfy their tolerances, including any redundant section.
#[derive(Clone, Copy, Debug)]
pub struct IntersectionOptions {
    pub bounds: [[f64; 2]; 3],
    pub parameter_scale: V3,
    pub residual_tolerance: V3,
    pub max_iterations: u32,
}

#[derive(Clone, Copy, Debug)]
pub struct Intersection {
    pub parameters: V3,
    pub contact: Contact,
    /// Residuals in physical units, not the conditioned values used by the solver.
    pub residuals: V3,
    pub iterations: u32,
}

/// Intersect an envelope with two section equations, such as axial position and radius.
/// Supply a previous intersection as the next seed to follow a local branch.
/// The exact envelope residual is evaluated at every trial; only its Jacobian is differenced.
pub fn intersect(
    surface: impl Fn(f64, f64) -> SurfacePoint,
    motion: impl Fn(f64) -> Motion,
    section: impl Fn(&Contact) -> [f64; 2],
    seed: V3,
    options: IntersectionOptions,
) -> Result<Intersection, Error> {
    intersect_parameters(surface, motion, |_, c| section(c), seed, options)
}

/// As `intersect`, with section equations allowed to read [u,v,t]. This also permits
/// sampling a generating surface at fixed (u,v), solving only for its contact motion.
/// Section tolerances then have parameter units, not necessarily length units.
pub fn intersect_parameters(
    surface: impl Fn(f64, f64) -> SurfacePoint,
    motion: impl Fn(f64) -> Motion,
    section: impl Fn(V3, &Contact) -> [f64; 2],
    seed: V3,
    options: IntersectionOptions,
) -> Result<Intersection, Error> {
    intersect_evaluated(|p| contact(surface(p[0],p[1]),motion(p[2])),section,seed,options)
}

/// Common fallible evaluator for named and closure-defined envelopes. A source evaluation
/// failure rejects a trial; it never substitutes another surface or motion.
pub(crate) fn intersect_evaluated(
    sample: impl Fn(V3) -> Result<Contact,Error>,
    section: impl Fn(V3, &Contact) -> [f64;2],
    seed: V3,
    options: IntersectionOptions,
) -> Result<Intersection,Error> {
    let result = crate::intersection::solve(|p| {
        let c = sample(p)?;
        let s = section(p,&c);
        Ok((c,[c.normal_velocity,s[0],s[1]]))
    },seed,options)?;
    Ok(Intersection {parameters:result.parameters,contact:result.value,
        residuals:result.residuals,iterations:result.iterations})
}
