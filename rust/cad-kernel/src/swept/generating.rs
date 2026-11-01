//! **When a point is in contact under a generating sweep** (#721 F2c): the contact chart of a tool
//! point carrying a normal along a trajectory that turns it, in closed form — under a turn, or a
//! turn seen from another, `n·v` is one sinusoid in the roll; under a slide seen from a turn (a rack
//! against the blank it cuts) a line in it — and its isolated roots, each a contact time
//! (`Form`, `Chart`). The generating class reads each tool point by its contact times; a trajectory
//! with no chart is refused by what it is (`Uncharted`), and the solver says that in the drawing's
//! words.
#[allow(unused_imports)]
use crate::util::fmath::Det;
use crate::contact::{SurfacePoint,Contact,contact,check_tolerance};
use crate::trajectory::{Error,Motion,Screw,Trajectory,normalized};
use crate::util::space::{cross,dot,scale,sub};
use std::f64::consts::TAU;

/// Normal velocity at one fixed source point under a charted trajectory: under a rotation, or one
/// relative to a rotation, f(t) = constant + cosine*cos(rate*t) + sine*sin(rate*t); under a
/// translation relative to a rotation, f(t) = constant + slope*t, one root at most. Coefficients
/// describe the solved floating-point geometry, not interval bounds.
#[derive(Clone,Copy,Debug)]
pub enum NormalVelocity {
    Sinusoid { constant: f64,cosine: f64,sine: f64,rate: f64 },
    Affine { constant: f64,slope: f64 },
}

/// One isolated temporal root. Branch and turn identify the algebraic root and
/// winding at this source point; they are not global surface-sheet identities.
#[derive(Clone,Copy,Debug)]
pub struct ContactTime {
    pub time: f64,
    pub branch: usize,
    pub turn: i64,
}

/// Why a trajectory has no contact chart (`Form::of`), by what the trajectory is.
#[derive(Clone,Copy,Debug,PartialEq)]
pub enum Uncharted {
    /// Neither a turn, nor a turn or a slide seen from a turn about a fixed axis: a slide or a screw
    /// alone, a screw seen from anything or anything seen from one, a relative of a relative, a path
    /// of pieces.
    Motion,
    /// A path taken at a pace (a joint driven by a law), whose rate is its own, so its chart is no
    /// sinusoid of the roll.
    Paced,
    /// A rigid motion evaluated (a frame along a path, one the drawing moves), whose twist moves as
    /// it runs.
    Evaluated,
}

impl std::fmt::Display for Uncharted {
    fn fmt(&self,f: &mut std::fmt::Formatter) -> std::fmt::Result {
        f.write_str(match self {
            Uncharted::Motion => "temporal contacts require a rotation, or a rotation or a translation \
                seen from a rotation, without advance",
            Uncharted::Paced => "temporal contacts are read of constant rates, and a path taken at a \
                pace runs at its own",
            Uncharted::Evaluated => "temporal contacts are read of constant rates, and a motion \
                evaluated turns as it runs",
        })
    }
}

/// **What a trajectory's contact chart is**, read once of its structure: a turn without advance,
/// alone or seen from one (`Turned`), or a slide seen from such a turn, a rack (`Rack`). A turn of no
/// ratio is the slide it is, and an observer of no ratio is a frame at rest. Each screw's axis is
/// read as the unit axis the trajectory holds.
#[derive(Clone,Copy,Debug)]
pub enum Form { Turned { source: Screw,observer: Option<Screw> },Rack { slide: Screw,observer: Screw } }

impl Form {
    /// The form `path`'s chart takes, or why it has none. Nested relative motions can contain
    /// additional harmonics and are refused, as are paced and evaluated ones (`Uncharted`).
    pub fn of(path: &Trajectory) -> Result<Form,Uncharted> {
        let (source,observer) = match path {
            Trajectory::Relative {source,observer} => (&**source,Some(&**observer)),
            t => (t,None),
        };
        let twist = |t: &Trajectory| match t {
            Trajectory::Twist(s) => Ok(*s),
            Trajectory::Paced {..} => Err(Uncharted::Paced),
            Trajectory::General(_) => Err(Uncharted::Evaluated),
            Trajectory::Relative {..} | Trajectory::Pieces(_) => Err(Uncharted::Motion),
        };
        let (source,observer) = (twist(source)?,observer.map(twist).transpose()?);
        match observer {
            Some(o) if !o.is_turn() => Err(Uncharted::Motion),
            _ if source.is_turn() && !source.is_slide() => Ok(Form::Turned {source,observer}),
            Some(observer) if source.is_slide() => Ok(Form::Rack {slide:source,observer}),
            _ => Err(Uncharted::Motion),
        }
    }

    /// **The contact chart of `surface`'s point** (`NormalVelocity`). For M = B⁻¹A with fixed-axis
    /// rotations, the observer's rotation cancels from n·v. In the shared world frame, with
    /// x = p − origin_A,
    /// f = rate_A n·(axis_A × x) − ω_B·R_A(x × n) − (ω_B × (origin_A − origin_B))·R_A n.
    /// Rodrigues' formula makes this a single sinusoid in A's angle. Different axis origins and
    /// signed rates are retained; roll 0 is as drawn, so neither turn has a phase. A slide seen from
    /// a turn is affine in the roll (`rack`).
    pub fn normal_velocity(&self,surface: SurfacePoint) -> Result<NormalVelocity,PointContactError> {
        let n = contact(surface,Motion::identity()).map_err(|e| PointContactError::Failed(format!("{e:?}")))?.normal;
        match *self {
            Form::Turned {source,observer} => turned(surface.position,n,source,observer),
            Form::Rack {slide,observer} => rack(surface.position,n,slide,observer),
        }
    }
}

/// A turn `source` (no advance) seen from `observer`, a turn about a fixed axis, or alone: the chart
/// of the point at `p` with unit normal `n`.
fn turned(p: [f64;3],n: [f64;3],source: Screw,observer: Option<Screw>) -> Result<NormalVelocity,PointContactError> {
    let Screw {origin,axis,ratio,..} = source;
    let (omega,offset) = match observer {
        None => ([0.;3],[0.;3]),
        Some(o) => (scale(o.axis,o.ratio),sub(origin,o.origin)),
    };
    let x = sub(p,origin);
    let mut coefficients = [ratio*dot(n,cross(axis,x)),0.,0.];
    for (v,w) in [(omega,cross(x,n)),(cross(omega,offset),n)] {
        let parallel = scale(axis,dot(axis,w));
        let perpendicular = sub(w,parallel);
        coefficients[0] -= dot(v,parallel);
        coefficients[1] -= dot(v,perpendicular);
        coefficients[2] -= dot(v,cross(axis,w));
    }
    if !coefficients.iter().all(|v| v.is_finite()) { return Err(PointContactError::Overflow); }
    Ok(NormalVelocity::Sinusoid {constant:coefficients[0],cosine:coefficients[1],sine:coefficients[2],rate:ratio})
}

/// For M = B⁻¹ A with A a slide along unit `d` at `s` a radian and B a turn of rate vector ω about a
/// line through `o`: the normal is carried unturned, the source moves at `s d`, and the observer's
/// frame turns under it, so f(t) = s n·d − n·(ω × (p − o)) − s t n·(ω × d) — affine in the roll.
fn rack(p: [f64;3],n: [f64;3],slide: Screw,observer: Screw) -> Result<NormalVelocity,PointContactError> {
    let (d,s,omega) = (slide.axis,slide.advance/TAU,scale(observer.axis,observer.ratio));
    let x = sub(p,observer.origin);
    let (constant,slope) = (s*dot(n,d)-dot(n,cross(omega,x)),-s*dot(n,cross(omega,d)));
    if !constant.is_finite() || !slope.is_finite() { return Err(PointContactError::Overflow); }
    Ok(NormalVelocity::Affine {constant,slope})
}

impl NormalVelocity {
    /// `[c, a, b]`: the point is in contact at every time exactly where `c` and the vector `(a, b)`
    /// all vanish, and has roots only where `|c| ≤ |(a, b)|` — a sinusoid's constant and its cosine
    /// and sine coefficients; an affine form's `0` and its constant and slope, both of which a
    /// point in contact at every time has zero (admission's M2 looks for that zero between samples
    /// by the winding of `(a, b)`).
    #[inline]
    pub fn variation(&self) -> [f64;3] {
        match *self {
            NormalVelocity::Sinusoid {constant,cosine,sine,..} => [constant,cosine,sine],
            NormalVelocity::Affine {constant,slope} => [0.,constant,slope],
        }
    }

    #[inline]
    pub fn at(&self,time: f64) -> Result<f64,Error> {
        let value = match *self {
            NormalVelocity::Sinusoid {constant,cosine,sine,rate} => {
                let angle = rate*time;
                constant+cosine*angle.dcos()+sine*angle.dsin()
            }
            NormalVelocity::Affine {constant,slope} => constant+slope*time,
        };
        if time.is_finite() && value.is_finite() { Ok(value) } else { Err(Error::NonFinite) }
    }

    /// Enumerate isolated roots in a closed time interval. Constant-zero and
    /// near-double-root cases need another chart and return Degenerate. A root
    /// budget failure returns no partial result. Tolerance is normal velocity,
    /// not a root-position or end-to-end geometry error bound.
    pub fn roots(&self,domain: [f64;2],tolerance: f64,max_roots: usize)
        -> Result<Vec<ContactTime>,Error> {
        check_tolerance(tolerance)?;
        if tolerance == 0. || max_roots == 0 || max_roots > 1_000_000 || domain[0] > domain[1] {
            return Err(Error::InvalidOptions);
        }
        if !domain.iter().all(|v| v.is_finite()) { return Err(Error::NonFinite); }
        let (constant,cosine,sine,rate) = match *self {
            NormalVelocity::Sinusoid {constant,cosine,sine,rate} => (constant,cosine,sine,rate),
            // one root where the line crosses zero: degenerate where it is flat on zero
            NormalVelocity::Affine {constant,slope} => {
                let span = domain[1].abs().max(domain[0].abs()).max(1.);
                if slope.abs()*span <= tolerance {
                    return if constant.abs() <= tolerance { Err(Error::Degenerate) } else { Ok(Vec::new()) };
                }
                let time = -constant/slope;
                if !time.is_finite() { return Err(Error::NonFinite); }
                let clamped = time.clamp(domain[0],domain[1]);
                if (time-clamped).abs() > 16.*f64::EPSILON*(time.abs()+1.) { return Ok(Vec::new()); }
                return Ok(vec![ContactTime {time:clamped,branch:0,turn:0}]);
            }
        };
        let amplitude = cosine.dhypot(sine);
        if !amplitude.is_finite() { return Err(Error::NonFinite); }
        if rate == 0. || amplitude <= tolerance {
            return if self.at(domain[0])?.abs() <= tolerance { Err(Error::Degenerate) }
                else if rate == 0. || constant.abs() > amplitude+tolerance { Ok(Vec::new()) }
                else { Err(Error::Degenerate) };
        }
        if constant.abs() > amplitude+tolerance { return Ok(Vec::new()); }
        if (constant.abs()-amplitude).abs() <= tolerance { return Err(Error::Degenerate); }
        let offset = sine.datan2(cosine);
        let spread = (-constant/amplitude).dacos();
        let angles = domain.map(|t| rate*t);
        let lo = angles[0].min(angles[1]); let hi = angles[0].max(angles[1]);
        if !lo.is_finite() || !hi.is_finite() { return Err(Error::NonFinite); }
        let mut result = Vec::new();
        for (branch,base) in [offset-spread,offset+spread].into_iter().enumerate() {
            // Include neighboring windings so endpoint arithmetic cannot exclude
            // a root by one ulp. Only roundoff-sized endpoint overruns are clamped.
            let first = ((lo-base)/TAU).floor()-1.;
            let last = ((hi-base)/TAU).ceil()+1.;
            if first.abs().max(last.abs()) >= (1_u64<<52) as f64
                || last-first > max_roots as f64+8. { return Err(Error::NotConverged); }
            for turn in first as i64..=last as i64 {
                let angle = base+TAU*turn as f64;
                let angular_roundoff = 16.*f64::EPSILON*(angle.abs()+lo.abs()+hi.abs()+1.);
                if angle < lo-angular_roundoff || angle > hi+angular_roundoff { continue; }
                let time = angle/rate;
                if !time.is_finite() { return Err(Error::NonFinite); }
                let clamped = time.clamp(domain[0],domain[1]);
                let roundoff = 16.*f64::EPSILON*(time.abs()+clamped.abs()+1.);
                if (time-clamped).abs() > roundoff { continue; }
                if self.at(clamped)?.abs() > tolerance { return Err(Error::NotConverged); }
                result.push(ContactTime {time:clamped,branch,turn});
                if result.len() > max_roots { return Err(Error::NotConverged); }
            }
        }
        result.sort_by(|a,b| a.time.total_cmp(&b.time));
        if result.windows(2).any(|p| p[0].time == p[1].time) { return Err(Error::Degenerate); }
        Ok(result)
    }
}

/// A contact: when (`root`), and where the point is then and its normal (`contact`).
#[derive(Clone,Copy,Debug)]
pub struct TimedContact {
    pub root: ContactTime,
    pub contact: Contact,
}

/// Why a point carrying a normal has no contact times: its normal is no direction, its contact
/// equation is degenerate (in contact at every time, or only grazing, which the root finder
/// cannot tell apart), its chart's coefficients overflow, a root found fails the contact equation it
/// was found of (`Unsatisfied`), or something failed that says nothing about the point.
#[derive(Clone,Debug,PartialEq)]
pub enum PointContactError { DegenerateNormal,Degenerate,Overflow,Unsatisfied,Failed(String) }

impl PointContactError {
    fn of(e: Error) -> Self {
        if e == Error::Degenerate { PointContactError::Degenerate } else { PointContactError::Failed(format!("{e:?}")) }
    }
}

impl std::fmt::Display for PointContactError {
    fn fmt(&self,f: &mut std::fmt::Formatter) -> std::fmt::Result {
        match self {
            PointContactError::DegenerateNormal => f.write_str("degenerate normal"),
            PointContactError::Degenerate => f.write_str("Degenerate"),
            PointContactError::Overflow => f.write_str("normal-velocity coefficients overflowed"),
            PointContactError::Unsatisfied => f.write_str("point-normal contact failed the normal-velocity equation"),
            PointContactError::Failed(m) => f.write_str(m),
        }
    }
}

/// **A generating sweep's contact times**: the trajectory a tool is carried along, the form of its
/// chart (`Form`, read once) and the roll it is swept over (`domain`), each tool point read by when
/// it is in contact.
#[derive(Clone,Copy,Debug)]
pub struct Chart<'a> { pub path: &'a Trajectory,form: Form,roll: [f64;2] }

impl<'a> Chart<'a> {
    /// The chart of `path` over `roll`, or why it has none.
    pub fn new(path: &'a Trajectory,roll: [f64;2]) -> Result<Self,Uncharted> { Ok(Chart {path,form:Form::of(path)?,roll}) }

    /// The roll the contacts are read over.
    #[inline]
    pub fn domain(&self) -> [f64;2] { self.roll }

    /// Contact times of a point carrying a given unit normal, over a motion interval. This is the
    /// sharp-edge sweep condition: a convex source edge carries every normal between its incident
    /// faces', and each such normal contacts where it is perpendicular to the point's velocity.
    /// The caller supplies the normal; nothing here checks that the edge actually carries it.
    pub fn at_point_normal(&self,position: [f64;3],normal: [f64;3],interval: [f64;2],tolerance: f64)
        -> Result<Vec<TimedContact>,PointContactError> {
        let n = normalized(normal).ok_or(PointContactError::DegenerateNormal)?;
        let seed = if n[0].abs() < 0.9 { [1.,0.,0.] } else { [0.,1.,0.] };
        let du = normalized(cross(seed,n)).ok_or(PointContactError::DegenerateNormal)?;
        let dv = cross(n,du);
        self.at_surface(SurfacePoint {position,du,dv},interval,tolerance)
    }

    /// Contact times of a surface's point (its normal `du × dv`) over a motion interval. Contacts
    /// outside the declared roll lie on the envelope of a longer motion, not on this sweep's
    /// boundary; a caller may use them as an extended candidate sheet only where a separate
    /// material classification decides what is exposed.
    pub fn at_surface(&self,surface: SurfacePoint,interval: [f64;2],tolerance: f64)
        -> Result<Vec<TimedContact>,PointContactError> {
        let roots = self.form.normal_velocity(surface)?.roots(interval,tolerance,4096).map_err(PointContactError::of)?;
        roots.into_iter().map(|root| {
            let at = self.path.at(root.time).map_err(|e| PointContactError::Failed(e.msg))?;
            let contact = contact(surface,at).map_err(PointContactError::of)?;
            if contact.normal_velocity.abs() > tolerance { return Err(PointContactError::Unsatisfied) }
            Ok(TimedContact {root,contact})
        }).collect()
    }
}
