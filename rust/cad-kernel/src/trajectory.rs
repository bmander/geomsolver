//! The **trajectory** a solid is swept along, as the kernel reads it (#721 F2): a rigid motion
//! keeping its structure as data, so that a sweep chooses its method from what the motion is and
//! never from how a drawing wrote it. A constant twist is held exactly (`Screw`: a screw, a turn or
//! a slide), one motion seen from another as the two (`Relative`), a motion taken at a pace as the
//! path and the pace (`Paced`: a sweep is its path's, not its pace's, #684 L), a path made of
//! pieces as its pieces (`Pieces`, K1), and any other rigid motion as a pose and its rate at each
//! time (`General`, K2).
//!
//! Every reading of a trajectory is a `Motion`, a pose and its derivative by the trajectory's
//! parameter, and at each time any rigid motion is a screw (Mozzi–Chasles): its **twist**, the
//! velocity field `v(x) = v₀ + ω × x` (`Twist`), read in the world (`Twist::spatial`) or on what
//! moves (`Twist::body`). A surface's contact under a twist is `contact`'s.
//!
//! The kernel knows no drawing: the solver lowers its motion to a `Trajectory`
//! (`motion::Family::trajectory`), and a refusal here is the kernel's, said by the solver.
#[allow(unused_imports)]
use crate::util::fmath::Det;
use crate::util::space::{add,cross,dot,length,scale,sub};
use crate::{Kind,KernelError};
use std::sync::Arc;

type V3 = [f64;3];
type M3 = [[f64;3];3];

// Everything a pose is read by is `#[inline]`: an envelope's root search reads a pose and its rate
// many thousand times from the solver, another crate, where only an inline function is inlined
// outside a whole-program build.
#[inline]
fn mv(a: M3, b: V3) -> V3 { a.map(|r| dot(r, b)) }
// (written out: the sum of an iterator is -0.0 plus its terms in order, the same number, and was not
// inlined, the hottest thing in a motion's pose)
#[inline]
fn mm(a: M3, b: M3) -> M3 {
    std::array::from_fn(|i| std::array::from_fn(|j| a[i][0]*b[0][j]+a[i][1]*b[1][j]+a[i][2]*b[2][j]))
}
#[inline]
fn ma(a: M3, b: M3) -> M3 {
    std::array::from_fn(|i| add(a[i], b[i]))
}
#[inline]
fn transpose(a: M3) -> M3 { std::array::from_fn(|i| std::array::from_fn(|j| a[j][i])) }
/// `a` over its length, where it has one.
#[inline]
pub fn normalized(a: V3) -> Option<V3> {
    let n = length(a);
    (n > 0.0 && n.is_finite()).then(|| a.map(|v| v/n))
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
    #[inline]
    pub fn identity() -> Self {
        Self { r: [[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]], dr: [[0.;3];3],
            p: [0.;3], dp: [0.;3] }
    }

    /// Right-handed rotation about an axis through the origin. Angle is in radians;
    /// rate is radians per unit of the motion parameter. The axis need not be a unit vector.
    pub fn rotation(axis: V3, angle: f64, rate: f64) -> Result<Self, Error> {
        if !angle.is_finite() || !rate.is_finite() { return Err(Error::NonFinite); }
        Ok(Self::turn(normalized(axis).ok_or(Error::Degenerate)?, angle, rate))
    }

    /// `rotation` about the unit axis `a`, read as it is.
    fn turn(a: V3, angle: f64, rate: f64) -> Self {
        let (s, c) = angle.dsin_cos();
        let k = [[0.,-a[2],a[1]], [a[2],0.,-a[0]], [-a[1],a[0],0.]];
        let r = std::array::from_fn(|i| std::array::from_fn(|j|
            c * if i == j { 1. } else { 0. } + (1.-c)*a[i]*a[j] + s*k[i][j]));
        let dr = mm(k, r).map(|row| scale(row, rate));
        Self { r, dr, ..Self::identity() }
    }

    #[inline]
    pub fn translation(offset: V3, velocity: V3) -> Result<Self, Error> {
        if !offset.iter().chain(&velocity).all(|x| x.is_finite()) {
            return Err(Error::NonFinite);
        }
        Ok(Self { p: offset, dp: velocity, ..Self::identity() })
    }

    /// A pose `x ↦ r·x + p` and its rate, `dr` and `dp`, given outright: a frame read off a path
    /// (#398 C), whose turn is no rotation about one axis.
    #[inline]
    pub fn of_parts(r: M3, dr: M3, p: V3, dp: V3) -> Self {
        Self { r, dr, p, dp }
    }

    /// Apply this motion, then `next`: the resulting matrix is `next * self`.
    #[inline]
    pub fn then(self, next: Self) -> Self {
        Self {
            r: mm(next.r, self.r),
            dr: ma(mm(next.dr, self.r), mm(next.r, self.dr)),
            p: add(mv(next.r, self.p), next.p),
            dp: add(add(mv(next.dr, self.p), mv(next.r, self.dp)), next.dp),
        }
    }

    #[inline]
    pub fn is_finite(&self) -> bool {
        self.r.iter().flatten().chain(self.dr.iter().flatten()).chain(&self.p).chain(&self.dp)
            .all(|v| v.is_finite())
    }

    #[inline]
    pub fn point(self, p: V3) -> V3 { add(mv(self.r, p), self.p) }
    #[inline]
    pub fn vector(self, v: V3) -> V3 { mv(self.r, v) }
    #[inline]
    pub fn velocity(self, p: V3) -> V3 { add(mv(self.dr, p), self.dp) }

    /// The moving inverse frame, including the derivative of its translated origin.
    #[inline]
    pub fn inverse(self) -> Self {
        let (r, dr) = (transpose(self.r), transpose(self.dr));
        Self { r, dr, p: scale(mv(r, self.p), -1.),
            dp: scale(add(mv(dr, self.p), mv(r, self.dp)), -1.) }
    }

    /// The same pose, its rate taken `rate` times as fast: the pose a path stands at when a pace
    /// running at `rate` reaches it (`Trajectory::Paced`).
    #[inline]
    pub fn paced(self, rate: f64) -> Self {
        Self { dr: self.dr.map(|row| scale(row, rate)), dp: scale(self.dp, rate), ..self }
    }
}

/// Why a motion's pose or a contact could not be read.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    InvalidOptions,
    OutsideDomain,
    NonFinite,
    Degenerate,
    SingularIntersection,
    NotConverged,
}

/// A rigid motion's **twist** at one instant: the velocity field `v(x) = v₀ + ω × x` it moves
/// space by, `ω` its angular velocity and `v₀` the velocity of the point at the origin, both per
/// unit of the trajectory's parameter. Every instant of a rigid motion is one (Mozzi–Chasles), and a
/// screw's is the same at every time.
#[derive(Clone,Copy,Debug,PartialEq)]
pub struct Twist { pub omega: V3, pub v0: V3 }

impl Twist {
    /// The velocity of the point at `x`.
    #[inline]
    pub fn velocity(&self, x: V3) -> V3 { add(self.v0, cross(self.omega, x)) }

    /// The twist `m` moves the world by at its pose: the velocity of what stands at `y` now is
    /// `ṙ rᵀ (y − p) + ṗ`.
    pub fn spatial(m: Motion) -> Twist {
        let w = mm(m.dr, transpose(m.r));
        let omega = skew_axis(w);
        Twist { omega, v0: sub(m.dp, cross(omega, m.p)) }
    }

    /// The twist `m` moves what it carries by, read on what it carries: the velocity of its point
    /// `x` is `rᵀ (ṙ x + ṗ)`, in its own coordinates. A world normal `r n` dotted with the world
    /// velocity is `n` dotted with this one's, so a carried surface's contact is read on the
    /// surface as it is written (`contact`).
    pub fn body(m: Motion) -> Twist {
        let rt = transpose(m.r);
        Twist { omega: skew_axis(mm(rt, m.dr)), v0: mv(rt, m.dp) }
    }

    /// The twist read off the screw it is, where it turns: the line along which every point moves,
    /// through `point`, its nearest the origin, along the unit `direction` of `ω`, advancing
    /// `pitch` along it for each radian it turns (`motion::Isa`'s).
    pub fn axis(&self) -> Option<(V3, V3, f64)> {
        let ww = dot(self.omega, self.omega);
        if !(ww > 0.) { return None }
        let point = cross(self.omega, self.v0).map(|x| x/ww);
        Some((point, normalized(self.omega)?, dot(self.omega, self.v0)/ww))
    }
}

/// The axial vector of a (nearly) skew matrix, its skew part's: `w` with `w × x = a x`.
fn skew_axis(a: M3) -> V3 {
    [(a[2][1]-a[1][2])/2., (a[0][2]-a[2][0])/2., (a[1][0]-a[0][1])/2.]
}

/// A **constant twist** (#684), a screw: it turns `ratio` radians a radian about the line through
/// `origin` along the unit `axis`, and slides `advance` along it a turn of the parameter. A turn
/// (`advance` zero) and a slide (`ratio` zero) are its two limits. The solver reads one off a
/// motion as `motion::Screw` (`Family::twist`).
#[derive(Clone,Copy,Debug,PartialEq)]
pub struct Screw { pub origin: [f64;3], pub axis: [f64;3], pub ratio: f64, pub advance: f64 }

impl Screw {
    /// Whether it turns without advancing: a turn, whose sections (`station`) are half-planes
    /// about its axis, where a screw's and a slide's are planes square to it.
    #[inline]
    pub fn is_turn(&self) -> bool { self.advance == 0. }
    /// Whether it advances without turning: a slide, whose path has no period.
    #[inline]
    pub fn is_slide(&self) -> bool { self.ratio == 0. }
    /// Which of the three it is, as a refusal or a report says it.
    pub fn kind(&self) -> &'static str { if self.is_turn() { "a turn" } else if self.is_slide() { "a slide" } else { "a screw" } }
    /// How far round its axis `p` stands, from the axis's own frame's `x` (`Frame::about`): a
    /// turn carries it round at the same height and radius.
    pub fn angle(&self,p: [f64;3]) -> f64 {
        let l = crate::geom::Frame::about(self.origin,self.axis).local(p);
        l[1].datan2(l[0])
    }
    /// How far `p` stands from the axis, which none of the motions changes.
    #[inline]
    pub fn radius(&self,p: [f64;3]) -> f64 {
        let q = crate::util::space::sub(p,self.origin);
        crate::util::space::norm(crate::util::space::sub(q,crate::util::space::scale(self.axis,crate::util::space::dot(q,self.axis))))
    }
    /// Where `p` stands across the motion's sections: its height along the axis for a screw or a
    /// slide, which carry it from one plane square to the axis to the next, or its angle about
    /// the axis for a turn, which carries it from one half-plane through the axis to the next.
    #[inline]
    pub fn station(&self,p: [f64;3]) -> f64 { if self.is_turn() { self.angle(p) } else { self.height(p) } }
    /// How much of the roll carries it once round its axis (infinite for a slide).
    #[inline]
    pub fn period(&self) -> f64 { std::f64::consts::TAU/self.ratio.abs() }
    /// The time at which the motion carries `p` to `station` (`station`'s kind): for a turn, which
    /// reaches it once a turn, the one nearest `near`.
    pub fn time_to_station(&self,p: [f64;3],station: f64,near: f64) -> f64 {
        if !self.is_turn() { return self.time_to(p,station) }
        let period = self.period();
        let t = (station-self.angle(p))/self.ratio;
        t+((near-t)/period).round()*period
    }
    /// The times, from now, at which the motion holds `p` between the heights `band` along its
    /// axis: a screw or a slide carries it through them once, and a turn keeps it at its own
    /// height, so holds it there always or never (`None`).
    pub fn passing(&self,p: [f64;3],[lo,hi]: [f64;2]) -> Option<[f64;2]> {
        if self.is_turn() {
            let h = self.height(p);
            return (h >= lo.min(hi) && h <= lo.max(hi)).then_some([f64::NEG_INFINITY,f64::INFINITY])
        }
        let (x,y) = (self.time_to(p,lo),self.time_to(p,hi));
        Some([x.min(y),x.max(y)])
    }
    /// The velocity per radian of the point at `p` of what the screw carries, in the carried frame
    /// as in the world's: the same at every time, since a screw carries its own axis into itself.
    #[inline]
    pub fn velocity(&self,p: [f64;3]) -> [f64;3] {
        let turn = crate::util::space::cross(self.axis,crate::util::space::sub(p,self.origin));
        std::array::from_fn(|k| self.ratio*turn[k]+self.advance/std::f64::consts::TAU*self.axis[k])
    }
    /// How far along the axis `p` stands from the origin.
    #[inline]
    pub fn height(&self,p: [f64;3]) -> f64 { crate::util::space::dot(crate::util::space::sub(p,self.origin),self.axis) }
    /// The time at which the screw carries `p` to `height` along its axis.
    #[inline]
    pub fn time_to(&self,p: [f64;3],height: f64) -> f64 { (height-self.height(p))*std::f64::consts::TAU/self.advance }
    /// How far along the axis the box from `lo` to `hi` reaches (read in the screw's frame through
    /// `back`), and how far from the axis.
    pub fn extent(&self,lo: [f64;3],hi: [f64;3],back: Motion) -> ([f64;2],f64) {
        (0..8).map(|k| back.point(std::array::from_fn(|i| if k>>i & 1 == 0 { lo[i] } else { hi[i] })))
            .fold(([f64::INFINITY,f64::NEG_INFINITY],0_f64),|([a,b],r),p| {
                let h = self.height(p);
                ([a.min(h),b.max(h)],r.max(self.radius(p)))
            })
    }
    /// `p` carried along its own path for a time `t`: turned `ratio·t` about the axis and slid
    /// along it, as the screw's pose at `s + t` is its pose at `s` followed by this one (turns and
    /// slides about one axis commute).
    #[inline]
    pub fn carry(&self,p: [f64;3],t: f64) -> [f64;3] {
        let q = self.turn(crate::util::space::sub(p,self.origin),t);
        std::array::from_fn(|k| self.origin[k]+q[k]+self.advance*t/std::f64::consts::TAU*self.axis[k])
    }
    /// A direction turned as `carry` turns what it carries.
    #[inline]
    pub fn turn(&self,v: [f64;3],t: f64) -> [f64;3] {
        let (s,c) = (self.ratio*t).dsin_cos();
        let a = self.axis;
        let along = crate::util::space::dot(a,v);
        let across = crate::util::space::cross(a,v);
        std::array::from_fn(|k| v[k]*c+across[k]*s+a[k]*along*(1.-c))
    }

    /// Its twist, the same at every time and in the carried frame as in the world's: `ω` its
    /// `ratio` along its axis, `v₀` the velocity of the origin (`velocity`).
    pub fn twist(&self) -> Twist { Twist { omega: scale(self.axis, self.ratio), v0: self.velocity([0.;3]) } }

    /// Its pose at `t` and its rate: turned `ratio · t` about its axis and slid `advance · t / τ`
    /// along it, a slide (no `ratio`) by its step alone.
    pub fn at(&self, t: f64) -> Result<Motion, Error> { self.at_rate(t, 1.) }

    /// Its pose at `t`, its rate taken `rate` times as fast: where a pace running at `rate` takes it
    /// (`Trajectory::Paced`), each step's rate its own times the pace's. Its axis is unit, read as it
    /// is.
    pub fn at_rate(&self, t: f64, rate: f64) -> Result<Motion, Error> {
        let advance = self.advance/std::f64::consts::TAU;
        let slide = Motion::translation(self.axis.map(|v| v*advance*t), self.axis.map(|v| v*advance*rate))?;
        if self.ratio == 0. { return Ok(slide) }
        let (angle, turning) = (self.ratio*t, self.ratio*rate);
        if !angle.is_finite() || !turning.is_finite() { return Err(Error::NonFinite) }
        let rotation = Motion::turn(self.axis, angle, turning);
        let pose = Motion::translation(self.origin.map(|v| -v), [0.;3])?
            .then(rotation).then(Motion::translation(self.origin, [0.;3])?);
        Ok(if self.advance == 0. { pose } else { pose.then(slide) })
    }
}

/// A pace a path is taken at: the path's own parameter as a function of the trajectory's, its value
/// and its rate (a joint driven by a law, #391 S5).
pub trait Pace: Send + Sync + std::fmt::Debug {
    /// The path's parameter at `t`, and its rate by `t`.
    fn jet(&self, t: f64) -> Result<[f64; 2], KernelError>;
    /// The least and the greatest value the pace takes over `[a, b]`: being continuous, it takes
    /// every value between.
    fn range(&self, roll: [f64; 2]) -> Result<[f64; 2], KernelError>;
}

/// A rigid motion read only by evaluating it: its pose and its rate at each time (K2).
pub trait Path: Send + Sync + std::fmt::Debug {
    fn at(&self, t: f64) -> Result<Motion, KernelError>;
}

/// One piece of a path made of pieces: its own trajectory over `over`, a stretch of the parameter.
#[derive(Clone, Debug)]
pub struct Piece { pub over: [f64; 2], pub path: Trajectory }

/// Pieces that make one path, in order, each beginning where the one before it ends and standing
/// what it carries at the same pose there: made only by `Trajectory::pieces`, which checks it.
#[derive(Clone, Debug)]
pub struct Pieces(Vec<Piece>);

impl Pieces {
    pub fn pieces(&self) -> &[Piece] { &self.0 }
    /// The piece standing at `t`: at a join, the first, the two standing alike there.
    fn at(&self, t: f64) -> Option<&Piece> {
        let p = self.0.get(self.0.partition_point(|p| p.over[1] < t))?;
        (p.over[0] <= t).then_some(p)
    }
}

/// A rigid motion as the kernel sweeps along it, its structure kept (the module's doc).
#[derive(Clone, Debug)]
pub enum Trajectory {
    /// A constant twist: a screw, a turn or a slide, at a constant rate.
    Twist(Screw),
    /// `source` seen from `observer`: at each time `observer⁻¹ · source` (the generating class's
    /// relative rotation).
    Relative { source: Box<Trajectory>, observer: Box<Trajectory> },
    /// `path` taken at `pace`: at `t` the path's pose at the pace's value.
    Paced { path: Box<Trajectory>, pace: Arc<dyn Pace> },
    /// A path made of pieces, each over its stretch of the parameter (`Trajectory::pieces`).
    Pieces(Pieces),
    /// Any other rigid motion, read by evaluating it.
    General(Arc<dyn Path>),
}

/// The parameter or the error a reading failed at, as the kernel says it.
fn read(e: Error, t: f64) -> KernelError {
    KernelError::new(Kind::Failed, format!("a trajectory's pose at {t} could not be read: {e:?}"))
}

impl Trajectory {
    /// The pose at `t` and its rate by `t`.
    pub fn at(&self, t: f64) -> Result<Motion, KernelError> {
        if !t.is_finite() { return Err(read(Error::NonFinite, t)) }
        let m = self.unchecked(t)?;
        if !m.is_finite() { return Err(read(Error::NonFinite, t)) }
        Ok(m)
    }

    /// `at`, its parameter and its pose checked by `at` once rather than at each level.
    fn unchecked(&self, t: f64) -> Result<Motion, KernelError> {
        Ok(match self {
            Trajectory::Twist(s) => s.at(t).map_err(|e| read(e, t))?,
            Trajectory::Relative {source, observer} => source.unchecked(t)?.then(observer.unchecked(t)?.inverse()),
            Trajectory::Paced {path, pace} => {
                let [s, rate] = pace.jet(t)?;
                match **path {
                    Trajectory::Twist(screw) => screw.at_rate(s, rate).map_err(|e| read(e, s))?,
                    _ => path.unchecked(s)?.paced(rate),
                }
            }
            Trajectory::Pieces(pieces) => {
                let all = pieces.pieces();
                pieces.at(t).ok_or_else(|| KernelError::new(Kind::Refused, format!("a path of pieces over \
                    [{}, {}] has no pose at {t}", all[0].over[0], all[all.len()-1].over[1])))?.path.unchecked(t)?
            }
            Trajectory::General(p) => p.at(t)?,
        })
    }

    /// The constant twist this trajectory is, where it is one.
    pub fn screw(&self) -> Option<Screw> {
        match self { Trajectory::Twist(s) => Some(*s), _ => None }
    }

    /// **A sweep is its path's, not its pace's** (#684 L): the trajectory and the stretch of its
    /// parameter that stand what it carries at the same poses as this one over `roll`, every pace
    /// dropped. A pace being continuous, the path is taken over every value between the least and
    /// the greatest its pace reaches over the roll.
    pub fn unpaced(&self, roll: [f64; 2]) -> Result<(&Trajectory, [f64; 2]), KernelError> {
        match self {
            Trajectory::Paced {path, pace} => path.unpaced(pace.range(roll)?),
            _ => Ok((self, roll)),
        }
    }

    /// A path made of `pieces`: refused where a piece's stretch is empty or runs backwards, where
    /// one does not begin where the one before it ends, or where two do not stand what they carry
    /// at the same pose there to `tolerance`, the distance a point at `reach` from the origin may
    /// stand apart (the witness the parameter where they part).
    pub fn pieces(pieces: Vec<Piece>, tolerance: f64, reach: f64) -> Result<Trajectory, KernelError> {
        let refused = |m: String| KernelError::new(Kind::Refused, m);
        if pieces.is_empty() { return Err(refused("a path of pieces has none".into())) }
        for p in &pieces {
            if !(p.over[0] < p.over[1]) { return Err(refused(format!("a piece over [{}, {}] spans nothing", p.over[0], p.over[1]))) }
        }
        for w in pieces.windows(2) {
            let t = w[0].over[1];
            if w[1].over[0] != t {
                return Err(refused(format!("a piece ending at {t} is followed by one beginning at {}", w[1].over[0])))
            }
            let (a, b) = (w[0].path.at(t)?, w[1].path.at(t)?);
            // the farthest two poses carry a point within `reach` apart: the offsets' and the
            // turns' parting at that distance, a bound
            let turn = (0..3).map(|i| (0..3).map(|j| (a.r[i][j]-b.r[i][j]).powi(2)).sum::<f64>()).sum::<f64>().sqrt();
            let apart = length(sub(a.p, b.p))+turn*reach;
            if !(apart <= tolerance) {
                return Err(refused(format!("two pieces part by {apart} where they meet at {t}")))
            }
        }
        Ok(Trajectory::Pieces(Pieces(pieces)))
    }
}
