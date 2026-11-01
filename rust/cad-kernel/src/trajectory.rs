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
use crate::util::space::{add,cross,dot,length,sub};
use crate::{Kind,KernelError};
use std::sync::Arc;

type V3 = [f64;3];
type M3 = [[f64;3];3];

fn mv(a: M3, b: V3) -> V3 { a.map(|r| dot(r, b)) }
// (written out: the sum of an iterator is -0.0 plus its terms in order, the same number, and was not
// inlined, the hottest thing in a motion's pose)
fn mm(a: M3, b: M3) -> M3 {
    std::array::from_fn(|i| std::array::from_fn(|j| a[i][0]*b[0][j]+a[i][1]*b[1][j]+a[i][2]*b[2][j]))
}
fn ma(a: M3, b: M3) -> M3 {
    std::array::from_fn(|i| add(a[i], b[i]))
}
fn scaled(a: V3, k: f64) -> V3 {
    [a[0] * k, a[1] * k, a[2] * k]
}
/// `a` over its length, where it has one.
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
    pub fn identity() -> Self {
        Self { r: [[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]], dr: [[0.;3];3],
            p: [0.;3], dp: [0.;3] }
    }

    /// Right-handed rotation about an axis through the origin. Angle is in radians;
    /// rate is radians per unit of the motion parameter. The axis need not be a unit vector.
    pub fn rotation(axis: V3, angle: f64, rate: f64) -> Result<Self, Error> {
        if !angle.is_finite() || !rate.is_finite() { return Err(Error::NonFinite); }
        let a = normalized(axis).ok_or(Error::Degenerate)?;
        let (s, c) = angle.dsin_cos();
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

    /// A pose `x ↦ r·x + p` and its rate, `dr` and `dp`, given outright: a frame read off a path
    /// (a frame read off a path, #398 C), whose turn is no rotation about one axis.
    pub fn of_parts(r: M3, dr: M3, p: V3, dp: V3) -> Self {
        Self { r, dr, p, dp }
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

    pub fn is_finite(&self) -> bool {
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


impl Motion {
    /// The same pose, its rate taken `rate` times as fast: the pose a path stands at when a pace
    /// running at `rate` reaches it (`Trajectory::Paced`).
    pub fn paced(self, rate: f64) -> Self {
        Self { dr: self.dr.map(|row| scaled(row, rate)), dp: scaled(self.dp, rate), ..self }
    }

    /// The pose's turn, row-major, and its offset: `x ↦ r·x + p`.
    pub fn pose(self) -> (M3, V3) { (self.r, self.p) }
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

    /// The same field written in the coordinates `x' = r x + t` (a frame's `placing`).
    pub fn moved(&self, m: &crate::geom::Rigid) -> Twist {
        let omega = m.vector(self.omega);
        Twist { omega, v0: sub(m.vector(self.v0), cross(omega, m.t)) }
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

fn transpose(a: M3) -> M3 { std::array::from_fn(|i| std::array::from_fn(|j| a[j][i])) }

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
    pub fn is_turn(&self) -> bool { self.advance == 0. }
    /// Whether it advances without turning: a slide, whose path has no period.
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
    pub fn radius(&self,p: [f64;3]) -> f64 {
        let q = crate::util::space::sub(p,self.origin);
        crate::util::space::norm(crate::util::space::sub(q,crate::util::space::scale(self.axis,crate::util::space::dot(q,self.axis))))
    }
    /// Where `p` stands across the motion's sections: its height along the axis for a screw or a
    /// slide, which carry it from one plane square to the axis to the next, or its angle about
    /// the axis for a turn, which carries it from one half-plane through the axis to the next.
    pub fn station(&self,p: [f64;3]) -> f64 { if self.is_turn() { self.angle(p) } else { self.height(p) } }
    /// How much of the roll carries it once round its axis (infinite for a slide).
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
    pub fn velocity(&self,p: [f64;3]) -> [f64;3] {
        let turn = crate::util::space::cross(self.axis,crate::util::space::sub(p,self.origin));
        std::array::from_fn(|k| self.ratio*turn[k]+self.advance/std::f64::consts::TAU*self.axis[k])
    }
    /// How far along the axis `p` stands from the origin.
    pub fn height(&self,p: [f64;3]) -> f64 { crate::util::space::dot(crate::util::space::sub(p,self.origin),self.axis) }
    /// The time at which the screw carries `p` to `height` along its axis.
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
    pub fn carry(&self,p: [f64;3],t: f64) -> [f64;3] {
        let q = self.turn(crate::util::space::sub(p,self.origin),t);
        std::array::from_fn(|k| self.origin[k]+q[k]+self.advance*t/std::f64::consts::TAU*self.axis[k])
    }
    /// A direction turned as `carry` turns what it carries.
    pub fn turn(&self,v: [f64;3],t: f64) -> [f64;3] {
        let (s,c) = (self.ratio*t).dsin_cos();
        let a = self.axis;
        let along = crate::util::space::dot(a,v);
        let across = crate::util::space::cross(a,v);
        std::array::from_fn(|k| v[k]*c+across[k]*s+a[k]*along*(1.-c))
    }
}


impl Screw {
    /// Its twist, the same at every time and in the carried frame as in the world's: `ω` its
    /// `ratio` along its axis, `v₀` the velocity of the origin (`velocity`).
    pub fn twist(&self) -> Twist {
        let omega = scaled(self.axis, self.ratio);
        Twist { omega, v0: add(cross(self.origin, omega), scaled(self.axis, self.advance/std::f64::consts::TAU)) }
    }

    /// Its pose at `t` and its rate: turned `ratio · t` about its axis and slid `advance · t / τ`
    /// along it, a slide (no `ratio`) by its step alone.
    pub fn at(&self, t: f64) -> Result<Motion, Error> {
        let rate = self.advance/std::f64::consts::TAU;
        let slide = Motion::translation(self.axis.map(|v| v*rate*t), self.axis.map(|v| v*rate))?;
        if self.ratio == 0. { return Ok(slide) }
        let rotation = Motion::rotation(self.axis, self.ratio*t, self.ratio)?;
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
    /// A path made of pieces, each over its stretch of the parameter, in order, each meeting the
    /// next where their stretches meet (`Trajectory::pieces`).
    Pieces(Vec<Piece>),
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
        let m = match self {
            Trajectory::Twist(s) => s.at(t).map_err(|e| read(e, t))?,
            Trajectory::Relative {source, observer} => source.at(t)?.then(observer.at(t)?.inverse()),
            Trajectory::Paced {path, pace} => {
                let [s, rate] = pace.jet(t)?;
                path.at(s)?.paced(rate)
            }
            Trajectory::Pieces(pieces) => pieces.iter().find(|p| p.over[0] <= t && t <= p.over[1])
                .ok_or_else(|| KernelError::new(Kind::Refused, format!("a path of pieces over \
                    [{}, {}] has no pose at {t}", pieces[0].over[0], pieces[pieces.len()-1].over[1])))?
                .path.at(t)?,
            Trajectory::General(p) => p.at(t)?,
        };
        if !m.is_finite() { return Err(read(Error::NonFinite, t)) }
        Ok(m)
    }

    /// The twist at `t`, read on what the trajectory carries (`Twist::body`): a constant twist's
    /// own, at every time.
    pub fn twist(&self, t: f64) -> Result<Twist, KernelError> {
        match self {
            Trajectory::Twist(s) => Ok(s.twist()),
            _ => Ok(Twist::body(self.at(t)?)),
        }
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
            let (ra, pa) = a.pose();
            let (rb, pb) = b.pose();
            let turn = (0..3).map(|i| (0..3).map(|j| (ra[i][j]-rb[i][j]).powi(2)).sum::<f64>()).sum::<f64>().sqrt();
            let apart = length(sub(pa, pb))+turn*reach;
            if !(apart <= tolerance) {
                return Err(refused(format!("two pieces part by {apart} where they meet at {t}")))
            }
        }
        Ok(Trajectory::Pieces(pieces))
    }
}
