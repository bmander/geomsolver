//! Rigid-motion families: graphs of turns, screws and slides about fixed axes and of one motion
//! seen from another, every node reading the same angular parameter in radians — the roll.  A
//! family is built from its steps (`Family::new`), or read off a caller's document through its
//! `Source`.  A joint driven by a law of the roll (`Drive`) and a frame carried along a path or
//! by an outside march (`Carrier`) are the caller's: the family asks them for their value, their
//! pose and an enclosure over a stretch of the roll, and knows nothing of how they are read.
#[allow(unused_imports)]
use crate::util::fmath::Det;
mod bounds;
pub use bounds::MotionBounds;
mod contact;
pub use contact::{NormalVelocity,ContactTime};

use crate::sweep::envelope::Motion;
use crate::util::interval::{Error as IntervalError,Interval as I};
use crate::util::space::{cross,dot,normalised,scale,sub};
use std::sync::Arc;

/// **A joint's own number as a law of the roll** (a cam, #391 S5 in the solver): its value, its
/// value and rate, and both enclosed over an interval of the roll.  The caller's; the family
/// stands a driven joint at its law's value where an undriven one stands at the roll itself.
pub trait Drive: std::fmt::Debug+Send+Sync {
    /// The law's value at the roll `theta`.
    fn value(&self,theta: f64) -> Result<f64,String>;
    /// The law's value and its rate a radian of the roll at `theta`.
    fn jet(&self,theta: f64) -> Result<[f64;2],String>;
    /// The law's value and rate enclosed over the interval `theta`.
    fn enclose(&self,theta: I) -> Result<[I;2],IntervalError>;
}

/// **A frame carried along the roll by something other than a joint** — along a path, or as an
/// outside march moves it (#398 C, F in the solver): its pose and rate, its pose alone, and what
/// it does over a stretch of the roll enclosed (`Stretch`), `None` where it has no bound.
pub trait Carrier: std::fmt::Debug+Send+Sync {
    /// The pose and its rate at `u`.
    fn motion(&self,u: f64) -> Result<Motion,String>;
    /// The pose alone at `u`.
    fn pose(&self,u: f64) -> Result<Pose,String>;
    /// What the carriage does over `[a, b]`, enclosed, or `None` where it has no bound.
    fn stretch(&self,a: f64,b: f64) -> Option<Stretch>;
    /// The fastest a point of the box `b` is carried over `[lo, hi]`, where the carrier bounds it
    /// box by box rather than by its stretch (a march read span by span).
    fn speed_over(&self,_b: [I;3],_lo: f64,_hi: f64) -> Option<f64> { None }
}

/// A frame carried over a stretch of its parameter, enclosed (`Carrier::stretch`): the
/// carried point (the path's), the turn's entries, the speed and the frame's angular speed
/// bounded, and the point at the home the turn is about.
#[derive(Clone, Copy, Debug)]
pub struct Stretch {
    pub point: [I; 3],
    pub turn: [[I; 3]; 3],
    pub speed: f64,
    pub spin: f64,
    pub origin: [f64; 3],
}

/// `r` reflected in the plane square to `v`.
fn reflect(r: [f64; 3], v: [f64; 3], vv: f64) -> [f64; 3] {
    sub(r, scale(v, 2.0 * dot(v, r) / vv))
}

/// **One step of the rotation-minimising frame** (double reflection): the frame `f` (the tangent,
/// then two directions square to it) at the point `x0` carried to the point `x1` with tangent `t1`.
/// A step that does not move keeps the frame.
pub fn reflected(f: &[[f64; 3]; 3], x0: [f64; 3], x1: [f64; 3], t1: [f64; 3]) -> [[f64; 3]; 3] {
    let v1 = sub(x1, x0);
    let c1 = dot(v1, v1);
    if c1 == 0.0 {
        return *f;
    }
    let (rl, tl) = (reflect(f[1], v1, c1), reflect(f[0], v1, c1));
    let v2 = sub(t1, tl);
    let c2 = dot(v2, v2);
    let u = if c2 == 0.0 { rl } else { reflect(rl, v2, c2) };
    // held square to the tangent and unit, the rounding of every step not carried on
    let u = normalised(sub(u, scale(t1, dot(u, t1)))).unwrap_or(u);
    [t1, u, cross(t1, u)]
}

/// `advance` is the length travelled along the unit axis per full turn of the
/// shared parameter, so a rotation with a nonzero advance is a screw.  A joint **driven** by a law
/// (#391 S5) stands at its law's value of the roll where an undriven one stands at the roll
/// itself: turned `ratio · law(θ)`, risen `advance · law(θ) / τ`.
#[derive(Clone,Debug)]
pub enum Step {
    Rotation {origin:[f64;3],axis:[f64;3],ratio:f64,advance:f64,law:Option<Arc<dyn Drive>>},
    Translation {axis:[f64;3],advance:f64,law:Option<Arc<dyn Drive>>},
    /// `observer⁻¹ · source`, each the index of an earlier step.
    Relative {source:usize,observer:usize},
    /// A frame along a path (#398 C), its pose at the path's parameter, or that pose undone.
    Guided {guide:Arc<dyn Carrier>,back:bool},
    /// A frame the drawing moves (#398 F), carried from where it stands to where the drawing
    /// followed along its parameter puts it, or that carriage undone.
    Followed {ride:Arc<dyn Carrier>,back:bool},
}

/// The joint's own number at the roll `angle` and its rate a radian of the roll: the law's where
/// it is driven, the roll itself otherwise.
fn driven(law: &Option<Arc<dyn Drive>>,angle: f64) -> Result<(f64,f64),String> {
    match law {
        None => Ok((angle,1.)),
        Some(l) => l.jet(angle).map(|[v,r]| (v,r)),
    }
}

/// `driven`'s value alone, for a reading that needs no rate.
fn driven_value(law: &Option<Arc<dyn Drive>>,angle: f64) -> Result<f64,String> {
    match law { None => Ok(angle), Some(l) => l.value(angle) }
}

impl Step {
    /// The step that undoes this one at every time: a turn, screw or slide run backwards, and the
    /// relative `observer⁻¹·source` undone as `source⁻¹·observer`.
    pub fn inverse(&self) -> Step {
        match *self {
            Step::Rotation {origin,axis,ratio,advance,ref law} =>
                Step::Rotation {origin,axis,ratio:-ratio,advance:-advance,law:law.clone()},
            Step::Translation {axis,advance,ref law} =>
                Step::Translation {axis,advance:-advance,law:law.clone()},
            Step::Relative {source,observer} => Step::Relative {source:observer,observer:source},
            Step::Guided {ref guide,back} => Step::Guided {guide:guide.clone(),back:!back},
            Step::Followed {ref ride,back} => Step::Followed {ride:ride.clone(),back:!back},
        }
    }

    /// A frame along a path's or a frame the drawing moves' stretch `[lo, hi]` of the roll,
    /// enclosed forward (#398 C, F): `None` for any other step, and where it has no bound.
    fn stretch(&self,lo: f64,hi: f64) -> Option<Stretch> {
        match self {
            Step::Guided {guide,..} => guide.stretch(lo,hi),
            Step::Followed {ride,..} => ride.stretch(lo,hi),
            _ => None,
        }
    }

    /// Whether this step is a joint driven by a law: no constant twist, whatever its numbers.
    fn is_driven(&self) -> bool {
        matches!(self,Step::Rotation {law:Some(_),..} | Step::Translation {law:Some(_),..})
    }

    /// Whether this step carries the plane with unit `normal` within itself: a turn about an axis
    /// square to it that does not screw along it, or a slide along it. A relative is its steps'.
    fn keeps(&self,normal: [f64;3]) -> bool {
        use crate::util::space::{cross,dot,norm};
        match *self {
            Step::Rotation {axis,advance,..} => advance == 0. && unit(axis).is_some_and(|a| norm(cross(a,normal)) <= 1e-9),
            Step::Translation {axis,..} => unit(axis).is_some_and(|a| dot(a,normal).abs() <= 1e-9),
            Step::Relative {..} => true,
            Step::Guided {..} | Step::Followed {..} => false,
        }
    }
}

fn unit(axis: [f64;3]) -> Option<[f64;3]> {
    // sqrt of the sum and not `hypot`: evaluated at every pose, and axes are of drawing size
    let n = crate::util::space::norm(axis);
    (n > 0. && n.is_finite()).then(|| axis.map(|v| v/n))
}

/// How a motion carries a plane it leaves where it is: a turn about an axis square to it, or a
/// slide along it. Rates are per radian of the shared parameter, from whatever pose is read.
#[derive(Clone,Copy,Debug,PartialEq)]
pub enum PlaneRigid {
    /// About the axis through `centre` along `axis` (a unit vector), `rate` radians per radian.
    Turn { centre: [f64;3], axis: [f64;3], rate: f64 },
    /// By `velocity` per radian, which lies in the plane.
    Slide { velocity: [f64;3] },
}

/// One step of a motion read in a plane it carries within itself (`Family::in_frame`): a turn
/// about a point at a ratio, a slide along a unit direction by an advance a
/// turn, and the relative of the two steps before it.
#[derive(Clone,Copy,Debug,PartialEq)]
pub enum PlanarStep {
    Turn { centre: [f64;2], ratio: f64 },
    Slide { direction: [f64;2], advance: f64 },
    Relative,
}

/// A screw read off a motion (`Family::twist`): it turns `ratio` radians a radian about the line
/// through `origin` along the unit `axis`, and slides `advance` along it a turn of the parameter.
/// A turn (`advance` zero) and a slide (`ratio` zero) are its two limits.
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
    pub fn extent(&self,lo: [f64;3],hi: [f64;3],back: crate::sweep::envelope::Motion) -> ([f64;2],f64) {
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

/// A motion's instantaneous screw axis (`Family::instantaneous`, #326): the line through `point`
/// (its nearest the origin) along the unit `direction`, along which every point moves by `pitch` a
/// radian; and the twist it is read from, `v(x) = v0 + omega × x`.
#[derive(Clone,Copy,Debug,PartialEq)]
pub struct Isa { pub point: [f64;3], pub direction: [f64;3], pub pitch: f64, pub omega: [f64;3], pub v0: [f64;3] }

impl Isa {
    /// The velocity per radian of the point at `x`, in the fixed frame.
    pub fn velocity(&self,x: [f64;3]) -> [f64;3] {
        crate::util::space::add(self.v0,crate::util::space::cross(self.omega,x))
    }
}

/// A snapshot of a named rigid-motion graph and its solved world axes. Re-read after
/// changing the sketch. Sampling a snapshot never reads drawing coordinates or solves it.
#[derive(Clone,Debug)]
pub struct Family {
    pub name: String,
    steps: Vec<Step>,
}

impl Family {
    /// The family of `steps`, the last its root, each relative naming earlier steps; refused where
    /// relatives nest more than 64 deep.
    pub fn new(name: String,steps: Vec<Step>) -> Result<Self,String> {
        let mut heights: Vec<usize> = Vec::with_capacity(steps.len());
        for step in &steps {
            let height = match *step {
                Step::Rotation {..} | Step::Translation {..} | Step::Guided {..}
                    | Step::Followed {..} => 1,
                Step::Relative {source,observer} => 1+heights[source].max(heights[observer]),
            };
            if height > 64 { return Err("motion dependencies exceed 64 levels".into()); }
            heights.push(height);
        }
        Ok(Self {name,steps})
    }

    /// The family read off `source`'s motion `index`, as it now stands.
    pub fn read<S: Source+?Sized>(source: &S,index: usize) -> Result<Self,String> { source.family(index) }

    /// The steps, the last the root.
    pub fn steps(&self) -> &[Step] { &self.steps }

    /// The step the motion is: the last, every other one read by it.  A joint at the root is the
    /// whole motion, whatever steps an inverse left before it.
    fn root(&self) -> &Step { self.steps.last().expect("a motion family has a root") }

    /// Upper bound on the speed of inverse(M(t))*point per radian, for every t in
    /// `domain`. This bounds the mathematical rigid family represented by the
    /// solved axes; it does not bound roundoff in `at` or error in the solved
    /// source geometry. A translation's reach grows with the domain, which is
    /// why the domain is an input.
    pub fn inverse_point_speed_bound(&self,point: [f64;3],domain: crate::util::interval::Interval)
        -> Result<f64,crate::util::interval::Error> {
        use crate::util::interval::Interval as I;
        let point = [I::point(point[0])?,I::point(point[1])?,I::point(point[2])?];
        self.inverse_point_speed_bound_over(point,domain)
    }

    /// The same bound over a whole point box, per step: a rotation or screw
    /// moves a point at `|ratio| · distance(point, axis) + |advance|/τ`, the
    /// same forward and inverse; a relative motion `observer⁻¹ ∘ source` moves
    /// it at most as fast as the source moves it plus as fast as the observer
    /// moves the source's image, and its inverse the other way round, the
    /// images enclosed by each step's interval pose over the whole domain. A
    /// point on an axis does not move and resolves at once; the earlier bound
    /// from the point's radius about the world origin left such a point
    /// refining the whole roll, and read hundreds of millimetres per radian
    /// on a gear cutter whose contact moves at tens.
    pub fn inverse_point_speed_bound_over(&self,point: [crate::util::interval::Interval;3],domain: crate::util::interval::Interval)
        -> Result<f64,crate::util::interval::Error> {
        use crate::util::interval::{Error,Interval as I};
        let tau = I::point(std::f64::consts::TAU)?;
        let reach = { let [lo,hi] = domain.bounds(); I::point(lo.abs().max(hi.abs()))? };
        // the box's foot on a step's axis and its farthest distance from it
        let about = |origin: [f64;3],axis: [f64;3],b: [I;3]| -> Result<([I;3],I),Error> {
            let a = unit(axis).ok_or(Error::InvalidBounds)?;
            let d: [I;3] = [b[0].sub(I::point(origin[0])?)?,b[1].sub(I::point(origin[1])?)?,b[2].sub(I::point(origin[2])?)?];
            let along = d[0].mul(I::point(a[0])?)?.add(d[1].mul(I::point(a[1])?)?)?.add(d[2].mul(I::point(a[2])?)?)?;
            let square = d[0].square()?.add(d[1].square()?)?.add(d[2].square()?)?.sub(along.square()?)?;
            let radius = I::new(square.bounds()[0].max(0.),square.bounds()[1].max(0.))?.sqrt()?;
            let foot: [I;3] = [I::point(origin[0])?.add(along.mul(I::point(a[0])?)?)?,I::point(origin[1])?.add(along.mul(I::point(a[1])?)?)?,
                I::point(origin[2])?.add(along.mul(I::point(a[2])?)?)?];
            Ok((foot,radius))
        };
        // how far each step's own number reaches over the domain and how fast it runs a radian
        // of it, once for every box: the roll's own reach at rate 1, or a drive's law enclosed
        // over the domain (#391 S5), so a cam's throw and its speed bound what it carries
        let magnitude = |x: I| { let [lo,hi] = x.bounds(); I::point(lo.abs().max(hi.abs())) };
        let numbers = self.steps.iter().map(|step| match step {
            Step::Rotation {law:Some(l),..} | Step::Translation {law:Some(l),..} => {
                let [v,r] = l.enclose(domain)?;
                Ok((magnitude(v)?,magnitude(r)?))
            }
            _ => Ok((reach,I::ONE)),
        }).collect::<Result<Vec<(I,I)>,Error>>()?;
        // a frame along a path over the domain of its parameter, enclosed (#398 C): its path's
        // point, its speed and its frame's angular speed, or no bound at all
        let (lo,hi) = { let [a,b] = domain.bounds(); (a,b) };
        let guided = self.steps.iter().map(|step| match step {
            Step::Guided {..} | Step::Followed {..} =>
                step.stretch(lo,hi).ok_or(Error::InvalidBounds).map(Some),
            _ => Ok(None),
        }).collect::<Result<Vec<Option<Stretch>>,Error>>()?;
        // the farthest a box stands from the path over the stretch, or from the home
        let far = |g: &Stretch,b: [I;3]| -> Result<I,Error> {
            let home = [I::point(g.origin[0])?,I::point(g.origin[1])?,I::point(g.origin[2])?];
            let reach = |c: [I;3]| -> Result<f64,Error> {
                let d: [I;3] = [b[0].sub(c[0])?,b[1].sub(c[1])?,b[2].sub(c[2])?];
                let s = d.iter().map(|x| { let [l,h] = x.bounds(); l.abs().max(h.abs()) })
                    .map(|m| m*m).sum::<f64>();
                Ok(s.sqrt()*(1.+1e-12))
            };
            I::point(reach(g.point)?.max(reach(home)?))
        };
        // a step's own speed on a box, the same forward and inverse
        let own = |i: usize,b: [I;3]| -> Result<I,Error> {
            let (_,rate) = numbers[i];
            match self.steps[i] {
                Step::Rotation {origin,axis,ratio,advance,..} => {
                    let (_,radius) = about(origin,axis,b)?;
                    I::point(ratio.abs())?.mul(radius)?.add(I::point(advance.abs())?.div(tau)?)?
                        .mul(rate)
                }
                Step::Translation {advance,..} => I::point(advance.abs())?.div(tau)?.mul(rate),
                // a point carried along a path moves as fast as the path runs, and turns about
                // it at the frame's angular speed, the same forward and inverse
                Step::Guided {..} => {
                    let g = guided[i].as_ref().ok_or(Error::InvalidBounds)?;
                    I::point(g.spin)?.mul(far(g,b)?)?.add(I::point(g.speed)?)
                }
                // a frame the drawing moves, span by span (#398 F)
                Step::Followed {ref ride,..} =>
                    I::point(ride.speed_over(b,lo,hi).ok_or(Error::InvalidBounds)?),
                Step::Relative {..} => unreachable!("composed below"),
            }
        };
        // where a step carries a box over the whole domain, forward or
        // inverse: within the box's farthest distance of its foot on the
        // axis, and the advance's reach along it; no angle enters, so no
        // interval trigonometry limits the domain
        let image = |i: usize,b: [I;3]| -> Result<[I;3],Error> {
            let (reach,_) = numbers[i];
            match self.steps[i] {
                Step::Rotation {origin,axis,advance,..} => {
                    let a = unit(axis).ok_or(Error::InvalidBounds)?;
                    let (foot,radius) = about(origin,axis,b)?;
                    let slide = I::point(advance.abs())?.div(tau)?.mul(reach)?;
                    let spread = radius.add(slide)?;
                    let _ = a;
                    Ok([foot[0].add(I::new(-spread.bounds()[1],spread.bounds()[1])?)?,foot[1].add(I::new(-spread.bounds()[1],spread.bounds()[1])?)?,
                        foot[2].add(I::new(-spread.bounds()[1],spread.bounds()[1])?)?])
                }
                Step::Translation {axis,advance,..} => {
                    let a = unit(axis).ok_or(Error::InvalidBounds)?;
                    let slide = I::point(advance.abs())?.div(tau)?.mul(reach)?.bounds()[1];
                    Ok([b[0].add(I::new(-slide*a[0].abs(),slide*a[0].abs())?)?,b[1].add(I::new(-slide*a[1].abs(),slide*a[1].abs())?)?,
                        b[2].add(I::new(-slide*a[2].abs(),slide*a[2].abs())?)?])
                }
                // carried along a path, a box stands within its farthest reach of the path's
                // stretch, forward, and of the home, inverse
                Step::Guided {..} | Step::Followed {..} => {
                    let g = guided[i].as_ref().ok_or(Error::InvalidBounds)?;
                    let r = far(g,b)?.bounds()[1];
                    let around = |k: usize| {
                        let [lo,hi] = g.point[k].bounds();
                        I::new(lo.min(g.origin[k])-r,hi.max(g.origin[k])+r)
                    };
                    Ok([around(0)?,around(1)?,around(2)?])
                }
                Step::Relative {..} => unreachable!("composed below"),
            }
        };
        fn image_of(family: &Family,image: &dyn Fn(usize,[I;3]) -> Result<[I;3],Error>,i: usize,b: [I;3],forward: bool) -> Result<[I;3],Error> {
            match family.steps[i] {
                Step::Relative {source,observer} => {
                    // forward: x ↦ O⁻¹(S x); inverse: x ↦ S⁻¹(O x)
                    let (first,second) = if forward { (source,observer) } else { (observer,source) };
                    let moved = image_of(family,image,first,b,true)?;
                    image_of(family,image,second,moved,false)
                }
                _ => image(i,b),
            }
        }
        fn speed(family: &Family,own: &dyn Fn(usize,[I;3]) -> Result<I,Error>,image: &dyn Fn(usize,[I;3]) -> Result<[I;3],Error>,
            i: usize,b: [I;3],forward: bool) -> Result<I,Error> {
            match family.steps[i] {
                Step::Relative {source,observer} => {
                    let (first,second) = if forward { (source,observer) } else { (observer,source) };
                    let moved = speed(family,own,image,first,b,true)?;
                    let carried = image_of(family,image,first,b,true)?;
                    moved.add(speed(family,own,image,second,carried,false)?)
                }
                _ => own(i,b),
            }
        }
        let root = self.steps.len()-1;
        Ok(speed(self,&own,&image,root,point,false)?.bounds()[1])
    }

    /// The screw this motion is, when it is one: a single rotation that slides along its axis as
    /// it turns. Every point's velocity in the moving frame is then the same at every time — the
    /// twist is constant — which is what `characteristic` builds a swept boundary from.
    /// A rotation without advance, a translation and a relative motion are none.
    pub fn screw(&self) -> Option<Screw> {
        match *self.root() {
            Step::Rotation {origin,axis,ratio,advance,law:None} if advance != 0. && ratio != 0. =>
                Some(Screw {origin,axis:unit(axis)?,ratio,advance}),
            _ => None,
        }
    }

    /// The **constant twist** this motion is (#684): a screw (`screw`), a turn about one axis with
    /// no advance, or a slide along one line turning not at all, each at a constant rate. Every
    /// point it carries then moves the same way relative to what carries it at every time, which
    /// is what `characteristic` builds a swept boundary from. A slide is read as a screw
    /// turning at no ratio about the line along its direction through the world's origin; a
    /// relative motion, a driven one and one along a path are none.
    pub fn twist(&self) -> Option<Screw> {
        self.turning().or_else(|| self.sliding().filter(|&(_,advance)| advance != 0.)
            .map(|(axis,advance)| Screw {origin:[0.;3],axis,ratio:0.,advance}))
    }

    /// The slide this motion is, where it is one translation: its unit direction and its advance a
    /// turn of the parameter.
    pub fn sliding(&self) -> Option<([f64;3],f64)> {
        match *self.root() {
            Step::Translation {axis,advance,law:None} => Some((unit(axis)?,advance)),
            _ => None,
        }
    }

    /// The turn this motion is, where it is one rotation about one axis: a screw (`screw`), or a
    /// rotation with no advance (`advance` nothing) — what a ball under it sweeps is the tube about
    /// a helix or an arc of a circle (a ball's tube, #238 in the solver).
    pub fn turning(&self) -> Option<Screw> {
        match *self.root() {
            Step::Rotation {origin,axis,ratio,advance,law:None} if ratio != 0. => Some(Screw {origin,axis:unit(axis)?,ratio,advance}),
            _ => None,
        }
    }

    /// The **instantaneous screw** of this motion (#326): the axis every point moves along at the
    /// drawn pose, read where the twist is the same at every time in the fixed frame — a turn, a
    /// screw or a slide about a fixed line, or one such seen from another (`relative_to:`), whose
    /// twist is the source's less the observer's.  A slide alone, or two turns that cancel, turn
    /// nothing and have none; a relative of a relative is not constant and is refused, saying so.
    pub fn instantaneous(&self) -> Result<Isa,String> {
        let one = |s: &Step| -> Result<([f64;3],[f64;3]),String> {
            let tau = std::f64::consts::TAU;
            if s.is_driven() {
                return Err(format!("`{}` is driven by a law, whose screw moves as it runs: a \
                    screw is read of turns, screws and slides at a constant rate",self.name));
            }
            match *s {
                Step::Rotation {origin,axis,ratio,advance,..} => {
                    let d = unit(axis).ok_or("a turn's axis has no direction")?;
                    let w = d.map(|x| x*ratio);
                    let v = crate::util::space::cross(origin,w);
                    Ok((w,std::array::from_fn(|k| v[k]+d[k]*advance/tau)))
                }
                Step::Translation {axis,advance,..} => {
                    let d = unit(axis).ok_or("a slide's line has no direction")?;
                    Ok(([0.;3],d.map(|x| x*advance/tau)))
                }
                Step::Guided {..} => Err(format!("`{}` rides along a path, whose screw moves as it \
                    runs: a screw is read of turns, screws and slides at a constant rate",
                    self.name)),
                Step::Followed {..} => Err(format!("`{}` stands where the drawing puts it as its \
                    parameter runs, whose screw moves as it runs: a screw is read of turns, screws \
                    and slides at a constant rate",self.name)),
                Step::Relative {..} => Err(format!("`{}` is a motion relative to a relative one, whose \
                    screw moves as it runs: a screw is read of turns, screws and slides about fixed \
                    lines, and of one relative to another",self.name)),
            }
        };
        let (w,v0) = match self.steps.last().ok_or("a motion with no step")? {
            Step::Relative {source,observer} => {
                let (s,o) = (one(&self.steps[*source])?,one(&self.steps[*observer])?);
                (crate::util::space::sub(s.0,o.0),crate::util::space::sub(s.1,o.1))
            }
            s => one(s)?,
        };
        let ww = crate::util::space::dot(w,w);
        if !(ww > 0.) {
            return Err(format!("`{}` turns nothing, so its screw has no axis",self.name));
        }
        let point = crate::util::space::cross(w,v0).map(|x| x/ww);
        Ok(Isa {point,direction:unit(w).expect("a turn"),pitch:crate::util::space::dot(w,v0)/ww,omega:w,v0})
    }

    /// How this motion carries the plane through `point` with unit `normal`, when it carries it
    /// within itself: a rotation whose axis is square to the plane and does not screw along it,
    /// or a translation along it. Every other motion (a screw, a relative motion, a graph of
    /// several steps) leaves the plane, and is none: a plane a motion does not preserve has no
    /// such reading.
    pub fn in_plane(&self,normal: [f64;3],point: [f64;3]) -> Option<PlaneRigid> {
        use crate::util::space::dot;
        let normal = unit(normal)?;
        if self.root().is_driven() { return None; }
        match *self.root() {
            ref step @ Step::Rotation {origin,axis,ratio,..} => {
                let axis = unit(axis)?;
                if !step.keeps(normal) || ratio == 0. { return None; }
                // the plane turns about where its axis meets it
                let along = dot([point[0]-origin[0],point[1]-origin[1],point[2]-origin[2]],axis);
                Some(PlaneRigid::Turn {centre:std::array::from_fn(|k| origin[k]+along*axis[k]),axis,rate:ratio})
            }
            ref step @ Step::Translation {axis,advance,..} => {
                let axis = unit(axis)?;
                if !step.keeps(normal) || advance == 0. { return None; }
                let rate = advance/std::f64::consts::TAU;
                Some(PlaneRigid::Slide {velocity:axis.map(|x| x*rate)})
            }
            _ => None,
        }
    }

    /// This motion as a program in the plane of `frame`, when every step carries that plane within
    /// itself (`Step::keeps`): each turn about a point of it, each slide along it, in postfix order
    /// (a relative's source, its observer, then the relative), coordinates in the frame's and
    /// lengths times `scale` (a planar envelope's steps). None where any step leaves the plane.
    pub fn in_frame(&self,frame: &crate::geom::Frame,scale: f64) -> Option<Vec<PlanarStep>> {
        fn emit(family: &Family,i: usize,frame: &crate::geom::Frame,scale: f64,out: &mut Vec<PlanarStep>) -> Option<()> {
            let step = &family.steps[i];
            if !matches!(step,Step::Relative {..}) && !step.keeps(frame.z) || step.is_driven() { return None }
            match *step {
                Step::Rotation {origin,axis,ratio,..} => {
                    // a turn about the plane's normal reversed is the opposite turn about it
                    let sign = crate::util::space::dot(axis,frame.z).signum();
                    let l = frame.local(origin.map(|v| v*scale));
                    out.push(PlanarStep::Turn {centre:[l[0],l[1]],ratio:sign*ratio});
                }
                Step::Translation {axis,advance,..} => {
                    let d = frame.dir_local(axis);
                    let length = d[0].dhypot(d[1]);
                    out.push(PlanarStep::Slide {direction:[d[0]/length,d[1]/length],advance:advance*scale});
                }
                Step::Guided {..} | Step::Followed {..} => return None,
                Step::Relative {source,observer} => {
                    emit(family,source,frame,scale,out)?;
                    emit(family,observer,frame,scale,out)?;
                    out.push(PlanarStep::Relative);
                }
            }
            Some(())
        }
        let mut out = Vec::new();
        emit(self,self.steps.len()-1,frame,scale,&mut out)?;
        Some(out)
    }

    /// Exact pose and derivative per radian of the shared angle. Relative motion is
    /// observer^-1 * source, including the derivative of the moving inverse.
    pub fn at(&self, angle: f64) -> Result<Motion,String> {
        if !angle.is_finite() { return Err("a motion angle must be finite".into()); }
        // the steps' values on the stack for a family of up to eight steps, as `pose_at` keeps
        // them: an envelope's root search reads a pose and its rate many thousand times
        let mut fixed = [Motion::identity();8];
        let mut spilled;
        let values: &mut [Motion] = if self.steps.len() <= fixed.len() { &mut fixed[..self.steps.len()] }
            else { spilled = vec![Motion::identity();self.steps.len()]; &mut spilled };
        for (n,step) in self.steps.iter().enumerate() {
            let value = match *step {
                Step::Rotation {origin,axis,ratio,advance,ref law} => {
                    // a drive's value and its rate by the chain rule (#391 S5)
                    let (a,da) = driven(law,angle)?;
                    let rotation = Motion::rotation(axis,ratio*a,ratio*da)
                        .map_err(|_| "a motion angle overflowed")?;
                    let pose = Motion::translation(origin.map(|v| -v),[0.;3]).unwrap()
                        .then(rotation).then(Motion::translation(origin,[0.;3]).unwrap());
                    if advance == 0. { pose } else {
                        let direction = unit(axis).ok_or("a motion needs a nondegenerate axis")?;
                        let rate = advance/std::f64::consts::TAU;
                        pose.then(Motion::translation(direction.map(|v| v*rate*a),direction.map(|v| v*rate*da))
                            .map_err(|_| "a motion advance overflowed")?)
                    }
                }
                Step::Translation {axis,advance,ref law} => {
                    let (a,da) = driven(law,angle)?;
                    let direction = unit(axis).ok_or("a motion needs a nondegenerate axis")?;
                    let rate = advance/std::f64::consts::TAU;
                    Motion::translation(direction.map(|v| v*rate*a),direction.map(|v| v*rate*da))
                        .map_err(|_| "a motion advance overflowed")?
                }
                Step::Relative {source,observer} => values[source].then(values[observer].inverse()),
                // the path's parameter is what the motion is read at (#398 C)
                Step::Guided {ref guide,back} => {
                    let m = guide.motion(angle)?;
                    if back { m.inverse() } else { m }
                }
                // the roll is the drawing's parameter (#398 F)
                Step::Followed {ref ride,back} => {
                    let m = ride.motion(angle)?;
                    if back { m.inverse() } else { m }
                }
            };
            if !value.is_finite() { return Err("a motion pose or derivative overflowed".into()); }
            values[n] = value;
        }
        values.last().copied().ok_or_else(|| "a motion family has a root".into())
    }
}

/// A rigid pose without its derivative: `x ↦ r·x + p`.
#[derive(Clone,Copy,Debug)]
pub struct Pose { pub r: [[f64;3];3], pub p: [f64;3] }

impl Pose {
    /// A gradient read at `self.point(x)` turned into the gradient at `x`: the rotation's
    /// transpose applied.
    pub fn gradient(&self,g: [f64;3]) -> [f64;3] {
        std::array::from_fn(|i| self.r[0][i]*g[0]+self.r[1][i]*g[1]+self.r[2][i]*g[2])
    }
    pub fn point(&self,x: [f64;3]) -> [f64;3] {
        std::array::from_fn(|i| self.r[i][0]*x[0]+self.r[i][1]*x[1]+self.r[i][2]*x[2]+self.p[i])
    }
    /// A direction turned by the pose.
    pub fn vector(&self,v: [f64;3]) -> [f64;3] {
        std::array::from_fn(|i| self.r[i][0]*v[0]+self.r[i][1]*v[1]+self.r[i][2]*v[2])
    }
    /// Apply this pose, then `next`.
    pub fn then(self,next: Pose) -> Pose {
        Pose {r:std::array::from_fn(|i| std::array::from_fn(|j| next.r[i][0]*self.r[0][j]+next.r[i][1]*self.r[1][j]+next.r[i][2]*self.r[2][j])),
            p:next.point(self.p)}
    }
    pub fn inverse(self) -> Pose {
        let r: [[f64;3];3] = std::array::from_fn(|i| std::array::from_fn(|j| self.r[j][i]));
        let p = std::array::from_fn(|i| -(r[i][0]*self.p[0]+r[i][1]*self.p[1]+r[i][2]*self.p[2]));
        Pose {r,p}
    }
}

impl Family {
    /// The pose alone at `angle`, as `at` gives it without the derivative and without allocating
    /// for a family of up to eight steps: for a caller evaluating many poses that needs no
    /// velocity (a floating-point field reading).
    pub fn pose_at(&self,angle: f64) -> Result<Pose,String> {
        if !angle.is_finite() { return Err("a motion angle must be finite".into()); }
        let identity = Pose {r:[[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]],p:[0.;3]};
        let mut fixed = [identity;8];
        let mut spilled: Vec<Pose> = Vec::new();
        let many = self.steps.len() > fixed.len();
        for (n,step) in self.steps.iter().enumerate() {
            let get = |k: usize,fixed: &[Pose;8],spilled: &Vec<Pose>| if many { spilled[k] } else { fixed[k] };
            let value = match *step {
                Step::Rotation {origin,axis,ratio,advance,ref law} => {
                    let angle = driven_value(law,angle)?;
                    let a = unit(axis).ok_or("a motion needs a nondegenerate axis")?;
                    let (s,c) = (ratio*angle).dsin_cos();
                    let k = [[0.,-a[2],a[1]],[a[2],0.,-a[0]],[-a[1],a[0],0.]];
                    let r: [[f64;3];3] = std::array::from_fn(|i| std::array::from_fn(|j|
                        c*if i == j { 1. } else { 0. }+(1.-c)*a[i]*a[j]+s*k[i][j]));
                    let turned = Pose {r,p:[0.;3]}.point(origin);
                    let mut p: [f64;3] = std::array::from_fn(|i| origin[i]-turned[i]);
                    if advance != 0. {
                        let rate = advance/std::f64::consts::TAU;
                        for i in 0..3 { p[i] += a[i]*rate*angle; }
                    }
                    Pose {r,p}
                }
                Step::Translation {axis,advance,ref law} => {
                    let angle = driven_value(law,angle)?;
                    let a = unit(axis).ok_or("a motion needs a nondegenerate axis")?;
                    let rate = advance/std::f64::consts::TAU;
                    Pose {p:a.map(|v| v*rate*angle),..identity}
                }
                Step::Relative {source,observer} => get(source,&fixed,&spilled).then(get(observer,&fixed,&spilled).inverse()),
                Step::Guided {ref guide,back} => {
                    let p = guide.pose(angle)?;
                    if back { p.inverse() } else { p }
                }
                Step::Followed {ref ride,back} => {
                    let p = ride.pose(angle)?;
                    if back { p.inverse() } else { p }
                }
            };
            if many { spilled.push(value) } else { fixed[n] = value }
        }
        let last = self.steps.len().checked_sub(1).ok_or("a motion family has a root")?;
        let pose = if many { spilled[last] } else { fixed[last] };
        if !pose.r.iter().flatten().chain(&pose.p).all(|v| v.is_finite()) { return Err("a motion pose overflowed".into()); }
        Ok(pose)
    }
}

/// **Where a caller's motions are read from**: its document, as it now stands.  What
/// `Family::read` asks.
pub trait Source {
    /// The family of motion `index`.
    fn family(&self,index: usize) -> Result<Family,String>;
}
