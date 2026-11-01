//! Named rigid-motion families. Every node reads the same angular parameter in radians.
#[allow(unused_imports)]
use crate::fmath::Det;
mod bounds;
pub use bounds::MotionBounds;
mod contact;
pub use contact::{NormalVelocity,ContactTime};
mod law;
pub use law::{Law, unread_drive};
mod guide;
use guide::{Guide, Stretch};
pub(crate) use guide::reflected;
mod track;
pub use track::Track;
pub(crate) use track::{window, Ride};

use crate::{envelope::Motion,model::{MotionDef,Sketch}};
pub use crate::brep::trajectory::Trajectory;

/// `advance` is the length travelled along the unit axis per full turn of the
/// shared parameter, so a rotation with a nonzero advance is a screw.  A joint **driven** by a law
/// (#391 S5) stands at its law's value of the roll where an undriven one stands at the roll
/// itself: turned `ratio · law(θ)`, risen `advance · law(θ) / τ`.
#[derive(Clone,Debug)]
enum Step {
    Rotation {origin:[f64;3],axis:[f64;3],ratio:f64,advance:f64,law:Option<Law>},
    Translation {axis:[f64;3],advance:f64,law:Option<Law>},
    Relative {source:usize,observer:usize},
    /// A frame along a path (#398 C), its pose at the path's parameter, or that pose undone.
    Guided {guide:std::sync::Arc<Guide>,back:bool},
    /// A frame the drawing moves (#398 F), carried from where it stands to where the drawing
    /// followed along its parameter puts it, or that carriage undone.
    Followed {ride:std::sync::Arc<Ride>,back:bool},
}

/// The joint's own number at the roll `angle`: the law's where it is driven, the roll itself
/// otherwise (its rate is the lowered trajectory's, `Trajectory::Paced`).
fn driven_value(law: &Option<Law>,angle: f64) -> Result<f64,String> {
    match law { None => Ok(angle), Some(l) => l.value(angle) }
}

impl Step {
    /// The step that undoes this one at every time: a turn, screw or slide run backwards, and the
    /// relative `observer⁻¹·source` undone as `source⁻¹·observer`.
    fn inverse(&self) -> Step {
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
        use crate::space::{cross,dot,norm};
        match *self {
            Step::Rotation {axis,advance,..} => advance == 0. && unit(axis).is_some_and(|a| norm(cross(a,normal)) <= 1e-9),
            Step::Translation {axis,..} => unit(axis).is_some_and(|a| dot(a,normal).abs() <= 1e-9),
            Step::Relative {..} => true,
            Step::Guided {..} | Step::Followed {..} => false,
        }
    }
}

/// A law as the kernel's pace (`Trajectory::Paced`): its value and rate, and its range over a roll.
#[derive(Debug)]
struct Paced(Law);

impl crate::brep::trajectory::Pace for Paced {
    fn jet(&self,t: f64) -> Result<[f64;2],crate::brep::KernelError> { Ok(self.0.jet(t)?) }
    fn range(&self,roll: [f64;2]) -> Result<[f64;2],crate::brep::KernelError> { Ok(self.0.range(roll)?) }
}

/// A frame along a path or one the drawing moves as the kernel's evaluated path
/// (`Trajectory::General`): its pose and rate at each time, undone where it is `back`, as
/// `Family::at` reads the step.
#[derive(Debug)]
struct Evaluated { carrier: Carrier,back: bool }

#[derive(Debug)]
enum Carrier { Guide(std::sync::Arc<Guide>),Ride(std::sync::Arc<Ride>) }

impl crate::brep::trajectory::Path for Evaluated {
    fn at(&self,t: f64) -> Result<Motion,crate::brep::KernelError> {
        let m = match &self.carrier { Carrier::Guide(g) => g.motion(t)?, Carrier::Ride(r) => r.motion(t)? };
        Ok(if self.back { m.inverse() } else { m })
    }
}

fn unit(axis: [f64;3]) -> Option<[f64;3]> {
    // sqrt of the sum and not `hypot`: evaluated at every pose, and axes are of drawing size
    let n = crate::space::norm(axis);
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

/// A screw read off a motion (`Family::twist`), the kernel's constant twist (#721 F2): it turns
/// `ratio` radians a radian about the line through `origin` along the unit `axis`, and slides
/// `advance` along it a turn of the parameter.
pub use crate::brep::trajectory::Screw;

/// A motion's instantaneous screw axis (`Family::instantaneous`, #326): the line through `point`
/// (its nearest the origin) along the unit `direction`, along which every point moves by `pitch` a
/// radian; and the twist it is read from, `v(x) = v0 + omega × x`.
#[derive(Clone,Copy,Debug,PartialEq)]
pub struct Isa { pub point: [f64;3], pub direction: [f64;3], pub pitch: f64, pub omega: [f64;3], pub v0: [f64;3] }

impl Isa {
    /// The velocity per radian of the point at `x`, in the fixed frame.
    pub fn velocity(&self,x: [f64;3]) -> [f64;3] {
        crate::space::add(self.v0,crate::space::cross(self.omega,x))
    }
}

/// A snapshot of a named rigid-motion graph and its solved world axes. Re-read after
/// changing the sketch. Sampling a snapshot never reads drawing coordinates or solves it.
#[derive(Clone,Debug)]
pub struct Family {
    pub name: String,
    steps: Vec<Step>,
    /// The motion lowered to the kernel's trajectory (`trajectory`), once: what its pose, its rate,
    /// its constant twist and its instantaneous screw are read off, or why it could not be.
    lowered: Result<Trajectory,String>,
}

impl Family {
    /// The step the motion is: the last, every other one read by it.  A joint at the root is the
    /// whole motion, whatever steps an inverse left before it.
    fn root(&self) -> &Step { self.steps.last().expect("a motion family has a root") }

    /// Upper bound on the speed of inverse(M(t))*point per radian, for every t in
    /// `domain`. This bounds the mathematical rigid family represented by the
    /// solved axes; it does not bound roundoff in `at` or error in the solved
    /// source geometry. A translation's reach grows with the domain, which is
    /// why the domain is an input.
    pub fn inverse_point_speed_bound(&self,point: [f64;3],domain: crate::interval::Interval)
        -> Result<f64,crate::interval::Error> {
        use crate::interval::Interval as I;
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
    pub fn inverse_point_speed_bound_over(&self,point: [crate::interval::Interval;3],domain: crate::interval::Interval)
        -> Result<f64,crate::interval::Error> {
        use crate::interval::{Error,Interval as I};
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
    /// twist is constant — which is what `solid::constant_twist` builds a swept boundary from.
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
    /// is what `solid::constant_twist` builds a swept boundary from. It is the lowered trajectory's
    /// (`Trajectory::screw`, #721 F2b): a slide is read as a screw turning at no ratio about the line
    /// along its direction through the world's origin, and a turn of no ratio as the slide it is; a
    /// relative motion, a driven one, one along a path and one that moves nothing are none.
    pub fn twist(&self) -> Option<Screw> {
        // (one that moves nothing, a turn of no ratio and no advance, has no characteristic)
        self.lowered.as_ref().ok()?.screw().filter(|s| s.ratio != 0. || s.advance != 0.)
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
    /// a helix or an arc of a circle (`solid::canal`, #238).
    pub fn turning(&self) -> Option<Screw> {
        match *self.root() {
            Step::Rotation {origin,axis,ratio,advance,law:None} if ratio != 0. => Some(Screw {origin,axis:unit(axis)?,ratio,advance}),
            _ => None,
        }
    }

    /// **The motion lowered to the kernel's trajectory** (#721 F2), its structure kept: a turn, a
    /// screw or a slide as the constant twist it is (`Trajectory::Twist`), a joint driven by a law as
    /// that joint taken at the law's pace (`Paced`), one motion seen from another as the two
    /// (`Relative`), and a frame along a path or one the drawing moves as what it evaluates to at
    /// each time (`General`). Its pose at every time is `at`'s; where `twist` reads a constant twist
    /// it is that one (`a_motion_lowers_to_the_kernels_trajectory`).
    pub fn trajectory(&self) -> Result<Trajectory,String> { self.path().cloned() }

    /// The lowered trajectory (`trajectory`), as it is held.
    pub fn path(&self) -> Result<&Trajectory,String> { self.lowered.as_ref().map_err(Clone::clone) }

    fn lowered(&self,k: usize) -> Result<Trajectory,String> {
        let paced = |path: Trajectory,law: &Option<Law>| match law {
            None => path,
            Some(l) => Trajectory::Paced {path:Box::new(path),pace:std::sync::Arc::new(Paced(l.clone()))},
        };
        // a slide is a screw about the line through the origin along it, turning not at all
        let (origin,axis,ratio,advance,law) = match self.steps[k] {
            Step::Rotation {origin,axis,ratio,advance,ref law} => (origin,axis,ratio,advance,law),
            Step::Translation {axis,advance,ref law} => ([0.;3],axis,0.,advance,law),
            Step::Relative {source,observer} => return Ok(Trajectory::Relative {source:Box::new(self.lowered(source)?),
                observer:Box::new(self.lowered(observer)?)}),
            Step::Guided {ref guide,back} => return Ok(Trajectory::General(std::sync::Arc::new(Evaluated {carrier:Carrier::Guide(guide.clone()),back}))),
            Step::Followed {ref ride,back} => return Ok(Trajectory::General(std::sync::Arc::new(Evaluated {carrier:Carrier::Ride(ride.clone()),back}))),
        };
        let axis = unit(axis).ok_or("a motion needs a nondegenerate axis")?;
        Ok(paced(Trajectory::Twist(Screw {origin,axis,ratio,advance}),law))
    }

    /// The **instantaneous screw** of this motion (#326): the axis every point moves along at the
    /// drawn pose, read where the twist is the same at every time in the fixed frame — a turn, a
    /// screw or a slide about a fixed line, or one such seen from another (`relative_to:`), whose
    /// twist is the source's less the observer's.  A slide alone, or two turns that cancel, turn
    /// nothing and have none; a relative of a relative is not constant and is refused, saying so.
    /// The twists are the lowered trajectory's (`Screw::twist`, #721 F2b).
    pub fn instantaneous(&self) -> Result<Isa,String> {
        use crate::brep::trajectory::Twist;
        let root = self.steps.last().ok_or("a motion with no step")?;
        // a step's twist where it is the same at every time in the fixed frame: a turn, a screw or a
        // slide about a fixed line, at a constant rate
        let constant = |t: &Trajectory| match t { Trajectory::Twist(s) => Some(s.twist()),_ => None };
        let Twist {omega:w,v0} = match self.path()? {
            Trajectory::Relative {source,observer} => match (constant(source),constant(observer)) {
                (Some(s),Some(o)) => Twist {omega:crate::space::sub(s.omega,o.omega),v0:crate::space::sub(s.v0,o.v0)},
                (s,_) => {
                    let Step::Relative {source,observer} = *root else { unreachable!("a relative lowers from a relative") };
                    return Err(self.unsteady(&self.steps[if s.is_none() { source } else { observer }]))
                }
            },
            t => constant(t).ok_or_else(|| self.unsteady(root))?,
        };
        let ww = crate::space::dot(w,w);
        if !(ww > 0.) {
            return Err(format!("`{}` turns nothing, so its screw has no axis",self.name));
        }
        let point = crate::space::cross(w,v0).map(|x| x/ww);
        Ok(Isa {point,direction:unit(w).expect("a turn"),pitch:crate::space::dot(w,v0)/ww,omega:w,v0})
    }

    /// Why step `s` has no screw the same at every time (`instantaneous`), as the drawing says it.
    fn unsteady(&self,s: &Step) -> String {
        if s.is_driven() {
            return format!("`{}` is driven by a law, whose screw moves as it runs: a \
                screw is read of turns, screws and slides at a constant rate",self.name)
        }
        match s {
            Step::Guided {..} => format!("`{}` rides along a path, whose screw moves as it \
                runs: a screw is read of turns, screws and slides at a constant rate",
                self.name),
            Step::Followed {..} => format!("`{}` stands where the drawing puts it as its \
                parameter runs, whose screw moves as it runs: a screw is read of turns, screws \
                and slides at a constant rate",self.name),
            _ => format!("`{}` is a motion relative to a relative one, whose \
                screw moves as it runs: a screw is read of turns, screws and slides about fixed \
                lines, and of one relative to another",self.name),
        }
    }

    /// How this motion carries the plane through `point` with unit `normal`, when it carries it
    /// within itself: a rotation whose axis is square to the plane and does not screw along it,
    /// or a translation along it. Every other motion (a screw, a relative motion, a graph of
    /// several steps) leaves the plane, and is none: a plane a motion does not preserve has no
    /// such reading.
    pub fn in_plane(&self,normal: [f64;3],point: [f64;3]) -> Option<PlaneRigid> {
        use crate::space::dot;
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
    /// lengths times `scale` (`generate::planar`'s steps). None where any step leaves the plane.
    pub fn in_frame(&self,frame: &crate::brep::geom::Frame,scale: f64) -> Option<Vec<PlanarStep>> {
        fn emit(family: &Family,i: usize,frame: &crate::brep::geom::Frame,scale: f64,out: &mut Vec<PlanarStep>) -> Option<()> {
            let step = &family.steps[i];
            if !matches!(step,Step::Relative {..}) && !step.keeps(frame.z) || step.is_driven() { return None }
            match *step {
                Step::Rotation {origin,axis,ratio,..} => {
                    // a turn about the plane's normal reversed is the opposite turn about it
                    let sign = crate::space::dot(axis,frame.z).signum();
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

    pub fn read(sk: &Sketch, index: usize) -> Result<Self,String> {
        let name = sk.motions.get(index).ok_or("no such motion")?.name.clone();
        let mut done = vec![None;sk.motions.len()];
        let mut visiting = vec![false;sk.motions.len()];
        let mut steps = Vec::new();
        fn visit(sk: &Sketch, i: usize, done: &mut [Option<usize>],
            visiting: &mut [bool], steps: &mut Vec<Step>, depth: usize) -> Result<usize,String> {
            let node = sk.motions.get(i).ok_or("no such motion")?;
            if let Some(i) = done[i] { return Ok(i); }
            if visiting[i] { return Err(format!("motion dependency cycle at `{}`",node.name)); }
            if depth >= 64 { return Err("motion dependencies exceed 64 levels".into()); }
            visiting[i] = true;
            let value = match node.def {
                MotionDef::Rotation {axis,..} => {
                    // a number written as a measurement is read off the drawing as it stands
                    let (ratio,advance) = node.rotation(sk)?;
                    let axis = sk.lines.get(axis as usize).ok_or("no such motion axis")?;
                    let origin = sk.world_point(axis.p1 as usize);
                    let b = sk.world_point(axis.p2 as usize);
                    let axis = std::array::from_fn(|k| b[k]-origin[k]);
                    if !origin.iter().all(|x| x.is_finite()) || !advance.is_finite()
                        || Motion::rotation(axis,0.,ratio).is_err() {
                        return Err(format!("`{}` needs a finite nondegenerate axis and angle",node.name));
                    }
                    Step::Rotation {origin,axis,ratio,advance,law:None}
                }
                // a turn in a view is a rotation about the line through its centre square to
                // the view
                MotionDef::Turn {centre,..} => {
                    let (ratio,_) = node.rotation(sk)?;
                    if centre as usize >= sk.points.len() { return Err("no such motion centre".into()); }
                    let origin = sk.world_point(centre as usize);
                    let axis = sk.plane_of(centre as usize).map(|i| sk.basis(i))
                        .unwrap_or_else(crate::plane::Basis::page).normal();
                    if !origin.iter().all(|x| x.is_finite()) || Motion::rotation(axis,0.,ratio).is_err() {
                        return Err(format!("`{}` needs a finite centre and angle",node.name));
                    }
                    Step::Rotation {origin,axis,ratio,advance:0.,law:None}
                }
                MotionDef::Translation {axis,..} => {
                    let advance = node.advance(sk)?;
                    let line = sk.lines.get(axis as usize).ok_or("no such motion axis")?;
                    let a = sk.world_point(line.p1 as usize);
                    let b = sk.world_point(line.p2 as usize);
                    let axis: [f64;3] = std::array::from_fn(|k| b[k]-a[k]);
                    if !axis.iter().all(|x| x.is_finite()) || unit(axis).is_none() || !advance.is_finite() {
                        return Err(format!("`{}` needs a finite nondegenerate axis and advance",node.name));
                    }
                    Step::Translation {axis,advance,law:None}
                }
                MotionDef::Relative {source,observer} => Step::Relative {
                    source:visit(sk,source as usize,done,visiting,steps,depth+1)?,
                    observer:visit(sk,observer as usize,done,visiting,steps,depth+1)?,
                },
                // a joint about an axis as the solve leaves it (#383): a turn, a screw, or with no
                // ratio a slide along its direction
                MotionDef::Joint {axis,ratio,advance} => {
                    // its numbers as the solve left them, where it found them (#383 stage 5)
                    let [ratio,advance] = node.numbers.map_or([ratio,advance],
                        |q| q.map(|p| sk.params.get(p as usize).map_or(f64::NAN,|x| x.value)));
                    let ax = sk.axes.get(axis as usize).ok_or("no such joint axis")?;
                    let origin: [f64;3] = ax.a.map(|q| sk.params[q as usize].value);
                    let d: [f64;3] = ax.d.map(|q| sk.params[q as usize].value);
                    if !origin.iter().all(|x| x.is_finite()) || unit(d).is_none()
                        || !ratio.is_finite() || !advance.is_finite() {
                        return Err(format!("`{}` needs a finite nondegenerate axis and numbers",
                            node.name));
                    }
                    if ratio == 0. { Step::Translation {axis:d,advance,law:None} }
                    else { Step::Rotation {origin,axis:d,ratio,advance,law:None} }
                }
                // a joint driven by its frame's value (#391 S5): the joint's step, standing at the
                // law's value of the roll
                MotionDef::Driven {of,ref law,ref over} => {
                    let k = visit(sk,of as usize,done,visiting,steps,depth+1)?;
                    let law = Some(Law::read(sk,law,over).map_err(|m| format!("`{}`: {m}",node.name))?);
                    match steps[k].clone() {
                        Step::Rotation {origin,axis,ratio,advance,..} =>
                            Step::Rotation {origin,axis,ratio,advance,law},
                        Step::Translation {axis,advance,..} => Step::Translation {axis,advance,law},
                        Step::Relative {..} | Step::Guided {..} | Step::Followed {..} =>
                            return Err(format!("`{}` drives no joint",node.name)),
                    }
                }
                // a frame along a path, read off the path as the solve left it (#398 C)
                MotionDef::Guided {guide,frame,frenet} => {
                    let read = Guide::read(sk,&node.name,guide,frame,frenet)?;
                    Step::Guided {guide:std::sync::Arc::new(read),back:false}
                }
                // a frame the drawing moves, read off its march as the solve left it (#398 F)
                MotionDef::Followed {frame,ref over} => {
                    let ride = Ride::read(sk,&node.name,frame,over)?;
                    Step::Followed {ride:std::sync::Arc::new(ride),back:false}
                }
                // a joint's inverse turns and slides back, and a relative's swaps its sides
                MotionDef::Inverse {of} => {
                    let k = visit(sk,of as usize,done,visiting,steps,depth+1)?;
                    steps[k].inverse()
                }
            };
            visiting[i] = false;
            let next = steps.len();
            steps.push(value);
            done[i] = Some(next);
            Ok(next)
        }
        visit(sk,index,&mut done,&mut visiting,&mut steps,0)?;
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
        let mut family = Self {name,steps,lowered:Err(String::new())};
        family.lowered = family.steps.len().checked_sub(1).ok_or_else(|| "a motion with no step".to_string())
            .and_then(|root| family.lowered(root));
        Ok(family)
    }

    /// Exact pose and derivative per radian of the shared angle. Relative motion is
    /// observer^-1 * source, including the derivative of the moving inverse. It is the lowered
    /// trajectory's (`Trajectory::at`, #721 F2b).
    pub fn at(&self, angle: f64) -> Result<Motion,String> {
        if !angle.is_finite() { return Err("a motion angle must be finite".into()); }
        self.path()?.at(angle).map_err(|e| e.msg)
    }
}

/// A rigid pose without its derivative: `x ↦ r·x + p`.
#[derive(Clone,Copy,Debug)]
pub struct Pose { pub r: [[f64;3];3], pub p: [f64;3] }

/// A frame's chart as a pose, as `sk` stands: its axes and normal the columns, its origin the
/// offset.
pub(crate) fn chart_pose(sk: &Sketch, f: usize) -> Pose {
    let b = sk.basis(f);
    let n = b.normal();
    Pose { r: std::array::from_fn(|i| [b.u[i], b.v[i], n[i]]), p: b.o }
}

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
    pub(crate) fn then(self,next: Pose) -> Pose {
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

/// Why a relation the solve holds cannot read the frame `f` the drawing moves as `over` runs: a
/// row reads no march, which is the drawing's own solve at every instant.  The one wording,
/// wherever the row is refused (#398 F).
pub fn unread_march(f: &str, over: &str) -> String {
    format!("`{f}` stands where the drawing puts it as `{over}` runs, read off the march after the \
             solve, and a relation the solve holds reads no march (#398): hold it to the drawing \
             itself, or read the frame over `{over}` — a sweep, a placement, an envelope")
}

/// **Motion node `m`, a frame the drawing moves, carried from where it stands** (#398 F): what a
/// profile generated in the plane reads its table from (`Ride::planar`), or why it cannot be.
pub(crate) fn ride_of(sk: &Sketch, m: usize) -> Result<Ride, String> {
    let node = sk.motions.get(m).ok_or("no such motion")?;
    let MotionDef::Followed {frame,ref over} = node.def else {
        return Err(format!("`{}` is no frame the drawing moves",node.name));
    };
    Ride::read(sk,&node.name,frame,over)
}

/// Read and evaluate against the current solved geometry. Reuse `Family` for many samples
/// of the same solved state, just as a surface evaluator reuses a solved surface snapshot.
pub fn evaluate(sk: &Sketch, index: usize, angle: f64) -> Result<Motion,String> {
    Family::read(sk,index)?.at(angle)
}
