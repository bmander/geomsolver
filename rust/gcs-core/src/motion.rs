//! Named rigid-motion families. Every node reads the same angular parameter in radians.
mod bounds;
pub use bounds::MotionBounds;
mod contact;
pub use contact::{NormalVelocity,ContactTime,NormalVelocityBounds};
use crate::{envelope::Motion,model::{MotionDef,Sketch}};

/// `advance` is the length travelled along the unit axis per full turn of the
/// shared parameter, so a rotation with a nonzero advance is a screw.
#[derive(Clone,Debug)]
enum Step {
    Rotation {origin:[f64;3],axis:[f64;3],ratio:f64,phase:f64,advance:f64},
    Translation {axis:[f64;3],advance:f64},
    Relative {source:usize,observer:usize},
}

fn unit(axis: [f64;3]) -> Option<[f64;3]> {
    let n = axis[0].hypot(axis[1]).hypot(axis[2]);
    (n > 0. && n.is_finite()).then(|| axis.map(|v| v/n))
}

/// A snapshot of a named rigid-motion graph and its solved world axes. Re-read after
/// changing the sketch. Sampling a snapshot never reads drawing coordinates or solves it.
#[derive(Clone,Debug)]
pub struct Family {
    pub name: String,
    steps: Vec<Step>,
}

impl Family {
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
        // a step's own speed on a box, the same forward and inverse
        let own = |step: &Step,b: [I;3]| -> Result<I,Error> {
            match *step {
                Step::Rotation {origin,axis,ratio,advance,..} => {
                    let (_,radius) = about(origin,axis,b)?;
                    I::point(ratio.abs())?.mul(radius)?.add(I::point(advance.abs())?.div(tau)?)
                }
                Step::Translation {advance,..} => I::point(advance.abs())?.div(tau),
                Step::Relative {..} => unreachable!("composed below"),
            }
        };
        // where a step carries a box over the whole domain, forward or
        // inverse: within the box's farthest distance of its foot on the
        // axis, and the advance's reach along it; no angle enters, so no
        // interval trigonometry limits the domain
        let image = |step: &Step,b: [I;3]| -> Result<[I;3],Error> {
            match *step {
                Step::Rotation {origin,axis,advance,..} => {
                    let a = unit(axis).ok_or(Error::InvalidBounds)?;
                    let (foot,radius) = about(origin,axis,b)?;
                    let slide = I::point(advance.abs())?.div(tau)?.mul(reach)?;
                    let spread = radius.add(slide)?;
                    let _ = a;
                    Ok([foot[0].add(I::new(-spread.bounds()[1],spread.bounds()[1])?)?,foot[1].add(I::new(-spread.bounds()[1],spread.bounds()[1])?)?,
                        foot[2].add(I::new(-spread.bounds()[1],spread.bounds()[1])?)?])
                }
                Step::Translation {axis,advance} => {
                    let a = unit(axis).ok_or(Error::InvalidBounds)?;
                    let slide = I::point(advance.abs())?.div(tau)?.mul(reach)?.bounds()[1];
                    Ok([b[0].add(I::new(-slide*a[0].abs(),slide*a[0].abs())?)?,b[1].add(I::new(-slide*a[1].abs(),slide*a[1].abs())?)?,
                        b[2].add(I::new(-slide*a[2].abs(),slide*a[2].abs())?)?])
                }
                Step::Relative {..} => unreachable!("composed below"),
            }
        };
        fn image_of(family: &Family,image: &dyn Fn(&Step,[I;3]) -> Result<[I;3],Error>,i: usize,b: [I;3],forward: bool) -> Result<[I;3],Error> {
            match family.steps[i] {
                Step::Relative {source,observer} => {
                    // forward: x ↦ O⁻¹(S x); inverse: x ↦ S⁻¹(O x)
                    let (first,second) = if forward { (source,observer) } else { (observer,source) };
                    let moved = image_of(family,image,first,b,true)?;
                    image_of(family,image,second,moved,false)
                }
                ref step => image(step,b),
            }
        }
        fn speed(family: &Family,own: &dyn Fn(&Step,[I;3]) -> Result<I,Error>,image: &dyn Fn(&Step,[I;3]) -> Result<[I;3],Error>,
            i: usize,b: [I;3],forward: bool) -> Result<I,Error> {
            match family.steps[i] {
                Step::Relative {source,observer} => {
                    let (first,second) = if forward { (source,observer) } else { (observer,source) };
                    let moved = speed(family,own,image,first,b,true)?;
                    let carried = image_of(family,image,first,b,true)?;
                    moved.add(speed(family,own,image,second,carried,false)?)
                }
                ref step => own(step,b),
            }
        }
        let root = self.steps.len()-1;
        Ok(speed(self,&own,&image,root,point,false)?.bounds()[1])
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
                MotionDef::Rotation {axis,ratio,phase,advance} => {
                    let axis = sk.lines.get(axis as usize).ok_or("no such motion axis")?;
                    let origin = sk.world_point(axis.p1 as usize);
                    let b = sk.world_point(axis.p2 as usize);
                    let axis = std::array::from_fn(|k| b[k]-origin[k]);
                    if !origin.iter().all(|x| x.is_finite()) || !advance.is_finite()
                        || Motion::rotation(axis,phase,ratio).is_err() {
                        return Err(format!("`{}` needs a finite nondegenerate axis and angle",node.name));
                    }
                    Step::Rotation {origin,axis,ratio,phase,advance}
                }
                MotionDef::Translation {axis,advance} => {
                    let line = sk.lines.get(axis as usize).ok_or("no such motion axis")?;
                    let a = sk.world_point(line.p1 as usize);
                    let b = sk.world_point(line.p2 as usize);
                    let axis: [f64;3] = std::array::from_fn(|k| b[k]-a[k]);
                    if !axis.iter().all(|x| x.is_finite()) || unit(axis).is_none() || !advance.is_finite() {
                        return Err(format!("`{}` needs a finite nondegenerate axis and advance",node.name));
                    }
                    Step::Translation {axis,advance}
                }
                MotionDef::Relative {source,observer} => Step::Relative {
                    source:visit(sk,source as usize,done,visiting,steps,depth+1)?,
                    observer:visit(sk,observer as usize,done,visiting,steps,depth+1)?,
                },
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
                Step::Rotation {..} | Step::Translation {..} => 1,
                Step::Relative {source,observer} => 1+heights[source].max(heights[observer]),
            };
            if height > 64 { return Err("motion dependencies exceed 64 levels".into()); }
            heights.push(height);
        }
        Ok(Self {name,steps})
    }

    /// Exact pose and derivative per radian of the shared angle. Relative motion is
    /// observer^-1 * source, including the derivative of the moving inverse.
    pub fn at(&self, angle: f64) -> Result<Motion,String> {
        if !angle.is_finite() { return Err("a motion angle must be finite".into()); }
        let mut values: Vec<Motion> = Vec::with_capacity(self.steps.len());
        for step in &self.steps {
            let value = match *step {
                Step::Rotation {origin,axis,ratio,phase,advance} => {
                    let rotation = Motion::rotation(axis,phase+ratio*angle,ratio)
                        .map_err(|_| "a motion angle overflowed")?;
                    let pose = Motion::translation(origin.map(|v| -v),[0.;3]).unwrap()
                        .then(rotation).then(Motion::translation(origin,[0.;3]).unwrap());
                    if advance == 0. { pose } else {
                        let direction = unit(axis).ok_or("a motion needs a nondegenerate axis")?;
                        let rate = advance/std::f64::consts::TAU;
                        pose.then(Motion::translation(direction.map(|v| v*rate*angle),direction.map(|v| v*rate))
                            .map_err(|_| "a motion advance overflowed")?)
                    }
                }
                Step::Translation {axis,advance} => {
                    let direction = unit(axis).ok_or("a motion needs a nondegenerate axis")?;
                    let rate = advance/std::f64::consts::TAU;
                    Motion::translation(direction.map(|v| v*rate*angle),direction.map(|v| v*rate))
                        .map_err(|_| "a motion advance overflowed")?
                }
                Step::Relative {source,observer} => values[source].then(values[observer].inverse()),
            };
            if !value.is_finite() { return Err("a motion pose or derivative overflowed".into()); }
            values.push(value);
        }
        Ok(*values.last().expect("a motion family has a root"))
    }
}

/// Read and evaluate against the current solved geometry. Reuse `Family` for many samples
/// of the same solved state, just as a surface evaluator reuses a solved surface snapshot.
pub fn evaluate(sk: &Sketch, index: usize, angle: f64) -> Result<Motion,String> {
    Family::read(sk,index)?.at(angle)
}
