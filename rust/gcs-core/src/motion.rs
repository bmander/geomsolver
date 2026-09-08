//! Named rigid-motion families. Every node reads the same angular parameter in radians.
mod bounds;
pub use bounds::MotionBounds;
use crate::{envelope::Motion,model::{MotionDef,Sketch}};

#[derive(Clone,Debug)]
enum Step {
    Rotation {origin:[f64;3],axis:[f64;3],ratio:f64,phase:f64},
    Relative {source:usize,observer:usize},
}

/// A snapshot of a named rigid-motion graph and its solved world axes. Re-read after
/// changing the sketch. Sampling a snapshot never reads drawing coordinates or solves it.
#[derive(Clone,Debug)]
pub struct Family {
    pub name: String,
    steps: Vec<Step>,
}

impl Family {
    /// Upper bound on the speed of inverse(M(t))*point per radian, over every t.
    /// This bounds the mathematical rigid family represented by the solved axes;
    /// it does not bound roundoff in `at` or error in the solved source geometry.
    pub fn inverse_point_speed_bound(&self,point: [f64;3]) -> Result<f64,crate::interval::Error> {
        use crate::interval::{Error,Interval as I};
        // A transform sends |x| to at most |x|+a and a moving point's speed to
        // at most |x'|+b*|x|+c. Keep both senses of every DAG node, avoiding
        // exponential expansion of shared relative motions.
        type Bound = [I;3];
        fn then(x: Bound,y: Bound) -> Result<Bound,Error> {
            Ok([x[0].add(y[0])?,x[1].add(y[1])?,x[2].add(y[2])?.add(y[1].mul(x[0])?)?])
        }
        // Outward addition can place a sum of exact zeros infinitesimally below
        // zero, so norms accumulate squares with an explicit nonnegative range.
        let norm = |p: [f64;3]| -> Result<I,Error> {
            let sum = p.into_iter().try_fold(I::ZERO,|s,x| s.add(I::point(x)?.square()?))?;
            I::new(sum.bounds()[0].max(0.),sum.bounds()[1])?.sqrt()
        };
        let mut bounds: Vec<[Bound;2]> = Vec::with_capacity(self.steps.len());
        for step in &self.steps {
            bounds.push(match *step {
                Step::Rotation {origin,ratio,..} => {
                    let r = norm(origin)?; let w = I::point(ratio.abs())?;
                    let b = [I::point(2.)?.mul(r)?,w,w.mul(r)?];
                    [b,b]
                }
                Step::Relative {source,observer} => [then(bounds[source][0],bounds[observer][1])?,
                    then(bounds[observer][0],bounds[source][1])?],
            });
        }
        let [_,b,c] = bounds.last().expect("a motion family has a root")[1];
        Ok(b.mul(norm(point)?)?.add(c)?.bounds()[1])
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
                MotionDef::Rotation {axis,ratio,phase} => {
                    let axis = sk.lines.get(axis as usize).ok_or("no such motion axis")?;
                    let origin = sk.world_point(axis.p1 as usize);
                    let b = sk.world_point(axis.p2 as usize);
                    let axis = std::array::from_fn(|k| b[k]-origin[k]);
                    if !origin.iter().all(|x| x.is_finite())
                        || Motion::rotation(axis,phase,ratio).is_err() {
                        return Err(format!("`{}` needs a finite nondegenerate axis and angle",node.name));
                    }
                    Step::Rotation {origin,axis,ratio,phase}
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
                Step::Rotation {..} => 1,
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
                Step::Rotation {origin,axis,ratio,phase} => {
                    let rotation = Motion::rotation(axis,phase+ratio*angle,ratio)
                        .map_err(|_| "a motion angle overflowed")?;
                    Motion::translation(origin.map(|v| -v),[0.;3]).unwrap()
                        .then(rotation).then(Motion::translation(origin,[0.;3]).unwrap())
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
