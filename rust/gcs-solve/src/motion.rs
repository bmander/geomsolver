//! Named rigid-motion families, read off the sketch. Every node reads the same angular parameter
//! in radians. The families, their poses, screws, enclosures and temporal contacts are the
//! kernel's (`cad_kernel::sweep::motion`); here is what reads them off the model — a node's axes
//! as the solve left them, a drive's law (`law`), a frame along a path (`guide`) and a frame the
//! drawing moves (`track`), each handed to the kernel as its `Drive` or `Carrier`.
#[allow(unused_imports)]
use crate::fmath::Det;
pub use cad_kernel::sweep::motion::*;
mod law;
pub use law::{Law, unread_drive};
mod guide;
use guide::Guide;
mod track;
pub use track::Track;
pub(crate) use track::{window, Ride};

use crate::{envelope::Motion,interval::{Error as IntervalError,Interval as I},model::{MotionDef,Sketch}};
use std::sync::Arc;

impl Source for Sketch {
    fn family(&self, index: usize) -> Result<Family,String> { read(self,index) }
}

fn unit(axis: [f64;3]) -> Option<[f64;3]> {
    // sqrt of the sum and not `hypot`: evaluated at every pose, and axes are of drawing size
    let n = crate::space::norm(axis);
    (n > 0. && n.is_finite()).then(|| axis.map(|v| v/n))
}

/// Motion node `index` read off `sk` as the solve left it: its steps, each joint's numbers, a
/// drive's law, a frame's path or march, refused where a node is degenerate or the graph cycles.
fn read(sk: &Sketch, index: usize) -> Result<Family,String> {
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
                let law: Option<Arc<dyn Drive>> = Some(Arc::new(Law::read(sk,law,over).map_err(|m| format!("`{}`: {m}",node.name))?));
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
                Step::Guided {guide:Arc::new(read),back:false}
            }
            // a frame the drawing moves, read off its march as the solve left it (#398 F)
            MotionDef::Followed {frame,ref over} => {
                let ride = Ride::read(sk,&node.name,frame,over)?;
                Step::Followed {ride:Arc::new(ride),back:false}
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
    Family::new(name,steps)
}

impl Drive for Law {
    fn value(&self,theta: f64) -> Result<f64,String> { Law::value(self,theta) }
    fn jet(&self,theta: f64) -> Result<[f64;2],String> { Law::jet(self,theta) }
    fn enclose(&self,theta: I) -> Result<[I;2],IntervalError> { Law::enclose(self,theta) }
}

impl Carrier for Guide {
    fn motion(&self,u: f64) -> Result<Motion,String> { Guide::motion(self,u) }
    fn pose(&self,u: f64) -> Result<Pose,String> { Guide::pose(self,u) }
    fn stretch(&self,a: f64,b: f64) -> Option<Stretch> { Guide::stretch(self,a,b) }
}

impl Carrier for Ride {
    fn motion(&self,u: f64) -> Result<Motion,String> { Ride::motion(self,u) }
    fn pose(&self,u: f64) -> Result<Pose,String> { Ride::pose(self,u) }
    fn stretch(&self,a: f64,b: f64) -> Option<Stretch> { Ride::stretch(self,a,b) }
    fn speed_over(&self,b: [I;3],lo: f64,hi: f64) -> Option<f64> { Ride::speed_over(self,b,lo,hi) }
}

/// A frame's chart as a pose, as `sk` stands: its axes and normal the columns, its origin the
/// offset.
pub(crate) fn chart_pose(sk: &Sketch, f: usize) -> Pose {
    let b = sk.basis(f);
    let n = b.normal();
    Pose { r: std::array::from_fn(|i| [b.u[i], b.v[i], n[i]]), p: b.o }
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
