//! **The closed class** (#721 F2c5's dispatch, docs/swept-solid-design.md): a sweep the other
//! classes refuse, of a polyhedral tool under one slide, turn or screw (or under a slide a tool of
//! planes, cylinders, cones and balls, its edges lines and circles; or under any other rigid motion,
//! a frame along a path, a convex tool of revolution, #780 L3), built by the kernel's one
//! pipeline (`brep::swept::closed`) — the blank split by every candidate face of the sweep's
//! boundary at once, each cell kept where the kernel's membership says it is no member of the
//! sweep. Nothing about the tool's contacts is sampled at admission: what the pipeline asks is the
//! tool's and the motion's structure (rows C1 and C2, the kernel's `closed::reads`), and what it
//! cannot settle it refuses while it builds (C3, a cell whose membership stays unsettled, with the
//! point). The solver reads the tool, the motion and the roll off the drawing and says the
//! kernel's refusals; the geometry is the kernel's.
use super::admission::{Basis,Class,Condition,Equivalence,Placement,SweepEvidence};
use super::cad;
use crate::brep::swept::closed::{self,Unbuilt};
use crate::brep::topo::Brep;
use crate::brep::trajectory::{Screw,Trajectory};
use crate::envelope::Motion;
use crate::model::{Sketch,SolidDef};

type V = [f64;3];

/// What the closed class read of a sweep: its tool's B-rep and name, its motion's constant twist
/// and the roll it is carried over, in millimetres — what the build hands the kernel. A motion no
/// constant twist takes (#780 L3, K2: a frame along a path) is `general`, the trajectory itself in
/// millimetres, its `screw` then moving nothing.
#[derive(Clone,Debug)]
pub struct Closed {
    pub tool: Brep,
    pub tool_name: String,
    pub screw: Screw,
    pub roll: [f64;2],
    pub general: Option<Trajectory>,
}

impl Closed {
    /// The motion's kind, in the words a report uses.
    pub fn kind(&self) -> &'static str { if self.general.is_some() { "any rigid motion" } else { self.screw.kind() } }

    /// The motion the kernel builds the sweep under, in millimetres.
    pub fn path(&self) -> Trajectory { self.general.clone().unwrap_or(Trajectory::Twist(self.screw)) }
}

/// **A trajectory read in millimetres**: the model's poses with their offsets scaled by `k` (a turn
/// is a turn in any unit), its speed bounds likewise — what a motion no constant twist takes is
/// handed the kernel as (`Closed::general`).
#[derive(Debug)]
struct Scaled { path: Trajectory,k: f64 }

impl crate::brep::trajectory::Path for Scaled {
    fn at(&self,t: f64) -> Result<crate::brep::trajectory::Motion,crate::brep::KernelError> {
        let m = self.path.at(t)?;
        let p = m.point([0.;3]);
        let dp = m.velocity([0.;3]);
        let column = |k: usize| { let mut e = [0.;3]; e[k] = 1.; (crate::space::sub(m.point(e),p),crate::space::sub(m.velocity(e),dp)) };
        let cols = [column(0),column(1),column(2)];
        let r = std::array::from_fn(|i| std::array::from_fn(|j| cols[j].0[i]));
        let dr = std::array::from_fn(|i| std::array::from_fn(|j| cols[j].1[i]));
        Ok(crate::brep::trajectory::Motion::of_parts(r,dr,p.map(|x| x*self.k),dp.map(|x| x*self.k)))
    }
    fn pose(&self,t: f64) -> Result<crate::brep::trajectory::Pose,crate::brep::KernelError> {
        let q = self.path.pose(t)?;
        Ok(crate::brep::trajectory::Pose {r:q.r,p:q.p.map(|x| x*self.k)})
    }
    fn stretch(&self,a: f64,b: f64) -> Option<crate::brep::trajectory::Stretch> {
        let Trajectory::General(inner) = &self.path else { return None };
        let g = inner.stretch(a,b)?;
        let k = crate::interval::Interval::point(self.k).ok()?;
        let point = [g.point[0].mul(k).ok()?,g.point[1].mul(k).ok()?,g.point[2].mul(k).ok()?];
        Some(crate::brep::trajectory::Stretch {point,turn:g.turn,speed:g.speed*self.k,spin:g.spin,origin:g.origin.map(|x| x*self.k)})
    }
    fn speed_over(&self,b: [crate::interval::Interval;3],lo: f64,hi: f64,_: &crate::brep::trajectory::Stretch)
        -> Result<crate::interval::Interval,crate::interval::Error> {
        let Trajectory::General(inner) = &self.path else { return Err(crate::interval::Error::InvalidBounds) };
        let g = inner.stretch(lo,hi).ok_or(crate::interval::Error::InvalidBounds)?;
        let k = crate::interval::Interval::point(self.k)?;
        let back = crate::interval::Interval::point(1./self.k)?;
        inner.speed_over([b[0].mul(back)?,b[1].mul(back)?,b[2].mul(back)?],lo,hi,&g)?.mul(k)
    }
}

/// A closed sweep's build refused: its row, what was seen, and where (in the sweep's own
/// coordinates, millimetres), where there is a point.
pub type Failure = (Condition,String,Option<V>);

/// The kernel's refusal said in the class's rows: a curved face or edge (one the motion's
/// candidates do not hold) is C1's, and so is a contact whose sheet folds, a motion C2's, a cell left unsettled C3's; what the kernel failed at is said as it is.
pub fn said(unbuilt: Unbuilt,tool: &str) -> Failure {
    match unbuilt {
        Unbuilt::Curved {at} => (Condition::Faces,format!("a face or an edge of `{tool}` is curved past what the motion's \
            candidates hold"),Some(at)),
        Unbuilt::Folded {at} => (Condition::Faces,format!("a contact of `{tool}` folds as the motion carries it, its swept sheet \
            doubling back where the patch is not yet cut"),Some(at)),
        Unbuilt::Returns {at} => (Condition::Faces,format!("the sheet `{tool}`'s contact sweeps comes back onto itself as the motion \
            carries it, where it is not yet entered in stretches"),Some(at)),
        Unbuilt::Motion => (Condition::Constant,"the motion is no one slide, turn or screw".into(),None),
        Unbuilt::Unsettled {at} => (Condition::Settled,"a cell of the arrangement stands within the tool's mesh of the swept \
            boundary at every point it is read at".into(),Some(at)),
        Unbuilt::Budget {stage,at,what} => (Condition::Settled,format!("the build went past its budget in its {stage} \
            stage, on a {what}"),Some(at)),
        Unbuilt::Wrong {at,held} => (Condition::Settled,format!("the built solid {} a point the sweep's definition {}: refused, \
            not returned wrong",if held { "holds" } else { "leaves out" },if held { "leaves out" } else { "holds" }),Some(at)),
        Unbuilt::Failed(m) => (Condition::Settled,format!("the kernel could not build it: {m}"),None),
    }
}

/// The sweep `swept`'s motion, roll and tool read in millimetres and asked the kernel's structural
/// rows (C1, C2), the motion first, which needs no tool built: the evidence for each of its
/// `placements` (model units), or none where they refuse (the first class's refusal then stands).
pub fn admit(sk: &Sketch,swept: usize,placements: &[Motion]) -> Option<SweepEvidence> {
    let solid = &sk.solids[swept];
    let SolidDef::Swept {source,motion,..} = &solid.def else { return None };
    let path = crate::motion::Family::read(sk,*motion as usize).ok()?.trajectory().ok()?;
    let roll = super::roll::interval(sk,swept).ok()?;
    // a sweep is its path's, not its pace's (#684 L)
    let (path,roll) = path.unpaced(roll).ok()?;
    let mm = cad::millimetres(sk);
    let tool = crate::brep::recipe::build(&cad::recipe(sk,*source as usize).ok()?).ok()?;
    // a constant twist, or a frame along a path (K2, #780 L3: each instant's contact swept into a
    // patch, the tool a convex solid of revolution)
    let (screw,general) = match path.screw() {
        Some(s) => { let screw = s.scaled(mm); closed::reads(&tool,&Trajectory::Twist(screw)).ok()?; (screw,None) }
        None => {
            let general = if mm == 1. { path.clone() } else {
                Trajectory::General(std::sync::Arc::new(Scaled {path:path.clone(),k:mm}))
            };
            closed::reads_general(&tool,&general,roll).ok()?;
            (Screw {origin:[0.;3],axis:[0.,0.,1.],ratio:0.,advance:0.},Some(general))
        }
    };
    Some(SweepEvidence {sweep:swept,name:solid.name.clone(),
        placements:placements.iter().map(|&pose| Placement {pose,equivalent_to:None}).collect(),
        samples:0,contacts:0,spacing:0.,least_area_factor:1.,near_double_roots:0,near_tangent_pairs:0,
        basis:Basis::Sampled {rows:0,columns:0},equivalence:Equivalence::Sampled,
        class:Class::Closed(Box::new(Closed {tool,tool_name:sk.solids[*source as usize].name.clone(),screw,roll,general}))})
}
