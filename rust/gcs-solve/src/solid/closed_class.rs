//! **The closed class** (#721 F2c5's dispatch, docs/swept-solid-design.md): a sweep the other
//! classes refuse, of a polyhedral tool under one slide, turn or screw (or under a slide a tool of
//! planes, cylinders, cones and balls, its edges lines and circles; or under any other rigid motion,
//! a frame along a path, a convex tool of revolution, #780 L3), built by the kernel's one
//! pipeline (`brep::swept::closed`) — the blank split by every candidate face of the sweep's
//! boundary at once, each cell kept where the kernel's membership says it is no member of the
//! sweep. Under a constant twist nothing about the tool's contacts is sampled at admission: what
//! the pipeline asks is the tool's and the motion's structure (rows C1 and C2, the kernel's
//! `closed::reads`). Under any other motion the contact moves on the tool, so its structure is
//! read at instants along the roll (`closed::reads_general`: one closed curve whose pieces keep
//! their kinds, a path that comes back nowhere), and a sweep it refuses keeps the refusal of the
//! class asked first. What it cannot settle it refuses while it builds (C3, a cell whose membership
//! stays unsettled, with the
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

/// What the closed class read of a sweep: its tool's B-rep and name, its motion and the roll it is
/// carried over, in millimetres — what the build hands the kernel: a constant twist
/// (`Trajectory::Twist`), or any other rigid motion (#780 L3, K2: a frame along a path).
#[derive(Clone,Debug)]
pub struct Closed {
    pub tool: Brep,
    pub tool_name: String,
    pub path: Trajectory,
    pub roll: [f64;2],
}

impl Closed {
    /// The constant twist the motion is, where it is one.
    pub fn screw(&self) -> Option<Screw> { self.path.screw() }

    /// The motion's kind, in the words a report uses.
    pub fn kind(&self) -> &'static str { self.screw().map_or("any rigid motion",|s| s.kind()) }
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
    // a constant twist, or any other motion (K2, #780 L3: each instant's contact swept into a patch,
    // the tool a convex solid of revolution)
    let path = path.scaled(mm);
    match path.screw() {
        Some(_) => closed::reads(&tool,&path).map(|_| ()),
        None => closed::reads_general(&tool,&path,roll),
    }.ok()?;
    Some(SweepEvidence {sweep:swept,name:solid.name.clone(),
        placements:placements.iter().map(|&pose| Placement {pose,equivalent_to:None}).collect(),
        samples:0,contacts:0,spacing:0.,least_area_factor:1.,near_double_roots:0,near_tangent_pairs:0,
        basis:Basis::Sampled {rows:0,columns:0},equivalence:Equivalence::Sampled,
        class:Class::Closed(Box::new(Closed {tool,tool_name:sk.solids[*source as usize].name.clone(),path,roll}))})
}
