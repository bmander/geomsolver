//! **The closed class** (#721 F2c5's dispatch, docs/swept-solid-design.md): a sweep the other
//! classes refuse, of a polyhedral tool under one slide, turn or screw (or under a slide a tool of
//! planes, cylinders, cones and balls, its edges lines and circles), built by the kernel's one
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
/// and the roll it is carried over, in millimetres — what the build hands the kernel.
#[derive(Clone,Debug)]
pub struct Closed {
    pub tool: Brep,
    pub tool_name: String,
    pub screw: Screw,
    pub roll: [f64;2],
}

impl Closed {
    /// The motion's kind, in the words a report uses.
    pub fn kind(&self) -> &'static str { self.screw.kind() }
}

/// A closed sweep's build refused: its row, what was seen, and where (in the sweep's own
/// coordinates, millimetres), where there is a point.
pub type Failure = (Condition,String,Option<V>);

/// The kernel's refusal said in the class's rows: a curved face or edge (one the motion's
/// candidates do not hold) is C1's, a motion C2's, a cell left unsettled C3's; what the kernel failed at is said as it is.
pub fn said(unbuilt: Unbuilt,tool: &str) -> Failure {
    match unbuilt {
        Unbuilt::Curved {at} => (Condition::Faces,format!("a face or an edge of `{tool}` is curved past what the motion's \
            candidates hold"),Some(at)),
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
    let screw = path.screw()?.scaled(cad::millimetres(sk));
    let tool = crate::brep::recipe::build(&cad::recipe(sk,*source as usize).ok()?).ok()?;
    closed::reads(&tool,&Trajectory::Twist(screw)).ok()?;
    Some(SweepEvidence {sweep:swept,name:solid.name.clone(),
        placements:placements.iter().map(|&pose| Placement {pose,equivalent_to:None}).collect(),
        samples:0,contacts:0,spacing:0.,least_area_factor:1.,near_double_roots:0,near_tangent_pairs:0,
        basis:Basis::Sampled {rows:0,columns:0},equivalence:Equivalence::Sampled,
        class:Class::Closed(Box::new(Closed {tool,tool_name:sk.solids[*source as usize].name.clone(),screw,roll}))})
}
