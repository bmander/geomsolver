//! **The closed class** (#721 F2c5's dispatch, docs/swept-solid-design.md): a sweep the other
//! classes refuse, of a polyhedral tool under one slide, turn or screw, built by the kernel's one
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

/// What the closed class read of a sweep: its tool's B-rep, its motion's constant twist and the
/// roll it is carried over, in millimetres — what the build hands the kernel.
#[derive(Clone,Debug)]
pub struct Closed {
    pub tool: Brep,
    pub screw: Screw,
    pub roll: [f64;2],
}

impl Closed {
    /// The motion's kind, in the words a report uses.
    pub fn kind(&self) -> &'static str { self.screw.kind() }
    /// The path the kernel is handed: the constant twist itself.
    pub fn path(&self) -> Trajectory { Trajectory::Twist(self.screw) }
}

/// A sweep outside the closed class: its row, what was seen, and where (in the sweep's own
/// coordinates, millimetres), where there is a point.
pub type Failure = (Condition,String,Option<V>);

/// The kernel's refusal said in the class's rows: a curved face or edge is C1's, a motion or a
/// leaning edge C2's, a cell left unsettled C3's; what the kernel failed at is said as it is.
pub fn said(unbuilt: Unbuilt,tool: &str) -> Failure {
    match unbuilt {
        Unbuilt::Curved {at} => (Condition::Faces,format!("a face or an edge of `{tool}` is curved"),Some(at)),
        Unbuilt::Motion => (Condition::Constant,"the motion is no one slide, turn or screw".into(),None),
        Unbuilt::Leaning {at} => (Condition::Constant,format!("an edge or a face of `{tool}` leans to the screw's axis, neither along \
            it nor square to it"),Some(at)),
        Unbuilt::Unsettled {at} => (Condition::Settled,"a cell of the arrangement stands within the tool's mesh of the swept \
            boundary at every point it is read at".into(),Some(at)),
        Unbuilt::Failed(m) => (Condition::Settled,format!("the kernel could not build it: {m}"),None),
    }
}

/// The sweep `swept`'s tool, motion and roll read in millimetres, and asked the kernel's
/// structural rows (C1, C2): the evidence for each of its `placements` (model units), or the row it
/// fails, its witness in the sweep's coordinates (model units).
pub fn admit(sk: &Sketch,swept: usize,placements: &[Motion]) -> Result<SweepEvidence,Failure> {
    let solid = &sk.solids[swept];
    let SolidDef::Swept {source,motion,..} = &solid.def else {
        return Err((Condition::Tool,format!("`{}` is not a continuous sweep",solid.name),None))
    };
    let tool_name = &sk.solids[*source as usize].name;
    let mm = cad::millimetres(sk);
    let unread = |condition,m: String| (condition,m,None);
    let tool = cad::recipe(sk,*source as usize).and_then(|r| crate::brep::recipe::build(&r))
        .map_err(|m| unread(Condition::Faces,format!("`{tool_name}` could not be built: {m}")))?;
    let family = crate::motion::Family::read(sk,*motion as usize).map_err(|m| unread(Condition::Constant,m))?;
    let path = family.trajectory().map_err(|m| unread(Condition::Constant,m))?;
    let roll = super::roll::interval(sk,swept).map_err(|m| unread(Condition::Constant,m))?;
    // a sweep is its path's, not its pace's (#684 L)
    let (path,roll) = path.unpaced(roll).map_err(|e| unread(Condition::Constant,e.msg))?;
    let Some(s) = path.screw() else { return Err(said(Unbuilt::Motion,tool_name)) };
    let screw = Screw {origin:s.origin.map(|x| x*mm),advance:s.advance*mm,..s};
    closed::reads(&tool,&Trajectory::Twist(screw)).map_err(|u| {
        let (c,m,w) = said(u,tool_name);
        (c,m,w.map(|p| p.map(|x| x/mm)))
    })?;
    let (faces,edges) = (tool.faces.len(),tool.edges.len());
    Ok(SweepEvidence {sweep:swept,name:solid.name.clone(),
        placements:placements.iter().map(|&pose| Placement {pose,equivalent_to:None}).collect(),
        samples:faces,contacts:edges,spacing:0.,least_area_factor:1.,near_double_roots:0,near_tangent_pairs:0,
        basis:Basis::Sampled {rows:0,columns:0},equivalence:Equivalence::Sampled,
        class:Class::Closed(Box::new(Closed {tool,screw,roll}))})
}
