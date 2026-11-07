//! The planar subtraction class (#606, #607): a sweep of a tool standing along one direction under
//! a motion that turns the plane square to it in itself, cut from any blank — what the sweep
//! removes in each slab of the blank is the region its section sweeps, carried through the slab.
//! The class is the kernel's, its tool read from its B-rep (`brep::swept::subtraction`, #721 F1c):
//! however the tool was drawn — prisms joined by union, common or cut, drafted or not, a disc
//! turned — it is asked where every face of its boundary stands along, or square to, the normal of
//! a plane the motion keeps, and the kernel's dispatch asks it (`brep::swept::dispatch`). What is
//! here is the drawing's reading of a sweep for the class: its tool's B-rep, its motion and roll,
//! and the names its refusals say.
use super::admission::{Basis,Class,Equivalence,Placement,SweepEvidence};
use crate::brep::swept::subtraction::{Names,Subtraction};
use crate::envelope::Motion;
use crate::model::{Sketch,SolidDef};

type V = [f64;3];

/// What the class finds, its slabs, and how far and at how many stations a slab is cut, are the
/// kernel's.
pub use crate::brep::swept::subtraction::{Found,Slab,PAST,STATIONS,stations};

/// A row of the class that fails, what was seen, and a point of the body where.
pub type Failure = (super::admission::Condition,String,Option<V>);

/// Sweep `swept` read off the drawing for the class: its tool's B-rep, its motion and its roll, and
/// the names a refusal says; refused at P1 where the motion keeps no plane a face of its tool is
/// square to, at Q1 where the tool stands along no normal the motion keeps.
pub fn read(sk: &Sketch,swept: usize) -> Result<Subtraction,Failure> {
    let SolidDef::Swept {source,motion,..} = &sk.solids[swept].def else {
        return Err((super::admission::Condition::Members,"not a sweep".into(),None))
    };
    let q1 = |m: String| (super::admission::Condition::Members,m,None);
    let family = crate::motion::Family::read(sk,*motion as usize).map_err(|e| (super::admission::Condition::Plane,e,None))?;
    let path = family.trajectory().map_err(|e| (super::admission::Condition::Plane,e,None))?;
    let roll = super::roll::interval(sk,swept).map_err(|m| (super::admission::Condition::Closes,m,None))?;
    let tool = super::cad::tool_brep(sk,swept).map_err(q1)?;
    let names = Names {sweep:sk.solids[swept].name.clone(),tool:sk.solids[*source as usize].name.clone(),motion:family.name.clone()};
    Subtraction::read(&tool,&path,roll,super::cad::millimetres(sk),names).map_err(|(row,m,w)| (row.into(),m,w))
}

/// Whether the class reads sweep `swept`: its tool's boundary stands along the normal of a plane
/// its motion keeps (P1, Q1).
pub fn asks(sk: &Sketch,swept: usize) -> bool { read(sk,swept).is_ok() }

/// Sweep `swept`, cut from a blank at `poses` (the sweep's frame in the body's), the blank read by
/// `inside` and boxed by `bounds`, admitted to the class (rows P1, Q1–Q4) with its evidence, or
/// refused by row: what the accuracy meter reads.
pub fn admit(sk: &Sketch,swept: usize,poses: &[Motion],inside: &(dyn Fn(V) -> bool+Sync),bounds: Option<(V,V)>)
    -> Result<SweepEvidence,Failure> {
    let class = read(sk,swept)?;
    let found = class.check(poses,inside,bounds).map_err(|(row,m,w)| (row.into(),m,w))?;
    let sampled = class.sampled(&found);
    Ok(SweepEvidence {sweep:swept,name:sk.solids[swept].name.clone(),
        placements:poses.iter().map(|&pose| Placement {pose,equivalent_to:None}).collect(),samples:sampled.candidates,
        contacts:sampled.runs,spacing:(1./crate::brep::swept::region::PER_DEGREE).to_radians(),least_area_factor:1.,near_double_roots:0,
        near_tangent_pairs:0,basis:Basis::Sampled {rows:sampled.candidates,columns:sampled.steps},equivalence:Equivalence::Sampled,
        class:Class::Subtraction(Box::new(found))})
}
