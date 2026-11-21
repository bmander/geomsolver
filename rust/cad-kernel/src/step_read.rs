//! A STEP file read into the solids it gives (#993, stage 1): AP203, AP214 and AP242's advanced
//! B-rep, each face on the surface the file gives it and each edge on its curve, nothing
//! refitted. **Read correctly or refused, never read wrong**: what the reader does not read — an
//! entity it has no geometry for, a shell that does not close, a face whose bounds cross — is a
//! `Refusal` naming the entity (`Refusal::entity`, the file's own `#n`), and no solid is returned
//! beside it. The file's units and its stated uncertainty are read (`context`), never assumed:
//! the solid is in millimetres and radians whatever the file measures in. A tolerance is
//! *measured* (`topo::Brep::measure_tolerances`: how far each edge's curve stands from its
//! faces' surfaces and its ends from its vertices), never taken from the file's bookkeeping and
//! never widened to make a check pass.
//!
//! The parts: `step_file` is the text's syntax; `context` finds the solids a file represents,
//! each with its units and uncertainty; `geometry` reads a curve or a surface; `topology` walks a
//! solid from its shells down into a `Brep`; `sound` holds the result to what a solid's boundary
//! must be, beyond `Brep::check`.
pub mod context;
pub mod geometry;
pub mod topology;
pub mod sound;

use super::step_file::File;
use super::topo::Brep;
use super::{KernelError,Kind};

/// Why a file was not read.
#[derive(Clone,Copy,Debug,PartialEq,Eq)]
pub enum Why {
    /// The text is no STEP file, or an entity of it does not say what its kind must (a parameter
    /// missing or of another type, a reference to an entity of the wrong kind).
    Malformed,
    /// An entity, a unit or a structure the reader does not read yet: the file may be a good one.
    Unsupported,
    /// The file represents no solid.
    Empty,
    /// A shell that does not close (`Kind::Open`).
    Open,
    /// A boundary that is no manifold (`Kind::NotManifold`).
    NotManifold,
    /// A boundary that closes and is no solid's: a face whose bounds cross or turn the wrong way,
    /// a shell inside out, an edge off its face farther than the reader accepts.
    Unsound,
}

/// A file refused: why, the entity that witnesses it (the file's `#n`; none where the witness is
/// the file itself, as a text that does not parse), and what there is to say.
#[derive(Clone,Debug,PartialEq)]
pub struct Refusal { pub why: Why,pub entity: Option<u32>,pub msg: String }

impl Refusal {
    pub fn new(why: Why,entity: Option<u32>,msg: impl Into<String>) -> Refusal { Refusal {why,entity,msg:msg.into()} }
    /// `#entity` does not say what its kind must.
    pub fn malformed(entity: u32,msg: impl Into<String>) -> Refusal { Refusal::new(Why::Malformed,Some(entity),msg) }
    /// `#entity` is something the reader does not read yet.
    pub fn unsupported(entity: u32,msg: impl Into<String>) -> Refusal { Refusal::new(Why::Unsupported,Some(entity),msg) }
}

impl std::fmt::Display for Refusal {
    fn fmt(&self,f: &mut std::fmt::Formatter) -> std::fmt::Result {
        match self.entity { Some(e) => write!(f,"#{e}: {}",self.msg),None => f.write_str(&self.msg) }
    }
}

impl std::error::Error for Refusal {}

impl From<Refusal> for KernelError {
    fn from(r: Refusal) -> KernelError {
        let kind = match r.why { Why::Open => Kind::Open,Why::NotManifold => Kind::NotManifold,
            Why::Unsupported => Kind::Refused,Why::Malformed | Why::Empty | Why::Unsound => Kind::Failed };
        KernelError::new(kind,r.to_string())
    }
}

/// A solid a file gives: its boundary in millimetres and radians, and where each part of it came
/// from.
#[derive(Clone,Debug)]
pub struct Given {
    /// Its product's name, as the file says it (empty where it says none).
    pub name: String,
    /// The solid's entity (a `MANIFOLD_SOLID_BREP` or a `BREP_WITH_VOIDS`).
    pub entity: u32,
    pub brep: Brep,
    /// The units and uncertainty of the representation it was read from.
    pub context: context::Context,
    /// The entity each face, edge and vertex of `brep` was read from, by its index there; 0 for
    /// one the file does not write and the reader laid (a pole's degenerate edge).
    pub faces: Vec<u32>,
    pub edges: Vec<u32>,
    pub vertices: Vec<u32>,
}

/// Every solid `text` gives, in the file's order; refused where any of them is, or where there
/// is none.
pub fn read(text: &str) -> Result<Vec<Given>,Refusal> {
    let file = File::parse(text,&|_| {}).map_err(|m| Refusal::new(Why::Malformed,None,m))?;
    let found = context::solids(&file)?;
    if found.is_empty() { return Err(Refusal::new(Why::Empty,None,"the file represents no solid")) }
    found.into_iter().map(|f| {
        let given = topology::solid(&file,f)?;
        sound::sound(&given.brep,&given.context).map_err(|u| u.refusal(&given))?;
        Ok(given)
    }).collect()
}
