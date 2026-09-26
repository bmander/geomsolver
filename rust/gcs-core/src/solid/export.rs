//! Why an export was refused, in one vocabulary whichever backend refused it: the stage it was
//! met at, the class row when admission refused it, a point where when there is one, and the
//! words a person reads. A host writes files and builds kernel shapes; what a refusal *is* is
//! the core's, so a harness, a front end and the terminal all read the same thing.
use super::admission;
use std::fmt;

/// The stages of an export, in the order a swept export completes them. `key` is what a
/// harness's stage trace records (`SOLVENT_STAGE_TRACE`); the prose on stderr may change and
/// these may not.
#[derive(Clone,Copy,Debug,PartialEq,Eq,PartialOrd,Ord)]
pub enum Stage {
    Admission, Blank, Clearance, Reach, Sheet, Fit, Withheld, Split, Classify, Fuse, Step, Stl, Mesh,
    Agreement, Written,
    /// The core's refinement of the material field (`--stl-backend refine`): its features and
    /// its mesh, before any file. Off the native order: a native export never passes it.
    Refine,
}

impl Stage {
    /// The native export's stages in the order they complete; a record that stops short stopped
    /// at the first of these it did not complete.
    pub const ORDER: [Stage;15] = [Stage::Admission,Stage::Blank,Stage::Clearance,Stage::Reach,Stage::Sheet,
        Stage::Fit,Stage::Withheld,Stage::Split,Stage::Classify,Stage::Fuse,Stage::Step,Stage::Stl,Stage::Mesh,
        Stage::Agreement,Stage::Written];

    pub fn key(self) -> &'static str {
        match self {
            Stage::Admission => "admission", Stage::Blank => "blank", Stage::Clearance => "clearance",
            Stage::Reach => "reach", Stage::Sheet => "sheet", Stage::Fit => "fit", Stage::Withheld => "withheld",
            Stage::Split => "split", Stage::Classify => "classify", Stage::Fuse => "fuse", Stage::Step => "step",
            Stage::Stl => "stl", Stage::Mesh => "mesh", Stage::Agreement => "agreement", Stage::Written => "written",
            Stage::Refine => "refine",
        }
    }

    pub fn from_key(key: &str) -> Option<Stage> {
        Stage::ORDER.into_iter().chain([Stage::Refine]).find(|s| s.key() == key)
    }

    /// Whose stage it is. The class: admission says the design is outside it, which is no
    /// failure. Construction: our own analytic sampling and tracing of the sheet, or the field's
    /// refinement. Fit: the sheet as a kernel surface, judged by our contract. Kernel: what OCCT
    /// builds, splits, fuses and meshes, and our checks of its output. Gate: the final field
    /// probe. Where a refusal is met, not what caused it: a kernel refusal may be a bad sheet
    /// handed on.
    pub fn owner(self) -> Option<&'static str> {
        match self {
            Stage::Admission => Some("class"),
            Stage::Reach | Stage::Sheet | Stage::Refine => Some("construction"),
            Stage::Fit | Stage::Withheld => Some("fit"),
            Stage::Blank | Stage::Clearance | Stage::Split | Stage::Classify | Stage::Fuse | Stage::Step | Stage::Stl
                | Stage::Mesh => Some("kernel"),
            Stage::Agreement => Some("gate"),
            Stage::Written => None,
        }
    }
}

/// An export refused: at which stage, by which row of the class when admission refused it, a
/// point where (the body's coordinates) when one is known, and what a person is told.
#[derive(Clone,Debug)]
pub struct ExportRefusal {
    pub stage: Stage,
    pub condition: Option<admission::Condition>,
    pub witness: Option<[f64;3]>,
    pub message: String,
}

impl ExportRefusal {
    pub fn at(stage: Stage,message: impl Into<String>) -> ExportRefusal {
        ExportRefusal {stage,condition:None,witness:None,message:message.into()}
    }
}

impl From<admission::Error> for ExportRefusal {
    fn from(e: admission::Error) -> ExportRefusal {
        let message = e.to_string();
        match e {
            admission::Error::Refused(r) =>
                ExportRefusal {stage:Stage::Admission,condition:Some(r.condition),witness:r.witness,message},
            admission::Error::Unreadable(_) => ExportRefusal::at(Stage::Admission,message),
        }
    }
}

impl fmt::Display for ExportRefusal {
    fn fmt(&self,f: &mut fmt::Formatter) -> fmt::Result { f.write_str(&self.message) }
}

/// A fallible step of an export, told its stage: `construct(..).at(Stage::Blank)?`.
pub trait AtStage<T> {
    fn at(self,stage: Stage) -> Result<T,ExportRefusal>;
}

impl<T,E: Into<String>> AtStage<T> for Result<T,E> {
    fn at(self,stage: Stage) -> Result<T,ExportRefusal> { self.map_err(|e| ExportRefusal::at(stage,e)) }
}
