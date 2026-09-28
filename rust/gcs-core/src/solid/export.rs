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

/// A native export's stated tolerance: how far the written surface may lie from the exact one,
/// in native millimetres (a physical length, whatever unit the document is written in), and
/// every bar that follows from it. Without one an export keeps its gross bars (a fit within
/// 0.25 mm and 20°, the mesher's 0.01 mm deflection, the field probed 0.1 mm off), which separate
/// a fit that follows its contacts from one that does not and are no fabrication claim.
#[derive(Clone,Copy,Debug,PartialEq)]
pub struct Tolerance { pub millimetres: f64 }

impl Tolerance {
    /// The fabrication tolerance an export is held to when none is named: 10 µm.
    pub const FABRICATION: Tolerance = Tolerance {millimetres:0.01};
    /// The share of the tolerance a fitted sheet may use; the STL's chordal sag has the rest, so
    /// the sheet (the STEP's surface) plus the sag (the STL's) stays within the tolerance.
    pub const FIT_SHARE: f64 = 0.5;
    /// The gross bar a fitted sheet's normal is never allowed past, in degrees.
    pub const MOST_TURN: f64 = 20.;
    /// The least distance the field is probed at off a mesh: four times the largest vertex and
    /// edge tolerance the kernel has left on a united member (4.8e-4 mm), inside which a probe
    /// could stand in the kernel's own tolerance band rather than on one side of the surface.
    pub const LEAST_PROBE: f64 = 0.002;

    pub fn new(millimetres: f64) -> Result<Tolerance,String> {
        if millimetres.is_finite() && millimetres > 0. { Ok(Tolerance {millimetres}) }
        else { Err(format!("an export tolerance must be a positive length, not {millimetres} mm")) }
    }

    /// How far a fitted sheet may pass from any withheld contact.
    pub fn fit(&self) -> f64 { self.millimetres*Self::FIT_SHARE }

    /// How far a fitted sheet's normal may turn from a withheld contact's, in degrees, where the
    /// sheet passes `gap` from the contact and its nodes there are `spacing` apart (the shortest
    /// side of the cells beside it). Withheld contacts lie on a grid of half cells, so no point of
    /// the sheet is further than a quarter of a cell from one along either direction; a sheet
    /// turned by `θ` there departs from the exact surface by about `gap + tan θ · spacing / 4`
    /// within that reach, and `θ` may not carry it past the whole tolerance, the STEP's claim:
    /// `tan θ ≤ 4 (tolerance − gap) / spacing`. A pleat between contacts (the gear's, phase 2,
    /// turned 83° within 60 µm) fails it; a fit rounding a narrow strip where the exact normal
    /// turns fast (a convex corner's fan, a small fillet), which is near in position and turned by
    /// a few degrees, does not. Never past `MOST_TURN`, whatever the spacing.
    pub fn turn(&self,gap: f64,spacing: f64) -> f64 {
        let room = (self.millimetres-gap).max(0.);
        (4.*room/spacing.max(1e-12)).atan().to_degrees().min(Self::MOST_TURN)
    }

    /// The STL mesher's absolute chordal deflection: the share of the tolerance the sheet does
    /// not use.
    pub fn deflection(&self) -> f64 { self.millimetres*(1.-Self::FIT_SHARE) }

    /// The field-agreement probe, in millimetres: how far off each triangle a probe stands, the
    /// band a triangle's centroid must read within to withdraw a one-sided disagreement, and the
    /// value tolerance a probe's sign is read to. A mesh within the tolerance of the exact surface
    /// leaves a probe twice the tolerance off at least the tolerance clear of it on its own side,
    /// read to half the tolerance; a triangle within the tolerance of the surface is one lying
    /// on it. Never nearer than `LEAST_PROBE`.
    pub fn probe(&self) -> [f64;3] {
        let offset = (2.*self.millimetres).max(Self::LEAST_PROBE);
        [offset,offset/2.,offset/4.]
    }
}
