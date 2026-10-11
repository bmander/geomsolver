//! The material fields read in millimetres, the unit the B-rep kernel builds in. A field reads
//! the document's own length units; a swept body's sheets, sides and cells are millimetres. The
//! one place the two meet is here: a point asked of a `Millimetres` field is divided by the
//! document's millimetres per unit on the way in, and a value (a length) multiplied by it on the
//! way out, so the field stays one-Lipschitz in the unit it is asked in. Its constructors read
//! that scale from the sketch themselves, and the export's stages take this type and not the
//! fields it wraps, so a millimetre point cannot reach a field in model units unconverted.
use super::{MaterialEvaluator,MaterialField,ProbeState,SpatialField,point};
use crate::interval::minimum::Options;
use crate::model::Sketch;

/// A field asked in millimetres: its points, its values, a probe's radius and tolerance.
#[derive(Clone,Debug)]
pub struct Millimetres<F> { field: F,scale: f64 }

impl<F> Millimetres<F> {
    fn over(sk: &Sketch,field: F) -> Result<Self,String> {
        Ok(Millimetres {field,scale:crate::solid::cad::millimetres(sk)?})
    }
    /// Millimetres in a document unit: what the export's motions and placements are read by.
    pub fn scale(&self) -> f64 { self.scale }
    /// The field itself, asked in the document's units: only for what is read in them (a motion's
    /// extent, a check whose points are already the document's).
    pub fn in_model_units(&self) -> &F { &self.field }
}

impl Millimetres<SpatialField> {
    /// `solid`'s static field (`SpatialField::read`), asked in millimetres.
    pub fn read(sk: &Sketch,solid: usize,axis_tolerance: f64) -> Result<Self,String> {
        Millimetres::over(sk,SpatialField::read(sk,solid,axis_tolerance)?)
    }
    /// What remains of `root` without its swept cuts (`admission::static_remainder`), asked in
    /// millimetres, and the cuts.
    pub fn static_remainder(sk: &Sketch,root: usize,axis_tolerance: f64) -> Result<(Self,Vec<crate::solid::cad::SweptCut>),String> {
        let (field,cuts) = crate::solid::admission::static_remainder(sk,root,axis_tolerance)?;
        Ok((Millimetres::over(sk,field)?,cuts))
    }
    /// The field at `p` (mm), in mm: negative inside.
    pub fn value(&self,p: [f64;3]) -> f64 { self.field.value(p.map(|x| x/self.scale))*self.scale }
}

impl Millimetres<MaterialField> {
    /// `solid`'s whole material field (`MaterialField::read`), asked in millimetres.
    pub fn read(sk: &Sketch,solid: usize,axis_tolerance: f64) -> Result<Self,String> {
        Millimetres::over(sk,MaterialField::read(sk,solid,axis_tolerance)?)
    }
    /// An evaluator asked in millimetres (`MaterialField::evaluator`).
    pub fn evaluator(&self,max_cached_poses_per_sweep: usize) -> MillimetreEvaluator {
        MillimetreEvaluator {inner:self.field.evaluator(max_cached_poses_per_sweep),scale:self.scale}
    }
}

/// A material evaluator asked in millimetres.
pub struct MillimetreEvaluator { inner: MaterialEvaluator,scale: f64 }

impl MillimetreEvaluator {
    /// Whether the ball of `radius` about `p` (both mm) is all material, all removed or neither:
    /// `MaterialEvaluator::probe`'s state, its value read to a quarter of the radius within 40000
    /// evaluations. (Its bounds are left behind: they are in the document's units.)
    pub fn ball(&mut self,p: [f64;3],radius: f64) -> Result<ProbeState,String> {
        let s = self.scale;
        let at = point(p.map(|x| x/s)).map_err(|e| format!("{e:?}"))?;
        let options = Options {value_tolerance:radius/s/4.,max_evaluations:40000};
        self.inner.probe(at,[1.,0.,0.],radius/s,options).map(|probe| probe.state).map_err(|e| format!("{e:?}"))
    }
}
