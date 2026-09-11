//! The traced sheets a construction starts from: the tracer's candidate
//! sheets over exactly the declared interval, no overrun, so the first and
//! last columns are the contact curves at the two end poses.
use crate::model::Sketch;
use crate::solid::{SweepContacts,SweepPatch};

/// Read the sweep's tool and motion and trace its candidate sheets.
pub fn seeds(sk: &Sketch,swept: usize,spacing: f64,sagitta: f64,progress: &dyn Fn(&str))
    -> Result<(SweepContacts,Vec<SweepPatch>),String> {
    let sweep = SweepContacts::read(sk,swept,1e-10)?;
    let sheets = sweep.characteristic_sheets(spacing,sagitta,0.,1e-9,None,progress)?;
    Ok((sweep,sheets))
}
