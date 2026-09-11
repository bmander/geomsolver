//! The traced sheets a construction starts from: the tracer's candidate
//! sheets over exactly the declared interval, no overrun, so the first and
//! last columns are the contact curves at the two end poses.
use super::caps::Cap;
use crate::model::Sketch;
use crate::solid::{SweepContacts,SweepPatch};

/// A seed of the construction: a traced sheet, or a cap cut from the tool at an end of the roll.
#[derive(Clone,Debug)]
pub enum Seed { Traced(SweepPatch), Cap(Cap) }

impl Seed {
    pub fn patch(&self) -> &SweepPatch {
        match self { Seed::Traced(p) => p,Seed::Cap(c) => &c.patch }
    }
    /// A cap's vertices on an edge of the tool; none for a traced sheet, whose vertex labels
    /// decide.
    pub fn tool_edges(&self) -> Option<&[bool]> {
        match self { Seed::Traced(_) => None,Seed::Cap(c) => Some(&c.tool_edges) }
    }
}

/// Read the sweep's tool and motion and trace its candidate sheets.
pub fn seeds(sk: &Sketch,swept: usize,spacing: f64,sagitta: f64,progress: &dyn Fn(&str))
    -> Result<(SweepContacts,Vec<SweepPatch>),String> {
    let sweep = SweepContacts::read(sk,swept,1e-10)?;
    let sheets = sweep.characteristic_sheets(spacing,sagitta,0.,1e-9,None,progress)?;
    Ok((sweep,sheets))
}
