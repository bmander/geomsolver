//! Why an export was refused, in one vocabulary whichever backend refused it: the kernel's
//! (`cad_kernel::sweep::refusal`), and an admission's refusal said as one.
#[allow(unused_imports)]
use crate::fmath::Det;
use super::admission;
pub use cad_kernel::sweep::refusal::*;

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
