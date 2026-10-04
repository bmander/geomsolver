//! The Wankel project (`rust/examples/wankel`) at a configuration: the engine with numbers of its
//! `configuration` module replaced.
use gcs_core::program;
use std::path::PathBuf;

/// The project's directory.
pub fn project() -> PathBuf { PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../examples/wankel") }

/// The engine's entry document, `wankel.sv`.
pub fn source() -> String { std::fs::read_to_string(project().join("wankel.sv")).unwrap() }

/// The engine with its configuration's named numbers replaced (`drill::configure`'s rule).
pub fn read(numbers: &[(&str,&str)]) -> program::Elaborated { read_text(&source(),numbers) }

/// `text` in place of `wankel.sv` (an edit of it), its configuration's named numbers replaced.
pub fn read_text(text: &str,numbers: &[(&str,&str)]) -> program::Elaborated {
    crate::read_beside(text,&project(),&mut |name,text| crate::drill::configure(name,text,numbers))
}

/// The engine as configured: the 13B's rotor and housing.
pub fn standard() -> program::Elaborated { read(&[]) }
