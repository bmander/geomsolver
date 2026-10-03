//! The twist-drill project (`rust/examples/twist_drill`) at a configuration: the drill with
//! numbers of its `configuration` module replaced, and the readers.
use gcs_core::program;
use std::path::PathBuf;

/// The project's directory.
pub fn project() -> PathBuf { PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../examples/twist_drill") }

/// The drill's entry document, `drill.sv`.
pub fn source() -> String { std::fs::read_to_string(project().join("drill.sv")).unwrap() }

/// The `configuration` module with each named number's line replaced by `name := value` (the value
/// as written, unit and all); every other module unchanged.
pub fn configure(name: &str,text: String,numbers: &[(&str,&str)]) -> String {
    if name != "configuration" { return text; }
    let mut out = String::new();
    for line in text.lines() {
        let named = numbers.iter().find(|(n,_)| line.strip_prefix(n).is_some_and(|rest| rest.starts_with([' ',':'])));
        match named {
            Some((n,v)) => out += &format!("{n} := {v}\n"),
            None => { out += line; out.push('\n'); }
        }
    }
    for (n,_) in numbers { assert!(out.contains(&format!("{n} := ")),"the configuration states no `{n}`"); }
    out
}

/// The drill with its configuration's named numbers replaced.
pub fn read(numbers: &[(&str,&str)]) -> program::Elaborated {
    crate::read_beside(&source(),&project(),&mut |name,text| configure(name,text,numbers))
}

/// The drill as configured: two flutes 40 mm long, the point ground, the shank added.
pub fn standard() -> program::Elaborated { read(&[]) }

/// A drill one flute and `length` long, its end left square: what a test exports in seconds.
pub fn reduced(length: &str) -> program::Elaborated { read(&[("flutes","1"),("fluted_length",length),("point","0")]) }
