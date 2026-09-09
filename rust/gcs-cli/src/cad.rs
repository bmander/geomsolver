//! Optional native CAD process. The binary embeds its adapter so module paths
//! and the caller's current directory do not select different implementations.
use std::{io::Write,process::{Command,Stdio}};

pub fn step(sk: &gcs_core::model::Sketch,solid: usize,path: &str) -> Result<(),String> {
    let recipe = gcs_core::solid::cad::recipe(sk,solid)?;
    let python = std::env::var_os("SOLVENT_CAD_PYTHON").unwrap_or_else(|| "python3".into());
    let mut child = Command::new(python)
        .args(["-c",include_str!("../backend/occt.py"),path])
        .stdin(Stdio::piped()).stdout(Stdio::from(std::io::stderr())).stderr(Stdio::inherit())
        .spawn().map_err(|e| format!("cannot start CAD Python host: {e}; \
            set SOLVENT_CAD_PYTHON to an interpreter with cadquery-ocp installed"))?;
    let sent = child.stdin.take().ok_or("CAD host has no input")?
        .write_all(recipe.dump(None).as_bytes());
    let status = child.wait().map_err(|e| format!("cannot wait for CAD host: {e}"))?;
    if !status.success() { return Err(format!("CAD host failed ({status}); see backend diagnostic")); }
    sent.map_err(|e| format!("cannot send CAD construction data: {e}"))
}
