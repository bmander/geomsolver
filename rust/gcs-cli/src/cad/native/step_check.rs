//! How fully an export's STEP file is verified. The check itself — the file parsed and held to the
//! solid it was written from, entity for entity — is the core's (`gcs_core::brep::step_check`), so
//! the browser's export makes it too; what is this binary's is whether a file is also read back by
//! the native kernel (`--verify-step full`).
#[allow(unused_imports)]
pub(crate) use gcs_core::brep::step_check::*;

/// How a written STEP file is verified: its text against the solid (`verify`), or that and the
/// kernel's own read-back as well (`solvent_cad_step`'s `full`).
#[derive(Clone,Copy,Debug,PartialEq)]
pub(crate) enum Verification { Light,Full }

static FULL: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);

/// Ask for the full verification (`--verify-step full`) of every STEP file this process writes.
#[allow(dead_code)]
pub(crate) fn ask_full() { FULL.store(true,std::sync::atomic::Ordering::Relaxed); }

/// How files are verified: in full where `ask_full` or `SOLVENT_STEP_VERIFY=full` asks, else light.
pub(crate) fn verification() -> Verification {
    if FULL.load(std::sync::atomic::Ordering::Relaxed) || std::env::var("SOLVENT_STEP_VERIFY").is_ok_and(|v| v == "full") {
        Verification::Full
    } else { Verification::Light }
}
