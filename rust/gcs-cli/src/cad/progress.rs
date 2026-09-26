//! What an export says while it works: prose on stderr, and the stage keys a harness reads.
#![cfg_attr(not(feature="occt"),allow(dead_code))]
pub use gcs_core::solid::export::Stage;

/// Progress on stderr: a member takes minutes, and the JSON report owns stdout. Each line
/// carries the time since the first, so a whole export reads as one timeline.
static START: std::sync::OnceLock<std::time::Instant> = std::sync::OnceLock::new();
pub fn stage(message: &str) {
    let at = START.get_or_init(std::time::Instant::now).elapsed().as_secs_f64();
    eprintln!("solventc: [{at:7.1} s] {message}");
}

/// A stage completed, for a harness: with `SOLVENT_STAGE_TRACE` naming a file, one line
/// `key<TAB>seconds` is appended to it, the keys `Stage::key`'s in the order `Stage::ORDER`
/// completes them. Independent of the prose lines, which may change.
pub fn mark(stage: Stage) { trace(stage.key()); }

/// A stage refused, for the same harness: the line `refused:key<TAB>seconds`.
pub fn refused(stage: Stage) { trace(&format!("refused:{}",stage.key())); }

fn trace(key: &str) {
    let Ok(path) = std::env::var("SOLVENT_STAGE_TRACE") else { return };
    let at = START.get_or_init(std::time::Instant::now).elapsed().as_secs_f64();
    use std::io::Write;
    if let Ok(mut file) = std::fs::OpenOptions::new().create(true).append(true).open(path) {
        let _ = writeln!(file,"{key}\t{at:.3}");
    }
}
