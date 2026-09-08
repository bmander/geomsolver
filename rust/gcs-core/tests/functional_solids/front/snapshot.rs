//! Opt-in snapshots of candidate geometry, including incomplete growth.
//! Saving a snapshot neither accepts a boundary nor suppresses a test failure.
use super::*;

pub(super) fn grow(name:&str,front:&mut Front,surface:&mut Surface,start:std::time::Instant) {
    let Some(output) = std::env::var_os("SOLVENT_FRONT_SNAPSHOT") else {
        front.grow(surface); return;
    };
    // Keep the last retained triangles when a work budget panics. Resume the
    // original panic after writing; ordinary closure/accuracy checks still run
    // when growth returns. This is a diagnostic path, not error recovery.
    let failure = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| front.grow(surface))).err();
    let elapsed = start.elapsed().as_secs_f64();
    let status = if failure.is_some() { "growth_panicked" }
        else if front.boundary.is_empty() { "unchecked_candidate" } else { "open_front" };
    let output = std::path::PathBuf::from(output);
    std::fs::create_dir_all(&output).unwrap();
    let path = output.join(format!("{name}.json"));
    let json = format!(
        "{{\"status\":{status:?},\"accuracy\":{},\"seconds\":{elapsed},\"queries\":{},\"repairs\":{},\"vertices\":{:?},\"triangles\":{:?},\"boundary\":{:?}}}\n",
        surface.accuracy,surface.queries,front.repairs,
        front.vertices.iter().map(|v| v.p).collect::<Vec<_>>(),front.triangles,
        front.boundary.iter().copied().collect::<Vec<_>>());
    std::fs::write(&path,json).unwrap();
    eprintln!("diagnostic snapshot {name}: {status}, {} triangles, {} open edges, {} queries, {elapsed:.3}s; {}",
        front.triangles.len(),front.boundary.len(),surface.queries,path.display());
    if let Some(failure) = failure { std::panic::resume_unwind(failure); }
}
