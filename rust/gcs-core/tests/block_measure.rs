//! The block-triangular solve measured across the corpus (docs/block-triangular-solve-plan.md,
//! phase 4): a tool, not a gate.  Every example document (`rust/examples/**/*.sv`) and the twelve
//! spiral-bevel designs `tests/hypoid_layout.rs` records, each solved as solventc solves it
//! (`SolveOpts::default()`, one `System` compiled per solve) with `BlockMode::Off`, `Rescue` and
//! `First`:
//!
//! * `timing` — the median of seven solves from the document's own seeds, per mode;
//! * `robustness` — from the default solve's pose scaled about its centroid (by 0.5 and by 2) and
//!   jittered (three seeded draws of up to a thousandth of the extent), how many starts each
//!   mode solves, and — for a document with no freedom left, where the pose is determined — how
//!   many of those land on the reference pose;
//! * `blast_radius` — which documents' solved coordinates `First` changes, bit for bit, against
//!   `Off` from the document's own seeds: what making `First` the default would re-record.
//!
//! Each test's `ignore` is its command; `timing` runs on one thread, its clocks being wall-clock.
use gcs_core::solve::{self, BlockMode, SolveOpts};
use std::time::Instant;

use crate::common::{
    apart, bits, blocks_of, corpus, is_determined, off, rough_starts, with_blocks, ROUGH,
};

const MODES: [BlockMode; 3] = [BlockMode::Off, BlockMode::Rescue, BlockMode::First];

fn median(mut v: Vec<f64>) -> f64 {
    v.sort_by(|a, b| a.partial_cmp(b).unwrap());
    v[v.len() / 2]
}

#[test]
#[ignore = "a tool: cargo test --manifest-path rust/Cargo.toml -p gcs-core --test core \
    block_measure::timing -- --ignored --nocapture --test-threads 1"]
fn timing() {
    const REPS: usize = 7;
    let docs = corpus();
    let mut total = [0.0; 3];
    println!("doc\tparams\tblocks\toff_ms\trescue_ms\tfirst_ms\toff\trescue\tfirst");
    for d in &docs {
        let mut line = format!("{}\t{}\t{}", d.name, d.sketch.params.iter().filter(|p| !p.fixed).count(),
            blocks_of(&d.sketch));
        let mut paths = String::new();
        for (k, mode) in MODES.iter().enumerate() {
            let mut times = Vec::new();
            let mut last = None;
            for _ in 0..REPS {
                let mut sk = d.sketch.clone();
                let clock = Instant::now();
                let r = solve::solve(&mut sk, with_blocks(*mode));
                times.push(clock.elapsed().as_secs_f64() * 1e3);
                last = Some(r);
            }
            let m = median(times);
            total[k] += m;
            line += &format!("\t{m:.3}");
            let r = last.unwrap();
            paths += &format!("\t{}{}", if r.success { "" } else { "FAILED " }, r.method);
        }
        println!("{line}{paths}");
    }
    println!("TOTAL\t\t\t{:.1}\t{:.1}\t{:.1}", total[0], total[1], total[2]);
}

#[test]
#[ignore = "a tool: cargo test --manifest-path rust/Cargo.toml -p gcs-core --test core \
    block_measure::robustness -- --ignored --nocapture"]
fn robustness() {
    let docs = corpus();
    // per mode, over every document and over the determined ones: starts, solved, and (for the
    // determined) solved on the reference root
    let mut total = [[0usize; 3]; 3];
    let mut determined = [[0usize; 3]; 3];
    // determined documents, per mode and kind of start (halved, doubled, jittered): solved, same
    let mut kinds = [[[0usize; 2]; 3]; 3];
    println!("doc\tdetermined\tstarts\toff solved/same\trescue solved/same\tfirst solved/same");
    for d in &docs {
        let mut reference = d.sketch.clone();
        if !solve::solve(&mut reference, SolveOpts::default()).success {
            println!("{}\tno reference: the default solve fails", d.name);
            continue;
        }
        let fixed = is_determined(&reference);
        let starts = rough_starts(&reference);
        let mut line = format!("{}\t{}\t{}", d.name, if fixed { "yes" } else { "no" }, starts.len());
        for (k, mode) in MODES.iter().enumerate() {
            let (mut solved, mut same) = (0, 0);
            for (j, _, s) in &starts {
                let mut sk = s.clone();
                if solve::solve(&mut sk, with_blocks(*mode)).success {
                    solved += 1;
                    let on = off(&sk, &reference) <= 1e-6;
                    if on { same += 1; }
                    if fixed {
                        kinds[k][*j][0] += 1;
                        if on { kinds[k][*j][1] += 1; }
                    }
                }
            }
            total[k][0] += starts.len();
            total[k][1] += solved;
            if fixed {
                determined[k][0] += starts.len();
                determined[k][1] += solved;
                determined[k][2] += same;
                line += &format!("\t{solved}/{same}");
            } else {
                line += &format!("\t{solved}/-");
            }
        }
        println!("{line}");
    }
    for (k, mode) in MODES.iter().enumerate() {
        println!("TOTAL {mode:?}: {} starts, {} solved; determined documents: {} starts, {} solved, \
            {} on the reference root", total[k][0], total[k][1], determined[k][0], determined[k][1],
            determined[k][2]);
        for (j, kind) in ROUGH.iter().enumerate() {
            println!("  {kind}: {} solved, {} on the reference root", kinds[k][j][0], kinds[k][j][1]);
        }
    }
}

#[test]
#[ignore = "a tool: cargo test --manifest-path rust/Cargo.toml -p gcs-core --test core \
    block_measure::blast_radius -- --ignored --nocapture"]
fn blast_radius() {
    let docs = corpus();
    let (mut same, mut changed) = (0, Vec::new());
    for d in &docs {
        let mut off_sk = d.sketch.clone();
        let r_off = solve::solve(&mut off_sk, with_blocks(BlockMode::Off));
        let mut first_sk = d.sketch.clone();
        let r_first = solve::solve(&mut first_sk, with_blocks(BlockMode::First));
        let (a, b) = (bits(&off_sk), bits(&first_sk));
        let differ = a.iter().zip(&b).filter(|(x, y)| x != y).count();
        if differ == 0 {
            same += 1;
        } else {
            let worst = apart(&first_sk, &off_sk);
            changed.push(format!("{}\t{} of {} params differ, worst {worst:.1e} of the extent; \
                Off {} by {}, First {} by {}, {} blocks", d.name, differ, a.len(),
                if r_off.success { "solved" } else { "FAILED" }, r_off.method,
                if r_first.success { "solved" } else { "FAILED" }, r_first.method, blocks_of(&d.sketch)));
        }
    }
    for c in &changed { println!("{c}"); }
    println!("TOTAL {} documents: {} identical, {} changed", docs.len(), same, changed.len());
}

