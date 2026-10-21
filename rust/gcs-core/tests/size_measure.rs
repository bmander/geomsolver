//! Rows judged at their own sizes, measured across the corpus (#492, docs/iteration-limit-rescue-
//! plan.md): a tool, not a gate.  Every document `common::corpus` holds, solved from its own seeds
//! (`own_seeds`) and from `common::rough_starts` of its default solution (`rough_starts`), and the
//! belt of `unsettled.rs` seeded from −96 to ten kilometres out (`far_seeds`).  Each line is the
//! solve's outcome and its largest hard row over the drawing's extent (`extent`, what "solved" was
//! judged by before) and over its own size (`own`, what it is judged by now), with that row's
//! size and kind; `FLIP` marks a solve the two would judge differently.
//! `cargo test --manifest-path rust/Cargo.toml -p gcs-core --test core size_measure:: --
//! --ignored --nocapture --test-threads=1`, about two minutes.
use gcs_core::diagnose::{diagnose, DiagnoseOptions};
use gcs_core::model::Sketch;
use gcs_core::solve::{self, SolveOpts, SolveResult};
use gcs_core::system::System;

use crate::common::{at, corpus, read, rough_starts};
use crate::unsettled::belt;

fn report(name: &str, sk: &mut Sketch) -> SolveResult {
    let r = solve::solve(sk, SolveOpts::default());
    let mut sys = System::new(sk);
    let z = sys.z0(sk);
    let res = sys.residuals(&z);
    let (mut worst, mut own, mut extent) = (None, 0.0f64, 0.0f64);
    for row in sys.hard_rows() {
        extent = extent.max(res[row].abs());
        let j = sys.judged(row, res[row]);
        if worst.is_none() || j > own || j.is_nan() {
            (worst, own) = (Some(row), j);
        }
    }
    let (kind, size) = worst.map_or((String::new(), 0.0), |row| {
        let (b, i) = sys.instance_of(row);
        let c = sk.constraint(sys.blocks[b].cids[i]).map(|c| format!("{:?}", c.kind));
        (c.unwrap_or_default().chars().take(24).collect(), sys.row_size[row])
    });
    let flip = (extent < 1e-6) != (own < 1e-6);
    println!(
        "{name:40} {:22} st={} it={:3} ext={:9.3e} extent={extent:9.2e} own={own:9.2e} \
         size={size:9.3e} {kind}{}",
        r.outcome(), r.status, r.iterations, sk.extent(), if flip { " FLIP" } else { "" }
    );
    r
}

#[test]
#[ignore = "a measurement"]
fn own_seeds() {
    for mut d in corpus() {
        report(&d.name, &mut d.sketch);
    }
}

#[test]
#[ignore = "a measurement"]
fn rough_starts_of_the_corpus() {
    for d in corpus() {
        let mut solution = d.sketch.clone();
        let _ = solve::solve(&mut solution, SolveOpts::default());
        for (_, label, mut start) in rough_starts(&solution) {
            report(&format!("{}@{label}", d.name), &mut start);
        }
    }
}

#[test]
#[ignore = "a measurement"]
fn far_seeds() {
    for rb in [20.0, 20.001, 45.0] {
        for seed in [-96.0, -1e5, -1e6, -1e7] {
            let mut e = read(&belt(rb, Some(seed)));
            let r = report(&format!("belt r₂ {rb} from {seed}"), &mut e.sketch);
            let opts = DiagnoseOptions { settled: r.settled(), ..Default::default() };
            let d = diagnose(&mut e.sketch, opts);
            println!(
                "    x {:?}, theorem {:?}, {:?}, conflict {:?}",
                at(&e, "x"), d.claims_theorem, d.status, d.conflicts
            );
        }
    }
}
