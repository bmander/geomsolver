//! A solve that ran out of iterations, measured across the corpus (docs/iteration-limit-rescue-
//! plan.md, phase 1): a tool, not a gate.  Every document `block_measure` reads (the examples
//! that elaborate and the twelve spiral-bevel designs), started from its own seeds and from
//! `block_measure::robustness`'s starts (the default solution halved and doubled about its
//! centroid, and jittered three times by up to a thousandth of the extent), and solved by the
//! whole-system DogLeg alone — the first attempt of every one-shot solve, and the only one a
//! success ever came from before the rescue.  Every start's status is counted; for each that
//! *succeeded* on a status other than a clean stop (3, the trust region collapsed; 4, the
//! iteration limit), the line says how far the stop is from a tight solve continued from it
//! (`tight`, a fraction of the extent) and what each candidate rescue makes of it:
//!
//! * `resume` — one more whole-system DogLeg budget from the stop, no retry;
//! * `blocks@start` — the block pass and its polish from the start (`BlockMode::First`, no
//!   retry), which is what the rescue after a failure runs;
//! * `blocks@stop` — the same from the stop;
//! * `default` — the default solve from the start: the rule `solve_compiled` chose from the
//!   three (the pass from the stop, kept if it converges; else from the start, kept if it
//!   settles; else the stop's pass if it settled).
//!
//! The two block passes only with two blocks or more.  A rescue *settles* a start when it succeeds on a
//! status other than 4.  Beside each: how far it lands from the stop (a rescue that keeps the
//! stop's branch lands about as far as the stop is from its solution; one that jumps to another
//! root lands much further), and — for a determined document — from the own-seed solution (`ref`,
//! under 1e-6 back on that root).
use gcs_core::{model::Sketch, solve::{self, BlockMode, SolveOpts, SolveResult}};
use std::time::Instant;

use crate::block_measure::{corpus, jittered, off, scaled};

/// The tight continuation: the fixtures' accuracy, and budget enough never to stop on it.
fn tight() -> SolveOpts {
    SolveOpts { tol: 1e-16, acceptance_tol: 1e-12, max_iter: 5000, ..SolveOpts::default() }
}

/// The whole-system DogLeg alone.
fn plain() -> SolveOpts {
    SolveOpts { retry: false, blocks: BlockMode::Off, ..SolveOpts::default() }
}

/// The block pass and its polish alone.
fn blocks() -> SolveOpts {
    SolveOpts { retry: false, blocks: BlockMode::First, ..SolveOpts::default() }
}

/// One candidate rescue solved from `start`: the result, the pose and how long it took.
fn rescue(start: &Sketch, opts: SolveOpts) -> (SolveResult, Sketch, f64) {
    let mut sk = start.clone();
    let clock = Instant::now();
    let r = solve::solve(&mut sk, opts);
    (r, sk, clock.elapsed().as_secs_f64() * 1e3)
}

const CANDIDATES: [&str; 4] = ["resume", "blocks@start", "blocks@stop", "default"];

#[derive(Default)]
struct Tally {
    tried: usize,
    settles: usize,
    /// settled on a determined document, back on the own-seed solution
    on_reference: usize,
    /// settled more than a hundred times further from the stop than the stop is from `tight`,
    /// and more than 1e-6 of the extent: another root
    jumped: usize,
    ms: f64,
}

#[test]
#[ignore = "a tool: cargo test --manifest-path rust/Cargo.toml -p gcs-core --test core \
    limit_measure::limit_stops -- --ignored --nocapture"]
fn limit_stops() {
    let docs = corpus();
    const KINDS: [&str; 4] = ["own", "halved", "doubled", "jittered"];
    // per kind of start: starts, failed, and successes by status 0..=4
    let mut counts = [[0usize; 7]; 4];
    // per unclean status (3, 4): stops, on a determined document, worst distance from `tight`,
    // over 1e-9, over 1e-6, `tight` failing; and each candidate's tally
    let mut stops = [[0usize; 5]; 2];
    let mut worst = [0f64; 2];
    let mut tallies: [[Tally; 4]; 2] = Default::default();
    let mut own_limit = Vec::new();
    println!("doc\tstart\tstop\ttight\t{}", CANDIDATES.join("\t"));
    for d in &docs {
        let mut reference = d.sketch.clone();
        let ok = solve::solve(&mut reference, SolveOpts::default()).settled();
        let mut starts = vec![(0, "own".to_string(), d.sketch.clone())];
        if ok {
            starts.push((1, "x0.5".into(), scaled(&reference, 0.5)));
            starts.push((2, "x2".into(), scaled(&reference, 2.)));
            for seed in 1..=3 { starts.push((3, format!("jitter{seed}"), jittered(&reference, 0.001, seed))); }
        }
        let (n_blocks, determined) = {
            let mut sys = gcs_core::system::System::new(&d.sketch);
            let order = sys.block_order();
            (order.blocks.len(), order.under_cols.is_empty() && ok)
        };
        for (kind, label, start) in &starts {
            let mut sk = start.clone();
            let r = solve::solve(&mut sk, plain());
            let c = &mut counts[*kind];
            c[0] += 1;
            if !r.success { c[1] += 1; continue; }
            if (0..=4).contains(&r.status) { c[2 + r.status as usize] += 1; }
            if r.status < 3 { continue; }
            let u = (r.status - 3) as usize;
            let mut t = sk.clone();
            let rt = solve::solve(&mut t, tight());
            let dist = off(&sk, &t);
            stops[u][0] += 1;
            if determined { stops[u][1] += 1; }
            worst[u] = worst[u].max(dist);
            if dist > 1e-9 { stops[u][2] += 1; }
            if dist > 1e-6 { stops[u][3] += 1; }
            if !rt.success { stops[u][4] += 1; }
            let reference_off = |p: &Sketch| if determined { format!(" ref {:.1e}", off(p, &reference)) }
                else { String::new() };
            let mut line = format!("{}\t{label}\ts{} {:.1e}{}\t{dist:.1e}{}", d.name, r.status,
                r.max_residual, reference_off(&sk), if rt.success { "" } else { " (FAILED)" });
            for (k, name) in CANDIDATES.iter().enumerate() {
                let (from, opts) = match *name {
                    "resume" => (&sk, plain()),
                    "blocks@start" => (start, blocks()),
                    "blocks@stop" => (&sk, blocks()),
                    _ => (start, SolveOpts::default()),
                };
                if (k == 1 || k == 2) && n_blocks < 2 { line += "\t-"; continue; }
                let (rr, rsk, ms) = rescue(from, opts);
                let tally = &mut tallies[u][k];
                tally.tried += 1;
                tally.ms += ms;
                let from_stop = off(&rsk, &sk);
                if rr.settled() {
                    tally.settles += 1;
                    if determined && off(&rsk, &reference) <= 1e-6 { tally.on_reference += 1; }
                    if from_stop > 1e-6 && from_stop > 100. * dist { tally.jumped += 1; }
                }
                line += &format!("\t{} s{} {:.1e}, {from_stop:.1e} from stop{} ({ms:.1} ms)",
                    if rr.settled() { "settles" } else { "no" }, rr.status, rr.max_residual,
                    reference_off(&rsk));
            }
            if *kind == 0 { own_limit.push(format!("{} (s{}, {dist:.1e})", d.name, r.status)); }
            println!("{line}");
        }
    }
    println!("\nthe whole-system DogLeg alone:\nstart\tstarts\tfailed\tstatus0\tstatus1\tstatus2\tstatus3\tstatus4");
    for (k, c) in counts.iter().enumerate() {
        println!("{}\t{}", KINDS[k], c.iter().map(|v| v.to_string()).collect::<Vec<_>>().join("\t"));
    }
    for u in 0..2 {
        let s = stops[u];
        println!("status {} successes: {} ({} determined); from tight: worst {:.1e}, > 1e-9: {}, > 1e-6: {}; \
            tight fails {}", u + 3, s[0], s[1], worst[u], s[2], s[3], s[4]);
        for (k, name) in CANDIDATES.iter().enumerate() {
            let t = &tallies[u][k];
            println!("  {name}: tried {}, settles {}, on the reference {}, jumped {}, {:.0} ms", t.tried,
                t.settles, t.on_reference, t.jumped, t.ms);
        }
    }
    println!("own seeds stopping unclean: {}", own_limit.join(", "));
}
