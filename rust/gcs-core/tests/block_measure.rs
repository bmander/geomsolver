//! The block-triangular solve measured across the corpus (docs/block-triangular-solve-plan.md,
//! phase 4): a tool, not a gate.  Every example document (`rust/examples/**/*.sv`) and the twelve
//! spiral-bevel designs `tests/hypoid_layout.rs` records, each solved as solventc solves it
//! (`SolveOpts::default()`, one `System` compiled per solve) with `BlockMode::Off`, `Rescue` and
//! `First`:
//!
//! * `timing` — the median of several solves from the document's own seeds, per mode;
//! * `robustness` — from the default solve's pose scaled about its centroid (by 0.5 and by 2) and
//!   jittered (three seeded draws of up to a thousandth of the extent), how many starts each
//!   mode solves, and — for a document with no freedom left, where the pose is determined — how
//!   many of those land on the reference pose;
//! * `blast_radius` — which documents' solved coordinates `First` changes, bit for bit, against
//!   `Off` from the document's own seeds: what making `First` the default would re-record.
//!
//! `cargo test --manifest-path rust/Cargo.toml -p gcs-core --test core block_measure -- --ignored
//! --nocapture --test-threads 1` (one thread: the timings are wall-clock).
use gcs_core::{library, model::Sketch, modules, program, rng::Rng, solve::{self, BlockMode, SolveOpts}, syntax};
use std::path::{Path, PathBuf};
use std::time::Instant;

const MODES: [BlockMode; 3] = [BlockMode::Off, BlockMode::Rescue, BlockMode::First];

struct Doc {
    name: String,
    sketch: Sketch,
}

fn walk(dir: &Path, out: &mut Vec<PathBuf>) {
    for entry in std::fs::read_dir(dir).unwrap() {
        let path = entry.unwrap().path();
        if path.is_dir() {
            walk(&path, out);
        } else if path.extension().is_some_and(|e| e == "sv") {
            out.push(path);
        }
    }
}

/// Every example document that elaborates, as solventc reads it (modules beside it, then in its
/// ancestors, then the library), and the spiral-bevel designs.
fn corpus() -> Vec<Doc> {
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../examples");
    let mut files = Vec::new();
    walk(&root, &mut files);
    files.sort();
    let mut docs = Vec::new();
    for f in files {
        let name = f.strip_prefix(&root).unwrap().to_string_lossy().to_string();
        let text = std::fs::read_to_string(&f).unwrap();
        let (mut p, errs) = syntax::parse(&text);
        let dir = f.parent().unwrap().to_path_buf();
        let mut resolve = |m: &str| -> Option<String> {
            modules::search_paths(m, &name).iter()
                .find_map(|rel| std::fs::read_to_string(dir.join(rel)).ok())
                .or_else(|| library::resolve(m))
        };
        let _ = modules::link(&mut p, &mut resolve);
        let e = program::elaborate(&p);
        if !errs.is_empty() || !e.ok() {
            println!("skip {name}: does not elaborate");
            continue;
        }
        docs.push(Doc { name, sketch: e.sketch });
    }
    let project = fixtures::gear::project();
    let designs: Vec<(String, String)> = {
        let text = std::fs::read_to_string(project.join("configuration.sv")).unwrap();
        let mut all = vec![
            ("configured".to_string(), fixtures::gear::design("configuration", text.clone(), 25., 12.5, 25.)),
            ("bevel".to_string(), fixtures::gear::bevel("configuration", text.clone())),
            ("hypoid6".to_string(), fixtures::gear::hypoid6("configuration", text)),
        ];
        for teeth in [[24, 48], [32, 32], [28, 49]] {
            for module in [0.2, 2., 25.4] {
                all.push((format!("{}x{} m{module}", teeth[0], teeth[1]),
                    fixtures::gear::configuration(teeth, module, 0., 0., 35.)));
            }
        }
        all
    };
    for (label, configuration) in designs {
        let e = fixtures::unsolved(&fixtures::gear::source(), &mut fixtures::beside(&project,
            &mut |name, text| if name == "configuration" { configuration.clone() } else { text }));
        docs.push(Doc { name: format!("spiral_bevel@{label}"), sketch: e.sketch });
    }
    docs
}

fn opts(blocks: BlockMode) -> SolveOpts {
    SolveOpts { blocks, ..Default::default() }
}

fn median(mut v: Vec<f64>) -> f64 {
    v.sort_by(|a, b| a.partial_cmp(b).unwrap());
    v[v.len() / 2]
}

fn blocks_of(sk: &Sketch) -> usize {
    gcs_core::system::System::new(sk).block_order().blocks.len()
}

#[test]
#[ignore = "a tool: times the corpus under each BlockMode"]
fn timing() {
    let reps: usize = std::env::var("REPS").ok().and_then(|r| r.parse().ok()).unwrap_or(7);
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
            for _ in 0..reps {
                let mut sk = d.sketch.clone();
                let clock = Instant::now();
                let r = solve::solve(&mut sk, opts(*mode));
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

/// Scale every free point coordinate about the free points' centroid by `f`, and every free
/// radius by `f`.
fn scaled(sk: &Sketch, f: f64) -> Sketch {
    let mut out = sk.clone();
    let free: Vec<(u32, u32)> = sk.points.iter().map(|p| (p.x, p.y))
        .filter(|&(x, y)| !sk.params[x as usize].fixed || !sk.params[y as usize].fixed).collect();
    if free.is_empty() { return out; }
    let n = free.len() as f64;
    let cx = free.iter().map(|&(x, _)| sk.params[x as usize].value).sum::<f64>() / n;
    let cy = free.iter().map(|&(_, y)| sk.params[y as usize].value).sum::<f64>() / n;
    for &(x, y) in &free {
        for (i, c) in [(x, cx), (y, cy)] {
            let p = &mut out.params[i as usize];
            if !p.fixed { p.value = c + f * (p.value - c); }
        }
    }
    for r in sk.circles.iter().map(|c| c.radius).chain(sk.arcs.iter().map(|a| a.radius)) {
        let p = &mut out.params[r as usize];
        if !p.fixed { p.value *= f; }
    }
    out
}

/// Jitter every free point coordinate by up to `amount` of the extent, and every free radius by
/// up to `amount` of itself.
fn jittered(sk: &Sketch, amount: f64, seed: u32) -> Sketch {
    let mut out = sk.clone();
    let mut rng = Rng::new(seed);
    let e = sk.extent();
    for p in &sk.points {
        for i in [p.x, p.y] {
            let q = &mut out.params[i as usize];
            if !q.fixed { q.value += rng.uniform(-amount, amount) * e; }
        }
    }
    for r in sk.circles.iter().map(|c| c.radius).chain(sk.arcs.iter().map(|a| a.radius)) {
        let q = &mut out.params[r as usize];
        if !q.fixed { q.value *= 1. + rng.uniform(-amount, amount); }
    }
    out
}

/// How far `sk`'s points (in space, so that a view turned over with its drawing is the same
/// pose) and radii are from `reference`'s, over the reference's extent.
fn off(sk: &Sketch, reference: &Sketch) -> f64 {
    let e = reference.extent();
    let points = (0..reference.points.len()).map(|i| {
        let (a, b) = (sk.world_point(i), reference.world_point(i));
        (0..3).map(|k| (a[k] - b[k]).abs()).fold(0., f64::max)
    });
    let radii = reference.circles.iter().map(|c| c.radius).chain(reference.arcs.iter().map(|a| a.radius))
        .map(|i| (sk.params[i as usize].value - reference.params[i as usize].value).abs());
    points.chain(radii).fold(0., f64::max) / e
}

#[test]
#[ignore = "a tool: solves the corpus from perturbed starts under each BlockMode"]
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
        let fixed = {
            let mut sys = gcs_core::system::System::new(&reference);
            let order = sys.block_order();
            order.under_cols.is_empty()
        };
        let mut starts = vec![scaled(&reference, 0.5), scaled(&reference, 2.)];
        for seed in 1..=3 { starts.push(jittered(&reference, 0.001, seed)); }
        let mut line = format!("{}\t{}\t{}", d.name, if fixed { "yes" } else { "no" }, starts.len());
        for (k, mode) in MODES.iter().enumerate() {
            let (mut solved, mut same) = (0, 0);
            for (j, s) in starts.iter().enumerate() {
                let mut sk = s.clone();
                if solve::solve(&mut sk, opts(*mode)).success {
                    solved += 1;
                    let on = off(&sk, &reference) <= 1e-6;
                    if on { same += 1; }
                    if fixed {
                        kinds[k][j.min(2)][0] += 1;
                        if on { kinds[k][j.min(2)][1] += 1; }
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
        for (j, kind) in ["halved", "doubled", "jittered"].iter().enumerate() {
            println!("  {kind}: {} solved, {} on the reference root", kinds[k][j][0], kinds[k][j][1]);
        }
    }
}

#[test]
#[ignore = "a tool: which documents First would change, bit for bit"]
fn blast_radius() {
    let docs = corpus();
    let (mut same, mut changed) = (0, Vec::new());
    for d in &docs {
        let mut off_sk = d.sketch.clone();
        let r_off = solve::solve(&mut off_sk, opts(BlockMode::Off));
        let mut first_sk = d.sketch.clone();
        let r_first = solve::solve(&mut first_sk, opts(BlockMode::First));
        let (a, b) = (off_sk.get_x(), first_sk.get_x());
        let bits = a.iter().zip(&b).filter(|(x, y)| x.to_bits() != y.to_bits()).count();
        if bits == 0 {
            same += 1;
        } else {
            let worst = a.iter().zip(&b).map(|(x, y)| (x - y).abs()).fold(0., f64::max) / off_sk.extent();
            changed.push(format!("{}\t{} of {} params differ, worst {worst:.1e} of the extent; \
                Off {} by {}, First {} by {}, {} blocks", d.name, bits, a.len(),
                if r_off.success { "solved" } else { "FAILED" }, r_off.method,
                if r_first.success { "solved" } else { "FAILED" }, r_first.method, blocks_of(&d.sketch)));
        }
    }
    for c in &changed { println!("{c}"); }
    println!("TOTAL {} documents: {} identical, {} changed", docs.len(), same, changed.len());
}

