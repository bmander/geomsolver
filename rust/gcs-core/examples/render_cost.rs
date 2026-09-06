//! Native projection costs, excluding parsing and solving. Pass example names to narrow the run.
//! cargo run --release -p gcs-core --example render_cost -- vtwin_throttle
use gcs_core::{examples, renderer, solid::ApproximationPolicy, solve};
use std::{collections::BTreeSet, hint::black_box, time::Instant};

fn ms(start: Instant) -> f64 {
    start.elapsed().as_secs_f64() * 1000.0
}
fn median(mut times: Vec<f64>) -> f64 {
    times.sort_by(f64::total_cmp);
    times[times.len() / 2]
}
fn main() {
    let mut names: Vec<String> = std::env::args().skip(1).collect();
    if names.is_empty() {
        names = examples::CASES.iter().map(|(_, name, _)| name.to_string()).collect();
    }
    let unit = 0.15;
    for name in names {
        let Some(mut sk) = examples::example(&name) else {
            eprintln!("{name}: no core example constructor; skipped");
            continue;
        };
        if sk.derived.is_empty() { continue; }
        assert!(solve::solve(&mut sk, Default::default()).success, "{name}: solve failed");
        let ids: BTreeSet<_> = sk.derived.iter().map(|v| v.solid as usize).collect();
        let (mut evaluation, mut preparation, mut projection) = (Vec::new(), Vec::new(), Vec::new());
        let mut strokes = 0;
        let mut stats = renderer::RenderStats::default();
        for _ in 0..3 {
            sk.solid_cache.borrow_mut().clear();
            let start = Instant::now();
            let solids: Vec<_> = ids.iter().map(|&i| {
                sk.evaluated_solid(i, ApproximationPolicy::View { unit }).unwrap()
            }).collect();
            evaluation.push(ms(start));
            let start = Instant::now();
            for solid in &solids { black_box(renderer::Renderer::prepare(solid)); }
            preparation.push(ms(start));
            let start = Instant::now();
            let (drawing, counts) = black_box(renderer::layout_with_stats(&sk, unit));
            stats = counts;
            projection.push(ms(start));
            strokes = drawing.len();
        }
        println!("{name}: evaluate {:.2} ms, prepare {:.2} ms, project {:.2} ms; {} views, {strokes} strokes",
            median(evaluation), median(preparation), median(projection), sk.derived.len());
        println!("  boundary candidates {}/{}, crossing candidates {}/{}",
            stats.boundary_candidates, stats.boundary_exhaustive, stats.crossing_candidates, stats.crossing_exhaustive);
    }
}
