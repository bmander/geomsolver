//! Benchmark: solve times, plan replay and drag frame rates — the native build.
//!
//!     cargo run --release -p gcs-core --bin bench
//!
//! Solve times are for a *compiled* `System` (what dragging pays per frame) and for
//! compile + solve (one-shot).  `npm run bench` measures the same things through wasm; this is
//! the native half, and the two are meant to be read side by side.
//!
//! Wall-clock medians over repetitions, and nothing else: the core has no dependencies and a
//! benchmark harness would be the first.  Kill background CPU hogs before trusting the numbers —
//! `uptime` first.

use std::time::Instant;

use gcs_core::decompose::{PlanDrag, PlanSolver};
use gcs_core::examples::{self, truss, truss_floating, zigzag};
use gcs_core::model::Sketch;
use gcs_core::newton::Method;
use gcs_core::solve::{Drag, SolveOpts};
use gcs_core::system::System;

const METHODS: [Method; 2] = [Method::DogLeg, Method::Lm];

fn median(mut v: Vec<f64>) -> f64 {
    if v.is_empty() {
        return f64::NAN;
    }
    v.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let n = v.len();
    if n % 2 == 1 {
        v[n / 2]
    } else {
        0.5 * (v[n / 2 - 1] + v[n / 2])
    }
}

fn ms(t: Instant) -> f64 {
    t.elapsed().as_secs_f64() * 1e3
}

/// (median compiled-solve ms, all ok, mean iterations)
fn bench_solve(make: &dyn Fn() -> Sketch, method: Method, reps: u32) -> (f64, bool, f64) {
    let (mut ts, mut ok, mut its) = (Vec::new(), true, 0.0);
    for i in 0..reps {
        let mut sk = make();
        examples::jitter(&mut sk, 1.0, i + 1);
        let mut s = System::new(&sk);
        let t0 = Instant::now();
        let r = s.solve(&mut sk, SolveOpts { method, ..SolveOpts::default() });
        ts.push(ms(t0));
        ok &= r.success;
        its += r.iterations as f64;
    }
    (median(ts), ok, its / reps as f64)
}

/// (median compile ms, free columns, residual rows) — the dimensions come off the last system
/// it built, so nothing compiles a `System` twice to print two integers.
fn bench_compile(sk: &Sketch, reps: u32) -> (f64, usize, usize) {
    let mut ts = Vec::new();
    let (mut free, mut res) = (0, 0);
    for _ in 0..reps {
        let t0 = Instant::now();
        let s = System::new(sk);
        ts.push(ms(t0));
        (free, res) = (s.n_free, s.n_res);
    }
    (median(ts), free, res)
}

fn bench_drag(mut sk: Sketch, frames: u32) -> (f64, bool) {
    let p = sk.points.len() / 2;
    let (x, y) = sk.point_xy(p);
    let mut d = Drag::new(&mut sk, p, x, y, Method::DogLeg, 1.0, Vec::new(), 0.05);
    let mut ts = Vec::new();
    let mut ok = false;
    // The cursor has to keep moving.  Held at one target the drag reaches it on the first frame
    // and the rest time a solve with nothing to do — which is what printed 30,000 fps.
    for i in 0..frames {
        let a = 0.3 * i as f64;
        let t0 = Instant::now();
        let r = d.move_to(&mut sk, x + 1.0 + a.cos(), y + 0.5 + a.sin());
        ts.push(ms(t0));
        ok = r.success;
    }
    d.end(&mut sk);
    (median(ts), ok)
}

/// (drag start ms, median frame ms) of the app's drag on `point` — on the sketch's cached plan
/// when given one, as the app drags, else with a plan of its own.
fn bench_plan_drag(sk: &mut Sketch, point: usize, frames: u32, own: bool) -> (f64, f64) {
    let (x, y) = sk.point_xy(point);
    // The cached plan is the app's: decomposed and solved by the edit before the gesture, so
    // neither cost lands in `start`.  Its absence is exactly what the `own` column measures.
    let mut plan = (!own).then(|| {
        let mut p = PlanSolver::new(sk, true);
        p.solve(sk, 1e-9, true, Method::DogLeg);
        p
    });
    let t0 = Instant::now();
    let mut d = match plan.as_mut() {
        Some(p) => PlanDrag::on(sk, p, point, x, y, None, 0.05),
        None => PlanDrag::new(sk, point, x, y, None, 0.05),
    };
    let start = ms(t0);
    let cached = plan.as_ref().map(|p| &p.plan);
    let mut ts = Vec::new();
    for i in 0..frames {
        let a = 0.3 * i as f64;
        let t0 = Instant::now();
        d.move_to(sk, cached, x + 2.0 * a.cos(), y + 2.0 * a.sin());
        ts.push(ms(t0));
    }
    d.end();
    (start, median(ts))
}

type Case = (&'static str, Box<dyn Fn() -> Sketch>);

fn cases() -> Vec<Case> {
    let mut v: Vec<Case> = Vec::new();
    for name in examples::EXAMPLES {
        v.push((name, Box::new(move || examples::example(name).expect("example"))));
    }
    v.push(("truss_50", Box::new(|| truss(50, 20.0, 15.0, true))));
    v.push(("truss_100", Box::new(|| truss(100, 20.0, 15.0, true))));
    v
}

fn main() {
    let cases = cases();

    println!("== solve (jittered warm start): compiled-solve ms / iterations ==");
    print!("{:<16}{:>5}{:>5} |", "sketch", "free", "res");
    for m in METHODS {
        print!(" {:^24} |", m.as_str());
    }
    println!(" compile");
    for (name, make) in &cases {
        let sk = make();
        let reps = if sk.params.len() > 100 { 5 } else { 20 };
        let mut cells = Vec::new();
        for m in METHODS {
            let (t, ok, it) = bench_solve(make.as_ref(), m, reps);
            cells.push(format!("{t:8.2} ms {:3} it={it:4.1}", if ok { "" } else { "BAD" }));
        }
        let (tc, free, res) = bench_compile(&sk, reps);
        print!("{name:<16}{free:>5}{res:>5} |");
        for c in cells {
            print!(" {c} |");
        }
        println!(" {tc:5.2} ms");
    }

    println!("\n== decomposition plan: compile once, replay per solve ==");
    for (name, make) in &cases {
        let mut sk = make();
        let t0 = Instant::now();
        let mut ps = PlanSolver::new(&sk, false);
        let tc = ms(t0);
        let mut te = Vec::new();
        for i in 0..5 {
            examples::jitter(&mut sk, 1.0, i + 1);
            let t0 = Instant::now();
            gcs_core::decompose::execute(&mut ps.plan, &mut sk, None);
            te.push(ms(t0));
        }
        let r = ps.solve(&mut sk, 1e-9, false, Method::DogLeg);
        println!(
            "{name:<16} compile {tc:7.1} ms | replay {:7.2} ms | {} | {}",
            median(te),
            ps.plan.summary(),
            if r.success { "exact" } else { "needs fallback" }
        );
    }

    println!("\n== drag frame (pull + polish), dogleg ==");
    for bays in [30usize, 50, 100, 200] {
        let sk = truss(bays, 20.0, 15.0, true);
        let ents = sk.points.len() + sk.lines.len();
        let (t, ok) = bench_drag(sk, 20);
        let (t2, ok2) = bench_drag(truss_floating(bays), 20);
        println!(
            "truss({bays:3}) {ents:5} entities: fully constrained {t:6.1} ms ({:4.0} fps) | \
             floating rigid {t2:6.1} ms ({:4.0} fps) {}",
            1e3 / t,
            1e3 / t2,
            if ok && ok2 { "ok" } else { "BAD" }
        );
    }

    println!("\n== drag of one figure among many (PlanDrag start + frame): the cost of the \
              region ==");
    println!("   own plan: the drag decomposes the figure | cached plan: as the app drags");
    for (n, copies) in [(32usize, 1usize), (32, 3), (32, 30), (128, 1), (2048, 1)] {
        let mut sk = zigzag(n, copies);
        let npts = sk.points.len();
        let (start, frame) = bench_plan_drag(&mut sk, n / 2, 20, true);
        let mut sk = zigzag(n, copies); // from the same geometry as the row above
        let (start2, frame2) = bench_plan_drag(&mut sk, n / 2, 20, false);
        println!(
            "zigzag {n:4} x {copies:2} ({npts:5} points): own plan start {start:7.2} ms \
             frame {frame:6.3} ms | cached plan start {start2:6.2} ms frame {frame2:6.3} ms"
        );
    }
}
