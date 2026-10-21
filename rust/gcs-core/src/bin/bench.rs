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

/// The regular triangulation (`delaunay`): insertion time per point, in spatial order, over
/// point sets a mesher meets. Median of three runs; exact predicate fallbacks counted.
fn bench_delaunay(only: Option<&str>) {
    use gcs_core::delaunay::{Regular,spatial_order,predicates::exact_calls};
    use gcs_core::rng::Rng;
    println!("\n== regular triangulation: µs a point, spatial order ==");
    let mut rng = Rng::new(1);
    // `DELAUNAY_N` sizes the random cases (100 000 by default).
    let size: usize = std::env::var("DELAUNAY_N").ok().and_then(|v| v.parse().ok()).unwrap_or(100_000);
    let uniform: Vec<([f64;3],f64)> = (0..size).map(|_| ([rng.uniform(-1.,1.),rng.uniform(-1.,1.),rng.uniform(-1.,1.)],0.)).collect();
    let weighted: Vec<([f64;3],f64)> = uniform.iter().map(|&(p,_)| (p,rng.uniform(0.,1e-4))).collect();
    let shell: Vec<([f64;3],f64)> = (0..size).map(|_| {
        let (t,z,r) = (rng.uniform(0.,std::f64::consts::TAU),rng.uniform(-1.,1.),rng.uniform(0.999,1.));
        let s = (1.-z*z).sqrt();
        ([r*s*t.cos(),r*s*t.sin(),r*z],0.)
    }).collect();
    let mut lattice = Vec::new();
    for i in 0..40 { for j in 0..40 { for k in 0..40 { lattice.push(([i as f64/20.-1.,j as f64/20.-1.,k as f64/20.-1.],0.)); } } }
    {
        // Exactly degenerate input goes to the exact path: coplanar and cospherical lattice points.
        use gcs_core::delaunay::predicates::{orient_exact,power_exact,Weighted};
        let w = |p: [f64;3]| Weighted {p,w:0.};
        let (a,b,c,d) = ([0.1,0.2,0.3],[1.1,0.2,0.3],[0.1,1.2,0.3],[0.1,0.2,1.3]);
        let n = 20_000;
        let t0 = Instant::now();
        let mut s = 0i64;
        for k in 0..n { let f = (k % 7) as f64; s += orient_exact([f,0.,0.],[f+1.,0.,0.],[f,1.,0.],[f+3.,7.,0.]) as i64; }
        let orient_ns = t0.elapsed().as_secs_f64()*1e9/n as f64;
        let t0 = Instant::now();
        for k in 0..n { let f = (k % 7) as f64;
            s += power_exact(w([5.+f,0.,0.]),w([f,5.,0.]),w([f,0.,5.]),w([f-3.,-4.,0.]),w([f+3.,0.,4.])) as i64; }
        let power_ns = t0.elapsed().as_secs_f64()*1e9/n as f64;
        let _ = (a,b,c,d);
        println!("exact orient {orient_ns:8.0} ns, exact power {power_ns:8.0} ns (checksum {s})");
    }
    for (name,points) in [("uniform",&uniform),("weighted",&weighted),("shell",&shell),("lattice 40³",&lattice)] {
        if only.is_some_and(|o| !name.starts_with(o)) { continue; }
        let coordinates: Vec<[f64;3]> = points.iter().map(|p| p.0).collect();
        let order = spatial_order(&coordinates);
        let mut times = Vec::new();
        let mut tets = 0;
        let calls = exact_calls();
        for _ in 0..3 {
            let t0 = Instant::now();
            let mut r = Regular::new([0.;3],1.8);
            for &i in &order { r.insert(points[i].0,points[i].1).unwrap(); }
            times.push(t0.elapsed().as_secs_f64()*1e6/points.len() as f64);
            tets = r.tets().count();
        }
        println!("{name:<14}{:>8} points {:>9} tetrahedra {:8.2} µs  exact calls {}",points.len(),tets,median(times),
            (exact_calls()-calls)/3);
    }
}

/// A point on a meshing (#283, #302): its kernel's residual and Jacobian at `contact.sv`'s
/// drawing, µs an evaluation — the Jacobian the cost of differentiating both flanks.
fn bench_meshing() {
    use gcs_core::constraints::CKind;
    println!("\n== a point on a meshing (spiral_bevel/contact.sv): µs an evaluation ==");
    let (prog, errs, _) =
        gcs_core::library::parse_linked(include_str!("../../../examples/spiral_bevel/contact.sv"));
    assert!(errs.is_empty(), "{errs:?}");
    let sk = gcs_core::program::elaborate(&prog).sketch;
    let c = sk.constraints.iter().find(|c| c.kind == CKind::PointOnMeshing).expect("a contact");
    let (k, v, kc) = (c.kernel_in(&sk), c.local_values(&sk), c.consts(&sk));
    let (mut r, mut j) = (vec![0.0; k.n_res], vec![0.0; k.n_res * v.len()]);
    let time = |f: &mut dyn FnMut()| {
        let reps = 200;
        let ts: Vec<f64> = (0..15).map(|_| {
            let t0 = Instant::now();
            for _ in 0..reps {
                f();
            }
            ms(t0) * 1e3 / reps as f64
        }).collect();
        median(ts)
    };
    let tr = time(&mut || (k.res)(1, &v, &kc, &mut r));
    let tj = time(&mut || (k.jac)(1, &v, &kc, &mut j));
    let w = v.len();
    println!("contact         {w:>4} columns | residual {tr:8.2} µs | Jacobian {tj:8.2} µs");
}

/// **A scrub step of the V-twin** (#412): the stand at a crank angle against the objects asked
/// for where it leaves them — moved, since each moving part is rigid in its frame — and against
/// the same objects evaluated afresh there, which is what each step cost before.
fn bench_scrub() {
    println!("\n== a scrub step of the V-twin (#412): ms a step, median of 12 ==");
    let mut sk = examples::case("vtwin").expect("the V-twin");
    gcs_core::solve::solve(&mut sk, Default::default());
    let objects = gcs_core::overview::objects(&sk);
    let ask = |sk: &Sketch| for &i in &objects {
        let policy = gcs_core::solid::ApproximationPolicy::Mesh;
        std::hint::black_box(sk.evaluated_solid(i, policy).expect("evaluates").mesh());
    };
    ask(&sk);
    gcs_core::follow::followed(&sk, "t");
    let (mut stand, mut moved, mut built) = (Vec::new(), Vec::new(), Vec::new());
    for k in 1..=12 {
        let t0 = Instant::now();
        gcs_core::follow::stand(&mut sk, "t", 30.0 * k as f64 - 7.0).expect("stood");
        stand.push(ms(t0));
        let t0 = Instant::now();
        ask(&sk);
        moved.push(ms(t0));
        let fresh = sk.clone();
        fresh.caches.clear();
        let t0 = Instant::now();
        ask(&fresh);
        built.push(ms(t0));
    }
    println!("stand {:8.2} ms | {} objects moved {:8.2} ms | built afresh {:8.2} ms",
             median(stand), objects.len(), median(moved), median(built));
}

/// **A body rounded all over** (`rounds/bores.sv`: nine faces, a region for each pair and triple
/// of them, #415): its elaboration, its first evaluation — every region lowered and held
/// to its field, the body built and meshed — and the solid asked for again, as a page asks it for
/// each thing it draws of it.
fn bench_rounded() {
    println!("\n== a body rounded all over (rounds/bores.sv): ms ==");
    let (prog, errs, _) =
        gcs_core::library::parse_linked(include_str!("../../../examples/rounds/bores.sv"));
    assert!(errs.is_empty(), "{errs:?}");
    let t0 = Instant::now();
    let mut sk = gcs_core::program::elaborate(&prog).sketch;
    let elaborated = ms(t0);
    gcs_core::solve::solve(&mut sk, Default::default());
    let slab = sk.solids.iter().position(|s| s.name == "slab").expect("the slab");
    let policy = gcs_core::solid::ApproximationPolicy::Mesh;
    let t0 = Instant::now();
    std::hint::black_box(sk.evaluated_solid(slab, policy).expect("evaluates"));
    let first = ms(t0);
    let again: Vec<f64> = (0..5).map(|_| {
        let t0 = Instant::now();
        std::hint::black_box(sk.evaluated_solid(slab, policy).expect("evaluates"));
        ms(t0)
    }).collect();
    println!("{} solids: elaborate {elaborated:8.1} ms | first evaluation {first:8.1} ms | asked again {:6.2} ms",
             sk.solids.len(), median(again));
}

/// **Boundaries against the flat solve** (#459): each document of `boundary::measure` solved flat
/// (compile and solve, as a document is) and through its instances (`solve_through`, #454's
/// prototype), timed, with the counts that do not depend on the machine beside each.  `bench
/// boundaries [most]`: drives past `most` (100 by default) are solved flat only, and edits to
/// more than it not at all.
fn bench_boundaries(most: usize) {
    use gcs_core::boundary::measure::{self, Document, Drive};
    use gcs_core::model::Sketch;
    use gcs_core::solve::{solve_through, BlockMode, Boundary, SolveOpts, Through};
    // the worst relative disagreement of two poses
    let apart = |a: &Sketch, b: &Sketch| a.params.iter().zip(&b.params)
        .map(|(p, q)| (p.value - q.value).abs() / (1.0 + q.value.abs())).fold(0.0, f64::max);
    let reps = |n: usize| if n >= 100 { 1 } else if n >= 30 { 3 } else { 7 };
    // (flat, flat block by block, through) medians in ms, and the last of each
    let run = |doc: &Document, n: usize| {
        let e = measure::read(&doc.src).expect("reads");
        let t0 = Instant::now();
        let bs: Vec<Boundary> = measure::bounds(&e, &doc.boundaries).expect("boundaries");
        let named = ms(t0);
        let (mut tf, mut tb, mut tt) = (Vec::new(), Vec::new(), Vec::new());
        let (mut f, mut fb, mut t) = (None, None, None);
        let (mut sf, mut st) = (e.sketch.clone(), e.sketch.clone());
        for _ in 0..reps(n) {
            let mut sk = e.sketch.clone();
            let t0 = Instant::now();
            f = Some(measure::flat(&mut sk, SolveOpts::default()));
            tf.push(ms(t0));
            sf = sk;
            let mut sk = e.sketch.clone();
            let t0 = Instant::now();
            let first = SolveOpts { blocks: BlockMode::First, ..SolveOpts::default() };
            fb = Some(measure::flat(&mut sk, first));
            tb.push(ms(t0));
            let mut sk = e.sketch.clone();
            let t0 = Instant::now();
            t = Some(solve_through(&mut sk, &bs, SolveOpts::default()).expect("a boundary"));
            tt.push(ms(t0));
            st = sk;
        }
        let gap = apart(&st, &sf);
        (median(tf), median(tb), median(tt), f.unwrap(), fb.unwrap(), t.unwrap(), gap, named)
    };
    let solves = |t: &Through| t.evaluations.iter().map(|e| e.1).sum::<usize>();
    let ok = |b: bool| if b { "ok" } else { "FAIL" };

    println!("\n== boundaries (#459): repeated and distinct belt drives, flat and through their \
              runs ==");
    println!("{:<22}{:>6}{:>6} | {:^30} | {:^9} | {:^62} | apart", "case", "N", "free",
             "flat: ms / it / largest block", "blocks ms",
             "through: ms / it / inner solves / inner it / again / parent");
    for (label, distinct, shared) in [("identical", false, false), ("distinct", true, false),
                                      ("distinct, one unknown", true, true)] {
        for n in [1usize, 3, 10, 30, 100, 300, 1000] {
            let drives: Vec<Drive> = (0..n).map(|k| {
                let spread = 40.0 + 10.0 * (k as f64 + 0.5) / n as f64;
                let r_large = if distinct { spread } else { 45.0 };
                Drive { r_large, ..Drive::PAPER }
            }).collect();
            let doc = measure::belts(&drives, shared);
            // the dense parent is `parent_rows × parent_unknowns`, factored every step: past
            // `most` drives only the flat solve runs (a hundred is seconds, three hundred minutes)
            if n > most {
                let e = measure::read(&doc.src).expect("reads");
                let mut sk = e.sketch.clone();
                let t0 = Instant::now();
                let f = measure::flat(&mut sk, SolveOpts::default());
                let tf = ms(t0);
                println!("{label:<22}{n:>6}{:>6} | {tf:9.1} {:>4} {:>4} {:>8} | {:^9} | {:^62} |",
                         f.unknowns, f.result.iterations, f.largest_block, ok(f.result.success),
                         "-", format!("not run: the parent is {0}x{0}, dense", 8 * n));
                continue;
            }
            let (tf, tb, tt, f, fb, t, gap, named) = run(&doc, n);
            println!("{label:<22}{n:>6}{:>6} | {tf:9.1} {:>4} {:>4} {:>8} | {tb:9.1} | {tt:9.1} \
                      {:>4} {:>6} {:>7} {:>6} {:>5}x{:<5} {:>4} | {gap:.0e}  \
                      (named {named:.0} ms{})",
                     f.unknowns, f.result.iterations, f.largest_block,
                     ok(f.result.success && fb.result.success),
                     t.result.iterations, solves(&t), t.inner_iterations, t.answered_again,
                     t.parent_rows, t.parent_unknowns, ok(t.result.success),
                     if fb.result.success { "" } else { "; block by block FAILED" });
        }
    }

    println!("\n== boundaries (#459): a closed loop through k two-root points, flat and through \
              them ==");
    println!("{:>5}{:>6} | {:^30} | {:^9} | {:^54} | apart", "k", "free",
             "flat: ms / it / largest block", "blocks ms",
             "through: ms / it / inner solves / inner it / parent");
    for k in [2usize, 4, 8, 16, 32, 64, 128, 256] {
        let doc = measure::dyads(k);
        let (tf, tb, tt, f, fb, t, gap, _) = run(&doc, k);
        println!("{k:>5}{:>6} | {tf:9.1} {:>4} {:>4} {:>8} | {tb:9.1} | {tt:9.1} {:>4} {:>6} {:>7} \
                  {:>5}x{:<5} {:>4} | {gap:.0e}",
                 f.unknowns, f.result.iterations, f.largest_block,
                 ok(f.result.success && fb.result.success), t.result.iterations, solves(&t),
                 t.inner_iterations, t.parent_rows, t.parent_unknowns, ok(t.result.success));
    }

    // an input swept, each solve started where the last ended, flat and through alike
    let sweep = |title: &str, label: &str, values: &[f64], make: &dyn Fn(f64) -> Document| {
        println!("\n== boundaries (#459): {title} ==");
        println!("{label:>10} | {:^24} | {:^56} | {:^22} | apart", "flat: it / max residual",
                 "through: it / inner solves / inner it / rejected / failure",
                 "max |dy/dx| / FD error");
        let (mut sf, mut st): (Option<Sketch>, Option<Sketch>) = (None, None);
        for &v in values {
            let doc = make(v);
            let e = measure::read(&doc.src).expect("reads");
            let bs = measure::bounds(&e, &doc.boundaries).expect("boundaries");
            let mut a = e.sketch.clone();
            if let Some(s) = &sf {
                measure::carry(s, &mut a);
            }
            let f = measure::flat(&mut a, SolveOpts::default());
            let mut b = e.sketch.clone();
            if let Some(s) = &st {
                measure::carry(s, &mut b);
            }
            let t = solve_through(&mut b, &bs, SolveOpts::default()).expect("a boundary");
            let failure = t.failure.as_ref().map_or("-".to_string(), |f| f.to_string());
            let d = bs.iter().map(|x| measure::derivative(&b, x))
                .try_fold((0.0f64, 0.0f64),
                          |(l, r), d| d.map(|d| (l.max(d.largest), r.max(d.error))));
            let d = match d {
                Ok((l, r)) => format!("{l:9.3e} {r:9.1e}"),
                Err(f) => format!("{f}"),
            };
            println!("{v:>10} | {:>4} {:>9.1e} {:>8} | {:>4} {:>6} {:>7} {:>4} {failure:<24} \
                      {:>4} | \
                      {d:<22} | {:.0e}",
                     f.result.iterations, f.result.max_residual, ok(f.result.success),
                     t.result.iterations, solves(&t), t.inner_iterations, t.rejected,
                     ok(t.result.success), apart(&b, &a));
            sf = Some(a);
            st = Some(b);
        }
    };
    sweep("a branch: the centres swept until one rim holds the other (r 20 and 45, edge at 25)",
          "centres", &[120.0, 80.0, 50.0, 35.0, 30.0, 27.0, 26.0, 25.5, 25.1, 25.01, 25.001,
                       25.0001, 25.0, 24.9],
          &|c| measure::belts(&[Drive { centres: c, ..Drive::PAPER }], false));
    sweep("singular in the parent: the radii swept equal, the runs parallel and their crossing \
           gone (r₁ 20)",
          "r_large", &[45.0, 30.0, 25.0, 22.0, 21.0, 20.5, 20.1, 20.01, 20.001, 20.0001, 20.0],
          &|r| measure::belts(&[Drive { r_large: r, ..Drive::PAPER }], false));
    sweep("singular in the boundary: a line swept onto the circle it cuts (x² = u at reach 40)",
          "reach", &[35.0, 38.0, 39.0, 39.9, 39.99, 39.999, 39.9999, 39.99999, 40.0, 40.1],
          &measure::meet);

    println!("\n== boundaries (#459): an edit to one drive of N, re-solved from the solved \
              pose ==");
    println!("{:>6} | {:^22} | {:^74}", "N", "flat: ms / it",
             "through: ms / it / inner solves / inner it / boundaries whose inputs moved / \
              drives moved");
    for n in [1usize, 10, 100] {
        if n > most {
            continue;
        }
        let before = measure::belts(&vec![Drive::PAPER; n], false);
        let mut drives = vec![Drive::PAPER; n];
        drives[0].centres = 125.0;
        let after = measure::belts(&drives, false);
        let (_, solved) = measure::solved(&before.src).expect("solves");
        let e = measure::read(&after.src).expect("reads");
        let bs = measure::bounds(&e, &after.boundaries).expect("boundaries");
        let mut start = e.sketch.clone();
        measure::carry(&solved, &mut start);
        let (mut tf, mut tt) = (Vec::new(), Vec::new());
        let (mut f, mut t) = (None, None);
        let mut sk_t = start.clone();
        for _ in 0..reps(n) {
            let mut sk = start.clone();
            let t0 = Instant::now();
            f = Some(measure::flat(&mut sk, SolveOpts::default()));
            tf.push(ms(t0));
            sk_t = start.clone();
            let t0 = Instant::now();
            t = Some(solve_through(&mut sk_t, &bs, SolveOpts::default()).expect("a boundary"));
            tt.push(ms(t0));
        }
        let (f, t) = (f.unwrap(), t.unwrap());
        let moved_inputs = t.evaluations.iter().filter(|e| e.1 > 1).count();
        // a drive the edit's solve moved: any bit of its runs' unknowns or of what they read
        let listed = gcs_core::solve::Boundaries::new(&start, &bs).expect("boundaries");
        let moved = bs.chunks(2).filter(|runs| runs.iter()
            .flat_map(|b| listed.unknowns(&b.name).unwrap().into_iter()
                .chain(listed.inputs(&b.name).unwrap()))
            .any(|p| sk_t.params[p as usize].value.to_bits()
                 != solved.params[p as usize].value.to_bits())).count();
        println!("{n:>6} | {:9.2} {:>4} {:>6} | {:9.2} {:>4} {:>6} {:>7} {:>6} of {:<6} \
                  {:>6} of {:<4} \
                  {}",
                 median(tf), f.result.iterations, ok(f.result.success), median(tt),
                 t.result.iterations, solves(&t), t.inner_iterations, moved_inputs,
                 t.evaluations.len(), moved, n, ok(t.result.success));
    }
}

fn main() {
    // `bench delaunay [case]`: the triangulation alone, or one of its cases (for timing a single
    // case's CPU from outside, as `/usr/bin/time` does when the machine is busy).
    if std::env::args().nth(1).as_deref() == Some("meshing") { bench_meshing(); return; }
    if std::env::args().nth(1).as_deref() == Some("scrub") { bench_scrub(); return; }
    if std::env::args().nth(1).as_deref() == Some("rounded") { bench_rounded(); return; }
    if std::env::args().nth(1).as_deref() == Some("boundaries") {
        bench_boundaries(std::env::args().nth(2).and_then(|n| n.parse().ok()).unwrap_or(100));
        return;
    }
    if std::env::args().nth(1).as_deref() == Some("delaunay") { bench_delaunay(std::env::args().nth(2).as_deref()); return; }
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
    bench_meshing();
    bench_scrub();
    bench_rounded();
    bench_delaunay(None);
}
