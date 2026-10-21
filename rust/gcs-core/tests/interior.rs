//! A row whose value is the output of a linear solve (#510 P5): a structure of bars, its
//! displacements `K(x)u = f`, and the adjoint gradient of an output of them.  The rules are
//! `docs/contract/analysis.md`'s; the kernel's finite-difference check is `jacobians.rs`'s
//! (`a_linear_interior_agrees_with_finite_differences`).
use gcs_core::interior::{self, Bar, Failure, Structure};
use gcs_core::kernels::KernelKey;
use gcs_core::model::Sketch;
use gcs_core::rng::Rng;
use gcs_core::system::build_kernel;

/// A braced structure of `n` nodes in `dim` dimensions and the columns it stands at: each node
/// past the first `dim` (held) joined to the `dim` before it, so it is rigid (Henneberg), and to
/// the one before those as well, so it is redundant and its forces read its areas.  The nodes run
/// along x with room across, each loaded, and the output a random weighing of every
/// displacement.
pub(crate) fn braced(dim: usize, n: usize, rng: &mut Rng) -> (Structure, Vec<f64>) {
    let mut x = Vec::new();
    for i in 0..n {
        x.push(10.0 * i as f64 + rng.uniform(-2.0, 2.0));
        for _ in 1..dim {
            x.push(rng.uniform(-8.0, 8.0));
        }
    }
    let mut bars = Vec::new();
    for i in dim..n {
        for j in i.saturating_sub(dim + 1)..i {
            bars.push(Bar { a: j, b: i, e: rng.uniform(100.0, 200.0) });
        }
    }
    x.extend(bars.iter().map(|_| rng.uniform(1.0, 5.0)));
    let dofs = dim * n;
    let s = Structure {
        dim,
        nodes: n,
        bars,
        fixed: (0..dofs).map(|i| i < dim * dim).collect(),
        load: (0..dofs).map(|_| rng.uniform(-10.0, 10.0)).collect(),
        weight: (0..dofs).map(|_| rng.uniform(-1.0, 1.0)).collect(),
    };
    (s, x)
}

/// Two bars from `(∓w, 0)`, held, to an apex at `(0, h)` loaded `(0, −p)`, each of modulus `e`
/// and area `a`, its output the apex's rise.
fn two_bar(w: f64, h: f64, e: f64, a: f64, p: f64) -> (Structure, Vec<f64>) {
    let s = Structure {
        dim: 2,
        nodes: 3,
        bars: vec![Bar { a: 0, b: 2, e }, Bar { a: 1, b: 2, e }],
        fixed: vec![true, true, true, true, false, false],
        load: vec![0.0, 0.0, 0.0, 0.0, 0.0, -p],
        weight: vec![0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
    };
    (s, vec![-w, 0.0, w, 0.0, 0.0, h, a, a])
}

/// The apex of two bars sinks `P L³ / (2 E A h²)`, and its rate in each number is the closed
/// form's: an area's half of `−δ/A` (the two areas move it alike), the rise's
/// `−P/(2EA)·(3√(w²+h²)/h − 2(w²+h²)^{3/2}/h³)`.
#[test]
fn a_two_bar_truss_sinks_as_its_closed_form() {
    let (w, h, e, a, p) = (300.0, 400.0, 200e3, 50.0, 10e3);
    let (s, x) = two_bar(w, h, e, a, p);
    let st = s.solve(&x).unwrap();
    let l2: f64 = w * w + h * h;
    let sink = -p * l2.powf(1.5) / (2.0 * e * a * h * h);
    let got = s.output(&st);
    assert!((got - sink).abs() <= 1e-12 * sink.abs(), "{got} against {sink}");
    // the apex moves straight down: the bars are alike
    assert!(st.u[4].abs() <= 1e-12 * sink.abs());
    let mut g = vec![0.0; s.columns()];
    s.gradient(&st, &x, &mut g);
    let by_h = -p / (2.0 * e * a) * (3.0 * l2.sqrt() / h - 2.0 * l2.powf(1.5) / (h * h * h));
    assert!((g[5] - by_h).abs() <= 1e-10 * by_h.abs(), "{} against {by_h}", g[5]);
    for i in [6, 7] {
        assert!((g[i] + sink / (2.0 * a)).abs() <= 1e-10 * (sink / a).abs(), "{}", g[i]);
    }
    // the apex across: its rate is the bars' asymmetry, zero where they are alike
    assert!(g[4].abs() <= 1e-10 * by_h.abs());
}

/// A bar in space pulled along itself stretches `P L / (E A)`, its far end held across.
#[test]
fn a_bar_in_space_stretches_by_pl_over_ea() {
    let (l, e, a, p) = (120.0, 70e3, 12.0, 3e3);
    let s = Structure {
        dim: 3,
        nodes: 2,
        bars: vec![Bar { a: 0, b: 1, e }],
        fixed: vec![true, true, true, false, true, true],
        load: vec![0.0, 0.0, 0.0, p, 0.0, 0.0],
        weight: vec![0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
    };
    let x = vec![5.0, -2.0, 1.0, 5.0 + l, -2.0, 1.0, a];
    let st = s.solve(&x).unwrap();
    let want = p * l / (e * a);
    assert!((s.output(&st) - want).abs() <= 1e-13 * want);
}

/// A structure with no displacement to give fails, and says why: a node held only along a line
/// of bars (free across it), a node no bar reaches, a bar of no length, a bar of no area.  The
/// kernel reads each as NaN, residual and Jacobian alike.
#[test]
fn a_structure_with_no_displacement_to_give_says_why() {
    let line = Structure {
        dim: 2,
        nodes: 3,
        bars: vec![Bar { a: 0, b: 1, e: 1.0 }, Bar { a: 1, b: 2, e: 1.0 }],
        fixed: vec![true, true, false, false, true, true],
        load: vec![0.0, 0.0, 0.0, -1.0, 0.0, 0.0],
        weight: vec![0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
    };
    let x = vec![0.0, 0.0, 1.0, 0.0, 2.0, 0.0, 1.0, 1.0];
    assert_eq!(line.solve(&x).err(), Some(Failure::Free));
    let mut lone = line.clone();
    lone.bars.truncate(1);
    lone.fixed = vec![true, true, true, true, false, false];
    assert_eq!(lone.solve(&x[..7]).err(), Some(Failure::Free));
    let mut x2 = x.clone();
    x2[2] = 0.0;
    assert_eq!(line.solve(&x2).err(), Some(Failure::Collapsed));
    let (s, mut x3) = two_bar(3.0, 4.0, 1.0, 1.0, 1.0);
    x3[7] = 0.0;
    assert_eq!(s.solve(&x3).err(), Some(Failure::Unstiff));

    let k = line.consts(&[0.0]);
    let (n_par, n_const) = line.widths(false);
    let kn = build_kernel(&Sketch::new(), KernelKey::Interior { n_par, n_const });
    let (mut r, mut j) = ([0.0], vec![0.0; n_par]);
    (kn.res)(1, &x, &k, &mut r);
    (kn.jac)(1, &x, &k, &mut j);
    assert!(r[0].is_nan() && j.iter().all(|v| v.is_nan()));
}

/// The gradient is one solve on the factor the value was found on, whatever number of columns
/// the row reads: the adjoint's, where a column at a time would be one each.  `gradient` only
/// reads the `State` (`Ata::resolve`, no factorisation), so asked twice it answers the same bits.
#[test]
fn the_gradient_is_one_more_solve_whatever_the_columns() {
    let mut rng = Rng::new(11);
    for (dim, n) in [(2, 4), (2, 400), (3, 300)] {
        let (s, x) = braced(dim, n, &mut rng);
        let st = s.solve(&x).unwrap();
        let (mut g, mut again) = (vec![0.0; s.columns()], vec![0.0; s.columns()]);
        s.gradient(&st, &x, &mut g);
        s.gradient(&st, &x, &mut again);
        assert!(g.iter().all(|v| v.is_finite()), "{dim}D, {n} nodes, {} columns", s.columns());
        assert_eq!(g, again);
    }
}

/// The constants are the structure, written and read back exactly, stated or free.
#[test]
fn a_structure_is_its_constants() {
    let mut rng = Rng::new(3);
    for dim in [2, 3] {
        let (s, _) = braced(dim, 7, &mut rng);
        for stated in [&[1.5][..], &[2.0, -0.25]] {
            let free = stated.len() == 2;
            let k = s.consts(stated);
            let (back, got) = Structure::read(&k, free);
            assert_eq!(back, s);
            assert_eq!(got, stated);
            assert_eq!(s.widths(free).1, k.len());
        }
    }
}

/// A requirement drives a dimension: the two bars' area found so the apex sinks half a
/// millimetre, by Newton on the row's residual and its adjoint gradient, to the closed form's
/// `P L³ / (2 E h² δ)`.
#[test]
fn a_requirement_drives_the_area() {
    let (w, h, e, p, sink) = (300.0, 400.0, 200e3, 10e3, 0.5);
    let want = p * f64::powf(w * w + h * h, 1.5) / (2.0 * e * h * h * sink);
    let (s, mut x) = two_bar(w, h, e, want / 2.0, p);
    let k = s.consts(&[-sink]);
    let (n_par, n_const) = s.widths(false);
    let kn = interior::kernel(n_par, n_const);
    let mut steps = 0;
    loop {
        let (mut r, mut j) = ([0.0], vec![0.0; n_par]);
        (kn.res)(1, &x, &k, &mut r);
        if r[0].abs() <= 1e-12 * sink {
            break;
        }
        (kn.jac)(1, &x, &k, &mut j);
        let da = -r[0] / (j[6] + j[7]);
        x[6] += da;
        x[7] += da;
        steps += 1;
        assert!(steps < 20, "Newton on the area did not settle");
    }
    assert!((x[6] - want).abs() <= 1e-9 * want, "{} against {want}", x[6]);
}
