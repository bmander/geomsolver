//! A row whose value is the output of a linear solve (#510 P5): a structure of bars, its
//! displacements `K(x)u = f`, and the adjoint gradient of an output of them.  The rules are
//! `docs/contract/analysis.md`'s; the kernel's finite-difference check is `jacobians.rs`'s
//! (`a_linear_interior_agrees_with_finite_differences`).
use gcs_core::interior::{self, Bar, Beam, Failure, Structure};
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
        beams: Vec::new(),
        fixed: (0..dofs).map(|i| i < dim * dim).collect(),
        load: (0..dofs).map(|_| rng.uniform(-10.0, 10.0)).collect(),
        weight: (0..dofs).map(|_| rng.uniform(-1.0, 1.0)).collect(),
    };
    (s, x)
}

/// Two bars from `(∓w, 0)`, held, to an apex at `(0, h)` loaded `(0, −p)`, each of modulus `e`
/// and area `a`, its output the apex's rise.
pub(crate) fn two_bar(w: f64, h: f64, e: f64, a: f64, p: f64) -> (Structure, Vec<f64>) {
    let s = Structure {
        dim: 2,
        nodes: 3,
        bars: vec![Bar { a: 0, b: 2, e }, Bar { a: 1, b: 2, e }],
        beams: Vec::new(),
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
        beams: Vec::new(),
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
        beams: Vec::new(),
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

/* -- the beam (#510 S0) ----------------------------------------------------------------------- */

/// A braced frame of `n` nodes in `dim` dimensions and the columns it stands at, as `braced`'s
/// bars but of beams, rigid at their ends: each node joined to the one before it and the one
/// before that, the first held whole, every degree of freedom loaded and weighed at random, each
/// beam's section's z a random direction (in the plane, z).
pub(crate) fn framed(dim: usize, n: usize, rng: &mut Rng) -> (Structure, Vec<f64>) {
    let mut x = Vec::new();
    for i in 0..n {
        x.push(10.0 * i as f64 + rng.uniform(-2.0, 2.0));
        for _ in 1..dim {
            x.push(rng.uniform(-8.0, 8.0));
        }
    }
    let mut beams = Vec::new();
    for i in 1..n {
        for j in i.saturating_sub(2)..i {
            let z = if dim == 2 {
                [0.0, 0.0, 1.0]
            } else {
                [rng.uniform(-1.0, 1.0), rng.uniform(-1.0, 1.0), rng.uniform(0.5, 1.0)]
            };
            let e = rng.uniform(100.0, 200.0);
            beams.push(Beam { a: j, b: i, e, g: 0.4 * e, rho: rng.uniform(1.0, 2.0), z });
        }
    }
    for _ in &beams {
        x.extend([rng.uniform(1.0, 5.0), rng.uniform(1.0, 5.0), rng.uniform(1.0, 5.0),
                  rng.uniform(1.0, 5.0)]);
    }
    let mut s = Structure { dim, nodes: n, bars: Vec::new(), beams, fixed: Vec::new(),
                            load: Vec::new(), weight: Vec::new() };
    let dofs = s.per() * n;
    s.fixed = (0..dofs).map(|i| i < s.per()).collect();
    s.load = (0..dofs).map(|_| rng.uniform(-10.0, 10.0)).collect();
    s.weight = (0..dofs).map(|_| rng.uniform(-1.0, 1.0)).collect();
    (s, x)
}

/// A cantilever of `pieces` beams along x from the origin to `l`, drawn in the front (its
/// section's z along y), its root held whole, of modulus `e`, shear modulus `g` and density
/// `rho`, its section `[A, Iy, Iz, J]`: the structure and its columns.
fn cantilever(dim: usize, pieces: usize, l: f64, e: f64, g: f64, rho: f64, section: [f64; 4])
    -> (Structure, Vec<f64>)
{
    let z = if dim == 2 { [0.0, 0.0, 1.0] } else { [0.0, 1.0, 0.0] };
    let beams: Vec<Beam> =
        (0..pieces).map(|i| Beam { a: i, b: i + 1, e, g, rho, z }).collect();
    let mut x = Vec::new();
    for i in 0..=pieces {
        x.push(l * i as f64 / pieces as f64);
        x.extend(std::iter::repeat_n(0.0, dim - 1));
    }
    for _ in 0..pieces {
        x.extend(section);
    }
    let mut s = Structure { dim, nodes: pieces + 1, bars: Vec::new(), beams, fixed: Vec::new(),
                            load: Vec::new(), weight: Vec::new() };
    let dofs = s.per() * s.nodes;
    s.fixed = (0..dofs).map(|i| i < s.per()).collect();
    s.load = vec![0.0; dofs];
    s.weight = vec![0.0; dofs];
    (s, x)
}

/// **A cantilever bends, twists and stretches as its closed forms** (#510 S0): its tip loaded
/// along z sinks `P L³ / 3EIz` and turns `P L² / 2EIz` (bending in its plane, about its section's
/// z), loaded along y moves `P L³ / 3EIy` (across its plane), twisted turns `T L / GJ`, pulled
/// stretches `P L / EA` — in one beam or several, in space and in the plane, to rounding: a cubic
/// is the beam's exact deflection under end loads.
#[test]
fn a_cantilever_bends_twists_and_stretches_as_its_closed_forms() {
    let (l, e, g, p) = (1000.0, 2e8, 7.7e7, 1e6);
    let (a, iy, iz, j) = (1250.0, 65104.0, 260417.0, 178000.0);
    for dim in [2, 3] {
        for pieces in [1, 5] {
            let (mut s, x) = cantilever(dim, pieces, l, e, g, 1.0, [a, iy, iz, j]);
            let per = s.per();
            let tip = |c: usize| per * pieces + if dim == 2 && c == 5 { 2 } else { c };
            // along the drawing's up, z in space and y in the plane
            let up = if dim == 2 { 1 } else { 2 };
            let mut cases: Vec<(usize, usize, f64)> = vec![
                (up, up, -p * l.powi(3) / (3.0 * e * iz)),
                (0, 0, p * l / (e * a)),
            ];
            if dim == 3 {
                cases.push((1, 1, p * l.powi(3) / (3.0 * e * iy)));
                cases.push((3, 3, p * l / (g * j)));
            }
            for (at, read, want) in cases {
                s.load.fill(0.0);
                s.weight.fill(0.0);
                s.load[tip(at)] = if at == up { -p } else { p };
                s.weight[tip(read)] = 1.0;
                let st = s.solve(&x).unwrap();
                let got = s.output(&st);
                assert!((got - want).abs() <= 1e-10 * want.abs(),
                        "{dim}D, {pieces} pieces, load {at}: {got} against {want}");
                if at == up {
                    // the tip's turn, about the plane's normal: y in space, z in the plane
                    let turn = if dim == 2 { st.u[tip(5)] } else { st.u[tip(4)] };
                    let want = p * l * l / (2.0 * e * iz);
                    assert!((turn.abs() - want).abs() <= 1e-10 * want, "{turn} against {want}");
                }
            }
        }
    }
}

/// **A cantilever's modes and its buckling are its closed forms'** (#510 S0) as the beams it is
/// cut into grow many: its frequencies `βᵢ²/(2πL²)·√(EI/ρA)`, β₁L = 1.8751, β₂L = 4.6941 — the
/// first in its weaker plane, then the first in its stronger (`Iz = 4 Iy`, twice the frequency),
/// then the second in its weaker — and the first load factor pressing its head, Euler's
/// `π² E I / 4L²` over the load of the weaker second moment.  The cubic's error falls as the
/// fourth power of the piece: within 1e-5 at sixteen, the first two within 1e-6.
#[test]
fn a_cantilevers_modes_and_buckling_are_its_closed_forms() {
    let (l, e, g, rho) = (1000.0, 2e8, 7.7e7, 7.85e-6);
    let (a, iy, iz, j) = (1250.0, 65104.0, 4.0 * 65104.0, 178000.0);
    let (mut s, x) = cantilever(3, 16, l, e, g, rho, [a, iy, iz, j]);
    let k = s.stiffness(&x).unwrap();
    let m = s.mass(&x).unwrap();
    let pairs = gcs_core::eigen::lowest(&k, &m, 3, &Default::default()).unwrap();
    let freq = |v: f64| v.sqrt() / (2.0 * std::f64::consts::PI);
    for (n, beta, i) in [(0, 1.8751040687, iy), (1, 1.8751040687, iz), (2, 4.6940911330, iy)] {
        let want = beta * beta / (2.0 * std::f64::consts::PI * l * l) * (e * i / (rho * a)).sqrt();
        let got = freq(pairs.values[n]);
        let within = if n < 2 { 1e-6 } else { 1e-5 };
        assert!((got - want).abs() <= within * want, "mode {}: {got} against {want}", n + 1);
    }
    // pressed at its head: every beam carries the load
    let p = 1e5;
    let head = s.dof(16, 0).unwrap();
    s.load[head] = -p;
    let st = s.solve(&x).unwrap();
    let axial: Vec<f64> = s.end_actions(&st.u, &x).iter().map(|f| f.unwrap()[6]).collect();
    assert!(axial.iter().all(|n| (n + p).abs() <= 1e-9 * p), "{axial:?}");
    let kg = s.geometric(&x, &axial).unwrap();
    let buckled = gcs_core::eigen::buckling(&k, &kg, 1, &Default::default()).unwrap();
    let euler = std::f64::consts::PI.powi(2) * e * iy / (4.0 * l * l) / p;
    assert!((buckled.values[0] - euler).abs() <= 1e-6 * euler, "{:?} against {euler}",
            buckled.values);
}

/// A frame's constants are the frame, written and read back exactly, stated or free; and a free
/// frame names what leaves it free.
#[test]
fn a_frame_is_its_constants() {
    let mut rng = Rng::new(5);
    for dim in [2, 3] {
        let (s, x) = framed(dim, 6, &mut rng);
        for stated in [&[1.5][..], &[2.0, -0.25]] {
            let free = stated.len() == 2;
            let k = s.consts(stated);
            let (back, got) = Structure::read(&k, free);
            assert_eq!(back, s);
            assert_eq!(got, stated);
            assert_eq!(s.widths(free).1, k.len());
        }
        assert!(s.solve(&x).is_ok());
        assert_eq!(s.free(&x).text(dim), None, "held whole at its root");
        // its root's turns let go: held at a point, it turns about it
        let mut loose = s.clone();
        for k in dim..loose.per() {
            loose.fixed[k] = false;
        }
        assert_eq!(loose.solve(&x).err(), Some(Failure::Free));
        let said = loose.free(&x).text(dim).unwrap();
        assert!(said.starts_with("free to turn about"), "{said}");
    }
}

