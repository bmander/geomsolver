//! A row whose value is the output of a linear solve (#510 P5): a structure of bars, its
//! displacements `K(x)u = f`, and the adjoint gradient of an output of them.  The rules are
//! `docs/contract/analysis.md`'s; the kernel's finite-difference check is `jacobians.rs`'s
//! (`a_linear_interior_agrees_with_finite_differences`).
use gcs_core::delaunay::tetra::{self, EDGES};
use gcs_core::interior::{self, Bar, Beam, Failure, Spread, Structure};
use gcs_core::kernels::KernelKey;
use gcs_core::model::Sketch;
use gcs_core::rng::Rng;
use gcs_core::space::{cross, dot, sub};
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
        spread: Vec::new(),
        tris: Vec::new(),
        laid: Vec::new(),
        ..Structure::default()
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
        spread: Vec::new(),
        tris: Vec::new(),
        laid: Vec::new(),
        ..Structure::default()
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
        spread: Vec::new(),
        tris: Vec::new(),
        laid: Vec::new(),
        ..Structure::default()
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
        spread: Vec::new(),
        tris: Vec::new(),
        laid: Vec::new(),
        ..Structure::default()
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
/// beam's section's z a random direction (in the plane, z); a force spread along its first three
/// beams and a body load along every other one.
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
                            load: Vec::new(), weight: Vec::new(), spread: Vec::new(),
                            tris: Vec::new(), laid: Vec::new(),
                            ..Structure::default() };
    let dofs = s.per() * n;
    s.fixed = (0..dofs).map(|i| i < s.per()).collect();
    s.load = (0..dofs).map(|_| rng.uniform(-10.0, 10.0)).collect();
    s.weight = (0..dofs).map(|_| rng.uniform(-1.0, 1.0)).collect();
    // a force spread evenly along the first beams, and a body load on every other one (#510 S1)
    let m = s.beams.len();
    let mut force = || [0, 1, 2].map(|k| if k < dim { rng.uniform(-10.0, 10.0) } else { 0.0 });
    s.spread = vec![
        Spread { beams: (0..m.min(3)).collect(), force: force(), per_area: false },
        Spread { beams: (0..m).step_by(2).collect(), force: force(), per_area: true },
    ];
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
                            load: Vec::new(), weight: Vec::new(), spread: Vec::new(),
                            tris: Vec::new(), laid: Vec::new(),
                            ..Structure::default() };
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


/// **A load spread along a cantilever is its consistent load** (#510 S1, `Spread`): spread
/// evenly, its whole `F` over the length, the tip sinks `q L⁴ / 8EIz` and turns `q L³ / 6EIz`
/// with `q = F / L`, and as a body load `q = ρ a A` the same — in one beam or several, in the
/// plane and in space, to rounding, the cubic's nodal values being exact under it.  One beam
/// alone carries the end moments `± q L² / 12` uncancelled, so a wrong one shows here.
#[test]
fn a_spread_load_is_its_consistent_load() {
    let (l, e, g) = (1000.0, 2e8, 7.7e7);
    let (a, iy, iz, j) = (1250.0, 65104.0, 260417.0, 178000.0);
    for dim in [2, 3] {
        for pieces in [1, 4] {
            let up = if dim == 2 { 1 } else { 2 };
            for per_area in [false, true] {
                let (mut s, x) = cantilever(dim, pieces, l, e, g, 1.0, [a, iy, iz, j]);
                let mut force = [0.0; 3];
                force[up] = if per_area { -0.8 } else { -1e5 };
                s.spread = vec![Spread { beams: (0..pieces).collect(), force, per_area }];
                let q = if per_area { 0.8 * a } else { 1e5 / l };
                let per = s.per();
                let tip = per * pieces + up;
                s.weight[tip] = 1.0;
                let st = s.solve(&x).unwrap();
                let want = -q * l.powi(4) / (8.0 * e * iz);
                let got = s.output(&st);
                assert!((got - want).abs() <= 1e-10 * want.abs(),
                        "{dim}D, {pieces} pieces, per area {per_area}: {got} against {want}");
                let turn = if dim == 2 { st.u[per * pieces + 2] } else { st.u[per * pieces + 4] };
                let want = q * l.powi(3) / (6.0 * e * iz);
                assert!((turn.abs() - want).abs() <= 1e-10 * want, "{turn} against {want}");
            }
        }
    }
}

/// **A row's sections read over unknowns** (#510 S1, `Sizing`): the cantilever's depth `h` an
/// unknown, its section `b h`, `h b³/12`, `b h³/12`, a constant, each a tape over it — the row's
/// columns its nodes' then `h`'s, and its value the tip's sink at the sections the tapes give.
/// Newton on the row finds the depth at which the tip sinks `δ`, `(4 P L³ / (E b δ))^(1/3)`.
#[test]
fn a_rows_sections_are_read_over_its_unknowns() {
    use gcs_core::interior::Sizing;
    let (l, e, g, b, p, sink) = (1000.0, 2e5, 7.7e4, 25.0, 1e3, 4.0);
    let (mut s, x) = cantilever(2, 1, l, e, g, 1.0, [1.0; 4]);
    let tip = s.per() + 1;
    s.load[tip] = -p;
    s.weight[tip] = 1.0;
    let vars = ["h".to_string()];
    let tape = |text: &str| {
        let ast = gcs_core::expr::parse_in(text, gcs_core::units::Units::default()).unwrap();
        gcs_core::tape::Tape::compile(&ast.body, &vars).unwrap().flat
    };
    let sizing = Sizing { unknowns: 1, sections: vec![[tape("25 * h"), tape("h * 25^3 / 12"),
                                                        tape("25 * h^3 / 12"), tape("1000")]] };
    let k = s.consts_sized(&[-sink], Some(&sizing));
    let (n_par, n_const) = s.widths_sized(false, Some(&sizing));
    assert_eq!((n_par, n_const), (2 * 2 + 1, k.len()));
    let kn = interior::kernel(n_par, n_const);
    let mut v = [&x[..4], &[40.0]].concat();
    for _ in 0..30 {
        let (mut r, mut jac) = ([0.0], vec![0.0; n_par]);
        (kn.res)(1, &v, &k, &mut r);
        if r[0].abs() <= 1e-13 * sink {
            break;
        }
        (kn.jac)(1, &v, &k, &mut jac);
        v[4] -= r[0] / jac[4];
    }
    let want = (4.0 * p * l.powi(3) / (e * b * sink)).cbrt();
    assert!((v[4] - want).abs() <= 1e-9 * want, "{} against {want}", v[4]);
}

/* -- the plane element (#510 S2) ------------------------------------------------------------- */

/// A sheet of quadratic triangles over `[0, w] × [0, h]`, `nx` by `ny` cells each cut on its
/// diagonal, its inner corners knocked about by `jitter` of a cell and every middle node at the
/// middle of its side (so the elements are straight): the nodes on a grid of `(2nx + 1)(2ny + 1)`,
/// node `(a, b)` at `b (2nx + 1) + a`.  Nothing held or loaded.
pub(crate) fn sheet(nx: usize, ny: usize, w: f64, h: f64, jitter: f64, strain: bool,
                    rng: &mut Rng) -> (Structure, Vec<f64>) {
    let (cols, rows) = (2 * nx + 1, 2 * ny + 1);
    let id = |a: usize, b: usize| b * cols + a;
    let mut x = vec![0.0; 2 * cols * rows];
    for b in (0..rows).step_by(2) {
        for a in (0..cols).step_by(2) {
            let inner = a > 0 && b > 0 && a + 1 < cols && b + 1 < rows;
            let (dx, dy) = if inner {
                (rng.uniform(-jitter, jitter) * w / nx as f64,
                 rng.uniform(-jitter, jitter) * h / ny as f64)
            } else {
                (0.0, 0.0)
            };
            x[2 * id(a, b)] = a as f64 * w / (2 * nx) as f64 + dx;
            x[2 * id(a, b) + 1] = b as f64 * h / (2 * ny) as f64 + dy;
        }
    }
    let mut tris = Vec::new();
    let t = |n: [(usize, usize); 6]| interior::Tri6 { n: n.map(|(a, b)| id(a, b)), e: 2e5,
                                                       nu: 0.3, rho: 7.8e-9, depth: 2.0, strain };
    for j in 0..ny {
        for i in 0..nx {
            let (a, b) = (2 * i, 2 * j);
            tris.push(t([(a, b), (a + 2, b), (a + 2, b + 2), (a + 1, b), (a + 2, b + 1),
                         (a + 1, b + 1)]));
            tris.push(t([(a, b), (a + 2, b + 2), (a, b + 2), (a + 1, b + 1), (a + 1, b + 2),
                         (a, b + 1)]));
        }
    }
    // every middle node at the middle of the side it is on
    for tri in &tris {
        for (m, (p, q)) in [(3, (0, 1)), (4, (1, 2)), (5, (2, 0))] {
            for c in 0..2 {
                x[2 * tri.n[m] + c] = 0.5 * (x[2 * tri.n[p] + c] + x[2 * tri.n[q] + c]);
            }
        }
    }
    let dofs = 2 * cols * rows;
    let s = Structure { dim: 2, nodes: cols * rows, bars: Vec::new(), beams: Vec::new(),
                        fixed: vec![false; dofs], load: vec![0.0; dofs], weight: vec![0.0; dofs],
                        spread: Vec::new(), tris, laid: Vec::new(),
                        ..Structure::default() };
    (s, x)
}

/// The sheet's sides along its right edge, run upward with the material on their left.
fn right_sides(nx: usize, ny: usize) -> Vec<[usize; 3]> {
    let cols = 2 * nx + 1;
    (0..ny).map(|j| [2 * j, 2 * j + 1, 2 * j + 2].map(|b| b * cols + 2 * nx)).collect()
}

/// **The patch test** (#510 S2): a sheet of knocked-about quadratic triangles, its left edge held
/// along x and one corner across, pulled by a pressure of `−σ` on its right edge, stresses
/// uniformly — `σx = σ`, `σy = τ = 0` at every node of every element — and stretches as the
/// closed form: `σ w / E'` along it and `−ν' σ h / E'` across, `E' = E`, `ν' = ν` in plane stress
/// and `E/(1 − ν²)`, `ν/(1 − ν)` in plane strain.
#[test]
fn a_sheet_pulled_stresses_uniformly_as_the_patch_test_asks() {
    let (nx, ny, w, h, sigma) = (4, 2, 40.0, 20.0, 50.0);
    for strain in [false, true] {
        let mut rng = Rng::new(7);
        let (mut s, x) = sheet(nx, ny, w, h, 0.2, strain, &mut rng);
        let cols = 2 * nx + 1;
        for b in 0..2 * ny + 1 {
            let d = s.dof(b * cols, 0).unwrap();
            s.fixed[d] = true;
        }
        let d = s.dof(0, 1).unwrap();
        s.fixed[d] = true;
        s.laid = vec![interior::Laid::Pressure { sides: right_sides(nx, ny), p: -sigma,
                                                 depth: 2.0 }];
        let corner = (2 * ny) * cols + 2 * nx;
        let st = s.solve(&x).unwrap();
        let (e, nu) = (2e5, 0.3);
        let (e1, nu1) = if strain { (e / (1.0 - nu * nu), nu / (1.0 - nu)) } else { (e, nu) };
        let (ux, uy) = (st.u[s.dof(corner, 0).unwrap()], st.u[s.dof(corner, 1).unwrap()]);
        assert!((ux - sigma * w / e1).abs() <= 1e-10 * ux.abs(), "{ux}");
        assert!((uy + nu1 * sigma * h / e1).abs() <= 1e-10 * ux.abs(), "{uy}");
        for (i, t) in s.tris.iter().enumerate() {
            let u: [f64; 12] = std::array::from_fn(|k| st.u[s.dof(t.n[k / 2], k % 2).unwrap()]);
            for sn in t.stresses(&s.tri_places(i, &x), &u).unwrap() {
                assert!((sn[0] - sigma).abs() <= 1e-9 * sigma, "{sn:?}");
                assert!(sn[1].abs() <= 1e-9 * sigma && sn[2].abs() <= 1e-9 * sigma, "{sn:?}");
            }
        }
    }
}

/// **What a plane element is** (#510 S2): its stiffness moves nothing in the three rigid
/// motions, and only them — held at three moves it gives a displacement, at two it is free; its
/// mass weighs `ρ t A` along each axis; its area is `∫ det J`, exact for a curved side; and a
/// load laid on the plane lays its whole: a body load `b t A`, a force spread over the area or
/// along sides `F`, a pressure `p t` times the side's chord turned.  Turned over, it is
/// `Collapsed`.
#[test]
fn a_plane_element_is_rigid_in_three_motions_and_weighs_its_area() {
    let mut rng = Rng::new(3);
    let (mut s, x) = sheet(1, 1, 30.0, 10.0, 0.0, false, &mut rng);
    s.tris.truncate(1);
    let t = s.tris[0];
    let p = s.tri_places(0, &x);
    let k = t.stiffness(&p).unwrap();
    let rigid = |c: usize| -> [f64; 12] {
        std::array::from_fn(|i| {
            let (n, axis) = (i / 2, i % 2);
            match c {
                0 | 1 => if axis == c { 1.0 } else { 0.0 },
                _ => if axis == 0 { -p[n][1] } else { p[n][0] },
            }
        })
    };
    let top = k.iter().flatten().fold(0.0f64, |m, v| m.max(v.abs()));
    for c in 0..3 {
        let r = rigid(c);
        for row in &k {
            let f: f64 = (0..12).map(|i| row[i] * r[i]).sum();
            assert!(f.abs() <= 1e-10 * top * 30.0, "motion {c}: {f}");
        }
    }
    // held at three moves of its corners it has a displacement; at two it turns (the sheet's
    // nodes no element reaches are held)
    let mut held = s.clone();
    for n in (0..held.nodes).filter(|n| !t.n.contains(n)) {
        held.fixed[2 * n] = true;
        held.fixed[2 * n + 1] = true;
    }
    for (n, c) in [(t.n[0], 0), (t.n[0], 1), (t.n[1], 1)] {
        let d = held.dof(n, c).unwrap();
        held.fixed[d] = true;
    }
    let d = held.dof(t.n[2], 0).unwrap();
    held.load[d] = 1.0;
    assert!(held.solve(&x).is_ok());
    let d = held.dof(t.n[1], 1).unwrap();
    held.fixed[d] = false;
    assert_eq!(held.solve(&x).err(), Some(Failure::Free));
    // its mass and its area, straight, then with a side bowed out
    let m = t.mass(&p).unwrap();
    let area = 0.5 * 30.0 * 10.0;
    let along_x: f64 =
        (0..6).flat_map(|a| (0..6).map(move |b| (a, b))).map(|(a, b)| m[2 * a][2 * b]).sum();
    assert!((along_x - t.rho * t.depth * area).abs() <= 1e-12 * along_x);
    assert!((t.area(&p) - area).abs() <= 1e-12 * area);
    let mut bowed = p;
    bowed[3][1] -= 2.0; // the side 0–1's middle pushed out by 2: a parabola's 2/3 of its chord's 2
    assert!((t.area(&bowed) - (area + 2.0 / 3.0 * 30.0 * 2.0)).abs() <= 1e-12 * area);
    // what a load lays, in whole
    let lays = |l: interior::Laid| -> [f64; 2] {
        let mut s1 = s.clone();
        s1.laid = vec![l];
        let f = s1.forces(&x);
        [0, 1].map(|c| (0..s1.nodes).map(|n| f[s1.dof(n, c).unwrap()]).sum())
    };
    let near =
        |a: [f64; 2], b: [f64; 2]| (a[0] - b[0]).hypot(a[1] - b[1]) <= 1e-12 * b[0].hypot(b[1]);
    let body = lays(interior::Laid::Body { elements: vec![0], force: [3.0, -5.0] });
    assert!(near(body, [3.0 * t.depth * area, -5.0 * t.depth * area]), "{body:?}");
    let over = lays(interior::Laid::Over { elements: vec![0], force: [3.0, -5.0] });
    assert!(near(over, [3.0, -5.0]), "{over:?}");
    let side = [t.n[0], t.n[3], t.n[1]];
    let along = lays(interior::Laid::Along { sides: vec![side], force: [3.0, -5.0] });
    assert!(near(along, [3.0, -5.0]), "{along:?}");
    // the side runs along x with the material above it, so its outward normal is −y
    let pressed = lays(interior::Laid::Pressure { sides: vec![side], p: 4.0, depth: t.depth });
    assert!(near(pressed, [0.0, 4.0 * t.depth * 30.0]), "{pressed:?}");
    // turned over
    let mut over_p = p;
    over_p.swap(1, 2);
    assert_eq!(t.stiffness(&over_p).err(), Some(Failure::Collapsed));
}

/// **A plane structure's adjoint gradient agrees with its differences** (#510 S2): sheets of
/// knocked-about elements with their middle nodes moved off their sides (curved), in plane stress
/// and strain, held at their left edges, under every load laid on the plane at once — a pressure
/// and a force along the right edge, a force over the area and a body load — and a load at a
/// node, the output a random weighing of the moves.  Through the kernel, so its constants are
/// read back too; stated and with an unknown setting the output.
#[test]
fn a_plane_interior_agrees_with_finite_differences() {
    use gcs_core::fdcheck::fd_jacobian;
    for seed in 0..4u32 {
        let mut rng = Rng::new(seed + 120);
        let (nx, ny) = (2 + seed as usize % 2, 2);
        let (mut s, mut x) = sheet(nx, ny, 30.0, 20.0, 0.15, seed % 2 == 1, &mut rng);
        for t in &s.tris {
            for m in 3..6 {
                for c in 0..2 {
                    x[2 * t.n[m] + c] += rng.uniform(-0.3, 0.3);
                }
            }
        }
        let cols = 2 * nx + 1;
        for b in 0..2 * ny + 1 {
            for c in 0..2 {
                let d = s.dof(b * cols, c).unwrap();
                s.fixed[d] = true;
            }
        }
        let n_el = s.tris.len();
        s.laid = vec![
            interior::Laid::Pressure { sides: right_sides(nx, ny), p: 30.0, depth: 2.0 },
            interior::Laid::Along { sides: right_sides(nx, ny), force: [-40.0, 700.0] },
            interior::Laid::Over { elements: (0..n_el).step_by(2).collect(), force: [90.0, 20.0] },
            interior::Laid::Body { elements: (1..n_el).step_by(2).collect(), force: [0.5, -3.0] },
        ];
        let last = s.nodes - 1;
        let d = s.dof(last, 1).unwrap();
        s.load[d] = -500.0;
        for w in s.weight.iter_mut() {
            *w = rng.uniform(-1.0, 1.0);
        }
        for stated in [&[0.01][..], &[1.7, -0.2]] {
            let free = stated.len() == 2;
            let (n_par, n_const) = s.widths(free);
            let k = s.consts(stated);
            assert_eq!(k.len(), n_const);
            assert_eq!(Structure::read(&k, free).0, s);
            let kn = build_kernel(&Sketch::new(), KernelKey::Interior { n_par, n_const });
            let mut v = x.clone();
            if free {
                v.push(0.4);
            }
            let res = |w: &[f64]| {
                let mut r = vec![0.0; 1];
                (kn.res)(1, w, &k, &mut r);
                r
            };
            let jn = fd_jacobian(res, &v, 1e-5);
            let mut ja = vec![0.0; n_par];
            (kn.jac)(1, &v, &k, &mut ja);
            let scale = gcs_core::linalg::absmax(&jn.data);
            for (col, a) in ja.iter().enumerate() {
                let n = jn.data[col];
                assert!((a - n).abs() <= 1e-6 * scale, "seed {seed}, free {free}: column {col}: \
                                                         {a} against {n}");
            }
        }
    }
}

/* -- the solid element (#510 S3) ------------------------------------------------------------- */

/// A block of quadratic tetrahedra over `[0, w] × [0, h] × [0, d]`, `nx` by `ny` by `nz` cells
/// each cut into Kuhn's six (the paths from its low corner to its high one, so neighbours share
/// their faces' diagonals), its inner corners knocked about by `jitter` of a cell and every
/// middle node at the middle of its edge (so the elements are straight): the nodes on a grid of
/// `(2nx + 1)(2ny + 1)(2nz + 1)`, node `(a, b, c)` at `(c (2ny + 1) + b)(2nx + 1) + a`.  Nothing
/// held or loaded.
pub(crate) fn block(n: [usize; 3], size: [f64; 3], jitter: f64, rng: &mut Rng)
    -> (Structure, Vec<f64>)
{
    let g = n.map(|k| 2 * k + 1);
    let id = |p: [usize; 3]| (p[2] * g[1] + p[1]) * g[0] + p[0];
    let count = g[0] * g[1] * g[2];
    let mut x = vec![0.0; 3 * count];
    for c in (0..g[2]).step_by(2) {
        for b in (0..g[1]).step_by(2) {
            for a in (0..g[0]).step_by(2) {
                let p = [a, b, c];
                let inner = (0..3).all(|k| p[k] > 0 && p[k] + 1 < g[k]);
                for k in 0..3 {
                    let cell = size[k] / n[k] as f64;
                    let dx = if inner { rng.uniform(-jitter, jitter) * cell } else { 0.0 };
                    x[3 * id(p) + k] = p[k] as f64 * cell / 2.0 + dx;
                }
            }
        }
    }
    let place = |x: &[f64], i: usize| [x[3 * i], x[3 * i + 1], x[3 * i + 2]];
    let mut tets = Vec::new();
    for k in 0..n[2] {
        for j in 0..n[1] {
            for i in 0..n[0] {
                let low = [2 * i, 2 * j, 2 * k];
                for perm in [[0, 1, 2], [0, 2, 1], [1, 0, 2], [1, 2, 0], [2, 0, 1], [2, 1, 0]] {
                    let mut corners = [low; 4];
                    for s in 0..3 {
                        corners[s + 1] = corners[s];
                        corners[s + 1][perm[s]] += 2;
                    }
                    if tetra::volume6(corners.map(|q| place(&x, id(q)))) < 0.0 {
                        corners.swap(1, 2);
                    }
                    let mut nn = [0usize; 10];
                    for m in 0..4 {
                        nn[m] = id(corners[m]);
                    }
                    for (m, (p, q)) in EDGES.iter().enumerate() {
                        let mid: [usize; 3] =
                            std::array::from_fn(|c| (corners[*p][c] + corners[*q][c]) / 2);
                        nn[4 + m] = id(mid);
                    }
                    tets.push(interior::Tet10 { n: nn, e: 2e5, nu: 0.3, rho: 7.8e-9 });
                }
            }
        }
    }
    for t in &tets {
        for (m, (p, q)) in EDGES.iter().enumerate() {
            for c in 0..3 {
                x[3 * t.n[4 + m] + c] = 0.5 * (x[3 * t.n[*p] + c] + x[3 * t.n[*q] + c]);
            }
        }
    }
    let dofs = 3 * count;
    let s = Structure { dim: 3, nodes: count, fixed: vec![false; dofs], load: vec![0.0; dofs],
                        weight: vec![0.0; dofs], tets, ..Structure::default() };
    (s, x)
}

/// The block's boundary facets whose corners all pass `on`, each `[a, b, c, ab, bc, ca]` turned
/// out of the material (`bodies::boundary`).
fn facets_where(s: &Structure, x: &[f64], on: impl Fn([f64; 3]) -> bool) -> Vec<[usize; 6]> {
    let nodes: Vec<[f64; 3]> = x.as_chunks::<3>().0.to_vec();
    let tets: Vec<[usize; 10]> = s.tets.iter().map(|t| t.n).collect();
    gcs_core::bodies::boundary(&tets, &nodes).into_iter()
        .filter(|f| f[..3].iter().all(|&k| on(nodes[k]))).collect()
}

/// **The patch test in space** (#510 S3): a block of knocked-about quadratic tetrahedra, its
/// near face held along x and three of its corners held just enough to take out the rest of the
/// rigid motions, pulled by a pressure of `−σ` on its far face, stresses uniformly — `σx = σ`
/// and every other component nothing at every node of every element — and stretches as the
/// closed form: `σ w / E` along it, `−ν σ h / E` and `−ν σ d / E` across.
#[test]
fn a_block_pulled_stresses_uniformly_as_the_patch_test_asks() {
    let (n, size, sigma) = ([3, 2, 2], [30.0, 20.0, 10.0], 50.0);
    let mut rng = Rng::new(11);
    let (mut s, x) = block(n, size, 0.2, &mut rng);
    let place = |i: usize| [x[3 * i], x[3 * i + 1], x[3 * i + 2]];
    let at = |p: [f64; 3]| (0..s.nodes).find(|&i| sub(place(i), p).iter().all(|v| v.abs() < 1e-9))
        .unwrap();
    for i in 0..s.nodes {
        if place(i)[0].abs() < 1e-9 {
            s.fixed[3 * i] = true;
        }
    }
    let (o, top, deep) = (at([0.0; 3]), at([0.0, size[1], 0.0]), at([0.0, 0.0, size[2]]));
    for (node, c) in [(o, 1), (o, 2), (top, 2), (deep, 1)] {
        s.fixed[3 * node + c] = true;
    }
    s.borne = vec![interior::Borne::Pressure {
        facets: facets_where(&s, &x, |p| (p[0] - size[0]).abs() < 1e-9), p: -sigma }];
    let st = s.solve(&x).unwrap();
    let e = 2e5;
    let corner = at(size);
    let u: [f64; 3] = std::array::from_fn(|c| st.u[3 * corner + c]);
    let want = [sigma * size[0] / e, -0.3 * sigma * size[1] / e, -0.3 * sigma * size[2] / e];
    for c in 0..3 {
        assert!((u[c] - want[c]).abs() <= 1e-10 * want[0], "{u:?} against {want:?}");
    }
    for (i, t) in s.tets.iter().enumerate() {
        let ue: [f64; 30] = std::array::from_fn(|k| st.u[3 * t.n[k / 3] + k % 3]);
        for sn in t.stresses(&s.tet_places(i, &x), &ue).unwrap() {
            assert!((sn[0] - sigma).abs() <= 1e-9 * sigma, "{sn:?}");
            assert!(sn[1..].iter().all(|v| v.abs() <= 1e-9 * sigma), "{sn:?}");
            assert!((interior::Tet10::von_mises(sn) - sigma).abs() <= 1e-9 * sigma);
        }
    }
}

/// **What a solid element is** (#510 S3): its stiffness has exactly six motions that cost
/// nothing, the rigid ones — three slides and three turns; its mass weighs `ρ V` along each
/// axis; its volume is `∫ det J`, exact for a curved edge; and a load borne by it lays its
/// whole: a body load `b V`, a force spread through it or over a facet `F`, a pressure `p` times
/// the facet's area along its inward normal.  Turned over, it is `Collapsed`; and a stress makes
/// a geometric stiffness that stiffens under a pull along it.
#[test]
fn a_solid_element_is_rigid_in_six_motions_and_weighs_its_volume() {
    let mut rng = Rng::new(5);
    let (mut s, x) = block([1, 1, 1], [3.0, 2.0, 1.0], 0.0, &mut rng);
    s.tets.truncate(1);
    let t = s.tets[0];
    let p = s.tet_places(0, &x);
    let k = t.stiffness(&p).unwrap();
    let mut m = gcs_core::linalg::Mat::zeros(30, 30);
    for a in 0..30 {
        for b in 0..30 {
            m.set(a, b, k[a][b]);
        }
    }
    let values = gcs_core::linalg::sym_eigenvalues(&m);
    let top = values.iter().fold(0.0f64, |a, v| a.max(v.abs()));
    let zero = values.iter().filter(|v| v.abs() <= 1e-10 * top).count();
    assert_eq!(zero, 6, "{values:?}");
    for c in 0..6 {
        let r: [f64; 30] = std::array::from_fn(|i| {
            let (n, axis) = (i / 3, i % 3);
            let q = p[n];
            match c {
                0..3 => (axis == c) as u8 as f64,
                _ => cross([(c == 3) as u8 as f64, (c == 4) as u8 as f64, (c == 5) as u8 as f64],
                           q)[axis],
            }
        });
        for row in &k {
            let f: f64 = (0..30).map(|i| row[i] * r[i]).sum();
            assert!(f.abs() <= 1e-10 * top * 3.0, "motion {c}: {f}");
        }
    }
    let vol = tetra::volume6([p[0], p[1], p[2], p[3]]) / 6.0;
    assert!(vol > 0.0);
    let mass = t.mass(&p).unwrap();
    for axis in 0..3 {
        let sum: f64 = (0..10).flat_map(|a| (0..10).map(move |b| (a, b)))
            .map(|(a, b)| mass[3 * a + axis][3 * b + axis]).sum();
        assert!((sum - t.rho * vol).abs() <= 1e-12 * sum, "{sum} against {}", t.rho * vol);
    }
    assert!((t.volume(&p) - vol).abs() <= 1e-12 * vol);
    // with its middle nodes knocked about, its volume is the Bernstein coefficients' mean of the
    // volume mesh's own element (#510 P6), an independent reading of the same cubic
    let mut bowed = p;
    for q in bowed.iter_mut().skip(4) {
        for c in 0..3 {
            q[c] += rng.uniform(-0.1, 0.1);
        }
    }
    let exact = gcs_core::delaunay::tetra::Tet10 { nodes: bowed }.volume();
    assert!((t.volume(&bowed) - exact).abs() <= 1e-12 * vol, "{} against {exact}",
            t.volume(&bowed));
    // what a load lays, in whole
    let lays = |l: interior::Borne| -> [f64; 3] {
        let mut s1 = s.clone();
        s1.borne = vec![l];
        let f = s1.forces(&x);
        [0, 1, 2].map(|c| (0..s1.nodes).map(|n| f[3 * n + c]).sum())
    };
    let near = |a: [f64; 3], b: [f64; 3]| {
        let d = sub(a, b);
        dot(d, d).sqrt() <= 1e-12 * dot(b, b).sqrt()
    };
    let force = [3.0, -5.0, 2.0];
    let body = lays(interior::Borne::Body { elements: vec![0], force });
    assert!(near(body, force.map(|f| f * vol)), "{body:?}");
    assert!(near(lays(interior::Borne::Through { elements: vec![0], force }), force));
    let facet = facets_where(&s, &x, |q| q[2].abs() < 1e-9)[0];
    assert!(near(lays(interior::Borne::Across { facets: vec![facet], force }), force));
    let corners = [facet[0], facet[1], facet[2]].map(|i| [x[3 * i], x[3 * i + 1], x[3 * i + 2]]);
    let nda = cross(sub(corners[1], corners[0]), sub(corners[2], corners[0])).map(|v| v / 2.0);
    let pressed = lays(interior::Borne::Pressure { facets: vec![facet], p: 4.0 });
    assert!(near(pressed, nda.map(|v| -4.0 * v)), "{pressed:?} against {nda:?}");
    // a pull along x stiffens a move across it
    let u: [f64; 30] = std::array::from_fn(|i| if i % 3 == 0 { 1e-3 * p[i / 3][0] } else { 0.0 });
    let kg = t.geometric(&p, &u).unwrap();
    let across: [f64; 30] = std::array::from_fn(|i| if i % 3 == 1 { p[i / 3][0] } else { 0.0 });
    let energy: f64 =
        (0..30).map(|a| (0..30).map(|b| across[a] * kg[a][b] * across[b]).sum::<f64>()).sum();
    // σx = (λ + 2μ) ε along x, and the move across it is a unit slope: σx V
    let (l, mu) = t.lame();
    let want = (l + 2.0 * mu) * 1e-3 * vol;
    assert!((energy - want).abs() <= 1e-10 * want, "{energy} against {want}");
    // mirrored, it is turned over everywhere
    let over = p.map(|q| [-q[0], q[1], q[2]]);
    assert_eq!(t.stiffness(&over).err(), Some(Failure::Collapsed));
}

/// **A mesh's matrices are one pattern's, assembled in place** (#632): a solid's stiffness, mass
/// and geometric stiffness come out on the mesh's one pattern, each the sum of its elements'
/// matrices (against the same sums given as triplets); the stiffness its displacements were
/// solved with, factored or by CG, is the one `stiffness` assembles, to the bit; and the modes
/// and the buckling of the matrices in place are the triplets' own.
#[test]
fn a_meshs_matrices_are_assembled_in_place_on_one_pattern() {
    use gcs_core::cholesky::Symmetric;
    use gcs_core::eigen::{self, Opts};
    let (h, p) = (10.0, 5.0);
    let mut rng = Rng::new(17);
    let (mut s, x) = block([2, 2, 2], [20.0, 20.0, h], 0.2, &mut rng);
    for i in 0..s.nodes {
        if x[3 * i + 2].abs() < 1e-9 {
            s.fixed[3 * i..3 * i + 3].fill(true);
        }
    }
    s.borne = vec![interior::Borne::Pressure {
        facets: facets_where(&s, &x, |q| (q[2] - h).abs() < 1e-9), p }];
    let factored = s.solve_by(&x, false).unwrap();
    let k = s.stiffness(&x).unwrap();
    let m = s.mass(&x).unwrap();
    let kg = s.stressed(&x, &factored.u).unwrap();
    assert!(k.same_pattern(&m) && k.same_pattern(&kg), "one pattern");
    // the same sums as triplets, both triangles, at the equations no support holds
    let mut eq = vec![None; s.fixed.len()];
    let mut n = 0;
    for (d, held) in s.fixed.iter().enumerate() {
        if !held {
            eq[d] = Some(n);
            n += 1;
        }
    }
    let triplets = |local: &dyn Fn(usize) -> [[f64; 30]; 30]| {
        let mut t = Vec::new();
        for (i, tet) in s.tets.iter().enumerate() {
            let e = local(i);
            let at: [Option<usize>; 30] = std::array::from_fn(|a| eq[3 * tet.n[a / 3] + a % 3]);
            for a in 0..30 {
                for b in 0..30 {
                    if let (Some(ea), Some(eb)) = (at[a], at[b]) {
                        t.push((ea, eb, e[a][b]));
                    }
                }
            }
        }
        Symmetric::from_triplets(n, &t)
    };
    let places = |i: usize| s.tet_places(i, &x);
    let tk = triplets(&|i| s.tets[i].stiffness(&places(i)).unwrap());
    let tm = triplets(&|i| s.tets[i].mass(&places(i)).unwrap());
    let tg = triplets(&|i| {
        let u: [f64; 30] = std::array::from_fn(|a| factored.u[3 * s.tets[i].n[a / 3] + a % 3]);
        s.tets[i].geometric(&places(i), &u).unwrap()
    });
    let v: Vec<f64> = (0..n).map(|_| rng.uniform(-1.0, 1.0)).collect();
    for (what, got, want) in [("K", &k, &tk), ("M", &m, &tm), ("K_G", &kg, &tg)] {
        let (a, b) = (got.mul_vec(&v), want.mul_vec(&v));
        let scale = b.iter().fold(0.0f64, |s, w| s.max(w.abs()));
        for i in 0..n {
            assert!((a[i] - b[i]).abs() <= 1e-12 * scale, "{what} v at {i}: {} against {}",
                    a[i], b[i]);
        }
    }
    // the stiffness each solve was made with, read again by no assembly
    let cg = s.solve_by(&x, true).unwrap();
    assert!(cg.by_cg() && !factored.by_cg());
    for st in [&factored, &cg] {
        let held = st.stiffness().expect("a mesh's solve holds its stiffness");
        assert!(held.same_pattern(&k) && held.values == k.values, "the solve's own stiffness");
    }
    // the pencils in place against the triplets'
    let close = |a: &[f64], b: &[f64], what: &str| {
        assert_eq!(a.len(), b.len(), "{what}");
        for (a, b) in a.iter().zip(b) {
            assert!((a - b).abs() <= 1e-9 * b.abs(), "{what}: {a} against {b}");
        }
    };
    let opts = Opts::default();
    let modes = eigen::lowest(&k, &m, 4, &opts).unwrap();
    close(&modes.values, &eigen::lowest(&tk, &tm, 4, &opts).unwrap().values, "modes");
    let buckled = eigen::buckling(&k, &kg, 1, &opts).unwrap();
    assert_eq!(buckled.values.len(), 1, "a pressed block buckles");
    close(&buckled.values, &eigen::buckling(&tk, &tg, 1, &opts).unwrap().values, "buckling");
}

/// **A solid structure's adjoint gradient agrees with its differences** (#510 S3): blocks of
/// knocked-about elements with their middle nodes moved off their edges (curved), held at their
/// near faces, under every load a solid bears at once — a pressure on the far face, a force
/// spread over the top, a force through half the elements and a body load through the rest —
/// and a load at a node, the output a random weighing of the moves.  Through the kernel, so its
/// constants are read back too; stated and with an unknown setting the output.
#[test]
fn a_solid_interior_agrees_with_finite_differences() {
    use gcs_core::fdcheck::fd_jacobian;
    for seed in 0..2u32 {
        let mut rng = Rng::new(seed + 140);
        let size = [30.0, 12.0, 10.0];
        let (mut s, mut x) = block([2 + seed as usize, 1, 1], size, 0.15, &mut rng);
        for t in &s.tets {
            for m in 4..10 {
                for c in 0..3 {
                    x[3 * t.n[m] + c] += rng.uniform(-0.2, 0.2);
                }
            }
        }
        let place = |x: &[f64], i: usize| [x[3 * i], x[3 * i + 1], x[3 * i + 2]];
        for i in 0..s.nodes {
            if place(&x, i)[0].abs() < 1e-9 {
                for c in 0..3 {
                    s.fixed[3 * i + c] = true;
                }
            }
        }
        let n_el = s.tets.len();
        let far = facets_where(&s, &x, |p| (p[0] - size[0]).abs() < 1e-9);
        let top = facets_where(&s, &x, |p| (p[1] - size[1]).abs() < 1e-9);
        s.borne = vec![
            interior::Borne::Pressure { facets: far, p: 30.0 },
            interior::Borne::Across { facets: top, force: [-40.0, 700.0, 20.0] },
            interior::Borne::Through { elements: (0..n_el).step_by(2).collect(),
                                       force: [90.0, 20.0, -10.0] },
            interior::Borne::Body { elements: (1..n_el).step_by(2).collect(),
                                    force: [0.5, -3.0, 1.0] },
        ];
        let last = s.nodes - 1;
        s.load[3 * last + 1] = -500.0;
        for w in s.weight.iter_mut() {
            *w = rng.uniform(-1.0, 1.0);
        }
        for stated in [&[0.01][..], &[1.7, -0.2]] {
            let free = stated.len() == 2;
            let (n_par, n_const) = s.widths(free);
            let k = s.consts(stated);
            assert_eq!(k.len(), n_const);
            assert_eq!(Structure::read(&k, free).0, s);
            let kn = build_kernel(&Sketch::new(), KernelKey::Interior { n_par, n_const });
            let mut v = x.clone();
            if free {
                v.push(0.4);
            }
            let res = |w: &[f64]| {
                let mut r = vec![0.0; 1];
                (kn.res)(1, w, &k, &mut r);
                r
            };
            let jn = fd_jacobian(res, &v, 1e-5);
            let mut ja = vec![0.0; n_par];
            (kn.jac)(1, &v, &k, &mut ja);
            let scale = gcs_core::linalg::absmax(&jn.data);
            for (col, a) in ja.iter().enumerate() {
                let n = jn.data[col];
                assert!((a - n).abs() <= 1e-6 * scale, "seed {seed}, free {free}: column {col}: \
                                                         {a} against {n}");
            }
        }
    }
}
