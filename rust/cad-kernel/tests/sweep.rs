//! A solid sweep through the kernel alone (`sweep::cut`): a tool, a rigid motion, a roll and a
//! blank, built from plain data, cut exactly, and held to closed forms — or refused by the row of
//! the class it fails. (The solver's suite holds the same entry point to its own pipeline, sweep for
//! sweep: `rust/gcs-core/tests/swept_api.rs`.)
use cad_kernel::boolean::{combined, Op};
use cad_kernel::build::{disc, prism, Profile, ProfileEdge};
use cad_kernel::props::{area, volume};
use cad_kernel::sweep::cut::{swept_cut, swept_cut_with, swept_solid, swept_solid_with, Ask, Motion, Tool};
use cad_kernel::sweep::refusal::{ExportRefusal, Stage};
use cad_kernel::sweep::Say;
use cad_kernel::topo::Brep;
use std::f64::consts::{PI, TAU};

fn said() -> Say<'static> { Say { stage: &|l: &str| eprintln!("{l}"), mark: &|_| {} } }

fn measured(b: &Brep) -> (f64, f64) { (volume(b), b.faces.iter().map(|f| area(b, f)).sum()) }

/// The area of the lens two circles of radii `r1` and `r2`, `d` apart, share.
fn lens(r1: f64, r2: f64, d: f64) -> f64 {
    let a1 = ((d * d + r1 * r1 - r2 * r2) / (2. * d * r1)).acos();
    let a2 = ((d * d + r2 * r2 - r1 * r1) / (2. * d * r2)).acos();
    r1 * r1 * (a1 - a1.sin() * a1.cos()) + r2 * r2 * (a2 - a2.sin() * a2.cos())
}

/// `got` within `bar` of `want`, relatively.
fn near(what: &str, got: f64, want: f64, bar: f64) {
    eprintln!("{what}: {got:.12} against {want:.12} ({:.2e})", (got - want) / want);
    assert!((got - want).abs() <= bar * want.abs(), "{what}: {got} against {want}");
}

/// The rod the slides groove: radius 5 about z, from z = 0 to 20.
fn rod() -> Brep { prism(&disc([0., 0., 0.], [0., 0., 1.], 5.), 0., 20.).unwrap() }

/// A slide up z, 40 mm a whole turn of the roll.
fn up() -> Motion { Motion::Slide { direction: [0., 0., 1.], advance: 40. } }

/// The ring sector the turns groove: radii 8 to 12 about z, heights −1 to 1, 0° to 60°.
fn sector() -> Brep {
    let ring = cad_kernel::build::revolve(&polygon([0.; 3], [0., -1., 0.], &[[8., 0., -1.], [12., 0., -1.], [12., 0., 1.], [8., 0., 1.]]),
        [0.; 3], [0., 0., 1.], TAU).unwrap();
    let wedge = prism(&polygon([0., 0., -2.], [0., 0., 1.], &[[0., 0., -2.], [24., 0., -2.], [12., 12. * 3f64.sqrt(), -2.]]), 0., 4.).unwrap();
    combined(&ring, &wedge, Op::Common, 0.).unwrap()
}

/// A turn about z, a radian a radian.
fn about_z() -> Motion { Motion::Turn { origin: [0.; 3], axis: [0., 0., 1.], ratio: 1. } }

fn refused<T>(built: Result<T, ExportRefusal>) -> ExportRefusal {
    match built { Ok(_) => panic!("built, not refused"), Err(r) => { eprintln!("{}", r.message); r } }
}

/// A torus slid along a rod (brep-bench's `swept/torus-slid-through-rod`): square to the slide its
/// characteristic is its tube's circle, so the groove is the lens the rod's disc and the tube's
/// share, the rod's length. Its sheet is a fit to the characteristic's exact points, so the
/// tolerance it is held to is how near the closed form it comes: within 0.01 mm (the default), and
/// held to a micrometre.
#[test]
fn a_torus_slid_along_a_rod_takes_its_lens_the_rods_length() {
    let wheel = torus("wheel", [11.5, 0., -9.], [0., 1., 0.], [-1., 0., 0.], 6., 2.);
    let volume = PI * 25. * 20. - 20. * lens(5., 2., 5.5);
    // the rod's caps less the lenses, its wall less the lens's arc of it, and the tube's arc in the rod
    let (d, big, small) = (5.5f64, 5f64, 2f64);
    let alpha = ((d * d + big * big - small * small) / (2. * d * big)).acos();
    let beta = ((d * d + small * small - big * big) / (2. * d * small)).acos();
    let surface = 2. * (PI * 25. - lens(5., 2., 5.5)) + 20. * (TAU * big - 2. * alpha * big) + 20. * 2. * beta * small;
    for (tolerance, bars) in [(None, [3e-8, 3e-7]), (Some(1e-6), [3e-9, 1e-8])] {
        let cut = swept_cut_with(&rod(), &wheel, &up(), [0., TAU], &Ask { tolerance, ..Ask::default() }, &said())
            .unwrap_or_else(|r| panic!("{r}"));
        cut.check(1e-6).unwrap();
        let (v, a) = measured(&cut);
        near(&format!("volume within {tolerance:?}"), v, volume, bars[0]);
        near(&format!("area within {tolerance:?}"), a, surface, bars[1]);
    }
}

/// Two beads united and slid along the rod (`swept/two-beads-slid-through-rod`): a union's sweep is
/// its members', each built and the rod split by both; held to the figure three kernels agree on.
#[test]
fn two_beads_slid_take_what_either_covers() {
    let beads = Tool::Union(vec![torus("bead", [11.5, 0., -9.], [0., 1., 0.], [-1., 0., 0.], 6., 2.),
        torus("bead2", [11.5, 1.5, -9.], [0., 1., 0.], [-1., 0., 0.], 6., 1.5)]);
    let cut = swept_cut(&rod(), &beads, &up(), [0., TAU], None).unwrap_or_else(|r| panic!("{r}"));
    let (v, a) = measured(&cut);
    near("volume", v, 1486.4498667518676, 3e-8);
    near("area", a, 787.2349038041657, 3e-7);
}

/// An upright torus turned about z across the ring sector: its shadow in a half-plane through z
/// is everything within 7.5 + √(4 − z²) of the axis, so the groove is that less the sector's inner
/// 8, from z = −1 to 1, turned through the sector's 60° (Pappus). A turn's sheet is a fit across
/// the turn too, its columns ten degrees apart where no tolerance asks for more.
#[test]
fn an_upright_torus_turned_takes_its_shadow_turned() {
    let upright = torus("torus", [5., 0., 0.], [0., 0., 1.], [1., 0., 0.], 2.5, 2.);
    // ∫ ((7.5 + w)² − 8²)/2 dz over z from −1 to 1, w = √(4 − z²): ∫ w = √3 + 2π/3, ∫ w² = 22/3
    let removed = PI / 3. * (112.5 + 15. * (3f64.sqrt() + TAU / 3.) + 22. / 3. - 128.) / 2.;
    let left = PI / 3. * (144. - 64.) - removed;
    for (tolerance, bar) in [(None, 1e-5), (Some(1e-6), 1e-7)] {
        let cut = swept_cut_with(&sector(), &upright, &about_z(), [-140f64.to_radians(), 140f64.to_radians()],
            &Ask { tolerance, ..Ask::default() }, &said()).unwrap_or_else(|r| panic!("{r}"));
        near(&format!("volume within {tolerance:?}"), volume(&cut), left, bar);
    }
}

/// A torus tumbled (`swept/tumbling-torus`): turned about one axis seen from a frame turning about
/// another, swept from nothing. A solid swept with ends holds the tool at each, and the classes
/// build only what a tool clear of its blank at both ends cuts: E1, at a point of the tool where
/// the roll starts.
#[test]
fn a_tumbling_torus_is_refused_by_its_ends() {
    let small = torus("torus", [3., 0., 2.], [0., 0., 1.], [1., 0., 0.], 1., 0.5);
    let tumble = Motion::Turn { origin: [2., 0., 0.], axis: [0., 0., 1.], ratio: 0.25 }
        .seen_from(Motion::Turn { origin: [0., 0.5, 0.5], axis: [1., 0., 0.], ratio: 1. });
    let r = refused(swept_solid(&small, &tumble, [-75f64.to_radians(), 75f64.to_radians()], None));
    assert!(r.stage == Stage::Admission && r.condition.map(|c| c.code()) == Some("E1") && r.witness.is_some());
    assert!(r.message.contains("generating-sweep class"));
}

/// A whole turn has no ends, but its tool's characteristic is a closed curve wholly within any box
/// holding what it sweeps, which the walk needs as a stretch with ends: refused by the row that
/// finds it (S1 or S2), not built.
#[test]
fn a_whole_turn_swept_from_nothing_is_refused_by_its_characteristic() {
    let upright = torus("torus", [5., 0., 0.], [0., 0., 1.], [1., 0., 0.], 2.5, 2.);
    let r = refused(swept_solid_with(&upright, &about_z(), [0., TAU], &Ask::default(), &said()));
    assert!(matches!(r.condition.map(|c| c.code()), Some("S1" | "S2")), "{}", r.message);
}

/// A wheel slid past the rod, never reaching it: E0.
#[test]
fn a_tool_clear_of_the_blank_is_refused() {
    let far = torus("wheel", [31.5, 0., -9.], [0., 1., 0.], [-1., 0., 0.], 6., 2.);
    let r = refused(swept_cut(&rod(), &far, &up(), [0., TAU], None));
    assert_eq!(r.condition.map(|c| c.code()), Some("E0"));
}

/// A union within an intersection is no tool of the class (T1): a union is swept member by member
/// only as the whole tool.
#[test]
fn a_union_within_an_intersection_is_refused() {
    let beads = Tool::Union(vec![torus("bead", [11.5, 0., -9.], [0., 1., 0.], [-1., 0., 0.], 6., 2.),
        torus("bead2", [11.5, 1.5, -9.], [0., 1., 0.], [-1., 0., 0.], 6., 1.5)]);
    let tool = Tool::Intersection(vec![beads, torus("other", [11.5, 0., -9.], [0., 1., 0.], [-1., 0., 0.], 6., 2.5)]);
    let r = refused(swept_cut(&rod(), &tool, &up(), [0., TAU], None));
    assert_eq!(r.condition.map(|c| c.code()), Some("T1"));
}

/// A plus turning in mesh with a wheel through a whole period (`swept/plus-cut-wheel`): never clear
/// of the wheel, so outside the generating class (E1). Its own class, the planar subtraction
/// class, is the solver's (it reads the plus's planar envelope off its material field).
#[test]
fn a_tool_never_clear_of_the_blank_is_refused() {
    let wheel = prism(&disc([-50., 0., -8.], [0., 0., 1.], 40.), 0., 6.).unwrap();
    let (a, w) = (30., 4.);
    let plus = [[w, -w], [a, -w], [a, w], [w, w], [w, a], [-w, a], [-w, w], [-a, w], [-a, -w], [-w, -w], [-w, -a], [w, -a]]
        .map(|[x, y]| [x, y, -10.]);
    let plus = Tool::Prism { name: "plus".into(), profile: polygon([0., 0., -10.], [0., 0., 1.], &plus), from: 0., to: 10., draft: None };
    let meshing = about_z().seen_from(Motion::Turn { origin: [-50., 0., 0.], axis: [0., 0., 1.], ratio: -2. / 3. });
    let r = refused(swept_cut(&wheel, &plus, &meshing, [0., 3. * PI], None));
    assert_eq!(r.condition.map(|c| c.code()), Some("E1"));
}

fn polygon(origin: [f64; 3], normal: [f64; 3], points: &[[f64; 3]]) -> Profile {
    let n = points.len();
    Profile { origin, normal, names: Vec::new(),
        loops: vec![(0..n).map(|k| ProfileEdge::Line { a: points[k], b: points[(k + 1) % n] }).collect()] }
}

fn torus(name: &str, centre: [f64; 3], axis: [f64; 3], out: [f64; 3], ring: f64, tube: f64) -> Tool {
    let normal = [out[1] * axis[2] - out[2] * axis[1], out[2] * axis[0] - out[0] * axis[2], out[0] * axis[1] - out[1] * axis[0]];
    let at = |s: f64| [centre[0] + s * out[0], centre[1] + s * out[1], centre[2] + s * out[2]];
    Tool::Revolved { name: name.into(), profile: disc(at(ring), normal, tube), origin: centre, axis }
}

fn report(case: &str, built: Result<Brep, cad_kernel::sweep::refusal::ExportRefusal>, started: std::time::Instant) {
    match built {
        Ok(b) => { let (v, a) = measured(&b); eprintln!("CASE {case}: volume {v:.12e} area {a:.12e} faces {} ({:?})", b.faces.len(), started.elapsed()); }
        Err(r) => eprintln!("CASE {case}: refused at {:?} ({:?}): {}", r.stage, r.condition.map(|c| c.code()), r.message),
    }
}

/// The brep-bench `swept/` scenarios through the API, measured and said, not held (the ones with
/// closed forms are held above): `SWEEP_TOLERANCES=none,1e-6` lists the tolerances to build each at.
#[test]
#[ignore = "the brep-bench scenarios, measured"]
fn scenarios() {
    let rod = || prism(&disc([0., 0., 0.], [0., 0., 1.], 5.), 0., 20.).unwrap();
    let deg = |d: f64| d.to_radians();
    let tolerances: Vec<Option<f64>> = match std::env::var("SWEEP_TOLERANCES") {
        Ok(t) => t.split(',').map(|x| x.parse().ok()).collect(),
        Err(_) => vec![None],
    };
    for tolerance in tolerances {
        let ask = Ask { tolerance, ..Ask::default() };
        eprintln!("TOLERANCE {tolerance:?}");
        // torus-slid-through-rod
        let t = std::time::Instant::now();
        let wheel = torus("wheel", [11.5, 0., -9.], [0., 1., 0.], [-1., 0., 0.], 6., 2.);
        report("torus-slid-through-rod", swept_cut_with(&rod(), &wheel, &Motion::Slide { direction: [0., 0., 1.], advance: 40. },
            [0., TAU], &ask, &said()), t);
        // two-beads-slid-through-rod
        let t = std::time::Instant::now();
        let beads = Tool::Union(vec![torus("bead", [11.5, 0., -9.], [0., 1., 0.], [-1., 0., 0.], 6., 2.),
            torus("bead2", [11.5, 1.5, -9.], [0., 1., 0.], [-1., 0., 0.], 6., 1.5)]);
        report("two-beads-slid-through-rod", swept_cut_with(&rod(), &beads, &Motion::Slide { direction: [0., 0., 1.], advance: 40. },
            [0., TAU], &ask, &said()), t);
        // torus-screwed-through-rod
        let t = std::time::Instant::now();
        let h = (TAU * 5.5 / 30.).atan();
        let wheel = torus("wheel", [11.5, 0., 0.], [0., h.cos(), -h.sin()], [-1., 0., 0.], 6., 2.);
        let screw = Motion::Screw { origin: [0.; 3], axis: [0., 0., 1.], ratio: 1., advance: 30. };
        report("torus-screwed-through-rod", swept_cut_with(&rod(), &wheel, &screw, [deg(-108.), deg(348.)], &ask, &said()), t);
        // v-wheel-screwed-through-rod
        let t = std::time::Instant::now();
        let h = (TAU * 4.5 / 30.).atan();
        let along = [0., h.cos(), -h.sin()];
        let side = 2.5 * (PI / 6.).tan();
        let corner = |s: f64| [6.5, s * along[1], s * along[2]];
        let normal = [0., h.sin(), h.cos()];
        let vwheel = Tool::Revolved { name: "vwheel".into(), profile: polygon([4., 0., 0.], normal, &[[4., 0., 0.], corner(-side), corner(side)]),
            origin: [14., 0., 0.], axis: along };
        report("v-wheel-screwed-through-rod", swept_cut_with(&rod(), &vwheel, &screw, [deg(-144.), deg(384.)], &ask, &said()), t);
        // torus-turned-across-block
        let t = std::time::Instant::now();
        let ring = cad_kernel::build::revolve(&polygon([0.; 3], [0., -1., 0.], &[[8., 0., -1.], [12., 0., -1.], [12., 0., 1.], [8., 0., 1.]]),
            [0.; 3], [0., 0., 1.], TAU).unwrap();
        let wedge = prism(&polygon([0., 0., -2.], [0., 0., 1.], &[[0., 0., -2.], [24., 0., -2.], [12., 12. * 3f64.sqrt(), -2.]]), 0., 4.).unwrap();
        let block = combined(&ring, &wedge, Op::Common, 0.).unwrap();
        let lean = deg(25.);
        let cutter = torus("cutter", [5., 0., 0.], [0., lean.sin(), lean.cos()], [1., 0., 0.], 2.5, 2.);
        report("torus-turned-across-block", swept_cut_with(&block, &cutter, &Motion::Turn { origin: [0.; 3], axis: [0., 0., 1.], ratio: 1. },
            [deg(-140.), deg(140.)], &ask, &said()), t);
        // rack-cut-tooth-space
        let t = std::time::Instant::now();
        let blank = prism(&disc([0.; 3], [0., 1., 0.], 22.), 0., 6.).unwrap();
        let tooth = Tool::Prism { name: "tooth".into(), profile: polygon([0.; 3], [0., 1., 0.],
            &[[18., 0., -0.842856], [18., 0., 0.842856], [24., 0., 3.026666], [24., 0., -3.026666]]), from: -2., to: 8., draft: None };
        let rack = Motion::Slide { direction: [0., 0., 1.], advance: TAU * 20. }
            .seen_from(Motion::Turn { origin: [0.; 3], axis: [0., -1., 0.], ratio: 1. });
        report("rack-cut-tooth-space", swept_cut_with(&blank, &tooth, &rack, [deg(-60.), deg(60.)], &ask, &said()), t);
        // plus-cut-wheel
        let t = std::time::Instant::now();
        let wheel = prism(&disc([-50., 0., -8.], [0., 0., 1.], 40.), 0., 6.).unwrap();
        let (a, w) = (30., 4.);
        let plus = [[w, -w], [a, -w], [a, w], [w, w], [w, a], [-w, a], [-w, w], [-a, w], [-a, -w], [-w, -w], [-w, -a], [w, -a]]
            .map(|[x, y]| [x, y, -10.]);
        let plus = Tool::Prism { name: "plus".into(), profile: polygon([0., 0., -10.], [0., 0., 1.], &plus), from: 0., to: 10., draft: None };
        let meshing = Motion::Turn { origin: [0.; 3], axis: [0., 0., 1.], ratio: 1. }
            .seen_from(Motion::Turn { origin: [-50., 0., 0.], axis: [0., 0., 1.], ratio: -2. / 3. });
        report("plus-cut-wheel", swept_cut_with(&wheel, &plus, &meshing, [0., deg(540.)], &ask, &said()), t);
        // tumbling-torus
        let t = std::time::Instant::now();
        let small = torus("torus", [3., 0., 2.], [0., 0., 1.], [1., 0., 0.], 1., 0.5);
        let tumble = Motion::Turn { origin: [2., 0., 0.], axis: [0., 0., 1.], ratio: 0.25 }
            .seen_from(Motion::Turn { origin: [0., 0.5, 0.5], axis: [1., 0., 0.], ratio: 1. });
        report("tumbling-torus", swept_solid_with(&small, &tumble, [deg(-75.), deg(75.)], &ask, &said()), t);
        // a whole turn of an upright torus, swept from nothing
        let t = std::time::Instant::now();
        let upright = torus("torus", [5., 0., 0.], [0., 0., 1.], [1., 0., 0.], 2.5, 2.);
        report("whole-turn-solid", swept_solid_with(&upright, &Motion::Turn { origin: [0.; 3], axis: [0., 0., 1.], ratio: 1. },
            [0., TAU], &ask, &said()), t);
        // an upright torus turned across the block
        let t = std::time::Instant::now();
        report("upright-turned-across-block", swept_cut_with(&block, &upright, &Motion::Turn { origin: [0.; 3], axis: [0., 0., 1.], ratio: 1. },
            [deg(-140.), deg(140.)], &ask, &said()), t);
        // a tool clear of the blank
        let t = std::time::Instant::now();
        let far = torus("wheel", [31.5, 0., -9.], [0., 1., 0.], [-1., 0., 0.], 6., 2.);
        report("clear", swept_cut_with(&rod(), &far, &Motion::Slide { direction: [0., 0., 1.], advance: 40. }, [0., TAU], &ask, &said()), t);
    }
}
