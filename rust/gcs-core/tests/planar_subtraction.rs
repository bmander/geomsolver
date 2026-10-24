//! **The planar subtraction class** (#606, #607, #590): a sweep of prisms under a motion that turns
//! their plane in itself, cut from a blank as the blank less the region the tool's section sweeps
//! (`envelope::swept_region`), slab by slab.  The case is the radiator bot's wheel: a valve's
//! plus-shaped star handle (arms 30 mm long, 8 mm wide) turning about `stem`, seen from a wheel 50 mm
//! away turning about the parallel `spindle` at 4:6, so that over 540° of the handle the wheel turns
//! once and the handle stands where it began.  The generating class refuses every spelling of it
//! (E1 over a whole period, E2 for one arm, T1 for a union or a draft); this class cuts each.
//!
//! What is removed is held to arithmetic made apart from the core (`kept_area`: the plus's signed
//! distance and the two turns, read ray by ray from the wheel's centre), to the long form's bites
//! (#589: the handle stood at a run of rolls and each copy cut, which keeps a scallop more), and to
//! the field, whose agreement gates every export.
use gcs_core::brep::{export, props, sweep::Say};
use gcs_core::program::{self, Code, Elaborated};
use gcs_core::solid::admission::{admit_body, Class, Condition, Error, Options};
use gcs_core::solid::export::Tolerance;
use gcs_core::solve::{solve, SolveOpts};
use std::f64::consts::PI;

/// The #585 document: the stem, the spindle 50 mm off it, the plus on `std.top`, the wheel's rim
/// 40 mm about the spindle; `tool` declares `star`, and `space` sweeps it and cuts `wheel`.
fn wheel(tool: &str, space: &str) -> String {
    format!("\
unit mm
use std
use std (vertical, turns, coords)

in std.front {{
  construction stem := vertical line(std.origin, hint((0, 10)))
  length(10mm) stem
  wc := point hint((-50, 0))
  std.origin distance(-50mm, along: u) wc
  construction spindle := vertical line(wc, hint((-50, 10)))
  length(10mm) spindle
}}
in std.top {{
  repeat 4 as k {{
    a := point hint((30 * cos(90deg * k) - 4 * sin(90deg * k), 30 * sin(90deg * k) + 4 * cos(90deg * k)))
    b := point hint((30 * cos(90deg * k) + 4 * sin(90deg * k), 30 * sin(90deg * k) - 4 * cos(90deg * k)))
    c := point hint((4 * cos(90deg * k + 45deg) * sqrt(2), 4 * sin(90deg * k + 45deg) * sqrt(2)))
    a coords(du: 30mm * cos(90deg * k) - 4mm * sin(90deg * k), dv: 30mm * sin(90deg * k) + 4mm * cos(90deg * k)) std.top
    b coords(du: 30mm * cos(90deg * k) + 4mm * sin(90deg * k), dv: 30mm * sin(90deg * k) - 4mm * cos(90deg * k)) std.top
    c coords(du: 4mm * sqrt(2) * cos(90deg * k - 45deg), dv: 4mm * sqrt(2) * sin(90deg * k - 45deg)) std.top
  }}
  plus := face(c[0], b[0], a[0], c[1], b[1], a[1], c[2], b[2], a[2], c[3], b[3], a[3], -> close)
  wo := point
  wo coincident wc
  rim := circle(center: wo) hint(r: 40)
  radius(40mm) rim
}}
{tool}
tool_turn := frame
tool_turn turns(1) stem
wheel_turn := frame
wheel_turn turns(-4 / 6) spindle
blank := solid(face(rim), from: -8mm, to: -2mm)
wheel := solid(blank)
{space}
")
}

const STAR: &str = "star := solid(plus, from: -10mm, to: 0mm)";
const DRAFTED: &str = "star := solid(plus, from: -10mm, to: 0mm, draft: 8deg)";
const PERIOD: &str = "space := sweep(star over t in (0deg, 540deg), under: tool_turn, seen_from: wheel_turn)\nspace cut wheel";

fn solved(src: &str) -> (Elaborated, usize) {
    let mut e = crate::common::build(src);
    let r = solve(&mut e.sketch, SolveOpts::default());
    assert!(r.success, "{}", r.message);
    let wheel = e.map.ent_named("wheel").unwrap().i();
    (e, wheel)
}

fn say() -> Say<'static> { Say { stage: &|_: &str| {}, mark: &|_| {} } }

/// The wheel built exactly, its STL written within `held` and agreeing with the field: its volume.
fn cut(src: &str, held: Option<Tolerance>) -> f64 {
    let (e, wheel) = solved(src);
    let exact = export::exact(&e.sketch, wheel, None, held, &say())
        .unwrap_or_else(|r| panic!("refused at {:?}: {}", r.stage, r.message));
    exact.solid.check(1e-6).unwrap();
    export::stl(&e.sketch, wheel, &exact, held, &say())
        .unwrap_or_else(|r| panic!("the STL refused at {:?}: {}", r.stage, r.message));
    props::volume(&exact.solid)
}

/// The plus's signed distance: two bars, 60 by 8, crossing at the origin.
fn plus(q: [f64; 2]) -> f64 {
    let (x, y) = (q[0].abs(), q[1].abs());
    let bar = |w: f64, h: f64| {
        let (dx, dy) = (x - w, y - h);
        dx.max(0.).hypot(dy.max(0.)) + dx.max(dy).min(0.)
    };
    bar(30., 4.).min(bar(4., 30.))
}

/// A point of the wheel (about its centre, in its own frame) carried into the handle's at roll `t`:
/// the wheel turns `−2t/3` about its centre at (−50, 0), the handle `t` about the origin.
fn in_handle(p: [f64; 2], t: f64) -> [f64; 2] {
    let (s, c) = (-2. * t / 3.).sin_cos();
    let w = [-50. + c * p[0] - s * p[1], s * p[0] + c * p[1]];
    let (s, c) = (-t).sin_cos();
    [c * w[0] - s * w[1], s * w[0] + c * w[1]]
}

/// The least of the plus's distance along a point's path over the period: a grid of the roll, then
/// a golden-section search about its least.
fn least(p: [f64; 2]) -> f64 {
    let n = 8640;
    let at = |k: usize| 3. * PI * k as f64 / n as f64;
    let (k, v) = (0..n).map(|k| (k, plus(in_handle(p, at(k))))).min_by(|a, b| a.1.total_cmp(&b.1)).unwrap();
    let (mut lo, mut hi) = (at(k) - 3. * PI / n as f64, at(k) + 3. * PI / n as f64);
    for _ in 0..80 {
        let (a, b) = (lo + 0.382 * (hi - lo), lo + 0.618 * (hi - lo));
        if plus(in_handle(p, a)) < plus(in_handle(p, b)) { hi = b } else { lo = a }
    }
    v.min(plus(in_handle(p, 0.5 * (lo + hi))))
}

/// **What the period leaves of the rim's disc, apart from the core**: along each ray from the
/// wheel's centre, every change between kept and cut found (a grid of radii, then bisection),
/// `∫ r dr` over what is kept, integrated over the ray's angle across a twelfth of the turn (the
/// wheel is six pockets, each mirrored about its middle).
fn kept_area(rays: usize) -> f64 {
    let ray = |alpha: f64| {
        let u = [alpha.cos(), alpha.sin()];
        let kept = |r: f64| least([r * u[0], r * u[1]]) >= 0.;
        let n = 200;
        let rs: Vec<f64> = (0..=n).map(|k| 40. * k as f64 / n as f64).collect();
        let marks: Vec<bool> = rs.iter().map(|&r| kept(r)).collect();
        let (mut area, mut last, mut inside) = (0., 0., marks[0]);
        for k in 0..n {
            if marks[k] == marks[k + 1] { continue }
            let (mut lo, mut hi) = (rs[k], rs[k + 1]);
            for _ in 0..40 {
                let m = 0.5 * (lo + hi);
                if kept(m) == marks[k] { lo = m } else { hi = m }
            }
            let e = 0.5 * (lo + hi);
            if inside { area += 0.5 * (e * e - last * last) }
            (inside, last) = (!inside, e);
        }
        if inside { area += 0.5 * (40. * 40. - last * last) }
        area
    };
    let alphas: Vec<f64> = (0..=rays).map(|k| PI / 6. * k as f64 / rays as f64).collect();
    let f: Vec<f64> = gcs_core::par::map(&alphas, |&a| ray(a));
    12. * (0..rays).map(|k| 0.5 * (f[k] + f[k + 1]) * (alphas[k + 1] - alphas[k])).sum::<f64>()
}

/// **The star cuts its wheel over a whole period** (#585): no roll of 540° is clear of the blank,
/// and none need be — the period's region closes, so nothing is capped.  Admitted to this class (the
/// generating class refuses it at E1), one slab, and built: the region's two loops within the
/// blank's box, its wheel's teeth and an overhang either side of each pocket's mouth that the arms
/// leave as they swing out.  Its volume is the disc's kept area times the 6 mm slab.
#[test]
fn the_star_cuts_its_wheel_over_a_whole_period() {
    let src = wheel(STAR, PERIOD);
    let (e, w) = solved(&src);
    let admitted = admit_body(&e.sketch, w, &Options::default()).unwrap_or_else(|f| panic!("{f}"));
    let [sweep] = admitted.sweeps() else { panic!("one sweep") };
    let Class::Subtraction(found) = &sweep.class else { panic!("{:?}", sweep.class) };
    assert_eq!(found.slabs.len(), 1);
    let v = cut(&src, None);
    let want = 6. * kept_area(90);
    assert!((v - want).abs() < 2e-4 * want, "{v:.4} mm³ against {want:.4}");
}

/// **The long form keeps a scallop more** (#589): the star stood every 6° and each copy cut keeps
/// what lies between its bites; the continuous cut is below it, by less than the material a 6°
/// step leaves between bites.
#[test]
fn the_long_form_keeps_a_scallop_more() {
    let bites = "wheel_cut := solid(blank)\nrepeat 90 as j {\n  bite := star at(under: tool_turn, seen_from: wheel_turn) (j * 6deg)\n  bite cut wheel_cut\n}";
    let (e, _) = solved(&wheel(STAR, bites));
    let long = e.map.ent_named("wheel_cut").unwrap().i();
    let built = export::exact(&e.sketch, long, None, None, &say()).unwrap_or_else(|r| panic!("{}", r.message));
    let long = props::volume(&built.solid);
    let v = cut(&wheel(STAR, PERIOD), None);
    assert!(v < long && long - v < 0.06 * v, "the continuous cut {v:.3} mm³, the long form's {long:.3}");
}

/// **A union of bands cuts as one** (#586): two bands of the same plus, stacked, are the one band
/// slab for slab; the slabs either side of the plane they meet at hold the same section and are read
/// as one, and the wheel is the one band's to the last digit.
#[test]
fn two_bands_of_the_star_cut_as_the_one() {
    let bands = "upper := solid(plus, from: -5mm, to: 0mm)\nlower := solid(plus, from: -10mm, to: -5mm)\nstar := solid(upper)\nlower union star";
    let (one, two) = (cut(&wheel(STAR, PERIOD), None), cut(&wheel(bands, PERIOD), None));
    assert!((one - two).abs() <= 1e-9 * one, "{one} against {two}");
}

/// **A stepped tool cuts each slab by its own section**: the plus above the plane at −5 and a hub
/// of radius 12 below it.  Below, the hub's region is a ring 38 to 62 from the wheel's centre, so
/// the wheel there is a disc of 38; above, half the star's cut.
#[test]
fn a_stepped_tool_cuts_each_slab_by_its_own_section() {
    let stepped = "upper := solid(plus, from: -5mm, to: 0mm)\nin std.top {\n  hub := circle(center: std.top.origin) hint(r: 12)\n  \
        radius(12mm) hub\n}\nlower := solid(face(hub), from: -10mm, to: -5mm)\nstar := solid(upper)\nlower union star";
    let v = cut(&wheel(stepped, PERIOD), None);
    let want = PI * 38. * 38. * 3. + cut(&wheel(STAR, PERIOD), None) / 2.;
    assert!((v - want).abs() < 1e-4 * want, "{v:.4} mm³ against {want:.4}");
}

/// **One arm rolled through the wheel** (#587): the star within a wedge about the arm that points
/// at the wheel, its roll found through the wheel's material.  A point of the arm's side touches
/// the blank twice in its passage (E2); the region keeps only the contact that bounds it.
#[test]
fn one_arm_rolled_through_the_wheel_cuts_its_pocket() {
    let arm = format!("{STAR}
in std.top {{
  wk := point
  wk coincident std.top.origin
  wa := point hint((-60, 60))
  wb := point hint((-60, -60))
  wa coords(du: -60mm, dv: 60mm) std.top
  wb coords(du: -60mm, dv: -60mm) std.top
  wedge_f := face(wk, wa, wb, -> close)
}}
wedge := solid(wedge_f, from: -20mm, to: 10mm)
arm := solid(star)
wedge common arm");
    let src = wheel(&arm, "space := sweep(arm, through: wheel, under: tool_turn, seen_from: wheel_turn)\nspace cut wheel");
    let (e, w) = solved(&src);
    let admitted = admit_body(&e.sketch, w, &Options::default()).unwrap_or_else(|f| panic!("{f}"));
    assert!(matches!(admitted.sweeps()[0].class, Class::Subtraction(_)));
    let removed = PI * 40. * 40. * 6. - cut(&src, Some(Tolerance { millimetres: 0.01 }));
    // one pocket of the six the whole period cuts, give or take what one passage leaves at its rim
    let pocket = (PI * 40. * 40. * 6. - cut(&wheel(STAR, PERIOD), None)) / 6.;
    assert!((removed - pocket).abs() < 0.02 * pocket, "one arm removes {removed:.3} mm³, a pocket being {pocket:.3}");
}

/// **A roll short of a period is refused naming the period** (Q3): over 270° the handle ends
/// in the blank, and what the region would cap there is no face this class builds.
#[test]
fn a_roll_short_of_a_period_names_the_period() {
    let (e, w) = solved(&wheel(STAR, &PERIOD.replace("540deg", "270deg")));
    match admit_body(&e.sketch, w, &Options::default()) {
        Err(Error::Refused(r)) => {
            assert_eq!(r.condition, Condition::Closes, "{r}");
            assert!(r.to_string().contains("roll it a whole period, `over t in (0deg, 540deg)`"), "{r}");
        }
        other => panic!("{other:?}"),
    }
}

/// **A roll found through a wheel always in mesh says what to roll** (#585): the tool is never
/// clear of the wheel, so `through:` finds no roll, and says the period.
#[test]
fn a_tool_always_in_mesh_names_its_period() {
    let mut e = crate::common::build(&wheel(STAR, "space := sweep(star, through: wheel, under: tool_turn, seen_from: wheel_turn)\nspace cut wheel"));
    solve(&mut e.sketch, SolveOpts::default());
    let diags = program::solid_diagnostics(&e.sketch, &e.map, &e.program);
    assert!(diags.iter().any(|d| d.code == Code::E080 && d.message.contains("always in mesh")
        && d.message.contains("`over t in (0deg, 540deg)`")), "{diags:#?}");
}

/// **The drafted star cuts its wheel** (#607, #590): the handle's arms lean 8°, so each height of
/// the slab has its own section — the plus moved into itself `|h| tan 8°` — and its own region;
/// the slab is lofted through its stations and held to the ones between.  Narrower arms keep more of
/// the wheel than the square ones, and the 4° long form keeps a scallop more still.
#[test]
fn the_drafted_star_cuts_its_wheel() {
    let v = cut(&wheel(DRAFTED, PERIOD), Some(Tolerance { millimetres: 0.01 }));
    let square = cut(&wheel(STAR, PERIOD), None);
    assert!(v > square && v < square * 1.15, "{v} against the square arms' {square}");
}

/// **The field reads a drafted prism of any loop** (#607, #588): the plus is not convex, so its
/// drafted prism is read as the signed distance to its section at each height — at depth `w` an
/// arm's side stands `|w| tan 8°` inside where it is drawn.
#[test]
fn the_field_reads_a_drafted_plus() {
    let (e, _) = solved(&wheel(DRAFTED, PERIOD));
    let star = e.map.ent_named("star").unwrap().i();
    let field = gcs_core::solid::SpatialField::read(&e.sketch, star, 1e-9).unwrap_or_else(|m| panic!("{m}"));
    for w in [-1_f64, -5., -9.] {
        let side = 4. - w.abs() * 8f64.to_radians().tan();
        // across the arm along +u at 15 mm, on `std.top` (its normal the world's z)
        let at = |y: f64| field.value([15., y, w]);
        assert!(at(side - 1e-6) < 0. && at(side + 1e-6) > 0., "at depth {w}: {} {}", at(side - 1e-6), at(side + 1e-6));
    }
}

/// **A sweep the field cannot read is said at its statement** (#588): a solved document whose
/// swept tool is a loft — read by no field — says so where the sweep is, and what is built on it
/// follows from it.
#[test]
fn a_sweep_the_field_cannot_read_is_said_at_its_statement() {
    let mut e = crate::common::build("\
unit mm
use std
use std (vertical, turns)
in std.front {
  a0 := point
  a1 := point
  a2 := point
  a3 := point
  fix((-10, -10)) a0
  fix((10, -10)) a1
  fix((10, 10)) a2
  fix((-10, 10)) a3
  construction stem := vertical line(std.origin, hint((0, 10)))
  length(10mm) stem
}
tip := point
fix((0, 30, 2)) tip
base := face(a0, a1, a2, a3, -> close)
param s: Scalar over (0, 1)
pyramid := sweep(std.Apex(base, tip).f over s)
turn := frame
turn turns(1) stem
space := sweep(pyramid over t in (0deg, 90deg), under: turn)
");
    let r = solve(&mut e.sketch, SolveOpts::default());
    assert!(r.success, "{}", r.message);
    let diags = program::solid_diagnostics(&e.sketch, &e.map, &e.program);
    let [d] = &diags[..] else { panic!("{diags:#?}") };
    assert_eq!((d.code, d.at(e.text()).0), (Code::E080, 23), "{d:?}");
    assert!(d.message.starts_with("`space` sweeps `pyramid`, whose material the field does not read"), "{d:?}");
}

/// The wheel stood by a frame turning with it at `degrees` of its value (`x in f`), and a body made
/// from that copy (#619).
fn stood(degrees: f64) -> String {
    format!("{}held := frame\nheld turns(-4 / 6) spindle\nfix(value == {degrees}deg) held\nshown := wheel in held\ncopy := solid(shown)\n",
        wheel(STAR, PERIOD))
}

/// **A wheel a frame stands is the wheel moved** (#619): built once and its boundary carried by the
/// frame's turn, so `shown` and a body made from it export with the wheel's volume, and at 30° of
/// the handle every vertex is the wheel's turned 20° back about the spindle.  The page's build by
/// stages moves it as the export does, and the report reads all three from one B-rep, moved.
#[test]
fn a_wheel_a_frame_stands_is_the_wheel_moved() {
    for degrees in [0., 30.] {
        let src = stood(degrees);
        let (e, w) = solved(&src);
        let solid = |name: &str| e.map.ent_named(name).unwrap().i();
        let built = |i: usize| {
            let exact = export::exact(&e.sketch, i, None, None, &say())
                .unwrap_or_else(|r| panic!("{degrees}°: refused at {:?}: {}", r.stage, r.message));
            export::stl(&e.sketch, i, &exact, None, &say())
                .unwrap_or_else(|r| panic!("{degrees}°: the STL refused at {:?}: {}", r.stage, r.message));
            exact.solid
        };
        let (wheel, shown) = (built(w), built(solid("shown")));
        let v = props::volume(&wheel);
        for (name, moved) in [("shown", props::volume(&shown)), ("copy", props::volume(&built(solid("copy"))))] {
            assert!((moved - v).abs() <= 1e-9 * v, "{degrees}°: `{name}` {moved} mm³ against {v}");
        }
        // the handle's 30° is the wheel's −20°, about the spindle through (−50, 0) along z
        let (s, c) = (-2. * degrees.to_radians() / 3.).sin_cos();
        for (a, b) in wheel.vertices.iter().zip(&shown.vertices) {
            let (x, y) = (a.p[0] + 50., a.p[1]);
            let want = [c * x - s * y - 50., s * x + c * y, a.p[2]];
            assert!((0..3).all(|k| (b.p[k] - want[k]).abs() < 1e-9), "{degrees}°: {:?} against {want:?}", b.p);
        }
        let display = |i: usize| {
            let mut builder = export::Builder::new(&e.sketch, i).unwrap_or_else(|r| panic!("{}", r.message));
            while !builder.step().unwrap_or_else(|r| panic!("{degrees}°: {}", r.message)) {}
            builder.display().unwrap().volume
        };
        let shown_at = display(solid("shown"));
        for name in ["wheel", "copy"] {
            let volume = display(solid(name));
            assert!((volume - shown_at).abs() <= 1e-9 * volume, "{degrees}°: `{name}`'s display {volume} against {shown_at}");
        }
        let names = ["wheel", "shown", "copy"];
        gcs_core::brep::export::supply_exact_where(&e.sketch, &|i| names.contains(&e.sketch.solids[i].name.as_str()));
        let report = gcs_core::report::positions_where(&e.sketch, &e.map, &|n| names.contains(&n));
        let read = |name: &str| report.iter().find(|(k, _)| *k == format!("{name}.volume")).map(|r| r.1).unwrap();
        for name in ["shown", "copy"] {
            assert!((read(name) - read("wheel")).abs() <= 1e-6 * v, "{degrees}°: `{name}` reads {} against {}", read(name), read("wheel"));
        }
    }
}

/// What `solventc --where` reads of solid `name`: its exact B-rep supplied where a sweep builds
/// one, then the report's positions under the name.
fn read(e: &Elaborated, name: &str) -> Vec<(String, f64)> {
    let asked = |n: &str| n == name || n.starts_with(&format!("{name}."));
    export::supply_exact_where(&e.sketch, &|i| asked(&e.sketch.solids[i].name));
    gcs_core::report::positions_where(&e.sketch, &e.map, &asked).into_iter()
        .filter(|(n, _)| asked(n)).collect()
}

/// **A Boolean of the cut wheel that leaves no material is empty, and its volume is 0** (#618):
/// the star the wheel was cut by shares nothing with it, so the common of the two is built with no
/// faces — a body with no material, read as a static one's empty Boolean is, and not a solid that
/// failed to build, whose name would read nothing.
#[test]
fn a_swept_bodys_empty_boolean_reads_volume_zero() {
    let space = format!("{PERIOD}\noverlap := solid(wheel)\nstar common overlap");
    let (e, _) = solved(&wheel(STAR, &space));
    let said = read(&e, "overlap");
    let at = |n: &str| said.iter().find(|(k, _)| k == n).map(|r| r.1);
    assert_eq!((at("overlap.volume"), at("overlap.area")), (Some(0.0), Some(0.0)), "{said:?}");
    assert!(!said.iter().any(|(n, _)| n.starts_with("overlap.bounds")), "{said:?}");
}
