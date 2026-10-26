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
use std.place (vertical, coords)
use std.motion (turns)

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

fn solved(src: &str) -> (Elaborated, usize) { solved_body(src, "wheel") }

/// `solved`, with the body named `body`.
fn solved_body(src: &str, body: &str) -> (Elaborated, usize) {
    let mut e = crate::common::build(src);
    let r = solve(&mut e.sketch, SolveOpts::default());
    assert!(r.success, "{}", r.message);
    let id = e.map.ent_named(body).unwrap().i();
    (e, id)
}

fn say() -> Say<'static> { Say { stage: &|_: &str| {}, mark: &|_| {} } }

/// The wheel built exactly, its STL written within `held` and agreeing with the field: its volume.
fn cut(src: &str, held: Option<Tolerance>) -> f64 { cut_body(src, "wheel", held) }

/// `cut`, of the body named `body`.
fn cut_body(src: &str, body: &str, held: Option<Tolerance>) -> f64 {
    let (e, wheel) = solved_body(src, body);
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

/// The radiator valve's star handle as measured (#612): four arms with straight sides, tips of
/// 3.25 mm and fillets of 4 mm between them, `span` across the tips and `waist` across opposite
/// fillets, on a plane 1.1 mm under the top; a wheel of 46.1 mm 60 mm off the stem, 15 mm thick.
/// `tool` declares `star` from `StarOutline`s.
fn handle(tool: &str) -> String {
    format!("\
unit mm
use std
use std.place (vertical)
use std.motion (turns)

interface StarSizes {{
  span: Length, waist: Length, tip_r: Length, fillet_r: Length
}}
component StarOutline(top: plane, s: StarSizes) {{
  in top {{
    hub := point
    hub coincident top.origin
    ring 4 about hub {{
      tip_c := point hint((s.span / 2 - s.tip_r, 0))
      fillet_c := point hint(((s.waist / 2 + s.fillet_r) / sqrt(2), (s.waist / 2 + s.fillet_r) / sqrt(2)))
      apex := point hint((s.span / 2, 0))
      hub distance(s.waist / 2 + s.fillet_r) fillet_c
      line(fillet_c, tip_c) equal line(fillet_c, next.tip_c)
      ccw(hub, tip_c, fillet_c)
      apex coincident(t == s.span / (s.span - 2 * s.tip_r)) line(hub, tip_c)
      p1 := point hint((tip_c.x, s.tip_r))
      p2 := point hint((fillet_c.x - 0.2 * s.fillet_r, fillet_c.y - 0.98 * s.fillet_r))
      p3 := point hint((fillet_c.x - 0.98 * s.fillet_r, fillet_c.y - 0.2 * s.fillet_r))
      p4 := point hint((s.tip_r, tip_c.x))
      tip_half := arc(center: tip_c, start: apex, end: p1)
      side := line(p1, p2)
      fillet := arc(center: fillet_c, start: p3, end: p2)
      next_side := line(p3, p4)
      next_tip_half := arc(center: next.tip_c, start: p4, end: next.apex)
      radius(s.fillet_r) fillet
      side tangent(at: p1) tip_half
      side tangent(at: p2) fillet
      next_side tangent(at: p1) fillet
      next_side tangent(at: p2) next_tip_half
      ccw(p1, p2, tip_c)
      cw(p1, p2, fillet_c)
      ccw(p3, p4, next.tip_c)
      cw(p3, p4, fillet_c)
    }}
    hub distance(s.span / 2 - s.tip_r, along: u) tip_c[0]
    hub level(v) tip_c[0]
    outline := face(tip_half[0], side[0], fillet[0], next_side[0], next_tip_half[0],
                    tip_half[1], side[1], fillet[1], next_side[1], next_tip_half[1],
                    tip_half[2], side[2], fillet[2], next_side[2], next_tip_half[2],
                    tip_half[3], side[3], fillet[3], next_side[3], next_tip_half[3])
  }}
}}

in std.front {{
  construction stem := vertical line(std.origin, hint((0, 10)))
  length(10mm) stem
  wc := point hint((-60, 0))
  std.origin distance(-60mm, along: u) wc
  construction spindle := vertical line(wc, hint((-60, 10)))
  length(10mm) spindle
}}
top := plane
fix(origin == (0mm, 0mm, -1.1mm)) top
fix(dir == (1, 0, 0)) top.u
fix(dir == (0, 1, 0)) top.v
{tool}
in std.top {{
  wo := point
  wo coincident wc
  rim := radius(46.1mm) circle(center: wo)
}}
handle_turn := frame
handle_turn turns(1) stem
wheel_turn := frame
wheel_turn turns(-4 / 6) spindle
space := sweep(star over t in (0deg, 540deg), under: handle_turn, seen_from: wheel_turn)
wheel := solid(face(rim), from: -15.6mm, to: -0.6mm)
space cut wheel
")
}

/// The handle's outline moved `d` into itself, as an 8° draft moves it at depth `d / tan 8°`:
/// the tips' radius less `d`, the fillets' more, their centres where they were.
fn outline(name: &str, d: f64) -> String {
    format!("{name} := StarOutline(top, s: {{span: {}mm, waist: {}mm, tip_r: {}mm, fillet_r: {}mm}})",
        74.2 - 2. * d, 25.8 - 2. * d, 3.25 - d, 4. + d)
}

/// **The drafted handle cuts its wheel** (#612): the arms narrow 8° through the 14.5 mm the wheel
/// spans, and the ring the hub sweeps narrows with them, so its inner edge crosses the sides of
/// the box the region is read within, 48 mm about the wheel's centre: two loops below, a hole in
/// the box above, and no change at all within the blank.  The slab's box is grown until
/// its stations are alike, and one slab is lofted.  The drafted star removes the least: its wheel
/// keeps more than the square arms' and than six bands stepped down it, each its widest section
/// over its band.
#[test]
fn the_drafted_handle_cuts_its_wheel() {
    let drafted = format!("{}\nstar := solid(plan.outline, from: -16mm, to: 0mm, draft: 8deg)", outline("plan", 0.));
    let src = handle(&drafted);
    let (e, w) = solved(&src);
    let admitted = admit_body(&e.sketch, w, &Options::default()).unwrap_or_else(|f| panic!("{f}"));
    let [sweep] = admitted.sweeps() else { panic!("one sweep") };
    let Class::Subtraction(found) = &sweep.class else { panic!("{:?}", sweep.class) };
    let [slab] = found.slabs.as_slice() else { panic!("{} slabs", found.slabs.len()) };
    let (_, base) = &slab.stations[0];
    for (z, r) in &slab.stations {
        assert!(r.matching(base).is_some() && r.loops.len() == 2, "at {z}: {} loops", r.loops.len());
    }
    let v = cut(&src, Some(Tolerance { millimetres: 0.01 }));
    let square = cut(&handle(&format!("{}\nstar := solid(plan.outline, from: -16mm, to: 0mm)", outline("plan", 0.))), None);
    // six bands through the 14.5 mm the star spans in the wheel, the last on to its foot
    let (band, tan) = (14.5 / 6., 8f64.to_radians().tan());
    let mut stepped = String::new();
    for k in 0..6 {
        let from = if k == 5 { -16. } else { -band * (k + 1) as f64 };
        stepped += &format!("{}\nband{k} := solid(plan{k}.outline, from: {from}mm, to: {}mm)\n", outline(&format!("plan{k}"), band * k as f64 * tan),
            -band * k as f64);
    }
    stepped += "star := solid(band0)\n";
    for k in 1..6 { stepped += &format!("band{k} union star\n") }
    let stepped = cut(&handle(&stepped), None);
    assert!(v > stepped && stepped > square, "drafted {v:.3} mm³, stepped {stepped:.3}, square {square:.3}");
}

/// **A drafted tool whose region changes within the blank is refused where it does** (#612): a
/// disc of 3 mm 25 mm off the stem, drafted 12°, traces a path with three loops in the wheel, and
/// the thinner the disc the more of each loop's island it leaves.  Where an island's edge changes
/// is the blank's own, so no box makes the stations alike; the refusal says the depth of the disc
/// to step it at, between the two stations that differ.
#[test]
fn a_drafted_tool_whose_islands_change_is_refused_at_their_depth() {
    let disc = "in std.top {\n  dc := point\n  dc coords(du: 25mm, dv: 1mm) std.top\n  disc := circle(center: dc) hint(r: 3)\n  radius(3mm) disc\n}\n\
        star := solid(face(disc), from: -10mm, to: 0mm, draft: 12deg)";
    let (e, w) = solved(&wheel(disc, &PERIOD.replace("540deg", "1080deg")));
    match admit_body(&e.sketch, w, &Options::default()) {
        Err(Error::Refused(r)) => {
            assert_eq!(r.condition, Condition::Slabs, "{r}");
            let m = r.to_string();
            let at = m.split("step the tool at about ").nth(1).and_then(|t| t.split(' ').next()).and_then(|t| t.parse::<f64>().ok());
            assert!(at.is_some_and(|z| (-8. ..-2.).contains(&z)) && m.contains("of `star`"), "{m}");
        }
        other => panic!("{other:?}"),
    }
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
use std.place (vertical)
use std.stations
use std.motion (turns)
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
pyramid := sweep(std.stations.Apex(base, tip).f over s)
turn := frame
turn turns(1) stem
space := sweep(pyramid over t in (0deg, 90deg), under: turn)
");
    let r = solve(&mut e.sketch, SolveOpts::default());
    assert!(r.success, "{}", r.message);
    let diags = program::solid_diagnostics(&e.sketch, &e.map, &e.program);
    let [d] = &diags[..] else { panic!("{diags:#?}") };
    assert_eq!((d.code, d.at(e.text()).0), (Code::E080, 25), "{d:?}");
    assert!(d.message.starts_with("`space` sweeps `pyramid`, whose material the field does not read"), "{d:?}");
}

/// #613's pinion: 12 teeth of module 0.8, its tooth spaces cut by one tooth of the basic rack (its
/// pitch line shifted +0.3 m) rolled across the blank on the pitch circle, from clear of the tip to
/// clear of it; `cuts` cuts `space` (or its copies about the spindle) from `gear`.
fn pinion(cuts: &str) -> String {
    rolled_pinion("0.8mm", "tooth over t in (-window, window)", cuts)
}

/// `pinion`, of module `m`, its `space` swept as
/// `sweep({tool}, under: rack, seen_from: blank_turn)`.
fn rolled_pinion(m: &str, tool: &str, cuts: &str) -> String {
    format!("\
unit mm
use std
use std.place (vertical, coords, on_v)
use std.motion (turns, slides)

m := {m}
N := 12
phi := 20deg
R := m * N / 2
Rp := R + 0.3 * m
Rt := Rp + m
hw := pi * m / 4
reach := sqrt(Rt ^ 2 - (Rp - 1.25 * m) ^ 2) + hw + 1.5 * m * tan(phi)
window := reach / R * 1rad

in std.front {{
  construction spindle := vertical line(std.origin, hint((0, 10)))
  length(10mm) spindle
}}
in std.top {{
  construction rack_line := line(hint((10, Rp)), hint((0, Rp)))
  rack_line.p1 coords(du: 10mm, dv: Rp) std.top
  rack_line.p2 on_v(d: Rp) std.top
  a := point hint((-hw - 1.5 * m * tan(phi), Rp + 1.5 * m))
  b := point hint((-hw + 1.25 * m * tan(phi), Rp - 1.25 * m))
  c := point hint((hw - 1.25 * m * tan(phi), Rp - 1.25 * m))
  d := point hint((hw + 1.5 * m * tan(phi), Rp + 1.5 * m))
  a coords(du: -hw - 1.5 * m * tan(phi), dv: Rp + 1.5 * m) std.top
  b coords(du: -hw + 1.25 * m * tan(phi), dv: Rp - 1.25 * m) std.top
  c coords(du: hw - 1.25 * m * tan(phi), dv: Rp - 1.25 * m) std.top
  d coords(du: hw + 1.5 * m * tan(phi), dv: Rp + 1.5 * m) std.top
  rack_tooth := face(a, b, c, d, -> close)
  tip := circle(center: std.top.origin) hint(r: Rt)
  radius(Rt) tip
  hole := circle(center: std.top.origin) hint(r: 1)
  radius(1mm) hole
}}
tooth := solid(rack_tooth, from: -1mm, to: 6mm)
blank_turn := frame
blank_turn turns(1) spindle
rack := frame
rack slides(2 * pi * R) rack_line
index := frame
index turns(1) spindle
space := sweep({tool}, under: rack, seen_from: blank_turn)
gear := solid(face(tip, holes: hole), from: 0mm, to: 5mm)
{cuts}
")
}

/// **A tooth space cut at a copy turned 30° is the space turned** (#613): the blank's box at a
/// placement no quarter turn reaches past the rack tooth's tip at each end pose, where the held top
/// edge, the corner's path and a hair of the flank the keeping test's tolerance keeps all meet.  The
/// boundary takes the corner's path (the rightmost turn) and the hair is dropped, so the copy alone
/// removes what the space removes, and a whole pinion — eleven copies — builds, agrees with the field
/// and removes twelve spaces.
#[test]
fn a_tooth_space_cut_at_a_turned_copy_is_the_space_turned() {
    // the tip circle's radius is 4.8 + 0.24 + 0.8 mm, the hole's 1 mm, the blank 5 mm thick
    let blank = PI * (5.84 * 5.84 - 1.) * 5.;
    let one = blank - cut_body(&pinion("space cut gear"), "gear", None);
    let turned = blank - cut_body(&pinion("turned := space at(under: index) 30deg\nturned cut gear"), "gear", None);
    assert!(one > 10. && (turned - one).abs() < 1e-3 * one, "the copy removes {turned:.5} mm³, the space {one:.5}");
    let copies: String = (1..12).map(|k| format!("t{k} := space at(under: index) {}deg\nt{k} cut gear\n", 30 * k)).collect();
    let whole = blank - cut_body(&pinion(&format!("space cut gear\n{copies}")), "gear", None);
    // the spaces are apart: twelve remove twelve times one
    assert!((whole - 12. * one).abs() < 1e-4 * whole, "twelve spaces remove {whole:.5} mm³, one {one:.5}");
}

/// **A roll that ends as the flank grazes its held edge closes** (the radiator bot's pinion): at
/// the end pose the flank's contact lies on the flank held there, tangent to it, so the keeping
/// test finds where the held flank starts to be kept only to `√(2 tol ρ)`, `ρ` the involute's
/// radius there.  At module 1.5 a roll of 56° (a copy turned 30°, its box reaching past the end
/// pose) left it 16 µm off the contact, past the joins' reach, and the boundary did not close.
/// The contact is the held flank's end however far the test finds it.  Over 60° the contact reaches
/// the tooth's back corner 0.06° before the roll ends, and the held flank is kept from that corner
/// a hair as long, 13.6 µm, joined to nothing: a spur, however far the test keeps it.  A cut over
/// 56°, over 60°, and over the roll found `through:` the blank remove what the generous stated
/// window does.
#[test]
fn a_roll_ending_where_the_flank_grazes_its_held_edge_closes() {
    let turned = "turned := space at(under: index) 30deg\nturned cut gear";
    // the tip circle's radius is 9 + 0.45 + 1.5 mm, the hole's 1 mm, the blank 5 mm thick
    let blank = PI * (10.95 * 10.95 - 1.) * 5.;
    let removed =
        |tool: &str| blank - cut_body(&rolled_pinion("1.5mm", tool, turned), "gear", None);
    let generous = removed("tooth over t in (-window, window)");
    assert!(generous > 10., "the space removes {generous:.5} mm³");
    for tool in ["tooth over t in (-56deg, 56deg)", "tooth over t in (-60deg, 60deg)",
                 "tooth, through: gear"] {
        let r = removed(tool);
        assert!((r - generous).abs() < 1e-4 * generous,
                "`{tool}` removes {r:.6} mm³, the stated window {generous:.6}");
    }
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

/// **A wheel placed is supplied as the wheel moved** (#619): asked for the placement alone, the
/// wheel is built as itself and its placement given that B-rep moved, never a build of its own —
/// a claim along a roll places the wheel at each pose, and the radiator bot's thirteen poses were
/// thirteen builds of it.
#[test]
fn a_placed_wheel_is_supplied_as_the_wheel_moved() {
    let (e, w) = solved(&stood(30.));
    let shown = e.map.ent_named("shown").unwrap().i();
    gcs_core::brep::export::supply_exact_where(&e.sketch, &|i| i == shown);
    let wheel = e.sketch.supplied_exact_solid(w).expect("the wheel, supplied as itself");
    let placed = e.sketch.supplied_exact_solid(shown).expect("its placement");
    assert_eq!(wheel.brep.vertices.len(), placed.brep.vertices.len());
    // the handle's 30° is the wheel's −20°, about the spindle through (−50, 0) along z
    let (s, c) = (-20f64.to_radians()).sin_cos();
    for (a, b) in wheel.brep.vertices.iter().zip(&placed.brep.vertices) {
        let (x, y) = (a.p[0] + 50., a.p[1]);
        let want = [c * x - s * y - 50., s * x + c * y, a.p[2]];
        assert!((0..3).all(|k| (b.p[k] - want[k]).abs() < 1e-9), "{:?} against {want:?}", b.p);
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

/// **A claim about a cut wheel is judged from its exact build** (#615): the radiator bot's star
/// handle swept through its wheel, a document where nothing runs, and a field the mesher cannot
/// protect the feature curves of.  The claim reads the B-rep the drawing reads, and the star at
/// the drawing's pose sits in the pocket it cut: no gap, so `clear(0.1mm)` is refuted, measured
/// within its uncertainty of nothing.
#[test]
fn a_claim_about_a_cut_wheel_reads_its_exact_build() {
    let (e, wheel) = solved(include_str!("fixtures/star_wheel.sv"));
    let [v] = &gcs_core::diagnose::judge_solids(&e.sketch)[..] else { panic!("one claim") };
    assert!(e.sketch.supplied_exact_solid(wheel).is_some(), "the wheel is read from its B-rep");
    assert_eq!(v.outcome(), gcs_core::diagnose::SolidOutcome::Refuted, "{v:?}");
    let i = v.measurement().and_then(|m| m.interval()).unwrap_or_else(|| panic!("{v:?}"));
    assert!(i.lower() <= 0.0 && 0.0 <= i.upper() && i.upper() < 0.1, "{i:?}");
    let text = gcs_core::report::solid_claim_text(v);
    assert!(text.starts_with("claim wheel clear(0.1mm) star — refuted, measured "), "{text}");
    assert!(!text.contains("poses") && !text.contains("unresolved"), "{text}");
}

/// #644's pinion, the radiator bot's `ShapedSpur` written out: 16 teeth of module 0.8, its rack
/// shifted +0.3 m and drawn 0.15 mm thick, the blank's edges at its faces chamfered 0.4 mm at 45°
/// by two cones about the spindle, and one tooth space cut.  The rack tooth's corner loops back
/// across its flank's cut there (the generating rows' fold), so the space is the planar subtraction
/// class's: a prism of the swept region, its flanks splines extruded along the spindle.
fn chamfered() -> String {
    "\
unit mm
use std
use std.place (vertical, coords, on_v, raised)
use std.sets
use std.motion (turns, slides)

m := 0.8mm
N := 16
phi := 20deg
R := m * N / 2
Rp := R + 0.3 * m
Rt := Rp + m
hw := pi * m / 4 + 0.075mm
reach := sqrt(Rt ^ 2 - (Rp - 1.25 * m) ^ 2) + hw + 1.5 * m * tan(phi)
window := reach / R * 1rad

in std.front {
  construction spindle := vertical line(std.origin, hint((0, 10)))
  length(10mm) spindle
}
in std.top {
  construction rack_line := line(hint((10, Rp)), hint((0, Rp)))
  rack_line.p1 coords(du: 10mm, dv: Rp) std.top
  rack_line.p2 on_v(d: Rp) std.top
  a := point hint((-hw - 1.5 * m * tan(phi), Rp + 1.5 * m))
  b := point hint((-hw + 1.25 * m * tan(phi), Rp - 1.25 * m))
  c := point hint((hw - 1.25 * m * tan(phi), Rp - 1.25 * m))
  d := point hint((hw + 1.5 * m * tan(phi), Rp + 1.5 * m))
  a coords(du: -hw - 1.5 * m * tan(phi), dv: Rp + 1.5 * m) std.top
  b coords(du: -hw + 1.25 * m * tan(phi), dv: Rp - 1.25 * m) std.top
  c coords(du: hw - 1.25 * m * tan(phi), dv: Rp - 1.25 * m) std.top
  d coords(du: hw + 1.5 * m * tan(phi), dv: Rp + 1.5 * m) std.top
  rack_tooth := face(a, b, c, d, -> close)
  tip := circle(center: std.top.origin) hint(r: Rt)
  radius(Rt) tip
  hole := circle(center: std.top.origin) hint(r: 4)
  radius(4.125mm) hole
  top_at := point hint((0, Rt - 0.4mm))
  top_at on_v(d: Rt - 0.4mm) std.top
  bottom_at := point hint((0, Rt + 5.35mm))
  bottom_at on_v(d: Rt + 5.35mm) std.top
}
tooth := solid(rack_tooth, from: -1mm, to: 6.75mm)
blank_turn := frame
blank_turn turns(1) spindle
rack := frame
rack slides(2 * pi * R) rack_line
gear := solid(face(tip, holes: hole), from: 0mm, to: 5.75mm)
top_q := point
top_q raised(d: 5.75mm, along: std.z) top_at
bottom_q := point
bottom_q raised(d: 5.75mm, along: std.z) bottom_at
top_cone := std.sets.ConeThrough(spindle, top_q, lean: -45deg)
bottom_cone := std.sets.ConeThrough(spindle, bottom_q, lean: 45deg)
bicone := { p | p inside top_cone; p inside bottom_cone }
chamfers := solid(bicone)
chamfers common gear
space := sweep(tooth over t in (-window, window), under: rack, seen_from: blank_turn)
space cut gear
".into()
}

/// **A flank's prism meets the tip and the chamfers in closed form** (#644): where the space's
/// extruded flanks meet the tip cylinder are lines along the spindle, and where they meet the
/// chamfers' cones each flank's spline carried onto the cone, a quintic of a few dozen poles.
/// Traced, each line took the flank's sharpest turn anywhere for its step — eight thousand steps,
/// fitted through a thousand — and the whole pinion's STEP grew sixfold.  The cut builds, checks,
/// and its mesh agrees with the field; no curve the file writes has more than 40 poles.
#[test]
fn a_flanks_prism_meets_the_tip_and_the_chamfers_in_closed_form() {
    use gcs_core::brep::geom::{Curve, Surface};
    use gcs_core::brep::topo::EdgeCurve;
    let (e, gear) = solved_body(&chamfered(), "gear");
    let admitted = admit_body(&e.sketch, gear, &Options::default()).unwrap_or_else(|f| panic!("{f}"));
    assert!(matches!(admitted.sweeps()[0].class, Class::Subtraction(_)), "{:?}", admitted.sweeps()[0].class);
    let exact = export::exact(&e.sketch, gear, None, None, &say())
        .unwrap_or_else(|r| panic!("refused at {:?}: {}", r.stage, r.message));
    let solid = &exact.solid;
    solid.check(1e-6).unwrap();
    // each edge's two faces' surfaces
    let faces = |ei: usize| -> Vec<&Surface> {
        solid.faces.iter().filter(|f| f.loops.iter().flatten().any(|c| c.edge as usize == ei)).map(|f| &f.surface).collect()
    };
    let (mut lines, mut lifted) = (0, 0);
    for (ei, edge) in solid.edges.iter().enumerate() {
        let EdgeCurve::Curve(curve) = &edge.curve else { continue };
        let [a, b] = faces(ei)[..] else { continue };
        let mut kinds = [a.kind(), b.kind()];
        kinds.sort();
        if kinds == ["cylinder", "extrusion"] {
            assert!(matches!(curve, Curve::Line { .. }), "a flank meets the tip in {curve:?}");
            lines += 1;
        }
        if kinds == ["cone", "extrusion"] {
            let Curve::BSpline(s) = curve else { panic!("a flank meets a chamfer in {curve:?}") };
            assert!(s.poles.len() <= 40, "{} poles", s.poles.len());
            for k in 0..=16 {
                let p = curve.point(edge.t[0] + (edge.t[1] - edge.t[0]) * k as f64 / 16.);
                assert!(a.implicit(p).abs() < 1e-7 && b.implicit(p).abs() < 1e-7, "{p:?}: {} {}", a.implicit(p), b.implicit(p));
            }
            lifted += 1;
        }
    }
    // a flank either side of the space, under the tip and under each chamfer
    assert_eq!((lines, lifted), (2, 4));
    let step = export::step(&exact, "gear", None, &say()).unwrap_or_else(|r| panic!("{}", r.message));
    let most = step.split("B_SPLINE_CURVE_WITH_KNOTS('',").skip(1)
        .map(|s| s.split_once(",(").unwrap().1.split_once(')').unwrap().0.split(',').count()).max().unwrap();
    assert!(most <= 40, "a curve of {most} poles");
    export::stl(&e.sketch, gear, &exact, None, &say()).unwrap_or_else(|r| panic!("the STL refused at {:?}: {}", r.stage, r.message));
}
