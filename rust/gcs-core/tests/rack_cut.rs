//! **A rack cuts a spur gear, natively** (issue #61): the generating-sweep class widened to a
//! prism tool under a translation seen from a rotation — the rack rolling on the blank's pitch
//! circle.  The fixture is one tooth space of a 20-tooth, module-2 gear at a 20° pressure angle:
//! no undercut, so the class's fold and crossing rows hold.  `gear` indexes it round a bored blank
//! into the whole gear, built as one sector patterned.
//!
//! Every number is in the page: the blank a disc extruded 6 mm along the page's normal (`depth:`
//! runs from −6 to 0), the rack tooth a trapezoid extruded past both of its faces, −8 to 2 (its
//! caps clear of the blank), the blank turning about the page's normal through `o`
//! (`motion(about: o)`).  What is removed is checked against arithmetic (`space_area`), not a
//! second kernel.

use gcs_core::solid::admission::{admit_body, Condition, Error, Options};

use crate::common::build;
use std::f64::consts::{PI, TAU};

/// Pitch radius 20, module 2: addendum circle 22, the rack's tip `m` below the pitch line (the
/// root circle 18), its tooth `π m / 2` thick at the pitch line, flanks 20° from the radius.
/// The tip stays inside the line of action, which ends `r sin² α` = 2.34 below the pitch line:
/// a flank reaching deeper generates the involute's folded branch below the base circle, which a
/// real cutter's tip trims away and the class's fold row (E3) refuses.  The rack rolls on the
/// pitch circle: `2π · 20` mm of slide a turn of the blank.
pub const SPACE: &str = "\
unit mm
o := point
fix(x == 0, y == 0) o
c := circle(center: o) hint(r: 22)
radius(22mm) c
disc := face(c)
blank := solid(disc, depth: 6mm)

s0 := point hint(x: 20, y: 0)
s1 := point hint(x: 20, y: 10)
fix(x == 20, y == 0) s0
fix(x == 20, y == 10) s1
slide := line(s0, s1)

t0 := point hint(x: 18, y: -0.84)
t1 := point hint(x: 18, y: 0.84)
t2 := point hint(x: 24, y: 3.03)
t3 := point hint(x: 24, y: -3.03)
fix(x == 18, y == -0.842856) t0
fix(x == 18, y == 0.842856) t1
fix(x == 24, y == 3.026666) t2
fix(x == 24, y == -3.026666) t3
tooth := face(t0, t1, t2, t3, -> close)
construction rack_tooth := solid(tooth, from: -8mm, to: 2mm)

turn := motion(about: o, ratio: 1)
rack := motion(along: slide, advance: 2 * pi * 20mm)
cutting := motion(rack, relative_to: turn)
construction space := solid(rack_tooth, under: cutting, from: -60deg, to: 60deg)
gear := solid(blank)
space cut gear
";

fn admit(src: &str) -> Result<gcs_core::solid::admission::Admission, Error> {
    let mut e = build(src);
    let r = gcs_core::solve::solve(&mut e.sketch, gcs_core::solve::SolveOpts::default());
    assert!(r.success, "{}", r.message);
    let gear = e.map.ent_named("gear").unwrap().i();
    admit_body(&e.sketch, gear, &Options::default())
}

/// **The rack's sweep is in the class.**  A prism tool (its sides extruded lines), a translation
/// seen from a rotation, clear of the blank at both ends of the roll, each tool point touching
/// once, no fold, no crossing: admitted, with contacts in the blank.
#[test]
fn a_racks_sweep_is_admitted() {
    let admission = match admit(SPACE) {
        Ok(a) => a,
        Err(e) => panic!("refused: {e}"),
    };
    let sweep = &admission.sweeps()[0];
    assert!(sweep.contacts > 100, "only {} contacts in the blank", sweep.contacts);
    assert!(sweep.least_area_factor > 1e-3);
}

/// A rack tooth no longer than the blank is thick has caps in it, and the class cuts with a
/// prism's sides only: refused at T1, naming the cap.
#[test]
fn a_rack_with_a_cap_in_the_blank_is_refused() {
    let src = SPACE.replace("solid(tooth, from: -8mm, to: 2mm)", "solid(tooth, from: -5mm, to: 2mm)");
    match admit(&src) {
        Err(Error::Refused(r)) => {
            assert_eq!(r.condition, Condition::Tool);
            assert!(r.message.contains("a cap of the prism"), "{}", r.message);
        }
        other => panic!("not refused at T1: {:?}", other.map(|_| ())),
    }
}

/// The tooth space's area by arithmetic: at each radius the circle meets the moved trapezoid in
/// one arc at each roll (each edge's half-plane holds an arc of the circle, in closed form, and
/// the four meet in one), and the space at that radius runs from the least start to the greatest
/// end over the roll.  Integrated over the radius by midpoints; the rolls' extremes are smooth,
/// so sampling them errs at second order.
fn space_area(radii: usize, rolls: usize) -> f64 {
    let tooth = [(18., -0.842856), (18., 0.842856), (24., 3.026666), (24., -3.026666)]; // clockwise
    let (gx, gy) = (21., 0.);
    let wrap = |a: f64| (a + PI).rem_euclid(TAU) - PI;
    // the arc of circle `r` about the blank's centre inside the tooth at roll `t`, in the blank's angles
    let arc = |r: f64, t: f64| -> Option<(f64, f64)> {
        let (cx, cy) = (0., -20. * t);
        let reference = (gy - cy).atan2(gx - cx);
        let (mut lo, mut hi) = (-PI, PI);
        for i in 0..4 {
            let ((ax, ay), (bx, by)) = (tooth[i], tooth[(i + 1) % 4]);
            let (nx, ny) = (-(by - ay), bx - ax);
            let k = (nx * (ax - cx) + ny * (ay - cy)) / (r * nx.hypot(ny));
            if k >= 1. { continue }
            if k <= -1. { return None }
            let (c, w) = (wrap(ny.atan2(nx) + PI - reference), PI - k.acos());
            (lo, hi) = (lo.max(c - w), hi.min(c + w));
            if lo >= hi { return None }
        }
        Some((reference + lo - t, reference + hi - t))
    };
    let (t0, t1) = (-PI / 3., PI / 3.);
    (0..radii).map(|i| {
        let r = 18. + 4. * (i as f64 + 0.5) / radii as f64;
        let (lo, hi) = (0..=rolls).filter_map(|j| arc(r, t0 + (t1 - t0) * j as f64 / rolls as f64))
            .fold((f64::INFINITY, f64::NEG_INFINITY), |(l, h), (a, b)| (l.min(a), h.max(b)));
        if hi > lo { (hi - lo) * r * 4. / radii as f64 } else { 0. }
    }).sum()
}

/// **The rack's tooth space is built exactly** by this kernel: the prism sectioned along its
/// extrusion (each station the profile itself), the sheet traced and fitted, the blank split by
/// it.  One sweep, so the body is built whole.  What it removes is the space's area by
/// arithmetic times the blank's thickness, within what the gross fit allows; and its STL is
/// written only where the material field agrees with it.
#[test]
fn a_racks_tooth_space_is_built() {
    use gcs_core::brep::{export, sweep::Say};
    let mut e = build(SPACE);
    let r = gcs_core::solve::solve(&mut e.sketch, gcs_core::solve::SolveOpts::default());
    assert!(r.success, "{}", r.message);
    let gear = e.map.ent_named("gear").unwrap().i();
    let say = Say { stage: &|l: &str| eprintln!("{l}"), mark: &|_| {} };
    let exact = match export::exact(&e.sketch, gear, None, None, &say) {
        Ok(x) => x,
        Err(r) => panic!("refused at {:?}: {}", r.stage, r.message),
    };
    assert!(exact.swept && exact.pattern.is_none());
    let removed = PI * 22. * 22. * 6. - gcs_core::brep::props::volume(&exact.solid);
    let want = 6. * space_area(400, 4000);
    assert!((removed - want).abs() < 0.03, "removed {removed:.5} mm³, the space being {want:.5}");
    if let Err(r) = export::stl(&e.sketch, gear, &exact, None, &say) {
        panic!("the STL refused at {:?}: {}", r.stage, r.message);
    }
}

/// The whole gear: a bore of radius 6 through the blank (a sector's sides meet on the axis, so a
/// blank reaching it is built whole), and the tooth space indexed round the axis `teeth` times.
pub fn gear(teeth: usize) -> String {
    SPACE.replace("space cut gear\n", &format!("\
bore_c := circle(center: o) hint(r: 6)
radius(6mm) bore_c
construction bore := solid(face(bore_c), from: -8mm, to: 2mm)
bore cut gear
repeat {teeth} as i {{
  construction indexed := solid(space, under: turn, at: i * 360deg / {teeth})
  indexed cut gear
}}
"))
}

/// **A rack cuts the whole gear**: the space indexed round the axis, built as one sector and
/// patterned.  Twenty spaces remove twenty times what one does, and the mesh agrees with the
/// field.
#[test]
fn a_rack_cuts_a_whole_gear() {
    use gcs_core::brep::{export, sweep::Say};
    let mut e = build(&gear(20));
    let r = gcs_core::solve::solve(&mut e.sketch, gcs_core::solve::SolveOpts::default());
    assert!(r.success, "{}", r.message);
    let gear = e.map.ent_named("gear").unwrap().i();
    let say = Say { stage: &|l: &str| eprintln!("{l}"), mark: &|_| {} };
    let exact = match export::exact(&e.sketch, gear, None, None, &say) {
        Ok(x) => x,
        Err(r) => panic!("refused at {:?}: {}", r.stage, r.message),
    };
    assert!(exact.pattern.is_some(), "built whole");
    let removed = PI * (22. * 22. - 6. * 6.) * 6. - gcs_core::brep::props::volume(&exact.solid);
    let want = 20. * 6. * space_area(400, 4000);
    eprintln!("removed {removed:.5} mm³, the spaces being {want:.5}");
    assert!((removed - want).abs() < 0.6, "removed {removed:.5} mm³, the spaces being {want:.5}");
    if let Err(r) = export::step(&exact, "gear", None, &say) {
        panic!("the STEP refused at {:?}: {}", r.stage, r.message);
    }
    if let Err(r) = export::stl(&e.sketch, gear, &exact, None, &say) {
        panic!("the STL refused at {:?}: {}", r.stage, r.message);
    }
}


/// Held to 10 µm the sheet is refined until its fit passes within half of it of every withheld
/// contact, and the gear is built and agrees with the field the same way.
#[test]
fn a_rack_cut_gear_is_held_to_a_tolerance() {
    use gcs_core::brep::{export, sweep::Say};
    use gcs_core::solid::export::Tolerance;
    let mut e = build(&gear(20));
    let r = gcs_core::solve::solve(&mut e.sketch, gcs_core::solve::SolveOpts::default());
    assert!(r.success, "{}", r.message);
    let gear = e.map.ent_named("gear").unwrap().i();
    let say = Say { stage: &|_: &str| {}, mark: &|_| {} };
    let held = Some(Tolerance { millimetres: 0.01 });
    let exact = export::exact(&e.sketch, gear, None, held, &say)
        .unwrap_or_else(|r| panic!("refused at {:?}: {}", r.stage, r.message));
    assert!(exact.pattern.is_some(), "built whole");
    let removed = PI * (22. * 22. - 6. * 6.) * 6. - gcs_core::brep::props::volume(&exact.solid);
    let want = 20. * 6. * space_area(400, 4000);
    assert!((removed - want).abs() < 0.6, "removed {removed:.5} mm³, the spaces being {want:.5}");
    export::stl(&e.sketch, gear, &exact, held, &say)
        .unwrap_or_else(|r| panic!("the STL refused at {:?}: {}", r.stage, r.message));
}

/// **`generation/rack_cut_gear.sv`**: the same gear written from its teeth, module and pressure angle, the
/// rack's tooth placed by dimensions from the blank's centre: it solves to the fixture's numbers
/// (written there to six places) and is admitted, one sweep placed twenty times; the involute it
/// draws in the plane from the same motion is the base circle's.
#[test]
fn the_rack_cut_gear_example_is_admitted() {
    let src = include_str!("../../examples/generation/rack_cut_gear.sv");
    let mut e = build(src);
    let r = gcs_core::solve::solve(&mut e.sketch, gcs_core::solve::SolveOpts::default());
    assert!(r.success, "{}", r.message);
    let at = |name: &str| {
        let (x, y) = e.sketch.point_xy(e.map.ent_named(name).unwrap().i());
        [x, y]
    };
    let tan = 20f64.to_radians().tan();
    for (name, want) in [("t0", [18., -(PI / 2. - 2. * tan)]), ("t2", [24., PI / 2. + 4. * tan])] {
        let got = at(name);
        assert!((got[0] - want[0]).abs() < 1e-6 && (got[1] - want[1]).abs() < 1e-6, "{name} at {got:?}");
    }
    let gear = e.map.ent_named("gear").unwrap().i();
    let admission = admit_body(&e.sketch, gear, &Options::default()).unwrap_or_else(|e| panic!("refused: {e}"));
    assert_eq!(admission.sweeps().len(), 1);
    // the planar envelope of the rack's flank, drawn beside the solid, is the base circle's
    // involute: its normal at every roll is tangent to that circle
    let (involute, rb) = (e.map.ent_named("involute").unwrap().i(), 20. * 20f64.to_radians().cos());
    for k in 0..=8 {
        let t = -20. + 5. * k as f64;
        let (p, (a, b)) = (e.sketch.curve_point(involute, t),
                           (e.sketch.curve_point(involute, t - 1e-4), e.sketch.curve_point(involute, t + 1e-4)));
        let (tx, ty) = (b.0 - a.0, b.1 - a.1);
        let off = (p.0 * tx + p.1 * ty).abs() / tx.hypot(ty);
        assert!((off - rb).abs() < 1e-6, "at roll {t} the involute's normal is {off} from the centre, not {rb}");
    }
}
