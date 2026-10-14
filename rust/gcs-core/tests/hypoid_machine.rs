//! The spiral-bevel pair's face mill found rather than stated (`spiral_bevel/hypoid.sv`, #283):
//! its pressure shift and cutter radius are formals of `HypoidPair` the call leaves to the solve,
//! and `Balanced` finds them as a designer would — the generated flanks' undercut margins (the
//! radius of each flank's profile circle a module inside and outside the pitch, in a section
//! square to the trace) equal at the mean point, which sets the shift, and at the toe, which sets
//! the cutter.  The rolls are the crown's tooth count over each member's, written as counts.
//!
//! The two numbers recorded here were taken of the document as it stands; the rest is the
//! balance's own words, read back off the solve.
use gcs_core::program::Elaborated;

type V = [f64;3];

fn sub(a: V,b: V) -> V { [a[0]-b[0],a[1]-b[1],a[2]-b[2]] }
fn dist(a: V,b: V) -> f64 { let d = sub(a,b); (d[0]*d[0]+d[1]*d[1]+d[2]*d[2]).sqrt() }

fn index(e: &Elaborated,name: &str) -> usize {
    e.map.ent_named(name).unwrap_or_else(|| panic!("no `{name}`")).i()
}
fn point(e: &Elaborated,name: &str) -> V { e.sketch.world_point(index(e,name)) }
fn radius(e: &Elaborated,name: &str) -> f64 {
    e.sketch.params[e.sketch.circles[index(e,name)].radius as usize].value
}
/// A number the document leaves unknown, where the solve put it, in the unit the document
/// writes it in (an angle in degrees).
fn unknown(e: &Elaborated,name: &str) -> f64 {
    let q = *e.sketch.free_vars.get(name)
        .unwrap_or_else(|| panic!("no unknown `{name}` in {:?}",e.sketch.free_vars.keys()));
    e.sketch.params[q as usize].value
}

/// The face mill as found at the configured hypoid: its pressure shift in degrees and its
/// cutter's radius in millimetres.
const SHIFT: f64 = 11.2527;
const CUTTER: f64 = 75.6122;

#[test]
fn the_face_mill_is_found_by_balancing_the_undercut_margins() {
    let e = fixtures::gear::read_as_configured();
    let (shift,cutter) = (unknown(&e,"pair.pressure_shift"),unknown(&e,"pair.cutter_radius"));
    assert!((shift-SHIFT).abs() < 1e-3,"the pressure shift: {shift}°");
    assert!((cutter-CUTTER).abs() < 1e-3,"the cutter radius: {cutter} mm");
    // the balance: each member's margin the other's, at the mean point and at the toe, and each
    // a margin (the profile no nearer its cusp than a few modules)
    for station in ["mean","at_toe"] {
        let (pinion,gear) = (radius(&e,&format!("pair.balance.{station}.pinion_profile")),
            radius(&e,&format!("pair.balance.{station}.gear_profile")));
        assert!((pinion-gear).abs() < 1e-9*pinion,"{station}: {pinion} against {gear}");
        assert!(pinion > 5.,"{station}: a margin of {pinion} mm");
    }
    let margin = |station: &str| radius(&e,&format!("pair.balance.{station}.pinion_profile"));
    assert!(margin("mean") > margin("at_toe"),"the margin shrinks toward the toe");
    // the trace is the cutter's circle, through M and the toe, no smaller than the cone distance
    let (o,m) = (point(&e,"pair.gear_cone.O"),point(&e,"pair.gear_cone.M"));
    let (c,toe) = (point(&e,"pair.trace.C"),point(&e,"pair.balance.toe"));
    assert!((dist(c,m)-cutter).abs() < 1e-9*cutter && (dist(c,toe)-cutter).abs() < 1e-9*cutter);
    assert!(dist(c,m) >= dist(o,m),"the cutter no smaller than the mean cone distance");
    assert!(dist(o,toe) < dist(o,m),"the toe nearer the apex than M");
}

#[test]
fn the_rolls_are_the_crowns_tooth_count_over_each_members() {
    // the bevel pair, its mill stated: the rolls read nothing the balance finds
    let e = fixtures::gear::read(&fixtures::gear::bevel(&fixtures::gear::source()));
    let crown = (24f64*24.+48.*48.).sqrt();
    for (member,ratio) in [("pinion_roll",crown/24.),("gear_roll",-crown/48.)] {
        let at = e.sketch.motions.iter().position(|m| m.name == format!("pair.{member}"))
            .unwrap_or_else(|| panic!("no motion `{member}`"));
        let read = e.sketch.motions[at].rotation(&e.sketch).unwrap().0;
        assert!((read-ratio).abs() < 1e-12*ratio.abs(),"{member}: {read} against {ratio}");
    }
}

#[test]
fn the_recorded_machine_states_the_mill_the_suites_were_cut_by() {
    // as configured, the pair leaves both settings to the solve; a design the suites recorded
    // states them (the shift it was recorded at, the cutter 0.8 R) and drops the balance
    let text = fixtures::gear::source();
    let found = fixtures::gear::unsolved(&text);
    for name in ["pair.pressure_shift","pair.cutter_radius"] {
        assert!(found.sketch.free_vars.contains_key(name),"`{name}` is unknown as configured");
    }
    let stated = fixtures::gear::bevel(&text);
    assert!(!stated.contains("balance := Balanced("));
    let e = fixtures::gear::unsolved(&stated);
    for name in ["pair.pressure_shift","pair.cutter_radius"] {
        assert!(!e.sketch.free_vars.contains_key(name),"`{name}` is stated at a recorded design");
    }
    assert!(e.map.ent_named("pair.balance.toe").is_none());
}
