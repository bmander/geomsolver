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
const SHIFT: f64 = fixtures::gear::FOUND_MILL[0];
const CUTTER: f64 = fixtures::gear::FOUND_MILL[1];

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

/// **The pair scaled finds its face mill scaled** (#904 A3): with every length of the pair (the
/// module, the offset and the allowances) 0.625, 0.75 or 1.5 times as long, the balance finds the
/// same pressure shift and the cutter scaled with the pair — the root its seeds lead to there, the
/// cutter's seed (`hint(40 * mean_module)`) scaled too.
#[test]
fn the_face_mill_scaled_is_found_scaled() {
    for s in [0.625,0.75,1.5] {
        let e = fixtures::gear::read(&fixtures::gear::scaled(&fixtures::gear::source(),s));
        let (shift,cutter) = (unknown(&e,"pair.pressure_shift"),unknown(&e,"pair.cutter_radius"));
        assert!((shift-SHIFT).abs() < 1e-3,"at {s}: the pressure shift {shift}°");
        assert!((cutter-CUTTER*s).abs() < 1e-3*s,"at {s}: the cutter radius {cutter} mm against {} mm",CUTTER*s);
    }
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

/// **At a ten-tooth pinion no face mill balances the undercut** (#283): the balance has no root,
/// so the pair configured there must state its mill, as the suites cut it (`recorded_machine`).
/// With the mill stated and the margins read, not held equal, the pinion's margin stays below the
/// gear's at every pressure shift up to the outer blade's limit — the shift is short of the 20°
/// pressure angle, where the outer blade, `pressure − shift`, stands upright — at the mean point
/// and at the toe alike. The pinion's grows with the shift and the gear's, after a least near 15°,
/// grows faster as its blade stands up: they never meet. (Left to the solve, the balance runs to its
/// iteration limit there, whatever its seeds.)
#[test]
fn at_a_ten_tooth_pinion_no_face_mill_balances_the_undercut() {
    let ten = fixtures::gear::configure(&fixtures::gear::source(),&[("pinion_teeth",10.)]);
    let seeds = "pressure_shift: hint(10deg), cutter_radius: hint(40 * mean_module))";
    assert!(ten.contains(seeds),"the pair leaves its mill to the balance");
    // the mean cone distance at 10:48, 2 mm module: the cutter is no smaller
    let r = 2.*(10f64*10.+48.*48.).sqrt()/2.;
    for shift in [8.,14.,19.] {
        let text = ten.replacen(seeds,&format!("pressure_shift: {shift}deg, cutter_radius: {}mm)",1.5*r),1)
            .replace("pair.margins.pinion_mean equal pair.margins.gear_mean\n","")
            .replace("pair.margins.pinion_toe equal pair.margins.gear_toe\n","");
        let e = fixtures::gear::read(&text);
        for station in ["mean","at_toe"] {
            let (pinion,gear) = (radius(&e,&format!("pair.balance.{station}.pinion_profile")),
                radius(&e,&format!("pair.balance.{station}.gear_profile")));
            assert!(pinion < 0.5*gear,"at {shift}°, {station}: the pinion's margin {pinion} mm against the gear's {gear} mm");
        }
    }
}

/// **At no offset the balance finds a mill that undercuts the pinion** (#904 A2). The shift trades
/// the flanks the inner blade cuts against those the outer blade cuts, and `Balanced` weighs the
/// pinion's inner-cut flank against the gear's outer-cut one: the pair that binds once the
/// offset has steepened the pinion's outer-cut flank. With no offset nothing has, and the shift it
/// finds, 11.68°, stands the outer blade at 8.3°, which undercuts the pinion's flank it cuts — so
/// the sweep's admission refuses the pinion's cut for a fold on the crown's `outer` face, and
/// says which face. The same pair cut with no shift, the cutter as found, folds nowhere.
#[test]
fn at_no_offset_the_balanced_mill_undercuts_the_pinion_by_its_outer_blade() {
    use gcs_core::solid::admission::{self,Condition,Error,Options};
    let text = fixtures::gear::configure(&fixtures::gear::source(),&[("offset",0.)]);
    let e = fixtures::gear::read(&text);
    let (shift,cutter) = (unknown(&e,"pair.pressure_shift"),unknown(&e,"pair.cutter_radius"));
    assert!((shift-11.6757).abs() < 1e-3,"the pressure shift: {shift}°");
    let admit = |e: &Elaborated| {
        admission::admit_body(&e.sketch,fixtures::solid(e,"pair.pinion.body"),&Options::default())
    };
    match admit(&e) {
        Err(Error::Refused(r)) => {
            eprintln!("{r}");
            assert_eq!(r.condition,Condition::Fold,"{r}");
            assert!(r.message.contains("`pair.tooth.crown.outer`"),
                "the fold is said on the outer blade's face: {r}");
        }
        Ok(_) => panic!("the pinion's cut was admitted at a {shift}° shift"),
        Err(e) => panic!("{e}"),
    }
    let seeds = "pressure_shift: hint(10deg), cutter_radius: hint(40 * mean_module))";
    let symmetric = fixtures::gear::recorded_machine(&text,0.).replacen(
        "cutter_radius: 0.8 * mean_module * sqrt(pinion_teeth^2 + gear_teeth^2) / 2)",
        &format!("cutter_radius: {cutter}mm)"),1);
    assert!(!symmetric.contains(seeds) && symmetric.contains(&format!("{cutter}mm")));
    let least = admit(&fixtures::gear::read(&symmetric))
        .unwrap_or_else(|e| panic!("with no shift: {e}")).sweeps()[0].least_area_factor;
    assert!(least > 0.3,"with no shift the pinion's least area factor is {least}");
}

/// **A pair no face mill balances is re-targeted by stating its mill** (#904, A1). At 12/24 (a
/// 12.5 mm offset), 13/27 (12 mm) and 16/32 at a 1.5 mm module (12 mm) the balance has no root,
/// as at a ten-tooth pinion: over pressure shifts from 8° to 18° and cutters from R to 4 R the
/// pinion's margin stays below the gear's at the toe, and at M too but for the largest cutters
/// at the gear's least margin, near 12°. The layout itself solves there from the pair's own start.
/// So the balance is the caller's to state over the margins the pair hands it (`pair.margins`,
/// as `hypoid.sv` does), and a drawing standing on the pair at such counts states its mill
/// instead — which, while the balance stood inside `HypoidPair`, was two rows past what its
/// unknowns take (E109).
#[test]
fn a_pair_no_mill_balances_is_laid_out_with_its_mill_stated() {
    let examples = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../examples");
    for (teeth,module,offset) in [([12,24],2.,12.5),([13,27],2.,12.),([16,32],1.5,12.)] {
        let r = module*f64::from(teeth[0]*teeth[0]+teeth[1]*teeth[1]).sqrt()/2.;
        let text = format!("unit mm\nuse std\nuse spiral_bevel.hypoid (HypoidPair)\n\
            pair := HypoidPair(pinion_teeth: {}, gear_teeth: {}, mean_module: {module}mm,\n  \
            shaft_angle: 90deg, offset: {offset}mm, spiral_angle: 25deg, backlash: 0mm,\n  \
            tip_relief: 0mm, end_relief: 0mm, pressure_shift: 12deg, cutter_radius: {}mm)\n",
            teeth[0],teeth[1],1.5*r);
        let e = fixtures::read_beside(&text,&examples,&mut |_,t| t);
        let label = format!("{}/{} m{module} E{offset}",teeth[0],teeth[1]);
        // the pair as asked: its mean cone distance, and the shafts the offset apart
        let (o,m) = (point(&e,"pair.gear_cone.O"),point(&e,"pair.gear_cone.M"));
        assert!((dist(o,m)-r).abs() < 1e-9*r,"{label}: R {}",dist(o,m));
        let (g0,g1) = (o,point(&e,"pair.gear_cone.foot"));
        let (p0,p1) = (point(&e,"pair.pinion_cone.A"),point(&e,"pair.pinion_cone.foot"));
        let (g,p) = (sub(g1,g0),sub(p1,p0));
        let n = [g[1]*p[2]-g[2]*p[1],g[2]*p[0]-g[0]*p[2],g[0]*p[1]-g[1]*p[0]];
        let skew = sub(p0,g0).iter().zip(n).map(|(a,b)| a*b).sum::<f64>().abs()/dist(n,[0.;3]);
        assert!((skew-offset).abs() < 1e-9*r,"{label}: offset {skew}");
        // and its margins read, the pinion's the nearer undercut
        for station in ["mean","at_toe"] {
            let (pinion,gear) = (radius(&e,&format!("pair.balance.{station}.pinion_profile")),
                radius(&e,&format!("pair.balance.{station}.gear_profile")));
            assert!(pinion < gear,"{label}, {station}: the pinion's margin {pinion} mm against the gear's {gear} mm");
        }
    }
}
