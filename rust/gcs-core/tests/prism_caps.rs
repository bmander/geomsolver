//! **A prism's cap in the blank** (#684 F): the generating class sections a prism standing alone
//! across its profile where its cap reaches the blank (`cutter::Cutter::across`, row T3), so its
//! walk runs from a side over the cap's rim and across the cap, and the cap and its rims' fans are
//! built with its sides. The bar of `rust/examples/solid_sweeps/bar_end.sv` is the case: its flat
//! end cuts the floor of a tooth space as a rack's tip does, each height of the disc losing the
//! space its own slice of the bar sweeps; its volume is read slice by slice by `rack_cut`'s
//! arithmetic (`rack_space`), apart from the code under test. And what the class still refuses: a cap whose
//! corners dig past it (E3, at its rim), a band of sections meeting a corner of the profile (T3),
//! and a cap of a tool that is no prism standing alone (T1); a
//! prism a Boolean of the tool leaves whole is one standing alone, read from its B-rep (#721 F1b).
use gcs_core::brep::{export,props,sweep::Say};
use gcs_core::solid::admission::{self,Class,Condition,Error,Options};
use gcs_core::solid::export::Tolerance;
use std::f64::consts::PI;

/// The example's text.
fn bar_end() -> String { crate::solid_sweeps_examples::text("bar_end.sv") }

/// What `bar_end.sv`'s bar removes from the disc: at each height `z` of its 6 mm the bar's slice is
/// the rectangle 18 to 30 mm from the centre and `w(z)` either side of its axis (its profile's sides
/// run from 1 mm at z = -9 to 2 at z = 3), rolled over ±60° on the circle of 16; Simpson's rule
/// over 12 slices of the height. Each area's midpoints in the radius err at first order (the
/// floor's edge, where the space's arc at a radius stops short, falls between them), so 400 and
/// 800 of them are extrapolated (Richardson).
pub(crate) fn bar_end_space() -> f64 {
    // read once for every test that holds the bar to it
    static SPACE: std::sync::OnceLock<f64> = std::sync::OnceLock::new();
    *SPACE.get_or_init(|| {
        let w = |z: f64| 1.+(z+9.)/12.;
        let slices = 12;
        let h = 6./slices as f64;
        let volume = |radii: usize| (0..=slices).map(|k| {
            let z = -6.+h*k as f64;
            let weight = if k == 0 || k == slices { 1. } else if k%2 == 1 { 4. } else { 2. };
            weight*crate::rack_cut::rack_space(&[(18.,-w(z)),(18.,w(z)),(30.,w(z)),(30.,-w(z))],16.,[18.,22.],[-PI/3.,PI/3.],radii,1000)
        }).sum::<f64>()*h/3.;
        2.*volume(800)-volume(400)
    })
}

fn refused(source: &str,body: &str) -> admission::Refusal {
    let e = fixtures::read(source);
    match admission::admit_body(&e.sketch,fixtures::solid(&e,body),&Options::default()) {
        Err(Error::Refused(r)) => { eprintln!("{r}"); r }
        Err(Error::Unreadable(m)) => panic!("unreadable: {m}"),
        Ok(a) => panic!("admitted: {:?}",a.sweeps().iter().map(|s| &s.name).collect::<Vec<_>>()),
    }
}

/// The bar's sweep is admitted with its cap in the disc, sectioned across its height (the disc's
/// axis, the one direction whose sections keep clear of the profile's corners, all four of them
/// above or below the disc), and held to a tighter tolerance than the series' it is built within
/// 2e-6 of the arithmetic's figure (its 0.01 mm build is held in `solid_sweeps_examples`).
#[test]
fn the_bars_end_cuts_the_floor_of_its_space() {
    let e = fixtures::read(&bar_end());
    let part = fixtures::solid(&e,"part");
    let a = admission::admit_body(&e.sketch,part,&Options::default()).unwrap_or_else(|r| panic!("{r}"));
    let [s] = a.sweeps() else { panic!("one sweep") };
    let Class::Capped(capped) = &s.class else { panic!("{:?}",s.class) };
    eprintln!("across {:?}, band {:?}, clearance {}",capped.across,capped.band,capped.clearance);
    assert!(capped.across[2].abs() > 0.99,"sectioned across the disc's height: {:?}",capped.across);
    assert!(capped.clearance >= 2.,"three millimetres of the profile above and below the disc, less the band's spread");
    let say = Say {stage:&|_: &str| {},mark:&|_| {}};
    let exact = export::exact(&e.sketch,part,None,Some(Tolerance::new(0.002).unwrap()),&say).unwrap_or_else(|r| panic!("{}",r.message));
    exact.solid.check(1e-6).unwrap();
    let removed = PI*22.*22.*6.-props::volume(&exact.solid);
    let want = bar_end_space();
    eprintln!("removed {removed:.6} mm³, the arithmetic {want:.6}");
    assert!((removed-want).abs() <= 2e-6*want,"removed {removed} against {want}");
}

/// The same bar swung on an arm about an axis beside the disc's, the disc turning too: its end's
/// corners swing farther from the arm's axis than the end does, so they dig past the end's own
/// envelope and their paths cross under it. Where the end's contact reaches its rim, the rim's fan
/// sweeps back over the end's envelope, and E3 refuses that fold before any sheet is fitted (the
/// fit found it otherwise, folding).
#[test]
fn a_flat_end_its_corners_dig_past_is_refused() {
    let source = bar_end().replace("  s0 := point hint((16, 0))\n  s1 := point hint((16, 10))\n  fix((16, 0)) s0\n  fix((16, 10)) s1\n  pitch_line := line(s0, s1)\n",
        "  c := point hint((40, 0))\n  fix((40, 0)) c\n")
        .replace("rack := frame\nrack slides(2 * pi * 16mm) pitch_line\n","arm := frame\narm spins(2) c\n")
        .replace("solid({ p | some t in (-60deg, 60deg): p in bar at(under: rack, seen_from: turn) t })","solid({ p | some t in (-25deg, 25deg): p in bar at(under: arm, seen_from: turn) t })");
    assert!(source.contains("under: arm") && source.contains("arm spins"),"the arm is written in");
    let r = refused(&source,"part");
    assert_eq!((r.condition,r.class()),(Condition::Fold,"generating-sweep class"));
    assert!(r.message.contains("the cap's envelope"),"{}",r.message);
}

/// A profile with a corner on its side within the disc's height: every section through it meets
/// the corner, where its side leaps from one face's normal to the next's, and every direction across
/// the profile puts the corner inside the band the cap's contacts reach. Refused at T3.
#[test]
fn a_band_of_sections_meeting_a_corner_is_refused() {
    let source = bar_end().replace("  fix((2, 3)) b2\n","  fix((2, 3)) b2\n  bm := point hint((1.9, -3))\n  fix((1.9, -3)) bm\n")
        .replace("face(b0, b1, b2, b3, -> close)","face(b0, b1, bm, b2, b3, -> close)");
    assert!(source.contains("bm, b2"));
    let r = refused(&source,"part");
    assert_eq!(r.condition,Condition::Sections);
    assert!(r.message.contains("a corner of the profile"),"{}",r.message);
}

/// The same block cutting the bar's end on a slant, about 20 mm from the disc's centre, inside its
/// rim: the tool is no prism standing alone, its slanted end no cap square to its sides, and that
/// end passing through the disc is refused at T1, as every cap was before (#684 F).
#[test]
fn a_cap_of_a_tool_no_lone_prism_is_refused() {
    let source = bar_end().replace("construction space := solid({ p | some t in (-60deg, 60deg): p in bar at(under: rack, seen_from: turn) t })","in std.top {\n  k0 := point hint((10, -10))\n  k1 := point hint((21, -10))\n  \
        k2 := point hint((19, 10))\n  k3 := point hint((10, 10))\n  fix((10, -10)) k0\n  fix((21, -10)) k1\n  fix((19, 10)) k2\n  fix((10, 10)) k3\n}\n\
        construction keep := solid(std.solids.Prism(face(k0, k1, k2, k3, -> close), from: -20mm, to: 20mm))\nconstruction tool := solid(bar)\nkeep common tool\n\
        construction space := solid({ p | some t in (-60deg, 60deg): p in tool at(under: rack, seen_from: turn) t })");
    assert!(source.contains("keep common tool"));
    assert!(source.contains("keep common tool"));
    let r = refused(&source,"part");
    assert_eq!(r.condition,Condition::Tool);
    assert!(r.message.contains("standing alone"),"{}",r.message);
}

/// The bar shared with a block (a `common`) that holds it whole is the bar: read from its B-rep
/// (#721 F1b) it is one prism standing alone, however it was drawn, and its cap in the disc is
/// sectioned across and cut as the bar's own is, to the same arithmetic. (Before the tool was read
/// by its B-rep, a cap of an intersection was refused at T1: #721 G9's negative control.)
#[test]
fn a_cap_of_a_prism_a_common_leaves_whole_cuts_as_the_bars() {
    let source = bar_end().replace("construction space := solid({ p | some t in (-60deg, 60deg): p in bar at(under: rack, seen_from: turn) t })","in std.top {\n  k0 := point hint((10, -10))\n  k1 := point hint((40, -10))\n  \
        k2 := point hint((40, 10))\n  k3 := point hint((10, 10))\n  fix((10, -10)) k0\n  fix((40, -10)) k1\n  fix((40, 10)) k2\n  fix((10, 10)) k3\n}\n\
        construction keep := solid(std.solids.Prism(face(k0, k1, k2, k3, -> close), from: -20mm, to: 20mm))\nconstruction tool := solid(bar)\nkeep common tool\n\
        construction space := solid({ p | some t in (-60deg, 60deg): p in tool at(under: rack, seen_from: turn) t })");
    assert!(source.contains("keep common tool"));
    let e = fixtures::read(&source);
    let part = fixtures::solid(&e,"part");
    let a = admission::admit_body(&e.sketch,part,&Options::default()).unwrap_or_else(|r| panic!("{r}"));
    let [s] = a.sweeps() else { panic!("one sweep") };
    assert!(matches!(s.class,Class::Capped(_)),"sectioned across its cap: {:?}",s.class);
    let say = Say {stage:&|_: &str| {},mark:&|_| {}};
    let exact = export::exact(&e.sketch,part,None,Some(Tolerance::new(0.002).unwrap()),&say).unwrap_or_else(|r| panic!("{}",r.message));
    exact.solid.check(1e-6).unwrap();
    let removed = PI*22.*22.*6.-props::volume(&exact.solid);
    let want = bar_end_space();
    eprintln!("removed {removed:.6} mm³, the arithmetic {want:.6}");
    assert!((removed-want).abs() <= 2e-6*want,"removed {removed} against {want}");
}

/// **A bar with a hole cuts as the bar** (#721 F1d, G9): `bar_end.sv`'s bar with a round hole 0.3 in
/// radius through it at 2.4 above the disc's top, read from its B-rep as one prism standing alone,
/// its profile two loops (`Lone::read`), so its cap in the disc is sectioned across the disc's
/// height as the bar's is, each section's chords parted by the hole only above the band the sheet
/// spans (`Cutter::across`, its hole's ends kept clear as a corner is). The hole stands clear of the
/// disc, so the disc loses `bar_end_space`.
#[test]
fn a_bar_with_a_hole_cuts_the_floor_of_its_space() {
    let text = bar_end().replace("construction bar := solid(std.solids.Prism(face(b0, b1, b2, b3, -> close)",
        "  hc := point hint((0, 2.4))\n  fix((0, 2.4)) hc\n  hole := radius(0.3mm) circle(center: hc) hint(r: 0.3)\n  \
        l0 := line(b0, b1)\n  l1 := line(b1, b2)\n  l2 := line(b2, b3)\n  l3 := line(b3, b0)\n}\n\
        construction bar := solid(std.solids.Prism(face(l0, l1, l2, l3, holes: hole)")
        .replace("  fix((-2, 3)) b3\n}\n","  fix((-2, 3)) b3\n");
    assert!(text.contains("holes: hole"));
    let e = fixtures::read(&text);
    let part = fixtures::solid(&e,"part");
    let a = admission::admit_body(&e.sketch,part,&Options::default()).unwrap_or_else(|r| panic!("{r}"));
    let [s] = a.sweeps() else { panic!("one sweep") };
    let Class::Capped(capped) = &s.class else { panic!("{:?}",s.class) };
    assert!(capped.across[2].abs() > 0.99,"sectioned across the disc's height: {:?}",capped.across);
    let say = Say {stage:&|_: &str| {},mark:&|_| {}};
    let exact = export::exact(&e.sketch,part,None,Some(Tolerance::new(0.002).unwrap()),&say).unwrap_or_else(|r| panic!("{}",r.message));
    exact.solid.check(1e-6).unwrap();
    let removed = PI*22.*22.*6.-props::volume(&exact.solid);
    let want = bar_end_space();
    eprintln!("removed {removed:.6} mm³, the arithmetic {want:.6}");
    assert!((removed-want).abs() <= 2e-6*want,"removed {removed} against {want}");
}

/// **A drafted bar's end cuts the floor of its space** (#721 F1d, G9): `bar_end.sv`'s bar drafted
/// half a degree, its sides narrowing from its profile's plane outward, read from its B-rep as one
/// prism drafted (`Lone::read`), its cap in the disc sectioned across the disc's height
/// (`Cutter::across` over its one half: each section's sides the drafted planes' rulings). The
/// disc loses the arithmetic's figure: at each height the bar's slice is the trapezoid whose
/// half-width at radius `x` is `w(z) − x tan δ √145 / 12` (a side's run of 12 over its rise of 1
/// moved `x tan δ` into the bar), rolled as `bar_end_space`'s rectangle is: within 2e-5 at 0.002 mm
/// (8e-6; 4e-6 either side of it at 0.0005 and 0.0002 mm, the fit's own scatter).
#[test]
fn a_drafted_bars_end_cuts_the_floor_of_its_space() {
    let draft: f64 = 0.5;
    let text = bar_end().replace("construction bar := solid(std.solids.Prism(face(b0, b1, b2, b3, -> close), from: 18mm, to: 30mm))",
        &format!("construction bar := solid(std.solids.Drafted(face(b0, b1, b2, b3, -> close), from: 18mm, to: 30mm, draft: {draft}deg))"));
    assert!(text.contains("Drafted"));
    let e = fixtures::read(&text);
    let part = fixtures::solid(&e,"part");
    let a = admission::admit_body(&e.sketch,part,&Options::default()).unwrap_or_else(|r| panic!("{r}"));
    let [s] = a.sweeps() else { panic!("one sweep") };
    let Class::Capped(capped) = &s.class else { panic!("{:?}",s.class) };
    assert!(capped.across[2].abs() > 0.99,"sectioned across the disc's height: {:?}",capped.across);
    let say = Say {stage:&|_: &str| {},mark:&|_| {}};
    let exact = export::exact(&e.sketch,part,None,Some(Tolerance::new(0.002).unwrap()),&say).unwrap_or_else(|r| panic!("{}",r.message));
    exact.solid.check(1e-6).unwrap();
    let removed = PI*22.*22.*6.-props::volume(&exact.solid);
    let lean = draft.to_radians().tan()*145f64.sqrt()/12.;
    let w = |z: f64,x: f64| 1.+(z+9.)/12.-x*lean;
    let (slices,h) = (12,0.5);
    let volume = |radii: usize| (0..=slices).map(|k| {
        let z = -6.+h*k as f64;
        let weight = if k == 0 || k == slices { 1. } else if k%2 == 1 { 4. } else { 2. };
        weight*crate::rack_cut::rack_space(&[(18.,-w(z,18.)),(18.,w(z,18.)),(30.,w(z,30.)),(30.,-w(z,30.))],16.,[18.,22.],[-PI/3.,PI/3.],radii,1000)
    }).sum::<f64>()*h/3.;
    let want = 2.*volume(800)-volume(400);
    eprintln!("removed {removed:.6} mm³, the arithmetic {want:.6}, undrafted {:.6}",bar_end_space());
    assert!(want < bar_end_space()-0.5,"the draft narrows the bar");
    assert!((removed-want).abs() <= 2e-5*want,"removed {removed} against {want}");
}
