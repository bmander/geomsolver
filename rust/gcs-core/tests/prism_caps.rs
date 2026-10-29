//! **A prism's cap in the blank** (#684 F): the generating class sections a prism standing alone
//! across its profile where its cap reaches the blank (`cutter::Cutter::across`, row T3), so its
//! walk runs from a side over the cap's rim and across the cap, and the cap and its rims' fans are
//! built with its sides. The bar of `rust/examples/solid_sweeps/bar_end.sv` is the case: its flat
//! end cuts the floor of a tooth space as a rack's tip does, each height of the disc losing the
//! space its own slice of the bar sweeps; its volume is read slice by slice by `rack_cut`'s
//! arithmetic, apart from the code under test. And what the class still refuses: a cap whose
//! corners dig past it (E3, at its rim), a band of sections meeting a corner of the profile (T3),
//! and a cap of a tool that is no prism standing alone (T1).
use gcs_core::brep::{export,props,sweep::Say};
use gcs_core::solid::admission::{self,Class,Condition,Error,Options};
use gcs_core::solid::export::Tolerance;
use std::f64::consts::{PI,TAU};

/// The example's text.
fn bar_end() -> String { crate::solid_sweeps_examples::text("bar_end.sv") }

/// The area a rack tooth (its corners clockwise in the rack's plane, in millimetres, the blank's
/// centre on the plane's `x` axis) removes from the annulus `[inner, outer]` about the blank's
/// centre as it rolls over `roll` on a pitch circle of `pitch`: `rack_cut::space_area`'s
/// arithmetic — at each radius the circle meets the moved tooth in one arc at each roll, and the
/// space at that radius runs from the least start to the greatest end — given its tooth and its
/// numbers, and each extreme read again finely about the roll that gave it, so the roll's
/// sampling errs at second order.
pub(crate) fn rack_space(tooth: &[(f64,f64)],pitch: f64,[inner,outer]: [f64;2],[t0,t1]: [f64;2],radii: usize,rolls: usize) -> f64 {
    let n = tooth.len();
    let (gx,gy) = (tooth.iter().map(|p| p.0).sum::<f64>()/n as f64,tooth.iter().map(|p| p.1).sum::<f64>()/n as f64);
    let wrap = |a: f64| (a+PI).rem_euclid(TAU)-PI;
    // the arc of circle `r` about the blank's centre inside the tooth at roll `t`, in the blank's angles
    let arc = |r: f64,t: f64| -> Option<(f64,f64)> {
        let (cx,cy) = (0.,-pitch*t);
        let reference = (gy-cy).atan2(gx-cx);
        let (mut lo,mut hi) = (-PI,PI);
        for i in 0..n {
            let ((ax,ay),(bx,by)) = (tooth[i],tooth[(i+1)%n]);
            let (nx,ny) = (-(by-ay),bx-ax);
            let k = (nx*(ax-cx)+ny*(ay-cy))/(r*nx.hypot(ny));
            if k >= 1. { continue }
            if k <= -1. { return None }
            let (c,w) = (wrap(ny.atan2(nx)+PI-reference),PI-k.acos());
            (lo,hi) = (lo.max(c-w),hi.min(c+w));
            if lo >= hi { return None }
        }
        Some((reference+lo-t,reference+hi-t))
    };
    let step = (t1-t0)/rolls as f64;
    (0..radii).map(|i| {
        let r = inner+(outer-inner)*(i as f64+0.5)/radii as f64;
        let at = |t: f64| arc(r,t.clamp(t0,t1));
        let coarse: Vec<Option<(f64,f64)>> = (0..=rolls).map(|j| at(t0+step*j as f64)).collect();
        // the greatest of `key` over the roll: the coarse samples' best, read again finely beside it
        let best = |key: &dyn Fn((f64,f64)) -> f64| -> Option<f64> {
            let j = (0..=rolls).filter(|&j| coarse[j].is_some())
                .max_by(|&a,&b| key(coarse[a].unwrap()).total_cmp(&key(coarse[b].unwrap())))?;
            let t = t0+step*j as f64;
            (0..=400).filter_map(|k| at(t-step+2.*step*k as f64/400.)).map(key).max_by(f64::total_cmp)
        };
        match (best(&|(a,_)| -a),best(&|(_,b)| b)) {
            (Some(lo),Some(hi)) if hi > -lo => (hi+lo)*r*(outer-inner)/radii as f64,
            _ => 0.,
        }
    }).sum()
}

/// What `bar_end.sv`'s bar removes from the disc: at each height `z` of its 6 mm the bar's slice is
/// the rectangle 18 to 30 mm from the centre and `w(z)` either side of its axis (its profile's sides
/// run from 1 mm at z = -9 to 2 at z = 3), rolled over ±60° on the circle of 16; Simpson's rule
/// over 12 slices of the height. Each area's midpoints in the radius err at first order (the
/// floor's edge, where the space's arc at a radius stops short, falls between them), so 400 and
/// 800 of them are extrapolated (Richardson).
pub(crate) fn bar_end_space() -> f64 {
    let w = |z: f64| 1.+(z+9.)/12.;
    let slices = 12;
    let h = 6./slices as f64;
    let volume = |radii: usize| (0..=slices).map(|k| {
        let z = -6.+h*k as f64;
        let weight = if k == 0 || k == slices { 1. } else if k%2 == 1 { 4. } else { 2. };
        weight*rack_space(&[(18.,-w(z)),(18.,w(z)),(30.,w(z)),(30.,-w(z))],16.,[18.,22.],[-PI/3.,PI/3.],radii,1000)
    }).sum::<f64>()*h/3.;
    2.*volume(800)-volume(400)
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
        .replace("sweep(bar over t in (-60deg, 60deg), under: rack, seen_from: turn)","sweep(bar over t in (-25deg, 25deg), under: arm, seen_from: turn)");
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

/// The bar shared with a block (a `common`) is an intersection, no prism standing alone: its cap in
/// the disc is refused at T1, as every cap was before (#684 F).
#[test]
fn a_cap_of_a_tool_no_lone_prism_is_refused() {
    let source = bar_end().replace("construction space := sweep(bar","in std.top {\n  k0 := point hint((10, -10))\n  k1 := point hint((40, -10))\n  \
        k2 := point hint((40, 10))\n  k3 := point hint((10, 10))\n  fix((10, -10)) k0\n  fix((40, -10)) k1\n  fix((40, 10)) k2\n  fix((10, 10)) k3\n}\n\
        construction keep := solid(face(k0, k1, k2, k3, -> close), from: -20mm, to: 20mm)\nconstruction tool := solid(bar)\nkeep common tool\n\
        construction space := sweep(tool");
    let r = refused(&source,"part");
    assert_eq!(r.condition,Condition::Tool);
    assert!(r.message.contains("standing alone"),"{}",r.message);
}
