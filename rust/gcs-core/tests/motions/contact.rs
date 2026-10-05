use super::*;
use gcs_core::envelope::{self,Error,SurfacePoint};
use std::f64::consts::PI;

fn family(source_rate: f64,observer_rate: f64,source_phase: f64,observer_phase: f64) -> motion::Family {
    from_axes(AXES,source_rate,observer_rate,source_phase,observer_phase)
}
fn from_axes(axes: &str,source_rate: f64,observer_rate: f64,source_phase: f64,observer_phase: f64) -> motion::Family {
    let e = solved(&format!("{axes}\n\
        source := motion(about: ax,ratio: {source_rate},phase: {source_phase}deg)\n\
        observer := motion(about: other,ratio: {observer_rate},phase: {observer_phase}deg)\n\
        relative := motion(source,relative_to: observer)\n"));
    motion::Family::read(&e.sketch,e.map.ent_named("relative").unwrap().i()).unwrap()
}

#[test]
fn temporal_coefficients_match_full_motion_with_offset_axes_and_signed_rates() {
    let surfaces = [
        SurfacePoint {position:[2.,0.,1.],du:[0.,0.,1.],dv:[1.,0.,0.]},
        SurfacePoint {position:[-1.,2.,3.],du:[1.,2.,3.],dv:[-2.,1.,4.]},
        SurfacePoint {position:[1.,-2.,0.5],du:[-1.,1.,0.],dv:[0.,-2.,1.]},
    ];
    let (mut roots,mut empty) = (0,0);
    let shifted = AXES.replace("fix(x == 0, y == 0) o","fix(x == -1, y == 2) o")
        .replace("fix(x == 1, y == 0) x","fix(x == 3, y == 1) x");
    for axes in [AXES,shifted.as_str()] {
    for source_rate in [-2.,0.,0.7] { for observer_rate in [-0.5,0.,3.] {
        let family = from_axes(axes,source_rate,observer_rate,31.,-47.);
        for surface in surfaces {
            let harmonic = family.normal_velocity(surface).unwrap();
            let actual = |t| envelope::contact(surface,family.at(t).unwrap()).unwrap().normal_velocity;
            for i in -100..=100 {
                let t = i as f64/19.;
                let expected = actual(t);
                assert!((harmonic.at(t).unwrap()-expected).abs() < 1e-12,"{source_rate} {observer_rate} {t}");
            }
            if source_rate == 0. || observer_rate == 0. { continue; }
            let result = harmonic.roots([-4.,4.],1e-10,64).unwrap();
            let mut reference = Vec::new();
            // Independently find every sign crossing in this low-frequency
            // fixture using the original differentiated matrix composition.
            for i in 0..4000 {
                let (mut lo,mut hi) = (-4.+i as f64/500.,-4.+(i+1) as f64/500.);
                if actual(lo)*actual(hi) >= 0. { continue; }
                for _ in 0..45 {
                    let mid = (lo+hi)/2.;
                    if actual(lo)*actual(mid) <= 0. { hi = mid; } else { lo = mid; }
                }
                reference.push((lo+hi)/2.);
            }
            assert_eq!(result.len(),reference.len());
            if result.is_empty() { empty += 1; }
            for (root,expected) in result.iter().zip(reference) {
                assert!((root.time-expected).abs() < 1e-9);
                assert!(actual(root.time).abs() < 1e-10);
                roots += 1;
            }
        }
    } }
    }
    assert!(roots > 0 && empty > 0);
}

#[test]
fn temporal_roots_retain_windings_closed_endpoints_and_observer_phase() {
    let surface = SurfacePoint {position:[2.,0.,1.],du:[0.,0.,1.],dv:[1.,0.,0.]};
    let a = family(-2.,0.5,90.,0.);
    let b = family(-2.,0.5,90.,73.);
    let a_curve = a.normal_velocity(surface).unwrap();
    let b_curve = b.normal_velocity(surface).unwrap();
    // f(t) = 0.5*cos(pi/2-2*t). Both endpoints are roots.
    let roots = a_curve.roots([-PI,PI],1e-10,5).unwrap();
    let other = b_curve.roots([-PI,PI],1e-10,5).unwrap();
    assert_eq!(roots.len(),5);
    for (i,root) in roots.iter().enumerate() {
        assert!((root.time-(i as f64-2.)*PI/2.).abs() < 1e-12);
        assert!((root.time-other[i].time).abs() < 1e-12);
        let single = a_curve.roots([root.time,root.time],1e-10,1).unwrap();
        assert_eq!(single.len(),1);
        assert_eq!((single[0].branch,single[0].turn),(root.branch,root.turn));
    }
    assert_eq!(a_curve.roots([-PI,PI],1e-10,4).unwrap_err(),Error::NotConverged);
    assert_eq!(a_curve.roots([1.,0.],1e-10,5).unwrap_err(),Error::InvalidOptions);
    assert_eq!(a_curve.roots([0.,1.],0.,5).unwrap_err(),Error::InvalidOptions);
    assert_eq!(a_curve.roots([0.,1.],1e-10,0).unwrap_err(),Error::InvalidOptions);
    assert_eq!(a_curve.roots([0.,f64::NAN],1e-10,5).unwrap_err(),Error::NonFinite);
    assert!(a_curve.at(f64::INFINITY).is_err());
    let pa = a.at(0.).unwrap().point(surface.position);
    let pb = b.at(0.).unwrap().point(surface.position);
    assert!((pa[2]-pb[2]).abs() > 0.1,"observer phase still changes geometry");
}

#[test]
fn empty_constant_and_double_root_contacts_are_not_confused() {
    let surface = |x| SurfacePoint {position:[x,0.,1.],du:[0.,0.,1.],dv:[1.,0.,0.]};
    let relative = family(1.,0.5,0.,0.);
    assert!(relative.normal_velocity(surface(3.)).unwrap().roots([-1.,1.],1e-10,8).unwrap().is_empty());
    assert_eq!(relative.normal_velocity(surface(2.5)).unwrap().roots([-1.,1.],1e-10,8)
        .unwrap_err(),Error::Degenerate); // -0.5+0.5*cos(t)
    let stationary = family(0.,0.5,90.,0.);
    assert_eq!(stationary.normal_velocity(surface(2.)).unwrap().roots([-1.,1.],1e-10,8)
        .unwrap_err(),Error::Degenerate);
    let e = solved(&format!("{AXES}\nspin := motion(about: ax)\n\
        identity := motion(spin,relative_to: spin)\n\
        nested := motion(identity,relative_to: spin)\n"));
    let read = |name| motion::Family::read(&e.sketch,e.map.ent_named(name).unwrap().i()).unwrap();
    assert_eq!(read("spin").normal_velocity(surface(2.)).unwrap().roots([-1.,1.],1e-10,8)
        .unwrap_err(),Error::Degenerate);
    assert!(read("spin").normal_velocity(surface(3.)).unwrap().roots([-1.,1.],1e-10,8).unwrap().is_empty());
    assert!(read("nested").normal_velocity(surface(2.)).unwrap_err().contains("two relative rotations"));
    // Roots far outside the requested interval need not be representable as times.
    assert!(family(1e-320,0.5,0.,0.).normal_velocity(surface(2.)).unwrap()
        .roots([-1.,1.],1e-10,8).unwrap().is_empty());
}

/// **A rack's contact equation is a line in the roll.**  A translation along `other` seen from a
/// rotation about `axis` (a rack against the blank it cuts) carries the normal unturned, so its
/// normal velocity is `s n·d − n·(ω × (p − o)) − s t n·(ω × d)`: checked against the full
/// differentiated motion at every sample, and its one root against bisection of that.
#[test]
fn a_racks_contact_equation_is_affine_in_the_roll() {
    let surfaces = [
        SurfacePoint {position:[2.,0.,1.],du:[0.,0.,1.],dv:[1.,0.,0.]},
        SurfacePoint {position:[-1.,2.,3.],du:[1.,2.,3.],dv:[-2.,1.,4.]},
        SurfacePoint {position:[1.,-2.,0.5],du:[-1.,1.,0.],dv:[0.,-2.,1.]},
    ];
    let mut found = 0;
    for (advance,ratio) in [(5.,0.7),(-3.,-1.5),(12.,2.)] {
        let e = solved(&format!("{AXES}\n\
            slide := motion(along: other,advance: {advance})\n\
            blank := motion(about: ax,ratio: {ratio},phase: 20deg)\n\
            rack := motion(slide,relative_to: blank)\n"));
        let family = motion::Family::read(&e.sketch,e.map.ent_named("rack").unwrap().i()).unwrap();
        for surface in surfaces {
            let line = family.normal_velocity(surface).unwrap();
            assert!(matches!(line,motion::NormalVelocity::Affine {..}));
            let actual = |t| envelope::contact(surface,family.at(t).unwrap()).unwrap().normal_velocity;
            for i in -40..=40 {
                let t = i as f64/9.;
                assert!((line.at(t).unwrap()-actual(t)).abs() < 1e-11,"{advance} {ratio} at {t}");
            }
            let roots = line.roots([-4.,4.],1e-10,8).unwrap();
            let (lo,hi) = (actual(-4.),actual(4.));
            assert_eq!(roots.len(),usize::from(lo*hi < 0.),"{advance} {ratio}: one root where it changes sign");
            for root in roots {
                assert!(actual(root.time).abs() < 1e-10);
                found += 1;
            }
        }
    }
    assert!(found >= 3,"only {found} roots among the samples");
}
