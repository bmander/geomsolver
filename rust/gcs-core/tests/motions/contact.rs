use super::*;
use gcs_core::envelope::{self,Error,SurfacePoint};
use std::f64::consts::PI;

fn family(source_rate: f64,observer_rate: f64) -> motion::Family {
    from_axes(AXES,source_rate,observer_rate)
}
/// The turn about `ax` seen from the one about `other`.  Roll 0 is as drawn: where a member must
/// stand turned there, the surface point it carries is drawn turned (`turned`).  An observer that
/// does not turn is a frame at rest, the world: a turn at ratio 0 is a slide that goes nowhere
/// (`program::motions`), and a turn seen from a slide is no contact the reduction reads.
fn from_axes(axes: &str,source_rate: f64,observer_rate: f64) -> motion::Family {
    let observer = if observer_rate == 0. { "observer := frame\n".to_string() } else {
        format!("observer := frame\nobserver turns({observer_rate}) other\n")
    };
    let e = solved(&format!("{axes}\n\
        source := frame\nsource turns({source_rate}) ax\n{observer}{}",
        reading("source","observer")));
    let node = if observer_rate == 0. { joint(&e,"source") } else { seen(&e,"source","observer") };
    motion::Family::read(&e.sketch,node).unwrap()
}

/// The surface point `[2, 0, 1]` on `ax` (world z through (2, 0, 0)), its second tangent turned
/// `degrees` about it from x: the member drawn turned.
fn turned(degrees: f64) -> SurfacePoint {
    let (s,c) = degrees.to_radians().sin_cos();
    SurfacePoint {position:[2.,0.,1.],du:[0.,0.,1.],dv:[c,s,0.]}
}

#[test]
fn temporal_coefficients_match_full_motion_with_offset_axes_and_signed_rates() {
    let surfaces = [
        SurfacePoint {position:[2.,0.,1.],du:[0.,0.,1.],dv:[1.,0.,0.]},
        SurfacePoint {position:[-1.,2.,3.],du:[1.,2.,3.],dv:[-2.,1.,4.]},
        SurfacePoint {position:[1.,-2.,0.5],du:[-1.,1.,0.],dv:[0.,-2.,1.]},
    ];
    let (mut roots,mut empty) = (0,0);
    let shifted = AXES.replace("fix((0, 0)) o","fix((-1, 2)) o")
        .replace("fix((1, 0)) x","fix((3, 1)) x");
    for axes in [AXES,shifted.as_str()] {
    for source_rate in [-2.,0.,0.7] { for observer_rate in [-0.5,0.,3.] {
        // neither turning, there is a slide that goes nowhere and nothing seeing it: no rotation
        if source_rate == 0. && observer_rate == 0. { continue; }
        let family = from_axes(axes,source_rate,observer_rate);
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

/// The roots keep their windings and a closed interval's endpoints, and an observer standing
/// turned at roll 0 moves the geometry and not the contact.  Roll 0 is as drawn, so the source's
/// quarter turn there is its surface drawn a quarter turned; the observer 73° on is the same pair
/// read 146° later (73° at its ratio of ½), the surface drawn turned back by what the source turns
/// in those 146°.
#[test]
fn temporal_roots_retain_windings_closed_endpoints_and_an_observer_standing_turned() {
    let a = family(-2.,0.5);
    let shift = 146f64.to_radians();
    let surface = turned(90.);
    let later = turned(90.+2.*146.);
    let a_curve = a.normal_velocity(surface).unwrap();
    let b_curve = a.normal_velocity(later).unwrap();
    // f(t) = 0.5*cos(pi/2-2*t). Both endpoints are roots.
    let roots = a_curve.roots([-PI,PI],1e-10,5).unwrap();
    let other = b_curve.roots([-PI+shift,PI+shift],1e-10,5).unwrap();
    assert_eq!(roots.len(),5);
    for (i,root) in roots.iter().enumerate() {
        assert!((root.time-(i as f64-2.)*PI/2.).abs() < 1e-12);
        assert!((root.time-(other[i].time-shift)).abs() < 1e-12);
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
    let pb = a.at(shift).unwrap().point(later.position);
    assert!((pa[2]-pb[2]).abs() > 0.1,"the observer standing turned still changes geometry");
}

#[test]
fn empty_constant_and_double_root_contacts_are_not_confused() {
    let surface = |x| SurfacePoint {position:[x,0.,1.],du:[0.,0.,1.],dv:[1.,0.,0.]};
    let relative = family(1.,0.5);
    assert!(relative.normal_velocity(surface(3.)).unwrap().roots([-1.,1.],1e-10,8).unwrap().is_empty());
    assert_eq!(relative.normal_velocity(surface(2.5)).unwrap().roots([-1.,1.],1e-10,8)
        .unwrap_err(),Error::Degenerate); // -0.5+0.5*cos(t)
    let stationary = family(0.,0.5);
    assert_eq!(stationary.normal_velocity(turned(90.)).unwrap().roots([-1.,1.],1e-10,8)
        .unwrap_err(),Error::Degenerate);
    let mut e = solved(&format!("{AXES}\nspin := frame\nspin turns(1) ax\n{}",
        reading("spin","spin")));
    // the spin seen from itself, seen from the spin again: made by hand, as a document reads one
    // frame seen from another
    let (identity,spin) = (seen(&e,"spin","spin"),joint(&e,"spin"));
    let nested = push_relative(&mut e.sketch,identity,spin);
    let read = |m| motion::Family::read(&e.sketch,m).unwrap();
    assert_eq!(read(spin).normal_velocity(surface(2.)).unwrap().roots([-1.,1.],1e-10,8)
        .unwrap_err(),Error::Degenerate);
    assert!(read(spin).normal_velocity(surface(3.)).unwrap()
        .roots([-1.,1.],1e-10,8).unwrap().is_empty());
    assert!(read(nested).normal_velocity(surface(2.)).unwrap_err()
        .contains("two relative rotations"));
    // Roots far outside the requested interval need not be representable as times.
    assert!(family(1e-320,0.5).normal_velocity(surface(2.)).unwrap()
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
            slide := frame\nslide slides({advance}mm) other\n\
            blank := frame\nblank turns({ratio}) ax\n{}",
            reading("slide","blank")));
        let family = motion::Family::read(&e.sketch,seen(&e,"slide","blank")).unwrap();
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
