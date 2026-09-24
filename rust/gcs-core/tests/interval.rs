use gcs_core::interval::{Error,Interval as I};

mod minimum;

fn point(x: f64) -> I { I::point(x).unwrap() }
fn interval(a: f64,b: f64) -> I { I::new(a,b).unwrap() }

#[test]
fn interval_bounds_refuse_invalid_domains_and_never_drop_singular_values() {
    for (a,b) in [(1.,0.),(f64::NAN,1.),(0.,f64::INFINITY)] {
        assert_eq!(I::new(a,b).unwrap_err(),Error::InvalidBounds);
    }
    for zero in [interval(-1.,1.),interval(0.,1.),interval(-1.,-0.),I::ZERO] {
        assert_eq!(I::ONE.div(zero).unwrap_err(),Error::DivisionByZero);
    }
    assert_eq!(interval(-1e-300,1.).sqrt().unwrap_err(),Error::OutsideDomain);
    assert_eq!(point(f64::MAX).add(point(f64::MAX)).unwrap_err(),Error::Overflow);
    assert_eq!(point(f64::MAX).mul(point(2.)).unwrap_err(),Error::Overflow);
    assert_eq!(point(8.00001).sin_cos().unwrap_err(),Error::OutsideDomain);
    assert_eq!(point(-8.00001).sin_cos().unwrap_err(),Error::OutsideDomain);
}

#[test]
fn interval_arithmetic_handles_signs_cancellation_and_subnormal_rounding() {
    assert!(point(0.1).add(point(0.2)).unwrap().contains(0.3));
    assert!(point(1e100).add(I::ONE).unwrap().sub(point(1e100)).unwrap().contains(1.));
    let tiny = f64::from_bits(1);
    let p = point(tiny).mul(point(0.5)).unwrap();
    assert!(p.bounds()[0] <= 0. && p.bounds()[1] >= tiny);
    assert!(point(-tiny).div(point(2.)).unwrap().contains(-tiny));
    assert!(interval(-2.,3.).square().unwrap().contains(0.));
    assert!(interval(-2.,3.).square().unwrap().contains(9.));
    assert_eq!(I::ZERO.sqrt().unwrap().bounds()[0],0.);
    let x = interval(-3.,-2.).mul(interval(-5.,7.)).unwrap();
    assert!(x.contains(-21.) && x.contains(15.));
    let x = interval(-3.,-2.).div(interval(-5.,-4.)).unwrap();
    assert!(x.contains(0.4) && x.contains(0.75));
    assert_eq!(interval(-3.,4.).neg().bounds(),[-4.,3.]);
}

#[test]
fn polynomial_trig_enclosures_cover_extrema_and_small_and_wide_intervals() {
    for input in [interval(-8.,8.),interval(-1.,2.),interval(3.,3.3),interval(-0.001,0.001)] {
        let (s,c) = input.sin_cos().unwrap();
        for i in 0..=200 {
            let [a,b] = input.bounds(); let x = (1.-i as f64/200.)*a+i as f64/200.*b;
            assert!(s.contains(x.sin()),"{input:?}: {s:?}");
            assert!(c.contains(x.cos()),"{input:?}: {c:?}");
        }
    }
    assert!(interval(1.5,1.6).sin_cos().unwrap().0.contains(1.));
    assert!(interval(3.,3.3).sin_cos().unwrap().1.contains(-1.));
    for i in -80..=80 {
        let x = i as f64/10.;
        let (s,c) = point(x).sin_cos().unwrap();
        assert!(s.contains(x.sin()) && c.contains(x.cos()),"x={x}: {s:?} {c:?}");
        assert!(s.bounds()[1]-s.bounds()[0] < 1e-10);
        assert!(c.bounds()[1]-c.bounds()[0] < 1e-10);
    }
}

#[test]
fn narrow_trig_boxes_stay_tight_away_from_the_taylor_origin() {
    for center in [-7.9,-6.28,-3.14,0.,3.14,6.28,7.9] {
        let input = interval(center-0.001,center+0.001);
        let (s,c) = input.sin_cos().unwrap();
        for bound in [s,c] { assert!(bound.bounds()[1]-bound.bounds()[0] < 0.0021,"{input:?}: {bound:?}"); }
        for i in 0..=40 {
            let t = center-0.001+i as f64*0.002/40.;
            assert!(s.contains(t.sin()) && c.contains(t.cos()));
        }
    }
}

#[test]
fn interval_results_can_be_checked_with_independent_exact_rational_arithmetic() {
    let mut rows = String::new();
    let mut record = |op: &str,a: I,b: I,result: Result<I,Error>| {
        if let Ok(result) = result {
            rows.push_str(op);
            for x in a.bounds().into_iter().chain(b.bounds()).chain(result.bounds()) {
                rows.push_str(&format!("\t{:016x}",x.to_bits()));
            }
            rows.push('\n');
        }
    };
    // Dyadic endpoints give the independent verifier exact, unambiguous inputs.
    // Include overflow/underflow neighborhoods as well as ordinary geometric scales.
    let values = [0.,f64::from_bits(1),f64::MIN_POSITIVE,1e-150,0.1,1.,3.,1e100,1e150];
    for (i,&a) in values.iter().enumerate() {
        for (j,&b) in values.iter().enumerate() {
            for sign in [-1.,1.] {
                let a = interval((sign*a).min(a),(sign*a).max(a));
                let b = interval((sign*b).min(b),(sign*b).max(b));
                record("add",a,b,a.add(b)); record("sub",a,b,a.sub(b));
                record("mul",a,b,a.mul(b)); record("div",a,b,a.div(b));
                record("square",a,I::ZERO,a.square());
                record("sqrt",a,I::ZERO,a.sqrt());
                let x = point((i as f64-j as f64)*0.9);
                let (s,c) = x.sin_cos().unwrap();
                record("sin",x,I::ZERO,Ok(s)); record("cos",x,I::ZERO,Ok(c));
            }
        }
    }
    // Check the new whole-box bounds at independently evaluated rational points,
    // including intervals far from the Taylor origin. These sampled checks
    // complement the angle-addition and displacement bounds used by the code.
    for center in [-7.5,-6.25,-3.25,0.,3.25,6.25,7.5] { for radius in [0.001,0.01,0.5] {
        let input = interval(center-radius,center+radius);
        let (s,c) = input.sin_cos().unwrap();
        for i in 0..=16 {
            let x = point(center-radius+2.*radius*i as f64/16.);
            record("sin",x,I::ZERO,Ok(s)); record("cos",x,I::ZERO,Ok(c));
        }
    } }
    if let Some(file) = std::env::var_os("SOLVENT_INTERVAL_OUTPUT") {
        std::fs::write(file,rows).unwrap();
    }
}

#[test]
fn periodic_sine_and_cosine_enclose_every_angle_of_any_interval() {
    // Narrow intervals far out (several turns, both signs), one exactly a period's worth wide,
    // one just under and one wider: every sampled angle's sine and cosine lie in the enclosure.
    let cases = [(18.8,18.9),(-40.,-39.),(100.,103.),(8.,8.5),(0.,18.85),(20.,20.+std::f64::consts::TAU),
        (30.,36.2),(1e4,1e4+0.1)];
    for (a,b) in cases {
        let (s,c) = interval(a,b).sin_cos_periodic().unwrap();
        for k in 0..=400 {
            let x = a+(b-a)*k as f64/400.;
            assert!(s.contains(x.sin()) && c.contains(x.cos()),"[{a}, {b}] at {x}: {s:?} {c:?}");
        }
        // No looser than it must be where it is narrow: a tenth of a radian stays under 0.25 wide.
        if b-a <= 0.1 { assert!(s.bounds()[1]-s.bounds()[0] < 0.25 && c.bounds()[1]-c.bounds()[0] < 0.25,"{s:?} {c:?}"); }
    }
    // Inside [-8,8] it is `sin_cos` itself.
    assert_eq!(interval(1.,2.).sin_cos_periodic().unwrap(),interval(1.,2.).sin_cos().unwrap());
}
