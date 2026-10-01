//! The core's own elementary functions (`fmath`), the same bits on every target, held to the
//! platform's within two units in the last place (each of the two within one of the truth; `tan`
//! three, and `pow`, whose logarithm's error its exponent multiplies, sixteen), and exact where
//! they must be.
use gcs_core::fmath::*;

fn ulps(a: f64,b: f64) -> u64 {
    if a == b { return 0 }
    if a.is_nan() || b.is_nan() || a.signum() != b.signum() { return u64::MAX }
    (a.to_bits() as i64-b.to_bits() as i64).unsigned_abs()
}

fn check(name: &str,ours: &dyn Fn(f64) -> f64,theirs: &dyn Fn(f64) -> f64,xs: &[f64],most: u64) {
    let mut worst = (0,0.);
    for &x in xs {
        let (a,b) = (ours(x),theirs(x));
        if a.is_nan() && b.is_nan() { continue }
        let u = if b == 0. { if a.abs() < 1e-300 { 0 } else { u64::MAX } } else { ulps(a,b) };
        if u > worst.0 { worst = (u,x); }
    }
    assert!(worst.0 <= most,"{name}: {} ulps at {:e}: {} against {}",worst.0,worst.1,ours(worst.1),theirs(worst.1));
}

fn samples(lo: f64,hi: f64,n: usize) -> Vec<f64> {
    let mut rng = gcs_core::rng::Rng::new(0xf3a7);
    (0..n).map(|_| rng.uniform(lo,hi)).collect()
}

#[test]
fn the_trigonometric_functions_are_within_two_ulps() {
    let mut xs = samples(-10.,10.,20000);
    xs.extend(samples(-1e4,1e4,5000));
    xs.extend(samples(-0.8,0.8,5000));
    xs.extend([0.,-0.,1e-9,-1e-9,std::f64::consts::FRAC_PI_4,std::f64::consts::FRAC_PI_2,std::f64::consts::PI,1e5,1e6]);
    check("sin",&sin,&f64::sin,&xs,2);
    check("cos",&cos,&f64::cos,&xs,2);
    check("tan",&tan,&f64::tan,&xs,3);
    for &x in &xs { let (s,c) = sin_cos(x); assert_eq!((s.to_bits(),c.to_bits()),(sin(x).to_bits(),cos(x).to_bits())); }
    let unit = samples(-1.,1.,20000);
    check("asin",&asin,&f64::asin,&unit,2);
    check("acos",&acos,&f64::acos,&unit,2);
    let mut wide = samples(-50.,50.,20000);
    wide.extend(samples(-1e8,1e8,2000));
    check("atan",&atan,&f64::atan,&wide,2);
    let pairs = samples(-100.,100.,40000);
    for w in pairs.chunks(2) {
        let (a,b) = (atan2(w[0],w[1]),w[0].atan2(w[1]));
        assert!(ulps(a,b) <= 2,"atan2({}, {}): {a} against {b}",w[0],w[1]);
    }
    // the axes exactly
    assert_eq!(atan2(0.,1.),0.);
    assert_eq!(atan2(1.,0.),std::f64::consts::FRAC_PI_2);
    assert_eq!(atan2(0.,-1.),std::f64::consts::PI);
    assert_eq!(atan2(-1.,0.),-std::f64::consts::FRAC_PI_2);
    assert_eq!(asin(1.),std::f64::consts::FRAC_PI_2);
    assert_eq!(acos(-1.),std::f64::consts::PI);
    assert_eq!(acos(1.),0.);
}

#[test]
fn the_exponentials_and_logarithms_are_within_two_ulps() {
    let mut xs = samples(-700.,700.,20000);
    xs.extend(samples(-1.,1.,5000));
    check("exp",&exp,&f64::exp,&xs,2);
    let mut pos = samples(1e-300,1e300,5000);
    pos.extend(samples(0.,10.,20000));
    pos.extend([1.,2.,0.5,1e-310,f64::MAX]);
    check("ln",&ln,&f64::ln,&pos,2);
    check("log10",&log10,&f64::log10,&pos,2);
    check("log2",&log2,&f64::log2,&pos,2);
    for k in 0..23 { assert_eq!(log10(10f64.powi(k)),k as f64,"log10 of 1e{k}"); }
    for k in -1022..1023 { assert_eq!(log2(2f64.powi(k)),k as f64); }
    assert_eq!(ln(1.),0.);
    assert!(ln(0.) == f64::NEG_INFINITY && ln(-1.).is_nan());
    let pairs: Vec<f64> = samples(0.01,20.,20000);
    for w in pairs.chunks(2) {
        let (a,b) = (pow(w[0],w[1]-10.),w[0].powf(w[1]-10.));
        assert!(ulps(a,b) <= 16,"pow({}, {}): {a} against {b}",w[0],w[1]-10.);
    }
    assert_eq!(pow(2.,10.),1024.);
    assert_eq!(pow(-2.,3.),-8.);
    assert!(pow(-2.,0.5).is_nan());
    for &x in &samples(-3.,3.,1000) { for n in -6..7 { assert!(ulps(powi(x,n),x.powi(n)) <= 4,"{x}^{n}"); } }
}

#[test]
fn hypot_is_within_one_ulp_and_never_overflows() {
    let pairs = samples(-1e3,1e3,40000);
    for w in pairs.chunks(2) {
        let (a,b) = (hypot(w[0],w[1]),w[0].hypot(w[1]));
        assert!(ulps(a,b) <= 1,"hypot({}, {}): {a} against {b}",w[0],w[1]);
    }
    assert_eq!(hypot(3.,4.),5.);
    assert_eq!(hypot(1e300,1e300),1e300*2f64.sqrt());
    assert!((hypot(1e-300,1e-300)-1e-300*2f64.sqrt()).abs() < 1e-315);
    assert_eq!(hypot(f64::INFINITY,f64::NAN),f64::INFINITY);
}
