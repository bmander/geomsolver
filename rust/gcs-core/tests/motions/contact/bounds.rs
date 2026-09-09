use super::*;
use gcs_core::interval::Interval as I;
fn dot(a: [f64;3],b: [f64;3]) -> f64 { (0..3).map(|k| a[k]*b[k]).sum() }

#[test]
fn interval_contact_coefficients_cover_source_boxes_and_full_motion_intervals() {
    let shifted = AXES.replace("point o hint(x: 0,y: 0)","point o hint(x: -1,y: 2)")
        .replace("point x hint(x: 1,y: 0)","point x hint(x: 3,y: 1)");
    for axes in [AXES,shifted.as_str()] {
    for rate in [-2.,0.,0.7] { for observer in [-0.5,0.,3.] {
        let motion = from_axes(axes,rate,observer,31.,-47.);
        let box_around = |p: [f64;3]| p.map(|x| I::new(x-0.01,x+0.01).unwrap());
        let position = box_around([1.,-2.,0.5]); let normal = box_around([1.,1.,2.]);
        let equation = motion.normal_velocity_bounds(position,normal).unwrap();
        assert_eq!(equation.is_time_independent(),rate == 0. || observer == 0.);
        for (lo,hi) in [(-1.,1.),(0.13,0.14)] {
            let domain = I::new(lo,hi).unwrap();
            let value = equation.at(domain).unwrap(); let derivative = equation.derivative(domain).unwrap();
            for i in 0..=8 {
                let t = lo+(hi-lo)*i as f64/8.;
                let point_in = |p: [I;3]| p.map(|x| { let [a,b] = x.bounds(); a+(b-a)*i as f64/8. });
                let p = point_in(position); let n = point_in(normal);
                let actual = |t| { let m = motion.at(t).unwrap(); dot(m.vector(n),m.velocity(p)) };
                assert!(value.contains(actual(t)),"{rate} {observer}: {} vs {value:?}",actual(t));
                if rate != 0. {
                    // Independent differentiated matrix samples at a quarter turn
                    // give the derivative of this one-frequency motion exactly,
                    // apart from floating arithmetic (covered here by the box width).
                    let delta = PI/(2.*rate);
                    let expected = rate*(actual(t+delta)-actual(t-delta))/2.;
                    if observer != 0. { assert!(derivative.contains(expected),"{expected} vs {derivative:?}"); }
                }
            }
            if equation.is_time_independent() { assert_eq!(derivative,I::ZERO); }
        }
    } } }
}
