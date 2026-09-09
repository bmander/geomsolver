use super::*;
use gcs_core::interval::Interval as I;
fn dot(a: [f64;3],b: [f64;3]) -> f64 { (0..3).map(|k| a[k]*b[k]).sum() }
fn cross(a: [I;3],b: [I;3]) -> [I;3] {
    std::array::from_fn(|k| a[(k+1)%3].mul(b[(k+2)%3]).unwrap()
        .sub(a[(k+2)%3].mul(b[(k+1)%3]).unwrap()).unwrap())
}

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
        let moment_equation = motion.normal_velocity_moment_bounds(normal,cross(position,normal)).unwrap();
        assert_eq!(equation.is_time_independent(),rate == 0. || observer == 0.);
        for (lo,hi) in [(-1.,1.),(0.13,0.14)] {
            let domain = I::new(lo,hi).unwrap();
            let value = equation.at(domain).unwrap(); let derivative = equation.derivative(domain).unwrap();
            let moment_value = moment_equation.at(domain).unwrap();
            for i in 0..=8 {
                let t = lo+(hi-lo)*i as f64/8.;
                let point_in = |p: [I;3]| p.map(|x| { let [a,b] = x.bounds(); a+(b-a)*i as f64/8. });
                let p = point_in(position); let n = point_in(normal);
                let actual = |t| { let m = motion.at(t).unwrap(); dot(m.vector(n),m.velocity(p)) };
                assert!(value.contains(actual(t)),"{rate} {observer}: {} vs {value:?}",actual(t));
                assert!(moment_value.contains(actual(t)),"moment {rate} {observer}: {} vs {moment_value:?}",actual(t));
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

#[test]
fn source_directional_contact_bounds_include_both_product_rule_terms_and_offsets() {
    let shifted = AXES.replace("point o hint(x: 0,y: 0)","point o hint(x: -1,y: 2)")
        .replace("point x hint(x: 1,y: 0)","point x hint(x: 3,y: 1)");
    let (s,c) = I::new(0.17,0.21).unwrap().sin_cos().unwrap();
    let two = I::point(2.).unwrap(); let minus_two = I::point(-2.).unwrap();
    let p = [I::ONE.add(c.mul(two).unwrap()).unwrap(),minus_two.add(s.mul(two).unwrap()).unwrap(),
        I::new(0.67,0.71).unwrap()];
    let n = [c,s,I::point(0.3).unwrap()];
    let dp = [s.mul(minus_two).unwrap(),c.mul(two).unwrap(),I::ONE];
    let dn = [s.neg(),c,I::ZERO];
    for rate in [-2.,0.,0.7] { for observer in [-0.5,0.,3.] {
        let motion = from_axes(&shifted,rate,observer,31.,-47.);
        let equation = motion.normal_velocity_directional_bounds(p,n,dp,dn).unwrap();
        let bound = equation.at(I::new(-0.8,0.7).unwrap()).unwrap();
        let (a,b) = (cross(dp,n),cross(p,dn));
        let dm = std::array::from_fn(|k| a[k].add(b[k]).unwrap());
        let moment_bound = motion.normal_velocity_moment_bounds(dn,dm).unwrap().at(I::new(-0.8,0.7).unwrap()).unwrap();
        for i in 0..=10 { for j in 0..=10 {
            let v = 0.17+0.04*i as f64/10.; let t = -0.8+1.5*j as f64/10.;
            let (s,c) = v.sin_cos();
            let p = [1.+2.*c,-2.+2.*s,0.5+v]; let n = [c,s,0.3];
            let dp = [-2.*s,2.*c,1.]; let dn = [-s,c,0.];
            let m = motion.at(t).unwrap();
            let vector_velocity: [f64;3] = std::array::from_fn(|k| m.velocity(dp)[k]-m.velocity([0.;3])[k]);
            let expected = dot(m.vector(dn),m.velocity(p))+dot(m.vector(n),vector_velocity);
            assert!(bound.contains(expected),"{rate} {observer}: {expected} vs {bound:?}");
            assert!(moment_bound.contains(expected),"moment {rate} {observer}: {expected} vs {moment_bound:?}");
        } }
    } }
}
