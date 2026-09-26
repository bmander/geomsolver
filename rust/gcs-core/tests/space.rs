//! `space`, the one home of the vector arithmetic in three dimensions: each helper against the
//! property it exists for, over seeded random points, and the degenerate inputs that must come
//! back as none rather than as a number.
use gcs_core::interval::Interval;
use gcs_core::rng::Rng;
use gcs_core::space::*;

type V = [f64;3];

fn point(rng: &mut Rng) -> V { std::array::from_fn(|_| rng.uniform(-10.,10.)) }
fn close(a: f64,b: f64,scale: f64) -> bool { (a-b).abs() <= 1e-9*scale.max(1.) }

#[test]
fn cross_is_antisymmetric_and_orthogonal_to_both() {
    let mut rng = Rng::new(0x5ace);
    for _ in 0..1000 {
        let (a,b) = (point(&mut rng),point(&mut rng));
        let (ab,ba) = (cross(a,b),cross(b,a));
        assert_eq!(ab,ba.map(|x| -x));
        let scale = norm(a)*norm(b);
        assert!(close(dot(ab,a),0.,scale*norm(a)) && close(dot(ab,b),0.,scale*norm(b)));
        // |a × b|² + (a · b)² = |a|² |b|² (Lagrange)
        assert!(close(dot(ab,ab)+dot(a,b)*dot(a,b),dot(a,a)*dot(b,b),scale*scale));
    }
    assert_eq!(cross([1.,0.,0.],[0.,1.,0.]),[0.,0.,1.]);
}

#[test]
fn the_two_lengths_agree_but_are_different_numbers_by_design() {
    let mut rng = Rng::new(7);
    for _ in 0..1000 {
        let a = point(&mut rng);
        assert!(close(norm(a),length(a),norm(a)));
        assert_eq!(norm(a),dot(a,a).sqrt());
        assert_eq!(distance(a,[0.;3]),norm(a));
        assert_eq!(distance_squared(a,[0.;3]),dot(a,a));
    }
    // `hypot` neither overflows nor underflows where the squares would
    assert!((length([3e200,4e200,0.])/5e200-1.).abs() < 1e-15);
    assert!(norm([3e200,4e200,0.]).is_infinite());
    assert!((length([3e-200,4e-200,0.])/5e-200-1.).abs() < 1e-15);
    assert_eq!(norm([3e-200,4e-200,0.]),0.);
}

#[test]
fn normalised_is_unit_or_none() {
    let mut rng = Rng::new(11);
    for _ in 0..200 {
        let a = point(&mut rng);
        assert!(close(norm(normalised(a).unwrap()),1.,1.));
    }
    assert_eq!(normalised([0.;3]),None);
    assert_eq!(normalised([-0.,0.,-0.]),None);
}

#[test]
fn lerp_meets_its_ends_exactly() {
    let mut rng = Rng::new(13);
    for _ in 0..200 {
        let (a,b) = (point(&mut rng),point(&mut rng));
        assert_eq!(lerp(a,b,0.),a);
        let m = lerp(a,b,0.5);
        assert!(close(distance(a,m),distance(m,b),distance(a,b)));
        assert!(close(distance(lerp(a,b,1.),b),0.,norm(b)));
    }
}

#[test]
fn segment_distance_is_the_nearest_point_of_the_segment() {
    let mut rng = Rng::new(17);
    for _ in 0..500 {
        let (p,a,b) = (point(&mut rng),point(&mut rng),point(&mut rng));
        let d = segment_distance(p,a,b);
        // never farther than either end, never nearer than any sample of the segment
        assert!(d <= distance(p,a)+1e-12 && d <= distance(p,b)+1e-12);
        let sampled = (0..=1000).map(|i| distance(p,lerp(a,b,i as f64/1000.))).fold(f64::INFINITY,f64::min);
        assert!(d <= sampled+1e-12 && sampled-d <= 1e-2*distance(a,b));
        // the nearest interior point sees the segment square on
        let c = closest_on_segment(p,a,b);
        if c != a && c != b { assert!(close(dot(sub(p,c),sub(b,a)),0.,distance(a,b)*(1.+d))); }
    }
    // a segment of no length is its point
    assert_eq!(closest_on_segment([1.,2.,3.],[4.;3],[4.;3]),[4.;3]);
    assert_eq!(segment_distance([4.,4.,0.],[4.;3],[4.;3]),4.);
}

#[test]
fn polyline_length_sums_its_sides() {
    assert_eq!(polyline_length(&[]),0.);
    assert_eq!(polyline_length(&[[1.;3]]),0.);
    assert_eq!(polyline_length(&[[0.;3],[3.,4.,0.],[3.,4.,12.]]),17.);
}

#[test]
fn circumcentre_is_equidistant_and_in_the_plane() {
    let mut rng = Rng::new(19);
    for _ in 0..500 {
        let (a,b,c) = (point(&mut rng),point(&mut rng),point(&mut rng));
        let o = circumcentre(a,b,c).unwrap();
        let r = distance(o,a);
        assert!(close(distance(o,b),r,r) && close(distance(o,c),r,r),"{a:?} {b:?} {c:?}");
        let n = cross(sub(b,a),sub(c,a));
        assert!(close(dot(sub(o,a),n),0.,norm(n)*(1.+r)));
    }
    assert_eq!(circumcentre([0.;3],[2.,0.,0.],[0.,2.,0.]),Some([1.,1.,0.]));
}

#[test]
fn orthocentre_has_equal_power_to_its_weighted_corners() {
    let mut rng = Rng::new(23);
    for _ in 0..500 {
        let (a,b,c) = (point(&mut rng),point(&mut rng),point(&mut rng));
        let w: [f64;3] = std::array::from_fn(|_| rng.uniform(0.,4.));
        let (o,n) = orthocentre(a,b,c,w).unwrap();
        assert_eq!(n,cross(sub(b,a),sub(c,a)));
        let power = [a,b,c].map(|p| distance_squared(o,p));
        let power = [power[0]-w[0],power[1]-w[1],power[2]-w[2]];
        let scale = power.iter().fold(1_f64,|m,x| m.max(x.abs()));
        assert!(close(power[1],power[0],scale*10.) && close(power[2],power[0],scale*10.),"{power:?}");
        assert!(close(dot(sub(o,a),n),0.,norm(n)*(1.+scale)));
    }
    // zero weights are the circumcentre, to the bit
    let (a,b,c) = ([0.3,-1.,2.],[4.,0.5,-1.],[-2.,3.,0.25]);
    assert_eq!(orthocentre(a,b,c,[0.;3]).map(|(o,_)| o),circumcentre(a,b,c));
}

#[test]
fn a_triangle_with_no_area_has_no_centre_or_normal() {
    let (a,b) = ([0.;3],[1.,2.,3.]);
    assert_eq!(circumcentre(a,b,b),None);
    assert_eq!(circumcentre(a,b,lerp(a,b,0.25)),None);
    assert_eq!(orthocentre(a,a,a,[1.,2.,3.]),None);
    assert_eq!(triangle_normal(a,b,[2.,4.,6.]),None);
    assert!(degenerate(a,b,[2.,4.,6.]));
    assert_eq!(altitude(a,b,b),0.);
    assert_eq!(triangle_normal([0.;3],[1.,0.,0.],[0.,1.,0.]),Some([0.,0.,1.]));
}

#[test]
fn a_box_has_its_middle_and_its_diagonal() {
    let b = [Interval::new(-1.,3.).unwrap(),Interval::new(0.,0.).unwrap(),Interval::new(2.,5.).unwrap()];
    assert_eq!(box_centre_diagonal(&b),([1.,0.,3.5],5.));
    assert_eq!(box_centre_diagonal(&[Interval::point(2.).unwrap();3]),([2.;3],0.));
}
