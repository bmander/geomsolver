use super::*;
use gcs_core::interval::{Error,Interval as I};

fn box_at(p: [f64;3]) -> [I;3] { p.map(|v| I::point(v).unwrap()) }
fn contains(b: [I;3],p: [f64;3]) {
    for i in 0..3 { assert!(b[i].contains(p[i]),"coordinate {i}: {b:?} excludes {p:?}"); }
}

#[test]
fn interval_rotations_include_axis_normalization_offset_and_both_senses() {
    let e = solved(&format!("{AXES}\nmotion turn(about: axis,ratio: -2,phase: 30deg)\n"));
    let family = motion::Family::read(&e.sketch,0).unwrap();
    let domain = I::new(-0.3,0.4).unwrap();
    let bounds = family.bounds(domain).unwrap();
    let p = [3.,4.,5.];
    for i in 0..=20 {
        let t = -0.3+0.7*i as f64/20.;
        let angle = 30f64.to_radians()-2.*t;
        let (s,c) = angle.sin_cos();
        // Axis is world z through (2,0,0); independent coordinate formula.
        contains(bounds.point(box_at(p)).unwrap(),[2.+c-4.*s,s+4.*c,5.]);
        contains(bounds.inverse_point(box_at(p)).unwrap(),[2.+c+4.*s,-s+4.*c,5.]);
    }
    let fixed = family.bounds(I::point(0.).unwrap()).unwrap();
    for q in [fixed.point(box_at(p)).unwrap(),fixed.inverse_point(box_at(p)).unwrap()] {
        assert!(q.iter().all(|v| v.bounds()[1]-v.bounds()[0] < 1e-10));
    }
}

#[test]
fn relative_motion_bounds_cover_input_boxes_and_nested_observer_inverses() {
    let e = solved(&format!("{AXES}\n\
        motion turn(about: axis,ratio: 2,phase: 30deg)\n\
        motion observer(about: other,ratio: -0.3,phase: 20deg)\n\
        motion relative(turn,relative_to: observer)\n\
        motion nested(relative,relative_to: turn)\n"));
    for name in ["relative","nested"] {
        let family = motion::Family::read(&e.sketch,e.map.ent_named(name).unwrap().i()).unwrap();
        let b = family.bounds(I::new(-0.1,0.2).unwrap()).unwrap();
        let p = [I::new(2.,3.).unwrap(),I::new(-1.,1.).unwrap(),I::new(4.,5.).unwrap()];
        let forward = b.point(p).unwrap(); let inverse = b.inverse_point(p).unwrap();
        for t in [-0.1,0.,0.1,0.2] {
            let pose = family.at(t).unwrap();
            for x in [2.,3.] { for y in [-1.,1.] { for z in [4.,5.] {
                contains(forward,pose.point([x,y,z]));
                contains(inverse,pose.inverse().point([x,y,z]));
            } } }
        }
    }
}

#[test]
fn motion_bounds_retain_solved_snapshots_and_enclose_angles_of_any_size() {
    let mut e = solved(&format!("{AXES}\nmotion turn(about: axis,ratio: 2)\n"));
    let family = motion::Family::read(&e.sketch,0).unwrap();
    // Many turns wide, the bounds hold every pose in them, the identity among them; a single
    // angle past sin_cos's own domain reads the same pose as that angle a turn nearer zero.
    let q = [3.,1.,4.];
    let wide = family.bounds(I::new(-5.,5.).unwrap()).unwrap().point(box_at(q)).unwrap();
    assert!((0..3).all(|k| wide[k].contains(q[k])),"{wide:?}");
    let far = family.bounds(I::point(9.).unwrap()).unwrap().point(box_at(q)).unwrap();
    let near = family.bounds(I::point(9.-std::f64::consts::PI).unwrap()).unwrap().point(box_at(q)).unwrap();
    for k in 0..3 {
        let (a,b) = (far[k].bounds(),near[k].bounds());
        assert!(a[0] <= b[1]+1e-12 && b[0] <= a[1]+1e-12 && a[1]-a[0] < 1e-9,"{k}: {a:?} {b:?}");
    }
    assert_eq!(family.bounds(I::point(f64::MAX).unwrap()).unwrap_err(),Error::Overflow);
    let p = box_at([3.,1.,4.]);
    let original = family.bounds(I::point(0.3).unwrap()).unwrap().point(p).unwrap();
    let a = e.map.ent_named("a").unwrap().i();
    let x = e.sketch.points[a].x as usize;
    e.sketch.params[x].value = -2.;
    assert_eq!(family.bounds(I::point(0.3).unwrap()).unwrap().point(p).unwrap(),original);
    let updated = motion::Family::read(&e.sketch,0).unwrap();
    assert_ne!(updated.bounds(I::point(0.3).unwrap()).unwrap().point(p).unwrap(),original);
}
