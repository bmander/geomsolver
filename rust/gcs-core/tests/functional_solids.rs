use gcs_core::{interval::{Error,Interval as I},solid::{PlanarField as F,RevolvedField,SpatialField}};

mod sweeps;
mod material;
mod boundary;

fn point<const N: usize>(p: [f64;N]) -> [I;N] { p.map(|v| I::point(v).unwrap()) }
fn close(b: I,value: f64) {
    // Independent closed-form reference values may themselves round by a few ulps.
    assert!(b.bounds()[0] <= value+1e-12 && b.bounds()[1] >= value-1e-12,"{b:?} excludes {value}");
    assert!(b.bounds()[1]-b.bounds()[0] < 1e-10,"loose point enclosure: {b:?}");
}

#[test]
fn functional_revolutions_bound_spheres_tori_and_holes_without_an_axis_wall() {
    let sphere = RevolvedField::new(F::disk([0.,0.],2.).unwrap(),[0.;3],[0.,0.,3.]).unwrap();
    let torus = RevolvedField::new(F::disk([3.,0.],1.).unwrap(),[0.;3],[0.,0.,1.]).unwrap();
    let hole = F::disk([3.,0.],1.).unwrap().difference(F::disk([3.,0.],0.5).unwrap()).unwrap();
    let shell = RevolvedField::new(hole,[0.;3],[0.,0.,-1.]).unwrap();
    for x in 0..=20 { for z in -10..=10 {
        let x = x as f64/4.; let z = z as f64/4.; let p = point([x,0.,z]);
        close(sphere.bounds(p).unwrap(),x.hypot(z)-2.);
        let d = (x-3.).hypot(z);
        close(torus.bounds(p).unwrap(),d-1.);
        close(shell.bounds(p).unwrap(),(d-1.).max(0.5-d));
    } }
    assert!(sphere.bounds(point([0.;3])).unwrap().bounds()[1] < -1.99);
    let whole = sphere.bounds([I::new(-3.,3.).unwrap();3]).unwrap();
    assert!(whole.contains(-2.) && whole.bounds()[1] > 0.);
}

#[test]
fn rounded_corner_boolean_has_the_intended_boundary_and_material_sides() {
    // A square [-2,2]^2 with its northeast corner rounded to radius 1 about (1,1).
    let mut square = F::half_plane([2.,0.],[1.,0.]).unwrap();
    for (p,n) in [([-2.,0.],[-1.,0.]),([0.,2.],[0.,1.]),([0.,-2.],[0.,-1.])] {
        square = square.intersection(F::half_plane(p,n).unwrap()).unwrap();
    }
    // The cut applies in the NE sector, so the material union must include x<=1 or y<=1.
    let corner = F::disk([1.,1.],1.).unwrap()
        .union(F::half_plane([1.,1.],[1.,0.]).unwrap()).unwrap()
        .union(F::half_plane([1.,1.],[0.,1.]).unwrap()).unwrap();
    let rounded = square.intersection(corner).unwrap();
    for x in -20..=20 { for y in -20..=20 {
        let p = [x as f64/8.,y as f64/8.];
        let expected = p[0].abs().max(p[1].abs())-2.;
        let corner = if p[0] > 1. && p[1] > 1. { (p[0]-1.).hypot(p[1]-1.)-1. } else { -1. };
        let inside = expected < -1e-8 && corner < -1e-8;
        let outside = expected > 1e-8 || corner > 1e-8;
        let b = rounded.bounds(point(p)).unwrap();
        if inside { assert!(b.bounds()[1] < 0.,"{p:?}: {b:?}"); }
        if outside { assert!(b.bounds()[0] > 0.,"{p:?}: {b:?}"); }
    } }
}

#[test]
fn field_zero_sets_are_not_automatically_material_boundaries() {
    let a = F::disk([0.;2],1.).unwrap();
    let empty = a.clone().difference(a).unwrap();
    assert!(empty.bounds(point([0.,0.])).unwrap().bounds()[0] > 0.);
    // f=max(d,-d) vanishes on the circle, but {f<0} is empty. An extractor must
    // establish material on one side, not wrap every zero of a Boolean field.
    assert!(empty.bounds(point([1.,0.])).unwrap().contains(0.));
    assert!(empty.bounds(point([2.,0.])).unwrap().bounds()[0] > 0.);
}

#[test]
fn functional_fields_reject_invalid_normalization_and_excessive_depth() {
    assert!(F::half_plane([0.;2],[0.;2]).is_err());
    assert!(F::disk([0.;2],0.).is_err());
    assert!(F::disk([f64::NAN,0.],1.).is_err());
    assert!(RevolvedField::new(F::disk([0.;2],1.).unwrap(),[0.;3],[0.;3]).is_err());
    let mut field = F::disk([0.;2],1.).unwrap();
    for _ in 1..64 { field = field.union(F::disk([0.;2],1.).unwrap()).unwrap(); }
    assert_eq!(field.union(F::disk([0.;2],1.).unwrap()).unwrap_err(),Error::OutsideDomain);
}

#[test]
fn spatial_composition_bounds_lenses_shells_and_shared_empty_results() {
    let sphere = |x| SpatialField::from(RevolvedField::new(F::disk([0.;2],2.).unwrap(),
        [x,0.,0.],[0.,0.,1.]).unwrap());
    let a = sphere(-1.); let b = sphere(1.);
    let union = a.clone().union(b.clone()).unwrap();
    let lens = a.clone().intersection(b.clone()).unwrap();
    let cut = a.clone().difference(b).unwrap();
    let empty = a.clone().difference(a.clone()).unwrap();
    for x in -12..=12 { for z in -8..=8 {
        let x = x as f64/4.; let z = z as f64/4.; let p = point([x,0.,z]);
        let da = (x+1.).hypot(z)-2.; let db = (x-1.).hypot(z)-2.;
        close(union.bounds(p).unwrap(),da.min(db));
        close(lens.bounds(p).unwrap(),da.max(db));
        close(cut.bounds(p).unwrap(),da.max(-db));
        close(empty.bounds(p).unwrap(),da.abs());
    } }
    let box_bounds = lens.bounds([I::new(-0.5,0.5).unwrap();3]).unwrap();
    assert!(box_bounds.bounds()[1] < 0.);
    let mut deep = a.clone();
    // Only 64 unique nodes; naive recursive expansion would visit 2^63 leaves.
    for _ in 1..64 { deep = deep.clone().union(deep).unwrap(); }
    close(deep.bounds(point([0.;3])).unwrap(),-1.);
    assert_eq!(deep.union(a).unwrap_err(),Error::OutsideDomain);
}

#[test]
fn spatial_transforms_use_inverse_fixed_poses_and_refuse_invalid_angles() {
    use gcs_core::{syntax,program,solve,motion::Family};
    let (source,errors) = syntax::parse("point a hint(x: 1,y: 0)\npoint b hint(x: 1,y: 1)\n\
        ground a\nground b\nline axis(a,b)\nmotion turn(about: axis)\n");
    assert!(errors.is_empty());
    let mut model = program::elaborate(&source); assert!(model.ok());
    assert!(solve::solve(&mut model.sketch,Default::default()).success);
    let family = Family::read(&model.sketch,0).unwrap();
    let sphere = SpatialField::from(RevolvedField::new(F::disk([0.;2],1.).unwrap(),
        [-1.,0.,0.],[0.,0.,1.]).unwrap());
    // World-z rotation about (1,0,0) sends the center to (1,-2,0).
    let moved = sphere.clone().transformed(&family,std::f64::consts::FRAC_PI_2).unwrap();
    for x in -4..=8 { for y in -12..=4 {
        let x = x as f64/4.; let y = y as f64/4.;
        close(moved.bounds(point([x,y,0.])).unwrap(),(x-1.).hypot(y+2.)-1.);
    } }
    assert!(moved.bounds(point([1.,-2.,0.])).unwrap().bounds()[1] < -0.99);
    // Both operands share one leaf but query it in different coordinate boxes.
    let both = sphere.clone().union(moved).unwrap();
    assert!(both.bounds(point([1.,-2.,0.])).unwrap().bounds()[1] < -0.99);
    assert!(sphere.clone().transformed(&family,f64::NAN).is_err());
    assert_eq!(sphere.transformed(&family,9.).unwrap_err(),Error::OutsideDomain);
}
