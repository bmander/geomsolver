use gcs_core::{interval::{Error,Interval as I},solid::{PlanarField as F,RevolvedField,SpatialField}};

mod sweeps;
mod material;
mod probe;
mod boundary;
mod front;
mod document;

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
    let (source,errors) = syntax::parse("a := point\nb := point\n\
        fix(x == 1, y == 0) a\nfix(x == 1, y == 1) b\naxis := line(a,b)\nturn := motion(about: axis)\n");
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
    assert_eq!(sphere.transformed(&family,f64::MAX).unwrap_err(),Error::Overflow);
}

#[test]
fn axial_field_query_cost() {
    let field = RevolvedField::new(F::half_plane([0.,0.],[0.,1.]).unwrap(),
        [1.,2.,-1.],[1.,2.,2.]).unwrap();
    let count = if std::env::var_os("SOLVENT_FIELD_BENCH").is_some() { 100_000 } else { 256 };
    let start = std::time::Instant::now();
    let mut sum = 0.;
    for i in 0..count {
        let p = std::hint::black_box(point([i as f64/64.,0.5,-0.25]));
        sum += field.bounds(p).unwrap().bounds()[0];
    }
    eprintln!("axial field: {count} queries, {:?}, checksum {sum}",start.elapsed());
    let n = count as f64; let expected = n*(n-1.)/384.-n*5./6.;
    assert!((sum-expected).abs() < expected.abs().max(1.)*1e-12);
}

#[test]
fn exact_coordinate_dependencies_skip_only_irrelevant_arithmetic() {
    let wide = I::new(-1e200,1e200).unwrap();
    for (through,normal,p) in [([f64::MAX,2.],[-0.,3.],[wide,I::new(3.,4.).unwrap()]),
        ([2.,f64::MAX],[3.,0.],[I::new(3.,4.).unwrap(),wide])] {
        let result = F::half_plane(through,normal).unwrap().bounds(p).unwrap();
        assert!(result.contains(1.) && result.contains(2.));
        assert!(result.bounds()[0] > 0.99 && result.bounds()[1] < 2.01);
    }
    let axial = F::half_plane([f64::MAX,2.],[0.,3.]).unwrap();
    let revolution = RevolvedField::new(axial,[0.,0.,1.],[0.,0.,2.]).unwrap();
    let result = revolution.bounds([wide,wide,I::new(4.,5.).unwrap()]).unwrap();
    assert!(result.contains(1.) && result.contains(2.));
    assert!(result.bounds()[0] > 0.99 && result.bounds()[1] < 2.01);
    assert!(revolution.support_bounds().unwrap().is_none());

    // Exact zero is special; a tiny nonzero radial coefficient cannot be
    // discarded. Its planar value is significant at a sufficiently large x.
    let almost_axial = F::half_plane([0.;2],[1e-100,1.]).unwrap();
    let result = almost_axial.bounds(point([1e100,0.])).unwrap();
    assert!(result.contains(1.) && result.bounds()[0] > 0.99);
    let revolution = RevolvedField::new(almost_axial,[0.;3],[0.,0.,1.]).unwrap();
    assert_eq!(revolution.bounds([wide,wide,I::ZERO]).unwrap_err(),Error::Overflow);
    assert!(F::half_plane([f64::NAN,0.],[0.,1.]).is_err());
}

#[test]
fn axial_dependency_propagates_through_booleans_without_dropping_radial_operands() {
    let up = F::half_plane([0.,1.],[0.,1.]).unwrap();
    let down = F::half_plane([0.,-1.],[0.,-1.]).unwrap();
    let wide = I::new(-1e200,1e200).unwrap();
    for (profile,expected) in [(up.clone().union(down.clone()).unwrap(),-1.5),
        (up.clone().intersection(down.clone()).unwrap(),-0.5),
        (up.clone().difference(down.clone()).unwrap(),1.5)] {
        let field = RevolvedField::new(profile,[0.;3],[0.,0.,1.]).unwrap();
        let result = field.bounds([wide,wide,I::point(0.5).unwrap()]).unwrap();
        assert!(result.contains(expected));
        assert!(result.bounds()[1]-result.bounds()[0] < 1e-12);
    }
    let radial = F::half_plane([1.,0.],[1.,0.]).unwrap();
    for profile in [up.clone().union(radial.clone()).unwrap(),
        up.clone().intersection(radial.clone()).unwrap(),up.difference(radial).unwrap()] {
        let field = RevolvedField::new(profile,[0.;3],[0.,0.,1.]).unwrap();
        assert_eq!(field.bounds([wide,wide,I::ZERO]).unwrap_err(),Error::Overflow);
    }
}

#[test]
fn rotated_axial_field_encloses_affine_box_extrema() {
    // The exact normalization of (1,2,2) is division by 3. The extrema of
    // this affine field on a box occur at the corners, independently of the
    // revolution implementation and of the location of its unused radius.
    let field = RevolvedField::new(F::half_plane([9.,0.5],[0.,7.]).unwrap(),
        [1.,2.,-1.],[1.,2.,2.]).unwrap();
    let box_ = [I::new(-2.,-1.).unwrap(),I::new(1.,4.).unwrap(),I::new(-0.5,0.5).unwrap()];
    let result = field.bounds(box_).unwrap();
    let mut lo = f64::INFINITY; let mut hi = f64::NEG_INFINITY;
    for x in box_[0].bounds() { for y in box_[1].bounds() { for z in box_[2].bounds() {
        let expected = ((x-1.)+2.*(y-2.)+2.*(z+1.))/3.-0.5;
        assert!(result.contains(expected)); lo = lo.min(expected); hi = hi.max(expected);
        let single = field.bounds(point([x,y,z])).unwrap();
        assert!(single.contains(expected));
        assert!(single.bounds()[1]-single.bounds()[0] < 1e-12);
    }}}
    assert!(result.bounds()[1]-result.bounds()[0] < hi-lo+1e-12);
}

#[test]
fn repeated_stock_subtraction_has_no_discarded_boundary_zeros() {
    let sphere = |origin| SpatialField::from(RevolvedField::new(
        F::disk([0.;2],2.).unwrap(),origin,[0.,0.,1.]).unwrap());
    let a = sphere([0.;3]); let b = sphere([3.,0.,0.]);
    let lens = a.clone().intersection(b.clone()).unwrap();
    let double_cut = a.clone().difference(a.clone().difference(b).unwrap()).unwrap();
    // This is on A's boundary, well outside the lens. A literal nested min/max
    // expression yields zero here even though there is no material nearby.
    assert!(double_cut.bounds(point([0.,0.,-2.])).unwrap().bounds()[0] > 1.6);
    for x in -4..=10 { for z in -6..=6 {
        let p = point([x as f64/2.,0.,z as f64/2.]);
        assert_eq!(double_cut.bounds(p).unwrap(),lens.bounds(p).unwrap());
    } }
    // Similar but distinct operands must not trigger the identity.
    let smaller = SpatialField::from(RevolvedField::new(
        F::disk([0.;2],1.).unwrap(),[0.;3],[0.,0.,1.]).unwrap());
    let other = a.difference(smaller.difference(sphere([3.,0.,0.])).unwrap()).unwrap();
    assert!(other.bounds(point([-1.5,0.,0.])).unwrap().bounds()[1] < 0.);
}
