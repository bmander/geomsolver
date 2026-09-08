//! Small geometry fixtures, accuracy checks and acceptance cases.
use super::*;

fn sphere() -> MaterialField {
    SpatialField::from(RevolvedField::new(F::disk([0.;2],1.).unwrap(),[0.;3],[0.,0.,1.]).unwrap()).into()
}

fn cube() -> MaterialField {
    cube_in_basis([[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]])
}

fn cube_in_basis(basis:[V;3]) -> MaterialField {
    // The radius-two ball only supplies finite construction support; all six
    // half-spaces determine the cube. The marcher never sees the planes.
    let mut field = MaterialField::from(SpatialField::from(RevolvedField::new(
        F::disk([0.;2],2.).unwrap(),[0.;3],[0.,0.,1.]).unwrap()));
    for direction in basis { for sign in [-1.,1.] {
        let axis = mul(direction,sign);
        let half = RevolvedField::new(F::half_plane([0.,1.],[0.,1.]).unwrap(),[0.;3],axis).unwrap();
        field = field.intersection(SpatialField::from(half).into()).unwrap();
    }}
    field
}

fn check_front(name:&str,field:MaterialField,genus:usize,distance:impl Fn(V)->f64) {
    let start = std::time::Instant::now();
    let mut surface = Surface::new(field,0.02);
    let mut front = Front::start(&mut surface); front.grow(&mut surface);
    let elapsed = start.elapsed();
    eprintln!("generic front {name}: extraction {elapsed:?}");
    eprintln!("generic front {name}: {} triangles, {} boundary edges, {} queries",front.triangles.len(),front.boundary.len(),surface.queries);
    eprintln!("generic front {name}: {} seed box queries",surface.box_queries);
    eprintln!("generic front {name}: {} local retriangulations",front.repairs);
    assert!(front.boundary.is_empty(),"front stalled on {name}");
    let shell = gcs_core::topology::ClosedShell::from_triangles(front.vertices.len(),&front.triangles).unwrap();
    assert_eq!(shell.genus(),genus);
    for v in &front.vertices {
        assert!(v.radius <= surface.point_tolerance);
        assert!(surface.value(v.inside).bounds()[1] < 0. && surface.value(v.outside).bounds()[0] > 0.);
    }
    let mut deviation = 0_f64;
    let mut min_angle = std::f64::consts::PI;
    for t in &front.triangles {
        let [a,b,c] = t.map(|i| front.vertices[i].p);
        for [p,q,r] in [[a,b,c],[b,c,a],[c,a,b]] {
            let u = sub(q,p); let v = sub(r,p);
            min_angle = min_angle.min(length(cross(u,v)).atan2(dot(u,v)));
        }
        for p in [mul(add(a,b),0.5),mul(add(b,c),0.5),mul(add(c,a),0.5),mul(add(add(a,b),c),1./3.)] {
            deviation = deviation.max(distance(p));
        }
    }
    eprintln!("generic front {name}: sampled triangle deviation {deviation}, minimum angle {} degrees",min_angle.to_degrees());
    assert!(min_angle.to_degrees() > 6.,"candidate contains a sliver triangle");
    assert!(deviation <= surface.accuracy,"candidate has excessive sampled deviation");
    // Timing gates are opt-in and run serially, outside the parallel core suite.
    // An open mesh has already failed above, regardless of elapsed time.
    if matches!(name,"sphere"|"cube") {
        if let Ok(limit) = std::env::var("SOLVENT_FRONT_MAX_MS") {
            let limit:f64 = limit.parse().expect("SOLVENT_FRONT_MAX_MS must be a positive number");
            assert!(limit.is_finite() && limit > 0.);
            assert!(elapsed.as_secs_f64()*1000. < limit,"{name} exceeds {limit} ms extraction target");
        }
    }
    if let Some(output) = std::env::var_os("SOLVENT_FRONT_OUTPUT") {
        let output = std::path::PathBuf::from(output); std::fs::create_dir_all(&output).unwrap();
        let pieces:Vec<_> = front.triangles.iter().map(|t| gcs_core::csg::Piece {
            pts:t.iter().map(|&i| front.vertices[i].p).collect(),n:[0.;3],path:name.into(),prim:0,smooth:false,
        }).collect();
        let bytes = gcs_core::mesh::checked_stl(&pieces,"general advancing-front candidate").unwrap();
        std::fs::write(output.join(format!("{name}.stl")),bytes).unwrap();
    }
}

#[test]
fn generic_front_sphere() {
    check_front("sphere",sphere(),0,|p| (length(p)-1.).abs());
}

#[test]
fn generic_front_torus() {
    let torus = SpatialField::from(RevolvedField::new(F::disk([2.,0.],0.6).unwrap(),[0.;3],[0.,0.,1.]).unwrap()).into();
    check_front("torus",torus,1,|p| ((p[0].hypot(p[1])-2.).hypot(p[2])-0.6).abs());
}

#[test]
#[ignore = "Unmet acceptance case: generic sharp-edge discovery and front closure are missing"]
fn generic_front_cube() {
    check_front("cube",cube(),0,|p| {
        let q = p.map(|x| x.abs()-1.);
        length(q.map(|x| x.max(0.)))+q.into_iter().fold(f64::NEG_INFINITY,f64::max).min(0.).abs()
    });
}

#[test]
#[should_panic(expected="seed discovery exhausted without a retained point")]
fn generic_front_does_not_seed_a_zero_only_boolean_surface() {
    let sphere = MaterialField::from(SpatialField::from(RevolvedField::new(F::disk([0.;2],1.).unwrap(),[0.;3],[0.,0.,1.]).unwrap()));
    Surface::new(sphere.clone().difference(sphere).unwrap(),0.02).seed();
}

#[test]
fn generic_front_cache_reuses_work_without_changing_the_mesh() {
    let mut cached = Surface::new(sphere(),0.02);
    let mut uncached = Surface::new(sphere(),0.02); uncached.cache = false;
    let mut a = Front::start(&mut cached); a.grow(&mut cached);
    let mut b = Front::start(&mut uncached); b.grow(&mut uncached);
    assert!(a.boundary.is_empty() && b.boundary.is_empty());
    assert_eq!(a.triangles,b.triangles);
    assert_eq!(a.vertices,b.vertices);
    assert!(cached.queries*2 < uncached.queries,"repeated projection work was not reused");
    eprintln!("generic front sphere: cache {} vs {} point queries",cached.queries,uncached.queries);
}

fn verify_feature(surface:&mut Surface,guess:V,radius:f64,rank:usize,expected:V) -> features::Feature {
    let start = std::time::Instant::now();
    let feature = surface.local_feature(guess,radius).expect("feature was not discovered");
    eprintln!("local feature rank {rank}: {:?}, {} queries, {:?}",start.elapsed(),surface.queries,feature.p);
    assert_eq!(feature.rank,rank);
    assert!(length(sub(feature.p,expected)) < surface.point_tolerance*3.,"{feature:?}");
    assert!(surface.value(feature.inside).bounds()[1] < 0.);
    assert!(surface.value(feature.outside).bounds()[0] > 0.);
    assert!(boundary_radius(feature.p,feature.inside,feature.outside) <= surface.point_tolerance);
    feature
}

#[test]
fn generic_front_local_probe_recovers_cube_edges_and_corners() {
    for accuracy in [0.02,0.001] { for (p,rank) in [([1.,1.,0.2],2),([1.,1.,1.],3)] {
        let mut surface = Surface::new(cube(),accuracy);
        let guess = sub(p,[0.015;3]);
        // An edge's free coordinate stays anchored to the query neighborhood.
        let expected = if rank == 2 { [p[0],p[1],guess[2]] } else { p };
        verify_feature(&mut surface,guess,0.2,rank,expected);
    }}
}

#[test]
fn generic_front_local_probe_does_not_require_axis_aligned_features() {
    let axis = unit([1.,2.,3.]); let (s,c) = 0.47_f64.sin_cos();
    let rotate = |p| add(add(mul(p,c),mul(cross(axis,p),s)),mul(axis,dot(axis,p)*(1.-c)));
    let basis = [[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]].map(rotate);
    for (p,rank) in [([1.,1.,0.2],2),([1.,1.,1.],3)] {
        let mut surface = Surface::new(cube_in_basis(basis),0.001);
        let guess = sub(p,[0.015;3]);
        let expected = if rank == 2 { [p[0],p[1],guess[2]] } else { p };
        verify_feature(&mut surface,rotate(guess),0.2,rank,rotate(expected));
    }
}

#[test]
fn generic_front_local_probe_does_not_call_smooth_curvature_a_crease() {
    for radius in [1.,0.05,0.005] {
        let field = SpatialField::from(RevolvedField::new(F::disk([0.;2],radius).unwrap(),[0.;3],[0.,0.,1.]).unwrap()).into();
        let mut surface = Surface::new(field,radius*0.001);
        for n in [[0.,0.,1.],unit([1.,2.,3.])] {
            assert!(surface.local_feature(mul(n,radius),radius*0.4).is_none());
        }
    }
    let mut surface = Surface::new(cube(),0.001);
    assert!(surface.local_feature([0.,0.,1.],0.2).is_none());
    let empty = cube();
    let mut surface = Surface::new(empty.clone().difference(empty).unwrap(),0.001);
    assert!(surface.local_feature([1.,1.,1.],0.2).is_none());
}

#[test]
fn generic_front_local_probe_recovers_a_curved_boolean_crease() {
    let ball = |x| MaterialField::from(SpatialField::from(RevolvedField::new(
        F::disk([0.;2],1.).unwrap(),[x,0.,0.],[0.,0.,1.]).unwrap()));
    let field = ball(-0.5).intersection(ball(0.5)).unwrap();
    let p = [0.,0.75_f64.sqrt(),0.];
    let mut surface = Surface::new(field,0.001);
    verify_feature(&mut surface,p,0.2,2,p);
}

#[test]
fn generic_front_crosses_discovered_creases_without_averaging_their_faces() {
    for rotation in [0_f64,0.47] { for angle in [90_f64,160.] {
        let axis = unit([1.,2.,3.]); let (s,c) = rotation.sin_cos();
        let rotate = |p| add(add(mul(p,c),mul(cross(axis,p),s)),mul(axis,dot(axis,p)*(1.-c)));
        let (s,c) = angle.to_radians().sin_cos();
        let outward = [rotate([1.,0.,0.]),rotate([c,s,0.])];
        let mut field = MaterialField::from(SpatialField::from(RevolvedField::new(
            F::disk([0.;2],2.).unwrap(),[0.;3],[0.,0.,1.]).unwrap()));
        for n in outward {
            let half = SpatialField::from(RevolvedField::new(F::half_plane([0.;2],[0.,1.]).unwrap(),[0.;3],n).unwrap());
            field = field.intersection(half.into()).unwrap();
        }
        let mut surface = Surface::new(field,0.001);
        // Only the incoming patch is supplied by this local fixture. Its
        // feature points and incident normals are recovered from the field.
        let a = surface.feature_vertex(rotate([0.,0.,-0.1]),0.2).expect("first crease point");
        let b = surface.feature_vertex(rotate([0.,0.,0.1]),0.2).expect("second crease point");
        let c = surface.project(rotate([0.,-0.2,0.])).unwrap();
        let mut front = Front::from_seed(&mut surface,[a,b,c]);
        let start = std::time::Instant::now();
        assert!(front.advance(&mut surface),"crease crossing stalled at {angle} degrees");
        eprintln!("crease crossing {angle}, rotation {rotation}: {:?}",start.elapsed());
        let t = *front.triangles.last().unwrap();
        assert!(t.contains(&0) && t.contains(&1),"growth skipped the discovered crease");
        let [p,q,r] = t.map(|i| front.vertices[i].p);
        let n = unit(cross(sub(q,p),sub(r,p)));
        assert!(dot(n,outward[1]) > 0.9999,"triangle did not follow the outgoing face");
        let new = front.vertices.last().unwrap().p;
        assert!(dot(new,outward[0]) < -0.01,"front did not cross onto the other face");
        assert!(dot(new,outward[1]).abs() < surface.point_tolerance);
        assert_eq!(front.triangles.len(),2);
        assert_eq!(front.boundary.len(),4);
        assert!(!front.boundary.contains(&[0,1]) && !front.boundary.contains(&[1,0]));
    }}
}

fn spiky_tetrahedron() -> (MaterialField,[[V;3];4]) {
    spiky_tetrahedron_at([0.;3])
}

fn spiky_tetrahedron_at(shift:V) -> (MaterialField,[[V;3];4]) {
    spiky_tetrahedron_in_frame(shift,1.,0.,0.06)
}

fn rotate_tetrahedron(p:V,angle:f64) -> V {
    let axis = unit([1.,2.,3.]); let (s,c) = angle.sin_cos();
    add(add(mul(p,c),mul(cross(axis,p),s)),mul(axis,dot(axis,p)*(1.-c)))
}

fn spiky_tetrahedron_in_frame(shift:V,scale:f64,angle:f64,radius:f64) -> (MaterialField,[[V;3];4]) {
    // Default height 2, base circumradius 0.06: a sharper apex than a regular
    // tetrahedron. Face data constructs the field and independent test oracle;
    // it is never passed to the marcher or feature probe.
    let points = [[0.,0.,1.5],[radius,0.,-0.5],[-radius*0.5,radius*0.5*3_f64.sqrt(),-0.5],
        [-radius*0.5,-radius*0.5*3_f64.sqrt(),-0.5]]
        .map(|p| add(mul(rotate_tetrahedron(p,angle),scale),shift));
    assert!(points.iter().all(|&p| length(p) < 2.*scale),"fixture support clips the tetrahedron");
    let center = mul(points.into_iter().fold([0.;3],add),0.25);
    let faces = [[0,1,2],[0,2,3],[0,3,1],[1,3,2]].map(|t| t.map(|i| points[i]));
    let mut field = MaterialField::from(SpatialField::from(RevolvedField::new(
        F::disk([0.;2],2.*scale).unwrap(),[0.;3],[0.,0.,1.]).unwrap()));
    for [a,b,c] in faces {
        let mut n = unit(cross(sub(b,a),sub(c,a)));
        if dot(n,sub(center,a)) > 0. { n = mul(n,-1.); }
        let half = SpatialField::from(RevolvedField::new(
            F::half_plane([0.;2],[0.,1.]).unwrap(),a,n).unwrap());
        field = field.intersection(half.into()).unwrap();
    }
    (field,faces)
}

fn triangle_distance(p:V,[a,b,c]:[V;3]) -> f64 {
    let n = unit(cross(sub(b,a),sub(c,a)));
    let height = dot(sub(p,a),n); let q = sub(p,mul(n,height));
    let edges = [(a,b),(b,c),(c,a)];
    if edges.iter().all(|&(a,b)| dot(cross(sub(b,a),sub(q,a)),n) >= 0.) { return height.abs(); }
    edges.into_iter().map(|(a,b)| {
        let e = sub(b,a); let t = (dot(sub(p,a),e)/dot(e,e)).clamp(0.,1.);
        length(sub(p,add(a,mul(e,t))))
    }).fold(f64::INFINITY,f64::min)
}

#[test]
#[ignore = "Unmet acceptance case: front stalls across the thin tetrahedron's faces"]
fn generic_front_spiky_tetrahedron() {
    let (field,faces) = spiky_tetrahedron();
    check_front("spiky_tetrahedron",field,0,|p| faces.map(|t| triangle_distance(p,t))
        .into_iter().fold(f64::INFINITY,f64::min));
}

#[test]
fn generic_front_local_probe_recovers_a_spiky_tetrahedron_apex() {
    let translated = [0.31,-0.27,0.12];
    for (scale,angle,offset) in [(1.,0.,[0.;3]),(0.1,0.,translated),
        (1.,0.47,translated),(10.,1.1,translated)] { for radius in [0.06,0.006] {
        let shift = mul(offset,scale);
        let (field,faces) = spiky_tetrahedron_in_frame(shift,scale,angle,radius);
        let position = |z| add(mul(rotate_tetrahedron([0.,0.,z],angle),scale),shift);
        let mut surface = Surface::new(field,scale*0.001);
        let feature = verify_feature(&mut surface,position(1.49),scale*0.2,3,position(1.5));
        assert_eq!(feature.normals.len(),3);
        for [a,b,c] in faces.into_iter().take(3) {
            let n = unit(cross(sub(b,a),sub(c,a)));
            assert!(feature.normals.iter().any(|&m| dot(m,n) > 1.-1e-8));
        }
    }}
}

#[test]
fn generic_front_finds_thin_material_without_a_supplied_interior_point() {
    for shift in [[0.;3],[0.31,-0.27,0.12]] {
        let (field,_) = spiky_tetrahedron_at(shift);
        let mut surface = Surface::new(field,0.001);
        let start = std::time::Instant::now();
        let p = surface.interior_seed().expect("thin material was missed");
        eprintln!("thin seed {shift:?}: {:?}, {} point and {} box queries",start.elapsed(),surface.queries,surface.box_queries);
        assert!(surface.value(p).bounds()[1] < 0.);
        assert!(surface.box_queries > 0,"fixture did not exercise bounded seed search");
    }
}
