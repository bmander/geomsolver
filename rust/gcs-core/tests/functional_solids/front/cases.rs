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
    let mut front = Front::start(&mut surface);
    snapshot::grow(name,&mut front,&mut surface,start);
    let elapsed = start.elapsed();
    eprintln!("generic front {name}: extraction {elapsed:?}");
    eprintln!("generic front {name}: {} triangles, {} boundary edges, {} queries",front.triangles.len(),front.boundary.len(),surface.queries);
    eprintln!("generic front {name}: {} seed box queries",surface.box_queries);
    eprintln!("generic front {name}: {} local retriangulations",front.repairs);
    if std::env::var_os("SOLVENT_FRONT_DIAG").is_some() {
        eprintln!("front mesh: {{\"vertices\":{:?},\"triangles\":{:?}}}",
            front.vertices.iter().map(|v| v.p).collect::<Vec<_>>(),front.triangles);
        for &e in &front.boundary {
            let Some((v,h)) = front.candidate(&mut surface,e) else { eprintln!("stalled edge {e:?}: no candidate"); continue; };
            let t = [e[1],e[0],front.vertices.len()];
            let refusal = front.refusal(t,Some(&v));
            let fits = surface.fits([front.vertices[e[1]].p,front.vertices[e[0]].p,v.p]);
            eprintln!("stalled edge {e:?}: {refusal:?}, fits {fits}, h {h}, features {:?}->{}, points {:?} -> {:?}",
                e.map(|i| front.vertices[i].branches.len()),v.branches.len(),e.map(|i| front.vertices[i].p),v.p);
        }
    }
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
    assert!(front.triangles.iter().all(|t| quality::acceptable(t.map(|i| &front.vertices[i]))),
        "candidate contains a sliver without an intrinsic corner");
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

fn wedge(outward:[V;2]) -> MaterialField {
    let mut field = MaterialField::from(SpatialField::from(RevolvedField::new(
        F::disk([0.;2],2.).unwrap(),[0.;3],[0.,0.,1.]).unwrap()));
    for n in outward {
        let half = SpatialField::from(RevolvedField::new(
            F::half_plane([0.;2],[0.,1.]).unwrap(),[0.;3],n).unwrap());
        field = field.intersection(half.into()).unwrap();
    }
    field
}

#[test]
fn generic_front_crosses_discovered_creases_without_averaging_their_faces() {
    for rotation in [0_f64,0.47] { for angle in [90_f64,160.] {
        let axis = unit([1.,2.,3.]); let (s,c) = rotation.sin_cos();
        let rotate = |p| add(add(mul(p,c),mul(cross(axis,p),s)),mul(axis,dot(axis,p)*(1.-c)));
        let (s,c) = angle.to_radians().sin_cos();
        let outward = [rotate([1.,0.,0.]),rotate([c,s,0.])];
        let field = wedge(outward);
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

fn rotate_fixture(p:V,angle:f64) -> V {
    let axis = unit([1.,2.,3.]); let (s,c) = angle.sin_cos();
    add(add(mul(p,c),mul(cross(axis,p),s)),mul(axis,dot(axis,p)*(1.-c)))
}

fn spiky_tetrahedron_in_frame(shift:V,scale:f64,angle:f64,radius:f64) -> (MaterialField,[[V;3];4]) {
    // Default height 2, base circumradius 0.06: a sharper apex than a regular
    // tetrahedron. Face data constructs the field and independent test oracle;
    // it is never passed to the marcher or feature probe.
    let points = [[0.,0.,1.5],[radius,0.,-0.5],[-radius*0.5,radius*0.5*3_f64.sqrt(),-0.5],
        [-radius*0.5,-radius*0.5*3_f64.sqrt(),-0.5]]
        .map(|p| add(mul(rotate_fixture(p,angle),scale),shift));
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
        let position = |z| add(mul(rotate_fixture([0.,0.,z],angle),scale),shift);
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

#[test]
fn generic_front_discovers_a_crease_from_an_incoming_patch() {
    for rotation in [0.,0.47] { for angle in [90_f64,160.] {
        let rotate = |p| rotate_fixture(p,rotation);
        let (s,c) = angle.to_radians().sin_cos();
        let outward = [rotate([1.,0.,0.]),rotate([c,s,0.])];
        let field = wedge(outward);
        let mut surface = Surface::new(field,0.001);
        let initial = [[0.,-0.05,-0.1],[0.,-0.05,0.1],[0.,-0.2,0.]]
            .map(|p| surface.project(rotate(p)).unwrap());
        assert!(initial.iter().all(|v| v.branches.is_empty()));
        let mut front = Front::from_seed(&mut surface,initial);
        for _ in 0..40 { if !front.advance(&mut surface) { break; } }
        eprintln!("automatic crease {angle}, rotation {rotation}: {} triangles, {} features, {} queries",front.triangles.len(),
            front.vertices.iter().filter(|v| !v.branches.is_empty()).count(),surface.queries);
        assert!(front.vertices.iter().any(|v| v.branches.len() == 2),"crease was not discovered");
        assert!(front.triangles.iter().any(|t| {
            let [a,b,c] = t.map(|i| front.vertices[i].p);
            dot(unit(cross(sub(b,a),sub(c,a))),outward[1]) > 0.9999
        }),"front did not reach the outgoing face");
    }}
}

#[test]
fn generic_front_feature_cache_preserves_successes_failures_and_radius() {
    let field = wedge([[1.,0.,0.],[0.,1.,0.]]);
    let mut cached = Surface::new(field.clone(),0.001);
    let mut uncached = Surface::new(field,0.001); uncached.cache = false;
    for guess in [[0.,0.,0.],[0.,-0.5,0.]] { for radius in [0.2,0.1] {
        let before = cached.queries;
        let a = cached.feature_vertex(guess,radius);
        assert!(cached.queries > before,"distinct radius did not evaluate a new neighborhood");
        let before = cached.queries;
        assert_eq!(cached.feature_vertex(guess,radius),a);
        assert_eq!(cached.queries,before,"identical feature probe repeated field work");
        assert_eq!(uncached.feature_vertex(guess,radius),a);
        assert_eq!(a.is_some(),guess[1] == 0.);
    }}
}

#[test]
fn generic_front_retains_an_intrinsically_acute_corner() {
    for (scale,rotation) in [(0.1,0.),(1.,0.47),(10.,1.1)] { for radius in [0.06,0.006] {
        let (field,faces) = spiky_tetrahedron_in_frame(mul([0.31,-0.27,0.12],scale),scale,rotation,radius);
        let [a,b,c] = faces[0];
        let b = mul(add(a,b),0.5); let c = mul(add(a,c),0.5);
        let u = sub(b,a); let v = sub(c,a);
        let angle = length(cross(u,v)).atan2(dot(u,v)).to_degrees();
        assert!(angle > 0.29 && angle < 3.1);
        // A conforming triangulation cannot increase the sum of angles at this
        // corner. Subdividing it only creates still smaller corner angles.
        let mut surface = Surface::new(field,scale*radius/60.);
        let edge_radius = radius*scale*0.2;
        let vertices = [surface.feature_vertex(a,scale*0.2).unwrap(),
            surface.feature_vertex(b,edge_radius).unwrap(),surface.feature_vertex(c,edge_radius).unwrap()];
        assert_eq!(vertices[0].branches.len(),3);
        assert!(vertices[1..].iter().all(|v| v.branches.len() == 2));
        let front = Front::from_seed(&mut surface,vertices.clone());
        assert_eq!(front.triangles.len(),1);
        assert!(!front.legal([0,2,1],None),"reversed acute triangle was accepted");
        for k in 0..3 {
            let mut unsupported = vertices.clone(); unsupported[k].branches.clear();
            assert!(!quality::acceptable(unsupported.each_ref()),"missing incident branches were ignored");
        }
        // The same true corner must not excuse an avoidable second small
        // angle caused by advancing unequal distances along its two creases.
        let shorter = surface.feature_vertex(mul(add(a,c),0.5),edge_radius).unwrap();
        assert!(surface.fits([a,b,shorter.p]));
        assert!(!quality::acceptable([&vertices[0],&vertices[1],&shorter]));
    }}
}

#[test]
fn generic_front_local_probe_isolates_a_nearby_crease_from_a_distant_corner() {
    let (field,faces) = spiky_tetrahedron();
    let [a,b,_] = faces[0]; let direction = sub(b,a);
    // A wide probe sees all three sides of this thin body. Their common apex
    // lies far outside the neighborhood; the nearer two-face crease is local.
    let guess = [0.078,-0.078,0.108];
    let expected = add(a,mul(direction,dot(sub(guess,a),direction)/dot(direction,direction)));
    let mut surface = Surface::new(field,0.02);
    verify_feature(&mut surface,guess,0.131,2,expected);
}

#[test]
fn generic_front_clearance_requires_edge_triangle_separation() {
    let triangle = [[0.,0.,0.],[1.,0.,0.],[0.,1.,0.]];
    assert!(clearance::separated([[0.2,0.2,0.01],[0.4,0.2,0.01]],triangle));
    assert!(clearance::separated([[2.,2.,-1.],[2.,2.,1.]],triangle));
    assert!(!clearance::separated([[0.2,0.2,-1.],[0.2,0.2,1.]],triangle));
    assert!(!clearance::separated([[0.2,0.2,0.],[0.4,0.2,0.]],triangle));
    assert!(!clearance::separated([[0.,0.,-1.],[0.,0.,1.]],triangle));
    assert!(!clearance::separated([[0.2,0.2,0.],[0.2,0.2,1.]],triangle));
    // A crossing of the triangle's plane can lie beyond the segment.
    assert!(clearance::separated([[0.2,0.2,1.],[0.2,0.2,2.]],triangle));
}

#[test]
fn generic_front_clearance_matches_exact_integer_plane_crossings() {
    let triangle = [[0.,0.,0.],[1.,0.,0.],[0.,1.,0.]];
    // Every edge crosses z=0 halfway along its length. Twice the intersection
    // coordinates are integers, giving an exact independent inclusion test.
    for px in -2..=2 { for py in -2..=2 { for qx in -2..=2 { for qy in -2..=2 {
        let x = px+qx; let y = py+qy;
        let intersects = x >= 0 && y >= 0 && x+y <= 2;
        let edge = [[px as f64,py as f64,-1.],[qx as f64,qy as f64,1.]];
        assert_eq!(clearance::separated(edge,triangle),!intersects,"{edge:?}");
    }}}}
    assert!(!clearance::separated([[f64::NAN,0.,0.],[0.,0.,1.]],triangle));
}

#[test]
fn generic_front_clearance_distinguishes_close_faces_from_overlapping_fronts() {
    for rotation in [0.,0.47] {
        let rotate = |p| rotate_fixture(p,rotation);
        let mut field = MaterialField::from(SpatialField::from(RevolvedField::new(
            F::disk([0.;2],2.).unwrap(),[0.;3],[0.,0.,1.]).unwrap()));
        for sign in [-1.,1.] {
            let half = SpatialField::from(RevolvedField::new(F::half_plane([0.;2],[0.,1.]).unwrap(),
                rotate([sign*0.02,0.,0.]),rotate([sign,0.,0.])).unwrap());
            field = field.intersection(half.into()).unwrap();
        }
        let mut surface = Surface::new(field,0.02);
        let initial = [[0.02,-0.1,-0.1],[0.02,-0.1,0.1],[0.02,-0.3,0.]]
            .map(|p| surface.project(rotate(p)).unwrap());
        let mut front = Front::from_seed(&mut surface,initial);
        let candidate = surface.project(rotate([0.02,0.1,0.])).unwrap();
        assert!(front.clearance > 0.04,"fixture does not enter the original thick prism");
        for (x,separate) in [(-0.02,true),(0.02,false)] {
            let mut patch = [[x,-0.04,-0.02],[x,0.04,-0.02],[x,0.,0.04]];
            if x < 0. { patch.swap(1,2); }
            let patch = patch.map(|p| surface.project(rotate(p)).unwrap());
            assert!(surface.fits(patch.each_ref().map(|v| v.p)));
            let first = front.vertices.len(); front.vertices.extend(patch);
            // Represent a second front arriving via the other side of the
            // solid. No normals or feature tags are supplied by this fixture.
            front.insert([first,first+1,first+2]);
            let t = [1,0,front.vertices.len()];
            assert_eq!(front.legal(t,Some(&candidate)),separate);
            if !separate { assert!(matches!(front.refusal(t,Some(&candidate)),Some(mesh::Refusal::Clearance(_)))); }
        }
    }
}

#[test]
fn generic_front_clearance_allows_only_declared_shared_simplex_contacts() {
    let triangle = [[0.,0.,0.],[1.,0.,0.],[0.,1.,0.]];
    let edge = [[0.,0.,0.],[0.,0.,1.]];
    assert!(!clearance::separated(edge,triangle));
    assert!(clearance::separated_except_shared(edge,triangle,[true,false]));
    assert!(!clearance::separated_except_shared([[0.2,0.2,0.],[0.2,0.2,1.]],triangle,[true,false]));
    assert!(!clearance::separated_except_shared([[0.,0.,0.],[0.2,0.2,0.]],triangle,[true,false]));
    assert!(clearance::separated_except_shared([[0.,0.,0.],[1.,0.,0.]],triangle,[true;2]));
}

#[test]
fn generic_front_single_crease_endpoint_keeps_prediction_in_the_shared_face() {
    for rotation in [0.,0.47] {
        let rotate = |p| rotate_fixture(p,rotation);
        let outward = [rotate([1.,0.,0.]),rotate([0.,1.,0.])];
        let mut surface = Surface::new(wedge(outward),0.001);
        let a = surface.feature_vertex(rotate([0.,0.,-0.1]),0.2).unwrap();
        let b = surface.project(rotate([0.,-0.05,0.1])).unwrap();
        let c = surface.project(rotate([0.,-0.2,0.])).unwrap();
        assert_eq!(a.branches.len(),2);
        assert!(b.branches.is_empty() && c.branches.is_empty());
        let front = Front::from_seed(&mut surface,[a,b,c]);
        for edge in [[0,1],[2,0]] {
            assert!(dot(front.growth_normal(edge),outward[0]) > 1.-1e-8,
                "prediction averaged a different incident face into this patch");
        }
    }
}

#[test]
fn generic_front_triangle_interior_must_face_out_of_material() {
    for rotation in [0.,0.47] {
        let (field,faces) = spiky_tetrahedron_in_frame([0.;3],1.,rotation,0.06);
        let [a,b,c] = faces[0];
        // The vertices lie on two creases of one face. Opposite winding can
        // otherwise borrow a different incident face's normal at each vertex.
        let p = mul(add(a,b),0.5); let q = add(mul(a,0.25),mul(c,0.75));
        let r = mul(add(a,c),0.5);
        let mut surface = Surface::new(field,0.001);
        assert!(surface.fits([p,q,r]));
        assert!(!surface.fits([p,r,q]),"inward face passed the field-fit check");
    }
}

#[test]
fn generic_front_must_not_span_a_retained_crease_sample_through_a_tiny_gap() {
    let mut surface = Surface::new(wedge([[1.,0.,0.],[0.,1.,0.]]),0.001);
    let a = surface.feature_vertex([0.,0.,-0.2],0.2).unwrap();
    let b = surface.feature_vertex([0.,0.,0.2],0.2).unwrap();
    let c = surface.project([0.,-0.2,0.]).unwrap();
    let mut front = Front::from_seed(&mut surface,[a,b,c]);
    let mut middle = surface.feature_vertex([0.,0.,0.],0.2).unwrap();
    // Model two independent discoveries that differ by much less than their
    // boundary-witness radii. Exact coordinate inequality is not clearance.
    middle.p[0] += 1e-12;
    let index = front.vertices.len(); front.vertices.push(middle);
    let other = surface.project([-0.2,0.,0.]).unwrap();
    assert_eq!(front.refusal([1,0,front.vertices.len()],Some(&other)),Some(mesh::Refusal::NearVertex(index)));
    let middle = &front.vertices[index];
    assert!(middle.near_edge_interior(&front.vertices[0],&front.vertices[1]));
    let mut distant = middle.clone(); distant.p[0] += surface.point_tolerance*10.;
    assert!(!distant.near_edge_interior(&front.vertices[0],&front.vertices[1]));
}
