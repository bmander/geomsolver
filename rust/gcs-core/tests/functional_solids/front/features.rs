//! Local field-only feature probes. These are candidate measurements, not a
//! completeness or nonsmoothness proof. Local crossing tests pass their normal
//! branches to the front; automatic feature acquisition during growth remains.
use super::*;
use gcs_core::linalg::{Mat,min_norm_solve};

#[derive(Debug)]
struct Feature {p:V,rank:usize,normals:Vec<V>,inside:V,outside:V}

impl Surface {
    fn local_projection(&mut self,p:V) -> Option<(V,V,V,V)> {
        self.correct_with_step(p,self.point_tolerance*0.1)
    }

    fn feature_vertex(&mut self,guess:V,radius:f64) -> Option<Vertex> {
        let feature = self.local_feature(guess,radius)?;
        let n = unit(feature.normals.iter().copied().fold([0.;3],add));
        let mut vertex = self.vertex(feature.p,n,feature.inside,feature.outside);
        vertex.branches = feature.normals;
        Some(vertex)
    }

    fn differential(&mut self,p:V,h:f64) -> Option<V> {
        let (g,error) = self.gradient_measurement(p,h);
        (length(g) > 1e-10 && error < length(g)*0.01).then(|| unit(g))
    }

    fn branch_normal(&mut self,p:V,mut radius:f64) -> Option<(V,f64)> {
        // Mixed finite differences at a crease must not become an extra face.
        // Keep only normals that are locally stable under small perturbations.
        for _ in 0..8 {
            let h = radius*0.01;
            if h < self.point_tolerance*0.01 { break; }
            let (g,error) = self.gradient_measurement(p,h); let magnitude = length(g);
            if magnitude < 1e-10 || error > magnitude*0.01 { return None; }
            let n = mul(g,1./magnitude);
            let mut stable = true;
            for k in 0..3 { for sign in [-1.,1.] {
                let mut q = p; q[k] += sign*radius*0.05;
                if length(sub(self.differential(q,h)?,n)) > 0.08 { stable = false; break; }
            }}
            if stable { return Some((n,magnitude)); }
            radius *= 0.25;
        }
        None
    }

    fn local_feature(&mut self,guess:V,mut radius:f64) -> Option<Feature> {
        assert!(radius.is_finite() && radius > 0.);
        let mut center = self.local_projection(guess).map_or(guess,|(p,_,_,_)| p);
        let mut previous:Option<Feature> = None;
        for level in 0..4 {
            if radius < self.point_tolerance*8. { return None; }
            let mut samples = vec![];
            // Fixed directions sample space, not known model planes or edges.
            for x in -1_i32..=1 { for y in -1_i32..=1 { for z in -1_i32..=1 {
                if x.abs()+y.abs()+z.abs() != 1 && x.abs()+y.abs()+z.abs() != 3 { continue; }
                let direction = unit([x as f64,y as f64,z as f64]);
                let Some((p,_,_,_)) = self.local_projection(add(center,mul(direction,radius))) else { continue; };
                if let Some((n,magnitude)) = self.branch_normal(p,radius) {
                    // A bracketed point still has a small field residual.
                    // Remove its first-order plane offset before a poorly
                    // conditioned sharp-tip fit amplifies that residual.
                    let [lo,hi] = self.query(p,true).bounds();
                    let corrected = sub(p,mul(n,(lo*0.5+hi*0.5)/magnitude));
                    samples.push((corrected,n));
                }
            }}}
            let mut groups:Vec<(V,f64)> = vec![];
            for &(p,n) in &samples {
                let offset = dot(sub(p,guess),n);
                if let Some((sum,rhs)) = groups.iter_mut().find(|(m,_)| length(sub(unit(*m),n)) < 0.12) {
                    *sum = add(*sum,n); *rhs += offset;
                } else { groups.push((n,offset)); }
            }
            let normals:Vec<_> = groups.iter().map(|&(n,_)| unit(n)).collect();
            if normals.len() < 2 { return None; }
            // Work relative to the neighborhood center. The minimum-norm
            // solution anchors the free direction of a rank-two intersection.
            // Fit one equation per normal group. Curvature within a sampled
            // branch must not force an artificially large rank cutoff that
            // also discards the axial direction of a thin genuine corner.
            let matrix = Mat::from_vec(normals.len(),3,normals.iter().flatten().copied().collect());
            let rhs:Vec<_> = groups.iter().map(|&(n,rhs)| rhs/length(n)).collect();
            let (delta,rank) = min_norm_solve(&matrix,&rhs,1e-10);
            if rank < 2 { return None; }
            let delta = [delta[0],delta[1],delta[2]];
            let proposed = add(guess,delta);
            if length(sub(proposed,center)) > radius { return None; }
            // A central-difference normal at a sharp tip need not point into
            // its material cone. Try a direction positive against every
            // recovered branch, and accept it only with strict field signs.
            let (direction,_) = min_norm_solve(&matrix,&vec![1.;normals.len()],1e-10);
            let direction = [direction[0],direction[1],direction[2]];
            let crossing = if length(direction) > 1e-10 {
                let offset = mul(unit(direction),self.point_tolerance*0.5);
                let inside = sub(proposed,offset); let outside = add(proposed,offset);
                (self.value(inside).bounds()[1] < 0. && self.value(outside).bounds()[0] > 0.)
                    .then_some((proposed,inside,outside))
            } else { None };
            let (p,inside,outside) = crossing.or_else(|| self.local_projection(proposed).map(|(p,_,a,b)| (p,a,b)))?;
            if length(sub(p,proposed)) > radius*0.1 { return None; }
            let candidate = Feature {p,rank,normals,inside,outside};
            if level == 3 {
                let previous = previous?;
                let matches = |a:&[V],b:&[V]| a.iter().all(|&n| b.iter().any(|&m| length(sub(n,m)) < 0.12));
                return (previous.rank == rank && matches(&previous.normals,&candidate.normals) &&
                    matches(&candidate.normals,&previous.normals)).then_some(candidate);
            }
            center = p; previous = Some(candidate); radius *= 0.25;
        }
        unreachable!()
    }
}

fn verify_feature(surface:&mut Surface,guess:V,radius:f64,rank:usize,expected:V) {
    let start = std::time::Instant::now();
    let feature = surface.local_feature(guess,radius).expect("feature was not discovered");
    eprintln!("local feature rank {rank}: {:?}, {} queries, {:?}",start.elapsed(),surface.queries,feature.p);
    assert_eq!(feature.rank,rank);
    assert!(length(sub(feature.p,expected)) < surface.point_tolerance*3.,"{feature:?}");
    assert!(surface.value(feature.inside).bounds()[1] < 0.);
    assert!(surface.value(feature.outside).bounds()[0] > 0.);
    assert!(boundary_radius(feature.p,feature.inside,feature.outside) <= surface.point_tolerance);
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
    // Height 2, base circumradius 0.06: a much sharper apex than a regular
    // tetrahedron. Face data constructs the field and independent test oracle;
    // it is never passed to the marcher or feature probe.
    let points = [[0.,0.,1.5],[0.06,0.,-0.5],[-0.03,0.03*3_f64.sqrt(),-0.5],
        [-0.03,-0.03*3_f64.sqrt(),-0.5]].map(|p| add(p,shift));
    let center = mul(points.into_iter().fold([0.;3],add),0.25);
    let faces = [[0,1,2],[0,2,3],[0,3,1],[1,3,2]].map(|t| t.map(|i| points[i]));
    let mut field = MaterialField::from(SpatialField::from(RevolvedField::new(
        F::disk([0.;2],2.).unwrap(),[0.;3],[0.,0.,1.]).unwrap()));
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
#[ignore = "Unmet acceptance case: local projection does not recover all incident apex branches"]
fn generic_front_local_probe_recovers_a_spiky_tetrahedron_apex() {
    let (field,_) = spiky_tetrahedron();
    let mut surface = Surface::new(field,0.001);
    verify_feature(&mut surface,[0.,0.,1.49],0.2,3,[0.,0.,1.5]);
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
