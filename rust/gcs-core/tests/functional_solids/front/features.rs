//! Local field-only feature probes. These are candidate measurements, not a
//! completeness or nonsmoothness proof, and do not yet guide the front.
use super::*;
use gcs_core::linalg::{Mat,min_norm_solve};

#[derive(Debug)]
struct Feature {p:V,rank:usize,normals:Vec<V>,inside:V,outside:V}

impl Surface {
    fn differential(&mut self,p:V,h:f64) -> Option<V> {
        let g = self.gradient_with_step(p,h);
        (length(g) > 1e-10).then(|| unit(g))
    }

    fn branch_normal(&mut self,p:V,radius:f64) -> Option<V> {
        let h = radius*0.01;
        let n = self.differential(p,h)?;
        // Mixed finite differences at a crease must not become an extra face.
        // Keep only normals that are locally stable under small perturbations.
        for k in 0..3 { for sign in [-1.,1.] {
            let mut q = p; q[k] += sign*radius*0.05;
            if length(sub(self.differential(q,h)?,n)) > 0.08 { return None; }
        }}
        Some(n)
    }

    fn local_feature(&mut self,guess:V,mut radius:f64) -> Option<Feature> {
        assert!(radius.is_finite() && radius > 0.);
        let (mut center,_,_,_) = self.correct(guess)?;
        let mut previous:Option<Feature> = None;
        for level in 0..4 {
            if radius < self.point_tolerance*8. { return None; }
            let mut samples = vec![];
            // Fixed directions sample space, not known model planes or edges.
            for x in -1_i32..=1 { for y in -1_i32..=1 { for z in -1_i32..=1 {
                if x.abs()+y.abs()+z.abs() != 1 && x.abs()+y.abs()+z.abs() != 3 { continue; }
                let direction = unit([x as f64,y as f64,z as f64]);
                let Some((p,_,_,_)) = self.correct(add(center,mul(direction,radius))) else { continue; };
                if let Some(n) = self.branch_normal(p,radius) { samples.push((p,n)); }
            }}}
            let mut normals:Vec<V> = vec![];
            for &(_,n) in &samples {
                if normals.iter().all(|&m| length(sub(m,n)) > 0.12) { normals.push(n); }
            }
            if normals.len() < 2 { return None; }
            // Work relative to the neighborhood center. The minimum-norm
            // solution anchors the free direction of a rank-two intersection.
            let matrix = Mat::from_vec(samples.len(),3,samples.iter().flat_map(|(_,n)| *n).collect());
            let rhs:Vec<_> = samples.iter().map(|&(p,n)| dot(sub(p,center),n)).collect();
            let (delta,rank) = min_norm_solve(&matrix,&rhs,0.08);
            if rank < 2 { return None; }
            let delta = [delta[0],delta[1],delta[2]];
            if length(delta) > radius { return None; }
            let proposed = add(center,delta);
            let (p,_,inside,outside) = self.correct(proposed)?;
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
