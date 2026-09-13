//! Negative controls for the distinction between field values and boundary evidence.
use gcs_core::solid::{MaterialField,PlanarField,RevolvedField,SpatialField};
use gcs_core::solid::swept_boundary::{FieldJudge,Projection,Sign,certify};

fn ball(center: [f64;3],radius: f64) -> MaterialField {
    SpatialField::from(RevolvedField::new(PlanarField::disk([0.;2],radius).unwrap(),center,[0.,0.,1.]).unwrap()).into()
}
fn judge(field: MaterialField) -> FieldJudge { FieldJudge::new(field,0.0025,4000,1000,4096) }

#[test]
fn a_small_union_value_is_not_boundary_proximity() {
    let field = ball([-0.999,0.,0.],1.).union(ball([0.999,0.,0.],1.)).unwrap();
    let mut j = judge(field);
    assert_eq!(j.sign([0.;3]).unwrap().0,Sign::Material);
    // The nearest exposed boundary is the intersection circle, radius sqrt(1-c²).
    assert!((1.-0.999_f64.powi(2)).sqrt() > 0.04);
    assert_eq!(j.project([0.;3],[0.,1.,0.],0.005,0.02).unwrap(),Projection::Inner);
}

#[test]
fn a_minus_a_has_no_boundary_witness() {
    let a = ball([0.;3],1.);
    let mut j = judge(a.clone().difference(a).unwrap());
    assert!(!matches!(j.project([1.,0.,0.],[0.,1.,0.],0.005,0.02).unwrap(),
        Projection::Kept {..} | Projection::Moved {..}));
}

#[test]
fn buried_triangles_and_slivers_are_not_complete() {
    for vertices in [vec![[-0.2,-0.2,0.],[0.2,-0.2,0.],[0.,0.2,0.]],
        vec![[-0.2,0.,0.],[0.2,0.,0.],[0.,0.0001,0.]]] {
        let c = certify(&mut judge(ball([0.;3],1.)),&vertices,&[[0,1,2]],0.04,0.01).unwrap();
        assert!(!c.is_complete(),"buried geometry was accepted: {c:?}");
    }
}

#[test]
fn a_zero_set_sliver_is_not_a_material_boundary() {
    let a = ball([0.;3],1.);
    let vertices = [[1.,-0.001,0.],[1.,0.001,0.],[1.,0.,0.000001]];
    let c = certify(&mut judge(a.clone().difference(a).unwrap()),&vertices,&[[0,1,2]],0.04,0.01).unwrap();
    assert!(!c.is_complete(),"a residual-only sliver was accepted: {c:?}");
}

pub(super) fn slab(half_height: f64) -> MaterialField {
    let mut square = PlanarField::disk([0.;2],2.).unwrap();
    for (p,n) in [([1.,0.],[1.,0.]),([-1.,0.],[-1.,0.]),([0.,1.],[0.,1.]),([0.,-1.],[0.,-1.])] {
        square = square.intersection(PlanarField::half_plane(p,n).unwrap()).unwrap();
    }
    SpatialField::from(gcs_core::solid::ExtrudedField::new(square,[0.;3],[1.,0.,0.],[0.,1.,0.],[-half_height,half_height]).unwrap()).into()
}

#[test]
fn thin_material_requires_real_witnesses_and_a_sufficient_probe_budget() {
    let vertices = [[-0.1,-0.1,0.001],[0.1,-0.1,0.001],[0.,0.1,0.001]];
    let field = slab(0.001);
    let coarse = certify(&mut judge(field.clone()),&vertices,&[[0,1,2]],0.04,0.01).unwrap();
    assert!(!coarse.is_complete());
    assert_eq!(coarse.unresolved.len(),1);
    let fine = certify(&mut judge(field),&vertices,&[[0,1,2]],0.04,0.0001).unwrap();
    assert!(fine.is_complete(),"{fine:?}");
    assert_eq!(fine.halved,1);
    let w = fine.brackets[0].1;
    assert!(w.inside().0[2].abs() < 0.001 && w.outside().0[2] > 0.001);
    assert!(w.inside().1[1] < 0. && w.outside().1[0] > 0.);
}

#[test]
fn an_ambiguous_midpoint_keeps_the_spatial_bracket() {
    let field = slab(1.);
    // The boundary is at x=1; widening from x=0.97 places a bisection
    // midpoint on it. Interval normalization leaves that value ambiguous.
    let mut j = judge(field);
    assert!(matches!(j.sign([1.,0.,0.]).unwrap().0,Sign::Near {..}));
    let p = [0.97,0.,0.];
    let projection = j.project(p,[7.,0.,0.],0.005,0.1).unwrap();
    let Projection::Moved {point,radius,bracket,..} = projection else { panic!("{projection:?}"); };
    assert!(radius <= 0.005 && (point[0]-1.).abs() <= radius);
    assert!(bracket.inside().0[0] < 1. && bracket.outside().0[0] > 1.);
    assert_eq!(radius,bracket.radius_from(point).unwrap());
}

#[test]
fn unresolved_bisection_returns_its_original_witnesses() {
    // Along y=z=0: f=x+1 for x<-1, zero for -1<=x<=1, x-1 for x>1.
    let x_plus_one = PlanarField::half_plane([-1.,0.],[1.,0.]).unwrap();
    let y = PlanarField::half_plane([0.,0.],[0.,1.]).unwrap();
    let x_minus_one = PlanarField::half_plane([1.,0.],[1.,0.]).unwrap();
    let profile = x_plus_one.union(y).unwrap().intersection(x_minus_one).unwrap();
    let field = SpatialField::from(gcs_core::solid::ExtrudedField::new(profile,[0.;3],[1.,0.,0.],[0.,1.,0.],[-3.,3.]).unwrap()).into();
    let p = judge(field).project([0.;3],[1.,0.,0.],0.5,2.).unwrap();
    let Projection::Unresolved {bracket:Some(w),radius,..} = p else { panic!("{p:?}"); };
    assert!(radius > 0.5);
    assert!(w.inside().0[0] < -1. && w.outside().0[0] > 1.);
}

pub(super) fn cube() -> gcs_core::solid::swept_boundary::KeptMesh {
    let vertices: Vec<_> = (0..8).map(|i| std::array::from_fn(|k| if i&(1<<k) == 0 {-1.} else {1.})).collect();
    let mut triangles = Vec::new();
    for axis in 0..3 { for side in 0..2 {
        let a = (axis+1)%3; let b = (axis+2)%3;
        let base = if side == 0 { 0 } else { 1<<axis };
        let ids = [base,base|(1<<a),base|(1<<a)|(1<<b),base|(1<<b)];
        for mut t in [[ids[0],ids[1],ids[2]],[ids[0],ids[2],ids[3]]] {
            if side == 0 { t.swap(1,2); }
            triangles.push(t);
        }
    } }
    let sheet = vec![0;triangles.len()];
    gcs_core::solid::swept_boundary::KeptMesh {vertices,triangles,sheet}
}
fn audit_options() -> gcs_core::solid::swept_boundary::AuditOptions {
    gcs_core::solid::swept_boundary::AuditOptions {tolerance:0.2,max_cells:100000,max_depth:24,box_budget:64}
}

#[test]
fn an_accepted_cube_has_whole_surface_and_reverse_coverage_evidence() {
    use gcs_core::solid::swept_boundary::{SweptBoundaryOptions,validate};
    let accepted = validate(slab(1.),cube(),&SweptBoundaryOptions::default(),audit_options()).unwrap();
    assert!(accepted.certificate().is_complete());
    assert_eq!(accepted.shell().genus(),0);
    let proof = accepted.spatial();
    assert!(proof.distance_bound() <= 0.2);
    for w in proof.surface() {
        assert!(w.bracket.inside().1[1] < 0. && w.bracket.outside().1[0] > 0.);
        assert!(w.distance <= 0.2);
    }
    assert!(proof.coverage().iter().any(|c| c.mesh_witness.is_none()));
    assert!(proof.coverage().iter().any(|c| c.mesh_witness.is_some()));
    for c in proof.coverage() { assert_eq!(c.value.is_none(),c.mesh_witness.is_some());
        if let Some(value) = c.value { assert!(!value.contains(0.)); } }
    // The retained terminal boxes cover the whole support, not a caller crop.
    let volume = |b: [gcs_core::interval::Interval;3]| b.iter().map(|v| {let [a,b]=v.bounds(); b-a}).product::<f64>();
    let sum: f64 = proof.coverage().iter().map(|c| volume(c.bounds)).sum();
    assert!((sum-volume(proof.domain())).abs() < 1e-9);
}

#[test]
fn a_passing_centroid_cannot_hide_an_unsupported_triangle_interior() {
    use gcs_core::solid::swept_boundary::{SweptBoundaryOptions,ConstructError,AuditError,validate};
    let field = slab(1.).difference(ball([0.5,0.5,1.],0.3)).unwrap();
    let mesh = cube();
    let samples = certify(&mut judge(field.clone()),&mesh.vertices,&mesh.triangles,0.04,0.01).unwrap();
    assert!(samples.is_complete(),"the negative control must pass the old centroid gate");
    let result = validate(field,mesh,&SweptBoundaryOptions::default(),audit_options());
    assert!(matches!(result,Err(ConstructError::Spatial(AuditError::Triangle {..}))),"{result:?}");
}

#[test]
fn a_hidden_component_is_a_reverse_coverage_refusal() {
    use gcs_core::solid::swept_boundary::{SweptBoundaryOptions,ConstructError,AuditError,validate};
    let field = slab(1.).union(ball([4.,0.,0.],0.3)).unwrap();
    let result = validate(field,cube(),&SweptBoundaryOptions::default(),audit_options());
    assert!(matches!(result,Err(ConstructError::Spatial(AuditError::Coverage {..}))),"{result:?}");
}

#[test]
fn topology_and_spatial_budgets_are_acceptance_gates() {
    use gcs_core::solid::swept_boundary::{SweptBoundaryOptions,ConstructError,AuditError,validate};
    let mut open = cube(); open.triangles.pop(); open.sheet.pop();
    assert!(matches!(validate(slab(1.),open,&SweptBoundaryOptions::default(),audit_options()),Err(ConstructError::Topology(_))));
    let mut options = audit_options(); options.max_cells = 1;
    assert!(matches!(validate(slab(1.),cube(),&SweptBoundaryOptions::default(),options),Err(ConstructError::Spatial(AuditError::Budget {..}))));
}

#[test]
fn invalid_mesh_indices_and_nonfinite_points_are_refused_before_compaction() {
    use gcs_core::solid::swept_boundary::{SweptBoundaryOptions,ConstructError,validate};
    let mut bad = cube(); bad.triangles[0][0] = u32::MAX;
    assert!(matches!(validate(slab(1.),bad,&SweptBoundaryOptions::default(),audit_options()),Err(ConstructError::InvalidMesh)));
    let mut bad = cube(); bad.vertices[0][0] = f64::NAN;
    assert!(matches!(validate(slab(1.),bad,&SweptBoundaryOptions::default(),audit_options()),Err(ConstructError::InvalidMesh)));
}
