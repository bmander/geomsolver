use super::evidence::{cube,slab};
use gcs_core::solid::swept_boundary::{inspect,AuditOptions,AuditError,AuditIssue,AuditReport,BoundaryReport,Check,ConstructError,SweptBoundaryOptions};

fn options() -> AuditOptions { AuditOptions {tolerance:0.2,max_cells:10000,max_depth:24,box_budget:64} }
fn spatial(r: &BoundaryReport) -> &AuditReport {
    let Check::Attempted(a) = r.spatial() else { panic!("spatial check was suppressed"); }; a
}
fn partition(a: &AuditReport,triangles: usize) {
    // Every binary split halves parameter area, regardless of which edge is cut.
    for triangle in 0..triangles {
        let sum: f64 = a.surface().iter().filter(|w| w.triangle == triangle).map(|w| 2_f64.powi(-(w.path.len() as i32)))
            .chain(a.surface_unfinished().iter().filter(|w| w.triangle == triangle).map(|w| 2_f64.powi(-(w.path.len() as i32)))).sum();
        assert!((sum-1.).abs() < 1e-12,"triangle {triangle} partition {sum}");
    }
    let volume = |b: [gcs_core::interval::Interval;3]| b.iter().map(|v| {let [a,b]=v.bounds(); b-a}).product::<f64>();
    let sum: f64 = a.coverage().iter().map(|w| volume(w.bounds)).chain(a.coverage_unfinished().iter().map(|w| volume(w.bounds))).sum();
    assert!((sum-volume(a.domain().unwrap())).abs() < 1e-9);
}

#[test]
fn constrained_audit_keeps_witnesses_and_all_remaining_regions() {
    let mut o = options(); o.max_cells = 700;
    let r = inspect(slab(1.),cube(),&SweptBoundaryOptions::default(),o);
    let a = spatial(&r);
    assert!(!a.is_complete());
    assert!(!a.surface().is_empty() && !a.surface_unfinished().is_empty());
    assert!(!a.coverage().is_empty() && !a.coverage_unfinished().is_empty());
    assert_eq!(a.visited().iter().sum::<usize>(),o.max_cells);
    assert!(a.surface_unfinished().iter().any(|w| matches!(w.issue,AuditIssue::Unresolved(AuditError::Budget {..}))));
    partition(a,12);
    assert!(matches!(r.into_accepted(),Err(ConstructError::Spatial(_))));
}

#[test]
fn open_topology_does_not_hide_spatial_evidence() {
    let mut mesh = cube(); mesh.triangles.pop(); mesh.sheet.pop();
    let r = inspect(slab(1.),mesh,&SweptBoundaryOptions::default(),options());
    assert!(matches!(r.topology(),Check::Attempted(Err(_))));
    assert!(matches!(r.certificate(),Check::Attempted(Ok(_))));
    let a = spatial(&r);
    assert!(a.visited().iter().all(|&n| n > 0));
    partition(a,11);
    assert!(matches!(r.into_accepted(),Err(ConstructError::Topology(_))));
}

#[test]
fn reverse_coverage_uses_mesh_even_when_no_forward_bracket_succeeds() {
    let mut mesh = cube(); for t in &mut mesh.triangles { t.swap(0,1); }
    let r = inspect(slab(1.),mesh,&SweptBoundaryOptions::default(),options());
    let a = spatial(&r);
    assert!(a.surface().is_empty());
    assert!(a.coverage().iter().any(|w| w.mesh_witness.is_some()));
    partition(a,12);
    assert!(matches!(r.into_accepted(),Err(ConstructError::Certificate {..})));
}

#[test]
fn a_strict_exclusion_is_a_contradiction_but_depth_exhaustion_is_not() {
    let mut mesh = cube(); for p in &mut mesh.vertices { *p = p.map(|x| x*0.25); }
    let r = inspect(slab(1.),mesh,&SweptBoundaryOptions::default(),options());
    let a = spatial(&r);
    assert!(a.surface_unfinished().iter().any(|w| matches!(w.issue,AuditIssue::OffSurface {value,..} if !value.contains(0.))));
    partition(a,12);
    let mut o = options(); o.max_depth = 0;
    let r = inspect(slab(1.),cube(),&SweptBoundaryOptions::default(),o);
    let a = spatial(&r);
    assert!(a.surface_unfinished().iter().all(|w| matches!(w.issue,AuditIssue::Unresolved(AuditError::Triangle {..}))));
    partition(a,12);
    assert!(r.into_accepted().is_err());
}

#[test]
fn invalid_inputs_and_options_explicitly_mark_unattempted_checks() {
    let mut mesh = cube(); mesh.triangles[0][0] = u32::MAX;
    let r = inspect(slab(1.),mesh,&SweptBoundaryOptions::default(),options());
    assert!(matches!(r.topology(),Check::NotAttempted(ConstructError::InvalidMesh)));
    assert!(matches!(r.certificate(),Check::NotAttempted(ConstructError::InvalidMesh)));
    assert!(matches!(r.spatial(),Check::NotAttempted(ConstructError::InvalidMesh)));
    assert!(matches!(r.into_accepted(),Err(ConstructError::InvalidMesh)));
    let mut o = options(); o.tolerance = f64::NAN;
    let r = inspect(slab(1.),cube(),&SweptBoundaryOptions::default(),o);
    assert!(matches!(r.topology(),Check::Attempted(Ok(_))));
    assert_eq!(spatial(&r).not_attempted(),Some(&AuditError::InvalidOptions));
    assert!(!spatial(&r).is_complete());
    assert!(matches!(r.into_accepted(),Err(ConstructError::Spatial(AuditError::InvalidOptions))));
}

#[test]
fn missing_support_blocks_only_reverse_coverage() {
    use gcs_core::solid::{PlanarField,ExtrudedField,SpatialField};
    let field: SpatialField = ExtrudedField::new(PlanarField::half_plane([1.,0.],[1.,0.]).unwrap(),
        [0.;3],[1.,0.,0.],[0.,1.,0.],[-1.,1.]).unwrap().into();
    let r = inspect(field.into(),cube(),&SweptBoundaryOptions::default(),options());
    let a = spatial(&r);
    assert!(a.not_attempted().is_none());
    assert_eq!(a.coverage_not_attempted(),Some(&AuditError::NoFiniteSupport));
    assert!(a.visited()[0] > 0 && a.visited()[1] == 0);
    assert!(!a.is_complete());
    assert!(r.into_accepted().is_err());
}

#[test]
fn invalid_centroid_options_do_not_suppress_independent_spatial_work() {
    let options = SweptBoundaryOptions {probe_distance:Some(f64::NAN),..Default::default()};
    let r = inspect(slab(1.),cube(),&options,self::options());
    assert!(matches!(r.certificate(),Check::Attempted(Err(_))));
    assert!(spatial(&r).visited().iter().all(|&n| n > 0));
    assert!(r.into_accepted().is_err());
}
