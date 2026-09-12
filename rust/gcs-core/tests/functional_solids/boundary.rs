use super::*;
use super::sweeps::{rotation,options};
use gcs_core::{interval::minimum::Options,solid::{MaterialField,SweptField,BoundaryOptions,BoundaryError,FieldBoundary}};

fn ball(center: [f64;3],radius: f64) -> MaterialField {
    SpatialField::from(RevolvedField::new(F::disk([0.;2],radius).unwrap(),center,[0.,0.,1.]).unwrap()).into()
}
fn settings(tolerance: f64) -> BoundaryOptions {
    BoundaryOptions {spatial_tolerance:tolerance,max_depth:12,max_cells:200000,sweep:options(),domain:None}
}
fn inside(b: [I;3],p: [f64;3]) { assert!((0..3).all(|k| b[k].contains(p[k])),"{b:?} excludes {p:?}"); }

#[test]
fn construction_supports_cover_disconnected_material_and_all_sweep_poses() {
    let near = ball([0.;3],1.); let far = ball([20.,0.,0.],0.1);
    let both = near.clone().union(far.clone()).unwrap();
    let b = both.support_bounds().unwrap().unwrap();
    inside(b,[-1.,0.,0.]); inside(b,[20.1,0.,0.]);
    let field = SpatialField::from(RevolvedField::new(F::half_plane([0.;2],[0.,1.]).unwrap(),[0.;3],[0.,0.,1.]).unwrap());
    assert!(field.support_bounds().unwrap().is_none());
    let half = MaterialField::from(field);
    assert!(half.clone().union(near.clone()).unwrap().support_bounds().unwrap().is_none());
    assert_eq!(half.intersection(near.clone()).unwrap().support_bounds().unwrap(),near.support_bounds().unwrap());
    assert_eq!(near.clone().difference(far).unwrap().support_bounds().unwrap(),near.support_bounds().unwrap());
    let placed = both.transformed(&rotation(),0.7).unwrap().support_bounds().unwrap().unwrap();
    inside(placed,[20.1*0.7_f64.cos(),20.1*0.7_f64.sin(),0.]);
    let source = SpatialField::from(RevolvedField::new(F::disk([0.;2],1.).unwrap(),[3.,0.,0.],[0.,0.,1.]).unwrap());
    let sweep = SweptField::new(source,rotation(),I::new(-4.,4.).unwrap());
    let b = sweep.support_bounds().unwrap().unwrap();
    for i in 0..=80 { let t = -4.+i as f64/10.;
        for r in [2.,4.] { inside(b,[r*t.cos(),r*t.sin(),0.]); }
        inside(b,[3.*t.cos(),3.*t.sin(),1.]);
    }
    let mut shared = MaterialField::from(sweep);
    for _ in 1..64 { shared = shared.clone().intersection(shared).unwrap(); }
    assert_eq!(shared.support_bounds().unwrap().unwrap(),b);
}

fn serialize(mesh: &FieldBoundary) -> String {
    let vertices = mesh.vertices(); let triangles = mesh.triangles();
    let domain = mesh.domain().map(I::bounds); let divisions = mesh.divisions();
    let error = mesh.spatial_error_bound();
    let cells: Vec<_> = mesh.cells().iter().map(|c| {
        let start = c.start; let step = c.step; let b = c.bounds.map(I::bounds);
        let p = c.center; let v = c.center_value.bounds(); let radius = c.radius; let value = c.value.bounds();
        let vertex = c.mesh_vertex.map_or("null".into(),|i| i.to_string());
        format!("{{\"start\":{start:?},\"step\":{step},\"bounds\":{b:?},\"center\":{p:?},\"center_value\":{v:?},\"radius\":{radius},\"value\":{value:?},\"mesh_vertex\":{vertex}}}")
    }).collect();
    let crossings: Vec<_> = mesh.crossings().iter().map(|c| {
        let cell = c.cell; let range = [c.triangles.start,c.triangles.end];
        let a = c.inside.point; let av = c.inside.value.bounds();
        let b = c.outside.point; let bv = c.outside.value.bounds();
        format!("{{\"cell\":{cell},\"triangles\":{range:?},\"inside\":{{\"point\":{a:?},\"value\":{av:?}}},\"outside\":{{\"point\":{b:?},\"value\":{bv:?}}}}}")
    }).collect();
    format!("{{\"schema\":1,\"source\":{{\"sphere\":{{\"center\":[0,0,0],\"radius\":1}}}},\"domain\":{domain:?},\"divisions\":{divisions},\"spatial_error\":{error},\"vertices\":{vertices:?},\"triangles\":{triangles:?},\"cells\":[{}],\"crossings\":[{}]}}",cells.join(","),crossings.join(","))
}

#[test]
fn sphere_boundary_has_complete_spatial_evidence_and_a_closed_shell() {
    let mut sphere = ball([0.;3],1.).evaluator(0);
    let mut previous = f64::INFINITY;
    for tolerance in [0.9,0.4] {
        let mesh = sphere.boundary(settings(tolerance)).unwrap();
        assert_eq!(mesh.shell().genus(),0);
        let error = mesh.spatial_error_bound(); assert!(error <= tolerance && error < previous); previous = error;
        assert_eq!(mesh.cells().iter().map(|c| u64::from(c.step).pow(3)).sum::<u64>(),u64::from(mesh.divisions()).pow(3));
        for c in mesh.cells() { assert_eq!(c.value.contains(0.),c.mesh_vertex.is_some()); }
        for c in mesh.crossings() {
            assert!(c.inside.value.bounds()[1] < 0. && c.outside.value.bounds()[0] > 0.);
            assert!(c.inside.point[0].hypot(c.inside.point[1]).hypot(c.inside.point[2]) < 1.);
            assert!(c.outside.point[0].hypot(c.outside.point[1]).hypot(c.outside.point[2]) > 1.);
        }
        // Independent exact distance to the unit sphere at vertices and triangle centers.
        for t in mesh.triangles() {
            let pts = t.map(|i| mesh.vertices()[i]);
            let center: [f64;3] = std::array::from_fn(|k| (pts[0][k]+pts[1][k]+pts[2][k])/3.);
            for p in pts.into_iter().chain([center]) {
                assert!((p[0].hypot(p[1]).hypot(p[2])-1.).abs() <= error);
            }
        }
        if tolerance == 0.9 { if let Some(path) = std::env::var_os("SOLVENT_FIELD_BOUNDARY_OUTPUT") {
            std::fs::write(path,serialize(&mesh)).unwrap();
        } }
    }
}

#[test]
fn extraction_refuses_unknown_support_budgets_and_hidden_disconnected_features() {
    let half = SpatialField::from(RevolvedField::new(F::half_plane([0.;2],[0.,1.]).unwrap(),[0.;3],[0.,0.,1.]).unwrap());
    assert_eq!(MaterialField::from(half).evaluator(0).boundary(settings(1.)).unwrap_err(),BoundaryError::NoFiniteSupport);
    let mut sphere = ball([0.;3],1.).evaluator(0);
    // This grid places vertices within rounding uncertainty of the sphere.
    // Extraction must refuse their unresolved signs, never perturb the field.
    assert!(matches!(sphere.boundary(settings(1.)),Err(BoundaryError::AmbiguousPoint {..})));
    assert_eq!(sphere.boundary(BoundaryOptions {max_cells:1,..settings(1.)}).unwrap_err(),BoundaryError::CellBudget {visited:1});
    assert_eq!(sphere.boundary(BoundaryOptions {max_depth:1,..settings(1.)}).unwrap_err(),BoundaryError::ResolutionLimit);
    assert_eq!(sphere.boundary(settings(f64::NAN)).unwrap_err(),BoundaryError::InvalidOptions);
    assert_eq!(sphere.boundary(BoundaryOptions {sweep:Options {max_evaluations:3,..options()},..settings(1.)}).unwrap_err(),BoundaryError::InvalidOptions);
    // This separate small sphere fits between all fine-grid corner samples.
    // The full support still contains it. The unresolved cell cannot be omitted
    // merely because the large sphere supplied a plausible closed output mesh.
    let hidden = ball([0.;3],1.).union(ball([4.123,0.173,0.217],0.0001)).unwrap();
    assert!(matches!(hidden.evaluator(0).boundary(settings(1.)),Err(BoundaryError::UnresolvedCell {..})));
    let a = ball([0.;3],1.);
    assert!(a.clone().difference(a).unwrap().evaluator(0).boundary(settings(1.)).is_err());
    let separate = ball([0.;3],1.).union(ball([4.,0.,0.],1.)).unwrap();
    let BoundaryError::Topology {error,mut components} = separate.evaluator(0).boundary(settings(0.9)).unwrap_err()
        else { panic!("disconnected balls need component diagnostics") };
    assert_eq!(error,gcs_core::topology::Error::DisconnectedShell);
    assert_eq!(components.len(),2);
    components.sort_by(|a,b| a.bounds[0].bounds()[0].total_cmp(&b.bounds[0].bounds()[0]));
    for (c,center) in components.iter().zip([[0.;3],[4.,0.,0.]]) {
        inside(c.bounds,center);
        assert!((3.0..5.0).contains(&c.signed_volume)); // independent sphere volume is 4*pi/3
        assert!(c.vertices > 0 && c.triangles > 0);
    }
}

#[test]
fn a_continuously_swept_sphere_extracts_a_torus_with_a_checked_hole() {
    let source = SpatialField::from(RevolvedField::new(F::disk([0.;2],1.).unwrap(),[3.,0.,0.],[0.,0.,1.]).unwrap());
    let field = MaterialField::from(SweptField::new(source,rotation(),I::new(-4.,4.).unwrap()));
    let mesh = field.evaluator(10000).boundary(settings(4.)).unwrap();
    assert_eq!(mesh.shell().genus(),1);
    for p in mesh.vertices() {
        assert!(((p[0].hypot(p[1])-3.).hypot(p[2])-1.).abs() <= mesh.spatial_error_bound());
    }
}

#[test]
fn zero_only_geometry_can_use_distant_mesh_evidence_without_becoming_a_shell() {
    // Union with A-A leaves the actual sphere unchanged, but introduces a zero
    // sheet away from it. Possible-boundary cells near that sheet need the
    // wider witness search; there must be no second manufactured surface.
    let ghost = ball([1.4,0.,0.],0.04);
    let field = ball([0.;3],1.).union(ghost.clone().difference(ghost).unwrap()).unwrap();
    let mesh = field.evaluator(0).boundary(settings(0.9)).unwrap();
    assert_eq!(mesh.shell().genus(),0);
    assert!(mesh.cells().iter().any(|c| c.value.contains(0.) && c.bounds[0].bounds()[0] > 1.3));
    assert!(mesh.vertices().iter().all(|p| p[0] < 1.2));
    for p in mesh.vertices() {
        assert!((p[0].hypot(p[1]).hypot(p[2])-1.).abs() <= mesh.spatial_error_bound());
    }
}
