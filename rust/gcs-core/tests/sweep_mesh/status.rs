//! Reproducible candidate reports; unlike acceptance tests this instrument can
//! finish successfully while reporting refused geometry.
use super::{creases,harness,motions,tools};
use gcs_core::solid::swept_boundary::{SweptBoundaryOptions,ConstructError as E,candidate_from,seeds,BoundaryCandidate};

// Enduring tests check reporting and acceptance invariants. Exact current refusal
// categories/counts belong to the exported matrix, not construction requirements.
macro_rules! reported_case {
    ($name:ident,$source:expr,$sagitta:expr) => {
        #[test]
        fn $name() {
            use gcs_core::solid::swept_boundary::{candidate,Stage};
            let e = harness::read(&$source);
            let swept = harness::solid(&e,"swept");
            let options = SweptBoundaryOptions {sagitta:$sagitta,..Default::default()};
            let mut reported = false;
            let c = candidate(&e.sketch,swept,&options,&|_| {},&mut |stage,_| {
                if let Stage::Certified {..} = stage { reported = true; }
            }).unwrap();
            assert!(reported,"candidate diagnostics must not wait for closure");
            let cert = &c.certificate;
            assert_eq!(cert.certified+cert.failures.len()+cert.unresolved.len(),c.mesh.triangles.len());
            assert!(cert.surface_area >= cert.failed_area+cert.unresolved_area-1e-9);
            check_acceptance(&c,&options);
        }
    }
}
fn check_acceptance(c: &BoundaryCandidate,options: &SweptBoundaryOptions) {
    match c.accept(options,options.audit()) {
        Ok(accepted) => {
            assert!(c.topology.is_ok() && c.certificate.is_complete());
            assert!(accepted.certificate().is_complete());
            assert!(accepted.spatial().distance_bound() <= options.audit().tolerance);
        }
        Err(E::Topology(_)) => assert!(c.topology.is_err()),
        Err(E::Certificate {failed,unresolved}) => {
            assert!(c.topology.is_ok() && !c.certificate.is_complete());
            assert_eq!((failed,unresolved),(c.certificate.failures.len(),c.certificate.unresolved.len()));
        }
        Err(E::Spatial(_)) => assert!(c.topology.is_ok() && c.certificate.is_complete()),
        Err(e) => panic!("unexpected acceptance barrier: {e:?}"),
    }
}
reported_case!(turning_prism_reports_open_topology,creases::turning_prism(),0.02);
reported_case!(turned_lens_reports_vertex_topology,creases::turned_lens(),0.02);
reported_case!(turned_box_reports_surface_evidence,creases::turned_box(),0.02);
reported_case!(tumbling_cylinder_reports_surface_before_closure,creases::tumbling_cylinder(),0.02);
reported_case!(sliding_box_reports_surface_evidence,format!("{}{}{}",tools::BOX,motions::slide_x(10.),motions::swept("feed",0.,360.)),0.02);
reported_case!(sliding_prism_reports_surface_evidence,format!("{}{}{}",tools::TRIANGLE_PRISM,motions::slide_x(10.),motions::swept("feed",0.,360.)),0.02);
reported_case!(plunged_cylinder_reports_disconnected_topology,format!("{}{}{}",tools::CYLINDER,"motion plunge(along: axis, advance: 6mm)\n",motions::swept("plunge",0.,360.)),0.02);
reported_case!(turned_cylinder_reports_open_topology,format!("{}{}{}",tools::CYLINDER,motions::TURN_SPINDLE,motions::swept("turn",-60.,60.)),0.02);
reported_case!(torus_candidate_reports_spatial_evidence,format!("{}{}{}",tools::SPHERE,motions::TURN_SPINDLE,motions::swept("turn",-60.,60.)),0.02);
reported_case!(capsule_candidate_reports_spatial_evidence,format!("{}{}{}",tools::SPHERE,motions::slide_z(10.),motions::swept("feed",0.,360.)),0.02);
reported_case!(negative_capsule_reports_spatial_evidence,format!("{}{}{}",tools::SPHERE,motions::slide_z(-10.),motions::swept("feed",0.,360.)),0.02);
reported_case!(stationary_sphere_reports_spatial_evidence,format!("{}{}{}",tools::SPHERE,motions::TURN_OWN_AXIS,motions::swept("turn",-60.,60.)),0.05);
reported_case!(whole_turn_box_reports_surface_evidence,format!("{}{}{}",tools::BOX,motions::turn_about(4.,-1.5,5.,-1.5),motions::swept("turn",0.,360.)),0.02);

#[test]
fn dumbbell_reports_construction_or_a_candidate() {
    use gcs_core::solid::swept_boundary::{candidate,JudgeError};
    let source = format!("{}{}{}",tools::DUMBBELL,motions::slide_x(4.),motions::swept("feed",0.,360.));
    let e = harness::read(&source);
    let swept = harness::solid(&e,"swept");
    let result = candidate(&e.sketch,swept,&SweptBoundaryOptions::default(),&|_| {},&mut |_,_| {});
    match result {
        Err(E::Judge(JudgeError::ReversedNormal {..})) => {},
        Ok(c) => assert_eq!(c.certificate.triangles,c.mesh.triangles.len()),
        Err(e) => panic!("new construction barrier: {e:?}"),
    }
}

#[test]
fn perturbed_cylinders_keep_candidate_and_acceptance_evidence_separate() {
    let e = harness::read(&creases::tumbling_cylinder());
    let swept = harness::solid(&e,"swept");
    let options = SweptBoundaryOptions::default();
    let (_,sheets,_) = seeds(&e.sketch,swept,options.spacing,options.sagitta,&|_| {}).unwrap();
    for phase in [0.,1.] {
        let mut moved = sheets.clone(); let mut k = 0_f64;
        for s in &mut moved { for p in &mut s.points {
            k += 1.; *p = [p[0]+1e-12*(1.7*k+phase).sin(),p[1]+1e-12*(2.3*k+phase).cos(),p[2]+1e-12*(3.1*k+1.+phase).sin()];
        } }
        let c = candidate_from(&e.sketch,swept,&options,moved,&mut |_,_| {}).unwrap();
        check_acceptance(&c,&options);
    }
}

#[test]
#[ignore = "exports the phase-zero status matrix and candidate STLs to SOLVENT_EXPORT"]
fn phase_zero_status() {
    let dir = std::path::PathBuf::from(std::env::var("SOLVENT_EXPORT").expect("set a separate artifact directory"));
    std::fs::create_dir_all(&dir).unwrap();
    let cylinder = creases::tumbling_cylinder();
    let cases = [
        ("turning_prism",creases::turning_prism(),None),
        ("turned_lens",creases::turned_lens(),None),
        ("turned_box",creases::turned_box(),None),
        ("tumbling_cylinder",cylinder.clone(),None),
        ("sliding_dumbbell",format!("{}{}{}",tools::DUMBBELL,motions::slide_x(4.),motions::swept("feed",0.,360.)),None),
        ("whole_turn_box",format!("{}{}{}",tools::BOX,motions::turn_about(4.,-1.5,5.,-1.5),motions::swept("turn",0.,360.)),None),
        ("tumbling_cylinder_perturbed_0",cylinder.clone(),Some(0.)),
        ("tumbling_cylinder_perturbed_1",cylinder,Some(1.)),
    ];
    let mut report = String::from("case\ttriangles\topen_loops\tcentroid_brackets\tfailed\tunresolved\tarea\tfailed_area\tunresolved_area\ttopology\tacceptance\n");
    for (name,source,phase) in cases {
        eprintln!("== {name}");
        std::fs::write(dir.join(format!("{name}.sv")),&source).unwrap();
        let e = harness::read(&source);
        let swept = harness::solid(&e,"swept");
        let options = SweptBoundaryOptions::default();
        let (_,mut sheets,_) = seeds(&e.sketch,swept,options.spacing,options.sagitta,&|_| {}).unwrap();
        if let Some(phase) = phase {
            let mut k = 0_f64;
            for s in &mut sheets { for p in &mut s.points {
                k += 1.; *p = [p[0]+1e-12*(1.7*k+phase).sin(),p[1]+1e-12*(2.3*k+phase).cos(),p[2]+1e-12*(3.1*k+1.+phase).sin()];
            } }
        }
        let line = match candidate_from(&e.sketch,swept,&options,sheets,&mut |_,_| {}) {
            Ok(c) => {
                let mesh = c.mesh.compact();
                let (bytes,_) = creases::stl(&mesh.vertices,&mesh.triangles,name);
                std::fs::write(dir.join(format!("{name}.stl")),bytes).unwrap();
                let cert = &c.certificate;
                let acceptance = c.accept(&options,options.audit()).map(|b| format!("accepted within {}",b.spatial().distance_bound())).unwrap_or_else(|e| format!("{e:?}"));
                format!("{name}\t{}\t{:?}\t{}\t{}\t{}\t{:.6}\t{:.6}\t{:.6}\t{:?}\t{acceptance}\n",
                    mesh.triangles.len(),c.unpaired.iter().map(Vec::len).collect::<Vec<_>>(),cert.certified,
                    cert.failures.len(),cert.unresolved.len(),cert.surface_area,cert.failed_area,cert.unresolved_area,
                    c.topology.as_ref().map(|s| s.genus()))
            }
            Err(e) => format!("{name}\tconstruction refused before final candidate: {e:?}\n"),
        };
        eprint!("{line}"); report.push_str(&line);
        std::fs::write(dir.join("status.tsv"),&report).unwrap();
    }
}

#[test]
fn cylinder_spatial_report_does_not_wait_for_closure() {
    use gcs_core::solid::swept_boundary::{candidate,Check};
    let e = harness::read(&creases::tumbling_cylinder());
    let options = SweptBoundaryOptions::default();
    let c = candidate(&e.sketch,harness::solid(&e,"swept"),&options,&|_| {},&mut |_,_| {}).unwrap();
    let mut audit = options.audit(); audit.max_cells = 128;
    let r = c.inspect(&options,audit);
    let Check::Attempted(a) = r.spatial() else { panic!("missing independent audit"); };
    assert!(a.visited().iter().all(|&n| n > 0));
    assert!(a.visited().iter().sum::<usize>() <= 128);
    assert!(!a.surface_unfinished().is_empty() && !a.coverage_unfinished().is_empty());
    eprintln!("cylinder audit: visits {:?}, completed {:?}, unfinished {:?}",a.visited(),
        [a.surface().len(),a.coverage().len()],[a.surface_unfinished().len(),a.coverage_unfinished().len()]);
    assert!(r.into_accepted().is_err());
}
