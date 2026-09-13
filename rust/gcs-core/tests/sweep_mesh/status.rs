//! Reproducible candidate reports; unlike acceptance tests this instrument can
//! finish successfully while reporting refused geometry.
use super::{creases,harness,motions,tools};
use gcs_core::solid::swept_boundary::{SweptBoundaryOptions,ConstructError as E,candidate_from,seeds};
use gcs_core::topology::Error as T;

macro_rules! refused_case {
    ($name:ident,$source:expr,$sagitta:expr,$expected:pat) => {
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
            let result = c.accept(&options,options.audit());
            assert!(matches!(result,Err($expected)),"{result:?}");
        }
    }
}
refused_case!(turning_prism_reports_open_topology,creases::turning_prism(),0.02,E::Topology(T::EdgeUseCount {..}));
refused_case!(turned_lens_reports_vertex_topology,creases::turned_lens(),0.02,E::Topology(T::NonManifoldVertex {..}));
refused_case!(turned_box_reports_unresolved_surface,creases::turned_box(),0.02,E::Certificate {failed:0,unresolved:17});
refused_case!(tumbling_cylinder_reports_surface_before_closure,creases::tumbling_cylinder(),0.02,E::Topology(T::EdgeUseCount {..}));
refused_case!(sliding_box_reports_unresolved_surface,format!("{}{}{}",tools::BOX,motions::slide_x(10.),motions::swept("feed",0.,360.)),0.02,E::Certificate {failed:0,unresolved:2});
refused_case!(sliding_prism_reports_unresolved_surface,format!("{}{}{}",tools::TRIANGLE_PRISM,motions::slide_x(10.),motions::swept("feed",0.,360.)),0.02,E::Certificate {failed:0,unresolved:20});
refused_case!(plunged_cylinder_reports_disconnected_topology,format!("{}{}{}",tools::CYLINDER,"motion plunge(along: axis, advance: 6mm)\n",motions::swept("plunge",0.,360.)),0.02,E::Topology(T::DisconnectedShell));
refused_case!(turned_cylinder_reports_open_topology,format!("{}{}{}",tools::CYLINDER,motions::TURN_SPINDLE,motions::swept("turn",-60.,60.)),0.02,E::Topology(T::EdgeUseCount {..}));
refused_case!(torus_candidate_requires_spatial_evidence,format!("{}{}{}",tools::SPHERE,motions::TURN_SPINDLE,motions::swept("turn",-60.,60.)),0.02,E::Spatial(_));
refused_case!(capsule_candidate_requires_spatial_evidence,format!("{}{}{}",tools::SPHERE,motions::slide_z(10.),motions::swept("feed",0.,360.)),0.02,E::Spatial(_));
refused_case!(negative_capsule_requires_spatial_evidence,format!("{}{}{}",tools::SPHERE,motions::slide_z(-10.),motions::swept("feed",0.,360.)),0.02,E::Spatial(_));
refused_case!(stationary_sphere_requires_spatial_evidence,format!("{}{}{}",tools::SPHERE,motions::TURN_OWN_AXIS,motions::swept("turn",-60.,60.)),0.05,E::Spatial(_));
refused_case!(whole_turn_box_reports_reversed_surface,format!("{}{}{}",tools::BOX,motions::turn_about(4.,-1.5,5.,-1.5),motions::swept("turn",0.,360.)),0.02,E::Certificate {failed:74,unresolved:0});

#[test]
fn dumbbell_reports_the_earlier_field_refusal() {
    use gcs_core::solid::swept_boundary::{candidate,JudgeError};
    let source = format!("{}{}{}",tools::DUMBBELL,motions::slide_x(4.),motions::swept("feed",0.,360.));
    let e = harness::read(&source);
    let swept = harness::solid(&e,"swept");
    let result = candidate(&e.sketch,swept,&SweptBoundaryOptions::default(),&|_| {},&mut |_,_| {});
    assert!(matches!(result,Err(E::Judge(JudgeError::ReversedNormal {..}))));
}

#[test]
fn perturbation_cannot_turn_the_refused_cylinder_into_an_accepted_surface() {
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
        assert!(!c.unpaired.is_empty());
        assert!(!c.certificate.is_complete());
        assert!(matches!(c.accept(&options,options.audit()),Err(gcs_core::solid::swept_boundary::ConstructError::Topology(_))));
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
