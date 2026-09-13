//! Phase 3 source connection: ordinary tracing, without the Phase 2 atlas.
use super::{harness,tools,motions};
use gcs_core::solid::{SweepContacts,sweep_source::{Chart,Source,SourcePoint,Error},swept_boundary as sb};

fn read(tool: &str,motion: &str) -> SweepContacts {
    let e = harness::read(&format!("{tool}{motion}{}",motions::swept("turn",-30.,30.)));
    SweepContacts::read(&e.sketch,harness::solid(&e,"swept"),1e-10).unwrap()
}
fn chart_parameters(chart: Chart,p: SourcePoint,roll: f64) -> [f64;2] {
    match chart { Chart::Endpoint {..} => p.parameters,
        Chart::Stationary {..} => [p.parameters[1],roll],_ => [p.parameters[0],roll] }
}

#[test]
fn native_coordinates_survive_contact_chaining_across_tool_families() {
    let mut kinds = [false;3]; let mut joints = 0; let mut charts = 0;
    for tool in [tools::CYLINDER,tools::BOX,tools::SPHERE,tools::LENS] {
        let motion = if tool == tools::LENS { motions::turn_about(3.4,0.,3.4,1.) }
            else if tool == tools::SPHERE { motions::TURN_SPINDLE.into() } else { motions::TUMBLE.into() };
        let sweep = read(tool,&motion);
        for roll in [-0.4,0.1,0.4] {
            for curve in sweep.characteristics_over(roll,1e-9).unwrap() {
                assert_eq!(curve.points.len(),curve.sources.len());
                for (p,observations) in curve.points.iter().zip(&curve.sources) {
                    assert!(!observations.is_empty()); joints += usize::from(observations.len() > 1);
                    for observation in observations {
                        let source = observation.point;
                        kinds[match source.source { Source::Face(_) => 0,Source::Edge(_) => 1,Source::Crease(_) => 2 }] = true;
                        let at = sweep.source_at(source,roll).unwrap();
                        // Chaining previously kept just one of these observations.
                        let posed = sweep.motion().at(roll).unwrap().point(*p);
                        assert!(harness::distance(at.position,posed) < 1e-5,"{source:?}");
                        for f in &at.faces {
                            assert!(harness::distance(f.position,at.faces[0].position) < 1e-7,"edge consumers {at:?}");
                        }
                        if let Some(chart) = observation.chart {
                            let (_,_,q) = chart.at(&sweep,chart_parameters(chart,source,roll),1e-9).unwrap();
                            assert!(harness::distance(q.position,at.position) < 1e-8);
                            charts += 1;
                        }
                    }
                }
            }
        }
    }
    assert_eq!(kinds,[true;3]); assert!(joints > 0); assert!(charts > 100);
}

#[test]
fn traced_cylinder_keeps_source_snapshot_after_simplification_and_motion_sampling() {
    for spacing in [0.35,0.5,0.7] {
        let source = super::creases::tumbling_cylinder(); let e = harness::read(&source);
        let swept = harness::solid(&e,"swept");
        let (_,sheets,_) = sb::seeds(&e.sketch,swept,spacing,0.02,&|_| {}).unwrap();
        assert!(!sheets.is_empty());
        if spacing == 0.5 {
            // FNV-1a of the geometry arrays and counts from Phase 1a's frozen
            // seeds-and-label-witnesses.txt. Provenance is intentionally excluded.
            let hashes: Vec<_> = sheets.iter().map(geometry_fingerprint).collect();
            assert_eq!(hashes,vec![0x250425aa35f01c98,0xaa5b5df11e5a8eea,0x28971cd1af04a4b0,
                0xbb31c3f531a9a6e7,0xfef177db9f69c298,0x71da149935079f89,0x329c28ea122d98c0,
                0xde97b33ae7db83af,0x1fa543d7e934b219]);
        }
        let mut total = 0; let mut worst = 0_f64;
        for mut sheet in sheets {
            let provenance = sheet.provenance.as_ref().unwrap();
            assert_eq!(provenance.samples().len(),sheet.points.len());
            for (i,(roll,observations)) in provenance.samples().iter().enumerate() {
                assert_eq!(*roll,sheet.times[sheet.column[i] as usize]);
                assert!(!observations.is_empty()); total += observations.len();
            }
            let errors = provenance.deviations(&sheet.points).unwrap();
            worst = worst.max(errors.into_iter().fold(0.,f64::max));
            // Editing a candidate must not edit its source geometry or manufacture
            // native coordinates for the moved position.
            let before = provenance.at(0).unwrap()[0].position;
            sheet.points[0][0] += 1.;
            assert_eq!(provenance.at(0).unwrap()[0].position,before);
            assert!(provenance.deviations(&sheet.points).unwrap()[0] > 0.99);
            assert_eq!(provenance.deviations(&[]),Err(Error::VertexCount));
        }
        assert!(total > 100); assert!(worst < 1e-5,"spacing={spacing} worst={worst}");
        eprintln!("source replay spacing={spacing} observations={total} max residual={worst}");
    }
}

#[test]
fn carried_sheets_keep_row_major_native_coordinates() {
    let sweep = read(tools::CYLINDER,motions::TURN_SPINDLE);
    let sheets = sweep.carried_sheets(0.5,0.02,0.01,1e-9).unwrap();
    assert!(!sheets.is_empty());
    for sheet in sheets {
        assert_eq!(sheet.provenance.samples().len(),sheet.rows*sheet.columns);
        for (i,(t,_)) in sheet.provenance.samples().iter().enumerate() { assert_eq!(*t,sheet.times[i%sheet.columns]); }
        assert!(sheet.provenance.deviations(&sheet.points).unwrap().into_iter().all(|e| e < 1e-5));
    }
}

#[test]
fn source_charts_evaluate_between_trace_samples_and_refuse_invalid_requests() {
    let sweep = read(tools::CYLINDER,motions::TUMBLE);
    // Both consumers are evaluated from their own charts at non-grid parameters.
    for edge in 0..sweep.edges().len() {
        for t in [0.137,0.371,0.823] {
            let (p,roll,q) = Chart::Edge(edge).at(&sweep,[t,0.123],1e-9).unwrap();
            assert_eq!(p.parameters,[t,0.]); assert_eq!(roll,0.123); assert_eq!(q.faces.len(),2);
            assert!(harness::distance(q.faces[0].position,q.faces[1].position) < 1e-12);
            let tool = q.faces[0].position;
            // This source's rims are exact unit circles, even between samples.
            assert!(((tool[0]-3.).powi(2)+tool[1].powi(2)-1.).abs() < 1e-12);
        }
    }
    for face in 0..sweep.faces().len() { for end in [sb::End::From,sb::End::To] {
        let (_,roll,q) = Chart::Endpoint {face,end}.at(&sweep,[0.371,0.823],1e-9).unwrap();
        assert_eq!(roll,sweep.domain()[usize::from(end == sb::End::To)]);
        let [x,y,z] = q.faces[0].position; let (s,c) = roll.sin_cos();
        assert!(harness::distance(q.position,[x,c*y-s*z,s*y+c*z]) < 1e-12);
        let radial = (x-3.).hypot(y);
        assert!((radial-1.).abs() < 1e-12 || (z.abs()-1.).abs() < 1e-12);
    } }
    let curves = sweep.characteristics_over(0.123,1e-9).unwrap();
    let mut evaluated = 0;
    for o in curves.iter().flat_map(|c| &c.sources).flatten() {
        if let Some(chart @ Chart::Station {..}) = o.chart {
            // Query new u and roll directly on the original station equation.
            if let Ok((_,_,q)) = chart.at(&sweep,[0.37123,0.23456],1e-9) {
                let p = q.faces[0].position;
                assert!(sweep.source_material().value(p).abs() < 1e-9); evaluated += 1;
            }
        }
    }
    assert!(evaluated > 0);
    assert!(matches!(Chart::Edge(usize::MAX).at(&sweep,[0.5,0.],1e-9),Err(Error::MissingSource(_))));
    for p in [[f64::NAN,0.],[0.5,f64::INFINITY],[-0.1,0.],[0.5,2.]] {
        assert!(Chart::Edge(0).at(&sweep,p,1e-9).is_err());
    }
    assert!(Chart::Station {face:1,branch:usize::MAX}.at(&sweep,[0.371,0.],1e-9).is_err());
    assert!(Chart::Stationary {face:1,u:0.371}.at(&sweep,[0.173,0.],1e-9).is_err());
    assert!(Chart::Edge(0).at(&sweep,[0.5,0.],f64::NAN).is_err());
}

#[test]
fn validation_controls_do_not_change_construction_spacing_or_trimming() {
    let original = sb::SweptBoundaryOptions::default();
    let refined = sb::SweptBoundaryOptions {field_value_tolerance:Some(0.000005),
        minimum_probe_distance:Some(0.00002),spatial_audit:Some(sb::AuditOptions {
            max_cells:800000,max_depth:36,..original.audit()}),..original};
    assert_eq!(refined.vertex_tolerance(),original.vertex_tolerance());
    assert_eq!(refined.snap(),original.snap()); assert_eq!(refined.crease_merge(),original.crease_merge());
    assert_eq!(refined.judge_tolerance(),0.000005); assert_eq!(refined.least_probe(),0.00002);
    assert_eq!(refined.audit().tolerance,0.04); assert_eq!(refined.audit().max_cells,800000);
    // The actual 24-triangle calibration refusal survives; both explicit
    // controls are needed and no change to the mesh is involved.
    let m = sb::shared::tessellate(&super::cylinder_domains::domains(super::cylinder_oracle::Cylinder::fixture().half_roll,0.),
        sb::shared::Options {sagitta:0.004,agreement:1e-11,max_level:7,max_triangles:200000}).unwrap().mesh;
    let e = harness::read(&super::creases::tumbling_cylinder());
    let field = gcs_core::solid::MaterialField::read(&e.sketch,harness::solid(&e,"swept"),1e-10).unwrap();
    let certify = |opts: sb::SweptBoundaryOptions,triangles: &[[u32;3]]| {
        let mut judge = sb::FieldJudge::new(field.clone(),opts.judge_tolerance(),opts.near_budget,opts.far_budget,opts.cached_poses);
        sb::certify(&mut judge,&m.vertices,triangles,opts.probe_distance(),opts.least_probe()).unwrap()
    };
    let coarse = certify(original,&m.triangles); assert_eq!(coarse.unresolved.len(),24);
    let triangles: Vec<_> = coarse.unresolved.iter().map(|x| m.triangles[x.0]).collect();
    assert_eq!(certify(sb::SweptBoundaryOptions {field_value_tolerance:refined.field_value_tolerance,..original},&triangles).certified,0);
    assert_eq!(certify(sb::SweptBoundaryOptions {minimum_probe_distance:refined.minimum_probe_distance,..original},&triangles).certified,0);
    assert!(certify(refined,&triangles).is_complete());
}

fn geometry_fingerprint(p: &gcs_core::solid::SweepPatch) -> u64 {
    let mut h = 0xcbf29ce484222325u64;
    let mut bytes = |data: &[u8]| { for &b in data { h = (h ^ b as u64).wrapping_mul(0x100000001b3); } };
    for array in [&p.points,&p.normals] {
        bytes(&(array.len() as u64).to_le_bytes());
        for x in array.iter().flatten() { bytes(&x.to_le_bytes()); }
    }
    bytes(&(p.triangles.len() as u64).to_le_bytes());
    for x in p.triangles.iter().flatten() { bytes(&x.to_le_bytes()); }
    bytes(&(p.column.len() as u64).to_le_bytes());
    for x in &p.column { bytes(&x.to_le_bytes()); }
    bytes(&(p.times.len() as u64).to_le_bytes());
    for x in &p.times { bytes(&x.to_le_bytes()); }
    bytes(&[u8::from(p.closed)]); h
}
