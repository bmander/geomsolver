//! Phase 2: source-domain construction and independent validation kept separate.
use super::{cylinder_domains::domains,cylinder_oracle::Cylinder,cylinder_replay as replay};
use gcs_core::solid::swept_boundary::{self as sb,shared::{self,Options,Error}};
fn options() -> Options { Options {sagitta:0.004,agreement:1e-11,max_level:7,max_triangles:200000} }

// At a concave seam, even an exact face's outward normal can enter the
// neighboring material at a large offset. Keep the fixed-probe report, and
// independently look for smaller strict sides; neither is a spatial proof.
fn assert_local_sides(c: Cylinder,m: &sb::KeptMesh) {
    use super::cylinder_oracle::Side;
    for (i,t) in m.triangles.iter().enumerate() {
        let p = t.map(|v| m.vertices[v as usize]);
        let n = sb::triangle_normal(p[0],p[1],p[2]).unwrap();
        for weights in [[1./3.;3],[0.6,0.2,0.2],[0.2,0.6,0.2],[0.2,0.2,0.6]] {
            let at: [f64;3] = std::array::from_fn(|k| (0..3).map(|j| weights[j]*p[j][k]).sum());
            assert!((0..15).any(|k| {
                let h = 0.04/2_f64.powi(k);
                [-1.,1.].map(|s| c.side(std::array::from_fn(|k| at[k]+s*h*n[k]))) == [Side::Material,Side::Exterior]
            }),"no local side witnesses triangle={i} source={} points={p:?} at={at:?}",m.sheet[i]);
        }
    }
}

#[test]
fn shared_domains_close_without_aliases_or_bands() {
    let p = domains(Cylinder::fixture().half_roll,0.);
    let m = shared::tessellate(&p,options()).unwrap();
    eprintln!("shared cylinder patches={} edges={} level={} vertices={} triangles={} error={} volume={}",p.len(),m.edges.len(),m.level,m.mesh.vertices.len(),m.mesh.triangles.len(),m.sampled_error,replay::signed_volume(&m.mesh));
    assert_eq!(m.faces.len(),m.mesh.triangles.len());
    assert!(sb::boundary_loops(&m.mesh.triangles).is_empty());
    assert!(super::shared_intersections::unexpected(&m.mesh,1e-9).is_empty());
    assert!(m.mesh.sheet.iter().all(|&s| s < 8));
    let sides = replay::measure(Cylinder::fixture(),&m.mesh,0.02,4);
    eprintln!("{sides:?}");
    assert_eq!(sides.reversed_area,0.);
    assert_local_sides(Cylinder::fixture(),&m.mesh);
    assert!((replay::signed_volume(&m.mesh)-9.426350).abs() < 0.025);
    for e in &m.edges { for u in &e.uses {
        let p = p.iter().find(|p| p.id == u.patch).unwrap();
        for (&v,&uv) in e.vertices.iter().zip(&u.parameters) {
            assert!(super::harness::distance((p.evaluate)(uv),m.mesh.vertices[v as usize]) < 1e-11);
        }
    } }
}

#[test]
fn shared_domains_refuse_inconsistent_identity_and_resource_limits() {
    let mut p = domains(Cylinder::fixture().half_roll,0.);
    let saved = std::mem::replace(&mut p[0].evaluate,Box::new(|_| [0.;3]));
    assert!(matches!(shared::tessellate(&p,options()),Err(Error::InconsistentCorner(_))));
    p[0].evaluate = Box::new(move |uv| {
        let mut x = saved(uv); x[0] += 0.01*(std::f64::consts::PI*uv[0]).sin(); x
    });
    assert!(matches!(shared::tessellate(&p,options()),Err(Error::InconsistentEdge(_))));
    let p = domains(Cylinder::fixture().half_roll,0.);
    assert!(matches!(shared::tessellate(&p,Options {max_triangles:1,..options()}),Err(Error::Budget)));
}

#[test]
fn shared_domains_are_order_independent_and_sampling_stable() {
    let c = Cylinder::fixture(); let mut p = domains(c.half_roll,0.);
    let a = shared::tessellate(&p,options()).unwrap(); p.reverse();
    let b = shared::tessellate(&p,options()).unwrap();
    assert_eq!(a.mesh.vertices,b.mesh.vertices); assert_eq!(a.mesh.triangles,b.mesh.triangles); assert_eq!(a.mesh.sheet,b.mesh.sheet);
    for h in [c.half_roll-0.001,c.half_roll,c.half_roll+0.001] {
        let c = Cylinder {half_roll:h}; let p = domains(h,0.1);
        let b = shared::tessellate(&p,options()).unwrap();
        let sides = replay::measure(c,&b.mesh,0.02,2);
        assert_eq!(sides.reversed_area,0.); assert_local_sides(c,&b.mesh);
        assert!((replay::signed_volume(&b.mesh)-c.volume(256)).abs() < 0.025);
        assert!(super::shared_intersections::unexpected(&b.mesh,1e-9).is_empty());
        let forward = replay::forward_distance(&a.mesh,&b.mesh,0.01);
        let reverse = replay::forward_distance(&b.mesh,&a.mesh,0.01);
        assert_eq!(forward.0,0.); assert_eq!(reverse.0,0.);
        eprintln!("roll={h} interior warp=.1 baseline/variant distances={forward:?}/{reverse:?}");
    }
}

/// Reuses the Phase 1 oracle/reporting and exact Phase 1a archived baseline.
/// Explicitly run because continuous-field audits are substantially slower than
/// the independent analytic diagnostics in the ordinary regression tests.
#[test]
#[ignore]
fn phase_two_cylinder_replay() {
    use super::{creases,harness,shared_intersections};
    use gcs_core::solid::MaterialField;
    use std::{fmt::Write,path::PathBuf};
    let dir = PathBuf::from(std::env::var("SOLVENT_EXPORT").expect("set SOLVENT_EXPORT to a new replay directory"));
    std::fs::create_dir_all(&dir).unwrap();
    let baseline = dir.join("baseline"); std::fs::create_dir_all(&baseline).unwrap();
    let bundle = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../docs/fixtures/cylinder-phase-one-a/replay.tar.gz");
    assert!(std::process::Command::new("tar").arg("-xzf").arg(&bundle).arg("-C").arg(&baseline).status().unwrap().success());
    let c = Cylinder::fixture(); let source = creases::tumbling_cylinder();
    std::fs::write(dir.join("source.sv"),&source).unwrap();
    let e = harness::read(&source); let swept = harness::solid(&e,"swept");
    let field = MaterialField::read(&e.sketch,swept,1e-10).unwrap();
    let p = domains(c.half_roll,0.); let built = shared::tessellate(&p,options()).unwrap();
    let mut report = format!("# Phase 2 explicit source-chart experiment on the Phase 1a fixture\n# Sampled binary64 geometry diagnostics are not interval certificates.\n# Eight analytic support IDs replace, and are not aliases of, eleven old sheet IDs.\npatches={} edges={} level={} triangles={} vertices={} sampled_chord_error={}\n",p.len(),built.edges.len(),built.level,built.mesh.triangles.len(),built.mesh.vertices.len(),built.sampled_error);
    std::fs::write(dir.join("options.txt"),"sagitta=0.004 agreement=1e-11 max_level=7 max_triangles=200000\naudit tolerance=0.04 max_cells=256 max_depth=36 box_budget=1000\n").unwrap();
    replay::write_mesh(&dir.join("shared.mesh"),&built.mesh);
    std::fs::write(dir.join("domains-and-edges.txt"),format!("supports: 0/1 endpoint wall; 2/3 endpoint disk; 4/5 circular rim; 6/7 extremal wall contact\npatches={:?}\nedges={:#?}\nfaces={:#?}\n",p.iter().map(|p| (p.id,p.source,p.corners,p.edges)).collect::<Vec<_>>(),built.edges,built.faces)).unwrap();
    for n in [1,2,4,8] { writeln!(report,"shared sides subdivision={n}: {:?}",replay::measure(c,&built.mesh,0.02,n)).unwrap(); }
    writeln!(report,"shared volume={:.12} oracle volume={:.12}",replay::signed_volume(&built.mesh),c.volume(1024)).unwrap();
    let intersections = shared_intersections::unexpected(&built.mesh,1e-9);
    writeln!(report,"unexpected intersections at numeric tolerance 1e-9: {} {:?}",intersections.len(),intersections.iter().take(10).collect::<Vec<_>>()).unwrap();
    std::fs::write(dir.join("report.txt"),&report).unwrap();
    assert!(intersections.is_empty(),"{}",&report);
    for n in [32,64,128] {
        let reference = replay::reference(c,n);
        for tol in [0.01,0.02,0.04] {
            let reverse = replay::coverage(c,&reference,&built.mesh,tol);
            writeln!(report,"reference n={n} to shared tol={tol}: {reverse:?}").unwrap();
            assert_eq!(reverse.1,0.);
            let forward = replay::forward_distance(&built.mesh,&reference,tol);
            writeln!(report,"shared to reference n={n} tol={tol}: {forward:?}").unwrap();
            assert_eq!(forward.0,0.);
        }
    }
    std::fs::write(dir.join("sections.svg"),replay::sections(c,&built.mesh,&[2.05,2.3,3.,3.7])).unwrap();
    // Compare the unmodified complete group, not just the later zip window.
    for name in ["01-clipped.mesh","08-zipped.mesh"] {
        let path = baseline.join(name);
        let m = replay::read_mesh(&std::fs::read_to_string(&path).unwrap());
        writeln!(report,"baseline {name}: triangles={} sides={:?}",m.triangles.len(),replay::measure(c,&m,0.02,4)).unwrap();
    }
    eprintln!("{report}\nStarting unchanged strict field validator...");
    std::fs::write(dir.join("report.txt"),&report).unwrap();
    report.push_str(&audit_report(field,built.mesh,&dir,256,None).0);
    eprintln!("{report}"); std::fs::write(dir.join("report.txt"),report).unwrap();
}

#[test]
fn shared_planar_domains_use_declared_identity_and_refuse_missing_consumers() {
    use std::collections::BTreeMap;
    let points = [[-1.,-1.,-1.],[1.,-1.,-1.],[1.,1.,-1.],[-1.,1.,-1.],
        [-1.,-1.,1.],[1.,-1.,1.],[1.,1.,1.],[-1.,1.,1.]];
    let faces = [[0,3,2,1],[4,5,6,7],[0,1,5,4],[1,2,6,5],[2,3,7,6],[3,0,4,7]];
    let mut ids = BTreeMap::new(); let mut patches = Vec::new();
    for (id,corners) in faces.into_iter().enumerate() {
        let edges = std::array::from_fn(|k| { let (a,b) = (corners[k],corners[(k+1)%4]);
            let next = ids.len() as u32; *ids.entry((a.min(b),a.max(b))).or_insert(next)
        });
        let p = corners.map(|v| points[v as usize]);
        patches.push(shared::Patch {id:id as u32,source:id as u32,corners,edges,minimum_levels:[0,0],evaluate:Box::new(move |[u,v]| {
            let weights = [(1.-u)*(1.-v),(1.-u)*v,u*v,u*(1.-v)];
            std::array::from_fn(|k| (0..4).map(|i| weights[i]*p[i][k]).sum())
        })});
    }
    let cube = shared::tessellate(&patches,options()).unwrap();
    assert_eq!(cube.mesh.vertices.len(),8); assert_eq!(cube.mesh.triangles.len(),12);
    assert_eq!(replay::signed_volume(&cube.mesh),8.);
    assert!(super::shared_intersections::unexpected(&cube.mesh,1e-10).is_empty());
    patches[0].edges[0] = 1000;
    assert!(matches!(shared::tessellate(&patches,options()),Err(Error::EdgeIncidence(_))));
}

#[test]
fn frozen_compressed_rim_triangle_has_no_local_normal_bracket() {
    use super::cylinder_oracle::Side;
    let m = replay::read_mesh(include_str!("../../../../docs/fixtures/cylinder-phase-two/isotropic-rim.mesh"));
    let p = m.triangles[0].map(|v| m.vertices[v as usize]);
    let n = sb::triangle_normal(p[0],p[1],p[2]).unwrap();
    let at: [f64;3] = std::array::from_fn(|k| 0.6*p[0][k]+0.2*p[1][k]+0.2*p[2][k]);
    assert!((0..15).all(|k| {
        let h = 0.04/2_f64.powi(k);
        [-1.,1.].map(|s| Cylinder::fixture().side(std::array::from_fn(|k| at[k]+s*h*n[k]))) != [Side::Material,Side::Exterior]
    }));
}

fn audit_report(field: gcs_core::solid::MaterialField,mesh: sb::KeptMesh,dir: &std::path::Path,cells: usize,vertex_tolerance: Option<f64>) -> (String,bool) {
    use std::fmt::Write;
    let start = std::time::Instant::now();
    let mut report = format!("audit tolerance=.04 max_cells={cells} depth=36 vertex_tolerance={vertex_tolerance:?}\n");
    let opts = sb::SweptBoundaryOptions {vertex_tolerance,..Default::default()};
    let audit = sb::AuditOptions {max_cells:cells,max_depth:36,..opts.audit()};
    let validation = sb::inspect(field,mesh,&opts,audit);
    if let sb::Check::Attempted(Ok(shell)) = validation.topology() {
        writeln!(report,"topology: closed, vertices={} edges={} faces={} Euler={}",shell.vertex_count(),shell.edges().len(),shell.faces().len(),shell.euler_characteristic()).unwrap();
    } else { writeln!(report,"topology: {:?}",validation.topology()).unwrap(); }
    writeln!(report,"query stats={:?}",validation.stats()).unwrap();
    if let sb::Check::Attempted(Ok(c)) = validation.certificate() {
        writeln!(report,"centroid certified={} failed={} unresolved={} failed_area={} unresolved_area={}",c.certified,c.failures.len(),c.unresolved.len(),c.failed_area,c.unresolved_area).unwrap();
    }
    std::fs::write(dir.join("certificate-and-stats.txt"),format!("{:#?}\n{:#?}",validation.certificate(),validation.stats())).unwrap();
    if let sb::Check::Attempted(a) = validation.spatial() {
        writeln!(report,"spatial complete={} visited={:?} forward witnesses={} reverse witnesses={} forward unfinished={} reverse unfinished={} not attempted={:?}/{:?}",a.is_complete(),a.visited(),a.surface().len(),a.coverage().len(),a.surface_unfinished().len(),a.coverage_unfinished().len(),a.not_attempted(),a.coverage_not_attempted()).unwrap();
        std::fs::write(dir.join("spatial-witnesses.txt"),format!("{:#?}\n{:#?}",a.surface(),a.coverage())).unwrap();
        std::fs::write(dir.join("spatial-obligations.txt"),format!("{:#?}\n{:#?}",a.surface_unfinished(),a.coverage_unfinished())).unwrap();
    }
    let accepted = match validation.into_accepted() {
        Ok(surface) => { writeln!(report,"unchanged acceptance gate: PASS distance_bound={}",surface.spatial().distance_bound()).unwrap(); true },
        Err(e) => { writeln!(report,"unchanged acceptance gate: REFUSED {e:?}; no replacement installed").unwrap(); false },
    };
    writeln!(report,"audit seconds={:.3}",start.elapsed().as_secs_f64()).unwrap();
    (report,accepted)
}

/// A small, complete report of unresolved work when the calibration-sized
/// audit is too expensive. It never promotes a partial report to acceptance.
#[test]
#[ignore]
fn phase_two_bounded_audit() {
    use super::{harness,creases};
    let root = std::path::PathBuf::from(std::env::var("SOLVENT_EXPORT").unwrap());
    let vertex_tolerance: Option<f64> = std::env::var("SOLVENT_AUDIT_VERTEX_TOLERANCE").ok().map(|s| s.parse().unwrap());
    let cells = std::env::var("SOLVENT_AUDIT_CELLS").ok().map(|s| s.parse().unwrap()).unwrap_or(256);
    let name = if vertex_tolerance.is_some() { format!("refined-audit-{cells}") } else { "bounded-audit".into() };
    let dir = root.join(name); std::fs::create_dir_all(&dir).unwrap();
    let mesh = shared::tessellate(&domains(Cylinder::fixture().half_roll,0.),options()).unwrap().mesh;
    let source = creases::tumbling_cylinder();
    let e = harness::read(&source); let swept = harness::solid(&e,"swept");
    let field = gcs_core::solid::MaterialField::read(&e.sketch,swept,1e-10).unwrap();
    let (report,accepted) = audit_report(field,mesh,&dir,cells,vertex_tolerance);
    if cells >= 800000 && vertex_tolerance == Some(0.00001) { assert!(accepted,"{report}"); }
    eprintln!("{report}"); std::fs::write(dir.join("report.txt"),report).unwrap();
}

/// Separate value precision from allowed probe length: vertex_tolerance in the
/// public options controls both, so an acceptance change alone cannot isolate them.
#[test]
#[ignore]
fn phase_two_probe_controls() {
    use super::{harness,creases};
    use gcs_core::solid::MaterialField;
    use std::fmt::Write;
    let dir = std::path::PathBuf::from(std::env::var("SOLVENT_EXPORT").unwrap());
    std::fs::create_dir_all(&dir).unwrap();
    let m = shared::tessellate(&domains(Cylinder::fixture().half_roll,0.),options()).unwrap().mesh;
    let e = harness::read(&creases::tumbling_cylinder()); let swept = harness::solid(&e,"swept");
    let field = MaterialField::read(&e.sketch,swept,1e-10).unwrap();
    let certify = |triangles: &[[u32;3]],value_width,least| {
        let mut judge = sb::FieldJudge::new(field.clone(),value_width,4000,1000,4096);
        sb::certify(&mut judge,&m.vertices,triangles,0.04,least).unwrap()
    };
    let coarse = certify(&m.triangles,0.0025,0.01);
    let ids: Vec<_> = coarse.unresolved.iter().map(|x| x.0).collect();
    let triangles: Vec<_> = ids.iter().map(|&i| m.triangles[i]).collect();
    let mut report = format!("Unresolved original triangle IDs: {ids:?}\n");
    if !ids.is_empty() {
        for width in [0.0025,0.000005] { for least in [0.01,0.00002] {
            let c = certify(&triangles,width,least);
            writeln!(report,"value width={width} least={least} certified={} failed={} unresolved={} least_used={}",c.certified,c.failures.len(),c.unresolved.len(),c.least_used).unwrap();
            if width == 0.000005 && least == 0.00002 { assert!(c.is_complete()); }
        } }
    }
    eprintln!("{report}"); std::fs::write(dir.join("probe-controls.txt"),report).unwrap();
}

/// The perturbed domains must pass the same complete interval gate, not just
/// the fast oracle samples. Each source declares its own actual roll interval.
#[test]
#[ignore]
fn phase_two_perturbed_acceptance() {
    use super::{harness,tools,motions};
    let root = std::path::PathBuf::from(std::env::var("SOLVENT_EXPORT").unwrap());
    for (i,delta) in [-0.001,0.,0.001].into_iter().enumerate() {
        let h = Cylinder::fixture().half_roll+delta;
        let dir = root.join(format!("variant-{i}")); std::fs::create_dir_all(&dir).unwrap();
        let source = format!("{}{}{}",tools::CYLINDER,motions::TUMBLE,motions::swept("turn",-h.to_degrees(),h.to_degrees()));
        std::fs::write(dir.join("source.sv"),&source).unwrap();
        let e = harness::read(&source); let swept = harness::solid(&e,"swept");
        let field = gcs_core::solid::MaterialField::read(&e.sketch,swept,1e-10).unwrap();
        let mesh = shared::tessellate(&domains(h,0.1),options()).unwrap().mesh;
        replay::write_mesh(&dir.join("shared.mesh"),&mesh);
        let (report,accepted) = audit_report(field,mesh,&dir,800000,Some(0.00001));
        eprintln!("variant {i}, roll={h}, warp=.1\n{report}");
        std::fs::write(dir.join("report.txt"),&report).unwrap();
        assert!(accepted,"{report}");
    }
}
