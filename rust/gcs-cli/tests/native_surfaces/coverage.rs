//! Automatic contact-domain discovery feeds the same native fitting primitive.
use super::*;
use gcs_core::solid::{ContactCover,ContactCoverOptions,ContactEvidence,ContactLimit,ContactParameter};

fn audit_partition(sweep: &SweepContacts,cover: &ContactCover) {
    audit_domain(sweep,cover,sweep.domain());
}

fn audit_domain(sweep: &SweepContacts,cover: &ContactCover,time: [f64;2]) {
    for (patch,surface) in sweep.patches().iter().enumerate() {
        let [u,v] = surface.domain();
        let original = [u,v,time];
        let cells: Vec<_> = cover.cells.iter().filter(|c| c.patch == patch).collect();
        let mut volume = 0.;
        for cell in &cells {
            volume += cell.parameters.iter().zip(original).map(|(p,o)| {
                let [a,b] = p.bounds();
                assert!(a >= o[0] && b <= o[1]);
                if o[0] == o[1] { assert_eq!(a,b); 1. } else { (b-a)/(o[1]-o[0]) }
            }).product::<f64>();
            if let ContactEvidence::Excluded {value} = cell.evidence { assert!(!value.contains(0.)); }
            if let ContactEvidence::OffSource {material} = cell.evidence { assert!(!material.contains(0.)); }
            if let ContactEvidence::Chart(chart) = cell.evidence {
                let axis = chart.dependent.index(); let [a,b] = chart.range.bounds();
                let [lo,hi] = cell.parameters[axis].bounds();
                let allowed = if chart.dependent == ContactParameter::Angle {
                    surface.angular_chart_domain()
                } else { original[axis] };
                assert!(a >= allowed[0] && b <= allowed[1] && a <= lo && b >= hi);
                assert!(!chart.derivative.contains(0.));
                let [a,b] = chart.ends.map(|i| i.bounds());
                assert!((a[1] < 0. && b[0] > 0.) || (b[1] < 0. && a[0] > 0.));
            }
        }
        assert!((volume-1.).abs() < 1e-10,"patch {patch}: domain volume {volume}");
        for i in 0..29 {
            let p: [f64;3] = std::array::from_fn(|k| {
                let f = ((i*(k+3)+k*11)%29) as f64/29.+0.37/29.;
                original[k][0]+f*(original[k][1]-original[k][0])
            });
            assert_eq!(cells.iter().filter(|c| c.parameters.iter().zip(p).all(|(b,x)| b.contains(x))).count(),1);
        }
    }
}

#[test]
fn complete_source_domain_keeps_unresolved_poles_and_exhausted_regions() {
    let base = Path::new(env!("CARGO_MANIFEST_DIR")).join("../examples");
    let e = read(include_str!("../../../examples/solid_generating_sweep.sv"),&base);
    let sweep = SweepContacts::read(&e.sketch,e.map.ent_named("removal.body").unwrap().i(),1e-10).unwrap();
    for budget in [1,10000] {
        let start = std::time::Instant::now();
        let cover = sweep.cover(ContactCoverOptions {max_depth:10,max_cells:budget}).unwrap();
        audit_partition(&sweep,&cover);
        let unresolved: Vec<_> = cover.cells.iter().filter(|c| matches!(c.evidence,ContactEvidence::Unresolved {..})).collect();
        assert!(!unresolved.is_empty());
        assert!(sweep.at_chart(unresolved[0],0.5,0.5,1e-10).is_err());
        assert!(cover.evaluations <= budget);
        // A single rotation's equation is independent of time. Its contact set
        // requires an angular chart; time subdivision must not duplicate work.
        for cell in &cover.cells { assert_eq!(cell.parameters[2].bounds(),sweep.domain()); }
        if budget == 1 {
            assert!(unresolved.iter().any(|c| matches!(c.evidence,ContactEvidence::Unresolved {limit:ContactLimit::Budget,..})));
        } else {
            assert!(!unresolved.iter().any(|c| matches!(c.evidence,ContactEvidence::Unresolved {limit:ContactLimit::Budget,..})));
            assert!(cover.cells.iter().any(|c| matches!(c.evidence,ContactEvidence::Excluded {..})));
            let charts: Vec<_> = cover.cells.iter().filter_map(|c| match c.evidence {
                ContactEvidence::Chart(chart) => Some((c,chart)), _ => None,
            }).collect();
            assert!(charts.len() >= 4,"time-independent contacts need angular charts");
            assert!(charts.iter().any(|(c,chart)| chart.range != c.parameters[chart.dependent.index()]),
                "the torus contact on a subdivision boundary needs an overlapping chart");
            let seam_charts: Vec<_> = charts.iter().filter(|(_,chart)| {
                let [a,b] = chart.range.bounds(); a < 0. || b > 1.
            }).collect();
            assert!(seam_charts.len() >= 4,"the full revolution needs charts through its seam");
            for u in [0.13,0.47,0.81] { for v in [0.,1.] {
                assert!(seam_charts.iter().any(|(c,_)| c.parameters.iter().zip([u,v,0.]).all(|(b,x)| b.contains(x))),
                    "regular seam contact must have a chart at {u}, {v}: {:?}",cover.cells.iter().filter(|c|
                        c.parameters.iter().zip([u,v,0.]).all(|(b,x)| b.contains(x))).collect::<Vec<_>>());
            } }
            let cad = Cad::new();
            for (cell,chart) in charts.iter().take(4).chain(seam_charts.into_iter().take(4)) {
                assert_eq!(chart.dependent,ContactParameter::Angle);
                assert!(sweep.at_chart(cell,f64::NAN,0.5,1e-10).is_err());
                let points: Vec<_> = (0..=16).flat_map(|i| {
                    let sweep = &sweep;
                    (0..=16).map(move |j| sweep.at_chart(cell,i as f64/16.,j as f64/16.,1e-10).unwrap().position)
                }).collect();
                let face = cad.fit(&points,17,17).unwrap();
                for (a,b) in [(0.13,0.27),(0.63,0.81),(0.41,0.53)] {
                    let p = cad.at(face,a,b).unwrap().0;
                    assert!(((p[0].hypot(p[1])-3.).hypot(p[2])-1.).abs() < 0.002);
                }
            }
            // Both analytic torus contact branches, including poles, must remain.
            for u in [0.13,0.47,0.81] { for t in [-1.,0.,1.] {
                for contact in sweep.at(0,u,t,1e-10).unwrap() {
                    assert!(cover.cells.iter().any(|c| !matches!(c.evidence,ContactEvidence::Excluded {..})
                        && c.parameters.iter().zip([u,contact.v,t]).all(|(b,x)| b.contains(x))));
                }
            } }
            for u in [0.,1.] { for v in [0.,0.37,0.81,1.] { for t in [-1.,0.,1.] {
                assert!(unresolved.iter().any(|c| c.parameters.iter().zip([u,v,t]).all(|(b,x)| b.contains(x))),
                    "a source pole must remain explicit");
            } } }
        }
        eprintln!("sphere source cover: {} cells, {} evaluations in {:?}",cover.cells.len(),cover.evaluations,start.elapsed());
    }
    assert!(sweep.cover(ContactCoverOptions {max_depth:49,max_cells:1}).is_err());
    assert!(sweep.cover(ContactCoverOptions {max_depth:1,max_cells:0}).is_err());
}

#[test]
fn meridian_charts_discover_the_rotated_sphere_equator_and_fit_the_same_torus() {
    let base = Path::new(env!("CARGO_MANIFEST_DIR")).join("../examples");
    let source = include_str!("../../../examples/solid_generating_sweep.sv")
        .replace("removal: GeneratingCut(tool,", "\
line tilt_axis(std.origin, std.front.toward)\n\
motion tilt(about: tilt_axis,phase: 90deg)\n\
solid placed(tool,under: tilt,at: 0deg)\n\
removal: GeneratingCut(placed,");
    let e = read(&source,&base);
    let sweep = SweepContacts::read(&e.sketch,e.map.ent_named("removal.body").unwrap().i(),1e-10).unwrap();
    let start = std::time::Instant::now();
    let cover = sweep.cover(ContactCoverOptions {max_depth:10,max_cells:10000}).unwrap();
    audit_partition(&sweep,&cover);
    let charts: Vec<_> = cover.cells.iter().filter(|c| matches!(c.evidence,
        ContactEvidence::Chart(chart) if chart.dependent == ContactParameter::Meridian)).collect();
    assert!(!charts.is_empty(),"the rotated sphere needs meridian charts");
    for v in [0.,0.13,0.47,0.81,1.] {
        assert!(charts.iter().any(|c| c.parameters.iter().zip([0.5,v,0.]).all(|(b,x)| b.contains(x))),
            "the regular equator must have a chart at v={v}");
    }
    let cad = Cad::new();
    for cell in charts.iter().take(4) {
        let points: Vec<_> = (0..=16).flat_map(|i| {
            let sweep = &sweep;
            (0..=16).map(move |j| sweep.at_chart(cell,i as f64/16.,j as f64/16.,1e-10).unwrap().position)
        }).collect();
        let face = cad.fit(&points,17,17).unwrap();
        for (a,b) in [(0.13,0.27),(0.63,0.81),(0.41,0.53)] {
            let expected = sweep.at_chart(cell,a,b,1e-10).unwrap();
            assert!(expected.normal_velocity.abs() < 1e-10);
            let p = cad.at(face,a,b).unwrap().0;
            assert!(((p[0].hypot(p[1])-3.).hypot(p[2])-1.).abs() < 0.002);
            assert!(distance(p,expected.position) < 0.002);
        }
    }
    for cell in &cover.cells { assert_eq!(cell.parameters[2].bounds(),sweep.domain()); }
    eprintln!("rotated sphere: {} meridian charts, {} cells, {} evaluations and native fits in {:?}",
        charts.len(),cover.cells.len(),cover.evaluations,start.elapsed());
}

#[test]
fn fixed_time_discovery_keeps_both_gear_endpoint_domains() {
    let base = Path::new(env!("CARGO_MANIFEST_DIR")).join("../examples/spiral_bevel");
    let e = read(&std::fs::read_to_string(base.join("gears.sv")).unwrap(),&base);
    let cad = Cad::new();
    for member in ["pinion","gear"] {
        let id = e.map.ent_named(&format!("pair.{member}.removal")).unwrap().i();
        let sweep = SweepContacts::read(&e.sketch,id,1e-10).unwrap();
        let caps = cad.0.sweep_caps(&e.sketch,id).unwrap();
        assert!(sweep.cover_at(f64::NAN,ContactCoverOptions {max_depth:1,max_cells:1}).is_err());
        assert!(sweep.cover_at(sweep.domain()[1]+1.,ContactCoverOptions {max_depth:1,max_cells:1}).is_err());
        let mut native_joined_cuts = 0;
        for cap in &caps.endpoints {
            let start = std::time::Instant::now();
            let cover = sweep.cover_at(cap.parameter,ContactCoverOptions {max_depth:20,max_cells:30000}).unwrap();
            let joined = sweep.join_contact_curves(cover,1e-10).unwrap();
            let cover = &joined.cover;
            audit_domain(&sweep,cover,[cap.parameter;2]);
            eprintln!("{member} endpoint {}: {} joined intervals from {} charts",cap.parameter,joined.curves.len(),
                joined.curves.iter().map(|c| c.cells.len()).sum::<usize>());
            let mut counts = [0;2];
            let mut incidence = 0;
            for cell in &cover.cells {
                assert_eq!(cell.parameters[2].bounds(),[cap.parameter;2]);
                let ContactEvidence::Chart(chart) = cell.evidence else { continue; };
                assert_ne!(chart.dependent,ContactParameter::Time);
                counts[chart.dependent.index()] += 1;
                let mut on_native = false;
                for a in [0.,0.25,0.5,0.75,1.] {
                    let contact = sweep.at_chart(cell,a,0.5,1e-10).unwrap();
                    assert!(contact.normal_velocity.abs() < 1e-10);
                    assert!(distance(contact.position,sweep.at_chart(cell,a,0.91,1e-10).unwrap().position) < 1e-12);
                    // A source chart can cross a narrow Boolean trim without
                    // its midpoint being on the native boundary. These samples
                    // witness incidence, not complete trim coverage.
                    if !on_native && cap.faces.iter().any(|&face| cad.0.face_parameters(face,contact.position,1e-6)
                        .is_ok_and(|p| p.is_some())) { on_native = true; }
                }
                if on_native { incidence += 1; }
            }
            eprintln!("{member} endpoint {}: {} meridian/{} angular charts, {incidence} native incidence samples, {} cells, {} evaluations in {:?}",
                cap.parameter,counts[0],counts[1],cover.cells.len(),cover.evaluations,start.elapsed());
            assert!(counts.iter().sum::<usize>() > 0 && incidence > 0);
            if let Some((faces,pieces,error)) = super::curves::split_joined_intervals(&cad,&sweep,&joined,&cap.faces,cap.pose) {
                eprintln!("{member} endpoint {}: {faces} native faces split from {pieces} contributing charts, withheld error {error:e} mm",cap.parameter);
                assert!(pieces > 1);
                native_joined_cuts += 1;
            }
            // Direct roots found independently of the partition must never be
            // excluded, including roots at the two physical motion endpoints.
            for patch in 0..sweep.patches().len() { for v in [0.13,0.47,0.81] {
                let Ok(roots) = sweep.at_angle(patch,v,cap.parameter,1e-10) else { continue; };
                for root in roots {
                    assert!(cover.cells.iter().any(|c| c.patch == patch
                        && !matches!(c.evidence,ContactEvidence::Excluded {..})
                        && c.parameters.iter().zip([root.u,v,cap.parameter]).all(|(b,x)| b.contains(x))));
                }
            } }
        }
        assert!(native_joined_cuts > 0,"{member}: joined intervals never reached native trimming");
    }
}

#[test]
fn automatic_contact_charts_of_both_source_members_fit_natively() {
    let base = Path::new(env!("CARGO_MANIFEST_DIR")).join("../examples/spiral_bevel");
    let e = read(&std::fs::read_to_string(base.join("gears.sv")).unwrap(),&base);
    let cad = Cad::new();
    let mut meridian_charts = 0;
    for member in ["pinion","gear"] {
        let id = e.map.ent_named(&format!("pair.{member}.removal")).unwrap().i();
        let sweep = SweepContacts::read(&e.sketch,id,1e-10).unwrap();
        let start = std::time::Instant::now();
        let cover = sweep.cover(ContactCoverOptions {max_depth:21,max_cells:30000}).unwrap();
        audit_partition(&sweep,&cover);
        let charts: Vec<_> = cover.cells.iter().filter(|c| matches!(c.evidence,ContactEvidence::Chart(_))).collect();
        let crosses_seam = |c: &gcs_core::solid::ContactCell| matches!(c.evidence,
            ContactEvidence::Chart(chart) if chart.dependent == ContactParameter::Angle
                && (chart.range.bounds()[0] < 0. || chart.range.bounds()[1] > 1.));
        eprintln!("{member}: {} charts, {} cells, {} evaluations in {:?}",charts.len(),cover.cells.len(),cover.evaluations,start.elapsed());
        assert!(!charts.is_empty());
        let seams = charts.iter().filter(|c| crosses_seam(c)).count();
        eprintln!("{member}: {seams} charts cross revolution seams");
        assert!(seams > 0);
        for dependent in [ContactParameter::Time,ContactParameter::Angle,ContactParameter::Meridian] {
            let count = charts.iter().filter(|c| matches!(c.evidence,ContactEvidence::Chart(chart) if chart.dependent == dependent)).count();
            eprintln!("{member}: {count} {dependent:?} charts");
            if dependent == ContactParameter::Meridian { meridian_charts += count; }
            else { assert!(count > 0); }
        }
        let mut contacts = 0;
        for patch in 0..sweep.patches().len() {
            let [u,v] = sweep.patches()[patch].domain();
            let initial = [u,v,sweep.domain()];
            // A subdivided pending cell also witnesses parent evaluation. A
            // breadth-first budget may leave every current leaf pending.
            assert!(cover.cells.iter().any(|c| c.patch == patch && (!matches!(c.evidence,
                ContactEvidence::Unresolved {value:None,..}) || c.parameters.map(|p| p.bounds()) != initial)),
                "every patch must receive work");
            for u in [0.13,0.47,0.81] { for t in [-0.3,0.,0.3] {
                let roots = match sweep.at(patch,u,t,1e-10) {
                    Ok(roots) => roots,
                    Err(gcs_core::envelope::Error::Degenerate) => continue,
                    Err(error) => panic!("{member} patch {patch}: {error:?}"),
                };
                for root in roots {
                    assert!(cover.cells.iter().any(|c| c.patch == patch
                        && !matches!(c.evidence,ContactEvidence::Excluded {..})
                        && c.parameters.iter().zip([u,root.v,t]).all(|(b,x)| b.contains(x))),
                        "{member} patch {patch}: contact lost at {u}, {}, {t}",root.v);
                    contacts += 1;
                }
            } }
        }
        assert!(contacts > 30);
        for dependent in [ContactParameter::Time,ContactParameter::Angle,ContactParameter::Meridian] {
        let mut selected: Vec<_> = charts.iter().filter(|c| matches!(c.evidence,ContactEvidence::Chart(chart)
            if chart.dependent == dependent)).collect();
        selected.sort_by_key(|c| !crosses_seam(c));
        for cell in selected.into_iter().take(4) {
            let at = |a,b| sweep.at_chart(cell,a,b,1e-10).unwrap().position;
            let mut worst = f64::INFINITY;
            for n in [8,16,32] {
                let points: Vec<_> = (0..=n).flat_map(|i| {
                    let at = &at;
                    (0..=n).map(move |j| at(i as f64/n as f64,j as f64/n as f64))
                }).collect();
                let face = cad.fit(&points,n+1,n+1).unwrap();
                worst = [(0.13,0.27),(0.63,0.81),(0.41,0.53)].into_iter().map(|(u,v)|
                    distance(cad.at(face,u,v).unwrap().0,at(u,v))).fold(0.,f64::max);
                if worst < 0.002 { break; }
            }
            assert!(worst < 0.002,"{member}: automatic {dependent:?} chart fit error {worst}");
        } }
    }
    assert!(meridian_charts > 0,"the pair must exercise automatic meridian fits");
}
