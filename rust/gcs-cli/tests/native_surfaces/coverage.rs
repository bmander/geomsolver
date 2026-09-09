//! Automatic contact-domain discovery feeds the same native fitting primitive.
use super::*;
use gcs_core::solid::{ContactCover,ContactCoverOptions,ContactEvidence,ContactLimit,ContactParameter};

fn audit_partition(sweep: &SweepContacts,cover: &ContactCover) {
    for (patch,surface) in sweep.patches().iter().enumerate() {
        let [u,v] = surface.domain();
        let original = [u,v,sweep.domain()];
        let cells: Vec<_> = cover.cells.iter().filter(|c| c.patch == patch).collect();
        let mut volume = 0.;
        for cell in &cells {
            volume += cell.parameters.iter().zip(original).map(|(p,o)| {
                let [a,b] = p.bounds();
                assert!(a >= o[0] && b <= o[1]);
                (b-a)/(o[1]-o[0])
            }).product::<f64>();
            if let ContactEvidence::Excluded {value} = cell.evidence { assert!(!value.contains(0.)); }
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
fn automatic_contact_charts_of_both_source_members_fit_natively() {
    let base = Path::new(env!("CARGO_MANIFEST_DIR")).join("../examples/spiral_bevel");
    let e = read(&std::fs::read_to_string(base.join("gears.sv")).unwrap(),&base);
    let cad = Cad::new();
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
        for dependent in [ContactParameter::Time,ContactParameter::Angle] {
            let count = charts.iter().filter(|c| matches!(c.evidence,ContactEvidence::Chart(chart) if chart.dependent == dependent)).count();
            eprintln!("{member}: {count} {dependent:?} charts");
            assert!(count > 0);
        }
        let mut contacts = 0;
        for patch in 0..sweep.patches().len() {
            assert!(cover.cells.iter().any(|c| c.patch == patch && !matches!(c.evidence,
                ContactEvidence::Unresolved {value:None,..})),"every patch must receive work");
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
        for dependent in [ContactParameter::Time,ContactParameter::Angle] {
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
}
