//! Automatic contact-domain discovery feeds the same native fitting primitive.
use super::*;
use gcs_core::solid::{ContactCover,ContactCoverOptions,ContactEvidence,ContactLimit};

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
        assert!(cover.evaluations <= budget);
        // A single rotation's equation is independent of time. Its contact set
        // requires an angular chart; time subdivision must not duplicate work.
        for cell in &cover.cells { assert_eq!(cell.parameters[2].bounds(),sweep.domain()); }
        if budget == 1 {
            assert!(unresolved.iter().any(|c| matches!(c.evidence,ContactEvidence::Unresolved {limit:ContactLimit::Budget,..})));
        } else {
            assert!(!unresolved.iter().any(|c| matches!(c.evidence,ContactEvidence::Unresolved {limit:ContactLimit::Budget,..})));
            assert!(cover.cells.iter().any(|c| matches!(c.evidence,ContactEvidence::Excluded {..})));
            // Both analytic torus contact branches, including poles, must remain.
            for u in [0.13,0.47,0.81] { for t in [-1.,0.,1.] {
                for contact in sweep.at(0,u,t,1e-10).unwrap() {
                    assert!(unresolved.iter().any(|c| c.parameters.iter().zip([u,contact.v,t]).all(|(b,x)| b.contains(x))));
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
fn automatic_temporal_charts_of_both_source_members_fit_natively() {
    let base = Path::new(env!("CARGO_MANIFEST_DIR")).join("../examples/spiral_bevel");
    let e = read(&std::fs::read_to_string(base.join("gears.sv")).unwrap(),&base);
    let cad = Cad::new();
    for member in ["pinion","gear"] {
        let id = e.map.ent_named(&format!("pair.{member}.removal")).unwrap().i();
        let sweep = SweepContacts::read(&e.sketch,id,1e-10).unwrap();
        let start = std::time::Instant::now();
        let cover = sweep.cover(ContactCoverOptions {max_depth:21,max_cells:30000}).unwrap();
        audit_partition(&sweep,&cover);
        let charts: Vec<_> = cover.cells.iter().filter(|c| matches!(c.evidence,ContactEvidence::TemporalChart {..})).collect();
        eprintln!("{member}: {} charts, {} cells, {} evaluations in {:?}",charts.len(),cover.cells.len(),cover.evaluations,start.elapsed());
        assert!(!charts.is_empty());
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
        for cell in charts.iter().take(4) {
            let at = |u: f64,v: f64| {
                let map = |i: usize,s: f64| { let [a,b] = cell.parameters[i].bounds(); a+(b-a)*s };
                let roots: Vec<_> = sweep.at_source(cell.patch,map(0,u),map(1,v),1e-10).unwrap()
                    .into_iter().filter(|c| cell.parameters[2].contains(c.root.time)).collect();
                assert_eq!(roots.len(),1);
                roots[0].contact.position
            };
            let points: Vec<_> = (0..=8).flat_map(|i| {
                let at = &at;
                (0..=8).map(move |j| at(i as f64/8.,j as f64/8.))
            }).collect();
            let face = cad.fit(&points,9,9).unwrap();
            for (u,v) in [(0.13,0.27),(0.63,0.81),(0.41,0.53)] {
                let error = distance(cad.at(face,u,v).unwrap().0,at(u,v));
                assert!(error < 0.002,"{member}: automatic chart fit error {error}");
            }
        }
    }
}
