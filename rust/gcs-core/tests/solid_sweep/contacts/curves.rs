use super::*;
use gcs_core::solid::{ContactCoverOptions,ContactEvidence,ContactLimit,ContactParameter};

fn sphere(rotated: bool) -> SweepContacts {
    let extra = if rotated { "point right hint(x: 1,y: 0)\nground right\n\
        line tilt_axis(o,right)\nmotion tilt(about: tilt_axis,phase: 90deg)\n\
        solid tilted(tool,under: tilt,at: 0deg)\n" } else { "" };
    let source = if rotated { "tilted" } else { "tool" };
    let e = read(&format!("{SOURCE}\n{extra}\nsolid swept({source},under: generating,from: -60deg,to: 60deg)\n"));
    SweepContacts::read(&e.sketch,e.map.ent_named("swept").unwrap().i(),1e-10).unwrap()
}
const OPTIONS: ContactCoverOptions = ContactCoverOptions {max_depth:12,max_cells:12000};

#[test]
fn touching_charts_join_without_merging_distinct_contact_branches() {
    for rotated in [false,true] {
        let sweep = sphere(rotated);
        let cover = sweep.cover_at(0.,OPTIONS).unwrap();
        let cells = cover.cells.len();
        let joined = sweep.join_contact_curves(cover,1e-10).unwrap();
        assert_eq!(joined.cover.cells.len(),cells);
        assert_eq!(joined.curves.len(),if rotated { 1 } else { 2 });
        for curve in &joined.curves {
            assert!(curve.cells.len() > 1);
            for i in 0..=96 {
                let p = sweep.at_contact_curve(curve,i as f64/96.,1e-10).unwrap();
                assert!(p.contact.normal_velocity.abs() < 1e-10);
                let [x,y,z] = p.contact.position;
                assert!(y.abs() < 1e-12,"the contact circle lies in the XZ plane");
                assert!(((x-3.).hypot(z)-1.).abs() < 1e-12);
            }
        }
        if rotated {
            assert_eq!(joined.curves[0].dependent,ContactParameter::Meridian);
            assert_eq!(joined.curves[0].range,[0.,1.]);
        } else {
            assert_ne!(joined.curves[0].branch,joined.curves[1].branch);
        }
        let mut reversed = joined.cover.clone(); reversed.cells.reverse();
        let reordered = sweep.join_contact_curves(reversed,1e-10).unwrap();
        let descriptions = |curves: &[gcs_core::solid::ContactCurve]| curves.iter()
            .map(|c| (c.patch,c.dependent,c.branch,c.range,c.cells.len())).collect::<Vec<_>>();
        assert_eq!(descriptions(&joined.curves),descriptions(&reordered.curves));
    }
}

#[test]
fn unresolved_intervals_remain_gaps_in_joined_curves() {
    let sweep = sphere(true);
    let mut cover = sweep.cover_at(0.,OPTIONS).unwrap();
    for cell in &mut cover.cells {
        if let ContactEvidence::Chart(chart) = cell.evidence {
            let [lo,hi] = cell.parameters[chart.dependent.free()[0]].bounds();
            if lo < 0.625 && hi > 0.375 {
                cell.evidence = ContactEvidence::Unresolved {value:None,limit:ContactLimit::Budget};
            }
        }
    }
    let unresolved = cover.cells.iter().filter(|c| matches!(c.evidence,ContactEvidence::Unresolved {..})).count();
    let joined = sweep.join_contact_curves(cover,1e-10).unwrap();
    assert_eq!(joined.curves.len(),2);
    assert!(joined.curves[0].range[1] < joined.curves[1].range[0]);
    assert_eq!(joined.cover.cells.iter().filter(|c| matches!(c.evidence,ContactEvidence::Unresolved {..})).count(),unresolved);
    for s in [-0.1,1.1,f64::NAN] { assert!(sweep.at_contact_curve(&joined.curves[0],s,1e-10).is_err()); }
    assert!(sweep.join_contact_curves(joined.cover.clone(),0.).is_err());
    assert!(sweep.join_contact_curves(sweep.cover(OPTIONS).unwrap(),1e-10).is_err());
}
