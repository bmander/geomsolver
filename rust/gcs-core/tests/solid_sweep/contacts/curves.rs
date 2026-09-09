use super::*;
use gcs_core::solid::{ContactCoverOptions,ContactEvidence,ContactLimit,ContactParameter};

fn sphere(rotated: bool) -> SweepContacts {
    sphere_at_tilt(if rotated { 90. } else { 0. })
}
fn sphere_at_tilt(degrees: f64) -> SweepContacts {
    let extra = format!("point right hint(x: 1,y: 0)\nground right\n\
        line tilt_axis(o,right)\nmotion tilt(about: tilt_axis,phase: {degrees}deg)\n\
        solid tilted(tool,under: tilt,at: 0deg)\n");
    let source = "tilted";
    let e = read(&format!("{SOURCE}\n{extra}\nsolid swept({source},under: generating,from: -60deg,to: 60deg)\n"));
    SweepContacts::read(&e.sketch,e.map.ent_named("swept").unwrap().i(),1e-10).unwrap()
}
const OPTIONS: ContactCoverOptions = ContactCoverOptions {max_depth:12,max_cells:12000};

#[test]
fn alternate_contact_charts_connect_through_shared_root_enclosures() {
    for angle in [15.,35.,65.] {
        let sweep = sphere_at_tilt(angle);
        let cover = sweep.cover_at(0.,ContactCoverOptions {max_depth:20,max_cells:30000}).unwrap();
        let mut joined = sweep.join_contact_curves(cover,1e-10).unwrap();
        let evaluations = joined.cover.evaluations;
        let key = |c: &gcs_core::solid::ContactCell| (c.patch,c.parameters.map(|p| p.bounds().map(f64::to_bits)));
        let retained: std::collections::BTreeSet<_> = joined.cover.cells.iter()
            .filter(|c| !matches!(c.evidence,ContactEvidence::Unresolved {..})).map(key).collect();
        let cover = sweep.refine_contact_ends(joined,ContactCoverOptions {max_depth:8,max_cells:2000}).unwrap();
        assert!(cover.evaluations-evaluations <= 2000);
        if angle == 35. { assert!(cover.evaluations > evaluations,"the gap needs additional source evidence"); }
        let after: std::collections::BTreeSet<_> = cover.cells.iter().map(key).collect();
        assert!(retained.is_subset(&after),"existing evidence must be retained");
        let area: f64 = cover.cells.iter().map(|c| {
            let [u,v,_] = c.parameters.map(|p| p.bounds()); (u[1]-u[0])*(v[1]-v[0])
        }).sum();
        assert!((area-1.).abs() < 1e-12);
        joined = sweep.join_contact_curves(cover,1e-10).unwrap();
        let transitions = sweep.contact_transitions(&joined).unwrap();
        eprintln!("tilt {angle}: {} curves, {} transitions",joined.curves.len(),transitions.iter().flatten().flatten().count());
        let mut checked = 0;
        for (i,ends) in transitions.iter().enumerate() { for (end,link) in ends.iter().enumerate() {
            let Some(link) = link else { continue; };
            let from = &joined.curves[i]; let to = &joined.curves[link.curve];
            assert_ne!(from.dependent,to.dependent);
            let p = sweep.at_contact_curve(from,end as f64,1e-10).unwrap();
            let [a,b] = link.coordinate.bounds();
            assert!(b-a < 1e-9);
            let value = a*0.5+b*0.5;
            let s = (value-to.range[0])/(to.range[1]-to.range[0]);
            let q = sweep.at_contact_curve(to,s,1e-10).unwrap();
            for k in 0..3 { assert!((p.contact.position[k]-q.contact.position[k]).abs() < 1e-9); }
            // Independent sphere geometry and an outgoing point distinguish a
            // continuation from reversing along the already traversed curve.
            let toward = if link.forward { 1. } else { 0. };
            let next = sweep.at_contact_curve(to,s+(toward-s)*1e-4,1e-10).unwrap();
            let previous = sweep.at_contact_curve(from,if end == 0 { 1e-6 } else { 1.-1e-6 },1e-10).unwrap();
            let dot: f64 = (0..3).map(|k| (p.contact.position[k]-previous.contact.position[k])
                *(next.contact.position[k]-q.contact.position[k])).sum();
            assert!(dot > 0.);
            let [x,y,z] = next.contact.position;
            assert!(y.abs() < 1e-12 && ((x-3.).hypot(z)-1.).abs() < 1e-12);
            checked += 1;
        } }
        assert!(checked >= 4);
        let mut ends = Vec::new();
        for (i,links) in transitions.iter().enumerate() {
            let curve = &joined.curves[i];
            let domain = sweep.patches()[curve.patch].domain()[curve.dependent.free()[0]];
            for (end,link) in links.iter().enumerate() {
                if link.is_none() && curve.range[end] == domain[end] { ends.push((i,end == 0)); }
            }
        }
        for forward in [false,true] {
            // A parameter-seam endpoint starts the open representation of the
            // same spatial circle. Otherwise trace from either direction in a loop.
            let (start,direction) = if ends.is_empty() { (0,forward) }
                else { ends[if forward { 0 } else { ends.len()-1 }] };
            let path = sweep.trace_contact_path(&joined,start,direction,1e-10,1e-8).unwrap();
            eprintln!("tilt {angle}: {} path segments, closed={}",path.segments.len(),path.closed);
            assert_eq!(path.closed,ends.is_empty());
            assert!(path.segments.len() >= 2);
            assert!(path.join_error < 1e-9);
            let mut points = Vec::new();
            for segment in &path.segments {
                for i in 0..=32 {
                    let s = segment.range[0]+(segment.range[1]-segment.range[0])*i as f64/32.;
                    let p = sweep.at_contact_curve(&joined.curves[segment.curve],s,1e-10).unwrap().contact.position;
                    assert!(p[1].abs() < 1e-12 && ((p[0]-3.).hypot(p[2])-1.).abs() < 1e-12);
                    points.push(p);
                }
            }
            let first = points[0]; let last = points[points.len()-1];
            assert!((0..3).all(|k| (first[k]-last[k]).abs() < 1e-9));
            let mut angle = 0.;
            for pair in points.windows(2) {
                let a = [pair[0][0]-3.,pair[0][2]]; let b = [pair[1][0]-3.,pair[1][2]];
                angle += (a[0]*b[1]-a[1]*b[0]).atan2(a[0]*b[0]+a[1]*b[1]);
            }
            assert!((angle.abs()-2.*PI).abs() < 1e-9,"the path must cover the circle exactly once: {angle}");
            let seed = path.segments[path.segments.len()/2].curve;
            let component = sweep.trace_contact_component(&joined,seed,1e-10,1e-8).unwrap();
            let first = component.segments.first().unwrap(); let last = component.segments.last().unwrap();
            let a = sweep.at_contact_curve(&joined.curves[first.curve],first.range[0],1e-10).unwrap().contact.position;
            let b = sweep.at_contact_curve(&joined.curves[last.curve],last.range[1],1e-10).unwrap().contact.position;
            assert!((0..3).all(|k| (a[k]-b[k]).abs() < 1e-9),"both ends of an interior seed must be traced");
            assert_eq!(component.segments.iter().filter(|s| s.curve == seed).count(),1,"the seed interval must not be duplicated");
        }
    }
}

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
    assert!(sweep.contact_transitions(&joined).unwrap().iter().flatten().all(Option::is_none));
    let path = sweep.trace_contact_path(&joined,0,true,1e-10,1e-8).unwrap();
    assert!(!path.closed && path.segments.len() == 1,"a path cannot bridge the missing interval");
    for s in [-0.1,1.1,f64::NAN] { assert!(sweep.at_contact_curve(&joined.curves[0],s,1e-10).is_err()); }
    assert!(sweep.join_contact_curves(joined.cover.clone(),0.).is_err());
    assert!(sweep.join_contact_curves(sweep.cover(OPTIONS).unwrap(),1e-10).is_err());
}
