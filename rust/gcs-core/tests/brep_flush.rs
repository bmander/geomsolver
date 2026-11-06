//! **An end flush with a face is on it, and a sliver is said wherever it stands** (#836): a body's
//! Boolean moves an operand's planar face standing within `flush::FLUSH` of its tolerances of a
//! parallel plane of the solid onto that plane, exactly, where the face's walls run along its
//! normal (`flush::flushed`), so no strip is left between them; and what the B-rep holds thinner
//! than a bar — a face somewhere, an edge — is found by its boundary, not by where it stands
//! (`flush::slivers`).
use gcs_core::brep::boolean::{combined,Op};
use gcs_core::brep::build::{prism,revolve,Profile,ProfileEdge};
use gcs_core::brep::flush::{flushed,onto,slivers,Thin,FLUSH};
use gcs_core::brep::geom::{tolerance,Frame,V};
use gcs_core::brep::props::volume;
use gcs_core::brep::topo::Brep;
use std::f64::consts::{PI,TAU};

const XY: V = [0.,0.,1.];

fn poly(pts: &[V]) -> Vec<ProfileEdge> {
    (0..pts.len()).map(|i| ProfileEdge::Line {a:pts[i],b:pts[(i+1)%pts.len()]}).collect()
}
fn block(lo: V,hi: V) -> Brep {
    let p = Profile {names:vec![],origin:[0.;3],normal:XY,loops:vec![poly(&[[lo[0],lo[1],0.],[hi[0],lo[1],0.],[hi[0],hi[1],0.],[lo[0],hi[1],0.]])]};
    prism(&p,lo[2],hi[2]).unwrap()
}
fn rod(c: V,r: f64,z: [f64;2]) -> Brep {
    let arc = ProfileEdge::Arc {frame:Frame::new([c[0],c[1],0.],XY,[1.,0.,0.]),r,span:None};
    prism(&Profile {names:vec![],origin:[0.;3],normal:XY,loops:vec![vec![arc]]},z[0],z[1]).unwrap()
}
fn close(a: f64,b: f64,within: f64) { assert!((a-b).abs() <= within,"{a} against {b}"); }

/// A 4 mm block with a slot 3 × 0.4 cut up into it from below to z = 2: its ceiling the plane an
/// end cut up across it may stand all but flush with.
fn slotted() -> Brep { combined(&block([0.,0.,0.],[4.,4.,4.]),&block([0.5,1.8,-1.],[3.5,2.2,2.]),Op::Cut,0.).unwrap() }

/// **An end within reach of a plane of the solid is moved onto it, exactly** (#836): a bore and a
/// pocket cut up across the slot to its ceiling give or take a few nanometres — inside a thousand
/// of the Boolean's tolerances, a part in a million of the solid's size — have their ends moved
/// onto the ceiling, their walls (a cylinder, planes) stretched or shortened to them, every corner
/// on it, and the operand still a closed boundary of the volume it now holds; the cut is
/// the cut of an end exactly on the ceiling. An end already on it, or farther than the reach, is
/// left.
#[test]
fn an_end_within_reach_of_a_face_is_moved_onto_it() {
    let solid = slotted();
    let tol = tolerance(solid.size());
    let reach = FLUSH*tol;
    let flush = |z: f64| [rod([2.,2.,0.],0.5,[-1.,z]),block([1.5,0.5,-1.],[2.5,3.5,z])];
    for (k,flush_end) in flush(2.).into_iter().enumerate() {
        let want = combined(&solid,&flush_end,Op::Cut,0.).unwrap();
        assert!(flushed(&solid,&flush_end,tol).is_none(),"an end on the ceiling already stays");
        for off in [1e-8,-1e-8,0.5*reach,-0.5*reach] {
            let operand = &flush(2.+off)[k];
            let moved = flushed(&solid,operand,tol).unwrap_or_else(|| panic!("{k}: an end {off:e} off the ceiling is moved"));
            moved.check(1e-9).unwrap();
            // every corner that stood at the end stands on the ceiling
            for v in &moved.vertices { assert!(v.p[2] == -1. || (v.p[2]-2.).abs() <= 1e-15,"{k} {off:e}: {:?}",v.p); }
            close(volume(&moved),volume(&flush_end),1e-12);
            let cut = combined(&solid,&moved,Op::Cut,0.).unwrap();
            cut.check(1e-8).unwrap();
            assert_eq!((cut.faces.len(),cut.edges.len()),(want.faces.len(),want.edges.len()),"{k} {off:e}");
            close(volume(&cut),volume(&want),1e-12);
            assert!(onto(&solid,operand,0.).is_some(),"the body's Boolean reads it at its own tolerance");
        }
        // drawn apart by more than the reach: as drawn
        assert!(flushed(&solid,&flush(2.+2.*reach)[k],tol).is_none());
        assert!(flushed(&solid,&flush(2.+1e-3)[k],tol).is_none());
    }
}

/// **An end whose walls lean stays where it is**: a cone's flat end within reach of the ceiling is
/// not moved — moving it would move the circle it meets the cone in along the cone, not along the
/// end's normal — and the sliver it leaves is the export's to say (`slivers`).
#[test]
fn an_end_whose_walls_lean_is_left_where_it_is() {
    let solid = slotted();
    let tol = tolerance(solid.size());
    let top = 2.+1e-8;
    let p = Profile {names:vec![],origin:[2.,2.,0.],normal:[0.,1.,0.],loops:vec![poly(&[[2.,2.,-1.],[2.4,2.,-1.],[2.6,2.,top],[2.,2.,top]])]};
    let cone = revolve(&p,[2.,2.,0.],XY,TAU).unwrap();
    assert!(flushed(&solid,&cone,tol).is_none());
}

/// **A sliver is found by the boundary, wherever it stands** (#836): a bore cut up across the slot
/// to 1 µm over its ceiling leaves its wall a band 1 µm tall where it crosses the slot — no edge of
/// it short, its rims far apart along its boundary — which a bar of 10 µm finds
/// and one of 0.5 µm does not, the same a kilometre out as at the origin. A plain block, a disc
/// 2 mm thick and a wedge with a 1° corner — thin near its tip, as every corner is — hold none;
/// the same disc 5 µm thick is a sliver all
/// round its rim.
#[test]
fn a_sliver_is_found_by_its_boundary_wherever_it_stands() {
    use gcs_core::brep::geom::Rigid;
    let stepped = combined(&slotted(),&rod([2.,2.,0.],0.5,[-1.,2.001]),Op::Cut,0.).unwrap();
    stepped.check(1e-8).unwrap();
    for at in [0.,1e6] {
        let b = stepped.moved(&Rigid {t:[at,0.,at],..Rigid::identity()});
        let found = slivers(&b,0.01);
        assert!(!found.is_empty());
        for s in &found {
            let Thin::Face(f) = s.thin else { panic!("{s:?}: the bore's wall is thin") };
            assert!(matches!(b.faces[f].surface,gcs_core::brep::geom::Surface::Cylinder(..)),"{s:?}");
        }
        let s = &found[0];
        close(s.size,1e-3,1e-9);
        // on the wall, midway up the band, beside the slot's ceiling and the bore's end
        close(((s.at[0]-at-2.).powi(2)+(s.at[1]-2.).powi(2)).sqrt(),0.5,1e-9);
        close(s.at[2]-at,2.0005,1e-6);
        assert_eq!(s.beside.len(),2,"{s:?}");
        assert!(slivers(&b,5e-4).is_empty());
    }
    assert!(slivers(&block([0.;3],[4.,4.,4.]),0.01).is_empty());
    assert!(slivers(&rod([0.;3],5.,[0.,2.]),0.01).is_empty());
    let wedge = |h: f64| {
        let tip = 1f64.to_radians()/2.;
        let p = Profile {names:vec![],origin:[0.;3],normal:XY,loops:vec![poly(&[[0.,0.,0.],[10.,-10.*tip.tan(),0.],[10.,10.*tip.tan(),0.]])]};
        prism(&p,0.,h).unwrap()
    };
    assert!(slivers(&wedge(1.),0.01).is_empty(),"{:?}",slivers(&wedge(1.),0.01));
    let thin = slivers(&rod([0.;3],5.,[0.,0.005]),0.01);
    assert_eq!(thin.len(),1,"{thin:?}");
    assert!(matches!(thin[0].thin,Thin::Face(_)));
    close(thin[0].size,0.005,1e-9);
    // a whole turn's volume: the disc is no less a solid for being thin
    close(volume(&rod([0.;3],5.,[0.,0.005])),PI*25.*0.005,1e-12);
}
