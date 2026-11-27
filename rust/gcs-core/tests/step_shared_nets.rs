//! **The faces on one sheet's net that share no edge are written on one surface**
//! (`step::face_nets`): each face written on its own stretch of the net its window's faces lie on
//! (#1013) wrote the poles where their stretches overlap once a face. A block under a gentle patch
//! with a slot cut down across its top, short of its floor, leaves its top two faces on one net
//! (`Arc::ptr_eq`) sharing no edge, one lump: they are written on one surface, the file reads back
//! to the same solid (`step_roundtrip::trip`), and no two faces on one written surface share an
//! edge (on a sheet's double curve the edge's two pcurves lie on two layers of the net, and only
//! their surfaces tell a reader which is whose). Cut through, the block is two lumps, and each
//! solid is read on its own surfaces: nothing is shared between them.
use crate::patch_layers::{rising,under};
use crate::step_roundtrip::{trip,Outcome};
use gcs_core::brep::boolean::{boolean,Op};
use gcs_core::brep::build::block;
use gcs_core::brep::geom::Surface;
use gcs_core::brep::step::{face_nets,write,Fit};
use std::collections::BTreeSet;
use std::sync::Arc;

#[test]
fn faces_on_one_net_sharing_no_edge_are_written_on_one_surface() {
    // the graph of z = 1/2 + x²/5 + y/10 over the unit square, its column slotted across its top
    let patch = rising(&[0.,1./3.,2./3.,1.],&[0.5,0.5,0.5+1./15.,0.7]);
    let column = under(&patch,[0.,1.],[0.,1.],false);
    let slotted = |floor: f64| boolean(&column,&block([-1.,0.4,floor],[2.,0.6,3.]).unwrap(),Op::Cut,1e-7).unwrap();
    let b = slotted(0.3);
    b.check(1e-7).unwrap();
    let nets = face_nets(&b);
    let edges = |fi: usize| b.faces[fi].loops.iter().flatten().map(|c| c.edge).collect::<BTreeSet<u32>>();
    let mut pairs = 0;
    for i in 0..b.faces.len() { for j in i+1..b.faces.len() {
        let (Surface::BSpline(_,m),Surface::BSpline(_,n)) = (&b.faces[i].surface,&b.faces[j].surface) else { continue };
        let (Some(Ok(p)),Some(Ok(q))) = (&nets[i],&nets[j]) else { panic!("faces {i} and {j} written as no net") };
        let shared = Arc::ptr_eq(&p.net,&q.net);
        let touch = !edges(i).is_disjoint(&edges(j));
        assert!(!(shared && touch),"faces {i} and {j} share an edge and one surface");
        if Arc::ptr_eq(m,n) {
            assert!(shared || touch,"faces {i} and {j} on one net, sharing no edge, written on two surfaces");
            pairs += 1;
        }
    } }
    assert_eq!(pairs,1,"the top's halves");
    // one surface written for each pair
    let text = write(&b,"slotted",1e-7,Fit::Exact).unwrap();
    let written = text.matches("B_SPLINE_SURFACE_WITH_KNOTS(").count();
    let sheets = b.faces.iter().filter(|f| matches!(f.surface,Surface::BSpline(..))).count();
    assert_eq!(written,sheets-pairs,"{sheets} sheet faces in {written} surfaces");
    // and the file reads back to the same solid
    match trip(&b,"slotted",1e-7,Fit::Exact).0 {
        Outcome::Same => {}
        other => panic!("not read back the same: {other:?}"),
    }
    // cut through: two lumps, the top's, the floor's and two walls' halves each on one net, none
    // written on one surface with its other half
    let c = slotted(-1.);
    assert_eq!(gcs_core::brep::step::written_lumps(&c).len(),2);
    let nets = face_nets(&c);
    for i in 0..c.faces.len() { for j in i+1..c.faces.len() {
        if let (Some(Ok(p)),Some(Ok(q))) = (&nets[i],&nets[j]) { assert!(!Arc::ptr_eq(&p.net,&q.net),"faces {i} and {j} of two lumps on one surface") }
    } }
    match trip(&c,"cut through",1e-7,Fit::Exact).0 {
        Outcome::Same => {}
        other => panic!("cut through: not read back the same: {other:?}"),
    }
}

