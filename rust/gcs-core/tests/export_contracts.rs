//! The export's refusal vocabulary (`solid::export`) and the contracts a kernel's output answers
//! to (`solid::contracts`): the stage keys a harness records, and the mesh and cell judgements.
use gcs_core::solid::{admission,contracts::{self,CellVolume},export::{AtStage,ExportRefusal,Stage}};

#[test]
fn every_stage_key_reads_back_and_the_order_is_the_native_export() {
    for s in Stage::ORDER.into_iter().chain([Stage::Refine]) { assert_eq!(Stage::from_key(s.key()),Some(s)); }
    assert!(Stage::ORDER.windows(2).all(|w| w[0] < w[1]));
    assert!(!Stage::ORDER.contains(&Stage::Refine));
    assert_eq!(Stage::Admission.owner(),Some("class"));
    assert_eq!(Stage::Agreement.owner(),Some("gate"));
    assert_eq!(Stage::Written.owner(),None);
    assert_eq!(Stage::from_key("nothing"),None);
}

#[test]
fn a_refusal_says_its_message_and_keeps_its_stage() {
    let r: Result<(),ExportRefusal> = Err::<(),_>("the split removed 1 cells for 2 placements").at(Stage::Classify);
    let r = r.unwrap_err();
    assert_eq!((r.stage,r.to_string().as_str()),(Stage::Classify,"the split removed 1 cells for 2 placements"));
    let refused = admission::Error::Refused(admission::Refusal {condition:admission::Condition::Clearance,
        sweep:"removal".into(),message:"the tool lies in the blank".into(),witness:Some([1.,2.,3.])});
    let text = refused.to_string();
    let r = ExportRefusal::from(refused);
    assert_eq!((r.stage,r.condition,r.witness),(Stage::Admission,Some(admission::Condition::Clearance),Some([1.,2.,3.])));
    assert_eq!(r.message,text);
}

#[test]
fn the_mesh_contract_refuses_a_cluster_of_microscopic_triangles() {
    // A fan of slivers about the origin, each well under a square micrometre at 1 mm a unit.
    let mut vertices = vec![[0.;3]];
    let mut triangles = Vec::new();
    for k in 0..=150 { vertices.push([1e-4*(k as f64).cos(),1e-4*(k as f64).sin(),0.]); }
    for k in 1..151u32 { triangles.push([0,k,k+1]); }
    let few = contracts::tiny_triangles(&vertices,&triangles[..contracts::MOST_TINY],1.);
    assert_eq!(few.count,contracts::MOST_TINY);
    assert!(few.clustered().is_ok());
    let many = contracts::tiny_triangles(&vertices,&triangles,1.);
    assert_eq!((many.count,many.total),(150,150));
    assert!(many.clustered().unwrap_err().contains("crumpled or folded"));
    // At a thousand millimetres a unit the same triangles are large.
    assert_eq!(contracts::tiny_triangles(&vertices,&triangles,1e3).count,0);
}

/// The contract is about clusters: slivers strung along a mesh's edges, a few in each millimetre,
/// pass however many there are in all; the same count in one millimetre is a crumpled patch.
#[test]
fn the_mesh_contract_refuses_a_cluster_not_a_count() {
    let sliver = |x: f64| [[x,0.,0.],[x+1e-3,0.,0.],[x,1e-5,0.]];
    let strung: Vec<[[f64;3];3]> = (0..3*contracts::MOST_TINY).map(|k| sliver(0.1+k as f64)).collect();
    let bunched: Vec<[[f64;3];3]> = (0..3*contracts::MOST_TINY).map(|k| sliver(0.1+k as f64*1e-3)).collect();
    let mesh = |t: &[[[f64;3];3]]| -> (Vec<[f64;3]>,Vec<[u32;3]>) {
        (t.iter().flatten().copied().collect(),(0..t.len() as u32).map(|k| [3*k,3*k+1,3*k+2]).collect())
    };
    let (v,t) = mesh(&strung);
    let spread = contracts::tiny_triangles_under(&v,&t,1.,1e-7);
    assert_eq!((spread.count,spread.densest),(3*contracts::MOST_TINY,1));
    assert!(spread.clustered().is_ok());
    let (v,t) = mesh(&bunched);
    let cluster = contracts::tiny_triangles_under(&v,&t,1.,1e-7);
    assert_eq!(cluster.densest,3*contracts::MOST_TINY);
    assert!(cluster.clustered().unwrap_err().contains("crumpled or folded"));
    // Over the area limit, nothing counts.
    assert_eq!(contracts::tiny_triangles_under(&v,&t,1.,1e-9).count,0);
}

#[test]
fn the_cell_contract_wants_one_congruent_removed_cell_per_placement() {
    let cell = |volume: f64| CellVolume {volume,point:[0.;3]};
    assert!(contracts::cells(&[cell(10.)],&[cell(1.),cell(1.)],&[2]).is_ok());
    assert!(contracts::cells(&[cell(10.)],&[cell(1.)],&[2]).unwrap_err().contains("removed 1 cells for 2"));
    assert!(contracts::cells(&[cell(10.)],&[cell(1.),cell(1.1)],&[2]).unwrap_err().contains("not congruent"));
    assert!(contracts::cells(&[cell(1e-4)],&[cell(1.)],&[1]).unwrap_err().contains("floor"));
    // Two sweeps at the same two indices (a crown and its relief): classes of one cell an index.
    let removed = [cell(1.),cell(0.2),cell(1.),cell(0.2),cell(0.5),cell(0.5)];
    assert!(contracts::cells(&[cell(10.)],&removed,&[2,2]).is_ok());
    assert!(contracts::cells(&[cell(10.)],&removed[..5],&[2,2]).unwrap_err().contains("1 of 0.5"));
    assert!(contracts::cells(&[cell(10.)],&removed,&[2,3]).unwrap_err().contains("one count"));
}
