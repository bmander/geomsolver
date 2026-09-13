//! Source ownership through geometric edits, checked against independent parent
//! domains rather than the current position of an entry in the sheet array.
use gcs_core::solid::swept_boundary::{self as sb,KeptMesh,Rim};
type P = [f64;3];

fn fixture() -> (KeptMesh,Vec<Rim>) {
    // The first triangle collapses. The following two sources touch at vertex 6,
    // which lies in the middle of the first surviving source's boundary edge.
    let mesh = KeptMesh {
        vertices:vec![[10.,0.,0.],[10.001,0.,0.],[10.,1.,0.],
            [0.,0.,0.],[2.,0.,0.],[0.,2.,0.],[1.,0.,0.],[1.,-1.,0.],[2.,-1.,0.]],
        triangles:vec![[0,1,2],[3,4,5],[6,7,8]],sheet:vec![11,23,37],
    };
    let rims = vec![Rim {sheet:0,vertices:vec![0],closed:false},Rim {sheet:1,vertices:vec![1],closed:false}];
    (mesh,rims)
}
fn inside(p: P,tri: [P;3]) -> bool {
    (0..3).all(|k| { let (a,b) = (tri[k],tri[(k+1)%3]);
        (b[0]-a[0])*(p[1]-a[1])-(b[1]-a[1])*(p[0]-a[0]) >= -1e-12
    })
}
fn check_parents(mesh: &KeptMesh,parents: &[(u32,[P;3])]) {
    assert_eq!(mesh.sheet.len(),mesh.triangles.len());
    for (i,t) in mesh.triangles.iter().enumerate() {
        let p = t.map(|v| mesh.vertices[v as usize]);
        let owners: Vec<_> = parents.iter().filter(|(_,tri)| p.iter().all(|&p| inside(p,*tri))).map(|(s,_)| *s).collect();
        assert_eq!(owners.len(),1,"each surviving facet must lie in exactly one parent domain: {p:?}");
        assert_eq!(mesh.sheet[i],owners[0],"wrong source on triangle {i}");
    }
}

#[test]
fn crease_removal_then_splitting_and_compaction_preserve_parent_ownership() {
    let (mut mesh,rims) = fixture();
    let parents: Vec<_> = [1,2].map(|i| (mesh.sheet[i],mesh.triangles[i].map(|v| mesh.vertices[v as usize]))).into();
    sb::merge_creases(&mut mesh,&rims,0.02,0.01);
    check_parents(&mesh,&parents);
    let before = mesh.triangles.len();
    sb::split_at_vertices(&mut mesh,1e-6);
    assert!(mesh.triangles.len() > before,"exercise child creation after removal");
    check_parents(&mesh,&parents);
    assert_eq!(mesh.sheet.iter().filter(|&&s| s == 23).count(),2);
    // Dedupe chooses the first copy, which has the parent's source, even when a
    // later copy carries another source ID. Then compact and remove a whole source.
    mesh.triangles.push(mesh.triangles[0]); mesh.sheet.push(99);
    assert_eq!(sb::dedupe(&mut mesh),1);
    sb::weld(&mut mesh,1e-10);
    mesh = mesh.compact(); check_parents(&mesh,&parents);
    let keep: Vec<_> = mesh.sheet.iter().map(|&s| s == 23).collect();
    mesh = sb::retained(&mesh,&keep).compact(); check_parents(&mesh,&parents[..1]);
    assert_eq!(mesh.triangles.len(),2);
}

#[test]
fn overlap_fragments_keep_the_source_that_was_cut() {
    let mut mesh = KeptMesh {
        vertices:vec![[0.,0.,0.],[1.,0.,0.],[0.,1.,0.],[-1.,0.,0.],[2.,0.,0.],[0.,2.,0.]],
        triangles:vec![[0,1,2],[3,4,5]],sheet:vec![3,7],
    };
    let (cut,changed) = sb::clip_overlaps(&mesh,0.02);
    assert_eq!(changed,1); assert!(cut.triangles.len() > 2);
    // The earlier triangle stays verbatim; every other piece comes from source 7.
    assert_eq!(cut.triangles[0],mesh.triangles[0]); assert_eq!(cut.sheet[0],3);
    assert!(cut.sheet[1..].iter().all(|&s| s == 7));
    let parent = mesh.triangles[1].map(|v| mesh.vertices[v as usize]);
    for t in &cut.triangles[1..] { assert!(t.iter().all(|&v| inside(cut.vertices[v as usize],parent))); }
    mesh = cut.compact(); assert_eq!(mesh.sheet.len(),mesh.triangles.len());
}

#[test]
fn low_level_edits_refuse_misaligned_metadata_before_mutation() {
    let (original,_) = fixture();
    for short in [false,true] {
        let mut bad = original.clone();
        if short { bad.sheet.pop(); } else { bad.sheet.push(99); }
        assert!(std::panic::catch_unwind(|| bad.compact()).is_err());
        assert!(std::panic::catch_unwind(|| sb::retained(&bad,&[true;3])).is_err());
        for edit in [sb::weld as fn(&mut KeptMesh,f64),sb::split_at_vertices] {
            let mut mesh = bad.clone();
            assert!(std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| edit(&mut mesh,0.01))).is_err());
            assert_eq!(mesh.vertices,bad.vertices); assert_eq!(mesh.triangles,bad.triangles); assert_eq!(mesh.sheet,bad.sheet);
        }
    }
}

#[test]
fn newly_filled_faces_have_explicit_constructed_provenance() {
    let mut mesh = KeptMesh {
        vertices:vec![[0.,0.,0.],[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]],
        triangles:vec![[0,2,1],[0,1,3],[0,3,2]],sheet:vec![11,23,37],
    };
    let (_,open) = sb::rim_zip(&mut mesh,0.1,1e-8,1e-8,&mut |_,_| false);
    assert!(open.is_empty()); assert_eq!(mesh.triangles.len(),4);
    assert_eq!(mesh.sheet,[11,23,37,u32::MAX]);
}
