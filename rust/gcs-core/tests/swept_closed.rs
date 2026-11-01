//! **The swept solid, closed** (#721 X6, `brep::swept::closed`, docs/swept-solid-design.md): a
//! polyhedral tool under a slide, built as one B-rep by the design's pipeline — the candidate
//! planes (faces entering at the start, leaving at the end, the silhouette edges swept), a box
//! split by them, each cell kept by the kernel's membership. Held to what is known apart from it: a
//! box slid on a slant is a zonotope, its volume `lx ly lz + |Dx| ly lz + |Dy| lx lz + |Dz| lx ly`;
//! an L-shaped prism's sweep is its two blocks' sweeps united (`sweep(A ∪ B) = sweep(A) ∪
//! sweep(B)`), each built alone; and every built solid's points agree with membership at seeded
//! points. Sampled where it samples.
use gcs_core::brep::build::{Profile,ProfileEdge,prism};
use gcs_core::brep::boolean::{Op,combined};
use gcs_core::brep::props::volume;
use gcs_core::brep::swept::closed::slid;
use gcs_core::brep::swept::member::{Solid,Swept};
use gcs_core::brep::topo::Brep;
use gcs_core::brep::trajectory::{Screw,Trajectory};
use gcs_core::space::{norm,unit};

/// The block with corners `lo` and `hi`.
fn block(lo: [f64;3],hi: [f64;3]) -> Brep {
    let at = |x: f64,y: f64| [x,y,lo[2]];
    let c = [at(lo[0],lo[1]),at(hi[0],lo[1]),at(hi[0],hi[1]),at(lo[0],hi[1])];
    let edges = (0..4).map(|k| ProfileEdge::Line {a:c[k],b:c[(k+1)%4]}).collect();
    prism(&Profile {origin:lo,normal:[0.,0.,1.],loops:vec![edges],names:Vec::new()},0.,hi[2]-lo[2]).unwrap()
}

/// The slide carrying a tool by `d` over the roll `[0, 1]`.
fn slide(d: [f64;3]) -> Trajectory {
    Trajectory::Twist(Screw {origin:[0.;3],axis:unit(d),ratio:0.,advance:norm(d)*std::f64::consts::TAU})
}

/// `built`, the closed sweep of `tool` under `path`, read against membership at seeded points of
/// its box: every settled point inside it a member, every one outside not.
fn agrees(built: &Brep,tool: &Brep,path: &Trajectory) {
    let (solid,body) = (Solid::new(tool,0.002,1.).unwrap(),Solid::new(built,0.002,1.).unwrap());
    let swept = Swept {tool:&solid,path,roll:[0.,1.]};
    let (lo,hi) = built.bounds();
    let mut rng = gcs_core::rng::Rng::new(6);
    let mut held = 0;
    for _ in 0..200 {
        let p: [f64;3] = std::array::from_fn(|k| rng.uniform(lo[k]-0.2,hi[k]+0.2));
        let d = body.signed_distance(p);
        if d.abs() < 0.01 { continue }
        let Ok(member) = swept.member(p) else { continue };
        assert_eq!(member,d < 0.,"at {p:?}: membership {member}, the built solid's distance {d}");
        held += 1;
    }
    assert!(held > 150,"{held} points held");
}

#[test]
fn a_box_slid_aslant_closes_into_its_zonotope() {
    let (l,d) = ([2.,1.,1.5],[1.3,-0.8,0.6]);
    let tool = block([0.;3],l);
    let path = slide(d);
    let built = slid(&tool,&path,[0.,1.],0.002,1e-7).unwrap();
    let expected = l[0]*l[1]*l[2]+d[0].abs()*l[1]*l[2]+d[1].abs()*l[0]*l[2]+d[2].abs()*l[0]*l[1];
    assert!((volume(&built)-expected).abs() <= 1e-9*expected,"{} against {expected}",volume(&built));
    agrees(&built,&tool,&path);
}

#[test]
fn an_l_shaped_prism_slid_is_its_blocks_sweeps_united() {
    let (a,b) = (block([0.,0.,0.],[3.,1.,1.]),block([0.,1.,0.],[1.,3.,1.]));
    let tool = combined(&a,&b,Op::Union,0.).unwrap();
    let path = slide([1.1,0.7,0.5]);
    let built = slid(&tool,&path,[0.,1.],0.002,1e-7).unwrap();
    let united = combined(&slid(&a,&path,[0.,1.],0.002,1e-7).unwrap(),&slid(&b,&path,[0.,1.],0.002,1e-7).unwrap(),Op::Union,0.).unwrap();
    assert!((volume(&built)-volume(&united)).abs() <= 1e-8*volume(&united),"{} against {}",volume(&built),volume(&united));
    agrees(&built,&tool,&path);
}
