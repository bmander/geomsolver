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

/// The turn about the line through `o` along `axis` by `angle` radians over the roll `[0, 1]`.
fn turn(o: [f64;3],axis: [f64;3],angle: f64) -> Trajectory {
    Trajectory::Twist(Screw {origin:o,axis:unit(axis),ratio:angle,advance:0.})
}

/// **A bar turned about a line along it closes into the region it sweeps, extruded**: a block
/// standing along the axis, beside it, turned a quarter turn. Its section's swept area is the polar
/// integral of what each circle about the axis covers: the arc it meets the block in, carried
/// through the turn, `A + θ (r_max² − r_min²) / 2`, and past the far face the gap between the two
/// arcs it meets there, which the turn covers too; the solid that times its height — closed form,
/// apart from the build. Its inner face and its far face touch their own contact cylinders at each
/// end pose, so this is also where the touch is parted.
#[test]
fn a_bar_turned_about_a_line_along_it_closes_into_its_swept_region() {
    let (x,y,h,theta) = ([2.,4.],[-0.25,0.25],1.,std::f64::consts::FRAC_PI_2);
    let tool = block([x[0],y[0],0.],[x[1],y[1],h]);
    let path = turn([0.;3],[0.,0.,1.],theta);
    let built = gcs_core::brep::swept::closed::closed(&tool,&path,[0.,1.],0.002,1e-6).unwrap();
    let (rmin,rmax) = (x[0],x[1].hypot(y[1]));
    agrees(&built,&tool,&path);
    // past the far face, `r > x₁`, a circle meets the block in two arcs, one at each far corner, and
    // the turn carries the lower one over the gap between them: `∫ 2 r acos(x₁ / r) dr` more,
    // `r² acos(c / r) − c √(r² − c²)` from `c = x₁` to `r_max`
    let c = x[1];
    let gap = rmax*rmax*(c/rmax).acos()-c*(rmax*rmax-c*c).sqrt();
    let expected = h*((x[1]-x[0])*(y[1]-y[0])+theta*(rmax*rmax-rmin*rmin)/2.+gap);
    assert!((volume(&built)-expected).abs() <= 1e-9*expected,"{} against {expected}",volume(&built));
}

/// What the build refuses it refuses by what the tool and the motion are: a block leaning to the
/// axis (its edges turn into hyperboloids) and a screw, the design's later steps.
#[test]
fn what_the_closed_build_cannot_yet_build_it_names() {
    use gcs_core::brep::swept::closed::{Unbuilt,closed};
    let tool = block([2.,0.,0.],[3.,1.,1.]);
    assert_eq!(closed(&tool,&turn([0.;3],[0.,1.,1.],1.),[0.,1.],0.01,1e-7).unwrap_err(),Unbuilt::Skew);
    let screw = Trajectory::Twist(Screw {origin:[0.;3],axis:[0.,0.,1.],ratio:1.,advance:2.});
    assert_eq!(closed(&tool,&screw,[0.,1.],0.01,1e-7).unwrap_err(),Unbuilt::Motion);
}
