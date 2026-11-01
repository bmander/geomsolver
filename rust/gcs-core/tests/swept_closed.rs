//! **The swept solid, closed** (#721 X6, `brep::swept::closed`, docs/swept-solid-design.md): a
//! polyhedral tool under a slide or a turn, built as one B-rep by the design's pipeline — the
//! candidate faces (faces at the end poses, silhouette edges and contact lines swept), a box split
//! by them, each cell kept by the kernel's membership — or, turned whole, as its solid of
//! revolution. Under a turn each is held to the volume its slices sweep (`turned_volume`). Held to
//! what is known apart from it: a
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
    let screw = Trajectory::Twist(Screw {origin:[0.;3],axis:[0.,0.,1.],ratio:1.,advance:2.});
    assert_eq!(closed(&tool,&screw,[0.,1.],0.01,1e-7).unwrap_err(),Unbuilt::Motion);
}

/// A block leaning to the axis: `block` turned `lean` about the world's `x` and moved out `off`.
fn leaning(lean: f64,off: f64) -> Brep {
    let m = gcs_core::brep::geom::Rigid::turn([0.;3],[1.,0.,0.],lean).then(&gcs_core::brep::geom::Rigid {t:[off,0.,0.],..gcs_core::brep::geom::Rigid::identity()});
    block([0.,-0.5,-0.75],[1.,0.5,0.75]).moved(&m)
}

/// The radii a convex tool's slice at height `z` spans about the world's `z` axis: the nearest of
/// it and the farthest, its edges cut by the slice.
fn spans(tool: &Brep,z: f64) -> Option<(f64,f64)> {
    let pts: Vec<[f64;2]> = tool.edges.iter().filter_map(|e| {
        let (p,q) = (e.point(e.t[0],&tool.vertices),e.point(e.t[1],&tool.vertices));
        if (p[2]-z)*(q[2]-z) > 0. || p[2] == q[2] { return None }
        let t = (z-p[2])/(q[2]-p[2]);
        Some([p[0]+t*(q[0]-p[0]),p[1]+t*(q[1]-p[1])])
    }).collect();
    if pts.len() < 3 { return None }
    let c = [pts.iter().map(|p| p[0]).sum::<f64>()/pts.len() as f64,pts.iter().map(|p| p[1]).sum::<f64>()/pts.len() as f64];
    let mut ring = pts.clone();
    ring.sort_by(|a,b| (a[1]-c[1]).atan2(a[0]-c[0]).total_cmp(&(b[1]-c[1]).atan2(b[0]-c[0])));
    let far = ring.iter().map(|p| p[0].hypot(p[1])).fold(0.,f64::max);
    // the slice holds the axis where the origin is left of every side of the ring
    let holds = (0..ring.len()).all(|k| {
        let (a,b) = (ring[k],ring[(k+1)%ring.len()]);
        (b[0]-a[0])*(-a[1])-(b[1]-a[1])*(-a[0]) >= 0.
    });
    if holds { return Some((0.,far)) }
    let near = (0..ring.len()).map(|k| {
        let (a,b) = (ring[k],ring[(k+1)%ring.len()]);
        let (dx,dy) = (b[0]-a[0],b[1]-a[1]);
        let t = (-(a[0]*dx+a[1]*dy)/(dx*dx+dy*dy)).clamp(0.,1.);
        (a[0]+t*dx).hypot(a[1]+t*dy)
    }).fold(f64::INFINITY,f64::min);
    Some((near,far))
}

/// **A block leaning to the axis turned whole is its solid of revolution**: its leaning edges turn
/// into hyperboloids and its leaning faces' contact lines (which meet the axis) into double cones,
/// each held exactly, its faces at the end poses touching those cones along a line (a plane tangent
/// to a cone, met in closed form). Turned a whole turn it sweeps the solid of revolution of its
/// slices' radial spans, `∫ π (ρ_max² − ρ_min²) dz`, integrated here from its slices apart from the
/// build (Simpson's rule between the heights its vertices stand at).
#[test]
fn a_block_leaning_to_the_axis_turned_whole_is_its_solid_of_revolution() {
    turned_whole_is_its_solid_of_revolution(&leaning(0.35,3.));
}

/// **A block straddling the axis, tipped two ways, turned whole is its solid of revolution**: its
/// slices hold the axis over some heights and not others, the nearest of a slice reached on a side
/// over some and at a corner over others, each stretch of the profile its own curve.
#[test]
fn a_block_straddling_the_axis_tipped_two_ways_turned_whole_is_its_solid_of_revolution() {
    use gcs_core::brep::geom::Rigid;
    let m = Rigid::turn([0.;3],[1.,0.,0.],0.6).then(&Rigid::turn([0.;3],[0.,1.,0.],0.45))
        .then(&Rigid {t:[0.3,-0.2,0.1],..Rigid::identity()});
    turned_whole_is_its_solid_of_revolution(&block([-0.5,-0.4,-1.],[0.7,0.6,1.]).moved(&m));
}

/// `tool` turned a whole turn about the world's `z` axis, held to the volume of its slices' radial
/// spans and to membership.
fn turned_whole_is_its_solid_of_revolution(tool: &Brep) {
    let tool = tool.clone();
    let path = turn([0.;3],[0.,0.,1.],std::f64::consts::TAU);
    let built = gcs_core::brep::swept::closed::closed(&tool,&path,[0.,1.],0.002,1e-6).unwrap();
    let mut heights: Vec<f64> = tool.vertices.iter().map(|v| v.p[2]).collect();
    heights.sort_by(f64::total_cmp);
    heights.dedup_by(|a,b| (*a-*b).abs() < 1e-12);
    let area = |z: f64| spans(&tool,z).map_or(0.,|(near,far)| std::f64::consts::PI*(far*far-near*near));
    let mut expected = 0.;
    for w in heights.windows(2) {
        let n = 4000;
        let h = (w[1]-w[0])/n as f64;
        expected += h/3.*(0..=n).map(|k| {
            let z = (w[0]+h*k as f64).clamp(w[0]+1e-12,w[1]-1e-12);
            area(z)*if k == 0 || k == n { 1. } else if k%2 == 1 { 4. } else { 2. }
        }).sum::<f64>();
    }
    assert!((volume(&built)-expected).abs() <= 1e-6*expected,"{} against {expected}",volume(&built));
    agrees(&built,&tool,&path);
}

/// The volume `tool` sweeps turned by `angle` about the world's `z` axis, apart from the build:
/// each slice's swept area `∫ L(ρ) ρ dρ`, `L` the angle the circle of radius `ρ` sweeps (its arcs
/// within the slice, each carried through the turn, united), by Gauss–Legendre between the radii
/// where `L` turns (the slice's corners' and its sides' feet), cut `pieces` times more; and the
/// areas by Simpson's rule on `n` steps between the heights the tool's vertices stand at.
fn turned_volume(tool: &Brep,angle: f64,n: usize,pieces: usize) -> f64 { turned_volume_of(&[tool],angle,n,pieces) }

/// `turned_volume` of the tool that is the union of the convex `parts`: on each circle the arcs of
/// every part's slice, each carried through the turn, united.
fn turned_volume_of(parts: &[&Brep],angle: f64,n: usize,pieces: usize) -> f64 {
    use std::f64::consts::TAU;
    // the 8-point Gauss–Legendre rule on [−1, 1]
    const X: [f64;4] = [0.1834346424956498,0.5255324099163290,0.7966664774136267,0.9602898564975363];
    const W: [f64;4] = [0.3626837833783620,0.3137066458778873,0.2223810344533745,0.1012285362903763];
    // the arcs of the circle of radius `r` within a convex slice, by angle
    let arcs = |ring: &[[f64;2]],r: f64| -> Vec<[f64;2]> {
        let inside = |phi: f64| { let p = [r*phi.cos(),r*phi.sin()]; (0..ring.len()).all(|k| {
            let (a,b) = (ring[k],ring[(k+1)%ring.len()]);
            (b[0]-a[0])*(p[1]-a[1])-(b[1]-a[1])*(p[0]-a[0]) >= 0. }) };
        let mut cuts: Vec<f64> = Vec::new();
        for k in 0..ring.len() {
            let (a,b) = (ring[k],ring[(k+1)%ring.len()]);
            let d = [b[0]-a[0],b[1]-a[1]];
            let (qa,qb,qc) = (d[0]*d[0]+d[1]*d[1],2.*(a[0]*d[0]+a[1]*d[1]),a[0]*a[0]+a[1]*a[1]-r*r);
            let disc = qb*qb-4.*qa*qc;
            if disc < 0. { continue }
            for s in [-1.,1.] {
                let t = (-qb+s*disc.sqrt())/(2.*qa);
                if (0. ..1.).contains(&t) { cuts.push((a[1]+t*d[1]).atan2(a[0]+t*d[0])) }
            }
        }
        if cuts.is_empty() { return if inside(0.) { vec![[0.,TAU]] } else { vec![] } }
        cuts.sort_by(f64::total_cmp);
        (0..cuts.len()).filter_map(|k| {
            let (a,b) = (cuts[k],if k+1 < cuts.len() { cuts[k+1] } else { cuts[0]+TAU });
            inside(0.5*(a+b)).then_some([a,b])
        }).collect()
    };
    // the angle a set of arcs covers carried through the turn: united on the circle, unrolled from
    // the start of the first
    let covered = |arcs: &[[f64;2]]| -> f64 {
        if arcs.is_empty() { return 0. }
        if arcs.iter().any(|a| a[1]-a[0] >= TAU) { return TAU }
        let base = arcs.iter().map(|a| a[0]).fold(f64::INFINITY,f64::min);
        let mut spans: Vec<[f64;2]> = arcs.iter().flat_map(|a| [[a[0]-base,a[1]+angle-base],[a[0]-base-TAU,a[1]+angle-base-TAU]]).collect();
        spans.sort_by(|x,y| x[0].total_cmp(&y[0]));
        let (mut total,mut reach) = (0.,f64::NEG_INFINITY);
        for s in spans {
            let (a,b) = (s[0].max(reach).max(0.),s[1].min(TAU));
            if b > a { total += b-a }
            reach = reach.max(s[1]);
        }
        total
    };
    let area = |z: f64| -> f64 {
        let rings: Vec<Vec<[f64;2]>> = parts.iter().filter_map(|t| slice(t,z)).collect();
        if rings.is_empty() { return 0. }
        let mut radii: Vec<f64> = vec![0.];
        for ring in &rings {
            for k in 0..ring.len() {
                let (a,b) = (ring[k],ring[(k+1)%ring.len()]);
                radii.push(a[0].hypot(a[1]));
                let d = [b[0]-a[0],b[1]-a[1]];
                let t = -(a[0]*d[0]+a[1]*d[1])/(d[0]*d[0]+d[1]*d[1]);
                if t > 0. && t < 1. { radii.push((a[0]+t*d[0]).hypot(a[1]+t*d[1])) }
            }
        }
        radii.sort_by(f64::total_cmp);
        let mut total = 0.;
        for w in radii.windows(2) {
            for p in 0..pieces {
                let (r0,r1) = (w[0]+(w[1]-w[0])*p as f64/pieces as f64,w[0]+(w[1]-w[0])*(p+1) as f64/pieces as f64);
                let (m,h) = (0.5*(r0+r1),0.5*(r1-r0));
                for (x,wt) in X.iter().zip(W) { for r in [m-h*x,m+h*x] {
                    let all: Vec<[f64;2]> = rings.iter().flat_map(|ring| arcs(ring,r)).collect();
                    total += wt*h*covered(&all)*r
                } }
            }
        }
        total
    };
    let mut heights: Vec<f64> = parts.iter().flat_map(|t| t.vertices.iter().map(|v| v.p[2])).collect();
    heights.sort_by(f64::total_cmp);
    heights.dedup_by(|a,b| (*a-*b).abs() < 1e-12);
    let mut volume = 0.;
    for w in heights.windows(2) {
        let h = (w[1]-w[0])/n as f64;
        volume += h/3.*(0..=n).map(|k| {
            let z = (w[0]+h*k as f64).clamp(w[0]+1e-12,w[1]-1e-12);
            area(z)*if k == 0 || k == n { 1. } else if k%2 == 1 { 4. } else { 2. }
        }).sum::<f64>();
    }
    volume
}

/// `tool`'s slice at height `z`, a convex ring counter-clockwise.
fn slice(tool: &Brep,z: f64) -> Option<Vec<[f64;2]>> {
    let pts: Vec<[f64;2]> = tool.edges.iter().filter_map(|e| {
        let (p,q) = (e.point(e.t[0],&tool.vertices),e.point(e.t[1],&tool.vertices));
        if (p[2]-z)*(q[2]-z) > 0. || p[2] == q[2] { return None }
        let t = (z-p[2])/(q[2]-p[2]);
        Some([p[0]+t*(q[0]-p[0]),p[1]+t*(q[1]-p[1])])
    }).collect();
    if pts.len() < 3 { return None }
    let c = [pts.iter().map(|p| p[0]).sum::<f64>()/pts.len() as f64,pts.iter().map(|p| p[1]).sum::<f64>()/pts.len() as f64];
    let mut ring = pts;
    ring.sort_by(|a,b| (a[1]-c[1]).atan2(a[0]-c[0]).total_cmp(&(b[1]-c[1]).atan2(b[0]-c[0])));
    Some(ring)
}

/// **A block leaning to the axis turned part of a turn is its slices swept**: its arrangement —
/// the faces at both end poses, its edges' hyperboloids, its faces' contact cones and cylinders,
/// the planes parting their touches — split, classified and united, the hyperboloids met by the
/// planes in closed form. Held to the volume its slices sweep, worked apart from the build: at each
/// height and radius the circle's arcs within the slice, each carried through the turn, united
/// (`turned_volume`), to 1e-6; and to membership.
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, about 26 s: a leaning block's part turn, its arrangement of 800 cells")]
fn a_block_leaning_to_the_axis_turned_part_of_a_turn_is_its_slices_swept() {
    let tool = leaning(0.35,3.);
    let path = turn([0.;3],[0.,0.,1.],2.);
    let built = gcs_core::brep::swept::closed::closed(&tool,&path,[0.,1.],0.002,1e-6).unwrap();
    let expected = turned_volume(&tool,2.,200,8);
    assert!((volume(&built)-expected).abs() <= 1e-6*expected,"{} against {expected}",volume(&built));
    agrees(&built,&tool,&path);
}

/// **A bar turned through all but its own width meets itself, and the pipeline trims it** (X5 and
/// X1 under a turn): turned 6 radians, its end pose overlaps its start, and the faces at the two
/// poses and the cylinders its edges along the axis turn into bury one another where they meet,
/// which membership drops. Held to the volume its slices sweep, each circle's arc carried through
/// the turn and united round the circle (`turned_volume`), and to membership.
#[test]
fn a_bar_turned_through_all_but_its_width_meets_itself() {
    let bar = block([2.,0.,0.],[3.,1.,1.]);
    let path = turn([0.;3],[0.,0.,1.],6.);
    let built = gcs_core::brep::swept::closed::closed(&bar,&path,[0.,1.],0.002,1e-7).unwrap();
    let expected = turned_volume(&bar,6.,4,256);
    assert!((volume(&built)-expected).abs() <= 1e-8*expected,"{} against {expected}",volume(&built));
    agrees(&built,&bar,&path);
}

/// **An L-shaped prism turned is its blocks' sweeps united** (X3 under a turn): its concave edge's
/// faces' sheets cross, the arrangement splits them where they do and membership drops what is
/// buried. Held to the volume the two blocks' slices sweep together (`turned_volume_of`, each
/// circle's arcs of both carried and united) and to membership.
#[test]
fn an_l_shaped_prism_turned_is_its_blocks_sweeps_united() {
    let (a,b) = (block([2.,0.,0.],[4.,1.,1.]),block([2.,1.,0.],[3.,3.,1.]));
    let tool = combined(&a,&b,Op::Union,0.).unwrap();
    let path = turn([0.;3],[0.,0.,1.],1.);
    let built = gcs_core::brep::swept::closed::closed(&tool,&path,[0.,1.],0.002,1e-7).unwrap();
    let expected = turned_volume_of(&[&a,&b],1.,4,64);
    assert!((volume(&built)-expected).abs() <= 1e-7*expected,"{} against {expected}",volume(&built));
    agrees(&built,&tool,&path);
}
