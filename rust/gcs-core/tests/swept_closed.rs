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
use crate::brep::{block,simpson};
use gcs_core::brep::boolean::{Op,combined};
use gcs_core::brep::props::volume;
use gcs_core::brep::swept::closed::{closed,slid};
use gcs_core::brep::swept::member::{Solid,Swept};
use gcs_core::brep::topo::Brep;
use gcs_core::brep::trajectory::{Screw,Trajectory};
use gcs_core::space::{norm,unit};

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
    let built = closed(&tool,&path,[0.,1.],0.002,1e-6).unwrap();
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

/// **A bar beside the axis under a screw closes into the region it sweeps** (#735's gate): its edges
/// along the axis and its side faces' contact lines screw into cylinders about the axis, its level
/// edges into helicoidal sheets, each a level triangle on the line screwed past the box. Apart from
/// the build: in helical coordinates `(ρ, ψ, ζ = z − k ψ)`, `k` the step a radian, the screw is a
/// shift in `ψ` and the volume element `ρ dρ dψ dζ`; each helix `(ρ, ζ)` meets the bar in one
/// interval of `ψ` (it climbs `2πk` a turn, past the bar's height), which the turn `Θ` lengthens by
/// `Θ`. So the sweep is the bar's volume and `Θ` times the measure `∫ ρ dρ dζ` of the helices that
/// meet the bar: at each radius the bar's arc `Δψ(ρ)` and its height `h` reach `h + k Δψ(ρ)` of `ζ`,
/// so `∫ ρ (h + k Δψ) dρ = h (ρ_max² − ρ_min²) / 2 + k A`, the bar's section `A`. For a unit cube
/// two out, `1 + Θ (3 + k)`. Built at 1e-6 and at 1e-7: its inner face at the end pose holds a
/// ruling of its inner level edge's sheet and touches it at the edge's foot, where the cylinder of
/// the foot's radius touches both — three exact curves through one vertex at any tolerance.
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, about 50 s: a bar under a screw, one arrangement at two tolerances")]
fn a_bar_under_a_screw_closes_into_its_helical_sweep() {
    let tool = block([2.,0.,0.],[3.,1.,1.]);
    let (theta,advance) = (1.,2.);
    let path = Trajectory::Twist(Screw {origin:[0.;3],axis:[0.,0.,1.],ratio:theta,advance});
    let k = advance/std::f64::consts::TAU/theta;
    let expected = 1.+theta*(3.+k);
    for tol in [1e-6,1e-7] {
        let built = closed(&tool,&path,[0.,1.],0.002,tol).unwrap();
        assert!((volume(&built)-expected).abs() <= 1e-9*expected,"at {tol:e}: {} against {expected}",volume(&built));
        agrees(&built,&tool,&path);
    }
}

/// What the build refuses it refuses by what the tool and the motion are: under a screw, an edge
/// leaning to the axis, the design's later step.
#[test]
fn what_the_closed_build_cannot_yet_build_it_names() {
    use gcs_core::brep::swept::closed::Unbuilt;
    let screw = Trajectory::Twist(Screw {origin:[0.;3],axis:[0.,0.,1.],ratio:1.,advance:2.});
    let tool = leaning(0.35,3.);
    let Err(Unbuilt::Leaning {at}) = closed(&tool,&screw,[0.,1.],0.01,1e-7) else { panic!("refused as leaning") };
    // its witness on the tool
    let solid = Solid::new(&tool,0.002,1.).unwrap();
    assert!(solid.signed_distance(at).abs() <= 1e-9,"{at:?} on the tool");
    assert!(matches!(gcs_core::brep::swept::closed::reads(&tool,&screw),Err(Unbuilt::Leaning {..})),"read so before any build");
}

/// A block leaning to the axis: `block` turned `lean` about the world's `x` and moved out `off`.
fn leaning(lean: f64,off: f64) -> Brep {
    let m = gcs_core::brep::geom::Rigid::turn([0.;3],[1.,0.,0.],lean).then(&gcs_core::brep::geom::Rigid {t:[off,0.,0.],..gcs_core::brep::geom::Rigid::identity()});
    block([0.,-0.5,-0.75],[1.,0.5,0.75]).moved(&m)
}

/// The radii a convex tool's slice at height `z` spans about the world's `z` axis: the nearest of
/// it and the farthest, its edges cut by the slice.
fn spans(tool: &Brep,z: f64) -> Option<(f64,f64)> {
    let ring = slice(tool,z)?;
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
    let path = turn([0.;3],[0.,0.,1.],std::f64::consts::TAU);
    let built = closed(tool,&path,[0.,1.],0.002,1e-6).unwrap();
    let expected = revolved_volume(tool);
    assert!((volume(&built)-expected).abs() <= 1e-6*expected,"{} against {expected}",volume(&built));
    agrees(&built,tool,&path);
}

/// The volume of a convex `tool`'s solid of revolution about the world's `z` axis, apart from any
/// build: `∫ π (ρ_max² − ρ_min²) dz` over its slices' radial spans (`spans`), by Simpson's rule
/// between the heights its vertices stand at.
pub(crate) fn revolved_volume(tool: &Brep) -> f64 {
    between_vertices(&[tool],4000,&|z| spans(tool,z).map_or(0.,|(near,far)| std::f64::consts::PI*(far*far-near*near)))
}

/// `∫ area(z) dz` over the heights the `parts`' vertices stand at, by Simpson's rule on `n` steps
/// between each two (the slices' combinatorics, so `area`'s kinks, change only there).
fn between_vertices(parts: &[&Brep],n: usize,area: &dyn Fn(f64) -> f64) -> f64 {
    let mut heights: Vec<f64> = parts.iter().flat_map(|t| t.vertices.iter().map(|v| v.p[2])).collect();
    heights.sort_by(f64::total_cmp);
    heights.dedup_by(|a,b| (*a-*b).abs() < 1e-12);
    heights.windows(2).map(|w| simpson(w[0],w[1],n,|z| area(z.clamp(w[0]+1e-12,w[1]-1e-12)))).sum()
}

/// The volume `tool` sweeps turned by `angle` about the world's `z` axis, apart from the build:
/// each slice's swept area `∫ L(ρ) ρ dρ`, `L` the angle the circle of radius `ρ` sweeps (its arcs
/// within the slice, each carried through the turn, united), by Gauss–Legendre between the radii
/// where `L` turns (the slice's corners' and its sides' feet), cut `pieces` times more; and the
/// areas by Simpson's rule on `n` steps between the heights the tool's vertices stand at
/// (`between_vertices`). The tool is the union of the convex `parts`: on each circle the arcs of
/// every part's slice, each carried through the turn, united.
fn turned_volume(parts: &[&Brep],angle: f64,n: usize,pieces: usize) -> f64 {
    use std::f64::consts::TAU;
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
                for (x,wt) in gcs_core::integral::GAUSS8 {
                    let r = m+h*x;
                    let all: Vec<[f64;2]> = rings.iter().flat_map(|ring| arcs(ring,r)).collect();
                    total += wt*h*covered(&all)*r
                }
            }
        }
        total
    };
    between_vertices(parts,n,&area)
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
/// the faces at both end poses, its edges' hyperboloids, its faces' contact cones and cylinders —
/// one split of the box by all of them at once, each cell classified, and the members' boundary
/// selected (#735: no separator, no order of cuts, no Boolean a cell, no union). Its touches are
/// the arrangement's: a face at an end pose holding the ruling of a hyperboloid it is tangent to, a
/// hyperboloid touching the cylinder of its throat's radius round the throat, a plane crossing that
/// circle where the two meetings it lays are tangent. Held to the volume its slices sweep, worked
/// apart from the build: at each height and radius the circle's arcs within the slice, each carried
/// through the turn, united (`turned_volume`), to 1e-6; and to membership.
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, about 85 s: a leaning block's part turn, one arrangement of 17 candidates")]
fn a_block_leaning_to_the_axis_turned_part_of_a_turn_is_its_slices_swept() {
    let tool = leaning(0.35,3.);
    let path = turn([0.;3],[0.,0.,1.],2.);
    let built = closed(&tool,&path,[0.,1.],0.002,1e-6).unwrap();
    let expected = turned_volume(&[&tool],2.,200,8);
    assert!((volume(&built)-expected).abs() <= 1e-6*expected,"{} against {expected}",volume(&built));
    agrees(&built,&tool,&path);
}

/// **The same part turn at a tenth of the tolerance** (#735's regression gate): the one
/// arrangement held at 1e-7, where a face at an end pose holding a hyperboloid's ruling is met in
/// that ruling and its partner whatever the tolerance (`ssi`: a constant at the centre that is
/// nothing but rounding), and crossings a hair from vertices leave pieces shorter than a vertex's
/// reach (collapsed, `tolerant`).
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, about 90 s: a leaning block's part turn at 1e-7")]
fn a_block_leaning_to_the_axis_turned_part_of_a_turn_holds_at_a_tenth_of_the_tolerance() {
    let tool = leaning(0.35,3.);
    let path = turn([0.;3],[0.,0.,1.],2.);
    let built = closed(&tool,&path,[0.,1.],0.002,1e-7).unwrap();
    let expected = turned_volume(&[&tool],2.,200,8);
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
    let built = closed(&bar,&path,[0.,1.],0.002,1e-7).unwrap();
    let expected = turned_volume(&[&bar],6.,4,256);
    assert!((volume(&built)-expected).abs() <= 1e-8*expected,"{} against {expected}",volume(&built));
    agrees(&built,&bar,&path);
}

/// **An L-shaped prism turned is its blocks' sweeps united** (X3 under a turn): its concave edge's
/// faces' sheets cross, the arrangement splits them where they do and membership drops what is
/// buried. Held to the volume the two blocks' slices sweep together (`turned_volume`, each
/// circle's arcs of both carried and united) and to membership.
#[test]
fn an_l_shaped_prism_turned_is_its_blocks_sweeps_united() {
    let (a,b) = (block([2.,0.,0.],[4.,1.,1.]),block([2.,1.,0.],[3.,3.,1.]));
    let tool = combined(&a,&b,Op::Union,0.).unwrap();
    let path = turn([0.;3],[0.,0.,1.],1.);
    let built = closed(&tool,&path,[0.,1.],0.002,1e-7).unwrap();
    let expected = turned_volume(&[&a,&b],1.,4,64);
    assert!((volume(&built)-expected).abs() <= 1e-7*expected,"{} against {expected}",volume(&built));
    agrees(&built,&tool,&path);
}

/// `built`, `tool`'s sweep under `path` cut from `blank`, read against the definition at seeded
/// points of the blank's box: every settled point inside it in the blank and no member of the
/// sweep, every one outside either out of the blank or a member.
fn agrees_cut(built: &Brep,blank: &Brep,tool: &Brep,path: &Trajectory) {
    let (solid,body,stock) = (Solid::new(tool,0.002,1.).unwrap(),Solid::new(built,0.002,1.).unwrap(),Solid::new(blank,0.002,1.).unwrap());
    let swept = Swept {tool:&solid,path,roll:[0.,1.]};
    let (lo,hi) = blank.bounds();
    let mut rng = gcs_core::rng::Rng::new(9);
    let mut held = 0;
    for _ in 0..200 {
        let p: [f64;3] = std::array::from_fn(|k| rng.uniform(lo[k]-0.2,hi[k]+0.2));
        let (d,s) = (body.signed_distance(p),stock.signed_distance(p));
        if d.abs() < 0.01 || s.abs() < 0.01 { continue }
        let Ok(member) = swept.member(p) else { continue };
        assert_eq!(s < 0. && !member,d < 0.,"at {p:?}: in the blank {}, swept {member}, the cut's distance {d}",s < 0.);
        held += 1;
    }
    assert!(held > 150,"{held} points held");
}

/// **A sweep cut from a blank is one arrangement** (#735 A4, #699 C: `swept::closed::cut_from`):
/// the blank split by the sweep's candidates at once and its cells kept where they are not swept —
/// no closed solid built and then subtracted. A bar slid along a block, sunk into its top, cuts a
/// groove: `4 × 1 × 1` of the block's `24`, and the block's faces whole where the groove leaves
/// them (its ends, its sides, its bottom), the top two strips: ten faces.
#[test]
fn a_bar_slid_along_a_block_cuts_a_groove_on_one_arrangement() {
    use gcs_core::brep::swept::closed::cut_from;
    let blank = block([0.;3],[4.,3.,2.]);
    let tool = block([-3.,1.,1.],[-2.,2.,3.]);
    let path = slide([10.,0.,0.]);
    let cut = cut_from(&blank,&tool,&path,[0.,1.],0.002,1e-7).unwrap();
    cut.check(1e-7).unwrap();
    assert!((volume(&cut)-20.).abs() <= 1e-9*20.,"{}",volume(&cut));
    assert_eq!(cut.faces.len(),10);
    agrees_cut(&cut,&blank,&tool,&path);
}

/// **A bar turned through a disc is cut from it on one arrangement**: a bar standing beside the
/// axis, taller than the disc, turned a quarter turn through it, takes from it the region its section
/// sweeps (`a_bar_turned_about_a_line_along_it_closes_into_its_swept_region`'s polar integral) times
/// the disc's thickness — apart from the build — and agrees with the definition.
#[test]
fn a_bar_turned_through_a_disc_is_cut_from_it_on_one_arrangement() {
    use gcs_core::brep::build::{disc,prism};
    use gcs_core::brep::swept::closed::cut_from;
    use std::f64::consts::{FRAC_PI_2,PI};
    let (x,y,theta,rdisc,h) = ([2.,4.],[-0.25,0.25],FRAC_PI_2,6.,1.);
    let blank = prism(&disc([0.;3],[0.,0.,1.],rdisc),0.,h).unwrap();
    let tool = block([x[0],y[0],-1.],[x[1],y[1],h+1.]);
    let path = turn([0.;3],[0.,0.,1.],theta);
    let cut = cut_from(&blank,&tool,&path,[0.,1.],0.002,1e-6).unwrap();
    cut.check(1e-6).unwrap();
    let (rmin,rmax,c) = (x[0],x[1].hypot(y[1]),x[1]);
    let gap = rmax*rmax*(c/rmax).acos()-c*(rmax*rmax-c*c).sqrt();
    let taken = h*((x[1]-x[0])*(y[1]-y[0])+theta*(rmax*rmax-rmin*rmin)/2.+gap);
    let expected = PI*rdisc*rdisc*h-taken;
    assert!((volume(&cut)-expected).abs() <= 1e-9*expected,"{} against {expected}",volume(&cut));
    agrees_cut(&cut,&blank,&tool,&path);
}

/// A drum lying along `x`, radius 2 and 8 long from `x = 6`, its axis at `z = −5`: `punch.sv`'s.
fn drum() -> Brep {
    use gcs_core::brep::build::{disc,prism};
    prism(&disc([6.,0.,-5.],[1.,0.,0.],2.),0.,8.).unwrap()
}

/// **A drum slid aslant closes into its shadow carried**: a convex tool slid by `D` sweeps its own
/// volume and its shadow along `D` (on the plane square to it) times `|D|`; a cylinder's shadow at
/// `θ` from its axis is `π r² |cos θ| + 2 r L sin θ`. Its side's contact rulings sweep their
/// tangent planes and its rims oblique cylinders (#721 F2c5, a curved tool under a slide).
#[test]
fn a_drum_slid_aslant_closes_into_its_shadow_carried() {
    let d = [4.8,0.,16.*0.9539392014169457];
    let tool = drum();
    let path = slide(d);
    let built = slid(&tool,&path,[0.,1.],0.002,1e-6).unwrap();
    let (r,l,len) = (2.,8.,norm(d));
    let cos = d[0]/len;
    let expected = std::f64::consts::PI*r*r*l+len*(std::f64::consts::PI*r*r*cos+2.*r*l*(1.-cos*cos).sqrt());
    assert!((volume(&built)-expected).abs() <= 1e-9*expected,"{} against {expected}",volume(&built));
    agrees(&built,&tool,&path);
}

/// **A frustum slid aslant closes into its shadow carried**: `build::cone`'s frustum, radii 2 and 1
/// three apart along z, slid by `D` at `θ` from its axis. Its side's two rulings where its normal
/// is square to the slide (`A cos u + B sin u = C`) sweep their tangent planes, its rims oblique
/// cylinders. Apart from the build: its shadow along `D` is the hull of its two rims' ellipses, of
/// one aspect `cos θ` along the axis's shadow, so stretched by `1/cos θ` that way the hull of two
/// circles of radii `r₀ ≥ r₁` whose centres stand `L = h tan θ` apart: `r₀² (π + 2β)/2 + r₁² (π −
/// 2β)/2 + (r₀ + r₁) L cos β`, `sin β = (r₀ − r₁)/L`.
#[test]
fn a_frustum_slid_aslant_closes_into_its_shadow_carried() {
    use std::f64::consts::PI;
    let (r0,r1,h) = (2.,1.,3.);
    let tool = gcs_core::brep::build::cone([0.;3],[0.,0.,1.],r0,r1,h).unwrap();
    let d = [2.,0.5,1.5];
    let path = slide(d);
    let built = slid(&tool,&path,[0.,1.],0.002,1e-6).unwrap();
    let len = norm(d);
    let cos = d[2]/len;
    let l = h*(1.-cos*cos).sqrt()/cos;
    let beta = ((r0-r1)/l).asin();
    let hull = r0*r0*(PI+2.*beta)/2.+r1*r1*(PI-2.*beta)/2.+(r0+r1)*l*beta.cos();
    let expected = PI*h*(r0*r0+r0*r1+r1*r1)/3.+len*cos*hull;
    assert!((volume(&built)-expected).abs() <= 1e-9*expected,"{} against {expected}",volume(&built));
    agrees(&built,&tool,&path);
}

/// **A drum slid along its own axis is a longer drum**: its side is in contact all over, swept
/// along itself into one cylinder; `π r² (L + |D|)`.
#[test]
fn a_drum_slid_along_its_axis_is_a_longer_drum() {
    let tool = drum();
    let path = slide([3.,0.,0.]);
    let built = slid(&tool,&path,[0.,1.],0.002,1e-6).unwrap();
    let expected = std::f64::consts::PI*4.*11.;
    assert!((volume(&built)-expected).abs() <= 1e-9*expected,"{} against {expected}",volume(&built));
    agrees(&built,&tool,&path);
}

/// A capsule along z: a cylinder of radius `r` from 0 to `h`, a half ball on each end.
fn capsule(r: f64,h: f64) -> Brep {
    use gcs_core::brep::build::{Profile,ProfileEdge,revolve};
    use gcs_core::brep::geom::Frame;
    let f = Frame::about([0.;3],[0.,0.,1.]);
    let arc = |z: f64,span: [f64;2]| ProfileEdge::Arc {frame:Frame::new(f.at([0.,0.,z]),f.y,f.z),r,span:Some(span)};
    let half = std::f64::consts::FRAC_PI_2;
    let profile = Profile {origin:f.o,normal:f.y,names:Vec::new(),loops:vec![vec![
        arc(0.,[half,2.*half]),ProfileEdge::Line {a:f.at([r,0.,0.]),b:f.at([r,0.,h])},arc(h,[0.,half]),
        ProfileEdge::Line {a:f.at([0.,0.,h+r]),b:f.at([0.,0.,-r])}]]};
    revolve(&profile,f.o,f.z,std::f64::consts::TAU).unwrap()
}

/// **A capsule slid aslant closes into its shadow carried**: its half balls' contacts are their
/// great circles square to the slide, each swept into the cylinder of its radius along the
/// centre's path; its side's two rulings sweep their tangent planes. Its shadow is a disc and the
/// rectangle between, `π r² + 2 r h sin θ`.
#[test]
fn a_capsule_slid_aslant_closes_into_its_shadow_carried() {
    use std::f64::consts::PI;
    let (r,h) = (1.,3.);
    let tool = capsule(r,h);
    let d = [2.,0.5,1.5];
    let path = slide(d);
    let built = slid(&tool,&path,[0.,1.],0.002,1e-6).unwrap();
    let len = norm(d);
    let sin = (1.-(d[2]/len).powi(2)).sqrt();
    let expected = PI*r*r*h+4./3.*PI*r*r*r+len*(PI*r*r+2.*r*h*sin);
    assert!((volume(&built)-expected).abs() <= 1e-9*expected,"{} against {expected}",volume(&built));
    agrees(&built,&tool,&path);
}
