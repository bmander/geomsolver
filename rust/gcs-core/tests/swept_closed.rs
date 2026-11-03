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
fn agrees(built: &Brep,tool: &Brep,path: &Trajectory) { agrees_over(built,tool,path,[0.,1.]) }

/// `agrees` over the roll `roll`.
fn agrees_over(built: &Brep,tool: &Brep,path: &Trajectory,roll: [f64;2]) {
    let (solid,body) = (Solid::new(tool,0.002,1.).unwrap(),Solid::new(built,0.002,1.).unwrap());
    let swept = Swept {tool:&solid,path,roll};
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
fn agrees_cut(built: &Brep,blank: &Brep,tool: &Brep,path: &Trajectory) { agrees_cut_over(built,blank,tool,path,[0.,1.]) }

/// `agrees_cut` over the roll `roll`.
fn agrees_cut_over(built: &Brep,blank: &Brep,tool: &Brep,path: &Trajectory,roll: [f64;2]) {
    let (solid,body,stock) = (Solid::new(tool,0.002,1.).unwrap(),Solid::new(built,0.002,1.).unwrap(),Solid::new(blank,0.002,1.).unwrap());
    let swept = Swept {tool:&solid,path,roll};
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

/// **Round tools turned part of a turn about an axis along their own** (#721 F2c5, a curved tool
/// under a turn): a drum, a frustum and a ball beside the axis, each turned 100°. A cylinder's or a
/// cone's normal passes through the turn's axis along its two generators in the plane through both
/// axes, which turn into a cylinder or a cone about the turn's; a ball's along its great circle in
/// the plane through the axis and its centre, which turns into a torus; their rims, square to the
/// axis, turn within their own planes. Apart from the build, by Pappus: a disc of radius `r` whose
/// centre stands `D` from the axis, turned by `Θ`, covers itself and the annular sector between
/// `D ∓ r`, `π r² + 2 Θ D r`, so a drum sweeps `h (π r² + 2 Θ D r)`, a frustum that integrated over
/// its slices, `π h (r₀² + r₀ r₁ + r₁²)/3 + Θ D h (r₀ + r₁)`, and a ball `4/3 π r³ + Θ D π r²`.
#[test]
fn round_tools_turned_part_of_a_turn_sweep_their_sections_by_pappus() {
    use gcs_core::brep::build::{cone,disc,prism,sphere};
    use std::f64::consts::PI;
    let theta = 100f64.to_radians();
    let path = turn([0.;3],[0.,0.,1.],theta);
    let d = 6.;
    let cases: Vec<(&str,Brep,f64)> = vec![
        ("a drum",prism(&disc([d,0.,0.],[0.,0.,1.],1.5),0.,2.).unwrap(),2.*(PI*2.25+2.*theta*d*1.5)),
        ("a frustum",cone([d,0.,0.],[0.,0.,1.],2.,1.,3.).unwrap(),PI*3.*(4.+2.+1.)/3.+theta*d*3.*3.),
        ("a ball",sphere([d,0.,0.5],unit([0.2,0.3,1.]),1.5).unwrap(),4./3.*PI*1.5f64.powi(3)+theta*d*PI*2.25),
    ];
    for (what,tool,expected) in cases {
        let built = closed(&tool,&path,[0.,1.],0.002,1e-6).unwrap_or_else(|e| panic!("{what}: {e:?}"));
        assert!((volume(&built)-expected).abs() <= 1e-9*expected,"{what}: {} against {expected}",volume(&built));
        agrees(&built,&tool,&path);
    }
}

/// **A toolpath of a slide and a turn closes into its pieces' sweeps united** (#721 K1, a path of
/// pieces): a unit cube slid 4 along x, then turned a quarter turn clockwise about the vertical edge
/// it stops beside, at x = 5 — the turn a constant twist placed where the slide left the cube
/// (`Trajectory::Placed`), the two one path (`Trajectory::pieces`). Every piece's candidates split
/// one box at once, and each cell is read in the sweep of any piece (`member` piece by piece).
/// Apart from the build: the slide covers `[0, 5] × [0, 1]`, the turn its square turned about the
/// corner, `1 + π/2` (at each angle about the corner the farthest the square reaches over the turn),
/// overlapping the slide's in that square alone: `5 + π/2` through its height 1.
#[test]
fn a_toolpath_of_a_slide_and_a_turn_is_its_pieces_sweeps_united() {
    use gcs_core::brep::trajectory::Piece;
    let tool = block([0.;3],[1.,1.,1.]);
    let slide = Screw {origin:[0.;3],axis:[1.,0.,0.],ratio:0.,advance:4.*std::f64::consts::TAU};
    let turn = Screw {origin:[5.,0.,0.],axis:[0.,0.,1.],ratio:-std::f64::consts::FRAC_PI_2,advance:0.};
    let at = slide.pose(1.).then(turn.pose(1.).inverse());
    let path = Trajectory::pieces(vec![Piece {over:[0.,1.],path:Trajectory::Twist(slide)},
        Piece {over:[1.,2.],path:Trajectory::Placed {twist:turn,at}}],1e-12,10.).unwrap();
    let built = closed(&tool,&path,[0.,2.],0.002,1e-6).unwrap();
    let expected = 5.+std::f64::consts::FRAC_PI_2;
    assert!((volume(&built)-expected).abs() <= 1e-9*expected,"{} against {expected}",volume(&built));
    // membership over the whole path, at seeded points of the built solid's box
    let (solid,body) = (Solid::new(&tool,0.002,1.).unwrap(),Solid::new(&built,0.002,1.).unwrap());
    let swept = Swept {tool:&solid,path:&path,roll:[0.,2.]};
    let (lo,hi) = built.bounds();
    let mut rng = gcs_core::rng::Rng::new(9);
    let mut held = 0;
    for _ in 0..300 {
        let p: [f64;3] = std::array::from_fn(|k| rng.uniform(lo[k]-0.2,hi[k]+0.2));
        let d = body.signed_distance(p);
        if d.abs() < 0.01 { continue }
        let Ok(member) = swept.member(p) else { continue };
        assert_eq!(member,d < 0.,"at {p:?}");
        held += 1;
    }
    assert!(held > 200,"{held} points held");
}

/// **A level circle screwed meets what the screw's axis runs along in closed form** (#721 F2c5, a
/// round tool's rim under a screw; `ssi`'s screwed circle): the sheet a disc's rim screws into (a
/// drum's top, 3 from the axis, screwed 1.5 rad and a step of 2 a turn) meets a plane square to the
/// axis in the circle at that height, a cylinder about the axis in the helices of its points at that
/// radius (touching it along one where the circle reaches no farther), and the drum's own side at
/// either end pose in the circle there and the curve its two branches `θ±(v)` make leaving it —
/// every point of each, read along it, on both surfaces to rounding.
#[test]
fn a_screwed_circle_meets_planes_and_cylinders_in_closed_form() {
    use gcs_core::brep::build::{disc,prism,screwed};
    use gcs_core::brep::geom::{Curve,Frame,Rigid,Surface};
    use gcs_core::brep::ssi::{intersect,Ssi};
    let lift = 2./std::f64::consts::TAU;
    let coin = screwed(&disc([3.,0.,1.],[0.,0.,1.],1.),[0.;3],[0.,0.,1.],1.5,lift).unwrap();
    let sheet = coin.faces.iter().find(|f| f.surface.kind() == "blend").unwrap().surface.clone();
    let side = |m: &Rigid| prism(&disc([3.,0.,0.],[0.,0.,1.],1.),0.,1.).unwrap().moved(m).faces.iter()
        .find(|f| f.surface.kind() == "cylinder").unwrap().surface.clone();
    let end = Rigid::turn([0.;3],[0.,0.,1.],1.5).then(&Rigid {t:[0.,0.,lift],..Rigid::identity()});
    let cases: Vec<(&str,Surface,&[&str])> = vec![
        ("a level plane",Surface::Plane(Frame::about([0.,0.,1.2],[0.,0.,1.])),&["circle"]),
        ("the cylinder of the rim's farthest radius",Surface::Cylinder(Frame::about([0.;3],[0.,0.,1.]),4.),&["iso"]),
        ("a cylinder about the axis across the rim",Surface::Cylinder(Frame::about([0.;3],[0.,0.,1.]),3.2),&["iso","iso"]),
        ("the drum's side where it starts",side(&Rigid::identity()),&["circle","traced"]),
        ("the drum's side where it ends",side(&end),&["circle","traced"]),
    ];
    for (what,other,kinds) in cases {
        let Ssi::Curves(curves) = intersect(&sheet,&other,1e-6) else { panic!("{what}: curves") };
        let mut found: Vec<&str> = curves.iter().map(|c| c.kind()).collect();
        found.sort();
        assert_eq!(found,kinds,"{what}");
        for c in &curves {
            let points: Vec<[f64;3]> = match c {
                Curve::Traced(t) => t.pts.clone(),
                Curve::Iso(..) => (0..=32).map(|j| c.point(j as f64/32.)).collect(),
                _ => (0..64).map(|j| c.point(j as f64*std::f64::consts::TAU/64.)).collect(),
            };
            for p in points {
                let off = sheet.implicit(p).abs().max(other.implicit(p).abs());
                assert!(off <= 1e-9,"{what}: a point of its {} {off:e} off",c.kind());
            }
        }
    }
}

/// **A drum screwed beside the axis closes into its helical sweep** (#721 F2c5, a round tool under a
/// screw: helical milling): a drum of radius 1 and height 1 standing 3 from the screw's axis, along
/// it, screwed 1.5 rad with a step of 2 a roll. Its side's generators in the plane through both axes
/// screw into cylinders about the axis, its rims into the solids their discs sweep, each rim's sheet
/// meeting the drum's side at its end pose in closed form (`ssi`'s screwed circle). Apart from the
/// build, in helical coordinates `(ρ, ψ, ζ = z − k ψ)`: the drum is less tall than a lead, so each
/// helix meets it once, over an arc `2α(ρ)` at radius `ρ`, which the turn lengthens by `Θ`; the sweep
/// is the drum's own volume and `Θ ∫ ρ (h + 2kα) dρ = Θ (h (4² − 2²)/2 + k π)`, `π` the disc's polar
/// integral: `π + Θ (6 + k π)`.
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, about 2 min: a drum's screwed rims on one arrangement")]
fn a_drum_screwed_beside_the_axis_closes_into_its_helical_sweep() {
    use gcs_core::brep::build::{disc,prism};
    use std::f64::consts::{PI,TAU};
    let tool = prism(&disc([3.,0.,0.],[0.,0.,1.],1.),0.,1.).unwrap();
    let (theta,advance) = (1.5,2.);
    let path = Trajectory::Twist(Screw {origin:[0.;3],axis:[0.,0.,1.],ratio:theta,advance});
    let k = advance/TAU/theta;
    let built = closed(&tool,&path,[0.,1.],0.002,1e-6).unwrap();
    let expected = PI+theta*(6.+k*PI);
    assert!((volume(&built)-expected).abs() <= 1e-9*expected,"{} against {expected}",volume(&built));
    agrees(&built,&tool,&path);
}

/// A drum of radius 1 and height 1 standing 10 from a screw's axis, along it, its disc screwed by
/// `lead` a turn through `turn` radians: the sheet its rim sweeps bounds a thin helical ramp.
fn ramp(lead: f64,turn: f64,at: f64) -> Brep {
    let rise = lead/std::f64::consts::TAU*turn;
    let (s,c) = at.sin_cos();
    gcs_core::brep::build::screwed(&gcs_core::brep::build::disc([10.*c,10.*s,-0.5],[0.,0.,1.],1.),[0.;3],[0.,0.,1.],turn,rise).unwrap()
}

/// **A line meets the sheet a screwed rim sweeps where it does, exactly** (#721 F2c5): the circle level
/// with a point of the line is at the turn its height gives, and the line's distance from it is
/// bounded over each stretch, so both crossings of a ramp thinner than any step a sampled search
/// would take are found. Apart from the kernel: a point is in the ramp exactly where the disc,
/// screwed back by the turn its height gives, holds it — and the solid's point location (a ray's
/// crossings) agrees at every point drawn about the ramp, half a millimetre a turn and two.
#[test]
fn a_line_meets_a_screwed_rims_sheet_exactly() {
    use gcs_core::brep::query::{Located,Place};
    for lead in [0.5,2.] {
        let theta = 280f64.to_radians();
        let rise = lead/std::f64::consts::TAU*theta;
        let ramp = ramp(lead,theta,0.);
        let at = Located::new(&ramp,1e-6);
        let mut rng = gcs_core::rng::Rng::new(3);
        for _ in 0..400 {
            let (r,a) = (rng.uniform(8.8,11.2),rng.uniform(-0.2,theta+0.2));
            let z = -0.5+rise*a/theta+rng.uniform(-0.03,0.03);
            let p = [r*a.cos(),r*a.sin(),z];
            let turn = (z+0.5)/rise*theta;
            let (s,c) = (-turn).sin_cos();
            let q = [c*p[0]-s*p[1],s*p[0]+c*p[1]];
            let inside = (0.0..=theta).contains(&turn) && (q[0]-10.).hypot(q[1]) < 1.;
            let place = at.solid_place(p);
            assert!(place != Place::On && (place == Place::In) == inside,"lead {lead}: at {p:?} {place:?}, the disc screwed back {inside}");
        }
    }
}

/// **Planes through a screw's axis cut a screwed rim's ramp in closed form** (#721 F2c5): each level
/// circle meets the plane where `A cos θ + B sin θ = C(v)`, the two roots joining where the circle
/// touches it, handed over as one loop of exact points. Apart from the kernel, in helical
/// coordinates: the ramp passes wholly across a sixth of a turn, so it holds the disc's area times
/// the lead's rate over that sixth, `π/3 · k π`.
#[test]
fn planes_through_the_axis_cut_a_screwed_rims_ramp_in_closed_form() {
    use gcs_core::brep::build::{revolve,Profile,ProfileEdge};
    use gcs_core::brep::boolean::selected;
    use gcs_core::brep::geom::Frame;
    use std::f64::consts::PI;
    let f = Frame::about([0.;3],[0.,0.,1.]);
    let at = |x: f64,z: f64| f.at([x,0.,z]);
    let rect = [at(8.,-1.),at(12.,-1.),at(12.,1.),at(8.,1.)];
    let profile = Profile {origin:rect[0],normal:f.y,names:Vec::new(),loops:vec![(0..4).map(|k| ProfileEdge::Line {a:rect[k],b:rect[(k+1)%4]}).collect()]};
    let sector = revolve(&profile,[0.;3],[0.,0.,1.],PI/3.).unwrap();
    let theta = 280f64.to_radians();
    let k = 0.5/std::f64::consts::TAU;
    let r = selected(&[&ramp(0.5,theta,-theta/2.),&sector],&|m| m[0] && m[1],1e-6).unwrap();
    let expected = PI/3.*k*PI;
    assert!((volume(&r.brep)-expected).abs() <= 1e-6*expected,"{} against {expected}",volume(&r.brep));
}

/// **A drum screwed a short lead through most of a turn closes into its helical sweep** (#721 F2c5):
/// the drum of `ramp`, screwed half a millimetre a turn through 280°, its rims' ramps thinner than a
/// hundredth of a millimetre where they fold against the cylinders its side sweeps. Apart from the
/// build, as `a_drum_screwed_beside_the_axis_closes_into_its_helical_sweep`'s: `π + Θ (20 + k π)`.
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, about a minute: a drum's screwed rims on one arrangement")]
fn a_drum_screwed_a_short_lead_through_most_of_a_turn_closes() {
    use gcs_core::brep::build::{disc,prism};
    use std::f64::consts::{PI,TAU};
    let tool = prism(&disc([10.,0.,-0.5],[0.,0.,1.],1.),0.,1.).unwrap();
    let theta = 280f64.to_radians();
    let path = Trajectory::Twist(Screw {origin:[0.;3],axis:[0.,0.,1.],ratio:1.,advance:0.5});
    let built = closed(&tool,&path,[0.,theta],0.002,1e-6).unwrap();
    let expected = PI+theta*(20.+0.5/TAU*PI);
    assert!((volume(&built)-expected).abs() <= 1e-9*expected,"{} against {expected}",volume(&built));
    agrees_over(&built,&tool,&path,[0.,theta]);
}

/// **A drum screwed across a ring sector bores its helical groove on one arrangement** (#721 F2c5,
/// #699 C): the drum of `ramp` screwed half a millimetre a turn through 280° about the middle of a
/// sixth of a ring 8 to 12 across and 2 high, which it passes wholly across, the sector its blank.
/// The sector's planes through the axis cut the rims' ramps in closed form
/// (`planes_through_the_axis_cut_a_screwed_rims_ramp_in_closed_form`). Apart from the build: at each
/// point of the sector the heights the drum covers are its own and twice its half-width in angle
/// there, `a(ρ)`, times the lead's rate `k`, so it takes `π/3 ∫ ρ (1 + 2 k a(ρ)) dρ = π/3 (20 + k π)`
/// of the sector's `π/3 · 80`.
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, about 20 s: a drum's screwed rims through a sector")]
fn a_drum_screwed_across_a_ring_sector_bores_its_helical_groove() {
    use gcs_core::brep::build::{disc,prism,revolve,Profile,ProfileEdge};
    use gcs_core::brep::geom::Frame;
    use gcs_core::brep::swept::closed::cut_from;
    use std::f64::consts::{PI,TAU};
    let f = Frame::about([0.;3],[0.,0.,1.]);
    let at = |x: f64,z: f64| f.at([x,0.,z]);
    let rect = [at(8.,-1.),at(12.,-1.),at(12.,1.),at(8.,1.)];
    let profile = Profile {origin:rect[0],normal:f.y,names:Vec::new(),loops:vec![(0..4).map(|k| ProfileEdge::Line {a:rect[k],b:rect[(k+1)%4]}).collect()]};
    let sector = revolve(&profile,[0.;3],[0.,0.,1.],PI/3.).unwrap();
    let tool = prism(&disc([10.,0.,-0.5],[0.,0.,1.],1.),0.,1.).unwrap();
    let theta = 280f64.to_radians();
    let path = Trajectory::Twist(Screw {origin:[0.;3],axis:[0.,0.,1.],ratio:1.,advance:0.5});
    let roll = [-theta/2.,theta/2.];
    let cut = cut_from(&sector,&tool,&path,roll,0.002,1e-6).unwrap();
    let expected = PI/3.*80.-PI/3.*(20.+0.5/TAU*PI);
    assert!((volume(&cut)-expected).abs() <= 1e-8*expected,"{} against {expected}",volume(&cut));
    agrees_cut_over(&cut,&sector,&tool,&path,roll);
}


/// **Two tori touching round a meridian, and a plane both their tops touch, hold every piece**
/// (#721 F2c5, `ssi`'s `tori_touching`): a ring of middle radius 1 and tube 0.3 standing 10 from an
/// axis, the torus of middle radius 11 about that axis its outer meridian turns into, and the plane
/// on both tubes' tops. The tori touch round the meridian and cross beside it, branches that reach
/// the meridian's top tangent to the ring's top circle, where the plane touches both: the three meet
/// pairwise tangent at one point. Read between their exact points on a parabola, not a chord (whose
/// sag there put the branch on the ring's other side, and its piece of the big torus went missing),
/// each piece is there: the big torus less the ring is its volume less their common part, with the
/// plane or without, to 1e-6 (the pieces the branches bound read by their traced points).
#[test]
fn two_tori_touching_and_a_plane_on_both_hold_every_piece() {
    use gcs_core::brep::boolean::selected;
    let outer = gcs_core::brep::build::torus([0.;3],[0.,0.,1.],11.,0.3).unwrap();
    let ring = gcs_core::brep::build::torus([10.,0.,0.],[0.,0.,1.],1.,0.3).unwrap();
    let up = crate::brep::block([-20.,-20.,0.3],[20.,20.,5.]);
    let part = |ops: &[&Brep],pattern: [bool;2]| selected(ops,&|m| m[..2] == pattern && (m.len() < 3 || !m[2]),1e-6)
        .map(|s| volume(&s.brep)).unwrap();
    let common = part(&[&ring,&outer],[true,true]);
    for ops in [vec![&ring,&outer],vec![&ring,&outer,&up]] {
        let (only,both) = (part(&ops,[false,true]),part(&ops,[true,true]));
        assert!((only+both-volume(&outer)).abs() <= 1e-6*volume(&outer),"{only} and {both} against {}",volume(&outer));
        assert!((both-common).abs() <= 1e-6*common,"{both} against {common}");
        assert!((part(&ops,[true,false])+both-volume(&ring)).abs() <= 1e-6*volume(&ring));
    }
}

/// **A ring turned about an axis along its own closes into its sweep** (#721 F2c5, a torus under a
/// turn): a torus of middle radius 1 and tube 0.3 standing 10 from the turn's axis, turned 1.5 rad.
/// It touches round its two meridians in the plane through both axes, turned into tori about the
/// axis (of middle radii 9 and 11), and round its top and bottom rings, kept in their planes; each
/// end pose touches each meridian's torus round that meridian (`ssi`'s `tori_touching`). Apart from
/// the build: the turn is longer than the angle the ring's hole spans at any radius, so each level
/// `z` of it sweeps its outer disc, radius `R + w` (`w = √(r² − z²)`), and the band that disc's
/// shadow turns through, `2 d Θ (R + w)`: `π (2 r R² + π R r² + 4 r³/3) + 2 d Θ (2 r R + π r²/2)`.
#[test]
fn a_ring_turned_about_an_axis_along_its_own_closes_into_its_sweep() {
    use std::f64::consts::PI;
    let (big,r,d,theta) = (1.,0.3,10.,1.5);
    let tool = gcs_core::brep::build::torus([d,0.,0.],[0.,0.,1.],big,r).unwrap();
    let path = Trajectory::Twist(Screw {origin:[0.;3],axis:[0.,0.,1.],ratio:theta,advance:0.});
    let built = closed(&tool,&path,[0.,1.],0.002,1e-6).unwrap();
    let expected = PI*(2.*r*big*big+PI*big*r*r+4.*r*r*r/3.)+2.*d*theta*(2.*r*big+PI*r*r/2.);
    assert!((volume(&built)-expected).abs() <= 1e-9*expected,"{} against {expected}",volume(&built));
    agrees(&built,&tool,&path);
}

/// A bull-nose mill of radius `big`, corner `c` and height `h`, standing about an upright axis `d`
/// out along x, its flat end on `z = 0`.
fn bull_nose(d: f64,big: f64,c: f64,h: f64) -> Brep {
    use gcs_core::brep::build::{revolve,Profile,ProfileEdge};
    use gcs_core::brep::geom::Frame;
    let at = |x: f64,z: f64| [d+x,0.,z];
    let corner = Frame::new(at(big-c,c),[0.,-1.,0.],[1.,0.,0.]);
    let edges = vec![ProfileEdge::Line {a:at(0.,0.),b:at(big-c,0.)},
        ProfileEdge::Arc {frame:corner,r:c,span:Some([-std::f64::consts::FRAC_PI_2,0.])},
        ProfileEdge::Line {a:at(big,c),b:at(big,h)},ProfileEdge::Line {a:at(big,h),b:at(0.,h)},ProfileEdge::Line {a:at(0.,h),b:at(0.,0.)}];
    let profile = Profile {origin:at(0.,0.),normal:[0.,1.,0.],names:Vec::new(),loops:vec![edges]};
    revolve(&profile,[d,0.,0.],[0.,0.,1.],std::f64::consts::TAU).unwrap()
}

/// **A bull-nose mill turned about an axis along its own closes into its sweep** (#721 F2c5): radius
/// 1, corner 0.3 and 1 tall, 10 from the turn's axis, turned 1.5 rad. Its round, a quarter of a torus,
/// touches round the quarter of its outer meridian, turned into a torus about the axis, and its flat
/// end, side and top as a drum's do. Apart from the build: each level is a disc of radius `R(z)`, so
/// the turn sweeps the mill and the band its shadow turns through, `π ∫R² dz + 2 d Θ ∫R dz`.
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, about a minute: a bull-nose's round turned on one arrangement")]
fn a_bull_nose_turned_about_an_axis_along_its_own_closes_into_its_sweep() {
    use std::f64::consts::PI;
    let (d,big,c,h,theta) = (10.,1.,0.3,1.,1.5);
    let tool = bull_nose(d,big,c,h);
    let path = Trajectory::Twist(Screw {origin:[0.;3],axis:[0.,0.,1.],ratio:theta,advance:0.});
    let rz = (big-c)*c+PI*c*c/4.+big*(h-c);
    let r2z = (big-c)*(big-c)*c+(big-c)*PI*c*c/2.+2.*c*c*c/3.+big*big*(h-c);
    assert!((volume(&tool)-PI*r2z).abs() <= 1e-12*PI*r2z);
    let expected = PI*r2z+theta*2.*d*rz;
    let built = closed(&tool,&path,[0.,1.],0.002,1e-6).unwrap();
    assert!((volume(&built)-expected).abs() <= 1e-9*expected,"{} against {expected}",volume(&built));
    agrees(&built,&tool,&path);
}
