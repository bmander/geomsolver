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

/// **A block leaning to a screw's axis closes into its helical sweep** (#777 M5's gate: the leaning
/// block under a screw, docs/swept-solid-design.md's "tried and set aside, for a leaning edge under
/// a screw"): its leaning edges, and its leaning faces' contact lines (where a face's normal is
/// square to the screw's velocity), each screw into a patch of a ruled helicoid bounded inside the
/// domain, its rim standing in a cell just past the candidates that seal it, its meetings stopping
/// partway across the faces it meets. Held to its volume apart from the build
/// (`leaning_sweep_volume`) to 1e-6, and to membership at seeded points.
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, #777 M5: the leaning block's build takes minutes")]
fn a_block_leaning_to_a_screws_axis_closes_into_its_helical_sweep() {
    let (lean,off) = (0.35,3.);
    let tool = leaning(lean,off);
    let (theta,advance) = (1.,2.);
    let path = Trajectory::Twist(Screw {origin:[0.;3],axis:[0.,0.,1.],ratio:theta,advance});
    let expected = leaning_sweep_volume(lean,off,theta,advance/std::f64::consts::TAU/theta);
    eprintln!("a block leaning to a screw's axis: its sweep {expected} apart from the build");
    let built = closed(&tool,&path,[0.,1.],0.002,1e-6).unwrap();
    assert!((volume(&built)-expected).abs() <= 1e-6*expected,"{} against {expected}",volume(&built));
    agrees(&built,&tool,&path);
}

/// **The volume a block leaning to a screw's axis sweeps** (`leaning(lean, off)`, turned `Θ` about
/// the world's `z` and climbing `k` a radian), apart from the kernel. In helical coordinates
/// `(ρ, ψ, ζ = z − kψ)`, volume `ρ dρ dψ dζ`, the screw is a shift of `ψ` by `[0, Θ]`, so the sweep's
/// slice by the cylinder of radius `ρ` is the block's slice `S_ρ` stretched along `ψ` — its area
/// read line by line of constant `ψ'`: there the sweep holds `ζ` where some `ψ ∈ [ψ' − Θ, ψ']` does,
/// and on each run of `ψ` where the vertical line meets the block (between `z_lo(ψ)` and
/// `z_hi(ψ)`, clipped exactly) that is the interval from the least `z_lo − kψ` to the greatest
/// `z_hi − kψ`, read at the window's ends and at the run's interior extremes. The runs are where a
/// line meets the block at all — two of them at some radii, which is why a helix can meet the block
/// twice and the sweep is less than its own volume and `Θ ∫ ρ L(ρ) dρ`. The section is smooth
/// between the run ends and extremes and those shifted by `Θ`, integrated by Gauss–Legendre on
/// those pieces, and `ρ` likewise between the radii of the block's corners and of its edges'
/// points nearest the axis.
fn leaning_sweep_volume(lean: f64,off: f64,theta: f64,k: f64) -> f64 {
    let (sn,cs) = lean.sin_cos();
    // the vertical line at (x, y): the block's bottom and top along it, and how far it is from
    // missing the block (positive where it meets it); local (x − off, c y + s z, −s y + c z)
    let line = |x: f64,y: f64| -> (f64,f64,f64) {
        let (mut lo,mut hi) = (f64::NEG_INFINITY,f64::INFINITY);
        for (a,b,h) in [(cs*y,sn,0.5),(-sn*y,cs,0.75)] {
            let (z0,z1) = ((-h-a)/b,(h-a)/b);
            lo = lo.max(z0.min(z1));
            hi = hi.min(z0.max(z1));
        }
        (lo,hi,(x-off).min(1.-(x-off)).min(hi-lo))
    };
    // a sign change of `f` on [a, b] settled, and a least of `f` on [a, b]
    let root = |f: &dyn Fn(f64) -> f64,mut a: f64,mut b: f64| {
        let fa = f(a) > 0.;
        for _ in 0..100 { let m = 0.5*(a+b); if (f(m) > 0.) == fa { a = m } else { b = m } }
        0.5*(a+b)
    };
    let least = |f: &dyn Fn(f64) -> f64,mut a: f64,mut b: f64| {
        for _ in 0..100 { let (m1,m2) = (a+(b-a)*0.381966,a+(b-a)*0.618034); if f(m1) < f(m2) { b = m2 } else { a = m1 } }
        0.5*(a+b)
    };
    let corners: Vec<[f64;3]> = (0..8).map(|i| {
        let (x,y,z) = (off+(i&1) as f64,if i&2 == 0 { -0.5 } else { 0.5 },if i&4 == 0 { -0.75 } else { 0.75 });
        [x,cs*y-sn*z,sn*y+cs*z]
    }).collect();
    let pieces = |breaks: &[f64],n: usize,f: &dyn Fn(f64) -> f64| -> f64 {
        let mut sum = 0.;
        for w in breaks.windows(2) { for j in 0..n {
            let (a,b) = (w[0]+(w[1]-w[0])*j as f64/n as f64,w[0]+(w[1]-w[0])*(j+1) as f64/n as f64);
            for (x,g) in gcs_core::integral::GAUSS8 { sum += 0.5*(b-a)*g*f(0.5*(a+b)+0.5*(b-a)*x) }
        } }
        sum
    };
    let area = |rho: f64| -> f64 {
        let at = |p: f64| line(rho*p.cos(),rho*p.sin());
        let gap = |p: f64| at(p).2;
        // the runs of ψ where the line meets the block, a run between samples found at its gap's
        // greatest
        let n = 1000;
        let ps: Vec<f64> = (0..=n).map(|i| -0.7+1.4*i as f64/n as f64).collect();
        let g: Vec<f64> = ps.iter().map(|&p| gap(p)).collect();
        let mut runs = vec![];
        let mut start = None;
        for i in 0..=n {
            if g[i] > 0. && start.is_none() { start = Some(if i > 0 { root(&gap,ps[i-1],ps[i]) } else { ps[0] }) }
            if g[i] > 0. && (i == n || g[i+1] <= 0.) {
                runs.push((start.take().unwrap(),if i < n { root(&gap,ps[i],ps[i+1]) } else { ps[n] }));
            }
            if i > 0 && i < n && g[i] <= 0. && g[i] >= g[i-1] && g[i] >= g[i+1] {
                let p = least(&|q| -gap(q),ps[i-1],ps[i+1]);
                if gap(p) > 0. { runs.push((root(&gap,ps[i-1],p),root(&gap,ps[i+1],p))) }
            }
        }
        if runs.is_empty() { return 0. }
        // `z_lo − kψ` and `−(z_hi − kψ)`, each least at a run's ends and its interior minima
        let bottom = |p: f64| at(p).0-k*p;
        let top = |p: f64| -(at(p).1-k*p);
        let ends: [&dyn Fn(f64) -> f64;2] = [&bottom,&top];
        let minima: Vec<[Vec<(f64,f64)>;2]> = runs.iter().map(|&(a,b)| ends.map(|f| {
            let m = 200;
            let qs: Vec<f64> = (0..=m).map(|t| a+(b-a)*t as f64/m as f64).collect();
            let v: Vec<f64> = qs.iter().map(|&q| f(q)).collect();
            (1..m).filter(|&t| v[t] <= v[t-1] && v[t] <= v[t+1])
                .map(|t| { let q = least(f,qs[t-1],qs[t+1]); (q,f(q)) }).collect()
        })).collect();
        // the sweep's section at ψ': the union of each run's interval over the window
        let section = |pp: f64| -> f64 {
            let mut spans: Vec<(f64,f64)> = vec![];
            for (&(a,b),mins) in runs.iter().zip(&minima) {
                let (u,v) = (a.max(pp-theta),b.min(pp));
                if u > v { continue }
                let low = |j: usize| mins[j].iter().filter(|&&(q,_)| u < q && q < v).map(|&(_,f)| f)
                    .fold(ends[j](u).min(ends[j](v)),f64::min);
                spans.push((low(0),-low(1)));
            }
            spans.sort_by(|x,y| x.0.total_cmp(&y.0));
            let (mut length,mut current) = (0.,None::<(f64,f64)>);
            for (x,y) in spans {
                current = match current {
                    Some((c0,c1)) if x <= c1 => Some((c0,c1.max(y))),
                    Some((c0,c1)) => { length += c1-c0; Some((x,y)) }
                    None => Some((x,y)),
                };
            }
            length+current.map_or(0.,|(c0,c1)| c1-c0)
        };
        let mut breaks: Vec<f64> = runs.iter().zip(&minima).flat_map(|(&(a,b),mins)|
            [a,b].into_iter().chain(mins.iter().flatten().map(|&(q,_)| q)).collect::<Vec<_>>())
            .flat_map(|q| [q,q+theta]).collect();
        breaks.sort_by(f64::total_cmp);
        breaks.dedup();
        pieces(&breaks,4,&section)
    };
    // ρ from the block's face nearest the axis to its farthest corner, broken at the corners' radii
    // and at each edge's point nearest the axis
    let radius = |p: [f64;3]| p[0].hypot(p[1]);
    let mut radii: Vec<f64> = corners.iter().map(|&p| radius(p)).collect();
    radii.push(off);
    for i in 0..8usize { for j in 0..8usize { if (i^j).count_ones() == 1 && i < j {
        let (a,b) = (corners[i],corners[j]);
        let (dx,dy) = (b[0]-a[0],b[1]-a[1]);
        let q = dx*dx+dy*dy;
        if q <= 1e-24 { continue }
        let t = -(a[0]*dx+a[1]*dy)/q;
        if 0. < t && t < 1. { radii.push((a[0]+t*dx).hypot(a[1]+t*dy)) }
    } } }
    let far = radii.iter().copied().fold(0.,f64::max);
    radii.retain(|&r| r >= off);
    radii.sort_by(f64::total_cmp);
    radii.dedup_by(|a,b| (*a-*b).abs() <= 1e-12);
    debug_assert!(radii.last() == Some(&far));
    pieces(&radii,64,&|rho| rho*area(rho))
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


/// **A ball screwed closes into the pipe about its centre's helix** (#721 F2c5, a ball under a
/// screw): a ball of radius 1 standing 10 from a screw's axis, turned 1.5 rad while it climbs half a
/// unit a radian. It touches round its great circle square to its centre's velocity, which the
/// screw carries into the pipe about its centre's helix (`Cutter::Pipe`, the disc there screwed),
/// and at each end pose the ball touches the pipe round that circle (`ssi`'s `pipe_end`). Apart from
/// the build: a tube about a curve is its section times the curve's length, here the helix's
/// `Θ √(d² + k²)`, and the two half balls at its ends a ball: `4π/3 + π Θ √(d² + k²)`.
#[test]
fn a_ball_screwed_closes_into_the_pipe_about_its_centres_helix() {
    use std::f64::consts::{PI,TAU};
    let (r,d,theta,k) = (1.,10.,1.5,0.5);
    let tool = gcs_core::brep::build::sphere([d,0.,0.],[0.,0.,1.],r).unwrap();
    let path = Trajectory::Twist(Screw {origin:[0.;3],axis:[0.,0.,1.],ratio:theta,advance:TAU*k*theta});
    let built = closed(&tool,&path,[0.,1.],0.002,1e-6).unwrap();
    let expected = 4./3.*PI*r*r*r+PI*r*r*theta*(d*d+k*k).sqrt();
    assert!((volume(&built)-expected).abs() <= 1e-9*expected,"{} against {expected}",volume(&built));
    agrees(&built,&tool,&path);
}

/// The point `x` screwed back by `v` of the screw about the world's `z` axis turning `angle` and
/// rising `rise` over `v` in `[0, 1]`.
fn screwed_back(x: [f64;3],angle: f64,rise: f64,v: f64) -> [f64;3] {
    let (s,c) = (-angle*v).sin_cos();
    [c*x[0]-s*x[1],s*x[0]+c*x[1],x[2]-rise*v]
}

/// **A line grazing a hyperboloid touches it, read once** (`query::curve_surface`'s certified
/// reading of a line against a surface of revolution, `line_revolution`): the hyperboloid a leaning
/// edge turns into, `ρ = √(10² + k² z²)`, its meridian the rational conic the closed class lays
/// (`closed::meridian_arc`'s), and lines tangent to it at points of several heights, leaning in
/// its tangent plane off the horizontal by several angles (none a ruling). Each is one touch at
/// its point: a double root, which rounding can part into two simple ones a hair apart, read as a
/// crossing once cut the plane face of a leaning bar turned through a ring sector where its edge
/// only grazed the hyperboloid, and left a vertex on another edge's chord its mesh refused.
#[test]
fn a_line_grazing_a_hyperboloid_touches_it_once() {
    use gcs_core::brep::build::{revolve,Profile,ProfileEdge};
    use gcs_core::brep::geom::Curve;
    use gcs_core::brep::nurbs::BSpline;
    use gcs_core::brep::query::{curve_surface,Meets};
    let (throat,k) = (10.,20f64.to_radians().tan());
    let rho = |z: f64| (throat*throat+k*k*z*z).sqrt();
    let (za,zb) = (-1.,1.);
    let (a,b) = ([rho(za),za],[rho(zb),zb]);
    // the conic through the meridian's ends, its end tangents' meeting and its shoulder
    let slope = |z: f64| k*k*z/rho(z);
    let (ta,tb) = ([slope(za),1.],[slope(zb),1.]);
    let det = -ta[0]*tb[1]+ta[1]*tb[0];
    let t = (-(b[0]-a[0])*tb[1]+(b[1]-a[1])*tb[0])/det;
    let m = [a[0]+t*ta[0],a[1]+t*ta[1]];
    let mid = [0.5*(a[0]+b[0]),0.5*(a[1]+b[1])];
    let (dr,dz) = (m[0]-mid[0],m[1]-mid[1]);
    let (qa,qb,qc) = (dr*dr-k*k*dz*dz,2.*(mid[0]*dr-k*k*mid[1]*dz),mid[0]*mid[0]-throat*throat-k*k*mid[1]*mid[1]);
    let u = gcs_core::roots::quadratic(qa,qb,qc).into_iter().find(|u| *u > 0. && *u < 1.).unwrap();
    let at = |r: f64,z: f64| [r,0.,z];
    let conic = BSpline::rational(2,vec![0.,0.,0.,1.,1.,1.],vec![at(a[0],a[1]),at(m[0],m[1]),at(b[0],b[1])],vec![1.,u/(1.-u),1.]).unwrap();
    let edges = vec![ProfileEdge::Spline(std::sync::Arc::new(conic)),ProfileEdge::Line {a:at(b[0],zb),b:at(0.,zb)},
        ProfileEdge::Line {a:at(0.,zb),b:at(0.,za)},ProfileEdge::Line {a:at(0.,za),b:at(a[0],za)}];
    let solid = revolve(&Profile {origin:[0.;3],normal:[0.,1.,0.],names:Vec::new(),loops:vec![edges]},[0.;3],[0.,0.,1.],
        std::f64::consts::TAU).unwrap();
    let sheet = &solid.faces.iter().find(|f| f.surface.kind() == "revolution").expect("a hyperboloid").surface;
    let tol = 1e-6;
    let mut read = 0;
    for z in [-0.6385066634855452,-0.3,0.,0.25,0.6385066634855529] {
        for angle in [15f64,40.,65.,80.] {
            // at (ρ(z), 0, z): the tangent plane holds y and the meridian's tangent (ρ'(z), 0, 1)
            let p = [rho(z),0.,z];
            let along = unit([slope(z),0.,1.]);
            let (s,c) = angle.to_radians().sin_cos();
            let d = [c*0.+s*along[0],c*1.+s*along[1],s*along[2]];
            let line = Curve::Line {p:[p[0]-0.7*d[0],p[1]-0.7*d[1],p[2]-0.7*d[2]],d};
            match curve_surface(&line,[0.,1.4],sheet,tol) {
                Meets::At(found) => {
                    assert_eq!(found.len(),1,"z {z}, {angle}°: {found:?}");
                    let (t,touch) = found[0];
                    assert!(touch,"z {z}, {angle}°: read as a crossing at {t}");
                    assert!((t-0.7).abs() <= 1e-3,"z {z}, {angle}°: at {t}");
                }
                Meets::Along => panic!("z {z}, {angle}°: along"),
            }
            read += 1;
        }
    }
    assert_eq!(read,20);
}

/// **An edge is read by the readings a ray is, settled by one rule** (#780 R2): a line against a
/// cone and a torus — an edge a Boolean meets with them (`query::curve_surface`), not a ray — is
/// certified, never sampled (the profile's count, `util::meter`), and what it returns is settled
/// as every reader's is: a line tangent to the surface off its rulings one touch at its point, a
/// line through it its crossings, a ruling `Along`, and a line ending on the surface one crossing
/// exactly at its end. The cone's and the torus's polynomials once served rays alone, an edge
/// within the tolerance of tangent left to the samples because the two read it differently.
#[test]
fn an_edge_is_read_certified_and_settled_as_a_ray_is() {
    use gcs_core::brep::geom::{Curve,Frame,Surface};
    use gcs_core::brep::query::{curve_surface,Meets};
    use gcs_core::meter::{self as meter,Budget,Meter};
    let tol = 1e-7;
    let f = Frame::about([0.;3],[0.,0.,1.]);
    let at = |x: f64,y: f64,z: f64| -> [f64;3] { std::array::from_fn(|k| f.o[k]+x*f.x[k]+y*f.y[k]+z*f.z[k]) };
    let (r,a) = (1.,20f64.to_radians());
    let cone = Surface::Cone(f,r,a);
    let (big,small) = (2.,0.5);
    let ring = Surface::Torus(f,big,small);
    let m = Meter::new(Budget::UNLIMITED);
    let line = |p: [f64;3],d: [f64;3]| Curve::Line {p,d:unit(d)};
    // a line from `p` through `q`, ending there
    let ending = |p: [f64;3],q: [f64;3]| -> (Curve,f64) {
        let d: [f64;3] = std::array::from_fn(|k| q[k]-p[k]);
        (line(p,d),norm(d))
    };
    let read = |c: &Curve,t: [f64;2],s: &Surface| meter::within(&m,|| curve_surface(c,t,s,tol));
    let one_touch = |what: &str,found: Meets,at: f64| {
        let Meets::At(found) = found else { panic!("{what}: along") };
        assert_eq!(found.len(),1,"{what}: {found:?}");
        assert!(found[0].1 && (found[0].0-at).abs() <= 1e-3,"{what}: {found:?}, a touch at {at}");
    };
    let one_end = |what: &str,found: Meets,end: f64| {
        assert_eq!(found,Meets::At(vec![(end,false)]),"{what}: a crossing exactly at its end {end}");
    };
    // the cone: tangent at its point over the x axis, leaning in its tangent plane off the ruling
    let k = a.tan();
    let ruling = unit([k,0.,1.]);
    for (z,turn) in [(0.,30f64),(0.4,55.),(-0.3,80.)] {
        let q = at(r+k*z,0.,z);
        let (s,c) = turn.to_radians().sin_cos();
        let d: [f64;3] = std::array::from_fn(|i| c*f.y[i]+s*ruling[i]);
        let p: [f64;3] = std::array::from_fn(|i| q[i]-0.7*d[i]);
        one_touch(&format!("the cone touched at height {z}, {turn}°"),read(&line(p,d),[0.,1.4],&cone),0.7);
    }
    assert_eq!(read(&line(at(r-k*0.5,0.,-0.5),ruling),[0.,1.],&cone),Meets::Along,"a ruling of the cone");
    let Meets::At(through) = read(&line(at(-3.,0.2,0.3),[1.,0.,0.]),[0.,6.],&cone) else { panic!("a chord: along") };
    assert!(through.len() == 2 && through.iter().all(|x| !x.1),"a chord of the cone: {through:?}");
    let (edge,end) = ending(at(2.4,1.1,0.9),at((r+k*0.4)*0.7f64.cos(),(r+k*0.4)*0.7f64.sin(),0.4));
    one_end("an edge ending on the cone",read(&edge,[0.,end],&cone),end);
    // the torus: tangent at its outer equator along the ring, and over the tube's top both ways
    one_touch("the torus's outer equator",read(&line(at(big+small,-1.,0.),[0.,1.,0.]),[0.,2.],&ring),1.);
    one_touch("the tube's top along the ring",read(&line(at(big,-1.,small),[0.,1.,0.]),[0.,2.],&ring),1.);
    one_touch("the tube's top across it",read(&line(at(big-1.,0.,small),[1.,0.,0.]),[0.,2.],&ring),1.);
    let Meets::At(through) = read(&line(at(-4.,0.1,0.2),[1.,0.,0.]),[0.,8.],&ring) else { panic!("through: along") };
    assert!(through.len() == 4 && through.iter().all(|x| !x.1),"a line through the ring: {through:?}");
    let (edge,end) = ending(at(3.5,0.4,1.2),at((big+small*0.6f64.cos())*0.3f64.cos(),(big+small*0.6f64.cos())*0.3f64.sin(),small*0.6f64.sin()));
    one_end("an edge ending on the torus",read(&edge,[0.,end],&ring),end);
    assert_eq!(m.sampled(),0,"a search sampled\n{}",m.report());
}

/// **A conic edge and a spline edge are read certified too** (#780 R2: the closed class's edges
/// sampled none): a circle or an ellipse against a plane, a cylinder, a ball, a cone and a torus is
/// its surface's algebraic form along it, a trigonometric polynomial bounded by its amplitudes; a
/// B-spline curve against a plane its poles' heights, a scalar B-spline bounded by its differences
/// span by span. Each against what is known apart from the kernel: the circle of radius 2 meets the
/// plane `x = 1` at `±60°` and touches `x = 2`; the cylinder of radius 1 about the x axis at the
/// four `sin t = ±½`; the ball of radius 3 about `(3, 0, 1)` where `cos t = ⅓`; a circle on a cone,
/// on a torus's equator and on its tube's top lies along them; a meridian circle crosses the tube
/// twice; a spline's crossings where bisection of its own heights finds them, none where its
/// weights lift it clear, and an end on the plane its end.
#[test]
fn a_conic_and_a_spline_edge_are_read_certified() {
    use gcs_core::brep::geom::{Curve,Frame,Surface};
    use gcs_core::brep::nurbs::BSpline;
    use gcs_core::brep::query::{curve_surface,Meets};
    use gcs_core::meter::{self as meter,Budget,Meter};
    use std::f64::consts::{PI,TAU};
    let tol = 1e-7;
    let m = Meter::new(Budget::UNLIMITED);
    let read = |c: &Curve,t: [f64;2],s: &Surface| meter::within(&m,|| curve_surface(c,t,s,tol));
    let about = |o: [f64;3],z: [f64;3]| Frame::about(o,z);
    let flat = Curve::Circle(about([0.;3],[0.,0.,1.]),2.);
    // its roots: as many as `want`, crossings or touches as `touch`, and each point where `on` is
    // nought
    let check = |what: &str,found: Meets,c: &Curve,want: usize,touch: bool,on: &dyn Fn([f64;3]) -> f64| {
        let Meets::At(found) = found else { panic!("{what}: along") };
        assert_eq!(found.len(),want,"{what}: {found:?}");
        for &(t,k) in &found {
            assert_eq!(k,touch,"{what}: {found:?}");
            let q = c.point(t);
            assert!(on(q).abs() <= if touch { 1e-3 } else { 1e-9 },"{what}: at {t}, {q:?} off by {}",on(q));
        }
    };
    check("a circle across a plane",read(&flat,[0.,TAU],&Surface::Plane(about([1.,0.,0.],[1.,0.,0.]))),&flat,2,false,&|q| q[0]-1.);
    check("a circle touching a plane",read(&flat,[-PI,PI],&Surface::Plane(about([2.,0.,0.],[1.,0.,0.]))),&flat,1,true,&|q| q[0]-2.);
    check("a circle across a cylinder",read(&flat,[0.,TAU],&Surface::Cylinder(about([0.;3],[1.,0.,0.]),1.)),&flat,4,false,
        &|q| q[1].hypot(q[2])-1.);
    let high = Curve::Circle(about([0.,0.,1.],[0.,0.,1.]),2.);
    check("a circle across a ball",read(&high,[-PI,PI],&Surface::Sphere(about([3.,0.,1.],[0.,0.,1.]),3.)),&high,2,false,
        &|q| (q[0]-3.).hypot(q[1]).hypot(q[2]-1.)-3.);
    assert_eq!(read(&high,[0.,TAU],&Surface::Sphere(about([0.;3],[0.,0.,1.]),5f64.sqrt())),Meets::Along,"a circle on a ball");
    let cone = Surface::Cone(about([0.;3],[0.,0.,1.]),1.,PI/4.);
    assert_eq!(read(&high,[0.,TAU],&cone),Meets::Along,"a circle on a cone");
    let upright = Curve::Circle(about([1.5,0.,0.5],[0.,1.,0.]),0.5);
    check("an upright circle across a cone",read(&upright,[0.,TAU],&cone),&upright,2,false,&|q| q[0].hypot(q[1])-(1.+q[2]));
    let ring = Surface::Torus(about([0.;3],[0.,0.,1.]),2.,0.5);
    assert_eq!(read(&Curve::Circle(about([0.;3],[0.,0.,1.]),2.5),[0.,TAU],&ring),Meets::Along,"the torus's equator");
    assert_eq!(read(&Curve::Circle(about([0.,0.,0.5],[0.,0.,1.]),2.),[0.,TAU],&ring),Meets::Along,"the tube's top");
    let meridian = Curve::Circle(about([2.5,0.,0.],[0.,1.,0.]),0.5);
    check("a meridian circle across the tube",read(&meridian,[0.,TAU],&ring),&meridian,2,false,
        &|q| (q[0].hypot(q[1])-2.).hypot(q[2])-0.5);
    let oval = Curve::Ellipse(about([0.;3],[0.,0.,1.]),3.,1.);
    check("an ellipse across a plane",read(&oval,[-PI,PI],&Surface::Plane(about([1.5,0.,0.],[1.,0.,0.]))),&oval,2,false,&|q| q[0]-1.5);
    // a cubic over two spans, and its heights over `y = 0` bisected apart from the kernel's search
    let ground = Surface::Plane(about([0.;3],[0.,1.,0.]));
    let poles = vec![[0.,0.,0.],[1.,2.,0.],[2.,-3.,0.],[3.,3.,0.],[4.,-1.,0.]];
    let two = BSpline::new(3,vec![0.,0.,0.,0.,1.,2.,2.,2.,2.],poles).unwrap();
    let bisected: Vec<f64> = {
        let y = |t: f64| two.point(t)[1];
        let n = 4000;
        (1..n).filter_map(|i| {
            let (mut a,mut b) = (2.*i as f64/n as f64,2.*(i+1) as f64/n as f64);
            if y(a)*y(b) > 0. { return None }
            for _ in 0..80 { let c = 0.5*(a+b); if (y(c) < 0.) == (y(a) < 0.) { a = c } else { b = c } }
            Some(0.5*(a+b))
        }).collect()
    };
    let spline = Curve::BSpline(std::sync::Arc::new(two));
    let Meets::At(found) = read(&spline,[0.,2.],&ground) else { panic!("a spline along a plane") };
    assert!(bisected.len() >= 3,"{bisected:?}");
    assert_eq!(found.len(),bisected.len()+1,"{found:?} against its start and {bisected:?}");
    assert_eq!(found[0],(0.,false),"its start on the plane");
    for (x,y) in found[1..].iter().zip(&bisected) { assert!(!x.1 && (x.0-y).abs() <= 1e-9,"{found:?} against {bisected:?}") }
    // weighed, `19t² − 30t + 12` over the one span has no root: only the start meets it
    let lifted = BSpline::rational(3,vec![0.,0.,0.,0.,1.,1.,1.,1.],vec![[0.,0.,0.],[1.,2.,0.],[2.,-2.,0.],[3.,1.,0.]],vec![1.,2.,1.,1.]).unwrap();
    assert_eq!(read(&Curve::BSpline(std::sync::Arc::new(lifted)),[0.,1.],&ground),Meets::At(vec![(0.,false)]),"a rational spline lifted clear");
    // unweighed, `13t² − 18t + 6`: its start and two crossings
    let plain = BSpline::new(3,vec![0.,0.,0.,0.,1.,1.,1.,1.],vec![[0.,0.,0.],[1.,2.,0.],[2.,-2.,0.],[3.,1.,0.]]).unwrap();
    let Meets::At(found) = read(&Curve::BSpline(std::sync::Arc::new(plain)),[0.,1.],&ground) else { panic!("along") };
    let roots = [0.,(18.-12f64.sqrt())/26.,(18.+12f64.sqrt())/26.];
    assert!(found.len() == 3 && found.iter().zip(roots).all(|(x,y)| !x.1 && (x.0-y).abs() <= 1e-9),"{found:?} against {roots:?}");
    assert_eq!(m.sampled(),0,"a search sampled\n{}",m.report());
}

/// **Every surface the closed class emits locates a point exactly** (#777 V1's gate): a ray's
/// crossings with each kind of candidate face are certified, never sampled — planes, a cylinder, a
/// cone, a ball and a torus, a hyperboloid a leaning edge turns into (a surface of revolution of a
/// conic, `closed::revolved`), an oblique cylinder (a cylinder's affine image), and the sheets a
/// screw sweeps: a level line's (a screwed triangle, a thin ramp), a level circle's (a screwed
/// disc, thinner) and a leaning circle's (a ball's pipe). Each solid's point location
/// (`Located::solid_place`) is held to its membership computed directly from its definition, apart
/// from the kernel, at seeded points crowded about its boundary and its thin sheets: no point wrong,
/// none `On`, and no search sampled (the profile's count, `util::meter`).
#[test]
fn every_surface_the_closed_class_emits_locates_points_exactly() {
    use gcs_core::brep::build::{cone,disc,prism,screwed,sphere,torus,Profile,ProfileEdge};
    use gcs_core::brep::query::{Located,Place};
    use gcs_core::meter::{self as meter,Budget,Meter};
    use std::f64::consts::TAU;
    let hyperboloid = gcs_core::brep::swept::closed::revolved([0.;3],[0.,0.,1.],[2.,0.,0.],unit([0.,1.,1.]),[-1.,1.5],1e-9).unwrap().remove(0);
    // the cylinder of radius 1 about the z axis, 2 tall, sheared so its top stands 1.5 along x
    let shear = gcs_core::brep::affine::Affine::new([[1.,0.,0.75],[0.,1.,0.],[0.,0.,1.]],[0.;3]).unwrap();
    let oblique = prism(&disc([0.;3],[0.,0.,1.],1.),0.,2.).unwrap().mapped(&shear);
    // a level triangle 10 out, screwed 280° rising half a unit a turn: a ramp a few hundredths thick
    let (theta,lead) = (280f64.to_radians(),0.5);
    let rise = lead/TAU*theta;
    let tri = [[9.,-0.4,0.],[11.,-0.4,0.],[10.,0.5,0.]];
    let triangle = Profile {origin:tri[0],normal:[0.,0.,1.],names:Vec::new(),loops:vec![(0..3).map(|k| ProfileEdge::Line {a:tri[k],b:tri[(k+1)%3]}).collect()]};
    let ramp_line = screwed(&triangle,[0.;3],[0.,0.,1.],theta,rise).unwrap();
    let ramp_circle = screwed(&disc([10.,0.,0.],[0.,0.,1.],1.),[0.;3],[0.,0.,1.],theta,rise).unwrap();
    // a ball's great circle square to its centre's velocity, 10 out, screwed 1.5 rad climbing half a
    // unit a radian: the pipe about the centre's helix
    let (turn,climb) = (1.5,0.75);
    let n = unit([0.,10.,0.5]);
    let pipe = screwed(&disc([10.,0.,0.],n,1.),[0.;3],[0.,0.,1.],turn,climb).unwrap();
    // membership from each definition: `Some(inside)`, or none within a hair of its boundary
    let hair = 1e-6;
    let sure = |margin: f64,inside: bool| (margin.abs() > hair).then_some(inside);
    let rho = |p: [f64;3]| p[0].hypot(p[1]);
    let in_triangle = |q: [f64;3]| -> f64 {
        // the least of the signed distances in from each side
        (0..3).map(|k| { let (a,b) = (tri[k],tri[(k+1)%3]); let (ex,ey) = (b[0]-a[0],b[1]-a[1]); let l = ex.hypot(ey);
            (ex*(q[1]-a[1])-ey*(q[0]-a[0]))/l }).fold(f64::INFINITY,f64::min)
    };
    // a level section at height 0 screwed: the point screwed back by the turn its height gives,
    // inside the section; past either end's level, outside
    let screwed_level = |p: [f64;3],inside: &dyn Fn([f64;3]) -> f64| -> Option<bool> {
        let v = p[2]/rise;
        if v < 0. { return sure(p[2],false) }
        if v > 1. { return sure(p[2]-rise,false) }
        let m = inside(screwed_back(p,theta,rise,v));
        if m > 0. { sure(m.min(p[2]).min(rise-p[2]),true) } else { sure(m,false) }
    };
    let in_pipe = |p: [f64;3]| -> Option<bool> {
        // the turns `v` that bring `p` into the disc's plane, sampled finely and each settled
        let phi = |v: f64| { let q = screwed_back(p,turn,climb,v); n[0]*(q[0]-10.)+n[1]*q[1]+n[2]*q[2] };
        let mut found: Option<f64> = None;
        let steps = 4000;
        for k in 0..steps {
            let (a,b) = (k as f64/steps as f64,(k+1) as f64/steps as f64);
            let (fa,fb) = (phi(a),phi(b));
            if fa*fb > 0. { continue }
            let (mut l,mut r) = (a,b);
            for _ in 0..80 { let m = 0.5*(l+r); if (phi(m) < 0.) == (fa < 0.) { l = m } else { r = m } }
            let q = screwed_back(p,turn,climb,0.5*(l+r));
            let d = (q[0]-10.).hypot(q[1]).hypot(q[2])-1.;
            found = Some(found.map_or(d,|f: f64| f.min(d)));
        }
        // beyond either end the half-spaces past the end discs hold nothing
        let ends = phi(0.).abs().min(phi(1.).abs());
        match found { Some(d) => sure(d.abs().min(if d < 0. { ends } else { f64::INFINITY }),d < 0.),None => sure(ends,false) }
    };
    let cases: Vec<(&str,Brep,Box<dyn Fn([f64;3]) -> Option<bool>>)> = vec![
        ("a block",crate::brep::block([0.;3],[2.,1.,1.5]),Box::new(|p| {
            let m = (0..3).map(|k| p[k].min([2.,1.,1.5][k]-p[k])).fold(f64::INFINITY,f64::min); sure(m,m > 0.) })),
        ("a drum",prism(&disc([0.;3],[0.,0.,1.],1.),0.,2.).unwrap(),Box::new(|p| { let m = (1.-rho(p)).min(p[2]).min(2.-p[2]); sure(m,m > 0.) })),
        ("a frustum",cone([0.;3],[0.,0.,1.],1.,0.4,1.5).unwrap(),Box::new(|p| {
            let m = (1.-0.4*p[2]-rho(p)).min(p[2]).min(1.5-p[2]); sure(m,m > 0.) })),
        ("a ball",sphere([0.5,0.,0.],[0.,0.,1.],1.).unwrap(),Box::new(|p| { let m = 1.-(p[0]-0.5).hypot(p[1]).hypot(p[2]); sure(m,m > 0.) })),
        ("a ring",torus([0.;3],[0.,0.,1.],2.,0.3).unwrap(),Box::new(|p| { let m = 0.3-(rho(p)-2.).hypot(p[2]); sure(m,m > 0.) })),
        ("a hyperboloid",hyperboloid,Box::new(|p| { let m = (4.+p[2]*p[2]).sqrt()-rho(p); let m = m.min(p[2]+1.).min(1.5-p[2]); sure(m,m > 0.) })),
        ("an oblique cylinder",oblique,Box::new(|p| { let q = [p[0]-0.75*p[2],p[1],p[2]]; let m = (1.-rho(q)).min(p[2]).min(2.-p[2]); sure(m,m > 0.) })),
        ("a screwed line's ramp",ramp_line,Box::new(move |p| screwed_level(p,&in_triangle))),
        ("a screwed rim's ramp",ramp_circle,Box::new(move |p| screwed_level(p,&|q| 1.-(q[0]-10.).hypot(q[1])))),
        ("a ball's pipe",pipe,Box::new(in_pipe)),
    ];
    for (what,solid,member) in cases {
        let at = Located::new(&solid,1e-6);
        let (lo,hi) = solid.bounds();
        let mut rng = gcs_core::rng::Rng::new(11);
        let m = Meter::new(Budget::UNLIMITED);
        let (mut held,mut checked) = (0,0);
        // points over the box, and points crowded within a few hundredths of the boundary: a point of
        // the box nudged toward a vertex-free spot of a face along its normal
        let faces: Vec<(usize,[f64;3])> = solid.faces.iter().enumerate().flat_map(|(fi,f)| {
            let [[u0,u1],[v0,v1]] = gcs_core::brep::boolean::face_parameters(&solid,fi);
            (1..8).flat_map(move |i| (1..8).map(move |j| (fi,f.surface.point([u0+(u1-u0)*i as f64/8.,v0+(v1-v0)*j as f64/8.]))))
        }).filter(|&(fi,p)| at.face_place(fi,p) == Place::In).collect();
        for k in 0..600 {
            let p: [f64;3] = if k%2 == 0 || faces.is_empty() { std::array::from_fn(|i| rng.uniform(lo[i]-0.1,hi[i]+0.1)) } else {
                let (fi,q) = faces[(rng.uniform(0.,faces.len() as f64) as usize).min(faces.len()-1)];
                let nn = solid.faces[fi].outward(q).unwrap();
                // within a few hundredths of it, or of a thin ramp's few thousandths
                let s = rng.uniform(-1.,1.)*if k%4 == 1 { 0.03 } else { 0.003 };
                std::array::from_fn(|i| q[i]+s*nn[i])
            };
            let Some(inside) = member(p) else { continue };
            checked += 1;
            let place = meter::within(&m,|| at.solid_place(p));
            assert!(place != Place::On && (place == Place::In) == inside,"{what}: at {p:?} {place:?}, its definition {inside}");
            if inside { held += 1 }
        }
        assert!(checked > 400 && held > 40,"{what}: {checked} points read, {held} inside");
        assert_eq!(m.sampled(),0,"{what}: a search sampled\n{}",m.report());
    }
}

/// **A level rim's sheet in closed form and by the general search agree** (#777 M1, M3: a closed form
/// kept as a fast path is held to the general method): the lines through a screwed rim's thin ramp
/// meet its sheet at the same points by `line_screwed_circle`'s bound on one unknown as by the
/// three-unknown search (`query::curve_surface_searched`), each to the tolerance — and the closed
/// form is the faster, measured here and said.
#[test]
fn a_rims_sheet_in_closed_form_is_held_to_the_general_search() {
    use gcs_core::brep::geom::Curve;
    use gcs_core::brep::query::{curve_surface,curve_surface_searched,Meets};
    let ramp = ramp(0.5,280f64.to_radians(),0.);
    let sheet = ramp.faces.iter().find(|f| f.surface.kind() == "blend").unwrap().surface.clone();
    let mut rng = gcs_core::rng::Rng::new(5);
    let (mut fast,mut general,mut met) = (std::time::Duration::ZERO,std::time::Duration::ZERO,0);
    for _ in 0..60 {
        let a = rng.uniform(0.,4.5);
        let p = [10.*a.cos()+rng.uniform(-1.5,1.5),10.*a.sin()+rng.uniform(-1.5,1.5),rng.uniform(-0.6,0.2)];
        let d = unit([rng.uniform(-1.,1.),rng.uniform(-1.,1.),rng.uniform(-0.2,0.2)]);
        let line = Curve::Line {p,d};
        let clock = std::time::Instant::now();
        let Meets::At(one) = curve_surface(&line,[-3.,3.],&sheet,1e-7) else { panic!("along") };
        fast += clock.elapsed();
        let clock = std::time::Instant::now();
        let Some(Meets::At(two)) = curve_surface_searched(&line,[-3.,3.],&sheet,1e-7) else { panic!("the search gave up") };
        general += clock.elapsed();
        assert_eq!(one.len(),two.len(),"from {p:?} along {d:?}: {one:?} against {two:?}");
        for (x,y) in one.iter().zip(&two) { assert!((x.0-y.0).abs() <= 1e-6,"from {p:?} along {d:?}: {one:?} against {two:?}") }
        met += one.len();
    }
    assert!(met > 20,"{met} crossings");
    eprintln!("a screwed rim's sheet against 60 lines: in closed form {fast:?}, by the general search {general:?}");
}

/// **A build past its budget refuses by name, within it** (#777 V3's gate): the ring turned about
/// an axis along its own, its membership reads held to a fortieth of what it takes, is refused in a
/// fraction of its time, naming the stage that ran away and a point — the same way at every run,
/// the budget counted in steps. Within the default budget it builds.
#[test]
fn a_build_past_its_budget_refuses_by_name_within_it() {
    use gcs_core::brep::swept::closed::{closed_within,Unbuilt,BUDGET};
    use gcs_core::meter::{Budget,Stage};
    let tool = gcs_core::brep::build::torus([10.,0.,0.],[0.,0.,1.],1.,0.3).unwrap();
    let path = Trajectory::Twist(Screw {origin:[0.;3],axis:[0.,0.,1.],ratio:1.5,advance:0.});
    let tight = Budget::UNLIMITED.with(Stage::Membership,20_000);
    let refused = |_: ()| {
        let clock = std::time::Instant::now();
        let r = closed_within(&tool,&path,[0.,1.],0.002,1e-6,tight);
        (r,clock.elapsed())
    };
    let (first,took) = refused(());
    let Err(Unbuilt::Budget {stage,at,what}) = first else { panic!("{first:?}") };
    assert_eq!(stage,Stage::Membership);
    assert!(at.iter().all(|x| x.is_finite()) && !what.is_empty(),"{at:?} {what}");
    assert!(took.as_secs_f64() < 2.,"refused after {took:?}");
    let (again,_) = refused(());
    assert_eq!(again.err(),Some(Unbuilt::Budget {stage,at,what}),"refused the same way again");
    assert!(BUDGET.0[Stage::Membership as usize] > 20_000);
}

/// **A build past its budget stops by one typed check** (#780 R3, `util::meter::go_on`): with no
/// meter a build always goes on; past its budget `go_on` answers `Stopped`, naming the stage that
/// ran away, which a closed build turns into its refusal by budget (`Unbuilt::Budget`) and a
/// Boolean's phases into their message. And a certified reading stopped there reads nothing more: a
/// line through a torus (a quartic, its roots isolated) within a budget of one step of the ray stage
/// is read once, its isolation stopped at its first stretch, and nothing sampled in its place.
#[test]
fn a_build_past_its_budget_stops_by_one_typed_check() {
    use gcs_core::brep::geom::{Curve,Frame,Surface};
    use gcs_core::brep::query::{curve_surface,Meets};
    use gcs_core::brep::swept::closed::Unbuilt;
    use gcs_core::meter::{self as meter,Budget,Meter,Stage,Stopped};
    assert_eq!(meter::go_on(),Ok(()));
    let ring = Surface::Torus(Frame::about([0.;3],[0.,0.,1.]),2.,0.5);
    let line = Curve::Line {p:[-4.,0.,0.1],d:[1.,0.,0.]};
    let m = Meter::new(Budget::UNLIMITED);
    let (found,on) = meter::within(&m,|| (curve_surface(&line,[0.,8.],&ring,1e-7),meter::go_on()));
    let Meets::At(found) = found else { panic!("along") };
    assert_eq!((found.len(),on,m.sampled()),(4,Ok(()),0),"{found:?}");
    let m = Meter::new(Budget::UNLIMITED.with(Stage::Ray,1));
    let (found,on) = meter::within(&m,|| (curve_surface(&line,[0.,8.],&ring,1e-7),meter::go_on()));
    assert_eq!(found,Meets::At(Vec::new()));
    assert_eq!((m.count(Stage::Ray),m.sampled()),(2,0),"read once, one stretch isolated, nothing sampled:\n{}",m.report());
    let Err(Stopped(over)) = on else { panic!("{on:?}") };
    assert_eq!(over.stage,Stage::Ray);
    let said = String::from(Stopped(over.clone()));
    assert!(said.starts_with("the build went past its budget in its ray stage"),"{said}");
    assert!(matches!(Unbuilt::from(Stopped(over)),Unbuilt::Budget {stage:Stage::Ray,..}));
}

/// A box about `b`'s, a tenth of its size wider each way: a domain to judge a closed build in.
fn box_about(b: &Brep) -> Brep {
    let (lo,hi) = b.bounds();
    let pad = 0.1*(0..3).map(|k| hi[k]-lo[k]).fold(0.,f64::max);
    crate::brep::block(lo.map(|x| x-pad),hi.map(|x| x+pad))
}

/// `b` turned inside out: every face the other side, its loops run the other way.
fn inside_out(b: &Brep) -> Brep {
    let mut out = b.clone();
    for f in &mut out.faces {
        f.reversed = !f.reversed;
        f.loops = f.loops.drain(..).map(gcs_core::brep::topo::reverse_loop).collect();
    }
    out
}

/// **A solid built wrong is refused at a witness, never returned** (#777 V2's gate): the three ways
/// a closed build was returned wrong on #776, recreated on a fast case each and judged by the
/// sweep's definition (`closed::judge`) — a ball's pipe turned inside out (as a drum screwed half a
/// millimetre a turn came out at −427), the pipe missing a piece (a drum cut from a ring sector
/// came out at 54.69 of 62.57), and a groove cut with more taken than the bar sweeps (a ball-end
/// mill screwed through the sector removed 14.4 of 12.0). Each is refused at a point the solid and
/// the definition disagree on; what the build itself returned is judged right.
#[test]
fn a_solid_built_wrong_is_refused_at_a_witness() {
    use gcs_core::brep::swept::closed::{cut_from,judge,Unbuilt};
    use std::f64::consts::TAU;
    let tool = gcs_core::brep::build::sphere([10.,0.,0.],[0.,0.,1.],1.).unwrap();
    let path = Trajectory::Twist(Screw {origin:[0.;3],axis:[0.,0.,1.],ratio:1.5,advance:TAU*0.5*1.5});
    let built = closed(&tool,&path,[0.,1.],0.002,1e-6).unwrap();
    let solid = Solid::new(&tool,0.002,1.).unwrap();
    let swept = Swept {tool:&solid,path:&path,roll:[0.,1.]};
    let domain = box_about(&built);
    assert_eq!(judge(&built,&swept,&domain,None,&[],1e-6),Ok(()));
    let refused = |b: &Brep,blank: Option<&Brep>,swept: &Swept,domain: &Brep,what: &str| match judge(b,swept,domain,blank,&[],1e-6) {
        Err(Unbuilt::Wrong {at,..}) => assert!(at.iter().all(|x| x.is_finite()),"{what}: at {at:?}"),
        other => panic!("{what}: {other:?}"),
    };
    refused(&inside_out(&built),None,&swept,&domain,"turned inside out");
    // a piece of the pipe missing: a block taken out across its middle, where its centre's helix
    // stands at (7.3, 6.8, 0.375)
    let missing = combined(&built,&block([6.,5.5,-1.],[8.5,8.2,2.]),Op::Cut,1e-6).unwrap();
    assert!(volume(&missing) < volume(&built)-1.);
    refused(&missing,None,&swept,&domain,"missing a piece");
    // a groove cut with more taken than the bar sweeps
    let blank = block([0.,0.,0.],[6.,2.,2.]);
    let bar = block([1.,0.5,1.5],[2.,1.5,3.]);
    let slide = slide([3.,0.,0.]);
    let cut = cut_from(&blank,&bar,&slide,[0.,1.],0.002,1e-6).unwrap();
    let bar_solid = Solid::new(&bar,0.002,1.).unwrap();
    let bar_swept = Swept {tool:&bar_solid,path:&slide,roll:[0.,1.]};
    assert_eq!(judge(&cut,&bar_swept,&blank,Some(&blank),&[],1e-6),Ok(()));
    let over = combined(&cut,&block([1.5,0.2,0.8],[4.5,1.8,1.6]),Op::Cut,1e-6).unwrap();
    assert!(volume(&over) < volume(&cut)-0.5);
    refused(&over,Some(&blank),&bar_swept,&blank,"cut too deep");
}

/// **A failed build's arrangement is written, read back whole, replayed alone and reduced** (#777
/// V5): the candidates a closed build splits its domain by — a screwed rim's ramp, a hyperboloid, a
/// torus, an oblique cylinder, a block — each written and read back bit for bit (`swept::dump`);
/// their arrangement replayed from the files has the cells the arrangement built directly has; and
/// a failure that needs two of them is reduced to those two.
#[test]
fn a_failed_builds_arrangement_is_dumped_replayed_and_reduced() {
    use gcs_core::brep::boolean::arrangement_within;
    use gcs_core::brep::swept::dump::{dump,load,read,reduce,write};
    use gcs_core::brep::build::{disc,prism};
    let shear = gcs_core::brep::affine::Affine::new([[1.,0.,0.5],[0.,1.,0.],[0.,0.,1.]],[0.;3]).unwrap();
    let candidates: Vec<Brep> = vec![
        ramp(2.,1.5,0.),
        gcs_core::brep::swept::closed::revolved([0.;3],[0.,0.,1.],[9.,0.,0.],unit([0.,1.,1.]),[-1.,1.],1e-9).unwrap().remove(0),
        gcs_core::brep::build::torus([10.,0.,0.],[0.,0.,1.],1.,0.3).unwrap(),
        prism(&disc([10.,1.,-1.],[0.,0.,1.],0.5),0.,2.).unwrap().mapped(&shear),
        block([9.,-2.,-0.2],[12.,2.,0.2]),
    ];
    for c in &candidates {
        let text = write(c).unwrap();
        let back = read(&text).unwrap();
        assert_eq!(write(&back).unwrap(),text);
        assert_eq!(volume(&back),volume(c));
    }
    let domain = block([7.,-3.,-1.5],[13.,3.,1.5]);
    let refs: Vec<&Brep> = candidates.iter().collect();
    let dir = std::env::temp_dir().join(format!("sweep-dump-{}",std::process::id()));
    dump(&dir,&domain,&refs,1e-6,"a test").unwrap();
    let loaded = load(&dir).unwrap();
    let _ = std::fs::remove_dir_all(&dir);
    let direct = arrangement_within(&domain,&refs[..3],1e-6).unwrap();
    let replayed = loaded.replay(&[0,1,2]).unwrap();
    assert_eq!(replayed.cells().unwrap().len(),direct.cells().unwrap().len());
    // a failure that needs the ramp and the torus both: reduced to those two
    let fails = |r: &Result<gcs_core::brep::complex::Complex,String>| r.as_ref().is_ok_and(|c| {
        let names: std::collections::BTreeSet<usize> = c.origins.iter().flatten().map(|o| o.operand).collect();
        let kinds: std::collections::BTreeSet<&str> = c.b.faces.iter().map(|f| f.surface.kind()).collect();
        names.len() > 1 && kinds.contains("blend") && kinds.contains("torus")
    });
    assert_eq!(reduce(&loaded,&fails),vec![0,2]);
}

/// The ball-end mill of #777 V3's gate: a mill of radius 1, its ball's centre 10 from a screw's
/// axis and its side 3 tall above it, screwed 1.5 rad climbing half a unit a radian through a
/// sixth of a ring 8 to 12 across and 4 high, which it passes wholly across. The sector, the mill,
/// the screw and the roll.
fn ball_end_mill_through_a_sector() -> (Brep,Brep,Trajectory,[f64;2]) {
    use gcs_core::brep::build::{revolve,Profile,ProfileEdge};
    use gcs_core::brep::geom::Frame;
    use std::f64::consts::{FRAC_PI_2,PI};
    let (d,r,h,turn,k) = (10.,1.,3.,1.5,0.5);
    let at = |x: f64,z: f64| [d+x,0.,z];
    let edges = vec![ProfileEdge::Arc {frame:Frame::new(at(0.,0.),[0.,-1.,0.],[1.,0.,0.]),r,span:Some([-FRAC_PI_2,0.])},
        ProfileEdge::Line {a:at(r,0.),b:at(r,h)},ProfileEdge::Line {a:at(r,h),b:at(0.,h)},ProfileEdge::Line {a:at(0.,h),b:at(0.,-r)}];
    let mill = revolve(&Profile {origin:at(0.,0.),normal:[0.,1.,0.],names:Vec::new(),loops:vec![edges]},[d,0.,0.],[0.,0.,1.],
        2.*PI).unwrap();
    let f = Frame::about([0.;3],[0.,0.,1.]);
    let rect = [f.at([8.,0.,-2.]),f.at([12.,0.,-2.]),f.at([12.,0.,2.]),f.at([8.,0.,2.])];
    let profile = Profile {origin:rect[0],normal:f.y,names:Vec::new(),loops:vec![(0..4).map(|j| ProfileEdge::Line {a:rect[j],b:rect[(j+1)%4]}).collect()]};
    let sector = revolve(&profile,[0.;3],[0.,0.,1.],PI/3.).unwrap().moved(&gcs_core::brep::geom::Rigid::turn([0.;3],[0.,0.,1.],-PI/6.));
    let path = Trajectory::Twist(Screw {origin:[0.;3],axis:[0.,0.,1.],ratio:1.,advance:k*std::f64::consts::TAU});
    (sector,mill,path,[-turn/2.,turn/2.])
}

/// **The ball-end mill screwed through a ring sector is refused by name when its sheet's inverse
/// map runs past its budget** (#777 V3's gate, the case M3 and M4 were for; #780 R1): on #776 this
/// run stalled for hours, and a build of it removed 14.4 against 12.0. Its inverse stage given a
/// thousandth of `closed::BUDGET`'s and every other stage its whole, the build runs its
/// arrangement, traces, rays and outlines as under the default budget and is refused in the stage
/// that ran away there — the inverse map, at a point, naming the surface — the same way again, in
/// seconds, where the default budget's refusal takes minutes
/// (`the_ball_end_mill_screwed_through_a_ring_sector_is_cut_right_or_refused_by_name`, slow tier).
/// A stopped build stops its crossings' search too (`meeting::crossings`): before, refused at a
/// hundredth of the budget, the build still walked every pair of near chords to its crossing, 23
/// minutes.
#[test]
fn the_ball_end_mill_screwed_through_a_ring_sector_is_refused_by_its_inverse_maps_budget() {
    use gcs_core::brep::swept::closed::{cut_from_within,Unbuilt,BUDGET};
    use gcs_core::meter::Stage;
    let (sector,mill,path,roll) = ball_end_mill_through_a_sector();
    let tight = BUDGET.with(Stage::Inverse,BUDGET.0[Stage::Inverse as usize]/1000);
    let refused = || {
        let clock = std::time::Instant::now();
        (cut_from_within(&sector,&mill,&path,roll,0.002,1e-6,tight),clock.elapsed())
    };
    let (first,took) = refused();
    eprintln!("refused in {took:?}: {first:?}");
    let Err(Unbuilt::Budget {stage,at,what}) = first else { panic!("{first:?}") };
    assert_eq!(stage,Stage::Inverse);
    assert!(at.iter().all(|x| x.is_finite()) && !what.is_empty(),"{at:?} {what}");
    let (again,_) = refused();
    assert_eq!(again.err(),Some(Unbuilt::Budget {stage,at,what}),"refused the same way again");
}

/// **The ball-end mill screwed through a ring sector is cut right, or refused by name within its
/// budget** (#777 V3's gate, under the default budget): held to the volume it removes apart from
/// the kernel, or refused naming the stage that ran away — never hung, never wrong. In helical
/// coordinates `(ρ, ψ, ζ = z − kψ)` the screw is a shift in `ψ`, so a point of the sector is
/// removed where the helix through it meets the mill: the section of such helices, `(ρ, ζ)` on a
/// fine grid (the side's where `|ρ e^{iψ} − d| ≤ 1` for some `ψ` whose height is on it, the ball's
/// within 1 of the centre's helix), each weighed by `ρ` and by the angle of the sector over which
/// its helix stays within the sector's heights. Refused, it runs the inverse stage's whole budget
/// first: about three minutes.
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, about three minutes: the ball-end mill's budget run down")]
fn the_ball_end_mill_screwed_through_a_ring_sector_is_cut_right_or_refused_by_name() {
    use gcs_core::brep::swept::closed::{cut_from,Unbuilt};
    use std::f64::consts::PI;
    let (d,r,h,k) = (10.,1.,3.,0.5);
    let (sector,mill,path,roll) = ball_end_mill_through_a_sector();
    // apart from the kernel: whether the helix (ρ, ζ) meets the mill, and over what angle of the
    // sector it stays within the sector's heights
    let meets = |rho: f64,zeta: f64| -> bool {
        let c = (rho*rho+d*d-r*r)/(2.*rho*d);
        if c <= 1. {
            let a = c.max(-1.).acos();
            if zeta+k*a >= 0. && zeta-k*a <= h { return true }
        }
        let dist = |psi: f64| { let (sn,cs) = psi.sin_cos(); (rho*cs-d).hypot(rho*sn).hypot(zeta+k*psi) };
        let (mut lo,mut hi) = (-0.3,0.3);
        for _ in 0..80 { let (m1,m2) = (lo+(hi-lo)*0.382,lo+(hi-lo)*0.618); if dist(m1) < dist(m2) { hi = m2 } else { lo = m1 } }
        dist(0.5*(lo+hi)) <= r
    };
    let within = |zeta: f64| -> f64 {
        // ψ in [−π/6, π/6] with −2 ≤ ζ + kψ ≤ 2
        let (lo,hi) = ((-2.-zeta)/k,(2.-zeta)/k);
        (hi.min(PI/6.)-lo.max(-PI/6.)).max(0.)
    };
    let (n,m) = (1000,2400);
    let (z0,z1) = (-2.-k*PI/6.,2.+k*PI/6.);
    let mut removed = 0.;
    for i in 0..n { for j in 0..m {
        let (rho,zeta) = (d-1.1+2.2*(i as f64+0.5)/n as f64,z0+(z1-z0)*(j as f64+0.5)/m as f64);
        if meets(rho,zeta) { removed += rho*within(zeta) }
    } }
    let removed = removed*(2.2/n as f64)*((z1-z0)/m as f64);
    let blank = volume(&sector);
    eprintln!("the ball-end mill through a sector: removes {removed} apart from the build");
    let clock = std::time::Instant::now();
    match cut_from(&sector,&mill,&path,roll,0.002,1e-6) {
        Ok(cut) => {
            eprintln!("cut in {:?}: removes {}",clock.elapsed(),blank-volume(&cut));
            assert!((blank-volume(&cut)-removed).abs() <= 1e-3*removed,"removes {} against {removed}",blank-volume(&cut));
            agrees_cut_over(&cut,&sector,&mill,&path,roll);
        }
        Err(e) => {
            eprintln!("refused in {:?}: {e:?}",clock.elapsed());
            assert!(matches!(e,Unbuilt::Budget {..}),"refused other than by its budget: {e:?}");
        }
    }
}

/// **The ball-end mill screwed through a ring sector, reduced to its Boolean, holds every cell**
/// (#777 M4's gate): a ball of radius 1 on a mill's axis 10 from a screw's, screwed 1.5 rad climbing
/// half a unit a radian, sweeps the pipe about its centre's helix; the mill's side screws into the
/// cylinder of radius 11 about the axis, which the pipe touches from inside along the helix of the
/// ball's outer equator, so the cells beside that touch are cusp-thin; and the blank is a sixth of a
/// ring 8 to 12 across. Their arrangement (`boolean::selected`) is read cell by cell, each by its
/// own exact point location: the pipe within the sector, the cylinder's part within it the pipe
/// leaves, and the sector's part neither holds. Apart from the kernel, in helical coordinates
/// `(ρ, ψ, ζ = z − kψ)` the screw is a shift in `ψ`, so the pipe within the sector (which it passes
/// wholly across) is the sector's angle times its meridian section's polar integral `∫∫ ρ dρ dz`,
/// summed here on a fine grid of the section, each point in the pipe where its distance from the
/// helix is under the radius.
#[test]
fn the_ball_end_mill_screwed_through_a_sector_reduced_to_its_boolean_holds_every_cell() {
    use gcs_core::brep::boolean::selected;
    use gcs_core::brep::build::{disc,prism,revolve,screwed,Profile,ProfileEdge};
    use gcs_core::brep::geom::Frame;
    use std::f64::consts::PI;
    let (d,r,turn,k) = (10.,1.,1.5,0.5);
    // the pipe: the ball's great circle square to its centre's velocity, screwed from 0.75 rad
    // before the sector to 0.75 after, the sector standing from −π/6 to π/6
    let start: f64 = -0.75;
    let (s,c) = start.sin_cos();
    let centre = [d*c,d*s,k*start];
    let velocity = unit([-d*s,d*c,k]);
    let pipe = screwed(&disc(centre,velocity,r),[0.;3],[0.,0.,1.],turn,k*turn).unwrap();
    let side = prism(&disc([0.,0.,-3.],[0.,0.,1.],d+r),0.,6.).unwrap();
    let f = Frame::about([0.;3],[0.,0.,1.]);
    let at = |x: f64,z: f64| f.at([x,0.,z]);
    let rect = [at(8.,-2.),at(12.,-2.),at(12.,2.),at(8.,2.)];
    let profile = Profile {origin:rect[0],normal:f.y,names:Vec::new(),loops:vec![(0..4).map(|j| ProfileEdge::Line {a:rect[j],b:rect[(j+1)%4]}).collect()]};
    let sector = revolve(&profile,[0.;3],[0.,0.,1.],PI/3.).unwrap().moved(&gcs_core::brep::geom::Rigid::turn([0.;3],[0.,0.,1.],-PI/6.));
    // the meridian section's polar integral: points (ρ, z) of the half-plane ψ = 0 within `r` of the
    // helix (ψ, k ψ) at radius d, its nearest point found over ψ near 0
    let near = |rho: f64,z: f64| -> f64 {
        let dist = |psi: f64| { let (sn,cs) = psi.sin_cos(); (rho-d*cs).hypot(d*sn).hypot(z-k*psi) };
        let (mut a,mut b) = (-0.3,0.3);
        for _ in 0..80 { let (m1,m2) = (a+(b-a)*0.382,a+(b-a)*0.618); if dist(m1) < dist(m2) { b = m2 } else { a = m1 } }
        dist(0.5*(a+b))
    };
    let n = 1200;
    let mut moment = 0.;
    for i in 0..n { for j in 0..n {
        let (rho,z) = (d-1.2+2.4*(i as f64+0.5)/n as f64,-1.2+2.4*(j as f64+0.5)/n as f64);
        if near(rho,z) < r { moment += rho }
    } }
    let moment = moment*(2.4/n as f64)*(2.4/n as f64);
    let in_sector = PI/3.*moment;
    let sector_volume = PI/3.*(12.*12.-8.*8.)/2.*4.;
    let side_in_sector = PI/3.*((d+r)*(d+r)-64.)/2.*4.;
    let part = |keep: &dyn Fn(&[bool]) -> bool| volume(&selected(&[&pipe,&side,&sector],keep,1e-6).unwrap().brep);
    let pipe_part = part(&|m| m[0] && m[2]);
    let side_part = part(&|m| !m[0] && m[1] && m[2]);
    let rest = part(&|m| !m[0] && !m[1] && m[2]);
    // the quadrature's own error, a grid of 1200 on the section, about 1e-5 of it
    assert!((pipe_part-in_sector).abs() <= 1e-4*in_sector,"the pipe within the sector {pipe_part} against {in_sector}");
    assert!((side_part-(side_in_sector-pipe_part)).abs() <= 1e-6*side_in_sector,"the side's part {side_part} against {}",side_in_sector-pipe_part);
    assert!((rest-(sector_volume-side_in_sector)).abs() <= 1e-6*sector_volume,"the rest {rest} against {}",sector_volume-side_in_sector);
}
