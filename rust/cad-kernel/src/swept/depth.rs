//! **How deep another instant covers a point of a candidate sheet** (#942, the depth-based trim):
//! the swept solid is the union of the tool's poses over the roll, so a point of a candidate — a
//! tube's sheet, the tool at an end pose — is on its boundary exactly where no pose holds it. On a
//! sheet the pose of the point's own instant touches it (the depth there nought by construction),
//! and the instants beside it hold it or not by the sign of `s''` (the fold, which the ring road
//! reads in closed form); so what is read here is the least depth over the instants *outside* the
//! point's own contact — the stretch of the roll about its own instant over which its distance from
//! the tool climbs away from nought on each side — and the instant and the place of the tool where
//! that least depth is taken. Where that least is below nought another instant covers the point;
//! the curve where it passes nought is a trim curve, and the covering instant names the partner
//! face and its point there, so both faces' parameters of the curve come by construction.
//!
//! **Beside it, the sampled reading** (`Sampled`, #942's touched entry): for a path that encloses
//! nothing, read only by evaluating it, the least over the roll from equal samples refined — no
//! proof, a basin narrower than a sample's step can be missed; used by `swept::touched` alone.
//!
//! (The peer kernel cadrs, MIT OR Apache-2.0, `crates/cadrs_kernel/src/native/sweep_tube.rs`,
//! trims its sheets by the same reading; nothing of its code is ported here.)

use super::closed::Unbuilt;
use super::rings::{Label,Ring};
use crate::trajectory::{Trajectory,Twist};
use crate::util::fmath::HALF_READ;

use crate::trajectory::Pose;
use crate::util::space::{distance,norm};
type V = [f64;3];

/// **What covers a point beside its own contact**: the least depth of the tool at an instant
/// outside the point's own (`signed`, below nought inside), that instant, the point in the tool's
/// coordinates there and the piece of the meridian (the tool's face) nearest it.
#[derive(Clone,Copy,Debug)]
pub struct Cover { pub depth: f64,pub at: f64,pub point: V,pub piece: usize }

/// **How a point of a sheet stands under the sweep's other instants.**
#[derive(Clone,Copy,Debug)]
pub enum Reading {
    /// The instants beside its own hold it (the far side of a fold, or a pose at a roll's end
    /// moving into it): covered with no other instant needed.
    Near,
    /// Its own contact climbs to each end of the roll: no other instant to cover it.
    Alone,
    /// The least depth outside its own contact, and where.
    Far(Cover),
}

/// The tool's signed depth at the point `x` (in space) at the roll `t`, and the point in the tool's
/// coordinates and its nearest piece.
fn depth_at(ring: &Ring,path: &Trajectory,x: V,t: f64) -> Option<(f64,V,usize)> {
    let p = path.pose(t).ok()?.inverse().point(x);
    let (d,k) = ring.signed(p);
    Some((d,p,k))
}

/// The golden-section least of `f` over `[lo, hi]`, to the rounding of the roll.
fn least(f: &dyn Fn(f64) -> f64,mut lo: f64,mut hi: f64) -> f64 {
    let g = 0.5*(5f64.sqrt()-1.);
    loop {
        let (x1,x2) = (hi-g*(hi-lo),lo+g*(hi-lo));
        if !(x1 > lo && x2 < hi && x1 < x2) { break }
        if f(x1) < f(x2) { hi = x2 } else { lo = x1 }
    }
    0.5*(lo+hi)
}

/// **How the point `x` of a sheet, its own instant `own`, stands under the sweep over `[a, b]`**
/// (`Reading`), the point standing off its sheet by as much as `off`, its depth read to `within` (the tolerance: a point held less deep than that is on
/// the boundary as the tolerance reads it). Its distance from the tool `d(τ)` is read along the roll
/// each way from `own`, the steps growing from the rounding's share of the roll by a quarter each,
/// and never longer than the stretch over which the point moves the larger of `|d|` and `within`
/// (`d` 1-Lipschitz in space: so no stretch held more than `within` deep is stepped over — a
/// certified march) nor a quarter of the ring's thickness (a feature size); the own contact runs
/// on each side while `d` climbs, its end (the greatest) settled by golden sections, and the least
/// of `d` past both ends settled the same way about the least sample.
pub fn cover(ring: &Ring,path: &Trajectory,[a,b]: [f64;2],x: V,own: f64,off: f64,within: f64) -> Result<Reading,Unbuilt> {
    let domain = crate::util::interval::Interval::new(a,b).map_err(|_| Unbuilt::Failed("a roll with no extent".into()))?;
    let speed = path.inverse_point_speed_bound(x,domain).map_err(|_| Unbuilt::Failed("a point's speed under the motion unbounded".into()))?;
    let most = if speed > 0. { 0.25*ring.thickness/speed } else { b-a };
    let first = HALF_READ*(b-a);
    let d = |t: f64| depth_at(ring,path,x,t).map_or(f64::INFINITY,|r| r.0);
    // (a side whose first depth clear of its rounding — the pose's and the distance's few roundings
    // of the point's place, read at the tool's size — is below nought: its neighbours cover the
    // point. Nearer, `d` is nought to its rounding either way)
    // (or, where the point stands off its sheet — a fitted sheet's point, `off` its fit — that:
    // about its own instant `d` is then nought to it)
    let rounding = (16.*f64::EPSILON*(crate::util::space::norm(x)+ring.size)).max(off);
    // each side's samples: the roll and the depth; none where the side holds the point (its march
    // stopped there, the lower side read whole before the upper is begun)
    let side = |dir: f64| -> Option<Vec<(f64,f64)>> {
        let end = if dir > 0. { b } else { a };
        let mut out = Vec::new();
        let (mut s,mut step) = (own,first);
        let mut clear = false;
        while (end-s)*dir > 0. {
            s = if (end-s-dir*step)*dir <= 0. { end } else { s+dir*step };
            let v = d(s);
            out.push((s,v));
            if !clear && v.abs() > rounding {
                if v < 0. { return None }
                clear = true;
            }
            step = (1.25*step).min(most).min(v.abs().max(within)/speed);
        }
        Some(out)
    };
    let Some(lower) = side(-1.) else { return Ok(Reading::Near) };
    let Some(upper) = side(1.) else { return Ok(Reading::Near) };
    // each side's own contact's end: the greatest sample before `d` first falls a rounding below
    // the greatest so far, settled between its neighbours
    let past = |s: &[(f64,f64)]| -> (usize,Option<f64>) {
        // (a fall within the rounding is no fall: about its own instant `d` is nought to it. Read
        // against the greatest sample so far, not the one before: where the own contact climbs
        // barely past the rounding and the far instant then falls slowly, no two neighbours stand a
        // rounding apart — the thin torus's lens beside its second swallowtail point climbed to
        // 3.1e-8 against a rounding of 3e-8, fell in steps of 1e-8, and was read `Alone` where an
        // instant 0.018 of the roll on held it 2.2e-6 deep)
        let mut top = 0;
        let fell = (1..s.len()).any(|k| {
            if s[k-1].1 > s[top].1 { top = k-1 }
            s[top].1 > rounding && s[k].1 < s[top].1-rounding
        });
        if !fell { return (s.len(),None) }
        let lo = if top >= 1 { s[top-1].0 } else { own };
        let hi = s[top+1].0;
        let (l,h) = if lo < hi { (lo,hi) } else { (hi,lo) };
        (top+1,Some(least(&|t| -d(t),l,h)))
    };
    let ((kl,el),(ku,eu)) = (past(&lower),past(&upper));
    // the least sample past each end, settled between its neighbours and within the far stretch
    // (each with the tool's reading at its instant where the golden section's was taken: read once)
    let mut best: Option<(f64,f64,Option<(f64,V,usize)>)> = None;
    for (s,k,e) in [(&lower,kl,el),(&upper,ku,eu)] {
        let Some(e) = e else { continue };
        let Some(i) = (k..s.len()).min_by(|&i,&j| s[i].1.total_cmp(&s[j].1)) else { continue };
        let ends = |j: usize| if j < k { e } else { s[j].0 };
        let (p,q) = (ends(i.saturating_sub(1).max(k-1)),if i+1 < s.len() { s[i+1].0 } else { s[i].0 });
        let (l,h) = if p < q { (p,q) } else { (q,p) };
        let t = if h > l { least(&d,l,h) } else { s[i].0 };
        let r = depth_at(ring,path,x,t);
        let dt = r.map_or(f64::INFINITY,|r| r.0);
        let v = dt.min(s[i].1);
        let (t,r) = if dt <= s[i].1 { (t,r) } else { (s[i].0,None) };
        if best.is_none_or(|b| v < b.1) { best = Some((t,v,r)) }
    }
    let Some((t,_,r)) = best else { return Ok(Reading::Alone) };
    let Some((depth,point,piece)) = r.or_else(|| depth_at(ring,path,x,t)) else { return Err(Unbuilt::Failed("a pose not read".into())) };
    Ok(Reading::Far(Cover {depth,at:t,point,piece}))
}

/// **The point of a ring's sheet at `(θ, t)` on the contact labelled `l`**, in space, and how its
/// sheet bends there (`s''`, the fold's sign): `None` where the contact has no such root.
pub fn sheet(ring: &Ring,path: &Trajectory,whole: [f64;2],theta: f64,t: f64,l: Label) -> Option<(V,f64)> {
    let (w,dw) = super::rings::rates(path,t,whole).ok()?;
    let th = theta.rem_euclid(std::f64::consts::TAU);
    let r = ring.root(&w,th,l,false)?;
    Some((path.pose(t).ok()?.point(r.x),ring.bend(&w,&dw,th,&r)))
}

/// The twist and the pose at the roll `t`, read once for every point of a sheet's row there
/// (`sheet`'s, but for its bend): none where either is not read.
fn row(path: &Trajectory,whole: [f64;2],t: f64) -> Option<(Twist,Pose)> {
    let (w,_) = super::rings::rates(path,t,whole).ok()?;
    Some((w,path.pose(t).ok()?))
}

/// The point of a ring's sheet at `θ` on the contact labelled `l` on the row `(w, pose)` (`row`):
/// `sheet`'s point.
fn on_row(ring: &Ring,(w,pose): &(Twist,Pose),theta: f64,l: Label) -> Option<V> {
    Some(pose.point(ring.root(w,theta.rem_euclid(std::f64::consts::TAU),l,false)?.x))
}

/// The label of the contact at `(θ, t)` nearest the place `σ` round the meridian.
pub fn label_at(ring: &Ring,path: &Trajectory,theta: f64,t: f64,sigma: f64) -> Option<Label> {
    let w = Twist::body(path.at(t).ok()?);
    let nf = ring.elements.len() as f64;
    let apart = |x: f64| { let d = (x-sigma).rem_euclid(nf); d.min(nf-d) };
    ring.roots(&w,theta.rem_euclid(std::f64::consts::TAU)).into_iter().min_by(|a,b| apart(a.1.sigma).total_cmp(&apart(b.1.sigma))).map(|r| r.0)
}

/// The far cover's depth at the sheet's point `(θ, t)` on `l` (`cover`): below nought covered by
/// another instant; `None` where the sheet has no point there, the neighbours hold it, or nothing
/// else can.
pub fn far_depth(ring: &Ring,path: &Trajectory,roll: [f64;2],theta: f64,t: f64,l: Label,within: f64) -> Option<Cover> {
    let (x,_) = sheet(ring,path,roll,theta,t,l)?;
    match cover(ring,path,roll,x,t,0.,within).ok()? { Reading::Far(c) => Some(c),_ => None }
}

/// The tool's depth at the point `x` at each of `ts` (a probe of `cover`).
pub fn depths(ring: &Ring,path: &Trajectory,x: V,ts: &[f64]) -> Vec<f64> {
    ts.iter().map(|&t| depth_at(ring,path,x,t).map_or(f64::NAN,|r| r.0)).collect()
}

/// **Where the tool at the roll's start meets the tool at its end on the boundary** (#942
/// T1d-tori, the full turn): over a whole turn the end pose stands beside the start, and the two
/// tools meet. Most of that meeting is buried — one of the two, or both, covered there by the
/// instants between (a tangent touch where their tubes cross among them: no cut is needed there) —
/// but an arc of it may bound the sweep, both tools uncovered beside it but by one another: the full
/// turn's, 0.027 long between the start pose's rim and its cut by the tube near the end, which no
/// edge crosses and the trim's search did not find. Such an arc ends where its cover changes hands,
/// at a corner on another seam of the start pose's face (a tube's trace with it, a rim), each
/// given: so the corners are found where the other pose's signed distance changes sign along the
/// curves `given` lays on a face of either pose (`corners_along`), the meeting traced through them
/// (`meeting::through`) and settled on both surfaces, cut at them, and each
/// piece between two corners given where both tools are uncovered at its middle (`cover`, its point
/// standing off each sheet by the trace's tolerance). Nothing where the poses do not meet
/// uncovered, nor from a corner whose branch the tracer does not read.
pub fn ends_met(ring: &Ring,path: &Trajectory,[a,b]: [f64;2],ends: [&crate::topo::Brep;2],given: &[crate::boolean::Given],tol: f64)
    -> Result<Vec<crate::boolean::Given>,Unbuilt> {
    use crate::util::space::distance;
    let poses = [path.pose(a)?,path.pose(b)?];
    let inverses = poses.map(|p| p.inverse());
    // (whether the point `x` of the pose at the roll's end `e`, standing off its sheet by as much as
    // `off`, is uncovered: on the boundary as the tolerance reads it)
    let uncovered = |x: V,e: usize,off: f64| match cover(ring,path,[a,b],x,[a,b][e],off,tol)? {
        Reading::Near => Ok::<bool,Unbuilt>(false),
        Reading::Alone => Ok(true),
        Reading::Far(c) => Ok(c.depth >= -tol),
    };
    let mut out = Vec::new();
    // (each given curve's corners against the pose at the end `e` depend on the curve and `e` alone:
    // read once for every pair of faces, by its place in `given` — each with whether it is on the
    // boundary, read the first time it is asked)
    let mut along: std::collections::BTreeMap<(usize,usize,usize),Vec<(V,Option<bool>)>> = std::collections::BTreeMap::new();
    for fs in &ends[0].faces { for fe in &ends[1].faces {
        // the corners: where the other pose's signed distance changes sign along a seam of either
        // face (a seam of the start pose's against the end pose, of the end pose's against the start)
        let mut corners: Vec<V> = Vec::new();
        for (gi,gv) in given.iter().enumerate() {
            let Some(e) = [&fs.surface,&fe.surface].iter().position(|s| gv.surfaces.iter().any(|t| t == *s)) else { continue };
            let other = |x: V| ring.signed(inverses[1-e].point(x)).0;
            for (ci,c) in gv.curves.iter().enumerate() {
                let found = along.entry((gi,ci,e)).or_insert_with(|| corners_along(c,&other,tol).into_iter().map(|x| {
                    // (a corner within twice the tolerance of an end of a given curve — where a seam passes
                    // from one face of a tube to the next, its two curves' shared end — is that end: the
                    // arc then ends at the vertex the seams meet at, by construction)
                    let x = given.iter().flat_map(|h| h.curves.iter()).flat_map(curve_ends)
                        .filter(|&y| distance(x,y) <= 2.*tol).min_by(|p,q| distance(x,*p).total_cmp(&distance(x,*q))).unwrap_or(x);
                    (x,None)
                }).collect());
                for (x,on) in found.iter_mut() {
                    let x = *x;
                    // (two corners within twice the tolerance are one place)
                    if corners.iter().any(|&y| distance(x,y) <= 2.*tol) { continue }
                    // (a corner of an arc on the boundary is on it: neither pose covered there by more than
                    // the tolerance, nor held by its neighbours)
                    let on = match on { Some(o) => *o,None => *on.insert(uncovered(x,0,0.)? && uncovered(x,1,tol)?) };
                    if !on { continue }
                    corners.push(x);
                }
            }
        }
        if corners.len() < 2 { continue }
        // the meeting traced through each corner (`meeting::through`, the kernel's one fallback) within
        // a box about it, grown from the nearest other corner's distance till the branch through it
        // reaches another corner each way it is uncovered, or the box holds both poses
        let (da,db) = (super::general::parameters(ends[0],tol),super::general::parameters(ends[1],tol));
        let (l0,h0) = ends[0].bounds();
        let (l1,h1) = ends[1].bounds();
        let (lw,hw): (V,V) = (std::array::from_fn(|k| l0[k].min(l1[k])),std::array::from_fn(|k| h0[k].max(h1[k])));
        let mut arcs: Vec<Vec<V>> = Vec::new();
        for (i,&c) in corners.iter().enumerate() {
            let near = corners.iter().enumerate().filter(|&(j,_)| j != i).map(|(_,&p)| distance(p,c)).fold(f64::INFINITY,f64::min);
            let mut grow = near.max(tol);
            loop {
                let (lo,hi): (V,V) = (std::array::from_fn(|k| (c[k]-grow).max(lw[k])),std::array::from_fn(|k| (c[k]+grow).min(hw[k])));
                let whole = (0..3).all(|k| lo[k] <= lw[k] && hi[k] >= hw[k]);
                // (a branch the tracer does not read from this corner gives nothing from it: the trim then
                // meets the two poses by its own search, as it did before)
                let traced = match crate::meeting::through(&fs.surface,da,&fe.surface,db,&[c],lo,hi,tol) {
                    Ok(t) => t,
                    Err(e) => { if crate::timing() { eprintln!("ring: the end poses' meeting from {c:?} not traced: {e}") } break }
                };
                // (each point settled on the two surfaces' meeting by alternating feet: the traced spline
                // strays from it between its points by more than the tolerance)
                let settle = |mut x: V| { for _ in 0..FEET { let y = fe.surface.point(fe.surface.inverse(fs.surface.point(fs.surface.inverse(x))));
                    if distance(x,y) == 0. { break } x = y } x };
                let lines: Vec<(Vec<V>,bool)> = traced.iter().filter_map(polyline).map(|(xs,closed)| (xs.into_iter().map(settle).collect(),closed)).collect();
                let (found,open) = between_corners(&lines,&corners,c,tol);
                let open = open.iter().map(|xs| Ok::<bool,Unbuilt>(uncovered(xs[xs.len()/2],0,tol)? && uncovered(xs[xs.len()/2],1,tol)?))
                    .collect::<Result<Vec<bool>,_>>()?.into_iter().any(|o| o);
                if !open || whole {
                    for xs in found {
                        // (each arc once: traced from either of its corners)
                        let (p,q) = (xs[0],xs[xs.len()-1]);
                        if arcs.iter().any(|y| distance(y[0],q) <= tol && distance(y[y.len()-1],p) <= tol
                            || distance(y[0],p) <= tol && distance(y[y.len()-1],q) <= tol) { continue }
                        arcs.push(xs);
                    }
                    break
                }
                grow *= 2.;
            }
        }
        let mut curves = Vec::new();
        for xs in arcs {
            // (a traced point stands off either pose by as much as the trace's tolerance)
            let mid = xs[xs.len()/2];
            if !(uncovered(mid,0,tol)? && uncovered(mid,1,tol)?) { continue }
            if crate::timing() { eprintln!("ring: the end poses meet uncovered from {:?} to {:?}, {} points",xs[0],xs[xs.len()-1],xs.len()) }
            curves.push(super::rings::curve_through(xs,tol)?.0);
        }
        if !curves.is_empty() { out.push(crate::boolean::Given {surfaces:[fs.surface.clone(),fe.surface.clone()],curves,touches:Vec::new()}) }
    } }
    Ok(out)
}

/// A given curve's two ends, where it has a span (`corners_along`'s).
fn curve_ends(c: &crate::geom::Curve) -> Vec<V> {
    match c {
        crate::geom::Curve::BSpline(b) => { let [t0,t1] = b.domain(); vec![c.point(t0),c.point(t1)] }
        crate::geom::Curve::Iso(..) => vec![c.point(0.),c.point(1.)],
        _ => Vec::new(),
    }
}

/// Where `g` (1-Lipschitz in space) changes sign along the curve `c`: marched over its span, each
/// step no longer in space than the larger of `|g|` and the tolerance (so no stretch where `g`
/// stands clear of nought by more is stepped over), each change halved to the rounding.
fn corners_along(c: &crate::geom::Curve,g: &dyn Fn(V) -> f64,tol: f64) -> Vec<V> {
    use crate::geom::Curve;
    let [t0,t1] = match c {
        Curve::BSpline(b) => b.domain(),
        Curve::Iso(..) => [0.,1.],
        Curve::Circle(..) | Curve::Ellipse(..) => [0.,std::f64::consts::TAU],
        _ => return Vec::new(),
    };
    let mut out = Vec::new();
    let (mut t,mut v) = (t0,g(c.point(t0)));
    while t < t1 {
        let speed = crate::util::space::norm(c.tangent(t)).max(f64::MIN_POSITIVE);
        let next = (t+v.abs().max(tol)/speed).min(t1);
        let w = g(c.point(next));
        if v*w < 0. {
            let (mut lo,mut hi) = (t,next);
            while lo < 0.5*(lo+hi) && 0.5*(lo+hi) < hi {
                let m = 0.5*(lo+hi);
                if g(c.point(m))*v > 0. { lo = m } else { hi = m }
            }
            out.push(c.point(0.5*(lo+hi)));
        }
        (t,v) = (next,w);
    }
    out
}

/// How many alternating feet settle a point on two surfaces' meeting at most: at a crossing at an
/// angle each halves its distance at least by the cosine's share, so fifty reach the rounding from
/// the trace's tolerance where the surfaces cross at a sine above a tenth (the arcs given: 0.6).
const FEET: usize = 50;

/// A traced meeting's points and whether it closes: a trace's own, a spline's at its knots.
fn polyline(c: &crate::geom::Curve) -> Option<(Vec<V>,bool)> {
    match c {
        crate::geom::Curve::Traced(t) => Some((t.pts.clone(),t.closed)),
        // (a spline through a trace's points: its knots are the points, each span cut in sixteen, each
        // point then settled on the meeting)
        crate::geom::Curve::BSpline(b) => {
            let mut ts: Vec<f64> = b.knots.windows(2).filter(|w| w[1] > w[0]).flat_map(|w| (0..16).map(move |k| w[0]+(w[1]-w[0])*k as f64/16.)).collect();
            ts.push(b.domain()[1]);
            let xs: Vec<V> = ts.iter().map(|&t| c.point(t)).collect();
            let closed = crate::util::space::distance(xs[0],xs[xs.len()-1]) == 0.;
            Some((xs,closed))
        }
        _ => None,
    }
}

/// The traced pieces through the corner `from` cut at the corners (each corner placed at its nearest
/// point of the piece it lies on, within twice the tolerance): each stretch between two corners, its
/// points in space and the corners its ends; and each stretch from `from` to the window's rim
/// (the window grown where one is on the boundary).
fn between_corners(lines: &[(Vec<V>,bool)],corners: &[V],from: V,tol: f64) -> (Vec<Vec<V>>,Vec<Vec<V>>) {
    use crate::util::space::distance;
    let mut arcs = Vec::new();
    let mut open = Vec::new();
    for (xs,closed) in lines {
        let closed = *closed;
        // (each corner on the piece: the index of its nearest point)
        // (each corner on the piece: the index of the chord it lies on, within twice the tolerance —
        // the chord's to the zero set and the corner's to its seam — and its place along the chord)
        let mut cuts: Vec<(usize,f64,V)> = corners.iter().filter_map(|&c| {
            let (i,d) = xs.windows(2).enumerate().map(|(i,w)| (i,crate::util::space::segment_distance(c,w[0],w[1])))
                .min_by(|a,b| a.1.total_cmp(&b.1))?;
            let along = distance(c,xs[i]);
            (d <= 2.*tol).then_some((i,along,c))
        }).collect();
        cuts.sort_by(|a,b| (a.0,a.1).partial_cmp(&(b.0,b.1)).unwrap());
        if !cuts.iter().any(|c| c.2 == from) { continue }
        // (from `from`, the first or the last corner, to the window's rim: a stretch not yet ended)
        let k = cuts.iter().position(|c| c.2 == from).unwrap();
        if k == 0 && !closed { let c = cuts[0]; open.push([&[c.2],&xs[..=c.0]].concat()) }
        if k+1 == cuts.len() && !closed { let c = cuts[k]; open.push([&[c.2],&xs[c.0+1..]].concat()) }
        // (a stretch between two corners on the chords `i` and `j`: the corners and the points between)
        let stretch = |i: usize,ci: V,j: usize,cj: V| -> Vec<V> {
            let mut arc = vec![ci];
            if j > i { arc.extend(xs[i+1..=j].iter().copied()) } else if j < i { arc.extend(xs[i+1..xs.len()-1].iter().chain(&xs[..=j]).copied()) }
            arc.push(cj);
            arc
        };
        for w in cuts.windows(2) { arcs.push(stretch(w[0].0,w[0].2,w[1].0,w[1].2)) }
        // (a closed piece: the stretch from its last corner round to its first)
        if closed && cuts.len() >= 2 {
            let (l,f) = (cuts[cuts.len()-1],cuts[0]);
            arcs.push(stretch(l.0,l.2,f.0,f.2));
        }
    }
    // (a stretch of no length — every point within twice the tolerance of its first — bounds nothing:
    // two corners that are one place, #942 F6)
    arcs.retain(|a| a.iter().any(|&x| distance(x,a[0]) > 2.*tol));
    (arcs,open)
}

/// **The far cover as a field over a ring's sheet** (#942, the depth-based trim): a sheet's points
/// by `(θ, t)` on the contact `label`, each read by `cover` (`within` the tolerance) — the value the
/// far cover's depth, what it was read at the `Cover` (the covering instant and the tool's point
/// there: the partner). Where the sheet has no point, folds away (`s''` at or below nought: the far
/// side of a fold, which its neighbours hold), or nothing else covers it (`Alone`), the field is not
/// defined: `zero_set` reads that as outside its domain. Its gradient is the envelope theorem's
/// (`gradient`).
pub struct Covered<'a> { pub ring: &'a Ring,pub path: &'a Trajectory,pub roll: [f64;2],pub label: Label,pub within: f64 }

impl Covered<'_> {
    /// The sheet's point at `(θ, t)`, where it touches.
    fn point(&self,[th,t]: [f64;2]) -> Option<V> {
        let (x,bend) = sheet(self.ring,self.path,self.roll,th,t,self.label)?;
        (bend > 0.).then_some(x)
    }

    /// The covering instant's preimage on the ring's sheet: the angle of the tool's point round the
    /// ring's axis, the instant, and the place round the meridian of the root there nearest it.
    /// `None` at an end of the roll (the pose there is the tool's own face, met by the Boolean) or
    /// where no root stands within the tolerance of the point.
    pub fn partner(&self,c: &Cover) -> Option<(f64,f64,f64)> {
        let [a,b] = self.roll;
        if !(c.at > a && c.at < b) { return None }
        self.preimage(c)
    }

    /// The covering instant's preimage on the ring's sheet, `partner`'s, at an end of the roll too:
    /// there the end pose's row.
    fn preimage(&self,c: &Cover) -> Option<(f64,f64,f64)> {
        let l = self.ring.frame.local(c.point);
        let th = l[1].atan2(l[0]).rem_euclid(std::f64::consts::TAU);
        let w = Twist::body(self.path.at(c.at).ok()?);
        self.ring.roots(&w,th).into_iter().map(|(_,r)| (crate::util::space::distance(r.x,c.point),r.sigma))
            .filter(|p| p.0 <= self.within).min_by(|p,q| p.0.total_cmp(&q.0)).map(|p| (th,c.at,p.1))
    }

    /// **The far cover's gradient in `(θ, t)`** where its least is taken at the instant `at`: by
    /// the envelope theorem the tool's signed distance's gradient there (its normal, read in the
    /// tool by central differences a cube root of the rounding of its thickness apart, turned into
    /// space) against the sheet's two rates. Differences of the depth itself, at the half digits
    /// `zero_set` steps by, read the noise of the sheet's root (the half turn's chords accepted
    /// 3.6e-5 off its zero set, eleven tolerances).
    fn gradient(&self,uv: [f64;2],x: V,at: f64) -> Option<[f64;2]> {
        let pose = self.path.pose(at).ok()?;
        let p = pose.inverse().point(x);
        let h = f64::EPSILON.powf(1./3.)*self.ring.thickness;
        let n = [0,1,2].map(|k| {
            let (mut a,mut b) = (p,p);
            a[k] += h;
            b[k] -= h;
            (self.ring.signed(a).0-self.ring.signed(b).0)/(2.*h)
        });
        let n = pose.vector(n);
        let (_,su,sv) = crate::zero_set::Field::surface(self,uv);
        let g = [crate::util::space::dot(n,su),crate::util::space::dot(n,sv)];
        g.iter().all(|v| v.is_finite()).then_some(g)
    }

    /// The place round the meridian of the sheet's own root at `(θ, t)`.
    pub fn sigma(&self,[th,t]: [f64;2]) -> Option<f64> {
        let w = Twist::body(self.path.at(t).ok()?);
        Some(self.ring.root(&w,th.rem_euclid(std::f64::consts::TAU),self.label,false)?.sigma)
    }
}

impl crate::zero_set::Field for Covered<'_> {
    type At = Cover;
    fn least(&self,uv: [f64;2]) -> Option<crate::zero_set::Value<Cover>> {
        let x = self.point(uv)?;
        match cover(self.ring,self.path,self.roll,x,uv[1],0.,self.within).ok()? {
            Reading::Far(c) => Some(crate::zero_set::Value {f:c.depth,grad:self.gradient(uv,x,c.at),at:c}),
            _ => None,
        }
    }

    /// The least of the tool's depth at the point about the covering instant `from` names: over the
    /// stretch of the roll a quarter of the ring's thickness long in the point's motion either side
    /// of it (the feature `cover` steps by), within the roll and short of the point's own instant
    /// by half the way to it (its own contact, nought there, is no cover).
    fn along(&self,uv: [f64;2],from: &Cover) -> Option<crate::zero_set::Value<Cover>> {
        let x = self.point(uv)?;
        let [a,b] = self.roll;
        let domain = crate::util::interval::Interval::new(a,b).ok()?;
        let speed = self.path.inverse_point_speed_bound(x,domain).ok()?;
        let reach = if speed > 0. { 0.25*self.ring.thickness/speed } else { b-a };
        let (mut lo,mut hi) = ((from.at-reach).max(a),(from.at+reach).min(b));
        let own = uv[1];
        if own > lo && own < hi { if from.at > own { lo = 0.5*(own+from.at) } else { hi = 0.5*(own+from.at) } }
        let d = |t: f64| depth_at(self.ring,self.path,x,t).map_or(f64::INFINITY,|r| r.0);
        let t = if hi > lo { least(&d,lo,hi) } else { from.at };
        let (depth,point,piece) = depth_at(self.ring,self.path,x,t)?;
        Some(crate::zero_set::Value {f:depth,grad:self.gradient(uv,x,t),at:Cover {depth,at:t,point,piece}})
    }

    /// The sheet's point and its two rates, by central differences a cube root of the rounding of
    /// an angle and of the roll apart (where the difference's rounding and its truncation balance).
    /// (each row's twist and pose read once, for the three points on it)
    fn surface(&self,[th,t]: [f64;2]) -> (V,V,V) {
        let h = f64::EPSILON.powf(1./3.);
        let ht = h*(self.roll[1]-self.roll[0]);
        let at = |r: &Option<(Twist,Pose)>,th: f64| r.as_ref().and_then(|r| on_row(self.ring,r,th,self.label)).unwrap_or([f64::NAN;3]);
        let rate = |p: V,q: V,step: f64| crate::util::space::scale(crate::util::space::sub(p,q),0.5/step);
        let (own,later,earlier) = (row(self.path,self.roll,t),row(self.path,self.roll,t+ht),row(self.path,self.roll,t-ht));
        (at(&own,th),rate(at(&own,th+h),at(&own,th-h),h),rate(at(&later,th),at(&earlier,th),ht))
    }

    fn place(&self,[th,t]: [f64;2]) -> V {
        row(self.path,self.roll,t).and_then(|r| on_row(self.ring,&r,th,self.label)).unwrap_or([f64::NAN;3])
    }
}

/// **A boundary read from the depth, as a seam** (#942): one piece of the curve where another
/// instant's cover of a sheet begins — its points on the sheet `(θ, t, σ)`, how its two ends end
/// (`zero_set::End`), and its partner: the covering instant's preimages `(θ', τ, σ')`, run by run
/// where the covering instant lies inside the roll (where it is an end of the roll the partner is
/// the tool's own face posed there, which the Boolean meets) — and the sheet's own points under each
/// run of the partner, point for point (`own`); and where a run's cover passes on to an end pose,
/// its corner there before its first point and after its last, with its partner on that end's row
/// (`corners`: held to the tolerance in space, not traced).
#[derive(Clone,Debug)]
pub struct Seam { pub sheet: Vec<(f64,f64,f64)>,pub ends: [crate::zero_set::End;2],pub partner: Vec<Vec<(f64,f64,f64)>>,
    pub own: Vec<Vec<(f64,f64,f64)>>,pub corners: Vec<[Option<((f64,f64,f64),(f64,f64,f64))>;2]> }

impl Seam {
    /// Its curves as the ring road lays a double curve's (`rings::cut_along`): the sheet's, then
    /// each run of its partner's.
    pub fn curves(&self) -> Vec<Vec<(f64,f64,f64)>> {
        std::iter::once(self.sheet.clone()).chain(self.partner.iter().cloned()).filter(|c| c.len() >= 2).collect()
    }
}

/// **Where another instant's cover of a ring's sheet begins, traced as seams** (#942, the
/// depth-based trim): the zero set of the far cover's depth (`Covered`) over the box `rect` of the
/// sheet's `(θ, t)` on the contact `label`, by `zero_set::trace` to `within` (the tolerance) in space
/// with `feature` its longest step and the spacing it samples at, from `seeds` (events the caller
/// knows: a swallowtail point, a double curve's crossing of a stretch's end) — each piece with the
/// covering instant's preimages, so both faces' parameters come by construction. A cell the search
/// for closed curves left unresolved is refused by name, never dropped.
#[allow(clippy::too_many_arguments)]
pub fn seams(ring: &Ring,path: &Trajectory,roll: [f64;2],label: Label,rect: crate::zero_set::Rect,seeds: &[[f64;2]],within: f64,feature: f64)
    -> Result<Vec<Seam>,Unbuilt> {
    let field = Covered {ring,path,roll,label,within};
    let q = crate::zero_set::Question {rect,edges:&[],seeds,tol:within,feature};
    let traced = crate::zero_set::trace(&field,&q).map_err(|why| Unbuilt::Failed(format!("the cover's boundary on a sheet: {why}")))?;
    if let Some(cell) = traced.search.unresolved.first() {
        return Err(Unbuilt::Failed(format!("the cover's boundary on a sheet: the cell {cell:?} of its (θ, t) not resolved")))
    }
    seamed(&field,&traced,within,feature)
}

/// **`seams` through the caller's seeds alone** (#942, `zero_set::through`): each seed pulled onto
/// the cover's boundary and marched both ways, with no search for the rest — for a caller that knows
/// where its seams leave a row (the full turn's tube where it passes back over its own start) and
/// cannot pay for the proof over the whole sheet. Nothing is claimed complete.
#[allow(clippy::too_many_arguments)]
pub fn seams_through(ring: &Ring,path: &Trajectory,roll: [f64;2],label: Label,rect: crate::zero_set::Rect,seeds: &[[f64;2]],within: f64,feature: f64)
    -> Result<Vec<Seam>,Unbuilt> {
    let field = Covered {ring,path,roll,label,within};
    let q = crate::zero_set::Question {rect,edges:&[],seeds,tol:within,feature};
    let traced = crate::zero_set::through(&field,&q).map_err(|why| Unbuilt::Failed(format!("the cover's boundary on a sheet: {why}")))?;
    seamed(&field,&traced,within,feature)
}

/// The pieces traced of the cover's boundary as seams: each held to the tolerance in space
/// (`refined`), with its places round the meridian and its partner, each once.
fn seamed(field: &Covered,traced: &crate::zero_set::Traced<Cover>,within: f64,feature: f64) -> Result<Vec<Seam>,Unbuilt> {
    let mut out = Vec::new();
    // (each seam's sheet's points in space, in step with `out`)
    let mut places: Vec<Vec<Option<V>>> = Vec::new();
    for p in &traced.pieces {
        let (uvs,ats) = refined(field,&p.uv,&p.at)?;
        let mut sheet = Vec::with_capacity(uvs.len());
        for &uv in &uvs {
            let Some(sg) = field.sigma(uv) else { return Err(Unbuilt::Failed(format!("the cover's boundary at {uv:?}: no root of the sheet"))) };
            sheet.push((uv[0],uv[1],sg));
        }
        // (each run of points a covering instant inside the roll holds; where it passes to an end
        // pose's cover, its corner there — the point whose covering instant reaches that end, where
        // the covering sheet's row at the end crosses the seam — put on it, with its partner)
        let mut partner: Vec<Vec<(f64,f64,f64)>> = Vec::new();
        let mut own: Vec<Vec<(f64,f64,f64)>> = Vec::new();
        let mut corners = Vec::new();
        let partners: Vec<Option<(f64,f64,f64)>> = ats.iter().map(|c| field.partner(c)).collect();
        let mut j = 0;
        while j < ats.len() {
            if partners[j].is_none() { j += 1; continue }
            let mut k = j;
            while k+1 < ats.len() && partners[k+1].is_some() { k += 1 }
            let run: Vec<((f64,f64,f64),(f64,f64,f64))> = (j..=k).filter_map(|i| Some((sheet[i],partners[i]?))).collect();
            if run.len() >= 2 {
                own.push(run.iter().map(|r| r.0).collect());
                partner.push(run.iter().map(|r| r.1).collect());
                corners.push([if j > 0 { corner(field,uvs[j-1],uvs[j],&ats[j]) } else { None },
                    if k+1 < ats.len() { corner(field,uvs[k+1],uvs[k],&ats[k]) } else { None }]);
            }
            j = k+1;
        }
        // (a piece traced again from a seed on one already traced: its ends those of the earlier
        // within twice the tolerance — each within it of the one end — and every point of it within
        // the feature of the earlier's points, the spacing below which the trace tells nothing
        // apart. Two traces of one curve beside a swallowtail point stand some 1e-5 apart between
        // their points: along the kernel of the sheet's rates a chord's sagitta is not its depth's)
        // (each point's place read once)
        let xs: Vec<Option<V>> = sheet.iter().map(|q| field.point([q.0,q.1])).collect();
        let close = |x: Option<V>,y: Option<V>,r: f64| x.zip(y).is_some_and(|(x,y)| crate::util::space::distance(x,y) <= r);
        let near = |ys: &Vec<Option<V>>| {
            let (n,m) = (xs.len(),ys.len());
            let ends = (close(xs[0],ys[0],2.*within) && close(xs[n-1],ys[m-1],2.*within))
                || (close(xs[0],ys[m-1],2.*within) && close(xs[n-1],ys[0],2.*within));
            ends && xs.iter().all(|&x| ys.iter().any(|&y| close(x,y,feature)))
        };
        if places.iter().any(near) { continue }
        out.push(Seam {sheet,ends:p.ends,partner,own,corners});
        places.push(xs);
    }
    Ok(out)
}

/// **Where a ring's sheet passes back over itself, as the ring road lays a double curve** (#942
/// T1d-tori, the full turn): over a whole turn the tube late in the roll sweeps back across the tube
/// where it began, and their crossing — no fold pair's, so no double curve the road traces — parts
/// what the late tube covers of the early one from what stays on the boundary. Without it the early
/// tube's piece beside the start pose's rim held both, and was kept. Such a crossing leaves a sheet
/// at a row of the roll's end, where another instant's cover of the row begins: each row (`t` at
/// either end of the roll, every contact's root on it touching) read for its far cover (`far_depth`)
/// at angles no farther apart round the ring than the cover's feature — a quarter of the ring's
/// thickness, the stretch of the roll `cover` steps by — and wherever the depth changes sign between
/// two, the cover's boundary traced through there over the sheet (`seams_through`). Of each seam only
/// the runs a covering instant inside the roll holds are a crossing of the sheet with itself: where
/// the end pose covers, its trace with the sheet is the meeting the end pose is given already
/// (`rings::entered_at`). Each run is laid as a double curve is — the covered sheet's points, then
/// the covering instant's preimages (`Seam::own`, `Seam::partner`) — unless it is a double curve
/// the road traced (`known`: every point within the feature of one of its points, the spacing below
/// which a trace tells nothing apart). Where its cover passes to an end pose, a run ends at its
/// corner on that pose's trace (`corner`). Entered by the road (`closed::ring_entered`), cut into the
/// stretches with the double curves (`rings::cut_along`): the full turn's run through the start rim
/// closes the start torus, and another, on the end tube's row at θ 0.48–1.17 covered by the tube at
/// τ ≈ −1.94…−2.09, has its partner end on the end torus's trace across that tube, a closed curve
/// inside a face — read through those ends (`rings::traces`' seeds), a hole in the face's piece.
pub fn passed_back(ring: &Ring,path: &Trajectory,[a,b]: [f64;2],known: &[Vec<(f64,f64,f64)>],tol: f64)
    -> Result<Vec<Vec<(f64,f64,f64)>>,Unbuilt> {
    use crate::util::space::distance;
    use std::f64::consts::{PI,TAU};
    let feature = 0.25*ring.thickness;
    // (the angles round the ring each row is read at: its points stand within the ring's size of
    // the axis, so these are no farther apart along it than the feature)
    let n = (TAU*ring.size/feature).ceil().max(1.) as usize;
    let angles: Vec<f64> = (0..n).map(|k| TAU*k as f64/n as f64).collect();
    // (the double curves' points in space, each on the root its place round the meridian names)
    let point = |q: (f64,f64,f64)| -> Option<V> {
        let l = label_at(ring,path,q.0,q.1,q.2)?;
        Some(sheet(ring,path,[a,b],q.0,q.1,l)?.0)
    };
    // (each read once, on the first seed: the points traced, and each double curve's chords in space,
    // a swallowtail point's marker parting them)
    let known_read = std::cell::OnceCell::new();
    let read_known = || known_read.get_or_init(|| {
        let xs: Vec<Vec<Option<V>>> = known.iter().map(|c| c.iter().map(|&q| point(q)).collect()).collect();
        let traced: Vec<V> = known.iter().flatten().zip(xs.iter().flatten()).filter(|(q,_)| !q.0.is_nan()).filter_map(|(_,&x)| x).collect();
        let chords: Vec<[V;2]> = xs.iter().flat_map(|c| c.windows(2).filter_map(|w| Some([w[0]?,w[1]?]))).collect();
        (traced,chords)
    });
    let mut out: Vec<Vec<(f64,f64,f64)>> = Vec::new();
    // (the places of `out`'s points, in step with it)
    let mut laid: Vec<V> = Vec::new();
    for t in [a,b] {
        // each angle's touching roots on the row, and their far cover's depth
        let w = Twist::body(path.at(t)?);
        let read: Vec<Vec<(Label,f64)>> = crate::util::par::map(&angles,|&th| ring.roots(&w,th).into_iter()
            .filter_map(|(l,_)| Some((l,far_depth(ring,path,[a,b],th,t,l,tol)?.depth))).collect());
        let labels: std::collections::BTreeSet<Label> = read.iter().flatten().map(|r| r.0).collect();
        for l in labels {
            let depth = |k: usize| read[k%n].iter().find(|r| r.0 == l).map(|r| r.1);
            let seeds: Vec<[f64;2]> = (0..n).filter_map(|k| match (depth(k),depth(k+1)) {
                (Some(x),Some(y)) if (x < 0.) != (y < 0.) => Some([angles[k]+0.5*TAU/n as f64,t]),
                _ => None,
            }).collect();
            for seed in seeds {
                // (where a double curve the road traced crosses the row, the change is that curve's:
                // the seed, within half the angles' spacing of it along the row, no farther from its
                // chords than the feature)
                let Some(x) = sheet(ring,path,[a,b],seed[0],t,l).map(|s| s.0) else { continue };
                let (traced,chords) = read_known();
                let on_known = chords.iter().any(|&[p,q]| crate::util::space::segments_distance(x,x,p,q) <= feature);
                if std::env::var_os("SOLVENT_RING_DEBUG").is_some() {
                    eprintln!("ring: the cover changes on the row at {t} on {l:?} at θ {}{}",seed[0],if on_known { ", a double curve's" } else { "" })
                }
                if on_known { continue }
                // (round the ring a half turn either way from the seed, over the whole roll)
                let rect = [[seed[0]-PI,seed[0]+PI],[a,b]];
                for seam in seams_through(ring,path,[a,b],l,rect,&[seed],tol,feature)? {
                    for ((own,partner),ends) in seam.own.iter().zip(&seam.partner).zip(&seam.corners) {
                        // (with its corners, where its cover passes on to an end pose)
                        let (mut own,mut partner) = (own.clone(),partner.clone());
                        if let Some((o,p)) = ends[0] { own.insert(0,o); partner.insert(0,p) }
                        if let Some((o,p)) = ends[1] { own.push(o); partner.push(p) }
                        let xs: Vec<Option<V>> = own.iter().map(|&q| point(q)).collect();
                        let again = |ys: &[V]| xs.iter().all(|x| x.is_some_and(|x| ys.iter().any(|&y| distance(x,y) <= feature)));
                        if again(traced) { continue }
                        // (a run found from two seeds is laid once: every point of it within the
                        // feature of one laid)
                        if again(&laid) { continue }
                        laid.extend(xs.into_iter().flatten());
                        laid.extend(partner.iter().filter_map(|&q| point(q)));
                        out.push(own);
                        out.push(partner);
                    }
                }
            }
        }
    }
    if std::env::var_os("SOLVENT_RING_DEBUG").is_some() {
        for c in &out { eprintln!("ring: passed back over itself, a curve of {} from {:?} to {:?}",c.len(),c[0],c[c.len()-1]) }
    }
    Ok(out)
}

/// **Where a seam's cover passes from an instant inside the roll to an end pose** (`seamed`): on
/// the chord from `out`, a point an end pose covers, to `inside`, one the instant `from` covers,
/// halved — each middle read along `from`'s branch of the cover (`Covered::along`), its least at an
/// end of the roll on `out`'s side — to the rounding. There the covering instant is the end's, its preimage the covering sheet's row at that end: the point on
/// `inside`'s side and that preimage, or `None` where the chord holds no such change.
fn corner(field: &Covered,out: [f64;2],inside: [f64;2],from: &Cover) -> Option<((f64,f64,f64),(f64,f64,f64))> {
    use crate::zero_set::Field;
    // (an instant is the end's where the tool's point at the place moves within the tolerance
    // between them: the least's search settles some 1e-9 of the roll short of the end it is at)
    let [a,b] = field.roll;
    let speed = field.path.inverse_point_speed_bound(field.point(out)?,crate::util::interval::Interval::new(a,b).ok()?).ok()?;
    let at_end = |c: &Cover| (c.at-a).min(b-c.at)*speed <= field.within;
    let (mut p,mut q) = (out,inside);
    let mut c = *from;
    if !at_end(&field.along(p,from)?.at) { return None }
    // (each halving takes the chord nearer the corner, to the rounding: the count is its, past which
    // a middle is one of its ends)
    for _ in 0..64 {
        let m = [0.5*(p[0]+q[0]),0.5*(p[1]+q[1])];
        if m == p || m == q { break }
        let v = field.along(m,&c)?;
        if at_end(&v.at) { p = m } else { q = m; c = v.at }
    }
    let r = field.preimage(&c)?;
    // (the preimage on the end's row itself: the covering sheet's edge there)
    let r = (if c.at-a < b-c.at { a } else { b },r);
    let r = (r.1.0,if (r.1.1-a).abs() < (r.1.1-b).abs() { a } else { b },r.1.2);
    Some(((q[0],q[1],field.sigma(q)?),r))
}

/// Newton's steps a point is pulled onto the zero set by: one started within a chord's sagitta
/// converges quadratically in three or four, one that has not in sixteen is not converging
/// (structural, as `zero_set`'s corrector).
const PULLS: usize = 16;

/// **A traced piece held to the tolerance in space** (#942): where a sheet's two rates are nearly
/// one way (beside a swallowtail point, along the kernel of its rates) a first-order distance to
/// the zero set through the inverse metric reads a point far off it as near — the half turn's
/// double curve 0.05 from its swallowtail point, its chords accepted 3.6e-5 off. So each chord's
/// middle in the parameters is pulled onto the zero set (Newton along the branch's gradient in the
/// parameters) and, where that moves it more than the tolerance in space, put in and both halves
/// read again; a middle that does not settle is refused by name.
fn refined(field: &Covered,uv: &[[f64;2]],at: &[Cover]) -> Result<(Vec<[f64;2]>,Vec<Cover>),Unbuilt> {
    use crate::zero_set::Field;
    let pulled = |m: [f64;2],from: &Cover| -> Option<([f64;2],Cover)> {
        let mut m = m;
        let mut c = *from;
        for _ in 0..PULLS {
            let v = field.along(m,&c)?;
            c = v.at;
            let g = v.grad?;
            let gg = g[0]*g[0]+g[1]*g[1];
            if !(gg > 0.) { return None }
            let step = [v.f*g[0]/gg,v.f*g[1]/gg];
            m = [m[0]-step[0],m[1]-step[1]];
            if step[0].abs() <= f64::EPSILON*m[0].abs().max(1.) && step[1].abs() <= f64::EPSILON*m[1].abs().max(1.) { break }
        }
        let v = field.along(m,&c)?;
        let x = field.point(m)?;
        // (settled where what is left of the depth is within the rounding of the point's place)
        (v.f.abs() <= 16.*f64::EPSILON*(crate::util::space::norm(x)+field.ring.size)).then_some((m,v.at))
    };
    let (mut uvs,mut ats) = (vec![uv[0]],vec![at[0]]);
    for k in 1..uv.len() {
        let mut stack = vec![(uv[k],at[k])];
        while let Some(&(q,cq)) = stack.last() {
            let p = *uvs.last().unwrap();
            let cp = *ats.last().unwrap();
            let m = [0.5*(p[0]+q[0]),0.5*(p[1]+q[1])];
            let Some((z,cz)) = pulled(m,&cp) else {
                return Err(Unbuilt::Failed(format!("the cover's boundary on a sheet: the middle {m:?} of a chord not pulled onto it")))
            };
            let (Some(xm),Some(xz)) = (field.point(m),field.point(z)) else {
                return Err(Unbuilt::Failed(format!("the cover's boundary on a sheet: no point of it at {m:?}")))
            };
            let _ = cq;
            if crate::util::space::distance(xm,xz) <= field.within { uvs.push(q); ats.push(cq); stack.pop(); }
            else if stack.len() > PULLS*4 { return Err(Unbuilt::Failed(format!("the cover's boundary on a sheet: a chord at {m:?} halved \
                {} times and still off it",PULLS*4))) }
            else { stack.push((z,cz)) }
        }
    }
    Ok((uvs,ats))
}


// **The sampled reading** (#942, the touched entry): a path that encloses nothing.

/// **A ring carried over a roll, read for depth by sampling** (#942, the touched entry): its
/// instants sampled at `n + 1` equal steps (the poses' inverses kept), the least over each basin
/// the samples show settled from there. **No proof**: a basin narrower than a step can be missed,
/// where `cover`'s certified march cannot. The reading for a path that encloses nothing (the
/// bench's runner's frame, read only by evaluating it), used by `swept::touched` alone; where the
/// path encloses its motion, `cover` is the reading.
pub struct Sampled<'a> { pub ring: &'a Ring,pub path: &'a Trajectory,pub roll: [f64;2],ts: Vec<f64>,inv: Vec<Pose> }

/// **What holds a point** (the sampled reading's): the instant `t`, the signed distance there (below nought inside), the
/// tool's meridian piece nearest it and the point in the tool's own coordinates.
#[derive(Clone,Copy,Debug)]
pub struct Held { pub t: f64,pub depth: f64,pub piece: usize,pub at: V }

impl<'a> Sampled<'a> {
    /// The ring over `roll`, its instants first read at `n + 1` equal steps.
    pub fn new(ring: &'a Ring,path: &'a Trajectory,roll: [f64;2],n: usize) -> Result<Sampled<'a>,Unbuilt> {
        let ts: Vec<f64> = (0..=n).map(|i| roll[0]+(roll[1]-roll[0])*i as f64/n as f64).collect();
        let inv = ts.iter().map(|&t| path.pose(t).map(Pose::inverse)).collect::<Result<Vec<_>,_>>()
            .map_err(|e| Unbuilt::Failed(format!("a pose for depth: {e}")))?;
        Ok(Sampled {ring,path,roll,ts,inv})
    }

    /// The tool's signed distance at `x` at the instant `t` (`Ring::signed`), and its piece there.
    pub fn signed(&self,x: V,t: f64) -> (f64,usize) {
        let Ok(p) = self.path.pose(t) else { return (f64::NAN,0) };
        self.ring.signed(p.inverse().point(x))
    }

    fn cover_at(&self,x: V,t: f64) -> Held {
        let at = self.path.pose(t).map(|p| p.inverse().point(x)).unwrap_or([f64::NAN;3]);
        let (depth,piece) = self.ring.signed(at);
        Held {t,depth,piece,at}
    }

    /// **Every least of the depth at `x` over the roll**: each basin the samples show (an end of the
    /// roll among them where the depth rises from it) settled by golden section within its two
    /// neighbouring samples, to where the depth no longer reads a difference.
    pub fn minima(&self,x: V) -> Vec<Held> { self.minima_beside(x,None) }

    /// `minima`, with `own` taken in among the samples: a point's own instant on a sheet, so that its
    /// basin is read as its own (`cover`).
    fn minima_beside(&self,x: V,own: Option<f64>) -> Vec<Held> {
        let mut s: Vec<(f64,f64)> = self.ts.iter().zip(&self.inv).map(|(&t,p)| (t,self.ring.signed(p.point(x)).0)).collect();
        if let Some(t0) = own {
            // (and samples halving their way in toward it from the steps either side, until the
            // roll no longer reads a difference: a partner basin closing on the own one — the wedge
            // beside a corner, held from an instant 0.11 of its angle on — is parted from it)
            let h = (self.roll[1]-self.roll[0])/(self.ts.len()-1).max(1) as f64;
            let mut near = vec![t0];
            let mut d = h;
            while t0+d != t0 && t0-d != t0 {
                near.extend([t0-d,t0+d]);
                d *= 0.5;
            }
            s.extend(near.into_iter().filter(|&t| t >= self.roll[0] && t <= self.roll[1]).map(|t| (t,self.signed(x,t).0)));
            s.sort_by(|a,b| a.0.total_cmp(&b.0));
            s.dedup_by(|a,b| a.0 == b.0);
        }
        let n = s.len();
        let rounding = 16.*f64::EPSILON*(norm(x)+self.ring.size);
        let mut out = Vec::new();
        for i in 0..n {
            let d = s[i].1;
            let below = i > 0 && s[i-1].1 < d;
            let above = i+1 < n && s[i+1].1 < d;
            if below || above || (i > 0 && s[i-1].1 == d) { continue }
            if Some(s[i].0) == own { out.push(self.cover_at(x,s[i].0)); continue }
            // (beside the own instant, a sample at nought to the rounding is its touch read again)
            if own.is_some() && d.abs() <= rounding { continue }
            let (lo,hi) = (s[i.saturating_sub(1)].0,s[(i+1).min(n-1)].0);
            out.push(self.settled(x,lo,hi,s[i].0));
        }
        out
    }

    /// The least of the depth at `x` over `[lo, hi]`, from the sample `best`, by golden section.
    fn settled(&self,x: V,mut lo: f64,mut hi: f64,best: f64) -> Held {
        let g = 0.5*(5f64.sqrt()-1.);
        let f = |t: f64| self.signed(x,t).0;
        let mut b = self.cover_at(x,best);
        // (shrunk until the bracket is the roll's rounding: the depth is flat at its least to half the
        // digits, so the bracket, not the values, says when it is done)
        let (mut c,mut d) = (hi-g*(hi-lo),lo+g*(hi-lo));
        let (mut fc,mut fd) = (f(c),f(d));
        while hi-lo > 4.*f64::EPSILON*hi.abs().max(lo.abs()).max(1.) {
            if fc < fd { hi = d; d = c; fd = fc; c = hi-g*(hi-lo); fc = f(c) }
            else { lo = c; c = d; fc = fd; d = lo+g*(hi-lo); fd = f(d) }
        }
        for t in [c,d,0.5*(lo+hi)] { let k = self.cover_at(x,t); if k.depth < b.depth { b = k } }
        b
    }

    /// **What holds `x` from outside its own instant `t₀`**: the deepest least of the depth over the
    /// roll but `t₀`'s own basin — its own where `t₀` is itself a least among the samples (the sheet
    /// touching, not folded under), every basin otherwise. `None` where no other basin is.
    pub fn cover(&self,x: V,t0: f64) -> Option<Held> {
        // (a least at nought to the rounding of the depth there — the point's coordinates' digits,
        // the pose carrying it back — is the point's own touch read again beside its instant, the
        // depth that flat there; and an instant touching it exactly from beyond keeps it all the same)
        let rounding = 16.*f64::EPSILON*(norm(x)+self.ring.size);
        self.minima_beside(x,Some(t0)).into_iter().filter(|c| c.t != t0 && c.depth.abs() > rounding).min_by(|a,b| a.depth.total_cmp(&b.depth))
    }

    /// The point in space the tool puts its own point `at` at the instant `t`.
    pub fn placed(&self,at: V,t: f64) -> V { self.path.pose(t).map_or([f64::NAN;3],|p| p.point(at)) }

    /// How far the sample is from the settled least: a check on `minima`.
    pub fn apart(&self,a: &Held,x: V) -> f64 { distance(self.placed(a.at,a.t),x) }
}

/// **A point of a seam**: its parameters on the sheet `(e, s)`, the point, its own instant, and what
/// holds it from beyond — the partner instant, and the tool's point there, which is the partner
/// face's point by construction.
#[derive(Clone,Copy,Debug)]
pub struct SeamPoint { pub e: f64,pub s: f64,pub x: V,pub own: f64,pub cover: Held }

impl Sampled<'_> {
    /// The depth at a sheet's point from beyond its own instant: the deepest other least, `+∞` where
    /// none is.
    fn beyond(&self,(x,own): (V,f64)) -> f64 { self.cover(x,own).map_or(f64::INFINITY,|c| c.depth) }

    /// **Where cover from beyond begins on the line `e` of a sheet** (`sheet(e, s)`, its point and own
    /// instant): between `kept` and `held`, where the depth from beyond is not below nought at the
    /// one and below at the other, halved until the two points are within a quarter of `tol` in space.
    /// `None` where the ends do not read so. The kept end may be held by less than `tol`: a point
    /// held so is on the boundary to the tolerance — the rim beside a whole-touched meridian, where
    /// the tube's buried sliver stands 1e-10 off it, reads so.
    pub fn edge_on(&self,sheet: &dyn Fn(f64,f64) -> (V,f64),e: f64,[mut kept,mut held]: [f64;2],tol: f64) -> Option<SeamPoint> {
        if self.beyond(sheet(e,kept)) < -tol || self.beyond(sheet(e,held)) >= 0. { return None }
        // (each halving parts the stretch in two; it ends where the points are a quarter of the
        // tolerance apart, or where the parameter no longer halves)
        while distance(sheet(e,kept).0,sheet(e,held).0) > 0.25*tol {
            let mid = 0.5*(kept+held);
            if mid == kept || mid == held { break }
            if self.beyond(sheet(e,mid)) < 0. { held = mid } else { kept = mid }
        }
        let (x,own) = sheet(e,kept);
        let cover = self.cover(x,own).unwrap_or(Held {t:f64::NAN,depth:f64::INFINITY,piece:0,at:[f64::NAN;3]});
        Some(SeamPoint {e,s:kept,x,own,cover})
    }

    /// **A seam traced across a sheet as a graph over `e`** (the depth trim's curve, #942): from `e0`
    /// toward `e1`, at each `e` the edge of what is kept between the ends `bracket(e)` gives
    /// (`edge_on`), stepped as far as the chord between two edges sags within `tol` of the edge
    /// found at its middle, halved where it sags more. It stops where the bracket no longer reads a
    /// change of cover, or where the step can no longer be halved — refused by name only then.
    /// Each point carries its partner instant and the tool's point there (`SeamPoint::cover`).
    pub fn seam_over(&self,sheet: &dyn Fn(f64,f64) -> (V,f64),[e0,e1]: [f64;2],bracket: &dyn Fn(f64) -> [f64;2],tol: f64)
        -> Result<Vec<SeamPoint>,Unbuilt> {
        let at = |e: f64| self.edge_on(sheet,e,bracket(e),tol);
        let Some(first) = at(e0) else { return Ok(Vec::new()) };
        let mut out = vec![first];
        // (the first step a sixteenth of the stretch: structural, the chord test refines it)
        let mut step = (e1-e0)/16.;
        while (e1-out.last().unwrap().e)*(e1-e0) > 0. {
            let last = *out.last().unwrap();
            let e = if (e1-last.e-step)*(e1-e0) < 0. { e1 } else { last.e+step };
            let (Some(p),Some(m)) = (at(e),at(0.5*(last.e+e))) else { break };
            let mid = [0.5*(last.x[0]+p.x[0]),0.5*(last.x[1]+p.x[1]),0.5*(last.x[2]+p.x[2])];
            if distance(mid,m.x) > tol {
                step *= 0.5;
                if last.e+step == last.e { return Err(Unbuilt::Failed("a seam of the depth trim not traced: its step no longer halves".into())) }
                continue
            }
            out.push(m);
            out.push(p);
            step *= 2.;
        }
        Ok(out)
    }
}
