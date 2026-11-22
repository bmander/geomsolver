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
//! (The peer kernel cadrs, MIT OR Apache-2.0, `crates/cadrs_kernel/src/native/sweep_tube.rs`,
//! trims its sheets by the same reading; nothing of its code is ported here.)

use super::closed::Unbuilt;
use super::rings::{Label,Ring};
use crate::trajectory::{Trajectory,Twist};
use crate::util::fmath::HALF_READ;

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
    // each side's samples: the roll and the depth
    let side = |dir: f64| -> Vec<(f64,f64)> {
        let end = if dir > 0. { b } else { a };
        let mut out = Vec::new();
        let (mut s,mut step) = (own,first);
        while (end-s)*dir > 0. {
            s = if (end-s-dir*step)*dir <= 0. { end } else { s+dir*step };
            let v = d(s);
            out.push((s,v));
            step = (1.25*step).min(most).min(v.abs().max(within)/speed);
        }
        out
    };
    let (lower,upper) = (side(-1.),side(1.));
    // (a side whose first depth clear of its rounding — the pose's and the distance's few roundings
    // of the point's place, read at the tool's size — is below nought: its neighbours cover the
    // point. Nearer, `d` is nought to its rounding either way)
    // (or, where the point stands off its sheet — a fitted sheet's point, `off` its fit — that:
    // about its own instant `d` is then nought to it)
    let rounding = (16.*f64::EPSILON*(crate::util::space::norm(x)+ring.size)).max(off);
    let held = |s: &[(f64,f64)]| s.iter().find(|p| p.1.abs() > rounding).is_some_and(|p| p.1 < 0.);
    if held(&lower) || held(&upper) { return Ok(Reading::Near) }
    // each side's own contact's end: the first sample past which `d` falls
    let past = |s: &[(f64,f64)]| -> (usize,Option<f64>) {
        // (a fall within the rounding is no fall: about its own instant `d` is nought to it)
        let Some(k) = (1..s.len()).find(|&k| s[k-1].1 > rounding && s[k].1 < s[k-1].1-rounding) else { return (s.len(),None) };
        let lo = if k >= 2 { s[k-2].0 } else { own };
        let (l,h) = if lo < s[k].0 { (lo,s[k].0) } else { (s[k].0,lo) };
        (k,Some(least(&|t| -d(t),l,h)))
    };
    let ((kl,el),(ku,eu)) = (past(&lower),past(&upper));
    // the least sample past each end, settled between its neighbours and within the far stretch
    let mut best: Option<(f64,f64)> = None;
    for (s,k,e) in [(&lower,kl,el),(&upper,ku,eu)] {
        let Some(e) = e else { continue };
        let Some(i) = (k..s.len()).min_by(|&i,&j| s[i].1.total_cmp(&s[j].1)) else { continue };
        let ends = |j: usize| if j < k { e } else { s[j].0 };
        let (p,q) = (ends(i.saturating_sub(1).max(k-1)),if i+1 < s.len() { s[i+1].0 } else { s[i].0 });
        let (l,h) = if p < q { (p,q) } else { (q,p) };
        let t = if h > l { least(&d,l,h) } else { s[i].0 };
        let v = d(t).min(s[i].1);
        let t = if d(t) <= s[i].1 { t } else { s[i].0 };
        if best.is_none_or(|b| v < b.1) { best = Some((t,v)) }
    }
    let Some((t,_)) = best else { return Ok(Reading::Alone) };
    let Some((depth,point,piece)) = depth_at(ring,path,x,t) else { return Err(Unbuilt::Failed("a pose not read".into())) };
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
/// given: so the corners are found where the end pose's signed distance changes sign along the
/// curves `given` lays on a face of either pose (`corners`), the meeting traced as the zero set of
/// that distance over the start pose's face (`zero_set::trace`) from them, cut at them, and each
/// piece between two corners given where both tools are uncovered at its middle (`cover`, the end
/// pose's point standing off its sheet by the trace's tolerance). Nothing where the poses do not
/// meet uncovered.
pub fn ends_met(ring: &Ring,path: &Trajectory,[a,b]: [f64;2],ends: [&crate::topo::Brep;2],given: &[crate::boolean::Given],tol: f64)
    -> Result<Vec<crate::boolean::Given>,Unbuilt> {
    use crate::util::space::distance;
    let poses = [path.pose(a)?,path.pose(b)?];
    // (the end pose's signed distance, read off the ring exactly)
    let g = |x: V| ring.signed(poses[1].inverse().point(x)).0;
    // (whether the point `x` of the pose at the roll's end `e`, standing off its sheet by as much as
    // `off`, is uncovered: on the boundary as the tolerance reads it)
    let uncovered = |x: V,e: usize,off: f64| match cover(ring,path,[a,b],x,[a,b][e],off,tol)? {
        Reading::Near => Ok::<bool,Unbuilt>(false),
        Reading::Alone => Ok(true),
        Reading::Far(c) => Ok(c.depth >= -tol),
    };
    let mut out = Vec::new();
    for fs in &ends[0].faces { for fe in &ends[1].faces {
        // the corners: where the other pose's signed distance changes sign along a seam of either
        // face (a seam of the start pose's against the end pose, of the end pose's against the start)
        let mut corners: Vec<V> = Vec::new();
        for gv in given {
            let Some(e) = [&fs.surface,&fe.surface].iter().position(|s| gv.surfaces.iter().any(|t| t == *s)) else { continue };
            let other = |x: V| ring.signed(poses[1-e].inverse().point(x)).0;
            for c in &gv.curves { for x in corners_along(c,&other,tol) {
                // (two corners within twice the tolerance are one place)
                if corners.iter().any(|&y| distance(x,y) <= 2.*tol) { continue }
                // (a corner of an arc on the boundary is on it: neither pose covered there by more than
                // the tolerance, nor held by its neighbours)
                if !(uncovered(x,0,0.)? && uncovered(x,1,tol)?) { continue }
                corners.push(x);
            } }
        }
        if corners.len() < 2 { continue }
        let s = &fs.surface;
        let at: Vec<crate::geom::Uv> = corners.iter().map(|&x| s.inverse(x)).collect();
        // the meeting over a window of the start face's parameters about each corner, grown from the
        // nearest other corner's distance till the piece through it reaches another corner both ways
        // or the window is the face's whole square
        let whole = face_rect(ends[0],fs);
        let field = Met {s,g:&g};
        let mut arcs: Vec<Vec<V>> = Vec::new();
        for (i,c) in at.iter().enumerate() {
            let near = at.iter().enumerate().filter(|&(j,_)| j != i).map(|(_,p)| (p[0]-c[0]).abs().max((p[1]-c[1]).abs()))
                .fold(f64::INFINITY,f64::min);
            // (no narrower than half the digits of the parameters read: the rounding's)
            let mut grow = near.max(HALF_READ);
            loop {
                let rect: crate::zero_set::Rect = std::array::from_fn(|k| [(c[k]-grow).max(whole[k][0]),(c[k]+grow).min(whole[k][1])]);
                let traced = crate::zero_set::trace(&field,&crate::zero_set::Question {rect,edges:&[],seeds:&[*c],tol,feature:ring.thickness})
                    .map_err(|e| Unbuilt::Failed(format!("the tool's meeting with itself at the roll's ends not traced: {e}")))?;
                let (found,open) = between_corners(&traced.pieces,s,&corners,corners[i],tol);
                let open = open.iter().map(|xs| Ok::<bool,Unbuilt>(uncovered(xs[xs.len()/2],0,0.)? && uncovered(xs[xs.len()/2],1,tol)?))
                    .collect::<Result<Vec<bool>,_>>()?.into_iter().any(|o| o);
                if !open || rect == whole {
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
            let mid = xs[xs.len()/2];
            if !(uncovered(mid,0,0.)? && uncovered(mid,1,tol)?) { continue }
            if crate::timing() { eprintln!("ring: the end poses meet uncovered from {:?} to {:?}, {} points",xs[0],xs[xs.len()-1],xs.len()) }
            curves.push(super::rings::curve_through(xs,tol)?.0);
        }
        if !curves.is_empty() { out.push(crate::boolean::Given {surfaces:[fs.surface.clone(),fe.surface.clone()],curves,touches:Vec::new()}) }
    } }
    Ok(out)
}

/// The square of a face's parameters its loops span.
fn face_rect(b: &crate::topo::Brep,f: &crate::topo::Face) -> crate::zero_set::Rect {
    let uv: Vec<crate::geom::Uv> = f.loops.iter().flatten().flat_map(|c| b.uv_ends(f,c)).collect();
    std::array::from_fn(|k| [uv.iter().map(|p| p[k]).fold(f64::INFINITY,f64::min),uv.iter().map(|p| p[k]).fold(f64::NEG_INFINITY,f64::max)])
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

/// The end pose's signed distance over the start pose's face, as the zero-set tracer reads it: its
/// gradient by central differences at the rounding's third root of the place (where their rounding
/// and their truncation balance), and bounded over a cell as a 1-Lipschitz function in space over
/// the cell's reach from its middle (the partials' lengths there, half again for their change over
/// the cell — a face of the tool, so curved no more than its own feature).
struct Met<'a> { s: &'a crate::geom::Surface,g: &'a dyn Fn(V) -> f64 }

impl crate::zero_set::Field for Met<'_> {
    type At = ();
    fn least(&self,uv: crate::geom::Uv) -> Option<crate::zero_set::Value<()>> {
        use crate::util::space::{add,scale,sub,norm};
        let (x,su,sv) = self.s.d1(uv);
        let h = f64::EPSILON.cbrt()*(1.+norm(x));
        let d = |e: V| { let n = norm(e).max(f64::MIN_POSITIVE); ((self.g)(add(x,scale(e,h/n)))-(self.g)(sub(x,scale(e,h/n))))/(2.*h)*n };
        Some(crate::zero_set::Value {f:(self.g)(x),grad:Some([d(su),d(sv)]),at:()})
    }
    fn along(&self,uv: crate::geom::Uv,_: &()) -> Option<crate::zero_set::Value<()>> { self.least(uv) }
    fn surface(&self,uv: crate::geom::Uv) -> (V,V,V) { self.s.d1(uv) }
    fn bound(&self,c: crate::zero_set::Rect) -> Option<crate::zero_set::Bound> {
        let m = [0.5*(c[0][0]+c[0][1]),0.5*(c[1][0]+c[1][1])];
        let (x,su,sv) = self.s.d1(m);
        let n = crate::util::space::norm;
        let reach = 0.75*(n(su)*(c[0][1]-c[0][0])+n(sv)*(c[1][1]-c[1][0]));
        Some(crate::zero_set::Bound::lipschitz((self.g)(x),1.,reach))
    }
}

/// The traced pieces through the corner `from` cut at the corners (each corner placed at its nearest
/// point of the piece it lies on, within twice the tolerance): each stretch between two corners, its
/// points in space and the corners its ends; and each stretch from `from` to the window's rim
/// (the window grown where one is on the boundary).
fn between_corners(pieces: &[crate::zero_set::Piece<()>],s: &crate::geom::Surface,corners: &[V],from: V,tol: f64) -> (Vec<Vec<V>>,Vec<Vec<V>>) {
    use crate::util::space::distance;
    let mut arcs = Vec::new();
    let mut open = Vec::new();
    for p in pieces {
        let xs: Vec<V> = p.uv.iter().map(|&u| s.point(u)).collect();
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
        if k == 0 && p.ends[0] == crate::zero_set::End::Rim { let c = cuts[0]; open.push([&[c.2],&xs[..=c.0]].concat()) }
        if k+1 == cuts.len() && p.ends[1] == crate::zero_set::End::Rim { let c = cuts[k]; open.push([&[c.2],&xs[c.0+1..]].concat()) }
        // (a stretch between two corners on the chords `i` and `j`: the corners and the points between)
        let stretch = |i: usize,ci: V,j: usize,cj: V| -> Vec<V> {
            let mut arc = vec![ci];
            if j > i { arc.extend(xs[i+1..=j].iter().copied()) } else if j < i { arc.extend(xs[i+1..xs.len()-1].iter().chain(&xs[..=j]).copied()) }
            arc.push(cj);
            arc
        };
        for w in cuts.windows(2) { arcs.push(stretch(w[0].0,w[0].2,w[1].0,w[1].2)) }
        // (a closed piece: the stretch from its last corner round to its first)
        if p.closed() && cuts.len() >= 2 {
            let (l,f) = (cuts[cuts.len()-1],cuts[0]);
            arcs.push(stretch(l.0,l.2,f.0,f.2));
        }
    }
    // (a stretch of no length — every point within twice the tolerance of its first — bounds nothing:
    // two corners that are one place, #942 F6)
    arcs.retain(|a| a.iter().any(|&x| distance(x,a[0]) > 2.*tol));
    (arcs,open)
}
