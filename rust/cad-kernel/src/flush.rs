//! **An end flush with a face is on it** (#836): an operand of a body's Boolean whose planar face
//! stands parallel to a plane of the solid and nearer it than `FLUSH` of the Boolean's tolerances
//! — a cut ending all but flush with another cut's floor, a boss all but level with the top — has
//! that face moved onto the plane before the two are combined, so the Boolean reads them as one
//! plane (a coincidence, `ssi::same`) and leaves no sliver between them: no strip of wall a few
//! nanometres tall, no face no export can write. The move is exact: the face's plane, its edges and
//! its corners carried along its normal, and the walls it ends — every face beside it a surface
//! the move leaves where it was (a plane, a cylinder or an extrusion along that normal), every
//! other edge reaching it a line along the normal — lengthened or shortened, their uses read anew.
//! A face whose walls are anything else (a cone, a sweep) is left where it is, and a sliver it
//! makes is said by the export (`brep::export::slivers` in the solver). Ends farther apart than
//! the reach are as drawn, to the Boolean's own tolerance.
use super::geom::{widen,Box3,Curve,Rigid,Surface,EMPTY,V};
use super::topo::{Brep,EdgeCurve,Face,Pcurve};
use crate::util::space::{add,cross,dot,norm,scale,sub};
use std::collections::BTreeSet;

/// How many of a Boolean's tolerances (`geom::tolerance_over`, a part in 10⁹ of the solids' size)
/// a planar face of an operand may stand off a parallel plane of the solid and be moved onto it: a
/// part in a million of their size, far under anything a person draws apart on purpose and far
/// over what arithmetic leaves of two ends written the same.
pub const FLUSH: f64 = 1e3;

/// How far a surface's axis may lean off the normal a face moves along and still be read as along
/// it, as a sine: over a reach of a part in a million it moves the surface a part in 10¹⁵.
const ALONG: f64 = 1e-9;

/// `flushed`, at the tolerance a body's Boolean of the two is made to (`boolean::combined_known`,
/// never under `floor`).
pub fn onto(solid: &Brep,operand: &Brep,floor: f64) -> Option<Brep> {
    flushed(solid,operand,super::geom::tolerance_over(solid.size().max(operand.size()),floor))
}

/// `operand` with each of its planar faces standing within `FLUSH tol` of a parallel plane of a
/// face of `solid` near it moved onto that plane (the nearest, where two are near), where its walls
/// let it move; `None` where none moves. A face already on a plane of the solid, within `tol`, stays.
pub fn flushed(solid: &Brep,operand: &Brep,tol: f64) -> Option<Brep> {
    let reach = FLUSH*tol;
    // the solid's planes, each with the box of its face
    let planes: Vec<(super::geom::Frame,Box3)> = solid.faces.iter().filter(|f| !f.loops.is_empty())
        .filter_map(|f| match f.surface { Surface::Plane(fr) => Some((fr,face_box(solid,f))),_ => None }).collect();
    if planes.is_empty() { return None }
    let mut out: Option<Brep> = None;
    let mut seen = vec![false;operand.faces.len()];
    for fi in 0..operand.faces.len() {
        if seen[fi] { continue }
        let b = out.as_ref().unwrap_or(operand);
        let Surface::Plane(f) = b.faces[fi].surface else { continue };
        // every face of the operand on this plane moves with it
        let group: Vec<usize> = (0..b.faces.len()).filter(|&j| !b.faces[j].loops.is_empty()
            && matches!(b.faces[j].surface,Surface::Plane(_)) && super::ssi::same(&b.faces[fi].surface,&b.faces[j].surface,tol)).collect();
        for &j in &group { seen[j] = true; }
        let mut bx = EMPTY;
        for &j in &group { widen(&mut bx,face_box(b,&b.faces[j])); }
        // the nearest parallel plane of a face of the solid whose box comes within reach of this one
        let mut nearest: Option<f64> = None;
        let mut on = false;
        for (g,gb) in &planes {
            if norm(cross(g.z,f.z)) > 1e-12 || !near(&bx,gb,reach) { continue }
            let d = dot(sub(g.o,f.o),f.z);
            if d.abs() <= tol { on = true; break }
            if d.abs() <= reach && nearest.is_none_or(|n| d.abs() < n.abs()) { nearest = Some(d); }
        }
        let (false,Some(d)) = (on,nearest) else { continue };
        if let Some(moved) = moved_along(b,&group,scale(f.z,d),tol) { out = Some(moved); }
    }
    out
}

/// Whether two boxes come within `reach` of each other.
fn near(a: &Box3,b: &Box3,reach: f64) -> bool {
    (0..3).all(|k| a.0[k] <= b.1[k]+reach && b.0[k] <= a.1[k]+reach)
}

/// A face's box: its edges'.
fn face_box(b: &Brep,f: &Face) -> Box3 {
    let mut bx = EMPTY;
    for c in f.loops.iter().flatten() {
        let e = &b.edges[c.edge as usize];
        match &e.curve {
            EdgeCurve::Curve(curve) => widen(&mut bx,curve.bounds(e.t)),
            EdgeCurve::Degenerate => { let p = b.vertices[e.v[0] as usize].p; widen(&mut bx,(p,p)); }
        }
    }
    bx
}

/// `b` with its faces `group` (all on one plane) carried by `delta` along the plane's normal: the
/// plane, the group's edges and their vertices moved, and every other edge ending at one of those
/// vertices — a line along `delta` — lengthened or shortened to it; the faces beside the group,
/// each a surface the move leaves where it was, have their uses of those edges read anew. `None`
/// where a face beside is another surface, an edge reaching the group is no such line or reaches it
/// at both ends, or a line would be left shorter than the tolerance allows.
fn moved_along(b: &Brep,group: &[usize],delta: V,tol: f64) -> Option<Brep> {
    let n = scale(delta,1./norm(delta));
    let grouped: BTreeSet<usize> = group.iter().copied().collect();
    let own: BTreeSet<u32> = group.iter().flat_map(|&fi| b.faces[fi].loops.iter().flatten().map(|c| c.edge)).collect();
    let corners: BTreeSet<u32> = own.iter().flat_map(|&e| b.edges[e as usize].v).collect();
    // the edges reaching the group from elsewhere, and which of their ends does
    let mut stretched: Vec<(u32,usize)> = Vec::new();
    for (ei,e) in b.edges.iter().enumerate() {
        if own.contains(&(ei as u32)) { continue }
        let k = match e.v.map(|v| corners.contains(&v)) {
            [false,false] => continue,
            [true,true] => return None,
            [at_start,_] => usize::from(!at_start),
        };
        match &e.curve {
            EdgeCurve::Curve(Curve::Line {d,..}) if norm(cross(*d,n)) <= ALONG => stretched.push((ei as u32,k)),
            _ => return None,
        }
    }
    if own.iter().any(|&e| matches!(b.edges[e as usize].curve,EdgeCurve::Degenerate)) { return None }
    let reached: BTreeSet<u32> = own.iter().copied().chain(stretched.iter().map(|s| s.0)).collect();
    // each face beside the group: how its parameters move with a point moved by `delta`, where the
    // move leaves its surface where it was
    let mut beside: Vec<(usize,[f64;2])> = Vec::new();
    for (fi,f) in b.faces.iter().enumerate() {
        if grouped.contains(&fi) || !f.loops.iter().flatten().any(|c| reached.contains(&c.edge)) { continue }
        let shift = match f.surface {
            Surface::Plane(g) if dot(g.z,n).abs() <= ALONG => [dot(delta,g.x),dot(delta,g.y)],
            Surface::Cylinder(g,_) | Surface::Extrusion(g,_) if norm(cross(g.z,n)) <= ALONG => [0.,dot(delta,g.z)],
            _ => return None,
        };
        beside.push((fi,shift));
    }
    let m = Rigid {t:delta,..Rigid::identity()};
    let mut out = b.clone();
    for &fi in group { out.faces[fi].surface = b.faces[fi].surface.moved(&m); }
    for &e in &own {
        if let EdgeCurve::Curve(c) = &b.edges[e as usize].curve { out.edges[e as usize].curve = EdgeCurve::Curve(c.moved(&m)); }
    }
    for &v in &corners { out.vertices[v as usize].p = add(b.vertices[v as usize].p,delta); }
    for &(e,k) in &stretched {
        let edge = &mut out.edges[e as usize];
        let EdgeCurve::Curve(Curve::Line {p,d}) = edge.curve else { unreachable!("a stretched edge is a line") };
        edge.t[k] = dot(sub(out.vertices[edge.v[k] as usize].p,p),d);
        if !(edge.t[1]-edge.t[0] > super::tolerant::ON_CURVE*tol) { return None }
    }
    // the uses beside: a moved edge's pcurve moved whole, a stretched one's at the end that moved —
    // a pcurve of its own read at its old ends first, since the edge's parameters are not what they
    // were (a line along a plane, a cylinder or an extrusion along the move is straight in its
    // parameters, and a moved curve is the same curve shifted)
    for (fi,shift) in beside {
        let surface = b.faces[fi].surface.clone();
        let moved = |q: [f64;2]| [q[0]+shift[0],q[1]+shift[1]];
        for c in out.faces[fi].loops.iter_mut().flatten() {
            let old = &b.edges[c.edge as usize];
            let ends = [0,1].map(|k| c.pcurve.at(old.t[k],old,&surface,&b.vertices));
            let move_end = if own.contains(&c.edge) { [true,true] }
                else if let Some(&(_,k)) = stretched.iter().find(|s| s.0 == c.edge) { [k == 0,k == 1] }
                else { continue };
            let [a,z] = [0,1].map(|k| if move_end[k] { moved(ends[k]) } else { ends[k] });
            c.pcurve = match c.pcurve {
                Pcurve::Line {..} => Pcurve::Line {a,b:z},
                Pcurve::Inverse {..} => Pcurve::Inverse {a,b:z},
                Pcurve::Curve(_) if own.contains(&c.edge) => Pcurve::Inverse {a,b:z},
                Pcurve::Curve(_) => Pcurve::Line {a,b:z},
            };
        }
    }
    Some(out)
}

/// What of a B-rep is thinner than a bar (`slivers`).
#[derive(Clone,Copy,Debug,PartialEq)]
pub enum Thin {
    /// A face thinner than the bar somewhere: two points of its boundary nearer each other than the
    /// bar, on edges that share no vertex, where the boundary between them runs far (`thinnest`) —
    /// a strip's two long sides, a wall's top within the bar of a floor it stands on, a hole within
    /// it of the rim.
    Face(usize),
    /// An edge shorter than the bar, of no face found so.
    Edge(usize),
}

/// **A sliver a B-rep holds** (#836): a face thinner than a bar somewhere or an edge shorter than
/// it, how thin or how short (mm), a point of it (midway across the face where it is thinnest, or
/// the edge's middle), the faces it is of — a face its own, an edge the faces using it — and the
/// faces beside it there: across the two edges a face is thinnest between, beside an edge's ends.
#[derive(Clone,Debug)]
pub struct Sliver { pub thin: Thin,pub size: f64,pub at: V,pub of: Vec<usize>,pub beside: Vec<usize> }

/// How far along its boundary a face's two near points must stand apart, in bars, for the face to
/// be thin between them rather than across an edge shorter than the bar (which is said as such).
const ROUND_BARS: f64 = 4.;

/// How many times their distance apart two near points must stand along the boundary between
/// them.
const ROUND_RATIO: f64 = 16.;

/// **The slivers of `b` under `bar`** (mm), the thinnest first: each face thinner than it
/// somewhere (`thinnest`), then each edge shorter than it that no such face uses — what one cut
/// ending all but flush with another's face leaves (`flushed` moves such an end onto the face
/// where it can), and what the B-rep holds wherever it stands, whether or not a writer's arithmetic
/// can tell its sides apart. A pole (a degenerate edge) is no sliver.
pub fn slivers(b: &Brep,bar: f64) -> Vec<Sliver> {
    let lines: Vec<Vec<V>> = b.edges.iter().map(|e| polyline(e,&b.vertices)).collect();
    let length = |e: usize| lines[e].windows(2).map(|w| norm(sub(w[1],w[0]))).sum::<f64>();
    let mut users: Vec<Vec<usize>> = vec![Vec::new();b.edges.len()];
    for (fi,f) in b.faces.iter().enumerate() {
        for c in f.loops.iter().flatten() {
            if !users[c.edge as usize].contains(&fi) { users[c.edge as usize].push(fi); }
        }
    }
    // the faces sharing an edge with any of `of`, but those
    let beside = |of: &[usize]| -> Vec<usize> {
        let mut out: Vec<usize> = of.iter().flat_map(|&fi| b.faces[fi].loops.iter().flatten())
            .flat_map(|c| users[c.edge as usize].iter().copied()).filter(|fi| !of.contains(fi)).collect();
        out.sort();
        out.dedup();
        out
    };
    let thin: Vec<Option<(f64,V,[u32;2])>> = crate::util::par::indices(b.faces.len(),|fi| thinnest(b,&b.faces[fi],&lines,bar));
    let mut out: Vec<Sliver> = Vec::new();
    let mut thin_faces = BTreeSet::new();
    for (fi,x) in thin.into_iter().enumerate() {
        let Some((across,at,edges)) = x else { continue };
        thin_faces.insert(fi);
        // beside it where it is thin: the faces across the two edges it is thin between
        let mut by: Vec<usize> = edges.iter().flat_map(|&e| users[e as usize].iter().copied()).filter(|&g| g != fi).collect();
        by.sort();
        by.dedup();
        out.push(Sliver {thin:Thin::Face(fi),size:across,at,of:vec![fi],beside:by});
    }
    let faces = out.len();
    for (ei,e) in b.edges.iter().enumerate() {
        if matches!(e.curve,EdgeCurve::Degenerate) || users[ei].is_empty() || users[ei].iter().any(|fi| thin_faces.contains(fi)) { continue }
        let long = length(ei);
        if long >= bar { continue }
        let at = e.point(0.5*(e.t[0]+e.t[1]),&b.vertices);
        out.push(Sliver {thin:Thin::Edge(ei),size:long,at,of:users[ei].clone(),beside:beside(&users[ei])});
    }
    out[..faces].sort_by(|p,q| p.size.total_cmp(&q.size));
    out[faces..].sort_by(|p,q| p.size.total_cmp(&q.size));
    out
}

/// **Where face `f` is thinnest, under `bar`**: how far apart, the point midway and the edges they
/// are on, two points of its boundary nearer each other than `bar` — on two edges that share no
/// vertex, so not two sides of one corner (however sharp: a round's run-out meets its fill's cap
/// tangentially, and a seam cut in pieces passes its vertices twice), and on two loops (a hole
/// within the bar of the rim) or on one where the boundary between them, either way round, runs
/// more than `ROUND_BARS` bars and `ROUND_RATIO` times their distance; of pairs as near, the one
/// farthest apart along the boundary (a strip's middle). Each edge is read as its
/// polyline (`lines`), a point of one against the chords of another. `None` where it is nowhere so
/// thin.
fn thinnest(b: &Brep,f: &Face,lines: &[Vec<V>],bar: f64) -> Option<(f64,V,[u32;2])> {
    /// A use of an edge along its loop: its points in the loop's order, how far along the loop each
    /// stands, and the box of each run of `CHUNK` chords.
    struct Run { edge: u32,ends: [u32;2],pts: Vec<V>,at: Vec<f64>,chunks: Vec<Box3> }
    const CHUNK: usize = 16;
    let mut loops: Vec<(Vec<Run>,f64)> = Vec::new();
    for l in &f.loops {
        let mut runs = Vec::new();
        let mut s = 0.;
        for c in l {
            if matches!(b.edges[c.edge as usize].curve,EdgeCurve::Degenerate) { continue }
            let mut pts = lines[c.edge as usize].clone();
            if c.reversed { pts.reverse(); }
            let mut at = vec![s];
            for w in pts.windows(2) { s += norm(sub(w[1],w[0])); at.push(s); }
            let chunks = (0..pts.len().saturating_sub(1)).step_by(CHUNK).map(|k| {
                let mut bx = EMPTY;
                for &p in &pts[k..(k+CHUNK+1).min(pts.len())] { widen(&mut bx,(p,p)); }
                bx
            }).collect();
            runs.push(Run {edge:c.edge,ends:b.edges[c.edge as usize].v,pts,at,chunks});
        }
        loops.push((runs,s));
    }
    // the thinnest, and of those as thin, the farthest apart along the boundary: the middle of a
    // strip, not its end
    let mut best: Option<(f64,f64,V,[u32;2])> = None;
    let thinner = |d: f64,apart: f64,best: &Option<(f64,f64,V,[u32;2])>| best.is_none_or(|(x,far,..)|
        d < x*(1.-1e-6) || (d <= x*(1.+1e-6) && apart > far));
    for (la,(runs_a,_)) in loops.iter().enumerate() {
        for a in runs_a {
            for (lb,(runs_b,perimeter)) in loops.iter().enumerate() {
                for r in runs_b.iter().filter(|r| !r.ends.iter().any(|v| a.ends.contains(v))) {
                    for (i,&p) in a.pts.iter().enumerate() {
                        for (k,bx) in r.chunks.iter().enumerate() {
                            if !near(&(p,p),bx,bar) { continue }
                            for j in k*CHUNK..((k+1)*CHUNK).min(r.pts.len()-1) {
                                let (d,u) = to_chord(p,r.pts[j],r.pts[j+1]);
                                if !(d < bar) || best.is_some_and(|(x,..)| x*(1.+1e-6) < d) { continue }
                                let apart = if la != lb { f64::INFINITY } else {
                                    let x = (a.at[i]-(r.at[j]+u*(r.at[j+1]-r.at[j]))).abs();
                                    x.min(perimeter-x)
                                };
                                if apart > (ROUND_BARS*bar).max(ROUND_RATIO*d) && thinner(d,apart,&best) {
                                    let q = add(r.pts[j],scale(sub(r.pts[j+1],r.pts[j]),u));
                                    best = Some((d,apart,scale(add(p,q),0.5),[a.edge,r.edge]));
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    best.map(|(d,_,at,edges)| (d,at,edges))
}

/// How far `p` stands from the chord from `a` to `b`, and how far along it its foot is (0 to 1).
fn to_chord(p: V,a: V,b: V) -> (f64,f64) {
    let d = sub(b,a);
    let l = dot(d,d);
    let u = if l > 0. { (dot(sub(p,a),d)/l).clamp(0.,1.) } else { 0. };
    (norm(sub(p,add(a,scale(d,u)))),u)
}

/// An edge as a polyline, its chords between its curve's breaks, each stretch cut in sixteen (one
/// chord a stretch where the curve has many: a traced one's points); a pole, its one point.
fn polyline(e: &super::topo::Edge,vertices: &[super::topo::Vertex]) -> Vec<V> {
    let EdgeCurve::Curve(c) = &e.curve else { return vec![vertices[e.v[0] as usize].p] };
    let mut at = vec![e.t[0]];
    at.extend(c.breaks(e.t).into_iter().filter(|&t| t > e.t[0] && t < e.t[1]));
    at.push(e.t[1]);
    let cuts = if at.len() > 64 { 1 } else { 16 };
    let mut pts = vec![e.point(e.t[0],vertices)];
    for w in at.windows(2) {
        for k in 1..=cuts { pts.push(e.point(w[0]+(w[1]-w[0])*k as f64/cuts as f64,vertices)); }
    }
    pts
}
