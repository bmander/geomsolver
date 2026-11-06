//! **A sliver a B-rep holds** (#836): a face thinner than a bar somewhere, or an edge shorter than
//! it, found by the boundary — what one cut ending all but flush with another's face leaves where
//! the Booleans could not make the two one (`flush`), and what an export says beside its file.
use super::boolean::overlap;
use super::geom::{hull,Box3,V};
use super::topo::{Brep,EdgeCurve,Face};
use crate::util::space::{add,dot,norm,polyline_length,scale,sub};

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
/// ending all but flush with another's face leaves (`flush::flushed` moves such an end onto the
/// face where it can), and what the B-rep holds wherever it stands, whether or not a writer's
/// arithmetic can tell its sides apart. A pole (a degenerate edge) is no sliver.
pub fn slivers(b: &Brep,bar: f64) -> Vec<Sliver> {
    let lines: Vec<Vec<V>> = crate::util::par::indices(b.edges.len(),|e| polyline(&b.edges[e],&b.vertices));
    // the faces using each edge
    let users: Vec<Vec<usize>> = b.uses().into_iter().map(|u| { let mut f: Vec<usize> = u.into_iter().map(|x| x.0).collect(); f.dedup(); f }).collect();
    // the faces using any of `edges`, but `but`
    let around = |edges: &mut dyn Iterator<Item=usize>,but: &[usize]| -> Vec<usize> {
        let mut out: Vec<usize> = edges.flat_map(|e| users[e].iter().copied()).filter(|f| !but.contains(f)).collect();
        out.sort();
        out.dedup();
        out
    };
    let thin: Vec<Option<(f64,V,[u32;2])>> = crate::util::par::indices(b.faces.len(),|fi| thinnest(b,&b.faces[fi],&lines,bar));
    let mut out: Vec<Sliver> = Vec::new();
    for (fi,x) in thin.iter().enumerate() {
        let &Some((across,at,edges)) = x else { continue };
        // beside it where it is thin: the faces across the two edges it is thin between
        out.push(Sliver {thin:Thin::Face(fi),size:across,at,of:vec![fi],beside:around(&mut edges.iter().map(|&e| e as usize),&[fi])});
    }
    for (ei,e) in b.edges.iter().enumerate() {
        if matches!(e.curve,EdgeCurve::Degenerate) || users[ei].is_empty() || users[ei].iter().any(|&fi| thin[fi].is_some()) { continue }
        let long = polyline_length(&lines[ei]);
        if long >= bar { continue }
        let at = e.point(0.5*(e.t[0]+e.t[1]),&b.vertices);
        let edges: Vec<usize> = users[ei].iter().flat_map(|&fi| b.faces[fi].loops.iter().flatten().map(|c| c.edge as usize)).collect();
        out.push(Sliver {thin:Thin::Edge(ei),size:long,at,of:users[ei].clone(),beside:around(&mut edges.into_iter(),&users[ei])});
    }
    // faces first, each kind the thinnest first
    out.sort_by(|p,q| matches!(p.thin,Thin::Edge(_)).cmp(&matches!(q.thin,Thin::Edge(_))).then(p.size.total_cmp(&q.size)));
    out
}

/// **Where face `f` is thinnest, under `bar`**: how far apart, the point midway and the edges they
/// are on, two points of its boundary nearer each other than `bar` — on two edges that share no
/// vertex, so not two sides of one corner (however sharp: a round's run-out meets its fill's cap
/// tangentially, and a seam cut in pieces passes its vertices twice), and on two loops (a hole
/// within the bar of the rim) or on one where the boundary between them, either way round, runs
/// more than `ROUND_BARS` bars and `ROUND_RATIO` times their distance; of pairs as near, the one
/// farthest apart along the boundary (a strip's middle). Each edge is read as its polyline
/// (`lines`), a point of one against the chords of another, two uses' boxes first and then each
/// run of `CHUNK` chords'. `None` where it is nowhere so thin.
fn thinnest(b: &Brep,f: &Face,lines: &[Vec<V>],bar: f64) -> Option<(f64,V,[u32;2])> {
    /// A use of an edge along its loop: its loop, its edge and that edge's ends, its points in the
    /// loop's order, how far along the loop each stands, its box and the box of each run of
    /// `CHUNK` chords.
    struct Run { lp: usize,edge: u32,ends: [u32;2],pts: Vec<V>,at: Vec<f64>,bx: Box3,chunks: Vec<Box3> }
    const CHUNK: usize = 16;
    let mut runs: Vec<Run> = Vec::new();
    let mut perimeters: Vec<f64> = Vec::new();
    for (lp,l) in f.loops.iter().enumerate() {
        let mut s = 0.;
        for c in l {
            let e = &b.edges[c.edge as usize];
            if matches!(e.curve,EdgeCurve::Degenerate) { continue }
            let mut pts = lines[c.edge as usize].clone();
            if c.reversed { pts.reverse(); }
            let mut at = vec![s];
            for w in pts.windows(2) { s += norm(sub(w[1],w[0])); at.push(s); }
            let chunks = (0..pts.len()-1).step_by(CHUNK).map(|k| hull(pts[k..(k+CHUNK+1).min(pts.len())].iter().copied())).collect();
            runs.push(Run {lp,edge:c.edge,ends:e.v,bx:hull(pts.iter().copied()),pts,at,chunks});
        }
        perimeters.push(s);
    }
    // the thinnest, and of those as thin, the farthest apart along the boundary: the middle of a
    // strip, not its end
    let mut best: Option<(f64,f64,V,[u32;2])> = None;
    for a in &runs {
        for r in runs.iter().filter(|r| !r.ends.iter().any(|v| a.ends.contains(v)) && overlap(&a.bx,&r.bx,bar)) {
            for (i,&p) in a.pts.iter().enumerate() {
                for (k,bx) in r.chunks.iter().enumerate() {
                    if !overlap(&(p,p),bx,bar) { continue }
                    for j in k*CHUNK..((k+1)*CHUNK).min(r.pts.len()-1) {
                        let (d,u) = to_chord(p,r.pts[j],r.pts[j+1]);
                        if !(d < bar) { continue }
                        let apart = if a.lp != r.lp { f64::INFINITY } else {
                            let x = (a.at[i]-(r.at[j]+u*(r.at[j+1]-r.at[j]))).abs();
                            x.min(perimeters[a.lp]-x)
                        };
                        let thinner = best.is_none_or(|(x,far,..)| d < x*(1.-1e-6) || (d <= x*(1.+1e-6) && apart > far));
                        if apart > (ROUND_BARS*bar).max(ROUND_RATIO*d) && thinner {
                            let q = add(r.pts[j],scale(sub(r.pts[j+1],r.pts[j]),u));
                            best = Some((d,apart,scale(add(p,q),0.5),[a.edge,r.edge]));
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
