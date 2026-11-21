//! **The contact's road under any motion** (#880 T1d): the candidates of the closed solid a tool
//! sweeps under a rigid motion no constant twist takes — a frame along a path, two turns at once, a
//! screw whose lead grows — read as the contact's road reads them under a constant twist
//! (`contacts::candidates`), and trimmed by the same membership (`trim::trim`).
//!
//! Under such a motion no contact stands still on the tool: at each instant `s` the motion is a
//! screw (Mozzi–Chasles), its twist read on the tool (`Twist::body`), and the tool touches its sweep
//! where its outward normal is square to that twist's velocity, `n · v_s(x) = 0`. **The contact is
//! read per tool** (`contact`): on a polyhedral tool `n · v` is affine over each face, so
//!
//! - **a face's chord**: the face's contact is the line of its plane where `n · v` vanishes, within
//!   the face — a chord from one of its edges to another, each end the root of the face's `n · v`
//!   along that edge;
//! - **a sharp edge's stretch** (L2's rule): the edge bounds the sweep where its two faces' `n · v`
//!   differ in sign — affine along it, so between their roots, or a vertex — each end where one
//!   face's chord reaches the edge, or a vertex.
//!
//! So each instant's contact is closed polygons, each chord running on into the stretch of the edge
//! it ends on and each stretch into the next edge's at a vertex. Its **shape** — which pieces, and
//! what each end is — changes only at instants (`events`) where a root passes a vertex or two meet,
//! found by halving the roll between two readings that differ. Between two such instants each piece
//! is a **wall whose ends move with the roll** (`family::Sheet`, a line from end to end): at each
//! station its ends are read where the motion has the tool, and between stations each number is the
//! quintic through six of them, piece by piece, held to a ten-billionth of the tool where the motion
//! is read between them (`walls`). Every wall of a stretch of the roll is read at the same stations,
//! so two pieces sharing an end share their rail exactly, and a piece reaching the next stretch
//! meets the wall it runs on into along a rim.
//!
//! The candidates are those walls and **the tool at each end of the roll**. What is known of their
//! meetings is given (`boolean::Given`): two walls sharing an end, that end's path, a rail of both;
//! a wall and the one it runs on into past an instant, its rim there; and each wall's whole meeting
//! with each face of the tool at either end (`contacts::pose_meeting`, read in its own parameters,
//! its rim there where it touches the face its section lies on: a chord touching its face, a stretch
//! crossing its two). A vertex enters nothing, and the trim keeps of each wall what bounds the sweep.
//!
//! **The road `closed` asks where K2 cannot read a tool** (`closed::general_built`): a tool no
//! convex solid of revolution, every face a plane and every edge a line, under a motion no constant
//! twist takes and keeping no plane. A bar under a screw whose lead grows builds, cut from its rod to
//! the bench's volume at 7.5e-11: its two vertical edges at one radius each sweep the one cylinder,
//! taken as that cylinder (`on_a_cylinder`, its faces' parameters its edges inverted onto it), so the
//! trim knows them as one surface; and its bottom edge's strip is cut where it leaves the bar posed at
//! the start (the branch from the rim's corner in `contacts::pose_meeting`, each end at its own
//! nearest corner). A cube tumbling two degrees builds too, and five: two edges' fans sharing a
//! vertex are given its path, and where they cross a second time beside it, that branch too
//! (`beside_rails`, traced from where an edge of either crosses the other). What the trim still
//! refuses:
//!
//! - **the cube turned farther** (brep-bench's `tumbling-cube`, 15° and 75°): an edge's fan passes
//!   back through itself, two of its strips crossing — cut into runs of stretches, each its own
//!   surface and candidate, their crossing given (`crossed`, #942 T1d-cube) — and two fans from one
//!   corner cross a second time beside the corner's path, a branch that runs into the path where the
//!   two come to touch and whose trace stops some 1e-3 short of it, its ends open, parting nothing.
//!
//! And a face that is no plane (its contact a curve that moves over it and changes as it goes), an
//! edge that is no line, and a contact running along an edge (a face's `n · v` nought along one of
//! its edges at an instant) are refused by name (`Unbuilt::Curved`).
use crate::family::{Run,Sheet,Stations,NODES};
use crate::geom::{Curve,Frame,Rigid,Surface};
use crate::topo::{Brep,Coedge,EdgeCurve,Face,Pcurve};
use crate::trajectory::{Trajectory,Twist};
use crate::util::space::{add,cross,distance,dot,scale,sub};
use super::closed::Unbuilt;
use std::sync::Arc;

type V = [f64;3];

/// A sheet's box of parameters: its faces' together (`Located::parameter_box`).
pub(super) fn parameters(b: &Brep,tol: f64) -> [[f64;2];2] {
    let located = crate::query::Located::new(b,tol);
    let mut out = [[f64::INFINITY,f64::NEG_INFINITY];2];
    for f in 0..b.faces.len() {
        let [lo,hi] = located.parameter_box(f);
        for k in 0..2 { out[k] = [out[k][0].min(lo[k]),out[k][1].max(hi[k])] }
    }
    out
}

/// **Where two walls sharing a rail cross elsewhere** (`candidates`): seeded where an edge of either
/// wall crosses the other's surface farther than twice the tolerance from every rail (a seed on a
/// rail and the rail each on the meeting within it, #942 F6), met
/// through both within their shared box by the kernel's one fallback (`meeting::through`: traced in
/// space, and where that is refused as shallow marched in both walls' parameters), each seed on its
/// own, and kept where some point of the curve stands that far off the rails. None where nothing
/// seeds, or every seed is refused.
fn beside_rails(a: &Brep,sa: &Surface,b: &Brep,sb: &Surface,rails: &[Curve],tol: f64) -> Vec<Curve> {
    let (la,ha) = a.bounds();
    let (lb,hb) = b.bounds();
    let (lo,hi): (V,V) = (std::array::from_fn(|k| la[k].max(lb[k])-tol),std::array::from_fn(|k| ha[k].min(hb[k])+tol));
    if (0..3).any(|k| lo[k] > hi[k]) { return Vec::new() }
    let off_rails = |p: V| rails.iter().all(|r| distance(r.point(r.inverse(p)),p) > 2.*tol);
    let mut seeds: Vec<V> = Vec::new();
    for (from,onto) in [(a,sb),(b,sa)] {
        for e in &from.edges {
            let EdgeCurve::Curve(c) = &e.curve else { continue };
            if let crate::query::Meets::At(roots) = crate::query::curve_surface(c,e.t,onto,tol) {
                seeds.extend(roots.iter().map(|&(t,_)| c.point(t)).filter(|&p| off_rails(p)));
            }
        }
    }
    if crate::debug() { eprintln!("general: beside rails from {:?}: {} seeds {:?}",rails.first().and_then(|r| crate::query::curve_domain(r).map(|d| r.point(d[0]))),seeds.len(),seeds) }
    if seeds.is_empty() { return Vec::new() }
    // (each seed met on its own, one already on a branch met skipped: where the march in space is
    // refused — two fans all but tangent, the cube tumbling 75°, meeting at 0.02° where the branch
    // reaches the pinch a third's rail starts at — the one fallback marches it in the two sheets'
    // own parameters, and a seed it refuses leaves the others' branches as they are)
    let (da,db) = (parameters(a,tol),parameters(b,tol));
    let mut traced: Vec<Curve> = Vec::new();
    for &p in &seeds {
        // (a seed on a branch met already: within the reach a point of two surfaces meeting at
        // their angle there has of their meeting, `tolerant::on_meeting` — the two fans of the cube
        // tumbling 75° meeting so shallowly that a second seed's trace of one branch ran 4e-7 off
        // the first, and the two laid side by side closed no face, #942 T1d-cube)
        let sine = crate::util::space::norm(cross(sa.gradient(p),sb.gradient(p)));
        let reach = crate::tolerant::on_meeting(tol,sine);
        let on = traced.iter().any(|c| distance(c.point(c.inverse(p)),p) <= reach);
        if on { continue }
        match crate::meeting::through(sa,da,sb,db,&[p],lo,hi,tol) {
            Ok(t) => traced.extend(t),
            Err(e) => if crate::debug() { eprintln!("general: beside rails, the meeting through {p:?} refused: {e}") },
        }
    }
    let kept: Vec<Curve> = traced.into_iter().filter(|c| {
        let Some([t0,t1]) = crate::query::curve_domain(c) else { return false };
        (0..=16).any(|k| off_rails(c.point(t0+(t1-t0)*k as f64/16.)))
    }).collect();
    if crate::debug() { for c in &kept { let d = crate::query::curve_domain(c).unwrap_or([0.,1.]); eprintln!("general: beside a rail, a {} from {:?} to {:?}",c.kind(),c.point(d[0]),c.point(d[1])) } }
    kept
}

/// **A branch beside a rail ends on it where the two walls touch** (#942 T1d-cube): two branches
/// of one meeting cross only where its two surfaces are tangent, so a branch beside the rail two
/// edges' fans share reaches the rail where they touch — and there the trace, its surfaces meeting
/// ever more shallowly, stops short or runs on along the rail (the cube tumbling 75°: its two
/// fans' branch ran 3.95e-6 past the corner they touch at, two thousand tolerances, at a sine of
/// 7e-7, and hung in the face as a slit). An end within its meeting's reach of a rail
/// (`tolerant::on_meeting`) is carried to the rail's point where the walls' normals turn past each
/// other — where `(nₐ × n_b) · r′` changes sign, within that reach along it — and the curve cut
/// there; an end with no such point is left as traced.
///
/// The two fans touch where the corner's velocity lies in the face holding both edges, an instant
/// the face turns and the contact changes, where the road cuts its stretches: so a corner of
/// either wall on the rail nearer that root than the end stood is the touch, as the road placed it
/// (the root, read through two numerical feet, 5e-9 off it, past the pool's tolerance). (Not for
/// one fan's stretches meeting each other: their walls are one sheet across the rim they share,
/// the sine along it nought throughout, and its sign there rounding's.)
fn ended_on_rails(mut c: Curve,a: &Brep,sa: &Surface,b: &Brep,sb: &Surface,rails: &[Curve],tol: f64) -> Curve {
    use crate::util::space::{norm,normalised};
    let unit = |s: &Surface,p: V| normalised(s.gradient(p)).unwrap_or([0.;3]);
    for high in [false,true] {
        let Some([t0,t1]) = crate::query::curve_domain(&c) else { return c };
        let p = c.point(if high { t1 } else { t0 });
        let reach = crate::tolerant::on_meeting(tol,norm(cross(unit(sa,p),unit(sb,p))));
        let touch = rails.iter().find_map(|r| {
            let [r0,r1] = crate::query::curve_domain(r)?;
            let f = r.inverse(p);
            if distance(r.point(f),p) > reach { return None }
            // the sine of the walls' angle, signed along the rail
            let signed = |s: f64| { let (q,d) = r.d1(s); dot(cross(unit(sa,q),unit(sb,q)),normalised(d).unwrap_or([0.;3])) };
            let w = reach/norm(r.d1(f).1).max(1e-300);
            let (lo,hi) = ((f-w).max(r0),(f+w).min(r1));
            const LOOKS: usize = 16;
            let at: Vec<(f64,f64)> = (0..=LOOKS).map(|k| { let s = lo+(hi-lo)*k as f64/LOOKS as f64; (s,signed(s)) }).collect();
            let (from,to) = at.windows(2).filter(|w| w[0].1*w[1].1 <= 0. && w[0].1 != w[1].1).map(|w| (w[0],w[1]))
                .min_by(|x,y| (0.5*(x.0.0+x.1.0)-f).abs().total_cmp(&(0.5*(y.0.0+y.1.0)-f).abs()))?;
            let (s,_) = crate::util::roots::bisect(from.0,to.0,|x,y| 0.5*(x+y),|x,y| y-x > f64::EPSILON*(x.abs()+y.abs()),
                |m| signed(m)*from.1 > 0.);
            let q = r.point(s);
            if distance(q,p) > reach { return None }
            let corner = a.vertices.iter().chain(&b.vertices).map(|v| v.p).filter(|&v| distance(r.point(r.inverse(v)),v) <= tol)
                .min_by(|x,y| distance(*x,q).total_cmp(&distance(*y,q))).filter(|&v| distance(v,q) < distance(p,q));
            Some(corner.unwrap_or(q))
        });
        let Some(q) = touch else { continue };
        let tc = c.inverse(q);
        // (cut only within the end's half: the branch's other end is never the touch's)
        let mid = 0.5*(t0+t1);
        let cut = if high { tc > mid && tc < t1 } else { tc < mid && tc > t0 };
        let ended = match &c {
            Curve::BSpline(b) => {
                let mut b = if cut { b.segment(if high { [t0,tc] } else { [tc,t1] }) } else { (**b).clone() };
                let n = b.poles.len();
                b.poles[if high { n-1 } else { 0 }] = q;
                Some(Curve::BSpline(Arc::new(b)))
            }
            Curve::Traced(t) => {
                let mut pts = t.pts.clone();
                if high {
                    if cut { pts.truncate(tc.floor() as usize+1) }
                    if pts.len() > 1 && distance(pts[pts.len()-1],q) <= tol { pts.pop(); }
                    pts.push(q);
                } else {
                    if cut { pts.drain(..tc.ceil() as usize); }
                    if pts.len() > 1 && distance(pts[0],q) <= tol { pts.remove(0); }
                    pts.insert(0,q);
                }
                Some(Curve::Traced(Arc::new(crate::geom::Traced {a:t.a.clone(),b:t.b.clone(),pts,closed:t.closed})))
            }
            _ => None,
        };
        let Some(ended) = ended else { continue };
        if crate::debug() { eprintln!("general: a branch beside a rail ended where the walls touch, at {q:?}, {:.3e} from where it was traced to",distance(p,q)) }
        c = ended;
    }
    c
}


/// How many pieces a stretch of the roll is cut into at the least, before any is halved.
const FIRST_PIECES: usize = 4;

/// How many readings of the contact's shape over the roll the instants it changes are sought
/// between.
const LOOKS: usize = 4096;

/// Where an end of a piece of the contact stands.
#[derive(Clone,Copy,Debug,PartialEq,Eq,PartialOrd,Ord)]
enum End {
    /// A vertex of the tool.
    Vertex(u32),
    /// On edge `edge`, where face `face`'s `n · v` vanishes along it: a face's chord reaching the edge.
    Root { edge: usize,face: usize },
}

/// A piece of an instant's contact, from its first end to its second.
#[derive(Clone,Copy,Debug,PartialEq,Eq)]
enum Piece {
    /// A stretch of sharp edge `edge`.
    Stretch { edge: usize,ends: [End;2] },
    /// A chord of plane face `face`.
    Chord { face: usize,ends: [End;2] },
}

impl Piece {
    fn ends(&self) -> [End;2] { match *self { Piece::Stretch {ends,..} | Piece::Chord {ends,..} => ends } }
}

/// The tool as the contact reads it: each line edge's ends, and each plane face's outward normal and
/// edges.
struct Tool<'a> {
    brep: &'a Brep,
    /// Per edge: its ends, `p₀` and `p₁`, and its faces where it is sharp.
    edges: Vec<Option<([V;2],Option<[usize;2]>)>>,
    /// Per face: its outward normal, and its edges.
    faces: Vec<(V,Vec<usize>)>,
}

impl<'a> Tool<'a> {
    fn read(brep: &'a Brep) -> Result<Tool<'a>,Unbuilt> {
        let sharp = super::contacts::sharp_edges(brep).map_err(|at| Unbuilt::Curved {at})?;
        let mut edges = vec![None;brep.edges.len()];
        for (e,edge) in brep.edges.iter().enumerate() {
            let EdgeCurve::Curve(c) = &edge.curve else { continue };
            if !matches!(c,Curve::Line {..}) { return Err(Unbuilt::Curved {at:c.point(edge.t[0])}) }
            let faces = sharp.iter().find(|x| x.0 == e).map(|x| x.1);
            edges[e] = Some(([c.point(edge.t[0]),c.point(edge.t[1])],faces));
        }
        let mut faces = Vec::new();
        for face in &brep.faces {
            let Surface::Plane(frame) = &face.surface else {
                let at = face.loops.iter().flatten().next().map_or([0.;3],|c| brep.vertices[brep.ends(c)[0] as usize].p);
                return Err(Unbuilt::Curved {at})
            };
            let n = if face.reversed { scale(frame.z,-1.) } else { frame.z };
            let mut own: Vec<usize> = face.loops.iter().flatten().map(|c| c.edge as usize).collect();
            own.sort();
            own.dedup();
            faces.push((n,own));
        }
        Ok(Tool {brep,edges,faces})
    }

    /// Face `f`'s `n · v` along edge `e` under twist `w`: its value at `p₀` and its rate to `p₁`.
    fn along(&self,w: &Twist,f: usize,e: usize) -> [f64;2] {
        let ([p0,p1],_) = self.edges[e].unwrap();
        let n = self.faces[f].0;
        [dot(n,w.velocity(p0)),dot(n,cross(w.omega,sub(p1,p0)))]
    }

    /// Where `end` stands under twist `w`, in the tool.
    fn point(&self,w: &Twist,end: End) -> V {
        match end {
            End::Vertex(v) => self.brep.vertices[v as usize].p,
            End::Root {edge,face} => {
                let ([p0,p1],_) = self.edges[edge].unwrap();
                let [a,b] = self.along(w,face,edge);
                add(p0,scale(sub(p1,p0),-a/b))
            }
        }
    }

    /// **The contact under twist `w`** (see the module): each sharp edge's stretches along it, then each
    /// face's chords along its line, in that order; refused where a face's `n · v` vanishes all along
    /// one of its edges.
    fn contact(&self,w: &Twist) -> Result<Vec<Piece>,Unbuilt> {
        let mut out = Vec::new();
        for (e,x) in self.edges.iter().enumerate() {
            let Some((_,Some([f1,f2]))) = *x else { continue };
            let (g1,g2) = (self.along(w,f1,e),self.along(w,f2,e));
            let v = self.brep.edges[e].v;
            // the places along it where a sign may change, and what stands there
            let mut cuts: Vec<(f64,End)> = vec![(0.,End::Vertex(v[0])),(1.,End::Vertex(v[1]))];
            for (g,f) in [(g1,f1),(g2,f2)] {
                let r = -g[0]/g[1];
                if r > 0. && r < 1. { cuts.push((r,End::Root {edge:e,face:f})) }
            }
            cuts.sort_by(|x,y| x.0.total_cmp(&y.0));
            let mut run: Option<End> = None;
            for k in 0..cuts.len()-1 {
                let m = 0.5*(cuts[k].0+cuts[k+1].0);
                let inside = (g1[0]+g1[1]*m)*(g2[0]+g2[1]*m) < 0.;
                match (inside,run) {
                    (true,None) => run = Some(cuts[k].1),
                    (false,Some(first)) => { out.push(Piece::Stretch {edge:e,ends:[first,cuts[k].1]}); run = None }
                    _ => {}
                }
            }
            if let Some(first) = run { out.push(Piece::Stretch {edge:e,ends:[first,cuts[cuts.len()-1].1]}) }
        }
        for (f,(n,own)) in self.faces.iter().enumerate() {
            // its line's way in its plane, `n × (n × ω)`, along which its crossings are ordered
            let way = cross(*n,cross(*n,w.omega));
            let mut crossings: Vec<(f64,End)> = Vec::new();
            for &e in own {
                let Some(([p0,p1],_)) = self.edges[e] else { continue };
                let [a,b] = self.along(w,f,e);
                if a == 0. && b == 0. { return Err(Unbuilt::Curved {at:p0}) }
                let r = -a/b;
                if !(r >= 0. && r < 1.) { continue }
                let end = if r == 0. { End::Vertex(self.brep.edges[e].v[0]) } else { End::Root {edge:e,face:f} };
                crossings.push((dot(add(p0,scale(sub(p1,p0),r)),way),end));
            }
            crossings.sort_by(|x,y| x.0.total_cmp(&y.0));
            for pair in crossings.chunks(2) {
                if let [x,y] = pair { if x.1 != y.1 { out.push(Piece::Chord {face:f,ends:[x.1,y.1]}) } }
            }
        }
        Ok(out)
    }
}

/// **The instants over `[a, b]` where the contact's shape changes**, and the shape between each two:
/// the shape read at `LOOKS` places, and between two that differ halved to the rounding of the roll,
/// as often as it changes there.
fn events(shape: &dyn Fn(f64) -> Result<Vec<Piece>,Unbuilt>,[a,b]: [f64;2],grain: f64) -> Result<(Vec<f64>,Vec<Vec<Piece>>),Unbuilt> {
    let at = |k: usize| if k == LOOKS { b } else { a+(b-a)*k as f64/LOOKS as f64 };
    let mut cuts = vec![a];
    let mut shapes: Vec<Vec<Piece>> = Vec::new();
    let mut here = shape(a)?;
    let mut lo = a;
    for k in 1..=LOOKS {
        let hi = at(k);
        let there = shape(hi)?;
        let mut from = lo;
        while there != here {
            // the first change past `from`: the last place reading `here`, halved to the rounding
            let (mut x,mut y) = (from,hi);
            while y-x > 1e-15*(b-a).max(1.)*(1.+x.abs()) {
                let m = 0.5*(x+y);
                if m <= x || m >= y { break }
                if shape(m)? == here { x = m } else { y = m }
            }
            shapes.push(here);
            cuts.push(y);
            here = shape(y)?;
            from = y;
            if y >= hi { here = there.clone(); break }
        }
        lo = hi;
    }
    shapes.push(here);
    cuts.push(b);
    // (a shape over less of the roll than `grain` — over which no point of the tool moves the
    // tolerance, #942 F6: two changes at one instant as the tolerance reads it — is the passing
    // state between its neighbours, and dropped: the next runs from the last instant kept)
    let mut out_cuts = vec![cuts[0]];
    let mut out_shapes: Vec<Vec<Piece>> = Vec::new();
    for (k,s) in shapes.into_iter().enumerate() {
        if cuts[k+1]-cuts[k] <= grain && k+1 < cuts.len()-1 { continue }
        if cuts[k+1]-out_cuts.last().unwrap() <= grain && !out_shapes.is_empty() { *out_cuts.last_mut().unwrap() = cuts[k+1]; continue }
        out_cuts.push(cuts[k+1]);
        out_shapes.push(s);
    }
    Ok((out_cuts,out_shapes))
}

/// **A surface pieces of the contact lie on over the roll** (a face family's wall, `family::Sheet`, a
/// line from end to end at each `s`): a sharp edge's line carried, its `u` the way along the edge;
/// or a plane face's characteristic line over a stretch of the roll it comes near the face, its `u`
/// the way along the chord `reach` either side of its point nearest the face's middle `q`.
struct Bearer { sheet: Arc<Sheet>,of: Bears }

#[derive(Clone,Copy)]
enum Bears { Edge(usize),Face { face: usize,q: V,n: V,reach: f64 } }

/// A bearer read over `[0, 1]²` (`contacts::pose_meeting`): its `u`, and its span of the roll as `v`.
struct Rolled<'a>(&'a Sheet);

impl super::contacts::Rolled for Rolled<'_> {
    fn at(&self,u: f64,v: f64) -> V { let [a,b] = self.0.span(); self.0.rim(u,a+(b-a)*v).0 }
    fn rate(&self,u: f64,v: f64) -> V { let [a,b] = self.0.span(); scale(self.0.d1([u,a+(b-a)*v]).2,b-a) }
    fn still(&self,_: f64,_: f64) -> bool { false }
}

/// One side of a strip of a bearer: its `u` at either end of the strip's stretch, its curve in space
/// over the stretch, and its curve in the bearer's parameters.
struct Side { u: [f64;2],curve: Curve,pcurve: Pcurve }

/// A piece of the contact over a stretch of the roll, on its bearer: its stretch, and its sides at
/// its lower and upper `u`.
struct Strip { span: [f64;2],sides: [Side;2] }

/// **The candidates of the closed solid `tool` sweeps under `path` over `roll`** (see the module):
/// each bearer of pieces of the contact, its strips — each piece over each stretch of the roll its
/// shape holds — its faces, and the tool at either end; with what is known of their meetings.
pub fn candidates(tool: &Brep,path: &Trajectory,roll: [f64;2],tol: f64) -> Result<(Vec<Brep>,Vec<crate::boolean::Given>),Unbuilt> {
    use crate::boolean::Given;
    let [a,b] = [roll[0].min(roll[1]),roll[0].max(roll[1])];
    let size = tool.size();
    let exact = crate::family::EXACT*size.max(1.);
    let read = Tool::read(tool)?;
    let posed = |s: f64| -> Result<Rigid,Unbuilt> { let q = path.pose(s)?; Ok(Rigid {r:q.r,t:q.p}) };
    let twist = |s: f64| -> Result<Twist,Unbuilt> { Ok(Twist::body(path.at(s)?)) };
    // (two changes nearer than the roll's grain at the tolerance are one instant, #942 F6)
    let grain = super::closed::grain(tool,path,[a,b],tol)?;
    let (cuts,shapes) = events(&|s| read.contact(&twist(s)?),[a,b],grain)?;
    // **no stretch longer than a quarter turn or the tool's size of travel** (the tool's motion read
    // at its box's corners): a strip's sides over a long roll — a bar screwed four turns — are curves
    // a search for their meeting with another strip reads more boxes of than any may
    let (cuts,shapes) = {
        let corners = { let (lo,hi) = tool.bounds(); [lo,hi,[lo[0],hi[1],lo[2]],[hi[0],lo[1],hi[2]]] };
        let reading = |s: f64| -> Result<Rigid,Unbuilt> { posed(s) };
        let mut out_cuts = vec![cuts[0]];
        let mut out_shapes = Vec::new();
        for (k,sh) in shapes.into_iter().enumerate() {
            let [lo,hi] = [cuts[k],cuts[k+1]];
            let looks = 64;
            let mut turn = 0f64;
            let mut travel = 0f64;
            let mut last = reading(lo)?;
            for j in 1..=looks {
                let m = reading(lo+(hi-lo)*j as f64/looks as f64)?;
                let tr: f64 = (0..3).map(|i| (0..3).map(|r| last.r[r][i]*m.r[r][i]).sum::<f64>()).sum();
                turn += ((tr-1.)/2.).clamp(-1.,1.).acos();
                travel = travel.max(corners.iter().map(|&c| distance(last.point(c),m.point(c))).fold(0.,f64::max)*looks as f64);
                last = m;
            }
            let n = ((turn/std::f64::consts::FRAC_PI_2).max(travel/size.max(1e-300)).ceil() as usize).max(1);
            for j in 1..=n {
                out_cuts.push(if j == n { hi } else { lo+(hi-lo)*j as f64/n as f64 });
                out_shapes.push(sh.clone());
            }
        }
        (out_cuts,out_shapes)
    };
    if crate::debug() {
        for (k,sh) in shapes.iter().enumerate() { eprintln!("general: over [{:.6}, {:.6}] {} pieces: {sh:?}",cuts[k],cuts[k+1],sh.len()) }
    }
    let line_numbers = |x: V,y: V| vec![x[0],x[1],x[2],y[0],y[1],y[2]];
    // **the bearers**: each sharp edge's line carried over the whole roll; each face's characteristic
    // line over each stretch of the roll it comes near the face
    let mut bearers: Vec<Bearer> = Vec::new();
    for (e,x) in read.edges.iter().enumerate() {
        let Some(([p0,p1],Some(_))) = *x else { continue };
        let numbers = |s: f64| -> Result<Vec<Vec<f64>>,Unbuilt> { let m = posed(s)?; Ok(vec![line_numbers(m.point(p0),m.point(p1))]) };
        let sheet = sheets(1,&numbers,[a,b],exact)?.remove(0);
        bearers.push(Bearer {sheet:Arc::new(sheet),of:Bears::Edge(e)});
    }
    for (f,(n,_)) in read.faces.iter().enumerate() {
        let face = &tool.faces[f];
        let corners: Vec<V> = face.loops.iter().flatten().map(|c| tool.vertices[tool.ends(c)[0] as usize].p).collect();
        let middle = scale(corners.iter().fold([0.;3],|acc,&p| add(acc,p)),1./corners.len().max(1) as f64);
        // (its characteristic line read as far as the face's reach about its middle either way from
        // the line's point nearest the middle: every chord of the face lies within the face, so
        // within that reach of the middle, and of that point along the line)
        let reach = corners.iter().map(|&p| distance(p,middle)).fold(0.,f64::max);
        let (n,q) = (*n,middle);
        let near = |s: f64| -> Result<bool,Unbuilt> { Ok(line(path,n,q,s)?.is_some_and(|(c,_)| distance(c,q) < reach)) };
        for span in runs(&near,&|s| Ok(line(path,n,q,s)?.is_some()),[a,b])? {
            let numbers = |s: f64| -> Result<Vec<Vec<f64>>,Unbuilt> {
                let (c,d) = line(path,n,q,s)?.ok_or_else(|| Unbuilt::Failed("a face's characteristic line lost partway".into()))?;
                let m = posed(s)?;
                Ok(vec![line_numbers(m.point(add(c,scale(d,-reach))),m.point(add(c,scale(d,reach))))])
            };
            if crate::debug() { eprintln!("general: face {f}'s characteristic line near it over [{:.6}, {:.6}]",span[0],span[1]) }
            let sheet = sheets(1,&numbers,span,exact)?.remove(0);
            bearers.push(Bearer {sheet:Arc::new(sheet),of:Bears::Face {face:f,q,n,reach}});
        }
    }
    // `end`'s `u` on bearer `bi` at `s`, under twist `w`
    let u_of = |bi: usize,w: &Twist,end: End,s: f64| -> Result<f64,Unbuilt> {
        match bearers[bi].of {
            Bears::Edge(e) => {
                let v = tool.edges[e].v;
                Ok(match end {
                    End::Vertex(x) if x == v[0] => 0.,
                    End::Vertex(x) if x == v[1] => 1.,
                    End::Root {face,..} => { let [g0,g1] = read.along(w,face,e); -g0/g1 }
                    End::Vertex(_) => return Err(Unbuilt::Failed("a stretch's end off its edge".into())),
                })
            }
            Bears::Face {q,n,reach,..} => {
                let (c,d) = line(path,n,q,s)?.ok_or_else(|| Unbuilt::Failed("a face's characteristic line lost partway".into()))?;
                Ok((dot(sub(read.point(w,end),c),d)+reach)/(2.*reach))
            }
        }
    };
    // **each stretch's strips**: each piece on its bearer, its moving sides' curves read at one set of
    // stations
    let mut strips: Vec<Vec<Strip>> = (0..bearers.len()).map(|_| Vec::new()).collect();
    for (k,shape) in shapes.iter().enumerate() {
        let span = [cuts[k],cuts[k+1]];
        // each piece's bearer
        let on: Vec<usize> = shape.iter().map(|p| bearers.iter().position(|br| match (br.of,*p) {
            (Bears::Edge(e),Piece::Stretch {edge,..}) => e == edge,
            (Bears::Face {face,..},Piece::Chord {face:f,..}) => face == f && { let [lo,hi] = br.sheet.span(); lo <= span[0] && hi >= span[1] },
            _ => false,
        }).ok_or_else(|| Unbuilt::Failed(format!("a piece of the contact over [{}, {}] on no bearer: {p:?}",span[0],span[1])))).collect::<Result<_,_>>()?;
        // the moving sides: each end that is no vertex of its edge bearer, its path in space and its
        // `u` on the bearer
        let mut moving: Vec<(usize,End)> = Vec::new();
        for (p,&bi) in shape.iter().zip(&on) {
            for end in p.ends() {
                let still = matches!((bearers[bi].of,end),(Bears::Edge(_),End::Vertex(_)));
                if !still && !moving.contains(&(bi,end)) { moving.push((bi,end)) }
            }
        }
        let numbers = |s: f64| -> Result<Vec<Vec<f64>>,Unbuilt> {
            let (w,m) = (twist(s)?,posed(s)?);
            let mut out = Vec::new();
            for &(bi,end) in &moving {
                let x = m.point(read.point(&w,end));
                let u = u_of(bi,&w,end,s)?;
                out.push(line_numbers(x,x));
                out.push(line_numbers([u,s,0.],[u,s,0.]));
            }
            Ok(out)
        };
        let made = if moving.is_empty() { Vec::new() } else { sheets(2*moving.len(),&numbers,span,exact)? };
        let made: Vec<Arc<Sheet>> = made.into_iter().map(Arc::new).collect();
        let w = [twist(span[0])?,twist(span[1])?];
        for (p,&bi) in shape.iter().zip(&on) {
            let sides = p.ends().map(|end| -> Result<Side,Unbuilt> {
                let u = [u_of(bi,&w[0],end,span[0])?,u_of(bi,&w[1],end,span[1])?];
                Ok(match moving.iter().position(|&x| x == (bi,end)) {
                    None => Side {u,curve:Curve::Rail(bearers[bi].sheet.clone(),u[0]),pcurve:Pcurve::Line {a:[u[0],span[0]],b:[u[1],span[1]]}},
                    Some(j) => Side {u,curve:Curve::Rail(made[2*j].clone(),0.),pcurve:Pcurve::Curve(Arc::new(Curve::Rail(made[2*j+1].clone(),0.)))},
                })
            });
            let [lo,hi] = sides;
            strips[bi].push(Strip {span,sides:[lo?,hi?]});
        }
    }
    if crate::debug() {
        for (bi,br) in bearers.iter().enumerate() {
            let what = match br.of { Bears::Edge(e) => format!("edge {e}"),Bears::Face {face,..} => format!("face {face}") };
            for st in &strips[bi] { eprintln!("general: bearer {bi} ({what}) strip over [{:.9}, {:.9}] u {:?} to {:?}",st.span[0],st.span[1],st.sides[0].u,st.sides[1].u) }
        }
    }
    // the candidates: each bearer holding strips, its strips its faces
    let held: Vec<usize> = (0..bearers.len()).filter(|&bi| !strips[bi].is_empty()).collect();
    let breps: Vec<Brep> = held.iter().map(|&bi| faces_of(&bearers[bi],&strips[bi],exact)).collect::<Result<_,_>>()?;
    let surfaces: Vec<Surface> = held.iter().map(|&bi| breps[held.iter().position(|&x| x == bi).unwrap()].faces[0].surface.clone()).collect();
    // **a fan that passes back through itself, its sheet restricted to runs of stretches**, found
    // before the bearers are met, so a meeting beside a rail is traced on each run's own wall
    // (#942 T1d-cube: traced on the whole wall, the trace's feet fell on the fold its other run
    // lays over it, and its branches ran 1e-5 off both walls, one out and back along itself)
    let spans: Vec<Vec<[f64;2]>> = held.iter().map(|&bi| strips[bi].iter().map(|st| st.span).collect()).collect();
    let mut breps = breps;
    let split = crossed(&mut breps,&spans,&surfaces,tol)?;
    let mut given: Vec<Given> = Vec::new();
    let mut per_run: Vec<Given> = Vec::new();
    for x in 0..held.len() { for y in x+1..held.len() {
        let (bi,bj) = (held[x],held[y]);
        let mut curves = Vec::new();
        match (bearers[bi].of,bearers[bj].of) {
            // **two edges with a vertex in common**: its path, a rail of both
            (Bears::Edge(e),Bears::Edge(e2)) => {
                let (v,v2) = (tool.edges[e].v,tool.edges[e2].v);
                for i in 0..2 { if v2.contains(&v[i]) { curves.push(Curve::Rail(bearers[bi].sheet.clone(),i as f64)) } }
                // (and where the two fans cross away from it: a given meeting is the whole of it,
                // and two edges' fans from one corner cross a second time beside its path — the
                // cube tumbling 5°, its top front edge's fan and its top side edge's, the hole
                // between them left open — traced from where an edge of either crosses the other)
                if !curves.is_empty() {
                    let beside = |a: &Brep,sa: &Surface,b: &Brep,sb: &Surface| -> Vec<Curve> {
                        beside_rails(a,sa,b,sb,&curves,tol).into_iter().map(|c| ended_on_rails(c,a,sa,b,sb,&curves,tol)).collect()
                    };
                    if split[x].is_none() && split[y].is_none() {
                        let met = beside(&breps[x],&surfaces[x],&breps[y],&surfaces[y]);
                        curves.extend(met);
                    } else {
                        for (a,sa) in walls_of(&breps[x],&surfaces[x],&split[x]) { for (b,sb) in walls_of(&breps[y],&surfaces[y],&split[y]) {
                            let met = beside(&a,&sa,&b,&sb);
                            if crate::debug() { for c in &met { let d = crate::query::curve_domain(c).unwrap_or([0.,1.]); eprintln!("general: given between bearers {bi} and {bj}, run by run: {} from {:?} to {:?}",c.kind(),c.point(d[0]),c.point(d[1])) } }
                            if !met.is_empty() { per_run.push(Given {surfaces:[sa.clone(),sb],curves:met,touches:Vec::new()}) }
                        } }
                    }
                }
            }
            // **a face and an edge of it**: the path of where the face's chord reaches the edge, where
            // the two touch — over each stretch whose shape holds that end, so each piece ends where
            // the end is born or passes a vertex
            (Bears::Edge(e),Bears::Face {face,..}) | (Bears::Face {face,..},Bears::Edge(e)) => {
                let end = End::Root {edge:e,face};
                let span = if let Bears::Face {..} = bearers[bi].of { bearers[bi].sheet.span() } else { bearers[bj].sheet.span() };
                for (k,shape) in shapes.iter().enumerate() {
                    if cuts[k] < span[0] || cuts[k+1] > span[1] || !shape.iter().any(|p| p.ends().contains(&end)) { continue }
                    let numbers = |s: f64| -> Result<Vec<Vec<f64>>,Unbuilt> {
                        let y = posed(s)?.point(read.point(&twist(s)?,end));
                        Ok(vec![line_numbers(y,y)])
                    };
                    curves.push(Curve::Rail(Arc::new(sheets(1,&numbers,[cuts[k],cuts[k+1]],exact)?.remove(0)),0.));
                }
            }
            _ => {}
        }
        if crate::debug() { for c in &curves { eprintln!("general: given between bearers {bi} and {bj}: {} from {:?} to {:?}",c.kind(),crate::query::curve_domain(c).map(|d| c.point(d[0])),crate::query::curve_domain(c).map(|d| c.point(d[1]))) } }
        if !curves.is_empty() { given.push(Given {surfaces:[surfaces[x].clone(),surfaces[y].clone()],curves,touches:Vec::new()}) }
    } }
    // **each bearer's whole meeting with each face of the tool at either end**
    let ends = [posed(a)?,posed(b)?];
    let poses: Vec<Brep> = ends.iter().map(|m| tool.moved(m)).collect();
    let located = crate::query::Located::new(tool,tol);
    let boxes: Vec<Vec<crate::geom::Box3>> = poses.iter().map(|p| (0..p.faces.len()).map(|fi| crate::query::face_box(p,fi)).collect()).collect();
    let walled: Vec<crate::geom::Box3> = breps.iter().map(|s| s.bounds()).collect();
    let jobs: Vec<(usize,usize,usize)> = (0..held.len()).flat_map(|k| (0..2).flat_map(move |e| (0..tool.faces.len()).map(move |fi| (k,e,fi)))).collect();
    let met: Vec<Given> = crate::util::par::indices(jobs.len(),|j| {
        let (k,e,fi) = jobs[j];
        let br = &bearers[held[k]];
        let [lo,hi] = br.sheet.span();
        // (the bearer's own end at this end of the roll, where its span holds it, and the faces its
        // line lies on there: an edge's two, crossing each simply; a face's characteristic line its
        // face, touching it)
        let at_end = if e == 0 { lo == a } else { hi == b };
        let (on,order) = match br.of { Bears::Edge(edge) => (read.edges[edge].and_then(|x| x.1).map_or(vec![],|f| f.to_vec()),1),
            Bears::Face {face,..} => (vec![face],2) };
        let touch = (at_end && on.contains(&fi)).then_some((e as f64,order));
        let f = &poses[e].faces[fi];
        let mut curves: Vec<Curve> = touch.map(|_| Curve::Rim(br.sheet.clone(),if e == 0 { lo } else { hi })).into_iter().collect();
        if touch.is_some() || crate::boolean::overlap(&walled[k],&boxes[e][fi],tol) {
            let back = ends[e].inverse();
            let place = |p: V| located.face_place(fi,back.point(p));
            // (the strips' corners on the rim there, where a branch leaving the rim at the end of a
            // piece of the contact — beside where its `h` reads it — ends exactly)
            let at = if e == 0 { lo } else { hi };
            let corners: Vec<[f64;2]> = if touch.is_none() { Vec::new() } else {
                strips[held[k]].iter().flat_map(|st| (0..2).filter(move |&i| st.span[i] == at).flat_map(move |i| st.sides.iter().map(move |sd| [sd.u[i],e as f64]))).collect()
            };
            curves.extend(super::contacts::pose_meeting(&Rolled(&br.sheet),&f.surface,boxes[e][fi],touch,None,&corners,&place,tol));
        }
        if crate::debug() && !curves.is_empty() {
            let what = match br.of { Bears::Edge(x) => format!("edge {x}"),Bears::Face {face,..} => format!("face {face}") };
            eprintln!("general: bearer {} ({what}) meets the tool's face {fi} at end {e}{}: {:?}",held[k],if touch.is_some() { ", touching" } else { "" },
                curves.iter().map(|c| { let d = crate::query::curve_domain(c).unwrap_or([0.,0.]); (c.kind(),c.point(d[0]),c.point(d[1])) }).collect::<Vec<_>>());
        }
        Given {surfaces:[surfaces[k].clone(),f.surface.clone()],curves,touches:Vec::new()}
    });
    given.extend(met);
    let (breps,given) = given_to_runs(breps,&split,&surfaces,given,per_run,tol)?;
    let mut out = breps;
    out.extend(poses);
    Ok((out,given))
}

/// **A bearer whose fan passes back through itself, its sheet a surface per run of stretches**
/// (#942 T1d-cube): an edge's fan is one wall over the whole roll, its strips each a face on it, and
/// where the edge's line passes back through where it stood a while before — the bench's cube
/// tumbling 75°, its edge 5's strip over `[0.39, 0.50]` crossing its strip over `[0.23, 0.29]` — two
/// faces of one surface cross, a meeting of a surface with itself that no given meeting describes,
/// and the trim refused it from its witness. Each two of a bearer's stretches are met as two
/// surfaces, its wall restricted to each (`Sheet::restricted`), from where an edge of either's
/// strips crosses the other away from the rim they share (`beside_rails`); a fan crossing itself
/// nowhere stays one surface, as it was. One that does is cut into the fewest runs of stretches that
/// part every two crossing ones, each run a surface of its own — the whole wall framed at the run, so
/// every curve on the wall lies on it — and a candidate of its own (the trim meets no two faces of
/// one candidate); each
/// meeting given of the whole wall is given of each run, and each two runs are given the rim they
/// share and where they cross, met run against run as the wall restricted to each (past a stretch
/// its own end pieces carried on, so two runs of one wall meet only where it crosses itself), so a
/// branch is one curve across a run.
fn crossed(breps: &mut [Brep],spans: &[Vec<[f64;2]>],surfaces: &[Surface],tol: f64) -> Result<Vec<Option<Runs>>,Unbuilt> {
    let surface_of = |sheet: &Sheet,[s0,s1]: [f64;2]| -> Result<Surface,Unbuilt> {
        let r = Arc::new(sheet.restricted([s0,s1]).map_err(Unbuilt::Failed)?);
        let (p,du,ds) = r.d1([0.5,0.5*(s0+s1)]);
        let normal = crate::util::space::normalised(cross(du,ds)).unwrap_or([0.,0.,1.]);
        Ok(Surface::Family(Frame::about(p,normal),r))
    };

    // per bearer split: its runs, each its span, surface and faces' indices, and the curves given
    // between each two runs
    let mut split: Vec<Option<Runs>> = vec![None;breps.len()];
    for x in 0..breps.len() {
        let Surface::Family(_,sheet) = &surfaces[x] else { continue };
        let mut stretches: Vec<[f64;2]> = spans[x].clone();
        stretches.sort_by(|a,b| a[0].total_cmp(&b[0]));
        stretches.dedup();
        if stretches.len() < 2 { continue }
        let alone: Vec<Surface> = stretches.iter().map(|&sp| surface_of(sheet,sp)).collect::<Result<_,_>>()?;
        let faces_on = |k: usize| -> Brep {
            let mut b = breps[x].clone();
            let keep: Vec<bool> = spans[x].iter().map(|&sp| sp == stretches[k]).collect();
            let mut i = 0;
            b.faces.retain(|_| { i += 1; keep[i-1] });
            for f in &mut b.faces { f.surface = alone[k].clone() }
            b.compacted()
        };
        let pieces: Vec<Brep> = (0..stretches.len()).map(faces_on).collect();
        let rim = |i: usize,j: usize| -> Vec<Curve> {
            if stretches[i][1] == stretches[j][0] { vec![Curve::Rim(sheet.clone(),stretches[i][1])] } else { Vec::new() }
        };
        let mut crossing: Vec<(usize,usize,Vec<Curve>)> = Vec::new();
        for i in 0..stretches.len() { for j in i+1..stretches.len() {
            let met = beside_rails(&pieces[i],&alone[i],&pieces[j],&alone[j],&rim(i,j),tol);
            if !met.is_empty() {
                if crate::debug() { eprintln!("general: a fan crossing itself: its stretches over {:?} and {:?}, {} curves",stretches[i],stretches[j],met.len()) }
                crossing.push((i,j,met));
            }
        } }
        if crossing.is_empty() { continue }
        // the fewest cuts parting every two crossing stretches: each pair's first cut at its later
        // stretch where no cut already parts it (the earliest-ending first)
        let mut by_end: Vec<(usize,usize)> = crossing.iter().map(|c| (c.0,c.1)).collect();
        by_end.sort_by_key(|&(i,j)| (j,i));
        let mut cuts: Vec<usize> = Vec::new();
        for (i,j) in by_end { if !cuts.iter().any(|&c| c > i && c <= j) { cuts.push(j) } }
        cuts.sort();
        let run_of = |k: usize| cuts.iter().filter(|&&c| c <= k).count();
        let runs = cuts.len()+1;
        let mut out: Vec<([f64;2],Surface)> = Vec::new();
        for r in 0..runs {
            let ks: Vec<usize> = (0..stretches.len()).filter(|&k| run_of(k) == r).collect();
            let sp = [stretches[ks[0]][0],stretches[ks[ks.len()-1]][1]];
            out.push((sp,surface_of(sheet,sp)?));
        }
        for (f,sp) in breps[x].faces.iter_mut().zip(&spans[x]) {
            let k = stretches.iter().position(|s| s == sp).unwrap_or(0);
            f.surface = out[run_of(k)].1.clone();
        }
        // (each two runs met again as the two surfaces they are, so a branch crossing several of a
        // run's stretches is one curve: traced a pair of stretches at a time, its pieces ended some
        // tolerances apart where one stretch gives way to the next, and closed nothing)
        let on_run = |r: usize| { let mut b = breps[x].clone(); b.faces.retain(|f| f.surface == out[r].1); b.compacted() };
        let mut between: Vec<(usize,usize,Vec<Curve>)> = Vec::new();
        for g in 0..runs { for h in g+1..runs {
            let rails: Vec<Curve> = if out[g].0[1] == out[h].0[0] { vec![Curve::Rim(sheet.clone(),out[g].0[1])] } else { Vec::new() };
            let met = beside_rails(&on_run(g),&out[g].1,&on_run(h),&out[h].1,&rails,tol);
            let mut curves = rails;
            curves.extend(met);
            between.push((g,h,curves));
        } }
        if crate::debug() { eprintln!("general: a fan cut into {} runs: {:?}",out.len(),out.iter().map(|r| r.0).collect::<Vec<_>>()) }
        split[x] = Some((out,between));
    }
    Ok(split)
}

/// A split bearer's runs, each its span and surface, and the curves given between each two runs.
type Runs = (Vec<([f64;2],Surface)>,Vec<(usize,usize,Vec<Curve>)>);

/// A bearer's wall as the pieces it is met in: the whole of it, or each of its runs where it passes
/// back through itself (`crossed`), each the faces on it and its surface.
fn walls_of(b: &Brep,whole: &Surface,split: &Option<Runs>) -> Vec<(Brep,Surface)> {
    match split {
        None => vec![(b.clone(),whole.clone())],
        Some((runs,_)) => runs.iter().map(|r| { let mut c = b.clone(); c.faces.retain(|f| f.surface == r.1); (c.compacted(),r.1.clone()) }).collect(),
    }
}

/// **The meetings given of split walls, given of their runs** (#942 T1d-cube): each split bearer a
/// candidate per run, and each meeting given of its whole wall given of each run (`on_run`); with
/// `per_run`, the meetings found run by run, joined to their runs' meetings.
fn given_to_runs(breps: Vec<Brep>,split: &[Option<Runs>],surfaces: &[Surface],given: Vec<crate::boolean::Given>,
    per_run: Vec<crate::boolean::Given>,tol: f64) -> Result<(Vec<Brep>,Vec<crate::boolean::Given>),Unbuilt> {
    use crate::boolean::Given;
    if split.iter().all(Option::is_none) {
        let mut given = given;
        given.extend(per_run);
        return Ok((breps,given))
    }
    // (each run a candidate of its own: a candidate is one sheet, its faces never met with one
    // another, and two runs that cross are met as two candidates are)
    let breps: Vec<Brep> = breps.iter().zip(surfaces).zip(split).flat_map(|((b,s),sp)| walls_of(b,s,sp).into_iter().map(|w| w.0)).collect();
    // each meeting given of a split bearer's whole wall, given of each of its runs, each curve as
    // much of it as lies on the run (a given curve lies on both its surfaces, and a run's is its wall
    // over the run alone): a rail, a path over the roll, over the run's span; any other cut to the
    // stretches of it that lie on the run's wall (#942 T1d-cube: a curve traced on the whole wall
    // passes from one run's stretches to the other's, and given whole to each, wherever a point of
    // it stood within the run, ran 3.8e-3 off the run's wall along the other run's fold)
    let runs_of = |s: &Surface| -> Vec<(Surface,Option<(Arc<Sheet>,[f64;2])>)> {
        let found = surfaces.iter().position(|x| x == s).and_then(|x| split[x].as_ref().map(|sp| (x,sp)));
        match found {
            Some((x,(runs,_))) => {
                let Surface::Family(_,sheet) = &surfaces[x] else { return vec![(s.clone(),None)] };
                runs.iter().map(|r| (r.1.clone(),Some((sheet.clone(),r.0)))).collect()
            }
            None => vec![(s.clone(),None)],
        }
    };
    // (one restriction of a wall to a span, shared by every curve given over it, as a dump read back
    // shares it: a wall read anew from where its own was cached reads its points otherwise to the
    // rounding)
    let restricted = std::cell::RefCell::new(std::collections::BTreeMap::<(usize,u64,u64),Arc<Sheet>>::new());
    let on_run = |c: &Curve,wall: &Surface,run: &Option<(Arc<Sheet>,[f64;2])>| -> Result<Vec<Curve>,Unbuilt> {
        let Some((sheet,[r0,r1])) = run else { return Ok(vec![c.clone()]) };
        // (a roll within the run, to the parameters' rounding of its span)
        let slack = 1e-12*(r1-r0).abs().max(1.);
        let within = |s: f64| s >= r0-slack && s <= r1+slack;
        Ok(match c {
            Curve::Rail(w,u) => {
                let [a,b] = w.span();
                let [lo,hi] = [a.max(*r0),b.min(*r1)];
                if !(hi > lo) { Vec::new() } else if lo == a && hi == b { vec![c.clone()] }
                else {
                    let key = (Arc::as_ptr(w) as usize,lo.to_bits(),hi.to_bits());
                    let cached = restricted.borrow().get(&key).cloned();
                    let r = match cached { Some(r) => r,None => {
                        let r = Arc::new(w.restricted([lo,hi]).map_err(Unbuilt::Failed)?);
                        restricted.borrow_mut().insert(key,r.clone());
                        r
                    } };
                    vec![Curve::Rail(r,*u)]
                }
            }
            Curve::Rim(w,s) => if Arc::ptr_eq(w,sheet) { within(*s).then(|| c.clone()).into_iter().collect() } else { on_wall(c,wall,tol) },
            _ => on_wall(c,wall,tol),
        })
    };
    let mut out = Vec::new();
    for g in given {
        for (a,ra) in runs_of(&g.surfaces[0]) { for (b,rb) in runs_of(&g.surfaces[1]) {
            let mut curves = Vec::new();
            for c in &g.curves {
                for c in on_run(c,&a,&ra)? { curves.extend(on_run(&c,&b,&rb)?) }
            }
            out.push(Given {surfaces:[a.clone(),b],curves,touches:g.touches.clone()})
        } }
    }
    for (runs,between) in split.iter().flatten() {
        for (g,h,curves) in between { out.push(Given {surfaces:[runs[*g].1.clone(),runs[*h].1.clone()],curves:curves.clone(),touches:Vec::new()}) }
    }
    // (a meeting found run by run joined to what its two runs are given already: one given meeting
    // of two surfaces is the whole of it)
    for g in per_run {
        match out.iter_mut().find(|o| o.surfaces == g.surfaces) {
            Some(o) => o.curves.extend(g.curves),
            None => out.push(g),
        }
    }
    Ok((breps,out))
}

/// **The stretches of a curve that lie on a wall** (#942 T1d-cube): its points read against the
/// wall, a run of a fan over its stretches, at `LOOKS` places along it — on it where the wall's
/// nearest point stands within what a surface's patch is read to (`1e3 tol`, `boolean`'s), off it
/// on the other run's fold — and each change from one to the other found by halving to the
/// rounding, the curve cut there (a B-spline exactly, a trace at its points); a curve of another
/// kind kept whole where any point of it is on the wall.
fn on_wall(c: &Curve,wall: &Surface,tol: f64) -> Vec<Curve> {
    const LOOKS: usize = 64;
    let Some([t0,t1]) = crate::query::curve_domain(c) else { return vec![c.clone()] };
    let on = |t: f64| { let p = c.point(t); distance(wall.point(wall.inverse(p)),p) <= 1e3*tol };
    let at: Vec<(f64,bool)> = (0..=LOOKS).map(|k| { let t = t0+(t1-t0)*k as f64/LOOKS as f64; (t,on(t)) }).collect();
    if at.iter().all(|x| x.1) { return vec![c.clone()] }
    if !matches!(c,Curve::BSpline(_) | Curve::Traced(_)) {
        return if at.iter().any(|x| x.1) { vec![c.clone()] } else { Vec::new() }
    }
    // each change between two looks, found by halving: where the curve leaves the wall or comes on it
    let change = |a: f64,b: f64,from: bool| crate::util::roots::bisect(a,b,|x,y| 0.5*(x+y),|x,y| y-x > f64::EPSILON*(x.abs()+y.abs()).max(1.),
        |m| on(m) == from);
    let mut pieces: Vec<[f64;2]> = Vec::new();
    let mut start = at[0].1.then_some(t0);
    for w in at.windows(2) {
        match (w[0].1,w[1].1) {
            (true,false) => { if let Some(s) = start.take() { pieces.push([s,change(w[0].0,w[1].0,true).0]) } }
            (false,true) => { start = Some(change(w[0].0,w[1].0,false).1) }
            _ => {}
        }
    }
    if let Some(s) = start { pieces.push([s,t1]) }
    pieces.into_iter().filter(|&[a,b]| b > a).filter_map(|[a,b]| match c {
        Curve::BSpline(sp) => Some(Curve::BSpline(Arc::new(sp.segment([a,b])))),
        Curve::Traced(tr) => {
            // (its points between, and the two ends where it was cut)
            let mut pts = vec![c.point(a)];
            pts.extend(tr.pts.iter().enumerate().filter(|&(i,_)| (i as f64) > a && (i as f64) < b).map(|(_,&p)| p));
            pts.push(c.point(b));
            (pts.len() >= 2).then(|| Curve::Traced(Arc::new(crate::geom::Traced {a:tr.a.clone(),b:tr.b.clone(),pts,closed:false})))
        }
        _ => None,
    }).collect()
}

/// How many stations a bearer is read at for whether its lines stand on one cylinder.
const CYLINDER_LOOKS: usize = 64;

/// **The cylinder a bearer's every line stands on**, if one does (#880 T1d): its lines all parallel
/// and their feet in the plane square to them on one circle, each within `slack` (a few times what
/// the sheet is fitted to: it stands so near the cylinder that the kernel's tolerance holds both) — a
/// bar's edge parallel to the axis of a screw whose lead grows, carried round at one radius, as its
/// opposite edge is. `None` where the lines stand still or sweep a plane.
fn on_a_cylinder(sheet: &Sheet,slack: f64) -> Option<Surface> {
    let [a,b] = sheet.span();
    let lines: Vec<(V,V)> = (0..=CYLINDER_LOOKS).map(|k| {
        let s = a+(b-a)*k as f64/CYLINDER_LOOKS as f64;
        (sheet.rim(0.,s).0,sheet.rim(1.,s).0)
    }).collect();
    let d = crate::util::space::normalised(sub(lines[0].1,lines[0].0))?;
    if lines.iter().any(|(p,q)| crate::util::space::norm(cross(sub(*q,*p),d)) > 2.*slack) { return None }
    // each line's foot in the plane through the origin square to `d`
    let feet: Vec<V> = lines.iter().map(|(p,_)| sub(*p,scale(d,dot(*p,d)))).collect();
    let (p0,p1,p2) = (feet[0],feet[CYLINDER_LOOKS/2],feet[CYLINDER_LOOKS]);
    let (e1,e2) = (sub(p1,p0),sub(p2,p0));
    let n = cross(e1,e2);
    let nn = dot(n,n);
    let reach = feet.iter().map(|&f| distance(f,p0)).fold(0.,f64::max);
    // (feet within `slack` of a line make no circle: the third stands `|n|/|e₁|` off the first two's
    // line, and `|e₁|` is within the reach, #942 F6)
    if !(nn.sqrt() > slack*reach) { return None }
    // the circle through three feet: its centre
    let c = add(p0,scale(add(scale(cross(n,e1),dot(e2,e2)),scale(cross(e2,n),dot(e1,e1))),0.5/nn));
    let r = distance(p0,c);
    if feet.iter().any(|&f| (distance(f,c)-r).abs() > slack) { return None }
    Some(Surface::Cylinder(Frame::new(c,d,sub(p0,c)),r))
}

/// **A bearer's strips as the faces of one candidate**: each strip a face on the bearer, bounded by
/// its two sides and its rims at either end of its stretch; two strips meeting at an instant share
/// the rim edges between them, cut at every strip's corner there, and a strip born or vanishing at
/// an instant (its rim one point) has none there.
fn faces_of(bearer: &Bearer,strips: &[Strip],exact: f64) -> Result<Brep,Unbuilt> {
    let sheet = &bearer.sheet;
    // (a bearer whose every line stands on one cylinder is that cylinder: two of them alike are one
    // surface the Boolean knows as one, each face's parameters its edges inverted onto it)
    let cylinder = on_a_cylinder(sheet,4.*exact);
    let surface = match &cylinder {
        Some(c) => c.clone(),
        None => {
            let [a,b] = sheet.span();
            let (p,du,ds) = sheet.d1([0.5,0.5*(a+b)]);
            let normal = crate::util::space::normalised(cross(du,ds)).unwrap_or([0.,0.,1.]);
            Surface::Family(Frame::about(p,normal),sheet.clone())
        }
    };
    let mut out = Brep::default();
    // (two corners at an instant one where their rims' points stand within `exact` of one another,
    // #942 F6)
    let same = |s: f64,u: f64,w: f64| u == w || distance(sheet.rim(u,s).0,sheet.rim(w,s).0) <= exact;
    // the corners at each instant: (s, u) to a vertex
    let mut corners: Vec<(f64,f64,u32)> = Vec::new();
    for st in strips { for i in 0..2 { for side in &st.sides {
        let (s,u) = (st.span[i],side.u[i]);
        if corners.iter().any(|c| c.0 == s && same(s,c.1,u)) { continue }
        let v = out.vertex(sheet.rim(u,s).0);
        corners.push((s,u,v));
    } } }
    let corner = |s: f64,u: f64| corners.iter().find(|c| c.0 == s && same(s,c.1,u)).map_or(0,|c| c.2);
    // the rim edges at each instant between each two corners there, made as a strip asks
    let mut rims: Vec<(f64,[f64;2],u32)> = Vec::new();
    let mut faces = Vec::new();
    for st in strips {
        let mut l: Vec<Coedge> = Vec::new();
        // along the rim at the stretch's start, lower side to upper; up the upper side; back along the
        // rim at its end; down the lower side
        for (i,forward) in [(0,true),(1,false)] {
            let s = st.span[i];
            let (lo,hi) = (st.sides[0].u[i],st.sides[1].u[i]);
            let mut stops: Vec<(f64,u32)> = corners.iter().filter(|c| c.0 == s && (c.1 >= lo || same(s,c.1,lo)) && (c.1 <= hi || same(s,c.1,hi)))
                .map(|c| (c.1,c.2)).collect();
            stops.sort_by(|x,y| x.0.total_cmp(&y.0));
            stops.dedup_by(|x,y| x.1 == y.1);
            let mut pieces: Vec<Coedge> = Vec::new();
            for w in stops.windows(2) {
                let ([u0,v0],[u1,v1]) = ([w[0].0,w[0].1 as f64],[w[1].0,w[1].1 as f64]);
                let (v0,v1) = (v0 as u32,v1 as u32);
                if v0 == v1 { continue }
                let edge = match rims.iter().find(|r| r.0 == s && same(s,r.1[0],u0) && same(s,r.1[1],u1)) {
                    Some(r) => r.2,
                    None => { let e = out.edge(EdgeCurve::Curve(Curve::Rim(sheet.clone(),s)),[u0,u1],[v0,v1]); rims.push((s,[u0,u1],e)); e }
                };
                pieces.push(Coedge {edge,reversed:!forward,pcurve:Pcurve::Line {a:[u0,s],b:[u1,s]}});
            }
            if !forward { pieces.reverse() }
            l.extend(pieces);
            let side = &st.sides[if i == 0 { 1 } else { 0 }];
            let ends = [corner(st.span[0],side.u[0]),corner(st.span[1],side.u[1])];
            let e = out.edge(EdgeCurve::Curve(side.curve.clone()),st.span,ends);
            l.push(Coedge {edge:e,reversed:i == 1,pcurve:side.pcurve.clone()});
        }
        let name = match bearer.of { Bears::Edge(e) => format!("edge {e} over [{:.4}, {:.4}]",st.span[0],st.span[1]),
            Bears::Face {face,..} => format!("face {face} over [{:.4}, {:.4}]",st.span[0],st.span[1]) };
        faces.push(Face {surface:surface.clone(),reversed:false,loops:vec![l],name});
    }
    out.faces = faces;
    if let Some(Surface::Cylinder(frame,_)) = &cylinder { on_cylinder(&mut out,sheet,frame) }
    Ok(out)
}

/// **A bearer's faces put on the cylinder its lines stand on** (`on_a_cylinder`): each coedge's
/// parameters its edge inverted onto the cylinder, its angle unwrapped round the loop — at the
/// coedge's start beside the last one's end, its middle beside its start, its end beside its middle
/// (no strip turns half a turn: a stretch is at most a quarter) — and each face turned over where the
/// bearer's own normal there, `Σ_u × Σ_s`, stands against the cylinder's outward one.
fn on_cylinder(b: &mut Brep,sheet: &Sheet,frame: &Frame) {
    let angle = |p: V| { let q = sub(p,frame.o); dot(q,frame.y).atan2(dot(q,frame.x)) };
    let height = |p: V| dot(sub(p,frame.o),frame.z);
    let near = |x: f64,to: f64| to+(x-to+std::f64::consts::PI).rem_euclid(std::f64::consts::TAU)-std::f64::consts::PI;
    let [lo,hi] = sheet.span();
    for f in 0..b.faces.len() {
        let mut last: Option<f64> = None;
        let mut loops = b.faces[f].loops.clone();
        for l in &mut loops {
            for c in l.iter_mut() {
                let e = &b.edges[c.edge as usize];
                let [p0,p1] = e.v.map(|v| b.vertices[v as usize].p);
                let pm = e.point(0.5*(e.t[0]+e.t[1]),&b.vertices);
                let (first,second) = if c.reversed { (p1,p0) } else { (p0,p1) };
                let t0 = { let t = angle(first); last.map_or(t,|w| near(t,w)) };
                let tm = near(angle(pm),t0);
                let t1 = near(angle(second),tm);
                last = Some(t1);
                let (ua,ub) = ([t0,height(first)],[t1,height(second)]);
                c.pcurve = if c.reversed { Pcurve::Inverse {a:ub,b:ua} } else { Pcurve::Inverse {a:ua,b:ub} };
            }
        }
        // (the face's middle: its first corner, read where the bearer is)
        let (s,u) = (0.5*(lo+hi),0.5);
        let (p,du,ds) = sheet.d1([u,s]);
        let out = sub(sub(p,frame.o),scale(frame.z,dot(sub(p,frame.o),frame.z)));
        b.faces[f].loops = loops;
        b.faces[f].reversed = dot(cross(du,ds),out) < 0.;
    }
}

/// **`count` walls, each a line whose ends at `s` are `numbers(s)[k]`, over `[a, b]`**
/// (`family::Sheet`), read at one set of stations: the stretch cut into pieces, each a quintic through
/// the numbers at its six Gauss–Lobatto stations, and a piece halved where any wall's quintic stands
/// off its numbers by more than `exact` midway between two of its stations.
fn sheets(count: usize,numbers: &dyn Fn(f64) -> Result<Vec<Vec<f64>>,Unbuilt>,[a,b]: [f64;2],exact: f64) -> Result<Vec<Sheet>,Unbuilt> {
    let mut todo: Vec<[f64;2]> = (0..FIRST_PIECES).rev().map(|k| {
        let w = (b-a)/FIRST_PIECES as f64;
        [a+w*k as f64,if k+1 == FIRST_PIECES { b } else { a+w*(k+1) as f64 }]
    }).collect();
    let (mut spans,mut at) = (Vec::new(),Vec::new());
    let mut nodes: Vec<Vec<Vec<f64>>> = vec![Vec::new();count];
    while let Some([lo,hi]) = todo.pop() {
        let (mid,half) = (0.5*(lo+hi),0.5*(hi-lo));
        let ss: [f64;6] = NODES.map(|x| mid+half*x);
        let ns: Vec<Vec<Vec<f64>>> = ss.iter().map(|&s| numbers(s)).collect::<Result<_,_>>()?;
        // each wall's quintic read midway between each two stations, against its numbers there
        let mut off = 0f64;
        for s in ss.windows(2).map(|w| 0.5*(w[0]+w[1])) {
            let wt = crate::family::weights(&ss,s);
            let want = numbers(s)?;
            for k in 0..count {
                for (x,&y) in want[k].iter().enumerate() {
                    off = off.max((y-(0..6).map(|i| wt[i]*ns[i][k][x]).sum::<f64>()).abs());
                }
            }
        }
        // (halved while its middle stands between its ends, the roll's rounding: refused there)
        if off > exact && mid > lo && mid < hi {
            todo.push([mid,hi]);
            todo.push([lo,mid]);
            continue
        }
        if off > exact { return Err(Unbuilt::Failed(format!("a moving piece of contact's ends are no quintic within {exact:e} over [{lo}, {hi}]: {off:e} off"))) }
        spans.push([lo,hi]);
        at.push(ss);
        for k in 0..count { nodes[k].extend(ns.iter().map(|n| n[k].clone())) }
    }
    let stations = Arc::new(Stations::new(spans,at).map_err(Unbuilt::Failed)?);
    nodes.into_iter().map(|n| Sheet::new(Run::Line,stations.clone(),n,false).map_err(Unbuilt::Failed)).collect()
}

/// The characteristic line of the plane through `q` with outward normal `n` at `s` under `path`, in
/// the tool's coordinates: its point nearest `q` and its way, `n × (n × ω)`. `n · v` is affine over
/// the plane, `n · v₀ + x · (n × ω)`, and the line where it vanishes runs square to `n × ω` within
/// the plane; none where `n × ω` vanishes (the plane turning about its own normal, or not turning).
fn line(path: &Trajectory,n: V,q: V,s: f64) -> Result<Option<(V,V)>,Unbuilt> {
    let w = Twist::body(path.at(s)?);
    let g = cross(n,w.omega);
    let gg = dot(g,g);
    // (where `n × ω` all but vanishes the line stands far off, past any face's reach about `q`,
    // where `near` reads it so: only none at all is none, #942 F6)
    if !(gg > 0.) { return Ok(None) }
    let c = add(q,scale(g,(-dot(n,w.v0)-dot(q,g))/gg));
    Ok(Some((c,scale(cross(n,g),1./gg.sqrt()))))
}

/// The stretches of `[a, b]` where `holds`, read at `LOOKS` points and each run a step on past
/// either end where `read` (what `holds` reads is defined there): so a surface over one runs on past
/// where it holds.
fn runs(holds: &dyn Fn(f64) -> Result<bool,Unbuilt>,read: &dyn Fn(f64) -> Result<bool,Unbuilt>,[a,b]: [f64;2]) -> Result<Vec<[f64;2]>,Unbuilt> {
    let looks: Vec<f64> = (0..=LOOKS).map(|k| if k == LOOKS { b } else { a+(b-a)*k as f64/LOOKS as f64 }).collect();
    let mut found: Vec<[usize;2]> = Vec::new();
    for (k,&s) in looks.iter().enumerate() {
        if !holds(s)? { continue }
        match found.last_mut() { Some(r) if r[1]+1 == k => r[1] = k,_ => found.push([k,k]) }
    }
    let mut out = Vec::new();
    for [i,j] in found {
        let i = if i > 0 && read(looks[i-1])? { i-1 } else { i };
        let j = if j < LOOKS && read(looks[j+1])? { j+1 } else { j };
        if j > i { out.push([looks[i],looks[j]]) }
    }
    Ok(out)
}
