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
//! - **the cube turned farther** (brep-bench's `tumbling-cube`, 75°): a fan's stretch born at a
//!   point of its edge, where both faces' `n · v` vanish, the edge's velocity runs along it, and its
//!   carried line has a pinch point (a Whitney umbrella). The strip's two sides leave it tangent within
//!   the kernel's tolerance for 1.4e-4, and the Boolean merges the start of one onto the other a little
//!   way along it, its first stretch a spur the face's pieces now drop (`boolean`'s walk). Past 15° on
//!   the start pose's face two cuts meet where three sheets do, one ending 1.5e-4 short of the other,
//!   and the face's pieces do not close.
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
        // (a seed on a branch met already: the seed and the branch each on the meeting within the
        // tolerance)
        let on = traced.iter().any(|c| distance(c.point(c.inverse(p)),p) <= 2.*tol);
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
    let mut given: Vec<Given> = Vec::new();
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
                if !curves.is_empty() { curves.extend(beside_rails(&breps[x],&surfaces[x],&breps[y],&surfaces[y],&curves,tol)) }
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
    let mut out = breps;
    out.extend(poses);
    Ok((out,given))
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
