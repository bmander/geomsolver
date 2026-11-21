//! What a boundary read from a file must be to be a solid's, beyond what `topo::Brep::check`
//! holds every boundary to: a reader is handed topology it did not build, and must refuse what
//! closes edge by edge and still bounds no solid.
//!
//! `check` holds each loop closed, each pcurve on its edge, each loop turning the way its face's
//! sense says, every edge used once each way; and it accepts whatever tolerance was measured. What
//! `sound` holds besides, each refusal witnessed by the face or the edge it is read off:
//!
//! * **Every tolerance small against the geometry it blurs** (`edges`). An edge's measured
//!   tolerance under `THIN` of its own length (a tube wider than it is long has no direction of its
//!   own), and no edge of no length (`geom::tolerance` of the boundary's size, the kernel's own
//!   floor); an edge's two vertices, and any two vertices of one face, standing farther apart than
//!   their tolerances reach together (else which corner a point near both is, is the tolerance's
//!   choice, not the file's); no edge or vertex tolerance past `LOOSE` of the boundary's size (a gap
//!   that wide is drawn, not tolerated). Each is said against the uncertainty the file states,
//!   which is never the bar: real files state 1e-7 mm and carry edges 1e-3 mm off their faces.
//! * **Every face's loops simple and apart, its holes inside its outer loop** (`Drawn`), decided in
//!   its parameters on the pcurves themselves: each use cut at its curve's breaks into pieces
//!   whose middles stray from their chords less than a sixteenth of their length, every two
//!   pieces whose boxes (each grown by twice its sag and its edge's tolerance carried into the
//!   parameters by the surface's speed) meet read against each other, halved until they are apart,
//!   cross with room to spare, or stand within the tolerance of each other. Two pieces meeting at a
//!   corner of the loop (the same vertex, the same place in the parameters: consecutive uses, or a
//!   loop touching itself or another at a vertex, a kernel's pinch) are apart when each lies in
//!   a cone about its chord from the corner (half-angle eight times its sag over its length, what a
//!   parabola's tangents turn through, doubled) and the cones part, or when both lie within the
//!   corner's reach: its tolerance, or its **tongue** where the two uses leave it tangent (an arc
//!   tangent to a line at their vertex, the face between them thinner than the tolerance for a
//!   stretch: `Drawn::tongue`). A crossing is refused; so is a **touch**, two pieces nearer than
//!   their tolerance away from any corner they share: the tolerant boundary's topology is not
//!   decided there, and "never read wrong" says an unprovable face is refused (OCCT's own check
//!   refuses wires intersecting within tolerance away from their vertices). At a corner passed more
//!   than once, the passages' ways in and out must not interleave round it (a figure of eight
//!   through a vertex). A loop wider than its surface's period overlaps itself. Holes are placed by
//!   whole periods beside the outer loop's middle; each hole's point, refined against the loops'
//!   pieces until its winding is read exactly, must be wound once by the others together, as the
//!   outer loop winds it — inside the outer loop, inside no other hole. Seams are two uses a period
//!   apart and poles a degenerate edge's line in the parameters, so neither is a crossing. A face
//!   whose area in its parameters is provably under its tolerance times its perimeter has none.
//! * **One solid outside, its voids inside it** (`shells`): the faces' components by shared edges
//!   are the shells; each shell's signed volume from `props::fluxes`; none of no volume (under the
//!   floor times the size squared). The shell of greatest positive volume, with every positive
//!   shell sharing a vertex with it or with one that does (a kernel's solid pinched along a line,
//!   its edges there held twice by `Brep::unpinched`), is the solid; any other positive shell is a
//!   second solid, refused; none positive, the boundary is inside out. Each negative shell is a
//!   void: one of its first eight vertices read off the solid's shells is located against them
//!   alone (`query::Located::solid_place`, a ray's crossings, exact where the queries prove them):
//!   inside, or refused; none read off them, its place is undecided and refused. A void inside
//!   another (their faces' boxes nesting, then located the same way) is refused.
//!
//! **Not decided**: two faces that are not neighbours crossing each other in space (a global
//! self-intersection), and a void's shell crossing its outer shell where a vertex of it reads
//! inside: no face–face intersection is searched. A cheap necessary condition worth holding next:
//! every face's boxes against the boxes of the faces it shares no edge or vertex with, the pairs
//! whose boxes meet then read edge against face (`query::curve_surface`) — an edge piercing a face
//! it does not bound is a crossing, found by the queries the Booleans already prove. The sag
//! bound a piece is read to is measured at its middle, as `query::Outline` reads its chords, not
//! proved: a curve that wiggles within a piece more than a parabola would is read as its chord.
use super::context::Context;
use super::{Given,Refusal,Why};
use crate::geom::Uv;
use crate::topo::{unwrap,Brep,Coedge,Edge,EdgeCurve,Pcurve};
use crate::util::space::{distance,norm};

/// A boundary that is no solid's: why, the face or the edge of it that witnesses it (by its
/// index in the `Brep`), and what there is to say.
#[derive(Clone,Debug,PartialEq)]
pub struct Unsound { pub why: Why,pub face: Option<usize>,pub edge: Option<usize>,pub msg: String }

impl Unsound {
    /// The refusal of the file this is, its witness the entity `given` read that face or edge
    /// from.
    pub fn refusal(&self,given: &Given) -> Refusal {
        let entity = self.face.and_then(|f| given.faces.get(f)).or(self.edge.and_then(|e| given.edges.get(e))).copied().filter(|&e| e != 0);
        Refusal::new(self.why,entity.or(Some(given.entity)),self.msg.clone())
    }
    fn face(fi: usize,msg: String) -> Unsound { Unsound {why:Why::Unsound,face:Some(fi),edge:None,msg} }
    fn edge(ei: usize,msg: String) -> Unsound { Unsound {why:Why::Unsound,face:None,edge:Some(ei),msg} }
}

/// The most an edge's measured tolerance may be of its length: past half, its tube is wider than
/// it is long, and its ends no longer tell its way.
pub const THIN: f64 = 0.5;
/// The most any tolerance may be of the boundary's size (the diagonal of its box): a gap of a
/// thousandth of the part is a shape the file draws, not a tolerance it keeps.
pub const LOOSE: f64 = 1e-3;
/// How far the sag a piece of a pcurve is read to is trusted: its chord's tube twice as wide.
const SAFE: f64 = 2.;
/// How many halvings any two pieces are read to before the face is refused undecided.
const DEEPEST: u32 = 96;
/// How many readings of pairs of pieces a face may take before it is refused undecided.
const MOST: usize = 1<<22;
/// The joint at a piece's end where it is no corner and no break of its use (one a halving laid).
const NONE: u32 = u32::MAX;

/// Whether `b`, read in `context`, is a solid's boundary.
pub fn sound(b: &Brep,context: &Context) -> Result<(),Unsound> {
    indexed(b)?;
    let size = extent(b);
    let floor = crate::geom::tolerance(size);
    edges(b,context,size,floor)?;
    let faces = crate::util::par::indices(b.faces.len(),|fi| face(b,fi,context,floor));
    for r in faces { r? }
    shells(b,size,floor)
}

/// Every index in range and every number finite, so nothing after reads past an array or loops on
/// a NaN.
fn indexed(b: &Brep) -> Result<(),Unsound> {
    let (nv,ne) = (b.vertices.len(),b.edges.len());
    for (ei,e) in b.edges.iter().enumerate() {
        if e.v.iter().any(|&v| v as usize >= nv) { return Err(Unsound::edge(ei,format!("edge {ei} ends on no vertex of the boundary"))) }
        if !(e.t.iter().all(|t| t.is_finite()) && e.tol.is_finite()) { return Err(Unsound::edge(ei,format!("edge {ei}'s parameters are not finite"))) }
    }
    if let Some(vi) = b.vertices.iter().position(|v| !(v.p.iter().all(|x| x.is_finite()) && v.tol.is_finite())) {
        let ei = b.edges.iter().position(|e| e.v.contains(&(vi as u32)));
        return Err(Unsound {why:Why::Unsound,face:None,edge:ei,msg:format!("vertex {vi} is not finite")})
    }
    for (fi,f) in b.faces.iter().enumerate() {
        if f.loops.iter().flatten().any(|c| c.edge as usize >= ne) { return Err(Unsound::face(fi,format!("face {fi} uses no edge of the boundary"))) }
    }
    Ok(())
}

/// The diagonal of the box about the boundary's vertices and its edges' curves: the size the
/// floor and `LOOSE` are taken against (`Brep::size` reads every face's surface besides, which
/// a bar needs no more than its edges).
fn extent(b: &Brep) -> f64 {
    let mut bx = crate::geom::EMPTY;
    for v in &b.vertices { crate::geom::include(&mut bx,v.p); }
    for e in &b.edges { if let EdgeCurve::Curve(c) = &e.curve { crate::geom::widen(&mut bx,c.bounds(e.t)); } }
    let d = norm(std::array::from_fn(|k| bx.1[k]-bx.0[k]));
    if d.is_finite() { d } else { 0. }
}

/// `x` mm, said against the uncertainty the file states.
fn stated(x: f64,context: &Context) -> String {
    match context.uncertainty {
        Some(u) if u > 0. => format!("{x:e} mm, {:.3e} times the uncertainty the file states ({u:e} mm)",x/u),
        _ => format!("{x:e} mm (the file states no uncertainty)"),
    }
}

/// An edge's length, the sum of its chords at its curve's breaks and sixteen even steps: never
/// more than its length.
fn length(e: &Edge) -> f64 {
    let EdgeCurve::Curve(c) = &e.curve else { return 0. };
    let mut ts: Vec<f64> = (0..=16).map(|k| e.t[0]+(e.t[1]-e.t[0])*k as f64/16.).collect();
    ts.extend(c.breaks(e.t));
    ts.sort_by(f64::total_cmp);
    ts.windows(2).map(|w| distance(c.point(w[0]),c.point(w[1]))).sum()
}

/// Each edge's tolerance against its length and the boundary's size, its length against the
/// floor, and its vertices apart by more than their tolerances; each vertex's tolerance against
/// the boundary's size.
fn edges(b: &Brep,context: &Context,size: f64,floor: f64) -> Result<(),Unsound> {
    let loose = LOOSE*size;
    let found = crate::util::par::indices(b.edges.len(),|ei| -> Result<(),Unsound> {
        let e = &b.edges[ei];
        if matches!(e.curve,EdgeCurve::Degenerate) { return Ok(()) }
        if e.tol > loose {
            return Err(Unsound::edge(ei,format!("edge {ei} stands off its faces by {}: more than {LOOSE:e} of the solid's size {size:e} mm",stated(e.tol,context))))
        }
        let len = length(e);
        if !(len > floor) { return Err(Unsound::edge(ei,format!("edge {ei} ({}) has no length ({len:e} mm)",e.curve.kind()))) }
        if e.tol > THIN*len {
            return Err(Unsound::edge(ei,format!("edge {ei}'s tolerance {} is not small against its length {len:e} mm",stated(e.tol,context))))
        }
        let [a,z] = e.v.map(|v| &b.vertices[v as usize]);
        if e.v[0] != e.v[1] && distance(a.p,z.p) <= a.tol+z.tol+floor {
            return Err(Unsound::edge(ei,format!("edge {ei}'s vertices {} and {} stand {:e} mm apart, within their tolerances ({} and {})",e.v[0],e.v[1],
                distance(a.p,z.p),stated(a.tol,context),stated(z.tol,context))))
        }
        Ok(())
    });
    for r in found { r? }
    for (vi,v) in b.vertices.iter().enumerate() {
        if v.tol > loose {
            let ei = b.edges.iter().position(|e| e.v.contains(&(vi as u32)));
            return Err(Unsound {why:Why::Unsound,face:None,edge:ei,msg:format!("vertex {vi} gathers points {} apart: more than {LOOSE:e} of the solid's size {size:e} mm",
                stated(v.tol,context))})
        }
    }
    Ok(())
}

/// One face held to its rules: its vertices apart, its loops simple, apart and nested as its sense
/// says, and of some area.
fn face(b: &Brep,fi: usize,context: &Context,floor: f64) -> Result<(),Unsound> {
    let f = &b.faces[fi];
    if f.loops.is_empty() || f.loops.iter().any(Vec::is_empty) { return Ok(()) }
    vertices_apart(b,fi,context,floor)?;
    let mut d = Drawn::new(b,fi,floor)?;
    d.wraps()?;
    d.passages()?;
    d.crossings()?;
    d.area()?;
    d.holes()
}

/// No two vertices of face `fi` within each other's tolerances.
fn vertices_apart(b: &Brep,fi: usize,context: &Context,floor: f64) -> Result<(),Unsound> {
    let f = &b.faces[fi];
    let mut vs: Vec<u32> = f.loops.iter().flatten().flat_map(|c| b.edges[c.edge as usize].v).collect();
    vs.sort_unstable();
    vs.dedup();
    let reach = vs.iter().map(|&v| b.vertices[v as usize].tol).fold(0.,f64::max)*2.+floor;
    vs.sort_by(|&x,&y| b.vertices[x as usize].p[0].total_cmp(&b.vertices[y as usize].p[0]).then(x.cmp(&y)));
    for (i,&x) in vs.iter().enumerate() {
        let a = &b.vertices[x as usize];
        for &y in &vs[i+1..] {
            let z = &b.vertices[y as usize];
            if z.p[0]-a.p[0] > reach { break }
            let d = distance(a.p,z.p);
            if d <= a.tol+z.tol+floor {
                return Err(Unsound::face(fi,format!("face {fi}'s vertices {x} and {y} stand {d:e} mm apart, within their tolerances ({} and {})",
                    stated(a.tol,context),stated(z.tol,context))))
            }
        }
    }
    Ok(())
}

/// One use of an edge read in its face's parameters along its own way, `s` from 0 at its start to
/// 1 at its end, moved by whole periods by `shift` (a hole placed beside its outer loop).
struct Run<'a> { c: &'a Coedge,e: &'a Edge,straight: bool,ends: [Uv;2],shift: Uv,tau: f64 }

/// A stretch `s` of a run: its ends' parameters `p`, its middle's, how far that strays from the
/// chord, the surface's greater speed there, and the joints at its ends.
#[derive(Clone,Copy,Debug)]
struct Piece { run: u32,s: [f64;2],p: [Uv;2],mid: Uv,sag: f64,speed: f64,joint: [u32;2] }

impl Piece {
    fn len(&self) -> f64 { dist(self.p[0],self.p[1]) }
}

/// A face's loops drawn in its parameters as pieces of its uses, and the joints between them.
struct Drawn<'a> {
    b: &'a Brep,
    fi: usize,
    runs: Vec<Run<'a>>,
    pieces: Vec<Piece>,
    /// Each loop's pieces.
    loops: Vec<std::ops::Range<usize>>,
    /// Each joint's vertex (`NONE` for a break within a use), and the corner it was merged into.
    joints: Vec<u32>,
    floor: f64,
    /// The face's greatest speed read, its parameters' scale where a piece reads none.
    fastest: f64,
    steps: usize,
    /// Each run's corners: the joint at its start and at its end, and its parameters there.
    corners: Vec<[(u32,Uv);2]>,
    /// The tongues read (`tongue`), by joint and the two runs.
    tongues: std::collections::BTreeMap<(u32,u32,u32),f64>,
}

fn dist(a: Uv,b: Uv) -> f64 { (a[0]-b[0]).hypot(a[1]-b[1]) }
fn cross(o: Uv,a: Uv,b: Uv) -> f64 { (a[0]-o[0])*(b[1]-o[1])-(a[1]-o[1])*(b[0]-o[0]) }
/// How far `p` is from the segment `a b`.
fn to_segment(p: Uv,a: Uv,b: Uv) -> f64 {
    let d = [b[0]-a[0],b[1]-a[1]];
    let l = d[0]*d[0]+d[1]*d[1];
    if !(l > 0.) { return dist(p,a) }
    let t = (((p[0]-a[0])*d[0]+(p[1]-a[1])*d[1])/l).clamp(0.,1.);
    dist(p,[a[0]+t*d[0],a[1]+t*d[1]])
}
/// How far apart the segments `a b` and `c d` are: 0 where they cross.
fn segments(a: Uv,b: Uv,c: Uv,d: Uv) -> f64 {
    let (x,y,z,w) = (cross(a,b,c),cross(a,b,d),cross(c,d,a),cross(c,d,b));
    if ((x > 0. && y < 0.) || (x < 0. && y > 0.)) && ((z > 0. && w < 0.) || (z < 0. && w > 0.)) { return 0. }
    to_segment(a,c,d).min(to_segment(b,c,d)).min(to_segment(c,a,b)).min(to_segment(d,a,b))
}
/// Whether the chords of `p` and `q` cross with every end farther than `m` from the other's line:
/// the curves within `m` of them cross too.
fn crosses(p: &Piece,q: &Piece,m: f64) -> bool {
    let ([a,b],[c,d]) = (p.p,q.p);
    let (lab,lcd) = (dist(a,b),dist(c,d));
    if !(lab > 0. && lcd > 0.) { return false }
    let (x,y,z,w) = (cross(a,b,c)/lab,cross(a,b,d)/lab,cross(c,d,a)/lcd,cross(c,d,b)/lcd);
    x*y < 0. && z*w < 0. && x.abs().min(y.abs()).min(z.abs()).min(w.abs()) > m
}

/// Why a face is refused at a point of its parameters.
fn at_uv(fi: usize,what: &str,uv: Uv) -> Unsound { Unsound::face(fi,format!("face {fi}: {what}, near {:?} in its parameters",uv.map(|x| (x*1e6).round()/1e6))) }

impl<'a> Drawn<'a> {
    fn new(b: &'a Brep,fi: usize,floor: f64) -> Result<Drawn<'a>,Unsound> {
        let f = &b.faces[fi];
        let mut d = Drawn {b,fi,runs:Vec::new(),pieces:Vec::new(),loops:Vec::new(),joints:Vec::new(),floor,fastest:0.,steps:0,corners:Vec::new(),tongues:Default::default()};
        let mut centre = [0.;2];
        for (li,l) in f.loops.iter().enumerate() {
            let start = d.pieces.len();
            // the corners after each use: use k runs from corner k − 1 to corner k
            let corner = d.joints.len() as u32;
            d.joints.extend(l.iter().map(|c| b.ends(c)[1]));
            let n = l.len() as u32;
            let mut shift = [0.;2];
            for (k,c) in l.iter().enumerate() {
                let e = &b.edges[c.edge as usize];
                let straight = c.pcurve.straight(e,&f.surface) || matches!(&c.pcurve,Pcurve::Curve(x) if matches!(**x,crate::geom::Curve::Line {..}));
                let mut run = Run {c,e,straight,ends:[[0.;2];2],shift:[0.;2],tau:floor.max(e.tol)};
                run.ends = [run.raw(0.,b,f),run.raw(1.,b,f)];
                if k == 0 && li > 0 {
                    // a hole placed by whole periods beside the outer loop's middle
                    let w = unwrap(run.ends[0],centre,f.surface.periods());
                    shift = [w[0]-run.ends[0][0],w[1]-run.ends[0][1]];
                }
                run.shift = shift;
                run.ends = run.ends.map(|p| [p[0]+shift[0],p[1]+shift[1]]);
                if !run.ends.iter().flatten().all(|x| x.is_finite()) {
                    return Err(Unsound::face(fi,format!("face {fi}: edge {}'s pcurve reads no parameters at its ends",c.edge)))
                }
                d.runs.push(run);
                let joints = [corner+(k as u32+n-1)%n,corner+k as u32];
                d.lay(d.runs.len()-1,joints)?;
            }
            d.loops.push(start..d.pieces.len());
            if li == 0 {
                let (mut lo,mut hi) = ([f64::INFINITY;2],[f64::NEG_INFINITY;2]);
                for p in &d.pieces { for q in [p.p[0],p.mid] { for k in 0..2 { lo[k] = lo[k].min(q[k]); hi[k] = hi[k].max(q[k]); } } }
                centre = [(lo[0]+hi[0])/2.,(lo[1]+hi[1])/2.];
            }
        }
        d.fastest = d.pieces.iter().map(|p| p.speed).filter(|s| s.is_finite()).fold(0.,f64::max);
        d.merge();
        d.corners = vec![[(NONE,[0.;2]);2];d.runs.len()];
        for (i,p) in d.pieces.iter().enumerate() {
            let r = p.run as usize;
            if i == 0 || d.pieces[i-1].run != p.run { d.corners[r][0] = (p.joint[0],p.p[0]); }
            if i+1 == d.pieces.len() || d.pieces[i+1].run != p.run { d.corners[r][1] = (p.joint[1],p.p[1]); }
        }
        Ok(d)
    }

    /// Run `r`'s parameters at `s`, unshifted.
    fn raw_at(&self,r: usize,s: f64) -> Uv {
        let run = &self.runs[r];
        if run.straight {
            let [a,z] = run.ends;
            return [a[0]+s*(z[0]-a[0]),a[1]+s*(z[1]-a[1])]
        }
        let p = run.raw(s,self.b,&self.b.faces[self.fi]);
        [p[0]+run.shift[0],p[1]+run.shift[1]]
    }

    /// The piece of run `r` over `s`, its ends `p` read already.
    fn piece(&self,r: usize,s: [f64;2],p: [Uv;2],joint: [u32;2],speed: Option<f64>) -> Piece {
        let run = &self.runs[r];
        let (mid,sag) = if run.straight { ([(p[0][0]+p[1][0])/2.,(p[0][1]+p[1][1])/2.],0.) } else {
            let m = self.raw_at(r,(s[0]+s[1])/2.);
            (m,to_segment(m,p[0],p[1]))
        };
        let speed = speed.unwrap_or_else(|| {
            let (_,su,sv) = self.b.faces[self.fi].surface.d1(mid);
            let v = norm(su).max(norm(sv));
            if v.is_finite() { v } else { 0. }
        });
        // (a NaN read inside a piece is a sag no halving settles)
        Piece {run:r as u32,s,p,mid,sag:if sag.is_finite() && mid.iter().all(|x| x.is_finite()) { sag } else { f64::INFINITY },speed,joint}
    }

    /// The piece's two halves, each its end's joint and none between.
    fn halves(&self,p: &Piece) -> [Piece;2] {
        let m = (p.s[0]+p.s[1])/2.;
        [self.piece(p.run as usize,[p.s[0],m],[p.p[0],p.mid],[p.joint[0],NONE],Some(p.speed)),
         self.piece(p.run as usize,[m,p.s[1]],[p.mid,p.p[1]],[NONE,p.joint[1]],Some(p.speed))]
    }

    /// Run `r` laid as pieces from joint `joints[0]` to `joints[1]`: cut at its curve's breaks, at
    /// least twice where it is curved, each halved until its sag is under a sixteenth of its chord.
    fn lay(&mut self,r: usize,joints: [u32;2]) -> Result<(),Unsound> {
        let run = &self.runs[r];
        let e = run.e;
        let mut cuts = vec![0.,1.];
        if !run.straight {
            let mut breaks = if let EdgeCurve::Curve(c) = &e.curve { c.breaks(e.t) } else { vec![] };
            if let Pcurve::Curve(pc) = &run.c.pcurve { breaks.extend(pc.breaks(e.t)); }
            let span = e.t[1]-e.t[0];
            cuts.extend(breaks.into_iter().map(|t| { let s = (t-e.t[0])/span; if run.c.reversed { 1.-s } else { s } }).filter(|&s| s > 0. && s < 1.));
            cuts.sort_by(f64::total_cmp);
            cuts.dedup();
            if cuts.len() == 2 { cuts.insert(1,0.5); }
        }
        let ends = run.ends;
        let at: Vec<Uv> = cuts.iter().enumerate().map(|(k,&s)| if k == 0 { ends[0] } else if k == cuts.len()-1 { ends[1] } else { self.raw_at(r,s) }).collect();
        let mut stack: Vec<(Piece,u32)> = (0..cuts.len()-1).rev().map(|k| (self.piece(r,[cuts[k],cuts[k+1]],[at[k],at[k+1]],[NONE;2],None),0)).collect();
        let mut laid: Vec<Piece> = Vec::new();
        while let Some((p,depth)) = stack.pop() {
            if p.sag > p.len()/16. && depth < 8 {
                let [a,z] = self.halves(&p);
                stack.push((z,depth+1));
                stack.push((a,depth+1));
                continue
            }
            if !p.p.iter().flatten().all(|x| x.is_finite()) {
                return Err(at_uv(self.fi,&format!("edge {}'s pcurve reads no parameters",self.runs[r].c.edge),p.p[0]))
            }
            laid.push(p);
        }
        // the use's ends its corners, each break between two pieces a joint of their own
        let n = laid.len();
        for k in 0..n {
            laid[k].joint[0] = if k == 0 { joints[0] } else { laid[k-1].joint[1] };
            laid[k].joint[1] = if k == n-1 { joints[1] } else { self.joint(NONE) };
        }
        self.pieces.extend(laid);
        Ok(())
    }

    /// A new joint at `vertex`.
    fn joint(&mut self,vertex: u32) -> u32 { self.joints.push(vertex); (self.joints.len()-1) as u32 }

    /// Corners at one vertex and one place in the parameters made one joint: a loop touching itself
    /// or another there (a pinch, a hole meeting its outer loop at a vertex).
    fn merge(&mut self) {
        // each corner's place: the end of the piece arriving there
        let mut corners: Vec<(u32,u32,Uv)> = Vec::new();
        for p in &self.pieces {
            let j = p.joint[1];
            if self.joints[j as usize] != NONE && !corners.iter().any(|c| c.1 == j) { corners.push((self.joints[j as usize],j,p.p[1])); }
        }
        corners.sort_by(|a,b| a.0.cmp(&b.0).then(a.1.cmp(&b.1)));
        let mut alias: Vec<u32> = (0..self.joints.len() as u32).collect();
        for i in 0..corners.len() {
            for k in i+1..corners.len() {
                if corners[k].0 != corners[i].0 { break }
                let (a,z) = (corners[i].2,corners[k].2);
                if (a[0]-z[0]).abs().max((a[1]-z[1]).abs()) <= 1e-3 && alias[corners[k].1 as usize] == corners[k].1 {
                    alias[corners[k].1 as usize] = alias[corners[i].1 as usize];
                }
            }
        }
        for p in &mut self.pieces { p.joint = p.joint.map(|j| alias[j as usize]); }
    }

    /// The tolerance of pieces `p` and `q` in the face's parameters: the larger of their edges'
    /// (or the floor) over the surface's greater speed there.
    fn tol(&self,p: &Piece,q: &Piece) -> f64 {
        let tau = self.runs[p.run as usize].tau.max(self.runs[q.run as usize].tau);
        let speed = p.speed.max(q.speed);
        tau/if speed > 0. { speed } else if self.fastest > 0. { self.fastest } else { 1. }
    }

    /// No loop reaching farther along a period than the period: it would overlap itself.
    fn wraps(&self) -> Result<(),Unsound> {
        let f = &self.b.faces[self.fi];
        for (k,period) in f.surface.periods().into_iter().enumerate() {
            let Some(period) = period else { continue };
            for (li,r) in self.loops.iter().enumerate() {
                let (lo,hi) = self.pieces[r.clone()].iter().flat_map(|p| [p.p[0],p.mid]).fold((f64::INFINITY,f64::NEG_INFINITY),|(lo,hi),q| (lo.min(q[k]),hi.max(q[k])));
                let slack = self.pieces[r.clone()].iter().map(|p| SAFE*p.sag+self.tol(p,p)).fold(0.,f64::max);
                if hi-lo > period+slack {
                    return Err(Unsound::face(self.fi,format!("face {}: loop {li} reaches {:e} along a period of {period:e}: it overlaps itself",self.fi,hi-lo)))
                }
            }
        }
        Ok(())
    }

    /// At a corner the boundary passes more than once, the passages' ways in and out not
    /// interleaving round it: a loop through a vertex touches itself there, never crosses.
    fn passages(&self) -> Result<(),Unsound> {
        // the corners passed more than once
        let mut count: std::collections::BTreeMap<u32,u32> = Default::default();
        for p in &self.pieces { if self.joints[p.joint[1] as usize] != NONE { *count.entry(p.joint[1]).or_default() += 1; } }
        if count.values().all(|&n| n < 2) { return Ok(()) }
        // each passage there: the way it arrives from and the way it leaves to
        let mut by: std::collections::BTreeMap<u32,Vec<(f64,f64,Uv)>> = Default::default();
        for r in &self.loops {
            for i in r.clone() {
                let p = &self.pieces[i];
                let j = p.joint[1];
                if count.get(&j).is_none_or(|&n| n < 2) { continue }
                // the piece leaving that corner: the next of the loop (its first after its last)
                let q = &self.pieces[if i+1 == r.end { r.start } else { i+1 }];
                let x = p.p[1];
                let way = |r: usize,s: f64,from: Uv| { let y = self.raw_at(r,s); (y[1]-from[1]).atan2(y[0]-from[0]) };
                let back = way(p.run as usize,p.s[1]-1e-3*(p.s[1]-p.s[0]),x);
                let on = way(q.run as usize,q.s[0]+1e-3*(q.s[1]-q.s[0]),q.p[0]);
                if back.is_finite() && on.is_finite() { by.entry(j).or_default().push((back,on,x)); }
            }
        }
        for (_,ps) in by.into_iter().filter(|(_,ps)| ps.len() > 1) {
            for (i,a) in ps.iter().enumerate() {
                for b in &ps[i+1..] {
                    // `b`'s ways on either side of `a`'s chord round the corner
                    let inside = |x: f64| { let (lo,hi) = (a.0.min(a.1),a.0.max(a.1)); x > lo && x < hi };
                    let apart = [b.0,b.1].iter().all(|&x| x != a.0 && x != a.1);
                    if apart && inside(b.0) != inside(b.1) { return Err(at_uv(self.fi,"its boundary crosses itself at a vertex it passes twice",a.2)) }
                }
            }
        }
        Ok(())
    }

    /// Every two pieces whose boxes meet read against each other (`resolve`), the boxes swept
    /// along `u`.
    fn crossings(&mut self) -> Result<(),Unsound> {
        let grown = |d: &Drawn,p: &Piece| SAFE*p.sag+d.tol(p,p);
        let mut order: Vec<(f64,usize)> = self.pieces.iter().enumerate().map(|(i,p)| (p.p[0][0].min(p.p[1][0])-grown(self,p),i)).collect();
        order.sort_by(|a,b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
        let mut active: Vec<(f64,usize)> = Vec::new();
        for &(lo,i) in &order {
            active.retain(|&(hi,_)| hi >= lo);
            let p = self.pieces[i];
            let g = grown(self,&p);
            let (vlo,vhi) = (p.p[0][1].min(p.p[1][1])-g,p.p[0][1].max(p.p[1][1])+g);
            for k in 0..active.len() {
                let q = self.pieces[active[k].1];
                let h = grown(self,&q);
                if q.p[0][1].min(q.p[1][1])-h > vhi || q.p[0][1].max(q.p[1][1])+h < vlo { continue }
                let (a,z) = if active[k].1 < i { (q,p) } else { (p,q) };
                self.resolve(a,z)?;
            }
            active.push((p.p[0][0].max(p.p[1][0])+g,i));
        }
        Ok(())
    }

    /// Two pieces read until they are apart, joined at a corner they share and apart beside it,
    /// crossing, or touching within their tolerance.
    fn resolve(&mut self,p: Piece,q: Piece) -> Result<(),Unsound> {
        let mut stack = vec![(p,q,0u32)];
        while let Some((p,q,depth)) = stack.pop() {
            self.steps += 1;
            if self.steps > MOST { return Err(at_uv(self.fi,"its loops were not read apart within the work a face may take",p.p[0])) }
            let tol = self.tol(&p,&q);
            let m = SAFE*(p.sag+q.sag);
            let gap = segments(p.p[0],p.p[1],q.p[0],q.p[1]);
            if gap > m+tol { continue }
            let shared = (0..2).flat_map(|i| (0..2).map(move |k| (i,k))).find(|&(i,k)| p.joint[i] != NONE && p.joint[i] == q.joint[k]);
            if let Some((i,k)) = shared {
                let (x,y) = (p.p[i],q.p[k]);
                let slop = dist(x,y);
                let v = self.joints[p.joint[i] as usize];
                let vt = if v == NONE { 0. } else { self.b.vertices[v as usize].tol };
                let speed = p.speed.max(q.speed);
                let r = tol.max(if speed > 0. { vt/speed } else { 0. })+2.*slop;
                if p.len()+SAFE*p.sag <= r && q.len()+SAFE*q.sag <= r { continue }
                if cones_apart(&p,i,&q,k,slop) { continue }
                let r = r.max(self.tongue(p.joint[i],p.run as usize,q.run as usize));
                if dist(p.p[1-i],x)+SAFE*p.sag <= r && dist(q.p[1-k],y)+SAFE*q.sag <= r { continue }
            } else {
                // beside a corner both pieces' uses reach (one a hair from it, its neighbour the
                // other's): settled once both lie within the corner's tolerance of it
                let corner = self.corner(&p,&q,tol);
                if let Some(r) = corner { if p.len()+SAFE*p.sag <= r && q.len()+SAFE*q.sag <= r { continue } }
                if corner.is_none() {
                    if p.sag.max(q.sag)*SAFE <= tol/8. && gap <= tol*1.125+m {
                        if crosses(&p,&q,0.) { return Err(at_uv(self.fi,&self.say("crosses",&p,&q),p.p[0])) }
                        return Err(at_uv(self.fi,&format!("{}: within the tolerance of each other ({tol:e} in its parameters), away from any vertex they share",
                            self.say("touches",&p,&q)),p.p[0]))
                    }
                    if crosses(&p,&q,m) { return Err(at_uv(self.fi,&self.say("crosses",&p,&q),p.p[0])) }
                }
            }
            if shared.is_some() && crosses(&p,&q,m) { return Err(at_uv(self.fi,&self.say("crosses",&p,&q),p.p[0])) }
            if depth >= DEEPEST {
                return Err(at_uv(self.fi,&format!("{}: not read apart within {DEEPEST} halvings",self.say("may touch",&p,&q)),p.p[0]))
            }
            // the longer halved (or the more bent)
            let key = |x: &Piece| x.len()+4.*SAFE*x.sag;
            if key(&p) >= key(&q) {
                for h in self.halves(&p) { stack.push((h,q,depth+1)); }
            } else {
                for h in self.halves(&q) { stack.push((p,h,depth+1)); }
            }
        }
        Ok(())
    }

    /// The reach of a corner both pieces' uses run to or from, where both pass within it: the
    /// tolerance, the corner's vertex's carried into the parameters, and twice how far apart the
    /// two uses reach it.
    fn corner(&mut self,p: &Piece,q: &Piece,tol: f64) -> Option<f64> {
        let speed = p.speed.max(q.speed);
        let pairs: Vec<[(u32,Uv);2]> = self.corners[p.run as usize].iter().flat_map(|&a| self.corners[q.run as usize].map(|b| [a,b])).collect();
        for [(j,x),(k,y)] in pairs {
            if j == NONE || k != j { continue }
            let v = self.joints[j as usize];
            let vt = if v == NONE || !(speed > 0.) { 0. } else { self.b.vertices[v as usize].tol/speed };
            let r = (tol+vt+2.*dist(x,y)).max(self.tongue(j,p.run as usize,q.run as usize));
            if to_segment(x,p.p[0],p.p[1]) <= r+SAFE*p.sag && to_segment(y,q.p[0],q.p[1]) <= r+SAFE*q.sag { return Some(r) }
        }
        None
    }

    /// **The tongue at a corner**: how far from joint `j` runs `a` and `b`, which both reach it,
    /// stay within their tolerance of each other — read at their points a half, a quarter, … of
    /// the way from the corner, each against the ray the other leaves the corner along (its
    /// chord to its point nearest the corner) — doubled, so that past it they stand four
    /// tolerances apart where they part as two curves leaving one point tangent do, quadratically.
    /// A loop through a cusp (an arc tangent to a line at their vertex, the face between them
    /// thinner than the tolerance for a stretch) touches itself there, and that touch is the
    /// corner's. Two runs within the tolerance of each other all along are no tongue: nothing.
    fn tongue(&mut self,j: u32,a: usize,b: usize) -> f64 {
        let key = (j,a.min(b) as u32,a.max(b) as u32);
        if let Some(&t) = self.tongues.get(&key) { return t }
        let t = self.tongue_of(j,a,b);
        self.tongues.insert(key,t);
        t
    }

    fn tongue_of(&self,j: u32,a: usize,b: usize) -> f64 {
        if a == b { return 0. }
        let end = |r: usize| (0..2).find(|&e| self.corners[r][e].0 == j);
        let (Some(ea),Some(eb)) = (end(a),end(b)) else { return 0. };
        let (x,y) = (self.corners[a][ea].1,self.corners[b][eb].1);
        let (_,su,sv) = self.b.faces[self.fi].surface.d1(x);
        let speed = norm(su).max(norm(sv));
        let speed = if speed > 0. && speed.is_finite() { speed } else if self.fastest > 0. { self.fastest } else { 1. };
        let v = self.joints[j as usize];
        let tau = self.runs[a].tau.max(self.runs[b].tau)+if v == NONE { 0. } else { self.b.vertices[v as usize].tol };
        let tol = tau/speed+2.*dist(x,y);
        // each run's points a half, a quarter, … of the way from the corner, until within a
        // quarter of the tolerance of it
        let points = |r: usize,e: usize,from: Uv| {
            let mut out = Vec::new();
            for l in 0..60 {
                let f = 0.5f64.powi(l);
                let p = self.raw_at(r,if e == 0 { f } else { 1.-f });
                if !p.iter().all(|x| x.is_finite()) { break }
                out.push(p);
                if dist(p,from) <= tol/4. { break }
            }
            out
        };
        let (pa,pb) = (points(a,ea,x),points(b,eb,y));
        let (Some(&da),Some(&db)) = (pa.last(),pb.last()) else { return 0. };
        // how far `p` stands from the ray from `o` through `d`
        let ray = |p: Uv,o: Uv,d: Uv| {
            let w = [d[0]-o[0],d[1]-o[1]];
            let l = w[0].hypot(w[1]);
            let q = [p[0]-o[0],p[1]-o[1]];
            let along = q[0]*w[0]+q[1]*w[1];
            if !(l > 0.) || along <= 0. { dist(p,o) } else { (q[0]*w[1]-q[1]*w[0]).abs()/l }
        };
        let n = pa.len().max(pb.len());
        let sep = |l: usize| ray(pa[l.min(pa.len()-1)],y,db).max(ray(pb[l.min(pb.len()-1)],x,da));
        // the first level from which on every one stands within the tolerance
        let mut first = n;
        while first > 0 && sep(first-1) <= tol { first -= 1; }
        if first == 0 || first == n { return 0. }
        2.*dist(pa[(first-1).min(pa.len()-1)],x).max(dist(pb[(first-1).min(pb.len()-1)],y))
    }

    /// What two pieces do to each other, said of their edges and loops.
    fn say(&self,what: &str,p: &Piece,q: &Piece) -> String {
        let (a,z) = (&self.runs[p.run as usize],&self.runs[q.run as usize]);
        let lp = |x: &Piece| { let i = self.pieces.iter().position(|y| y.run == x.run).unwrap_or(0); self.loops.iter().position(|r| r.contains(&i)).unwrap_or(0) };
        let (la,lz) = (lp(p),lp(q));
        if la == lz { format!("loop {la} {what} itself (edges {} and {})",a.c.edge,z.c.edge) }
        else { format!("loop {la} {what} loop {lz} (edges {} and {})",a.c.edge,z.c.edge) }
    }

    /// A face whose area in its parameters is provably under its tolerance times its perimeter.
    fn area(&self) -> Result<(),Unsound> {
        let (mut area,mut err,mut perimeter) = (0.,0.,0.);
        for r in &self.loops {
            let ps = &self.pieces[r.clone()];
            for p in ps {
                area += (cross([0.;2],p.p[0],p.mid)+cross([0.;2],p.mid,p.p[1]))/2.;
                err += SAFE*p.sag*p.len();
                perimeter += p.len();
            }
            // the loop's corners closed straight
            for (k,p) in ps.iter().enumerate() {
                let q = &ps[(k+1)%ps.len()];
                area += cross([0.;2],p.p[1],q.p[0])/2.;
            }
        }
        let tol = self.floor/if self.fastest > 0. { self.fastest } else { 1. };
        if area.abs()+err <= tol*perimeter/2. {
            return Err(Unsound::face(self.fi,format!("face {} has no area: {area:e} in its parameters, within its tolerance along its boundary",self.fi)))
        }
        Ok(())
    }

    /// Each hole inside the outer loop and inside no other hole: a point of it wound by the other
    /// loops together as the outer loop winds the face's inside.
    fn holes(&mut self) -> Result<(),Unsound> {
        let f = &self.b.faces[self.fi];
        let want = if f.reversed { -1 } else { 1 };
        let boxes: Vec<[Uv;2]> = self.loops.iter().map(|r| {
            let (mut lo,mut hi) = ([f64::INFINITY;2],[f64::NEG_INFINITY;2]);
            for p in &self.pieces[r.clone()] {
                let g = SAFE*p.sag+self.tol(p,p);
                for k in 0..2 { lo[k] = lo[k].min(p.p[0][k].min(p.p[1][k])-g); hi[k] = hi[k].max(p.p[0][k].max(p.p[1][k])+g); }
            }
            [lo,hi]
        }).collect();
        for li in 1..self.loops.len() {
            let r = self.loops[li].clone();
            // a point of the hole: its pieces' ends, then their middles, until one reads
            let tries: Vec<Uv> = self.pieces[r.clone()].iter().map(|p| p.p[0]).chain(self.pieces[r.clone()].iter().map(|p| p.mid)).take(16).collect();
            let mut read = None;
            'tries: for x in tries {
                let mut w = 0;
                for (lk,bx) in boxes.iter().enumerate() {
                    if lk == li || x[0] < bx[0][0] || x[0] > bx[1][0] || x[1] < bx[0][1] || x[1] > bx[1][1] { continue }
                    match self.winding(lk,x) { Some(k) => w += k,None => continue 'tries }
                }
                read = Some((w,x));
                break
            }
            match read {
                Some((w,_)) if w == want => {}
                Some((_,x)) => return Err(at_uv(self.fi,&format!("hole {li} lies outside its outer loop or inside another hole"),x)),
                None => return Err(at_uv(self.fi,&format!("hole {li}'s place against its other loops is not decided"),self.pieces[r.start].p[0])),
            }
        }
        Ok(())
    }

    /// How many times loop `lk` winds about `x`, counter-clockwise: each piece's crossings of the
    /// ray from `x` along `u`, exact where its tube lies wholly beside the ray's line or to one side
    /// of `x`, halved where it holds `x`; none where `x` is within the tolerance of the loop.
    fn winding(&mut self,lk: usize,x: Uv) -> Option<i32> {
        let above = |p: Uv| p[1] > x[1];
        let mut w = 0;
        // the loop closed straight between each piece's end and the next's start (the gaps
        // `check` lets a corner's two ends stand apart by), each read exactly
        let r = self.loops[lk].clone();
        for i in r.clone() {
            let (a,b) = (self.pieces[i].p[1],self.pieces[if i+1 == r.end { r.start } else { i+1 }].p[0]);
            if a == b || (above(a) == above(b) && !(to_segment(x,a,b) <= self.tol(&self.pieces[i],&self.pieces[i]))) { continue }
            if to_segment(x,a,b) <= self.tol(&self.pieces[i],&self.pieces[i]) { return None }
            // where it crosses the level line through `x`
            let u = a[0]+(x[1]-a[1])/(b[1]-a[1])*(b[0]-a[0]);
            if u > x[0] { w += above(b) as i32-above(a) as i32; }
        }
        let mut stack: Vec<(Piece,u32)> = self.pieces[self.loops[lk].clone()].iter().map(|&p| (p,0)).collect();
        while let Some((p,depth)) = stack.pop() {
            let g = SAFE*p.sag+self.tol(&p,&p);
            let (vlo,vhi) = (p.p[0][1].min(p.p[1][1])-g,p.p[0][1].max(p.p[1][1])+g);
            if x[1] < vlo || x[1] > vhi { continue }
            let (ulo,uhi) = (p.p[0][0].min(p.p[1][0])-g,p.p[0][0].max(p.p[1][0])+g);
            if uhi < x[0] { continue }
            if ulo > x[0] { w += above(p.p[1]) as i32-above(p.p[0]) as i32; continue }
            if p.len()+SAFE*p.sag <= g || depth >= 64 { return None }
            for h in self.halves(&p) { stack.push((h,depth+1)); }
        }
        Some(w)
    }
}

impl Run<'_> {
    /// The use's parameters at `s` along its way, unshifted, its ends read at the edge's own ends.
    fn raw(&self,s: f64,b: &Brep,f: &crate::topo::Face) -> Uv {
        let e = self.e;
        let s = if self.c.reversed { 1.-s } else { s };
        let t = if s <= 0. { e.t[0] } else if s >= 1. { e.t[1] } else { e.t[0]+(e.t[1]-e.t[0])*s };
        self.c.pcurve.at(t,e,&f.surface,&b.vertices)
    }
}

/// Whether pieces `p` and `q`, joined at `p`'s end `i` and `q`'s end `k`, each lie in a cone
/// about its chord from there — half-angle eight times its sag over its length, what a parabola's
/// tangents turn through, doubled — and the cones part by more than the two ends stand apart
/// subtends: then they meet only at the corner.
fn cones_apart(p: &Piece,i: usize,q: &Piece,k: usize,slop: f64) -> bool {
    let (a,b) = ([p.p[1-i][0]-p.p[i][0],p.p[1-i][1]-p.p[i][1]],[q.p[1-k][0]-q.p[k][0],q.p[1-k][1]-q.p[k][1]]);
    let (la,lb) = (a[0].hypot(a[1]),b[0].hypot(b[1]));
    if !(la > 0. && lb > 0.) { return false }
    let half = |s: f64,l: f64| (4.*SAFE*s/l).min(std::f64::consts::PI);
    let angle = (a[0]*b[1]-a[1]*b[0]).abs().atan2(a[0]*b[0]+a[1]*b[1]);
    let off = (2.*slop/la.min(lb)).min(std::f64::consts::PI);
    angle > half(p.sag,la)+half(q.sag,lb)+off
}

/// The faces' components by the edges they share, each a shell: one of positive volume, the rest
/// negative and inside it, none inside another.
fn shells(b: &Brep,size: f64,floor: f64) -> Result<(),Unsound> {
    let nf = b.faces.len();
    if nf == 0 { return Ok(()) }
    let mut root: Vec<usize> = (0..nf).collect();
    fn find(r: &mut [usize],mut i: usize) -> usize { while r[i] != i { r[i] = r[r[i]]; i = r[i]; } i }
    let mut user = vec![usize::MAX;b.edges.len()];
    for (fi,f) in b.faces.iter().enumerate() {
        for c in f.loops.iter().flatten() {
            let e = c.edge as usize;
            if user[e] == usize::MAX { user[e] = fi; continue }
            let (x,y) = (find(&mut root,fi),find(&mut root,user[e]));
            if x != y { root[x.max(y)] = x.min(y); }
        }
    }
    let mut shells: Vec<Vec<usize>> = Vec::new();
    let mut label = vec![usize::MAX;nf];
    for fi in 0..nf {
        let r = find(&mut root,fi);
        if label[r] == usize::MAX { label[r] = shells.len(); shells.push(Vec::new()); }
        shells[label[r]].push(fi);
    }
    let flux = crate::props::fluxes(b);
    let volumes: Vec<f64> = shells.iter().map(|s| s.iter().map(|&f| flux[f]).sum::<f64>()/3.).collect();
    for (k,s) in shells.iter().enumerate() {
        // (a slab the floor thick across the whole boundary is no volume)
        if !(volumes[k].abs() > floor*size*size) {
            return Err(Unsound::face(s[0],format!("the shell of face {} bounds no volume ({:e} mm³)",s[0],volumes[k])))
        }
    }
    let mut positive: Vec<usize> = (0..shells.len()).filter(|&k| volumes[k] > 0.).collect();
    if positive.is_empty() {
        let k = (0..shells.len()).max_by(|&x,&y| volumes[x].abs().total_cmp(&volumes[y].abs()).then(y.cmp(&x))).unwrap();
        return Err(Unsound::face(shells[k][0],format!("the shell of face {} is inside out: its volume is {:e} mm³",shells[k][0],volumes[k])))
    }
    positive.sort_by(|&x,&y| volumes[y].total_cmp(&volumes[x]).then(x.cmp(&y)));
    // the positive shells touching the largest at its vertices, and those touching them (a solid
    // pinched along a line, its edges there held twice, `Brep::unpinched`): one solid
    let vertices_of = |k: usize| -> Vec<u32> {
        let mut vs: Vec<u32> = shells[k].iter().flat_map(|&f| b.faces[f].loops.iter().flatten()).flat_map(|c| b.edges[c.edge as usize].v).collect();
        vs.sort_unstable();
        vs.dedup();
        vs
    };
    let mut body = vec![positive[0]];
    let mut held: std::collections::BTreeSet<u32> = vertices_of(positive[0]).into_iter().collect();
    let mut rest: Vec<usize> = positive[1..].to_vec();
    while let Some(at) = rest.iter().position(|&k| vertices_of(k).iter().any(|v| held.contains(v))) {
        let k = rest.remove(at);
        held.extend(vertices_of(k));
        body.push(k);
    }
    if let Some(&k) = rest.iter().min() {
        return Err(Unsound::face(shells[k][0],format!("the shell of face {} bounds a second solid ({:e} mm³) beside the one of face {}",shells[k][0],volumes[k],shells[positive[0]][0])))
    }
    if shells.len() == body.len() { return Ok(()) }
    // shells alone, their edges and vertices numbered again
    let part = |s: &[usize]| Brep {vertices:b.vertices.clone(),edges:b.edges.clone(),faces:s.iter().map(|&f| b.faces[f].clone()).collect()}.compacted();
    let tol_of = |s: &[usize]| s.iter().flat_map(|&f| b.faces[f].loops.iter().flatten()).map(|c| b.edges[c.edge as usize].tol).fold(floor,f64::max);
    // where shell `k`'s vertices stand against `within`: In, Out, or none read off it
    let place = |k: usize,within: &crate::query::Located| -> Option<crate::query::Place> {
        vertices_of(k).iter().take(8).map(|&v| within.solid_place(b.vertices[v as usize].p)).find(|p| *p != crate::query::Place::On)
    };
    let solid: Vec<usize> = body.iter().flat_map(|&k| shells[k].iter().copied()).collect();
    let whole = part(&solid);
    let located = crate::query::Located::new(&whole,tol_of(&solid));
    let voids: Vec<usize> = (0..shells.len()).filter(|&k| !body.contains(&k)).collect();
    for &k in &voids {
        match place(k,&located) {
            Some(crate::query::Place::In) => {}
            Some(_) => return Err(Unsound::face(shells[k][0],format!("the void of face {} lies outside the solid's outer shell",shells[k][0]))),
            None => return Err(Unsound::face(shells[k][0],format!("the void of face {} touches the solid's outer shell: its place is not decided",shells[k][0]))),
        }
    }
    let boxes: Vec<crate::geom::Box3> = shells.iter().enumerate().map(|(k,s)| {
        let mut bx = crate::geom::EMPTY;
        if !body.contains(&k) { for &fi in s { crate::geom::widen(&mut bx,crate::query::face_box(b,fi)); } }
        bx
    }).collect();
    let inside = |a: &crate::geom::Box3,z: &crate::geom::Box3| (0..3).all(|i| a.0[i] >= z.0[i]-floor && a.1[i] <= z.1[i]+floor);
    for &j in &voids {
        let nested: Vec<usize> = voids.iter().copied().filter(|&k| k != j && inside(&boxes[k],&boxes[j])).collect();
        if nested.is_empty() { continue }
        let other = part(&shells[j]);
        let located = crate::query::Located::new(&other,tol_of(&shells[j]));
        for k in nested {
            if place(k,&located) != Some(crate::query::Place::Out) {
                return Err(Unsound::face(shells[k][0],format!("the void of face {} lies inside the void of face {}",shells[k][0],shells[j][0])))
            }
        }
    }
    Ok(())
}
