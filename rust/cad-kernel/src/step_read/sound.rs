//! What a boundary read from a file must be to be a solid's, beyond what `topo::Brep::check`
//! holds every boundary to: a reader is handed topology it did not build, and must refuse what
//! closes edge by edge and still bounds no solid.
//!
//! **A tolerant boundary is known to within its measured tolerances, and that is what such a file
//! means.** Pieces of it that come within their tolerances of each other, an edge shorter than a
//! few of its tolerances, two distinct vertices nearer than their tolerances, a face of no area:
//! the habits of tolerant writers, read (the file says which vertex is which). A refusal is for a
//! boundary that is no solid's by more than its tolerances explain, each witnessed by the face or
//! the edge it is read off:
//!
//! * **No tolerance past `LOOSE` of the boundary's size** (`edges`): a gap that wide is drawn,
//!   not tolerated, and a reading within it says nothing. Said against the uncertainty the file
//!   states, which is reported, never the bar.
//! * **Every face's loops crossing nowhere deeper than their tolerance, its holes inside its outer
//!   loop** (`Drawn`), decided in its parameters on the pcurves themselves: each use cut at its
//!   curve's breaks into pieces whose middles stray from their chords less than a sixteenth of
//!   their length, every two pieces whose boxes (each grown by twice its sag and its edge's
//!   tolerance carried into the parameters by the surface's greater speed) meet read against
//!   each other, halved until they are apart, or within the tolerance of each other. Two pieces
//!   within it touch, which is read; where their chords cross the crossing is followed out along
//!   both loops (`deep`) and refused only where each arc runs on past the other farther than the
//!   tolerance, measured in space, and not within a few tolerances of a corner both reach (the
//!   slop of a tolerant corner, a pinch of two loops at their vertices). Pieces meeting at a
//!   corner of the loop are apart when they leave it in parting cones, or lie within its reach or
//!   its **tongue** (`Drawn::tongue`). At a corner passed more than once, the passages' ways in
//!   and out must not interleave round it (a figure of eight through a vertex), a way shared (an
//!   edge used both ways, an antenna) reading as one. Holes are placed by whole periods beside
//!   the outer loop's middle; each hole's point must be wound once by the others together, as the
//!   outer loop winds it — inside the outer loop, inside no other hole.
//! * **Every use one curve in its face's parameters** (`Drawn::leap`): a use whose parameters
//!   leap — by a whole period (an inverse curve changing branch mid-edge), or along a pole's line
//!   (its end read on another branch of the pole than its curve, where a degenerate edge should
//!   stand) — is refused: what is measured over the face's parameters would skip the leap. A
//!   leap no wider than the slop `check` lets a corner's ends stand apart by is read.
//! * **A face reaching round its surface more than a period overlaps itself only where it meets
//!   its own image a period along** (`Drawn::turns`): a thread's band, a seam that leans.
//! * **Lumps and voids nested by their senses** (`shells`): the faces' components by shared
//!   edges are the shells, each facing out or in as a point stepped off its faces is located
//!   against it (`facing`); a lump inside nothing or a void, a void inside a lump, none crossing
//!   another — however many lumps, as a cut parts a body (`step::write` writes each its own solid,
//!   by this nesting: `nesting`).
//!
//! **Not decided**: two faces that are not neighbours crossing each other in space (a global
//! self-intersection), and two shells crossing where every vertex of each reads on one side of
//! the other: no face–face intersection is searched. The sag bound a piece is read to is
//! measured at its middle, not proved.
use super::context::Context;
use super::{Given,Refusal,Why};
use crate::geom::Uv;
use crate::topo::{unwrap,Brep,Coedge,Edge,EdgeCurve,Pcurve};
use crate::util::space::{distance,norm,segment_distance};
use crate::geom::V;
use crate::plane2::{self,dist};
use crate::util::fmath::Det;

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
    if b.faces.is_empty() { return Err(Unsound {why:Why::Open,face:None,edge:None,msg:"the boundary has no faces: it bounds nothing".into()}) }
    indexed(b)?;
    let size = extent(b);
    let floor = crate::geom::tolerance(size);
    edges(b,context,size)?;
    let faces = crate::util::par::indices(b.faces.len(),|fi| face(b,fi,floor));
    for r in faces { r? }
    shells(b,shells_of(b),size,floor).map(|_| ())
}

/// **How a boundary's shells nest** (`shells`): its faces' components by the edges they share, in
/// the order of their first faces; whether each faces out (a lump) or in (a void); and the shell
/// nearest about each, where one is — a void's the lump it lies in, a lump's the void it lies in.
#[derive(Clone,Debug)]
pub struct Nesting { pub shells: Vec<Vec<usize>>,pub outward: Vec<bool>,pub about: Vec<Option<usize>> }

/// The nesting of `b`'s shells (`Nesting`), `shells` its faces' components (`shells_of`), each
/// judged as `sound` judges it: refused where a shell faces neither way, a void lies outside every
/// lump or inside another void, a lump inside a lump, or two shells cross.
pub fn nesting(b: &Brep,shells: Vec<Vec<usize>>) -> Result<Nesting,Unsound> {
    if b.faces.is_empty() { return Err(Unsound {why:Why::Open,face:None,edge:None,msg:"the boundary has no faces: it bounds nothing".into()}) }
    indexed(b)?;
    let size = extent(b);
    self::shells(b,shells,size,crate::geom::tolerance(size))
}

/// **A boundary's faces' components by the edges they share**, each a shell: numbered as first
/// seen, so the shells stand in the order of their first faces, and each shell's faces in their
/// order. Every edge a face uses must be in range (`indexed`).
pub fn shells_of(b: &Brep) -> Vec<Vec<usize>> {
    let mut joined = crate::util::union_find::UnionFind::new(b.faces.len());
    let mut user = vec![usize::MAX;b.edges.len()];
    for (fi,f) in b.faces.iter().enumerate() {
        for c in f.loops.iter().flatten() {
            let e = c.edge as usize;
            if user[e] == usize::MAX { user[e] = fi } else { joined.union(user[e],fi) }
        }
    }
    let (label,n) = joined.labels();
    let mut shells: Vec<Vec<usize>> = vec![Vec::new();n];
    for (fi,&l) in label.iter().enumerate() { shells[l].push(fi); }
    shells
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

/// Each edge's and each vertex's tolerance against the boundary's size. (An edge's tolerance
/// against its own length, or two vertices within their tolerances of each other, are a tolerant
/// writer's habits, not a solid's defects: the file says which vertex is which.)
fn edges(b: &Brep,context: &Context,size: f64) -> Result<(),Unsound> {
    let loose = LOOSE*size;
    let found = crate::util::par::indices(b.edges.len(),|ei| -> Result<(),Unsound> {
        let e = &b.edges[ei];
        if matches!(e.curve,EdgeCurve::Degenerate) { return Ok(()) }
        if e.tol > loose {
            return Err(Unsound::edge(ei,format!("edge {ei} stands off its faces by {}: more than {LOOSE:e} of the solid's size {size:e} mm",stated(e.tol,context))))
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

/// One face held to its rules: its loops crossing nowhere deeper than their tolerance, nested as
/// its sense says, of some area, apart from their images a period along, and each turning as its
/// sense says — the outer loop about the face's normal, each hole against it (`Brep::turned`, the
/// rule the export's `check` holds a solid to, a sliver its tolerance cannot orient let be). A
/// face whose loop turns the other way states a normal into the material its own edges bound it
/// on: its sense and its boundary contradict each other, and which the file meant is not read
/// (CADGenBench 240's face #92335, its `ADVANCED_FACE` `.F.` on a cylinder whose two edges its
/// neighbours, tangent across both, use as a face turned the other way would).
fn face(b: &Brep,fi: usize,floor: f64) -> Result<(),Unsound> {
    let f = &b.faces[fi];
    if f.loops.is_empty() || f.loops.iter().any(Vec::is_empty) { return Ok(()) }
    let mut d = Drawn::new(b,fi,floor)?;
    d.passages()?;
    d.crossings(None)?;
    d.holes()?;
    d.turns()?;
    for li in 0..f.loops.len() {
        if let Some(area) = b.turned(fi,li,floor) {
            let which = if li == 0 { "its outer loop".to_string() } else { format!("its hole {li}") };
            return Err(Unsound::face(fi,format!("face {fi}: {which} turns against the face's sense ({area:e} enclosed in its \
                parameters): the face's normal points into the material its edges bound it on")))
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
    /// The face's greatest speed read, its parameters' scale where a piece reads none.
    fastest: f64,
    steps: usize,
    /// Each run's corners: the joint at its start and at its end, and its parameters there.
    corners: Vec<[(u32,Uv);2]>,
    /// The tongues read (`tongue`), by joint and the two runs.
    tongues: std::collections::BTreeMap<(u32,u32,u32),f64>,
    /// Each run past the face's own (`turns`): the run it is the image of, and how far along.
    copies: Vec<(u32,Uv)>,
    /// Each loop's corners: the first's joint, and how many.
    rings: Vec<(u32,u32)>,
}

fn cross(o: Uv,a: Uv,b: Uv) -> f64 { plane2::cross(plane2::sub(a,o),plane2::sub(b,o)) }
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
        let mut d = Drawn {b,fi,runs:Vec::new(),pieces:Vec::new(),loops:Vec::new(),joints:Vec::new(),fastest:0.,steps:0,corners:Vec::new(),tongues:Default::default(),copies:Vec::new(),rings:Vec::new()};
        let mut centre = [0.;2];
        for (li,l) in f.loops.iter().enumerate() {
            let start = d.pieces.len();
            // the corners after each use: use k runs from corner k − 1 to corner k
            let corner = d.joints.len() as u32;
            d.joints.extend(l.iter().map(|c| b.ends(c)[1]));
            let n = l.len() as u32;
            d.rings.push((corner,n));
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
        // a piece whose chord spans over half a period, whose middle stands near one end of its
        // chord or which no halving flattened: the use may leap there
        if !self.runs[r].straight {
            let periods = self.b.faces[self.fi].surface.periods();
            let n = laid.len();
            for (i,p) in laid.iter().enumerate() {
                let (d,l) = ([p.p[1][0]-p.p[0][0],p.p[1][1]-p.p[0][1]],p.len());
                let along = if l > 0. { ((p.mid[0]-p.p[0][0])*d[0]+(p.mid[1]-p.p[0][1])*d[1])/(l*l) } else { 0.5 };
                let wide = (0..2).any(|k| periods[k].is_some_and(|t| d[k].abs() > t/2.));
                // (and at the use's ends, where an end read on another branch of a pole than its
                // curve leaps along the pole's line, straight as the piece beside it: the end
                // against a point a billionth of the way in)
                let end = |s: f64,from: f64| dist(self.raw_at(r,s),self.raw_at(r,from+(s-from)*(1.-1e-9))) > self.tol(p,p);
                let ends = (i == 0 && end(p.s[0],p.s[1])) || (i+1 == n && end(p.s[1],p.s[0]));
                if !(wide || ends || p.sag > l/16. || !(along > 0.125 && along < 0.875)) { continue }
                if let Some((s,from,to)) = self.leap(p) {
                    let surface = &self.b.faces[self.fi].surface;
                    // (the edge's tolerance and its vertices': a use's end is read at its
                    // vertex, which may stand off its curve by the vertex's tolerance)
                    let e = self.runs[r].e;
                    let (by,tau) = (dist(from,to),self.runs[r].tau+e.v.iter().map(|&v| self.b.vertices[v as usize].tol).fold(0.,f64::max));
                    let apart = distance(surface.point(from),surface.point(to));
                    let period = (0..2).find(|&k| periods[k].is_some_and(|t| ((to[k]-from[k]).abs()/t-1.).abs() < 1e-3));
                    // **What the leap does to the face's region**: the face is read as the use
                    // closed across the leap by the chord between its two sides (`props` closes a
                    // use's ends so), and a chord whose image in space keeps within twice the
                    // tolerance of the leap's start adds to or takes from the region only points
                    // within that disc — the most a use's feet swinging round a pole it passes
                    // within its tolerance of can (`reach`, the farthest the chord's image goes);
                    // a chord a period long, or across a closed sheet's seam, or whose ends stand
                    // apart in space, adds a strip
                    let reach = self.reach(from,to);
                    let how = if let Some(k) = period { format!("a period of its {}",["u","v"][k]) }
                        else if apart > 2.*tau { format!("{apart:e} mm in space, past twice its tolerance") }
                        else if reach <= 2.*tau { continue }
                        else { format!("its ends {apart:e} mm apart in space, but the parameters between them reach {reach:e} mm from them, \
                            past twice its tolerance {tau:e} mm: across a closed sheet's seam, or round a pole it passes farther from \
                            than that") };
                    return Err(at_uv(self.fi,&format!("edge {}'s curve in its parameters leaps {by:e} ({how}) at {s:.6} of its use's way: it is no one curve there",
                        self.runs[r].c.edge),from))
                }
            }
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

    /// **Where a use leaps**: piece `p`'s half of the longer chord followed down sixty halvings;
    /// a curve that is one curve there spans less and less, a leap (an inverse curve's branch
    /// changing mid-edge, its end read on another branch of a pole than its curve) spans as much
    /// at every scale. The `s` and the parameters either side, where the chord still spans more
    /// than the tolerance at the end.
    fn leap(&self,p: &Piece) -> Option<(f64,Uv,Uv)> {
        let r = p.run as usize;
        let (mut s,mut a,mut z) = (p.s,p.p[0],p.p[1]);
        for _ in 0..60 {
            let m = (s[0]+s[1])/2.;
            if !(m > s[0] && m < s[1]) { break }
            let x = self.raw_at(r,m);
            if !x.iter().all(|v| v.is_finite()) { return None }
            if dist(a,x) >= dist(x,z) { s[1] = m; z = x } else { s[0] = m; a = x }
        }
        (dist(a,z) > self.tol(p,p)).then_some(((s[0]+s[1])/2.,a,z))
    }

    /// **How far the chord from `from` to `to` in the face's parameters reaches in space** from its
    /// start's point: the farthest of its points read at 256 steps, each with half a step at the
    /// speed it is run at there (so a bound on every point of it between). A chord along a pole's
    /// line reaches nothing; a chord across a closed sheet's domain reaches round the sheet.
    fn reach(&self,from: Uv,to: Uv) -> f64 {
        let s = &self.b.faces[self.fi].surface;
        let o = s.point(from);
        let d = [to[0]-from[0],to[1]-from[1]];
        const N: usize = 256;
        (0..=N).map(|j| {
            let x = j as f64/N as f64;
            let (p,su,sv) = s.d1([from[0]+x*d[0],from[1]+x*d[1]]);
            let rate = norm([su[0]*d[0]+sv[0]*d[1],su[1]*d[0]+sv[1]*d[1],su[2]*d[0]+sv[2]*d[1]]);
            distance(p,o)+0.5*rate/N as f64
        }).fold(0.,|m: f64,x| if x.is_finite() { m.max(x) } else { f64::INFINITY })
    }

    /// A new joint at `vertex`.
    fn joint(&mut self,vertex: u32) -> u32 { self.joints.push(vertex); (self.joints.len()-1) as u32 }

    /// Corners at one vertex and one place in the parameters made one joint: a loop touching itself
    /// or another there (a pinch, a hole meeting its outer loop at a vertex).
    fn merge(&mut self) {
        // each corner's place: the end of the piece arriving there
        let mut corners: Vec<(u32,u32,Uv)> = Vec::new();
        let mut seen = vec![false;self.joints.len()];
        for p in &self.pieces {
            let j = p.joint[1];
            if self.joints[j as usize] != NONE && !seen[j as usize] { seen[j as usize] = true; corners.push((self.joints[j as usize],j,p.p[1])); }
        }
        // two corners one use apart along a loop (an edge closing on its vertex, a pole's
        // degenerate edge) are two places of it, never a loop touching itself
        let adjacent = |a: u32,z: u32| self.rings.iter().any(|&(base,n)| {
            let (a,z) = (a.wrapping_sub(base),z.wrapping_sub(base));
            a < n && z < n && ((a+1)%n == z || (z+1)%n == a)
        });
        corners.sort_by(|a,b| a.0.cmp(&b.0).then(a.1.cmp(&b.1)));
        let mut alias: Vec<u32> = (0..self.joints.len() as u32).collect();
        for i in 0..corners.len() {
            for k in i+1..corners.len() {
                if corners[k].0 != corners[i].0 { break }
                let (a,z) = (corners[i].2,corners[k].2);
                if (a[0]-z[0]).abs().max((a[1]-z[1]).abs()) <= 1e-3 && alias[corners[k].1 as usize] == corners[k].1 && !adjacent(corners[i].1,corners[k].1) {
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

    /// **A face reaching farther along a period than the period** (a thread's band wound round
    /// its cylinder, a helix's strip) overlaps itself only where its boundary meets its own
    /// image a whole number of periods along: those images are laid beside it (`copies`) and read
    /// against it as its loops are against each other (`crossings`), a crossing deeper than the
    /// tolerance refused. (A region can never lie inside its own translate, so a boundary apart
    /// from its images is a face that does not overlap itself.)
    fn turns(&mut self) -> Result<(),Unsound> {
        let periods = self.b.faces[self.fi].surface.periods();
        let mut reach = [0usize;2];
        for (k,period) in periods.into_iter().enumerate() {
            let Some(period) = period else { continue };
            let (lo,hi) = self.pieces.iter().flat_map(|p| [p.p[0],p.mid]).fold((f64::INFINITY,f64::NEG_INFINITY),|(lo,hi),q| (lo.min(q[k]),hi.max(q[k])));
            let slack = self.pieces.iter().map(|p| SAFE*p.sag+self.tol(p,p)).fold(0.,f64::max);
            let span = hi-lo+slack;
            if span > period {
                // (a loop reaching a thousand periods is no face read right: said so, not read)
                if !(span < 1e3*period) {
                    return Err(Unsound::face(self.fi,format!("face {}: its boundary reaches {:e} along a period of {period:e}",self.fi,hi-lo)))
                }
                reach[k] = (span/period).ceil() as usize;
            }
        }
        if reach == [0,0] { return Ok(()) }
        let p = periods.map(|x| x.unwrap_or(0.));
        let (ku,kv) = (reach[0] as i64,reach[1] as i64);
        let shifts: Vec<Uv> = (-ku..=ku).flat_map(|i| (-kv..=kv).map(move |j| (i,j))).filter(|&(i,j)| i > 0 || (i == 0 && j > 0))
            .map(|(i,j)| [i as f64*p[0],j as f64*p[1]]).collect();
        let n = self.pieces.len();
        let runs = self.runs.len();
        for d in &shifts {
            let at = self.runs.len() as u32;
            for r in 0..runs {
                let run = &self.runs[r];
                let moved = Run {c:run.c,e:run.e,straight:run.straight,ends:run.ends.map(|q| [q[0]+d[0],q[1]+d[1]]),shift:[run.shift[0]+d[0],run.shift[1]+d[1]],tau:run.tau};
                self.runs.push(moved);
                self.copies.push((r as u32,*d));
                self.corners.push([(NONE,[0.;2]);2]);
            }
            for i in 0..n {
                let q = self.pieces[i];
                let m = |x: Uv| [x[0]+d[0],x[1]+d[1]];
                self.pieces.push(Piece {run:at+q.run,p:q.p.map(m),mid:m(q.mid),joint:[NONE;2],..q});
            }
        }
        // each image read against the face itself, never against another image
        self.crossings(Some(n))?;
        // and, its boundary nowhere crossing the face's, covering no point of the surface
        // twice: beside each run's middle, of the face and of each image, the face's loops and
        // all their images together wind a point as the face's inside is wound once, or not at
        // all (a hole across the seam is wound -1 a period along, where the face's outer loop
        // winds the same point of the surface +1); else two of them overlap, their boundaries
        // meeting along a stretch, never crossing
        let want = if self.b.faces[self.fi].reversed { -1 } else { 1 };
        let all: Vec<Uv> = std::iter::once([0.;2]).chain(shifts.iter().flat_map(|d| [*d,[-d[0],-d[1]]])).collect();
        let beside: Vec<(Uv,Uv)> = (0..self.pieces.len()).filter(|&i| i+1 == self.pieces.len() || self.pieces[i+1].run != self.pieces[i].run).map(|i| {
            let p = &self.pieces[i];
            let (d,l) = ([p.p[1][0]-p.p[0][0],p.p[1][1]-p.p[0][1]],p.len());
            let e = 4.*self.tol(p,p)+SAFE*p.sag;
            let nrm = if l > 0. { [-d[1]/l*e,d[0]/l*e] } else { [0.;2] };
            (p.mid,nrm)
        }).collect();
        for (x,nrm) in beside {
            for y in [[x[0]+nrm[0],x[1]+nrm[1]],[x[0]-nrm[0],x[1]-nrm[1]]] {
                let mut w = 0;
                let mut read = true;
                'shifts: for d in &all {
                    let z = [y[0]-d[0],y[1]-d[1]];
                    for lk in 0..self.loops.len() {
                        match self.winding(lk,z) { Some(k) => w += k,None => { read = false; break 'shifts } }
                    }
                }
                if read && w != 0 && w != want {
                    return Err(at_uv(self.fi,&format!("it covers a point of its surface {} times: its turns a period apart overlap",w.abs()),y))
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
                let way = |r: usize,s: f64,from: Uv| { let y = self.raw_at(r,s); (y[1]-from[1]).datan2(y[0]-from[0]) };
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
                    // (a way shared — an edge used both ways through the corner, an antenna into
                    // the face — read alike to a millionth of a radian: no interleaving there)
                    let same = |x: f64,y: f64| { let d = (x-y).rem_euclid(std::f64::consts::TAU); d.min(std::f64::consts::TAU-d) <= 1e-6 };
                    let apart = [b.0,b.1].iter().all(|&x| !same(x,a.0) && !same(x,a.1));
                    if apart && inside(b.0) != inside(b.1) { return Err(at_uv(self.fi,"its boundary crosses itself at a vertex it passes twice",a.2)) }
                }
            }
        }
        Ok(())
    }

    /// Every two pieces whose boxes meet read against each other (`resolve`), the boxes swept
    /// along `u`.
    /// With `images`, only pieces before it against pieces from it on (`turns`).
    fn crossings(&mut self,images: Option<usize>) -> Result<(),Unsound> {
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
                if images.is_some_and(|n| (i < n) == (active[k].1 < n)) { continue }
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
                        // within the tolerance of each other: a touch, which a tolerant boundary
                        // makes; refused only where it is a crossing reaching past the tolerance
                        // on either side (`deep`)
                        if crosses(&p,&q,0.) { self.crossed(&p,&q,tol)? }
                        continue
                    }
                    if crosses(&p,&q,m+tol) { self.crossed(&p,&q,tol)? }
                }
            }
            if shared.is_some() && crosses(&p,&q,m+tol) { self.crossed(&p,&q,tol)? }
            // (a piece no halving shortens further whose chord is one point in space — a use's
            // parameters at a pole, where the collapsed one is noise within a hair of it — meets
            // the other only at that point)
            let stuck = |x: &Piece| !(x.s[1]-x.s[0] > 1e-12) && self.collapsed(x);
            if stuck(&p) || stuck(&q) { continue }
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

    /// Whether piece `p`'s chord in the face's parameters is one point in space, within its
    /// tolerance: read at its ends, its quarters and its middle.
    fn collapsed(&self,p: &Piece) -> bool {
        let surface = &self.b.faces[self.fi].surface;
        let tau = self.runs[p.run as usize].tau;
        let o = surface.point(p.p[0]);
        [0.25,0.5,0.75,1.].iter().all(|&l| distance(surface.point([p.p[0][0]+l*(p.p[1][0]-p.p[0][0]),p.p[0][1]+l*(p.p[1][1]-p.p[0][1])]),o) <= tau)
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
            let l = w[0].dhypot(w[1]);
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

    /// Two pieces whose chords cross refused where they cross deeper than their tolerance
    /// (`deep`).
    /// A crossing within a few tolerances of a vertex both pieces' uses reach (their ends there as
    /// far apart as `check` lets a corner's ends stand, its vertex's tolerance besides) is the
    /// corner's slop, not a crossing: a tolerant corner's two uses run a hair past each other.
    fn crossed(&self,p: &Piece,q: &Piece,tol: f64) -> Result<(),Unsound> {
        let ([a,b],[c,d]) = (p.p,q.p);
        let (x,y) = (cross(c,d,a),cross(c,d,b));
        let l = if x != y { x/(x-y) } else { 0.5 };
        let at = [a[0]+l*(b[0]-a[0]),a[1]+l*(b[1]-a[1])];
        let speed = p.speed.max(q.speed);
        for &(j,xp) in &self.corners[p.run as usize] {
            for &(k,xq) in &self.corners[q.run as usize] {
                if j == NONE || k == NONE { continue }
                let (v,w) = (self.joints[j as usize],self.joints[k as usize]);
                if v == NONE || w == NONE { continue }
                // (one vertex, or two standing where the loops pinch: within their tolerances)
                let vt = if speed > 0. { (self.b.vertices[v as usize].tol+if v == w { 0. } else { self.b.vertices[w as usize].tol })/speed } else { 0. };
                let reach = 4.*(tol+vt);
                if v != w && dist(xp,xq) > reach { continue }
                if dist(at,xp).min(dist(at,xq)) <= reach+4.*dist(xp,xq) { return Ok(()) }
            }
        }
        match self.deep(p,q,tol) {
            Some((r,by)) => Err(at_uv(self.fi,&format!("{}: they cross, each running on past the other farther than their tolerance ({by:e} mm) within {r:e} of the crossing in its parameters",
                self.say("crosses",p,q)),p.p[0])),
            None => Ok(()),
        }
    }

    /// **Whether two pieces whose chords cross within their tolerance cross deeper than it**: the
    /// radius about the crossing at which it is seen to, if any. Each loop is followed both ways
    /// from the crossing to the radius `r` (four tolerances, doubled up to ten times), as the
    /// polyline of its pieces' ends: once each arc's two ends stand farther from the other arc than
    /// the tolerance (and the polylines' sag), the arcs cross deeper than the tolerance exactly
    /// where their polylines cross an odd number of times — a curve passing through the other —
    /// and only touch where it is even. Arcs never so far apart run within the tolerance of each
    /// other: a touch. Two points of one loop nearer along it than four tolerances are its own
    /// neighbourhood (a pcurve wiggling within its tolerance), never a crossing; a loop's arcs
    /// are followed no farther than half the way between them, so they never overlap.
    fn deep(&self,p: &Piece,q: &Piece,tol: f64) -> Option<(f64,f64)> {
        let ([a,b],[c,d]) = (p.p,q.p);
        let den = cross([0.;2],[b[0]-a[0],b[1]-a[1]],[d[0]-c[0],d[1]-c[1]]);
        if !(den != 0.) { return None }
        let lp = cross(c,d,a)/(cross(c,d,a)-cross(c,d,b));
        let lq = cross(a,b,c)/(cross(a,b,c)-cross(a,b,d));
        let ((x,dx),(y,dy)) = (self.locate(p,lp)?,self.locate(q,lq)?);
        let tau = self.runs[p.run as usize].tau.max(self.runs[q.run as usize].tau);
        let speed = p.speed.max(q.speed);
        let mut cap = f64::INFINITY;
        if x.0 == y.0 && dx == dy {
            let (at,whole) = self.along(x.0);
            let (u,w) = (self.position(&x,&at),self.position(&y,&at));
            let sep = (u-w).abs().min(whole-(u-w).abs());
            if sep <= 4.*tol { return None }
            cap = sep/2.;
        }
        let mut r = 4.*tol;
        for _ in 0..=10 {
            let r_ = r.min(cap);
            let (Some((pa,ma)),Some((pb,mb))) = (self.arc(&x,r_),self.arc(&y,r_)) else { return None };
            let moved = |a: Vec<Uv>,d: Uv| -> Vec<Uv> { a.into_iter().map(|q| [q[0]+d[0],q[1]+d[1]]).collect() };
            // (the second arc moved a millionth of the radius aside: both pass through where they
            // were found to cross, where a count of their segments' crossings would read none,
            // and a move far within the tolerance changes no crossing of arcs whose ends stand
            // farther apart than it)
            let (pa,pb) = (moved(pa,dx),moved(pb,[dy[0]+0.6e-6*r_,dy[1]+0.8e-6*r_]));
            // (how far apart, read in space: a tolerance is a distance there, and a surface's two
            // speeds may differ by any factor)
            let far = tau+ma.max(mb)*speed;
            let surface = &self.b.faces[self.fi].surface;
            let on = |a: &[Uv]| -> Vec<V> { a.iter().map(|&q| surface.point(q)).collect() };
            let (sa,sb) = (on(&pa),on(&pb));
            let off = |e: V,poly: &[V]| poly.windows(2).map(|s| segment_distance(e,s[0],s[1])).fold(f64::INFINITY,f64::min);
            let ends = |a: &[V]| [a[0],a[a.len()-1]];
            let by = ends(&sa).iter().map(|&e| off(e,&sb)).chain(ends(&sb).iter().map(|&e| off(e,&sa))).fold(f64::INFINITY,f64::min);
            if by > far {
                let mut n = 0;
                for s in pa.windows(2) {
                    for t in pb.windows(2) {
                        let (x,y,z,w) = (cross(s[0],s[1],t[0]),cross(s[0],s[1],t[1]),cross(t[0],t[1],s[0]),cross(t[0],t[1],s[1]));
                        if x*y < 0. && z*w < 0. { n += 1; }
                    }
                }
                return if n%2 == 1 { Some((r_,by)) } else { None }
            }
            if r_ >= cap { return None }
            r *= 2.;
        }
        None
    }

    /// Where along its chord `l` of the way piece `p` is: its loop, the index there of the piece
    /// laid that holds it, and its run's `s` — and, for a piece of an image (`turns`), how far
    /// the image is moved from the face's own run.
    fn locate(&self,p: &Piece,l: f64) -> Option<((usize,usize,f64),Uv)> {
        let s = p.s[0]+l.clamp(0.,1.)*(p.s[1]-p.s[0]);
        let own = self.runs.len()-self.copies.len();
        let (run,d) = if (p.run as usize) < own { (p.run,[0.;2]) } else { *self.copies.get(p.run as usize-own)? };
        self.loops.iter().enumerate().find_map(|(li,r)| self.pieces[r.clone()].iter().position(|x| x.run == run && x.s[0] <= s && s <= x.s[1]).map(|k| ((li,k,s),d)))
    }

    /// Loop `li`'s pieces' starts along it (their chords' lengths and the gaps between them
    /// summed), and its whole length.
    fn along(&self,li: usize) -> (Vec<f64>,f64) {
        let ps = &self.pieces[self.loops[li].clone()];
        let mut at = Vec::with_capacity(ps.len());
        let mut sum = 0.;
        for (k,p) in ps.iter().enumerate() {
            at.push(sum);
            sum += p.len()+dist(p.p[1],ps[(k+1)%ps.len()].p[0]);
        }
        (at,sum)
    }

    fn position(&self,x: &(usize,usize,f64),at: &[f64]) -> f64 {
        let p = &self.pieces[self.loops[x.0].start+x.1];
        let l = if p.s[1] > p.s[0] { (x.2-p.s[0])/(p.s[1]-p.s[0]) } else { 0. };
        at[x.1]+l*p.len()
    }

    /// The arc of the loop through `x` (`locate`) both ways to `r` from it, as a polyline, and the
    /// most its pieces stray from their chords; none where the loop stays within `r` all round.
    fn arc(&self,x: &(usize,usize,f64),r: f64) -> Option<(Vec<Uv>,f64)> {
        let range = self.loops[x.0].clone();
        let n = range.len();
        let start = self.pieces[range.start+x.1];
        let o = self.raw_at(start.run as usize,x.2);
        let mut sag = 0f64;
        // the stretch of run `run` over `a`..`z` laid as its chord: how far its middle strays
        let mut stray = |run: usize,a: f64,z: f64,pa: Uv,pz: Uv| sag = sag.max(SAFE*to_segment(self.raw_at(run,(a+z)/2.),pa,pz));
        let mut ways: [Vec<Uv>;2] = [Vec::new(),Vec::new()];
        for (w,way) in ways.iter_mut().enumerate() {
            let (mut k,mut from,mut at) = (x.1,x.2,o);
            let mut reached = false;
            for _ in 0..=n {
                let p = self.pieces[range.start+k];
                let run = p.run as usize;
                let (end,to) = if w == 0 { (p.p[1],p.s[1]) } else { (p.p[0],p.s[0]) };
                if dist(end,o) >= r {
                    // the first place the piece reaches `r`, between `from` (within it) and its end
                    let (mut lo,mut hi) = (from,to);
                    for _ in 0..60 {
                        let m = (lo+hi)/2.;
                        if !(m != lo && m != hi) { break }
                        if dist(self.raw_at(run,m),o) < r { lo = m } else { hi = m }
                    }
                    let y = if hi == to { end } else { self.raw_at(run,hi) };
                    stray(run,from,hi,at,y);
                    way.push(y);
                    reached = true;
                    break
                }
                stray(run,from,to,at,end);
                way.push(end);
                k = if w == 0 { (k+1)%n } else { (k+n-1)%n };
                let next = self.pieces[range.start+k];
                let (gap,s) = if w == 0 { (next.p[0],next.s[0]) } else { (next.p[1],next.s[1]) };
                way.push(gap);
                if dist(gap,o) >= r { reached = true; break }
                (from,at) = (s,gap);
            }
            if !reached { return None }
        }
        let [fore,back] = ways;
        Some((back.into_iter().rev().chain(std::iter::once(o)).chain(fore).collect(),sag))
    }

    /// What two pieces do to each other, said of their edges and loops.
    fn say(&self,what: &str,p: &Piece,q: &Piece) -> String {
        let (a,z) = (&self.runs[p.run as usize],&self.runs[q.run as usize]);
        let lp = |x: &Piece| { let i = self.pieces.iter().position(|y| y.run == x.run).unwrap_or(0); self.loops.iter().position(|r| r.contains(&i)).unwrap_or(0) };
        let (la,lz) = (lp(p),lp(q));
        if la == lz { format!("loop {la} {what} itself (edges {} and {})",a.c.edge,z.c.edge) }
        else { format!("loop {la} {what} loop {lz} (edges {} and {})",a.c.edge,z.c.edge) }
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
            if a == b { continue }
            if to_segment(x,a,b) <= self.tol(&self.pieces[i],&self.pieces[i]) { return None }
            if above(a) == above(b) { continue }
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
    let (la,lb) = (a[0].dhypot(a[1]),b[0].dhypot(b[1]));
    if !(la > 0. && lb > 0.) { return false }
    let half = |s: f64,l: f64| (4.*SAFE*s/l).min(std::f64::consts::PI);
    let angle = (a[0]*b[1]-a[1]*b[0]).abs().datan2(a[0]*b[0]+a[1]*b[1]);
    let off = (2.*slop/la.min(lb)).min(std::f64::consts::PI);
    angle > half(p.sag,la)+half(q.sag,lb)+off
}

/// The faces' components by the edges they share (`shells_of`), each a shell: each facing one
/// way, out or in (`facing`); the outward ones (lumps) and the inward ones (voids) nested so that
/// the shell nearest about each is of the other sense — a lump inside nothing or inside a void, a
/// void inside a lump — and no two crossing. However many lumps: a cut can part a body, and a
/// file written before #993 stage 2 put all its lumps in one shell's faces.
fn shells(b: &Brep,shells: Vec<Vec<usize>>,size: f64,floor: f64) -> Result<Nesting,Unsound> {
    use crate::query::Place;
    let tol_of = |s: &[usize]| s.iter().flat_map(|&f| b.faces[f].loops.iter().flatten()).map(|c| b.edges[c.edge as usize].tol).fold(floor,f64::max);
    // each shell alone, and where a point stands against it
    let parts: Vec<Brep> = if shells.len() == 1 { Vec::new() } else {
        shells.iter().map(|s| part(b,s)).collect()
    };
    let located: Vec<crate::query::Located> = if shells.len() == 1 { vec![crate::query::Located::new(b,tol_of(&shells[0]))] }
        else { parts.iter().zip(&shells).map(|(p,s)| crate::query::Located::new(p,tol_of(s))).collect() };
    let outward: Vec<bool> = shells.iter().zip(&located).map(|(s,l)| facing(b,s,l,tol_of(s).max(1e-6*size),size,floor)).collect::<Result<_,_>>()?;
    if outward.iter().all(|&o| !o) {
        return Err(Unsound::face(shells[0][0],format!("the shell of face {} is inside out: a point just outside each face it is read at lies inside it",shells[0][0])))
    }
    if shells.len() == 1 { return Ok(Nesting {shells,outward,about:vec![None]}) }
    let vertices_of = |k: usize| -> Vec<u32> {
        let mut vs: Vec<u32> = shells[k].iter().flat_map(|&f| b.faces[f].loops.iter().flatten()).flat_map(|c| b.edges[c.edge as usize].v).collect();
        vs.sort_unstable();
        vs.dedup();
        vs
    };
    let boxes: Vec<crate::geom::Box3> = shells.iter().map(|s| {
        let mut bx = crate::geom::EMPTY;
        for &fi in s { crate::geom::widen(&mut bx,crate::query::face_box(b,fi)); }
        bx
    }).collect();
    let grow = |k: usize| tol_of(&shells[k]).max(floor);
    let within = |x: usize,y: usize| { let g = grow(y); (0..3).all(|i| boxes[x].0[i] >= boxes[y].0[i]-g && boxes[x].1[i] <= boxes[y].1[i]+g) };
    let meet = |x: usize,y: usize| { let g = grow(x)+grow(y); (0..3).all(|i| boxes[x].0[i] <= boxes[y].1[i]+g && boxes[y].0[i] <= boxes[x].1[i]+g) };
    let vs: Vec<Vec<u32>> = (0..shells.len()).map(vertices_of).collect();
    // `inside[x]`: the shells `x` lies inside
    let mut inside: Vec<Vec<usize>> = vec![Vec::new();shells.len()];
    for y in 0..shells.len() {
        for x in (0..shells.len()).filter(|&x| x != y && meet(x,y)) {
            // where `x`'s vertices stand against `y`: each read one of its own, the shared ones
            // (a pinch) on it; read alike, or the two cross
            let mut read: Option<Place> = None;
            for &v in vs[x].iter().filter(|v| vs[y].binary_search(v).is_err()).take(16) {
                let at = located[y].solid_place(b.vertices[v as usize].p);
                if at == Place::On { continue }
                match read {
                    None => read = Some(at),
                    Some(was) if was != at => {
                        return Err(Unsound::face(shells[x][0],format!("the shell of face {} crosses the shell of face {}: its vertex {v} is {}, another {}",
                            shells[x][0],shells[y][0],if at == Place::In { "inside" } else { "outside" },if at == Place::In { "outside" } else { "inside" })))
                    }
                    Some(_) => {}
                }
            }
            match read {
                Some(Place::In) => inside[x].push(y),
                Some(_) => {}
                None if within(x,y) => return Err(Unsound::face(shells[x][0],format!("the shell of face {} touches the shell of face {} all round: its place is not decided",shells[x][0],shells[y][0]))),
                None => {}
            }
        }
    }
    let mut abouts = Vec::with_capacity(shells.len());
    for x in 0..shells.len() {
        // the shell nearest about `x`: the one of those it lies inside that lies inside all the
        // others
        let about = inside[x].iter().copied().find(|&y| inside[x].iter().all(|&z| z == y || inside[y].contains(&z)));
        abouts.push(about);
        let (f,sense) = (shells[x][0],outward[x]);
        match (sense,about) {
            (true,None) => {}
            (false,None) => return Err(Unsound::face(f,format!("the void of face {f} lies outside the solid's outer shell"))),
            (_,Some(y)) if outward[y] != sense => {}
            (true,Some(y)) => return Err(Unsound::face(f,format!("the shell of face {f} bounds a solid inside the solid of face {}, both outward",shells[y][0]))),
            (false,Some(y)) => return Err(Unsound::face(f,format!("the void of face {f} lies inside the void of face {}",shells[y][0]))),
        }
    }
    Ok(Nesting {shells,outward,about:abouts})
}

/// The faces `s` of `b` alone, with the edges and vertices they use: `Brep::compacted` of a copy
/// of `b` holding only those faces (each kept in its order and renumbered), without copying the
/// rest.
fn part(b: &Brep,s: &[usize]) -> Brep {
    let mut used = vec![false;b.edges.len()];
    for c in s.iter().flat_map(|&f| b.faces[f].loops.iter().flatten()) { used[c.edge as usize] = true; }
    let mut held = vec![false;b.vertices.len()];
    for (e,_) in b.edges.iter().zip(&used).filter(|x| *x.1) { for v in e.v { held[v as usize] = true; } }
    let at = |kept: &[bool]| -> Vec<u32> { let mut k = 0; kept.iter().map(|&x| if x { k += 1; k-1 } else { u32::MAX }).collect() };
    let (edge_at,vertex_at) = (at(&used),at(&held));
    let vertices = b.vertices.iter().zip(&held).filter(|x| *x.1).map(|(v,_)| v.clone()).collect();
    let edges = b.edges.iter().zip(&used).filter(|x| *x.1).map(|(e,_)| Edge {v:e.v.map(|v| vertex_at[v as usize]),..e.clone()}).collect();
    let faces = s.iter().map(|&f| {
        let mut f = b.faces[f].clone();
        for c in f.loops.iter_mut().flatten() { c.edge = edge_at[c.edge as usize]; }
        f
    }).collect();
    Brep {vertices,edges,faces}
}

/// **Which way a shell faces**: a point stepped off its largest faces (up to seven, by their
/// boxes; at each of two of its longest edges that is neither degenerate nor a seam of it, until
/// three read alike and none against) along the face's outward normal, from a point of the face
/// beside the middle of the edge, located against the shell alone: outside, the shell faces out
/// (a lump); inside, in (a void, or a boundary inside out). Each step is twice the shell's
/// greatest tolerance (or a millionth of the boundary's size), then eight, thirty-two and a
/// hundred and twenty-eight times, until the point is read off the shell;
/// inward from the edge twice as far again, so that a point beside a concave edge stands in the
/// wedge outside it. The probes vote, and the shell faces the way most of them say. Where none is
/// read, or they tie, the shell's signed volume says (what it would say everywhere, but its
/// integral over every face costs more than the whole read), refused where it is none.
fn facing(b: &Brep,shell: &[usize],located: &crate::query::Located,tau: f64,size: f64,floor: f64) -> Result<bool,Unsound> {
    use crate::query::Place;
    use crate::util::space::{add,cross,scale};
    let mut picks: Vec<(f64,usize)> = shell.iter().map(|&fi| { let (lo,hi) = crate::query::face_box(b,fi); (distance(lo,hi),fi) }).collect();
    picks.sort_by(|a,b| b.0.total_cmp(&a.0).then(a.1.cmp(&b.1)));
    let (mut out,mut inward) = (0,0);
    // each of those faces' two longest edges that are neither degenerate nor a seam of it (an
    // edge the face uses twice may run through a hole of it)
    let probes: Vec<(usize,&Coedge)> = picks.iter().take(7).flat_map(|&(_,fi)| {
        let f = &b.faces[fi];
        let uses = |e: u32| f.loops.iter().flatten().filter(|c| c.edge == e).count();
        let mut cs: Vec<(f64,&Coedge)> = f.loops.iter().flatten().filter(|c| !matches!(b.edges[c.edge as usize].curve,EdgeCurve::Degenerate) && uses(c.edge) == 1)
            .map(|c| { let e = &b.edges[c.edge as usize]; (distance(e.point(e.t[0],&b.vertices),e.point((e.t[0]+e.t[1])/2.,&b.vertices)),c) }).collect();
        cs.sort_by(|x,y| y.0.total_cmp(&x.0));
        cs.into_iter().take(2).map(move |(_,c)| (fi,c))
    }).collect();
    // (a shell of seams and poles alone, a whole sphere: its seams then)
    let probes: Vec<(usize,&Coedge)> = if !probes.is_empty() { probes } else {
        picks.iter().take(7).flat_map(|&(_,fi)| b.faces[fi].loops.iter().flatten().filter(|c| !matches!(b.edges[c.edge as usize].curve,EdgeCurve::Degenerate)).take(2).map(move |c| (fi,c))).collect()
    };
    for (fi,c) in probes {
        // (three alike and none against settle it)
        if (out >= 3 || inward >= 3) && out.min(inward) == 0 { break }
        let f = &b.faces[fi];
        let e = &b.edges[c.edge as usize];
        let (tm,dt) = ((e.t[0]+e.t[1])/2.,(e.t[1]-e.t[0])*1e-4);
        let at = |t: f64| c.pcurve.at(t,e,&f.surface,&b.vertices);
        let (uv,a,z) = (at(tm),at(tm-dt),at(tm+dt));
        let mut way = [z[0]-a[0],z[1]-a[1]];
        if c.reversed { way = [-way[0],-way[1]]; }
        // the face's inside to the left of its loops' way, or to the right where it is reversed
        let mut side = [-way[1],way[0]];
        if f.reversed { side = [-side[0],-side[1]]; }
        let l = side[0].dhypot(side[1]);
        if !(l > 0.) || !uv.iter().all(|x| x.is_finite()) { continue }
        let side = [side[0]/l,side[1]/l];
        let (_,su,sv) = f.surface.d1(uv);
        let speed = crate::util::space::norm(add(scale(su,side[0]),scale(sv,side[1])));
        if !(speed > 0.) { continue }
        for k in 0..4 {
            let h = 2.*tau*4f64.powi(k);
            let d = 2.*h/speed;
            let w = [uv[0]+d*side[0],uv[1]+d*side[1]];
            let (p,su,sv) = f.surface.d1(w);
            let nrm = cross(su,sv);
            let ln = crate::util::space::norm(nrm);
            if !(ln > 0.) { break }
            let nrm = scale(nrm,if f.reversed { -1./ln } else { 1./ln });
            match located.solid_place(add(p,scale(nrm,h))) {
                Place::Out => { out += 1; break }
                Place::In => { inward += 1; break }
                Place::On => {}
            }
        }
    }
    if out > inward { return Ok(true) }
    if inward > out { return Ok(false) }
    let f = shell[0];
    // none read off the shell (a ray along a face, a meeting the queries do not prove), or as many
    // one way as the other (steps off a small void's faces beside its concave edges, CADGenBench
    // 217's drilled cavity of three faces read three and three): its signed volume says, its faces'
    // fluxes summed
    let volume: f64 = shell.iter().map(|&fi| crate::props::terms(b,fi).iter().flatten().map(|t| t.0+t.1).sum::<f64>()).sum::<f64>()/3.;
    if !(volume.abs() > floor*size.powi(2)) {
        return Err(Unsound::face(f,format!("the shell of face {f} bounds no volume ({volume:e} mm³)")))
    }
    Ok(volume > 0.)
}
