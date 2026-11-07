//! **The tool read from its B-rep** (#721 F1, #828 F1a): a sweep's tool as the kernel reads it off
//! the boundary it builds, by its faces' surfaces and the edges they share, never by how it was
//! drawn.
//! - A face of revolution spanning a whole turn is read about its own axis, and so is a plane
//!   bounded by circles about one centre (a disc or a ring, #684 O).
//! - A face whose one loop is two lines or two coaxial arcs joined end to end by two lines is a
//!   ruled strip, a prism's side, straight or drafted. The strips that share their rulings are read
//!   one way along them, so a box is a prism one way of the three.
//! - A face over part of a turn sharing a circle with a face read about that circle's axis — a part
//!   of a drum's, a cone's, a ball's or a ring's surface beside it, or a plane square to the axis —
//!   is read about it before any face is read as a strip, whatever its loop (#721 F1d): the faces
//!   of one turn read alike.
//! - A face of revolution a Boolean trims to part of a turn is its surface's whole turn, and a
//!   plane sharing a circle with a revolution a flat of it (one about each axis it shares a circle
//!   with, from its centre where its loops wind about it): every face is kept whole, the material
//!   telling the face from the rest, as the tool keeps the faces a Boolean hides (`Tool`).
//! - Any other plane is a cap.
//!
//! Corners are where two faces read share an edge at the ends of their meridians or edges, or along
//! a ring partway along a face of revolution's meridian (#721 F1d), creases where two strips share
//! a rim and their rulings go on across it; each revolution's chart starts where its own surface's
//! frame does, nothing the sweep reads depending on where (#721 F1d). Faces a Boolean cuts from one
//! surface are read once, over the span of its meridian they reach (`once_a_surface`). The material
//! is the boundary's own (`Boundary`), its side at a corner the corner's pseudo-normal's. What no
//! reading takes is refused by its geometry, naming the face and a point of it (`Unread`).
//!
//! The classes read the tool this way (#721 F1b: the solver's `SweepContacts::read`), and
//! `tests/tool_reading.rs` holds the reading to its own B-rep and to the drawing's field; the
//! drawing's reading of the tool, its oracle while the classes switched, is retired (#721 F1c).
use crate::util::fmath::Det;
use crate::geom::{Curve,Frame,Surface,Uv};
use crate::query::{Located,Place};
use crate::recipe::joined;
use crate::topo::{unwrap,Brep,EdgeCurve,Face};
use crate::util::space::{add,cross,distance,dot,norm,normalised,scale,sub};
use super::surface::{Meridian,RevolvedSurface};
use super::tool::{CapSurface,Edge,ExtrudedSurface,Material,PatchEdge,Side,Tool,ToolSurface};
use std::f64::consts::TAU;
use std::sync::Arc;

type V = [f64;3];

/// Why a tool's B-rep has no reading: the face, by its name, and a point of it (in the reading's
/// units), with what it is that no reading takes yet.
#[derive(Clone,Debug,PartialEq)]
pub enum Unread {
    /// A face no closed form reads: a B-spline, a blend, a face family's wall or an affine image
    /// (#721 G10).
    FreeForm { face: String,kind: &'static str,at: V },
    /// A face of revolution whose reading does not lie on it to the boundary's tolerance, or has
    /// no outward side (a face no wider than its tolerance).
    Unfit { face: String,kind: &'static str,at: V },
    /// A face of revolution whose meridian is no line or circle (#721 G8).
    Meridian { face: String,kind: &'static str,at: V },
    /// The boundary has no face, or is read at no scale.
    Boundary(String),
}

impl std::fmt::Display for Unread {
    fn fmt(&self,f: &mut std::fmt::Formatter) -> std::fmt::Result {
        match self {
            Unread::FreeForm {face,kind,..} => write!(f,"its face `{face}` is {} {kind}, which no closed form reads",article(kind)),
            Unread::Unfit {face,kind,..} => write!(f,"its face `{face}` is {} {kind} its reading does not lie on",article(kind)),
            Unread::Meridian {face,kind,..} => write!(f,"its face `{face}` turns {} {kind}, not a line or a circle",article(kind)),
            Unread::Boundary(m) => write!(f,"{m}"),
        }
    }
}

/// The article a kind's name takes.
fn article(kind: &str) -> &'static str { if kind.starts_with(['a','e','i','o','u']) { "an" } else { "a" } }

impl Unread {
    /// The point of the face it names, where it names one.
    pub fn witness(&self) -> Option<V> {
        match self { Unread::FreeForm {at,..} | Unread::Unfit {at,..} | Unread::Meridian {at,..} => Some(*at),_ => None }
    }
}

impl Tool {
    /// **The tool `b` is, read from its boundary** (#828), in units `1/scale` of the boundary's
    /// own (a millimetre B-rep read in a model's units of `scale` millimetres): its faces by their
    /// surfaces, its corners and creases where the faces it reads share an edge, and its material
    /// the boundary's (`Boundary`). Refused, naming a face, where a face is of no kind it reads.
    pub fn read(b: &Brep,scale: f64) -> Result<Tool,Unread> {
        if b.faces.is_empty() { return Err(Unread::Boundary("the tool has no face".into())) }
        if !(scale > 0. && scale.is_finite()) { return Err(Unread::Boundary(format!("a scale of {scale}"))) }
        let size = b.size();
        let tol = 1e-7*(1.+size);
        let reader = Reader {b,tol,uses:b.uses(),parameters:vec![Default::default();b.faces.len()]};
        let order: Vec<usize> = (0..b.faces.len()).filter(|&fi| !matches!(b.faces[fi].surface,Surface::Plane(_)))
            .chain((0..b.faces.len()).filter(|&fi| matches!(b.faces[fi].surface,Surface::Plane(_)))).collect();
        let witness = |fi: usize| reader.witness(fi).map(|x| x/scale);
        // each face's reading: the faces of revolution whole round first (those curved before the
        // planes, so an axis starts its revolutions where a curved face about it does), then the
        // strips (again those curved first, whose rulings are their own), then what is left
        let mut read: Vec<Option<Read>> = vec![None;b.faces.len()];
        for &fi in &order { read[fi] = reader.revolved(fi,false).map(Read::Revolved) }
        // a face over part of a turn sharing a circle with a face read about that circle's axis — a
        // part of a drum's, a cone's, a ball's or a ring's surface beside it, or a plane square to the
        // axis — is read about the same axis, as a revolution whatever its loop, before any face is
        // read as a strip: the faces of one turn are read alike, and meet at corners along their
        // meridians (#721 F1d: a part-cylinder's kind no longer its loop's)
        let mut more: Vec<(usize,Read)> = Vec::new();
        loop {
            let mut found = false;
            for &fi in &order {
                if read[fi].is_some() { continue }
                if matches!(b.faces[fi].surface,Surface::Plane(_)) {
                    found |= reader.read_flats(fi,&mut read,&mut more);
                } else if let Some(s) = reader.revolved(fi,true).filter(|s| reader.beside(fi,s,&read)) {
                    read[fi] = Some(Read::Revolved(s)); found = true
                }
            }
            if !found { break }
        }
        let options: Vec<Vec<Strip>> = (0..b.faces.len())
            .map(|fi| if read[fi].is_some() { Vec::new() } else { reader.strips(fi) }).collect();
        for &seed in &order {
            if read[seed].is_some() { continue }
            let Some(s) = options[seed].iter().find(|s| reader.fits(s,seed,&read)).cloned() else { continue };
            read[seed] = Some(Read::Strip(s.clone()));
            // the strips beside it along a ruling, or on across a crease, read the same way round
            let mut pending = vec![(seed,s)];
            while let Some((fi,s)) = pending.pop() {
                for &e in s.rulings.iter().chain(&s.rims) {
                    for &(g,_,_) in &reader.uses[e as usize] {
                        if g == fi || read[g].is_some() { continue }
                        let Some(t) = options[g].iter().find(|t| (t.rulings.contains(&e) || reader.continues(&s,t,e))
                            && reader.fits(t,g,&read)).cloned() else { continue };
                        read[g] = Some(Read::Strip(t.clone()));
                        pending.push((g,t));
                    }
                }
            }
        }
        // a face of revolution over part of a turn that is no strip — a ball's, a ring's or a drum's
        // a Boolean of the tool trims — is its surface's whole turn over the face's span of its
        // meridian, the material telling the face from the rest of the turn, as every face a
        // Boolean hides is kept
        for fi in 0..b.faces.len() {
            if read[fi].is_none() { read[fi] = reader.revolved(fi,true).map(Read::Revolved) }
        }
        // a plane sharing a circle with a face of revolution about the circle's axis — a disc or a
        // ring a Boolean trims — is a flat of that revolution (#684 O) over the radii its loops
        // reach, the material telling the face from the rest of the ring
        // (one about each axis it shares a circle with: a plane two crowns about two axes stand on
        // is a flat of each)
        // (a cap reads only the strips beside it, a flat only the revolutions: one pass reads both)
        for fi in 0..b.faces.len() {
            if read[fi].is_some() { continue }
            if reader.read_flats(fi,&mut read,&mut more) { continue }
            let f = &b.faces[fi];
            let (face,kind,at) = (joined(&f.name),f.surface.kind(),witness(fi));
            read[fi] = Some(match &f.surface {
                Surface::Plane(frame) => reader.cap(fi,frame,&read),
                Surface::Revolution(_,c) if !matches!(**c,Curve::Line {..} | Curve::Circle(..)) =>
                    return Err(Unread::Meridian {face,kind:c.kind(),at}),
                Surface::Revolution(..) | Surface::Cylinder(..) | Surface::Cone(..) | Surface::Sphere(..) | Surface::Torus(..) =>
                    return Err(Unread::Unfit {face,kind,at}),
                Surface::Extrusion(..) | Surface::Blend(..) | Surface::BSpline(..) | Surface::Family(..) | Surface::Affine(..) =>
                    return Err(Unread::FreeForm {face,kind,at}),
            });
        }
        // every reading, each face's first and then the flats beyond a face's first, and the
        // readings of each face
        let read: Vec<(usize,Read)> = read.into_iter().enumerate().map(|(fi,r)| (fi,r.expect("every face read or refused")))
            .chain(more).collect();
        let mut of: Vec<Vec<usize>> = vec![Vec::new();b.faces.len()];
        for (k,(fi,_)) in read.iter().enumerate() { of[*fi].push(k) }
        // corners: two faces sharing an edge each at an end of its meridian or its edge; creases:
        // two strips sharing an edge, the one's far rim and the other's near, their rulings going on
        let (mut edges,mut creases) = (Vec::new(),Vec::new());
        for (e,uses) in reader.uses.iter().enumerate() {
            let faces: Vec<usize> = uses.iter().map(|u| u.0).collect::<std::collections::BTreeSet<_>>()
                .into_iter().collect();
            let e = e as u32;
            // a meridian closed on itself (a ring's circle turned whole) meets itself at its seam
            if let ([a],2) = (&faces[..],uses.len()) {
                for &k in &of[*a] {
                    if let Read::Revolved(s) = &read[k].1 {
                        if reader.on_end(s,e,1.) && reader.on_end(s,e,0.) {
                            edges.push(PatchEdge {patches:[k,k],ends:[1.,0.]})
                        }
                    }
                }
                continue
            }
            let [a,c] = faces[..] else { continue };
            for &ka in &of[a] { for &kc in &of[c] {
                if let (Some(ua),Some(uc)) = (reader.end_of(&read[ka].1,e),reader.end_of(&read[kc].1,e)) {
                    edges.push(PatchEdge {patches:[ka,kc],ends:[ua,uc]});
                }
            } }
            if let (Read::Strip(s),Read::Strip(t)) = (&read[a].1,&read[c].1) {
                if s.rims[1] == e && t.rims[0] == e && reader.continues(s,t,e) { creases.push([a,c]) }
                else if t.rims[1] == e && s.rims[0] == e && reader.continues(t,s,e) { creases.push([c,a]) }
            }
        }
        let faces = read.into_iter().map(|(fi,r)| r.surface(&b.faces[fi],scale)).collect();
        let (faces,edges,creases) = once_a_surface(faces,edges,creases,1e2*tol/scale);
        Ok(Tool {faces,edges,creases,material:Arc::new(Boundary::new(b,scale,tol,&reader.uses))})
    }
}

/// **One reading a surface** (#721 F1b, F1d): the faces a Boolean cuts from one surface of
/// revolution are each read as that surface's whole turn (`Reader::revolved`) over its own span of
/// the meridian; they are read once, the first's reading over the span they reach together (its
/// meridian's line or circle carried past its ends, `RevolvedSurface::over`), and the corners and
/// creases that name the others name it, each corner's place along its meridian read on the one
/// reading — partway along it where another face of the surface went on past it, which the walk
/// reads (`constant_twist`'s partway corners). `tol` is how near two readings' points must lie to
/// be one surface's.
fn once_a_surface(faces: Vec<ToolSurface>,edges: Vec<PatchEdge>,creases: Vec<[usize;2]>,tol: f64)
    -> (Vec<ToolSurface>,Vec<PatchEdge>,Vec<[usize;2]>) {
    let n = faces.len();
    // each reading's first of its surface, and where its meridian's ends lie on that one's, alike
    // wherever either chart starts (#721 F1d)
    let mut first: Vec<usize> = (0..n).collect();
    let mut spans: Vec<[f64;2]> = vec![[0.,1.];n];
    for j in 0..n {
        if let Some((i,span)) = (0..j).filter(|&i| first[i] == i).find_map(|i| one_surface(&faces[i],&faces[j],tol).map(|s| (i,s))) {
            (first[j],spans[j]) = (i,span);
        }
    }
    // each surface's span, the hull of its readings', and its one reading over it
    let mut hull: Vec<[f64;2]> = vec![[0.,1.];n];
    for j in 0..n { let (i,[a,b]) = (first[j],spans[j]); hull[i] = [hull[i][0].min(a.min(b)),hull[i][1].max(a.max(b))] }
    let mut index = vec![0;n];
    let mut kept: Vec<ToolSurface> = Vec::new();
    for (i,f) in faces.iter().enumerate() {
        if first[i] != i { continue }
        index[i] = kept.len();
        kept.push(match f {
            ToolSurface::Revolved(s) if hull[i] != [0.,1.] => ToolSurface::Revolved(s.over(hull[i]).expect("a line's or a circle's meridian")),
            _ => f.clone(),
        });
    }
    for j in 0..n { index[j] = index[first[j]] }
    // a reading's `u` on its surface's one reading
    let place = |k: usize,u: f64| -> f64 {
        let (i,[a,b]) = (first[k],spans[k]);
        let [lo,hi] = hull[i];
        (a+u*(b-a)-lo)/(hi-lo)
    };
    let mut merged: Vec<PatchEdge> = Vec::new();
    for e in edges {
        let patches = e.patches.map(|k| index[k]);
        // (a corner between two faces of the one surface is none, but a meridian's seam with itself;
        // and each once, though the B-rep splits its circle into edges)
        if patches[0] == patches[1] && e.patches[0] != e.patches[1] { continue }
        let ends = [place(e.patches[0],e.ends[0]),place(e.patches[1],e.ends[1])].map(|u| if u.abs() <= 1e-12 { 0. }
            else if (u-1.).abs() <= 1e-12 { 1. } else { u });
        if merged.iter().any(|m| m.patches == patches && (0..2).all(|s| (m.ends[s]-ends[s]).abs() <= 1e-9)) { continue }
        merged.push(PatchEdge {patches,ends});
    }
    let creases = creases.into_iter().map(|c| c.map(|k| index[k])).collect();
    (kept,merged,creases)
}

/// Whether reading `c` lies on reading `a`'s surface: two faces of revolution about one axis, whole
/// turns, `c`'s meridian on `a`'s line or circle and their charts' normals (each the face's
/// outward one) the same way there; where it does, the parameters of `a`'s meridian, carried past
/// its ends, at `c`'s meridian's two ends.
fn one_surface(a: &ToolSurface,c: &ToolSurface,tol: f64) -> Option<[f64;2]> {
    let (ToolSurface::Revolved(a),ToolSurface::Revolved(c)) = (a,c) else { return None };
    let (x,y) = (a.axis_direction(),c.axis_direction());
    if norm(cross(x,y)) > 1e-9 || a.sweep().abs() != c.sweep().abs() { return None }
    if a.polar(c.origin())?[0] > tol { return None }
    // `a`'s meridian at `t`, past its ends too
    let on = |t: f64| a.over([t,t+1.]).and_then(|s| s.at(0.,0.).ok());
    let mut ts = [0.;5];
    for (k,u) in [0.,0.25,0.5,0.75,1.].into_iter().enumerate() {
        let p = c.at(u,0.).ok()?;
        let t = a.meridian_param(p.position)?;
        let (q,r) = (a.polar(on(t)?.position)?,a.polar(p.position)?);
        if (q[0]-r[0]).abs() > tol || (q[1]-r[1]).abs() > tol { return None }
        ts[k] = t;
    }
    // the parameter runs along `c`'s alike (a line's or a circle's, each at one pace)
    let span = (ts[4]-ts[0]).abs().max(1e-300);
    if (0..5).any(|k| (ts[0]+(ts[4]-ts[0])*k as f64/4.-ts[k]).abs() > 1e-6*span) { return None }
    // and the face is outward on it as on `a`'s
    let p = c.at(0.5,0.).ok()?;
    let q = a.over([ts[2],ts[2]+1.])?.at(0.,a.turn_of(p.position)?).ok()?;
    if dot(cross(p.du,p.dv),cross(q.du,q.dv)) <= 0. { return None }
    Some([ts[0],ts[4]])
}

/// A face as read, in the boundary's units: a face of revolution, a ruled strip, or a cap.
#[derive(Clone,Debug)]
enum Read { Revolved(RevolvedSurface),Strip(Strip),Cap { end: bool,corner: V,du: V,dv: V } }

impl Read {
    /// The tool's face in units `1/scale` of the boundary's, named as the face.
    fn surface(self,f: &Face,scale: f64) -> ToolSurface {
        let k = 1./scale;
        let v = |x: V| x.map(|c| c*k);
        let edge = |e: Edge| match e {
            Edge::Line {start,delta} => Edge::Line {start:v(start),delta:v(delta)},
            Edge::Round {center,a,b,sweep} => Edge::Round {center:v(center),a:v(a),b:v(b),sweep},
        };
        match self {
            Read::Revolved(s) => {
                let meridian = match *s.meridian() {
                    Meridian::Line {start,delta} => Meridian::Line {start:v(start),delta:v(delta)},
                    Meridian::Round {center,a,b,sweep} => Meridian::Round {center:v(center),a:v(a),b:v(b),sweep},
                    Meridian::Conic {a,m,b,w} => Meridian::Conic {a:v(a),m:v(m),b:v(b),w},
                };
                ToolSurface::Revolved(RevolvedSurface::new(joined(&f.name),meridian,v(s.origin()),s.axis_direction(),s.sweep()))
            }
            Read::Strip(s) => ToolSurface::Extruded(ExtrudedSurface::new(joined(&f.name),edge(s.edge),match s.side {
                Side::Straight(d) => Side::Straight(v(d)),
                Side::Leaning(far) => Side::Leaning(edge(far)),
            })),
            Read::Cap {end,corner,du,dv} => ToolSurface::Cap(CapSurface::new(joined(&f.name),end,v(corner),v(du),v(dv))),
        }
    }
}

/// A face read as a ruled strip: its edge at `v = 0` (the use `near` of its loop, walked as the
/// loop walks it), how it runs to its edge at `v = 1`, the edges at its ends in `u` (`rulings`, at
/// `u = 0` then `u = 1`) and at its ends in `v` (`rims`), the vertex each ruling leaves its near
/// rim from, and the way its rulings run (the sum of its two end rulings' directions).
#[derive(Clone,Debug)]
struct Strip { edge: Edge,side: Side,rulings: [u32;2],rims: [u32;2],bases: [u32;2],along: V }

/// What reads a boundary: it, the tolerance its points are matched to, each edge's uses, and each
/// face's parameters.
struct Reader<'a> {
    b: &'a Brep,
    tol: f64,
    uses: Vec<Vec<(usize,usize,usize)>>,
    /// Each face's parameters (`parameters`), read once.
    parameters: Vec<std::cell::OnceCell<[[f64;2];2]>>,
}

impl Reader<'_> {
    /// A point of face `fi`: its first edge's middle.
    fn witness(&self,fi: usize) -> V {
        let f = &self.b.faces[fi];
        f.loops.iter().flatten().find_map(|c| {
            let e = &self.b.edges[c.edge as usize];
            matches!(e.curve,EdgeCurve::Curve(_)).then(|| e.point(0.5*(e.t[0]+e.t[1]),&self.b.vertices))
        }).unwrap_or_else(|| f.surface.point([0.;2]))
    }

    /// Face `fi`'s parameters over its loops: the least and greatest of each, each loop unwrapped
    /// along itself.
    fn parameters(&self,fi: usize) -> [[f64;2];2] { *self.parameters[fi].get_or_init(|| self.span(fi)) }

    fn span(&self,fi: usize) -> [[f64;2];2] {
        let f = &self.b.faces[fi];
        let mut span = [[f64::INFINITY,f64::NEG_INFINITY];2];
        for l in &f.loops {
            let mut last: Option<Uv> = None;
            for c in l {
                let e = &self.b.edges[c.edge as usize];
                for k in 0..=8 {
                    let s = k as f64/8.;
                    let t = if c.reversed { e.t[1]-(e.t[1]-e.t[0])*s } else { e.t[0]+(e.t[1]-e.t[0])*s };
                    let mut uv = c.pcurve.at(t,e,&f.surface,&self.b.vertices);
                    if let Some(near) = last { uv = unwrap(uv,near,f.surface.periods()) }
                    for i in 0..2 { span[i] = [span[i][0].min(uv[i]),span[i][1].max(uv[i])] }
                    last = Some(uv);
                }
            }
        }
        span
    }

    /// Whether every point of `s` sampled lies on face `fi`'s surface.
    fn on_surface(&self,fi: usize,at: &dyn Fn(f64,f64) -> Option<V>) -> bool {
        let surface = &self.b.faces[fi].surface;
        let on = |p: V| distance(surface.point(surface.inverse(p)),p) <= 1e2*self.tol;
        (0..=4).all(|i| (0..=4).all(|j| at(i as f64/4.,j as f64/4.).is_some_and(on)))
    }

    /// **Face `fi` as a face of revolution, where it is one whole turn round** (or, `part`, where it
    /// is over part of one, read as its surface's whole turn): a cylinder, cone, sphere, torus or
    /// revolution of a line or a circle spanning a whole period of its turn, or a plane bounded by
    /// circles about one centre, each square to the plane (#684 O). Its meridian lies in the
    /// half-plane every revolution about its axis starts from (`start`), run so that the face's
    /// chart's normal is its outward one.
    fn revolved(&self,fi: usize,part: bool) -> Option<RevolvedSurface> {
        let f = &self.b.faces[fi];
        let [[u0,u1],[v0,v1]] = self.parameters(fi);
        let fr = match &f.surface {
            Surface::Plane(_) if part => return None,
            Surface::Plane(frame) => return self.disc(fi,frame),
            _ if (u1-u0 >= TAU*(1.-1e-9)) == part => return None,
            Surface::Cylinder(fr,..) | Surface::Cone(fr,..) | Surface::Sphere(fr,..) | Surface::Torus(fr,..)
                | Surface::Revolution(fr,..) => fr,
            _ => return None,
        };
        let ((a,x),o) = (self.start(fr),fr.o);
        // the meridian's point `ρ` out along the half-plane it starts in and `h` along the axis
        let at = |rho: f64,h: f64| add(o,add(scale(x,rho),scale(fr.z,h)));
        let meridian = match &f.surface {
            Surface::Cylinder(_,r) => Meridian::Line {start:at(*r,v0),delta:scale(fr.z,v1-v0)},
            Surface::Cone(_,r,alpha) => {
                let (sn,cs) = alpha.dsin_cos();
                let (p,q) = (at(r+v0*sn,v0*cs),at(r+v1*sn,v1*cs));
                Meridian::Line {start:p,delta:sub(q,p)}
            }
            Surface::Sphere(_,r) => Meridian::round(o,x,fr.z,*r,v0,v1-v0),
            Surface::Torus(_,big,r) => Meridian::round(at(*big,0.),x,fr.z,*r,v0,v1-v0),
            // the curve, which lies in its frame's half-plane, set in the half-plane every
            // revolution about the axis starts from
            Surface::Revolution(_,c) => {
                let lift = |p: V| { let l = fr.local(p); at(l[0],l[2]) };
                let dir = |d: V| add(scale(x,dot(d,fr.x)),scale(fr.z,dot(d,fr.z)));
                match **c {
                    Curve::Line {..} => { let (p,q) = (lift(c.point(v0)),lift(c.point(v1))); Meridian::Line {start:p,delta:sub(q,p)} }
                    Curve::Circle(cf,r) => Meridian::round(lift(cf.o),dir(cf.x),dir(cf.y),r,v0,v1-v0),
                    _ => return None,
                }
            }
            _ => return None,
        };
        self.outward(fi,RevolvedSurface::new(f.name.clone(),meridian,o,a,TAU))
    }

    /// A plane face bounded by circles about one centre, square to it: a disc, or a ring between
    /// two radii (#684 O), turned about the line through that centre square to the plane.
    fn disc(&self,fi: usize,frame: &Frame) -> Option<RevolvedSurface> {
        let f = &self.b.faces[fi];
        let mut radii: Vec<f64> = Vec::new();
        let mut centre: Option<V> = None;
        for l in &f.loops {
            let mut radius: Option<f64> = None;
            for c in l {
                let EdgeCurve::Curve(Curve::Circle(cf,r)) = &self.b.edges[c.edge as usize].curve else { return None };
                if norm(cross(cf.z,frame.z)) > 1e-9 { return None }
                match centre { Some(o) if distance(o,cf.o) > self.tol => return None,_ => centre = Some(cf.o) }
                match radius { Some(q) if (q-r).abs() > self.tol => return None,_ => radius = Some(*r) }
            }
            radii.push(radius?);
        }
        let o = centre?;
        let (inner,outer) = match radii[..] {
            [r] => (0.,r),
            [a,b] => (a.min(b),a.max(b)),
            _ => return None,
        };
        self.flat(fi,frame,o,[inner,outer])
    }

    /// Plane face `fi` (of `frame`) as a flat turned about the line through `o` square to it, from
    /// the radius `radii[0]` to `radii[1]`.
    fn flat(&self,fi: usize,frame: &Frame,o: V,[inner,outer]: [f64;2]) -> Option<RevolvedSurface> {
        if !(outer-inner > self.tol) { return None }
        let (a,x) = self.start(&Frame {o,..*frame});
        let meridian = Meridian::Line {start:add(o,scale(x,inner)),delta:scale(x,outer-inner)};
        self.outward(fi,RevolvedSurface::new(self.b.faces[fi].name.clone(),meridian,o,a,TAU))
    }

    /// Plane face `fi` as a flat of each revolution it shares a circle with, about that circle's
    /// axis, over the least and greatest radii its loops reach about it: one for each axis.
    fn flats(&self,fi: usize,read: &[Option<Read>]) -> Vec<RevolvedSurface> {
        let f = &self.b.faces[fi];
        let Surface::Plane(frame) = &f.surface else { return Vec::new() };
        let mut axes: Vec<V> = Vec::new();
        for cf in self.shared_circles(fi,read) {
            if norm(cross(cf.z,frame.z)) > 1e-9 || axes.iter().any(|&o| distance(o,cf.o) <= 1e2*self.tol) { continue }
            axes.push(cf.o)
        }
        axes.into_iter().filter_map(|axis| {
            let (mut inner,mut outer) = (f64::INFINITY,0_f64);
            // (and how many times the loops wind about the axis, walked as they run: a face that
            // holds the axis's point reaches it, whatever radius its loops' nearest point is at)
            let mut winding = 0.;
            for l in &f.loops {
                let mut last: Option<f64> = None;
                for c in l {
                    let e = &self.b.edges[c.edge as usize];
                    for k in 0..=32 {
                        let s = k as f64/32.;
                        let t = if c.reversed { e.t[1]-(e.t[1]-e.t[0])*s } else { e.t[0]+(e.t[1]-e.t[0])*s };
                        let q = sub(e.point(t,&self.b.vertices),axis);
                        let r = norm(q);
                        (inner,outer) = (inner.min(r),outer.max(r));
                        let angle = dot(q,frame.y).datan2(dot(q,frame.x));
                        if let Some(a) = last { winding += (angle-a+std::f64::consts::PI).rem_euclid(TAU)-std::f64::consts::PI }
                        last = Some(angle);
                    }
                }
            }
            if inner <= 1e2*self.tol || winding.abs() > std::f64::consts::PI { inner = 0. }
            self.flat(fi,frame,axis,[inner,outer])
        }).collect()
    }

    /// Plane face `fi` read as its flats (`flats`), where it is any: the first its reading, the rest
    /// readings beyond it (`more`).
    fn read_flats(&self,fi: usize,read: &mut [Option<Read>],more: &mut Vec<(usize,Read)>) -> bool {
        let mut flats = self.flats(fi,read).into_iter().map(Read::Revolved);
        let Some(flat) = flats.next() else { return false };
        read[fi] = Some(flat);
        more.extend(flats.map(|r| (fi,r)));
        true
    }

    /// Whether face `fi`, read as `s`, shares a circle about `s`'s axis with a face already read as
    /// a revolution about it.
    fn beside(&self,fi: usize,s: &RevolvedSurface,read: &[Option<Read>]) -> bool {
        self.shared_circles(fi,read).iter().any(|cf| norm(cross(cf.z,s.axis_direction())) <= 1e-9
            && s.polar(cf.o).is_some_and(|q| q[0] <= 1e2*self.tol))
    }

    /// The circles of face `fi`'s loops it shares with a face already read as a revolution about
    /// the circle's axis, each its frame.
    fn shared_circles(&self,fi: usize,read: &[Option<Read>]) -> Vec<Frame> {
        self.b.faces[fi].loops.iter().flatten().filter_map(|c| {
            let EdgeCurve::Curve(Curve::Circle(cf,_)) = &self.b.edges[c.edge as usize].curve else { return None };
            self.uses[c.edge as usize].iter().any(|&(g,_,_)| g != fi && matches!(&read[g],Some(Read::Revolved(t))
                if norm(cross(t.axis_direction(),cf.z)) <= 1e-9 && t.polar(cf.o).is_some_and(|q| q[0] <= 1e2*self.tol))).then_some(*cf)
        }).collect()
    }

    /// `s` run so its chart's normal at its middle is face `fi`'s outward one, where it lies on the
    /// face's surface.
    fn outward(&self,fi: usize,s: RevolvedSurface) -> Option<RevolvedSurface> {
        if !self.on_surface(fi,&|u,v| s.at(u,v).ok().map(|p| p.position)) { return None }
        let p = s.at(0.5,0.).ok()?;
        let n = self.b.faces[fi].outward(p.position)?;
        Some(if dot(cross(p.du,p.dv),n) < 0. { s.reversed() } else { s })
    }

    /// **Face `fi`'s readings as a ruled strip**: its one loop four uses, two lines or two coaxial
    /// arcs joined end to end by two lines, the first rim each of the four ways its loop starts.
    /// The loop walks with the face on its left, so the chart's normal is the face's outward one.
    fn strips(&self,fi: usize) -> Vec<Strip> {
        let f = &self.b.faces[fi];
        if f.loops.len() != 1 || f.loops[0].len() != 4 { return Vec::new() }
        let l = &f.loops[0];
        let line = |k: usize| matches!(self.b.edges[l[k%4].edge as usize].curve,EdgeCurve::Curve(Curve::Line {..}));
        // a use's ends and its edge's curve walked as the loop walks it: a line, or an arc
        let walked = |k: usize| -> Option<Edge> {
            let c = &l[k%4];
            let e = &self.b.edges[c.edge as usize];
            let [p,q] = self.b.ends(c).map(|v| self.b.vertices[v as usize].p);
            match &e.curve {
                EdgeCurve::Curve(Curve::Line {..}) => Some(Edge::Line {start:p,delta:sub(q,p)}),
                EdgeCurve::Curve(Curve::Circle(cf,r)) => {
                    let (start,sweep) = if c.reversed { (e.t[1],-(e.t[1]-e.t[0])) } else { (e.t[0],e.t[1]-e.t[0]) };
                    Some(Edge::round(cf.o,*r,start,sweep,|x,y| cf.dir([x,y,0.])))
                }
                _ => None,
            }
        };
        let mut out = Vec::new();
        for i in 0..4 {
            if !line(i+1) || !line(i+3) { continue }
            let (Some(near),Some(far)) = (walked(i),walked(i+2)) else { continue };
            // the far rim walked the near rim's way: from the end of the ruling at u = 0
            let far = match far {
                Edge::Line {start,delta} => Edge::Line {start:add(start,delta),delta:scale(delta,-1.)},
                Edge::Round {center,a,b,sweep} => {
                    let (sn,cs) = sweep.dsin_cos();
                    Edge::Round {center,a:add(scale(a,cs),scale(b,sn)),b:sub(scale(a,sn),scale(b,cs)),sweep}
                }
            };
            let side = match (near,far) {
                (Edge::Line {start:p,delta:d},Edge::Line {start:q,delta:e}) => {
                    let along = sub(q,p);
                    if norm(sub(d,e)) <= self.tol { Side::Straight(along) } else { Side::Leaning(far) }
                }
                (Edge::Round {center:c,a,b,sweep},Edge::Round {center:o,a:a2,b:b2,sweep:s2}) => {
                    // coaxial arcs over one span, ruled point for point: a cylinder's, or a cone's
                    if (sweep-s2).abs() > 1e-9 || norm(cross(cross(a,b),cross(a2,b2))) > 1e-9*norm(cross(a,b)).max(1e-300) { continue }
                    let along = sub(o,c);
                    if norm(sub(a,a2)) <= self.tol && norm(sub(b,b2)) <= self.tol { Side::Straight(along) } else { Side::Leaning(far) }
                }
                _ => continue,
            };
            let strip = ExtrudedSurface::new(String::new(),near,side);
            if !self.on_surface(fi,&|u,v| strip.at(u,v).ok().map(|p| p.position)) { continue }
            let [p0,p1] = self.b.ends(&l[i%4]);
            let at = |u: f64,v: f64| strip.at(u,v).map(|p| p.position).unwrap_or([f64::NAN;3]);
            let ruling = |u: f64| normalised(sub(at(u,1.),at(u,0.))).unwrap_or([0.;3]);
            out.push(Strip {edge:near,side,rulings:[l[(i+3)%4].edge,l[(i+1)%4].edge],rims:[l[i%4].edge,l[(i+2)%4].edge],
                bases:[p0,p1],along:add(ruling(0.),ruling(1.))});
        }
        out
    }

    /// Whether strip `s` of face `fi` reads each edge it shares with a strip already read as that
    /// strip does: a ruling as a ruling left from the same rim, a rim as a rim, and a crease (its
    /// rulings going on across it) the one's far rim and the other's near.
    fn fits(&self,s: &Strip,fi: usize,read: &[Option<Read>]) -> bool {
        let f = &self.b.faces[fi];
        f.loops.iter().flatten().all(|c| self.uses[c.edge as usize].iter().all(|&(g,_,_)| {
            let (true,Some(Read::Strip(t))) = (g != fi,&read[g]) else { return true };
            let e = c.edge;
            match (s.rulings.iter().position(|&x| x == e),t.rulings.iter().position(|&x| x == e)) {
                (Some(i),Some(j)) => s.bases[i] == t.bases[j],
                (None,None) => !(self.continues(s,t,e) || self.continues(t,s,e))
                    || (s.rims[1] == e && t.rims[0] == e) || (s.rims[0] == e && t.rims[1] == e),
                _ => false,
            }
        }))
    }

    /// Whether strips `s` and `t`, sharing the rim `e`, go on across it: their rulings there
    /// within a right angle of each other, one leaving the edge as the other arrives.
    fn continues(&self,s: &Strip,t: &Strip,e: u32) -> bool {
        let (Some(i),Some(j)) = (s.rims.iter().position(|&x| x == e),t.rims.iter().position(|&x| x == e)) else { return false };
        if i == j { return false }
        dot(s.along,t.along) > 0.5*norm(s.along)*norm(t.along)
    }

    /// Plane face `fi` as a cap: the box its loops lie in within its plane, a little past them,
    /// `du × dv` its outward normal; at the strips' far end where it meets a strip's far rim.
    fn cap(&self,fi: usize,frame: &Frame,read: &[Option<Read>]) -> Read {
        let f = &self.b.faces[fi];
        let (mut lo,mut hi) = ([f64::INFINITY;2],[f64::NEG_INFINITY;2]);
        for c in f.loops.iter().flatten() {
            let e = &self.b.edges[c.edge as usize];
            for k in 0..=16 {
                let q = frame.local(e.point(e.t[0]+(e.t[1]-e.t[0])*k as f64/16.,&self.b.vertices));
                lo = [lo[0].min(q[0]),lo[1].min(q[1])];
                hi = [hi[0].max(q[0]),hi[1].max(q[1])];
            }
        }
        // a little past the loops' points, which their arcs bulge beyond
        let pad = 0.02*(hi[0]-lo[0]).max(hi[1]-lo[1]);
        let (lo,hi) = (lo.map(|x| x-pad),hi.map(|x| x+pad));
        let lift = |x: f64,y: f64| frame.at([x,y,0.]);
        let (du,dv) = (scale(frame.x,hi[0]-lo[0]),scale(frame.y,hi[1]-lo[1]));
        let end = f.loops.iter().flatten().any(|c| self.uses[c.edge as usize].iter()
            .any(|&(g,_,_)| matches!(&read[g],Some(Read::Strip(t)) if t.rims[1] == c.edge)));
        if f.reversed { Read::Cap {end,corner:lift(lo[0],hi[1]),du,dv:scale(dv,-1.)} }
        else { Read::Cap {end,corner:lift(lo[0],lo[1]),du,dv} }
    }

    /// The end in `u` of face `r` edge `e` lies along, where it lies along one: a strip's ruling,
    /// or the circle a revolution's meridian's end turns.
    fn end_of(&self,r: &Read,e: u32) -> Option<f64> {
        match r {
            Read::Strip(s) => s.rulings.iter().position(|&x| x == e).map(|i| i as f64),
            // (or a ring partway along its meridian, where a Boolean cuts the face there over part
            // of its turn: the walk reads the corner there, #721 F1d)
            Read::Revolved(s) => [0.,1.].into_iter().find(|&u| self.on_end(s,e,u)).or_else(|| {
                let edge = &self.b.edges[e as usize];
                let EdgeCurve::Curve(curve) = &edge.curve else { return None };
                let u = s.meridian_param(curve.point(0.5*(edge.t[0]+edge.t[1])))?;
                (u > 1e-9 && u < 1.-1e-9 && self.on_end(s,e,u)).then_some(u)
            }),
            Read::Cap {..} => None,
        }
    }
}

impl Reader<'_> {
    /// Whether edge `e` lies along the circle the meridian of `s` turns at `u` (an end, or partway).
    fn on_end(&self,s: &RevolvedSurface,e: u32,u: f64) -> bool {
        let edge = &self.b.edges[e as usize];
        let EdgeCurve::Curve(curve) = &edge.curve else { return false };
        let a = s.axis_direction();
        let polar = |p: V| { let q = sub(p,s.origin()); let h = dot(q,a); [norm(sub(q,scale(a,h))),h] };
        let Ok(end) = s.at(u,0.) else { return false };
        let want = polar(end.position);
        (0..=4).all(|k| {
            let got = polar(curve.point(edge.t[0]+(edge.t[1]-edge.t[0])*k as f64/4.));
            (got[0]-want[0]).abs() <= 1e2*self.tol && (got[1]-want[1]).abs() <= 1e2*self.tol
        })
    }
}

impl Reader<'_> {
    /// **Where a revolution about `f`'s axis starts its chart**: its axis, and the direction
    /// square to it its meridian is laid along from a point of the axis, the frame's own `x`. Each
    /// face starts where its own surface's frame does, since nothing the sweep reads depends on
    /// where a chart starts (#721 F1d: `Tool::corner` probes a corner's second face at the first's
    /// half-plane, and the walk crosses from face to face by position).
    fn start(&self,f: &Frame) -> (V,V) { (f.z,f.x) }
}

/// **A tool's material read from its boundary**: below nought within it and above without, the
/// distance to the nearest point of its faces and edges, in units `1/scale` of the boundary's. Its
/// side is read off the nearest face's outward normal where the nearest point is a face's, off the
/// sum of its faces' where it is inside an edge, and off its angle-weighted pseudo-normal where it
/// is a corner's (Bærentzen and Aanæs 2005: each face's normal there weighted by the angle its
/// loop turns through at the corner); by the boundary's own point location only where none of
/// these is read (a point on the boundary, or a corner some face has no normal at, an apex).
pub struct Boundary {
    located: Located<'static>,
    scale: f64,
    /// Each edge's faces, and the edges that bound something (a seam, an edge a face uses twice,
    /// bounds nothing).
    faces: Vec<Vec<usize>>,
    bounding: Vec<u32>,
    /// Each vertex's angle-weighted pseudo-normal, where every face meeting there has a normal.
    corners: Vec<Option<V>>,
}

impl std::fmt::Debug for Boundary {
    fn fmt(&self,f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f,"Boundary {{ faces: {},scale: {},tol: {} }}",self.located.b.faces.len(),self.scale,self.located.tol)
    }
}

/// The point of a boundary nearest another: a face's foot at its parameters, or an edge's at its.
enum Foot { Face(usize,Uv),Edge(u32,f64) }

impl Boundary {
    /// `b`'s material, read in units `1/scale` of its own, its points within `tol` (its units) of
    /// a face on it; `uses` its edges' (`Brep::uses`).
    pub fn new(b: &Brep,scale: f64,tol: f64,uses: &[Vec<(usize,usize,usize)>]) -> Boundary {
        let seams: std::collections::BTreeSet<u32> = b.faces.iter().flat_map(Face::seams).collect();
        let faces = uses.iter().map(|u| u.iter().map(|x| x.0).collect::<std::collections::BTreeSet<_>>().into_iter().collect())
            .collect();
        let bounding = (0..b.edges.len() as u32).filter(|e| !seams.contains(e)).collect();
        Boundary {located:Located::owned(b.clone(),tol),scale,faces,bounding,corners:pseudo_normals(b)}
    }

    /// The distance from `q` (the boundary's units) to the boundary: to the foot of the nearest
    /// face it falls within (the faces asked nearest box first), or to the nearest edge that bounds
    /// something; and which side of it `q` stands on (below nought within), where a face's foot or
    /// an edge's inside is nearest.
    pub fn nearest(&self,q: V) -> (f64,Option<f64>) {
        let (b,l) = (&*self.located.b,&self.located);
        let mut gaps: Vec<(f64,usize)> = (0..b.faces.len()).map(|fi| (l.face_gap(fi,q),fi)).collect();
        gaps.sort_by(|x,y| x.0.total_cmp(&y.0));
        let (mut best,mut foot) = (f64::INFINITY,None);
        for (gap,fi) in gaps {
            if gap >= best { break }
            let surface = &b.faces[fi].surface;
            let uv = surface.inverse(q);
            let p = surface.point(uv);
            let d = distance(q,p);
            if d < best && l.face_place(fi,p) != Place::Out { (best,foot) = (d,Some(Foot::Face(fi,uv))) }
        }
        for &e in &self.bounding {
            if l.edge_gap(e,q) >= best { continue }
            let (t,d) = l.nearest(e,q);
            if d < best { (best,foot) = (d,Some(Foot::Edge(e,t))) }
        }
        let outward = |fi: usize,uv: Uv| { let f = &b.faces[fi]; f.surface.normal(uv).map(|n| if f.reversed { scale(n,-1.) } else { n }) };
        let side = match foot {
            Some(Foot::Face(fi,uv)) => outward(fi,uv).map(|n| dot(sub(q,b.faces[fi].surface.point(uv)),n)),
            // beside an edge's inside, the side of the sum of its faces' normals there (its
            // pseudo-normal); beside its end, a corner's, the side of the corner's
            Some(Foot::Edge(e,t)) => {
                let edge = &b.edges[e as usize];
                if t > edge.t[0] && t < edge.t[1] {
                    let x = edge.point(t,&b.vertices);
                    let n = self.faces[e as usize].iter().filter_map(|&fi| b.faces[fi].outward(x)).fold([0.;3],add);
                    Some(dot(sub(q,x),n))
                } else {
                    let v = edge.v[if t <= edge.t[0] { 0 } else { 1 }] as usize;
                    self.corners[v].map(|n| dot(sub(q,b.vertices[v].p),n))
                }.filter(|s| s.abs() > 0.)
            }
            None => None,
        };
        (best,side)
    }
}

/// **Each vertex's angle-weighted pseudo-normal**: the sum, over every corner a face's loop turns
/// at it, of the face's outward normal there weighted by the angle the face spans there — from the
/// use leaving the vertex round its normal to the use arriving, reversed, the loop walking with
/// the face on its left, so a reflex corner of a face weighs more than a half turn. The sign of a
/// point's offset from the vertex along it says which side of the boundary the point is on where
/// the vertex is its nearest point of the boundary. None at a vertex where a face has no normal, or
/// an edge has no curve (an apex, a pole), or where the weights cancel.
fn pseudo_normals(b: &Brep) -> Vec<Option<V>> {
    let mut sum: Vec<Option<V>> = vec![Some([0.;3]);b.vertices.len()];
    // a use's unit direction where it leaves its start (`end` 0) or arrives at its end (1)
    let direction = |c: &crate::topo::Coedge,end: usize| -> Option<V> {
        let e = &b.edges[c.edge as usize];
        let EdgeCurve::Curve(curve) = &e.curve else { return None };
        let at = if c.reversed { e.t[1-end] } else { e.t[end] };
        let d = curve.tangent(at);
        normalised(if c.reversed { scale(d,-1.) } else { d })
    };
    for f in &b.faces {
        for l in &f.loops {
            for (i,c) in l.iter().enumerate() {
                let next = &l[(i+1)%l.len()];
                let v = b.ends(c)[1] as usize;
                if b.ends(next)[0] as usize != v { continue }
                let p = b.vertices[v].p;
                let read = (|| -> Option<V> {
                    let n = f.outward(p)?;
                    // each direction laid in the face's tangent plane there
                    let flat = |d: V| normalised(sub(d,scale(n,dot(d,n))));
                    let (out,back) = (flat(direction(next,0)?)?,flat(scale(direction(c,1)?,-1.))?);
                    let angle = dot(n,cross(out,back)).atan2(dot(out,back)).rem_euclid(TAU);
                    Some(scale(n,angle))
                })();
                sum[v] = match (sum[v],read) { (Some(s),Some(w)) => Some(add(s,w)),_ => None };
            }
        }
    }
    sum.into_iter().map(|s| s.and_then(normalised)).collect()
}

impl Material for Boundary {
    fn value(&self,p: V) -> f64 {
        let q = p.map(|x| x*self.scale);
        let (d,side) = self.nearest(q);
        // beside a face, an edge or a corner, the side of it; where none is read, where the point
        // stands
        let inside = match side {
            Some(s) => s < 0.,
            None if !(d > 1e-3*self.located.tol) => return d/self.scale,
            None => self.located.solid_place(q) == Place::In,
        };
        if inside { -d/self.scale } else { d/self.scale }
    }
}
