//! **The tool read from its B-rep** (#721 F1, #828 F1a): a sweep's tool as the kernel reads it off
//! the boundary it builds, by its faces' surfaces and the edges they share, never by how it was
//! drawn.
//! - A face of revolution spanning a whole turn is read about its own axis, and so is a plane
//!   bounded by circles about one centre (a disc or a ring, #684 O).
//! - A face whose one loop is two lines or two coaxial arcs joined end to end by two lines is a
//!   ruled strip, a prism's side, straight or drafted. The strips that share their rulings are read
//!   one way along them, so a box is a prism one way of the three.
//! - A face of revolution a Boolean trims to part of a turn is its surface's whole turn, and a
//!   plane sharing a circle with a revolution a flat of it (one about each axis it shares a circle
//!   with): every face is kept whole, the material telling the face from the rest, as the tool
//!   keeps the faces a Boolean hides (`Tool`).
//! - Any other plane is a cap.
//!
//! Corners are where two faces read share an edge at the ends of their meridians or edges, creases
//! where two strips share a rim and their rulings go on across it; each revolution about an axis
//! starts from the half-plane the first face about it does. The material is the boundary's own
//! (`Boundary`). What no reading takes is refused by its geometry, naming the face and a point of
//! it (`Unread`).
//!
//! Nothing switches to it yet: the solver still reads the tool off the drawing
//! (`SweepContacts::read`), and `tests/tool_reading.rs` holds the two readings to one tool.
#[allow(unused_imports)]
use crate::util::fmath::Det;
use crate::geom::{Curve,Frame,Surface,Uv};
use crate::query::{Located,Place};
use crate::topo::{unwrap,Brep,EdgeCurve,Face};
use crate::util::space::{add,cross,distance,dot,norm,normalised,scale,sub};
use super::member::Solid;
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
        let reader = Reader {b,tol,uses:b.uses(),axes:Default::default()};
        let order: Vec<usize> = (0..b.faces.len()).filter(|&fi| !matches!(b.faces[fi].surface,Surface::Plane(_)))
            .chain((0..b.faces.len()).filter(|&fi| matches!(b.faces[fi].surface,Surface::Plane(_)))).collect();
        let witness = |fi: usize| reader.witness(fi).map(|x| x/scale);
        // each face's reading: the faces of revolution whole round first (those curved before the
        // planes, so an axis starts its revolutions where a curved face about it does), then the
        // strips (again those curved first, whose rulings are their own), then what is left
        let mut read: Vec<Option<Read>> = vec![None;b.faces.len()];
        for &fi in &order { read[fi] = reader.revolved(fi,false).map(Read::Revolved) }
        let options: Vec<Vec<Strip>> = (0..b.faces.len())
            .map(|fi| if read[fi].is_some() { Vec::new() } else { reader.strips(fi) }).collect();
        for &seed in &order {
            if read[seed].is_some() || options[seed].is_empty() { continue }
            let Some(s) = options[seed].iter().find(|s| reader.fits(s,seed,&read)).cloned() else { continue };
            read[seed] = Some(Read::Strip(s));
            // the strips beside it along a ruling, or on across a crease, read the same way round
            let mut pending = vec![seed];
            while let Some(fi) = pending.pop() {
                let Some(Read::Strip(s)) = read[fi].clone() else { unreachable!("only a strip is walked from") };
                let along: Vec<u32> = s.rulings.iter().chain(&s.rims).copied().collect();
                for e in along {
                    for &(g,_,_) in &reader.uses[e as usize] {
                        if g == fi || read[g].is_some() || options[g].is_empty() { continue }
                        let Some(t) = options[g].iter().find(|t| (t.rulings.contains(&e) || reader.continues(&s,t,e))
                            && reader.fits(t,g,&read)).cloned() else { continue };
                        read[g] = Some(Read::Strip(t));
                        pending.push(g);
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
        let mut more: Vec<(usize,Read)> = Vec::new();
        for fi in 0..b.faces.len() {
            if read[fi].is_some() { continue }
            let mut flats = reader.flats(fi,&read).into_iter().map(Read::Revolved);
            read[fi] = flats.next();
            more.extend(flats.map(|r| (fi,r)));
        }
        for fi in 0..b.faces.len() {
            if read[fi].is_some() { continue }
            let f = &b.faces[fi];
            read[fi] = Some(match &f.surface {
                Surface::Plane(frame) => reader.cap(fi,frame,&read),
                Surface::Revolution(_,c) if matches!(**c,Curve::Line {..} | Curve::Circle(..)) =>
                    return Err(Unread::Unfit {face:f.name.clone(),kind:f.surface.kind(),at:witness(fi)}),
                Surface::Cylinder(..) | Surface::Cone(..) | Surface::Sphere(..) | Surface::Torus(..) =>
                    return Err(Unread::Unfit {face:f.name.clone(),kind:f.surface.kind(),at:witness(fi)}),
                Surface::Revolution(_,c) => return Err(Unread::Meridian {face:f.name.clone(),kind:c.kind(),at:witness(fi)}),
                Surface::Extrusion(..) | Surface::Blend(..) | Surface::BSpline(..) | Surface::Family(..) | Surface::Affine(..) =>
                    return Err(Unread::FreeForm {face:f.name.clone(),kind:f.surface.kind(),at:witness(fi)}),
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
        Ok(Tool {faces,edges,creases,material:Arc::new(Boundary::new(b,scale,tol))})
    }
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
            Read::Revolved(s) => ToolSurface::Revolved(scaled_revolution(&s,f.name.clone(),scale)),
            Read::Strip(s) => ToolSurface::Extruded(ExtrudedSurface::new(f.name.clone(),edge(s.edge),match s.side {
                Side::Straight(d) => Side::Straight(v(d)),
                Side::Leaning(far) => Side::Leaning(edge(far)),
            })),
            Read::Cap {end,corner,du,dv} => ToolSurface::Cap(CapSurface::new(f.name.clone(),end,v(corner),v(du),v(dv))),
        }
    }
}

/// A revolution read in the boundary's units, rewritten in units `1/scale` of them.
fn scaled_revolution(s: &RevolvedSurface,name: String,scale: f64) -> RevolvedSurface {
    let k = 1./scale;
    let v = |x: V| x.map(|c| c*k);
    let meridian = match *s.meridian() {
        Meridian::Line {start,delta} => Meridian::Line {start:v(start),delta:v(delta)},
        Meridian::Round {center,a,b,sweep} => Meridian::Round {center:v(center),a:v(a),b:v(b),sweep},
        Meridian::Conic {a,m,b,w} => Meridian::Conic {a:v(a),m:v(m),b:v(b),w},
    };
    RevolvedSurface::new(name,meridian,v(s.origin()),s.axis_direction(),s.sweep())
}

/// A face read as a ruled strip: its edge at `v = 0` (the use `near` of its loop, walked as the
/// loop walks it), how it runs to its edge at `v = 1`, the edges at its ends in `u` (`rulings`, at
/// `u = 0` then `u = 1`) and at its ends in `v` (`rims`), and the vertex each ruling leaves its near
/// rim from.
#[derive(Clone,Debug)]
struct Strip { edge: Edge,side: Side,rulings: [u32;2],rims: [u32;2],bases: [u32;2],direction: [V;2] }

/// What reads a boundary: it, the tolerance its points are matched to, each edge's uses, and the
/// axes its revolutions turn about, each with the half-plane they start from (`start`).
struct Reader<'a> {
    b: &'a Brep,
    tol: f64,
    uses: Vec<Vec<(usize,usize,usize)>>,
    axes: std::cell::RefCell<Vec<(V,V,V)>>,
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
    fn parameters(&self,fi: usize) -> [[f64;2];2] {
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
        let whole = u1-u0 >= TAU*(1.-1e-9);
        let read = match &f.surface {
            Surface::Plane(_) if part => return None,
            Surface::Plane(frame) => return self.disc(fi,frame),
            _ if whole == part => return None,
            Surface::Cylinder(fr,r) => {
                let ((_,a,x),o) = (self.start(fr),fr.o);
                Some((o,a,Meridian::Line {start:add(add(o,scale(x,*r)),scale(fr.z,v0)),delta:scale(fr.z,v1-v0)}))
            }
            Surface::Cone(fr,r,alpha) => {
                let ((_,a,x),o) = (self.start(fr),fr.o);
                let (sn,cs) = alpha.dsin_cos();
                let at = |v: f64| add(o,add(scale(x,r+v*sn),scale(fr.z,v*cs)));
                Some((o,a,Meridian::Line {start:at(v0),delta:sub(at(v1),at(v0))}))
            }
            Surface::Sphere(fr,r) => { let ((_,a,x),o) = (self.start(fr),fr.o); Some((o,a,Meridian::round(o,x,fr.z,*r,v0,v1-v0))) }
            Surface::Torus(fr,big,r) => {
                let ((_,a,x),o) = (self.start(fr),fr.o);
                Some((o,a,Meridian::round(add(o,scale(x,*big)),x,fr.z,*r,v0,v1-v0)))
            }
            Surface::Revolution(fr,c) => {
                let ((_,a,x),o) = (self.start(fr),fr.o);
                // the curve's point in its half-plane, `ρ` along the frame's `x` and `h` along its
                // axis, set in the half-plane every revolution starts from
                let lift = |p: V| { let q = sub(p,fr.o); add(o,add(scale(x,dot(q,fr.x)),scale(fr.z,dot(q,fr.z)))) };
                let dir = |d: V| add(scale(x,dot(d,fr.x)),scale(fr.z,dot(d,fr.z)));
                match **c {
                    Curve::Line {..} => {
                        let (p,q) = (lift(c.point(v0)),lift(c.point(v1)));
                        Some((o,a,Meridian::Line {start:p,delta:sub(q,p)}))
                    }
                    Curve::Circle(cf,r) => Some((o,a,Meridian::round(lift(cf.o),dir(cf.x),dir(cf.y),r,v0,v1-v0))),
                    _ => None,
                }
            }
            _ => None,
        };
        let (o,a,meridian) = read?;
        let s = RevolvedSurface::new(f.name.clone(),meridian,o,a,TAU);
        self.outward(fi,s)
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
        if !(outer-inner > self.tol) { return None }
        let ((_,a,x),o) = (self.start(&Frame {o,..*frame}),o);
        let s = RevolvedSurface::new(f.name.clone(),Meridian::Line {start:add(o,scale(x,inner)),delta:scale(x,outer-inner)},o,a,TAU);
        self.outward(fi,s)
    }

    /// Plane face `fi` as a flat of each revolution it shares a circle with, about that circle's
    /// axis, over the least and greatest radii its loops reach about it: one for each axis.
    fn flats(&self,fi: usize,read: &[Option<Read>]) -> Vec<RevolvedSurface> {
        let f = &self.b.faces[fi];
        let Surface::Plane(frame) = &f.surface else { return Vec::new() };
        let mut axes: Vec<V> = Vec::new();
        for c in f.loops.iter().flatten() {
            let EdgeCurve::Curve(Curve::Circle(cf,_)) = &self.b.edges[c.edge as usize].curve else { continue };
            if norm(cross(cf.z,frame.z)) > 1e-9 || axes.iter().any(|&o| distance(o,cf.o) <= 1e2*self.tol) { continue }
            if self.uses[c.edge as usize].iter().any(|&(g,_,_)| g != fi && matches!(&read[g],Some(Read::Revolved(s))
                if norm(cross(s.axis_direction(),cf.z)) <= 1e-9 && norm(cross(sub(s.origin(),cf.o),cf.z)) <= 1e2*self.tol)) {
                axes.push(cf.o)
            }
        }
        axes.into_iter().filter_map(|axis| {
            let (mut inner,mut outer) = (f64::INFINITY,0_f64);
            for c in f.loops.iter().flatten() {
                let e = &self.b.edges[c.edge as usize];
                for k in 0..=32 {
                    let r = distance(e.point(e.t[0]+(e.t[1]-e.t[0])*k as f64/32.,&self.b.vertices),axis);
                    (inner,outer) = (inner.min(r),outer.max(r));
                }
            }
            if inner <= 1e2*self.tol { inner = 0. }
            if !(outer-inner > self.tol) { return None }
            let ((_,a,x),o) = (self.start(&Frame {o:axis,..*frame}),axis);
            let meridian = Meridian::Line {start:add(o,scale(x,inner)),delta:scale(x,outer-inner)};
            self.outward(fi,RevolvedSurface::new(f.name.clone(),meridian,o,a,TAU))
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
                    Some(Edge::round(cf.o,*r,start,sweep,|x,y| add(scale(cf.x,x),scale(cf.y,y))))
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
            let direction = [ruling(0.),ruling(1.)];
            out.push(Strip {edge:near,side,rulings:[l[(i+3)%4].edge,l[(i+1)%4].edge],rims:[l[i%4].edge,l[(i+2)%4].edge],
                bases:[p0,p1],direction});
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
        let along = |x: &Strip| add(x.direction[0],x.direction[1]);
        dot(along(s),along(t)) > 0.5*norm(along(s))*norm(along(t))
    }

    /// Plane face `fi` as a cap: the box its loops lie in within its plane, a little past them,
    /// `du × dv` its outward normal; at the strips' far end where it meets a strip's far rim.
    fn cap(&self,fi: usize,frame: &Frame,read: &[Option<Read>]) -> Read {
        let f = &self.b.faces[fi];
        let (mut lo,mut hi) = ([f64::INFINITY;2],[f64::NEG_INFINITY;2]);
        for c in f.loops.iter().flatten() {
            let e = &self.b.edges[c.edge as usize];
            for k in 0..=16 {
                let p = sub(e.point(e.t[0]+(e.t[1]-e.t[0])*k as f64/16.,&self.b.vertices),frame.o);
                let q = [dot(p,frame.x),dot(p,frame.y)];
                lo = [lo[0].min(q[0]),lo[1].min(q[1])];
                hi = [hi[0].max(q[0]),hi[1].max(q[1])];
            }
        }
        // a little past the loops' points, which their arcs bulge beyond
        let pad = 0.02*(hi[0]-lo[0]).max(hi[1]-lo[1]);
        let (lo,hi) = (lo.map(|x| x-pad),hi.map(|x| x+pad));
        let lift = |x: f64,y: f64| add(frame.o,add(scale(frame.x,x),scale(frame.y,y)));
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
            Read::Revolved(s) => [0.,1.].into_iter().find(|&u| self.on_end(s,e,u)),
            Read::Cap {..} => None,
        }
    }
}

impl Reader<'_> {
    /// Whether edge `e` lies along the circle the end `u` of `s`'s meridian turns.
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
    /// **A revolution's axis, set as every revolution about it reads it**: a point of it, its
    /// direction, and the direction square to it (a face's meridian is laid from a point of the
    /// axis its own, along that direction) every revolution's meridian about it starts from —
    /// the first face's about that line, its frame's `x` (for a face a revolution built, its
    /// profile's half-plane) — so faces about one axis share their charts' start, where the walk
    /// reads two faces' corner (`Tool::corner`) and where it starts round a ring.
    fn start(&self,f: &Frame) -> (V,V,V) {
        let mut axes = self.axes.borrow_mut();
        for &(o,a,x) in axes.iter() {
            if norm(cross(a,f.z)) <= 1e-9 && norm(cross(sub(f.o,o),a)) <= 1e2*self.tol { return (o,a,x) }
        }
        let x = normalised(sub(f.x,scale(f.z,dot(f.x,f.z)))).unwrap_or(f.x);
        axes.push((f.o,f.z,x));
        (f.o,f.z,x)
    }
}

/// **A tool's material read from its boundary**: below nought within it and above without, the
/// distance to the nearest point of its faces and edges, in units `1/scale` of the boundary's. Its
/// side is read off the nearest face's outward normal where the nearest point is a face's, off the
/// sum of its faces' where it is inside an edge; where it is a corner's, off a mesh of the boundary
/// where the point stands clear of it by more than the mesh's sag, and by the boundary's own point
/// location nearer.
pub struct Boundary { located: Located<'static>,mesh: Option<Solid>,scale: f64,tol: f64,
    /// Each edge's uses, and the edges a face uses twice (its seams, which bound nothing).
    uses: Vec<Vec<(usize,usize,usize)>>,seams: std::collections::BTreeSet<u32> }

impl std::fmt::Debug for Boundary {
    fn fmt(&self,f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f,"Boundary {{ faces: {},scale: {},tol: {} }}",self.located.b.faces.len(),self.scale,self.tol)
    }
}

impl Boundary {
    /// `b`'s material, read in units `1/scale` of its own, its points within `tol` (its units) of
    /// a face on it.
    pub fn new(b: &Brep,scale: f64,tol: f64) -> Boundary {
        let size = b.size();
        let mesh = Solid::new(b,(1e-3*size).max(tol),1.).ok();
        let seams = b.faces.iter().flat_map(Face::seams).collect();
        Boundary {located:Located::owned(b.clone(),tol),mesh,scale,tol,uses:b.uses(),seams}
    }

    /// The distance from `q` (the boundary's units) to the boundary: to the foot of the nearest
    /// face it falls within, or to the nearest edge (a seam bounds nothing); and which side of it
    /// `q` stands on (below nought within), where a face's foot or an edge's inside is nearest.
    pub fn nearest(&self,q: V) -> (f64,Option<f64>) {
        let b = &*self.located.b;
        let (mut best,mut side) = (f64::INFINITY,None);
        for (fi,f) in b.faces.iter().enumerate() {
            if self.located.face_gap(fi,q) >= best { continue }
            let foot = f.surface.point(f.surface.inverse(q));
            let d = distance(q,foot);
            if d < best && self.located.face_place(fi,foot) != Place::Out {
                best = d;
                side = f.outward(foot).map(|n| dot(sub(q,foot),n));
            }
        }
        for e in 0..b.edges.len() as u32 {
            if self.seams.contains(&e) || self.located.edge_gap(e,q) >= best { continue }
            let (t,d) = self.located.nearest(e,q);
            if !(d < best) { continue }
            best = d;
            // beside an edge's inside, the side of the sum of its faces' normals there (its
            // pseudo-normal); beside its end, a corner's, none
            let edge = &b.edges[e as usize];
            side = (t > edge.t[0] && t < edge.t[1]).then(|| {
                let x = edge.point(t,&b.vertices);
                let faces: std::collections::BTreeSet<usize> = self.uses[e as usize].iter().map(|u| u.0).collect();
                let n = faces.iter().filter_map(|&fi| b.faces[fi].outward(x)).fold([0.;3],add);
                dot(sub(q,x),n)
            }).filter(|s| s.abs() > 0.);
        }
        (best,side)
    }
}

impl Material for Boundary {
    fn value(&self,p: V) -> f64 {
        let q = p.map(|x| x*self.scale);
        let (d,side) = self.nearest(q);
        // beside a face, the side of it; beside an edge or a corner, where the point stands
        let inside = match side {
            Some(s) => s < 0.,
            None if !(d > 1e-3*self.tol) => return d/self.scale,
            None => match &self.mesh {
                Some(m) if m.distance(q) > 2.*m.sag+self.tol => m.winding(q) > 0.5,
                _ => self.located.solid_place(q) == Place::In,
            },
        };
        if inside { -d/self.scale } else { d/self.scale }
    }
}
