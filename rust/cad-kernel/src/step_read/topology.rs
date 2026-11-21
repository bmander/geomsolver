//! A solid walked from its shells down into a `Brep`: faces, their bounds, each bound's oriented
//! edges, the edges and the vertices, each read once however often it is used, so the boundary's
//! identity is the file's own and never found by proximity.
//!
//! **An edge is trimmed by its vertices.** STEP bounds an edge by its two vertices, not by
//! parameters: each end's parameter is the point of the edge's curve nearest its vertex (in
//! closed form on a line or a conic, searched on a spline), a conic's arc running on from the
//! first to the second (a whole turn where the two are one vertex), a closed spline's whole
//! domain. An edge whose curve runs against it (`same_sense` false) is laid along its curve, its
//! vertices taken the other way round, and every use of it turned.
//!
//! **A face's parameters are walked round each loop.** The kernel's pcurve of a use says where
//! the edge lies in its face's parameters; the branch of a periodic surface it lies on is the
//! loop's: each use starts where the last one ended, the edge's points read onto the surface one
//! after another, each on the branch nearest the last (`track`), so a seam's two uses fall either
//! side of it as the loop walks across. At a pole (a sphere's, a cone's apex) the collapsed
//! parameter is free, and the branch the next use leaves along is the one that keeps the face on
//! the loop's left (`pole_branch`). A loop that comes round to a whole period from where it began
//! winds round its surface without a seam: refused, not read yet. Each use's pcurve is then the
//! edge inverted onto the surface between the ends it was walked to (`Pcurve::Inverse`), exact; on
//! a spline surface, whose inversion is a search, the file's own curve in the face's parameters
//! where it lies on the edge over the edge's parameter (`file_pcurve`), as `Pcurve::Line` or
//! `Pcurve::Curve`. A file that writes no curve in the face's parameters is read the same way.
//!
//! **The outer loop is the one enclosing the most** of the face's parameters (STEP's
//! `FACE_OUTER_BOUND` is optional, and Open CASCADE writes none), the others moved by whole periods
//! into its span; a pole's degenerate edge, which no writer writes, is laid by `Brep::close_poles`;
//! a whole torus written as vertex loops alone has its two seams laid. Tolerances are measured
//! and the boundary checked (`Brep::check`), a failure refused naming the face or edge it names.
#[allow(unused_imports)]
use crate::util::fmath::Det;
use super::context::Found;
use super::geometry::{list,logical,Flat,Geometry,Sheet};
use super::{Given,Refusal,Why};
use crate::geom::{Curve,Frame,Surface,Uv,V};
use crate::nurbs::BSpline;
use crate::step_file::{File,Value};
use crate::topo::{pole_at,poleless as topo_poleless,reverse_loop,unwrap,Brep,Coedge,Edge,EdgeCurve,Face,Pcurve,Vertex};
use crate::util::space::{add,cross,distance,norm,scale,sub};
use std::collections::{BTreeMap,BTreeSet};
use std::f64::consts::TAU;
use std::sync::Arc;

/// A use of an edge by a loop, as the file orients it: the edge's index, whether along the
/// file's edge (from its first vertex to its second), and the oriented edge's entity.
#[derive(Clone,Copy,Debug)]
struct Use { edge: usize,along: bool }

/// A face's bound: a loop of edges, or a lone vertex.
enum Bound { Edges(Vec<Use>),Vertex(u32,usize) }

/// A face as walked: its entity, its surface's, whether its material normal is its surface's
/// (`same_sense`, turned where its shell is), its name and its bounds, each already turned as the
/// face uses it.
struct Walked { entity: u32,surface: u32,reversed: bool,name: String,bounds: Vec<Bound> }

/// An edge as the file writes it: its entity, its first and last vertex, its geometry's entity
/// and whether it runs along its curve.
struct Written { entity: u32,ends: [usize;2],geometry: u32,same_sense: bool }

/// An edge read: the kernel's, whether it runs against the file's (its vertices taken the other
/// way round), the kernel's parameter per the file's along its curve, and its curves in faces'
/// parameters as the file writes them, by the surface's entity.
struct Read { edge: Edge,flip: bool,rate: f64,pcurves: Vec<(u32,Flat)> }

/// The topology walked from a solid down, each edge and vertex numbered once by its entity.
struct Walk<'g,'f,'a> {
    g: &'g Geometry<'f,'a>,
    edges: BTreeMap<u32,usize>,
    written: Vec<Written>,
    vertices: BTreeMap<u32,usize>,
    points: Vec<u32>,
}

impl Walk<'_,'_,'_> {
    fn vertex(&mut self,from: u32,r: Option<u32>) -> Result<usize,Refusal> {
        let (r,_,_) = self.g.of(from,r,&["VERTEX_POINT"],"a vertex")?;
        if let Some(&k) = self.vertices.get(&r) { return Ok(k) }
        self.points.push(r);
        self.vertices.insert(r,self.points.len()-1);
        Ok(self.points.len()-1)
    }
    fn edge(&mut self,from: u32,r: Option<u32>) -> Result<usize,Refusal> {
        let (r,_,v) = self.g.of(from,r,&["EDGE_CURVE"],"an edge")?;
        if let Some(&k) = self.edges.get(&r) { return Ok(k) }
        if v.len() != 5 { return Err(Refusal::malformed(r,"an edge of five parameters expected")) }
        let ends = [self.vertex(r,v[1].reference())?,self.vertex(r,v[2].reference())?];
        let geometry = v[3].reference().ok_or_else(|| Refusal::malformed(r,"an edge's curve: a reference expected"))?;
        let same_sense = logical(v.get(4),r,"an edge's sense")?;
        self.written.push(Written {entity:r,ends,geometry,same_sense});
        self.edges.insert(r,self.written.len()-1);
        Ok(self.written.len()-1)
    }
    /// An oriented edge: the edge it uses and whether along it (an oriented edge of an oriented
    /// edge composed).
    fn oriented(&mut self,from: u32,r: Option<u32>,depth: usize) -> Result<Use,Refusal> {
        let (r,_,v) = self.g.of(from,r,&["ORIENTED_EDGE"],"an edge of a loop")?;
        if v.len() != 5 { return Err(Refusal::malformed(r,"an oriented edge of five parameters expected")) }
        let along = logical(v.get(4),r,"an oriented edge's orientation")?;
        let element = v[3].reference();
        match element.map(|e| self.g.file.get(e).simple().map(|s| s.0)) {
            Some(Some("ORIENTED_EDGE")) if depth < 8 => {
                let inner = self.oriented(r,element,depth+1)?;
                Ok(Use {edge:inner.edge,along:inner.along == along})
            }
            _ => Ok(Use {edge:self.edge(r,element)?,along}),
        }
    }
    /// A face, `turned` where its shell or its oriented face uses it the other way.
    fn face(&mut self,from: u32,r: Option<u32>,turned: bool,depth: usize) -> Result<Walked,Refusal> {
        let (r,kind,v) = self.g.of(from,r,&["ADVANCED_FACE","FACE_SURFACE","ORIENTED_FACE"],"a face of a shell")?;
        if kind == "ORIENTED_FACE" {
            if depth > 8 || v.len() != 4 { return Err(Refusal::malformed(r,"an oriented face of four parameters expected")) }
            let along = logical(v.get(3),r,"an oriented face's orientation")?;
            return self.face(r,v[2].reference(),turned != !along,depth+1)
        }
        if v.len() != 4 { return Err(Refusal::malformed(r,"a face of four parameters expected")) }
        let surface = v[2].reference().ok_or_else(|| Refusal::malformed(r,"a face's surface: a reference expected"))?;
        let same_sense = logical(v.get(3),r,"a face's sense")?;
        let mut bounds = Vec::new();
        for bound in list(v.get(1),r,"a face's bounds")? {
            let (b,_,bv) = self.g.of(r,bound.reference(),&["FACE_BOUND","FACE_OUTER_BOUND"],"a face's bound")?;
            if bv.len() != 3 { return Err(Refusal::malformed(b,"a bound of three parameters expected")) }
            let orientation = logical(bv.get(2),b,"a bound's orientation")?;
            let Some(lp) = bv[1].reference() else { return Err(Refusal::malformed(b,"a bound's loop: a reference expected")) };
            match self.g.file.get(lp).simple() {
                Some(("EDGE_LOOP",lv)) => {
                    let mut uses = Vec::new();
                    for o in list(lv.get(1),lp,"a loop's edges")? { uses.push(self.oriented(lp,o.reference(),0)?); }
                    if uses.is_empty() { return Err(Refusal::malformed(lp,"a loop of no edge")) }
                    // a loop turned by its bound, or by its face's use, is walked the other way
                    if !orientation != turned {
                        uses.reverse();
                        for u in &mut uses { u.along = !u.along; }
                    }
                    // each edge ending at the vertex the next one starts at (the standard's
                    // `edge_loop`: no two vertices at one place taken for one)
                    let ends = |u: &Use| { let e = &self.written[u.edge].ends; if u.along { [e[0],e[1]] } else { [e[1],e[0]] } };
                    if let Some(k) = (0..uses.len()).find(|&k| ends(&uses[k])[1] != ends(&uses[(k+1)%uses.len()])[0]) {
                        return Err(Refusal::malformed(lp,format!("a loop whose edge #{} ends at a vertex the next does not start at",
                            self.written[uses[k].edge].entity)))
                    }
                    bounds.push(Bound::Edges(uses));
                }
                Some(("VERTEX_LOOP",lv)) => { let k = self.vertex(lp,lv.get(1).and_then(Value::reference))?; bounds.push(Bound::Vertex(lp,k)); }
                Some((kind,_)) => return Err(Refusal::unsupported(lp,format!("a face's bound of the kind {kind}"))),
                None => return Err(Refusal::malformed(b,format!("a bound's loop #{lp} is {}",self.g.kinds(lp)))),
            }
        }
        if bounds.is_empty() { return Err(Refusal::malformed(r,"a face with no bound")) }
        let name = self.g.file.get(r).simple().and_then(|(_,v)| v.first()).and_then(Value::text).unwrap_or("").to_string();
        Ok(Walked {entity:r,surface,reversed:!same_sense != turned,name,bounds})
    }
    /// A closed shell's faces, each `turned` where the shell is.
    fn shell(&mut self,from: u32,r: Option<u32>,turned: bool,out: &mut Vec<Walked>) -> Result<(),Refusal> {
        let Some(r) = r else { return Err(Refusal::malformed(from,"a shell: a reference expected")) };
        match self.g.file.get(r).simple() {
            Some(("CLOSED_SHELL",v)) => {
                for f in list(v.get(1),r,"a shell's faces")? { out.push(self.face(r,f.reference(),turned,0)?); }
                Ok(())
            }
            Some(("ORIENTED_CLOSED_SHELL",v)) if v.len() == 4 => {
                let along = logical(v.get(3),r,"an oriented shell's orientation")?;
                self.shell(r,v[2].reference(),turned != !along,out)
            }
            Some(("OPEN_SHELL" | "ORIENTED_OPEN_SHELL",_)) => Err(Refusal::new(Why::Open,Some(r),"a solid bounded by an open shell")),
            _ => Err(Refusal::malformed(from,format!("a solid's shell #{r} is {}",self.g.kinds(r)))),
        }
    }
}

/// The parameters of an edge's ends on its curve, the points `p` of its first and last vertex
/// (`one` where they are one vertex): refused where the edge runs against its curve.
fn parameters(c: &Curve,p: [V;2],one: bool,entity: u32,within: f64) -> Result<[f64;2],Refusal> {
    match c {
        Curve::Circle(..) | Curve::Ellipse(..) => {
            let t0 = c.inverse(p[0]);
            let mut t1 = c.inverse(p[1]);
            if one || t1 <= t0+1e-15 { while t1 <= t0+1e-15 { t1 += TAU; } }
            Ok([t0,t1])
        }
        Curve::BSpline(b) => {
            let [a,z] = b.domain();
            let shut = distance(b.point(a),b.point(z)) <= within;
            if one {
                if !shut { return Err(Refusal::malformed(entity,"a closed edge on a spline that does not close")) }
                if distance(b.point(a),p[0]) > within {
                    return Err(Refusal::unsupported(entity,"a closed edge whose vertex is not at its closed spline's ends"))
                }
                return Ok([a,z])
            }
            let (mut t0,mut t1) = (c.inverse(p[0]),c.inverse(p[1]));
            if shut && t1 <= t0 {
                let end = 1e-9*(z-a);
                if (t0-z).abs() <= end { t0 = a; }
                if (t1-a).abs() <= end { t1 = z; }
            }
            if !(t1 > t0) { return Err(Refusal::malformed(entity,"an edge whose vertices run against its curve")) }
            Ok([t0,t1])
        }
        _ => {
            let (t0,t1) = (c.inverse(p[0]),c.inverse(p[1]));
            if one || !(t1 > t0) { return Err(Refusal::malformed(entity,"an edge whose vertices run against its curve, or along none of it")) }
            Ok([t0,t1])
        }
    }
}

/// An edge read: its curve, its vertices' parameters on it, and its curves in faces' parameters.
fn read_edge(g: &Geometry,w: &Written,points: &[V],within: f64) -> Result<Read,Refusal> {
    let r = w.entity;
    let (curve,pcurves) = match g.file.get(w.geometry).simple() {
        Some(("SURFACE_CURVE" | "SEAM_CURVE" | "INTERSECTION_CURVE",v)) => {
            let curve = g.curve(w.geometry,v.get(1).and_then(Value::reference))?;
            let mut pcurves = Vec::new();
            for a in list(v.get(2),w.geometry,"a curve's curves on its surfaces")? {
                let Some(a) = a.reference() else { return Err(Refusal::malformed(w.geometry,"a curve on a surface: a reference expected")) };
                // (a surface it lies on, named without a curve of its own, says nothing more)
                let Some(("PCURVE",pv)) = g.file.get(a).simple() else { continue };
                let surface = pv.get(1).and_then(Value::reference).ok_or_else(|| Refusal::malformed(a,"a pcurve's surface: a reference expected"))?;
                let (d,_,dv) = g.of(a,pv.get(2).and_then(Value::reference),&["DEFINITIONAL_REPRESENTATION"],"a pcurve's curve")?;
                let item = list(dv.get(1),d,"a representation's items")?.first().and_then(Value::reference);
                match g.flat(d,item) {
                    Ok(f) => pcurves.push((surface,f)),
                    // a curve in a face's parameters the reader has no form of is not needed
                    Err(e) if e.why == Why::Unsupported => {}
                    Err(e) => return Err(e),
                }
            }
            (curve,pcurves)
        }
        _ => (g.curve(r,Some(w.geometry))?,Vec::new()),
    };
    // laid along its curve: from its first vertex where it runs along, else its last
    let v = if w.same_sense { w.ends } else { [w.ends[1],w.ends[0]] };
    let t = parameters(&curve.curve,[points[v[0]],points[v[1]]],v[0] == v[1],r,within)?;
    Ok(Read {edge:Edge {curve:EdgeCurve::Curve(curve.curve),t,v:[v[0] as u32,v[1] as u32],tol:0.},flip:!w.same_sense,rate:curve.rate,pcurves})
}

/// The file's curve of an edge in a spline face's parameters as the kernel's pcurve, read over
/// the kernel's parameter of the edge (`rate` of the file's, and moved by whole periods of a
/// closed curve to where its domain says): kept where its image lies as near the edge as the
/// surface does, within `near`, at every eighth of it, else none.
fn file_pcurve(flat: &Flat,e: &Edge,rate: f64,s: &Surface,vs: &[Vertex],near: f64) -> Option<Pcurve> {
    let period = match &e.curve { EdgeCurve::Curve(c) => c.period(),EdgeCurve::Degenerate => None };
    let shifts: Vec<f64> = match (period,flat.domain()) {
        (Some(p),Some([d0,d1])) => vec![((0.5*(e.t[0]+e.t[1])-rate*0.5*(d0+d1))/p).round()*p],
        (Some(p),None) => vec![0.,p,-p],
        (None,_) => vec![0.],
    };
    for shift in shifts {
        let pc = match flat {
            Flat::Line {p,d} => {
                let at = |t: f64| { let x = (t-shift)/rate; [p[0]+x*d[0],p[1]+x*d[1]] };
                Pcurve::Line {a:at(e.t[0]),b:at(e.t[1])}
            }
            Flat::Curve(Curve::BSpline(b)) => {
                let knots = b.knots.iter().map(|k| rate*k+shift).collect();
                Pcurve::Curve(Arc::new(Curve::BSpline(Arc::new(BSpline {knots,..(**b).clone()}))))
            }
            Flat::Curve(c) if rate == 1. => Pcurve::Curve(Arc::new(c.clone())),
            Flat::Curve(_) => return None,
        };
        // as near the edge as the surface is, within `near` (a file's edge may stand its
        // tolerance off its faces, and its curve in each lies under it)
        let on = (0..=8).all(|j| {
            let t = e.t[0]+(e.t[1]-e.t[0])*j as f64/8.;
            let (q,p) = (pc.at(t,e,s,vs),e.point(t,vs));
            let d = distance(s.point(q),p);
            d <= near || d <= s.foot_from(p,q).map_or(0.,|f| distance(s.point(f),p))+near
        });
        if on { return Some(pc) }
        if std::env::var_os("STEP_READ_DEBUG").is_some() {
            let g: Vec<String> = (0..=8).map(|j| { let t = e.t[0]+(e.t[1]-e.t[0])*j as f64/8.; let (q,p) = (pc.at(t,e,s,vs),e.point(t,vs));
                format!("{:?}/{:.2e}/{:?}",q,distance(s.point(q),p),s.foot_from(p,q).map(|f| distance(s.point(f),p))) }).collect();
            eprintln!("file pcurve off: t {:?} shift {shift} {g:?}",e.t);
        }
    }
    None
}

/// Where the file's curve `flat` has an edge's point at its parameter `t`, roughly: read at the
/// file's parameter `t / rate`, moved by whole periods of a closed curve into its domain — a
/// curve in a face's parameters need not run at its edge's own (a cone's, read across the axis),
/// but it lies on the branch of the surface its edge does.
fn roughly(flat: &Flat,e: &Edge,rate: f64,t: f64) -> Uv {
    let period = match &e.curve { EdgeCurve::Curve(c) => c.period(),EdgeCurve::Degenerate => None };
    let shift = match (period,flat.domain()) {
        (Some(p),Some([d0,d1])) => ((0.5*(e.t[0]+e.t[1])-rate*0.5*(d0+d1))/p).round()*p,
        _ => 0.,
    };
    let x = (t-shift)/rate;
    flat.at(match flat.domain() { Some([d0,d1]) => x.clamp(d0,d1),None => x })
}

/// The parameters of a use's points on `s`, `n` steps along `travel` (the edge's parameters in
/// the use's direction), read one after another from `from` (the start's, on the branch the loop
/// reached it on), each sought from `hint` of its step and the last (the last itself, but for a
/// seam walked again on its other side) and on the branch nearest that. At a pole the collapsed
/// parameter is read a step in from it (`topo::pole_at`).
fn track(s: &Surface,e: &Edge,vs: &[Vertex],travel: [f64;2],from: Uv,n: usize,within: f64,hint: &dyn Fn(usize,Uv) -> Uv) -> Vec<Uv> {
    let periods = s.periods();
    let mut out = vec![from];
    let pole = pole_at(s,from,within);
    for j in 1..=n {
        let t = travel[0]+(travel[1]-travel[0])*j as f64/n as f64;
        let at = out[j-1];
        let near = hint(j,at);
        let mut uv = unwrap(s.inverse_near(e.point(t,vs),near),near,periods);
        if j == n { if let Some(k) = pole_at(s,uv,within) { uv[k] = at[k]; } }
        if j == 1 { if let Some(k) = pole { out[0][k] = uv[k]; } }
        out.push(uv);
    }
    out
}

/// **The branch a use leaving a pole sets out on**: its collapsed parameter `k`'s value `x`
/// moved by whole periods `p` so that the pole's degenerate edge from `arrived`, the value the
/// last use arrived at, runs the way that keeps the face on the loop's left — along `+k` where
/// the face lies to the left of that, which is toward `+v` from a `u` pole (and toward `−u` from
/// a `v` pole) on a face whose material normal is its surface's, the other way on a reversed
/// one. `inward` is the sign the use's other parameter moves at as it leaves the pole. A seam
/// leaving where the last use arrived (its own other side) is a whole period on, not none.
fn pole_branch(x: f64,arrived: f64,p: f64,k: usize,inward: f64,reversed: bool,seam: bool) -> f64 {
    let along = inward*if k == 0 { 1. } else { -1. }*if reversed { -1. } else { 1. };
    let mut gap = ((x-arrived)*along).rem_euclid(p);
    if gap > p*(1.-1e-9) { gap = 0.; }
    if seam && gap < p*1e-9 { gap = p; }
    arrived+along*gap
}

/// `pc` moved by `d` in its face's parameters.
fn moved(pc: &Pcurve,d: Uv) -> Pcurve {
    let m = |a: &Uv| [a[0]+d[0],a[1]+d[1]];
    match pc {
        Pcurve::Line {a,b} => Pcurve::Line {a:m(a),b:m(b)},
        Pcurve::Inverse {a,b} => Pcurve::Inverse {a:m(a),b:m(b)},
        Pcurve::Curve(c) => Pcurve::Curve(Arc::new(match &**c {
            Curve::BSpline(s) => Curve::BSpline(Arc::new(BSpline {poles:s.poles.iter().map(|p| [p[0]+d[0],p[1]+d[1],p[2]]).collect(),..(**s).clone()})),
            Curve::Circle(f,r) => Curve::Circle(Frame {o:[f.o[0]+d[0],f.o[1]+d[1],f.o[2]],..*f},*r),
            Curve::Ellipse(f,a,b) => Curve::Ellipse(Frame {o:[f.o[0]+d[0],f.o[1]+d[1],f.o[2]],..*f},*a,*b),
            other => other.clone(),
        })),
    }
}

/// What a face is read in: the edges and vertices read, the tolerances a point is taken to stand
/// at a pole within and a file's curve to lie on its edge within.
struct Setting<'r> { b: &'r Brep,reads: &'r [Read],within: f64,near: f64 }

/// **A loop walked in its face's parameters** (the module's doc): each use's pcurve, its ends on
/// the branch the walk reached them on.
fn walk_loop(st: &Setting,sheet: &Sheet,surface: u32,uses: &[(usize,bool)],reversed: bool,seams: &BTreeSet<usize>,face: u32,
    taken: &mut BTreeMap<usize,usize>,placed: &mut BTreeMap<usize,Uv>) -> Result<(Vec<Coedge>,Option<Wound>),Refusal> {
    let s = &sheet.surface;
    let (b,periods) = (st.b,s.periods());
    let spline = matches!(s,Surface::BSpline(..));
    let steps = if matches!(s,Surface::Plane(_)) { 2 } else { 16 };
    let mut out = Vec::with_capacity(uses.len());
    let mut last: Option<Uv> = None;
    let mut first = None;
    for &(ei,rev) in uses {
        let e = &b.edges[ei];
        let travel = if rev { [e.t[1],e.t[0]] } else { e.t };
        let begin = b.vertices[e.v[rev as usize] as usize].p;
        // the file's own curve, moved by whole periods to where the loop stands, the nearest of
        // them there (a seam's two on a spline); on a surface with poles, of a use reaching none,
        // whose ends at a pole the walk reads (`pole_branch`)
        let end = b.vertices[e.v[!rev as usize] as usize].p;
        let free = spline || topo_poleless(s) || [begin,end].iter().all(|&p| pole_at(s,s.inverse(p),st.within).is_none());
        let given = if free {
            // (a seam's other use took the other of its two)
            let mine: Vec<&Flat> = st.reads[ei].pcurves.iter().filter(|(id,_)| *id == surface).map(|(_,f)| f).collect();
            let other = taken.get(&ei).copied().filter(|_| mine.len() > 1);
            let mut found: Vec<(Pcurve,Uv,usize)> = mine.iter().enumerate().filter(|&(k,_)| Some(k) != other)
                .filter_map(|(k,f)| Some((f.scaled(sheet.scale)?,k))).filter_map(|(f,k)| Some((file_pcurve(&f,e,st.reads[ei].rate,s,&b.vertices,st.near)?,k)))
                .map(|(pc,k)| {
                    let a = pc.at(travel[0],e,s,&b.vertices);
                    let Some(l) = last else { return (pc,a,k) };
                    let d: Uv = std::array::from_fn(|k| periods[k].map_or(0.,|p| ((l[k]-a[k])/p).round()*p));
                    if d == [0.;2] { (pc,a,k) } else { (moved(&pc,d),[a[0]+d[0],a[1]+d[1]],k) }
                }).collect();
            if let Some(l) = last { found.sort_by(|x,y| distance2(x.1,l).total_cmp(&distance2(y.1,l))); }
            found.into_iter().next().map(|(pc,a,k)| { taken.insert(ei,k); (pc,a) })
        } else { None };
        let (pcurve,start,end) = match given {
            Some((pc,a)) => { let z = pc.at(travel[1],e,s,&b.vertices); (pc,a,z) }
            None => {
                // where the walk begins, the vertex read near where the file's curve has it, where
                // the file gives one (the side of a closed spline's seam)
                let from = last.unwrap_or_else(|| {
                    let near = st.reads[ei].pcurves.iter().find(|(id,_)| *id == surface).map(|(_,f)| roughly(f,e,st.reads[ei].rate,travel[0]));
                    // (else from a step in along the edge, whose foot is the face's own where its
                    // start is on a closed sheet's seam)
                    let q = near.unwrap_or_else(|| s.inverse(e.point(travel[0]+(travel[1]-travel[0])/16.,&b.vertices)));
                    unwrap(s.inverse_near(begin,q),q,periods)
                });
                let mut walked = track(s,e,&b.vertices,travel,from,steps,st.within,&|_,at| at);
                // a seam of a sheet closed without a period, walked where its other use was: walked
                // again along the other side of the sheet's domain, each point sought from its
                // mirror there
                if let (Surface::BSpline(_,n),Some(&other)) = (s,placed.get(&ei)) {
                    let (dom,m) = (n.domain(),walked[steps/2]);
                    if (0..2).all(|k| (m[k]-other[k]).abs() <= 1e-6*(dom[k][1]-dom[k][0])) {
                        if let Some(k) = (0..2).find(|&k| { let [lo,hi] = dom[k]; (m[k]-lo).min(hi-m[k]) <= 1e-6*(hi-lo) }) {
                            let mirror = |mut q: Uv| { q[k] = dom[k][0]+dom[k][1]-q[k]; q };
                            let first = walked.clone();
                            walked = track(s,e,&b.vertices,travel,mirror(first[0]),steps,st.within,&|j,_| mirror(first[j]));
                        }
                    }
                }
                let [mut a,mut m,mut z] = [walked[0],walked[steps/2],walked[steps]];
                placed.insert(ei,m);
                // leaving a pole after another use: on the branch that keeps the face on the left
                if let (Some(l),Some(k)) = (last,pole_at(s,from,st.within)) {
                    if let Some(p) = periods[k] {
                        let x = pole_branch(a[k],l[k],p,k,(m[1-k]-a[1-k]).signum(),reversed,seams.contains(&ei));
                        let d = x-a[k];
                        for q in [&mut a,&mut m,&mut z] { q[k] += d; }
                    }
                }
                let (ea,eb) = if rev { (z,a) } else { (a,z) };
                (Pcurve::Inverse {a:ea,b:eb},a,z)
            }
        };
        if std::env::var_os("STEP_READ_DEBUG").is_some() { eprintln!("face #{face} use edge {ei} rev {rev} {} {start:?} -> {end:?}",match pcurve { Pcurve::Inverse {..} => "inverse",_ => "file" }); }
        first.get_or_insert((start,begin));
        out.push(Coedge {edge:ei as u32,reversed:rev,pcurve});
        last = Some(end);
    }
    // round to where it began, on the same branch, or round a whole period of `u`
    let mut wound = None;
    if let (Some(l),Some((a,p))) = (last,first) {
        let pole = pole_at(s,a,st.within);
        for k in 0..2 {
            let Some(period) = periods[k] else { continue };
            if pole == Some(k) || pole_at(s,l,st.within) == Some(k) { continue }
            if (l[k]-a[k]).abs() > 0.5*period {
                if k == 0 && matches!(s,Surface::Sphere(..) | Surface::Cone(..)) && (l[1]-a[1]).abs() <= 1e-3 {
                    wound = Some(Wound {at:a,turn:l[0]-a[0]});
                    continue
                }
                return Err(Refusal::unsupported(face,format!("a face whose bound winds round its surface's period with no seam, at {p:?}: \
                    a face whole round its surface without a seam edge is not read yet")))
            }
        }
    }
    Ok((out,wound))
}

/// A loop winding once round a sphere's or a cone's axis: where it begins in the face's
/// parameters, and how far round it comes (a whole turn, either way).
struct Wound { at: Uv,turn: f64 }

/// **The seam a loop winding round a pole is joined to it along** (a face holding a sphere's pole
/// or a cone's apex inside it, which a file writes with no seam): the meridian (a sphere's) or
/// generator (a cone's) from the loop's first vertex `v` to the pole the face holds — north of
/// the loop where it winds `+u` with the face's material normal its surface's, south where either
/// is turned — and the pole, laid; the loop walked on up the seam from where it came round to,
/// and back down it to where it began (`Brep::close_poles` lays the pole's edge between).
fn pole_seam(s: &Surface,w: &Wound,v: u32,reversed: bool,laid: u32) -> Option<(V,Edge,[Coedge;2])> {
    let north = (w.turn > 0.) != reversed;
    let u = w.at[0];
    let (pole,curve) = match *s {
        Surface::Sphere(f,r) => {
            let out = add(scale(f.x,u.dcos()),scale(f.y,u.dsin()));
            (if north { std::f64::consts::FRAC_PI_2 } else { -std::f64::consts::FRAC_PI_2 },
                Curve::Circle(Frame {o:f.o,x:out,y:f.z,z:cross(out,f.z)},r))
        }
        Surface::Cone(_,r,half) => {
            let apex = -r/half.dsin();
            // the apex lies north (toward +v) of the loop or south of it, as the face holds it
            if (apex > w.at[1]) != north { return None }
            let (p,q) = (s.point([u,apex]),s.point(w.at));
            let d = crate::util::space::normalised(sub(q,p))?;
            (apex,Curve::Line {p,d})
        }
        _ => return None,
    };
    let point = s.point([u,pole]);
    // the seam's own parameters at the vertex and at the pole, and which runs first
    let (tv,tp) = (curve.inverse(s.point(w.at)),curve.inverse(point));
    let tp = if let Curve::Circle(..) = curve { if north { std::f64::consts::FRAC_PI_2 } else { -std::f64::consts::FRAC_PI_2 } } else { tp };
    let tv = if let Curve::Circle(..) = curve { w.at[1] } else { tv };
    let up = tv < tp;
    let (t,ends) = if up { ([tv,tp],[v,laid]) } else { ([tp,tv],[laid,v]) };
    let edge = Edge {curve:EdgeCurve::Curve(curve),t,v:ends,tol:0.};
    let along = |u: f64| { let (a,b) = ([u,w.at[1]],[u,pole]); if up { Pcurve::Inverse {a,b} } else { Pcurve::Inverse {a:b,b:a} } };
    // up from where the loop came round to, and down to where it began
    let seam = LAID;
    Some((point,edge,[Coedge {edge:seam,reversed:!up,pcurve:along(u+w.turn)},Coedge {edge:seam,reversed:up,pcurve:along(u)}]))
}

fn distance2(a: Uv,b: Uv) -> f64 { (a[0]-b[0]).dhypot(a[1]-b[1]) }

/// The span of a loop's parameters: each use read at its ends and quarters.
fn span(b: &Brep,f: &Face,l: &[Coedge]) -> [[f64;2];2] {
    let mut out = [[f64::INFINITY,f64::NEG_INFINITY];2];
    for c in l {
        let e = &b.edges[c.edge as usize];
        for j in 0..=4 {
            let uv = c.pcurve.at(e.t[0]+(e.t[1]-e.t[0])*j as f64/4.,e,&f.surface,&b.vertices);
            for k in 0..2 { out[k][0] = out[k][0].min(uv[k]); out[k][1] = out[k][1].max(uv[k]); }
        }
    }
    out
}

/// A face built, the edges it lays (a whole torus's seams, a pole's seam), which its loops name
/// from `LAID` down, and the vertices it lays (a pole), which those edges name so.
struct Built { face: Face,laid: Vec<Edge>,poles: Vec<V> }

const LAID: u32 = u32::MAX;

/// A face read: its loops walked, the outer one first and the others moved into its span.
fn face(st: &Setting,sheet: &Sheet,w: &Walked) -> Result<Built,Refusal> {
    let b = st.b;
    let s = &sheet.surface;
    let reversed = w.reversed;
    let uses: Vec<Vec<(usize,bool)>> = w.bounds.iter().filter_map(|bound| match bound {
        Bound::Edges(u) => Some(u.iter().map(|u| (u.edge,!u.along != st.reads[u.edge].flip)).collect()),
        Bound::Vertex(..) => None,
    }).collect();
    let mut count: BTreeMap<usize,usize> = BTreeMap::new();
    for &(e,_) in uses.iter().flatten() { *count.entry(e).or_default() += 1; }
    let seams: BTreeSet<usize> = count.iter().filter(|(_,&n)| n > 1).map(|(&e,_)| e).collect();
    let used: BTreeSet<u32> = uses.iter().flatten().flat_map(|&(e,_)| b.edges[e].v).collect();
    // a lone vertex bounds nothing where a loop passes it or it is a pole; a torus bounded by
    // lone vertices alone is the whole torus
    for bound in &w.bounds {
        let Bound::Vertex(lp,v) = *bound else { continue };
        let p = b.vertices[v].p;
        if used.contains(&(v as u32)) || pole_at(s,s.inverse(p),st.within).is_some() { continue }
        if uses.is_empty() { if let Surface::Torus(..) = s { return Ok(whole_torus(s,p,v as u32,reversed,&w.name)) } }
        return Err(Refusal::unsupported(lp,"a face bounded by a lone vertex that is no pole"))
    }
    if uses.is_empty() { return Err(Refusal::unsupported(w.entity,"a face bounded by lone vertices alone")) }
    // each loop walked from a use that is no seam and starts at no pole, where it has one: where
    // the walk begins, the branch is the surface's inverse's, or the first of the file's curves
    let begins = |&(e,rev): &(usize,bool)| {
        let p = b.vertices[b.edges[e].v[rev as usize] as usize].p;
        (seams.contains(&e),pole_at(s,s.inverse(p),st.within).is_some())
    };
    let uses: Vec<Vec<(usize,bool)>> = uses.into_iter().map(|mut l| {
        let k = (0..l.len()).min_by_key(|&k| { let (seam,pole) = begins(&l[k]); (pole,seam,k) }).unwrap_or(0);
        l.rotate_left(k);
        l
    }).collect();
    let (mut taken,mut placed) = (BTreeMap::new(),BTreeMap::new());
    let walked = uses.iter().map(|l| walk_loop(st,sheet,w.surface,l,reversed,&seams,w.entity,&mut taken,&mut placed)).collect::<Result<Vec<_>,_>>()?;
    // a loop winding round a pole the face holds is joined to it by a seam laid (one loop alone:
    // two winding bound a band, which wants a seam between them)
    let (mut laid,mut poles) = (Vec::new(),Vec::new());
    if walked.iter().filter(|x| x.1.is_some()).count() > 1 {
        return Err(Refusal::unsupported(w.entity,"a face between two bounds winding round its surface, with no seam: not read yet"))
    }
    let mut loops = Vec::new();
    for ((mut l,wound),u) in walked.into_iter().zip(&uses) {
        if let Some(x) = wound {
            let v = b.edges[u[0].0].v[u[0].1 as usize];
            let Some((p,e,uses)) = pole_seam(s,&x,v,reversed,LAID) else {
                return Err(Refusal::unsupported(w.entity,"a face whose bound winds round its surface's period with no seam: not read yet"))
            };
            // (the edge laid names the pole laid as `LAID`, patched where it is numbered)
            laid.push(e);
            poles.push(p);
            l.extend(uses);
        }
        loops.push(l);
    }
    let mut f = Face {surface:s.clone(),reversed,loops,name:w.name.clone()};
    if std::env::var_os("STEP_READ_DEBUG").is_some() {
        for c in f.loops.iter().flatten() {
            if c.edge > LAID-16 { continue }
            let e = &b.edges[c.edge as usize];
            for j in 0..=32 { let t = e.t[0]+(e.t[1]-e.t[0])*j as f64/32.; let q = c.pcurve.at(t,e,s,&b.vertices); if !(q[0].is_finite() && q[1].is_finite()) { eprintln!("face #{} edge {} t {:?} at {t} {q:?} {:?}",w.entity,c.edge,e.t,c.pcurve); break } }
        }
    }
    // the outer loop first: the one enclosing the most of the face's parameters
    if f.loops.len() > 1 {
        let areas: Vec<f64> = f.loops.iter().map(|l| b.loop_area(&f,l).abs()).collect();
        let outer = (0..areas.len()).max_by(|&i,&j| areas[i].total_cmp(&areas[j])).unwrap_or(0);
        let l = f.loops.remove(outer);
        f.loops.insert(0,l);
        // the others moved by whole periods to lie nearest its middle
        let periods = s.periods();
        let o = span(b,&f,&f.loops[0]);
        for li in 1..f.loops.len() {
            let h = span(b,&f,&f.loops[li]);
            let mut d = [0.;2];
            for k in 0..2 {
                let Some(p) = periods[k] else { continue };
                d[k] = ((0.5*(o[k][0]+o[k][1]-h[k][0]-h[k][1]))/p).round()*p;
            }
            if d != [0.;2] { for c in &mut f.loops[li] { c.pcurve = moved(&c.pcurve,d); } }
        }
    }
    Ok(Built {face:f,laid,poles})
}

/// **A whole torus**, bounded by the vertex at `p` alone (Open CASCADE writes one so): its two
/// seams laid through it — the circle round the axis and the tube's circle — and walked once each
/// way round the square of its parameters.
fn whole_torus(s: &Surface,p: V,v: u32,reversed: bool,name: &str) -> Built {
    let Surface::Torus(f,big,r) = *s else { unreachable!("a torus") };
    let [u0,v0] = s.inverse(p);
    let (sv,cv) = v0.dsin_cos();
    let (su,cu) = u0.dsin_cos();
    let round = Curve::Circle(Frame {o:add(f.o,scale(f.z,r*sv)),..f},big+r*cv);
    let out = add(scale(f.x,cu),scale(f.y,su));
    let tube = Curve::Circle(Frame {o:add(f.o,scale(out,big)),x:out,y:f.z,z:cross(out,f.z)},r);
    let laid = vec![Edge {curve:EdgeCurve::Curve(round),t:[u0,u0+TAU],v:[v,v],tol:0.},Edge {curve:EdgeCurve::Curve(tube),t:[v0,v0+TAU],v:[v,v],tol:0.}];
    let line = |a: Uv,b: Uv| Pcurve::Line {a,b};
    let (a,bu,c,d) = ([u0,v0],[u0+TAU,v0],[u0+TAU,v0+TAU],[u0,v0+TAU]);
    let mut l = vec![Coedge {edge:LAID,reversed:false,pcurve:line(a,bu)},Coedge {edge:LAID-1,reversed:false,pcurve:line(bu,c)},
        Coedge {edge:LAID,reversed:true,pcurve:line(d,c)},Coedge {edge:LAID-1,reversed:true,pcurve:line(a,d)}];
    if reversed { l = reverse_loop(l); }
    Built {face:Face {surface:s.clone(),reversed,loops:vec![l],name:name.to_string()},laid,poles:Vec::new()}
}

/// The entity a check's message names: its first face or edge, by its index in the `Brep`.
fn witness(msg: &str,given: &Given) -> Option<u32> {
    for (word,of) in [("face ",&given.faces),("edge ",&given.edges)] {
        let mut at = 0;
        while let Some(k) = msg[at..].find(word) {
            let rest = &msg[at+k+word.len()..];
            let digits: String = rest.chars().take_while(char::is_ascii_digit).collect();
            if let Ok(i) = digits.parse::<usize>() { if let Some(&e) = of.get(i).filter(|&&e| e != 0) { return Some(e) } }
            at += k+word.len();
        }
    }
    None
}

/// The solid `found` names, read; refused naming the entity that could not be.
pub fn solid(file: &File,found: Found) -> Result<Given,Refusal> {
    let clock = std::time::Instant::now(); let lap = |w: &str| if std::env::var_os("STEP_READ_TIME").is_some() { eprintln!("{w} {:?}",clock.elapsed()) };
    let g = Geometry {file,context:found.context};
    let mut w = Walk {g:&g,edges:BTreeMap::new(),written:Vec::new(),vertices:BTreeMap::new(),points:Vec::new()};
    let mut faces = Vec::new();
    let r = found.solid;
    let e = file.get(r);
    match (e.simple(),e.part("MANIFOLD_SOLID_BREP")) {
        (Some(("BREP_WITH_VOIDS",v)),_) => {
            w.shell(r,v.get(1).and_then(Value::reference),false,&mut faces)?;
            for void in list(v.get(2),r,"a solid's voids")? { w.shell(r,void.reference(),false,&mut faces)?; }
        }
        (_,Some(v)) => w.shell(r,v.get(1).and_then(Value::reference),false,&mut faces)?,
        _ => return Err(Refusal::malformed(r,format!("a solid of the kinds {}",g.kinds(r)))),
    }
    // the vertices, and the size the tolerances are read against
    let points: Vec<V> = w.points.iter().map(|&v| {
        let (_,_,p) = g.of(v,Some(v),&["VERTEX_POINT"],"a vertex")?;
        g.point(v,p.get(1).and_then(Value::reference))
    }).collect::<Result<_,_>>()?;
    let (lo,hi) = points.iter().fold(([f64::INFINITY;3],[f64::NEG_INFINITY;3]),|(lo,hi),p| (std::array::from_fn(|k| lo[k].min(p[k])),std::array::from_fn(|k| hi[k].max(p[k]))));
    let size = if points.is_empty() { 0. } else { norm(sub(hi,lo)) };
    let uncertainty = found.context.uncertainty.unwrap_or(0.);
    let within = (1e-9*size).max(uncertainty);
    lap("walked");
    let reads = crate::util::par::indices(w.written.len(),|i| read_edge(&g,&w.written[i],&points,within.max(1e-7*size)))
        .into_iter().collect::<Result<Vec<_>,_>>()?;
    lap("edges");
    let mut b = Brep {vertices:points.iter().map(|&p| Vertex {p,tol:0.}).collect(),edges:reads.iter().map(|r| r.edge.clone()).collect(),faces:Vec::new()};
    // the surfaces, each read once
    let ids: Vec<u32> = faces.iter().map(|f| f.surface).collect::<BTreeSet<_>>().into_iter().collect();
    let sheets: BTreeMap<u32,Sheet> = ids.iter().copied().zip(crate::util::par::indices(ids.len(),|i| g.surface(ids[i],Some(ids[i])))
        .into_iter().collect::<Result<Vec<_>,_>>()?).collect();
    lap("surfaces");
    let built = {
        let st = Setting {b:&b,reads:&reads,within:1e-7*(1.+size),near:uncertainty.max(1e-6)};
        crate::util::par::indices(faces.len(),|i| face(&st,&sheets[&faces[i].surface],&faces[i])).into_iter().collect::<Result<Vec<_>,_>>()?
    };
    lap("faces");
    let mut edges: Vec<u32> = w.written.iter().map(|x| x.entity).collect();
    let mut vertices = w.points.clone();
    for mut x in built {
        let (at,pole) = (b.edges.len() as u32,b.vertices.len() as u32);
        for c in x.face.loops.iter_mut().flatten() { if c.edge > LAID-16 { c.edge = at+(LAID-c.edge); } }
        for e in &mut x.laid { for v in &mut e.v { if *v > LAID-16 { *v = pole+(LAID-*v); } } }
        b.edges.extend(x.laid.drain(..));
        for p in x.poles { b.vertices.push(Vertex {p,tol:0.}); vertices.push(0); }
        b.faces.push(x.face);
    }
    // a vertex at a pole stands as far off it as half a turn of the collapsed parameter moves
    // there: what `close_poles` reads the pole within
    let pole = 1e-7*(1.+size);
    let mut at_poles = Vec::new();
    for f in &b.faces {
        for c in f.loops.iter().flatten() {
            let ends = b.uv_ends(f,c);
            for (k,uv) in ends.into_iter().enumerate() {
                let Some(along) = pole_at(&f.surface,uv,pole) else { continue };
                let (_,su,sv) = f.surface.d1(uv);
                // (and as far as twice the farthest the uses' ends there stand from it, so two
                // of them are one point of it)
                let v = b.ends(c)[k] as usize;
                let off = f.loops.iter().flatten().flat_map(|d| { let e = b.uv_ends(f,d); let w = b.ends(d); [(w[0],e[0]),(w[1],e[1])] })
                    .filter(|&(w,_)| w as usize == v).map(|(_,uv)| distance(f.surface.point(uv),b.vertices[v].p)).fold(0.,f64::max);
                at_poles.push((v,(std::f64::consts::PI*norm([su,sv][along])).max(2.*off)*(1.+1e-9)));
            }
        }
    }
    for (v,d) in at_poles { b.vertices[v].tol = b.vertices[v].tol.max(d); }
    lap("poles marked");
    b.close_poles();
    lap("poles closed");
    // (twice: a vertex's tolerance, measured, is how far an inverse curve's end reads as on a pole
    // there, which the edges' tolerances are then measured with)
    b.measure_tolerances();
    b.measure_tolerances();
    // (a foot searched for on a spline reads the last bits of a gap differently from one reading to
    // the next: the gap measured is held to a billionth more)
    for e in &mut b.edges { e.tol *= 1.+1e-9; }
    edges.resize(b.edges.len(),0);
    if std::env::var_os("STEP_READ_DEBUG").is_some() {
        for (i,e) in b.edges.iter().enumerate().filter(|(_,e)| e.tol > 1e-4) {
            let EdgeCurve::Curve(c) = &e.curve else { continue };
            let ends = [0,1].map(|k| distance(c.point(e.t[k]),b.vertices[e.v[k] as usize].p));
            eprintln!("edge {i} #{} {} t {:?} tol {:e} ends {ends:?}",edges[i],c.kind(),e.t,e.tol);
            for f in &b.faces { for u in f.loops.iter().flatten().filter(|u| u.edge as usize == i) {
                let g = (0..=32).map(|j| { let t = e.t[0]+(e.t[1]-e.t[0])*j as f64/32.; distance(f.surface.point(u.pcurve.at(t,e,&f.surface,&b.vertices)),e.point(t,&b.vertices)) }).fold(0.,f64::max);
                let k = match u.pcurve { Pcurve::Line {..} => "line",Pcurve::Inverse {..} => "inverse",Pcurve::Curve(_) => "curve" };
                eprintln!("   on {} by {k}: {g:e}",f.surface.kind());
            } }
        }
    }
    let given = Given {name:found.name,entity:r,brep:b,context:found.context,faces:faces.iter().map(|f| f.entity).collect(),edges,vertices};
    lap("measured");
    // an edge standing off its faces (or its vertices off it) farther than a hundred times the
    // file's uncertainty and a thousandth of the solid's size is no edge of theirs read right — a
    // loop's foot clamped at a closed sheet's seam it crosses, a curve the file gives that is not
    // its face's: refused, not measured into a tolerance
    let far = (100.*uncertainty).max(1e-3*size);
    if let Some((i,e)) = given.brep.edges.iter().enumerate().find(|(_,e)| e.tol > far) {
        return Err(Refusal::new(Why::Unsound,Some(given.edges[i]).filter(|&x| x != 0).or(Some(r)),
            format!("an edge stands {:.3e} mm off its faces or its vertices, past the {far:.3e} mm the reader accepts",e.tol)))
    }
    if let Err(e) = given.brep.check(within) {
        let why = match e.kind { crate::Kind::Open => Why::Open,crate::Kind::NotManifold => Why::NotManifold,_ => Why::Unsound };
        let entity = witness(&e.msg,&given).or(Some(r));
        return Err(Refusal::new(why,entity,e.msg))
    }
    lap("checked");
    Ok(given)
}
