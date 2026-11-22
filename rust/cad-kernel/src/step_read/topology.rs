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
mod closed;
use super::context::Found;
use super::geometry::{list,logical,Flat,Geometry,Sheet};
use super::{Given,Refusal,Why};
use crate::geom::{Curve,Frame,Surface,Uv,V};
use crate::nurbs::{BSpline,Net};
use crate::step_file::{File,Value};
use crate::topo::{pole_at,poleless as topo_poleless,reverse_loop,unwrap,Brep,Coedge,Edge,EdgeCurve,Face,Pcurve,Vertex};
use crate::util::space::{add,cross,distance,dot,norm,scale,sub};
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
struct Walked { entity: u32,surface: u32,reversed: bool,name: String,bounds: Vec<Bound>,points: Vec<u32> }

/// An edge as the file writes it: its entity, its first and last vertex, its geometry's entity
/// and whether it runs along its curve.
struct Written { entity: u32,ends: [usize;2],geometry: u32,same_sense: bool }

/// An edge read: its entity, the kernel's, whether it runs against the file's (its vertices taken the other
/// way round), the kernel's parameter per the file's along its curve, and its curves in faces'
/// parameters as the file writes them, by the surface's entity.
struct Read { entity: u32,edge: Edge,flip: bool,rate: f64,pcurves: Vec<(u32,Flat)> }

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
        let mut points = Vec::new();
        for bound in list(v.get(1),r,"a face's bounds")? {
            let (b,_,bv) = self.g.of(r,bound.reference(),&["FACE_BOUND","FACE_OUTER_BOUND"],"a face's bound")?;
            if bv.len() != 3 { return Err(Refusal::malformed(b,"a bound of three parameters expected")) }
            let orientation = logical(bv.get(2),b,"a bound's orientation")?;
            let Some(lp) = bv[1].reference() else { return Err(Refusal::malformed(b,"a bound's loop: a reference expected")) };
            match self.g.file.get(lp).simple() {
                // a loop of edges each from one vertex to itself along a line of no length (a
                // torus's two seams written as points, brepkit's): the point it is, a lone vertex,
                // its edges the seams the reader lays (`Walked::points`)
                Some(("EDGE_LOOP",lv)) if self.points_only(lv.get(1)).is_some() => {
                    let (k,named) = self.points_only(lv.get(1)).unwrap_or_default();
                    let k = self.vertex(lp,Some(k))?;
                    bounds.push(Bound::Vertex(lp,k));
                    points.extend(named);
                }
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
        Ok(Walked {entity:r,surface,reversed:!same_sense != turned,name,bounds,points})
    }
    /// **A loop of points alone**: where every edge of the loop `uses` runs from one vertex to
    /// itself along a `LINE` whose vector has no length standing at that vertex
    /// (`Geometry::point_line`; each edge a point, the loop the point), that
    /// vertex and the edges, each once, in the loop's order; none otherwise.
    fn points_only(&self,uses: Option<&Value>) -> Option<(u32,Vec<u32>)> {
        let (mut at,mut named) = (None,Vec::new());
        for o in uses?.list()? {
            let Some(("ORIENTED_EDGE",ov)) = self.g.file.get(o.reference()?).simple() else { return None };
            let e = ov.get(3)?.reference()?;
            let Some(("EDGE_CURVE",ev)) = self.g.file.get(e).simple() else { return None };
            let (a,z) = (ev.get(1)?.reference()?,ev.get(2)?.reference()?);
            if a != z || at.is_some_and(|x| x != a) { return None }
            at = Some(a);
            let mut c = ev.get(3)?.reference()?;
            if let Some(("SURFACE_CURVE" | "SEAM_CURVE",cv)) = self.g.file.get(c).simple() { c = cv.get(1)?.reference()?; }
            // (a point at its vertex, to the file's uncertainty: one off it is no point of the loop's)
            let p = self.g.point_line(c).ok()??;
            let Some(("VERTEX_POINT",av)) = self.g.file.get(a).simple() else { return None };
            let q = self.g.point(a,av.get(1)?.reference()).ok()?;
            if distance(p,q) > self.g.context.uncertainty.unwrap_or(0.).max(1e-9*(1.+norm(q))) { return None }
            if !named.contains(&e) { named.push(e); }
        }
        Some((at?,named))
    }
    /// A closed shell's faces, each `turned` where the shell is.
    fn shell(&mut self,from: u32,r: Option<u32>,turned: bool,out: &mut Vec<Walked>) -> Result<(),Refusal> {
        let Some(r) = r else { return Err(Refusal::malformed(from,"a shell: a reference expected")) };
        match self.g.file.get(r).simple() {
            Some(("CLOSED_SHELL",v)) => {
                let faces = list(v.get(1),r,"a shell's faces")?;
                // (a shell of no face bounds nothing: no solid of no volume)
                if faces.is_empty() { return Err(Refusal::new(Why::Empty,Some(r),"a closed shell of no face bounds no solid")) }
                for f in faces { out.push(self.face(r,f.reference(),turned,0)?); }
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
    // (a curve without end, a parabola's, read as its arc between the edge's vertices)
    let ends = [points[w.ends[0]],points[w.ends[1]]];
    let (curve,pcurves) = match g.file.get(w.geometry).simple() {
        Some(("SURFACE_CURVE" | "SEAM_CURVE" | "INTERSECTION_CURVE",v)) => {
            let curve = g.edge_curve(w.geometry,v.get(1).and_then(Value::reference),ends)?;
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
        _ => (g.edge_curve(r,Some(w.geometry),ends)?,Vec::new()),
    };
    let (curve,pcurves) = closed::restarted(curve,pcurves,w,points,within);
    // laid along its curve: from its first vertex where it runs along, else its last
    let v = if w.same_sense { w.ends } else { [w.ends[1],w.ends[0]] };
    let t = parameters(&curve.curve,[points[v[0]],points[v[1]]],v[0] == v[1],r,within)?;
    Ok(Read {entity:r,edge:Edge {curve:EdgeCurve::Curve(curve.curve),t,v:[v[0] as u32,v[1] as u32],tol:0.},flip:!w.same_sense,rate:curve.rate,pcurves})
}

/// The file's curve of an edge in a spline face's parameters as the kernel's pcurve, read over
/// the kernel's parameter of the edge (`rate` of the file's, and moved by whole periods of a
/// closed curve to where its domain says): kept where its image lies as near the edge as the
/// surface does, within `near`, at its ends and eight points between, spaced unevenly (a curve
/// matching the edge only where a symmetry makes it — a line through a rational arc's
/// parameters, right at its knots and its spans' middles — is not kept), else none.
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
        let Some(pc) = closed::onto_net(pc,e,s,vs) else { continue };
        // as near the edge as the surface is, within `near` (a file's edge may stand its
        // tolerance off its faces, and its curve in each lies under it)
        let on = [0.,0.07,0.19,0.31,0.43,0.5,0.61,0.73,0.89,1.].iter().all(|&j| {
            let t = e.t[0]+(e.t[1]-e.t[0])*j;
            let (q,p) = (pc.at(t,e,s,vs),e.point(t,vs));
            let d = distance(s.point(q),p);
            // (the foot sought from the curve's own point: where it goes farther, none found)
            d <= near || d <= s.foot_from(p,q).map(|f| distance(s.point(f),p)).filter(|&x| x <= d).unwrap_or(0.)+near
        });
        if on { return Some(pc) }
        // else where its image is the edge's all the same, run at another parameter (a writer's
        // curve fitted to its edge's points as a set, its parameter within the writer's tolerance
        // of the edge's): each of its points as near the edge's curve as the edge's point there is
        // to the surface, within `near`, and the edge's own point there within the tolerance the
        // edge measures whatever its curve in this face — the farthest it stands from the surface
        // or its ends from its vertices — so that keeping it widens no tolerance
        let EdgeCurve::Curve(c) = &e.curve else { continue };
        let at: Vec<(V,V,f64)> = [0.,0.07,0.19,0.31,0.43,0.5,0.61,0.73,0.89,1.].iter().map(|&j| {
            let t = e.t[0]+(e.t[1]-e.t[0])*j;
            let (q,p) = (pc.at(t,e,s,vs),e.point(t,vs));
            // (the edge's point's own foot, sought from the file's curve's point and found
            // globally where that does not settle on the surface by it: a curve run at another
            // parameter seeds Newton off the foot, which may settle on another)
            (s.point(q),p,distance(s.point(s.inverse_near(p,q)),p))
        }).collect();
        let ends = (0..2).map(|k| distance(c.point(e.t[k]),vs[e.v[k] as usize].p)).fold(0.,f64::max);
        let measured = at.iter().map(|x| x.2).fold(ends,f64::max);
        let image = measured.is_finite() && at.iter().all(|&(y,p,gap)| distance(y,p) <= measured+near && distance(y,c.point(c.inverse(y))) <= gap+near);
        if image { return Some(pc) }
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
/// parameter is read a step in from it (`topo::pole_at`). A step that sweeps more than a twelfth
/// of a period round is read again in halves (a curve passing near a pole sweeps half a turn of
/// the collapsed parameter in a hair of its span), and where halving does not end it — the curve
/// crossing the pole, the parameter jumping by half a turn there — the jump goes the way that
/// keeps the face on the use's left (`pole_branch`), the face `reversed` or not. Beside the `n + 1`
/// points, every point read, by the fraction of the way along it lies at.
fn track(s: &Surface,periods: [Option<f64>;2],e: &Edge,vs: &[Vertex],travel: [f64;2],from: Uv,n: usize,within: f64,reversed: bool,
    hint: &dyn Fn(usize,Uv) -> Uv) -> (Vec<Uv>,Vec<(f64,Uv)>) {
    let mut out = vec![from];
    let mut all = vec![(0.,from)];
    let pole = pole_at(s,from,within);
    let foot = |f: f64,near: Uv| unwrap(s.inverse_near(e.point(travel[0]+(travel[1]-travel[0])*f,vs),near),near,periods);
    // (a point at a pole, or a step from one, sweeps nothing: its collapsed parameter is free)
    let sweeps = |a: Uv,b: Uv| (0..2).find(|&k| periods[k].is_some_and(|p| (b[k]-a[k]).abs() > p/12.)
        && pole_at(s,a,within) != Some(k) && pole_at(s,b,within) != Some(k));
    for j in 1..=n {
        let (f0,f1) = ((j-1) as f64/n as f64,j as f64/n as f64);
        let at = out[j-1];
        let near = hint(j,at);
        let mut uv = foot(f1,near);
        if hint(j,at) == at && sweeps(at,uv).is_some() {
            // (read again in halves from the last, each half from the last read)
            let mut stack = vec![(f1,0usize)];
            let (mut fa,mut a) = (f0,at);
            while let Some((fb,depth)) = stack.pop() {
                let b = foot(fb,a);
                match sweeps(a,b) {
                    Some(_) if depth < 52 && fb-fa > 0. => { stack.push((fb,depth+1)); stack.push((0.5*(fa+fb),depth+1)); }
                    Some(k) => {
                        // crossing the pole: the jump the way that keeps the face on the left
                        let mut b = b;
                        if let Some(p) = periods[k] {
                            let g = |x: f64| { let mut q = b; q[1-k] = x; let (_,su,sv) = s.d1(q); norm([su,sv][k]) };
                            let h = 1e-6*(1.+b[1-k].abs());
                            let inward = (g(b[1-k]+h)-g(b[1-k]-h)).signum();
                            b[k] = pole_branch(b[k],a[k],p,k,inward,reversed,false);
                        }
                        all.push((fb,b));
                        (fa,a) = (fb,b);
                    }
                    None => { all.push((fb,b)); (fa,a) = (fb,b); }
                }
            }
            uv = a;
        } else { all.push((f1,uv)); }
        if j == n { if let Some(k) = pole_at(s,uv,within) {
            let before = all[all.len().saturating_sub(2)].1;
            uv[k] = before[k];
            if let Some(x) = all.last_mut() { x.1 = uv; }
        } }
        if j == 1 { if let Some(k) = pole { out[0][k] = uv[k]; all[0].1[k] = uv[k]; } }
        out.push(uv);
    }
    (out,all)
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

/// **`pole_branch` for a use that may be a seam's**: `seam` none for a use of no seam, else the value
/// of the collapsed parameter its other use took (none where that is not walked yet). A seam's
/// second use is a whole period from its first, on the face's left: where the branch that keeps
/// the face on the left is not where the first lies — the loop having come round the pole between
/// them, along an edge at the pole of its own (a cone's apex written as a closed edge) — that
/// branch, else a whole period on (`pole_branch`'s seam).
fn seam_branch(x: f64,arrived: f64,p: f64,k: usize,inward: f64,reversed: bool,seam: Option<Option<f64>>) -> f64 {
    let plain = pole_branch(x,arrived,p,k,inward,reversed,false);
    match seam {
        None => plain,
        Some(Some(o)) if (plain-o).abs() > 1e-6*p => plain,
        Some(_) => pole_branch(x,arrived,p,k,inward,reversed,true),
    }
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
/// at a pole within and a file's curve to lie on its edge within, and the closed spline sheets
/// rolled out (`Rolled`), by their surfaces' entities.
struct Setting<'r> { b: &'r Brep,reads: &'r [Read],within: f64,near: f64,rolled: &'r BTreeMap<u32,Rolled>,other: Option<bool> }

impl Setting<'_> {
    /// The periods the face on the surface `id` is walked with: the surface's own, or a closed
    /// spline sheet's widths where it closes (`Rolled`).
    fn periods(&self,id: u32,s: &Surface) -> [Option<f64>;2] { self.rolled.get(&id).map_or(s.periods(),|r| r.widths) }
}

/// **A point's other foot on a closed sheet covering its points twice, facing the other way**: for
/// `p`, whose foot `q` is, the parameters half a width along a parameter the sheet closes in
/// (`periods`) and wherever along the other at which the sheet stands at `p` too, its normal there
/// turned against the one at `q` — a whole curve turned a whole turn about an axis it is
/// symmetric about (kentron's oblate spheroid: each point once on the profile's near half, once on
/// its far half turned half a turn on). None where the sheet has no such second sheet at `p`.
fn other_foot(s: &Surface,periods: [Option<f64>;2],q: Uv,p: V,within: f64) -> Option<Uv> {
    let normal = |uv: Uv| { let (_,su,sv) = s.d1(uv); let n = cross(su,sv); let l = norm(n); if l > 0. { Some(scale(n,1./l)) } else { None } };
    let nq = normal(q)?;
    let dom = match s { Surface::BSpline(_,n) | Surface::Offset(_,n,_) => n.domain(),_ => return None };
    for k in 0..2 {
        let Some(w) = periods[k] else { continue };
        let x = q[k]+0.5*w;
        let [lo,hi] = match periods[1-k] { Some(w2) => [q[1-k]-0.5*w2,q[1-k]+0.5*w2],None => dom[1-k] };
        let at = |y: f64| if k == 0 { [x,y] } else { [y,x] };
        let (mut best,mut y) = (f64::INFINITY,lo);
        for j in 0..=128 {
            let z = lo+(hi-lo)*j as f64/128.;
            let d = distance(s.point(at(z)),p);
            if d < best { (best,y) = (d,z); }
        }
        let r = s.inverse_near(p,at(y));
        if !(distance(s.point(r),p) <= within) || (r[k]-q[k]).abs() < 0.25*w { continue }
        if normal(r).is_some_and(|n| dot(n,nq) < -0.5) { return Some(r) }
    }
    None
}

/// **A closed spline sheet rolled out** (`rolled`): the sheet as the file gives it (`sheet`), and
/// the width of its domain along each parameter it closes on itself in; the surface the faces on
/// it are walked on is that sheet put three times end to end along each, its own in the middle.
#[derive(Clone)]
struct Rolled { sheet: Surface,widths: [Option<f64>;2] }

impl Rolled {
    /// The whole widths that carry `q` into the sheet's own domain, its rolled surface's middle copy.
    fn home(&self,q: Uv) -> Uv {
        let (Surface::BSpline(_,n) | Surface::Offset(_,n,_)) = &self.sheet else { return [0.;2] };
        let dom = n.domain();
        std::array::from_fn(|k| self.widths[k].map_or(0.,|w| -((q[k]-dom[k][0])/w).floor()*w))
    }
}

/// **The parameters along which a spline sheet closes on itself** — a surface periodic whatever its
/// form, which the kernel's net holds no period of: along `k`, its rows at the domain's two ends one
/// curve, to the rounding of its size, and its derivative across them one direction where it is
/// not nought (a pole's), to a thousandth of a radian — a writer's fitted net closes with its
/// tangents some 4e-5 apart (`solid_sweeps/helical_groove.sv`'s block) — so the sheet runs on
/// round the closing line (a crease there is no period: not rolled). Read off the surface's own points and tangents, never the file's
/// `u_closed`. The width of the domain along each, none along the others.
fn closes(s: &Surface) -> [Option<f64>;2] {
    let n = match s { Surface::BSpline(_,n) | Surface::Offset(_,n,_) => n,_ => return [None;2] };
    let dom = n.domain();
    std::array::from_fn(|k| {
        let ([lo,hi],[a,b]) = (dom[k],dom[1-k]);
        let at = |x: f64,y: f64| if k == 0 { [x,y] } else { [y,x] };
        let smooth = (0..=16).all(|j| {
            let y = a+(b-a)*j as f64/16.;
            let ((p,pu,pv),(q,qu,qv)) = (s.d1(at(lo,y)),s.d1(at(hi,y)));
            let (dp,dq) = ([pu,pv][k],[qu,qv][k]);
            let size = 1.+norm(p);
            if distance(p,q) > 1e-7*size { return false }
            let (lp,lq) = (norm(dp),norm(dq));
            // (a pole's row: its derivative along it nought, which no direction is asked of)
            lp <= 1e-9*size || lq <= 1e-9*size || norm(cross(dp,dq)) <= 1e-3*lp*lq && crate::util::space::dot(dp,dq) > 0.
        });
        smooth.then_some(hi-lo)
    })
}

/// A net's knots and poles along `k` (its rows for `u`, its columns for `v`), weights beside.
type Strip = (Vec<f64>,Vec<Vec<V>>,Option<Vec<Vec<f64>>>);

/// `n`'s knots, poles and weights laid out along `k` as rows.
fn strip(n: &Net,k: usize) -> Strip {
    if k == 0 { return (n.uknots.clone(),n.poles.clone(),n.weights.clone()) }
    let t = |r: &Vec<Vec<V>>| (0..r[0].len()).map(|j| r.iter().map(|x| x[j]).collect()).collect::<Vec<Vec<V>>>();
    let tw = |r: &Vec<Vec<f64>>| (0..r[0].len()).map(|j| r.iter().map(|x| x[j]).collect()).collect::<Vec<Vec<f64>>>();
    (n.vknots.clone(),t(&n.poles),n.weights.as_ref().map(tw))
}

/// The net `n` with its knots, poles and weights along `k` those of `x`.
fn unstrip(n: &Net,k: usize,(knots,poles,weights): Strip) -> Net {
    if k == 0 { return Net {uknots:knots,poles,weights,..n.clone()} }
    let t = (0..poles[0].len()).map(|i| poles.iter().map(|x| x[i]).collect()).collect();
    let w = weights.map(|w| (0..w[0].len()).map(|i| w.iter().map(|x| x[i]).collect()).collect());
    Net {vknots:knots,poles:t,weights:w,..n.clone()}
}

/// **A closed sheet put three times end to end along `k`** (its domain `[lo, hi]` of width `w`):
/// the copies over `[lo − w, lo]`, `[lo, hi]` and `[hi, hi + w]`, joined by knots of full
/// multiplicity, each copy's poles as written — the same points at every parameter of the middle
/// copy, exactly, and the sheet carried on round its closing line a whole width each way.
fn rolled(n: &Net,k: usize,w: f64) -> Net {
    let p = if k == 0 { n.du } else { n.dv };
    let (knots,poles,weights) = strip(n,k);
    let mut out: Vec<f64> = knots[..knots.len()-1].iter().map(|t| t-w).collect();
    for d in [0.,w] {
        out.push(knots[knots.len()-1]+d-w);
        out.extend(knots[p+1..knots.len()-1].iter().map(|t| t+d));
    }
    out.push(knots[knots.len()-1]+w);
    let three = |x: &Vec<Vec<V>>| x.iter().chain(x).chain(x).cloned().collect::<Vec<_>>();
    let threew = |x: &Vec<Vec<f64>>| x.iter().chain(x).chain(x).cloned().collect::<Vec<_>>();
    unstrip(n,k,(out,three(&poles),weights.as_ref().map(threew)))
}

/// **The window of a rolled sheet a face is read on**: `n` (rolled) over the span `c_k` along each
/// parameter `k` given one (a width, or a hair more), its original domain `dom` along the others — the closed sheet cut
/// at `c_k` and its two pieces put end to end, the same points at the same parameters (knots put
/// in, never moved), now starting where no loop of the face crosses. Held to `sheet`, the sheet as
/// the file gives it, at points either side of the cut.
fn window(n: &Net,frame: Frame,offset: Option<f64>,c: [Option<[f64;2]>;2],dom: [[f64;2];2],sheet: &Surface) -> Option<Surface> {
    let range: [[f64;2];2] = std::array::from_fn(|k| c[k].unwrap_or(dom[k]));
    let net = n.segment(range[0],range[1]);
    net.check().ok()?;
    let surface = match offset { Some(d) => Surface::Offset(frame,Arc::new(net),d),None => Surface::BSpline(frame,Arc::new(net)) };
    let back = |x: f64,k: usize| { let [lo,hi] = dom[k]; lo+(x-lo).rem_euclid(hi-lo) };
    let same = [0.03,0.29,0.5,0.71,0.97].iter().all(|&f| [0.03,0.29,0.5,0.71,0.97].iter().all(|&g| {
        let q = [range[0][0]+(range[0][1]-range[0][0])*f,range[1][0]+(range[1][1]-range[1][0])*g];
        let (a,b) = (surface.point(q),sheet.point([back(q[0],0),back(q[1],1)]));
        distance(a,b) <= 1e-9*(1.+norm(a))
    }));
    same.then_some(surface)
}

/// **A loop walked in its face's parameters** (the module's doc): each use's pcurve, its ends on
/// the branch the walk reached them on.
fn walk_loop(st: &Setting,sheet: &Sheet,surface: u32,uses: &[(usize,bool)],reversed: bool,seams: &BTreeSet<usize>,face: u32,
    taken: &mut BTreeMap<usize,usize>,placed: &mut BTreeMap<usize,Uv>) -> Result<(Vec<Coedge>,Option<Wound>,Vec<Crossed>),Refusal> {
    let s = &sheet.surface;
    let (b,periods) = (st.b,st.periods(surface,s));
    let spline = matches!(s,Surface::BSpline(..));
    let plain = if matches!(s,Surface::Plane(_)) { 2 } else { 16 };
    let mut out = Vec::with_capacity(uses.len());
    let mut crossed = Vec::new();
    let mut last: Option<Uv> = None;
    let mut before: Option<Uv> = None;
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
                    // (on a spline sheet closed round a parameter, a curve a whole span outside its
                    // domain — a writer's of a periodic sheet — moved by the span into it)
                    let pc = match s {
                        Surface::BSpline(_,n) => {
                            let (a,dom) = (pc.at(travel[0],e,s,&b.vertices),n.domain());
                            let d: Uv = std::array::from_fn(|k| {
                                let [lo,hi] = dom[k];
                                let w = hi-lo;
                                if a[k] < lo-0.5*w { ((lo-a[k])/w).round()*w } else if a[k] > hi+0.5*w { -((a[k]-hi)/w).round()*w } else { 0. }
                            });
                            let mut d = d;
                            // (onto the side of the seam its other use is not on)
                            if d != [0.;2] { if let Some(&o) = placed.get(&ei) {
                                for k in 0..2 {
                                    let [lo,hi] = dom[k];
                                    let x = a[k]+d[k];
                                    if d[k] != 0. && (x-o[k]).abs() <= 1e-6*(hi-lo) { d[k] += if x-lo < hi-x { hi-lo } else { lo-hi }; }
                                }
                            } }
                            if d == [0.;2] { pc } else { moved(&pc,d) }
                        }
                        _ => pc,
                    };
                    let a = pc.at(travel[0],e,s,&b.vertices);
                    // (a sphere's `v` is held to its half turn, but its points are the formula's of
                    // every whole turn of it on, where a writer's curve may lie: moved back there)
                    let wraps: [Option<f64>;2] = match s { Surface::Sphere(..) => [periods[0],Some(TAU)],_ => periods };
                    let Some(l) = last else {
                        let mut d = [0.,wraps[1].filter(|_| periods[1].is_none()).map_or(0.,|p| -(a[1]/p).round()*p)];
                        if let Some(r) = st.rolled.get(&surface) { d = r.home(a); }
                        return if d == [0.;2] { (pc,a,k) } else { (moved(&pc,d),[a[0]+d[0],a[1]+d[1]],k) }
                    };
                    let mut d: Uv = std::array::from_fn(|k| wraps[k].map_or(0.,|p| ((l[k]-a[k])/p).round()*p));
                    // (leaving a pole, where the loop arrived along any value of its collapsed
                    // parameter: on the branch that keeps the face on the left, a seam's a whole
                    // period on from where it arrived, `pole_branch`)
                    if let Some(k) = pole_at(s,l,st.within).filter(|&k| st.rolled.contains_key(&surface) && periods[k].is_some()) {
                        let inner = pc.at(travel[0]+0.1*(travel[1]-travel[0]),e,s,&b.vertices);
                        let x = seam_branch(a[k],l[k],periods[k].unwrap_or(1.),k,(inner[1-k]-a[1-k]).signum(),reversed,
                            seams.contains(&ei).then(|| placed.get(&ei).map(|o| o[k])));
                        d[k] = x-a[k];
                    }
                    if d == [0.;2] { (pc,a,k) } else { (moved(&pc,d),[a[0]+d[0],a[1]+d[1]],k) }
                }).collect();
            if let Some(l) = last { found.sort_by(|x,y| distance2(x.1,l).total_cmp(&distance2(y.1,l))); }
            found.into_iter().next().map(|(pc,a,k)| {
                // a seam's first use on the side of it the face lies left of
                let (pc,a) = if seams.contains(&ei) && !placed.contains_key(&ei) && last.is_none() {
                    let mid = 0.5*(e.t[0]+e.t[1]);
                    let dd = pc.derivative(mid,e,s,&b.vertices);
                    let dd = if rev { [-dd[0],-dd[1]] } else { dd };
                    let shift = seam_side(s,pc.at(mid,e,s,&b.vertices),dd,reversed);
                    if shift == [0.;2] { (pc,a) } else { (moved(&pc,shift),[a[0]+shift[0],a[1]+shift[1]]) }
                } else { (pc,a) };
                taken.insert(ei,k);
                placed.entry(ei).or_insert(pc.at(0.5*(e.t[0]+e.t[1]),e,s,&b.vertices));
                (pc,a)
            })
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
                    let step = e.point(travel[0]+(travel[1]-travel[0])/16.,&b.vertices);
                    let q = near.unwrap_or_else(|| s.inverse(step));
                    // (the face read on a sheet covering its points twice set out on the sheet's
                    // other half, `other_foot`, from the step in, which stands at no pole: which
                    // of the two it is, the solid says, `twice`)
                    let q = match (st.other == Some(true)).then(|| other_foot(s,periods,q,step,st.within.max(1e-9*(1.+norm(step))))).flatten() {
                        Some(r) => r,
                        None => q,
                    };
                    let q = unwrap(s.inverse_near(begin,q),q,periods);
                    // (a loop on a closed sheet rolled out set out from its middle copy, so the
                    // walk, which may reach a width round, stays on it)
                    let d = st.rolled.get(&surface).map_or([0.;2],|r| r.home(q));
                    [q[0]+d[0],q[1]+d[1]]
                });
                // (a step of the walk turning less than a twelfth of a period: a thread's helix
                // winding sixteen turns round its cylinder, walked in sixteen steps, read each a
                // whole turn on as no turn at all — its length over the surface's speed along the
                // period at its middle, the turn it makes at most)
                let steps = {
                    // (its length by its speed at 64 points, which no turn of it aliases as chords do)
                    let length = match &e.curve {
                        EdgeCurve::Curve(c) => (0..64).map(|j| norm(c.d1(e.t[0]+(e.t[1]-e.t[0])*(j as f64+0.5)/64.).1)).sum::<f64>()*(e.t[1]-e.t[0]).abs()/64.,
                        EdgeCurve::Degenerate => 0.,
                    };
                    let (_,su,sv) = s.d1(s.inverse(e.point(0.5*(e.t[0]+e.t[1]),&b.vertices)));
                    let turn = (0..2).filter_map(|k| periods[k].map(|p| length/(norm([su,sv][k])*p))).filter(|x| x.is_finite()).fold(0.,f64::max);
                    (plain.max((48.*turn).ceil().min(4096.) as usize)+1)/2*2
                };
                // (on a sheet covering its points twice, `twice`'s readings: each use's first step
                // sought from where the loop stands on its half of the sheet — the first use's from
                // the step in's foot on the half asked for, a use leaving a pole from the side of it
                // the loop arrived on — not from the pole, where both halves meet)
                let lead: Option<Uv> = st.other.and_then(|o| match last {
                    Some(l) => { let k = pole_at(s,l,st.within)?; let p = before?; let mut h = l; h[1-k] = p[1-k]; Some(h) }
                    None => {
                        let step = e.point(travel[0]+(travel[1]-travel[0])/16.,&b.vertices);
                        let q = s.inverse(step);
                        let q = if o { other_foot(s,periods,q,step,st.within.max(1e-9*(1.+norm(step))))? } else { q };
                        // (on the rolled sheet's middle copy, as `from` is, by the same widths)
                        let d = st.rolled.get(&surface).map_or([0.;2],|r| r.home(q));
                        let q = [q[0]+d[0],q[1]+d[1]];
                        Some(std::array::from_fn(|k| periods[k].map_or(q[k],|p| q[k]+((from[k]-q[k])/p).round()*p)))
                    }
                });
                let (mut walked,mut read) = track(s,periods,e,&b.vertices,travel,from,steps,st.within,reversed,&|j,at| if j == 1 { lead.unwrap_or(at) } else { at });
                before = walked.len().checked_sub(2).map(|k| walked[k]);
                // a seam of a sheet closed without a period: walked again, every point of it on the
                // domain's edge sought on the one side — the side its other use is not on, or for
                // the first, the side the face lies left of (`seam_side`)
                if let (Surface::BSpline(_,n),true) = (s,seams.contains(&ei)) {
                    let (dom,m) = (n.domain(),walked[steps/2]);
                    let on = |q: Uv,k: usize| { let [lo,hi] = dom[k]; (q[k]-lo).min(hi-q[k]) <= 1e-6*(hi-lo) };
                    // (an edge of the domain the sheet closes across: its points there one with
                    // those on the opposite edge)
                    let closed = |k: usize| {
                        let [lo,hi] = dom[k];
                        let (mut x,mut y) = (m,m);
                        x[k] = lo; y[k] = hi;
                        distance(s.point(x),s.point(y)) <= 1e-7*(1.+norm(s.point(x)))
                    };
                    if let Some(k) = (0..2).find(|&k| on(m,k) && closed(k)) {
                        let [lo,hi] = dom[k];
                        let here = if m[k]-lo < hi-m[k] { lo } else { hi };
                        let side = match placed.get(&ei) {
                            Some(o) if on(*o,k) => if o[k]-lo < hi-o[k] { hi } else { lo },
                            _ => {
                                let (p,q) = (walked[steps/2-1],walked[steps/2+1]);
                                let mut x = m;
                                x[k] = here;
                                here+seam_side(s,x,[q[0]-p[0],q[1]-p[1]],reversed)[k]
                            }
                        };
                        let first = walked.clone();
                        let snap = |mut q: Uv| { if on(q,k) { q[k] = side; } q };
                        (walked,read) = track(s,periods,e,&b.vertices,travel,snap(first[0]),steps,st.within,reversed,&|j,_| snap(first[j]));
                        // (its ends too, where a pole's collapse left them on the other edge)
                        walked[0] = snap(walked[0]);
                        walked[steps] = snap(walked[steps]);
                    }
                }
                let [mut a,mut m,mut z] = [walked[0],walked[steps/2],walked[steps]];
                let before = placed.insert(ei,m);
                // leaving a pole after another use: on the branch that keeps the face on the left
                if let (Some(l),Some(k)) = (last,pole_at(s,from,st.within)) {
                    if let Some(p) = periods[k] {
                        let inward = (m[1-k]-a[1-k]).signum();
                        let x = seam_branch(a[k],l[k],p,k,inward,reversed,seams.contains(&ei).then_some(before).map(|o| o.map(|o| o[k])));
                        let d = x-a[k];
                        for q in [&mut a,&mut m,&mut z] { q[k] += d; }
                    }
                }
                // (the edge's inverse is read on the branch nearest the line between its ends,
                // `Pcurve::Inverse`: where its walk strays half a period from that line — a curve
                // winding unevenly round a pole — the inverse changes branch mid-edge, which `sound`
                // refuses as a leap of a period, `step_sound::a_use_leaping_a_period_is_refused`)
                let off = [a[0]-walked[0][0],a[1]-walked[0][1]];
                // a curve crossing its face's pole between its ends (a meridian over a ball's
                // top): its collapsed parameter jumps there, which no one inverse curve reads — the
                // edge cut at the pole (`Cut`), each piece its own inverse to the pole on its own
                // branch, and
                // the pole's degenerate edge between them laid by `close_poles`; the jump the way
                // that keeps the face on the left (`pole_branch`), unless that leaves the loop a
                // whole turn from where it began (a seam to the pole lying along the curve itself,
                // the face a sliver there): then the way that closes it
                let crossings: Vec<_> = (1..read.len().saturating_sub(1)).filter_map(|i| {
                    let (f,q) = read[i];
                    let k = pole_at(s,q,st.within)?;
                    let p = periods[k]?;
                    let (before,after) = (read[..i].iter().rev().find(|x| pole_at(s,x.1,st.within).is_none())?,
                        read[i+1..].iter().find(|x| pole_at(s,x.1,st.within).is_none())?);
                    (f > 0. && f < 1. && (after.1[k]-before.1[k]).abs() > p/12.).then_some((f,q,k,p,before.1,after.1))
                }).collect();
                let crossing = crossings.first().copied();
                if let Some((f,q,k,p,before,after)) = crossing {
                    let EdgeCurve::Curve(curve) = &e.curve else { unreachable!("a walked edge has a curve") };
                    let pole = s.point(q);
                    let t = { let t = curve.inverse(pole); if t > e.t[0] && t < e.t[1] { t } else { travel[0]+(travel[1]-travel[0])*f } };
                    if !(t > e.t[0] && t < e.t[1]) || distance(curve.point(t),pole) > st.within {
                        return Err(Refusal::unsupported(st.reads[ei].entity,"an edge crossing its face's pole at none of its points: not read"))
                    }
                    let inward = (after[1-k]-q[1-k]).signum();
                    let x = pole_branch(after[k]+off[k],before[k]+off[k],p,k,inward,reversed,false);
                    let (mut m1,mut m2) = (q,q);
                    m1[k] = before[k]+off[k];
                    m2[k] = x;
                    m1[1-k] += off[1-k];
                    m2[1-k] += off[1-k];
                    let mut z2 = z;
                    z2[k] += x-(after[k]+off[k]);
                    // the pieces in the loop's order: to the pole, and from it; the edge keeps its
                    // first piece along its curve, the far one is laid
                    let (to,from_pole) = if rev {
                        (Coedge {edge:CROSSED,reversed:true,pcurve:Pcurve::Inverse {a:m1,b:a}},Coedge {edge:ei as u32,reversed:true,pcurve:Pcurve::Inverse {a:z2,b:m2}})
                    } else {
                        (Coedge {edge:ei as u32,reversed:false,pcurve:Pcurve::Inverse {a,b:m1}},Coedge {edge:CROSSED,reversed:false,pcurve:Pcurve::Inverse {a:m2,b:z2}})
                    };
                    crossed.push(Crossed {at:out.len()+if rev { 0 } else { 1 },past:out.len()+1,k,p,edge:ei,t,point:curve.point(t)});
                    first.get_or_insert((a,begin));
                    out.push(to);
                    out.push(from_pole);
                    last = Some(z2);
                    continue
                }
                let (ea,eb) = if rev { (z,a) } else { (a,z) };
                (Pcurve::Inverse {a:ea,b:eb},a,z)
            }
        };
        first.get_or_insert((start,begin));
        out.push(Coedge {edge:ei as u32,reversed:rev,pcurve});
        last = Some(end);
    }
    // (a loop crossing a pole once: the half turn its collapsed parameter jumps there either way,
    // the one that brings the loop back round to where it began — a whole turn off is the jump
    // the other way, every use past the pole moved by it)
    if let ([c],Some(l),Some((a,_))) = (&crossed[..],last,first) {
        let turn = l[c.k]-a[c.k];
        if (turn.abs()-c.p).abs() <= 1e-3*c.p {
            let mut d = [0.;2];
            d[c.k] = -turn;
            for co in &mut out[c.past..] { co.pcurve = moved(&co.pcurve,d); }
            last = Some([l[0]+d[0],l[1]+d[1]]);
        }
    }
    // round to where it began, on the same branch, or round a whole period of `u`
    let mut wound = None;
    if let (Some(l),Some((a,p))) = (last,first) {
        let pole = pole_at(s,a,st.within);
        for k in 0..2 {
            let Some(period) = periods[k] else { continue };
            if pole == Some(k) || pole_at(s,l,st.within) == Some(k) { continue }
            if (l[k]-a[k]).abs() > 0.5*period {
                // round once along `k` and back to where it began along the other: wound, for
                // `face` to join to a pole or to another loop by a seam it lays
                let closes = periods[1-k].map_or(true,|q| (l[1-k]-a[1-k]).abs() <= 0.5*q) && (l[1-k]-a[1-k]).abs() <= 1e-3
                    && ((l[k]-a[k]).abs()-period).abs() <= 1e-3*period;
                if wound.is_none() && closes {
                    wound = Some(Wound {at:a,k,turn:l[k]-a[k]});
                    continue
                }
                return Err(Refusal::unsupported(face,format!("a face whose bound winds round its surface's periods otherwise than \
                    once round one of them, at {p:?}: not read")))
            }
        }
    }
    Ok((out,wound,crossed))
}

/// An edge a loop's walk crossed its face's pole on: the index in the loop of the use of the far
/// piece laid (named `CROSSED` until `face` numbers it) and of the first use past the pole, the
/// collapsed parameter and its period, the edge, its parameter at the pole and the point there.
struct Crossed { at: usize,past: usize,k: usize,p: f64,edge: usize,t: f64,point: V }

/// The name a walked loop gives the far piece of an edge cut at its face's pole.
const CROSSED: u32 = u32::MAX-15;

/// A loop winding once round its surface's period along parameter `k` (a sphere's or a cone's axis
/// for `u`): where it begins in the face's parameters, and how far round it comes (a whole period,
/// either way).
struct Wound { at: Uv,k: usize,turn: f64 }

/// **The seam a loop winding round a pole is joined to it along** (a face holding a sphere's pole
/// or a cone's apex inside it, which a file writes with no seam): the meridian (a sphere's) or
/// generator (a cone's) from the loop's first vertex `v` to the pole the face holds — north of
/// the loop where it winds `+u` with the face's material normal its surface's, south where either
/// is turned — and the pole, laid; the loop walked on up the seam from where it came round to,
/// and back down it to where it began (`Brep::close_poles` lays the pole's edge between). The
/// seam is named `seam`, the pole `laid`. On a closed spline sheet rolled out (`Rolled`) the
/// same, the seam its iso-curve (`iso`) from the vertex to the row the sheet collapses to at the
/// end of its other parameter the face holds — left of the loop's way round: toward `+v` of a loop
/// winding `+u`, toward `−u` of one winding `+v` — none where that row is no point.
fn pole_seam(s: &Surface,w: &Wound,v: u32,reversed: bool,seam: u32,laid: u32) -> Option<(V,Edge,[Coedge;2])> {
    let north = (w.turn > 0.) != reversed;
    if let Surface::BSpline(_,n) = s {
        let (k,o) = (w.k,1-w.k);
        let at = |x: f64,y: f64| { let mut q = [0.;2]; q[k] = x; q[o] = y; q };
        let [lo,hi] = n.domain()[o];
        let end = if (k == 0) == north { hi } else { lo };
        let x = w.at[k];
        let point = s.point(at(x,end));
        let width = w.turn.abs();
        if (0..8).any(|j| distance(s.point(at(x+width*j as f64/8.,end)),point) > 1e-9*(1.+norm(point))) { return None }
        let curve = iso(s,k,x)?;
        let up = w.at[o] < end;
        let (t,ends) = if up { ([w.at[o],end],[v,laid]) } else { ([end,w.at[o]],[laid,v]) };
        let edge = Edge {curve:EdgeCurve::Curve(curve),t,v:ends,tol:0.};
        let along = |x: f64| { let (a,b) = (at(x,w.at[o]),at(x,end)); if up { Pcurve::Inverse {a,b} } else { Pcurve::Inverse {a:b,b:a} } };
        return Some((point,edge,[Coedge {edge:seam,reversed:!up,pcurve:along(x+w.turn)},Coedge {edge:seam,reversed:up,pcurve:along(x)}]))
    }
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
    Some((point,edge,[Coedge {edge:seam,reversed:!up,pcurve:along(u+w.turn)},Coedge {edge:seam,reversed:up,pcurve:along(u)}]))
}

/// The curve of `s` along which its parameter `k` is `x` — a cylinder's or a cone's generator, a
/// sphere's meridian, a torus's tube circle or its circle round the axis — whose own parameter
/// is the other of the surface's at each point; on a spline sheet its iso-curve, exactly (the net
/// cut at `x` by knot insertion, the row there); none on any other surface. Held to the surface
/// at three points.
fn iso(s: &Surface,k: usize,x: f64) -> Option<Curve> {
    if let Surface::BSpline(_,n) = s {
        let dom = n.domain();
        let ([lo,hi],[a,b]) = (dom[k],dom[1-k]);
        if !(x >= lo && x <= hi) { return None }
        let first = x < hi;
        let cut = if first { [x,hi] } else { [lo,x] };
        let piece = if k == 0 { n.segment(cut,[a,b]) } else { n.segment([a,b],cut) };
        let c = match (k,first) {
            (0,true) => piece.row(0),(0,false) => piece.row(piece.poles.len()-1),
            (_,true) => piece.column(0),(_,false) => piece.column(piece.poles[0].len()-1),
        };
        let at = |y: f64| if k == 0 { [x,y] } else { [y,x] };
        let scale_of = 1.+norm(s.point(at(a)));
        return [0.,0.37,1.].iter().all(|&f| { let y = a+(b-a)*f; distance(c.point(y),s.point(at(y))) <= 1e-9*scale_of })
            .then_some(Curve::BSpline(Arc::new(c)))
    }
    let out = |f: &Frame,u: f64| add(scale(f.x,u.dcos()),scale(f.y,u.dsin()));
    let c = match (s,k) {
        (&Surface::Cylinder(f,_),0) => Curve::Line {p:s.point([x,0.]),d:f.z},
        (Surface::Cone(..),0) => Curve::Line {p:s.point([x,0.]),d:crate::util::space::normalised(sub(s.point([x,1.]),s.point([x,0.])))?},
        (&Surface::Sphere(f,r),0) => { let o = out(&f,x); Curve::Circle(Frame {o:f.o,x:o,y:f.z,z:cross(o,f.z)},r) }
        (&Surface::Torus(f,big,r),0) => { let o = out(&f,x); Curve::Circle(Frame {o:add(f.o,scale(o,big)),x:o,y:f.z,z:cross(o,f.z)},r) }
        (&Surface::Torus(f,big,r),1) => Curve::Circle(Frame {o:add(f.o,scale(f.z,r*x.dsin())),..f},big+r*x.dcos()),
        _ => return None,
    };
    let at = |y: f64| if k == 0 { [x,y] } else { [y,x] };
    let scale_of = 1.+norm(s.point(at(0.)));
    [-0.7,0.,1.3].iter().all(|&y| distance(c.point(y),s.point(at(y))) <= 1e-12*scale_of).then_some(c)
}

/// A use's pcurve cut at its edge's parameter `t`, where it reads `m`: the pieces over `[t₀, t]`
/// and `[t, t₁]` — a line's and an inverse curve's between their ends and `m`, a curve of its own
/// read at the edge's parameter as it is, both.
fn cut_pcurve(pc: &Pcurve,m: Uv) -> (Pcurve,Pcurve) {
    match *pc {
        Pcurve::Line {a,b} => (Pcurve::Line {a,b:m},Pcurve::Line {a:m,b}),
        Pcurve::Inverse {a,b} => (Pcurve::Inverse {a,b:m},Pcurve::Inverse {a:m,b}),
        Pcurve::Curve(_) => (pc.clone(),pc.clone()),
    }
}

/// An edge of the file cut where a seam the reader lays meets it: the edge, the parameter, and
/// the vertex and the edge laid there (by the placeholders `face` names them with) — the edge
/// keeping `[t₀, t]`, the laid one `[t, t₁]` on the same curve, both the file's one entity.
#[derive(Clone,Copy)]
struct Cut { edge: usize,t: f64,vertex: u32,laid: u32 }

/// A band joined: its loop, the edges laid (the seam, and the far piece of a cut edge), the vertex
/// laid (where the seam meets a cut edge) and the cut.
struct Joined { lp: Vec<Coedge>,laid: Vec<Edge>,vertex: Option<V>,cut: Option<Cut> }

/// **The seam a band is joined along** (a face bounded by two loops each winding once round its
/// surface's period, opposite ways, which a file writes with no seam between them): the curve of
/// the surface at one value of the period (`iso`) from a vertex of the one loop to the other — to
/// a vertex of it standing at that value where one does (the first such pair, each loop taken from
/// its own first use on), else to the point of an edge of it at that value, the edge cut there
/// (`Cut`) and a vertex laid — the seam crossing none of the face's `others` (its holes: their span
/// in the period holds not the seam's value between the loops). The two loops are joined into one
/// along it: the first from its vertex round, up the seam on the far side of its turn, the second
/// round from the seam's other end, and back down the seam. None where no such seam is. The edges
/// laid are named `laid` down, the vertex `pole`.
fn band(st: &Setting,s: &Surface,periods: [Option<f64>;2],reversed: bool,(la,wa,ua): (&[Coedge],Option<&Wound>,&[(usize,bool)]),
    (lb,wb,ub): (&[Coedge],Option<&Wound>,&[(usize,bool)]),others: &[&[Coedge]],laid: u32,pole: u32) -> Option<Joined> {
    let (wa,wb) = (wa?,wb?);
    let k = wa.k;
    let p = periods[k]?;
    if wb.k != k || (wa.turn+wb.turn).abs() > 1e-6*p { return None }
    let b = st.b;
    let f = Face {surface:s.clone(),reversed,loops:vec![],name:String::new()};
    // each loop's vertices, where its uses start, at their parameters on its walk
    let starts = |l: &[Coedge],u: &[(usize,bool)]| -> Vec<(usize,u32,Uv)> {
        l.iter().zip(u).enumerate().map(|(i,(c,&(e,rev)))| (i,b.edges[e].v[rev as usize],b.uv_ends(&f,c)[0])).collect()
    };
    let (sa,sb) = (starts(la,ua),starts(lb,ub));
    let spans: Vec<[[f64;2];2]> = others.iter().map(|l| span(b,&f,l)).collect();
    let clear = |x: f64,ya: f64,yb: f64| {
        let (lo,hi) = (ya.min(yb),ya.max(yb));
        (ya-yb).abs() > 1e-12 && !spans.iter().any(|h| {
            let x = x+((0.5*(h[k][0]+h[k][1])-x)/p).round()*p;
            h[1-k][1] > lo && h[1-k][0] < hi && h[k][0] <= x && x <= h[k][1]
        })
    };
    // the loops taken from a use on, each use after on the branch its walk left it moved by `d`,
    // those before a turn on
    let from = |l: &[Coedge],i: usize,turn: f64,d: f64| -> Vec<Coedge> {
        let (mut dd,mut late) = ([0.;2],[0.;2]);
        dd[k] = d;
        late[k] = d+turn;
        l[i..].iter().map(|c| Coedge {pcurve:moved(&c.pcurve,dd),..c.clone()})
            .chain(l[..i].iter().map(|c| Coedge {pcurve:moved(&c.pcurve,late),..c.clone()})).collect()
    };
    // on a surface closed round the other parameter too (a torus's band round its axis), how far
    // the second loop is moved round it, by whole periods, from `y` where it meets the seam at
    // `ya` on the first: onto the side of the first the face lies, its left as it runs (its right
    // where the face's sense turns the surface's normal) — the seam then across the face and not
    // round the rest of the tube, which the walks' branches, each its own, settle nothing of
    let lift = |ya: f64,y: f64| -> f64 {
        let Some(q) = periods[1-k] else { return 0. };
        let left = wa.turn.signum()*if k == 0 { 1. } else { -1. }*if reversed { -1. } else { 1. };
        let n = if left > 0. { ((ya-y)/q).floor()+1. } else { ((ya-y)/q).ceil()-1. };
        if ((y-ya)/q-((y-ya)/q).round()).abs() <= 1e-12 { 0. } else { n*q }
    };
    // the seam from `qa` (the first loop's vertex `va`) to `y` along the other parameter, at
    // vertex `vb`, and the loop joined round the second loop's uses `second`
    let join = |qa: Uv,va: u32,y: f64,vb: u32,first: Vec<Coedge>,second: Vec<Coedge>| -> Option<(Vec<Coedge>,Edge)> {
        let curve = iso(s,k,qa[k])?;
        let ya = qa[1-k];
        let (up,t) = if ya < y { (true,[ya,y]) } else { (false,[y,ya]) };
        let edge = Edge {curve:EdgeCurve::Curve(curve),t,v:if up { [va,vb] } else { [vb,va] },tol:0.};
        let along = |x: f64| { let (mut a,mut z) = (qa,qa); a[k] = x; z[k] = x; a[1-k] = t[0]; z[1-k] = t[1]; Pcurve::Inverse {a,b:z} };
        let mut out = first;
        out.push(Coedge {edge:laid,reversed:!up,pcurve:along(qa[k]+wa.turn)});
        out.extend(second);
        out.push(Coedge {edge:laid,reversed:up,pcurve:along(qa[k])});
        Some((out,edge))
    };
    for &(ia,va,qa) in &sa {
        for &(ib,vb,qb) in &sb {
            // one value of the period, to the reader's pole tolerance in space
            let mut at = qa;
            at[1-k] = qb[1-k];
            if distance(s.point(at),b.vertices[vb as usize].p) > st.within.max(b.vertices[vb as usize].tol) { continue }
            let up = lift(qa[1-k],qb[1-k]);
            if !clear(qa[k],qa[1-k],qb[1-k]+up) { continue }
            let mut second = from(lb,ib,wb.turn,((qa[k]+wa.turn-qb[k])/p).round()*p);
            let mut dy = [0.;2];
            dy[1-k] = up;
            if up != 0. { for c in &mut second { c.pcurve = moved(&c.pcurve,dy); } }
            let (lp,edge) = join(qa,va,qb[1-k]+up,vb,from(la,ia,wa.turn,0.),second)?;
            return Some(Joined {lp,laid:vec![edge],vertex:None,cut:None})
        }
    }
    // no vertex of the second at the value of one of the first's: the edge of the second the
    // first's vertex's value falls on, cut there
    for &(ia,va,qa) in &sa {
        for (j,c) in lb.iter().enumerate() {
            let (ei,rev) = ub[j];
            let e = &b.edges[ei];
            let EdgeCurve::Curve(_) = &e.curve else { continue };
            let g = |t: f64| c.pcurve.at(t,e,s,&b.vertices);
            let (g0,g1) = (g(e.t[0]),g(e.t[1]));
            let x = qa[k]+((0.5*(g0[k]+g1[k])-qa[k])/p).round()*p;
            if !((x-g0[k])*(x-g1[k]) < 0.) { continue }
            let (mut lo,mut hi) = (e.t[0],e.t[1]);
            for _ in 0..80 {
                let mid = 0.5*(lo+hi);
                if (g(mid)[k] < x) == (g0[k] < x) { lo = mid } else { hi = mid }
                if hi-lo <= 1e-15*(1.+hi.abs()) { break }
            }
            let t = 0.5*(lo+hi);
            if !(t > e.t[0] && t < e.t[1]) { continue }
            let m = g(t);
            let up = lift(qa[1-k],m[1-k]);
            if !clear(qa[k],qa[1-k],m[1-k]+up) { continue }
            // the use's pieces before and after the cut along the loop; the far piece laid
            let (near,far) = cut_pcurve(&c.pcurve,m);
            let (piece0,piece1) = (Coedge {edge:ei as u32,reversed:rev,pcurve:near},Coedge {edge:laid-1,reversed:rev,pcurve:far});
            let (before,after) = if rev { (piece1,piece0) } else { (piece0,piece1) };
            let d = ((qa[k]+wa.turn-m[k])/p).round()*p;
            let (mut dd,mut late) = ([0.;2],[0.;2]);
            dd[k] = d;
            late[k] = d+wb.turn;
            let mut second = vec![Coedge {pcurve:moved(&after.pcurve,dd),..after}];
            second.extend(from(lb,j+1,0.,d).into_iter().take(lb.len()-j-1));
            second.extend(lb[..j].iter().map(|c| Coedge {pcurve:moved(&c.pcurve,late),..c.clone()}));
            second.push(Coedge {pcurve:moved(&before.pcurve,late),..before});
            let mut dy = [0.;2];
            dy[1-k] = up;
            if up != 0. { for c in &mut second { c.pcurve = moved(&c.pcurve,dy); } }
            let Some((lp,edge)) = join(qa,va,m[1-k]+up,pole,from(la,ia,wa.turn,0.),second) else { continue };
            let piece = Edge {curve:e.curve.clone(),t:[t,e.t[1]],v:[pole,e.v[1]],tol:0.};
            return Some(Joined {lp,laid:vec![edge,piece],vertex:Some(e.point(t,&b.vertices)),cut:Some(Cut {edge:ei,t,vertex:pole,laid:laid-1})})
        }
    }
    None
}

/// **The side of a closed spline sheet's seam a use runs along**: where `m`, a use's middle, lies
/// on an edge of the sheet's domain the sheet closes across (its points there one with those on
/// the opposite edge), and `d` its direction in the face's parameters runs along it, the shift
/// (by the domain's span) to the other edge where the face would not lie on the use's left there
/// — inward from the domain's edge, with the material normal up (`reversed` turning it).
fn seam_side(s: &Surface,m: Uv,d: Uv,reversed: bool) -> Uv {
    let Surface::BSpline(_,n) = s else { return [0.;2] };
    let dom = n.domain();
    let left = if reversed { [d[1],-d[0]] } else { [-d[1],d[0]] };
    let mut out = [0.;2];
    for k in 0..2 {
        let [lo,hi] = dom[k];
        let w = hi-lo;
        if !(d[k].abs() <= 1e-3*d[1-k].abs()) { continue }
        let mut other = m;
        other[k] = if (m[k]-lo).abs() <= 1e-6*w { hi } else if (hi-m[k]).abs() <= 1e-6*w { lo } else { continue };
        let scale = 1.+norm(s.point(m));
        if distance(s.point(m),s.point(other)) > 1e-7*scale { continue }
        let at_lo = other[k] == hi;
        if at_lo && left[k] < 0. { out[k] = w } else if !at_lo && left[k] > 0. { out[k] = -w }
    }
    out
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

/// A face built, the edges it lays (a whole torus's seams, a pole's seam, a band's seam and the
/// far piece of an edge it cuts), which its loops name from `LAID` down, the vertices it lays (a
/// pole, a seam's end on a cut edge), which those edges name so, and the file's edges it cuts.
#[derive(Clone)]
struct Built { face: Face,laid: Vec<Edge>,poles: Vec<V>,cuts: Vec<Cut>,names: Vec<u32> }

const LAID: u32 = u32::MAX;

/// A face read: its loops walked, the outer one first and the others moved into its span.
fn face(st: &Setting,sheet: &Sheet,w: &Walked) -> Result<Built,Refusal> {
    let b = st.b;
    let s = &sheet.surface;
    let periods = st.periods(w.surface,s);
    // (a closed sheet rolled out: the sheet the file gives, where a face is its whole)
    let given = st.rolled.get(&w.surface).map_or(s,|r| &r.sheet);
    if let Some(x) = closed::whole(st,given,w) { return x }
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
    // (the lone vertices at a sphere's poles, which bound a face holding both, or the whole)
    let lone: Vec<u32> = w.bounds.iter().filter_map(|x| match *x { Bound::Vertex(_,v) => Some(v as u32),_ => None })
        .filter(|&v| !used.contains(&v) && pole_at(s,s.inverse(b.vertices[v as usize].p),st.within).is_some()).collect();
    let whole = |s: &Surface| match s {
        Surface::Sphere(..) if !lone.is_empty() => {
            // the other pole the file's own vertex where a lone one or a loop's is there
            let p = b.vertices[lone[0] as usize].p;
            let far = s.point({ let [u,v] = s.inverse(p); [u,-v] });
            let other = lone.get(1).copied().or_else(|| used.iter().copied().find(|&x| distance(b.vertices[x as usize].p,far) <= st.within));
            whole_sphere(s,p,lone[0],other,reversed,&w.name,st.within)
        }
        Surface::BSpline(..) if !lone.is_empty() => whole_sheet(given,&lone,b,reversed,&w.name,st.within),
        _ => None,
    };
    if uses.is_empty() { if let Some(x) = whole(s) { return Ok(x) } }
    for bound in &w.bounds {
        let Bound::Vertex(lp,v) = *bound else { continue };
        let p = b.vertices[v].p;
        if used.contains(&(v as u32)) || pole_at(s,s.inverse(p),st.within).is_some() { continue }
        // (its seams named by the file's edges that are its points, where it writes them so)
        if uses.is_empty() { if let Surface::Torus(_,big,r) = *s {
            // (a spindle torus's whole chart holds its lemon inside its apple, the one turned in:
            // which of them a face bounded by a point alone is, nothing says)
            if !(big > r) { return Err(Refusal::unsupported(w.entity,"a whole spindle torus bounded by a point alone: whether its face is \
                the outer part or the inner, nothing says")) }
            return Ok(Built {names:w.points.clone(),..whole_torus(s,p,v as u32,reversed,&w.name)})
        } }
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
    // a loop winding round a pole the face holds is joined to it by a seam laid; two winding
    // opposite ways round one period bound a band, joined by a seam laid between them (`band`)
    let (mut laid,mut poles) = (Vec::new(),Vec::new());
    let mut cuts = Vec::new();
    // (an edge a walk crossed the pole on: its far piece and the pole's vertex laid, the edge cut)
    let mut walked: Vec<(Vec<Coedge>,Option<Wound>)> = walked.into_iter().map(|(mut l,wound,crossed)| {
        if !crossed.is_empty() && wound.is_some() {
            return Err(Refusal::unsupported(w.entity,"a face whose bound winds round its surface and crosses its pole: not read"))
        }
        for c in crossed {
            let (lid,vid) = (LAID-laid.len() as u32,LAID-poles.len() as u32);
            let e = &b.edges[c.edge];
            laid.push(Edge {curve:e.curve.clone(),t:[c.t,e.t[1]],v:[vid,e.v[1]],tol:0.});
            poles.push(c.point);
            cuts.push(Cut {edge:c.edge,t:c.t,vertex:vid,laid:lid});
            l[c.at].edge = lid;
        }
        Ok((l,wound))
    }).collect::<Result<_,_>>()?;
    let wound: Vec<usize> = (0..walked.len()).filter(|&i| walked[i].1.is_some()).collect();
    if wound.len() == 2 {
        let (i,j) = (wound[0],wound[1]);
        let others: Vec<&[Coedge]> = (0..walked.len()).filter(|&x| x != i && x != j).map(|x| &walked[x].0[..]).collect();
        let joined = band(st,s,periods,reversed,(&walked[i].0,walked[i].1.as_ref(),&uses[i]),(&walked[j].0,walked[j].1.as_ref(),&uses[j]),&others,
            LAID-laid.len() as u32,LAID-poles.len() as u32).ok_or_else(|| Refusal::unsupported(w.entity,"a face between two bounds \
            winding round its surface, with no seam, where no seam can be laid between them crossing no other bound: not read"))?;
        laid.extend(joined.laid);
        poles.extend(joined.vertex);
        cuts.extend(joined.cut);
        walked[i] = (joined.lp,None);
        walked.remove(j);
    } else if wound.len() > 2 {
        return Err(Refusal::unsupported(w.entity,"a face between more than two bounds winding round its surface, with no seam: not read"))
    }
    let uses: Vec<Vec<(usize,bool)>> = uses.into_iter().enumerate().filter(|&(x,_)| wound.len() != 2 || x != wound[1]).map(|(_,u)| u).collect();
    let mut loops = Vec::new();
    for ((mut l,wound),u) in walked.into_iter().zip(&uses) {
        if let Some(x) = wound {
            let v = b.edges[u[0].0].v[u[0].1 as usize];
            let Some((p,e,uses)) = pole_seam(s,&x,v,reversed,LAID-laid.len() as u32,LAID-poles.len() as u32).filter(|_| x.k == 0 || matches!(s,Surface::BSpline(..))) else {
                return Err(Refusal::unsupported(w.entity,"a face whose bound winds round its surface's period with no seam and no pole \
                    inside it: not read"))
            };
            // (the edge laid names the pole laid as `LAID`, patched where it is numbered)
            laid.push(e);
            poles.push(p);
            l.extend(uses);
        }
        loops.push(l);
    }
    // a sphere's face holding its poles (a lone vertex at one, or both), its loops holes in it:
    // the whole sphere its outer loop, laid through the file's poles
    if poles.is_empty() && laid.is_empty() && !lone.is_empty() {
        if let Some(x) = whole(s) {
            loops.extend(x.face.loops);
            laid.extend(x.laid);
            poles.extend(x.poles);
        }
    } else if poles.is_empty() && laid.is_empty() && matches!(s,Surface::Sphere(..)) {
        // a sphere's face whose every loop turns as a hole does (a ball less a cap, written with no
        // seam): the whole sphere its outer loop, both poles laid
        let g = Face {surface:s.clone(),reversed,loops:loops.clone(),name:String::new()};
        if loops.iter().all(|l| (b.loop_area(&g,l) > 0.) == reversed) {
            let p = s.point([0.,std::f64::consts::FRAC_PI_2]);
            if let Some(x) = whole_sphere(s,p,LAID-1,None,reversed,&w.name,st.within) {
                loops.extend(x.face.loops);
                laid.extend(x.laid);
                poles.extend(x.poles);
                poles.push(p);
            }
        }
    }
    let mut f = Face {surface:s.clone(),reversed,loops,name:w.name.clone()};
    // (on a closed sheet rolled out, the face read on the window of the sheet its loops lie in,
    // which has no period)
    let periods = match st.rolled.get(&w.surface) {
        Some(r) => {
            f.surface = unrolled(st.b,&mut f,r,periods,&laid,&poles,&cuts).ok_or_else(|| Refusal::unsupported(w.entity,
                "a face reaching round its closed spline sheet farther than once, or one the reader cannot read on a window of the sheet one \
                width wide without a use leaping it: not read"))?;
            [None;2]
        }
        None => periods,
    };
    // the outer loop first: the one enclosing the most of the face's parameters
    if f.loops.len() > 1 {
        // (a face laying an edge read against the boundary with what it lays, its laid edges and
        // pole numbered after the boundary's)
        let with_laid;
        let (at,mut g) = if laid.is_empty() { (b,f.clone()) } else {
            let mut x = b.clone();
            let (e0,p0) = (x.edges.len() as u32,x.vertices.len() as u32);
            x.edges.extend(laid.iter().cloned().map(|mut e| { for v in &mut e.v { if *v > LAID-16 { *v = p0+(LAID-*v); } } e }));
            x.vertices.extend(poles.iter().map(|&p| Vertex {p,tol:0.}));
            let mut g = f.clone();
            for c in g.loops.iter_mut().flatten() { if c.edge > LAID-16 { c.edge = e0+(LAID-c.edge); } }
            with_laid = x;
            (&with_laid,g)
        };
        let areas: Vec<f64> = g.loops.iter().map(|l| at.loop_area(&g,l).abs()).collect();
        let outer = (0..areas.len()).max_by(|&i,&j| areas[i].total_cmp(&areas[j])).unwrap_or(0);
        let l = f.loops.remove(outer);
        f.loops.insert(0,l);
        let l = g.loops.remove(outer);
        g.loops.insert(0,l);
        // the others moved by whole periods to lie nearest its middle
        let o = span(at,&g,&g.loops[0]);
        for li in 1..f.loops.len() {
            let h = span(at,&g,&g.loops[li]);
            let mut d = [0.;2];
            for k in 0..2 {
                let Some(p) = periods[k] else { continue };
                d[k] = ((0.5*(o[k][0]+o[k][1]-h[k][0]-h[k][1]))/p).round()*p;
            }
            if d != [0.;2] { for c in &mut f.loops[li] { c.pcurve = moved(&c.pcurve,d); } }
        }
    }
    Ok(Built {face:f,laid,poles,cuts,names:Vec::new()})
}

/// **A face on a closed spline sheet read on the window of it its loops lie in** (`window`): its
/// loops, walked on the sheet rolled out (`Rolled`) as on any periodic surface, moved by whole
/// widths to lie over the sheet's own domain where they may; the sheet as the file gives it where
/// they lie within that domain, else the sheet cut where no loop crosses — at the middle of the
/// widths the face leaves free, or, where its loops reach right round (a seam the file writes, or
/// one laid between two winding loops or to a pole), at their start. None where a loop reaches
/// round farther than once.
fn unrolled(b: &Brep,f: &mut Face,r: &Rolled,periods: [Option<f64>;2],laid: &[Edge],poles: &[V],cuts: &[Cut]) -> Option<Surface> {
    let s = f.surface.clone();
    let (frame,n,offset) = match &s { Surface::BSpline(fr,n) => (*fr,n,None),Surface::Offset(fr,n,d) => (*fr,n,Some(*d)),_ => return None };
    let dom = match &r.sheet { Surface::BSpline(_,m) | Surface::Offset(_,m,_) => m.domain(),_ => return None };
    // (the face's own and the laid edges and vertices, as `face` reads its outer loop with them, a
    // file's edge a seam laid cuts read to the cut)
    let with: Option<(Brep,Face)> = (!laid.is_empty() || !poles.is_empty()).then(|| {
        let mut x = b.clone();
        for c in cuts { x.edges[c.edge].t[1] = c.t; }
        let (e0,p0) = (x.edges.len() as u32,x.vertices.len() as u32);
        x.edges.extend(laid.iter().cloned().map(|mut e| { for v in &mut e.v { if *v > LAID-16 { *v = p0+(LAID-*v); } } e }));
        x.vertices.extend(poles.iter().map(|&p| Vertex {p,tol:0.}));
        let mut g = f.clone();
        for c in g.loops.iter_mut().flatten() { if c.edge > LAID-16 { c.edge = e0+(LAID-c.edge); } }
        (x,g)
    });
    let (at,g): (&Brep,&Face) = match &with { Some((x,g)) => (x,g),None => (b,&*f) };
    // each loop's reach along each parameter, its uses read at seventeen points (an inverse
    // curve's on the branch of the periods nearest the line between its ends, as it is read)
    let reaches: Vec<[[f64;2];2]> = g.loops.iter().map(|l| {
        let mut reach = [[f64::INFINITY,f64::NEG_INFINITY];2];
        for c in l {
            let e = &at.edges[c.edge as usize];
            for j in 0..=16 {
                let t = e.t[0]+(e.t[1]-e.t[0])*j as f64/16.;
                let q = c.pcurve.at(t,e,&g.surface,&at.vertices);
                let q = match c.pcurve { Pcurve::Inverse {a,b} => unwrap(q,[a[0]+(b[0]-a[0])*j as f64/16.,a[1]+(b[1]-a[1])*j as f64/16.],periods),_ => q };
                for k in 0..2 { reach[k][0] = reach[k][0].min(q[k]); reach[k][1] = reach[k][1].max(q[k]); }
            }
        }
        reach
    }).collect();
    // the loops moved by whole widths to lie nearest the widest's middle, as holes are in a face's
    // outer loop
    let widest = (0..reaches.len()).max_by(|&i,&j| {
        let x = |i: usize| (0..2).map(|k| reaches[i][k][1]-reaches[i][k][0]).sum::<f64>();
        x(i).total_cmp(&x(j))
    })?;
    let mut reach = [[f64::INFINITY,f64::NEG_INFINITY];2];
    for (li,h) in reaches.iter().enumerate() {
        let mut d = [0.;2];
        for k in 0..2 {
            let Some(w) = periods[k] else { continue };
            d[k] = ((0.5*(reaches[widest][k][0]+reaches[widest][k][1]-h[k][0]-h[k][1]))/w).round()*w;
            reach[k] = [reach[k][0].min(h[k][0]+d[k]),reach[k][1].max(h[k][1]+d[k])];
        }
        if d != [0.;2] { for c in &mut f.loops[li] { c.pcurve = moved(&c.pcurve,d); } }
    }
    let (mut d,mut cut,mut over) = ([0.;2],[None;2],[0.;2]);
    for k in 0..2 {
        let Some(w) = r.widths[k] else { continue };
        let [lo,hi] = dom[k];
        let [mn,mx] = reach[k];
        over[k] = (mx-mn-w).max(0.);
        if !(mx-mn <= w*(1.+1e-6)) { return None }
        // (the middle of its reach within the sheet's own domain)
        d[k] = -((0.5*(mn+mx)-lo)/w).floor()*w;
        let (mn,mx) = (mn+d[k],mx+d[k]);
        if mn >= lo-1e-9*w && mx <= hi+1e-9*w { continue }
        // (centred on the loops' reach: where that is a hair more than a width — a file's seam
        // leaning, its two uses not one width apart, CADGenBench 217's by 2.5e-7 — the hair split
        // between the window's two edges, so that a foot a hair past one is still read on it)
        cut[k] = Some(if mx-mn >= w*(1.-1e-9) && mx-mn <= w*(1.+1e-9) { [mn,mn+w] } else { [0.5*(mn+mx)-0.5*w,0.5*(mn+mx)+0.5*w] });
    }
    if d != [0.;2] { for c in f.loops.iter_mut().flatten() { c.pcurve = moved(&c.pcurve,d); } }
    // (a walk's ends within rounding of the window's edges put on them, as the sheet's own domain
    // held them where the walk was clamped into it: a corner a hair past both edges read 1e-3 mm²
    // off a whole torus's net)
    let range: [[f64;2];2] = std::array::from_fn(|k| cut[k].unwrap_or(dom[k]));
    let snap = |mut q: Uv| { for k in 0..2 { let [lo,hi] = range[k]; for e in [lo,hi] { if (q[k]-e).abs() <= 1e-9*(hi-lo) { q[k] = e; } } } q };
    for c in f.loops.iter_mut().flatten() {
        if let Pcurve::Inverse {a,b} | Pcurve::Line {a,b} = &mut c.pcurve { *a = snap(*a); *b = snap(*b); }
    }
    if cut == [None;2] { return Some(r.sheet.clone()) }
    let surface = window(n,frame,offset,cut,dom,&r.sheet)?;
    // (each use read on the window as its measures read it, at 33 points, running on without a
    // jump of half the window: a foot a hair past an edge read on the far one leaps the width and
    // its band is measured half — refused rather than measured so)
    let whole = (0..2).all(|k| {
        let Some(w) = r.widths[k] else { return true };
        f.loops.iter().flatten().all(|c| {
            let i = if c.edge > LAID-16 { b.edges.len()+(LAID-c.edge) as usize } else { c.edge as usize };
            let Some(e) = at.edges.get(i) else { return false };
            let q: Vec<Uv> = (0..=32).map(|j| c.pcurve.at(e.t[0]+(e.t[1]-e.t[0])*j as f64/32.,e,&surface,&at.vertices)).collect();
            q.windows(2).all(|x| (x[1][k]-x[0][k]).abs() < 0.5*w)
        })
    });
    whole.then_some(surface)
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
    Built {face:Face {surface:s.clone(),reversed,loops:vec![l],name:name.to_string()},laid,poles:Vec::new(),cuts:Vec::new(),names:Vec::new()}
}

/// **A whole sphere**, bounded by the vertex at one of its poles alone (Open CASCADE writes one
/// so), or at both (`other`): the other pole, where the file gives none, and the meridian through
/// `u = 0` from the one to the other laid, the
/// meridian walked up one side of the square of the face's parameters and down the other
/// (`Brep::close_poles` lays the poles' edges between). None where the vertex is no pole.
fn whole_sphere(s: &Surface,p: V,v: u32,other: Option<u32>,reversed: bool,name: &str,within: f64) -> Option<Built> {
    let Surface::Sphere(f,r) = *s else { return None };
    let [_,at] = s.inverse(p);
    pole_at(s,[0.,at],within)?;
    use std::f64::consts::FRAC_PI_2;
    let north = at > 0.;
    let far_point = s.point([0.,if north { -FRAC_PI_2 } else { FRAC_PI_2 }]);
    let meridian = Curve::Circle(Frame {o:f.o,x:f.x,y:f.z,z:cross(f.x,f.z)},r);
    // from the south pole to the north, the laid pole named `LAID` until it is numbered
    let far = other.unwrap_or(LAID);
    let ends = if north { [far,v] } else { [v,far] };
    let edge = Edge {curve:EdgeCurve::Curve(meridian),t:[-FRAC_PI_2,FRAC_PI_2],v:ends,tol:0.};
    let side = |u: f64| Pcurve::Inverse {a:[u,-FRAC_PI_2],b:[u,FRAC_PI_2]};
    let mut l = vec![Coedge {edge:LAID,reversed:false,pcurve:side(TAU)},Coedge {edge:LAID,reversed:true,pcurve:side(0.)}];
    if reversed { l = reverse_loop(l); }
    let poles = if other.is_some() { vec![] } else { vec![far_point] };
    Some(Built {face:Face {surface:s.clone(),reversed,loops:vec![l],name:name.to_string()},laid:vec![edge],poles,cuts:Vec::new(),names:Vec::new()})
}

/// **A whole closed spline sheet**, bounded by the vertices at its poles alone (a net closed round
/// one parameter and collapsed to a point at both ends of the other: an ellipsoid's, a spheroid's,
/// a lemon's, which ISO 10303-42 lets a face bound by `VERTEX_LOOP`s): the sheet's whole domain,
/// its seam — the iso-curve at the start of the parameter it closes round, the net's own end row,
/// exactly — laid from the one pole to the other and walked up one side of the domain and down
/// the other (`Brep::close_poles` lays the poles' edges between), a pole the file gives no vertex
/// at laid too. `lone`: the file's vertices at poles. None where the sheet closes round neither
/// parameter, a pole row is no point, or a vertex stands at neither pole.
fn whole_sheet(s: &Surface,lone: &[u32],b: &Brep,reversed: bool,name: &str,within: f64) -> Option<Built> {
    let Surface::BSpline(_,n) = s else { return None };
    let dom = n.domain();
    for c in 0..2 {
        let o = 1-c;
        let at = |x: f64,y: f64| { let mut q = [0.;2]; q[c] = x; q[o] = y; q };
        let ([c0,c1],[o0,o1]) = (dom[c],dom[o]);
        // closed round `c`, its rows at `o`'s two ends each one point (its poles)
        let closed = (0..=8).all(|j| { let y = o0+(o1-o0)*j as f64/8.; let p = s.point(at(c0,y)); distance(p,s.point(at(c1,y))) <= 1e-7*(1.+norm(p)) });
        let pole = |y: f64| (0..=8).all(|j| distance(s.point(at(c0+(c1-c0)*j as f64/8.,y)),s.point(at(c0,y))) <= within);
        if !(closed && pole(o0) && pole(o1)) { continue }
        let ends = [s.point(at(c0,o0)),s.point(at(c0,o1))];
        // each lone vertex at one of the two poles
        let mut given = [None;2];
        for &v in lone {
            let p = b.vertices[v as usize].p;
            let k = (0..2).find(|&k| distance(p,ends[k]) <= within.max(1e-7*(1.+norm(p))))?;
            given[k] = Some(v);
        }
        let poles: Vec<V> = (0..2).filter(|&k| given[k].is_none()).map(|k| ends[k]).collect();
        let mut laid_at = LAID;
        let v: [u32;2] = std::array::from_fn(|k| given[k].unwrap_or_else(|| { let x = laid_at; laid_at -= 1; x }));
        let seam = if c == 0 { n.row(0) } else { n.column(0) };
        let edge = Edge {curve:EdgeCurve::Curve(Curve::BSpline(Arc::new(seam))),t:[o0,o1],v,tol:0.};
        let line = |x: f64| Pcurve::Line {a:at(x,o0),b:at(x,o1)};
        // counter-clockwise round the domain: up `o` at the end of `c` and down it at the start where
        // `c` is `u`, the other way where `c` is `v`
        let mut l = if c == 0 { vec![Coedge {edge:LAID,reversed:false,pcurve:line(c1)},Coedge {edge:LAID,reversed:true,pcurve:line(c0)}] }
            else { vec![Coedge {edge:LAID,reversed:false,pcurve:line(c0)},Coedge {edge:LAID,reversed:true,pcurve:line(c1)}] };
        if reversed { l = reverse_loop(l); }
        return Some(Built {face:Face {surface:s.clone(),reversed,loops:vec![l],name:name.to_string()},laid:vec![edge],poles,cuts:Vec::new(),names:Vec::new()})
    }
    None
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

/// Whether the face `w` lies on a closed sheet covering its points twice facing opposite ways, with
/// no curve of the file's in it to say which of the two it is on: the point halfway along its first
/// edge has another foot on the sheet, its normal turned (`other_foot`).
fn doubled(st: &Setting,sheet: &Sheet,w: &Walked) -> bool {
    let s = &sheet.surface;
    let periods = st.periods(w.surface,s);
    if periods == [None;2] || !matches!(s,Surface::BSpline(..) | Surface::Offset(..)) { return false }
    let uses: Vec<usize> = w.bounds.iter().flat_map(|x| match x { Bound::Edges(u) => u.iter().map(|u| u.edge).collect(),Bound::Vertex(..) => Vec::new() }).collect();
    if uses.iter().any(|&e| st.reads[e].pcurves.iter().any(|(id,_)| *id == w.surface)) { return false }
    let Some(&e) = uses.first() else { return false };
    let e = &st.b.edges[e];
    let p = e.point(0.5*(e.t[0]+e.t[1]),&st.b.vertices);
    other_foot(s,periods,s.inverse(p),p,st.within.max(1e-9*(1.+norm(p)))).is_some()
}

/// The material normal of a face read at the middle of its use of the file's edge `e`, and the
/// direction square to the edge into the face (the face lies on its use's left): none where the
/// face does not use it once, or where the edge has no curve.
fn side(b: &Brep,f: &Face,e: usize) -> Option<(V,V)> {
    let mut uses = f.loops.iter().flatten().filter(|c| c.edge as usize == e);
    let c = uses.next()?;
    if uses.next().is_some() { return None }
    let ed = &b.edges[e];
    let EdgeCurve::Curve(curve) = &ed.curve else { return None };
    let t = 0.5*(ed.t[0]+ed.t[1]);
    let d = curve.d1(t).1;
    let d = if c.reversed { scale(d,-1.) } else { d };
    let (_,su,sv) = f.surface.d1(c.pcurve.at(t,ed,&f.surface,&b.vertices));
    let n = cross(su,sv);
    let n = scale(n,if f.reversed { -1. } else { 1. }/norm(n));
    let into = cross(n,d);
    let l = norm(into);
    (l > 0. && l.is_finite()).then(|| (n,scale(into,1./l)))
}

/// Whether two faces' readings may meet along the edge `e`: where they meet tangent there, they
/// lie on opposite sides of it (a face read on a doubled sheet's wrong half lies on the same side
/// as its neighbour, over it); none where they do not meet tangent (anything may then meet).
fn meet(b: &Brep,f: &Face,g: &Face,e: usize) -> Option<bool> {
    let ((n,d),(m,c)) = (side(b,f,e)?,side(b,g,e)?);
    (dot(n,m).abs() >= 0.9).then(|| dot(d,c) < 0.)
}

/// **A face on a closed sheet covering its points twice, facing opposite ways, is the one of its
/// two readings the solid needs** (`doubled`). On such a sheet (a whole curve turned a whole turn
/// about an axis it is symmetric about) every point is the sheet's twice, its normal turned, so a
/// face's sense says nothing of which way it faces, and its bound reads two faces on it — on one
/// half the region on its left with the normal one way, on the other the rest of the sheet with the
/// normal the other way — each a face of its own: which, the walk's first foot decided (kentron's
/// oblate spheroid with its profile started half a turn on was refused inside out; a spheroid cut by
/// two meridians read as its small wedge twice, facing both ways, volume nought). Each such face is
/// read on both; faces of it meeting tangent along an edge are read so that they lie on opposite
/// sides of it, as they must with any neighbour they meet tangent (`meet`) — which leaves each group
/// of them two readings, one the other turned — and of those the one the solid is sound with
/// (`sound`: its shells facing out, a void in) is taken. Refused naming the face where both are, or
/// where no reading of a group meets its neighbours.
fn twice(st: &Setting,other: &Setting,sheets: &BTreeMap<u32,Sheet>,faces: &[Walked],built: Vec<Built>,
    finish: &dyn Fn(Vec<Built>) -> Result<Given,Refusal>,context: &super::context::Context) -> Result<Vec<Built>,Refusal> {
    let amb: Vec<usize> = (0..faces.len()).filter(|&i| doubled(st,&sheets[&faces[i].surface],&faces[i])).collect();
    if amb.is_empty() { return Ok(built) }
    let b = st.b;
    // each read on the half of the sheet its first foot is on, and on the other
    let on: [BTreeMap<usize,Built>;2] = [st,other].map(|x| amb.iter().filter_map(|&i| face(x,&sheets[&faces[i].surface],&faces[i]).ok().map(|f| (i,f))).collect());
    let reading = |i: usize,o: u8| -> Option<&Face> { on[o as usize].get(&i).map(|x| &x.face) };
    let edges = |f: &Face| -> BTreeSet<usize> { f.loops.iter().flatten().map(|c| c.edge as usize).filter(|&e| e < b.edges.len()).collect() };
    let shared: Vec<BTreeSet<usize>> = built.iter().map(|x| edges(&x.face)).collect();
    // the groups: faces of a doubled sheet meeting tangent along an edge
    let tangent = |i: usize,j: usize,e: usize| side(b,&built[i].face,e).zip(side(b,&built[j].face,e)).is_some_and(|((n,_),(m,_))| dot(n,m).abs() >= 0.9);
    let mut group: BTreeMap<usize,usize> = BTreeMap::new();
    for (g,&i) in amb.iter().enumerate() {
        if group.contains_key(&i) { continue }
        let mut stack = vec![i];
        group.insert(i,g);
        while let Some(x) = stack.pop() {
            for &j in &amb {
                if group.contains_key(&j) { continue }
                if shared[x].intersection(&shared[j]).any(|&e| tangent(x,j,e)) { group.insert(j,g); stack.push(j); }
            }
        }
    }
    let mut chosen: BTreeMap<usize,u8> = amb.iter().map(|&i| (i,0)).collect();
    let groups: BTreeSet<usize> = group.values().copied().collect();
    for g in groups {
        let members: Vec<usize> = amb.iter().copied().filter(|i| group[i] == g).collect();
        // the group's two readings: from each reading of its first face, its others set by meeting
        let mut options: Vec<BTreeMap<usize,u8>> = Vec::new();
        'seed: for o in [0u8,1] {
            let mut pick: BTreeMap<usize,u8> = BTreeMap::new();
            if reading(members[0],o).is_none() { continue }
            pick.insert(members[0],o);
            let mut stack = vec![members[0]];
            while let Some(x) = stack.pop() {
                let fx = reading(x,pick[&x]).expect("a reading picked");
                for &j in &members {
                    let es: Vec<usize> = shared[x].intersection(&shared[j]).copied().filter(|&e| tangent(x,j,e)).collect();
                    if j == x || es.is_empty() { continue }
                    let fits = |p: u8| reading(j,p).is_some_and(|fj| es.iter().all(|&e| meet(b,fx,fj,e) != Some(false)));
                    match pick.get(&j) {
                        Some(&p) => if !fits(p) { continue 'seed },
                        None => match (fits(0),fits(1)) {
                            (true,false) => { pick.insert(j,0); stack.push(j); }
                            (false,true) => { pick.insert(j,1); stack.push(j); }
                            _ => continue 'seed,
                        },
                    }
                }
            }
            // and each meets the faces off doubled sheets it meets tangent
            for &i in &members {
                let fi = reading(i,pick[&i]).expect("a reading picked");
                for j in (0..faces.len()).filter(|j| !chosen.contains_key(j)) {
                    if shared[i].intersection(&shared[j]).any(|&e| meet(b,fi,&built[j].face,e) == Some(false)) { continue 'seed }
                }
            }
            options.push(pick);
        }
        let entity = faces[members[0]].entity;
        let with = |pick: &BTreeMap<usize,u8>| -> Vec<Built> {
            (0..faces.len()).map(|i| match pick.get(&i).or(chosen.get(&i)) {
                Some(&o) => on[o as usize].get(&i).cloned().unwrap_or_else(|| built[i].clone()),
                None => built[i].clone(),
            }).collect()
        };
        let sound: Vec<bool> = options.iter().map(|pick| finish(with(pick)).is_ok_and(|x| super::sound::sound(&x.brep,context).is_ok())).collect();
        match (options.len(),&sound[..]) {
            (0,_) => return Err(Refusal::unsupported(entity,"a face on a surface covering its points twice, read on neither of them meeting its \
                neighbours as a solid's faces do: not read")),
            (2,[true,true]) => return Err(Refusal::unsupported(entity,"a face on a surface covering its points twice facing opposite ways, the \
                solid sound with it read on either: which it is, nothing says")),
            _ => {
                let k = sound.iter().position(|&x| x).unwrap_or(0);
                for (&i,&o) in &options[k] { chosen.insert(i,o); }
            }
        }
    }
    Ok((0..faces.len()).map(|i| chosen.get(&i).and_then(|&o| on[o as usize].get(&i).cloned()).unwrap_or_else(|| built[i].clone())).collect())
}

/// **A use of an edge lying along a row of its sheet's domain from end to end, read on that row**:
/// the row `k` (the parameter held) at `at`, the other parameter's feet `y` of the edge's points
/// at its parameters `t`, each within `tau` of the row's point there and running one way. None
/// where any stands farther, or where they turn back.
fn along_row(f: &Face,e: &Edge,vs: &[Vertex],k: usize,at: f64,tau: f64) -> Option<(Vec<f64>,Vec<f64>,f64)> {
    let Surface::BSpline(_,n) = &f.surface else { return None };
    let [a,z] = n.domain()[1-k];
    let with = |y: f64| if k == 0 { [at,y] } else { [y,at] };
    // (a row collapsed to a point is a pole's, no row an edge lies along)
    if distance(f.surface.point(with(a)),f.surface.point(with(z))) <= tau && distance(f.surface.point(with(a)),f.surface.point(with(0.5*(a+z)))) <= tau { return None }
    let foot = |p: V,y0: f64| -> (f64,f64) {
        let mut y = y0;
        for _ in 0..60 {
            let (x,su,sv) = f.surface.d1(with(y));
            let d = if k == 0 { sv } else { su };
            let g = dot(sub(x,p),d)/dot(d,d);
            if !g.is_finite() { break }
            let next = (y-g).clamp(a,z);
            if (next-y).abs() <= 1e-15*(1.+(z-a).abs()) { y = next; break }
            y = next;
        }
        (y,distance(f.surface.point(with(y)),p))
    };
    const N: usize = 64;
    let ts: Vec<f64> = (0..=N).map(|j| e.t[0]+(e.t[1]-e.t[0])*j as f64/N as f64).collect();
    let p0 = e.point(ts[0],vs);
    let y0 = (0..=256).map(|j| a+(z-a)*j as f64/256.).min_by(|&x,&y| distance(f.surface.point(with(x)),p0).total_cmp(&distance(f.surface.point(with(y)),p0)))?;
    let (mut ys,mut worst) = (Vec::with_capacity(N+1),0f64);
    let mut y = y0;
    for &t in &ts {
        let (x,d) = foot(e.point(t,vs),y);
        if !(d <= tau) { return None }
        worst = worst.max(d);
        ys.push(x);
        y = x;
    }
    let up = ys[N] >= ys[0];
    if ys.windows(2).any(|w| if up { w[1] < w[0] } else { w[1] > w[0] }) || (ys[N]-ys[0]).abs() <= 1e-12*(z-a) { return None }
    Some((ts,ys,worst))
}

/// **A face whose uses lie along its sheet's domain's rows, read on them** (`solid`, where `sound`
/// refuses the face read by its edges' nearest feet): each use of a file's edge lying along a row of
/// the face's spline sheet at its domain's edge (`along_row`, within `tau`), its pcurve along that
/// row — a line where the row's feet run linear in the edge's parameter, else through the feet at
/// 65 of its parameters — the rest as read. The foot of an edge is where its face is: a sheet
/// folding back under itself near its domain's edge (benchy's #27487, its corner crumpled, its
/// normal turning over) holds points nearer the edge than the row the face ends at, and read by
/// them the edge's curve in the face's parameters leaps into the fold and back. Its tolerance is
/// then measured against the row, as any edge's is. None where no use lies along a row, or none
/// reads otherwise there.
fn rows(b: &Brep,f: &Face,tau: f64) -> Option<Face> {
    let Surface::BSpline(_,n) = &f.surface else { return None };
    let dom = n.domain();
    let mut g = f.clone();
    let mut changed = false;
    for c in g.loops.iter_mut().flatten() {
        let Some(e) = b.edges.get(c.edge as usize) else { continue };
        if !matches!(e.curve,EdgeCurve::Curve(_)) { continue }
        let best = (0..2).flat_map(|k| dom[k].map(|at| (k,at))).filter_map(|(k,at)| along_row(f,e,&b.vertices,k,at,tau).map(|x| (k,at,x)))
            .min_by(|x,y| x.2.2.total_cmp(&y.2.2));
        let Some((k,at,(ts,ys,_))) = best else { continue };
        let uv = |y: f64| if k == 0 { [at,y] } else { [y,at] };
        // (a use read on the row already stays as it is)
        if (0..=8).all(|j| { let t = e.t[0]+(e.t[1]-e.t[0])*j as f64/8.; (c.pcurve.at(t,e,&f.surface,&b.vertices)[k]-at).abs() <= 1e-9*(1.+at.abs()) }) { continue }
        let m = ys.len()-1;
        let linear = (0..=m).all(|j| (ys[j]-(ys[0]+(ys[m]-ys[0])*j as f64/m as f64)).abs() <= 1e-9*(dom[1-k][1]-dom[1-k][0]));
        c.pcurve = if linear { Pcurve::Line {a:uv(ys[0]),b:uv(ys[m])} } else {
            let mut knots = vec![ts[0]];
            knots.extend(&ts);
            knots.push(ts[m]);
            let poles = ys.iter().map(|&y| { let q = uv(y); [q[0],q[1],0.] }).collect();
            let Ok(polyline) = BSpline::new(1,knots,poles) else { continue };
            Pcurve::Curve(Arc::new(Curve::BSpline(Arc::new(polyline))))
        };
        changed = true;
    }
    changed.then_some(g)
}

/// The solid `found` names, read; refused naming the entity that could not be.
pub fn solid(file: &File,found: Found) -> Result<Given,Refusal> { read_solid(file,found,&[]) }

/// **The solid `found` names, read and sound** (`sound`), or refused. Where `sound` refuses a face
/// read by its edges' nearest feet, the solid is read again with that face's uses along its
/// sheet's domain's rows laid on them (`rows`), and taken where that is sound — again for the next
/// face it then refuses, up to eight; else the first refusal stands. A face sound as first read is
/// never read again.
pub fn sound_solid(file: &File,found: &Found) -> Result<Given,Refusal> {
    let given = solid(file,found.clone())?;
    let face = |u: &super::sound::Unsound,g: &Given| u.face.or_else(|| u.edge.and_then(|e| g.brep.faces.iter().position(|f| f.loops.iter().flatten().any(|c| c.edge as usize == e))));
    let u = match super::sound::sound(&given.brep,&given.context) { Ok(()) => return Ok(given),Err(u) => u };
    let refusal = u.refusal(&given);
    let (mut rowed,mut at) = (Vec::new(),face(&u,&given));
    while let Some(fi) = at.filter(|f| !rowed.contains(f) && rowed.len() < 8) {
        rowed.push(fi);
        let Ok(g) = read_solid(file,found.clone(),&rowed) else { break };
        match super::sound::sound(&g.brep,&g.context) { Ok(()) => return Ok(g),Err(u) => at = face(&u,&g) }
    }
    Err(refusal)
}

/// `solid`, the faces `rowed` read on their sheets' rows (`rows`).
fn read_solid(file: &File,found: Found,rowed: &[usize]) -> Result<Given,Refusal> {
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
    let reads = crate::util::par::indices(w.written.len(),|i| read_edge(&g,&w.written[i],&points,within.max(1e-7*size)))
        .into_iter().collect::<Result<Vec<_>,_>>()?;
    let b = Brep {vertices:points.iter().map(|&p| Vertex {p,tol:0.}).collect(),edges:reads.iter().map(|r| r.edge.clone()).collect(),faces:Vec::new()};
    // the surfaces, each read once
    let ids: Vec<u32> = faces.iter().map(|f| f.surface).collect::<BTreeSet<_>>().into_iter().collect();
    let sheets: BTreeMap<u32,Sheet> = ids.iter().copied().zip(crate::util::par::indices(ids.len(),|i| g.surface(ids[i],Some(ids[i])))
        .into_iter().collect::<Result<Vec<_>,_>>()?).collect();
    // a surface whose form waits on where its faces lie (a revolution held as a net, its seams put
    // where no face crosses them; a line or a parabola turned, bounded past its faces): seated on its
    // faces' edges and lone vertices, a seam's points apart (`geometry::Sheet::seated`)
    let sheets: BTreeMap<u32,Sheet> = sheets.into_iter().map(|(id,s)| {
        if s.seat.is_none() { return Ok((id,s)) }
        let (mut on,mut seams) = (Vec::new(),Vec::new());
        for f in faces.iter().filter(|f| f.surface == id) {
            let mut count: BTreeMap<usize,usize> = BTreeMap::new();
            for bound in &f.bounds {
                match bound {
                    Bound::Edges(uses) => for u in uses { *count.entry(u.edge).or_default() += 1; },
                    Bound::Vertex(_,v) => on.push(b.vertices[*v].p),
                }
            }
            for (&e,&n) in &count {
                let e = &b.edges[e];
                let at: Vec<V> = (0..=8).map(|j| e.point(e.t[0]+(e.t[1]-e.t[0])*j as f64/8.,&b.vertices)).collect();
                on.extend(&at);
                if n > 1 { seams.push(at); }
            }
        }
        Ok((id,s.seated(id,&on,&seams)?))
    }).collect::<Result<_,Refusal>>()?;
    // a spline sheet closed on itself rolled out, its faces walked on it as on any periodic
    // surface (`Rolled`)
    let mut rolls: BTreeMap<u32,Rolled> = BTreeMap::new();
    let sheets: BTreeMap<u32,Sheet> = sheets.into_iter().map(|(id,s)| {
        let widths = closes(&s.surface);
        let (frame,n,offset) = match &s.surface {
            Surface::BSpline(f,n) if s.seat.is_none() => (*f,n,None),
            Surface::Offset(f,n,d) if s.seat.is_none() => (*f,n,Some(*d)),
            _ => return (id,s),
        };
        if widths == [None;2] { return (id,s) }
        let mut net = (**n).clone();
        for k in 0..2 { if let Some(w) = widths[k] { net = rolled(&net,k,w); } }
        if net.check().is_err() { return (id,s) }
        let surface = match offset { Some(d) => Surface::Offset(frame,Arc::new(net),d),None => Surface::BSpline(frame,Arc::new(net)) };
        rolls.insert(id,Rolled {sheet:s.surface.clone(),widths});
        (id,Sheet {surface,..s})
    }).collect();
    let setting = |other: Option<bool>| Setting {b:&b,reads:&reads,within:1e-7*(1.+size),near:uncertainty.min(1e-6*size).max(1e-12*size),rolled:&rolls,other};
    let built = {
        let st = setting(None);
        crate::util::par::indices(faces.len(),|i| face(&st,&sheets[&faces[i].surface],&faces[i])).into_iter().collect::<Result<Vec<_>,_>>()?
    };
    // the solid of the faces built: the edges a seam cuts cut, the poles closed, the tolerances
    // measured, and the boundary checked
    let finish = |built: Vec<Built>| -> Result<Given,Refusal> {
        let mut b = b.clone();
        let mut edges: Vec<u32> = w.written.iter().map(|x| x.entity).collect();
        let mut vertices = w.points.clone();
        let mut cuts: Vec<(usize,Cut)> = Vec::new();
        let mut named: Vec<(usize,u32)> = Vec::new();
        for (fi,mut x) in built.into_iter().enumerate() {
            let (at,pole) = (b.edges.len() as u32,b.vertices.len() as u32);
            for c in x.face.loops.iter_mut().flatten() { if c.edge > LAID-16 { c.edge = at+(LAID-c.edge); } }
            for e in &mut x.laid { for v in &mut e.v { if *v > LAID-16 { *v = pole+(LAID-*v); } } }
            cuts.extend(x.cuts.iter().map(|c| (fi,Cut {vertex:pole+(LAID-c.vertex),laid:at+(LAID-c.laid),..*c})));
            named.extend(x.names.iter().enumerate().map(|(k,&e)| (at as usize+k,e)));
            b.edges.extend(x.laid.drain(..));
            for p in x.poles { b.vertices.push(Vertex {p,tol:0.}); vertices.push(0); }
            b.faces.push(x.face);
        }
        // a file's edge a band's seam meets, cut there: the edge keeps its first piece, its far piece
        // laid on its curve, and every other face's use of it walked along both
        let mut laid_of: Vec<(usize,usize)> = Vec::new();
        for &(fi,c) in &cuts {
            if laid_of.iter().any(|&(e,_)| e == c.edge) {
                return Err(Refusal::unsupported(w.written[c.edge].entity,"an edge two bands' seams meet: not read"))
            }
            let whole = b.edges[c.edge].clone();
            for (gi,g) in b.faces.iter_mut().enumerate() {
                if gi == fi { continue }
                for l in &mut g.loops {
                    let mut out = Vec::with_capacity(l.len()+1);
                    for co in l.drain(..) {
                        if co.edge as usize != c.edge { out.push(co); continue }
                        let m = co.pcurve.at(c.t,&whole,&g.surface,&b.vertices);
                        let (near,far) = cut_pcurve(&co.pcurve,m);
                        let (a,z) = (Coedge {edge:c.edge as u32,reversed:co.reversed,pcurve:near},Coedge {edge:c.laid,reversed:co.reversed,pcurve:far});
                        if co.reversed { out.push(z); out.push(a); } else { out.push(a); out.push(z); }
                    }
                    *l = out;
                }
            }
            b.edges[c.edge].t[1] = c.t;
            b.edges[c.edge].v[1] = c.vertex;
            laid_of.push((c.edge,c.laid as usize));
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
        b.close_poles();
        // (twice: a vertex's tolerance, measured, is how far an inverse curve's end reads as on a pole
        // there, which the edges' tolerances are then measured with)
        b.measure_tolerances();
        b.measure_tolerances();
        // (a foot searched for on a spline reads the last bits of a gap differently from one reading to
        // the next: the gap measured is held to a billionth more)
        for e in &mut b.edges { e.tol *= 1.+1e-9; }
        edges.resize(b.edges.len(),0);
        // (a cut edge's far piece the file's one entity)
        for &(e,l) in &laid_of { edges[l] = edges[e]; }
        // (a seam laid where the file writes it as a point, named by that edge)
        for &(k,e) in &named { if let Some(x) = edges.get_mut(k) { *x = e; } }
        let given = Given {name:found.name.clone(),entity:r,brep:b,context:found.context,faces:faces.iter().map(|f| f.entity).collect(),edges,vertices};
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
        Ok(given)
    };
    let mut built = twice(&setting(Some(false)),&setting(Some(true)),&sheets,&faces,built,&finish,&found.context)?;
    for &fi in rowed {
        let face = built.get(fi).and_then(|x| rows(&b,&x.face,(100.*uncertainty).max(1e-3*size)))
            .ok_or_else(|| Refusal::unsupported(faces.get(fi).map_or(r,|f| f.entity),"a face with no use along its sheet's rows read otherwise"))?;
        built[fi].face = face;
    }
    finish(built)
}
