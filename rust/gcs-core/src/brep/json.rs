//! A B-rep handed over as JSON by a native kernel (`gcs-cli/backend/dump.cpp`; phase 1 of
//! docs/rust-kernel-plan.md): its vertices, its edges on their curves, its faces on their surfaces
//! with loops of oriented uses, each use carrying its curve in the face's parameters, read at the
//! edge's own parameter (`Pcurve::Curve`). Each face is dumped as its forward self; a reversed one's
//! loops are turned here to run as this B-rep's do.
//!
//! Two things are turned as they are read. A placement whose axes are left-handed (the kernel
//! allows one) becomes right-handed: a plane's normal is `x × y`, which leaves its parameters as
//! they were; a surface of revolution keeps its axis and has `y = z × x`, which runs its `u` the
//! other way, so its pcurves' `u` is negated and the face's sense turned with it. And each edge's
//! tolerance is *measured* — the largest gap between its curve and its pcurves' images, and between
//! its curve's ends and its vertices — never taken from the kernel's own bookkeeping.
#[allow(unused_imports)]
use crate::fmath::Det;
use super::geom::{BSpline,Curve,Frame,Surface,V};
use super::nurbs::Net;
use super::topo::{Brep,Coedge,EdgeCurve,Face,Pcurve};
use crate::json::Json;
use crate::space::{cross,distance,dot};
use std::sync::Arc;

fn field<'a>(j: &'a Json,k: &str) -> Result<&'a Json,String> { j.get(k).ok_or(format!("brep json: no `{k}`")) }
fn vec3(j: &Json) -> V { let a = j.arr(); [a[0].as_f64(),a[1].as_f64(),a[2].as_f64()] }
fn reals(j: &Json) -> Vec<f64> { j.arr().iter().map(Json::as_f64).collect() }

/// A placement's axes as written, and whether they are right-handed.
fn axes(j: &Json) -> Result<(Frame,bool),String> {
    let f = field(j,"frame")?;
    let (o,x,y,z) = (vec3(field(f,"o")?),vec3(field(f,"x")?),vec3(field(f,"y")?),vec3(field(f,"z")?));
    Ok((Frame {o,x,y,z},dot(cross(z,x),y) > 0.))
}

fn curve(j: &Json) -> Result<Curve,String> {
    Ok(match field(j,"kind")?.as_str() {
        "line" => Curve::Line {p:vec3(field(j,"p")?),d:vec3(field(j,"d")?)},
        "circle" => Curve::Circle(axes(j)?.0,field(j,"r")?.as_f64()),
        "ellipse" => Curve::Ellipse(axes(j)?.0,field(j,"a")?.as_f64(),field(j,"b")?.as_f64()),
        "bspline" => {
            if j.get("weights").is_some() { return Err("a rational B-spline curve: not built yet".into()) }
            let poles = field(j,"poles")?.arr().iter().map(vec3).collect();
            Curve::BSpline(Arc::new(BSpline::new(field(j,"degree")?.as_f64() as usize,reals(field(j,"knots")?),poles)?))
        }
        k => return Err(format!("brep json: a curve of kind `{k}`")),
    })
}

/// A surface, and whether its `u` runs against the written placement's (a left-handed placement
/// of a surface of revolution, turned right-handed).
fn surface(j: &Json) -> Result<(Surface,bool),String> {
    let kind = field(j,"kind")?.as_str();
    if kind == "bspline" {
        if j.get("weights").is_some() { return Err("a rational B-spline surface: not built yet".into()) }
        let poles: Vec<Vec<V>> = field(j,"poles")?.arr().iter().map(|row| row.arr().iter().map(vec3).collect()).collect();
        let net = Net {du:field(j,"du")?.as_f64() as usize,dv:field(j,"dv")?.as_f64() as usize,
            uknots:reals(field(j,"uknots")?),vknots:reals(field(j,"vknots")?),poles};
        let (nu,nv) = (net.poles.len(),net.poles.first().map_or(0,Vec::len));
        if net.du == 0 || net.dv == 0 || net.du > super::nurbs::MAX_DEGREE || net.dv > super::nurbs::MAX_DEGREE
            || nu <= net.du || nv <= net.dv || net.poles.iter().any(|r| r.len() != nv)
            || net.uknots.len() != nu+net.du+1 || net.vknots.len() != nv+net.dv+1 {
            return Err("brep json: a B-spline surface whose net and knots do not agree".into())
        }
        return Ok((Surface::BSpline(Frame::new([0.;3],[0.,0.,1.],[1.,0.,0.]),Arc::new(net)),false))
    }
    let (f,right) = axes(j)?;
    let num = |k: &str| field(j,k).map(Json::as_f64);
    // a left-handed placement made right-handed: a plane by its normal, a revolution by its y
    let (turned,flip) = match (right,kind) {
        (true,_) => (f,false),
        (false,"plane") => (Frame {z:cross(f.x,f.y),..f},false),
        (false,_) => (Frame {y:cross(f.z,f.x),..f},true),
    };
    let s = match kind {
        "plane" => Surface::Plane(turned),
        "cylinder" => Surface::Cylinder(turned,num("r")?),
        "cone" => Surface::Cone(turned,num("r")?,num("angle")?),
        "sphere" => Surface::Sphere(turned,num("r")?),
        "torus" => Surface::Torus(turned,num("big")?,num("r")?),
        k => return Err(format!("brep json: a surface of kind `{k}`")),
    };
    Ok((s,flip))
}

/// A curve in a face's parameters, its `u` negated where the face's surface runs it the other way.
/// A rational one (a reader's conic on a plane) is taken as the preimage it is of its edge,
/// `Pcurve::Inverse` from its ends at the edge's parameters `t` — read in homogeneous coordinates,
/// a non-rational curve of `(w x, w y, w)`, so no rational curve is built.
fn pcurve(j: &Json,flip: bool,t: [f64;2]) -> Result<Pcurve,String> {
    if let Some(w) = j.get("weights").filter(|_| field(j,"kind").is_ok_and(|k| k.as_str() == "bspline")) {
        let w = reals(w);
        let poles = field(j,"poles")?.arr().iter().map(vec3).collect::<Vec<V>>();
        if w.len() != poles.len() || w.iter().any(|&x| !(x > 0.)) { return Err("brep json: a rational pcurve's weights".into()) }
        let h = BSpline::new(field(j,"degree")?.as_f64() as usize,reals(field(j,"knots")?),
            poles.iter().zip(&w).map(|(p,&w)| [w*p[0],w*p[1],w]).collect())?;
        let at = |t: f64| { let q = h.point(t); [if flip { -q[0]/q[2] } else { q[0]/q[2] },q[1]/q[2]] };
        return Ok(Pcurve::Inverse {a:at(t[0]),b:at(t[1])})
    }
    let mut c = curve(j)?;
    if flip {
        let neg = |p: V| [-p[0],p[1],p[2]];
        c = match c {
            Curve::Line {p,d} => Curve::Line {p:neg(p),d:neg(d)},
            Curve::BSpline(b) => Curve::BSpline(Arc::new(BSpline {poles:b.poles.iter().map(|&p| neg(p)).collect(),..(*b).clone()})),
            other => other,
        };
    }
    Ok(Pcurve::Curve(Arc::new(c)))
}

/// The B-rep the JSON describes, each edge's tolerance measured.
pub fn read(text: &str) -> Result<Brep,String> {
    let j = crate::json::parse(text)?;
    let mut b = Brep::default();
    for v in field(&j,"vertices")?.arr() { b.vertex(vec3(field(v,"p")?)); }
    for e in field(&j,"edges")?.arr() {
        let v = field(e,"v")?.arr();
        let (v0,v1) = (v[0].as_i64() as u32,v[1].as_i64() as u32);
        if v0 as usize >= b.vertices.len() || v1 as usize >= b.vertices.len() { return Err("brep json: an edge's vertex out of range".into()) }
        let t = reals(field(e,"t")?);
        let c = if e.get("degenerate").is_some_and(Json::as_bool) { EdgeCurve::Degenerate } else { EdgeCurve::Curve(curve(field(e,"curve")?)?) };
        b.edge(c,[t[0],t[1]],[v0,v1]);
    }
    for f in field(&j,"faces")?.arr() {
        let (s,flip) = surface(field(f,"surface")?)?;
        let reversed = field(f,"reversed")?.as_bool() != flip;
        let mut loops = Vec::new();
        for l in field(f,"loops")?.arr() {
            let mut found = Vec::new();
            for u in field(l,"uses")?.arr() {
                let edge = field(u,"edge")?.as_i64() as u32;
                if edge as usize >= b.edges.len() { return Err("brep json: a use's edge out of range".into()) }
                let t = b.edges[edge as usize].t;
                found.push(Coedge {edge,reversed:field(u,"reversed")?.as_bool(),pcurve:pcurve(field(u,"pcurve")?,flip,t)?});
            }
            let mut uses = chained(found,&b,&s);
            // a reversed face's loops run the other way round from its forward self's, as dumped
            if field(f,"reversed")?.as_bool() {
                uses = uses.into_iter().rev().map(|c| Coedge {reversed:!c.reversed,..c}).collect();
            }
            // the outer loop first
            if field(l,"outer")?.as_bool() { loops.insert(0,uses) } else { loops.push(uses) }
        }
        b.faces.push(Face {surface:s,reversed,loops,name:String::new()});
    }
    measure(&mut b);
    Ok(b)
}

/// A loop's uses in walking order: each the next whose start is the last one's end vertex, the one
/// nearest it in the face's parameters where several are (a seam's two uses, a closed edge), since
/// the kernel's own order need not be a walk.
fn chained(mut left: Vec<Coedge>,b: &Brep,s: &Surface) -> Vec<Coedge> {
    let ends = |c: &Coedge| {
        let e = &b.edges[c.edge as usize];
        let (t0,t1) = if c.reversed { (e.t[1],e.t[0]) } else { (e.t[0],e.t[1]) };
        let (v0,v1) = if c.reversed { (e.v[1],e.v[0]) } else { (e.v[0],e.v[1]) };
        ((v0,c.pcurve.at(t0,e,s,&b.vertices)),(v1,c.pcurve.at(t1,e,s,&b.vertices)))
    };
    let mut walk = Vec::with_capacity(left.len());
    if left.is_empty() { return walk }
    walk.push(left.remove(0));
    while !left.is_empty() {
        let (v,uv) = ends(walk.last().unwrap()).1;
        let gap = |c: &Coedge| { let (w,p) = ends(c).0; (w != v,(p[0]-uv[0]).dhypot(p[1]-uv[1])) };
        let k = (0..left.len()).min_by(|&i,&j| { let (a,b) = (gap(&left[i]),gap(&left[j])); a.0.cmp(&b.0).then(a.1.total_cmp(&b.1)) }).unwrap();
        walk.push(left.remove(k));
    }
    walk
}

/// Each edge's tolerance: the largest gap between its curve and each use's pcurve image (sampled
/// along it, its knots and ends included), and between its curve's ends and its vertices.
pub fn measure(b: &mut Brep) {
    let mut gaps = vec![0_f64;b.edges.len()];
    for f in &b.faces {
        for c in f.loops.iter().flatten() {
            let e = &b.edges[c.edge as usize];
            let gap = &mut gaps[c.edge as usize];
            let mut ts: Vec<f64> = (0..=32).map(|k| e.t[0]+(e.t[1]-e.t[0])*k as f64/32.).collect();
            if let EdgeCurve::Curve(curve) = &e.curve { ts.extend(curve.breaks(e.t)); }
            if let Pcurve::Curve(pc) = &c.pcurve { ts.extend(pc.breaks(e.t)); }
            for t in ts {
                let d = distance(f.surface.point(c.pcurve.at(t,e,&f.surface,&b.vertices)),e.point(t,&b.vertices));
                *gap = gap.max(d);
            }
        }
    }
    for (e,gap) in b.edges.iter_mut().zip(gaps) {
        let ends = match &e.curve {
            EdgeCurve::Curve(c) => (0..2).map(|k| distance(c.point(e.t[k]),b.vertices[e.v[k] as usize].p)).fold(0.,f64::max),
            EdgeCurve::Degenerate => 0.,
        };
        e.tol = gap.max(ends);
    }
}
