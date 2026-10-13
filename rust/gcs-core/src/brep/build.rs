//! The primitives, built with their topology whole: a profile swept along its normal (a prism),
//! turned about a line in its plane (a revolution), or lofted along a guide to a second section,
//! from the loops of lines, arcs, circles and B-splines the CAD recipe carries
//! (`solid::cad::recipe`).
#[allow(unused_imports)]
use crate::fmath::Det;
use super::geom::{tolerance,BSpline,Blend,Carry,Curve,Frame,Rigid,Surface,Uv,V};
use std::sync::Arc;
use super::topo::{reverse_loop,Brep,Coedge,EdgeCurve,Face,Pcurve};
use super::vec3;
use crate::json::Json;
use crate::space::{add,cross,distance,dot,norm,scale,sub,unit};

const TAU: f64 = std::f64::consts::TAU;

/// One edge of a profile: a line, an arc (a whole circle where `span` is none) turning
/// counter-clockwise about its frame's `z`, which is the profile's normal, or a B-spline whose
/// ends are its first and last poles.
#[derive(Clone,Debug)]
pub enum ProfileEdge { Line { a: V,b: V },Arc { frame: Frame,r: f64,span: Option<[f64;2]> },Spline(Arc<BSpline>) }

/// A planar region: an outer loop and its holes, each a closed walk of edges (listed in any
/// order and direction).
#[derive(Clone,Debug)]
pub struct Profile { pub origin: V,pub normal: V,pub loops: Vec<Vec<ProfileEdge>>,
    /// Each loop's edges' names, as the document spells them (`block.side_l`'s `side_l`): the face
    /// a sweep makes of an edge is named by it, as the faceted kernel names it. Empty where unnamed.
    pub names: Vec<Vec<String>> }

impl Profile {
    /// The same section carried by `m`.
    pub fn moved(&self,m: &Rigid) -> Profile {
        let loops = self.loops.iter().map(|l| l.iter().map(|e| match e {
            ProfileEdge::Line {a,b} => ProfileEdge::Line {a:m.point(*a),b:m.point(*b)},
            ProfileEdge::Arc {frame,r,span} => ProfileEdge::Arc {frame:frame.moved(m),r:*r,span:*span},
            ProfileEdge::Spline(b) => {
                let mut b = (**b).clone();
                b.poles = b.poles.iter().map(|p| m.point(*p)).collect();
                ProfileEdge::Spline(Arc::new(b))
            }
        }).collect()).collect();
        Profile {origin:m.point(self.origin),normal:m.vector(self.normal),loops,names:self.names.clone()}
    }
    /// The name of loop `l`'s edge `k`, or nothing.
    pub fn name(&self,l: usize,k: usize) -> String { self.names.get(l).and_then(|n| n.get(k)).cloned().unwrap_or_default() }
    /// From a recipe's `profile` (millimetres).
    pub fn from_json(j: &Json) -> Result<Profile,String> {
        fn field<'a>(j: &'a Json,k: &str) -> Result<&'a Json,String> { super::field(j,k,"profile") }
        let mut loops = Vec::new();
        let mut names = Vec::new();
        for l in field(j,"loops")?.arr() {
            let mut edges = Vec::new();
            names.push(l.arr().iter().map(|e| e.get("name").map_or(String::new(),|n| n.as_str().to_string())).collect());
            for e in l.arr() {
                edges.push(match field(e,"kind")?.as_str() {
                    "line" => ProfileEdge::Line {a:vec3(field(e,"start")?),b:vec3(field(e,"end")?)},
                    "circle" => {
                        let frame = Frame::new(vec3(field(e,"center")?),vec3(field(e,"normal")?),vec3(field(e,"x_dir")?));
                        let span = e.get("angles").map(|a| {
                            let (a0,a1) = (a.arr()[0].as_f64(),a.arr()[1].as_f64());
                            let sweep = (a1-a0).rem_euclid(TAU);
                            [a0,a0+if sweep == 0. { TAU } else { sweep }]
                        });
                        ProfileEdge::Arc {frame,r:field(e,"radius")?.as_f64(),span}
                    }
                    "bspline" => {
                        let poles = field(e,"poles")?.arr().iter().map(vec3).collect();
                        let knots = field(e,"knots")?.arr().iter().map(|k| k.as_f64()).collect();
                        let degree = field(e,"degree")?.as_f64() as usize;
                        let b = match e.get("weights") {
                            Some(w) => BSpline::rational(degree,knots,poles,w.arr().iter().map(|x| x.as_f64()).collect()),
                            None => BSpline::new(degree,knots,poles),
                        };
                        ProfileEdge::Spline(Arc::new(b.map_err(|m| format!("profile: {m}"))?))
                    }
                    k => return Err(format!("profile: an edge of kind `{k}`")),
                });
            }
            loops.push(edges);
        }
        Ok(Profile {origin:vec3(field(j,"origin")?),normal:unit(vec3(field(j,"normal")?)),loops,names})
    }

    /// As a recipe writes it (`from_json`'s inverse), its lengths times `scale`: lines, arcs and
    /// B-splines (their weights where rational), each named where it is. It never fails.
    pub fn to_json(&self,scale: f64) -> Result<Json,String> {
        use crate::json::object;
        let v = |p: V,s: f64| Json::Arr(p.iter().map(|&x| Json::from(x*s)).collect());
        let mut loops = Vec::new();
        for (l,edges) in self.loops.iter().enumerate() {
            let mut out = Vec::new();
            for (k,e) in edges.iter().enumerate() {
                let mut j = match *e {
                    ProfileEdge::Line {a,b} => object([("kind","line".into()),("start",v(a,scale)),("end",v(b,scale))]),
                    ProfileEdge::Arc {frame,r,span} => {
                        let mut o = object([("kind","circle".into()),("center",v(frame.o,scale)),
                            ("normal",v(frame.z,1.)),("x_dir",v(frame.x,1.)),("radius",(r*scale).into())]);
                        if let Some([a0,a1]) = span { o.set("angles",Json::Arr(vec![a0.into(),a1.into()])); }
                        o
                    }
                    ProfileEdge::Spline(ref b) => {
                        let mut o = object([("kind","bspline".into()),("degree",b.degree.into()),
                            ("knots",Json::Arr(b.knots.iter().map(|&k| k.into()).collect())),
                            ("poles",Json::Arr(b.poles.iter().map(|&p| v(p,scale)).collect()))]);
                        if let Some(w) = &b.weights { o.set("weights",Json::Arr(w.iter().map(|&x| x.into()).collect())); }
                        o
                    }
                };
                let name = self.name(l,k);
                if !name.is_empty() { j.set("name",name.into()); }
                out.push(j);
            }
            loops.push(Json::Arr(out));
        }
        Ok(object([("origin",v(self.origin,scale)),("normal",v(self.normal,1.)),("loops",Json::Arr(loops))]))
    }

    /// A length the profile's tolerances are taken against.
    fn size(&self) -> f64 {
        let mut s: f64 = 0.;
        for l in &self.loops { for e in l { match *e {
            ProfileEdge::Line {a,b} => s = s.max(norm(sub(a,self.origin))).max(norm(sub(b,self.origin))),
            ProfileEdge::Arc {frame,r,..} => s = s.max(norm(sub(frame.o,self.origin))+r),
            ProfileEdge::Spline(ref b) => for &q in &b.poles { s = s.max(norm(sub(q,self.origin))) },
        } } }
        s.max(1e-300)
    }
}

/// One step of a walk: a curve from `t[0]` to `t[1]` (either way round), and which of its loop's
/// edges (in written order) it is.
#[derive(Clone,Debug)]
pub struct Seg { pub curve: Curve,pub t: [f64;2],pub source: usize }

impl Seg {
    pub fn start(&self) -> V { self.curve.point(self.t[0]) }
    pub fn end(&self) -> V { self.curve.point(self.t[1]) }
    pub fn forward(&self) -> bool { self.t[1] > self.t[0] }
    fn reversed(&self) -> Seg { Seg {curve:self.curve.clone(),t:[self.t[1],self.t[0]],source:self.source} }
    /// The edge's parameters, increasing.
    pub fn span(&self) -> [f64;2] { if self.forward() { self.t } else { [self.t[1],self.t[0]] } }
    fn samples(&self,n: usize) -> impl Iterator<Item = V> + '_ {
        (0..n).map(move |j| self.curve.point(self.t[0]+(self.t[1]-self.t[0])*j as f64/n as f64))
    }
}

/// Each loop of a profile as a closed walk, the outer one counter-clockwise in `(x, y)` and the
/// holes clockwise, where `coords` reads a point's coordinates in the plane.
pub fn walks(p: &Profile,coords: &dyn Fn(V) -> [f64;2]) -> Result<Vec<Vec<Seg>>,String> {
    let tol = tolerance(p.size());
    let mut out = Vec::new();
    for (li,l) in p.loops.iter().enumerate() {
        let segs: Vec<Seg> = l.iter().enumerate().map(|(source,e)| match *e {
            ProfileEdge::Line {a,b} => {
                let d = sub(b,a);
                Seg {curve:Curve::Line {p:a,d:unit(d)},t:[0.,norm(d)],source}
            }
            ProfileEdge::Arc {frame,r,span} => Seg {curve:Curve::Circle(frame,r),t:span.unwrap_or([0.,TAU]),source},
            ProfileEdge::Spline(ref b) => Seg {curve:Curve::BSpline(b.clone()),t:b.domain(),source},
        }).collect();
        if segs.iter().any(|s| (s.t[1]-s.t[0]).abs() <= tol) { return Err(format!("loop {li} has an edge of no length")) }
        let mut used = vec![false;segs.len()];
        let mut walk = vec![segs[0].clone()];
        used[0] = true;
        while walk.len() < segs.len() {
            let end = walk.last().unwrap().end();
            let next = (0..segs.len()).filter(|&i| !used[i]).find_map(|i| {
                if distance(segs[i].start(),end) <= tol { Some((i,segs[i].clone())) }
                else if distance(segs[i].end(),end) <= tol { Some((i,segs[i].reversed())) }
                else { None }
            });
            let Some((i,s)) = next else { return Err(format!("loop {li} does not close: nothing continues it")) };
            used[i] = true;
            walk.push(s);
        }
        if distance(walk.last().unwrap().end(),walk[0].start()) > tol { return Err(format!("loop {li} does not close")) }
        let pts: Vec<[f64;2]> = walk.iter().flat_map(|s| s.samples(64)).map(|q| coords(q)).collect();
        let n = pts.len();
        let area = (0..n).map(|i| pts[i][0]*pts[(i+1)%n][1]-pts[i][1]*pts[(i+1)%n][0]).sum::<f64>()/2.;
        if area == 0. { return Err(format!("loop {li} encloses nothing")) }
        if (area > 0.) != (li == 0) { walk = walk.iter().rev().map(Seg::reversed).collect(); }
        out.push(walk);
    }
    Ok(out)
}

/// The use of a walk's step in the walk's direction (`along`) or against it.
fn use_of(edge: u32,s: &Seg,along: bool,pcurve: Pcurve) -> Coedge { Coedge {edge,reversed:s.forward() != along,pcurve} }

/// A plane's pcurve for an edge: its inverse (a plane has no branches).
fn on_plane() -> Pcurve { Pcurve::Inverse {a:[0.;2],b:[0.;2]} }

/// The profile swept along its normal from `from` to `to` (ordinates along the normal).
pub fn prism(p: &Profile,from: f64,to: f64) -> Result<Brep,String> {
    let (h0,h1) = (from.min(to),from.max(to));
    if !(h1 > h0) { return Err("a prism of no depth".into()) }
    let n = unit(p.normal);
    let plane = Frame::about(p.origin,n);
    let walks = walks(p,&|q| { let l = plane.local(q); [l[0],l[1]] })?;
    let named = |li: usize,s: &Seg| p.name(li,s.source);
    let mut b = Brep::default();
    let lift = |q: V,h: f64| add(q,scale(n,h));
    let (mut bottom_loops,mut top_loops) = (Vec::new(),Vec::new());
    for (li,walk) in walks.iter().enumerate() {
        let m = walk.len();
        let bv: Vec<u32> = walk.iter().map(|s| b.vertex(lift(s.start(),h0))).collect();
        let tv: Vec<u32> = walk.iter().map(|s| b.vertex(lift(s.start(),h1))).collect();
        let up: Vec<u32> = (0..m).map(|k| b.edge(EdgeCurve::Curve(Curve::Line {p:walk[k].start(),d:n}),[h0,h1],[bv[k],tv[k]])).collect();
        let rim = |h: f64,v: &[u32],b: &mut Brep| -> Vec<u32> { (0..m).map(|k| {
            let s = &walk[k];
            let curve = s.curve.moved(&Rigid {r:Rigid::identity().r,t:scale(n,h)});
            let ends = [v[k],v[(k+1)%m]];
            b.edge(EdgeCurve::Curve(curve),s.span(),if s.forward() { ends } else { [ends[1],ends[0]] })
        }).collect() };
        let bottom = rim(h0,&bv,&mut b);
        let top = rim(h1,&tv,&mut b);
        for k in 0..m {
            let s = walk[k].clone();
            let k1 = (k+1)%m;
            let (surface,reversed,u0,u1) = match s.curve {
                Curve::Line {..} => {
                    let d = unit(sub(s.end(),s.start()));
                    (Surface::Plane(Frame::new(s.start(),cross(d,n),d)),false,0.,distance(s.end(),s.start()))
                }
                Curve::Circle(f,r) => (Surface::Cylinder(f,r),!s.forward(),s.t[0],s.t[1]),
                // C(u) + v n: the step's tangent turned clockwise about n is its normal
                Curve::BSpline(_) => (Surface::Extrusion(plane,Arc::new(s.curve.clone())),!s.forward(),s.t[0],s.t[1]),
                Curve::Ellipse(..) | Curve::Traced(..) | Curve::Iso(..) => unreachable!("a profile has only lines, circles and splines"),
            };
            // the side's parameters along the walk step, for the step's edge parameter
            let u_at = |t: f64| match s.curve { Curve::Line {..} => (t-s.t[0]).abs(),_ => t };
            let [ta,tb] = s.span();
            let side = vec![
                use_of(bottom[k],&s,true,Pcurve::Line {a:[u_at(ta),h0],b:[u_at(tb),h0]}),
                Coedge {edge:up[k1],reversed:false,pcurve:Pcurve::Line {a:[u1,h0],b:[u1,h1]}},
                use_of(top[k],&s,false,Pcurve::Line {a:[u_at(ta),h1],b:[u_at(tb),h1]}),
                Coedge {edge:up[k],reversed:true,pcurve:Pcurve::Line {a:[u0,h0],b:[u0,h1]}},
            ];
            b.faces.push(Face {surface,reversed,loops:vec![side],name:named(li,&s)});
        }
        bottom_loops.push((0..m).rev().map(|k| use_of(bottom[k],&walk[k],false,on_plane())).collect::<Vec<_>>());
        top_loops.push((0..m).map(|k| use_of(top[k],&walk[k],true,on_plane())).collect::<Vec<_>>());
    }
    let cap = |h: f64| Surface::Plane(Frame {o:lift(plane.o,h),..plane});
    // named as the document names them: `near` the cap toward the viewer (along the normal), as
    // the faceted kernel and `against` read them
    b.faces.push(Face {surface:cap(h0),reversed:true,loops:bottom_loops,name:"far".into()});
    b.faces.push(Face {surface:cap(h1),reversed:false,loops:top_loops,name:"near".into()});
    Ok(b)
}

/// What names a drafted prism's section at its parting plane while its halves are built: gone once
/// they are glued there.
const PARTING: &str = "#parting";

/// The profile swept along its normal from `from` to `to`, its sides tapered by `draft` (radians)
/// from its own plane, the parting plane (#245): the section at ordinate `h` is the profile moved
/// `|h| tan draft` into its material (`offset`), so a straight side is a plane and a circle's a
/// cone, each leaning away from the plane either side of it. A prism crossing the plane is two
/// lofts (`ruled`), one each side, glued at the section they share; its sides there are two faces
/// of one name, its caps named as a prism's.
pub fn drafted(p: &Profile,from: f64,to: f64,draft: f64) -> Result<Brep,String> {
    if draft == 0. { return prism(p,from,to) }
    let (h0,h1) = (from.min(to),from.max(to));
    if !(h1 > h0) { return Err("a prism of no depth".into()) }
    let n = unit(p.normal);
    let taper = draft.dtan();
    let section = |h: f64| -> Result<(f64,Profile),String> {
        let q = offset(p,h.abs()*taper)?;
        Ok((h,q.moved(&Rigid {r:Rigid::identity().r,t:scale(n,h)})))
    };
    let piece = |(a,start): &(f64,Profile),(b,end): &(f64,Profile)| -> Result<Brep,String> {
        let mut s = ruled(start,end,Carry::Line {delta:scale(n,b-a)})?;
        let k = s.faces.len();
        s.faces[k-2].name = if *a == h0 { "far".into() } else { PARTING.into() };
        s.faces[k-1].name = if *b == h1 { "near".into() } else { PARTING.into() };
        Ok(s)
    };
    let (lo,hi) = (section(h0)?,section(h1)?);
    if !(h0 < 0. && h1 > 0.) { return piece(&lo,&hi) }
    let tol = tolerance(p.size()+h1-h0);
    let parting = section(0.)?;
    let b = super::boolean::glued(&piece(&lo,&parting)?,&piece(&parting,&hi)?,tol)
        .ok_or("a drafted prism's halves do not meet at its parting plane")?;
    if b.faces.iter().any(|f| f.name == PARTING) { return Err("a drafted prism's halves meet across more than its parting plane".into()) }
    Ok(b)
}

/// The profile moved `d` into its material, each side along its own normal (#245): a line moved
/// square to itself, a circle's radius less `d` where its material is inside it (more where it is
/// outside, a hole's), each corner where the moved sides either side of it meet. Where those are
/// two lines the corner is their crossing; where the sides go on tangent the corner moves square
/// to both. An arc meeting another side at an angle is refused (its cone and the side's plane meet
/// along a curve, which no loft carries), as is a spline, and a side the move takes to nothing.
fn offset(p: &Profile,d: f64) -> Result<Profile,String> {
    if d == 0. { return Ok(p.clone()) }
    let n = unit(p.normal);
    let frame = Frame::about(p.origin,n);
    let walks = walks(p,&|q| { let l = frame.local(q); [l[0],l[1]] })?;
    let tol = tolerance(p.size());
    let mut out = p.clone();
    for (li,walk) in walks.iter().enumerate() {
        let m = walk.len();
        let named = |s: &Seg| match p.name(li,s.source) { x if x.is_empty() => "a side".to_string(),x => format!("`{x}`") };
        // a step's direction along the walk at its parameter `t`, and into its material: its left
        let along = |s: &Seg,t: f64| unit(scale(s.curve.tangent(t),if s.forward() { 1. } else { -1. }));
        let inward = |s: &Seg,t: f64| cross(n,along(s,t));
        for s in walk {
            if !matches!(s.curve,Curve::Line {..} | Curve::Circle(..)) {
                return Err(format!("{}: a drafted side of a spline is not built yet",named(s)))
            }
        }
        // each corner, where step `k` starts
        let corners: Vec<V> = (0..m).map(|k| -> Result<V,String> {
            let (a,b) = (&walk[(k+m-1)%m],&walk[k]);
            let c = b.start();
            let (ta,tb) = (along(a,a.t[1]),along(b,b.t[0]));
            if norm(cross(ta,tb)) <= 1e-9 && dot(ta,tb) > 0. { return Ok(add(c,scale(inward(b,b.t[0]),d))) }
            let (Curve::Line {..},Curve::Line {..}) = (&a.curve,&b.curve) else {
                return Err(format!("{} meets {} at an angle where one is an arc: a drafted corner there \
                    is a curve where a cone meets a plane, not built yet",named(a),named(b)))
            };
            // the moved lines, each through the corner moved square to it, crossing
            let (pa,pb) = (add(c,scale(inward(a,a.t[1]),d)),add(c,scale(inward(b,b.t[0]),d)));
            let w = dot(cross(ta,tb),n);
            if w.abs() <= 1e-12 { return Err(format!("{} turns straight back at a corner",named(b))) }
            Ok(add(pa,scale(ta,dot(cross(sub(pb,pa),tb),n)/w)))
        }).collect::<Result<_,_>>()?;
        for (k,s) in walk.iter().enumerate() {
            let (start,end) = (corners[k],corners[(k+1)%m]);
            let gone = || format!("a draft takes {} to nothing",named(s));
            out.loops[li][s.source] = match p.loops[li][s.source] {
                ProfileEdge::Line {a,b} => {
                    let (a2,b2) = if s.forward() { (start,end) } else { (end,start) };
                    if dot(sub(b2,a2),sub(b,a)) <= tol*tol { return Err(gone()) }
                    ProfileEdge::Line {a:a2,b:b2}
                }
                ProfileEdge::Arc {frame:f,r,span} => {
                    let radial = unit(sub(s.start(),f.o));
                    let r2 = r+d*dot(inward(s,s.t[0]),radial);
                    if !(r2 > tol) { return Err(gone()) }
                    let span = match span {
                        None => None,
                        Some([a0,a1]) => {
                            let angle = |q: V,near: f64| super::geom::around(Curve::Circle(f,r2).inverse(q),near-0.5*TAU,TAU);
                            let (q0,q1) = if s.forward() { (start,end) } else { (end,start) };
                            let (b0,b1) = (angle(q0,a0),angle(q1,a1));
                            if !(b1 > b0) { return Err(gone()) }
                            Some([b0,b1])
                        }
                    };
                    ProfileEdge::Arc {frame:f,r:r2,span}
                }
                ProfileEdge::Spline(_) => unreachable!("refused above"),
            };
        }
    }
    Ok(out)
}

/// What a profile step sweeps as it turns about the axis.
#[derive(Clone,Copy,Debug,PartialEq)]
enum Swept { Axis,Disk,Cylinder,Cone,Sphere,Torus,Spline }

/// The profile turned about the line through `origin` along `axis` by `angle` (right-handed; a
/// negative angle turns the other way, and anything past a turn is a whole one).
pub fn revolve(p: &Profile,origin: V,axis: V,angle: f64) -> Result<Brep,String> {
    let z = scale(unit(axis),angle.signum());
    let theta = angle.abs().min(TAU);
    if !(theta > 0.) { return Err("a revolution through no angle".into()) }
    let full = theta >= TAU-1e-12;
    let tol = tolerance(p.size());
    let normal = unit(p.normal);
    if dot(normal,z).abs() > 1e-9 || dot(sub(origin,p.origin),normal).abs() > tol {
        return Err("a revolution's axis must lie in its profile's plane".into())
    }
    // x: square to the axis in the profile's plane, toward the profile
    let mut x = unit(cross(z,normal));
    let far = p.loops.iter().flatten().flat_map(|e| match *e {
        ProfileEdge::Line {a,b} => vec![a,b],
        // points along the arc itself (its frame's x may lie along the axis: a semicircle's ends)
        ProfileEdge::Arc {frame,r,span} => {
            let [a0,a1] = span.unwrap_or([0.,TAU]);
            (0..=16).map(|k| { let t = a0+(a1-a0)*k as f64/16.; add(frame.o,add(scale(frame.x,r*t.dcos()),scale(frame.y,r*t.dsin()))) }).collect()
        }
        ProfileEdge::Spline(ref b) => b.poles.clone(),
    }).map(|q| dot(sub(q,origin),x)).fold(0.,|m: f64,v| if v.abs() > m.abs() { v } else { m });
    if far < 0. { x = scale(x,-1.); }
    let f = Frame::new(origin,z,x);
    let rz = |q: V| { let l = f.local(q); [l[0],l[2]] };
    let walks = walks(p,&rz)?;
    for s in walks.iter().flatten() {
        if s.samples(64).chain([s.end()]).any(|q| rz(q)[0] < -tol) { return Err("a revolution's profile crosses its axis".into()) }
    }
    let turn = Rigid::turn(origin,z,theta);
    let mut b = Brep::default();
    let (mut cap0,mut cap1) = (Vec::new(),Vec::new());
    for (li,walk) in walks.iter().enumerate() {
        let m = walk.len();
        let pts: Vec<V> = walk.iter().map(|s| s.start()).collect();
        let on_axis: Vec<bool> = pts.iter().map(|&q| rz(q)[0] <= tol).collect();
        let v0: Vec<u32> = pts.iter().map(|&q| b.vertex(q)).collect();
        let v1: Vec<u32> = (0..m).map(|k| if full || on_axis[k] { v0[k] } else { b.vertex(turn.point(pts[k])) }).collect();
        let kinds: Vec<Swept> = walk.iter().enumerate().map(|(k,s)| {
            // an end on the axis to the tolerance is on it: a cone placed at its apex has radius
            // zero there, never the rounding either side of it (a negative one no reader takes)
            let on = |r: f64| if r.abs() <= tol { 0. } else { r };
            let ([r0,z0],[r1,z1]) = (rz(s.start()),rz(s.end()));
            let (r0,r1) = (on(r0),on(r1));
            match &s.curve {
                Curve::Line {..} if on_axis[k] && on_axis[(k+1)%m] => Swept::Axis,
                Curve::Line {..} if (z0-z1).abs() <= tol => Swept::Disk,
                Curve::Line {..} if (r0-r1).abs() <= tol => Swept::Cylinder,
                Curve::Line {..} => Swept::Cone,
                Curve::Circle(c,_) if rz(c.o)[0].abs() <= tol => Swept::Sphere,
                Curve::BSpline(_) => Swept::Spline,
                _ => Swept::Torus,
            }
        }).collect();
        // the circle each profile vertex off the axis turns through
        let circles: Vec<Option<u32>> = (0..m).map(|k| (!on_axis[k]).then(|| {
            let [r,h] = rz(pts[k]);
            let c = Curve::Circle(Frame {o:f.at([0.,0.,h]),..f},r);
            b.edge(EdgeCurve::Curve(c),[0.,theta],[v0[k],v1[k]])
        })).collect();
        // each step where it starts (φ = 0) and, turned whole, where it ends (φ = θ)
        let needs = |k: usize| !full || !matches!(kinds[k],Swept::Disk | Swept::Axis);
        let first: Vec<Option<u32>> = (0..m).map(|k| needs(k).then(|| {
            let s = &walk[k];
            let ends = [v0[k],v0[(k+1)%m]];
            b.edge(EdgeCurve::Curve(s.curve.clone()),s.span(),if s.forward() { ends } else { [ends[1],ends[0]] })
        })).collect();
        let last: Vec<Option<u32>> = (0..m).map(|k| if full || kinds[k] == Swept::Axis { first[k] } else {
            let s = &walk[k];
            let ends = [v1[k],v1[(k+1)%m]];
            Some(b.edge(EdgeCurve::Curve(s.curve.moved(&turn)),s.span(),if s.forward() { ends } else { [ends[1],ends[0]] }))
        }).collect();
        for k in 0..m {
            let s = walk[k].clone();
            let k1 = (k+1)%m;
            // an end on the axis to the tolerance is on it: a cone placed at its apex has radius
            // zero there, never the rounding either side of it (a negative one no reader takes)
            let on = |r: f64| if r.abs() <= tol { 0. } else { r };
            let ([r0,z0],[r1,z1]) = (rz(s.start()),rz(s.end()));
            let (r0,r1) = (on(r0),on(r1));
            // the material's outward normal at the step's middle, in (ρ, z): right of the walk
            let mid = (s.t[0]+s.t[1])/2.;
            let (q,dq) = (s.curve.point(mid),s.curve.tangent(mid));
            let d = { let l = f.dir_local(dq); [l[0],l[2]] };
            let sense = if s.forward() { 1. } else { -1. };
            let out = [d[1]*sense,-d[0]*sense];
            let [rq,zq] = rz(q);
            let (surface,normal,v_of): (Surface,[f64;2],Box<dyn Fn(f64) -> f64>) = match kinds[k] {
                Swept::Axis => continue,
                Swept::Disk => (Surface::Plane(Frame {o:f.at([0.,0.,z0]),..f}),[0.,1.],Box::new(|_| 0.)),
                Swept::Cylinder => { let c = s.curve.clone(); (Surface::Cylinder(f,r0),[1.,0.],Box::new(move |t| rz(c.point(t))[1])) }
                Swept::Cone => {
                    let a = ((r1-r0)/(z1-z0)).datan();
                    (Surface::Cone(Frame {o:f.at([0.,0.,z0]),..f},r0,a),[a.dcos(),-a.dsin()],
                        { let c = s.curve.clone(); Box::new(move |t| (rz(c.point(t))[1]-z0)/a.dcos()) })
                }
                Swept::Spline => {
                    // C(v) turned by u: in (ρ, z) its normal is the tangent turned clockwise
                    let l = d[0].dhypot(d[1]);
                    (Surface::Revolution(f,Arc::new(s.curve.clone())),[d[1]/l,-d[0]/l],Box::new(|t| t))
                }
                Swept::Sphere | Swept::Torus => {
                    let Curve::Circle(c,r) = s.curve.clone() else { unreachable!() };
                    let [rc,zc] = rz(c.o);
                    let big = if kinds[k] == Swept::Sphere { 0. } else { rc };
                    let surface = if kinds[k] == Swept::Sphere { Surface::Sphere(Frame {o:f.at([0.,0.,zc]),..f},r) }
                        else { Surface::Torus(Frame {o:f.at([0.,0.,zc]),..f},big,r) };
                    // v at the step's start, and which way it runs with the circle's parameter
                    let [ra,za] = rz(s.start());
                    let va = (za-zc).datan2(ra-big);
                    let dt = { let l = f.dir_local(s.curve.tangent(s.t[0])); [l[0],l[2]] };
                    let sigma = ((ra-big)*dt[1]-(za-zc)*dt[0]).signum();
                    let t0 = s.t[0];
                    (surface,[(rq-big)/r,(zq-zc)/r],Box::new(move |t| va+sigma*(t-t0)))
                }
            };
            let reversed = out[0]*normal[0]+out[1]*normal[1] < 0.;
            let [ta,tb] = s.span();
            let mut l = Vec::new();
            if kinds[k] == Swept::Disk {
                if full {
                    // an annulus or a disk: the outer circle, the inner as a hole
                    let (outer,inner) = if r0 > r1 { (k,k1) } else { (k1,k) };
                    let rim = |i: usize,outer: bool| Coedge {edge:circles[i].unwrap(),reversed:(outer == reversed),pcurve:on_plane()};
                    let mut loops = vec![vec![rim(outer,true)]];
                    if let Some(_) = circles[inner] { loops.push(vec![rim(inner,false)]); }
                    b.faces.push(Face {surface,reversed,loops,name:p.name(li,s.source)});
                    continue;
                }
                l.push(use_of(first[k].unwrap(),&s,true,on_plane()));
                if let Some(c) = circles[k1] { l.push(Coedge {edge:c,reversed:false,pcurve:on_plane()}); }
                l.push(use_of(last[k].unwrap(),&s,false,on_plane()));
                if let Some(c) = circles[k] { l.push(Coedge {edge:c,reversed:true,pcurve:on_plane()}); }
            } else {
                let (va,vb) = (v_of(ta),v_of(tb));
                let (vs,ve) = (v_of(s.t[0]),v_of(s.t[1]));
                let rim = |i: usize,v: f64,reversed: bool,b: &mut Brep| -> Coedge {
                    let pcurve = Pcurve::Line {a:[0.,v],b:[theta,v]};
                    match circles[i] {
                        Some(c) => Coedge {edge:c,reversed,pcurve},
                        None => Coedge {edge:b.edge(EdgeCurve::Degenerate,[0.,theta],[v0[i],v0[i]]),reversed,pcurve},
                    }
                };
                l.push(use_of(first[k].unwrap(),&s,true,Pcurve::Line {a:[0.,va],b:[0.,vb]}));
                l.push(rim(k1,ve,false,&mut b));
                l.push(use_of(last[k].unwrap(),&s,false,Pcurve::Line {a:[theta,va],b:[theta,vb]}));
                l.push(rim(k,vs,true,&mut b));
            }
            let ccw = b.loop_area(&Face {surface:surface.clone(),reversed,loops:vec![],name:String::new()},&l) > 0.;
            if ccw == reversed { l = reverse_loop(l); }
            b.faces.push(Face {surface,reversed,loops:vec![l],name:p.name(li,s.source)});
        }
        if !full {
            cap0.push((0..m).map(|k| use_of(first[k].unwrap(),&walk[k],true,on_plane())).collect::<Vec<_>>());
            cap1.push((0..m).rev().map(|k| use_of(last[k].unwrap(),&walk[k],false,on_plane())).collect::<Vec<_>>());
        }
    }
    if !full {
        b.faces.push(Face {surface:Surface::Plane(Frame::new(origin,cross(x,z),x)),reversed:false,loops:cap0,name:"start".into()});
        let fx = turn.vector(x);
        b.faces.push(Face {surface:Surface::Plane(Frame::new(origin,cross(fx,z),fx)),reversed:true,loops:cap1,name:"end".into()});
    }
    Ok(b)
}

/// What a loft follows: a line (the step from its start to its end), or an arc (its centre, the
/// axis it turns about right-handedly and the angle it turns through). Where it starts is the
/// start section's business, which the faceted kernel checks against it (`solid::loft`).
#[derive(Clone,Copy,Debug)]
pub enum Guide { Line { delta: V },Arc { center: V,axis: V,angle: f64 },
    /// A helix: the arc's turn and a step of `advance` along its axis with it (#238).
    Screw { center: V,axis: V,angle: f64,advance: f64 } }

/// **The tube a ball sweeps** (#238): the ball of radius `r` about `c` carried by the screw about the
/// line through `origin` along `axis` — a turn by `angle` and a step of `advance` along it (none, a
/// turn) — is the points within `r` of the path its centre runs, and those are the ball's disc
/// square to the path carried along it (a point's nearest point on the path inside it lies in the
/// disc there) with the ball where the path starts and where it ends: the disc turned (a torus's
/// stretch, a whole torus past a whole turn) or lofted along the helix (`Guide::Screw`), each end's
/// half ball beyond its disc united with it. Every face is named `name`. The caller's to know the
/// tube meets itself nowhere: the path's centre stands off the axis by more than `r`, and a helix
/// of more than a turn climbs more than `2 r` a turn.
pub fn canal(c: V,r: f64,origin: V,axis: V,angle: f64,advance: f64,name: &str) -> Result<Brep,String> {
    let a = unit(axis);
    let radial = sub(c,add(origin,scale(a,dot(sub(c,origin),a))));
    // the centre's velocity at the start, square to the radial line
    let rate = add(scale(cross(a,radial),angle),scale(a,advance));
    let named = |mut b: Brep| { for f in &mut b.faces { f.name = name.to_string(); } b };
    // a centre that does not move sweeps the ball itself
    if norm(rate) <= 1e-12*r.max(1.) { return Ok(named(ball(c,a,r)?)) }
    let t = unit(rate);
    // where the disc's circle starts, so where the tube's seams run: turned off the radial line by
    // an angle no design's symmetry gives (on it, the arc the seam's vertex sweeps lies in the plane
    // of the centre's path, which a face often does — a groove's floor's block top)
    let x0 = if norm(radial) > 0. { unit(radial) } else { Frame::about(c,t).x };
    let (s,k) = SEAM.dsin_cos();
    let x = add(scale(x0,k),scale(cross(t,x0),s));
    // the balls' poles: leaning off the path by the same angle, toward the seam
    let w = add(scale(t,k),scale(x,s));
    let disc = Profile {origin:c,normal:t,loops:vec![vec![ProfileEdge::Arc {frame:Frame::new(c,t,x),r,span:None}]],names:Vec::new()};
    if advance == 0. && angle.abs() >= TAU-1e-12 {
        return Ok(named(revolve(&disc,origin,a,TAU*angle.signum())?))
    }
    let tube = if advance == 0. { revolve(&disc,origin,a,angle)? }
        else { loft(&disc,None,&Guide::Screw {center:origin,axis:a,angle,advance})? };
    let end = screw(origin,a,angle,advance);
    let union = |s: &Brep,b: Brep| super::recipe::combined(s,&named(b),super::boolean::Op::Union,0.);
    union(&union(&named(tube),half_ball(c,scale(t,-1.),w,r)?)?,half_ball(c,t,w,r)?.moved(&end))
}

/// How far round from the radial line a ball's tube's seams run (radians): the golden angle's
/// fraction of a turn, which no symmetry of a design's makes.
const SEAM: f64 = 0.6180339887498949;

/// The ball of radius `r` about `c`: a half disc turned once about the diameter along the unit `w`,
/// its poles where `w` meets the sphere.
fn ball(c: V,w: V,r: f64) -> Result<Brep,String> {
    let f = Frame::about(c,w);
    let arc = ProfileEdge::Arc {frame:Frame::new(c,f.y,f.z),r,span:Some([0.,std::f64::consts::PI])};
    let shut = ProfileEdge::Line {a:f.at([0.,0.,-r]),b:f.at([0.,0.,r])};
    revolve(&Profile {origin:c,normal:f.y,loops:vec![vec![arc,shut]],names:Vec::new()},c,f.z,TAU)
}

/// The half of the ball of radius `r` about `c` on the side the unit `n` points to: the ball turned
/// about the unit `w` kept beyond the plane through `c` square to `n`. `w` leans off `n` and off
/// that plane, so neither of the sphere's poles is on the disc's circle or on the line through the
/// centre along the path — which lies in the path's plane, as a face of the design often does,
/// and a plane through a pole is a meeting this kernel's Booleans do not split.
fn half_ball(c: V,n: V,w: V,r: f64) -> Result<Brep,String> {
    super::recipe::kept(&ball(c,w,r)?,c,n,0.)
}

/// The screw about the line through `c` along the unit `a`: a turn by `angle` and a step of `advance`
/// along it.
fn screw(c: V,a: V,angle: f64,advance: f64) -> Rigid {
    Rigid::turn(c,a,angle).then(&Rigid {r:Rigid::identity().r,t:scale(a,advance)})
}

/// A section swept along a guide, or lofted from it to a second section at the guide's end (Solvent
/// §6.9: the start section square to the guide at its start, the end square to it at its end).
/// Without an end section a line's sweep is a prism and an arc's a revolution about its axis, and a
/// screw's carries the start section to its own image at the guide's end; with one, each start edge
/// is joined to the end edge written in its place (`ruled`).
pub fn loft(start: &Profile,end: Option<&Profile>,guide: &Guide) -> Result<Brep,String> {
    match (guide,end) {
        (Guide::Line {delta,..},None) => {
            // a prism, its caps (the last two faces, at its lower height and its upper) named as a
            // sweep's are: `start` where the section stands, `end` where the guide takes it
            let d = dot(*delta,unit(start.normal));
            let mut b = prism(start,0.,d)?;
            let n = b.faces.len();
            let (lower,upper) = if d > 0. { ("start","end") } else { ("end","start") };
            b.faces[n-2].name = lower.into();
            b.faces[n-1].name = upper.into();
            Ok(b)
        }
        (Guide::Arc {center,axis,angle,..},None) => revolve(start,*center,*axis,*angle),
        (Guide::Line {delta,..},Some(end)) => ruled(start,end,Carry::Line {delta:*delta}),
        (Guide::Arc {center,axis,angle,..},Some(end)) =>
            ruled(start,end,Carry::Arc {center:*center,axis:unit(*axis),angle:*angle}),
        (&Guide::Screw {center,axis,angle,advance},end) => {
            let carry = Carry::Screw {center,axis:unit(axis),angle,advance};
            let end = match end {
                Some(end) => end.clone(),
                None => start.moved(&screw(center,unit(axis),angle,advance)),
            };
            ruled(start,&end,carry)
        }
    }
}

/// The loft of `start` to `end` as `carry` takes the section along its guide: every point of a
/// start edge joined to the point of the end edge written in its place at the same fraction of its
/// own parameter (an arc's by angle), blended while carried (`Blend`). Carried along a line, edges
/// that are coplanar lines join in a plane and coaxial circles and arcs in step in a cone or a
/// cylinder, and the rails between them are straight; everything else is a `Blend`, and turned
/// about an axis the rails are its iso-curves.
fn ruled(start: &Profile,end: &Profile,carry: Carry) -> Result<Brep,String> {
    let n = unit(start.normal);
    // what carries the end section back to the start, and the direction a line carries along
    let (back,line) = match carry {
        Carry::Line {delta} => (Rigid {r:Rigid::identity().r,t:scale(delta,-1.)},Some(unit(delta))),
        Carry::Arc {center,axis,angle} => (Rigid::turn(center,axis,-angle),None),
        Carry::Screw {center,axis,angle,advance} => (screw(center,axis,-angle,-advance),None),
    };
    let tol = tolerance(start.size()+end.size()+norm(sub(end.origin,start.origin)));
    let frame = Frame::about(start.origin,n);
    let starts = walks(start,&|q| { let l = frame.local(q); [l[0],l[1]] })?;
    let ends = walks(end,&|q| { let l = frame.local(back.point(q)); [l[0],l[1]] })?;
    if starts.len() != ends.len() { return Err("a loft's sections must have as many holes".into()) }
    let mut b = Brep::default();
    let (mut start_loops,mut end_loops) = (Vec::new(),Vec::new());
    for (li,(sw,ew)) in starts.iter().zip(&ends).enumerate() {
        let m = sw.len();
        if ew.len() != m { return Err(format!("a loft's loop {li} has {} edges at its start and {} at its end",m,ew.len())) }
        // the end's steps in the start walk's order, by source edge
        let mut ew: Vec<Seg> = sw.iter().map(|s| ew.iter().find(|x| x.source == s.source).cloned()
            .ok_or("a loft's sections must have their edges written in corresponding order")).collect::<Result<_,_>>()?;
        // a whole circle is joined radially: the end's starts where the start's does, seen from the
        // start circle's centre with the end carried back to it
        if m == 1 {
            if let (Curve::Circle(fa,_),Curve::Circle(fb,rb)) = (&sw[0].curve,&ew[0].curve) {
                let fb = fb.moved(&back);
                let d = sub(sw[0].start(),fa.o);
                let d = unit(sub(d,scale(fa.z,dot(d,fa.z))));
                let t0 = Curve::Circle(fb,*rb).inverse(add(fb.o,scale(d,*rb)));
                ew[0].t = if ew[0].forward() { [t0,t0+TAU] } else { [t0+TAU,t0] };
            }
        }
        for k in 0..m {
            if distance(ew[k].end(),ew[(k+1)%m].start()) > 1e3*tol {
                return Err("a loft's sections must have their edges written in corresponding order".into())
            }
        }
        let pv: Vec<u32> = sw.iter().map(|s| b.vertex(s.start())).collect();
        let qv: Vec<u32> = ew.iter().map(|s| b.vertex(s.start())).collect();
        let curve_edge = |b: &mut Brep,s: &Seg,v: &[u32],k: usize| {
            let ends = [v[k],v[(k+1)%m]];
            b.edge(EdgeCurve::Curve(s.curve.clone()),s.span(),if s.forward() { ends } else { [ends[1],ends[0]] })
        };
        let se: Vec<u32> = (0..m).map(|k| curve_edge(&mut b,&sw[k],&pv,k)).collect();
        let ee: Vec<u32> = (0..m).map(|k| curve_edge(&mut b,&ew[k],&qv,k)).collect();
        let blend = |k: usize| Blend {a:sw[k].curve.clone(),ta:sw[k].t,b:ew[k].curve.moved(&back),tb:ew[k].t,carry,
            closed:m == 1 && matches!(sw[k].curve,Curve::Circle(..)) && matches!(ew[k].curve,Curve::Circle(..))};
        // the rails, each where an edge of the start section begins: straight along a line, the
        // blend's own iso-curve turned about an axis
        let rails: Vec<u32> = (0..m).map(|k| {
            let (p,q) = (sw[k].start(),ew[k].start());
            match line {
                Some(_) => b.edge(EdgeCurve::Curve(Curve::Line {p,d:unit(sub(q,p))}),[0.,distance(p,q)],[pv[k],qv[k]]),
                None => b.edge(EdgeCurve::Curve(Curve::Iso(Arc::new(blend(k)),0.)),[0.,1.],[pv[k],qv[k]]),
            }
        }).collect();
        for k in 0..m {
            let (a,z) = (&sw[k],&ew[k]);
            let k1 = (k+1)%m;
            // outward, in the section's plane: right of a walk counter-clockwise about n
            let mid = (a.t[0]+a.t[1])/2.;
            let d = scale(a.curve.tangent(mid),if a.forward() { 1. } else { -1. });
            let outward = cross(d,n);
            let (surface,mut l) = match line.and_then(|e| analytic_ruled(a,z,e,outward,tol)) {
                Some((surface,rim)) => {
                    let at = |p: V| surface.inverse(p);
                    // the face's parameters at each corner, the rails at the arc's start (u = 0) and end
                    let e = line.unwrap();
                    let turn = if rim { (a.t[1]-a.t[0])*dot(match &a.curve { Curve::Circle(f,_) => f.z,_ => e },e) } else { 0. };
                    let corner = |p: V,u_end: bool| { let mut uv = at(p); if rim { uv[0] = if u_end { turn } else { 0. }; } uv };
                    let (u_p0,u_p1,u_q0,u_q1) = (corner(a.start(),false),corner(a.end(),true),corner(z.start(),false),corner(z.end(),true));
                    let pc = |x: Uv,y: Uv| Pcurve::Inverse {a:x,b:y};
                    let edge_uv = |s: &Seg,first: Uv,last: Uv| if s.forward() { pc(first,last) } else { pc(last,first) };
                    (surface,vec![
                        Coedge {edge:se[k],reversed:!a.forward(),pcurve:edge_uv(a,u_p0,u_p1)},
                        Coedge {edge:rails[k1],reversed:false,pcurve:pc(u_p1,u_q1)},
                        Coedge {edge:ee[k],reversed:z.forward(),pcurve:edge_uv(z,u_q0,u_q1)},
                        Coedge {edge:rails[k],reversed:true,pcurve:pc(u_p0,u_q0)},
                    ])
                }
                None => {
                    // S(u, v) with u along the walk: an edge runs u forward where it is walked forward
                    let along = |s: &Seg,v: f64| if s.forward() { Pcurve::Line {a:[0.,v],b:[1.,v]} } else { Pcurve::Line {a:[1.,v],b:[0.,v]} };
                    (Surface::Blend(frame,Arc::new(blend(k))),vec![
                        Coedge {edge:se[k],reversed:!a.forward(),pcurve:along(a,0.)},
                        Coedge {edge:rails[k1],reversed:false,pcurve:Pcurve::Line {a:[1.,0.],b:[1.,1.]}},
                        Coedge {edge:ee[k],reversed:z.forward(),pcurve:along(z,1.)},
                        Coedge {edge:rails[k],reversed:true,pcurve:Pcurve::Line {a:[0.,0.],b:[0.,1.]}},
                    ])
                }
            };
            let reversed = match &surface {
                Surface::Plane(_) => false,
                // u along the walk, so the walk's middle is u = ½
                Surface::Blend(..) => dot(surface.normal_raw([0.5,0.]),outward) < 0.,
                _ => dot(surface.normal_raw(surface.inverse(a.curve.point(mid))),outward) < 0.,
            };
            let face = Face {surface,reversed,loops:vec![],name:start.name(li,a.source)};
            let ccw = b.loop_area(&face,&l) > 0.;
            if ccw == reversed { l = reverse_loop(l); }
            b.faces.push(Face {loops:vec![l],..face});
        }
        start_loops.push((0..m).map(|k| use_of(se[k],&sw[k],true,on_plane())).collect::<Vec<_>>());
        end_loops.push((0..m).map(|k| use_of(ee[k],&ew[k],true,on_plane())).collect::<Vec<_>>());
    }
    // the caps: the start's material toward the guide, the end's behind it. The guide leaves the
    // start along the way the carry moves the section (read at one of its corners, off any axis)
    let corner = starts[0][0].start();
    let forward = {
        let step = sub(carry.at(corner,1e-6),corner);
        if dot(n,step) < 0. { scale(n,-1.) } else { n }
    };
    let cap = |origin: V,normal: V,outward: V,loops: Vec<Vec<Coedge>>,name: &str,b: &Brep| {
        let surface = Surface::Plane(Frame::about(origin,normal));
        let reversed = dot(normal,outward) < 0.;
        let face = Face {surface,reversed,loops:vec![],name:name.into()};
        let loops = loops.into_iter().enumerate().map(|(li,l)| {
            let ccw = b.loop_area(&face,&l) > 0.;
            // the outer loop turns with the face's sense, its holes against it
            if (li == 0) != (ccw != reversed) { reverse_loop(l) } else { l }
        }).collect();
        Face {loops,..face}
    };
    let start_cap = cap(start.origin,n,scale(forward,-1.),start_loops,"start",&b);
    let end_cap = cap(end.origin,unit(end.normal),carry.dir(forward,1.),end_loops,"end",&b);
    b.faces.push(start_cap);
    b.faces.push(end_cap);
    Ok(b)
}

/// The analytic face joining a start edge to its end edge along the line `e`, where there is one
/// (and whether its parameters run round an axis): a plane for coplanar lines, a cone or cylinder
/// for coaxial circles and arcs in step.
fn analytic_ruled(a: &Seg,z: &Seg,e: V,outward: V,tol: f64) -> Option<(Surface,bool)> {
    match (&a.curve,&z.curve) {
        (Curve::Line {..},Curve::Line {..}) => {
            let (p0,p1,q0,q1) = (a.start(),a.end(),z.start(),z.end());
            let mut normal = cross(sub(p1,p0),sub(q0,p0));
            if norm(normal) <= tol*tol { normal = cross(sub(p1,p0),sub(q1,p0)); }
            let normal = crate::space::normalised(normal)?;
            if dot(sub(q1,p0),normal).abs() > 1e3*tol || dot(sub(q0,p0),normal).abs() > 1e3*tol { return None }
            let normal = if dot(normal,outward) < 0. { scale(normal,-1.) } else { normal };
            Some((Surface::Plane(Frame::new(p0,normal,sub(p1,p0))),false))
        }
        (Curve::Circle(fa,ra),Curve::Circle(fb,rb)) => {
            // coaxial along the guide, and joined at the same angles
            let h = dot(sub(fb.o,fa.o),e);
            let off = norm(sub(sub(fb.o,fa.o),scale(e,h)));
            let radial = |p: V,c: V| { let d = sub(p,c); unit(sub(d,scale(e,dot(d,e)))) };
            let aligned = |sa: V,sz: V| norm(cross(radial(sa,fa.o),radial(sz,fb.o))) <= 1e-9
                && dot(radial(sa,fa.o),radial(sz,fb.o)) > 0.;
            let mid = (a.t[0]+a.t[1])/2.;
            if norm(cross(fa.z,e)) > 1e-9 || norm(cross(fb.z,e)) > 1e-9 || off > 1e3*tol || h <= 0.
                || !aligned(a.start(),z.start()) || !aligned(a.end(),z.end())
                || !aligned(a.curve.point(mid),z.curve.point((z.t[0]+z.t[1])/2.)) { return None }
            let f = Frame::new(fa.o,e,radial(a.start(),fa.o));
            Some((if (ra-rb).abs() <= tol { Surface::Cylinder(f,*ra) } else { Surface::Cone(f,*ra,((rb-ra)/h).datan()) },true))
        }
        _ => None,
    }
}

/// A sheet of several faces (#212): B-spline surfaces one after another along `u`, each face's
/// last iso-`u` curve the next one's first (each a segment of one net, `Net::segment`, at a shared
/// `v` domain), each named. The faces share those edges and their ends — the first's curve standing
/// for both — so a split by it never meets their seams; each face's sense is its net's own.
pub fn sheets(nets: Vec<(crate::brep::nurbs::Net,String)>) -> Result<Brep,String> {
    let Some((first,_)) = nets.first() else { return Err("a sheet of no faces".into()) };
    let [_,[v0,v1]] = first.domain();
    let row = |net: &crate::brep::nurbs::Net,i: usize| -> Curve { Curve::BSpline(Arc::new(BSpline {degree:net.dv,knots:net.vknots.clone(),
        poles:net.poles[i].clone(),weights:net.weights.as_ref().map(|w| w[i].clone())})) };
    let column = |net: &crate::brep::nurbs::Net,j: usize| -> Curve { Curve::BSpline(Arc::new(BSpline {degree:net.du,knots:net.uknots.clone(),
        poles:net.poles.iter().map(|r| r[j]).collect(),weights:net.weights.as_ref().map(|w| w.iter().map(|r| r[j]).collect())})) };
    let mut b = Brep::default();
    let [u0,_] = first.domain()[0];
    // the first face's first iso-u edge and its ends
    let mut low = [b.vertex(first.point(u0,v0)),b.vertex(first.point(u0,v1))];
    let mut iso = b.edge(EdgeCurve::Curve(row(first,0)),[v0,v1],low);
    let line = |a: [f64;2],z: [f64;2]| Pcurve::Line {a,b:z};
    for (net,name) in nets {
        let [[u0,u1],[w0,w1]] = net.domain();
        if (w0-v0).abs() > 1e-12*(1.+v0.abs()) || (w1-v1).abs() > 1e-12*(1.+v1.abs()) {
            return Err("a sheet's faces must share their v domain".into())
        }
        let (last_u,last_v) = (net.poles.len()-1,net.poles[0].len()-1);
        let high = [b.vertex(net.point(u1,v0)),b.vertex(net.point(u1,v1))];
        let e0 = b.edge(EdgeCurve::Curve(column(&net,0)),[u0,u1],[low[0],high[0]]);
        let e1 = b.edge(EdgeCurve::Curve(row(&net,last_u)),[v0,v1],high);
        let e2 = b.edge(EdgeCurve::Curve(column(&net,last_v)),[u0,u1],[low[1],high[1]]);
        let lp = vec![
            Coedge {edge:e0,reversed:false,pcurve:line([u0,v0],[u1,v0])},
            Coedge {edge:e1,reversed:false,pcurve:line([u1,v0],[u1,v1])},
            Coedge {edge:e2,reversed:true,pcurve:line([u0,v1],[u1,v1])},
            Coedge {edge:iso,reversed:true,pcurve:line([u0,v0],[u0,v1])},
        ];
        b.faces.push(Face {surface:Surface::BSpline(Frame::new([0.;3],[0.,0.,1.],[1.,0.,0.]),Arc::new(net)),reversed:false,loops:vec![lp],name});
        (low,iso) = (high,e1);
    }
    Ok(b)
}

/// A sheet: the whole of a B-spline surface as one face, bounded by its four boundary curves
/// (each its net's outermost row or column of poles, exactly), its sense the net's own.
pub fn sheet(net: crate::brep::nurbs::Net) -> Result<Brep,String> {
    let [[u0,u1],[v0,v1]] = net.domain();
    // a rational net's boundary rows and columns weighed as the net weighs them
    let row = |i: usize| -> Curve { Curve::BSpline(Arc::new(BSpline {degree:net.dv,knots:net.vknots.clone(),poles:net.poles[i].clone(),
        weights:net.weights.as_ref().map(|w| w[i].clone())})) };
    let column = |j: usize| -> Curve { Curve::BSpline(Arc::new(BSpline {degree:net.du,knots:net.uknots.clone(),poles:net.poles.iter().map(|r| r[j]).collect(),
        weights:net.weights.as_ref().map(|w| w.iter().map(|r| r[j]).collect())})) };
    let (last_u,last_v) = (net.poles.len()-1,net.poles[0].len()-1);
    let mut b = Brep::default();
    let corner = [net.point(u0,v0),net.point(u1,v0),net.point(u1,v1),net.point(u0,v1)];
    let vs: Vec<u32> = corner.iter().map(|&p| b.vertex(p)).collect();
    // the loop u0v0 → u1v0 → u1v1 → u0v1 → back, counter-clockwise in (u, v)
    let e0 = b.edge(EdgeCurve::Curve(column(0)),[u0,u1],[vs[0],vs[1]]);
    let e1 = b.edge(EdgeCurve::Curve(row(last_u)),[v0,v1],[vs[1],vs[2]]);
    let e2 = b.edge(EdgeCurve::Curve(column(last_v)),[u0,u1],[vs[3],vs[2]]);
    let e3 = b.edge(EdgeCurve::Curve(row(0)),[v0,v1],[vs[0],vs[3]]);
    let line = |a: [f64;2],z: [f64;2]| Pcurve::Line {a,b:z};
    let lp = vec![
        Coedge {edge:e0,reversed:false,pcurve:line([u0,v0],[u1,v0])},
        Coedge {edge:e1,reversed:false,pcurve:line([u1,v0],[u1,v1])},
        Coedge {edge:e2,reversed:true,pcurve:line([u0,v1],[u1,v1])},
        Coedge {edge:e3,reversed:true,pcurve:line([u0,v0],[u0,v1])},
    ];
    b.faces.push(Face {surface:Surface::BSpline(Frame::new([0.;3],[0.,0.,1.],[1.,0.,0.]),Arc::new(net)),reversed:false,loops:vec![lp],name:String::new()});
    Ok(b)
}
