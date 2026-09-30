//! The primitives, built with their topology whole: a profile swept along its normal (a prism)
//! or turned about a line in its plane (a revolution), from the loops of lines, arcs and circles
//! the CAD recipe carries (`solid::cad::recipe`).
use super::geom::{Curve,Frame,Rigid,Surface,V};
use super::topo::{Brep,Coedge,EdgeCurve,Face,Pcurve};
use crate::json::Json;
use crate::space::{add,cross,distance,dot,norm,scale,sub};

const TAU: f64 = std::f64::consts::TAU;

fn unit(a: V) -> V { scale(a,1./norm(a)) }
fn vec3(j: &Json) -> V { let a = j.arr(); [a[0].as_f64(),a[1].as_f64(),a[2].as_f64()] }

/// One edge of a profile: a line, or an arc (a whole circle where `span` is none) turning
/// counter-clockwise about its frame's `z`, which is the profile's normal.
#[derive(Clone,Copy,Debug)]
pub enum ProfileEdge { Line { a: V,b: V },Arc { frame: Frame,r: f64,span: Option<[f64;2]> } }

/// A planar region: an outer loop and its holes, each a closed walk of edges (listed in any
/// order and direction).
#[derive(Clone,Debug)]
pub struct Profile { pub origin: V,pub normal: V,pub loops: Vec<Vec<ProfileEdge>> }

impl Profile {
    /// From a recipe's `profile` (millimetres).
    pub fn from_json(j: &Json) -> Result<Profile,String> {
        let field = |j: &Json,k: &str| j.get(k).cloned().ok_or(format!("profile: no `{k}`"));
        let mut loops = Vec::new();
        for l in field(j,"loops")?.arr() {
            let mut edges = Vec::new();
            for e in l.arr() {
                edges.push(match field(e,"kind")?.as_str() {
                    "line" => ProfileEdge::Line {a:vec3(&field(e,"start")?),b:vec3(&field(e,"end")?)},
                    "circle" => {
                        let frame = Frame::new(vec3(&field(e,"center")?),vec3(&field(e,"normal")?),vec3(&field(e,"x_dir")?));
                        let span = e.get("angles").map(|a| {
                            let (a0,a1) = (a.arr()[0].as_f64(),a.arr()[1].as_f64());
                            let sweep = (a1-a0).rem_euclid(TAU);
                            [a0,a0+if sweep == 0. { TAU } else { sweep }]
                        });
                        ProfileEdge::Arc {frame,r:field(e,"radius")?.as_f64(),span}
                    }
                    k => return Err(format!("profile: an edge of kind `{k}`")),
                });
            }
            loops.push(edges);
        }
        Ok(Profile {origin:vec3(&field(j,"origin")?),normal:unit(vec3(&field(j,"normal")?)),loops})
    }

    /// A length the profile's tolerances are taken against.
    fn size(&self) -> f64 {
        let mut s: f64 = 0.;
        for l in &self.loops { for e in l { match *e {
            ProfileEdge::Line {a,b} => s = s.max(norm(sub(a,self.origin))).max(norm(sub(b,self.origin))),
            ProfileEdge::Arc {frame,r,..} => s = s.max(norm(sub(frame.o,self.origin))+r),
        } } }
        s.max(1e-300)
    }
}

/// One step of a walk: a curve from `t[0]` to `t[1]` (either way round).
#[derive(Clone,Debug)]
pub struct Seg { pub curve: Curve,pub t: [f64;2] }

impl Seg {
    pub fn start(&self) -> V { self.curve.point(self.t[0]) }
    pub fn end(&self) -> V { self.curve.point(self.t[1]) }
    pub fn forward(&self) -> bool { self.t[1] > self.t[0] }
    fn reversed(&self) -> Seg { Seg {curve:self.curve.clone(),t:[self.t[1],self.t[0]]} }
    /// The edge's parameters, increasing.
    pub fn span(&self) -> [f64;2] { if self.forward() { self.t } else { [self.t[1],self.t[0]] } }
    fn samples(&self,n: usize) -> impl Iterator<Item = V> + '_ {
        (0..n).map(move |j| self.curve.point(self.t[0]+(self.t[1]-self.t[0])*j as f64/n as f64))
    }
}

/// Each loop of a profile as a closed walk, the outer one counter-clockwise in `(x, y)` and the
/// holes clockwise, where `coords` reads a point's coordinates in the plane.
pub fn walks(p: &Profile,coords: &dyn Fn(V) -> [f64;2]) -> Result<Vec<Vec<Seg>>,String> {
    let tol = 1e-9*p.size();
    let mut out = Vec::new();
    for (li,l) in p.loops.iter().enumerate() {
        let segs: Vec<Seg> = l.iter().map(|e| match *e {
            ProfileEdge::Line {a,b} => {
                let d = sub(b,a);
                Seg {curve:Curve::Line {p:a,d:unit(d)},t:[0.,norm(d)]}
            }
            ProfileEdge::Arc {frame,r,span} => Seg {curve:Curve::Circle(frame,r),t:span.unwrap_or([0.,TAU])},
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
    let mut b = Brep::default();
    let lift = |q: V,h: f64| add(q,scale(n,h));
    let (mut bottom_loops,mut top_loops) = (Vec::new(),Vec::new());
    for walk in &walks {
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
                Curve::Ellipse(..) | Curve::Traced(..) => unreachable!("a profile has only lines and circles"),
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
            b.faces.push(Face {surface,reversed,loops:vec![side],name:String::new()});
        }
        bottom_loops.push((0..m).rev().map(|k| use_of(bottom[k],&walk[k],false,on_plane())).collect::<Vec<_>>());
        top_loops.push((0..m).map(|k| use_of(top[k],&walk[k],true,on_plane())).collect::<Vec<_>>());
    }
    let cap = |h: f64| Surface::Plane(Frame {o:lift(plane.o,h),..plane});
    b.faces.push(Face {surface:cap(h0),reversed:true,loops:bottom_loops,name:"near".into()});
    b.faces.push(Face {surface:cap(h1),reversed:false,loops:top_loops,name:"far".into()});
    Ok(b)
}

/// What a profile step sweeps as it turns about the axis.
#[derive(Clone,Copy,Debug,PartialEq)]
enum Swept { Axis,Disk,Cylinder,Cone,Sphere,Torus }

/// The profile turned about the line through `origin` along `axis` by `angle` (right-handed; a
/// negative angle turns the other way, and anything past a turn is a whole one).
pub fn revolve(p: &Profile,origin: V,axis: V,angle: f64) -> Result<Brep,String> {
    let z = scale(unit(axis),angle.signum());
    let theta = angle.abs().min(TAU);
    if !(theta > 0.) { return Err("a revolution through no angle".into()) }
    let full = theta >= TAU-1e-12;
    let tol = 1e-9*p.size();
    let normal = unit(p.normal);
    if dot(normal,z).abs() > 1e-9 || dot(sub(origin,p.origin),normal).abs() > tol {
        return Err("a revolution's axis must lie in its profile's plane".into())
    }
    // x: square to the axis in the profile's plane, toward the profile
    let mut x = unit(cross(z,normal));
    let far = p.loops.iter().flatten().flat_map(|e| match *e {
        ProfileEdge::Line {a,b} => vec![a,b],
        ProfileEdge::Arc {frame,r,..} => vec![add(frame.o,scale(frame.x,r)),sub(frame.o,scale(frame.x,r))],
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
    for walk in &walks {
        let m = walk.len();
        let pts: Vec<V> = walk.iter().map(|s| s.start()).collect();
        let on_axis: Vec<bool> = pts.iter().map(|&q| rz(q)[0] <= tol).collect();
        let v0: Vec<u32> = pts.iter().map(|&q| b.vertex(q)).collect();
        let v1: Vec<u32> = (0..m).map(|k| if full || on_axis[k] { v0[k] } else { b.vertex(turn.point(pts[k])) }).collect();
        let kinds: Vec<Swept> = walk.iter().enumerate().map(|(k,s)| {
            let ([r0,z0],[r1,z1]) = (rz(s.start()),rz(s.end()));
            match &s.curve {
                Curve::Line {..} if on_axis[k] && on_axis[(k+1)%m] => Swept::Axis,
                Curve::Line {..} if (z0-z1).abs() <= tol => Swept::Disk,
                Curve::Line {..} if (r0-r1).abs() <= tol => Swept::Cylinder,
                Curve::Line {..} => Swept::Cone,
                Curve::Circle(c,_) if rz(c.o)[0].abs() <= tol => Swept::Sphere,
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
            let ([r0,z0],[r1,z1]) = (rz(s.start()),rz(s.end()));
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
                    let a = ((r1-r0)/(z1-z0)).atan();
                    (Surface::Cone(Frame {o:f.at([0.,0.,z0]),..f},r0,a),[a.cos(),-a.sin()],
                        { let c = s.curve.clone(); Box::new(move |t| (rz(c.point(t))[1]-z0)/a.cos()) })
                }
                Swept::Sphere | Swept::Torus => {
                    let Curve::Circle(c,r) = s.curve.clone() else { unreachable!() };
                    let [rc,zc] = rz(c.o);
                    let big = if kinds[k] == Swept::Sphere { 0. } else { rc };
                    let surface = if kinds[k] == Swept::Sphere { Surface::Sphere(Frame {o:f.at([0.,0.,zc]),..f},r) }
                        else { Surface::Torus(Frame {o:f.at([0.,0.,zc]),..f},big,r) };
                    // v at the step's start, and which way it runs with the circle's parameter
                    let [ra,za] = rz(s.start());
                    let va = (za-zc).atan2(ra-big);
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
                    b.faces.push(Face {surface,reversed,loops,name:String::new()});
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
            let ccw = b.loop_area(&Face {surface,reversed,loops:vec![],name:String::new()},&l) > 0.;
            if ccw == reversed { l = l.into_iter().rev().map(|c| Coedge {reversed:!c.reversed,..c}).collect(); }
            b.faces.push(Face {surface,reversed,loops:vec![l],name:String::new()});
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
