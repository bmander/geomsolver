//! **Offsets** (docs/contract/brep.md, "Offsets"): a solid grown or shrunk by a distance, a solid
//! hollowed to a wall, a sheet thickened — each the union of the pieces its points are nearest.
//!
//! A point outside a solid within `d` of it has a nearest point on the boundary, inside a face, on
//! an edge or at a vertex, and stands off it along the face's outward normal, in the wedge of the
//! normals of an edge's two faces, or in the cone of a vertex's.  So the solid grown by `d` is the
//! solid with, for each face, its **normal sweep** (`normal_sweep`: every `q + s n(q)`, `q` in the
//! face, `s` over `[0, d]`), for each convex edge the **tube** of radius `d` about it, and for each
//! corner the **ball** of radius `d` about it: every piece lies within `d` of the solid, and every
//! point within `d` lies in a piece.  Shrunk is the dual: the solid less the inward sweeps, the
//! tubes about its concave edges and the balls at its concave corners.  Nothing is classified by
//! shape, and where the grown faces meet one another — a groove grown shut, a torus grown past its
//! hole — the union decides, as the definition does.
//!
//! **Sharp joins** (`Join::Sharp`) are the other set an offset is asked for: each face moved along
//! its normal and the moved faces extended until they meet.  Two planes meeting along a line are
//! joined by the prism of the kite between the edge, its two moved copies and the line the moved
//! planes meet in; a corner of three planes by the parallelepiped the moved planes and the faces'
//! own bound.  Sharp joins of curved faces are refused, naming the edge.
//!
//! Every piece is built exactly by the kernel's own sweeps (`build::prism`, `build::revolve`,
//! `build::sphere`), and combined by its Booleans.  A face whose normal sweep has no exact
//! construction here yet — a curved face not bounded by its own parameter lines, a surface with no
//! offset in closed form — is refused, naming it and a point on it.
use super::boolean::{combined,combined_known,Known,Op};
use super::build::{kept,prism,revolve,sphere,square,disc,Profile,ProfileEdge};
use super::geom::{tolerance,Curve,Frame,Rigid,Surface,Uv,V};
use super::topo::{Brep,EdgeCurve,Face};
use crate::util::space::{add,cross,distance,dot,lerp,norm,scale,sub,unit};
use std::collections::{BTreeMap,BTreeSet};
use std::f64::consts::{FRAC_PI_2,PI,TAU};

/// How the moved faces of an offset are joined where they part: by the round of the distance set
/// (a tube about an edge, a ball about a corner), or by extending them until they meet.
#[derive(Clone,Copy,Debug,PartialEq,Eq)]
pub enum Join { Round,Sharp }

/// Which way an offset moves the faces: along their outward normals or against them.
#[derive(Clone,Copy,Debug,PartialEq,Eq)]
pub enum Side { Out,In }

impl Side {
    fn sign(self) -> f64 { match self { Side::Out => 1.,Side::In => -1. } }
}

/// A piece of an offset, and what it is called where a Boolean refuses it.
type Piece = (String,Brep);

/// The solid `b` grown by `d` (> 0): every point within `d` of it (`Join::Round`), or with its
/// faces moved out by `d` and extended until they meet (`Join::Sharp`).
pub fn grown(b: &Brep,d: f64,join: Join) -> Result<Brep,String> {
    positive(d,"a growth")?;
    let all = vec![true;b.faces.len()];
    folded(b.clone(),pieces(b,&all,Side::Out,d,join)?,Op::Union)
}

/// The solid `b` shrunk by `d` (> 0): every point of it at least `d` inside it (`Join::Round`),
/// or with its faces moved in by `d` and extended until they meet (`Join::Sharp`).
pub fn shrunk(b: &Brep,d: f64,join: Join) -> Result<Brep,String> {
    positive(d,"a shrinking")?;
    let all = vec![true;b.faces.len()];
    let out = folded(b.clone(),pieces(b,&all,Side::In,d,join)?,Op::Cut)?;
    if out.faces.is_empty() { return Err(format!("the solid shrunk by {d} is nothing: no point of it is that far inside")) }
    Ok(out)
}

/// The solid `b` hollowed to a wall `t` (> 0) thick within it (`Side::In`) or outside it
/// (`Side::Out`), open at the faces `open`: the wall every point of the solid (or outside it)
/// within `t` of its other faces, joined as `join` says.  A face left open is no part of the wall,
/// so the cavity runs out through it (inside), or the wall ends where it is (outside).
pub fn shelled(b: &Brep,open: &[usize],t: f64,side: Side,join: Join) -> Result<Brep,String> {
    positive(t,"a wall")?;
    if let Some(&f) = open.iter().find(|&&f| f >= b.faces.len()) { return Err(format!("no face {f} to leave open")) }
    let walled: Vec<bool> = (0..b.faces.len()).map(|f| !open.contains(&f)).collect();
    if !walled.contains(&true) { return Err("a shell open at every face has no wall".into()) }
    let mut pieces = pieces(b,&walled,side,t,join)?.into_iter();
    let (_,first) = pieces.next().ok_or("a shell with no wall")?;
    let wall = folded(first,pieces.collect(),Op::Union)?;
    match side {
        Side::In => combined(b,&wall,Op::Common,0.).map_err(|e| format!("the wall kept within the solid: {e}")),
        Side::Out => {
            // outside, the wall ends at each opening's plane: what lies beyond it is cut off
            let mut wall = combined(&wall,b,Op::Cut,0.).map_err(|e| format!("the solid cut from its wall: {e}"))?;
            for &o in open {
                let Surface::Plane(fr) = b.faces[o].surface else {
                    return Err(format!("{} is left open on a wall outside the solid: only a plane cuts one off yet",face_name(b,o)))
                };
                let n = if b.faces[o].reversed { fr.z } else { scale(fr.z,-1.) };
                wall = kept(&wall,fr.o,n,0.).map_err(|e| format!("the wall cut off at {}: {e}",face_name(b,o)))?;
            }
            Ok(wall)
        }
    }
}

/// A sheet `sheet` (faces bounding no solid, each oriented, neighbours turning one way) thickened
/// over `s` along its faces' normals: every `q + s n(q)`, `s` from `s[0]` to `s[1]` (`s[0] < s[1]`,
/// either side of the sheet), its creases joined as `join` says on the side where its moved faces
/// part, and its rims the walls its boundary sweeps.
pub fn thickened(sheet: &Brep,s: [f64;2],join: Join) -> Result<Brep,String> {
    if !(s[0] < s[1] && s.iter().all(|x| x.is_finite())) { return Err(format!("a sheet thickened over {s:?}, no depth")) }
    let all = vec![true;sheet.faces.len()];
    let uses = uses(sheet);
    let mut pieces = (0..sheet.faces.len()).map(|fi| normal_sweep(sheet,fi,s).map(|p| (face_name(sheet,fi),p)))
        .collect::<Result<Vec<Piece>,String>>()?;
    for (side,depth) in [(Side::Out,s[1]),(Side::In,-s[0])] {
        if depth > 0. { pieces.extend(joins(sheet,&uses,&all,side,depth,join,Of::Sheet)?); }
    }
    let mut pieces = pieces.into_iter();
    let (_,first) = pieces.next().ok_or("a sheet with no face")?;
    folded(first,pieces.collect(),Op::Union)
}

/// Refused unless `d` is a positive length.
fn positive(d: f64,what: &str) -> Result<(),String> {
    if d > 0. && d.is_finite() { Ok(()) } else { Err(format!("{what} by {d}: the distance must be positive")) }
}

/// `start` combined with each piece in turn by `op`, what each step knows of its faces handed to
/// the next (#589), so a face far from every piece is boxed and checked once.
fn folded(start: Brep,pieces: Vec<Piece>,op: Op) -> Result<Brep,String> {
    let (mut out,mut known): (Brep,Option<Known>) = (start,None);
    for (what,p) in pieces {
        let (o,k) = combined_known(&out,known.as_ref(),&p,op,0.).map_err(|e| format!("{what} {}: {e}",match op {
            Op::Union => "united",Op::Cut => "cut",Op::Common => "kept in common" }))?;
        (out,known) = (o,Some(k));
    }
    Ok(out)
}

/// A face's name for a refusal: its own, or its index and surface.
fn face_name(b: &Brep,fi: usize) -> String {
    let f = &b.faces[fi];
    if f.name.is_empty() { format!("face {fi} ({})",f.surface.kind()) } else { format!("`{}` ({})",f.name,f.surface.kind()) }
}

/// The pieces of the faces `faces` marks of `b` moved `d` to `side`: each face's normal sweep, and
/// the joins where they part.
fn pieces(b: &Brep,faces: &[bool],side: Side,d: f64,join: Join) -> Result<Vec<Piece>,String> {
    let s = match side { Side::Out => [0.,d],Side::In => [-d,0.] };
    let uses = uses(b);
    let mut out = Vec::new();
    for fi in (0..b.faces.len()).filter(|&f| faces[f]) {
        // inside, a wall's moved face runs on across a face left open until it meets it
        let across = match side { Side::In => open_sides(b,&uses,fi,faces),Side::Out => [false;4] };
        if let Some(p) = sweep(b,fi,s,Reach {across,cut_at_axis:true})? { out.push((format!("the sweep of {}",face_name(b,fi)),p)); }
    }
    out.extend(joins(b,&uses,faces,side,d,join,Of::Solid)?);
    Ok(out)
}

/// Face `fi`'s outward normal at `p` on it, or a refusal naming it.
fn outward(b: &Brep,fi: usize,p: V) -> Result<V,String> {
    b.faces[fi].outward(p).ok_or_else(|| format!("{} has no normal at {p:?}",face_name(b,fi)))
}

/// The outward normals of the faces `faces` at `p`, turned to `side`: three only.
fn normals_to(b: &Brep,faces: &[usize],p: V,side: Side,what: &str) -> Result<[V;3],String> {
    let [a,c,e] = faces[..] else { return Err(format!("no {what} of {} faces yet: one of three is",faces.len())) };
    Ok([outward(b,a,p)?,outward(b,c,p)?,outward(b,e,p)?].map(|n| scale(n,side.sign())))
}

/// An edge between two faces, read at a point of it: its faces, their outward normals there, and
/// the edge's tangent as the first face's loop walks it.
struct Crease { faces: [usize;2],normals: [V;2],tangent: V }

impl Crease {
    /// Whether the faces meet smoothly there, their normals one.
    fn smooth(&self) -> bool { norm(cross(self.normals[0],self.normals[1])) <= 1e-9 && dot(self.normals[0],self.normals[1]) > 0. }
    /// Whether the faces part to `side` there: the second face turns away from the first's normal
    /// (a convex edge) for `Side::Out`, toward it (a concave edge) for `Side::In`.
    fn parts(&self,side: Side) -> bool {
        // the direction into the second face from the edge, the material on the left of its walk
        let into = cross(self.normals[1],scale(self.tangent,-1.));
        let concave = dot(into,self.normals[0]) > 0.;
        match side { Side::Out => !concave,Side::In => concave }
    }
}

/// Each edge's uses: the faces whose loops walk it, and whether each walks it against its curve.
fn uses(b: &Brep) -> Vec<Vec<(usize,bool)>> {
    let mut uses = vec![Vec::new();b.edges.len()];
    for (fi,f) in b.faces.iter().enumerate() { for c in f.loops.iter().flatten() { uses[c.edge as usize].push((fi,c.reversed)); } }
    uses
}

/// What the faces offset are: a solid's (every edge of one of them is the solid's, the points
/// nearest an edge or a corner are joined whichever faces it bounds, and a corner of no crease —
/// a cone's apex — is a ball), or a sheet's (only the creases between two of its faces are
/// joined, its rims swept straight along the normals).
#[derive(Clone,Copy,Debug,PartialEq,Eq)]
enum Of { Solid,Sheet }

/// The joins of the faces `faces` marks of `b` moved `d` to `side`, where they part: along each
/// crease that turns away to that side, a tube (`Join::Round`) or a kite's prism (`Join::Sharp`);
/// at each corner the points nearest which are nearest it, a ball or the corner's parallelepiped.
fn joins(b: &Brep,uses: &[Vec<(usize,bool)>],faces: &[bool],side: Side,d: f64,join: Join,of: Of) -> Result<Vec<Piece>,String> {
    let mut out = Vec::new();
    // each vertex's creases: the tangent each leaves it by, whether it is joined, its faces
    let mut at: BTreeMap<u32,Vec<(V,bool,[usize;2])>> = BTreeMap::new();
    for (ei,e) in b.edges.iter().enumerate() {
        let [(fa,ra),(fb,_)] = uses[ei][..] else { continue };
        let both = faces[fa] && faces[fb];
        let wanted = match of { Of::Sheet => both,Of::Solid => faces[fa] || faces[fb] };
        if fa == fb || !wanted { continue }
        let EdgeCurve::Curve(c) = &e.curve else { continue };
        let mid = 0.5*(e.t[0]+e.t[1]);
        let p = c.point(mid);
        let t = unit(c.tangent(mid));
        let crease = Crease {faces:[fa,fb],normals:[outward(b,fa,p)?,outward(b,fb,p)?],tangent:if ra { scale(t,-1.) } else { t }};
        if crease.smooth() { continue }
        let parts = crease.parts(side);
        for (k,&v) in e.v.iter().enumerate() {
            let tk = unit(c.tangent(e.t[k]));
            at.entry(v).or_default().push((if k == 0 { tk } else { scale(tk,-1.) },parts,[fa,fb]));
        }
        if !parts { continue }
        let what = format!("the join of {} and {}",face_name(b,fa),face_name(b,fb));
        let piece = match join {
            // a solid's points within `d` of the edge are all of the tube; a sheet's, only the
            // wedge between its two moved faces (the rest lies across the sheet or within a face's sweep)
            Join::Round if of == Of::Solid => tube(c,e.t,d),
            Join::Round => wedge(c,e.t,d,crease.normals.map(|n| scale(n,side.sign()))),
            // (a sharp join along a face left open is the opening's to cut off)
            Join::Sharp if !both => continue,
            Join::Sharp => kite(b,&crease,c,e.t,side,d),
        }.map_err(|m| format!("{what}: {m}"))?;
        out.push((what,piece));
    }
    // the corners
    let mut face_at: BTreeMap<u32,BTreeSet<usize>> = BTreeMap::new();
    for (fi,f) in b.faces.iter().enumerate() {
        for c in f.loops.iter().flatten() { for v in b.edges[c.edge as usize].v { face_at.entry(v).or_default().insert(fi); } }
    }
    for (vi,vertex) in b.vertices.iter().enumerate() {
        let v = vi as u32;
        let Some(fs) = face_at.get(&v) else { continue };
        if !fs.iter().any(|&f| faces[f]) { continue }
        let creases = at.get(&v).map(Vec::as_slice).unwrap_or(&[]);
        // a crease turning the other way: the points near the corner are nearer it or its faces
        if creases.iter().any(|c| !c.1) { continue }
        // two creases running on one into the other: their tubes' ends meet there, no corner
        let straight = creases.len() == 2 && dot(creases[0].0,creases[1].0) < -1.+1e-9;
        let needed = match of {
            Of::Sheet => creases.len() >= 2 && !straight,
            Of::Solid => if creases.is_empty() { !smooth_at(b,fs,v) } else { !straight },
        };
        if !needed { continue }
        let p = vertex.p;
        let what = format!("the corner at {p:?}");
        let joined: Vec<usize> = creases.iter().flat_map(|c| c.2).collect::<BTreeSet<usize>>().into_iter().collect();
        let piece = match join {
            Join::Round if of == Of::Solid => {
                // its poles along the corner's mean normal, in the middle of what the ball keeps
                // and off the circles where the tubes end on it: a meeting through a sphere's pole
                // is read less exactly than one clear of it (a known limit of the Booleans)
                let mean = fs.iter().filter_map(|&f| b.faces[f].outward(p)).fold([0.;3],add);
                sphere(p,if norm(mean) > 1e-6 { scale(mean,side.sign()) } else { [0.36,0.48,0.8] },d)
            }
            Join::Round => ball_sector(b,&joined,p,side,d),
            Join::Sharp if !joined.iter().all(|&f| faces[f]) => continue,
            Join::Sharp => corner(b,&joined,p,side,d),
        }.map_err(|m| format!("{what}: {m}"))?;
        out.push((what,piece));
    }
    Ok(out)
}

/// Each use of an edge on face `f`, sampled in the face's parameters at nine points along it, and
/// the box of every sample.
fn samples(b: &Brep,f: &Face) -> (Vec<(u32,Vec<Uv>)>,[Uv;2]) {
    let (mut lo,mut hi) = ([f64::INFINITY;2],[f64::NEG_INFINITY;2]);
    let runs = f.loops.iter().flatten().map(|c| {
        let e = &b.edges[c.edge as usize];
        let run: Vec<Uv> = (0..=8).map(|k| c.pcurve.at(e.t[0]+(e.t[1]-e.t[0])*k as f64/8.,e,&f.surface,&b.vertices)).collect();
        for uv in &run { for j in 0..2 { lo[j] = lo[j].min(uv[j]); hi[j] = hi[j].max(uv[j]); } }
        (c.edge,run)
    }).collect();
    (runs,[lo,hi])
}

/// Whether the faces `fs` at vertex `v` have one normal there: read just inside each face from
/// every use of an edge ending at `v` (along a pole's edge too), so a sphere's pole is smooth and a
/// cone's apex is not.
fn smooth_at(b: &Brep,fs: &BTreeSet<usize>,v: u32) -> bool {
    let mut normals: Vec<V> = Vec::new();
    for &fi in fs {
        let f = &b.faces[fi];
        let (runs,[lo,hi]) = samples(b,f);
        let centre = [(lo[0]+hi[0])/2.,(lo[1]+hi[1])/2.];
        for (edge,run) in runs {
            let e = &b.edges[edge as usize];
            let near = run.iter().enumerate().filter(|&(k,_)| matches!(e.curve,EdgeCurve::Degenerate) && e.v[0] == v
                || k == 0 && e.v[0] == v || k == 8 && e.v[1] == v);
            for (_,uv) in near {
                let q = [uv[0]+1e-7*(centre[0]-uv[0]),uv[1]+1e-7*(centre[1]-uv[1])];
                let Some(n) = f.surface.normal(q) else { return false };
                normals.push(if f.reversed { scale(n,-1.) } else { n });
            }
        }
    }
    normals.iter().all(|&n| dot(n,normals[0]) > 1.-1e-8)
}

/// A profile of `loops` about `origin` square to `normal`, its edges unnamed.
fn profile(origin: V,normal: V,loops: Vec<Vec<ProfileEdge>>) -> Profile { Profile {origin,normal,loops,names:Vec::new()} }

/// The closed polygon through `c` as a loop of lines, its sides shorter than `tol` dropped.
fn polygon(c: &[V],tol: f64) -> Vec<ProfileEdge> {
    let n = c.len();
    (0..n).filter(|&k| distance(c[k],c[(k+1)%n]) > tol).map(|k| ProfileEdge::Line {a:c[k],b:c[(k+1)%n]}).collect()
}

/// The tube of radius `d` about the stretch `t` of the curve `c`, its ends flat (square to the
/// curve): a cylinder about a line, a torus's stretch about a circle (cut at the axis where it is
/// wider than the circle).
fn tube(c: &Curve,t: [f64;2],d: f64) -> Result<Brep,String> {
    match *c {
        Curve::Line {..} => {
            let (a,z) = (c.point(t[0]),c.point(t[1]));
            prism(&disc(a,unit(sub(z,a)),d),0.,distance(a,z))
        }
        Curve::Circle(f,r) => {
            let e = add(scale(f.x,t[0].cos()),scale(f.y,t[0].sin()));
            let centre = add(f.o,scale(e,r));
            let n = cross(f.z,e);
            let plane = Frame::new(centre,n,e);
            // the disc clear of the axis: where it reaches past it, the arc on its side of it, shut
            // along the axis
            let c0 = (-r/d).clamp(-1.,1.).acos();
            let loops = if d < r-tolerance(r+d) || c0 >= PI-1e-12 {
                vec![vec![ProfileEdge::Arc {frame:plane,r:d,span:None}]]
            } else {
                let at = |a: f64| plane.at([d*a.cos(),d*a.sin(),0.]);
                vec![vec![ProfileEdge::Arc {frame:plane,r:d,span:Some([-c0,c0])},ProfileEdge::Line {a:at(c0),b:at(-c0)}]]
            };
            revolve(&profile(centre,n,loops),f.o,f.z,(t[1]-t[0]).min(TAU))
        }
        _ => Err(format!("no tube is built about a {} yet: lines and circles are",c.kind())),
    }
}

/// The wedge of the tube of radius `d` about the stretch `t` of the curve `c` between the
/// directions `n` (each square to the curve at its start), the round between two faces moved
/// along them: a sector's prism along a line, turned about a circle.
fn wedge(c: &Curve,t: [f64;2],d: f64,n: [V;2]) -> Result<Brep,String> {
    let a = c.point(t[0]);
    // the sector turns from `n[0]` to `n[1]` about the side `n[0] × n[1]` points along
    let axis = unit(cross(n[0],n[1]));
    let angle = dot(n[0],n[1]).clamp(-1.,1.).acos();
    if !(angle > 1e-9) { return Err("its faces' normals are one: nothing to join".into()) }
    let far = add(a,scale(n[1],d));
    let sector = profile(a,axis,vec![vec![ProfileEdge::Line {a,b:add(a,scale(n[0],d))},
        ProfileEdge::Arc {frame:Frame::new(a,axis,n[0]),r:d,span:Some([0.,angle])},ProfileEdge::Line {a:far,b:a}]]);
    match *c {
        Curve::Line {..} => {
            // swept along the sector's own normal (its arc's frame's), forward or back along the edge
            let len = distance(a,c.point(t[1]));
            let (from,to) = if dot(axis,c.tangent(t[0])) > 0. { (0.,len) } else { (-len,0.) };
            prism(&sector,from,to)
        }
        Curve::Circle(f,_) => revolve(&sector,f.o,f.z,(t[1]-t[0]).min(TAU)),
        _ => Err(format!("no round is built along a {} yet: lines and circles are",c.kind())),
    }
}

/// The sector of the ball of radius `d` about the corner `p` of the faces `faces` that their
/// normals (to `side`) span: the ball kept on the inner side of the plane through each two of
/// them.  Three faces only.
fn ball_sector(b: &Brep,faces: &[usize],p: V,side: Side,d: f64) -> Result<Brep,String> {
    let ns = normals_to(b,faces,p,side,"round corner")?;
    let mut piece = sphere(p,ns.iter().fold([0.;3],|a,&n| add(a,n)),d)?;
    for k in 0..3 {
        let m = unit(cross(ns[k],ns[(k+1)%3]));
        piece = kept(&piece,p,if dot(m,ns[(k+2)%3]) < 0. { scale(m,-1.) } else { m },0.)?;
    }
    Ok(piece)
}

/// The prism of the kite between a line edge of two planes, its two copies moved `d` along each
/// plane's normal to `side`, and the line the moved planes meet in.
fn kite(b: &Brep,crease: &Crease,c: &Curve,t: [f64;2],side: Side,d: f64) -> Result<Brep,String> {
    let Curve::Line {..} = c else { return Err(format!("no sharp join along a {} yet: a line between two planes is", c.kind())) };
    if !crease.faces.iter().all(|&f| matches!(b.faces[f].surface,Surface::Plane(_))) {
        return Err("no sharp join of curved faces yet: a line between two planes is".into())
    }
    let [n1,n2] = crease.normals.map(|n| scale(n,side.sign()));
    let cos = dot(n1,n2);
    if !(1.+cos > 1e-9) { return Err("the faces fold back on one another: their moved planes never meet".into()) }
    let (a,z) = (c.point(t[0]),c.point(t[1]));
    let kite = polygon(&[a,add(a,scale(n1,d)),add(a,scale(add(n1,n2),d/(1.+cos))),add(a,scale(n2,d))],0.);
    prism(&profile(a,unit(sub(z,a)),vec![kite]),0.,distance(a,z))
}

/// The corner of the planar faces `faces` at `p` moved `d` to `side` and extended until they meet:
/// the parallelepiped every point of which stands between `0` and `d` from each face's plane to
/// that side — the slab of the first plane cut by the other two's.  Three planes only.
fn corner(b: &Brep,faces: &[usize],p: V,side: Side,d: f64) -> Result<Brep,String> {
    if !faces.iter().all(|&f| matches!(b.faces[f].surface,Surface::Plane(_))) {
        return Err("no sharp corner of curved faces yet: one of three planes is".into())
    }
    let [n0,n1,n2] = normals_to(b,faces,p,side,"sharp corner")?;
    // the parallelepiped's far corner, where the three moved planes meet: its size
    let det = dot(n0,cross(n1,n2));
    if !(det.abs() > 1e-9) { return Err("its three planes meet in no point".into()) }
    let far = scale(add(add(cross(n1,n2),cross(n2,n0)),cross(n0,n1)),d/det);
    let reach = 2.*(norm(far)+d);
    let mut piece = prism(&square(p,n0,reach),0.,d)?;
    for n in [n1,n2] {
        piece = kept(&piece,p,n,0.)?;
        piece = kept(&piece,add(p,scale(n,d)),scale(n,-1.),0.)?;
    }
    Ok(piece)
}

/// **A face swept along its normal**: every `q + s n(q)` for `q` in face `fi` of `b`, `n` its
/// outward normal, `s` over `[s[0], s[1]]` — the solid between the face moved to `s[0]` and to
/// `s[1]` and the walls its boundary sweeps.  A plane's is the prism of its loops; a cylinder's,
/// a cone's, a sphere's or a torus's, bounded by its own parameter lines, the section between the
/// face's meridian and its moved copies turned through the face's span.  Refused where the sweep
/// passes the surface's centre.
pub fn normal_sweep(b: &Brep,fi: usize,s: [f64;2]) -> Result<Brep,String> {
    sweep(b,fi,s,Reach::default())?.ok_or_else(|| format!("{} is swept wholly past its axis",face_name(b,fi)))
}

/// How far a face's sweep reaches past the face itself: across each side of its box of
/// parameters (`[u0, u1, v0, v1]`) that borders a face left open, on to the end of its surface's
/// own range (a moved wall runs on until it meets the opening, which the solid then trims); and
/// whether the part past the surface's axis is cut away, as a distance set's piece may be (a point
/// past the axis is nearer the surface on the axis's far side, and not this face's).
#[derive(Clone,Copy,Debug,Default)]
struct Reach { across: [bool;4],cut_at_axis: bool }

/// The sweep `normal_sweep` builds, reaching as `reach` says; none where it is cut away wholly at
/// the axis.
fn sweep(b: &Brep,fi: usize,s: [f64;2],reach: Reach) -> Result<Option<Brep>,String> {
    let f = &b.faces[fi];
    let refuse = |why: String| -> String {
        let witness = f.loops.iter().flatten().next().map(|c| b.vertices[b.edges[c.edge as usize].v[0] as usize].p);
        format!("{} is not swept along its normal: {why} (at {witness:?})",face_name(b,fi))
    };
    match f.surface {
        Surface::Plane(fr) => plane_sweep(b,f,fr,s).map(Some).map_err(refuse),
        Surface::Cylinder(..) | Surface::Cone(..) | Surface::Sphere(..) | Surface::Torus(..) => revolved_sweep(b,f,s,reach).map_err(refuse),
        _ => Err(refuse(format!("a {} has no offset in closed form",f.surface.kind()))),
    }
}

/// A plane face's sweep: the prism of its loops along its outward normal.
fn plane_sweep(b: &Brep,f: &Face,fr: Frame,s: [f64;2]) -> Result<Brep,String> {
    let normal = scale(fr.z,if f.reversed { -1. } else { 1. });
    let mut loops = Vec::new();
    for l in &f.loops {
        let mut edges = Vec::new();
        for c in l {
            let e = &b.edges[c.edge as usize];
            let EdgeCurve::Curve(curve) = &e.curve else { continue };
            edges.push(match curve {
                Curve::Line {..} => ProfileEdge::Line {a:b.vertices[e.v[0] as usize].p,b:b.vertices[e.v[1] as usize].p},
                Curve::Circle(frame,r) => {
                    // the arc's frame turned to the profile's normal: about the other way, its
                    // parameter runs back
                    let (frame,span) = if dot(frame.z,normal) < 0. {
                        (Frame {y:scale(frame.y,-1.),z:scale(frame.z,-1.),..*frame},[-e.t[1],-e.t[0]])
                    } else { (*frame,e.t) };
                    ProfileEdge::Arc {frame,r:*r,span:(e.t[1]-e.t[0] < TAU-1e-12).then_some(span)}
                }
                Curve::BSpline(sp) => {
                    let [lo,hi] = sp.domain();
                    if (e.t[0]-lo).abs() > 1e-12 || (e.t[1]-hi).abs() > 1e-12 { return Err("an edge on a stretch of its spline".into()) }
                    ProfileEdge::Spline(sp.clone())
                }
                other => return Err(format!("an edge on a {}",other.kind())),
            });
        }
        loops.push(edges);
    }
    prism(&profile(fr.o,normal,loops),s[0],s[1])
}

/// A cylinder's, cone's, sphere's or torus's face's sweep: its meridian's section between its
/// moved copies, turned through its span about the axis.
fn revolved_sweep(b: &Brep,f: &Face,s: [f64;2],reach: Reach) -> Result<Option<Brep>,String> {
    let fr = *f.surface.frame();
    let out = if f.reversed { -1. } else { 1. };
    let ([mut u0,u1],[mut v0,mut v1]) = iso_box(b,f).ok_or("its boundary is not its own parameter lines")?;
    let mut span = (u1-u0).min(TAU);
    // across a side bordering an opening: on to the end of the surface's range
    let [ul,uh,vl,vh] = reach.across;
    if ul || uh { u0 = 0.; span = TAU; }
    let long = 4.*b.size();
    match f.surface {
        Surface::Sphere(..) => { if vl { v0 = -FRAC_PI_2; } if vh { v1 = FRAC_PI_2; } }
        Surface::Torus(..) => if vl || vh { v1 = v0+TAU; },
        Surface::Cone(_,r,a) if a.sin().abs() > 1e-12 => {
            // not past the apex
            let apex = -r/a.sin();
            if vl { v0 = if apex < v0 { apex.max(v0-long) } else { v0-long }; }
            if vh { v1 = if apex > v1 { apex.min(v1+long) } else { v1+long }; }
        }
        _ => { if vl { v0 -= long; } if vh { v1 += long; } }
    }
    let e = add(scale(fr.x,u0.cos()),scale(fr.y,u0.sin()));
    let plane_normal = cross(fr.z,e);
    let at = |v: f64| f.surface.point([u0,v]);
    let n_at = |v: f64| f.surface.normal([u0,v]).map(|n| scale(n,out)).ok_or("no normal");
    let tol = tolerance(b.size());
    let turned = |loops| revolve(&profile(fr.o,plane_normal,loops),fr.o,fr.z,span);
    let (centre,r) = match f.surface {
        Surface::Sphere(_,r) => (fr.o,r),
        Surface::Torus(_,big,r) => (add(fr.o,scale(e,big)),r),
        _ => {
            // a straight meridian: the section a quadrilateral
            let n = n_at(0.5*(v0+v1))?;
            let (p0,p1) = (at(v0),at(v1));
            let c = [add(p0,scale(n,s[0])),add(p1,scale(n,s[0])),add(p1,scale(n,s[1])),add(p0,scale(n,s[1]))];
            // the section's distance from the axis, in its half-plane
            let rho = |q: V| dot(sub(q,fr.o),e);
            if !c.iter().any(|&q| rho(q) < -tol) { return turned(vec![polygon(&c,tol)]).map(Some) }
            // past the axis the section stands on the far side of it: that part mirrored through
            // the axis, turned half a turn on, is the sweep's there, which a distance set's piece
            // leaves to the far side's faces
            let near = clipped(&c,&|q| rho(q));
            let near = if near.len() < 3 { None } else { Some(turned(vec![polygon(&near,tol)])?) };
            let past: Vec<V> = clipped(&c,&|q| -rho(q)).into_iter().map(|q| sub(q,scale(e,2.*rho(q)))).collect();
            if reach.cut_at_axis || past.len() < 3 { return Ok(near) }
            let far = turned(vec![polygon(&past,tol)])?.moved(&Rigid::turn(fr.o,fr.z,PI));
            return match near {
                None => Ok(Some(far)),
                Some(near) => Ok(Some(combined(&near,&far,Op::Union,0.).map_err(|e| e.msg)?)),
            }
        }
    };
    let vm = 0.5*(v0+v1);
    let k = dot(n_at(vm)?,sub(at(vm),centre)).signum();
    let radii = [r+k*s[0],r+k*s[1]];
    if radii.iter().any(|&q| q < -tol) {
        return Err(format!("swept past the centre of its {}: no point stands that far off it",f.surface.kind()))
    }
    if v1-v0 >= TAU-1e-12 {
        // a whole tube's sweep: the tube of the far radius about its middle circle less the near
        // one's, each cut at the axis where it is wider than the circle
        let Surface::Torus(tf,big,_) = f.surface else { unreachable!() };
        let middle = Curve::Circle(tf,big);
        let (near,far) = (radii[0].min(radii[1]),radii[0].max(radii[1]));
        if far > big+tol && !reach.cut_at_axis { return Err("its sweep passes the axis of its torus".into()) }
        let outer = tube(&middle,[u0,u0+span],far)?;
        if near <= tol { return Ok(Some(outer)) }
        return Ok(Some(combined(&outer,&tube(&middle,[u0,u0+span],near)?,Op::Cut,0.).map_err(|e| e.msg)?))
    }
    // the section's frame turns about the profile's normal, `z × e`, so its `y` is the axis's
    // opposite: the surface's `v` is the arc's `−v`
    let frame = Frame::new(centre,plane_normal,e);
    let pt = |q: f64,v: f64| add(centre,add(scale(e,q*v.cos()),scale(fr.z,q*v.sin())));
    if matches!(f.surface,Surface::Torus(..)) && [v0,v1].iter().flat_map(|&v| radii.map(|q| pt(q,v)))
        .chain(radii.iter().map(|&q| sub(centre,scale(e,q)))).any(|q| dot(sub(q,fr.o),e) < -tol) {
        return Err("its sweep passes the axis of its torus".into())
    }
    let mut l: Vec<ProfileEdge> = radii.iter().filter(|&&q| q > tol).map(|&q| ProfileEdge::Arc {frame,r:q,span:Some([-v1,-v0])}).collect();
    for v in [v0,v1] {
        let (a,z) = (pt(radii[0].max(0.),v),pt(radii[1].max(0.),v));
        if distance(a,z) > tol { l.push(ProfileEdge::Line {a,b:z}); }
    }
    turned(vec![l]).map(Some)
}

/// The polygon `c` kept where `rho` is at least nothing (Sutherland–Hodgman against one line).
fn clipped(c: &[V],rho: &dyn Fn(V) -> f64) -> Vec<V> {
    let mut out = Vec::new();
    for k in 0..c.len() {
        let (p,q) = (c[k],c[(k+1)%c.len()]);
        let (a,z) = (rho(p),rho(q));
        if a >= 0. { out.push(p); }
        if (a < 0.) != (z < 0.) { out.push(lerp(p,q,a/(a-z))); }
    }
    out
}

/// Which sides of face `fi`'s box of parameters (`[u0, u1, v0, v1]`) border faces `walled` does
/// not mark all along: a curved face's sides an opening runs along.
fn open_sides(b: &Brep,uses: &[Vec<(usize,bool)>],fi: usize,walled: &[bool]) -> [bool;4] {
    let f = &b.faces[fi];
    if !matches!(f.surface,Surface::Cylinder(..) | Surface::Cone(..) | Surface::Sphere(..) | Surface::Torus(..)) { return [false;4] }
    let Some(([u0,u1],[v0,v1])) = iso_box(b,f) else { return [false;4] };
    let eps = 1e-9*(1.+u1.abs().max(v1.abs()));
    // each side: whether some edge runs along it, and whether every one that does borders an opening
    let (mut along,mut open) = ([false;4],[true;4]);
    for c in f.loops.iter().flatten() {
        let e = &b.edges[c.edge as usize];
        if matches!(e.curve,EdgeCurve::Degenerate) { continue }
        let Some(g) = uses[c.edge as usize].iter().map(|u| u.0).find(|&g| g != fi) else { continue };
        let mid = c.pcurve.at(0.5*(e.t[0]+e.t[1]),e,&f.surface,&b.vertices);
        for (k,(x,at)) in [(mid[0],u0),(mid[0],u1),(mid[1],v0),(mid[1],v1)].into_iter().enumerate() {
            if (x-at).abs() <= eps { along[k] = true; if walled[g] { open[k] = false; } }
        }
    }
    [0,1,2,3].map(|k| along[k] && open[k])
}

/// The box of a face's parameters, where its loops run only along its own parameter lines and
/// round that box's sides: `([u0, u1], [v0, v1])`.
fn iso_box(b: &Brep,f: &Face) -> Option<([f64;2],[f64;2])> {
    let (runs,[lo,hi]) = samples(b,f);
    let eps = 1e-9*(1.+hi[0].abs().max(hi[1].abs()));
    for (_,run) in &runs {
        let along = (0..2).find(|&k| run.iter().all(|p| (p[k]-run[0][k]).abs() <= eps))?;
        let x = run[0][along];
        if (x-lo[along]).abs() > eps && (x-hi[along]).abs() > eps { return None }
    }
    Some(([lo[0],hi[0]],[lo[1],hi[1]]))
}

/// **The faces `faces` of `b` as a sheet**: those faces alone, with the edges and vertices they
/// use, bounding no solid — a face of a solid lifted out to be thickened.
pub fn sheet_of(b: &Brep,faces: &[usize]) -> Result<Brep,String> {
    if let Some(&f) = faces.iter().find(|&&f| f >= b.faces.len()) { return Err(format!("no face {f} to lift out")) }
    Ok(Brep {faces:faces.iter().map(|&f| b.faces[f].clone()).collect(),..b.clone()}.compacted())
}
