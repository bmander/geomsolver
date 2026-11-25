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
//! `build::sphere`), and combined by its Booleans.  A face no profile's prism or band of
//! revolution takes — a plane's loop of ellipses, a cylinder's face an ellipse bounds — is carried
//! between its moved copies where its move along its normal is an affine map (`carried_sweep`: a
//! plane's, a cylinder's, a sphere's), and a crease along a curve no closed form carries a tube
//! along is the ball carried along it (`swept_tube`).  A face whose normal sweep has no exact
//! construction here yet — a cone's or a torus's face not bounded by its own parameter lines, a
//! surface with no offset in closed form — is refused, naming it and a point on it.
#[allow(unused_imports)]
use crate::util::fmath::Det;
use super::boolean::{combined,combined_known,Known,Op};
use super::build::{kept,prism,revolve,sphere,square,disc,Profile,ProfileEdge};
use super::affine::Affine;
use super::geom::{tolerance,Blend,Carry,Curve,Frame,Offset,Rigid,Surface,Uv,V};
use super::topo::{reverse_loop,Brep,Coedge,EdgeCurve,Face,Pcurve};
use std::sync::Arc;
use super::recipe::Spine;
use super::swept::canal;
use super::trajectory::{Motion,Path,Trajectory};
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

/// What a piece of an offset is made from: a face's sweep, the join along an edge (its index), or
/// the corner at a vertex (its index).
#[derive(Clone,Copy,Debug,PartialEq,Eq,PartialOrd,Ord)]
enum By { Face(usize),Edge(usize),Corner(u32) }

/// The solid `b` grown by `d` (> 0): every point within `d` of it (`Join::Round`), or with its
/// faces moved out by `d` and extended until they meet (`Join::Sharp`).
pub fn grown(b: &Brep,d: f64,join: Join) -> Result<Brep,String> {
    positive(d,"a growth")?;
    let b = &on_spheres(b);
    let all = vec![true;b.faces.len()];
    folded(b.clone(),pieces(b,&all,Side::Out,d,join)?,Op::Union)
}

/// The solid `b` shrunk by `d` (> 0): every point of it at least `d` inside it (`Join::Round`),
/// or with its faces moved in by `d` and extended until they meet (`Join::Sharp`).
pub fn shrunk(b: &Brep,d: f64,join: Join) -> Result<Brep,String> {
    positive(d,"a shrinking")?;
    let b = &on_spheres(b);
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
    let b = &on_spheres(b);
    let walled: Vec<bool> = (0..b.faces.len()).map(|f| !open.contains(&f)).collect();
    if !walled.contains(&true) { return Err("a shell open at every face has no wall".into()) }
    let pieces = match side {
        Side::In => pieces(b,&walled,side,t,join)?,
        // outside, the wall ends where each opening is: the pieces that reach it cut off at its
        // plane, and no other (a wall beyond the plane elsewhere is the solid's own)
        Side::Out => {
            let openings = open.iter().map(|&o| Opening::of(b,o)).collect::<Result<Vec<_>,_>>()?;
            ended(b,&openings,owned_pieces(b,&walled,side,t,join)?,join)?
        }
    };
    let wall = united(pieces,"a shell with no wall")?;
    match side {
        Side::In => combined(b,&wall,Op::Common,0.).map_err(|e| format!("the wall kept within the solid: {e}")),
        Side::Out => combined(&wall,b,Op::Cut,0.).map_err(|e| format!("the solid cut from its wall: {e}")),
    }
}

/// A face left open on a wall outside a solid: its plane, by a point of it and its outward normal,
/// and the edges and vertices it is bounded by.
struct Opening { face: usize,o: V,n: V,edges: BTreeSet<usize>,vertices: BTreeSet<u32> }

impl Opening {
    fn of(b: &Brep,o: usize) -> Result<Opening,String> {
        let f = &b.faces[o];
        let Surface::Plane(fr) = f.surface else {
            return Err(format!("{} is left open on a wall outside the solid: only a plane cuts one off yet",face_name(b,o)))
        };
        let edges: BTreeSet<usize> = f.loops.iter().flatten().map(|c| c.edge as usize).collect();
        let vertices = edges.iter().flat_map(|&e| b.edges[e].v).collect();
        Ok(Opening {face:o,o:fr.o,n:if f.reversed { scale(fr.z,-1.) } else { fr.z },edges,vertices})
    }
    /// How far `p` stands beyond the opening's plane (outward positive).
    fn beyond(&self,p: V) -> f64 { dot(sub(p,self.o),self.n) }
    /// Which side of the plane the points `ps` lie: `Some(-1)` or `Some(1)` all on one side, `Some(0)`
    /// all on it (within `tol`), `None` on both.
    fn side(&self,ps: &[V],tol: f64) -> Option<i8> {
        let (lo,hi) = ps.iter().map(|&p| self.beyond(p)).fold((f64::INFINITY,f64::NEG_INFINITY),|(l,h),x| (l.min(x),h.max(x)));
        match (lo >= -tol,hi <= tol) { (true,true) => Some(0),(false,true) => Some(-1),(true,false) => Some(1),(false,false) => None }
    }
}

/// **Where a piece of an outward shell is ended at an opening** (`by` names what it is made from;
/// `joined`, the edges and corners joins were built at): kept on one side of the opening's plane
/// (`Some((side, must))`, `side` along its outward normal, `must` where the piece reaches past the
/// plane; a cut not a must changes no point of the piece, and is made only beside one that is, to
/// leave the plane one face where it ends them), or left whole (`None`).  A wall lies wholly on
/// one side of the plane, or meets it square, or it is refused, never cut where it should not be:
/// - a join along an opening's edge, or at a corner of it, is a round's (a sharp shell builds none
///   there): kept on the solid's side of its plane, the side its walls stand on, the crease convex;
/// - a face meeting an opening (along an edge or at a corner), or a join along an edge ending at a
///   corner of it, that meets it square ends in its plane: one lying across the plane is left whole
///   (so a face reaching past the plane elsewhere, the floor of an L open at its step, keeps all of
///   itself), as is one lying in it, its wall the opening's neighbour's;
/// - one meeting it aslant is kept on the side it stands on, cut where moved it runs past the
///   plane; where it falls short of the plane the opening's tube or corner ball, a round's, fills
///   the gap, and a sharp shell, whose moved walls would have to be carried on to the plane, and a
///   gap no such join fills, are refused, as is a wall aslant across the plane.
fn ending(b: &Brep,o: &Opening,joined: &BTreeSet<By>,by: By,join: Join,tol: f64) -> Result<Option<(i8,bool)>,String> {
    // what meets the opening: its points, whether it meets the plane square, and, aslant, whether
    // moved it runs past the plane, falls short of it, and what falls short is filled
    let (what,points,square,rates,filled): (&str,Vec<V>,bool,Vec<f64>,bool) = match by {
        By::Corner(v) => return Ok(o.vertices.contains(&v).then_some((-1,true))),
        By::Edge(e) if o.edges.contains(&e) => return Ok(Some((-1,true))),
        By::Edge(e) => {
            let edge = &b.edges[e];
            let EdgeCurve::Curve(c) = &edge.curve else { return Ok(None) };
            let ends: Vec<usize> = (0..2).filter(|&k| o.vertices.contains(&edge.v[k])).collect();
            if ends.is_empty() { return Ok(None) }
            let points = (0..=8).map(|k| edge.point(edge.t[0]+(edge.t[1]-edge.t[0])*k as f64/8.,&b.vertices)).collect();
            let square = ends.iter().all(|&k| dot(unit(c.tangent(edge.t[k])),o.n).abs() >= 1.-1e-9);
            // aslant, the tube's flat end runs past the plane on one side and falls short of it on
            // the other, where the corner's ball fills it
            ("its edge",points,square,vec![-1.,1.],ends.iter().all(|&k| joined.contains(&By::Corner(edge.v[k]))))
        }
        By::Face(fi) => {
            let f = &b.faces[fi];
            let (runs,_) = samples(b,f);
            let shared: Vec<u32> = runs.iter().map(|r| r.0).filter(|&e| o.edges.contains(&(e as usize))).collect();
            let corners: BTreeSet<u32> = runs.iter().flat_map(|r| b.edges[r.0 as usize].v).filter(|v| o.vertices.contains(v)).collect();
            if corners.is_empty() { return Ok(None) }
            let mut contact: Vec<V> = runs.iter().filter(|r| shared.contains(&r.0)).flat_map(|r| r.1.iter().map(|&uv| f.surface.point(uv))).collect();
            contact.extend(corners.iter().map(|&v| b.vertices[v as usize].p));
            let points = runs.iter().flat_map(|(_,run)| run.iter().map(|&uv| f.surface.point(uv))).collect();
            // how fast each point of contact runs past the plane as the face moves out
            let rates = contact.iter().map(|&p| outward(b,fi,p).map(|n| dot(n,o.n))).collect::<Result<Vec<f64>,String>>()?;
            let square = rates.iter().all(|m| m.abs() <= 1e-9);
            // short of the plane: filled where the opening's own tube and corner balls are
            let filled = shared.iter().all(|&e| joined.contains(&By::Edge(e as usize))) && corners.iter().all(|&v| joined.contains(&By::Corner(v)));
            ("its face",points,square,rates,filled)
        }
    };
    match o.side(&points,tol) {
        Some(0) => Ok(None),
        None if square => Ok(None),
        None => Err(format!("{what} runs across the plane of {}, left open, aslant",face_name(b,o.face))),
        Some(s) if square => Ok(Some((s,false))),
        Some(s) => {
            // a rate along the side `s` falls short of the plane, against it runs past
            let k = s as f64;
            if join == Join::Sharp || rates.iter().any(|&m| k*m > 1e-9) && !filled {
                return Err(format!("{what} meets {}, left open, aslant: {}",face_name(b,o.face),match join {
                    Join::Sharp => "a sharp wall carried on to an opening's plane is built only where it meets it square, yet",
                    Join::Round => "what its moved wall leaves short of the plane no round of the opening fills",
                }))
            }
            Ok(Some((s,rates.iter().any(|&m| k*m < -1e-9))))
        }
    }
}

/// **An outward shell's pieces ended at its openings** (`ending`): the pieces ended alike united
/// first and cut once, so each opening's plane holds one face where it ends them all (a box's rim
/// one ring, not a face for each piece that reaches it), and a piece no opening ends left whole.
fn ended(b: &Brep,openings: &[Opening],made: Vec<(By,Piece)>,join: Join) -> Result<Vec<Piece>,String> {
    let tol = tolerance(b.size());
    let joined: BTreeSet<By> = made.iter().map(|m| m.0).filter(|by| !matches!(by,By::Face(_))).collect();
    // the pieces by the cuts that end them, (opening, side) each, and the cuts any of them must take
    let mut groups: BTreeMap<Vec<(usize,i8)>,(Vec<Piece>,BTreeSet<(usize,i8)>)> = BTreeMap::new();
    for (by,(what,p)) in made {
        let (mut cuts,mut musts) = (Vec::new(),Vec::new());
        for (k,o) in openings.iter().enumerate() {
            if let Some((s,must)) = ending(b,o,&joined,by,join,tol).map_err(|e| format!("{what}: {e}"))? {
                cuts.push((k,s));
                if must { musts.push((k,s)); }
            }
        }
        let g = groups.entry(cuts).or_default();
        g.0.push((what,p));
        g.1.extend(musts);
    }
    let mut out = Vec::new();
    for (pieces,musts) in groups.into_values() {
        let what = match &pieces[..] { [(w,_)] => w.clone(),_ => format!("{} pieces ended alike",pieces.len()) };
        let mut group = united(pieces,"no piece")?;
        for (k,s) in musts {
            let o = &openings[k];
            group = kept(&group,o.o,scale(o.n,s as f64),0.).map_err(|e| format!("{what} cut off at the plane of {}: {e}",face_name(b,o.face)))?;
            if group.faces.is_empty() { break }
        }
        if !group.faces.is_empty() { out.push((what,group)); }
    }
    Ok(out)
}

/// A sheet `sheet` (faces bounding no solid, each oriented, neighbours turning one way) thickened
/// over `s` along its faces' normals: every `q + s n(q)`, `s` from `s[0]` to `s[1]` (`s[0] < s[1]`,
/// either side of the sheet), its creases joined as `join` says on the side where its moved faces
/// part, and its rims the walls its boundary sweeps.
pub fn thickened(sheet: &Brep,s: [f64;2],join: Join) -> Result<Brep,String> {
    if !(s[0] < s[1] && s.iter().all(|x| x.is_finite())) { return Err(format!("a sheet thickened over {s:?}, no depth")) }
    let sheet = &on_spheres(sheet);
    let all = vec![true;sheet.faces.len()];
    let uses = uses(sheet);
    let mut pieces = (0..sheet.faces.len()).map(|fi| normal_sweep(sheet,fi,s).map(|p| (face_name(sheet,fi),p)))
        .collect::<Result<Vec<Piece>,String>>()?;
    for (side,depth) in [(Side::Out,s[1]),(Side::In,-s[0])] {
        if depth > 0. { pieces.extend(joins(sheet,&uses,&all,side,depth,join,Of::Sheet)?.into_iter().map(|(_,p)| p)); }
    }
    united(pieces,"a sheet with no face")
}

/// **`b` with each spline face that lies on a sphere put on it** (`sphere_of`, `recharted`): the
/// same set, each such face's edges its own, its pcurves read again on the sphere — so its sweep
/// is a sphere's, exactly, as a ball's is however it was drawn.  A spline face on no sphere, or
/// one the sphere's chart does not hold whole, is left as it is (and refused by its sweep, naming
/// it).
fn on_spheres(b: &Brep) -> Brep {
    let mut out = b.clone();
    for fi in 0..b.faces.len() {
        let Surface::BSpline(_,net) = &b.faces[fi].surface else { continue };
        let Some(sphere) = sphere_of(net,b,fi) else { continue };
        if let Some(face) = recharted(b,fi,sphere) { out.faces[fi] = face; }
    }
    out
}

/// **The sphere a spline sheet lies on, exactly**: on each span of its net the sphere's equation
/// `|P − c w|² − r² w²` (`P / w` the rational sheet) is a polynomial of degree `2 du` in `u` and
/// `2 dv` in `v`, so where it vanishes on a grid of `(2 du + 1) × (2 dv + 1)` points of every span
/// it vanishes on the whole sheet.  The sphere is the least-squares one through those points, held
/// to them within a part in 10¹⁰ of its radius.  Where face `fi` of `b` has a corner its chart
/// collapses to a point (a pole of the net), the sphere's axis runs through it, so its pole stands
/// there too.  `None` for a sheet on no sphere.
fn sphere_of(net: &crate::nurbs::Net,b: &Brep,fi: usize) -> Option<Surface> {
    let spans = |knots: &[f64]| -> Vec<[f64;2]> { knots.windows(2).filter(|w| w[1] > w[0]).map(|w| [w[0],w[1]]).collect() };
    let mut pts = Vec::new();
    for [u0,u1] in spans(&net.uknots) {
        for [v0,v1] in spans(&net.vknots) {
            for i in 0..=2*net.du { for j in 0..=2*net.dv {
                let (a,c) = ((i as f64+0.5)/(2*net.du+1) as f64,(j as f64+0.5)/(2*net.dv+1) as f64);
                pts.push(net.point(u0+a*(u1-u0),v0+c*(v1-v0)));
            } }
        }
    }
    // |p|² = 2 c·p + k, linear in (c, k): the normal equations, solved by elimination
    let mut m = [[0f64;5];4];
    for p in &pts {
        let row = [2.*p[0],2.*p[1],2.*p[2],1.];
        let rhs = dot(*p,*p);
        for i in 0..4 { for j in 0..4 { m[i][j] += row[i]*row[j]; } m[i][4] += row[i]*rhs; }
    }
    for col in 0..4 {
        let piv = (col..4).max_by(|&i,&j| m[i][col].abs().total_cmp(&m[j][col].abs()))?;
        m.swap(col,piv);
        if !(m[col][col].abs() > 1e-300) { return None }
        for i in 0..4 {
            if i == col { continue }
            let f = m[i][col]/m[col][col];
            for j in col..5 { m[i][j] -= f*m[col][j]; }
        }
    }
    let c = [m[0][4]/m[0][0],m[1][4]/m[1][1],m[2][4]/m[2][2]];
    let r2 = m[3][4]/m[3][3]+dot(c,c);
    if !(r2 > 0.) { return None }
    let r = r2.sqrt();
    if pts.iter().any(|&p| (distance(p,c)-r).abs() > 1e-10*r) { return None }
    // its axis through a collapsed corner of the face, where it has one
    let pole = b.faces[fi].loops.iter().flatten().map(|co| &b.edges[co.edge as usize])
        .find(|e| matches!(e.curve,EdgeCurve::Degenerate)).map(|e| b.vertices[e.v[0] as usize].p);
    let z = match pole { Some(p) => unit(sub(p,c)),None => [0.,0.,1.] };
    let x = if z[0].abs() < 0.9 { [1.,0.,0.] } else { [0.,1.,0.] };
    let x = unit(sub(x,scale(z,dot(x,z))));
    Some(Surface::Sphere(Frame::new(c,z,x),r))
}

/// **Face `fi` of `b` put on `surface`**, a surface its points lie on (`sphere_of`): each use's
/// pcurve its edge inverted onto it (`Pcurve::Inverse`), its ends' parameters walked along the
/// loop so a periodic one unwraps, a pole's taken from the use it leads on to or comes from, and a
/// collapsed edge the line between its neighbours' ends; its sense its outward normal's.  `None`
/// where a pole edge stands off the surface's poles, a loop does not close in its parameters, or
/// the face's area on it is not its area on its own surface.
fn recharted(b: &Brep,fi: usize,surface: Surface) -> Option<Face> {
    let old = &b.faces[fi];
    let tol = tolerance(b.size());
    let periods = surface.periods();
    let pole = |uv: Uv| crate::topo::near_pole(&surface,uv,tol).is_some();
    let mut face = old.clone();
    for (li,l) in old.loops.iter().enumerate() {
        // each use's parameters at its walk's start and end, a collapsed one's to be filled in
        let mut ends: Vec<Option<(Uv,Uv)>> = Vec::with_capacity(l.len());
        let mut at: Option<Uv> = None;
        for c in l {
            let e = &b.edges[c.edge as usize];
            if matches!(e.curve,EdgeCurve::Degenerate) { ends.push(None); continue }
            let n = 32;
            let t = |k: usize| { let f = k as f64/n as f64; if c.reversed { e.t[1]-(e.t[1]-e.t[0])*f } else { e.t[0]+(e.t[1]-e.t[0])*f } };
            let mut run: Vec<Uv> = (0..=n).map(|k| surface.inverse(e.point(t(k),&b.vertices))).collect();
            // a pole's collapsed parameter from the next point along the use
            for k in [0,n] {
                if pole(run[k]) { let near = if k == 0 { run[1] } else { run[n-1] }; run[k][0] = near[0]; }
            }
            // walked on from the last use's end, unless one of them stands at a pole
            if let Some(prev) = at { if !pole(run[0]) && !pole(prev) { run[0] = crate::topo::unwrap(run[0],prev,periods); } }
            for k in 1..=n { run[k] = crate::topo::unwrap(run[k],run[k-1],periods); }
            at = Some(run[n]);
            ends.push(Some((run[0],run[n])));
        }
        let m = l.len();
        for (k,c) in face.loops[li].iter_mut().enumerate() {
            let (a,z) = match ends[k] {
                Some(ends) => ends,
                None => {
                    // a collapsed edge: from the last use's end to the next's start, both at the pole
                    let (Some((_,from)),Some((to,_))) = (ends[(k+m-1)%m],ends[(k+1)%m]) else { return None };
                    if !pole(from) || !pole(to) { return None }
                    let to = [to[0],from[1]];
                    c.pcurve = if c.reversed { Pcurve::Line {a:to,b:from} } else { Pcurve::Line {a:from,b:to} };
                    continue
                }
            };
            c.pcurve = if c.reversed { Pcurve::Inverse {a:z,b:a} } else { Pcurve::Inverse {a,b:z} };
        }
    }
    face.surface = surface;
    // its sense: its outward normal there as on its own surface, at a point inside it
    let (_,[lo,hi]) = samples(b,old);
    let mid = [(lo[0]+hi[0])/2.,(lo[1]+hi[1])/2.];
    let x = old.surface.point(mid);
    let out = old.outward(x)?;
    let n = face.surface.normal(face.surface.inverse(x))?;
    face.reversed = dot(n,out) < 0.;
    let (want,got) = (crate::props::area(b,old),crate::props::area(b,&face));
    ((want-got).abs() <= 1e-9*want.max(f64::MIN_POSITIVE)).then_some(face)
}

/// Refused unless `d` is a positive length.
fn positive(d: f64,what: &str) -> Result<(),String> {
    if d > 0. && d.is_finite() { Ok(()) } else { Err(format!("{what} by {d}: the distance must be positive")) }
}

/// The pieces united, or refused as `none` where there are none.
fn united(pieces: Vec<Piece>,none: &str) -> Result<Brep,String> {
    let mut pieces = pieces.into_iter();
    let (_,first) = pieces.next().ok_or(none)?;
    folded(first,pieces.collect(),Op::Union)
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
    Ok(owned_pieces(b,faces,side,d,join)?.into_iter().map(|(_,p)| p).collect())
}

/// `pieces`, each with what it is made from.
fn owned_pieces(b: &Brep,faces: &[bool],side: Side,d: f64,join: Join) -> Result<Vec<(By,Piece)>,String> {
    let s = match side { Side::Out => [0.,d],Side::In => [-d,0.] };
    let uses = uses(b);
    let mut out = Vec::new();
    for fi in (0..b.faces.len()).filter(|&f| faces[f]) {
        // inside, a wall's moved face runs on across a face left open until it meets it
        let (across,opening) = match side {
            Side::In => (open_sides(b,&uses,fi,faces),b.faces[fi].loops.iter().flatten().any(|c| uses[c.edge as usize].iter().any(|&(g,_)| !faces[g]))),
            Side::Out => ([false;4],false),
        };
        if let Some(p) = sweep(b,fi,s,Reach {across,cut_at_axis:true,opening})? { out.push((By::Face(fi),(format!("the sweep of {}",face_name(b,fi)),p))); }
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
fn joins(b: &Brep,uses: &[Vec<(usize,bool)>],faces: &[bool],side: Side,d: f64,join: Join,of: Of) -> Result<Vec<(By,Piece)>,String> {
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
        let closed = e.v[0] == e.v[1];
        let made = match join {
            // a solid's points within `d` of the edge are all of the tube; where the tube cannot be
            // built (it would fold where the edge turns tighter than `d`), the sector its faces'
            // normals span, which is all of it a point nearest the edge stands in
            Join::Round if of == Of::Solid => tube(c,e.t,d).map(|p| vec![p]).or_else(|why| {
                if !turns_tighter(c,e.t,d) { return Err(why) }
                sectors(b,[fa,fb],side,c,e.t,d,closed,true).map_err(|w| format!("{why}; nor its faces' sector: {w}"))
            }),
            // a sheet's, only the wedge between its two moved faces (the rest lies across the
            // sheet or within a face's sweep)
            Join::Round => match c {
                Curve::Line {..} | Curve::Circle(..) => wedge(c,e.t,d,crease.normals.map(|n| scale(n,side.sign()))).map(|p| vec![p]),
                _ => sectors(b,[fa,fb],side,c,e.t,d,closed,false),
            },
            // (a sharp join along a face left open is the opening's to cut off)
            Join::Sharp if !both => continue,
            Join::Sharp => kite(b,&crease,c,e.t,side,d).map(|p| vec![p]),
        }.map_err(|m| format!("{what}: {m}"))?;
        for piece in made { out.push((By::Edge(ei),(what.clone(),piece))); }
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
        out.push((By::Corner(v),(what,piece)));
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
            let e = add(scale(f.x,t[0].dcos()),scale(f.y,t[0].dsin()));
            let centre = add(f.o,scale(e,r));
            let n = cross(f.z,e);
            let plane = Frame::new(centre,n,e);
            // the disc clear of the axis: where it reaches past it, the arc on its side of it, shut
            // along the axis
            let c0 = (-r/d).clamp(-1.,1.).dacos();
            let tol = tolerance(r+d);
            let loops = if d < r-tol {
                vec![vec![ProfileEdge::Arc {frame:plane,r:d,span:None}]]
            } else {
                // as wide as the circle (a horn torus), the disc touches the axis at one point:
                // the arc closes there, its ends on the axis as a sphere's meridian's are
                let at = |a: f64| plane.at([d*a.dcos(),d*a.dsin(),0.]);
                let mut l = vec![ProfileEdge::Arc {frame:plane,r:d,span:Some([-c0,c0])}];
                if distance(at(c0),at(-c0)) > tol { l.push(ProfileEdge::Line {a:at(c0),b:at(-c0)}); }
                vec![l]
            };
            revolve(&profile(centre,n,loops),f.o,f.z,(t[1]-t[0]).min(TAU))
        }
        _ => swept_tube(c,t,d),
    }
}

/// A path that steps along the curve `curve`: at `t` the translation from `start` to its point at
/// `t`, at its speed there. What carries a ball's centre along an edge.
#[derive(Debug)]
struct Stepped { curve: Curve,start: V }

impl Path for Stepped {
    fn at(&self,t: f64) -> Result<Motion,crate::KernelError> {
        let (p,d) = self.curve.d1(t);
        Motion::translation(sub(p,self.start),d)
            .map_err(|e| crate::KernelError::new(crate::Kind::Failed,format!("the curve at {t}: {e:?}")))
    }
}

/// **The tube of radius `d` about the stretch `t` of a curve no closed form carries a tube along**,
/// its ends flat: the ball of radius `d` carried along it (`swept::canal::tube`, the one tube a
/// ball sweeps along any path), its wall the face family of its disc square to the curve, held
/// within the family's `EXACT` of its diameter. A closed curve's closes on itself with no end
/// (`canal::ring`, `family::ring`), so each face's moved copy of the curve lies along its wall
/// whole. Refused, by the tube's rows, where the curve turns tighter than `d` or comes back within
/// `2 d` of itself (the tube would fold or cross itself), and where it closes at a corner.
fn swept_tube(c: &Curve,t: [f64;2],d: f64) -> Result<Brep,String> {
    let (k,start) = (c.kind(),c.point(t[0]));
    let closed = distance(start,c.point(t[1])) <= tolerance(norm(start)+d);
    if closed && dot(unit(c.tangent(t[0])),unit(c.tangent(t[1]))) < 1.-1e-9 {
        return Err(format!("the {k} closes on itself at a corner at {start:?}: no ball's tube runs round it smoothly"))
    }
    let path = Trajectory::General(Arc::new(Stepped {curve:c.clone(),start}));
    let walls = if closed { canal::ring(&path,start,d,"",t) } else {
        canal::tube(&path,start,d,"",t).map(|tube| tube.and_then(|tube| match tube.spine { Spine::Tube(f) => Some(f),_ => None }))
    };
    let why = match walls {
        Ok(Some(f)) => return crate::recipe::family_solid(&f,closed),
        // the ball's own, where the centre never moves: an edge of no length
        Ok(None) => return Err(format!("the {k} has no length to carry the ball along")),
        Err(why) => why,
    };
    Err(match why {
        canal::Why::Refused(r) => match r.seen {
            canal::Seen::Turns {radius,..} =>
                format!("the {k} turns on a radius of {radius} at {:?}, tighter than the tube's {d}: the tube would fold",r.witness),
            canal::Seen::Stops {..} => format!("the {k} stops at {:?}, where it has no tangent",r.witness),
            canal::Seen::Returns {distance,..} => format!("the {k} comes back within {distance} of itself near {:?}, closer than \
                the tube's diameter {}: the tube would cross itself",r.witness,2.*d),
        },
        other => format!("the ball's tube along the {k} is not built: {other:?}"),
    })
}

/// The wedge of the tube of radius `d` about the stretch `t` of the curve `c` between the
/// directions `n` (each square to the curve at its start), the round between two faces moved
/// along them: a sector's prism along a line, turned about a circle.
fn wedge(c: &Curve,t: [f64;2],d: f64,n: [V;2]) -> Result<Brep,String> {
    let a = c.point(t[0]);
    // the sector turns from `n[0]` to `n[1]` about the side `n[0] × n[1]` points along
    let axis = unit(cross(n[0],n[1]));
    let angle = dot(n[0],n[1]).clamp(-1.,1.).dacos();
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

/// **The sector of the tube of radius `d` about the stretch `t` of the crease `c` between the faces
/// `faces`**, moved to `side`: at each point of the curve the arc of the ball's disc square to it
/// from the first face's normal round to the second's — every point nearest the crease stands in
/// it, the sector a ball rolled along the crease between the two moved faces sweeps — and, where
/// `widen` says, past each normal on into the face's own sweep (by an eighth of a turn, or as far as
/// keeps it clear of the fold), and its apex stood back from the crease into the solid (a quarter
/// of `d`, or as far as keeps it clear), so it overlaps its neighbours and holds the crease within
/// it, as the tube does, rather than meeting them face to face along it.  The disc is convex, so
/// the hull of any arc of it and any point of it lies within `d` of the crease, and the sector
/// widened is no less exact.  A face family (`family::family`) of the section's walls: its two radii and its arc, the
/// arc's numbers moving with the faces' normals (`Run::Round`), held within the family's `EXACT`
/// of its diameter.  Where the full tube would fold — the curve turning tighter than `d` — the
/// fold lies toward the curve's centre of curvature, and the sector, standing on the faces'
/// outward side, clear of it; refused where it is not (the arc would fold too).  A closed crease's
/// closes on itself with no end (`family::rings`), as its tube does, so each face's moved copy of
/// the crease lies along its arc's wall whole.
fn sectors(b: &Brep,faces: [usize;2],side: Side,c: &Curve,t: [f64;2],d: f64,closed: bool,widen: bool) -> Result<Vec<Brep>,String> {
    let period = t[1]-t[0];
    if !(period > 0.) { return Err("the crease has no length".into()) }
    // the curve's own parameter for a station's, round again past its end where it closes
    let wrap = |s: f64| if closed && s > t[1] { t[0]+(s-t[0]).rem_euclid(period) } else { s };
    // the axis the arc turns about, the curve's tangent, the way that turns the first normal to the second
    let normals = |s: f64| -> Result<(V,[V;2]),String> {
        let p = c.point(wrap(s));
        Ok((p,[outward(b,faces[0],p)?,outward(b,faces[1],p)?].map(|n| scale(n,side.sign()))))
    };
    let mid = 0.5*(t[0]+t[1]);
    let (_,[a,z]) = normals(mid)?;
    let turn = dot(cross(a,z),c.tangent(mid)).signum();
    // the arc at `s` from its start, widened by `m` each way: its point, its start and its quarter
    // turn on (unit), and its sweep
    let arc = |s: f64,m: f64| -> Result<(V,V,V,f64),String> {
        let (p,[n0,n1]) = normals(s)?;
        let axis = scale(unit(c.tangent(wrap(s))),turn);
        let angle = dot(cross(n0,n1),axis).datan2(dot(n0,n1));
        if !(angle > 1e-9 && angle < PI-1e-9) { return Err(format!("its faces' normals do not part at {p:?}")) }
        let (sn,cs) = (-m).dsin_cos();
        let y = cross(axis,n0);
        let u0 = add(scale(n0,cs),scale(y,sn));
        Ok((p,u0,cross(axis,u0),angle+2.*m))
    };
    // how far the section's farthest point (an end of the arc's, or the apex `back` behind the
    // crease along the arc's middle) is toward the curve's centre of curvature, as a part of the
    // fold's distance there: the fold where it reaches 1
    let steps = 256;
    let (s0,s1) = if closed { (t[0],t[0]+period) } else { (t[0],t[1]) };
    let bend = |m: f64,back: f64| -> Result<f64,String> {
        let mut most = f64::NEG_INFINITY;
        for k in 0..=steps {
            let s = s0+(s1-s0)*k as f64/steps as f64;
            let (_,v,acc) = c.d2(wrap(s));
            let vv = dot(v,v);
            if !(vv > 0.) { return Err(format!("it stops at {:?}, where it has no tangent",c.point(wrap(s)))) }
            // the curvature vector
            let kappa = scale(sub(acc,scale(v,dot(acc,v)/vv)),1./vv);
            let (_,u0,y,sweep) = arc(s,m)?;
            for j in 0..=32 {
                let (sn,cs) = (sweep*j as f64/32.).dsin_cos();
                most = most.max(d*dot(add(scale(u0,cs),scale(y,sn)),kappa));
            }
            let (sn,cs) = (0.5*sweep).dsin_cos();
            most = most.max(-back*dot(add(scale(u0,cs),scale(y,sn)),kappa));
        }
        Ok(most)
    };
    let (mut m,mut back) = (0.,0.);
    if widen {
        'wide: for wide in [FRAC_PI_2/2.,FRAC_PI_2/4.,FRAC_PI_2/8.] {
            for k in [d/4.,d/8.,d/16.] {
                if bend(wide,k).is_ok_and(|x| x <= 0.5) { (m,back) = (wide,k); break 'wide }
            }
        }
    }
    let worst = bend(m,back)?;
    if !(worst < 1.-1e-6) { return Err(format!("its arc between its faces' normals reaches {worst} of the way to where it folds")) }
    let section = |s: f64| -> Result<Vec<Vec<f64>>,String> {
        let (p,u0,y,sweep) = arc(s,m)?;
        let (sn,cs) = sweep.dsin_cos();
        let u1 = add(scale(u0,cs),scale(y,sn));
        let (a,z) = (add(p,scale(u0,d)),add(p,scale(u1,d)));
        let (sn,cs) = (0.5*sweep).dsin_cos();
        let apex = sub(p,scale(add(scale(u0,cs),scale(y,sn)),back));
        Ok(vec![[apex,a].concat(),[p.as_slice(),&scale(u0,d),&scale(y,d),&[sweep]].concat(),[z,apex].concat()])
    };
    use crate::family::Run;
    let f = fitted(&section,vec![Run::Line,Run::Round,Run::Line],[s0,s1],d,closed)?;
    Ok(vec![crate::recipe::family_solid(&f,closed)?])
}

/// Whether the stretch `t` of the curve `c` turns anywhere on a radius under `d`, read at 512
/// points: where the tube of radius `d` about it would fold.
fn turns_tighter(c: &Curve,t: [f64;2],d: f64) -> bool {
    (0..=512).any(|k| {
        let (_,v,acc) = c.d2(t[0]+(t[1]-t[0])*k as f64/512.);
        let vv = dot(v,v);
        vv > 0. && d*norm(cross(v,acc)) >= vv*vv.sqrt()*(1.-1e-9)
    })
}

/// **A section's face family fitted over the interval `span`**: the section's walls (of the kinds
/// `runs`, in walk order, as `section` gives their numbers) at the six Gauss–Lobatto stations of
/// each piece, the pieces halved until the numbers read between their nodes agree with the
/// section's within the family's `EXACT` of the size `d` (a plain number, an arc's sweep, counted
/// as so much of `d`).  Where the section comes back to itself (`closed`), the last station is the
/// first, to the bit.
fn fitted(section: &dyn Fn(f64) -> Result<Vec<Vec<f64>>,String>,runs: Vec<crate::family::Run>,[from,to]: [f64;2],d: f64,closed: bool)
    -> Result<crate::recipe::Family,String> {
    use crate::family::{EXACT,LONGEST_PIECE,NODES,weights};
    use crate::recipe::{Family,FamilyWall};
    const CHECKED_AT: [f64;3] = [0.06,0.5,0.94];
    let span = to-from;
    let tol = EXACT*2.*d;
    let flat = |w: &Vec<Vec<f64>>| -> Vec<f64> { w.concat() };
    let mut here = flat(&section(from)?);
    let lens: Vec<usize> = section(from)?.iter().map(Vec::len).collect();
    if lens.len() != runs.len() { return Err("a section's walls and their kinds differ in number".into()) }
    // which numbers are plain (counted as so much of `d`): each wall's after its points and directions
    let plain: Vec<bool> = runs.iter().zip(&lens).flat_map(|(run,&len)| {
        let (p,dirs,_) = run.shape(len);
        (0..len).map(move |j| j >= 3*(p+dirs))
    }).collect();
    let (mut h,mut s) = (span/16.,from);
    let (mut spans,mut at,mut nodes): (Vec<[f64;2]>,Vec<[f64;6]>,Vec<Vec<f64>>) = (Vec::new(),Vec::new(),Vec::new());
    while to-s > span*1e-12 {
        if spans.len() >= 4096 { return Err(format!("its walls are not held within {tol} in 4096 pieces")) }
        let end = if h >= to-s { to } else { s+h };
        let ss: [f64;6] = NODES.map(|x| s+(end-s)*(x+1.)/2.);
        let mut piece = vec![here.clone()];
        for &q in &ss[1..] { piece.push(flat(&section(q)?)); }
        let mut miss = 0f64;
        for f in CHECKED_AT {
            let q = s+(end-s)*f;
            let w = weights(&ss,q);
            let want = flat(&section(q)?);
            for j in 0..want.len() {
                let got: f64 = (0..6).map(|i| w[i]*piece[i][j]).sum();
                let scale = if plain[j] { d } else { 1. };
                miss = miss.max(scale*(got-want[j]).abs());
            }
        }
        if miss > tol && end-s > span*1e-9 { h = (end-s)*0.5; continue }
        if miss > tol*1e3 { return Err(format!("its walls miss its section by {miss} between {s} and {end}")) }
        h = ((end-s)*if miss > 0. { (0.9*(tol/miss).powf(1./6.)).clamp(0.5,2.) } else { 2. }).min(span*LONGEST_PIECE);
        spans.push([s,end]);
        at.push(ss);
        (s,here) = (end,piece[5].clone());
        nodes.extend(piece);
    }
    if closed {
        let first = nodes[0].clone();
        if let Some(last) = nodes.last_mut() { *last = first; }
    }
    let mut offset = 0;
    let loops = vec![runs.into_iter().zip(&lens).map(|(run,&len)| {
        let wall = FamilyWall {name:String::new(),run,closed:false,nodes:nodes.iter().map(|n| n[offset..offset+len].to_vec()).collect()};
        offset += len;
        wall
    }).collect()];
    Ok(Family {spans,at,vanish:1e-6*d,loops,carry:None})
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
/// face's meridian and its moved copies turned through the face's span; any other plane's,
/// cylinder's or sphere's the face carried between its moved copies (`carried_sweep`).  Refused
/// where the sweep passes the surface's centre.
pub fn normal_sweep(b: &Brep,fi: usize,s: [f64;2]) -> Result<Brep,String> {
    sweep(b,fi,s,Reach::default())?.ok_or_else(|| format!("{} is swept wholly past its axis",face_name(b,fi)))
}

/// How far a face's sweep reaches past the face itself: across each side of its box of
/// parameters (`[u0, u1, v0, v1]`) that borders a face left open, on to the end of its surface's
/// own range (a moved wall runs on until it meets the opening, which the solid then trims); and
/// whether the part past the surface's axis is cut away, as a distance set's piece may be (a point
/// past the axis is nearer the surface on the axis's far side, and not this face's); and whether
/// the face borders an opening at all, across which a face carried between its moved copies does
/// not run on yet.
#[derive(Clone,Copy,Debug,Default)]
struct Reach { across: [bool;4],cut_at_axis: bool,opening: bool }

/// The sweep `normal_sweep` builds, reaching as `reach` says; none where it is cut away wholly at
/// the axis.
fn sweep(b: &Brep,fi: usize,s: [f64;2],reach: Reach) -> Result<Option<Brep>,String> {
    let f = &b.faces[fi];
    let refuse = |why: String| -> String {
        let witness = f.loops.iter().flatten().next().map(|c| b.vertices[b.edges[c.edge as usize].v[0] as usize].p);
        format!("{} is not swept along its normal: {why} (at {witness:?})",face_name(b,fi))
    };
    // where the face's loops are no profile's or its boundary not its own parameter lines, the face
    // carried between its moved copies
    let carried = |why: &str| if reach.opening { Err(format!("{why}, and it borders an opening")) } else {
        carried_sweep(b,fi,s).map(Some).map_err(|m| format!("{why}, and carried between its moved copies, {m}"))
    };
    match f.surface {
        Surface::Plane(fr) => {
            let normal = scale(fr.z,if f.reversed { -1. } else { 1. });
            match f.loops.iter().map(|l| plane_loop(b,l,normal)).collect::<Result<Vec<_>,_>>() {
                Ok(loops) => prism(&profile(fr.o,normal,loops),s[0],s[1]).map(Some),
                Err(why) => carried(&format!("its loops are no profile's: {why}")),
            }
        }
        Surface::Cylinder(..) | Surface::Cone(..) | Surface::Sphere(..) | Surface::Torus(..) => match iso_box(b,f) {
            Some(bx) => revolved_sweep(b,f,bx,s,reach),
            None => carried("its boundary is not its own parameter lines"),
        },
        Surface::Extrusion(..) => match iso_box(b,f) {
            _ if reach.opening || reach.across.contains(&true) => Err("an extrusion's face bordering an opening is not swept yet".into()),
            Some(bx) => extruded_sweep(b,fi,bx,s,reach.cut_at_axis).map(Some),
            None => Err("an extrusion's face is swept where its boundary is its own parameter lines, and this one's is not".into()),
        },
        _ => Err(format!("a {} has no offset in closed form",f.surface.kind())),
    }.map_err(refuse)
}

/// **An extrusion's face swept along its normal**: a curve `c(u)` swept along a line `z`,
/// `S(u, v) = c(u) + v z`, has its normal `n(u)` square to `z` and the same all along each line of
/// the sweep, so the face's points over a line `u` moved along it fill the rectangle between
/// `c(u) + v z` (`v` over the face's span) and its copy `s` off it along `n(u)` — four lines in the
/// plane of `z` and `n(u)`.  The sweep is the face family of that rectangle as `u` runs
/// (`family::family`; closed on itself, with no end, where the face runs round its curve,
/// `family::rings`), each wall a line run, held within the family's `EXACT` of its size; the face
/// moved is its wall at `s[1]`, exactly `c(u) + s n(u)` swept along `z` — the offset of a
/// spline or an ellipse, which has no closed form.  As a distance set's piece (`overlap`), the
/// rectangle reaches back past the face by a quarter of the distance (or less, clear of the fold)
/// into what the solid or the face's other sweep covers, so the face itself is no wall of it and
/// no Boolean meets the solid's face along it: every point within a distance less than `d` of a
/// point of the face is within `d` of the solid.  Refused where the rectangle's moved side would
/// pass the curve's centre of curvature (the sweep would fold).
fn extruded_sweep(b: &Brep,fi: usize,([u0,u1],[v0,v1]): ([f64;2],[f64;2]),s: [f64;2],overlap: bool) -> Result<Brep,String> {
    use crate::family::Run;
    let f = &b.faces[fi];
    let Surface::Extrusion(frame,_) = f.surface else { return Err("no extrusion".into()) };
    let out = if f.reversed { -1. } else { 1. };
    let z = unit(frame.z);
    let normal = |u: f64| -> Result<(V,V),String> {
        let (p,du,_) = f.surface.d1([u,v0]);
        let n = cross(du,z);
        if !(norm(n) > 0.) { return Err(format!("its curve stops at {p:?}, where it has no normal")) }
        Ok((p,scale(unit(n),out)))
    };
    // the face closes round its curve where its two sides `u0`, `u1` are one line
    let closed = distance(f.surface.point([u0,v0]),f.surface.point([u1,v0])) <= tolerance(b.size());
    // how far round the fold each side `s` reaches: `∂u (c + s n) · c'` over `|c'|²`, which is 1 at
    // the face and nothing where the sweep folds
    let steps = 512;
    let h = 1e-6*(u1-u0);
    let unfolded = |sides: [f64;2]| -> Result<f64,String> {
        let mut least = f64::INFINITY;
        for k in 0..=steps {
            let u = (u0+(u1-u0)*k as f64/steps as f64).clamp(u0+h,u1-h);
            let (_,du,_) = f.surface.d1([u,v0]);
            let dn = scale(sub(normal(u+h)?.1,normal(u-h)?.1),0.5/h);
            for sd in sides { least = least.min(1.+sd*dot(dn,du)/dot(du,du)); }
        }
        Ok(least)
    };
    let mut span = s;
    if overlap {
        let reach = 0.25*(s[1]-s[0]);
        let back = [reach,reach/4.,reach/16.].into_iter().find(|&k| {
            let x = if s[0] == 0. { [-k,s[1]] } else { [s[0],k] };
            unfolded(x).is_ok_and(|m| m >= 0.25)
        });
        if let Some(k) = back { span = if s[0] == 0. { [-k,s[1]] } else { [s[0],k] }; }
    }
    let least = unfolded(span)?;
    if !(least > 1e-6) {
        return Err(format!("its sweep to {} folds where its curve turns tighter than that (its rate across the fold {least})",if s[0] == 0. { s[1] } else { s[0] }))
    }
    let section = |u: f64| -> Result<Vec<Vec<f64>>,String> {
        let (p,n) = normal(u)?;
        let at = |v: f64,sd: f64| add(add(p,scale(z,v-v0)),scale(n,sd));
        let c = [at(v0,span[0]),at(v1,span[0]),at(v1,span[1]),at(v0,span[1])];
        Ok((0..4).map(|k| [c[k],c[(k+1)%4]].concat()).collect())
    };
    let size = (v1-v0).abs().max(span[1]-span[0]);
    let fam = fitted(&section,vec![Run::Line;4],[u0,u1],size,closed)?;
    crate::recipe::family_solid(&fam,closed)
}

/// **A face swept along its normal by carrying it between its moved copies** (`normal_sweep`, any
/// boundary): on a plane, a cylinder or a sphere the move `q ↦ q + h n(q)` of the face's points is
/// an affine map `A_h` of space, linear in `h`, carrying the surface onto its
/// offset (`Surface::offset`) with its parameters kept. So the face moved to `h` is the face's
/// image under `A_h` — its edges' images (`Curve::mapped`: an ellipse's an ellipse) on the offset
/// surface, its pcurves its own — and the wall an edge sweeps is the ruled surface joining each of
/// its points' images at `s[0]` and `s[1]`, `(1 − v) A₀(q) + v A₁(q) = A_{s₀ + v (s₁ − s₀)}(q)`:
/// exactly a `Blend` of the edge's two images (a plane where they lie in one). The rails between
/// them are the normals at the face's vertices. Refused where the move takes the surface past its
/// centre or axis, and for a cone or a torus, whose move moves each point by its own map.
fn carried_sweep(b: &Brep,fi: usize,s: [f64;2]) -> Result<Brep,String> {
    let f = &b.faces[fi];
    let out = if f.reversed { -1. } else { 1. };
    let tol = tolerance(b.size()+s[1]-s[0]);
    let sheet = sheet_of(b,&[fi])?;
    // the face moved to `h`: its image under the move, on the surface's offset
    let moved = |h: f64| -> Result<Brep,String> {
        let d = out*h;
        let Some(Offset::Surface(surface)) = f.surface.offset(d,tol) else {
            return Err(format!("swept past the centre of its {}: no point stands that far off it",f.surface.kind()))
        };
        let map = match f.surface {
            Surface::Plane(fr) => Affine::new(Affine::IDENTITY.m,scale(fr.z,d)),
            Surface::Cylinder(fr,r) => Affine::stretch(&fr,[(r+d)/r,(r+d)/r,1.]),
            Surface::Sphere(fr,r) => Affine::stretch(&fr,[(r+d)/r;3]),
            _ => return Err(format!("a {}'s move along its normal is no affine map",f.surface.kind())),
        }?;
        let mut c = sheet.mapped(&map);
        // the offset's parameters are the face's own (an affine image's, wrapped, are too)
        c.faces[0].surface = surface;
        Ok(c)
    };
    let (near,far) = (moved(s[0])?,moved(s[1])?);
    let (nv,ne) = (near.vertices.len() as u32,near.edges.len() as u32);
    let mut solid = near.clone();
    solid.join(&far);
    // the face moved to `s[0]` bounds the sweep from the other side
    solid.faces[0] = crate::complex::turned(&solid.faces[0]);
    // the normal at each vertex of the face, between its two copies, made where a wall uses it
    let mut rails: BTreeMap<u32,u32> = BTreeMap::new();
    let mut rail = |solid: &mut Brep,v: u32| -> u32 {
        *rails.entry(v).or_insert_with(|| {
            let (p,q) = (solid.vertices[v as usize].p,solid.vertices[(v+nv) as usize].p);
            solid.edge(EdgeCurve::Curve(Curve::Line {p,d:unit(sub(q,p))}),[0.,distance(p,q)],[v,v+nv])
        })
    };
    let face = &near.faces[0];
    let seams = face.seams();
    for c in face.loops.iter().flatten() {
        let (lo,hi) = (c.edge,c.edge+ne);
        let e = &near.edges[lo as usize];
        let EdgeCurve::Curve(a) = &e.curve else { continue };
        if seams.contains(&lo) { continue }
        let EdgeCurve::Curve(z) = &far.edges[lo as usize].curve else { unreachable!("an edge's image is a curve") };
        let tb = far.edges[lo as usize].t;
        let closed = e.v[0] == e.v[1];
        // out of the face across the edge where its walk is the face's: the walk's tangent across
        // the face's outward normal
        let mid = 0.5*(e.t[0]+e.t[1]);
        let p = a.point(mid);
        let walk = scale(a.tangent(mid),if c.reversed { -1. } else { 1. });
        let n = face.outward(p).ok_or_else(|| format!("no normal at {p:?} of its edge"))?;
        let away = cross(walk,n);
        let pts: Vec<V> = (0..=16).flat_map(|k| { let u = k as f64/16.; [a.point(e.t[0]+u*(e.t[1]-e.t[0])),z.point(tb[0]+u*(tb[1]-tb[0]))] }).collect();
        // the wall's surface, and its parameters at the edge's ends and their images (a plane's
        // read off it, a blend's the corners of its square), each use's pcurve between two of them
        let corners = [e.v[0],e.v[1],e.v[0]+nv,e.v[1]+nv].map(|v| solid.vertices[v as usize].p);
        let (surface,uv,pc): (Surface,[Uv;4],fn(Uv,Uv) -> Pcurve) = match flat(&pts,tol) {
            Some(plane) => (Surface::Plane(plane),corners.map(|q| { let l = plane.local(q); [l[0],l[1]] }),|a,b| Pcurve::Inverse {a,b}),
            None => {
                let blend = Blend {a:a.clone(),ta:e.t,b:z.clone(),tb,carry:Carry::Line {delta:[0.;3]},closed};
                (Surface::Blend(Frame::about(p,away),Arc::new(blend)),[[0.,0.],[1.,0.],[0.,1.],[1.,1.]],|a,b| Pcurve::Line {a,b})
            }
        };
        let used = |edge: u32,reversed: bool,i: usize,j: usize| Coedge {edge,reversed,pcurve:pc(uv[i],uv[j])};
        let mut loops = if closed && matches!(surface,Surface::Plane(_)) {
            // an annulus: the edge and its image, each a loop of its own
            vec![vec![used(lo,false,0,1)],vec![used(hi,true,2,3)]]
        } else {
            let (r0,r1) = (rail(&mut solid,e.v[0]),rail(&mut solid,e.v[1]));
            vec![vec![used(lo,false,0,1),used(r1,false,1,3),used(hi,true,2,3),used(r0,true,0,2)]]
        };
        let reversed = dot(surface.normal_raw(surface.inverse(p)),away) < 0.;
        let wall = Face {surface,reversed,loops:vec![],name:String::new()};
        // the outer loop counter-clockwise about the material's side, a hole's the other way
        let areas: Vec<f64> = loops.iter().map(|l| solid.loop_area(&wall,l)).collect();
        let outer = (0..loops.len()).max_by(|&i,&j| areas[i].abs().total_cmp(&areas[j].abs())).unwrap_or(0);
        for (k,l) in loops.iter_mut().enumerate() {
            if (areas[k] > 0.) == (reversed == (k == outer)) { *l = reverse_loop(std::mem::take(l)); }
        }
        loops.swap(0,outer);
        solid.faces.push(Face {loops,..wall});
    }
    // every rail between two walls, or the face's boundary is not one the walls close
    let uses = solid.uses();
    if let Some((_,&r)) = rails.iter().find(|&(_,&r)| uses[r as usize].len() != 2) {
        return Err(format!("its walls do not close at the normal through {:?}",solid.vertices[solid.edges[r as usize].v[0] as usize].p))
    }
    Ok(solid)
}

/// The plane the points `pts` lie in within `tol`, or none: through the first, across the farthest
/// from it and the point farthest off that line.
fn flat(pts: &[V],tol: f64) -> Option<Frame> {
    let o = pts[0];
    let far = *pts.iter().max_by(|a,b| distance(**a,o).total_cmp(&distance(**b,o)))?;
    let x = unit(sub(far,o));
    let off = *pts.iter().max_by(|a,b| norm(cross(x,sub(**a,o))).total_cmp(&norm(cross(x,sub(**b,o)))))?;
    let n = cross(x,sub(off,o));
    if !(norm(n) > tol) { return None }
    let n = unit(n);
    pts.iter().all(|&q| dot(sub(q,o),n).abs() <= tol).then(|| Frame::new(o,n,x))
}

/// A plane face's loop `l` as a profile's edges about `normal`: its lines, its arcs (each frame
/// turned to the normal) and its whole splines; a degenerate edge is passed over. What a plane face
/// is swept from (`sweep`), and what a tool's cap is read as one prism's profile from
/// (`swept::admission::Lone::read`).
pub(crate) fn plane_loop(b: &Brep,l: &[crate::topo::Coedge],normal: V) -> Result<Vec<ProfileEdge>,String> {
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
    Ok(edges)
}

/// A cylinder's, cone's, sphere's or torus's face's sweep: its meridian's section between its
/// moved copies, turned through its span about the axis.
fn revolved_sweep(b: &Brep,f: &Face,([mut u0,u1],[mut v0,mut v1]): ([f64;2],[f64;2]),s: [f64;2],reach: Reach) -> Result<Option<Brep>,String> {
    let fr = *f.surface.frame();
    let out = if f.reversed { -1. } else { 1. };
    let mut span = (u1-u0).min(TAU);
    // across a side bordering an opening: on to the end of the surface's range
    let [ul,uh,vl,vh] = reach.across;
    if ul || uh { u0 = 0.; span = TAU; }
    let long = 4.*b.size();
    match f.surface {
        Surface::Sphere(..) => { if vl { v0 = -FRAC_PI_2; } if vh { v1 = FRAC_PI_2; } }
        Surface::Torus(..) => if vl || vh { v1 = v0+TAU; },
        Surface::Cone(_,r,a) if a.dsin().abs() > 1e-12 => {
            // not past the apex
            let apex = -r/a.dsin();
            if vl { v0 = if apex < v0 { apex.max(v0-long) } else { v0-long }; }
            if vh { v1 = if apex > v1 { apex.min(v1+long) } else { v1+long }; }
        }
        _ => { if vl { v0 -= long; } if vh { v1 += long; } }
    }
    let e = add(scale(fr.x,u0.dcos()),scale(fr.y,u0.dsin()));
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
    // past the centre of its section's circle (a torus's tube, a sphere's) the moved face turns
    // inside out; a point there is no nearer this face than the centre's distance (the focal
    // distance bounds a nearest point's), so a distance set's piece stops at the centre, and the
    // far side's faces own what lies beyond it (a concave round grown past its radius, a convex
    // one shrunk past it)
    let mut radii = [r+k*s[0],r+k*s[1]];
    if reach.cut_at_axis { radii = radii.map(|q| q.max(0.)); }
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
    let pt = |q: f64,v: f64| add(centre,add(scale(e,q*v.dcos()),scale(fr.z,q*v.dsin())));
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
pub(crate) fn clipped(c: &[V],rho: &dyn Fn(V) -> f64) -> Vec<V> {
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
