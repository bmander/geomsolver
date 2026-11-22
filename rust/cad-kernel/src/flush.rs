//! **An end flush with a face is on it** (#836): an operand of a body's Boolean whose planar face
//! stands parallel to a plane of the solid and nearer it than `FLUSH` of the Boolean's tolerances
//! — a cut ending all but flush with another cut's floor, a boss all but level with the top — has
//! that face moved onto the plane before the two are combined (`boolean::combined_written`), so the
//! Boolean reads them as one plane (a coincidence, `ssi::same`) and leaves no sliver between them:
//! no strip of wall a few nanometres tall, no face no export can write. The move is exact: the
//! face's plane, its edges and its corners carried along its normal, and the walls it ends — every
//! face beside it a surface the move leaves where it was (a plane, a cylinder or an extrusion along
//! that normal), every other edge reaching it a line along the normal — lengthened or shortened,
//! their uses read anew. A face whose walls are anything else (a cone, a sweep) is left where it
//! is, and a sliver it makes is found by `sliver::slivers`. Ends farther apart than the reach are as
//! drawn, to the Boolean's own tolerance.
use super::boolean::overlap;
use super::geom::{include,widen,Box3,Curve,Frame,Rigid,Surface,EMPTY,V};
use super::topo::{Brep,EdgeCurve,Face,Pcurve};
use crate::util::space::{add,cross,distance,dot,norm,scale,sub};
use std::collections::{BTreeMap,BTreeSet};

/// How many of a Boolean's tolerances (`geom::tolerance_over`, a part in 10⁹ of the solids' size)
/// a planar face of an operand may stand off a parallel plane of the solid and be moved onto it: a
/// part in a million of their size, far under anything a person draws apart on purpose and far
/// over what arithmetic leaves of two ends written the same.
pub const FLUSH: f64 = 1e3;

/// How far a surface's axis may lean off the normal a face moves along and still be read as along
/// it, as a sine: over a reach of a part in a million it moves the surface a part in 10¹⁵.
const ALONG: f64 = 1e-9;

/// Whether two planes' normals are one line.
fn parallel(a: V,b: V) -> bool { norm(cross(a,b)) <= 1e-12 }

/// `operand` with each of its planar faces standing within `FLUSH tol` of a parallel plane of a
/// face of `solid` near it moved onto that plane (the nearest, where two are near), where its walls
/// let it move; `None` where none moves. A face already on a plane of the solid, within `tol`,
/// stays. A face's box is read only once a plane stands within reach of it.
pub fn flushed(solid: &Brep,operand: &Brep,tol: f64) -> Option<Brep> {
    let reach = FLUSH*tol;
    let planes: Vec<(&Face,Frame)> = solid.faces.iter().filter(|f| !f.loops.is_empty())
        .filter_map(|f| match f.surface { Surface::Plane(fr) => Some((f,fr)),_ => None }).collect();
    let mut out: Option<Brep> = None;
    let mut seen = vec![false;operand.faces.len()];
    'faces: for fi in 0..operand.faces.len() {
        if seen[fi] { continue }
        let b = out.as_ref().unwrap_or(operand);
        let Surface::Plane(f) = b.faces[fi].surface else { continue };
        // the solid's planes parallel to it and within reach, by how far
        let near: Vec<(f64,&Face)> = planes.iter().filter(|(_,g)| parallel(g.z,f.z))
            .map(|&(face,g)| (dot(sub(g.o,f.o),f.z),face)).filter(|(d,_)| d.abs() <= reach).collect();
        if near.is_empty() { continue }
        // every face of the operand on this plane moves with it
        let group: Vec<usize> = (0..b.faces.len()).filter(|&j| !b.faces[j].loops.is_empty()
            && matches!(b.faces[j].surface,Surface::Plane(g) if parallel(g.z,f.z))
            && super::ssi::same(&b.faces[fi].surface,&b.faces[j].surface,tol)).collect();
        let mut bx = EMPTY;
        for &j in &group { seen[j] = true; widen(&mut bx,face_box(b,&b.faces[j])); }
        // the nearest of those whose face comes within reach of the group's
        let mut nearest: Option<f64> = None;
        for (d,face) in near {
            if !overlap(&bx,&face_box(solid,face),reach) { continue }
            if d.abs() <= tol { continue 'faces }
            if nearest.is_none_or(|n| d.abs() < n.abs()) { nearest = Some(d); }
        }
        let Some(d) = nearest else { continue };
        if let Some(moved) = moved_along(b,&group,scale(f.z,d),tol) { out = Some(moved); }
    }
    out
}

/// A face's box: its edges' (`Curve::bounds`, unpadded) — a planar face's own, a face lying in the
/// hull of its boundary.
pub(super) fn face_box(b: &Brep,f: &Face) -> Box3 {
    let mut bx = EMPTY;
    for c in f.loops.iter().flatten() {
        let e = &b.edges[c.edge as usize];
        match &e.curve {
            EdgeCurve::Curve(curve) => widen(&mut bx,curve.bounds(e.t)),
            EdgeCurve::Degenerate => include(&mut bx,b.vertices[e.v[0] as usize].p),
        }
    }
    bx
}

/// `b` with its faces `group` (all on one plane) carried by `delta` along the plane's normal: the
/// plane, the group's edges and their vertices moved, and every other edge ending at one of those
/// vertices — a line along `delta` — lengthened or shortened to it; the faces beside the group,
/// each a surface the move leaves where it was, have their uses of those edges read anew. `None`
/// where a face beside is another surface, an edge reaching the group is no such line or reaches it
/// at both ends, or a line would be left shorter than the tolerance allows.
fn moved_along(b: &Brep,group: &[usize],delta: V,tol: f64) -> Option<Brep> {
    let n = scale(delta,1./norm(delta));
    let own: BTreeSet<u32> = group.iter().flat_map(|&fi| b.faces[fi].loops.iter().flatten().map(|c| c.edge)).collect();
    if own.iter().any(|&e| matches!(b.edges[e as usize].curve,EdgeCurve::Degenerate)) { return None }
    let corners: BTreeSet<u32> = own.iter().flat_map(|&e| b.edges[e as usize].v).collect();
    // the edges reaching the group from elsewhere: which of their ends does, and its parameter
    // once moved
    let mut stretched: BTreeMap<u32,(usize,f64)> = BTreeMap::new();
    for (ei,e) in b.edges.iter().enumerate() {
        if own.contains(&(ei as u32)) { continue }
        let k = match e.v.map(|v| corners.contains(&v)) {
            [false,false] => continue,
            [true,true] => return None,
            [at_start,_] => usize::from(!at_start),
        };
        let &EdgeCurve::Curve(Curve::Line {p,d}) = &e.curve else { return None };
        if norm(cross(d,n)) > ALONG { return None }
        let mut t = e.t;
        t[k] = dot(sub(add(b.vertices[e.v[k] as usize].p,delta),p),d);
        if !(t[1]-t[0] > super::tolerant::ON_CURVE*tol) { return None }
        stretched.insert(ei as u32,(k,t[k]));
    }
    // each face beside the group: how its parameters move with a point moved by `delta`, where the
    // move leaves its surface where it was
    let mut beside: Vec<(usize,[f64;2])> = Vec::new();
    for (fi,f) in b.faces.iter().enumerate() {
        if group.contains(&fi) || !f.loops.iter().flatten().any(|c| own.contains(&c.edge) || stretched.contains_key(&c.edge)) { continue }
        let shift = match f.surface {
            Surface::Plane(g) if dot(g.z,n).abs() <= ALONG => [dot(delta,g.x),dot(delta,g.y)],
            Surface::Cylinder(g,_) | Surface::Extrusion(g,_) if norm(cross(g.z,n)) <= ALONG => [0.,dot(delta,g.z)],
            _ => return None,
        };
        beside.push((fi,shift));
    }
    let m = Rigid {t:delta,..Rigid::identity()};
    let mut out = b.clone();
    for &fi in group { out.faces[fi].surface = b.faces[fi].surface.moved(&m); }
    for &e in &own {
        if let EdgeCurve::Curve(c) = &b.edges[e as usize].curve { out.edges[e as usize].curve = EdgeCurve::Curve(c.moved(&m)); }
    }
    for &v in &corners { out.vertices[v as usize].p = add(b.vertices[v as usize].p,delta); }
    for (&e,&(k,t)) in &stretched { out.edges[e as usize].t[k] = t; }
    // the uses beside: a moved edge's pcurve moved whole, a stretched one's at the end that moved —
    // a pcurve of its own read at its old ends first, since the edge's parameters are not what they
    // were (a line along a plane, a cylinder or an extrusion along the move is straight in its
    // parameters, and a moved curve is the same curve shifted)
    for (fi,shift) in beside {
        let surface = &b.faces[fi].surface;
        for c in out.faces[fi].loops.iter_mut().flatten() {
            let moving = if own.contains(&c.edge) { [true,true] }
                else if let Some(&(k,_)) = stretched.get(&c.edge) { [k == 0,k == 1] }
                else { continue };
            let old = &b.edges[c.edge as usize];
            let [a,z] = [0,1].map(|k| {
                let q = c.pcurve.at(old.t[k],old,surface,&b.vertices);
                if moving[k] { [q[0]+shift[0],q[1]+shift[1]] } else { q }
            });
            c.pcurve = match c.pcurve {
                Pcurve::Inverse {..} => Pcurve::Inverse {a,b:z},
                Pcurve::Curve(_) if own.contains(&c.edge) => Pcurve::Inverse {a,b:z},
                _ => Pcurve::Line {a,b:z},
            };
        }
    }
    Some(out)
}

/// **A tube all but tangent to another at a seam is on it** (#996): the solid and the operand of
/// a body's Boolean, where a cylinder of one ends in a circle concentric with and in the plane of a
/// circle of the other — a leg standing on a turn's cap, the turn's tube a hair wider or narrower
/// than the leg — whose radii differ at all but by no more than `FLUSH` of the Boolean's
/// tolerances (a part in a million of their size, as an end all but flush with a plane is,
/// `flushed`), with that cylinder moved onto the other's radius: exactly, the cylinder's surface, its circles
/// and its seam carried out or in square to its axis, its ends' planes left where they are. The
/// operand's cylinders first, onto any circle of the solid's; then the solid's, onto a circle of
/// the operand's that is no cylinder's (a turn's rim). Between the tolerance and that reach the
/// arrangement's readings of the seam — a point onto a vertex within the tolerance, a vertex onto
/// an edge and an edge onto an edge within eight of it, a strip's inside, a pole's parameter —
/// part the two circles in one place and take them for one in another, and the solid came out
/// refused one way or another for one near-miss in three; moved onto it, the seam is the tangent
/// one, which every reading agrees on. A cylinder whose ends meet anything but planes square to
/// its axis, or whose circles ask for two radii, is left as drawn. `None` for each left whole.
pub fn tubes_flushed(solid: &Brep,operand: &Brep,tol: f64) -> (Option<Brep>,Option<Brep>) {
    let reach = FLUSH*tol;
    // each circle edge of a boundary: its centre, its normal, its radius, and whether a cylinder
    // of the boundary is bounded by it
    let circles = |b: &Brep| -> Vec<(V,V,f64,bool)> {
        b.edges.iter().enumerate().filter_map(|(ei,e)| match &e.curve {
            EdgeCurve::Curve(Curve::Circle(f,r)) => {
                let on_tube = b.faces.iter().any(|g| matches!(g.surface,Surface::Cylinder(..))
                    && g.loops.iter().flatten().any(|c| c.edge as usize == ei));
                Some((f.o,f.z,r.abs(),on_tube))
            }
            _ => None,
        }).collect()
    };
    // the radius a cylinder face of `b` is asked to take by `others`' circles, where every ask agrees
    let asked = |b: &Brep,fi: usize,others: &[(V,V,f64,bool)],tubes_too: bool| -> Option<f64> {
        let Surface::Cylinder(g,r) = b.faces[fi].surface else { return None };
        let mut want: Option<f64> = None;
        for c in b.faces[fi].loops.iter().flatten() {
            let EdgeCurve::Curve(Curve::Circle(f,_)) = &b.edges[c.edge as usize].curve else { continue };
            for &(o,n,rr,tube) in others {
                if tube && !tubes_too { continue }
                if norm(cross(n,g.z)) > ALONG || distance(o,f.o) > tol { continue }
                // (two circles of one radius to the last bits are one already; one a hair apart,
                // however little, has poles and crossings a hair apart that the readings split)
                let d = (rr-r.abs()).abs();
                if !(d > 1e-12*rr && d <= reach) { continue }
                match want {
                    None => want = Some(rr),
                    Some(w) if (w-rr).abs() <= tol => {}
                    Some(_) => return None,
                }
            }
        }
        want
    };
    let mut moved_operand: Option<Brep> = None;
    let theirs = circles(solid);
    for fi in 0..operand.faces.len() {
        let b = moved_operand.as_ref().unwrap_or(operand);
        let Some(r) = asked(b,fi,&theirs,true) else { continue };
        if let Some(m) = reradiused(b,fi,r,tol) { moved_operand = Some(m); }
    }
    let mut moved_solid: Option<Brep> = None;
    let theirs = circles(moved_operand.as_ref().unwrap_or(operand));
    for fi in 0..solid.faces.len() {
        let b = moved_solid.as_ref().unwrap_or(solid);
        let Some(r) = asked(b,fi,&theirs,false) else { continue };
        if let Some(m) = reradiused(b,fi,r,tol) { moved_solid = Some(m); }
    }
    (moved_solid,moved_operand)
}

/// `b` with its cylinder face `fi` of radius `r`: the surface, its circles about its axis and its
/// seam lines along it carried square to the axis, their vertices with them, the planes square to
/// the axis its edges bound read anew; `None` where an edge of the face is anything else, a face
/// beside it is no such plane, or another edge reaches one of its vertices.
fn reradiused(b: &Brep,fi: usize,r: f64,tol: f64) -> Option<Brep> {
    let Surface::Cylinder(g,old) = b.faces[fi].surface else { return None };
    let k = r/old.abs();
    let on_axis = |p: V| add(g.o,scale(g.z,dot(sub(p,g.o),g.z)));
    let carried = |p: V| { let a = on_axis(p); add(a,scale(sub(p,a),k)) };
    let own: BTreeSet<u32> = b.faces[fi].loops.iter().flatten().map(|c| c.edge).collect();
    for &e in &own {
        match &b.edges[e as usize].curve {
            EdgeCurve::Curve(Curve::Circle(f,_)) if norm(cross(f.z,g.z)) <= ALONG && distance(on_axis(f.o),f.o) <= tol => {}
            EdgeCurve::Curve(Curve::Line {d,..}) if norm(cross(*d,g.z)) <= ALONG*norm(*d) => {}
            _ => return None,
        }
    }
    let corners: BTreeSet<u32> = own.iter().flat_map(|&e| b.edges[e as usize].v).collect();
    if b.edges.iter().enumerate().any(|(ei,e)| !own.contains(&(ei as u32)) && e.v.iter().any(|v| corners.contains(v))) { return None }
    let beside: Vec<usize> = (0..b.faces.len()).filter(|&h| h != fi && b.faces[h].loops.iter().flatten().any(|c| own.contains(&c.edge))).collect();
    for &h in &beside {
        match b.faces[h].surface {
            Surface::Plane(p) if norm(cross(p.z,g.z)) <= ALONG => {}
            _ => return None,
        }
    }
    let mut out = b.clone();
    out.faces[fi].surface = Surface::Cylinder(g,old.signum()*r);
    for &e in &own {
        let edge = &mut out.edges[e as usize];
        edge.curve = match &b.edges[e as usize].curve {
            EdgeCurve::Curve(Curve::Circle(f,rr)) => EdgeCurve::Curve(Curve::Circle(*f,rr*k)),
            EdgeCurve::Curve(Curve::Line {p,d}) => EdgeCurve::Curve(Curve::Line {p:carried(*p),d:*d}),
            other => other.clone(),
        };
    }
    for &v in &corners { out.vertices[v as usize].p = carried(b.vertices[v as usize].p); }
    // each plane's uses of the edges carried: the plane's own coordinates of their ends
    for &h in &beside {
        let Surface::Plane(p) = out.faces[h].surface else { continue };
        let uv = |x: V| { let l = p.local(x); [l[0],l[1]] };
        let edges = out.edges.clone();
        let vertices = out.vertices.clone();
        for c in out.faces[h].loops.iter_mut().flatten() {
            if !own.contains(&c.edge) { continue }
            let e = &edges[c.edge as usize];
            let ends = [0,1].map(|j| uv(e.point(e.t[j],&vertices)));
            c.pcurve = Pcurve::Inverse {a:ends[0],b:ends[1]};
        }
    }
    Some(out)
}
