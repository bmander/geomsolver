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
use crate::util::space::{add,cross,dot,norm,scale,sub};
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

/// A face's box: its edges'.
fn face_box(b: &Brep,f: &Face) -> Box3 {
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
