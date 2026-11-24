//! **Faces deleted** (brep-bench's `edit/` cases; OCCT's `BRepAlgoAPI_Defeaturing` studied for
//! what is asked, not how): faces taken out of a solid and the gap they leave closed by the faces
//! round them, each run on along its own surface until they meet.
//!
//! **As a set.** The faces round the deleted ones — every face sharing an edge with one — are
//! each run on along its surface: a sheet of that surface over its own face and past it, as far as
//! the solid reaches and half as far again (`sheet`). The solid and those sheets part space into
//! cells (`boolean::arrangement`), each inside the solid or out. A **gap** is a bounded cell bounded
//! by nothing but pieces of deleted faces and of the sheets run on past their faces — no piece of a
//! face the solid keeps — and by at least one deleted face: what the deleted faces and the faces
//! round them, run on, close off. The result is every cell the solid holds, each gap turned over —
//! a gap the solid held (a round in a concave corner, a boss's root) given up, a gap outside it (a
//! round on a convex edge, a hole) taken in — and its boundary is the selection's
//! (`boolean::selection`). Nothing is classified by shape: a convex round and a concave one, a hole
//! and a boss, a bevel and a flat are each the gap their faces close, and the sheets' pieces
//! running on through the material or out into the air bound nothing kept and go.
//!
//! **The faces round them made whole.** A sheet is its face's own surface over a box of its own
//! parameters holding the face, so each of its pieces is a piece of that face and is joined to it
//! where they share an edge (the selection's healing, read for the sheet's pieces as for the face's
//! own): a top run on over a round's corner is one face again, a side run on across a notch one
//! rectangle. Faces round the deleted ones on one surface (a side a notch parted, a cone a groove
//! parted) share one sheet and are one face again where the gap joins them.
//!
//! **Refused** where nothing closes: a deleted face no gap holds — the faces round it, run on as
//! far as the sheets reach, do not meet past it — is named, as is a face round them whose surface
//! is not one this module runs on (a plane, a cylinder, a cone, a sphere, a torus).
use std::collections::{BTreeMap,BTreeSet};

use super::boolean::{arrangement,face_parameters,members,selection};
use super::complex::Origin;
use super::geom::{Curve,Frame,Surface,Uv};
use super::topo::{reverse_loop,Brep,Coedge,EdgeCurve,Face,Pcurve};
use super::{KernelError,Kind};
use crate::util::space::{add,cross,dot,scale,sub};
use std::f64::consts::{FRAC_PI_2,TAU};

/// **`solid` with `faces` deleted** (the module's account), to `tol`.
pub fn deleted(solid: &Brep,faces: &[usize],tol: f64) -> Result<Brep,KernelError> {
    let failed = |m: String| KernelError::new(Kind::Failed,m);
    let gone: BTreeSet<usize> = faces.iter().copied().collect();
    if gone.is_empty() { return Ok(solid.clone()) }
    if let Some(&f) = gone.iter().find(|&&f| f >= solid.faces.len()) {
        return Err(failed(format!("there is no face {f} to delete: the solid has {}",solid.faces.len())))
    }
    if gone.len() == solid.faces.len() { return Err(failed("deleting every face of a solid leaves nothing to close the gap".into())) }
    // the faces round the deleted ones: each sharing an edge with one
    let uses = solid.uses();
    let mut round: BTreeSet<usize> = BTreeSet::new();
    for u in &uses {
        if !u.iter().any(|&(f,..)| gone.contains(&f)) { continue }
        round.extend(u.iter().map(|&(f,..)| f).filter(|f| !gone.contains(f)));
    }
    if round.is_empty() { return Err(failed("the faces deleted share no edge with a face kept: nothing runs on to close the gap".into())) }
    // one sheet for each surface they lie on, the faces on it its class
    let mut classes: Vec<Vec<usize>> = Vec::new();
    for &f in &round {
        let face = &solid.faces[f];
        match classes.iter_mut().find(|c| { let g = &solid.faces[c[0]]; g.surface == face.surface && g.reversed == face.reversed }) {
            Some(c) => c.push(f),
            None => classes.push(vec![f]),
        }
    }
    let size = solid.size();
    let (lo,hi) = solid.bounds();
    let grow = size/2.;
    let reach: Vec<[f64;3]> = (0..8).map(|k| [0,1,2].map(|i| if k>>i & 1 == 1 { hi[i]+grow } else { lo[i]-grow })).collect();
    let mut sheets: Vec<Brep> = Vec::new();
    for class in &classes {
        let face = &solid.faces[class[0]];
        let own: Vec<[[f64;2];2]> = class.iter().map(|&f| face_parameters(solid,f)).collect();
        let [u,v] = run_on(&face.surface,&own,&reach,size).map_err(|m| failed(format!("{} round the faces deleted: {m}",named(face))))?;
        sheets.push(sheet(&face.surface,face.reversed,u,v).map_err(|m| failed(format!("{} run on: {m}",named(face))))?);
    }
    let mut operands: Vec<&Brep> = vec![solid];
    operands.extend(sheets.iter());
    let mut complex = arrangement(&operands,tol).map_err(failed)?;
    let inside = members(&complex,&[solid],tol).map_err(failed)?;
    let cells = complex.cells().map_err(failed)?.to_vec();
    // each face of the complex: a piece of a face kept, of a face deleted, or of a sheet alone
    #[derive(Clone,Copy,PartialEq)]
    enum Piece { Kept,Round,Deleted,Sheet }
    let piece: Vec<Piece> = complex.origins.iter().map(|o| {
        if o.iter().any(|o| o.operand == 0 && !gone.contains(&o.face) && !round.contains(&o.face)) { Piece::Kept }
        else if o.iter().any(|o| o.operand == 0 && round.contains(&o.face)) { Piece::Round }
        else if o.iter().any(|o| o.operand == 0) { Piece::Deleted }
        else { Piece::Sheet }
    }).collect();
    let mut gap = vec![false;cells.len()];
    for c in 1..cells.len() {
        let sides: Vec<(usize,bool)> = cells[c].shells.iter().flat_map(|&k| complex.shells[k].iter().copied()).collect();
        let fin = |f: usize| sides.contains(&(f,true)) && sides.contains(&(f,false));
        let bounding: Vec<usize> = sides.iter().map(|&(f,_)| f).filter(|&f| !fin(f)).collect();
        gap[c] = bounding.iter().any(|&f| piece[f] == Piece::Deleted) && bounding.iter().any(|&f| piece[f] == Piece::Sheet)
            && bounding.iter().all(|&f| piece[f] != Piece::Kept);
    }
    if crate::debug() {
        for c in 1..cells.len() { eprintln!("deleted: cell {c}, volume {}, inside {}, gap {}",cells[c].volume,inside[c][0],gap[c]) }
    }
    let keep: Vec<bool> = (0..cells.len()).map(|c| c != 0 && inside[c][0] != gap[c]).collect();
    // a deleted face left between a cell kept and one not is one no gap closed
    for (f,p) in piece.iter().enumerate() {
        if *p != Piece::Deleted { continue }
        let [a,b] = complex.sides(f).map_err(failed)?;
        if keep[a] != keep[b] {
            let o = complex.origins[f].iter().find(|o| o.operand == 0).map_or(0,|o| o.face);
            return Err(failed(format!("{} closes no gap: the faces round it, run on, do not meet past it",named(&solid.faces[o]))))
        }
    }
    // each sheet's piece a piece of its class's first face, and each face of a class one with it:
    // an edge between two pieces of one face, either a sheet's, is a meeting healed out where both
    // are kept
    let of_class: BTreeMap<usize,usize> = classes.iter().flat_map(|c| c.iter().map(move |&f| (f,c[0]))).collect();
    let pre: Vec<Origin> = complex.origins.iter().map(|o| o[0]).collect();
    for o in complex.origins.iter_mut() {
        let first = o[0];
        let host = if first.operand == 0 { of_class.get(&first.face).copied() } else { Some(classes[first.operand-1][0]) };
        if let Some(h) = host { o[0] = Origin {operand:0,face:h,alike:true}; }
    }
    let mut by_edge: Vec<Vec<(usize,usize,usize)>> = vec![Vec::new();complex.b.edges.len()];
    for (f,face) in complex.b.faces.iter().enumerate() {
        for (l,lp) in face.loops.iter().enumerate() {
            for (k,c) in lp.iter().enumerate() { by_edge[c.edge as usize].push((f,l,k)); }
        }
    }
    if complex.met.len() < complex.b.edges.len() { complex.met.resize(complex.b.edges.len(),false); }
    for (e,u) in by_edge.iter().enumerate() {
        if complex.met[e] { continue }
        let mut by_origin: BTreeMap<(usize,usize),Vec<(usize,usize,usize)>> = BTreeMap::new();
        for &(f,l,k) in u { by_origin.entry((complex.origins[f][0].operand,complex.origins[f][0].face)).or_default().push((f,l,k)); }
        complex.met[e] = by_origin.values().any(|w| {
            let &[(fa,la,ka),(fb,lb,kb)] = &w[..] else { return false };
            if fa == fb || pre[fa] == pre[fb] || complex.b.faces[fa].reversed != complex.b.faces[fb].reversed { return false }
            // the two pieces read the edge at one place of their surface (a seam's two sides do not)
            let edge = &complex.b.edges[e];
            let t = (edge.t[0]+edge.t[1])/2.;
            let at = |f: usize,l: usize,k: usize| {
                let face = &complex.b.faces[f];
                face.loops[l][k].pcurve.at(t,edge,&face.surface,&complex.b.vertices)
            };
            let (p,q) = (at(fa,la,ka),at(fb,lb,kb));
            (p[0]-q[0]).abs().max((p[1]-q[1]).abs()) <= 1e-9*(1.+p[0].abs().max(p[1].abs()))
        });
    }
    let out = selection(&complex,&|c| keep[c]).map_err(failed)?;
    out.check(tol).map_err(|e| e.saying(|m| format!("faces deleted: {m}")))?;
    Ok(out)
}

/// A face as the kernel says it: its name, else its surface's kind.
fn named(f: &Face) -> String {
    if f.name.is_empty() { format!("the {} face",f.surface.kind()) } else { format!("the face `{}` ({})",f.name,f.surface.kind()) }
}

/// The box of `s`'s parameters a sheet running on past faces `own` (each one's box) covers: the
/// faces', and as far as `reach` (points of space) stands from them along the surface — a plane and
/// a cylinder or a cone along their straight parameters over the reach's extent; a closed
/// parameter (a turn, a sphere's latitude, a torus's tube) whole. A cone is run on toward its apex
/// no nearer it than a twentieth of `size`, unless a face reaches nearer.
fn run_on(s: &Surface,own: &[[[f64;2];2]],reach: &[[f64;3]],size: f64) -> Result<[[f64;2];2],String> {
    let mut u = [f64::INFINITY,f64::NEG_INFINITY];
    let mut v = [f64::INFINITY,f64::NEG_INFINITY];
    for b in own {
        u = [u[0].min(b[0][0]),u[1].max(b[0][1])];
        v = [v[0].min(b[1][0]),v[1].max(b[1][1])];
    }
    // a period whole, started where it holds the faces' middle half a turn either side
    let whole = |r: [f64;2]| { let m = (r[0]+r[1])/2.; [m-TAU/2.,m+TAU/2.] };
    let span = |f: &Frame,axis: V,lo: [f64;2]| -> [f64;2] {
        reach.iter().fold(lo,|r,&p| { let x = dot(sub(p,f.o),axis); [r[0].min(x),r[1].max(x)] })
    };
    Ok(match *s {
        Surface::Plane(ref f) => {
            let su = span(f,f.x,u);
            let sv = span(f,f.y,v);
            [su,sv]
        }
        Surface::Cylinder(ref f,_) => [whole(u),span(f,f.z,v)],
        Surface::Cone(ref f,r,a) => {
            let (sa,ca) = (a.sin(),a.cos());
            let h = span(f,f.z,[v[0]*ca,v[1]*ca]);
            let mut sv = [h[0]/ca,h[1]/ca];
            // the apex's side: the faces' radius r + v sin α is positive
            let near = size/20.;
            if sa != 0. {
                let apex = -r/sa;
                if sa > 0. { sv[0] = sv[0].max((apex+near/sa).min(v[0])) } else { sv[1] = sv[1].min((apex+near/sa).max(v[1])) }
            }
            [whole(u),[sv[0].min(v[0]),sv[1].max(v[1])]]
        }
        Surface::Sphere(..) => [whole(u),[-FRAC_PI_2,FRAC_PI_2]],
        Surface::Torus(..) => [whole(u),whole(v)],
        _ => return Err(format!("a {} face is not run on past its edges",s.kind())),
    })
}

type V = [f64;3];

/// **A sheet of `s`** over the box `u × v` of its parameters, its face `reversed` as given: one
/// face bounded by its four parameter lines, a closed parameter's two (a whole turn's) one seam
/// used both ways, a sphere's latitude at a pole a degenerate edge. Every edge's curve is the
/// surface's own parameter line, read at the parameter running along it, and every pcurve the
/// segment it is.
fn sheet(s: &Surface,reversed: bool,u: [f64;2],v: [f64;2]) -> Result<Brep,String> {
    if !(u[1] > u[0] && v[1] > v[0]) { return Err(format!("no sheet over {u:?} × {v:?}")) }
    let periods = s.periods();
    let closed = [0,1].map(|k| periods[k].is_some_and(|p| ((if k == 0 { u[1]-u[0] } else { v[1]-v[0] })-p).abs() <= 1e-12*p));
    let pole = |v: f64| matches!(s,Surface::Sphere(..)) && (v.abs()-FRAC_PI_2).abs() <= 1e-12;
    let mut b = Brep::default();
    // the corners (u0, v0), (u1, v0), (u1, v1), (u0, v1), one where a closed parameter or a pole
    // makes them one
    let c00 = b.vertex(s.point([u[0],v[0]]));
    let c10 = if closed[0] || pole(v[0]) { c00 } else { b.vertex(s.point([u[1],v[0]])) };
    let c01 = if closed[1] { c00 } else { b.vertex(s.point([u[0],v[1]])) };
    let c11 = if closed[1] { c10 } else if closed[0] || pole(v[1]) { c01 } else { b.vertex(s.point([u[1],v[1]])) };
    // the line of constant v (`along_u`) or u, from its first parameter to its last
    let iso = |b: &mut Brep,along_u: bool,at: f64,ends: [u32;2]| -> Result<u32,String> {
        let t = if along_u { u } else { v };
        if along_u && pole(at) { return Ok(b.edge(EdgeCurve::Degenerate,t,ends)) }
        Ok(b.edge(EdgeCurve::Curve(parameter_line(s,along_u,at)?),t,ends))
    };
    let bottom = iso(&mut b,true,v[0],[c00,c10])?;
    let right = iso(&mut b,false,u[1],[c10,c11])?;
    let top = if closed[1] { bottom } else { iso(&mut b,true,v[1],[c01,c11])? };
    let left = if closed[0] { right } else { iso(&mut b,false,u[0],[c00,c01])? };
    let line = |a: Uv,z: Uv| Pcurve::Line {a,b:z};
    let mut l = vec![
        Coedge {edge:bottom,reversed:false,pcurve:line([u[0],v[0]],[u[1],v[0]])},
        Coedge {edge:right,reversed:false,pcurve:line([u[1],v[0]],[u[1],v[1]])},
        Coedge {edge:top,reversed:true,pcurve:line([u[0],v[1]],[u[1],v[1]])},
        Coedge {edge:left,reversed:true,pcurve:line([u[0],v[0]],[u[0],v[1]])},
    ];
    // counter-clockwise in the parameters as built: the face's material normal the surface's;
    // walked the other way for a reversed face
    if reversed { l = reverse_loop(l); }
    b.faces.push(Face {surface:s.clone(),reversed,loops:vec![l],name:String::new()});
    Ok(b)
}

/// The line of `s`'s parameters at `at` — of constant `v` where `along_u`, read at `u`, else of
/// constant `u`, read at `v` — as a curve whose parameter is that one.
fn parameter_line(s: &Surface,along_u: bool,at: f64) -> Result<Curve,String> {
    let round = |f: &Frame,u: f64| add(scale(f.x,u.cos()),scale(f.y,u.sin()));
    let meridian = |o: V,e: V,z: V,r: f64| Curve::Circle(Frame {o,x:e,y:z,z:cross(e,z)},r);
    Ok(match (s,along_u) {
        (Surface::Plane(f),true) => Curve::Line {p:add(f.o,scale(f.y,at)),d:f.x},
        (Surface::Plane(f),false) => Curve::Line {p:add(f.o,scale(f.x,at)),d:f.y},
        (Surface::Cylinder(f,r),true) => Curve::Circle(Frame {o:add(f.o,scale(f.z,at)),..*f},*r),
        (Surface::Cylinder(f,r),false) => Curve::Line {p:add(f.o,scale(round(f,at),*r)),d:f.z},
        (&Surface::Cone(ref f,r,a),true) => {
            let rho = r+at*a.sin();
            if !(rho > 0.) { return Err(format!("a cone's parallel at {at} has no radius")) }
            Curve::Circle(Frame {o:add(f.o,scale(f.z,at*a.cos())),..*f},rho)
        }
        (&Surface::Cone(ref f,r,a),false) => {
            let e = round(f,at);
            Curve::Line {p:add(f.o,scale(e,r)),d:add(scale(e,a.sin()),scale(f.z,a.cos()))}
        }
        (&Surface::Sphere(ref f,r),true) => Curve::Circle(Frame {o:add(f.o,scale(f.z,r*at.sin())),..*f},r*at.cos()),
        (&Surface::Sphere(ref f,r),false) => meridian(f.o,round(f,at),f.z,r),
        (&Surface::Torus(ref f,big,r),true) => Curve::Circle(Frame {o:add(f.o,scale(f.z,r*at.sin())),..*f},big+r*at.cos()),
        (&Surface::Torus(ref f,big,r),false) => { let e = round(f,at); meridian(add(f.o,scale(e,big)),e,f.z,r) }
        _ => return Err(format!("a {} surface's parameter lines are not drawn",s.kind())),
    })
}
