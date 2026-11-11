//! **The convex hull of points, as a solid** (`hull`): the least convex set holding them, bounded
//! by one plane face for each of its facets, built with its topology whole and without a Boolean.
//!
//! The hull is found first as triangles, exactly: by quickhull (Barber, Dobkin and Huhdanpaa,
//! 1996), each point a triangle sees placed beyond it on the exact orientation test
//! (`util::predicates::orient`), so whether a point is past a triangle, on its plane or beneath
//! it is never a rounding's to say, and the triangles close round a convex surface whatever lies
//! on one plane. A point is seen only from strictly beyond: one on a triangle's plane is no corner.
//!
//! The triangles are then made faces at the kernel's tolerance (`geom::tolerance` of the points'
//! box): a face grows from its largest triangle across each edge to a neighbour whose corners
//! all lie within the tolerance of the first one's plane (measured from that plane, so that a
//! run of neighbours each nearly level with the next never bends a face), and only while it stays
//! a disc, its rim one cycle; a corner within the tolerance of the line through its neighbours on
//! the edge between two faces is dropped from both (a point midway along an edge of the hull is
//! no corner of it). Points within the tolerance of one kept are merged into it
//! (`tolerant::Pool`, the first given kept). Each face is the plane of its largest triangle, its
//! edges lines between its corners.
use super::on_plane;
use crate::geom::{tolerance,Curve,Frame,Surface,V};
use crate::tolerant::Pool;
use crate::topo::{Brep,Coedge,EdgeCurve,Face};
use crate::util::predicates::orient;
use crate::util::space::{cross,distance,dot,norm,segment_distance,sub,unit};
use std::collections::{BTreeMap,BTreeSet};

/// A triangle of the hull: its corners counter-clockwise seen from outside, the triangle across
/// each edge (edge `k` from corner `k` to corner `k + 1`), and the points beyond it not yet placed.
struct Tri { v: [usize;3],across: [usize;3],beyond: Vec<usize>,live: bool }

/// A face of the hull: the plane of its largest triangle (a point on it and its unit outward
/// normal), and its corners counter-clockwise seen from outside.
struct Facet { plane: (V,V),corners: Vec<usize> }

/// **The convex hull of `points`**, a solid of plane faces (the module's rules). Refused where
/// there are no four points standing apart from one plane by more than the tolerance, or where a
/// point is not finite (or lies past 10¹², as `tolerant::Pool` holds).
pub fn hull(points: &[V]) -> Result<Brep,String> {
    if points.len() < 4 { return Err(format!("a hull of {} points: a solid needs four",points.len())) }
    if points.iter().flatten().any(|x| !(x.abs() < 1e12)) { return Err("a hull of a point not finite".into()) }
    let (lo,hi) = crate::geom::hull(points.iter().copied());
    let tol = tolerance(distance(lo,hi));
    let mut pool = Pool::new(tol);
    for &p in points { pool.at(p); }
    let pts = pool.pts;
    let tris = triangles(&pts,tol)?;
    let mut faces = faces(&pts,&tris,tol)?;
    drop_straight(&pts,&mut faces,tol);
    solid(&pts,&faces,tol)
}

/// The normal of `t`'s plane, twice its area long.
fn normal(pts: &[V],t: &Tri) -> V { cross(sub(pts[t.v[1]],pts[t.v[0]]),sub(pts[t.v[2]],pts[t.v[0]])) }

/// Whether point `p` lies strictly beyond the plane of `t`.
fn sees(pts: &[V],t: &Tri,p: usize) -> bool { orient(pts[t.v[0]],pts[t.v[1]],pts[t.v[2]],pts[p]) > 0 }

/// The hull of `pts` as triangles, by quickhull on the exact orientation test: those live are
/// its boundary, each `across` naming its neighbours.
fn triangles(pts: &[V],tol: f64) -> Result<Vec<Tri>,String> {
    // the first tetrahedron: the point least in x, the farthest from it, the farthest from the
    // line through both, the farthest from the plane through the three (the first given of
    // those as far)
    let farthest = |key: &dyn Fn(V) -> f64| (0..pts.len()).map(|i| (key(pts[i]),i))
        .fold((f64::NEG_INFINITY,0),|best,c| if c.0 > best.0 { c } else { best });
    let (_,a) = farthest(&|p| -p[0]);
    let (d,b) = farthest(&|p| distance(p,pts[a]));
    if d <= tol { return Err("a hull of points that are one point".into()) }
    let along = unit(sub(pts[b],pts[a]));
    let (d,c) = farthest(&|p| norm(cross(sub(p,pts[a]),along)));
    if d <= tol { return Err("a hull of points on one line: a solid needs four standing off a plane".into()) }
    let up = unit(cross(sub(pts[b],pts[a]),sub(pts[c],pts[a])));
    let (d,e) = farthest(&|p| dot(sub(p,pts[a]),up).abs());
    if d <= tol { return Err("a hull of points on one plane: a solid needs four standing off it".into()) }
    // its faces turned so that the fourth corner of each lies beneath it, each meeting the other
    // three across its edges
    let (b,c) = if orient(pts[a],pts[b],pts[c],pts[e]) > 0 { (c,b) } else { (b,c) };
    let corners = [[a,b,c],[a,e,b],[b,e,c],[c,e,a]];
    let mut tris: Vec<Tri> = corners.iter().map(|&v| Tri {v,across:[0;3],beyond:Vec::new(),live:true}).collect();
    for t in 0..4 {
        for k in 0..3 {
            let (p,q) = (tris[t].v[k],tris[t].v[(k+1)%3]);
            tris[t].across[k] = (0..4).find(|&u| (0..3).any(|j| tris[u].v[j] == q && tris[u].v[(j+1)%3] == p)).unwrap();
        }
    }
    // each other point placed beyond the first face that sees it
    for i in (0..pts.len()).filter(|i| ![a,b,c,e].contains(i)) {
        if let Some(t) = (0..4).find(|&t| sees(pts,&tris[t],i)) { tris[t].beyond.push(i); }
    }
    for next in 0.. {
        if next == tris.len() { break }
        if !tris[next].live || tris[next].beyond.is_empty() { continue }
        // the point farthest beyond the face, the first given of those as far
        let (t,n) = (&tris[next],normal(pts,&tris[next]));
        let p = t.beyond.iter().copied().map(|i| (dot(sub(pts[i],pts[t.v[0]]),n),i))
            .fold((f64::NEG_INFINITY,usize::MAX),|best,c| if c.0 > best.0 || (c.0 == best.0 && c.1 < best.1) { c } else { best }).1;
        add_point(pts,&mut tris,next,p)?;
    }
    Ok(tris)
}

/// The hull grown to take in point `p`, which face `from` sees: every face it sees taken off, and
/// the rim they leave joined to `p`.
fn add_point(pts: &[V],tris: &mut Vec<Tri>,from: usize,p: usize) -> Result<(),String> {
    // the faces `p` sees, a connected cap of the hull reached from `from`
    let mut seen = vec![from];
    let mut sees_p = BTreeMap::from([(from,true)]);
    let mut k = 0;
    while k < seen.len() {
        for u in tris[seen[k]].across {
            if sees_p.contains_key(&u) { continue }
            let s = sees(pts,&tris[u],p);
            sees_p.insert(u,s);
            if s { seen.push(u); }
        }
        k += 1;
    }
    // the rim: each edge of a seen face whose neighbour is not seen, joined to `p` — the new
    // face meeting the one beyond the rim across the rim's edge
    let mut new = Vec::new();
    let mut from_start = BTreeMap::new();
    for &t in &seen {
        for k in 0..3 {
            let outside = tris[t].across[k];
            if sees_p[&outside] { continue }
            let (a,b) = (tris[t].v[k],tris[t].v[(k+1)%3]);
            let id = tris.len();
            tris.push(Tri {v:[a,b,p],across:[outside,0,0],beyond:Vec::new(),live:true});
            let o = &mut tris[outside];
            let j = (0..3).find(|&j| o.v[j] == b && o.v[(j+1)%3] == a).ok_or("a hull whose rim does not close")?;
            o.across[j] = id;
            if from_start.insert(a,id).is_some() { return Err("a hull whose rim is no cycle".into()) }
            new.push(id);
        }
    }
    // the new faces meet one another across their edges to `p`: each the next round the rim
    for &t in &new {
        let next = *from_start.get(&tris[t].v[1]).ok_or("a hull whose rim does not close")?;
        tris[t].across[1] = next;
        tris[next].across[2] = t;
    }
    // the points beyond the faces taken off placed beyond a new face that sees them; one no new
    // face sees lies within the grown hull, or on it
    for &t in &seen {
        tris[t].live = false;
        for i in std::mem::take(&mut tris[t].beyond) {
            if let Some(&u) = new.iter().find(|&&u| i != p && sees(pts,&tris[u],i)) { tris[u].beyond.push(i); }
        }
    }
    Ok(())
}

/// The live triangles gathered into faces (the module's rule), each read round its rim.
fn faces(pts: &[V],tris: &[Tri],tol: f64) -> Result<Vec<Facet>,String> {
    let normals: Vec<V> = tris.iter().map(|t| normal(pts,t)).collect();
    let mut order: Vec<usize> = (0..tris.len()).filter(|&t| tris[t].live).collect();
    order.sort_by(|&s,&t| norm(normals[t]).total_cmp(&norm(normals[s])).then(s.cmp(&t)));
    let mut face_of = vec![usize::MAX;tris.len()];
    let mut out = Vec::new();
    for seed in order {
        if face_of[seed] != usize::MAX { continue }
        let id = out.len();
        let (o,n) = (pts[tris[seed].v[0]],unit(normals[seed]));
        face_of[seed] = id;
        let mut members = vec![seed];
        let mut corners: BTreeSet<usize> = tris[seed].v.into_iter().collect();
        let mut k = 0;
        while k < members.len() {
            for u in tris[members[k]].across {
                if face_of[u] != usize::MAX || dot(normals[u],n) <= 0.
                    || tris[u].v.iter().any(|&v| dot(sub(pts[v],o),n).abs() > tol) { continue }
                // kept a disc: a triangle meeting the face along one edge only adds its far
                // corner, which must be new (a triangle across two edges fills a notch)
                let shared = (0..3).filter(|&j| face_of[tris[u].across[j]] == id).count();
                let far = (0..3).find(|&j| face_of[tris[u].across[j]] == id).map(|j| tris[u].v[(j+2)%3]);
                if shared == 1 && far.is_some_and(|v| corners.contains(&v)) { continue }
                face_of[u] = id;
                corners.extend(tris[u].v);
                members.push(u);
            }
            k += 1;
        }
        // its rim: its triangles' edges another face is across, one cycle round a disc
        let mut next = BTreeMap::new();
        for &t in &members {
            for j in (0..3).filter(|&j| face_of[tris[t].across[j]] != id) { next.insert(tris[t].v[j],tris[t].v[(j+1)%3]); }
        }
        let start = *next.keys().next().expect("a face has a rim");
        let mut ring = vec![start];
        while let Some(&v) = next.get(ring.last().unwrap()).filter(|&&v| v != start && ring.len() <= next.len()) { ring.push(v); }
        if ring.len() != next.len() { return Err("a hull's face whose rim is no one cycle".into()) }
        out.push(Facet {plane:(o,n),corners:ring});
    }
    Ok(out)
}

/// Each corner on the edge between two faces only, within `tol` of the line through its
/// neighbours there, dropped from both: no corner of the hull.
fn drop_straight(pts: &[V],faces: &mut [Facet],tol: f64) {
    let mut at: BTreeMap<usize,Vec<usize>> = BTreeMap::new();
    for (f,face) in faces.iter().enumerate() { for &v in &face.corners { at.entry(v).or_default().push(f); } }
    // a corner of two faces has the same neighbours in both; one dropped leaves the others' faces
    // as they were, so each is looked at until none is dropped
    let mut dropped = true;
    while dropped {
        dropped = false;
        for (&v,fs) in at.iter().filter(|(_,fs)| fs.len() == 2) {
            let ring = &faces[fs[0]].corners;
            let Some(k) = ring.iter().position(|&w| w == v) else { continue };
            let m = ring.len();
            if fs.iter().any(|&f| faces[f].corners.len() <= 3)
                || segment_distance(pts[v],pts[ring[(k+m-1)%m]],pts[ring[(k+1)%m]]) > tol { continue }
            for &f in fs { faces[f].corners.retain(|&w| w != v); }
            dropped = true;
        }
    }
}

/// The faces made a solid: a vertex at each corner, a line between corners for each edge, and
/// a plane face for each facet.
fn solid(pts: &[V],faces: &[Facet],tol: f64) -> Result<Brep,String> {
    let mut b = Brep::default();
    let mut vertex = BTreeMap::new();
    let mut edges: BTreeMap<(usize,usize),u32> = BTreeMap::new();
    for face in faces {
        let m = face.corners.len();
        let uses = (0..m).map(|k| {
            let (i,j) = (face.corners[k],face.corners[(k+1)%m]);
            // an edge made along its first use, its second use against it
            let (edge,reversed) = match edges.get(&(i.min(j),i.max(j))) {
                Some(&e) => (e,true),
                None => {
                    let [vi,vj] = [i,j].map(|v| *vertex.entry(v).or_insert_with(|| b.vertex(pts[v])));
                    let d = sub(pts[j],pts[i]);
                    let e = b.edge(EdgeCurve::Curve(Curve::Line {p:pts[i],d:unit(d)}),[0.,norm(d)],[vi,vj]);
                    edges.insert((i.min(j),i.max(j)),e);
                    (e,false)
                }
            };
            Coedge {edge,reversed,pcurve:on_plane()}
        }).collect();
        let (o,n) = face.plane;
        b.faces.push(Face {surface:Surface::Plane(Frame::about(o,n)),reversed:false,loops:vec![uses],name:String::new()});
    }
    b.measure_tolerances();
    b.check(tol).map_err(|e| format!("a hull not closed: {e}"))?;
    Ok(b)
}
