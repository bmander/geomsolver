//! **The convex hull of points, as a solid** (`hull`): the least convex set holding them, bounded
//! by one plane face for each of its facets, built with its topology whole and without a Boolean.
//!
//! The hull is found first as triangles, exactly: by quickhull (Barber, Dobkin and Huhdanpaa,
//! 1996), each point a facet sees placed beyond it on the exact orientation test
//! (`util::predicates::orient`), so whether a point is past a facet, on its plane or beneath it is
//! never a rounding's to say, and the triangles close round a convex surface whatever lies on
//! one plane. A point is seen only from strictly beyond, so a point on the plane of a facet is
//! never placed; one that is past the hull still is.
//!
//! The triangles are then made faces at the kernel's tolerance (`geom::tolerance` of the points'
//! box): a face grows from its largest triangle across each edge to a neighbour whose corners
//! all lie within the tolerance of the first one's plane (measured from that plane, so that a
//! run of neighbours each nearly level with the next never bends a face), and a corner within
//! the tolerance of the line through its neighbours on the edge between two faces is dropped
//! from both (a point midway along an edge of the hull is no corner of it). Points within the
//! tolerance of one kept are dropped. Each face is the plane its corners
//! fit best (Newell's normal through their centroid), its edges lines between its corners.
use super::geom::{tolerance,Curve,Frame,Surface,V};
use super::topo::{Brep,Coedge,EdgeCurve,Face,Pcurve};
use crate::util::predicates::orient;
use crate::util::space::{add,cross,distance,dot,norm,scale,sub,unit};
use std::collections::BTreeMap;

/// A triangle of the hull: its corners counter-clockwise seen from outside, the triangle across
/// each edge (edge `k` from corner `k` to corner `k + 1`), and the points beyond it not yet placed.
struct Tri { v: [usize;3],across: [usize;3],beyond: Vec<usize>,live: bool }

/// **The convex hull of `points`**, a solid of plane faces (the module's rules). Refused where
/// there are no four points standing apart from one plane by more than the tolerance, or where a
/// point is not finite.
pub fn hull(points: &[V]) -> Result<Brep,String> {
    if points.iter().flatten().any(|x| !x.is_finite()) { return Err("a hull of a point not finite".into()) }
    let (lo,hi) = super::geom::hull(points.iter().copied());
    if points.len() < 4 { return Err(format!("a hull of {} points: a solid needs four",points.len())) }
    let tol = tolerance(distance(lo,hi));
    let pts = apart(points,tol);
    let tris = triangles(&pts,tol)?;
    let mut faces = faces(&pts,&tris,tol);
    drop_straight(&pts,&mut faces,tol);
    solid(&pts,&faces,&tris,tol)
}

/// The points less each within `tol` of one kept before it, met in order of x (the first given
/// of those level in x first).
fn apart(points: &[V],tol: f64) -> Vec<V> {
    let mut order: Vec<usize> = (0..points.len()).collect();
    order.sort_by(|&i,&j| points[i][0].total_cmp(&points[j][0]).then(i.cmp(&j)));
    // swept along x: a point is near only those within `tol` of it in x
    let mut kept: Vec<usize> = Vec::new();
    let mut window: Vec<usize> = Vec::new();
    for i in order {
        window.retain(|&j| points[i][0]-points[j][0] <= tol);
        if window.iter().any(|&j| distance(points[i],points[j]) <= tol) { continue }
        window.push(i);
        kept.push(i);
    }
    kept.sort();
    kept.into_iter().map(|i| points[i]).collect()
}

/// The hull of `pts` as triangles, by quickhull on the exact orientation test: those live are
/// its boundary, each `across` naming its neighbours.
fn triangles(pts: &[V],tol: f64) -> Result<Vec<Tri>,String> {
    let n = pts.len();
    // the first tetrahedron: the point least in x, the farthest from it, the farthest from the
    // line through both, the farthest from the plane through the three
    let farthest = |key: &dyn Fn(V) -> f64| (0..n).map(|i| (key(pts[i]),i))
        .fold((f64::NEG_INFINITY,0),|best,c| if c.0 > best.0 { c } else { best });
    let a = (0..n).fold(0,|best,i| if pts[i][0] < pts[best][0] { i } else { best });
    let (d,b) = farthest(&|p| distance(p,pts[a]));
    if d <= tol { return Err("a hull of points that are one point".into()) }
    let along = unit(sub(pts[b],pts[a]));
    let (d,c) = farthest(&|p| { let q = sub(p,pts[a]); norm(sub(q,scale(along,dot(q,along)))) });
    if d <= tol { return Err("a hull of points on one line: a solid needs four standing off a plane".into()) }
    let up = unit(cross(sub(pts[b],pts[a]),sub(pts[c],pts[a])));
    let (d,e) = farthest(&|p| dot(sub(p,pts[a]),up).abs());
    if d <= tol { return Err("a hull of points on one plane: a solid needs four standing off it".into()) }
    // its faces turned so that the fourth corner of each lies beneath it
    let (b,c) = if orient(pts[a],pts[b],pts[c],pts[e]) > 0 { (c,b) } else { (b,c) };
    let corners = [[a,b,c],[a,e,b],[b,e,c],[c,e,a]];
    let mut tris: Vec<Tri> = corners.iter().map(|&v| Tri {v,across:[0;3],beyond:Vec::new(),live:true}).collect();
    link(&mut tris,&[0,1,2,3]);
    // each other point placed beyond the first face that sees it
    for i in 0..n {
        if [a,b,c,e].contains(&i) { continue }
        if let Some(t) = (0..4).find(|&t| sees(pts,&tris[t],i)) { tris[t].beyond.push(i); }
    }
    let mut next = 0;
    while next < tris.len() {
        if !tris[next].live || tris[next].beyond.is_empty() { next += 1; continue }
        // the point farthest beyond the face, the first given of those as far
        let t = &tris[next];
        let normal = cross(sub(pts[t.v[1]],pts[t.v[0]]),sub(pts[t.v[2]],pts[t.v[0]]));
        let p = t.beyond.iter().copied().map(|i| (dot(sub(pts[i],pts[t.v[0]]),normal),i))
            .fold((f64::NEG_INFINITY,usize::MAX),|best,c| if c.0 > best.0 || (c.0 == best.0 && c.1 < best.1) { c } else { best }).1;
        // a point placed again on a face passed over is come back for
        if let Some(back) = add_point(pts,&mut tris,next,p)? { next = next.min(back); }
    }
    Ok(tris)
}

/// Whether point `p` lies strictly beyond the plane of `t`.
fn sees(pts: &[V],t: &Tri,p: usize) -> bool { orient(pts[t.v[0]],pts[t.v[1]],pts[t.v[2]],pts[p]) > 0 }

/// `across` set between the triangles `new`, matched by their shared edges.
fn link(tris: &mut [Tri],new: &[usize]) {
    let mut by_edge = BTreeMap::new();
    for &t in new { for k in 0..3 { by_edge.insert((tris[t].v[k],tris[t].v[(k+1)%3]),t); } }
    for &t in new {
        for k in 0..3 {
            if let Some(&u) = by_edge.get(&(tris[t].v[(k+1)%3],tris[t].v[k])) { tris[t].across[k] = u; }
        }
    }
}

/// The hull grown to take in point `p`, which face `from` sees: every face it sees taken off, and
/// the rim they leave joined to `p`. Where a point is placed beyond a face made before the new
/// ones, that face.
fn add_point(pts: &[V],tris: &mut Vec<Tri>,from: usize,p: usize) -> Result<Option<usize>,String> {
    // the faces `p` sees, a connected cap of the hull reached from `from`
    let mut seen = vec![from];
    let mut is_seen = BTreeMap::from([(from,true)]);
    let mut k = 0;
    while k < seen.len() {
        let t = seen[k];
        for u in tris[t].across {
            if is_seen.contains_key(&u) { continue }
            let s = sees(pts,&tris[u],p);
            is_seen.insert(u,s);
            if s { seen.push(u); }
        }
        k += 1;
    }
    let visible = |u: usize| is_seen.get(&u) == Some(&true);
    // the rim: each edge of a seen face whose neighbour is not seen, joined to `p`
    let mut rim = Vec::new();
    for &t in &seen {
        for k in 0..3 {
            let u = tris[t].across[k];
            if !visible(u) { rim.push((tris[t].v[k],tris[t].v[(k+1)%3],u)); }
        }
    }
    let mut new = Vec::new();
    for &(a,b,outside) in &rim {
        let id = tris.len();
        tris.push(Tri {v:[a,b,p],across:[outside,0,0],beyond:Vec::new(),live:true});
        // the face beyond the rim now meets the new one across the rim's edge
        let o = &mut tris[outside];
        let k = (0..3).find(|&k| o.v[k] == b && o.v[(k+1)%3] == a).ok_or("a hull whose rim does not close")?;
        o.across[k] = id;
        new.push(id);
    }
    // the new faces meet one another across their edges to `p`: the rim is one cycle
    let mut from_start = BTreeMap::new();
    for &t in &new {
        if from_start.insert(tris[t].v[0],t).is_some() { return Err("a hull whose rim is no cycle".into()) }
    }
    for &t in &new {
        let [a,b,_] = tris[t].v;
        let next = *from_start.get(&b).ok_or("a hull whose rim does not close")?;
        let prev = new.iter().copied().find(|&u| tris[u].v[1] == a).ok_or("a hull whose rim does not close")?;
        tris[t].across[1] = next;
        tris[t].across[2] = prev;
    }
    // the points beyond the faces taken off placed beyond a new face that sees them; one no new
    // face sees, it lies within the grown hull — unless it lies on a new face's plane, where
    // only the whole hull can say
    let mut loose = Vec::new();
    for &t in &seen {
        tris[t].live = false;
        loose.extend(std::mem::take(&mut tris[t].beyond).into_iter().filter(|&i| i != p));
    }
    let mut back = None;
    for i in loose {
        if let Some(&t) = new.iter().find(|&&t| sees(pts,&tris[t],i)) { tris[t].beyond.push(i); continue }
        let level = new.iter().any(|&t| { let v = tris[t].v; orient(pts[v[0]],pts[v[1]],pts[v[2]],pts[i]) == 0 });
        if level {
            if let Some(t) = (0..tris.len()).find(|&t| tris[t].live && sees(pts,&tris[t],i)) {
                tris[t].beyond.push(i);
                back = Some(back.map_or(t,|b: usize| b.min(t)));
            }
        }
    }
    Ok(back)
}

/// A face of the hull: the triangles it is made of, the first its largest, and its corners
/// counter-clockwise seen from outside.
struct Facet { tris: Vec<usize>,corners: Vec<usize> }

/// The live triangles gathered into faces, each grown from its largest triangle (the module's
/// rule), and each face's corners read round its rim.
fn faces(pts: &[V],tris: &[Tri],tol: f64) -> Vec<Facet> {
    let normal = |t: &Tri| cross(sub(pts[t.v[1]],pts[t.v[0]]),sub(pts[t.v[2]],pts[t.v[0]]));
    let mut order: Vec<usize> = (0..tris.len()).filter(|&t| tris[t].live).collect();
    order.sort_by(|&s,&t| norm(normal(&tris[t])).total_cmp(&norm(normal(&tris[s]))).then(s.cmp(&t)));
    let mut face_of = vec![usize::MAX;tris.len()];
    let mut out = Vec::new();
    for seed in order {
        if face_of[seed] != usize::MAX { continue }
        let id = out.len();
        let n = unit(normal(&tris[seed]));
        let o = pts[tris[seed].v[0]];
        let level = |t: usize| dot(normal(&tris[t]),n) > 0. && tris[t].v.iter().all(|&v| dot(sub(pts[v],o),n).abs() <= tol);
        face_of[seed] = id;
        let mut members = vec![seed];
        let mut k = 0;
        while k < members.len() {
            for u in tris[members[k]].across {
                if face_of[u] == usize::MAX && level(u) { face_of[u] = id; members.push(u); }
            }
            k += 1;
        }
        out.push(Facet {tris:members,corners:Vec::new()});
    }
    // each face's rim: its triangles' edges that another face is across, chained; a face whose
    // rim is no one cycle (never, on a convex surface) is its triangles apart
    let mut i = 0;
    while i < out.len() {
        let id = i;
        let mut next = BTreeMap::new();
        for &t in &out[id].tris {
            for k in 0..3 {
                if face_of[tris[t].across[k]] != id { next.insert(tris[t].v[k],tris[t].v[(k+1)%3]); }
            }
        }
        let start = *next.keys().next().expect("a face has a rim");
        let mut ring = vec![start];
        while let Some(&v) = next.get(ring.last().unwrap()) {
            if v == start || ring.len() > next.len() { break }
            ring.push(v);
        }
        if ring.len() == next.len() && next.get(ring.last().unwrap()) == Some(&start) {
            out[id].corners = ring;
        } else {
            let apart: Vec<usize> = out[id].tris.split_off(1);
            out[id].corners = tris[out[id].tris[0]].v.to_vec();
            for t in apart {
                face_of[t] = out.len();
                out.push(Facet {tris:vec![t],corners:tris[t].v.to_vec()});
            }
        }
        i += 1;
    }
    out
}

/// Each corner on the edge between two faces only, within `tol` of the line through its
/// neighbours there, dropped from both: no corner of the hull.
fn drop_straight(pts: &[V],faces: &mut [Facet],tol: f64) {
    loop {
        let mut at: BTreeMap<usize,Vec<usize>> = BTreeMap::new();
        for (f,face) in faces.iter().enumerate() { for &v in &face.corners { at.entry(v).or_default().push(f); } }
        let straight = at.iter().find_map(|(&v,fs)| {
            if fs.len() != 2 || fs.iter().any(|&f| faces[f].corners.len() <= 3) { return None }
            let ring = &faces[fs[0]].corners;
            let k = ring.iter().position(|&w| w == v)?;
            let (a,b) = (pts[ring[(k+ring.len()-1)%ring.len()]],pts[ring[(k+1)%ring.len()]]);
            let along = unit(sub(b,a));
            let q = sub(pts[v],a);
            (norm(sub(q,scale(along,dot(q,along)))) <= tol).then(|| (v,fs.clone()))
        });
        let Some((v,fs)) = straight else { return };
        for f in fs { faces[f].corners.retain(|&w| w != v); }
    }
}

/// The faces made a solid: a vertex at each corner, a line between corners for each edge, and
/// a plane face for each facet, through its corners as they fit best.
fn solid(pts: &[V],faces: &[Facet],tris: &[Tri],tol: f64) -> Result<Brep,String> {
    let mut b = Brep::default();
    let mut vertex = BTreeMap::new();
    let mut edges: BTreeMap<(usize,usize),u32> = BTreeMap::new();
    for face in faces {
        let m = face.corners.len();
        let at: Vec<V> = face.corners.iter().map(|&v| pts[v]).collect();
        // Newell's normal and the centroid; the first triangle's plane where a corner stands
        // farther off that than the tolerance
        let mut n = [0.;3];
        for k in 0..m { let (p,q) = (at[k],at[(k+1)%m]); n = add(n,cross(p,q)); }
        let mut n = unit(n);
        let mut o = scale(at.iter().fold([0.;3],|s,&p| add(s,p)),1./m as f64);
        if !n.iter().all(|x| x.is_finite()) || at.iter().any(|&p| dot(sub(p,o),n).abs() > tol) {
            let v = tris[face.tris[0]].v.map(|v| pts[v]);
            n = unit(cross(sub(v[1],v[0]),sub(v[2],v[0])));
            o = v[0];
        }
        let mut uses = Vec::with_capacity(m);
        for k in 0..m {
            let (i,j) = (face.corners[k],face.corners[(k+1)%m]);
            let key = (i.min(j),i.max(j));
            let along = match edges.get(&key) {
                Some(&e) => Coedge {edge:e,reversed:true,pcurve:Pcurve::Inverse {a:[0.;2],b:[0.;2]}},
                None => {
                    let [vi,vj] = [i,j].map(|v| *vertex.entry(v).or_insert_with(|| b.vertex(pts[v])));
                    let d = sub(pts[j],pts[i]);
                    let e = b.edge(EdgeCurve::Curve(Curve::Line {p:pts[i],d:unit(d)}),[0.,norm(d)],[vi,vj]);
                    edges.insert(key,e);
                    Coedge {edge:e,reversed:false,pcurve:Pcurve::Inverse {a:[0.;2],b:[0.;2]}}
                }
            };
            uses.push(along);
        }
        b.faces.push(Face {surface:Surface::Plane(Frame::about(o,n)),reversed:false,loops:vec![uses],name:String::new()});
    }
    b.measure_tolerances();
    b.check(tol).map_err(|e| format!("a hull not closed: {e}"))?;
    Ok(b)
}
