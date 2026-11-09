//! **Edges rounded together, the kernel placing every piece**: `rounded(b, edges, r, tol)` rounds a
//! set of a solid's edges to the radius `r` in one call. Where the edges are straight, between
//! planes, and every vertex they reach has all three of its edges among them, the solid is built
//! anew by surgery on its boundary — nothing intersected, nothing fitted:
//!
//! - each edge's round is a **cylinder** about its spine, the edge carried `s = r / tan(θ/2)` along
//!   each face (θ the angle between the faces, spanned the way the ball stands: inside the material
//!   at a convex edge, in the void at a concave one) and `r` off both planes;
//! - each face keeps its loops, each rounded edge's use replaced by the round's **contact line** on
//!   the face, between where the balls of the corners at its ends touch it;
//! - each vertex is closed by a **sphere** about the ball touching its three faces, its triangle
//!   bounded by the three rounds' arcs where they end: each spine runs through that ball's centre,
//!   so each round ends in the plane through it square to its edge, on a great circle of the ball.
//!
//! What the surgery cannot build is refused by name: a face no plane carries, an edge no line does,
//! a vertex of other than three edges or some of them not rounded, convex and concave edges at one
//! vertex (the setback such a corner needs), and a round the face beside it cannot hold (a contact
//! line run past its corners). Edges of which no two share a vertex are rounded instead each in
//! turn by `fillet::roll` (any faces it rolls between), each piece cut off a convex edge and filled
//! in at a concave one.
use super::boolean::{combined,Op};
use super::geom::{Curve,Frame,Surface,Uv,V};
use super::topo::{reverse_loop,Brep,Coedge,Edge,EdgeCurve,Face,Pcurve};
use crate::util::fmath::Det;
use crate::util::space::{add,cross,distance,dot,norm,scale,sub,unit};
use std::collections::{BTreeMap,BTreeSet};

/// One rounded edge: its two faces (`[a, b]`) and their outward normals, the run's sense along the
/// edge, and whether it is convex.
struct Round {
    edge: usize,
    faces: [usize;2],
    normals: [V;2],
    d: V,
    convex: bool,
    /// The cylinder's axes, `z` along the spine and `x` from it toward the contact on `faces[0]` (its
    /// origin set where it is used, at a corner's ball's centre); and the arc's angle from there to
    /// the contact on the other face.
    axes: Frame,
    turn: f64,
}

/// A corner's ball: its centre, the rounds meeting there (indices into the rounds) and the vertex
/// where it touches each of its three faces.
struct Ball { centre: V,here: Vec<usize>,touches: BTreeMap<usize,u32> }

/// `b` with `edges` (indices into `b.edges`) rounded to the radius `r`, to the tolerance `tol` (the
/// module's rules).
pub fn rounded(b: &Brep,edges: &[usize],r: f64,tol: f64) -> Result<Brep,String> {
    if !(r > 0.) { return Err(format!("a round of radius {r}")) }
    let mut edges: Vec<usize> = edges.to_vec();
    edges.sort_unstable();
    edges.dedup();
    if let Some(&e) = edges.iter().find(|&&e| e >= b.edges.len()) { return Err(format!("no edge {e} to round")) }
    if edges.is_empty() { return Ok(b.clone()) }
    // the edges at each vertex the rounds reach, and whether all three of them are rounded
    let mut at: BTreeMap<u32,Vec<usize>> = BTreeMap::new();
    for (e,ed) in b.edges.iter().enumerate() { for v in ed.v { at.entry(v).or_default().push(e); } }
    let reached: BTreeSet<u32> = edges.iter().flat_map(|&e| b.edges[e].v).collect();
    let whole = |v: &u32| { let es = &at[v]; es.len() == 3 && es.iter().all(|e| edges.binary_search(e).is_ok()) };
    if let Some(v) = reached.iter().find(|v| !whole(v)) {
        // no two rounds meet: each rolled on its own
        if edges.iter().enumerate().all(|(i,&e)| edges[i+1..].iter().all(|&f| b.edges[e].v.iter().all(|v| !b.edges[f].v.contains(v)))) {
            return rolled_in_turn(b,&edges,r)
        }
        return Err(format!("rounds meeting at {:?}, where {} edges meet and not every one is rounded: only a corner \
            whose three edges are all rounded is closed by its ball",b.vertices[*v as usize].p,at[v].len()))
    }
    surgery(b,&edges,&reached,r,tol)
}

/// `rounded` where no two of `edges` share a vertex: each rolled against the solid in turn.
fn rolled_in_turn(b: &Brep,edges: &[usize],r: f64) -> Result<Brep,String> {
    // each edge picked by its middle, as it stands on the solid first given
    let middles: Vec<V> = edges.iter().map(|&e| { let ed = &b.edges[e]; ed.point(0.5*(ed.t[0]+ed.t[1]),&b.vertices) }).collect();
    let mut solid = b.clone();
    for at in middles {
        let round = super::fillet::roll(&solid,at,r,super::geom::tolerance(solid.size()))?;
        solid = combined(&solid,&round.piece,if round.concave { Op::Union } else { Op::Cut },0.).map_err(|e| e.to_string())?;
    }
    Ok(solid)
}

/// A plane face's outward normal, or why it has none.
fn plane_normal(f: &Face) -> Result<V,String> {
    match f.surface {
        Surface::Plane(fr) => Ok(if f.reversed { scale(fr.z,-1.) } else { fr.z }),
        ref s => Err(format!("a round beside a face `{}` on a {}: only rounds between planes are built in one call",f.name,s.kind())),
    }
}

fn surgery(b: &Brep,edges: &[usize],reached: &BTreeSet<u32>,r: f64,tol: f64) -> Result<Brep,String> {
    let is_round = |e: usize| edges.binary_search(&e).is_ok();
    let uses = b.uses();
    // each round's faces, directions and convexity
    let mut rounds: Vec<Round> = Vec::new();
    for &e in edges {
        let ed = &b.edges[e];
        let (p0,p1) = (b.vertices[ed.v[0] as usize].p,b.vertices[ed.v[1] as usize].p);
        if !matches!(ed.curve,EdgeCurve::Curve(Curve::Line {..})) {
            return Err(format!("a round along an edge from {p0:?} to {p1:?} that is no line: only straight edges are \
                rounded in one call"))
        }
        let [ua,ub] = match uses[e][..] { [a,b] if a.0 != b.0 => [a,b],_ => return Err(format!("the edge from {p0:?} to {p1:?} \
            bounds {} uses of faces, not two faces",uses[e].len())) };
        let len = distance(p0,p1);
        if !(len > tol) { return Err(format!("a round along an edge of no length at {p0:?}")) }
        let d = scale(sub(p1,p0),1./len);
        // across from the edge into each face: its interior is on the left of a use, seen from outside
        let faces = [ua.0,ub.0];
        let mut across = [[0.;3];2];
        let mut normals = [[0.;3];2];
        for (k,(fi,li,ci)) in [ua,ub].into_iter().enumerate() {
            normals[k] = plane_normal(&b.faces[fi])?;
            let along = if b.faces[fi].loops[li][ci].reversed { scale(d,-1.) } else { d };
            across[k] = unit(cross(normals[k],along));
        }
        let convex = dot(across[0],normals[1]) < 0.;
        if !(norm(cross(across[0],across[1])) > 1e-9) {
            return Err(format!("the faces `{}` and `{}` meet along {p0:?}–{p1:?} tangent or folded flat: no ball rounds them",
                b.faces[faces[0]].name,b.faces[faces[1]].name))
        }
        // from the spine toward each contact: along each face's outward normal at a convex edge, the
        // ball inside the material; against it at a concave one, the ball in the void
        let side = if convex { 1. } else { -1. };
        let (x,to_b) = (scale(normals[0],side),scale(normals[1],side));
        // the axis turned so the arc runs the positive way from `x` to the other contact
        let z = if dot(to_b,cross(d,x)) < 0. { scale(d,-1.) } else { d };
        let y = cross(z,x);
        let turn = dot(to_b,y).datan2(dot(to_b,x));
        rounds.push(Round {edge:e,faces,normals,d,convex,axes:Frame {o:[0.;3],x,y,z},turn});
    }
    // the new boundary: untouched vertices kept, each corner's ball touching its three faces at new
    // ones
    let mut out = Brep::default();
    let mut kept_vertex: BTreeMap<u32,u32> = BTreeMap::new();
    for (v,vx) in b.vertices.iter().enumerate() {
        if !reached.contains(&(v as u32)) { kept_vertex.insert(v as u32,out.vertices.len() as u32); out.vertices.push(*vx); }
    }
    let mut balls: BTreeMap<u32,Ball> = BTreeMap::new();
    for &v in reached {
        let p = b.vertices[v as usize].p;
        let here: Vec<usize> = (0..rounds.len()).filter(|&k| b.edges[rounds[k].edge].v.contains(&v)).collect();
        let convex = rounds[here[0]].convex;
        if here.iter().any(|&k| rounds[k].convex != convex) {
            return Err(format!("convex and concave rounds meeting at {p:?}: a corner between them is not closed by its ball \
                alone"))
        }
        let faces: BTreeMap<usize,V> = here.iter().flat_map(|&k| rounds[k].faces.into_iter().zip(rounds[k].normals)).collect();
        let faces: Vec<(usize,V)> = faces.into_iter().collect();
        let [(fa,na),(fb,nb),(fc,nc)] = faces[..] else { return Err(format!("the corner at {p:?} lies between {} faces, not three",faces.len())) };
        let n = [na,nb,nc];
        if !(dot(n[0],cross(n[1],n[2])).abs() > 1e-9) { return Err(format!("the corner at {p:?} has two of its faces parallel")) }
        // the centre `r` inside each plane at a convex corner, outside at a concave one
        let sign = if convex { -1. } else { 1. };
        let centre = super::ssi::solve3(n,n.map(|ni| dot(ni,p)+sign*r));
        let touches = [fa,fb,fc].into_iter().zip(n).map(|(f,ni)| (f,out.vertex(sub(centre,scale(ni,sign*r))))).collect();
        balls.insert(v,Ball {centre,here,touches});
    }
    let mut kept_edge: BTreeMap<usize,u32> = BTreeMap::new();
    for (e,ed) in b.edges.iter().enumerate() {
        if is_round(e) { continue }
        kept_edge.insert(e,out.edges.len() as u32);
        out.edges.push(Edge {v:ed.v.map(|v| kept_vertex[&v]),..ed.clone()});
    }
    // each round's contact lines on its two faces, each with its use's line in the face's
    // parameters, and its arcs at its two ends
    let mut contact: BTreeMap<(usize,usize),(u32,Pcurve)> = BTreeMap::new();
    let mut arcs: Vec<[u32;2]> = Vec::new();
    for rd in &rounds {
        let ed = &b.edges[rd.edge];
        for f in rd.faces {
            let (a,z) = (balls[&ed.v[0]].touches[&f],balls[&ed.v[1]].touches[&f]);
            let (pa,pz) = (out.vertices[a as usize].p,out.vertices[z as usize].p);
            if !(dot(sub(pz,pa),rd.d) > tol) {
                return Err(format!("a round of {r} along {:?}–{:?} is larger than the face `{}` can hold: its contact runs \
                    past the corners",b.vertices[ed.v[0] as usize].p,b.vertices[ed.v[1] as usize].p,b.faces[f].name))
            }
            let e = out.edge(EdgeCurve::Curve(Curve::Line {p:pa,d:unit(sub(pz,pa))}),[0.,distance(pa,pz)],[a,z]);
            let frame = b.faces[f].surface.frame();
            let uv = |p: V| -> Uv { let [u,v,_] = frame.local(p); [u,v] };
            contact.insert((rd.edge,f),(e,Pcurve::Line {a:uv(pa),b:uv(pz)}));
        }
        arcs.push(ed.v.map(|v| {
            let ball = &balls[&v];
            let frame = Frame {o:ball.centre,..rd.axes};
            out.edge(EdgeCurve::Curve(Curve::Circle(frame,r)),[0.,rd.turn],[ball.touches[&rd.faces[0]],ball.touches[&rd.faces[1]]])
        }));
    }
    // the faces: each use of a rounded edge walked along its contact line instead, in the same sense
    for (fi,f) in b.faces.iter().enumerate() {
        let loops = f.loops.iter().map(|l| l.iter().map(|c| match contact.get(&(c.edge as usize,fi)) {
            Some((e,pc)) => Coedge {edge:*e,reversed:c.reversed,pcurve:pc.clone()},
            None => Coedge {edge:kept_edge[&(c.edge as usize)],..c.clone()},
        }).collect()).collect();
        out.faces.push(Face {loops,..f.clone()});
    }
    // each round's cylinder: arc, far contact, arc back, near contact back — counter-clockwise in
    // its parameters where the run's sense and the arc's agree, its material inside at a convex edge
    for (k,rd) in rounds.iter().enumerate() {
        let ed = &b.edges[rd.edge];
        // from the first corner's ball, whose arc is at `v = 0`, to the second's
        let frame = Frame {o:balls[&ed.v[0]].centre,..rd.axes};
        let v1 = dot(sub(balls[&ed.v[1]].centre,frame.o),frame.z);
        let (la,lb) = (contact[&(rd.edge,rd.faces[0])].0,contact[&(rd.edge,rd.faces[1])].0);
        let mut l = vec![
            Coedge {edge:arcs[k][0],reversed:false,pcurve:Pcurve::Line {a:[0.,0.],b:[rd.turn,0.]}},
            Coedge {edge:lb,reversed:false,pcurve:Pcurve::Line {a:[rd.turn,0.],b:[rd.turn,v1]}},
            Coedge {edge:arcs[k][1],reversed:true,pcurve:Pcurve::Line {a:[0.,v1],b:[rd.turn,v1]}},
            Coedge {edge:la,reversed:true,pcurve:Pcurve::Line {a:[0.,0.],b:[0.,v1]}},
        ];
        let reversed = !rd.convex;
        if (v1 > 0.) == reversed { l = reverse_loop(l); }
        out.faces.push(Face {surface:Surface::Cylinder(frame,r),reversed,loops:vec![l],name:"round".into()});
    }
    // each corner's sphere: its frame's poles and seam turned away from the triangle, which stands
    // about `u = π` on its equator; the three arcs walked end to start
    for (&v,ball) in &balls {
        let middle = unit(ball.touches.values().fold([0.;3],|s,&t| add(s,sub(out.vertices[t as usize].p,ball.centre))));
        let frame = Frame::new(ball.centre,Frame::about(ball.centre,middle).x,scale(middle,-1.));
        let sphere = Surface::Sphere(frame,r);
        // each arc's use, from one touch to the other, in its edge's sense
        let arc_of = |k: usize| { let end = usize::from(b.edges[rounds[k].edge].v[1] == v); arcs[k][end] };
        let mut walk: Vec<(u32,bool)> = vec![(arc_of(ball.here[0]),false)];
        let mut left: Vec<u32> = ball.here[1..].iter().map(|&k| arc_of(k)).collect();
        let mut tip = out.edges[walk[0].0 as usize].v[1];
        while !left.is_empty() {
            let i = left.iter().position(|&e| out.edges[e as usize].v.contains(&tip)).ok_or("a corner's arcs do not close")?;
            let e = left.remove(i);
            let rev = out.edges[e as usize].v[1] == tip;
            tip = out.edges[e as usize].v[usize::from(!rev)];
            walk.push((e,rev));
        }
        // each use's ends on the sphere, and where it starts
        let (mut l,corners): (Vec<Coedge>,Vec<Uv>) = walk.iter().map(|&(e,rev)| {
            let ends = out.edges[e as usize].v.map(|w| sphere.inverse(out.vertices[w as usize].p));
            (Coedge {edge:e,reversed:rev,pcurve:Pcurve::Inverse {a:ends[0],b:ends[1]}},ends[usize::from(rev)])
        }).unzip();
        // the triangle's turn in the sphere's parameters, from its corners
        let area: f64 = (0..3).map(|i| { let (p,q) = (corners[i],corners[(i+1)%3]); p[0]*q[1]-p[1]*q[0] }).sum();
        let reversed = !rounds[ball.here[0]].convex;
        if (area > 0.) == reversed { l = reverse_loop(l); }
        out.faces.push(Face {surface:sphere,reversed,loops:vec![l],name:"corner".into()});
    }
    out.check(tol).map_err(|e| format!("the rounded solid: {e}"))?;
    Ok(out)
}
