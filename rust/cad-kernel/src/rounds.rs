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
use super::topo::{reverse_loop,Brep,Coedge,EdgeCurve,Face,Pcurve};
use crate::util::fmath::Det;
use crate::util::space::{add,cross,distance,dot,norm,scale,sub,unit};
use std::collections::BTreeMap;

/// One rounded edge: its two faces (`[a, b]`), the run's sense along the edge, and whether it is
/// convex.
struct Round {
    edge: usize,
    faces: [usize;2],
    d: V,
    convex: bool,
    /// The cylinder's frame: about the spine, its `x` toward the contact on `faces[0]`, `o` on the
    /// spine (moved to a corner's ball's centre where it is used); and the arc's angle to the other
    /// contact.
    frame: Frame,
    turn: f64,
}

/// `b` with `edges` (indices into `b.edges`) rounded to the radius `r`, to the tolerance `tol` (the
/// module's rules).
pub fn rounded(b: &Brep,edges: &[usize],r: f64,tol: f64) -> Result<Brep,String> {
    if !(r > 0.) { return Err(format!("a round of radius {r}")) }
    let mut edges: Vec<usize> = edges.to_vec();
    edges.sort_unstable();
    edges.dedup();
    if let Some(&e) = edges.iter().find(|&&e| e >= b.edges.len()) { return Err(format!("no edge {e} to round")) }
    if edges.is_empty() { return Ok(b.clone()) }
    // the edges at each vertex the rounds reach
    let mut at: BTreeMap<u32,Vec<usize>> = BTreeMap::new();
    for (e,ed) in b.edges.iter().enumerate() { for v in ed.v { at.entry(v).or_default().push(e); } }
    let reached: std::collections::BTreeSet<u32> = edges.iter().flat_map(|&e| b.edges[e].v).collect();
    let whole = reached.iter().all(|v| { let es = &at[v]; es.len() == 3 && es.iter().all(|e| edges.binary_search(e).is_ok()) });
    if !whole {
        // no two rounds meet: each rolled on its own
        if edges.iter().enumerate().all(|(i,&e)| edges[i+1..].iter().all(|&f| b.edges[e].v.iter().all(|v| !b.edges[f].v.contains(v)))) {
            return rolled_in_turn(b,&edges,r)
        }
        let v = reached.iter().find(|v| { let es = &at[v]; !(es.len() == 3 && es.iter().all(|e| edges.binary_search(e).is_ok())) }).unwrap();
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

/// A plane face's frame and outward normal, or why it has none.
fn plane_of(f: &Face) -> Result<(Frame,V),String> {
    match f.surface {
        Surface::Plane(fr) => Ok((fr,if f.reversed { scale(fr.z,-1.) } else { fr.z })),
        ref s => Err(format!("a round beside a face `{}` on a {}: only rounds between planes are built in one call",f.name,s.kind())),
    }
}

/// A point's parameters on a plane.
fn plane_uv(f: &Frame,p: V) -> Uv { let d = sub(p,f.o); [dot(d,f.x),dot(d,f.y)] }

fn surgery(b: &Brep,edges: &[usize],reached: &std::collections::BTreeSet<u32>,r: f64,tol: f64) -> Result<Brep,String> {
    let is_round = |e: usize| edges.binary_search(&e).is_ok();
    // each edge's uses: (face, loop, index)
    let mut uses: Vec<Vec<(usize,usize,usize)>> = vec![Vec::new();b.edges.len()];
    for (fi,f) in b.faces.iter().enumerate() {
        for (li,l) in f.loops.iter().enumerate() { for (k,c) in l.iter().enumerate() { uses[c.edge as usize].push((fi,li,k)); } }
    }
    // each round's faces, directions and convexity
    let mut rounds: Vec<Round> = Vec::new();
    for &e in edges {
        let ed = &b.edges[e];
        let (p0,p1) = (b.vertices[ed.v[0] as usize].p,b.vertices[ed.v[1] as usize].p);
        match &ed.curve {
            EdgeCurve::Curve(Curve::Line {..}) => {}
            _ => return Err(format!("a round along an edge from {p0:?} to {p1:?} that is no line: only straight edges are \
                rounded in one call")),
        }
        let [ua,ub] = match uses[e][..] { [a,b] if a.0 != b.0 => [a,b],_ => return Err(format!("the edge from {p0:?} to {p1:?} \
            bounds {} uses of faces, not two faces",uses[e].len())) };
        let len = distance(p0,p1);
        if !(len > tol) { return Err(format!("a round along an edge of no length at {p0:?}")) }
        let d = scale(sub(p1,p0),1./len);
        // across from the edge into each face: its interior is on the left of a use, seen from outside
        let mut faces = [0;2];
        let mut across = [[0.;3];2];
        let mut normals = [[0.;3];2];
        for (k,(fi,li,ci)) in [ua,ub].into_iter().enumerate() {
            let (_,n) = plane_of(&b.faces[fi])?;
            let along = if b.faces[fi].loops[li][ci].reversed { scale(d,-1.) } else { d };
            faces[k] = fi;
            normals[k] = n;
            across[k] = unit(cross(n,along));
        }
        let convex = dot(across[0],normals[1]) < 0.;
        let sin = norm(cross(across[0],across[1]));
        let cos = dot(across[0],across[1]);
        if !(sin > 1e-9) {
            return Err(format!("the faces `{}` and `{}` meet along {p0:?}–{p1:?} tangent or folded flat: no ball rounds them",
                b.faces[faces[0]].name,b.faces[faces[1]].name))
        }
        // the setback along each face, and the spine: `r` inside both planes at a convex edge, outside
        // at a concave one
        let s = r*(1.+cos)/sin;
        let off = |n: V| if convex { scale(n,-r) } else { scale(n,r) };
        let c0 = add(add(p0,scale(across[0],s)),off(normals[0]));
        let x = if convex { normals[0] } else { scale(normals[0],-1.) };
        let to_b = if convex { normals[1] } else { scale(normals[1],-1.) };
        let mut z = d;
        let mut y = cross(z,x);
        let mut turn = dot(to_b,y).datan2(dot(to_b,x));
        if turn < 0. { z = scale(d,-1.); y = cross(z,x); turn = -turn; }
        rounds.push(Round {edge:e,faces,d,convex,frame:Frame {o:c0,x,y,z},turn});
    }
    // each corner's ball: its centre `r` from each of its three faces, on the material's side or
    // the void's, and where it touches each
    struct Ball { centre: V,touches: BTreeMap<usize,V>,convex: bool }
    let mut balls: BTreeMap<u32,Ball> = BTreeMap::new();
    for &v in reached {
        let p = b.vertices[v as usize].p;
        let here: Vec<&Round> = rounds.iter().filter(|rd| b.edges[rd.edge].v.contains(&v)).collect();
        let convex = here[0].convex;
        if here.iter().any(|rd| rd.convex != convex) {
            return Err(format!("convex and concave rounds meeting at {p:?}: a corner between them is not closed by its ball \
                alone"))
        }
        let mut faces: Vec<usize> = here.iter().flat_map(|rd| rd.faces).collect();
        faces.sort_unstable();
        faces.dedup();
        let [fa,fb,fc] = faces[..] else { return Err(format!("the corner at {p:?} lies between {} faces, not three",faces.len())) };
        let n = [fa,fb,fc].map(|f| plane_of(&b.faces[f]).map(|x| x.1)).into_iter().collect::<Result<Vec<V>,_>>()?;
        let sign = if convex { -1. } else { 1. };
        let rhs: Vec<f64> = n.iter().map(|&ni| dot(ni,p)+sign*r).collect();
        let det = dot(n[0],cross(n[1],n[2]));
        if !(det.abs() > 1e-9) { return Err(format!("the corner at {p:?} has two of its faces parallel")) }
        // Cramer's rule, by the rows' cross products
        let centre = scale(add(add(scale(cross(n[1],n[2]),rhs[0]),scale(cross(n[2],n[0]),rhs[1])),scale(cross(n[0],n[1]),rhs[2])),1./det);
        let touches = [fa,fb,fc].into_iter().zip(&n).map(|(f,&ni)| (f,sub(centre,scale(ni,sign*r)))).collect();
        balls.insert(v,Ball {centre,touches,convex});
    }
    // the new boundary: untouched vertices kept, each touched one replaced by its ball's touches
    let mut out = Brep::default();
    let mut kept_vertex: BTreeMap<u32,u32> = BTreeMap::new();
    for (v,vx) in b.vertices.iter().enumerate() {
        if !reached.contains(&(v as u32)) { kept_vertex.insert(v as u32,out.vertex(vx.p)); out.vertices.last_mut().unwrap().tol = vx.tol; }
    }
    let mut touch_vertex: BTreeMap<(u32,usize),u32> = BTreeMap::new();
    for (&v,ball) in &balls { for (&f,&t) in &ball.touches { touch_vertex.insert((v,f),out.vertex(t)); } }
    let mut kept_edge: BTreeMap<usize,u32> = BTreeMap::new();
    for (e,ed) in b.edges.iter().enumerate() {
        if is_round(e) { continue }
        if ed.v.iter().any(|v| reached.contains(v)) { unreachable!("an edge at a rounded corner is rounded") }
        kept_edge.insert(e,out.edges.len() as u32);
        out.edges.push(super::topo::Edge {v:ed.v.map(|v| kept_vertex[&v]),..ed.clone()});
    }
    // each round's contact lines on its two faces and its arcs at its two ends
    let mut contact: BTreeMap<(usize,usize),u32> = BTreeMap::new();
    let mut arcs: Vec<[u32;2]> = Vec::new();
    for rd in &rounds {
        let ed = &b.edges[rd.edge];
        for k in 0..2 {
            let f = rd.faces[k];
            let (a,z) = (touch_vertex[&(ed.v[0],f)],touch_vertex[&(ed.v[1],f)]);
            let (pa,pz) = (out.vertices[a as usize].p,out.vertices[z as usize].p);
            let run = dot(sub(pz,pa),rd.d);
            if !(run > tol) {
                return Err(format!("a round of {r} along {:?}–{:?} is larger than the face `{}` can hold: its contact runs \
                    past the corners",b.vertices[ed.v[0] as usize].p,b.vertices[ed.v[1] as usize].p,b.faces[f].name))
            }
            let line = Curve::Line {p:pa,d:unit(sub(pz,pa))};
            contact.insert((rd.edge,f),out.edge(EdgeCurve::Curve(line),[0.,distance(pa,pz)],[a,z]));
        }
        let ends = ed.v.map(|v| {
            let c = balls[&v].centre;
            let frame = Frame {o:c,..rd.frame};
            let (a,z) = (touch_vertex[&(v,rd.faces[0])],touch_vertex[&(v,rd.faces[1])]);
            out.edge(EdgeCurve::Curve(Curve::Circle(frame,r)),[0.,rd.turn],[a,z])
        });
        arcs.push(ends);
    }
    // the faces: untouched loops kept, touched ones walked along the contact lines
    for (fi,f) in b.faces.iter().enumerate() {
        let mut loops = Vec::new();
        for l in &f.loops {
            let touched = l.iter().filter(|c| is_round(c.edge as usize)).count();
            if touched == 0 {
                loops.push(l.iter().map(|c| Coedge {edge:kept_edge[&(c.edge as usize)],..c.clone()}).collect());
                continue
            }
            if touched != l.len() { unreachable!("a loop rounded in part reaches a corner with an edge not rounded") }
            let (frame,_) = plane_of(f)?;
            loops.push(l.iter().map(|c| {
                let e = contact[&(c.edge as usize,fi)];
                let ed = &out.edges[e as usize];
                let ends = ed.v.map(|v| plane_uv(&frame,out.vertices[v as usize].p));
                Coedge {edge:e,reversed:c.reversed,pcurve:Pcurve::Line {a:ends[0],b:ends[1]}}
            }).collect());
        }
        out.faces.push(Face {loops,..f.clone()});
    }
    // each round's cylinder: arc, far contact, arc back, near contact back — counter-clockwise in
    // its parameters where the run's sense and the arc's agree, its material inside at a convex edge
    for (k,rd) in rounds.iter().enumerate() {
        let ed = &b.edges[rd.edge];
        // from the first corner's ball, whose arc is at `v = 0`, to the second's
        let frame = Frame {o:balls[&ed.v[0]].centre,..rd.frame};
        let v1 = dot(sub(balls[&ed.v[1]].centre,frame.o),frame.z);
        let (la,lb) = (contact[&(rd.edge,rd.faces[0])],contact[&(rd.edge,rd.faces[1])]);
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
        let here: Vec<usize> = rounds.iter().enumerate().filter(|(_,rd)| b.edges[rd.edge].v.contains(&v)).map(|(k,_)| k).collect();
        let middle = unit(ball.touches.values().fold([0.;3],|s,&t| add(s,sub(t,ball.centre))));
        let frame = Frame::new(ball.centre,Frame::about(ball.centre,middle).x,scale(middle,-1.));
        let sphere = Surface::Sphere(frame,r);
        // each arc's use, from one touch to the other, in its edge's sense
        let arc_of = |k: usize| { let end = usize::from(b.edges[rounds[k].edge].v[1] == v); arcs[k][end] };
        let mut walk: Vec<(u32,bool)> = vec![(arc_of(here[0]),false)];
        let mut left: Vec<u32> = here[1..].iter().map(|&k| arc_of(k)).collect();
        let mut tip = out.edges[walk[0].0 as usize].v[1];
        while !left.is_empty() {
            let i = left.iter().position(|&e| out.edges[e as usize].v.contains(&tip)).ok_or("a corner's arcs do not close")?;
            let e = left.remove(i);
            let rev = out.edges[e as usize].v[1] == tip;
            tip = out.edges[e as usize].v[usize::from(!rev)];
            walk.push((e,rev));
        }
        let uv = |p: V| sphere.inverse(p);
        let mut l: Vec<Coedge> = walk.iter().map(|&(e,rev)| {
            let ed = &out.edges[e as usize];
            let ends = ed.v.map(|w| uv(out.vertices[w as usize].p));
            Coedge {edge:e,reversed:rev,pcurve:Pcurve::Inverse {a:ends[0],b:ends[1]}}
        }).collect();
        // the triangle's turn in the sphere's parameters, from its corners
        let corners: Vec<Uv> = l.iter().map(|c| uv(out.vertices[out.ends(c)[0] as usize].p)).collect();
        let area: f64 = (0..3).map(|i| { let (p,q) = (corners[i],corners[(i+1)%3]); p[0]*q[1]-p[1]*q[0] }).sum();
        let reversed = !ball.convex;
        if (area > 0.) == reversed { l = reverse_loop(l); }
        out.faces.push(Face {surface:sphere,reversed,loops:vec![l],name:"corner".into()});
    }
    out.check(tol).map_err(|e| format!("the rounded solid: {e}"))?;
    Ok(out)
}
