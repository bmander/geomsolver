//! **Faces sewn into a solid** (brep-bench's geometry/nurbs-cone): sheets built apart — a B-spline
//! net with a pole, the disc closing it, the faces of any other kernel's shell — joined where their
//! boundaries are one curve, into the boundary of a solid. Nothing is intersected: the pieces
//! must meet along their edges and nowhere else, the caller's to know.
//!
//! Features are the same by the rules of tolerant topology (`tolerant`), against the tolerance
//! `tol` the pieces were built to, in this order:
//! - **an edge onto its point**: an edge whose whole curve lies within `tol` of its start (a
//!   B-spline's poles there, its hull holding it; any other curve sampled) is a surface's pole, a
//!   degenerate edge its one face uses once;
//! - **a point onto a vertex** (`tolerant::Pool`): the pieces' vertices merged;
//! - **a vertex onto an edge** (`tolerant::ON_CURVE`): a boundary edge is cut at another's vertex
//!   within `8 tol` of its curve inside its stretch and clear of its ends by as much, so a side
//!   holding in pieces what the other holds whole pairs one to one;
//! - **an edge onto an edge** (`ON_CURVE`): two boundary edges between the same vertices, each
//!   lying along the other, are one edge — two only: a third is no manifold, none an open shell.
//!
//! Of two edges made one, the curve kept is the one bounding the face whose surface is the hardest
//! to invert onto — a B-spline's or any other surface read by search over an analytic one, an
//! analytic one over a plane, the first given where they are alike — so its pcurve stays as built.
//! The other face's use takes it on its own pcurve where the two curves are one parametrisation
//! (each an affine function of the other's parameter), else on the kept curve inverted onto its
//! surface. Each edge's tolerance is then measured (`Brep::measure_tolerances`) and an edge
//! standing farther than `8 tol` off a face it bounds refused.
//!
//! The faces are turned to agree across every edge, each connected set a shell; a shell that
//! cannot agree (one side of a Möbius band) is refused. Each shell is turned to enclose its
//! material outward, a void — a shell inside an odd number of others — inward. What comes back
//! passes `Brep::check` at `tol`.
use super::geom::{Curve,Surface,Uv,V};
use super::topo::{reverse_loop,Brep,Coedge,Edge,EdgeCurve,Pcurve};
use super::tolerant::{Pool,ON_CURVE};
use super::{KernelError,Kind};
use crate::util::space::{distance,dot};

/// **The pieces' faces sewn into one solid's boundary**, to the tolerance `tol` they were built to
/// (the module's rules). `Kind::Open` where an edge bounds one face only, `Kind::NotManifold` where
/// three or more lie along one another, `Kind::Refused` where the faces cannot be turned to agree
/// or a shell encloses nothing.
pub fn sewn(pieces: &[Brep],tol: f64) -> Result<Brep,KernelError> {
    if !(tol > 0.) { return Err(KernelError::new(Kind::Failed,format!("faces sewn to a tolerance of {tol}"))) }
    let mut b = Brep::default();
    for p in pieces { b.join(p); }
    if b.faces.is_empty() { return Err(KernelError::new(Kind::Open,"no faces to sew")) }
    poles(&mut b,tol);
    merge_vertices(&mut b,tol);
    split_at_vertices(&mut b,tol);
    pair_edges(&mut b,tol)?;
    let mut b = b.compacted();
    b.measure_tolerances();
    if let Some((i,e)) = b.edges.iter().enumerate().find(|(_,e)| e.tol > ON_CURVE*tol) {
        return Err(KernelError::new(Kind::Refused,format!("a sewn edge {i} near {:?} stands {:e} off a face it bounds",
            middle(&b,i),e.tol)))
    }
    orient(&mut b,tol)?;
    b.check(tol)?;
    Ok(b)
}

/// Every boundary edge whose curve lies within `tol` of its start made degenerate: a pole.
fn poles(b: &mut Brep,tol: f64) {
    let free = free(b);
    for (e,_) in b.edges.iter_mut().zip(free).filter(|(_,f)| *f) {
        let EdgeCurve::Curve(c) = &e.curve else { continue };
        let at = c.point(e.t[0]);
        let collapsed = match c {
            Curve::BSpline(s) => s.poles_over(e.t).iter().all(|&q| distance(q,at) <= tol),
            _ => samples(e,33).chain(c.breaks(e.t)).all(|t| distance(c.point(t),at) <= tol),
        };
        if collapsed { e.curve = EdgeCurve::Degenerate; }
    }
}

/// The point halfway along edge `e`'s stretch: where an error says it is.
fn middle(b: &Brep,e: usize) -> V { let e = &b.edges[e]; e.point(0.5*(e.t[0]+e.t[1]),&b.vertices) }

/// `n+1` parameters evenly along the edge's stretch, its ends included.
fn samples(e: &Edge,n: usize) -> impl Iterator<Item=f64> + '_ {
    (0..=n).map(move |k| e.t[0]+(e.t[1]-e.t[0])*k as f64/n as f64)
}

/// The pieces' vertices merged where they come within `tol` (`Pool`), each keeping its tolerance.
fn merge_vertices(b: &mut Brep,tol: f64) {
    let mut pool = Pool::new(tol);
    let at: Vec<u32> = b.vertices.iter().map(|v| pool.at(v.p)).collect();
    let mut vertices = pool.vertices();
    for (v,&i) in b.vertices.iter().zip(&at) { vertices[i as usize].tol = vertices[i as usize].tol.max(v.tol); }
    for e in &mut b.edges { e.v = e.v.map(|v| at[v as usize]); }
    b.vertices = vertices;
}

/// How many times each edge is used, by `(face, loop, use)`.
fn uses(b: &Brep) -> Vec<Vec<(usize,usize,usize)>> {
    let mut uses = vec![Vec::new();b.edges.len()];
    for (fi,f) in b.faces.iter().enumerate() {
        for (li,l) in f.loops.iter().enumerate() {
            for (ci,c) in l.iter().enumerate() { uses[c.edge as usize].push((fi,li,ci)); }
        }
    }
    uses
}

/// The boundary edges: each on a curve and used once.
fn free(b: &Brep) -> Vec<bool> {
    let uses = uses(b);
    b.edges.iter().zip(&uses).map(|(e,u)| matches!(e.curve,EdgeCurve::Curve(_)) && u.len() == 1).collect()
}

/// The parameter of the point of an edge's stretch nearest `p`, and how far it stands.
fn foot(e: &Edge,p: V) -> Option<(f64,f64)> {
    let EdgeCurve::Curve(c) = &e.curve else { return None };
    let mut t = c.inverse(p);
    if let Some(period) = c.period() {
        // onto the period from the stretch's start, then nearer its end if past it
        t = e.t[0]+(t-e.t[0]).rem_euclid(period);
        if t > e.t[1] && t-e.t[1] > e.t[0]+period-t { t -= period; }
    }
    let t = t.clamp(e.t[0],e.t[1]);
    Some((t,distance(c.point(t),p)))
}

/// Each boundary edge cut at every boundary vertex standing on it, clear of its ends: its pieces
/// on its curve, the first keeping its index, each use of it walking them in its own direction.
fn split_at_vertices(b: &mut Brep,tol: f64) {
    let reach = ON_CURVE*tol;
    let free = free(b);
    let mut ends: Vec<u32> = b.edges.iter().zip(&free).filter(|(_,&f)| f).flat_map(|(e,_)| e.v).collect();
    ends.sort();
    ends.dedup();
    // each cut edge's pieces, in its direction
    let mut pieces: std::collections::BTreeMap<u32,Vec<u32>> = Default::default();
    for ei in 0..free.len() {
        if !free[ei] { continue }
        let e = b.edges[ei].clone();
        let EdgeCurve::Curve(c) = &e.curve else { continue };
        let (lo,hi) = c.bounds(e.t);
        let (a,z) = (b.vertices[e.v[0] as usize].p,b.vertices[e.v[1] as usize].p);
        let mut cuts: Vec<(f64,u32)> = ends.iter().filter(|&&v| !e.v.contains(&v)).filter_map(|&v| {
            let p = b.vertices[v as usize].p;
            if (0..3).any(|k| p[k] < lo[k]-reach || p[k] > hi[k]+reach) || distance(p,a) <= reach || distance(p,z) <= reach { return None }
            let (t,d) = foot(&e,p)?;
            (d <= reach && t > e.t[0] && t < e.t[1]).then_some((t,v))
        }).collect();
        if cuts.is_empty() { continue }
        cuts.sort_by(|x,y| x.0.total_cmp(&y.0));
        let ts: Vec<f64> = std::iter::once(e.t[0]).chain(cuts.iter().map(|c| c.0)).chain([e.t[1]]).collect();
        let vs: Vec<u32> = std::iter::once(e.v[0]).chain(cuts.iter().map(|c| c.1)).chain([e.v[1]]).collect();
        let cut = (0..ts.len()-1).map(|k| {
            let piece = Edge {t:[ts[k],ts[k+1]],v:[vs[k],vs[k+1]],..e.clone()};
            if k == 0 { b.edges[ei] = piece; ei as u32 } else { b.edges.push(piece); (b.edges.len()-1) as u32 }
        }).collect();
        pieces.insert(ei as u32,cut);
    }
    if pieces.is_empty() { return }
    let Brep {vertices,edges,faces} = b;
    for face in faces.iter_mut() {
        for l in &mut face.loops {
            *l = l.iter().flat_map(|c| {
                let Some(cut) = pieces.get(&c.edge) else { return vec![c.clone()] };
                // the whole edge's parameters along the use, read where the edge was whole
                let (first,last) = (&edges[cut[0] as usize],&edges[*cut.last().unwrap() as usize]);
                let whole = Edge {t:[first.t[0],last.t[1]],..first.clone()};
                let uv = |t: f64| c.pcurve.at(t,&whole,&face.surface,vertices);
                let mut walked: Vec<Coedge> = cut.iter().map(|&piece| {
                    let [t0,t1] = edges[piece as usize].t;
                    Coedge {edge:piece,reversed:c.reversed,pcurve:match &c.pcurve {
                        Pcurve::Line {..} => Pcurve::Line {a:uv(t0),b:uv(t1)},
                        Pcurve::Inverse {..} => Pcurve::Inverse {a:uv(t0),b:uv(t1)},
                        Pcurve::Curve(pc) => Pcurve::Curve(pc.clone()),
                    }}
                }).collect();
                if c.reversed { walked.reverse(); }
                walked
            }).collect();
        }
    }
}

/// How hard a surface is to invert onto: a plane's inverse is exact, an analytic surface's a
/// closed form, the rest searched.
fn hardness(s: &Surface) -> u8 {
    match s {
        Surface::Plane(_) => 0,
        Surface::Cylinder(..) | Surface::Cone(..) | Surface::Sphere(..) | Surface::Torus(..) => 1,
        _ => 2,
    }
}

/// Each two boundary edges lying along one another made one (the module's rules).
fn pair_edges(b: &mut Brep,tol: f64) -> Result<(),KernelError> {
    let reach = ON_CURVE*tol;
    let free = free(b);
    let uses = uses(b);
    // the boundary edges by the vertices they join
    let mut by_ends: std::collections::BTreeMap<[u32;2],Vec<usize>> = Default::default();
    for (i,e) in b.edges.iter().enumerate().filter(|&(i,_)| free[i]) {
        by_ends.entry([e.v[0].min(e.v[1]),e.v[0].max(e.v[1])]).or_default().push(i);
    }
    // whether `j` lies along `i`, at a few points of each
    let along = |b: &Brep,i: usize,j: usize| -> bool {
        let (ei,ej) = (&b.edges[i],&b.edges[j]);
        let on = |x: &Edge,y: &Edge| samples(x,8).skip(1).take(7)
            .all(|t| foot(y,x.point(t,&b.vertices)).is_some_and(|(_,d)| d <= reach));
        on(ej,ei) && on(ei,ej)
    };
    for (_,group) in by_ends {
        let mut taken = vec![false;group.len()];
        for x in 0..group.len() {
            if taken[x] { continue }
            let mates: Vec<usize> = (x+1..group.len()).filter(|&y| !taken[y] && along(b,group[x],group[y])).collect();
            match mates.len() {
                0 => return Err(KernelError::new(Kind::Open,format!("the faces do not close: an edge near {:?} bounds one face",
                    middle(b,group[x])))),
                1 => {}
                _ => return Err(KernelError::new(Kind::NotManifold,format!("{} faces meet along the edge near {:?}",
                    mates.len()+1,middle(b,group[x])))),
            }
            let y = mates[0];
            taken[x] = true;
            taken[y] = true;
            let (i,j) = (group[x],group[y]);
            let face_of = |e: usize| { let (fi,_,_) = uses[e][0]; fi };
            // the edge kept, the one bounding the face hardest to invert onto
            let (keep,drop) = if hardness(&b.faces[face_of(j)].surface) > hardness(&b.faces[face_of(i)].surface) { (j,i) } else { (i,j) };
            joined(b,keep,drop,uses[drop][0],tol);
        }
    }
    Ok(())
}

/// The use of `drop` at `(face, loop, use)` given to `keep`, which lies along it.
fn joined(b: &mut Brep,keep: usize,drop: usize,(fi,li,ci): (usize,usize,usize),tol: f64) {
    let (k,d) = (b.edges[keep].clone(),b.edges[drop].clone());
    let (EdgeCurve::Curve(ck),EdgeCurve::Curve(cd)) = (&k.curve,&d.curve) else { unreachable!("boundary edges lie on curves") };
    // whether they run the same way: by their ends where they part, else by their tangents
    let same = if k.v[0] != k.v[1] { k.v[0] == d.v[0] } else {
        let t = 0.5*(d.t[0]+d.t[1]);
        let (s,_) = foot(&k,cd.point(t)).expect("a curve");
        dot(cd.tangent(t),ck.tangent(s)) > 0.
    };
    let face = &b.faces[fi];
    let c = face.loops[li][ci].clone();
    // the dropped use's parameters at `drop`'s ends, in `keep`'s order
    let ends: [Uv;2] = { let e = [c.pcurve.at(d.t[0],&d,&face.surface,&b.vertices),c.pcurve.at(d.t[1],&d,&face.surface,&b.vertices)];
        if same { e } else { [e[1],e[0]] } };
    // `drop`'s parameter at `keep`'s, where one is an affine function of the other
    let affine = |t: f64| { let s = (t-k.t[0])/(k.t[1]-k.t[0]); if same { d.t[0]+s*(d.t[1]-d.t[0]) } else { d.t[1]-s*(d.t[1]-d.t[0]) } };
    let one = samples(&k,16).all(|t| distance(ck.point(t),cd.point(affine(t))) <= tol);
    let pcurve = match &c.pcurve {
        Pcurve::Line {..} if one => Pcurve::Line {a:ends[0],b:ends[1]},
        Pcurve::Curve(pc) if one && same && k.t == d.t => Pcurve::Curve(pc.clone()),
        _ => Pcurve::Inverse {a:ends[0],b:ends[1]},
    };
    b.faces[fi].loops[li][ci] = Coedge {edge:keep as u32,reversed:c.reversed != !same,pcurve};
}

/// The faces turned to agree across every edge and each shell outward (its voids inward).
fn orient(b: &mut Brep,tol: f64) -> Result<(),KernelError> {
    let uses = uses(b);
    let n = b.faces.len();
    // each face's turn and shell, by a walk across the edges from each face not yet reached
    let (mut flip,mut shell) = (vec![false;n],vec![usize::MAX;n]);
    let mut shells = 0;
    for start in 0..n {
        if shell[start] != usize::MAX { continue }
        shell[start] = shells;
        let mut todo = vec![start];
        while let Some(f) = todo.pop() {
            for c in b.faces[f].loops.iter().flatten() {
                let u = &uses[c.edge as usize];
                if u.len() != 2 { continue }
                let walked = |&(fi,li,ci): &(usize,usize,usize)| b.faces[fi].loops[li][ci].reversed != flip[fi];
                let (x,y) = (u[0],u[1]);
                let other = if x.0 == f { y.0 } else { x.0 };
                if shell[other] == usize::MAX {
                    shell[other] = shells;
                    // turned so its use runs against this one's
                    let (mine,theirs) = if x.0 == f { (x,y) } else { (y,x) };
                    flip[other] = (b.faces[theirs.0].loops[theirs.1][theirs.2].reversed) == walked(&mine);
                    todo.push(other);
                } else if walked(&x) == walked(&y) {
                    return Err(KernelError::new(Kind::Refused,format!("the faces sewn about the edge near {:?} cannot be turned \
                        to agree: a one-sided shell",middle(b,c.edge as usize))))
                }
            }
        }
        shells += 1;
    }
    let turn = |b: &mut Brep,f: usize| {
        let face = &mut b.faces[f];
        face.reversed = !face.reversed;
        for l in &mut face.loops { *l = reverse_loop(std::mem::take(l)); }
    };
    for f in 0..n { if flip[f] { turn(b,f); } }
    // each shell's volume, and whether it lies inside an odd number of the others
    let fluxes = super::props::fluxes(b);
    let mut volumes = vec![0.;shells];
    for f in 0..n { volumes[shell[f]] += fluxes[f]/3.; }
    let scale = b.size();
    for (s,&v) in volumes.iter().enumerate() {
        if !(v.abs() > 1e-12*scale*scale*scale) {
            return Err(KernelError::new(Kind::Refused,format!("sewn shell {s} encloses nothing ({v:e})")))
        }
    }
    let alone = |b: &Brep,s: usize| Brep {vertices:b.vertices.clone(),edges:b.edges.clone(),
        faces:(0..n).filter(|&f| shell[f] == s).map(|f| b.faces[f].clone()).collect()};
    let void: Vec<bool> = (0..shells).map(|s| {
        if shells == 1 { return false }
        let f = (0..n).find(|&f| shell[f] == s).unwrap();
        let p = b.vertices[b.edges[b.faces[f].loops[0][0].edge as usize].v[0] as usize].p;
        (0..shells).filter(|&t| t != s)
            .filter(|&t| super::query::Located::new(&alone(b,t),tol).solid_place(p) == super::query::Place::In).count()%2 == 1
    }).collect();
    for f in 0..n { if (volumes[shell[f]] < 0.) != void[shell[f]] { turn(b,f); } }
    Ok(())
}
