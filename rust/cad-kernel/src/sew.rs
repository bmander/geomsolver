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
//! - **a vertex onto an edge** (`tolerant::on_stretch`): a boundary edge is cut at another's vertex
//!   within `8 tol` of its curve inside its stretch and clear of its ends by as much
//!   (`boolean::cut_with`), so a side holding in pieces what the other holds whole pairs one to one;
//! - **an edge onto an edge** (`tolerant::same_edge`): two boundary edges between the same
//!   vertices, the middle of one within `8 tol` of the other, are one edge — two only: a third is
//!   no manifold, none an open shell.
//!
//! Of two edges made one, the curve kept is the one bounding the face whose surface is the hardest
//! to invert onto (`Surface::inversion`: one searched over a closed form, a closed form over a
//! plane), the first given where they are alike, so its pcurve stays as built.
//! The other face's use takes it on its own pcurve where the two curves are one parametrisation
//! (each an affine function of the other's parameter), else on the kept curve inverted onto its
//! surface. Each edge's tolerance is then measured (`Brep::measure_tolerances`) and an edge
//! standing farther than `8 tol` off a face it bounds refused: what holds the middle alone to the
//! rest of the curve.
//!
//! The faces are turned to agree across every edge, each connected set a shell; a shell that
//! cannot agree (one side of a Möbius band) is refused. Each shell is turned to enclose its
//! material outward, a void — a shell inside an odd number of others — inward. What comes back
//! passes `Brep::check` at `tol`.
use super::boolean::cut_with;
use super::complex::faced;
use super::geom::{Curve,V};
use super::query::{Located,Place};
use super::topo::{Brep,Coedge,EdgeCurve,Pcurve};
use super::tolerant::{on_stretch,same_edge,Pool,ON_CURVE};
use super::{KernelError,Kind};
use crate::util::bvh::{Bounds,Bvh};
use crate::util::space::distance;
use std::collections::BTreeMap;

/// Where a use is: `(face, loop, use)`.
type At = (usize,usize,usize);

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
    let free = free(b,&b.uses());
    for (e,_) in b.edges.iter_mut().zip(free).filter(|(_,f)| *f) {
        let EdgeCurve::Curve(c) = &e.curve else { continue };
        let at = c.point(e.t[0]);
        let collapsed = match c {
            Curve::BSpline(s) => s.poles_over(e.t).iter().all(|&q| distance(q,at) <= tol),
            _ => (0..=32).map(|k| e.t[0]+(e.t[1]-e.t[0])*k as f64/32.).chain(c.breaks(e.t)).all(|t| distance(c.point(t),at) <= tol),
        };
        if collapsed { e.curve = EdgeCurve::Degenerate; }
    }
}

/// The point halfway along edge `e`'s stretch: where an error says it is.
fn middle(b: &Brep,e: usize) -> V { let e = &b.edges[e]; e.point(0.5*(e.t[0]+e.t[1]),&b.vertices) }

/// The pieces' vertices merged where they come within `tol` (`Pool`), each keeping its tolerance.
fn merge_vertices(b: &mut Brep,tol: f64) {
    let mut pool = Pool::new(tol);
    let at: Vec<u32> = b.vertices.iter().map(|v| pool.at(v.p)).collect();
    let mut vertices = pool.vertices();
    for (v,&i) in b.vertices.iter().zip(&at) { vertices[i as usize].tol = vertices[i as usize].tol.max(v.tol); }
    for e in &mut b.edges { e.v = e.v.map(|v| at[v as usize]); }
    b.vertices = vertices;
}

/// The boundary edges, by the uses `Brep::uses` counts: each on a curve and used once.
fn free(b: &Brep,uses: &[Vec<At>]) -> Vec<bool> {
    b.edges.iter().zip(uses).map(|(e,u)| matches!(e.curve,EdgeCurve::Curve(_)) && u.len() == 1).collect()
}

/// Each boundary edge cut at every boundary vertex standing on it (`on_stretch`), the edges near a
/// vertex found by their boxes.
fn split_at_vertices(b: &mut Brep,tol: f64) {
    let reach = ON_CURVE*tol;
    let free = free(b,&b.uses());
    let edges: Vec<usize> = (0..b.edges.len()).filter(|&i| free[i]).collect();
    let mut ends: Vec<u32> = edges.iter().flat_map(|&i| b.edges[i].v).collect();
    ends.sort();
    ends.dedup();
    let boxes = Bvh::new(edges.iter().map(|&i| {
        let e = &b.edges[i];
        let EdgeCurve::Curve(c) = &e.curve else { unreachable!("a boundary edge lies on a curve") };
        let (lo,hi) = c.bounds(e.t);
        Bounds {lo:lo.map(|x| x-reach),hi:hi.map(|x| x+reach)}
    }));
    let mut cuts: BTreeMap<u32,Vec<(f64,u32)>> = BTreeMap::new();
    for &v in &ends {
        let p = b.vertices[v as usize].p;
        boxes.query_box(Bounds {lo:p,hi:p},|k| {
            let e = &b.edges[edges[k]];
            let EdgeCurve::Curve(c) = &e.curve else { return };
            if e.v.contains(&v) { return }
            if let Some(t) = on_stretch(c,e.t,p,tol) { cuts.entry(edges[k] as u32).or_default().push((t,v)); }
        });
    }
    for at in cuts.values_mut() { at.sort_by(|x,y| x.0.total_cmp(&y.0).then(x.1.cmp(&y.1))); }
    cut_with(b,&cuts);
}

/// Each two boundary edges lying along one another made one (the module's rules).
fn pair_edges(b: &mut Brep,tol: f64) -> Result<(),KernelError> {
    let uses = b.uses();
    let free = free(b,&uses);
    // the boundary edges by the vertices they join
    let mut by_ends: BTreeMap<[u32;2],Vec<usize>> = BTreeMap::new();
    for (i,e) in b.edges.iter().enumerate().filter(|&(i,_)| free[i]) {
        by_ends.entry([e.v[0].min(e.v[1]),e.v[0].max(e.v[1])]).or_default().push(i);
    }
    // whether `j` is `i`, and then whether it runs the same way
    let along = |b: &Brep,i: usize,j: usize| -> Option<bool> {
        let (ei,ej) = (&b.edges[i],&b.edges[j]);
        let (EdgeCurve::Curve(ci),EdgeCurve::Curve(cj)) = (&ei.curve,&ej.curve) else { return None };
        same_edge(ci,ei.t,ei.v,cj,ej.t,ej.v,tol)
    };
    let face_of = |e: usize| uses[e][0].0;
    // the edge kept: the one bounding the face hardest to invert onto, else the face given first
    let rank = |b: &Brep,e: usize| (std::cmp::Reverse(b.faces[face_of(e)].surface.inversion()),face_of(e));
    for group in by_ends.into_values() {
        let mut taken = vec![false;group.len()];
        for x in 0..group.len() {
            if taken[x] { continue }
            // two mates tell none, one and more apart
            let mates: Vec<(usize,bool)> = (x+1..group.len()).filter(|&y| !taken[y])
                .filter_map(|y| along(b,group[x],group[y]).map(|same| (y,same))).take(2).collect();
            let &[(y,same)] = &mates[..] else {
                return Err(if mates.is_empty() {
                    KernelError::new(Kind::Open,format!("the faces do not close: an edge near {:?} bounds one face",middle(b,group[x])))
                } else {
                    KernelError::new(Kind::NotManifold,format!("three or more faces meet along the edge near {:?}",middle(b,group[x])))
                })
            };
            taken[x] = true;
            taken[y] = true;
            let (i,j) = (group[x],group[y]);
            let (keep,drop) = if rank(b,j) < rank(b,i) { (j,i) } else { (i,j) };
            joined(b,keep,drop,same,uses[drop][0],tol);
        }
    }
    Ok(())
}

/// The use of `drop` at `at` given to `keep`, which lies along it, running the same way where `same`.
fn joined(b: &mut Brep,keep: usize,drop: usize,same: bool,(fi,li,ci): At,tol: f64) {
    let (k,d) = (&b.edges[keep],&b.edges[drop]);
    let (EdgeCurve::Curve(ck),EdgeCurve::Curve(cd)) = (&k.curve,&d.curve) else { unreachable!("boundary edges lie on curves") };
    let face = &b.faces[fi];
    let c = &face.loops[li][ci];
    // `drop`'s ends in `keep`'s order, and its parameter at `keep`'s where one is an affine function
    // of the other: then the two curves are one parametrisation
    let [d0,d1] = if same { d.t } else { [d.t[1],d.t[0]] };
    let affine = |t: f64| d0+(t-k.t[0])/(k.t[1]-k.t[0])*(d1-d0);
    let one = (0..=16).map(|j| k.t[0]+(k.t[1]-k.t[0])*j as f64/16.).all(|t| distance(ck.point(t),cd.point(affine(t))) <= tol);
    let [a,z] = [d0,d1].map(|t| c.pcurve.at(t,d,&face.surface,&b.vertices));
    let pcurve = match &c.pcurve {
        Pcurve::Line {..} if one => Pcurve::Line {a,b:z},
        Pcurve::Curve(pc) if one && same && k.t == d.t => Pcurve::Curve(pc.clone()),
        _ => Pcurve::Inverse {a,b:z},
    };
    let reversed = c.reversed == same;
    b.faces[fi].loops[li][ci] = Coedge {edge:keep as u32,reversed,pcurve};
}

/// The faces turned to agree across every edge and each shell outward (its voids inward).
fn orient(b: &mut Brep,tol: f64) -> Result<(),KernelError> {
    let uses = b.uses();
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
                let &[x,y] = &uses[c.edge as usize][..] else { continue };
                // the way a use runs once its face is turned
                let runs = |(fi,li,ci): At,flipped: bool| b.faces[fi].loops[li][ci].reversed != flipped;
                let (mine,theirs) = if x.0 == f { (x,y) } else { (y,x) };
                let other = theirs.0;
                if shell[other] == usize::MAX {
                    // turned so its use runs against this one's
                    shell[other] = shells;
                    flip[other] = runs(theirs,false) == runs(mine,flip[f]);
                    todo.push(other);
                } else if runs(mine,flip[f]) == runs(theirs,flip[other]) {
                    return Err(KernelError::new(Kind::Refused,format!("the faces sewn about the edge near {:?} cannot be turned \
                        to agree: a one-sided shell",middle(b,c.edge as usize))))
                }
            }
        }
        shells += 1;
    }
    for f in 0..n { if flip[f] { b.faces[f] = faced(&b.faces[f],true); } }
    // each shell's volume, and whether it lies inside an odd number of the others
    let fluxes = super::props::fluxes(b);
    let mut volumes = vec![0.;shells];
    for f in 0..n { volumes[shell[f]] += fluxes[f]/3.; }
    let size = b.size();
    for (s,&v) in volumes.iter().enumerate() {
        if !(v.abs() > 1e-12*size*size*size) {
            return Err(KernelError::new(Kind::Refused,format!("sewn shell {s} encloses nothing ({v:e})")))
        }
    }
    let mut void = vec![false;shells];
    if shells > 1 {
        let alone: Vec<Located> = (0..shells).map(|s| Located::owned(Brep {vertices:b.vertices.clone(),edges:b.edges.clone(),
            faces:(0..n).filter(|&f| shell[f] == s).map(|f| b.faces[f].clone()).collect()}.compacted(),tol)).collect();
        for s in 0..shells {
            let f = shell.iter().position(|&x| x == s).unwrap();
            let p = b.vertices[b.edges[b.faces[f].loops[0][0].edge as usize].v[0] as usize].p;
            void[s] = (0..shells).filter(|&t| t != s && alone[t].solid_place(p) == Place::In).count()%2 == 1;
        }
    }
    for f in 0..n { if (volumes[shell[f]] < 0.) != void[shell[f]] { b.faces[f] = faced(&b.faces[f],true); } }
    Ok(())
}
