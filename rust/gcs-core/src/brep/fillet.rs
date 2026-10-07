//! **A ball rolled along a closed traced loop** (issue #66, rung 2): where two analytic faces meet
//! on a loop no line or circle carries — a pipe tee's crotch, a rod through an inclined plate — a
//! ball of radius `r` touching both rolls with its centre on the meeting of the two surfaces
//! offset by `r` (`Surface::offset`), traced. Its face is the canal swept by the ball's arc between
//! the two contacts, fitted as a B-spline net within a stated bar; the piece of material it rounds
//! is bounded by that face and the two strips of the faces between the loop and the contacts,
//! built here with its edges shared by construction (nothing intersected). A body takes it by the
//! Boolean, which follows the canal's tangent contact with each face along the edge it lies in.
#[allow(unused_imports)]
use crate::fmath::Det;
use super::geom::{around,Curve,Frame,Surface,Uv,V};
use super::nurbs::{fit_curve,fit_net,BSpline,Net};
use super::query::Located;
use super::topo::{unwrap,Brep,Coedge,EdgeCurve,Face,Pcurve};
use crate::space::{add,cross,distance,dot,norm,scale,sub};
use std::sync::Arc;

const TAU: f64 = std::f64::consts::TAU;
/// Faces nearer parallel than this (radians, either way) meet at no edge a ball can round.
const MIN_TURN: f64 = std::f64::consts::PI/180.;

/// A rolled fillet: its piece of material, the spine the ball's centre runs along (closed, its
/// parameter over `[0, 1]`), whether the material is added (concave) and how far the corner
/// stands from the spine at most; all in the boundary's own units.
#[derive(Clone,Debug)]
pub struct Rolled {
    pub piece: Brep,
    /// The edges of the boundary it rolled along: the loop between the two faces.
    pub chain: Vec<usize>,
    pub spine: BSpline,
    /// Where the ball touches each face, over the spine's parameter.
    pub contacts: [BSpline;2],
    pub concave: bool,
    pub r: f64,
    pub reach: f64,
    /// Where a run that ends is cut off: at each end a point of the plane and its normal toward
    /// the piece. None for a closed loop.
    pub trims: Vec<(V,V)>,
}

fn unit(a: V) -> V { scale(a,1./norm(a)) }

/// Along the arc of the unit sphere from `a` to `b` (the short way), a fraction `v`.
fn slerp(a: V,b: V,v: f64) -> V {
    let w = dot(a,b).clamp(-1.,1.).dacos();
    let s = w.dsin();
    if s < 1e-12 { return a }
    add(scale(a,((1.-v)*w).dsin()/s),scale(b,(v*w).dsin()/s))
}

/// The ball of radius `r` rolled along the edge of `b` nearest `at`, the faces either side of it
/// meeting on a closed loop; `tol` the boundary's tolerance, and the bar the canal and the spine
/// are fitted within. `Err` says why it cannot be rolled exactly.
pub fn roll(b: &Brep,at: V,r: f64,tol: f64) -> Result<Rolled,String> {
    let located = Located::new(b,tol);
    // the uses of every edge
    let mut uses: Vec<Vec<(usize,usize,usize)>> = vec![Vec::new();b.edges.len()];
    for (fi,f) in b.faces.iter().enumerate() {
        for (li,l) in f.loops.iter().enumerate() { for (ci,c) in l.iter().enumerate() { uses[c.edge as usize].push((fi,li,ci)); } }
    }
    // the edge nearest `at` between two faces
    let nearest = (0..b.edges.len()).filter(|&e| matches!(uses[e].as_slice(),[x,y] if x.0 != y.0))
        .filter_map(|e| { let EdgeCurve::Curve(c) = &b.edges[e].curve else { return None };
            let mut t = c.inverse(at);
            if let Some(p) = c.period() { t = around(t,b.edges[e].t[0],p); }
            let t = t.clamp(b.edges[e].t[0],b.edges[e].t[1]);
            Some((distance(c.point(t),at),e)) })
        .min_by(|x,y| x.0.total_cmp(&y.0)).ok_or("no edge to roll a ball along")?.1;
    let [(fa,la,ca),(fb,_,_)] = [uses[nearest][0],uses[nearest][1]];
    let (face_a,face_b) = (&b.faces[fa],&b.faces[fb]);
    let names = || format!("`{}` with `{}`",face_a.name,face_b.name);
    // a ball's centre stands off each face along its normal: a surface with no closed-form offset
    // (a swept spline, a fitted sheet) is not rounded yet
    for f in [face_a,face_b] {
        if f.surface.offset(0.).is_none() {
            return Err(format!("`{}` ({}) has no offset in closed form, so no ball rolls on it yet: planes, \
                cylinders, cones, spheres and tori are rounded (rung 2)",f.name,f.surface.kind()));
        }
    }
    // each side: every face on its surface, the same way out (a face split at its seam is one
    // face to the ball)
    let on = |f: &Face| (0..b.faces.len()).filter(|&g| b.faces[g].reversed == f.reversed
        && super::ssi::same(&b.faces[g].surface,&f.surface,tol)).collect::<Vec<usize>>();
    let sides_faces = [on(face_a),on(face_b)];
    // the run: the edges between these two sides reached from this one through their vertices,
    // closing on itself or ending at two vertices
    let between: Vec<usize> = (0..b.edges.len()).filter(|&e| match uses[e].as_slice() {
        [x,y] => (0..2).any(|k| sides_faces[k].contains(&x.0) && sides_faces[1-k].contains(&y.0)),
        _ => false,
    }).collect();
    let mut chain = vec![nearest];
    let mut k = 0;
    while k < chain.len() {
        let vs = b.edges[chain[k]].v;
        for &e in &between {
            if !chain.contains(&e) && b.edges[e].v.iter().any(|v| vs.contains(v)) { chain.push(e); }
        }
        k += 1;
    }
    chain.sort();
    let mut degree = std::collections::BTreeMap::<u32,usize>::new();
    for &e in &chain { for v in b.edges[e].v { *degree.entry(v).or_default() += 1; } }
    let stops: Vec<u32> = degree.iter().filter(|(_,&d)| d % 2 != 0).map(|(&v,_)| v).collect();
    if !(stops.is_empty() || stops.len() == 2 && degree.values().all(|&d| d <= 2)) {
        return Err(format!("the meeting of {} branches: a fillet along it is rung 3",names()));
    }
    // the faces' outward normals and the directions into each from the edge, at its middle
    let e = &b.edges[nearest];
    let EdgeCurve::Curve(curve) = &e.curve else { return Err(format!("{} meet at a point",names())) };
    let mid = 0.5*(e.t[0]+e.t[1]);
    let p = curve.point(mid);
    let tangent = unit(curve.tangent(mid));
    let outward = |fi: usize| -> Result<V,String> {
        let f = &b.faces[fi];
        let n = f.surface.normal(located.uv(fi,p)).ok_or_else(|| format!("`{}` has no normal where it is rounded",f.name))?;
        Ok(if f.reversed { scale(n,-1.) } else { n })
    };
    let (na,nb) = (outward(fa)?,outward(fb)?);
    let reversed_a = b.faces[fa].loops[la][ca].reversed;
    let ta = if reversed_a { scale(tangent,-1.) } else { tangent };
    let da = unit(cross(na,ta));
    let db = unit(cross(nb,scale(ta,-1.)));
    let theta = dot(da,db).clamp(-1.,1.).dacos();
    if !(theta > MIN_TURN && theta < std::f64::consts::PI-MIN_TURN) {
        return Err(format!("{} meet too nearly flat or too sharply to round",names()));
    }
    let concave = dot(db,na) > 0.;
    // each surface offset by `r` toward the ball: along its outward normal at a concave edge
    let side = |f: &Face| { let out = if f.reversed { -1. } else { 1. }; if concave { out } else { -out } };
    let sides = [side(face_a),side(face_b)];
    let surfaces = [face_a.surface.clone(),face_b.surface.clone()];
    let [Some(oa),Some(ob)] = [0,1].map(|k| surfaces[k].offset(sides[k]*r)) else {
        return Err(format!("the ball between {} is larger than one of them can hold",names()));
    };
    // the spine: the offsets' meeting through the ball's centre in the corner at `p`
    let onto = super::geom::Traced {a:oa,b:ob,pts:Vec::new(),closed:false};
    let (oa,ob) = (&onto.a,&onto.b);
    let guess = add(p,scale(unit(add(da,db)),r/(0.5*theta).dsin()));
    let seed = onto.project(guess);
    let spine = match super::ssi::intersect(oa,ob,tol) {
        super::ssi::Ssi::Curves(cs) => cs,
        super::ssi::Ssi::Same => return Err(format!("{} lie on one surface",names())),
        super::ssi::Ssi::Traced => {
            let (lo,hi) = b.bounds();
            let grow = 4.*r+norm(sub(hi,lo));
            super::ssi::trace(oa,ob,&[seed],lo.map(|x| x-grow),hi.map(|x| x+grow),tol)?
        }
    }.into_iter().min_by(|x,y| {
        let off = |c: &Curve| distance(c.point(c.inverse(seed)),seed);
        off(x).total_cmp(&off(y))
    }).ok_or_else(|| format!("the ball between {} rolls nowhere",names()))?;
    // where the ball touches each face, and the direction from its centre to there
    let toward = |k: usize,c: V| scale(unit(surfaces[k].gradient(c)),-sides[k]);
    // **a run that ends**: at each end the faces there besides the two lie in one plane, which cuts
    // the fillet off; the ball rolls on past it and the piece is trimmed there
    let mut ends: Vec<Stop> = Vec::new();
    for &v in &stops {
        let p = b.vertices[v as usize].p;
        let others: std::collections::BTreeSet<usize> = (0..b.edges.len()).filter(|&e| b.edges[e].v.contains(&v))
            .flat_map(|e| uses[e].iter().map(|u| u.0).collect::<Vec<_>>()).filter(|f| !sides_faces.iter().any(|s| s.contains(f))).collect();
        let plane = |f: usize| match b.faces[f].surface { Surface::Plane(fr) => Some(fr),_ => None };
        let &first = others.iter().next().ok_or_else(|| format!("the run of {} ends where nothing meets it",names()))?;
        let Some(fr) = plane(first) else {
            return Err(format!("the fillet of {} runs out onto `{}` ({}): a fillet ending on a curved face is rung 3",
                names(),b.faces[first].name,b.faces[first].surface.kind()));
        };
        if let Some(&f) = others.iter().find(|&&f| !plane(f).is_some_and(|g| 1.-dot(g.z,fr.z).abs() <= 1e-9
            && dot(sub(g.o,fr.o),fr.z).abs() <= tol)) {
            return Err(format!("the fillet of {} ends where `{}` and `{}` meet: a fillet ending at a corner is rung 3",
                names(),b.faces[first].name,b.faces[f].name));
        }
        // not a face either runs on into smoothly (a chain of traced and straight edges)
        let normals = [0,1].map(|k| unit(surfaces[k].gradient(p)));
        if normals.iter().any(|&n| 1.-dot(n,fr.z).abs() <= 1e-9) {
            return Err(format!("the fillet of {} runs on smoothly into `{}`: a chain of traced and straight edges is rung 3",
                names(),b.faces[first].name));
        }
        // the side the run lies on, along its edge leaving the vertex
        let e = &b.edges[*chain.iter().find(|&&e| b.edges[e].v.contains(&v)).unwrap()];
        let EdgeCurve::Curve(c) = &e.curve else { return Err(format!("{} meet at a point",names())) };
        let leaving = if e.v[0] == v { unit(c.tangent(e.t[0])) } else { scale(unit(c.tangent(e.t[1])),-1.) };
        let s = dot(leaving,fr.z);
        if s.abs() < MIN_TURN.dsin() {
            return Err(format!("the fillet of {} meets `{}` along it",names(),b.faces[first].name));
        }
        // the ball's centre over the vertex: `r` from each face along its normal toward the ball
        let m = [0,1].map(|k| scale(normals[k],sides[k]));
        let guess = add(p,scale(add(m[0],m[1]),r/(1.+dot(m[0],m[1]))));
        ends.push(Stop {vertex:v,o:p,keep:if s > 0. { fr.z } else { scale(fr.z,-1.) },centre:onto.project(guess),
            faces:others.into_iter().collect()});
    }
    // each strip's surface, its seam turned away from the loop, and whether the loop winds round
    // it (a branch pipe's crotch round the branch)
    let loop_pts: Vec<V> = chain.iter().flat_map(|&e| { let ed = &b.edges[e];
        (0..16).map(move |k| ed.point(ed.t[0]+(ed.t[1]-ed.t[0])*k as f64/16.,&b.vertices)) }).collect();
    let strips = [0,1].map(|k| reframed(&surfaces[k],&loop_pts));
    let winding = [0,1].map(|k| ends.is_empty() && winds(&strips[k],&loop_pts));
    if winding[0] && winding[1] {
        return Err(format!("the fillet of {} winds round both faces: not built yet",names()));
    }
    let (t0,span) = if ends.is_empty() {
        let Some(period) = spine.period() else {
            return Err(format!("the ball between {} rolls off the end of their meeting",names()));
        };
        // the canal starts where its contact with a winding strip crosses the strip's iso line
        // through a vertex of the loop, so the strip's seam runs between vertices both already have
        let mut t0 = spine.inverse(seed);
        let vertex = b.vertices[b.edges[chain[0]].v[0] as usize].p;
        if let Some(k) = (0..2).find(|&k| winding[k]) {
            let s = &strips[k];
            let target = s.inverse(vertex)[0];
            let contact = |t: f64| { let c = spine.point(t); add(c,scale(toward(k,c),r)) };
            t0 = crossing(&|t| s.inverse(contact(t))[0],target,s.periods()[0].unwrap_or(TAU),t0,period)
                .ok_or_else(|| format!("the fillet of {}: its contact never crosses the strip's seam",names()))?;
        }
        (t0,period)
    } else {
        // from one end's centre to the other's through the seed, then on past each end's plane
        // until the whole section there (the centre, the arc, the corner, within its reach of the
        // centre) lies beyond it; the ends in the order the stretch runs
        let at_seed = spine.inverse(seed);
        let ts = [0,1].map(|k| spine.inverse(ends[k].centre));
        let (mut lo,mut hi,[a,z]) = match spine.period() {
            Some(p) => {
                let back = |t: f64| (at_seed-t).rem_euclid(p);
                let fwd = |t: f64| (t-at_seed).rem_euclid(p);
                if back(ts[0]) <= back(ts[1]) { (at_seed-back(ts[0]),at_seed+fwd(ts[1]),[0,1]) }
                else { (at_seed-back(ts[1]),at_seed+fwd(ts[0]),[1,0]) }
            }
            None if ts[0] <= ts[1] => (ts[0],ts[1],[0,1]),
            None => (ts[1],ts[0],[1,0]),
        };
        if ends.len() == 2 && ends[0].vertex == ends[1].vertex || !(lo <= at_seed && at_seed <= hi) {
            return Err(format!("the run of {} does not lie between its ends",names()));
        }
        let (floor,ceiling) = match &spine {
            Curve::Traced(c) if !c.closed => (0.,c.pts.len() as f64-1.),
            _ => (f64::NEG_INFINITY,f64::INFINITY),
        };
        let length = spine.period().unwrap_or(ceiling-floor);
        let step = if length.is_finite() { length/2048. } else { r/64. };
        let past = |t: f64,e: &Stop| {
            let c = onto.project(spine.point(t));
            let w = dot(toward(0,c),toward(1,c)).clamp(-1.,1.).dacos();
            dot(sub(c,e.o),e.keep) < -(r/(0.5*w).dcos()+0.1*r)
        };
        let room = |lo: f64,hi: f64| lo >= floor && hi <= ceiling && spine.period().is_none_or(|p| hi-lo < p);
        while !past(lo,&ends[a]) { lo -= step; if !room(lo,hi) { break } }
        while !past(hi,&ends[z]) { hi += step; if !room(lo,hi) { break } }
        if !room(lo,hi) {
            return Err(format!("the ball of {} cannot roll on past where `{}` and `{}` cut it off",
                names(),b.faces[ends[a].faces[0]].name,b.faces[ends[z].faces[0]].name));
        }
        ends.swap(0,a);
        (lo,hi-lo)
    };
    // the spine read along a smooth guide: a trace's own parameter turns at every point it was
    // marched through, which no smooth fit follows; a cubic through its points, by chord length,
    // projected back onto both offsets point by point, is exact and smooth between its ends
    let guide = {
        let n = 256;
        let pts: Vec<V> = (0..=n).map(|k| spine.point(t0+span*k as f64/n as f64)).collect();
        let mut ts = vec![0.];
        for w in pts.windows(2) { let last = *ts.last().unwrap(); ts.push(last+distance(w[0],w[1])); }
        let total = *ts.last().unwrap();
        let ts: Vec<f64> = ts.into_iter().map(|t| t/total).collect();
        super::nurbs::interpolate(&pts,&ts,3).ok_or("the spine could not be read smoothly")?
    };
    // the ball's centre at `u` and the directions to its contacts, each projected once however
    // many points of the canal's arc there are read (the fit asks each `u` for its whole row)
    let frames = std::cell::RefCell::new(std::collections::BTreeMap::<u64,(V,[V;2])>::new());
    let frame_at = |u: f64| -> (V,[V;2]) {
        let u = u.clamp(0.,1.);
        if let Some(&f) = frames.borrow().get(&u.to_bits()) { return f }
        let c = onto.project(guide.point(u));
        let f = (c,[toward(0,c),toward(1,c)]);
        frames.borrow_mut().insert(u.to_bits(),f);
        f
    };
    let c_at = |u: f64| frame_at(u).0;
    let canal = |u: f64,v: f64| { let (c,e) = frame_at(u); add(c,scale(slerp(e[0],e[1],v),r)) };
    // the corner's angle all the way round: no fold, no ball wedged flat
    let mut reach: f64 = 0.;
    for k in 0..=256 {
        let (_,e) = frame_at(k as f64/256.);
        let w = dot(e[0],e[1]).clamp(-1.,1.).dacos();
        if !(w > MIN_TURN && w < std::f64::consts::PI-MIN_TURN) {
            return Err(format!("the ball between {} wedges flat somewhere along their meeting",names()));
        }
        // the corner stands `r / cos(w / 2)` from the centre, `w` the angle between the contacts
        reach = reach.max(r/(0.5*w).dcos());
    }
    // **a ball no larger than each face holds**: the band of a face it rolls on — where it stands
    // nearer the other face's surface than its contact does — crossed by none of the face's other
    // edges. Read against the other surface offset to the band's height (`curve_surface`)
    for k in 0..2 {
        let o = 1-k;
        let contact = |u: f64| { let (c,e) = frame_at(u); add(c,scale(e[k],r)) };
        let height = (0..=256).map(|i| sides[o]*surfaces[o].implicit(contact(i as f64/256.))).fold(0_f64,f64::max)*1.01+tol;
        let level = surfaces[o].offset(sides[o]*height).ok_or_else(|| format!("the band of {} cannot be read",names()))?;
        // an end's face cuts the fillet off: the ball crosses its edges with these two; and an
        // edge between two faces of one side is no edge to it
        let ending = |e: u32| uses[e as usize].iter().any(|u| ends.iter().any(|s| s.faces.contains(&u.0))
            || u.0 != uses[e as usize][0].0 && sides_faces[k].contains(&u.0) && sides_faces[k].contains(&uses[e as usize][0].0));
        for &fi in &sides_faces[k] {
        let face = &b.faces[fi];
        let seams = face.seams();
        for c in face.loops.iter().flatten() {
            if chain.contains(&(c.edge as usize)) || seams.contains(&c.edge) || ending(c.edge) { continue }
            let g = &b.edges[c.edge as usize];
            let EdgeCurve::Curve(gc) = &g.curve else { continue };
            let at_start = sides[o]*surfaces[o].implicit(gc.point(g.t[0]));
            let crosses = match super::query::curve_surface(gc,g.t,&level,tol) {
                super::query::Meets::At(roots) => !roots.is_empty(),
                super::query::Meets::Along => true,
            };
            if crosses || (at_start > tol && at_start < height) {
                return Err(format!("the ball of {} is larger than `{}` can hold",names(),face.name));
            }
        }
        }
    }
    // fitted to the bar asked, or as near it as the densest net comes, no worse than a ten-millionth
    // of the boundary: its contacts are read at what was measured (`Edge::tol`)
    let bar = tol.max(1e-7*b.size());
    let (net,net_err) = fit_net(&canal,3,3,64,4,tol).ok_or("the canal could not be fitted")?;
    if net_err > bar { return Err(format!("the canal between {} misses its fit by {net_err:e}",names())); }
    let (spine_fit,spine_err) = fit_curve(&c_at,3,64,tol).ok_or("the spine could not be fitted")?;
    if spine_err > bar { return Err(format!("the spine between {} misses its fit by {spine_err:e}",names())); }
    // a run cut off by a plane crosses it only where it is run on past it (once for each end in
    // it): a run turning back across it would be cut there too
    let rails = [spine_fit.clone(),net.column(0),net.column(net.poles[0].len()-1)];
    for e in &ends {
        let plane = Surface::Plane(Frame::about(e.o,e.keep));
        let at = ends.iter().filter(|f| dot(f.keep,e.keep) >= 1.-1e-9 && dot(sub(f.o,e.o),e.keep).abs() <= tol).count();
        for c in &rails {
            let crossings = match super::query::curve_surface(&Curve::BSpline(Arc::new(c.clone())),c.domain(),&plane,tol) {
                super::query::Meets::At(roots) => roots.len(),
                super::query::Meets::Along => 0,
            };
            if crossings != at {
                return Err(format!("the fillet of {} turns back across the plane of `{}` it ends at",names(),b.faces[e.faces[0]].name));
            }
        }
    }
    let reversed = [face_a.reversed,face_b.reversed];
    let piece = if ends.is_empty() { build(b,&chain,&net,&strips,reversed,concave,tol,net_err)? } else {
        let meeting = super::geom::Traced {a:surfaces[0].clone(),b:surfaces[1].clone(),pts:Vec::new(),closed:false};
        let run = build_open(b,&chain,&net,&strips,&meeting,&frame_at,&guide,&ends,reversed,concave,tol,net_err)?;
        trimmed(run,&ends,tol)?
    };
    let contacts = [net.column(0),net.column(net.poles[0].len()-1)];
    let trims = ends.iter().map(|e| (e.o,e.keep)).collect();
    Ok(Rolled {piece,chain,spine:spine_fit,contacts,concave,r,reach,trims})
}

/// An end of a run that ends: its vertex (at `o`), the faces there besides the run's two (all in
/// one plane), the plane's normal toward the run, and the ball's centre over the vertex.
struct Stop { vertex: u32,o: V,keep: V,centre: V,faces: Vec<usize> }

/// The point where `meeting` (two surfaces' meeting) crosses the plane through `o` square to `n`,
/// from `guess`: projected onto the meeting, then along its tangent to the plane, until it stays.
fn crossing_plane(meeting: &super::geom::Traced,o: V,n: V,guess: V) -> V {
    let mut k = meeting.project(guess);
    for _ in 0..32 {
        let t = cross(meeting.a.gradient(k),meeting.b.gradient(k));
        let along = dot(t,n);
        if along == 0. { break }
        let step = dot(sub(o,k),n)/along;
        let next = meeting.project(add(k,scale(t,step)));
        let moved = distance(next,k);
        k = next;
        if moved <= 1e-15*norm(k).max(1.) { break }
    }
    k
}

/// A curve on `meeting` from `a` to `z` (both on it), through `along(s)` projected onto it for `s`
/// in `[0, 1]`, fitted within `tol`: its ends exactly `a` and `z`.
fn fitted(meeting: &super::geom::Traced,a: V,z: V,along: &dyn Fn(f64) -> V,tol: f64) -> Result<(Curve,f64),String> {
    let at = |s: f64| if s <= 0. { a } else if s >= 1. { z } else { meeting.project(along(s)) };
    let (c,err) = fit_curve(&at,3,16,tol).ok_or("a fillet's run-on could not be fitted")?;
    Ok((Curve::BSpline(Arc::new(c)),err))
}

/// The piece of a run that ends, rolled on past both ends (`ends`, in the order the net's `u`
/// runs): the canal face over `net`, on each surface the strip between the run (`chain`, edges of
/// `b`, run on past each end along the faces' `meeting`) and the canal's contact with it, and at
/// each end of the net a cap in the plane of its arc. `frame_at` is the ball's centre and the
/// directions to its contacts at `u`, `guide` the spine `u` follows.
#[allow(clippy::too_many_arguments)]
fn build_open(b: &Brep,chain: &[usize],net: &Net,surfaces: &[Surface;2],meeting: &super::geom::Traced,
    frame_at: &dyn Fn(f64) -> (V,[V;2]),guide: &BSpline,ends: &[Stop],face_reversed: [bool;2],concave: bool,tol: f64,fit: f64)
    -> Result<Brep,String> {
    let [[u0,u1],[v0,v1]] = net.domain();
    let r = distance(net.point(u0,v0),frame_at(0.).0);
    let mut out = Brep::default();
    let column = |j: usize| Curve::BSpline(Arc::new(net.column(j)));
    let row = |i: usize| Curve::BSpline(Arc::new(BSpline {degree:net.dv,knots:net.vknots.clone(),poles:net.poles[i].clone(),
        weights:net.weights.as_ref().map(|w| w[i].clone())}));
    let (last_u,last_v) = (net.poles.len()-1,net.poles[0].len()-1);
    // the section's plane at `u`: through the ball's centre, square to the spine (the plane of its
    // arc), its normal along `u`
    let section = |u: f64| -> (V,V,[V;2]) {
        let (c,e) = frame_at(u);
        let n = unit(cross(e[0],e[1]));
        (c,if dot(n,guide.d2(u.clamp(0.,1.)).1) < 0. { scale(n,-1.) } else { n },e)
    };
    // where the faces meet in the section at `u`: across the ball from its centre
    let corner = |u: f64| { let (c,n,e) = section(u);
        crossing_plane(meeting,c,n,add(c,scale(add(e[0],e[1]),r/(1.+dot(e[0],e[1]))))) };
    let mut worst = fit;
    // the net's corners: contact `k` at each end
    let ct = [[net.point(u0,v0),net.point(u1,v0)],[net.point(u0,v1),net.point(u1,v1)]];
    let cv = ct.map(|p| p.map(|q| out.vertex(q)));
    let contact = [0,1].map(|k| out.edge(EdgeCurve::Curve(column([0,last_v][k])),[u0,u1],cv[k]) as usize);
    let arc = [0,1].map(|j| out.edge(EdgeCurve::Curve(row([0,last_u][j])),[v0,v1],[cv[0][j],cv[1][j]]) as usize);
    // the run's edges, copied with their vertices
    let mut copied = std::collections::BTreeMap::<u32,u32>::new();
    let mut run = Vec::new();
    for &e in chain {
        let ed = &b.edges[e];
        let vs = ed.v.map(|v| *copied.entry(v).or_insert_with(|| out.vertex(b.vertices[v as usize].p)));
        let id = out.edge(ed.curve.clone(),ed.t,vs);
        out.edges[id as usize].tol = ed.tol;
        run.push(id as usize);
    }
    // at each end: the section through the vertex (where the corner passes it), the run on along
    // the faces' meeting from the cap's corner to the vertex, and on each face the cap's curve from
    // the contact to the corner
    let mut caps: Vec<[usize;3]> = Vec::new();
    for (j,stop) in ends.iter().enumerate() {
        let vertex = b.vertices[stop.vertex as usize].p;
        let ahead = |u: f64| { let (c,n,_) = section(u); dot(sub(vertex,c),n) };
        let n = 512;
        let us: Vec<f64> = (0..=n).map(|i| i as f64/n as f64).collect();
        let pass = if j == 0 { (0..n).find(|&i| ahead(us[i]) >= 0. && ahead(us[i+1]) < 0.) }
            else { (0..n).rev().find(|&i| ahead(us[i]) >= 0. && ahead(us[i+1]) < 0.) };
        let i = pass.ok_or("a fillet's run-on never passes the vertex it ends at")?;
        let at = crate::roots::bracketed_root(|u| Some(ahead(u)),us[i],ahead(us[i]),us[i+1],ahead(us[i+1]),1e-15);
        let end = [u0,u1][j];
        let k_end = corner(end);
        let k_v = out.vertex(k_end);
        let (from,to) = if j == 0 { (end,at) } else { (at,end) };
        let (curve,err) = fitted(meeting,if j == 0 { k_end } else { vertex },if j == 0 { vertex } else { k_end },
            &|s| corner(from+(to-from)*s),tol)?;
        worst = worst.max(err);
        let vv = copied[&stop.vertex];
        let ext = out.edge(EdgeCurve::Curve(curve),[0.,1.],if j == 0 { [k_v,vv] } else { [vv,k_v] }) as usize;
        out.edges[ext].tol = out.edges[ext].tol.max(err);
        run.push(ext);
        let (c,nrm,_) = section(end);
        let cap_plane = Surface::Plane(Frame::about(c,nrm));
        let across: Vec<usize> = (0..2).map(|k| -> Result<usize,String> {
            let on = super::geom::Traced {a:surfaces[k].clone(),b:cap_plane.clone(),pts:Vec::new(),closed:false};
            let p = ct[k][j];
            let (curve,err) = fitted(&on,p,k_end,&|s| crate::space::lerp(p,k_end,s),tol)?;
            worst = worst.max(err);
            let e = out.edge(EdgeCurve::Curve(curve),[0.,1.],[cv[k][j],k_v]) as usize;
            out.edges[e].tol = out.edges[e].tol.max(err);
            Ok(e)
        }).collect::<Result<_,_>>()?;
        caps.push([arc[j],across[0],across[1]]);
        // the cap: its piece lies toward the run, its outward normal away along the spine
        let outward = if j == 0 { scale(nrm,-1.) } else { nrm };
        let l = closed_loop(&out,&caps[j],&cap_plane)?;
        let (_,su,sv) = cap_plane.d1(cap_plane.inverse(c));
        let reversed = dot(cross(su,sv),outward) < 0.;
        out.faces.push(Face {surface:cap_plane,reversed,loops:vec![turned(l.coedges,(l.area > 0.) == reversed)],
            name:["start","end"][j].into()});
    }
    // the canal: the contacts along `u`, the arcs at its ends
    let line = |a: Uv,z: Uv| Pcurve::Line {a,b:z};
    let canal_loop = vec![
        Coedge {edge:contact[0] as u32,reversed:false,pcurve:line([u0,v0],[u1,v0])},
        Coedge {edge:arc[1] as u32,reversed:false,pcurve:line([u1,v0],[u1,v1])},
        Coedge {edge:contact[1] as u32,reversed:true,pcurve:line([u0,v1],[u1,v1])},
        Coedge {edge:arc[0] as u32,reversed:true,pcurve:line([u0,v0],[u0,v1])},
    ];
    let surface = Surface::BSpline(Frame::new([0.;3],[0.,0.,1.],[1.,0.,0.]),Arc::new(net.clone()));
    let (m,su,sv) = surface.d1([0.5*(u0+u1),0.5*(v0+v1)]);
    let spine_side = {
        let (a,z) = (net.point(0.5*(u0+u1),v0),net.point(0.5*(u0+u1),v1));
        unit(sub(scale(add(a,z),0.5),m))
    };
    let canal_reversed = dot(cross(su,sv),spine_side) < 0.;
    out.faces.push(Face {surface,reversed:canal_reversed,loops:vec![turned(canal_loop,canal_reversed)],name:"round".into()});
    // each strip: the run (on past both ends), a cap's curve, the contact, the other cap's curve
    for k in 0..2 {
        let reversed = face_reversed[k] != concave;
        let s = &surfaces[k];
        let mut es = run.clone();
        es.extend([caps[0][1+k],contact[k],caps[1][1+k]]);
        let l = closed_loop(&out,&es,s)?;
        let want = !reversed;
        let coedges = if (l.area > 0.) == want { l.coedges } else { reverse(l.coedges) };
        out.faces.push(Face {surface:s.clone(),reversed,loops:vec![coedges],name:["a","b"][k].into()});
    }
    super::json::measure(&mut out);
    for c in contact { let e = &mut out.edges[c]; e.tol = e.tol.max(fit); }
    out.check(10.*tol.max(worst))?;
    Ok(out)
}

/// A run's piece cut off at each end's plane, kept on the run's side: each end a block standing on
/// the plane, as large as the piece, in common with it.
fn trimmed(mut piece: Brep,ends: &[Stop],tol: f64) -> Result<Brep,String> {
    for stop in ends {
        let (lo,hi) = piece.bounds();
        let size = 2.*norm(sub(hi,lo))+distance(scale(add(lo,hi),0.5),stop.o);
        let f = Frame::about(stop.o,stop.keep);
        let at = |x: f64,y: f64| f.at([x,y,0.]);
        let corners = [at(-size,-size),at(size,-size),at(size,size),at(-size,size)];
        let edges = (0..4).map(|k| super::build::ProfileEdge::Line {a:corners[k],b:corners[(k+1)%4]}).collect();
        let block = super::build::prism(&super::build::Profile {origin:stop.o,normal:stop.keep,loops:vec![edges],names:vec![]},0.,size)?;
        piece = super::recipe::combined(&piece,&block,super::boolean::Op::Common,0.)
            .map_err(|m| format!("a fillet's run cannot be cut off where it ends: {m}"))?;
    }
    let _ = tol;
    Ok(piece)
}

/// The piece: the canal face over `net` (seamed along its first column), and on each surface the
/// strip between the loop `chain` (edges of `b`) and the canal's contact with it.
fn build(b: &Brep,chain: &[usize],net: &Net,surfaces: &[Surface;2],face_reversed: [bool;2],concave: bool,tol: f64,fit: f64)
    -> Result<Brep,String> {
    let [[u0,u1],[v0,v1]] = net.domain();
    let mut out = Brep::default();
    let column = |j: usize| Curve::BSpline(Arc::new(net.column(j)));
    let row = |i: usize| Curve::BSpline(Arc::new(BSpline {degree:net.dv,knots:net.vknots.clone(),poles:net.poles[i].clone(),
        weights:net.weights.as_ref().map(|w| w[i].clone())}));
    let last_v = net.poles[0].len()-1;
    let (va,vb) = (out.vertex(net.point(u0,v0)),out.vertex(net.point(u0,v1)));
    // the contacts, each one closed edge, and the canal's seam between them
    let contact = [out.edge(EdgeCurve::Curve(column(0)),[u0,u1],[va,va]),out.edge(EdgeCurve::Curve(column(last_v)),[u0,u1],[vb,vb])];
    let seam = out.edge(EdgeCurve::Curve(row(0)),[v0,v1],[va,vb]);
    let line = |a: Uv,z: Uv| Pcurve::Line {a,b:z};
    let canal_loop = vec![
        Coedge {edge:contact[0],reversed:false,pcurve:line([u0,v0],[u1,v0])},
        Coedge {edge:seam,reversed:false,pcurve:line([u1,v0],[u1,v1])},
        Coedge {edge:contact[1],reversed:true,pcurve:line([u0,v1],[u1,v1])},
        Coedge {edge:seam,reversed:true,pcurve:line([u0,v0],[u0,v1])},
    ];
    let surface = Surface::BSpline(Frame::new([0.;3],[0.,0.,1.],[1.,0.,0.]),Arc::new(net.clone()));
    // the piece lies outside the ball: its canal face looks toward the spine
    let (m,su,sv) = surface.d1([0.5*(u0+u1),0.5*(v0+v1)]);
    let spine_side = {
        // the ball's centre a radius behind the middle of the arc
        let (a,z) = (net.point(0.5*(u0+u1),v0),net.point(0.5*(u0+u1),v1));
        let chord = scale(add(a,z),0.5);
        unit(sub(chord,m))
    };
    let canal_reversed = dot(cross(su,sv),spine_side) < 0.;
    out.faces.push(Face {surface,reversed:canal_reversed,loops:vec![turned(canal_loop,canal_reversed)],name:"round".into()});
    // the loop's edges, copied with their vertices
    let mut copied = std::collections::BTreeMap::<u32,u32>::new();
    let mut edge_of = Vec::new();
    for &e in chain {
        let ed = &b.edges[e];
        let vs = ed.v.map(|v| *copied.entry(v).or_insert_with(|| out.vertex(b.vertices[v as usize].p)));
        let id = out.edge(ed.curve.clone(),ed.t,vs);
        out.edges[id as usize].tol = ed.tol;
        edge_of.push(id);
    }
    for k in 0..2 {
        let name = ["a","b"][k];
        // the strip's surface: the face's own, its material away from the piece at a concave edge
        let reversed = face_reversed[k] != concave;
        let s = &surfaces[k];
        let mut loops = vec![closed_loop(&out,&[contact[k] as usize],s)?,closed_loop(&out,&edge_of.iter().map(|&e| e as usize).collect::<Vec<_>>(),s)?];
        let periods = s.periods();
        let winds = |l: &Loop| periods[0].is_some_and(|p| (l.turn[0]/p).round().abs() >= 1.);
        let face_loops = if loops.iter().all(|l| !winds(l)) {
            // a hole and its rim: the one enclosing the other runs round it the face's way
            loops.sort_by(|x,y| y.area.abs().total_cmp(&x.area.abs()));
            loops.into_iter().enumerate().map(|(i,l)| {
                let outer = i == 0;
                let ccw = l.area > 0.;
                let want = outer != reversed;
                if ccw == want { l.coedges } else { reverse(l.coedges) }
            }).collect()
        } else if loops.iter().all(winds) {
            // both round the surface's period: one loop through a seam along the surface's iso
            // line from a vertex of the loop up to the contact
            vec![seamed(&mut out,loops,s,reversed)?]
        } else {
            return Err(format!("the strip of the fillet's `{name}` side winds round its surface on one edge and not the other"));
        };
        out.faces.push(Face {surface:s.clone(),reversed,loops:face_loops,name:name.into()});
    }
    super::json::measure(&mut out);
    // a contact is the canal's iso line, off its face's surface by the fit at most — which a
    // measure at the net's own nodes (where the fit is exact) does not see
    for c in contact { let e = &mut out.edges[c as usize]; e.tol = e.tol.max(fit); }
    out.check(10.*tol.max(fit))?;
    Ok(out)
}

/// A closed walk of edges on a surface: its uses, in order, the turn its parameters make over the
/// walk (whole periods where it winds) and the area it sweeps in them.
struct Loop { coedges: Vec<Coedge>,turn: Uv,area: f64 }

/// The edges `es` (of `b`) walked as one closed loop on `s`, each use's pcurve the edge inverted
/// onto `s` continuously from the last.
fn closed_loop(b: &Brep,es: &[usize],s: &Surface) -> Result<Loop,String> {
    // order the edges end to start
    let mut left: Vec<usize> = es.to_vec();
    let first = left.remove(0);
    let mut walk = vec![(first,false)];
    let mut at = b.edges[first].v[1];
    while !left.is_empty() {
        let k = left.iter().position(|&e| b.edges[e].v.contains(&at)).ok_or("a fillet's loop does not close")?;
        let e = left.remove(k);
        let rev = b.edges[e].v[1] == at && b.edges[e].v[0] != at;
        at = if rev { b.edges[e].v[0] } else { b.edges[e].v[1] };
        walk.push((e,rev));
    }
    let periods = s.periods();
    let mut prev: Option<Uv> = None;
    let mut start: Option<Uv> = None;
    let mut pts: Vec<Uv> = Vec::new();
    let mut coedges = Vec::new();
    for (e,rev) in walk {
        let ed = &b.edges[e];
        let EdgeCurve::Curve(c) = &ed.curve else { return Err("a degenerate edge in a fillet's loop".into()) };
        // walked in the use's direction, the parameters unwrapped step by step
        let n = 64;
        let uv_at = |t: f64,hint: Option<Uv>| -> Uv {
            let q = c.point(t);
            match hint { Some(h) => unwrap(s.inverse_near(q,h),h,periods),None => s.inverse(q) }
        };
        let ts: Vec<f64> = (0..=n).map(|k| ed.t[0]+(ed.t[1]-ed.t[0])*k as f64/n as f64).collect();
        let order: Vec<f64> = if rev { ts.iter().rev().copied().collect() } else { ts };
        let mut run = Vec::new();
        for &t in &order { let uv = uv_at(t,prev); prev = Some(uv); run.push(uv); }
        if start.is_none() { start = Some(run[0]); }
        let (from,to) = (run[0],*run.last().unwrap());
        // the pcurve runs in the edge's own direction
        let (a,z) = if rev { (to,from) } else { (from,to) };
        coedges.push(Coedge {edge:e as u32,reversed:rev,pcurve:Pcurve::Inverse {a,b:z}});
        pts.extend(run);
    }
    let turn = { let s0 = start.unwrap(); let e = prev.unwrap(); [e[0]-s0[0],e[1]-s0[1]] };
    let n = pts.len();
    let area = (0..n).map(|i| { let (p,q) = (pts[i],pts[(i+1)%n]); p[0]*q[1]-q[0]*p[1] }).sum::<f64>()/2.;
    Ok(Loop {coedges,turn,area})
}

/// A loop's uses walked the other way round.
fn reverse(l: Vec<Coedge>) -> Vec<Coedge> {
    l.into_iter().rev().map(|c| Coedge {reversed:!c.reversed,..c}).collect()
}

/// A face's loop the right way round for its side: counter-clockwise in its parameters as built,
/// reversed for a reversed face.
fn turned(l: Vec<Coedge>,reversed: bool) -> Vec<Coedge> { if reversed { reverse(l) } else { l } }

/// Two loops each once round a surface's period — the contact first, the original edges second,
/// each starting at a vertex on one iso line of the surface (constant first parameter) — joined
/// into one by a seam along that line between them, used both ways.
fn seamed(out: &mut Brep,loops: Vec<Loop>,s: &Surface,reversed: bool) -> Result<Vec<Coedge>,String> {
    let [contact,edges]: [Loop;2] = loops.try_into().map_err(|_| "a strip has two loops".to_string())?;
    let period = s.periods()[0].ok_or("a winding strip is on a periodic surface")?;
    let start = |l: &Loop| -> (Uv,u32) {
        let c = &l.coedges[0];
        let (e,v) = (&out.edges[c.edge as usize],out.ends(c)[0]);
        let t = if c.reversed { e.t[1] } else { e.t[0] };
        (c.pcurve.at(t,e,s,&out.vertices),v)
    };
    let (low,v_low) = start(&edges);
    let (high,v_high) = start(&contact);
    // the seam, along the iso line from the loop's vertex to the contact's
    let (p,q) = (out.vertices[v_low as usize].p,out.vertices[v_high as usize].p);
    let curve = iso(s,low[0],p,q)?;
    let t = [curve.inverse(p),curve.inverse(q)];
    let seam = out.edge(EdgeCurve::Curve(curve),t,[v_low,v_high]);
    // the contact runs the other way round to the loop, from the far end of the seam
    let w = (edges.turn[0]/period).round();
    let contact = if (contact.turn[0]/period).round() == w { reverse(contact.coedges) } else { contact.coedges };
    let shift = |c: Coedge,du: f64| match c.pcurve {
        Pcurve::Inverse {a,b} => Coedge {pcurve:Pcurve::Inverse {a:[a[0]+du,a[1]],b:[b[0]+du,b[1]]},..c},
        _ => c,
    };
    let u_end = low[0]+w*period;
    let contact: Vec<Coedge> = {
        // its first use starts where the seam arrives, `u_end`
        let first = &contact[0];
        let e = &out.edges[first.edge as usize];
        let t = if first.reversed { e.t[1] } else { e.t[0] };
        let at = first.pcurve.at(t,e,s,&out.vertices)[0];
        let du = u_end-at;
        contact.into_iter().map(|c| shift(c,du)).collect()
    };
    let mut l = edges.coedges;
    l.push(Coedge {edge:seam,reversed:false,pcurve:Pcurve::Line {a:[u_end,low[1]],b:[u_end,high[1]]}});
    l.extend(contact);
    l.push(Coedge {edge:seam,reversed:true,pcurve:Pcurve::Line {a:[low[0],low[1]],b:[low[0],high[1]]}});
    // counter-clockwise for the face's side: the loop runs `w` turns at `low`, back at `high`
    let ccw = w*(high[1]-low[1]) > 0.;
    Ok(if ccw == !reversed { l } else { reverse(l) })
}

/// The iso line of `s` at first parameter `u` from `p` to `q` (both on it): a generator of a
/// cylinder or a cone, a meridian circle of a sphere or a torus.
fn iso(s: &Surface,u: f64,p: V,q: V) -> Result<Curve,String> {
    match *s {
        Surface::Cylinder(..) | Surface::Cone(..) => Ok(Curve::Line {p,d:unit(sub(q,p))}),
        Surface::Sphere(f,r) | Surface::Torus(f,_,r) => {
            let (su,cu) = u.dsin_cos();
            let radial = f.dir([cu,su,0.]);
            let centre = match *s { Surface::Torus(_,big,_) => add(f.o,scale(radial,big)),_ => f.o };
            Ok(Curve::Circle(Frame {o:centre,x:radial,y:f.z,z:cross(radial,f.z)},r))
        }
        _ => Err(format!("a {}'s iso line is not built yet",s.kind())),
    }
}

/// `s` with its seam (first parameter 0) turned away from the points `pts` (on it), so a loop of
/// them that does not wind round the surface stays clear of its period's cut; a sphere's poles
/// turned away too.
fn reframed(s: &Surface,pts: &[V]) -> Surface {
    let mean = scale(pts.iter().fold([0.;3],|m,&p| add(m,p)),1./pts.len().max(1) as f64);
    match *s {
        Surface::Cylinder(f,r) | Surface::Cone(f,r,_) => {
            let away = sub(mean,f.o);
            let radial = sub(away,scale(f.z,dot(away,f.z)));
            if norm(radial) < 1e-12 { return s.clone() }
            let x = scale(unit(radial),-1.);
            let frame = Frame {x,y:cross(f.z,x),..f};
            match *s { Surface::Cone(_,_,a) => Surface::Cone(frame,r,a),_ => Surface::Cylinder(frame,r) }
        }
        Surface::Sphere(f,r) => {
            let m = sub(mean,f.o);
            if norm(m) < 1e-12 { return s.clone() }
            let x = scale(unit(m),-1.);
            Surface::Sphere(Frame::new(f.o,Frame::about(f.o,x).x,x),r)
        }
        _ => s.clone(),
    }
}

/// Whether a closed loop through `pts` (in order) winds round `s`'s first period.
fn winds(s: &Surface,pts: &[V]) -> bool {
    let Some(period) = s.periods()[0] else { return false };
    let mut prev = s.inverse(pts[0]);
    let start = prev;
    for &p in pts[1..].iter().chain(std::iter::once(&pts[0])) {
        prev = unwrap(s.inverse_near(p,prev),prev,s.periods());
    }
    ((prev[0]-start[0])/period).round().abs() >= 1.
}

/// The parameter in `[from, from + span)` where `u(t)` (a parameter going round a period, read
/// continuously) passes `target` modulo the period, or none.
fn crossing(u: &dyn Fn(f64) -> f64,target: f64,period: f64,from: f64,span: f64) -> Option<f64> {
    let n = 1024;
    let at = |k: usize| from+span*k as f64/n as f64;
    let mut last = u(at(0));
    let level = |x: f64,near: f64| { let mut x = x; x += ((near-x)/period).round()*period; x-target-((near-target)/period).round()*period };
    for k in 0..n {
        let next = { let x = u(at(k+1)); x+((last-x)/period).round()*period };
        let (fa,fb) = (level(last,last),level(next,last));
        if fa == 0. { return Some(at(k)) }
        if (fa < 0.) != (fb < 0.) {
            return Some(crate::roots::bracketed_root(|m| Some(level(u(m),last)),at(k),fa,at(k+1),fb,1e-15*span.abs().max(1.)))
        }
        last = next;
    }
    None
}

/// **A spherical corner** (rung 3): where three planar faces meet at `vertex` and a fillet of
/// radius `r` rounds all three of its edges, the ball touching all three faces sits at `centre`,
/// and the corner's piece is the corner's cell less the ball — the cell the trihedron at the vertex
/// cut by the three planes through `centre` square to its edges, its vertices the vertex, the feet
/// of the centre on the edges, the touches and the centre. `toward[i]` is the unit direction from
/// face `i` toward the ball (into the material at a convex corner, the void at a concave one), so
/// `(centre − vertex) · toward[i] = r`. Faces: `a`, `b`, `c` on the three faces, `near_ab`,
/// `near_bc`, `near_ca` the sections (each a neighbouring edge piece's cap), and `round`, the
/// sphere's triangle.
pub fn corner(vertex: V,centre: V,r: f64,toward: [V;3],tol: f64) -> Result<Brep,String> {
    let along = sub(centre,vertex);
    let pairs = [(0,1),(1,2),(2,0)];
    let dirs = corner_sections(vertex,centre,toward);
    let feet = dirs.map(|e| add(vertex,scale(e,dot(along,e))));
    let touches = toward.map(|m| sub(centre,scale(m,r)));
    let mut out = Brep::default();
    let pv = out.vertex(vertex);
    let fv = feet.map(|p| out.vertex(p));
    let tv = touches.map(|p| out.vertex(p));
    let line = |out: &mut Brep,a: V,z: V,va: u32,vz: u32| {
        let l = distance(a,z);
        out.edge(EdgeCurve::Curve(Curve::Line {p:a,d:scale(sub(z,a),1./l)}),[0.,l],[va,vz]) as usize
    };
    // along each edge from the vertex to its foot; across each face from a foot to its touch
    let runs: Vec<usize> = (0..3).map(|k| line(&mut out,vertex,feet[k],pv,fv[k])).collect();
    // across[k] = [from foot k to face i's touch, to face j's], (i, j) = pairs[k]
    let across: Vec<[usize;2]> = pairs.iter().enumerate().map(|(k,&(i,j))|
        [i,j].map(|f| line(&mut out,feet[k],touches[f],fv[k],tv[f]))).collect();
    // the ball's arc in each section, from one touch to the other about the centre
    let arcs: Vec<usize> = pairs.iter().enumerate().map(|(k,&(i,j))| {
        let x = unit(sub(touches[i],centre));
        let z = if dot(cross(x,sub(touches[j],centre)),dirs[k]) >= 0. { dirs[k] } else { scale(dirs[k],-1.) };
        let frame = Frame {o:centre,x,y:cross(z,x),z};
        let w = sub(touches[j],centre);
        let end = dot(w,frame.y).datan2(dot(w,frame.x));
        out.edge(EdgeCurve::Curve(Curve::Circle(frame,r)),[0.,end],[tv[i],tv[j]]) as usize
    }).collect();
    // a face's one loop, counter-clockwise in its parameters for its outward side
    let face = |out: &mut Brep,s: Surface,outward: V,es: &[usize],name: &str| -> Result<(),String> {
        let l = closed_loop(out,es,&s)?;
        let (_,su,sv) = s.d1(s.inverse(out.vertices[out.edges[es[0]].v[0] as usize].p));
        let reversed = dot(cross(su,sv),outward) < 0.;
        out.faces.push(Face {surface:s,reversed,loops:vec![turned(l.coedges,(l.area > 0.) == reversed)],name:name.into()});
        Ok(())
    };
    // on each face: vertex, foot, touch, foot — face i lies between sections i − 1 and i (pairs
    // (i − 1, i) and (i, i + 1)); the piece toward the ball from it
    for (i,name) in ["a","b","c"].into_iter().enumerate() {
        let (k0,k1) = ((i+2)%3,i);
        let s = Surface::Plane(Frame::about(vertex,toward[i]));
        face(&mut out,s,scale(toward[i],-1.),&[runs[k0],across[k0][1],across[k1][0],runs[k1]],name)?;
    }
    // each section: foot, touch, arc, touch; the piece lies toward the vertex from it
    for k in 0..3 {
        let s = Surface::Plane(Frame::about(centre,dirs[k]));
        face(&mut out,s,dirs[k],&[across[k][0],arcs[k],across[k][1]],["near_ab","near_bc","near_ca"][k])?;
    }
    // the ball's triangle, its frame's poles and seam turned away from it; the piece outside it
    let middle = unit(scale(add(add(toward[0],toward[1]),toward[2]),-1.));
    let sphere = Surface::Sphere(Frame::new(centre,Frame::about(centre,middle).x,scale(middle,-1.)),r);
    face(&mut out,sphere,scale(middle,-1.),&arcs,"round")?;
    super::json::measure(&mut out);
    out.check(10.*tol)?;
    Ok(out)
}

/// The directions of a corner's three edges from `vertex` toward the feet of `centre` on them —
/// each square to two faces' `toward`, in the order (0, 1), (1, 2), (2, 0) — the normals of its
/// sections through the centre.
pub fn corner_sections(vertex: V,centre: V,toward: [V;3]) -> [V;3] {
    let along = sub(centre,vertex);
    [(0,1),(1,2),(2,0)].map(|(i,j)| { let e = unit(cross(toward[i],toward[j])); if dot(e,along) < 0. { scale(e,-1.) } else { e } })
}
