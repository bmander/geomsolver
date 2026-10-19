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
use super::geom::{around,tolerance,Curve,Frame,Surface,Uv,V};
use super::nurbs::{fit_curve,fit_net,BSpline,Net};
use super::query::Located;
use super::topo::{reverse_loop,unwrap,Brep,Coedge,EdgeCurve,Face,Pcurve};
use crate::space::{add,cross,distance,dot,norm,scale,sub,unit};
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
    /// Where a run that ends is cut off, at each end; empty for a closed loop.
    pub trims: Vec<Trim>,
}


/// Along the arc of the unit sphere from `a` to `b` (the short way), a fraction `v`.
pub(super) fn slerp(a: V,b: V,v: f64) -> V {
    let w = dot(a,b).clamp(-1.,1.).dacos();
    let s = w.dsin();
    if s < 1e-12 { return a }
    add(scale(a,((1.-v)*w).dsin()/s),scale(b,(v*w).dsin()/s))
}

/// The run of `b`'s edges nearest `at` between two faces, as a ball rolled along it or a flat cut
/// along it reads it: the faces either side (each with every face on its surface the same way
/// out), the run's edges (closing on itself, or ending at the two vertices `stops`), the uses of
/// every edge, the edge's middle `p` and the directions into each face from there, whether the edge is concave, and the way each surface is offset toward the region
/// (`sides`).
struct Meeting<'b> {
    uses: Vec<Vec<(usize,usize,usize)>>,
    faces: [&'b Face;2],
    sides_faces: [Vec<usize>;2],
    chain: Vec<usize>,
    stops: Vec<u32>,
    p: V,
    into: [V;2],
    theta: f64,
    concave: bool,
    sides: [f64;2],
    surfaces: [Surface;2],
    names: String,
}

fn meeting<'b>(b: &'b Brep,at: V,tol: f64) -> Result<Meeting<'b>,String> {
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
        // (a meeting branches only where the two surfaces touch: there they are tangent, and no
        // ball touches both through it, of any size)
        return Err(format!("the meeting of {} branches where the two touch: no ball rolls through a point where they \
            are tangent",names()));
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
    Ok(Meeting {uses,faces:[face_a,face_b],sides_faces,chain,stops,p,into:[da,db],theta,concave,
        sides,surfaces,names:names()})
}

/// A curve read along `span` from `t0` as a smooth guide: a cubic through 257 of its points by
/// chord length. A trace's own parameter turns at every point it was marched through, which no
/// smooth fit follows; projected back point by point, the guide is exact and smooth between its
/// ends.
pub(super) fn smooth_guide(c: &Curve,t0: f64,span: f64) -> Option<BSpline> {
    let n = 256;
    let pts: Vec<V> = (0..=n).map(|k| c.point(t0+span*k as f64/n as f64)).collect();
    let mut ts = vec![0.];
    for w in pts.windows(2) { let last = *ts.last().unwrap(); ts.push(last+distance(w[0],w[1])); }
    let total = *ts.last().unwrap();
    let ts: Vec<f64> = ts.into_iter().map(|t| t/total).collect();
    super::nurbs::interpolate(&pts,&ts,3)
}

/// The ball of radius `r` rolled along the edge of `b` nearest `at`, the faces either side of it
/// meeting on a closed loop; `tol` the boundary's tolerance, and the bar the canal and the spine
/// are fitted within. `Err` says why it cannot be rolled exactly.
pub fn roll(b: &Brep,at: V,r: f64,tol: f64) -> Result<Rolled,String> { roll_across(b,at,r,tol,false) }

/// `roll`, its ball let cross a seam where one of the two faces goes on tangent into another
/// (`seams`): where a piece beyond that seam's normal surface takes the stretch of the run the
/// ball crosses it along, and this one is cut away there (#422).
pub fn roll_across(b: &Brep,at: V,r: f64,tol: f64,seams: bool) -> Result<Rolled,String> {
    let Meeting {uses,faces:[face_a,face_b],sides_faces,chain,stops,p,into:[da,db],theta,concave,sides,surfaces,
        names,..} = meeting(b,at,tol)?;
    let names = || names.clone();
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
        let m = [0,1].map(|k| scale(toward(k,p),-1.));
        // a tangent chain's seam: the ball over the vertex, where its contact with the side going on
        // must stand on the seam too, or no one section joins this run to the next
        let section = |k: usize,f: usize,leaving: V| -> Result<Cut,String> {
            let c = onto.project(across(p,m,r));
            let e = [toward(0,c),toward(1,c)];
            let n = unit(cross(e[0],e[1]));
            let keep = if dot(n,leaving) < 0. { scale(n,-1.) } else { n };
            seam_aligned(b,&uses,&sides_faces[k],f,v,add(c,scale(e[k],r)),tol,&names())?;
            Ok(Cut::Section {o:c,keep,reach:4.*r})
        };
        let (trim,faces) = end_at(b,&uses,&sides_faces,&chain,v,m,tol,&names(),&section)?;
        // the ball's centre over the vertex, `r` from each face
        let t = spine.inverse(onto.project(across(p,m,r)));
        ends.push(Stop {trim,t,faces});
    }
    // each strip's surface, its seam turned away from the loop
    let loop_pts: Vec<V> = chain.iter().flat_map(|&e| { let ed = &b.edges[e];
        (0..16).map(move |k| ed.point(ed.t[0]+(ed.t[1]-ed.t[0])*k as f64/16.,&b.vertices)) }).collect();
    let strips = [0,1].map(|k| reframed(&surfaces[k],&loop_pts));
    // whether a closed loop winds round a strip (a branch pipe's crotch round the branch; round
    // both where a bore down a cone's tip holds its axis): the second is split where it is
    // seamed, below
    let winding = [0,1].map(|k| ends.is_empty() && winds(&strips[k],&loop_pts));
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
        // from one end's centre to the other's through the seed (the ends put in the order the
        // stretch runs), then on past each end's plane until the whole section there (the centre,
        // the arc, the corner, within its reach of the centre) lies beyond it
        let at_seed = spine.inverse(seed);
        let (mut lo,mut hi,flip) = match spine.period() {
            Some(p) => {
                let back = |t: f64| (at_seed-t).rem_euclid(p);
                let fwd = |t: f64| (t-at_seed).rem_euclid(p);
                if back(ends[0].t) <= back(ends[1].t) { (at_seed-back(ends[0].t),at_seed+fwd(ends[1].t),false) }
                else { (at_seed-back(ends[1].t),at_seed+fwd(ends[0].t),true) }
            }
            None => (ends[0].t.min(ends[1].t),ends[0].t.max(ends[1].t),ends[0].t > ends[1].t),
        };
        if flip { ends.swap(0,1); }
        if !(lo <= at_seed && at_seed <= hi) {
            return Err(format!("the run of {} does not lie between its ends",names()));
        }
        let (floor,ceiling) = match &spine {
            Curve::Traced(c) if !c.closed => (0.,c.pts.len() as f64-1.),
            _ => (f64::NEG_INFINITY,f64::INFINITY),
        };
        let length = spine.period().unwrap_or(ceiling-floor);
        let step = if length.is_finite() { length/2048. } else { r/64. };
        // the section: the ball's centre, its arc, and the corner across it
        let past = |t: f64,e: &Trim| {
            let c = spine.point(t);
            let ends = [toward(0,c),toward(1,c)];
            let corner = across(c,ends,r);
            (0..=8).map(|k| add(c,scale(slerp(ends[0],ends[1],k as f64/8.),r))).chain([c,corner]).all(|x| e.beyond(x,0.1*r))
        };
        let room = |lo: f64,hi: f64| lo >= floor && hi <= ceiling && spine.period().is_none_or(|p| hi-lo < p);
        while !past(lo,&ends[0].trim) { lo -= step; if !room(lo,hi) { break } }
        while !past(hi,&ends[1].trim) { hi += step; if !room(lo,hi) { break } }
        if !room(lo,hi) {
            return Err(format!("the ball of {} cannot roll on past where `{}` and `{}` cut it off",
                names(),b.faces[ends[0].faces[0]].name,b.faces[ends[1].faces[0]].name));
        }
        (lo,hi-lo)
    };
    // the spine read along a smooth guide: a trace's own parameter turns at every point it was
    // marched through, which no smooth fit follows; a cubic through its points, by chord length,
    // projected back onto both offsets point by point, is exact and smooth between its ends
    let guide = smooth_guide(&spine,t0,span).ok_or("the spine could not be read smoothly")?;
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
    holds_ball(b,&uses,&sides_faces,&chain,&ends,&frame_at,&surfaces,sides,r,tol,&names(),seams)?;
    // fitted to the bar asked, or as near it as the densest net comes, no worse than a ten-millionth
    // of the boundary: its contacts are read at what was measured (`Edge::tol`)
    let bar = tol.max(1e-7*b.size());
    let (net,net_err) = fit_net(&canal,3,3,64,4,tol).ok_or("the canal could not be fitted")?;
    if net_err > bar { return Err(format!("the canal between {} misses its fit by {net_err:e}",names())); }
    let (spine_fit,spine_err) = fit_curve(&c_at,3,64,tol).ok_or("the spine could not be fitted")?;
    if spine_err > bar { return Err(format!("the spine between {} misses its fit by {spine_err:e}",names())); }
    // (each end cuts the whole piece off: the set keeps a point on its side of every face at either
    // end, `crease.ends`, wherever along the run it stands)
    let trims: Vec<Trim> = ends.iter().map(|e| e.trim.clone()).collect();
    let reversed = [face_a.reversed,face_b.reversed];
    // a loop winding round both strips: the second's contact split where it crosses that strip's
    // iso line through the loop's first vertex (the first's starts there already)
    let split = if winding[0] && winding[1] {
        let s = &strips[1];
        let vertex = b.vertices[b.edges[chain[0]].v[0] as usize].p;
        let contact = |u: f64| canal(u,1.);
        Some(crossing(&|u| s.inverse(contact(u))[0],s.inverse(vertex)[0],s.periods()[0].unwrap_or(TAU),0.,1.)
            .ok_or_else(|| format!("the fillet of {}: its second contact never crosses its strip's seam",names()))?)
            // (none where it crosses there at the canal's start already)
            .filter(|&u| u > 1e-9 && u < 1.-1e-9)
    } else { None };
    let piece = if ends.is_empty() { build(b,&chain,&net,&strips,reversed,concave,tol,net_err,split)? } else {
        let meeting = super::geom::Traced {a:surfaces[0].clone(),b:surfaces[1].clone(),pts:Vec::new(),closed:false};
        // the section's plane at `u`: through the ball's centre, square to the spine (the plane of
        // its arc), its normal along `u`; where the faces meet in it, across the ball from its centre
        let section = |u: f64| -> (V,V) {
            let (c,e) = frame_at(u);
            let n = unit(cross(e[0],e[1]));
            (c,if dot(n,guide.d2(u.clamp(0.,1.)).1) < 0. { scale(n,-1.) } else { n })
        };
        let corner = |u: f64| { let (c,e) = frame_at(u); crossing_plane(&meeting,c,section(u).1,across(c,e,r)) };
        let run = build_open(b,&chain,&net,&strips,&meeting,&section,&corner,[ends[0].trim.vertex,ends[1].trim.vertex],
            reversed,concave,tol,net_err,None)?;
        trimmed(run,&trims)?
    };
    let contacts = [net.column(0),net.column(net.poles[0].len()-1)];
    Ok(Rolled {piece,chain,spine:spine_fit,contacts,concave,r,reach,trims})
}

/// **A ball no larger than each face holds**: the band of a face it rolls on — where it stands
/// nearer the other face's surface than its contact does — crossed by none of the face's other
/// edges. Read against the other surface offset to the band's height (`curve_surface`); `frame_at`
/// the ball's centre and the directions to its contacts along the canal.
#[allow(clippy::too_many_arguments)]
fn holds_ball(b: &Brep,uses: &[Vec<(usize,usize,usize)>],sides_faces: &[Vec<usize>;2],chain: &[usize],ends: &[Stop],
    frame_at: &dyn Fn(f64) -> (V,[V;2]),surfaces: &[Surface;2],sides: [f64;2],r: f64,tol: f64,names: &str,seams_crossed: bool)
    -> Result<(),String> {
    let names = || names.to_string();
    let ending: std::collections::BTreeSet<usize> = ends.iter().flat_map(|s| s.faces.iter().copied()).collect();
    for k in 0..2 {
        let o = 1-k;
        let contact = |u: f64| { let (c,e) = frame_at(u); add(c,scale(e[k],r)) };
        let height = (0..=256).map(|i| sides[o]*surfaces[o].implicit(contact(i as f64/256.))).fold(0_f64,f64::max)*1.01+tol;
        let level = surfaces[o].offset(sides[o]*height).ok_or_else(|| format!("the band of {} cannot be read",names()))?;
        // an end's face cuts the fillet off: the ball crosses its edges with these two; and an
        // edge between two faces of one side is no edge to it
        // (and, where the ball is let cross them, the seams where a face goes on tangent: their
        // outward normals one at the middle)
        let tangent = |e: usize,x: usize,y: usize| {
            let ed = &b.edges[e];
            let p = ed.point(0.5*(ed.t[0]+ed.t[1]),&b.vertices);
            let out = |f: usize| { let g = unit(b.faces[f].surface.gradient(p)); if b.faces[f].reversed { scale(g,-1.) } else { g } };
            norm(sub(out(x),out(y))) < 1e-6
        };
        let skip = |e: usize| match uses[e].as_slice() {
            [x,y] => ending.contains(&x.0) || ending.contains(&y.0) || sides_faces[k].contains(&x.0) && sides_faces[k].contains(&y.0)
                || seams_crossed && tangent(e,x.0,y.0),
            _ => false,
        };
        for &fi in &sides_faces[k] {
            let face = &b.faces[fi];
            let seams = face.seams();
            for c in face.loops.iter().flatten() {
                if chain.contains(&(c.edge as usize)) || seams.contains(&c.edge) || skip(c.edge as usize) { continue }
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
    Ok(())
}

/// **A chamfer's flat cut along a traced run** (#218's 6): where two analytic faces meet on a loop
/// no line or circle carries, the flat a chamfer leaves — `std.Flat`'s, the points whose distances
/// from the two surfaces, each over the other's leg, sum to one — between the leg on each face:
/// on face `k`, where it stands `legs[k]` from the other face's surface (its meeting with that
/// surface offset by the leg). Sectioned square to the faces' meeting, each section's flat is the
/// curve between the two legs where that sum is one, pulled onto it by Newton's steps across the
/// chord; the flat face is those fitted as a net within `tol`, and the piece is bounded by it and
/// the two strips of the faces between the loop and the legs (`build`). `on_a` is a point beside
/// the face whose leg is `legs[0]`, nearer its surface than the other's (none where the legs are
/// alike). A run that ends is cut on past each end and cut off by the plane of the faces there, as
/// a rolled one is.
pub fn flat_along(b: &Brep,at: V,legs: [f64;2],on_a: Option<V>,tol: f64) -> Result<Brep,String> {
    let Meeting {uses,faces:[face_a,face_b],sides_faces,chain,stops,p,sides,surfaces,concave,names,..} = meeting(b,at,tol)?;
    // which face is `a`
    let swap = on_a.is_some_and(|q| surfaces[1].implicit(q).abs() < surfaces[0].implicit(q).abs());
    let legs = if swap { [legs[1],legs[0]] } else { legs };
    // the faces' meeting through the edge, closed
    let meet = super::geom::Traced {a:surfaces[0].clone(),b:surfaces[1].clone(),pts:Vec::new(),closed:false};
    let seed = meet.project(p);
    let edge = match super::ssi::intersect(&surfaces[0],&surfaces[1],tol) {
        super::ssi::Ssi::Curves(cs) => cs,
        super::ssi::Ssi::Same => return Err(format!("{names} lie on one surface")),
        super::ssi::Ssi::Traced => {
            let (lo,hi) = b.bounds();
            let grow = 4.*legs[0].max(legs[1])+norm(sub(hi,lo));
            super::ssi::trace(&surfaces[0],&surfaces[1],&[seed],lo.map(|x| x-grow),hi.map(|x| x+grow),tol)?
        }
    }.into_iter().min_by(|x,y| {
        let off = |c: &Curve| distance(c.point(c.inverse(seed)),seed);
        off(x).total_cmp(&off(y))
    }).ok_or_else(|| format!("{names} meet nowhere near their edge"))?;
    // each leg: face `k`'s surface met with the other's offset by `legs[k]` toward the region
    let leg = |k: usize| -> Result<super::geom::Traced,String> {
        let off = surfaces[1-k].offset(sides[1-k]*legs[k]).ok_or_else(|| format!("the flat of {names} is wider than a face holds"))?;
        Ok(super::geom::Traced {a:surfaces[k].clone(),b:off,pts:Vec::new(),closed:false})
    };
    let leg_meetings = [leg(0)?,leg(1)?];
    // the section through a point `e` of the meeting square to it (`t` its way): each leg's point
    // in it, from the edge along the face toward where the other's distance grows
    let legs_at = |e: V,t: V| -> [V;2] {
        std::array::from_fn(|k| {
            let n = unit(surfaces[k].gradient(e));
            let g = scale(unit(surfaces[1-k].gradient(e)),sides[1-k]);
            let w = unit(sub(g,scale(n,dot(g,n))));
            let guess = add(e,scale(w,legs[k]/dot(w,g).max(0.1)));
            crossing_plane(&leg_meetings[k],e,t,guess)
        })
    };
    let way_at = |t: f64| -> (V,V) {
        let e = meet.project(edge.point(t));
        let d = cross(surfaces[0].gradient(e),surfaces[1].gradient(e));
        (e,if dot(d,edge.tangent(t)) < 0. { scale(unit(d),-1.) } else { unit(d) })
    };
    // each strip's surface, its seam turned away from the loop
    let loop_pts: Vec<V> = chain.iter().flat_map(|&e| { let ed = &b.edges[e];
        (0..16).map(move |k| ed.point(ed.t[0]+(ed.t[1]-ed.t[0])*k as f64/16.,&b.vertices)) }).collect();
    let strips = [0,1].map(|k| reframed(&surfaces[k],&loop_pts));
    // **a run that ends**: cut off at each end by the plane of the faces there, the flat carried on
    // past it until its whole section has crossed that plane
    let mut ends: Vec<Stop> = Vec::new();
    for &v in &stops {
        let q = b.vertices[v as usize].p;
        let m = [0,1].map(|k| scale(unit(surfaces[k].gradient(q)),sides[k]));
        let section = |k: usize,f: usize,leaving: V| -> Result<Cut,String> {
            let (e,t) = (meet.project(q),unit(cross(surfaces[0].gradient(q),surfaces[1].gradient(q))));
            let keep = if dot(t,leaving) < 0. { scale(t,-1.) } else { t };
            seam_aligned(b,&uses,&sides_faces[k],f,v,legs_at(e,keep)[k],tol,&names)?;
            Ok(Cut::Section {o:e,keep,reach:4.*legs[0].max(legs[1])})
        };
        let (trim,faces) = end_at(b,&uses,&sides_faces,&chain,v,m,tol,&names,&section)?;
        ends.push(Stop {trim,t:edge.inverse(q),faces});
    }
    let winding = [0,1].map(|k| ends.is_empty() && winds(&strips[k],&loop_pts));
    let (t0,span) = if ends.is_empty() {
        let Some(period) = edge.period() else { return Err(format!("the meeting of {names} does not close")) };
        // the flat starts where its leg on a winding strip crosses the strip's iso line through a
        // vertex of the loop, so the strip's seam runs between vertices both already have
        let mut t0 = edge.inverse(seed);
        let vertex = b.vertices[b.edges[chain[0]].v[0] as usize].p;
        if let Some(k) = (0..2).find(|&k| winding[k]) {
            let s = &strips[k];
            let target = s.inverse(vertex)[0];
            let contact = |t: f64| { let (e,d) = way_at(t); legs_at(e,d)[k] };
            t0 = crossing(&|t| s.inverse(contact(t))[0],target,s.periods()[0].unwrap_or(TAU),t0,period)
                .ok_or_else(|| format!("the flat of {names}: its leg never crosses the strip's seam"))?;
        }
        (t0,period)
    } else {
        // from one end to the other through the seed, then on past each end's plane until the
        // section there — the edge's point and both legs, the flat between them a little more —
        // lies wholly beyond it
        let at_seed = edge.inverse(seed);
        let (mut lo,mut hi,flip) = match edge.period() {
            Some(pd) => {
                let back = |t: f64| (at_seed-t).rem_euclid(pd);
                let fwd = |t: f64| (t-at_seed).rem_euclid(pd);
                if back(ends[0].t) <= back(ends[1].t) { (at_seed-back(ends[0].t),at_seed+fwd(ends[1].t),false) }
                else { (at_seed-back(ends[1].t),at_seed+fwd(ends[0].t),true) }
            }
            None => (ends[0].t.min(ends[1].t),ends[0].t.max(ends[1].t),ends[0].t > ends[1].t),
        };
        if flip { ends.swap(0,1); }
        let (floor,ceiling) = match &edge {
            Curve::Traced(c) if !c.closed => (0.,c.pts.len() as f64-1.),
            _ => (f64::NEG_INFINITY,f64::INFINITY),
        };
        let length = edge.period().unwrap_or(ceiling-floor);
        let step = if length.is_finite() { length/2048. } else { legs[0].max(legs[1])/64. };
        let wide = legs[0].max(legs[1]);
        let past = |t: f64,e: &Trim| {
            let (q,d) = way_at(t);
            std::iter::once(q).chain(legs_at(q,d)).all(|x| e.beyond(x,0.1*wide))
        };
        let room = |lo: f64,hi: f64| lo >= floor && hi <= ceiling && edge.period().is_none_or(|pd| hi-lo < pd);
        while !past(lo,&ends[0].trim) { lo -= step; if !room(lo,hi) { break } }
        while !past(hi,&ends[1].trim) { hi += step; if !room(lo,hi) { break } }
        if !room(lo,hi) {
            return Err(format!("the flat of {names} cannot be cut on past where `{}` and `{}` cut it off",
                b.faces[ends[0].faces[0]].name,b.faces[ends[1].faces[0]].name));
        }
        (lo,hi-lo)
    };
    // the meeting read along a smooth guide, as `roll` reads its spine
    let guide = smooth_guide(&edge,t0,span).ok_or("the meeting could not be read smoothly")?;
    let frames = std::cell::RefCell::new(std::collections::BTreeMap::<u64,(V,V,[V;2])>::new());
    let frame_at = |u: f64| -> (V,V,[V;2]) {
        let u = u.clamp(0.,1.);
        if let Some(&f) = frames.borrow().get(&u.to_bits()) { return f }
        let e = meet.project(guide.point(u));
        let d = cross(surfaces[0].gradient(e),surfaces[1].gradient(e));
        let t = if dot(d,guide.d2(u).1) < 0. { scale(unit(d),-1.) } else { unit(d) };
        let f = (e,t,legs_at(e,t));
        frames.borrow_mut().insert(u.to_bits(),f);
        f
    };
    // the flat's sum of distances, nothing on it, and its gradient
    let sum = |q: V| sides[0]*surfaces[0].implicit(q)/legs[1]+sides[1]*surfaces[1].implicit(q)/legs[0]-1.;
    let grad = |q: V| add(scale(surfaces[0].gradient(q),sides[0]/legs[1]),scale(surfaces[1].gradient(q),sides[1]/legs[0]));
    let flat = |u: f64,v: f64| -> V {
        let (_,t,[pa,pb]) = frame_at(u);
        if v <= 0. { return pa }
        if v >= 1. { return pb }
        let across = unit(cross(t,sub(pb,pa)));
        let mut q = crate::space::lerp(pa,pb,v);
        for _ in 0..32 {
            let (f,g) = (sum(q),dot(grad(q),across));
            if g.abs() < 1e-300 { break }
            let step = f/g;
            q = sub(q,scale(across,step));
            if step.abs() <= 1e-15*(1.+norm(q)) { break }
        }
        q
    };
    // the flat no wider than each face: its leg on each face stays off the face's other edges
    for k in 0..2 {
        for i in 0..=64 {
            let (_,_,ps) = frame_at(i as f64/64.);
            let q = ps[k];
            if (sum(q)).abs() > 1e-6*(1.+norm(q)) { return Err(format!("the flat of {names} leaves its face")) }
        }
    }
    let bar = tol.max(1e-7*b.size());
    let (net,net_err) = fit_net(&flat,3,3,64,4,tol).ok_or("the flat could not be fitted")?;
    if net_err > bar { return Err(format!("the flat between {names} misses its fit by {net_err:e}")); }
    let middle = frame_at(0.5).0;
    let reversed = [face_a.reversed,face_b.reversed];
    if ends.is_empty() {
        // winding round both strips: the second's leg split where it is seamed, as `roll`'s contact
        let split = if winding[0] && winding[1] {
            let s = &strips[1];
            let vertex = b.vertices[b.edges[chain[0]].v[0] as usize].p;
            Some(crossing(&|u| s.inverse(flat(u,1.))[0],s.inverse(vertex)[0],s.periods()[0].unwrap_or(TAU),0.,1.)
                .ok_or_else(|| format!("the flat of {names}: its second leg never crosses its strip's seam"))?)
                .filter(|&u| u > 1e-9 && u < 1.-1e-9)
        } else { None };
        return build_with(b,&chain,&net,&strips,reversed,concave,tol,net_err,Some(middle),split)
    }
    let trims: Vec<Trim> = ends.iter().map(|e| e.trim.clone()).collect();
    let section = |u: f64| { let (e,t,_) = frame_at(u); (e,t) };
    let corner = |u: f64| frame_at(u).0;
    let run = build_open(b,&chain,&net,&strips,&meet,&section,&corner,[ends[0].trim.vertex,ends[1].trim.vertex],reversed,concave,
        tol,net_err,Some(middle))?;
    trimmed(run,&trims)
}

/// Where a run of `b`'s edges `chain` between the faces `sides_faces` ends, at its vertex `v`: the
/// other faces there, each cutting the run off (`Trim`: a plane, or a cylinder's, a cone's or a
/// sphere's solid, kept on the side the run lies on) — or, where it goes on smoothly from face `k`
/// (`m[k]` the way from face `k` toward the region there), what `section(k, face, leaving)` cuts
/// it at — or the run is refused, naming why.
#[allow(clippy::too_many_arguments)]
fn end_at(b: &Brep,uses: &[Vec<(usize,usize,usize)>],sides_faces: &[Vec<usize>;2],chain: &[usize],v: u32,m: [V;2],tol: f64,
    names: &str,section: &dyn Fn(usize,usize,V) -> Result<Cut,String>) -> Result<(Trim,Vec<usize>),String> {
    let names = || names.to_string();
    let p = b.vertices[v as usize].p;
    let faces: Vec<usize> = (0..b.edges.len()).filter(|&e| b.edges[e].v.contains(&v))
        .flat_map(|e| uses[e].iter().map(|u| u.0)).filter(|f| !sides_faces.iter().any(|s| s.contains(f)))
        .collect::<std::collections::BTreeSet<_>>().into_iter().collect();
    if faces.is_empty() { return Err(format!("the run of {} ends where nothing meets it",names())) }
    // the run's way leaving the vertex, along its edge
    let e = &b.edges[*chain.iter().find(|&&e| b.edges[e].v.contains(&v)).unwrap()];
    let EdgeCurve::Curve(c) = &e.curve else { return Err(format!("{} meet at a point",names())) };
    let leaving = if e.v[0] == v { unit(c.tangent(e.t[0])) } else { scale(unit(c.tangent(e.t[1])),-1.) };
    // the two faces' own other pieces alone, meeting the run at an angle: a drafted side crossing
    // its parting plane (#245), where the run turns and the canal each side would meet the other's
    // in a mitre
    let smooth = |g: usize| { let n = unit(b.faces[g].surface.gradient(p)); (0..2).any(|k| 1.-dot(m[k],n).abs() <= 1e-9) };
    let side_named = |g: usize| sides_faces.iter().flatten().any(|&s| b.faces[s].name == b.faces[g].name);
    let turning: Vec<usize> = faces.iter().copied().filter(|&g| !smooth(g)).collect();
    if !turning.is_empty() && turning.iter().all(|&g| side_named(g)) {
        return Err(format!("the fillet of {} turns where each face's pieces meet (as a drafted side's do at its parting \
            plane): two canals meeting in a mitre are rung 3",names()));
    }
    // each surface the faces there lie on, once: a plane, or a curved face's solid
    let mut cuts: Vec<Cut> = Vec::new();
    for &f in &faces {
        let face = &b.faces[f];
        let n = unit(face.surface.gradient(p));
        // a face one side runs on into smoothly (a chain of traced and straight edges): the run
        // ends at the section the caller makes there, the next piece taking on from it
        if let Some(k) = (0..2).find(|&k| 1.-dot(m[k],n).abs() <= 1e-9) {
            let cut = section(k,f,leaving)?;
            if !cuts.iter().any(|c| c.same(&cut,tol)) { cuts.push(cut); }
            continue;
        }
        let s = dot(leaving,n);
        if s.abs() < MIN_TURN.dsin() {
            return Err(format!("the fillet of {} meets `{}` along it",names(),face.name));
        }
        let cut = match face.surface {
            Surface::Plane(fr) => Cut::Plane {o:p,keep:if dot(leaving,fr.z) > 0. { fr.z } else { scale(fr.z,-1.) }},
            Surface::Cylinder(..) | Surface::Cone(..) | Surface::Sphere(..) => Cut::Solid {surface:face.surface.clone(),inside:s < 0.},
            _ => return Err(format!("the fillet of {} runs out onto `{}` ({}), which cuts nothing off yet",names(),face.name,
                face.surface.kind())),
        };
        if !cuts.iter().any(|k| k.same(&cut,tol)) { cuts.push(cut); }
    }
    Ok((Trim {vertex:v,o:p,cuts},faces))
}

/// Whether `contact` (where a ball or a flat meets the side `side` of a run, in the section through
/// its vertex `v`) stands on the seam at `v` between that side and the face `going` on from it
/// tangent: else the contact crosses the seam in another section than the edge does, and no one
/// section joins the run to the piece going on from it (#218's 7).
#[allow(clippy::too_many_arguments)]
fn seam_aligned(b: &Brep,uses: &[Vec<(usize,usize,usize)>],side: &[usize],going: usize,v: u32,contact: V,tol: f64,names: &str)
    -> Result<(),String> {
    let seams: Vec<usize> = (0..b.edges.len()).filter(|&e| b.edges[e].v.contains(&v)
        && uses[e].iter().any(|u| u.0 == going) && uses[e].iter().any(|u| side.contains(&u.0))).collect();
    let off = seams.iter().filter_map(|&e| match &b.edges[e].curve {
        EdgeCurve::Curve(c) => Some(distance(c.point(c.inverse(contact)),contact)),
        EdgeCurve::Degenerate => None,
    }).fold(f64::INFINITY,f64::min);
    if off <= 1e-6*(1.+norm(contact)).max(1e3*tol) { return Ok(()) }
    Err(format!("the fillet of {names} runs on smoothly into `{}`, but its contact crosses their seam in another section than \
        its edge does ({off:.2e} off it there): no one section joins the run to the piece going on",b.faces[going].name))
}

/// What cuts a run off where it ends (#218's 7): a plane through `o`, the run kept on the side
/// `keep` points to; or the solid a curved face's surface bounds (a cylinder, a cone or a sphere),
/// the run kept within it (`inside`) or outside it.
#[derive(Clone,Debug)]
pub enum Cut {
    Plane { o: V,keep: V },
    Solid { surface: Surface,inside: bool },
    /// Where a face goes on tangent from one the run lies on (a tangent chain): the section of the
    /// ball over the vertex, through `o`, the run kept on the side `keep` points to within `reach`
    /// of it — the next piece takes on from there (#218's 7).
    Section { o: V,keep: V,reach: f64 },
}

impl Cut {
    /// How far within the side kept `x` stands (negative beyond it): a plane's distance, a curved
    /// surface's signed distance.
    fn kept(&self,x: V) -> f64 {
        match self {
            Cut::Plane {o,keep} | Cut::Section {o,keep,..} => dot(sub(x,*o),*keep),
            Cut::Solid {surface,inside} => if *inside { -surface.implicit(x) } else { surface.implicit(x) },
        }
    }

    /// Whether `other` cuts the same way.
    fn same(&self,other: &Cut,tol: f64) -> bool {
        match (self,other) {
            (Cut::Plane {o,keep},Cut::Plane {o: p,keep: k}) => dot(*keep,*k) >= 1.-1e-9 && dot(sub(*p,*o),*keep).abs() <= tol,
            (Cut::Solid {surface,inside},Cut::Solid {surface: s,inside: i}) => inside == i && super::ssi::same(surface,s,tol),
            (Cut::Section {o,keep,..},Cut::Section {o: p,keep: k,..}) => dot(*keep,*k) >= 1.-1e-9 && distance(*o,*p) <= tol,
            _ => false,
        }
    }
}

/// Where a run that ends is cut off: the vertex it ends at (at `o`), and what cuts it off there —
/// every face at that end but the two it runs between, the run kept on its side of each.
#[derive(Clone,Debug)]
pub struct Trim { pub vertex: u32,pub o: V,pub cuts: Vec<Cut> }

impl Trim {
    /// Whether `x` stands beyond the run's end there by more than `margin`: beyond one of its cuts.
    fn beyond(&self,x: V,margin: f64) -> bool { self.cuts.iter().any(|c| c.kept(x) < -margin) }
}

/// An end of a run that ends: where it is cut off, the spine's parameter at the ball over its
/// vertex, and the faces there besides the run's two (all in one plane).
struct Stop { trim: Trim,t: f64,faces: Vec<usize> }

/// The point `r` from both planes through `p` square to the unit `a` and `b`, along their bisector:
/// a ball's centre from the corner it rounds (`a`, `b` from each face toward it), or the corner from
/// the centre (`a`, `b` toward the touches).
pub(crate) fn across(p: V,[a,b]: [V;2],r: f64) -> V { add(p,scale(add(a,b),r/(1.+dot(a,b)))) }

/// The point where `meeting` (two surfaces' meeting) crosses the plane through `o` square to `n`,
/// from `guess`: projected onto the meeting, then along its tangent to the plane, until it stays.
pub(super) fn crossing_plane(meeting: &super::geom::Traced,o: V,n: V,guess: V) -> V {
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

/// The piece of a run that ends, rolled (or cut) on past both ends (at vertices `stops` of `b`, in
/// the order the net's `u` runs): the fitted face over `net`, on each surface the strip between the
/// run (`chain`, edges of `b`, run on past each end along the faces' `meeting`) and the net's edge
/// on it, and at each end of the net a cap in the plane of its section. `section` is the plane the
/// net's row at `u` lies in (a point of it and its normal, along `u`), `corner` where the faces
/// meet in it; `edge` orients a flat's face (`surface_face`).
#[allow(clippy::too_many_arguments)]
fn build_open(b: &Brep,chain: &[usize],net: &Net,surfaces: &[Surface;2],meeting: &super::geom::Traced,
    section: &dyn Fn(f64) -> (V,V),corner: &dyn Fn(f64) -> V,stops: [u32;2],face_reversed: [bool;2],concave: bool,
    tol: f64,fit: f64,edge: Option<V>) -> Result<Brep,String> {
    let [[u0,u1],[v0,v1]] = net.domain();
    let mut out = Brep::default();
    let (last_u,last_v) = (net.poles.len()-1,net.poles[0].len()-1);
    let mut worst = fit;
    // the net's corners: contact `k` at each end
    let ct = [[net.point(u0,v0),net.point(u1,v0)],[net.point(u0,v1),net.point(u1,v1)]];
    let cv = ct.map(|p| p.map(|q| out.vertex(q)));
    let contact = [0,1].map(|k| out.edge(EdgeCurve::Curve(column(net,[0,last_v][k])),[u0,u1],cv[k]) as usize);
    let arc = [0,1].map(|j| out.edge(EdgeCurve::Curve(row(net,[0,last_u][j])),[v0,v1],[cv[0][j],cv[1][j]]) as usize);
    let (copied,mut run) = copy_run(b,chain,&mut out);
    // at each end: the section through the vertex (where the corner passes it), the run on along
    // the faces' meeting from the cap's corner to the vertex, and on each face the cap's curve from
    // the contact to the corner
    let mut caps: Vec<[usize;3]> = Vec::new();
    for (j,&stop) in stops.iter().enumerate() {
        let vertex = b.vertices[stop as usize].p;
        let n = 512;
        let ahead = |i: usize| { let (c,n) = section(i as f64/n as f64); dot(sub(vertex,c),n) };
        let passes = |&i: &usize| ahead(i) >= 0. && ahead(i+1) < 0.;
        let i = if j == 0 { (0..n).find(passes) } else { (0..n).rev().find(passes) }
            .ok_or("a fillet's run-on never passes the vertex it ends at")?;
        let at = crate::roots::bracketed_root(|u| { let (c,n) = section(u); Some(dot(sub(vertex,c),n)) },
            i as f64/n as f64,ahead(i),(i+1) as f64/n as f64,ahead(i+1),1e-15);
        let end = [u0,u1][j];
        let k_end = corner(end);
        let k_v = out.vertex(k_end);
        let (curve,err) = fitted(meeting,k_end,vertex,&|s| corner(end+(at-end)*s),tol)?;
        worst = worst.max(err);
        let ext = out.edge(EdgeCurve::Curve(curve),[0.,1.],[k_v,copied[&stop]]) as usize;
        out.edges[ext].tol = out.edges[ext].tol.max(err);
        run.push(ext);
        let (c,nrm) = section(end);
        let cap_plane = Surface::Plane(Frame::about(c,nrm));
        let mut cap = vec![arc[j]];
        for k in 0..2 {
            let on = super::geom::Traced {a:surfaces[k].clone(),b:cap_plane.clone(),pts:Vec::new(),closed:false};
            let p = ct[k][j];
            let (curve,err) = fitted(&on,p,k_end,&|s| crate::space::lerp(p,k_end,s),tol)?;
            worst = worst.max(err);
            let e = out.edge(EdgeCurve::Curve(curve),[0.,1.],[cv[k][j],k_v]) as usize;
            out.edges[e].tol = out.edges[e].tol.max(err);
            cap.push(e);
        }
        // the cap: its piece lies toward the run, its outward normal away along the spine
        face_out(&mut out,cap_plane,if j == 0 { scale(nrm,-1.) } else { nrm },&cap,["start","end"][j])?;
        caps.push([cap[0],cap[1],cap[2]]);
    }
    surface_face(&mut out,net,[contact[0] as u32,contact[1] as u32],[arc[0] as u32,arc[1] as u32],edge);
    // each strip: the run (on past both ends), a cap's curve, the contact, the other cap's curve
    for k in 0..2 {
        let reversed = face_reversed[k] != concave;
        let mut es = run.clone();
        es.extend([caps[0][1+k],contact[k],caps[1][1+k]]);
        let l = closed_loop(&out,&es,&surfaces[k])?;
        out.faces.push(Face {surface:surfaces[k].clone(),reversed,loops:vec![turned(l.coedges,(l.area > 0.) == reversed)],
            name:["a","b"][k].into()});
    }
    out.measure_tolerances();
    for c in contact { let e = &mut out.edges[c]; e.tol = e.tol.max(fit); }
    out.check(10.*tol.max(worst))?;
    Ok(out)
}

/// A run's piece cut off by what each end is cut off by, kept on the run's side: a plane's
/// half-space (`build::kept`), a curved face's solid (`build::kept_by_solid`).
fn trimmed(mut piece: Brep,trims: &[Trim]) -> Result<Brep,String> {
    let tol = tolerance(piece.size());
    let mut done: Vec<&Cut> = Vec::new();
    for cut in trims.iter().flat_map(|t| &t.cuts) {
        if done.iter().any(|c| c.same(cut,tol)) { continue }
        done.push(cut);
        piece = match cut {
            Cut::Plane {o,keep} => super::build::kept(&piece,*o,*keep,0.),
            Cut::Solid {surface,inside} => super::build::kept_by_solid(&piece,surface,*inside,0.),
            Cut::Section {o,keep,reach} => super::build::cut_before(&piece,*o,*keep,*reach,0.),
        }.map_err(|m| format!("a fillet's run cannot be cut off where it ends: {m}"))?;
    }
    Ok(piece)
}

/// The net's iso curve along `u` at its `j`th column of poles.
pub(super) fn column(net: &Net,j: usize) -> Curve { Curve::BSpline(Arc::new(net.column(j))) }

/// The net's iso curve along `v` at its `i`th row of poles.
fn row(net: &Net,i: usize) -> Curve {
    Curve::BSpline(Arc::new(BSpline {degree:net.dv,knots:net.vknots.clone(),poles:net.poles[i].clone(),
        weights:net.weights.as_ref().map(|w| w[i].clone())}))
}

/// The edges `chain` of `b` copied into `out` with their vertices: which of `out`'s each vertex of
/// `b` became, and the copies.
fn copy_run(b: &Brep,chain: &[usize],out: &mut Brep) -> (std::collections::BTreeMap<u32,u32>,Vec<usize>) {
    let mut copied = std::collections::BTreeMap::<u32,u32>::new();
    let mut run = Vec::new();
    for &e in chain {
        let ed = &b.edges[e];
        let vs = ed.v.map(|v| *copied.entry(v).or_insert_with(|| out.vertex(b.vertices[v as usize].p)));
        let id = out.edge(ed.curve.clone(),ed.t,vs);
        out.edges[id as usize].tol = ed.tol;
        run.push(id as usize);
    }
    (copied,run)
}

/// The fitted face over `net`, bounded by its two contacts (along `u`, at its first and last
/// columns) and the edges at its ends (along `v`: a seam used twice, or an arc at each end): a
/// round's canal (`edge` none: its piece lies outside the ball, so it looks toward the spine,
/// behind the middle of its arc), or a flat, its piece looking away from `edge`, the point of the
/// faces' meeting in its middle section.
fn surface_face(out: &mut Brep,net: &Net,contact: [u32;2],ends: [u32;2],edge: Option<V>) {
    let [[u0,u1],_] = net.domain();
    surface_face_of(out,net,[&[(contact[0],[u0,u1])],&[(contact[1],[u0,u1])]],ends,edge)
}

/// `surface_face` with each contact in pieces, each an edge over a stretch of `u`, in order.
fn surface_face_of(out: &mut Brep,net: &Net,contact: [&[(u32,[f64;2])];2],ends: [u32;2],edge: Option<V>) {
    let [[u0,u1],[v0,v1]] = net.domain();
    let line = |a: Uv,z: Uv| Pcurve::Line {a,b:z};
    let mut canal_loop: Vec<Coedge> = contact[0].iter()
        .map(|&(e,[a,z])| Coedge {edge:e,reversed:false,pcurve:line([a,v0],[z,v0])}).collect();
    canal_loop.push(Coedge {edge:ends[1],reversed:false,pcurve:line([u1,v0],[u1,v1])});
    canal_loop.extend(contact[1].iter().rev().map(|&(e,[a,z])| Coedge {edge:e,reversed:true,pcurve:line([a,v1],[z,v1])}));
    canal_loop.push(Coedge {edge:ends[0],reversed:true,pcurve:line([u0,v0],[u0,v1])});
    let surface = Surface::BSpline(Frame::new([0.;3],[0.,0.,1.],[1.,0.,0.]),Arc::new(net.clone()));
    let (m,su,sv) = surface.d1([0.5*(u0+u1),0.5*(v0+v1)]);
    // the ball's centre a radius behind the middle of the arc; a flat's edge behind it
    let (a,z) = (net.point(0.5*(u0+u1),v0),net.point(0.5*(u0+u1),v1));
    let away = match edge { Some(e) => unit(sub(m,e)),None => unit(sub(scale(add(a,z),0.5),m)) };
    let reversed = dot(cross(su,sv),away) < 0.;
    out.faces.push(Face {surface,reversed,loops:vec![turned(canal_loop,reversed)],name:if edge.is_some() { "flat" } else { "round" }.into()});
}

/// A face of `out` on `s` bounded by the one loop `es`, counter-clockwise in its parameters for
/// its outward side `outward`.
pub(super) fn face_out(out: &mut Brep,s: Surface,outward: V,es: &[usize],name: &str) -> Result<(),String> {
    let l = closed_loop(out,es,&s)?;
    let (_,su,sv) = s.d1(s.inverse(out.vertices[out.edges[es[0]].v[0] as usize].p));
    let reversed = dot(cross(su,sv),outward) < 0.;
    out.faces.push(Face {surface:s,reversed,loops:vec![turned(l.coedges,(l.area > 0.) == reversed)],name:name.into()});
    Ok(())
}

/// The piece: the canal face over `net` (seamed along its first column), and on each surface the
/// strip between the loop `chain` (edges of `b`) and the canal's contact with it.
fn build(b: &Brep,chain: &[usize],net: &Net,surfaces: &[Surface;2],face_reversed: [bool;2],concave: bool,tol: f64,fit: f64,
    split: Option<f64>) -> Result<Brep,String> { build_with(b,chain,net,surfaces,face_reversed,concave,tol,fit,None,split) }

/// `build` over a fitted face `surface_face` orients by `edge`, the contact on the second face
/// split at `split` (its `u`) where a strip winding round its surface too has its seam (#218's 7:
/// the loop winding round both).
#[allow(clippy::too_many_arguments)]
fn build_with(b: &Brep,chain: &[usize],net: &Net,surfaces: &[Surface;2],face_reversed: [bool;2],concave: bool,tol: f64,fit: f64,
    edge: Option<V>,split: Option<f64>) -> Result<Brep,String> {
    let [[u0,u1],[v0,v1]] = net.domain();
    let mut out = Brep::default();
    let last_v = net.poles[0].len()-1;
    let (va,vb) = (out.vertex(net.point(u0,v0)),out.vertex(net.point(u0,v1)));
    // the contacts, each one closed edge (the second in two where it is split), and the canal's
    // seam between them
    let first = out.edge(EdgeCurve::Curve(column(net,0)),[u0,u1],[va,va]);
    let second: Vec<(u32,[f64;2])> = match split {
        None => vec![(out.edge(EdgeCurve::Curve(column(net,last_v)),[u0,u1],[vb,vb]),[u0,u1])],
        Some(us) => {
            let vs = out.vertex(net.point(us,v1));
            vec![(out.edge(EdgeCurve::Curve(column(net,last_v)),[u0,us],[vb,vs]),[u0,us]),
                (out.edge(EdgeCurve::Curve(column(net,last_v)),[us,u1],[vs,vb]),[us,u1])]
        }
    };
    let contact: [Vec<u32>;2] = [vec![first],second.iter().map(|&(e,_)| e).collect()];
    let seam = out.edge(EdgeCurve::Curve(row(net,0)),[v0,v1],[va,vb]);
    surface_face_of(&mut out,net,[&[(first,[u0,u1])],&second],[seam,seam],edge);
    let (_,edge_of) = copy_run(b,chain,&mut out);
    for k in 0..2 {
        let name = ["a","b"][k];
        // the strip's surface: the face's own, its material away from the piece at a concave edge
        let reversed = face_reversed[k] != concave;
        let s = &surfaces[k];
        // (a split contact walked from where it is split, where its seam reaches it)
        let walk: Vec<usize> = contact[k].iter().rev().map(|&e| e as usize).collect();
        let mut loops = vec![closed_loop(&out,&walk,s)?,closed_loop(&out,&edge_of,s)?];
        let periods = s.periods();
        let winds = |l: &Loop| periods[0].is_some_and(|p| (l.turn[0]/p).round().abs() >= 1.);
        let face_loops = if loops.iter().all(|l| !winds(l)) {
            // a hole and its rim: the one enclosing the other runs round it the face's way
            loops.sort_by(|x,y| y.area.abs().total_cmp(&x.area.abs()));
            loops.into_iter().enumerate().map(|(i,l)| {
                let outer = i == 0;
                let ccw = l.area > 0.;
                let want = outer != reversed;
                if ccw == want { l.coedges } else { reverse_loop(l.coedges) }
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
    out.measure_tolerances();
    // a contact is the canal's iso line, off its face's surface by the fit at most — which a
    // measure at the net's own nodes (where the fit is exact) does not see
    for c in contact.iter().flatten() { let e = &mut out.edges[*c as usize]; e.tol = e.tol.max(fit); }
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

/// A face's loop the right way round for its side: counter-clockwise in its parameters as built,
/// reversed for a reversed face.
fn turned(l: Vec<Coedge>,reversed: bool) -> Vec<Coedge> { if reversed { reverse_loop(l) } else { l } }

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
    let contact = if (contact.turn[0]/period).round() == w { reverse_loop(contact.coedges) } else { contact.coedges };
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
    Ok(if ccw == !reversed { l } else { reverse_loop(l) })
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
/// and the corner's piece is the corner's cell less the ball (`corner_among` over the three
/// planes). `toward[i]` is the unit direction from face `i` toward the ball (into the material at a
/// convex corner, the void at a concave one), so `(centre − vertex) · toward[i] = r`.
pub fn corner(vertex: V,centre: V,r: f64,toward: [V;3],tol: f64) -> Result<Brep,String> {
    let planes = toward.map(|m| Surface::Plane(Frame::about(vertex,m)));
    corner_among(vertex,centre,r,[&planes[0],&planes[1],&planes[2]],tol)
}

/// **A ball's patch where three faces meet** (#218's 4): the corner's cell — the trihedron at
/// `vertex` cut by the three sections through the ball's centre square to its edges — less the
/// ball, over faces lying on `faces`: planes, or surfaces a plane through the centre square to an
/// edge cuts in a closed form (a cylinder about a line through the edge or along it: a line or a
/// circle; or, where no closed form holds them, traced about the corner). The ball touches each face
/// where the face's surface is nearest its centre; each edge is where two of the surfaces meet
/// through the vertex, its section the plane of the ball's arc between those two (through its
/// centre and both touches: square to the edge where the centre runs along it), at the edge's
/// foot where it crosses it (the section a neighbouring run, ring or rolled run ends in). Faces:
/// `a`, `b`, `c`, a stretch of each face (vertex, foot, touch, foot); `near_ab`, `near_bc`,
/// `near_ca`, the sections (foot, touch, the ball's arc, touch), each a neighbouring piece's cap;
/// and `round`, the ball's triangle. Every curve is the meeting `ssi::intersect` gives in closed
/// form, or `ssi::trace` near the corner.
pub fn corner_among(vertex: V,centre: V,r: f64,faces: [&Surface;3],tol: f64) -> Result<Brep,String> {
    use super::ssi::{intersect,Ssi};
    let pairs = [(0,1),(1,2),(2,0)];
    // where the ball touches each face, and the way from it to the ball
    let touches: [V;3] = faces.map(|s| s.point(s.inverse(centre)));
    if touches.iter().any(|&t| (distance(t,centre)-r).abs() > 1e-6*(1.+r)) {
        return Err("a corner's ball touches its faces nowhere".into())
    }
    let toward: [V;3] = std::array::from_fn(|i| unit(sub(centre,touches[i])));
    // the curve through `p` of where two surfaces meet: in closed form, or traced about the corner
    // (a section of a cone, two cylinders' saddle) within a box holding its cell
    let reach = distance(vertex,centre)+2.*r;
    let (lo,hi) = (centre.map(|x| x-reach),centre.map(|x| x+reach));
    let meeting = |a: &Surface,b: &Surface,p: V| -> Result<Curve,String> {
        let cs = match intersect(a,b,tol) {
            Ssi::Curves(cs) => cs,
            Ssi::Traced => super::ssi::trace(a,b,&[p],lo,hi,tol)?,
            Ssi::Same => return Err(format!("a corner where a {} and a {} lie on one surface",a.kind(),b.kind())),
        };
        cs.into_iter().find(|c| distance(c.point(c.inverse(p)),p) <= 1e3*tol)
            .ok_or_else(|| format!("a corner where a {} and a {} meet away from it",a.kind(),b.kind()))
    };
    let edges: Vec<Curve> = pairs.iter().map(|&(i,j)| meeting(faces[i],faces[j],vertex)).collect::<Result<_,_>>()?;
    // each edge's section: the plane of the ball's arc between the two faces — through its centre
    // and where it touches both (square to the edge where the ball's centre runs along it: planes,
    // a cylinder along it, a coaxial ring; not along a cone's generator) — its normal away from
    // the vertex, and the edge's foot where it crosses that plane
    let dirs: [V;3] = std::array::from_fn(|k| {
        let (i,j) = pairs[k];
        let n = cross(sub(touches[i],centre),sub(touches[j],centre));
        let n = if norm(n) > 1e-12*(1.+r*r) { unit(n) } else { unit(edges[k].tangent(edges[k].inverse(centre))) };
        if dot(n,sub(centre,vertex)) < 0. { scale(n,-1.) } else { n }
    });
    let feet: [V;3] = std::array::from_fn(|k| {
        let c = &edges[k];
        let mut t = c.inverse(centre);
        for _ in 0..64 {
            let (p,d) = (c.point(t),c.tangent(t));
            let along = dot(d,dirs[k]);
            if along.abs() < 1e-300 { break }
            let step = dot(sub(p,centre),dirs[k])/along;
            t -= step;
            if step.abs() <= 1e-15*(1.+t.abs()) { break }
        }
        c.point(t)
    });
    let mut out = Brep::default();
    let pv = out.vertex(vertex);
    let fv = feet.map(|p| out.vertex(p));
    let tv = touches.map(|p| out.vertex(p));
    // a stretch of a curve from `a` to `z` (the short way round a closed one)
    let stretch = |out: &mut Brep,c: &Curve,a: V,z: V,va: u32,vz: u32| -> usize {
        let (t0,mut t1) = (c.inverse(a),c.inverse(z));
        if let Some(period) = c.period() { t1 = t0+around(t1-t0,-0.5*period,period); }
        if t0 <= t1 { out.edge(EdgeCurve::Curve(c.clone()),[t0,t1],[va,vz]) as usize }
        else { out.edge(EdgeCurve::Curve(c.clone()),[t1,t0],[vz,va]) as usize }
    };
    // along each edge from the vertex to its foot; across each face from a foot to its touch
    let runs: Vec<usize> = (0..3).map(|k| stretch(&mut out,&edges[k],vertex,feet[k],pv,fv[k])).collect();
    let sections: [Surface;3] = dirs.map(|d| Surface::Plane(Frame::about(centre,d)));
    let mut across: Vec<[usize;2]> = Vec::new();
    for (k,&(i,j)) in pairs.iter().enumerate() {
        let mut both = [0;2];
        for (m,f) in [i,j].into_iter().enumerate() {
            let c = meeting(&sections[k],faces[f],feet[k])?;
            if distance(c.point(c.inverse(touches[f])),touches[f]) > 1e3*tol {
                return Err(format!("a corner's section meets its {} away from the ball's touch",faces[f].kind()))
            }
            both[m] = stretch(&mut out,&c,feet[k],touches[f],fv[k],tv[f]);
        }
        across.push(both);
    }
    // the ball's arc in each section, from one touch to the other about the centre
    let arcs: Vec<usize> = pairs.iter().enumerate().map(|(k,&(i,j))| {
        let x = unit(sub(touches[i],centre));
        let z = if dot(cross(x,sub(touches[j],centre)),dirs[k]) >= 0. { dirs[k] } else { scale(dirs[k],-1.) };
        let frame = Frame {o:centre,x,y:cross(z,x),z};
        let w = sub(touches[j],centre);
        let end = dot(w,frame.y).datan2(dot(w,frame.x));
        out.edge(EdgeCurve::Curve(Curve::Circle(frame,r)),[0.,end],[tv[i],tv[j]]) as usize
    }).collect();
    // on each face, the piece toward the ball from it: outward, away from the ball, where the loop
    // starts
    for (i,name) in ["a","b","c"].into_iter().enumerate() {
        let (k0,k1) = ((i+2)%3,i);
        let es = [runs[k0],across[k0][1],across[k1][0],runs[k1]];
        let at = out.vertices[out.edges[es[0]].v[0] as usize].p;
        let g = unit(faces[i].gradient(at));
        let away = if dot(unit(faces[i].gradient(touches[i])),toward[i]) > 0. { scale(g,-1.) } else { g };
        face_out(&mut out,faces[i].clone(),away,&es,name)?;
    }
    // each section: foot, touch, arc, touch; the piece lies toward the vertex from it
    for k in 0..3 {
        face_out(&mut out,sections[k].clone(),dirs[k],&[across[k][0],arcs[k],across[k][1]],["near_ab","near_bc","near_ca"][k])?;
    }
    // the ball's triangle, its frame's poles and seam turned away from it; the piece outside it
    let middle = unit(scale(add(add(toward[0],toward[1]),toward[2]),-1.));
    let sphere = Surface::Sphere(Frame::new(centre,Frame::about(centre,middle).x,scale(middle,-1.)),r);
    face_out(&mut out,sphere,scale(middle,-1.),&arcs,"round")?;
    out.measure_tolerances();
    out.check(10.*tol)?;
    Ok(out)
}

/// A corner's cell's points besides its vertex and centre: the centre's feet on its three edges
/// (in `corner_sections`' order) and where the ball touches each face.
pub fn corner_points(vertex: V,centre: V,r: f64,toward: [V;3]) -> ([V;3],[V;3]) {
    let along = sub(centre,vertex);
    (corner_sections(vertex,centre,toward).map(|e| add(vertex,scale(e,dot(along,e)))),toward.map(|m| sub(centre,scale(m,r))))
}

/// The directions of a corner's three edges from `vertex` toward the feet of `centre` on them —
/// each square to two faces' `toward`, in the order (0, 1), (1, 2), (2, 0) — the normals of its
/// sections through the centre.
fn corner_sections(vertex: V,centre: V,toward: [V;3]) -> [V;3] {
    let along = sub(centre,vertex);
    [(0,1),(1,2),(2,0)].map(|(i,j)| { let e = unit(cross(toward[i],toward[j])); if dot(e,along) < 0. { scale(e,-1.) } else { e } })
}
