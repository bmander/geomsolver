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
    /// The two faces' surfaces, and which way along each one's own normal the ball stands (±1):
    /// a contact is `c − side · r · ∇S(c)` for spine point `c`.
    pub surfaces: [Surface;2],
    pub sides: [f64;2],
    /// The bars the canal net and the spine were fitted within.
    pub fit: [f64;2],
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
/// meeting on a closed loop; `tol` the boundary's tolerance, `fit` the bar the canal is fitted
/// within. `Err` says why it cannot be rolled exactly.
pub fn roll(b: &Brep,at: V,r: f64,tol: f64,fit: f64) -> Result<Rolled,String> {
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
    // the loop: every edge between these two faces, closing on itself
    let chain: Vec<usize> = (0..b.edges.len()).filter(|&e| {
        let fs: Vec<usize> = uses[e].iter().map(|u| u.0).collect();
        fs.len() == 2 && fs.contains(&fa) && fs.contains(&fb)
    }).collect();
    let mut degree = std::collections::BTreeMap::<u32,usize>::new();
    for &e in &chain { for v in b.edges[e].v { *degree.entry(v).or_default() += 1; } }
    if degree.values().any(|&d| d % 2 != 0) {
        return Err(format!("the edge of {} runs on into other faces: a fillet along it ends, rung 3",names()));
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
    let offsets = [0,1].map(|k| surfaces[k].offset(sides[k]*r));
    let (Some(oa),Some(ob)) = (offsets[0].clone(),offsets[1].clone()) else {
        return Err(format!("the ball between {} is larger than one of them can hold",names()));
    };
    // the spine: the offsets' meeting through the ball's centre in the corner at `p`
    let guess = add(p,scale(unit(add(da,db)),r/(0.5*theta).dsin()));
    let seed = super::geom::Traced {a:oa.clone(),b:ob.clone(),pts:Vec::new(),closed:false}.project(guess);
    let spine = match super::ssi::intersect(&oa,&ob,tol) {
        super::ssi::Ssi::Curves(cs) => cs,
        super::ssi::Ssi::Same => return Err(format!("{} lie on one surface",names())),
        super::ssi::Ssi::Traced => {
            let (lo,hi) = b.bounds();
            let grow = 4.*r+norm(sub(hi,lo));
            super::ssi::trace(&oa,&ob,&[seed],lo.map(|x| x-grow),hi.map(|x| x+grow),tol)?
        }
    }.into_iter().min_by(|x,y| {
        let off = |c: &Curve| distance(c.point(c.inverse(seed)),seed);
        off(x).total_cmp(&off(y))
    }).ok_or_else(|| format!("the ball between {} rolls nowhere",names()))?;
    let Some(period) = spine.period() else {
        return Err(format!("the ball between {} rolls off the end of their meeting: a fillet that ends is rung 3",names()));
    };
    // where the ball touches each face, and the direction from its centre to there
    let toward = |k: usize,c: V| scale(unit(surfaces[k].gradient(c)),-sides[k]);
    // each strip's surface, its seam turned away from the loop, and whether the loop winds round
    // it (a branch pipe's crotch round the branch)
    let loop_pts: Vec<V> = chain.iter().flat_map(|&e| { let ed = &b.edges[e];
        (0..16).map(move |k| ed.point(ed.t[0]+(ed.t[1]-ed.t[0])*k as f64/16.,&b.vertices)) }).collect();
    let strips = [0,1].map(|k| reframed(&surfaces[k],&loop_pts));
    let winding = [0,1].map(|k| winds(&strips[k],&loop_pts));
    if winding[0] && winding[1] {
        return Err(format!("the fillet of {} winds round both faces: not built yet",names()));
    }
    // the canal starts where its contact with a winding strip crosses the strip's iso line through
    // a vertex of the loop, so the strip's seam runs between vertices both already have
    let mut t0 = spine.inverse(seed);
    let vertex = b.vertices[b.edges[chain[0]].v[0] as usize].p;
    if let Some(k) = (0..2).find(|&k| winding[k]) {
        let s = &strips[k];
        let target = s.inverse(vertex)[0];
        let contact = |t: f64| { let c = spine.point(t); add(c,scale(toward(k,c),r)) };
        t0 = crossing(&|t| s.inverse(contact(t))[0],target,s.periods()[0].unwrap_or(TAU),t0,period)
            .ok_or_else(|| format!("the fillet of {}: its contact never crosses the strip's seam",names()))?;
    }
    // the spine read along a smooth guide: a trace's own parameter turns at every point it was
    // marched through, which no smooth fit follows; a cubic through its points, by chord length,
    // projected back onto both offsets point by point, is exact and smooth between its ends
    let guide = {
        let n = 256;
        let pts: Vec<V> = (0..=n).map(|k| spine.point(t0+period*k as f64/n as f64)).collect();
        let mut ts = vec![0.];
        for w in pts.windows(2) { let last = *ts.last().unwrap(); ts.push(last+distance(w[0],w[1])); }
        let total = *ts.last().unwrap();
        let ts: Vec<f64> = ts.into_iter().map(|t| t/total).collect();
        super::nurbs::interpolate(&pts,&ts,3).ok_or("the spine could not be read smoothly")?
    };
    let onto = super::geom::Traced {a:oa.clone(),b:ob.clone(),pts:Vec::new(),closed:false};
    let c_at = |u: f64| onto.project(guide.point(u.clamp(0.,1.)));
    let canal = |u: f64,v: f64| { let c = c_at(u); add(c,scale(slerp(toward(0,c),toward(1,c),v),r)) };
    // the corner's angle all the way round: no fold, no ball wedged flat
    let mut reach: f64 = 0.;
    for k in 0..=256 {
        let c = c_at(k as f64/256.);
        let w = dot(toward(0,c),toward(1,c)).clamp(-1.,1.).dacos();
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
        let contact = |u: f64| { let c = c_at(u); add(c,scale(toward(k,c),r)) };
        let height = (0..=256).map(|i| sides[o]*surfaces[o].implicit(contact(i as f64/256.))).fold(0_f64,f64::max)*1.01+tol;
        let level = surfaces[o].offset(sides[o]*height).ok_or_else(|| format!("the band of {} cannot be read",names()))?;
        let fi = [fa,fb][k];
        let face = &b.faces[fi];
        let seams = face.seams();
        for c in face.loops.iter().flatten() {
            if chain.contains(&(c.edge as usize)) || seams.contains(&c.edge) { continue }
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
    // fitted to the bar asked, or as near it as the densest net comes, no worse than a ten-millionth
    // of the boundary: its contacts are read at what was measured (`Edge::tol`)
    let bar = fit.max(1e-7*b.size());
    let (net,net_err) = fit_net(&canal,3,3,64,4,fit).ok_or("the canal could not be fitted")?;
    if net_err > bar { return Err(format!("the canal between {} misses its fit by {net_err:e}",names())); }
    let (spine_fit,spine_err) = fit_curve(&c_at,3,64,fit).ok_or("the spine could not be fitted")?;
    if spine_err > bar { return Err(format!("the spine between {} misses its fit by {spine_err:e}",names())); }
    let piece = build(b,&chain,&net,&strips,&sides,concave,tol,net_err)?;
    let iso = |j: usize| BSpline {degree:net.du,knots:net.uknots.clone(),poles:net.poles.iter().map(|row| row[j]).collect()};
    let contacts = [iso(0),iso(net.poles[0].len()-1)];
    Ok(Rolled {piece,chain,spine:spine_fit,contacts,concave,r,reach,surfaces,sides,fit:[net_err,spine_err]})
}

/// The piece: the canal face over `net` (seamed along its first column), and on each surface the
/// strip between the loop `chain` (edges of `b`) and the canal's contact with it.
fn build(b: &Brep,chain: &[usize],net: &Net,surfaces: &[Surface;2],sides: &[f64;2],concave: bool,tol: f64,fit: f64) -> Result<Brep,String> {
    let [[u0,u1],[v0,v1]] = net.domain();
    let mut out = Brep::default();
    let column = |j: usize| Curve::BSpline(Arc::new(BSpline {degree:net.du,knots:net.uknots.clone(),poles:net.poles.iter().map(|row| row[j]).collect()}));
    let row = |i: usize| Curve::BSpline(Arc::new(BSpline {degree:net.dv,knots:net.vknots.clone(),poles:net.poles[i].clone()}));
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
        let out_sign = if concave { sides[k] } else { -sides[k] };
        let reversed = (out_sign > 0.) == concave;
        let s = &surfaces[k];
        let mut loops = vec![closed_loop(&out,&[contact[k] as usize],s,tol)?,closed_loop(&out,&edge_of.iter().map(|&e| e as usize).collect::<Vec<_>>(),s,tol)?];
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
fn closed_loop(b: &Brep,es: &[usize],s: &Surface,tol: f64) -> Result<Loop,String> {
    let _ = tol;
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
    let (u_end,u_high) = (low[0]+w*period,high[0]);
    let contact: Vec<Coedge> = {
        // its first use starts where the seam arrives, `u_end`
        let first = &contact[0];
        let e = &out.edges[first.edge as usize];
        let t = if first.reversed { e.t[1] } else { e.t[0] };
        let at = first.pcurve.at(t,e,s,&out.vertices)[0];
        let du = u_end-at;
        let _ = u_high;
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
            let (mut lo,mut hi,mut flo) = (at(k),at(k+1),fa);
            for _ in 0..60 {
                let m = 0.5*(lo+hi);
                let fm = level(u(m),last);
                if (fm < 0.) == (flo < 0.) { lo = m; flo = fm; } else { hi = m; }
            }
            return Some(0.5*(lo+hi))
        }
        last = next;
    }
    None
}
