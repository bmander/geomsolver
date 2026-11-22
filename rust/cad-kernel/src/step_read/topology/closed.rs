//! What a closed surface asks of a face's reading: a face bounded by vertices alone, the whole of
//! its surface (`whole`); an edge that is a point (`point_edge`, `points_dropped`,
//! `points_placed`); a file's curve in the parameters of a spline sheet that closes, written past
//! its domain (`onto_net`).
use super::{iso,moved,Bound,Built,Read,Setting,Walked,Written,LAID};
use super::super::geometry::{Flat,Geometry,Spatial};
use super::super::Refusal;
use crate::geom::{Curve,Frame,Rigid,Surface,Uv,V};
use crate::topo::{reverse_loop,Brep,Coedge,Edge,EdgeCurve,Face,Pcurve,Vertex};
use crate::util::space::{add,distance,norm,scale};
use std::sync::Arc;

/// **A file's curve in a spline sheet's parameters, held to the sheet's domain** (#993): as it is
/// where it lies in the domain; moved by the domain's width where it lies wholly past one end of a
/// parameter the sheet closes in (its points at the two ends one to rounding) — a writer's curve
/// on a sheet it holds periodic, springrep's bull nose at `v ∈ [−1.53, 0]` of `[0, 2π]`; none
/// where it leaves the domain otherwise. A sheet is clamped into its domain, so a point read past
/// it stands at its edge, and the curve's foot sought there settles there too: a curve read so was
/// taken for on its edge 2.6 mm off it.
pub(super) fn onto_net(pc: Pcurve,e: &Edge,s: &Surface,vs: &[Vertex]) -> Option<Pcurve> {
    let (Surface::BSpline(_,n) | Surface::Offset(_,n,_)) = s else { return Some(pc) };
    let dom = n.domain();
    let at: Vec<Uv> = (0..=16).map(|j| pc.at(e.t[0]+(e.t[1]-e.t[0])*j as f64/16.,e,s,vs)).collect();
    let mut d = [0.;2];
    for k in 0..2 {
        let [lo,hi] = dom[k];
        let (w,slack) = (hi-lo,1e-9*(hi-lo));
        let inside = |x: f64| x >= lo-slack && x <= hi+slack;
        if at.iter().all(|uv| inside(uv[k])) { continue }
        // (the sheet closes in `k`: its two ends one, at points across the other parameter)
        let [a,b] = dom[1-k];
        let size = 1.+norm(s.point([lo,a]));
        let closes = (0..=8).all(|j| {
            let y = a+(b-a)*j as f64/8.;
            let (p,q) = if k == 0 { ([lo,y],[hi,y]) } else { ([y,lo],[y,hi]) };
            distance(s.point(p),s.point(q)) <= 1e-9*size
        });
        if !closes { return None }
        let shift = if at.iter().all(|uv| uv[k] <= lo+slack) { w } else if at.iter().all(|uv| uv[k] >= hi-slack) { -w } else { return None };
        if !at.iter().all(|uv| inside(uv[k]+shift)) { return None }
        d[k] = shift;
    }
    Some(if d == [0.;2] { pc } else { moved(&pc,d) })
}

/// **An edge that is a point** (#993): its two vertices one entity, its curve a line of no length
/// (`Geometry::point_line`) standing at that vertex within `within` — brepkit's and vcad's habit
/// for a torus's seams and an edge of no length. Read as the kernel's degenerate edge there, a
/// point (`EdgeCurve::Degenerate`); none for any other edge, whose line of no length between two
/// vertices, or away from its vertex, is refused as malformed where its curve is read.
pub(super) fn point_edge(g: &Geometry,w: &Written,points: &[V],within: f64) -> Result<Option<Read>,Refusal> {
    if w.ends[0] != w.ends[1] { return Ok(None) }
    let Some(p) = g.point_line(w.geometry)? else { return Ok(None) };
    if distance(p,points[w.ends[0]]) > within { return Ok(None) }
    let v = w.ends[0] as u32;
    Ok(Some(Read {entity:w.entity,edge:Edge {curve:EdgeCurve::Degenerate,t:[0.,1.],v:[v,v],tol:0.},flip:false,rate:1.,pcurves:Vec::new()}))
}

/// **The uses of edges that are points, taken out of their loops** (`point_edge`): a loop's
/// corner at a point is no turn of it, and a loop of such uses alone is the vertex they stand at,
/// a vertex loop (which `whole` reads). Each face and the edges it used so, to be laid back as
/// points at the vertex (`points_placed`).
pub(super) fn points_dropped(faces: &mut [Walked],reads: &[Read]) -> Vec<(usize,usize)> {
    let mut out = Vec::new();
    for (fi,f) in faces.iter_mut().enumerate() {
        for bound in &mut f.bounds {
            let Bound::Edges(uses) = bound else { continue };
            let point = |e: usize| matches!(reads[e].edge.curve,EdgeCurve::Degenerate);
            if !uses.iter().any(|u| point(u.edge)) { continue }
            for u in uses.iter().filter(|u| point(u.edge)) { if !out.contains(&(fi,u.edge)) { out.push((fi,u.edge)); } }
            let v = reads[uses[0].edge].edge.v[0] as usize;
            uses.retain(|u| !point(u.edge));
            if uses.is_empty() { *bound = Bound::Vertex(f.entity,v); }
        }
    }
    out
}

/// **The edges that are points laid back** where their faces' loops reach their vertex: each a
/// degenerate use at the first corner there, from the one use's end to the next's start (a pole's
/// edge, which `Brep::close_poles` then runs along the pole, where the corner is one) — the file's
/// edge in the first face using it, a copy of it, the same entity, in each other (a degenerate edge
/// is used once). Refused naming the edge where a face's loops reach its vertex nowhere.
pub(super) fn points_placed(b: &mut Brep,edges: &mut Vec<u32>,dropped: &[(usize,usize)]) -> Result<(),Refusal> {
    let mut placed: Vec<usize> = Vec::new();
    for &(fi,e) in dropped {
        let entity = edges[e];
        let v = b.edges[e].v[0];
        let f = &b.faces[fi];
        let Some((li,k)) = f.loops.iter().enumerate().find_map(|(li,l)| (0..l.len()).find(|&k| b.ends(&l[k])[1] == v).map(|k| (li,k))) else {
            return Err(Refusal::unsupported(entity,"an edge that is a point, at a vertex its face's other bounds do not reach: not read"))
        };
        let l = &f.loops[li];
        let (a,z) = (b.uv_ends(f,&l[k])[1],b.uv_ends(f,&l[(k+1)%l.len()])[0]);
        let id = if placed.contains(&e) {
            b.edges.push(Edge {curve:EdgeCurve::Degenerate,t:[0.,1.],v:[v,v],tol:0.});
            if edges.len() < b.edges.len()-1 { edges.resize(b.edges.len()-1,0); }
            edges.push(entity);
            b.edges.len()-1
        } else { placed.push(e); e };
        b.faces[fi].loops[li].insert(k+1,Coedge {edge:id as u32,reversed:false,pcurve:Pcurve::Line {a,b:z}});
    }
    Ok(())
}

/// The curve of `s` along which its parameter `c` is `x`, its own parameter the other of the
/// surface's at each point: a torus's circles (`iso`), a revolution's curve turned by `x` or the
/// circle a point of it turns round, a spline sheet's first or last row or column where `x` is an
/// end of its domain. Held to the surface at five points; none on any other.
fn seam_curve(s: &Surface,c: usize,x: f64,[a,b]: [f64;2],within: f64) -> Option<Curve> {
    let curve = match (s,c) {
        (Surface::Torus(..),_) => iso(s,c,x)?,
        (Surface::Revolution(f,m),0) => m.moved(&Rigid::turn(f.o,f.z,x)),
        (Surface::Revolution(f,m),_) => {
            let l = f.local(m.point(x));
            if !(l[0] > within) { return None }
            Curve::Circle(Frame {o:add(f.o,scale(f.z,l[2])),..*f},l[0])
        }
        (Surface::BSpline(_,n),_) => {
            let [lo,hi] = n.domain()[c];
            let end = if x == lo { 0 } else if x == hi { 1 } else { return None };
            Curve::BSpline(Arc::new(match (c,end) {
                (0,0) => n.row(0),(0,_) => n.row(n.poles.len()-1),
                (_,0) => n.column(0),_ => n.column(n.poles[0].len()-1),
            }))
        }
        _ => return None,
    };
    let at = |y: f64| if c == 0 { [x,y] } else { [y,x] };
    (0..=4).all(|j| { let y = a+(b-a)*j as f64/4.; distance(curve.point(y),s.point(at(y))) <= within }).then_some(curve)
}

/// The span of parameter `k` of `s` a whole face of it runs over: a period's from `from`, a spline
/// sheet's domain, a revolution's curve's; none where it runs on without end.
fn span(s: &Surface,k: usize,from: f64) -> Option<[f64;2]> {
    if let Some(p) = s.periods()[k] { return Some([from,from+p]) }
    match (s,k) {
        (Surface::BSpline(_,n),_) => Some(n.domain()[k]),
        (Surface::Revolution(_,m),1) => match &**m { Curve::BSpline(b) => Some(b.domain()),_ => None },
        _ => None,
    }
}

/// The points of `s` along its parameter `k` held at `x`, the other run over `[a, b]`.
fn row(s: &Surface,k: usize,x: f64,[a,b]: [f64;2]) -> Vec<V> {
    (0..=16).map(|j| { let y = a+(b-a)*j as f64/16.; s.point(if k == 0 { [x,y] } else { [y,x] }) }).collect()
}

/// Whether the points are one, within `within`.
fn one(p: &[V],within: f64) -> bool { p.iter().all(|&q| distance(q,p[0]) <= within) }

/// **Whether `s` is pinched to a point inside `[lo, hi]` along `k`** (a row of it, `k` held,
/// collapsed): where a whole face of it would touch itself — a spindle torus's apple and lemon at
/// its poles, a circle crossing its axis turned, an ellipse about it turned twice over. The rows'
/// reach read at 128 values and each least of them sought down to its bottom.
fn pinched(s: &Surface,k: usize,[lo,hi]: [f64;2],other: [f64;2],within: f64) -> bool {
    let reach = |x: f64| { let p = row(s,k,x,other); p.iter().map(|&q| distance(q,p[0])).fold(0.,f64::max) };
    let n = 128;
    let xs: Vec<f64> = (0..=n).map(|j| lo+(hi-lo)*j as f64/n as f64).collect();
    let r: Vec<f64> = xs.iter().map(|&x| reach(x)).collect();
    (1..n).any(|j| {
        if !(r[j] <= r[j-1] && r[j] <= r[j+1]) { return false }
        // (golden sections of the bracket about the least)
        let (mut a,mut b) = (xs[j-1],xs[j+1]);
        for _ in 0..80 {
            let (m1,m2) = (b-0.618*(b-a),a+0.618*(b-a));
            if reach(m1) <= reach(m2) { b = m2; } else { a = m1; }
        }
        let m = 0.5*(a+b);
        m-lo > 1e-6*(hi-lo) && hi-m > 1e-6*(hi-lo) && reach(m) <= within
    })
}

/// **A face bounded by vertices alone, the whole of its surface** (#993; ISO 10303-42 lets a
/// face's bounds be `VERTEX_LOOP`s): of a surface closed round one parameter whose other's two
/// ends are each collapsed to a point (sphere-like: an ellipsoid's net, a revolution of an arc from
/// its axis to its axis, a horn torus), the seam along the closed parameter's start and the poles,
/// which `Brep::close_poles` joins; of one closed round both and pinched nowhere (torus-like), its
/// two seams. Closure is the surface's own — its points at a parameter's two ends one to rounding,
/// or its period — never a file's flags. Each vertex is a pole's (the pole laid where none
/// stands) or stands on a seam laid, which it cuts; a period's start is put at a vertex. Refused
/// naming the face where the surface does not close so, or touches itself inside (`pinched`), and
/// naming the bound where a vertex is on neither a pole nor a seam. None for a face with an edge,
/// a sphere's and a ring torus's (read as before).
pub(super) fn whole(st: &Setting,s: &Surface,w: &Walked) -> Option<Result<Built,Refusal>> {
    let lone: Vec<(u32,usize)> = w.bounds.iter().map(|b| match *b { Bound::Vertex(lp,v) => Some((lp,v)),Bound::Edges(_) => None }).collect::<Option<_>>()?;
    match *s { Surface::Sphere(..) => return None,Surface::Torus(_,big,small) if big > small.abs() => return None,_ => {} }
    Some(whole_face(st,s,w,&lone))
}

fn whole_face(st: &Setting,s: &Surface,w: &Walked,lone: &[(u32,usize)]) -> Result<Built,Refusal> {
    let b = st.b;
    let within = st.within;
    let refuse = |m: &str| Refusal::unsupported(w.entity,format!("a face bounded by vertices alone on a surface that {m}: not read"));
    let mut vs: Vec<(u32,usize,V)> = Vec::new();
    for &(lp,v) in lone { if !vs.iter().any(|x| x.1 == v) { vs.push((lp,v,b.vertices[v].p)); } }
    let at: Vec<Uv> = vs.iter().map(|x| s.inverse(x.2)).collect();
    // the spans from where the first vertex stands, a period's
    let spans = |from: Uv| -> Option<[[f64;2];2]> { Some([span(s,0,from[0])?,span(s,1,from[1])?]) };
    let Some(first) = spans(at[0]) else { return Err(refuse("runs on without end")) };
    let closes = |k: usize,d: [[f64;2];2]| s.periods()[k].is_some() || {
        let [lo,hi] = d[k];
        let (p,q) = (row(s,k,lo,d[1-k]),row(s,k,hi,d[1-k]));
        p.iter().zip(&q).all(|(a,c)| distance(*a,*c) <= within)
    };
    let pole = |k: usize,x: f64,d: [[f64;2];2]| one(&row(s,k,x,d[1-k]),within);
    // sphere-like: closed round `c`, its rows at the ends of `k` points; the start of a period
    // `k` at a vertex at a pole, of a period `c` at a vertex off the poles
    for c in 0..2 {
        let k = 1-c;
        let mut from = at[0];
        if s.periods()[k].is_some() { if let Some(x) = at.iter().find(|uv| pole(k,uv[k],first)) { from[k] = x[k]; } }
        let Some(d) = spans(from) else { continue };
        if !closes(c,d) || !pole(k,d[k][0],d) || !pole(k,d[k][1],d) { continue }
        let ends = [s.point(if k == 1 { [d[0][0],d[1][0]] } else { [d[0][0],d[1][0]] }),s.point(if k == 1 { [d[0][0],d[1][1]] } else { [d[0][1],d[1][0]] })];
        let off: Vec<usize> = (0..vs.len()).filter(|&i| ends.iter().all(|&e| distance(e,vs[i].2) > within)).collect();
        let mut d = d;
        if let (Some(p),Some(&i)) = (s.periods()[c],off.first()) { d[c] = [at[i][c],at[i][c]+p]; }
        if pinched(s,k,d[k],d[c],within) || pinched(s,c,d[c],d[k],within) { return Err(refuse("touches itself inside it, pinched to a point (no face of a manifold solid)")) }
        return sphere_like(st,s,w,&vs,c,d)
    }
    // torus-like: closed round both, pinched nowhere
    if closes(0,first) && closes(1,first) {
        if (0..2).any(|k| pole(k,first[k][0],first) || pinched(s,k,first[k],first[1-k],within)) {
            return Err(refuse("touches itself, pinched to a point (no face of a manifold solid)"))
        }
        return torus_like(st,s,w,&vs,first)
    }
    Err(refuse("does not close round it"))
}

/// A seam laid along `c` held at `x` over `d[k]`, cut at the vertices `cut` (each a parameter along
/// it and a vertex), from `start` to `end` (vertex ids): its pieces, the laid edges named from
/// `LAID - first` down.
fn pieces(curve: &Curve,[lo,hi]: [f64;2],mut cut: Vec<(f64,u32)>,start: u32,end: u32) -> Vec<Edge> {
    cut.sort_by(|a,b| a.0.total_cmp(&b.0));
    let mut knots = vec![(lo,start)];
    knots.extend(cut);
    knots.push((hi,end));
    knots.windows(2).map(|w| Edge {curve:EdgeCurve::Curve(curve.clone()),t:[w[0].0,w[1].0],v:[w[0].1,w[1].1],tol:0.}).collect()
}

/// Where a vertex stands on a seam: its parameter along it strictly inside `[lo, hi]`.
fn on_seam(curve: &Curve,p: V,[lo,hi]: [f64;2],within: f64) -> Option<f64> {
    let t = match curve { Curve::BSpline(b) => b.nearest(p),c => {
        let t = c.inverse(p);
        // (a closed conic's parameter into the span)
        match c.period() { Some(q) => lo+(t-lo).rem_euclid(q),None => t }
    } };
    (distance(curve.point(t),p) <= within && t > lo && t < hi).then_some(t)
}

fn sphere_like(st: &Setting,s: &Surface,w: &Walked,vs: &[(u32,usize,V)],c: usize,d: [[f64;2];2]) -> Result<Built,Refusal> {
    let k = 1-c;
    let within = st.within;
    let uv = |x: f64,y: f64| if c == 0 { [x,y] } else { [y,x] };
    let Some(seam) = seam_curve(s,c,d[c][0],d[k],within) else {
        return Err(Refusal::unsupported(w.entity,"a face bounded by vertices alone on a surface the reader lays no seam on: not read"))
    };
    let (p0,p1) = (s.point(uv(d[c][0],d[k][0])),s.point(uv(d[c][0],d[k][1])));
    let same = distance(p0,p1) <= within;
    let mut poles = Vec::new();
    let (mut v0,mut v1,mut cut) = (None,None,Vec::new());
    for &(lp,v,p) in vs {
        if distance(p,p0) <= within && v0.is_none() { v0 = Some(v as u32); }
        else if distance(p,p1) <= within && v1.is_none() && !same { v1 = Some(v as u32); }
        else if let Some(t) = on_seam(&seam,p,d[k],within) { cut.push((t,v as u32)); }
        else { return Err(Refusal::unsupported(lp,"a face bounded by a vertex neither at a pole of its surface nor on the seam laid through it: not read")) }
    }
    let mut lay = |p: V| { poles.push(p); LAID-(poles.len() as u32-1) };
    let v0 = v0.unwrap_or_else(|| lay(p0));
    let v1 = if same { v0 } else { v1.unwrap_or_else(|| lay(p1)) };
    let laid = pieces(&seam,d[k],cut,v0,v1);
    let line = |x: f64,e: &Edge| Pcurve::Line {a:uv(x,e.t[0]),b:uv(x,e.t[1])};
    let n = laid.len() as u32;
    // closed round `u`: up its far side, down its near; round `v`: along its near side, back its far
    let (along,back) = if c == 0 { (d[c][1],d[c][0]) } else { (d[c][0],d[c][1]) };
    let mut l: Vec<Coedge> = laid.iter().enumerate().map(|(j,e)| Coedge {edge:LAID-j as u32,reversed:false,pcurve:line(along,e)}).collect();
    l.extend(laid.iter().enumerate().rev().map(|(j,e)| Coedge {edge:LAID-j as u32,reversed:true,pcurve:line(back,e)}));
    debug_assert!(n > 0);
    if w.reversed { l = reverse_loop(l); }
    Ok(Built {face:Face {surface:s.clone(),reversed:w.reversed,loops:vec![l],name:w.name.clone()},laid,poles,cuts:Vec::new()})
}

fn torus_like(st: &Setting,s: &Surface,w: &Walked,vs: &[(u32,usize,V)],d: [[f64;2];2]) -> Result<Built,Refusal> {
    let within = st.within;
    let none = || Refusal::unsupported(w.entity,"a face bounded by vertices alone on a surface the reader lays no seam on: not read");
    // `a` along `v` at the start of `u`, `b` along `u` at the start of `v`
    let a = seam_curve(s,0,d[0][0],d[1],within).ok_or_else(none)?;
    let bs = seam_curve(s,1,d[1][0],d[0],within).ok_or_else(none)?;
    let corner = s.point([d[0][0],d[1][0]]);
    let mut poles = Vec::new();
    let (mut v0,mut ca,mut cb) = (None,Vec::new(),Vec::new());
    for &(lp,v,p) in vs {
        if distance(p,corner) <= within && v0.is_none() { v0 = Some(v as u32); }
        else if let Some(t) = on_seam(&a,p,d[1],within) { ca.push((t,v as u32)); }
        else if let Some(t) = on_seam(&bs,p,d[0],within) { cb.push((t,v as u32)); }
        else { return Err(Refusal::unsupported(lp,"a face bounded by a vertex on neither seam laid through its surface: not read")) }
    }
    let v0 = v0.unwrap_or_else(|| { poles.push(corner); LAID });
    let (la,lb) = (pieces(&a,d[1],ca,v0,v0),pieces(&bs,d[0],cb,v0,v0));
    let (na,_) = (la.len() as u32,lb.len());
    let id = |j: usize,second: bool| LAID-(j as u32+if second { na } else { 0 });
    let line = |p: Uv,q: Uv| Pcurve::Line {a:p,b:q};
    let mut l = Vec::new();
    for (j,e) in lb.iter().enumerate() { l.push(Coedge {edge:id(j,true),reversed:false,pcurve:line([e.t[0],d[1][0]],[e.t[1],d[1][0]])}); }
    for (j,e) in la.iter().enumerate() { l.push(Coedge {edge:id(j,false),reversed:false,pcurve:line([d[0][1],e.t[0]],[d[0][1],e.t[1]])}); }
    for (j,e) in lb.iter().enumerate().rev() { l.push(Coedge {edge:id(j,true),reversed:true,pcurve:line([e.t[0],d[1][1]],[e.t[1],d[1][1]])}); }
    for (j,e) in la.iter().enumerate().rev() { l.push(Coedge {edge:id(j,false),reversed:true,pcurve:line([d[0][0],e.t[0]],[d[0][0],e.t[1]])}); }
    if w.reversed { l = reverse_loop(l); }
    let mut laid = la;
    laid.extend(lb);
    Ok(Built {face:Face {surface:s.clone(),reversed:w.reversed,loops:vec![l],name:w.name.clone()},laid,poles,cuts:Vec::new()})
}

/// **A closed edge on a closed spline whose vertex stands off its ends** (#993): the spline
/// started again at the vertex (`geometry::restarted`, the same points), so the edge runs once
/// round it from its vertex to its vertex; its curves in faces' parameters, over the file's
/// parameter of it, no longer read (the edge inverted onto its faces instead). Any other edge as
/// it is.
pub(super) fn restarted(curve: Spatial,pcurves: Vec<(u32,Flat)>,w: &Written,points: &[V],within: f64) -> (Spatial,Vec<(u32,Flat)>) {
    let Curve::BSpline(b) = &curve.curve else { return (curve,pcurves) };
    let p = points[w.ends[0]];
    let [a,_] = b.domain();
    if w.ends[0] != w.ends[1] || distance(b.point(a),p) <= within { return (curve,pcurves) }
    let t = b.nearest(p);
    if distance(b.point(t),p) > within { return (curve,pcurves) }
    match super::super::geometry::restarted(b,t,within) {
        Some(c) => (Spatial {curve:Curve::BSpline(Arc::new(c)),rate:curve.rate},Vec::new()),
        None => (curve,pcurves),
    }
}
