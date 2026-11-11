//! **A face's contact under a constant twist, in pieces** (#880 Q1b S1, docs/swept-solid-design.md
//! §13): the zero set of the contact function `g = n · v` over a face's parameters, for any surface,
//! cut into regular pieces at the face's boundary and where the set branches.
//!
//! Under a slide, a turn or a screw every point of the tool moves the same way relative to the tool
//! at every instant, so the points in contact are fixed on the tool, and the swept boundary lies in
//! them carried over the roll (with the tool at the roll's ends). A whole tool's contact is a graph,
//! not one curve (§13): its branches meet at double points, end at poles, and leave a face across its
//! boundary. Each regular piece is a candidate of its own, which the trim keeps where it bounds the
//! sweep, so the pieces need not be ordered into one curve.
//!
//! The set is found on a grid over the face's parameter box (`Located::parameter_box`): each edge of
//! a cell where `g` changes sign gives a root, closed on by bisection; the roots are linked cell by
//! cell into polylines; a cell crossed four times holds a saddle of `g`, where two branches cross at a
//! double point (`∇g = 0` there, `g = 0`) or pass by each other, read by the saddle's own value. The
//! polylines are cut where they leave the face (`Located::holds_parameters`), each end placed on the
//! boundary by bisection; each point is settled onto `g = 0` by Newton across the curve; points are
//! added until every chord is within the tolerance of the curve; and pieces ending on either side of a
//! seam where the face wraps round its period are joined. Read numerically on every surface: the
//! closed forms (`contact::Form`) are what it is held to.
use crate::geom::{Surface,Uv,V};
use crate::query::Located;
use crate::topo::Brep;
use crate::trajectory::Twist;
use crate::util::space::{distance,scale,add};

/// How a piece of contact ends.
#[derive(Clone,Copy,Debug,PartialEq,Eq)]
pub enum End {
    /// On the face's boundary: the contact goes on, if at all, on a neighbouring face or an edge's fan.
    Boundary,
    /// Where the set branches, or the contact function is not defined: a double point (`∇g = 0`,
    /// shared by the pieces that end there), or beside a pole or an apex, where the normal is not.
    Singular,
}

/// A regular piece of a face's contact: its points in order, with their parameters, and how each
/// end of it ends; a closed piece is a loop, its last point its first.
#[derive(Clone,Debug)]
pub struct Piece { pub face: usize,pub uv: Vec<Uv>,pub points: Vec<V>,pub ends: [End;2],pub closed: bool }

/// A face's contact under a twist.
#[derive(Clone,Debug)]
pub enum Contact {
    /// Its regular pieces, none where it is in contact nowhere.
    Pieces(Vec<Piece>),
    /// Every point of it: the motion keeps the face in its own surface (`n · v ≡ 0`, a cylinder turned
    /// about its own axis, a plane slid along itself), so the face sweeps only itself.
    Whole,
}

/// The cells a face's parameter box is read in, each way, at least.
pub const CELLS: usize = 96;

/// **Face `fi`'s contact under `twist`** (the velocity field in the tool's own coordinates,
/// `Screw::twist`), in pieces within `tol` of the curve: read on a grid of `cells` each way at least.
pub fn face_contact(tool: &Brep,located: &Located,fi: usize,twist: &Twist,tol: f64,cells: usize) -> Contact {
    let surface = &tool.faces[fi].surface;
    let [lo,hi] = located.parameter_box(fi);
    let span = [hi[0]-lo[0],hi[1]-lo[1]];
    if !(span[0] > 0. && span[1] > 0.) { return Contact::Pieces(Vec::new()) }
    let g = |uv: Uv| surface.normal_velocity(uv,twist);
    // the grid: as many cells each way as its side's length in space asks, `cells` at least
    let reach = |k: usize| {
        let at = |x: f64| { let mut uv = [lo[0]+0.5*span[0],lo[1]+0.5*span[1]]; uv[k] = lo[k]+x*span[k]; surface.point(uv) };
        (0..16).map(|i| distance(at(i as f64/16.),at((i+1) as f64/16.))).sum::<f64>()
    };
    let (ru,rv) = (reach(0),reach(1));
    let n = [(ru,rv),(rv,ru)].map(|(a,b)| cells.max((cells as f64*a/b.max(1e-300)).min(4.*cells as f64) as usize));
    let node = |i: usize,j: usize| -> Uv { [lo[0]+span[0]*i as f64/n[0] as f64,lo[1]+span[1]*j as f64/n[1] as f64] };
    let values: Vec<Vec<Option<f64>>> = crate::util::par::indices(n[0]+1,|i| (0..=n[1]).map(|j| g(node(i,j))).collect());
    let most = values.iter().flatten().flatten().fold(0f64,|m,x| m.max(x.abs()));
    // the motion keeps the face in its own surface: in contact everywhere
    let speed = {
        let c = surface.point([lo[0]+0.5*span[0],lo[1]+0.5*span[1]]);
        crate::util::space::norm(twist.velocity(c)).max(crate::util::space::norm(twist.omega)*ru.max(rv))
    };
    if most <= 1e-12*speed.max(1e-300) { return Contact::Whole }
    let settled = 1e-13*most;
    let h = [1e-7*span[0],1e-7*span[1]];
    let grad = |uv: Uv| gradient(surface,twist,uv,h);
    let settle = |uv: Uv| settled_on(surface,twist,uv,h,Some([lo,hi]),settled);
    // the roots on the cells' edges, each edge read once: a horizontal one (i, j)–(i+1, j), a vertical
    // one (i, j)–(i, j+1)
    let mut roots: Vec<Uv> = Vec::new();
    let mut root_on = |a: Uv,b: Uv,ga: Option<f64>,gb: Option<f64>| -> Option<usize> {
        let (ga,gb) = (ga?,gb?);
        if (ga < 0.) == (gb < 0.) || ga == 0. && gb == 0. { return None }
        let x = crate::util::roots::bracketed_root(|x: f64| g([a[0]+(b[0]-a[0])*x,a[1]+(b[1]-a[1])*x]),0.,ga,1.,gb,1e-12);
        roots.push([a[0]+(b[0]-a[0])*x,a[1]+(b[1]-a[1])*x]);
        Some(roots.len()-1)
    };
    let across: Vec<Vec<Option<usize>>> = (0..n[0]).map(|i| (0..=n[1]).map(|j| root_on(node(i,j),node(i+1,j),values[i][j],values[i+1][j])).collect()).collect();
    let up: Vec<Vec<Option<usize>>> = (0..=n[0]).map(|i| (0..n[1]).map(|j| root_on(node(i,j),node(i,j+1),values[i][j],values[i][j+1])).collect()).collect();
    // the links: each cell's crossings joined in pairs, a saddle's through its double point where it
    // has one; and the double points, each a node of four links
    let mut links: Vec<[usize;2]> = Vec::new();
    let mut singular: std::collections::BTreeSet<usize> = std::collections::BTreeSet::new();
    for i in 0..n[0] { for j in 0..n[1] {
        // round the cell: its bottom, right, top and left edges
        let sides = [across[i][j],up[i+1][j],across[i][j+1],up[i][j]];
        let found: Vec<usize> = sides.iter().flatten().copied().collect();
        match found.len() {
            2 => links.push([found[0],found[1]]),
            4 => {
                let [b,r,t,l] = sides.map(Option::unwrap);
                // a double point: `∇g = 0` by Newton from the cell's middle, held where `g` is nought there
                let mid = [lo[0]+span[0]*(i as f64+0.5)/n[0] as f64,lo[1]+span[1]*(j as f64+0.5)/n[1] as f64];
                let cell = [span[0]/n[0] as f64,span[1]/n[1] as f64];
                let mut p = mid;
                for _ in 0..30 {
                    let (Some(gu0),Some(gu1),Some(gv0),Some(gv1)) = (grad([p[0]-h[0],p[1]]),grad([p[0]+h[0],p[1]]),grad([p[0],p[1]-h[1]]),grad([p[0],p[1]+h[1]])) else { break };
                    let Some(d) = grad(p) else { break };
                    let (a11,a21) = ((gu1[0]-gu0[0])/(2.*h[0]),(gu1[1]-gu0[1])/(2.*h[0]));
                    let (a12,a22) = ((gv1[0]-gv0[0])/(2.*h[1]),(gv1[1]-gv0[1])/(2.*h[1]));
                    let det = a11*a22-a12*a21;
                    if !(det.abs() > 0.) { break }
                    let step = [(a22*d[0]-a12*d[1])/det,(a11*d[1]-a21*d[0])/det];
                    p = [p[0]-step[0],p[1]-step[1]];
                    if step[0].abs() <= 1e-14*span[0] && step[1].abs() <= 1e-14*span[1] { break }
                }
                let held = (p[0]-mid[0]).abs() <= cell[0] && (p[1]-mid[1]).abs() <= cell[1]
                    && g(p).is_some_and(|v| v.abs() <= 1e-9*most);
                if held {
                    roots.push(p);
                    let x = roots.len()-1;
                    singular.insert(x);
                    for s in [b,r,t,l] { links.push([s,x]) }
                } else {
                    // two branches passing by: which corners the saddle's own value joins
                    let (g00,gc) = (values[i][j].unwrap_or(0.),g(mid).unwrap_or(0.));
                    if (gc < 0.) == (g00 < 0.) { links.push([b,r]); links.push([t,l]) } else { links.push([b,l]); links.push([t,r]) }
                }
            }
            _ => {}
        }
    } }
    // the polylines: from each node of other than two links (an end, a double point) out, then the
    // loops left
    let mut next: Vec<Vec<usize>> = vec![Vec::new();roots.len()];
    for (k,&[a,b]) in links.iter().enumerate() { next[a].push(k); next[b].push(k); }
    let mut used = vec![false;links.len()];
    let mut lines: Vec<(Vec<usize>,bool)> = Vec::new();
    let walk = |start: usize,first: usize,used: &mut Vec<bool>| -> Vec<usize> {
        let mut line = vec![start];
        let (mut at,mut via) = (start,first);
        loop {
            used[via] = true;
            let [a,b] = links[via];
            at = if a == at { b } else { a };
            line.push(at);
            if next[at].len() != 2 || singular.contains(&at) { break }
            let Some(&k) = next[at].iter().find(|&&k| !used[k]) else { break };
            via = k;
        }
        line
    };
    for s in 0..roots.len() {
        if next[s].len() == 2 && !singular.contains(&s) { continue }
        for &k in &next[s].clone() { if !used[k] { lines.push((walk(s,k,&mut used),false)) } }
    }
    for k in 0..links.len() {
        if used[k] { continue }
        let s = links[k][0];
        let line = walk(s,k,&mut used);
        let closed = line.first() == line.last();
        lines.push((line,closed));
    }
    // each polyline settled, cut where it leaves the face, its ends placed on the boundary
    let holds = |uv: Uv| located.holds_parameters(fi,uv);
    let on_box = |uv: Uv| (0..2).any(|k| (uv[k]-lo[k]).abs() <= 1e-9*span[k] || (uv[k]-hi[k]).abs() <= 1e-9*span[k]);
    let boundary_between = |inn: Uv,out: Uv| -> Uv {
        let wide = |a: Uv,b: Uv| (a[0]-b[0]).abs() > 1e-14*span[0] || (a[1]-b[1]).abs() > 1e-14*span[1];
        crate::util::roots::bisect(inn,out,|a,b| settle([(a[0]+b[0])/2.,(a[1]+b[1])/2.]),wide,holds).0
    };
    let mut pieces: Vec<Piece> = Vec::new();
    for (line,closed) in lines {
        let first = pieces.len();
        let pts: Vec<Uv> = line.iter().map(|&k| if singular.contains(&k) { roots[k] } else { settle(roots[k]) }).collect();
        let end_kind = |k: usize| if singular.contains(&k) || next[k].len() == 1 && !on_box(roots[k]) { End::Singular } else { End::Boundary };
        let inside: Vec<bool> = pts.iter().map(|&p| holds(p)).collect();
        // runs of points inside the face
        let mut run: Vec<Uv> = Vec::new();
        let mut start_end = End::Boundary;
        let n_pts = pts.len();
        let all_in = inside.iter().all(|&x| x);
        for k in 0..n_pts {
            if inside[k] {
                if run.is_empty() {
                    if k == 0 { start_end = end_kind(line[0]) }
                    else { run.push(boundary_between(pts[k],pts[k-1])); start_end = End::Boundary }
                }
                run.push(pts[k]);
                if k+1 == n_pts {
                    let last = end_kind(line[n_pts-1]);
                    let closed_here = closed && all_in;
                    pieces.push(Piece {face:fi,uv:std::mem::take(&mut run),points:Vec::new(),ends:[start_end,last],closed:closed_here});
                }
            } else if !run.is_empty() {
                run.push(boundary_between(pts[k-1],pts[k]));
                pieces.push(Piece {face:fi,uv:std::mem::take(&mut run),points:Vec::new(),ends:[start_end,End::Boundary],closed:false});
            }
        }
        // a loop that leaves the face and comes back: its last run goes on into its first
        if closed && !all_in && inside[0] && inside[n_pts-1] && pieces.len() > first+1 {
            let last = pieces.pop().unwrap();
            let head = &mut pieces[first];
            let mut uv = last.uv;
            uv.extend(head.uv.iter().skip(1).copied());
            head.uv = uv;
            head.ends[0] = last.ends[0];
        }
    }
    // pieces ending at one point either side of a seam, joined
    let ends_of = |p: &Piece| [surface.point(p.uv[0]),surface.point(*p.uv.last().unwrap())];
    let seam = |x: Uv,y: Uv| on_box(x) && on_box(y) && ((x[0]-y[0]).abs()/span[0]).max((x[1]-y[1]).abs()/span[1]) > 1e-6;
    let near = |x: V,y: V| distance(x,y) <= tol;
    loop {
        let ends: Vec<[V;2]> = pieces.iter().map(ends_of).collect();
        // a's last onto b's first, else onto b's last (b reversed)
        let found = (0..pieces.len()).filter(|&a| !pieces[a].closed).find_map(|a| (0..pieces.len()).filter(|&b| b != a && !pieces[b].closed).find_map(|b| {
            let (a1,[b0,b1]) = (*pieces[a].uv.last().unwrap(),[pieces[b].uv[0],*pieces[b].uv.last().unwrap()]);
            if near(ends[a][1],ends[b][0]) && seam(a1,b0) { Some((a,b,false)) } else if near(ends[a][1],ends[b][1]) && seam(a1,b1) { Some((a,b,true)) } else { None }
        }));
        let Some((a,b,rev)) = found else { break };
        let mut tail = pieces.remove(b);
        if rev { tail.uv.reverse(); tail.ends.reverse(); }
        let head = &mut pieces[if b < a { a-1 } else { a }];
        head.uv.extend(tail.uv);
        head.ends[1] = tail.ends[1];
    }
    for p in &mut pieces {
        if !p.closed && p.uv.len() > 2 && on_box(p.uv[0]) { let e = ends_of(p); if near(e[0],e[1]) { p.closed = true; } }
    }
    // every chord within the tolerance of the curve: points added between, settled
    for p in &mut pieces {
        let mut out: Vec<Uv> = Vec::with_capacity(p.uv.len());
        fn fill(a: Uv,b: Uv,depth: u32,out: &mut Vec<Uv>,surface: &Surface,settle: &dyn Fn(Uv) -> Uv,tol: f64) {
            if depth == 0 { out.push(b); return }
            let m = settle([(a[0]+b[0])/2.,(a[1]+b[1])/2.]);
            let chord = scale(add(surface.point(a),surface.point(b)),0.5);
            if distance(surface.point(m),chord) <= tol { out.push(b); return }
            fill(a,m,depth-1,out,surface,settle,tol);
            fill(m,b,depth-1,out,surface,settle,tol);
        }
        out.push(p.uv[0]);
        for w in p.uv.windows(2) {
            // a seam crossed between two points is no chord
            if (w[0][0]-w[1][0]).abs() > 0.5*span[0] || (w[0][1]-w[1][1]).abs() > 0.5*span[1] { out.push(w[1]); continue }
            fill(w[0],w[1],12,&mut out,surface,&settle,tol);
        }
        p.uv = out;
        p.points = p.uv.iter().map(|&uv| surface.point(uv)).collect();
    }
    Contact::Pieces(pieces)
}

/// The gradient of the contact `g` of `surface` under `twist` in its parameters at `uv`, by central
/// differences `h` apart; none where the normal is not defined there.
fn gradient(surface: &Surface,twist: &Twist,uv: Uv,h: [f64;2]) -> Option<[f64;2]> {
    let g = |uv: Uv| surface.normal_velocity(uv,twist);
    Some([(g([uv[0]+h[0],uv[1]])?-g([uv[0]-h[0],uv[1]])?)/(2.*h[0]),(g([uv[0],uv[1]+h[1]])?-g([uv[0],uv[1]-h[1]])?)/(2.*h[1])])
}

/// `uv` settled onto the contact `g = 0` of `surface` under `twist` by Newton across the curve (along
/// `g`'s gradient, `gradient`), held within the parameter box `within` where there is one, until `g`
/// is within `settled` of nought or a step is a hundred-millionth of the differences' `h`.
fn settled_on(surface: &Surface,twist: &Twist,mut uv: Uv,h: [f64;2],within: Option<[Uv;2]>,settled: f64) -> Uv {
    for _ in 0..30 {
        let (Some(v),Some(d)) = (surface.normal_velocity(uv,twist),gradient(surface,twist,uv,h)) else { break };
        if v.abs() <= settled { break }
        let dd = d[0]*d[0]+d[1]*d[1];
        if !(dd > 0.) { break }
        let step = [v*d[0]/dd,v*d[1]/dd];
        uv = [uv[0]-step[0],uv[1]-step[1]];
        if let Some([lo,hi]) = within { uv = [uv[0].clamp(lo[0],hi[0]),uv[1].clamp(lo[1],hi[1])] }
        if step[0].abs() <= 1e-8*h[0] && step[1].abs() <= 1e-8*h[1] { break }
    }
    uv
}

/// **A piece as a curve**: a cubic B-spline over `[0, 1]`, by chord length along the piece, fitted
/// within a tenth of `tol` of the contact itself — each sample placed along the piece and settled onto
/// `g = 0` under `twist` (a seam crossed between two points unwrapped by the surface's period) — from
/// sixteen spans, doubled as the fit asks.
pub fn piece_curve(tool: &Brep,piece: &Piece,twist: &Twist,tol: f64) -> Option<crate::geom::Curve> {
    let surface = &tool.faces[piece.face].surface;
    let periods = surface.periods();
    let pts = &piece.points;
    let mut at = vec![0.];
    for w in pts.windows(2) { let last = *at.last().unwrap(); at.push(last+distance(w[0],w[1])); }
    let length = *at.last().unwrap();
    if !(length > 0.) { return None }
    // the parameters' scale, for the settling's differences
    let (lo,hi) = piece.uv.iter().fold(([f64::INFINITY;2],[f64::NEG_INFINITY;2]),|(lo,hi),p| ([lo[0].min(p[0]),lo[1].min(p[1])],[hi[0].max(p[0]),hi[1].max(p[1])]));
    let h = [1e-7*(hi[0]-lo[0]).max(1e-3),1e-7*(hi[1]-lo[1]).max(1e-3)];
    let along = |t: f64| -> V {
        let s = t.clamp(0.,1.)*length;
        let k = at.partition_point(|&x| x <= s).clamp(1,pts.len()-1);
        let (a,b) = (at[k-1],at[k]);
        let f = if b > a { (s-a)/(b-a) } else { 0. };
        let u0 = piece.uv[k-1];
        let u1 = crate::topo::unwrap(piece.uv[k],u0,periods);
        let uv = [u0[0]+(u1[0]-u0[0])*f,u0[1]+(u1[1]-u0[1])*f];
        if f == 0. { return pts[k-1] }
        surface.point(settled_on(surface,twist,uv,h,None,0.))
    };
    let (curve,_) = crate::nurbs::fit_curve(&along,3,16,0.1*tol)?;
    Some(crate::geom::Curve::BSpline(std::sync::Arc::new(curve)))
}

/// The rigid map carrying what stands at the pose of `t0` to where it stands at `t1` under `path`.
fn between(path: &crate::trajectory::Trajectory,t0: f64,t1: f64) -> Result<crate::geom::Rigid,super::closed::Unbuilt> {
    let m = path.pose(t0)?.inverse().then(path.pose(t1)?);
    Ok(crate::geom::Rigid {r:m.r,t:m.p})
}

/// **The sheet `curve` sweeps carried by the constant twist `path` over `roll`** (#880 Q1b S3): the
/// curve, on the tool at its own placement, placed at the roll's start and carried exactly to its end
/// — one face of a blend (`Surface::Blend`) between the curve and its image, its rails the paths of the
/// curve's ends (`Curve::Iso`), a closed curve's one rail its seam. The carry is held to the path's own
/// pose at the roll's end.
pub fn carried(curve: &crate::geom::Curve,closed: bool,path: &crate::trajectory::Trajectory,roll: [f64;2])
    -> Result<Carried,super::closed::Unbuilt> {
    use crate::geom::{Blend,Carry,Curve,Frame};
    use crate::topo::EdgeCurve;
    use crate::topo::{Coedge,Face,Pcurve};
    use super::closed::Unbuilt;
    let screw = path.screw().ok_or(Unbuilt::Motion)?;
    let start = between(path,0.,roll[0])?;
    let a = curve.moved(&start);
    let (span,axis) = (roll[1]-roll[0],crate::util::space::unit(screw.axis));
    let rise = screw.advance/std::f64::consts::TAU*span;
    let carry = if screw.is_slide() { Carry::Line {delta:scale(axis,rise)} }
        else { Carry::Screw {center:screw.origin,axis,angle:screw.ratio*span,advance:rise} };
    // the carry is the path's: the curve's points carried agree with its pose at the roll's end
    let whole = between(path,roll[0],roll[1])?;
    for k in 0..=8 {
        let p = a.point(k as f64/8.);
        let (x,y) = (carry.at(p,1.),whole.point(p));
        if distance(x,y) > 1e-9*(1.+crate::util::space::norm(p)) {
            return Err(Unbuilt::Failed(format!("a contact's carry is not the path's: {x:?} against {y:?}")))
        }
    }
    let image = a.moved(&whole);
    let rims = [a.clone(),image.clone()];
    let blend = std::sync::Arc::new(Blend {a:a.clone(),ta:[0.,1.],b:a.clone(),tb:[0.,1.],carry,closed});
    let mut b = Brep::default();
    let p0 = b.vertex(a.point(0.));
    let p1 = if closed { p0 } else { b.vertex(a.point(1.)) };
    let q0 = b.vertex(image.point(0.));
    let q1 = if closed { q0 } else { b.vertex(image.point(1.)) };
    let ea = b.edge(EdgeCurve::Curve(a.clone()),[0.,1.],[p0,p1]);
    let eb = b.edge(EdgeCurve::Curve(image),[0.,1.],[q0,q1]);
    let r0 = b.edge(EdgeCurve::Curve(Curve::Iso(blend.clone(),0.)),[0.,1.],[p0,q0]);
    let r1 = if closed { r0 } else { b.edge(EdgeCurve::Curve(Curve::Iso(blend.clone(),1.)),[0.,1.],[p1,q1]) };
    let line = |x: [f64;2],y: [f64;2]| Pcurve::Line {a:x,b:y};
    let mut l = vec![
        Coedge {edge:ea,reversed:false,pcurve:line([0.,0.],[1.,0.])},
        Coedge {edge:r1,reversed:false,pcurve:line([1.,0.],[1.,1.])},
        Coedge {edge:eb,reversed:true,pcurve:line([0.,1.],[1.,1.])},
        Coedge {edge:r0,reversed:true,pcurve:line([0.,0.],[0.,1.])},
    ];
    let mid = a.point(0.5);
    let surface = crate::geom::Surface::Blend(Frame::about(mid,axis),blend);
    let face = Face {surface,reversed:false,loops:vec![],name:"contact".into()};
    if b.loop_area(&face,&l) < 0. { l = crate::topo::reverse_loop(l); }
    b.faces.push(Face {loops:vec![l],..face});
    Ok(Carried {sheet:b,rims})
}

/// A piece's sheet carried over the roll (`carried`), and its rims: the piece at the roll's start
/// and at its end, where the sheet meets the tool placed there.
pub struct Carried { pub sheet: Brep,pub rims: [crate::geom::Curve;2] }

/// **The candidates of the closed solid `tool` sweeps under the constant twist `path` over `roll`**,
/// short of a whole turn (#880 Q1b S4): each piece of each face's contact carried over the roll, and
/// the tool at the roll's two ends. Refused, by name, where a face is in contact all over (`Whole`) or
/// the tool has an edge whose fan is in contact (S2, not yet read).
/// With them, the meetings known by construction (P4): each sheet meets its face at either end pose in
/// that end's rim (`crate::boolean::Given`), never searched for.
pub fn candidates(tool: &Brep,path: &crate::trajectory::Trajectory,roll: [f64;2],tol: f64)
    -> Result<(Vec<Brep>,Vec<crate::boolean::Given>),super::closed::Unbuilt> {
    use super::closed::Unbuilt;
    let screw = path.screw().ok_or(Unbuilt::Motion)?;
    // (refused first, before any face is read: an edge between two faces not smooth there: its fan's contact is S2's)
    let uses = tool.uses();
    for (e,edge) in tool.edges.iter().enumerate() {
        if matches!(edge.curve,crate::topo::EdgeCurve::Degenerate) { continue }
        let [(f1,..),(f2,..)] = uses[e][..] else { continue };
        if f1 == f2 { continue }
        let mid = edge.point(0.5*(edge.t[0]+edge.t[1]),&tool.vertices);
        let (Some(n1),Some(n2)) = (tool.faces[f1].outward(mid),tool.faces[f2].outward(mid)) else { return Err(Unbuilt::Curved {at:mid}) };
        if crate::util::space::dot(n1,n2) < 1.-1e-9 { return Err(Unbuilt::Curved {at:mid}) }
    }
    let twist = screw.twist();
    let located = Located::new(tool,tol);
    let mut out = Vec::new();
    let mut given = Vec::new();
    let posed = |t: f64| -> Result<crate::geom::Rigid,Unbuilt> { let q = path.pose(t)?; Ok(crate::geom::Rigid {r:q.r,t:q.p}) };
    let ends = [posed(roll[0])?,posed(roll[1])?];
    for fi in 0..tool.faces.len() {
        match face_contact(tool,&located,fi,&twist,tol,CELLS) {
            Contact::Whole => {
                let at = tool.faces[fi].loops.iter().flatten().next().map_or([0.;3],|c| tool.vertices[tool.ends(c)[0] as usize].p);
                return Err(Unbuilt::Curved {at})
            }
            Contact::Pieces(pieces) => for p in pieces {
                let curve = piece_curve(tool,&p,&twist,tol).ok_or_else(|| Unbuilt::Failed("a piece of contact not fitted".into()))?;
                let c = carried(&curve,p.closed,path,roll)?;
                for (k,rim) in c.rims.iter().enumerate() {
                    given.push(crate::boolean::Given {surfaces:[tool.faces[fi].surface.moved(&ends[k]),c.sheet.faces[0].surface.clone()],curves:vec![rim.clone()]});
                }
                out.push(c.sheet);
            },
        }
    }
    for m in &ends { out.push(tool.moved(m)); }
    Ok((out,given))
}
