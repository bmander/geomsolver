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
    let n = [cells.max((cells as f64*ru/rv.max(1e-300)).min(4.*cells as f64) as usize),cells.max((cells as f64*rv/ru.max(1e-300)).min(4.*cells as f64) as usize)];
    let node = |i: usize,j: usize| -> Uv { [lo[0]+span[0]*i as f64/n[0] as f64,lo[1]+span[1]*j as f64/n[1] as f64] };
    let values: Vec<Vec<Option<f64>>> = (0..=n[0]).map(|i| (0..=n[1]).map(|j| g(node(i,j))).collect()).collect();
    let most = values.iter().flatten().flatten().fold(0f64,|m,x| m.max(x.abs()));
    // the motion keeps the face in its own surface: in contact everywhere
    let speed = {
        let c = surface.point([lo[0]+0.5*span[0],lo[1]+0.5*span[1]]);
        crate::util::space::norm(twist.velocity(c)).max(crate::util::space::norm(twist.omega)*ru.max(rv))
    };
    if most <= 1e-12*speed.max(1e-300) { return Contact::Whole }
    let settled = 1e-13*most;
    // the gradient of `g` in the parameters, by central differences
    let h = [1e-7*span[0],1e-7*span[1]];
    let grad = |uv: Uv| -> Option<[f64;2]> {
        let du = (g([uv[0]+h[0],uv[1]])?-g([uv[0]-h[0],uv[1]])?)/(2.*h[0]);
        let dv = (g([uv[0],uv[1]+h[1]])?-g([uv[0],uv[1]-h[1]])?)/(2.*h[1]);
        Some([du,dv])
    };
    // a point settled onto `g = 0` by Newton across the curve (along the gradient), kept in the box
    let settle = |mut uv: Uv| -> Uv {
        for _ in 0..30 {
            let (Some(v),Some(d)) = (g(uv),grad(uv)) else { break };
            if v.abs() <= settled { break }
            let dd = d[0]*d[0]+d[1]*d[1];
            if !(dd > 0.) { break }
            let step = [v*d[0]/dd,v*d[1]/dd];
            uv = [(uv[0]-step[0]).clamp(lo[0],hi[0]),(uv[1]-step[1]).clamp(lo[1],hi[1])];
            if step[0].abs() <= 1e-15*span[0] && step[1].abs() <= 1e-15*span[1] { break }
        }
        uv
    };
    // the roots on the cells' edges: a horizontal edge (i, j)–(i+1, j), a vertical one (i, j)–(i, j+1)
    let mut roots: Vec<Uv> = Vec::new();
    let mut on_edge: std::collections::BTreeMap<(u8,usize,usize),usize> = std::collections::BTreeMap::new();
    let mut root_on = |kind: u8,i: usize,j: usize,roots: &mut Vec<Uv>| -> Option<usize> {
        if let Some(&r) = on_edge.get(&(kind,i,j)) { return Some(r) }
        let (a,b) = if kind == 0 { (node(i,j),node(i+1,j)) } else { (node(i,j),node(i,j+1)) };
        let (ga,gb) = if kind == 0 { (values[i][j]?,values[i+1][j]?) } else { (values[i][j]?,values[i][j+1]?) };
        if (ga < 0.) == (gb < 0.) || ga == 0. && gb == 0. { return None }
        let at = |x: f64| g([a[0]+(b[0]-a[0])*x,a[1]+(b[1]-a[1])*x]);
        let x = crate::util::roots::bracketed_root(at,0.,ga,1.,gb,1e-12);
        roots.push([a[0]+(b[0]-a[0])*x,a[1]+(b[1]-a[1])*x]);
        on_edge.insert((kind,i,j),roots.len()-1);
        Some(roots.len()-1)
    };
    // the links: each cell's crossings joined in pairs, a saddle's through its double point where it
    // has one; and the double points, each a node of four links
    let mut links: Vec<[usize;2]> = Vec::new();
    let mut singular: std::collections::BTreeSet<usize> = std::collections::BTreeSet::new();
    for i in 0..n[0] { for j in 0..n[1] {
        // round the cell: its bottom, right, top and left edges
        let sides = [root_on(0,i,j,&mut roots),root_on(1,i+1,j,&mut roots),root_on(0,i,j+1,&mut roots),root_on(1,i,j,&mut roots)];
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
        let (mut a,mut b) = (inn,out);
        for _ in 0..60 {
            let m = settle([(a[0]+b[0])/2.,(a[1]+b[1])/2.]);
            if holds(m) { a = m } else { b = m }
            if (a[0]-b[0]).abs() <= 1e-14*span[0] && (a[1]-b[1]).abs() <= 1e-14*span[1] { break }
        }
        a
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
    let point = |uv: Uv| surface.point(uv);
    let mut joined = true;
    while joined {
        joined = false;
        'outer: for a in 0..pieces.len() {
            if pieces[a].closed { continue }
            for b in 0..pieces.len() {
                if b == a || pieces[b].closed { continue }
                let (pa,pb) = (&pieces[a],&pieces[b]);
                let ends = |p: &Piece| [point(p.uv[0]),point(*p.uv.last().unwrap())];
                let (ea,eb) = (ends(pa),ends(pb));
                let seam = |x: Uv,y: Uv| on_box(x) && on_box(y) && ((x[0]-y[0]).abs()/span[0]).max((x[1]-y[1]).abs()/span[1]) > 1e-6;
                let ([_,a1],[b0,b1]) = ([pa.uv[0],*pa.uv.last().unwrap()],[pb.uv[0],*pb.uv.last().unwrap()]);
                let near = |x: V,y: V| distance(x,y) <= tol;
                // a's last onto b's first, else onto b's last (b reversed)
                let found = if near(ea[1],eb[0]) && seam(a1,b0) { Some(false) } else if near(ea[1],eb[1]) && seam(a1,b1) { Some(true) } else { None };
                let Some(rev) = found else { continue };
                let mut tail = pieces[b].clone();
                if rev { tail.uv.reverse(); tail.ends.reverse(); }
                let mut head = pieces[a].clone();
                head.uv.extend(tail.uv);
                head.ends[1] = tail.ends[1];
                pieces[a] = head;
                pieces.remove(b);
                joined = true;
                break 'outer;
            }
        }
    }
    for p in &mut pieces {
        if !p.closed && p.uv.len() > 2 && distance(point(p.uv[0]),point(*p.uv.last().unwrap())) <= tol && on_box(p.uv[0]) { p.closed = true; }
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
