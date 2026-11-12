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
    // where `∇g = 0` near `from`, by Newton
    let cell = [span[0]/n[0] as f64,span[1]/n[1] as f64];
    let critical = |from: Uv| -> Uv {
        let mut p = from;
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
        p
    };
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
                let p = critical(mid);
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
    // **a piece is cut at every double point it passes** (#880 Q1b; S1: each piece ends on the face's
    // boundary or where `∇g = 0`): where the grid has not seen one as a saddle — one standing on a
    // node (the leaning torus's, on the crossing of its two seams, where the one closed piece ran
    // through it twice and turned a corner each time, its sheets creased along that point's path),
    // or one the walk ran straight through along both its branches (the torus's second, whose two
    // stretches' sheets touch along its path). Read where the piece turns sharply, which the
    // contact, smooth but where two branches cross, does only there, and where `|∇g|` falls to a least
    // and Newton holds a double point; cut there, placed on the double point, each end `Singular`;
    // a closed piece opened at its first
    let size = tool.size();
    let mut split: Vec<Piece> = Vec::new();
    for p in pieces {
        let m = p.points.len();
        // the turn at point `k`, between the chords to the nearest points either side standing apart from it
        let turn = |k: usize| -> bool {
            let apart = 1e-9*size;
            let step = |d: isize| -> Option<usize> {
                let mut j = k as isize;
                loop {
                    j += d;
                    let j_ = if p.closed { j.rem_euclid(m as isize-1) as usize } else if j < 0 || j >= m as isize { return None } else { j as usize };
                    if distance(p.points[j_],p.points[k]) > apart { return Some(j_) }
                    if (j-k as isize).unsigned_abs() >= m { return None }
                }
            };
            let (Some(a),Some(b)) = (step(-1),step(1)) else { return false };
            let (d1,d2) = (crate::util::space::sub(p.points[k],p.points[a]),crate::util::space::sub(p.points[b],p.points[k]));
            crate::util::space::dot(d1,d2) < 0.866*crate::util::space::norm(d1)*crate::util::space::norm(d2)
        };
        let last = if p.closed { m-1 } else { m };
        let inner = if p.closed { 0 } else { 1 }..last.saturating_sub(if p.closed { 0 } else { 1 });
        // the double point near point `k`, where Newton on `∇g = 0` finds one within a cell of it
        let double = |k: usize| -> Option<Uv> {
            let c = critical(p.uv[k]);
            ((c[0]-p.uv[k][0]).abs() <= cell[0] && (c[1]-p.uv[k][1]).abs() <= cell[1] && g(c).is_some_and(|v| v.abs() <= 1e-9*most)).then_some(c)
        };
        // and the points a piece passes straight through a double point at: where `|∇g|` falls to a
        // least a tenth of its middling value along the piece and Newton holds a double point there
        // (the leaning torus's second, which the piece crossed twice without turning)
        let steep: Vec<f64> = p.uv.iter().map(|&uv| grad(uv).map_or(f64::NAN,|d| (d[0]*span[0]).hypot(d[1]*span[1]))).collect();
        let middling = { let mut v: Vec<f64> = steep.iter().copied().filter(|x| x.is_finite()).collect(); v.sort_by(f64::total_cmp); v.get(v.len()/2).copied().unwrap_or(0.) };
        let wrap = |k: isize| if p.closed { k.rem_euclid(m as isize-1) as usize } else { k.clamp(0,m as isize-1) as usize };
        let least = |k: usize| steep[k] < 0.1*middling && steep[k] <= steep[wrap(k as isize-1)] && steep[k] <= steep[wrap(k as isize+1)];
        let mut corners: Vec<usize> = inner.filter(|&k| turn(k) || least(k) && double(k).is_some()).collect();
        // (one double point once: a corner beside the least of `|∇g|`)
        corners.dedup_by(|b,a| *b <= *a+2 && double(*a).zip(double(*b)).is_some_and(|(x,y)| distance(surface.point(x),surface.point(y)) <= tol));
        if corners.is_empty() { split.push(p); continue }
        let at = |k: usize| -> (Uv,V) {
            match double(k) { Some(c) => (c,surface.point(c)),None => (p.uv[k],p.points[k]) }
        };
        // the points in order from the first corner (round a closed piece to it again), and the corners along them
        let (order,cuts): (Vec<usize>,Vec<usize>) = if p.closed {
            let c0 = corners[0];
            let order: Vec<usize> = (0..m).map(|i| (c0+i)%(m-1)).collect();
            (order,corners.iter().map(|&c| (c+m-1-c0)%(m-1)).chain(std::iter::once(m-1)).collect())
        } else {
            ((0..m).collect(),std::iter::once(0).chain(corners.iter().copied()).chain(std::iter::once(m-1)).collect())
        };
        for (w,pair) in cuts.windows(2).enumerate() {
            let (s,e) = (pair[0],pair[1]);
            if e <= s { continue }
            let mut uv: Vec<Uv> = order[s..=e].iter().map(|&i| p.uv[i]).collect();
            let mut points: Vec<V> = order[s..=e].iter().map(|&i| p.points[i]).collect();
            let first = p.closed || w > 0;
            let final_ = p.closed || w+2 < cuts.len();
            if first { let (c,x) = at(order[s]); uv[0] = c; points[0] = x; }
            if final_ { let (c,x) = at(order[e]); *uv.last_mut().unwrap() = c; *points.last_mut().unwrap() = x; }
            let ends = [if first { End::Singular } else { p.ends[0] },if final_ { End::Singular } else { p.ends[1] }];
            split.push(Piece {face:p.face,uv,points,ends,closed:false});
        }
    }
    Contact::Pieces(split)
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

/// **A piece as a curve**: the cubic B-spline through points of the contact, by chord length,
/// within a tenth of `tol` of the contact itself — from seventeen of the piece's own points spread
/// along it, its ends among them; every span whose middle stands farther than that off the contact
/// (the middle's foot on the surface settled onto `g = 0` under `twist`) given a point of the contact
/// between its two, the parameters' midpoint settled (a seam crossed between them unwrapped by the
/// surface's period), until none does. Each point is a point of the curve and the spline is smooth
/// in its chord length, so the fit closes at a cubic's rate: read instead along the piece's polyline
/// by its own chord length, whose speed jumps at each vertex, it closed at a line's, and doubled its
/// spans to four thousand, an inverse on the sheet carrying it reading them all (#880 Q1b).
pub fn piece_curve(tool: &Brep,piece: &Piece,twist: &Twist,tol: f64) -> Option<crate::geom::Curve> {
    let surface = &tool.faces[piece.face].surface;
    let periods = surface.periods();
    let n = piece.points.len();
    if n < 2 { return None }
    // the parameters' scale, for the settling's differences
    let (lo,hi) = piece.uv.iter().fold(([f64::INFINITY;2],[f64::NEG_INFINITY;2]),|(lo,hi),p| ([lo[0].min(p[0]),lo[1].min(p[1])],[hi[0].max(p[0]),hi[1].max(p[1])]));
    let h = [1e-7*(hi[0]-lo[0]).max(1e-3),1e-7*(hi[1]-lo[1]).max(1e-3)];
    // the piece's parameters unwrapped along it, so a midpoint between two is between them
    let mut uv = vec![piece.uv[0]];
    for k in 1..n { let last = uv[k-1]; uv.push(crate::topo::unwrap(piece.uv[k],last,periods)); }
    let mut at = vec![0.];
    for w in piece.points.windows(2) { let last = *at.last().unwrap(); at.push(last+distance(w[0],w[1])); }
    if !(at[n-1] > 0.) { return None }
    let mut idx: Vec<usize> = (0..=16).map(|k| at.partition_point(|&x| x < at[n-1]*k as f64/16.).min(n-1)).collect();
    idx[0] = 0;
    idx[16] = n-1;
    idx.dedup();
    let nodes: Vec<(Uv,V)> = idx.iter().map(|&i| (uv[i],piece.points[i])).collect();
    let between = |a: Uv,b: Uv| { let m = settled_on(surface,twist,[0.5*(a[0]+b[0]),0.5*(a[1]+b[1])],h,None,0.); (m,surface.point(m)) };
    fitted(nodes,&between,tol)
}

/// **The cubic B-spline through points of a curve known implicitly**, by chord length, within a
/// tenth of `tol` of the curve: `nodes` its first points (parameters and place), every span whose
/// middle stands farther than that off the curve given the curve's point between its two
/// (`between`, from their parameters), until none does — each node a point of the curve, so the fit
/// closes at a cubic's rate (`piece_curve`, `pose_meeting`). A span's middle is measured from that
/// point across the span's chord: along it, the two differ only in where along the curve each
/// stands, and the point is the node the span would be split at.
fn fitted(mut nodes: Vec<(Uv,V)>,between: &(dyn Fn(Uv,Uv) -> (Uv,V)+Sync),tol: f64) -> Option<crate::geom::Curve> {
    const MOST: usize = 4097;
    // (a node standing on the one before it is no node: the interpolation's parameters must rise —
    // two of a march's roots at one place, near where a branch turns back across the grid, refused
    // the whole branch, and its sheet was met by no curve there)
    // (and nodes within the tolerance of one another are one: a cubic interpolated through two such,
    // its parameters a hair apart, swung far off between them — 160 off a branch of a stretch's
    // meeting with the tool, beside the double point it leaves, that no span long enough to be
    // read showed)
    let apart = |a: V,b: V| distance(a,b) > tol;
    let last = *nodes.last()?;
    nodes.dedup_by(|b,a| !apart(a.1,b.1));
    if nodes.len() < 2 { return None }
    if let Some(end) = nodes.last_mut() { *end = last; }
    // each span's point between, kept until the span is split
    let mut mids: Vec<Option<(Uv,V)>> = vec![None;nodes.len()-1];
    loop {
        let mut t = vec![0.];
        for w in nodes.windows(2) { let last = *t.last().unwrap(); t.push(last+distance(w[0].1,w[1].1)); }
        let total = *t.last().unwrap();
        if !(total > 0.) { return None }
        let t: Vec<f64> = t.into_iter().map(|x| x/total).collect();
        let spline = crate::nurbs::interpolate(&nodes.iter().map(|x| x.1).collect::<Vec<_>>(),&t,3)?;
        for k in 0..mids.len() { if mids[k].is_none() { mids[k] = Some(between(nodes[k].0,nodes[k+1].0)) } }
        // (a span no longer than the tolerance is not split: its middle is off the curve by what its
        // ends are, where a node is set on a point known exactly and the curve read near it only to
        // its own rounding — a branch's end on a double point of the contact)
        let wide: Vec<usize> = (0..nodes.len()-1).filter(|&k| {
            let chord = crate::util::space::sub(nodes[k+1].1,nodes[k].1);
            let d = crate::util::space::sub(spline.point(0.5*(t[k]+t[k+1])),mids[k].unwrap().1);
            let along = crate::util::space::dot(d,chord)/crate::util::space::dot(chord,chord);
            norm_of(crate::util::space::sub(d,scale(chord,along))) > 0.1*tol
        }).collect();
        if wide.is_empty() || nodes.len()+wide.len() > MOST {
            return Some(crate::geom::Curve::BSpline(std::sync::Arc::new(spline)))
        }
        let mut grew = false;
        for &k in wide.iter().rev() {
            let m = mids[k].unwrap();
            // (a point between within the tolerance of an end of its span splits nothing)
            if !apart(m.1,nodes[k].1) || !apart(m.1,nodes[k+1].1) { continue }
            nodes.insert(k+1,m);
            mids.splice(k..=k,[None,None]);
            grew = true;
        }
        if !grew { return Some(crate::geom::Curve::BSpline(std::sync::Arc::new(spline))) }
    }
}

fn norm_of(v: V) -> f64 { crate::util::space::norm(v) }

/// **A sheet's whole meeting with a face of the tool posed at an end of the roll** (#880 Q1b): the
/// sheet `S(u, v) = carry_v(a(u))`, the face's surface `surface`, and their meeting the zero set of
/// `F(u, v)`, the surface's signed distance at `S(u, v)`, read in the sheet's own parameters. Where
/// the face is the one the section lies on and the pose the sheet's end at `v = e` (`touch`), the
/// sheet touches it along that end's rim: `F` vanishes there twice over, `F = (v − e)² h`, and the
/// meeting is the rim (the caller's) and the zero set of `h` — what of the sheet passes back into the
/// tool, met transversally, reaching the rim where `h` changes sign along it (where the two surfaces'
/// curvatures across it change order) and leaving it at a double point of the contact tangent to it,
/// as a curve and not as a step a march cannot land. `h` is read as `F` less its value and rate at
/// the rim (the section's fit, off the face by up to a tenth of the tolerance, and the contact's own
/// rounding), over `(v − e)²` — its value and rate read exactly, and at the rim by differences of
/// that rate. Each branch is marched across
/// a grid of the parameters as the contact is (`face_contact`), its crossings of the cells' edges
/// exact, and fitted through points of it within a tenth of the tolerance (`fitted`). A branch ending
/// beside one of `corners` — where the section ends at a double point of the contact, on the rim, and
/// the branch leaves through it, `h` nought there but for its differences' rounding — by no more than
/// that rounding puts it off (`h` there over its slope toward the end) ends there exactly: its root
/// fell a hair beside it (7e-7 of the section on the leaning torus), beside the rim and the rail that
/// end there.
fn pose_meeting(blend: &crate::geom::Blend,surface: &Surface,touch: Option<(f64,u32)>,corners: &[Uv],tol: f64)
    -> Vec<crate::geom::Curve> {
    let a = &blend.a;
    let at = |u: f64,v: f64| blend.carry.at(a.point(u),v);
    let f = |u: f64,v: f64| surface.implicit(at(u,v));
    // `F`'s value and rate at the rim read exactly — its value, and its gradient along the carry's own
    // velocity (`Carry::rate`) — and its second and third derivatives by differences of that rate a
    // hundred-thousandth of the roll apart: within that, `h` its Taylor series' `c₂ + c₃ w`; beyond,
    // `F` less its value and rate, over `w²`, and between the series passing smoothly into it — each
    // read where its rounding is least, and the two one function. (By five-point differences of `F`
    // a thousandth of the roll apart, the rate took the series' fifth-order truncation: on the leaning
    // torus, where the carry moves a point a tenth of a millimetre in that step past a tube of radius
    // 2 and `F`'s cubic term is two thousand times its square's, `c₁` read 2.7e-5 for its own 1e-8,
    // and `h`'s zero set beside the rim stood that much times `w` off the tool — 4e-8 at the kernel's
    // tolerance, which the check refused.)
    const STEP: f64 = 1e-3;
    const SERIES: f64 = 1e-5;
    let rate = |u: f64,v: f64| { let q = at(u,v); crate::util::space::dot(surface.gradient(q),blend.carry.rate(q)) };
    let taylor = |u: f64,e: f64| -> [f64;4] {
        let (r0,r1,m1) = (rate(u,e),rate(u,e+SERIES),rate(u,e-SERIES));
        [f(u,e),r0,(r1-m1)/(4.*SERIES),(r1-2.*r0+m1)/(6.*SERIES*SERIES)]
    };
    // (a section crossing the face simply, an edge's fan on either of its faces: `F = (v − e) h`, `h` its
    // value less `c₀` over `v − e`, and `c₁ + c₂ w` within the step)
    // (the series within half its step, `F`'s own form past it, and between, the one passing smoothly
    // into the other: they part by the series' truncation there, and a branch crossing where one
    // jumped to the other had its points between two curves, which the fit chased)
    let h = |u: f64,v: f64| -> f64 {
        let Some((e,order)) = touch else { return f(u,v) };
        let [c0,c1,c2,c3] = taylor(u,e);
        let w = v-e;
        let series = if order == 1 { c1+c2*w } else { c2+c3*w };
        let own = || if order == 1 { (f(u,v)-c0)/w } else { (f(u,v)-c0-c1*w)/(w*w) };
        if w.abs() <= 0.5*SERIES { return series }
        if w.abs() >= SERIES { return own() }
        let x = (w.abs()-0.5*SERIES)/(0.5*SERIES);
        let mu = 1.-x*x*(3.-2.*x);
        mu*series+(1.-mu)*own()
    };
    // **what a branch is settled on**: `h` within a step of the rim, `F`'s zero set beyond four, and
    // between, `F` less a share of the rim's value and rate that falls smoothly from all of it to none
    // (`λ`, a smoothstep) — one zero set carried from `h`'s into `F`'s. Settled on `h` within the step
    // and on `F` past it, a branch crossing the step's edge had its points between two curves that
    // part there by the rim's value and rate over `w²` and `h`'s slope, a hair at 1e-5 and past a
    // tenth of the kernel's tolerance; the fit chased them, its nodes stepping back and forth across
    // that hair until four thousand stood in a span (`fitted`: two of the leaning torus's branches,
    // the build twice as long)
    let k = |u: f64,v: f64| -> f64 {
        let Some((e,order)) = touch else { return f(u,v) };
        let w = v-e;
        if w.abs() < STEP { return h(u,v) }
        let x = ((w.abs()-STEP)/(3.*STEP)).min(1.);
        let share = 1.-x*x*(3.-2.*x);
        if share == 0. { return f(u,v)/if order == 1 { w } else { w*w } }
        let [c0,c1,..] = taylor(u,e);
        if order == 1 { return (f(u,v)-share*c0)/w }
        (f(u,v)-share*(c0+c1*w))/(w*w)
    };
    // the grid: as many cells each way as its sides' lengths in space ask, 64 at least
    let length = |p: &dyn Fn(f64) -> V| (0..32).map(|k| distance(p(k as f64/32.),p((k+1) as f64/32.))).sum::<f64>();
    let (lu,lv) = (length(&|u| at(u,0.5)),length(&|v| at(0.5,v)));
    let n = [(lu,lv),(lv,lu)].map(|(x,y)| 64usize.max((64.*x/y.max(1e-300)).min(256.) as usize));
    let node = |i: usize,j: usize| -> Uv { [i as f64/n[0] as f64,j as f64/n[1] as f64] };
    let values: Vec<Vec<f64>> = crate::util::par::indices(n[0]+1,|i| (0..=n[1]).map(|j| { let p = node(i,j); h(p[0],p[1]) }).collect());
    let mut roots: Vec<Uv> = Vec::new();
    let mut root_on = |p: Uv,q: Uv,fp: f64,fq: f64| -> Option<usize> {
        if (fp < 0.) == (fq < 0.) || !fp.is_finite() || !fq.is_finite() { return None }
        let x = crate::util::roots::bracketed_root(|x: f64| Some(h(p[0]+(q[0]-p[0])*x,p[1]+(q[1]-p[1])*x)),0.,fp,1.,fq,1e-13);
        roots.push([p[0]+(q[0]-p[0])*x,p[1]+(q[1]-p[1])*x]);
        Some(roots.len()-1)
    };
    let across: Vec<Vec<Option<usize>>> = (0..n[0]).map(|i| (0..=n[1]).map(|j| root_on(node(i,j),node(i+1,j),values[i][j],values[i+1][j])).collect()).collect();
    let up: Vec<Vec<Option<usize>>> = (0..=n[0]).map(|i| (0..n[1]).map(|j| root_on(node(i,j),node(i,j+1),values[i][j],values[i][j+1])).collect()).collect();
    // whether a root `p` stands beside corner `c` by no more than `h`'s rounding there puts it off:
    // its value over its slope toward the root, four times over
    let rounding = |c: Uv,p: Uv| {
        let (du,dv) = (p[0]-c[0],p[1]-c[1]);
        let apart = du.hypot(dv);
        if !(apart > 0.) { return true }
        let d = 1e-6;
        let slope = (h(c[0]+du/apart*d,c[1]+dv/apart*d)-h(c[0],c[1]))/d;
        apart <= 4.*(h(c[0],c[1])/slope).abs()
    };
    let mut links: Vec<[usize;2]> = Vec::new();
    for i in 0..n[0] { for j in 0..n[1] {
        let sides = [across[i][j],up[i+1][j],across[i][j+1],up[i][j]];
        // **a cell at one of `corners`**, the branch leaving the corner through it: `h` nought there
        // but for its rounding, its zero line clips the corner or passes beside it, a root of rounding
        // on one or both of the corner's sides besides the root the branch leaves the cell by — which
        // no pairing joins rightly (three), or joins as a step along the rim from where the branch
        // leaves, and then onto the corner, the branch arriving tangent to the rim. The corner is the
        // root they stand for, joined to the one left
        let at = [node(i,j),node(i+1,j),node(i+1,j+1),node(i,j+1)];
        if let Some(k) = (0..4).find(|&k| corners.iter().any(|c| (c[0]-at[k][0]).abs() <= 1e-12 && (c[1]-at[k][1]).abs() <= 1e-12)) {
            // the sides at corner `k` (bottom 0, right 1, top 2, left 3): the side before it and the side after
            let beside = [(k+3)%4,k];
            let noise = |x: usize| beside.contains(&x) && sides[x].is_some_and(|r| rounding(at[k],roots[r]));
            let left: Vec<usize> = (0..4).filter(|&x| !noise(x)).filter_map(|x| sides[x]).collect();
            if (0..4).any(noise) && left.len() == 1 {
                roots.push(at[k]);
                links.push([roots.len()-1,left[0]]);
                continue
            }
        }
        let found: Vec<usize> = sides.iter().flatten().copied().collect();
        match found.len() {
            2 => links.push([found[0],found[1]]),
            4 => {
                // a saddle of `h`: which corners its middle's value joins
                let [b,r,t,l] = sides.map(Option::unwrap);
                let mid = node(i,j);
                let c = h(mid[0]+0.5/n[0] as f64,mid[1]+0.5/n[1] as f64);
                if (c < 0.) == (values[i][j] < 0.) { links.push([b,r]); links.push([t,l]) } else { links.push([b,l]); links.push([t,r]) }
            }
            _ => {}
        }
    } }
    // the polylines: from each end out, then the loops left
    let mut next: Vec<Vec<usize>> = vec![Vec::new();roots.len()];
    for (k,&[x,y]) in links.iter().enumerate() { next[x].push(k); next[y].push(k); }
    let mut used = vec![false;links.len()];
    let walk = |start: usize,first: usize,used: &mut Vec<bool>| -> Vec<usize> {
        let mut line = vec![start];
        let (mut here,mut via) = (start,first);
        loop {
            used[via] = true;
            let [x,y] = links[via];
            here = if x == here { y } else { x };
            line.push(here);
            let Some(&k) = next[here].iter().find(|&&k| !used[k]) else { break };
            via = k;
        }
        line
    };
    let mut lines: Vec<Vec<usize>> = Vec::new();
    for r in 0..roots.len() {
        if next[r].len() != 1 { continue }
        if let Some(&k) = next[r].iter().find(|&&k| !used[k]) { lines.push(walk(r,k,&mut used)) }
    }
    for k in 0..links.len() { if !used[k] { lines.push(walk(links[k][0],k,&mut used)) } }
    // each fitted: a point between two of its own settled across it, by Newton on `h`'s gradient —
    // or `F`'s away from the rim, the same zero set and sign at a sixth of the cost
    let d = 1e-7;
    // (`h` only within a step of the rim, where `F`'s double root leaves Newton nothing to settle on:
    // beyond, `h`'s zero set is where `F` is the rim's own value and rate carried out — the
    // section's fit — not nought, which grows with the distance from the rim)
    let near_rim = |p: Uv| touch.is_some_and(|(e,_)| (p[1]-e).abs() < STEP);
    let newton = |k: &dyn Fn(f64,f64) -> f64,mut p: Uv,steps: usize| -> Uv {
        for _ in 0..steps {
            let v = k(p[0],p[1]);
            if v == 0. { break }
            let g = [(k(p[0]+d,p[1])-k(p[0]-d,p[1]))/(2.*d),(k(p[0],p[1]+d)-k(p[0],p[1]-d))/(2.*d)];
            let gg = g[0]*g[0]+g[1]*g[1];
            if !(gg > 0.) || !v.is_finite() { break }
            let step = [v*g[0]/gg,v*g[1]/gg];
            p = [(p[0]-step[0]).clamp(0.,1.),(p[1]-step[1]).clamp(0.,1.)];
            if step[0].abs() <= 1e-13 && step[1].abs() <= 1e-13 { break }
        }
        p
    };
    // settled on `h`, smooth wherever the two meet; then, off the rim, onto `F` itself from there,
    // where Newton there settles within a cell of the grid (near a fold of the contact, where the
    // branch leaves the rim and `F = (v − e)² h` is small in both, Newton on `F` alone wandered off
    // the branch; and the two differ by the section's fit over the angle the sheet meets the tool at,
    // shallow near the rim — 6e-5 along the sheet for 3e-6 off the tool, at 1e-7)
    let cell = [1./n[0] as f64,1./n[1] as f64];
    let onto = |p: Uv| -> Uv {
        let p = newton(&h,p,30);
        if touch.is_none() || near_rim(p) { return p }
        let q = newton(&k,p,20);
        let (kp,kq) = (k(p[0],p[1]).abs(),k(q[0],q[1]).abs());
        if kq <= kp && (q[0]-p[0]).abs() <= cell[0] && (q[1]-p[1]).abs() <= cell[1] { q } else { p }
    };
    let between = |x: Uv,y: Uv| { let p = onto([0.5*(x[0]+y[0]),0.5*(x[1]+y[1])]); (p,at(p[0],p[1])) };
    // an end beside a corner by what `h`'s rounding there puts it off: its value over its slope toward
    // the end, four times over
    let snapped = |p: Uv| corners.iter().copied().find(|&c| rounding(c,p)).unwrap_or(p);
    lines.into_iter().filter(|l| l.len() >= 2).filter_map(|l| {
        // each root off the rim settled onto `F` itself (the march's roots are `h`'s)
        let settle = |p: Uv| -> Uv {
            if near_rim(p) { return p }
            let rail = [p[0] == 0. || p[0] == 1.,p[1] == 0. || p[1] == 1.];
            // a root on the parameters' boundary stays on it: the root of `F` along that side, beside it
            if rail[0] || rail[1] {
                let i = if rail[0] { 1 } else { 0 };
                let along = |x: f64| { let mut q = p; q[i] = x; k(q[0],q[1]) };
                let w = 1./n[i] as f64;
                let (a,b) = ((p[i]-w).max(0.),(p[i]+w).min(1.));
                let (fa,fb) = (along(a),along(b));
                if (fa < 0.) == (fb < 0.) { return p }
                let x = crate::util::roots::bracketed_root(|x: f64| Some(along(x)),a,fa,b,fb,1e-15);
                let mut q = p;
                q[i] = x;
                return q
            }
            onto(p)
        };
        let mut uv: Vec<Uv> = l.iter().map(|&k| settle(roots[k])).collect();
        let last = uv.len()-1;
        if next[l[0]].len() == 1 { uv[0] = snapped(uv[0]) }
        if next[l[last]].len() == 1 { uv[last] = snapped(uv[last]) }
        uv.dedup();
        if uv.len() < 2 { return None }
        // seventeen of its points spread along it, its ends among them, refined where the fit asks
        let pts: Vec<V> = uv.iter().map(|&p| at(p[0],p[1])).collect();
        let mut run = vec![0.];
        for w in pts.windows(2) { let last = *run.last().unwrap(); run.push(last+distance(w[0],w[1])); }
        let total = *run.last().unwrap();
        let mut idx: Vec<usize> = (0..=16).map(|k| run.partition_point(|&x| x < total*k as f64/16.).min(pts.len()-1)).collect();
        idx[0] = 0;
        idx[16] = pts.len()-1;
        idx.dedup();
        fitted(idx.iter().map(|&i| (uv[i],pts[i])).collect(),&between,tol)
    }).collect()
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
    let surface = crate::geom::Surface::Blend(Frame::about(mid,axis),blend.clone());
    let face = Face {surface,reversed:false,loops:vec![],name:"contact".into()};
    if b.loop_area(&face,&l) < 0. { l = crate::topo::reverse_loop(l); }
    b.faces.push(Face {loops:vec![l],..face});
    let rails = [Curve::Iso(blend.clone(),0.),Curve::Iso(blend.clone(),1.)];
    Ok(Carried {sheet:b,rims,rails})
}

/// A piece's sheet carried over the roll (`carried`), and its rims: the piece at the roll's start
/// and at its end, where the sheet meets the tool placed there.
pub struct Carried { pub sheet: Brep,pub rims: [crate::geom::Curve;2],
    /// The paths of the curve's ends over the roll (a closed curve's two the same).
    pub rails: [crate::geom::Curve;2] }

/// **A piece cut into stretches each a candidate of its own** (`candidates`): a sheet is kept or
/// dropped whole by the trim, which meets a candidate only with others, so a piece whose sheet both
/// bounds the sweep and lies buried in it — one closed contact through a torus's outer and inner
/// branches, carried more than a turn — is cut so its stretches meet one another where the buried
/// part crosses the rest. At least four stretches to a closed piece, none longer than a quarter of
/// the tool; a stretch shares its ends with its neighbours exactly.
///
/// Each cut's rail is the path of one point, and every sheet is a union of such paths: two paths of
/// one carry are one or apart, so a rail meets another stretch's sheet nowhere or lies in it, and a
/// rail a hair from another stretch's paths runs beside that sheet its whole length, which no search
/// settles. So each cut is moved, within a quarter of its stretch, to the nearest point whose path
/// stands clear, in the carry's invariants (`invariants`), of every point of the piece not beside it
/// along the piece — by a thousandth of the tool, or as far as any does. (On the leaning torus under
/// its screw, a cut at an even length stood 7e-6 from another stretch's paths.)
fn stretches(piece: &Piece,size: f64,carry: &crate::trajectory::Screw) -> Vec<Piece> {
    let pts = &piece.points;
    let mut at = vec![0.];
    for w in pts.windows(2) { let last = *at.last().unwrap(); at.push(last+distance(w[0],w[1])); }
    let length = *at.last().unwrap();
    let n = ((length/(0.25*size)).ceil() as usize).max(if piece.closed { 4 } else { 1 }).min(pts.len()-1);
    if n <= 1 { return vec![piece.clone()] }
    let inv: Vec<[f64;2]> = pts.iter().map(|&p| invariants(carry,p)).collect();
    let pitch = (!carry.is_slide() && !carry.is_turn()).then(|| (carry.advance/carry.ratio).abs());
    let gap = |x: [f64;2],y: [f64;2]| {
        let mut d = x[1]-y[1];
        if let Some(p) = pitch { d = (d+0.5*p).rem_euclid(p)-0.5*p }
        ((x[0]-y[0]).powi(2)+d*d).sqrt()
    };
    // how clear point `c`'s path stands of the piece's points not within a twentieth of it along it
    let along = |x: usize,y: usize| { let d = (at[x]-at[y]).abs(); if piece.closed { d.min(length-d) } else { d } };
    let clear = |c: usize| (0..pts.len()).step_by(4).filter(|&j| along(j,c) > 0.05*length).map(|j| gap(inv[c],inv[j])).fold(f64::INFINITY,f64::min);
    let margin = 1e-3*size;
    let mut cuts: Vec<usize> = (0..=n).map(|k| at.partition_point(|&x| x < length*k as f64/n as f64).min(pts.len()-1)).collect();
    let (first,last) = (0,cuts.len()-1);
    for k in 0..cuts.len() {
        // an open piece's ends are its own
        if !piece.closed && (k == first || k == last) { continue }
        if piece.closed && k == last { cuts[k] = cuts[first]; continue }
        let c0 = cuts[k];
        let reach = 0.25*length/n as f64;
        let near: Vec<usize> = (0..pts.len()).filter(|&j| along(j,c0) <= reach && (piece.closed || j > 0 && j+1 < pts.len())).collect();
        let mut by: Vec<(f64,usize)> = near.iter().map(|&j| (along(j,c0),j)).collect();
        by.sort_by(|x,y| x.0.total_cmp(&y.0));
        let mut best = (clear(c0),c0);
        if best.0 < margin {
            for &(_,j) in &by {
                let g = clear(j);
                if g >= margin { best = (g,j); break }
                if g > best.0 { best = (g,j) }
            }
        }
        cuts[k] = best.1;
    }
    // a closed piece is read from its first cut round to it again
    let order: Vec<usize> = if piece.closed {
        let start = cuts[first];
        (0..pts.len()).map(|i| (start+i)%(pts.len()-1)).chain(std::iter::once(start)).collect()
    } else { (0..pts.len()).collect() };
    let place = |c: usize| order.iter().position(|&i| i == c).unwrap_or(0);
    let mut marks: Vec<usize> = cuts.iter().map(|&c| place(c)).collect();
    if piece.closed { *marks.last_mut().unwrap() = order.len()-1 }
    marks.sort();
    marks.dedup();
    marks.windows(2).filter(|w| w[1] > w[0]).enumerate().map(|(k,w)| {
        let idx = &order[w[0]..=w[1]];
        Piece {face:piece.face,uv:idx.iter().map(|&i| piece.uv[i]).collect(),points:idx.iter().map(|&i| pts[i]).collect(),
            ends:[if k == 0 && !piece.closed { piece.ends[0] } else { End::Boundary },if w[1] == order.len()-1 && !piece.closed { piece.ends[1] } else { End::Boundary }],closed:false}
    }).collect()
}

/// A point's invariants under `carry`: a slide's shadow on the plane square to it, a turn's radius
/// and height about its axis, a screw's radius and its height along the helix (`z − c φ`, read modulo
/// a turn's rise by the caller) — what the carry keeps of a point, so two points with the same are on
/// one path.
fn invariants(carry: &crate::trajectory::Screw,p: V) -> [f64;2] {
    let f = crate::geom::Frame::about(carry.origin,carry.axis);
    let l = f.local(p);
    if carry.is_slide() { return [l[0],l[1]] }
    let rho = (l[0]*l[0]+l[1]*l[1]).sqrt();
    if carry.is_turn() { return [rho,l[2]] }
    let lead = carry.advance/(std::f64::consts::TAU*carry.ratio);
    [rho,l[2]-lead*l[1].atan2(l[0])]
}

/// **Where two sections of one carry share a path** (#880 Q1b): the sheets two curves sweep under
/// one carry meet only along whole paths of it, a point's path lying in both where each curve has a
/// point on it — where the two curves cross in the carry's invariants (`invariants`: a screw's
/// radius and height along the helix, the height modulo a turn's rise), or touch. Each such meeting's
/// parameter on `a` and on `b` over `[0, 1]`: sought from every two chords of the curves' images,
/// sampled at `count` points each (a screw's unwrapped along each and shifted by whole rises), that
/// cross or come within `reach` of one another, and closed on by damped Gauss–Newton on the
/// invariants' difference — which closes on a crossing at Newton's rate and on a touch, a double
/// root where Newton alone wanders along the valley, at least linearly. Kept where the two come
/// within `tol`, and once: two whose points on both curves stand within `tol` are one (#880 Q1b:
/// the leaning torus's stretches touch in the invariants, their sheets along a path, and Newton
/// alone settled on five points of that valley, five edges a hair apart).
fn shared_paths(carry: &crate::trajectory::Screw,a: &crate::geom::Curve,b: &crate::geom::Curve,count: [usize;2],reach: f64,tol: f64)
    -> Vec<[f64;2]> {
    let rise = (!carry.is_slide() && !carry.is_turn()).then(|| (carry.advance/carry.ratio).abs());
    let read = |c: &crate::geom::Curve,n: usize| -> Vec<[f64;2]> {
        let mut out: Vec<[f64;2]> = Vec::with_capacity(n+1);
        for k in 0..=n {
            let mut x = invariants(carry,c.point(k as f64/n as f64));
            if let (Some(r),Some(last)) = (rise,out.last()) { x[1] += r*((last[1]-x[1])/r).round(); }
            out.push(x);
        }
        out
    };
    let (pa,pb) = (read(a,count[0]),read(b,count[1]));
    let range = |p: &[[f64;2]]| p.iter().fold([f64::INFINITY,f64::NEG_INFINITY],|r,x| [r[0].min(x[1]),r[1].max(x[1])]);
    let shifts: Vec<f64> = match rise {
        None => vec![0.],
        Some(r) => {
            let (ra,rb) = (range(&pa),range(&pb));
            (((ra[0]-rb[1]-reach)/r).floor() as i64..=((ra[1]-rb[0]+reach)/r).ceil() as i64).map(|m| m as f64*r).collect()
        }
    };
    // the two sections' invariants apart, the height's difference taken modulo a rise
    let apart = |u: f64,s: f64| -> [f64;2] {
        let (x,y) = (invariants(carry,a.point(u)),invariants(carry,b.point(s)));
        let mut d = [x[0]-y[0],x[1]-y[1]];
        if let Some(r) = rise { d[1] -= r*(d[1]/r).round(); }
        d
    };
    let size = |f: [f64;2]| f[0].hypot(f[1]);
    let close = |mut u: f64,mut s: f64| -> Option<[f64;2]> {
        let h = 1e-7;
        let mut f = apart(u,s);
        let mut damp = 0.;
        for _ in 0..200 {
            if size(f) <= 1e-15*(1.+reach) { break }
            let (u0,u1,s0,s1) = ((u-h).max(0.),(u+h).min(1.),(s-h).max(0.),(s+h).min(1.));
            let (fu0,fu1,fs0,fs1) = (apart(u0,s),apart(u1,s),apart(u,s0),apart(u,s1));
            let ju = [(fu1[0]-fu0[0])/(u1-u0),(fu1[1]-fu0[1])/(u1-u0)];
            let js = [(fs1[0]-fs0[0])/(s1-s0),(fs1[1]-fs0[1])/(s1-s0)];
            // (JᵀJ + λ diag JᵀJ) δ = Jᵀ f
            let (auu,aus,ass) = (ju[0]*ju[0]+ju[1]*ju[1],ju[0]*js[0]+ju[1]*js[1],js[0]*js[0]+js[1]*js[1]);
            let (gu,gs) = (ju[0]*f[0]+ju[1]*f[1],js[0]*f[0]+js[1]*f[1]);
            let mut moved = false;
            for _ in 0..30 {
                let (puu,pss) = (auu*(1.+damp)+1e-300,ass*(1.+damp)+1e-300);
                let det = puu*pss-aus*aus;
                if !(det > 0.) { damp = damp.max(1e-12)*10.; continue }
                let (du,ds) = ((gu*pss-gs*aus)/det,(gs*puu-gu*aus)/det);
                let (nu,ns) = ((u-du).clamp(0.,1.),(s-ds).clamp(0.,1.));
                let nf = apart(nu,ns);
                if size(nf) < size(f) {
                    let still = (nu-u).abs() <= 1e-15 && (ns-s).abs() <= 1e-15;
                    (u,s,f) = (nu,ns,nf);
                    damp *= 0.1;
                    moved = !still;
                    break
                }
                damp = damp.max(1e-12)*10.;
            }
            if !moved { break }
        }
        (size(f) <= tol).then_some([u,s])
    };
    // the least distance between two chords, and where along each it is
    let nearest = |a0: [f64;2],a1: [f64;2],b0: [f64;2],b1: [f64;2]| -> (f64,f64,f64) {
        let (da,db) = ([a1[0]-a0[0],a1[1]-a0[1]],[b1[0]-b0[0],b1[1]-b0[1]]);
        let w = [b0[0]-a0[0],b0[1]-a0[1]];
        let den = da[0]*db[1]-da[1]*db[0];
        if den.abs() > 0. {
            let (x,y) = ((w[0]*db[1]-w[1]*db[0])/den,(w[0]*da[1]-w[1]*da[0])/den);
            if (0. ..=1.).contains(&x) && (0. ..=1.).contains(&y) { return (0.,x,y) }
        }
        let on = |p: [f64;2],q0: [f64;2],d: [f64;2]| { let dd = d[0]*d[0]+d[1]*d[1]; if dd > 0. { (((p[0]-q0[0])*d[0]+(p[1]-q0[1])*d[1])/dd).clamp(0.,1.) } else { 0. } };
        let gap = |x: f64,y: f64| ((a0[0]+da[0]*x)-(b0[0]+db[0]*y)).hypot((a0[1]+da[1]*x)-(b0[1]+db[1]*y));
        [(0.,on(a0,b0,db)),(1.,on(a1,b0,db)),(on(b0,a0,da),0.),(on(b1,a0,da),1.)].into_iter()
            .map(|(x,y)| (gap(x,y),x,y)).min_by(|p,q| p.0.total_cmp(&q.0)).unwrap()
    };
    let mut found: Vec<[f64;2]> = Vec::new();
    let (na,nb) = (pa.len()-1,pb.len()-1);
    for &shift in &shifts {
        for i in 0..na {
            let (a0,a1) = (pa[i],pa[i+1]);
            let (alo,ahi) = ([a0[0].min(a1[0])-reach,a0[1].min(a1[1])-reach],[a0[0].max(a1[0])+reach,a0[1].max(a1[1])+reach]);
            for j in 0..nb {
                let (b0,b1) = ([pb[j][0],pb[j][1]+shift],[pb[j+1][0],pb[j+1][1]+shift]);
                if b0[0].max(b1[0]) < alo[0] || b0[0].min(b1[0]) > ahi[0] || b0[1].max(b1[1]) < alo[1] || b0[1].min(b1[1]) > ahi[1] { continue }
                let (gap,x,y) = nearest(a0,a1,b0,b1);
                if gap > reach { continue }
                let Some(us) = close((i as f64+x)/na as f64,(j as f64+y)/nb as f64) else { continue };
                if !found.iter().any(|f| (f[0]-us[0]).abs() <= 1e-9 && (f[1]-us[1]).abs() <= 1e-9) { found.push(us) }
            }
        }
    }
    // **a valley is one meeting**: points closed on whose gap stays within `tol` all the way between
    // them (sampled along the straight between their parameters) are of one touch, or of crossings
    // nearer than the tolerance; placed where the gap is least, by golden section along `a` of the
    // least over `b` — the gap at a touch is a double root, and where a section turns a corner there
    // (two branches of a tool's contact crossing at a double point, the stretches each taking a
    // corner through it) a kinked one
    found.sort_by(|x,y| x[0].total_cmp(&y[0]));
    let golden = |f: &dyn Fn(f64) -> f64,mut lo: f64,mut hi: f64| -> f64 {
        for _ in 0..80 {
            let (m1,m2) = (lo+(hi-lo)*0.381966,lo+(hi-lo)*0.618034);
            if f(m1) <= f(m2) { hi = m2 } else { lo = m1 }
        }
        0.5*(lo+hi)
    };
    // along the valley's floor between two points: at each `u` between, the least gap over `s` about
    // theirs (the floor bends where a foot turns a corner, and near two images leaving one point
    // tangent, as every two branches of a double point do)
    let joined = |p: [f64;2],q: [f64;2]| {
        let w = (p[1]-q[1]).abs()+1e-6;
        let (slo,shi) = ((p[1].min(q[1])-w).max(0.),(p[1].max(q[1])+w).min(1.));
        (1..8).all(|k| {
            let u = p[0]+(q[0]-p[0])*k as f64/8.;
            let s = golden(&|s| size(apart(u,s)),slo,shi);
            size(apart(u,s)) <= tol
        })
    };
    let mut valleys: Vec<Vec<[f64;2]>> = Vec::new();
    for p in found {
        match valleys.iter_mut().find(|v| v.iter().any(|&q| joined(q,p))) { Some(v) => v.push(p),None => valleys.push(vec![p]) }
    }
    // a valley holding an end of both sections where they meet exactly is placed there: two stretches
    // ending on one path — the four of a double point, every two branches tangent there in the
    // invariants — part so slowly that the sections' own fit crosses them again beside it (on the
    // leaning torus, 7e-3 along, which laid as a path was a slit in the sheet)
    let end = |x: f64| x <= 1e-9 || x >= 1.-1e-9;
    let place = |v: &[[f64;2]]| -> [f64;2] {
        if let Some(p) = v.iter().find(|p| end(p[0]) && end(p[1]) && size(apart(p[0],p[1])) <= 1e-12*(1.+reach)) { return *p }
        if v.len() == 1 { return v[0] }
        let bound = |k: usize| v.iter().fold([f64::INFINITY,f64::NEG_INFINITY],|r,p| [r[0].min(p[k]),r[1].max(p[k])]);
        let (ru,rs) = (bound(0),bound(1));
        let (ws,wu) = (rs[1]-rs[0],ru[1]-ru[0]);
        let (slo,shi) = ((rs[0]-ws).max(0.),(rs[1]+ws).min(1.));
        let foot = |u: f64| golden(&|s| size(apart(u,s)),slo,shi);
        let u = golden(&|u| size(apart(u,foot(u))),(ru[0]-0.25*wu).max(0.),(ru[1]+0.25*wu).min(1.));
        [u,foot(u)]
    };
    // two valleys whose least points join are one (the straight between two points of a valley
    // can leave it where the foot along `b` turns a corner), placed again over both
    let mut placed: Vec<(Vec<[f64;2]>,[f64;2])> = valleys.into_iter().map(|v| { let p = place(&v); (v,p) }).collect();
    'merge: loop {
        for i in 0..placed.len() { for j in i+1..placed.len() {
            if joined(placed[i].1,placed[j].1) {
                let (mut v,_) = placed.swap_remove(j);
                v.extend(placed[i].0.drain(..));
                v.push(placed[i].1);
                let p = place(&v);
                placed[i] = (v,p);
                continue 'merge
            }
        } }
        break
    }
    placed.into_iter().map(|(_,p)| p).collect()
}

/// **The stretches of a sharp edge's fan in contact under `twist`** (#880 Q1b S2): the runs of the
/// edge `e`'s parameter where its velocity is square to a normal of the fan between its two faces'
/// `[f1, f2]` — where those faces' `n · v` differ in sign — each end a root of one face's `n · v` on
/// the edge or an end of the edge; a closed edge's run through its seam is one, its end past `t1`.
pub fn fan_runs(tool: &Brep,e: usize,[f1,f2]: [usize;2],twist: &Twist) -> Vec<[f64;2]> {
    let edge = &tool.edges[e];
    let crate::topo::EdgeCurve::Curve(c) = &edge.curve else { return Vec::new() };
    let normal = |f: usize,p: V| tool.faces[f].outward(p).map_or(0.,|n| crate::util::space::dot(n,twist.velocity(p)));
    let fan = |t: f64| { let p = c.point(t); normal(f1,p)*normal(f2,p) };
    const N: usize = 512;
    let [t0,t1] = edge.t;
    let ts: Vec<f64> = (0..=N).map(|k| t0+(t1-t0)*k as f64/N as f64).collect();
    let fs: Vec<f64> = ts.iter().map(|&t| fan(t)).collect();
    // where the fan's sign turns, each root of one of the two faces' `n · v` on the edge
    // (the face whose `n · v` turns over the bracket, the bracket's root of it)
    let root = |a: f64,b: f64| {
        let side = |f: usize| move |t: f64| normal(f,c.point(t));
        let (g1,g2) = (side(f1),side(f2));
        let g: &dyn Fn(f64) -> f64 = if (g1(a) < 0.) != (g1(b) < 0.) { &g1 } else { &g2 };
        crate::util::roots::bracketed_root(|t: f64| Some(g(t)),a,g(a),b,g(b),1e-14*(1.+t1.abs()))
    };
    let mut runs: Vec<[f64;2]> = Vec::new();
    let mut start: Option<f64> = (fs[0] < 0.).then_some(t0);
    for k in 0..N {
        if (fs[k] < 0.) == (fs[k+1] < 0.) { continue }
        let x = root(ts[k],ts[k+1]);
        match start.take() { Some(a) => runs.push([a,x]),None => start = Some(x) }
    }
    if let Some(a) = start { runs.push([a,t1]) }
    // a closed edge's run through its seam is one
    if edge.v[0] == edge.v[1] && runs.len() > 1 && runs[0][0] == t0 && runs[runs.len()-1][1] == t1 {
        let last = runs.pop().unwrap();
        runs[0] = [last[0],runs[0][1]+(t1-t0)];
    }
    runs
}

/// **The candidates of the closed solid `tool` sweeps under the constant twist `path` over `roll`**,
/// short of a whole turn (#880 Q1b S4): each piece of each face's contact and each stretch of a sharp
/// edge's fan in contact (S2, `fan_runs`) carried over the roll, and the tool at the roll's two ends.
/// Refused, by name, where a face is in contact all over (`Whole`).
/// With them, the meetings known by construction (P4): each sheet meets its face at either end pose in
/// that end's rim (`crate::boolean::Given`), never searched for.
pub fn candidates(tool: &Brep,path: &crate::trajectory::Trajectory,roll: [f64;2],tol: f64)
    -> Result<(Vec<Brep>,Vec<crate::boolean::Given>),super::closed::Unbuilt> {
    use super::closed::Unbuilt;
    let screw = path.screw().ok_or(Unbuilt::Motion)?;
    // the sharp edges, each with its two faces: an edge between two faces not smooth there
    let uses = tool.uses();
    let mut sharp: Vec<(usize,usize,usize)> = Vec::new();
    for (e,edge) in tool.edges.iter().enumerate() {
        if matches!(edge.curve,crate::topo::EdgeCurve::Degenerate) { continue }
        let [(f1,..),(f2,..)] = uses[e][..] else { continue };
        if f1 == f2 { continue }
        let mid = edge.point(0.5*(edge.t[0]+edge.t[1]),&tool.vertices);
        let (Some(n1),Some(n2)) = (tool.faces[f1].outward(mid),tool.faces[f2].outward(mid)) else { return Err(Unbuilt::Curved {at:mid}) };
        if crate::util::space::dot(n1,n2) < 1.-1e-9 { sharp.push((e,f1,f2)) }
    }
    let twist = screw.twist();
    let located = Located::new(tool,tol);
    let mut out = Vec::new();
    let mut given = Vec::new();
    // every stretch's sheet, the sheet following it along its piece, the faces its section lies on and
    // the order it touches them in (a contact's two, an edge's fan's one), and how its section ends
    let mut carried_all: Vec<(Carried,Option<usize>,Vec<usize>,u32,[End;2])> = Vec::new();
    let posed = |t: f64| -> Result<crate::geom::Rigid,Unbuilt> { let q = path.pose(t)?; Ok(crate::geom::Rigid {r:q.r,t:q.p}) };
    let ends = [posed(roll[0])?,posed(roll[1])?];
    for fi in 0..tool.faces.len() {
        match face_contact(tool,&located,fi,&twist,tol,CELLS) {
            Contact::Whole => {
                let at = tool.faces[fi].loops.iter().flatten().next().map_or([0.;3],|c| tool.vertices[tool.ends(c)[0] as usize].p);
                return Err(Unbuilt::Curved {at})
            }
            Contact::Pieces(pieces) => for p in pieces {
                let parts = stretches(&p,tool.size(),&screw);
                let whole = parts.len() == 1;
                let mut sheets: Vec<Carried> = Vec::new();
                for part in &parts {
                    let curve = piece_curve(tool,part,&twist,tol).ok_or_else(|| Unbuilt::Failed("a piece of contact not fitted".into()))?;
                    sheets.push(carried(&curve,whole && p.closed,path,roll)?);
                }
                // each stretch followed by the next along the piece (a closed piece's last by its first)
                let (n,first) = (sheets.len(),carried_all.len());
                for (k,c) in sheets.into_iter().enumerate() {
                    let next = if k+1 < n { Some(first+k+1) } else if p.closed && n > 1 { Some(first) } else { None };
                    carried_all.push((c,next,vec![p.face],2,parts[k].ends));
                }
            },
        }
    }
    // **an edge's fan** (#880 Q1b S2): a sharp edge's point bounds the sweep where its velocity is
    // square to a normal of the fan between its two faces' — where those faces' `n · v` differ in sign.
    // Each such stretch of it, its ends where one face's contact reaches the edge, carried over the
    // roll: its section the edge itself, fitted through points of it, crossing each of its two faces
    // posed at an end of the roll along the rim, simply
    for &(e,f1,f2) in &sharp {
        let edge = &tool.edges[e];
        let crate::topo::EdgeCurve::Curve(c) = &edge.curve else { continue };
        let runs = fan_runs(tool,e,[f1,f2],&twist);
        let [t0,t1] = edge.t;
        let closed = edge.v[0] == edge.v[1];
        let whole = closed && runs.len() == 1 && runs[0] == [t0,t1];
        for run in runs {
            let at = |t: f64| c.point(if t > t1 { t-(t1-t0) } else { t });
            let m = 64;
            let ts: Vec<f64> = (0..=m).map(|k| run[0]+(run[1]-run[0])*k as f64/m as f64).collect();
            let piece = Piece {face:f1,uv:ts.iter().map(|&t| [t,0.]).collect(),points:ts.iter().map(|&t| at(t)).collect(),
                ends:[End::Boundary,End::Boundary],closed:whole};
            let parts = stretches(&piece,tool.size(),&screw);
            let one = parts.len() == 1;
            let (n,first) = (parts.len(),carried_all.len());
            for (k,part) in parts.iter().enumerate() {
                let between = |a: Uv,b: Uv| { let t = 0.5*(a[0]+b[0]); ([t,0.],at(t)) };
                let nodes: Vec<(Uv,V)> = part.uv.iter().zip(&part.points).map(|(&u,&p)| (u,p)).collect();
                let curve = fitted(nodes,&between,tol).ok_or_else(|| Unbuilt::Failed("an edge's fan not fitted".into()))?;
                let next = if k+1 < n { Some(first+k+1) } else if whole && n > 1 { Some(first) } else { None };
                carried_all.push((carried(&curve,one && whole,path,roll)?,next,vec![f1,f2],1,part.ends));
            }
        }
    }
    // **every two sheets given their whole meeting** (#880 Q1b): the paths of the carry both hold,
    // where their sections cross in its invariants (`shared_paths`), each the one sheet's `Iso` there;
    // and two neighbours the rail they share, met smoothly there, their crossing at it the rail
    // itself. A pair sharing no path is given none, and not searched: two sheets of one carry nearly
    // touching between two close paths (the leaning torus's stretches crossing back over one another
    // under its screw) sent a march along the one path across to the other
    let section = |c: &Carried| -> (crate::geom::Curve,usize) {
        let n = match &c.rims[0] { crate::geom::Curve::BSpline(s) => 8*s.poles.len(),_ => 256 };
        (c.rims[0].clone(),n)
    };
    let blend = |c: &Carried| match &c.sheet.faces[0].surface { Surface::Blend(_,b) => Some(b.clone()),_ => None };
    for i in 0..carried_all.len() { for j in i+1..carried_all.len() {
        let ((ci,ni,..),(cj,nj,..)) = (&carried_all[i],&carried_all[j]);
        let (Some(bi),Some(_)) = (blend(ci),blend(cj)) else { continue };
        let ((si,ki),(sj,kj)) = (section(ci),section(cj));
        let mut curves = Vec::new();
        if *ni == Some(j) { curves.push(ci.rails[1].clone()) }
        if *nj == Some(i) { curves.push(cj.rails[1].clone()) }
        let near = |x: f64,y: f64| (x-y).abs() <= 1e-6;
        for [u,s] in shared_paths(&screw,&si,&sj,[ki,kj],1e-3*tool.size(),tol) {
            // the rail two neighbours share: given already
            if (*ni == Some(j) && near(u,1.) && near(s,0.)) || (*nj == Some(i) && near(u,0.) && near(s,1.)) { continue }
            curves.push(crate::geom::Curve::Iso(bi.clone(),u));
        }
        given.push(crate::boolean::Given {surfaces:[ci.sheet.faces[0].surface.clone(),cj.sheet.faces[0].surface.clone()],curves});
    } }
    // **every sheet given its whole meeting with the tool at each end** (`pose_meeting`): the rim it
    // touches the face its section lies on along, at that end, and what of it passes back into the
    // tool there, met in its own parameters — sought, a branch arriving at the rim where the two
    // part as the square of the way was marched to a stop short of it, and at a double point of the
    // contact, where it leaves the rim's end tangent to the rim, laid beside the rim
    let poses: Vec<Brep> = ends.iter().map(|m| tool.moved(m)).collect();
    let jobs: Vec<(usize,usize,usize)> = (0..carried_all.len()).flat_map(|k| (0..2).flat_map(move |e| (0..tool.faces.len()).map(move |fi| (k,e,fi)))).collect();
    let met: Vec<Option<crate::boolean::Given>> = crate::util::par::indices(jobs.len(),|j| {
        let (k,e,fi) = jobs[j];
        let (c,_,on,order,section) = &carried_all[k];
        let b = blend(c)?;
        let sheet = &c.sheet.faces[0].surface;
        let f = &poses[e].faces[fi];
        let touch = on.contains(&fi).then_some((e as f64,*order));
        let mut curves: Vec<crate::geom::Curve> = touch.map(|_| c.rims[e].clone()).into_iter().collect();
        let corners: Vec<Uv> = if touch.is_none() { Vec::new() } else {
            [0.,1.].into_iter().zip(section).filter(|(_,end)| matches!(end,End::Singular)).map(|(u,_)| [u,e as f64]).collect()
        };
        curves.extend(pose_meeting(&b,&f.surface,touch,&corners,tol));
        Some(crate::boolean::Given {surfaces:[sheet.clone(),f.surface.clone()],curves})
    });
    given.extend(met.into_iter().flatten());
    out.extend(carried_all.into_iter().map(|(c,..)| c.sheet));
    out.extend(poses);
    Ok((out,given))
}
