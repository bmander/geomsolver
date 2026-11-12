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

/// How far a contact grid's inner lines stand off the even ones, a part of a cell: irrational, so no
/// line falls where a symmetric tool's contact does (`face_contact`).
const GRID_OFF: f64 = 0.1180339887498949;

/// **Face `fi`'s contact under `twist`** (the velocity field in the tool's own coordinates,
/// `Screw::twist`), in pieces within `tol` of the curve: read on a grid of `cells` each way at least.
pub fn face_contact(tool: &Brep,located: &Located,fi: usize,twist: &Twist,tol: f64,cells: usize) -> Contact {
    let surface = &tool.faces[fi].surface;
    let [lo0,hi0] = located.parameter_box(fi);
    let span = [hi0[0]-lo0[0],hi0[1]-lo0[1]];
    if !(span[0] > 0. && span[1] > 0.) { return Contact::Pieces(Vec::new()) }
    // (a parameter the face spans a whole period of read over a window set an irrational part of the
    // period on, so its seam stands inside the grid, not on its side: a contact along a seam — the
    // ball swung across its own centre line, in contact along its great circle through the axis, half
    // of which is the ball's seam meridian — read on the box's side was lost; whether a point lies in
    // the face read with its parameters wrapped back into the box)
    let periods = surface.periods();
    let whole: [bool;2] = std::array::from_fn(|k| periods[k].is_some_and(|p| (span[k]-p).abs() <= 1e-9*p));
    let shift: [f64;2] = std::array::from_fn(|k| if whole[k] { GRID_OFF*0.3236067977499790*span[k] } else { 0. });
    let (lo,hi) = ([lo0[0]+shift[0],lo0[1]+shift[1]],[hi0[0]+shift[0],hi0[1]+shift[1]]);
    let wrapped = |uv: Uv| -> Uv { std::array::from_fn(|k| if whole[k] { lo0[k]+(uv[k]-lo0[k]).rem_euclid(span[k]) } else { uv[k] }) };
    let g = |uv: Uv| surface.normal_velocity(uv,twist);
    // the grid: as many cells each way as its side's length in space asks, `cells` at least
    let reach = |k: usize| {
        let at = |x: f64| { let mut uv = [lo[0]+0.5*span[0],lo[1]+0.5*span[1]]; uv[k] = lo[k]+x*span[k]; surface.point(uv) };
        (0..16).map(|i| distance(at(i as f64/16.),at((i+1) as f64/16.))).sum::<f64>()
    };
    let (ru,rv) = (reach(0),reach(1));
    let n = [(ru,rv),(rv,ru)].map(|(a,b)| cells.max((cells as f64*a/b.max(1e-300)).min(4.*cells as f64) as usize));
    // (the box's edges nodes, its inner lines set an irrational part of a cell off the even ones: a
    // tool drawn symmetrically about the motion has its contact along an even line of its parameters —
    // the rod spun about a line through its middle in contact all round its middle circle, crossing
    // its top and bottom lines at a node — where `g` read exactly nought at every node of the line
    // and the walk read a crossing as two pieces of no length)
    let line = |k: usize,i: usize| -> f64 { if i == 0 { lo[k] } else if i == n[k] { hi[k] } else { lo[k]+span[k]*(i as f64+GRID_OFF)/n[k] as f64 } };
    let node = |i: usize,j: usize| -> Uv { [line(0,i),line(1,j)] };
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
    let cell = [(1.+GRID_OFF)*span[0]/n[0] as f64,(1.+GRID_OFF)*span[1]/n[1] as f64];
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
                let mid = { let (a,b) = (node(i,j),node(i+1,j+1)); [0.5*(a[0]+b[0]),0.5*(a[1]+b[1])] };
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
    let holds = |uv: Uv| located.holds_parameters(fi,wrapped(uv));
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
    // **a piece beside a pole runs on to it** (#880 Q1b S5): where a side of the parameters is one
    // point in space — a cone's apex — the normal is not defined, `g` is read on no node of that side,
    // and a branch reaching the pole ended a cell short of it (the cone spun about a line through
    // it, whose generators in the plane through the turn's axis are in contact to the apex); such an
    // end, within two cells of the side, is carried on to it along the piece's last chord
    let poled: Vec<(usize,f64)> = (0..2).filter(|&k| !whole[k]).flat_map(|k| [(k,lo0[k]),(k,hi0[k])]).filter(|&(k,at)| {
        let side = |x: f64| { let mut uv = [0.;2]; uv[k] = at; uv[1-k] = lo0[1-k]+x*span[1-k]; surface.point(uv) };
        let p0 = side(0.);
        (1..=8).all(|i| distance(side(i as f64/8.),p0) <= tol)
    }).collect();
    for p in &mut pieces {
        for end in 0..2 {
            if p.uv.len() < 2 || !matches!(p.ends[end],End::Singular) { continue }
            let m = p.uv.len();
            let (last,before) = if end == 0 { (p.uv[0],p.uv[1]) } else { (p.uv[m-1],p.uv[m-2]) };
            let Some(&(k,at)) = poled.iter().find(|&&(k,at)| (last[k]-at).abs() <= 2.*cell[k] && (at-last[k])*(last[k]-before[k]) >= 0.) else { continue };
            if !((last[k]-before[k]).abs() > 0.) { continue }
            let x = (at-last[k])/(last[k]-before[k]);
            let mut q = [last[0]+x*(last[0]-before[0]),last[1]+x*(last[1]-before[1])];
            q[k] = at;
            if end == 0 { p.uv.insert(0,q) } else { p.uv.push(q) }
        }
    }
    // pieces ending at one point either side of a seam, joined
    let ends_of = |p: &Piece| [surface.point(p.uv[0]),surface.point(*p.uv.last().unwrap())];
    // (not at a pole, where the pieces reaching it meet but are not joined: the cone's apex)
    let at_pole = |x: Uv| poled.iter().any(|&(k,at)| (x[k]-at).abs() <= 1e-9*span[k]);
    let seam = |x: Uv,y: Uv| on_box(x) && on_box(y) && !at_pole(x) && !at_pole(y) && ((x[0]-y[0]).abs()/span[0]).max((x[1]-y[1]).abs()/span[1]) > 1e-6;
    let near = |x: V,y: V| distance(x,y) <= tol;
    loop {
        let ends: Vec<[V;2]> = pieces.iter().map(ends_of).collect();
        // an end of a onto an end of b: a turned to end there, b to start there
        let found = (0..pieces.len()).filter(|&a| !pieces[a].closed).find_map(|a| (0..pieces.len()).filter(|&b| b != a && !pieces[b].closed).find_map(|b| {
            let (ua,ub) = ([pieces[a].uv[0],*pieces[a].uv.last().unwrap()],[pieces[b].uv[0],*pieces[b].uv.last().unwrap()]);
            (0..2).flat_map(|x| (0..2).map(move |y| (x,y))).find(|&(x,y)| near(ends[a][x],ends[b][y]) && seam(ua[x],ub[y])).map(|(x,y)| (a,b,x == 0,y == 1))
        }));
        let Some((a,b,turn_a,turn_b)) = found else { break };
        let mut tail = pieces.remove(b);
        if turn_b { tail.uv.reverse(); tail.ends.reverse(); }
        let head = &mut pieces[if b < a { a-1 } else { a }];
        if turn_a { head.uv.reverse(); head.ends.reverse(); }
        head.uv.extend(tail.uv);
        head.ends[1] = tail.ends[1];
    }
    for p in &mut pieces {
        if !p.closed && p.uv.len() > 2 && on_box(p.uv[0]) { let e = ends_of(p); if near(e[0],e[1]) { p.closed = true; } }
    }
    // **a piece ending on the face's boundary ends at `g`'s root along that edge** (#880 Q1b S6): its
    // end placed by halving across the boundary in the face's own parameters stood off the root a
    // face on the edge's other side places its own piece's end at, where the contact passes smoothly
    // from one face to the next — the coil spring's tube into its ball ends, 4.3e-5 apart at 1e-5,
    // two sheets' rails a hair apart along one path. Read along the edge itself, `g` with this face's
    // normal, the two faces' roots are one where they are tangent, and each its own across a crease
    let edges_of: Vec<(u32,&crate::geom::Curve)> = tool.faces[fi].loops.iter().flatten().filter_map(|co| match &tool.edges[co.edge as usize].curve {
        crate::topo::EdgeCurve::Curve(c) => Some((co.edge,c)),_ => None }).collect();
    let (periods,size) = (surface.periods(),tool.size());
    for p in pieces.iter_mut().filter(|p| !p.closed) {
        for end in 0..2 {
            if !matches!(p.ends[end],End::Boundary) { continue }
            let k = if end == 0 { 0 } else { p.uv.len()-1 };
            let (uv,x) = (p.uv[k],surface.point(p.uv[k]));
            let Some((e,c,t0,_)) = edges_of.iter().map(|&(e,c)| { let (t,d) = located.nearest(e,x); (e,c,t,d) }).min_by(|a,b| a.3.total_cmp(&b.3)) else { continue };
            let edge = &tool.edges[e as usize];
            let at = |t: f64| -> Uv { crate::topo::unwrap(surface.inverse_near(c.point(t),uv),uv,periods) };
            let gt = |t: f64| surface.normal_velocity(at(t),twist);
            let Some(g0) = gt(t0) else { continue };
            let width = edge.t[1]-edge.t[0];
            // bracketed out from the foot, doubling, as far as a hundredth of the edge
            let mut d = 1e-9*width;
            let found = loop {
                if d > 1e-2*width { break None }
                let (a,b) = ((t0-d).max(edge.t[0]),(t0+d).min(edge.t[1]));
                match (gt(a),gt(b)) {
                    (Some(ga),_) if (ga < 0.) != (g0 < 0.) => break Some((a,ga,t0,g0)),
                    (_,Some(gb)) if (gb < 0.) != (g0 < 0.) => break Some((t0,g0,b,gb)),
                    _ => d *= 2.,
                }
            };
            let Some((a,ga,b,gb)) = found else { continue };
            let t = crate::util::roots::bracketed_root(gt,a,ga,b,gb,1e-14*width);
            let q = at(t);
            // (within a thousandth of the tool of where halving put it, as coarsely as a ball's cap
            // placed it, 4.2e-5 off at the kernel's tolerance)
            if distance(surface.point(q),x) <= 1e-3*size { p.uv[k] = q }
        }
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
    // **a contact along the face's own edge is the edge's** (#880 Q1b S5, `fan_runs`): where the face's
    // `g` vanishes along an edge it bounds, the edge is in contact, its fan holding the face's normal,
    // and is read there; read here, along the side of the grid, `g`'s nought on the side took the sign
    // of the cells within, and half of it was found (the coin turned about a line through its rim,
    // whose side is in contact all round its rim on the plane through the axis) — but not along its
    // seam, which bounds it on both sides and has no fan (the cone spun about a line through it, whose
    // seam's generator is in contact)
    let all: Vec<u32> = tool.faces[fi].loops.iter().flatten().map(|c| c.edge).collect();
    let edges: Vec<u32> = all.iter().copied().filter(|&e| all.iter().filter(|&&x| x == e).count() == 1).collect();
    let along = |x: V| edges.iter().any(|&e| !matches!(tool.edges[e as usize].curve,crate::topo::EdgeCurve::Degenerate) && located.nearest(e,x).1 <= tol);
    split.retain(|p| { let m = p.points.len(); !(0..=4).all(|k| along(p.points[(k*(m-1))/4])) });
    // (each point's parameters back in the face's box, as the face reads them)
    for p in &mut split { for uv in &mut p.uv { *uv = wrapped(*uv) } }
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
pub fn piece_curve(tool: &Brep,located: &Located,piece: &Piece,twist: &Twist,tol: f64) -> Option<crate::geom::Curve> {
    let surface = &tool.faces[piece.face].surface;
    // **a face closed by its own seam is unwrapped across it** (#880 Q1b S6): a parameter the
    // surface gives no period whose box's two sides are one curve in space — an extrusion of a closed
    // spline, whose seam is a side of its parameters — is read as one a period the box's width, so a
    // piece joined across that seam (`face_contact`) is unwrapped along it, and a midpoint between two
    // of its nodes either side stands beside them, not across the bar: settled there, the spline
    // bar's inner contact was fitted through a cubic 61 long for its 17, 1.4 off the face
    let [lo0,hi0] = located.parameter_box(piece.face);
    let native = surface.periods();
    let periods: [Option<f64>;2] = std::array::from_fn(|k| native[k].or_else(|| {
        let side = |at: f64,x: f64| { let mut uv = [0.;2]; uv[k] = at; uv[1-k] = lo0[1-k]+x*(hi0[1-k]-lo0[1-k]); surface.point(uv) };
        let width = hi0[k]-lo0[k];
        (width > 0. && (0..=8).all(|i| distance(side(lo0[k],i as f64/8.),side(hi0[k],i as f64/8.)) <= tol)).then_some(width)
    }));
    let wrap = |uv: Uv| -> Uv { std::array::from_fn(|k| match (native[k],periods[k]) {
        (None,Some(p)) => crate::geom::around(uv[k],lo0[k],p),_ => uv[k] }) };
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
    // (the point between settled where the face reads it, and kept unwrapped beside the two)
    let between = |a: Uv,b: Uv| {
        let mid = [0.5*(a[0]+b[0]),0.5*(a[1]+b[1])];
        let m = settled_on(surface,twist,wrap(mid),h,None,0.);
        (crate::topo::unwrap(m,mid,periods),surface.point(m))
    };
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
fn pose_meeting(blend: &crate::geom::Blend,surface: &Surface,within: ([f64;3],[f64;3]),touch: Option<(f64,u32)>,turn: Option<f64>,corners: &[Uv],
    tol: f64) -> Vec<crate::geom::Curve> {
    let a = &blend.a;
    let at = |u: f64,v: f64| blend.carry.at(a.point(u),v);
    let f = |u: f64,v: f64| surface.implicit(at(u,v));
    // **a corner of the sheet on the face** (#880 Q1b S5): where an end of the section, at either end
    // of the roll, lies on the face — the section ending on an edge of the tool the face bounds — the
    // meeting has a corner there as at a double point, which a branch may reach along the sheet's side
    // with `F` touching nought on it (the rod spun about a line through it: its end disc's line,
    // carried, meets the rod's side at the start pose in a half ellipse tangent to the sheet's side at
    // the rim's top, which the march left short of it, 6e-3 off, on the side). The rim a face touches
    // is the sheet's own edge, its corners `h`'s, not these
    let corners: Vec<Uv> = corners.iter().copied().chain([[0.,0.],[1.,0.],[0.,1.],[1.,1.]].into_iter()
        .filter(|c| touch.is_none_or(|(e,_)| c[1] != e) && f(c[0],c[1]).abs() <= tol)).collect();
    let corners = &corners[..];
    // **`h`, `F` over its root along the rim**: `F = (v − e) h` where the section crosses the face
    // simply (an edge's fan on either of its faces), `F = (v − e)² h` on the face it lies on — `F` less
    // its value and rate at the rim, `c₀` and `c₁`, over `w`'s power, the two read exactly (`F` there,
    // and its gradient along the carry's own velocity, `Carry::rate`); and within a hundred-thousandth
    // of the roll of the rim its series, its terms by differences of that rate, passing smoothly into
    // it, so the two are one function (one jumping to the other made a sign change of its own along the
    // rail). Five-point differences of `F` a thousandth of the roll apart took the series' truncation
    // into the rate: on the leaning torus, whose carry moves a point a tenth of a millimetre in that
    // step past a tube of radius 2, 2.7e-5 for its own 1e-8, and the branch beside the rim stood 4e-8
    // off the tool. **A branch is settled on `F` less a share of `c₀ + c₁ w`** over `w`'s power: all
    // of it within a step of the rim, where that is `h`; none past four steps, where its zero set is
    // `F`'s; and between, a share falling smoothly, one zero set carried from the one into the other.
    // Settled on `h` within the step and on `F` past it, a branch had its points between two curves
    // that part at the step's edge by the rim's value and rate over `w²` and `h`'s slope, past a tenth
    // of the kernel's tolerance; the fit chased them until four thousand nodes stood in a span
    // (`fitted`: two of the leaning torus's branches, the build twice as long)
    const STEP: f64 = 1e-3;
    const SERIES: f64 = 1e-5;
    // all of a share within `lo` of the rim, none past `hi`, a smoothstep between
    let fall = |w: f64,lo: f64,hi: f64| { let x = ((w.abs()-lo)/(hi-lo)).clamp(0.,1.); 1.-x*x*(3.-2.*x) };
    let rate = |u: f64,v: f64| { let q = at(u,v); crate::util::space::dot(surface.gradient(q),blend.carry.rate(q)) };
    let rim = |u: f64| touch.map_or([0.;2],|(e,_)| [f(u,e),rate(u,e)]);
    // at `u` (the section's point there, `p`) and `v`, `F` less `share` of the rim's value and rate `c`
    // over `w`'s power, the series within its step
    let over = |u: f64,p: V,v: f64,[c0,c1]: [f64;2],share: f64| -> f64 {
        let f = surface.implicit(blend.carry.at(p,v));
        let Some((e,order)) = touch else { return f };
        let w = v-e;
        let own = || (f-share*(c0+if order == 1 { 0. } else { c1*w }))/w.powi(order as i32);
        let mu = fall(w,0.5*SERIES,SERIES);
        if mu == 0. { return own() }
        let (r1,m1) = (rate(u,e+SERIES),rate(u,e-SERIES));
        let series = if order == 1 { c1+w*(r1-m1)/(4.*SERIES) } else { (r1-m1)/(4.*SERIES)+w*(r1-2.*c1+m1)/(6.*SERIES*SERIES) };
        if mu == 1. { return series }
        mu*series+(1.-mu)*own()
    };
    let h = |u: f64,v: f64| over(u,a.point(u),v,rim(u),1.);
    let k = |u: f64,v: f64| {
        let share = touch.map_or(0.,|(e,_)| fall(v-e,STEP,4.*STEP));
        over(u,a.point(u),v,if share > 0. { rim(u) } else { [0.;2] },share)
    };
    // the grid: as many cells each way as its sides' lengths in space ask, 64 at least
    let length = |p: &dyn Fn(f64) -> V| (0..32).map(|k| distance(p(k as f64/32.),p((k+1) as f64/32.))).sum::<f64>();
    let (lu,lv) = (length(&|u| at(u,0.5)),length(&|v| at(0.5,v)));
    let n = [(lu,lv),(lv,lu)].map(|(x,y)| 64usize.max((64.*x/y.max(1e-300)).min(256.) as usize));
    let node = |i: usize,j: usize| -> Uv { [i as f64/n[0] as f64,j as f64/n[1] as f64] };
    // (read only where the face is, its box grown by two cells: a sheet screwed whole turns past a
    // plane it touches along lines — the end disc's diameter of a drum screwed through a rod, whose
    // sheet lies on the cylinder about the axis the disc's plane is tangent to — has `F` a square
    // along each, where the grid read a crumb of a branch at every rounding of its sign)
    let margin = 2.*(lu/n[0] as f64).max(lv/n[1] as f64);
    let (lo,hi) = within;
    let far = |q: V| (0..3).any(|k| q[k] < lo[k]-margin || q[k] > hi[k]+margin);
    // (nor along the rim carried whole turns on, where it lies on the face's surface again — a
    // screw's base disc, whose plane holds the screw's axis, touched by its rim's fan each turn
    // within the disc — the sheet touching it there as along the rim, `F` a square, its crumbs
    // read as branches: a touch, crossing nothing, is no meeting to cut by)
    let lifted: Vec<f64> = match (touch,turn) {
        (Some((e,_)),Some(w)) if w > 0. => (1..).map(|k| k as f64*w).take_while(|&d| d < 1.).flat_map(|d| [e-d,e+d])
            .filter(|&v| v > 0. && v < 1. && (0..=8).all(|i| f(i as f64/8.,v).abs() <= tol)).collect(),
        _ => Vec::new(),
    };
    let band = 2./n[1] as f64;
    let far = |q: V,v: f64| far(q) || lifted.iter().any(|&x| (v-x).abs() <= band);
    // (the section's point and the rim's value and rate read once a column)
    let values: Vec<Vec<f64>> = crate::util::par::indices(n[0]+1,|i| {
        let u = node(i,0)[0];
        let (p,c) = (a.point(u),rim(u));
        (0..=n[1]).map(|j| { let v = node(i,j)[1]; if far(blend.carry.at(p,v),v) { f64::NAN } else { over(u,p,v,c,1.) } }).collect()
    });
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
        let hc = h(c[0],c[1]);
        let slope = (h(c[0]+du/apart*d,c[1]+dv/apart*d)-hc)/d;
        apart <= 4.*(hc/slope).abs()
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
    // the sheet's sides that are one point: a section's end on the turn's axis, which the carry holds
    let poles: Vec<(f64,V)> = [0.,1.].into_iter().map(|u| (u,a.point(u))).filter(|&(_,q)| distance(blend.carry.at(q,1.),q) <= 1e-9*(1.+norm_of(q))).collect();
    let on_pole = |p: Uv| poles.iter().any(|&(u,q)| (p[0]-u).abs() <= 1e-9 || distance(at(p[0],p[1]),q) <= tol);
    lines.into_iter().filter(|l| l.len() >= 2).flat_map(|l| {
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
        // (split where it runs along a side of the sheet that is one point — a pole, where the section
        // meets the turn's axis and every point of that side lies on the tool — each part reaching the
        // pole: the run along the side meets nothing, and laid it was a loop of no size round the pole)
        let mut parts: Vec<Vec<Uv>> = vec![Vec::new()];
        for (k,&p) in uv.iter().enumerate() {
            let at_pole = on_pole(p);
            let part = parts.last_mut().unwrap();
            if !at_pole { part.push(p); continue }
            if !part.is_empty() { part.push(p); parts.push(Vec::new()) }
            else if uv.get(k+1).is_some_and(|&n| !on_pole(n)) { part.push(p) }
        }
        // (and a part from a pole back to it meets nothing: beside the pole the sheet stands within
        // its rounding of the tool all round, a cone's tip on the axis, and the signs of that rounding
        // drew loops round it — the ball swung across its own centre line, at its axis points)
        let at_a_pole = |p: Uv| { let x = at(p[0],p[1]); poles.iter().any(|&(_,q)| distance(x,q) <= tol) };
        parts.into_iter().filter(|p| p.len() >= 2 && !(at_a_pole(p[0]) && at_a_pole(p[p.len()-1]))).filter_map(|uv| {
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
        }).collect::<Vec<_>>()
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
pub fn carried(curve: &crate::geom::Curve,closed: bool,path: &crate::trajectory::Trajectory,roll: [f64;2],on: Option<Surface>,tol: f64)
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
    // **a pole** (#880 Q1b S5): an end of the curve on a turn's axis does not move, so its rail is
    // one point, a degenerate edge of one vertex, as a cone's apex is; read as a curve of no length,
    // the sheet's boundary had no edge along that side of its parameters, and the arrangement's walk
    // arrived at one corner of it with nothing leaving for the other
    let size = 1.+crate::util::space::norm(a.point(0.)).max(crate::util::space::norm(a.point(1.)));
    let pole = |t: f64| !closed && screw.is_turn() && distance(carry.at(a.point(t),1.),a.point(t)) <= 1e-9*size;
    let mut b = Brep::default();
    let p0 = b.vertex(a.point(0.));
    let p1 = if closed { p0 } else { b.vertex(a.point(1.)) };
    let q0 = if pole(0.) { p0 } else { b.vertex(image.point(0.)) };
    let q1 = if closed { q0 } else if pole(1.) { p1 } else { b.vertex(image.point(1.)) };
    let ea = b.edge(EdgeCurve::Curve(a.clone()),[0.,1.],[p0,p1]);
    let eb = b.edge(EdgeCurve::Curve(image.clone()),[0.,1.],[q0,q1]);
    let rail = |b: &mut Brep,t: f64,ends: [u32;2]| if pole(t) { b.edge(EdgeCurve::Degenerate,[0.,1.],ends) }
        else { b.edge(EdgeCurve::Curve(Curve::Iso(blend.clone(),t)),[0.,1.],ends) };
    let line = |x: [f64;2],y: [f64;2]| Pcurve::Line {a:x,b:y};
    let mid = a.point(0.5);
    let (surface,mut l) = match on {
        None => {
            let r0 = rail(&mut b,0.,[p0,q0]);
            let r1 = if closed { r0 } else { rail(&mut b,1.,[p1,q1]) };
            (crate::geom::Surface::Blend(Frame::about(mid,axis),blend.clone()),vec![
                Coedge {edge:ea,reversed:false,pcurve:line([0.,0.],[1.,0.])},
                Coedge {edge:r1,reversed:false,pcurve:line([1.,0.],[1.,1.])},
                Coedge {edge:eb,reversed:true,pcurve:line([0.,1.],[1.,1.])},
                Coedge {edge:r0,reversed:true,pcurve:line([0.,0.],[0.,1.])},
            ])
        }
        // **on the surface the turn sweeps the section over** (`meridian_surface`): each corner's
        // parameters read there, unwrapped along the rims (the angle about the axis advancing by the
        // turn from the start rim to the end's), every edge inverted onto it; a pole a degenerate
        // edge only where the surface has one too (a ball's, a cone's apex, a meridian's end on the
        // axis), where a plane holds the point as any other and the rims meet at it
        Some(s) => {
            let periods = s.periods();
            let unwrap = |x: Uv,near: Uv| crate::topo::unwrap(x,near,periods);
            // (the frame of a surface about the turn's axis, none for a plane square to it)
            let axial: Option<&Frame> = match &s {
                Surface::Cylinder(f,..) | Surface::Cone(f,..) | Surface::Sphere(f,..) | Surface::Torus(f,..) | Surface::Revolution(f,..) => Some(f),
                _ => None,
            };
            // a rim's parameters at its ends, walked along it from its start (read a hair inside an
            // end at a pole, where the angle is not)
            // (on a surface about the turn's axis its `u` is the angle about the axis, read so — near the
            // axis a torus's inverse takes the meridian across it, half a turn off, where a rim starts
            // at a pole of it — and a torus's `v` the point's place on its own meridian)
            let about = |x: V| -> Uv {
                let mut uv = s.inverse(x);
                if let Some(f) = axial {
                    let l = f.local(x);
                    if l[0].hypot(l[1]) > 0. { uv[0] = l[1].atan2(l[0]) }
                    if let Surface::Torus(_,big,_) = s { uv[1] = l[2].atan2(l[0].hypot(l[1])-big) }
                }
                uv
            };
            // a rim's parameters at `K + 1` points evenly along it, unwrapped, the angle of a point within a
            // millionth of the size of the axis that of the nearest point clear of it: a rim reaching the
            // axis tangent to it — a horn torus's meridian — stands off it by the square of the way, and
            // a hair along it its angle is rounding's
            let near_axis = |x: V| meridian_image(x,screw.origin,axis)[0] <= 1e-6*size;
            let read_rim = |c: &Curve,n: usize| -> Vec<Uv> {
                let pts: Vec<V> = (0..=n).map(|k| c.point(k as f64/n as f64)).collect();
                let mut uv: Vec<Uv> = pts.iter().map(|&x| about(x)).collect();
                let clear: Vec<usize> = (0..=n).filter(|&k| !near_axis(pts[k])).collect();
                if !clear.is_empty() && axial.is_some() {
                    for k in 0..=n {
                        if !near_axis(pts[k]) { continue }
                        let j = *clear.iter().min_by_key(|&&j| j.abs_diff(k)).unwrap();
                        uv[k][0] = uv[j][0];
                    }
                }
                for k in 1..=n { uv[k] = unwrap(uv[k],uv[k-1]) }
                uv
            };
            let walk = |c: &Curve| -> [Uv;2] { let uv = read_rim(c,64); [uv[0],uv[64]] };
            let [s0,s1] = walk(&a);
            // (the end rim's angle the start's turned by the turn, signed as the surface's frame runs
            // along the axis or against it, not the nearest branch: a turn past half a turn — the pin
            // swung 280° — read so ran back the short way round)
            let du = axial.map_or(0.,|f| crate::util::space::dot(f.z,axis).signum()*screw.ratio*span);
            let ahead = |x: Uv| [x[0]+du,x[1]];
            let [e0,e1] = { let [x,y] = walk(&image); [unwrap(x,ahead(s0)),unwrap(y,ahead(s1))] };
            // (on a surface about the axis, a rim's own curve in the parameters, read by the angle about
            // the axis against the edge's parameter, and a rail the line it is there: inverted by the
            // surface, near a pole of it a point's angle was read across the axis, half a turn off)
            let about_axis = axial.is_some();
            // (interpolated through as many points as hold it within a tenth of the tolerance of the
            // rim between them, from 64 doubled)
            let rim_curve = |c: &Curve,from: Uv| -> Option<Pcurve> {
                let mut n = 64;
                loop {
                    let ts: Vec<f64> = (0..=n).map(|k| k as f64/n as f64).collect();
                    let read = read_rim(c,n);
                    let shift = unwrap(read[0],from);
                    let pts: Vec<V> = read.iter().map(|uv| [uv[0]+shift[0]-read[0][0],uv[1]+shift[1]-read[0][1],0.]).collect();
                    let spline = crate::nurbs::interpolate(&pts,&ts,3)?;
                    // (its own error: against the rim's foot on the surface, which a fitted meridian holds
                    // only to its fit)
                    let off = (0..n).map(|k| {
                        let t = (k as f64+0.5)/n as f64;
                        let (q,x) = (spline.point(t),c.point(t));
                        let foot = if near_axis(x) { x } else { s.point(about(x)) };
                        distance(s.point([q[0],q[1]]),foot)
                    }).fold(0f64,f64::max);
                    if off <= 0.1*tol || n >= 4096 {
                        if n >= 1024 && crate::debug() { eprintln!("contacts: a rim's pcurve on a {} through {} points, {off:e} off it",s.kind(),n+1) }
                        return Some(Pcurve::Curve(std::sync::Arc::new(Curve::BSpline(std::sync::Arc::new(spline)))))
                    }
                    n *= 2;
                }
            };
            let rim_on = |c: &Curve,x: Uv,y: Uv| if about_axis { rim_curve(c,x).unwrap_or(Pcurve::Inverse {a:x,b:y}) } else { Pcurve::Inverse {a:x,b:y} };
            let rail_on_s = |x: Uv,y: Uv| if about_axis { line(x,y) } else { Pcurve::Inverse {a:x,b:y} };
            // (read at the pole's own parameters, where a ball's `S_u` is nought, not a hair inside a rim)
            let pole_on = |t: f64| { let (_,su,_) = s.d1(s.inverse(a.point(t))); norm_of(su) <= 1e-9*size };
            let mut l = vec![Coedge {edge:ea,reversed:false,pcurve:rim_on(&a,s0,s1)}];
            // the rail at the section's end, then the end rim back, then the rail at its start
            let rail_on = |b: &mut Brep,t: f64,ends: [u32;2],[x,y]: [Uv;2]| -> Option<Coedge> {
                if pole(t) {
                    if !pole_on(t) { return None }
                    // (at the pole's own `v`: read a hair inside a rim it stood that hair along the tube off)
                    let v = unwrap(about(a.point(t)),x)[1];
                    return Some(Coedge {edge:b.edge(EdgeCurve::Degenerate,[0.,1.],ends),reversed:false,pcurve:line([x[0],v],[y[0],v])})
                }
                // (an arc about the turn's axis, laid as `ssi` lays a circle two surfaces about it meet in,
                // so the meeting the arrangement finds along it is it: read as the carried section's
                // `Iso`, a meeting circle along a rail crossed it anywhere along their run)
                let p = a.point(t);
                let along = crate::util::space::unit(screw.axis);
                let foot = add(screw.origin,scale(along,crate::util::space::dot(crate::util::space::sub(p,screw.origin),along)));
                let turn = screw.ratio*span;
                let z = if turn >= 0. { along } else { scale(along,-1.) };
                let frame = Frame::about(foot,z);
                let l = frame.local(p);
                let rho = l[0].hypot(l[1]);
                let start = l[1].atan2(l[0]);
                let edge = b.edge(EdgeCurve::Curve(Curve::Circle(frame,rho)),[start,start+turn.abs()],ends);
                Some(Coedge {edge,reversed:false,pcurve:rail_on_s(x,y)})
            };
            let r1 = if closed { None } else { rail_on(&mut b,1.,[p1,q1],[s1,e1]) };
            l.extend(r1);
            l.push(Coedge {edge:eb,reversed:true,pcurve:rim_on(&image,e0,e1)});
            let r0 = rail_on(&mut b,0.,[p0,q0],[s0,e0]);
            if let Some(r) = r0 { l.push(Coedge {reversed:true,..r}) }
            (s,l)
        }
    };
    let face = Face {surface,reversed:false,loops:vec![],name:"contact".into()};
    if b.loop_area(&face,&l) < 0. { l = crate::topo::reverse_loop(l); }
    b.faces.push(Face {loops:vec![l],..face});
    let rails = [Curve::Iso(blend.clone(),0.),Curve::Iso(blend.clone(),1.)];
    Ok(Carried {sheet:b,rims,rails,blend})
}

/// A point's meridian image about the axis through `o` along the unit `z`: its distance from the
/// axis and its height along it, what a turn about that axis keeps of it.
fn meridian_image(p: V,o: V,z: V) -> [f64;2] {
    let q = crate::util::space::sub(p,o);
    let h = crate::util::space::dot(q,z);
    [norm_of(crate::util::space::sub(q,scale(z,h))),h]
}

/// **The surface a section sweeps under a turn** (#880 Q1b S5): its points carried about the turn's
/// axis (through `o` along the unit `z`) run round circles, so its sheet lies on the surface of
/// revolution of its meridian image `(ρ, h)`. A plane, a cylinder, a cone, a ball or a torus about the
/// axis where the image is a level or upright line, a line or a circle within `tol` — which
/// `ssi::same` reads as one with any other built so, and the arrangement meets with another about the
/// axis in circles, in closed form; else a surface of revolution of the image fitted (`fitted`), one
/// already in `made` whose meridian holds this image within the tolerance used again, so two
/// sections of one image — a tool's mirror pieces in a plane through the axis, the rod's end rim's
/// two halves — sweep one surface, whose coincident faces the trim keeps once.
fn meridian_surface(section: &crate::geom::Curve,o: V,z: V,tol: f64,made: &mut Vec<(Surface,Vec<[f64;2]>)>) -> Option<Surface> {
    use crate::geom::Frame;
    let image = |p: V| meridian_image(p,o,z);
    const N: usize = 64;
    let pts: Vec<[f64;2]> = (0..=N).map(|k| image(section.point(k as f64/N as f64))).collect();
    let mean = |k: usize| pts.iter().map(|p| p[k]).sum::<f64>()/pts.len() as f64;
    let within = |f: &dyn Fn([f64;2]) -> f64| pts.iter().all(|&p| f(p).abs() <= tol);
    let at = |h: f64| add(o,scale(z,h));
    // a level line: a plane square to the axis
    let h = mean(1);
    if within(&|p| p[1]-h) { return Some(Surface::Plane(Frame::about(at(h),z))) }
    // an upright line: a cylinder about it
    let r = mean(0);
    if r > tol && within(&|p| p[0]-r) { return Some(Surface::Cylinder(Frame::about(o,z),r)) }
    let (a,b) = (pts[0],pts[N]);
    let chord = [b[0]-a[0],b[1]-a[1]];
    let length = chord[0].hypot(chord[1]);
    // a line: a cone about the axis, opening along `z` or against it
    if length > tol && within(&|p| ((p[0]-a[0])*chord[1]-(p[1]-a[1])*chord[0])/length) {
        let (z2,dh) = if chord[1] >= 0. { (z,chord[1]) } else { (scale(z,-1.),-chord[1]) };
        return Some(Surface::Cone(Frame::about(at(a[1]),z2),a[0],chord[0].atan2(dh)))
    }
    // a circle through three of its points: a ball on the axis, a torus off it (its quarters' where its
    // ends meet: a closed image, the coin's rim in a plane through the axis, round from where it
    // touches the axis back to it)
    let (a,b,m) = if length > 1e-6*(1.+a[0].abs()+a[1].abs()) { (a,b,pts[N/2]) } else { (pts[N/4],pts[3*N/4],pts[N/2]) };
    let length = (b[0]-a[0]).hypot(b[1]-a[1]).max((m[0]-a[0]).hypot(m[1]-a[1]));
    let circle = || -> Option<([f64;2],f64)> {
        let (bx,by,cx,cy) = (b[0]-a[0],b[1]-a[1],m[0]-a[0],m[1]-a[1]);
        let d = 2.*(bx*cy-by*cx);
        if !(d.abs() > 1e-12*length*length) { return None }
        let (b2,c2) = (bx*bx+by*by,cx*cx+cy*cy);
        let c = [a[0]+(cy*b2-by*c2)/d,a[1]+(bx*c2-cx*b2)/d];
        Some((c,(a[0]-c[0]).hypot(a[1]-c[1])))
    };
    if let Some((c,radius)) = circle().filter(|&(c,r)| within(&|p| (p[0]-c[0]).hypot(p[1]-c[1])-r)) {
        if c[0].abs() <= tol { return Some(Surface::Sphere(Frame::about(at(c[1]),z),radius)) }
        // (a circle about a centre beyond the axis, its mirror's: revolved, one surface — a ball reaching
        // across the axis, whose great circle's two arcs sweep one spindle torus)
        return Some(Surface::Torus(Frame::about(at(c[1]),z),c[0].abs(),radius))
    }
    // a surface of revolution of the image, one already made for it if one holds it
    let off = |q: [f64;2],line: &[[f64;2]]| line.windows(2).map(|w| {
        let (d,e) = ([w[1][0]-w[0][0],w[1][1]-w[0][1]],[q[0]-w[0][0],q[1]-w[0][1]]);
        let dd = d[0]*d[0]+d[1]*d[1];
        let t = if dd > 0. { ((e[0]*d[0]+e[1]*d[1])/dd).clamp(0.,1.) } else { 0. };
        (e[0]-t*d[0]).hypot(e[1]-t*d[1])
    }).fold(f64::INFINITY,f64::min);
    if let Some((s,_)) = made.iter().find(|(_,line)| pts.iter().all(|&p| off(p,line) <= 2.*tol)) { return Some(s.clone()) }
    let frame = Frame::about(o,z);
    let place = |p: [f64;2]| add(o,add(scale(frame.x,p[0]),scale(z,p[1])));
    let nodes: Vec<(Uv,V)> = (0..=16).map(|k| { let t = k as f64/16.; ([t,0.],place(image(section.point(t)))) }).collect();
    let between = |x: Uv,y: Uv| { let t = 0.5*(x[0]+y[0]); ([t,0.],place(image(section.point(t)))) };
    let meridian = fitted(nodes,&between,tol)?;
    let dense: Vec<[f64;2]> = (0..=256).map(|k| image(meridian.point(k as f64/256.))).collect();
    let surface = Surface::Revolution(frame,std::sync::Arc::new(meridian));
    made.push((surface.clone(),dense));
    Some(surface)
}

/// A piece's sheet carried over the roll (`carried`), and its rims: the piece at the roll's start
/// and at its end, where the sheet meets the tool placed there.
pub struct Carried { pub sheet: Brep,pub rims: [crate::geom::Curve;2],
    /// The paths of the curve's ends over the roll (a closed curve's two the same).
    pub rails: [crate::geom::Curve;2],
    /// The section carried: the sheet's points by the section's parameter and the roll's, whatever
    /// surface its face lies on.
    pub blend: std::sync::Arc<crate::geom::Blend> }

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
    let speed = ts.iter().map(|&t| norm_of(twist.velocity(c.point(t)))).fold(0f64,f64::max);
    // where the fan's sign turns, each root of one of the two faces' `n · v` on the edge
    // (the face whose `n · v` turns over the bracket, the bracket's root of it)
    // (a root of either at an end of the bracket is the root: a sample standing on it — a symmetric
    // tool's, the coin's rim's lowest point a sample — read with the other face's sign, the bracket's
    // far end was taken, a sample off)
    let root = |a: f64,b: f64| {
        let small = 1e-12*speed;
        for f in [f1,f2] { for x in [a,b] { if normal(f,c.point(x)).abs() <= small { return x } } }
        let (ga,gb) = (normal(f1,c.point(a)),normal(f1,c.point(b)));
        let f = if (ga < 0.) != (gb < 0.) { f1 } else { f2 };
        let g = |t: f64| normal(f,c.point(t));
        crate::util::roots::bracketed_root(|t: f64| Some(g(t)),a,g(a),b,g(b),1e-14*(1.+t1.abs()))
    };
    // (a sample in contact where the product is not past its rounding above nought: where both faces'
    // `n · v` vanish at once and the product touches nought from below — the rod spun about a line
    // through it, whose end rims are in contact all round but where its top and bottom lines reach
    // them — the fan stays in contact through it; read by sign alone, its rounding cut the rim there,
    // into runs of no length among them)
    let noise = 1e-12*speed*speed;
    let inside: Vec<bool> = fs.iter().map(|&x| x <= noise).collect();
    let mut runs: Vec<[f64;2]> = Vec::new();
    let mut start: Option<f64> = inside[0].then_some(t0);
    for k in 0..N {
        if inside[k] == inside[k+1] { continue }
        let x = root(ts[k],ts[k+1]);
        match start.take() { Some(a) => runs.push([a,x]),None => start = Some(x) }
    }
    if let Some(a) = start { runs.push([a,t1]) }
    // (and none where the product is nowhere below nought past its rounding — a fan touching contact
    // at a point, both faces' `n · v` vanishing there, holds no stretch — unless one face's `n · v`
    // vanishes all along it: the edge lies in that face's contact, its fan holding the face's normal)
    let along = |f: usize,k0: usize,k1: usize| ts[k0..k1].iter().all(|&t| normal(f,c.point(t)).abs() <= 1e-9*speed);
    runs.retain(|r| { let k0 = ts.partition_point(|&t| t < r[0]); let k1 = ts.partition_point(|&t| t <= r[1]);
        k1 > k0 && (fs[k0..k1].iter().any(|&x| x < -noise) || k1 >= k0+3 && (along(f1,k0,k1) || along(f2,k0,k1))) });
    // a closed edge's run through its seam is one
    if edge.closed() && runs.len() > 1 && runs[0][0] == t0 && runs[runs.len()-1][1] == t1 {
        let last = runs.pop().unwrap();
        runs[0] = [last[0],runs[0][1]+(t1-t0)];
    }
    runs
}

/// **Where a section's meridian image folds back** under a turn about the axis through `o` along the
/// unit `z` (#880 Q1b S5): the parameters among `ts` where the image `(ρ, h)` of `at` turns back on
/// itself, each set where the image stands farthest along its way in (golden section between the
/// samples either side). There the section runs along the turn's velocity, and its sheet folds onto
/// itself: the stretches either side sweep one surface, each over its own angles.
fn folds(at: &dyn Fn(f64) -> V,ts: &[f64],o: V,z: V,tol: f64) -> Vec<(f64,bool)> {
    let image = |t: f64| meridian_image(at(t),o,z);
    let im: Vec<[f64;2]> = ts.iter().map(|&t| image(t)).collect();
    let step = |i: usize,j: usize| [im[j][0]-im[i][0],im[j][1]-im[i][1]];
    let scale_of = im.windows(2).map(|w| (w[1][0]-w[0][0]).hypot(w[1][1]-w[0][1])).fold(0f64,f64::max);
    // where `f` is least between `lo` and `hi`, by golden section
    let least = |f: &dyn Fn(f64) -> f64,mut lo: f64,mut hi: f64| -> f64 {
        for _ in 0..80 {
            let (m1,m2) = (lo+(hi-lo)*0.381966011250105,lo+(hi-lo)*0.618033988749895);
            if f(m1) <= f(m2) { hi = m2 } else { lo = m1 }
        }
        0.5*(lo+hi)
    };
    let mut out = Vec::new();
    let tiny = 1e-9*scale_of;
    for k in 1..ts.len().saturating_sub(1) {
        let (d1,d2) = (step(k-1,k),step(k,k+1));
        if d1[0].hypot(d1[1]) <= tiny || d2[0].hypot(d2[1]) <= tiny || d1[0]*d2[0]+d1[1]*d2[1] >= 0. { continue }
        // farthest along `d1` between the samples either side
        let along = |t: f64| { let p = image(t); (p[0]-im[k-1][0])*d1[0]+(p[1]-im[k-1][1])*d1[1] };
        out.push((least(&|t| -along(t),ts[k-1],ts[k+1]),false));
    }
    // and where it crosses the axis: a point that does not move, a pole of the sheets either side,
    // where its angle about the axis turns by half a turn (the ball swung across its own centre line,
    // whose great circle crosses the axis twice) — read where `ρ` falls to a least within a ten
    // tolerance of nought
    for k in 1..ts.len().saturating_sub(1) {
        if !(im[k][0] <= im[k-1][0] && im[k][0] <= im[k+1][0]) { continue }
        let t = least(&|t| image(t)[0],ts[k-1],ts[k+1]);
        if image(t)[0] <= tol {
            out.retain(|&(x,_)| (x-t).abs() > ts[k+1]-ts[k-1]);
            out.push((t,true));
        }
    }
    out.sort_by(|a,b| a.0.total_cmp(&b.0));
    out
}

/// **A piece of contact cut where its meridian image folds back** under a turn (`folds`, read on its
/// own points, each cut at the point nearest the fold, `Singular` there), a closed piece opened at
/// its first fold; whole where it folds nowhere.
fn folded(p: &Piece,surface: &Surface,o: V,z: V,tol: f64) -> Vec<Piece> {
    let m = p.points.len();
    if m < 3 { return vec![p.clone()] }
    let at = |t: f64| { let i = (t.floor() as usize).min(m-2); crate::util::space::lerp(p.points[i],p.points[i+1],t-i as f64) };
    let ts: Vec<f64> = (0..m).map(|k| k as f64).collect();
    let found = folds(&at,&ts,o,z,tol);
    // (a crossing of the axis put on it, between the two points it lies between, its parameters
    // there in proportion: the piece's own points stand off it by their spacing)
    let mut p = p.clone();
    let mut shift = 0;
    let mut cuts: Vec<usize> = Vec::new();
    for (t,on_axis) in found {
        let i = (t.floor() as usize).min(m-2)+shift;
        let f = t-t.floor();
        if on_axis && f > 1e-9 && f < 1.-1e-9 {
            let q = at(t);
            // (settled onto the face's surface along the axis: every point of it there is in contact,
            // not moving; the chord's own height stood its sag off it, past the kernel's tolerance)
            let mut h = crate::util::space::dot(crate::util::space::sub(q,o),z);
            for _ in 0..30 {
                let x = add(o,scale(z,h));
                let (f,d) = (surface.implicit(x),crate::util::space::dot(surface.gradient(x),z));
                if !(d.abs() > 1e-12) { break }
                let step = f/d;
                h -= step;
                if step.abs() <= 1e-15*(1.+h.abs()) { break }
            }
            let uv = [p.uv[i][0]+f*(p.uv[i+1][0]-p.uv[i][0]),p.uv[i][1]+f*(p.uv[i+1][1]-p.uv[i][1])];
            p.points.insert(i+1,add(o,scale(z,h)));
            p.uv.insert(i+1,uv);
            shift += 1;
            cuts.push(i+1);
        } else {
            cuts.push(((t.round() as usize)+shift).clamp(1,p.points.len()-2));
        }
    }
    cuts.dedup();
    split_at(&p,&cuts)
}

/// `p` cut at its points `cuts` (in order along it), each cut a corner (`Singular`) of the two
/// pieces either side; a closed piece opened at its first cut. No cut, `p` itself.
fn split_at(p: &Piece,cuts: &[usize]) -> Vec<Piece> {
    let m = p.points.len();
    if cuts.is_empty() { return vec![p.clone()] }
    let order: Vec<usize> = if p.closed { (0..m).map(|i| (cuts[0]+i)%(m-1)).collect() } else { (0..m).collect() };
    let at_order = |c: usize| if p.closed { (c+m-1-cuts[0])%(m-1) } else { c };
    let mut bounds: Vec<usize> = vec![0];
    bounds.extend(cuts.iter().map(|&c| at_order(c)).filter(|&c| c > 0));
    bounds.push(m-1);
    bounds.dedup();
    bounds.windows(2).enumerate().filter(|(_,w)| w[1] > w[0]).map(|(i,w)| {
        let first = p.closed || i > 0;
        let last = p.closed || i+2 < bounds.len();
        Piece {face:p.face,uv:order[w[0]..=w[1]].iter().map(|&k| p.uv[k]).collect(),points:order[w[0]..=w[1]].iter().map(|&k| p.points[k]).collect(),
            ends:[if first { End::Singular } else { p.ends[0] },if last { End::Singular } else { p.ends[1] }],closed:false}
    }).collect()
}

/// **Whether a piece of contact sweeps only the inside of its sweep** (#880 Q1b S5): at a point of
/// contact the motion carries the point along the face, and to second order its path dips inside the
/// face or rises off it — `κ_n + k · n`, the face's curvature along the velocity and the path's own
/// toward its normal. Where the path dips inside on either side of the point, the point's every pose
/// over the roll but its first (or its last) lies inside a pose of the tool just before (after) it,
/// so the piece's sheet bounds nothing past its rim, which the pose there holds: the end disc's line on the rod spun about a line through it,
/// whose path bends round the axis away from the flat disc, sweeps a band of radius 3 inside the
/// sweep, which only touches the planes and the ball its rails lie on. Read at seven of the piece's
/// inner points, each where the face's signed distance at the point carried a little either way falls
/// below nought past its rounding; a piece buried at all of them left out. Its points lie on the faces `on`
/// (a face's piece on its one, an edge's fan's on its two): the tool's inside there the greater of
/// their signed distances, so a fan's path dips inside where it passes into both faces' sides (the
/// coin turned about a line through its rim: its rim on the plane through the axis, in its side's
/// contact all round, moves into the coin where it also stays within the side, on its far half,
/// which sweeps only the inside). `size` is the tool's.
fn buried_on(tool: &Brep,size: f64,points: &[V],on: &[usize],screw: &crate::trajectory::Screw) -> bool {
    let m = points.len();
    // (read at its inner points: its ends are where it was cut, at a node of the contact where its
    // burial may change)
    const K: usize = 8;
    let reads: Vec<Option<bool>> = (1..K).map(|k| dips_at(tool,size,points[(k*(m-1))/K],on,screw)).collect();
    let known = reads.iter().flatten().count();
    known >= 3 && reads.iter().flatten().all(|&x| x)
}

/// Whether the path of `q`, a point of contact on the faces `on`, dips inside the tool to second order
/// (`buried_on`); `None` where it does not move (on the turn's axis, where a piece ends at a pole:
/// it abstains).
fn dips_at(tool: &Brep,size: f64,q: V,on: &[usize],screw: &crate::trajectory::Screw) -> Option<bool> {
    let twist = screw.twist();
    let speed = norm_of(twist.velocity(q));
    if !(speed > 1e-6*size*norm_of(twist.omega).max(1e-300)) { return None }
    let mut signs = Vec::new();
    for &fi in on {
        let face = &tool.faces[fi];
        let Some(n) = face.outward(q) else { return Some(false) };
        signs.push(if crate::util::space::dot(n,face.surface.gradient(q)) < 0. { -1. } else { 1. });
    }
    let step = 1e-3*size;
    let d = step/speed;
    let moved = |t: f64| screw.pose(t).point(q);
    let f = |x: V| on.iter().zip(&signs).map(|(&fi,&sg)| sg*tool.faces[fi].surface.implicit(x)).fold(f64::NEG_INFINITY,f64::max);
    // (either side enough: carried forward into the tool, each pose after the first lies inside
    // the one just before it; backward, each before the last inside the one just after — the
    // coin's rim, which moves into the coin and out of it backward through its own face)
    let dips = |x: V| f(x)/(step*step)*size < -1e-3;
    Some(dips(moved(d)) || dips(moved(-d)))
}

/// The parts of a piece not buried (`buried` true of those left out), each with the one of them that
/// follows it along the piece: the next kept, where it was the next before the buried were left out
/// (a closed piece's first after its last) — a buried part between two leaves them no rail to share.
fn unburied(parts: Vec<Piece>,closed: bool,buried: impl Fn(&Piece) -> bool) -> Vec<(Piece,Option<usize>)> {
    let all = parts.len();
    let (kept,parts): (Vec<usize>,Vec<Piece>) = parts.into_iter().enumerate().filter(|(_,q)| !buried(q)).unzip();
    let n = kept.len();
    parts.into_iter().enumerate().map(|(k,q)| {
        let next = if k+1 < n { (kept[k+1] == kept[k]+1).then_some(k+1) }
            else { (closed && n > 1 && kept[k] == all-1 && kept[0] == 0).then_some(0) };
        (q,next)
    }).collect()
}

/// **A piece of contact cut where its burial changes** (#880 Q1b S6): where the path of a point of it
/// passes from dipping inside the tool to rising off it (`dips_at`), the sheet passes from sweeping
/// only the inside to bounding, and its meeting with the tool posed at an end leaves its rim there; cut
/// there, a corner (`Singular`), which `pose_meeting` reads as a branch's end. Cut at a length instead,
/// a buried stretch beside one that bounds left a sliver where the two and the pose met that no
/// membership read settled (the spline bar's far contact). Each change read at 64 points along it and
/// bisected to a point of the piece, none within a thirty-second of its ends; an open piece only.
fn cut_at_burial(tool: &Brep,size: f64,p: &Piece,screw: &crate::trajectory::Screw) -> Vec<Piece> {
    let m = p.points.len();
    if p.closed || m < 4 { return vec![p.clone()] }
    let read = |i: usize| dips_at(tool,size,p.points[i],&[p.face],screw);
    const K: usize = 64;
    let reads: Vec<(usize,bool)> = (1..K).map(|k| (k*(m-1))/K).filter_map(|i| read(i).map(|r| (i,r))).collect();
    let mut cuts: Vec<usize> = Vec::new();
    for w in reads.windows(2) {
        let ((a,ra),(b,rb)) = (w[0],w[1]);
        if ra == rb { continue }
        let (mut lo,mut hi) = (a,b);
        while hi-lo > 1 {
            let mid = (lo+hi)/2;
            match read(mid) { Some(r) if r == ra => lo = mid,Some(_) => hi = mid,None => break }
        }
        // (not beside an end, already a cut: at a double point of the contact, where the leaning torus's
        // buried branch crosses its bounding one, the burial changes too, and a cut a hair from it
        // left a sheet of no width)
        if hi >= m/32 && hi+m/32 < m { cuts.push(hi) }
    }
    split_at(p,&cuts)
}

/// **The sharp edges of `tool`**, each with its two faces: an edge between two faces whose outward
/// normals part at its middle (S2). Refused at the middle of an edge where a face has no normal.
pub fn sharp_edges(tool: &Brep) -> Result<Vec<(usize,[usize;2])>,V> {
    let uses = tool.uses();
    let mut sharp = Vec::new();
    for (e,edge) in tool.edges.iter().enumerate() {
        if matches!(edge.curve,crate::topo::EdgeCurve::Degenerate) { continue }
        let [(f1,..),(f2,..)] = uses[e][..] else { continue };
        if f1 == f2 { continue }
        let mid = edge.point(0.5*(edge.t[0]+edge.t[1]),&tool.vertices);
        let (Some(n1),Some(n2)) = (tool.faces[f1].outward(mid),tool.faces[f2].outward(mid)) else { return Err(mid) };
        if crate::util::space::dot(n1,n2) < 1.-1e-9 { sharp.push((e,[f1,f2])) }
    }
    Ok(sharp)
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
    // **a ball given poles off its contact** (#880 Q1b S5): under a turn or a screw a ball is in
    // contact along its great circle in the plane through its centre and the motion's axis, and a
    // ball drawn about an axis in that plane has both its poles on it — a meeting through a pole,
    // which the arrangement does not split (`build::sphere`); the same ball drawn about that plane's
    // normal has them at the circle's own poles (the ball swung across its own centre line)
    let repoled;
    let tool = match &tool.faces[..] {
        [f] => match f.surface {
            Surface::Sphere(frame,r) => {
                let normal = crate::util::space::cross(crate::util::space::sub(frame.o,screw.origin),crate::util::space::unit(screw.axis));
                if norm_of(normal) > 1e-9*(1.+r) && norm_of(crate::util::space::cross(normal,frame.z)) > 1e-9 {
                    repoled = crate::build::sphere(frame.o,normal,r).map_err(Unbuilt::Failed)?;
                    &repoled
                } else { tool }
            }
            _ => tool,
        },
        _ => tool,
    };
    let sharp = sharp_edges(tool).map_err(|at| Unbuilt::Curved {at})?;
    let size = tool.size();
    let twist = screw.twist();
    let located = Located::new(tool,tol);
    let mut out = Vec::new();
    let mut given = Vec::new();
    // **under a turn, each sheet on the surface it sweeps** (#880 Q1b S5, `meridian_surface`): a
    // section's points run round circles about the axis, so its sheet lies on the surface of
    // revolution of its meridian image, and two sheets on surfaces about one axis meet in circles,
    // which the arrangement reads in closed form; a section is cut where its image folds back
    // (`folds`), each side its own sheet, and is not cut in stretches, which a screw asks for
    let turn = screw.is_turn();
    let (axis_o,axis_z) = (screw.origin,crate::util::space::unit(screw.axis));
    let mut made: Vec<(Surface,Vec<[f64;2]>)> = Vec::new();
    let mut on = |curve: &crate::geom::Curve| -> Result<Option<Surface>,Unbuilt> {
        if !turn { return Ok(None) }
        meridian_surface(curve,axis_o,axis_z,tol,&mut made).map(Some).ok_or_else(|| Unbuilt::Failed("a turned section's meridian not fitted".into()))
    };
    // every stretch's sheet, the sheet following it along its piece, the faces its section lies on and
    // the order it touches them in (a contact's two, an edge's fan's one), and how its section ends
    let mut carried_all: Vec<(Carried,Option<usize>,Vec<usize>,u32,[End;2])> = Vec::new();
    // where a face's contact ends: on an edge, a node of the contact the edge's fan is cut at
    let mut junctions: Vec<V> = Vec::new();
    let posed = |t: f64| -> Result<crate::geom::Rigid,Unbuilt> { let q = path.pose(t)?; Ok(crate::geom::Rigid {r:q.r,t:q.p}) };
    let ends = [posed(roll[0])?,posed(roll[1])?];
    for fi in 0..tool.faces.len() {
        match face_contact(tool,&located,fi,&twist,tol,CELLS) {
            Contact::Whole => {
                let at = tool.faces[fi].loops.iter().flatten().next().map_or([0.;3],|c| tool.vertices[tool.ends(c)[0] as usize].p);
                return Err(Unbuilt::Curved {at})
            }
            Contact::Pieces(pieces) => for p in pieces {
                junctions.extend([p.points[0],*p.points.last().unwrap()]);
                let parts = if turn { folded(&p,&tool.faces[p.face].surface,axis_o,axis_z,tol) }
                    else { cut_at_burial(tool,size,&p,&screw).into_iter().flat_map(|q| stretches(&q,size,&screw)).collect() };
                let whole = parts.len() == 1;
                // (each part read for burial alone: a piece crossing the turn's axis buried on one side
                // of it and bounding on the other, the ball swung across its own centre line)
                let parts = unburied(parts,p.closed,|part| {
                    let out = buried_on(tool,size,&part.points,&[p.face],&screw);
                    if crate::debug() { eprintln!("contacts: face {fi}'s part from {:?} to {:?}{}",part.points[0],part.points.last().unwrap(),if out { ", buried" } else { "" }) }
                    out
                });
                let whole = whole && parts.len() == 1;
                let mut sheets: Vec<Carried> = Vec::new();
                for (part,_) in &parts {
                    let curve = piece_curve(tool,&located,part,&twist,tol).ok_or_else(|| Unbuilt::Failed("a piece of contact not fitted".into()))?;
                    let surface = on(&curve)?;
                    sheets.push(carried(&curve,whole && p.closed,path,roll,surface,tol)?);
                }
                // each stretch followed by the next along the piece
                let first = carried_all.len();
                for (c,(part,next)) in sheets.into_iter().zip(&parts) {
                    carried_all.push((c,next.map(|j| first+j),vec![p.face],2,part.ends));
                }
            },
        }
    }
    // **an edge's fan** (#880 Q1b S2): a sharp edge's point bounds the sweep where its velocity is
    // square to a normal of the fan between its two faces' — where those faces' `n · v` differ in sign.
    // Each such stretch of it, its ends where one face's contact reaches the edge, carried over the
    // roll: its section the edge itself, fitted through points of it, crossing each of its two faces
    // posed at an end of the roll along the rim, simply
    for &(e,[f1,f2]) in &sharp {
        let edge = &tool.edges[e];
        let crate::topo::EdgeCurve::Curve(c) = &edge.curve else { continue };
        let runs = fan_runs(tool,e,[f1,f2],&twist);
        let [t0,t1] = edge.t;
        let whole = edge.closed() && runs.len() == 1 && runs[0] == [t0,t1];
        // (a closed edge read round its period: a run through its seam past `t1`, and a stretch of a
        // run all round it unwrapped along it, so the point between two of its parameters is between
        // them — taken across the seam as it stood, a stretch's midpoint stood on the far side of the
        // circle, and its section ran round it twice; the rod spun about a line through it, whose end
        // rims are in contact all round)
        let period = t1-t0;
        let at = |t: f64| c.point(if edge.closed() { t0+(t-t0).rem_euclid(period) } else { t });
        let between = |a: Uv,b: Uv| { let t = 0.5*(a[0]+b[0]); ([t,0.],at(t)) };
        // (under a turn, each run cut where its image folds back, at the points where both faces'
        // `n · v` vanish: a run all round opened at its first)
        let runs: Vec<([f64;2],[End;2],bool)> = if !turn { runs.into_iter().map(|r| (r,[End::Boundary;2],whole)).collect() } else {
            runs.into_iter().flat_map(|r| {
                let ts: Vec<f64> = (0..=512).map(|k| r[0]+(r[1]-r[0])*k as f64/512.).collect();
                let mut cuts: Vec<f64> = folds(&at,&ts,axis_o,axis_z,tol).into_iter().map(|(t,_)| t).collect();
                // (and where a face's contact ends on the edge: a node of the contact, where the fan's
                // burial may change — the coin's rim, buried past where its side's top and bottom
                // lines reach it)
                for &q in &junctions {
                    let (t,gap) = located.nearest(e as u32,q);
                    if gap > tol { continue }
                    let t = if edge.closed() { r[0]+(t-r[0]).rem_euclid(period) } else { t };
                    // (one cut where a fold stands at the same point)
                    if t > r[0]+1e-9*period && t < r[1]-1e-9*period && cuts.iter().all(|&x| distance(at(x),at(t)) > 1e3*tol) { cuts.push(t) }
                }
                cuts.sort_by(f64::total_cmp);
                if cuts.is_empty() { return vec![(r,[End::Boundary;2],whole)] }
                if whole { cuts.push(cuts[0]+period) }
                let mut bounds: Vec<(f64,End)> = Vec::new();
                if !whole { bounds.push((r[0],End::Boundary)) }
                bounds.extend(cuts.iter().map(|&t| (t,End::Singular)));
                if !whole { bounds.push((r[1],End::Boundary)) }
                // (a run all round cut once is cut again half way round: one section from a point
                // round back to it is a face over its surface's whole period, its two ends' lines one —
                // the coin's rim in a plane through the axis, from where it touches the axis round)
                if whole && bounds.len() == 2 { let half = 0.5*(bounds[0].0+bounds[1].0); bounds.insert(1,(half,End::Boundary)) }
                bounds.windows(2).map(|w| ([w[0].0,w[1].0],[w[0].1,w[1].1],false)).collect()
            }).collect()
        };
        for (run,run_ends,whole) in runs {
            let ts: Vec<f64> = (0..=64).map(|k| run[0]+(run[1]-run[0])*k as f64/64.).collect();
            let piece = Piece {face:f1,uv:ts.iter().map(|&t| [t,0.]).collect(),points:ts.iter().map(|&t| at(t)).collect(),
                ends:run_ends,closed:whole};
            let parts = if turn { vec![piece] } else { stretches(&piece,size,&screw) };
            // (a part whose path dips inside both faces' sides sweeps only the inside)
            let parts = unburied(parts,whole,|part| buried_on(tool,size,&part.points,&[f1,f2],&screw));
            let one = parts.len() == 1;
            let first = carried_all.len();
            for (part,next) in &parts {
                let mut ts: Vec<f64> = part.uv.iter().map(|u| u[0]).collect();
                if edge.closed() { for i in 1..ts.len() { ts[i] += period*((ts[i-1]-ts[i])/period).round() } }
                let nodes: Vec<(Uv,V)> = ts.iter().zip(&part.points).map(|(&t,&p)| ([t,0.],p)).collect();
                let curve = fitted(nodes,&between,tol).ok_or_else(|| Unbuilt::Failed("an edge's fan not fitted".into()))?;
                let surface = on(&curve)?;
                carried_all.push((carried(&curve,one && whole,path,roll,surface,tol)?,next.map(|j| first+j),vec![f1,f2],1,part.ends));
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
    // **under a turn, two sheets given their meeting less the rails they share** (#880 Q1b S5): both
    // on surfaces about the axis, they meet in circles in closed form (`ssi::intersect`); a circle
    // through an end of both sections is the path of a point both hold — a rail of each, an edge of
    // both faces, which the arrangement takes for one — and is not laid again: laid, a third circle
    // along the two, it was cut where neither runs (the coin's bottom sector and its rim's fan, along
    // the path of their shared corner)
    if turn {
        let image = |p: V| meridian_image(p,axis_o,axis_z);
        let ends = |c: &Carried| [image(c.rims[0].point(0.)),image(c.rims[0].point(1.))];
        for i in 0..carried_all.len() { for j in i+1..carried_all.len() {
            let (si,sj) = (&carried_all[i].0.sheet.faces[0].surface,&carried_all[j].0.sheet.faces[0].surface);
            let crate::ssi::Ssi::Curves(cs) = crate::ssi::intersect(si,sj,tol) else { continue };
            let (ei,ej) = (ends(&carried_all[i].0),ends(&carried_all[j].0));
            let shared = |c: &crate::geom::Curve| {
                let crate::geom::Curve::Circle(f,r) = c else { return false };
                let at = image(f.o);
                let on = |e: &[[f64;2];2]| e.iter().any(|q| (q[0]-r).abs() <= tol && (q[1]-at[1]).abs() <= tol);
                on(&ei) && on(&ej)
            };
            let curves: Vec<crate::geom::Curve> = cs.into_iter().filter(|c| !shared(c)).collect();
            given.push(crate::boolean::Given {surfaces:[si.clone(),sj.clone()],curves});
        } }
    } else {
        for i in 0..carried_all.len() { for j in i+1..carried_all.len() {
            let ((ci,ni,..),(cj,nj,..)) = (&carried_all[i],&carried_all[j]);
            let bi = &ci.blend;
            let ((si,ki),(sj,kj)) = (section(ci),section(cj));
            let mut curves = Vec::new();
            if *ni == Some(j) { curves.push(ci.rails[1].clone()) }
            if *nj == Some(i) { curves.push(cj.rails[1].clone()) }
            let near = |x: f64,y: f64| (x-y).abs() <= 1e-6;
            for [u,s] in shared_paths(&screw,&si,&sj,[ki,kj],1e-3*size,tol) {
                // (a path a hair from a section's end is the rail there, its end exactly)
                let snap = |x: f64| if x.abs() <= 1e-12 { 0. } else if (1.-x).abs() <= 1e-12 { 1. } else { x };
                let (u,s) = (snap(u),snap(s));
                // the rail two neighbours share: given already
                if (*ni == Some(j) && near(u,1.) && near(s,0.)) || (*nj == Some(i) && near(u,0.) && near(s,1.)) { continue }
                curves.push(crate::geom::Curve::Iso(bi.clone(),u));
            }
            given.push(crate::boolean::Given {surfaces:[ci.sheet.faces[0].surface.clone(),cj.sheet.faces[0].surface.clone()],curves});
        } }
    }
    // **every sheet given its whole meeting with the tool at each end** (`pose_meeting`): the rim it
    // touches the face its section lies on along, at that end, and what of it passes back into the
    // tool there, met in its own parameters — sought, a branch arriving at the rim where the two
    // part as the square of the way was marched to a stop short of it, and at a double point of the
    // contact, where it leaves the rim's end tangent to the rim, laid beside the rim
    let poses: Vec<Brep> = ends.iter().map(|m| tool.moved(m)).collect();
    let faces: Vec<Vec<crate::geom::Box3>> = poses.iter().map(|b| (0..b.faces.len()).map(|fi| crate::query::face_box(b,fi)).collect()).collect();
    // (a sheet clear of a face it does not touch meets it nowhere: not read)
    let sheets: Vec<crate::geom::Box3> = carried_all.iter().map(|(c,..)| c.sheet.bounds()).collect();
    // (the part of the roll a whole turn takes)
    let whole = (screw.ratio != 0.).then(|| std::f64::consts::TAU/(screw.ratio*(roll[1]-roll[0])).abs());
    let jobs: Vec<(usize,usize,usize)> = (0..carried_all.len()).flat_map(|k| (0..2).flat_map(move |e| (0..tool.faces.len()).map(move |fi| (k,e,fi)))).collect();
    let met: Vec<crate::boolean::Given> = crate::util::par::indices(jobs.len(),|j| {
        let (k,e,fi) = jobs[j];
        let (c,_,on,order,section) = &carried_all[k];
        let sheet = &c.sheet.faces[0].surface;
        let f = &poses[e].faces[fi];
        let touch = on.contains(&fi).then_some((e as f64,*order));
        let mut curves: Vec<crate::geom::Curve> = touch.map(|_| c.rims[e].clone()).into_iter().collect();
        let corners: Vec<Uv> = if touch.is_none() { Vec::new() } else {
            [0.,1.].into_iter().zip(section).filter(|(_,end)| matches!(end,End::Singular)).map(|(u,_)| [u,e as f64]).collect()
        };
        if touch.is_some() || crate::boolean::overlap(&sheets[k],&faces[e][fi],tol) {
            curves.extend(pose_meeting(&c.blend,&f.surface,faces[e][fi],touch,whole,&corners,tol));
        }
        // (a branch along a side of the sheet's parameters that is one point in space — a pole's,
        // where the section meets the turn's axis — meets nothing: laid, a loop of no size round the pole)
        curves.retain(|c| { let p = c.point(0.); (1..=16).any(|k| distance(c.point(k as f64/16.),p) > tol) });
        crate::boolean::Given {surfaces:[sheet.clone(),f.surface.clone()],curves}
    });
    given.extend(met);
    out.extend(carried_all.into_iter().map(|(c,..)| c.sheet));
    out.extend(poses);
    Ok((out,given))
}
