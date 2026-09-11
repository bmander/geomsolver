//! A grazing planar face: one the motion carries within its own plane (a face square to a turn's
//! axis, a face along a slide), where `n·v ≡ 0` and no contact curve says where the swept
//! material's boundary lies in that plane. What the face sweeps there is computed exactly instead,
//! in rows: circles about the pivot for a turn, lines across the advance for a slide. On a row the
//! face is a set of intervals along it, and what it sweeps those intervals widened by the motion
//! and merged. Between the rows at which that structure changes (a vertex, an edge's foot, an
//! arc's extreme, a gap between two intervals closing) every merged interval is a cell bounded by
//! two curves, face edges at the two end poses; each cell is sampled row by row and zipped. A row
//! two strips meet at is sampled once for both, so the mesh is watertight, and there the images of
//! every vertex, foot and extreme at the tracer's poses are samples too: the rim is the vertex
//! arcs, the edges' envelope arcs and the end-pose edges, and the other sheets' columns end on it
//! exactly where the region has a vertex.
use crate::motion::PlaneRigid;
use crate::solid::{PlanarEdge,PlanarLoop,SweepContacts,ToolFace,zip_polylines};
use crate::space::{cross,dot,normalised,sub};

type P2 = [f64;2];
const TAU: f64 = std::f64::consts::TAU;

/// The motion of a face within its own plane, in the plane's coordinates: turned about `pivot`
/// through `sweep` (radians, signed counter-clockwise), or slid by `advance`.
#[derive(Clone,Copy,Debug,PartialEq)]
pub enum PlaneMotion { Turn { pivot: P2, sweep: f64 }, Slide { advance: P2 } }

/// The swept region as a triangle mesh in the plane's coordinates, every triangle counter-clockwise:
/// its points, whether each is on the region's rim, and the row each was sampled on.
#[derive(Clone,Debug,Default,PartialEq)]
pub struct Region { pub points: Vec<P2>, pub triangles: Vec<[u32;3]>, pub rim: Vec<bool>, pub row: Vec<u32> }

/// A stretch of a face edge along which the row coordinate is monotone (`flat` where it is
/// constant: an arc about the pivot, a line along the slide).
#[derive(Clone,Copy,Debug)]
struct Piece { edge: usize, t: [f64;2], r: [f64;2], flat: bool }

/// A merged interval of a row: the pieces its two ends lie on and its ends along the row.
type Cell = (usize,usize,f64,f64);

struct Sweep { edges: Vec<PlanarEdge>, outer: PlanarLoop, holes: Vec<PlanarLoop>, turn: bool, o: P2, d: P2, n: P2, lo: f64, hi: f64, scale: f64, pieces: Vec<Piece> }

/// The region `outer` (less `holes`) sweeps under `motion`, rows no more than `spacing` apart and
/// every curve within `sagitta` of its chords; `poses` are the motion's own parameters (angles for
/// a turn, from 0 to the sweep; distances along the advance for a slide) at which the images of
/// the face's vertices, feet and extremes are samples of the rim.
pub fn swept_region(outer: &[PlanarEdge],holes: &[Vec<PlanarEdge>],motion: PlaneMotion,sagitta: f64,spacing: f64,poses: &[f64]) -> Result<Region,String> {
    if !(sagitta > 0.) || !(spacing > 0.) { return Err("a swept region needs a positive sagitta and spacing".into()); }
    Sweep::new(outer,holes,motion)?.mesh(sagitta,spacing,poses)
}

impl Sweep {
    fn new(outer: &[PlanarEdge],holes: &[Vec<PlanarEdge>],motion: PlaneMotion) -> Result<Sweep,String> {
        if outer.is_empty() { return Err("a swept region needs a face".into()); }
        let (turn,o,d,lo,hi) = match motion {
            PlaneMotion::Turn {pivot,sweep} => {
                if !(sweep.is_finite() && sweep != 0.) { return Err("a turn needs a nonzero sweep".into()); }
                (true,pivot,[1.,0.],sweep.min(0.),sweep.max(0.))
            }
            PlaneMotion::Slide {advance} => {
                let l = advance[0].hypot(advance[1]);
                if !(l.is_finite() && l > 0.) { return Err("a slide needs a nonzero advance".into()); }
                (false,[0.,0.],[advance[0]/l,advance[1]/l],0.,l)
            }
        };
        let outer_loop = PlanarLoop::new(outer.to_vec());
        let holes_loops: Vec<PlanarLoop> = holes.iter().map(|h| PlanarLoop::new(h.clone())).collect();
        let (blo,bhi) = (outer_loop.lo,outer_loop.hi);
        let scale = (bhi[0]-blo[0]).max(bhi[1]-blo[1]).max(o[0].abs()).max(o[1].abs()).max(1.);
        let edges: Vec<PlanarEdge> = outer.iter().chain(holes.iter().flatten()).copied().collect();
        let mut sweep = Sweep {edges,outer:outer_loop,holes:holes_loops,turn,o,d,n:[-d[1],d[0]],lo,hi,scale,pieces:Vec::new()};
        sweep.pieces = sweep.split();
        Ok(sweep)
    }

    fn rho(&self,p: P2) -> f64 { if self.turn { (p[0]-self.o[0]).hypot(p[1]-self.o[1]) } else { p[0]*self.n[0]+p[1]*self.n[1] } }
    fn theta(&self,p: P2) -> f64 { if self.turn { (p[1]-self.o[1]).atan2(p[0]-self.o[0]) } else { p[0]*self.d[0]+p[1]*self.d[1] } }
    fn point(&self,rho: f64,theta: f64) -> P2 {
        if self.turn { [self.o[0]+rho*theta.cos(),self.o[1]+rho*theta.sin()] } else { [rho*self.n[0]+theta*self.d[0],rho*self.n[1]+theta*self.d[1]] }
    }
    /// `x` moved by whole turns to within half a turn of `near` (a slide's rows do not wrap).
    fn near(&self,x: f64,near: f64) -> f64 { if self.turn { x+TAU*((near-x)/TAU).round() } else { x } }
    fn inside(&self,p: P2) -> bool { self.outer.contains(p) && !self.holes.iter().any(|h| h.contains(p)) }
    fn snap(&self) -> f64 { 1e-12*self.scale }
    /// How near two samples of one row are one: a row where a gap closes is found by bisecting
    /// the rows, so the gap left there is wider than the rows' own precision, and two ends that
    /// meet there are this far apart at most. Far below the sagitta, and below the weld.
    fn together(&self) -> f64 { 1e-9*self.scale }

    /// Every edge cut where its row coordinate turns: a line at its foot from the pivot, an arc at
    /// its nearest and farthest points (for a slide, where it runs along the advance).
    fn split(&self) -> Vec<Piece> {
        let mut pieces = Vec::new();
        for (i,e) in self.edges.iter().enumerate() {
            let mut cuts = vec![0.,1.];
            match *e {
                PlanarEdge::Line {start,end} => if self.turn {
                    let (ex,ey) = (end[0]-start[0],end[1]-start[1]);
                    let l2 = ex*ex+ey*ey;
                    if l2 > 0. { let f = ((self.o[0]-start[0])*ex+(self.o[1]-start[1])*ey)/l2; if f > 0. && f < 1. { cuts.push(f); } }
                },
                PlanarEdge::Arc {center,start,sweep,..} => {
                    let toward = if self.turn { [center[0]-self.o[0],center[1]-self.o[1]] } else { self.n };
                    if toward[0].hypot(toward[1]) > self.snap() && sweep != 0. {
                        let a = toward[1].atan2(toward[0]);
                        for extreme in [a,a+std::f64::consts::PI] {
                            for k in -3..=3 {
                                let t = (extreme+TAU*k as f64-start)/sweep;
                                if t > 0. && t < 1. { cuts.push(t); }
                            }
                        }
                    }
                }
            }
            cuts.sort_by(f64::total_cmp); cuts.dedup();
            for w in cuts.windows(2) {
                let r = [self.rho(e.at(w[0])),self.rho(e.at(w[1]))];
                // an arc about the pivot keeps its radius: its middle says so where its ends cannot
                let mid = self.rho(e.at(0.5*(w[0]+w[1])));
                let flat = (r[1]-r[0]).abs() <= self.snap() && (mid-r[0]).abs() <= self.snap();
                if (r[1]-r[0]).abs() <= self.snap() && !flat { continue; }
                pieces.push(Piece {edge:i,t:[w[0],w[1]],r,flat});
            }
        }
        pieces
    }

    /// Where piece `i` crosses the row at `rho`: its end exactly at an end's row, else bisected.
    fn crossing(&self,i: usize,rho: f64) -> P2 {
        let p = self.pieces[i];
        let e = self.edges[p.edge];
        if rho == p.r[0] { return e.at(p.t[0]); }
        if rho == p.r[1] { return e.at(p.t[1]); }
        let up = p.r[1] > p.r[0];
        let (mut a,mut b) = (p.t[0],p.t[1]);
        for _ in 0..200 {
            let m = 0.5*(a+b);
            if !(m > a.min(b) && m < a.max(b)) { break; }
            if (self.rho(e.at(m)) < rho) == up { a = m; } else { b = m; }
        }
        e.at(0.5*(a+b))
    }

    /// The row at `rho` (strictly between critical rows): whether the swept region covers it whole,
    /// and otherwise its merged intervals, each with the pieces its ends lie on.
    fn union(&self,rho: f64) -> (bool,Vec<Cell>) {
        let mut xs: Vec<(f64,usize)> = self.pieces.iter().enumerate()
            .filter(|(_,p)| !p.flat && rho > p.r[0].min(p.r[1]) && rho < p.r[0].max(p.r[1]))
            .map(|(i,_)| (self.theta(self.crossing(i,rho)),i)).collect();
        xs.sort_by(|a,b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
        let m = xs.len();
        if m == 0 { return (self.turn && self.inside(self.point(rho,0.)),Vec::new()); }
        let mut cells: Vec<Cell> = Vec::new();
        for i in 0..if self.turn { m } else { m-1 } {
            let (a,p) = xs[i];
            let (mut b,q) = xs[(i+1)%m];
            if i+1 == m { b += TAU; }
            if b-a > 0. && self.inside(self.point(rho,0.5*(a+b))) { cells.push((p,q,a+self.lo,b+self.hi)); }
        }
        cells.sort_by(|x,y| x.2.total_cmp(&y.2).then(x.3.total_cmp(&y.3)));
        // Sorted by their starts, intervals merge with the one before while they overlap it. An
        // interval inside another keeps that one's end, which is why the end is taken by
        // comparison and not from the later interval: two cells that overlap and are left as two
        // would both claim the stretch they share, and its edges would be walked twice over.
        let merge = |cells: Vec<Cell>| -> Vec<Cell> {
            let mut out: Vec<Cell> = Vec::new();
            for c in cells { match out.last_mut() { Some(l) if c.2 <= l.3 => if c.3 > l.3 { l.1 = c.1; l.3 = c.3; },_ => out.push(c) } }
            out
        };
        let mut merged = merge(cells);
        // round the circle: the last interval reaching past the first's start takes it in
        while self.turn && merged.len() > 1 && merged[merged.len()-1].3 >= merged[0].2+TAU {
            let last = merged.pop().unwrap();
            let first = merged[0];
            merged[0] = (last.0,if first.3+TAU >= last.3 { first.1 } else { last.1 },last.2-TAU,(first.3).max(last.3-TAU));
            merged = merge(merged);
        }
        if self.turn && merged.iter().any(|c| c.3-c.2 >= TAU) { return (true,Vec::new()); }
        (false,merged)
    }

    /// What the structure of a row is: covered whole, or its cells by their ends' pieces.
    fn signature(&self,rho: f64) -> (bool,Vec<(usize,usize)>) {
        let (full,cells) = self.union(rho);
        let mut s: Vec<(usize,usize)> = cells.iter().map(|c| (c.0,c.1)).collect();
        s.sort_unstable();
        (full,s)
    }

    /// The rows at which the structure changes: every piece's ends, the pivot where the face
    /// holds it, and every radius at which a gap between two widened intervals closes or opens
    /// (sampled a quarter spacing apart and bisected).
    fn critical(&self,spacing: f64) -> Vec<f64> {
        let mut rows: Vec<f64> = self.pieces.iter().flat_map(|p| p.r).collect();
        if self.turn && self.inside(self.o) { rows.push(0.); }
        rows.sort_by(f64::total_cmp);
        let snap = self.snap();
        let mut kept: Vec<f64> = Vec::new();
        for r in rows { if kept.last().map_or(true,|l| r-l > snap) { kept.push(r); } }
        let mut events = Vec::new();
        for w in kept.windows(2) {
            let (a,b) = (w[0],w[1]);
            let n = ((b-a)/(0.25*spacing)).ceil().max(4.) as usize;
            let at = |i: usize| a+(b-a)*(i as f64+0.5)/n as f64;
            let mut before = self.signature(at(0));
            for i in 1..n {
                let now = self.signature(at(i));
                if now != before {
                    let (mut x,mut y) = (at(i-1),at(i));
                    for _ in 0..200 {
                        if y-x <= snap { break; }
                        let m = 0.5*(x+y);
                        if self.signature(m) == before { x = m; } else { y = m; }
                    }
                    events.push(0.5*(x+y));
                }
                before = now;
            }
        }
        kept.extend(events);
        kept.sort_by(f64::total_cmp);
        let mut out: Vec<f64> = Vec::new();
        for r in kept { if out.last().map_or(true,|l| r-l > snap) { out.push(r); } }
        out
    }

    /// The strip's cells' intervals on one of the rows it shares with the strip beyond, made to
    /// partition it: cells that are apart in the middle of a strip may overlap at its end, where
    /// the gap between them closes (a row every circle about the pivot lies inside the face
    /// within). Two cells meshing one stretch would walk its edges twice over, so the stretch is
    /// halved between them.
    fn share(&self,cells: &[Cell],rho: f64) -> Vec<(f64,f64)> {
        let mut out: Vec<(f64,f64)> = cells.iter().map(|&c| self.interval(c,rho)).collect();
        let mut order: Vec<usize> = (0..out.len()).collect();
        order.sort_by(|&i,&j| out[i].0.total_cmp(&out[j].0));
        for w in order.windows(2) {
            let (i,j) = (w[0],w[1]);
            if out[i].1 > out[j].0 { let mid = 0.5*(out[i].1+out[j].0); out[i].1 = mid; out[j].0 = mid; }
        }
        if self.turn && order.len() > 1 {
            // and round the circle, where the last reaches into the first
            let (i,j) = (order[order.len()-1],order[0]);
            if out[i].1 > out[j].0+TAU { let mid = 0.5*(out[i].1+out[j].0+TAU); out[i].1 = mid; out[j].0 = mid-TAU; }
        }
        for c in out.iter_mut() { if c.1 < c.0 { c.1 = c.0; } }
        out
    }

    /// A cell's interval on the row at `rho`, kept continuous with its interval `reference` at
    /// the middle of its strip.
    fn interval(&self,cell: Cell,rho: f64) -> (f64,f64) {
        let s = self.near(self.theta(self.crossing(cell.0,rho))+self.lo,cell.2);
        let e = self.near(self.theta(self.crossing(cell.1,rho))+self.hi,s+(cell.3-cell.2));
        (s,e.max(s))
    }

    /// How many segments a row's stretch from `a` to `b` needs: none longer than `spacing`, and
    /// for a turn's circle none whose chord leaves it by more than `sagitta`.
    fn segments(&self,rho: f64,a: f64,b: f64,sagitta: f64,spacing: f64) -> usize {
        let w = (b-a).max(0.);
        let length = if self.turn { rho*w } else { w };
        let mut n = (length/spacing).ceil();
        if self.turn && rho > sagitta*0.5 { n = n.max(w/(2.*(1.-sagitta/rho).clamp(-1.,1.).acos()).max(1e-300)).ceil(); }
        n.max(1.) as usize
    }

    fn mesh(&self,sagitta: f64,spacing: f64,poses: &[f64]) -> Result<Region,String> {
        let critical = self.critical(spacing);
        let snap = self.snap();
        // the strips: their cells (by the middle row) and their rows, refined until every cell's
        // two curves are within the sagitta of their chords and no row is a spacing from the next
        struct Strip { full: bool, cells: Vec<Cell>, rows: Vec<f64>, ends: [Vec<(f64,f64)>;2] }
        let mut strips: Vec<Strip> = Vec::new();
        for w in critical.windows(2) {
            let (a,b) = (w[0],w[1]);
            let (full,cells) = self.union(0.5*(a+b));
            if !full && cells.is_empty() { continue; }
            let mut rows = vec![a];
            let mut stack = vec![(a,b,0)];
            let mut inner = Vec::new();
            while let Some((x,y,depth)) = stack.pop() {
                let m = 0.5*(x+y);
                let straight = cells.iter().all(|&c| {
                    let ends = |r: f64| { let (s,e) = self.interval(c,r); (self.point(r,s),self.point(r,e)) };
                    let ((sx,ex),(sm,em),(sy,ey)) = (ends(x),ends(m),ends(y));
                    off(sm,sx,sy) <= sagitta && off(em,ex,ey) <= sagitta
                });
                if depth < 40 && (y-x > spacing || !straight) { stack.push((m,y,depth+1)); stack.push((x,m,depth+1)); } else { inner.push(y); }
            }
            inner.sort_by(f64::total_cmp);
            rows.extend(inner);
            let ends = [self.share(&cells,rows[0]),self.share(&cells,*rows.last().unwrap())];
            strips.push(Strip {full,cells,rows,ends});
        }
        // the critical rows' samples, shared by the strips on either side: every cell's ends, the
        // images of the vertices, feet and extremes on the row, and fill between them
        let mut region = Region::default();
        let index = |r: f64| critical.iter().position(|&c| c == r);
        let mut touching: Vec<Vec<(f64,f64,bool)>> = vec![Vec::new();critical.len()];
        for s in &strips {
            for (r,end) in [(s.rows[0],0),(*s.rows.last().unwrap(),1)] {
                let k = index(r).ok_or("a strip ends off a critical row")?;
                if s.full { touching[k].push((0.,TAU,true)); }
                for &(a,b) in &s.ends[end] { touching[k].push((a,b,false)); }
            }
        }
        // per critical row: its samples as (along, vertex), in order along each covered stretch
        let mut shared: Vec<Vec<(f64,u32)>> = vec![Vec::new();critical.len()];
        let mut full_row = vec![false;critical.len()];
        for (k,list) in touching.iter().enumerate() {
            if list.is_empty() { continue; }
            let rho = critical[k];
            if self.turn && rho <= snap {
                // the pivot: one vertex for every sample
                let v = push(&mut region,self.o,k as u32);
                shared[k] = vec![(0.,v)];
                continue;
            }
            // the covered stretches of the row (round the circle for a turn)
            let mut spans: Vec<(f64,f64)> = list.iter().map(|&(a,b,full)| if full { (0.,TAU) } else if self.turn { let a0 = a.rem_euclid(TAU); (a0,a0+(b-a)) } else { (a,b) }).collect();
            spans.sort_by(|x,y| x.0.total_cmp(&y.0));
            let mut merged: Vec<(f64,f64)> = Vec::new();
            for s in spans { match merged.last_mut() { Some(l) if s.0 <= l.1+snap => l.1 = l.1.max(s.1),_ => merged.push(s) } }
            let full = self.turn && (merged.iter().any(|m| m.1-m.0 >= TAU-snap) || (merged.len() > 1 && merged[merged.len()-1].1 >= merged[0].0+TAU-snap && merged.windows(2).all(|w| w[1].0 <= w[0].1+snap)));
            full_row[k] = full;
            let spans: Vec<(f64,f64)> = if full { vec![(0.,TAU)] } else { merged };
            // required samples: the cells' ends and the images at the poses
            let mut required: Vec<f64> = list.iter().filter(|x| !x.2).flat_map(|&(a,b,_)| [a,b]).collect();
            for p in &self.pieces {
                for (end,&r) in p.r.iter().enumerate() {
                    if (r-rho).abs() > snap { continue; }
                    let base = self.theta(self.edges[p.edge].at(p.t[end]));
                    required.extend(poses.iter().map(|&psi| base+psi));
                }
            }
            let mut samples: Vec<f64> = Vec::new();
            for &(a,b) in &spans {
                let within = |x: f64| -> Option<f64> {
                    let y = if self.turn { a+(x-a).rem_euclid(TAU) } else { x };
                    let y = if self.turn && (y-b).abs() <= snap { b } else { y };
                    (y >= a-snap && y <= b+snap).then_some(y.clamp(a,b))
                };
                let mut here: Vec<f64> = required.iter().filter_map(|&x| within(x)).collect();
                here.push(a); if !full { here.push(b); }
                here.sort_by(f64::total_cmp);
                let mut fine: Vec<f64> = Vec::new();
                let top = if full { a+TAU } else { b };
                let mut stops = here.clone(); stops.push(top);
                for w in stops.windows(2) {
                    let n = self.segments(rho,w[0],w[1],sagitta,spacing);
                    for i in 0..n { fine.push(w[0]+(w[1]-w[0])*i as f64/n as f64); }
                }
                if !full { fine.push(b); }
                fine.sort_by(f64::total_cmp);
                let step = if self.turn { self.together()/rho.max(snap) } else { self.together() };
                for x in fine { if samples.last().map_or(true,|&l| x-l > step) { samples.push(x); } }
            }
            shared[k] = samples.into_iter().map(|x| (x,push(&mut region,self.point(rho,x),k as u32))).collect();
        }
        // the vertices of a cell's stretch on a critical row, in order (closed for a whole row)
        let pick = |k: usize,a: f64,b: f64,full: bool| -> Vec<u32> {
            let list = &shared[k];
            if list.len() == 1 { return vec![list[0].1]; }
            if full { return list.iter().map(|x| x.1).collect(); }
            let step = if self.turn { 4.*self.together()/critical[k].max(snap) } else { 4.*self.together() };
            let mut out: Vec<(f64,u32)> = Vec::new();
            let copies: &[f64] = if self.turn { &[-TAU,0.,TAU,2.*TAU] } else { &[0.] };
            for &(x,v) in list { for &c in copies { let y = x+c; if y >= a-step && y <= b+step { out.push((y,v)); } } }
            out.sort_by(|x,y| x.0.total_cmp(&y.0));
            let mut ids: Vec<u32> = Vec::new();
            for (_,v) in out { if ids.last() != Some(&v) { ids.push(v); } }
            ids
        };
        // each cell's rows, zipped
        let row_index = |r: f64| -> u32 { index(r).unwrap_or(critical.len()) as u32 };
        for s in &strips {
            let cells: Vec<(Option<usize>,Option<Cell>)> = if s.full { vec![(None,None)] } else { s.cells.iter().enumerate().map(|(i,&c)| (Some(i),Some(c))).collect() };
            for (which,cell) in cells {
                let last = s.rows.len()-1;
                let mut previous: Option<Vec<u32>> = None;
                for (j,&r) in s.rows.iter().enumerate() {
                    let ends = if j == 0 { Some(0) } else if j == last { Some(1) } else { None };
                    let (a,b) = match (cell,ends,which) {
                        (Some(_),Some(e),Some(i)) => s.ends[e][i],
                        (Some(c),_,_) => self.interval(c,r),
                        (None,_,_) => (0.,TAU),
                    };
                    let closed = cell.is_none();
                    let ids: Vec<u32> = if j == 0 || j == last {
                        let k = index(r).ok_or("a strip ends off a critical row")?;
                        pick(k,a,b,closed || full_row[k] && (b-a) >= TAU-snap)
                    } else if self.turn && r <= snap {
                        vec![push(&mut region,self.o,row_index(r))]
                    } else {
                        let n = self.segments(r,a,b,sagitta,spacing);
                        let count = if closed { n.max(3) } else { n+1 };
                        (0..count).map(|i| { let x = a+(b-a)*i as f64/if closed { count as f64 } else { n as f64 }; push(&mut region,self.point(r,x),u32::MAX) }).collect()
                    };
                    if let Some(prev) = &previous { band(&mut region,prev,&ids,closed); }
                    previous = Some(ids);
                }
            }
        }
        // the rim: the vertices on an edge only one triangle has
        let mut uses: std::collections::BTreeMap<(u32,u32),usize> = Default::default();
        for t in &region.triangles { for k in 0..3 { let (a,b) = (t[k],t[(k+1)%3]); *uses.entry((a.min(b),a.max(b))).or_default() += 1; } }
        region.rim = vec![false;region.points.len()];
        for (&(a,b),&n) in &uses { if n == 1 { region.rim[a as usize] = true; region.rim[b as usize] = true; } }
        Ok(region)
    }
}

fn push(region: &mut Region,p: P2,row: u32) -> u32 {
    region.points.push(p); region.row.push(row);
    (region.points.len()-1) as u32
}

/// How far `m` is from the chord between `a` and `b`.
fn off(m: P2,a: P2,b: P2) -> f64 {
    let (dx,dy) = (b[0]-a[0],b[1]-a[1]);
    let l2 = dx*dx+dy*dy;
    let f = if l2 > 0. { (((m[0]-a[0])*dx+(m[1]-a[1])*dy)/l2).clamp(0.,1.) } else { 0. };
    (m[0]-a[0]-f*dx).hypot(m[1]-a[1]-f*dy)
}

/// The triangles between two rows of one cell (a row of one vertex is a fan), every one wound
/// counter-clockwise and none with a repeated corner.
fn band(region: &mut Region,a: &[u32],b: &[u32],closed: bool) {
    let dedup = |l: &[u32]| { let mut o: Vec<u32> = Vec::new(); for &v in l { if o.last() != Some(&v) { o.push(v); } } if closed && o.len() > 1 && o[0] == o[o.len()-1] { o.pop(); } o };
    let (a,b) = (dedup(a),dedup(b));
    let mut tris: Vec<[u32;3]> = Vec::new();
    let fan = |p: u32,l: &[u32],tris: &mut Vec<[u32;3]>| {
        for k in 0..l.len()-1 { tris.push([p,l[k],l[k+1]]); }
        if closed && l.len() > 2 { tris.push([p,l[l.len()-1],l[0]]); }
    };
    if a.len() == 1 && b.len() > 1 { fan(a[0],&b,&mut tris); }
    else if b.len() == 1 && a.len() > 1 { fan(b[0],&a,&mut tris); }
    else if a.len() > 1 && b.len() > 1 {
        let pts = |l: &[u32]| l.iter().map(|&v| { let p = region.points[v as usize]; [p[0],p[1],0.] }).collect::<Vec<_>>();
        for t in zip_polylines(&pts(&a),&pts(&b),closed) { tris.push(t.map(|(on_b,k)| if on_b { b[k as usize] } else { a[k as usize] })); }
    }
    for mut t in tris {
        if t[0] == t[1] || t[1] == t[2] || t[2] == t[0] { continue; }
        let [p,q,r] = t.map(|v| region.points[v as usize]);
        let area2 = (q[0]-p[0])*(r[1]-p[1])-(q[1]-p[1])*(r[0]-p[0]);
        if area2 == 0. { continue; }
        if area2 < 0. { t.swap(1,2); }
        region.triangles.push(t);
    }
}

/// A planar face the motion carries within its own plane, with that plane in world coordinates at
/// the roll's start and the face's loops in the plane's own: what the region is swept from.
#[derive(Clone,Debug)]
pub struct GrazingFace {
    pub face: usize,
    pub motion: PlaneMotion,
    /// The plane at the start of the roll: `lift(a, b) = origin + a·u + b·v`.
    pub origin: [f64;3],
    pub u: [f64;3],
    pub v: [f64;3],
    /// The face's outward normal there.
    pub outward: [f64;3],
    pub outer: Vec<PlanarEdge>,
    pub holes: Vec<Vec<PlanarEdge>>,
}

impl GrazingFace {
    /// The plane point at the face's own coordinates.
    pub fn lift(&self,p: P2) -> [f64;3] { std::array::from_fn(|k| self.origin[k]+p[0]*self.u[k]+p[1]*self.v[k]) }
}

/// Every planar face of the tool whose plane the motion carries within itself — a face square to a
/// turn's axis, a face along a slide — and which lies on the tool's boundary. On such a face
/// `n·v ≡ 0` exactly, so the tracer's contact equation says nothing about it and what it sweeps
/// is computed from its loops instead. Anything else (a screw, a relative motion, a face the
/// Boolean hides) is not one, and keeps to the traced sheets.
pub fn grazing_faces(sweep: &SweepContacts) -> Result<Vec<GrazingFace>,String> {
    let [from,to] = sweep.domain();
    let start = sweep.motion().at(from)?;
    let mut out = Vec::new();
    for (i,face) in sweep.faces().iter().enumerate() {
        let ToolFace::Planar(f) = face else { continue };
        let (origin,u,v) = (start.point(f.origin),start.vector(f.u),start.vector(f.v));
        let Some(n) = normalised(cross(u,v)) else { continue };
        let Some(rigid) = sweep.motion().in_plane(n,origin) else { continue };
        let Some(sign) = sweep.face_outward(i)? else { continue };
        let plane = |p: [f64;3]| -> P2 { let r = sub(p,origin); [dot(r,u),dot(r,v)] };
        let motion = match rigid {
            PlaneRigid::Turn {centre,axis,rate} => PlaneMotion::Turn {pivot:plane(centre),sweep:rate*(to-from)*dot(axis,n).signum()},
            PlaneRigid::Slide {velocity} => PlaneMotion::Slide {advance:plane(std::array::from_fn(|k| origin[k]+velocity[k]*(to-from)))},
        };
        out.push(GrazingFace {face:i,motion,origin,u,v,outward:n.map(|x| x*sign),outer:f.outer.edges.clone(),holes:f.holes.iter().map(|h| h.edges.clone()).collect()});
    }
    Ok(out)
}
