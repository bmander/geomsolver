//! Protecting balls along the feature curves (`protect`), and their shrinking where refinement
//! needs them smaller (`shrink`).
use super::*;
use crate::space::{lerp,polyline_length,Grid};

/// The finest a protecting ball's gaps may become, as a fraction of `edge_size`.
pub(super) const LEAST: f64 = 1024.;
/// Rebuilds with shrunk balls before refusing.
pub(super) const REBUILDS: usize = 12;

/// How finely a feature curve is sampled: a uniform grid no coarser than `base`, its gaps halved
/// wherever they exceed the target spacing — `h` at arc length `s` for each `(s, h)` in `local`,
/// growing by `GRADING` per unit of arc length away from it. Halving a fixed grid rather than
/// walking the target keeps every station a later refinement does not reach exactly where it
/// was, so a rebuild meets the same balls, tetrahedra and orthocentres there as before.
#[derive(Clone,Debug)]
pub(super) struct Sizing { pub(super) base: f64,pub(super) local: Vec<(f64,f64)> }

const GRADING: f64 = 0.25;
/// A ball's radius over the shorter of its two gaps. The gaps are balanced — within a factor of
/// two of their neighbours, and none longer than both — so across every gap one ball has at least
/// the gap for its shorter one and the other at least half: 0.7 × 1.5 > 1, consecutive balls
/// overlap; next but one are a gap each apart and 0.7 < 1, they do not.
const BALL: f64 = 0.7;

impl Sizing {
    /// The finest target anywhere on the stretch `[a, b]`: a gap touching a constraint's
    /// place must come down to its spacing, where the target at the gap's middle would allow
    /// the grading's worth more and a station beside it would never be as fine as asked.
    fn over(&self,a: f64,b: f64,length: f64,closed: bool) -> f64 {
        self.local.iter().fold(self.base,|h,&(t,ht)| {
            let off = |t: f64| if t < a { a-t } else if t > b { t-b } else { 0. };
            let d = if closed { off(t).min(off(t-length)).min(off(t+length)) } else { off(t) };
            h.min(ht+GRADING*d)
        })
    }
    /// Arc-length stations from 0 to `length`; a closed curve's last, being its first, is left
    /// out.
    fn stations(&self,length: f64,closed: bool) -> Vec<f64> {
        let n = ((length/self.base).ceil() as usize).max(if closed { 3 } else { 1 });
        let mut gaps: Vec<(f64,f64)> = (0..n).map(|k| (length*k as f64/n as f64,length*(k+1) as f64/n as f64)).collect();
        let long = |a: f64,b: f64| a > b*1.0001;
        loop {
            let m = gaps.len();
            let size = |k: usize| gaps[k].1-gaps[k].0;
            let neighbours = |k: usize| -> Vec<f64> {
                let mut out = Vec::new();
                if k > 0 { out.push(size(k-1)); } else if closed { out.push(size(m-1)); }
                if k+1 < m { out.push(size(k+1)); } else if closed { out.push(size(0)); }
                out
            };
            let split: Vec<bool> = (0..m).map(|k| {
                let g = size(k);
                let near = neighbours(k);
                long(g,self.over(gaps[k].0,gaps[k].1,length,closed))
                    || near.iter().any(|&h| long(g,2.*h))
                    || (!near.is_empty() && near.iter().all(|&h| long(g,h)))
            }).collect();
            if !split.contains(&true) { break; }
            gaps = gaps.iter().zip(&split).flat_map(|(&(a,b),&cut)| {
                let mid = 0.5*(a+b);
                if cut { vec![(a,mid),(mid,b)] } else { vec![(a,b)] }
            }).collect();
        }
        let mut out: Vec<f64> = gaps.iter().map(|g| g.0).collect();
        if !closed { out.push(length); }
        out
    }
}

/// A protecting ball's place: its curve, its arc length along it, and the shorter of its gaps.
#[derive(Clone,Copy,Debug)]
pub(super) struct Place { curve: usize,s: f64,gap: f64 }

/// The feature curves as `protect` reads them: each with its length and whether it closes (its
/// last point its first).
struct Curves<'c> { curves: &'c [Vec<P>],lengths: Vec<f64>,closed: Vec<bool> }

impl<'c> Curves<'c> {
    fn new(curves: &'c [Vec<P>]) -> Self {
        Self {curves,lengths:curves.iter().map(|c| polyline_length(c)).collect(),
            closed:curves.iter().map(|c| c.len() >= 2 && dist2(c[0],*c.last().unwrap()) == 0.).collect()}
    }

    /// The point at arc length `s` along curve `k`.
    fn at(&self,k: usize,s: f64) -> P {
        let c = &self.curves[k];
        let mut left = s;
        for w in c.windows(2) {
            let l = dist2(w[0],w[1]).sqrt();
            if left <= l && l > 0. { return lerp(w[0],w[1],left/l); }
            left -= l;
        }
        *c.last().unwrap()
    }
}

/// One round's protecting balls: the curves' stations under the sizing, each ball's centre and
/// radius, and the `(curve, station)` places it stands at — a corner at several — with `at_place`
/// finding a ball by place.
struct Protection<'c> {
    curves: &'c Curves<'c>,
    edge_size: f64,
    stations: Vec<Vec<f64>>,
    balls: Vec<(P,f64)>,
    places: Vec<Vec<(usize,usize)>>,
    at_place: std::collections::BTreeMap<(usize,usize),usize>,
    /// Each ball's neighbours along its curves, and whether it is a corner.
    neighbours: Vec<Vec<usize>>,
    corner: Vec<bool>,
    /// The corners, in ball order.
    corners: Vec<usize>,
}

impl<'c> Protection<'c> {
    fn new(curves: &'c Curves<'c>,sizing: &[Sizing],edge_size: f64) -> Self {
        let stations: Vec<Vec<f64>> = (0..curves.curves.len()).map(|k| if curves.curves[k].len() < 2 { Vec::new() }
            else { sizing[k].stations(curves.lengths[k],curves.closed[k]) }).collect();
        let mut p = Self {curves,edge_size,stations,balls:Vec::new(),places:Vec::new(),at_place:Default::default(),
            neighbours:Vec::new(),corner:Vec::new(),corners:Vec::new()};
        for (k,c) in curves.curves.iter().enumerate() {
            let n = p.stations[k].len();
            for j in 0..n {
                // A curve's ends are its own points exactly: an end is a corner another curve
                // must meet.
                let end = j == 0 || (!curves.closed[k] && j+1 == n);
                let at = if j == 0 { c[0] } else if end { *c.last().unwrap() } else { curves.at(k,p.stations[k][j]) };
                let r = BALL*p.gap(k,j);
                let same = |q: P| dist2(q,at) <= (1e-9*edge_size)*(1e-9*edge_size);
                let b = match if end { p.balls.iter().position(|b| same(b.0)) } else { None } {
                    Some(b) => { p.balls[b].1 = p.balls[b].1.min(r); b }
                    None => { p.balls.push((at,r)); p.places.push(Vec::new()); p.balls.len()-1 }
                };
                p.places[b].push((k,j));
                p.at_place.insert((k,j),b);
            }
        }
        p.neighbours = (0..p.balls.len()).map(|b| p.next_to(b)).collect();
        p.corner = p.places.iter().map(|q| q.len() > 1).collect();
        p.corners = (0..p.balls.len()).filter(|&b| p.corner[b]).collect();
        p
    }

    /// The shorter gap either side of station `j` of curve `k`.
    fn gap(&self,k: usize,j: usize) -> f64 {
        let (st,closed,length) = (&self.stations[k],self.curves.closed[k],self.curves.lengths[k]);
        let n = st.len();
        let next = if j+1 < n { st[j+1]-st[j] } else if closed { length-st[j] } else { f64::INFINITY };
        let prev = if j > 0 { st[j]-st[j-1] } else if closed { length-st[n-1] } else { f64::INFINITY };
        next.min(prev)
    }

    /// Ball `b`'s neighbours along its curves, cyclically on a closed one.
    fn next_to(&self,b: usize) -> Vec<usize> {
        let mut out = Vec::new();
        for &(k,j) in &self.places[b] {
            let n = self.stations[k].len();
            let mut near = Vec::new();
            if j > 0 { near.push(j-1); } else if self.curves.closed[k] { near.push(n-1); }
            if j+1 < n { near.push(j+1); } else if self.curves.closed[k] { near.push(0); }
            out.extend(near.into_iter().filter_map(|m| self.at_place.get(&(k,m)).copied()));
        }
        out
    }

    /// Two balls may meet when consecutive, or both next to one corner.
    fn may_meet(&self,x: usize,y: usize) -> bool {
        self.neighbours[x].contains(&y)
            || self.neighbours[x].iter().any(|&c| self.corner[c] && self.neighbours[y].contains(&c))
    }

    /// Whether every place of ball `b` is within `edge_size` along its curve of a place of `c`.
    /// Near a corner where two curves meet at a small angle they part only linearly (or
    /// quadratically, tangent), and no spacing keeps their balls apart there: once a curve is at
    /// its finest spacing (an eighth of `edge_size`), balls within `edge_size` along their curves
    /// of a corner both share may meet. A distance and not a count of stations: a rebuild
    /// samples a curve more finely than the finest, and the region stays the same.
    fn near_corner(&self,b: usize,c: usize) -> bool {
        let reach = self.edge_size*1.0001;
        self.places[b].iter().all(|&(k,j)| self.places[c].iter().any(|&(kc,jc)| {
            kc == k && {
                let along = (self.stations[k][j]-self.stations[k][jc]).abs();
                along <= reach || (self.curves.closed[k] && self.curves.lengths[k]-along <= reach)
            }
        }))
    }

    /// Both balls beside one corner they share (or each other): likewise a curve turning more
    /// tightly than its finest balls, stations within `edge_size` of each other along it, the
    /// turn being a corner at that scale.
    fn beside(&self,x: usize,y: usize) -> bool {
        self.corners.iter().any(|&c| self.near_corner(x,c) && self.near_corner(y,c)) || self.near_corner(x,y)
    }

    /// Every place of ball `b` at the finest spacing.
    fn fine(&self,b: usize,finest: f64) -> bool { self.places[b].iter().all(|&(k,j)| self.gap(k,j) <= finest*1.0001) }

    /// The balls that meet and may not: each place to refine, `(curve, arc length, spacing)`, and
    /// the first such pair, or none when every ball keeps its distance. Beside a corner both
    /// share, straight to the finest spacing (halving one station at a time, the next round's
    /// stations stand elsewhere and the pair that met is a different pair). Elsewhere two curves
    /// merely pass close, and their balls are sized to the distance between them, however fine
    /// that is, down to `least`.
    fn meeting(&self,finest: f64,least: f64) -> (Vec<(usize,f64,f64)>,Option<(usize,usize)>) {
        let mut tight = Vec::new();
        let mut example = None;
        let later = self.candidates();
        for x in 0..self.balls.len() { for &y in &later[x] {
            let reach = self.balls[x].1+self.balls[y].1;
            if dist2(self.balls[x].0,self.balls[y].0) >= reach*reach || self.may_meet(x,y) { continue; }
            if self.fine(x,finest) && self.fine(y,finest) && self.beside(x,y) { continue; }
            let d = dist2(self.balls[x].0,self.balls[y].0).sqrt();
            let target = if self.beside(x,y) { finest } else { (0.9*d/(2.*BALL)).max(least) };
            for &(k,j) in self.places[x].iter().chain(&self.places[y]) {
                if self.gap(k,j) > target*1.0001 { tight.push((k,self.stations[k][j],target)); }
            }
            example.get_or_insert((x,y));
        } }
        (tight,example)
    }

    /// For each ball, the later balls (in ball order) that could meet it: those in the grid
    /// cells about it, the cells as wide as the two largest balls' reach, so a pair that meets is
    /// always among them. What `meeting` walks in place of every pair.
    fn candidates(&self) -> Vec<Vec<usize>> {
        let n = self.balls.len();
        let largest = self.balls.iter().map(|b| b.1).fold(0.,f64::max);
        if !(largest > 0.) || !largest.is_finite() { return (0..n).map(|x| (x+1..n).collect()).collect(); }
        // a hair wider than the reach, so a rounded quotient cannot put a meeting pair two cells apart
        let mut grid = Grid::new(2.*largest*1.0001);
        for (b,ball) in self.balls.iter().enumerate() { grid.insert(ball.0,b as u32); }
        (0..n).map(|x| {
            let mut later = Vec::new();
            grid.around(self.balls[x].0,|y| if y as usize > x { later.push(y as usize); });
            later.sort_unstable();
            later
        }).collect()
    }

    /// Where each ball stands, for a later shrink (`Place`).
    fn owners(&self) -> Vec<Vec<Place>> {
        self.places.iter().map(|p| p.iter().map(|&(k,j)| Place {curve:k,s:self.stations[k][j],gap:self.gap(k,j)}).collect()).collect()
    }

    /// Why balls `x` and `y` could not be kept apart.
    fn refusal(&self,x: usize,y: usize,finest: f64,round: usize) -> String {
        let gaps = |b: usize| self.places[b].iter().map(|&(k,j)| self.gap(k,j)).collect::<Vec<_>>();
        format!("the feature curves could not be protected: balls at {:?} and {:?} (places {:?} and {:?}) \
            still meet, finest {finest:.2e} (radii {:.3e} and {:.3e}, gaps {:?} and {:?}, round {round})",
            self.balls[x].0,self.balls[y].0,self.places[x],self.places[y],self.balls[x].1,self.balls[y].1,gaps(x),gaps(y))
    }
}

/// Protecting balls along the curves: `(centre, radius)` and where each stands, with corners
/// (curve ends, and ends shared by curves) once each. A closed curve repeats its first point at
/// its end. Where balls neither consecutive on one curve nor sharing a corner meet, the curves
/// are refined there (`Protection::meeting`), a round at a time.
pub(super) fn protect(curves: &[Vec<P>],edge_size: f64,sizing: &mut [Sizing]) -> Result<(Vec<(P,f64)>,Vec<Vec<Place>>),String> {
    let curves = Curves::new(curves);
    let (finest,least) = (edge_size/8.,edge_size/LEAST);
    for round in 0..16 {
        let p = Protection::new(&curves,sizing,edge_size);
        let (tight,example) = p.meeting(finest,least);
        let Some((x,y)) = example else { return Ok((p.balls.clone(),p.owners())); };
        if tight.is_empty() || round == 15 { return Err(p.refusal(x,y,finest,round)); }
        for (k,s,h) in tight { sizing[k].local.push((s,h)); }
    }
    unreachable!()
}

/// Halve the feature spacing at the places of balls `blocking`, down to `least`: whether any
/// could shrink.
pub(super) fn shrink(r: &Refiner,blocking: &[usize],sizing: &mut [Sizing],least: f64) -> bool {
    let mut blocking = blocking.to_vec();
    blocking.sort_unstable();
    blocking.dedup();
    let mut shrunk = false;
    for &b in &blocking {
        for place in &r.owners[b] {
            if place.gap > least*1.0001 { sizing[place.curve].local.push((place.s,(place.gap*0.5).max(least))); shrunk = true; }
        }
    }
    shrunk
}
