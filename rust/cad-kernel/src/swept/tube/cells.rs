//! **The cells a face's surface is fitted over** (#880 T1f): each face of a sheet or of the collar
//! lies on nets each regular over its whole rectangle. A chart's map folds back through its
//! cuspidal edges (`Tube::side` nought) and, on a sheet, is singular at the pinch: a net over a
//! window holding either carries layers a hair apart with opposite normals, and a surface read by
//! its nearest foot (the Boolean's) jumps between them. So a face is covered by **cells**, each a
//! part of the chart where it bounds the sweep throughout, read as a rectangle:
//! * a **box** of the chart's parameters, where the chart bounds the sweep everywhere in it;
//! * a **slab**: a run of lines across one parameter (columns of constant `θ`, or rows of constant
//!   `t`), each line from one side to the other, a side either a constant or the first place along
//!   the line where the chart stops bounding the sweep — its cuspidal edge or the pinch's rim
//!   (`Stop`) — read where that curve is a graph over the run with its slope bounded in the chart's
//!   metric (`STEEPEST`: of the two ways across, one the curve leans less to). A face touches a
//!   cuspidal edge only at isolated points (a swallowtail point, a cuspidal edge's corner on a rim)
//!   and the pinch's rim along arcs: there the cell's side is that curve, so a cuspidal edge is only
//!   ever a net's boundary row and no net enters the pinch's hole.
//!
//! The cells are found by splitting the face's box (`cover`): a part of it that is neither a box nor a
//! slab is split across one parameter, where the lines read change what they meet (the breaks), the
//! one nearest the middle, or else at the middle of its longer way in space; each split's line is a
//! seam of the face. Where the chart stops is read from its cuspidal edges and the pinch's rim as
//! traced (`edges`, `Tube::hole`), a line's crossings of them its places of change, and each run of
//! a line between two read by the chart at its middle — so an edge closer to a line than the
//! lines' spacing is not missed; each cell's side is then solved on each line by a bracketed root
//! of the curve's own field.
use super::{Chart,Tube};
use crate::util::space::{distance,norm};
use std::cell::RefCell;
use std::collections::BTreeMap;

type P = [f64;2];

/// Lines a part of a face is read on, each way (structural: as `fit_net`'s first grid, the part's
/// breaks found to a sixty-fourth of its run, each then split again as it asks).
const LINES: usize = 64;
/// The steepest a cell's side may lean against its run, in the chart's metric (structural: a curve
/// leans at most one way or the other at any point, so twice that leaves either way a margin, and
/// a curve at forty-five degrees a slab both ways).
const STEEPEST: f64 = 4.;
/// How deep the splits of a face's box may go before the face is refused (structural: each split
/// shortens the part's run by at least the lines' spacing, so forty is below the rounding of any
/// part of a turn).
const DEEPEST: usize = 40;

/// Which way a cell's lines run: `Columns` lines of constant first parameter (`θ`, or the collar's
/// `ψ`), the cell's sides across them read along the second; `Rows` lines of constant second.
#[derive(Clone,Copy,Debug,PartialEq)]
pub enum Axis { Columns,Rows }

impl Axis {
    /// The coordinate a line is read at (the run's), and the one along the line (across the run).
    pub fn along(self) -> usize { match self { Axis::Columns => 0,Axis::Rows => 1 } }
    pub fn across(self) -> usize { 1-self.along() }
    /// The chart's point at `x` of the run and `y` across it.
    pub fn point(self,x: f64,y: f64) -> P { match self { Axis::Columns => [x,y],Axis::Rows => [y,x] } }
}

/// Where a chart stops bounding the sweep: its own side's zero set (`Fold`, `Tube::side`: a sheet's
/// cuspidal edge, the collar's), or the pinch's rim on a sheet (`Hole`, `Pinch::side`).
#[derive(Clone,Copy,Debug,PartialEq)]
pub enum Stop { Fold,Hole }

/// A side of a cell across its lines: a constant, or where the chart stops (`Stop`), first met from
/// the cell's inside, read on each line by a bracketed root about its value interpolated from those
/// read on the run's lines (`at`, ordered along the run).
#[derive(Clone,Debug)]
pub enum Bound { At(f64),Stop { stop: Stop,at: Vec<P> } }

/// **A cell**: its chart, the way its lines run, the run's extent, and its two sides across the
/// lines (`lo` below, `hi` above).
#[derive(Clone,Debug)]
pub struct Cell { pub chart: Chart,pub axis: Axis,pub run: [f64;2],pub lo: Bound,pub hi: Bound }

impl Tube<'_> {
    /// **Where the chart stops at `uv`, as a field**: positive where it bounds the sweep; the chart's
    /// side for `Fold` (none read as stopped), the pinch's rim's for `Hole`.
    fn stop(&self,chart: Chart,stop: Stop,uv: P) -> f64 {
        match stop {
            Stop::Fold => self.side(chart,uv).unwrap_or(-1.),
            Stop::Hole => self.pinch.as_ref().map_or(1.,|p| p.side(uv)),
        }
    }

    /// Whether the chart bounds the sweep at `uv`: its side above nought and, on a sheet, outside
    /// the pinch's hole.
    fn bounds_at(&self,chart: Chart,uv: P) -> bool {
        self.stop(chart,Stop::Fold,uv) > 0. && (!matches!(chart,Chart::Sheet(_)) || self.stop(chart,Stop::Hole,uv) > 0.)
    }

    /// **The chart's rates' lengths at `uv`, regular through its cuspidal edges**: a sheet's
    /// spine's (as `Edge` reads its metric), the collar's own way round the tube and rate in `ρ`.
    pub fn rates_at(&self,chart: Chart,uv: P) -> Option<[f64;2]> {
        match chart {
            Chart::Sheet(_) => self.spine(uv[0],uv[1]).map(|s| [norm(s.s_th),norm(s.s_t)]),
            Chart::Collar => self.collar_metric(uv[0],uv[1]).map(|(_,a,b)| [norm(a),norm(b)]),
            Chart::End(_) => self.at(chart,uv).map(|p| [norm(p.x_th),norm(p.x_t)]),
        }
    }
}

impl Bound {
    /// **The side's value on the line at `x`**, `inside` a value across the line within the cell
    /// beside it: a constant's own; a stop's root of its field (`Tube::stop`) along the line, bracketed
    /// from its value interpolated from the run's (the bracket doubled from `step` until the field
    /// changes sign beside it, the inside positive) and halved by false position to the rounding.
    pub fn value(&self,tube: &Tube,chart: Chart,axis: Axis,x: f64,step: f64) -> Option<f64> {
        let Bound::Stop {stop,at} = self else { let Bound::At(c) = self else { unreachable!() }; return Some(*c) };
        let n = at.len();
        let j = at.partition_point(|q| q[0] < x).clamp(1,n-1);
        let (p,q) = (at[j-1],at[j]);
        let guess = if q[0] > p[0] { p[1]+(q[1]-p[1])*(x-p[0])/(q[0]-p[0]) } else { p[1] };
        root(&|y: f64| tube.stop(chart,*stop,axis.point(x,y)),guess,step)
    }
}

/// **A root of `f` beside `guess`**, its inside end (where `f` is positive): bracketed by doubling
/// from `step` either way until `f` changes sign across the bracket, then halved by false position
/// (Illinois's) to the rounding. None where no bracket is found in sixty-four doublings.
fn root(f: &dyn Fn(f64) -> f64,guess: f64,step: f64) -> Option<f64> {
    let (mut a,mut b) = (guess,guess);
    let mut h = step;
    let (mut fa,mut fb) = (f(a),f(b));
    let mut tries = 0;
    while (fa > 0.) == (fb > 0.) {
        a = guess-h; b = guess+h;
        fa = f(a); fb = f(b);
        h *= 2.;
        tries += 1;
        if tries > 64 { return None }
    }
    // (false position, Illinois's, while it shrinks the bracket; then halving to the rounding — a
    // false position stalled at one end leaves the bracket wide)
    let mut side = 0;
    for _ in 0..64 {
        let m = (a*fb-b*fa)/(fb-fa);
        if !(m > a.min(b) && m < a.max(b)) { break }
        let fm = f(m);
        if fm == 0. { return Some(m) }
        if (fm > 0.) == (fa > 0.) { a = m; fa = fm; if side == -1 { fb *= 0.5 } side = -1 }
        else { b = m; fb = fm; if side == 1 { fa *= 0.5 } side = 1 }
        if (b-a).abs() <= 4.*f64::EPSILON*a.abs().max(b.abs()) { break }
    }
    loop {
        let m = 0.5*(a+b);
        if !(m > a.min(b) && m < a.max(b)) { break }
        let fm = f(m);
        if fm == 0. { return Some(m) }
        if (fm > 0.) == (fa > 0.) { a = m; fa = fm } else { b = m }
    }
    Some(if fa > 0. { a } else { b })
}

impl Cell {
    /// The two sides' values on the line at `x`, cached per line read.
    fn sides(&self,tube: &Tube,x: f64,step: f64,cache: &RefCell<BTreeMap<u64,Option<[f64;2]>>>) -> Option<[f64;2]> {
        if let Some(s) = cache.borrow().get(&x.to_bits()) { return *s }
        let s = (|| Some([self.lo.value(tube,self.chart,self.axis,x,step)?,self.hi.value(tube,self.chart,self.axis,x,step)?]))();
        cache.borrow_mut().insert(x.to_bits(),s);
        s
    }

    /// **The chart's place at `(u, v)` of the unit square**: `u` along the chart's first parameter,
    /// `v` its second — the run's parameter linear, the other between the two sides on its line.
    pub fn place(&self,tube: &Tube,uv: P,step: f64,cache: &RefCell<BTreeMap<u64,Option<[f64;2]>>>) -> Option<P> {
        let (s,w) = match self.axis { Axis::Columns => (uv[0],uv[1]),Axis::Rows => (uv[1],uv[0]) };
        let x = self.run[0]+(self.run[1]-self.run[0])*s;
        let [lo,hi] = self.sides(tube,x,step,cache)?;
        Some(self.axis.point(x,lo+(hi-lo)*w))
    }

    /// **The unit square's `(u, v)` of the chart's place `q`** (`place`'s inverse).
    pub fn unit(&self,tube: &Tube,q: P,step: f64,cache: &RefCell<BTreeMap<u64,Option<[f64;2]>>>) -> Option<P> {
        let (x,y) = (q[self.axis.along()],q[self.axis.across()]);
        let s = (x-self.run[0])/(self.run[1]-self.run[0]);
        let [lo,hi] = self.sides(tube,x,step,cache)?;
        let w = (y-lo)/(hi-lo);
        Some(match self.axis { Axis::Columns => [s,w],Axis::Rows => [w,s] })
    }

    /// Whether the cell is a box of the chart's parameters (both sides constants): its places then
    /// affine in the square's.
    pub fn boxed(&self) -> bool { matches!((&self.lo,&self.hi),(Bound::At(_),Bound::At(_))) }

    /// **The cell's net**, its chart's point through `place` fitted as a map over the unit square
    /// (`nurbs::fit_net`) to `bar`; none where it is not read or not fitted within it.
    pub fn net(&self,tube: &Tube,step: f64,bar: f64) -> Option<(crate::nurbs::Net,f64)> {
        let cache = RefCell::new(BTreeMap::new());
        let f = |u: f64,v: f64| self.place(tube,[u,v],step,&cache).and_then(|q| tube.at(self.chart,q)).map_or([f64::NAN;3],|p| p.x);
        if std::env::var_os("SOLVENT_TUBE_FAIL").is_some() {
            let mut bad = Vec::new();
            for i in 0..=16 { let u = i as f64/16.; let x = self.run[0]+(self.run[1]-self.run[0])*u;
                let sd = self.sides(tube,x,step,&cache);
                let ok = (0..=8).all(|j| f(if self.axis == Axis::Columns { u } else { j as f64/8. },if self.axis == Axis::Columns { j as f64/8. } else { u })[0].is_finite());
                bad.push((x,sd,ok)) }
            eprintln!("NET {:?} run {:?}: {bad:?}",self.axis,self.run);
            let r = crate::nurbs::fit_net(&f,super::faces::DEGREE,super::faces::DEGREE,32,16,bar);
            eprintln!("  fit {:?}",r.as_ref().map(|r| (r.1,r.0.poles.len(),r.0.poles[0].len())));
        }
        let (net,err) = crate::nurbs::fit_net(&f,super::faces::DEGREE,super::faces::DEGREE,32,16,bar)?;
        (err <= bar && net.poles.iter().flatten().flatten().all(|x| x.is_finite())).then_some((net,err))
    }
}

/// **Where the chart stops, as traced**: each curve a polyline in the chart's parameters placed in
/// the window (an angle moved by whole turns to meet it), and what it is; and the window's extent
/// (its first parameter's, and the chart's second).
pub struct Stops { pub lines: Vec<(Vec<P>,Stop)>,pub limits: [[f64;2];2] }

impl Stops {
    /// The chart's cuspidal edges (`folds`) and, on a sheet, the pinch's rim (`Tube::hole`), each
    /// placed in the window `(w0, w1)` of the chart's first parameter.
    pub fn new(tube: &Tube,chart: Chart,folds: &[Vec<P>],(w0,w1): (f64,f64)) -> Result<Stops,super::Unbuilt> {
        let tau = std::f64::consts::TAU;
        let mut lines = Vec::new();
        let mut put = |l: &Vec<P>,stop: Stop| {
            let (lo,hi) = l.iter().fold((f64::INFINITY,f64::NEG_INFINITY),|(a,b),p| (a.min(p[0]),b.max(p[0])));
            for k in -2..=2 {
                let s = k as f64*tau;
                if hi+s >= w0 && lo+s <= w1 { lines.push((l.iter().map(|p| [p[0]+s,p[1]]).collect(),stop)) }
            }
        };
        for l in folds { if l.len() >= 2 { put(l,Stop::Fold) } }
        for l in tube.hole(chart)? { put(&l,Stop::Hole) }
        Ok(Stops {lines,limits:[[w0,w1],tube.rect(chart)[1]]})
    }
}

/// A crossing of a line with a polyline: its place across the line, which polyline, which segment,
/// and the stop it lies on (a stop's own, or a face's edge on the pinch's rim).
#[derive(Clone,Copy,Debug)]
struct Crossing { y: f64,line: usize,seg: usize,stop: Option<Stop> }

/// The crossings of the line at `x` (run's coordinate `axis.along()`) with the polylines, each segment
/// read half-open so a vertex on the line is crossed once.
fn crossings(polys: &[&[P]],axis: Axis,x: f64) -> Vec<Crossing> {
    let (i,j) = (axis.along(),axis.across());
    let mut out = Vec::new();
    for (k,l) in polys.iter().enumerate() {
        for s in 0..l.len().saturating_sub(1) {
            let (p,q) = (l[s],l[s+1]);
            if (p[i] <= x) != (q[i] <= x) {
                let f = (x-p[i])/(q[i]-p[i]);
                out.push(Crossing {y:p[j]+(q[j]-p[j])*f,line:k,seg:s,stop:None});
            }
        }
    }
    out.sort_by(|a,b| a.y.total_cmp(&b.y));
    out
}

/// **A face's part as read on one line**: the face's extent along it (none where the line misses
/// it), and the stops nearest it either way — read out to the window's extent, beyond the part's
/// box — each with the polyline and segment it was met on; `blocked` where the chart stops inside
/// the extent (by more than the tolerance across).
#[derive(Clone,Debug)]
struct Read { x: f64,face: Option<[f64;2]>,lo: Option<(f64,Stop,usize,usize)>,hi: Option<(f64,Stop,usize,usize)>,blocked: bool }

/// **How an edge of a face is read exactly**, a segment of one of its rings at a time (the ring, the
/// segment from its point, the hold): its own curve's point at a fraction along the segment
/// (`At`), or where it meets the line of coordinate `0` or `1` at a value (`Line`) — an arc's own
/// curve (a join's normal form by its parameter, the pinch's rim by the collar's angle, a traced run
/// settled on its double curve with the line's parameter held), never the chord. None where the
/// segment's curve does not meet the line. The ring's polygon proposes the segment; the curve
/// decides.
#[derive(Clone,Copy,Debug)]
pub enum Hold { At(f64),Line(usize,f64) }

/// The context a face's cells are found in: the tube and chart, the face's rings (outer first) placed
/// in the window, their edges read exactly (`Hold`) and the stop each lies on (`lies`: an edge on the
/// pinch's rim is the hole's stop itself), where the chart stops, the tolerance, the bar a cell's net
/// is fitted to and the step its sides' roots are bracketed from.
pub struct Ask<'a> { pub tube: &'a Tube<'a>,pub chart: Chart,pub rings: Vec<Vec<P>>,pub edge: &'a dyn Fn(usize,usize,Hold) -> Option<P>,
    pub lies: &'a dyn Fn(usize,usize) -> Option<Stop>,pub stops: &'a Stops,pub tol: f64,pub bar: f64,pub step: f64 }

/// **A face's cells**: each leaf of the splits of its box, the part of the box it covers (`rect`) and
/// its cell; and the splits, each the line it was split at (its axis, its value, the part's extent
/// across), in the order made (a split's line ends on the box or on an earlier split's).
pub struct Cover { pub leaves: Vec<([[f64;2];2],Cell)>,pub splits: Vec<(Axis,f64,[f64;2])> }

impl Ask<'_> {
    /// **The box of the face's part inside `rect`**, read on its edges' own curves: each segment of
    /// its rings whose chord or curve meets the rect, its ends within the rect, the curve's crossings
    /// of the rect's sides within the rect's extent, and the curve's extremes inside the segment (a
    /// curve bowing past its chord) within the rect; and the rect's corners inside the face. None where
    /// it holds none of the face. So a sliver between a chord and its curve is no part: a part is
    /// where the face's edges are, as every line reads them.
    fn part(&self,rect: [[f64;2];2]) -> Option<[[f64;2];2]> {
        let mut b = [[f64::INFINITY,f64::NEG_INFINITY];2];
        let inside = |p: P| (0..2).all(|k| p[k] >= rect[k][0] && p[k] <= rect[k][1]);
        let mut put = |p: P| if inside(p) { for k in 0..2 { b[k] = [b[k][0].min(p[k]),b[k][1].max(p[k])] } };
        for (r,ring) in self.rings.iter().enumerate() {
            let n = ring.len();
            for s in 0..n {
                let (p,q) = (ring[s],ring[(s+1)%n]);
                // (the segment's curve's middle: how far it bows from its chord, which widens the
                // chord's box for the question whether the segment meets the rect at all)
                let lo = [p[0].min(q[0]),p[1].min(q[1])];
                let hi = [p[0].max(q[0]),p[1].max(q[1])];
                let near = (0..2).all(|k| hi[k] >= rect[k][0] && lo[k] <= rect[k][1]);
                let m = if near { (self.edge)(r,s,Hold::At(0.5)) } else {
                    // (a chord's box apart from the rect by more than its length cannot hold its
                    // curve's bow — a segment is a short run of a smooth curve)
                    let len = (hi[0]-lo[0]).max(hi[1]-lo[1]);
                    if (0..2).any(|k| lo[k] > rect[k][1]+len || hi[k] < rect[k][0]-len) { continue }
                    (self.edge)(r,s,Hold::At(0.5))
                };
                put(p);
                put(q);
                for k in 0..2 {
                    for v in rect[k] {
                        if !v.is_finite() { continue }
                        let crossed = (p[k] <= v) != (q[k] <= v) || m.is_some_and(|m| (m[k] <= v) != (p[k] <= v));
                        if !crossed { continue }
                        if let Some(c) = (self.edge)(r,s,Hold::Line(k,v)) { let mut c = c; c[k] = v; put(c) }
                    }
                }
                // (each coordinate's extreme along the segment where the curve's middle stands past
                // both its ends: golden section on the curve's fraction)
                let Some(m) = m else { continue };
                put(m);
                for k in 0..2 {
                    let sign = if m[k] > p[k].max(q[k]) { 1. } else if m[k] < p[k].min(q[k]) { -1. } else { continue };
                    let g = |f: f64| (self.edge)(r,s,Hold::At(f)).map_or(f64::NEG_INFINITY,|x| sign*x[k]);
                    let (mut a,mut e) = (0f64,1f64);
                    let phi = 0.5*(5f64.sqrt()-1.);
                    let (mut c,mut d) = (e-phi*(e-a),a+phi*(e-a));
                    let (mut gc,mut gd) = (g(c),g(d));
                    while e-a > 1e-12 {
                        if gc > gd { e = d; d = c; gd = gc; c = e-phi*(e-a); gc = g(c) }
                        else { a = c; c = d; gc = gd; d = a+phi*(e-a); gd = g(d) }
                    }
                    if let Some(x) = (self.edge)(r,s,Hold::At(0.5*(a+e))) { put(x) }
                }
            }
        }
        // (and the box's corners inside the face: where it fills the box to a corner, no ring meets
        // the box's sides there)
        if rect.iter().flatten().all(|x| x.is_finite()) {
            for c in [[rect[0][0],rect[1][0]],[rect[0][1],rect[1][0]],[rect[0][0],rect[1][1]],[rect[0][1],rect[1][1]]] {
                if self.holds(c) { for k in 0..2 { b[k] = [b[k][0].min(c[k]),b[k][1].max(c[k])] } }
            }
        }
        (b[0][0] < b[0][1] && b[1][0] < b[1][1]).then_some(b)
    }

    /// Whether the face holds `c`: its rings' crossings of the row through `c`, each read on its
    /// curve, odd to its right.
    fn holds(&self,c: P) -> bool {
        let closed: Vec<Vec<P>> = self.rings.iter().map(|r| { let mut c = r.clone(); c.push(r[0]); c }).collect();
        let cs = crossings(&closed.iter().map(|c| &c[..]).collect::<Vec<_>>(),Axis::Rows,c[1]);
        cs.iter().filter(|x| self.on_curve(x,Axis::Rows,c[1]) > c[0]).count()%2 == 1
    }

    /// A ring's crossing of the line at `x` read on its edge's own curve (`edge`); the chord's where
    /// the curve does not meet the line within the segment (the chord crosses, the curve's ends
    /// straddle it: only a failure of the reading itself).
    fn on_curve(&self,c: &Crossing,axis: Axis,x: f64) -> f64 {
        (self.edge)(c.line,c.seg,Hold::Line(axis.along(),x)).map_or(c.y,|p| p[axis.across()])
    }

    /// A stop's crossing of the line at `x` settled on its own field (`Tube::stop`) beside the
    /// polyline's.
    fn stop_on(&self,c: &Crossing,axis: Axis,x: f64) -> f64 {
        let stop = self.stops.lines[c.line].1;
        let m = self.tube.rates_at(self.chart,axis.point(x,c.y)).map_or(1.,|m| m[axis.across()].max(f64::MIN_POSITIVE));
        root(&|y: f64| self.tube.stop(self.chart,stop,axis.point(x,y)),c.y,self.tol/m).unwrap_or(c.y)
    }

    /// **The part `b` read on the line at `x`** (`Read`).
    fn read(&self,b: [[f64;2];2],axis: Axis,x: f64) -> Read {
        let (lo,hi) = (b[axis.across()][0],b[axis.across()][1]);
        let [lim0,lim1] = self.stops.limits[axis.across()];
        let (lim0,lim1) = (lim0.min(lo),lim1.max(hi));
        // (the rings closed for the crossing)
        let closed: Vec<Vec<P>> = self.rings.iter().map(|r| { let mut c = r.clone(); c.push(r[0]); c }).collect();
        let mut cs = crossings(&closed.iter().map(|c| &c[..]).collect::<Vec<_>>(),axis,x);
        for c in cs.iter_mut() { c.y = self.on_curve(c,axis,x); c.stop = (self.lies)(c.line,c.seg) }
        cs.sort_by(|a,b| a.y.total_cmp(&b.y));
        // (the face's runs on the line — between its crossings, even and odd — clipped to the box,
        // and their extent: a run beyond the box is another part's)
        let face = cs.chunks(2).filter(|w| w.len() == 2).map(|w| [w[0].y.max(lo),w[1].y.min(hi)]).filter(|w| w[0] < w[1])
            .fold(None,|e: Option<[f64;2]>,w| Some(e.map_or(w,|e| [e[0].min(w[0]),e[1].max(w[1])])));
        if std::env::var_os("SOLVENT_TUBE_CROSS").is_some() { eprintln!("    crossings at {x}: {:?}",cs.iter().map(|c| (c.y,c.line,c.seg)).collect::<Vec<_>>()) }
        let polys: Vec<&[P]> = self.stops.lines.iter().map(|(l,_)| &l[..]).collect();
        let mut stops: Vec<Crossing> = crossings(&polys,axis,x);
        for c in stops.iter_mut() { c.y = self.stop_on(c,axis,x); c.stop = Some(self.stops.lines[c.line].1) }
        // (a face's edge on a stop is that stop where the line meets it, on the edge's own curve:
        // the stop's polyline's crossing beside it, within the tolerance in space, is the same
        // point — and a line passing between the stop's chord and its curve, where it turns,
        // crosses the edge all the same)
        let space = |y: f64| self.tube.at(self.chart,axis.point(x,y)).map(|p| p.x);
        let own: Vec<Crossing> = cs.iter().filter(|c| c.stop.is_some()).map(|c| Crossing {line:usize::MAX,seg:0,..*c}).collect();
        stops.retain(|c| !own.iter().any(|o| o.stop == c.stop && match (space(o.y),space(c.y)) {
            (Some(p),Some(q)) => distance(p,q) <= self.tol,
            _ => false,
        }));
        stops.extend(own);
        stops.sort_by(|a,b| a.y.total_cmp(&b.y));
        stops.retain(|c| c.y > lim0 && c.y < lim1);
        // (each run between the crossings read at its middle)
        let mut cuts: Vec<f64> = vec![lim0];
        cuts.extend(stops.iter().map(|c| c.y));
        cuts.push(lim1);
        let stopped: Vec<bool> = cuts.windows(2).map(|w| !self.tube.bounds_at(self.chart,axis.point(x,0.5*(w[0]+w[1])))).collect();
        let mut r = Read {x,face,lo:None,hi:None,blocked:false};
        let Some([f0,f1]) = face else { return r };
        // (how far the face's end at `y` along the line stands in space from the stop it overlaps,
        // crossed at `c`: the chart's two points' distance — beside a swallowtail point the face's
        // join and the cuspidal edge are tangent to the kernel, along which the chart's map has no
        // rate, so two places apart in the parameters by more than their polylines resolve are one
        // point in space)
        let off = |c: &Crossing,y: f64| -> f64 {
            match (self.tube.at(self.chart,axis.point(x,c.y)),self.tube.at(self.chart,axis.point(x,y))) {
                (Some(p),Some(q)) => distance(p.x,q.x),
                _ => f64::INFINITY,
            }
        };
        for (k,&s) in stopped.iter().enumerate() {
            if !s { continue }
            let (a,e) = (cuts[k],cuts[k+1]);
            // (a run's ends inside the face's extent, each beside the face's end it overlaps within
            // the tolerance in space, read as at that end)
            let below = e <= f0 || (k+1 < cuts.len()-1 && a <= f0 && e < f1 && off(&stops[k],f0) <= self.tol);
            let above = a >= f1 || (k >= 1 && e >= f1 && a > f0 && off(&stops[k-1],f1) <= self.tol);
            if below {
                // (below the face: the highest such run's top, a crossing)
                if k+1 < cuts.len()-1 {
                    let c = stops[k];
                    if r.lo.is_none_or(|l| l.0 < e) { r.lo = Some((e,c.stop.expect("a stop's crossing"),c.line,c.seg)) }
                }
            } else if above {
                // (above it: the lowest such run's bottom)
                if r.hi.is_none() && k >= 1 { let c = stops[k-1]; r.hi = Some((a,c.stop.expect("a stop's crossing"),c.line,c.seg)) }
            } else { r.blocked = true }
        }
        r
    }

    /// **The cell the part `b` is, read on its lines along `axis`**, or the breaks between lines where
    /// what they read changes.
    fn slab(&self,b: [[f64;2];2],axis: Axis) -> Result<Cell,Vec<f64>> {
        let [x0,x1] = b[axis.along()];
        let reads: Vec<Read> = (0..LINES).map(|k| self.read(b,axis,x0+(x1-x0)*(k as f64+0.5)/LINES as f64)).collect();
        let mut breaks = Vec::new();
        if std::env::var_os("SOLVENT_TUBE_LINES").is_some() { for r in reads.iter().step_by(6) { eprintln!("  line {axis:?} {r:?}") } }
        let mid = |k: usize| 0.5*(reads[k].x+reads[k+1].x);
        let [b0,b1] = b[axis.across()];
        // (a side is the box's where no line meets a stop between the face and it, else the stops)
        let boxed = [reads.iter().all(|r| r.lo.is_none_or(|l| l.0 <= b0)),reads.iter().all(|r| r.hi.is_none_or(|h| h.0 >= b1))];
        // (where a side's stop enters the box: the box's side serves the lines beyond, the stop the
        // lines within — read first, so the stop's curve is not followed out of the box)
        let within = |r: &Read,side: usize| if side == 0 { r.lo.is_some_and(|l| l.0 > b0) } else { r.hi.is_some_and(|h| h.0 < b1) };
        for side in 0..2 {
            if boxed[side] { continue }
            for k in 0..LINES-1 { if within(&reads[k],side) != within(&reads[k+1],side) { breaks.push(mid(k)) } }
        }
        if !breaks.is_empty() { breaks.sort_by(f64::total_cmp); breaks.dedup(); return Err(breaks) }
        for k in 0..LINES-1 {
            let (r,s) = (&reads[k],&reads[k+1]);
            if r.face.is_some() != s.face.is_some() || r.blocked != s.blocked { breaks.push(mid(k)); continue }
            for (side,(p,q)) in [(r.lo,s.lo),(r.hi,s.hi)].into_iter().enumerate() {
                if boxed[side] { continue }
                match (p,q) {
                    (None,None) => {},
                    (Some(p),Some(q)) => {
                        // (one stop, its curve a graph over the run, its slope bounded)
                        let at = axis.point(0.5*(r.x+s.x),0.5*(p.0+q.0));
                        let m = self.tube.rates_at(self.chart,at).unwrap_or([1.,1.]);
                        let steep = (q.0-p.0).abs()*m[axis.across()] > STEEPEST*(s.x-r.x).abs()*m[axis.along()];
                        // (a curve traced in two pieces — the trace's parts meet at a half turn —
                        // is one, its points' places continuous across the meeting)
                        if p.1 != q.1 || steep { breaks.push(mid(k)) }
                    }
                    _ => breaks.push(mid(k)),
                }
            }
        }
        if !breaks.is_empty() { breaks.dedup(); return Err(breaks) }
        let on: Vec<&Read> = reads.iter().filter(|r| r.face.is_some()).collect();
        if on.is_empty() || on.iter().any(|r| r.blocked) {
            if std::env::var_os("SOLVENT_TUBE_FAIL").is_some() { eprintln!("  (empty {} blocked {:?})",on.is_empty(),reads.iter().filter(|r| r.blocked).take(2).collect::<Vec<_>>()) }
            return Err(Vec::new())
        }
        let side = |pick: &dyn Fn(&Read) -> Option<(f64,Stop,usize,usize)>,boxed: bool,end: f64| -> Option<Bound> {
            if boxed { return Some(Bound::At(end)) }
            let (_,stop,..) = pick(on[0])?;
            Some(Bound::Stop {stop,at:reads.iter().filter_map(|r| pick(r).map(|p| [r.x,p.0])).collect()})
        };
        let (Some(lo),Some(hi)) = (side(&|r| r.lo,boxed[0],b0),side(&|r| r.hi,boxed[1],b1)) else {
            if std::env::var_os("SOLVENT_TUBE_FAIL").is_some() { eprintln!("  (no side: {:?} {:?})",on[0].lo,on[0].hi) }
            return Err(Vec::new())
        };
        // (no stop met inside: every polyline's vertex within the run stands outside the cell, but
        // those of the very curves its sides follow)
        // (each polyline a side follows, and the segments of it the side's lines met)
        let own = |r: &dyn Fn(&Read) -> Option<(f64,Stop,usize,usize)>| -> Vec<(usize,usize,usize)> {
            let mut by: BTreeMap<usize,(usize,usize)> = BTreeMap::new();
            for (l,g) in reads.iter().filter_map(|x| r(x).map(|p| (p.2,p.3))) {
                let e = by.entry(l).or_insert((g,g));
                *e = (e.0.min(g),e.1.max(g));
            }
            by.into_iter().map(|(l,(a,b))| (l,a,b+1)).collect()
        };
        let skip: Vec<(usize,usize,usize)> = [(boxed[0],own(&|r: &Read| r.lo)),(boxed[1],own(&|r: &Read| r.hi))].into_iter()
            .filter(|(b,_)| !b).flat_map(|(_,o)| o).collect();
        let interp = |bd: &Bound,x: f64| -> f64 {
            match bd { Bound::At(c) => *c,Bound::Stop {at,..} => {
                let j = at.partition_point(|q| q[0] < x).clamp(1,at.len()-1);
                let (p,q) = (at[j-1],at[j]);
                if q[0] > p[0] { p[1]+(q[1]-p[1])*(x-p[0])/(q[0]-p[0]) } else { p[1] }
            } }
        };
        for (li,(l,_)) in self.stops.lines.iter().enumerate() {
            for (vi,p) in l.iter().enumerate() {
                let (x,y) = (p[axis.along()],p[axis.across()]);
                if !(x > x0 && x < x1) { continue }
                if skip.iter().any(|&(k,a,e)| k == li && vi >= a && vi <= e) { continue }
                if !(y > interp(&lo,x) && y < interp(&hi,x)) { continue }
                // (a vertex of a stop within the tolerance in space of a side, as the side is
                // settled on that line, is on the side: a stop's polyline and the side's curve read
                // one curve two ways)
                let step = self.tol/self.tube.rates_at(self.chart,*p).map_or(1.,|m| m[axis.across()].max(f64::MIN_POSITIVE));
                let at = |y: f64| self.tube.at(self.chart,axis.point(x,y)).map(|q| q.x);
                let near = [&lo,&hi].iter().any(|bd| match (bd.value(self.tube,self.chart,axis,x,step).and_then(at),at(y)) {
                    (Some(s),Some(v)) => distance(s,v) <= self.tol,
                    _ => false,
                });
                if !near { breaks.push(x) }
            }
        }
        if !breaks.is_empty() { breaks.sort_by(f64::total_cmp); breaks.dedup(); return Err(breaks) }
        // (the face inside the run lies between the sides: each vertex of its rings within the run
        // and each crossing of a ring's edge with the box's sides across, read on its own curve,
        // stands between them or within the tolerance in space of one — a sliver of the face
        // narrower than the lines' spacing, beside a swallowtail point, beyond a side, no line sees)
        let beyond = |p: P| -> bool {
            let (x,y) = (p[axis.along()],p[axis.across()]);
            if !(x > x0 && x < x1 && y >= b0 && y <= b1) { return false }
            let step = self.tol/self.tube.rates_at(self.chart,p).map_or(1.,|m| m[axis.across()].max(f64::MIN_POSITIVE));
            let (Some(l),Some(h)) = (lo.value(self.tube,self.chart,axis,x,step),hi.value(self.tube,self.chart,axis,x,step)) else { return false };
            if y >= l && y <= h { return false }
            let at = |y: f64| self.tube.at(self.chart,axis.point(x,y)).map(|q| q.x);
            let side = if y < l { l } else { h };
            match (at(side),at(y)) { (Some(s),Some(v)) => distance(s,v) > self.tol,_ => false }
        };
        for (r,ring) in self.rings.iter().enumerate() {
            let n = ring.len();
            for s in 0..n {
                let (p,q) = (ring[s],ring[(s+1)%n]);
                if beyond(p) { breaks.push(p[axis.along()]) }
                for v in [b0,b1] {
                    if (p[axis.across()] <= v) == (q[axis.across()] <= v) { continue }
                    if let Some(mut c) = (self.edge)(r,s,Hold::Line(axis.across(),v)) { c[axis.across()] = v; if beyond(c) { breaks.push(c[axis.along()]) } }
                }
            }
        }
        if !breaks.is_empty() { breaks.sort_by(f64::total_cmp); breaks.dedup(); return Err(breaks) }
        // (no side's curve turns at the run's end: its lines read it as a graph whose slope grows
        // without bound there between the last line and the end, which the slope read at the lines'
        // middles never sees — that way is no slab; the other is, the curve a graph over it there)
        let delta = 0.5*(x1-x0)/LINES as f64;
        for (bd,boxed) in [(&lo,boxed[0]),(&hi,boxed[1])] {
            if boxed { continue }
            for x in [x0,x1] {
                let Some(y) = bd.value(self.tube,self.chart,axis,x,self.step) else { continue };
                // (the turning point on this side: within the run's length of its end in space)
                let m = self.tube.rates_at(self.chart,axis.point(x,y)).unwrap_or([1.,1.]);
                let near = |t: &P| (t[axis.across()]-y).abs()*m[axis.across()] <= (x1-x0)*m[axis.along()];
                if self.turns(axis,x,delta).iter().any(|t| (t[axis.along()]-x).abs() <= delta && near(t)) { return Err(Vec::new()) }
            }
        }
        Ok(Cell {chart:self.chart,axis,run:[x0,x1],lo,hi})
    }

    /// **A stop's curve at the fraction `f` of its polyline's segment `s`**: the chord's point there
    /// settled on the stop's own field (`Tube::stop`) along the chord's normal — the polyline
    /// proposes, the field decides.
    fn stop_at(&self,li: usize,s: usize,f: f64) -> Option<P> {
        let (l,stop) = (&self.stops.lines[li].0,self.stops.lines[li].1);
        let (p,q) = (l[s],l[s+1]);
        let c = [p[0]+(q[0]-p[0])*f,p[1]+(q[1]-p[1])*f];
        let d = [q[0]-p[0],q[1]-p[1]];
        let len = d[0].hypot(d[1]);
        if !(len > 0.) { return Some(c) }
        let n = [-d[1]/len,d[0]/len];
        let m = self.tube.rates_at(self.chart,c).map_or(1.,|m| m[0].max(m[1]).max(f64::MIN_POSITIVE));
        let at = |w: f64| [c[0]+n[0]*w,c[1]+n[1]*w];
        root(&|w: f64| self.tube.stop(self.chart,stop,at(w)),0.,self.tol/m).map(at)
    }

    /// **Where each curve turns across the line at `x`** (run's coordinate along `axis`), within a
    /// segment of its polyline: each segment of the face's rings and of the stops whose curve
    /// (`edge`, `stop_at`) bows past both its ends along the run, its extreme there (golden section
    /// on the curve's fraction) where the line meets the curve twice within the segment — the
    /// extreme beyond the line from the segment's ends — or passes within `delta` of it. The
    /// polylines only propose: a line nearly tangent to a curve meets it twice inside one chord, and
    /// no count of the chords' crossings sees that.
    fn turns(&self,axis: Axis,x: f64,delta: f64) -> Vec<P> {
        let a = axis.along();
        let mut out = Vec::new();
        let mut look = |p: P,q: P,g: &dyn Fn(f64) -> Option<P>| {
            let (lo,hi) = (p[a].min(q[a]),p[a].max(q[a]));
            let len = (p[0]-q[0]).abs().max((p[1]-q[1]).abs());
            if x < lo-len-delta || x > hi+len+delta { return }
            let Some(m) = g(0.5) else { return };
            let sign = if m[a] > hi { 1. } else if m[a] < lo { -1. } else { return };
            let h = |f: f64| g(f).map_or(f64::NEG_INFINITY,|v| sign*v[a]);
            let (mut s,mut e) = (0f64,1f64);
            let phi = 0.5*(5f64.sqrt()-1.);
            let (mut c,mut d) = (e-phi*(e-s),s+phi*(e-s));
            let (mut hc,mut hd) = (h(c),h(d));
            while e-s > 1e-12 {
                if hc > hd { e = d; d = c; hd = hc; c = e-phi*(e-s); hc = h(c) }
                else { s = c; c = d; hc = hd; d = s+phi*(e-s); hd = h(d) }
            }
            let Some(t) = g(0.5*(s+e)) else { return };
            // (twice met: the extreme beyond the line, the ends this side of it)
            let end = if sign > 0. { hi } else { lo };
            let twice = sign*(t[a]-x) > 0. && sign*(x-end) > 0.;
            if twice || (t[a]-x).abs() <= delta { out.push(t) }
        };
        for (r,ring) in self.rings.iter().enumerate() {
            let n = ring.len();
            for s in 0..n { look(ring[s],ring[(s+1)%n],&|f| (self.edge)(r,s,Hold::At(f))) }
        }
        for (li,(l,_)) in self.stops.lines.iter().enumerate() {
            for s in 0..l.len().saturating_sub(1) { look(l[s],l[s+1],&|f| self.stop_at(li,s,f)) }
        }
        out
    }

    /// **Where a curve turns between the lines at `x − δ` and `x + δ`** (run's coordinate along
    /// `axis`), or meets the line at `x` twice within one segment of its polyline (`turns`): its
    /// turning point across the run, the nearest to the line. Else, where the face's rings and the
    /// stops cross the two lines a different number of times, a curve's two crossings meet between
    /// them at a vertex of its polyline — read as the middle of the nearest two crossings of one
    /// curve on the line that has them, each settled on its own curve. None where no curve turns
    /// there.
    fn turning(&self,axis: Axis,x: f64,delta: f64) -> Option<f64> {
        let (a,c) = (axis.along(),axis.across());
        if let Some(t) = self.turns(axis,x,delta).into_iter().min_by(|p,q| (p[a]-x).abs().total_cmp(&(q[a]-x).abs())) { return Some(t[c]) }
        let closed: Vec<Vec<P>> = self.rings.iter().map(|r| { let mut c = r.clone(); c.push(r[0]); c }).collect();
        let rings: Vec<&[P]> = closed.iter().map(|c| &c[..]).collect();
        let polys: Vec<&[P]> = self.stops.lines.iter().map(|(l,_)| &l[..]).collect();
        let count = |x: f64| crossings(&rings,axis,x).len()+crossings(&polys,axis,x).len();
        let (a,e) = (count(x-delta),count(x+delta));
        if a == e { return None }
        let at = if a > e { x-delta } else { x+delta };
        // (each crossing settled, with its source: a ring's, or a stop's)
        let mut cs: Vec<(f64,usize)> = crossings(&rings,axis,at).iter().map(|c| (self.on_curve(c,axis,at),c.line)).collect();
        cs.extend(crossings(&polys,axis,at).iter().map(|c| (self.stop_on(c,axis,at),usize::MAX-c.line)));
        cs.sort_by(|p,q| p.0.total_cmp(&q.0));
        cs.windows(2).filter(|w| w[0].1 == w[1].1).min_by(|p,q| (p[1].0-p[0].0).total_cmp(&(q[1].0-q[0].0))).map(|w| 0.5*(w[0].0+w[1].0))
    }

    /// **The face's cells** (`Cover`), from its box; refused by name where a part is split deeper
    /// than `DEEPEST` or is under the tolerance across in space and still neither a box nor a slab.
    pub fn cover(&self) -> Result<Cover,super::Unbuilt> {
        let mut out = Cover {leaves:Vec::new(),splits:Vec::new()};
        let big = [[f64::NEG_INFINITY,f64::INFINITY];2];
        let Some(b) = self.part(big) else { return Ok(out) };
        self.split(b,0,&mut out)?;
        Ok(out)
    }

    fn split(&self,rect: [[f64;2];2],depth: usize,out: &mut Cover) -> Result<(),super::Unbuilt> {
        let Some(b) = self.part(rect) else { return Ok(()) };
        let mut breaks: Vec<Vec<(Axis,f64)>> = Vec::new();
        for axis in [Axis::Columns,Axis::Rows] {
            // (the run the part's, across it the node's own extent: a side at the part's box would
            // move with each split, and a stop met beyond it be split toward without end)
            let mut c = b;
            c[axis.across()] = rect[axis.across()];
            // (a run's end within the tolerance in space of the node's side is that side: the
            // part's corner read from its polygon, its edge's end settled a rounding beyond)
            let (i,mid) = (axis.along(),[0.5*(b[0][0]+b[0][1]),0.5*(b[1][0]+b[1][1])]);
            let rate = self.tube.rates_at(self.chart,mid).map_or(0.,|m| m[i]);
            for k in 0..2 { if rect[i][k].is_finite() && (b[i][k]-rect[i][k]).abs()*rate <= self.tol { c[i][k] = rect[i][k] } }
            match self.slab(c,axis) {
                // (a cell is one its net fits within the bar: where the chart's map turns faster
                // than a net over the whole cell follows — beside the pinch, its spine's normal
                // turning round the umbrella's handle — the cell is split, as at its middle)
                Ok(cell) if cell.net(self.tube,self.step,self.bar).is_some() => { out.leaves.push((rect,cell)); return Ok(()) }
                Ok(_) => breaks.push(Vec::new()),
                Err(bs) => breaks.push(bs.into_iter().map(|x| (axis,x)).collect()),
            }
        }
        if std::env::var_os("SOLVENT_TUBE_BOTH").is_some() { eprintln!("both: depth {depth} rect {rect:?} part {b:?} {:?}",breaks.iter().map(|b| b.iter().map(|x| (x.0 == Axis::Columns,x.1)).collect::<Vec<_>>()).collect::<Vec<_>>()) }
        // (the way with the fewest breaks: the lines across it change least)
        let breaks: Vec<(Axis,f64)> = breaks.into_iter().filter(|b| !b.is_empty()).min_by_key(|b| b.len()).unwrap_or_default();
        let mid = [0.5*(b[0][0]+b[0][1]),0.5*(b[1][0]+b[1][1])];
        // (a break at a curve's turning point across the run — where an edge of the face or a stop
        // is tangent to the line — is split through that point the other way: a line tangent to a
        // curve leaves between them a sliver its lines read the face on only in part, and the
        // curve is a graph over the other way there)
        let breaks: Vec<(Axis,f64)> = breaks.into_iter().filter_map(|(axis,x)| {
            let [x0,x1] = b[axis.along()];
            let delta = 0.5*(x1-x0)/LINES as f64;
            match self.turning(axis,x,delta) {
                None => Some((axis,x)),
                Some(y) => {
                    let other = if axis == Axis::Columns { Axis::Rows } else { Axis::Columns };
                    let [y0,y1] = b[other.along()];
                    (y > y0 && y < y1).then_some((other,y))
                }
            }
        }).collect();
        if std::env::var_os("SOLVENT_TUBE_CELLS").is_some() { eprintln!("cells: {:?} depth {depth} part {b:?} breaks {breaks:?}",self.chart) }
        let m = self.tube.rates_at(self.chart,mid).unwrap_or([1.,1.]);
        let lengths = [(b[0][1]-b[0][0])*m[0],(b[1][1]-b[1][0])*m[1]];
        if depth >= DEEPEST || lengths[0].max(lengths[1]) <= self.tol {
            if std::env::var_os("SOLVENT_TUBE_FAIL").is_some() { for axis in [Axis::Columns,Axis::Rows] {
                let mut c = b; c[axis.across()] = rect[axis.across()];
                let [x0,x1] = c[axis.along()];
                eprintln!("FAIL rect {rect:?} part {b:?} axis {axis:?} -> {:?}",self.slab(c,axis).err());
                for k in (0..LINES).step_by(8) { eprintln!("  {:?}",self.read(c,axis,x0+(x1-x0)*(k as f64+0.5)/LINES as f64)) }
            } }
            return Err(super::Unbuilt::Failed(format!("the tube road: a face of {:?} about {mid:?} is covered by no regular cell",self.chart)))
        }
        // (of those, the break nearest its run's middle, as a part of the run)
        let (axis,x) = breaks.iter().copied().min_by(|p,q| {
            let off = |(a,x): (Axis,f64)| { let [x0,x1] = b[a.along()]; ((x-x0)/(x1-x0)-0.5).abs() };
            off(*p).total_cmp(&off(*q))
        }).unwrap_or(if lengths[0] >= lengths[1] { (Axis::Columns,mid[0]) } else { (Axis::Rows,mid[1]) });
        let i = axis.along();
        out.splits.push((axis,x,rect[axis.across()]));
        let mut below = rect; below[i][1] = x;
        let mut above = rect; above[i][0] = x;
        self.split(below,depth+1,out)?;
        self.split(above,depth+1,out)
    }
}

/// The distance in space between a cell's net's point and the chart's at the cell's place, worst
/// over a grid of `n` × `n` (a probe of a net against its chart).
pub fn worst(tube: &Tube,cell: &Cell,net: &crate::nurbs::Net,n: usize,step: f64) -> f64 {
    let cache = RefCell::new(BTreeMap::new());
    let mut w = 0f64;
    for i in 0..=n { for j in 0..=n {
        let uv = [i as f64/n as f64,j as f64/n as f64];
        let Some(x) = cell.place(tube,uv,step,&cache).and_then(|q| tube.at(cell.chart,q)) else { continue };
        w = w.max(distance(net.point(uv[0],uv[1]),x.x));
    } }
    w
}
