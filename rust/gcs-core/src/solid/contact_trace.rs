//! A cutter's contact curves under a generating sweep, and the candidate sheet they make: the
//! numerics of the native swept construction (docs/generating-sweeps.md), which a host feeds
//! with sections of its own cutter. A **station** is one section of the cutter by a meridian
//! half-plane, walked by augmented length (position length plus turning across convex corners);
//! the host samples it, a point and its outward normal at a walk length, and nothing here knows
//! how. What is here: every root of a sampled point's contact equation, a station's contact
//! curve traced through the plane of walk length and time (turning at a fold of the time chart,
//! where the envelope carries on though the walk turns back), and the sheet whose columns are
//! those curves, resampled, with a withheld contact at the centre of every cell (and at the
//! middles of its sides, for a sheet held to a tolerance and refined where its fit misses:
//! `Layout`, `Grid`, `marked`). Positions are native millimetres, `scale` of them a model unit;
//! the contact math runs in model units.
use super::SweepContacts;
use super::sweep_contacts::PointContactError;
use crate::space::{scale as scaled,distance};
use std::{cell::RefCell,collections::BTreeMap,f64::consts::TAU,fmt,sync::{Arc,Mutex}};

type V = [f64;3];

/// How far a contact's time may move in one step along a traced contact curve and still be
/// the same root (radians of the motion's parameter).
pub const STEP_TIME: f64 = 1.;
/// How many times a step along a steep contact curve is halved before the root is taken to end.
pub const STEEP_DEPTH: u32 = 8;
/// Target node spacing along the profile and along the band, in millimetres. A contact curve is
/// traced on a grid of a quarter row.
pub const ROW_SPACING: f64 = 0.15;
pub const COLUMN_SPACING: f64 = 0.5;

/// Why a station's contacts could not be read or traced.
#[derive(Clone,Debug)]
pub enum TraceError {
    /// No one contact time here: the contact equation at this sample is degenerate (in contact
    /// at every time, or only grazing), or the sample carries no direction. A traced root ends
    /// here; a scan passes over it.
    Degenerate(String),
    /// A station traced in the blank has no contact of its own there.
    Missed,
    Failed(String),
}

impl fmt::Display for TraceError {
    fn fmt(&self,f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            TraceError::Degenerate(m) | TraceError::Failed(m) => f.write_str(m),
            TraceError::Missed => f.write_str("no contact of this station lies in the blank"),
        }
    }
}

impl From<String> for TraceError { fn from(m: String) -> Self { TraceError::Failed(m) } }
impl From<&str> for TraceError { fn from(m: &str) -> Self { TraceError::Failed(m.into()) } }
impl From<TraceError> for String { fn from(e: TraceError) -> String { e.to_string() } }

/// A cutter surface point with its outward normal (native millimetres).
#[derive(Clone,Copy,Debug)]
pub struct Sample { pub position: V,pub normal: V }

/// One root of a cutter point's contact equation: where, the normal there, when, and which.
#[derive(Clone,Copy,Debug)]
pub struct Found { pub position: V,pub normal: V,pub time: f64,pub branch: usize }

/// A point of a traced contact curve: its walk length, its unfolded walk length (the row
/// coordinate, which equals the walk length on a curve without folds), and the contact.
#[derive(Clone,Copy,Debug)]
pub struct Point { pub s: f64,pub tau: f64,pub found: Found }

/// A traced contact curve, its points in order of unfolded walk length, and its anchor's index.
pub struct Traced { pub curve: Vec<Point>,pub anchor: usize }

/// How far a station's contact curve runs: out of the blank by the sheet's margin, or over an
/// interval of unfolded walk length; anchored in the blank, or beside a neighbour's anchor at
/// walk length `s` and time `time` when the station's contacts miss the blank.
#[derive(Clone,Copy)]
pub enum Extent { Blank, Span {lo: f64,hi: f64,beside: Option<(f64,f64)>} }

/// Whether points lie inside the blank, in native millimetres.
pub type Inside<'a> = &'a (dyn Fn(&[V]) -> Result<Vec<bool>,String>+Sync);

/// One section of the cutter as the host samples it: its augmented length (the period of its
/// walk), the window of walk length its contacts are anchored in, and its sampler.
pub struct Station<'a> {
    pub length: f64,
    pub window: [f64;2],
    /// Called from several threads at once: a station's columns are traced side by side.
    pub sample: Box<dyn Fn(f64) -> Result<Sample,TraceError>+Send+Sync+'a>,
}

/// How a host makes the station at an angle: called from several threads at once.
pub type StationAt<'t,'s> = dyn Fn(f64) -> Result<Station<'s>,TraceError>+Sync+'t;

/// The stations' band as a first coarse pass found it: its station angles, and the profile's
/// mean distance from the axis, for column spacing.
#[derive(Clone,Copy,Debug)]
pub struct Band { pub stations: [f64;2],pub radius: f64 }

/// Where a sheet's rows fall along each column's contact curve, between the same two unfolded
/// walk lengths at its ends. `Walk`: at even unfolded walk lengths, one grid for every column.
/// `Length`: at even lengths in space along each column's own curve. Where the envelope stretches
/// unevenly along the profile (a cutter's round generating a long fillet beside a flank whose
/// contacts crowd) and the stretch begins on different rows in different columns, walk-length rows
/// give the columns chord-length parameters their average cannot follow, and the fitted face pleats
/// between rows; length rows are even in every column. Where a column's curve runs far in space
/// outside the blank, length rows leave its rows in the blank sparse against its neighbours', and
/// walk-length rows are the ones that hold.
#[derive(Clone,Copy,Debug,PartialEq,Eq)]
pub enum Rows { Walk,Length }

/// A candidate sheet: contact positions on a row-major grid with the outward
/// normal of the cutter at each contact, in native millimetres.
#[derive(Debug)]
pub struct Sheet {
    pub points: Vec<V>,
    pub normals: Vec<V>,
    /// Each point's contact time.
    pub times: Vec<f64>,
    pub rows: usize,
    pub columns: usize,
    /// Contacts withheld from the fit (`Withheld`), with their normals.
    pub withheld: Vec<V>,
    pub withheld_normals: Vec<V>,
    /// Where each withheld contact is, in half rows and half columns: `[2r+1, 2c+1]` the centre
    /// of the cell after node `(r, c)`, `[2r+1, 2c]` the middle of column `c`'s step from row `r`,
    /// `[2r, 2c+1]` the middle of row `r`'s step from column `c`.
    pub sites: Vec<[usize;2]>,
}

impl Sheet {
    /// The chart contract: where the grid lies in the blank it samples one regular chart of the
    /// envelope. Down each column, the contact time does not leap (a root a turn away, or a
    /// root elsewhere taken for this one); the root itself may change, at a fold where the two
    /// merge, which the traced column walks through. Position is no test: near a fold a contact
    /// runs fast along the sweep, which is steep, not broken; whether the fit can follow it is
    /// the fit contract's question. Steps wholly
    /// outside the blank are the margin that carries the sheet's edge out, and bound nothing;
    /// a fault there that disturbs the fit inside is the fit contract's to find. The first
    /// violation, where it is.
    pub fn chart_fault(&self,inside: Inside) -> Result<Option<String>,String> {
        let within = inside(&self.points)?;
        let mut faults = Vec::new();
        for c in 0..self.columns {
            let at = |r: usize| r*self.columns+c;
            for r in 1..self.rows {
                if !within[at(r)] && !within[at(r-1)] { continue; }
                let p = self.points[at(r)].map(|x| (x*1e3).round()/1e3);
                if (self.times[at(r)]-self.times[at(r-1)]).abs() > 1. {
                    faults.push(format!("the contact time leaps from {:.3} to {:.3} between rows {} and {r} of column {c} at {p:?}",
                        self.times[at(r-1)],self.times[at(r)],r-1));
                }
            }
        }
        Ok(faults.first().map(|f| format!("{f} ({} such steps in the blank, over {} columns)",faults.len(),self.columns)))
    }

    /// The shortest side, in space, of the cells a withheld contact's site touches: the spacing
    /// of the nodes the fit's error there is spread between.
    pub fn spacing(&self,site: [usize;2]) -> f64 {
        let cells = |i: usize,n: usize| -> Vec<usize> {
            if i%2 == 1 { vec![i/2] } else { [(i/2).checked_sub(1),(i/2+1 < n).then_some(i/2)].into_iter().flatten().collect() }
        };
        let at = |r: usize,c: usize| self.points[r*self.columns+c];
        let mut least = f64::INFINITY;
        for r in cells(site[0],self.rows) { for c in cells(site[1],self.columns) {
            for d in [distance(at(r,c),at(r+1,c)),distance(at(r,c+1),at(r+1,c+1)),distance(at(r,c),at(r,c+1)),distance(at(r+1,c),at(r+1,c+1))] {
                least = least.min(d);
            }
        } }
        least
    }
}

/// The rows `[first, last]` a sheet keeps of its resampled columns, given each column's contact
/// times and whether each contact lies in the blank, row by row: every row holding a contact in
/// the blank, and outward from those, rows while no column's contact time leaps between
/// consecutive rows by `STEP_TIME` or more (the same root a turn away, or another root). A sheet
/// with no contact in the blank keeps every row.
pub fn charted(times: &[Vec<f64>],within: &[Vec<bool>]) -> [usize;2] {
    let rows = times.first().map_or(0,Vec::len);
    let held: Vec<usize> = (0..rows).filter(|&r| within.iter().any(|c| c[r])).collect();
    let (Some(&low),Some(&high)) = (held.first(),held.last()) else { return [0,rows.saturating_sub(1)] };
    let leaps = |r: usize| times.iter().any(|c| (c[r]-c[r-1]).abs() >= STEP_TIME);
    let first = (1..=low).rev().find(|&r| leaps(r)).unwrap_or(0);
    let last = (high+1..rows).find(|&r| leaps(r)).map_or(rows-1,|r| r-1);
    [first,last]
}

/// A walk along a contact curve so far: its points, its length in the blank, how far it has run outside
/// the blank since last inside, where it is, and its unfolded walk length.
struct Walked { out: Vec<Point>,within: f64,outside: f64,last: V,last_s: f64,tau: f64,sense: f64 }

impl Walked {
    /// One more point: `tau` moves by the walk length stepped, in the walk's sense from the
    /// anchor, so it keeps its direction through a fold where the walk length turns back.
    fn push(&mut self,s: f64,f: Found,inside: Inside) -> Result<(),String> {
        let step = distance(self.last,f.position);
        if inside(&[f.position])?[0] { self.outside = 0.; self.within += step; } else { self.outside += step; }
        self.tau += self.sense*(s-self.last_s).abs();
        self.last = f.position; self.last_s = s;
        self.out.push(Point {s,tau:self.tau,found:f});
        Ok(())
    }
}

/// The sweep, the blank and the units every station of one cutter is traced against.
/// `debug` prints where a station's contact curve ends, leaves the root window or runs away:
/// the instruments that located the faults of the traced construction, kept for the next one.
pub struct Tracer<'a> {
    pub sweep: &'a SweepContacts,
    pub scale: f64,
    pub inside: Inside<'a>,
    pub debug: bool,
}

impl Tracer<'_> {
    /// The motion interval a contact curve is followed over: the declared roll and a turn
    /// either side. A station's curve may run through times outside the declared roll where the
    /// sheet must still continue (those contacts lie on the envelope of a longer motion, and the
    /// classification decides what is exposed); a fixed window cut such curves off where their
    /// time crossed it, leaving a station ending inside the blank.
    pub fn wide(&self) -> [f64;2] { let d = self.sweep.domain(); [d[0]-TAU,d[1]+TAU] }

    /// Every root of a cutter point's contact equation over `interval`, as native positions
    /// with their normals, times and which root each is.
    pub fn contacts(&self,sample: Sample,interval: [f64;2]) -> Result<Vec<Found>,TraceError> {
        let scale = self.scale;
        let roots = self.sweep.at_point_normal_over(scaled(sample.position,1./scale),sample.normal,interval,1e-9)
            .map_err(|e| match e {
                // Constant zero (in contact at every time: a pole on the spin axis, or a revolution
                // about the motion's own axis) and a double root (a point grazing at one time) are
                // alike to the root finder; neither gives the per-point time this construction needs.
                PointContactError::Degenerate => TraceError::Degenerate(format!("a cutter point's contact equation is \
                    degenerate (in contact at every time, or only grazing) at {:?} with normal {:?}; the section \
                    construction needs one contact time per point",sample.position.map(|x| (x*1e4).round()/1e4),
                    sample.normal.map(|x| (x*1e4).round()/1e4))),
                PointContactError::DegenerateNormal => TraceError::Degenerate(e.to_string()),
                PointContactError::Failed(m) => TraceError::Failed(m),
            })?;
        Ok(roots.iter().map(|r| Found {position:scaled(r.contact.position,scale),normal:r.contact.normal,
            time:r.root.time,branch:r.root.branch}).collect())
    }

    /// The contact position and time of a cutter sample under the sweep, the root nearest `near`.
    pub fn nearest(&self,sample: Sample,interval: [f64;2],near: f64) -> Result<Option<(V,f64)>,TraceError> {
        Ok(self.contacts(sample,interval)?.into_iter().min_by(|x,y| (x.time-near).abs().total_cmp(&(y.time-near).abs()))
            .map(|f| (f.position,f.time)))
    }

    /// A station's contact curve: the zero set of the contact equation in the plane of walk
    /// length and time, traced from an anchor in both directions around the station's closed
    /// section loop. It follows one root with continuous time; where that root merges with the
    /// other (a fold of the tool's time chart, where the envelope carries on smoothly though the
    /// walk turns back) it turns onto the other root and reverses. How far it runs is decided
    /// along the curve, never by a window of walk length: for `Extent::Blank` the anchor is a
    /// contact in the blank within the declared roll found in the station's window, and each
    /// end runs until it has been outside the blank for `margin` millimetres; for
    /// `Extent::Span` (a station whose contacts miss the blank) the anchor is the contact
    /// nearest a neighbour's and each end runs the neighbour's lengths. A curve that comes back
    /// to its anchor inside the blank is refused. Returns the curve's points in order, with
    /// their walk lengths, and the anchor's place in it.
    pub fn trace(&self,station: &Station,margin: f64,extent: Extent) -> Result<Traced,TraceError> {
        let inside = self.inside;
        let declared = self.sweep.domain();
        let window = station.window;
        let wide = self.wide();
        let h = ROW_SPACING/4.;
        let total = station.length;
        let memo = RefCell::new(BTreeMap::<i64,Vec<Found>>::new());
        let roots_at = |s: f64| self.contacts((station.sample)(s)?,wide);
        let roots = |i: i64| -> Result<Vec<Found>,TraceError> {
            if let Some(r) = memo.borrow().get(&i) { return Ok(r.clone()); }
            let r = roots_at(i as f64*h)?;
            memo.borrow_mut().insert(i,r.clone());
            Ok(r)
        };
        // The anchor.
        let (a,anchor) = match extent {
            Extent::Blank | Extent::Span {beside:None,..} => {
                let (first,last) = ((window[0]/h).floor() as i64,(window[1]/h).ceil() as i64);
                let mut held = Vec::new();
                for i in first..=last {
                    let rs = roots(i)?;
                    let within: Vec<&Found> = rs.iter().filter(|f| f.time >= declared[0]-1e-9 && f.time <= declared[1]+1e-9).collect();
                    for (f,state) in within.iter().zip(inside(&within.iter().map(|f| f.position).collect::<Vec<_>>())?) {
                        if state { held.push((i,**f)); }
                    }
                }
                *held.get(held.len()/2).ok_or(TraceError::Missed)?
            }
            Extent::Span {beside:Some((s,time)),..} => {
                let i = (s/h).round() as i64;
                let near: Vec<(i64,Found)> = (i-8..=i+8).map(|k| roots(k).map(|rs| rs.into_iter().map(move |f| (k,f)).collect::<Vec<_>>()))
                    .collect::<Result<Vec<_>,_>>()?.into_iter().flatten().collect();
                near.into_iter().min_by(|x,y| ((x.0-i).abs() as f64*h+(x.1.time-time).abs()).total_cmp(&((y.0-i).abs() as f64*h+(y.1.time-time).abs())))
                    .ok_or("a station beside the blank has no contact near its neighbour's")?
            }
        };
        // The same root carried from `s0` to `s1`: within a radian of its time, since near a fold
        // the time runs steeply and a turn away (at least 2π over the motion's rate) is another
        // root; where one step moves it further, through halved steps.
        fn steep_from(roots_at: &dyn Fn(f64) -> Result<Vec<Found>,TraceError>,s0: f64,f: Found,s1: f64,depth: u32)
            -> Result<Option<Vec<(f64,Found)>>,TraceError> {
            let next = roots_at(s1)?.into_iter().filter(|g| g.branch == f.branch && (g.time-f.time).abs() < STEP_TIME)
                .min_by(|x,y| (x.time-f.time).abs().total_cmp(&(y.time-f.time).abs()));
            if let Some(g) = next { return Ok(Some(vec![(s1,g)])); }
            if depth >= STEEP_DEPTH { return Ok(None); }
            let m = 0.5*(s0+s1);
            let Some(mut first) = steep_from(roots_at,s0,f,m,depth+1)? else { return Ok(None) };
            let &(_,g) = first.last().unwrap();
            let Some(rest) = steep_from(roots_at,m,g,s1,depth+1)? else { return Ok(None) };
            first.extend(rest);
            Ok(Some(first))
        }
        // Roots at a walk length, memoised on the grid.
        let lookup = |s: f64| { let k = (s/h).round() as i64; if (s/h-k as f64).abs() < 1e-9 { roots(k) } else { roots_at(s) } };
        let steep = |s0: f64,f: Found,s1: f64,depth: u32| steep_from(&lookup,s0,f,s1,depth);
        let longest = 3.*total;
        // Walk from the anchor in one direction until the extent is reached, turning at folds.
        // The walk steps to the grid of walk lengths (whose roots are memoised), following a
        // steep root in halved steps; where the root ends between two grid points, halving the
        // step finds where, and if the other root meets it there it is a fold: the walk turns
        // onto the other root and back.
        let period = (total/h).round() as i64;
        let walk = |direction: i64| -> Result<Vec<Point>,TraceError> {
            let (mut s,mut f,mut dir) = (a as f64*h,anchor,direction);
            let mut w = Walked {out:Vec::new(),within:0.,outside:0.,last:anchor.position,last_s:s,tau:s,
                sense:direction as f64};
            loop {
                // A curve runs to its extent however far its contacts move outside the blank, but
                // must not stay in the blank for more than a few turns of the loop.
                let runaway = w.within > longest || w.out.len() > 64*period as usize;
                if runaway {
                    if self.debug {
                        for (n,p) in w.out.iter().enumerate().filter(|(n,_)| n % 8 == 0) {
                            eprintln!("  {n} s={:.4} tau={:.4} b{} t={:.4} in={} p={:?}",p.s,p.tau,p.found.branch,p.found.time,
                                inside(&[p.found.position])?[0],p.found.position.map(|x| (x*1e3).round()/1e3));
                        }
                    }
                    return Err("a station's contact curve does not leave the blank".into());
                }
                match extent {
                    Extent::Blank => if w.outside >= margin { break },
                    Extent::Span {lo,hi,..} => if (direction > 0 && w.tau >= hi) || (direction < 0 && w.tau <= lo) { break },
                }
                let here = (s/h).round() as i64;
                let on_grid = (s/h-here as f64).abs() < 1e-9;
                if !w.out.is_empty() && on_grid && (here-a).rem_euclid(period) == 0 && f.branch == anchor.branch
                    && (f.time-anchor.time).abs() < 1e-6 {
                    return Err("a station's contact curve closes on itself inside the blank".into());
                }
                let target = if on_grid { here+dir } else if dir > 0 { (s/h).ceil() as i64 } else { (s/h).floor() as i64 };
                if let Some(path) = steep(s,f,target as f64*h,0)? {
                    for (t,g) in path { w.push(t,g,inside)?; f = g; }
                    s = target as f64*h; continue;
                }
                // The root ends before the target: find where by halving the step.
                let mut step = target as f64*h-s;
                let mut tries = 0;
                while step.abs() > h*1e-7 && tries < 400 {
                    tries += 1;
                    // At the fold itself the two roots are one double root, which the root finder
                    // calls degenerate: the root has ended there.
                    let here = match roots_at(s+step) { Ok(r) => r, Err(TraceError::Degenerate(_)) => Vec::new(), Err(e) => return Err(e) };
                    let next = here.into_iter().filter(|g| g.branch == f.branch && (g.time-f.time).abs() < STEP_TIME)
                        .min_by(|x,y| (x.time-f.time).abs().total_cmp(&(y.time-f.time).abs()));
                    match next { Some(g) => { s += step; w.push(s,g,inside)?; f = g; } None => step *= 0.5 }
                }
                // A root that ends at the edge of the window has left it, and is no fold.
                if (f.time-wide[0]).abs() < 1e-3 || (f.time-wide[1]).abs() < 1e-3 {
                    if self.debug { eprintln!("trace leaves the window at s={s:.4} tau={:.4}",w.tau); }
                    break;
                }
                let other = roots_at(s)?.into_iter().filter(|g| g.branch != f.branch && (g.time-f.time).abs() < STEP_TIME)
                    .min_by(|x,y| (x.time-f.time).abs().total_cmp(&(y.time-f.time).abs()));
                let Some(other) = other else {
                    if self.debug {
                        eprintln!("trace end at s={s:.6} tau={:.4} branch {} time {:.4}; roots here {:?}",w.tau,f.branch,f.time,
                            roots_at(s)?.iter().map(|g| (g.branch,g.time)).collect::<Vec<_>>());
                    }
                    break
                };
                w.push(s,other,inside)?;
                f = other; dir = -dir;
            }
            Ok(w.out)
        };
        let back = walk(-1)?;
        let ahead = walk(1)?;
        let mut curve: Vec<Point> = back.into_iter().rev().collect();
        let anchor_at = curve.len();
        curve.push(Point {s:a as f64*h,tau:a as f64*h,found:anchor});
        curve.extend(ahead);
        Ok(Traced {curve,anchor:anchor_at})
    }

    /// Points of a traced contact curve at the given unfolded walk lengths. Between two points
    /// on one root the point is the exact contact at the walk length interpolated between them;
    /// across a fold, where the root turns, the nearer point.
    pub fn along(&self,station: &Station,curve: &[Point],at: &[f64]) -> Result<Vec<Found>,TraceError> {
        at.iter().map(|&tau| {
            let k = curve.partition_point(|p| p.tau < tau).clamp(1,curve.len()-1);
            let (a,b) = (curve[k-1],curve[k]);
            let t = if b.tau > a.tau { ((tau-a.tau)/(b.tau-a.tau)).clamp(0.,1.) } else { 0. };
            if a.found.branch == b.found.branch && b.tau > a.tau {
                let (s,near) = (a.s+(b.s-a.s)*t,a.found.time+(b.found.time-a.found.time)*t);
                if let Some(f) = self.contacts((station.sample)(s)?,self.wide())?.into_iter()
                    .filter(|f| f.branch == a.found.branch).min_by(|x,y| (x.time-near).abs().total_cmp(&(y.time-near).abs())) {
                    return Ok(f);
                }
            }
            Ok(if t < 0.5 { a.found } else { b.found })
        }).collect()
    }

    /// A traced contact curve's positions between unfolded walk lengths `lo` and `hi`, ends included.
    fn stretch(&self,station: &Station,curve: &[Point],lo: f64,hi: f64) -> Result<Vec<(f64,V)>,TraceError> {
        let ends = self.along(station,curve,&[lo,hi])?;
        let mut path = vec![(lo,ends[0].position)];
        path.extend(curve.iter().filter(|p| p.tau > lo && p.tau < hi).map(|p| (p.tau,p.found.position)));
        path.push((hi,ends[1].position));
        Ok(path)
    }

    /// The length in space of a traced contact curve between unfolded walk lengths `lo` and `hi`.
    fn length(&self,station: &Station,curve: &[Point],lo: f64,hi: f64) -> Result<f64,TraceError> {
        let path = self.stretch(station,curve,lo,hi)?;
        Ok(path.windows(2).map(|w| distance(w[0].1,w[1].1)).sum())
    }

    /// Contacts of a traced curve at fractions of its length in space between unfolded walk lengths
    /// `lo` and `hi`: the walk length at each is interpolated along the curve's points.
    fn at_lengths(&self,station: &Station,curve: &[Point],lo: f64,hi: f64,fractions: &[f64]) -> Result<Vec<Found>,TraceError> {
        let path = self.stretch(station,curve,lo,hi)?;
        let mut cumulative = vec![0.];
        for w in path.windows(2) { cumulative.push(cumulative.last().unwrap()+distance(w[0].1,w[1].1)); }
        let total = *cumulative.last().unwrap();
        let taus: Vec<f64> = fractions.iter().map(|f| {
            let l = f*total;
            let k = cumulative.partition_point(|c| *c < l).clamp(1,path.len()-1);
            let (a,b) = (cumulative[k-1],cumulative[k]);
            let t = if b > a { ((l-a)/(b-a)).clamp(0.,1.) } else { 0. };
            path[k-1].0+(path[k].0-path[k-1].0)*t
        }).collect();
        self.along(station,curve,&taus)
    }

    /// Whether any contact of the station, within the declared roll, lies in the blank.
    fn reaches(&self,station: &Station) -> Result<bool,TraceError> {
        let h = ROW_SPACING/4.;
        let declared = self.sweep.domain();
        let mut held = Vec::new();
        for i in 0..(station.length/h).ceil() as usize {
            let sample = (station.sample)(i as f64*h)?;
            let found = match self.contacts(sample,declared) {
                Ok(r) => r, Err(TraceError::Degenerate(_)) => continue, Err(e) => return Err(e) };
            held.extend(found.into_iter().map(|f| f.position));
        }
        Ok(!held.is_empty() && (self.inside)(&held)?.iter().any(|b| *b))
    }

    /// The candidate sheet over the band with the given margins, one chart of the envelope, on
    /// its first grid with a withheld contact at the centre of every cell: `layout`'s.
    pub fn sheet<'s>(&self,station_at: &(dyn Fn(f64) -> Result<Station<'s>,TraceError>+Sync),band: Band,margin: f64,
        station_margin: f64,placement: Rows) -> Result<Sheet,TraceError> {
        let layout = self.layout(station_at,band,margin,station_margin,placement)?;
        layout.sheet(&layout.grid,Withheld::Centres)
    }

    /// The stations of a candidate sheet over the band with the given margins, traced once and
    /// kept to evaluate the sheet on any grid over them (`Layout::sheet`): each column is a
    /// station's traced contact curve, resampled evenly as `placement` says, so a fold of the
    /// time chart is walked through rather than jumped. The first grid (`Layout::grid`) spaces
    /// columns `COLUMN_SPACING` apart at the band's mean radius and rows `ROW_SPACING`; a
    /// contact is withheld at the centre of every cell (the mid-angle station's curve at each
    /// row's mid length). The band's ends first move outward until the station there has no
    /// contact in the blank within the declared roll anywhere on its loop, since the band was
    /// sampled coarsely. `station_at` is the host's section of the cutter at a station angle.
    pub fn layout<'t,'s>(&'t self,station_at: &'t StationAt<'t,'s>,band: Band,margin: f64,
        station_margin: f64,placement: Rows) -> Result<Layout<'t,'s>,TraceError> {
        self.columns(station_at,band,margin,station_margin)?.layout(placement)
    }

    /// A sheet's columns traced and read at even walk lengths (`layout`'s first part), which every
    /// row placement is laid out from (`Columns::layout`): a second placement of the same columns
    /// traces nothing again.
    pub fn columns<'t,'s>(&'t self,station_at: &'t StationAt<'t,'s>,band: Band,margin: f64,
        station_margin: f64) -> Result<Columns<'t,'s>,TraceError> {
        let inside = self.inside;
        let step = (band.stations[1]-band.stations[0]).max(COLUMN_SPACING/band.radius);
        let [mut lo,mut hi] = band.stations;
        while self.reaches(&station_at(lo-station_margin)?)? {
            lo -= step; if hi-lo > TAU { return Err("the sheet's band of stations does not leave the blank".into()); }
        }
        while self.reaches(&station_at(hi+station_margin)?)? {
            hi += step; if hi-lo > TAU { return Err("the sheet's band of stations does not leave the blank".into()); }
        }
        let span = hi-lo+2.*station_margin;
        let columns = ((span*band.radius/COLUMN_SPACING).ceil() as usize).clamp(24,200);
        let angle_of = |c: f64| lo-station_margin+span*c/(columns-1) as f64;
        // Pass 1: each station reaching the blank traced until it has left it by the margin; the
        // union of their spans of unfolded walk length is the sheet's row range. The columns are
        // sectioned and traced side by side, and what each says taken in their order.
        let stations: Vec<Station> = crate::par::indices(columns,|c| station_at(angle_of(c as f64)))
            .into_iter().collect::<Result<_,_>>()?;
        let mut first: Vec<Option<Traced>> = Vec::with_capacity(columns);
        for traced in crate::par::map(&stations,|station| self.trace(station,margin,Extent::Blank)) {
            match traced {
                Ok(t) => first.push(Some(t)),
                Err(TraceError::Missed) => first.push(None),
                Err(e) => return Err(e),
            }
        }
        let reached: Vec<usize> = (0..columns).filter(|&c| first[c].is_some()).collect();
        if reached.is_empty() { return Err("no station of the sheet has a contact in the blank".into()); }
        let (lo_tau,hi_tau) = reached.iter().map(|&c| { let t = first[c].as_ref().unwrap(); (t.curve[0].tau,t.curve.last().unwrap().tau) })
            .fold((f64::INFINITY,f64::NEG_INFINITY),|(l,h),(a,b)| (l.min(a),h.max(b)));
        // Pass 2: every station over the common span, those missing the blank anchored beside
        // their nearest reaching neighbour.
        let nearest = |c: usize| *reached.iter().min_by_key(|&&k| k.abs_diff(c)).unwrap();
        let beside = |c: usize| { let t = first[nearest(c)].as_ref().unwrap(); let p = t.curve[t.anchor]; (p.s,p.found.time) };
        let traces: Vec<Traced> = crate::par::indices(columns,|c| {
            let extent = Extent::Span {lo:lo_tau,hi:hi_tau,beside:if first[c].is_some() { None } else { Some(beside(c)) }};
            self.trace(&stations[c],margin,extent)
        }).into_iter().collect::<Result<_,_>>()?;
        // The rows run over what every station's curve reached, which must still hold every
        // contact in the blank: a curve that ends first ends the chart inside the blank.
        let (lo_tau,hi_tau) = traces.iter().map(|t| (t.curve[0].tau,t.curve.last().unwrap().tau))
            .fold((f64::NEG_INFINITY,f64::INFINITY),|(l,h),(a,b)| (l.max(a),h.min(b)));
        let mut held = (f64::INFINITY,f64::NEG_INFINITY);
        for t in first.iter().flatten() {
            let within = inside(&t.curve.iter().map(|p| p.found.position).collect::<Vec<_>>())?;
            for (p,w) in t.curve.iter().zip(within) { if w { held = (held.0.min(p.tau),held.1.max(p.tau)); } }
        }
        if !(lo_tau < held.0 && hi_tau > held.1) {
            return Err(format!("a station's contact curve ends inside the blank: the stations reach walk lengths \
                {lo_tau:.3} to {hi_tau:.3} where the blank holds contacts from {:.3} to {:.3}",held.0,held.1).into());
        }
        let rows = (((hi_tau-lo_tau)/ROW_SPACING).ceil() as usize).clamp(24,240);
        let taus: Vec<f64> = (0..rows).map(|r| lo_tau+(hi_tau-lo_tau)*r as f64/(rows-1) as f64).collect();
        let columns_data: Vec<Vec<Found>> = crate::par::indices(columns,|c| self.along(&stations[c],&traces[c].curve,&taus))
            .into_iter().collect::<Result<_,_>>()?;
        // The rows run on outside the blank only while every column is still one chart: the fit's
        // chord-length parameters are averaged over the columns, so one column's margin leaping to
        // another root (a turn away, where a far corner's fan carries it) moves the parameters of
        // every row, the ones in the blank included. Rows are trimmed back to before such a leap;
        // a row holding a contact in the blank is never trimmed, the chart contract's to judge.
        let within = inside(&columns_data.iter().flatten().map(|f| f.position).collect::<Vec<_>>())?;
        let [first_row,last] = charted(&columns_data.iter().map(|c| c.iter().map(|f| f.time).collect()).collect::<Vec<Vec<f64>>>(),
            &within.chunks(rows).map(<[bool]>::to_vec).collect::<Vec<_>>());
        let middles: Vec<f64> = (0..rows-1).map(|r| lo_tau+(hi_tau-lo_tau)*(r as f64+0.5)/(rows-1) as f64)
            .skip(first_row).take(last-first_row).collect();
        let (lo_tau,hi_tau) = (if first_row == 0 { lo_tau } else { taus[first_row] },if last+1 == rows { hi_tau } else { taus[last] });
        let beside = (0..columns).map(|c| (angle_of(c as f64),beside(c))).collect();
        let stations = stations.into_iter().zip(traces).map(Arc::new).collect();
        Ok(Columns {tracer:self,station_at,margin,angles:(0..columns).map(|c| angle_of(c as f64)).collect(),
            column_mids:(0..columns-1).map(|c| angle_of(c as f64+0.5)).collect(),stations,beside,taus,columns_data,first_row,last,middles,
            span:[lo_tau,hi_tau]})
    }
}

/// A sheet's columns traced (`Tracer::columns`): each station and its contact curve, the contacts
/// read at the rows' even walk lengths, and the rows kept in the chart.
pub struct Columns<'t,'s> {
    tracer: &'t Tracer<'t>,
    station_at: &'t StationAt<'t,'s>,
    margin: f64,
    angles: Vec<f64>,
    column_mids: Vec<f64>,
    stations: Vec<Arc<(Station<'s>,Traced)>>,
    beside: Vec<(f64,(f64,f64))>,
    taus: Vec<f64>,
    columns_data: Vec<Vec<Found>>,
    first_row: usize,
    last: usize,
    middles: Vec<f64>,
    span: [f64;2],
}

impl<'t,'s> Columns<'t,'s> {
    /// The columns laid out with their rows placed as `placement` says.
    pub fn layout(&self,placement: Rows) -> Result<Layout<'t,'s>,TraceError> {
        let (tracer,columns) = (self.tracer,self.angles.len());
        let (taus,first_row,last) = (&self.taus,self.first_row,self.last);
        let [lo_tau,hi_tau] = self.span;
        // The rows as placed: kept as resampled, or each column again at even lengths in space.
        let (row_coordinates,row_mids) = match placement {
            Rows::Walk => (taus[first_row..=last].to_vec(),self.middles.clone()),
            Rows::Length => {
                let longest = crate::par::indices(columns,|c| tracer.length(&self.stations[c].0,&self.stations[c].1.curve,lo_tau,hi_tau))
                    .into_iter().collect::<Result<Vec<f64>,_>>()?.into_iter().fold(0_f64,f64::max);
                let rows = ((longest/ROW_SPACING).ceil() as usize).clamp(24,240);
                ((0..rows).map(|r| r as f64/(rows-1) as f64).collect(),(0..rows-1).map(|r| (r as f64+0.5)/(rows-1) as f64).collect())
            }
        };
        let grid = Grid {rows:row_coordinates,row_mids,columns:self.angles.clone(),column_mids:self.column_mids.clone()};
        let mut nodes = BTreeMap::new();
        if placement == Rows::Walk {
            for (c,column) in self.columns_data.iter().enumerate() {
                for (tau,f) in taus.iter().zip(column) { nodes.insert((grid.columns[c].to_bits(),tau.to_bits()),*f); }
            }
        }
        let traced = self.stations.iter().enumerate().map(|(c,entry)| (grid.columns[c].to_bits(),Some(entry.clone()))).collect();
        Ok(Layout {tracer,station_at:self.station_at,placement,margin:self.margin,span:[lo_tau,hi_tau],
            beside:self.beside.clone(),stations:Mutex::new(traced),nodes:Mutex::new(nodes),grid})
    }
}

/// Where a sheet's nodes lie and where its withheld contacts: the row coordinates (unfolded walk
/// lengths for `Rows::Walk`, fractions of each column's length between the sheet's ends for
/// `Rows::Length`) and the column station angles, each ascending, with the coordinate a withheld
/// contact takes in each interval between them.
#[derive(Clone,Debug,PartialEq)]
pub struct Grid { pub rows: Vec<f64>,pub row_mids: Vec<f64>,pub columns: Vec<f64>,pub column_mids: Vec<f64> }

/// How far one interval may be longer than its neighbour after a refinement: an interpolating
/// spline through nodes whose spacing jumps by more rings in the long interval, and the
/// refinement would chase its own ringing.
const GRADING: f64 = 2.;

impl Grid {
    /// The grid with each marked interval (row intervals, then column intervals) split at its
    /// withheld coordinate, which becomes a node (the contact withheld there, a node's), with a
    /// withheld coordinate in the middle of each half; and further intervals split until none is
    /// more than `GRADING` times as long as its neighbour.
    pub fn refined(&self,rows: &[bool],columns: &[bool]) -> Grid {
        let (r,rm) = split(&self.rows,&self.row_mids,rows);
        let (c,cm) = split(&self.columns,&self.column_mids,columns);
        Grid {rows:r,row_mids:rm,columns:c,column_mids:cm}
    }
}

/// One direction of `Grid::refined`.
fn split(nodes: &[f64],mids: &[f64],marked: &[bool]) -> (Vec<f64>,Vec<f64>) {
    let mut marked = marked.to_vec();
    let width = |i: usize,m: &[bool]| (nodes[i+1]-nodes[i]).abs()*if m[i] { 0.5 } else { 1. };
    loop {
        let mut changed = false;
        for i in 0..marked.len() {
            if marked[i] { continue }
            let w = width(i,&marked);
            let over = [i.checked_sub(1),(i+1 < marked.len()).then_some(i+1)].into_iter().flatten()
                .any(|j| w > GRADING*width(j,&marked)*(1.+1e-9));
            if over { marked[i] = true; changed = true; }
        }
        if !changed { break }
    }
    let (mut n,mut m) = (vec![nodes[0]],Vec::new());
    for i in 0..mids.len() {
        if marked[i] {
            m.push(0.5*(nodes[i]+mids[i])); n.push(mids[i]);
            m.push(0.5*(mids[i]+nodes[i+1]));
        } else { m.push(mids[i]); }
        n.push(nodes[i+1]);
    }
    (n,m)
}

/// Which intervals of a sheet's grid to refine, from its withheld contacts (`Sheet::sites`) and
/// which of them miss the fit's bar: a contact at the middle of a column's step between two rows
/// says the rows there are too far apart, one at the middle of a row's step between two columns
/// that the columns are; one at a cell's centre whose sides said neither, that both are (a twist
/// neither side's contact sees). Row intervals, then column intervals.
pub fn marked(sites: &[[usize;2]],over: &[bool],rows: usize,columns: usize) -> (Vec<bool>,Vec<bool>) {
    let (mut r,mut c) = (vec![false;rows.saturating_sub(1)],vec![false;columns.saturating_sub(1)]);
    for (s,_) in sites.iter().zip(over).filter(|(_,o)| **o) {
        match (s[0]%2,s[1]%2) { (1,0) => r[s[0]/2] = true, (0,1) => c[s[1]/2] = true, _ => {} }
    }
    let (sides_r,sides_c) = (r.clone(),c.clone());
    for (s,_) in sites.iter().zip(over).filter(|(_,o)| **o) {
        let (i,j) = (s[0]/2,s[1]/2);
        if s[0]%2 == 1 && s[1]%2 == 1 && !sides_r[i] && !sides_c[j] { r[i] = true; c[j] = true; }
    }
    (r,c)
}

/// Which contacts a sheet withholds from its fit: one at the centre of every cell, or also one
/// at the middle of each side of every cell (down each column between its rows, and across each
/// row between its columns), which say which way a cell is under-sampled.
#[derive(Clone,Copy,Debug,PartialEq,Eq)]
pub enum Withheld { Centres,Sides }

/// A candidate sheet's traced stations, kept to evaluate the sheet on any grid over them: a
/// station traced once (the first grid's, and each one a finer grid or a withheld contact asks
/// for) and every contact read once.
pub struct Layout<'t,'s> {
    tracer: &'t Tracer<'t>,
    station_at: &'t StationAt<'t,'s>,
    placement: Rows,
    margin: f64,
    /// The rows' ends, in unfolded walk length.
    span: [f64;2],
    /// The first grid's column angles, each with the anchor (walk length and time) of the
    /// station nearest it that reaches the blank: a station between two of them is traced from
    /// the anchor of the one before it.
    beside: Vec<(f64,(f64,f64))>,
    /// Traced stations by angle; `None` where a withheld station could not be traced.
    stations: Mutex<BTreeMap<u64,Option<Arc<(Station<'s>,Traced)>>>>,
    /// Contacts read, by station angle and row coordinate.
    nodes: Mutex<BTreeMap<(u64,u64),Found>>,
    /// The first grid.
    pub grid: Grid,
}

impl<'s> Layout<'_,'s> {
    /// The station at `angle`, traced over the sheet's rows beside the anchor of the first grid's
    /// column before it; `None` where it cannot be traced.
    fn station(&self,angle: f64) -> Result<Option<Arc<(Station<'s>,Traced)>>,TraceError> {
        if let Some(entry) = self.stations.lock().unwrap_or_else(|e| e.into_inner()).get(&angle.to_bits()) { return Ok(entry.clone()); }
        let station = (self.station_at)(angle)?;
        let before = self.beside.partition_point(|b| b.0 <= angle).saturating_sub(1);
        let extent = Extent::Span {lo:self.span[0],hi:self.span[1],beside:Some(self.beside[before].1)};
        let entry = self.tracer.trace(&station,self.margin,extent).ok().map(|t| Arc::new((station,t)));
        // two threads tracing one angle trace it alike; the first kept
        let mut stations = self.stations.lock().unwrap_or_else(|e| e.into_inner());
        Ok(stations.entry(angle.to_bits()).or_insert(entry).clone())
    }

    /// A station's contacts at row coordinates, each read once.
    fn at(&self,angle: f64,entry: &(Station<'s>,Traced),coordinates: &[f64]) -> Result<Vec<Found>,TraceError> {
        let key = angle.to_bits();
        let missing: Vec<f64> = { let nodes = self.nodes.lock().unwrap_or_else(|e| e.into_inner());
            coordinates.iter().copied().filter(|x| !nodes.contains_key(&(key,x.to_bits()))).collect() };
        if !missing.is_empty() {
            let (station,traced) = entry;
            let found = match self.placement {
                Rows::Walk => self.tracer.along(station,&traced.curve,&missing)?,
                Rows::Length => self.tracer.at_lengths(station,&traced.curve,self.span[0],self.span[1],&missing)?,
            };
            let mut nodes = self.nodes.lock().unwrap_or_else(|e| e.into_inner());
            for (x,f) in missing.iter().zip(found) { nodes.entry((key,x.to_bits())).or_insert(f); }
        }
        let nodes = self.nodes.lock().unwrap_or_else(|e| e.into_inner());
        Ok(coordinates.iter().map(|x| nodes[&(key,x.to_bits())]).collect())
    }

    /// The sheet on `grid` (the first grid, or one refined from it), withholding the contacts
    /// `withheld` says. A node station that cannot be traced refuses the sheet; a withheld
    /// station that cannot be traced withholds nothing.
    pub fn sheet(&self,grid: &Grid,withheld: Withheld) -> Result<Sheet,TraceError> {
        let (rows,columns) = (grid.rows.len(),grid.columns.len());
        // Every station the sheet reads traced, and its contacts read, side by side first: what the
        // pass below then reads is what it would have traced and read itself, one after another,
        // and where a station fails here it fails there again, in order.
        let asks: Vec<(f64,Vec<&[f64]>)> = grid.columns.iter().map(|&a| (a,if withheld == Withheld::Sides {
            vec![&grid.rows[..],&grid.row_mids[..]] } else { vec![&grid.rows[..]] }))
            .chain(grid.column_mids.iter().map(|&a| (a,if withheld == Withheld::Sides {
                vec![&grid.row_mids[..],&grid.rows[..]] } else { vec![&grid.row_mids[..]] })))
            .collect();
        crate::par::map(&asks,|(angle,coordinates)| {
            let Ok(Some(entry)) = self.station(*angle) else { return };
            for c in coordinates { if self.at(*angle,&entry,c).is_err() { return } }
        });
        let mut data = Vec::with_capacity(columns);
        for &angle in &grid.columns {
            let entry = self.station(angle)?.ok_or_else(|| TraceError::Failed(format!(
                "the sheet's station at {:.4} degrees could not be traced over its rows",angle.to_degrees())))?;
            data.push(self.at(angle,&entry,&grid.rows)?);
        }
        let mut sheet = Sheet {points:Vec::new(),normals:Vec::new(),times:Vec::new(),rows,columns,
            withheld:Vec::new(),withheld_normals:Vec::new(),sites:Vec::new()};
        let mut hold = |f: Found,site: [usize;2]| {
            sheet.withheld.push(f.position); sheet.withheld_normals.push(f.normal); sheet.sites.push(site);
        };
        // Each mid-angle station at mid rows (the cells' centres), and at the rows (the middles
        // of the cells' sides across the columns).
        for (c,&angle) in grid.column_mids.iter().enumerate() {
            let Some(entry) = self.station(angle)? else { continue };
            for (r,f) in self.at(angle,&entry,&grid.row_mids)?.into_iter().enumerate() { hold(f,[2*r+1,2*c+1]); }
            if withheld == Withheld::Sides {
                for (r,f) in self.at(angle,&entry,&grid.rows)?.into_iter().enumerate() { hold(f,[2*r,2*c+1]); }
            }
        }
        // Each node station at mid rows: the middles of the cells' sides down the columns.
        if withheld == Withheld::Sides {
            for (c,&angle) in grid.columns.iter().enumerate() {
                let entry = self.station(angle)?.expect("a node station is traced");
                for (r,f) in self.at(angle,&entry,&grid.row_mids)?.into_iter().enumerate() { hold(f,[2*r+1,2*c]); }
            }
        }
        for r in 0..rows { for column in &data {
            sheet.points.push(column[r].position); sheet.normals.push(column[r].normal); sheet.times.push(column[r].time);
        } }
        Ok(sheet)
    }
}
