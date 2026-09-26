//! A cutter's contact curves under a generating sweep, and the candidate sheet they make: the
//! numerics of the native swept construction (docs/generating-sweeps.md), which a host feeds
//! with sections of its own cutter. A **station** is one section of the cutter by a meridian
//! half-plane, walked by augmented length (position length plus turning across convex corners);
//! the host samples it, a point and its outward normal at a walk length, and nothing here knows
//! how. What is here: every root of a sampled point's contact equation, a station's contact
//! curve traced through the plane of walk length and time (turning at a fold of the time chart,
//! where the envelope carries on though the walk turns back), and the sheet whose columns are
//! those curves, resampled, with a withheld contact at the centre of every cell. Positions are
//! native millimetres, `scale` of them a model unit; the contact math runs in model units.
use super::SweepContacts;
use super::sweep_contacts::PointContactError;
use crate::space::{scale as scaled,distance};
use std::{cell::RefCell,collections::BTreeMap,f64::consts::TAU,fmt};

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
pub type Inside<'a> = &'a dyn Fn(&[V]) -> Result<Vec<bool>,String>;

/// One section of the cutter as the host samples it: its augmented length (the period of its
/// walk), the window of walk length its contacts are anchored in, and its sampler.
pub struct Station<'a> {
    pub length: f64,
    pub window: [f64;2],
    pub sample: Box<dyn Fn(f64) -> Result<Sample,TraceError>+'a>,
}

/// The stations' band as a first coarse pass found it: its station angles, and the profile's
/// mean distance from the axis, for column spacing.
#[derive(Clone,Copy,Debug)]
pub struct Band { pub stations: [f64;2],pub radius: f64 }

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
    /// Contacts at the centre of every grid cell, withheld from the fit, with their normals.
    pub withheld: Vec<V>,
    pub withheld_normals: Vec<V>,
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

    /// The candidate sheet over the band with the given margins, one chart of the envelope:
    /// each column is a station's traced contact curve, resampled evenly by arc length, so a
    /// fold of the time chart is walked through rather than jumped. A contact is withheld at the
    /// centre of every cell (the mid-angle station's curve at each row's mid length). The band's
    /// ends first move outward until the station there has no contact in the blank within the
    /// declared roll anywhere on its loop, since the band was sampled coarsely. `station_at`
    /// is the host's section of the cutter at a station angle.
    pub fn sheet<'s>(&self,station_at: &dyn Fn(f64) -> Result<Station<'s>,TraceError>,band: Band,margin: f64,
        station_margin: f64) -> Result<Sheet,TraceError> {
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
        // union of their spans of unfolded walk length is the sheet's row range.
        let stations: Vec<Station> = (0..columns).map(|c| station_at(angle_of(c as f64))).collect::<Result<_,_>>()?;
        let mut first: Vec<Option<Traced>> = Vec::with_capacity(columns);
        for station in &stations {
            match self.trace(station,margin,Extent::Blank) {
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
        let traces: Vec<Traced> = (0..columns).map(|c| {
            let extent = Extent::Span {lo:lo_tau,hi:hi_tau,beside:if first[c].is_some() { None } else { Some(beside(c)) }};
            self.trace(&stations[c],margin,extent)
        }).collect::<Result<_,_>>()?;
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
        let columns_data: Vec<Vec<Found>> = traces.iter().zip(&stations)
            .map(|(t,station)| self.along(station,&t.curve,&taus)).collect::<Result<_,_>>()?;
        // Withheld: each mid-angle station over the same span, at mid rows.
        let (mut withheld,mut withheld_normals) = (Vec::new(),Vec::new());
        let middles: Vec<f64> = (0..rows-1).map(|r| lo_tau+(hi_tau-lo_tau)*(r as f64+0.5)/(rows-1) as f64).collect();
        for c in 0..columns-1 {
            let station = station_at(angle_of(c as f64+0.5))?;
            let extent = Extent::Span {lo:lo_tau,hi:hi_tau,beside:Some(beside(c))};
            let Ok(t) = self.trace(&station,margin,extent) else { continue };
            for f in self.along(&station,&t.curve,&middles)? { withheld.push(f.position); withheld_normals.push(f.normal); }
        }
        let (mut points,mut normals,mut times) = (Vec::new(),Vec::new(),Vec::new());
        for r in 0..rows { for column in &columns_data {
            points.push(column[r].position); normals.push(column[r].normal); times.push(column[r].time);
        } }
        Ok(Sheet {points,normals,times,rows,columns,withheld,withheld_normals})
    }
}
