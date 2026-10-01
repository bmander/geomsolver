//! Continuous volume sweeps of explicit one-Lipschitz material fields.
#[allow(unused_imports)]
use crate::fmath::Det;
use super::{SpatialField,Error,I,V,Query,Reading,Source,Want,memo::{self,Grid,Memo}};
use crate::{interval::minimum::{self,Minimum,Options,Stop},motion::{Family,MotionBounds},roots::brent};
use std::{collections::{BTreeMap,BinaryHeap},sync::{Arc,OnceLock}};

pub type SweepError = minimum::Error<Error>;

/// A sweep's least value at a point, the roll time it is least at, and whether a second contact
/// time reads nearly as low (`SweptField::minimum_relative`).
#[derive(Clone,Copy,Debug,PartialEq)]
pub struct SweptMinimum { pub value: f64,pub time: f64,pub tied: bool }

/// The field min_t source(inverse(motion(t)) x) over a finite closed interval.
/// Source, motion and domain are immutable solved snapshots. Its material is
/// closure({f<0}); the field is one-Lipschitz but need not be signed distance.
/// A point domain represents one fixed pose. Source-solve error is separate.
#[derive(Clone,Debug)]
pub struct SweptField {source:SpatialField,motion:Family,domain:I,caches:Arc<SweepCaches>}

/// What a sweep works out once and keeps, shared by every clone: an indexed cut's copies are one
/// sweep placed apart, and each reads the one table at its own point turned into the sweep's frame.
/// Nothing here is ever invalidated — source, motion and domain are an immutable snapshot, and an
/// edit to the model builds a new sweep, and new caches with it.
///
/// (`SweepEvaluator` keeps interval motion bounds at the midpoints of the cells its refiner asks,
/// capped by its caller: interval poses of arbitrary times for enclosures, where these are plain
/// poses at the fixed dyadic times every reading shares. The two are different numbers of different
/// times and are kept apart.)
#[derive(Debug,Default)]
struct SweepCaches {
    /// The inverse poses at the first `ROLL_LEVELS` dyadic divisions of the roll, which every
    /// search reads (`Roll`).
    poses:OnceLock<Vec<Option<crate::motion::Pose>>>,
    /// The box every pose of the source lies in, as plain numbers: a point outside it is outside
    /// the material (`clear_of`).
    support:OnceLock<Option<[[f64;2];3]>>,
    /// The side of the floor table's cubes: a fraction of the source's own size.
    cube:OnceLock<f64>,
    /// Proven lower bounds of the sweep at the centres of cubes a `floor` query has reached, keyed
    /// by cube and coarsening, filled as they are asked for.
    floors:Memo<([i32;3],u32),Arc<Cube>>,
    /// Adaptive distance fields of the sweep, by the resolution asked for (`cached`).
    adfs:Memo<[u64;3],super::adf::Adf>,
}

/// A sweep's partition of its roll at one cube's centre (`SweptField::floor`): the least bound
/// proven over the roll, and each stretch `(bound, start, end)` in dyadic roll indices. At a point
/// a distance d from the centre every bound holds less d, the motion being rigid.
#[derive(Debug)]
struct Cube { low: f64,stretches: Box<[(f64,u64,u64)]> }

/// A stretch of roll between two dyadic indices: a lower bound on the field over it, and the
/// readings at its ends (NaN until made). Ordered lowest bound first in a heap.
#[derive(PartialEq)]
struct Stretch(f64,u64,f64,u64,f64);
impl Eq for Stretch {}
impl PartialOrd for Stretch { fn partial_cmp(&self,o: &Self) -> Option<std::cmp::Ordering> { Some(self.cmp(o)) } }
impl Ord for Stretch { fn cmp(&self,o: &Self) -> std::cmp::Ordering { o.0.total_cmp(&self.0) } }

/// Source evaluations made by every sweep search so far, for a caller measuring the oracle.
pub static SIDE_EVALUATIONS: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);

/// The source read along one point's path through the roll: at dyadic indices of a grid
/// `ROLL_DEPTH` levels fine (the first `ROLL_LEVELS` levels' inverse poses from the table every
/// search shares), or at any time. It counts its readings, and adds them to `SIDE_EVALUATIONS`
/// when dropped.
struct Roll<'a> { field: &'a SweptField,p: [f64;3],a: f64,b: f64,
    table: &'a [Option<crate::motion::Pose>],evaluations: std::cell::Cell<u64> }

impl Roll<'_> {
    /// The grid's last index, and its time.
    const LAST: u64 = 1 << ROLL_DEPTH;
    fn time(&self,i: u64) -> f64 { self.a+(self.b-self.a)*(i as f64/Self::LAST as f64) }
    /// The roll one grid step covers.
    fn per(&self) -> f64 { (self.b-self.a)/Self::LAST as f64 }
    fn value(&self,pose: Option<crate::motion::Pose>) -> f64 {
        self.evaluations.set(self.evaluations.get()+1);
        pose.map_or(f64::INFINITY,|m| self.field.source.value(m.point(self.p)))
    }
    fn at(&self,i: u64) -> f64 {
        let step = ROLL_DEPTH-ROLL_LEVELS;
        let pose = if i & ((1u64 << step)-1) == 0 { self.table[(i >> step) as usize] }
            else { self.field.motion.pose_at(self.time(i)).ok().map(|m| m.inverse()) };
        self.value(pose)
    }
    fn at_time(&self,t: f64) -> f64 { self.value(self.field.motion.pose_at(t).ok().map(|m| m.inverse())) }
    fn evaluations(&self) -> u64 { self.evaluations.get() }
    /// The first-order bound over the stretch `[i0, i1]` read `f0` and `f1` at its ends, the
    /// source moving at most `speed` per unit roll: at least their mean less the speed times half
    /// its width.
    fn bound(&self,speed: f64,i0: u64,f0: f64,i1: u64,f1: f64) -> f64 {
        0.5*(f0+f1)-0.5*speed*self.per()*(i1-i0) as f64
    }
}

impl Drop for Roll<'_> {
    fn drop(&mut self) { SIDE_EVALUATIONS.fetch_add(self.evaluations.get(),std::sync::atomic::Ordering::Relaxed); }
}

/// Readings at dyadic roll indices, each made once: stretches carried from a cube share ends.
struct Reads<'a,'b> { roll: &'a Roll<'b>,made: std::cell::RefCell<Vec<(u64,f64)>> }
impl<'a,'b> Reads<'a,'b> {
    fn new(roll: &'a Roll<'b>) -> Self { Self {roll,made:std::cell::RefCell::new(Vec::new())} }
    fn at(&self,i: u64) -> f64 {
        if let Some(&(_,v)) = self.made.borrow().iter().find(|m| m.0 == i) { return v; }
        let v = self.roll.at(i);
        self.made.borrow_mut().push((i,v));
        v
    }
    /// A reading already in hand, or made now where it is not.
    fn or(&self,i: u64,v: f64) -> f64 { if v.is_nan() { self.at(i) } else { v } }
}

/// The stretches `open` still holds, grouped into contiguous runs in roll order, each a basin:
/// its readings, and the bracket about the lowest of them — the readings either side, or a
/// stretch's width past a run's end.
fn basins(mut open: Vec<(u64,f64,u64,f64)>) -> Vec<(Vec<(u64,f64)>,u64,u64)> {
    open.sort_by_key(|s| s.0);
    let mut runs: Vec<Vec<(u64,f64,u64,f64)>> = Vec::new();
    for s in open {
        match runs.last_mut() { Some(r) if r.last().unwrap().2 == s.0 => r.push(s), _ => runs.push(vec![s]) }
    }
    runs.into_iter().map(|run| {
        let mut readings: Vec<(u64,f64)> = run.iter().map(|s| (s.0,s.1)).collect();
        let end = run[run.len()-1];
        readings.push((end.2,end.3));
        let k = (0..readings.len()).min_by(|&x,&y| readings[x].1.total_cmp(&readings[y].1)).unwrap();
        let w = run[0].2-run[0].0;
        let w0 = readings[k.saturating_sub(1)].0.saturating_sub(if k == 0 { w } else { 0 });
        let w1 = (if k+1 < readings.len() { readings[k+1].0 } else { readings[k].0+w }).min(Roll::LAST);
        (readings,w0,w1)
    }).collect()
}

/// Whether a minimum found at `t` in the window `[lo, hi]` lies within `margin` of an edge the roll
/// goes on past (`open_lo`, `open_hi`): then the minimum may be beyond it.
fn at_edge(t: f64,lo: f64,hi: f64,open_lo: bool,open_hi: bool,margin: f64) -> bool {
    (t-lo < margin && open_lo) || (hi-t < margin && open_hi)
}

/// The least of `value` over the roll `[a, b]` near `time`: Brent's search in a narrow window about
/// it, the window moved on and widened (to a sixty-fourth of the roll) each time the least is
/// found at its edge, so a continuation follows its minimum past the window. Without a time the
/// window is about the least of a coarse sampling, a sixty-fourth of the roll wide. `(value, time)`.
/// A reading, not a bound: a deeper minimum away from the window is not looked for.
pub(super) fn follow(value: &impl Fn(f64) -> f64,a: f64,b: f64,time: Option<f64>,accuracy: f64) -> (f64,f64) {
    let mut width = if time.is_some_and(f64::is_finite) { (b-a)/1024. } else { (b-a)/64. };
    let mut centre = time.filter(|t| t.is_finite()).unwrap_or_else(|| (0..=256)
        .map(|k| a+(b-a)*k as f64/256.).min_by(|x,y| value(*x).total_cmp(&value(*y))).unwrap_or(a));
    let mut found = (value(centre),centre);
    for _ in 0..16 {
        let (lo,hi) = ((centre-0.5*width).max(a),(centre+0.5*width).min(b));
        // Brent to the accuracy asked, read off the parabola through its best points
        found = brent(value,lo,hi,1e-12*(1.+hi.abs()),60,|_,slack| slack <= 0.25*accuracy);
        // a least value at the roll's own end is no parabola's: read the end itself
        for end in [a,b] {
            if end >= lo && end <= hi { let v = value(end); if v < found.0 { found = (v,end); } }
        }
        // At the window's edge, and not the roll's: the minimum lies beyond; follow it.
        let margin = 0.02*width;
        if !at_edge(found.1,lo,hi,lo > a,hi < b,margin) { break; }
        centre = found.1+if found.1-lo < margin { -0.45*width } else { 0.45*width };
        width = (2.*width).min((b-a)/64.);
    }
    found
}

/// The cubes a sweep's `floor` table is kept in are the source's diagonal over this.
const FLOOR_CUBES: f64 = 256.;
/// Source evaluations a cube's bound may spend.
const FLOOR_BUDGET: u64 = 64;

/// Dyadic level at which the searches stop splitting and search each basin left (see `side`).
const ROLL_BASIN: u32 = 6;

/// Dyadic levels of the roll whose inverse poses every search shares: 2¹⁰ + 1 of them.
const ROLL_LEVELS: u32 = 10;
/// Dyadic levels a stretch may be split to: its ends are integers on a grid this fine.
const ROLL_DEPTH: u32 = 40;

/// What a roll search is after, and so when it stops (`RollSearch::run`).
#[derive(Clone,Copy)]
enum Goal {
    /// Every bound at least `cap`, or a reading below it or `budget` evaluations spent
    /// (`SweptField::at_least`).
    AtLeast { cap: f64,budget: u64 },
    /// The least bound within `slack` of the least reading, or `budget` spent
    /// (`SweptField::bound_at`).
    Floor { slack: f64,budget: u64 },
    /// The field's sign: a negative reading, or every bound above `-tolerance` (`SweptField::sign`).
    Sign { tolerance: f64 },
    /// The least value, to `accuracy` or `relative` of itself, a basin's minimum found by Brent's
    /// search; two basins' minima within `tie` are two contact times (`SweptField::minimum_hinted`).
    Minimum { accuracy: f64,relative: f64,tie: f64 },
}

/// How a roll search ended.
enum Ended {
    /// Every bound reached what was asked (`AtLeast`: the least of them; `Floor`: the least bound
    /// with the least reading).
    Proven(f64),
    /// A reading fell below what was asked, or the budget ran out first (`AtLeast`).
    Short,
    /// The goal is met: the search's `best` (with `best_t` and `tied`) is the answer.
    Done,
}

/// One point's search over the roll, the one loop every sweep reading runs: stretches of roll kept
/// lowest bound first, each bound the first-order one (`Roll::bound`) — the source is Lipschitz in
/// the roll by the motion's inverse-point speed bound — the lowest split at its middle until the
/// goal is met; where a goal reads minima rather than bounds, splitting stops a basin wide
/// (`ROLL_BASIN`) and each basin left is searched by Brent's method about its lowest reading.
struct RollSearch<'r,'f> {
    read: Reads<'r,'f>,
    goal: Goal,
    /// The motion's speed bound at the point: given, or worked out when first needed and infinite
    /// where the motion gives none.
    speed: std::cell::OnceCell<f64>,
    /// Whether a stretch keeps the bound it came with where that is higher than its own: a stretch
    /// split from another, or carried from a cube. The bound searches that prove a cube's own
    /// bounds do not.
    inherit: bool,
    open: BinaryHeap<Stretch>,
    best: f64,
    best_t: f64,
    tied: bool,
    /// The window a warm start searched already (`minimum_hinted`): stretches inside it are
    /// skipped, and its minimum is a basin of its own.
    window: Option<(u64,u64,(f64,f64))>,
}

impl<'r,'f> RollSearch<'r,'f> {
    fn new(roll: &'r Roll<'f>,goal: Goal,speed: Option<f64>,inherit: bool) -> Self {
        let cell = std::cell::OnceCell::new();
        if let Some(s) = speed { let _ = cell.set(s); }
        Self {read:Reads::new(roll),goal,speed:cell,inherit,open:BinaryHeap::new(),best:f64::INFINITY,best_t:roll.a,
            tied:false,window:None}
    }
    fn roll(&self) -> &'r Roll<'f> { self.read.roll }
    fn speed(&self) -> f64 {
        let roll = self.roll();
        *self.speed.get_or_init(|| roll.field.motion.inverse_point_speed_bound(roll.p,roll.field.domain).unwrap_or(f64::INFINITY))
    }
    /// A stretch's bound from its readings, or the bound `given` it came with where that is higher.
    fn bound(&self,i0: u64,f0: f64,i1: u64,f1: f64,given: f64) -> f64 {
        let b = self.roll().bound(self.speed(),i0,f0,i1,f1);
        if self.inherit { b.max(given) } else { b }
    }
    /// A reading made: the best so far, and when the goal tracks it, its time.
    fn note(&mut self,i: u64,f: f64) {
        match self.goal {
            Goal::Minimum {..} => if f < self.best { self.best = f; self.best_t = self.roll().time(i); },
            _ => self.best = self.best.min(f),
        }
    }
    /// Whether a reading settles the search by itself: one inside the material asked the sign.
    fn decided(&self) -> bool { matches!(self.goal,Goal::Sign {..}) && self.best < 0. }
    /// The whole roll, read at its ends.
    fn seed_ends(&mut self,fa: f64,fb: f64) {
        let last = Roll::LAST;
        self.open.push(Stretch(self.bound(0,fa,last,fb,f64::NEG_INFINITY),0,fa,last,fb));
    }
    /// The stretches a cube's bound was proven over, each bound less `d`, the distance from its
    /// centre, and read here when first taken.
    fn seed_cube(&mut self,d: f64,cube: &Cube) {
        for &(bl,i0,i1) in cube.stretches.iter() { self.open.push(Stretch(bl-d,i0,f64::NAN,i1,f64::NAN)); }
    }
    /// A stretch split at its middle: its three readings (ends and middle), made where not in
    /// hand, noted, and its two halves open. The middle's reading.
    fn halve(&mut self,s: Stretch) -> f64 {
        let Stretch(low,i0,f0,i1,f1) = s;
        let (f0,f1) = (self.read.or(i0,f0),self.read.or(i1,f1));
        let im = i0+(i1-i0)/2;
        let fm = self.read.at(im);
        for (i,f) in [(i0,f0),(i1,f1),(im,fm)] { self.note(i,f); }
        let halves = [Stretch(self.bound(i0,f0,im,fm,low),i0,f0,im,fm),Stretch(self.bound(im,fm,i1,f1,low),im,fm,i1,f1)];
        self.open.extend(halves);
        fm
    }
    /// The widest stretch still open (bounded at or below `below`) that is wider than a basin,
    /// taken out of the heap.
    fn widest(&mut self,basin: u64,below: f64) -> Option<Stretch> {
        let k = self.open.iter().enumerate().filter(|(_,s)| s.0 <= below && s.3-s.1 > basin)
            .max_by_key(|(_,s)| s.3-s.1).map(|(k,_)| k)?;
        let mut all = std::mem::take(&mut self.open).into_vec();
        let wide = all.swap_remove(k);
        self.open = all.into();
        Some(wide)
    }
    /// Whether a stretch lies inside the window already searched.
    fn searched(&self,i0: u64,i1: u64) -> bool { self.window.is_some_and(|(w0,w1,_)| i0 >= w0 && i1 <= w1) }

    fn run(&mut self) -> Ended {
        let roll = self.roll();
        let basin = Roll::LAST >> ROLL_BASIN;
        loop {
            // What a stretch must be bounded below to be worth splitting further, for the goals
            // that search basins (the bound searches split to the grid and ask their own test).
            let below = match self.goal {
                Goal::Sign {tolerance} => -tolerance,
                Goal::Minimum {accuracy,relative,..} =>
                    self.best-accuracy.max(if self.best.is_finite() { relative*self.best.abs() } else { 0. }),
                Goal::AtLeast {..} | Goal::Floor {..} => f64::NAN,
            };
            if let Goal::Floor {slack,budget} = self.goal {
                // Peeked, not taken: the partition the bound is proven over keeps it.
                let Some(top) = self.open.peek() else { return Ended::Done };
                let low = top.0.min(self.best);
                if self.best-low <= slack || roll.evaluations() >= budget || top.3-top.1 < 2 { return Ended::Proven(low); }
            }
            let Some(s) = self.open.pop() else { return Ended::Done };
            match self.goal {
                Goal::AtLeast {cap,budget} => {
                    if s.0 >= cap { return Ended::Proven(s.0); }
                    if roll.evaluations() >= budget || s.3-s.1 < 2 { return Ended::Short; }
                    let fm = self.halve(s);
                    if !(fm >= cap) { return Ended::Short; }
                    continue;
                }
                Goal::Floor {..} => { let _ = self.halve(s); continue; }
                Goal::Sign {..} if s.0 > below => {
                    // every stretch left is above zero: a reading says how far, if none was made
                    if !self.best.is_finite() { self.best = self.read.at(s.1).min(self.read.at(s.3)); }
                    return Ended::Done;
                }
                Goal::Minimum {..} if s.0 > below => return Ended::Done,
                Goal::Minimum {..} if self.searched(s.1,s.3) => continue,
                _ => {}
            }
            let Stretch(low,i0,f0,i1,f1) = s;
            // A stretch carried from a cube is read here before it is split.
            if f0.is_nan() || f1.is_nan() {
                let (f0,f1) = (self.read.at(i0),self.read.at(i1));
                self.note(i0,f0);
                self.note(i1,f1);
                if self.decided() { return Ended::Done; }
                self.open.push(Stretch(self.bound(i0,f0,i1,f1,low),i0,f0,i1,f1));
                continue;
            }
            if i1-i0 <= basin {
                // A basin is searched only once every stretch still open is a basin wide: one
                // carried from a cube may be wider, and its interior is no reading's neighbour.
                if let Some(wide) = self.widest(basin,below) {
                    self.open.push(Stretch(low,i0,f0,i1,f1));
                    let _ = self.halve(wide);
                    if self.decided() { return Ended::Done; }
                    continue;
                }
                self.search_basins((i0,f0,i1,f1),below);
                return Ended::Done;
            }
            let _ = self.halve(Stretch(low,i0,f0,i1,f1));
            if self.decided() { return Ended::Done; }
        }
    }

    /// The stretches still able to meet the goal (bounded at or below `below`, outside the window
    /// searched), `first` among them, grouped into contiguous runs in roll order — each a basin
    /// whose minimum Brent's search finds about its lowest reading. That is a reading, not a bound:
    /// a second dip inside one run, between readings, can be missed.
    fn search_basins(&mut self,first: (u64,f64,u64,f64),below: f64) {
        let roll = self.roll();
        let mut open: Vec<(u64,f64,u64,f64)> = vec![first];
        let window = self.window;
        let rest: Vec<Stretch> = self.open.drain()
            .filter(|s| s.0 <= below && !window.is_some_and(|(w0,w1,_)| s.1 >= w0 && s.3 <= w1)).collect();
        open.extend(rest.into_iter().map(|s| (s.1,self.read.or(s.1,s.2),s.3,self.read.or(s.3,s.4))));
        match self.goal {
            Goal::Sign {tolerance} => for (readings,w0,w1) in basins(open) {
                if let Some(&(_,v)) = readings.iter().find(|r| r.1 < 0.) { self.best = v; return; }
                let (lo,hi) = (roll.time(w0),roll.time(w1));
                // settled once a reading is inside, or the least the minimum can be is outside
                let (found,_) = brent(&|t| roll.at_time(t),lo,hi,1e-6*(hi-lo),60,|v,slack| v < 0. || v-slack > tolerance);
                self.best = self.best.min(found);
                if found < 0. { self.best = found; return; }
            },
            Goal::Minimum {accuracy,relative,tie} => {
                // the accuracy the loop pruned by, as coarse as the best reading allows
                let accuracy = accuracy.max(if self.best.is_finite() { relative*self.best.abs() } else { 0. });
                let mut minima: Vec<(f64,f64)> = basins(open).into_iter()
                    .map(|(_,w0,w1)| minimum_between(roll,roll.time(w0),roll.time(w1),accuracy)).collect();
                // The window's minimum is a basin too. A run's beside it is the same basin carried
                // on past the window's edge (the contact moved further than the window reaches),
                // and the lower of the two is that basin's minimum.
                if let Some((w0,w1,found)) = self.window {
                    let reach = roll.time(w1)-roll.time(w0);
                    let mut basin_min = found;
                    let mut others = Vec::new();
                    for m in minima {
                        if (m.1-found.1).abs() <= reach { if m.0 < basin_min.0 { basin_min = m; } }
                        else { others.push(m); }
                    }
                    minima = others;
                    minima.push(basin_min);
                }
                minima.sort_by(|x,y| x.0.total_cmp(&y.0));
                if let Some(&(v,t)) = minima.first() { if v < self.best { self.best = v; self.best_t = t; } }
                self.tied = minima.len() > 1 && minima[1].0-minima[0].0 <= tie;
            }
            Goal::AtLeast {..} | Goal::Floor {..} => unreachable!("the bound searches split to the grid"),
        }
    }
}

/// Source evaluations a sweep may spend proving an operand cannot decide a reading before it is
/// read in full (`SweptField::query`): about what a warm reading's local search costs.
const AT_LEAST_BUDGET: u64 = 16;

/// The minimum in a bracket of the roll, by Brent's search, to `stop` of its value — the accuracy
/// asked, a thousandth of the value itself for a reading far from the boundary — or to 10⁻⁸ of the
/// bracket, where the value is within 10⁻¹⁶ of the bracket's rise, whichever comes first.
fn minimum_between(roll: &Roll,lo: f64,hi: f64,stop: f64) -> (f64,f64) {
    brent(&|t| roll.at_time(t),lo,hi,(1e-8*(hi-lo)).max(1e-15*(1.+hi.abs())),80,|_,slack| slack <= stop)
}

impl SweptField {
    pub fn new(source: SpatialField,motion: Family,domain: I) -> Self {
        Self {source,motion,domain,caches:Default::default()}
    }
    pub fn domain(&self) -> I { self.domain }

    /// The source read along `p`'s path through the roll (`Roll`).
    fn roll(&self,p: [f64;3]) -> Roll<'_> {
        let [a,b] = self.domain.bounds();
        let table = self.caches.poses.get_or_init(|| {
            let n = (1u64 << ROLL_DEPTH) as f64;
            (0..=1u64 << ROLL_LEVELS).map(|k| {
                let t = a+(b-a)*((k << (ROLL_DEPTH-ROLL_LEVELS)) as f64/n);
                self.motion.pose_at(t).ok().map(|m| m.inverse())
            }).collect()
        });
        Roll {field:self,p,a,b,table,evaluations:std::cell::Cell::new(0)}
    }

    /// Enclose every generating pose of the source's finite material support.
    /// The entire roll domain is passed through interval motion evaluation.
    pub fn support_bounds(&self) -> Result<Option<V>,Error> {
        self.source.support_bounds()?.map(|b| self.motion.bounds(self.domain)?.point(b)).transpose()
    }

    /// A lower bound at least `cap` on the sweep's value at `p`, when the roll search's first-order
    /// bounds, split lowest first until every one is at least `cap`, prove one (`Goal::AtLeast`).
    /// `None` as soon as a reading falls below `cap`, or after `budget` source evaluations: then
    /// the caller needs the minimum itself (`query`).
    fn at_least(&self,p: [f64;3],cap: f64,budget: u64) -> Option<f64> {
        let roll = self.roll(p);
        let (fa,fb) = (roll.at(0),roll.at(Roll::LAST));
        if !(fa.min(fb) >= cap) { return None; }
        if !(roll.b > roll.a) { return Some(fa.min(fb)); }
        let speed = self.motion.inverse_point_speed_bound(p,self.domain).ok()?;
        let mut search = RollSearch::new(&roll,Goal::AtLeast {cap,budget},Some(speed),false);
        search.seed_ends(fa,fb);
        match search.run() { Ended::Proven(low) => Some(low), _ => None }
    }

    /// A lower bound on the sweep at `p` read from a table: the bound proven over the roll at the
    /// centre of the cube `p` is in (`Cube`), less `p`'s distance from that centre — the field is
    /// one-Lipschitz, a minimum over rigid motions of a one-Lipschitz source. The first query in a
    /// cube pays for its bound and every later one, from any indexed copy of the cut, looks it
    /// up. `None` without a finite source to size the cubes by.
    fn floor(&self,p: [f64;3]) -> Option<f64> {
        // the bound alone, read under the lock: no cube handle taken out for a known cube
        let grid = Grid(self.cube_side());
        if !grid.usable() { return None; }
        let key = grid.key(p);
        let low = self.caches.floors.lock().get(&(key,1)).map(|c| c.low);
        match low {
            Some(low) => Some(low-crate::space::distance(p,grid.centre(key))),
            None => self.cube(p).map(|(d,c)| c.low-d),
        }
    }

    /// The cube `p` is in, filled on first asking, and `p`'s distance from its centre.
    fn cube(&self,p: [f64;3]) -> Option<(f64,Arc<Cube>)> { self.cube_of(p,1) }

    /// `floor` from cubes `coarsening` times as wide: a weaker bound, by the farther centre, from
    /// a table with that cube fewer entries — for sorting out which of many placed copies of a
    /// sweep can matter near a point, where most are far and each bound is asked once.
    pub(super) fn coarse_floor(&self,p: [f64;3],coarsening: u32) -> Option<f64> {
        self.cube_of(p,coarsening).map(|(d,c)| c.low-d)
    }

    fn cube_of(&self,p: [f64;3],coarsening: u32) -> Option<(f64,Arc<Cube>)> {
        let grid = Grid(self.cube_side()*coarsening as f64);
        if !grid.usable() { return None; }
        let key = grid.key(p);
        let centre = grid.centre(key);
        let d = crate::space::distance(p,centre);
        let known = self.caches.floors.lock().get(&(key,coarsening)).cloned();
        let cube = match known {
            Some(c) => c,
            None => {
                let c = Arc::new(self.bound_at(centre,0.5*grid.0,FLOOR_BUDGET,2.*grid.0));
                self.caches.floors.lock().insert((key,coarsening),c.clone());
                c
            }
        };
        Some((d,cube))
    }

    /// The side of the floor table's cubes: the source's diagonal over `FLOOR_CUBES`, or 0 without
    /// a finite source.
    pub(super) fn cube_side(&self) -> f64 {
        *self.caches.cube.get_or_init(|| self.source.support_bounds().ok().flatten().map_or(0.,|b| {
            let d: f64 = b.iter().map(|x| { let [lo,hi] = x.bounds(); (hi-lo)*(hi-lo) }).sum();
            d.sqrt()/FLOOR_CUBES
        }))
    }

    /// A proven lower bound on the sweep at `p` and the partition of the roll it was proven over:
    /// the roll search's first-order bounds, lowest stretch split first, stopped when the lowest
    /// bound is within `slack` of the lowest reading or after `budget` source evaluations
    /// (`Goal::Floor`). Contiguous stretches are merged into runs under their least bound, those
    /// bounded `keep` or more above the least apart from those that are not: a run near each
    /// contact, and the rest of the roll in the runs between, which a point in the cube dismisses
    /// without reading. Always a lower bound however soon it stops; minus infinity, over the whole
    /// roll, where the motion gives no speed bound.
    fn bound_at(&self,p: [f64;3],slack: f64,budget: u64,keep: f64) -> Cube {
        let roll = self.roll(p);
        let last = Roll::LAST;
        let whole = |low: f64| Cube {low,stretches:vec![(low,0,last)].into()};
        let (fa,fb) = (roll.at(0),roll.at(last));
        if !(roll.b > roll.a) { return whole(fa.min(fb)); }
        let Ok(speed) = self.motion.inverse_point_speed_bound(p,self.domain) else { return whole(f64::NEG_INFINITY) };
        let mut search = RollSearch::new(&roll,Goal::Floor {slack,budget},Some(speed),false);
        search.best = fa.min(fb);
        search.seed_ends(fa,fb);
        let Ended::Proven(low) = search.run() else { unreachable!("a floor search always has a stretch open") };
        // In roll order, the stretches near the least kept apart and the rest merged.
        let mut stretches = std::mem::take(&mut search.open).into_vec();
        stretches.sort_by_key(|s| s.1);
        let mut kept: Vec<(f64,u64,u64)> = Vec::new();
        for &Stretch(bl,i0,_,i1,_) in &stretches {
            match kept.last_mut() {
                Some(prev) if (prev.0 >= low+keep) == (bl >= low+keep) && prev.2 == i0 => {
                    prev.0 = prev.0.min(bl); prev.2 = i1;
                }
                _ => kept.push((bl,i0,i1)),
            }
        }
        Cube {low,stretches:kept.into()}
    }

    /// The sweep's value and gradient at `p` read from its adaptive distance field (`adf.rs`),
    /// refined to `resolution`: exact at the octree's corners (found to a hundredth of the
    /// tolerance) and interpolated between, refined only where the surface may pass. A reading
    /// for a mesher that accepts the tolerance, never an interval claim; `None` without a finite
    /// source to size the root cells by.
    fn cached(&self,p: [f64;3],resolution: super::adf::Resolution) -> Option<(f64,[f64;3])> {
        let h = self.cube_side();
        if !(h > 0.) || !h.is_finite() || !(resolution.finest > 0.) { return None; }
        // a root cell sixteen of the floor's cubes across: far from the surface a read is one cell
        let root = 16.*h;
        let accuracy = 1e-2*resolution.tolerance;
        let mut adfs = self.caches.adfs.lock();
        let key = memo::bits([resolution.finest,resolution.coarsest,resolution.tolerance]);
        let adf = adfs.entry(key).or_insert_with(|| super::adf::Adf::new(root,resolution));
        // a corner far from the boundary needs its value only to a hundredth of itself
        Some(adf.read(p,&mut |q| self.minimum_relative(q,accuracy,1e-2,0.).value))
    }

    /// How far `p` stands outside the box the whole sweep lies in, when it does: then no pose of
    /// the source reaches it, and the distance is a lower bound on the field there (it is
    /// one-Lipschitz and the material is inside the box), positive, with the field's sign. A cut
    /// indexed round a blank is far from most points asked about, and reading this costs nothing.
    pub(super) fn clear_of(&self,p: [f64;3]) -> Option<f64> {
        let support = (*self.caches.support.get_or_init(|| self.support_bounds().ok().flatten().map(|b| b.map(|x| x.bounds()))))?;
        let d2: f64 = (0..3).map(|k| (support[k][0]-p[k]).max(p[k]-support[k][1]).max(0.).dpowi(2)).sum();
        (d2 > 0.).then(|| d2.sqrt())
    }

    /// A number with the field's sign at a point (`Want::Sign`), in plain floating point, for a
    /// mesher that asks only which side a point is on: never an interval claim, and its magnitude only an
    /// upper bound on the field. The roll search (`Goal::Sign`) splits stretches lowest bound first
    /// until a reading is negative (inside), every bound is within a ten-billionth of the point's
    /// size of zero (a tie), or the stretches left are a basin wide: far from the boundary one
    /// bound decides, and near it the work grows as the logarithm of the distance.
    ///
    /// Near a rolling contact the path runs along the tool, so the source rises slowly away from
    /// its minimum: a long shallow basin that a first-order bound splits stretch by stretch at
    /// every level. So splitting stops at `ROLL_BASIN`, and each basin's minimum is found by
    /// Brent's search around its lowest reading — a reading, not a bound: a second dip inside one
    /// run, between readings, can be missed.
    ///
    /// With a `resolution`, a point the box and the floor table leave open reads its sign from the
    /// adaptive distance field (`cached`) before any search: a mesher's sign, off the exact field's
    /// only within about the resolution's tolerance of the boundary.
    fn sign(&self,p: [f64;3],cached: Option<super::Resolution>) -> f64 {
        if let Some(d) = self.clear_of(p) { return d; }
        if let Some(resolution) = cached {
            // the floor alone, read under the lock, before the octree is walked
            if let Some(low) = self.floor(p) { if low > 0. { return low; } }
            if let Some((v,_)) = self.cached(p,resolution) { return v; }
        }
        let seed = self.cube(p);
        if let Some((d,c)) = &seed { if c.low-d > 0. { return c.low-d; } }
        let roll = self.roll(p);
        // A bound this close to zero leaves the side a tie a mesher's bisection resolves by
        // position; refining it further buys a sign nothing downstream can use.
        let tolerance = 1e-10*(1.+crate::space::norm(p));
        let mut search = RollSearch::new(&roll,Goal::Sign {tolerance},None,true);
        match &seed {
            Some((d,c)) => search.seed_cube(*d,c),
            None => {
                let (fa,fb) = (search.read.at(0),search.read.at(Roll::LAST));
                search.best = fa.min(fb);
                if !(roll.b > roll.a) || search.best < 0. { return search.best; }
                search.seed_ends(fa,fb);
            }
        }
        search.run();
        search.best
    }

    /// The field's value at a point and the roll time it is least at, in plain floating point —
    /// `side`'s search carried on until no stretch of roll can read more than `accuracy` below the
    /// best reading (or `relative` of its own size, where that is coarser), where `side` stops at
    /// the first negative one (`Goal::Minimum`). A Newton step from a point far from the boundary
    /// needs its value to a few digits, and the search stops as soon as no stretch can read that
    /// much lower. The value is a reading, not an interval claim: a stretch is bounded by the
    /// motion's inverse-point speed, and a basin is searched by Brent's method around its lowest
    /// reading, so a second dip inside one basin can be missed. `tied` is set when the minima of
    /// two separate basins are within `tie` of each other: two contact times, which is a crease of
    /// the swept surface.
    fn minimum_relative(&self,p: [f64;3],accuracy: f64,relative: f64,tie: f64) -> SweptMinimum {
        self.minimum_hinted(p,accuracy,relative,tie,None,false)
    }

    /// `minimum_relative` warm-started from `hint`, the contact time at a nearby point: the
    /// basin-wide window about it (2^-`ROLL_BASIN` of the roll) is searched first, and its
    /// minimum is the best reading the bounded search over the whole roll starts from — so every
    /// stretch that cannot beat it is pruned as `side` prunes, the window itself is not searched
    /// again, and a deeper minimum anywhere else is still found. A hint far from the contact
    /// costs the window's search and nothing in correctness.
    ///
    /// `local` trusts the window: its minimum is returned without the search over the whole roll,
    /// unless it lies at the window's edge (the contact moved further than the window reaches),
    /// when the whole roll is searched after all. That is a continuation, not a minimum: a deeper
    /// contact elsewhere is not looked for, and a caller that takes it must check its conclusions
    /// another way (the refinement checks a crossing's bracket by `side`).
    fn minimum_hinted(&self,p: [f64;3],accuracy: f64,relative: f64,tie: f64,hint: Option<f64>,local: bool) -> SweptMinimum {
        let roll = self.roll(p);
        let (a,b) = (roll.a,roll.b);
        let last = Roll::LAST;
        let seed = if b > a { self.cube(p) } else { None };
        let goal = Goal::Minimum {accuracy,relative,tie};
        let mut search = RollSearch::new(&roll,goal,None,true);
        let (fa,fb) = if seed.is_some() { (f64::INFINITY,f64::INFINITY) } else { (search.read.at(0),search.read.at(last)) };
        (search.best,search.best_t) = if fa <= fb { (fa,a) } else { (fb,b) };
        let done = |s: &RollSearch| SweptMinimum {value:s.best,time:s.best_t,tied:s.tied};
        if !(b > a) { return done(&search); }
        let basin = last >> ROLL_BASIN;
        // The hint's window, in grid indices, searched first.
        if let Some(t) = hint.filter(|t| t.is_finite() && *t >= a && *t <= b) {
            let centre = ((t-a)/(b-a)*last as f64).round().clamp(0.,last as f64) as u64;
            let (w0,w1) = (centre.saturating_sub(basin/2),(centre+basin/2).min(last));
            let found = minimum_between(&roll,roll.time(w0),roll.time(w1),accuracy);
            if found.0 < search.best { (search.best,search.best_t) = found; }
            search.window = Some((w0,w1,found));
            if local {
                let (lo,hi) = (roll.time(w0),roll.time(w1));
                if !at_edge(found.1,lo,hi,w0 > 0,w1 < last,0.02*(hi-lo)) {
                    return SweptMinimum {value:found.0,time:found.1,tied:false};
                }
            }
        }
        let Ok(speed) = self.motion.inverse_point_speed_bound(p,self.domain) else { return done(&search) };
        let _ = search.speed.set(speed);
        match &seed {
            Some((d,c)) => search.seed_cube(*d,c),
            None => search.seed_ends(fa,fb),
        }
        search.run();
        done(&search)
    }

    /// The sweep's half of a point query (`MaterialField::query`), in the one order every query
    /// takes: the box the whole sweep lies in and the floor table, which settle most points for
    /// nothing; then, asked for them, the adaptive distance field's values (`Source::Cached`); then
    /// the search over the roll. A sign stops at the first of these that settles it. A reading at
    /// `cap` or more is left a bound (`Reading::bound`) — a Boolean asks each operand only below
    /// what already decides it — proven by the box, the floor or the roll search's first-order
    /// bounds, a few evaluations, where the minimum is a whole search. Otherwise it is the tool's
    /// own reading at the roll time the sweep is least at, turned into the world: its value is the
    /// minimum, and so is its gradient (the envelope theorem). `hint` is the contact time at a
    /// nearby point (`Source::Warm`), left holding this point's; the source's leaves are numbered
    /// from `first`.
    pub(super) fn query(&self,p: [f64;3],q: &Query,cap: f64,hint: &mut Option<f64>,first: usize) -> Reading {
        let cached = match q.source { Source::Cached(resolution) => Some(resolution),_ => None };
        if q.want == Want::Sign { return Reading::bound(self.sign(p,cached)); }
        if cap.is_finite() {
            let low = self.clear_of(p).filter(|&d| d >= cap).or_else(|| self.floor(p).filter(|&f| f >= cap));
            if let Some(low) = low { return Reading::bound(low); }
        }
        if let Some(resolution) = cached {
            if let Some((value,gradient)) = self.cached(p,resolution) { return Reading {gradient,..Reading::bound(value)}; }
        }
        if cap.is_finite() {
            if let Some(low) = self.at_least(p,cap,AT_LEAST_BUDGET) { return Reading::bound(low); }
        }
        let local = matches!(q.source,Source::Warm {local:true,..});
        let m = self.minimum_hinted(p,q.accuracy,q.relative,q.tie,*hint,local);
        *hint = Some(m.time);
        // a pose that cannot be read names no operand: the value alone, flagged
        let Some(inverse) = self.motion.pose_at(m.time).ok().map(|x| x.inverse()) else {
            return Reading {ambiguous:true,..Reading::bound(m.value)};
        };
        let r = self.source.reading(inverse.point(p),q,first);
        Reading {value:m.value,gradient:inverse.gradient(r.gradient),operand:r.operand.map(|o| o.at_time(Some(m.time))),
            ambiguous:r.ambiguous || m.tied}
    }

    /// The source, for a reading that follows a minimum to the tool's own gradient.
    pub(crate) fn source(&self) -> &SpatialField { &self.source }
    pub(crate) fn motion(&self) -> &Family { &self.motion }

    /// Numerical storage is an evaluator control, not a geometry parameter.
    /// Zero disables caching. Reaching the cap merely recomputes later poses;
    /// it cannot discard motion intervals or change the resulting enclosure.
    pub fn evaluator(&self,max_cached_poses: usize) -> SweepEvaluator {
        SweepEvaluator {field:self.clone(),poses:BTreeMap::new(),max_cached_poses}
    }
}

/// Reusable query state for one immutable swept field. The pose cache belongs
/// to that exact field/motion snapshot and has an explicit capacity.
pub struct SweepEvaluator {
    field:SweptField,
    poses:BTreeMap<u64,MotionBounds>,
    max_cached_poses:usize,
}

impl SweepEvaluator {
    pub fn domain(&self) -> I { self.field.domain }
    pub fn cached_poses(&self) -> usize { self.poses.len() }
    pub fn clear_cache(&mut self) { self.poses.clear(); }

    /// `query` to convergence, unobserved.
    pub fn bounds(&mut self,p: V,options: Options) -> Result<Minimum,SweepError> {
        self.query(p,Stop::Converged,options,None)
    }

    /// Enclose the swept field for every point in the input box, the roll search stopping as
    /// `stop` allows (see `minimum::Stop`): `Converged` to the value tolerance, a field width and
    /// not a geometric export tolerance; `Outside(band)` once the enclosure is strictly outside the
    /// band, which retains bounds and an attained witness and claims no convergence. Termination
    /// status and uncertainty are retained. `observe` sees the raw interval-oracle enclosures, e.g.
    /// to extract independently checkable coverage evidence; it supplies no geometry, bounds or
    /// pruning decisions and cannot change the result.
    pub fn query(&mut self,p: V,stop: Stop,options: Options,mut observe: Option<&mut dyn FnMut(I,I)>)
        -> Result<Minimum,SweepError> {
        // The motion's speed bound over the whole input box.
        let speed = self.field.motion.inverse_point_speed_bound_over(p,self.field.domain)
            .and_then(I::point).map_err(minimum::Error::Oracle)?;
        // The refiner bounds a new cell and then samples the cell's midpoint,
        // and both are the source at the pose of that one midpoint: the last
        // midpoint's value is kept, and only the travel differs.
        let mut last: Option<(u64,I)> = None;
        minimum::refine(self.field.domain,|t| {
            let [lo,hi] = t.bounds(); let mid = lo*0.5+hi*0.5;
            let key = mid.to_bits();
            let value = match last {
                Some((k,value)) if k == key => value,
                _ => {
                    let pose = if let Some(pose) = self.poses.get(&key) { *pose } else {
                        let pose = self.field.motion.bounds(I::point(mid)?)?;
                        if self.poses.len() < self.max_cached_poses { self.poses.insert(key,pose); }
                        pose
                    };
                    let value = self.field.source.bounds(pose.inverse_point(p)?)?;
                    last = Some((key,value));
                    value
                }
            };
            let dt = t.sub(I::point(mid)?)?.bounds();
            let travel = speed.mul(I::point(dt[0].abs().max(dt[1].abs()))?)?.bounds()[1];
            // SpatialField's constructors establish the one-Lipschitz contract.
            // No arbitrary value callback or assumed evaluation-error band enters.
            let bound = value.add(I::new(-travel,travel)?)?;
            if let Some(o) = observe.as_mut() { o(t,bound); }
            Ok::<_,Error>(bound)
        },options,stop)
    }
}
