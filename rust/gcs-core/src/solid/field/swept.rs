//! Continuous volume sweeps of explicit one-Lipschitz material fields.
use super::{SpatialField,Error,I,V};
use crate::{interval::minimum::{self,Minimum,Options,Stop},motion::{Family,MotionBounds}};
use std::collections::BTreeMap;

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
pub struct SweptField {source:SpatialField,motion:Family,domain:I,
    /// The inverse poses at the first `ROLL_LEVELS` dyadic divisions of the roll, which every
    /// search reads (`Roll`), filled once.
    poses:std::sync::OnceLock<std::sync::Arc<Vec<Option<crate::motion::Pose>>>>,
    /// The box every pose of the source lies in, as plain numbers, found once: a point outside
    /// it is outside the material (`clear_of`).
    support:std::sync::OnceLock<Option<[[f64;2];3]>>,
    /// Proven lower bounds of the sweep at the centres of cubes a `floor` query has reached, keyed
    /// by cube, filled as they are asked for and shared by every clone (every indexed copy of one
    /// cut reads the one table, each at its own point turned into the sweep's frame).
    floors:std::sync::Arc<std::sync::Mutex<super::adf::Map<([i32;3],u32),std::sync::Arc<Cube>>>>,
    /// The side of those cubes: a fraction of the source's own size.
    cube:std::sync::OnceLock<f64>,
    /// Adaptive distance fields of the sweep, by the resolution asked for (`cached`), shared by
    /// every clone as the cubes are.
    adfs:std::sync::Arc<std::sync::Mutex<std::collections::BTreeMap<[u64;3],super::adf::Adf>>>}

/// A sweep's partition of its roll at one cube's centre (`SweptField::floor`): the least bound
/// proven over the roll, and each stretch `(bound, start, end)` in dyadic roll indices. At a point
/// a distance d from the centre every bound holds less d, the motion being rigid.
#[derive(Debug)]
struct Cube { low: f64,stretches: Box<[(f64,u64,u64)]> }

/// The least of `f` on [lo, hi] by Brent's method — parabolic steps through the three best points,
/// golden section where a parabola is not to be trusted — until the bracket is within `xtol` or
/// `stop` accepts a value and how far below it the minimum may lie: `(value, argument)`. The
/// minimum of a smooth function is found in a few steps where golden section alone takes one per
/// 0.62 of the bracket.
///
/// How far below the best value the minimum may lie is read off the parabola through the three
/// best points: its curvature times the bracket's width squared, over two — infinite until three
/// points make a parabola that holds water. A reading, like golden section's, not a bound.
pub(super) fn brent(f: &impl Fn(f64) -> f64,lo: f64,hi: f64,xtol: f64,steps: usize,stop: impl Fn(f64,f64) -> bool) -> (f64,f64) {
    const GOLD: f64 = 0.381_966_011_250_105_1;
    let (mut a,mut b) = (lo,hi);
    let mut x = a+GOLD*(b-a);
    let (mut w,mut v) = (x,x);
    let mut fx = f(x);
    let (mut fw,mut fv) = (fx,fx);
    let (mut d,mut e) = (0f64,0f64);
    let slack = |x: f64,fx: f64,w: f64,fw: f64,v: f64,fv: f64,width: f64| -> f64 {
        if x == w || x == v || w == v { return f64::INFINITY; }
        let curvature = 2.*((fw-fx)/(w-x)-(fv-fx)/(v-x))/(w-v);
        if curvature > 0. && curvature.is_finite() { 0.5*curvature*width*width } else { f64::INFINITY }
    };
    for _ in 0..steps {
        if stop(fx,slack(x,fx,w,fw,v,fv,b-a)) { break; }
        let xm = 0.5*(a+b);
        let (tol1,tol2) = (xtol,2.*xtol);
        if (x-xm).abs() <= tol2-0.5*(b-a) { break; }
        let mut golden = true;
        if e.abs() > tol1 {
            let r = (x-w)*(fx-fv);
            let mut q = (x-v)*(fx-fw);
            let mut p = (x-v)*q-(x-w)*r;
            q = 2.*(q-r);
            if q > 0. { p = -p; }
            q = q.abs();
            let previous = e;
            e = d;
            if p.abs() < (0.5*q*previous).abs() && p > q*(a-x) && p < q*(b-x) {
                d = p/q;
                let u = x+d;
                if u-a < tol2 || b-u < tol2 { d = tol1.copysign(xm-x); }
                golden = false;
            }
        }
        if golden { e = if x >= xm { a-x } else { b-x }; d = GOLD*e; }
        let u = if d.abs() >= tol1 { x+d } else { x+tol1.copysign(d) };
        let fu = f(u);
        if fu <= fx {
            if u >= x { a = x } else { b = x }
            (v,fv,w,fw,x,fx) = (w,fw,x,fx,u,fu);
        } else {
            if u < x { a = u } else { b = u }
            if fu <= fw || w == x { (v,fv,w,fw) = (w,fw,u,fu); }
            else if fu <= fv || v == x || v == w { (v,fv) = (u,fu); }
        }
    }
    (fx,x)
}

/// A stretch of roll between two dyadic indices: a lower bound on the field over it, and the
/// readings at its ends (NaN until made). Ordered lowest bound first in a heap.
#[derive(PartialEq)]
struct Stretch(f64,u64,f64,u64,f64);
impl Eq for Stretch {}
impl PartialOrd for Stretch { fn partial_cmp(&self,o: &Self) -> Option<std::cmp::Ordering> { Some(self.cmp(o)) } }
impl Ord for Stretch { fn cmp(&self,o: &Self) -> std::cmp::Ordering { o.0.total_cmp(&self.0) } }

/// The widest stretch still open (bounded at or below `below`) that is wider than a basin, taken
/// out of the heap to be split.
fn widest(stretches: &mut std::collections::BinaryHeap<Stretch>,basin: u64,below: f64) -> Option<Stretch> {
    let k = stretches.iter().enumerate().filter(|(_,s)| s.0 <= below && s.3-s.1 > basin)
        .max_by_key(|(_,s)| s.3-s.1).map(|(k,_)| k)?;
    let mut all = std::mem::take(stretches).into_vec();
    let wide = all.swap_remove(k);
    *stretches = all.into();
    Some(wide)
}

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

/// Split the stretch `wide` at its middle: its three readings (ends and middle), made where not
/// in hand, and its two halves under `bound`.
fn split(read: &Reads,wide: Stretch,bound: &dyn Fn(u64,f64,u64,f64,f64) -> f64) -> ([(u64,f64);3],[Stretch;2]) {
    let Stretch(wl,w0,g0,w1,g1) = wide;
    let (g0,g1) = (read.or(w0,g0),read.or(w1,g1));
    let wm = w0+(w1-w0)/2;
    let gm = read.at(wm);
    ([(w0,g0),(w1,g1),(wm,gm)],
        [Stretch(bound(w0,g0,wm,gm,wl),w0,g0,wm,gm),Stretch(bound(wm,gm,w1,g1,wl),wm,gm,w1,g1)])
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

impl SweptField {
    pub fn new(source: SpatialField,motion: Family,domain: I) -> Self {
        Self {source,motion,domain,poses:std::sync::OnceLock::new(),support:std::sync::OnceLock::new(),
            floors:Default::default(),cube:std::sync::OnceLock::new(),adfs:Default::default()}
    }
    pub fn domain(&self) -> I { self.domain }

    /// The source read along `p`'s path through the roll (`Roll`).
    fn roll(&self,p: [f64;3]) -> Roll<'_> {
        let [a,b] = self.domain.bounds();
        let table = self.poses.get_or_init(|| {
            let n = (1u64 << ROLL_DEPTH) as f64;
            std::sync::Arc::new((0..=1u64 << ROLL_LEVELS).map(|k| {
                let t = a+(b-a)*((k << (ROLL_DEPTH-ROLL_LEVELS)) as f64/n);
                self.motion.pose_at(t).ok().map(|m| m.inverse())
            }).collect())
        });
        Roll {field:self,p,a,b,table,evaluations:std::cell::Cell::new(0)}
    }

    /// Enclose every generating pose of the source's finite material support.
    /// The entire roll domain is passed through interval motion evaluation.
    pub fn support_bounds(&self) -> Result<Option<V>,Error> {
        self.source.support_bounds()?.map(|b| self.motion.bounds(self.domain)?.point(b)).transpose()
    }

    /// A lower bound at least `cap` on the sweep's value at `p`, when one is cheap to prove: the
    /// support box's distance, or `side`'s first-order bound — the source is Lipschitz in the roll
    /// by the motion's inverse-point speed bound, so a stretch between two readings is at least
    /// their mean less that bound times half its width — split lowest first until every stretch
    /// is at least `cap`. `None` as soon as a reading falls below `cap`, or after `budget` source
    /// evaluations: then the caller needs the minimum itself. A Boolean asks this of an operand
    /// that can only matter below `cap` (`MaterialField::reading_capped`).
    pub(crate) fn at_least(&self,p: [f64;3],cap: f64,budget: u64) -> Option<f64> {
        if let Some(d) = self.clear_of(p) { if d >= cap { return Some(d); } }
        if let Some(f) = self.floor(p) { if f >= cap { return Some(f); } }
        let roll = self.roll(p);
        let last = Roll::LAST;
        let (fa,fb) = (roll.at(0),roll.at(last));
        if !(fa.min(fb) >= cap) { return None; }
        if !(roll.b > roll.a) { return Some(fa.min(fb)); }
        let speed = self.motion.inverse_point_speed_bound(p,self.domain).ok()?;
        let bound = |i0: u64,f0: f64,i1: u64,f1: f64| roll.bound(speed,i0,f0,i1,f1);
        // lowest bound first, as `side` orders them
        let mut stretches: Vec<(f64,u64,f64,u64,f64)> = vec![(bound(0,fa,last,fb),0,fa,last,fb)];
        loop {
            let k = (0..stretches.len()).min_by(|&x,&y| stretches[x].0.total_cmp(&stretches[y].0))?;
            let (low,i0,f0,i1,f1) = stretches.swap_remove(k);
            if low >= cap { return Some(low); }
            if roll.evaluations() >= budget || i1-i0 < 2 { return None; }
            let m = i0+(i1-i0)/2;
            let fm = roll.at(m);
            if !(fm >= cap) { return None; }
            stretches.push((bound(i0,f0,m,fm),i0,f0,m,fm));
            stretches.push((bound(m,fm,i1,f1),m,fm,i1,f1));
        }
    }

    /// A lower bound on the sweep at `p` read from a table: the bound proven over the roll at the
    /// centre of the cube `p` is in (`Cube`), less `p`'s distance from that centre — the field is
    /// one-Lipschitz, a minimum over rigid motions of a one-Lipschitz source. The first query in a
    /// cube pays for its bound and every later one, from any indexed copy of the cut, looks it
    /// up. `None` without a finite source to size the cubes by.
    pub(crate) fn floor(&self,p: [f64;3]) -> Option<f64> {
        // the bound alone, read under the lock: no cube handle taken out for a known cube
        let h = self.cube_side();
        if !(h > 0.) || !h.is_finite() { return None; }
        let key = p.map(|x| (x/h).floor().clamp(i32::MIN as f64,i32::MAX as f64) as i32);
        let low = self.floors.lock().ok()?.get(&(key,1)).map(|c| c.low);
        match low {
            Some(low) => {
                let centre = key.map(|k| (k as f64+0.5)*h);
                Some(low-((p[0]-centre[0]).powi(2)+(p[1]-centre[1]).powi(2)+(p[2]-centre[2]).powi(2)).sqrt())
            }
            None => self.cube(p).map(|(d,c)| c.low-d),
        }
    }

    /// The cube `p` is in, filled on first asking, and `p`'s distance from its centre.
    fn cube(&self,p: [f64;3]) -> Option<(f64,std::sync::Arc<Cube>)> { self.cube_of(p,1) }

    /// `floor` from cubes `coarsening` times as wide: a weaker bound, by the farther centre, from
    /// a table with that cube fewer entries — for sorting out which of many placed copies of a
    /// sweep can matter near a point, where most are far and each bound is asked once.
    pub(crate) fn coarse_floor(&self,p: [f64;3],coarsening: u32) -> Option<f64> {
        self.cube_of(p,coarsening).map(|(d,c)| c.low-d)
    }

    fn cube_of(&self,p: [f64;3],coarsening: u32) -> Option<(f64,std::sync::Arc<Cube>)> {
        let h = self.cube_side()*coarsening as f64;
        if !(h > 0.) || !h.is_finite() { return None; }
        let key = p.map(|x| (x/h).floor().clamp(i32::MIN as f64,i32::MAX as f64) as i32);
        let centre = key.map(|k| (k as f64+0.5)*h);
        let d = ((p[0]-centre[0]).powi(2)+(p[1]-centre[1]).powi(2)+(p[2]-centre[2]).powi(2)).sqrt();
        let known = self.floors.lock().ok()?.get(&(key,coarsening)).cloned();
        let cube = match known {
            Some(c) => c,
            None => {
                let c = std::sync::Arc::new(self.bound_at(centre,0.5*h,FLOOR_BUDGET,2.*h));
                self.floors.lock().ok()?.insert((key,coarsening),c.clone());
                c
            }
        };
        Some((d,cube))
    }

    /// The side of the floor table's cubes: the source's diagonal over `FLOOR_CUBES`, or 0 without
    /// a finite source.
    pub(super) fn cube_side(&self) -> f64 {
        *self.cube.get_or_init(|| self.source.support_bounds().ok().flatten().map_or(0.,|b| {
            let d: f64 = b.iter().map(|x| { let [lo,hi] = x.bounds(); (hi-lo)*(hi-lo) }).sum();
            d.sqrt()/FLOOR_CUBES
        }))
    }

    /// A proven lower bound on the sweep at `p` and the partition of the roll it was proven over:
    /// `side`'s first-order bound search, lowest stretch split first, stopped when the lowest bound
    /// is within `slack` of the lowest reading or after `budget` source evaluations. Contiguous
    /// stretches are merged into runs under their least bound, those bounded `keep` or more above
    /// the least apart from those that are not: a run near each contact, and the rest of the roll
    /// in the runs between, which a point in the cube dismisses without reading. Always a lower bound however soon it
    /// stops; minus infinity, over the whole roll, where the motion gives no speed bound.
    fn bound_at(&self,p: [f64;3],slack: f64,budget: u64,keep: f64) -> Cube {
        let roll = self.roll(p);
        let last = Roll::LAST;
        let whole = |low: f64| Cube {low,stretches:vec![(low,0,last)].into()};
        let (fa,fb) = (roll.at(0),roll.at(last));
        let mut best = fa.min(fb);
        if !(roll.b > roll.a) { return whole(best); }
        let Ok(speed) = self.motion.inverse_point_speed_bound(p,self.domain) else { return whole(f64::NEG_INFINITY) };
        let bound = |i0: u64,f0: f64,i1: u64,f1: f64| roll.bound(speed,i0,f0,i1,f1);
        let mut stretches: Vec<(f64,u64,f64,u64,f64)> = vec![(bound(0,fa,last,fb),0,fa,last,fb)];
        loop {
            let k = (0..stretches.len()).min_by(|&x,&y| stretches[x].0.total_cmp(&stretches[y].0)).unwrap();
            let low = stretches[k].0.min(best);
            if best-low <= slack || roll.evaluations() >= budget || stretches[k].3-stretches[k].1 < 2 {
                // In roll order, the stretches near the least kept apart and the rest merged.
                stretches.sort_by_key(|s| s.1);
                let mut kept: Vec<(f64,u64,u64)> = Vec::new();
                for &(bl,i0,_,i1,_) in &stretches {
                    match kept.last_mut() {
                        Some(prev) if (prev.0 >= low+keep) == (bl >= low+keep) && prev.2 == i0 => {
                            prev.0 = prev.0.min(bl); prev.2 = i1;
                        }
                        _ => kept.push((bl,i0,i1)),
                    }
                }
                return Cube {low,stretches:kept.into()};
            }
            let (_,i0,f0,i1,f1) = stretches.swap_remove(k);
            let m = i0+(i1-i0)/2;
            let fm = roll.at(m);
            best = best.min(fm);
            stretches.push((bound(i0,f0,m,fm),i0,f0,m,fm));
            stretches.push((bound(m,fm,i1,f1),m,fm,i1,f1));
        }
    }

    /// The sweep's value and gradient at `p` read from its adaptive distance field (`adf.rs`),
    /// refined to `resolution`: exact at the octree's corners (found to a hundredth of the
    /// tolerance) and interpolated between, refined only where the surface may pass. A reading
    /// for a mesher that accepts the tolerance, never an interval claim; `None` without a finite
    /// source to size the root cells by.
    pub(crate) fn cached(&self,p: [f64;3],resolution: super::adf::Resolution) -> Option<(f64,[f64;3])> {
        let h = self.cube_side();
        if !(h > 0.) || !h.is_finite() || !(resolution.finest > 0.) { return None; }
        // a root cell sixteen of the floor's cubes across: far from the surface a read is one cell
        let root = 16.*h;
        let accuracy = 1e-2*resolution.tolerance;
        let mut adfs = self.adfs.lock().ok()?;
        let key = [resolution.finest,resolution.coarsest,resolution.tolerance].map(f64::to_bits);
        let adf = adfs.entry(key).or_insert_with(|| super::adf::Adf::new(root,resolution));
        // a corner far from the boundary needs its value only to a hundredth of itself
        Some(adf.read(p,&mut |q| self.minimum_relative(q,accuracy,1e-2,0.).value))
    }

    /// How far `p` stands outside the box the whole sweep lies in, when it does: then no pose of
    /// the source reaches it, and the distance is a lower bound on the field there (it is
    /// one-Lipschitz and the material is inside the box), positive, with the field's sign. A cut
    /// indexed round a blank is far from most points asked about, and reading this costs nothing.
    pub(crate) fn clear_of(&self,p: [f64;3]) -> Option<f64> {
        let support = (*self.support.get_or_init(|| self.support_bounds().ok().flatten().map(|b| b.map(|x| x.bounds()))))?;
        let d2: f64 = (0..3).map(|k| (support[k][0]-p[k]).max(p[k]-support[k][1]).max(0.).powi(2)).sum();
        (d2 > 0.).then(|| d2.sqrt())
    }

    /// A number with the field's sign at a point, in plain floating point, for a mesher that
    /// asks only which side a point is on: never an interval claim, and its magnitude only an
    /// upper bound on the field. Along the point's path in the tool's frame the source's `value`
    /// is Lipschitz in the roll by the motion's inverse-point speed bound, so a stretch of roll
    /// between two readings is at least their mean less that bound times half its width. The
    /// stretches are split lowest bound first until a reading is negative (inside), every bound
    /// is within a ten-billionth of the point's size of zero (a tie), or the stretch is too short
    /// to split: far from the boundary one bound decides, and near it the work grows as the
    /// logarithm of the distance.
    pub fn side(&self,p: [f64;3]) -> f64 {
        if let Some(d) = self.clear_of(p) { return d; }
        let seed = self.cube(p);
        if let Some((d,c)) = &seed { if c.low-d > 0. { return c.low-d; } }
        let roll = self.roll(p);
        let read = Reads::new(&roll);
        let last = Roll::LAST;
        let speed = std::cell::OnceCell::new();
        let speed = || *speed.get_or_init(|| self.motion.inverse_point_speed_bound(p,self.domain).unwrap_or(f64::INFINITY));
        // A stretch's bound from its readings here, or the bound it came with where that is higher
        // (a cube's, carried to this point, or the stretch it was split from).
        let bound = |i0: u64,f0: f64,i1: u64,f1: f64,given: f64| roll.bound(speed(),i0,f0,i1,f1).max(given);
        let mut stretches = std::collections::BinaryHeap::new();
        let mut best = f64::INFINITY;
        match &seed {
            Some((d,c)) => for &(bl,i0,i1) in c.stretches.iter() { stretches.push(Stretch(bl-d,i0,f64::NAN,i1,f64::NAN)); },
            None => {
                let (fa,fb) = (read.at(0),read.at(last));
                best = fa.min(fb);
                if !(roll.b > roll.a) || best < 0. { return best; }
                stretches.push(Stretch(bound(0,fa,last,fb,f64::NEG_INFINITY),0,fa,last,fb));
            }
        }
        // A bound this close to zero leaves the side a tie a mesher's bisection resolves by
        // position; refining it further buys a sign nothing downstream can use.
        let tolerance = 1e-10*(1.+(p[0]*p[0]+p[1]*p[1]+p[2]*p[2]).sqrt());
        // Near a rolling contact the path runs along the tool, so the source rises slowly away
        // from its minimum: a long shallow basin that a first-order bound splits stretch by
        // stretch at every level. So splitting stops at `ROLL_BASIN`: the stretches that may
        // still hold a negative value are grouped into contiguous runs, each a basin, and each
        // basin's minimum is found by Brent's search around its lowest reading. That is a
        // reading, not a bound: a second dip inside one run, between readings, can be missed.
        let basin = last >> ROLL_BASIN;
        loop {
            let Some(Stretch(low,i0,f0,i1,f1)) = stretches.pop() else { return best };
            if low > -tolerance {
                // every stretch left is above zero: a reading says how far, if none was made
                if !best.is_finite() { best = read.at(i0).min(read.at(i1)); }
                return best;
            }
            // A stretch carried from a cube is read here before it is split.
            if f0.is_nan() || f1.is_nan() {
                let (f0,f1) = (read.at(i0),read.at(i1));
                best = best.min(f0).min(f1);
                if best < 0. { return best; }
                stretches.push(Stretch(bound(i0,f0,i1,f1,low),i0,f0,i1,f1));
                continue;
            }
            if i1-i0 <= basin {
                // A basin is searched only once every stretch still open is a basin wide: one
                // carried from a cube may be wider, and its interior is no reading's neighbour.
                if let Some(wide) = widest(&mut stretches,basin,-tolerance) {
                    stretches.push(Stretch(low,i0,f0,i1,f1));
                    let (readings,halves) = split(&read,wide,&bound);
                    best = readings.iter().fold(best,|b,r| b.min(r.1));
                    if best < 0. { return best; }
                    stretches.extend(halves);
                    continue;
                }
                let mut open: Vec<(u64,f64,u64,f64)> = vec![(i0,f0,i1,f1)];
                open.extend(stretches.drain().filter(|s| s.0 <= -tolerance).map(|s| (s.1,read.or(s.1,s.2),s.3,read.or(s.3,s.4))));
                for (readings,w0,w1) in basins(open) {
                    if let Some(&(_,v)) = readings.iter().find(|r| r.1 < 0.) { return v; }
                    let (lo,hi) = (roll.time(w0),roll.time(w1));
                    // settled once a reading is inside, or the least the minimum can be is outside
                    let (found,_) = brent(&|t| roll.at_time(t),lo,hi,1e-6*(hi-lo),60,|v,slack| v < 0. || v-slack > tolerance);
                    best = best.min(found);
                    if found < 0. { return found; }
                }
                return best;
            }
            let im = i0+(i1-i0)/2;
            let fm = read.at(im);
            best = best.min(fm);
            if fm < 0. { return fm; }
            stretches.push(Stretch(bound(i0,f0,im,fm,low),i0,f0,im,fm));
            stretches.push(Stretch(bound(im,fm,i1,f1,low),im,fm,i1,f1));
        }
    }

    /// The field's value at a point and the roll time it is least at, in plain floating point —
    /// `side`'s search carried on until no stretch of roll can read more than `accuracy` below the
    /// best reading (or `relative` of its own size, where that is coarser), where `side` stops at
    /// the first negative one. A Newton step from a point far from the boundary needs its value to
    /// a few digits, and the search stops as soon as no stretch can read that much lower. The
    /// value is a reading, not an interval claim: a stretch is bounded by the motion's
    /// inverse-point speed, and a basin is searched by Brent's method around its lowest reading, so
    /// a second dip inside one basin can be missed. `tied` is set when the minima of two separate
    /// basins are within `tie` of each other: two contact times, which is a crease of the swept
    /// surface.
    pub(crate) fn minimum_relative(&self,p: [f64;3],accuracy: f64,relative: f64,tie: f64) -> SweptMinimum {
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
    pub(crate) fn minimum_hinted(&self,p: [f64;3],accuracy: f64,relative: f64,tie: f64,hint: Option<f64>,local: bool) -> SweptMinimum {
        let roll = self.roll(p);
        let (a,b) = (roll.a,roll.b);
        let last = Roll::LAST;
        let seed = if b > a { self.cube(p) } else { None };
        let read = Reads::new(&roll);
        let (fa,fb) = if seed.is_some() { (f64::INFINITY,f64::INFINITY) } else { (read.at(0),read.at(last)) };
        let (mut best,mut best_t) = if fa <= fb { (fa,a) } else { (fb,b) };
        let done = |value: f64,time: f64,tied: bool| SweptMinimum {value,time,tied};
        if !(b > a) { return done(best,best_t,false); }
        let basin = last >> ROLL_BASIN;
        // The minimum in a bracket, by Brent's search, to `stop` of its value — the accuracy asked,
        // a thousandth of the value itself for a reading far from the boundary — or to 10⁻⁸ of the
        // bracket, where the value is within 10⁻¹⁶ of the bracket's rise, whichever comes first.
        let search = |lo: f64,hi: f64,stop: f64| -> (f64,f64) {
            brent(&|t| roll.at_time(t),lo,hi,(1e-8*(hi-lo)).max(1e-15*(1.+hi.abs())),80,|_,slack| slack <= stop)
        };
        // The hint's window, in grid indices, searched first; `None` when cold.
        let window = hint.filter(|t| t.is_finite() && *t >= a && *t <= b).map(|t| {
            let centre = ((t-a)/(b-a)*last as f64).round().clamp(0.,last as f64) as u64;
            let (w0,w1) = (centre.saturating_sub(basin/2),(centre+basin/2).min(last));
            let found = search(roll.time(w0),roll.time(w1),accuracy);
            if found.0 < best { best = found.0; best_t = found.1; }
            (w0,w1,found)
        });
        if local {
            if let Some((w0,w1,found)) = window {
                let margin = 0.02*(roll.time(w1)-roll.time(w0));
                let at_edge = (found.1-roll.time(w0) < margin && w0 > 0) || (roll.time(w1)-found.1 < margin && w1 < last);
                if !at_edge { return done(found.0,found.1,false); }
            }
        }
        let Ok(speed) = self.motion.inverse_point_speed_bound(p,self.domain) else { return done(best,best_t,false) };
        let bound = |i0: u64,f0: f64,i1: u64,f1: f64,given: f64| roll.bound(speed,i0,f0,i1,f1).max(given);
        let mut stretches = std::collections::BinaryHeap::new();
        match &seed {
            Some((d,c)) => for &(bl,i0,i1) in c.stretches.iter() { stretches.push(Stretch(bl-d,i0,f64::NAN,i1,f64::NAN)); },
            None => stretches.push(Stretch(bound(0,fa,last,fb,f64::NEG_INFINITY),0,fa,last,fb)),
        }
        let mut tied = false;
        let searched = |i0: u64,i1: u64| window.is_some_and(|(w0,w1,_)| i0 >= w0 && i1 <= w1);
        loop {
            let Some(Stretch(low,i0,f0,i1,f1)) = stretches.pop() else { break };
            let accuracy = accuracy.max(if best.is_finite() { relative*best.abs() } else { 0. });
            if low > best-accuracy { break; }
            if searched(i0,i1) { continue; }
            // A stretch carried from a cube is read here before it is split.
            if f0.is_nan() || f1.is_nan() {
                let (f0,f1) = (read.at(i0),read.at(i1));
                if f0 < best { best = f0; best_t = roll.time(i0); }
                if f1 < best { best = f1; best_t = roll.time(i1); }
                stretches.push(Stretch(bound(i0,f0,i1,f1,low),i0,f0,i1,f1));
                continue;
            }
            if i1-i0 <= basin {
                if let Some(wide) = widest(&mut stretches,basin,best-accuracy) {
                    stretches.push(Stretch(low,i0,f0,i1,f1));
                    let (readings,halves) = split(&read,wide,&bound);
                    for (i,g) in readings { if g < best { best = g; best_t = roll.time(i); } }
                    stretches.extend(halves);
                    continue;
                }
                // As in `side`: the stretches still able to beat the best reading, grouped into
                // contiguous runs, each a basin whose minimum Brent's search finds.
                let mut open: Vec<(u64,f64,u64,f64)> = vec![(i0,f0,i1,f1)];
                open.extend(stretches.drain().filter(|s| s.0 <= best-accuracy && !searched(s.1,s.3))
                    .map(|s| (s.1,read.or(s.1,s.2),s.3,read.or(s.3,s.4))));
                let mut minima: Vec<(f64,f64)> = basins(open).into_iter()
                    .map(|(_,w0,w1)| search(roll.time(w0),roll.time(w1),accuracy)).collect();
                // The window's minimum is a basin too. A run's beside it is the same basin carried
                // on past the window's edge (the contact moved further than the window reaches),
                // and the lower of the two is that basin's minimum.
                if let Some((w0,w1,found)) = window {
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
                if let Some(&(v,t)) = minima.first() { if v < best { best = v; best_t = t; } }
                tied = minima.len() > 1 && minima[1].0-minima[0].0 <= tie;
                break;
            }
            let im = i0+(i1-i0)/2;
            let fm = read.at(im);
            if fm < best { best = fm; best_t = roll.time(im); }
            stretches.push(Stretch(bound(i0,f0,im,fm,low),i0,f0,im,fm));
            stretches.push(Stretch(bound(im,fm,i1,f1,low),im,fm,i1,f1));
        }
        done(best,best_t,tied)
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

    /// Enclose the swept field for every point in the input box. Options bound
    /// the complete roll search; termination status and uncertainty are retained.
    /// The value tolerance is a field width, not a geometric export tolerance.
    pub fn bounds(&mut self,p: V,options: Options) -> Result<Minimum,SweepError> {
        self.bounds_with_observer(p,options,|_,_| {})
    }

    /// Stop when the full sweep enclosure is strictly outside the requested
    /// field-value band. `Separated` retains bounds and an attained witness;
    /// it does not claim convergence to the value-width tolerance.
    pub fn bounds_outside(&mut self,p: V,band: I,options: Options) -> Result<Minimum,SweepError> {
        self.evaluate(p,options,Stop::Outside(band),|_,_| {})
    }

    /// Observe raw interval-oracle enclosures, e.g. to extract independently
    /// checkable coverage evidence. The observer supplies no geometry, bounds
    /// or pruning decisions; it cannot change the oracle's mathematical result.
    pub fn bounds_with_observer(&mut self,p: V,options: Options,observe: impl FnMut(I,I))
        -> Result<Minimum,SweepError> {
        self.evaluate(p,options,Stop::Converged,observe)
    }

    /// The search stops as `stop` allows (see `minimum::Stop`).
    pub(super) fn evaluate(&mut self,p: V,options: Options,stop: Stop,mut observe: impl FnMut(I,I))
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
            observe(t,bound);
            Ok::<_,Error>(bound)
        },options,stop)
    }
}
