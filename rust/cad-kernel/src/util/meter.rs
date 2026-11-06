//! **The work a build may do, counted** (#777 V3 and V4): a closed sweep's arrangement, its traces,
//! its ray tests, its outlines, its inverse maps and its membership reads each charge the meter
//! installed for the build (`within`) by the steps they take, and a stage past its limit stops the
//! build: every metered loop reads `stopped` at its next step and winds down, and the build that
//! installed the meter refuses with the stage that ran away and where (`Meter::over`). The budget is
//! counted in steps, never seconds, so whether a build is refused depends on its inputs alone.
//!
//! The same counts, with what only a person reads beside them (the meetings found in closed form
//! against those traced and those given by construction (#810), per pair of surface kinds, and the
//! slowest; the curve–surface searches
//! that sampled, per pair; the deepest outline), are the build's **profile** (`Meter::report`),
//! printed where `SOLVENT_SWEEP_PROFILE` is set. A performance change cites it before and after.
//!
//! The meter is the thread's: `util::par` hands it to every worker a parallel map starts, so the
//! work of one build is counted wherever it runs. With no meter installed every charge is free and
//! nothing stops.
use std::collections::BTreeMap;
use std::sync::{Arc,Mutex};
use std::sync::atomic::{AtomicBool,AtomicU64,Ordering};

type V = [f64;3];

/// What a build spends its work on, each counted and limited on its own.
#[derive(Clone,Copy,Debug,PartialEq,Eq,PartialOrd,Ord,Hash)]
pub enum Stage {
    /// The arrangement's pairs of faces met and pieces read (`boolean`).
    Arrangement = 0,
    /// Steps of the traces of meetings with no closed form (`ssi::trace`).
    Trace = 1,
    /// A curve's crossings with a surface sought — a ray's, an edge's — and the samples a sampled
    /// search takes (`query::curve_surface`).
    Ray = 2,
    /// Chords an outline is cut in (`query::Outline`).
    Outline = 3,
    /// A swept sheet's inverse map (`geom::Blend::inverse`).
    Inverse = 4,
    /// Stretches of the roll a membership test reads (`swept::member`).
    Membership = 5,
    /// Points the built solid is judged at against its definition (`swept::closed`).
    Judge = 6,
}

/// How many stages there are.
pub const STAGES: usize = 7;

impl Stage {
    pub const ALL: [Stage;STAGES] = [Stage::Arrangement,Stage::Trace,Stage::Ray,Stage::Outline,Stage::Inverse,Stage::Membership,Stage::Judge];
    /// The stage's name, as a refusal and the profile say it.
    pub fn name(self) -> &'static str {
        match self {
            Stage::Arrangement => "arrangement",
            Stage::Trace => "trace",
            Stage::Ray => "ray",
            Stage::Outline => "outline",
            Stage::Inverse => "inverse",
            Stage::Membership => "membership",
            Stage::Judge => "judge",
        }
    }
}

impl std::fmt::Display for Stage {
    fn fmt(&self,f: &mut std::fmt::Formatter) -> std::fmt::Result { f.write_str(self.name()) }
}

/// The most steps each stage may take, by `Stage` (`u64::MAX` for no limit).
#[derive(Clone,Copy,Debug,PartialEq)]
pub struct Budget(pub [u64;STAGES]);

impl Budget {
    /// No limit on any stage: the profile alone.
    pub const UNLIMITED: Budget = Budget([u64::MAX;STAGES]);
    /// `self` with `stage` limited to `steps`.
    pub fn with(mut self,stage: Stage,steps: u64) -> Budget { self.0[stage as usize] = steps; self }
}

/// What ran away: the stage, a point where it was working when it went past its limit, and what it
/// was working on there (the surfaces, by kind).
#[derive(Clone,Debug,PartialEq)]
pub struct Over { pub stage: Stage,pub at: V,pub what: String }

/// One pair of surface kinds' meetings: found in closed form, traced, given before any was sought
/// (#810), and the time they took.
#[derive(Clone,Copy,Debug,Default)]
struct Meetings { exact: u64,traced: u64,given: u64,nanos: u128,slowest: u128 }

/// What only the profile reads.
#[derive(Clone,Debug,Default)]
struct Profile {
    /// Meetings by the two surfaces' kinds, the pair sorted.
    meetings: BTreeMap<(&'static str,&'static str),Meetings>,
    /// Curve–surface searches by the curve's and the surface's kinds: certified (closed form or a
    /// bounded isolation) and sampled.
    crossings: BTreeMap<(&'static str,&'static str),[u64;2]>,
    /// The deepest an outline's chords were halved.
    outline_depth: usize,
    /// A meeting's analysis, by what was done (`analysis`, #780 R4): critical sets found, the jets
    /// their searches read, branch tests asked and the curve points and feet they read, readings
    /// of `f` a level curve's march took.
    analysis: BTreeMap<&'static str,u64>,
}

/// A build's counts against its budget.
#[derive(Debug)]
pub struct Meter {
    limits: Budget,
    counts: [AtomicU64;STAGES],
    stopped: AtomicBool,
    over: Mutex<Option<Over>>,
    profile: Mutex<Profile>,
}

thread_local! {
    static CURRENT: std::cell::RefCell<Option<Arc<Meter>>> = const { std::cell::RefCell::new(None) };
}

impl Meter {
    /// A meter counting against `limits`.
    pub fn new(limits: Budget) -> Arc<Meter> {
        Arc::new(Meter {limits,counts:std::array::from_fn(|_| AtomicU64::new(0)),stopped:AtomicBool::new(false),
            over:Mutex::new(None),profile:Mutex::new(Profile::default())})
    }
    /// The steps `stage` has taken.
    pub fn count(&self,stage: Stage) -> u64 { self.counts[stage as usize].load(Ordering::Relaxed) }
    /// The first stage that went past its limit, and where; none while every stage is within.
    pub fn over(&self) -> Option<Over> { self.over.lock().unwrap_or_else(|e| e.into_inner()).clone() }
    /// Whether a stage went past its limit.
    pub fn stopped(&self) -> bool { self.stopped.load(Ordering::Relaxed) }
    /// `n` steps of `stage` taken; false once any stage is past its limit. The first to pass it
    /// names itself, and where (`at`, read only then).
    pub fn charge(&self,stage: Stage,n: u64,at: impl FnOnce() -> (V,String)) -> bool {
        let k = stage as usize;
        let before = self.counts[k].fetch_add(n,Ordering::Relaxed);
        if before.saturating_add(n) > self.limits.0[k] && before <= self.limits.0[k] {
            let mut over = self.over.lock().unwrap_or_else(|e| e.into_inner());
            if over.is_none() { let (at,what) = at(); *over = Some(Over {stage,at,what}); }
            self.stopped.store(true,Ordering::Relaxed);
        }
        !self.stopped()
    }
    /// The counts and what the profile keeps, for a person: each stage's steps against its
    /// limit, the meetings exact and traced per pair of surface kinds, the slowest pairs, the
    /// curve–surface searches certified and sampled per pair, and the deepest outline.
    pub fn report(&self) -> String {
        use std::fmt::Write;
        let mut s = String::new();
        let _ = writeln!(s,"sweep profile:");
        for stage in Stage::ALL {
            let limit = self.limits.0[stage as usize];
            let _ = writeln!(s,"  {:<12} {:>12} steps{}",stage.name(),self.count(stage),
                if limit == u64::MAX { String::new() } else { format!(" of {limit}") });
        }
        if let Some(o) = self.over() { let _ = writeln!(s,"  over budget: {} at {:?} ({})",o.stage,o.at,o.what); }
        let p = self.profile.lock().unwrap_or_else(|e| e.into_inner()).clone();
        let _ = writeln!(s,"  meetings (exact / traced / given, ms):");
        for ((a,b),m) in &p.meetings {
            let _ = writeln!(s,"    {a:>10} × {b:<10} {:>6} / {:<6} / {:<6} {:>10.1}",m.exact,m.traced,m.given,m.nanos as f64*1e-6);
        }
        let mut slow: Vec<_> = p.meetings.iter().collect();
        slow.sort_by(|x,y| y.1.slowest.cmp(&x.1.slowest));
        let _ = writeln!(s,"  slowest pairs (one meeting, ms): {}",slow.iter().take(5)
            .map(|((a,b),m)| format!("{a} × {b} {:.1}",m.slowest as f64*1e-6)).collect::<Vec<_>>().join(", "));
        let _ = writeln!(s,"  curve–surface searches (certified / sampled):");
        for ((c,f),[x,y]) in &p.crossings { let _ = writeln!(s,"    {c:>10} × {f:<10} {x:>6} / {y:<6}"); }
        let _ = writeln!(s,"  deepest outline: {} halvings",p.outline_depth);
        let _ = writeln!(s,"  meeting analysis:");
        for (what,n) in &p.analysis { let _ = writeln!(s,"    {what:<28} {n:>12}"); }
        s
    }
    /// The curve–surface searches that sampled, all pairs together (what V1's gate asks to be none
    /// on the surfaces it reads).
    pub fn sampled(&self) -> u64 {
        self.profile.lock().unwrap_or_else(|e| e.into_inner()).crossings.values().map(|c| c[1]).sum()
    }
}

/// `f` run with `meter` counting this thread's work (and every worker's a parallel map starts
/// inside it), the meter before it put back after.
pub fn within<R>(meter: &Arc<Meter>,f: impl FnOnce() -> R) -> R {
    let _restore = installed(meter);
    f()
}

/// `meter` counting this thread's work until what is returned is dropped, the meter before it put
/// back then: `within` for a scope, not a closure (a test profiled whole).
pub fn installed(meter: &Arc<Meter>) -> Installed {
    Installed(CURRENT.with(|c| c.replace(Some(meter.clone()))))
}

/// A meter installed for a scope (`installed`): dropped, the one before it is put back.
#[must_use]
pub struct Installed(Option<Arc<Meter>>);

impl Drop for Installed {
    fn drop(&mut self) { let m = self.0.take(); CURRENT.with(|c| *c.borrow_mut() = m); }
}

/// The meter this thread counts against, if any.
pub fn current() -> Option<Arc<Meter>> { CURRENT.with(|c| c.borrow().clone()) }

/// `f` run with `meter` installed, or as it is where there is none: what a worker thread runs
/// its items under.
pub fn under<R>(meter: Option<&Arc<Meter>>,f: impl FnOnce() -> R) -> R {
    match meter { Some(m) => within(m,f),None => f() }
}

/// `n` steps of `stage` charged to the thread's meter: false once the build is to stop (no meter,
/// never). `at` says where, read only when this charge passes the limit.
#[inline]
pub fn charge(stage: Stage,n: u64,at: impl FnOnce() -> (V,String)) -> bool {
    CURRENT.with(|c| match &*c.borrow() { Some(m) => m.charge(stage,n,at),None => true })
}

/// `charge` with nothing to say of where.
#[inline]
pub fn tick(stage: Stage) -> bool { charge(stage,1,|| ([f64::NAN;3],String::new())) }

/// Whether the thread's build is to stop: a stage went past its limit.
#[inline]
pub fn stopped() -> bool { CURRENT.with(|c| c.borrow().as_ref().is_some_and(|m| m.stopped())) }

/// **A build stopped past its budget** (#780 R3), as a typed answer: the stage that ran away, and
/// where (`Over`). What `go_on` returns once the build is to stop, carried out by `?` — as a
/// refusal by budget where a build answers in `swept::closed::Unbuilt`, and as its message where it
/// answers in text (a Boolean's phases) — never matched by its words.
#[derive(Clone,Debug,PartialEq)]
pub struct Stopped(pub Over);

impl std::fmt::Display for Stopped {
    fn fmt(&self,f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f,"the build went past its budget in its {} stage at {:?} ({})",self.0.stage,self.0.at,self.0.what)
    }
}

impl From<Stopped> for String { fn from(s: Stopped) -> String { s.to_string() } }

/// **Whether the thread's build goes on** (#780 R3): `Err` once a stage went past its limit, the one
/// check a metered pipeline makes between its phases (`go_on()?`). With no meter, always on.
#[inline]
pub fn go_on() -> Result<(),Stopped> {
    // (the first stage over names itself before the meter stops: a stopped meter has its `over`)
    CURRENT.with(|c| match &*c.borrow() {
        Some(m) if m.stopped() => m.over().map_or(Ok(()),|o| Err(Stopped(o))),
        _ => Ok(()),
    })
}

/// A meeting of surfaces of kinds `a` and `b` found, in closed form or traced, in `took`.
pub fn meeting(a: &'static str,b: &'static str,exact: bool,took: std::time::Duration) {
    meetings_of(a,b,|e| {
        if exact { e.exact += 1 } else { e.traced += 1 }
        e.nanos += took.as_nanos();
        e.slowest = e.slowest.max(took.as_nanos());
    })
}

/// A meeting of surfaces of kinds `a` and `b` given before any was sought (#810, `meeting::Shared::give`).
pub fn given(a: &'static str,b: &'static str) { meetings_of(a,b,|e| e.given += 1) }

/// `count` of the profile's meetings of surfaces of kinds `a` and `b` (the pair sorted), where a
/// meter is installed.
fn meetings_of(a: &'static str,b: &'static str,count: impl FnOnce(&mut Meetings)) {
    CURRENT.with(|c| if let Some(m) = &*c.borrow() {
        let key = if a <= b { (a,b) } else { (b,a) };
        count(m.profile.lock().unwrap_or_else(|e| e.into_inner()).meetings.entry(key).or_default())
    })
}

/// A curve of kind `curve` sought against a surface of kind `surface`: certified (a closed form or
/// a bounded isolation, `util::isolate`) or sampled.
pub fn crossing(curve: &'static str,surface: &'static str,certified: bool) {
    CURRENT.with(|c| if let Some(m) = &*c.borrow() {
        let mut p = m.profile.lock().unwrap_or_else(|e| e.into_inner());
        p.crossings.entry((curve,surface)).or_default()[if certified { 0 } else { 1 }] += 1;
    })
}

/// `n` more of `what` done by a meeting's analysis (`meeting`: a critical set found, a branch
/// test asked, a level curve's readings of `f`), for the profile alone: never charged or limited,
/// and counted only where the profile is printed (a branch test is asked per cell of a search).
pub fn analysis(what: &'static str,n: u64) {
    if n == 0 || !profiling() { return }
    CURRENT.with(|c| if let Some(m) = &*c.borrow() {
        *m.profile.lock().unwrap_or_else(|e| e.into_inner()).analysis.entry(what).or_default() += n;
    })
}

/// An outline's chords halved `depth` deep.
pub fn outline_depth(depth: usize) {
    CURRENT.with(|c| if let Some(m) = &*c.borrow() {
        let mut p = m.profile.lock().unwrap_or_else(|e| e.into_inner());
        p.outline_depth = p.outline_depth.max(depth);
    })
}

/// `SOLVENT_SWEEP_PROFILE` is set: a closed build prints its profile (read once).
pub fn profiling() -> bool {
    static ON: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    *ON.get_or_init(|| std::env::var_os("SOLVENT_SWEEP_PROFILE").is_some())
}
