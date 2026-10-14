//! What a long run is doing, for a front end that wants to say so (#320).
//!
//! A drawing's solids are built, and its claims about them judged, lazily and deep inside a
//! diagnosis, so a front end cannot time them from outside.  The core says when such work begins
//! and ends to whoever listens on this thread (`listen`); with nobody listening, `during` runs the
//! work and names nothing.  The words are the core's (`Work::done`, `Work::running`,
//! `Timings::lines`); the clock is `clock::Instant`, which reads zero where there is none; what to
//! print, and when, is the listener's.

use std::cell::RefCell;
use std::rc::Rc;

use crate::clock::Instant;
use crate::model::Sketch;

/// What kind of work.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Kind {
    /// The document parsed, linked and elaborated into a sketch.
    Elaborate,
    /// The drawing's unknowns solved for.
    Solve,
    /// The diagnosis, everything below it included.
    Diagnose,
    /// A solid's exact B-rep built (`Sketch::exact_solid`).
    Build,
    /// A solid evaluated at an approximation: its boundary, from the B-rep or otherwise
    /// (`Sketch::evaluated_solid`).
    Evaluate,
    /// A claim about solids judged, at every pose it is asked at.
    Claim,
}

impl Kind {
    /// Whether it is work on the drawing's objects — a solid or a claim about solids — rather than
    /// a stage of the run.
    pub fn is_object(self) -> bool {
        matches!(self, Kind::Build | Kind::Evaluate | Kind::Claim)
    }
}

/// The statement a piece of work answers to, for a front end to point at.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum At {
    /// The statement that declared solid `i` (`SourceMap::site_of`).
    Solid(usize),
    /// Statement `StmtId(n)`.
    Stmt(u32),
}

/// One piece of work: its kind, what it is about as the source names it, and where.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct Work {
    pub kind: Kind,
    pub subject: String,
    pub at: Option<At>,
}

impl Work {
    /// Document `name` elaborated.
    pub fn elaborate(name: &str) -> Work {
        Work { kind: Kind::Elaborate, subject: name.to_string(), at: None }
    }

    /// The solve of `sk`, counted in the unknowns it moves (the diagnosis's params).
    pub fn solve(sk: &Sketch) -> Work {
        let n = sk.free_indices().len();
        Work { kind: Kind::Solve, subject: format!("{n} params"), at: None }
    }

    /// Document `name` diagnosed.
    pub fn diagnose(name: &str) -> Work {
        Work { kind: Kind::Diagnose, subject: name.to_string(), at: None }
    }

    /// Solid `i`'s exact B-rep built.
    pub fn build(sk: &Sketch, i: usize) -> Work {
        Work { kind: Kind::Build, subject: sk.solid_name(i), at: Some(At::Solid(i)) }
    }

    /// Solid `i` evaluated.
    pub fn evaluate(sk: &Sketch, i: usize) -> Work {
        Work { kind: Kind::Evaluate, subject: sk.solid_name(i), at: Some(At::Solid(i)) }
    }

    /// A claim judged: `text` is the claim as the report words it (`claim a clear(1mm) b`).
    pub fn claim(text: String, stmt: u32) -> Work {
        Work { kind: Kind::Claim, subject: text, at: Some(At::Stmt(stmt)) }
    }

    fn verb(&self) -> (&'static str, &'static str) {
        match self.kind {
            Kind::Elaborate => ("elaborated", "elaborating"),
            Kind::Solve => ("solved for", "solving for"),
            Kind::Diagnose => ("diagnosed", "diagnosing"),
            Kind::Build => ("built", "building"),
            Kind::Evaluate => ("evaluated", "evaluating"),
            Kind::Claim => ("judged", "judging"),
        }
    }

    /// The work as done: `built stator.lamination`.
    pub fn text(&self) -> String {
        format!("{} {}", self.verb().0, self.subject)
    }

    /// The line said when it is done, `total` seconds in all and `own` of them not in work it
    /// started: `built stator.lamination in 2.1 s`, and what it waited on where that was most.
    pub fn done(&self, total: f64, own: f64) -> String {
        if total - own > 0.5 * total && total >= 0.1 {
            format!("{} in {} ({} its own)", self.text(), seconds(total), seconds(own))
        } else {
            format!("{} in {}", self.text(), seconds(total))
        }
    }

    /// The line said while it is still going, `elapsed` seconds in: `still building
    /// stator.copper, 10 s so far`.
    pub fn running(&self, elapsed: f64) -> String {
        format!("still {} {}, {} so far", self.verb().1, self.subject, seconds(elapsed))
    }
}

/// A duration as a person reads it: milliseconds below a tenth of a second.
pub fn seconds(s: f64) -> String {
    if s < 0.1 { format!("{:.0} ms", s * 1e3) } else { format!("{s:.1} s") }
}

/// Whoever is told about work on this thread.  `begin` and `end` nest: every `end` closes the
/// latest `begin` not yet closed.
pub trait Listener {
    fn begin(&self, work: &Work);
    /// `total` seconds from its begin, `own` of them not inside work it started.
    fn end(&self, work: &Work, total: f64, own: f64);
}

thread_local! {
    static LISTENER: RefCell<Option<Rc<dyn Listener>>> = const { RefCell::new(None) };
    /// For each piece of work open on this thread, the seconds its own nested work has taken.
    static OPEN: RefCell<Vec<f64>> = const { RefCell::new(Vec::new()) };
}

/// Puts back what was listening before, however the work it was around ended.
struct Restore(Option<Rc<dyn Listener>>, usize);

impl Drop for Restore {
    fn drop(&mut self) {
        LISTENER.with(|l| *l.borrow_mut() = self.0.take());
        OPEN.with(|o| o.borrow_mut().truncate(self.1));
    }
}

/// Run `work` with `listener` told about what is done on this thread meanwhile.
pub fn listen<T>(listener: Rc<dyn Listener>, work: impl FnOnce() -> T) -> T {
    let before = LISTENER.with(|l| l.borrow_mut().replace(listener));
    let _restore = Restore(before, OPEN.with(|o| o.borrow().len()));
    work()
}

/// Whether anything is listening on this thread.
pub fn listening() -> bool {
    LISTENER.with(|l| l.borrow().is_some())
}

/// Run `work`, said to be `what` to whoever listens.  `what` is only made when somebody does.
pub fn during<T>(what: impl FnOnce() -> Work, work: impl FnOnce() -> T) -> T {
    let Some(listener) = LISTENER.with(|l| l.borrow().clone()) else { return work() };
    let what = what();
    listener.begin(&what);
    let depth = OPEN.with(|o| {
        let mut o = o.borrow_mut();
        o.push(0.);
        o.len()
    });
    let clock = Instant::now();
    let result = work();
    let total = clock.elapsed().as_secs_f64();
    let nested = OPEN.with(|o| {
        let mut o = o.borrow_mut();
        // a listener swapped in below truncates to its own depth when it goes, never past this
        let nested = if o.len() == depth { o.pop().unwrap_or(0.) } else { 0. };
        if let Some(parent) = o.last_mut() { *parent += total; }
        nested
    });
    listener.end(&what, total, (total - nested).max(0.));
    result
}

/// Time spent, by piece of work: what a summary at the end of a run reads.  The same work done
/// again (a solid evaluated at each pose of a claim's sweep) is one row, counted.
#[derive(Clone, Debug, Default)]
pub struct Timings {
    rows: std::collections::BTreeMap<Work, Row>,
}

#[derive(Clone, Copy, Debug, Default)]
struct Row {
    count: usize,
    total: f64,
    own: f64,
}

impl Timings {
    /// Record one piece of work done: how many times it has been, this one included.
    pub fn add(&mut self, work: &Work, total: f64, own: f64) -> usize {
        let row = self.rows.entry(work.clone()).or_default();
        row.count += 1;
        row.total += total;
        row.own += own;
        row.count
    }

    /// The solids and claims, the most time of its own first, at most `most` of them: each line
    /// with the work it is about, for a front end to say where.  `own  total  what (×n)`.
    pub fn lines(&self, most: usize) -> Vec<(String, &Work)> {
        let mut rows: Vec<_> = self.rows.iter()
            .filter(|(w, _)| w.kind.is_object())
            .collect();
        rows.sort_by(|a, b| b.1.own.total_cmp(&a.1.own).then_with(|| a.0.cmp(b.0)));
        rows.into_iter().take(most).map(|(w, r)| {
            let times = if r.count > 1 { format!(" (×{})", r.count) } else { String::new() };
            (format!("{:>8} {:>8}  {}{times}", seconds(r.own), seconds(r.total), w.text()), w)
        }).collect()
    }

    /// What `lines` are, said above them.
    pub fn heading(&self) -> String {
        let own: f64 = self.rows.iter().filter(|(w, _)| w.kind.is_object()).map(|(_, r)| r.own).sum();
        format!("solids and claims, slowest first ({} in all): own time, then with the work it \
            waited on", seconds(own))
    }
}
