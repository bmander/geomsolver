//! A long run, said as it goes (#320): the core's `progress` heard on stderr, and timed.
//!
//! Each piece of work the core reports — the elaboration, the solve, the diagnosis, a solid built
//! or evaluated, a claim judged — is a line on stderr when it is done, worded by the core
//! (`Work::done`) on the export's timeline (`cad::progress::stage`).  The same work done again (a
//! solid evaluated at each pose of a claim's sweep) is said the first time only.  Work still going
//! is said again past `FIRST` seconds, then each time it doubles (`Work::running`), so a slow
//! boolean is told from a hang, and which statement it is.  Every piece is kept for `--timings`.
use std::rc::Rc;
use std::sync::atomic::{AtomicBool,Ordering};
use std::sync::{Arc,Mutex};
use std::time::Instant;

use gcs_core::progress::{Listener,Timings,Work};

use crate::cad::progress::stage;

/// How long work runs before it is said to be still going (seconds).
const FIRST: f64 = 5.;

/// How much is said.
#[derive(Clone,Copy,Debug,PartialEq,Eq)]
pub enum Say {
    /// Nothing; work is only timed.
    Nothing,
    /// Work that took a second or more, and work still going (a terminal, unasked).
    Slow,
    /// Every piece of work (`--progress`).
    Everything,
}

/// A piece of work open: since when, when next to say it is still going, whether that was said.
struct Open { work: Work,since: Instant,next: f64,said: bool }

/// The listener: what is open, shared with the thread that says what is still going, and what
/// was done, for `--timings`.
pub struct Watch {
    say: Say,
    open: Arc<Mutex<Vec<Open>>>,
    stop: Arc<AtomicBool>,
    pub timings: std::cell::RefCell<Timings>,
}

impl Watch {
    pub fn new(say: Say) -> Rc<Watch> {
        let watch = Rc::new(Watch {say,open: Default::default(),stop: Default::default(),timings: Default::default()});
        if say != Say::Nothing {
            crate::cad::progress::start();
            let (open,stop) = (watch.open.clone(),watch.stop.clone());
            std::thread::spawn(move || while !stop.load(Ordering::Relaxed) {
                std::thread::sleep(std::time::Duration::from_millis(100));
                still(&open);
            });
        }
        watch
    }

    /// Whether a line is said for work done, `total` seconds in all: `first` time it is done, or
    /// `said` to be still going.
    fn spoken(&self,total: f64,first: bool,said: bool) -> bool {
        match self.say {
            Say::Nothing => false,
            Say::Slow => said || (first && total >= 1.),
            Say::Everything => said || first,
        }
    }
}

impl Drop for Watch {
    fn drop(&mut self) { self.stop.store(true,Ordering::Relaxed); }
}

/// Say the innermost open work that has run past its time, and put it and what holds it off
/// until twice as long.
fn still(open: &Mutex<Vec<Open>>) {
    let Ok(mut open) = open.lock() else { return };
    let Some(k) = open.iter().rposition(|o| o.since.elapsed().as_secs_f64() >= o.next) else { return };
    let elapsed = open[k].since.elapsed().as_secs_f64();
    stage(&open[k].work.running(elapsed));
    for o in &mut open[..=k] {
        o.next = o.next.max(2. * o.since.elapsed().as_secs_f64());
    }
    open[k].said = true;
}

impl Listener for Watch {
    fn begin(&self,work: &Work) {
        if let Ok(mut open) = self.open.lock() {
            open.push(Open {work: work.clone(),since: Instant::now(),next: FIRST,said: false});
        }
    }

    fn end(&self,work: &Work,total: f64,own: f64) {
        let first = self.timings.borrow_mut().add(work,total,own) == 1;
        let said = self.open.lock().ok().and_then(|mut open| open.pop()).is_some_and(|o| o.said);
        if self.spoken(total,first,said) {
            stage(&work.done(total,own));
        }
    }
}
