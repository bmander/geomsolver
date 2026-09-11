//! Enclose a global minimum by refining a complete finite parameter interval.
//! The oracle must enclose its function over every supplied interval. Local root
//! solvers and sampled values alone do not satisfy this contract.
use super::Interval;
use std::{cmp::Ordering,collections::BinaryHeap};

#[derive(Clone,Copy,Debug)]
pub struct Options {
    /// Absolute width of the requested function-value enclosure, not spatial error.
    pub value_tolerance: f64,
    /// Includes interval and point queries. At least four queries are required.
    pub max_evaluations: usize,
}
impl Options {
    pub(crate) fn valid(self) -> bool {
        self.value_tolerance.is_finite() && self.value_tolerance > 0. && self.max_evaluations >= 4
    }
}

#[derive(Clone,Copy,Debug,PartialEq,Eq)]
pub enum Status {
    Converged,
    /// The complete minimum enclosure lies strictly outside the requested band.
    /// This does not assert the value-width tolerance was reached.
    Separated,
    /// The complete minimum enclosure lies strictly inside the requested band,
    /// for a caller that asked to stop there: which side of the band the value
    /// lies on is decided, and refining further could only narrow it inside.
    Contained,
    BudgetExhausted,
    ResolutionLimit,
}

#[derive(Clone,Copy,Debug)]
pub struct Minimum {
    pub value: Interval,
    /// A sampled parameter attaining the reported upper bound within oracle uncertainty.
    pub witness: f64,
    pub evaluations: usize,
    pub status: Status,
}

#[derive(Clone,Debug,PartialEq)]
pub enum Error<E> { InvalidOptions, Oracle(E), InconsistentBounds }

#[derive(Clone,Copy,Debug)]
struct Cell { domain: Interval,value: Interval }
impl PartialEq for Cell {
    fn eq(&self,b: &Self) -> bool { self.cmp(b) == Ordering::Equal }
}
impl Eq for Cell {}
impl Ord for Cell {
    fn cmp(&self,b: &Self) -> Ordering {
        // BinaryHeap's maximum is the cell with the smallest remaining lower bound.
        b.value.bounds()[0].total_cmp(&self.value.bounds()[0])
            .then_with(|| b.domain.bounds()[0].total_cmp(&self.domain.bounds()[0]))
            .then_with(|| b.domain.bounds()[1].total_cmp(&self.domain.bounds()[1]))
            .then_with(|| b.value.bounds()[1].total_cmp(&self.value.bounds()[1]))
    }
}
impl PartialOrd for Cell {
    fn partial_cmp(&self,b: &Self) -> Option<Ordering> { Some(self.cmp(b)) }
}

fn midpoint(domain: Interval) -> f64 {
    let [a,b] = domain.bounds();
    (a*0.5+b*0.5).clamp(a,b)
}
fn overlap<E>(a: Interval,b: Interval) -> Result<Interval,Error<E>> {
    let [al,ah] = a.bounds(); let [bl,bh] = b.bounds();
    Interval::new(al.max(bl),ah.min(bh)).map_err(|_| Error::InconsistentBounds)
}

/// Every live cell is a complete portion of the domain. A cell is discarded only
/// when its lower bound cannot improve the best sampled upper bound. Terminating
/// early retains the global enclosure and an explicit unresolved status.
/// Guarantees are conditional on the oracle's enclosure contract; overlapping
/// inconsistent or incorrect oracles cannot in general be detected by this routine.
pub fn enclose<E>(domain: Interval,bound: impl FnMut(Interval) -> Result<Interval,E>,
    options: Options) -> Result<Minimum,Error<E>> {
    refine(domain,bound,options,None)
}

/// Refine until the value-width tolerance is reached, the entire minimum
/// enclosure is strictly outside `band`, or the usual work/resolution limit is
/// reached. Separation uses the complete interval cover and an attained upper
/// bound, never sampled signs alone. The retained enclosure remains valid.
pub fn enclose_outside<E>(domain: Interval,bound: impl FnMut(Interval) -> Result<Interval,E>,
    options: Options,band: Interval) -> Result<Minimum,Error<E>> {
    refine(domain,bound,options,Some(band))
}

pub(crate) fn refine<E>(domain: Interval,bound: impl FnMut(Interval) -> Result<Interval,E>,
    options: Options,band: Option<Interval>) -> Result<Minimum,Error<E>> {
    search(domain,bound,options,band,false)
}

/// `refine`, and with `contain` also stopping as soon as the enclosure lies
/// strictly inside `band`. The enclosures a search reports are nested (a
/// cell's value is intersected with its parent's, and the attained upper
/// bound only falls), so a contained enclosure is where the value stays.
pub(crate) fn search<E>(domain: Interval,mut bound: impl FnMut(Interval) -> Result<Interval,E>,
    options: Options,band: Option<Interval>,contain: bool) -> Result<Minimum,Error<E>> {
    if !options.valid() { return Err(Error::InvalidOptions); }
    let root = bound(domain).map_err(Error::Oracle)?;
    let mut evaluations = 1;
    let mut best = (f64::INFINITY,domain.bounds()[0]);
    let mut sample = |t: f64,parent: Interval| -> Result<(),Error<E>> {
        let p = bound(Interval::point(t).unwrap()).map_err(Error::Oracle)?;
        evaluations += 1;
        let p = overlap(parent,p)?;
        if p.bounds()[1] < best.0 { best = (p.bounds()[1],t); }
        Ok(())
    };
    let [a,b] = domain.bounds();
    for t in [a,midpoint(domain),b] { sample(t,root)?; }
    let mut cells = BinaryHeap::from([Cell {domain,value:root}]);
    loop {
        while cells.peek().is_some_and(|c| c.value.bounds()[0] >= best.0) { cells.pop(); }
        let lower = cells.peek().map_or(best.0,|c| c.value.bounds()[0]);
        let result = |status| Minimum {value:Interval::new(lower,best.0).unwrap(),
            witness:best.1,evaluations,status};
        if (best.0-lower).next_up() <= options.value_tolerance {
            return Ok(result(Status::Converged));
        }
        if band.is_some_and(|b| best.0 < b.bounds()[0] || lower > b.bounds()[1]) {
            return Ok(result(Status::Separated));
        }
        if contain && band.is_some_and(|b| lower > b.bounds()[0] && best.0 < b.bounds()[1]) {
            return Ok(result(Status::Contained));
        }
        if options.max_evaluations-evaluations < 4 {
            return Ok(result(Status::BudgetExhausted));
        }
        let cell = *cells.peek().unwrap();
        let [a,b] = cell.domain.bounds(); let mid = midpoint(cell.domain);
        if mid <= a || mid >= b { return Ok(result(Status::ResolutionLimit)); }
        cells.pop();
        for domain in [Interval::new(a,mid).unwrap(),Interval::new(mid,b).unwrap()] {
            let value = overlap(cell.value,bound(domain).map_err(Error::Oracle)?)?;
            let t = midpoint(domain);
            let sampled = overlap(value,bound(Interval::point(t).unwrap()).map_err(Error::Oracle)?)?;
            evaluations += 2;
            if sampled.bounds()[1] < best.0 { best = (sampled.bounds()[1],t); }
            cells.push(Cell {domain,value});
        }
    }
}
