//! **Warm starts a thread keeps for its searches**: the last few feet found on each kind of
//! surface whose inverse is a search (a B-spline sheet, a blend, a face family's wall), by the
//! surface's address — a trace asks of one point after the next a step apart, so the next foot is a
//! Newton step or two from the last. A start changes where a search begins, and so the last bits
//! of what it settles on; so what a thread was asked before must not reach what it answers next
//! where that would make a result depend on anything but its inputs (#735's determinism gate):
//! the starts are forgotten at the start of every parallel item (`util::par`, which deals items to
//! threads as they come free) and of every arrangement (`boolean`), and kept as they were across
//! anything only narration asks (`kept`), so `SOLVENT_BREP_DEBUG` changes no result.
use std::cell::Cell;

/// The kinds of search a start is kept for, each its own few.
#[derive(Clone,Copy,Debug,PartialEq,Eq)]
pub enum Kind { Net = 0,Blend = 1,Wall = 2 }

/// How many starts a kind keeps, the newest first.
const KEPT: usize = 4;

type Starts = [[(usize,[f64;2]);KEPT];3];

thread_local! {
    static STARTS: Cell<Starts> = const { Cell::new([[(0,[0.;2]);KEPT];3]) };
}

/// The start kept for the surface at `key` of kind `k`, if any.
pub fn get(k: Kind,key: usize) -> Option<[f64;2]> {
    STARTS.with(|s| s.get()[k as usize].iter().find(|f| f.0 == key && key != 0).map(|f| f.1))
}

/// `uv` kept as the start for the surface at `key` of kind `k`, the oldest of its kind dropped.
pub fn put(k: Kind,key: usize,uv: [f64;2]) {
    STARTS.with(|s| {
        let mut all = s.get();
        let last = all[k as usize];
        let mut next = [(key,uv);KEPT];
        let mut i = 1;
        for f in last { if f.0 != key && i < KEPT { next[i] = f; i += 1; } }
        all[k as usize] = next;
        s.set(all);
    })
}

/// Every start this thread keeps forgotten.
pub fn forget() { STARTS.with(|s| s.set([[(0,[0.;2]);KEPT];3])) }

/// `f`'s answer, the starts as they were before it once it is done: what narration asks.
pub fn kept<R>(f: impl FnOnce() -> R) -> R {
    let before = STARTS.with(|s| s.get());
    let r = f();
    STARTS.with(|s| s.set(before));
    r
}
