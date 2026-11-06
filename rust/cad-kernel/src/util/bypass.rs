//! **Recognitions bypassed for a computation** (#810): the bits of a set the caller names (the
//! kernel's are `ssi::Recognised`), set on this thread for the length of a closure and on every
//! worker `util::par` starts inside it, as the meter is (`util::meter`). With none set nothing is
//! bypassed.
use std::cell::Cell;

thread_local! {
    static BITS: Cell<u8> = const { Cell::new(0) };
}

/// The bits bypassed on this thread.
pub fn current() -> u8 { BITS.with(Cell::get) }

/// `f` run with exactly `bits` bypassed on this thread, the bits before put back after — however
/// `f` ends, a panic too.
pub fn under<R>(bits: u8,f: impl FnOnce() -> R) -> R {
    struct Restore(u8);
    impl Drop for Restore { fn drop(&mut self) { BITS.with(|b| b.set(self.0)) } }
    let _restore = Restore(BITS.with(|b| b.replace(bits)));
    f()
}
