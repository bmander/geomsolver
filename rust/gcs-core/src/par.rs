//! Independent work spread over the machine's cores, answered in order: `map(items, f)` is
//! `items.iter().map(f).collect()` whatever the threads did, so a caller's result never depends on
//! how the work was split. WebAssembly has no threads, and there it is that serial map.

/// How many threads `map` uses at most: the machine's parallelism.
pub fn threads() -> usize {
    #[cfg(target_arch = "wasm32")]
    { 1 }
    #[cfg(not(target_arch = "wasm32"))]
    { std::thread::available_parallelism().map_or(1,|n| n.get()) }
}

/// `f` of every item, in the items' order, the items taken one at a time by as many threads as
/// there are cores (or items).
pub fn map<T: Sync,R: Send>(items: &[T],f: impl Fn(&T) -> R + Sync) -> Vec<R> {
    indices(items.len(),|i| f(&items[i]))
}

/// `f(0), f(1), …, f(n - 1)`, in that order, spread over the cores as `map` is.
pub fn indices<R: Send>(n: usize,f: impl Fn(usize) -> R + Sync) -> Vec<R> {
    indices_with(n,|| (),|_,i| f(i))
}

/// The same, each thread given its own `state` (made by `init`, once a thread) to work with: a
/// cache, say, whose contents change how fast `f` answers but never what.
pub fn indices_with<S,R: Send>(n: usize,init: impl Fn() -> S + Sync,f: impl Fn(&mut S,usize) -> R + Sync) -> Vec<R> {
    let threads = threads().min(n);
    if threads <= 1 { let mut state = init(); return (0..n).map(|i| f(&mut state,i)).collect(); }
    #[cfg(target_arch = "wasm32")]
    { unreachable!("one thread on wasm") }
    #[cfg(not(target_arch = "wasm32"))]
    {
        use std::sync::{Mutex,atomic::{AtomicUsize,Ordering}};
        let next = AtomicUsize::new(0);
        let done: Mutex<Vec<(usize,R)>> = Mutex::new(Vec::with_capacity(n));
        std::thread::scope(|scope| {
            for _ in 0..threads {
                scope.spawn(|| {
                    let mut mine = Vec::new();
                    let mut state = init();
                    loop {
                        let i = next.fetch_add(1,Ordering::Relaxed);
                        if i >= n { break; }
                        mine.push((i,f(&mut state,i)));
                    }
                    done.lock().unwrap_or_else(|e| e.into_inner()).extend(mine);
                });
            }
        });
        let mut done = done.into_inner().unwrap_or_else(|e| e.into_inner());
        done.sort_unstable_by_key(|(i,_)| *i);
        done.into_iter().map(|(_,r)| r).collect()
    }
}
