//! Wall-clock time where the target has a clock. `std::time::Instant::now` panics on
//! `wasm32-unknown-unknown`, which has none: there an instant is nothing and every elapsed time
//! zero, so the core's progress and timing lines read zero in the browser and never stop it.
#[derive(Clone,Copy,Debug)]
pub struct Instant(#[cfg(not(target_arch = "wasm32"))] std::time::Instant);

impl Instant {
    #[cfg(not(target_arch = "wasm32"))]
    pub fn now() -> Instant { Instant(std::time::Instant::now()) }
    #[cfg(target_arch = "wasm32")]
    pub fn now() -> Instant { Instant() }
    #[cfg(not(target_arch = "wasm32"))]
    pub fn elapsed(&self) -> std::time::Duration { self.0.elapsed() }
    #[cfg(target_arch = "wasm32")]
    pub fn elapsed(&self) -> std::time::Duration { std::time::Duration::ZERO }
}
