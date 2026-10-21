//! The field's caches and the keys they are read by.
//!
//! Every cache here holds what an immutable snapshot computes — a sweep's poses and floor bounds,
//! its adaptive distance fields, a union's candidate operands per cell — so none is ever
//! invalidated: an edit to the model builds new fields, and new caches with them. A cache shared
//! between clones is shared because the clones are the same field, an indexed cut's copies placed
//! apart.
use std::collections::HashMap;
use std::hash::{BuildHasherDefault,Hasher};
use std::sync::{Mutex,MutexGuard,PoisonError};

/// A multiply-rotate hash for integer keys: the octree asks one lookup per level of every read,
/// and SipHash's resistance to chosen keys buys nothing against coordinates we mint ourselves.
#[derive(Default)]
pub(super) struct Mix(u64);

impl Hasher for Mix {
    fn finish(&self) -> u64 { self.0 }
    fn write(&mut self,bytes: &[u8]) { for &b in bytes { self.write_u64(b as u64); } }
    fn write_u64(&mut self,x: u64) { self.0 = (self.0.rotate_left(5) ^ x).wrapping_mul(0x51_7c_c1_b7_27_22_0a_95); }
    fn write_i64(&mut self,x: i64) { self.write_u64(x as u64); }
    fn write_u32(&mut self,x: u32) { self.write_u64(x as u64); }
    fn write_usize(&mut self,x: usize) { self.write_u64(x as u64); }
}

/// A map with `Mix` keys. Looked up and filled, never iterated: its order is no one's business.
pub(super) type Map<K,V> = HashMap<K,V,BuildHasherDefault<Mix>>;

/// Numbers as a key: two are one entry only where they are the same numbers, bit for bit.
pub(super) fn bits<const N: usize>(x: [f64;N]) -> [u64;N] { x.map(f64::to_bits) }

/// An interval box as a key (`bits` of each side's ends).
pub(super) fn box_bits(b: &super::V) -> [[u64;2];3] { b.map(|x| bits(x.bounds())) }

/// A grid of cubes `h` across, the cube a point is in named by its integer coordinates, clamped
/// to `i32` so a point far out is in the outermost cube rather than wrapped round.
#[derive(Clone,Copy,Debug)]
pub(super) struct Grid(pub f64);

impl Grid {
    /// Whether the grid is one: a positive, finite side.
    pub(super) fn usable(self) -> bool { self.0 > 0. && self.0.is_finite() }
    pub(super) fn key(self,p: [f64;3]) -> [i32;3] { p.map(|x| (x/self.0).floor().clamp(i32::MIN as f64,i32::MAX as f64) as i32) }
    pub(super) fn centre(self,key: [i32;3]) -> [f64;3] { key.map(|k| (k as f64+0.5)*self.0) }
}

/// A `Map` behind a lock, filled by whichever thread asks first. A panic while it was held leaves
/// every entry it had written whole, so a poisoned lock is taken back rather than the cache
/// quietly given up.
#[derive(Debug)]
pub(super) struct Memo<K,V>(Mutex<Map<K,V>>);

impl<K,V> Default for Memo<K,V> { fn default() -> Self { Self(Mutex::new(Map::default())) } }

impl<K,V> Memo<K,V> {
    pub(super) fn lock(&self) -> MutexGuard<'_,Map<K,V>> { self.0.lock().unwrap_or_else(PoisonError::into_inner) }
}
