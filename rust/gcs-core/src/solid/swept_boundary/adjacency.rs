//! Which triangles of an indexed mesh walk each edge and stand on each vertex, built in one sort
//! and read by binary search (`Edges`), or kept up to date as the triangles change (`Live`): the
//! stitch asks these of every boundary edge in every round, where a tree per question, rebuilt
//! per round, was most of its time.
use std::collections::{BTreeSet,HashMap};
use std::hash::{BuildHasherDefault,Hasher};

/// Every directed edge of a mesh with the triangles walking it.
pub struct Edges { keys: Vec<(u64,u32)> }

fn key(a: u32,b: u32) -> u64 { (a as u64) << 32 | b as u64 }
fn split(k: u64) -> (u32,u32) { ((k >> 32) as u32,k as u32) }

impl Edges {
    pub fn new(triangles: &[[u32;3]]) -> Edges {
        let mut keys = Vec::with_capacity(3*triangles.len());
        for (i,t) in triangles.iter().enumerate() { for k in 0..3 { keys.push((key(t[k],t[(k+1)%3]),i as u32)); } }
        keys.sort_unstable();
        Edges {keys}
    }

    fn range(&self,a: u32,b: u32) -> &[(u64,u32)] {
        let k = key(a,b);
        let lo = self.keys.partition_point(|e| e.0 < k);
        let hi = lo+self.keys[lo..].partition_point(|e| e.0 == k);
        &self.keys[lo..hi]
    }

    /// The last triangle (by index) walking `a` to `b`.
    pub fn owner(&self,a: u32,b: u32) -> Option<usize> { self.range(a,b).last().map(|e| e.1 as usize) }
    /// Whether any triangle walks `a` to `b`.
    pub fn walks(&self,a: u32,b: u32) -> bool { !self.range(a,b).is_empty() }
    /// How many triangles use the edge between `a` and `b`, either way.
    pub fn uses(&self,a: u32,b: u32) -> usize { if a == b { self.range(a,b).len() } else { self.range(a,b).len()+self.range(b,a).len() } }
    /// Every directed edge used by one triangle alone (no other walks it either way).
    pub fn boundary(&self) -> BTreeSet<(u32,u32)> {
        let mut undirected: Vec<u64> = self.keys.iter().map(|e| { let (a,b) = split(e.0); key(a.min(b),a.max(b)) }).collect();
        undirected.sort_unstable();
        let mut out = BTreeSet::new();
        let mut i = 0;
        while i < undirected.len() {
            let j = i+undirected[i..].partition_point(|&k| k == undirected[i]);
            if j-i == 1 { let (a,b) = split(undirected[i]); out.insert(if self.walks(a,b) { (a,b) } else { (b,a) }); }
            i = j;
        }
        out
    }
}

/// A hash for integer keys: the standard one is built against adversaries a mesh is not. Nothing
/// is ever read from a map in its own order.
#[derive(Default,Clone,Copy)]
pub struct IntHasher(u64);

impl Hasher for IntHasher {
    fn write(&mut self,bytes: &[u8]) { for &b in bytes { self.write_u64(b as u64); } }
    fn write_u64(&mut self,x: u64) { self.0 = self.0.rotate_left(29)^x; }
    fn write_u32(&mut self,x: u32) { self.write_u64(x as u64); }
    // splitmix64's finaliser, so the low bits a table indexes by depend on every bit of the key
    fn finish(&self) -> u64 {
        let z = (self.0^(self.0 >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
        let z = (z^(z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
        z^(z >> 31)
    }
}

pub type IntMap<K,V> = HashMap<K,V,BuildHasherDefault<IntHasher>>;

/// The triangles walking one directed edge, ascending; one is the common case and allocates nothing.
enum Walkers { One(u32),Many(Vec<u32>) }

impl Walkers {
    fn list(&self) -> &[u32] { match self { Walkers::One(t) => std::slice::from_ref(t),Walkers::Many(v) => v } }
}

/// A mesh's edges and fans kept as its triangles change: which triangles walk each directed
/// edge, which stand on each vertex (ascending, a triangle once per corner there), and which
/// directed edges are boundary (used by one triangle alone). Answers exactly what `Edges` and
/// `Edges::boundary` built afresh from the same triangles would.
pub struct Live { walking: IntMap<u64,Walkers>,incident: Vec<Vec<u32>>,boundary: BTreeSet<(u32,u32)>,touched: Vec<(u32,u32)> }

impl Live {
    pub fn new(vertices: usize,triangles: &[[u32;3]]) -> Live {
        let mut live = Live {walking:IntMap::with_capacity_and_hasher(3*triangles.len(),Default::default()),incident:vec![Vec::new();vertices],boundary:BTreeSet::new(),touched:Vec::new()};
        for (t,tri) in triangles.iter().enumerate() { live.add(t,*tri); }
        live.touched.clear();
        for tri in triangles { for k in 0..3 { let (a,b) = (tri[k],tri[(k+1)%3]); if live.uses(a,b) == 1 { live.boundary.insert((a,b)); } } }
        live
    }

    fn add(&mut self,t: usize,tri: [u32;3]) {
        let t = t as u32;
        for k in 0..3 {
            let (a,b) = (tri[k],tri[(k+1)%3]);
            match self.walking.entry(key(a,b)) {
                std::collections::hash_map::Entry::Vacant(e) => { e.insert(Walkers::One(t)); }
                std::collections::hash_map::Entry::Occupied(mut e) => {
                    let w = e.get_mut();
                    let mut list = w.list().to_vec();
                    let at = list.partition_point(|&x| x <= t);
                    list.insert(at,t);
                    *w = Walkers::Many(list);
                }
            }
            self.touched.push((a,b));
        }
        for v in tri {
            let v = v as usize;
            if v >= self.incident.len() { self.incident.resize(v+1,Vec::new()); }
            let list = &mut self.incident[v];
            let at = list.partition_point(|&x| x <= t);
            list.insert(at,t);
        }
    }

    fn remove(&mut self,t: usize,tri: [u32;3]) {
        let t = t as u32;
        for k in 0..3 {
            let (a,b) = (tri[k],tri[(k+1)%3]);
            let std::collections::hash_map::Entry::Occupied(mut e) = self.walking.entry(key(a,b)) else { continue };
            let mut list = e.get().list().to_vec();
            if let Some(at) = list.iter().position(|&x| x == t) { list.remove(at); }
            match list.len() { 0 => { e.remove(); } 1 => { *e.get_mut() = Walkers::One(list[0]); } _ => { *e.get_mut() = Walkers::Many(list); } }
            self.touched.push((a,b));
        }
        for v in tri {
            let list = &mut self.incident[v as usize];
            if let Some(at) = list.iter().position(|&x| x == t) { list.remove(at); }
        }
    }

    /// Triangle `t` was `old` and is now `new`.
    pub fn replace(&mut self,t: usize,old: [u32;3],new: [u32;3]) { self.remove(t,old); self.add(t,new); }
    /// Triangle `t`, new.
    pub fn push(&mut self,t: usize,tri: [u32;3]) { self.add(t,tri); }

    /// The boundary brought up to date with every edge changed since it last was.
    pub fn settle(&mut self) {
        let mut touched = std::mem::take(&mut self.touched);
        touched.sort_unstable(); touched.dedup();
        for &(a,b) in &touched { for (x,y) in [(a,b),(b,a)] {
            if self.walks(x,y) && self.uses(x,y) == 1 { self.boundary.insert((x,y)); } else { self.boundary.remove(&(x,y)); }
        } }
        touched.clear();
        self.touched = touched;
    }

    fn list(&self,a: u32,b: u32) -> &[u32] { self.walking.get(&key(a,b)).map_or(&[],|w| w.list()) }
    /// The last triangle (by index) walking `a` to `b`.
    pub fn owner(&self,a: u32,b: u32) -> Option<usize> { self.list(a,b).last().map(|&t| t as usize) }
    pub fn walks(&self,a: u32,b: u32) -> bool { !self.list(a,b).is_empty() }
    pub fn uses(&self,a: u32,b: u32) -> usize { if a == b { self.list(a,b).len() } else { self.list(a,b).len()+self.list(b,a).len() } }
    pub fn on(&self,v: u32) -> &[u32] { self.incident.get(v as usize).map_or(&[],|l| &l[..]) }
    /// The directed edges used by one triangle alone. `settle` first.
    pub fn boundary(&self) -> &BTreeSet<(u32,u32)> { &self.boundary }
}
