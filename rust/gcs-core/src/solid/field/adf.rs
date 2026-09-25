//! An adaptive distance field: a lazily built octree over an exact one-Lipschitz field, holding
//! the exact value at each cell's corners and read between them by trilinear interpolation.
//!
//! A swept field's exact value is a search over the roll, tens of tool evaluations; a mesher asks
//! for hundreds of thousands of values, most of them within a cell or two of ones already asked.
//! The octree is refined only where the surface may pass: a cell whose corners all read farther
//! than its diagonal from zero, with one sign, holds no boundary (the field is one-Lipschitz), and
//! is read as it is. Any other is split while it is wider than `Resolution::coarsest`, and then
//! while the exact value at its centre is off the interpolated one by more than the tolerance,
//! down to cells `finest` across (Frisken's adaptive distance fields): a smooth stretch of surface
//! is read from cells as wide as its curvature allows, a crease from the finest. What comes back
//! is a reading, never an interval claim: the centre test samples the interpolation's error, it
//! does not bound it.
use std::collections::HashMap;
use std::hash::{BuildHasherDefault,Hasher};

type P = [f64;3];

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

pub(super) type Map<K,V> = HashMap<K,V,BuildHasherDefault<Mix>>;

/// A cell met on the way down: its corners' values and whether they settle it as a leaf.
#[derive(Debug,Clone,Copy)]
struct Cell { v: [f64;8], leaf: bool }

/// How finely an adaptive distance field is refined near the surface: never coarser than
/// `coarsest` there, finer while a cell's centre reads more than `tolerance` off its
/// interpolation, and never finer than `finest`.
#[derive(Debug,Clone,Copy,PartialEq)]
pub struct Resolution { pub finest: f64, pub coarsest: f64, pub tolerance: f64 }

#[derive(Debug)]
pub(super) struct Adf {
    /// The side of a root cell, and the levels below it to the finest.
    root: f64,
    levels: u32,
    /// The deepest level whose cells are no wider than `Resolution::coarsest`, and the tolerance.
    coarse: u32,
    tolerance: f64,
    /// Exact values at corners, by integer coordinates in finest cells.
    corners: Map<[i64;3],f64>,
    /// Cells by level and integer coordinates at that level, so a read costs a lookup a level.
    cells: Map<(u32,[i64;3]),Cell>,
    /// Recent leaves by the finest cell a read fell in, direct-mapped: a mesher's reads come in
    /// runs a finest cell apart, and an indexed cut's copies read the one octree in turns, so a
    /// single remembered leaf is taken back by the next copy before its own next read.
    recent: Box<[Option<([i64;3],u32,[i64;3],Cell)>]>,
}

/// Slots in `Adf::recent`, a power of two.
const RECENT: usize = 1024;

impl Adf {
    /// An octree refined to `resolution`, rooted in cells `root` across.
    pub(super) fn new(root: f64,resolution: Resolution) -> Self {
        let depth = |size: f64| (root/size).log2().ceil().clamp(0.,20.) as u32;
        let levels = depth(resolution.finest);
        let coarse = depth(resolution.coarsest).min(levels);
        Self {root,levels,coarse,tolerance:resolution.tolerance,corners:Map::default(),cells:Map::default(),
            recent:vec![None;RECENT].into_boxed_slice()}
    }

    /// The value and gradient at `p`, from the leaf cell holding it; `exact` is asked for each
    /// corner not yet known.
    pub(super) fn read(&mut self,p: P,exact: &mut dyn FnMut(P) -> f64) -> (f64,P) {
        let unit = self.root/(1u64 << self.levels) as f64;
        let size_at = |level: u32| unit*(1i64 << (self.levels-level)) as f64;
        let fine: [i64;3] = p.map(|x| (x/unit).floor() as i64);
        let slot = {
            let mut h = Mix::default();
            for k in fine { h.write_i64(k); }
            h.finish() as usize >> 7 & (RECENT-1)
        };
        if let Some((key,level,cell,c)) = self.recent[slot] {
            if key == fine { return interpolate(p,size_at(level),cell,&c.v); }
        }
        let mut level = 0;
        loop {
            let size = size_at(level);
            let cell: [i64;3] = p.map(|x| (x/size).floor() as i64);
            let c = match self.cells.get(&(level,cell)) {
                Some(&c) => c,
                None => {
                    let span = 1i64 << (self.levels-level);
                    let mut v = [0f64;8];
                    for (k,slot) in v.iter_mut().enumerate() {
                        let key = [(cell[0]+(k & 1) as i64)*span,(cell[1]+((k >> 1) & 1) as i64)*span,
                            (cell[2]+((k >> 2) & 1) as i64)*span];
                        *slot = *self.corners.entry(key).or_insert_with(|| exact(key.map(|c| c as f64*unit)));
                    }
                    // no boundary can cross a cell whose corners all read beyond its diagonal, one way
                    let reach = size*3f64.sqrt();
                    let clear = v.iter().all(|&x| x > reach) || v.iter().all(|&x| x < -reach);
                    // the centre, a corner of the cells it would split into, against the mean of
                    // the corners, which is what the interpolation reads there
                    let leaf = clear || level == self.levels || (level >= self.coarse && {
                        let half = span/2;
                        let key = cell.map(|c| c*span+half);
                        let centre = *self.corners.entry(key).or_insert_with(|| exact(key.map(|c| c as f64*unit)));
                        (centre-v.iter().sum::<f64>()/8.).abs() <= self.tolerance
                    });
                    let c = Cell {v,leaf};
                    self.cells.insert((level,cell),c);
                    c
                }
            };
            if c.leaf {
                self.recent[slot] = Some((fine,level,cell,c));
                return interpolate(p,size,cell,&c.v);
            }
            level += 1;
        }
    }
}

/// The trilinear value and its gradient, in world units, at `p` in the cell `size` across at
/// integer coordinates `cell` whose corners read `v`.
fn interpolate(p: P,size: f64,cell: [i64;3],v: &[f64;8]) -> (f64,P) {
    let [x,y,z]: P = std::array::from_fn(|k| p[k]/size-cell[k] as f64);
    let lerp = |a: f64,b: f64,t: f64| a+(b-a)*t;
    let c00 = lerp(v[0],v[1],x); let c10 = lerp(v[2],v[3],x);
    let c01 = lerp(v[4],v[5],x); let c11 = lerp(v[6],v[7],x);
    let value = lerp(lerp(c00,c10,y),lerp(c01,c11,y),z);
    let dx = lerp(lerp(v[1]-v[0],v[3]-v[2],y),lerp(v[5]-v[4],v[7]-v[6],y),z);
    let dy = lerp(lerp(v[2]-v[0],v[3]-v[1],x),lerp(v[6]-v[4],v[7]-v[5],x),z);
    let dz = lerp(lerp(v[4]-v[0],v[5]-v[1],x),lerp(v[6]-v[2],v[7]-v[3],x),y);
    (value,[dx/size,dy/size,dz/size])
}
