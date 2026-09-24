//! Exact floating-point expansions (Shewchuk, "Adaptive Precision Floating-Point Arithmetic and
//! Fast Robust Geometric Predicates", 1997): a number held as a sum of nonoverlapping doubles in
//! increasing magnitude, so that sums, differences and products of doubles are exact. Only the
//! predicates' exact fallback uses these; a filtered evaluation decides nearly every call.
//!
//! Every operation eliminates zero components, and the last component of a nonzero expansion is
//! its most significant, so the sign of an expansion is the sign of its last component. Assumes
//! round-to-nearest-even binary64 without overflow or underflow, as Shewchuk's do; the sums are
//! his FAST-EXPANSION-SUM, which needs that rounding to keep its output strongly nonoverlapping.
//!
//! Components live inline up to `INLINE` and spill to the heap past it: the exact evaluations
//! degenerate input actually meets (a lattice, a plane of points) stay short and allocate nothing.

const SPLITTER: f64 = 134_217_729.; // 2^27 + 1
const INLINE: usize = 24;

#[inline]
pub(crate) fn two_sum(a: f64,b: f64) -> (f64,f64) {
    let x = a+b;
    let bv = x-a;
    let av = x-bv;
    (x,(a-av)+(b-bv))
}

#[inline]
fn fast_two_sum(a: f64,b: f64) -> (f64,f64) {
    let x = a+b;
    (x,b-(x-a))
}

#[inline]
pub(crate) fn two_diff(a: f64,b: f64) -> (f64,f64) {
    let x = a-b;
    let bv = a-x;
    let av = x+bv;
    (x,(a-av)+(bv-b))
}

#[inline]
fn split(a: f64) -> (f64,f64) {
    let c = SPLITTER*a;
    let hi = c-(c-a);
    (hi,a-hi)
}

#[inline]
pub(crate) fn two_product(a: f64,b: f64) -> (f64,f64) {
    let x = a*b;
    let (ahi,alo) = split(a);
    let (bhi,blo) = split(b);
    let err = ((x-ahi*bhi)-alo*bhi)-ahi*blo;
    (x,alo*blo-err)
}

/// An exact sum of doubles, least significant first.
#[derive(Clone)]
pub struct Expansion { len: usize,inline: [f64;INLINE],heap: Vec<f64> }

impl std::fmt::Debug for Expansion {
    fn fmt(&self,f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_list().entries(self.parts()).finish()
    }
}

impl Default for Expansion {
    fn default() -> Self { Self {len:0,inline:[0.;INLINE],heap:Vec::new()} }
}

impl Expansion {
    fn with_capacity(n: usize) -> Self {
        let mut e = Self::default();
        if n > INLINE { e.heap.reserve(n); }
        e
    }
    #[inline]
    fn push(&mut self,v: f64) {
        if self.len < INLINE { self.inline[self.len] = v; }
        else {
            if self.len == INLINE { self.heap.extend_from_slice(&self.inline); }
            self.heap.push(v);
        }
        self.len += 1;
    }
    /// The components, least significant first.
    pub fn parts(&self) -> &[f64] {
        if self.len <= INLINE { &self.inline[..self.len] } else { &self.heap }
    }

    pub fn from(a: f64) -> Self { let mut e = Self::default(); if a != 0. { e.push(a); } e }
    /// `a − b`, exactly.
    pub fn diff(a: f64,b: f64) -> Self {
        let (x,y) = two_diff(a,b);
        let mut e = Self::default();
        if y != 0. { e.push(y); }
        if x != 0. { e.push(x); }
        e
    }
    /// `a · b`, exactly.
    pub fn product(a: f64,b: f64) -> Self {
        let (x,y) = two_product(a,b);
        let mut e = Self::default();
        if y != 0. { e.push(y); }
        if x != 0. { e.push(x); }
        e
    }
    /// The sign: −1, 0 or +1.
    pub fn sign(&self) -> i8 {
        match self.parts().last() {
            Some(v) if *v > 0. => 1,
            Some(v) if *v < 0. => -1,
            _ => 0,
        }
    }
    /// An approximation of the value.
    pub fn estimate(&self) -> f64 { self.parts().iter().sum() }

    /// `self + other` (FAST-EXPANSION-SUM with zero elimination): one merge by magnitude.
    pub fn add(&self,other: &Self) -> Self {
        let (e,f) = (self.parts(),other.parts());
        if e.is_empty() { return other.clone(); }
        if f.is_empty() { return self.clone(); }
        let mut h = Self::with_capacity(e.len()+f.len());
        let (mut i,mut j) = (0,0);
        // The component of smaller magnitude of the two fronts goes next.
        let smaller_is_e = |a: f64,b: f64| (b > a) == (b > -a);
        let mut q;
        if smaller_is_e(e[0],f[0]) { q = e[0]; i = 1; } else { q = f[0]; j = 1; }
        if i < e.len() && j < f.len() {
            let (x,y) = if smaller_is_e(e[i],f[j]) { i += 1; fast_two_sum(e[i-1],q) }
                else { j += 1; fast_two_sum(f[j-1],q) };
            q = x;
            if y != 0. { h.push(y); }
            while i < e.len() && j < f.len() {
                let (x,y) = if smaller_is_e(e[i],f[j]) { i += 1; two_sum(q,e[i-1]) }
                    else { j += 1; two_sum(q,f[j-1]) };
                q = x;
                if y != 0. { h.push(y); }
            }
        }
        for &v in e[i..].iter().chain(&f[j..]) {
            let (x,y) = two_sum(q,v);
            q = x;
            if y != 0. { h.push(y); }
        }
        if q != 0. { h.push(q); }
        h
    }
    pub fn neg(&self) -> Self {
        let mut h = Self::with_capacity(self.len);
        for &v in self.parts() { h.push(-v); }
        h
    }
    pub fn sub(&self,other: &Self) -> Self { self.add(&other.neg()) }
    /// `self · b`, for a double `b` (SCALE-EXPANSION with zero elimination).
    fn scale(&self,b: f64) -> Self {
        let e = self.parts();
        let mut h = Self::with_capacity(2*e.len());
        let Some((&first,rest)) = e.split_first() else { return h };
        let (mut q,low) = two_product(first,b);
        if low != 0. { h.push(low); }
        for &v in rest {
            let (p1,p0) = two_product(v,b);
            let (sum,low) = two_sum(q,p0);
            if low != 0. { h.push(low); }
            let (x,y) = fast_two_sum(p1,sum);
            q = x;
            if y != 0. { h.push(y); }
        }
        if q != 0. { h.push(q); }
        h
    }
    pub fn mul(&self,other: &Self) -> Self {
        let (small,large) = if self.len < other.len { (self,other) } else { (other,self) };
        let mut h = Self::default();
        for &b in small.parts() { h = h.add(&large.scale(b)); }
        h
    }
}
