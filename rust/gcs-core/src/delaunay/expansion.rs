//! Exact floating-point expansions (Shewchuk, "Adaptive Precision Floating-Point Arithmetic and
//! Fast Robust Geometric Predicates", 1997): a number held as a sum of nonoverlapping doubles in
//! increasing magnitude, so that sums, differences and products of doubles are exact. Only the
//! predicates' exact fallback uses these; a filtered evaluation decides nearly every call.
//!
//! Every operation eliminates zero components, and the last component of a nonzero expansion is
//! its most significant, so the sign of an expansion is the sign of its last component. Assumes
//! round-to-nearest binary64 without overflow or underflow, as Shewchuk's do.

const SPLITTER: f64 = 134_217_729.; // 2^27 + 1

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
#[derive(Clone,Debug,Default)]
pub struct Expansion(pub(crate) Vec<f64>);

impl Expansion {
    pub fn from(a: f64) -> Self { if a == 0. { Self(Vec::new()) } else { Self(vec![a]) } }
    /// `a − b`, exactly.
    pub fn diff(a: f64,b: f64) -> Self {
        let (x,y) = two_diff(a,b);
        Self([y,x].into_iter().filter(|v| *v != 0.).collect())
    }
    /// `a · b`, exactly.
    pub fn product(a: f64,b: f64) -> Self {
        let (x,y) = two_product(a,b);
        Self([y,x].into_iter().filter(|v| *v != 0.).collect())
    }
    /// The sign: −1, 0 or +1.
    pub fn sign(&self) -> i8 {
        match self.0.last() { None => 0, Some(v) if *v > 0. => 1, Some(v) if *v < 0. => -1, Some(_) => 0 }
    }
    /// An approximation of the value.
    pub fn estimate(&self) -> f64 { self.0.iter().sum() }

    /// `self + b`, for a double `b` (Shewchuk's GROW-EXPANSION with zero elimination).
    fn grow(&self,b: f64) -> Self {
        let mut h = Vec::with_capacity(self.0.len()+1);
        let mut q = b;
        for &e in &self.0 {
            let (x,y) = two_sum(q,e);
            q = x;
            if y != 0. { h.push(y); }
        }
        if q != 0. { h.push(q); }
        Self(h)
    }
    pub fn add(&self,other: &Self) -> Self {
        let (small,large) = if self.0.len() < other.0.len() { (self,other) } else { (other,self) };
        let mut h = large.clone();
        for &b in &small.0 { h = h.grow(b); }
        h
    }
    pub fn neg(&self) -> Self { Self(self.0.iter().map(|v| -v).collect()) }
    pub fn sub(&self,other: &Self) -> Self { self.add(&other.neg()) }
    /// `self · b`, for a double `b` (SCALE-EXPANSION with zero elimination).
    fn scale(&self,b: f64) -> Self {
        let mut h = Vec::with_capacity(2*self.0.len());
        let Some((&first,rest)) = self.0.split_first() else { return Self(h) };
        let (mut q,low) = two_product(first,b);
        if low != 0. { h.push(low); }
        for &e in rest {
            let (p1,p0) = two_product(e,b);
            let (sum,low) = two_sum(q,p0);
            if low != 0. { h.push(low); }
            let (x,y) = fast_two_sum(p1,sum);
            q = x;
            if y != 0. { h.push(y); }
        }
        if q != 0. { h.push(q); }
        Self(h)
    }
    pub fn mul(&self,other: &Self) -> Self {
        let (small,large) = if self.0.len() < other.0.len() { (self,other) } else { (other,self) };
        let mut h = Self::default();
        for &b in &small.0 { h = h.add(&large.scale(b)); }
        h
    }
}
