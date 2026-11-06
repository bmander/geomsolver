//! Finite closed intervals with outward rounding. Arithmetic encloses real
//! operations on binary64 endpoints; invalid domains and overflow fail closed.
//! Trigonometric bounds use Taylor polynomials and explicit remainder bounds,
//! not the platform libm's unspecified transcendental accuracy.

#[allow(unused_imports)]
use crate::util::fmath::Det;
use std::ops::{Add,Div,Mul,Neg,Sub};
pub mod minimum;

#[derive(Clone,Copy,Debug,PartialEq,Eq)]
pub enum Error { InvalidBounds, Overflow, DivisionByZero, OutsideDomain }

/// `1/n!` for `n` from 0 to 21, each enclosed in outward-rounded division, read once.
fn reciprocals() -> &'static [Interval;22] {
    static TABLE: std::sync::OnceLock<[Interval;22]> = std::sync::OnceLock::new();
    TABLE.get_or_init(|| {
        let mut out = [Interval::ONE;22];
        for n in 1..22 { out[n] = out[n-1].div(Interval {lo:n as f64,hi:n as f64}).expect("a positive factorial"); }
        out
    })
}

#[derive(Clone,Copy,Debug,PartialEq)]
pub struct Interval { lo: f64,hi: f64 }

impl Interval {
    pub const ZERO: Self = Self {lo:0.,hi:0.};
    pub const ONE: Self = Self {lo:1.,hi:1.};

    pub fn new(lo: f64,hi: f64) -> Result<Self,Error> {
        if !lo.is_finite() || !hi.is_finite() || lo > hi { Err(Error::InvalidBounds) }
        else { Ok(Self {lo,hi}) }
    }
    pub fn point(x: f64) -> Result<Self,Error> { Self::new(x,x) }
    pub fn bounds(self) -> [f64;2] { [self.lo,self.hi] }
    /// The midpoint in plain floating point: a reading, never an enclosure.
    pub fn mid(self) -> f64 { 0.5*(self.lo+self.hi) }
    pub fn contains(self,x: f64) -> bool { self.lo <= x && x <= self.hi }
    pub fn neg(self) -> Self { Self {lo:-self.hi,hi:-self.lo} }

    fn outward(lo: f64,hi: f64) -> Result<Self,Error> {
        let (lo,hi) = (lo.next_down(),hi.next_up());
        if !lo.is_finite() || !hi.is_finite() { Err(Error::Overflow) }
        else { Self::new(lo,hi) }
    }

    pub fn add(self,b: Self) -> Result<Self,Error> { Self::outward(self.lo+b.lo,self.hi+b.hi) }
    pub fn sub(self,b: Self) -> Result<Self,Error> { Self::outward(self.lo-b.hi,self.hi-b.lo) }
    pub fn mul(self,b: Self) -> Result<Self,Error> {
        let p = [self.lo*b.lo,self.lo*b.hi,self.hi*b.lo,self.hi*b.hi];
        Self::outward(p.into_iter().fold(f64::INFINITY,f64::min),
            p.into_iter().fold(f64::NEG_INFINITY,f64::max))
    }
    pub fn div(self,b: Self) -> Result<Self,Error> {
        if b.contains(0.) { return Err(Error::DivisionByZero); }
        let p = [self.lo/b.lo,self.lo/b.hi,self.hi/b.lo,self.hi/b.hi];
        Self::outward(p.into_iter().fold(f64::INFINITY,f64::min),
            p.into_iter().fold(f64::NEG_INFINITY,f64::max))
    }
    pub fn square(self) -> Result<Self,Error> {
        let a = self.lo*self.lo; let b = self.hi*self.hi;
        let mut result = Self::outward(if self.contains(0.) { 0. } else { a.min(b) },a.max(b))?;
        result.lo = result.lo.max(0.);
        Ok(result)
    }
    pub fn sqrt(self) -> Result<Self,Error> {
        if self.lo < 0. { return Err(Error::OutsideDomain); }
        // IEEE sqrt is correctly rounded. Preserve the known nonnegative range.
        let mut result = Self::outward(self.lo.sqrt(),self.hi.sqrt())?;
        result.lo = result.lo.max(0.);
        Ok(result)
    }

    /// Enclose sin and cos over an interval anywhere, by a reduction justified here, so that
    /// `sin_cos`'s own guarantee is untouched. An interval at least 6.3 wide (a computed width
    /// under-reads the true one by at most an ulp, and 6.3 exceeds 2π) holds a whole period, so
    /// both are [-1,1]. Any other is shifted by a whole number k of turns, with 2π enclosed
    /// between the doubles either side of TAU — correctly rounded, so within an ulp of it — and
    /// the shift made in outward-rounded arithmetic: the result holds x − 2πk for every x in the
    /// input, which has the same sine and cosine, and lies within π + 3.15 of zero, inside
    /// `sin_cos`'s domain.
    pub fn sin_cos_periodic(self) -> Result<(Self,Self),Error> {
        if self.lo.abs().max(self.hi.abs()) <= 8. { return self.sin_cos(); }
        if !(self.hi-self.lo < 6.3) {
            let full = Self::new(-1.,1.)?;
            return Ok((full,full));
        }
        let tau = std::f64::consts::TAU;
        let turn = Self::new(f64::from_bits(tau.to_bits()-1),f64::from_bits(tau.to_bits()+1))?;
        let k = ((self.lo*0.5+self.hi*0.5)/tau).round();
        self.sub(turn.mul(Self::point(k)?)?)?.sin_cos()
    }

    /// Enclose sin and cos over an interval within 0.8 of nought (a hair past an eighth of a turn):
    /// Taylor polynomials of degrees 21 (sin) and 20 (cos), their Lagrange remainders at most
    /// `r²¹/21!` and `r²²/22!` (every derivative of sin and cos at most 1; a sine's next term is of
    /// degree 23), `r ≤ 0.8` their radius — both below 1e-21, which bounds them.
    fn quarter(self) -> Result<(Self,Self),Error> {
        // the series by Horner's rule in x², its coefficients ±1/n! enclosed once (`RECIPROCALS`)
        let f = reciprocals();
        let x2 = self.square()?;
        let mut sine = f[21];
        let mut cosine = f[20];
        for n in (1..=10).rev() {
            sine = f[2*n-1].sub(x2.mul(sine)?)?;
            cosine = f[2*n-2].sub(x2.mul(cosine)?)?;
        }
        let sine = self.mul(sine)?;
        // (0.8²¹/21! < 1.8e-22 and 0.8²²/22! smaller still: one constant bounds both)
        let remainder = Self::new(-1e-21,1e-21)?;
        let (sine,cosine) = (sine.add(remainder)?,cosine.add(remainder)?);
        let clip = |x: Self| Self::new(x.lo.max(-1.),x.hi.min(1.));
        Ok((clip(sine)?,clip(cosine)?))
    }

    /// Enclose sin and cos for the entire input interval in [-8,8] radians.
    /// No argument reduction or approximate pi enters the guarantee. Larger
    /// inputs are refused; callers must supply a separately justified reduction.
    pub fn sin_cos(self) -> Result<(Self,Self),Error> {
        let radius = self.lo.abs().max(self.hi.abs());
        if radius > 8. { return Err(Error::OutsideDomain); }
        if self.lo != self.hi {
            // Evaluate the Taylor polynomial at a point, then enclose the
            // displacement with angle addition. Substituting a wide interval
            // directly into every Taylor power loses the repeated dependence on
            // x and becomes especially loose near the far end of this domain.
            let mid = Self::point(self.lo*0.5+self.hi*0.5)?;
            let (s,c) = mid.sin_cos()?;
            let delta = self.sub(mid)?;
            let r = delta.lo.abs().max(delta.hi.abs());
            // |sin(delta)| <= min(|delta|,1), cos(delta) >= 1-delta^2/2.
            let sd = Self::new(-r.min(1.),r.min(1.))?;
            let lower_cos = Self::ONE.sub(Self::point(r)?.square()?.div(Self::point(2.)?)?)?.lo;
            let cd = Self::new(lower_cos.max(-1.),1.)?;
            let sine = s.mul(cd)?.add(c.mul(sd)?)?;
            let cosine = c.mul(cd)?.sub(s.mul(sd)?)?;
            let clip = |x: Self| Self::new(x.lo.max(-1.),x.hi.min(1.));
            return Ok((clip(sine)?,clip(cosine)?));
        }
        // a point past an eighth of a turn: shifted by whole quarter turns into [−π/4, π/4] — π/2
        // enclosed between the doubles either side of it, the shift made in outward-rounded
        // arithmetic, so the result holds x − kπ/2 for the exact x — where the series needs ten
        // terms each, not twenty-four (#814: a certified search reads sines at every stretch), and
        // the quadrant's sine and cosine read back exactly (a swap and a sign)
        if radius > std::f64::consts::FRAC_PI_4 {
            let k = (self.lo/std::f64::consts::FRAC_PI_2).round();
            let half_pi = Self::new(std::f64::consts::FRAC_PI_2,std::f64::consts::FRAC_PI_2.next_up())?;
            let reduced = self.sub(half_pi.mul(Self::point(k)?)?)?;
            if reduced.lo.abs().max(reduced.hi.abs()) <= 0.8 {
                let (s,c) = reduced.quarter()?;
                return Ok(match (k as i64).rem_euclid(4) { 0 => (s,c),1 => (c,s.neg()),2 => (s.neg(),c.neg()),_ => (c.neg(),s) })
            }
        }
        if radius <= 0.8 { return self.quarter() }
        let x2 = self.square()?;
        let (mut sine,mut cosine) = (self,Self::ONE);
        let (mut st,mut ct) = (self,Self::ONE);
        for k in 1..=24 {
            ct = ct.mul(x2)?.neg().div(Self::point(((2*k-1)*(2*k)) as f64)?)?;
            st = st.mul(x2)?.neg().div(Self::point(((2*k)*(2*k+1)) as f64)?)?;
            cosine = cosine.add(ct)?;
            sine = sine.add(st)?;
        }
        // Taylor degrees 48 and 49. Every higher derivative of sin/cos has
        // magnitude <= 1, so Lagrange remainders are M^49/49! and M^50/50!.
        let m = Self::point(radius)?;
        let mut remainder = Self::ONE;
        for k in 1..=49 { remainder = remainder.mul(m)?.div(Self::point(k as f64)?)?; }
        cosine = cosine.add(Self::new(-remainder.hi,remainder.hi)?)?;
        remainder = remainder.mul(m)?.div(Self::point(50.)?)?;
        sine = sine.add(Self::new(-remainder.hi,remainder.hi)?)?;
        let clip = |x: Self| Self::new(x.lo.max(-1.),x.hi.min(1.));
        Ok((clip(sine)?,clip(cosine)?))
    }
}

/// A number a jet is read in: a float, or an outward-rounded interval (`Iv`).
pub trait Num: Copy+Add<Output=Self>+Sub<Output=Self>+Mul<Output=Self>+Div<Output=Self>+Neg<Output=Self> {
    /// The number `x`.
    fn of(x: f64) -> Self;
    /// Its square root.
    fn sqrt(self) -> Self;
    /// Its sine and cosine.
    fn sin_cos(self) -> (Self,Self);
    /// The float it is, where it is one (an interval is none).
    fn real(self) -> Option<f64>;
    /// It as an interval (a float the interval holding it alone).
    fn iv(self) -> Iv;
    /// The number an interval stands for: the interval itself, or a float its middle (what a reading
    /// worked in intervals hands back to a float one).
    fn from_iv(x: Iv) -> Self;
}

impl Num for f64 {
    fn of(x: f64) -> f64 { x }
    fn sqrt(self) -> f64 { f64::sqrt(self) }
    fn sin_cos(self) -> (f64,f64) { use crate::util::fmath::Det; self.dsin_cos() }
    fn real(self) -> Option<f64> { Some(self) }
    fn iv(self) -> Iv { Iv::point(self) }
    fn from_iv(x: Iv) -> f64 { x.mid() }
}

/// An outward-rounded interval, or none where an operation left its domain (a division by an
/// interval holding nought, a root of one reaching below it): every reading through it then none.
#[derive(Clone,Copy,Debug,PartialEq)]
pub struct Iv(pub Option<Interval>);

impl Iv {
    /// The interval between `lo` and `hi`, either way round.
    pub fn new(lo: f64,hi: f64) -> Iv { Iv(Interval::new(lo.min(hi),lo.max(hi)).ok()) }
    /// Its ends, where it is one.
    pub fn bounds(self) -> Option<[f64;2]> { self.0.map(|i| i.bounds()) }
    /// The most magnitude it holds, where it is one.
    pub fn most(self) -> Option<f64> { self.bounds().map(|[a,b]| a.abs().max(b.abs())) }
    /// The interval holding `x` alone.
    pub fn point(x: f64) -> Iv { Iv(Interval::point(x).ok()) }
    /// No enclosure: what a reading that left its domain holds, read as every number.
    pub const ANY: Iv = Iv(None);
    /// Its least end, `−∞` where it holds every number.
    pub fn lo(self) -> f64 { self.0.map_or(f64::NEG_INFINITY,|i| i.bounds()[0]) }
    /// Its greatest end, `+∞` where it holds every number.
    pub fn hi(self) -> f64 { self.0.map_or(f64::INFINITY,|i| i.bounds()[1]) }
    /// The most magnitude it holds, `+∞` where it holds every number.
    pub fn mag(self) -> f64 { self.most().unwrap_or(f64::INFINITY) }
    /// Whether it may hold nought (every number may).
    pub fn has_zero(self) -> bool { self.lo() <= 0. && 0. <= self.hi() }
    /// Its middle, a reading in plain floating point (nought where it holds every number).
    pub fn mid(self) -> f64 { self.0.map_or(0.,|i| i.mid()) }
    /// The least interval holding both (every number where either is).
    pub fn hull(self,b: Iv) -> Iv { Iv(self.0.zip(b.0).and_then(|(a,b)| { let ([a0,a1],[b0,b1]) = (a.bounds(),b.bounds()); Interval::new(a0.min(b0),a1.max(b1)).ok() })) }
    /// What both hold, the tighter enclosure of one number two ways: either where the other holds
    /// every number, and `self` where rounding has put them apart (each encloses the number, so they
    /// cannot be apart but by a reading's rounding; `self` is kept, never an empty answer).
    pub fn meet(self,b: Iv) -> Iv {
        match (self.0,b.0) {
            (None,_) => b,
            (_,None) => self,
            (Some(x),Some(y)) => { let ([a0,a1],[b0,b1]) = (x.bounds(),y.bounds()); Iv(Interval::new(a0.max(b0),a1.min(b1)).ok().or(Some(x))) }
        }
    }
    /// Scaled by the float `k`, outward.
    pub fn times(self,k: f64) -> Iv { self*Iv::point(k) }
    /// The hull of `x`, every float of it.
    pub fn of_all(x: impl IntoIterator<Item=f64>) -> Iv {
        let (lo,hi) = x.into_iter().fold((f64::INFINITY,f64::NEG_INFINITY),|(l,h),v| (l.min(v),h.max(v)));
        if lo <= hi { Iv::new(lo,hi) } else { Iv::ANY }
    }
}

impl Add for Iv { type Output = Iv; fn add(self,b: Iv) -> Iv { Iv(self.0.zip(b.0).and_then(|(a,b)| a.add(b).ok())) } }
impl Sub for Iv { type Output = Iv; fn sub(self,b: Iv) -> Iv { Iv(self.0.zip(b.0).and_then(|(a,b)| a.sub(b).ok())) } }
impl Mul for Iv { type Output = Iv; fn mul(self,b: Iv) -> Iv { Iv(self.0.zip(b.0).and_then(|(a,b)| a.mul(b).ok())) } }
impl Div for Iv { type Output = Iv; fn div(self,b: Iv) -> Iv { Iv(self.0.zip(b.0).and_then(|(a,b)| a.div(b).ok())) } }
impl Neg for Iv { type Output = Iv; fn neg(self) -> Iv { Iv(self.0.map(|a| a.neg())) } }

impl Num for Iv {
    fn of(x: f64) -> Iv { Iv(Interval::point(x).ok()) }
    fn sqrt(self) -> Iv { Iv(self.0.and_then(|a| a.sqrt().ok())) }
    fn sin_cos(self) -> (Iv,Iv) {
        match self.0.and_then(|a| a.sin_cos_periodic().ok()) { Some((s,c)) => (Iv(Some(s)),Iv(Some(c))),None => (Iv(None),Iv(None)) }
    }
    fn real(self) -> Option<f64> { None }
    fn iv(self) -> Iv { self }
    fn from_iv(x: Iv) -> Iv { x }
}

/// A number and its first two derivatives along one parameter.
#[derive(Clone,Copy,Debug,PartialEq)]
pub struct Jet<T> { pub v: T,pub d: T,pub dd: T }

impl<T: Num> Jet<T> {
    /// The constant `x`.
    pub fn of(x: f64) -> Jet<T> { Jet {v:T::of(x),d:T::of(0.),dd:T::of(0.)} }
    /// The parameter itself, at `t`.
    pub fn at(t: T) -> Jet<T> { Jet {v:t,d:T::of(1.),dd:T::of(0.)} }
    /// Scaled by the constant `k`.
    pub fn times(self,k: f64) -> Jet<T> { let k = T::of(k); Jet {v:self.v*k,d:self.d*k,dd:self.dd*k} }
    /// Its square root: `s' = x' / 2s`, `s'' = (x'' − 2 s'²) / 2s`.
    pub fn sqrt(self) -> Jet<T> {
        let s = self.v.sqrt();
        let two = T::of(2.);
        let d = self.d/(two*s);
        Jet {v:s,d,dd:(self.dd-two*d*d)/(two*s)}
    }
    /// Its sine and cosine: `sin' = cos x'`, `sin'' = cos x'' − sin x'²`; `cos' = −sin x'`,
    /// `cos'' = −sin x'' − cos x'²`.
    pub fn sin_cos(self) -> (Jet<T>,Jet<T>) {
        let (s,c) = self.v.sin_cos();
        let d2 = self.d*self.d;
        (Jet {v:s,d:c*self.d,dd:c*self.dd-s*d2},Jet {v:c,d:-(s*self.d),dd:-(s*self.dd)-c*d2})
    }
}

impl Jet<Iv> {
    /// The most magnitude its first and second derivatives hold, unbounded where a reading left its
    /// domain: what a root's isolation bounds its function by (`util::isolate::roots`).
    pub fn most(self) -> [f64;2] { [self.d.most().unwrap_or(f64::INFINITY),self.dd.most().unwrap_or(f64::INFINITY)] }
}

impl<T: Num> Add for Jet<T> { type Output = Jet<T>; fn add(self,b: Jet<T>) -> Jet<T> { Jet {v:self.v+b.v,d:self.d+b.d,dd:self.dd+b.dd} } }
impl<T: Num> Sub for Jet<T> { type Output = Jet<T>; fn sub(self,b: Jet<T>) -> Jet<T> { Jet {v:self.v-b.v,d:self.d-b.d,dd:self.dd-b.dd} } }
impl<T: Num> Neg for Jet<T> { type Output = Jet<T>; fn neg(self) -> Jet<T> { Jet {v:-self.v,d:-self.d,dd:-self.dd} } }
impl<T: Num> Mul for Jet<T> {
    type Output = Jet<T>;
    fn mul(self,b: Jet<T>) -> Jet<T> {
        Jet {v:self.v*b.v,d:self.d*b.v+self.v*b.d,dd:self.dd*b.v+T::of(2.)*self.d*b.d+self.v*b.dd}
    }
}
impl<T: Num> Div for Jet<T> {
    type Output = Jet<T>;
    /// `q = a / b`: `q' = (a' − q b') / b`, `q'' = (a'' − 2 q' b' − q b'') / b`.
    fn div(self,b: Jet<T>) -> Jet<T> {
        let v = self.v/b.v;
        let d = (self.d-v*b.d)/b.v;
        Jet {v,d,dd:(self.dd-T::of(2.)*d*b.d-v*b.dd)/b.v}
    }
}
