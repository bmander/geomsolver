//! Finite closed intervals with outward rounding. Arithmetic encloses real
//! operations on binary64 endpoints; invalid domains and overflow fail closed.
//! Trigonometric bounds use Taylor polynomials and explicit remainder bounds,
//! not the platform libm's unspecified transcendental accuracy.

pub mod minimum;

#[derive(Clone,Copy,Debug,PartialEq,Eq)]
pub enum Error { InvalidBounds, Overflow, DivisionByZero, OutsideDomain }

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

    /// Enclose sin and cos for the entire input interval in [-8,8] radians.
    /// No argument reduction or approximate pi enters the guarantee. Larger
    /// inputs are refused; callers must supply a separately justified reduction.
    pub fn sin_cos(self) -> Result<(Self,Self),Error> {
        let radius = self.lo.abs().max(self.hi.abs());
        if radius > 8. { return Err(Error::OutsideDomain); }
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
