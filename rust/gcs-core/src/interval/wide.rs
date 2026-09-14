//! Bounded fixed-point intervals for arrangement predicates. 80 fractional bits
//! distinguish source endpoints that ordinary binary64 intervals cannot order.
//! Integer arithmetic supplies directed rounding; overflow remains a refusal.
use super::{Error,Interval};
const SHIFT: u32 = 80;
const ONE: i128 = 1_i128 << SHIFT;
#[derive(Clone,Copy,Debug,PartialEq,Eq)]
pub(crate) struct Wide { pub lo: i128, pub hi: i128 }
type Result<T> = std::result::Result<T,Error>;
fn product(a: u128,b: u128) -> [u64;4] {
    let a = [a as u64,(a>>64) as u64]; let b = [b as u64,(b>>64) as u64];
    let mut out = [0u64;4];
    for i in 0..2 {
        let mut carry = 0u128;
        for j in 0..2 {
            let t = a[i] as u128*b[j] as u128+out[i+j] as u128+carry;
            out[i+j] = t as u64; carry = t>>64;
        }
        out[i+2] = carry as u64;
    }
    out
}
fn rounded(q: u128,remainder: bool,negative: bool) -> Result<(i128,i128)> {
    if q >= i128::MAX as u128 { return Err(Error::Overflow); }
    let (lo,hi) = (q as i128,q as i128+remainder as i128);
    Ok(if negative { (-hi,-lo) } else { (lo,hi) })
}
fn mul(a: i128,b: i128) -> Result<(i128,i128)> {
    let p = product(a.unsigned_abs(),b.unsigned_abs());
    if p[3]>>16 != 0 { return Err(Error::Overflow); }
    let lo = (p[1]>>16)|(p[2]<<48); let hi = (p[2]>>16)|(p[3]<<48);
    rounded(lo as u128|((hi as u128)<<64),p[0] != 0 || (p[1]&65535) != 0,(a < 0) != (b < 0))
}
fn div(a: i128,b: i128) -> Result<(i128,i128)> {
    if b == 0 { return Err(Error::DivisionByZero); }
    let a0 = a.unsigned_abs(); let b0 = b.unsigned_abs();
    let n = [(a0<<16) as u64,(a0>>48) as u64,(a0>>112) as u64];
    // a*2^80 has a zero low limb, followed by these three limbs.
    let n = [0,n[0],n[1],n[2]];
    let (mut rem,mut q) = (0u128,0u128);
    for bit in (0..256).rev() {
        rem = (rem<<1)|((n[bit/64]>>(bit%64))&1) as u128;
        if rem >= b0 {
            rem -= b0;
            if bit >= 127 { return Err(Error::Overflow); }
            q |= 1u128<<bit;
        }
    }
    rounded(q,rem != 0,(a < 0) != (b < 0))
}
impl Wide {
    pub const ZERO: Self = Self {lo:0,hi:0};
    pub const ONE: Self = Self {lo:ONE,hi:ONE};
    pub fn point(x: f64) -> Result<Self> {
        if !x.is_finite() { return Err(Error::InvalidBounds); }
        let bits = x.to_bits(); let exponent = ((bits>>52)&2047) as i32;
        let mantissa = (bits&((1u64<<52)-1))|if exponent == 0 { 0 } else { 1u64<<52 };
        let shift = SHIFT as i32+if exponent == 0 { -1074 } else { exponent-1023-52 };
        let (q,rem) = if shift >= 0 {
            if shift >= 127 || mantissa as u128 > (i128::MAX as u128)>>shift { return Err(Error::Overflow); }
            ((mantissa as u128)<<shift,false)
        } else if -shift >= 64 { (0,mantissa != 0) }
        else { ((mantissa>>-shift) as u128,(mantissa&((1u64<<-shift)-1)) != 0) };
        let (lo,hi) = rounded(q,rem,bits>>63 != 0)?;
        Ok(Self {lo,hi})
    }
    pub fn add(self,b: Self) -> Result<Self> {
        let lo = self.lo.checked_add(b.lo).ok_or(Error::Overflow)?;
        let hi = self.hi.checked_add(b.hi).ok_or(Error::Overflow)?;
        // All values remain negatable, including after cancellation/subtraction.
        if lo == i128::MIN { return Err(Error::Overflow); }
        Ok(Self {lo,hi})
    }
    pub fn neg(self) -> Self { Self {lo:-self.hi,hi:-self.lo} }
    pub fn sub(self,b: Self) -> Result<Self> { self.add(b.neg()) }
    pub fn mul(self,b: Self) -> Result<Self> {
        let p = [mul(self.lo,b.lo)?,mul(self.lo,b.hi)?,mul(self.hi,b.lo)?,mul(self.hi,b.hi)?];
        Ok(Self {lo:p.iter().map(|p| p.0).min().unwrap(),hi:p.iter().map(|p| p.1).max().unwrap()})
    }
    pub fn div(self,b: Self) -> Result<Self> {
        if b.lo <= 0 && b.hi >= 0 { return Err(Error::DivisionByZero); }
        let p = [div(self.lo,b.lo)?,div(self.lo,b.hi)?,div(self.hi,b.lo)?,div(self.hi,b.hi)?];
        Ok(Self {lo:p.iter().map(|p| p.0).min().unwrap(),hi:p.iter().map(|p| p.1).max().unwrap()})
    }
    pub fn sin_cos(self) -> Result<(Self,Self)> {
        if self == Self::ZERO { return Ok((Self::ZERO,Self::ONE)); }
        if self.lo < -8*ONE || self.hi > 8*ONE { return Err(Error::OutsideDomain); }
        let x2 = self.mul(self)?;
        let (mut s,mut c,mut st,mut ct) = (self,Self::ONE,self,Self::ONE);
        for k in 1..=32 {
            st = st.mul(x2)?.neg().div(Self::point(((2*k)*(2*k+1)) as f64)?)?;
            ct = ct.mul(x2)?.neg().div(Self::point(((2*k-1)*(2*k)) as f64)?)?;
            s = s.add(st)?; c = c.add(ct)?;
        }
        // Taylor degrees 65/64 on [-8,8]. Both remainders are less than
        // 8^65/65! < 2^-80; add one fixed-point unit on either side.
        let error = Self {lo:-1,hi:1};
        Ok((s.add(error)?,c.add(error)?))
    }
    pub fn mid(self) -> f64 { (self.lo/2+self.hi/2) as f64/ONE as f64 }
    pub fn interval(self) -> Result<Interval> {
        // Converting either i128 endpoint to f64 may round inward by one ulp.
        let a = self.lo as f64/ONE as f64; let b = self.hi as f64/ONE as f64;
        Interval::outward(a,b)
    }
    pub fn positive(self) -> bool { self.lo > 0 }
    pub fn negative(self) -> bool { self.hi < 0 }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn integer_rounding_preserves_small_residuals_and_signs() {
        assert_eq!(product(u128::MAX,u128::MAX),[1,0,u64::MAX-1,u64::MAX]);
        assert_eq!(mul(ONE+1,ONE+1).unwrap(),(ONE+2,ONE+3));
        assert_eq!(mul(-ONE-1,ONE+1).unwrap(),(-ONE-3,-ONE-2));
        assert_eq!(div(ONE,3*ONE).unwrap(),(ONE/3,ONE/3+1));
        assert_eq!(div(-ONE,3*ONE).unwrap(),(-ONE/3-1,-ONE/3));
        for a in [-1000000.,-3.,-0.125,0.,0.25,7.,1000000.] {
            for b in [-1000000.,-2.,0.125,4.,1000000.] {
                let (a0,b0) = (Wide::point(a).unwrap(),Wide::point(b).unwrap());
                assert_eq!(a0.mul(b0).unwrap(),Wide::point(a*b).unwrap());
                let q = a0.div(b0).unwrap();
                assert!(q.interval().unwrap().contains(a/b));
                assert!(q.hi-q.lo <= 1);
            }
        }
        let tiny = Wide::point(f64::from_bits(1)).unwrap();
        assert_eq!(tiny,Wide {lo:0,hi:1});
        assert_eq!(Wide::point(-f64::from_bits(1)).unwrap(),tiny.neg());
        assert_eq!(Wide::point(1e100),Err(Error::Overflow));
        assert_eq!(Wide::point(f64::NAN),Err(Error::InvalidBounds));
        assert_eq!(Wide::ONE.div(Wide {lo:-1,hi:1}),Err(Error::DivisionByZero));
        assert_eq!(Wide {lo:-i128::MAX,hi:0}.add(Wide {lo:-1,hi:0}),Err(Error::Overflow));
        assert_eq!(Wide::point(1e12).unwrap().mul(Wide::point(1e12).unwrap()),Err(Error::Overflow));
    }
    #[test]
    fn trigonometric_bounds_enclose_independent_rational_series() {
        // Integer floors of the degree-120/121 rational Taylor series times 2^80.
        // At |x| <= 8 the omitted remainder is < 2^-290, so [floor,floor+1]
        // encloses each exact transcendental value independently of this code.
        let references = [
            (1.,1017275999990815458903785,653185407961314874212934),
            (8.,1196060729191664242897240,-175898747626034361714976),
            (-8.,-1196060729191664242897241,-175898747626034361714976),
        ];
        for (x,s,c) in references {
            let (a,b) = Wide::point(x).unwrap().sin_cos().unwrap();
            for (v,floor) in [(a,s),(b,c)] {
                assert!(v.lo <= floor && v.hi >= floor+1,"{x}: {v:?} vs {floor}");
                assert!(v.hi-v.lo < 10000000);
            }
        }
        assert_eq!(Wide::ZERO.sin_cos().unwrap(),(Wide::ZERO,Wide::ONE));
        assert_eq!(Wide::point(8.01).unwrap().sin_cos(),Err(Error::OutsideDomain));
        let tau = Wide::point(std::f64::consts::TAU).unwrap();
        let a = tau.mul(Wide::point(0.01).unwrap()).unwrap().sin_cos().unwrap().0;
        let b = tau.mul(Wide::point(0.99).unwrap()).unwrap().sin_cos().unwrap().0.neg();
        assert!(a.hi < b.lo,"periodic endpoints are distinct: {a:?} {b:?}");
        let ordinary = |t| Interval::point(std::f64::consts::TAU).unwrap()
            .mul(Interval::point(t).unwrap()).unwrap().sin_cos().unwrap().0;
        assert!(ordinary(0.01).bounds()[1] >= ordinary(0.99).neg().bounds()[0],
            "ordinary binary64 Taylor intervals cannot order this pair");
    }
}
