//! The elementary functions, the same bits on every target. `f64::sin` and its kin are the
//! platform's: the system's libm natively, a port of musl's in a wasm build, and the two disagree in
//! the last place often enough that one drawing solves to different bits in the terminal and in the
//! browser, and an exported file differs between them (phase 5 of docs/rust-kernel-plan.md holds
//! the two byte-identical). These are the classic FreeBSD/musl algorithms (MIT-licensed, as this
//! repository is) written over IEEE arithmetic alone, which every target rounds alike: argument
//! reduction, a minimax polynomial, reconstruction. Each is within a unit or two in the last place
//! of the true value (`tests/fmath.rs` holds them to the platform's), and exact where it must be
//! (`atan2` on the axes, `log10` of a power of ten). `sqrt` and the basic operations are already
//! correctly rounded everywhere and stay the language's.
//!
//! The core calls them as methods (`x.dsin()`, `y.datan2(x)`) through `Det`.
use std::f64::consts::{FRAC_PI_2,FRAC_PI_4,PI};

fn hi(x: f64) -> u32 { (x.to_bits() >> 32) as u32 }
fn lo(x: f64) -> u32 { x.to_bits() as u32 }
fn with_hi(x: f64,h: u32) -> f64 { f64::from_bits(((h as u64) << 32) | (x.to_bits() & 0xffff_ffff)) }
fn zero_lo(x: f64) -> f64 { f64::from_bits(x.to_bits() & 0xffff_ffff_0000_0000) }

/// `x × 2ⁿ`, exactly where the result is a normal number.
pub fn scalbn(mut x: f64,mut n: i32) -> f64 {
    let p1023 = f64::from_bits(0x7fe0_0000_0000_0000);
    let pm1022 = f64::from_bits(0x0010_0000_0000_0000);
    if n > 1023 {
        x *= p1023; n -= 1023;
        if n > 1023 { x *= p1023; n -= 1023; if n > 1023 { n = 1023; } }
    } else if n < -1022 {
        // (scaled twice by 2^-1022·2^53 to keep the bits a subnormal result rounds from)
        let s = pm1022*f64::from_bits(0x4340_0000_0000_0000);
        x *= s; n += 1022-53;
        if n < -1022 { x *= s; n += 1022-53; if n < -1022 { n = -1022; } }
    }
    x*f64::from_bits(((0x3ff+n as i64) as u64) << 52)
}

// --- sin, cos, tan --------------------------------------------------------------------------

fn k_sin(x: f64,y: f64,iy: bool) -> f64 {
    const S1: f64 = -1.66666666666666324348e-01;
    const S2: f64 = 8.33333333332248946124e-03;
    const S3: f64 = -1.98412698298579493134e-04;
    const S4: f64 = 2.75573137070700676789e-06;
    const S5: f64 = -2.50507602534068634195e-08;
    const S6: f64 = 1.58969099521155010221e-10;
    let z = x*x;
    let w = z*z;
    let r = S2+z*(S3+z*S4)+z*w*(S5+z*S6);
    let v = z*x;
    if !iy { x+v*(S1+z*r) } else { x-((z*(0.5*y-v*r)-y)-v*S1) }
}

fn k_cos(x: f64,y: f64) -> f64 {
    const C1: f64 = 4.16666666666666019037e-02;
    const C2: f64 = -1.38888888888741095749e-03;
    const C3: f64 = 2.48015872894767294178e-05;
    const C4: f64 = -2.75573143513906633035e-07;
    const C5: f64 = 2.08757232129817482790e-09;
    const C6: f64 = -1.13596475577881948265e-11;
    let z = x*x;
    let w = z*z;
    let r = z*(C1+z*(C2+z*C3))+w*w*(C4+z*(C5+z*C6));
    let hz = 0.5*z;
    let w = 1.0-hz;
    w+(((1.0-w)-hz)+(z*r-x*y))
}

fn k_tan(mut x: f64,mut y: f64,odd: bool) -> f64 {
    const T: [f64;13] = [3.33333333333334091986e-01,1.33333333333201242699e-01,5.39682539762260521377e-02,
        2.18694882948595424599e-02,8.86323982359930005737e-03,3.59207910759131235356e-03,1.45620945432529025516e-03,
        5.88041240820264096874e-04,2.46463134818469906812e-04,7.81794442939557092300e-05,7.14072491382608190305e-05,
        -1.85586374855275456654e-05,2.59073051863633712884e-05];
    const PIO4: f64 = 7.85398163397448278999e-01;
    const PIO4LO: f64 = 3.06161699786838301793e-17;
    let hx = hi(x);
    let big = (hx & 0x7fff_ffff) >= 0x3fe5_9428;
    let sign = hx >> 31 != 0;
    if big {
        if sign { x = -x; y = -y; }
        x = (PIO4-x)+(PIO4LO-y);
        y = 0.0;
    }
    let z = x*x;
    let w = z*z;
    let r = T[1]+w*(T[3]+w*(T[5]+w*(T[7]+w*(T[9]+w*T[11]))));
    let v = z*(T[2]+w*(T[4]+w*(T[6]+w*(T[8]+w*(T[10]+w*T[12])))));
    let s = z*x;
    let r = y+z*(s*(r+v)+y)+s*T[0];
    let w = x+r;
    if big {
        let s = if odd { -1.0 } else { 1.0 };
        let v = s-2.0*(x+(r-w*w/(w+s)));
        return if sign { -v } else { v }
    }
    if !odd { return w }
    // −1/(x + r), its error kept to a unit: from the high halves and a correction
    let w0 = zero_lo(w);
    let v = r-(w0-x);
    let a = -1.0/w;
    let a0 = zero_lo(a);
    a0+a*(1.0+a0*w0+a0*v)
}

/// `x` less a whole number `n` of quarter turns: `(n, y0, y1)` with `x ≈ n·π/2 + y0 + y1`, the
/// quarter turn in three parts (Cody and Waite) so the remainder keeps its bits for every `n` this
/// core meets; past about 2³⁰ a remainder is not accurate, and no angle of a drawing is that large.
fn rem_pio2(x: f64) -> (i32,f64,f64) {
    const TOINT: f64 = 1.5/f64::EPSILON;
    const INVPIO2: f64 = 6.36619772367581382433e-01;
    const PIO2_1: f64 = 1.57079632673412561417e+00;
    const PIO2_1T: f64 = 6.07710050650619224932e-11;
    const PIO2_2: f64 = 6.07710050630396597660e-11;
    const PIO2_2T: f64 = 2.02226624879595063154e-21;
    const PIO2_3: f64 = 2.02226624871116645580e-21;
    const PIO2_3T: f64 = 8.47842766036889956997e-32;
    let f = x*INVPIO2+TOINT-TOINT;
    let n = f as i32;
    let ex = (hi(x) >> 20) & 0x7ff;
    let mut r = x-f*PIO2_1;
    let mut w = f*PIO2_1T;
    let mut y0 = r-w;
    let ey = |y: f64| (hi(y) >> 20) & 0x7ff;
    if ex as i32-ey(y0) as i32 > 16 {
        let t = r;
        w = f*PIO2_2;
        r = t-w;
        w = f*PIO2_2T-((t-r)-w);
        y0 = r-w;
        if ex as i32-ey(y0) as i32 > 49 {
            let t = r;
            w = f*PIO2_3;
            r = t-w;
            w = f*PIO2_3T-((t-r)-w);
            y0 = r-w;
        }
    }
    let y1 = (r-y0)-w;
    (n,y0,y1)
}

pub fn sin(x: f64) -> f64 {
    let ix = hi(x) & 0x7fff_ffff;
    if ix <= 0x3fe9_21fb {
        if ix < 0x3e50_0000 { return x }
        return k_sin(x,0.0,false)
    }
    if ix >= 0x7ff0_0000 { return x-x }
    let (n,y0,y1) = rem_pio2(x);
    match n & 3 { 0 => k_sin(y0,y1,true),1 => k_cos(y0,y1),2 => -k_sin(y0,y1,true),_ => -k_cos(y0,y1) }
}

pub fn cos(x: f64) -> f64 {
    let ix = hi(x) & 0x7fff_ffff;
    if ix <= 0x3fe9_21fb {
        if ix < 0x3e46_a09e { return 1.0 }
        return k_cos(x,0.0)
    }
    if ix >= 0x7ff0_0000 { return x-x }
    let (n,y0,y1) = rem_pio2(x);
    match n & 3 { 0 => k_cos(y0,y1),1 => -k_sin(y0,y1,true),2 => -k_cos(y0,y1),_ => k_sin(y0,y1,true) }
}

pub fn sin_cos(x: f64) -> (f64,f64) {
    let ix = hi(x) & 0x7fff_ffff;
    if ix <= 0x3fe9_21fb {
        if ix < 0x3e46_a09e { return (x,1.0) }
        return (k_sin(x,0.0,false),k_cos(x,0.0))
    }
    if ix >= 0x7ff0_0000 { let n = x-x; return (n,n) }
    let (n,y0,y1) = rem_pio2(x);
    let (s,c) = (k_sin(y0,y1,true),k_cos(y0,y1));
    match n & 3 { 0 => (s,c),1 => (c,-s),2 => (-s,-c),_ => (-c,s) }
}

pub fn tan(x: f64) -> f64 {
    let ix = hi(x) & 0x7fff_ffff;
    if ix <= 0x3fe9_21fb {
        if ix < 0x3e40_0000 { return x }
        return k_tan(x,0.0,false)
    }
    if ix >= 0x7ff0_0000 { return x-x }
    let (n,y0,y1) = rem_pio2(x);
    k_tan(y0,y1,n & 1 != 0)
}

// --- atan, atan2, asin, acos ----------------------------------------------------------------

pub fn atan(mut x: f64) -> f64 {
    const ATANHI: [f64;4] = [4.63647609000806093515e-01,7.85398163397448278999e-01,9.82793723247329054082e-01,1.57079632679489655800e+00];
    const ATANLO: [f64;4] = [2.26987774529616870924e-17,3.06161699786838301793e-17,1.39033110312309984516e-17,6.12323399573676603587e-17];
    const AT: [f64;11] = [3.33333333333329318027e-01,-1.99999999998764832476e-01,1.42857142725034663711e-01,
        -1.11111104054623557880e-01,9.09088713343650656196e-02,-7.69187620504482999495e-02,6.66107313738753120669e-02,
        -5.83357013379057348645e-02,4.97687799461593236017e-02,-3.65315727442169155270e-02,1.62858201153657823623e-02];
    let sign = hi(x) >> 31 != 0;
    let ix = hi(x) & 0x7fff_ffff;
    if ix >= 0x4410_0000 {
        if x.is_nan() { return x }
        let z = ATANHI[3]+f64::from_bits(0x3870_0000_0000_0000);
        return if sign { -z } else { z }
    }
    let id: i32;
    if ix < 0x3fdc_0000 {
        if ix < 0x3e40_0000 { return x }
        id = -1;
    } else {
        x = x.abs();
        if ix < 0x3ff3_0000 {
            if ix < 0x3fe6_0000 { id = 0; x = (2.0*x-1.0)/(2.0+x); } else { id = 1; x = (x-1.0)/(x+1.0); }
        } else if ix < 0x4003_8000 { id = 2; x = (x-1.5)/(1.0+1.5*x); } else { id = 3; x = -1.0/x; }
    }
    let z = x*x;
    let w = z*z;
    let s1 = z*(AT[0]+w*(AT[2]+w*(AT[4]+w*(AT[6]+w*(AT[8]+w*AT[10])))));
    let s2 = w*(AT[1]+w*(AT[3]+w*(AT[5]+w*(AT[7]+w*AT[9]))));
    if id < 0 { return x-x*(s1+s2) }
    let id = id as usize;
    let z = ATANHI[id]-(x*(s1+s2)-ATANLO[id]-x);
    if sign { -z } else { z }
}

pub fn atan2(y: f64,x: f64) -> f64 {
    const PI_LO: f64 = 1.2246467991473531772e-16;
    if x.is_nan() || y.is_nan() { return x+y }
    if x == 1.0 { return atan(y) }
    let m = ((hi(y) >> 31) | ((hi(x) >> 30) & 2)) as u8;
    let (ix,iy) = (hi(x) & 0x7fff_ffff,hi(y) & 0x7fff_ffff);
    if y == 0.0 { return match m { 0 | 1 => y,2 => PI,_ => -PI } }
    if x == 0.0 { return if m & 1 != 0 { -FRAC_PI_2 } else { FRAC_PI_2 } }
    if x.is_infinite() {
        return if y.is_infinite() {
            match m { 0 => FRAC_PI_4,1 => -FRAC_PI_4,2 => 3.0*FRAC_PI_4,_ => -3.0*FRAC_PI_4 }
        } else { match m { 0 => 0.0,1 => -0.0,2 => PI,_ => -PI } }
    }
    // |y/x| past 2⁶⁴: the axis
    if ix+(64 << 20) < iy || y.is_infinite() { return if m & 1 != 0 { -FRAC_PI_2 } else { FRAC_PI_2 } }
    let z = if m & 2 != 0 && iy+(64 << 20) < ix { 0.0 } else { atan((y/x).abs()) };
    match m { 0 => z,1 => -z,2 => PI-(z-PI_LO),_ => (z-PI_LO)-PI }
}

fn r_asin(z: f64) -> f64 {
    const PS0: f64 = 1.66666666666666657415e-01;
    const PS1: f64 = -3.25565818622400915405e-01;
    const PS2: f64 = 2.01212532134862925881e-01;
    const PS3: f64 = -4.00555345006794114027e-02;
    const PS4: f64 = 7.91534994289814532176e-04;
    const PS5: f64 = 3.47933107596021167570e-05;
    const QS1: f64 = -2.40339491173441421878e+00;
    const QS2: f64 = 2.02094576023350569471e+00;
    const QS3: f64 = -6.88283971605453293030e-01;
    const QS4: f64 = 7.70381505559019352791e-02;
    let p = z*(PS0+z*(PS1+z*(PS2+z*(PS3+z*(PS4+z*PS5)))));
    let q = 1.0+z*(QS1+z*(QS2+z*(QS3+z*QS4)));
    p/q
}

const PIO2_HI: f64 = 1.57079632679489655800e+00;
const PIO2_LO: f64 = 6.12323399573676603587e-17;

pub fn asin(x: f64) -> f64 {
    let hx = hi(x);
    let ix = hx & 0x7fff_ffff;
    if ix >= 0x3ff0_0000 {
        if (ix-0x3ff0_0000) | lo(x) == 0 { return x*PIO2_HI+f64::from_bits(0x3870_0000_0000_0000) }
        return f64::NAN
    }
    if ix < 0x3fe0_0000 {
        if ix < 0x3e50_0000 && ix >= 0x0010_0000 { return x }
        return x+x*r_asin(x*x)
    }
    let z = (1.0-x.abs())*0.5;
    let s = z.sqrt();
    let r = r_asin(z);
    let x = if ix >= 0x3fef_3333 { PIO2_HI-(2.0*(s+s*r)-PIO2_LO) } else {
        let f = zero_lo(s);
        let c = (z-f*f)/(s+f);
        0.5*PIO2_HI-(2.0*s*r-(PIO2_LO-2.0*c)-(0.5*PIO2_HI-2.0*f))
    };
    if hx >> 31 != 0 { -x } else { x }
}

pub fn acos(x: f64) -> f64 {
    let hx = hi(x);
    let ix = hx & 0x7fff_ffff;
    if ix >= 0x3ff0_0000 {
        if (ix-0x3ff0_0000) | lo(x) == 0 { return if hx >> 31 != 0 { 2.0*PIO2_HI+f64::from_bits(0x3870_0000_0000_0000) } else { 0.0 } }
        return f64::NAN
    }
    if ix < 0x3fe0_0000 {
        if ix <= 0x3c60_0000 { return PIO2_HI+f64::from_bits(0x3870_0000_0000_0000) }
        return PIO2_HI-(x-(PIO2_LO-x*r_asin(x*x)))
    }
    if hx >> 31 != 0 {
        let z = (1.0+x)*0.5;
        let s = z.sqrt();
        let w = r_asin(z)*s-PIO2_LO;
        return 2.0*(PIO2_HI-(s+w))
    }
    let z = (1.0-x)*0.5;
    let s = z.sqrt();
    let df = zero_lo(s);
    let c = (z-df*df)/(s+df);
    let w = r_asin(z)*s+c;
    2.0*(df+w)
}

// --- exp, log, log2, log10, pow -------------------------------------------------------------

pub fn exp(mut x: f64) -> f64 {
    const HALF: [f64;2] = [0.5,-0.5];
    const LN2HI: f64 = 6.93147180369123816490e-01;
    const LN2LO: f64 = 1.90821492927058770002e-10;
    const INVLN2: f64 = 1.44269504088896338700e+00;
    const P1: f64 = 1.66666666666666019037e-01;
    const P2: f64 = -2.77777777770155933842e-03;
    const P3: f64 = 6.61375632143793436117e-05;
    const P4: f64 = -1.65339022054652515390e-06;
    const P5: f64 = 4.13813679705723846039e-08;
    let hx = hi(x);
    let sign = (hx >> 31) as usize;
    let hx = hx & 0x7fff_ffff;
    if hx >= 0x4086_232b {
        if x.is_nan() { return x }
        if x > 709.782712893383973096 { return f64::INFINITY }
        if x < -745.13321910194110842 { return 0.0 }
    }
    let (k,hi_,lo_): (i32,f64,f64);
    if hx > 0x3fd6_2e42 {
        k = if hx >= 0x3ff0_a2b2 { (INVLN2*x+HALF[sign]) as i32 } else { 1-sign as i32-sign as i32 };
        hi_ = x-k as f64*LN2HI;
        lo_ = k as f64*LN2LO;
        x = hi_-lo_;
    } else if hx > 0x3e30_0000 {
        k = 0; hi_ = x; lo_ = 0.0;
    } else { return 1.0+x }
    let xx = x*x;
    let c = x-xx*(P1+xx*(P2+xx*(P3+xx*(P4+xx*P5))));
    let y = 1.0+(x*c/(2.0-c)-lo_+hi_);
    if k == 0 { y } else { scalbn(y,k) }
}

/// The reduction every logarithm shares: `x = 2ᵏ (1 + f)`, √½ ≤ 1 + f < √2, and the series
/// pieces about `f` (`hfsq = f²/2`, `s = f/(2 + f)`, `r` the polynomial in `s²`). `None` for zero,
/// negatives, infinities and NaN, which each logarithm answers itself.
fn log_parts(x: f64) -> Option<(i32,f64,f64,f64,f64)> {
    const LG1: f64 = 6.666666666666735130e-01;
    const LG2: f64 = 3.999999999940941908e-01;
    const LG3: f64 = 2.857142874366239149e-01;
    const LG4: f64 = 2.222219843214978396e-01;
    const LG5: f64 = 1.818357216161805012e-01;
    const LG6: f64 = 1.531383769920937332e-01;
    const LG7: f64 = 1.479819860511658591e-01;
    let mut x = x;
    let mut hx = hi(x);
    let mut k = 0i32;
    if hx < 0x0010_0000 || hx >> 31 != 0 {
        if x == 0.0 || hx >> 31 != 0 { return None }
        k -= 54;
        x *= f64::from_bits(0x4350_0000_0000_0000);
        hx = hi(x);
    } else if hx >= 0x7ff0_0000 { return None }
    hx = hx.wrapping_add(0x3ff0_0000-0x3fe6_a09e);
    k += (hx >> 20) as i32-0x3ff;
    hx = (hx & 0x000f_ffff)+0x3fe6_a09e;
    let x = with_hi(x,hx);
    let f = x-1.0;
    let hfsq = 0.5*f*f;
    let s = f/(2.0+f);
    let z = s*s;
    let w = z*z;
    let t1 = w*(LG2+w*(LG4+w*LG6));
    let t2 = z*(LG1+w*(LG3+w*(LG5+w*LG7)));
    Some((k,f,hfsq,s,t2+t1))
}

/// A logarithm's answer where `log_parts` has none.
fn log_special(x: f64) -> f64 {
    if x == 0.0 { f64::NEG_INFINITY } else if x < 0.0 || x.is_nan() { f64::NAN } else { x }
}

pub fn ln(x: f64) -> f64 {
    const LN2_HI: f64 = 6.93147180369123816490e-01;
    const LN2_LO: f64 = 1.90821492927058770002e-10;
    if x == 1.0 { return 0.0 }
    let Some((k,f,hfsq,s,r)) = log_parts(x) else { return log_special(x) };
    let dk = k as f64;
    s*(hfsq+r)+dk*LN2_LO-hfsq+f+dk*LN2_HI
}

pub fn log10(x: f64) -> f64 {
    const IVLN10HI: f64 = 4.34294481878168880939e-01;
    const IVLN10LO: f64 = 2.50829467116452752298e-11;
    const LOG10_2HI: f64 = 3.01029995663611771306e-01;
    const LOG10_2LO: f64 = 3.69423907715893078616e-13;
    if x == 1.0 { return 0.0 }
    let Some((k,f,hfsq,s,r)) = log_parts(x) else { return log_special(x) };
    let hi_ = zero_lo(f-hfsq);
    let lo_ = f-hi_-hfsq+s*(hfsq+r);
    let mut val_hi = hi_*IVLN10HI;
    let dk = k as f64;
    let y = dk*LOG10_2HI;
    let mut val_lo = dk*LOG10_2LO+(lo_+hi_)*IVLN10LO+lo_*IVLN10HI;
    let w = y+val_hi;
    val_lo += (y-w)+val_hi;
    val_hi = w;
    val_lo+val_hi
}

pub fn log2(x: f64) -> f64 {
    const IVLN2HI: f64 = 1.44269504072144627571e+00;
    const IVLN2LO: f64 = 1.67517131648865118353e-10;
    if x == 1.0 { return 0.0 }
    let Some((k,f,hfsq,s,r)) = log_parts(x) else { return log_special(x) };
    let hi_ = zero_lo(f-hfsq);
    let lo_ = f-hi_-hfsq+s*(hfsq+r);
    let mut val_hi = hi_*IVLN2HI;
    let mut val_lo = (lo_+hi_)*IVLN2LO+lo_*IVLN2HI;
    let y = k as f64;
    let w = y+val_hi;
    val_lo += (y-w)+val_hi;
    val_hi = w;
    val_lo+val_hi
}

/// `a·b` as an unevaluated sum `p + e` (Dekker's split; no fused multiply needed).
fn two_product(a: f64,b: f64) -> (f64,f64) {
    const SPLIT: f64 = 134217729.0;
    let p = a*b;
    let (ah,bh) = (a*SPLIT-(a*SPLIT-a),b*SPLIT-(b*SPLIT-b));
    let (al,bl) = (a-ah,b-bh);
    (p,((ah*bh-p)+ah*bl+al*bh)+al*bl)
}

/// `xⁿ` by repeated squaring, the same products on every target.
pub fn powi(x: f64,n: i32) -> f64 {
    let mut b = n.unsigned_abs();
    let (mut a,mut r) = (x,1.0);
    while b > 0 {
        if b & 1 != 0 { r *= a; }
        b >>= 1;
        if b > 0 { a *= a; }
    }
    if n < 0 { 1.0/r } else { r }
}

/// `xʸ`: a whole power of up to 64 by squaring; otherwise `exp(y ln x)`, the product carried to
/// twice the precision and `exp` taken of its high part and corrected by its low. The logarithm is
/// good to a unit in its last place, which `y` multiplies: within a few units of the last place
/// where `y ln x` is small, a dozen or so at thirty (a fraction a drawing's `^` rarely asks).
pub fn pow(x: f64,y: f64) -> f64 {
    if y == 0.0 || x == 1.0 { return 1.0 }
    if x.is_nan() || y.is_nan() { return x+y }
    let whole = y == y.trunc();
    if whole && y.abs() <= 64.0 { return powi(x,y as i32) }
    if x == 0.0 { return if y > 0.0 { if whole && y.rem_euclid(2.0) == 1.0 { x } else { 0.0 } } else { f64::INFINITY } }
    if x < 0.0 {
        if !whole { return f64::NAN }
        let odd = y.rem_euclid(2.0) == 1.0;
        let p = pow(-x,y);
        return if odd { -p } else { p }
    }
    if x.is_infinite() { return if y > 0.0 { f64::INFINITY } else { 0.0 } }
    // ln x as hi + lo
    const LN2_HI: f64 = 6.93147180369123816490e-01;
    const LN2_LO: f64 = 1.90821492927058770002e-10;
    let Some((k,f,hfsq,s,r)) = log_parts(x) else { return f64::NAN };
    let dk = k as f64;
    let tail = s*(hfsq+r)+dk*LN2_LO-hfsq;
    let (a,b) = (dk*LN2_HI,f);
    let s1 = a+b;
    let e1 = if a.abs() >= b.abs() { (a-s1)+b } else { (b-s1)+a };
    let lh = s1+(tail+e1);
    let ll = ((s1-lh)+tail)+e1;
    let (p,e) = two_product(y,lh);
    let e = e+y*ll;
    let z = p+e;
    let zl = (p-z)+e;
    let ez = exp(z);
    ez+ez*zl
}

/// √(x² + y²) without overflow or underflow, each square carried to twice the precision.
pub fn hypot(x: f64,y: f64) -> f64 {
    let (mut x,mut y) = (x.abs(),y.abs());
    // (ordered by their bits, so a NaN beside an infinity is the smaller and the infinity is said)
    if x.to_bits() < y.to_bits() { std::mem::swap(&mut x,&mut y); }
    let (ex,ey) = ((x.to_bits() >> 52) as i32,(y.to_bits() >> 52) as i32);
    if ey == 0x7ff { return y }
    if ex == 0x7ff || y == 0.0 { return x }
    if ex-ey > 64 { return x+y }
    let mut z = 1.0;
    if ex > 0x3ff+510 {
        z = f64::from_bits(0x6bb0_0000_0000_0000);
        x *= f64::from_bits(0x1430_0000_0000_0000);
        y *= f64::from_bits(0x1430_0000_0000_0000);
    } else if ey < 0x3ff-450 {
        z = f64::from_bits(0x1430_0000_0000_0000);
        x *= f64::from_bits(0x6bb0_0000_0000_0000);
        y *= f64::from_bits(0x6bb0_0000_0000_0000);
    }
    let (hx,lx) = two_product(x,x);
    let (hy,ly) = two_product(y,y);
    z*(ly+lx+hy+hx).sqrt()
}

/// The elementary functions as methods, called as `std`'s are with a `d` before the name.
pub trait Det: Copy {
    fn dsin(self) -> Self;
    fn dcos(self) -> Self;
    fn dtan(self) -> Self;
    fn dsin_cos(self) -> (Self,Self);
    fn dasin(self) -> Self;
    fn dacos(self) -> Self;
    fn datan(self) -> Self;
    fn datan2(self,x: Self) -> Self;
    fn dexp(self) -> Self;
    fn dln(self) -> Self;
    fn dlog10(self) -> Self;
    fn dlog2(self) -> Self;
    fn dpowf(self,y: Self) -> Self;
    fn dpowi(self,n: i32) -> Self;
    fn dhypot(self,y: Self) -> Self;
}

impl Det for f64 {
    fn dsin(self) -> f64 { sin(self) }
    fn dcos(self) -> f64 { cos(self) }
    fn dtan(self) -> f64 { tan(self) }
    fn dsin_cos(self) -> (f64,f64) { sin_cos(self) }
    fn dasin(self) -> f64 { asin(self) }
    fn dacos(self) -> f64 { acos(self) }
    fn datan(self) -> f64 { atan(self) }
    fn datan2(self,x: f64) -> f64 { atan2(self,x) }
    fn dexp(self) -> f64 { exp(self) }
    fn dln(self) -> f64 { ln(self) }
    fn dlog10(self) -> f64 { log10(self) }
    fn dlog2(self) -> f64 { log2(self) }
    fn dpowf(self,y: f64) -> f64 { pow(self,y) }
    fn dpowi(self,n: i32) -> f64 { powi(self,n) }
    fn dhypot(self,y: f64) -> f64 { hypot(self,y) }
}

impl Det for f32 {
    fn dsin(self) -> f32 { sin(self as f64) as f32 }
    fn dcos(self) -> f32 { cos(self as f64) as f32 }
    fn dtan(self) -> f32 { tan(self as f64) as f32 }
    fn dsin_cos(self) -> (f32,f32) { let (s,c) = sin_cos(self as f64); (s as f32,c as f32) }
    fn dasin(self) -> f32 { asin(self as f64) as f32 }
    fn dacos(self) -> f32 { acos(self as f64) as f32 }
    fn datan(self) -> f32 { atan(self as f64) as f32 }
    fn datan2(self,x: f32) -> f32 { atan2(self as f64,x as f64) as f32 }
    fn dexp(self) -> f32 { exp(self as f64) as f32 }
    fn dln(self) -> f32 { ln(self as f64) as f32 }
    fn dlog10(self) -> f32 { log10(self as f64) as f32 }
    fn dlog2(self) -> f32 { log2(self as f64) as f32 }
    fn dpowf(self,y: f32) -> f32 { pow(self as f64,y as f64) as f32 }
    fn dpowi(self,n: i32) -> f32 { powi(self as f64,n) as f32 }
    fn dhypot(self,y: f32) -> f32 { hypot(self as f64,y as f64) as f32 }
}
