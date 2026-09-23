//! Exact orientation and power tests, filtered: the determinant is evaluated in floating point
//! with a bound on its rounding error proportional to its permanent (the same expression over
//! absolute values), and only a value inside that bound is re-evaluated exactly in expansions.
//! The orientation bound is Shewchuk's `o3derrboundA`; the power test's is his insphere bound
//! doubled for the weight difference in each lifted coordinate.
use super::expansion::Expansion;

type P = [f64;3];

const EPS: f64 = f64::EPSILON*0.5; // 2^-53
const ORIENT_BOUND: f64 = (7.+56.*EPS)*EPS;
const POWER_BOUND: f64 = (32.+512.*EPS)*EPS;

/// Exact calls made so far, for a caller measuring how often the filter fails.
pub static EXACT_CALLS: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);

fn sign(v: f64) -> i8 { if v > 0. { 1 } else if v < 0. { -1 } else { 0 } }

/// The sign of `det[b − a; c − a; d − a]`: positive when `d` lies on the side of the plane `abc`
/// that `(b − a) × (c − a)` points to.
pub fn orient(a: P,b: P,c: P,d: P) -> i8 {
    let [bx,by,bz] = [b[0]-a[0],b[1]-a[1],b[2]-a[2]];
    let [cx,cy,cz] = [c[0]-a[0],c[1]-a[1],c[2]-a[2]];
    let [dx,dy,dz] = [d[0]-a[0],d[1]-a[1],d[2]-a[2]];
    let (m0,m1,m2) = (cy*dz-cz*dy,cz*dx-cx*dz,cx*dy-cy*dx);
    let det = bx*m0+by*m1+bz*m2;
    let permanent = bx.abs()*((cy*dz).abs()+(cz*dy).abs())+by.abs()*((cz*dx).abs()+(cx*dz).abs())
        +bz.abs()*((cx*dy).abs()+(cy*dx).abs());
    let bound = ORIENT_BOUND*permanent;
    if det > bound || -det > bound { return sign(det); }
    orient_exact(a,b,c,d)
}

pub fn orient_exact(a: P,b: P,c: P,d: P) -> i8 {
    EXACT_CALLS.fetch_add(1,std::sync::atomic::Ordering::Relaxed);
    let row = |p: P| [0,1,2].map(|k| Expansion::diff(p[k],a[k]));
    det3(&row(b),&row(c),&row(d)).sign()
}

fn det3(r0: &[Expansion;3],r1: &[Expansion;3],r2: &[Expansion;3]) -> Expansion {
    let m0 = r1[1].mul(&r2[2]).sub(&r1[2].mul(&r2[1]));
    let m1 = r1[2].mul(&r2[0]).sub(&r1[0].mul(&r2[2]));
    let m2 = r1[0].mul(&r2[1]).sub(&r1[1].mul(&r2[0]));
    r0[0].mul(&m0).add(&r0[1].mul(&m1)).add(&r0[2].mul(&m2))
}

/// A point with a weight: its power at `x` is `|x − p|² − w`.
#[derive(Clone,Copy,Debug,PartialEq)]
pub struct Weighted { pub p: P,pub w: f64 }

/// Whether `e` is in conflict with the positively oriented tetrahedron `abcd`: +1 when its power
/// with respect to the tetrahedron's orthosphere is negative (inside, for zero weights), 0 on it,
/// −1 outside. The sign of the lifted determinant with rows `(p − e, |p − e|² − (w − w_e))`, which
/// is negative for a point inside.
pub fn power(a: Weighted,b: Weighted,c: Weighted,d: Weighted,e: Weighted) -> i8 {
    let rows = [a,b,c,d].map(|q| {
        let x = [q.p[0]-e.p[0],q.p[1]-e.p[1],q.p[2]-e.p[2]];
        let dw = q.w-e.w;
        (x,x[0]*x[0]+x[1]*x[1]+x[2]*x[2]-dw,x[0]*x[0]+x[1]*x[1]+x[2]*x[2]+dw.abs())
    });
    const OTHERS: [[usize;3];4] = [[1,2,3],[0,2,3],[0,1,3],[0,1,2]];
    let minor = |i: usize| -> (f64,f64) {
        let r = OTHERS[i].map(|j| rows[j].0);
        let (m0,m1,m2) = (r[1][1]*r[2][2]-r[1][2]*r[2][1],r[1][2]*r[2][0]-r[1][0]*r[2][2],r[1][0]*r[2][1]-r[1][1]*r[2][0]);
        let det = r[0][0]*m0+r[0][1]*m1+r[0][2]*m2;
        let permanent = r[0][0].abs()*((r[1][1]*r[2][2]).abs()+(r[1][2]*r[2][1]).abs())
            +r[0][1].abs()*((r[1][2]*r[2][0]).abs()+(r[1][0]*r[2][2]).abs())
            +r[0][2].abs()*((r[1][0]*r[2][1]).abs()+(r[1][1]*r[2][0]).abs());
        (det,permanent)
    };
    let (mut det,mut permanent) = (0.,0.);
    for i in 0..4 {
        let (m,pm) = minor(i);
        let cofactor = if i % 2 == 0 { -1. } else { 1. };
        det += cofactor*rows[i].1*m;
        permanent += rows[i].2*pm;
    }
    let bound = POWER_BOUND*permanent;
    if det > bound || -det > bound { return -sign(det); }
    power_exact(a,b,c,d,e)
}

pub fn power_exact(a: Weighted,b: Weighted,c: Weighted,d: Weighted,e: Weighted) -> i8 {
    EXACT_CALLS.fetch_add(1,std::sync::atomic::Ordering::Relaxed);
    let rows: Vec<([Expansion;3],Expansion)> = [a,b,c,d].iter().map(|q| {
        let x = [0,1,2].map(|k| Expansion::diff(q.p[k],e.p[k]));
        let lift = x[0].mul(&x[0]).add(&x[1].mul(&x[1])).add(&x[2].mul(&x[2])).sub(&Expansion::diff(q.w,e.w));
        (x,lift)
    }).collect();
    let mut det = Expansion::default();
    for i in 0..4 {
        let r: Vec<&[Expansion;3]> = (0..4).filter(|&j| j != i).map(|j| &rows[j].0).collect();
        let term = rows[i].1.mul(&det3(r[0],r[1],r[2]));
        det = if i % 2 == 0 { det.sub(&term) } else { det.add(&term) };
    }
    -det.sign()
}
