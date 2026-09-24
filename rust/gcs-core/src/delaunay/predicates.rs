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

/// The sign of `det[b − a; c − a; d − a]`: positive when `d` lies on the side of the plane
/// `abc` that `(b − a) × (c − a)` points to.
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

/// Whether `e` is in conflict with the positively oriented tetrahedron `abcd`: +1 when its
/// power with respect to the tetrahedron's orthosphere is negative (inside, for zero weights),
/// 0 on it, −1 outside. The sign of the lifted determinant with rows
/// `(p − e, |p − e|² − (w − w_e))`, which is negative for a point inside.
pub fn power(a: Weighted,b: Weighted,c: Weighted,d: Weighted,e: Weighted) -> i8 {
    // Shewchuk's insphere layout: six 2×2 minors of the x, y columns shared by the four 3×3
    // cofactors of the lifted column.
    let (ax,ay,az) = (a.p[0]-e.p[0],a.p[1]-e.p[1],a.p[2]-e.p[2]);
    let (bx,by,bz) = (b.p[0]-e.p[0],b.p[1]-e.p[1],b.p[2]-e.p[2]);
    let (cx,cy,cz) = (c.p[0]-e.p[0],c.p[1]-e.p[1],c.p[2]-e.p[2]);
    let (dx,dy,dz) = (d.p[0]-e.p[0],d.p[1]-e.p[1],d.p[2]-e.p[2]);
    let (ab,bc,cd) = (ax*by-bx*ay,bx*cy-cx*by,cx*dy-dx*cy);
    let (da,ac,bd) = (dx*ay-ax*dy,ax*cy-cx*ay,bx*dy-dx*by);
    let abc = az*bc-bz*ac+cz*ab;
    let bcd = bz*cd-cz*bd+dz*bc;
    let cda = cz*da+dz*ac+az*cd;
    let dab = dz*ab+az*bd+bz*da;
    let (wa,wb,wc,wd) = (a.w-e.w,b.w-e.w,c.w-e.w,d.w-e.w);
    let (la,lb,lc,ld) = (ax*ax+ay*ay+az*az,bx*bx+by*by+bz*bz,cx*cx+cy*cy+cz*cz,dx*dx+dy*dy+dz*dz);
    let det = ((ld-wd)*abc-(lc-wc)*dab)+((lb-wb)*cda-(la-wa)*bcd);
    let m = |p: f64,q: f64,r: f64,s: f64| (p*q).abs()+(r*s).abs();
    let (abm,bcm,cdm) = (m(ax,by,bx,ay),m(bx,cy,cx,by),m(cx,dy,dx,cy));
    let (dam,acm,bdm) = (m(dx,ay,ax,dy),m(ax,cy,cx,ay),m(bx,dy,dx,by));
    let permanent = (cdm*bz.abs()+bdm*cz.abs()+bcm*dz.abs())*(la+wa.abs())
        +(dam*cz.abs()+acm*dz.abs()+cdm*az.abs())*(lb+wb.abs())
        +(abm*dz.abs()+bdm*az.abs()+dam*bz.abs())*(lc+wc.abs())
        +(bcm*az.abs()+acm*bz.abs()+abm*cz.abs())*(ld+wd.abs());
    let bound = POWER_BOUND*permanent;
    if det > bound || -det > bound { return -sign(det); }
    power_exact(a,b,c,d,e)
}

pub fn power_exact(a: Weighted,b: Weighted,c: Weighted,d: Weighted,e: Weighted) -> i8 {
    EXACT_CALLS.fetch_add(1,std::sync::atomic::Ordering::Relaxed);
    let rows: Vec<([Expansion;3],Expansion)> = [a,b,c,d].iter().map(|q| {
        let x = [0,1,2].map(|k| Expansion::diff(q.p[k],e.p[k]));
        let lift = x[0].mul(&x[0]).add(&x[1].mul(&x[1])).add(&x[2].mul(&x[2]))
            .sub(&Expansion::diff(q.w,e.w));
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
