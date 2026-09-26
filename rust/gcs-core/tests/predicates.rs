//! The exact predicates and expansions (`delaunay::predicates`, `delaunay::expansion`) against a
//! reference that shares none of their arithmetic: `i128` on integer input, where every sum and
//! product is exact by construction. `tests/delaunay.rs` compares the filters with the exact
//! evaluations; these compare the exact evaluations with the truth, near and exactly at
//! degeneracy, and hold the properties a sign must have under relabelling, translation and
//! scaling by powers of two, at magnitudes far from one.
use gcs_core::delaunay::expansion::Expansion;
use gcs_core::delaunay::predicates::{exact_calls,orient,orient_exact,power,power_exact,Weighted};
use gcs_core::rng::Rng;

type P = [f64;3];

/// An integer in `[-bound, bound]`, every bit drawn (two draws of 32).
fn int(rng: &mut Rng,bound: i64) -> i64 {
    let draw = |rng: &mut Rng| (rng.next()*4_294_967_296.) as u64;
    let r = (draw(rng) << 32) | draw(rng);
    (r % (2*bound+1) as u64) as i64-bound
}
fn ipoint(rng: &mut Rng,bound: i64) -> [i64;3] { [int(rng,bound),int(rng,bound),int(rng,bound)] }
fn f(p: [i64;3]) -> P { p.map(|x| x as f64) }
fn sign(v: i128) -> i8 { v.signum() as i8 }

fn det3(r: [[i128;3];3]) -> i128 {
    r[0][0]*(r[1][1]*r[2][2]-r[1][2]*r[2][1])+r[0][1]*(r[1][2]*r[2][0]-r[1][0]*r[2][2])
        +r[0][2]*(r[1][0]*r[2][1]-r[1][1]*r[2][0])
}

/// `orient` in exact integer arithmetic.
fn orient_ref(a: [i64;3],b: [i64;3],c: [i64;3],d: [i64;3]) -> i8 {
    let row = |p: [i64;3]| [0,1,2].map(|k| (p[k]-a[k]) as i128);
    sign(det3([row(b),row(c),row(d)]))
}

/// `power` in exact integer arithmetic: the lifted determinant with rows `(p − e, |p − e|² −
/// (w − w_e))`, negative for a point in conflict, so the predicate is its sign turned.
fn power_ref(q: [([i64;3],i64);5]) -> i8 {
    let e = q[4];
    let rows: Vec<([i128;3],i128)> = q[..4].iter().map(|&(p,w)| {
        let x = [0,1,2].map(|k| (p[k]-e.0[k]) as i128);
        (x,x[0]*x[0]+x[1]*x[1]+x[2]*x[2]-(w-e.1) as i128)
    }).collect();
    let mut det = 0i128;
    for i in 0..4 {
        let r: Vec<[i128;3]> = (0..4).filter(|&j| j != i).map(|j| rows[j].0).collect();
        let term = rows[i].1*det3([r[0],r[1],r[2]]);
        det += if i % 2 == 0 { -term } else { term };
    }
    -sign(det)
}

fn weighted(q: ([i64;3],i64)) -> Weighted { Weighted {p:f(q.0),w:q.1 as f64} }

/// Four integer points on one plane (`a + s·u + t·v`), the fourth moved `nudge` off it along one
/// axis: exactly coplanar at a nudge of zero, and a unit off it otherwise.
fn near_plane(rng: &mut Rng,bound: i64,nudge: i64) -> [[i64;3];4] {
    let a = ipoint(rng,bound);
    let (u,v) = (ipoint(rng,64),ipoint(rng,64));
    let at = |s: i64,t: i64| [0,1,2].map(|k| a[k]+s*u[k]+t*v[k]);
    let mut d = at(int(rng,50),int(rng,50));
    d[rng.int(3)] += nudge;
    [a,at(1,0),at(0,1),d]
}

#[test]
fn orientation_is_the_integer_determinants_sign() {
    let mut rng = Rng::new(101);
    let (mut zeros,mut decided,before) = (0,0,exact_calls());
    for n in 0..20_000 {
        // Coordinates to 2^20: the determinant needs about 66 bits, past a double's 53 and an i64.
        let [a,b,c,d] = if n % 2 == 0 { near_plane(&mut rng,1 << 20,n as i64 % 3-1) }
            else { [0;4].map(|_| ipoint(&mut rng,1 << 20)) };
        let truth = orient_ref(a,b,c,d);
        assert_eq!(orient(f(a),f(b),f(c),f(d)),truth,"{a:?} {b:?} {c:?} {d:?}");
        assert_eq!(orient_exact(f(a),f(b),f(c),f(d)),truth,"{a:?} {b:?} {c:?} {d:?}");
        if truth == 0 { zeros += 1 } else { decided += 1 }
    }
    assert!(zeros > 3000 && decided > 3000,"{zeros} coplanar and {decided} decided");
    assert!(exact_calls()-before > 3000,"the filter sent the degenerate cases to the exact path");
}

/// Integer points of norm 25, up to sign: `25² = 15² + 20² = 7² + 24² = 9² + 12² + 20²`, …
const SPHERE: [[i64;3];12] = [[25,0,0],[0,25,0],[0,0,25],[15,20,0],[20,0,15],[0,7,24],[24,7,0],
    [9,12,20],[12,20,9],[20,9,12],[16,12,15],[12,15,16]];

#[test]
fn the_power_test_is_the_integer_determinants_sign() {
    let mut rng = Rng::new(103);
    let before = exact_calls();
    let mut exact_before = 0;
    let (mut zeros,mut decided) = (0,0);
    for n in 0..20_000 {
        // A small lattice meets cospherical and orthogonal configurations exactly and often; a
        // wide one, to 2^12 with weights to 2^20, meets them rarely and needs about 80 bits.
        let (bound,weights) = if n % 2 == 0 { (3,2) } else { (1 << 12,1 << 20) };
        let mut q: [([i64;3],i64);5] = [0;5].map(|_| (ipoint(&mut rng,bound),int(&mut rng,weights)));
        // Every third: five of the integer points on a sphere of radius 25 about a random centre,
        // all with one weight — exactly on the orthosphere whatever the translation.
        if n % 3 == 0 {
            let (centre,w) = (ipoint(&mut rng,1 << 12),int(&mut rng,1 << 20));
            q = [0;5].map(|_| {
                let p = SPHERE[rng.int(SPHERE.len())];
                let (sx,sy,sz) = ([-1,1][rng.int(2)],[-1,1][rng.int(2)],[-1,1][rng.int(2)]);
                ([centre[0]+sx*p[0],centre[1]+sy*p[1],centre[2]+sz*p[2]],w)
            });
        }
        if orient_ref(q[0].0,q[1].0,q[2].0,q[3].0) <= 0 { continue; }
        let truth = power_ref(q);
        let w = q.map(weighted);
        let calls = exact_calls();
        assert_eq!(power(w[0],w[1],w[2],w[3],w[4]),truth,"{q:?}");
        if exact_calls() > calls { exact_before += 1; }
        assert_eq!(power_exact(w[0],w[1],w[2],w[3],w[4]),truth,"{q:?}");
        if truth == 0 { zeros += 1 } else { decided += 1 }
    }
    assert!(zeros > 1000 && decided > 3000,"{zeros} on the orthosphere and {decided} decided");
    assert!(exact_before > 100,"the power filter reached its exact path {exact_before} times");
    assert!(exact_calls() > before);
}

#[test]
fn a_sign_follows_relabelling_translation_and_scaling() {
    let mut rng = Rng::new(107);
    for n in 0..5000 {
        let [a,b,c,d] = near_plane(&mut rng,1 << 16,n as i64 % 3-1);
        let s = orient(f(a),f(b),f(c),f(d));
        // An odd permutation turns the sign, an even one keeps it.
        assert_eq!(orient(f(b),f(a),f(c),f(d)),-s);
        assert_eq!(orient(f(a),f(c),f(b),f(d)),-s);
        assert_eq!(orient(f(b),f(c),f(a),f(d)),s);
        // An integer translation is exact, and moves nothing.
        let t = ipoint(&mut rng,1 << 30);
        let moved = |p: [i64;3]| [0,1,2].map(|k| (p[k]+t[k]) as f64);
        assert_eq!(orient(moved(a),moved(b),moved(c),moved(d)),s);
        // A power of two is exact on every coordinate, far above one and far below it.
        for e in [-150,-40,40,200] {
            let k = 2f64.powi(e);
            let scaled = |p: [i64;3]| f(p).map(|x| x*k);
            assert_eq!(orient(scaled(a),scaled(b),scaled(c),scaled(d)),s,"at 2^{e}");
        }
    }
    for _ in 0..5000 {
        let q: [([i64;3],i64);5] = [0;5].map(|_| (ipoint(&mut rng,4),int(&mut rng,3)));
        if orient_ref(q[0].0,q[1].0,q[2].0,q[3].0) <= 0 { continue; }
        let w = q.map(weighted);
        let s = power(w[0],w[1],w[2],w[3],w[4]);
        // Swapping two of the tetrahedron's vertices turns its orientation and the sign with it.
        assert_eq!(power(w[1],w[0],w[2],w[3],w[4]),-s);
        assert_eq!(power(w[0],w[1],w[3],w[2],w[4]),-s);
        let t = ipoint(&mut rng,1 << 20);
        let moved = |x: Weighted| Weighted {p:[0,1,2].map(|k| x.p[k]+t[k] as f64),w:x.w};
        let m = w.map(moved);
        assert_eq!(power(m[0],m[1],m[2],m[3],m[4]),s);
        // Lengths by 2^e, and so weights (squared lengths) by 2^2e.
        for e in [-150,-20,20,150] {
            let (k,k2) = (2f64.powi(e),2f64.powi(2*e));
            let scaled = |x: Weighted| Weighted {p:x.p.map(|v| v*k),w:x.w*k2};
            let z = w.map(scaled);
            assert_eq!(power(z[0],z[1],z[2],z[3],z[4]),s,"at 2^{e}");
        }
    }
}

/// The value of the lowest set bit of a nonzero, finite, normal double.
fn lowest_bit(x: f64) -> f64 {
    let bits = x.abs().to_bits();
    let exponent = ((bits >> 52) & 0x7ff) as i32;
    let mantissa = (bits & ((1 << 52)-1)) | if exponent > 0 { 1 << 52 } else { 0 };
    2f64.powi(exponent.max(1)-1075+mantissa.trailing_zeros() as i32)
}

/// An expansion is a sum of nonoverlapping components, least significant first: each component
/// is smaller than the lowest set bit of the next, and none is zero.
fn well_formed(e: &Expansion) {
    let parts = e.parts();
    assert!(parts.iter().all(|&p| p != 0. && p.is_finite()),"{e:?}");
    for w in parts.windows(2) {
        assert!(w[0].abs() < lowest_bit(w[1]),"{:e} overlaps {:e} in {e:?}",w[0],w[1]);
    }
}

/// A double over a wide range of magnitudes and either sign: most bits set, and exact as an
/// integer times a power of two.
fn wide(rng: &mut Rng) -> f64 {
    let m = rng.uniform(-1.,1.);
    m*2f64.powi(int(rng,60) as i32)
}

#[test]
fn expansions_hold_their_invariants_and_are_exact() {
    let mut rng = Rng::new(109);
    for _ in 0..20_000 {
        let (a,b,c,d) = (wide(&mut rng),wide(&mut rng),wide(&mut rng),wide(&mut rng));
        let x = Expansion::product(a,b).add(&Expansion::diff(c,d));
        let y = Expansion::product(c,d).sub(&Expansion::from(a)).mul(&Expansion::diff(b,a));
        for e in [&x,&y,&x.add(&y),&x.sub(&y),&x.mul(&y),&x.neg()] { well_formed(e); }
        // (x + y) − y is x exactly, and a product commutes, to the last bit.
        assert_eq!(x.add(&y).sub(&y).sub(&x).sign(),0);
        assert_eq!(x.mul(&y).sub(&y.mul(&x)).sign(),0);
        assert_eq!(x.sub(&x).parts().len(),0,"x − x has no components");
        assert_eq!(x.neg().sign(),-x.sign());
    }
}

#[test]
fn an_expansions_sign_is_the_integer_sign() {
    // Products of 40-bit integers need 80 bits, and their differences cancel most of them.
    let mut rng = Rng::new(113);
    let mut zeros = 0;
    for n in 0..20_000 {
        let [a,b] = [0;2].map(|_| int(&mut rng,1 << 40));
        // Every fourth: `a·b − b·a`, the products cancelling exactly and only `e` left.
        let (c,d) = if n % 4 == 0 { (b,a) } else { (int(&mut rng,1 << 40),int(&mut rng,1 << 40)) };
        let e = int(&mut rng,3);
        let x = Expansion::product(a as f64,b as f64).sub(&Expansion::product(c as f64,d as f64)).add(&Expansion::from(e as f64));
        let truth = (a as i128)*(b as i128)-(c as i128)*(d as i128)+e as i128;
        assert_eq!(x.sign(),sign(truth),"{a}·{b} − {c}·{d} + {e}");
        if truth == 0 { zeros += 1; }
    }
    assert!(zeros > 0);
}

/// A point the predicates cannot answer for is refused by the triangulation, not triangulated
/// wrongly: past `MAGNITUDE`, the power test's products near the largest double.
#[test]
fn the_triangulation_refuses_a_point_past_the_predicates_range() {
    use gcs_core::delaunay::{Regular,regular::MAGNITUDE};
    let mut r = Regular::new([0.;3],1.);
    assert!(r.insert([0.1,0.2,0.3],0.).is_ok());
    let far = r.insert([4.*MAGNITUDE,0.,0.],0.).unwrap_err();
    assert!(far.contains("beyond the exact predicates' range"),"{far}");
    assert!(r.insert([0.,0.,0.],4.*MAGNITUDE*MAGNITUDE).is_err());
    r.check().unwrap();
}
