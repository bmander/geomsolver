//! The Delaunay machinery (`delaunay`, docs/field-meshing.md F3): exact expansions, filtered
//! predicates against their exact evaluation, and regular triangulations checked by brute force
//! on random, lattice, cospherical, coplanar, duplicated and weighted points.
use gcs_core::delaunay::{expansion::Expansion,predicates::{orient,orient_exact,power,power_exact,Weighted},Regular,Inserted};
use gcs_core::rng::Rng;

fn w(p: [f64;3]) -> Weighted { Weighted {p,w:0.} }

#[test]
fn expansions_are_exact() {
    // (2^53 + 1) − 2^53 is 1, which plain doubles lose.
    let big = 9_007_199_254_740_992.;
    let sum = Expansion::from(big).add(&Expansion::from(1.)).sub(&Expansion::from(big));
    assert_eq!(sum.estimate(),1.);
    // (1 + 2^-30)² − 1 − 2^-29 is 2^-60 exactly.
    let a = 1.+2f64.powi(-30);
    let square = Expansion::product(a,a).sub(&Expansion::from(1.)).sub(&Expansion::from(2f64.powi(-29)));
    assert_eq!(square.estimate(),2f64.powi(-60));
    assert_eq!(Expansion::diff(0.1,0.1).sign(),0);
    assert_eq!(Expansion::from(3.).mul(&Expansion::diff(1.,2f64.powi(-60))).sign(),1);
}

#[test]
fn orientation_is_exact_on_degenerate_and_nearly_degenerate_input() {
    // Coplanar integer points: exactly zero.
    assert_eq!(orient([0.,0.,0.],[1.,0.,0.],[0.,1.,0.],[3.,7.,0.]),0);
    assert_eq!(orient([1.,2.,3.],[4.,5.,6.],[7.,8.,9.],[10.,11.,12.]),0);
    // The fourth point a unit in the last place off the plane decides the sign.
    let up = f64::from_bits(1f64.to_bits()+1)-1.;
    assert_eq!(orient([0.,0.,0.],[1.,0.,0.],[0.,1.,0.],[0.3,0.3,up]),1);
    assert_eq!(orient([0.,0.,0.],[1.,0.,0.],[0.,1.,0.],[0.3,0.3,-up]),-1);
    // Nearly coplanar points far from the origin: the filter and the exact evaluation agree.
    let mut rng = Rng::new(7);
    let mut decided = 0;
    for _ in 0..100_000 {
        let base = [rng.uniform(-1e3,1e3),rng.uniform(-1e3,1e3),rng.uniform(-1e3,1e3)];
        let (u,v) = ([rng.uniform(-1.,1.),rng.uniform(-1.,1.),rng.uniform(-1.,1.)],[rng.uniform(-1.,1.),rng.uniform(-1.,1.),rng.uniform(-1.,1.)]);
        let on = |s: f64,t: f64| [0,1,2].map(|k| base[k]+s*u[k]+t*v[k]);
        let (a,b,c) = (on(0.,0.),on(1.,0.),on(0.,1.));
        let mut d = on(rng.uniform(-2.,2.),rng.uniform(-2.,2.));
        d[rng.int(3)] += rng.uniform(-1.,1.)*1e-12;
        let (fast,exact) = (orient(a,b,c,d),orient_exact(a,b,c,d));
        assert_eq!(fast,exact,"{a:?} {b:?} {c:?} {d:?}");
        if exact != 0 { decided += 1; }
    }
    assert!(decided > 1000);
}

#[test]
fn the_power_test_is_exact_on_cospherical_and_weighted_input() {
    // Points on the sphere of radius 5 about the origin, positively oriented.
    let (mut a,mut b,c,d) = (w([5.,0.,0.]),w([0.,5.,0.]),w([0.,0.,5.]),w([-3.,-4.,0.]));
    if orient(a.p,b.p,c.p,d.p) < 0 { std::mem::swap(&mut a,&mut b); }
    assert_eq!(power(a,b,c,d,w([0.,0.,0.])),1,"the centre is inside");
    assert_eq!(power(a,b,c,d,w([10.,10.,10.])),-1,"a far point is outside");
    for e in [[0.,0.,-5.],[3.,0.,4.],[0.,-3.,4.],[-4.,0.,-3.]] { assert_eq!(power(a,b,c,d,w(e)),0,"{e:?} is on it"); }
    // Weights: the centre with weight 25 is orthogonal to the sphere (power 0); heavier is in
    // conflict, lighter is not; a surface point with positive weight is in conflict.
    assert_eq!(power(a,b,c,d,Weighted {p:[0.,0.,0.],w:-25.}),0);
    assert_eq!(power(a,b,c,d,Weighted {p:[0.,0.,0.],w:-24.}),1);
    assert_eq!(power(a,b,c,d,Weighted {p:[0.,0.,0.],w:-26.}),-1);
    assert_eq!(power(a,b,c,d,Weighted {p:[3.,0.,4.],w:0.5}),1);
    // Nearly cospherical points: the filter and the exact evaluation agree.
    let mut rng = Rng::new(11);
    for _ in 0..100_000 {
        let r = rng.uniform(0.5,50.);
        let centre = [rng.uniform(-100.,100.),rng.uniform(-100.,100.),rng.uniform(-100.,100.)];
        let mut on = || {
            let (t,z) = (rng.uniform(0.,std::f64::consts::TAU),rng.uniform(-1.,1.));
            let s = (1.-z*z).sqrt();
            Weighted {p:[centre[0]+r*s*t.cos(),centre[1]+r*s*t.sin(),centre[2]+r*z],w:rng.uniform(-1e-9,1e-9)}
        };
        let (mut a,mut b,c,d,e) = (on(),on(),on(),on(),on());
        if orient(a.p,b.p,c.p,d.p) < 0 { std::mem::swap(&mut a,&mut b); }
        if orient(a.p,b.p,c.p,d.p) == 0 { continue; }
        assert_eq!(power(a,b,c,d,e),power_exact(a,b,c,d,e));
    }
}

fn checked(points: &[([f64;3],f64)],center: [f64;3],radius: f64) -> Regular {
    // Half the cases in input order and half in spatial order, which meets degeneracies in
    // another sequence.
    let mut r = Regular::new(center,radius);
    let order: Vec<usize> = if points.len() % 2 == 0 {
        gcs_core::delaunay::spatial_order(&points.iter().map(|p| p.0).collect::<Vec<_>>())
    } else { (0..points.len()).collect() };
    for &i in &order { let (p,weight) = points[i]; r.insert(p,weight).unwrap(); }
    r.check().unwrap();
    r
}

#[test]
fn random_points_triangulate_to_empty_spheres() {
    let mut rng = Rng::new(3);
    let points: Vec<([f64;3],f64)> = (0..1500).map(|_| ([rng.uniform(-1.,1.),rng.uniform(-1.,1.),rng.uniform(-1.,1.)],0.)).collect();
    let r = checked(&points,[0.;3],2.);
    assert!((4..r.points().len() as u32).all(|v| !r.is_hidden(v)),"no unweighted distinct point is hidden");
}

#[test]
fn a_lattice_is_degenerate_everywhere_and_still_triangulates() {
    let mut points = Vec::new();
    for i in 0..7 { for j in 0..7 { for k in 0..7 { points.push(([i as f64,j as f64,k as f64],0.)); } } }
    let r = checked(&points,[3.,3.,3.],6.);
    eprintln!("lattice: {} tetrahedra past a tie", r.ties);
}

#[test]
fn cospherical_and_coplanar_points_triangulate() {
    let mut rng = Rng::new(5);
    let sphere: Vec<([f64;3],f64)> = (0..400).map(|_| {
        let (t,z) = (rng.uniform(0.,std::f64::consts::TAU),rng.uniform(-1.,1.));
        let s = (1.-z*z).sqrt();
        ([s*t.cos(),s*t.sin(),z],0.)
    }).collect();
    checked(&sphere,[0.;3],1.5);
    let plane: Vec<([f64;3],f64)> = (0..400).map(|_| ([rng.uniform(-1.,1.),rng.uniform(-1.,1.),0.],0.)).collect();
    checked(&plane,[0.;3],1.5);
    // A lattice on a sphere of radius 5: every octant repeats (±3, ±4, 0) and its kin.
    let mut exact = Vec::new();
    for p in [[5.,0.,0.],[3.,4.,0.],[4.,3.,0.],[0.,3.,4.],[0.,4.,3.],[3.,0.,4.],[4.,0.,3.]] {
        for sx in [-1.,1.] { for sy in [-1.,1.] { for sz in [-1.,1.] {
            for perm in [[0,1,2],[1,2,0],[2,0,1]] {
                exact.push(([sx*p[perm[0]],sy*p[perm[1]],sz*p[perm[2]]],0.));
            }
        } } }
    }
    exact.push(([0.,0.,0.],0.));
    checked(&exact,[0.;3],6.);
}

#[test]
fn duplicates_and_weights_hide_points() {
    let mut r = Regular::new([0.;3],2.);
    let a = r.insert([0.1,0.2,0.3],0.).unwrap();
    assert!(matches!(a,Inserted::Vertex(_)));
    assert!(matches!(r.insert([0.1,0.2,0.3],0.).unwrap(),Inserted::Hidden(_)),"a duplicate is hidden");
    // The same place, heavier: it hides the first.
    let Inserted::Vertex(_) = r.insert([0.1,0.2,0.3],0.5).unwrap() else { panic!("a heavier duplicate is a vertex") };
    let Inserted::Vertex(first) = a else { unreachable!() };
    assert!(r.is_hidden(first),"the lighter duplicate is hidden");
    r.check().unwrap();
    // Random weights: some points have no power cell.
    let mut rng = Rng::new(9);
    let points: Vec<([f64;3],f64)> = (0..1200).map(|_| ([rng.uniform(-1.,1.),rng.uniform(-1.,1.),rng.uniform(-1.,1.)],rng.uniform(0.,0.02))).collect();
    let r = checked(&points,[0.;3],2.);
    let hidden = (4..r.points().len() as u32).filter(|&v| r.is_hidden(v)).count();
    eprintln!("weighted: {hidden} of {} hidden",points.len());
    assert!(hidden > 0);
    // Weighted lattice: equal weights are ties everywhere.
    let mut lattice = Vec::new();
    for i in 0..5 { for j in 0..5 { for k in 0..5 { lattice.push(([i as f64,j as f64,k as f64],if (i+j+k) % 2 == 0 { 0.1 } else { 0. })); } } }
    checked(&lattice,[2.,2.,2.],5.);
}

/// Insertion cost, measured: not a gate on time, which varies by machine.
#[test]
fn insertion_speed_is_measured() {
    let mut rng = Rng::new(21);
    let n = 50_000;
    let points: Vec<[f64;3]> = (0..n).map(|_| [rng.uniform(-1.,1.),rng.uniform(-1.,1.),rng.uniform(-1.,1.)]).collect();
    for sorted in [false,true] {
        gcs_core::delaunay::predicates::EXACT_CALLS.store(0,std::sync::atomic::Ordering::Relaxed);
        let started = std::time::Instant::now();
        let order: Vec<usize> = if sorted { gcs_core::delaunay::spatial_order(&points) } else { (0..n).collect() };
        let mut r = Regular::new([0.;3],2.);
        for &i in &order { r.insert(points[i],0.).unwrap(); }
        let elapsed = started.elapsed();
        eprintln!("{n} random points{} in {elapsed:?}: {:.2} µs a point, {} tetrahedra",if sorted { ", spatially ordered," } else { "" },
            elapsed.as_secs_f64()*1e6/n as f64,r.tets().count());
        eprintln!("  exact predicate calls: {}",gcs_core::delaunay::predicates::EXACT_CALLS.load(std::sync::atomic::Ordering::Relaxed));
    }
}

/// What a refinement reads of the triangulation: the conflict region a point would take is
/// what its insertion then removes; the tetrahedra made hold the new vertex; a facet seen from
/// its mirror is itself; every unhidden vertex has an incident tetrahedron; and each
/// orthosphere gives every vertex of its tetrahedron its own weight as power.
#[test]
fn an_insertion_reports_what_it_changed() {
    let mut rng = Rng::new(13);
    let mut r = Regular::new([0.;3],2.);
    let mut hint = None;
    for _ in 0..400 {
        let (p,weight) = ([rng.uniform(-1.,1.),rng.uniform(-1.,1.),rng.uniform(-1.,1.)],rng.uniform(0.,1e-3));
        let mut predicted = r.conflicts(p,weight,hint).unwrap();
        let inserted = r.insert_near(p,weight,hint).unwrap();
        let mut removed = r.removed().to_vec();
        predicted.sort_unstable(); removed.sort_unstable();
        assert_eq!(predicted,removed,"the conflict region is what the insertion removed");
        match inserted {
            Inserted::Vertex(v) => {
                assert!(!r.created().is_empty());
                for &t in r.created() { assert!(r.alive(t) && r.tet(t).v.contains(&v)); }
                for &t in r.removed() { assert!(!r.alive(t) || r.created().contains(&t),"a removed tetrahedron is dead or reused"); }
                hint = r.created().first().copied();
            }
            Inserted::Hidden(_) => assert!(r.created().is_empty() && r.removed().is_empty()),
        }
    }
    r.check().unwrap();
    for t in r.tets() {
        for i in 0..4 {
            if let Some((n,j)) = r.mirror(t,i) { assert_eq!(r.mirror(n,j),Some((t,i))); }
        }
        let (centre,radius2) = r.orthosphere(t);
        for v in r.tet(t).v {
            let q = r.points()[v as usize];
            let power = (0..3).map(|k| (q.p[k]-centre[k]).powi(2)).sum::<f64>()-radius2;
            assert!((power-q.w).abs() <= 1e-9*(1.+radius2),"vertex {v} has power {power} against tetrahedron {t}'s orthosphere, weight {}",q.w);
        }
    }
    for v in 0..r.points().len() as u32 {
        if r.is_hidden(v) { assert!(r.incident(v).is_none()); continue; }
        let t = r.incident(v).expect("an unhidden vertex has an incident tetrahedron");
        assert!(r.tet(t).v.contains(&v));
    }
}
