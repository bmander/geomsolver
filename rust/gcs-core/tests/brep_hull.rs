//! **The convex hull of points, as a solid** (`brep::build::hull`, docs/contract/brep.md): each
//! hull a valid closed boundary of plane faces, one a facet, every point given on or beneath every
//! face, every corner a point given and no two faces across an edge one plane — and its volume and
//! area the closed form, read off its faces with nothing meshed.
use gcs_core::brep::build::hull;
use gcs_core::brep::geom::{tolerance,Surface,V};
use gcs_core::brep::props::{area,volume};
use gcs_core::brep::topo::Brep;
use gcs_core::rng::Rng;
use gcs_core::space::{add,distance,dot,norm,scale,sub};
use std::collections::BTreeSet;

fn close(what: &str,a: f64,b: f64) { assert!((a-b).abs() <= 1e-12*b.abs().max(1.),"{what}: {a} against {b} ({:e})",(a-b)/b); }

/// Each face's unit outward normal and a point on it.
fn planes(b: &Brep) -> Vec<(V,V)> {
    b.faces.iter().map(|f| match &f.surface {
        Surface::Plane(frame) => (if f.reversed { scale(frame.z,-1.) } else { frame.z },frame.o),
        s => panic!("a hull's face on a {s:?}"),
    }).collect()
}

/// The vertices each face's loops pass through.
fn corners(b: &Brep) -> Vec<BTreeSet<u32>> {
    b.faces.iter().map(|f| f.loops.iter().flatten().flat_map(|c| b.edges[c.edge as usize].v).collect()).collect()
}

/// What every hull holds (the module's properties), and its volume and area.
fn holds(b: &Brep,points: &[V]) -> (f64,f64) {
    let (lo,hi) = points.iter().fold(([f64::INFINITY;3],[f64::NEG_INFINITY;3]),|(lo,hi),p|
        ([0,1,2].map(|k| lo[k].min(p[k])),[0,1,2].map(|k| hi[k].max(p[k]))));
    let tol = tolerance(distance(lo,hi));
    b.check(tol).unwrap();
    assert!(b.faces.iter().all(|f| f.loops.len() == 1),"a hull's face has one loop");
    assert_eq!(b.vertices.len() as i64-b.edges.len() as i64+b.faces.len() as i64,2,"V − E + F");
    for v in &b.vertices { assert!(points.contains(&v.p),"the corner {:?} is no point given",v.p); }
    let planes = planes(b);
    for p in points {
        for &(n,o) in &planes { assert!(dot(sub(*p,o),n) <= tol,"{p:?} stands {:e} past a face",dot(sub(*p,o),n)); }
    }
    // a face's corners on its plane, and the faces across each edge two planes: some corner of
    // the one stands clear beneath the other
    let corners = corners(b);
    for (f,(n,o)) in planes.iter().enumerate() {
        for &v in &corners[f] { assert!(dot(sub(b.vertices[v as usize].p,*o),*n).abs() <= tol,"face {f}: a corner off its plane"); }
    }
    for e in 0..b.edges.len() as u32 {
        let fs: Vec<usize> = (0..b.faces.len()).filter(|&f| b.faces[f].loops.iter().flatten().any(|c| c.edge == e)).collect();
        assert_eq!(fs.len(),2);
        let (n,o) = planes[fs[0]];
        let below = corners[fs[1]].iter().map(|&v| -dot(sub(b.vertices[v as usize].p,o),n)).fold(0.,f64::max);
        assert!(below > tol,"edge {e}: faces {} and {} are one plane",fs[0],fs[1]);
    }
    (volume(b),b.faces.iter().map(|f| area(b,f)).sum())
}

fn signs(k: usize) -> impl Iterator<Item = [f64;3]> { (0..1usize << k).map(move |m| [0,1,2].map(|i| if i < k && m >> i & 1 == 1 { -1. } else { 1. })) }

/// Each of `p`'s coordinates turned round: `(x, y, z)`, `(y, z, x)` and `(z, x, y)`.
fn cyclic(p: V) -> [V;3] { [p,[p[1],p[2],p[0]],[p[2],p[0],p[1]]] }

/// **The five Platonic solids are the hulls of their corners**, each face a whole polygon: four
/// triangles, six squares, eight triangles, twenty triangles, twelve pentagons — the pentagons'
/// corners irrational, so on one plane only to rounding.
#[test]
fn the_platonic_solids_are_the_hulls_of_their_corners() {
    let phi = (1.+5f64.sqrt())/2.;
    let s5 = 5f64.sqrt();
    let tetrahedron = vec![[3.,3.,3.],[3.,-3.,-3.],[-3.,3.,-3.],[-3.,-3.,3.]];
    let cube: Vec<V> = signs(3).map(|s| scale(s,2.)).collect();
    let octahedron: Vec<V> = [1.,-1.].iter().flat_map(|&s| cyclic([10.*s,0.,0.])).collect();
    let icosahedron: Vec<V> = signs(2).flat_map(|s| cyclic([0.,s[0],s[1]*phi])).collect();
    let mut dodecahedron: Vec<V> = signs(3).collect();
    dodecahedron.extend(signs(2).flat_map(|s| cyclic([0.,s[0]/phi,s[1]*phi])));
    let edge = |a: f64| a;
    for (name,points,faces,v,s) in [
        ("tetrahedron",tetrahedron,4,72.,72.*3f64.sqrt()),
        ("cube",cube,6,64.,96.),
        ("octahedron",octahedron,8,4000./3.,2.*3f64.sqrt()*200.),
        ("icosahedron",icosahedron,20,5.*(3.+s5)/12.*8.,5.*3f64.sqrt()*4.),
        ("dodecahedron",dodecahedron,12,(15.+7.*s5)/4.*edge(2./phi).powi(3),3.*(25.+10.*s5).sqrt()*(2./phi).powi(2)),
    ] {
        let b = hull(&points).unwrap_or_else(|e| panic!("{name}: {e}"));
        assert_eq!(b.faces.len(),faces,"{name}'s faces");
        assert_eq!(b.vertices.len(),points.len(),"{name}'s corners");
        let (volume,area) = holds(&b,&points);
        close(name,volume,v);
        close(name,area,s);
    }
}

/// **A point inside, on a face, along an edge or at a corner again is no corner**: a cube's
/// corners with its centre, the middles of its faces and its edges, each corner given twice and
/// once more a millionth of the tolerance off, and a lattice of points through it, are the cube.
#[test]
fn points_inside_on_a_face_or_along_an_edge_are_no_corners() {
    let corners: Vec<V> = signs(3).map(|s| scale(s,2.)).collect();
    let mut points = vec![[0.;3]];
    points.extend(corners.iter().flat_map(|&c| [c,c,add(c,[1e-15,0.,-1e-15])]));
    for i in 0..3 {
        for s in [-2.,2.] { let mut p = [0.;3]; p[i] = s; points.push(p); }
        for s in signs(2) { let mut p = [0.;3]; p[(i+1)%3] = 2.*s[0]; p[(i+2)%3] = 2.*s[1]; points.push(p); }
    }
    for i in 0..5 { for j in 0..5 { for k in 0..5 { points.push([i,j,k].map(|x| x as f64-2.)); } } }
    let b = hull(&points).unwrap();
    assert_eq!((b.vertices.len(),b.edges.len(),b.faces.len()),(8,12,6));
    let (v,s) = holds(&b,&points);
    close("cube",v,64.);
    close("cube",s,96.);
}

/// **A convex solid slid along a line fills the hull of where it starts and where it stops**:
/// brep-bench's swept/box-slid-aslant, twelve faces in six parallel pairs, and its
/// loft/square-to-octagon, the hull of the square and the octagon, to the closed forms the bench
/// gives.
#[test]
fn the_benchs_slid_box_and_lofted_octagon_are_hulls() {
    let slide = [8.,5.,3.];
    let mut points: Vec<V> = signs(3).map(|s| [0,1,2].map(|i| if s[i] > 0. { [10.,6.,4.][i] } else { 0. })).collect();
    points.extend(points.clone().into_iter().map(|p| add(p,slide)));
    let b = hull(&points).unwrap();
    assert_eq!(b.faces.len(),12);
    let (v,s) = holds(&b,&points);
    close("box slid",v,812.);
    close("box slid",s,542.6189318971692);

    let square = [[20.,-20.],[20.,20.],[-20.,20.],[-20.,-20.]];
    let octagon = [[6.,-14.],[14.,-6.],[14.,6.],[6.,14.],[-6.,14.],[-14.,6.],[-14.,-6.],[-6.,-14.]];
    let points: Vec<V> = square.iter().map(|&[x,y]| [x,y,0.]).chain(octagon.iter().map(|&[x,y]| [x,y,30.])).collect();
    let b = hull(&points).unwrap();
    assert_eq!(b.faces.len(),10);
    let (v,s) = holds(&b,&points);
    close("square to octagon",v,33760.);
    close("square to octagon",s,2256.+624.*26f64.sqrt()+160.*22f64.sqrt());
}

/// **A cloud of points**, in a ball and on its sphere, gives a hull holding the properties; given
/// in another order, the same hull (the same corners, the same volume to rounding).
#[test]
fn a_cloud_of_points_gives_one_hull_in_any_order() {
    let mut rng = Rng::new(17);
    let mut points = Vec::new();
    while points.len() < 400 {
        let p = [0;3].map(|_| rng.uniform(-1.,1.));
        if norm(p) <= 1. { points.push(p); }
    }
    for _ in 0..200 {
        let p = [0;3].map(|_| rng.normal(0.,1.));
        points.push(scale(p,1./norm(p)));
    }
    let b = hull(&points).unwrap();
    let (v,_) = holds(&b,&points);
    assert!(v < 4./3.*std::f64::consts::PI && v > 3.,"{v}");
    let mut turned = points.clone();
    for k in (1..turned.len()).rev() { let j = (rng.next()*(k+1) as f64) as usize; turned.swap(k,j.min(k)); }
    let c = hull(&turned).unwrap();
    let corners = |b: &Brep| b.vertices.iter().map(|v| v.p.map(f64::to_bits)).collect::<BTreeSet<_>>();
    assert_eq!(corners(&b),corners(&c));
    assert_eq!(b.faces.len(),c.faces.len());
    close("the cloud turned",holds(&c,&turned).0,v);
}

/// **Points that bound nothing are refused**, by what they are: too few, one line, one plane
/// (a lattice in it too), one point, or a point not finite.
#[test]
fn points_bounding_nothing_are_refused() {
    let plane: Vec<V> = (0..5).flat_map(|i| (0..5).map(move |j| [i as f64,j as f64,0.])).collect();
    for (points,says) in [
        (vec![[0.;3],[1.,0.,0.],[0.,1.,0.]],"needs four"),
        (vec![[0.;3],[1.,0.,0.],[2.,0.,0.],[3.,0.,0.]],"one line"),
        (plane,"one plane"),
        (vec![[1.;3];5],"one point"),
        (vec![[0.;3],[1.,0.,0.],[0.,1.,0.],[0.,0.,f64::NAN]],"not finite"),
    ] {
        let e = hull(&points).err().unwrap_or_else(|| panic!("{points:?} built"));
        assert!(e.contains(says),"{e}");
    }
}
