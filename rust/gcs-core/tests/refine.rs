//! Delaunay refinement of an implicit boundary with protected features (`delaunay::refine`,
//! docs/field-meshing.md F4), on shapes whose answers are closed forms: a sphere, a torus, two
//! separate spheres, a cylinder whose rims are features and a cube whose twelve edges are.
use gcs_core::delaunay::refine::{mesh,Criteria,Mesh,Readings,WithReadings};
use std::collections::BTreeMap;

type P = [f64;3];

fn criteria(size: f64) -> Criteria {
    Criteria {facet_size:size,facet_distance:size/20.,facet_angle:25.,edge_size:size,bisection:1e-9,max_points:20_000,normal_angle:0.}
}

/// The mesh is a closed oriented 2-manifold: every directed edge used once and its reverse once,
/// and each vertex's triangles one fan. Returns the Euler characteristic and the volume.
fn closed(m: &Mesh) -> (i64,f64) {
    let mut directed: BTreeMap<(u32,u32),usize> = BTreeMap::new();
    for t in &m.triangles {
        for j in 0..3 { *directed.entry((t[j],t[(j+1)%3])).or_default() += 1; }
    }
    for (&(a,b),&n) in &directed {
        assert_eq!(n,1,"edge {a}-{b} is walked {n} times one way");
        assert_eq!(directed.get(&(b,a)),Some(&1),"edge {a}-{b} is not walked back");
    }
    let (v,e,f) = (m.vertices.len() as i64,directed.len() as i64/2,m.triangles.len() as i64);
    let volume: f64 = m.triangles.iter().map(|t| {
        let [a,b,c] = t.map(|i| m.vertices[i as usize]);
        (a[0]*(b[1]*c[2]-b[2]*c[1])-a[1]*(b[0]*c[2]-b[2]*c[0])+a[2]*(b[0]*c[1]-b[1]*c[0]))/6.
    }).sum();
    (v-e+f,volume)
}

fn sphere(c: P,r: f64) -> impl Fn(P) -> f64 { move |p| ((p[0]-c[0]).powi(2)+(p[1]-c[1]).powi(2)+(p[2]-c[2]).powi(2)).sqrt()-r }

#[test]
fn a_sphere_is_closed_near_its_surface_and_holds_its_volume() {
    let f = sphere([0.;3],1.);
    let m = mesh(&mut |p| f(p),[0.;3],1.5,&[],&criteria(0.2)).unwrap();
    eprintln!("sphere: {} vertices, {} triangles, {:?}",m.vertices.len(),m.triangles.len(),m.report);
    let (chi,volume) = closed(&m);
    assert_eq!(chi,2);
    for v in &m.vertices { assert!(f(*v).abs() < 1e-8,"a vertex {v:?} is off the sphere by {}",f(*v)); }
    for t in &m.triangles {
        let c: P = std::array::from_fn(|k| t.iter().map(|&i| m.vertices[i as usize][k]).sum::<f64>()/3.);
        assert!(f(c).abs() <= 0.2/20.*1.5,"a centroid is {} from the sphere",f(c));
    }
    let exact = 4./3.*std::f64::consts::PI;
    assert!((volume-exact).abs() < 0.02*exact,"volume {volume} against {exact}");
}

#[test]
fn a_torus_has_genus_one_and_two_spheres_are_both_found() {
    let torus = |p: P| ((p[0]*p[0]+p[1]*p[1]).sqrt()-1.).hypot(p[2])-0.35;
    let m = mesh(&mut |p| torus(p),[0.;3],1.6,&[],&criteria(0.12)).unwrap();
    let (chi,volume) = closed(&m);
    eprintln!("torus: {} triangles, χ = {chi}, {:?}",m.triangles.len(),m.report);
    assert_eq!(chi,0);
    let exact = 2.*std::f64::consts::PI.powi(2)*0.35*0.35;
    assert!((volume-exact).abs() < 0.03*exact,"volume {volume} against {exact}");
    let (a,b) = (sphere([-1.,0.,0.],0.5),sphere([1.,0.,0.],0.5));
    let m = mesh(&mut |p| a(p).min(b(p)),[0.;3],1.8,&[],&criteria(0.12)).unwrap();
    let (chi,_) = closed(&m);
    assert_eq!(chi,4,"two spheres");
}

fn circle(c: P,r: f64,n: usize) -> Vec<P> {
    let mut v: Vec<P> = (0..n).map(|k| { let t = std::f64::consts::TAU*k as f64/n as f64; [c[0]+r*t.cos(),c[1]+r*t.sin(),c[2]] }).collect();
    v.push(v[0]);
    v
}

/// Whether every protected curve point lies within `tol` of a mesh edge: the feature survives.
fn curve_on_edges(m: &Mesh,curve: &[P],tol: f64) -> bool {
    let mut edges = Vec::new();
    for t in &m.triangles { for j in 0..3 { edges.push((m.vertices[t[j] as usize],m.vertices[t[(j+1)%3] as usize])); } }
    curve.iter().all(|&p| edges.iter().any(|&(a,b)| {
        let ab: P = std::array::from_fn(|k| b[k]-a[k]);
        let ap: P = std::array::from_fn(|k| p[k]-a[k]);
        let l = ab.iter().map(|x| x*x).sum::<f64>();
        let s = if l > 0. { (ab.iter().zip(&ap).map(|(x,y)| x*y).sum::<f64>()/l).clamp(0.,1.) } else { 0. };
        (0..3).map(|k| (a[k]+s*ab[k]-p[k]).powi(2)).sum::<f64>().sqrt() < tol
    }))
}

#[test]
fn a_cylinder_keeps_its_rims() {
    let cylinder = |p: P| ((p[0]*p[0]+p[1]*p[1]).sqrt()-0.6).max(p[2].abs()-0.5);
    let rims = [circle([0.,0.,0.5],0.6,256),circle([0.,0.,-0.5],0.6,256)];
    let m = mesh(&mut |p| cylinder(p),[0.;3],1.2,&rims,&criteria(0.12)).unwrap();
    let (chi,volume) = closed(&m);
    eprintln!("cylinder: {} triangles, {:?}",m.triangles.len(),m.report);
    assert_eq!(chi,2);
    let exact = std::f64::consts::PI*0.36;
    assert!((volume-exact).abs() < 0.02*exact,"volume {volume} against {exact}");
    // The rim, sampled finely, lies along mesh edges within the chord's sagitta.
    let fine = circle([0.,0.,0.5],0.6,2000);
    assert!(curve_on_edges(&m,&fine,0.6*(1.-(std::f64::consts::PI/ (2.*std::f64::consts::PI*0.6/0.12).ceil()).cos())+1e-6),
        "the top rim is not along mesh edges");
}

#[test]
fn a_cube_keeps_its_edges_and_corners() {
    let cube = |p: P| p[0].abs().max(p[1].abs()).max(p[2].abs())-0.5;
    let h = 0.5;
    let mut edges = Vec::new();
    let corners: Vec<P> = (0..8).map(|k| [if k&1 == 0 { -h } else { h },if k&2 == 0 { -h } else { h },if k&4 == 0 { -h } else { h }]).collect();
    for a in 0..8 { for b in a+1..8 {
        if (a^b as usize).count_ones() == 1 { edges.push(vec![corners[a],corners[b]]); }
    } }
    let m = mesh(&mut |p| cube(p),[0.;3],1.,&edges,&criteria(0.15)).unwrap();
    let (chi,volume) = closed(&m);
    eprintln!("cube: {} triangles, {:?}",m.triangles.len(),m.report);
    assert_eq!(chi,2);
    assert!((volume-1.).abs() < 1e-6,"volume {volume}: a cube with its edges kept is exact");
    for c in &corners { assert!(m.vertices.iter().any(|v| (0..3).all(|k| (v[k]-c[k]).abs() < 1e-12)),"corner {c:?} is a vertex"); }
}

#[test]
fn a_thin_ring_the_rays_miss_is_found_by_the_lattice() {
    // A torus of tube 0.06 round a circle of radius 1, centred on the rays' origin: rays spread
    // over the sphere pass above and below it, and only the lattice finds it.
    let ring = |p: P| ((p[0]*p[0]+p[1]*p[1]).sqrt()-1.).hypot(p[2])-0.06;
    let mut c = criteria(0.05);
    c.max_points = 200_000;
    let m = mesh(&mut |p| ring(p),[0.;3],1.2,&[],&c).unwrap();
    let (chi,volume) = closed(&m);
    eprintln!("thin ring: {} triangles, {:?}",m.triangles.len(),m.report);
    assert_eq!(chi,0);
    let exact = 2.*std::f64::consts::PI.powi(2)*0.06*0.06;
    assert!((volume-exact).abs() < 0.05*exact,"volume {volume} against {exact}");
}

#[test]
fn facets_follow_the_local_feature_size_by_their_normals() {
    // A pancake: an ellipsoid 1 across and 0.05 thick, meshed with facets far wider than it is
    // thick. With readings and `normal_angle`, a facet whose vertices' normals disagree — reaching
    // round the rim, or across from one face to the other — is refined, so the mesh closes as a
    // sphere does, holds the volume, and every facet keeps its normals within the angle asked.
    let (a,c) = (1.,0.05);
    let f = move |p: P| ((p[0]/a).powi(2)+(p[1]/a).powi(2)+(p[2]/c).powi(2)).sqrt()-1.;
    let grad = move |p: P| -> P {
        let r = ((p[0]/a).powi(2)+(p[1]/a).powi(2)+(p[2]/c).powi(2)).sqrt().max(1e-300);
        [p[0]/(a*a*r),p[1]/(a*a*r),p[2]/(c*c*r)]
    };
    let mut crit = criteria(0.3);
    crit.facet_distance = 0.01;
    crit.normal_angle = 45.;
    // no curves to space, but the spacing also sets the least surface ball refined (a twentieth of
    // it): kept under the rim's radius of curvature, c²/a = 0.0025, so the rim can be resolved
    crit.edge_size = 0.02;
    crit.max_points = 200_000;
    let domain = WithReadings {value:move |p: P| f(p),reading:move |p: P,_| (f(p),grad(p)),readings:Readings::Checked};
    let mut run = gcs_core::delaunay::refine::Progressive::new(Box::new(domain),[0.;3],1.2,Vec::new(),crit);
    while !run.step(usize::MAX).unwrap() {}
    let m = run.finished().unwrap();
    let (chi,volume) = closed(&m);
    eprintln!("pancake: {} triangles, {:?}",m.triangles.len(),m.report);
    assert_eq!(chi,2);
    let exact = 4./3.*std::f64::consts::PI*a*a*c;
    assert!((volume-exact).abs() < 0.05*exact,"volume {volume} against {exact}");
    let unit = |p: P| { let g = grad(p); let l = (g[0]*g[0]+g[1]*g[1]+g[2]*g[2]).sqrt(); g.map(|x| x/l) };
    for t in &m.triangles {
        let n = t.map(|i| unit(m.vertices[i as usize]));
        for (i,j) in [(0,1),(1,2),(2,0)] {
            let cos = (0..3).map(|k| n[i][k]*n[j][k]).sum::<f64>();
            assert!(cos.clamp(-1.,1.).acos().to_degrees() <= 45.+1e-6,"a facet's normals are {}° apart",cos.acos().to_degrees());
        }
    }
}
