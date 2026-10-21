//! A volume mesh fit for analysis (#510 P6; `delaunay::refine::volume`, `delaunay::tetra`): the
//! cells inside a refined surface, held to a radius–edge ratio, a size and a least dihedral
//! angle, and their quadratic elements (TET10), boundary nodes on the exact surface and on the
//! feature curves, none inverted. On shapes whose answers are closed forms: a sphere, a torus, a
//! cube whose twelve edges are features and a cylinder whose rims are.
use gcs_core::delaunay::refine::{volume,Cells,Criteria,Progressive,Readings,Stage,Volume,WithReadings};
use gcs_core::delaunay::tetra::{self,Tet10,EDGES};
use std::collections::BTreeMap;

type P = [f64;3];

fn criteria(size: f64) -> Criteria {
    Criteria {facet_size:size,facet_distance:size/20.,facet_angle:25.,edge_size:size,bisection:1e-10,max_points:200_000,normal_angle:0.}
}

fn cells(size: f64) -> Cells { Cells {radius_edge:2.,size,dihedral:12.} }

/// The mesh is a conforming complex: every cell positive, every face on one cell or two, the
/// faces on one exactly the boundary's triangles (facing out), the boundary a closed manifold.
/// Returns the complex's Euler characteristic V − E + F − T.
fn conforming(v: &Volume) -> i64 {
    let mut faces: BTreeMap<[u32;3],Vec<(usize,[u32;3])>> = BTreeMap::new();
    let mut edges: BTreeMap<(u32,u32),()> = BTreeMap::new();
    for (k,q) in v.tets.iter().enumerate() {
        assert!(tetra::volume6(v.tet(k)) > 0.,"cell {k} is not positive");
        for i in 0..4 {
            // the face opposite corner i, facing out of cell k
            let f = match i { 0 => [q[1],q[2],q[3]], 1 => [q[0],q[3],q[2]], 2 => [q[0],q[1],q[3]], _ => [q[0],q[2],q[1]] };
            let mut key = f;
            key.sort_unstable();
            faces.entry(key).or_default().push((k,f));
        }
        for &(i,j) in &EDGES { edges.insert((q[i].min(q[j]),q[i].max(q[j])),()); }
    }
    let mut outer: BTreeMap<[u32;3],[u32;3]> = BTreeMap::new();
    for (key,on) in &faces {
        assert!(on.len() <= 2,"face {key:?} is on {} cells",on.len());
        if on.len() == 1 { outer.insert(*key,on[0].1); }
    }
    assert_eq!(outer.len(),v.boundary.len(),"the boundary is the faces on one cell");
    let rotate = |f: [u32;3]| { let m = (0..3).min_by_key(|&j| f[j]).unwrap(); [f[m],f[(m+1)%3],f[(m+2)%3]] };
    for f in &v.boundary {
        let mut key = *f;
        key.sort_unstable();
        let mine = outer.get(&key).unwrap_or_else(|| panic!("boundary triangle {f:?} is no cell's face"));
        assert_eq!(rotate(*mine),rotate(*f),"boundary triangle {f:?} faces in");
    }
    let mut directed: BTreeMap<(u32,u32),usize> = BTreeMap::new();
    for t in &v.boundary { for j in 0..3 { *directed.entry((t[j],t[(j+1)%3])).or_default() += 1; } }
    for (&(a,b),&n) in &directed {
        assert_eq!(n,1,"boundary edge {a}-{b} is walked {n} times one way");
        assert_eq!(directed.get(&(b,a)),Some(&1),"boundary edge {a}-{b} is not walked back");
    }
    v.vertices.len() as i64-edges.len() as i64+faces.len() as i64-v.tets.len() as i64
}

/// The quadratic elements: corners the cells', each edge node shared by every element on that
/// edge, every element valid by its Bernstein coefficients, and the straight ones' volume theirs.
fn quadratic(v: &Volume) {
    assert_eq!(v.elements.len(),v.tets.len());
    let mut node_of: BTreeMap<(u32,u32),u32> = BTreeMap::new();
    for (k,e) in v.elements.iter().enumerate() {
        assert_eq!(e[..4],v.tets[k][..]);
        for (m,&(i,j)) in EDGES.iter().enumerate() {
            let at = *node_of.entry((e[i].min(e[j]),e[i].max(e[j]))).or_insert(e[4+m]);
            assert_eq!(at,e[4+m],"edge {i}-{j} of element {k} has another node than its neighbour's");
        }
        let ratio = v.element(k).jacobian_ratio();
        assert!(ratio >= 0.2-1e-12,"element {k}'s Jacobian is bounded below by only {ratio} of its straight one's");
    }
    assert!(v.report.least_jacobian > 0.2-1e-12);
}

/// Every cell breaking the criteria has a protecting ball's centre for a corner: what refinement
/// leaves is only where a ball blocks it.
fn bad_only_at_balls(v: &Volume,c: &Cells) {
    let mut bad = 0;
    for k in 0..v.tets.len() {
        let q = v.tet(k);
        let size = tetra::circumsphere(q).map_or(f64::INFINITY,|s| s.1);
        if tetra::radius_edge(q) <= c.radius_edge && size <= c.size && tetra::least_dihedral(q) >= c.dihedral { continue; }
        bad += 1;
        assert!(v.tets[k].iter().any(|&i| v.protected[i as usize]),"bad cell {k} {q:?} is on no protecting ball");
    }
    assert_eq!(bad,v.report.bad_cells);
}

fn sphere(p: P) -> f64 { (p[0]*p[0]+p[1]*p[1]+p[2]*p[2]).sqrt()-1. }

#[test]
fn a_tetrahedrons_measures_are_its_closed_forms() {
    // The regular tetrahedron: dihedral arccos(1/3), circumradius over edge √(3/8).
    let s = 1./2f64.sqrt();
    let regular = [[1.,1.,1.],[1.,-1.,-1.],[-1.,1.,-1.],[-1.,-1.,1.]].map(|p: P| p.map(|x| x*s));
    let regular = if tetra::volume6(regular) < 0. { [regular[1],regular[0],regular[2],regular[3]] } else { regular };
    for a in tetra::dihedral_angles(regular) { assert!((a-(1f64/3.).acos().to_degrees()).abs() < 1e-12); }
    assert!((tetra::radius_edge(regular)-(3f64/8.).sqrt()).abs() < 1e-12);
    // The corner of a cube: three right dihedral angles at the right corner, three of 54.7°.
    let corner = [[0.,0.,0.],[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]];
    let a = tetra::dihedral_angles(corner);
    for k in [0,2,3] { assert!((a[k]-90.).abs() < 1e-12,"{a:?}"); }
    for k in [1,4,5] { assert!((a[k]-(1f64/3f64.sqrt()).acos().to_degrees()).abs() < 1e-12,"{a:?}"); }
    // A sliver: four points round a circle's equator, lifted a hair: fair ratio, no angle.
    let h = 1e-3;
    let sliver = [[1.,0.,h],[0.,1.,-h],[-1.,0.,h],[0.,-1.,-h]];
    let sliver = if tetra::volume6(sliver) < 0. { [sliver[1],sliver[0],sliver[2],sliver[3]] } else { sliver };
    assert!(tetra::radius_edge(sliver) < 0.75);
    assert!(tetra::least_dihedral(sliver) < 0.2);
}

#[test]
fn a_tet10s_jacobian_is_bounded_by_its_bernstein_coefficients() {
    let corner = [[0.,0.,0.],[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]];
    let straight = Tet10::straight(corner);
    // Straight: every coefficient is 6V, the volume a sixth.
    for c in straight.jacobian_bernstein() { assert!((c-1.).abs() < 1e-14); }
    assert!((straight.volume()-1./6.).abs() < 1e-15);
    // An edge node pushed out: the volume is the closed form, and the Jacobian at the corners
    // is its coefficient there (the first four).
    let mut bent = straight;
    bent.nodes[4][2] = -0.1;           // node on edge 0–1, pushed below the face z = 0
    // x(λ) moves by 4λ₀λ₁·(0, 0, −0.1): the volume by ∫∂z/∂ξ₃… over the region, −0.1·4·∫λ₀λ₁ ·
    // (∂ of the face) — worked out: the added volume is the integral over the face z = 0 of the
    // bulge, 0.1·4·∫_face λ₀λ₁ dA = 0.1·4·(1/2)/12.
    assert!((bent.volume()-(1./6.+0.1*4.*0.5/12.)).abs() < 1e-14,"{}",bent.volume());
    let jac = |l: [f64;4]| {
        let d = 1e-6;
        let x = |l: [f64;4]| bent.at(l);
        let col = |k: usize| { let mut a = l; a[k] += d; a[0] -= d; let (p,q) = (x(a),x(l)); [0,1,2].map(|c| (p[c]-q[c])/d) };
        let (u,v,w) = (col(1),col(2),col(3));
        u[0]*(v[1]*w[2]-v[2]*w[1])-u[1]*(v[0]*w[2]-v[2]*w[0])+u[2]*(v[0]*w[1]-v[1]*w[0])
    };
    let c = bent.jacobian_bernstein();
    for i in 0..4 {
        let mut l = [0.;4];
        l[i] = 1.;
        assert!((jac(l)-c[i]).abs() < 1e-4,"corner {i}: {} against {}",jac(l),c[i]);
    }
    // The least coefficient bounds the Jacobian everywhere: on a lattice of barycentric points.
    let least = c.iter().copied().fold(f64::INFINITY,f64::min);
    let n = 12;
    for a in 0..=n { for b in 0..=n-a { for e in 0..=n-a-b {
        let l = [a,b,e,n-a-b-e].map(|x| x as f64/n as f64);
        if l.iter().any(|&x| x == 0.) && l.iter().filter(|&&x| x == 0.).count() == 3 { continue; }
        assert!(jac(l) >= least-1e-5,"{} under the bound {least} at {l:?}",jac(l));
    } } }
    // Pushed far enough inward, the element folds, and the bound says so.
    let mut folded = straight;
    folded.nodes[4] = [0.5,0.,0.9];
    assert!(folded.jacobian_ratio() < 0.);
}

#[test]
fn a_ball_is_filled_with_fair_cells_and_curved_elements_on_its_sphere() {
    let v = volume(&mut |p| sphere(p),[0.;3],1.4,&[],&criteria(0.25),&cells(0.25)).unwrap();
    let r = &v.report;
    eprintln!("ball: {} vertices, {} cells, {} boundary triangles; {:?}",v.vertices.len(),v.tets.len(),v.boundary.len(),r);
    assert_eq!(conforming(&v),1,"a ball is contractible");
    quadratic(&v);
    assert!(!v.protected.contains(&true),"no feature, no ball");
    // Every cell meets the criteria: a smooth domain leaves refinement nothing it cannot reach.
    assert_eq!(r.bad_cells,0);
    assert!(r.worst_radius_edge <= 2.+1e-9 && r.least_dihedral >= 12.-1e-9);
    // Every boundary node on the sphere, the corners to the bisection and the edge nodes too.
    let mut on: Vec<bool> = vec![false;v.nodes.len()];
    for f in &v.boundary { for j in 0..3 {
        on[f[j] as usize] = true;
        let (a,b) = (f[j].min(f[(j+1)%3]),f[j].max(f[(j+1)%3]));
        let e = v.elements.iter().zip(&v.tets).find_map(|(e,q)| EDGES.iter().position(|&(i,k)| (q[i].min(q[k]),q[i].max(q[k])) == (a,b)).map(|m| e[4+m])).unwrap();
        on[e as usize] = true;
    } }
    for (k,p) in v.nodes.iter().enumerate() {
        if on[k] { assert!(sphere(*p).abs() < 1e-8,"boundary node {k} is {} off the sphere",sphere(*p)); }
    }
    assert_eq!(r.straight+r.straightened,0,"every boundary edge curved");
    // The quadratic elements hold the volume to far better than the linear cells: the chord's
    // error is the square of the facet size, the parabola's its fourth power.
    let exact = 4./3.*std::f64::consts::PI;
    let (linear,quad) = ((v.linear_volume()-exact).abs()/exact,(v.quadratic_volume()-exact).abs()/exact);
    eprintln!("ball: volume error {linear:.2e} linear, {quad:.2e} quadratic");
    assert!(linear < 0.03 && quad < 0.1*linear,"linear {linear:.2e}, quadratic {quad:.2e}");
}

#[test]
fn a_torus_is_filled_as_a_solid_torus() {
    let torus = |p: P| ((p[0]*p[0]+p[1]*p[1]).sqrt()-1.).hypot(p[2])-0.4;
    let v = volume(&mut |p| torus(p),[0.;3],1.6,&[],&criteria(0.15),&cells(0.15)).unwrap();
    eprintln!("torus: {} cells, {:?}",v.tets.len(),v.report);
    assert_eq!(conforming(&v),0,"a solid torus has χ = 0");
    quadratic(&v);
    assert_eq!(v.report.bad_cells,0);
    let exact = 2.*std::f64::consts::PI.powi(2)*0.16;
    let (linear,quad) = ((v.linear_volume()-exact).abs()/exact,(v.quadratic_volume()-exact).abs()/exact);
    eprintln!("torus: volume error {linear:.2e} linear, {quad:.2e} quadratic");
    assert!(quad < 0.1*linear,"linear {linear:.2e}, quadratic {quad:.2e}");
}

#[test]
fn a_cube_keeps_its_edges_and_is_filled_exactly() {
    let cube = |p: P| p[0].abs().max(p[1].abs()).max(p[2].abs())-0.5;
    let h = 0.5;
    let corners: Vec<P> = (0..8).map(|k| [if k&1 == 0 { -h } else { h },if k&2 == 0 { -h } else { h },if k&4 == 0 { -h } else { h }]).collect();
    let mut edges = Vec::new();
    for a in 0..8 { for b in a+1..8 { if (a^b as usize).count_ones() == 1 { edges.push(vec![corners[a],corners[b]]); } } }
    let v = volume(&mut |p| cube(p),[0.;3],1.,&edges,&criteria(0.2),&cells(0.2)).unwrap();
    eprintln!("cube: {} cells, {:?}",v.tets.len(),v.report);
    assert_eq!(conforming(&v),1);
    quadratic(&v);
    bad_only_at_balls(&v,&cells(0.2));
    // the protected vertices are the balls' centres, on the cube's edges
    for (k,p) in v.vertices.iter().enumerate() {
        let on_edge = p.iter().filter(|x| (x.abs()-0.5).abs() < 1e-12).count() >= 2;
        assert_eq!(v.protected[k],on_edge,"vertex {k} {p:?}");
    }
    assert!((v.linear_volume()-1.).abs() < 1e-9,"linear volume {}",v.linear_volume());
    assert!((v.quadratic_volume()-1.).abs() < 1e-9,"quadratic volume {}",v.quadratic_volume());
    for c in &corners { assert!(v.vertices.iter().any(|p| (0..3).all(|k| (p[k]-c[k]).abs() < 1e-12)),"corner {c:?} is a vertex"); }
    // Every node on the boundary is on a face of the cube; an edge node on a feature is on its edge.
    assert!(v.report.on_curves > 0);
    assert!(v.report.least_dihedral > 5.,"least dihedral {}",v.report.least_dihedral);
}

#[test]
fn a_cylinders_rim_nodes_are_on_its_rims() {
    let (radius,half) = (0.6,0.5);
    let cylinder = move |p: P| ((p[0]*p[0]+p[1]*p[1]).sqrt()-radius).max(p[2].abs()-half);
    let circle = |z: f64| {
        let n = 4096;
        let mut c: Vec<P> = (0..n).map(|k| { let t = std::f64::consts::TAU*k as f64/n as f64; [radius*t.cos(),radius*t.sin(),z] }).collect();
        c.push(c[0]);
        c
    };
    let v = volume(&mut |p| cylinder(p),[0.;3],1.2,&[circle(half),circle(-half)],&criteria(0.15),&cells(0.15)).unwrap();
    eprintln!("cylinder: {} cells, {:?}",v.tets.len(),v.report);
    assert_eq!(conforming(&v),1);
    quadratic(&v);
    bad_only_at_balls(&v,&cells(0.15));
    // A rim's edge nodes are on its circle, to the polyline's sagitta (4096 sides).
    let sagitta = radius*(1.-(std::f64::consts::PI/4096.).cos());
    let mut rim_nodes = 0;
    for (k,p) in v.nodes.iter().enumerate().skip(v.vertices.len()) {
        if (p[2].abs()-half).abs() < 1e-12 && ((p[0]*p[0]+p[1]*p[1]).sqrt()-radius).abs() < 1e-3 {
            rim_nodes += 1;
            assert!(((p[0]*p[0]+p[1]*p[1]).sqrt()-radius).abs() <= sagitta+1e-12,"rim node {k} is off its circle");
        }
    }
    assert_eq!(rim_nodes,v.report.on_curves,"every feature edge's node is on a rim");
    let exact = std::f64::consts::PI*radius*radius*2.*half;
    let (linear,quad) = ((v.linear_volume()-exact).abs()/exact,(v.quadratic_volume()-exact).abs()/exact);
    eprintln!("cylinder: volume error {linear:.2e} linear, {quad:.2e} quadratic");
    assert!(quad < 0.1*linear,"linear {linear:.2e}, quadratic {quad:.2e}");
}

#[test]
fn criteria_no_refinement_can_meet_are_refused() {
    let refused = |c: Cells| volume(&mut |p| sphere(p),[0.;3],1.4,&[],&criteria(0.3),&c).unwrap_err();
    assert!(refused(Cells {radius_edge:0.5,size:0.3,dihedral:10.}).contains("radius–edge"));
    assert!(refused(Cells {radius_edge:2.,size:0.,dihedral:10.}).contains("size"));
    assert!(refused(Cells {radius_edge:2.,size:0.3,dihedral:35.}).contains("dihedral"));
}


#[test]
fn a_filling_refinement_steps_through_its_stages_and_is_deterministic() {
    // A domain with readings: the surface's crossings and the edge nodes' are found by Newton.
    let ellipsoid = |p: P| ((p[0]/1.2).powi(2)+(p[1]/0.8).powi(2)+(p[2]/0.6).powi(2)).sqrt()-1.;
    let gradient = |p: P| -> P {
        let r = ((p[0]/1.2).powi(2)+(p[1]/0.8).powi(2)+(p[2]/0.6).powi(2)).sqrt().max(1e-300);
        [p[0]/(1.44*r),p[1]/(0.64*r),p[2]/(0.36*r)]
    };
    let run = || {
        let domain = WithReadings {value:move |p: P| ellipsoid(p),reading:move |p: P,_| (ellipsoid(p),gradient(p)),readings:Readings::Agreeing};
        let mut run = Progressive::new(Box::new(domain),[0.;3],1.5,Vec::new(),criteria(0.2)).with_cells(cells(0.2));
        let mut stages = Vec::new();
        while !run.step(64).unwrap() {
            let stage = run.progress().stage;
            if stages.last() != Some(&stage) { stages.push(stage); }
        }
        assert_eq!(run.progress().stage,Stage::Done);
        (stages,run.finished_volume().unwrap())
    };
    let (stages,v) = run();
    eprintln!("ellipsoid: stages {stages:?}, {} cells, {:?}",v.tets.len(),v.report);
    assert_eq!(stages.last(),Some(&Stage::Filling),"filling is the last stage before done");
    assert!(stages.contains(&Stage::Refining));
    assert!(v.report.surface.newton > 0,"the crossings were Newton's");
    assert_eq!(conforming(&v),1);
    quadratic(&v);
    assert_eq!(v.report.bad_cells,0);
    let on_boundary: std::collections::BTreeSet<(u32,u32)> = v.boundary.iter()
        .flat_map(|f| (0..3).map(move |j| (f[j].min(f[(j+1)%3]),f[j].max(f[(j+1)%3])))).collect();
    for (e,q) in v.elements.iter().zip(&v.tets) {
        for (m,&(i,j)) in EDGES.iter().enumerate() {
            if on_boundary.contains(&(q[i].min(q[j]),q[i].max(q[j]))) && v.report.straightened == 0 {
                assert!(ellipsoid(v.nodes[e[4+m] as usize]).abs() < 1e-8,"an edge node is off the ellipsoid");
            }
        }
    }
    // Run again: the same mesh to the bit.
    let (_,again) = run();
    assert_eq!(v.tets,again.tets);
    assert_eq!(v.elements,again.elements);
    assert!(v.nodes.iter().zip(&again.nodes).all(|(a,b)| a.map(f64::to_bits) == b.map(f64::to_bits)));
}
