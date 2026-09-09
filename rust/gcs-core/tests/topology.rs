use gcs_core::topology::{ClosedShell,Direction,EdgeUse,Error,Face};
use std::collections::BTreeMap;

fn forward(edge: usize) -> EdgeUse { EdgeUse {edge,direction:Direction::Forward} }
fn reverse(edge: usize) -> EdgeUse { EdgeUse {edge,direction:Direction::Reverse} }
fn disk(uses: Vec<EdgeUse>) -> Face { Face {loops:vec![uses]} }
fn tetrahedron() -> Vec<[usize;3]> { vec![[0,2,1],[0,1,3],[1,2,3],[2,0,3]] }

fn torus(n: usize) -> Vec<[usize;3]> {
    let mut triangles = vec![];
    for i in 0..n {
        for j in 0..n {
            let [a,b,c,d] = [i*n+j,((i+1)%n)*n+j,((i+1)%n)*n+(j+1)%n,i*n+(j+1)%n];
            triangles.extend([[a,b,c],[a,c,d]]);
        }
    }
    triangles
}

// The previous gear-rim test checked only these conditions. The counterexamples
// below deliberately pass both, so the new vertex and connectivity tests matter.
fn paired_edges_and_zero_euler(vertices: usize,triangles: &[[usize;3]]) {
    let mut edges = BTreeMap::<[usize;2],(usize,i32)>::new();
    for &[a,b,c] in triangles {
        for (a,b) in [(a,b),(b,c),(c,a)] {
            let e = edges.entry([a.min(b),a.max(b)]).or_default();
            e.0 += 1; e.1 += if a < b { 1 } else { -1 };
        }
    }
    assert!(edges.values().all(|&e| e == (2,0)));
    assert_eq!(vertices+triangles.len(),edges.len());
}

#[test]
fn sphere_and_periodic_torus_cells_have_manifold_vertex_links() {
    let sphere = ClosedShell::new(1,vec![[0,0]],
        vec![disk(vec![forward(0)]),disk(vec![reverse(0)])]).unwrap();
    assert_eq!(sphere.euler_characteristic(),2);
    assert_eq!(sphere.genus(),0);
    let torus = ClosedShell::new(1,vec![[0,0],[0,0]],
        vec![disk(vec![forward(0),forward(1),reverse(0),reverse(1)])]).unwrap();
    assert_eq!(torus.genus(),1);
    assert_eq!(torus.euler_characteristic(),0);
    assert_eq!(torus.incident_faces(0),Some([0,0]));
    assert_eq!(torus.incident_faces(99),None);
    assert_eq!(torus.vertex_count(),1);
    assert_eq!(torus.edges().len(),2);
    assert_eq!(torus.faces().len(),1);
}

#[test]
fn holes_contribute_annular_faces_and_parallel_edges_keep_their_identity() {
    // Two annuli joined on both circular boundaries form a torus. Counting each
    // annulus as a disk would incorrectly report a sphere.
    let shell = ClosedShell::new(2,vec![[0,0],[1,1]],vec![
        Face {loops:vec![vec![forward(0)],vec![forward(1)]]},
        Face {loops:vec![vec![reverse(0)],vec![reverse(1)]]},
    ]).unwrap();
    assert_eq!(shell.genus(),1);
    // Two distinct edges between the same vertices bound two digons. Collapsing
    // identities by their endpoint pair would create a falsely nonmanifold edge.
    let sphere = ClosedShell::new(2,vec![[0,1],[0,1]],vec![
        disk(vec![forward(0),reverse(1)]),disk(vec![forward(1),reverse(0)]),
    ]).unwrap();
    assert_eq!(sphere.genus(),0);
    assert_eq!(sphere.edges().len(),2);
}

#[test]
fn indexed_sphere_and_tori_are_invariant_under_order_and_orientation_changes() {
    for (vertices,triangles,genus) in [(4,tetrahedron(),0),(9,torus(3),1),(49,torus(7),1)] {
        let a = ClosedShell::from_triangles(vertices,&triangles).unwrap();
        assert_eq!(a.genus(),genus);
        assert_eq!(a.loop_vertices(0,0).unwrap().collect::<Vec<_>>(),triangles[0]);
        assert!(a.loop_vertices(triangles.len(),0).is_none());
        assert!(a.loop_vertices(0,1).is_none());
        let mut reversed = triangles.iter().rev().map(|&[a,b,c]|
            [vertices-1-a,vertices-1-c,vertices-1-b]).collect::<Vec<_>>();
        let b = ClosedShell::from_triangles(vertices,&reversed).unwrap();
        assert_eq!(a.euler_characteristic(),b.euler_characteristic());
        // Cyclic changes preserve every face's orientation.
        for t in &mut reversed { t.rotate_left(1); }
        assert_eq!(ClosedShell::from_triangles(vertices,&reversed).unwrap().genus(),genus);
    }
}

#[test]
fn a_pinched_vertex_is_refused_even_when_every_edge_pairs_and_euler_is_zero() {
    // Two tori and a sphere, all sharing one vertex: chi = 0+0+2-2 = 0.
    // Its three disconnected vertex fans are not a disk neighborhood.
    let mut triangles = vec![];
    let mut next = 1;
    for (vertices,part) in [(9,torus(3)),(9,torus(3)),(4,tetrahedron())] {
        triangles.extend(part.into_iter().map(|t| t.map(|v| if v == 0 { 0 } else { next+v-1 })));
        next += vertices-1;
    }
    paired_edges_and_zero_euler(next,&triangles);
    assert_eq!(ClosedShell::from_triangles(next,&triangles).unwrap_err(),
        Error::NonManifoldVertex {vertex:0});
}

#[test]
fn disjoint_closed_tori_are_not_accepted_as_one_shell() {
    let mut triangles = torus(3);
    triangles.extend(torus(3).into_iter().map(|t| t.map(|v| v+9)));
    paired_edges_and_zero_euler(18,&triangles);
    assert_eq!(ClosedShell::from_triangles(18,&triangles).unwrap_err(),Error::DisconnectedShell);
}

#[test]
fn missing_faces_excess_edge_uses_and_inconsistent_orientation_are_refused() {
    let mut triangles = tetrahedron();
    triangles.pop();
    assert!(matches!(ClosedShell::from_triangles(4,&triangles),Err(Error::EdgeUseCount {count:1,..})));
    let mut triangles = tetrahedron();
    triangles.push(triangles[0]);
    assert!(matches!(ClosedShell::from_triangles(4,&triangles),Err(Error::EdgeUseCount {count:3,..})));
    let mut triangles = tetrahedron();
    triangles[0].swap(0,1);
    assert!(matches!(ClosedShell::from_triangles(4,&triangles),Err(Error::InconsistentOrientation {..})));
    // The minimal Klein-bottle polygon has one pair glued in the same direction.
    assert_eq!(ClosedShell::new(1,vec![[0,0],[0,0]],
        vec![disk(vec![forward(0),forward(1),reverse(0),forward(1)])]).unwrap_err(),
        Error::InconsistentOrientation {edge:1});
}

#[test]
fn invalid_indices_loops_and_unused_topology_fail_without_repair() {
    let f = || vec![disk(vec![forward(0)]),disk(vec![reverse(0)])];
    assert_eq!(ClosedShell::new(0,vec![],vec![]).unwrap_err(),Error::EmptyShell);
    assert_eq!(ClosedShell::new(1,vec![[0,99]],f()).unwrap_err(),
        Error::VertexOutOfRange {edge:0,vertex:99});
    assert_eq!(ClosedShell::new(1,vec![[0,0]],vec![disk(vec![forward(99)])]).unwrap_err(),
        Error::EdgeOutOfRange {face:0,edge:99});
    assert_eq!(ClosedShell::new(1,vec![[0,0]],vec![Face {loops:vec![]}]).unwrap_err(),
        Error::EmptyFace {face:0});
    assert_eq!(ClosedShell::new(1,vec![[0,0]],vec![disk(vec![])]).unwrap_err(),
        Error::EmptyLoop {face:0,boundary:0});
    assert_eq!(ClosedShell::new(3,vec![[0,1],[2,0]],vec![disk(vec![forward(0),forward(1)])])
        .unwrap_err(),Error::OpenLoop {face:0,boundary:0,after:0});
    assert_eq!(ClosedShell::new(2,vec![[0,0]],f()).unwrap_err(),Error::UnusedVertices);
    assert_eq!(ClosedShell::new(usize::MAX,vec![[0,0]],f()).unwrap_err(),Error::UnusedVertices);
    assert_eq!(ClosedShell::new(2,vec![[0,0],[1,1]],f()).unwrap_err(),
        Error::EdgeUseCount {edge:1,count:0});
    assert_eq!(ClosedShell::from_triangles(4,&[[0,1,1]]).unwrap_err(),Error::DegenerateTriangle {face:0});
}

// An independent tiny binary encoder exercises the actual file layout, rather
// than sharing the production encoder's indexing or float handling.
fn stl(points: &[[f64;3]],triangles: &[[usize;3]],signed_zero: bool) -> Vec<u8> {
    let mut bytes = vec![0;80];
    bytes.extend((triangles.len() as u32).to_le_bytes());
    for (i,t) in triangles.iter().enumerate() {
        bytes.extend([0u8;12]); // Stored normals do not define topology; winding does.
        for &v in t {
            for &x in &points[v] {
                let x = if signed_zero && i % 2 == 0 && x == 0. { -0.0f32 } else { x as f32 };
                bytes.extend(x.to_le_bytes());
            }
        }
        bytes.extend([0u8;2]);
    }
    bytes
}

#[test]
fn encoded_stl_topology_uses_exact_positions_and_normalizes_signed_zero() {
    let points = [[0.,0.,0.],[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]];
    for signed_zero in [false,true] {
        let bytes = stl(&points,&tetrahedron(),signed_zero);
        let shell = gcs_core::mesh::stl_topology(&bytes).unwrap();
        assert_eq!(shell.vertex_count(),4);
        assert_eq!(shell.genus(),0);
    }
    let bytes = stl(&points,&tetrahedron()[..3],false);
    assert!(gcs_core::mesh::stl_topology(&bytes).unwrap_err().contains("EdgeUseCount"));
}

#[test]
fn encoded_stl_shells_validate_separate_and_cavity_walls_without_hiding_pinches() {
    let outer = [[0.,0.,0.],[4.,0.,0.],[0.,4.,0.],[0.,0.,4.]];
    for inner in [outer.map(|p| p.map(|x| x+10.)),
        outer.map(|p| p.map(|x| 0.25+x/8.))] {
        let points: Vec<_> = outer.into_iter().chain(inner).collect();
        let mut triangles = tetrahedron();
        triangles.extend(tetrahedron().into_iter().map(|[a,b,c]| [a+4,c+4,b+4]));
        let bytes = stl(&points,&triangles,true);
        let shells = gcs_core::mesh::stl_shells(&bytes).unwrap();
        assert_eq!(shells.len(),2);
        assert!(shells.iter().all(|s| s.genus() == 0 && s.vertex_count() == 4));
        assert!(gcs_core::mesh::stl_topology(&bytes).unwrap_err().contains("DisconnectedShell"));
        let mut pinched = points;
        pinched[4] = pinched[0];
        assert!(gcs_core::mesh::stl_shells(&stl(&pinched,&triangles,false))
            .unwrap_err().contains("NonManifoldVertex"));
        assert!(gcs_core::mesh::stl_shells(&stl(&outer,&tetrahedron()[..3],false)).is_err());
    }
    assert!(gcs_core::mesh::stl_shells(&stl(&[],&[],false)).is_err());
}

#[test]
fn float32_vertex_collisions_cannot_hide_behind_nondegenerate_triangles() {
    let triangles = torus(7);
    let mut points = vec![];
    for i in 0..7 {
        for j in 0..7 {
            let u = std::f64::consts::TAU*i as f64/7.;
            let v = std::f64::consts::TAU*j as f64/7.;
            points.push([10.+(3.+v.cos())*u.cos(),10.+(3.+v.cos())*u.sin(),10.+v.sin()]);
        }
    }
    // These nonadjacent vertices remain different as f64 but become identical as
    // float32. No triangle contains both, so triangle-collapse tests miss the pinch.
    points[24] = points[0].map(|x| x+1e-8);
    assert_ne!(points[0],points[24]);
    assert_eq!(points[0].map(|x| x as f32),points[24].map(|x| x as f32));
    let neighbors = |v| triangles.iter().filter(|t| t.contains(&v)).flatten().copied()
        .filter(|&p| p != v).collect::<std::collections::BTreeSet<_>>();
    assert!(neighbors(0).is_disjoint(&neighbors(24)));
    assert_eq!(ClosedShell::from_triangles(49,&triangles).unwrap().genus(),1);
    let error = gcs_core::mesh::stl_topology(&stl(&points,&triangles,false)).unwrap_err();
    assert!(error.contains("NonManifoldVertex"),"{error}");
}

#[test]
fn malformed_binary_stl_is_refused_before_topology_allocation() {
    let points = [[0.,0.,0.],[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]];
    let bytes = stl(&points,&tetrahedron(),false);
    for n in [0,80,83,84,bytes.len()-1] {
        assert!(gcs_core::mesh::stl_topology(&bytes[..n]).is_err());
    }
    let mut bad = bytes.clone();
    bad[80..84].copy_from_slice(&u32::MAX.to_le_bytes());
    assert!(gcs_core::mesh::stl_topology(&bad).unwrap_err().contains("byte length"));
    let mut bad = bytes;
    bad[96..100].copy_from_slice(&f32::NAN.to_le_bytes());
    assert!(gcs_core::mesh::stl_topology(&bad).unwrap_err().contains("nonfinite"));
}
