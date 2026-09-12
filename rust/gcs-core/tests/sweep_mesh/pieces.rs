//! The construction's small pieces on meshes written by hand: the planar
//! union, the cut of a mesh along a polyline, and the tolerant T-junction
//! split. Each is judged on what it must do, not on a sweep.
use gcs_core::solid::swept_boundary::{CutMesh,KeptMesh,boundary_loops,planar_union,split_at_vertices};

type V3 = [f64;3];

fn area(mesh: &KeptMesh) -> f64 {
    mesh.triangles.iter().map(|t| {
        let [a,b,c] = t.map(|v| mesh.vertices[v as usize]);
        let (u,w) = ([b[0]-a[0],b[1]-a[1],b[2]-a[2]],[c[0]-a[0],c[1]-a[1],c[2]-a[2]]);
        let n = [u[1]*w[2]-u[2]*w[1],u[2]*w[0]-u[0]*w[2],u[0]*w[1]-u[1]*w[0]];
        (n[0]*n[0]+n[1]*n[1]+n[2]*n[2]).sqrt()/2.
    }).sum()
}

fn edge_uses(triangles: &[[u32;3]]) -> std::collections::BTreeMap<(u32,u32),usize> {
    let mut uses = std::collections::BTreeMap::new();
    for t in triangles { for k in 0..3 { let (a,b) = (t[k],t[(k+1)%3]); *uses.entry((a.min(b),a.max(b))).or_insert(0) += 1; } }
    uses
}

#[test]
fn the_planar_union_keeps_one_copy_of_a_doubled_triangle() {
    let mesh = KeptMesh {vertices:vec![[0.,0.,0.],[1.,0.,0.],[0.,1.,0.],[0.,0.,0.],[1.,0.,0.],[0.,1.,0.]],triangles:vec![[0,1,2],[3,4,5]],sheet:vec![0,1]};
    let (out,replaced) = planar_union(&mesh,1e-9);
    assert_eq!(replaced,2);
    assert_eq!(out.triangles.len(),1);
    assert_eq!(out.sheet,vec![0],"the earlier sheet's copy stays");
    assert!((area(&out)-0.5).abs() < 1e-12);
}

#[test]
fn the_planar_union_of_two_overlapping_squares_has_their_union_area() {
    // unit squares at the origin and offset by a half in x and y: 1 + 1 - 1/4
    let square = |x: f64,y: f64| -> Vec<V3> { vec![[x,y,0.],[x+1.,y,0.],[x+1.,y+1.,0.],[x,y+1.,0.]] };
    let mut vertices = square(0.,0.); vertices.extend(square(0.5,0.5));
    let mesh = KeptMesh {vertices,triangles:vec![[0,1,2],[0,2,3],[4,5,6],[4,6,7]],sheet:vec![0,0,1,1]};
    let (out,_) = planar_union(&mesh,1e-9);
    assert!((area(&out)-1.75).abs() < 1e-9,"area {}",area(&out));
    // every output triangle faces +z
    for t in &out.triangles {
        let [a,b,c] = t.map(|v| out.vertices[v as usize]);
        assert!((b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0]) > 0.);
    }
}

#[test]
fn the_planar_union_leaves_other_planes_alone() {
    let mesh = KeptMesh {vertices:vec![[0.,0.,0.],[1.,0.,0.],[0.,1.,0.],[0.,0.,1.],[1.,0.,1.],[0.,1.,1.]],triangles:vec![[0,1,2],[3,4,5]],sheet:vec![0,1]};
    let (out,replaced) = planar_union(&mesh,1e-9);
    assert_eq!(replaced,0);
    assert_eq!(out.triangles,mesh.triangles);
}

/// A unit cube as twelve triangles, wound outward.
fn cube() -> (Vec<V3>,Vec<[u32;3]>) {
    let vertices: Vec<V3> = (0..8).map(|i| [(i&1) as f64,((i>>1)&1) as f64,((i>>2)&1) as f64]).collect();
    let quads: [[u32;4];6] = [[0,2,3,1],[4,5,7,6],[0,1,5,4],[2,6,7,3],[0,4,6,2],[1,3,7,5]];
    let mut triangles = Vec::new();
    for q in quads { triangles.push([q[0],q[1],q[2]]); triangles.push([q[0],q[2],q[3]]); }
    (vertices,triangles)
}

#[test]
fn a_cut_round_a_cube_at_half_height_leaves_two_components_that_close() {
    let (vertices,triangles) = cube();
    let mut mesh = CutMesh::new(vertices,triangles.clone(),vec![0;triangles.len()],1e-3,1e-2);
    // the points and no normals: the mesh's own serve
    let belt: Vec<(V3,V3)> = vec![[0.,0.,0.5],[1.,0.,0.5],[1.,1.,0.5],[0.,1.,0.5]].into_iter().map(|p| (p,[0.;3])).collect();
    mesh.cut_along(&belt,true).unwrap();
    let components = mesh.components();
    let distinct: std::collections::BTreeSet<usize> = components.iter().copied().collect();
    assert_eq!(distinct.len(),2,"a belt round the cube parts it in two");
    // the whole mesh is still closed and the cut edges are shared by the two halves
    for (_,n) in edge_uses(&mesh.triangles) { assert_eq!(n,2); }
    for &(a,b) in &mesh.cuts {
        let sides: std::collections::BTreeSet<usize> = mesh.triangles.iter().enumerate().filter(|(_,t)| t.contains(&a) && t.contains(&b)).map(|(i,_)| components[i]).collect();
        assert_eq!(sides.len(),2,"a cut edge lies between the halves");
    }
    // every belt point is a vertex exactly
    for (p,_) in &belt { assert!(mesh.vertices.iter().any(|v| v == p)); }
}

#[test]
fn a_cut_along_an_open_line_across_one_face_parts_nothing() {
    let (vertices,triangles) = cube();
    let mut mesh = CutMesh::new(vertices,triangles.clone(),vec![0;triangles.len()],1e-3,1e-2);
    mesh.cut_along(&[([0.25,0.,0.5],[0.;3]),([0.75,0.,0.5],[0.;3])],false).unwrap();
    let distinct: std::collections::BTreeSet<usize> = mesh.components().into_iter().collect();
    assert_eq!(distinct.len(),1);
    for (_,n) in edge_uses(&mesh.triangles) { assert_eq!(n,2); }
}

#[test]
fn a_column_point_off_the_mesh_is_refused_by_name() {
    let (vertices,triangles) = cube();
    let mut mesh = CutMesh::new(vertices,triangles.clone(),vec![0;triangles.len()],1e-3,1e-2);
    let err = mesh.cut_along(&[([0.5,0.5,0.5],[0.;3]),([0.5,0.5,2.],[0.;3])],false).unwrap_err();
    assert!(err.contains("off the tool's mesh"),"{err}");
}

/// A cube belted at z = 0.5 and again at x = 0.3. The second belt crosses the
/// first on the faces y = 0 and y = 1 away from any vertex, so each crossing
/// splits an edge the first cut runs along.
fn crossed_belts() -> CutMesh {
    let (vertices,triangles) = cube();
    let mut mesh = CutMesh::new(vertices,triangles.clone(),vec![0;triangles.len()],1e-3,1e-2);
    let belt = |pts: [V3;4]| -> Vec<(V3,V3)> { pts.into_iter().map(|p| (p,[0.;3])).collect() };
    mesh.cut_along(&belt([[0.,0.,0.5],[1.,0.,0.5],[1.,1.,0.5],[0.,1.,0.5]]),true).unwrap();
    mesh.cut_along(&belt([[0.3,0.,0.],[0.3,0.,1.],[0.3,1.,1.],[0.3,1.,0.]]),true).unwrap();
    mesh
}

#[test]
fn a_second_belt_crossing_the_first_parts_a_cube_in_four() {
    let mesh = crossed_belts();
    let distinct: std::collections::BTreeSet<usize> = mesh.components().into_iter().collect();
    assert_eq!(distinct.len(),4,"two belts crossing twice part the surface in four");
    for (_,n) in edge_uses(&mesh.triangles) { assert_eq!(n,2); }
}

#[test]
fn every_cut_is_an_edge_a_triangle_still_walks() {
    let mesh = crossed_belts();
    // a cut the second belt split in two is two cuts: named after the edge that
    // is gone it separates nothing, and the components flood through the seam
    for &(a,b) in &mesh.cuts {
        assert!(mesh.triangles.iter().any(|t| t.contains(&a) && t.contains(&b)),
            "the cut {a}-{b} ({:?} to {:?}) is no edge of the mesh",mesh.vertices[a as usize],mesh.vertices[b as usize]);
    }
}

/// A mesh of the given triangles over the given points, every triangle from one sheet.
fn mesh_of(vertices: Vec<V3>,triangles: Vec<[u32;3]>) -> KeptMesh {
    let sheet = vec![0;triangles.len()];
    KeptMesh {vertices,triangles,sheet}
}

/// A square as two triangles wound the same way round.
fn square() -> KeptMesh {
    mesh_of(vec![[0.,0.,0.],[1.,0.,0.],[1.,1.,0.],[0.,1.,0.]],vec![[0,1,2],[0,2,3]])
}

const COINCIDENCE: f64 = 1e-7;
const SNAP: f64 = 5e-3;

#[test]
fn an_open_sheet_wound_one_way_is_clean() {
    let h = gcs_core::solid::swept_boundary::hygiene(&square(),COINCIDENCE,SNAP);
    assert!(h.clean(),"{}",h.report(SNAP));
    // the shared diagonal is walked once each way; the four sides once each
    assert_eq!(h.boundary_edges,4,"an open sheet's rim is no fault");
    assert_eq!((h.crowded_edges,h.same_way,h.degenerate),(0,0,0));
    assert_eq!((h.vertices,h.used,h.triangles),(4,4,2));
}

#[test]
fn hygiene_names_a_fold_a_crowded_edge_and_a_flat_triangle() {
    // the second triangle wound the other way: both walk the diagonal 2 -> 0
    let folded = mesh_of(vec![[0.,0.,0.],[1.,0.,0.],[1.,1.,0.],[0.,1.,0.]],vec![[0,1,2],[0,3,2]]);
    let h = gcs_core::solid::swept_boundary::hygiene(&folded,COINCIDENCE,SNAP);
    assert_eq!(h.same_way,1,"{}",h.report(SNAP));
    assert!(!h.clean());
    // a third triangle on the diagonal: three walk one edge
    let crowded = mesh_of(vec![[0.,0.,0.],[1.,0.,0.],[1.,1.,0.],[0.,1.,0.],[2.,2.,0.]],
        vec![[0,1,2],[0,2,3],[0,2,4]]);
    let h = gcs_core::solid::swept_boundary::hygiene(&crowded,COINCIDENCE,SNAP);
    assert_eq!(h.crowded_edges,1,"{}",h.report(SNAP));
    assert!(!h.clean());
    // three points on one line bound nothing
    let flat = mesh_of(vec![[0.,0.,0.],[1.,0.,0.],[2.,0.,0.]],vec![[0,1,2]]);
    let h = gcs_core::solid::swept_boundary::hygiene(&flat,COINCIDENCE,SNAP);
    assert_eq!(h.degenerate,1,"{}",h.report(SNAP));
    assert!(!h.clean());
}

#[test]
fn hygiene_reports_only_near_pairs_of_vertices_in_use() {
    // two vertices at one point are what every seed contributes and the weld joins: not a pair
    let mut doubled = square();
    doubled.vertices.push([0.,0.,0.]);
    doubled.triangles.push([4,1,2]); doubled.sheet.push(0);
    let h = gcs_core::solid::swept_boundary::hygiene(&doubled,COINCIDENCE,SNAP);
    assert_eq!(h.closest,None,"an exact duplicate is the weld's to join, not a whisker");
    // a pair between the coincidence and the snap is one nothing downstream closes
    let mut near = square();
    near.vertices.push([2e-3,0.,0.]);
    near.triangles.push([4,1,2]); near.sheet.push(0);
    let h = gcs_core::solid::swept_boundary::hygiene(&near,COINCIDENCE,SNAP);
    let (d,_,_) = h.closest.expect("the near pair is reported");
    assert!((d-2e-3).abs() < 1e-12,"{}",h.report(SNAP));
    // but a vertex no triangle uses is not in the mesh at all
    let mut spare = square();
    spare.vertices.push([2e-3,0.,0.]);
    let h = gcs_core::solid::swept_boundary::hygiene(&spare,COINCIDENCE,SNAP);
    assert_eq!(h.closest,None,"an unused point says nothing about the mesh");
    assert_eq!((h.vertices,h.used),(5,4));
}

#[test]
fn a_window_names_what_stands_in_a_ball_and_whose_it_is() {
    use gcs_core::solid::swept_boundary::window;
    let mut m = square();
    m.vertices.push([1e-3,0.,0.]);          // v4, a whisker off the corner, another sheet's
    m.vertices.push([5.,5.,0.]);            // v5 and v6, far away
    m.vertices.push([6.,5.,0.]);
    m.triangles.push([4,1,2]); m.sheet.push(7);
    m.triangles.push([5,6,1]); m.sheet.push(9);
    let w = window(&m,[0.,0.,0.],0.01);
    // only what is in the ball, nearest first
    assert_eq!(w.vertices.iter().map(|n| n.vertex).collect::<Vec<_>>(),vec![0,4]);
    assert_eq!(w.vertices[0].distance,0.);
    // whose each one is, which is what an index alone never says
    assert_eq!(w.vertices[0].sheets,vec![0]);
    assert_eq!(w.vertices[1].sheets,vec![7],"the whisker is the other sheet's");
    assert!(w.vertices[0].boundary,"the square's corner stands on its rim");
    // the pair, its distance, and that nothing joins them: a whisker's signature
    assert_eq!(w.pairs.len(),1);
    let (a,b,d,uses) = w.pairs[0];
    assert_eq!((a,b),(0,4));
    assert!((d-1e-3).abs() < 1e-12,"{d}");
    assert_eq!(uses,0,"no edge joins them, which is what makes it a whisker and not an edge");
    // every triangle with a corner in the ball, whosever it is, and no other
    assert_eq!(w.triangles.iter().map(|(i,_,_)| *i).collect::<Vec<_>>(),vec![0,1,2]);
    // a ball with nothing in it says so rather than printing a heading
    assert!(window(&m,[9.,9.,9.],0.5).vertices.is_empty());
}

/// How many vertices of the mesh stand exactly at `p`.
fn at(mesh: &CutMesh,p: V3) -> usize { mesh.vertices.iter().filter(|v| **v == p).count() }

#[test]
fn a_cut_point_beside_a_vertex_a_cut_uses_leaves_it_where_it_is() {
    let (vertices,triangles) = cube();
    let mut mesh = CutMesh::new(vertices,triangles.clone(),vec![0;triangles.len()],1e-3,1e-2);
    // the chain crosses the face's diagonal at its middle, and uses the vertex made there
    mesh.cut_along(&[([0.25,0.,0.5],[0.;3]),([0.75,0.,0.5],[0.;3])],false).unwrap();
    assert_eq!(at(&mesh,[0.5,0.,0.5]),1,"the crossing of the diagonal is a vertex");
    // a later point within the snap of it takes it where it stands
    mesh.cut_along(&[([0.5+2e-4,0.,0.5],[0.;3])],false).unwrap();
    assert_eq!(at(&mesh,[0.5,0.,0.5]),1,"the vertex the first cut runs through did not move");
    assert_eq!(at(&mesh,[0.5+2e-4,0.,0.5]),0,"and the later point did not put one of its own there");
    // a vertex no cut uses still moves onto the point, so the seam is the sheet's own
    mesh.cut_along(&[([0.9998,0.,0.],[0.;3])],false).unwrap();
    assert_eq!(at(&mesh,[0.9998,0.,0.]),1,"an unpinned corner moves onto the point");
    assert_eq!(at(&mesh,[1.,0.,0.]),0);
}

/// Two strips meeting along y = 0, the lower one sampled twice as finely
/// there, each a boundary loop of its own until the junctions are split.
fn strips(offset: f64) -> KeptMesh {
    let vertices = vec![[0.,0.,0.],[2.,0.,0.],[2.,1.,0.],[0.,1.,0.],[0.,-1.,0.],[1.,-1.,0.],[2.,-1.,0.],[1.,offset,0.]];
    let triangles = vec![[0,1,2],[0,2,3],[4,5,7],[4,7,0],[5,6,1],[5,1,7]];
    KeptMesh {vertices,triangles,sheet:vec![0,0,1,1,1,1]}
}

#[test]
fn a_t_junction_on_a_shared_line_is_split_and_the_seam_closes() {
    let mut mesh = strips(0.);
    assert_eq!(boundary_loops(&mesh.triangles).len(),2);
    split_at_vertices(&mut mesh,1e-9);
    let loops = boundary_loops(&mesh.triangles);
    assert_eq!(loops.len(),1,"one outer boundary");
    assert_eq!(edge_uses(&mesh.triangles).values().filter(|n| **n == 1).count(),7);
}

#[test]
fn a_vertex_a_whisker_off_the_shared_line_splits_within_the_tolerance() {
    let mut mesh = strips(0.005);
    split_at_vertices(&mut mesh,1e-9);
    assert_eq!(boundary_loops(&mesh.triangles).len(),2,"exact coincidence does not see it");
    split_at_vertices(&mut mesh,0.03);
    assert_eq!(boundary_loops(&mesh.triangles).len(),1);
    // the vertex stays where it was
    assert_eq!(mesh.vertices[7],[1.,0.005,0.]);
}

/// A floor of two triangles with a wall of two triangles standing on it at
/// x = 1: the floor's fragments beyond the wall's foot go where the floor's
/// area says its region is.
fn floor_and_wall(floor_to: f64) -> KeptMesh {
    // floor z = 0 over x in [0, floor_to], y in [0, 1], facing +z; the wall
    // on x = 1 rising to z = 1, facing +x
    let vertices = vec![[0.,0.,0.],[floor_to,0.,0.],[floor_to,1.,0.],[0.,1.,0.],[1.,0.,0.],[1.,1.,0.],[1.,0.,1.],[1.,1.,1.]];
    let triangles = vec![[0,1,2],[0,2,3],[4,5,7],[4,7,6]];
    KeptMesh {vertices,triangles,sheet:vec![0,0,1,1]}
}

fn floor_area(out: &KeptMesh) -> f64 {
    area(&KeptMesh {vertices:out.vertices.clone(),triangles:out.triangles.iter().zip(&out.sheet).filter(|(_,s)| **s == 0).map(|(t,_)| *t).collect(),sheet:vec![]})
}

#[test]
fn a_floor_overhanging_a_wall_is_trimmed_to_the_wall() {
    let (out,_) = planar_union(&floor_and_wall(1.05),1e-9);
    assert!((floor_area(&out)-1.).abs() < 1e-9,"floor area {}",floor_area(&out));
    assert!(out.triangles.iter().zip(&out.sheet).filter(|(_,s)| **s == 0).all(|(t,_)| t.iter().all(|&v| out.vertices[v as usize][0] <= 1.+1e-9)));
    assert_eq!(out.sheet.iter().filter(|s| **s == 1).count(),2,"the wall is untouched");
}

#[test]
fn a_floor_extending_both_ways_past_a_wall_is_left_alone() {
    let (out,_) = planar_union(&floor_and_wall(2.),1e-9);
    assert!((floor_area(&out)-2.).abs() < 1e-9,"floor area {}",floor_area(&out));
}
