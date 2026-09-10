//! Bodies with swept cuts through mesh arrangement and material classification.
#[path="../src/cad/manifold.rs"]
#[allow(dead_code)]
mod manifold;
mod support;

#[test]
fn manifold_round_trips_a_cube_and_subtracts_a_box() {
    let cube = |lo: [f64;3],hi: [f64;3]| {
        let v: Vec<[f64;3]> = (0..8).map(|i| [if i&1 == 0 { lo[0] } else { hi[0] },if i&2 == 0 { lo[1] } else { hi[1] },if i&4 == 0 { lo[2] } else { hi[2] }]).collect();
        // Outward-wound faces of the box.
        let t = vec![[0,2,1],[1,2,3],[4,5,6],[5,7,6],[0,1,4],[1,5,4],[2,6,3],[3,6,7],[0,4,2],[2,4,6],[1,3,5],[3,7,5]];
        manifold::Solid::from_triangles(&v,&t).unwrap()
    };
    let a = cube([0.;3],[2.;3]);
    assert!((a.volume()-8.).abs() < 1e-12);
    let b = cube([1.,1.,-1.],[3.,3.,3.]);
    let d = a.difference(&b).unwrap();
    assert!((d.volume()-6.).abs() < 1e-12);
    let (inside,outside) = a.split(&b).unwrap();
    assert!((inside.volume()-2.).abs() < 1e-12 && (outside.volume()-6.).abs() < 1e-12);
    let (v,t) = d.triangles().unwrap();
    let back = manifold::Solid::from_triangles(&v,&t).unwrap();
    assert!((back.volume()-6.).abs() < 1e-12);
    let moved = a.placed(&[1.,0.,0.,10., 0.,1.,0.,0., 0.,0.,1.,0.]).unwrap();
    assert!(moved.intersection(&a).unwrap().is_empty());
    assert_eq!(a.components().unwrap().len(),1);
}

#[path="../src/cad/mesh_sweep.rs"]
#[allow(dead_code)]
mod mesh_sweep;

#[test]
fn the_pinion_blank_meshes_as_a_closed_manifold() {
    let base = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../examples/spiral_bevel");
    let e = support::read(&std::fs::read_to_string(base.join("gears.sv")).unwrap(),&base);
    let body = e.map.ent_named("pair.pinion.body").unwrap().i();
    let started = std::time::Instant::now();
    let blank = gcs_core::solid::static_solid(&e.sketch,body,24).unwrap();
    eprintln!("static solid: {} primitives in {:?}, unit {}",blank.csg.prims.len(),started.elapsed(),blank.unit);
    let started = std::time::Instant::now();
    let mesh = mesh_sweep::solid_of(&blank).unwrap();
    eprintln!("manifold blank: {:.6} mm^3, {} triangles in {:?}",mesh.volume(),mesh.triangle_count(),started.elapsed());
    assert!((mesh.volume()-12038.99).abs() < 0.002*12038.99);
    assert!(blank.contains([54.,0.,0.]) || !blank.contains([0.;3]));
}

#[test]
fn a_slab_of_a_flat_grid_is_a_closed_box() {
    let (rows,columns) = (4,6);
    let mut points = Vec::new(); let mut normals = Vec::new();
    for r in 0..rows { for c in 0..columns { points.push([c as f64,r as f64,0.]); normals.push([0.,0.,1.]); } }
    let grid = mesh_sweep::SheetGrid {points,normals,rows,columns,closed_rows:false};
    let slab = mesh_sweep::slab(&grid,0.01).unwrap();
    assert!((slab.volume()-5.*3.*0.01).abs() < 1e-12,"{}",slab.volume());
}

#[test]
fn interior_points_of_the_blank_are_inside_it() {
    let base = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../examples/spiral_bevel");
    let e = support::read(&std::fs::read_to_string(base.join("gears.sv")).unwrap(),&base);
    let body = e.map.ent_named("pair.pinion.body").unwrap().i();
    let blank = gcs_core::solid::static_solid(&e.sketch,body,24).unwrap();
    let mesh = mesh_sweep::solid_of(&blank).unwrap();
    for depth in [0.003,0.03,0.3] {
        let p = mesh_sweep::interior_point(&mesh,depth).unwrap();
        eprintln!("depth {depth}: {p:?} inside {}",blank.contains(p));
        assert!(blank.contains(p));
    }
    let (v,t) = mesh.triangles().unwrap();
    // Signed volume from the returned winding must be positive.
    let mut six = 0.;
    for tri in &t { let [a,b,c] = tri.map(|i| v[i as usize]); six += a[0]*(b[1]*c[2]-b[2]*c[1])-a[1]*(b[0]*c[2]-b[2]*c[0])+a[2]*(b[0]*c[1]-b[1]*c[0]); }
    eprintln!("signed volume {}",six/6.);
    assert!(six > 0.);
}

#[cfg(feature="occt")]
#[path="../src/cad/native.rs"]
#[allow(dead_code)]
mod native;

/// Both members through the mesh arrangement, against the volumes the kernel
/// path recorded. The mesh path tessellates the blank and chords the sheet, so
/// the agreement is to a tenth of a percent, not to the kernel's tolerance.
#[cfg(feature="occt")]
fn whole_member(member: &str,expected: f64) {
    let base = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../examples/spiral_bevel");
    let e = support::read(&std::fs::read_to_string(base.join("gears.sv")).unwrap(),&base);
    let volume = member_volume(&e,member);
    assert!((volume-expected).abs() < 1e-3*expected,"{member}: {volume} against {expected}");
}

/// One member through the mesh arrangement, its signed volume from the STL's own winding.
#[cfg(feature="occt")]
fn member_volume(e: &gcs_core::program::Elaborated,member: &str) -> f64 {
    let body = e.map.ent_named(&format!("pair.{member}.body")).unwrap().i();
    let session = native::Session::new().unwrap();
    let scale = e.sketch.units.length.unwrap().1;
    let sheets = |swept: usize,inside: &dyn Fn(&[[f64;3]]) -> Result<Vec<bool>,String>| {
        let sheet = native::sweep_boundary::swept_sheet_grid(&session,&e.sketch,swept,inside)?;
        Ok(vec![mesh_sweep::SheetGrid {points:sheet.points.iter().map(|p| p.map(|v| v/scale)).collect(),
            normals:sheet.normals.clone(),rows:sheet.rows,columns:sheet.columns,closed_rows:false}])
    };
    let started = std::time::Instant::now();
    let (vertices,triangles) = mesh_sweep::construct(&e.sketch,body,&sheets).unwrap();
    let bytes = mesh_sweep::stl(&vertices,&triangles,member).unwrap();
    let mut six = 0.;
    for t in &triangles { let [a,b,c] = t.map(|i| vertices[i as usize]); six += a[0]*(b[1]*c[2]-b[2]*c[1])-a[1]*(b[0]*c[2]-b[2]*c[0])+a[2]*(b[0]*c[1]-b[1]*c[0]); }
    let volume = six/6.;
    eprintln!("{member}: {volume:.6} mm^3, {} triangles, {} bytes in {:?}",triangles.len(),bytes.len(),started.elapsed());
    volume
}

/// The pair as configured stands the pinion axis off the gear's: the pinion is
/// still one crown tooth swept through its blank at every index. Its blank is a
/// body of revolution about its own axis, so every placement of the sheet cuts,
/// and the offset is what makes it a hypoid rather than a bevel member.
#[cfg(feature="occt")]
#[test]
fn the_hypoid_pinion_exports_with_every_placement_cutting() {
    let base = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../examples/spiral_bevel");
    let source = std::fs::read_to_string(base.join("gears.sv")).unwrap();
    let hypoid = support::read_as_configured(&source,&base);
    let offset = hypoid.map.ent_named("pair.reference.pinion_axis").map(|l| {
        let line = &hypoid.sketch.lines[l.i()]; hypoid.sketch.world_point(line.p1 as usize)[1].abs() }).unwrap();
    assert!(offset > 1.,"the configured pair is a hypoid: axis offset {offset} mm");
    let volume = member_volume(&hypoid,"pinion");
    let bevel = member_volume(&support::read(&source,&base),"pinion");
    eprintln!("hypoid pinion {volume:.3} mm^3, bevel pinion {bevel:.3} mm^3");
    assert!((volume-bevel).abs() > 10.,"the offset changes the pinion: {volume} vs {bevel}");
}

#[cfg(feature="occt")]
#[test]
fn the_pinion_exports_through_the_mesh_path() { whole_member("pinion",9142.079); }

#[cfg(feature="occt")]
#[test]
fn the_gear_exports_through_the_mesh_path() { whole_member("gear",20284.173); }

const SPHERE_TOOL: &str = "unit mm
use std
construction centerline line spindle(std.origin, std.up.toward)
private point center
center distance(3mm, along: u) std.front
center distance(0mm, along: v) std.front
private point bottom hint(x: 3, y: -1)
private point top hint(x: 3, y: 1)
private line diameter(bottom, top)
center midpoint diameter
diameter parallel spindle
distance(2mm) diameter
private arc meridian(center: center, start: bottom, end: top)
radius(1mm) meridian
construction solid tool(face(meridian, diameter), about: diameter)
";
const BEAD: &str = "
private point bc
bc distance(3.8mm, along: u) std.front
bc distance(0mm, along: v) std.front
private point bb hint(x: 3.8, y: -1)
private point bt hint(x: 3.8, y: 1)
private line bd(bb, bt)
bc midpoint bd
bd parallel spindle
distance(2mm) bd
private arc bm(center: bc, start: bb, end: bt)
radius(1mm) bm
construction solid bead(face(bm, bd), about: bd)
solid part(bead)
removal cut part
";

/// Volume and membership of a constructed body against a closed form over a
/// box, with points within `skin` of the closed form's boundary skipped.
fn check_closed_form(vertices: &[[f64;3]],triangles: &[[u32;3]],inside: &dyn Fn([f64;3]) -> f64,lo: [f64;3],hi: [f64;3],skin: f64) {
    let mut six = 0.;
    for t in triangles { let [a,b,c] = t.map(|i| vertices[i as usize]); six += a[0]*(b[1]*c[2]-b[2]*c[1])-a[1]*(b[0]*c[2]-b[2]*c[0])+a[2]*(b[0]*c[1]-b[1]*c[0]); }
    let volume = six/6.;
    let n = 120;
    let mut count = 0_u64;
    for i in 0..n { for j in 0..n { for k in 0..n {
        let p: [f64;3] = std::array::from_fn(|a| lo[a]+(hi[a]-lo[a])*([i,j,k][a] as f64+0.5)/n as f64);
        if inside(p) < 0. { count += 1; }
    } } }
    let estimate = count as f64/(n*n*n) as f64*(hi[0]-lo[0])*(hi[1]-lo[1])*(hi[2]-lo[2]);
    eprintln!("volume {volume:.5} against closed-form quadrature {estimate:.5}");
    assert!((volume-estimate).abs() < 0.01*estimate,"volume {volume} vs {estimate}");
    let solid = manifold::Solid::from_triangles(vertices,triangles).unwrap();
    assert!((solid.volume()-volume).abs() < 1e-9);
    // Membership by ray parity against the mesh, at withheld points.
    let mut checked = 0;
    let m = 12;
    for i in 0..m { for j in 0..m { for k in 0..m {
        let p: [f64;3] = std::array::from_fn(|a| lo[a]+(hi[a]-lo[a])*([i,j,k][a] as f64+0.37)/m as f64);
        let f = inside(p);
        if f.abs() < skin { continue; }
        let mut crossings = 0;
        let d = [0.3127,0.7231,0.6129];
        for t in triangles {
            let [a,b,c] = t.map(|i| vertices[i as usize]);
            let (e1,e2) = ([b[0]-a[0],b[1]-a[1],b[2]-a[2]],[c[0]-a[0],c[1]-a[1],c[2]-a[2]]);
            let h = [d[1]*e2[2]-d[2]*e2[1],d[2]*e2[0]-d[0]*e2[2],d[0]*e2[1]-d[1]*e2[0]];
            let det = e1[0]*h[0]+e1[1]*h[1]+e1[2]*h[2];
            if det.abs() < 1e-12 { continue; }
            let s = [p[0]-a[0],p[1]-a[1],p[2]-a[2]];
            let u = (s[0]*h[0]+s[1]*h[1]+s[2]*h[2])/det; if !(0. ..=1.).contains(&u) { continue; }
            let q = [s[1]*e1[2]-s[2]*e1[1],s[2]*e1[0]-s[0]*e1[2],s[0]*e1[1]-s[1]*e1[0]];
            let v = (d[0]*q[0]+d[1]*q[1]+d[2]*q[2])/det; if v < 0. || u+v > 1. { continue; }
            let t = (e2[0]*q[0]+e2[1]*q[1]+e2[2]*q[2])/det; if t > 0. { crossings += 1; }
        }
        assert_eq!(crossings%2 == 1,f < 0.,"mesh and closed form disagree at {p:?}");
        checked += 1;
    } } }
    eprintln!("{checked} withheld points agree with the closed form");
    assert!(checked > 500);
}

#[test]
fn a_sphere_swept_past_a_bead_carves_a_torus_bite() {
    let source = format!("{SPHERE_TOOL}motion turn(about: spindle)\nconstruction solid removal(tool, under: turn, from: -60deg, to: 60deg)\n{BEAD}");
    let e = support::read(&source,std::path::Path::new("."));
    let part = e.map.ent_named("part").unwrap().i();
    let started = std::time::Instant::now();
    let (vertices,triangles) = mesh_sweep::construct(&e.sketch,part,&|_,_| Err("no kernel sheets in this test".into())).unwrap();
    eprintln!("torus bite in {:?}, {} triangles",started.elapsed(),triangles.len());
    // Inside the bead (radius 1 about (3.8,0,0)) and outside the tube (ring
    // radius 3 about z, tube radius 1), which the bead's angular span keeps
    // within the swept sector.
    let inside = |p: [f64;3]| ((p[0]-3.8).hypot(p[1]).hypot(p[2])-1.).max(1.-(p[0].hypot(p[1])-3.).hypot(p[2]));
    check_closed_form(&vertices,&triangles,&inside,[2.8,-1.,-1.],[4.8,1.,1.],0.03);
}

#[test]
fn a_cylinder_plunging_through_a_bead_bores_it() {
    let source = format!("unit mm
use std
construction centerline line spindle(std.origin, std.up.toward)
private point c0 hint(x: 3, y: -1)
private point c1 hint(x: 4, y: -1)
private point c2 hint(x: 4, y: 1)
private point c3 hint(x: 3, y: 1)
ground c0
ground c1
ground c2
ground c3
private line bottom(c0, c1)
private line wall(c1, c2)
private line top(c2, c3)
private line axis(c3, c0)
construction solid tool(face(bottom, wall, top, axis), about: axis)
motion plunge(along: axis, advance: 6mm)
construction solid removal(tool, under: plunge, from: 0deg, to: 360deg)
{BEAD}");
    let e = support::read(&source,std::path::Path::new("."));
    let part = e.map.ent_named("part").unwrap().i();
    let started = std::time::Instant::now();
    let (vertices,triangles) = mesh_sweep::construct(&e.sketch,part,&|_,_| Err("no kernel sheets in this test".into())).unwrap();
    eprintln!("bored bead in {:?}, {} triangles",started.elapsed(),triangles.len());
    // The tool starts at z in [-1,1] and travels 6 along the axis (world z),
    // so the bead, within |z| < 1, is bored by a cylinder of radius 1 about x=3.
    let inside = |p: [f64;3]| ((p[0]-3.8).hypot(p[1]).hypot(p[2])-1.).max(1.-(p[0]-3.).hypot(p[1]));
    check_closed_form(&vertices,&triangles,&inside,[2.8,-1.,-1.],[4.8,1.,1.],0.03);
}
