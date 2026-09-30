//! The generating-sweep class (`solid::admission`, docs/generating-sweeps.md): which sweeps
//! are admitted, and that every refusal names the row it fails. Small fixtures cut a post with
//! the sweep-mesh tools under a relative roll, the motion a generator uses; the gear pair is
//! the full-size case, at the bevel, at its configured hypoid and at an offset it leaves the
//! class by undercut.
use fixtures::{tools::{self,centred_post,skew_post,sphere,torus},motions::{self,CROSSED_ROLL,Observer,cradle_roll}};
use gcs_core::solid::admission::{self,Condition,Error,Options};

/// A tool, a motion named `motion`, the removal over [from, to] and the post it is cut from.
fn document(tool: &str,motion: &str,name: &str,from: f64,to: f64) -> String {
    format!("{tool}{motion}construction solid removal(tool, under: {name}, from: {from}deg, to: {to}deg)\n{}",centred_post())
}

fn admit(source: &str) -> Result<admission::Admission,Error> {
    let e = fixtures::read(source);
    admission::admit_body(&e.sketch,fixtures::solid(&e,"part"),&Options::default())
}

fn refused(result: Result<admission::Admission,Error>) -> admission::Refusal {
    match result {
        Err(Error::Refused(r)) => { eprintln!("{r}"); r }
        Err(Error::Unreadable(m)) => panic!("unreadable: {m}"),
        Ok(a) => panic!("admitted: {a:?}"),
    }
}




/// With the spin axis meeting the observer's, a surface of revolution's contact equation has
/// no constant term, and where its amplitude passes through zero a ring of the torus is in
/// contact at every time: refused, found between samples (or inside a sample cell).
#[test]
fn a_ring_in_contact_at_every_time_is_refused() {
    let source = format!("{}{CROSSED_ROLL}construction solid removal(tool, under: turn, from: -60deg, to: 60deg)\n{}",
        torus(2.),tools::post(4.,0.4,-2.,2.));
    let r = refused(admit(&source));
    assert_eq!(r.condition,Condition::Stationary);
    assert!(r.message.contains("at every time"),"{}",r.message);
}


#[test]
fn a_torus_rolled_about_a_skew_axis_through_a_post_is_admitted() {
    let source = format!("{}{}construction solid removal(tool, under: turn, from: -75deg, to: 75deg)\n{}",torus(2.),cradle_roll(0.25,Observer::Skew),skew_post(4.));
    let a = admit(&source).unwrap();
    let s = &a.sweeps()[0];
    eprintln!("{} samples, {} contacts, spacing {:.4}, least J {:.3}, {} near double roots, {} near tangent",
        s.samples,s.contacts,s.spacing,s.least_area_factor,s.near_double_roots,s.near_tangent_pairs);
    assert_eq!(s.name,"removal");
    assert!(s.contacts > 100,"the sweep reaches the post");
    assert_eq!(s.placements.len(),1);
}

/// With the spin and the observer's axes parallel, a sphere's poles (on its spin axis, normals
/// along it) are in contact at every time, and one passes through the post.
#[test]
fn a_pole_in_contact_at_every_time_is_refused() {
    let r = refused(admit(&document(&sphere(0.),&motions::roll(0.25),"turn",-60.,60.)));
    assert_eq!(r.condition,Condition::Stationary);
    assert!(r.message.contains("pole"),"{}",r.message);
}

/// A tool that never meets the blank is no cut: refused, not admitted vacuously.
#[test]
fn a_sweep_that_reaches_nothing_is_refused() {
    let source = format!("{}{}construction solid removal(tool, under: turn, from: -75deg, to: 75deg)\n{}",torus(2.),cradle_roll(0.25,Observer::Skew),skew_post(13.));
    let r = refused(admit(&source));
    assert_eq!(r.condition,Condition::Reach);
}

#[test]
fn a_single_rotation_is_refused_as_stationary() {
    let r = refused(admit(&document(&sphere(0.),motions::TURN_SPINDLE,"turn",-60.,60.)));
    assert_eq!(r.condition,Condition::Stationary);
    assert!(r.witness.is_some());
}

#[test]
fn a_prism_tool_is_refused() {
    let r = refused(admit(&document(tools::BOX,&motions::roll(0.25),"turn",-60.,60.)));
    assert_eq!(r.condition,Condition::Tool);
    assert!(r.message.contains("prism"),"{}",r.message);
}

#[test]
fn a_translation_is_refused() {
    let r = refused(admit(&document(&sphere(0.),&motions::slide_x(8.),"feed",-60.,60.)));
    assert_eq!(r.condition,Condition::Motion);
}

#[test]
fn a_roll_that_starts_in_the_blank_is_refused() {
    let r = refused(admit(&document(&sphere(0.),&motions::roll(0.25),"turn",-10.,60.)));
    assert_eq!(r.condition,Condition::Clearance);
    let p = r.witness.unwrap();
    assert!((p[0]-3.).hypot(p[1]) < 0.4+1e-6 && p[2].abs() <= 2.,"the witness is in the post: {p:?}");
}

#[test]
fn a_union_tool_is_refused() {
    let tool = format!("{}construction solid twin(tool, under: observer, at: 20deg)\nconstruction solid lump(tool)\ntwin on lump\n",sphere(0.));
    let source = format!("{tool}{}construction solid removal(lump, under: turn, from: -60deg, to: 60deg)\n{}",motions::roll(0.25),centred_post());
    let r = refused(admit(&source));
    assert_eq!(r.condition,Condition::Tool);
    assert!(r.message.contains("adds solids"),"{}",r.message);
}

/// An L-shaped profile revolved about x = 3: its corner at (3.5, 0) turns inward.
const ELL: &str = "unit mm
use std
construction centerline line spindle(std.origin, std.up.toward)
private point l0 hint(x: 3, y: -1)
private point l1 hint(x: 4, y: -1)
private point l2 hint(x: 4, y: 0)
private point l3 hint(x: 3.5, y: 0)
private point l4 hint(x: 3.5, y: 1)
private point l5 hint(x: 3, y: 1)
ground l0
ground l1
ground l2
ground l3
ground l4
ground l5
private line e0(l0, l1)
private line e1(l1, l2)
private line e2(l2, l3)
private line e3(l3, l4)
private line e4(l4, l5)
private line axis(l5, l0)
construction solid tool(face(e0, e1, e2, e3, e4, axis), about: axis)
";

#[test]
fn a_concave_corner_carried_through_the_blank_is_refused() {
    let r = refused(admit(&document(ELL,&motions::roll(0.25),"turn",-60.,60.)));
    assert_eq!(r.condition,Condition::Corner);
    let p = r.witness.unwrap();
    assert!((p[0]-3.).hypot(p[1]) < 0.4+1e-6,"the witness is in the post: {p:?}");
}

/// The gear pair at a design (the offset between the shafts in millimetres, the pressure shift
/// and the crown's spiral in degrees), its pinion with a blank of every index: the full-size case.
fn pinion(offset: f64,shift: f64,spiral: f64) -> Result<admission::Admission,Error> {
    member("pinion",offset,shift,spiral)
}

/// `pinion`, or the gear.
fn member(member: &str,offset: f64,shift: f64,spiral: f64) -> Result<admission::Admission,Error> {
    fabricated(member,[offset,shift,spiral],0.,0.,0.)
}

/// `member` with the fabrication allowances: the normal backlash, the tip relief and the end
/// relief in millimetres. A tip-relieved member has two sweeps, its crown's and its relief's; the
/// end relief is two more static cuts of its blank.
fn fabricated(member: &str,design: [f64;3],backlash: f64,relief: f64,ends: f64) -> Result<admission::Admission,Error> {
    let e = fixtures::gear::read_configured_with(&mut |name,text|
        fixtures::gear::fabricated(name,text,design,backlash,relief,ends));
    let started = std::time::Instant::now();
    let body = fixtures::solid(&e,&format!("pair.{member}.body"));
    let result = admission::admit_body(&e.sketch,body,&Options::default());
    eprintln!("{member} at {design:?}, backlash {backlash}, relief {relief}, ends {ends}: {:?}",started.elapsed());
    result
}

#[test]
fn the_bevel_pinion_is_admitted_once_for_every_index() {
    let a = pinion(0.,0.,35.).unwrap();
    let s = &a.sweeps()[0];
    eprintln!("{} samples, {} contacts, spacing {:.4}, least J {:.3}",s.samples,s.contacts,s.spacing,s.least_area_factor);
    assert_eq!(s.placements.len(),24);
    assert_eq!(s.placements.iter().filter(|p| p.equivalent_to.is_none()).count(),1,
        "the blank is a revolution about the indexing axis, so one placement's checks serve all");
    assert!(matches!(s.equivalence,admission::Equivalence::Revolved {..}),"proved by the solid graph: {:?}",s.equivalence);
}

/// A ring about the spindle cut at six indices: its blank a revolution about the line every
/// placement turns about, so one placement's checks stand for all by the solid graph; the same ring
/// bounded by a box that holds it whole is the same blank, but no revolution of the graph, and its
/// placements are compared by sampling — and found alike. The verdicts are the same.
#[test]
fn an_indexed_ring_is_alike_by_its_revolution_and_a_boxed_ring_by_sampling() {
    let ring = |extra: &str| format!("{}{}construction solid removal(tool, under: turn, from: -35deg, to: 35deg)\n{}{extra}",
        sphere(2.),cradle_roll(2.5,Observer::Parallel),tools::indexed_ring(6,3.5,4.2,1.7,2.3));
    let boxed = "private point b0 hint(x: -5, y: 1)
private point b1 hint(x: 5, y: 1)
private point b2 hint(x: 5, y: 3)
private point b3 hint(x: -5, y: 3)
ground b0
ground b1
ground b2
ground b3
private line bb(b0, b1)
private line bw(b1, b2)
private line bt(b2, b3)
private line ba(b3, b0)
construction solid holder(face(bb, bw, bt, ba), from: -5mm, to: 5mm)
holder bound part
";
    let [revolved,sampled] = [ring(""),ring(boxed)].map(|source| admit(&source).unwrap());
    let (a,b) = (&revolved.sweeps()[0],&sampled.sweeps()[0]);
    assert!(matches!(a.equivalence,admission::Equivalence::Revolved {..}),"{:?}",a.equivalence);
    assert_eq!(b.equivalence,admission::Equivalence::Sampled);
    for s in [a,b] {
        assert_eq!(s.placements.len(),6);
        assert_eq!(s.placements.iter().filter(|p| p.equivalent_to.is_none()).count(),1);
    }
    assert_eq!((a.samples,a.contacts,a.near_double_roots,a.near_tangent_pairs),(b.samples,b.contacts,b.near_double_roots,b.near_tangent_pairs));
    assert_eq!((a.spacing,a.least_area_factor),(b.spacing,b.least_area_factor));
}

/// The configured hypoid pinion, its backlash, tip relief and end relief in: the crown's sweep and
/// the relief's are both in the class, each with margin.
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, about 14 s: the configured pinion's two sweeps admitted")]
fn the_configured_hypoid_pinion_is_admitted() {
    let a = fabricated("pinion",[25.,12.5,25.],0.05,0.2,0.2).unwrap();
    assert_eq!(a.sweeps().len(),2);
    assert!(a.sweeps().iter().all(|s| s.least_area_factor > 0.1));
    assert!(a.sweeps().iter().all(|s| matches!(s.equivalence,admission::Equivalence::Revolved {..})),
        "the blank (its end spheres balls about the axis) proved alike under every index by the solid graph");
}

#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, about 14 s: the configured gear's two sweeps admitted")]
fn the_configured_hypoid_gear_is_admitted() {
    let a = fabricated("gear",[25.,12.5,25.],0.05,0.2,0.2).unwrap();
    assert_eq!(a.sweeps().len(),2);
    assert!(a.sweeps().iter().all(|s| s.least_area_factor > 0.1));
    assert!(a.sweeps().iter().all(|s| matches!(s.equivalence,admission::Equivalence::Revolved {..})),
        "the blank (its end spheres balls about the axis) proved alike under every index by the solid graph");
}

#[test]
fn a_hypoid_pinion_with_undercut_is_refused() {
    let r = refused(pinion(30.,0.,35.));
    assert!(matches!(r.condition,Condition::Single | Condition::Fold | Condition::Crossing),"{r}");
    assert_eq!(r.sweep,"pair.pinion.removal");
}

/// At 20 mm of offset with a symmetric rack no tool point touches the blank twice; the flank's
/// generated surface folds, and the fold is what refuses it. Split 7.5 degrees, it does not.
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, about 22 s: admission of two gear designs")]
fn a_folding_flank_is_refused_and_balanced_pressure_angles_admit_it() {
    let r = refused(pinion(20.,0.,35.));
    assert!(matches!(r.condition,Condition::Fold | Condition::Crossing),"{r}");
    pinion(20.,7.5,35.).unwrap();
}

/// A tool, not a check: the pair's designs against the class, for choosing one inside it with
/// margin. `SOLVENT_GRID='25/12.5/25;20/12.5/25/0.05/0.2' cargo test -p gcs-core --test core
/// admission::the_admission_grid -- --ignored --nocapture` (each design offset/shift/spiral, the
/// offset in millimetres, the rest in degrees, then optionally the backlash, the tip relief and the
/// end relief in millimetres, zero when left out) prints each member's verdict and margins, one line per sweep.
/// The tables it chose the configured design from are in docs/spiral-bevel-layout-plan.md.
#[test]
#[ignore = "a tool: SOLVENT_GRID names the designs to admit"]
fn the_admission_grid() {
    let grid = std::env::var("SOLVENT_GRID").expect("SOLVENT_GRID");
    for design in grid.split(';').filter(|d| !d.trim().is_empty()) {
        let v: Vec<f64> = design.split('/').map(|x| x.trim().parse().unwrap()).collect();
        let [backlash,relief,ends] = [3,4,5].map(|i| v.get(i).copied().unwrap_or(0.));
        for which in ["pinion","gear"] {
            let started = std::time::Instant::now();
            let verdicts = match fabricated(which,[v[0],v[1],v[2]],backlash,relief,ends) {
                Ok(a) => a.sweeps().iter().map(|s| format!("admitted\t{}\tleast J {:.3}, {} near double roots, \
                    {} near tangent, {} contacts",s.name,s.least_area_factor,s.near_double_roots,s.near_tangent_pairs,
                    s.contacts)).collect(),
                Err(Error::Refused(r)) => vec![format!("refused\t{}\t{} {}",r.sweep,r.condition.code(),r.message.replace('\n'," "))],
                Err(Error::Unreadable(m)) => vec![format!("unreadable\t\t{m}")],
            };
            for verdict in verdicts {
                println!("grid\t{}\t{}\t{}\t{backlash}\t{relief}\t{ends}\t{which}\t{verdict}\t{:.1} s",v[0],v[1],v[2],
                    started.elapsed().as_secs_f64());
            }
        }
    }
}

/// A UV sphere about (3, 0, 0), wound outward unless `inward`.
fn sphere_mesh(radius: f64,inward: bool) -> (Vec<[f64;3]>,Vec<[u32;3]>) {
    let (m,n) = (96,48);
    let mut vertices = vec![[3.,0.,-radius],[3.,0.,radius]];
    for i in 1..n { for j in 0..m {
        let (theta,phi) = (std::f64::consts::PI*i as f64/n as f64,std::f64::consts::TAU*j as f64/m as f64);
        vertices.push([3.+radius*theta.sin()*phi.cos(),radius*theta.sin()*phi.sin(),-radius*theta.cos()]);
    }}
    let at = |i: usize,j: usize| (2+(i-1)*m+j%m) as u32;
    let mut triangles = Vec::new();
    for j in 0..m {
        triangles.push([0,at(1,j+1),at(1,j)]);
        triangles.push([1,at(n-1,j),at(n-1,j+1)]);
        for i in 1..n-1 {
            triangles.push([at(i,j),at(i,j+1),at(i+1,j+1)]);
            triangles.push([at(i,j),at(i+1,j+1),at(i+1,j)]);
        }
    }
    if inward { for t in &mut triangles { t.swap(1,2); } }
    (vertices,triangles)
}

#[test]
fn a_mesh_agrees_with_its_field_only_where_it_lies_on_the_boundary() {
    use gcs_core::solid::{agreement,MaterialField};
    let e = fixtures::read(&sphere(0.));
    let mut material = MaterialField::read(&e.sketch,fixtures::solid(&e,"tool"),1e-10).unwrap().evaluator(4096);
    let options = agreement::Options::default();
    let (v,t) = sphere_mesh(1.,false);
    let good = agreement::of_triangles(&v,&t,&mut material,&options).unwrap();
    eprintln!("true sphere: {good:?}");
    assert!(good.agrees() && good.unresolved == 0 && good.probed_triangles > 500);
    let (v,t) = sphere_mesh(1.3,false);
    let large = agreement::of_triangles(&v,&t,&mut material,&options).unwrap();
    assert_eq!(large.disagreements.len(),large.probed_triangles,"every inner probe is outside the true sphere");
    assert!(large.disagreements.iter().all(|d| d.inside_mesh));
    let (v,t) = sphere_mesh(1.,true);
    let inverted = agreement::of_triangles(&v,&t,&mut material,&options).unwrap();
    assert_eq!(inverted.disagreements.len(),2*inverted.probed_triangles);
}

/// The triangular prism tool as a closed mesh wound outward, each side cut in `k` strips so
/// centroids come near its edges. Its profile's page coordinates are world x and z (corners
/// (3, -0.8), (4.5, 0), (3, 0.8)) and it is extruded along world y over [-1.5, 1.5].
fn prism_mesh(k: usize) -> (Vec<[f64;3]>,Vec<[u32;3]>) {
    let corners = [[3.,-0.8],[4.5,0.],[3.,0.8]];
    let (mut vertices,mut triangles): (Vec<[f64;3]>,Vec<[u32;3]>) = (Vec::new(),Vec::new());
    for side in 0..3 {
        let (a,b) = (corners[side],corners[(side+1)%3]);
        for i in 0..k {
            let at = |f: f64,z: f64| [a[0]+(b[0]-a[0])*f,a[1]+(b[1]-a[1])*f,z];
            let n = vertices.len() as u32;
            let (f0,f1) = (i as f64/k as f64,(i+1) as f64/k as f64);
            vertices.extend([at(f0,-1.5),at(f1,-1.5),at(f1,1.5),at(f0,1.5)]);
            triangles.extend([[n,n+1,n+2],[n,n+2,n+3]]);
        }
    }
    let n = vertices.len() as u32;
    for z in [-1.5,1.5] { for c in corners { vertices.push([c[0],c[1],z]); } }
    triangles.extend([[n,n+2,n+1],[n+3,n+4,n+5]]);
    // Built with the profile in x-y and the extrusion along z; swapping y and z is a
    // reflection, so the winding turns too.
    for p in &mut vertices { p.swap(1,2); }
    for t in &mut triangles { t.swap(1,2); }
    (vertices,triangles)
}

#[test]
fn a_probe_through_another_face_is_withdrawn_nearer_the_mesh() {
    use gcs_core::solid::{agreement,MaterialField};
    let e = fixtures::read(tools::TRIANGLE_PRISM);
    let mut material = MaterialField::read(&e.sketch,fixtures::solid(&e,"tool"),1e-10).unwrap().evaluator(4096);
    let (v,t) = prism_mesh(40);
    // Wide enough that an inner probe beside the 56-degree edge at x = 4.5 leaves through
    // the other side; asked again at 0.02 it is inside.
    let options = agreement::Options {offset:0.3,confirm:0.02,triangles:t.len(),..Default::default()};
    let report = agreement::of_triangles(&v,&t,&mut material,&options).unwrap();
    eprintln!("{} probes, {} unresolved, {} withdrawn, {} disagree",report.probes,report.unresolved,report.withdrawn,report.disagreements.len());
    assert!(report.withdrawn > 0,"some probes cross the other side");
    assert!(report.agrees());
    // Shifted 0.05 outward, the same mesh disagrees, and nearer the mesh the more surely.
    let moved: Vec<[f64;3]> = v.iter().map(|p| [p[0]+0.05,p[1],p[2]]).collect();
    let shifted = agreement::of_triangles(&moved,&t,&mut material,&agreement::Options {offset:0.03,confirm:0.01,
        triangles:t.len(),..Default::default()}).unwrap();
    assert!(!shifted.agrees());
}

/// The plain floating-point side of a swept material agrees with the certified
/// enclosure wherever that enclosure decides one, over a grid through the cut post.
#[test]
fn a_material_side_agrees_with_its_enclosure() {
    use gcs_core::interval::{Interval,minimum::{self,Stop}};
    let source = format!("{}{}construction solid removal(tool, under: turn, from: -75deg, to: 75deg)\n{}",torus(2.),cradle_roll(0.25,Observer::Skew),skew_post(4.));
    let e = fixtures::read(&source);
    let field = gcs_core::solid::MaterialField::read(&e.sketch,fixtures::solid(&e,"part"),1e-10).unwrap();
    let mut material = field.evaluator(4096);
    let (mut decided,mut agreed) = (0,0);
    for i in 0..12 { for j in 0..12 { for k in 0..12 {
        let p = [3.5+0.1*i as f64,-0.55+0.1*j as f64,0.4+0.15*k as f64];
        let value = field.side(p);
        let bounds = material.query(p.map(|x| Interval::point(x).unwrap()),Stop::Outside(Interval::ZERO),
            minimum::Options {value_tolerance:1e-9,max_evaluations:20000},None).unwrap().value.bounds();
        if bounds[0] > 0. || bounds[1] < 0. {
            decided += 1;
            if (value > 0.) == (bounds[0] > 0.) { agreed += 1; } else { eprintln!("{p:?}: {value} against {bounds:?}"); }
        }
    } } }
    eprintln!("{agreed} of {decided} decided points agree");
    assert!(decided > 1000);
    assert_eq!(agreed,decided);
}

/// An indexed body's field read a sector at a time (`agreement::Sector`): every point turned into
/// one sector and read by the cuts not proved clear of its box decides as the whole field does where
/// it stands — its side, and whether it lies within a band of the boundary — over a grid of points
/// in and about the ring cut at six indices, and fewer cuts are read than there are.
#[test]
fn an_indexed_field_read_a_sector_at_a_time_decides_as_the_whole_field() {
    use gcs_core::interval::{Interval,minimum::{self,Stop}};
    use gcs_core::solid::{agreement,MaterialField};
    let source = format!("{}{}construction solid removal(tool, under: turn, from: -35deg, to: 35deg)\n{}",sphere(2.),
        cradle_roll(2.5,Observer::Parallel),tools::indexed_ring(6,3.5,4.2,1.7,2.3));
    let e = fixtures::read(&source);
    let body = fixtures::solid(&e,"part");
    let admitted = admission::admit_body(&e.sketch,body,&Options::default()).unwrap();
    let admission::Equivalence::Revolved {origin,axis} = admitted.sweeps()[0].equivalence else { panic!("the ring is a revolution") };
    let field = MaterialField::read(&e.sketch,body,1e-10).unwrap();
    // points about the axis at every angle, over the ring's radii and heights and a little past them
    let e1 = { let c = [axis[1],-axis[0],0.]; let n = (c[0]*c[0]+c[1]*c[1]).sqrt(); if n > 0.5 { c.map(|x| x/n) } else { [0.,axis[2],-axis[1]] } };
    let e2 = [axis[1]*e1[2]-axis[2]*e1[1],axis[2]*e1[0]-axis[0]*e1[2],axis[0]*e1[1]-axis[1]*e1[0]];
    let mut points = Vec::new();
    for a in 0..36 { for r in 0..8 { for h in 0..6 {
        let (angle,radius,height) = (std::f64::consts::TAU*(a as f64+0.37)/36.,3.35+0.13*r as f64,1.6+0.16*h as f64);
        points.push(std::array::from_fn::<f64,3,_>(|k| origin[k]+axis[k]*height+radius*(angle.cos()*e1[k]+angle.sin()*e2[k])));
    } } }
    let options = agreement::Options::default();
    let indexed = agreement::Indexed {origin,axis,count:6};
    let sector = agreement::Sector::new(&field,indexed,&points,4096,&options).unwrap().expect("the ring reads alike under its turn");
    eprintln!("{:?}",sector.folding);
    assert!(sector.folding.kept < sector.folding.operands,"some cuts are proved clear of every box");
    let mut state = sector.state();
    let mut whole = field.evaluator(4096);
    let band = Interval::new(-options.confirm,options.confirm).unwrap();
    let (mut sides,mut bands) = ([0;3],[0;2]);
    for &p in &points {
        for (stop,tolerance) in [(Stop::Outside(Interval::ZERO),options.value_tolerance),(Stop::Decided(band),options.confirm/4.)] {
            let full = whole.query(p.map(|x| Interval::point(x).unwrap()),stop,minimum::Options {value_tolerance:tolerance,
                max_evaluations:options.max_evaluations},None).unwrap().value.bounds();
            let read = sector.read(&mut state,p,stop,tolerance).unwrap();
            let side = |b: [f64;2]| if b[0] > 0. { 0 } else if b[1] < 0. { 1 } else { 2 };
            let within = |b: [f64;2]| b[0] >= -options.confirm && b[1] <= options.confirm;
            assert_eq!(side(full),side(read),"{p:?}: {full:?} against {read:?}");
            assert_eq!(within(full),within(read),"{p:?}: {full:?} against {read:?}");
            if matches!(stop,Stop::Outside(_)) { sides[side(full)] += 1; } else { bands[usize::from(within(full))] += 1; }
        }
    }
    eprintln!("sides (outside, inside, undecided) {sides:?}; within the band {bands:?}; {} read where they stand",state.whole);
    assert!(sides[0] > 100 && sides[1] > 100 && bands[1] > 0);
}
