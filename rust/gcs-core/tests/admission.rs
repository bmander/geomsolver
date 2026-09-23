//! The generating-sweep class (`solid::admission`, docs/generating-sweeps.md): which sweeps
//! are admitted, and that every refusal names the row it fails. Small fixtures cut a post with
//! the sweep-mesh tools under a relative roll, the motion a generator uses; the gear pair is
//! the full-size case, at the bevel, at its configured hypoid and at an offset it leaves the
//! class by undercut.
use crate::sweep_mesh::{harness,tools,motions};
use gcs_core::solid::admission::{self,Condition,Error,Options};

/// A post of radius 0.4 about the vertical line x = 3, z in [-2, 2], the blank every small
/// fixture cuts: the tools start centred on it and a roll carries them round the spindle.
const POST: &str = "private point q0 hint(x: 3, y: -2)
private point q1 hint(x: 3.4, y: -2)
private point q2 hint(x: 3.4, y: 2)
private point q3 hint(x: 3, y: 2)
ground q0
ground q1
ground q2
ground q3
private line qb(q0, q1)
private line qw(q1, q2)
private line qt(q2, q3)
private line qa(q3, q0)
construction solid post(face(qb, qw, qt, qa), about: qa)
solid part(post)
removal cut part
";

/// A tool, a motion named `motion`, the removal over [from, to] and the post it is cut from.
fn document(tool: &str,motion: &str,name: &str,from: f64,to: f64) -> String {
    format!("{tool}{motion}construction solid removal(tool, under: {name}, from: {from}deg, to: {to}deg)\n{POST}")
}

fn admit(source: &str) -> Result<admission::Admission,Error> {
    let e = harness::read(source);
    admission::admit_body(&e.sketch,harness::solid(&e,"part"),&Options::default())
}

fn refused(result: Result<admission::Admission,Error>) -> admission::Refusal {
    match result {
        Err(Error::Refused(r)) => { eprintln!("{r}"); r }
        Err(Error::Unreadable(m)) => panic!("unreadable: {m}"),
        Ok(a) => panic!("admitted: {a:?}"),
    }
}

/// A torus about the vertical line through (3, 0), its tube of radius 0.5 centred 1 from that
/// axis at height 2: a profile that never meets its axis, as a cutter's does not.
const TORUS: &str = "unit mm
use std
construction centerline line spindle(std.origin, std.up.toward)
private point ta hint(x: 3, y: 1)
private point tb hint(x: 3, y: 3)
ground ta
ground tb
private line taxis(ta, tb)
private point tc hint(x: 4, y: 2)
ground tc
private circle ring(center: tc) hint(r: 0.5)
radius(0.5mm) ring
construction solid tool(face(ring), about: taxis)
";

/// The tool spinning about its own vertical axis through (3, 0), seen from an observer turning
/// about world x: crossed axes, as a generator's are.
const CROSSED_ROLL: &str = "private point hub hint(x: 3, y: 0)
hub distance(3mm, along: u) std.front
hub distance(0mm, along: v) std.front
private point hub_up hint(x: 3, y: 5)
hub_up distance(3mm, along: u) std.front
hub_up distance(5mm, along: v) std.front
construction centerline line own(hub, hub_up)
private point xend hint(x: 5, y: 0)
xend distance(5mm, along: u) std.front
xend distance(0mm, along: v) std.front
construction centerline line xaxis(std.origin, xend)
private motion spin(about: own, ratio: 0.25)
private motion observer(about: xaxis)
motion turn(spin, relative_to: observer)
";

/// A generator's roll: the tool carried about a vertical cradle axis through (2, 0), 1 off its
/// own (a spin about its own axis would change nothing of a revolution), seen from an observer
/// turning about the line parallel to world x through (0, 0.5, 0.5), skew to the cradle's.
const SKEW_ROLL: &str = "private point hub hint(x: 2, y: 0)
hub distance(2mm, along: u) std.front
hub distance(0mm, along: v) std.front
private point hub_up hint(x: 2, y: 5)
hub_up distance(2mm, along: u) std.front
hub_up distance(5mm, along: v) std.front
construction centerline line cradle(hub, hub_up)
private point xend hint(x: 5, y: 0)
xend distance(5mm, along: u) std.front
xend distance(0mm, along: v) std.front
private plane flat(origin: std.origin, toward: xend, u: (1, 0, 0), v: (0, 1, 1))
in flat {
  private point k0 hint(x: 0, y: 0.7071)
  private point k1 hint(x: 5, y: 0.7071)
  k0 distance(0mm, along: u) flat
  k0 distance(0.7071mm, along: v) flat
  k1 distance(5mm, along: u) flat
  k1 distance(0.7071mm, along: v) flat
  construction centerline line kaxis(k0, k1)
}
private motion spin(about: cradle, ratio: 0.25)
private motion observer(about: kaxis)
motion turn(spin, relative_to: observer)
";

/// With the spin axis meeting the observer's, a surface of revolution's contact equation has
/// no constant term, and where its amplitude passes through zero a ring of the torus is in
/// contact at every time: refused, found between samples (or inside a sample cell).
#[test]
fn a_ring_in_contact_at_every_time_is_refused() {
    let source = format!("{TORUS}{CROSSED_ROLL}construction solid removal(tool, under: turn, from: -60deg, to: 60deg)\n{}",
        POST.replace("x: 3,","x: 4,").replace("x: 3.4,","x: 4.4,"));
    let r = refused(admit(&source));
    assert_eq!(r.condition,Condition::Stationary);
    assert!(r.message.contains("at every time"),"{}",r.message);
}

/// The post under the torus's cradle roll: at x = 4, z in [0.5, 2], clear of the torus at both
/// limits of a ±75° roll and of the torus turned on past them.
fn skew_post(x: f64) -> String {
    POST.replace("x: 3,",&format!("x: {x},")).replace("x: 3.4,",&format!("x: {},",x+0.4)).replace("y: -2)","y: 0.5)")
}

#[test]
fn a_torus_rolled_about_a_skew_axis_through_a_post_is_admitted() {
    let source = format!("{TORUS}{SKEW_ROLL}construction solid removal(tool, under: turn, from: -75deg, to: 75deg)\n{}",skew_post(4.));
    let a = admit(&source).unwrap();
    let s = &a.sweeps[0];
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
    let r = refused(admit(&document(tools::SPHERE,&motions::roll(0.25),"turn",-60.,60.)));
    assert_eq!(r.condition,Condition::Stationary);
    assert!(r.message.contains("pole"),"{}",r.message);
}

/// A tool that never meets the blank is no cut: refused, not admitted vacuously.
#[test]
fn a_sweep_that_reaches_nothing_is_refused() {
    let source = format!("{TORUS}{SKEW_ROLL}construction solid removal(tool, under: turn, from: -75deg, to: 75deg)\n{}",skew_post(13.));
    let r = refused(admit(&source));
    assert_eq!(r.condition,Condition::Reach);
}

#[test]
fn a_single_rotation_is_refused_as_stationary() {
    let r = refused(admit(&document(tools::SPHERE,motions::TURN_SPINDLE,"turn",-60.,60.)));
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
    let r = refused(admit(&document(tools::SPHERE,&motions::slide_x(8.),"feed",-60.,60.)));
    assert_eq!(r.condition,Condition::Motion);
}

#[test]
fn a_roll_that_starts_in_the_blank_is_refused() {
    let r = refused(admit(&document(tools::SPHERE,&motions::roll(0.25),"turn",-10.,60.)));
    assert_eq!(r.condition,Condition::Clearance);
    let p = r.witness.unwrap();
    assert!((p[0]-3.).hypot(p[1]) < 0.4+1e-6 && p[2].abs() <= 2.,"the witness is in the post: {p:?}");
}

#[test]
fn a_union_tool_is_refused() {
    let tool = format!("{}construction solid twin(tool, under: observer, at: 20deg)\nconstruction solid lump(tool)\ntwin on lump\n",tools::SPHERE);
    let source = format!("{tool}{}construction solid removal(lump, under: turn, from: -60deg, to: 60deg)\n{POST}",motions::roll(0.25));
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

/// The gear pair at a design (offset, pressure shift, crown spiral, in degrees), its pinion
/// with a blank of every index: the full-size case.
fn pinion(offset: f64,shift: f64,spiral: f64) -> Result<admission::Admission,Error> {
    let base = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../examples/spiral_bevel");
    let source = std::fs::read_to_string(base.join("gears.sv")).unwrap();
    let set = ["param offset_angle","param pressure_shift","param spiral_angle"];
    let mut design = |name: &str,text: String| if name == "configuration" {
        text.lines().filter(|l| !set.iter().any(|s| l.starts_with(s))).map(|l| format!("{l}\n")).collect::<String>()
            + &format!("param offset_angle = {offset}deg\nparam pressure_shift = {shift}deg\nparam spiral_angle = {spiral}deg\n")
    } else { text };
    let mut resolver = harness::directory_resolver(&base,&mut design);
    let e = harness::read_resolving(&source,&mut resolver);
    let started = std::time::Instant::now();
    let result = admission::admit_body(&e.sketch,harness::solid(&e,"pair.pinion.body"),&Options::default());
    eprintln!("pinion at {offset}/{shift}/{spiral}: {:?}",started.elapsed());
    result
}

#[test]
fn the_bevel_pinion_is_admitted_once_for_every_index() {
    let a = pinion(0.,0.,35.).unwrap();
    let s = &a.sweeps[0];
    eprintln!("{} samples, {} contacts, spacing {:.4}, least J {:.3}",s.samples,s.contacts,s.spacing,s.least_area_factor);
    assert_eq!(s.placements.len(),24);
    assert_eq!(s.placements.iter().filter(|p| p.equivalent_to.is_none()).count(),1,
        "the blank is a revolution about the indexing axis, so one placement's checks serve all");
}

#[test]
fn the_configured_hypoid_pinion_is_admitted() {
    let a = pinion(25.,10.,25.).unwrap();
    assert!(a.sweeps[0].least_area_factor > 0.1);
}

#[test]
fn a_hypoid_pinion_with_undercut_is_refused() {
    let r = refused(pinion(30.,0.,35.));
    assert!(matches!(r.condition,Condition::Single | Condition::Fold | Condition::Crossing),"{r}");
    assert_eq!(r.sweep,"pair.pinion.removal");
}

/// At 20 degrees with a symmetric rack no tool point touches the blank twice; the flank's
/// generated surface folds, and the fold is what refuses it. Split 7.5 degrees, it does not.
#[test]
fn a_folding_flank_is_refused_and_balanced_pressure_angles_admit_it() {
    let r = refused(pinion(20.,0.,35.));
    assert!(matches!(r.condition,Condition::Fold | Condition::Crossing),"{r}");
    pinion(20.,7.5,35.).unwrap();
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
    let e = harness::read(tools::SPHERE);
    let mut material = MaterialField::read(&e.sketch,harness::solid(&e,"tool"),1e-10).unwrap().evaluator(4096);
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
    let e = harness::read(tools::TRIANGLE_PRISM);
    let mut material = MaterialField::read(&e.sketch,harness::solid(&e,"tool"),1e-10).unwrap().evaluator(4096);
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
    let source = format!("{TORUS}{SKEW_ROLL}construction solid removal(tool, under: turn, from: -75deg, to: 75deg)\n{}",skew_post(4.));
    let e = harness::read(&source);
    let field = gcs_core::solid::MaterialField::read(&e.sketch,harness::solid(&e,"part"),1e-10).unwrap();
    let mut material = field.evaluator(4096);
    let (mut decided,mut agreed) = (0,0);
    for i in 0..12 { for j in 0..12 { for k in 0..12 {
        let p = [3.5+0.1*i as f64,-0.55+0.1*j as f64,0.4+0.15*k as f64];
        let value = field.side(p);
        let bounds = material.bounds_stopping(p.map(|x| Interval::point(x).unwrap()),Stop::Outside(Interval::ZERO),
            minimum::Options {value_tolerance:1e-9,max_evaluations:20000}).unwrap().value.bounds();
        if bounds[0] > 0. || bounds[1] < 0. {
            decided += 1;
            if (value > 0.) == (bounds[0] > 0.) { agreed += 1; } else { eprintln!("{p:?}: {value} against {bounds:?}"); }
        }
    } } }
    eprintln!("{agreed} of {decided} decided points agree");
    assert!(decided > 1000);
    assert_eq!(agreed,decided);
}
