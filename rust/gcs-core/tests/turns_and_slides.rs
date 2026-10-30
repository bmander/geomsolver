//! A turn and a slide, the constant-twist class's two limits (#684 A and B,
//! docs/generating-sweeps.md): a tool turned about one axis, or slid along one line, has its
//! characteristic fixed on it as a screw's is, and its sheet is that curve carried — a turn's its
//! section in a half-plane through the axis revolved, a slide's its section square to the slide
//! extruded. A torus and a cylinder groove a rod slid along it, and a torus a ring sector turned
//! across it; what each removes is computed here again — by a circular segment and by Pappus where
//! the tool stands square, and tilted by its shadow, read from the torus's centre circle alone —
//! not by the code under test, and held to the exact solid this kernel builds, whose STL is held
//! to the field. The square torus slid and the leaning one turned are the series' own
//! (`solid_sweeps/slide.sv`, `turn.sv`, `solid_sweeps_examples.rs`). A V wheel slid tilted joins its
//! flanks' characteristics round its sharp rim's fan (#684 E), its shadow read here too; slid
//! square, or stood upright and turned, both flanks meet the rim at one point and the sheet is
//! creased along its path (#684 V), the section the V's own in closed form. Each row the two
//! motions add or meet refuses by name.
use gcs_core::brep::{export,props,sweep::Say};
use gcs_core::program::Elaborated;
use gcs_core::solid::admission::{self,Class,Condition,Error,Options};
use gcs_core::solid::export::Tolerance;
use gcs_core::space::{cross,unit};
use std::f64::consts::{PI,TAU};
use std::sync::Mutex;

type V = [f64;3];

/// What `f` returns, and the lines it said through the `Say` it is handed.
fn saying<T>(f: impl FnOnce(&Say) -> T) -> (T,String) {
    let lines = Mutex::new(Vec::<String>::new());
    let say = Say {stage:&|l: &str| lines.lock().unwrap().push(l.to_string()),mark:&|_| {}};
    let out = f(&say);
    (out,lines.into_inner().unwrap().join("\n"))
}

/// The solid `name` built exactly within 0.01 mm by this kernel, its STEP and STL written and the
/// STL held to the material field with no disagreement (where it cuts a sweep: a ball's tube is
/// static material, built as any is); its volume.
pub(crate) fn exported(e: &Elaborated,name: &str) -> f64 { exported_at(e,name,0.01) }

/// The same, built within `tolerance` millimetres.
pub(crate) fn exported_at(e: &Elaborated,name: &str,tolerance: f64) -> f64 {
    let (sk,body) = (&e.sketch,fixtures::solid(e,name));
    let tolerance = Some(Tolerance::new(tolerance).unwrap());
    let (exact,said) = saying(|say| export::exact(sk,body,None,tolerance,say));
    let exact = exact.unwrap_or_else(|r| panic!("{r}\n{said}"));
    eprintln!("{said}");
    exact.solid.check(1e-6).unwrap();
    let ((step,stl),said) = saying(|say| (export::step(&exact,"part",tolerance,say).unwrap(),
        export::stl(sk,body,&exact,tolerance,say).unwrap()));
    assert!(step.starts_with("ISO-10303-21;"));
    gcs_core::mesh::stl_shells(&stl).unwrap();
    assert!(!exact.swept || said.contains(", 0 disagree"),"{said}");
    props::volume(&exact.solid)
}

/// `volume` against `expected`, within `bar` of it.
pub(crate) fn near(volume: f64,expected: f64,bar: f64) {
    eprintln!("volume {volume:.9} against {expected:.9}");
    assert!((volume-expected).abs() <= bar*expected,"{volume} against {expected}");
}

pub(crate) fn admit(source: &str) -> Result<admission::Admission,Error> {
    let e = fixtures::read(source);
    admission::admit_body(&e.sketch,fixtures::solid(&e,"part"),&Options::default())
}

pub(crate) fn refused(result: Result<admission::Admission,Error>) -> admission::Refusal {
    match result {
        Err(Error::Refused(r)) => { eprintln!("{r}"); r }
        Err(Error::Unreadable(m)) => panic!("unreadable: {m}"),
        Ok(a) => panic!("admitted: {:?}",a.sweeps().iter().map(|s| &s.name).collect::<Vec<_>>()),
    }
}

/// The groove's admission: the constant-twist class, its motion read as a turn or a slide.
fn admitted_as(source: &str) -> gcs_core::motion::Screw {
    let a = admit(source).unwrap();
    let [s] = a.sweeps() else { panic!("one sweep") };
    assert_eq!(s.name,"groove");
    let Class::ConstantTwist(found) = &s.class else { panic!("`groove` is a constant twist's sweep") };
    assert!(s.contacts >= 8,"{} points across its reach",s.contacts);
    found.screw
}

// -- the slide -------------------------------------------------------------------------------

/// The rod a slide grooves: radius 5 about z, from z = 0 to 20.
const ROD: &str = "\
unit mm
use std
use std.place (coords)
use std.motion (slides)
in std.front {
  construction centerline ax := line(std.origin, hint((0, 1)))
  fix((0, 1)) ax.p2
  private r0 := point
  private r1 := point
  private r2 := point
  private r3 := point
  fix((0, 0)) r0
  fix((5, 0)) r1
  fix((5, 20)) r2
  fix((0, 20)) r3
  private rod_axis := line(r3, r0)
  private rod_end := line(r0, r1)
  private rod_wall := line(r1, r2)
  private rod_top := line(r2, r3)
  rod := solid(face(rod_axis, rod_end, rod_wall, rod_top), about: rod_axis)
}
";

/// What a slide grooves the rod with: a torus whose tube of radius 2 is centred `reach` from the
/// rod's axis, its own axis 6 beyond; a cylinder of radius 2 and length 20 about that line; or a
/// V wheel, its sharp rim `reach` from the rod's axis and 6 from its own, its flanks 30° either side
/// of its plane and 2.5 deep to its bore; or an engraving bit, a cone whose point is `reach` from
/// the rod's axis on its own, its flank 30° from it and 2.5 long to its back.
#[derive(Clone,Copy)]
enum Wheel { Torus,Cylinder,Vee,Bit }

/// The wheel drawn in the plane through (0, 0, -9) along x and along y tilted `tilt` degrees up
/// toward z, its axis along that y; slid 40 mm up the rod's axis over a turn of the roll, from
/// clear below the rod to clear above it.
fn slid(tilt: f64,reach: f64,wheel: Wheel) -> String {
    let (s,c) = tilt.to_radians().sin_cos();
    let drawn = match wheel {
        Wheel::Torus => format!("\
  private a0 := point
  private a1 := point
  a0 coords(du: {axis}mm, dv: -1mm) bench
  a1 coords(du: {axis}mm, dv: 1mm) bench
  private wheel_axis := line(a0, a1)
  private tc := point
  tc coords(du: {reach}mm, dv: 0mm) bench
  private tube := circle(center: tc) hint(r: 2)
  radius(2mm) tube
}}
construction wheel := solid(face(tube), about: wheel_axis)
",axis=reach+6.),
        Wheel::Cylinder => format!("\
  private c0 := point
  private c1 := point
  private c2 := point
  private c3 := point
  c0 coords(du: {reach}mm, dv: -10mm) bench
  c1 coords(du: {rim}mm, dv: -10mm) bench
  c2 coords(du: {rim}mm, dv: 10mm) bench
  c3 coords(du: {reach}mm, dv: 10mm) bench
  private cb := line(c0, c1)
  private cw := line(c1, c2)
  private ct := line(c2, c3)
  private ca := line(c3, c0)
}}
construction wheel := solid(face(cb, cw, ct, ca), about: ca)
",rim=reach+2.),
        Wheel::Vee => format!("\
  private a0 := point
  private a1 := point
  a0 coords(du: {axis}mm, dv: -1mm) bench
  a1 coords(du: {axis}mm, dv: 1mm) bench
  private wheel_axis := line(a0, a1)
  private v0 := point
  private v1 := point
  private v2 := point
  v0 coords(du: {reach}mm, dv: 0mm) bench
  v1 coords(du: {base}mm, dv: -{half}mm) bench
  v2 coords(du: {base}mm, dv: {half}mm) bench
  private lower := line(v0, v1)
  private bore := line(v1, v2)
  private upper := line(v2, v0)
}}
construction wheel := solid(face(lower, bore, upper), about: wheel_axis)
",axis=reach+6.,base=reach+2.5,half=2.5*(PI/6.).tan()),
        Wheel::Bit => format!("\
  private v0 := point
  private v1 := point
  private v2 := point
  v0 coords(du: {reach}mm, dv: 0mm) bench
  v1 coords(du: {base}mm, dv: {half}mm) bench
  v2 coords(du: {base}mm, dv: 0mm) bench
  private flank := line(v0, v1)
  private back := line(v1, v2)
  private bit_axis := line(v2, v0)
}}
construction wheel := solid(face(flank, back, bit_axis), about: bit_axis)
",base=reach+2.5,half=2.5*(PI/6.).tan()),
    };
    format!("{ROD}\
private bx := axis
fix(dir == (1, 0, 0)) bx
private by := axis
fix(dir == (0, {c}, {s})) by
bench := plane(u: bx, v: by)
fix(origin == (0, 0, -9)) bench
in bench {{
{drawn}feed := frame
feed slides(40mm) ax
construction groove := sweep(wheel over t in (0deg, 360deg), under: feed)
part := solid(rod)
groove cut part
")
}

/// A cylinder slid square to its axis, its flat caps (which the slide keeps in their own planes)
/// clear of the rod: its axis 5.5 from the rod's along y and its radius 2, its shadow along the
/// slide a band 4 wide, so it cuts the rod's circular segment beyond 3.5 from its axis, its wall
/// the nearer generator extruded, a plane.
#[test]
fn a_slid_cylinder_with_its_caps_clear_grooves_a_rod_by_its_section() {
    let source = slid(0.,5.5,Wheel::Cylinder);
    assert!(admitted_as(&source).is_slide(),"read as a slide");
    let segment = 25.*(0.7f64).acos()-3.5*(25.-3.5*3.5f64).sqrt();
    near(exported(&fixtures::read(&source),"part"),PI*25.*20.-segment*20.,1e-6);
}

/// Tilted 25 degrees, the torus's characteristic under the slide is no circle of it, and the
/// groove's section is the torus's shadow along the slide: every line up the rod passing within
/// the tube's radius of the torus's centre circle is cut.
#[test]
fn a_tilted_torus_slid_removes_its_shadow_along_the_slide() {
    let source = slid(25.,5.5,Wheel::Torus);
    admitted_as(&source);
    let volume = exported(&fixtures::read(&source),"part");
    // read about the rod's axis, where a run grows from nothing as the shadow's edge crosses the
    // rod's wall, not as a chord of it does
    let (s,c) = 25f64.to_radians().sin_cos();
    let torus = Torus::new([11.5,0.,-9.],[0.,c,s],6.,2.);
    let hit = |a: f64,r: f64| torus.within(&|q| (q[0]-r*a.cos()).hypot(q[1]-r*a.sin()));
    let shadow = measure([-PI,PI],&|_| [0.,5.],&hit,&|a,b| (b*b-a*a)/2.);
    // the shadow's measure reads to a few parts in a million at these rows (to 2e-7 of the solid
    // at four times as many)
    near(volume,PI*25.*20.-shadow*20.,1e-5);
}

/// A V wheel slid up the rod tilted 25 degrees, its sharp rim 4 from the rod's axis (#684 E): the
/// slide has a part along the wheel's axis, so each flank's characteristic meets the rim at its
/// own point and the characteristic runs between them round the rim's fan, the rim's own sweep
/// joining the flanks'. The groove's section is the wheel's shadow along the slide: every line up
/// the rod that passes into the V of the rim, read on each line by the signed distance from the
/// nearer flank (convex along it, so its least is found exactly); what the bore would add never
/// comes within 6 of the rod.
#[test]
fn a_tilted_v_wheel_slid_removes_its_shadow_round_its_rim() {
    let source = slid(25.,4.,Wheel::Vee);
    let a = admit(&source).unwrap();
    let [groove] = a.sweeps() else { panic!("one sweep") };
    let Class::ConstantTwist(found) = &groove.class else { panic!("`groove` is a constant twist's sweep") };
    assert!(found.screw.is_slide(),"read as a slide");
    assert_eq!(found.fans(),["the corner of `wheel.upper` and `wheel.lower`"]);
    let volume = exported(&fixtures::read(&source),"part");
    let (s,c) = 25f64.to_radians().sin_cos();
    let (axis,centre) = ([0.,c,s],[10.,0.,-9.]);
    let (sf,cf) = (PI/6.).sin_cos();
    let outside = |q: V| {
        let w: V = std::array::from_fn(|k| q[k]-centre[k]);
        let along = w[0]*axis[0]+w[1]*axis[1]+w[2]*axis[2];
        let rho = (0..3).map(|k| (w[k]-along*axis[k]).powi(2)).sum::<f64>().sqrt();
        along.abs()*cf+(rho-6.)*sf
    };
    let hit = |a: f64,r: f64| reaches(&|z| outside([r*a.cos(),r*a.sin(),z]),[-17.,-1.],64,0.);
    let shadow = measure([-PI,PI],&|_| [0.,5.],&hit,&|a,b| (b*b-a*a)/2.);
    near(volume,PI*25.*20.-shadow*20.,1e-5);
}

/// A V wheel slid up the rod square to its axis, its sharp rim 4 from the rod's axis (#684 V): the
/// slide runs along the rim's tangent where the rim is nearest the rod, so both flanks'
/// characteristics, the meridians of the wheel square to the slide, meet the rim there at one
/// point, every normal of the rim's fan in contact at it. The walk steps from the one flank
/// straight onto the other, and the groove's wall has a sharp edge along that point's path: the
/// sheet creased there, each flank's meridian slid a plane. The groove's section is the wheel's
/// profile in the plane through its axis, the wedge of half-angle 30° from (4, 0) toward the
/// wheel's axis within the rod's circle: two triangles to where its sides meet the circle and the
/// circle's segment beyond their chord, in closed form.
#[test]
fn a_v_wheel_slid_square_leaves_its_profile_creased_at_the_rim() {
    let source = slid(0.,4.,Wheel::Vee);
    let a = admit(&source).unwrap();
    let [groove] = a.sweeps() else { panic!("one sweep") };
    let Class::ConstantTwist(found) = &groove.class else { panic!("`groove` is a constant twist's sweep") };
    assert!(found.screw.is_slide(),"read as a slide");
    assert!(found.fans().is_empty(),"no fan walked: {:?}",found.fans());
    assert_eq!(found.sharp(),["the corner of `wheel.upper` and `wheel.lower`"]);
    assert_eq!(found.creases().len(),1);
    let (s,c) = (PI/6.).sin_cos();
    // where each side of the wedge meets the circle: (4 + t c)² + (t s)² = 25
    let t = -4.*c+(16.*c*c+9.).sqrt();
    let (x0,h) = (4.+t*c,t*s);
    let section = t*t*c*s+25.*(x0/5.).acos()-x0*h;
    near(exported(&fixtures::read(&source),"part"),PI*25.*20.-section*20.,1e-6);
}

/// The straight-fluted drill (`twist_drill/straight_flute.sv`): the twist drill's flute wheel set
/// square to the axis and slid up it. Under the slide the wheel's characteristic is its profile in
/// the plane through its axis square to the slide, so each flute's section is the wheel's drawn
/// profile within the stock — the round's disc, of radius 2.5 a web's half (0.75) off the axis,
/// with the wedge between its flanks, tangent to it and leaning 35 degrees — the same at every
/// height, both flutes carried the stock's 40 mm.
#[test]
fn the_straight_fluted_drill_is_its_wheels_profile_carried_up_the_slide() {
    let e = fixtures::read_beside(&std::fs::read_to_string(fixtures::drill::project().join("straight_flute.sv")).unwrap(),
        &fixtures::drill::project(),&mut |_,text| text);
    let a = admission::admit_body(&e.sketch,fixtures::solid(&e,"fluted"),&Options::default()).unwrap();
    let [s] = a.sweeps() else { panic!("one sweep") };
    let Class::ConstantTwist(found) = &s.class else { panic!("`flute` is a constant twist's sweep") };
    assert!(found.screw.is_slide(),"read as a slide: {:?}",found.screw);
    assert_eq!(s.placements.len(),2,"cut once a flute");
    // the wheel stands on −x, its profile read across (y) and out from the axis (−x)
    let (nearest,round,flank) = (0.75,2.5,35f64.to_radians());
    let (across,up) = (round*flank.cos(),nearest+round-round*flank.sin());
    let profile = |x: f64,y: f64| (x+nearest+round).hypot(y) <= round || (-x >= up && y.abs() <= across+(-x-up)*flank.tan());
    let hit = |a: f64,r: f64| profile(r*a.cos(),r*a.sin());
    let area = measure([-PI,PI],&|_| [0.,5.],&hit,&|a,b| (b*b-a*a)/2.);
    near(exported(&e,"fluted"),PI*25.*40.-2.*area*40.,1e-5);
}

// -- the turn --------------------------------------------------------------------------------

/// How a torus about (5, 0, 0) leans from upright by `tilt` degrees: across the turn, its axis
/// skew to z, or toward it in the front plane, its axis meeting z below.
#[derive(Clone,Copy)]
pub(crate) enum Lean { Across(f64),Toward(f64) }

impl Lean {
    /// The torus's axis, and the way out from it its tube's centre is drawn.
    fn axes(self) -> (V,V) {
        match self {
            Lean::Across(tilt) => { let (s,c) = tilt.to_radians().sin_cos(); ([0.,s,c],[1.,0.,0.]) }
            Lean::Toward(tilt) => { let (s,c) = tilt.to_radians().sin_cos(); ([s,0.,c],[c,0.,-s]) }
        }
    }
}

/// A sector of a ring about z, radii 8 to 12, heights -1 to 1, from x 60 degrees round (a
/// wedge's in common with the ring), and a torus about the line through (5, 0, 0) leaning so,
/// its tube of radius 2 centred 2.5 out from it, turned about z over the roll from `from` to `to`
/// degrees.
fn turned(lean: Lean,from: f64,to: f64) -> String { turned_in(lean,from,to,true) }

/// `turned`'s torus and roll over the whole ring (no wedge), or its sector.
fn turned_in(lean: Lean,from: f64,to: f64,sector: bool) -> String {
    let (a,o) = lean.axes();
    // the wedge the ring's sector is its common with
    let wedge = if !sector { String::new() } else { format!("\
in std.top {{
  private w0 := point
  private w1 := point
  private w2 := point
  fix((0, 0)) w0
  fix((20, 0)) w1
  fix((10, {wy})) w2
  private wa := line(w0, w1)
  private wb := line(w1, w2)
  private wc := line(w2, w0)
}}
construction wedge := solid(face(wa, wb, wc), from: -3mm, to: 3mm)
wedge common ring
",wy=20.*60f64.to_radians().sin()) };
    format!("\
unit mm
use std
use std.place (coords)
use std.motion (turns)
private tx := axis
fix(dir == ({o0}, {o1}, {o2})) tx
private ty := axis
fix(dir == ({a0}, {a1}, {a2})) ty
lathe := plane(u: tx, v: ty)
fix(origin == (5, 0, 0)) lathe
in lathe {{
  private ta := point
  private tb := point
  ta coords(du: 0mm, dv: -1mm) lathe
  tb coords(du: 0mm, dv: 1mm) lathe
  private taxis := line(ta, tb)
  private tc := point
  tc coords(du: 2.5mm, dv: 0mm) lathe
  private tube := circle(center: tc) hint(r: 2)
  radius(2mm) tube
}}
in std.front {{
  construction centerline ax := line(std.origin, hint((0, 1)))
  fix((0, 1)) ax.p2
  private q0 := point
  private q1 := point
  private q2 := point
  private q3 := point
  fix((8, -1)) q0
  fix((12, -1)) q1
  fix((12, 1)) q2
  fix((8, 1)) q3
  private inner := line(q3, q0)
  private bottom := line(q0, q1)
  private outer := line(q1, q2)
  private top := line(q2, q3)
}}
ring := solid(face(inner, bottom, outer, top), about: ax)
{wedge}construction wheel := solid(face(tube), about: taxis)
swing := frame
swing turns(1) ax
construction groove := sweep(wheel over t in ({from}deg, {to}deg), under: swing)
part := solid(ring)
groove cut part
",o0=o[0],o1=o[1],o2=o[2],a0=a[0],a1=a[1],a2=a[2])
}

/// The turned wheel's torus as drawn.
pub(crate) fn turned_torus(lean: Lean) -> Torus { Torus::new([5.,0.,0.],lean.axes().0,2.5,2.) }

/// What the turn removes from the sector: Pappus over the section of the ring the torus's
/// shadow in a half-plane through z covers, every circle about z that passes within the tube's
/// radius of the torus's centre circle.
pub(crate) fn turned_away(torus: &Torus) -> f64 {
    let hit = |z: f64,r: f64| torus.within(&|q| (q[0].hypot(q[1])-r).hypot(q[2]-z));
    PI/3.*measure([-1.,1.],&|_| [8.,12.],&hit,&|a,b| (b*b-a*a)/2.)
}

/// Square to the turn, the torus's shadow in a half-plane through z is its tube's circle pushed
/// out by its ring: the far wall stands at 7.5 + √(4 − z²), and the groove is Pappus's. The
/// shadow read from the torus's centre circle, which the leaning torus is held to, agrees.
#[test]
fn a_turned_torus_grooves_a_ring_by_pappus() {
    let upright = turned(Lean::Across(0.),-130.,130.);
    assert!(admitted_as(&upright).is_turn(),"read as a turn");
    let sector = PI/3.;
    let away = sector*crate::brep::simpson(-1.,1.,2000,|z| ((7.5+(4.-z*z).sqrt()).powi(2)-64.)/2.);
    let read = turned_away(&turned_torus(Lean::Across(0.)));
    assert!((read-away).abs() <= 1e-5*read,"the shadow's {read} against Pappus's {away}");
    near(exported(&fixtures::read(&upright),"part"),sector*(144.-64.)-away,2e-5);
}

/// `whole_turn.sv`'s V wheel stood upright, its axis parallel to the turn's (#684 V): the turn moves
/// each point square to the plane through it and the rod's axis, and a flank's normal lies in the
/// plane through it and the wheel's axis, so each flank touches along its meridian in the plane
/// through both axes, and the two meet the rim at one point, where it is nearest the rod. The walk
/// steps from the one flank straight onto the other, the sheet creased along that point's circle:
/// the groove is the V itself turned once round, the wedge `|z − 10| ≤ (r − 4) tan 30°` within the
/// rod, `4π tan 30° · 7/3` by Pappus.
#[test]
fn an_upright_v_wheel_turned_once_leaves_its_v_turned() {
    let e = fixtures::read(&crate::solid_sweeps_examples::text("whole_turn.sv").replace("lean := 20deg","lean := 0deg"));
    let Class::ConstantTwist(found) = crate::solid_sweeps_examples::class(&e,"rod") else { panic!("the constant-twist class") };
    assert!(found.screw.is_turn() && found.whole,"read as a whole turn");
    assert!(found.fans().is_empty(),"{:?}",found.fans());
    assert_eq!(found.sharp(),["the corner of `vwheel.upper` and `vwheel.lower`"]);
    near(exported(&e,"rod"),PI*25.*20.-4.*PI*(PI/6.).tan()*7./3.,1e-6);
}

// -- refusals --------------------------------------------------------------------------------

/// A whole turn (#684 N): the leaning torus turned once round the whole ring, from inside it, sweeps
/// a solid of revolution, its sheet the section revolved all the way round and closing on itself.
/// No end is clear of the ring, and none need be: the torus at either lies in what the turn
/// sweeps. What it removes is Pappus over the shadow all the way round, six times the sector's.
/// A roll of more than a whole turn passes the same path again: it is read once round from its
/// start, as one whole turn from there is.
#[test]
fn a_whole_turn_grooves_the_whole_ring() {
    let whole = |from: f64,to: f64| -> f64 {
        let source = turned_in(Lean::Across(25.),from,to,false);
        let a = admit(&source).unwrap();
        let [s] = a.sweeps() else { panic!("one sweep") };
        let Class::ConstantTwist(found) = &s.class else { panic!("`groove` is a constant twist's sweep") };
        assert!(found.screw.is_turn() && found.whole,"read as a whole turn");
        found.nodes[found.reach[0]].position[0]
    };
    assert_eq!(whole(-200.,520.),whole(-200.,160.),"two turns read as the one from their start");
    let away = 6.*turned_away(&turned_torus(Lean::Across(25.)));
    near(exported(&fixtures::read(&turned_in(Lean::Across(25.),0.,360.,false)),"part"),TAU*(144.-64.)-away,2e-5);
}

/// Each row a turn or a slide meets refuses by name: a roll that leaves the torus in the ring is not clear at its ends (E1);
/// a torus leaning toward the turn's axis, so that its own meets it, has a ring in contact all
/// round, every normal of it through where the axes meet, at which its characteristic branches
/// between two of the walk's stations (S1: the boundary leaves the walked branch there, which
/// the field gate saw before this row did); and a torus slid through the rod's middle touches it
/// all round its tube, a characteristic that closes on itself (refused so until #684 W), but at the
/// tube's top and bottom its rings are in contact all round, square to the slide, where the
/// characteristic branches into the groove's flat sides (S1). An engraving bit slid square
/// to its axis touches along the two lines of its cone square to the slide, which meet at its
/// point, on its axis: every normal of the cone round the point is in contact there, and the
/// groove's wall has a sharp edge along its path, which the class builds where the corner turns
/// to a circle (a V wheel's rim, #684 V, above) and not at an apex (S1).
#[test]
fn the_turn_and_slide_rows_refuse_by_name() {
    let short = refused(admit(&turned(Lean::Across(0.),-130.,30.)));
    assert_eq!(short.condition,Condition::Clearance);
    let branching = refused(admit(&turned(Lean::Toward(25.),-140.,140.)));
    assert_eq!(branching.condition,Condition::Regular,"{branching}");
    assert!(branching.message.contains("in contact all round"),"{}",branching.message);
    let closed = refused(admit(&slid(0.,0.,Wheel::Torus)));
    assert_eq!(closed.condition,Condition::Regular,"{closed}");
    assert!(closed.message.contains("`wheel.tube` is in contact all round a ring"),"{}",closed.message);
    let pointed = refused(admit(&slid(0.,4.,Wheel::Bit)));
    assert_eq!(pointed.condition,Condition::Regular,"{pointed}");
    assert!(pointed.message.contains("`wheel.flank` meets the tool's axis at an apex of its profile"),"{}",pointed.message);
}

// -- what the shadows are read by ------------------------------------------------------------

/// A torus: the points within `tube` of its centre circle, about `centre` square to `axis`, of
/// radius `ring`, read on the circle's own two directions across the axis.
pub(crate) struct Torus { centre: V,across: [V;2],ring: f64,tube: f64 }

impl Torus {
    fn new(centre: V,axis: V,ring: f64,tube: f64) -> Torus {
        let seed = if axis[0].abs() < 0.9 { [1.,0.,0.] } else { [0.,1.,0.] };
        let e1 = unit(cross(axis,seed));
        Torus {centre,across:[e1,cross(unit(axis),e1)],ring,tube}
    }
    /// The point of the centre circle at angle `a`.
    fn at(&self,a: f64) -> V {
        let [e1,e2] = self.across;
        std::array::from_fn(|k| self.centre[k]+self.ring*(a.cos()*e1[k]+a.sin()*e2[k]))
    }
    /// Whether `d` comes within the tube's radius anywhere on the centre circle: read four
    /// degrees apart (`reaches`).
    pub(crate) fn within(&self,d: &dyn Fn(V) -> f64) -> bool { reaches(&|a| d(self.at(a)),[0.,TAU],90,self.tube) }
}

/// Whether `f` comes within `bar` anywhere on [lo, hi]: read at `n` steps, the first reading
/// within it answering, and else each local least refined by golden section (one new reading a
/// step) over thirty steps, its bracket then a millionth of its first.
pub(crate) fn reaches(f: &dyn Fn(f64) -> f64,[lo,hi]: [f64;2],n: usize,bar: f64) -> bool {
    let at = |k: usize| lo+(hi-lo)*k as f64/n as f64;
    let mut values = Vec::with_capacity(n+1);
    for k in 0..=n {
        let v = f(at(k));
        if v <= bar { return true }
        values.push(v);
    }
    let g = (5f64.sqrt()-1.)/2.;
    (0..=n).filter(|&k| (k == 0 || values[k] <= values[k-1]) && (k == n || values[k] <= values[k+1])).any(|k| {
        let (mut a,mut b) = (at(k.saturating_sub(1)),at((k+1).min(n)));
        let (mut x,mut y) = (b-g*(b-a),a+g*(b-a));
        let (mut fx,mut fy) = (f(x),f(y));
        for _ in 0..30 {
            if fx < fy { b = y; y = x; fy = fx; x = b-g*(b-a); fx = f(x) } else { a = x; x = y; fx = fy; y = a+g*(b-a); fy = f(y) }
            if fx.min(fy) <= bar { return true }
        }
        false
    })
}

/// ∫ over `u` of the sum of `along(a, b)` over the runs [a, b] of `v` within `span(u)` where
/// `hit(u, v)` holds: each row read at 100 points with each run's ends bisected, the rows by
/// Simpson's rule over 1000 intervals, read on every core.
pub(crate) fn measure(u: [f64;2],span: &(dyn Fn(f64) -> [f64;2]+Sync),hit: &(dyn Fn(f64,f64) -> bool+Sync),along: &(dyn Fn(f64,f64) -> f64+Sync)) -> f64 {
    let rows = 1000;
    let h = (u[1]-u[0])/rows as f64;
    let row = |x: f64| -> f64 {
        let [lo,hi] = span(x);
        if !(hi > lo) { return 0. }
        let n = 100;
        let at = |k: usize| lo+(hi-lo)*k as f64/n as f64;
        let inside: Vec<bool> = (0..=n).map(|k| hit(x,at(k))).collect();
        // where the run's edge is between `a`, on the side `was`, and `b`
        let edge = |mut a: f64,mut b: f64,was: bool| { for _ in 0..40 { let m = 0.5*(a+b); if hit(x,m) == was { a = m } else { b = m } } 0.5*(a+b) };
        let mut sum = 0.;
        let mut start = if inside[0] { Some(lo) } else { None };
        for k in 1..=n {
            match (inside[k-1],inside[k]) {
                (false,true) => start = Some(edge(at(k-1),at(k),false)),
                (true,false) => { sum += along(start.take().unwrap(),edge(at(k-1),at(k),true)) }
                _ => {}
            }
        }
        if let Some(a) = start { sum += along(a,hi) }
        sum
    };
    let values: Vec<f64> = gcs_core::par::indices(rows+1,|k| row(u[0]+k as f64*h));
    values.iter().enumerate().map(|(k,v)| v*if k == 0 || k == rows { 1. } else if k%2 == 1 { 4. } else { 2. }).sum::<f64>()*h/3.
}

