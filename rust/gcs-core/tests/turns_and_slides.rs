//! A turn and a slide, the constant-twist class's two limits (#684 A and B,
//! docs/generating-sweeps.md): a tool turned about one axis, or slid along one line, has its
//! characteristic fixed on it as a screw's is, and its sheet is that curve carried — a turn's its
//! section in a half-plane through the axis revolved, a slide's its section square to the slide
//! extruded. A torus grooves a rod slid along it and a ring sector turned across it; what each
//! removes is computed here again — by the lens two discs share and by Pappus where the torus
//! stands square, and tilted by its shadow, read from its centre circle alone — not by the code
//! under test, and held to the exact solid this kernel builds, whose STL is held to the field.
//! Each row the two motions add or meet refuses by name.
use gcs_core::brep::{export,props,sweep::Say};
use gcs_core::model::Sketch;
use gcs_core::program::Elaborated;
use gcs_core::solid::admission::{self,Class,Condition,Error,Options};
use gcs_core::solid::export::Tolerance;
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
pub(crate) fn exported(e: &Elaborated,name: &str) -> f64 {
    let (sk,body): (&Sketch,usize) = (&e.sketch,fixtures::solid(e,name));
    let tolerance = Some(Tolerance::new(0.01).unwrap());
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

fn admit(source: &str) -> Result<admission::Admission,Error> {
    let e = fixtures::read(source);
    admission::admit_body(&e.sketch,fixtures::solid(&e,"part"),&Options::default())
}

fn refused(result: Result<admission::Admission,Error>) -> admission::Refusal {
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

/// The wheel a slide grooves the rod with: a torus whose axis runs through (11.5, 0, -9) along y
/// tilted `tilt` degrees up toward z, its tube of radius 2 centred 6 in toward the rod; slid 40 mm
/// up the rod's axis over a turn of the roll, from clear below the rod to clear above it.
fn slid(tilt: f64) -> String {
    let (s,c) = tilt.to_radians().sin_cos();
    format!("{ROD}\
private bx := axis
fix(dir == (1, 0, 0)) bx
private by := axis
fix(dir == (0, {c}, {s})) by
bench := plane(u: bx, v: by)
fix(origin == (0, 0, -9)) bench
in bench {{
  private a0 := point
  private a1 := point
  a0 coords(du: 11.5mm, dv: -1mm) bench
  a1 coords(du: 11.5mm, dv: 1mm) bench
  construction centerline wheel_axis := line(a0, a1)
  private tc := point
  tc coords(du: 5.5mm, dv: 0mm) bench
  private tube := circle(center: tc) hint(r: 2)
  radius(2mm) tube
}}
construction wheel := solid(face(tube), about: wheel_axis)
feed := frame
feed slides(40mm) ax
construction groove := sweep(wheel over t in (0deg, 360deg), under: feed)
part := solid(rod)
groove cut part
")
}

/// The slid wheel's torus as drawn: its centre circle and its tube.
fn slid_torus(tilt: f64) -> Torus {
    let (s,c) = tilt.to_radians().sin_cos();
    Torus {centre:[11.5,0.,-9.],axis:[0.,c,s],ring:6.,tube:2.}
}

/// The area two discs of radii `a` and `b`, centres `d` apart, have in common.
pub(crate) fn lens(a: f64,b: f64,d: f64) -> f64 {
    let alpha = ((d*d+a*a-b*b)/(2.*d*a)).acos();
    let beta = ((d*d+b*b-a*a)/(2.*d*b)).acos();
    a*a*alpha+b*b*beta-0.5*((-d+a+b)*(d+a-b)*(d-a+b)*(d+a+b)).sqrt()
}

/// A torus slid square to its axis cuts the rod by its tube's section, a disc of radius 2 whose
/// centre stands 5.5 from the rod's axis: the groove is the lens the two discs share carried up
/// the rod's 20 mm, and its walls are the tube's circle extruded.
#[test]
fn a_slid_torus_grooves_a_rod_by_its_tubes_section() {
    let slide = admitted_as(&slid(0.));
    assert!(slide.ratio == 0. && !slide.is_turn(),"read as a slide: {slide:?}");
    let volume = exported(&fixtures::read(&slid(0.)),"part");
    let expected = PI*25.*20.-lens(5.,2.,5.5)*20.;
    eprintln!("volume {volume:.9} against {expected:.9}");
    assert!((volume-expected).abs() <= 1e-6*expected,"{volume} against {expected}");
}

/// A cylinder slid square to its axis, its flat caps (which the slide keeps in their own planes)
/// clear of the rod: its axis 5.5 from the rod's along y and its radius 2, its shadow along the
/// slide a band 4 wide, so it cuts the rod's circular segment beyond 3.5 from its axis, its wall
/// the nearer generator extruded, a plane.
#[test]
fn a_slid_cylinder_with_its_caps_clear_grooves_a_rod_by_its_section() {
    let source = slid(0.).replace("  private tc := point\n  tc coords(du: 5.5mm, dv: 0mm) bench\n  private tube := circle(center: tc) hint(r: 2)\n  radius(2mm) tube\n",
        "  private c0 := point\n  private c1 := point\n  private c2 := point\n  private c3 := point\n  c0 coords(du: 5.5mm, dv: -10mm) bench\n  \
        c1 coords(du: 7.5mm, dv: -10mm) bench\n  c2 coords(du: 7.5mm, dv: 10mm) bench\n  c3 coords(du: 5.5mm, dv: 10mm) bench\n  \
        private cb := line(c0, c1)\n  private cw := line(c1, c2)\n  private ct := line(c2, c3)\n  private ca := line(c3, c0)\n")
        .replace("du: 11.5mm","du: 5.5mm").replace("construction wheel := solid(face(tube), about: wheel_axis)","construction wheel := solid(face(cb, cw, ct, ca), about: ca)");
    let slide = admitted_as(&source);
    assert!(slide.ratio == 0.,"read as a slide: {slide:?}");
    let volume = exported(&fixtures::read(&source),"part");
    let segment = 25.*(0.7f64).acos()-3.5*(25.-3.5*3.5f64).sqrt();
    let expected = PI*25.*20.-segment*20.;
    eprintln!("volume {volume:.9} against {expected:.9}");
    assert!((volume-expected).abs() <= 1e-6*expected,"{volume} against {expected}");
}

/// Tilted 25 degrees, the torus's characteristic under the slide is no circle of it, and the
/// groove's section is the torus's shadow along the slide: every line up the rod passing within
/// the tube's radius of the torus's centre circle is cut.
#[test]
fn a_tilted_torus_slid_removes_its_shadow_along_the_slide() {
    admitted_as(&slid(25.));
    let volume = exported(&fixtures::read(&slid(25.)),"part");
    // read about the rod's axis, where a run grows from nothing as the shadow's edge crosses the
    // rod's wall, not as a chord of it does
    let torus = slid_torus(25.);
    let hit = |a: f64,r: f64| torus.nearest(&|q| (q[0]-r*a.cos()).hypot(q[1]-r*a.sin())) <= torus.tube;
    let shadow = measure([-PI,PI],&|_| [0.,5.],&hit,&|a,b| (b*b-a*a)/2.);
    let expected = PI*25.*20.-shadow*20.;
    eprintln!("volume {volume:.9} against {expected:.9}");
    // the shadow's measure reads to a few parts in a million at these rows (to 2e-7 of the solid
    // at four times as many)
    assert!((volume-expected).abs() <= 1e-5*expected,"{volume} against {expected}");
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
    let body = fixtures::solid(&e,"fluted");
    let a = admission::admit_body(&e.sketch,body,&Options::default()).unwrap();
    let [s] = a.sweeps() else { panic!("one sweep") };
    let Class::ConstantTwist(found) = &s.class else { panic!("`flute` is a constant twist's sweep") };
    assert!(found.screw.ratio == 0.,"read as a slide: {:?}",found.screw);
    assert_eq!(s.placements.len(),2,"cut once a flute");
    let tolerance = Some(Tolerance::new(0.01).unwrap());
    let (exact,said) = saying(|say| export::exact(&e.sketch,body,None,tolerance,say));
    let exact = exact.unwrap_or_else(|r| panic!("{r}\n{said}"));
    exact.solid.check(1e-6).unwrap();
    let (stl,said) = saying(|say| export::stl(&e.sketch,body,&exact,tolerance,say).unwrap());
    gcs_core::mesh::stl_shells(&stl).unwrap();
    assert!(said.contains(", 0 disagree"),"{said}");
    // the wheel stands on −x, its profile read across (y) and out from the axis (−x)
    let (nearest,round,flank) = (0.75,2.5,35f64.to_radians());
    let (across,up) = (round*flank.cos(),nearest+round-round*flank.sin());
    let profile = |x: f64,y: f64| (x+nearest+round).hypot(y) <= round || (-x >= up && y.abs() <= across+(-x-up)*flank.tan());
    let hit = |a: f64,r: f64| profile(r*a.cos(),r*a.sin());
    let area = measure([-PI,PI],&|_| [0.,5.],&hit,&|a,b| (b*b-a*a)/2.);
    let expected = PI*25.*40.-2.*area*40.;
    let volume = props::volume(&exact.solid);
    eprintln!("volume {volume:.9} against {expected:.9}");
    assert!((volume-expected).abs() <= 1e-5*expected,"{volume} against {expected}");
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
fn turned(lean: Lean,from: f64,to: f64) -> String {
    let (a,o) = lean.axes();
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
construction wheel := solid(face(tube), about: taxis)
swing := frame
swing turns(1) ax
construction groove := sweep(wheel over t in ({from}deg, {to}deg), under: swing)
part := solid(ring)
groove cut part
",o0=o[0],o1=o[1],o2=o[2],a0=a[0],a1=a[1],a2=a[2],wy=20.*60f64.to_radians().sin())
}

/// The turned wheel's torus as drawn.
pub(crate) fn turned_torus(lean: Lean) -> Torus { Torus {centre:[5.,0.,0.],axis:lean.axes().0,ring:2.5,tube:2.} }

/// What the turn removes from the sector: Pappus over the section of the ring the torus's
/// shadow in a half-plane through z covers, every circle about z that passes within the tube's
/// radius of the torus's centre circle.
pub(crate) fn turned_away(torus: &Torus) -> f64 {
    let hit = |z: f64,r: f64| torus.nearest(&|q| (q[0].hypot(q[1])-r).hypot(q[2]-z)) <= torus.tube;
    PI/3.*measure([-1.,1.],&|_| [8.,12.],&hit,&|a,b| (b*b-a*a)/2.)
}

/// Square to the turn, the torus's shadow in a half-plane through z is its tube's circle pushed
/// out by its ring: the far wall stands at 7.5 + √(4 − z²), and the groove is Pappus's.
#[test]
fn a_turned_torus_grooves_a_ring_by_pappus() {
    let upright = turned(Lean::Across(0.),-130.,130.);
    let turn = admitted_as(&upright);
    assert!(turn.is_turn(),"read as a turn: {turn:?}");
    let volume = exported(&fixtures::read(&upright),"part");
    let sector = PI/3.;
    let n = 2000;
    let h = 2./n as f64;
    let away: f64 = (0..=n).map(|k| { let z = -1.+k as f64*h; ((7.5+(4.-z*z).sqrt()).powi(2)-64.)/2.
        *if k == 0 || k == n { 1. } else if k%2 == 1 { 4. } else { 2. } }).sum::<f64>()*h/3.;
    let expected = sector*(144.-64.)-sector*away;
    // the shadow read from the centre circle agrees with the closed form
    let read = turned_away(&turned_torus(Lean::Across(0.)));
    assert!((read-sector*away).abs() <= 1e-5*read,"the shadow's {read} against Pappus's {}",sector*away);
    eprintln!("volume {volume:.9} against {expected:.9}");
    assert!((volume-expected).abs() <= 2e-5*expected,"{volume} against {expected}");
}

/// Leaning 25 degrees across the turn, its axis skew to the turn's, the torus's characteristic
/// is no circle of it, and what it removes is its shadow turned.
#[test]
fn a_leaning_torus_turned_removes_its_shadow_turned() {
    let source = turned(Lean::Across(25.),-140.,140.);
    admitted_as(&source);
    let volume = exported(&fixtures::read(&source),"part");
    let expected = PI/3.*(144.-64.)-turned_away(&turned_torus(Lean::Across(25.)));
    eprintln!("volume {volume:.9} against {expected:.9}");
    assert!((volume-expected).abs() <= 2e-5*expected,"{volume} against {expected}");
}

// -- refusals --------------------------------------------------------------------------------

/// Each row a turn or a slide meets refuses by name: a turn carried round a whole turn lays its
/// sheet over itself (S4); a roll that leaves the torus in the ring is not clear at its ends (E1);
/// a torus leaning toward the turn's axis, so that its own meets it, has a ring in contact all
/// round, every normal of it through where the axes meet, at which its characteristic branches
/// between two of the walk's stations (S1: the boundary leaves the walked branch there, which
/// the field gate saw before this row did); and a torus slid through the rod's middle touches
/// it all round its tube, a characteristic that closes on itself (S1).
#[test]
fn the_turn_and_slide_rows_refuse_by_name() {
    let whole = refused(admit(&turned(Lean::Across(0.),-180.,180.)));
    assert_eq!((whole.condition,whole.class()),(Condition::Lead,"constant-twist class"));
    assert!(whole.message.contains("whole turn"),"{}",whole.message);
    let short = refused(admit(&turned(Lean::Across(0.),-130.,30.)));
    assert_eq!(short.condition,Condition::Clearance);
    let branching = refused(admit(&turned(Lean::Toward(25.),-140.,140.)));
    assert_eq!(branching.condition,Condition::Regular,"{branching}");
    assert!(branching.message.contains("in contact all round"),"{}",branching.message);
    let through = slid(0.).replace("du: 5.5mm","du: 0mm").replace("du: 11.5mm","du: 6mm");
    let closed = refused(admit(&through));
    assert_eq!(closed.condition,Condition::Regular,"{closed}");
}

// -- what the shadows are read by ------------------------------------------------------------

/// A torus: the points within `tube` of its centre circle, about `centre` square to `axis`, of
/// radius `ring`.
pub(crate) struct Torus { pub(crate) centre: V,pub(crate) axis: V,pub(crate) ring: f64,pub(crate) tube: f64 }

impl Torus {
    /// The point of the centre circle at angle `a`.
    fn at(&self,a: f64) -> V {
        let n = self.axis;
        let seed = if n[0].abs() < 0.9 { [1.,0.,0.] } else { [0.,1.,0.] };
        let e1 = unit(cross(n,seed));
        let e2 = cross(n,e1);
        std::array::from_fn(|k| self.centre[k]+self.ring*(a.cos()*e1[k]+a.sin()*e2[k]))
    }
    /// The least of `d` over the centre circle: read four degrees apart, each local least refined
    /// by golden section.
    pub(crate) fn nearest(&self,d: &dyn Fn(V) -> f64) -> f64 {
        let n = 90;
        let f = |a: f64| d(self.at(a));
        let values: Vec<f64> = (0..n).map(|k| f(TAU*k as f64/n as f64)).collect();
        let mut best = f64::INFINITY;
        for k in 0..n {
            let (before,here,after) = (values[(k+n-1)%n],values[k],values[(k+1)%n]);
            if here > before || here > after { continue }
            let (mut a,mut b) = (TAU*(k as f64-1.)/n as f64,TAU*(k as f64+1.)/n as f64);
            let g = (5f64.sqrt()-1.)/2.;
            for _ in 0..60 {
                let (x,y) = (b-g*(b-a),a+g*(b-a));
                if f(x) < f(y) { b = y } else { a = x }
            }
            best = best.min(f(0.5*(a+b)));
        }
        best
    }
}

fn cross(a: V,b: V) -> V { [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]] }
fn unit(a: V) -> V { let l = (a[0]*a[0]+a[1]*a[1]+a[2]*a[2]).sqrt(); a.map(|x| x/l) }

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
        let edge = |mut a: f64,mut b: f64| { for _ in 0..50 { let m = 0.5*(a+b); if hit(x,m) == hit(x,a) { a = m } else { b = m } } 0.5*(a+b) };
        let mut sum = 0.;
        let mut start = if inside[0] { Some(lo) } else { None };
        for k in 1..=n {
            match (inside[k-1],inside[k]) {
                (false,true) => start = Some(edge(at(k-1),at(k))),
                (true,false) => { sum += along(start.take().unwrap(),edge(at(k-1),at(k))) }
                _ => {}
            }
        }
        if let Some(a) = start { sum += along(a,hi) }
        sum
    };
    let values: Vec<f64> = gcs_core::par::indices(rows+1,|k| row(u[0]+k as f64*h));
    values.iter().enumerate().map(|(k,v)| v*if k == 0 || k == rows { 1. } else if k%2 == 1 { 4. } else { 2. }).sum::<f64>()*h/3.
}
