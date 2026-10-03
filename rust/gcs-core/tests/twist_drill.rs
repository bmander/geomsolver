//! The twist drill (`rust/examples/twist_drill`, issue #64): flutes ground by a wheel carried along
//! a screw, the constant-twist class (`solid::constant_twist`, docs/generating-sweeps.md). The
//! drill solves and is admitted; each row of the class refuses by name; the flute's section square
//! to the axis is the wheel's characteristic — computed here again, from the wheel's and the screw's
//! closed forms, not by the code under test — in the field and in the exact solid; a fluted section
//! holds the same material at every height and every length, in the field and exactly; and the
//! reduced drill exports by this kernel, its file held to its field, and builds for the page.
use gcs_core::brep::{export,props,sweep::Say};
use gcs_core::model::Sketch;
use gcs_core::program::Elaborated;
use gcs_core::solid::{admission::{self,Class,Condition,Error,Options},MaterialField,cad,constant_twist,export::Tolerance};
use std::f64::consts::{PI,TAU};
use std::sync::{Mutex,OnceLock};

type V = [f64;3];

// The configuration's numbers, read here as the drill states them.
const DIAMETER: f64 = 10.;
const HELIX: f64 = 30.;
const CLEARANCE: f64 = 0.25;
/// The flute wheel's: radius to the rim, how near the drill's axis it comes, the round's radius,
/// the flanks' lean and how far they run up.
const RIM: f64 = 25.;
const NEAREST: f64 = 0.75;
const ROUND: f64 = 2.5;
const FLANK: f64 = 35.;

fn lead() -> f64 { PI*DIAMETER/HELIX.to_radians().tan() }

/// Build `body` exactly within `tolerance`, saying nothing; what was said comes back with it.
fn built(sk: &Sketch,body: usize,tolerance: Option<Tolerance>) -> (export::Exact,String) {
    let lines = Mutex::new(Vec::<String>::new());
    let say = Say {stage:&|l: &str| lines.lock().unwrap().push(l.to_string()),mark:&|_| {}};
    let exact = export::exact(sk,body,None,tolerance,&say).unwrap_or_else(|r| panic!("{r}\n{}",lines.lock().unwrap().join("\n")));
    let said = lines.lock().unwrap().join("\n");
    (exact,said)
}

/// The reduced drill `length` long: one flute, its end square.
fn reduced(length: &str) -> Elaborated { fixtures::drill::reduced(length) }

/// The reduced drill's fluted body, built exactly within 0.01 mm, once a length for every test.
fn fluted(length: &str) -> &'static (export::Exact,String) {
    static SIX: OnceLock<(export::Exact,String)> = OnceLock::new();
    static TWELVE: OnceLock<(export::Exact,String)> = OnceLock::new();
    let build = || { let e = reduced(length); built(&e.sketch,fixtures::solid(&e,"fluted"),Some(Tolerance::new(0.01).unwrap())) };
    match length { "6mm" => SIX.get_or_init(build),"12mm" => TWELVE.get_or_init(build),_ => unreachable!() }
}

fn admit(e: &Elaborated,body: &str) -> Result<admission::Admission,Error> {
    admission::admit_body(&e.sketch,fixtures::solid(e,body),&Options::default())
}

fn refused(result: Result<admission::Admission,Error>) -> admission::Refusal {
    match result {
        Err(Error::Refused(r)) => { eprintln!("{r}"); r }
        Err(Error::Unreadable(m)) => panic!("unreadable: {m}"),
        Ok(a) => panic!("admitted: {:?}",a.sweeps().iter().map(|s| &s.name).collect::<Vec<_>>()),
    }
}

/// The drill solves with nothing left free, and both its sweeps — the flute and the body
/// clearance, each placed once a flute — are admitted to the constant-twist class, their reach
/// sampled finely whether it is a whole face of the wheel or a sliver of a large round.
#[test]
fn the_drill_solves_and_both_of_its_sweeps_are_admitted() {
    let mut e = fixtures::drill::standard();
    assert_eq!(gcs_core::diagnose::diagnose(&mut e.sketch,Default::default()).dof,0);
    let a = admit(&e,"fluted").unwrap();
    let names: Vec<&str> = a.sweeps().iter().map(|s| s.name.as_str()).collect();
    assert_eq!(names,["flute","body_clearance"]);
    for s in a.sweeps() {
        let Class::ConstantTwist(found) = &s.class else { panic!("`{}` is a screw's sweep",s.name) };
        assert_eq!(s.placements.len(),2,"`{}` is cut once a flute",s.name);
        assert!(s.contacts >= 64,"`{}`: {} points across its reach",s.name,s.contacts);
        assert!(found.least_factor > 0.05,"`{}`: {}",s.name,found.least_factor);
    }
}

/// What the field says of the drill: the web solid, each flute empty along its helix, a margin at
/// the full diameter right behind each lip and the body ground back behind it, the point ground
/// to the tip, and the shank whole.
#[test]
fn the_drills_field_has_its_flutes_margins_point_and_shank() {
    let e = fixtures::drill::standard();
    let field = MaterialField::read(&e.sketch,fixtures::solid(&e,"drill"),cad::AXIS_TOLERANCE).unwrap();
    let at = |r: f64,degrees: f64,z: f64| field.side([r*degrees.to_radians().cos(),r*degrees.to_radians().sin(),z]) < 0.;
    for z in [2.,10.,20.,30.] {
        // the flute wheel stands at 180° at z = 0, and the flutes turn with the helix
        let middle = 180.+360.*z/lead();
        assert!(at(0.3,middle,z),"the web at {z}");
        for flute in [0.,180.] {
            assert!(!at(3.,middle+flute,z) && !at(4.9,middle+flute,z),"a flute at {z}");
            assert!(at(3.,middle+flute+90.,z),"a land at {z}");
            // the lip leads the flute by its half width (51.8°): the margin is behind it, then the
            // clearance, ground 0.25 mm back
            let lip = middle+flute-51.83;
            assert!(at(4.99,lip-4.,z),"the margin at {z}");
            assert!(!at(4.9,lip-30.,z) && at(5.-CLEARANCE-0.05,lip-30.,z),"the body clearance at {z}");
        }
    }
    // the point: the tip on the axis half a millimetre into the stock, the outer corners a point's
    // length below it
    assert!(at(0.,0.,39.3) && !at(0.,0.,39.7));
    let corner = 39.5-5./(59f64.to_radians().tan());
    assert!((0..36).all(|k| !at(4.95,k as f64*10.,corner+0.3)),"ground away above the corners");
    // the shank, whole
    assert!((0..36).all(|k| at(4.95,k as f64*10.,-10.)));
}

/// Each row of the constant-twist class refuses a wheel that fails it, by name: flanks too steep
/// for the helix fold the flute's wall (S3), a narrow wheel with shallow flanks reaches the stock
/// twice on a ring (S2). A crossing within a lead (S4) is asked of the section's polyline.
#[test]
fn the_constant_twist_rows_refuse_by_name() {
    let wheel = |round: &str,flank: &str| format!("{{rim: 25mm, nearest: web / 2, round: {round}, flank: {flank}, depth: 7mm}}");
    let steep = wheel("2.5mm","20deg");
    let r = refused(admit(&fixtures::drill::read(&[("flutes","1"),("point","0"),("fluted_length","6mm"),("flute_wheel",&steep)]),"fluted"));
    assert_eq!(r.condition,Condition::Edgewise);
    assert!(r.to_string().contains("outside the constant-twist class (S3"),"{r}");
    assert!(r.message.contains("left_flank") && r.message.contains("turned its sign"),"{}",r.message);
    assert!(r.witness.is_some());
    let narrow = wheel("1.5mm","10deg");
    let r = refused(admit(&fixtures::drill::read(&[("flutes","1"),("point","0"),("fluted_length","6mm"),("flute_wheel",&narrow)]),"fluted"));
    assert_eq!(r.condition,Condition::Once);
    // S4's question: a polyline that turns back across itself crosses; one that does not, does not
    // (sampled off its crossing, which then falls inside two segments)
    let figure: Vec<[f64;2]> = (0..=64).map(|k| { let t = TAU*(k as f64+0.3)/64.; [t.sin(),(2.*t).sin()/2.] }).collect();
    assert!(constant_twist::crossing(&figure).is_some());
    let arc: Vec<[f64;2]> = (0..=64).map(|k| { let t = PI*k as f64/64.; [t.cos(),t.sin()] }).collect();
    assert_eq!(constant_twist::crossing(&arc),None);
}

/// The flute wheel's characteristic under the screw, computed from closed forms: for each point of
/// the wheel's profile (its round and its flanks), the points of its ring whose normal is square to
/// the screw's velocity, kept where the ring comes within `within` of the drill's axis, each carried
/// along its helix to height `z0`: the flute's section there. Each comes with the wheel's outward
/// normal, carried with it, which points into the drill's material.
fn characteristic_section(z0: f64,within: f64) -> Vec<(V,V)> {
    let (b,a) = (HELIX.to_radians(),RIM+NEAREST);
    let lead = lead();
    // the wheel's axis and the directions about it: a profile point (u, v) of its axial plane is
    // u along the axis and v from the drill's axis, `a - v` from the wheel's
    let w: V = [0.,b.cos(),b.sin()];
    let (e1,e2): (V,V) = ([1.,0.,0.],[0.,b.sin(),-b.cos()]);
    let mut profile: Vec<([f64;2],[f64;2])> = Vec::new();
    let flank = FLANK.to_radians();
    for k in 0..=80 {
        // the round between its tangent points, its outward normal from its centre
        let t = -PI/2.-(PI/2.-flank)+(PI-2.*flank)*k as f64/80.;
        let n = [t.cos(),t.sin()];
        profile.push(([ROUND*n[0],NEAREST+ROUND+ROUND*n[1]],n));
    }
    for k in 1..=40 {
        // each flank, up from its tangent point
        for side in [-1.,1.] {
            let n = [side*flank.cos(),-flank.sin()];
            let base = [side*ROUND*flank.cos(),NEAREST+ROUND-ROUND*flank.sin()];
            let s = 4.*k as f64/40.;
            profile.push(([base[0]+side*s*flank.sin(),base[1]+s*flank.cos()],n));
        }
    }
    let mut out = Vec::new();
    for (m,nm) in profile {
        let radius = a-m[1];
        let at = |phi: f64| -> (V,V) {
            let e: V = std::array::from_fn(|i| phi.cos()*e1[i]+phi.sin()*e2[i]);
            let p: V = std::array::from_fn(|i| -a*e1[i]+m[0]*w[i]+radius*e[i]);
            let n: V = std::array::from_fn(|i| nm[0]*w[i]-nm[1]*e[i]);
            (p,n)
        };
        let g = |phi: f64| { let (p,n) = at(phi); let v = [-p[1],p[0],lead/TAU]; n[0]*v[0]+n[1]*v[1]+n[2]*v[2] };
        let steps = 3600;
        for j in 0..steps {
            let (mut lo,mut hi) = (TAU*j as f64/steps as f64,TAU*(j+1) as f64/steps as f64);
            if g(lo)*g(hi) > 0. { continue }
            for _ in 0..80 { let mid = 0.5*(lo+hi); if g(lo)*g(mid) <= 0. { hi = mid } else { lo = mid } }
            let (p,n) = at(0.5*(lo+hi));
            if p[0].hypot(p[1]) >= within { continue }
            // along its helix to z0: turned by the screw's angle there
            let turn = TAU*(z0-p[2])/lead;
            let (c,s) = (turn.cos(),turn.sin());
            let q = [c*p[0]-s*p[1],s*p[0]+c*p[1],z0];
            let m = [c*n[0]-s*n[1],s*n[0]+c*n[1],n[2]];
            out.push((q,m));
        }
    }
    assert!(out.len() > 60,"the characteristic crosses the stock: {} points",out.len());
    out
}

/// A flute's section square to the axis is the wheel's characteristic carried along the screw: at
/// every point of it computed from the closed forms, the field changes from void to material across
/// a hundredth of a millimetre, and the exact solid's boundary passes within that tolerance. (The
/// section is read inside the body clearance's reach, where the flute alone bounds the drill.)
#[test]
fn a_flute_section_is_the_wheels_characteristic() {
    let e = reduced("12mm");
    let field = MaterialField::read(&e.sketch,fixtures::solid(&e,"fluted"),cad::AXIS_TOLERANCE).unwrap();
    let section = characteristic_section(6.,DIAMETER/2.-CLEARANCE-0.2);
    let h = 0.005;
    for (p,n) in &section {
        let off = |k: f64| field.side(std::array::from_fn(|i| p[i]+k*h*n[i]));
        assert!(off(1.) < 0. && off(-1.) > 0.,"the field does not change across the characteristic at {p:?}: {} and {}",off(-1.),off(1.));
    }
    let (exact,said) = fluted("12mm");
    let mut worst = 0_f64;
    for (p,_) in &section {
        let gap = exact.solid.faces.iter().map(|f| { let q = f.surface.point(f.surface.inverse(*p));
            (0..3).map(|i| (q[i]-p[i]).powi(2)).sum::<f64>().sqrt() }).fold(f64::INFINITY,f64::min);
        worst = worst.max(gap);
    }
    assert!(worst < 0.01,"the exact flute is {worst:.2e} mm from the characteristic\n{said}");
    eprintln!("the exact flute within {:.2} µm of {} points of the characteristic",worst*1e3,section.len());
}

/// The fluted section's area in the field at height `z`: rays out from the axis, a degree apart and
/// turned with the screw so that every height is read alike, each crossing of the material found by
/// bisection.
fn section_area(field: &MaterialField,z: f64) -> f64 {
    let (rays,steps,outer) = (360,120,DIAMETER/2.+0.1);
    let turn = TAU*z/lead();
    (0..rays).map(|k| {
        let t = TAU*k as f64/rays as f64+turn;
        let inside = |r: f64| field.side([r*t.cos(),r*t.sin(),z]) < 0.;
        let mut area = 0.;
        let mut from = if inside(0.) { Some(0.) } else { None };
        for j in 0..steps {
            let (mut lo,mut hi) = (outer*j as f64/steps as f64,outer*(j+1) as f64/steps as f64);
            if inside(lo) == inside(hi) { continue }
            let entering = inside(hi);
            for _ in 0..50 { let mid = 0.5*(lo+hi); if inside(mid) == inside(lo) { lo = mid } else { hi = mid } }
            let r = 0.5*(lo+hi);
            if entering { from = Some(r) } else if let Some(r0) = from.take() { area += (r*r-r0*r0)/2.; }
        }
        area*TAU/rays as f64
    }).sum()
}

/// The fluted body holds the same material at every height and every length: in the field, its
/// section's area read alike at three heights; exactly, the volumes of a 6 mm and a 12 mm drill
/// over their lengths; and the two agree.
#[test]
fn volume_per_length_is_constant_in_the_field_and_exactly() {
    let e = reduced("12mm");
    let field = MaterialField::read(&e.sketch,fixtures::solid(&e,"fluted"),cad::AXIS_TOLERANCE).unwrap();
    let areas: Vec<f64> = [3.,6.,9.].iter().map(|&z| section_area(&field,z)).collect();
    for a in &areas { assert!((a-areas[0]).abs() <= 1e-6*areas[0],"the field's sections {areas:?}"); }
    let per = |length: &str,l: f64| props::volume(&fluted(length).0.solid)/l;
    let (six,twelve) = (per("6mm",6.),per("12mm",12.));
    eprintln!("field section {:.6} mm², exact {six:.6} and {twelve:.6} mm³ a millimetre",areas[0]);
    assert!((six-twelve).abs() <= 1e-6*six,"exactly: {six} and {twelve} mm³ a millimetre");
    assert!((areas[0]-six).abs() <= 2e-3*six,"the field's {} mm² against the exact solid's {six}",areas[0]);
}

/// The reduced drill's fluted body is written by this kernel within 0.01 mm: its STEP parsed back,
/// its STL meshed within the tolerance and held to its material field with no disagreement.
#[test]
fn the_reduced_drill_exports_by_this_kernel() {
    let e = reduced("6mm");
    let body = fixtures::solid(&e,"fluted");
    let (exact,said) = fluted("6mm");
    assert!(said.contains("built whole, not as one sector"),"{said}");
    exact.solid.check(1e-6).unwrap();
    let tolerance = Some(Tolerance::new(0.01).unwrap());
    let lines = Mutex::new(Vec::<String>::new());
    let say = Say {stage:&|l: &str| lines.lock().unwrap().push(l.to_string()),mark:&|_| {}};
    let step = export::step(exact,"fluted",tolerance,&say).unwrap();
    assert!(step.starts_with("ISO-10303-21;"));
    let stl = export::stl(&e.sketch,body,exact,tolerance,&say).unwrap();
    gcs_core::mesh::stl_shells(&stl).unwrap();
    let said = lines.lock().unwrap().join("\n");
    assert!(said.contains(", 0 disagree"),"{said}");
}

/// The reduced drill with its shank, written as an STL within 0.01 mm, measured against the exact
/// faces it was made from by the accuracy meter (`solventc --measure`): its flute read as the
/// screw's sweep, the shank as a solid of its own, the body rule over the two.
#[test]
fn the_reduced_drill_measures_within_its_tolerance() {
    use gcs_core::solid::accuracy::{self,Meter,Options};
    let e = reduced("6mm");
    let body = fixtures::solid(&e,"drill");
    let tolerance = Some(Tolerance::new(0.01).unwrap());
    let (exact,_) = built(&e.sketch,body,tolerance);
    let lines = Mutex::new(Vec::<String>::new());
    let say = Say {stage:&|l: &str| lines.lock().unwrap().push(l.to_string()),mark:&|_| {}};
    let stl = export::stl(&e.sketch,body,&exact,tolerance,&say).unwrap();
    let meter = Meter::read(&e.sketch,body,Options::in_units(1.)).unwrap();
    let (vertices,triangles) = gcs_core::solid::agreement::stl_triangles(&stl,1.).unwrap();
    let samples = accuracy::mesh_samples(&vertices,&triangles,2000);
    let measurements: Vec<_> = samples.iter().map(|s| meter.measure(s.position)).collect();
    let (ok,line) = accuracy::within(&meter,&samples,&measurements,0.01,1.);
    assert!(ok,"{line}");
    for name in ["flute_wheel.body.round","shank_wall"] {
        assert!(meter.surfaces().iter().any(|s| s.name == name),"{name} among {:?}",meter.surfaces().iter().map(|s| &s.name).collect::<Vec<_>>());
    }
}

/// The page's exact surface of the reduced drill — its fluted body built by stages, then the
/// shank added by the body rule — has the volume of the drill the export builds, to the fit.
#[test]
fn the_page_builds_the_drills_exact_surface_by_stages() {
    let e = reduced("6mm");
    let body = fixtures::solid(&e,"drill");
    let mut builder = export::Builder::new(&e.sketch,body).unwrap();
    let mut doing = vec![builder.doing()];
    let (_,total) = builder.stages();
    while !builder.step().unwrap() { doing.push(builder.doing()); }
    assert_eq!(builder.stages(),(total,total),"{doing:?}");
    assert!(doing.iter().any(|d| d == "adding the rest of the body"),"{doing:?}");
    let shank = PI*25.*30.;
    let fluted = props::volume(&fluted("6mm").0.solid);
    let volume = builder.display().unwrap().volume;
    // (the display's sheets are fitted to the gross bars, the export's within 0.01 mm)
    assert!((volume-(fluted+shank)).abs() < 1e-5*volume,"{volume} against {}",fluted+shank);
}

/// The whole drill — two flutes and their clearances, the point and the shank — written by this
/// kernel within 0.01 mm and held to its field.
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, about 30 s: the whole drill exported")]
fn the_whole_drill_exports_by_this_kernel() {
    let e = fixtures::drill::standard();
    let body = fixtures::solid(&e,"drill");
    let tolerance = Some(Tolerance::new(0.01).unwrap());
    let (exact,said) = built(&e.sketch,body,tolerance);
    exact.solid.check(1e-6).unwrap();
    let lines = Mutex::new(Vec::<String>::new());
    let say = Say {stage:&|l: &str| lines.lock().unwrap().push(l.to_string()),mark:&|_| {}};
    export::step(&exact,"drill",tolerance,&say).unwrap();
    export::stl(&e.sketch,body,&exact,tolerance,&say).unwrap();
    let said = format!("{said}\n{}",lines.lock().unwrap().join("\n"));
    assert!(said.contains(", 0 disagree"),"{said}");
    eprintln!("{said}");
}

/// Two flutes 12 mm long with the point ground: the flutes' sheets cross the cylinder where the
/// cones bound it, and the meetings the kernel traces there pass through the vertices they must
/// (a fitted curve that hooked back at one turned the cylinder's pieces round: issue #64).
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, about 15 s: a short pointed drill exported")]
fn a_short_pointed_drill_exports_by_this_kernel() {
    let e = fixtures::drill::read(&[("fluted_length","12mm")]);
    let body = fixtures::solid(&e,"fluted");
    let (exact,said) = built(&e.sketch,body,None);
    exact.solid.check(1e-6).unwrap();
    let lines = Mutex::new(Vec::<String>::new());
    let say = Say {stage:&|l: &str| lines.lock().unwrap().push(l.to_string()),mark:&|_| {}};
    export::stl(&e.sketch,body,&exact,None,&say).unwrap();
    let said = format!("{said}\n{}",lines.lock().unwrap().join("\n"));
    assert!(said.contains(", 0 disagree"),"{said}");
}

/// The drawing (`drill.svd`): its views, section and dimensions compiled from the reduced drill,
/// the swept body drawn from its exact B-rep (the overall length is the shank's and the flutes'
/// to the micrometre, not a mesh's reading of it).
#[test]
fn the_drawing_compiles_from_the_exact_drill() {
    let dir = fixtures::drill::project();
    let svd = std::fs::read_to_string(dir.join("drill.svd")).unwrap();
    let svg = gcs_core::drawing::compile(&svd,dir.join("drill.svd").to_str().unwrap(),None,&mut |path,from| {
        let base = std::path::Path::new(from).parent().unwrap_or(std::path::Path::new("."));
        let p = base.join(path);
        let name = p.file_stem()?.to_str()?.to_string();
        let text = std::fs::read_to_string(&p).ok()?;
        let text = fixtures::drill::configure(&name,text,&[("flutes","1"),("fluted_length","6mm"),("point","0")]);
        Some((p.to_string_lossy().into_owned(),text))
    }).unwrap();
    assert!(svg.contains("<svg"));
    // the overall length, 30 mm of shank and 6 of flutes, and the diameter
    assert!(svg.contains(">36<") && svg.contains(">10<"),"the generated dimensions read the exact solid");
}
