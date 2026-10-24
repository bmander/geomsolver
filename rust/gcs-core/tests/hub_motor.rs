//! The hub motor (`rust/examples/hub_motor`): a 1 hp outer-rotor e-bike motor written as a
//! project of modules. It solves with nothing left free and every fit it claims holds; and each
//! part built exactly is the closed form of what its dimension table says — the lamination's slot
//! worked out here again from its walls' offsets and the circles they stop on, the magnets as
//! annular sectors, the covers by Pappus over their profiles less their drilled holes — computed
//! from the table's numbers, not by the code under test.
use gcs_core::brep::props;
use gcs_core::diagnose::{self,State};
use gcs_core::program::Elaborated;
use std::f64::consts::PI;
use std::path::PathBuf;

// The table's numbers (`motor/dims.sv`), as it states them.
const RS: f64 = 102.5;
const RM: f64 = RS+0.6;
const RR: f64 = RM+3.;
const RO: f64 = RR+6.;
const L: f64 = 35.;
const HALF: f64 = 26.;
const POLES: f64 = 46.;
const SLOTS: f64 = 51.;
const WT: f64 = 6.6;
const BO: f64 = 2.6;
const HS: f64 = 1.2;
const RY: f64 = 84.5;
const RB: f64 = 76.5;
const RC: f64 = RS-HS-2.;
const OVERHANG: f64 = 6.;
const RI: f64 = RY-0.5;

fn project() -> PathBuf { PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../examples/hub_motor") }

/// The motor, solved.
fn motor() -> Elaborated {
    let source = std::fs::read_to_string(project().join("hub_motor.sv")).unwrap();
    fixtures::read_beside(&source,&project(),&mut |_,text| text)
}

/// The volume of the named solid of `e`, built exactly.
fn volume(e: &Elaborated,name: &str) -> f64 {
    let built = e.sketch.exact_solid(fixtures::solid(e,name))
        .unwrap_or_else(|why| panic!("{name}: {why}"));
    props::volume(&built.brep)
}

/// The named solid's volume is `want`, to a part in a million.
fn close(e: &Elaborated,name: &str,want: f64) {
    let got = volume(e,name);
    assert!((got-want).abs() < 1e-6*want,"{name}: {got} against {want}");
}

/// A polygon's area, counter-clockwise positive (the slot walks clockwise).
fn shoelace(p: &[[f64;2]]) -> f64 {
    (0..p.len()).map(|i| { let (a,b) = (p[i],p[(i+1)%p.len()]); a[0]*b[1]-b[0]*a[1] })
        .sum::<f64>()/2.
}

/// The point at radius `r` lying `off` to the left of the ray from the origin at bearing `phi`
/// (degrees): on the circle, `asin(off / r)` further round.
fn left_of(phi: f64,off: f64,r: f64) -> [f64;2] {
    let a = phi.to_radians()+(off/r).asin();
    [r*a.cos(),r*a.sin()]
}

/// The area a circle of radius `r` holds beyond the chord between two of its points.
fn segment(r: f64,p: [f64;2],q: [f64;2]) -> f64 {
    let half = ((p[0]-q[0]).hypot(p[1]-q[1])/(2.*r)).asin();
    r*r*(half-half.sin()*half.cos())
}

/// One slot's area within the stack, and within the band the coils fill (from the slots' bottom
/// to `RC`): the walls `WT / 2` off the teeth's centrelines half a pitch either side, the shoes
/// closing it to `BO` at `RS − HS`, the mouth open to the tips' circle.
fn slot_areas() -> (f64,f64) {
    let p = 180./SLOTS;
    let a = left_of(-p,WT/2.,RY);
    let h = left_of(-p,WT/2.,RC);
    let g = left_of(0.,-BO/2.,RS-HS);
    let gs = left_of(0.,-BO/2.,RS);
    let mirror = |q: [f64;2]| [q[0],-q[1]];
    let (b,c,d,ds) = (mirror(a),mirror(h),mirror(g),mirror(gs));
    let in_stack = shoelace(&[a,b,c,d,ds,gs,g,h]).abs()+segment(RS,ds,gs);
    let in_band = shoelace(&[a,b,c,h]).abs()+segment(RC,c,h);
    (in_stack,in_band)
}

#[test]
fn the_motor_solves_with_nothing_left_free_and_every_fit_holds() {
    let mut sk = motor().sketch;
    let d = diagnose::diagnose(&mut sk,Default::default());
    assert_eq!(d.dof,0);
    assert_eq!(d.status,State::Well);
    assert!(d.warnings.is_empty(),"{:?}",d.warnings);
    assert_eq!(d.solid_claims.len(),12);
    for c in &d.solid_claims { assert_eq!(c.holds(),Some(true),"{}",c.text()); }
    // and the covers' clearance holes, each cover's own, on the ring's tapped holes: a theorem of
    // the drawing, the mirrored one's image as the other (#578)
    let claims = sk.constraints.iter().filter(|c| c.claim).count();
    assert_eq!(claims,24);
    assert!(d.claims_violated.is_empty() && d.claims_consuming.is_empty(),"{:?}",d.claims_violated);
    assert_eq!(d.claims_theorem.len(),claims);
}

/// The lamination is the annulus less 51 slots; the copper the band the coils fill, the stack's
/// length and the end turns' either side, less the iron in it: the end turns whole, and a coil
/// side in each slot.
#[test]
fn the_stator_is_its_slots() {
    let e = motor();
    let (in_stack,in_band) = slot_areas();
    close(&e,"stator.lamination",L*(PI*(RS*RS-RB*RB)-SLOTS*in_stack));
    close(&e,"stator.windings",2.*OVERHANG*PI*(RC*RC-RI*RI)+SLOTS*L*in_band);
    // three hall sensors, 1.5 across, between the wedge line and the tips, 4 deep
    let (w,r0,r1) = (1.5,RC+0.2,RS-0.3);
    let across = |r: f64| (r*r-w*w/4.).sqrt();
    close(&e,"stator.halls",3.*4.*w*(across(r1)-across(r0)));
}

/// The magnets are 46 annular sectors of 0.8 of a pole pitch; the ring a tube tapped 10 deep for
/// twelve M4 screws at each end.
#[test]
fn the_rotor_is_its_ring_and_magnets() {
    let e = motor();
    let arc = (0.8*360./POLES).to_radians();
    close(&e,"rotor.magnets",POLES*(RR*RR-RM*RM)/2.*arc*L);
    close(&e,"rotor.ring",2.*HALF*PI*(RO*RO-RR*RR)-24.*PI*(3.3f64/2.).powi(2)*10.);
}

/// Each cover is its profile turned (Pappus: the area times the turn of its centroid), less the
/// spoke holes through its flange, the screws' clearance through its web and, on the brake side,
/// the disc's six tapped holes.
#[test]
fn the_covers_are_their_turned_profiles_less_their_holes() {
    let e = motor();
    // (station, radius) round the right-hand cover, as `motor.cover` walks it
    let profile = |boss_to: f64,boss_r: f64| -> Vec<[f64;2]> {
        let (lip_in,lip_out,outer) = (36.,38.,HALF+5.);
        vec![[lip_in,14.],[boss_to,14.],[boss_to,boss_r],[lip_out,boss_r],[lip_out,27.],[outer,27.],
             [outer,RO],[HALF+3.,RO],[HALF+3.,121.],[HALF,121.],[HALF,RR],[HALF-3.,RR],[HALF-3.,RM],
             [HALF,RM],[HALF,27.],[23.,27.],[23.,21.],[lip_in,21.]]
    };
    let turned = |p: &[[f64;2]]| {
        // the profile's first moment about the axle, ∫∫ r dA, summed edge by edge as a polygon's
        let moment: f64 = (0..p.len()).map(|i| {
            let (a,b) = (p[i],p[(i+1)%p.len()]);
            (a[0]*b[1]-b[0]*a[1])*(a[1]+b[1])
        }).sum::<f64>()/6.;
        (2.*PI*moment).abs()
    };
    let hole = |d: f64,depth: f64| PI*d*d/4.*depth;
    let drilled = 18.*hole(2.6,3.)+12.*hole(4.5,5.);
    close(&e,"right.body",turned(&profile(38.+16.,1.375*25.4/2.))-drilled);
    close(&e,"left.body",turned(&profile(67.5-15.1,29.))-drilled-6.*hole(4.2,10.));
}

/// What a figure of the table comes to over the unit it is read in (`motor.dims.torque / 1Nm`),
/// read off the length slot a plain number stands in.
fn figure(read: &str) -> f64 {
    let source = format!("use std\nuse motor.dims\nin std.front {{\na := point hint((0, 0))\n\
        b := point hint((60, 0))\n}}\na distance({read}) b\n");
    let e = fixtures::read_beside(&source,&project(),&mut |_,text| text);
    assert!(e.ok(),"{read}: {:?}",e.errors().map(|d| d.message.clone()).collect::<Vec<_>>());
    e.sketch.user_constraints()[0].args[2].num()
}

/// **The sizing is checked arithmetic** (#311): the duty, the winding and the magnetic circuit
/// are values carrying their units, and each comes to its closed form, worked here from the
/// table's stated numbers in SI.  A figure read in the wrong unit is refused.
#[test]
fn the_tables_figures_are_their_closed_forms() {
    let (power,volts,efficiency) = (745.7,48.,0.86);
    let turning = 32e3/3600./2.09;                       // revolutions a second
    let omega = turning*2.*PI;
    let torque = power/omega;
    let freq = POLES/2.*turning;
    let k_w = (90f64*POLES/SLOTS).to_radians().sin()*0.955;
    let flux = 0.85*0.8*(RM*(360./POLES).to_radians())*1e-3*L*1e-3;
    let emf = PI*2f64.sqrt()*freq*7.*SLOTS/3.*k_w*flux;
    let copper = 5.*PI/4.*0.8*0.8;                       // mm²
    for (read,want) in [
        ("speed / 1rpm",turning*60.),
        ("omega * 1s",omega),
        ("torque / 1Nm",torque),
        ("loss / 1W",power/efficiency-power),
        ("amps / 1A",power/(volts*efficiency)),
        ("freq / 1Hz",freq),
        ("k_w",k_w),
        ("flux / 1mWb",flux*1e3),
        ("B_tooth / 1T",0.85*RS*(360./SLOTS).to_radians()/WT),
        ("B_yoke / 1T",flux/2./((RY-RB)*L*1e-6)),
        ("B_back / 1T",flux/2./((RO-RR)*L*1e-6)),
        ("back_min / 1mm",flux/2./(1.5*L*1e-3)*1e3),
        ("emf / 1V",emf),
        ("emf_peak / 1V",emf*6f64.sqrt()),
        ("phase_amps / 1A",power/(3.*emf)),
        ("density * 1mm * 1mm / 1A",power/(3.*emf)/copper),
        ("fill",2.*7.*copper/81.),
        ("shear / 1kPa",torque/(2.*PI*(RM*1e-3).powi(2)*L*1e-3)/1e3),
    ] {
        let got = figure(&format!("motor.dims.{read}"));
        assert!((got-want).abs() < 1e-9*want,"{read}: {got} against {want}");
    }
    // and to what the table's comments say, to the last figure each gives
    for (read,says,digit) in [("torque / 1Nm",27.9,0.1),("emf / 1V",16.4,0.1),
                              ("phase_amps / 1A",15.2,0.1),("shear / 1kPa",11.9,0.1),
                              ("fill",0.43,0.01)] {
        let got = figure(&format!("motor.dims.{read}"));
        assert!((got-says).abs() <= digit/2.,"{read}: {got}, the table says {says}");
    }
    let source = "use std\nuse motor.dims\nin std.front {\na := point hint((0, 0))\n\
        b := point hint((60, 0))\n}\na distance(motor.dims.torque / 1W) b\n";
    let (mut p,_) = gcs_core::syntax::parse(source);
    let base = project();
    let mut same = |_: &str,text: String| text;
    assert!(gcs_core::modules::link(&mut p,&mut fixtures::beside(&base,&mut same)).is_empty());
    let e = gcs_core::program::elaborate(&p);
    assert!(e.errors().any(|d| d.message.contains("is Length, and this is Time (s)")),
        "{:?}",e.diags.iter().map(|d| d.message.clone()).collect::<Vec<_>>());
}

/// **Each part opens alone** (`preview`): a part's file, opened, draws the assembly's frame and
/// that part in it, solves with nothing left free, and builds the very solid the assembly does.
#[test]
fn each_part_previews_as_the_assembly_builds_it() {
    let whole = motor();
    let parts: [(&str,&[(&str,&str)]);6] = [
        ("axle",&[("axle.body","axle.body"),("axle.key","axle.key")]),
        ("carrier",&[("carrier.body","carrier.body")]),
        ("stator",&[("stator.lamination","stator.lamination"),
                    ("stator.windings","stator.windings")]),
        ("rotor",&[("rotor.ring","rotor.ring"),("rotor.magnets","rotor.magnets")]),
        ("cover",&[("left.body","left.body")]),
        ("parts",&[("bearing.body","bearing_r.body"),("spacer.body","spacer_r.body"),
                   ("nut.body","nut_r.body")]),
    ];
    for (part,solids) in parts {
        let path = project().join(format!("motor/{part}.sv"));
        let e = fixtures::read_beside(&std::fs::read_to_string(path).unwrap(),&project(),
                                      &mut |_,text| text);
        let mut sk = e.sketch.clone();
        let d = diagnose::diagnose(&mut sk,Default::default());
        assert_eq!((d.dof,d.status),(0,State::Well),"{part}");
        for (mine,theirs) in solids { close(&e,mine,volume(&whole,theirs)); }
    }
}
