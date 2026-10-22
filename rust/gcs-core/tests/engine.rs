//! The four-cylinder engine (`rust/examples/engine.sv`, its parts in `engine/`) as solids: the
//! assembly shows one of every part it places — block and sump, head, camshafts and their caps,
//! valves and buckets, crankshaft, rods, pistons, pulleys — each built exactly, the turned and
//! swept ones the closed form of the table's numbers; and each part's file opens alone from its
//! `preview`, solving with nothing left free and building the solid the assembly does.
use gcs_core::brep::props;
use gcs_core::diagnose::{self,State};
use gcs_core::program::Elaborated;
use std::f64::consts::PI;
use std::path::PathBuf;

// The table's numbers (`engine/dims.sv`), as it states them.
const D: f64 = 80.;
const R: f64 = 45.;
const L: f64 = 150.;
const P: f64 = 90.;
const CH: f64 = 32.;
const PH: f64 = 62.;
const FRONT: f64 = -25.;
const BACK: f64 = 4.*P+25.;
const RAIL: f64 = -40.;
const SUMP: f64 = -130.;
const KW: f64 = 105.;
const GASKET: f64 = 2.;
const HEAD: f64 = 190.;
const RCAMJ: f64 = 13.;
const WCAMB: f64 = 16.;
const CAMCAP: f64 = 4.;
const RCP: f64 = 30.;
const RFW: f64 = 140.;
const DECK: f64 = R+L+CH+1.;
// the valvetrain's (`dims.sv`): the cam's base and nose circles and the valve timing, each lobe's
// nose distance what its duration asks of a tangent cam
const RB: f64 = 17.;
const RN: f64 = 7.;
const IDUR: f64 = 180.+12.+48.;
const EDUR: f64 = 180.+48.+12.;
const STEM: f64 = 100.;
const DIV: f64 = 28.;
const DEV: f64 = 24.;
// the camshaft's and the valve's own (`head.sv`, `valvetrain.sv`)
const RCAMS: f64 = 11.;
const WLOBE: f64 = 12.;
const VX: f64 = 16.;
const RSTEM: f64 = 3.5;
const MARGIN: f64 = 1.5;
const SEATW: f64 = 2.5;
const NECK: f64 = 14.;
const RBUCKET: f64 = 15.;
const DBUCKET: f64 = 20.;
const SKIRT: f64 = 1.5;
const CROWN: f64 = 3.;

/// A tangent cam's area: the hull of its base circle and its nose circle `dn` out, the two
/// sectors and the two trapezoids between the radii square to each flank.
fn tangent_cam(dn: f64) -> f64 {
    let lean = ((RB-RN)/dn).asin();
    RB*RB*(PI+2.*lean)/2.+RN*RN*(PI-2.*lean)/2.+(RB+RN)*(dn*dn-(RB-RN)*(RB-RN)).sqrt()
}

/// A camshaft: the journal from the pulley's stub through the first bearing, the shaft on to its
/// end, a journal at each other bearing and a lobe over each of the eight valves it opens.
fn camshaft(dur: f64) -> f64 {
    let dn = (RB-RN)/(dur/4.).to_radians().cos();
    let stub = FRONT+25.+WCAMB/2.-(FRONT-55.);
    PI*RCAMJ*RCAMJ*stub+PI*RCAMS*RCAMS*((BACK-10.)-(FRONT+25.+WCAMB/2.))
        +4.*PI*(RCAMJ*RCAMJ-RCAMS*RCAMS)*WCAMB+8.*(tangent_cam(dn)-PI*RCAMS*RCAMS)*WLOBE
}

/// A valve turned from its half section: the head's disc to its margin, the 45° seat, the neck
/// and the stem, a frustum each.
fn valve(head: f64) -> f64 {
    let frustum = |h: f64,r0: f64,r1: f64| PI*h*(r0*r0+r0*r1+r1*r1)/3.;
    let r = head/2.;
    frustum(MARGIN,r,r)+frustum(SEATW,r,r-SEATW)+frustum(NECK-MARGIN-SEATW,r-SEATW,RSTEM)
        +frustum(STEM-CROWN-NECK,RSTEM,RSTEM)
}

fn project() -> PathBuf { PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../examples") }

/// A document of the engine's, solved, its modules read beside it.
fn open(path: &str) -> Elaborated {
    let source = std::fs::read_to_string(project().join(path)).unwrap();
    fixtures::read_beside(&source,&project(),&mut |_,text| text)
}

/// The named solid of `e`, built exactly: its volume and its bounds.
fn built(e: &Elaborated,name: &str) -> (f64,[f64;6]) {
    let s = e.sketch.exact_solid(fixtures::solid(e,name)).unwrap_or_else(|why| panic!("{name}: {why}"));
    // the B-rep is in millimetres about the solid's origin
    let (lo,hi) = s.brep.bounds();
    let w = |k: usize,x: f64| s.origin[k]+x/s.mm;
    (props::volume(&s.brep)/s.mm.powi(3),[w(0,lo[0]),w(0,hi[0]),w(1,lo[1]),w(1,hi[1]),w(2,lo[2]),w(2,hi[2])])
}

/// `got` is `want` to a part in a million.
fn close(name: &str,got: f64,want: f64) {
    assert!((got-want).abs() < 1e-6*want.abs().max(1.),"{name}: {got} against {want}");
}

/// **The assembly is a solid engine**: what the workspace shows is a solid of every part the
/// document places, and nothing it is made of — the bores, bays, tunnels and edge-on outlines are
/// operands, not objects.
#[test]
fn the_engine_shows_a_solid_of_every_part() {
    let e = open("engine.sv");
    let mut shown: Vec<String> = gcs_core::overview::shown_objects(&e.sketch).into_iter()
        .map(|i| e.sketch.solid_name(i)).collect();
    shown.sort();
    let mut want: Vec<String> = ["block.casting","block.pan","head.casting","camshaft_i.shaft",
        "camshaft_i.caps","camshaft_e.shaft","camshaft_e.caps","crank.shaft","rod1.body",
        "rod2.body","rod3.body","rod4.body","pistons1.body","pistons2.body","pistons3.body",
        "pistons4.body","drive.crank_pulley","drive.cam_pulley_i","drive.cam_pulley_e"]
        .iter().map(|s| s.to_string()).collect();
    // every cylinder's four valves, a pair at each station, each with its bucket
    for c in 1..=4 {
        for at in ["f","r"] {
            for v in ["intake","exhaust"] {
                for s in ["valve","bucket"] { want.push(format!("valves{c}{at}.{v}.{s}")); }
            }
        }
    }
    want.sort();
    assert_eq!(shown,want);

    // the castings run the block's length, the block from its rail to its deck and the sump from
    // the rail to its floor; the head stands on its gasket
    let (_,b) = built(&e,"block.casting");
    for (got,want) in b.iter().zip([FRONT,BACK,-KW,KW,RAIL,DECK]) { close("block",*got,want); }
    let (_,b) = built(&e,"block.pan");
    close("pan floor",b[4],SUMP);
    close("pan top",b[5],RAIL);
    let (_,b) = built(&e,"head.casting");
    close("head face",b[4],DECK+GASKET);
    close("head top",b[5],DECK+HEAD);

    // a piston is a cylinder of its skirt's diameter, as tall as the table says, its crown `ch`
    // over the small end its rod puts it on
    let w = (D-0.5)/2.;
    for k in 1..=4 {
        let (v,b) = built(&e,&format!("pistons{k}.body"));
        close("piston",v,PI*w*w*PH);
        close("piston station",(b[0]+b[1])/2.,FRONT+25.+P/2.+(k-1) as f64*P);
    }
    // the rods are one shape at four angles
    let (rod,_) = built(&e,"rod1.body");
    for k in 2..=4 { close("rod",built(&e,&format!("rod{k}.body")).0,rod); }
    // the pulleys are prisms of the end view's circles
    close("crank pulley",built(&e,"drive.crank_pulley").0,PI*RCP*RCP*25.);
    close("cam pulley",built(&e,"drive.cam_pulley_i").0,PI*4.*RCP*RCP*25.);
    // the camshafts are prisms of the end view's circles and tangent cams, whichever way the
    // cycle points each lobe; each runs from the pulley's stub to its last bearing, and its caps
    // are the five bearings' blocks less the journal they hold
    close("intake camshaft",built(&e,"camshaft_i.shaft").0,camshaft(IDUR));
    let (v,b) = built(&e,"camshaft_e.shaft");
    close("exhaust camshaft",v,camshaft(EDUR));
    close("camshaft front",b[0],FRONT-55.);
    close("camshaft back",b[1],BACK-10.);
    let w = RCAMJ+CAMCAP;
    close("caps",built(&e,"camshaft_i.caps").0,5.*(2.*w*w-PI*RCAMJ*RCAMJ/2.)*WCAMB);
    // a valve and its bucket are turned from their half sections, at their own station along the
    // crank axis, `vx` either side of their bore's
    for c in 1..=4 {
        for (k,at) in ["f","r"].iter().enumerate() {
            let x = FRONT+25.+P/2.+(c-1) as f64*P+(2.*k as f64-1.)*VX;
            for (v,head) in [("intake",DIV),("exhaust",DEV)] {
                let (vol,b) = built(&e,&format!("valves{c}{at}.{v}.valve"));
                close("valve",vol,valve(head));
                close("valve station",(b[0]+b[1])/2.,x);
                let (vol,b) = built(&e,&format!("valves{c}{at}.{v}.bucket"));
                let inner = RBUCKET-SKIRT;
                close("bucket",vol,PI*(RBUCKET*RBUCKET*DBUCKET-inner*inner*(DBUCKET-CROWN)));
                close("bucket station",(b[0]+b[1])/2.,x);
            }
        }
    }
    // the crankshaft runs from its nose to the flywheel's back, as wide as the flywheel
    let (_,b) = built(&e,"crank.shaft");
    for (got,want) in b.iter().zip([FRONT-70.,BACK+50.,-RFW,RFW,-RFW,RFW]) {
        close("crankshaft",*got,want);
    }
}

/// **Each part opens alone** (`preview`): a part's file, opened, draws that part about the world's
/// origin as the assembly does, solves with nothing left free, and builds the very solid the
/// assembly does.
#[test]
fn each_part_previews_as_the_assembly_builds_it() {
    let whole = open("engine.sv");
    let parts: [(&str,&[(&str,&str)]);8] = [
        ("block",&[("block.casting","block.casting"),("block.pan","block.pan")]),
        ("head",&[("cylinder_head.casting","head.casting"),
                  ("camshaft_i.shaft","camshaft_i.shaft"),("camshaft_e.caps","camshaft_e.caps"),
                  ("valves3r.exhaust.valve","valves3r.exhaust.valve")]),
        ("crankshaft",&[("crank.shaft","crank.shaft")]),
        ("conrod",&[("rod.body","rod1.body")]),
        ("parts",&[("piston_s.body","pistons1.body")]),
        ("valvetrain",&[("poppet.valve","valves1f.intake.valve"),
                        ("poppet.bucket","valves1f.intake.bucket")]),
        ("end_view",&[("drive.crank_pulley","drive.crank_pulley")]),
        ("side_view",&[]),
    ];
    for (part,solids) in parts {
        let e = open(&format!("engine/{part}.sv"));
        assert!(!e.sketch.points.is_empty(),"{part} previews nothing");
        let mut sk = e.sketch.clone();
        let d = diagnose::diagnose(&mut sk,Default::default());
        assert_eq!((d.dof,d.status),(0,State::Well),"{part}");
        for (mine,theirs) in solids { close(mine,built(&e,mine).0,built(&whole,theirs).0); }
    }
}
