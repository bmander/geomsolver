//! The four-cylinder engine (`rust/examples/engine.sv`, its parts in `engine/`) as solids: the
//! assembly shows one of every part it places — block and sump, head and camshafts, crankshaft,
//! rods, pistons, pulleys — each built exactly, the turned and swept ones the closed form of the
//! table's numbers; and each part's file opens alone from its `preview`, solving with nothing left
//! free and building the solid the assembly does.
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
const RCP: f64 = 30.;
const RFW: f64 = 140.;
const RPIN: f64 = 11.;
const DECK: f64 = R+L+CH+1.;

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
    let mut want: Vec<String> = ["block.casting","block.pan","head.casting","head.camshaft_i",
        "head.camshaft_e","crank.shaft","crank.ring.gear","pistons1.body","pistons2.body",
        "pistons3.body","pistons4.body","drive.crank_pulley","drive.cam_pulley_i",
        "drive.cam_pulley_e"].into_iter().map(String::from).collect();
    // each rod with its cap, its two cap bolts and its small end's bush
    for k in 1..=4 {
        for part in ["body","cap","bush","bolt_left.bolt","bolt_right.bolt"] {
            want.push(format!("rod{k}.{part}"));
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
    // the rods are one shape at four angles, each cap too; the bush is the ring between the
    // small end's bore and the piston pin, as wide as the small end; a cap bolt an M8 shank and
    // its head
    let (rod,_) = built(&e,"rod1.body");
    let (cap,_) = built(&e,"rod1.cap");
    for k in 2..=4 {
        close("rod",built(&e,&format!("rod{k}.body")).0,rod);
        close("cap",built(&e,&format!("rod{k}.cap")).0,cap);
    }
    for k in 1..=4 {
        close("bush",built(&e,&format!("rod{k}.bush")).0,PI*((RPIN+1.)*(RPIN+1.)-RPIN*RPIN)*22.);
        for side in ["left","right"] {
            close("cap bolt",built(&e,&format!("rod{k}.bolt_{side}.bolt")).0,
                PI*(6.5*6.5*6.+4.*4.*(16.+12.)));
        }
    }
    // the pulleys and the camshafts are prisms of the end view's circles
    close("crank pulley",built(&e,"drive.crank_pulley").0,PI*RCP*RCP*25.);
    close("cam pulley",built(&e,"drive.cam_pulley_i").0,PI*4.*RCP*RCP*25.);
    close("camshaft",built(&e,"head.camshaft_e").0,PI*RCAMJ*RCAMJ*((BACK-10.)-(FRONT-55.)));
    // the crankshaft runs from its nose to the flywheel's back, as wide as the flywheel's shoulder
    // behind the ring gear
    let (_,b) = built(&e,"crank.shaft");
    for (got,want) in b.iter().zip([FRONT-70.,BACK+50.,-(RFW-8.),RFW-8.,-(RFW-8.),RFW-8.]) {
        close("crankshaft",*got,want);
    }

    // the starter's ring gear, its 110 teeth cut by a rack and built as one sector turned round:
    // two faces a tooth (its space's sheet, its tip) and its two sides and its bore; on the
    // flywheel's face toward the engine, 14 wide (the box of a B-rep of sheets is its control
    // points', a little wider than the solid); what the teeth leave of the blank between root and
    // tip is about half of it, as a space is about as wide as a tooth
    let ring = fixtures::solid(&e,"crank.ring.gear");
    let say = gcs_core::brep::sweep::Say { stage: &|_: &str| {}, mark: &|_| {} };
    let exact = gcs_core::brep::export::exact(&e.sketch,ring,None,None,&say)
        .unwrap_or_else(|r| panic!("the ring gear refused at {:?}: {}",r.stage,r.message));
    assert!(exact.pattern.is_some(),"the ring gear is built whole, not as one sector turned");
    assert_eq!(exact.solid.faces.len(),2*110+3,"the ring gear's faces");
    let (lo,hi) = exact.solid.bounds();
    assert!(lo[0] <= BACK+20.+1e-6 && lo[0] > BACK+20.-1.,"the ring's face at {}",lo[0]);
    assert!(hi[0] >= BACK+34.-1e-6 && hi[0] < BACK+34.+1.,"the ring's back at {}",hi[0]);
    for k in 1..3 {
        assert!(lo[k] > -RFW-1. && hi[k] < RFW+1. && hi[k] > RFW-1.,"ring tips {lo:?} {hi:?}");
    }
    let (root,seat) = (137.5-1.25*2.5,RFW-16.);
    let removed = PI*(RFW*RFW-seat*seat)*14.-props::volume(&exact.solid);
    let teeth = PI*(RFW*RFW-root*root)*14.;
    assert!(removed > 0.45*teeth && removed < 0.6*teeth,"the spaces take {removed} of {teeth}");
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
                  ("cylinder_head.camshaft_i","head.camshaft_i")]),
        ("crankshaft",&[("crank.shaft","crank.shaft")]),
        ("conrod",&[("rod.body","rod1.body"),("rod.cap","rod1.cap"),
                    ("rod.bolt_left.bolt","rod1.bolt_left.bolt"),("rod.bush","rod1.bush")]),
        ("parts",&[("piston_s.body","pistons1.body")]),
        ("valvetrain",&[]),
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
