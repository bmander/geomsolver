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
    let mut want = vec!["block.casting","block.pan","head.casting","head.camshaft_i",
        "head.camshaft_e","crank.shaft","rod1.body","rod2.body","rod3.body","rod4.body",
        "pistons1.body","pistons2.body","pistons3.body","pistons4.body","pistons1.gudgeon",
        "pistons2.gudgeon","pistons3.gudgeon","pistons4.gudgeon","drive.crank_pulley",
        "drive.cam_pulley_i","drive.cam_pulley_e"];
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

    // a piston is turned: its outline's right half — the crown dished, three ring grooves cut in
    // its flank, the skirt whole — about the bore axis, the closed form of its numbers by Pappus.
    // Hollowed under the crown it is a cup; the bosses give back what blocks `2 rboss` square,
    // from their faces to the walls they meet, hold of the hollow; the pin's bore and the
    // slipper's reliefs take some of that again, so the body lies under the cup with its bosses
    // and over it less the whole of each cutter.  It stands at its cylinder's station, under the
    // deck, its foot as far under its pin as the table says; its gudgeon pin is a tube on the same
    // station, as long as it is stated
    let pr = (D-0.5)/2.;
    let rg = pr-3.5;                                  // a ring groove's root
    let (rd,dd) = (26.,2.);                           // the dish: its rim's radius, its depth
    let cap = PI*dd*dd*(3.*(rd*rd+dd*dd)/(2.*dd)-dd)/3.;
    let turned = PI*pr*pr*PH-PI*(pr*pr-rg*rg)*(1.5+1.5+3.)-cap;
    let (crown,belt,rbelt,ri) = (7.,23.,pr-6.5,pr-2.5);
    let cup = turned-PI*rbelt*rbelt*(belt-crown)-PI*ri*ri*(PH-belt);
    let (rpin,rpi,rboss,gap) = (11.,6.5,16.,13.);
    // a boss, a depth of it at a time: the chord of the wall's bore across it, less its face
    let slice = |r: f64| rboss*(r*r-rboss*rboss).sqrt()+r*r*(rboss/r).asin()-2.*rboss*gap;
    let bosses = 2.*(slice(rbelt)*(belt-(CH-rboss))+slice(ri)*(CH+rboss-belt));
    let reach = pr+2.;
    let cutters = PI*rpin*rpin*2.*reach+2.*(reach-26.)*(PH+2.-(PH-10.))*2.*reach;
    let lp = 2.*pr-14.;
    let (piston,_) = built(&e,"pistons1.body");
    assert!(cup+bosses-cutters < piston && piston < cup+bosses,"piston: {piston}");
    for k in 1..=4 {
        close("piston turned",built(&e,&format!("pistons{k}.turned")).0,turned);
        let (v,b) = built(&e,&format!("pistons{k}.body"));
        close("pistons alike",v,piston);
        let station = FRONT+25.+P/2.+(k-1) as f64*P;
        close("piston station",(b[0]+b[1])/2.,station);
        assert!(b[5] < DECK,"piston {k}'s crown stands over the deck: {}",b[5]);
        let (g,gb) = built(&e,&format!("pistons{k}.gudgeon"));
        close("gudgeon",g,PI*(rpin*rpin-rpi*rpi)*lp);
        close("gudgeon station",(gb[0]+gb[1])/2.,station);
        close("gudgeon length",gb[1]-gb[0],lp);
        // the skirt's foot `ph - ch` under the pin, whose axis is the tube's
        close("piston foot",b[4],(gb[4]+gb[5])/2.+CH-PH);
    }
    // the rods are one shape at four angles
    let (rod,_) = built(&e,"rod1.body");
    for k in 2..=4 { close("rod",built(&e,&format!("rod{k}.body")).0,rod); }
    // the pulleys and the camshafts are prisms of the end view's circles
    close("crank pulley",built(&e,"drive.crank_pulley").0,PI*RCP*RCP*25.);
    close("cam pulley",built(&e,"drive.cam_pulley_i").0,PI*4.*RCP*RCP*25.);
    close("camshaft",built(&e,"head.camshaft_e").0,PI*RCAMJ*RCAMJ*((BACK-10.)-(FRONT-55.)));
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
                  ("cylinder_head.camshaft_i","head.camshaft_i")]),
        ("crankshaft",&[("crank.shaft","crank.shaft")]),
        ("conrod",&[("rod.body","rod1.body")]),
        ("parts",&[("piston_s.body","pistons1.body"),("piston_s.gudgeon","pistons1.gudgeon")]),
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
