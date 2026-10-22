//! The four-cylinder engine (`rust/examples/engine.sv`, its parts in `engine/`) as solids: the
//! assembly shows one of every part it places — block, bearing caps, core plugs and oil pan, head,
//! camshafts and their caps, valves and buckets, crankshaft, rods, pistons and gudgeon pins,
//! pulleys — each built exactly, the turned and swept ones the closed form of the table's numbers;
//! and each part's file opens alone from its `preview`, solving with nothing left free and building
//! the solid the assembly does.
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
const RJ: f64 = 25.;
const CAPD: f64 = 14.;
const WMB: f64 = 22.;
const HW: f64 = 75.;
const GASKET: f64 = 2.;
const HEAD: f64 = 190.;
const RCAMJ: f64 = 13.;
const WCAMB: f64 = 16.;
const CAMCAP: f64 = 4.;
const RCP: f64 = 26.;
const VS: f64 = 19.;
const RFW: f64 = 140.;
const RPIN: f64 = 11.;
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
    let i = fixtures::solid(e,name);
    let s = match e.sketch.exact_solid(i) {
        Ok(s) => s,
        // a body a generating sweep cuts (a toothed pulley) is built by the generating class, as
        // `solventc --step` and the page build it, in the document's millimetres where it stands
        Err(why) => {
            use gcs_core::brep::{export,sweep::Say};
            let say = Say { stage: &|_: &str| {}, mark: &|_| {} };
            let x = export::exact(&e.sketch,i,None,None,&say)
                .unwrap_or_else(|r| panic!("{name}: {why}; generated, refused at {:?}: {}",r.stage,
                                           r.message));
            let (lo,hi) = x.solid.bounds();
            return (props::volume(&x.solid),[lo[0],hi[0],lo[1],hi[1],lo[2],hi[2]]);
        }
    };
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
    let mut want: Vec<String> = ["block.casting","block.main_caps","block.plugs","block.pan",
        "head.casting","camshaft_i.shaft","camshaft_i.caps","camshaft_e.shaft","camshaft_e.caps",
        "crank.shaft","crank.ring.gear","pistons1.body","pistons2.body","pistons3.body",
        "pistons4.body","pistons1.gudgeon","pistons2.gudgeon","pistons3.gudgeon","pistons4.gudgeon",
        "drive.crank.pulley","drive.crank.flanges","drive.intake.pulley","drive.exhaust.pulley",
        "drive.belt","drive.tensioner"]
        .iter().map(|s| s.to_string()).collect();
    // every cylinder's four valves, a pair at each station, each with its bucket
    for c in 1..=4 {
        for at in ["f","r"] {
            for v in ["intake","exhaust"] {
                for s in ["valve","bucket"] { want.push(format!("valves{c}{at}.{v}.{s}")); }
            }
        }
    }
    // each rod with its cap, its two cap bolts and its small end's bush
    for k in 1..=4 {
        for part in ["body","cap","bush","bolt_left.bolt","bolt_right.bolt"] {
            want.push(format!("rod{k}.{part}"));
        }
    }
    want.sort();
    assert_eq!(shown,want);

    // the castings run the block's length, the block from its rail to its deck and the pan, its
    // flange the block's outline under the rail, from the rail to its floor; the head stands on
    // its gasket
    let (_,b) = built(&e,"block.casting");
    for (got,want) in b.iter().zip([FRONT,BACK,-KW,KW,RAIL,DECK]) { close("block",*got,want); }
    let (_,b) = built(&e,"block.pan");
    for (got,want) in b.iter().zip([FRONT,BACK,-KW,KW,SUMP-8.,RAIL]) { close("pan",*got,want); }
    // the five bearing caps are half rings round the shells, `wmb` long; the eight core plugs
    // discs 16 in radius and 4 thick, a millimetre inside the block's sides
    let rmb = RJ+2.;
    close("bearing caps",built(&e,"block.main_caps").0,5.*WMB*PI/2.*((rmb+CAPD).powi(2)-rmb*rmb));
    let (v,b) = built(&e,"block.plugs");
    close("core plugs",v,8.*PI*16.*16.*4.);
    close("plugs out",b[3],HW-1.);
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
    // the timing drive (`engine/end_view.sv`): 20 and 40 teeth on a belt pitch of 2π rcp / 20,
    // the tooth's sizes fractions of it; each pulley generated by the belt's tooth, its tips `pld`
    // inside its pitch circle and its roots a tooth's height `bh` inside them, bored to its shaft
    let pitch = 2.*PI*RCP/20.;
    let (pld,bh,bt) = (0.04*pitch,0.2*pitch,0.18*pitch);
    let (rnose,ridl) = (16.,16.);
    for (name,rp,bore,x0,x1) in [("drive.crank.pulley",RCP,rnose,FRONT-53.,FRONT-32.),
                                 ("drive.intake.pulley",2.*RCP,RCAMJ,FRONT-55.,FRONT-30.),
                                 ("drive.exhaust.pulley",2.*RCP,RCAMJ,FRONT-55.,FRONT-30.)] {
        let (v,b) = built(&e,name);
        let (tip,root,w) = (rp-pld,rp-pld-bh,x1-x0);
        // the B-rep's box is its faces' surfaces', the generated sheets' running a little past the
        // faces they bound: it holds the pulley's stretch along the axis and reaches its tips
        let reach = (b[3]-b[2]).max(b[5]-b[4])/2.;
        assert!(b[0] <= x0+1e-6 && b[0] > x0-2. && b[1] >= x1-1e-6 && b[1] < x1+2.,
                "{name}: stands from {} to {} along the axis against {x0} to {x1}",b[0],b[1]);
        assert!(reach > tip-0.2 && reach < tip+2.,"{name}: reaches {reach} against tips {tip}");
        let (most,least) = (PI*(tip*tip-bore*bore)*w,PI*(root*root-bore*bore)*w);
        assert!(v < most && v > least+0.3*(most-least),
                "{name}: {v} not between {least} and {most}");
    }
    // the cam pulleys are one shape on the two cams
    let (ve,vi) = (built(&e,"drive.exhaust.pulley").0,built(&e,"drive.intake.pulley").0);
    assert!((ve-vi).abs() < 1e-4*vi,"cam pulleys: {ve} against {vi}");
    // the crank pulley's guide flanges, `bt + 2` past the belt's back, two millimetres each
    let rf = RCP-pld+bt+2.;
    close("flanges",built(&e,"drive.crank.flanges").0,2.*PI*(rf*rf-rnose*rnose)*2.);
    // the tensioner's roller on its pin, across the teeth
    close("tensioner",built(&e,"drive.tensioner").0,PI*(ridl*ridl-36.)*21.);
    // the belt runs from under the crank to over the cams, and out past both cams' far sides
    let va = 20f64.to_radians();
    let camx = VS+(STEM+RB)*va.sin();
    let camh = DECK+(D/2.-VS)*va.tan()+(STEM+RB)*va.cos();
    let rk = 2.*RCP-pld;
    let (_,b) = built(&e,"drive.belt");
    for (got,want) in b.iter().zip([FRONT-52.,FRONT-33.,-(camx+rk+bt),camx+rk+bt,-(RCP-pld+bt),
                                   camh+GASKET+rk+bt]) {
        close("belt",*got,want);
    }
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
        ("block",&[("block.casting","block.casting"),("block.pan","block.pan"),
                   ("block.main_caps","block.main_caps"),("block.plugs","block.plugs")]),
        ("head",&[("cylinder_head.casting","head.casting"),
                  ("camshaft_i.shaft","camshaft_i.shaft"),("camshaft_e.caps","camshaft_e.caps"),
                  ("valves3r.exhaust.valve","valves3r.exhaust.valve")]),
        ("crankshaft",&[("crank.shaft","crank.shaft")]),
        ("conrod",&[("rod.body","rod1.body"),("rod.cap","rod1.cap"),
                    ("rod.bolt_left.bolt","rod1.bolt_left.bolt"),("rod.bush","rod1.bush")]),
        ("parts",&[("piston_s.body","pistons1.body"),("piston_s.gudgeon","pistons1.gudgeon")]),
        ("valvetrain",&[("poppet.valve","valves1f.intake.valve"),
                        ("poppet.bucket","valves1f.intake.bucket")]),
        ("end_view",&[("drive.crank.pulley","drive.crank.pulley"),("drive.belt","drive.belt")]),
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

/// **The crank turns the engine** (#398 L): `t`, the assembly's one parameter, is cylinder 1's
/// angle in its four-stroke cycle, and the drawing is followed over the whole of it.  Stood at
/// every eighth of a turn, each crank pin is where the throw puts it, each piston's small end
/// where the slider-crank's closed form puts it, and each valve as far off its seat as its lobe
/// reaches: the tangent cam's support along the valve's axis, `max(rb, dn cos φ + rn)`, the
/// camshaft turning at half the crank's speed and each cylinder a quarter of the cycle behind the
/// one before it in the firing order 1-3-4-2.  It is drawn at the table's 400°, and runs from
/// there round to it again.
#[test]
fn the_engine_turns_through_its_cycle() {
    let e = open("engine.sv");
    let march = gcs_core::follow::followed(&e.sketch,"t");
    let f = march.as_ref().as_ref().expect("followed");
    assert_eq!((f.over.from,f.reached(),f.singular),(400.,1120.,None));
    let mut sk = e.sketch.clone();
    let at = |sk: &gcs_core::model::Sketch,name: &str| {
        sk.world_point(e.map.ent_named(name).unwrap_or_else(|| panic!("{name}")).i())
    };
    let near = |what: &str,got: f64,want: f64| {
        assert!((got-want).abs() < 1e-6,"{what}: {got} against {want}");
    };
    let apart = |a: [f64;3],b: [f64;3]| ((a[0]-b[0]).powi(2)+(a[1]-b[1]).powi(2)+(a[2]-b[2]).powi(2)).sqrt();
    let dn = |dur: f64| (RB-RN)/(dur/4.).to_radians().cos();
    let lift = |dur: f64,phi: f64| (dn(dur)*phi.to_radians().cos()+RN).max(RB)-RB;
    // where in its cycle each cylinder is when cylinder 1 is at 0: the firing order 1-3-4-2
    let behind = [0.,540.,180.,360.];
    let (icenter,ecenter) = (450.+(48.-12.)/2.,270.+(12.-48.)/2.);
    for step in 0..=16 {
        let t = 400.+step as f64*45.;
        gcs_core::follow::stand(&mut sk,"t",t).expect("reached");
        // cylinder 1's throw in the end section, `t` clockwise from top dead centre
        let pin = at(&sk,"crank.t1.pin");
        let side = sk.basis(e.map.ent_named("std.side").unwrap().i());
        let (x,y) = side.view_coords(pin);
        near("pin across",x,R*t.to_radians().sin());
        near("pin up",y,R*t.to_radians().cos());
        for k in 1..=4 {
            // 1 and 4 are up together, 2 and 3 a half turn on; the small end rides the bore axis
            let ph = (t+if k == 2 || k == 3 { 180. } else { 0. }).to_radians();
            let small = R*ph.cos()+(L*L-R*R*ph.sin().powi(2)).sqrt();
            near(&format!("rod {k}'s small end at {t}"),at(&sk,&format!("rod{k}.sm_s"))[2],small);
            let c = (t-behind[k-1]).rem_euclid(720.);
            for (aft,(intake,dur,center)) in ["f","r"].iter()
                .flat_map(|a| [(a,("i",IDUR,icenter)),(a,("e",EDUR,ecenter))]) {
                let v = format!("valves{k}{aft}");
                let reach = apart(at(&sk,&format!("{v}.drawn_{intake}.fc")),
                                  at(&sk,&format!("{v}.cam_{intake}")));
                near(&format!("{v} {intake} at {t}"),reach,RB+lift(dur,(c-center)/2.));
            }
        }
    }
}

