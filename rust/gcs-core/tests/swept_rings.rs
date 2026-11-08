//! **A ring under any rigid motion** (`brep::swept::rings`, docs/swept-solid-design.md steps 6 and
//! 7 under K2): a tool of revolution clear of its axis — a torus, convex nowhere across its hole —
//! carried by a motion no constant twist takes, its contact read meridian by meridian in closed
//! form and cut where its sheet folds. Held to what is known apart from the road: the torus read as
//! one whole circle; its contact under `swept_tumble.sv`'s tumble two curves at every instant, the
//! outer touching all round and the inner folding four times; and every contact standing on the
//! side of the sweep its `s''` says, read against the ring itself at the instants beside.
use gcs_core::brep::swept::closed::Unbuilt;
use gcs_core::brep::swept::rings::{Corner,Ring,Side,admits,contacts};
use gcs_core::brep::topo::Brep;
use gcs_core::brep::trajectory::Trajectory;
use gcs_core::model::SolidDef;

/// The sweep `name` of `text` as the kernel takes it: its tool's B-rep, its trajectory and its roll.
fn sweep(text: &str,name: &str) -> (Brep,Trajectory,[f64;2]) {
    let e = fixtures::read(text);
    let sk = &e.sketch;
    let swept = fixtures::solid(&e,name);
    let SolidDef::Swept {source,motion,..} = &sk.solids[swept].def else { panic!("`{name}` is a sweep") };
    let path = gcs_core::motion::Family::read(sk,*motion as usize).unwrap().trajectory().unwrap();
    let roll = gcs_core::solid::roll::interval(sk,swept).unwrap();
    let (path,roll) = path.unpaced(roll).unwrap();
    let tool = gcs_core::brep::recipe::build(&gcs_core::solid::cad::recipe(sk,*source as usize).unwrap()).unwrap();
    (tool,path.clone(),roll)
}

/// `swept_tumble.sv`'s torus and tumble.
fn tumble() -> (Brep,Trajectory,[f64;2]) { sweep(include_str!("../../examples/swept_tumble.sv"),"tumbled") }

/// **A torus is a ring**: its one face its meridian's whole circle, turning counter-clockwise, no
/// corner, as thick as its tube; a tool reaching its axis (a bull-nose) is none, refused by name.
#[test]
fn a_torus_reads_as_a_ring_and_a_tool_on_its_axis_does_not() {
    let torus = gcs_core::brep::build::torus([1.,2.,3.],[0.,0.,1.],2.,0.5).unwrap();
    let ring = Ring::read(&torus).unwrap();
    assert_eq!(ring.elements.len(),1);
    let gcs_core::brep::swept::patches::Element::Arc {c,r,sweep,..} = ring.elements[0] else { panic!("{:?}",ring.elements) };
    assert!((c[0]-2.).abs() < 1e-9 && (r-0.5).abs() < 1e-9 && (sweep-std::f64::consts::TAU).abs() < 1e-9,"{:?}",ring.elements);
    assert_eq!(ring.corners,vec![Corner::Smooth]);
    assert!((ring.thickness-1.).abs() < 1e-6,"{}",ring.thickness);
    let bull = crate::swept_closed::bull_nose(0.,1.,0.3,1.);
    assert!(matches!(Ring::read(&bull),Err(Unbuilt::Curved {..})));
}

/// **The tumbling torus's contact, read meridian by meridian**: two curves at every instant over
/// the roll (the road admits it, every instant's pieces the first's) — the outer touching all
/// round, the inner folding four times: two stretches touching, two buried, and a bridge across
/// each fold between them.
#[test]
fn the_tumbling_torus_touches_along_two_curves_one_folding_four_times() {
    let (tool,path,roll) = tumble();
    admits(&tool,&path,roll).unwrap();
    let ring = Ring::read(&tool).unwrap();
    for t in [roll[0],0.,roll[1]] {
        let curves = contacts(&ring,&path,roll,t,1).unwrap();
        assert_eq!(curves.len(),2,"at {t}");
        let count = |c: &[([f64;3],Side)],s: Side| c.iter().filter(|p| p.1 == s).count();
        let (outer,inner) = if count(&curves[0],Side::Fold) == 0 { (&curves[0],&curves[1]) } else { (&curves[1],&curves[0]) };
        assert!(outer.iter().all(|p| p.1 == Side::Touching),"at {t}: {outer:?}");
        assert_eq!([count(inner,Side::Touching),count(inner,Side::Fold),count(inner,Side::Buried)],[2,4,2],"at {t}: {inner:?}");
    }
}

/// **Each contact stands on the side of the sweep its `s''` says**: at three rolls, every point of a
/// touching piece is held by the ring at no instant a little before or after (the sweep's boundary
/// there, short of what else holds it), and every point of a buried one by the ring at both (inside
/// the sweep) — read against the ring itself, carried back from those instants.
#[test]
fn each_contact_is_held_beside_its_instant_as_its_bend_says() {
    let (tool,path,roll) = tumble();
    let ring = Ring::read(&tool).unwrap();
    let d = 0.03;
    let mut read = 0;
    for t in [-1.,0.,1.] {
        let beside = [t-d,t+d].map(|s| path.pose(s).unwrap().inverse());
        for c in contacts(&ring,&path,roll,t,24).unwrap() {
            for (p,side) in c {
                let held = beside.map(|m| ring.holds(m.point(p)));
                match side {
                    Side::Touching => assert!(!held[0] && !held[1],"at {t}: a touching contact {p:?} held beside it: {held:?}"),
                    Side::Buried => assert!(held[0] && held[1],"at {t}: a buried contact {p:?} not held beside it: {held:?}"),
                    Side::Fold => continue,
                }
                read += 1;
            }
        }
    }
    assert_eq!(read,360,"contacts read");
}

/// **The tumbling torus built closed** — not yet. The tubes fit (at the build's tolerance of 1e-5,
/// each face refined apart: 577 × 257 and 321 × 65 poles, in a second), their meetings with the
/// torus where it starts and stops are read in their own domains, and the box is split by them all
/// at once into seven cells (in eleven seconds); but classifying those cells takes some seventeen
/// minutes — certified point location against the fitted sheets — and the selected solid fails its
/// check on the start's torus (`loop 1 turns the wrong way`, its kept piece a band round the ring).
/// Its volume, by a Monte Carlo of the definition apart from the kernel (400 000 points, the
/// torus's distance over 1201 rolls), is 25.33 ± 0.05.
#[test]
#[ignore = "not yet built: classifying the cells takes minutes, and the start's kept band of torus fails its check"]
fn the_tumbling_torus_is_built_closed() {
    let (tool,path,roll) = tumble();
    let built = gcs_core::brep::swept::closed::ring(&tool,&path,roll,0.002,1e-5).unwrap_or_else(|u| panic!("{u:?}"));
    let v = gcs_core::brep::props::volume(&built);
    assert!((v-25.33).abs() < 0.2,"{v}");
}

