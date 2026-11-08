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

/// **A tube that folds is entered where it stands outside the tool at both end poses**
/// (`rings::trimmed`): entered from its rim, the tumbling torus's inner tube lay over its own rim
/// beside its fold corners where it starts — a meeting of the tube with itself, which left the
/// torus's face a slit and the sweep's inside one cell with its outside. Trimmed, no point of either
/// tube stands inside the torus where it starts or where it stops, and every edge of each is
/// shared by two of its faces or lies on the torus at an end pose: a touching rim, or a cut.
#[test]
fn a_tube_that_folds_is_entered_where_it_stands_outside_the_tool() {
    use gcs_core::brep::query::{Located,Place};
    use gcs_core::brep::swept::rings::{entered,fitted,posed};
    use gcs_core::brep::topo::EdgeCurve;
    let (tool,path,roll) = tumble();
    let ring = Ring::read(&tool).unwrap();
    let fit = fitted(&ring,&path,roll,1e-6).unwrap();
    let [start,end] = [roll[0],roll[1]].map(|t| posed(&tool,&ring,&path,&fit,t).unwrap());
    let poses = [path.pose(roll[0]).unwrap(),path.pose(roll[1]).unwrap()];
    let (sheets,_) = entered(&fit,&ring,[&start,&end],[&poses[0],&poses[1]],1e-5).unwrap();
    assert_eq!(sheets.len(),2,"a tube for each contact curve");
    // the tool's signed distance at each end pose, at a point in space
    let at = |e: usize,x: [f64;3]| {
        let (m,d) = (&poses[e],[x[0]-poses[e].p[0],x[1]-poses[e].p[1],x[2]-poses[e].p[2]]);
        ring.signed(std::array::from_fn(|i| m.r[0][i]*d[0]+m.r[1][i]*d[1]+m.r[2][i]*d[2])).0
    };
    let on_tool = |x: [f64;3]| (0..2).any(|e| at(e,x).abs() <= 1e-4);
    for (k,sheet) in sheets.iter().enumerate() {
        let uses = sheet.uses();
        for (i,e) in sheet.edges.iter().enumerate() {
            if uses[i].len() == 2 { continue }
            let EdgeCurve::Curve(c) = &e.curve else { panic!("tube {k}: edge {i} has no curve") };
            assert!(uses[i].len() == 1 && [0.,0.5,1.].iter().all(|&f| on_tool(c.point(e.t[0]+(e.t[1]-e.t[0])*f))),
                "tube {k}: edge {i} is used {} times and stands off the tool at an end pose",uses[i].len());
        }
        let located = Located::new(sheet,1e-5);
        let mut read = 0;
        for fi in 0..sheet.faces.len() {
            let [[u0,u1],[v0,v1]] = gcs_core::brep::boolean::face_parameters(sheet,fi);
            for i in 1..12 { for j in 1..12 {
                let p = sheet.faces[fi].surface.point([u0+(u1-u0)*i as f64/12.,v0+(v1-v0)*j as f64/12.]);
                if located.face_place(fi,p) != Place::In { continue }
                read += 1;
                for e in 0..2 { assert!(at(e,p) >= -1e-4,"tube {k}, face {fi}: {p:?} stands {:.2e} inside the tool at end pose {e}",at(e,p)) }
            } }
        }
        assert!(read > 50*sheet.faces.len(),"tube {k}: {read} points read");
    }
    // the inner tube, folding four times, cut: more faces than its twelve pieces
    assert!(sheets[1].faces.len() > 12,"{}",sheets[1].faces.len());
}

/// **The tumbling torus built closed** (the slow tier): the tubes fit (at the build's tolerance of
/// 1e-5, each face refined apart: 577 × 257 and 321 × 65 poles, in a second), the inner one cut to
/// where it stands outside the torus where it starts and stops, the box split by them and the torus
/// at both end poses at once into six cells (in some sixteen seconds) and classified (in under a
/// second), the selected solid checked and judged by the definition. Its volume by a Monte Carlo of
/// the definition apart from the kernel — 80 million points over the sweep's box, each read
/// against the ring's signed distance at 40 001 poses (where no pose of 401 comes within its
/// motion's reach, none beside) — is 25.260 ± 0.005; the solid's own, read off its fitted faces,
/// 25.275. Read point by point against the definition, 100 000 points apart, the two disagree at
/// none.
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, about 30 s: the tumbling torus built closed")]
fn the_tumbling_torus_is_built_closed() {
    let (tool,path,roll) = tumble();
    let built = gcs_core::brep::swept::closed::ring(&tool,&path,roll,0.002,1e-5).unwrap_or_else(|u| panic!("{u:?}"));
    let v = gcs_core::brep::props::volume(&built);
    assert!((v-25.26).abs() < 0.05,"{v}");
}
