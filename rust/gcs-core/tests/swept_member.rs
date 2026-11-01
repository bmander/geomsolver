//! **Whether a point is in a swept solid, the kernel's reading** (#721 X0, step 1,
//! `brep::swept::member`): the tool's B-rep meshed, its signed distance read at the point carried
//! back, branch and bound over the roll with the trajectory's speed bound. Held here to the
//! solver's material field, the semantic reference, over the solid sweeps series: at seeded points
//! about each sweep, wherever the field proves a ball about the point all material or all removed,
//! the kernel says the same, and it settles most of them. Sampled.
use gcs_core::brep::recipe::Recipe;
use gcs_core::brep::swept::member::{Solid,Swept};
use gcs_core::model::SolidDef;
use gcs_core::solid::{cad,Millimetres,MaterialField,ProbeState};

/// The sweep `name` of example `file`: how many seeded points the field proved and the kernel
/// agreed with, and how many the kernel left unsettled; panics on a disagreement.
fn held(file: &str,name: &str) -> (usize,usize) {
    let e = fixtures::read(&crate::solid_sweeps_examples::text(file));
    let sk = &e.sketch;
    let swept = fixtures::solid(&e,name);
    let SolidDef::Swept {source,motion,..} = &sk.solids[swept].def else { panic!("`{name}` is a sweep") };
    let scale = cad::millimetres(sk);
    let tool = Recipe::read(&cad::recipe(sk,*source as usize).unwrap()).unwrap().build().unwrap();
    let solid = Solid::new(&tool,0.01,scale).unwrap();
    let path = gcs_core::motion::Family::read(sk,*motion as usize).unwrap().trajectory().unwrap();
    let roll = gcs_core::solid::roll::interval(sk,swept).unwrap();
    let swept_solid = Swept {tool:&solid,path:&path,roll};
    // a box about the tool at sixteen poses of the roll (mm), a fifth wider
    let (lo,hi) = tool.bounds();
    let (mut a,mut b) = ([f64::INFINITY;3],[f64::NEG_INFINITY;3]);
    for k in 0..=16 {
        let pose = path.pose(roll[0]+(roll[1]-roll[0])*k as f64/16.).unwrap();
        for c in 0..8 {
            let corner: [f64;3] = std::array::from_fn(|i| if (c>>i)&1 == 0 { lo[i] } else { hi[i] }/scale);
            let p = pose.point(corner).map(|x| x*scale);
            for i in 0..3 { a[i] = a[i].min(p[i]); b[i] = b[i].max(p[i]); }
        }
    }
    let grow: [f64;3] = std::array::from_fn(|i| 0.1*(b[i]-a[i]));
    let field = Millimetres::<MaterialField>::read(sk,swept,cad::AXIS_TOLERANCE).unwrap();
    let mut probe = field.evaluator(cad::POSE_CACHE);
    let mut rng = gcs_core::rng::Rng::new(721);
    let (mut agreed,mut unsettled) = (0,0);
    for _ in 0..60 {
        let p: [f64;3] = std::array::from_fn(|i| rng.uniform(a[i]-grow[i],b[i]+grow[i]));
        let proved = match probe.ball(p,0.05) {
            Ok(ProbeState::InteriorBall) => true,
            Ok(ProbeState::ExteriorBall) => false,
            _ => continue,
        };
        match swept_solid.member(p.map(|x| x/scale)) {
            Ok(inside) => { assert_eq!(inside,proved,"{file}: at {p:?} the kernel says {inside}, the field {proved}"); agreed += 1 }
            Err(_) => unsettled += 1,
        }
    }
    (agreed,unsettled)
}

#[test]
fn a_swept_solids_membership_is_the_material_fields() {
    let mut total = 0;
    for (file,name) in [("slide.sv","groove"),("turn.sv","groove"),("screw.sv","groove"),("rack.sv","space"),
        ("conjugate.sv","space"),("path.sv","groove")] {
        let (agreed,unsettled) = held(file,name);
        eprintln!("{file}: {agreed} agreed, {unsettled} unsettled");
        assert!(agreed >= 4*unsettled.max(1),"{file}: {agreed} agreed against {unsettled} unsettled");
        total += agreed;
    }
    assert!(total > 300,"{total} points held");
}
