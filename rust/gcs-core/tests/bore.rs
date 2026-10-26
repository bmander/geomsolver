//! The Wankel's bore (#65): the envelope of a point is its path, so `envelope(apex over t in (0deg,
//! 1080deg), under: spin, seen_from: counter)` is the epitrochoid, a closed curve of the drawing
//! that stands alone in a face, and the housing is a disc less its prism.
use gcs_core::{model::{EntKind,MotionDef},program};

/// The Wankel's planetary motion and its apex, drawn in the top plane: the rotor's frame `spin`
/// turns about its centre `hub`, `e` from the shaft's `centre`, at −2/3, seen from the housing's,
/// `counter`, turning the other way about the shaft, and the apex is `R` beyond the rotor's
/// centre.  `seen_housing` reads the housing seen from the rotor, as the rotor's flanks are.
pub fn wankel(r: f64, e: f64, extra: &str) -> String {
    format!("unit mm\nuse std\nuse std.motion (spins)\n\
        centre := point in std.top\n\
        centre level(u) std.top\ncentre level(v) std.top\n\
        hub := point in std.top\n\
        hub distance({e}mm, along: u) std.top\nhub level(v) std.top\n\
        counter := frame\ncounter spins(-1) centre\n\
        spin := frame\nspin spins(-2/3) hub\n\
        apex := point in std.top\n\
        apex distance({x}mm, along: u) std.top\napex level(v) std.top\n\
        seen_housing := point\n\
        seen_housing coincident(under: counter, seen_from: spin, t == 0deg) apex\n\
        {extra}\n",x = r+e)
}

/// A document elaborated with no error and solved.
pub fn solved(src: &str) -> program::Elaborated { fixtures::read(src) }

/// The motion node of frame `under` seen from frame `seen_from`, which a reading of it minted:
/// the rotor (`"spin", "counter"`) or the housing seen from it (`"counter", "spin"`).
pub fn seen(e: &program::Elaborated, under: &str, seen_from: &str) -> usize {
    let joint = |n: &str| {
        let f = e.map.ent_named(n).unwrap_or_else(|| panic!("no frame `{n}`"));
        assert_eq!(f.kind,EntKind::Frame);
        e.sketch.frames[f.i()].joint.as_ref().and_then(|j| j.motion).expect("a jointed frame")
    };
    let (s,o) = (joint(under),joint(seen_from));
    e.sketch.motions.iter().position(|m| matches!(m.def,
        MotionDef::Relative {source,observer} if source == s && observer == o))
        .unwrap_or_else(|| panic!("no reading of `{under}` seen from `{seen_from}`"))
}

#[test]
fn the_apex_under_the_rotors_motion_is_the_epitrochoid() {
    let (r,e) = (105.,15.);
    let el = solved(&wankel(r,e,
        "bore := envelope(apex over t in (0deg, 1080deg), under: spin, seen_from: counter)"));
    let bore = el.map.ent_named("bore").unwrap();
    assert_eq!(bore.kind,EntKind::Curve);
    let sk = &el.sketch;
    assert!(sk.curve_closed(bore.i()),"three turns of the shaft close the bore");
    for k in 0..=72 {
        let roll = k as f64*15.;
        let t = (roll/3.).to_radians();
        let (x,y) = sk.curve_point(bore.i(),roll);
        let want = (e*(3.*t).cos()+r*t.cos(),e*(3.*t).sin()+r*t.sin());
        assert!((x-want.0).abs() < 1e-9 && (y-want.1).abs() < 1e-9,"{roll}: ({x}, {y}) against {want:?}");
    }
}

/// The housing: a disc with the bore cut through it. A curve back where it started after whole
/// turns is a whole loop, as a circle is, so `face(bore)` sweeps to the pocket. The epitrochoid encloses π(R² + 3e²) (Green's
/// theorem: the cross terms of e·e^{3it} + R·e^{it} integrate away), and the exact kernel reads it
/// as the B-spline fitted within `FIT_MM`.
pub const HOUSING: &str = "bore := envelope(apex over t in (0deg, 1080deg), under: spin, seen_from: counter)
    rim := circle(center: centre) in std.top hint(r: 170)
    radius(170mm) rim
    block := solid(face(rim), from: 0mm, to: 80mm)
    pocket := solid(face(bore), from: -1mm, to: 81mm)
    housing := solid(block)
    pocket cut housing";

#[test]
fn a_closed_bore_bounds_a_face_and_the_housing_is_its_closed_form() {
    let (r,e,w) = (105.,15.,80.);
    let el = solved(&wankel(r,e,HOUSING));
    let sk = &el.sketch;
    let index = |n: &str| fixtures::solid(&el,n);
    let (pocket,housing) = (index("pocket"),index("housing"));
    let bore = std::f64::consts::PI*(r*r+3.*e*e);
    let disc = std::f64::consts::PI*170f64.powi(2);
    // the exact kernel: the bore's prism and the housing, within the fit
    let exact = |i: usize| {
        let b = gcs_core::brep::recipe::build(&gcs_core::solid::cad::recipe(sk,i).unwrap()).unwrap();
        b.check(1e-9).unwrap();
        gcs_core::brep::props::volume(&b)
    };
    let length = 2.*std::f64::consts::PI*r*1.1;
    let fit = length*82.*gcs_core::solid::cad::FIT_MM;
    let v = exact(pocket);
    assert!((v-bore*82.).abs() <= fit,"{v} against {}",bore*82.);
    let v = exact(housing);
    assert!((v-(disc-bore)*w).abs() <= fit,"{v} against {}",(disc-bore)*w);
    // the material field: inside the wall, outside in the bore, either side of the epitrochoid
    let field = gcs_core::solid::SpatialField::read(sk,housing,1e-9).unwrap();
    for k in 0..36 {
        let t = (k as f64*10.).to_radians();
        let p = [e*(3.*t).cos()+r*t.cos(),e*(3.*t).sin()+r*t.sin()];
        let n = [e*3.*(3.*t).cos()+r*t.cos(),e*3.*(3.*t).sin()+r*t.sin()];
        let l = n[0].hypot(n[1]);
        let at = |d: f64| [p[0]+d*n[0]/l,p[1]+d*n[1]/l,40.];
        assert!(field.value(at(0.01)) < 0. && field.value(at(-0.01)) > 0.,"{t}");
    }
}

/// The bore's field is the distance to its thousands of chords, found through the boxes about
/// them (`solid::field::profile`): at points scattered across the wall and the bore, nearer the bore
/// than the caps, its size is the distance to the epitrochoid sampled densely.
#[test]
fn the_bores_field_is_its_distance_through_the_boxes_about_its_chords() {
    let (r,e) = (105.,15.);
    let el = solved(&wankel(r,e,HOUSING));
    let sk = &el.sketch;
    let housing = fixtures::solid(&el,"housing");
    let field = gcs_core::solid::SpatialField::read(sk,housing,1e-9).unwrap();
    let n = 200_000;
    let curve: Vec<[f64;2]> = (0..n).map(|k| {
        let t = std::f64::consts::TAU*k as f64/n as f64;
        [e*(3.*t).cos()+r*t.cos(),e*(3.*t).sin()+r*t.sin()]
    }).collect();
    let mut rng = 0x2545F4914F6CDD1D_u64;
    let mut next = || { rng ^= rng << 13; rng ^= rng >> 7; rng ^= rng << 17; (rng >> 11) as f64/(1u64 << 53) as f64 };
    for _ in 0..300 {
        let (angle,radius) = (std::f64::consts::TAU*next(),40.+110.*next());
        let p = [radius*angle.cos(),radius*angle.sin()];
        let near = curve.iter().map(|c| (c[0]-p[0]).hypot(c[1]-p[1])).fold(f64::INFINITY,f64::min);
        // the epitrochoid decides the field where it is nearer than the caps, 40 away, and the
        // rim (a Boolean field is the distance only where one operand decides it)
        if near > 39. || radius > 125. { continue }
        let v = field.value([p[0],p[1],40.]);
        assert!((v.abs()-near).abs() < 5e-3,"{p:?}: {v} against {near}");
    }
}
