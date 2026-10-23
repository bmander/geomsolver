//! **A face family its statements say a motion carries is that motion's sweep** (#366 S3): every
//! row of the station reading its running number a point carried by one motion to a roll affine
//! in it (`std.Helix`, `std.OnHelix`, a frame `at(under: m) a·s + b`), so its walls are
//! the section's first rim carried by the motion's own screw — the carry the statement gives, not
//! one read off the stations.  The two springs meet at one export: `lofts/spring.sv`'s wire drawn
//! square to its helix and `swept_spring.sv`'s ball swept along it are one tube, carried by one
//! screw, the canal's two half balls the only difference; and the family exports as the canal
//! does.  A station something else reads the running number of is no motion's sweep.
use crate::common::{close, ent, parsed::determined};
use gcs_core::brep::geom::{Carry, Surface};
use gcs_core::brep::topo::Brep;
use gcs_core::brep::{export, props, sweep::Say};
use gcs_core::program::Elaborated;
use std::f64::consts::{PI, TAU};

/// Solid `name` of `e` built exactly, written as STEP and as STL by this kernel (each read back and
/// checked as `solventc` checks it), and its B-rep.
fn exported(e: &Elaborated, name: &str) -> Brep {
    let s = ent(e, name).i();
    let say = Say { stage: &|_: &str| {}, mark: &|_| {} };
    let exact = export::exact(&e.sketch, s, None, None, &say)
        .unwrap_or_else(|r| panic!("{name}: {r:?}"));
    export::step(&exact, name, None, &say).unwrap();
    let stl = export::stl(&e.sketch, s, &exact, None, &say).unwrap();
    gcs_core::mesh::stl_shells(&stl).unwrap();
    exact.solid
}

/// The carry of every wall of `b` that is a surface carried by one (`Blend`).
fn carries(b: &Brep) -> Vec<Carry> {
    b.faces.iter().filter_map(|f| match &f.surface {
        Surface::Blend(_, blend) => Some(blend.carry),
        _ => None,
    }).collect()
}

/// A screw's carry, as the numbers it says: its centre, its unit axis, its turn and its rise.
fn screw(c: Carry) -> ([f64; 3], [f64; 3], f64, f64) {
    match c {
        Carry::Screw { center, axis, angle, advance } => (center, axis, angle, advance),
        Carry::Line { .. } => panic!("a slide, not a screw"),
    }
}

/// The screw about the world's `z` through the origin, `turns` turns climbing `pitch` a turn, to
/// the last bit the solve leaves.
fn is_screw(c: Carry, turns: f64, pitch: f64, what: &str) {
    let (center, axis, angle, advance) = screw(c);
    let off = |v: [f64; 3], w: [f64; 3]| (0..3).map(|k| (v[k] - w[k]).abs()).fold(0.0, f64::max);
    assert!(off(center, [0.0; 3]) < 1e-12 && off(axis, [0.0, 0.0, 1.0]) < 1e-12, "{what}: {c:?}");
    assert!((angle - TAU * turns).abs() < 1e-12 && (advance - pitch * turns).abs() < 1e-12,
            "{what}: {c:?}");
}

/// **The two springs are one tube on one screw**: the wire drawn square to its helix
/// (`lofts/spring.sv`) and the ball swept along it (`swept_spring.sv`), given one coil — radius
/// 12, wire 2, pitch 8, two turns — are carried by the same screw, the screw the motions state,
/// and the family is the canal less its two half balls, to 1e-9.  Both are exported as the core
/// exports a solid: STEP and STL, each read back.
#[test]
fn the_two_springs_meet_at_one_export() {
    let lofted = include_str!("../../examples/lofts/spring.sv")
        .replace("r := 10mm", "r := 12mm").replace("wire := 1.5mm", "wire := 2mm")
        .replace("pitch := 5mm", "pitch := 8mm").replace("turns := 4", "turns := 2");
    let family = determined(&lofted);
    let canal = determined(include_str!("../../examples/swept_spring.sv"));
    let (f, c) = (exported(&family, "spring"), exported(&canal, "spring"));
    let ball = 4.0 / 3.0 * PI * 8.0;
    close(props::volume(&f) + ball, props::volume(&c));
    let length = 2.0 * (TAU * 12.0f64).hypot(8.0);
    close(props::volume(&f), PI * 4.0 * length);
    let (fc, cc) = (carries(&f), carries(&c));
    assert_eq!((fc.len(), cc.len()), (1, 1), "one tube each: {fc:?} {cc:?}");
    is_screw(fc[0], 2.0, 8.0, "the wire's");
    is_screw(cc[0], 2.0, 8.0, "the ball's");
}

/// **The thread and the flight are their screw's**: `std.Helix` carries `base` by the screw about
/// its normal, so every wall of the thread (drawn in `axial`) and of the flight (drawn in `n`) is
/// its first rim carried by that screw — three turns at 4 and two and a half at 12.
#[test]
fn a_helix_family_is_carried_by_its_screw() {
    let e = determined(include_str!("../../examples/lofts/thread.sv"));
    let thread = e.sketch.exact_solid(ent(&e, "thread").i()).unwrap();
    let walls = carries(&thread.brep);
    assert_eq!(walls.len(), 4);
    for c in walls {
        is_screw(c, 3.0, 4.0, "the thread's");
    }
    let flight = e.sketch.exact_solid(ent(&e, "flight").i()).unwrap();
    let walls = carries(&flight.brep);
    assert_eq!(walls.len(), 4);
    for c in walls {
        is_screw(c, 2.5, 12.0, "the flight's");
    }
    close(props::volume(&thread.brep), 240.0 * PI);
    close(props::volume(&flight.brep), 240.0);
}

/// **A section the running number reads otherwise is no motion's sweep**: the thread's square
/// swelling as it runs reads `s` in a dimension of its own, so its walls stay the family's sheets,
/// at Pappus over the swell, 2π · 10 · 3 · ∫(2 + s)² ds = 380π; and the twisted duct, its station
/// pinned to its guide by `t == s` and its section's frame turned within it, is no screw either.
#[test]
fn a_station_read_otherwise_is_built_wall_by_wall() {
    let src = include_str!("../../examples/lofts/thread.sv")
        .replace("sq := std.CenteredRectangle(c, w: size, h: size) in std.front",
                 "sq := std.CenteredRectangle(c, w: size + s * 1mm, h: size + s * 1mm) \
                  in std.front");
    assert!(src.contains("size + s * 1mm"));
    let e = determined(&src);
    let thread = e.sketch.exact_solid(ent(&e, "thread").i()).unwrap();
    assert!(carries(&thread.brep).is_empty(), "a swelling section is carried by nothing");
    close(props::volume(&thread.brep), 380.0 * PI);
    let e = determined(include_str!("../../examples/lofts/twisted_duct.sv"));
    let duct = e.sketch.exact_solid(ent(&e, "duct").i()).unwrap();
    close(props::volume(&duct.brep), 16000.0);
}

/// **A law reparametrises its screw's sweep** (#391 S5): the flight's screw stood at `900deg · s +
/// 30deg · sin(360deg · s)` rather than at `900deg · s` turns and rises at a pace that is not
/// constant, but monotonically, and a sweep reads the path alone: the walls are the same screw's
/// sweep of the section, two and a half turns rising 12 a turn, and the flight the same
/// 8 · 30 = 240 mm³.  The value is not affine in `s` as it is written (`expr::Ast::affine_in`),
/// so the statements do not say the carry (`solid::family::stated_carry`, which sampled the ends
/// and the middle through 0.142 and read 0°, 450° and 900° there); the walls are read as carried
/// off the stations.
#[test]
fn a_law_reparametrises_its_screws_sweep() {
    let src = include_str!("../../examples/lofts/thread.sv").replace(
        "  st := std.Helix(std.top, pitch: pitch, turns: turns, s: s)\n  in st.n {",
        "  private screw := frame\n  \
         screw screws(pitch) std.top.normal\n  \
         fix(value == 360deg * turns * s + 30deg * sin(360deg * s)) screw\n  \
         st_n := std.top in screw\n  in st_n {").replace(") st.n\n", ") st_n\n");
    assert!(src.contains("sin(360deg * s)") && !src.contains(") st.n\n"), "the flight moved");
    let e = determined(&src);
    let flight = e.sketch.exact_solid(ent(&e, "flight").i()).unwrap();
    let walls = carries(&flight.brep);
    assert_eq!(walls.len(), 4);
    for c in walls {
        let (center, axis, angle, advance) = screw(c);
        assert!(center.iter().all(|v| v.abs() < 1e-9) && (axis[2] - 1.0).abs() < 1e-9, "{c:?}");
        assert!((angle - TAU * 2.5).abs() < 1e-9 && (advance - 30.0).abs() < 1e-9, "{c:?}");
    }
    close(props::volume(&flight.brep), 240.0);
}
