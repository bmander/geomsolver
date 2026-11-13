//! **A hobbed helical gear's mesh has no facet facing against its surface** (#904 A7).  The gear
//! is its blank, the rim chamfered at both faces, less a helical rack's tooth space cut at every
//! tooth; each space's sheet is one face from the chamfer at one end to the chamfer at the other,
//! its root bending tight where the rack's tip corners pass.  The mesher refines a facet that turns
//! from the surface by more than its angular bar on a budget, and a long face's tight bends spent
//! it before one facet turned right over was reached: a facet well inside a flank, away from the
//! chamfers and the face's other edges, came out facing against it, one per tooth, and the STL was
//! refused.  Which facet the budget ran out before turned on the chamfers' size (0.4 mm at both
//! faces of an 18-tooth gear, 0.3 mm at both faces of a 17-tooth one), and on whether the gear was
//! built as a sector or whole.  A facet the export refuses is now refined whatever the budget, as a
//! sag past the bar is.
use gcs_core::brep::{export, mesh, sweep::Say};

/// `gears.Helical` of the shaft-drive benchmark, its hub left out: an involute helical gear generated
/// by a helical rack's tooth drawn in the normal section, rolled on the pitch circle.
const HELICAL: &str = "
unit mm
use std
use std.solids
use std.solids (cut)
use std.rounds (chamfer)
use std.place (horizontal, on_u, coords, right_of, above)
use std.motion (spins, slides)

component Helical(N: Int, mn: Length, alpha: Angle, beta: Angle, hand: Scalar, width: Length,
                  ha: Length, hf: Length, backlash: Length, bore: Length, edge: Length) {
  rp := N * mn / (2 * cos(beta))
  hw := pi * mn / 4 + backlash / 4
  in std.top {
    o := point
    o coincident std.top.origin
    rim := circle(center: o)
    radius(rp + ha) rim
    s0 := point
    s1 := point
    s0 right_of(d: rp) o
    s1 above(d: 10mm) s0
    construction pitch_line := line(s0, s1)
  }
  in std.side {
    l0 := point
    l1 := point
    l2 := point
    fix((0mm, 0mm)) l0
    construction across := line(l0, l2)
    construction lean := line(l0, l1)
    horizontal across
    length(10mm) across
    length(10mm) lean
    l0 distance(>= 0mm, along: u) l2
    across angle(-hand * beta) lean
  }
  normal := plane(u: std.x, v: lean)
  in normal {
    i1 := point
    i1 on_u(d: 10mm) normal
    construction radial := line(normal.origin, i1)
    pr := point
    pl := point
    pr coords(du: rp, dv: hw) normal
    pl coords(du: rp, dv: -hw) normal
    t0 := point
    t1 := point
    t2 := point
    t3 := point
    normal.origin distance(rp - hf, along: u) t0
    normal.origin distance(rp - hf, along: u) t1
    normal.origin distance(rp + 2 * mn, along: u) t2
    normal.origin distance(rp + 2 * mn, along: u) t3
    rack := (tip := line(t0, t1)) -> (right := line(t1, t2)) -> (back := line(t2, t3)) ->
            (left := line(t3, t0)) -> close
    pr coincident right
    pl coincident left
    radial angle(alpha) right
    radial angle(180deg - alpha) left
  }
  construction tooth := solid(std.solids.Prism(rack, from: -width, to: width))
  blank := solid(std.solids.Prism(face(rim), from: -width / 2, to: width / 2))
  when edge > 0mm {
    blank.near chamfer(d: edge) blank.rim
    blank.far chamfer(d: edge) blank.rim
  }
  when bore > 0mm {
    in std.top {
      bore_c := circle(center: o)
      radius(bore / 2) bore_c
    }
    construction bore_cut := solid(std.solids.Prism(face(bore_c), from: -width - 1mm,
      to: width + 1mm))
    bore_cut cut blank
  }
  turn := frame
  turn spins(1) o
  feed := frame
  feed slides(2 * pi * rp) pitch_line
  construction space := solid({ p | some t: p in tooth at(under: feed, seen_from: turn) t })
  body := solid(blank)
  repeat N as i {
    construction indexed := space at(under: turn) i * 360deg / N
    indexed cut body
  }
}
";

/// The gear of `n` teeth, bored `bore` across, its rim chamfered `edge` at both faces, at the
/// benchmark planet's numbers, built exactly and meshed within 0.01 mm: whether it was built as one
/// sector, and its mesh.
fn meshed(n: usize, bore: f64, edge: f64) -> (bool, mesh::Mesh) {
    let src = format!("{HELICAL}\ngear := Helical(N: {n}, mn: 1mm, alpha: 20deg, beta: 20deg, \
        hand: -1, width: 14mm, ha: 0.8mm, hf: 1mm, backlash: 0.04mm, bore: {bore}mm, edge: {edge}mm)\n");
    let e = crate::common::parsed::determined(&src);
    let body = e.map.ent_named("gear.body").unwrap().i();
    let say = Say { stage: &|_: &str| {}, mark: &|_| {} };
    let x = export::exact(&e.sketch, body, None, None, &say)
        .unwrap_or_else(|r| panic!("refused at {:?}: {}", r.stage, r.message));
    x.solid.check(1e-9 * x.solid.size()).unwrap_or_else(|k| panic!("{}", k.msg));
    let m = match &x.pattern {
        Some(p) => p.mesh(0.01, export::ANGULAR),
        None => mesh::mesh(&x.solid, 0.01, export::ANGULAR),
    }.unwrap();
    (x.pattern.is_some(), m)
}

/// The benchmark's case: 0.4 mm chamfers at both faces, built as one sector turned into the 18
/// teeth.  It left one facet of each tooth's flank facing against it, 18 in all.
#[test]
fn a_helical_gear_chamfered_at_both_faces_meshes_facing_out() {
    let (sector, m) = meshed(18, 7., 0.4);
    assert!(sector, "built as one sector");
    assert!(m.sag <= 0.01, "the mesh sags {}", m.sag);
    assert_eq!(m.turned, 0, "triangles facing against their surfaces");
}

/// The same gear unbored, so built whole (its blank reaches the axis): five facets of its flanks
/// faced against them, and two with no chamfer at all.
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, about 10 s: a gear split by 15 sheets at once")]
fn a_helical_gear_built_whole_meshes_facing_out() {
    let (sector, m) = meshed(15, 0., 0.4);
    assert!(!sector, "built whole");
    assert!(m.sag <= 0.01, "the mesh sags {}", m.sag);
    assert_eq!(m.turned, 0, "triangles facing against their surfaces");
}
