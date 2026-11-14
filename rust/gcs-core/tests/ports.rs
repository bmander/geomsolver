//! **Ports** (`docs/ports-plan.md`, [0.217]): a relation word's operands typed by interfaces,
//! `s: Journal fitted h: Bore := { … }`.  A use is held to each operand's interface as a call is
//! to a typed formal: whatever the interface asks that the operand lacks is E103 at the operand,
//! naming the member; inside the body an operand holds only what its interface declares (E101
//! otherwise); and a word given what its types ask elaborates to the rows the same word written
//! bare gives.
use crate::common::{built, lenient, parsed::determined};
use gcs_core::brep::props;

/// A pin seated in a block: each part declares a port, a group of its own geometry, and the word
/// relates the two ports — its frame's origin on the face's, and the pin's relief cut from the
/// face's host.  `TYPES` stands where the word's head types its operands, or nothing.
const SEATED: &str = "unit mm
use std
use std.sets
use std.solids (cut)

interface Shoulder { at: point, host: solid }
interface End { at: point, relief: solid }

component Block() {
  in std.front {
    c := point
    std.origin distance(0mm, along: u) c
    std.origin distance(0mm, along: v) c
  }
  body := solid({ q | q distance(<= 20mm) c })
  face := {at: c, host: body}
}

component Pin() {
  in std.front {
    c := point
    std.origin distance(0mm, along: u) c
    std.origin distance(0mm, along: v) c
  }
  construction relief := solid({ q | q distance(<= 5mm) c })
  end := {at: c, relief: relief}
}

pHEAD seats_in hTAIL := {
  p.at coincident h.at
  p.relief cut h.host
}

b := Block()
pin := Pin()
pin.end seats_in b.face
";

fn seated(head: &str, tail: &str) -> String {
    SEATED.replace("pHEAD", &format!("p{head}")).replace("hTAIL", &format!("h{tail}"))
}

/// The volume of `b.body` as the document builds it.
fn block_volume(src: &str) -> f64 {
    props::volume(&built(&determined(src), "b.body").0.solid)
}

#[test]
fn a_typed_word_reads_as_the_bare_word_does() {
    let bare = block_volume(&seated("", ""));
    let typed = block_volume(&seated(": End", ": Shoulder"));
    let want = 4. / 3. * std::f64::consts::PI * (20f64.powi(3) - 5f64.powi(3));
    assert!((bare - want).abs() < 1e-6 * want, "{bare}");
    assert!((typed - bare).abs() < 1e-9 * want, "{typed} against {bare}");
}

#[test]
fn an_operand_missing_a_member_its_interface_asks_for_is_refused_at_the_use() {
    // the face handed where the pin's end is asked for: it has no `relief`
    let src = seated(": End", ": Shoulder").replace("pin.end seats_in b.face", "b.face seats_in b.face");
    let (_, said) = lenient::read(&src);
    assert!(said.iter().any(|d| d.starts_with("E103") && d.contains("`b.face` has no `relief`")
                            && d.contains("`End`")), "{said:?}");
}

#[test]
fn the_body_reads_only_what_the_interface_declares() {
    // `End` says nothing of a host, though the pin's group would not have one either: the body is
    // refused for reading past its type, wherever it is used
    let src = seated(": End", ": Shoulder").replace("p.relief cut h.host", "h.relief cut p.relief");
    let (_, said) = lenient::read(&src);
    assert!(said.iter().any(|d| d.starts_with("E101")), "{said:?}");
}

#[test]
fn a_type_naming_no_interface_is_refused_where_it_is_written() {
    let (_, said) = lenient::read(&seated(": Ending", ": Shoulder"));
    assert!(said.iter().any(|d| d.starts_with("E101") && d.contains("Ending")), "{said:?}");
}

/// A housing and a shaft along one axis, a 6804 seated in the housing at a station and held by a
/// DIN 471 ring grooved into the shaft — written with the mates (`seats_in`, `abuts`, `grooves`)
/// or with the bundle `Held`, which `JOINT` stands for.
const FITTED: &str = "unit mm
use std
use std.sets
use std.ports (seats_in, abuts, grooves)
use std.bearings
use std.retaining
use std.place (vertical)

in std.front {
  up := point
  construction ax := line(std.origin, up)
  vertical ax
  length(10mm) ax
  std.origin distance(>= 0mm, along: v) up
}
housing := solid({ q | q in std.sets.Cylinder(ax, r: 25mm)
                       std.origin distance(in: (-10mm, 10mm), along: ax) q })
shaft := solid({ q | q in std.sets.Cylinder(ax, r: 10mm)
                     std.origin distance(in: (-30mm, 30mm), along: ax) q })
journal := {host: shaft}
st := std.ports.Station(ax, std.origin, d: 0mm, toward: std.z, east: std.x, host: housing)
JOINT
";

const MATES: &str = "b := std.bearings.Bearing(std.bearings.b6804)
b.od seats_in st.seat
clip := std.retaining.Ring(std.retaining.din471_20)
clip.face abuts b.left
clip.seat grooves journal";

const BUNDLE: &str = "j := std.bearings.Held(std.bearings.b6804, ring: std.retaining.din471_20,
  seat: st.seat, on: journal)";

/// The volumes of `names` as `src` builds them.
fn volumes(src: &str, names: &[&str]) -> Vec<f64> {
    let e = determined(src);
    names.iter().map(|n| props::volume(&built(&e, n).0.solid)).collect()
}

#[test]
fn a_bearing_seated_by_its_port_takes_its_room_and_its_ring_its_groove() {
    use std::f64::consts::PI;
    let v = volumes(&FITTED.replace("JOINT", MATES), &["housing", "shaft"]);
    // the housing less the bearing's room, its outside's cylinder a millimetre past either side
    let housing = PI * 625. * 20. - PI * 256. * 9.;
    // the shaft less the ring's groove, 1.2 mm wide from the groove's 19 mm out past its 20 mm
    let shaft = PI * 100. * 60. - PI * (100. - 90.25) * 1.2;
    assert!((v[0] - housing).abs() < 1e-6 * housing, "{} against {housing}", v[0]);
    assert!((v[1] - shaft).abs() < 1e-6 * shaft, "{} against {shaft}", v[1]);
}

#[test]
fn a_joint_of_joints_is_the_joints_it_is_written_as() {
    let mates = volumes(&FITTED.replace("JOINT", MATES), &["housing", "shaft"]);
    let bundle = volumes(&FITTED.replace("JOINT", BUNDLE), &["housing", "shaft"]);
    for (a, b) in mates.iter().zip(&bundle) {
        assert!((a - b).abs() < 1e-9 * a, "{a} against {b}");
    }
}

#[test]
fn a_part_handing_out_its_datum_in_a_port_stands_in_a_frame_its_mate_places() {
    // the bearing's ring abutting the bearing's left side: the ring's frame stands below the
    // bearing's, though nothing outside reads either's `std`
    let e = determined(&FITTED.replace("JOINT", MATES));
    let at = |name: &str| {
        let p = crate::common::ent(&e, name).i();
        e.sketch.world_point(p)
    };
    let (b, ring) = (at("b.lf.at.origin"), at("clip.ff.at.origin"));
    assert!((b[2] + 3.5).abs() < 1e-9 && (ring[2] + 3.5).abs() < 1e-9, "{b:?} {ring:?}");
}

#[test]
fn a_part_joined_only_by_a_side_it_frames_from_its_datums_stands_in_a_frame_of_its_own() {
    // the bearing against a shoulder 20 mm up the axis, joined by its left side alone: `left` is
    // `{at: lf.at}`, `lf` a frame the bearing stands on its own `std.z`, so the bearing has a
    // frame of its own for the mate to place (where it stood in the world, and the mate
    // conflicted with it)
    let src = FITTED.replace("JOINT", "sf := std.ports.FrameOn(ax, std.origin, d: 20mm, toward: std.z, east: std.x)
shoulder := {at: sf.at}
b := std.bearings.Bearing(std.bearings.b6804)
b.left abuts shoulder");
    let e = determined(&src);
    let at = |name: &str| {
        let p = crate::common::ent(&e, name).i();
        e.sketch.world_point(p)
    };
    let (left, right) = (at("b.lf.at.origin"), at("b.rf.at.origin"));
    assert!((left[2] - 20.).abs() < 1e-9 && (right[2] - 27.).abs() < 1e-9, "{left:?} {right:?}");
}
