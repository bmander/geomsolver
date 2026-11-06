//! **A characteristic that closes on itself** (#684 W): a tool passed wholly through the blank under
//! a slide or a turn touches it along a curve that closes on itself within the blank's reach, and
//! the walk comes round to where it began. The loop is begun again at a crease, where the sweep has
//! a sharp edge, and its sheet is a tube: the characteristic carried, its last row its first, cut
//! into faces at its creases that close on each other along it (`Characteristic::closed`,
//! `build::grid_sheet`). A drum pushed edge first through a plate punches its shadow
//! (`solid_sweeps/punch.sv`, `solid_sweeps_examples::the_drum_punches_its_shadow_through_a_plate`),
//! a bull-nosed drum its shadow with the corners rounded, and a drum turned through a ring sector,
//! its axis along the turn's, bores a tunnel of its section round the axis: each volume a closed
//! form, each exact solid's STL agreeing with the field. Refused by name: the drum turned a whole
//! turn, its sheet closed both ways; and screwed, its flat ends no longer kept in their planes. (A
//! prism slid aslant wholly through, its silhouette turning at a vertex, and a prism turned wholly
//! within, a rim's ruling in contact all along it, which this class refuses, are the closed
//! class's: `closed_class::a_bar_pushed_aslant_through_a_plate_takes_its_shadow`,
//! `closed_class::a_leaning_bar_turned_wholly_through_a_ring_sector_bores_its_shadow`.)
use gcs_core::solid::admission::Class;
use std::f64::consts::PI;

use crate::flat_faces::turned_mill;
use crate::solid_sweeps_examples::{class,text};
use crate::turns_and_slides::{exported,exported_at,near};

/// `punch.sv` with its plate named `part`, as `admit` reads it.
pub(crate) fn punch() -> String { text("punch.sv").replace("plate := solid","part := solid").replace("slot cut plate","slot cut part") }

/// A drum standing upright 10 from z, radius 1 and 1 high about the ring sector's middle height
/// (`flat_faces::turned_mill`'s mill cut short), turned about z from clear of the sector to clear.
pub(crate) fn turned_drum() -> String { turned_mill(1.,1.,10.,-0.5).replace("dv: 4mm","dv: 1mm") }

/// Turned about z with its axis along the turn's, the drum's flat ends are kept in their planes and
/// its side touches along its two lines in the plane through both axes, nearest and farthest the
/// axis: the characteristic runs up the inner line, across the top along the line from the axis
/// through its centre, down the outer line and back across the bottom, closing on itself within the
/// ring sector, creased at four corners. Turned, it bores a tunnel of its section, 2 × 1 from 9 to
/// 11 from the axis, through the sector's 60°: `π/3 (11² − 9²)/2` by Pappus, `20π/3`.
#[test]
fn a_drum_turned_through_a_ring_sector_bores_a_tunnel() {
    let e = fixtures::read(&turned_drum());
    let Class::ConstantTwist(found) = class(&e,"part") else { panic!("the constant-twist class") };
    assert!(found.screw.is_turn() && !found.whole && found.closed,"a turn, its characteristic closed");
    assert_eq!(found.reach,[0,found.nodes.len()-1]);
    assert_eq!(found.nodes[0].position,found.nodes[found.nodes.len()-1].position,"its last node its first");
    let mut flats = found.flats();
    flats.sort();
    assert_eq!(flats,["`mill.bottom`","`mill.top`"]);
    assert_eq!(found.creases().len(),4);
    near(exported(&e,"part"),PI/3.*80.-PI/3.*20.,1e-5);
}

/// The drum pushed through the plate rounded where its flat ends meet its side, by quarter rounds
/// of radius 0.5: each round's ring where it meets the flat is in contact all round, and the walk
/// runs on from the flat's line across it at the line's ends (#684 O), round each corner of the
/// shadow and on along the side's line. The slot is the drum's shadow along the push, 8 × 4 with
/// its corners rounded to 0.5, `32 − (4 − π)/4`, through the plate's 4: within 2e-6 at 0.001 mm
/// (1.2e-6, the fit over the four rounds leaving the slot 9 nm narrow on average; at 0.01 mm
/// 1.9e-5, as the bull-nose mill's slot reads there, `flat_faces`).
#[test]
fn a_bull_nosed_drum_punches_its_shadow_rounded() {
    let text = punch().replace("  e1 coords(du: radius, dv: 0mm) bench\n  e2 coords(du: radius, dv: drum) bench\n",
        "  e1 coords(du: 1.5mm, dv: 0mm) bench\n  e2 coords(du: radius, dv: 7.5mm) bench\n  private k1 := point\n  \
        k1 coords(du: 1.5mm, dv: 0.5mm) bench\n  private e5 := point\n  e5 coords(du: radius, dv: 0.5mm) bench\n  \
        private k2 := point\n  k2 coords(du: 1.5mm, dv: 7.5mm) bench\n  private e6 := point\n  e6 coords(du: 1.5mm, dv: drum) bench\n")
        .replace("  private side := line(e1, e2)\n  private far_face := line(e2, e3)\n",
        "  private near_round := arc(center: k1, start: e1, end: e5) hint(r: 0.5)\n  radius(0.5mm) near_round\n  \
        private side := line(e5, e2)\n  private far_round := arc(center: k2, start: e2, end: e6) hint(r: 0.5)\n  \
        radius(0.5mm) far_round\n  private far_face := line(e6, e3)\n")
        .replace("face(near_face, side, far_face, spindle)","face(near_face, near_round, side, far_round, far_face, spindle)");
    let e = fixtures::read(&text);
    let Class::ConstantTwist(found) = class(&e,"part") else { panic!("the constant-twist class") };
    assert!(found.screw.is_slide() && found.closed,"a slide, its characteristic closed");
    near(exported_at(&e,"part",0.001),800.-4.*(32.-(4.-PI)/4.),2e-6);
}

/// **The drum turned a whole turn bores the whole ring** (#721 F2c5, a curved tool under a turn): its
/// sheet would close round the turn as well as along the characteristic, a ring with no edge, which
/// the constant-twist class refuses (S1); the closed class builds it on one arrangement, the drum's
/// side's generators in the plane through both axes turned into the cylinders of radii 9 and 11,
/// its flat ends kept in their planes. The ring loses the annulus between them through the drum's
/// height within its 60°, `π/6 (11² − 9²)`.
#[test]
fn the_drum_turned_a_whole_turn_bores_the_whole_ring() {
    let e = fixtures::read(&turned_drum().replace("(-140deg, 140deg)","(0deg, 360deg)"));
    let Class::Closed(found) = class(&e,"part") else { panic!("the closed class") };
    assert!(found.screw().unwrap().is_turn(),"a turn");
    near(exported(&e,"part"),PI/3.*80.-PI/6.*40.,1e-9);
}

/// **The drum screwed half a millimetre a turn bores its helical groove** (#721 F2c5, a round tool
/// under a screw): its flat ends kept in no plane, both of each ring's points on its side reach the
/// blank with no flat between them — the constant-twist class's S2 — and the closed class builds it,
/// its side's generators screwed into cylinders about the axis, its rims into the solids their discs
/// sweep. Apart from the build: the drum passes wholly across the sector, so at each point of it the
/// heights it covers are its own height and twice its half-width in angle there, `a(ρ)`, times the
/// pitch `k = 0.5/2π`: within the sector's heights, `π/3 ∫ ρ (1 + 2k a(ρ)) dρ = π/3 (20 + k π)`.
/// The kernel's own gate holds the same cut to the membership test
/// (`swept_closed::a_drum_screwed_across_a_ring_sector_bores_its_helical_groove`); its STL agrees with
/// the field, the groove's floor folding against its wall over a ramp thinner than the bar meshed by
/// the wall's chords halved where the triangles beside them turned (`mesh::mesh_with`).
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, about 15 s: a drum's screwed rims through a sector")]
fn the_drum_screwed_bores_its_helical_groove() {
    let screwed = turned_drum().replace("use std.motion (turns)","use std.motion (screws)").replace("swing turns(1) ax","swing screws(0.5mm) ax");
    let e = fixtures::read(&screwed);
    let Class::Closed(found) = class(&e,"part") else { panic!("the closed class") };
    assert_eq!(found.kind(),"a screw");
    let k = 0.5/std::f64::consts::TAU;
    near(exported(&e,"part"),PI/3.*80.-PI/3.*(20.+k*PI),1e-8);
}

/// The square bar pushed square to its length keeps its section's plane: planar subtraction's.
#[test]
fn a_square_bar_pushed_square_to_its_length_is_planar_subtractions() {
    let bar = square_bar();
    assert!(matches!(class(&fixtures::read(&bar),"part"),Class::Subtraction(_)),"read by the planar subtraction class");
}

/// `punch.sv` with its drum a bar of square section stood on a corner, `|y| + |z + 6| ≤ 1` along x
/// from 6 to 14, pushed straight up through the plate.
pub(crate) fn square_bar() -> String {
    punch().replace("construction tool := solid(std.solids.Revolution(face(near_face, side, far_face, spindle), about: spindle))","in std.side {
  private c0 := point
  private c1 := point
  private c2 := point
  private c3 := point
  c0 coords(du: 0mm, dv: -7mm) std.side
  c1 coords(du: 1mm, dv: -6mm) std.side
  c2 coords(du: 0mm, dv: -5mm) std.side
  c3 coords(du: -1mm, dv: -6mm) std.side
  private lower := line(c0, c1)
  private upper := line(c1, c2)
  private outer := line(c2, c3)
  private under := line(c3, c0)
}
construction tool := solid(std.solids.Prism(face(lower, upper, outer, under), from: 6mm, to: 14mm))")
}
