//! **A round or a chamfer along one arc of a chain of tangent arcs** (#994 item 5, fixture 125: R5
//! along one lobe of a drum whose outline is convex lobes of R90 joined by concave waists, and R3
//! on its bore's edge).  The run's edge ends where its wall goes on tangent into the next arc's,
//! a seam and no corner: that face meets the run's other face (the cap) in an edge of the run's
//! own turn, along which a round would go on, so it carries the run on and ends nothing
//! (`BodyFaces::carries_on`).  The run ends on no face there, and what `std.rounds.Run` says is
//! cut off by `distance(p, crease) <= distance(p, meeting)` alone: the meeting is the whole circle
//! the cap and the arc's cylinder meet in, so past the meridian through the seam a point's
//! nearest point of it is off the crease.  The stretch ends square in that meridian — through
//! both arcs' axes, the waist's as well as the lobe's — and is the ring's sector exactly: no
//! curved face is cut at all, so nothing is asked of a cut that would remove nothing.  Each
//! volume is the closed form's to 1e-9: the outline's area by Green's theorem times the
//! thickness, less the section turned through the arc's angle about its axis (Pappus), its
//! centroid `R − inset` from the axis where the material is convex there and `R + inset` where it
//! is concave.
use gcs_core::brep::props;

use crate::common::rounds::{exact, exported};
use crate::common::{close, text::solved};
use crate::rim_seams::{area, document, lobed, moved, stated, taken, Outline, Seg, Word};

/// The drums: two lobes of 90 about centres 30 either side of the origin and two waists of 20
/// (four arcs), and three lobes of 90 about centres 20 out and three waists of 20 (six arcs) —
/// each lobe `k0`, `k2`, …, each waist `k1`, `k3`, …, every seam tangent, stated to the micron as
/// an author would.
fn drums() -> [Outline; 2] { [stated(&lobed(2, 30.0, 90.0, 20.0)), stated(&lobed(3, 20.0, 90.0, 20.0))] }

/// What `word` takes along arc `k` of `o` alone: its section turned through the arc's angle about
/// its axis, the centroid `R − inset` out where the material is convex there (the arc turns the
/// way the outline does, or the other way where `o` is a hole) and `R + inset` where it is
/// concave.
fn stretch(o: &Outline, k: usize, word: Word, hole: bool) -> f64 {
    let (a, inset) = word.section();
    let Seg::Arc { c, s, .. } = o[k] else { panic!("arc {k} is a line") };
    let radius = (s[0] - c[0]).hypot(s[1] - c[1]);
    let phi = o[k].sweep();
    let convex = (phi * area(o) > 0.0) != hole;
    a * (if convex { radius - inset } else { radius + inset }) * phi.abs()
}

/// The plate through `o`, 10 thick, its rim on `cap` taken by `word` along `faces` alone.
fn rimmed(o: &Outline, cap: &str, word: Word, faces: &str) -> String {
    document(o, cap, word).replace(&format!("{} plate\n", word.text()), &format!("{} {faces}\n", word.text()))
}

/// A disc of 200 about the origin, 10 thick, through which `o` is a hole (its faces `hole.k0`,
/// …), its edge on `cap` taken by `word` along `faces`.
fn bored(o: &Outline, cap: &str, word: Word, faces: &str) -> String {
    rimmed(o, cap, word, faces)
        .replace("  outline := face(", "  oc := point\n  fix((0, 0)) oc\n  rim := circle(center: oc) hint(r: 200)\n  \
                  radius(200mm) rim\n  hole := face(")
        .replace("\n}\nplate", "\n  outline := face(rim, holes: hole)\n}\nplate")
}

/// The disc's volume less its bore `o`, 10 thick.
fn disc_less(o: &Outline) -> f64 { (std::f64::consts::PI * 200.0 * 200.0 - area(o).abs()) * 10.0 }

/// Document `src` solved, its plate built exactly (and exported, with its STEP and its STL read
/// back, where `export`), and its volume.
fn built(src: &str, export: bool) -> f64 {
    let e = solved(src);
    assert!(e.diags.is_empty(), "{:?}\n{src}", e.diags);
    props::volume(&if export { exported(&e, "plate") } else { exact(&e, "plate") })
}

/// **One lobe, one waist** (fixture 125): the rim along a lobe of 90 alone (convex) and along a
/// waist of 20 alone (concave), each stretch ending square in the meridian through its seams;
/// rounded 5 and 3 and chamfered 2, on the top cap and the bottom, on the drum of four arcs and of
/// six.
#[test]
fn one_arc_of_a_drum_is_taken_to_its_seams() {
    for (d, o) in drums().iter().enumerate() {
        for word in [Word::Round(5.0), Word::Round(3.0), Word::Chamfer(2.0)] {
            for cap in ["near", "far"] {
                for k in [0, 1] {
                    let src = rimmed(o, cap, word, &format!("plate.k{k}"));
                    let export = d == 0 && cap == "near" && matches!(word, Word::Round(r) if r == 5.0);
                    close(built(&src, export), area(o).abs() * 10.0 - stretch(o, k, word, false));
                }
            }
        }
    }
}

/// **The same arcs mirrored and turned**: a lobe and a waist, round and chamfer, the drum of four
/// arcs mirrored, the drum of six mirrored and turned, and the drum of four turned alone.
#[test]
fn one_arc_of_a_drum_mirrored_and_turned() {
    let [four, six] = drums();
    for (o, mirror, turn) in [(&four, true, 0.0), (&six, true, 0.7), (&four, false, 2.0)] {
        let o = moved(o, mirror, turn);
        for (k, word) in [(0, Word::Round(5.0)), (1, Word::Round(3.0)), (0, Word::Chamfer(2.0))] {
            let src = rimmed(&o, "near", word, &format!("plate.k{k}"));
            close(built(&src, false), area(&o).abs() * 10.0 - stretch(&o, k, word, false));
        }
    }
}

/// **The bore's edge along one arc** (fixture 125's R3 on its inner edge): the drum's outline as
/// a hole through a disc, its edge taken along one lobe (concave there: the material is outside
/// it) and along one waist (convex), round 3 and 5 and chamfer 2, on either cap.
#[test]
fn one_arc_of_a_bore_is_taken_to_its_seams() {
    for (d, o) in drums().iter().enumerate() {
        for word in [Word::Round(3.0), Word::Round(5.0), Word::Chamfer(2.0)] {
            for cap in ["near", "far"] {
                for k in [0, 1] {
                    let src = bored(o, cap, word, &format!("plate.hole.k{k}"));
                    let export = d == 1 && cap == "far" && matches!(word, Word::Round(r) if r == 3.0);
                    close(built(&src, export), disc_less(o) - stretch(o, k, word, true));
                }
            }
        }
    }
}

/// **The whole rim as before**: the drum's top rim rounded 5 and its bore's edge rounded 3, every
/// arc's stretch glued to the next at its seam (`tests/rim_seams.rs`).
#[test]
fn a_drum_and_its_bore_are_rimmed_whole() {
    for o in drums() {
        close(built(&rimmed(&o, "near", Word::Round(5.0), "plate"), false),
              area(&o).abs() * 10.0 - taken(&o, Word::Round(5.0)));
        let bore: f64 = (0..o.len()).map(|k| stretch(&o, k, Word::Round(3.0), true)).sum();
        close(built(&bored(&o, "near", Word::Round(3.0), "plate.hole"), false), disc_less(&o) - bore);
    }
}

