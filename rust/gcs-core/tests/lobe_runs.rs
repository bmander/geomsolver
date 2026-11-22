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
fn drums() -> [Outline; 2] {
    [stated(&lobed(2, 30.0, 90.0, 20.0)), stated(&lobed(3, 20.0, 90.0, 20.0))]
}

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
    let text = word.text();
    document(o, cap, word).replace(&format!("{text} plate\n"), &format!("{text} {faces}\n"))
}

/// A disc of 200 about the origin, 10 thick, through which `o` is a hole (its faces `hole.k0`,
/// …), its edge on `cap` taken by `word` along `faces`.
fn bored(o: &Outline, cap: &str, word: Word, faces: &str) -> String {
    rimmed(o, cap, word, faces)
        .replace("  outline := face(", "  oc := point\n  fix((0, 0)) oc\n  \
                  rim := circle(center: oc) hint(r: 200)\n  radius(200mm) rim\n  hole := face(")
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
                    let export = d == 0 && cap == "near"
                        && matches!(word, Word::Round(r) if r == 5.0);
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
                    let export = d == 1 && cap == "far"
                        && matches!(word, Word::Round(r) if r == 3.0);
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
        let src = bored(&o, "near", Word::Round(3.0), "plate.hole");
        close(built(&src, false), disc_less(&o) - bore);
    }
}


/// The drum of four arcs (lobes of 90 about centres 30 either side, waists of 20) with each waist
/// moved in toward its lobes until they meet at a turn `theta` rather than tangent: the circles
/// cross, and each seam is the crossing on the side `way` takes (`1`: the outline's corner reflex
/// there, `-1`: convex) — a stated tangency rounded off by `theta`.
fn creased(theta: f64, way: f64) -> Outline {
    let (n, d, rl, rw) = (2usize, 30.0, 90.0, 20.0);
    let half = std::f64::consts::PI / n as f64;
    // (the centres `rl + rw − δ` apart meet at the turn `θ` with `δ = θ² / 2(1/rl + 1/rw)`)
    let apart = rl + rw - theta * theta / (2.0 * (1.0 / rl + 1.0 / rw));
    let e = d * half.cos() + (apart.powi(2) - (d * half.sin()).powi(2)).sqrt();
    let at = |r: f64, a: f64| [r * a.cos(), r * a.sin()];
    let th = ((apart * apart + rl * rl - rw * rw) / (2.0 * apart * rl)).acos();
    let hit = |l: [f64; 2], w: [f64; 2], sign: f64| {
        let a = (w[1] - l[1]).atan2(w[0] - l[0]) + sign * th;
        [l[0] + rl * a.cos(), l[1] + rl * a.sin()]
    };
    let mut o = Vec::new();
    for k in 0..n {
        let (a, b) = (2.0 * half * k as f64, 2.0 * half * (k + 1) as f64);
        let (lobe, waist, next, prev) = (at(d, a), at(e, a + half), at(d, b), at(e, a - half));
        o.push(Seg::Arc { c: lobe, s: hit(lobe, prev, way), e: hit(lobe, waist, -way), ccw: true });
        let (into, out) = (hit(lobe, waist, -way), hit(next, waist, way));
        o.push(Seg::Arc { c: waist, s: into, e: out, ccw: false });
    }
    o
}

/// The refusal of document `src`'s plate, as text.
fn refused(src: &str) -> String {
    let e = solved(src);
    assert!(e.diags.is_empty(), "{:?}\n{src}", e.diags);
    let say = gcs_core::brep::sweep::Say { stage: &|_: &str| {}, mark: &|_| {} };
    let body = crate::common::ent(&e, "plate").i();
    gcs_core::brep::export::exact(&e.sketch, body, None, None, &say).err().expect("refused").message
}

/// **A chamfer at a corner too nearly tangent, said as a round's is** (#994): the drum's seams a
/// stated tangency rounded off by `2e-6` to `5e-5` rad, under the kernel's bound (`0.00035` rad on
/// the drum, `0.00048` on the disc it bores): the chamfer's piece has a strip of the edge's own
/// face, which the face it ends on meets at the corner and again `2R sin θ` on, two meetings the
/// kernel cannot tell apart — refused by the faces, the turn and the bound, as a round is (before,
/// by the kernel's words: "a run cut off at a curved end: a shell of the arrangement is kept on one
/// side of a cylinder"), along a lobe, a waist and a bore's lobe, turned or not.  Past the bound
/// (`1e-2`) each builds, as before.
#[test]
fn a_chamfer_at_a_corner_too_nearly_tangent_is_said_by_its_faces() {
    let word = Word::Chamfer(2.0);
    for (theta, way, turn) in [(2e-6, 1.0, 0.0), (1e-5, -1.0, 0.5), (5e-5, 1.0, 2.0)] {
        let o = moved(&creased(theta, way), false, turn);
        let cases = if way > 0.0 {
            [rimmed(&o, "near", word, "plate.k0"), rimmed(&o, "near", word, "plate.k1"),
             bored(&o, "near", word, "plate.hole.k0")]
        } else {
            [rimmed(&o, "near", word, "plate.k1"), bored(&o, "far", word, "plate.hole.k0"),
             bored(&o, "near", word, "plate.hole.k1")]
        };
        for src in cases {
            let refusal = refused(&src);
            let turned = format!("meet at a turn of {} rad", gcs_core::json::fmt_g(theta, 2));
            assert!(refusal.contains(&turned), "{refusal}");
            let said = "too nearly tangent a corner for this kernel to chamfer at 2mm against";
            assert!(refusal.contains(said), "{refusal}");
            assert!(refusal.contains("rad or more is built). State the tangency"), "{refusal}");
            assert!(!refusal.contains("arrangement") && !refusal.contains("pieces"), "{refusal}");
        }
    }
    for way in [1.0, -1.0] {
        let o = creased(1e-2, way);
        for src in [rimmed(&o, "near", word, "plate.k0"), rimmed(&o, "far", word, "plate.k1"),
                    bored(&o, "near", word, "plate.hole.k0"),
                    bored(&o, "near", word, "plate.hole.k1")] {
            built(&src, false);
        }
    }
}
