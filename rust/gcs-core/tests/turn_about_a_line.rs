//! **`under:` a line is the turn about it** (#748): a radian a radian of the roll about the line
//! (or axis) named where a frame would be, minted once beside the inverse and the relative
//! (`motions::turn_about`) and found again by what it is, never by its name.  Every reader of
//! `under:` takes it, so a point carried to a roll, a solid placed at an instant, a solid swept
//! over an interval and a face's patch swept into a region solid are each read under `ax` as under
//! a frame that `turns(1) ax` — with no frame, so no unknown — and `seen_from:` a line likewise.
use crate::common::{at, ent, text::solved};
use gcs_core::model::MotionDef;
use gcs_core::program::Elaborated;
use gcs_core::solid::ApproximationPolicy;
use gcs_core::space::{add, cross, dot, norm, scale, sub, unit};

/// `x` turned `angle` radians about the line through `o` along the unit `d`: Rodrigues, worked
/// out here rather than asked of the core.
fn turned(x: [f64; 3], o: [f64; 3], d: [f64; 3], angle: f64) -> [f64; 3] {
    let w = sub(x, o);
    let (s, c) = angle.sin_cos();
    add(o, add(add(scale(w, c), scale(cross(d, w), s)), scale(d, dot(d, w) * (1.0 - c))))
}

/// Two skew lines in space, `ax` and `bx`, and a point `x` off both.
const SKEW: &str = "\
unit mm
use std
use std.motion (turns)
a0 := point hint((1, 2, 3))
a1 := point hint((4, 3, 11))
fix((1, 2, 3)) a0
fix((4, 3, 11)) a1
ax := line(a0, a1)
b0 := point hint((-5, 0, 2))
b1 := point hint((-5, 9, 0))
fix((-5, 0, 2)) b0
fix((-5, 9, 0)) b1
bx := line(b0, b1)
x := point hint((9, -4, 6))
fix((9, -4, 6)) x
";

/// **A point carried to a roll under a line** is turned that roll about it, and seen from
/// another line turned back about that one: the closed form, and the same point a frame that
/// `turns(1)` each line carries it to.  One node per line, however many readings name it, and no
/// node named by a prefix.
#[test]
fn a_point_carried_under_a_line_is_turned_about_it() {
    let e = solved(&format!("{SKEW}\
        P := point hint((9, -4, 6))\n\
        P coincident(under: ax, t == 40deg) x\n\
        Q := point hint((9, -4, 6))\n\
        Q coincident(under: ax, seen_from: bx, t == -25deg) x\n\
        R := point hint((9, -4, 6))\n\
        R coincident(under: ax, t == 40deg) x\n\
        spin := frame\n\
        spin turns(1) ax\n\
        back := frame\n\
        back turns(1) bx\n\
        P2 := point hint((9, -4, 6))\n\
        P2 coincident(under: spin, t == 40deg) x\n\
        Q2 := point hint((9, -4, 6))\n\
        Q2 coincident(under: spin, seen_from: back, t == -25deg) x\n"));
    let (a0, d) = ([1.0, 2.0, 3.0], unit([3.0, 1.0, 8.0]));
    let (b0, f) = ([-5.0, 0.0, 2.0], unit([0.0, 9.0, -2.0]));
    let x = [9.0, -4.0, 6.0];
    let p = turned(x, a0, d, 40f64.to_radians());
    let q = turned(turned(x, a0, d, -25f64.to_radians()), b0, f, 25f64.to_radians());
    for (name, want) in [("P", p), ("R", p), ("P2", p), ("Q", q), ("Q2", q)] {
        assert!(norm(sub(at(&e, name), want)) < 1e-9, "{name}: {:?} against {want:?}",
                at(&e, name));
    }
    // the turn about each line minted once, by what it is
    let about: Vec<_> = e.sketch.motions.iter().filter(|m| m.about).collect();
    assert_eq!(about.len(), 2, "{:?}", e.sketch.motions);
    assert!(about.iter().all(|m| matches!(m.def,
        MotionDef::Rotation { ratio: 1.0, advance: 0.0, .. })));
    assert!(e.sketch.motions.iter().all(|m| !m.name.starts_with('#')), "{:?}", e.sketch.motions);
}

/// A 4 × 6 rectangle `sec` in the front plane, 10 from the vertical line `ax` it turns about, and
/// the prism `stock` on it.
const RING: &str = "unit mm\nuse std\nuse std.place (horizontal)\nuse std.motion (turns)\n\
    use std.solids\nin std.front {\n  p0 := point hint((10, 0))\n  p1 := point hint((14, 0))\n\
    p2 := point hint((14, 6))\n  p3 := point hint((10, 6))\n  horizontal (e0 := line(p0, p1)) ->\n\
    vertical (e1 := line(p1, p2)) ->\n  horizontal (e2 := line(p2, p3)) ->\n\
    vertical (e3 := line(p3, p0)) -> close\n  q0 := point\n  q1 := point hint((0, 10))\n\
    ax := vertical line(q0, q1)\n  fix((0, 0)) q0\n  q0 distance(10) q1\n\
    q0 distance(10, along: u) p0\n  q0 horizontal p0\n  p0 distance(4) p1\n  p1 distance(6) p2\n\
    sec := face(e0, e1, e2, e3)\n}\nstock := solid(std.solids.Prism(sec, from: 0mm, to: 2mm))\n";

/// What is read under `m` in the ring: the prism placed at a quarter turn, swept over one, and the
/// section's patch swept over one into a region solid.
fn readings(m: &str) -> String {
    format!("placed := stock at(under: {m}) 90deg\n\
        swept := solid({{ p | p inside(under: {m}, over t in (0deg, 90deg)) stock }})\n\
        patch := solid({{ p | p coincident(under: {m}, over t in (0deg, 90deg)) sec }})\n")
}

/// The volume and the world bounds of the solid `name`, its approximation reported.
fn measured(e: &Elaborated, name: &str) -> (f64, [f64; 3], [f64; 3]) {
    let s = e.sketch.evaluated_solid(ent(e, name).i(), ApproximationPolicy::Report)
        .unwrap_or_else(|m| panic!("`{name}`: {m}"));
    let b = s.world_bounds();
    (s.volume(), b.lo, b.hi)
}

/// **A solid placed, a solid swept and a face's patch swept under a line** are each the solid
/// under a frame that `turns(1)` the line — its volume and where it stands — and reading them adds
/// no frame and no unknown to the drawing.
#[test]
fn solids_under_a_line_are_solids_under_its_turn() {
    let line = solved(&format!("{RING}{}", readings("ax")));
    let frame = solved(&format!("{RING}turn := frame\nturn turns(1) ax\n{}", readings("turn")));
    let near = |a: f64, b: f64| (a - b).abs() <= 1e-6 * (1.0 + b.abs());
    for name in ["placed", "swept", "patch"] {
        let (a, b) = (measured(&line, name), measured(&frame, name));
        assert!(near(a.0, b.0), "{name}'s volume: {} against {}", a.0, b.0);
        for k in 0..3 {
            assert!(near(a.1[k], b.1[k]) && near(a.2[k], b.2[k]),
                    "{name}'s bounds: {a:?} against {b:?}");
        }
    }
    // the prism turned a quarter about the line stands elsewhere, and is the prism still
    let (stock, placed) = (measured(&line, "stock"), measured(&line, "placed"));
    assert!((stock.0 - placed.0).abs() < 1e-9 * stock.0);
    assert!((0..3).any(|k| (stock.1[k] - placed.1[k]).abs() > 1.0), "{stock:?} {placed:?}");
    // the patch's volume is Pappus's: a quarter turn of 24 mm² about a centroid 12 from the line
    let pappus = std::f64::consts::FRAC_PI_2 * 12.0 * 24.0;
    assert!(near(measured(&line, "patch").0, pappus), "{:?}", measured(&line, "patch"));
    // nothing read under the line is a frame or an unknown: the readings add what they add under
    // the frame (the region solids' points), and no frame
    let bare = solved(RING);
    let framed = solved(&format!("{RING}turn := frame\nturn turns(1) ax\n"));
    assert_eq!(line.sketch.frames.len(), bare.sketch.frames.len());
    assert_eq!(line.sketch.params.len() - bare.sketch.params.len(),
               frame.sketch.params.len() - framed.sketch.params.len());
    assert_eq!(line.sketch.motions.iter().filter(|m| m.about).count(), 1);
}
