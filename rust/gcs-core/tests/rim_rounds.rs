//! **A round over a rounded body** (#879): a box's four upright edges rounded, then its rims
//! rounded or chamfered round the corners' arcs as one tangent run — the faces a round makes
//! listed to a block by the names the body's boundary has (`round`, `flat`), which faces meet read
//! in the layer the round that asks stands in, and the kernel's meeting of two rounds tangent to
//! one face ending at the point where both touch it; and a plate's top rim rounded past a reflex
//! corner a hair off tangent, its run cut off by the cylinder it leaves (#961 item 2).  Each
//! volume is the closed form's to 1e-9.
use gcs_core::brep::props;
use gcs_core::program::Code;
use std::f64::consts::PI;

use crate::common::linked::build as elaborated;
use crate::common::rounds::{exact, exported, exported_volume, inset, kinds, square_run};
use crate::common::{close, text::solved};

/// A box 50 × 25 × 10 drawn on the top plane, its sides `ab`, `bc`, `cd`, `da`, its caps `near`
/// (z = 10) and `far` (z = 0).
const BOX: &str = "unit mm\nuse std\nuse std.solids\nuse std.rounds (round, chamfer)\n\
    in std.top {\n  loop := step(u: 50mm, v: 0mm) (ab := line) -> step(v: 25mm) (bc := line) ->\n\
    step(u: 0mm) (cd := line) -> step(v: 0mm) (da := line) -> close\n}\n\
    box := solid(std.solids.Prism(loop, from: 0mm, to: 10mm))\n";

/// The four upright edges rounded 5, the statement in the block given `name` (`vert := `) or none.
fn upright(name: &str) -> String {
    format!("repeat f in box {{ repeat g in box {{ when f parallel std.z and g parallel std.z \
             and f != g {{ {name}f round(r: 5mm) g }} }} }}\n")
}

/// The box less its four corners rounded 5.
const CORNERED: f64 = 50.0 * 25.0 * 10.0 - (4.0 - PI) * 25.0 * 10.0;
/// The straight edges of a rim of the box with its corners rounded 5.
const STRAIGHT: f64 = 2.0 * (40.0 + 15.0);

/// What a ball of radius `r` cannot reach along a rim of the cornered box (`whole`), or round its
/// four arcs alone: its section along the straight edges and turned once about an arc's axis
/// (Pappus: the section's centroid stands `(10 − 3π) / (3 (4 − π)) r` in from each face).
fn rounded(r: f64, whole: bool) -> f64 {
    let inset = (10.0 - 3.0 * PI) / (3.0 * (4.0 - PI)) * r;
    square_run(r) * (2.0 * PI * (5.0 - inset) + if whole { STRAIGHT } else { 0.0 })
}

/// What a flat `d` along each face takes of a rim of the cornered box: its triangle the same.
fn chamfered(d: f64) -> f64 {
    0.5 * d * d * (2.0 * PI * (5.0 - d / 3.0) + STRAIGHT)
}

/// **The rim follows the corners' rounds** (`rounds/rim.sv`): the top rim rounded 1 and the bottom
/// rim chamfered 1 on a solid of the box whose upright edges are rounded 5 — a run along each
/// straight edge and a quarter ring round each arc, glued at the seams.  Ten planes (the caps,
/// the sides, the chamfer's four flats), eight cylinders (the corners, the round's four runs),
/// four tori and four cones round the arcs.
#[test]
fn a_rim_rounded_over_rounded_corners_is_one_tangent_run() {
    let e = solved(include_str!("../../examples/rounds/rim.sv"));
    assert!(e.diags.is_empty(), "{:?}", e.diags);
    let b = exported(&e, "part");
    close(props::volume(&b), CORNERED - rounded(1.0, true) - chamfered(1.0));
    assert_eq!(kinds(&b), [("cone", 4), ("cylinder", 8), ("plane", 10), ("torus", 4)].into());
}

/// **The top and the bottom alike, a round and a chamfer alike**: each of the four on its own.
#[test]
fn a_rim_over_rounded_corners_is_built_on_either_cap() {
    for cap in ["near", "far"] {
        for (word, taken) in [("round(r: 1mm)", rounded(1.0, true)),
                              ("chamfer(d: 1mm)", chamfered(1.0))] {
            let e = solved(&format!("{BOX}{}part := solid(box)\npart.box.{cap} {word} part.box\n",
                                    upright("")));
            assert!(e.diags.is_empty(), "{cap} {word}: {:?}", e.diags);
            close(props::volume(&exact(&e, "part")), CORNERED - taken);
        }
    }
}

/// **A round over the face a round made** ([0.149]): the corners' round named, and the top
/// rounded against its faces by a statement that names it — named itself or not — is the four
/// quarter rings, each ending square at the seams where a side goes on.  The round's face is
/// listed as the body's boundary names it, `vert.taken.round`: named by its set (`run`), a block
/// over `box.vert` paired the top with a face the body has not, and made nothing.
#[test]
fn a_round_over_a_named_rounds_faces_rounds_their_edges() {
    for rim in ["rim := ", ""] {
        for (cap, word, taken) in [("near", "round(r: 1mm)", rounded(1.0, false)),
                                   ("far", "chamfer(d: 1mm)", 0.5 * 2.0 * PI * (5.0 - 1.0 / 3.0))] {
            let e = elaborated(&format!("{BOX}{}{rim}box.{cap} {word} box.vert\n",
                                        upright("vert := ")));
            assert!(e.diags.is_empty(), "{rim}{cap} {word}: {:?}", e.diags);
            close(props::volume(&exact(&e, "box")), CORNERED - taken);
        }
    }
}

/// **Which faces meet is read where the round that asks reads them** (#879): `ab` and `bc` meet
/// in the box the corner's round reads, and no longer in the one a round over its face reads.
/// Read off the finished body alone, the corner's `when b meets a` came out false once the second
/// round stood over it, the corner was not made, and the elaboration went round until it gave up
/// with the second round naming nothing (W116).  One corner, and a rim over its face.
#[test]
fn faces_meet_in_the_layer_of_the_round_that_asks() {
    let e = elaborated(&format!("{BOX}vert := box.ab round(r: 5mm) box.bc\n\
                                 rim := box.near round(r: 1mm) box.vert\n"));
    assert!(e.diags.is_empty(), "{:?}", e.diags);
    // one corner of the four the arcs alone take round
    let arc = rounded(1.0, false) / 4.0;
    close(props::volume(&exact(&e, "box")), 12500.0 - square_run(5.0) * 10.0 - arc);
    // each round's faces under its own name, a name once however many faces bear it
    let listed = gcs_core::solid::faces::face_paths(&e.sketch, crate::common::ent(&e, "box").i());
    let names: Vec<String> = listed.iter().map(|f| f.text()).collect();
    for want in ["vert.taken.round", "rim.taken.round"] {
        assert_eq!(names.iter().filter(|n| *n == want).count(), 1, "{names:?}");
    }
}

/// **A path that names no face still says so** (W116), with a round's faces among those it could
/// have named.
#[test]
fn a_round_over_no_face_is_said() {
    let e = elaborated(&format!("{BOX}vert := box.ab round(r: 5mm) box.bc\n\
                                 rim := box.near round(r: 1mm) box.verts\n"));
    let said: Vec<&str> = e.diags.iter().filter(|d| d.code == Code::W116)
        .map(|d| d.message.as_str()).collect();
    let names = |m: &str| m.contains("`box.verts` names no face of `box`") && m.contains("`vert`");
    assert!(matches!(said[..], [m] if names(m)),
            "{:?}", e.diags);
}

/// **Rounds that name nothing of each other each read the body before either, on either cap**
/// (#879): a rim's run and a corner's round then only cross at the box's sharp corner — the
/// rim's cylinder and the corner's meet in a curve from the point where both touch a side to the
/// point where the rim's touches the cap.  The kernel's march stepped across the first and ended
/// past it, off the corner's face; the curve ran through that vertex twice, one stretch of it was
/// never laid, and two cells joined across it: the top was refused where the bottom, its steps
/// falling otherwise, was built.  Now both are, and to one volume.
#[test]
fn rounds_that_cross_at_a_corner_are_built_on_either_cap() {
    for word in ["round(r: 1mm)", "chamfer(d: 1mm)"] {
        let volume = |cap: &str| {
            let e = solved(&format!("{BOX}{}box.{cap} {word} box\n", upright("")));
            props::volume(&exact(&e, "box"))
        };
        let (near, far) = (volume("near"), volume("far"));
        close(near, far);
        // short of the box with its corners rounded, by less than the rim taken whole
        assert!(near < CORNERED && near > CORNERED - 4.0 * rounded(1.0, true), "{word}: {near}");
    }
}

/// **A round that does not build is said by its body and its label** (#879): its solid is named
/// by the use of the word that made it (`#17.0.….taken`), which no document wrote, so the recipe
/// says what a failure is to call it.
#[test]
fn a_rounds_node_says_its_body_and_its_label() {
    let e = solved(&format!("{BOX}vert := box.ab round(r: 5mm) box.bc\n\
                             box.near round(r: 1mm) box.cd\n"));
    let body = crate::common::ent(&e, "box").i();
    let recipe = gcs_core::solid::cad::recipe(&e.sketch, body).unwrap();
    let said: Vec<String> = recipe.nodes.iter()
        .filter(|n| n.said.is_some()).map(|n| n.called()).collect();
    assert_eq!(said, ["`box`'s round or chamfer `vert.taken`", "`box`'s round or chamfer `taken`"]);
}

/// A bar 40 × 20 and a rib across its far side (30 to 60, 17 to 23, its far side `e2`), each 10
/// tall on the top plane, `second` unioned into a solid of `first`, and the rib's far top edge
/// rounded 5 over the solid they leave.
fn bar_and_rib(first: &str, second: &str) -> String {
    format!("unit mm\nuse std\nuse std.solids\nuse std.solids (union)\nuse std.rounds (round)\n\
             in std.top {{\n  long := step(u: 40mm, v: 0mm) (ab := line) -> \
             step(v: 20mm) (bc := line) -> step(u: 0mm) (cd := line) -> \
             step(v: 0mm) (da := line) -> close\n  t0 := point\n  fix((30, 17)) t0\n  \
             t1 := point\n  fix((60, 17)) t1\n  t2 := point\n  fix((60, 23)) t2\n  \
             t3 := point\n  fix((30, 23)) t3\n  \
             (e0 := line(t0, t1)) -> (e1 := line(t1, t2)) -> (e2 := line(t2, t3)) -> \
             (e3 := line(t3, t0)) -> close\n  tab := face(e0, e1, e2, e3)\n}}\n\
             bar := solid(std.solids.Prism(long, from: 0mm, to: 10mm))\n\
             rib := solid(std.solids.Prism(tab, from: 0mm, to: 10mm))\n\
             plate := solid({first})\n{second} union plate\n\
             part := solid(plate)\npart.plate.rib.near round(r: 5mm) part.plate.rib.e2\n")
}

/// **A section stands against a surface, not the faces it is named by** (#952): a plate's top in
/// the pieces the union of its operands leaves it, each named by its own, is one plane to a round,
/// so a ball whose touch lands past the edge between two pieces stands against the same plane
/// there.  The rib unioned into the bar: of the top, the bar's piece holds where both are, and the
/// rib's is three wide beside its far side over the bar, so the round along that side touches the
/// top past the edge between them over a third of its length — refused as a seam into the next
/// face of a chain, one not along the edge, where nothing goes on but the plane itself.  The plate
/// less the run's square section over the rib's 30, either way round the union.
#[test]
fn a_round_over_a_top_in_pieces_stands_against_one_plane() {
    let want = (40.0 * 20.0 + 10.0 * 3.0 + 20.0 * 6.0) * 10.0 - 30.0 * square_run(5.0);
    for (first, second) in [("bar", "rib"), ("rib", "bar")] {
        let e = solved(&bar_and_rib(first, second));
        assert!(e.diags.is_empty(), "{first} first: {:?}", e.diags);
        close(props::volume(&exported(&e, "part")), want);
    }
}

/// A disc of 50 and a lobe of 20 whose centre stands 40 out (`core` and `ear`), each 10 tall on
/// the top plane, `second` unioned into a solid of `first`; their upright edges rounded 10 where
/// they meet, and the lobe's top rim rounded 5 over the solid they leave.
fn disc_and_lobe(first: &str, second: &str) -> String {
    format!("unit mm\nuse std\nuse std.solids\nuse std.solids (union)\nuse std.rounds (round)\n\
             in std.top {{\n  o := point\n  fix((0, 0)) o\n  \
             big := circle(center: o) hint(r: 50)\n  radius(50mm) big\n  h := point\n  \
             fix((40, 0)) h\n  lobe := circle(center: h) hint(r: 20)\n  \
             radius(20mm) lobe\n}}\n\
             core := solid(std.solids.Prism(face(big), from: 0mm, to: 10mm))\n\
             ear := solid(std.solids.Prism(face(lobe), from: 0mm, to: 10mm))\n\
             plate := solid({first})\n{second} union plate\n\
             repeat f in plate {{ repeat g in plate {{ when f parallel std.z and g parallel std.z \
             and f != g {{ f round(r: 10mm) g }} }} }}\n\
             part := solid(plate)\npart.plate.ear.near round(r: 5mm) part.plate.ear\n")
}

/// **A rim over a plate of pieces** (#952, a lobed flange's): the lobe unioned into the disc
/// leaves its piece of the top a crescent, narrower than the ball's reach where the rounds of 10
/// run into the rim, so the rim's ring touches the top past the arc where the disc's piece begins
/// — refused as a seam running only part of the ring, though both pieces are one plane.  The same
/// solid as the disc unioned into the lobe, whose piece of the top is its whole disc, reached
/// nowhere past: one volume, and faces of the same kinds.
#[test]
fn a_rim_over_a_plate_of_pieces_is_the_rim_over_one_top() {
    let built = |first: &str, second: &str| {
        let e = solved(&disc_and_lobe(first, second));
        assert!(e.diags.is_empty(), "{first} first: {:?}", e.diags);
        exported(&e, "part")
    };
    let (crescent, whole) = (built("core", "ear"), built("ear", "core"));
    close(props::volume(&crescent), props::volume(&whole));
    assert_eq!(kinds(&crescent), kinds(&whole));
}

/// The stepped plate's outline (#961 item 2), counter-clockwise on the top plane: its thirteen
/// vertices, and each side's arc — its centre and whether it turns clockwise — or none for a line.
/// The side `k3` leaves the lobe arc `k2` (radius 12 about (-12, 0)) at `v3`, `turn` round from the
/// lobe's rightmost point, so the line turns `turn` outward of the circle there: a reflex corner a
/// hair off tangent, the line crossing the circle again `2 · 12 sin turn` back along it.
type Outline = ([[f64; 2]; 13], [Option<([f64; 2], bool)>; 13]);

fn stepped(turn: f64) -> Outline {
    let v = [[-60.0, -4.25], [-26.26315182559591221, -4.25], [-22.06810717100887942, -6.52941176470588225],
             [12.0 * turn.cos() - 12.0, 12.0 * turn.sin()], [0.0, 13.75], [8.0, 21.75], [30.0, 21.75],
             [40.0, 31.75], [40.0, 40.0], [30.0, 50.0], [-60.0, 50.0], [-70.0, 40.0], [-70.0, 5.75]];
    let arc = |c: [f64; 2], cw: bool| Some((c, cw));
    let sides = [None, arc([-26.26315182559591221, -9.25], true), arc([-12.0, 0.0], false), None,
                 arc([8.0, 13.75], true), None, arc([30.0, 31.75], false), None, arc([30.0, 40.0], false),
                 None, arc([-60.0, 40.0], false), None, arc([-60.0, 5.75], false)];
    (v, sides)
}

/// The stepped plate drawn, extruded 0 to 10, its top rim rounded `r`; mirrored across the y axis
/// where asked (its arcs then turning the other way).
fn stepped_plate(turn: f64, r: f64, mirrored: bool) -> String {
    let (v, sides) = stepped(turn);
    let x = |x: f64| if mirrored { 0.0 - x } else { x };
    let mut s = String::from("unit mm\nuse std\nuse std.solids\nuse std.rounds (round)\nin std.top {\n");
    for (i, p) in v.iter().enumerate() {
        s += &format!("  v{i} := point\n  fix(({:?}, {:?})) v{i}\n", x(p[0]), p[1]);
    }
    for (k, side) in sides.iter().enumerate() {
        let (a, b) = (k, (k + 1) % 13);
        s += &match side {
            None => format!("  k{k} := line(v{a}, v{b})\n"),
            Some((c, cw)) => format!("  c{k} := point hint(({:?}, {:?}))\n  fix(y == {:?}mm) c{k}\n  \
                                      k{k} := arc(center: c{k}, start: v{a}, end: v{b}{})\n",
                                     x(c[0]), c[1], c[1], if *cw != mirrored { ", sense: cw" } else { "" }),
        };
    }
    let all: Vec<String> = (0..13).map(|k| format!("k{k}")).collect();
    s + &format!("  outline := face({})\n}}\nplate := solid(std.solids.Prism(outline, from: 0mm, to: 10mm))\n\
                  plate.near round(r: {r}mm) plate\n", all.join(", "))
}

/// `∫ f` over `[a, b]` by Gauss–Legendre's 16 nodes on each of 48 panels halving toward `a`: at a
/// reflex corner `turn` off tangent the crossing a section's boundary turns at is analytic in the
/// round's angle but for a branch point some `turn` off `a`, which one rule over the whole span
/// converges to only slowly.
fn legendre(a: f64, b: f64, f: impl Fn(f64) -> f64) -> f64 {
    let nodes = crate::common::rounds::gauss_legendre(16);
    (0..48).map(|j| {
        let (lo, hi) = (a + (b - a) * 0.5f64.powi(j + 1), a + (b - a) * 0.5f64.powi(j));
        let (m, h) = (0.5 * (lo + hi), 0.5 * (hi - lo));
        nodes.iter().map(|&(x, w)| w * h * f(m + h * x)).sum::<f64>()
    }).sum()
}

/// Where the line through `p` along the unit `u` leaves the circle of radius `big` about `c`, `p`
/// within it: how far along.
fn leaving(p: [f64; 2], u: [f64; 2], c: [f64; 2], big: f64) -> f64 {
    let f = [p[0] - c[0], p[1] - c[1]];
    let b = f[0] * u[0] + f[1] * u[1];
    -b + (b * b - (f[0] * f[0] + f[1] * f[1] - big * big)).sqrt()
}

/// What the stepped plate keeps with its top rim rounded `r`, in closed form: at the height where
/// the ball's arc stands `w` in from the side, the plate's section less every run's band `w` wide
/// (its area by Green's theorem over its lines and arcs, `½ ∮ x dy − y dx`), integrated up the
/// round's height in the angle about its axis, where the section is analytic, by Gauss–Legendre.
/// Every side but `k3` meets the next tangent, so its band ends square where the next one's
/// begins (a line's band to its offset, an arc's ring to its offset arc, `R − w` turning left and
/// `R + w` turning right). At the reflex corner `v3` each of `std.rounds`' two `Run`s is cut off
/// by the face it ends on: `k2`'s ring ends square at `v3`, and `k3`'s band is kept outside `k2`'s
/// cylinder, so the section's boundary runs from the ring's end out to `v3`, on round the lobe's
/// circle to where `k3`'s offset line leaves it, and up that line — what lies within the circle
/// run on past `v3` and within `w` of `k3` is taken by neither run, and stands.
fn stepped_kept(turn: f64, r: f64) -> f64 {
    let (v, sides) = stepped(turn);
    let seg = |a: [f64; 2], b: [f64; 2]| 0.5 * (a[0] * b[1] - a[1] * b[0]);
    let arc = |c: [f64; 2], rho: f64, t0: f64, sweep: f64| {
        let t1 = t0 + sweep;
        0.5 * (rho * rho * sweep + rho * c[0] * (t1.sin() - t0.sin()) - rho * c[1] * (t1.cos() - t0.cos()))
    };
    let angle = |c: [f64; 2], p: [f64; 2]| (p[1] - c[1]).atan2(p[0] - c[0]);
    let section = |w: f64| (0..13).map(|k| {
        let (a, b) = (v[k], v[(k + 1) % 13]);
        match sides[k] {
            None => {
                let l = (b[0] - a[0]).hypot(b[1] - a[1]);
                let (u, n) = ([(b[0] - a[0]) / l, (b[1] - a[1]) / l], [(a[1] - b[1]) / l, (b[0] - a[0]) / l]);
                let (a2, b2) = ([a[0] + w * n[0], a[1] + w * n[1]], [b[0] + w * n[0], b[1] + w * n[1]]);
                if k != 3 { return seg(a2, b2) }
                // from `v3` on round the lobe to where `k3`'s offset line leaves it
                let c = [-12.0, 0.0];
                let t = leaving(a2, u, c, 12.0);
                let x = [a2[0] + t * u[0], a2[1] + t * u[1]];
                arc(c, 12.0, angle(c, a), angle(c, x) - angle(c, a)) + seg(x, b2)
            }
            Some((c, cw)) => {
                let big = (a[0] - c[0]).hypot(a[1] - c[1]);
                let (rho, t0) = (if cw { big + w } else { big - w }, angle(c, a));
                let sweep = (angle(c, b) - t0).rem_euclid(2.0 * PI);
                let sweep = if cw { sweep - 2.0 * PI } else { sweep };
                let end = [c[0] + rho * (t0 + sweep).cos(), c[1] + rho * (t0 + sweep).sin()];
                // the lobe's ring ends square at `v3`: out along its radius to it
                arc(c, rho, t0, sweep) + if k == 2 { seg(end, b) } else { 0.0 }
            }
        }
    }).sum::<f64>();
    let whole = section(0.0);
    10.0 * whole - legendre(0.0, PI / 2.0, |t| (whole - section(r * (1.0 - t.cos()))) * r * t.cos())
}

/// **A rim rounded past a reflex corner a hair off tangent** (#961 item 2): the stepped plate's
/// top rim rounded 1 and 2, `k3` leaving the lobe `1e-3` off tangent, built and mirrored to the
/// closed form. `k3`'s run is cut off by the lobe's cylinder (`build::kept_by_solid`), the run's
/// round face meeting the cylinder at `1e-3` rad where it reaches the run's end at `v3` (and again
/// 24 µm back past it, off the face). The run's upright edge at `v3` lies in the cylinder, and was
/// read as an edge the round face and the cylinder touch along, though the face only reaches its
/// foot: the foot's seed was dropped as a touch along it, and the march, shallow there, ended
/// beside it 5 µm short of the foot, on no edge — refused, the arrangement's shells kept and
/// dropped on either side of the cylinder. It was built until the search for unseen meetings left
/// alone what lies off either face (`fe9ae66`): before, a near-contact on the cylinder past the
/// run's end, off its round face, was refused, which sent the pair to be read from its ridges, and
/// that carried the meeting through the foot. Turned `3e-3` or `1e-2`, or rounded 3 (which reaches
/// `v4`'s concave arc), the plate is refused before this Boolean, as it was before `fe9ae66`:
/// those are not held here.
#[test]
fn a_reflex_corner_off_tangent_rounds_its_rim() {
    for r in [1.0, 2.0] {
        let want = stepped_kept(1e-3, r);
        for mirrored in [false, true] {
            let e = solved(&stepped_plate(1e-3, r, mirrored));
            assert!(e.diags.is_empty(), "r {r}, mirrored {mirrored}: {:?}", e.diags);
            close(props::volume(&exported(&e, "plate")), want);
        }
    }
    // the closed form is the volume `694cb72` built the plate rounded 2 to
    assert!((stepped_kept(1e-3, 2.0) - 49556.781577).abs() < 1e-6);
}

/// **An edge of one operand lying in the other's face is a meeting of the faces it lies in**
/// (`boolean`'s stretches, #961 item 2): the stepped plate's run along `k3` — its round's section
/// swept from `v3` to `v4` — cut off by the lobe's cylinder it leaves `turn` off tangent, what
/// `a_reflex_corner_off_tangent_rounds_its_rim` asks of the kernel. The run's upright edge at `v3`
/// lies in the cylinder; its round face reaches that edge only at its foot, where the two cross at
/// `turn`, and their meeting is traced to that foot, not ended where the march comes within the
/// spread of a touch along the edge. Kept outside the cylinder: the run less what the lobe's disc
/// holds of it, each line of the run's section along `k3` inside the disc to where it leaves it
/// (`leaving`), integrated over the section — to within what the Boolean's tolerance moves its
/// boundary by, its area times the tolerance (the piece reads some 2e-9 of itself low, whatever
/// the turn). Turned `1e-3` to `1e-2`, rounded 1 to 3, mirrored. (Turned `3e-4`, its two crossings
/// 7 µm apart, the cylinder's pieces are refused as not closing, as the plate so turned and
/// rounded 1 was before #961: not held here.)
#[test]
fn a_run_cut_off_by_the_cylinder_it_leaves_off_tangent_is_met_to_its_corner() {
    use gcs_core::brep::build::{kept_by_solid, prism, Profile, ProfileEdge};
    use gcs_core::brep::geom::{tolerance, Frame, Surface};
    for (turn, r) in [1e-3, 3e-3, 1e-2].into_iter().flat_map(|t| [(t, 1.0), (t, 2.0), (t, 3.0)]) {
        let (v, _) = stepped(turn);
        let length = (v[4][0] - v[3][0]).hypot(v[4][1] - v[3][1]);
        let u = [(v[4][0] - v[3][0]) / length, (v[4][1] - v[3][1]) / length];
        // into the plate, left of `k3`
        let n = [-u[1], u[0]];
        let (c, big) = ([-12.0, 0.0], 12.0);
        // the section's line `s` in from the side stands `r (1 − sin φ)` high, `s = r (1 − cos φ)`
        let taken = legendre(0.0, PI / 2.0, |phi| {
            let s = r * (1.0 - phi.cos());
            r * (1.0 - phi.sin()) * leaving([v[3][0] + s * n[0], v[3][1] + s * n[1]], u, c, big) * r * phi.sin()
        });
        let want = square_run(r) * length - taken;
        for mirrored in [false, true] {
            let m = |p: [f64; 3]| if mirrored { [-p[0], p[1], p[2]] } else { p };
            let (u3, n3) = (m([u[0], u[1], 0.0]), m([n[0], n[1], 0.0]));
            let corner = m([v[3][0], v[3][1], 10.0]);
            let at = |s: f64, d: f64| [corner[0] + s * n3[0], corner[1] + s * n3[1], 10.0 - d];
            let frame = Frame::new(at(r, r), u3, [-n3[0], -n3[1], 0.0]);
            let angle = |p: [f64; 3]| {
                let d = [p[0] - frame.o[0], p[1] - frame.o[1], p[2] - frame.o[2]];
                let along = |a: [f64; 3]| d[0] * a[0] + d[1] * a[1] + d[2] * a[2];
                along(frame.y).atan2(along(frame.x))
            };
            let (wall, top) = (angle(at(0.0, r)), angle(at(r, 0.0)));
            let edges = vec![ProfileEdge::Line { a: corner, b: at(0.0, r) },
                             ProfileEdge::Arc { frame, r, span: Some([wall.min(top), wall.max(top)]) },
                             ProfileEdge::Line { a: at(r, 0.0), b: corner }];
            let section = Profile { origin: corner, normal: u3, loops: vec![edges], names: Vec::new() };
            let run = prism(&section, 0.0, length).unwrap();
            close(props::volume(&run), square_run(r) * length);
            let lobe = Surface::Cylinder(Frame::new(m([c[0], c[1], 0.0]), [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]), big);
            let kept = kept_by_solid(&run, &lobe, false, 0.0)
                .unwrap_or_else(|why| panic!("turn {turn}, r {r}, mirrored {mirrored}: {why}"));
            let tol = tolerance(kept.size());
            kept.check(tol).unwrap();
            let (got, area) = (props::volume(&kept), kept.faces.iter().map(|f| props::area(&kept, f)).sum::<f64>());
            assert!((got - want).abs() <= tol * area, "turn {turn}, r {r}, mirrored {mirrored}: {got} against {want}");
        }
    }
}

/// The plate 100 square and 20 tall with a post of 15 at `(x, 0)` 30 tall unioned into it, both
/// from the top plane's `far`, and a hole of 6.5 through the post; the hole's two rims treated by
/// `word` (`chamfer(d: 1.5mm)`), or left (`None`).  The plate's and the post's bottoms are flush, the
/// pieces of one plane (#994's first item).
fn post_on_plate(x: f64, word: Option<&str>) -> String {
    format!("unit mm\nuse std\nuse std.solids\nuse std.solids (cut, union)\nuse std.rounds (round, chamfer)\n\
             in std.top {{\n  a := point\n  b := point\n  c := point\n  d := point\n  \
             fix((-50, -50)) a\n  fix((50, -50)) b\n  fix((50, 50)) c\n  fix((-50, 50)) d\n  \
             plate_f := face(a, b, c, d, -> close)\n  o := point\n  fix(({x}, 0)) o\n  \
             bc := circle(center: o) hint(r: 15)\n  radius(15mm) bc\n  \
             hc := circle(center: o) hint(r: 6.5)\n  radius(6.5mm) hc\n}}\n\
             stock := solid(std.solids.Prism(plate_f, from: 0mm, to: 20mm))\n\
             post := solid(std.solids.Prism(face(bc), from: 0mm, to: 30mm))\n\
             shell := solid(stock)\npost union shell\n\
             cutter := solid(std.solids.Prism(face(hc), from: -1mm, to: 31mm))\n\
             part := solid(shell)\ncutter cut part\n{}",
            word.map_or_else(String::new, |w| format!("part.cutter {w} part.shell\n")))
}

/// **A flat over flush pieces of one plane** (#996's fifth item, #994's first): where the post
/// stands over the plate's edge, the hole's bottom rim runs over the plate's piece of the bottom
/// and the post's, and the flat of the run along one piece reaches past the edge between them onto
/// the other — the same plane, so a piece of the face's own surface, never a face it goes on into
/// (`piece::stands_within` reading each face's surface in its pieces, `BodyFaces::pieces`).  Each
/// spot of the post, the hole inside the plate, crossing its edge either way and outside it: the
/// two rims' chamfers and rounds take what Pappus's theorem says, their sections turned about the
/// hole's axis.
#[test]
fn a_hole_over_flush_pieces_of_a_plane_is_chamfered_and_rounded_as_one_face() {
    let (d, r) = (1.5, 1.5);
    let flat = 2.0 * 2.0 * PI * (6.5 + d / 3.0) * d * d / 2.0;
    let round = 2.0 * 2.0 * PI * (6.5 + inset(r)) * square_run(r);
    for x in [40.0, 45.0, 50.0, 55.0] {
        let plain = exported_volume(&post_on_plate(x, None), "part");
        close(exported_volume(&post_on_plate(x, Some("chamfer(d: 1.5mm)")), "part"), plain - flat);
        close(exported_volume(&post_on_plate(x, Some("round(r: 1.5mm)")), "part"), plain - round);
    }
}

/// A post of 15 standing 21 on the top plane, spelled as one prism or (`stacked`) as two, 0 to 20
/// and 20 to 21, unioned: its wall then two pieces of one cylinder.  Its top rim treated by `word`.
fn post_of(stacked: bool, word: &str) -> String {
    let posts = if stacked {
        "low := solid(std.solids.Prism(face(c), from: 0mm, to: 20mm))\n\
         top := solid(std.solids.Prism(face(c), from: 20mm, to: 21mm))\n\
         part := solid(top)\nlow union part\n"
    } else {
        "top := solid(std.solids.Prism(face(c), from: 0mm, to: 21mm))\npart := solid(top)\n"
    };
    format!("unit mm\nuse std\nuse std.solids\nuse std.solids (union)\nuse std.rounds (round, chamfer)\n\
             in std.top {{\n  o := point\n  fix((0, 0)) o\n  c := circle(center: o) hint(r: 15)\n  \
             radius(15mm) c\n}}\n{posts}part.top.near {word} part.top\n")
}

/// **A flat over flush pieces of one cylinder**: the post's top rim chamfered 1.5 when the post is
/// two prisms stacked, the top one 1 tall, so the flat's leg down the wall reaches past the circle
/// where the pieces meet onto the lower one's — its own surface.  The same solid as the post in
/// one, both the closed form's (Pappus); and the round the same.
#[test]
fn a_rim_over_flush_pieces_of_a_cylinder_is_the_rim_over_one_wall() {
    let solid = PI * 15.0 * 15.0 * 21.0;
    for (word, taken) in [("chamfer(d: 1.5mm)", 2.0 * PI * (15.0 - 0.5) * 1.125),
                          ("round(r: 1.5mm)", 2.0 * PI * (15.0 - inset(1.5)) * square_run(1.5))] {
        for stacked in [false, true] {
            close(exported_volume(&post_of(stacked, word), "part"), solid - taken);
        }
    }
}

/// A block 60 × 40 × 20 and a flange 10 thick standing across its end, 60 long, 10 past the block's
/// side `side` and its far side, their tops flush (`near`), unioned `first` into `second`; the
/// flange's top rim rounded 1, then — over a solid of that one (`nested`), or in it — the block's
/// side rounded 1 where it meets its top.
fn block_and_flange(first: &str, second: &str, nested: bool) -> String {
    let side = if nested {
        "whole := solid(part)\nwhole.part.block.side round(r: 1mm) whole.part.block.near\n"
    } else {
        "part.block.side round(r: 1mm) part.block.near\n"
    };
    format!("unit mm\nuse std\nuse std.solids\nuse std.solids (union)\nuse std.rounds (round)\n\
             in std.top {{\n  a := point\n  b := point\n  c := point\n  d := point\n  \
             fix((0, 0)) a\n  fix((60, 0)) b\n  fix((60, 40)) c\n  fix((0, 40)) d\n  \
             side := line(a, b)\n  e1 := line(b, c)\n  e2 := line(c, d)\n  e3 := line(d, a)\n  \
             bk := face(side, e1, e2, e3)\n  p := point\n  q := point\n  r := point\n  s := point\n  \
             fix((60, -10)) p\n  fix((70, -10)) q\n  fix((70, 50)) r\n  fix((60, 50)) s\n  \
             fl := face(p, q, r, s, -> close)\n}}\n\
             block := solid(std.solids.Prism(bk, from: 0mm, to: 20mm))\n\
             flange := solid(std.solids.Prism(fl, from: -30mm, to: 20mm))\n\
             part := solid({first})\n{second} union part\n\
             part.flange.near round(r: 1mm) part.flange\n{side}")
}

/// **A chain smooth across its seams that turns the other way along a face's outline** (#996's
/// fifth item): the block's top and the flange's are one plane in two pieces, and the flange's rim
/// round goes on from the flange's piece, so the block's top goes on smoothly into it — but past
/// the reflex corner where the block's side meets the flange's, the round's side carried on past
/// its patch cuts off the block's top, and the chain bulges no way (`BodyFaces::bulging`).  The
/// block's side rounded over a solid already rounded so is the run's section over its 60, the same
/// as both rounds in one solid, either way round the union.
#[test]
fn a_run_beside_a_chain_turning_back_along_its_outline_is_built() {
    for (first, second) in [("block", "flange"), ("flange", "block")] {
        let together = exported_volume(&block_and_flange(first, second, false), "part");
        let nested = exported_volume(&block_and_flange(first, second, true), "whole");
        close(nested, together);
        let rimmed = exported_volume(&block_and_flange(first, second, false)
            .replace("part.block.side round(r: 1mm) part.block.near\n", ""), "part");
        close(nested, rimmed - 60.0 * square_run(1.0));
    }
}

/// The necked plate of #996 §3: its outline's corners, the lug's centre and radius.
const NECK: [[f64; 2]; 8] = [[-40.0, -30.0], [40.0, -30.0], [40.0, 20.0], [11.0, 30.0], [9.9, 37.0],
                             [-3.5, 37.0], [-4.3, 33.0], [-40.0, 20.0]];
const LUG: ([f64; 2], f64) = ([3.2, 39.7], 7.5);

/// The plate whose top narrows into a neck, 13.2 thick, a lug's circle over the neck's end joined
/// to it and blended 8 where they meet (where `lug`), and its far rim rounded `r` (where `r > 0`),
/// that statement before the blend's where `rim_first`; mirrored across the y axis where asked.
fn necked_plate(r: f64, lug: bool, rim_first: bool, mirrored: bool) -> String {
    let x = |x: f64| if mirrored { 0.0 - x } else { x };
    let mut s = String::from("unit mm\nuse std\nuse std.solids\nuse std.solids (union)\nuse std.rounds (round)\n\
                              in std.top {\n");
    for (i, p) in NECK.iter().enumerate() {
        s += &format!("  p{i} := point\n  fix(({:?}, {:?})) p{i}\n", x(p[0]), p[1]);
    }
    let mut order: Vec<String> = (0..8).map(|i| format!("p{i}")).collect();
    if mirrored { order.reverse() }
    s += &format!("  plate_f := face({}, -> close)\n  lc := point\n  fix(({:?}, {:?})) lc\n  \
                   lug_c := circle(center: lc) hint(r: {:?})\n  radius({:?}mm) lug_c\n}}\n\
                   plate := solid(std.solids.Prism(plate_f, from: 0mm, to: 13.2mm))\n",
                  order.join(", "), x(LUG.0[0]), LUG.0[1], LUG.1, LUG.1);
    s += "lug := solid(std.solids.Prism(face(lug_c), from: 0mm, to: 13.2mm))\npart := solid(plate)\n";
    let rim = if r > 0.0 { format!("rim := part.plate.far round(r: {r}mm) part.plate\n") } else { String::new() };
    if !lug { return s + &rim }
    s += "lug union part\n";
    let blend = "blend := part.lug.lug_c round(r: 8mm) part.plate\n";
    if rim_first { s + &rim + blend } else { s + blend + &rim }
}

/// What the necked plate's rim, rounded `r`, takes inside the lug's circle: each line of the
/// section along `p3p4`, `p4p5` and `p5p6` — `s = r (1 − cos φ)` in from the side and
/// `r (1 − sin φ)` high — from the bisector of the corner it leaves to that of the corner it comes
/// to (`p3`'s, far from the lug, no matter), within the circle, integrated over the section.
fn necked_rim_in_the_lug(r: f64) -> f64 {
    let v = NECK;
    let cot_half = |i: usize| {
        let (a, b, q) = (v[i - 1], v[i], v[i + 1]);
        let (u, w) = ([b[0] - a[0], b[1] - a[1]], [q[0] - b[0], q[1] - b[1]]);
        let turn = (u[0] * w[1] - u[1] * w[0]).atan2(u[0] * w[0] + u[1] * w[1]);
        1.0 / ((PI - turn) / 2.0).tan()
    };
    let (c, big) = LUG;
    let inside = |s: f64| (3..6).map(|k| {
        let (a, b) = (v[k], v[k + 1]);
        let l = (b[0] - a[0]).hypot(b[1] - a[1]);
        let u = [(b[0] - a[0]) / l, (b[1] - a[1]) / l];
        let p = [a[0] - s * u[1], a[1] + s * u[0]];
        let (from, to) = (if k == 3 { f64::NEG_INFINITY } else { s * cot_half(k) }, l - s * cot_half(k + 1));
        let f = [p[0] - c[0], p[1] - c[1]];
        let h = f[0] * u[0] + f[1] * u[1];
        let d = h * h - (f[0] * f[0] + f[1] * f[1] - big * big);
        if d <= 0.0 { return 0.0 }
        (to.min(-h + d.sqrt()) - from.max(-h - d.sqrt())).max(0.0)
    }).sum::<f64>();
    let nodes = crate::common::rounds::gauss_legendre(16);
    let panels = 200;
    let h = PI / 2.0 / panels as f64;
    (0..panels).map(|j| {
        let m = (j as f64 + 0.5) * h;
        nodes.iter().map(|&(x, w)| {
            let phi = m + 0.5 * h * x;
            0.5 * w * h * r * (1.0 - phi.sin()) * inside(r * (1.0 - phi.cos())) * r * phi.sin()
        }).sum::<f64>()
    }).sum()
}

/// **A rim's round crossing the foot of a blend it does not round** (#996 §3): the necked plate's
/// far rim rounded `r` beside the lug blended 8 into its neck. The round's run along the neck's
/// short side `p5p6` and the blend's cylinder touch at a point where the blend's foot crosses the
/// round's tangent line on the side — a definite touch, `2 SHALLOW · 8` across, more than a
/// twentieth of the run's face — once refused as "a cylinder and a cylinder meet … where no edge crosses, in a curve
/// that could not be traced" (`Touches::alone`: a definite touch is alone by its own orders,
/// `ssi::alone_over`, not by the faces' size). The rim's runs end on the lug's cylinder, kept
/// outside it, as on the plate alone before the lug: so the part is the plate, lug and blend less
/// what the rim takes of the plate alone, but for what of that the lug's circle holds
/// (`necked_rim_in_the_lug`), to 1e-9 — rounded 1, 2 and 3, the rim written after the blend, before
/// it, and after it mirrored. (Blended 16 or more, the blend's foot past the side's end `p6`, the part is
/// refused at its export as touching itself at `p6`: not held here.)
#[test]
fn a_rims_round_crossing_the_foot_of_a_lugs_blend_is_built() {
    let volume = |src: String| exported_volume(&src, "part");
    let blended = volume(necked_plate(0.0, true, false, false));
    let plate = volume(necked_plate(0.0, false, false, false));
    for r in [1.0, 2.0, 3.0] {
        let taken = plate - volume(necked_plate(r, false, false, false)) - necked_rim_in_the_lug(r);
        for (rim_first, mirrored) in [(false, false), (true, false), (false, true)] {
            let got = volume(necked_plate(r, true, rim_first, mirrored));
            assert!((got - (blended - taken)).abs() <= 1e-9 * blended,
                    "r {r}, rim first {rim_first}, mirrored {mirrored}: {got} against {}", blended - taken);
        }
    }
}
