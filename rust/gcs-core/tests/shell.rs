//! **A body shelled** (`std.rounds`' `shell`): `A shell(t: …) B` keeps of `B`'s body only what lies
//! within `t` of one of its faces not among `A`'s — its cavity the points within the body's faces,
//! each kept face's wall `{ p | p distance(<= t) w }` cut from it — and `shell(t: …) X` what lies
//! within `t` of everything outside `X`.  The cavity stands on a region reading the body's faces,
//! so it is one of the stratum's readers and acts on the body after its other terms
//! (`faces::readers_of`).  Read point by point, each case held at a lattice of points to its
//! closed form, the field's side at each the closed form's; built exactly (`solid::shelled`) where
//! the body is a prism less what is cut from it, and by its walls where a boss is put on it, each
//! held to its closed form's volume to 1e-9.
use gcs_core::brep::props::volume;
use gcs_core::program::Elaborated;
use gcs_core::solid::MaterialField;
use std::f64::consts::PI;

use crate::common::{close, ent, text::solved};

/// A block 40 × 24 × 12 drawn on the front plane (x, z), standing 12 deep along y; an L of two
/// steps whose inner edge, along y through (10, 110), is concave.
const BLOCKS: &str = "unit mm\nuse std\nuse std.rounds (shell)\nin std.front {\n\
    b0 := point\nb1 := point\nb2 := point\nb3 := point\n\
    fix((0, 0)) b0\nfix((40, 0)) b1\nfix((40, 24)) b2\nfix((0, 24)) b3\n\
    (south := line(b0, b1)) -> (east := line(b1, b2)) -> (north := line(b2, b3)) -> \
    (west := line(b3, b0)) -> close\n\
    outline := face(south, east, north, west)\n\
    l0 := point\nl1 := point\nl2 := point\nl3 := point\nl4 := point\nl5 := point\n\
    fix((0, 100)) l0\nfix((20, 100)) l1\nfix((20, 110)) l2\nfix((10, 110)) l3\nfix((10, 120)) l4\n\
    fix((0, 120)) l5\n\
    (ls := line(l0, l1)) -> (le := line(l1, l2)) -> (lt := line(l2, l3)) -> (lr := line(l3, l4)) -> \
    (ln := line(l4, l5)) -> (lw := line(l5, l0)) -> close\n\
    ell := face(ls, le, lt, lr, ln, lw)\n}\n\
    block := solid(outline, depth: 12mm)\n\
    step := solid(ell, depth: 12mm)\n";

/// On the top plane a plinth: a slab 40 square and 10 thick, a boss 10 square standing 5 on it.
const PLINTH: &str = "unit mm\nuse std\nuse std.rounds (shell)\nin std.top {\n\
    q0 := point\nq1 := point\nq2 := point\nq3 := point\n\
    fix((0, 0)) q0\nfix((40, 0)) q1\nfix((40, 40)) q2\nfix((0, 40)) q3\n\
    (qs := line(q0, q1)) -> (qe := line(q1, q2)) -> (qn := line(q2, q3)) -> (qw := line(q3, q0)) -> close\n\
    slab := face(qs, qe, qn, qw)\n\
    m0 := point\nm1 := point\nm2 := point\nm3 := point\n\
    fix((15, 15)) m0\nfix((25, 15)) m1\nfix((25, 25)) m2\nfix((15, 25)) m3\n\
    (ms := line(m0, m1)) -> (me := line(m1, m2)) -> (mn := line(m2, m3)) -> (mw := line(m3, m0)) -> close\n\
    stud := face(ms, me, mn, mw)\n}\n\
    plinth := solid(slab, from: 0mm, to: 10mm)\n\
    boss := solid(stud, from: 9mm, to: 15mm)\nboss union plinth\n";

/// On the top plane a tray: a slab 40 square and 10 thick, a pocket 20 × 16 and 4 deep cut into
/// its top.
const STANDING: &str = "unit mm\nuse std\nuse std.rounds (shell)\nin std.top {\n\
    q0 := point\nq1 := point\nq2 := point\nq3 := point\n\
    fix((0, 0)) q0\nfix((40, 0)) q1\nfix((40, 40)) q2\nfix((0, 40)) q3\n\
    (qs := line(q0, q1)) -> (qe := line(q1, q2)) -> (qn := line(q2, q3)) -> (qw := line(q3, q0)) -> close\n\
    slab := face(qs, qe, qn, qw)\n\
    k0 := point\nk1 := point\nk2 := point\nk3 := point\n\
    fix((10, 12)) k0\nfix((30, 12)) k1\nfix((30, 28)) k2\nfix((10, 28)) k3\n\
    (ks := line(k0, k1)) -> (ke := line(k1, k2)) -> (kn := line(k2, k3)) -> (kw := line(k3, k0)) -> close\n\
    pocket := face(ks, ke, kn, kw)\n}\n\
    tray := solid(slab, from: 0mm, to: 10mm)\n\
    cavity := solid(pocket, from: 6mm, to: 11mm)\ncavity cut tray\n";

/// On the top plane a plate 40 square and 5 thick with a bore of radius 6 through it.
const BORED: &str = "unit mm\nuse std\nuse std.rounds (shell)\nin std.top {\n\
    p0 := point\np1 := point\np2 := point\np3 := point\n\
    fix((80, -20)) p0\nfix((120, -20)) p1\nfix((120, 20)) p2\nfix((80, 20)) p3\n\
    (ps := line(p0, p1)) -> (pe := line(p1, p2)) -> (pn := line(p2, p3)) -> (pw := line(p3, p0)) -> close\n\
    plate := face(ps, pe, pn, pw)\n\
    bc := point\nfix((100, 0)) bc\nbore := radius(6mm) circle(center: bc)\nhole := face(bore)\n}\n\
    slab := solid(plate, from: 0mm, to: 5mm)\n\
    drill := solid(hole, from: -1mm, to: 6mm)\ndrill cut slab\n";

/// On the front plane a hopper's section, a pentagon whose west wall leans in at 45° under its top,
/// standing 12 deep.
const HOPPER: &str = "unit mm\nuse std\nuse std.rounds (shell)\nin std.front {\n\
    h0 := point\nh1 := point\nh2 := point\nh3 := point\nh4 := point\n\
    fix((0, 0)) h0\nfix((30, 0)) h1\nfix((30, 20)) h2\nfix((10, 20)) h3\nfix((0, 10)) h4\n\
    (bottom := line(h0, h1)) -> (east := line(h1, h2)) -> (top := line(h2, h3)) -> (lean := line(h3, h4)) -> \
    (west := line(h4, h0)) -> close\n\
    section := face(bottom, east, top, lean, west)\n}\n\
    hopper := solid(section, depth: 12mm)\n";

/// Solid `name`'s field.
fn field(e: &Elaborated, name: &str) -> MaterialField {
    MaterialField::read(&e.sketch, ent(e, name).i(), 1e-10).unwrap_or_else(|m| panic!("`{name}`: {m:?}"))
}

/// Within an axis-aligned box.
fn within(p: [f64; 3], lo: [f64; 3], hi: [f64; 3]) -> bool {
    (0..3).all(|k| lo[k] < p[k] && p[k] < hi[k])
}

/// The field of `name` against `material` at a lattice of points over the box `lo`–`hi`, a step
/// of about `h` apart, its offsets irrational so no point lies on a face: the same side at each.
fn holds(e: &Elaborated, name: &str, lo: [f64; 3], hi: [f64; 3], h: f64, material: impl Fn([f64; 3]) -> bool) {
    let f = field(e, name);
    let n: Vec<usize> = (0..3).map(|k| ((hi[k] - lo[k]) / h).ceil() as usize).collect();
    let offset = [0.31830988618, 0.70710678118, 0.57735026919];
    let mut count = 0;
    for i in 0..n[0] {
        for j in 0..n[1] {
            for k in 0..n[2] {
                let ijk = [i, j, k];
                let p: [f64; 3] = std::array::from_fn(|a| {
                    lo[a] + (ijk[a] as f64 + offset[a]) * (hi[a] - lo[a]) / n[a] as f64
                });
                let side = f.side(p);
                assert_eq!(side < 0.0, material(p), "`{name}` at {p:?}: {side}");
                count += 1;
            }
        }
    }
    assert!(count > 1000, "{count}");
}

/// Shelled closed, the block keeps what lies within 2 of its outside: a box 36 × 8 × 20 sealed
/// within it, every wall 2 thick.
#[test]
fn a_block_shelled_closed_is_sealed() {
    let e = solved(&format!("{BLOCKS}shell(t: 2mm) block\n"));
    holds(&e, "block", [-1.0, -1.0, -1.0], [41.0, 13.0, 25.0], 0.7, |p| {
        within(p, [0.0, 0.0, 0.0], [40.0, 12.0, 24.0]) && !within(p, [2.0, 2.0, 2.0], [38.0, 10.0, 22.0])
    });
}

/// Open through its north face (z = 24) the cavity runs out through it: the walls stand 2 thick
/// to its rim and nothing closes it.
#[test]
fn a_block_shelled_open_through_a_face() {
    let e = solved(&format!("{BLOCKS}block.north shell(t: 2mm) block\n"));
    holds(&e, "block", [-1.0, -1.0, -1.0], [41.0, 13.0, 26.0], 0.7, |p| {
        within(p, [0.0, 0.0, 0.0], [40.0, 12.0, 24.0]) && !within(p, [2.0, 2.0, 2.0], [38.0, 10.0, 25.0])
    });
}

/// Within a concave edge the walls' inner faces are rounded by the thickness, a ball rolled within
/// both: the cavity holds whatever is more than 2 from the edge there, where the faces moved in
/// would have met square.  The L's concave edge runs along y through (10, 110).
#[test]
fn within_a_concave_edge_the_shell_is_rounded() {
    let e = solved(&format!("{BLOCKS}shell(t: 2mm) step\n"));
    let material = |p: [f64; 3]| -> bool {
        let (x, z) = (p[0], p[2]);
        let solid = within(p, [0.0, 0.0, 100.0], [20.0, 12.0, 110.0]) || within(p, [0.0, 0.0, 110.0], [10.0, 12.0, 120.0]);
        // each step's faces moved in by 2, and between them, past both, what is more than 2 from
        // the edge
        let hollow = within(p, [2.0, 2.0, 102.0], [18.0, 10.0, 108.0]) || within(p, [2.0, 2.0, 108.0], [8.0, 10.0, 118.0])
            || within(p, [8.0, 2.0, 108.0], [10.0, 10.0, 110.0]) && (x - 10.0).hypot(z - 110.0) > 2.0;
        solid && !hollow
    };
    holds(&e, "step", [-1.0, -1.0, 99.0], [21.0, 13.0, 121.0], 0.45, material);
    // where the faces moved in would have met, it is hollow
    let f = field(&e, "step");
    assert!(f.side([8.5, 6.0, 108.5]) > 0.0, "{}", f.side([8.5, 6.0, 108.5]));
    assert!(f.side([8.8, 6.0, 108.8]) < 0.0, "{}", f.side([8.8, 6.0, 108.8]));
}

/// A boss the body carries is shelled with it, the cavity running up into it: the shell acts on
/// the body its other terms make.
#[test]
fn a_boss_is_shelled_with_its_body() {
    let e = solved(&format!("{PLINTH}shell(t: 1mm) plinth\n"));
    holds(&e, "plinth", [-1.0, -1.0, -1.0], [41.0, 41.0, 16.0], 0.9, |p| {
        let solid = within(p, [0.0, 0.0, 0.0], [40.0, 40.0, 10.0]) || within(p, [15.0, 15.0, 9.0], [25.0, 25.0, 15.0]);
        // the slab's cavity, and the boss's rising from it; within 1 of the boss's foot edges the
        // plate's top bounds it, round them
        let foot = |p: [f64; 3]| {
            let (x, y) = (p[0], p[1]);
            let out = (15.0 - x).max(x - 25.0).max(15.0 - y).max(y - 25.0);
            let ring = if out <= 0.0 { f64::INFINITY } else {
                let dx = (15.0 - x).max(x - 25.0).max(0.0);
                let dy = (15.0 - y).max(y - 25.0).max(0.0);
                dx.hypot(dy).hypot(p[2] - 10.0)
            };
            p[2] > 9.0 && ring < 1.0
        };
        let hollow = within(p, [1.0, 1.0, 1.0], [39.0, 39.0, 9.0]) || within(p, [16.0, 16.0, 8.0], [24.0, 24.0, 14.0]);
        solid && !(hollow && !foot(p))
    });
}

/// `name` built exactly, its volume.
pub(crate) fn exact_volume(e: &Elaborated, name: &str) -> f64 {
    let x = e.sketch.exact_solid(ent(e, name).i()).unwrap_or_else(|m| panic!("`{name}`: {m}"));
    volume(&x.brep)
}

/// A prism shelled is built exactly: its cavity a prism, the section's edges moved in by the
/// thickness and its caps, an open face's moved out.  The block sealed loses a box 36 × 8 × 20,
/// open through its north face one 36 × 8 × 22; the L's cavity has an arc of radius 2 about its
/// reflex corner, so it takes `8 (160 − π)` of the L's 3600.
#[test]
fn a_prism_shelled_is_built_exactly() {
    let e = solved(&format!("{BLOCKS}shell(t: 2mm) block\nshell(t: 2mm) step\n"));
    close(exact_volume(&e, "block"), 11520.0 - 36.0 * 8.0 * 20.0);
    close(exact_volume(&e, "step"), 3600.0 - 8.0 * (160.0 - PI));
    let e = solved(&format!("{BLOCKS}block.north shell(t: 2mm) block\n"));
    close(exact_volume(&e, "block"), 11520.0 - 36.0 * 8.0 * 22.0);
}

/// What is cut from the prism takes from the cavity what lies within the thickness of it: the
/// cavity of `A` less `P` is `A`'s less `P` grown.  A tray, a slab 40 square and 10 thick with a
/// pocket 20 × 16 and 4 deep in its top, shelled by 1: the box 38 × 38 × 8 less the pocket grown by
/// 1 below its top, the offset rectangle's prism 3 deep and its rounded floor (Steiner's half
/// layer); open through its top, 9 deep and the prism 4.  A plate 40 square and 5 thick with a bore
/// of radius 6: the box 38 × 38 × 3 less a cylinder of radius 7.
#[test]
fn what_is_cut_from_a_prism_is_shelled_with_it() {
    let floor = 320.0 + PI / 4.0 * 72.0 + 4.0 * PI / 6.0;
    let tray = 16000.0 - 20.0 * 16.0 * 4.0;
    let e = solved(&format!("{STANDING}shell(t: 1mm) tray\n"));
    close(exact_volume(&e, "tray"), tray - (38.0 * 38.0 * 8.0 - (392.0 + PI) * 3.0 - floor));
    let e = solved(&format!("{STANDING}tray.near shell(t: 1mm) tray\n"));
    close(exact_volume(&e, "tray"), tray - (38.0 * 38.0 * 9.0 - (392.0 + PI) * 4.0 - floor));
    let e = solved(&format!("{BORED}shell(t: 1mm) slab\n"));
    close(exact_volume(&e, "slab"), 8000.0 - 180.0 * PI - (38.0 * 38.0 * 3.0 - 147.0 * PI));
}

/// A boss put on the body is shelled with it, built exactly: the walls are what lies within the
/// thickness of what is outside the body, built as a body grown is about its faces turned inside
/// out, so within the boss's foot the cavity is rounded by the thickness about each foot edge.  The
/// plinth shelled by 1: the slab's box 38 × 38 × 8, then over the boss's square up from the slab's
/// inner top, the section shrunk by the foot's quarter circle (`∫ (10 − 2√(1 − s²))² ds = 100 − 10π
/// + 8/3`), then the boss's inner square 8 × 8 up to 1 below its top — or, its top open, to it; a
/// pit cut beside the boss takes its own growth from the cavity, as a prism's cut does.
#[test]
fn a_body_with_a_boss_is_shelled_exactly() {
    let (plinth, foot) = (16500.0, 100.0 - 10.0 * PI + 8.0 / 3.0);
    let e = solved(&format!("{PLINTH}shell(t: 1mm) plinth\n"));
    close(exact_volume(&e, "plinth"), plinth - (38.0 * 38.0 * 8.0 + foot + 64.0 * 4.0));
    let e = solved(&format!("{PLINTH}plinth.boss.near shell(t: 1mm) plinth\n"));
    close(exact_volume(&e, "plinth"), plinth - (38.0 * 38.0 * 8.0 + foot + 64.0 * 5.0));
    // a pit 8 square and 4 deep sunk beside the boss: the cavity less the pit grown by 1 below the
    // slab's inner top, its offset square's prism 3 deep and its rounded floor
    let pit = "in std.top {\nd0 := point\nd1 := point\nd2 := point\nd3 := point\n\
               fix((4, 4)) d0\nfix((12, 4)) d1\nfix((12, 12)) d2\nfix((4, 12)) d3\n\
               (ds := line(d0, d1)) -> (de := line(d1, d2)) -> (dn := line(d2, d3)) -> (dw := line(d3, d0)) -> close\n\
               dimple := face(ds, de, dn, dw)\n}\npit := solid(dimple, from: 6mm, to: 11mm)\npit cut plinth\n";
    let e = solved(&format!("{PLINTH}{pit}shell(t: 1mm) plinth\n"));
    let grown = 3.0 * (96.0 + PI) + 64.0 + 8.0 * PI + 4.0 * PI / 6.0;
    close(exact_volume(&e, "plinth"), plinth - 256.0 - (38.0 * 38.0 * 8.0 + foot + 64.0 * 4.0 - grown));
}

/// What is not built is read by the field, and an exact export says why by name: a wall standing
/// on a face left open (the plinth's boss on its open top: its foot's quarter round hangs below),
/// a cut's faces left open, a section whose walls meet, a prism no longer than twice the thickness.
#[test]
fn what_is_not_built_is_read_by_the_field_and_said() {
    let refused = |doc: &str, src: &str, name: &str, says: &str| {
        let e = solved(&format!("{doc}{src}\n"));
        let Err(why) = e.sketch.exact_solid(ent(&e, name).i()) else { panic!("`{src}` built") };
        assert!(why.contains(says), "`{src}`: {why}");
        field(&e, name);
    };
    refused(PLINTH, "plinth.near shell(t: 1mm) plinth", "plinth", "a wall standing on a face left open");
    refused(STANDING, "tray.cavity shell(t: 1mm) tray", "tray", "none of the stock's");
    refused(BLOCKS, "shell(t: 6mm) step", "step", "narrower than twice the thickness");
    refused(BLOCKS, "shell(t: 7mm) block", "block", "no longer than twice the thickness");
}

/// The cavity's faces are named under it, as a region's are: each wall's inner face for the face
/// it is moved in from, the arc about a reflex corner `round`.
#[test]
fn a_shells_faces_are_named_for_the_faces_they_line() {
    let e = solved(&format!("{BLOCKS}block.north shell(t: 2mm) block\nshell(t: 2mm) step\n"));
    let names = |n: &str| -> Vec<String> {
        e.sketch.exact_solid(ent(&e, n).i()).unwrap().brep.faces.iter().map(|f| f.name.clone()).collect()
    };
    let block = names("block");
    for face in ["block.north", "block.south", ".cavity.south", ".cavity.far"] {
        assert!(block.iter().any(|n| n.ends_with(face)), "{face} among {block:?}");
    }
    assert!(!block.iter().any(|n| n.ends_with(".cavity.north")), "{block:?}");
    let step = names("step");
    assert!(step.iter().any(|n| n.ends_with(".cavity.round")), "{step:?}");
}

/// The distance from `p` to the rectangle square to axis `k` at `v`, its other two axes (in order)
/// between `lo` and `hi`.
fn rect(p: [f64; 3], k: usize, v: f64, lo: [f64; 2], hi: [f64; 2]) -> f64 {
    let (a, b) = ((k + 1) % 3, (k + 2) % 3);
    let (a, b) = (a.min(b), a.max(b));
    let da = p[a] - p[a].clamp(lo[0], hi[0]);
    let db = p[b] - p[b].clamp(lo[1], hi[1]);
    (p[k] - v).hypot(da).hypot(db)
}

/// The plinth's faces but its slab's top, each its distance from `p`: the slab's floor and sides,
/// the boss's sides from the slab's top up, and its top.
fn plinth_kept(p: [f64; 3]) -> f64 {
    [rect(p, 2, 0.0, [0.0, 0.0], [40.0, 40.0]),
     rect(p, 0, 0.0, [0.0, 0.0], [40.0, 10.0]), rect(p, 0, 40.0, [0.0, 0.0], [40.0, 10.0]),
     rect(p, 1, 0.0, [0.0, 0.0], [40.0, 10.0]), rect(p, 1, 40.0, [0.0, 0.0], [40.0, 10.0]),
     rect(p, 0, 15.0, [15.0, 10.0], [25.0, 15.0]), rect(p, 0, 25.0, [15.0, 10.0], [25.0, 15.0]),
     rect(p, 1, 15.0, [15.0, 10.0], [25.0, 15.0]), rect(p, 1, 25.0, [15.0, 10.0], [25.0, 15.0]),
     rect(p, 2, 15.0, [15.0, 15.0], [25.0, 25.0])].into_iter().fold(f64::INFINITY, f64::min)
}

/// Open through the slab's top, the boss's walls stand on its rim, an edge concave in the body:
/// within `t` of each wall is kept, so the wall runs on round the rim, a quarter round of radius
/// `t` into the hollow below it, as the definition says — what lies within 1 of a kept face.
#[test]
fn a_wall_runs_on_round_a_concave_rim() {
    let e = solved(&format!("{PLINTH}plinth.near shell(t: 1mm) plinth\n"));
    let body = |p: [f64; 3]| within(p, [0.0, 0.0, 0.0], [40.0, 40.0, 10.0]) || within(p, [15.0, 15.0, 9.0], [25.0, 25.0, 15.0]);
    holds(&e, "plinth", [-1.0, -1.0, -1.0], [41.0, 41.0, 16.0], 0.9, |p| body(p) && plinth_kept(p) < 1.0);
    // under the rim beside the boss, short of its walls' reach and within it
    let f = field(&e, "plinth");
    for (p, inside) in [([14.5, 20.0, 9.5], true), ([14.2, 20.0, 9.2], false), ([14.5, 20.0, 10.5], false)] {
        assert_eq!(f.side(p) < 0.0, inside, "{p:?}: {}", f.side(p));
    }
}

/// Something standing in the hollow is added to the body shelled, named: a post on a pan's floor
/// stands whole in the cavity the shell left, where added to the pan itself the shell would have
/// hollowed it too.
#[test]
fn what_stands_in_the_hollow_is_added_to_the_body_shelled() {
    let e = solved(&format!("{PLINTH}pan := solid(slab, from: 0mm, to: 10mm)\npan.near shell(t: 1mm) pan\n\
                             tray := solid(pan)\npost := solid(stud, from: 0.5mm, to: 6mm)\npost union tray\n"));
    holds(&e, "tray", [-1.0, -1.0, -1.0], [41.0, 41.0, 11.0], 0.9, |p| {
        within(p, [0.0, 0.0, 0.0], [40.0, 40.0, 10.0]) && !within(p, [1.0, 1.0, 1.0], [39.0, 39.0, 11.0])
            || within(p, [15.0, 15.0, 0.5], [25.0, 25.0, 6.0])
    });
}

/// Where a kept wall meets the open face at more than a right angle, leaning in over the opening,
/// a point past the wall's rim is nearer its corner than its line: the cavity runs on round the
/// corner on an arc of radius `t` to the open face, and the wall ends in a rounded lip.  A hopper's
/// section, a pentagon whose west wall leans in at 45° to its open top: the section's area less
/// the cavity's — the walls moved in, clipped by the top, and what lies past the leaning wall's
/// line moved in but outside the arc about its corner, `2 − π/2` below the top — over its depth,
/// the caps moved in by 2.
#[test]
fn a_wall_leaning_in_over_its_opening_keeps_a_rounded_lip() {
    let e = solved(&format!("{HOPPER}hopper.top shell(t: 2mm) hopper\n"));
    let area = |p: &[[f64; 2]]| (0..p.len()).map(|k| { let (a, b) = (p[k], p[(k + 1) % p.len()]); a[0] * b[1] - a[1] * b[0] })
        .sum::<f64>() / 2.0;
    let section = area(&[[0.0, 0.0], [30.0, 0.0], [30.0, 20.0], [10.0, 20.0], [0.0, 10.0]]);
    let s = 2f64.sqrt();
    let moved = area(&[[2.0, 2.0], [28.0, 2.0], [28.0, 20.0], [10.0 + 2.0 * s, 20.0], [2.0, 12.0 - 2.0 * s]]);
    let lip = 2.0 - PI / 2.0;
    close(exact_volume(&e, "hopper"), 12.0 * section - 8.0 * (moved + lip));
}
