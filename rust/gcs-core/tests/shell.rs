//! **A body shelled** (`std.rounds`' `shell`): `A shell(t: …) B` keeps of `B`'s body only what lies
//! within `t` of one of its faces not among `A`'s — its cavity the points within the body's faces,
//! each kept face's wall `{ p | p distance(<= t) w }` cut from it — and `shell(t: …) X` what lies
//! within `t` of everything outside `X`.  The cavity stands on a region reading the body's faces,
//! so it is one of the stratum's readers and acts on the body after its other terms
//! (`faces::readers_of`).  Read point by point; each case held at a lattice of points to its
//! closed form, the field's side at each the closed form's.
use gcs_core::program::Elaborated;
use gcs_core::solid::MaterialField;

use crate::common::{ent, text::solved};

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

/// Shelled, a body is read point by point: an exact export refuses it, saying why.
#[test]
fn a_shelled_body_is_not_built_exactly() {
    let e = solved(&format!("{BLOCKS}block.north shell(t: 2mm) block\n"));
    let Err(why) = e.sketch.exact_solid(ent(&e, "block").i()) else { panic!("built") };
    assert!(why.contains("read by the field") && why.contains("cavity"), "{why}");
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
