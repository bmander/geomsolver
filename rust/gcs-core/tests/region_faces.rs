//! **Faces as sets** (#183 F1) and **the material's angle at an edge** (F9): a region's body names a
//! face of a solid by its path (`block.near`), read off the solid's exact boundary: its side
//! (`inside`), its patch (`distance(p, block.near)`), its surface (`block.near.surface`), and the
//! angle the material makes where two faces meet.  Held to closed forms.
use gcs_core::program::Elaborated;
use gcs_core::solid::MaterialField;
use gcs_core::solve::{solve, SolveOpts};
use std::f64::consts::PI;

use crate::common::{ent, read};
use crate::region_general::field_volume;

fn solved(src: &str) -> Elaborated {
    let mut e = read(src);
    let r = solve(&mut e.sketch, SolveOpts::default());
    assert!(r.success, "{}\n{src}", r.message);
    e
}

/// A block 40 × 24 × 12 drawn on the front plane (x, z), standing 12 deep along y, and an L of
/// two such steps whose inner edge is concave.
const BLOCK: &str = "unit mm\nuse std\nin std.front {\n\
    b0 := point\nb1 := point\nb2 := point\nb3 := point\n\
    fix((0, 0)) b0\nfix((40, 0)) b1\nfix((40, 24)) b2\nfix((0, 24)) b3\n\
    (south := line(b0, b1)) -> (east := line(b1, b2)) -> (north := line(b2, b3)) -> \
    (west := line(b3, b0)) -> close\n\
    outline := face(south, east, north, west)\n\
    l0 := point\nl1 := point\nl2 := point\nl3 := point\nl4 := point\nl5 := point\n\
    fix((100, 0)) l0\nfix((120, 0)) l1\nfix((120, 10)) l2\nfix((110, 10)) l3\nfix((110, 20)) l4\n\
    fix((100, 20)) l5\n\
    (ls := line(l0, l1)) -> (le := line(l1, l2)) -> (lt := line(l2, l3)) -> (lr := line(l3, l4)) -> \
    (ln := line(l4, l5)) -> (lw := line(l5, l0)) -> close\n\
    ell := face(ls, le, lt, lr, ln, lw)\n}\n\
    block := solid(outline, depth: 12mm)\nstep := solid(ell, depth: 12mm)\n";

/// Where the block stands: its corners' box, from its own field's support.
fn block_box(e: &Elaborated) -> ([f64; 3], [f64; 3]) {
    let f = MaterialField::read(&e.sketch, ent(e, "block").i(), 1e-10).unwrap();
    let b = f.support_bounds().unwrap().expect("a block's box");
    let lo = std::array::from_fn(|k| b[k].bounds()[0]);
    let hi = std::array::from_fn(|k| b[k].bounds()[1]);
    (lo, hi)
}

/// The points within 1 of the block's near face: the rectangle grown by a ball,
/// `2rA + πr²P/2 + 4πr³/3`.
#[test]
fn the_points_near_a_face_are_its_patch_grown() {
    let e = solved(&format!("{BLOCK}pillow := {{ p | distance(p, block.near) <= 1mm }}\n\
                             s := solid(pillow)\n"));
    let (lo, hi) = block_box(&e);
    let s = ent(&e, "s").i();
    let field = MaterialField::read(&e.sketch, s, 1e-10).unwrap();
    let (a, perim, r) = (40.0 * 24.0, 2.0 * (40.0 + 24.0), 1.0);
    let want = 2.0 * r * a + PI * r * r * perim / 2.0 + 4.0 / 3.0 * PI * r.powi(3);
    let got = field_volume(&field, lo.map(|x| x - 2.0), hi.map(|x| x + 2.0), 1.0 / 32.0);
    assert!((got - want).abs() < 2e-3 * want, "{got} against {want}");
}

/// A ball about a point of the near face: within its side, half the ball; within 2 of its whole
/// surface, the ball's slice `π (4r² − 16/3)`; and both a face and its surface name the same
/// plane, though the ball reaches past the patch.
#[test]
fn a_face_has_a_side_and_a_surface() {
    let e = solved(&format!("{BLOCK}c := point hint((20, 0, 12))\nball := std.Sphere(c, r: 5)\n\
                             half := {{ p | p inside ball; p inside block.near }}\n\
                             slice := {{ p | p inside ball; p distance(<= 2mm) block.near.surface }}\n\
                             sh := solid(half)\nss := solid(slice)\n"));
    let (lo, hi) = block_box(&e);
    // the ball's centre on the near face, wherever the block stands
    let near = |f: &MaterialField| f.side([20.0, 0.0, 12.0]);
    let _ = near;
    let r: f64 = 5.0;
    for (name, want) in [("sh", 2.0 / 3.0 * PI * r.powi(3)), ("ss", PI * (4.0 * r * r - 16.0 / 3.0))] {
        let field = MaterialField::read(&e.sketch, ent(&e, name).i(), 1e-10).unwrap();
        let got = field_volume(&field, lo.map(|x| x - 20.0), hi.map(|x| x + 20.0), 1.0 / 16.0);
        assert!((got - want).abs() < 2e-3 * want, "{name}: {got} against {want}");
    }
}

/// The material's angle where two faces meet: a block's edges are convex (a quarter turn), and an
/// L's inner edge concave (three quarters), read at the point of the edge nearest.
#[test]
fn the_material_angle_at_an_edge_says_which_way_it_turns() {
    let e = solved(&format!("{BLOCK}c := point hint((0, 0, 0))\nball := std.Sphere(c, r: 300)\n\
        convex := {{ p | p inside ball; angle(block.near, block.south, at: p) <= 180deg }}\n\
        concave := {{ p | p inside ball; angle(block.near, block.south, at: p) >= 180deg }}\n\
        square := {{ p | p inside ball; angle(block.east, block.south, at: p) <= 91deg; \
                         angle(block.east, block.south, at: p) >= 89deg }}\n\
        inner := {{ p | p inside ball; angle(step.lt, step.lr, at: p) >= 269deg }}\n\
        outer := {{ p | p inside ball; angle(step.le, step.lt, at: p) >= 180deg }}\n\
        s1 := solid(convex)\ns2 := solid(concave)\ns3 := solid(square)\ns4 := solid(inner)\n\
        s5 := solid(outer)\n"));
    let filled = |name: &str| {
        let field = MaterialField::read(&e.sketch, ent(&e, name).i(), 1e-10).unwrap();
        field.side([1.0, 2.0, 3.0]) < 0.0
    };
    assert!(filled("s1"));
    assert!(!filled("s2"));
    assert!(filled("s3"));
    assert!(filled("s4"));
    assert!(!filled("s5"));
}

/// A face path naming no face of the solid (or one a boolean ate) is no point: a set within its
/// side holds nowhere.
#[test]
fn a_face_path_naming_no_face_holds_nowhere() {
    let e = read(&format!("{BLOCK}nowhere := {{ p | p inside block.top }}\nsn := solid(nowhere)\n"));
    let field = MaterialField::read(&e.sketch, ent(&e, "sn").i(), 1e-10).unwrap();
    assert!(field.side([20.0, 6.0, 12.0]) > 0.0 && field.side([0.0; 3]) > 0.0);
}

/// The edge where the near face meets the south face, and the faces' surfaces' meeting.
const EDGE: &str = "crease := { q | q coincident block.near; q coincident block.south }\n\
                    meeting := { q | q coincident block.near.surface; q coincident block.south.surface }\n";

/// **The distance from a set** (#183 F2): a weld's reach, the points within 1 of the block's edge
/// 40 long, a capsule `πr²L + 4πr³/3`.
#[test]
fn a_reach_from_an_edge_is_a_capsule() {
    let e = solved(&format!("{BLOCK}{EDGE}weld := {{ p | distance(p, crease) <= 1mm }}\n\
                             s := solid(weld)\n"));
    let (lo, hi) = block_box(&e);
    let field = MaterialField::read(&e.sketch, ent(&e, "s").i(), 1e-10).unwrap();
    let want = PI * 40.0 + 4.0 / 3.0 * PI;
    let got = field_volume(&field, lo.map(|x| x - 2.0), hi.map(|x| x + 2.0), 1.0 / 32.0);
    assert!((got - want).abs() < 2e-3 * want, "{got} against {want}");
}

/// A clearance envelope: the points within 1 of the block, a set of six faces' sides, is the block
/// grown (Steiner): `abc + 2(ab + bc + ca)r + πr²(a + b + c) + 4πr³/3`.
#[test]
fn a_clearance_is_a_set_grown() {
    let e = solved(&format!("{BLOCK}held := {{ c | c inside block.near; c inside block.far; \
                             c inside block.south; c inside block.north; c inside block.east; \
                             c inside block.west }}\n\
                             clear := {{ p | distance(p, held) <= 1mm }}\ns := solid(clear)\n"));
    let (lo, hi) = block_box(&e);
    let field = MaterialField::read(&e.sketch, ent(&e, "s").i(), 1e-10).unwrap();
    let (a, b, c, r) = (40.0, 24.0, 12.0, 1.0);
    let want = a * b * c + 2.0 * (a * b + b * c + c * a) * r + PI * r * r * (a + b + c)
        + 4.0 / 3.0 * PI * r.powi(3);
    let got = field_volume(&field, lo.map(|x| x - 2.0), hi.map(|x| x + 2.0), 1.0 / 8.0);
    assert!((got - want).abs() < 2e-3 * want, "{got} against {want}");
}

/// The points whose nearest point of the faces' meeting is on their edge: the slab across the
/// edge's 40, within a ball of 30 about its middle `π(2hR² − 2h³/3)`, h = 20; and the same slab as
/// the side of each face the edge ends on it runs on (#183 F7).
#[test]
fn the_reach_of_an_edge_and_where_it_ends_are_one_slab() {
    let e = solved(&format!("{BLOCK}{EDGE}c := point hint((20, 6, 0))\nball := std.Sphere(c, r: 30)\n\
                             reach := {{ p | p inside ball; distance(p, crease) <= distance(p, meeting) }}\n\
                             ended := {{ p | p inside ball; p inside crease.ends }}\n\
                             sr := solid(reach)\nse := solid(ended)\n"));
    let (r, h): (f64, f64) = (30.0, 20.0);
    let want = PI * (2.0 * h * r * r - 2.0 * h.powi(3) / 3.0);
    for name in ["sr", "se"] {
        let field = MaterialField::read(&e.sketch, ent(&e, name).i(), 1e-10).unwrap();
        assert!(field.side([20.0, 6.0, 25.0]) < 0.0 && field.side([45.0, 6.0, 0.0]) > 0.0, "{name}");
        let got = field_volume(&field, [-10.0, -24.0, -30.0], [50.0, 36.0, 30.0], 30.0 / 128.0);
        assert!((got - want).abs() < 2e-3 * want, "{name}: {got} against {want}");
    }
}


/// What a ball rolled along a convex edge cannot reach, written over F1–F3, F7 and F9 (#183 X3's
/// `Run`), on one edge of the block: `r²(1 − π/4)` along its 40.
const RUN: &str = "component Run(a: face, b: face, r: Length) := { p |\n\
    p inside a\n    p inside b\n    angle(a, b, at: p) <= 180deg\n\
    centres := { c | c inside a; c distance(>= r) a.surface; c inside b; c distance(>= r) b.surface }\n\
    crease := { q | q coincident a; q coincident b }\n\
    meeting := { q | q coincident a.surface; q coincident b.surface }\n\
    distance(p, centres) >= r\n\
    distance(p, crease) <= distance(p, meeting)\n\
    p inside crease.ends\n}\n";

#[test]
fn a_run_along_one_edge() {
    let e = solved(&format!("{BLOCK}{RUN}run := Run(block.near, block.south, r: 3mm)\n\
                             taken := solid(run)\n"));
    let field = MaterialField::read(&e.sketch, ent(&e, "taken").i(), 1e-10).unwrap();
    let want = 9.0 * (1.0 - PI / 4.0) * 40.0;
    // the edge runs along x at y = z = 0; its round within 3 of it
    let got = field_volume(&field, [-1.0, -4.0, -4.0], [41.0, 4.0, 4.0], 3.0 / 64.0);
    assert!((got - want).abs() < 2e-3 * want, "{got} against {want}");
}


/// **A word that makes solids** (#183 F4, F5, F8): `round` over two face groups, a run per pair
/// of faces cut from the body the faces are of — here one pair, the block's near and south faces,
/// so the block loses `r²(1 − π/4)` along its 40.
const ROUND: &str = "A round(r) B := {\n\
    repeat a in A {\n\
      repeat b in B if b != a {\n\
        run := Run(a, b, r: r)\n\
        taken := solid(run)\n\
        taken cut A.body\n\
      }\n\
    }\n}\n";

#[test]
fn a_word_rounds_an_edge() {
    let box_ = ([-1.0, -4.0, -4.0], [41.0, 4.0, 4.0]);
    let sharp = solved(&format!("{BLOCK}"));
    let before = field_volume(&MaterialField::read(&sharp.sketch, ent(&sharp, "block").i(), 1e-10).unwrap(),
                              box_.0, box_.1, 3.0 / 64.0);
    let e = solved(&format!("{BLOCK}{RUN}{ROUND}block.near round(r: 3mm) block.south\n"));
    let field = MaterialField::read(&e.sketch, ent(&e, "block").i(), 1e-10).unwrap();
    let after = field_volume(&field, box_.0, box_.1, 3.0 / 64.0);
    let want = 9.0 * (1.0 - PI / 4.0) * 40.0;
    assert!((before - after - want).abs() < 2e-3 * want, "{} against {want}", before - after);
}

/// **A library feature written in Solvent** (#183 step 4's gate): a word that makes a solid and cuts
/// it from the body its operands are faces of — an edge relief, the points within `d` of both
/// faces' patches — over each pair of faces it names: the block loses `d²` along its 40.
#[test]
fn a_word_makes_a_relief() {
    let word = "A relieved(d) B := {\n\
                  repeat a in A {\n\
                    repeat b in B if b != a {\n\
                      groove := { p | p distance(<= d) a; p distance(<= d) b; p inside a; p inside b }\n\
                      relief := solid(groove)\n\
                      relief cut A.body\n\
                    }\n\
                  }\n\
                }\n";
    let box_ = ([-1.0, -2.0, -2.0], [41.0, 2.0, 2.0]);
    let sharp = solved(BLOCK);
    let before = field_volume(&MaterialField::read(&sharp.sketch, ent(&sharp, "block").i(), 1e-10).unwrap(),
                              box_.0, box_.1, 1.0 / 32.0);
    let e = solved(&format!("{BLOCK}{word}block.near relieved(d: 1mm) block.south\n"));
    let after = field_volume(&MaterialField::read(&e.sketch, ent(&e, "block").i(), 1e-10).unwrap(),
                             box_.0, box_.1, 1.0 / 32.0);
    let want = 40.0;
    assert!((before - after - want).abs() < 2e-3 * want, "{} against {want}", before - after);
}

/// **One meaning, two representations**: a face path names the face, `.surface` the surface it
/// lies on, whoever reads it.  A sketch on the block's near face (a plane on its surface, read by
/// the solve as rows) leaves a region's reading of the same face (the patch, read off the solid)
/// as it was: the pillow round the near face is no thicker past the face's edge for it.
#[test]
fn a_sketch_on_a_face_leaves_a_regions_reading_of_it() {
    let region = "c := point hint((20, 0, 12))\nfix((20, 0, 12)) c\nball := std.Sphere(c, r: 30)\n\
                  pillow := { p | p inside ball; p distance(<= 1mm) block.near }\n\
                  s := solid(pillow)\n";
    let sketch = "v1 := plane\nv1.origin coincident block.near.surface\n\
                  v1 tangent(at: v1.origin) block.near.surface\n";
    // past the near face's edge (x = 40) and in front of its plane (y = 0): within 1 of the
    // plane, but more than 1 from the face
    let (past, over) = ([45.0, -0.5, 12.0], [20.0, -0.5, 12.0]);
    for extra in ["", sketch] {
        let e = solved(&format!("{BLOCK}{region}{extra}"));
        let field = MaterialField::read(&e.sketch, ent(&e, "s").i(), 1e-10).unwrap();
        assert!(field.side(past) > 0.0 && field.side(over) < 0.0, "{extra:?}");
    }
}
