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

/// A face path names a face the solid has, and a term about a face stands only in a region.
#[test]
fn a_face_path_names_a_face() {
    let e = read(&format!("{BLOCK}nowhere := {{ p | p inside block.top }}\nsn := solid(nowhere)\n"));
    let field = MaterialField::read(&e.sketch, ent(&e, "sn").i(), 1e-10);
    assert!(field.is_err_and(|m| m.contains("names no face")));
}
