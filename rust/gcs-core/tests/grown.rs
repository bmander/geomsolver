//! **A body grown** (`std.Grown`): the points within `d` of a body, `{ p | distance(p, within) <=
//! d }` over `within := { c | c inside B }`, built exactly (`solid::grown`) as the body and what
//! lies within reach of each face, edge and corner glued to it, where every edge of the body is
//! convex and every corner a prism's; shrunk, or a body with a concave edge, read by the field and
//! refused by name.  Held to closed forms: Steiner's for a polytope, Pappus's for a band's rims.
use gcs_core::program::Elaborated;
use gcs_core::solid::region::{route, Route};
use gcs_core::solid::MaterialField;
use std::f64::consts::PI;

use crate::common::{ent, text::solved};

/// A block 40 × 24 × 12 drawn on the front plane (x, z), standing 12 deep along y; a second one
/// 20 along x from it; an L of two steps whose inner edge is concave.
const BLOCKS: &str = "unit mm\nuse std\nin std.front {\n\
    b0 := point\nb1 := point\nb2 := point\nb3 := point\n\
    fix((0, 0)) b0\nfix((40, 0)) b1\nfix((40, 24)) b2\nfix((0, 24)) b3\n\
    (south := line(b0, b1)) -> (east := line(b1, b2)) -> (north := line(b2, b3)) -> \
    (west := line(b3, b0)) -> close\n\
    outline := face(south, east, north, west)\n\
    c0 := point\nc1 := point\nc2 := point\nc3 := point\n\
    fix((60, 0)) c0\nfix((100, 0)) c1\nfix((100, 24)) c2\nfix((60, 24)) c3\n\
    (s2 := line(c0, c1)) -> (e2 := line(c1, c2)) -> (n2 := line(c2, c3)) -> (w2 := line(c3, c0)) -> close\n\
    beside := face(s2, e2, n2, w2)\n\
    l0 := point\nl1 := point\nl2 := point\nl3 := point\nl4 := point\nl5 := point\n\
    fix((0, 100)) l0\nfix((20, 100)) l1\nfix((20, 110)) l2\nfix((10, 110)) l3\nfix((10, 120)) l4\n\
    fix((0, 120)) l5\n\
    (ls := line(l0, l1)) -> (le := line(l1, l2)) -> (lt := line(l2, l3)) -> (lr := line(l3, l4)) -> \
    (ln := line(l4, l5)) -> (lw := line(l5, l0)) -> close\n\
    ell := face(ls, le, lt, lr, ln, lw)\n}\n\
    construction block := solid(outline, depth: 12mm)\n\
    construction pair := solid(outline, depth: 12mm)\nother := solid(beside, depth: 12mm)\n\
    other union pair\n\
    construction step := solid(ell, depth: 12mm)\n";

/// On the top plane: a post of radius 5 and 10 tall; a hex bar 6 to its corners and 8 tall; a
/// plate 40 square and 5 thick with a bore of radius 6 through it.
const TURNED: &str = "unit mm\nuse std\nin std.top {\n\
    o := point\nfix((0, 0)) o\nrim := radius(5mm) circle(center: o)\n\
    h := point\nfix((40, 0)) h\nhx := point\nfix((50, 0)) hx\nref := line(h, hx)\n\
    hexes := std.Polygon(h, ref, n: 6, r: 6mm, phase: 0deg)\n\
    hex := face(hexes.v[0], hexes.v[1], hexes.v[2], hexes.v[3], hexes.v[4], hexes.v[5], -> close)\n\
    p0 := point\np1 := point\np2 := point\np3 := point\n\
    fix((80, -20)) p0\nfix((120, -20)) p1\nfix((120, 20)) p2\nfix((80, 20)) p3\n\
    (ps := line(p0, p1)) -> (pe := line(p1, p2)) -> (pn := line(p2, p3)) -> (pw := line(p3, p0)) -> close\n\
    plate := face(ps, pe, pn, pw)\n\
    bc := point\nfix((100, 0)) bc\nbore := radius(6mm) circle(center: bc)\nhole := face(bore)\n}\n\
    construction post := solid(face(rim), from: 0mm, to: 10mm)\n\
    construction bar := solid(hex, from: 0mm, to: 8mm)\n\
    construction slab := solid(plate, from: 0mm, to: 5mm)\n\
    drill := solid(hole, from: -1mm, to: 6mm)\ndrill cut slab\n";

/// `name`'s route, and its exact volume.
fn exact(e: &Elaborated, name: &str) -> f64 {
    let s = ent(e, name).i();
    let r = route(&e.sketch, s).unwrap_or_else(|m| panic!("`{name}`: {m}"));
    assert!(matches!(r, Route::Grown(Ok(_))), "`{name}` is not built as a body grown");
    let x = e.sketch.exact_solid(s).unwrap_or_else(|m| panic!("`{name}`: {m}"));
    gcs_core::brep::props::volume(&x.brep)
}

/// Steiner's for a box `a × b × c` grown by `r`.
fn steiner(a: f64, b: f64, c: f64, r: f64) -> f64 {
    a * b * c + 2.0 * (a * b + b * c + c * a) * r + PI * r * r * (a + b + c) + 4.0 / 3.0 * PI * r.powi(3)
}

/// A block grown by 2 is its faces' prisms, its edges' quarter cylinders and its corners' eighths
/// of a ball glued to it: Steiner's volume, and six planes, twelve cylinders and eight spheres.
#[test]
fn a_block_grown_is_built_exactly() {
    let e = solved(&format!("{BLOCKS}g := std.Grown(block, d: 2mm)\n"));
    let got = exact(&e, "g.body");
    let want = steiner(40.0, 24.0, 12.0, 2.0);
    assert!((got - want).abs() <= 1e-9 * want, "{got} against {want}");
    let x = e.sketch.exact_solid(ent(&e, "g.body").i()).unwrap();
    let count = |k: fn(&gcs_core::brep::geom::Surface) -> bool| x.brep.faces.iter().filter(|f| k(&f.surface)).count();
    use gcs_core::brep::geom::Surface;
    assert_eq!(count(|s| matches!(s, Surface::Plane(_))), 6);
    assert_eq!(count(|s| matches!(s, Surface::Cylinder(..))), 12);
    assert_eq!(count(|s| matches!(s, Surface::Sphere(..))), 8);
}

/// A post's band grows its radius, its rims are quarters of a torus (Pappus: the quarter disc's
/// centroid `4d/3π` out from the rim); a hex bar's corners are a prism's, the ball's sector between
/// two sides turned about the top's normal; a bore's band shrinks its radius, its rims' tori
/// within it.
#[test]
fn curved_and_slanted_faces_grown_are_built_exactly() {
    let e = solved(&format!("{TURNED}gp := std.Grown(post, d: 1mm)\ngb := std.Grown(bar, d: 1mm)\n\
                             gs := std.Grown(slab, d: 1mm)\n"));
    let d = 1.0;
    let quarter = PI * d * d / 4.0;
    let (r, h) = (5.0, 10.0);
    let post = PI * (r + d) * (r + d) * h + 2.0 * (PI * r * r * d + quarter * 2.0 * PI * (r + 4.0 * d / (3.0 * PI)));
    let s = 6.0;
    let (area, perimeter, tall) = (3.0 * 3f64.sqrt() / 2.0 * s * s, 6.0 * s, 8.0);
    // each edge's wedge its length times its turn times `d²/2`: the caps' a quarter turn, the
    // sides' a sixth
    let wedges = 12.0 * s * PI / 2.0 + 6.0 * tall * PI / 3.0;
    let bar = area * tall + (2.0 * area + perimeter * tall) * d + d * d / 2.0 * wedges + 4.0 / 3.0 * PI * d.powi(3);
    let (w, t, b) = (40.0, 5.0, 6.0);
    let slab = steiner(w, w, t, d) - PI * b * b * t - 2.0 * PI * b * b * d + PI * (b * b - (b - d) * (b - d)) * t
        + 2.0 * quarter * 2.0 * PI * (b - 4.0 * d / (3.0 * PI));
    for (name, want) in [("gp.body", post), ("gb.body", bar), ("gs.body", slab)] {
        let got = exact(&e, name);
        assert!((got - want).abs() <= 1e-9 * want, "{name}: {got} against {want}");
    }
}

/// A body in two pieces is grown piece by piece: twice Steiner's.  Its field reads the distance
/// from the nearer piece, where within both pieces at once is within neither.
#[test]
fn a_body_in_pieces_is_grown_piece_by_piece() {
    let e = solved(&format!("{BLOCKS}g := std.Grown(pair, d: 2mm)\n"));
    let want = 2.0 * steiner(40.0, 24.0, 12.0, 2.0);
    let got = exact(&e, "g.body");
    assert!((got - want).abs() <= 1e-9 * want, "{got} against {want}");
    let field = MaterialField::read(&e.sketch, ent(&e, "g.body").i(), 1e-10).unwrap();
    // under each block's south face, within and beyond the reach
    for (p, inside) in [([20.0, 6.0, -1.9], true), ([80.0, 6.0, -1.9], true), ([20.0, 6.0, -2.1], false),
                        ([50.0, 6.0, 12.0], false), ([41.9, 6.0, 12.0], true)] {
        assert_eq!(field.side(p) < 0.0, inside, "{p:?}: {}", field.side(p));
    }
}

/// A concave edge leaves the body grown to the field, said by name; so does a body shrunk.  At
/// nothing it is the body.
#[test]
fn what_is_not_built_is_said() {
    let e = solved(&format!("{BLOCKS}gl := std.Grown(step, d: 1mm)\nz := std.Grown(block, d: 0mm)\n"));
    let r = route(&e.sketch, ent(&e, "gl.body").i()).unwrap();
    assert!(r.field_only());
    let Route::Grown(Err(why)) = r else { panic!("`gl.body` is built") };
    assert!(why.contains("a concave edge"), "{why}");
    let z = e.sketch.exact_solid(ent(&e, "z.body").i()).unwrap();
    let (got, want) = (gcs_core::brep::props::volume(&z.brep), 40.0 * 24.0 * 12.0);
    assert!((got - want).abs() <= 1e-9 * want, "{got} against {want}");
    let e = solved(&format!("{TURNED}sp := std.Grown(post, d: -1mm)\n"));
    assert!(route(&e.sketch, ent(&e, "sp.body").i()).unwrap().field_only());
}

/// A prism drawn far from its plane's origin is read within a box of its own size (its profile's
/// box carried by its frame), not within the disc about the origin reaching it.
#[test]
fn a_prism_far_from_its_planes_origin_has_its_own_box() {
    let e = solved("unit mm\nuse std\nin std.front {\n\
        b0 := point\nb1 := point\nb2 := point\nb3 := point\n\
        fix((1000, 1000)) b0\nfix((1040, 1000)) b1\nfix((1040, 1024)) b2\nfix((1000, 1024)) b3\n\
        (s := line(b0, b1)) -> (e := line(b1, b2)) -> (n := line(b2, b3)) -> (w := line(b3, b0)) -> close\n\
        outline := face(s, e, n, w)\n}\nblock := solid(outline, depth: 12mm)\n");
    let field = MaterialField::read(&e.sketch, ent(&e, "block").i(), 1e-10).unwrap();
    let b = field.support_bounds().unwrap().expect("a box");
    let size: Vec<f64> = b.iter().map(|x| x.bounds()[1] - x.bounds()[0]).collect();
    for (k, want) in [40.0, 12.0, 24.0].into_iter().enumerate() {
        assert!(size[k] <= want + 1e-6, "{size:?}");
    }
}
