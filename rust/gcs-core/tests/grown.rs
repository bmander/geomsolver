//! **A body grown** (`std.rounds.Grown`): the points within `d` of a body, `{ p | distance(p,
//! within) <= d }` over `within := { c | c inside B }`, built exactly (`solid::grown`) as the body
//! and what lies within reach of each face, edge and corner glued to it, every corner a prism's or
//! a concave one, the prisms beside a concave edge cut back to its bisector; shrunk, or a boss's
//! foot, read by the field and refused by name.  Held to closed forms: Steiner's for a polytope,
//! its concave edges' terms negative, Pappus's for a band's rims.
use gcs_core::brep::geom::Surface;
use gcs_core::brep::props::volume;
use gcs_core::program::Elaborated;
use gcs_core::solid::Exact;
use gcs_core::solid::region::{route, Route};
use gcs_core::solid::MaterialField;
use std::f64::consts::PI;

use crate::common::{ent, text::solved};

/// A block 40 × 24 × 12 drawn on the front plane (x, z), standing 12 deep along y; a second one
/// 20 along x from it; an L of two steps whose inner edge is concave.
const BLOCKS: &str = "unit mm\nuse std\nuse std.shapes\nuse std.rounds\nin std.front {\n\
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
const TURNED: &str = "unit mm\nuse std\nuse std.rounds\nuse std.shapes\nin std.top {\n\
    o := point\nfix((0, 0)) o\nrim := radius(5mm) circle(center: o)\n\
    h := point\nfix((40, 0)) h\nhx := point\nfix((50, 0)) hx\nref := line(h, hx)\n\
    hexes := std.shapes.Polygon(h, ref, n: 6, r: 6mm, phase: 0deg)\n\
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

/// On the top plane, a slab 40 square and 10 thick three times over: a tray, a pocket 20 × 16 and
/// 4 deep cut into its top; a plinth, a boss 10 square standing 5 on it; a base, a post of radius
/// 5 standing 5 on it.
const STANDING: &str = "unit mm\nuse std\nuse std.shapes\nuse std.rounds\nin std.top {\n\
    q0 := point\nq1 := point\nq2 := point\nq3 := point\n\
    fix((0, 0)) q0\nfix((40, 0)) q1\nfix((40, 40)) q2\nfix((0, 40)) q3\n\
    (qs := line(q0, q1)) -> (qe := line(q1, q2)) -> (qn := line(q2, q3)) -> (qw := line(q3, q0)) -> close\n\
    slab := face(qs, qe, qn, qw)\n\
    k0 := point\nk1 := point\nk2 := point\nk3 := point\n\
    fix((10, 12)) k0\nfix((30, 12)) k1\nfix((30, 28)) k2\nfix((10, 28)) k3\n\
    (ks := line(k0, k1)) -> (ke := line(k1, k2)) -> (kn := line(k2, k3)) -> (kw := line(k3, k0)) -> close\n\
    pocket := face(ks, ke, kn, kw)\n\
    m0 := point\nm1 := point\nm2 := point\nm3 := point\n\
    fix((15, 15)) m0\nfix((25, 15)) m1\nfix((25, 25)) m2\nfix((15, 25)) m3\n\
    (ms := line(m0, m1)) -> (me := line(m1, m2)) -> (mn := line(m2, m3)) -> (mw := line(m3, m0)) -> close\n\
    stud := face(ms, me, mn, mw)\n\
    o := point\nfix((20, 20)) o\nrim := radius(5mm) circle(center: o)\n}\n\
    construction tray := solid(slab, from: 0mm, to: 10mm)\n\
    cavity := solid(pocket, from: 6mm, to: 11mm)\ncavity cut tray\n\
    construction plinth := solid(slab, from: 0mm, to: 10mm)\n\
    boss := solid(stud, from: 9mm, to: 15mm)\nboss union plinth\n\
    construction base := solid(slab, from: 0mm, to: 10mm)\n\
    post := solid(face(rim), from: 9mm, to: 15mm)\npost union base\n";

/// `name` built as a body grown, exactly.
fn exact(e: &Elaborated, name: &str) -> std::rc::Rc<Exact> {
    let s = ent(e, name).i();
    let r = route(&e.sketch, s).unwrap_or_else(|m| panic!("`{name}`: {m}"));
    match r {
        Route::Grown(Ok(_)) => {}
        Route::Grown(Err(why)) => panic!("`{name}`: {why}"),
        _ => panic!("`{name}` is not built as a body grown"),
    }
    e.sketch.exact_solid(s).unwrap_or_else(|m| panic!("`{name}`: {m}"))
}

/// Steiner's for a box `a × b × c` grown by `r`.
fn steiner(a: f64, b: f64, c: f64, r: f64) -> f64 {
    a * b * c + 2.0 * (a * b + b * c + c * a) * r + PI * r * r * (a + b + c) + 4.0 / 3.0 * PI * r.powi(3)
}

/// A block grown by 2 is its faces' prisms, its edges' quarter cylinders and its corners' eighths
/// of a ball glued to it: Steiner's volume, and six planes, twelve cylinders and eight spheres.
#[test]
fn a_block_grown_is_built_exactly() {
    let e = solved(&format!("{BLOCKS}g := std.rounds.Grown(block, d: 2mm)\n"));
    let x = exact(&e, "g.body");
    let (got, want) = (volume(&x.brep), steiner(40.0, 24.0, 12.0, 2.0));
    assert!((got - want).abs() <= 1e-9 * want, "{got} against {want}");
    let count = |k: fn(&Surface) -> bool| x.brep.faces.iter().filter(|f| k(&f.surface)).count();
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
    let e = solved(&format!("{TURNED}gp := std.rounds.Grown(post, d: 1mm)\ngb := std.rounds.Grown(bar, d: 1mm)\n\
                             gs := std.rounds.Grown(slab, d: 1mm)\n"));
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
        let got = volume(&exact(&e, name).brep);
        assert!((got - want).abs() <= 1e-9 * want, "{name}: {got} against {want}");
    }
}

/// A body in two pieces is grown piece by piece: twice Steiner's.  Its field reads the distance
/// from the nearer piece, where within both pieces at once is within neither.
#[test]
fn a_body_in_pieces_is_grown_piece_by_piece() {
    let e = solved(&format!("{BLOCKS}g := std.rounds.Grown(pair, d: 2mm)\n"));
    let want = 2.0 * steiner(40.0, 24.0, 12.0, 2.0);
    let got = volume(&exact(&e, "g.body").brep);
    assert!((got - want).abs() <= 1e-9 * want, "{got} against {want}");
    let field = MaterialField::read(&e.sketch, ent(&e, "g.body").i(), 1e-10).unwrap();
    // under each block's south face, within and beyond the reach
    for (p, inside) in [([20.0, 6.0, -1.9], true), ([80.0, 6.0, -1.9], true), ([20.0, 6.0, -2.1], false),
                        ([50.0, 6.0, 12.0], false), ([41.9, 6.0, 12.0], true)] {
        assert_eq!(field.side(p) < 0.0, inside, "{p:?}: {}", field.side(p));
    }
}

/// Beside a concave edge the two faces' prisms are cut back to its bisector, so each concave edge
/// of length `L` takes `d² L` (a square's): Steiner's edge term negative.  An L-block's step: at
/// each end of its inner edge the two wedges along the end face are cut back to the bisector too
/// (each losing `d³/3`), and no sector of the ball stands there.  A tray's pocket: its rim's corners
/// are the step's, its floor's three prisms meeting in a cube counted three times less three (`+d³`).
#[test]
fn concave_edges_grown_are_built_exactly() {
    let d = 1.0;
    let e = solved(&format!("{BLOCKS}gl := std.rounds.Grown(step, d: 1mm)\n"));
    // the L 80 round and 300 square, 12 deep: five convex edges along its depth and one
    // concave, five prism's corners and one concave at each end
    let step = 3600.0 + (2.0 * 300.0 + 80.0 * 12.0) * d + PI / 4.0 * (2.0 * 80.0 + 5.0 * 12.0) * d * d
        - 12.0 * d * d + (10.0 * PI / 6.0 - 4.0 / 3.0) * d.powi(3);
    let got = volume(&exact(&e, "gl.body").brep);
    assert!((got - step).abs() <= 1e-9 * step, "the step: {got} against {step}");
    let e = solved(&format!("{STANDING}gt := std.rounds.Grown(tray, d: 1mm)\n"));
    let (w, t, a, b, h) = (40.0, 10.0, 20.0, 16.0, 4.0);
    let rim = 2.0 * (a + b);
    let tray = w * w * t - a * b * h + (2.0 * (w * w + 2.0 * w * t) + rim * h) * d
        + PI / 4.0 * (4.0 * (2.0 * w + t) + rim) * d * d - (rim + 4.0 * h) * d * d
        + (8.0 * PI / 6.0 - 4.0 * 2.0 / 3.0 + 4.0) * d.powi(3);
    let got = volume(&exact(&e, "gt.body").brep);
    assert!((got - tray).abs() <= 1e-9 * tray, "the tray: {got} against {tray}");
}

/// Within a body with a concave edge no one face's side bounds the material, so its field reads
/// the body's own signed distance: at a convex corner of the step the edge's, never the plane's of
/// a face beside it; shrunk, at its inner corner the corner's.
#[test]
fn a_concave_bodys_field_reads_its_signed_distance() {
    let e = solved(&format!("{BLOCKS}gl := std.rounds.Grown(step, d: 1mm)\n"));
    let field = MaterialField::read(&e.sketch, ent(&e, "gl.body").i(), 1e-10).unwrap();
    // about the edge along y through (0, 100), beside a face, and in the step's inner corner
    for (p, want) in [([-0.8, 8.0, 99.2], 0.8f64.hypot(0.8)), ([-0.6, 8.0, 99.2], 1.0),
                      ([5.0, 8.0, 98.9], 1.1), ([10.8, 8.0, 111.5], 0.8), ([10.5, 8.0, 110.8], 0.5)] {
        let got = field.side(p);
        assert!((got - (want - 1.0)).abs() <= 1e-9, "{p:?}: {got} against {}", want - 1.0);
    }
    // shrunk, beyond it alike: within the step's inner corner, the corner's distance
    let e = solved(&format!("{BLOCKS}gs := std.rounds.Grown(step, d: -1mm)\n"));
    let field = MaterialField::read(&e.sketch, ent(&e, "gs.body").i(), 1e-10).unwrap();
    let (got, want) = (field.side([9.4, 8.0, 109.4]), 1.0 - 0.6 * 2f64.sqrt());
    assert!((got - want).abs() <= 1e-9, "{got} against {want}");
}

/// A boss's foot, a convex edge standing between two concave ones, leaves the body grown to the
/// field, said by name; so does a post's, its bisector a cone, and a pocket narrower than twice the
/// reach, and a body shrunk.  At nothing it is the body.
#[test]
fn what_is_not_built_is_said() {
    let e = solved(&format!("{STANDING}gp := std.rounds.Grown(plinth, d: 1mm)\ngb := std.rounds.Grown(base, d: 1mm)\n\
                             gn := std.rounds.Grown(tray, d: 9mm)\n"));
    for (name, says) in [("gp.body", "a convex edge stands between two concave ones"),
                         ("gb.body", "a concave edge on a curved face"),
                         ("gn.body", "a face narrower than the reach beside a concave edge")] {
        let r = route(&e.sketch, ent(&e, name).i()).unwrap();
        let Route::Grown(Err(why)) = r else { panic!("`{name}` is built") };
        assert!(why.contains(says), "`{name}`: {why}");
    }
    let e = solved(&format!("{BLOCKS}z := std.rounds.Grown(block, d: 0mm)\n"));
    let z = e.sketch.exact_solid(ent(&e, "z.body").i()).unwrap();
    let (got, want) = (volume(&z.brep), 40.0 * 24.0 * 12.0);
    assert!((got - want).abs() <= 1e-9 * want, "{got} against {want}");
    let e = solved(&format!("{TURNED}sp := std.rounds.Grown(post, d: -1mm)\n"));
    assert!(route(&e.sketch, ent(&e, "sp.body").i()).unwrap().field_only());
}

/// A prism drawn far from its plane's origin is read within a box of its own size (its profile's
/// box carried by its frame), not within the disc about the origin reaching it.
#[test]
fn a_prism_far_from_its_planes_origin_has_its_own_box() {
    let e = solved("unit mm\nuse std\nuse std.shapes\nuse std.rounds\nin std.front {\n\
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
