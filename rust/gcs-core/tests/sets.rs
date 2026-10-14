//! **Sets** (§6.21): a shape written as the points that satisfy a predicate — `{ p | p
//! distance(r) c }`, and the library's sphere, cylinder and cone as families of them — with `q
//! coincident S` the body at `q`, `l tangent S` the body at a contact on `l` with its derivative
//! along `l` — a body with geometry of its own included — and `S1 tangent(at: m) S2` their
//! derivatives at `m` along two directions both share.  Held to closed forms (a sphere's radius
//! from its centre, a cylinder's common perpendicular, a cone's half-angle and its tangent
//! plane), to the count a tangency must add (one condition, regular), to the exact Jacobian the
//! derivative kernel writes, through a document, and to every refusal at its span.
use gcs_core::diagnose::{diagnose, DiagnoseOptions};
use gcs_core::io;
use gcs_core::model::Sketch;
use gcs_core::program::Elaborated;
use gcs_core::solve::{solve, SolveOpts};
use gcs_core::space::{cross, dot, norm, sub};

use crate::common::{ends, ent, fd_jacobian, off_line, read, refused, solved, text::dof, unit};

/// Two planes square to each other: a centre and three axes grounded in the front one — `ax` in
/// the side plane, `bx` and `cx` square to it, `cx` from 30 before it — and a point and a line in
/// the side one.
const VIEWS: &str = "\
unit mm
use std
side := plane(u: std.z, v: std.y)
fix(origin == (0, 0, 0)) side
in std.front {
  c := point
  fix((0, 30)) c
  ax := line
  fix((0, 0)) ax.p1
  fix((0, 50)) ax.p2
  bx := line
  fix((0, 20)) bx.p1
  fix((50, 20)) bx.p2
  cx := line
  fix((-30, 20)) cx.p1
  fix((10, 20)) cx.p2
}
a := point hint((10, 20)) in side
l := line(hint((5, 3)), hint((30, 20))) in side
fix((5, 3)) l.p1
";

/// `VIEWS` with a sphere, a cylinder about each axis and a cone, and `more` after them.
fn with(more: &str) -> String {
    format!(
        "{VIEWS}ball := std.Sphere(c, r: 12)\nshaft := std.Cylinder(ax, r: 10)\n\
         rod := std.Cylinder(bx, r: 12)\nk := std.Cone(ax, half: 30deg)\n\
         kc := std.Cone(cx, half: 20deg)\n{more}\n"
    )
}

fn at(sk: &Sketch, e: &Elaborated, n: &str) -> [f64; 3] {
    sk.world_point(ent(e, n).i())
}

/// The angle in degrees between two directions.
fn angle(a: [f64; 3], b: [f64; 3]) -> f64 {
    dot(unit(a), unit(b)).clamp(-1.0, 1.0).acos().to_degrees()
}

/// Every statement the document's constraints read as, the way a culprit is named.
fn described(e: &Elaborated) -> Vec<String> {
    e.sketch
        .user_constraints()
        .iter()
        .map(|c| io::describe_with(c, &|x| e.map.name_of(x).cloned()))
        .collect()
}

/// A set adds nothing to the drawing until something is put on it: no unknown, no equation,
/// nothing drawn.
#[test]
fn a_set_is_nothing_until_it_is_used() {
    let (bare, sets) = (read(VIEWS), read(&with("near := { p | p distance(5) c }")));
    assert_eq!(sets.sketch.params.len(), bare.sketch.params.len());
    assert_eq!(sets.sketch.user_constraints().len(), bare.sketch.user_constraints().len());
    assert_eq!(sets.sketch.drawn().len(), bare.sketch.drawn().len());
}

/// A point on a sphere is its radius from the centre — across views, the centre in the front
/// and the point in the side — one equation, read as the statement wrote it.
#[test]
fn a_point_on_a_sphere_is_its_radius_from_the_centre() {
    let src = with("a coincident ball");
    let e = read(&src);
    let sk = solved(&e);
    let got = norm(sub(at(&sk, &e, "a"), at(&sk, &e, "c")));
    assert!((got - 12.0).abs() < 1e-9, "{got}");
    assert_eq!(dof(&with("")) - dof(&src), 1);
    assert!(described(&e).contains(&"a coincident ball".to_string()), "{:?}", described(&e));
}

/// A set written in place reads the names of the body it stands in, and its point is whatever is
/// put on it.
#[test]
fn a_set_written_in_place_reads_the_names_around_it() {
    let e = read(&with("near := { p | p distance(20) c }\na coincident near"));
    let sk = solved(&e);
    let got = norm(sub(at(&sk, &e, "a"), at(&sk, &e, "c")));
    assert!((got - 20.0).abs() < 1e-9, "{got}");
    // and a set of two conditions holds both: the point on the sphere and on the shaft
    let both = "on_both := { p |\n  p coincident ball\n  p coincident shaft\n}\na coincident on_both";
    let e = read(&with(both));
    let sk = solved(&e);
    let (p, q) = ends(&sk, ent(&e, "ax"));
    let a = at(&sk, &e, "a");
    assert!((norm(sub(a, at(&sk, &e, "c"))) - 12.0).abs() < 1e-9);
    assert!((off_line(a, p, sub(q, p)) - 10.0).abs() < 1e-9);
}

/// A plane met by a body is said of a point drawn in it already: the body is a predicate, true of
/// that point there, so it states no row — where `m coincident std.front` said directly asks
/// nothing (E061).  A sphere met with the front plane is the circle the front cuts from it.
#[test]
fn a_plane_in_a_body_holds_of_a_point_drawn_in_it() {
    let m = "m := point hint((10, 35)) in std.front
";
    let src = with(&format!("{m}m coincident {{ p | p coincident ball; p coincident std.front }}"));
    let e = read(&src);
    let sk = solved(&e);
    let got = norm(sub(at(&sk, &e, "m"), at(&sk, &e, "c")));
    assert!((got - 12.0).abs() < 1e-9, "{got}");
    // one condition, the sphere's: the plane's holds by construction
    assert_eq!(dof(&with(m)) - dof(&src), 1);
    // and a family whose body says so, the instance's point drawn in its plane
    let ring = "component Ring(on: plane, center: point, r: Length) := { p |
                  p coincident std.Sphere(center, r: r)
  p coincident on
}
";
    let src = with(&format!("{ring}{m}m coincident Ring(std.front, c, r: 12)"));
    let e = read(&src);
    let sk = solved(&e);
    assert!((norm(sub(at(&sk, &e, "m"), at(&sk, &e, "c"))) - 12.0).abs() < 1e-9);
    assert_eq!(dof(&with(m)) - dof(&src), 1);
    // said of the point directly, it is still asked of nothing
    refused(&with(&format!("{m}m coincident std.front")), "E061", "m is drawn in std.front",
            "m coincident std.front");
}

/// A set is found by what its name resolves to, however it is reached: through a group's
/// member, and with its head written over two lines.
#[test]
fn a_set_is_reached_by_any_name_for_it() {
    let e = read(&with("d := {sph: ball}\na coincident d.sph"));
    let sk = solved(&e);
    assert!((norm(sub(at(&sk, &e, "a"), at(&sk, &e, "c"))) - 12.0).abs() < 1e-9);
    let e = read(&with("near := {\n  p |\n  p distance(20) c\n}\na coincident near"));
    let sk = solved(&e);
    assert!((norm(sub(at(&sk, &e, "a"), at(&sk, &e, "c"))) - 20.0).abs() < 1e-9);
}

/// A point on a cylinder stands its radius off the axis, and one on a cone makes the half-angle
/// with the axis at the apex, on the nappe the axis points into.
#[test]
fn a_point_on_a_cylinder_and_on_a_cone() {
    let e = read(&with("a coincident shaft"));
    let sk = solved(&e);
    let (p, q) = ends(&sk, ent(&e, "ax"));
    let got = off_line(at(&sk, &e, "a"), p, sub(q, p));
    assert!((got - 10.0).abs() < 1e-9, "{got}");
    let e = read(&with("a coincident k"));
    let sk = solved(&e);
    let (apex, q) = ends(&sk, ent(&e, "ax"));
    let w = sub(at(&sk, &e, "a"), apex);
    assert!((angle(w, sub(q, apex)) - 30.0).abs() < 1e-9, "{}", angle(w, sub(q, apex)));
    assert!(dot(w, sub(q, apex)) > 0.0, "on the nappe the axis points into");
    assert!(described(&e).contains(&"a coincident k".to_string()), "{:?}", described(&e));
    // and a half-angle left unbound is found by the point
    let e = read(&format!(
        "{VIEWS}k := std.Cone(ax, half: hint(30deg))\nfix((130, 20)) a\na coincident k\n"
    ));
    let sk = solved(&e);
    let (apex, q) = ends(&sk, ent(&e, "ax"));
    let half = sk.params[sk.free_vars["k.half"] as usize].value;
    assert!((half - angle(sub(at(&sk, &e, "a"), apex), sub(q, apex))).abs() < 1e-9, "{half}");
}

/// **A set's body is read in space wherever its point is drawn** (§6.21): a cone about a line
/// drawn in the front is both its generators there, so a point drawn beside the axis lands on
/// the side it is seeded on — the clockwise one, which the page's angle, turning counter-clockwise,
/// could not reach — read as the angle in space.  A number that says the same in the plane keeps
/// the plane's kind.
#[test]
fn a_set_is_read_in_space_in_the_view_its_point_is_drawn_in() {
    let src = with("b := point hint((20, 40)) in std.front\nfix(y == 40) b\nb coincident k");
    let e = read(&src);
    let kinds: Vec<_> = e.sketch.user_constraints().iter().map(|c| c.kind).collect();
    assert_eq!(kinds, [gcs_core::constraints::CKind::Angle3]);
    let sk = solved(&e);
    let (apex, q) = ends(&sk, ent(&e, "ax"));
    let b = at(&sk, &e, "b");
    assert!((angle(sub(b, apex), sub(q, apex)) - 30.0).abs() < 1e-9, "{b:?}");
    assert!(b[0] > 0.0, "clockwise of the axis, where it was seeded: {b:?}");
    // a program lifted from it says the angle the way the pose turns, so it stays there
    let lifted = gcs_core::program::to_program(&sk).text().to_string();
    assert!(lifted.contains("sense: cw"), "{lifted}");
    let again = solved(&read(&lifted));
    for i in 0..sk.points.len() {
        assert!(norm(sub(sk.world_point(i), again.world_point(i))) < 1e-7, "p{i}\n{lifted}");
    }
    // a sphere's distance in the plane is the distance in space: the plane's kind
    let e = read(&with("b := point hint((20, 40)) in std.front\nb coincident ball"));
    let kinds: Vec<_> = e.sketch.user_constraints().iter().map(|c| c.kind).collect();
    assert_eq!(kinds, [gcs_core::constraints::CKind::Distance]);
    // and a side, which a line in space has none of, is refused where the set is used
    refused(&with("wall := { q | q distance(5, side: left) ax }\nb := point hint((20, 40)) in \
                   std.front\nb coincident wall"), "E040", "in space a line has no sides",
            "coincident");
}

/// The mark goes with the set, not with how it was reached: a set used in a relation word's body
/// is read in space as one used outright, and a word used in a set's body too.
#[test]
fn a_set_through_a_word_is_read_in_space() {
    let point = "b := point hint((20, 40)) in std.front\nfix(y == 40) b\n";
    for more in [
        format!("a opens(h) l := a coincident std.Cone(l, half: h)\n{point}b opens(h: 30deg) ax"),
        format!("a sees(h) l := {{\n  private construction g := line(l.p1, a)\n  l angle(h) g\n}}\n\
                 wedge := {{ p | p sees(h: 30deg) ax }}\n{point}b coincident wedge"),
    ] {
        let e = read(&with(&more));
        let kinds: Vec<_> = e.sketch.user_constraints().iter().map(|c| c.kind).collect();
        assert_eq!(kinds, [gcs_core::constraints::CKind::Angle3], "{more}");
        let b = at(&solved(&e), &e, "b");
        assert!(b[0] > 0.0, "{more}: {b:?}");
    }
    // a sense, which an angle in space turns neither way, is refused where the set is used
    refused(&with("fan := { q | ax angle(30deg, sense: cw) (g := line(ax.p1, q)) }\n\
                   b := point hint((20, 40)) in std.front\nb coincident fan"), "E040",
            "turn neither way", "coincident");
}

/// A tangency at a point drawn in the view both spheres' centres are drawn in: each body on its
/// own is the plane's distance, the same statement, but its derivative moves the point out of the
/// plane, so it is read in space.
#[test]
fn a_derivative_at_a_point_drawn_in_a_view_is_read_in_space() {
    let src = "unit mm\nuse std\nin std.front {\n  c1 := point\n  fix((0, 0)) c1\n  \
               c2 := point\n  fix((10, 0)) c2\n  m := point hint((3, 1))\n}\n\
               s1 := std.Sphere(c1, r: 4)\ns2 := std.Sphere(c2, r: hint(5))\n\
               m coincident s1\nm coincident s2\ns1 tangent(at: m) s2\n";
    let e = read(src);
    use gcs_core::constraints::CKind;
    let derived: Vec<_> = e.sketch.constraints.iter().filter(|c| c.along.is_some())
        .map(|c| c.kind).collect();
    assert!(!derived.is_empty() && derived.iter().all(|&k| k == CKind::Distance3), "{derived:?}");
    let sk = solved(&e);
    let m = at(&sk, &e, "m");
    assert!(norm(sub(m, [4.0, 0.0, 0.0])) < 1e-9, "{m:?}");
    assert!((sk.params[sk.free_vars["s2.r"] as usize].value - 6.0).abs() < 1e-9);
    fd_jacobian(&sk, 1e-5);
}

/// Two tangencies written in one set's body are two statements there, each with its own two
/// directions: both pairs touch, as they do written outright, though the application stamps both
/// with the one use's statement.
#[test]
fn two_tangencies_in_one_body_take_directions_apart() {
    let head = "unit mm\nuse std\nin std.front {\n  c1 := point\n  fix((0, 0)) c1\n  \
                c2 := point\n  fix((10, 0)) c2\n  c3 := point\n  fix((0, 40)) c3\n  \
                c4 := point\n  fix((10, 40)) c4\n  m := point hint((3, 1))\n  \
                n := point hint((3, 41))\n  z := point hint((50, 50))\n}\n\
                s1 := std.Sphere(c1, r: 4)\ns2 := std.Sphere(c2, r: hint(5))\n\
                s3 := std.Sphere(c3, r: 4)\ns4 := std.Sphere(c4, r: hint(5))\n\
                m coincident s1\nm coincident s2\nn coincident s3\nn coincident s4\n";
    let pairs = "s1 tangent(at: m) s2\n  s3 tangent(at: n) s4\n";
    for src in [
        format!("{head}{pairs}"),
        format!("{head}both := {{ q |\n  {pairs}}}\nz coincident both\n"),
    ] {
        let e = read(&src);
        let sk = solved(&e);
        for (p, at_, r) in [("m", [4.0, 0.0, 0.0], "s2.r"), ("n", [4.0, 0.0, 40.0], "s4.r")] {
            let got = at(&sk, &e, p);
            assert!(norm(sub(got, at_)) < 1e-9, "{p}: {got:?}\n{src}");
            let r = sk.params[sk.free_vars[r] as usize].value;
            assert!((r - 6.0).abs() < 1e-9, "{r}\n{src}");
        }
    }
}

/// And a plane tangent to two sets in one body: each tangency's directions are its own.
#[test]
fn two_plane_tangencies_in_one_body_take_directions_apart() {
    let head = "unit mm\nuse std\nM := point in std.top\nfix((40, 0)) M\n\
                C := point hint((35, 5, 18))\nb1 := std.Sphere(C, r: 20)\nM coincident b1\n\
                N := point in std.top\nfix((-40, 0)) N\nD := point hint((-35, 5, 18))\n\
                b2 := std.Sphere(D, r: 20)\nN coincident b2\nz := point hint((50, 50, 50))\n";
    let pairs = "std.top tangent(at: M) b1\n  std.top tangent(at: N) b2\n";
    for src in [
        format!("{head}{pairs}"),
        format!("{head}both := {{ q |\n  {pairs}}}\nz coincident both\n"),
    ] {
        let e = read(&src);
        let sk = solved(&e);
        for (p, at_) in [("C", [40.0, 0.0, 20.0]), ("D", [-40.0, 0.0, 20.0])] {
            let got = at(&sk, &e, p);
            assert!(norm(sub(got, at_)) < 1e-9, "{p}: {got:?}\n{src}");
        }
    }
}

/// Two cones tangent at a point drawn in the gear cone's own view, `G`: its body is read where
/// the point stands in space, as across views, so the pair stands as it does with the point
/// drawn in the front.
#[test]
fn two_sets_tangent_at_a_point_drawn_in_one_of_their_views() {
    let src = include_str!("../../examples/hypoid_pitch_cones.sv");
    let drawn = src.replace("M := point in std.front", "M := point in G");
    assert_ne!(drawn, src);
    let (a, b) = (read(src), read(&drawn));
    let (sa, sb) = (solved(&a), solved(&b));
    for n in ["M", "gax.p2", "pax.p1", "pax.p2"] {
        let (pa, pb) = (at(&sa, &a, n), at(&sb, &b, n));
        assert!(norm(sub(pa, pb)) < 1e-6, "{n}: {pa:?} {pb:?}");
    }
    let mut sb = sb;
    let d = diagnose(&mut sb, DiagnoseOptions::default());
    assert_eq!(d.dof, 0, "{d:?}");
    fd_jacobian(&sb, 1e-5);
}

/// A line tangent to a sphere passes the centre at the radius: the contact is the foot of the
/// perpendicular from the centre, on the line and on the sphere.
#[test]
fn a_line_tangent_to_a_sphere_passes_the_centre_at_its_radius() {
    let src = with("l tangent ball");
    let e = read(&src);
    let sk = solved(&e);
    let (p, q) = ends(&sk, ent(&e, "l"));
    let got = off_line(at(&sk, &e, "c"), p, sub(q, p));
    assert!((got - 12.0).abs() < 1e-9, "{got}");
    // one condition: the line keeps one of the two freedoms it had about its fixed end
    assert_eq!(dof(&with("")) - dof(&src), 1);
    let said = described(&e);
    assert!(said.iter().all(|s| s == "l tangent ball"), "{said:?}");
}

/// A line tangent to a cylinder is its radius from the axis along their common perpendicular —
/// what `axis distance(r) l` states directly, found here at a contact.
#[test]
fn a_line_tangent_to_a_cylinder_is_its_radius_from_the_axis() {
    let src = with("l tangent rod");
    let e = read(&src);
    let sk = solved(&e);
    let (p, q) = ends(&sk, ent(&e, "bx"));
    let (a, b) = ends(&sk, ent(&e, "l"));
    let m = cross(sub(q, p), sub(b, a));
    let gap = dot(unit(m), sub(a, p)).abs();
    assert!((gap - 12.0).abs() < 1e-9, "{gap}");
    assert_eq!(dof(&with("")) - dof(&src), 1);
}

/// A line tangent to a cone touches it and stays on one side of it: the half-angle's excess along
/// the line has a double root at the contact, not a crossing.  The cone's axis crosses the side
/// plane square, 30 from its apex, so the line drawn there touches the circle the plane cuts from
/// it, `30 tan 20°` about where the axis crosses.
#[test]
fn a_line_tangent_to_a_cone_touches_without_crossing() {
    let src = with("l tangent kc");
    let e = read(&src);
    let sk = solved(&e);
    let (apex, tip) = ends(&sk, ent(&e, "cx"));
    let (a, b) = ends(&sk, ent(&e, "l"));
    let got = off_line([0.0, 0.0, 20.0], a, sub(b, a));
    let want = 30.0 * 20f64.to_radians().tan();
    assert!((got - want).abs() < 1e-9, "{got} against {want}");
    let d = sub(tip, apex);
    let off = |s: f64| {
        let x = [a[0] + s * (b[0] - a[0]), a[1] + s * (b[1] - a[1]), a[2] + s * (b[2] - a[2])];
        angle(sub(x, apex), d) - 20.0
    };
    let samples: Vec<f64> = (-4000..=8000).map(|i| off(i as f64 / 2000.0)).collect();
    let least = samples.iter().cloned().fold(f64::INFINITY, |m, v| m.min(v.abs()));
    assert!(least < 1e-5, "the line comes within {least}° of the cone");
    let sign = samples.iter().find(|v| v.abs() > 1e-6).unwrap().signum();
    assert!(samples.iter().all(|v| v * sign > -1e-7), "the line crosses the cone: {samples:?}");
    assert_eq!(dof(&with("")) - dof(&src), 1);
}

/// A line held still and tangent to a cone whose half-angle is left unbound finds the half-angle:
/// the derivative reads the row's free twin, its unknown held while the contact moves.
#[test]
fn a_tangency_finds_an_unbound_half_angle() {
    let src = format!(
        "{VIEWS}kd := std.Cone(cx, half: hint(25deg))\nfix((5, 30)) l.p2\nl tangent kd\n"
    );
    let e = read(&src);
    assert!(e.sketch.constraints.iter().any(|c| c.along.is_some() && c.free.is_some()));
    let sk = solved(&e);
    let (a, b) = ends(&sk, ent(&e, "l"));
    let reach = off_line([0.0, 0.0, 20.0], a, sub(b, a));
    let half = sk.params[sk.free_vars["kd.half"] as usize].value;
    let want = (reach / 30.0).atan().to_degrees();
    assert!((half - want).abs() < 1e-9, "{half} against {want}");
    fd_jacobian(&sk, 1e-5);
}

/// A cylinder of radius 4 about `sx`, a line slanting up the front, written with a foot of its
/// own: the point `q` on the axis square across from `p`, which slides along the axis as `p`
/// moves.  `in` draws the foot in a view.
fn footed(view: &str) -> String {
    format!(
        "in std.front {{\n  sx := line\n  fix((0, 0)) sx.p1\n  fix((50, 50)) sx.p2\n}}\n\
         foot := {{ p |\n  q := point hint((10, 10)){view}\n  q coincident sx\n  \
         g := line(p, q)\n  g perpendicular sx\n  p distance(4) q\n}}\n"
    )
}

/// A set whose body makes a point of its own is tangent to a line: the foot's motion along the
/// set is an unknown of the tangency, solved with everything else — the same tangency as the
/// cylinder's, its radius from the axis along their common perpendicular, one condition and a
/// regular one, whether the foot stands in space or is drawn in a view (its lift's row moving
/// with it).
#[test]
fn a_set_with_geometry_of_its_own_is_tangent_to_a_line() {
    for view in ["", " in std.front"] {
        let src = with(&format!("{}l tangent foot", footed(view)));
        let e = read(&src);
        let sk = solved(&e);
        let (p, q) = ends(&sk, ent(&e, "sx"));
        let (a, b) = ends(&sk, ent(&e, "l"));
        let m = cross(sub(q, p), sub(b, a));
        let gap = dot(unit(m), sub(a, p)).abs();
        assert!((gap - 4.0).abs() < 1e-9, "{view:?}: {gap}");
        assert_eq!(dof(&with(&footed(view))) - dof(&src), 1, "{view:?}");
        let mut sk = sk;
        let d = diagnose(&mut sk, DiagnoseOptions::default());
        assert!(d.over.is_empty() && d.implied.is_empty() && d.warnings.is_empty(), "{view:?}: {d:?}");
        // the foot's motion is unknown: its tangent columns are free, and moved by the solve —
        // the foot slides along the axis as the contact runs along the line
        let moved = sk.duals.iter().flat_map(|d| d.tangent.values()).any(|&t| {
            let p = &sk.params[t as usize];
            !p.fixed && p.value.abs() > 1e-6
        });
        assert!(moved, "{view:?}: the foot holds still");
        // and drawn in a view, its lift's row moves with it
        let lifted = sk.constraints.iter().any(|c| c.intrinsic && c.along.is_some());
        assert_eq!(lifted, !view.is_empty(), "{view:?}");
        let said = described(&e);
        assert!(said.iter().all(|s| s == "l tangent foot"), "{said:?}");
    }
}

/// Two cones tangent at a point, written as one word: their tangent planes there are one, two
/// conditions — the hypoid's pitch cones, which the shipped example states this way and
/// `spatial_surfaces.rs` holds to the fold construction.  The chart its two directions are
/// gauged in rises along the pitch plane's normal, and every row's Jacobian is exact.
#[test]
fn two_sets_tangent_at_a_point() {
    let src = include_str!("../../examples/hypoid_pitch_cones.sv");
    assert!(src.contains("\ngc tangent(at: M) pc\n"), "the example states the word");
    let e = read(src);
    assert!(described(&e).contains(&"gc tangent(at: M) pc".to_string()), "{:?}", described(&e));
    let mut sk = solved(&e);
    let d = diagnose(&mut sk, DiagnoseOptions::default());
    assert_eq!(d.dof, 0, "{d:?}");
    assert!(d.over.is_empty() && d.warnings.is_empty(), "{d:?}");
    // two directions, each gauged along y, the front's normal
    let charts: Vec<_> = sk.duals.iter().map(|d| d.toward).collect();
    assert_eq!(
        charts,
        [0, 1].map(|k| gcs_core::model::Toward::Chart { k, axis: Some(1) }).to_vec(),
    );
    // the two cones' normals at M agree: the pinion's apex comes out on the pitch plane
    let (m, apex) = (at(&sk, &e, "M"), ends(&sk, ent(&e, "pax")).0);
    assert!(sub(apex, m)[1].abs() < 1e-9, "{apex:?}");
    fd_jacobian(&e.sketch, 1e-5);
    fd_jacobian(&sk, 1e-5);
    // and a document carries it
    let back = io::from_json(&io::to_json(&sk)).expect("reads back");
    assert_eq!(io::dumps(&back, None), io::dumps(&sk, None));
}

/// `std.TangentCones` is retired (#265): its whole body was `k1 tangent(at: m) k2`, the word
/// `two_sets_tangent_at_a_point` reads, and the library no longer has the name.
#[test]
fn the_retired_tangent_cones_is_no_name_of_the_library() {
    let src = include_str!("../../examples/hypoid_pitch_cones.sv")
        .replace("\ngc tangent(at: M) pc\n", "\nstd.TangentCones(gc, pc, M)\n");
    refused(&src, "E103", "`std` defines no component `TangentCones`", "std.TangentCones");
}

/// A tangency's derivative rows deleted, its tangent unknowns are no freedom: each is held again.
#[test]
fn a_tangencys_unknowns_go_with_its_rows() {
    let mut sk = solved(&read(&with(&format!("{}l tangent foot", footed("")))));
    let rows: Vec<u32> = sk.constraints.iter().filter(|c| c.along.is_some() && !c.intrinsic)
        .map(|c| c.id).collect();
    let free = |sk: &Sketch| sk.duals.iter().flat_map(|d| d.tangent.values())
        .filter(|&&t| !sk.params[t as usize].fixed).count();
    assert!(free(&sk) > 0);
    for id in rows {
        sk.remove(id);
    }
    assert_eq!(free(&sk), 0);
}

/// A tangency is one condition and a regular one: the diagnosis sees no redundancy, no shaky
/// motion, and the numeric rank agrees with the structural.
#[test]
fn a_tangency_adds_one_regular_condition() {
    for set in ["ball", "rod", "kc"] {
        let mut sk = solved(&read(&with(&format!("l tangent {set}"))));
        let d = diagnose(&mut sk, DiagnoseOptions::default());
        assert!(d.over.is_empty() && d.implied.is_empty(), "{set}: {d:?}");
        assert!(d.warnings.is_empty(), "{set}: {:?}", d.warnings);
    }
}

/// A derivative row's Jacobian is its residual's derivative: every column of the assembled
/// system, at a solution and away from one — the row's own Hessian along the motion, the
/// tangent columns and the line's ends, read exactly from the row's Taylor form.
#[test]
fn the_derivatives_jacobian_is_its_derivative() {
    let footed: Vec<String> =
        ["", " in std.front"].iter().map(|v| format!("{}l tangent foot", footed(v))).collect();
    let sets = ["l tangent ball", "l tangent rod", "l tangent kc"];
    for src in sets.iter().map(|s| s.to_string()).chain(footed) {
        let e = read(&with(&src));
        assert!(e.sketch.constraints.iter().any(|c| c.along.is_some()), "{src}");
        fd_jacobian(&e.sketch, 1e-5);
        fd_jacobian(&solved(&e), 1e-5);
    }
}

/// A derivative row travels through a document and a graft with its use's derivative.
#[test]
fn a_tangency_travels_through_a_document() {
    for src in ["l tangent rod".to_string(), format!("{}l tangent foot", footed(" in std.front"))] {
        let sk = solved(&read(&with(&src)));
        let along = |s: &Sketch| s.constraints.iter().filter_map(|c| c.along).collect::<Vec<_>>();
        assert!(!along(&sk).is_empty());
        let copy = io::copy(&sk, &sk.primitives());
        for mut back in [io::from_json(&io::to_json(&sk)).expect("reads back"), copy] {
            assert_eq!(along(&back), along(&sk), "{src}");
            assert_eq!(back.duals.len(), sk.duals.len(), "{src}");
            assert_eq!(io::dumps(&back, None), io::dumps(&sk, None), "{src}");
            // and what came back is the tangency still
            let r = solve(&mut back, SolveOpts::default());
            assert!(r.success, "{src}: {}", r.message);
            assert_eq!(dof_of(&mut back), dof_of(&mut sk.clone()), "{src}");
        }
    }
}

fn dof_of(sk: &mut Sketch) -> i64 {
    diagnose(sk, DiagnoseOptions::default()).dof
}

#[test]
fn what_a_set_refuses() {
    refused(&with("radius(5) ball"), "E040", "`ball` is a set", "ball");
    refused(&with("ball coincident rod"), "E040", "or to another set at a point", "ball coincident rod");
    // two sets touch at a point the word names, stated and not claimed, and each body is read at
    // the point, so neither may make geometry of its own
    refused(&with("k tangent kc"), "E040", "which the word names", "tangent");
    // a tangency at a point reads where it stands in space, so not a row over its place in a
    // view: a level along the front's own `up` has no kind in space to read it in
    refused(&with("shelf := { q | c distance(5, along: up) q }\nm := point hint((3, 35)) in \
                   std.front\nshelf tangent(at: m) k"), "E040", "reads it in its view",
            "tangent(at: m)");
    refused(&with("claim k tangent(at: a) kc"), "E040", "stated, not claimed", "tangent");
    refused(
        &with(&format!("{}k tangent(at: a) foot", footed(""))),
        "E040",
        "`k tangent(at: a) foot` reads each set's body at the point, where `foot`'s makes `q`",
        "tangent(at: a)",
    );
    refused(&with("l coincident ball"), "E040", "puts a point on it, not a line", "coincident");
    refused(
        &with("kk := circle(center: a) hint(r: 3) in side\nkk tangent ball"),
        "E040",
        "touched by a line, not a circle",
        "tangent",
    );
    refused(&with("claim l tangent ball"), "E040", "stated, not claimed", "tangent");
    refused(&with("a coincident(5) ball"), "E040", "takes nothing in its parentheses", "coincident(5)");
    refused(&with("loop := { p | p coincident loop }\na coincident loop"), "E003",
        "defined in terms of itself", "a coincident loop");
    // and a set says what its points satisfy
    let (_, errs, _) = gcs_core::library::parse_linked("unit mm\nempty := { p | }\n");
    assert!(errs.iter().any(|e| e.message.contains("says what its points satisfy")), "{errs:?}");
}

