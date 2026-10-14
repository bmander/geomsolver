//! **What a crown written as regions needs of the core** (#209, #145 step 4b): `std.ConeThrough`,
//! the cone about a line through a point, its radius growing `tan(lean)` along the line, against
//! its closed form, its apex the instance's (a family's body split by its point, §6.21, F3);
//! `std.HalfSpace`, a half-space along a line; a round at a corner where the region is outside a
//! set (`{ q | q outside inner }`, a distance from it is from `inner`'s surface); the tip land said
//! as a bound between the land's ends, choosing the root as the drawn section's tip bound does; and
//! a crown tooth region — two cones, two levels, its tip rounded — admitted to the generating
//! class and swept through a region blank, built exactly and written within 10 µm.
use gcs_core::brep::geom::Surface;
use gcs_core::brep::{export, props, sweep::Say};
use gcs_core::program::Elaborated;
use gcs_core::solid::export::Tolerance;
use gcs_core::solve::{solve, SolveOpts};
use gcs_core::space::{dot, sub};
use std::f64::consts::PI;
use std::sync::Mutex;

use crate::common::{at, ends, ent, off_line, read, text::solved, unit};

/// An axis up the front plane's `z` through the origin, and a point 20 from it at height 10.
const AXIS: &str = "\
unit mm
use std
in std.front {
  ax := line
  fix((0, 0)) ax.p1
  fix((0, 1)) ax.p2
  q := point
  fix((20, 10)) q
}
";

/// A cone through `q` leaning `lean`, and a point on it held at `x = 30`, `y = 5`.
fn cone_through(lean: &str, more: &str) -> String {
    format!("{AXIS}k := std.ConeThrough(ax, q, lean: {lean})\n\
             a := point hint((30, 5, 10))\nfix(x == 30) a\nfix(y == 5) a\na coincident k\n{more}")
}

/// **The cone about a line through a point** (#145's library): its apex on the line where the
/// generator through `q` meets it, `r_q / tan(lean)` back along the line from `q`'s foot for a
/// positive lean and ahead for a negative one, and every point on it the radius `r_q +
/// tan(lean)·(z − z_q)` from the line: its spine runs the way it opens, so either way its inside
/// is the side nearer the line.
#[test]
fn a_cone_through_a_point_is_its_closed_form() {
    let r_a = 30f64.hypot(5.0);
    for lean in [20f64, -20.0, 35.0] {
        let e = solved(&cone_through(&format!("{lean}deg"), ""));
        let t = lean.to_radians().tan();
        let apex = at(&e, "k.apex");
        let want = 10.0 - 20.0 / t;
        assert!(apex[0].abs() < 1e-9 && apex[1].abs() < 1e-9 && (apex[2] - want).abs() < 1e-9,
                "lean {lean}: apex {apex:?}, the closed form's at z = {want}");
        let z = at(&e, "a")[2];
        let want = 10.0 + (r_a - 20.0) / t;
        assert!((z - want).abs() < 1e-9, "lean {lean}: the point at z = {z}, the closed form {want}");
        // the spine from the apex the way the cone opens
        let (p, q) = ends(&e.sketch, ent(&e, "k.spine"));
        assert!(dot(unit(sub(q, p)), [0.0, 0.0, lean.signum()]) > 1.0 - 1e-12, "lean {lean}");
    }
}

/// **A family's own geometry is made once** (§6.21, F3): what `ConeThrough`'s body makes without
/// reading its point — the apex and the spine, placed by `q` — is the instance's, so a second
/// point put on the cone adds that point and its one row, and no second apex.
#[test]
fn a_cone_throughs_apex_is_made_once() {
    let b = "b := point hint((5, 30, 10))\nfix(x == 5) b\nfix(y == 30) b\nb coincident k\n";
    let one = read(&cone_through("20deg", ""));
    let e = solved(&cone_through("20deg", b));
    assert_eq!(e.sketch.params.len() - one.sketch.params.len(), 3);
    let (r, z) = (5f64.hypot(30.0), at(&e, "b")[2]);
    assert!((z - (10.0 + (r - 20.0) / 20f64.to_radians().tan())).abs() < 1e-9, "{z}");
    assert!(off_line(at(&e, "k.apex"), [0.0; 3], [0.0, 0.0, 1.0]) < 1e-12);
}

/// The crown tooth about a face mill's axis, the vertical through `c` = (20, 16, 0): flanks
/// through the pitch points `qi` and `qo`, a quarter of a module-1 pitch either side of the
/// mean point (20, 0, 0), leaning 20 degrees so the tooth narrows toward its tip; a back 1.5
/// above the pitch plane and its tip 1.25 below it.  `ROUNDED` rounds the tip's edges.
const TOOTH: &str = "\
c := point
fix((20, 16, 0)) c
c_up := point
fix((20, 16, 1)) c_up
construction cut_ax := line(c, c_up)
qi := point
qo := point
fix((20, 0.7853981633974483, 0)) qi
fix((20, -0.7853981633974483, 0)) qo
inner := std.ConeThrough(cut_ax, qi, lean: -20deg)
outer := std.ConeThrough(cut_ax, qo, lean: 20deg)
back := std.HalfSpace(c, cut_ax, d: 1.5mm)
point_ := std.HalfSpace(c, cut_ax, d: -1.25mm)
beyond := { x | x outside inner }
above := { x | x outside point_ }
";

const SHARP: &str = "\
sharp := { p | p outside inner; p inside outer; p inside back; p outside point_ }
sharp_tool := solid(sharp)
";

const ROUNDED: &str = "\
tooth := { p | p outside inner; p inside outer; p inside back; p outside point_
               p inside std.Rounded(beyond, above, r: 0.3mm)
               p inside std.Rounded(outer, above, r: 0.3mm) }
construction tool := solid(tooth)
";

/// The tooth's meridian about the cutter's axis: radius `r` from it, height `z`.  The flanks are
/// `r = 16 ∓ π/4 ∓ z tan 20°` (inner, outer), the tip `z = −1.25`, the back `z = 1.5`.
fn flank(outer: bool, z: f64) -> f64 {
    let s = if outer { 1.0 } else { -1.0 };
    16.0 + s * (PI / 4.0 + z * 20f64.to_radians().tan())
}

/// What a ball of radius `rho` rounding the tip's corner on one flank takes, turned once about
/// the axis (Pappus): the corner between the tip and the flank less the ball's sector,
/// `2π ∫ r dA`.
fn round_takes(outer: bool, rho: f64) -> f64 {
    let s = if outer { -1.0 } else { 1.0 };
    let a = 20f64.to_radians();
    let v = [flank(outer, -1.25), -1.25];
    // along the tip into the tooth, and up the flank
    let (e1, e2) = ([s, 0.0], [-s * a.sin(), a.cos()]);
    let theta = (e1[0] * e2[0] + e1[1] * e2[1]).acos();
    let leg = rho / (theta / 2.0).tan();
    let m = [e1[0] + e2[0], e1[1] + e2[1]];
    let b = [m[0] / m[0].hypot(m[1]), m[1] / m[0].hypot(m[1])];
    let o = [v[0] + b[0] * rho / (theta / 2.0).sin(), v[1] + b[1] * rho / (theta / 2.0).sin()];
    let t1 = [v[0] + e1[0] * leg, v[1] + e1[1] * leg];
    let t2 = [v[0] + e2[0] * leg, v[1] + e2[1] * leg];
    // the kite as two right triangles, each its area times its centroid's radius
    let tri = |p: [f64; 2], q: [f64; 2], r: [f64; 2]| {
        let area = ((q[0] - p[0]) * (r[1] - p[1]) - (r[0] - p[0]) * (q[1] - p[1])).abs() / 2.0;
        area * (p[0] + q[0] + r[0]) / 3.0
    };
    let kite = tri(v, t1, o) + tri(v, o, t2);
    let half = (PI - theta) / 2.0;
    let sector = rho * rho * half * (o[0] - b[0] * 2.0 * rho * half.sin() / (3.0 * half));
    2.0 * PI * (kite - sector)
}

/// **The tooth as a region is its closed form** (#209): two `ConeThrough` flanks and two
/// `Level`s about one line fold into one meridian, the trapezoid turned (`r_o + r_i` is 32 at
/// every height, so its volume is `32π ∫ (r_o − r_i) dz`); rounded at the tip — a round at the
/// corner where the region is outside `inner` reads `{ q | q outside inner }`, whose surface is
/// `inner`'s — it is the trapezoid less the two corners Pappus gives, its faces two cones, two
/// planes and two tori, to 1e-9.
#[test]
fn the_crown_tooth_region_is_its_closed_form() {
    let e = solved(&format!("unit mm\nuse std\n{TOOTH}{SHARP}{ROUNDED}"));
    let volume = |name: &str| {
        let ex = e.sketch.exact_solid(ent(&e, name).i()).unwrap();
        (props::volume(&ex.brep) / ex.mm.powi(3), ex)
    };
    let t = 20f64.to_radians().tan();
    let trapezoid = 32.0 * PI * (PI / 2.0 * 2.75 + t * (1.5f64.powi(2) - 1.25f64.powi(2)));
    let (sharp, _) = volume("sharp_tool");
    assert!((sharp - trapezoid).abs() <= 1e-9 * trapezoid, "{sharp} against {trapezoid}");
    let (rounded, ex) = volume("tool");
    let want = trapezoid - round_takes(false, 0.3) - round_takes(true, 0.3);
    assert!((rounded - want).abs() <= 1e-9 * want, "{rounded} against {want}");
    let count = |f: fn(&Surface) -> bool| ex.brep.faces.iter().filter(|x| f(&x.surface)).count();
    assert_eq!(count(|s| matches!(s, Surface::Cone(..))), 2);
    assert_eq!(count(|s| matches!(s, Surface::Plane(..))), 2);
    assert_eq!(count(|s| matches!(s, Surface::Torus(..))), 2);
}

/// A crown tooth's section given by its land, as a face mill's blade is by its point width: the
/// cutter's axis up the front page (`x` the radius from it), the land's outer end `eb` on the
/// tip, its inner end `ea` 0.6 from it along the tip, the flanks through the two leaning 20
/// degrees so the tooth widens toward its back.  `ea` is seeded beyond `eb`: a rough start on
/// the root where the flanks cross above the tip, so the tooth comes to an edge and its tip has
/// no land.  `LAND` is what says it has one.
const SECTION: &str = "\
unit mm
use std
in std.front {
  ax := line
  fix((0, 0)) ax.p1
  fix((0, 1)) ax.p2
  eb := point
  fix((16.4, -1.25)) eb
  ea := point hint((16.9, -1.25))
}
tip := std.HalfSpace(ax.p1, ax, d: -1.25mm)
back := std.HalfSpace(ax.p1, ax, d: 1.5mm)
ea coincident tip
ea distance(0.6mm) eb
inner := std.ConeThrough(ax, ea, lean: -20deg)
outer := std.ConeThrough(ax, eb, lean: 20deg)
tooth := { p | p outside inner; p inside outer; p inside back; p outside tip }
blade := solid(tooth)
";

/// The land: its inner end inward of its outer end, by at least a hundredth of the module.
const LAND: &str = "eb distance(>= 0.01mm, along: left) ea\n";

/// **A region's tip land is a bound between the land's ends** (#209; `docs/fillets`, finding 9):
/// a region draws no tip line for `length(>= …) tip` to stand on, so its section says the land
/// on its two ends, where the flanks meet the tip, and the bound chooses the root as the drawn
/// section's does (§9.6) — from the rough start, the section without it settles where the
/// flanks cross above the tip (the tooth's meridian a triangle, no step on the tip), and with
/// it on the tooth with its land.
#[test]
fn the_land_bound_keeps_a_rough_start_off_the_collapsed_tip() {
    // the meridian's steps: the two flanks, the back and — where it has a land — the tip
    let steps = |e: &Elaborated| {
        let m = gcs_core::solid::region::meridian(&e.sketch, ent(e, "blade").i()).unwrap();
        m.region.loops.iter().flatten().count()
    };
    let free = solved(SECTION);
    assert!((at(&free, "ea")[0] - 17.0).abs() < 1e-9, "{:?}", at(&free, "ea"));
    assert_eq!(steps(&free), 3, "the flanks cross above the tip: no land");
    let landed = solved(&format!("{SECTION}{LAND}"));
    assert!((at(&landed, "ea")[0] - 15.8).abs() < 1e-9, "{:?}", at(&landed, "ea"));
    assert_eq!(steps(&landed), 4);
    use gcs_core::diagnose::{diagnose, DiagnoseOptions};
    let d = diagnose(&mut landed.sketch.clone(), DiagnoseOptions::default());
    assert!(d.bounds_violated.is_empty() && d.dof == 0, "{d:?}");
}

/// The member a crown generates: a 45-degree pitch cone about `gear_ax` from the apex at the
/// origin, touching the pitch plane `z = 0` along `x`; its blank a region within the tip cone (the
/// pitch cone grown by the addendum, 1) and 3 inside the pitch cone, between spheres 17 and 23
/// about the apex.  The crown turns about `z` and the member at the crown's `1 / sin 45°`.
const BEVEL: &str = "\
unit mm
use std
in std.front {
  o := point
  fix((0, 0)) o
  ge := point
  fix((7.0710678118654755, -7.0710678118654755)) ge
  construction gear_ax := line(o, ge)
  ze := point
  fix((0, 1)) ze
  construction crown_ax := line(o, ze)
}
pitch := std.Cone(gear_ax, half: 45deg)
tip := std.Offset(pitch, d: 1mm)
toe := std.Sphere(o, r: 17mm)
heel := std.Sphere(o, r: 23mm)
blank_region := { p | p inside tip; p inside heel; p outside toe
  p distance(<= 3mm, side: inside) pitch }
blank := solid(blank_region)
";

const GENERATION: &str = "\
crown_roll := motion(about: crown_ax)
gear_roll := motion(about: gear_ax, ratio: -1.4142135623730951)
generation := motion(crown_roll, relative_to: gear_roll)
construction removal := solid(tool, under: generation, from: -40deg, to: 40deg)
part := solid(blank)
removal cut part
";

/// **A region tool is admitted and swept against a region blank** (#209): the crown tooth's
/// faces are its meridian's steps turned (`RevolvedSurface::meridian_loops`), read as a
/// revolution's are by admission's rows and by the cutter's sections, so the generating class
/// takes it; its contacts reach the blank on both flanks, both rounds and the land, the sheet is
/// fitted, the blank split by it, and the part written as STEP and as STL within 10 µm of its
/// exact surface, the STL measured by the accuracy meter (`solventc --measure`).
#[test]
fn a_crown_tooth_region_cuts_a_region_blank_exactly() {
    use gcs_core::solid::accuracy::{self, Meter, Options};
    use gcs_core::solid::admission;
    let e = solved(&format!("{BEVEL}{TOOTH}{ROUNDED}{GENERATION}"));
    let body = ent(&e, "part").i();
    let admitted = admission::admit_body(&e.sketch, body, &admission::Options::default())
        .unwrap_or_else(|r| panic!("refused: {r}"));
    assert!(admitted.sweeps()[0].contacts > 100);
    let tolerance = Some(Tolerance::new(0.01).unwrap());
    let lines = Mutex::new(Vec::<String>::new());
    let say = Say { stage: &|l: &str| lines.lock().unwrap().push(l.to_string()), mark: &|_| {} };
    let exact = export::exact(&e.sketch, body, Some(&admitted), tolerance, &say)
        .unwrap_or_else(|r| panic!("refused at {:?}: {}", r.stage, r.message));
    exact.solid.check(1e-6).unwrap();
    let step = export::step(&exact, "part", tolerance, &say).unwrap();
    assert!(step.starts_with("ISO-10303-21;"));
    let stl = export::stl(&e.sketch, body, &exact, tolerance, &say).unwrap();
    let said = lines.lock().unwrap().join("\n");
    assert!(said.contains("5 profile faces"), "{said}");
    assert!(said.contains(", 0 disagree"), "{said}");
    let left = props::volume(&exact.solid);
    let blank = {
        let ex = e.sketch.exact_solid(ent(&e, "blank").i()).unwrap();
        props::volume(&ex.brep) / ex.mm.powi(3)
    };
    assert!(left < blank - 1.0, "{left} of {blank}: the tooth space is cut");
    let meter = Meter::read(&e.sketch, body, Options::in_units(1.0)).unwrap();
    let (vertices, triangles) = gcs_core::solid::agreement::stl_triangles(&stl, 1.0).unwrap();
    let samples = accuracy::mesh_samples(&vertices, &triangles, 4000);
    let measurements: Vec<_> = samples.iter().map(|s| meter.measure(s.position)).collect();
    let (ok, line) = accuracy::within(&meter, &samples, &measurements, 0.01, 1.0);
    assert!(ok, "{line}");
}

// -- what the hypoid's crown as regions needed (#214) --------------------------------------------

/// The radius of `cone_through(20deg)`'s cone at height `z`: `20 + tan(20°)(z − 10)`.
fn radius_at(z: f64) -> f64 { 20.0 + 20f64.to_radians().tan() * (z - 10.0) }

/// **A side's surface is a set a drawn point is put on** (#214): `m coincident side.surface`, the
/// side `{ x | x distance(<= 0.5mm, side: inside) k }` of a cone through `q`, puts `m` on the cone
/// moved half a millimetre into it along its normal — the radius at its height less `0.5 /
/// cos(lean)` — as a topping's kink is put on a crown's flank as grown; and a set formal's
/// `.surface` is the same.
#[test]
fn a_point_is_put_on_a_sides_surface() {
    let e = solved(&format!("{AXIS}k := std.ConeThrough(ax, q, lean: 20deg)\n\
        side := {{ x | x distance(<= 0.5mm, side: inside) k }}\n\
        in std.front {{\n  m := point hint((15, -2))\n}}\nm coincident side.surface\n\
        ax.p1 distance(-2mm, along: ax) m\n\
        component On(f: set, at: point) {{\n  n := point hint((15, 0, -4))\n  n coincident f.surface\n  \
        fix(y == 0) n\n  at distance(-4mm, along: ax) n\n}}\non := On(side, ax.p1)\n"));
    let want = |z: f64| radius_at(z) - 0.5 / 20f64.to_radians().cos();
    let m = at(&e, "m");
    assert!((m[0] - want(-2.0)).abs() < 1e-9 && (m[2] + 2.0).abs() < 1e-9, "{m:?} against {}", want(-2.0));
    let n = at(&e, "on.n");
    assert!((n[0] - want(-4.0)).abs() < 1e-9, "{n:?} against {}", want(-4.0));
}

/// **A distance that comes to nothing** (#214): a set `std.Offset(S, d: 0mm)` is `S` — a point put
/// on it is on `S` (`coincident`, where a drawn point's distance would have nowhere to put its
/// foot), and a region within it is the region within `S`, as an allowance of none is; a zero
/// written as the distance is still refused, naming `coincident`.
#[test]
fn an_offset_of_nothing_is_the_set() {
    let src = format!("{AXIS}k := std.ConeThrough(ax, q, lean: 20deg)\nnone := std.Offset(k, d: 0mm)\n\
        in std.front {{\n  m := point hint((18, 4))\n}}\nm coincident none\nax.p1 distance(4mm, along: ax) m\n\
        lid := std.HalfSpace(ax.p1, ax, d: 6mm)\nfloor := {{ x | x outside std.HalfSpace(ax.p1, ax, d: -6mm) }}\n\
        grown := {{ p | p inside none; p inside lid; p inside floor }}\n\
        plain := {{ p | p inside k; p inside lid; p inside floor }}\n\
        a := solid(grown)\nb := solid(plain)\n");
    let e = solved(&src);
    let m = at(&e, "m");
    assert!((m[0] - radius_at(4.0)).abs() < 1e-9, "{m:?} against {}", radius_at(4.0));
    let volume = |name: &str| {
        let ex = e.sketch.exact_solid(ent(&e, name).i()).unwrap();
        props::volume(&ex.brep) / ex.mm.powi(3)
    };
    assert!((volume("a") - volume("b")).abs() <= 1e-12 * volume("b"), "{} against {}", volume("a"), volume("b"));
    let written = format!("{AXIS}k := std.ConeThrough(ax, q, lean: 20deg)\n\
        in std.front {{\n  m := point hint((18, 4))\n}}\nm distance(0mm, side: outside) k\n");
    let (p, _, _) = gcs_core::library::parse_linked(&written);
    let refused = gcs_core::program::elaborate(&p);
    assert!(refused.diags.iter().any(|d| d.message.contains("coincident")), "{:?}", refused.diags);
}

/// **A round's radius may read an unknown of the drawing** (#214), as the hypoid's reads the normal
/// module the trace constructs: the tooth rounded by `0.3 * m`, `m` an unknown a drawn length
/// holds at 1, is the tooth rounded by 0.3 to the closed form; and grown across its flanks by
/// `0.05 * m` it is the tooth grown by 0.05mm, the growth added to the round's centres as a number
/// (where a text over the unknown would read it as a plain number and refuse to add it).
#[test]
fn a_rounds_radius_reads_an_unknown() {
    let unknown = "param m: Length hint(2mm)\nin std.front {\n  u0 := point\n  u1 := point\n  \
        fix((40, 0)) u0\n  fix((41, 0)) u1\n}\nu0 distance(m) u1\n";
    let t = 20f64.to_radians().tan();
    let trapezoid = 32.0 * PI * (PI / 2.0 * 2.75 + t * (1.5f64.powi(2) - 1.25f64.powi(2)));
    let want = trapezoid - round_takes(false, 0.3) - round_takes(true, 0.3);
    // a multiple of it, and one with a length beside it (the measured bound the round's centres
    // are read through reads the unknown as declared)
    let mut volume = 0.0;
    for r in ["0.3 * m", "0.25 * m + 0.05mm"] {
        let rounded = ROUNDED.replace("r: 0.3mm", &format!("r: {r}"));
        let e = solved(&format!("unit mm\nuse std\n{unknown}{TOOTH}{rounded}"));
        let ex = e.sketch.exact_solid(ent(&e, "tool").i()).unwrap();
        volume = props::volume(&ex.brep) / ex.mm.powi(3);
        assert!((volume - want).abs() <= 1e-9 * want, "{r}: {volume} against {want}");
    }
    let grown = |g: &str| {
        let tooth = format!("beyond_g := {{ x | x distance(<= {g}, side: inside) inner }}\n\
            outer_g := std.Offset(outer, d: {g})\n\
            tooth := {{ p | p inside beyond_g; p inside outer_g; p inside back; p outside point_\n\
            p inside std.Rounded(beyond_g, above, r: 0.3 * m)\n\
            p inside std.Rounded(outer_g, above, r: 0.3 * m) }}\nconstruction tool := solid(tooth)\n");
        let e = solved(&format!("unit mm\nuse std\n{unknown}{TOOTH}{tooth}"));
        let ex = e.sketch.exact_solid(ent(&e, "tool").i()).unwrap();
        props::volume(&ex.brep) / ex.mm.powi(3)
    };
    let (by_unknown, by_number) = (grown("0.05 * m"), grown("0.05mm"));
    assert!(by_unknown > volume + 0.1, "the grown tooth {by_unknown} is the tooth's {volume} and more");
    assert!((by_unknown - by_number).abs() <= 1e-12 * by_number, "{by_unknown} against {by_number}");
}

/// **A seed's step from a point is taken in space** (#214): a point in space seeded up a line
/// square to the plane its `at:` point is drawn in starts that far off the plane — read in the
/// plane, the step would go nowhere and the cutter's axis would start with no length.
#[test]
fn a_seeds_step_is_taken_in_space() {
    let e = read("unit mm\nuse std\nin std.top {\n  c := point\n  fix((3, 4)) c\n}\n\
        in std.front {\n  up := line\n  fix((0, 0)) up.p1\n  fix((0, 10)) up.p2\n}\n\
        p := point hint(at: c, along: up, by: 0.5)\n");
    let p = at(&e, "p");
    assert!((p[0] - 3.0).abs() < 1e-12 && (p[1] - 4.0).abs() < 1e-12 && (p[2] - 5.0).abs() < 1e-12, "{p:?}");
}

/// **A cone through a point is held at any size** (#214): its apex by the angle between its line
/// and the generator through the point, its spine to the point's foot, nothing a fixed length.  At
/// a thousand times `cone_through`'s size and a lean of 7.5 degrees — the apex 7.6 radii up the
/// axis — every start jittered off the solution by up to a hundredth of the drawing settles back on
/// the closed form, solved as an editor solves it.
#[test]
fn a_cone_through_is_held_at_any_size() {
    let big = "unit mm\nuse std\nin std.front {\n  ax := line\n  fix((0, 0)) ax.p1\n  fix((0, 1000)) ax.p2\n  \
        q := point\n  fix((20000, 10000)) q\n}\nk := std.ConeThrough(ax, q, lean: -7.5deg)\n";
    let e = solved(big);
    let want = 10000.0 + 20000.0 / 7.5f64.to_radians().tan();
    assert!((at(&e, "k.apex")[2] - want).abs() < 1e-9 * want, "{:?} against {want}", at(&e, "k.apex"));
    for seed in 0..16 {
        let mut sk = crate::common::jittered(&e.sketch, 0.01, seed);
        let r = solve(&mut sk, SolveOpts::default());
        let apex = sk.world_point(ent(&e, "k.apex").i());
        assert!(r.success && (apex[2] - want).abs() < 1e-9 * want, "seed {seed}: {r:?}, apex {apex:?}");
    }
}

/// **A surface run the other way** (`RevolvedSurface::reversed`, #214): each face of the tooth's
/// meridian, reversed, is where it was at `1 − u`, with its `u` derivative turned.
#[test]
fn a_reversed_surface_is_the_surface_run_the_other_way() {
    use gcs_core::solid::RevolvedSurface;
    let e = solved(&format!("unit mm\nuse std\n{TOOTH}{ROUNDED}"));
    let m = gcs_core::solid::region::meridian(&e.sketch, ent(&e, "tool").i()).unwrap();
    let faces: Vec<RevolvedSurface> = RevolvedSurface::meridian_loops(&m, "tool").into_iter().flatten().flatten().collect();
    assert_eq!(faces.len(), 6);
    for f in &faces {
        let r = f.reversed();
        for u in [0.0, 0.3, 0.7, 1.0] { for v in [0.0, 0.4] {
            let (a, b) = (f.at(u, v).unwrap(), r.at(1.0 - u, v).unwrap());
            for k in 0..3 {
                assert!((a.position[k] - b.position[k]).abs() < 1e-12, "{}: {u} {v}", f.name);
                assert!((a.du[k] + b.du[k]).abs() < 1e-9, "{}: {u} {v}", f.name);
            }
        }}
    }
}
