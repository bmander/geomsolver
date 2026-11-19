//! **A body grown, shrunk or shelled whole by the kernel's offset** (docs/contract/rounds.md):
//! where neither construction of `solid::grown` nor of `solid::shelled` builds it — a face on a
//! cone, a sphere or a torus, a body rounded as well, a face left open that is no plane — the
//! body is moved by `brep::offset` (`grown`, `shrunk`, or `cavity`, the body less its wall), the
//! recipe's `offset` node, held to the region's field.  brep-bench's cases, to closed forms.
use gcs_core::brep::props::volume;
use std::f64::consts::PI;

use crate::common::{ent, text::solved};

fn close(what: &str, a: f64, b: f64) { assert!((a - b).abs() <= 1e-9 * b.abs().max(1.0), "{what}: {a} against {b}"); }

/// `part`'s exact boundary, and its volume held to `want`.
fn part_volume(text: &str, want: f64) {
    let e = solved(text);
    let x = e.sketch.exact_solid(ent(&e, "part").i()).unwrap_or_else(|m| panic!("{m}"));
    close("volume", volume(&x.brep), want);
}

/// A cone of radius 3 and height 4 grown by 1: Steiner's formula, its apex capped by a ball, whose field
/// reads past the apex by its pole.
#[test]
fn a_cone_grown_caps_its_apex_with_a_ball() {
    part_volume("unit mm\n\
use std\n\
use std.solids\n\
use std.rounds\n\
\n\
in std.front {\n\
  o := point\n\
  rim := point hint((3, 0))\n\
  apex := point hint((0, 4))\n\
  half := horizontal line(o, rim) -> line(rim, apex) -> vertical line(apex, o) -> close\n\
  fix((0, 0)) o\n\
  o distance(3) rim\n\
  o distance(4) apex\n\
}\n\
\n\
cone := solid(std.solids.Revolution(half, about: std.z))\n\
grown := std.rounds.Grown(cone, d: 1mm)\n\
part := solid(grown.body)\n", PI * 9.0 * 4.0 / 3.0 + PI * (9.0 + 15.0) + PI * (4.0 + 3.0 * (PI - (4f64 / 3.0).atan())) + 4.0 * PI / 3.0);
}

/// A torus of tube radius 3 about a circle of radius 10, shrunk by 1.5: the tube of radius 1.5.
#[test]
fn a_torus_shrunk_is_a_thinner_torus() {
    part_volume("unit mm\n\
use std\n\
use std.solids\n\
use std.rounds\n\
\n\
in std.front {\n\
  c := point\n\
  fix((10, 0)) c\n\
  tube := circle(center: c) hint(r: 3)\n\
  radius(3) tube\n\
}\n\
\n\
torus := solid(std.solids.Revolution(face(tube), about: std.z))\n\
shrunk := std.rounds.Grown(torus, d: -1.5mm)\n\
part := solid(shrunk.body)\n", 2.0 * PI * PI * 10.0 * 1.5 * 1.5);
}

/// An L-shaped plate 8 thick shrunk by 1: its reflex edge rounded by the reach, the rest moved in.
#[test]
fn an_l_shrunk_rounds_its_reflex_edge() {
    part_volume("unit mm\n\
use std\n\
use std.solids\n\
use std.rounds\n\
\n\
in std.top {\n\
  a := point\n\
  b := point hint((20, 0))\n\
  c := point hint((20, 5))\n\
  d := point hint((5, 5))\n\
  e := point hint((5, 15))\n\
  f := point hint((0, 15))\n\
  outline := horizontal line(a, b) -> vertical line(b, c) -> horizontal line(c, d) ->\n\
             vertical line(d, e) -> horizontal line(e, f) -> vertical line(f, a) -> close\n\
  fix((0, 0)) a\n\
  a distance(20) b\n\
  b distance(5) c\n\
  c distance(15) d\n\
  f distance(15) a\n\
}\n\
\n\
plate := solid(std.solids.Prism(outline, from: 0mm, to: 8mm))\n\
shrunk := std.rounds.Grown(plate, d: -1mm)\n\
part := solid(shrunk.body)\n", 6.0 * (150.0 - 70.0 + 5.0 - PI / 4.0));
}

/// A ball of radius 5 shelled shut, 1 thick: its cavity the ball of radius 4.
#[test]
fn a_ball_shelled_shut_is_two_spheres() {
    part_volume("unit mm\n\
use std\n\
use std.sets\n\
use std.rounds (shell)\n\
\n\
o := point\n\
fix((0, 0, 0)) o\n\
\n\
part := solid(std.sets.Ball(o, r: 5mm))\n\
shell(t: 1mm) part\n", 4.0 * PI * (125.0 - 64.0) / 3.0);
}

/// A half ball of radius 5 shelled open at its flat, 1 thick.
#[test]
fn a_hemisphere_shelled_open_at_its_flat() {
    part_volume("unit mm\n\
use std\n\
use std.solids\n\
use std.rounds (shell)\n\
\n\
in std.front {\n\
  o := point\n\
  fix((0, 0)) o\n\
  e := point hint((5, 0))\n\
  s := point hint((0, -5))\n\
  horizontal (flat := line(o, e))\n\
  (bowl := arc(center: o, from: s, to: e))\n\
  vertical (axle := line(s, o))\n\
  radius(5mm) bowl\n\
}\n\
\n\
part := solid(std.solids.Revolution(face(flat, bowl, axle), about: std.z))\n\
part.flat shell(t: 1mm) part\n", 2.0 * PI * (125.0 - 64.0) / 3.0);
}
