//! The motions every case is written from, as Solvent lines appended to a
//! tool. Each declares the motion under a fixed name; the sweep line names
//! it with its interval.

/// A turn about the spindle (world z through the origin).
pub const TURN_SPINDLE: &str = "turn := motion(about: spindle)\n";

/// A turn about a vertical axis through (3, 0): the sphere's and cylinder's
/// own axis, so their contact is motion-independent.
pub const TURN_OWN_AXIS: &str = "\
use std
in std.front {
private hub := point hint(x: 3, y: 0)
hub distance(3mm, along: u) std.front
hub distance(0mm, along: v) std.front
private hub_up := point hint(x: 3, y: 5)
hub_up distance(3mm, along: u) std.front
hub_up distance(5mm, along: v) std.front
construction centerline own := line(hub, hub_up)
}
turn := motion(about: own)
";

/// A turn about the line y = 0 in the plane, through (3, 0) and (4, 0): a
/// horizontal axis the tool tumbles about.
pub const TUMBLE: &str = "\
use std
in std.front {
private hub := point hint(x: 3, y: 0)
hub distance(3mm, along: u) std.front
hub distance(0mm, along: v) std.front
private hub_out := point hint(x: 4, y: 0)
hub_out distance(4mm, along: u) std.front
hub_out distance(0mm, along: v) std.front
construction centerline tumbler := line(hub, hub_out)
}
turn := motion(about: tumbler)
";

/// A turn about the vertical line through (2.5, 0): offset from the tools.
pub const TURN_OFFSET: &str = "\
use std
in std.front {
private a0 := point hint(x: 2.5, y: 0)
a0 distance(2.5mm, along: u) std.front
a0 distance(0mm, along: v) std.front
private a1 := point hint(x: 2.5, y: 5)
a1 distance(2.5mm, along: u) std.front
a1 distance(5mm, along: v) std.front
construction centerline pivot := line(a0, a1)
}
turn := motion(about: pivot)
";

/// A translation along the spindle by `advance` per full turn of the parameter.
pub fn slide_z(advance_mm: f64) -> String { format!("feed := motion(along: spindle, advance: {advance_mm}mm)\n") }

/// A translation along world x by `advance` per full turn.
pub fn slide_x(advance_mm: f64) -> String {
    format!("\
use std
in std.front {{
private rail_end := point hint(x: 10, y: 0)
rail_end distance(10mm, along: u) std.front
rail_end distance(0mm, along: v) std.front
construction centerline rail := line(std.origin, rail_end)
}}
feed := motion(along: rail, advance: {advance_mm}mm)
")
}

/// The sweep line: `swept := solid(tool, under: NAME, from: A, to: B)`.
pub fn swept(motion: &str,from_deg: f64,to_deg: f64) -> String {
    format!("swept := solid(tool, under: {motion}, from: {from_deg}deg, to: {to_deg}deg)\n")
}

/// A turn about the line through the page points (u0, v0) and (u1, v1)
/// (page u is world x, page v is world z).
pub fn turn_about(u0: f64,v0: f64,u1: f64,v1: f64) -> String {
    format!("\
use std
in std.front {{
private h0 := point hint(x: {u0}, y: {v0})
h0 distance({u0}mm, along: u) std.front
h0 distance({v0}mm, along: v) std.front
private h1 := point hint(x: {u1}, y: {v1})
h1 distance({u1}mm, along: u) std.front
h1 distance({v1}mm, along: v) std.front
construction centerline hinge := line(h0, h1)
}}
turn := motion(about: hinge)
")
}

/// A roll: the tool turned about the vertical through (3, 0) at `ratio` turns per turn of an
/// observer turning about the spindle, so the two are near enough to beat against each other and
/// a contact loop flickers open and closed.
pub fn roll(ratio: f64) -> String {
    format!("\
use std
in std.front {{
private hub := point hint(x: 3, y: 0)
hub distance(3mm, along: u) std.front
hub distance(0mm, along: v) std.front
private hub_up := point hint(x: 3, y: 5)
hub_up distance(3mm, along: u) std.front
hub_up distance(5mm, along: v) std.front
construction centerline own := line(hub, hub_up)
}}
private spin := motion(about: own, ratio: {ratio})
private observer := motion(about: spindle)
turn := motion(spin, relative_to: observer)
")
}

/// The tool spinning about its own vertical axis through (3, 0), seen from an observer turning
/// about world x: crossed axes, as a generator's are.
pub const CROSSED_ROLL: &str = "\
use std
in std.front {
private hub := point hint(x: 3, y: 0)
hub distance(3mm, along: u) std.front
hub distance(0mm, along: v) std.front
private hub_up := point hint(x: 3, y: 5)
hub_up distance(3mm, along: u) std.front
hub_up distance(5mm, along: v) std.front
construction centerline own := line(hub, hub_up)
private xend := point hint(x: 5, y: 0)
xend distance(5mm, along: u) std.front
xend distance(0mm, along: v) std.front
construction centerline xaxis := line(std.origin, xend)
}
private spin := motion(about: own, ratio: 0.25)
private observer := motion(about: xaxis)
turn := motion(spin, relative_to: observer)
";

/// Which axis a cradle roll's observer turns about.
#[derive(Clone,Copy,PartialEq,Debug)]
pub enum Observer {
    /// The spindle, world z: parallel to the tool's axis.
    Parallel,
    /// World x: meeting the tool's axis, crossed.
    Crossed,
    /// Parallel to world x through (0, 0.5, 0.5): skew to the tool's axis, as a generator's
    /// cutter axis is to the member's, and off a torus's mid-plane, since an axis in that plane
    /// leaves the equators in contact at every time.
    Skew,
}

/// A generator's roll: the tool (whose own axis is the vertical line through (3, 0)) carried
/// about a vertical cradle axis through (2, 0) at `ratio` turns a turn, seen from an observer
/// turning about another axis. A spin about the tool's own axis would change nothing of a
/// revolution, leaving a single rotation.
pub fn cradle_roll(ratio: f64,observer: Observer) -> String {
    let (axis,about) = match observer {
        Observer::Parallel => ("","spindle"),
        Observer::Crossed => ("\
use std
in std.front {
private xend := point hint(x: 5, y: 0)
xend distance(5mm, along: u) std.front
xend distance(0mm, along: v) std.front
construction centerline xaxis := line(std.origin, xend)
}
","xaxis"),
        Observer::Skew => ("\
use std
private rise := ray hint(x: 0, y: 1, z: 1)
fix(x == 0, y == sqrt(0.5), z == sqrt(0.5)) rise
private flat := plane(u: std.x, v: rise)
fix(x == 0, y == 0, z == 0) flat
in flat {
  private k0 := point hint(x: 0, y: 0.7071)
  private k1 := point hint(x: 5, y: 0.7071)
  k0 distance(0mm, along: u) flat
  k0 distance(0.7071mm, along: v) flat
  k1 distance(5mm, along: u) flat
  k1 distance(0.7071mm, along: v) flat
  construction centerline kaxis := line(k0, k1)
}
","kaxis"),
    };
    format!("\
use std
in std.front {{
private hub := point hint(x: 2, y: 0)
hub distance(2mm, along: u) std.front
hub distance(0mm, along: v) std.front
private hub_up := point hint(x: 2, y: 5)
hub_up distance(2mm, along: u) std.front
hub_up distance(5mm, along: v) std.front
construction centerline cradle := line(hub, hub_up)
}}
{axis}private spin := motion(about: cradle, ratio: {ratio})
private observer := motion(about: {about})
turn := motion(spin, relative_to: observer)
")
}
