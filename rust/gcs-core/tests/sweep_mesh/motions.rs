//! The motions every case is written from, as Solvent lines appended to a
//! tool. Each declares the motion under a fixed name; the sweep line names
//! it with its interval.
#![allow(dead_code)]

/// A turn about the spindle (world z through the origin).
pub const TURN_SPINDLE: &str = "motion turn(about: spindle)\n";

/// A turn about a vertical axis through (3, 0): the sphere's and cylinder's
/// own axis, so their contact is motion-independent.
pub const TURN_OWN_AXIS: &str = "private point hub hint(x: 3, y: 0)
hub distance(3mm, along: u) std.front
hub distance(0mm, along: v) std.front
private point hub_up hint(x: 3, y: 5)
hub_up distance(3mm, along: u) std.front
hub_up distance(5mm, along: v) std.front
construction centerline line own(hub, hub_up)
motion turn(about: own)
";

/// A turn about the line y = 0 in the plane, through (3, 0) and (4, 0): a
/// horizontal axis the tool tumbles about.
pub const TUMBLE: &str = "private point hub hint(x: 3, y: 0)
hub distance(3mm, along: u) std.front
hub distance(0mm, along: v) std.front
private point hub_out hint(x: 4, y: 0)
hub_out distance(4mm, along: u) std.front
hub_out distance(0mm, along: v) std.front
construction centerline line tumbler(hub, hub_out)
motion turn(about: tumbler)
";

/// A turn about the vertical line through (2.5, 0): offset from the tools.
pub const TURN_OFFSET: &str = "private point a0 hint(x: 2.5, y: 0)
a0 distance(2.5mm, along: u) std.front
a0 distance(0mm, along: v) std.front
private point a1 hint(x: 2.5, y: 5)
a1 distance(2.5mm, along: u) std.front
a1 distance(5mm, along: v) std.front
construction centerline line pivot(a0, a1)
motion turn(about: pivot)
";

/// A translation along the spindle by `advance` per full turn of the parameter.
pub fn slide_z(advance_mm: f64) -> String { format!("motion feed(along: spindle, advance: {advance_mm}mm)\n") }

/// A translation along world x by `advance` per full turn.
pub fn slide_x(advance_mm: f64) -> String {
    format!("private point rail_end hint(x: 10, y: 0)
rail_end distance(10mm, along: u) std.front
rail_end distance(0mm, along: v) std.front
construction centerline line rail(std.origin, rail_end)
motion feed(along: rail, advance: {advance_mm}mm)
")
}

/// The sweep line: `solid swept(tool, under: NAME, from: A, to: B)`.
pub fn swept(motion: &str,from_deg: f64,to_deg: f64) -> String {
    format!("solid swept(tool, under: {motion}, from: {from_deg}deg, to: {to_deg}deg)\n")
}

/// A turn about the line through the page points (u0, v0) and (u1, v1)
/// (page u is world x, page v is world z).
pub fn turn_about(u0: f64,v0: f64,u1: f64,v1: f64) -> String {
    format!("private point h0 hint(x: {u0}, y: {v0})
h0 distance({u0}mm, along: u) std.front
h0 distance({v0}mm, along: v) std.front
private point h1 hint(x: {u1}, y: {v1})
h1 distance({u1}mm, along: u) std.front
h1 distance({v1}mm, along: v) std.front
construction centerline line hinge(h0, h1)
motion turn(about: hinge)
")
}
