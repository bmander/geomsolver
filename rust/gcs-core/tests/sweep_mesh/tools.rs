//! The tools every case is written from, as Solvent. Each declares
//! `construction solid tool(...)` and the spindle `spindle` along world z
//! through the origin; a case appends its motion and its sweep.
#![allow(dead_code)]

/// A unit sphere centred at (3, 0, 0).
pub const SPHERE: &str = "unit mm
use std
construction centerline line spindle(std.origin, std.up.toward)
private point center
center distance(3mm, along: u) std.front
center distance(0mm, along: v) std.front
private point bottom hint(x: 3, y: -1)
private point top hint(x: 3, y: 1)
private line diameter(bottom, top)
center midpoint diameter
diameter parallel spindle
distance(2mm) diameter
private arc meridian(center: center, start: bottom, end: top)
radius(1mm) meridian
construction solid tool(face(meridian, diameter), about: diameter)
";

/// A cylinder of radius 1 and height 2 about the vertical line x = 3, z in [-1, 1].
pub const CYLINDER: &str = "unit mm
use std
construction centerline line spindle(std.origin, std.up.toward)
private point c0 hint(x: 3, y: -1)
private point c1 hint(x: 4, y: -1)
private point c2 hint(x: 4, y: 1)
private point c3 hint(x: 3, y: 1)
ground c0
ground c1
ground c2
ground c3
private line bottom(c0, c1)
private line wall(c1, c2)
private line top(c2, c3)
private line axis(c3, c0)
construction solid tool(face(bottom, wall, top, axis), about: axis)
";

/// A box x in [2, 4], y in [-3, 0], z in [-1, 1] (a 2 x 3 profile extruded 2).
pub const BOX: &str = "unit mm
use std
construction centerline line spindle(std.origin, std.up.toward)
private point b0 hint(x: 2, y: -3)
private point b1 hint(x: 4, y: -3)
private point b2 hint(x: 4, y: 0)
private point b3 hint(x: 2, y: 0)
ground b0
ground b1
ground b2
ground b3
private line e0(b0, b1)
private line e1(b1, b2)
private line e2(b2, b3)
private line e3(b3, b0)
construction solid tool(face(e0, e1, e2, e3), from: -1mm, to: 1mm)
";

/// A triangular prism with corners (3, -0.8), (4.5, 0), (3, 0.8), z in [-1.5, 1.5].
pub const TRIANGLE_PRISM: &str = "unit mm
use std
construction centerline line spindle(std.origin, std.up.toward)
private point t0 hint(x: 3, y: -0.8)
private point t1 hint(x: 4.5, y: 0)
private point t2 hint(x: 3, y: 0.8)
ground t0
ground t1
ground t2
private line e0(t0, t1)
private line e1(t1, t2)
private line e2(t2, t0)
construction solid tool(face(e0, e1, e2), from: -1.5mm, to: 1.5mm)
";

/// A lens: the unit sphere at (3, 0, 0) bounded by the unit sphere at
/// (3.8, 0, 0), a convex crease circle of radius 0.917 in the plane x = 3.4.
pub const LENS: &str = "unit mm
use std
construction centerline line spindle(std.origin, std.up.toward)
private point center
center distance(3mm, along: u) std.front
center distance(0mm, along: v) std.front
private point bottom hint(x: 3, y: -1)
private point top hint(x: 3, y: 1)
private line diameter(bottom, top)
center midpoint diameter
diameter parallel spindle
private arc meridian(center: center, start: bottom, end: top)
radius(1mm) meridian
construction solid stock(face(meridian, diameter), about: diameter)
construction solid tool(stock)
private point center2
center2 distance(3.8mm, along: u) std.front
center2 distance(0mm, along: v) std.front
private point bottom2 hint(x: 3.8, y: -1)
private point top2 hint(x: 3.8, y: 1)
private line diameter2(bottom2, top2)
center2 midpoint diameter2
diameter2 parallel spindle
private arc meridian2(center: center2, start: bottom2, end: top2)
radius(1mm) meridian2
construction solid other(face(meridian2, diameter2), about: diameter2)
other bound tool
";

/// A dumbbell along z: a bar of radius 0.25 from z = -1 to 1 about the
/// vertical through (3, 0), with balls of radius 0.5 at z = -1 and 1
/// (two concave crease circles where the bar enters each ball).
pub const DUMBBELL: &str = "unit mm
use std
construction centerline line spindle(std.origin, std.up.toward)
private point c0 hint(x: 3, y: -1)
private point c1 hint(x: 3.25, y: -1)
private point c2 hint(x: 3.25, y: 1)
private point c3 hint(x: 3, y: 1)
ground c0
ground c1
ground c2
ground c3
private line bottom(c0, c1)
private line wall(c1, c2)
private line top(c2, c3)
private line axis(c3, c0)
construction solid bar(face(bottom, wall, top, axis), about: axis)
construction solid tool(bar)
private point ca hint(x: 3, y: 1)
ca distance(3mm, along: u) std.front
ca distance(1mm, along: v) std.front
private point ba hint(x: 3, y: 0.5)
private point ta hint(x: 3, y: 1.5)
private line da(ba, ta)
ca midpoint da
da parallel spindle
private arc ma(center: ca, start: ba, end: ta)
radius(0.5mm) ma
construction solid ball_a(face(ma, da), about: da)
ball_a on tool
private point cb hint(x: 3, y: -1)
cb distance(3mm, along: u) std.front
cb distance(-1mm, along: v) std.front
private point bb hint(x: 3, y: -1.5)
private point tb hint(x: 3, y: -0.5)
private line db(bb, tb)
cb midpoint db
db parallel spindle
private arc mb(center: cb, start: bb, end: tb)
radius(0.5mm) mb
construction solid ball_b(face(mb, db), about: db)
ball_b on tool
";
