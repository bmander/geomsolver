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

/// A cylinder of radius 1 and height 2 whose axis is tilted `degrees` from the vertical in the
/// page, its middle at (3, 0): the outer branch of its contact switches between the rim and a
/// generator as it turns, and its strands run nearly along the stations.
pub fn tilted_cylinder(degrees: f64) -> String {
    let (s,c) = degrees.to_radians().sin_cos();
    // the axis' ends, and the rim corners a radius out from each, square to the axis
    let (ax,ay) = (3.-s,-c);
    let (bx,by) = (3.+s,c);
    format!("unit mm
use std
construction centerline line spindle(std.origin, std.up.toward)
private point a0 hint(x: {ax}, y: {ay})
a0 distance({ax}mm, along: u) std.front
a0 distance({ay}mm, along: v) std.front
private point a1 hint(x: {a1x}, y: {a1y})
a1 distance({a1x}mm, along: u) std.front
a1 distance({a1y}mm, along: v) std.front
private point a2 hint(x: {a2x}, y: {a2y})
a2 distance({a2x}mm, along: u) std.front
a2 distance({a2y}mm, along: v) std.front
private point a3 hint(x: {bx}, y: {by})
a3 distance({bx}mm, along: u) std.front
a3 distance({by}mm, along: v) std.front
private line bottom(a0, a1)
private line wall(a1, a2)
private line top(a2, a3)
private line axis(a3, a0)
construction solid tool(face(bottom, wall, top, axis), about: axis)
",ax=ax,ay=ay,bx=bx,by=by,a1x=ax+c,a1y=ay-s,a2x=bx+c,a2y=by-s)
}

/// A plate 2 wide, `thick` thick and 2 deep, standing at (3, 0) tilted 20 degrees in the page: a
/// thin tool whose contact loop is small and flickers open and closed.
pub fn thin_plate(thick: f64) -> String {
    let (s,c) = 20_f64.to_radians().sin_cos();
    let corner = |u: f64,v: f64| (3.+u*c-v*s,u*s+v*c);
    let (p0,p1,p2,p3) = (corner(-1.,-thick/2.),corner(1.,-thick/2.),corner(1.,thick/2.),corner(-1.,thick/2.));
    format!("unit mm
use std
construction centerline line spindle(std.origin, std.up.toward)
private point q0 hint(x: {x0}, y: {y0})
q0 distance({x0}mm, along: u) std.front
q0 distance({y0}mm, along: v) std.front
private point q1 hint(x: {x1}, y: {y1})
q1 distance({x1}mm, along: u) std.front
q1 distance({y1}mm, along: v) std.front
private point q2 hint(x: {x2}, y: {y2})
q2 distance({x2}mm, along: u) std.front
q2 distance({y2}mm, along: v) std.front
private point q3 hint(x: {x3}, y: {y3})
q3 distance({x3}mm, along: u) std.front
q3 distance({y3}mm, along: v) std.front
private line f0(q0, q1)
private line f1(q1, q2)
private line f2(q2, q3)
private line f3(q3, q0)
construction solid tool(face(f0, f1, f2, f3), from: -1mm, to: 1mm)
",x0=p0.0,y0=p0.1,x1=p1.0,y1=p1.1,x2=p2.0,y2=p2.1,x3=p3.0,y3=p3.1)
}
