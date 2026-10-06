//! The tools every case is written from, as Solvent. Each declares
//! `construction solid tool(...)` and the spindle `spindle` along world z
//! through the origin; a case appends its motion and its sweep.

/// A cylinder of radius 1 and height 2 about the vertical line x = 3, z in [-1, 1].
pub const CYLINDER: &str = "\
unit mm
use std
in std.front {
construction centerline spindle := line(std.origin, hint((0, 1)))
fix((0, 1)) spindle.p2
private c0 := point
private c1 := point
private c2 := point
private c3 := point
fix((3, -1)) c0
fix((4, -1)) c1
fix((4, 1)) c2
fix((3, 1)) c3
private bottom := line(c0, c1)
private wall := line(c1, c2)
private top := line(c2, c3)
private ax := line(c3, c0)
}
construction tool := solid(face(bottom, wall, top, ax), about: ax)
";

/// A box x in [2, 4], y in [-3, 0], z in [-1, 1] (a 2 x 3 profile extruded 2).
pub const BOX: &str = "\
unit mm
use std
in std.front {
construction centerline spindle := line(std.origin, hint((0, 1)))
fix((0, 1)) spindle.p2
private b0 := point
private b1 := point
private b2 := point
private b3 := point
fix((2, -3)) b0
fix((4, -3)) b1
fix((4, 0)) b2
fix((2, 0)) b3
private e0 := line(b0, b1)
private e1 := line(b1, b2)
private e2 := line(b2, b3)
private e3 := line(b3, b0)
}
construction tool := solid(face(e0, e1, e2, e3), from: -1mm, to: 1mm)
";

/// A triangular prism with corners (3, -0.8), (4.5, 0), (3, 0.8), z in [-1.5, 1.5].
pub const TRIANGLE_PRISM: &str = "\
unit mm
use std
in std.front {
construction centerline spindle := line(std.origin, hint((0, 1)))
fix((0, 1)) spindle.p2
private t0 := point
private t1 := point
private t2 := point
fix((3, -0.8)) t0
fix((4.5, 0)) t1
fix((3, 0.8)) t2
private e0 := line(t0, t1)
private e1 := line(t1, t2)
private e2 := line(t2, t0)
}
construction tool := solid(face(e0, e1, e2), from: -1.5mm, to: 1.5mm)
";

/// A dumbbell along z: a bar of radius 0.25 from z = -1 to 1 about the
/// vertical through (3, 0), with balls of radius 0.5 at z = -1 and 1
/// (two concave crease circles where the bar enters each ball).
pub const DUMBBELL: &str = "unit mm
use std
construction centerline spindle := line(std.origin, hint((0, 1)))
fix((0, 1)) spindle.p2
private c0 := point
private c1 := point
private c2 := point
private c3 := point
fix((3, -1)) c0
fix((3.25, -1)) c1
fix((3.25, 1)) c2
fix((3, 1)) c3
private bottom := line(c0, c1)
private wall := line(c1, c2)
private top := line(c2, c3)
private ax := line(c3, c0)
construction bar := solid(face(bottom, wall, top, ax), about: ax)
construction tool := solid(bar)
private ca := point hint((3, 1))
ca distance(3mm, along: u) std.front
ca distance(1mm, along: v) std.front
private ba := point hint((3, 0.5))
private ta := point hint((3, 1.5))
private da := line(ba, ta)
ca midpoint da
da parallel spindle
private ma := arc(center: ca, start: ba, end: ta)
radius(0.5mm) ma
construction ball_a := solid(face(ma, da), about: da)
ball_a on tool
private cb := point hint((3, -1))
cb distance(3mm, along: u) std.front
cb distance(-1mm, along: v) std.front
private bb := point hint((3, -1.5))
private tb := point hint((3, -0.5))
private db := line(bb, tb)
cb midpoint db
db parallel spindle
private mb := arc(center: cb, start: bb, end: tb)
radius(0.5mm) mb
construction ball_b := solid(face(mb, db), about: db)
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
    format!("\
unit mm
use std
in std.front {{
construction centerline spindle := line(std.origin, hint((0, 1)))
fix((0, 1)) spindle.p2
private a0 := point hint(({ax}, {ay}))
a0 distance({ax}mm, along: u) std.front
a0 distance({ay}mm, along: v) std.front
private a1 := point hint(({a1x}, {a1y}))
a1 distance({a1x}mm, along: u) std.front
a1 distance({a1y}mm, along: v) std.front
private a2 := point hint(({a2x}, {a2y}))
a2 distance({a2x}mm, along: u) std.front
a2 distance({a2y}mm, along: v) std.front
private a3 := point hint(({bx}, {by}))
a3 distance({bx}mm, along: u) std.front
a3 distance({by}mm, along: v) std.front
private bottom := line(a0, a1)
private wall := line(a1, a2)
private top := line(a2, a3)
private ax := line(a3, a0)
}}
construction tool := solid(face(bottom, wall, top, ax), about: ax)
",ax=ax,ay=ay,bx=bx,by=by,a1x=ax+c,a1y=ay-s,a2x=bx+c,a2y=by-s)
}

/// A plate 2 wide, `thick` thick and 2 deep, standing at (3, 0) tilted 20 degrees in the page: a
/// thin tool whose contact loop is small and flickers open and closed.
pub fn thin_plate(thick: f64) -> String {
    let (s,c) = 20_f64.to_radians().sin_cos();
    let corner = |u: f64,v: f64| (3.+u*c-v*s,u*s+v*c);
    let (p0,p1,p2,p3) = (corner(-1.,-thick/2.),corner(1.,-thick/2.),corner(1.,thick/2.),corner(-1.,thick/2.));
    format!("\
unit mm
use std
in std.front {{
construction centerline spindle := line(std.origin, hint((0, 1)))
fix((0, 1)) spindle.p2
private q0 := point hint(({x0}, {y0}))
q0 distance({x0}mm, along: u) std.front
q0 distance({y0}mm, along: v) std.front
private q1 := point hint(({x1}, {y1}))
q1 distance({x1}mm, along: u) std.front
q1 distance({y1}mm, along: v) std.front
private q2 := point hint(({x2}, {y2}))
q2 distance({x2}mm, along: u) std.front
q2 distance({y2}mm, along: v) std.front
private q3 := point hint(({x3}, {y3}))
q3 distance({x3}mm, along: u) std.front
q3 distance({y3}mm, along: v) std.front
private f0 := line(q0, q1)
private f1 := line(q1, q2)
private f2 := line(q2, q3)
private f3 := line(q3, q0)
}}
construction tool := solid(face(f0, f1, f2, f3), from: -1mm, to: 1mm)
",x0=p0.0,y0=p0.1,x1=p1.0,y1=p1.1,x2=p2.0,y2=p2.1,x3=p3.0,y3=p3.1)
}

/// A unit sphere centred at (3, 0, `h`), its axis vertical.
pub fn sphere(h: f64) -> String {
    format!("\
unit mm
use std
in std.front {{
construction centerline spindle := line(std.origin, hint((0, 1)))
fix((0, 1)) spindle.p2
private center := point
center distance(3mm, along: u) std.front
center distance({h}mm, along: v) std.front
private bottom := point hint((3, {b}))
private top := point hint((3, {t}))
private diameter := line(bottom, top)
center midpoint diameter
diameter parallel spindle
distance(2mm) diameter
private meridian := arc(center: center, start: bottom, end: top)
radius(1mm) meridian
}}
construction tool := solid(face(meridian, diameter), about: diameter)
",b=h-1.,t=h+1.)
}

/// A torus about the vertical line through (3, 0): a circle of radius 0.5 whose centre is 1
/// from that axis, at height `h`. Its profile never reaches its axis, as a cutter's does not.
pub fn torus(h: f64) -> String {
    format!("\
unit mm
use std
in std.front {{
construction centerline spindle := line(std.origin, hint((0, 1)))
fix((0, 1)) spindle.p2
private ta := point
private tb := point
fix((3, {b})) ta
fix((3, {t})) tb
private taxis := line(ta, tb)
private tc := point
fix((4, {h})) tc
private ring := circle(center: tc) hint(r: 0.5)
radius(0.5mm) ring
}}
construction tool := solid(face(ring), about: taxis)
",b=h-1.,t=h+1.)
}

/// A ring with a sharp rim, as a cutter blade's tip: two tori about the vertical line through
/// (3, 0), tubes of radius 0.5 centred 1 out at heights `h` ± 0.2, intersected. Its meridian is a
/// lens whose two corners are creases, each fanning its normals across a 47° turn, and it never
/// meets its axis.
pub fn ring_lens(h: f64) -> String {
    format!("\
unit mm
use std
in std.front {{
construction centerline spindle := line(std.origin, hint((0, 1)))
fix((0, 1)) spindle.p2
private ta := point
private tb := point
fix((3, {b})) ta
fix((3, {t})) tb
private taxis := line(ta, tb)
private tc := point
fix((4, {lo})) tc
private ring := circle(center: tc) hint(r: 0.5)
radius(0.5mm) ring
construction stock := solid(face(ring), about: taxis)
construction tool := solid(stock)
private tc2 := point
fix((4, {hi})) tc2
private ring2 := circle(center: tc2) hint(r: 0.5)
radius(0.5mm) ring2
}}
construction other := solid(face(ring2), about: taxis)
other bound tool
",b=h-1.,t=h+1.,lo=h-0.2,hi=h+0.2)
}

/// A lens about the tool's axis: two spheres of radius 1 centred on the vertical line through
/// (3, 0), `apart` above and below height `h`, intersected. Its crease, where the two meet, is a
/// convex edge whose normals fan across the dihedral. With `offset`, the second sphere is about
/// a parallel axis 0.8 away instead, at the same height: `lens(0., 0., true)` is the unit sphere
/// at (3, 0, 0) bounded by the one at (3.8, 0, 0), a crease circle of radius 0.917 in x = 3.4.
pub fn lens(h: f64,apart: f64,offset: bool) -> String {
    let (x2,h1,h2) = if offset { (3.8,h,h) } else { (3.,h-apart,h+apart) };
    format!("\
unit mm
use std
in std.front {{
construction centerline spindle := line(std.origin, hint((0, 1)))
fix((0, 1)) spindle.p2
private center := point
center distance(3mm, along: u) std.front
center distance({h1}mm, along: v) std.front
private bottom := point hint((3, {b1}))
private top := point hint((3, {t1}))
private diameter := line(bottom, top)
center midpoint diameter
diameter parallel spindle
private meridian := arc(center: center, start: bottom, end: top)
radius(1mm) meridian
construction stock := solid(face(meridian, diameter), about: diameter)
construction tool := solid(stock)
private center2 := point
center2 distance({x2}mm, along: u) std.front
center2 distance({h2}mm, along: v) std.front
private bottom2 := point hint(({x2}, {b2}))
private top2 := point hint(({x2}, {t2}))
private diameter2 := line(bottom2, top2)
center2 midpoint diameter2
diameter2 parallel spindle
private meridian2 := arc(center: center2, start: bottom2, end: top2)
radius(1mm) meridian2
}}
construction other := solid(face(meridian2, diameter2), about: diameter2)
other bound tool
",b1=h1-1.,t1=h1+1.,b2=h2-1.,t2=h2+1.)
}

/// A post of radius `radius` about the vertical line x = `cx`, z in [low, high], the blank a
/// small fixture cuts: `part`, less the removal.
pub fn post(cx: f64,radius: f64,low: f64,high: f64) -> String {
    let r = cx+radius;
    format!("\
use std
in std.front {{
private q0 := point
private q1 := point
private q2 := point
private q3 := point
fix(({cx}, {low})) q0
fix(({r}, {low})) q1
fix(({r}, {high})) q2
fix(({cx}, {high})) q3
private qb := line(q0, q1)
private qw := line(q1, q2)
private qt := line(q2, q3)
private qa := line(q3, q0)
}}
construction post := solid(face(qb, qw, qt, qa), about: qa)
part := solid(post)
removal cut part
")
}

/// The post the tools start centred on: radius 0.4 about x = 3, z in [-2, 2]; a roll carries
/// them round the spindle.
pub fn centred_post() -> String { post(3.,0.4,-2.,2.) }

/// The post under the torus's cradle roll: at x = `x`, z in [0.5, 2], clear of the torus at both
/// limits of a ±75° roll and of the torus turned on past them.
pub fn skew_post(x: f64) -> String { post(x,0.4,0.5,2.) }

/// A ring about the spindle, radii `inner` to `outer` and heights `low` to `high`, cut by the
/// removal turned to `count` places evenly about the spindle: an indexed body, as a gear's teeth
/// are, small enough to build whole in seconds.
pub fn indexed_ring(count: usize,inner: f64,outer: f64,low: f64,high: f64) -> String {
    format!("\
use std
in std.front {{
private q0 := point
private q1 := point
private q2 := point
private q3 := point
fix(({inner}, {low})) q0
fix(({outer}, {low})) q1
fix(({outer}, {high})) q2
fix(({inner}, {high})) q3
private qb := line(q0, q1)
private qw := line(q1, q2)
private qt := line(q2, q3)
private qa := line(q3, q0)
}}
construction ring_blank := solid(face(qb, qw, qt, qa), about: spindle)
part := solid(ring_blank)
index := motion(about: spindle)
in std.front {{
repeat {count} as i {{
  indexed := solid(removal, under: index, at: i * 360deg / {count})
  indexed cut part
}}
}}
")
}
