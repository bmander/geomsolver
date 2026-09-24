// A torus rolled through a post, with the shape's numbers up front: change one and the part
// rebuilds, refined in a worker and redrawn as it goes (⌘B for the glass box).
//
// The tool is a torus whose tube (radius `tube`) runs round a circle of radius `ring` about its
// own axis. It turns about an upright axis while the post's frame turns about a tilted one, rolling
// from -`roll` to +`roll`, and the part is the post with everything the torus passes through
// taken out of it. Keep `tube` below `ring`, so the torus has a hole.
unit mm
use std

param ring = 1mm        // the torus's centre circle
param tube = 0.5mm      // the torus's tube
param post_r = 0.4mm    // the post
param post_lo = 0.5mm
param post_hi = 2mm
param roll = 75deg      // how far the tool rolls each way

construction centerline line spindle(std.origin, std.up.toward)

// The tool: a circle revolved about an axis `ring` away from its centre.
private point ta hint(x: 3, y: 1)
private point tb hint(x: 3, y: 3)
ground ta
ground tb
private line taxis(ta, tb)
private point tc hint(x: 3mm + ring, y: 2)
ground tc
private circle ring_c(center: tc) hint(r: 0.5)
radius(tube) ring_c
construction solid tool(face(ring_c), about: taxis)

// The motion: a turn about an upright axis, seen from a frame turning about a tilted one.
private point hub hint(x: 2, y: 0)
hub distance(2mm, along: u) std.front
hub distance(0mm, along: v) std.front
private point hub_up hint(x: 2, y: 5)
hub_up distance(2mm, along: u) std.front
hub_up distance(5mm, along: v) std.front
construction centerline line cradle(hub, hub_up)
private motion spin(about: cradle, ratio: 0.25)
private point xend hint(x: 5, y: 0)
xend distance(5mm, along: u) std.front
xend distance(0mm, along: v) std.front
private plane flat(origin: std.origin, toward: xend, u: (1, 0, 0), v: (0, 1, 1))
in flat {
  private point k0 hint(x: 0, y: 0.7071)
  private point k1 hint(x: 5, y: 0.7071)
  k0 distance(0mm, along: u) flat
  k0 distance(0.7071mm, along: v) flat
  k1 distance(5mm, along: u) flat
  k1 distance(0.7071mm, along: v) flat
  construction centerline line kaxis(k0, k1)
}
private motion observer(about: kaxis)
motion turn(spin, relative_to: observer)
construction solid removal(tool, under: turn, from: -roll, to: roll)

// The post: a rectangle revolved about its left side.
private point q0 hint(x: 4, y: post_lo)
private point q1 hint(x: 4mm + post_r, y: post_lo)
private point q2 hint(x: 4mm + post_r, y: post_hi)
private point q3 hint(x: 4, y: post_hi)
ground q0
ground q1
ground q2
ground q3
private line qb(q0, q1)
private line qw(q1, q2)
private line qt(q2, q3)
private line qa(q3, q0)
construction solid stock_post(face(qb, qw, qt, qa), about: qa)
solid part(stock_post)
removal cut part
