// A torus tumbled through space: it turns about an upright axis beside it while being watched
// from a frame that turns about a tilted axis, and the body is everything it passes through
// between -`roll` and +`roll`. Change a number and apply (⌘↵) to watch it refine again; ⌘B shows
// it in the glass box. Keep `tube` below `ring`, so the torus has a hole.
unit mm
use std

param ring = 1mm        // the torus's centre circle
param tube = 0.5mm      // the torus's tube
param roll = 75deg      // how far it tumbles each way

// The torus: a circle turned about an upright axis `ring` away from its centre.
private point ta hint(x: 3, y: 1)
private point tb hint(x: 3, y: 3)
ground ta
ground tb
private line taxis(ta, tb)
private point tc hint(x: 3mm + ring, y: 2)
tc distance(ring, along: right) ta
tc distance(2mm, along: v) std.front
private circle section_c(center: tc) hint(r: tube)
radius(tube) section_c
construction solid torus(face(section_c), about: taxis)

// The motion: a quarter turn about an upright axis for every turn of a frame about a tilted one.
private point hub hint(x: 2, y: 0)
hub distance(2mm, along: u) std.front
hub distance(0mm, along: v) std.front
private point hub_up hint(x: 2, y: 5)
hub vertical hub_up
hub distance(5mm, along: up) hub_up
construction centerline line cradle(hub, hub_up)
private motion spin(about: cradle, ratio: 0.25)
private point xend hint(x: 5, y: 0)
std.origin horizontal xend
std.origin distance(5mm, along: right) xend
private plane tilted(origin: std.origin, toward: xend, u: (1, 0, 0), v: (0, 1, 1))
in tilted {
  private point k0 hint(x: 0, y: 0.7071)
  private point k1 hint(x: 5, y: 0.7071)
  k0 distance(0mm, along: u) tilted
  k0 distance(0.7071mm, along: v) tilted
  k1 distance(5mm, along: u) tilted
  k1 distance(0.7071mm, along: v) tilted
  construction centerline line kaxis(k0, k1)
}
private motion observer(about: kaxis)
motion tumble(spin, relative_to: observer)
solid tumbled(torus, under: tumble, from: -roll, to: roll)
