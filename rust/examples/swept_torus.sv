// A torus swept through a post: the tool turns about one axis while the post's frame turns
// about another, and the part is the post with everything the torus passes through removed.
// The part's boundary is the material field's, meshed by Delaunay refinement.

unit mm
use std
construction centerline line spindle(std.origin, std.up.toward)
private point ta hint(x: 3, y: 1)
private point tb hint(x: 3, y: 3)
ground ta
ground tb
private line taxis(ta, tb)
private point tc hint(x: 4, y: 2)
ground tc
private circle ring(center: tc) hint(r: 0.5)
radius(0.5mm) ring
construction solid tool(face(ring), about: taxis)
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
construction solid removal(tool, under: turn, from: -75deg, to: 75deg)
private point q0 hint(x: 4, y: 0.5)
private point q1 hint(x: 4.4, y: 0.5)
private point q2 hint(x: 4.4, y: 2)
private point q3 hint(x: 4, y: 2)
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
