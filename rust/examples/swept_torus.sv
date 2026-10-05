// A torus swept through a post: the tool turns about one axis while the post's frame turns
// about another, and the part is the post with everything the torus passes through removed.
// The part's boundary is the material field's, meshed by Delaunay refinement.

unit mm
use std
in std.front {
  construction centerline spindle := line(std.origin, hint(x: 0, y: 1))
  fix(x == 0, y == 1) spindle.p2
  private ta := point
  private tb := point
  fix(x == 3, y == 1) ta
  fix(x == 3, y == 3) tb
  private taxis := line(ta, tb)
  private tc := point
  fix(x == 4, y == 2) tc
  private ring := circle(center: tc) hint(r: 0.5)
  radius(0.5mm) ring
  construction tool := solid(face(ring), about: taxis)
  private hub := point hint(x: 2, y: 0)
  hub distance(2mm, along: u) std.front
  hub distance(0mm, along: v) std.front
  private hub_up := point hint(x: 2, y: 5)
  hub_up distance(2mm, along: u) std.front
  hub_up distance(5mm, along: v) std.front
  construction centerline cradle := line(hub, hub_up)
}
private spin := motion(about: cradle, ratio: 0.25)
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
private observer := motion(about: kaxis)
turn := motion(spin, relative_to: observer)
construction removal := solid(tool, under: turn, from: -75deg, to: 75deg)
in std.front {
  private q0 := point
  private q1 := point
  private q2 := point
  private q3 := point
  fix(x == 4, y == 0.5) q0
  fix(x == 4.4, y == 0.5) q1
  fix(x == 4.4, y == 2) q2
  fix(x == 4, y == 2) q3
  private qb := line(q0, q1)
  private qw := line(q1, q2)
  private qt := line(q2, q3)
  private qa := line(q3, q0)
}
construction stock_post := solid(face(qb, qw, qt, qa), about: qa)
part := solid(stock_post)
removal cut part
