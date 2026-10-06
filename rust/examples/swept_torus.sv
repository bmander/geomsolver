// A torus swept through a post: the tool turns about one axis while the post's frame turns
// about another, and the part is the post with everything the torus passes through removed.
// The part's boundary is the material field's, meshed by Delaunay refinement.

unit mm
use std
in std.front {
  construction centerline spindle := line(std.origin, hint((0, 1)))
  fix((0, 1)) spindle.p2
  private ta := point
  private tb := point
  fix((3, 1)) ta
  fix((3, 3)) tb
  private taxis := line(ta, tb)
  private tc := point
  fix((4, 2)) tc
  private ring := circle(center: tc) hint(r: 0.5)
  radius(0.5mm) ring
  construction tool := solid(face(ring), about: taxis)
  private hub := point hint((2, 0))
  hub distance(2mm, along: u) std.front
  hub level(v) std.front
  private hub_up := point hint((2, 5))
  hub_up distance(2mm, along: u) std.front
  hub_up distance(5mm, along: v) std.front
  construction centerline cradle := line(hub, hub_up)
}
private spin := motion(about: cradle, ratio: 0.25)
private rise := axis hint(dir: (0, 1, 1))
fix(dir == (0, sqrt(0.5), sqrt(0.5))) rise
private flat := plane(u: std.x, v: rise)
fix(origin == (0, 0, 0)) flat
in flat {
  private k0 := point hint((0, 0.7071))
  private k1 := point hint((5, 0.7071))
  k0 level(u) flat
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
  fix((4, 0.5)) q0
  fix((4.4, 0.5)) q1
  fix((4.4, 2)) q2
  fix((4, 2)) q3
  private qb := line(q0, q1)
  private qw := line(q1, q2)
  private qt := line(q2, q3)
  private qa := line(q3, q0)
}
construction stock_post := solid(face(qb, qw, qt, qa), about: qa)
part := solid(stock_post)
removal cut part
