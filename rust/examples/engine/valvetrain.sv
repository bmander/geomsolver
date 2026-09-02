// The valvetrain, seen along the camshafts: a tangent cam on a flat-faced bucket follower over an
// inclined valve.  A cam lobe here is what a draughtsman draws — a base circle, a nose circle and
// two straight flanks tangent to both — and the follower face is tangent to whichever circle it
// rides, so the valve's lift is the solver's answer and not a number anyone typed.

use engine.dims
use engine.parts

// A tangent cam about `c`, its nose at `phi` from the line `ref`, measured counter-clockwise.
component Lobe(c: point, ref: line, phi: Angle) {
  circle kb(center: c) hint(r: rb)
  port base = kb
  radius(rb) kb
  point n hint(x: c.x + dn * cos(phi + atan2(ref.p2.y - ref.p1.y, ref.p2.x - ref.p1.x)),
               y: c.y + dn * sin(phi + atan2(ref.p2.y - ref.p1.y, ref.p2.x - ref.p1.x)))
  line spine(c, n) class axis
  c distance(dn) n
  ref angle(phi) spine
  circle kn(center: n) hint(r: rn)
  port nose = kn
  radius(rn) kn
  fl: Span(kb, kn, side: 1)
  fr: Span(kb, kn, side: -1)
}

// A valve on the axis from its seat centre `seat` toward the cam centre at `axis.p2`, its flat
// follower face resting on the lobe's circle `ride`: the base circle shut, the nose circle open.
component Valve(seat: point, axis: line, ride: circle, head: Length) {
  // the follower face: on the axis and square to it, tangent to the lobe
  port fc: point hint at axis.p2
  fc on axis
  point f1 hint(x: axis.p2.x - 15mm, y: axis.p2.y - rb)
  point f2 hint(x: axis.p2.x + 15mm, y: axis.p2.y - rb)
  line face(f1, f2)
  fc midpoint face
  face perpendicular axis
  f1 distance(30) f2
  face tangent ride
  // the bucket under the face, 30 wide and 20 deep
  point b1 hint(x: axis.p2.x - 15mm, y: axis.p2.y - rb - 20mm)
  point b2 hint(x: axis.p2.x + 15mm, y: axis.p2.y - rb - 20mm)
  line bl(f1, b1)
  line br(f2, b2)
  line bb(b1, b2)
  bl parallel axis
  br parallel axis
  f1 distance(20) b1
  f2 distance(20) b2
  // the stem, `stem` down the axis to the head, which the lobe lifts off its seat or does not
  port hc: point hint at seat
  hc on axis
  hc distance(stem) fc
  line st(hc, fc)
  point h1 hint(x: seat.x - head / 2, y: seat.y)
  point h2 hint(x: seat.x + head / 2, y: seat.y)
  line hd(h1, h2)
  hc midpoint hd
  hd perpendicular axis
  h1 distance(head) h2
}
