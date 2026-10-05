// The valvetrain, seen along the camshafts: a tangent cam on a flat-faced bucket follower over an
// inclined valve.  A cam lobe here is what a draughtsman draws — a base circle, a nose circle and
// two straight flanks tangent to both — and the follower face is tangent to whichever circle it
// rides, so the valve's lift is the solver's answer and not a number anyone typed.

use engine.dims
use engine.parts

// A tangent cam about `c`, its nose `dn` out at `phi` from the line `ref`, measured
// counter-clockwise: the base circle, the nose circle and the two flanks tangent to both.
component Lobe(c: point, ref: line, phi: Angle, dn: Length, dims: group) {
  base := circle(center: c) hint(r: dims.rb)
  radius(dims.rb) base
  n := point hint(x: c.x + dn * cos(phi + atan2(ref.p2.y - ref.p1.y, ref.p2.x - ref.p1.x)),
               y: c.y + dn * sin(phi + atan2(ref.p2.y - ref.p1.y, ref.p2.x - ref.p1.x)))
  spine := line(c, n)
  c distance(dn) n
  ref angle(phi) spine
  nose := circle(center: n) hint(r: dims.rn)
  radius(dims.rn) nose
  fl := engine.parts.Span(base, nose, side: 1)
  fr := engine.parts.Span(base, nose, side: -1)
}

// A valve on the axis from its seat centre `seat` toward the cam centre at `axis.p2`, lifted
// `lift` off its seat by the lobe: the flat follower face stands `rb + lift` from the cam's
// centre, which is where the lobe's outline reaches at this moment (see `dims.sv`) — so the face
// is tangent to whichever of base circle, flank or nose is under it, without saying which.
component Valve(seat: point, axis: line, lift: Length, head: Length, dims: group) {
  // the follower face: on the axis and square to it, `rb + lift` short of the cam's centre
  fc := point hint(x: axis.p2.x - (dims.rb + lift) * (axis.p2.x - axis.p1.x) / dims.stem, y: axis.p2.y - (dims.rb + lift) * (axis.p2.y - axis.p1.y) / dims.stem)
  fc coincident axis
  axis.p2 distance(dims.rb + lift) fc
  f1 := point hint(x: axis.p2.x - 15mm, y: axis.p2.y - dims.rb)
  f2 := point hint(x: axis.p2.x + 15mm, y: axis.p2.y - dims.rb)
  flat := line(f1, f2)
  fc midpoint flat
  flat perpendicular axis
  f1 distance(30) f2
  // the bucket under the face, 30 wide and 20 deep
  b1 := point hint(x: axis.p2.x - 15mm, y: axis.p2.y - dims.rb - 20mm)
  b2 := point hint(x: axis.p2.x + 15mm, y: axis.p2.y - dims.rb - 20mm)
  bl := line(f1, b1)
  br := line(f2, b2)
  bb := line(b1, b2)
  bl parallel axis
  br parallel axis
  f1 distance(20) b1
  f2 distance(20) b2
  // the stem, `stem` down the axis to the head, which the lobe lifts off its seat or does not
  hc := point hint(at: seat)
  hc coincident axis
  hc distance(dims.stem) fc
  st := line(hc, fc)
  h1 := point hint(x: seat.x - head / 2, y: seat.y)
  h2 := point hint(x: seat.x + head / 2, y: seat.y)
  hd := line(h1, h2)
  hc midpoint hd
  hd perpendicular axis
  h1 distance(head) h2
}
