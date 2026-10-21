// The valvetrain, seen along the camshafts: a tangent cam on a flat-faced bucket follower over an
// inclined valve.  A cam lobe here is what a draughtsman draws — a base circle, a nose circle and
// two straight flanks tangent to both — and the follower face is tangent to whichever circle it
// rides, so the valve's lift is the solver's answer and not a number anyone typed.

use engine.dims (rb, rn, stem)
use engine.parts
use std

// A tangent cam about `c`, its nose `dn` out at `phi` from the line `ref`, measured
// counter-clockwise: the base circle, the nose circle and the two flanks tangent to both.
part Lobe(c: point, ref: line, phi: Angle, dn: Length) {
  base := circle(center: c) hint(r: rb)
  radius(rb) base
  n := point hint(x: c.x + dn * cos(phi + atan2(ref.p2.y - ref.p1.y, ref.p2.x - ref.p1.x)),
               y: c.y + dn * sin(phi + atan2(ref.p2.y - ref.p1.y, ref.p2.x - ref.p1.x)))
  spine := line(c, n)
  c distance(dn) n
  ref angle(phi) spine
  nose := circle(center: n) hint(r: rn)
  radius(rn) nose
  fl := engine.parts.Span(base, nose, side: 1)
  fr := engine.parts.Span(base, nose, side: -1)
}

// A valve on the axis from its seat centre `seat` toward the cam centre at `axis.p2`, lifted
// `lift` off its seat by the lobe: the flat follower face stands `rb + lift` from the cam's
// centre, which is where the lobe's outline reaches at this moment (see `dims.sv`) — so the face
// is tangent to whichever of base circle, flank or nose is under it, without saying which.
part Valve(seat: point, ax: line, lift: Length, head: Length) {
  // the follower face: on the axis and square to it, `rb + lift` short of the cam's centre
  fc := point hint((ax.p2.x - (rb + lift) * (ax.p2.x - ax.p1.x) / stem, ax.p2.y - (rb + lift) * (ax.p2.y - ax.p1.y) / stem))
  fc coincident ax
  ax.p2 distance(rb + lift) fc
  f1 := point hint((ax.p2.x - 15mm, ax.p2.y - rb))
  f2 := point hint((ax.p2.x + 15mm, ax.p2.y - rb))
  flat := line(f1, f2)
  fc midpoint flat
  flat perpendicular ax
  f1 distance(30) f2
  // the bucket under the face, 30 wide and 20 deep
  b1 := point hint((ax.p2.x - 15mm, ax.p2.y - rb - 20mm))
  b2 := point hint((ax.p2.x + 15mm, ax.p2.y - rb - 20mm))
  bl := line(f1, b1)
  br := line(f2, b2)
  bb := line(b1, b2)
  bl parallel ax
  br parallel ax
  f1 distance(20) b1
  f2 distance(20) b2
  // the stem, `stem` down the axis to the head, which the lobe lifts off its seat or does not
  hc := point hint(at: seat)
  hc coincident ax
  hc distance(stem) fc
  st := line(hc, fc)
  h1 := point hint((seat.x - head / 2, seat.y))
  h2 := point hint((seat.x + head / 2, seat.y))
  hd := line(h1, h2)
  hc midpoint hd
  hd perpendicular ax
  h1 distance(head) h2
  // which way each piece stands, the axis running from the seat up to the cam: the face's `f1`
  // and the head's `h1` on its left, the cam above the face and the bucket and the stem under it
  ccw(ax.p1, ax.p2, f1)
  ccw(ax.p1, ax.p2, h1)
  ccw(f1, f2, ax.p2)
  cw(f1, f2, b1)
  cw(f1, f2, b2)
  cw(f1, f2, hc)
}

// Open this file to preview cylinder 1's intake valve and its lobe where the table's cycle puts
// them, stood upright: the valve's seat at the origin of the end view, the cam's centre `stem +
// rb` above it.
preview {
  unit mm
  in std.side {
    cam := engine.parts.At(std.side.origin, dx: 0mm, dy: stem + rb)
    vaxis := line(std.side.origin, cam.p)
    ai := (engine.dims.cycle - engine.dims.icenter) / 2
    lobe := Lobe(cam.p, vaxis, phi: 180deg + ai, dn: engine.dims.dn_i)
    valve := Valve(std.side.origin, vaxis, lift: max(rb, engine.dims.dn_i * cos(ai) + rn) - rb,
                   head: engine.dims.div)
  }
}
