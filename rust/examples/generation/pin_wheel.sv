// Pin-wheel gearing, as a clockmaker cuts it: a lantern pinion's pin generates the wheel's tooth.
//
// A lantern pinion is two discs joined by round pins; the wheel it drives has teeth shaped so a
// pin rolls along each flank.  The flank is the envelope of the pin as the pinion and the wheel
// turn together at the ratio of their pitch radii, seen from the wheel.  The path of the pin's
// centre (the envelope of the centre itself) is the epicycloid the pinion's pitch circle traces
// rolling round the wheel's, and the flank is that epicycloid set in by the pin's radius.
//
// Both curves are drawn from the same motion; nothing writes a formula for either.  Edit the pin
// radius `rp` or the pitch radii and they are generated again: the 3D counterpart, which cuts
// the tooth space out of a solid wheel, is `../lantern_generation.sv`.

use std (horizontal)

r1 := 10      // the pinion's pitch radius, where its pins stand
r2 := 30      // the wheel's
rp := 2       // the pins

in std.front {
  o1 := point
  o2 := point hint((40, 0))
  fix((0, 0)) o1
  o1 distance(r1 + r2, along: x) o2
  o1 horizontal o2
  pitch1 := circle(center: o1) hint(r: 10)
  pitch2 := circle(center: o2) hint(r: 30)
  radius(r1) pitch1
  radius(r2) pitch2

  // one pin, at the pitch point where the pitch circles touch
  pc := point hint((10, 0))
  pc coincident pitch1
  o1 horizontal pc
  pin := circle(center: pc) hint(r: 2)
  radius(rp) pin
}

pinion := motion(about: o1, ratio: -r2 / r1)
wheel := motion(about: o2, ratio: 1)
seen := motion(pinion, relative_to: wheel)
epicycloid := envelope(pc, under: seen, from: -20deg, to: 20deg)
flank := envelope(pin, under: seen, from: -20deg, to: 20deg, side: far)
