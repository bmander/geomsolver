// A pinion rolled against a wheel blank, carrying one pin just inside its pitch circle: the pin
// cuts the tooth space it would drive. The roll's ratio is not stated: the pitch circles roll on
// one another without slipping at the pitch point, which times it from the two pitch radii, so
// editing either radius re-times the roll and the cut follows. The part is the blank with
// everything the pin passes through removed, meshed from its material field (the glass box shows
// it refining).

unit mm
use std
use std.place (coords, on_u)
use std.motion (turns, rolls)
wheel_pitch := 16mm
pinion_pitch := 8mm
pin_throw := 6mm
pin_radius := 1.5mm
addendum := 2.5mm
thickness := 4mm

// Both spindles stand upright in the page, the pitch radii apart. The two radii are drawn, and
// they meet at the pitch point.
in std.front {
  construction centerline spindle := line(std.origin, hint((0, 1)))
  fix((0, 1)) spindle.p2
  private pitch := point hint((16, 0))
  pitch on_u(d: wheel_pitch) std.front
  private hub := point hint((24, 0))
  private hub_up := point hint((24, 5))
  hub on_u(d: wheel_pitch + pinion_pitch) std.front
  hub_up coords(du: wheel_pitch + pinion_pitch, dv: 5mm) std.front
  construction centerline pinion_axis := line(hub, hub_up)
  construction wheel_radius := line(std.origin, pitch)
  construction pinion_radius := line(hub, pitch)
}

// The wheel turns once per turn of the shared angle; the pinion turns the other way, as fast as
// the two pitch circles rolling at the pitch point make it — along the page's normal, where the
// circles touch, the pinion's point there moves with the wheel's.  Its ratio is an unknown the
// rolling finds.
wheel_turn := frame
wheel_turn turns(1) spindle
param pinion_ratio: Scalar
pinion_turn := frame
pinion_turn turns(pinion_ratio) pinion_axis
pinion_turn rolls(at: pitch, along: std.front.normal) wheel_turn

// The blank: a disc out to the addendum circle, turned about the spindle.
in std.front {
  private b0 := point hint((0, -2))
  private b1 := point hint((18.5, -2))
  private b2 := point hint((18.5, 2))
  private b3 := point hint((0, 2))
  blank_section := horizontal (bb := line(b0, b1)) -> vertical (bo := line(b1, b2)) ->
                  horizontal (bt := line(b2, b3)) -> (ba := line(b3, b0)) -> close
  b0 coincident spindle
  std.origin midpoint ba
  b1 distance(wheel_pitch + addendum, side: right) spindle
  length(thickness) bo
  construction blank := solid(blank_section, about: spindle)

  // The pin: a short cylinder on the pinion's radius, taller than the blank is thick.
  private pin_at := point hint((18, 0))
  pin_at coincident pinion_radius
  hub distance(pin_throw) pin_at
  private q0 := point hint((18, -3))
  private q1 := point hint((19.5, -3))
  private q2 := point hint((19.5, 3))
  private q3 := point hint((18, 3))
  pin_section := horizontal (qb := line(q0, q1)) -> vertical (qo := line(q1, q2)) ->
                horizontal (qt := line(q2, q3)) -> vertical (qa := line(q3, q0)) -> close
  pin_at midpoint qa
  length(pin_radius) qb
  length(thickness + 2mm) qo
}
construction pin := solid(pin_section, about: qa)

// Everything the pin passes through while it meets the wheel's blank, found after the solve.
construction space := sweep(pin, through: wheel, under: pinion_turn, seen_from: wheel_turn)
wheel := solid(blank)
space cut wheel
