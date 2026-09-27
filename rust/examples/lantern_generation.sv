// A pinion rolled against a wheel blank, carrying one pin just inside its pitch circle: the pin
// cuts the tooth space it would drive. The roll's ratio is not typed in. It is measured off the
// drawing after the solve, the wheel's pitch radius over the pinion's, so editing either radius
// re-times the roll and the cut follows. The part is the blank with everything the pin passes
// through removed, meshed from its material field (the glass box shows it refining).

unit mm
use std
param wheel_pitch = 16mm
param pinion_pitch = 8mm
param pin_throw = 6mm
param pin_radius = 1.5mm
param addendum = 2.5mm
param thickness = 4mm

// Both spindles stand upright in the page, the pitch radii apart. The two radii are drawn, and
// they meet at the pitch point.
construction centerline line spindle(std.origin, std.up.toward)
private point pitch hint(x: 16, y: 0)
pitch distance(wheel_pitch, along: u) std.front
pitch distance(0mm, along: v) std.front
private point hub hint(x: 24, y: 0)
private point hub_up hint(x: 24, y: 5)
hub distance(wheel_pitch + pinion_pitch, along: u) std.front
hub distance(0mm, along: v) std.front
hub_up distance(wheel_pitch + pinion_pitch, along: u) std.front
hub_up distance(5mm, along: v) std.front
construction centerline line pinion_axis(hub, hub_up)
construction line wheel_radius(std.origin, pitch)
construction line pinion_radius(hub, pitch)

// The wheel turns once per turn of the shared angle; the pinion turns the other way at the
// ratio of the radii, read from the two drawn lines whenever the motion is read.
motion wheel_turn(about: spindle)
motion pinion_turn(about: pinion_axis, ratio: -length(wheel_radius) / length(pinion_radius))
motion roll(pinion_turn, relative_to: wheel_turn)

// The blank: a disc out to the addendum circle, turned about the spindle.
private point b0 hint(x: 0, y: -2)
private point b1 hint(x: 18.5, y: -2)
private point b2 hint(x: 18.5, y: 2)
private point b3 hint(x: 0, y: 2)
blank_section = horizontal line bb(b0, b1) -> vertical line bo(b1, b2) ->
                horizontal line bt(b2, b3) -> line ba(b3, b0) -> close
b0 on spindle
std.origin midpoint ba
b1 distance(wheel_pitch + addendum, side: right) spindle
distance(thickness) bo
construction solid blank(blank_section, about: spindle)

// The pin: a short cylinder on the pinion's radius, taller than the blank is thick.
private point pin_at hint(x: 18, y: 0)
pin_at on pinion_radius
hub distance(pin_throw) pin_at
private point q0 hint(x: 18, y: -3)
private point q1 hint(x: 19.5, y: -3)
private point q2 hint(x: 19.5, y: 3)
private point q3 hint(x: 18, y: 3)
pin_section = horizontal line qb(q0, q1) -> vertical line qo(q1, q2) ->
              horizontal line qt(q2, q3) -> vertical line qa(q3, q0) -> close
pin_at midpoint qa
distance(pin_radius) qb
distance(thickness + 2mm) qo
construction solid pin(pin_section, about: qa)

// Everything the pin passes through while the wheel turns 35 degrees either way.
construction solid space(pin, under: roll, from: -35deg, to: 35deg)
solid wheel(blank)
space cut wheel
