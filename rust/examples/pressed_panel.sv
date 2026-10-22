// A panel under pressure (#510 S3): a square steel panel clamped along all four edges and pressed
// on its face by a uniform pressure — a load across the plane it is drawn in, which no reading in
// that plane has a move for, so it is read as the solid it is.  Its middle sinks as Timoshenko's
// clamped square plate does, 0.00126 q a⁴ / D with D = E t³ / (12 (1 − ν²)); at forty times
// thinner than it is wide its shear adds a little to that, and the panel claims its sag within 3%.
unit mm
use std

a := 400mm
t := 10mm
q := 0.1MPa
steel := material(E: 200GPa, nu: 0.3, rho: 7850kg / 1m^3, yield: 250MPa)
D := steel.E * t^3 / (12 * (1 - steel.nu^2))
sag := 0.00126 * q * a^4 / D

// drawn in plan and swept up its thickness: its `near` cap the top, pressed down
in std.top {
  sw := point
  se := point
  ne := point
  nw := point
  middle := point
  fix((0mm, 0mm)) sw
  fix((a, 0mm)) se
  fix((a, a)) ne
  fix((0mm, a)) nw
  fix((a / 2, a / 2)) middle
  square := (south := line(sw, se)) -> (east := line(se, ne)) -> (north := line(ne, nw)) ->
            (west := line(nw, sw)) -> close
}
down := axis
fix(dir == (0, 0, -1), origin == (0, 0, 0)) down
panel := solid(square, from: 0mm, to: t) of steel

scenario pressed {
  fixed panel.south
  fixed panel.east
  fixed panel.north
  fixed panel.west
  pressure panel.near q
  claim deflection(middle, along: down) >= 0.97 * sag
  claim deflection(middle, along: down) <= 1.03 * sag
  claim mass(panel) == steel.rho * a^2 * t
}
