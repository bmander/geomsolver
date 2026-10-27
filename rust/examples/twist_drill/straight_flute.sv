// A straight-fluted drill, as cut for brass and plastics: the twist drill's flute wheel, set
// square to the drill's axis and slid along it rather than screwed. A slide, like a screw, moves
// every point of the wheel the same way at every time, so the flute's wall is where the wheel's
// characteristic (the points of the wheel moving along its own surface) is carried — here the
// wheel's own profile in the plane through its axis square to the slide, extruded up the drill
// (docs/generating-sweeps.md, the constant-twist class, #684). The drill's numbers and its wheel
// are the twist drill's own (`configuration.sv`, `wheel.sv`); the end is left square.
unit mm
use std
use std.motion (slides, turns)
use configuration
use wheel

// the drill's side datum: u down the drill's axis, v along y, so x is its normal
down := axis
fix(dir == (0, 0, -1)) down
side := plane(u: down, v: std.y)
fix(origin == (0, 0, 0)) side

in std.front {
  construction centerline ax := line(std.origin, hint((0, 1)))
  fix((0, 1)) ax.p2
}

// the fluted stock: a rectangle in the front plane (x right, z up), turned about z
in std.front {
  private f0 := point
  private f1 := point
  private f2 := point
  private f3 := point
  fix((0mm, 0mm)) f0
  fix((configuration.diameter / 2, 0mm)) f1
  fix((configuration.diameter / 2, configuration.fluted_length)) f2
  fix((0mm, configuration.fluted_length)) f3
  private stock_axis := line(f3, f0)
  private stock_end := line(f0, f1)
  private stock_wall := line(f1, f2)
  private stock_top := line(f2, f3)
  stock := solid(face(stock_axis, stock_end, stock_wall, stock_top), about: stock_axis)
}

// the slide the wheel rides up the drill, `travel` a turn of the roll, and the turn from one
// flute to the next; the roll runs from the wheel's rim below the stock to past its top
travel := 100mm
grind := frame
grind slides(travel) ax
index := frame
index turns(1) ax
start := -360deg * configuration.flute_wheel.rim / travel
finish := 360deg * (configuration.fluted_length + configuration.flute_wheel.rim) / travel
param t: Angle over (start, finish)

fluted := solid(stock)

flute_wheel := wheel.GrindingWheel(side, setting: 0deg, wheel: configuration.flute_wheel)
construction flute := sweep(flute_wheel.body over t, under: grind)

repeat configuration.flutes as i {
  construction flute_at := flute at(under: index) i * 360deg / configuration.flutes
  flute_at cut fluted
}

// square to the axis halfway along the flutes: where its section is the wheel's profile
section_plane := plane
fix(origin == (0, 0, configuration.fluted_length / 2)) section_plane
fix(dir == (1, 0, 0)) section_plane.u
fix(dir == (0, 1, 0)) section_plane.v
