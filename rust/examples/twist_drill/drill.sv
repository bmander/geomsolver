// A 10 mm twist drill, its flutes ground as a tool-room grinds them: a wheel carried along a screw
// about the drill's axis, one lead a turn, cuts each flute. Whatever a wheel point touches
// on the way is gone; the flute's wall is where the wheel's characteristic (the points of the
// wheel moving along its own surface) is carried, so the flute's section is not the wheel's.
// A second, wider wheel on the same screw grinds the body clearance and leaves a margin at the
// full diameter behind each lip, and two cones grind the point. Numbers live in
// `configuration.sv`; the wheel is `wheel.sv` and a point's flank `point.sv`.
//
// The flutes run clear through the fluted length (the wheel starts and ends its roll off the
// stock); the shank is added after, square to the axis. A real flute runs out into the shank
// as the wheel lifts away, which is no rigid motion.
unit mm
use std
use configuration
use wheel
use point

construction centerline axis := line(std.origin, std.up.toward)

// the fluted stock and the shank: rectangles in the front view (x right, z up), turned about z
in std.front {
  private f0 := point
  private f1 := point
  private f2 := point
  private f3 := point
  fix(x == 0mm, y == 0mm) f0
  fix(x == configuration.diameter / 2, y == 0mm) f1
  fix(x == configuration.diameter / 2, y == configuration.fluted_length) f2
  fix(x == 0mm, y == configuration.fluted_length) f3
  private stock_axis := line(f3, f0)
  private stock_end := line(f0, f1)
  private stock_wall := line(f1, f2)
  private stock_top := line(f2, f3)
  stock := solid(face(stock_axis, stock_end, stock_wall, stock_top), about: stock_axis)
  private s0 := point
  private s1 := point
  fix(x == 0mm, y == -configuration.shank_length) s0
  fix(x == configuration.diameter / 2, y == -configuration.shank_length) s1
  private shank_axis := line(f0, s0)
  private shank_end := line(s0, s1)
  private shank_wall := line(s1, f1)
  private shank_top := line(f1, f0)
  shank := solid(face(shank_axis, shank_end, shank_wall, shank_top), about: shank_axis)
}

// the screw a wheel rides along, one lead a turn, and the turn from one flute to the next
grind := motion(about: axis, advance: configuration.lead)
index := motion(about: axis)
// from a wheel's rim below the stock to past its top
start := -360deg * configuration.flute_wheel.rim / configuration.lead
finish := 360deg * (configuration.fluted_length + configuration.flute_wheel.rim) / configuration.lead

fluted := solid(stock)

flute_wheel := wheel.GrindingWheel(std.side, setting: configuration.helix, wheel: configuration.flute_wheel)
construction flute := solid(flute_wheel.body, under: grind, from: start, to: finish)

clearance_grind := motion(about: axis, advance: configuration.lead, phase: configuration.clearance_phase)
clearance_wheel := wheel.GrindingWheel(std.side, setting: configuration.helix, wheel: configuration.clearance_wheel)
construction body_clearance := solid(clearance_wheel.body, under: clearance_grind, from: start, to: finish)

repeat configuration.flutes as i {
  construction flute_at := solid(flute, under: index, at: i * 360deg / configuration.flutes)
  flute_at cut fluted
  construction clearance_at := solid(body_clearance, under: index, at: i * 360deg / configuration.flutes)
  clearance_at cut fluted
}

// the point: a cone behind each lip, ground into the stock's top so that the lips' outer corners
// stand a point's length below it, where the flutes' leading edges reach them
corner := configuration.fluted_length - configuration.point_length
lip_at_corner := configuration.lip + 360deg * corner / configuration.lead
flank := point.PointCone(std.side, top: corner, cone: configuration.cone,
  tilt: configuration.tilt, height: configuration.apex_height, offset: configuration.apex_offset,
  reach: configuration.fluted_length + configuration.diameter)
repeat configuration.point {
  repeat configuration.flutes as i {
    construction flank_at := solid(flank.body, under: index, at: lip_at_corner + i * 360deg / configuration.flutes)
    flank_at bound fluted
  }
}

drill := solid(fluted)
shank union drill
