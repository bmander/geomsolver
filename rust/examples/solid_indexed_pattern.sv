// One source cutter, placed by a named motion at each index.
unit mm
use std

component IndexedCuts(tool: solid, target: solid, indexing: motion, count: Int) {
  repeat count as i {
    solid indexed(tool, under: indexing, at: i * 360deg / count)
    indexed cut target
  }
}

construction centerline line shaft(std.origin, std.up.toward)
motion turn(about: shaft)
plane top(origin: std.origin, toward: std.front.toward, u: (1,0,0), v: (0,1,0))
in top {
  radius(20mm) circle rim(center: std.origin)
  solid stock(face(rim), depth: 5mm)
  private point hole_center
  hole_center distance(12mm, along: u) top
  hole_center distance(0mm, along: v) top
  radius(2mm) circle hole(center: hole_center)
  construction solid tool(face(hole), through: body)
}
solid body(stock)
pattern: IndexedCuts(tool, body, turn, count: 6)
