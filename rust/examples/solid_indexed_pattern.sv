// One source cutter, placed by a named motion at each index.
unit mm
use std

component IndexedCuts(tool: solid, target: solid, indexing: motion, count: Int) {
  repeat count as i {
    indexed := solid(tool, under: indexing, at: i * 360deg / count)
    indexed cut target
  }
}

in std.front {
  construction centerline shaft := line(std.origin, hint(x: 0, y: 1))
  fix(x == 0, y == 1) shaft.p2
}
turn := motion(about: shaft)
in std.top {
  rim := radius(20mm) circle(center: std.top.origin)
  stock := solid(face(rim), depth: 5mm)
  private hole_center := point
  hole_center distance(12mm, along: u) std.top
  hole_center distance(0mm, along: v) std.top
  hole := radius(2mm) circle(center: hole_center)
  construction tool := solid(face(hole), through: body)
}
body := solid(stock)
pattern := IndexedCuts(tool, body, turn, count: 6)
