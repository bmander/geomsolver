// One source cutter, placed by a frame's turn at each index.
unit mm
use std (on_u)

component IndexedCuts(tool: solid, target: solid, indexing: frame, count: Int) {
  repeat count as i {
    indexed := solid(tool, under: indexing, at: i * 360deg / count)
    indexed cut target
  }
}

in std.front {
  construction centerline shaft := line(std.origin, hint((0, 1)))
  fix((0, 1)) shaft.p2
}
turn := frame
std.Turn(turn, about: shaft, ratio: 1)
in std.top {
  rim := radius(20mm) circle(center: std.top.origin)
  stock := solid(face(rim), depth: 5mm)
  private hole_center := point
  hole_center on_u(d: 12mm) std.top
  hole := radius(2mm) circle(center: hole_center)
  construction tool := solid(face(hole), through: body)
}
body := solid(stock)
pattern := IndexedCuts(tool, body, turn, count: 6)
