// One source cutter, placed by a frame's turn at each index.
unit mm
use std
use std.place (on_u)
use std.motion (turns)

component IndexedCuts(tool: solid, target: solid, indexing: frame, count: Int) {
  repeat count as i {
    indexed := tool at(under: indexing) i * 360deg / count
    indexed cut target
  }
}

in std.front {
  construction centerline shaft := line(std.origin, hint((0, 1)))
  fix((0, 1)) shaft.p2
}
turn := frame
turn turns(1) shaft
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
