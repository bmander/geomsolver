// A hollow reducer: two square sections joined along a dimensioned centerline.
unit mm
use std
length := 40mm
inlet_size := 24mm
outlet_size := 12mm
wall := 2mm

component Section(c: point, size: Length, wall: Length) {
  private construction outer := std.CenteredRectangle(c, w: size, h: size)
  private construction inner := std.CenteredRectangle(c, w: size - 2 * wall, h: size - 2 * wall)
  profile := face(outer.loop, holes: inner.loop)
}

end_plane := plane(u: std.x, v: std.z) hint(x: 0mm, y: length, z: 0mm)
fix(x == 0mm, y == length, z == 0mm) end_plane
in std.front {
  inlet_center := point hint(x: 0mm, y: 0mm)
  inlet_center coincident std.origin
  inlet := Section(inlet_center, size: inlet_size, wall: wall)
}
in end_plane {
  outlet_center := point hint(x: 0mm, y: 0mm)
  outlet_center coincident end_plane.origin
  outlet := Section(outlet_center, size: outlet_size, wall: wall)
}
in std.top {
  entry := point hint(x: 0mm, y: 0mm)
  entry coincident std.top.origin
  exit := point hint(x: 0mm, y: length)
  construction centerline guide := line(entry, exit)
  vertical guide
  entry distance(length, along: y) exit
}
body := solid(inlet.profile, outlet.profile, along: guide)
