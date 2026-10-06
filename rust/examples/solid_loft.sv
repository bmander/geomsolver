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

end_plane := plane
fix(origin == (0mm, length, 0mm)) end_plane
fix(dir == (1, 0, 0)) end_plane.u
fix(dir == (0, 0, 1)) end_plane.v
in std.front {
  inlet_center := point hint((0mm, 0mm))
  inlet_center coincident std.origin
  inlet := Section(inlet_center, size: inlet_size, wall: wall)
}
in end_plane {
  outlet_center := point hint((0mm, 0mm))
  outlet_center coincident end_plane.origin
  outlet := Section(outlet_center, size: outlet_size, wall: wall)
}
in std.top {
  entry := point hint((0mm, 0mm))
  entry coincident std.top.origin
  exit := point hint((0mm, length))
  construction centerline guide := line(entry, exit)
  vertical guide
  entry distance(length, along: y) exit
}
body := solid(inlet.profile, outlet.profile, along: guide)
