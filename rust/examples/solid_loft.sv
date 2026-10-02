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

end_plane := plane(origin: std.origin, toward: std.front.toward, from: std.front, offset: -length)
in std.front {
  inlet_center := point hint(x: 0mm, y: 0mm)
  inlet_center coincident std.origin
  inlet := Section(inlet_center, size: inlet_size, wall: wall)
}
in end_plane {
  outlet_center := point hint(x: 0mm, y: 0mm)
  outlet_center coincident std.origin
  outlet := Section(outlet_center, size: outlet_size, wall: wall)
}
plan := plane(origin: std.origin, toward: std.front.toward, from: std.front, fold: 0deg)
in plan {
  entry := point hint(x: 0mm, y: 0mm)
  entry coincident std.origin
  exit := point hint(x: 0mm, y: length)
  construction centerline guide := line(entry, exit)
  vertical guide
  entry distance(length, along: y) exit
}
body := solid(inlet.profile, outlet.profile, along: guide)
