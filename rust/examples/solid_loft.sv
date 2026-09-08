// A hollow reducer: two square sections joined along a dimensioned centerline.
unit mm
use std
param length = 40mm
param inlet_size = 24mm
param outlet_size = 12mm
param wall = 2mm

component Section(c: point, size: Length, wall: Length) {
  private construction outer: CenteredRectangle(c, w: size, h: size)
  private construction inner: CenteredRectangle(c, w: size - 2 * wall, h: size - 2 * wall)
  face profile(outer.loop, holes: inner.loop)
}

plane end_plane(origin: std.origin, toward: std.front.toward, from: std.front, offset: -length)
in std.front {
  point inlet_center hint(x: 0mm, y: 0mm)
  inlet_center coincident std.origin
  inlet: Section(inlet_center, size: inlet_size, wall: wall)
}
in end_plane {
  point outlet_center hint(x: 0mm, y: 0mm)
  outlet_center coincident std.origin
  outlet: Section(outlet_center, size: outlet_size, wall: wall)
}
plane plan(origin: std.origin, toward: std.front.toward, from: std.front, fold: 0deg)
in plan {
  point entry hint(x: 0mm, y: 0mm)
  entry coincident std.origin
  point exit hint(x: 0mm, y: length)
  construction centerline line guide(entry, exit)
  vertical guide
  entry distance(length, along: y) exit
}
solid body(inlet.profile, outlet.profile, along: guide)
