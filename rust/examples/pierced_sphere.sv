// A sphere with a cylindrical hole straight through it. Change a number and apply (⌘↵) to see it
// rebuild; ⌘B shows it in the glass box. Keep `hole_r + offset` below `sphere_r`, or the hole
// breaks out of the side rather than passing through.
unit mm
use std

sphere_r := 20mm   // the sphere's radius
hole_r := 6mm      // the hole's radius
offset := 5mm      // from the sphere's centre to the hole's axis

// A sphere of radius `r` about `center`: a half disc turned about its diameter, which stands
// upright through the centre.
component Sphere(center: point, r: Length) {
  private bottom := point hint(x: center.x, y: center.y - r)
  private top := point hint(x: center.x, y: center.y + r)
  private diameter := line(bottom, top)
  center midpoint diameter
  vertical diameter
  private meridian := arc(center: center, start: bottom, end: top) hint(r: r)
  radius(r) meridian
  body := solid(face(meridian, diameter), about: diameter)
}

// A round hole of radius `r` through `body`, its axis square to the page, level with `center`
// and `offset` to its right. The hole is the hole's own business: it cuts the body it is given.
component Hole(body: solid, center: point, r: Length, offset: Length) {
  private axis := point hint(x: center.x + offset, y: center.y)
  center horizontal axis
  center distance(offset, along: right) axis
  rim := radius(r) circle(center: axis) hint(r: r)
  private drill := solid(face(rim), through: body)
  drill cut body
}

ball := Sphere(std.origin, r: sphere_r)
part := solid(ball.body)
bore := Hole(part, std.origin, r: hole_r, offset: offset)
