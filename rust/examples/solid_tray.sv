// A pocketed enclosure with four standoffs. Nested bodies preserve the order:
// hollow the stock first, then add the bosses inside the pocket.
unit mm
use std
param width = 72mm
param height = 48mm
param depth = 16mm
param wall = 3mm
param boss_radius = 5mm
param screw_radius = 2mm
param boss_height = 8mm

component Rectangle(center: point, w: Length, h: Length) {
  point a hint(x: center.x - w / 2, y: center.y - h / 2)
  point b hint(x: center.x + w / 2, y: center.y - h / 2)
  point c hint(x: center.x + w / 2, y: center.y + h / 2)
  point d hint(x: center.x - w / 2, y: center.y + h / 2)
  profile = horizontal line ab(a, b) -> vertical line bc(b, c) ->
            horizontal line cd(c, d) -> vertical line da(d, a) -> close
  distance(w) ab
  distance(h) bc
  line diagonal(a, c)
  center midpoint diagonal
}
outer: Rectangle(std.origin, w: width, h: height)
inner: Rectangle(std.origin, w: width - 2 * wall, h: height - 2 * wall)
solid stock(outer.profile, depth: depth)
solid pocket(inner.profile, depth: depth - wall)
solid shell(stock)
pocket cut shell
solid body(shell)

// Boss centers follow an inset rectangle: each boss clears the pocket walls by 2mm.
centers: Rectangle(std.origin, w: width - 2 * (wall + boss_radius + 2mm),
                              h: height - 2 * (wall + boss_radius + 2mm))
component Standoff(body: solid, c: point, r: Length, screw_r: Length, base_depth: Length, h: Length) {
    circle rim(center: c)
    circle screw(center: c)
    radius(r) rim
    radius(screw_r) screw
    // Annular bosses leave blind screw holes; the floor stays intact underneath.
    solid boss(face(rim, holes: screw), from: base_depth, to: base_depth + h)
    boss on body
}
// The contour's corner references, rather than independently calculated coordinates.
Standoff(body, centers.a, r: boss_radius, screw_r: screw_radius, base_depth: -depth + wall, h: boss_height)
Standoff(body, centers.b, r: boss_radius, screw_r: screw_radius, base_depth: -depth + wall, h: boss_height)
Standoff(body, centers.c, r: boss_radius, screw_r: screw_radius, base_depth: -depth + wall, h: boss_height)
Standoff(body, centers.d, r: boss_radius, screw_r: screw_radius, base_depth: -depth + wall, h: boss_height)
