// A pocketed enclosure with four standoffs. Nested bodies preserve the order:
// hollow the shell first, then add the bosses inside the pocket.
unit mm
use std
width := 72mm
height := 48mm
depth := 16mm
wall := 3mm
boss_radius := 5mm
screw_radius := 2mm
boss_height := 8mm

component Rectangle(center: point, w: Length, h: Length) {
  a := point hint(x: center.x - w / 2, y: center.y - h / 2)
  b := point hint(x: center.x + w / 2, y: center.y - h / 2)
  c := point hint(x: center.x + w / 2, y: center.y + h / 2)
  d := point hint(x: center.x - w / 2, y: center.y + h / 2)
  profile := horizontal (ab := line(a, b)) -> vertical (bc := line(b, c)) ->
            horizontal (cd := line(c, d)) -> vertical (da := line(d, a)) -> close
  distance(w) ab
  distance(h) bc
  diagonal := line(a, c)
  center midpoint diagonal
}
outer := Rectangle(std.origin, w: width, h: height)
inner := Rectangle(std.origin, w: width - 2 * wall, h: height - 2 * wall)
shell := solid(outer.profile, depth: depth)
pocket := solid(inner.profile, depth: depth - wall)
pocket cut shell
body := solid(shell)

// Boss centers follow an inset rectangle: each boss clears the pocket walls by 2mm.
centers := Rectangle(std.origin, w: width - 2 * (wall + boss_radius + 2mm),
                              h: height - 2 * (wall + boss_radius + 2mm))
component Standoff(body: solid, c: point, r: Length, screw_r: Length, base_depth: Length, h: Length) {
    rim := circle(center: c)
    screw := circle(center: c)
    radius(r) rim
    radius(screw_r) screw
    // Annular bosses leave blind screw holes; the floor stays intact underneath.
    boss := solid(face(rim, holes: screw), from: base_depth, to: base_depth + h)
    boss on body
}
// The contour's corner references, rather than independently calculated coordinates.
Standoff(body, centers.a, r: boss_radius, screw_r: screw_radius, base_depth: -depth + wall, h: boss_height)
Standoff(body, centers.b, r: boss_radius, screw_r: screw_radius, base_depth: -depth + wall, h: boss_height)
Standoff(body, centers.c, r: boss_radius, screw_r: screw_radius, base_depth: -depth + wall, h: boss_height)
Standoff(body, centers.d, r: boss_radius, screw_r: screw_radius, base_depth: -depth + wall, h: boss_height)
