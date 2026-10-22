// A shelf bracket sized by its sag (#510 S1): a frame of two members — a post built in at its
// foot and an arm off its head, joined rigid at the corner — one section `h` deep, the shelf's
// load spread evenly along the arm.  Its tip's deflection is a driving requirement, so the solve
// finds the depth at which the tip sinks `sag`: the frame's solve reads both members, the load
// spread along the arm is the arm's consistent load wherever the drawing puts it, and the
// adjoint carries the rate of every section number to `h`.  The tip sinks by three things, and
// the depth is the root of their sum: the arm bending off the corner, `w L³ / 8EI`; the post
// bending under the arm's moment `w L / 2`, which turns the corner and swings the tip down
// `w L² H / 2EI`; and the post shortening under the load, `w H / EA`.
unit mm
use std

H := 400mm
L := 300mm
b := 20mm
w := 600N
sag := 1.5mm
steel := material(E: 200GPa, nu: 0.3, rho: 7850kg / 1m^3, yield: 250MPa)
param h: Length hint(25mm)

in std.front {
  foot := point
  corner := point
  tip := point
  fix((0mm, 0mm)) foot
  fix((0mm, H)) corner
  fix((L, H)) tip
  post := line(foot, corner)
  arm := line(corner, tip)
}
down := axis
fix(dir == (0, 0, -1), origin == (0, 0, 0)) down
bracket := beam(post, arm, A: b * h, Iy: h * b^3 / 12, Iz: b * h^3 / 12, J: 0.229 * b^3 * h,
                Sy: h * b^2 / 6, Sz: b * h^2 / 6) of steel

scenario shelf {
  fixed foot
  load arm w along: down
  deflection(tip, along: down) == sag
  claim safety(bracket) >= 2
}
