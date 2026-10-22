// A gravity wall (#510 S2): a metre's length of a concrete wall holding back water, read in plane
// strain — a long prism held along its length — under its own weight (`accelerate`, the
// concrete's density times gravity) and the water's mean pressure on its back face, a uniform
// stand-in for one rising with depth.  Its base is built in.  It claims its mass, exactly the
// trapezoid's; that its crest leans downstream less than a tenth of a millimetre; and that its
// first mode in its plane is above 50Hz.
unit mm
use std (horizontal, vertical)

H := 6000mm
crest := 1500mm
foot := 4500mm
length := 1000mm
g := 9.81m / 1s^2
water := 1000kg / 1m^3
concrete := material(E: 30GPa, nu: 0.2, rho: 2400kg / 1m^3)

in std.front {
  heel := point
  toe := point hint((4500, 0))
  top_back := point hint((0, 6000))
  top_front := point hint((1500, 6000))
  fix((0mm, 0mm)) heel
  heel horizontal toe
  heel distance(foot) toe
  heel vertical top_back
  heel distance(H) top_back
  top_back horizontal top_front
  top_back distance(crest) top_front
  section := (base := line(heel, toe)) -> (face_down := line(toe, top_front)) ->
             (top := line(top_front, top_back)) -> (back := line(top_back, heel)) -> close
}
down := axis
fix(dir == (0, 0, -1), origin == (0, 0, 0)) down
wall := solid(section, from: -length / 2, to: length / 2) of concrete

scenario full {
  plane strain wall
  fixed wall.base
  accelerate wall (0m / 1s^2, 0m / 1s^2, -g)
  pressure wall.back water * g * H / 2
  claim mass(wall) >= 0.999999 * concrete.rho * (crest + foot) / 2 * H * length
  claim mass(wall) <= 1.000001 * concrete.rho * (crest + foot) / 2 * H * length
  claim deflection(top_front, along: std.x) <= 0.1mm
  claim mode(wall, 1) >= 50Hz
}
