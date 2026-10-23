// The cantilever as a frame (#510 S0): the corpus's cantilever (`cantilever.sv`) drawn as the
// line its neutral axis runs along, made a beam of its section, built in at its root and loaded
// at its tip.  Euler–Bernoulli's tip deflection P L³ / (3 E I) is a frame's exact answer, and so
// is its root's bending stress P L / (b h² / 6), whose factor against the yield is its safety.
// Its first mode bends in the breadth, the weaker way: 1.8751² / (2π L²) · √(E I / (ρ A)), the
// cubic's to approximate.  The claims bracket each as the solid's do, within 1% and 3%.
unit mm
use std

L := 1000mm
h := 50mm
b := 25mm
P := 1kN
steel := material(E: 200GPa, nu: 0.3, rho: 7850kg / 1m^3, yield: 250MPa)

// the section's second moments: about its z, the front's normal (bending in the front, under the
// load), and about its y (bending across the front); a rectangle's torsion constant at b/h = 1/2
I := b * h^3 / 12
I_weak := h * b^3 / 12
bending := P * L^3 / (3 * steel.E * I)
first := 1.8751^2 / (2 * pi * L^2) * sqrt(steel.E * I_weak / (steel.rho * b * h))
root := P * L / (b * h^2 / 6)

in std.front {
  foot := point
  end := point
  fix((0mm, 0mm)) foot
  fix((L, 0mm)) end
  span := line(foot, end)
}
down := axis
fix(dir == (0, 0, -1), origin == (0, 0, 0)) down
arm := beam(span, A: b * h, Iy: I_weak, Iz: I, J: 0.229 * h * b^3, Sy: h * b^2 / 6,
            Sz: b * h^2 / 6) of steel

// what it is made of answers no load, so it is the drawing's, claimed of it
claim mass(steel.rho * L * b * h) arm

scenario tip_load {
  fixed foot
  load end P along: down
  claim deflection(end, along: down) >= 0.99 * bending
  claim deflection(end, along: down) <= 1.01 * bending
  claim safety(arm) >= 0.97 * steel.yield / root
  claim safety(arm) <= 1.03 * steel.yield / root
}

scenario free {
  fixed foot
  claim mode(arm, 1) >= 0.99 * first
  claim mode(arm, 1) <= 1.01 * first
}
