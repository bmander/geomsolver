// NAFEMS FV32, the cantilevered tapered membrane (#510 P8): a plate 10 m long tapering evenly
// from 5 m at its built-in root to 1 m at its free end, its in-plane modes in plane stress.  The
// benchmark's six frequencies are 44.623, 130.03, 162.70, 246.05, 379.90 and 391.44 Hz.  They
// are the membrane's alone: read as a solid, a plate this thin bends out of its plane far below
// them, so this case is read in the plane (and a membrane's frequencies do not depend on its
// thickness, which only scales its mass and its stiffness together).
unit mm
use std

thickness := 50mm
fv32 := material(E: 200GPa, nu: 0.3, rho: 8000kg / 1m^3)

in std.front {
  p0 := point
  p1 := point
  p2 := point
  p3 := point
  fix((0mm, 0mm)) p0
  fix((10000mm, 2000mm)) p1
  fix((10000mm, 3000mm)) p2
  fix((0mm, 5000mm)) p3
  outline := (lower := line(p0, p1)) -> (end := line(p1, p2)) -> (upper := line(p2, p3)) ->
             (root := line(p3, p0)) -> close
}
membrane := solid(outline, from: -thickness / 2, to: thickness / 2) of fv32

scenario fv32 {
  fixed membrane.root
  claim mode(membrane, 1) >= 0.99 * 44.623Hz
  claim mode(membrane, 1) <= 1.01 * 44.623Hz
  claim mode(membrane, 2) >= 0.99 * 130.03Hz
  claim mode(membrane, 2) <= 1.01 * 130.03Hz
  claim mode(membrane, 3) >= 0.99 * 162.70Hz
  claim mode(membrane, 3) <= 1.01 * 162.70Hz
  claim mode(membrane, 4) >= 0.99 * 246.05Hz
  claim mode(membrane, 4) <= 1.01 * 246.05Hz
  claim mode(membrane, 5) >= 0.99 * 379.90Hz
  claim mode(membrane, 5) <= 1.01 * 379.90Hz
  claim mode(membrane, 6) >= 0.99 * 391.44Hz
  claim mode(membrane, 6) <= 1.01 * 391.44Hz
}
