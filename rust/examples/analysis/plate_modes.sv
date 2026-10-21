// A simply supported square plate's modes, the corpus's natural frequencies (#510 P8).  Each
// edge face is held as a diaphragm, square to the plate and along its edge, and free to move out
// of it, the support under which the plate's modes are exact in three dimensions: mode (m, n) is
// sin(mπx/a) sin(nπy/a) across, its frequency the flexural Rayleigh–Lamb root of a plate this
// thick at the wavenumber π √(m² + n²) / a.  Kirchhoff's (π/2)((m² + n²)/a²) √(D / ρh) is that
// root's thin limit, 1% high at (1, 1) here and 4% at (1, 3).  The first six are (1, 1), the pair
// (1, 2) and (2, 1), (2, 2), and the pair (1, 3) and (3, 1); each pair is one frequency twice.
unit mm
use std

a := 1000mm
h := 50mm
steel := material(E: 200GPa, nu: 0.3, rho: 7850kg / 1m^3, yield: 250MPa)

in std.front {
  b0 := point
  b1 := point
  b2 := point
  b3 := point
  fix((0mm, 0mm)) b0
  fix((a, 0mm)) b1
  fix((a, a)) b2
  fix((0mm, a)) b3
  square := (south := line(b0, b1)) -> (east := line(b1, b2)) -> (north := line(b2, b3)) ->
            (west := line(b3, b0)) -> close
}
// drawn in the front, x and z, and its thickness swept across y
plate := solid(square, from: -h / 2, to: h / 2) of steel

scenario ring {
  pinned plate.west along: std.z
  pinned plate.west along: std.y
  pinned plate.east along: std.z
  pinned plate.east along: std.y
  pinned plate.south along: std.x
  pinned plate.south along: std.y
  pinned plate.north along: std.x
  pinned plate.north along: std.y
  // the roots, to five figures
  claim mode(plate, 1) >= 0.99 * 237.87Hz
  claim mode(plate, 1) <= 1.01 * 237.87Hz
  claim mode(plate, 2) >= 0.99 * 587.22Hz
  claim mode(plate, 2) <= 1.01 * 587.22Hz
  claim mode(plate, 3) >= 0.99 * 587.22Hz
  claim mode(plate, 3) <= 1.01 * 587.22Hz
  claim mode(plate, 4) >= 0.99 * 928.17Hz
  claim mode(plate, 4) <= 1.01 * 928.17Hz
  claim mode(plate, 5) >= 0.99 * 1151.1Hz
  claim mode(plate, 5) <= 1.01 * 1151.1Hz
  claim mode(plate, 6) >= 0.99 * 1151.1Hz
  claim mode(plate, 6) <= 1.01 * 1151.1Hz
}
