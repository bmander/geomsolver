// A cantilevered plate's modes (#510 S3): a steel plate built in along its root and free
// elsewhere, ringing out of the plane it is drawn in — its first mode bending, its second
// twisting, its third bending again with a node along it — which no reading in that plane has a
// move for, so it is read as the solid it is.  A beam's first is 1.8751² / (2π L²) √(E I / (ρ A))
// and its second bending 4.6941² / 1.8751² times that; held at its root from curling across its
// width, the plate is a little stiffer than the beam, by no more than a plate's 1 / √(1 − ν²).
// A strip's twist, Saint-Venant's (1 / 4L) √(G J / (ρ Ip)) with J = b t³ / 3 and Ip = b³ t / 12,
// is less than the plate's, whose root holds its section from warping.
unit mm
use std
use std.place (horizontal, vertical)

L := 300mm
b := 150mm
t := 5mm
steel := material(E: 200GPa, nu: 0.3, rho: 7850kg / 1m^3, yield: 250MPa)
first := 1.8751^2 / (2 * pi * L^2) * sqrt(steel.E * t^2 / (12 * steel.rho))
plate := 1 / sqrt(1 - steel.nu^2)
twist := 1 / (4 * L) * 2 * t / b * sqrt(steel.E / (2 * (1 + steel.nu)) / steel.rho)

in std.top {
  root_low := point
  end_low := point hint((300, 0))
  end_high := point hint((300, 150))
  root_high := point hint((0, 150))
  fix((0mm, 0mm)) root_low
  root_low horizontal end_low
  root_low distance(L) end_low
  end_low vertical end_high
  end_low distance(b) end_high
  end_high horizontal root_high
  root_high vertical root_low
  outline := (low := line(root_low, end_low)) -> (end := line(end_low, end_high)) ->
             (high := line(end_high, root_high)) -> (root := line(root_high, root_low)) -> close
}
tongue := solid(outline, from: 0mm, to: t) of steel

scenario ring {
  fixed tongue.root
  claim mode(tongue, 1) >= first
  claim mode(tongue, 1) <= plate * first
  claim mode(tongue, 2) >= twist
  claim mode(tongue, 3) >= 4.6941^2 / 1.8751^2 * first
  claim mode(tongue, 3) <= plate * 4.6941^2 / 1.8751^2 * first
}
