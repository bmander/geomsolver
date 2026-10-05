// One bank, seen along the crank axis: the cylinder rocking on its pivot, sectioned through the
// bore, with its piston on the rod whose eye rides the crank pin.  The cylinder and the piston
// are the parts `components.cylinder` and `components.piston` design, drawn here in the plane of swing
// only; this file is the kinematics that places them.
//
// The kinematics is two statements.  The piston's crown is `L` from the pin, and the rod's line
// passes through the pivot — that is what an oscillating cylinder *is*: the rod cannot swing
// relative to the cylinder, so the cylinder swings instead.  Every point of both parts is then
// constrained to the cylinder's axis and to the other edges of its profile, and
// the whole bank turns with the crank.  Nothing here reads the crank angle: the pin is wherever
// the crank's freedom puts it.  Seeds start each point where the closed form puts it at the
// table's starting angle, which is what keeps the solve on the branch with the cylinder over the
// pin rather than folded back through the pivot.

use std
use components.dims
use components.parts
use components.cylinder
use components.piston

component Bank(pin: point, piv: point, fw: Length, dim: Int, dims: group) {
  // Start on the branch extending from the pin through the pivot.
  crown := point hint(x: pin.x + dims.L * cos(atan2(piv.y - pin.y, piv.x - pin.x)),
                   y: pin.y + dims.L * sin(atan2(piv.y - pin.y, piv.x - pin.x)))
  rod := line(pin, crown)
  piv coincident rod
  pin distance(dims.L) crown

  // Explicit moving datums: the cylinder points up from its pivot, the piston down
  // from its crown to the pin. Membership in the swing plane comes from the caller.
  cylinder_axes := plane(origin: piv, toward: crown)
  piston_axes := plane(origin: crown, toward: pin)
  cyl := components.cylinder.Cylinder(cylinder_axes, fw: fw, dims: dims)
  pis := components.piston.Piston(piston_axes, dims: dims)

  // the dimensions, on one bank
  repeat dim {
    claim cyl.b_tl distance(dims.D) cyl.b_tr
    claim pin distance(dims.L) crown
    claim pis.ra distance(dims.rt) pis.rc
  }
}

// Open this file to preview bank B at the assembly's starting angle.
preview {
  unit mm
  pin := point hint(x: std.up.origin.x + (components.dims.R * cos(components.dims.theta0)) * std.up.c - (-components.dims.R * sin(components.dims.theta0)) * std.up.s,
                    y: std.up.origin.y + (components.dims.R * cos(components.dims.theta0)) * std.up.s + (-components.dims.R * sin(components.dims.theta0)) * std.up.c)
  pivot := point hint(x: std.up.origin.x + (components.dims.H * cos(components.dims.alphaR)) * std.up.c - (-components.dims.H * sin(components.dims.alphaR)) * std.up.s,
                    y: std.up.origin.y + (components.dims.H * cos(components.dims.alphaR)) * std.up.s + (-components.dims.H * sin(components.dims.alphaR)) * std.up.c)
  bank := Bank(pin, pivot, fw: components.dims.fwB, dim: 1, dims: components.dims.vtwin_dims)
  reference := line(std.origin, std.up.toward)
  crank := line(std.origin, pin)
  bank_axis := line(std.origin, pivot)
  std.origin distance(components.dims.R) pin
  std.origin distance(components.dims.H) pivot
  reference angle(components.dims.theta0, sense: cw) crank
  reference angle(components.dims.alphaR, sense: cw) bank_axis
}
