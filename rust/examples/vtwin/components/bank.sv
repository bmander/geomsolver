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

use components.dims
use components.parts
use components.cylinder
use components.piston

component Bank(pin: point, piv: point, fw: Length, dim: Int, dims: group) {
  // Start on the branch extending from the pin through the pivot.
  point crown hint(x: pin.x + dims.L * cos(atan2(piv.y - pin.y, piv.x - pin.x)),
                   y: pin.y + dims.L * sin(atan2(piv.y - pin.y, piv.x - pin.x)))
  line rod(pin, crown)
  piv on rod
  pin distance(dims.L) crown

  // Explicit moving datums: the cylinder points up from its pivot, the piston down
  // from its crown to the pin. Membership in the swing plane comes from the caller.
  plane cylinder_axes(origin: piv, toward: crown)
  plane piston_axes(origin: crown, toward: pin)
  cyl: Cylinder(cylinder_axes, fw: fw, dims: dims)
  pis: Piston(piston_axes, dims: dims)

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
  point pin hint(x: std.up.origin.x + (R * cos(theta0)) * std.up.c - (-R * sin(theta0)) * std.up.s,
                    y: std.up.origin.y + (R * cos(theta0)) * std.up.s + (-R * sin(theta0)) * std.up.c)
  point pivot hint(x: std.up.origin.x + (H * cos(alphaR)) * std.up.c - (-H * sin(alphaR)) * std.up.s,
                    y: std.up.origin.y + (H * cos(alphaR)) * std.up.s + (-H * sin(alphaR)) * std.up.c)
  bank: Bank(pin, pivot, fw: fwB, dim: 1, dims: vtwin_dims)
  line reference(std.origin, std.up.toward)
  line crank(std.origin, pin)
  line bank_axis(std.origin, pivot)
  std.origin distance(R) pin
  std.origin distance(H) pivot
  reference angle(theta0, sense: cw) crank
  reference angle(alphaR, sense: cw) bank_axis
}
