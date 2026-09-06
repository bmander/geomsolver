// One bank, seen along the crank axis: the cylinder rocking on its pivot, sectioned through the
// bore, with its piston on the rod whose eye rides the crank pin.  The cylinder and the piston
// are the parts `components.cylinder` and `components.piston` design, drawn here in the plane of swing
// only; this file is the kinematics that places them.
//
// The kinematics is two statements.  The piston's crown is `L` from the pin, and the rod's line
// passes through the pivot — that is what an oscillating cylinder *is*: the rod cannot swing
// relative to the cylinder, so the cylinder swings instead.  Every point of both parts is then
// written in the cylinder's own frame (`Loc`: so far up the rod's line, so far across it), and
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
    claim cyl.b_tl.p distance(dims.D) cyl.b_tr.p
    claim pin distance(dims.L) crown
    claim cyl.k_bl.p distance(dims.ct - dims.cb) cyl.k_tl.p
    claim pis.ra.p distance(dims.rt) pis.rc.p
  }
}

// Open this file to preview bank B at the assembly's starting angle.
preview {
  unit mm
  pin: Loc(std.up, u: R * cos(theta0), v: -R * sin(theta0))
  pivot: Loc(std.up, u: H * cos(alphaR), v: -H * sin(alphaR))
  bank: Bank(pin.p, pivot.p, fw: fwB, dim: 1, dims: vtwin_dims)
}
