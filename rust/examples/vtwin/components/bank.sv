// One bank, seen along the crank axis: the cylinder rocking on its pivot, sectioned through the
// bore, with its piston on the rod whose eye rides the crank pin.  The cylinder and the piston
// are the parts `components.cylinder` and `components.piston` design, each drawn in its own frame;
// this file is the kinematics that places the frames, in the plane of swing `layout.front`.
//
// The kinematics is two statements.  The piston's crown is `L` from the pin, and the rod's line
// passes through the pivot — that is what an oscillating cylinder *is*: the rod cannot swing
// relative to the cylinder, so the cylinder swings instead.  The cylinder's frame stands at the
// pivot along the rod and the piston's at the crown down it, so the two statements place both
// parts, and the whole bank turns with the crank.  Nothing here reads the crank angle: the pin
// is wherever the assembly's `t` puts it, and the drawing is followed along `t` from where it
// starts.  The crown's seed is where the closed form puts it, which is what keeps the solve on
// the branch with the cylinder over the pin rather than folded back through the pivot.

use std
use components.dims
use components.parts
use components.cylinder
use components.piston

component Bank(layout: group, pin: point, piv: point, fw: Length, dim: Int, dims: group) {
  in layout.front {
    // Start on the branch extending from the pin through the pivot.
    crown := point hint(x: pin.x + dims.L * cos(atan2(piv.y - pin.y, piv.x - pin.x)),
                     y: pin.y + dims.L * sin(atan2(piv.y - pin.y, piv.x - pin.x)))
    rod := line(pin, crown)
    piv coincident rod
    pin distance(dims.L) crown

    // The parts' frames: the cylinder's points up from its pivot, the piston's down from its
    // crown to the pin.
    cylinder_axes := std.Turned(piv, crown)
    piston_axes := std.Turned(crown, pin)
  }
  cyl := components.cylinder.Cylinder(cylinder_axes, fw: fw, dims: dims)
  pis := components.piston.Piston(piston_axes, dims: dims)
  // the corners of the cylinder's outline and of its bore, for a view that carries their heights
  corners := {k_tl: cyl.k_tl, k_tr: cyl.k_tr, k_bl: cyl.k_bl, k_br: cyl.k_br, b_tl: cyl.b_tl,
    b_tr: cyl.b_tr, b_bl: cyl.b_bl, b_br: cyl.b_br}

  // the dimensions, on one bank
  when dim == 1 {
    claim cyl.b_tl distance(dims.D) cyl.b_tr
    claim pin distance(dims.L) crown
    claim pis.ra distance(dims.rt) pis.rc
  }
}

// Open this file to preview bank B with the crank pin at the bottom.
preview {
  unit mm
  in std.front {
    pin := point hint(at: std.up, (-components.dims.R, 0mm))
    pivot := point hint(at: std.up, (components.dims.H * cos(components.dims.alphaR), -components.dims.H * sin(components.dims.alphaR)))
    reference := line(std.origin, hint((0, 1)))
    fix((0, 1)) reference.p2
    crank := line(std.origin, pin)
    bank_axis := line(std.origin, pivot)
    std.origin distance(components.dims.R) pin
    std.origin distance(components.dims.H) pivot
    reference angle(180deg, sense: cw) crank
    reference angle(components.dims.alphaR, sense: cw) bank_axis
  }
  layout := {front: std.front, origin: std.origin, ax: reference}
  bank := Bank(layout, pin, pivot, fw: components.dims.fwB, dim: 1, dims: components.dims.vtwin_dims)
}
