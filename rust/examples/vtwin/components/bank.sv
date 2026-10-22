// One bank, seen along the crank axis: the cylinder rocking on its pivot, sectioned through the
// bore, with its piston on the rod whose eye rides the crank pin.  The cylinder and the piston
// are the parts `components.cylinder` and `components.piston` design, each drawn in its own frame;
// this file is the kinematics that places the frames, in the plane of swing: the plane the crank
// pin is drawn in.
//
// The kinematics is two statements.  The piston's crown is `L` from the pin, and the rod's line
// passes through the pivot — that is what an oscillating cylinder *is*: the rod cannot swing
// relative to the cylinder, so the cylinder swings instead.  The cylinder's frame stands at the
// pivot along the rod and the piston's at the crown down it, so the two statements place both
// parts, and the whole bank turns with the crank.  The cylinder stands in a frame of its own
// (#542), mated as a part is: its origin on the pivot, its axes square, its v square to the plane
// of swing and its u to the rod, so its z runs up the rod.  Each axis starts as the world's, the
// nearer of its two ways round on either bank, so no seed says which.  Nothing here reads the
// crank angle: the pin is wherever the assembly's `t` puts it, and the drawing is followed along
// `t` from where it starts.  The crown's seed is where the closed form puts it, which is what
// keeps the solve on the branch with the cylinder over the pin rather than folded back through
// the pivot.

use std
use components.dims (D, L, rt)
use components.parts
use components.cylinder
use components.piston

// `pin` is the crank pin and `piv` the pivot the cylinder rocks on; `z` is where the bank's rod
// rides along the pin, its mid-plane off the plate's face.  The cylinder's bore is centred on it,
// so its plate-side wall is what is left: a rod thicker on the bank whose rod rides outboard.
part Bank(pin: point, piv: point, z: Length) {
  in pin.plane {
    // Start on the branch extending from the pin through the pivot.
    crown := point hint(x: pin.x + L * cos(atan2(piv.y - pin.y, piv.x - pin.x)),
                     y: pin.y + L * sin(atan2(piv.y - pin.y, piv.x - pin.x)))
    rod := line(pin, crown)
    piv coincident rod
    pin distance(L) crown

    // The piston's frame, pointing down from its crown to the pin.
    piston_axes := std.Turned(crown, pin)
  }
  in (cylinder_frame := frame) {
    cyl := components.cylinder.Cylinder(fw: z - D / 2)
  }
  cylinder_frame.origin coincident piv
  cylinder_frame.u perpendicular cylinder_frame.v
  cylinder_frame.v parallel pin.plane.normal
  cylinder_frame.u perpendicular rod
  pis := components.piston.Piston(piston_axes)
  // what a view across the crank axis shows of the cylinder: the corners of its outline and of
  // its bore that stand highest and lowest, which are the left-hand top and right-hand bottom
  // where the bank leans clockwise of up (bank R), and the other two where it leans the other way
  // (`side_view.Span`)
  span_cw := {top: cyl.k_tl, bottom: cyl.k_br, bore_top: cyl.b_tl, bore_bottom: cyl.b_br}
  span_ccw := {top: cyl.k_tr, bottom: cyl.k_bl, bore_top: cyl.b_tr, bore_bottom: cyl.b_bl}

  // the dimensions
  claim cyl.b_tl distance(D) cyl.b_tr
  claim pin distance(L) crown
  claim pis.ra distance(rt) pis.rc
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
  bank := Bank(pin, pivot, z: components.dims.zB)
}
