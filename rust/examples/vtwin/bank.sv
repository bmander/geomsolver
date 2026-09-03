// One bank, seen along the crank axis: the cylinder rocking on its pivot, sectioned through the
// bore, with its piston on the rod whose eye rides the crank pin.  The cylinder and the piston
// are the parts `vtwin.cylinder` and `vtwin.piston` design, drawn here in the plane of swing
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

use vtwin.dims
use vtwin.parts
use vtwin.cylinder
use vtwin.piston

component Bank(swing: plane, side: plane, top: plane, o: point, pin: point,
               piv: point, alpha: Angle, fw: Length, o_s: point, o_t: point, dim: Int) {
  // where the cylinder points at the starting angle: from the pin through the pivot
  param px = R * sin(theta0)
  param py = R * cos(theta0)
  param vx = H * sin(alpha)
  param vy = H * cos(alpha)
  param dir = atan2(vy - py, vx - px)

  in swing {
    // the rod: through the pivot, the crown `L` out from the pin
    point crown hint(x: pin.x + L * cos(dir), y: pin.y + L * sin(dir))
    line rod(pin, crown) class axis
    piv on rod
    pin distance(L) crown

    // the cylinder's own frame: the rod's line, and a line across it through the pivot
    point q hint(x: piv.x - 10mm * sin(dir), y: piv.y + 10mm * cos(dir))
    line across(piv, q) class gone
    q distance(10) rod
    across perpendicular rod
  }

  // the parts, in the plane of swing and rocked with the rod
  cyl: Cylinder(swing, piv, rod, across, dir: dir, fw: fw)
  pis: Piston(swing, side, top, crown, rod,
              dir: dir, pin: pin, o_s: o_s, o_t: o_t, draw_side: 0, draw_top: 0)

  // the dimensions, on one bank
  repeat dim {
    claim cyl.b_tl.p distance(D) cyl.b_tr.p class shown at (0, 14)
    claim pin distance(L) crown class shown at (0, -36)
    claim cyl.k_bl.p distance(ct - cb) cyl.k_tl.p class shown at (0, -48)
    claim pis.ra distance(rt) pis.rc class shown at (0, -20)
  }
}
