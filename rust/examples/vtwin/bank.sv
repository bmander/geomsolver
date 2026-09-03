// One bank, seen along the crank axis: the cylinder rocking on its pivot, sectioned through the
// bore, with its piston on the rod whose eye rides the crank pin.
//
// The kinematics is two statements.  The piston's crown is `L` from the pin, and the rod's line
// passes through the pivot — that is what an oscillating cylinder *is*: the rod cannot swing
// relative to the cylinder, so the cylinder swings instead.  Every other point of the bank is
// then written in the cylinder's own frame (`Loc`: so far up the rod's line, so far across it),
// and the whole bank turns with the crank.  Nothing here reads the crank angle: the pin is
// wherever the crank's freedom puts it.  Seeds start each point where the closed form puts it at
// the table's starting angle, which is what keeps the solve on the branch with the cylinder over
// the pin rather than folded back through the pivot.

use vtwin.dims
use vtwin.parts

component Bank(o: point, pin: point, eye: circle, piv: point, alpha: Angle, dim: Int) {
  // where the cylinder points at the starting angle: from the pin through the pivot
  param px = R * sin(theta0)
  param py = R * cos(theta0)
  param vx = H * sin(alpha)
  param vy = H * cos(alpha)
  param dir = atan2(vy - py, vx - px)

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

  // the body, from the mouth of the bore to the head
  k_bl: Loc(piv, rod, across, dir, u: cb - H, v: hw)
  k_br: Loc(piv, rod, across, dir, u: cb - H, v: -hw)
  k_tr: Loc(piv, rod, across, dir, u: ct - H, v: -hw)
  k_tl: Loc(piv, rod, across, dir, u: ct - H, v: hw)
  line mouth(k_bl.p, k_br.p) -> line side_r(k_br.p, k_tr.p) -> line top(k_tr.p, k_tl.p) ->
    line side_l(k_tl.p, k_bl.p) -> close

  // the bore, in section
  b_bl: Loc(piv, rod, across, dir, u: cb - H, v: D / 2)
  b_br: Loc(piv, rod, across, dir, u: cb - H, v: -D / 2)
  b_tr: Loc(piv, rod, across, dir, u: head - H, v: -D / 2)
  b_tl: Loc(piv, rod, across, dir, u: head - H, v: D / 2)
  line bore_l(b_bl.p, b_tl.p)
  line bore_r(b_br.p, b_tr.p)
  line hd(b_tl.p, b_tr.p)

  // the port: `a` up from the pivot, drilled from the face into the top of the bore
  pt: Loc(piv, rod, across, dir, u: a, v: 0mm)
  circle port(center: pt.p) hint(r: dport / 2) class hidden
  radius(dport / 2) port

  // the piston: its crown square to the rod, a skirt `ph` below, the O-ring groove
  param pw = D / 2 - clr
  point cL hint(x: crown.x - pw * sin(dir), y: crown.y + pw * cos(dir))
  point cR hint(x: crown.x + pw * sin(dir), y: crown.y - pw * cos(dir))
  line crownl(cR, cL)
  crown midpoint crownl
  crownl perpendicular rod
  cL distance(pw) rod
  sL: Loc(crown, rod, crownl, dir, u: -ph, v: pw)
  sR: Loc(crown, rod, crownl, dir, u: -ph, v: -pw)
  line pl(cL, sL.p)
  line skirt(sL.p, sR.p)
  line pr(sR.p, cR)
  g0L: Loc(crown, rod, crownl, dir, u: -groove, v: pw)
  g0R: Loc(crown, rod, crownl, dir, u: -groove, v: -pw)
  g1L: Loc(crown, rod, crownl, dir, u: -(groove + oring), v: pw)
  g1R: Loc(crown, rod, crownl, dir, u: -(groove + oring), v: -pw)
  line g0(g0L.p, g0R.p) class thin
  line g1(g1L.p, g1R.p) class thin

  // the rod's two flanks, from the skirt to the eye
  point ra hint(x: crown.x - ph * cos(dir) - rt / 2 * sin(dir), y: crown.y - ph * sin(dir) + rt / 2 * cos(dir))
  point rb hint(x: pin.x + reye * cos(dir) - rt / 2 * sin(dir), y: pin.y + reye * sin(dir) + rt / 2 * cos(dir))
  point rc hint(x: crown.x - ph * cos(dir) + rt / 2 * sin(dir), y: crown.y - ph * sin(dir) - rt / 2 * cos(dir))
  point rd hint(x: pin.x + reye * cos(dir) + rt / 2 * sin(dir), y: pin.y + reye * sin(dir) - rt / 2 * cos(dir))
  ra on skirt
  rb on eye
  rc on skirt
  rd on eye
  ra distance(rt / 2) rod
  rb distance(rt / 2) rod
  rc distance(-rt / 2) rod
  rd distance(-rt / 2) rod
  line fl(ra, rb)
  line fr(rc, rd)

  // the dimensions, on one bank
  repeat dim {
    claim b_tl.p distance(D) b_tr.p class shown at (0, 14)
    claim pin distance(L) crown class shown at (0, -36)
    claim k_bl.p distance(ct - cb) k_tl.p class shown at (0, -48)
    claim ra distance(rt) rc class shown at (0, -20)
  }
}
