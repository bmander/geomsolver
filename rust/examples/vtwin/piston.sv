// The piston and its rod, one printed part: **one section, and the solid it is a section of**
// (§6.9).
//
// The outline in the plane of swing is the design, and the part is what that outline *is*: the
// piston is round, so its left-hand profile — crown, O-ring groove, skirt — turned about the
// rod's own line is the piston, groove and all; the rod is its two flanks `rw` thick, the eye a
// disc of the same thickness, and the clevis pin's hole a bore through both.  Nothing here says
// how wide the piston is from the side, because nothing needs to: it is as wide as it is across
// by construction, which is what a revolution means.
//
// It used to be written three times — this section, then the same body redrawn from the side as
// `Slab`s and from the crown as a disc, tied back by six projections, with the groove's depth
// stated twice and the eye's thickness in a place the section could not see.  The other views
// are now asked for (`view(pis.body) in …`), so they cannot disagree with it.
//
// The assembly instances it once a bank, its crown `L` from the pin along a rod that passes
// through the cylinder's pivot; the part sheet instances it once, upright.  The O-ring is a
// #014 and the groove is sized to it in `dims.sv`.  Print it crown down.

use vtwin.dims
use vtwin.parts

component Piston(swing: plane, crown: point, rod: line, dir: Angle, pin: point) {
  param pw = D / 2 - clr
  param gb = grooveb / 2

  in swing {
    // the crown, square to the rod
    point cL hint(x: crown.x - pw * sin(dir), y: crown.y + pw * cos(dir))
    point cR hint(x: crown.x + pw * sin(dir), y: crown.y - pw * cos(dir))
    line crownl(cR, cL)
    crown midpoint crownl
    crownl perpendicular rod
    cL distance(pw) rod
    // each side's profile: down to the groove, in to its bottom, along it, out, on to the skirt
    g0L: Loc(crown, rod, crownl, dir: dir, u: -groove, v: pw)
    g1L: Loc(crown, rod, crownl, dir: dir, u: -groove, v: gb)
    g2L: Loc(crown, rod, crownl, dir: dir, u: -(groove + groovew), v: gb)
    g3L: Loc(crown, rod, crownl, dir: dir, u: -(groove + groovew), v: pw)
    sL: Loc(crown, rod, crownl, dir: dir, u: -ph, v: pw)
    g0R: Loc(crown, rod, crownl, dir: dir, u: -groove, v: -pw)
    g1R: Loc(crown, rod, crownl, dir: dir, u: -groove, v: -gb)
    g2R: Loc(crown, rod, crownl, dir: dir, u: -(groove + groovew), v: -gb)
    g3R: Loc(crown, rod, crownl, dir: dir, u: -(groove + groovew), v: -pw)
    sR: Loc(crown, rod, crownl, dir: dir, u: -ph, v: -pw)
    line pL0(cL, g0L.p) -> line pL1(g0L.p, g1L.p) -> line pL2(g1L.p, g2L.p) ->
      line pL3(g2L.p, g3L.p) -> line pL4(g3L.p, sL.p)
    line pR0(cR, g0R.p) -> line pR1(g0R.p, g1R.p) -> line pR2(g1R.p, g2R.p) ->
      line pR3(g2R.p, g3R.p) -> line pR4(g3R.p, sR.p)
    line skirt(sL.p, sR.p)
    // the eye about the pin, and the hole the pin's shank rides in
    circle eye(center: pin) hint(r: reye)
    radius(reye) eye
    circle hole(center: pin) hint(r: pinclr / 2) class hidden detail
    radius(pinclr / 2) hole
    // the rod's two flanks, from the skirt to the eye
    point ra hint(x: crown.x - ph * cos(dir) - rt / 2 * sin(dir), y: crown.y - ph * sin(dir) + rt / 2 * cos(dir))
    point rb hint(x: pin.x + reye * cos(dir) - rt / 2 * sin(dir), y: pin.y + reye * sin(dir) + rt / 2 * cos(dir))
    point rc hint(x: crown.x - ph * cos(dir) + rt / 2 * sin(dir), y: crown.y - ph * sin(dir) - rt / 2 * cos(dir))
    point rd hint(x: pin.x + reye * cos(dir) + rt / 2 * sin(dir), y: pin.y + reye * sin(dir) - rt / 2 * cos(dir))
    ra on skirt
    rb on eye
    rc on skirt
    rd on eye
    ra distance(rt / 2, side: left) rod
    rb distance(rt / 2, side: left) rod
    rc distance(rt / 2, side: right) rod
    rd distance(rt / 2, side: right) rod
    line fl(ra, rb)
    line fr(rc, rd)
    // -- what the solid is made of, and nothing a view reads ---------------------------------
    // The piston is the *left* profile turned about the rod: a revolution takes one side of its
    // axis, and the right-hand one is drawn because the section shows it.  The loop turns at
    // three corners nothing draws — out to the rim at the crown, in to the axis at the skirt,
    // and back up the axis, which the turn sweeps into nothing — so it says the corners and
    // lets the face close itself.
    s0: Loc(crown, rod, crownl, dir: dir, u: -ph, v: 0mm)
    // the sizes a printer needs
    claim cL distance(2 * pw) cR class detail at (0, 8)
    claim cL distance(ph) sL.p class detail at (0, 12)
    claim cL distance(groove) g0L.p class detail at (0, 6)
    claim g1R.p distance(groovew) g2R.p class detail at (0, -8)
    claim g1L.p distance(grooveb) g1R.p class detail at (0, -3)
    claim crown distance(L) pin class detail at (0, 20)
    claim radius(reye) eye class detail at (-2.4, 14)
    claim radius(pinclr / 2) hole class detail at (-0.7, 16)
    claim ra distance(rt) rc class detail at (0, -8)
  }

  // -- the solid: the section's faces swept, and the body their one rule (§6.9) ----------------
  // The rod and the eye are half `rw` either side of the plane of swing, which is where the
  // section is drawn and where the crank pin's washers hold it; the piston needs no such
  // statement, being a turn about a line that lies in that plane.
  solid pist(face(crown, pL0, pL1, pL2, pL3, pL4, s0.p, -> close), about: rod)
  // the rod between its flanks, closed across the skirt and across the eye
  solid shank(face(fl, rd, fr, ra), from: -rw / 2, to: rw / 2)
  solid boss(face(eye), from: -rw / 2, to: rw / 2)
  solid pinhole(face(hole), from: -rw, to: rw)
  solid body(pist)
  shank on body
  boss on body
  pinhole through body
}
