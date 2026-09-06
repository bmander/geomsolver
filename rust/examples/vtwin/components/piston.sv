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

use std
use components.dims
use components.parts

component Piston(f: plane, dims: group) {
  // The datum runs down the rod, from crown to pin.
  line rod(f.toward, f.origin)
  param pw = dims.D / 2 - dims.clr
  param gb = dims.grooveb / 2

  // The crown is square to the datum, with its left and right stated explicitly.
  cL: Loc(f, u: 0mm, v: -pw)
  cR: Loc(f, u: 0mm, v: pw)
  line crownl(cR.p, cL.p)
  // each side's profile: down to the groove, in to its bottom, along it, out, on to the skirt
  g0L: Loc(f, u: dims.groove, v: -pw)
  g1L: Loc(f, u: dims.groove, v: -gb)
  g2L: Loc(f, u: dims.groove + dims.groovew, v: -gb)
  g3L: Loc(f, u: dims.groove + dims.groovew, v: -pw)
  sL: Loc(f, u: dims.ph, v: -pw)
  g0R: Loc(f, u: dims.groove, v: pw)
  g1R: Loc(f, u: dims.groove, v: gb)
  g2R: Loc(f, u: dims.groove + dims.groovew, v: gb)
  g3R: Loc(f, u: dims.groove + dims.groovew, v: pw)
  sR: Loc(f, u: dims.ph, v: pw)
  line pL0(cL.p, g0L.p) -> line pL1(g0L.p, g1L.p) -> line pL2(g1L.p, g2L.p) ->
    line pL3(g2L.p, g3L.p) -> line pL4(g3L.p, sL.p)
  line pR0(cR.p, g0R.p) -> line pR1(g0R.p, g1R.p) -> line pR2(g1R.p, g2R.p) ->
    line pR3(g2R.p, g3R.p) -> line pR4(g3R.p, sR.p)
  line skirt(sL.p, sR.p)
  // the eye about the pin, and the hole the pin's shank rides in
  circle eye(center: f.toward) hint(r: dims.reye)
  radius(dims.reye) eye
  circle hole(center: f.toward) hint(r: dims.pinclr / 2)
  radius(dims.pinclr / 2) hole
  // the rod's two flanks, from the skirt to the eye
  ra: Loc(f, u: dims.ph, v: -dims.rt / 2)
  rc: Loc(f, u: dims.ph, v: dims.rt / 2)
  point rb hint(at: eye, bearing: f.angle + 180deg)
  point rd hint(at: eye, bearing: f.angle + 180deg)
  rb on eye
  rd on eye
  rb distance(-dims.rt / 2, along: v) f
  rd distance(dims.rt / 2, along: v) f
  line fl(ra.p, rb)
  line fr(rc.p, rd)
  // -- what the solid is made of, and nothing a view reads ---------------------------------
  // The piston is the *left* profile turned about the rod: a revolution takes one side of its
  // axis, and the right-hand one is drawn because the section shows it.  The loop turns at
  // three corners nothing draws — out to the rim at the crown, in to the axis at the skirt,
  // and back up the axis, which the turn sweeps into nothing — so it says the corners and
  // lets the face close itself.
  s0: Loc(f, u: dims.ph, v: 0mm)
  // the sizes a printer needs
  claim cL.p distance(2 * pw) cR.p
  claim cL.p distance(dims.ph) sL.p
  claim cL.p distance(dims.groove) g0L.p
  claim g1R.p distance(dims.groovew) g2R.p
  claim g1L.p distance(dims.grooveb) g1R.p
  claim f.origin distance(dims.L) f.toward
  claim radius(dims.reye) eye
  claim radius(dims.pinclr / 2) hole
  claim ra.p distance(dims.rt) rc.p

  // -- the solid: the section's faces swept, and the body their one rule (§6.9) ----------------
  // The rod and the eye are half `rw` either side of the plane of swing, which is where the
  // section is drawn and where the crank pin's washers hold it; the piston needs no such
  // statement, being a turn about a line that lies in that plane.
  solid pist(face(f.origin, pL0, pL1, pL2, pL3, pL4, s0.p, -> close), about: rod)
  // the rod between its flanks, closed across the skirt and across the eye
  solid shank(face(fl, rd, fr, ra.p), from: -dims.rw / 2, to: dims.rw / 2)
  solid boss(face(eye), from: -dims.rw / 2, to: dims.rw / 2)
  solid pinhole(face(hole), through: body)
  solid body(pist)
  shank on body
  boss on body
  pinhole cut body
}

// Open this file to preview the piston upright, crown at the origin.
// ../piston.svd arranges three projections of this preview.
preview {
  unit mm
  pin: Loc(std.up, u: -L, v: 0mm)
  plane piston_axes(origin: std.origin, toward: pin.p)
  pis: Piston(piston_axes, dims: vtwin_dims)
}
