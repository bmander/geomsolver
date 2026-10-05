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
// #014, selected in `dims.sv`; its groove is derived below. Print it crown down.

use std
use components.dims
use components.parts

component Piston(f: group, dims: group) {
  // The datum runs down the rod, from crown to pin.
  rod := line(f.u.p2, f.u.p1)
  pw := dims.D / 2 - dims.clr
  // The selected O-ring and seal proportions determine the groove.
  grooveb := dims.D - 2 * (1 - dims.seal.squeeze) * dims.oring
  groovew := dims.seal.width_factor * dims.oring
  gb := grooveb / 2

  // The crown is square to the datum, with its left and right stated explicitly.
  cL := point hint(at: f.axes, x: 0mm, y: -pw)
  cR := point hint(at: f.axes, x: 0mm, y: pw)
  crownl := line(cR, cL)
  // each side's profile: down to the groove, in to its bottom, along it, out, on to the skirt
  g0L := point hint(at: f.axes, x: dims.groove, y: -pw)
  g1L := point hint(at: f.axes, x: dims.groove, y: -gb)
  g2L := point hint(at: f.axes, x: dims.groove + groovew, y: -gb)
  g3L := point hint(at: f.axes, x: dims.groove + groovew, y: -pw)
  sL := point hint(at: f.axes, x: dims.ph, y: -pw)
  g0R := point hint(at: f.axes, x: dims.groove, y: pw)
  g1R := point hint(at: f.axes, x: dims.groove, y: gb)
  g2R := point hint(at: f.axes, x: dims.groove + groovew, y: gb)
  g3R := point hint(at: f.axes, x: dims.groove + groovew, y: pw)
  sR := point hint(at: f.axes, x: dims.ph, y: pw)
  (pL0 := line(cL, g0L)) -> (pL1 := line(g0L, g1L)) -> (pL2 := line(g1L, g2L)) ->
    (pL3 := line(g2L, g3L)) -> (pL4 := line(g3L, sL))
  (pR0 := line(cR, g0R)) -> (pR1 := line(g0R, g1R)) -> (pR2 := line(g1R, g2R)) ->
    (pR3 := line(g2R, g3R)) -> (pR4 := line(g3R, sR))
  skirt := line(sL, sR)
  // The crown is centered on the rod. The groove is a rectangular step in its left flank;
  // symmetry determines the matching right flank from that one profile.
  f.u.p1 midpoint crownl
  crownl perpendicular rod
  distance(2 * pw) crownl
  pL0 parallel rod
  pL1 perpendicular rod
  pL2 parallel rod
  pL3 perpendicular rod
  pL4 parallel rod
  distance(dims.groove) pL0
  distance(groovew) pL2
  g1L distance(gb, side: left) rod
  pL1 equal pL3
  cL distance(dims.ph) sL
  g0L symmetry(rod) g0R
  g1L symmetry(rod) g1R
  g2L symmetry(rod) g2R
  g3L symmetry(rod) g3R
  sL symmetry(rod) sR

  // the eye about the pin, and the hole the pin's shank rides in
  eye := circle(center: f.u.p2) hint(r: dims.reye)
  radius(dims.reye) eye
  hole := circle(center: f.u.p2) hint(r: dims.pinclr / 2)
  radius(dims.pinclr / 2) hole
  // the rod's two flanks, from the skirt to the eye
  ra := point hint(at: f.axes, x: dims.ph, y: -dims.rt / 2)
  rc := point hint(at: f.axes, x: dims.ph, y: dims.rt / 2)
  rb := point hint(at: f.axes, x: dims.L - dims.reye, y: 0mm)
  rd := point hint(at: f.axes, x: dims.L - dims.reye, y: 0mm)
  rb coincident eye
  rd coincident eye
  fl parallel rod
  fr parallel rod
  fl := line(ra, rb)
  fr := line(rc, rd)
  ra coincident skirt
  ra distance(dims.rt / 2, side: left) rod
  ra symmetry(rod) rc
  // -- what the solid is made of, and nothing a view reads ---------------------------------
  // The piston is the *left* profile turned about the rod: a revolution takes one side of its
  // axis, and the right-hand one is drawn because the section shows it.  The loop turns at
  // three corners nothing draws — out to the rim at the crown, in to the axis at the skirt,
  // and back up the axis, which the turn sweeps into nothing — so it says the corners and
  // lets the face close itself.
  s0 := point hint(at: f.axes, x: dims.ph, y: 0mm)
  s0 midpoint skirt
  // the sizes a printer needs
  claim cL distance(2 * pw) cR
  claim cL distance(dims.ph) sL
  claim cL distance(dims.groove) g0L
  claim g1R distance(groovew) g2R
  claim g1L distance(grooveb) g1R
  claim f.u.p1 distance(dims.L) f.u.p2
  claim radius(dims.reye) eye
  claim radius(dims.pinclr / 2) hole
  claim ra distance(dims.rt) rc

  // -- the solid: the section's faces swept, and the body their one rule (§6.9) ----------------
  // The rod and the eye are half `rw` either side of the plane of swing, which is where the
  // section is drawn and where the crank pin's washers hold it; the piston needs no such
  // statement, being a turn about a line that lies in that plane.
  pist := solid(face(f.u.p1, pL0, pL1, pL2, pL3, pL4, s0, -> close), about: rod)
  // the rod between its flanks, closed across the skirt and across the eye
  shank := solid(face(fl, rd, fr, ra), from: -dims.rw / 2, to: dims.rw / 2)
  boss := solid(face(eye), from: -dims.rw / 2, to: dims.rw / 2)
  pinhole := solid(face(hole), through: body)
  body := solid(pist)
  shank union body
  boss union body
  pinhole cut body
}

// Open this file to preview the piston upright, crown at the origin.
// ../piston.svd arranges three projections of this preview.
preview {
  unit mm
  in std.front {
    pin := point hint(at: std.up, x: -components.dims.L, y: 0mm)
    std.origin vertical pin
    std.origin distance(components.dims.L) pin
    piston_axes := std.Turned(std.origin, pin)
    pis := Piston(piston_axes, dims: components.dims.vtwin_dims)
  }
}
