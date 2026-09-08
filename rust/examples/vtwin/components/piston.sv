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

component Piston(f: plane, dims: group) {
  // The datum runs down the rod, from crown to pin.
  line rod(f.toward, f.origin)
  param pw = dims.D / 2 - dims.clr
  // The selected O-ring and seal proportions determine the groove.
  param grooveb = dims.D - 2 * (1 - dims.seal.squeeze) * dims.oring
  param groovew = dims.seal.width_factor * dims.oring
  param gb = grooveb / 2

  // The crown is square to the datum, with its left and right stated explicitly.
  point cL hint(x: f.origin.x + (0mm) * f.c - (-pw) * f.s,
                    y: f.origin.y + (0mm) * f.s + (-pw) * f.c)
  point cR hint(x: f.origin.x + (0mm) * f.c - (pw) * f.s,
                    y: f.origin.y + (0mm) * f.s + (pw) * f.c)
  line crownl(cR, cL)
  // each side's profile: down to the groove, in to its bottom, along it, out, on to the skirt
  point g0L hint(x: f.origin.x + (dims.groove) * f.c - (-pw) * f.s,
                    y: f.origin.y + (dims.groove) * f.s + (-pw) * f.c)
  point g1L hint(x: f.origin.x + (dims.groove) * f.c - (-gb) * f.s,
                    y: f.origin.y + (dims.groove) * f.s + (-gb) * f.c)
  point g2L hint(x: f.origin.x + (dims.groove + groovew) * f.c - (-gb) * f.s,
                    y: f.origin.y + (dims.groove + groovew) * f.s + (-gb) * f.c)
  point g3L hint(x: f.origin.x + (dims.groove + groovew) * f.c - (-pw) * f.s,
                    y: f.origin.y + (dims.groove + groovew) * f.s + (-pw) * f.c)
  point sL hint(x: f.origin.x + (dims.ph) * f.c - (-pw) * f.s,
                    y: f.origin.y + (dims.ph) * f.s + (-pw) * f.c)
  point g0R hint(x: f.origin.x + (dims.groove) * f.c - (pw) * f.s,
                    y: f.origin.y + (dims.groove) * f.s + (pw) * f.c)
  point g1R hint(x: f.origin.x + (dims.groove) * f.c - (gb) * f.s,
                    y: f.origin.y + (dims.groove) * f.s + (gb) * f.c)
  point g2R hint(x: f.origin.x + (dims.groove + groovew) * f.c - (gb) * f.s,
                    y: f.origin.y + (dims.groove + groovew) * f.s + (gb) * f.c)
  point g3R hint(x: f.origin.x + (dims.groove + groovew) * f.c - (pw) * f.s,
                    y: f.origin.y + (dims.groove + groovew) * f.s + (pw) * f.c)
  point sR hint(x: f.origin.x + (dims.ph) * f.c - (pw) * f.s,
                    y: f.origin.y + (dims.ph) * f.s + (pw) * f.c)
  line pL0(cL, g0L) -> line pL1(g0L, g1L) -> line pL2(g1L, g2L) ->
    line pL3(g2L, g3L) -> line pL4(g3L, sL)
  line pR0(cR, g0R) -> line pR1(g0R, g1R) -> line pR2(g1R, g2R) ->
    line pR3(g2R, g3R) -> line pR4(g3R, sR)
  line skirt(sL, sR)
  // The crown is centered on the rod. The groove is a rectangular step in its left flank;
  // symmetry determines the matching right flank from that one profile.
  f.origin midpoint crownl
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
  circle eye(center: f.toward) hint(r: dims.reye)
  radius(dims.reye) eye
  circle hole(center: f.toward) hint(r: dims.pinclr / 2)
  radius(dims.pinclr / 2) hole
  // the rod's two flanks, from the skirt to the eye
  point ra hint(x: f.origin.x + (dims.ph) * f.c - (-dims.rt / 2) * f.s,
                    y: f.origin.y + (dims.ph) * f.s + (-dims.rt / 2) * f.c)
  point rc hint(x: f.origin.x + (dims.ph) * f.c - (dims.rt / 2) * f.s,
                    y: f.origin.y + (dims.ph) * f.s + (dims.rt / 2) * f.c)
  point rb hint(at: eye, bearing: f.angle + 180deg)
  point rd hint(at: eye, bearing: f.angle + 180deg)
  rb on eye
  rd on eye
  fl parallel rod
  fr parallel rod
  line fl(ra, rb)
  line fr(rc, rd)
  ra on skirt
  ra distance(dims.rt / 2, side: left) rod
  ra symmetry(rod) rc
  // -- what the solid is made of, and nothing a view reads ---------------------------------
  // The piston is the *left* profile turned about the rod: a revolution takes one side of its
  // axis, and the right-hand one is drawn because the section shows it.  The loop turns at
  // three corners nothing draws — out to the rim at the crown, in to the axis at the skirt,
  // and back up the axis, which the turn sweeps into nothing — so it says the corners and
  // lets the face close itself.
  point s0 hint(x: f.origin.x + (dims.ph) * f.c - (0mm) * f.s,
                    y: f.origin.y + (dims.ph) * f.s + (0mm) * f.c)
  s0 midpoint skirt
  // the sizes a printer needs
  claim cL distance(2 * pw) cR
  claim cL distance(dims.ph) sL
  claim cL distance(dims.groove) g0L
  claim g1R distance(groovew) g2R
  claim g1L distance(grooveb) g1R
  claim f.origin distance(dims.L) f.toward
  claim radius(dims.reye) eye
  claim radius(dims.pinclr / 2) hole
  claim ra distance(dims.rt) rc

  // -- the solid: the section's faces swept, and the body their one rule (§6.9) ----------------
  // The rod and the eye are half `rw` either side of the plane of swing, which is where the
  // section is drawn and where the crank pin's washers hold it; the piston needs no such
  // statement, being a turn about a line that lies in that plane.
  solid pist(face(f.origin, pL0, pL1, pL2, pL3, pL4, s0, -> close), about: rod)
  // the rod between its flanks, closed across the skirt and across the eye
  solid shank(face(fl, rd, fr, ra), from: -dims.rw / 2, to: dims.rw / 2)
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
  point pin hint(x: std.up.origin.x + (-L) * std.up.c - (0mm) * std.up.s,
                    y: std.up.origin.y + (-L) * std.up.s + (0mm) * std.up.c)
  std.origin vertical pin
  std.origin distance(L) pin
  plane piston_axes(origin: std.origin, toward: pin)
  pis: Piston(piston_axes, dims: vtwin_dims)
}
