// The piston and its rod, one printed part, designed in one place: three views (§6.7).
//
// In the plane of swing: the piston, `D` less a clearance, its crown square to the rod, the
// O-ring groove below the crown and the skirt below that; the rod down to the eye about the
// crank pin, with the hole the clevis pin's shank rides in.  From the side: the piston is round,
// so it is as wide as it is across, and the rod and eye are `rw` thick.  From the crown: a disc.
// The assembly instances it once a bank, its crown `L` from the pin along a rod that passes
// through the cylinder's pivot; the part sheet instances it once, upright.  The O-ring is a
// #014 and the groove is sized to it in `dims.sv`.  Print it crown down.

use vtwin.dims
use vtwin.parts

component Piston(swing: plane, side: plane, top: plane, crown: point, rod: line, dir: Angle,
                 pin: point, o_s: point, o_t: point, draw_side: Int, draw_top: Int) {
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
    ra distance(rt / 2) rod
    rb distance(rt / 2) rod
    rc distance(-rt / 2) rod
    rd distance(-rt / 2) rod
    line fl(ra, rb)
    line fr(rc, rd)
    // the far end of the eye, which the side view reads
    eb: Loc(crown, rod, crownl, dir: dir, u: -(L + reye), v: 0mm)
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

  // -- from the side: round, so as wide as across; the rod `rw` thick -------------------------
  repeat draw_side {
    in side {
      point crown_s hint(x: o_s.x, y: crown.y)
      point g0_s hint(x: o_s.x, y: crown.y - groove)
      point g2_s hint(x: o_s.x, y: crown.y - groove - groovew)
      point skirt_s hint(x: o_s.x, y: crown.y - ph)
      point pin_s hint(x: o_s.x, y: crown.y - L)
      point eb_s hint(x: o_s.x, y: crown.y - L - reye)
      o_s distance(0, along: x) crown_s
      o_s distance(0, along: x) g0_s
      o_s distance(0, along: x) g2_s
      o_s distance(0, along: x) skirt_s
      o_s distance(0, along: x) pin_s
      o_s distance(0, along: x) eb_s
      upper: Slab(o_s, x0: -pw, x1: pw, top: crown_s, bottom: g0_s)
      grv: Slab(o_s, x0: -gb, x1: gb, top: g0_s, bottom: g2_s)
      lower: Slab(o_s, x0: -pw, x1: pw, top: g2_s, bottom: skirt_s)
      shank: Slab(o_s, x0: -rw / 2, x1: rw / 2, top: skirt_s, bottom: eb_s)
      ph_s: Box(pin_s, x0: -rw / 2, y0: -pinclr / 2, x1: rw / 2, y1: pinclr / 2) class hidden
      claim shank.a distance(rw) shank.b class detail at (0, -6)
    }
    crown project crown_s
    g0L.p project g0_s
    g2L.p project g2_s
    sL.p project skirt_s
    pin project pin_s
    eb.p project eb_s
  }

  // -- from the crown: a disc, the rod's section under it ----------------------------------------
  repeat draw_top {
    in top {
      circle disc(center: o_t) hint(r: pw)
      radius(pw) disc
      sec: Box(o_t, x0: -rt / 2, y0: -rw / 2, x1: rt / 2, y1: rw / 2) class hidden
    }
  }
}
