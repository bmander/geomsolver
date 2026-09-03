// The cylinder, designed in one place: one component, three views (§6.7).
//
// The section in its plane of swing is what the assembly draws, rocked to the crank; edge on,
// from the side, the face wall shows with the bolt's pocket and the port passage drilled through
// it; from the head end, the bore sits a wall's thickness off the face.  `vtwin.sv` instances it
// twice and draws the section only; `vtwin_cylinder.sv` instances it once, upright, in all three
// views.  Both are this one definition — and the dimensions a printer needs are written here,
// once, `class detail`: the part sheet shows them and the assembly's sheet leaves them hidden,
// which is the whole difference between the two drawings.
//
// The body is a block: the bore `D` down its middle in the plane of swing but a wall `fw` thick
// off the face, since the face wall carries the pivot bolt — its hex head trapped in a slot cut
// into the wall from the cylinder's side, `trapz` behind the face, the shank out through a hole
// in the face: the bolt's tension pulls the head against the wall between the slot and the
// face, plastic in compression, and the slot's width stops the hex turning when the nut is
// tightened.  The bolt goes in sideways before the cylinder goes on the plate.  (Bank B's wall
// is a rod thicker than A's, so the two rods sit side by side on the pin.)  The mouth is open at
// the bottom, the head `wall` thick, and the port is drilled from the face into the top of the
// bore, `a` up from the bolt.  Print it mouth down.

use std
use vtwin.dims
use vtwin.parts

component Cylinder(swing: plane, side: plane, top: plane, piv: point, ax: line, ac: line,
                   dir: Angle, fw: Length, o_s: point, o_t: point, draw_side: Int, draw_top: Int) {
  param tcyl = fw + D + wall

  // -- in the plane of swing: the section through the bore ------------------------------------
  in swing {
    // the body, mouth to head, written in the cylinder's own frame (`Loc`)
    k_bl: Loc(piv, ax, ac, dir: dir, u: cb - H, v: hw)
    k_br: Loc(piv, ax, ac, dir: dir, u: cb - H, v: -hw)
    k_tr: Loc(piv, ax, ac, dir: dir, u: ct - H, v: -hw)
    k_tl: Loc(piv, ax, ac, dir: dir, u: ct - H, v: hw)
    line mouth(k_bl.p, k_br.p) -> line side_r(k_br.p, k_tr.p) -> line lid(k_tr.p, k_tl.p) ->
      line side_l(k_tl.p, k_bl.p) -> close
    // the bore
    b_bl: Loc(piv, ax, ac, dir: dir, u: cb - H, v: D / 2)
    b_br: Loc(piv, ax, ac, dir: dir, u: cb - H, v: -D / 2)
    b_tr: Loc(piv, ax, ac, dir: dir, u: head - H, v: -D / 2)
    b_tl: Loc(piv, ax, ac, dir: dir, u: head - H, v: D / 2)
    line bore_l(b_bl.p, b_tl.p)
    line bore_r(b_br.p, b_tr.p)
    line hd(b_tl.p, b_tr.p)
    // the port, `a` up from the bolt: drilled from the face into the top of the bore, so seen
    // end on here; and the bolt's head, trapped in its slot in the face wall beside the bore — the
    // hex (`std`'s `Hex`) with its flats along the bore's axis, and the slot from the body's left
    // side in past the axis.  The slot only carries the tension: the shank is located by the
    // hole through the wall to the face, a fit it does not turn in, and the cylinder's attitude
    // by its face on the plate.
    pt: Loc(piv, ax, ac, dir: dir, u: a, v: 0mm)
    circle port(center: pt.p) hint(r: dport / 2) class hidden
    radius(dport / 2) port
    pkt: Hex(piv, ax, af: boltaf, phase: 90deg) class hidden
    t0: Loc(piv, ax, ac, dir: dir, u: trapw / 2, v: hw)
    t1: Loc(piv, ax, ac, dir: dir, u: trapw / 2, v: -trapd)
    t2: Loc(piv, ax, ac, dir: dir, u: -trapw / 2, v: -trapd)
    t3: Loc(piv, ax, ac, dir: dir, u: -trapw / 2, v: hw)
    line trap0(t0.p, t1.p) class hidden
    line trap1(t1.p, t2.p) class hidden
    line trap2(t2.p, t3.p) class hidden
    // two more points on the outline, for the walls to be measured to
    m0: Loc(piv, ax, ac, dir: dir, u: cb - H, v: 0mm)
    h0: Loc(piv, ax, ac, dir: dir, u: ct - H, v: D / 2)
    // the sizes a printer needs, all judged: every point above is already placed
    claim k_bl.p distance(ct - cb) k_tl.p class detail at (0, 12)
    claim b_br.p distance(head - cb) b_tr.p class detail at (0, -12)
    claim b_tl.p distance(D) b_tr.p class detail at (0, 8)
    claim k_tl.p distance(2 * hw) k_tr.p class detail at (0, 18)
    claim piv distance(a) pt.p class detail at (0, 6)
    claim m0.p distance(H - cb) piv class detail at (0, 6)
    claim b_tl.p distance(wall) h0.p class detail at (0, 6)
    claim k_tl.p distance(hw - D / 2) h0.p class detail at (0, 6)
    claim radius(dport / 2) port class detail at (0.6, 9)
    claim t0.p distance(trapw) t3.p class detail at (0, 6)
    claim t1.p distance(hw + trapd) t0.p class detail at (0, -6)
  }

  // -- from the side: the face wall, the bolt's pocket and the port passage through it ----------
  // Page-x is depth, the face at `o_s`'s own x and the body running left of it; every height is
  // the section's, by projection.
  repeat draw_side {
    in side {
      point pv_s hint(x: o_s.x - tcyl / 2, y: o_s.y)
      point mo_s hint(x: o_s.x - tcyl / 2, y: o_s.y + cb - H)
      point hd_s hint(x: o_s.x - tcyl / 2, y: o_s.y + ct - H)
      point bt_s hint(x: o_s.x - tcyl / 2, y: o_s.y + head - H)
      point pp_s hint(x: o_s.x - tcyl / 2, y: o_s.y + a)
      o_s distance(-tcyl / 2, along: x) pv_s
      o_s distance(-tcyl / 2, along: x) mo_s
      o_s distance(-tcyl / 2, along: x) hd_s
      o_s distance(-tcyl / 2, along: x) bt_s
      o_s distance(-tcyl / 2, along: x) pp_s
      body: Slab(o_s, x0: -tcyl, x1: 0mm, top: hd_s, bottom: mo_s)
      bore: Slab(o_s, x0: -(fw + D), x1: -fw, top: bt_s, bottom: mo_s) class hidden
      trap_s: Box(pv_s, x0: tcyl / 2 - trapz - traph, y0: -trapw / 2, x1: tcyl / 2 - trapz, y1: trapw / 2) class hidden
      hole_s: Box(pv_s, x0: tcyl / 2 - trapz, y0: -trapfit / 2, x1: tcyl / 2, y1: trapfit / 2) class hidden
      passage: Box(pp_s, x0: tcyl / 2 - fw, y0: -dport / 2, x1: tcyl / 2, y1: dport / 2) class hidden
      // the bore's axis, the face wall and half a bore off the face
      bax: At(pv_s, dx: tcyl / 2 - fw - D / 2, dy: 0mm)
      fc: At(pv_s, dx: tcyl / 2, dy: 0mm)
      claim body.a distance(tcyl) body.b class detail at (0, -8)
      claim bax.p distance(fw + D / 2, along: x) fc.p class detail at (0, -6)
      claim bore.b distance(fw, along: x) body.b class detail at (0, 6)
      claim trap_s.a distance(traph) trap_s.b class detail at (0, 10)
      claim hole_s.a distance(trapz) hole_s.b class detail at (0, -10)
      claim trap_s.a distance(trapw, along: y) trap_s.d class detail at (0, 6)
    }
    piv project pv_s
    k_bl.p project mo_s
    k_tl.p project hd_s
    b_tl.p project bt_s
    pt.p project pp_s
  }

  // -- from the head end: the body's width and depth, the bore under the head, the bolt's pocket
  // under the face ---------------------------------------------------------------------------
  // The face lies along `o_t`'s own y and the body runs down the page from it, toward the
  // section it is projected from; the widths are the section's.
  repeat draw_top {
    in top {
      point tl hint(x: o_t.x - hw, y: o_t.y)
      point tr hint(x: o_t.x + hw, y: o_t.y)
      point br hint(x: o_t.x + hw, y: o_t.y - tcyl)
      point bl hint(x: o_t.x - hw, y: o_t.y - tcyl)
      line e1(tl, tr) -> line e2(tr, br) -> line e3(br, bl) -> line e4(bl, tl) -> close
      o_t distance(0, along: y) tl
      o_t distance(0, along: y) tr
      vertical e2
      vertical e4
      o_t distance(-tcyl, along: y) br
      o_t distance(-tcyl, along: y) bl
      bc: At(o_t, dx: 0mm, dy: -(fw + D / 2))
      circle bore_t(center: bc.p) hint(r: D / 2) class hidden
      radius(D / 2) bore_t
      trap_t: Box(o_t, x0: -hw, y0: -(trapz + traph), x1: trapd, y1: -trapz) class hidden
      hole_t: Box(o_t, x0: -trapfit / 2, y0: -trapz, x1: trapfit / 2, y1: 0mm) class hidden
      claim hole_t.a distance(trapfit) hole_t.b class detail at (0, -5)
    }
    k_tl.p project tl
    k_tr.p project tr
  }
}
