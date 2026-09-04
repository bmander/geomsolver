// The cylinder, designed in one place: **one section, and the solid it is a section of** (§6.9).
//
// This part used to be written three times — the section, and then the same body redrawn from
// nothing in two more views as page-aligned rectangles, re-tied to it by five projections, with
// every depth ordinate (`tcyl / 2`, `-fw`, `trapz`, `traph`) related to the section by no
// statement at all.  It is now written once.  The outline in the plane of swing is the design;
// the body is that outline swept a face wall `fw` toward the plate and a `wall` the other way,
// the bore is its own half-section turned about the rod's line, and the port, the shank hole and
// the head's slot are three more solids `through` it.  Every view of it — and the file a printer
// wants — is a question asked of that solid, so a depth cannot be right in one picture and wrong
// in another.
//
// `vtwin.sv` instances it twice, each rocked to the crank, and draws the section only;
// `vtwin_cylinder.sv` instances it once, upright, and asks for the side and head-end views.  Both
// are this one definition — and the dimensions a printer needs are written here, once,
// `class detail`: the part sheet shows them and the assembly's sheet leaves them hidden, which is
// the whole difference between the two drawings.
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

component Cylinder(swing: plane, piv: point, ax: line, ac: line, dir: Angle, fw: Length) {
  param tcyl = fw + D + wall
  // **the depths, once.**  Ordinates along the plane of swing's own normal, which passes down
  // the bore's axis: the plate-side face a face wall and half a bore away, the far side a wall
  // and half a bore the other way.  Every solid below is written in these, and no view is.
  param face = -(fw + D / 2)
  param back = D / 2 + wall

  // -- the section in the plane of swing: the design ------------------------------------------
  in swing {
    // the body, mouth to head, written in the cylinder's own frame (`Loc`)
    k_bl: Loc(piv, ax, ac, dir: dir, u: cb - H, v: hw)
    k_br: Loc(piv, ax, ac, dir: dir, u: cb - H, v: -hw)
    k_tr: Loc(piv, ax, ac, dir: dir, u: ct - H, v: -hw)
    k_tl: Loc(piv, ax, ac, dir: dir, u: ct - H, v: hw)
    line mouth(k_bl.p, k_br.p) -> line side_r(k_br.p, k_tr.p) -> line lid(k_tr.p, k_tl.p) ->
      line side_l(k_tl.p, k_bl.p) -> close
    face sec(mouth, side_r, lid, side_l)
    // the bore.  Both walls are drawn, because the section shows them; what is *turned* is the
    // half between the axis and one of them, since a revolution takes one side of its axis
    b_bl: Loc(piv, ax, ac, dir: dir, u: cb - H, v: D / 2)
    b_br: Loc(piv, ax, ac, dir: dir, u: cb - H, v: -D / 2)
    b_tr: Loc(piv, ax, ac, dir: dir, u: head - H, v: -D / 2)
    b_tl: Loc(piv, ax, ac, dir: dir, u: head - H, v: D / 2)
    line bore_l(b_bl.p, b_tl.p)
    line bore_r(b_br.p, b_tr.p)
    line hd(b_tl.p, b_tr.p)
    m0: Loc(piv, ax, ac, dir: dir, u: cb - H, v: 0mm)
    hx: Loc(piv, ax, ac, dir: dir, u: head - H, v: 0mm)
    face bore_f(m0.p, b_br.p, bore_r, hx.p, -> close)
    // the port, `a` up from the bolt: drilled from the face into the top of the bore, so seen
    // end on here; and the bolt's head, trapped in its slot in the face wall beside the bore — the
    // hex (`std`'s `Hex`) with its flats along the bore's axis, and the slot from the body's left
    // side in past the axis.  The slot only carries the tension: the shank is located by the
    // hole through the wall to the face, a fit it does not turn in, and the cylinder's attitude
    // by its face on the plate.
    pt: Loc(piv, ax, ac, dir: dir, u: a, v: 0mm)
    circle port(center: pt.p) hint(r: dport / 2) class hidden
    radius(dport / 2) port
    face port_f(port)
    circle shank(center: piv) hint(r: trapfit / 2) class hidden
    radius(trapfit / 2) shank
    face shank_f(shank)
    pkt: Hex(piv, ax, af: boltaf, phase: 90deg) class hidden
    t0: Loc(piv, ax, ac, dir: dir, u: trapw / 2, v: hw)
    t1: Loc(piv, ax, ac, dir: dir, u: trapw / 2, v: -trapd)
    t2: Loc(piv, ax, ac, dir: dir, u: -trapw / 2, v: -trapd)
    t3: Loc(piv, ax, ac, dir: dir, u: -trapw / 2, v: hw)
    line trap0(t0.p, t1.p) class hidden
    line trap1(t1.p, t2.p) class hidden
    line trap2(t2.p, t3.p) class hidden
    face trap_f(trap0, trap1, trap2, -> close)
    // one more point on the outline, for the head wall to be measured to
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
    claim radius(trapfit / 2) shank class detail at (0.6, 5)
    claim t0.p distance(trapw) t3.p class detail at (0, 6)
    claim t1.p distance(hw + trapd) t0.p class detail at (0, -6)
  }

  // -- the solid: the section's faces swept, and the body their one rule (§6.9) ----------------
  // Every extent is an expression over the table above and no solve moves one; the order lives
  // inside the term and not between these statements, so they may be written in any order.
  solid block(sec, from: face, to: back)
  solid bore(bore_f, about: ax)
  solid passage(port_f, from: face, to: 0mm)
  solid hole(shank_f, from: face, to: face + trapz)
  solid trap(trap_f, from: face + trapz, to: face + trapz + traph)
  solid body(block)
  bore through body
  passage through body
  hole through body
  trap through body
}
