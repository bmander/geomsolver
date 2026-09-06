// Reusable cylinder: section geometry, solid and dimension claims.
// Instantiated by Bank in the assembly; the preview below sets up the part drawing.

use std
use components.dims
use components.parts

// f is the moving datum: origin at the pivot, u toward the head, v to the left.
// The caller supplies plane membership with `in`; fw is the plate-side wall thickness.
component Cylinder(f: plane, fw: Length) {
  line axis(f.origin, f.toward)
  param mouth_u = cb - H
  param head_u = head - H
  param top_u = ct - H

  // Depths normal to the section, measured from the bore centre plane.
  param face = -(fw + D / 2)
  param back = D / 2 + wall

  // Body outline, from the open mouth to the head.
  k_bl: Loc(f, u: mouth_u, v: hw)
  k_br: Loc(f, u: mouth_u, v: -hw)
  k_tr: Loc(f, u: top_u, v: -hw)
  k_tl: Loc(f, u: top_u, v: hw)
  line mouth(k_bl.p, k_br.p) -> line side_r(k_br.p, k_tr.p) -> line lid(k_tr.p, k_tl.p) ->
    line side_l(k_tl.p, k_bl.p) -> close

  // Both bore walls appear in section; only one half is revolved into the cut.
  b_bl: Loc(f, u: mouth_u, v: D / 2)
  b_br: Loc(f, u: mouth_u, v: -D / 2)
  b_tr: Loc(f, u: head_u, v: -D / 2)
  b_tl: Loc(f, u: head_u, v: D / 2)
  line bore_l(b_bl.p, b_tl.p)
  line bore_r(b_br.p, b_tr.p)
  line hd(b_tl.p, b_tr.p)
  m0: Loc(f, u: mouth_u, v: 0mm)
  hx: Loc(f, u: head_u, v: 0mm)

  // The air port and pivot shank enter through the plate-side face.
  pt: Loc(f, u: a, v: 0mm)
  circle port(center: pt.p) hint(r: dport / 2)
  radius(dport / 2) port
  circle shank(center: f.origin) hint(r: trapfit / 2)
  radius(trapfit / 2) shank

  // The bolt head slides in from the left; the slot holds it against the face wall.
  pkt: Hex(f.origin, axis, af: boltaf, phase: 90deg)
  t0: Loc(f, u: trapw / 2, v: hw)
  t1: Loc(f, u: trapw / 2, v: -trapd)
  t2: Loc(f, u: -trapw / 2, v: -trapd)
  t3: Loc(f, u: -trapw / 2, v: hw)
  line trap0(t0.p, t1.p)
  line trap1(t1.p, t2.p)
  line trap2(t2.p, t3.p)

  // Witness point for the head and side wall thicknesses.
  h0: Loc(f, u: top_u, v: D / 2)

  // Part dimensions, checked against the placed geometry.
  claim k_bl.p distance(ct - cb) k_tl.p
  claim b_br.p distance(head - cb) b_tr.p
  claim b_tl.p distance(D) b_tr.p
  claim k_tl.p distance(2 * hw) k_tr.p
  claim f.origin distance(a) pt.p
  claim m0.p distance(H - cb) f.origin
  claim b_tl.p distance(wall) h0.p
  claim k_tl.p distance(hw - D / 2) h0.p
  claim radius(dport / 2) port
  claim radius(trapfit / 2) shank
  claim t0.p distance(trapw) t3.p
  claim t1.p distance(hw + trapd) t0.p

  // Sweep the body, turn the bore, then cut the port, shank hole and head slot.
  solid block(face(mouth, side_r, lid, side_l), from: face, to: back)
  solid bore(face(m0.p, b_br.p, bore_r, hx.p, -> close), about: axis)
  solid passage(face(port), from: face, to: 0mm)
  solid hole(face(shank), from: face, to: face + trapz)
  solid trap(face(trap0, trap1, trap2, -> close), from: face + trapz, to: face + trapz + traph)
  solid body(block)
  bore cut body
  passage cut body
  hole cut body
  trap cut body
}

// Open this file to preview bank A upright; use fwB for bank B's thicker face wall.
// ../cylinder.svd arranges the three projections of this same preview.
preview {
  unit mm
  point O hint(x: 0, y: 0) in front
  ground O
  point datum hint(x: 1, y: 0)
  ground datum
  plane front(origin: O, toward: datum)
  axes: Axes(O) in front
  cyl: Cylinder(axes.f, fw: fwA) in front
}
