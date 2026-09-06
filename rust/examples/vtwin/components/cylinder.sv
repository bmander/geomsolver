// Reusable cylinder: section geometry, solid and dimension claims.
// Instantiated by Bank in the assembly; the preview below sets up the part drawing.

use std
use components.dims

// f is the moving datum: origin at the pivot, u toward the head, v to the left.
// The caller supplies plane membership with `in`; fw is the plate-side wall thickness.
component Cylinder(f: plane, fw: Length, dims: group) {
  line axis(f.origin, f.toward)
  param mouth_u = dims.cb - dims.H
  param head_u = dims.head - dims.H
  param top_u = dims.ct - dims.H

  // Depths normal to the section, measured from the bore centre plane.
  param face = -(fw + dims.D / 2)
  param back = dims.D / 2 + dims.wall

  // Body outline, from the open mouth to the head.
  k_bl: Loc(f, u: mouth_u, v: dims.hw)
  k_br: Loc(f, u: mouth_u, v: -dims.hw)
  k_tr: Loc(f, u: top_u, v: -dims.hw)
  k_tl: Loc(f, u: top_u, v: dims.hw)
  line mouth(k_bl.p, k_br.p) -> line side_r(k_br.p, k_tr.p) -> line lid(k_tr.p, k_tl.p) ->
    line side_l(k_tl.p, k_bl.p) -> close

  // Both bore walls appear in section; only one half is revolved into the cut.
  b_bl: Loc(f, u: mouth_u, v: dims.D / 2)
  b_br: Loc(f, u: mouth_u, v: -dims.D / 2)
  b_tr: Loc(f, u: head_u, v: -dims.D / 2)
  b_tl: Loc(f, u: head_u, v: dims.D / 2)
  line bore_l(b_bl.p, b_tl.p)
  line bore_r(b_br.p, b_tr.p)
  line hd(b_tl.p, b_tr.p)
  m0: Loc(f, u: mouth_u, v: 0mm)
  hx: Loc(f, u: head_u, v: 0mm)

  // The air port and pivot shank enter through the plate-side face.
  pt: Loc(f, u: dims.a, v: 0mm)
  circle port(center: pt.p) hint(r: dims.dport / 2)
  radius(dims.dport / 2) port
  circle shank(center: f.origin) hint(r: dims.trapfit / 2)
  radius(dims.trapfit / 2) shank

  // The bolt head slides in from the left; the slot holds it against the face wall.
  pkt: Hex(f.origin, axis, af: dims.boltaf, phase: 90deg)
  t0: Loc(f, u: dims.trapw / 2, v: dims.hw)
  t1: Loc(f, u: dims.trapw / 2, v: -dims.trapd)
  t2: Loc(f, u: -dims.trapw / 2, v: -dims.trapd)
  t3: Loc(f, u: -dims.trapw / 2, v: dims.hw)
  line trap0(t0.p, t1.p)
  line trap1(t1.p, t2.p)
  line trap2(t2.p, t3.p)

  // Witness point for the head and side wall thicknesses.
  h0: Loc(f, u: top_u, v: dims.D / 2)

  // Part dimensions, checked against the placed geometry.
  claim k_bl.p distance(dims.ct - dims.cb) k_tl.p
  claim b_br.p distance(dims.head - dims.cb) b_tr.p
  claim b_tl.p distance(dims.D) b_tr.p
  claim k_tl.p distance(2 * dims.hw) k_tr.p
  claim f.origin distance(dims.a) pt.p
  claim m0.p distance(dims.H - dims.cb) f.origin
  claim b_tl.p distance(dims.wall) h0.p
  claim k_tl.p distance(dims.hw - dims.D / 2) h0.p
  claim radius(dims.dport / 2) port
  claim radius(dims.trapfit / 2) shank
  claim t0.p distance(dims.trapw) t3.p
  claim t1.p distance(dims.hw + dims.trapd) t0.p

  // Sweep the body, turn the bore, then cut the port, shank hole and head slot.
  solid block(face(mouth, side_r, lid, side_l), from: face, to: back)
  solid bore(face(m0.p, b_br.p, bore_r, hx.p, -> close), about: axis)
  solid passage(face(port), from: face, to: 0mm)
  solid hole(face(shank), from: face, to: face + dims.trapz)
  solid trap(face(trap0, trap1, trap2, -> close), from: face + dims.trapz, to: face + dims.trapz + dims.traph)
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
  cyl: Cylinder(std.up, fw: fwA, dims: vtwin_dims)
}
