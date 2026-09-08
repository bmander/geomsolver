// Reusable cylinder: section geometry, solid and dimension claims.
// Instantiated by Bank in the assembly; the preview below sets up the part drawing.

use std
use components.dims

// f is the moving datum: origin at the pivot, u toward the head, v to the left.
// The caller supplies plane membership with `in`; fw is the plate-side wall thickness.
component Cylinder(f: plane, fw: Length, dims: group) {
  line axis(f.origin, f.toward)
  // The skirt reaches the bore mouth at bottom dead centre.
  param mouth_u = dims.L - dims.R - dims.ph - dims.H
  param hw = dims.D / 2 + dims.wall
  param head_u = dims.head - dims.H
  param top_u = head_u + dims.wall
  param trap_clearance = 0.3mm
  param trapw = dims.boltaf + trap_clearance
  param trapd = dims.boltac / 2 + trap_clearance

  // Depths normal to the section, measured from the bore centre plane.
  param face = -(fw + dims.D / 2)
  param back = dims.D / 2 + dims.wall

  // Body outline, from the open mouth to the head.
  point k_bl hint(x: f.origin.x + (mouth_u) * f.c - (hw) * f.s,
                    y: f.origin.y + (mouth_u) * f.s + (hw) * f.c)
  point k_br hint(x: f.origin.x + (mouth_u) * f.c - (-hw) * f.s,
                    y: f.origin.y + (mouth_u) * f.s + (-hw) * f.c)
  point k_tr hint(x: f.origin.x + (top_u) * f.c - (-hw) * f.s,
                    y: f.origin.y + (top_u) * f.s + (-hw) * f.c)
  point k_tl hint(x: f.origin.x + (top_u) * f.c - (hw) * f.s,
                    y: f.origin.y + (top_u) * f.s + (hw) * f.c)
  line mouth(k_bl, k_br) -> line side_r(k_br, k_tr) -> line lid(k_tr, k_tl) ->
    line side_l(k_tl, k_bl) -> close

  // Both bore walls appear in section; only one half is revolved into the cut.
  point b_bl hint(x: f.origin.x + (mouth_u) * f.c - (dims.D / 2) * f.s,
                    y: f.origin.y + (mouth_u) * f.s + (dims.D / 2) * f.c)
  point b_br hint(x: f.origin.x + (mouth_u) * f.c - (-dims.D / 2) * f.s,
                    y: f.origin.y + (mouth_u) * f.s + (-dims.D / 2) * f.c)
  point b_tr hint(x: f.origin.x + (head_u) * f.c - (-dims.D / 2) * f.s,
                    y: f.origin.y + (head_u) * f.s + (-dims.D / 2) * f.c)
  point b_tl hint(x: f.origin.x + (head_u) * f.c - (dims.D / 2) * f.s,
                    y: f.origin.y + (head_u) * f.s + (dims.D / 2) * f.c)
  line bore_l(b_bl, b_tl)
  line bore_r(b_br, b_tr)
  line hd(b_tl, b_tr)
  point m0 hint(x: f.origin.x + (mouth_u) * f.c - (0mm) * f.s,
                    y: f.origin.y + (mouth_u) * f.s + (0mm) * f.c)
  point hx hint(x: f.origin.x + (head_u) * f.c - (0mm) * f.s,
                    y: f.origin.y + (head_u) * f.s + (0mm) * f.c)

  // The air port and pivot shank enter through the plate-side face.
  point pt hint(x: f.origin.x + (dims.a) * f.c - (0mm) * f.s,
                    y: f.origin.y + (dims.a) * f.s + (0mm) * f.c)
  circle port(center: pt) hint(r: dims.dport / 2)
  radius(dims.dport / 2) port
  circle shank(center: f.origin) hint(r: dims.trapfit / 2)
  radius(dims.trapfit / 2) shank

  // The bolt head slides in from the left; the slot holds it against the face wall.
  pkt: Hex(f.origin, axis, af: dims.boltaf, phase: 90deg)
  point t0 hint(x: f.origin.x + (trapw / 2) * f.c - (hw) * f.s,
                    y: f.origin.y + (trapw / 2) * f.s + (hw) * f.c)
  point t1 hint(x: f.origin.x + (trapw / 2) * f.c - (-trapd) * f.s,
                    y: f.origin.y + (trapw / 2) * f.s + (-trapd) * f.c)
  point t2 hint(x: f.origin.x + (-trapw / 2) * f.c - (-trapd) * f.s,
                    y: f.origin.y + (-trapw / 2) * f.s + (-trapd) * f.c)
  point t3 hint(x: f.origin.x + (-trapw / 2) * f.c - (hw) * f.s,
                    y: f.origin.y + (-trapw / 2) * f.s + (hw) * f.c)
  line trap0(t0, t1)
  line trap1(t1, t2)
  line trap2(t2, t3)

  // Witness point for the head and side wall thicknesses.
  point h0 hint(x: f.origin.x + (top_u) * f.c - (dims.D / 2) * f.s,
                    y: f.origin.y + (top_u) * f.s + (dims.D / 2) * f.c)

  // The outside is a rectangle centered on the bore axis. One axial dimension locates its mouth.
  mouth perpendicular axis
  lid perpendicular axis
  side_l parallel axis
  side_r parallel axis
  distance(2 * hw) mouth
  distance(top_u - mouth_u) side_l
  m0 midpoint mouth
  m0 on axis
  m0 distance(mouth_u, along: u) f
  // The bore opens at that mouth and leaves the specified side and head walls.
  b_bl on mouth
  b_br on mouth
  bore_l parallel axis
  bore_r parallel axis
  hd perpendicular axis
  k_bl distance(dims.wall) b_bl
  k_br distance(dims.wall) b_br
  b_tl distance(dims.wall) lid
  hx midpoint hd
  pt on axis
  f.origin distance(dims.a) pt
  // The head slot is rectangular and opens on the left body wall.
  trap0 perpendicular axis
  trap1 parallel axis
  trap2 perpendicular axis
  t0 on side_l
  t3 on side_l
  distance(trapw) trap1
  t1 distance(trapd, side: right) axis
  t0 distance(trapw / 2, along: u) f
  h0 on lid
  h0 on bore_l

  claim k_bl distance(top_u - mouth_u) k_tl
  claim b_br distance(head_u - mouth_u) b_tr
  claim b_tl distance(dims.D) b_tr
  claim k_tl distance(2 * hw) k_tr
  claim f.origin distance(dims.a) pt
  claim m0 distance(-mouth_u) f.origin
  claim b_tl distance(dims.wall) h0
  claim k_tl distance(hw - dims.D / 2) h0
  claim radius(dims.dport / 2) port
  claim radius(dims.trapfit / 2) shank
  claim t0 distance(trapw) t3
  claim t1 distance(hw + trapd) t0

  // Sweep the body, turn the bore, then cut the port, shank hole and head slot.
  solid block(face(mouth, side_r, lid, side_l), from: face, to: back)
  solid bore(face(m0, b_br, bore_r, hx, -> close), about: axis)
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
