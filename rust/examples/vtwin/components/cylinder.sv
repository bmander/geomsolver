// Reusable cylinder: section geometry, solid and dimension claims.
// Instantiated by Bank in the assembly; the preview below sets up the part drawing.

use std
use components.dims

// f is the moving datum: origin at the pivot, u toward the head, v to the left.
// The caller supplies plane membership with `in`; fw is the plate-side wall thickness.
component Cylinder(f: plane, fw: Length, dims: group) {
  axis := line(f.origin, f.toward)
  // The skirt reaches the bore mouth at bottom dead centre.
  mouth_u := dims.L - dims.R - dims.ph - dims.H
  hw := dims.D / 2 + dims.wall
  head_u := dims.head - dims.H
  top_u := head_u + dims.wall
  trap_clearance := 0.3mm
  trapw := dims.boltaf + trap_clearance
  trapd := dims.boltac / 2 + trap_clearance

  // Depths normal to the section, measured from the bore centre plane.
  face := -(fw + dims.D / 2)
  back := dims.D / 2 + dims.wall

  // Body outline, from the open mouth to the head.
  k_bl := point hint(x: f.origin.x + (mouth_u) * f.c - (hw) * f.s,
                    y: f.origin.y + (mouth_u) * f.s + (hw) * f.c)
  k_br := point hint(x: f.origin.x + (mouth_u) * f.c - (-hw) * f.s,
                    y: f.origin.y + (mouth_u) * f.s + (-hw) * f.c)
  k_tr := point hint(x: f.origin.x + (top_u) * f.c - (-hw) * f.s,
                    y: f.origin.y + (top_u) * f.s + (-hw) * f.c)
  k_tl := point hint(x: f.origin.x + (top_u) * f.c - (hw) * f.s,
                    y: f.origin.y + (top_u) * f.s + (hw) * f.c)
  (mouth := line(k_bl, k_br)) -> (side_r := line(k_br, k_tr)) -> (lid := line(k_tr, k_tl)) ->
    (side_l := line(k_tl, k_bl)) -> close

  // Both bore walls appear in section; only one half is revolved into the cut.
  b_bl := point hint(x: f.origin.x + (mouth_u) * f.c - (dims.D / 2) * f.s,
                    y: f.origin.y + (mouth_u) * f.s + (dims.D / 2) * f.c)
  b_br := point hint(x: f.origin.x + (mouth_u) * f.c - (-dims.D / 2) * f.s,
                    y: f.origin.y + (mouth_u) * f.s + (-dims.D / 2) * f.c)
  b_tr := point hint(x: f.origin.x + (head_u) * f.c - (-dims.D / 2) * f.s,
                    y: f.origin.y + (head_u) * f.s + (-dims.D / 2) * f.c)
  b_tl := point hint(x: f.origin.x + (head_u) * f.c - (dims.D / 2) * f.s,
                    y: f.origin.y + (head_u) * f.s + (dims.D / 2) * f.c)
  bore_l := line(b_bl, b_tl)
  bore_r := line(b_br, b_tr)
  hd := line(b_tl, b_tr)
  m0 := point hint(x: f.origin.x + (mouth_u) * f.c - (0mm) * f.s,
                    y: f.origin.y + (mouth_u) * f.s + (0mm) * f.c)
  hx := point hint(x: f.origin.x + (head_u) * f.c - (0mm) * f.s,
                    y: f.origin.y + (head_u) * f.s + (0mm) * f.c)

  // The air port and pivot shank enter through the plate-side face.
  pt := point hint(x: f.origin.x + (dims.a) * f.c - (0mm) * f.s,
                    y: f.origin.y + (dims.a) * f.s + (0mm) * f.c)
  port := circle(center: pt) hint(r: dims.dport / 2)
  radius(dims.dport / 2) port
  shank := circle(center: f.origin) hint(r: dims.trapfit / 2)
  radius(dims.trapfit / 2) shank

  // The bolt head slides in from the left; the slot holds it against the face wall.
  pkt := std.Hex(f.origin, axis, af: dims.boltaf, phase: 90deg)
  t0 := point hint(x: f.origin.x + (trapw / 2) * f.c - (hw) * f.s,
                    y: f.origin.y + (trapw / 2) * f.s + (hw) * f.c)
  t1 := point hint(x: f.origin.x + (trapw / 2) * f.c - (-trapd) * f.s,
                    y: f.origin.y + (trapw / 2) * f.s + (-trapd) * f.c)
  t2 := point hint(x: f.origin.x + (-trapw / 2) * f.c - (-trapd) * f.s,
                    y: f.origin.y + (-trapw / 2) * f.s + (-trapd) * f.c)
  t3 := point hint(x: f.origin.x + (-trapw / 2) * f.c - (hw) * f.s,
                    y: f.origin.y + (-trapw / 2) * f.s + (hw) * f.c)
  trap0 := line(t0, t1)
  trap1 := line(t1, t2)
  trap2 := line(t2, t3)

  // Witness point for the head and side wall thicknesses.
  h0 := point hint(x: f.origin.x + (top_u) * f.c - (dims.D / 2) * f.s,
                    y: f.origin.y + (top_u) * f.s + (dims.D / 2) * f.c)

  // The outside is a rectangle centered on the bore axis. One axial dimension locates its mouth.
  mouth perpendicular axis
  lid perpendicular axis
  side_l parallel axis
  side_r parallel axis
  distance(2 * hw) mouth
  distance(top_u - mouth_u) side_l
  m0 midpoint mouth
  m0 coincident axis
  m0 distance(mouth_u, along: u) f
  // The bore opens at that mouth and leaves the specified side and head walls.
  b_bl coincident mouth
  b_br coincident mouth
  bore_l parallel axis
  bore_r parallel axis
  hd perpendicular axis
  k_bl distance(dims.wall) b_bl
  k_br distance(dims.wall) b_br
  b_tl distance(dims.wall) lid
  hx midpoint hd
  pt coincident axis
  f.origin distance(dims.a) pt
  // The head slot is rectangular and opens on the left body wall.
  trap0 perpendicular axis
  trap1 parallel axis
  trap2 perpendicular axis
  t0 coincident side_l
  t3 coincident side_l
  distance(trapw) trap1
  t1 distance(trapd, side: right) axis
  t0 distance(trapw / 2, along: u) f
  h0 coincident lid
  h0 coincident bore_l

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
  block := solid(face(mouth, side_r, lid, side_l), from: face, to: back)
  bore := solid(face(m0, b_br, bore_r, hx, -> close), about: axis)
  passage := solid(face(port), from: face, to: 0mm)
  hole := solid(face(shank), from: face, to: face + dims.trapz)
  trap := solid(face(trap0, trap1, trap2, -> close), from: face + dims.trapz, to: face + dims.trapz + dims.traph)
  body := solid(block)
  bore cut body
  passage cut body
  hole cut body
  trap cut body
}

// Open this file to preview bank A upright; use fwB for bank B's thicker face wall.
// ../cylinder.svd arranges the three projections of this same preview.
preview {
  unit mm
  cyl := Cylinder(std.up, fw: components.dims.fwA, dims: components.dims.vtwin_dims)
}
