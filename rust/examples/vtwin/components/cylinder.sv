// Reusable cylinder: section geometry, solid and dimension claims.
// Instantiated by Bank in the assembly; the preview below sets up the part drawing.

use std (on_u)
use components.dims (D, H, L, R, a, boltac, boltaf, dport, head, ph, trapfit, traph, trapz, wall)

// The cylinder stands in its own frame (#542): origin at the pivot, z toward the head.  The part is
// drawn in the frame's `std.up`, z to the right and x to the left, so every number below is the
// part's own and the frame is all that moves; fw is the plate-side wall thickness.
component Cylinder(fw: Length) {
  // The skirt reaches the bore mouth at bottom dead centre.
  mouth_u := L - R - ph - H
  hw := D / 2 + wall
  head_u := head - H
  top_u := head_u + wall
  trap_clearance := 0.3mm
  trapw := boltaf + trap_clearance
  trapd := boltac / 2 + trap_clearance

  // Depths normal to the section, measured from the bore centre plane.
  face := -(fw + D / 2)
  back := D / 2 + wall

  in std.up {
    // The bore's axis, through the pivot and the air port `a` along it.
    pt := point hint((a, 0mm))
    pt on_u(d: a) std.up
    ax := line(std.up.origin, pt)

    // Body outline, from the open mouth to the head.
    k_bl := point hint((mouth_u, hw))
    k_br := point hint((mouth_u, -hw))
    k_tr := point hint((top_u, -hw))
    k_tl := point hint((top_u, hw))
    (mouth := line(k_bl, k_br)) -> (side_r := line(k_br, k_tr)) -> (lid := line(k_tr, k_tl)) ->
      (side_l := line(k_tl, k_bl)) -> close

    // Both bore walls appear in section; only one half is revolved into the cut.
    b_bl := point hint((mouth_u, D / 2))
    b_br := point hint((mouth_u, -D / 2))
    b_tr := point hint((head_u, -D / 2))
    b_tl := point hint((head_u, D / 2))
    bore_l := line(b_bl, b_tl)
    bore_r := line(b_br, b_tr)
    hd := line(b_tl, b_tr)
    m0 := point hint((mouth_u, 0mm))
    hx := point hint((head_u, 0mm))

    // The air port and pivot shank enter through the plate-side face.
    port := circle(center: pt) hint(r: dport / 2)
    radius(dport / 2) port
    shank := circle(center: std.up.origin) hint(r: trapfit / 2)
    radius(trapfit / 2) shank

    // The bolt head slides in from the left; the slot holds it against the face wall.
    pkt := std.Hex(std.up.origin, ax, af: boltaf, phase: 90deg)
    t0 := point hint((trapw / 2, hw))
    t1 := point hint((trapw / 2, -trapd))
    t2 := point hint((-trapw / 2, -trapd))
    t3 := point hint((-trapw / 2, hw))
    trap0 := line(t0, t1)
    trap1 := line(t1, t2)
    trap2 := line(t2, t3)

    // Witness point for the head and side wall thicknesses.
    h0 := point hint((top_u, D / 2))

    // The outside is a rectangle centered on the bore axis. One axial dimension locates its mouth.
    mouth perpendicular ax
    lid perpendicular ax
    side_l parallel ax
    side_r parallel ax
    length(2 * hw) mouth
    length(top_u - mouth_u) side_l
    m0 midpoint mouth
    m0 coincident ax
    m0 distance(mouth_u, along: u) std.up
    // The bore opens at that mouth and leaves the specified side and head walls.
    b_bl coincident mouth
    b_br coincident mouth
    bore_l parallel ax
    bore_r parallel ax
    hd perpendicular ax
    k_bl distance(wall) b_bl
    k_br distance(wall) b_br
    b_tl distance(wall) lid
    hx midpoint hd
    // The head slot is rectangular and opens on the left body wall.
    trap0 perpendicular ax
    trap1 parallel ax
    trap2 perpendicular ax
    t0 coincident side_l
    t3 coincident side_l
    length(trapw) trap1
    t1 distance(trapd, side: right) ax
    t0 distance(trapw / 2, along: u) std.up
    h0 coincident lid
    h0 coincident bore_l
    // which way round each of those is: the left wall on the left, the head beyond the mouth, the
    // bore inside the walls and below the lid, and the slot running in from the left wall
    k_br distance(>= 0, along: v) k_bl
    k_bl distance(>= 0, along: u) k_tl
    b_bl distance(>= 0, along: v) k_bl
    k_br distance(>= 0, along: v) b_br
    b_tl distance(>= 0, along: u) k_tl
    t2 distance(>= 0, along: u) t1

    claim k_bl distance(top_u - mouth_u) k_tl
    claim b_br distance(head_u - mouth_u) b_tr
    claim b_tl distance(D) b_tr
    claim k_tl distance(2 * hw) k_tr
    claim std.up.origin distance(a) pt
    claim m0 distance(-mouth_u) std.up.origin
    claim b_tl distance(wall) h0
    claim k_tl distance(hw - D / 2) h0
    claim radius(dport / 2) port
    claim radius(trapfit / 2) shank
    claim t0 distance(trapw) t3
    claim t1 distance(hw + trapd) t0
  }

  // Sweep the body, turn the bore, then cut the port, shank hole and head slot.
  block := solid(face(mouth, side_r, lid, side_l), from: face, to: back)
  bore := solid(face(m0, b_br, bore_r, hx, -> close), about: ax)
  passage := solid(face(port), from: face, to: 0mm)
  hole := solid(face(shank), from: face, to: face + trapz)
  trap := solid(face(trap0, trap1, trap2, -> close), from: face + trapz, to: face + trapz + traph)
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
  // upright: its frame the world's, so the cylinder's z is the world's and its section the
  // world's up plane
  in (f := frame) {
    cyl := Cylinder(fw: components.dims.fwA)
  }
  fix(dir == (1, 0, 0)) f.u
  fix(dir == (0, 1, 0)) f.v
  fix(origin == (0, 0, 0)) f
}
