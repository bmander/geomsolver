// A dog-bone plate with a notch in each waist, drawn tangent all the way round, extruded with a
// draft and its far rim rounded. The ends are convex arcs of 15 about (±24, 0). Each waist is two
// concave arcs of 30 (centres (±3, ±36)) running down into a concave fillet of 4 at the notch's
// root, on the y axis. Every seam is an arc meeting an arc: convex into concave at the ends,
// concave into concave at the notches.
//
// Drafted, each side is a cone about its arc's centre, and the round along the far rim is a
// stretch of a ring about each: eight tori, each going on from the next across a seam where the
// two cones meet tangent along a line. There is no edge there, so no corner: each stretch ends in
// the meridian through the seam, the next starts in the same section, and the two are glued there
// (#574). Unioned instead, two tori tangent along that section's circle meet in nothing a trace
// can follow, and the build failed at some drafts and radii and not others.
//
// The plate stands 5 deep on the top plane, its far face 5 below. Orbit (right-drag) to see the
// far rim. Edit the draft or the round's radius and apply (⌘↵).
unit mm
use std
use std.rounds (round)

in std.top {
  e_east := point
  e_west := point
  fix((24, 0)) e_east
  fix((-24, 0)) e_west
  // the waists' centres: the top waist's left arc's centre is right of the notch, its right arc's
  // left of it, so each arc meets the fillet on the far side of the y axis from its own centre
  c_tl := point
  c_tr := point
  fix((3, 36)) c_tl
  fix((-3, 36)) c_tr
  c_bl := point
  c_br := point
  fix((3, -36)) c_bl
  fix((-3, -36)) c_br
  // where the ends meet the waists: on the line between their centres, 15 from the end's
  nw := point
  ne := point
  sw := point
  se := point
  fix((-15, 12)) nw
  fix((15, 12)) ne
  fix((-15, -12)) sw
  fix((15, -12)) se
  // the notches' fillets, on the y axis, each tangent to both arcs of its waist
  f_top := point hint((0, 10))
  f_bottom := point hint((0, -10))
  fix(x == 0mm) f_top
  fix(x == 0mm) f_bottom
  t1 := point hint((-0.5, 6.2))
  t2 := point hint((0.5, 6.2))
  t3 := point hint((0.5, -6.2))
  t4 := point hint((-0.5, -6.2))
  t1 coincident line(c_tl, f_top)
  t2 coincident line(c_tr, f_top)
  t3 coincident line(c_br, f_bottom)
  t4 coincident line(c_bl, f_bottom)

  east := arc(center: e_east, start: se, end: ne)
  top_r := arc(center: c_tr, start: t2, end: ne)
  top_root := arc(center: f_top, start: t1, end: t2)
  top_l := arc(center: c_tl, start: nw, end: t1)
  west := arc(center: e_west, start: nw, end: sw)
  bot_l := arc(center: c_bl, start: t4, end: sw)
  bot_root := arc(center: f_bottom, start: t3, end: t4)
  bot_r := arc(center: c_br, start: se, end: t3)
  radius(4mm) top_root
  radius(4mm) bot_root
  outline := face(east, top_r, top_root, top_l, west, bot_l, bot_root, bot_r)
}
plate := solid(outline, depth: 5mm, draft: 3deg)
body := solid(plate)

// the far rim: eight stretches of rings, one statement
body.plate.far round(r: 1mm) body.plate
