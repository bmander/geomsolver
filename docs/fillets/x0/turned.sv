// X0 (#183): solid_fillet_turned.sv, a frustum standing on a disc, both turned about one axis:
// the frustum's foot filled, its top rim rounded off. Does not parse yet.
//
// Written twice. First as solids, the way the stack wrote it. Then as one region, to see whether
// a coaxial part can say its rounds inside a region's body (L5, #145 F5): it cannot, because the
// part is a union of two regions, and a union is not a region (§6.21: "union is a choice").
unit mm
use std
in std.front {
  a0 := point
  a1 := point
  fix((0, 0)) a0
  fix((0, 40)) a1
  spindle := line(a0, a1)
  p0 := point
  p1 := point
  p2 := point
  p3 := point
  fix((0, 0)) p0
  fix((40, 0)) p1
  fix((40, 10)) p2
  fix((0, 10)) p3
  (pbot := line(p0, p1)) -> (pside := line(p1, p2)) -> (ptop := line(p2, p3)) ->
    (paxis := line(p3, p0)) -> close
  q0 := point
  q1 := point
  q2 := point
  q3 := point
  fix((0, 10)) q0
  fix((15, 10)) q1
  fix((5, 30)) q2
  fix((0, 30)) q3
  (cbot := line(q0, q1)) -> (slant := line(q1, q2)) -> (ctop := line(q2, q3)) ->
    (caxis := line(q3, q0)) -> close
}
plate := solid(face(pbot, pside, ptop, paxis), about: spindle)
frustum := solid(face(cbot, slant, ctop, caxis), about: spindle)
body := solid(plate)
frustum union body

// The foot: where the frustum's faces meet the plate's (concave: a torus fills it).
body.frustum round(3mm) body.plate
// The top rim (convex: a torus rolls it off).
body.frustum.ctop round(1mm) body.frustum.slant

// --- The same part as a region: not expressible. ---
//
// The frustum alone is a region, and its rim is a round between two of its bounds:
//
//   frustum_r := { p | p inside std.Cone(spindle_from_apex, half: …); p outside base_level;
//                      p inside top_level; top_level round(1mm) cone }
//
// But the plate and the frustum together are a union of two regions, which no one region body
// says, so the foot's round (between a bound of one and a bound of the other) has nowhere to be
// written. Finding: rounds in a region's body (L5) cover a single region's own edges, convex
// between two bounds and concave where `outside` makes a difference. Joins of parts need rounds
// on solids (L6). Neither step can be skipped.
