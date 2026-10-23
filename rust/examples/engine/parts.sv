// The reciprocating parts, as the end view and the side view each draw them.
//
// Each part reads the numbers it needs from the engine's table, `engine.dims`, as constants of
// its definition: the `use` below lists them, so a piston is `D` wide in every view.

use engine.dims (D, ch, ph, rpin)
use std (above, below, left_of, offset, right_of)

// A line tangent to two circles at both ends — a belt run, a crank web's flank, a cam's flank.
// `side` says which side of the centre line: 1 the left of `k1` to `k2`, -1 the right.  That is
// stated as both circles on the run's other side, which is the outer tangent there — neither a
// crossed one nor the run of no length where overlapping circles meet; the seeds are the two
// contact points at the bearing square to the line of centres.
component Span(k1: circle, k2: circle, side: Scalar) {
  a := point hint(at: k1, bearing: atan2(k2.center.y - k1.center.y, k2.center.x - k1.center.x) + side * 90deg)
  b := point hint(at: k2, bearing: atan2(k2.center.y - k1.center.y, k2.center.x - k1.center.x) + side * 90deg)
  s := line(a, b)
  a coincident k1
  b coincident k2
  s tangent(at: p1) k1
  s tangent(at: p2) k2
  when side > 0 {
    cw(a, b, k1.center)
    cw(a, b, k2.center)
  }
  when side < 0 {
    ccw(a, b, k1.center)
    ccw(a, b, k2.center)
  }
}

// -- the piston -------------------------------------------------------------------------------
// The table gives the bore, the pin's height under the crown and the piston's height; the rest is
// the piston's own, every number a depth under the crown or a radius from the bore axis, as the
// turner dimensions it.
pr := (D - 0.5mm) / 2   // the skirt: a quarter of a millimetre's clearance a side in the bore
tl := 6mm               // the top land, the crown's edge to the top ring's groove
cw := 1.5mm             // a compression ring's groove, its width
ow := 3mm               // the oil control ring's groove
land := 4mm             // the second and third lands, between the grooves
gd := 3.5mm             // a groove's depth: the ring's radial wall and the room behind it
g2 := tl + cw + land    // the second ring's groove, under the crown…
g3 := g2 + cw + land    // …and the oil ring's
belt := g3 + ow + 3mm   // the ring belt's underside, inside: 3 of metal under the oil groove
rbelt := pr - gd - 3mm  // the ring belt's bore: 3 of metal behind the groove roots
crown_t := 7mm          // the crown's thickness at its rim; the dish leaves 5 at the centre
rd := 26mm              // the dish in the crown: its radius at the crown's face…
dd := 2mm               // …and its depth at the centre
ri := pr - 2.5mm        // the skirt's bore: a wall of 2.5
rboss := rpin + 5mm     // a pin boss: a block `2 rboss` square about the pin
gap := 13mm             // the bosses' faces off the bore axis: the rod's small end is 22 along
                        // the pin, so 2 of side clearance each side of it
rpi := rpin - 4.5mm     // the gudgeon pin is a tube: its bore…
lp := 2 * pr - 14mm     // …and its length, 7 inside the skirt each end for a circlip
rl := ph - 10mm         // the slipper skirt: cut 10 short across the pin's two ends, leaving the
sx := 26mm              // thrust faces `sx` either side of the bore axis at full length
reach := pr + 2mm       // how far a cutter reaches past the skirt, across the axis…
deep := ph + 2mm        // …and under its foot

// A piston on its rod's small end: a turned crown and ring belt over a thin skirt, hollow
// underneath, the gudgeon pin through two bosses inside it with the small end between them.
//
// From outside it is one outline in both views but for its foot: the crown, the three ring
// grooves notched into each flank with their edges drawn across, the lands between them, and the
// skirt, cut short across the pin's two ends and whole under the thrust faces between.  `pin` says
// which way the view looks: 1 along the pin (the end view), where the pin is seen end on, the
// thrust faces stand to each side and the relief's edge runs across between them; 0 across it
// (the side view), where the relief takes the skirt's two corners.  What the outline hides — the
// dish, the hollow, the bosses and the pin — is `hidden`, which the sheet draws dashed.
component Piston(small: point, pin: Int) {
  // where the skirt's foot is drawn: the corner `r2` off the axis, `d1` under the crown, and the
  // foot across between the two corners at `d2`
  d1 := rl + pin * (ph - rl)
  d2 := ph - pin * (ph - rl)
  r2 := sx + pin * (sqrt(pr * pr - sx * sx) - sx)
  in small.plane {
    cl := point hint((small.x - pr, small.y + ch))
    cl offset(dx: -pr, dy: ch) small
    // round from the crown's left edge, each run one number: down the right flank, groove by
    // groove, across the foot, and up the left flank to the crown again
    outline := step(du: 2 * pr) (crown := line(p1: cl)) ->
               step(dv: -tl) (land1_r := line) -> step(du: -gd) (g1a_r := line) ->
               step(dv: -cw) (g1_r := line) -> step(du: gd) (g1b_r := line) ->
               step(dv: -land) (land2_r := line) -> step(du: -gd) (g2a_r := line) ->
               step(dv: -cw) (g2_r := line) -> step(du: gd) (g2b_r := line) ->
               step(dv: -land) (land3_r := line) -> step(du: -gd) (g3a_r := line) ->
               step(dv: -ow) (g3_r := line) -> step(du: gd) (g3b_r := line) ->
               step(dv: g3 + ow - d1) (skirt_r := line) -> step(du: r2 - pr) (foot_r := line) ->
               step(dv: d1 - d2) (relief_r := line) -> step(du: -2 * r2) (foot := line) ->
               step(dv: d2 - d1) (relief_l := line) -> step(du: r2 - pr) (foot_l := line) ->
               step(dv: d1 - g3 - ow) (skirt_l := line) -> step(du: gd) (g3b_l := line) ->
               step(dv: ow) (g3_l := line) -> step(du: -gd) (g3a_l := line) ->
               step(dv: land) (land3_l := line) -> step(du: gd) (g2b_l := line) ->
               step(dv: cw) (g2_l := line) -> step(du: -gd) (g2a_l := line) ->
               step(dv: land) (land2_l := line) -> step(du: gd) (g1b_l := line) ->
               step(dv: cw) (g1_l := line) -> step(du: -gd) (g1a_l := line) ->
               (land1_l := line) -> close
  }
  // each groove's two edges, seen across the piston's face
  e1 := line(g1a_l.p2, land1_r.p2)
  e2 := line(land2_l.p2, g1b_r.p2)
  e3 := line(g2a_l.p2, land2_r.p2)
  e4 := line(land3_l.p2, g2b_r.p2)
  e5 := line(g3a_l.p2, land3_r.p2)
  e6 := line(skirt_l.p2, g3b_r.p2)
  // seen along it, the pin's end through the skirt: the tube and its bore
  when pin == 1 {
    k := circle(center: small) hint(r: rpin)
    radius(rpin) k
    kb := circle(center: small) hint(r: rpi)
    radius(rpi) kb
  }
  hidden := Inside(small, end_on: pin)

  // -- the solids: seen across the pin, the piston stands at its own station along the crank
  // axis, so that picture is the one made into them.  The body is the outline's right half, dish
  // and grooves and its skirt whole, turned about the bore axis, less the hollow under the crown.
  // The bosses are what the hollow leaves about the pin: each a block from its face out past the
  // skirt, `2 rboss` square, cut from the hollow before the hollow is cut from the body, so it
  // stands on whatever wall it meets.  Then the pin's bore through all of it, and the slipper's
  // two reliefs, prisms across the engine.  The gudgeon pin is a solid of its own, a tube turned
  // about its own line.
  when pin == 0 {
    construction pe := point hint((small.x + reach, small.y))
    construction pw := point hint((small.x - reach, small.y))
    pe right_of(d: reach) small
    pw left_of(d: reach) small
    construction pinline := line(pw, pe)
    // the corners nothing draws, where the solids' sections close on the axes or run past the
    // skirt: the foot whole, the hollow open beneath it, each boss's reach and the pin's bore
    construction q1 := point hint((small.x + pr, small.y + ch - ph))
    construction q2 := point hint((small.x, small.y + ch - ph))
    construction q3 := point hint((small.x, small.y + ch - crown_t))
    construction q4 := point hint((small.x + ri, small.y + ch - deep))
    construction q5 := point hint((small.x, small.y + ch - deep))
    q1 offset(dx: pr, dy: ch - ph) small
    q2 below(d: ph - ch) small
    // the bore axis, up from the foot to the dish: the line the body is turned about, and the
    // edge its section closes along
    construction bore_axis := line(q2, hidden.db)
    q3 above(d: ch - crown_t) small
    q4 offset(dx: ri, dy: ch - deep) small
    q5 below(d: deep - ch) small
    construction b1 := point hint((small.x + reach, small.y - rboss))
    construction b2 := point hint((small.x + reach, small.y + rboss))
    construction b3 := point hint((small.x - reach, small.y - rboss))
    construction b4 := point hint((small.x - reach, small.y + rboss))
    construction b5 := point hint((small.x + reach, small.y + rpin))
    construction b6 := point hint((small.x - reach, small.y + rpin))
    b1 offset(dx: reach, dy: -rboss) small
    b2 offset(dx: reach, dy: rboss) small
    b3 offset(dx: -reach, dy: -rboss) small
    b4 offset(dx: -reach, dy: rboss) small
    b5 offset(dx: reach, dy: rpin) small
    b6 offset(dx: -reach, dy: rpin) small
    construction s1 := point hint((small.x + reach, small.y + ch - rl))
    construction s2 := point hint((small.x + reach, small.y + ch - deep))
    construction s3 := point hint((small.x + sx, small.y + ch - deep))
    construction s4 := point hint((small.x - reach, small.y + ch - rl))
    construction s5 := point hint((small.x - reach, small.y + ch - deep))
    construction s6 := point hint((small.x - sx, small.y + ch - deep))
    s1 offset(dx: reach, dy: ch - rl) small
    s2 offset(dx: reach, dy: ch - deep) small
    s3 offset(dx: sx, dy: ch - deep) small
    s4 offset(dx: -reach, dy: ch - rl) small
    s5 offset(dx: -reach, dy: ch - deep) small
    s6 offset(dx: -sx, dy: ch - deep) small

    turned := solid(face(hidden.dish_r, crown.p2, land1_r.p2, g1a_r.p2, g1_r.p2, g1b_r.p2,
                         land2_r.p2, g2a_r.p2, g2_r.p2, g2b_r.p2, land3_r.p2, g3a_r.p2, g3_r.p2,
                         g3b_r.p2, q1, bore_axis), about: bore_axis)
    body := solid(turned)
    hollow := solid(face(q3, hidden.belt_r.p2, hidden.belt_r.p1, hidden.wall_r.p2, q4, q5,
                         -> close), about: bore_axis)
    boss_r := solid(face(hidden.boss_face_r.p1, hidden.boss_face_r.p2, b1, b2, -> close),
                    from: -rboss, to: rboss)
    boss_l := solid(face(hidden.boss_face_l.p1, hidden.boss_face_l.p2, b3, b4, -> close),
                    from: -rboss, to: rboss)
    boss_r cut hollow
    boss_l cut hollow
    hollow cut body
    bore := solid(face(pw, pe, b5, b6, -> close), about: pinline)
    bore cut body
    slip_r := solid(face(foot_r.p2, s1, s2, s3, -> close), from: -reach, to: reach)
    slip_l := solid(face(foot_l.p1, s4, s5, s6, -> close), from: -reach, to: reach)
    slip_r cut body
    slip_l cut body
    gudgeon := solid(face(hidden.pin_top.p1, hidden.pin_top.p2, hidden.bore_t.p2,
                          hidden.bore_t.p1, -> close), about: pinline)
  }
}

// What a piston's outline hides, about its small end: the dish sunk in the crown, and the hollow
// under the crown and inside the skirt — the ring belt's bore down to its underside, the skirt's
// below it — each seen down to the foot it ends at (`end_on` as the piston's `pin`).  Seen across
// the pin, the bosses each side of the small end and the gudgeon pin in them, the pin's bore
// along it; seen along it, a boss's square end.  The sheet draws all of it dashed.
component Inside(small: point, end_on: Int) {
  d1 := rl + end_on * (ph - rl)
  in small.plane {
    // the dish: a shallow sphere through its rim, `dd` deep on the axis, in two halves meeting
    // there
    dl := point hint((small.x - rd, small.y + ch))
    dr := point hint((small.x + rd, small.y + ch))
    dl offset(dx: -rd, dy: ch) small
    dr offset(dx: rd, dy: ch) small
    db := point hint((small.x, small.y + ch - dd))
    db above(d: ch - dd) small
    dc := point hint((small.x, small.y + ch - dd + (rd * rd + dd * dd) / (2 * dd)))
    dish_l := arc(center: dc, start: dl, end: db) hint(r: (rd * rd + dd * dd) / (2 * dd))
    dish_r := arc(center: dc, start: db, end: dr) hint(r: (rd * rd + dd * dd) / (2 * dd))
    // the hollow, up the skirt's bore from its foot, in under the ring belt, up to the crown,
    // across under it and down the other side
    w0 := point hint((small.x + ri, small.y + ch - d1))
    w0 offset(dx: ri, dy: ch - d1) small
    step(dv: d1 - belt) (wall_r := line(p1: w0)) -> step(du: rbelt - ri) (step_r := line) ->
      step(dv: belt - crown_t) (belt_r := line) -> step(du: -2 * rbelt) (under := line) ->
      step(dv: crown_t - belt) (belt_l := line) -> step(du: rbelt - ri) (step_l := line) ->
      step(dv: belt - d1) (wall_l := line)
    when end_on == 0 {
      // each boss from the side: its top from the ring belt's bore to its face, the face the small
      // end runs against, and its underside back out to the skirt's bore
      t_r := point hint((small.x + rbelt, small.y + rboss))
      t_l := point hint((small.x - rbelt, small.y + rboss))
      t_r offset(dx: rbelt, dy: rboss) small
      t_l offset(dx: -rbelt, dy: rboss) small
      step(du: gap - rbelt) (boss_top_r := line(p1: t_r)) ->
        step(dv: -2 * rboss) (boss_face_r := line) -> step(du: ri - gap) (boss_foot_r := line)
      step(du: rbelt - gap) (boss_top_l := line(p1: t_l)) ->
        step(dv: -2 * rboss) (boss_face_l := line) -> step(du: gap - ri) (boss_foot_l := line)
      // the gudgeon pin, its outline and its bore
      p0 := point hint((small.x - lp / 2, small.y + rpin))
      p0 offset(dx: -lp / 2, dy: rpin) small
      step(du: lp) (pin_top := line(p1: p0)) -> step(dv: -2 * rpin) (pin_r := line) ->
        step(du: -lp) (pin_foot := line) -> (pin_l := line) -> close
      i0 := point hint((small.x - lp / 2, small.y + rpi))
      i1 := point hint((small.x - lp / 2, small.y - rpi))
      i0 offset(dx: -lp / 2, dy: rpi) small
      i1 offset(dx: -lp / 2, dy: -rpi) small
      step(du: lp) (bore_t := line(p1: i0))
      step(du: lp) (bore_b := line(p1: i1))
    }
    when end_on == 1 {
      // a boss end on: the block the pin is bored through, as wide as it is deep
      k0 := point hint((small.x - rboss, small.y + rboss))
      k0 offset(dx: -rboss, dy: rboss) small
      step(du: 2 * rboss) (boss_top := line(p1: k0)) -> step(dv: -2 * rboss) (boss_r := line) ->
        step(du: -2 * rboss) (boss_foot := line) -> (boss_l := line) -> close
    }
  }
}

// A point placed from `o` by two ordinates — the corner of an outline, a centre on a pitch.
// One statement where a point and its two runs were three.
component At(o: point, dx: Length, dy: Length) {
  p := point hint((o.x + dx, o.y + dy))
  p offset(dx: dx, dy: dy) o
}

// An axis-aligned rectangle about a point: `a` is its lower-left corner offset from `o`.
component Box(o: point, x0: Length, y0: Length, x1: Length, y1: Length) {
  a := point hint((o.x + x0, o.y + y0))
  b := point hint((o.x + x1, o.y + y0))
  c := point hint((o.x + x1, o.y + y1))
  d := point hint((o.x + x0, o.y + y1))
  profile := (ab := line(a, b)) -> (bc := line(b, c)) -> (cd := line(c, d)) -> (da := line(d, a)) -> close
  a offset(dx: x0, dy: y0) o
  b offset(dx: x1, dy: y0) o
  c offset(dx: x1, dy: y1) o
  d offset(dx: x0, dy: y1) o
}

// Open this file to preview cylinder 1's piston at top dead centre: on its small end in the end
// section, its pin seen end on, and edge on in the side section, where it is made into its solids:
// the piston, and the gudgeon pin in it.
preview {
  unit mm
  in std.side {
    small := At(std.side.origin, dx: 0mm, dy: engine.dims.R + engine.dims.L)
    piston := Piston(small.p, pin: 1)
  }
  in std.front {
    small_s := At(std.origin, dx: engine.dims.front + 25mm + engine.dims.P / 2,
                  dy: engine.dims.R + engine.dims.L)
    piston_s := Piston(small_s.p, pin: 0)
  }
}
