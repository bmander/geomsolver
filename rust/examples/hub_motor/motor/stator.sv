// The stator: 51 teeth of bonded M270-35A laminations, and the copper wound on them.
//
// **One slot is drawn and the ring turns it.**  A slot is the space between two teeth, and a
// tooth is parallel-sided, so the slot's walls lie `wt / 2` off the two teeth's centrelines,
// which stand half a slot pitch either side of the slot's own.  The walls run from the slots'
// bottom circle up to the shoes; the shoes close the slot to its opening `bo`, `hs` below the
// tips, and the walls stop `hw` below that, where the slot wedge sits.  The cutter carries on
// past the tips so it opens cleanly.  Nothing is placed by a sine or a cosine: the slot is its
// walls' distances from two lines at stated angles, its far side the near side's mirror.
//
// The lamination is the annulus between the tips' circle and the bore, less the 51 slots: one
// cutter, drawn once, its copies the ring's.  The copper is the band the coils fill — from the
// slots' bottom to the wedges, over the stack and the end turns either side of it — less the
// lamination: a coil side in each slot under its wedge (two a slot, one each of two teeth's
// coils), and the end turns either side of the stack, which the covers must clear.  The three
// hall sensors stand in three slots' mouths.

use std (horizontal)
use motor.dims (bo, end_turn_in, hall_l, hall_r0, hall_r1, hall_w, hs, L, overhang, Rb, Rc, Rs, Ry,
                slot_pitch, slots, wt)
use motor.parts (Datums)

component Stator(f: Datums) {
  in f.o.plane {
    ring slots about f.o {
      // the slot's centreline, and the centrelines of the teeth either side of it
      construction sc := line(f.o, hint((Rs + 3mm, 0mm)))
      length(Rs + 3mm) sc
      construction t1 := line(f.o, hint((Rs, -1mm)))
      construction t2 := line(f.o, hint((Rs, 1mm)))
      length(Rs) t1
      length(Rs) t2
      t1 angle(slot_pitch / 2) sc
      sc angle(slot_pitch / 2) t2
      // the slot's near side (the clockwise one), from its bottom out past the tips
      a := point hint((Ry, -2mm))
      h := point hint((Rc, -3mm))
      g := point hint((Rs - hs, -bo / 2))
      k := point hint((Rs + 2mm, -bo / 2))
      a distance(wt / 2, side: left) t1
      h distance(wt / 2, side: left) t1
      f.o distance(Ry) a
      f.o distance(Rc) h
      g distance(bo / 2, side: right) sc
      k distance(bo / 2, side: right) sc
      f.o distance(Rs - hs) g
      f.o distance(Rs + 2mm) k
      // each on its circle ahead of the axle, not on the circle's far side behind it
      f.o distance(>= 0, along: sc) a
      f.o distance(>= 0, along: sc) h
      f.o distance(>= 0, along: sc) g
      f.o distance(>= 0, along: sc) k
      // and its far side, the mirror
      b := point hint((Ry, 2mm))
      c := point hint((Rc, 3mm))
      d := point hint((Rs - hs, bo / 2))
      e := point hint((Rs + 2mm, bo / 2))
      a symmetry(sc) b
      h symmetry(sc) c
      g symmetry(sc) d
      k symmetry(sc) e
      bottom := line(a, b)
      wall_r := line(b, c)
      shoe_r := line(c, d)
      // the cutter's run-out past the tips: its boundary, not the lamination's
      construction lip_r := line(d, e)
      construction mouth := line(e, k)
      construction lip_l := line(k, g)
      shoe_l := line(g, h)
      wall_l := line(h, a)
    }
    // the first slot stands on the datum
    f.datum angle(0deg) sc[0]
    tips := circle(center: f.o) hint(r: Rs)
    radius(Rs) tips
    bore := circle(center: f.o) hint(r: Rb)
    radius(Rb) bore
    stack := solid(face(tips, holes: bore), from: -L / 2, to: L / 2)
  }
  lamination := solid(stack)
  repeat slots as i {
    private slot := solid(face(bottom[i], wall_r[i], shoe_r[i], lip_r[i], mouth[i], lip_l[i],
                               shoe_l[i], wall_l[i]), from: -L / 2 - 1mm, to: L / 2 + 1mm)
    slot cut lamination
  }

  // -- the copper ---------------------------------------------------------------------------
  // The coils fill the band between the slots' bottom and the wedges, the stack's length and the
  // end turns' either side of it, wherever the iron is not: the band less the lamination is a
  // coil side in every slot, and the end turns either side.
  in f.o.plane {
    band_o := circle(center: f.o) hint(r: Rc)
    radius(Rc) band_o
    band_i := circle(center: f.o) hint(r: end_turn_in)
    radius(end_turn_in) band_i
    windings := solid(face(band_o, holes: band_i), from: -L / 2 - overhang, to: L / 2 + overhang)
  }
  lamination cut windings

  // -- the hall sensors ----------------------------------------------------------------------
  // Three SS41s, each in a slot's mouth over its coils, in slots 0, 17 and 34: 17 slots are a
  // third of a turn, and 17 slot pitches of 23 × 7.06° electrical are 240° — so the three read
  // the three phases exactly 120° electrical apart, and are themselves a ring of three.  Each is
  // set into the stack's left end, its leads out over the end turns toward the cable.
  in f.o.plane {
    ring 3 about f.o {
      construction hc := line(f.o, hint((Rs, 0mm)))
      length(Rs) hc
      q1 := point hint((Rc, -1mm))
      q2 := point hint((Rc, 1mm))
      q3 := point hint((Rs, 1mm))
      q4 := point hint((Rs, -1mm))
      q1 distance(hall_w / 2, side: right) hc
      q4 distance(hall_w / 2, side: right) hc
      q2 distance(hall_w / 2, side: left) hc
      q3 distance(hall_w / 2, side: left) hc
      f.o distance(hall_r0) q1
      f.o distance(hall_r0) q2
      f.o distance(hall_r1) q3
      f.o distance(hall_r1) q4
      f.o distance(>= 0, along: hc) q1
      f.o distance(>= 0, along: hc) q2
      f.o distance(>= 0, along: hc) q3
      f.o distance(>= 0, along: hc) q4
      hall_s := (h1 := line(q1, q2)) -> (h2 := line(q2, q3)) -> (h3 := line(q3, q4)) ->
                (h4 := line(q4, q1)) -> close
    }
    f.datum angle(0deg) hc[0]
    halls := solid(face(h1[0], h2[0], h3[0], h4[0]), from: L / 2 - hall_l, to: L / 2)
  }
  repeat 2 as i {
    private hall := solid(face(h1[i + 1], h2[i + 1], h3[i + 1], h4[i + 1]),
                          from: L / 2 - hall_l, to: L / 2)
    hall union halls
  }

  // the stack and the end turns as the half-section along the axle cuts them
  in f.so.plane {
    stack_s := motor.parts.Box(f.so, x0: -L / 2, y0: Rb, x1: L / 2, y1: Rs)
    end_l := motor.parts.Box(f.so, x0: -L / 2 - overhang, y0: end_turn_in, x1: -L / 2, y1: Rc)
    end_r := motor.parts.Box(f.so, x0: L / 2, y0: end_turn_in, x1: L / 2 + overhang, y1: Rc)
  }
}

// Open this file to preview the stator alone: the lamination, the windings and the hall
// sensors.
preview {
  unit mm
  // the frame, as the assembly draws it: the end view's datum and the axle's centreline
  in std.front {
    construction centerline datum := line(std.origin, hint((150, 0)))
    horizontal datum
    length(150mm) datum
  }
  in std.side {
    construction centerline ax := line(std.side.origin, hint((100, 0)))
    horizontal ax
    length(100mm) ax
  }
  frame := {o: std.origin, datum: datum, so: std.side.origin, ax: ax}

  stator := Stator(frame)
}
