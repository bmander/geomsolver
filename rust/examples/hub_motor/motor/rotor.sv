// The rotor: a steel ring that is both the magnets' back iron and the hub's shell, and the 46
// magnets bonded in its bore.
//
// A magnet is an arc segment, `hm` thick between the gap's circle and the ring's bore and
// `magnet_arc` wide about its own centreline — two arcs about the axle and two radial edges at
// stated angles from that centreline.  One is drawn and the ring of 46 turns it; poles alternate
// N, S round the ring (the drawing does not know a magnet's polarity: it is marked on the part).
// The ring's ends are tapped for the covers' screws, twelve a side on its mid-wall circle.

use std (horizontal)
use motor.dims (half, L, magnet_arc, poles, Rm, Ro, Rr, screw_r, screws, tap4, tap_depth)
use motor.parts (Datums)

component Rotor(f: Datums, theta: Angle) {
  in f.o.plane {
    ring poles about f.o {
      construction mc := line(f.o, hint((Rr + 2mm, 0mm)))
      length(Rr + 2mm) mc
      p1 := point hint((Rm, -6mm))
      p2 := point hint((Rm, 6mm))
      p3 := point hint((Rr, 6mm))
      p4 := point hint((Rr, -6mm))
      construction e1 := line(f.o, p4)
      construction e2 := line(f.o, p3)
      e1 angle(magnet_arc / 2) mc
      mc angle(magnet_arc / 2) e2
      p1 coincident e1
      p2 coincident e2
      // the inner corners on their edges' rays, not the lines' far side behind the axle
      f.o distance(>= 0, along: mc) p1
      f.o distance(>= 0, along: mc) p2
      inner := arc(center: f.o, start: p1, end: p2) hint(r: Rm)
      outer := arc(center: f.o, start: p4, end: p3) hint(r: Rr)
      radius(Rm) inner
      radius(Rr) outer
      side_r := line(p2, p3)
      side_l := line(p4, p1)
    }
    // the rotor's angle: a north pole's centre on the datum
    f.datum angle(theta) mc[0]

    bore := circle(center: f.o) hint(r: Rr)
    radius(Rr) bore
    shell := circle(center: f.o) hint(r: Ro)
    radius(Ro) shell
    ring := solid(face(shell, holes: bore), from: -half, to: half)
    magnets := solid(face(inner[0], side_r[0], outer[0], side_l[0]), from: -L / 2, to: L / 2)
  }
  repeat poles - 1 as i {
    private magnet := solid(face(inner[i + 1], side_r[i + 1], outer[i + 1], side_l[i + 1]),
                            from: -L / 2, to: L / 2)
    magnet union magnets
  }

  // the covers' screws: M4, tapped `tap_depth` into each end, half a pitch off the datum
  screw := motor.parts.HoleCircle(f, n: screws, r: screw_r, d: tap4, phase: 180deg / screws)
  repeat screws as i {
    private left := solid(face(screw.hole[i]), from: half - tap_depth, to: half)
    private right := solid(face(screw.hole[i]), from: -half, to: -half + tap_depth)
    left cut ring
    right cut ring
  }

  // the ring and a magnet as the half-section along the axle cuts them
  in f.so.plane {
    ring_s := motor.parts.Box(f.so, x0: -half, y0: Rr, x1: half, y1: Ro)
    magnet_s := motor.parts.Box(f.so, x0: -L / 2, y0: Rm, x1: L / 2, y1: Rr)
  }
}

// Open this file to preview the rotor alone: the ring and its magnets.
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

  rotor := Rotor(frame, theta: motor.dims.theta)
}
