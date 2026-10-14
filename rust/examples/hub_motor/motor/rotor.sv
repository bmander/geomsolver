// The rotor: a steel ring that is both the magnets' back iron and the hub's shell, and the 46
// magnets bonded in its bore.
//
// A magnet is an arc segment, `hm` thick between the gap's circle and the ring's bore and
// `magnet_arc` wide about its own centreline — two arcs about the axle and two radial edges at
// stated angles from that centreline.  One is drawn and the ring of 46 turns it; poles alternate
// N, S round the ring (the drawing does not know a magnet's polarity: it is marked on the part).
// The ring's ends are tapped for the covers' screws, twelve a side on its mid-wall circle.

use motor.parts

component Rotor(f: group, dims: group) {
  in f.front {
    ring dims.poles about f.o {
      construction mc := line(f.o, hint((dims.Rr + 2mm, 0mm)))
      length(dims.Rr + 2mm) mc
      p1 := point hint((dims.Rm, -6mm))
      p2 := point hint((dims.Rm, 6mm))
      p3 := point hint((dims.Rr, 6mm))
      p4 := point hint((dims.Rr, -6mm))
      construction e1 := line(f.o, p4)
      construction e2 := line(f.o, p3)
      e1 angle(dims.magnet_arc / 2) mc
      mc angle(dims.magnet_arc / 2) e2
      p1 coincident e1
      p2 coincident e2
      inner := arc(center: f.o, start: p1, end: p2) hint(r: dims.Rm)
      outer := arc(center: f.o, start: p4, end: p3) hint(r: dims.Rr)
      radius(dims.Rm) inner
      radius(dims.Rr) outer
      side_r := line(p2, p3)
      side_l := line(p4, p1)
    }
    // the rotor's angle: a north pole's centre on the datum
    f.datum angle(dims.theta) mc[0]

    bore := circle(center: f.o) hint(r: dims.Rr)
    radius(dims.Rr) bore
    shell := circle(center: f.o) hint(r: dims.Ro)
    radius(dims.Ro) shell
    ring := solid(face(shell, holes: bore), from: -dims.half, to: dims.half)
    magnets := solid(face(inner[0], side_r[0], outer[0], side_l[0]), from: -dims.L / 2,
                     to: dims.L / 2)
  }
  repeat dims.poles - 1 as i {
    private magnet := solid(face(inner[i + 1], side_r[i + 1], outer[i + 1], side_l[i + 1]),
                            from: -dims.L / 2, to: dims.L / 2)
    magnet union magnets
  }

  // the covers' screws: M4, tapped `tap_depth` into each end, half a pitch off the datum
  screw := motor.parts.HoleCircle(f, n: dims.screws, r: dims.screw_r, d: dims.tap4,
                                  phase: 180deg / dims.screws)
  repeat dims.screws as i {
    private left := solid(face(screw.hole[i]), from: dims.half - dims.tap_depth, to: dims.half)
    private right := solid(face(screw.hole[i]), from: -dims.half, to: -dims.half + dims.tap_depth)
    left cut ring
    right cut ring
  }

  // the ring and a magnet as the half-section along the axle cuts them
  in f.side {
    ring_s := motor.parts.Box(f.so, x0: -dims.half, y0: dims.Rr, x1: dims.half, y1: dims.Ro)
    magnet_s := motor.parts.Box(f.so, x0: -dims.L / 2, y0: dims.Rm, x1: dims.L / 2, y1: dims.Rr)
  }
}
