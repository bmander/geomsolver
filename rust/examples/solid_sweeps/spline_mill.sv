// A bull-nose mill riding a frame along a spline, leaning forward, its round sunk in a block: radius
// 2 mm and 3 mm tall, its bottom corner rounded to 0.5 mm, its axis leaning 30° from upright toward
// where it is going, and its flat end's middle 1 mm under the block's top where the spline starts.
// The frame (`std.stations.Along`, read at the spline's own parameter, `t` from 0 to 1 radian) turns
// the mill as the spline turns, so no constant twist carries it: at each instant the mill touches
// what it sweeps along one closed curve — down its side, round its round, up its side and over its
// top rim — and that curve moves on the mill as the instant's screw changes. The kernel reads the
// curve at each instant, fits the sheet it sweeps over the roll, and splits the block at once by it
// and by the mill where it starts and where it stops (the closed class, any rigid motion, #780 L3).
// Every point of the groove is entered once and left once (the spline turns no tighter than 7.5 mm,
// wider than the mill), so the block loses what the mill holds below its top where it starts and,
// over the roll, the rate at which its faces below the top advance outward: `∫ ∫ (n · v)⁺ dA dt`.
unit mm
use std
use std.solids (cut)
use std.place (coords)
use std.stations

lean := 30deg        // how far the mill's axis leans from upright, toward where it is going

// the path in the block's top face, from where the mill is drawn
in std.top {
  private k0 := point
  private k1 := point
  private k2 := point
  private k3 := point
  fix((0mm, 0mm)) k0
  fix((10mm, 0mm)) k1
  fix((20mm, 20mm)) k2
  fix((30mm, 20mm)) k3
  route := spline(k0, k1, k2, k3)
}
along := std.stations.Along(route, up: std.z)

// the mill, drawn in a plane through its axis, the middle of its flat end 1 mm under the block's top
private across := axis
fix(dir == (cos(lean), 0, -sin(lean))) across
private axle := axis
fix(dir == (sin(lean), 0, cos(lean))) axle
bench := plane(u: across, v: axle)
fix(origin == (0, 0, -1)) bench
in bench {
  private e0 := point
  private e1 := point
  private e2 := point
  private e3 := point
  e0 coords(du: 0mm, dv: 0mm) bench
  e1 coords(du: 1.5mm, dv: 0mm) bench
  e2 coords(du: 2mm, dv: 3mm) bench
  e3 coords(du: 0mm, dv: 3mm) bench
  private k := point
  k coords(du: 1.5mm, dv: 0.5mm) bench
  private e4 := point
  e4 coords(du: 2mm, dv: 0.5mm) bench
  private bottom := line(e0, e1)
  private corner := arc(center: k, start: e1, end: e4) hint(r: 0.5)
  radius(0.5mm) corner
  private side := line(e4, e2)
  private top := line(e2, e3)
  private spindle := line(e3, e0)
}
construction mill := solid(std.solids.Revolution(face(bottom, corner, side, top, spindle), about: spindle))

// the block below the top face, wide enough to hold the whole groove
in std.top {
  private b0 := point
  private b1 := point
  private b2 := point
  private b3 := point
  fix((-6, -6)) b0
  fix((36, -6)) b1
  fix((36, 26)) b2
  fix((-6, 26)) b3
  private s0 := line(b0, b1)
  private s1 := line(b1, b2)
  private s2 := line(b2, b3)
  private s3 := line(b3, b0)
}
block := solid(std.solids.Prism(face(s0, s1, s2, s3), from: -8mm, to: 0mm))

// the groove the mill sweeps as its frame runs the whole spline
construction groove := solid({ p | some t in (0rad, 1rad): p in mill at(under: along.f) t })
groove cut block
