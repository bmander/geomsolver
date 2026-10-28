// A ball carried along a curve, half sunk in a block: a ball looks the same however it is turned,
// so everything it passes through is the tube about the path its centre runs, whatever that path
// is — here a spline in the block's top face, the ball riding a frame along it (`std.stations.Along`,
// read at the spline's own parameter, `t` from 0 to 1 radian). The tube's wall is fitted to the
// path (the canal, `solid::canal`); the groove is half the tube, half of π r² L + 4/3 π r³ with L
// the spline's length, 36.8973 mm: 578.17 mm³ at these numbers. Keep `ball` below 7.5 mm, the
// spline's tightest turn, or the tube would fold on itself there.
unit mm
use std
use std.place (coords)
use std.stations

ball := 3mm          // the ball's radius

// the path in the block's top face, from where the ball is drawn
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

// the ball, its centre where the path starts, drawn as a half disc turned about its diameter
in std.front {
  private bottom := point
  private top := point
  bottom coords(du: 0mm, dv: -ball) std.front
  top coords(du: 0mm, dv: ball) std.front
  private diameter := line(bottom, top)
  private meridian := arc(center: std.origin, start: bottom, end: top) hint(r: 3)
  radius(ball) meridian
}
construction cutter := solid(face(meridian, diameter), about: diameter)

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
block := solid(face(s0, s1, s2, s3), from: -8mm, to: 0mm)

// the groove the ball sweeps as its frame runs the whole spline
construction groove := sweep(cutter over t in (0rad, 1rad), under: along.f)
groove cut block
