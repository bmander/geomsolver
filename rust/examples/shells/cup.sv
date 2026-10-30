// A cup: a cylinder 40 across and 30 tall, open at its top. Its section is one whole circle,
// moved in by `wall` to a circle of radius 18, so the cavity is a cylinder too.
//
// Built exactly: 2928π mm³, the cylinder less the cavity's 18² π × 28.
unit mm
use std
use std.rounds (shell)

wall := 2mm

in std.top {
  o := point
  fix((0, 0)) o
  rim := radius(20mm) circle(center: o)
}
cup := solid(face(rim), from: 0mm, to: 30mm)

cup.near shell(t: wall) cup
