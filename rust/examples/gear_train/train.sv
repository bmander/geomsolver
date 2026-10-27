// A three-stage gear train between two plates: the case `web/tools/open-bench.mjs` opens (#669).
//
// It stands for an assembly at the radiator bot's scale (bmander/radiatorbot): several shaped
// gears, each a generating sweep the exact kernel cuts in seconds, beside the quick static parts
// that hold them, and copies of one figure placed more than once.  What it costs to open is the
// number; it is drawn as a gearbox would be, not checked as one (no claim is stated).
//
// Four axles, A to D, stand in a row along the world's x, each a line of centres apart.  The
// motor's pinion on A drives B's wheel; B's pinion drives C's wheel, and C's pinion D's.  The
// second and third stages are one size, so their pinions are one figure placed twice and their
// wheels another: each is shaped once.  The wheels are spoked; every pinion is shifted +shift and
// the wheel it drives -shift, so a centre distance stays m (N1 + N2) / 2.  A compound (B, C) is
// its wheel and its pinion joined, the pinion reaching down into the wheel's hub.

unit mm
use std
use std.place (coords, raised, seats)
use gears (ShapedSpur)

pa := 20deg                  // pressure angle
shift := 0.3                 // the pinions' profile shift, taken off the wheels they drive
thin := 0.1mm                // each gear's backlash allowance
bore := 5mm                  // every gear's bore, on a 4.8 mm axle
axle_r := 2.4mm

// the first stage, module 1, and the two after it, module 1.5
m1 := 1mm
n_pin1 := 12
n_wheel1 := 40
m2 := 1.5mm
n_pin2 := 12
n_wheel2 := 36

cd1 := m1 * (n_pin1 + n_wheel1) / 2  // A to B
cd2 := m2 * (n_pin2 + n_wheel2) / 2  // B to C, and C to D

// faces, and where each gear stands up its axle
plate_t := 4mm
gap := 32mm                  // between the plates
f_pin1 := 8mm
f_wheel1 := 6mm
f_pin2 := 12mm
f_wheel2 := 8mm
z_wheel1 := 4mm
z_pin_b := z_wheel1 + f_wheel1 - 2mm // B's pinion reaches 2 mm into its wheel
z_wheel_c := z_pin_b + 4mm
z_pin_c := z_wheel_c + f_wheel2 - 2mm
z_wheel_d := z_pin_c + 4mm
z_pin1 := z_wheel1 - 1mm

// the plates' outline, clear of the last wheel's tips
x0 := -20mm
x1 := 2 * cd2 + cd1 + m2 * n_wheel2 / 2 + 6mm
y1 := m2 * n_wheel2 / 2 + 6mm

// A gear seated on an axle: its figure (gears.sv) with its origin `z` up the axle from `c`, its
// axis up the world's z.  `drives` says which of a pair this is: 1 for the pinion, which stands a
// tooth on the line of centres (+x), 0 for the wheel, which stands a space on it.  Gears of one
// size and face are one figure, shaped once.
component Gear(c: point, z: Length, drives: Int, N: Int, m: Length, face: Length,
               chamfer: Length, spokes: Int, hub: Length, web: Length) {
  at := point
  at raised(d: z, along: std.z) c
  g := ShapedSpur(N: N, m: m, phi: pa, shift: (2 * drives - 1) * shift, thin: thin,
                  z0: 0mm, z1: face, bore: bore, shaped: 1, chamfer: chamfer,
                  spokes: spokes, hub: hub, web: web)
  g seats(turn: 90deg + drives * (180deg - 180deg / N), up: std.z, east: std.x) at
  construction gear := solid(g.gear)
}

// A standoff between the plates, drawn about its own origin: one figure placed at each corner.
component Standoff(h: Length, r: Length) {
  in std.top {
    rim := circle(center: std.top.origin) hint(r: r)
    radius(r) rim
  }
  body := solid(face(rim), from: 0mm, to: h)
}

// == The axles ===================================================================================
in std.top {
  a_c := point hint((0, 0))
  a_c coords(du: 0mm, dv: 0mm) std.top
  b_c := point hint((cd1, 0))
  b_c coords(du: cd1, dv: 0mm) std.top
  c_c := point hint((cd1 + cd2, 0))
  c_c coords(du: cd1 + cd2, dv: 0mm) std.top
  d_c := point hint((cd1 + 2 * cd2, 0))
  d_c coords(du: cd1 + 2 * cd2, dv: 0mm) std.top

  a_r := circle(center: a_c) hint(r: 2.4)
  radius(axle_r) a_r
  b_r := circle(center: b_c) hint(r: 2.4)
  radius(axle_r) b_r
  c_r := circle(center: c_c) hint(r: 2.4)
  radius(axle_r) c_r
  d_r := circle(center: d_c) hint(r: 2.4)
  radius(axle_r) d_r

  // the plates' outline and the standoffs' places, inside its corners
  p0 := point hint((-20, -33))
  p0 coords(du: x0, dv: -y1) std.top
  p1 := point hint((137, -33))
  p1 coords(du: x1, dv: -y1) std.top
  p2 := point hint((137, 33))
  p2 coords(du: x1, dv: y1) std.top
  p3 := point hint((-20, 33))
  p3 coords(du: x0, dv: y1) std.top
  s0 := point hint((-14, -27))
  s0 coords(du: x0 + 6mm, dv: -y1 + 6mm) std.top
  s1 := point hint((131, -27))
  s1 coords(du: x1 - 6mm, dv: -y1 + 6mm) std.top
  s2 := point hint((131, 27))
  s2 coords(du: x1 - 6mm, dv: y1 - 6mm) std.top
  s3 := point hint((-14, 27))
  s3 coords(du: x0 + 6mm, dv: y1 - 6mm) std.top
}
a_axle := solid(face(a_r), from: -plate_t, to: gap + plate_t)
b_axle := solid(face(b_r), from: -plate_t, to: gap + plate_t)
c_axle := solid(face(c_r), from: -plate_t, to: gap + plate_t)
d_axle := solid(face(d_r), from: -plate_t, to: gap + plate_t)

// == The plates ==================================================================================
outline := face(p0, p1, p2, p3, -> close)
bottom := solid(outline, from: -plate_t, to: 0mm)
top := solid(outline, from: gap, to: gap + plate_t)
// each plate bored for the axles, by a cutter through both
construction a_bore := solid(face(a_r), from: -plate_t - 1mm, to: gap + plate_t + 1mm)
construction b_bore := solid(face(b_r), from: -plate_t - 1mm, to: gap + plate_t + 1mm)
construction c_bore := solid(face(c_r), from: -plate_t - 1mm, to: gap + plate_t + 1mm)
construction d_bore := solid(face(d_r), from: -plate_t - 1mm, to: gap + plate_t + 1mm)
a_bore cut bottom
b_bore cut bottom
c_bore cut bottom
d_bore cut bottom
a_bore cut top
b_bore cut top
c_bore cut top
d_bore cut top

stand0 := Standoff(h: gap, r: 4mm)
stand0 seats(turn: 0deg, up: std.z, east: std.x) s0
stand1 := Standoff(h: gap, r: 4mm)
stand1 seats(turn: 0deg, up: std.z, east: std.x) s1
stand2 := Standoff(h: gap, r: 4mm)
stand2 seats(turn: 0deg, up: std.z, east: std.x) s2
stand3 := Standoff(h: gap, r: 4mm)
stand3 seats(turn: 0deg, up: std.z, east: std.x) s3

// == The gears ===================================================================================
// the motor's pinion on A, and B's wheel it drives
a_pin := Gear(a_c, z: z_pin1, drives: 1, N: n_pin1, m: m1, face: f_pin1, chamfer: 0.4mm,
              spokes: 0, hub: 0mm, web: 0mm)
b_wheel := Gear(b_c, z: z_wheel1, drives: 0, N: n_wheel1, m: m1, face: f_wheel1, chamfer: 0.3mm,
                spokes: 5, hub: 11.5mm, web: 2mm)
// B's pinion, and C's wheel it drives; C's pinion, and D's wheel: one size, so one figure each
b_pin := Gear(b_c, z: z_pin_b, drives: 1, N: n_pin2, m: m2, face: f_pin2, chamfer: 0.5mm,
              spokes: 0, hub: 0mm, web: 0mm)
c_wheel := Gear(c_c, z: z_wheel_c, drives: 0, N: n_wheel2, m: m2, face: f_wheel2,
                chamfer: 0.4mm, spokes: 5, hub: 11.5mm, web: 2.4mm)
c_pin := Gear(c_c, z: z_pin_c, drives: 1, N: n_pin2, m: m2, face: f_pin2, chamfer: 0.5mm,
              spokes: 0, hub: 0mm, web: 0mm)
d_wheel := Gear(d_c, z: z_wheel_d, drives: 0, N: n_wheel2, m: m2, face: f_wheel2,
                chamfer: 0.4mm, spokes: 5, hub: 11.5mm, web: 2.4mm)

// the parts: the motor's pinion, the two compounds and the output wheel
a_part := solid(a_pin.gear)
b_part := solid(b_wheel.gear)
b_pin.gear union b_part
c_part := solid(c_wheel.gear)
c_pin.gear union c_part
d_part := solid(d_wheel.gear)
