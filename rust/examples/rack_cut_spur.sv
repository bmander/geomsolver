// A spur gear cut by a rack, as a gear shaper's rack cutter cuts one: generation by motion in space.
//
// Nothing here states an involute.  The rack's tooth is a trapezoid, its flanks `alpha` from the
// radius, extruded through the blank; `cutting` is the rack sliding along its pitch line while
// the blank turns, so the line rolls on the pitch circle without slipping.  `space` is everything
// the tooth sweeps through over the roll, and each `indexed` copy of it, a pitch apart, is cut
// from the blank.  The flanks it leaves are involutes of the base circle, and the root fillets
// trochoids of the rack's tip corners.
//
// `solventc --step` and `--stl` build this exactly (the generating-sweep class,
// docs/generating-sweeps.md): one tooth space's boundary traced as a sheet, one sector of the
// gear cut by it, and that sector turned into the whole.  The rack's tip stops at the working
// depth `m`: a flank reaching past the end of the line of action, `rp sin² alpha` below the pitch
// line, would generate the involute's folded branch, which the class refuses.

unit mm

N := 20                     // teeth
m := 2mm                    // module
alpha := 20deg              // pressure angle
width := 6mm                // face width
bore_r := 6mm
rp := m * N / 2             // pitch radius
hw := pi * m / 4            // half the rack tooth's thickness at the pitch line

o := point
fix(x == 0, y == 0) o
rim := circle(center: o) hint(r: 22)
radius(rp + m) rim
construction bore_c := circle(center: o) hint(r: 6)
radius(bore_r) bore_c
blank := solid(face(rim), depth: width)
construction bore := solid(face(bore_c), from: -width - 2mm, to: 2mm)

// the rack's pitch line, tangent to the pitch circle at the pitch point
s0 := point hint(x: 20, y: 0)
s1 := point hint(x: 20, y: 10)
o distance(rp, along: x) s0
o distance(0, along: y) s0
slide := vertical line(s0, s1)
s0 distance(10) s1

// the rack's tooth: its tip at the working depth, its back past the blank's rim
t0 := point hint(x: 18, y: -0.84)
t1 := point hint(x: 18, y: 0.84)
t2 := point hint(x: 24, y: 3.03)
t3 := point hint(x: 24, y: -3.03)
o distance(rp - m, along: x) t0
o distance(rp - m, along: x) t1
o distance(rp + 2 * m, along: x) t2
o distance(rp + 2 * m, along: x) t3
o distance(-(hw - m * tan(alpha)), along: y) t0
o distance(hw - m * tan(alpha), along: y) t1
o distance(hw + 2 * m * tan(alpha), along: y) t2
o distance(-(hw + 2 * m * tan(alpha)), along: y) t3
tooth := face(t0, t1, t2, t3, -> close)
construction rack_tooth := solid(tooth, from: -width - 2mm, to: 2mm)

turn := motion(about: o, ratio: 1)
rack := motion(along: slide, advance: 2 * pi * rp)
cutting := motion(rack, relative_to: turn)
construction space := solid(rack_tooth, under: cutting, from: -60deg, to: 60deg)

gear := solid(blank)
bore cut gear
repeat N as i {
  construction indexed := solid(space, under: turn, at: i * 360deg / N)
  indexed cut gear
}
