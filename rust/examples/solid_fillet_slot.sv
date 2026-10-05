// A plate with an obround boss and a rounded pocket (issue #66, rung 3): fillets carried along
// chains of tangent edges. The boss's foot runs straight along its sides and round its half-circle
// ends; the pocket's rim runs straight and round its four corners. Each straight run is a prism,
// each corner part of a ring, and every piece ends in the section the next begins in, so the ball
// rolls on round the chain with no seam to blend: `foot` fills the concave foot, `lip` rolls the
// convex rim off.
unit mm

// the plate, 80 × 50, its top on the page
p0 := point
p1 := point hint(x: 80, y: 0)
p2 := point hint(x: 80, y: 50)
p3 := point hint(x: 0, y: 50)
fix(x == 0, y == 0) p0
fix(x == 80, y == 0) p1
fix(x == 80, y == 50) p2
fix(x == 0, y == 50) p3
(ab := line(p0, p1)) -> (bc := line(p1, p2)) -> (cd := line(p2, p3)) -> (da := line(p3, p0)) -> close
top := face(ab, bc, cd, da)

// the boss: an obround 30 long and 12 wide, its ends half circles about centres 18 apart
o0 := point hint(x: 11, y: 25)
o1 := point hint(x: 29, y: 25)
fix(x == 11, y == 25) o0
fix(x == 29, y == 25) o1
s0 := point hint(x: 11, y: 19)
s1 := point hint(x: 29, y: 19)
s2 := point hint(x: 29, y: 31)
s3 := point hint(x: 11, y: 31)
fix(x == 11, y == 19) s0
fix(x == 29, y == 19) s1
fix(x == 29, y == 31) s2
fix(x == 11, y == 31) s3
(near_side := line(s0, s1)) -> (east := arc(center: o1) hint(r: 6)) -> (far_side := line(s2, s3)) ->
  (west := arc(center: o0) hint(r: 6)) -> close
outline := face(near_side, east, far_side, west)

// the pocket: 20 × 16, its corners rounded to 4
k0 := point hint(x: 52, y: 21)
k1 := point hint(x: 64, y: 21)
k2 := point hint(x: 64, y: 29)
k3 := point hint(x: 52, y: 29)
fix(x == 52, y == 21) k0
fix(x == 64, y == 21) k1
fix(x == 64, y == 29) k2
fix(x == 52, y == 29) k3
q0 := point hint(x: 52, y: 17)
q1 := point hint(x: 64, y: 17)
q2 := point hint(x: 68, y: 21)
q3 := point hint(x: 68, y: 29)
q4 := point hint(x: 64, y: 33)
q5 := point hint(x: 52, y: 33)
q6 := point hint(x: 48, y: 29)
q7 := point hint(x: 48, y: 21)
fix(x == 52, y == 17) q0
fix(x == 64, y == 17) q1
fix(x == 68, y == 21) q2
fix(x == 68, y == 29) q3
fix(x == 64, y == 33) q4
fix(x == 52, y == 33) q5
fix(x == 48, y == 29) q6
fix(x == 48, y == 21) q7
(bottom := line(q0, q1)) -> (corner1 := arc(center: k1) hint(r: 4)) -> (right := line(q2, q3)) ->
  (corner2 := arc(center: k2) hint(r: 4)) -> (upper := line(q4, q5)) -> (corner3 := arc(center: k3) hint(r: 4)) ->
  (left := line(q6, q7)) -> (corner0 := arc(center: k0) hint(r: 4)) -> close
mouth := face(bottom, corner1, right, corner2, upper, corner3, left, corner0)

plate := solid(top, depth: 10mm)
boss := solid(outline, from: 0mm, to: 8mm)
pocket := solid(mouth, from: 1mm, to: -6mm)
cupped := solid(plate)
pocket cut cupped
part := solid(cupped)
boss union part

// the boss's foot: its two sides and its two ends, one chain
foot := fillet(boss, cupped, r: 2mm)
foot union part
// the pocket's rim: four sides and four corners, one chain
lip := fillet(cupped.plate, cupped.pocket, r: 1mm)
lip cut part
