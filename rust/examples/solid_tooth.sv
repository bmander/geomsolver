// One involute gear tooth, extruded: its flanks are stretches of involutes, each running between
// the two points `gear.Flank` holds on it — where it crosses the root circle and the tip.
unit mm
use gear

N := 40
m := 2mm
phi := 25deg
R := m * N / 2
Rt := R + m
Rb := R * cos(phi)
Rr := R - 1.25 * m
half := 90deg / N + tan(phi) * 1rad - phi
u0 := sqrt((Rr / Rb) ^ 2 - 1) * 1rad
u1 := sqrt((Rt / Rb) ^ 2 - 1) * 1rad

center := point
fix(x == 0mm, y == 0mm) center
base := circle(center: center) hint(r: Rb)
root := circle(center: center) hint(r: Rr)
tip := circle(center: center) hint(r: Rt)
radius(Rb) base
radius(Rr) root
radius(Rt) tip

t := gear.Tooth(base, root, tip, a0: 0deg, half: half, u0: u0, u1: u1)
tooth := solid(face(t.r.e from t.r.lo to t.r.hi, t.crown, t.l.e from t.l.hi to t.l.lo, -> close),
            depth: 5mm)
