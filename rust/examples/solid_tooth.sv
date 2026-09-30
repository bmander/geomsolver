// One involute gear tooth, extruded: its flanks are stretches of involutes, each running between
// the two points `gear.Flank` holds on it — where it crosses the root circle and the tip.
unit mm
use gear

param N = 40
param m = 2mm
param phi = 25deg
param R = m * N / 2
param Rt = R + m
param Rb = R * cos(phi)
param Rr = R - 1.25 * m
param half = 90deg / N + tan(phi) * 1rad - phi
param u0 = sqrt((Rr / Rb) ^ 2 - 1) * 1rad
param u1 = sqrt((Rt / Rb) ^ 2 - 1) * 1rad

point center hint(x: 0mm, y: 0mm)
ground center
circle base(center: center) hint(r: Rb)
circle root(center: center) hint(r: Rr)
circle tip(center: center) hint(r: Rt)
radius(Rb) base
radius(Rr) root
radius(Rt) tip

t: Tooth(base, root, tip, a0: 0deg, half: half, u0: u0, u1: u1)
solid tooth(face(t.r.e from t.r.lo to t.r.hi, t.crown, t.l.e from t.l.hi to t.l.lo, -> close),
            depth: 5mm)
