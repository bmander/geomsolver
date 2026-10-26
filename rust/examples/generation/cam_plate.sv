// A cam plate as a solid: the plate is a blank with everything the follower's tappet sweeps
// through taken out of it, and its side is the surface the tappet's face cuts.
//
// `flat_cam.sv`'s follower, a flat face rising `h` and returning on the cycloidal law, is drawn
// here as what it is in a machine, a tappet: a block whose underside is the face.  Turned under
// it, the cam plate keeps only what the tappet never reaches, so the plate is a cut: the blank,
// a disc 6 thick, less the tappet swept over the turn as the cam sees it.  The tappet's
// underside is a side of a prism, and the surface it cuts is that prism's side generated under
// the drive (`surface(tappet, edge: flat)`, its envelope over the rise): the cam's planar profile
// extruded square to the page, through the plate.
//
// A point drawn in a view square to the page, 20 below the cam's centre and 3 off the page, is
// held on that surface: it lands where the rising flank crosses that level, which nothing
// states.  The plate is a field the page meshes; its exact export is refused, as a sweep under a
// law is outside the class built exactly.  Edit `base`, `h` or the law, and the plate is cut
// again.

unit mm
use std
use std.motion (spins, slides)

base := 25    // the face's nearest approach to the cam's centre
h := 15       // the follower's lift

param t: Angle over (0deg, 360deg)          // the cam's turn

in std.front {
  o := point
  fix((0, 0)) o
  s0 := point
  s1 := point
  fix((0, 0)) s0
  fix((10, 0)) s1
  path := line(s0, s1)                    // the follower's line, through the cam's centre
  // the tappet where the turn starts: its face square to the line, `base` out
  f0 := point
  f1 := point
  f2 := point
  f3 := point
  fix((base, -20)) f0
  fix((base, 20)) f1
  fix((base + 30, 20)) f2
  fix((base + 30, -20)) f3
  flat := line(f0, f1)
  rim := circle(center: o) hint(r: 45)
  radius(45mm) rim
}

cam := frame
cam spins(1) o
follower := frame
follower slides(h) path                 // the value `360deg` is the whole lift
// rise over the first half turn, return over the second, each cycloidal
fix(value == 360deg * (min(t / 180deg, 1) - sin(360deg * min(t / 180deg, 1)) / (2 * pi)
                       - max((t - 180deg) / 180deg, 0)
                       + sin(360deg * max((t - 180deg) / 180deg, 0)) / (2 * pi))) follower

construction tappet := solid(face(flat, f1, f2, f3, -> close), from: -8mm, to: 8mm)
plate := solid(face(rim), depth: 6mm)
swept := sweep(tappet over t, under: follower, seen_from: cam)
swept cut plate

// the plate's rising flank, a surface in space
wall := surface(tappet, edge: flat)
flank := envelope(wall over t in (0deg, 180deg), under: follower, seen_from: cam)

// a view square to the page, through the level 20 below the cam's centre
level := plane
fix(origin == (0, 0, -20)) level
fix(dir == (1, 0, 0)) level.u
fix(dir == (0, 1, 0)) level.v
gauge := point in level hint((15, 3))
fix(y == 3) gauge
gauge coincident flank
