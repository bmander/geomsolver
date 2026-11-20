// A cylinder floating at a draft (#953): a log 200 long and 50 in radius, its axis level, kept
// within the water up to `T` above its keel — the curved faces' case.  Its displacement is stated,
// the circular segment's `L R² (α − sin α cos α)` at the half-angle α = 60° where the waterline
// cuts it, so the solve finds the draft `R (1 − cos α)`, 25mm; the centre of buoyancy is driven
// onto the segment's centroid, `4 R sin³α / (3 (2α − sin 2α))` below the axis, 35.25mm.  The mesh
// reads the curved faces to its quadratic order, so each is good to about 1e-4 of itself — the
// error the report gives beside each value.  Edit the displacement and the log rides higher or
// lower in the water.
unit mm
use std
use std.solids
use std.sets
use std.measures (volume, centroid)

L := 200mm
R := 50mm
param T: Length hint(20mm)

// the half section, turned once about the axis: the log
in std.front {
  o := point
  a := point
  b := point
  c := point
  fix((0mm, 0mm)) o
  fix((L, 0mm)) a
  fix((L, R)) b
  fix((0mm, R)) c
  axis_line := line(o, a)
  end := line(a, b)
  top := line(b, c)
  front := line(c, o)
  section := face(axis_line, end, top, front)
}
log := solid(std.solids.Revolution(section, about: std.x))

keel := point
fix((0mm, 0mm, -R)) keel
water := std.sets.HalfSpace(keel, std.z, d: T)
immersed := solid({ p | p in log; p in water })
volume(L * R^2 * (pi / 3 - sqrt(3) / 4)) immersed

buoyancy := point hint((100, 0, -30))
buoyancy centroid immersed
