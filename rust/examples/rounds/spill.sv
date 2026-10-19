// A chamfer that spills onto a round (#419). A plate's far rim is rounded, and a boss drawn in a
// plane of its own — set on the plate's far face, touching it — stands near the rim, its foot
// chamfered against the far face. The chamfer's legs are 2: out from the boss's wall that is past
// where the far face goes on tangent into the rim's round, so the leg runs on across the seam and
// the flat lands on the round (std's sets stand against the face's smooth surface, `F.smooth`).
//
// The lowering table builds no flat whose face goes on tangent across a seam beside its edge, so
// the chamfer is the field's, and the part is meshed rather than built exactly. The field reads
// the rounded plate by its own exact boundary, not as its prism less the round, so no sheet of
// nothing lies on the far face's plane over the round for the flat to cross, and the part meshes.
// Edit the boss's radius (21 to 21.7 all mesh) and apply (⌘↵).
unit mm
use std (round, chamfer)

rim := circle(center: std.front.origin) hint(r: 33.54) in std.front
plate := solid(face(rim), depth: 20mm)
plate.rim round(r: 3mm) plate.far

// the boss's plane, on the far face and tangent to it there
stand := plane(u: hint(dir: (1, 0, 0)), v: hint(dir: (0, 0, 1))) hint(origin: (-0.110911353, 20, 8.93063091))
stand.origin coincident plate.far.surface
stand tangent(at: stand.origin) plate.far.surface

centre := point hint((-8.4, -6.7)) in stand
foot := circle(center: centre) hint(r: 21.7) in stand
boss := solid(face(foot), depth: 10mm)
plate union boss

boss.plate.far chamfer(d: 2mm) boss.foot
