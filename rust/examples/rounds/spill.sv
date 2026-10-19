// A chamfer that spills onto a round (#419). A plate's far rim is rounded, and a boss drawn in a
// plane of its own — set on the plate's far face, touching it — stands near the rim, its foot
// chamfered against the far face. The chamfer's legs are 2: out from the boss's wall that is past
// where the far face goes on tangent into the rim's round, so the leg runs on across the seam and
// the flat lands on the round (std's sets stand against the face's smooth surface, `F.smooth`).
//
// Built exactly (#422): the flat stands against whichever face of the chain is nearest, so it
// changes face where it crosses the seam's normal surface — the cylinder through the seam along the
// far face's normal. Short of it the chamfer is the ring it always was, a cone, kept within that
// cylinder; beyond it the flat against the round, sectioned in the half-planes through the boss's
// axis and fitted, glued to the ring on the cylinder. `solventc --step` writes a cone, one fitted
// flat and the plate's and the boss's faces: 85358.075 mm³. Edit the boss's radius (21 to 21.7 all
// build; at 19.7 or less the leg stays short of the seam and the chamfer is the ring alone) and
// apply (⌘↵). Write `round(r: 2mm)` for the chamfer and the ball touches the round past where its
// centre's foot crosses the seam: a ring's stretch, and between its ends the ball rolled against the
// round, glued to it there.
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
