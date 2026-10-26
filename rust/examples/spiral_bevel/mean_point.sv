// The lean a face mill's blades need for a stated pressure angle at the mean point, solved: step 2
// of #283. The pinion's flank is the envelope of the gear's flank under the mesh, an envelope the
// solve sees, so the contact a designer states at M — M on the pinion's flank, its normal there at
// the pressure angle to the pitch plane — holds the cutter's number that makes it. Nothing here
// states the lean; it is `flank.half`, an unknown of the drawing.
//
// The mesh is the hypoid's at its mean point reduced to its kinematics: two shafts square to one
// another and apart, the pitch surfaces touching at M, the teeth running along the sliding there.
// Here those surfaces are cylinders — crossed helical gears, 45° teeth on both — so the answer has
// a closed form to be read against: the cutter's axis leans `tilt` from the pitch plane's normal in
// the plane of that normal and the teeth's normal, and the cone's normal at M lies in that plane,
// so the blades lean the pressure angle less the tilt (`tests/held_envelopes.rs`). The pinion
// flank's profile at M is the circle osculating its section by the normal plane through `across`:
// the curvature an envelope's contact reads off its source and the mesh, with no second solve.
unit mm
use std
use std.sets
use std.motion (turns)

param pinion_teeth := 24
param gear_teeth := 48
param module := 2mm
param pressure := 20deg
param tilt := 5deg
param cutter_radius := 40mm

rg := module * gear_teeth / 2
rp := module * pinion_teeth / 2

// -- the mesh ------------------------------------------------------------------------------------
// The gear turns about the z axis, the pinion about a line along x, the two pitch radii apart; the
// mesh is the gear's turn seen from the pinion's, the ratio their tooth counts.
g0 := point hint((0, 0, 0))
g1 := point hint((0, 0, 10))
fix((0, 0, 0)) g0
fix((0mm, 0mm, 10mm)) g1
gear_ax := line(g0, g1)
p0 := point hint((0, 72, 0))
p1 := point hint((10, 72, 0))
fix((0mm, rg + rp, 0mm)) p0
fix((10mm, rg + rp, 0mm)) p1
pinion_ax := line(p0, p1)
gear_turn := frame
gear_turn turns(-pinion_teeth / gear_teeth) gear_ax
pinion_turn := frame
pinion_turn turns(1) pinion_ax

// -- the mean point ------------------------------------------------------------------------------
// M, where the pitch cylinders touch; the pitch plane is square to the y axis there. The gear's
// pitch line runs along x at M and the pinion's along z, a pitch velocity each, so the teeth slide
// along the diagonal between them and `across` is square to the teeth in the pitch plane.
M := point hint((0, 48, 0))
fix((0mm, rg, 0mm)) M
toward := point hint((30, 48, -30))
fix((30mm, rg, -30mm)) toward
across := line(M, toward)

// -- the face mill -------------------------------------------------------------------------------
// Its centre C on `across`, the cutter's radius from M, so the tooth trace is the cutter's circle;
// its axis through C, leaning `tilt` from the pitch plane's normal in the plane of that normal and
// `across`.
C := point hint((28, 48, -28))
C coincident across
M distance(cutter_radius) C
normal_plane := plane(u: across, v: std.y)
top := point hint((27, 58, -27))
top coincident normal_plane
C distance(10mm) top
mill_ax := line(C, top)
mill_ax angle(tilt) std.y

// The gear's flank: a cone about the mill's axis, its apex on the axis and its half — the blades'
// lean — unknown. M is on it where it meets the pinion's, which nothing here says: the contact does.
apex := point hint((35, -60, -35))
apex coincident mill_ax
spine := line(apex, C)
flank := std.sets.Cone(spine, half: hint(20deg))

// -- the contact ---------------------------------------------------------------------------------
// The pinion's flank: the gear's generated under the mesh. M is on it, and its normal there leans
// the pressure angle from the pitch plane.
pinion_flank := envelope(flank over t in (-30deg, 30deg), under: gear_turn, seen_from: pinion_turn)
M coincident pinion_flank
n := point hint((7, 58, -7))
contact_normal := line(M, n)
pinion_flank perpendicular(at: M) contact_normal
M distance(10mm) n
contact_normal angle(90deg - pressure) std.y

// The pinion flank's profile at M: the circle osculating its section by the plane of the contact
// normal and `across`.
section := plane(u: across, v: contact_normal)
in section {
  centre := point hint((0, 30))
  profile := circle(center: centre) hint(r: 30)
}
pinion_flank curvature(at: M) profile
