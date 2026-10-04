// The drill's numbers: a jobber's 10 mm twist drill, two flutes, a 30° helix and a 118° point,
// stated once. `drill.sv` builds the drill from them; the tests rewrite them (one flute, a short
// length, no point) for a fixture that exports in seconds.
unit mm

// -- the drill ------------------------------------------------------------------------------
diameter := 10mm
helix := 30deg          // the flute's helix angle at the drill's diameter
flutes := 2             // 1 or 2
fluted_length := 40mm
shank_length := 30mm
web := 0.15 * diameter  // how thick the core is between the flutes
lead := pi * diameter / tan(helix)

// -- the flute wheel ---------------------------------------------------------------------------
// A wheel whose rim is a round between two flanks leaning `flank` from its radius, its axis
// square to the helix where it grinds, sunk to half the web from the drill's axis. Steeper flanks
// (below about 30°) fold the flute's wall, and a shallower setting reaches the blank twice: the
// export refuses both, by the row they fail.
flute_wheel := {rim: 25mm, nearest: web / 2, round: 2.5mm, flank: 35deg, depth: 7mm}

// -- the margin and the body clearance --------------------------------------------------------
// Behind each land's leading edge a margin stays at the full diameter; the rest of the land is
// ground back `clearance` by a wide round wheel on the same screw, turned `clearance_phase`
// ahead of the flute wheel.
clearance := 0.25mm
clearance_wheel := {rim: 45mm, nearest: diameter / 2 - clearance, round: 39mm, flank: 35deg, depth: 18mm}
clearance_phase := 82.5deg

// -- the point --------------------------------------------------------------------------------
// Each lip's flank is a cone (a conical point): half-angle `cone`, its axis tilted `tilt` from
// the drill's axis behind the lip, its apex `apex_height` above the lip's corner and `apex_offset`
// back along the lip. These four give the 118° point angle and a 12° lip relief at the corner,
// with the chisel edge about 110° from the lip, worked out for this diameter. `lip` is where the
// flute's leading edge meets the diameter at the shank end, read off its section square to the
// axis; the lips follow the helix to the point.
point := 1              // 1 grinds the point, 0 leaves the end square
point_angle := 118deg
point_length := diameter / 2 / tan(point_angle / 2)   // from the lips' corners to the tip
lip_relief := 12deg
cone := 59.41142deg
tilt := 8.8765deg
apex_height := 0.330472 * diameter
apex_offset := -0.05 * diameter
lip := 128.17deg
