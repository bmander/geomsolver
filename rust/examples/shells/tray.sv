// An open tray: a box 60 × 40 and 20 tall, hollowed to walls `wall` thick and open through its
// top. `tray.near` names the face left open — the prism's cap at its top — and every other face
// keeps a wall: the bottom and the four sides, their inner faces meeting square at the corners.
//
// Built exactly: the cavity is a prism, the section's edges each moved in by the wall and its top
// moved out through the opening, so it crosses the rim and lies on none of the box's faces.
// 11712 mm³: 48000 less the cavity's 56 × 36 × 18. Edit `wall` and apply (⌘↵).
unit mm
use std
use std.rounds (shell)

wall := 2mm

in std.top {
  b0 := point
  b1 := point
  b2 := point
  b3 := point
  fix((0, 0)) b0
  fix((60, 0)) b1
  fix((60, 40)) b2
  fix((0, 40)) b3
  (south := line(b0, b1)) -> (east := line(b1, b2)) -> (north := line(b2, b3)) -> (west := line(b3, b0)) -> close
  outline := face(south, east, north, west)
}
tray := solid(outline, from: 0mm, to: 20mm)

tray.near shell(t: wall) tray
