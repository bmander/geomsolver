// One flank of a conical point: a cone the drill is kept within, as a point grinder leaves it.
// Drawn for a lip along the side datum's normal (x) at the top of a drill of radius `r` standing
// on z: its apex `height` above the lip's corner and `offset` along the lip, its axis tilted
// `tilt` from the drill's (towards y, behind the lip as the drill turns), and its half-angle
// `cone`. It reaches `reach` down the drill's axis from the corner, past everything it keeps.
use std

component PointCone(side: plane, top: Length, cone: Angle, tilt: Angle, height: Length,
                    offset: Length, reach: Length) {
  // the plane through the apex holding the cone's axis: the side datum, stood off along x
  axial := plane(origin: side.origin, toward: side.toward, from: side, offset: offset)
  in axial {
    // the side datum's u runs down the drill's axis and its v along y
    private apex := point
    apex distance(-(top + height), along: u) axial
    apex distance(0mm, along: v) axial
    private foot := point
    foot distance(-(top + height) + reach * cos(tilt), along: u) axial
    foot distance(reach * sin(tilt), along: v) axial
    private rim := point
    rim distance(-(top + height) + reach * cos(tilt) + reach * tan(cone) * sin(tilt), along: u) axial
    rim distance(reach * sin(tilt) - reach * tan(cone) * cos(tilt), along: v) axial
    private construction centerline axis := line(apex, foot)
    private base := line(foot, rim)
    flank := line(rim, apex)
  }
  body := solid(face(axis, base, flank), about: axis)
}

preview {
  unit mm
  tip := PointCone(std.side, top: 0mm, cone: 59.4deg, tilt: 8.9deg, height: 3.3mm, offset: -0.5mm, reach: 20mm)
}
