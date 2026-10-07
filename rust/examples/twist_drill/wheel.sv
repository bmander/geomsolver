// A grinding wheel: a revolution whose rim, the part that grinds, is a round between two straight
// flanks, each tangent to it. Its axis lies square to the drill's axis and to the helix it grinds
// along, `setting` from the drill's side datum; its rim comes within `nearest` of the drill's
// axis. Every face it grinds with meets the next one tangent, so its characteristic under a
// screw runs smoothly from face to face (docs/generating-sweeps.md, the constant-twist class).
use std (coords)

// `wheel` gives the wheel's radius to its rim (`rim`), how near the drill's axis the rim comes
// (`nearest`), the round's radius (`round`), the flanks' lean from the wheel's radius (`flank`),
// and how far up from the rim they run (`depth`) before the wheel's sides close it to the hub.
component GrindingWheel(side: plane, setting: Angle, wheel: group) {
  centre := wheel.rim + wheel.nearest
  // where the round meets each flank, and where the flanks end, across the wheel's mid-plane
  tangent_across := wheel.round * cos(wheel.flank)
  tangent_up := wheel.nearest + wheel.round - wheel.round * sin(wheel.flank)
  flank_across := tangent_across + (wheel.nearest + wheel.depth - tangent_up) * tan(wheel.flank)

  // the wheel's axial plane: square to the side datum on a line `setting` from its vertical, its v
  // out of the side datum the way its normal does not point
  in side {
    private o := point
    private h := point
    o coincident side.origin
    h coords(du: -10mm * sin(setting), dv: 10mm * cos(setting)) side
    private construction hinge := line(o, h)
  }
  private out := axis hint(dir: (-1, 0, 0))
  out perpendicular side
  section := plane(u: hinge, v: out)
  side.origin coincident section.origin

  in section {
    private a0 := point
    private a1 := point
    a0 coords(du: -10mm, dv: centre) section
    a1 coords(du: 10mm, dv: centre) section
    construction centerline ax := line(a0, a1)
    private round_centre := point hint((0, wheel.nearest + wheel.round))
    private tl := point
    private tr := point
    tl coords(du: -tangent_across, dv: tangent_up) section
    tr coords(du: tangent_across, dv: tangent_up) section
    private fl := point
    private fr := point
    fl coords(du: -flank_across, dv: wheel.nearest + wheel.depth) section
    fr coords(du: flank_across, dv: wheel.nearest + wheel.depth) section
    private hl := point
    private hr := point
    hl coords(du: -flank_across, dv: centre) section
    hr coords(du: flank_across, dv: centre) section
    round := arc(center: round_centre, start: tl, end: tr) hint(r: wheel.round)
    radius(wheel.round) round
    right_flank := line(tr, fr)
    right_side := line(fr, hr)
    private hub := line(hr, hl)
    left_side := line(hl, fl)
    left_flank := line(fl, tl)
  }
  body := solid(face(round, right_flank, right_side, hub, left_side, left_flank), about: hub)
}

preview {
  unit mm
  // the drill's side datum: u down the drill's axis, v along y, so x is its normal
  down := axis
  fix(dir == (0, 0, -1)) down
  side := plane(u: down, v: std.y)
  fix(origin == (0, 0, 0)) side
  sizes := {rim: 25mm, nearest: 0.75mm, round: 2.5mm, flank: 35deg, depth: 7mm}
  wheel := GrindingWheel(side, setting: 30deg, wheel: sizes)
}
