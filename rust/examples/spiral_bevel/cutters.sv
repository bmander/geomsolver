// Close one active rounded flank far from the working blank. The other flank
// of a reference crown tooth is not necessarily a boundary of its mate's space.
// The walk enters the flank from whichever end meets its round, so one section
// serves an outer flank (round after) and an inner flank (round before).
component FlankSection(f: plane, base: line, flank: line, corner: arc,
                       tip: line, limit: Length) {
  in f {
    private point base_end
    private point tip_end
    base_end on base
    tip_end on tip
    base_end distance(limit, along: u) f
    tip_end distance(limit, along: u) f
    face profile(base_end, flank, corner, tip_end, -> close)
  }
}

component ComplementarySpace(f: plane, axis: line, outside: group, inside: group,
                              indexing: motion, radial_start: Length, radial_end: Length) {
  private outer: FlankSection(f, outside.base, outside.inner,
    outside.inner_round, outside.tip, limit: radial_end)
  private inner: FlankSection(f, inside.base, inside.outer,
    inside.outer_round, inside.tip, limit: radial_start)
  private construction solid outer_crown(outer.profile, about: axis)
  private construction solid inner_crown(inner.profile, about: axis)
  private construction solid neighbor(inner_crown, under: indexing, at: 0deg)
  // The space is the outer crown within its indexed neighbour.
  solid body(outer_crown)
  neighbor bound body
}
