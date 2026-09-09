// Close one active rounded flank far from the working blank. The other flank
// of a reference crown tooth is not necessarily a boundary of its mate's space.
component OuterFlankSection(f: plane, base: line, flank: line, corner: arc,
                            tip: line, limit: Length) {
  in f {
    private point base_end
    private point tip_end
    base_end on base
    tip_end on tip
    base_end distance(limit, along: u) f
    tip_end distance(limit, along: u) f
    private line lower(base_end, flank.p1)
    private line upper(corner.end, tip_end)
    private line closure(tip_end, base_end)
    face profile(lower, flank, corner, upper, closure)
  }
}

component InnerFlankSection(f: plane, base: line, flank: line, corner: arc,
                            tip: line, limit: Length) {
  in f {
    private point base_end
    private point tip_end
    base_end on base
    tip_end on tip
    base_end distance(limit, along: u) f
    tip_end distance(limit, along: u) f
    private line lower(flank.p2, base_end)
    private line closure(base_end, tip_end)
    private line upper(tip_end, corner.start)
    face profile(lower, closure, upper, corner, flank)
  }
}

component ComplementarySpace(f: plane, axis: line, outside: group, inside: group,
                              indexing: motion, radial_start: Length, radial_end: Length) {
  private outer: InnerFlankSection(f, outside.base, outside.inner,
    outside.inner_round, outside.tip, limit: radial_end)
  private inner: OuterFlankSection(f, inside.base, inside.outer,
    inside.outer_round, inside.tip, limit: radial_start)
  private construction solid outer_crown(outer.profile, about: axis)
  private construction solid inner_crown(inner.profile, about: axis)
  private construction solid neighbor(inner_crown, under: indexing, at: 0deg)
  // Intersection expressed with the ordinary body rule.
  private construction solid outside_neighbor(outer_crown)
  neighbor cut outside_neighbor
  solid body(outer_crown)
  outside_neighbor cut body
}
