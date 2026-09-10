// Analytic boundary components. These are nominal geometry, independent of manufacture.

component ConeBoundary(f: plane, axis: line, axis_bearing: Angle, half_angle: Angle,
                       normal_offset: Length, span: group) {
  // The meridian is parallel to the pitch-cone generator and offset along its normal.
  // The revolution is about the member's own axis line, so the cone is coaxial with
  // the motions declared about that line; `axis_bearing` is where that line points.
  param near = span.near
  param far = span.far
  param bearing = axis_bearing - half_angle
  param near_axial = near * cos(half_angle) - normal_offset * sin(half_angle)
  param far_axial = far * cos(half_angle) - normal_offset * sin(half_angle)
  private point a
  private point b
  private point p
  private point q
  a distance(near_axial * cos(axis_bearing), along: u) f
  a distance(near_axial * sin(axis_bearing), along: v) f
  b distance(far_axial * cos(axis_bearing), along: u) f
  b distance(far_axial * sin(axis_bearing), along: v) f
  p distance(near * cos(bearing) + normal_offset * sin(bearing), along: u) f
  p distance(near * sin(bearing) - normal_offset * cos(bearing), along: v) f
  q distance(far * cos(bearing) + normal_offset * sin(bearing), along: u) f
  q distance(far * sin(bearing) - normal_offset * cos(bearing), along: v) f
  private profile = line near_cap(a,p) -> line meridian(p,q) ->
                    line far_cap(q,b) -> line spine(b,a) -> close
  private construction solid carrier(profile, about: axis)
  surface wall(carrier, meridian)
}

component SphericalBoundary(f: plane, size: Length) {
  // The ordinate hints select opposite poles. Incidence and radius determine the sphere.
  private point bottom hint(y: -size)
  private point top hint(y: size)
  bottom distance(0mm, along: u) f
  top distance(0mm, along: u) f
  private arc meridian(center: f.origin, start: bottom, end: top)
  radius(size) meridian
  private line diameter(top,bottom)
  private face profile(meridian,diameter)
  private construction solid carrier(profile, about: diameter)
  surface wall(carrier, meridian)
}

// A surface region is the common part of these material-side conditions. Its generating
// envelope keeps its own parameter domain; branch selection and solid closure are separate.
component ToothRegion(source: envelope, limits: group) {
  patch bounded(source, inside: limits.tip, inside: limits.heel,
                outside: limits.root, outside: limits.toe)
}

// Exact end intersections share their supporting surfaces with the tooth boundaries.
component EndCircles(wall: surface, toe: surface, heel: surface) {
  seam near(wall, toe)
  seam far(wall, heel)
}

// The working flank and its root transition share one finite join edge.
// Loop order makes their uses of that edge opposite.
component ToothSideFaces(flank: patch, fillet: patch,
                         tip: edge, join: edge, root: edge,
                         toe: edge, heel: edge, round_toe: edge, round_heel: edge) {
  face working(toe, tip, heel, join, on: flank)
  face transition(round_toe, join, round_heel, root, on: fillet)
}
