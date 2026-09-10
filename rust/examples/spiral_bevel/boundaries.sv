// Analytic boundary components. These are nominal geometry, independent of manufacture.

// A cone about `axis` with its apex at `apex`, a point of `f`. The caller's `in f`
// block places the instance in the view. Runs and rises are measured from the apex
// along the page's x and y, which are the plane's own u and v: every datum here is
// the page's, and the apex need not be the datum.
component ConeBoundary(f: plane, apex: point, axis: line, axis_bearing: Angle, half_angle: Angle,
                       normal_offset: Length, span: group) {
  // The meridian is parallel to the pitch-cone generator and offset along its normal.
  // The revolution is about the member's own axis line, so the cone is coaxial with
  // the motions declared about that line; `axis_bearing` is where that line points.
  param near = span.near
  param far = span.far
  param bearing = axis_bearing - half_angle
  param near_axial = near * cos(half_angle) - normal_offset * sin(half_angle)
  param far_axial = far * cos(half_angle) - normal_offset * sin(half_angle)
  // Seeded at their closed forms, so the joint solve starts where it ends.
  private point a hint(x: apex.x + near_axial * cos(axis_bearing), y: apex.y + near_axial * sin(axis_bearing))
  private point b hint(x: apex.x + far_axial * cos(axis_bearing), y: apex.y + far_axial * sin(axis_bearing))
  private point p hint(x: apex.x + near * cos(bearing) + normal_offset * sin(bearing),
                       y: apex.y + near * sin(bearing) - normal_offset * cos(bearing))
  private point q hint(x: apex.x + far * cos(bearing) + normal_offset * sin(bearing),
                       y: apex.y + far * sin(bearing) - normal_offset * cos(bearing))
  apex distance(near_axial * cos(axis_bearing), along: x) a
  apex distance(near_axial * sin(axis_bearing), along: y) a
  apex distance(far_axial * cos(axis_bearing), along: x) b
  apex distance(far_axial * sin(axis_bearing), along: y) b
  apex distance(near * cos(bearing) + normal_offset * sin(bearing), along: x) p
  apex distance(near * sin(bearing) - normal_offset * cos(bearing), along: y) p
  apex distance(far * cos(bearing) + normal_offset * sin(bearing), along: x) q
  apex distance(far * sin(bearing) - normal_offset * cos(bearing), along: y) q
  private profile = line near_cap(a,p) -> line meridian(p,q) ->
                    line far_cap(q,b) -> line spine(b,a) -> close
  private construction solid carrier(profile, about: axis)
  surface wall(carrier, meridian)
}

// A sphere about `apex`, a point of `f`, its poles along the plane's v.
component SphericalBoundary(f: plane, apex: point, size: Length) {
  // The seeds select opposite poles. Incidence and radius determine the sphere.
  private point bottom hint(x: apex.x, y: apex.y - size)
  private point top hint(x: apex.x, y: apex.y + size)
  apex distance(0mm, along: x) bottom
  apex distance(0mm, along: x) top
  private arc meridian(center: apex, start: bottom, end: top)
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
