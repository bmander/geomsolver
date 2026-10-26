// The standard library: what most drawings start from, written once.
//
// `use std` brings in its root, this file: the standard datums every drawing stands on.  The rest
// of the library is in modules under it, each asked for by its own `use` and named by its path
// (`use std.sets`, `std.sets.Sphere(c, r: 12mm)`), or imported bare where the `use` lists it
// (`use std.place (horizontal, vertical)`):
//
//   std.place     where points, lines and planes stand: `horizontal`, `vertical`, `offset`,
//                 `right_of`, `coords`, `on_u`, `skew`, `turned`, `square`, `seats`, `raised`
//   std.shapes    figures drawn in a plane: `CenteredRectangle`, `Ellipse`, `Polygon`, `Hex`,
//                 and a rope that `hangs`
//   std.sets      surfaces as sets of points: `Sphere`, `Cylinder`, `Cone`, `ConeThrough`,
//                 `HalfSpace`, `Offset`, and a circle `on_sphere`
//   std.rounds    rounds and chamfers: `round`, `chamfer`, `Rounded`, `Chamfered`, `Grown`
//   std.panels    developable panels: `Developable`, `Development`
//   std.stations  where a section stands along a sweep or a loft: `On`, `OnLine`, `Helix`,
//                 `Along`, `Ruled`, `Twisted`, `scaled`, …
//   std.motion    a frame's joint and its twist: `turns`, `screws`, `spins`, `slides`, `rolls`,
//                 `reciprocal`, `Screw`
//
// Each comes from the library compiled into the core, so it is there in the browser as in the
// terminal; a `std.sv`, or a `std/sets.sv`, beside a document would win over it, as any module
// does.

// The standard axes and planes, which every document that says `use std` has — as a CAD part
// has its origin planes — so the workspace can offer them as places to draw.  The axes are held
// outright, each through the world origin: x right, y away from the front's viewer, z up, and
// `back` the x axis reversed.  Each plane is two of them, and so stands where they meet, at the
// world origin: front is x right and z up (a document's 2D drawing, `in std.front`), top x right
// and y away (looked at from above), side y right and z up (looked at from +x), and up the front
// turned a quarter, z right and x to the left.  `origin` is a point drawn in the front plane at
// its origin.
component StandardDatums() {
  x := axis
  fix(dir == (1, 0, 0), origin == (0, 0, 0)) x
  y := axis
  fix(dir == (0, 1, 0), origin == (0, 0, 0)) y
  z := axis
  fix(dir == (0, 0, 1), origin == (0, 0, 0)) z
  back := axis
  fix(dir == (-1, 0, 0), origin == (0, 0, 0)) back
  front := plane(u: x, v: z)
  top := plane(u: x, v: y)
  side := plane(u: y, v: z)
  up := plane(u: z, v: back)
  origin := point in front
  fix((0, 0)) origin
}

// The standard datums of the frame `f` (#542): what `std` names in a definition's body whose
// instance stands in `f`, made once a frame when such a body first reads one.  They are the
// world's read along the frame's own axes: x is `f.u`, y `f.v` and z `f.normal`, which a body
// reads as those, and each plane is two of them, as the world's are, standing where the frame
// does — its place is the frame's, so it adds nothing.  `back`, the frame's x reversed, is the
// normal of `behind`, square to the front's normal and z.  A point in `f` at `(x, y, z)` stands at
// `origin + x·u + y·v + z·normal`, and a frame at the world's attitude and origin has the world's
// datums.
component FrameDatums(f: frame) {
  top := plane(u: f.u, v: f.v)
  front := plane(u: f.u, v: f.normal)
  side := plane(u: f.v, v: f.normal)
  behind := plane(u: front.normal, v: f.normal)
  up := plane(u: f.normal, v: behind.normal)
  origin := point in front
  fix((0, 0)) origin
}

// Axes turned within a plane: the plane through `o` whose u runs toward `t` and whose v is a
// quarter turn on, anticlockwise, in the plane `o` and `t` are drawn in — what a part standing at an angle is
// measured against (`p distance(d, along: u) axes.axes`) and seeded in (`hint(at: axes.axes,
// x: 3, y: 4)`).  Draw it in that plane: `axes := std.Turned(o, t) in std.front`; `axes.u` is
// the line from `o` to `t`.  The two lines share `o`, so the plane stands there unsaid.  The
// turn says its sense: square and equal alone have the mirrored frame as a second root, which a
// datum turned a quarter or more in one solve can land on (#497).
component Turned(o: point, t: point) {
  construction u := line(o, t)
  private q := point hint(at: o, toward: t, turn: 90deg)
  private construction v := line(o, q)
  u perpendicular(sense: ccw) v
  v equal u
  axes := plane(u: u, v: v)
}
