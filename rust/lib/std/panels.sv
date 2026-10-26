// `std.panels`: developable panels, and a panel laid flat.

// -- developable panels (#645) --------------------------------------------------------------------
//
// A sheet cut flat and bent to shape — a plywood boat's panel, a duct adapter, a rectangle to a
// round — comes off flat without stretching only where it is developable: ruled, with one tangent
// plane along each straight ruling.  These words say both things as definitions.  The core
// recognises what their bodies say, `std`'s or written by hand, and builds them exactly: the
// panel's rulings by the correspondence along its rails, its band as a solid, and its development
// laid in a plane.
//
//   panel := std.panels.Developable(chine, sheer)              // a set: what the panel is
//   skin := { q | q distance(<= 6mm, side: outside) panel }    // the sheet, 6 thick
//   side_panel := solid(skin)
//   flat := std.panels.Development(panel, s, d: 3mm) in sheet  // its neutral layer laid flat
//   flat.f coincident corner                                   // a figure with three freedoms,
//   line(flat.f, flat.g) parallel edge                         // placed by relations

// A developable panel between the paths `a` and `b`: the points on the ruling between a point
// `m` of `a` and a point `n` of `b` where one plane `T` touches both — `b` running through it the
// way `a` does (`sense: same`, so not the crossing ruling, a bow-tie that cuts through itself).
// The ruling at each place is solved, not paired by written order.  At a corner of a chain the
// plane supports it, between its two one-sided tangents, so the rulings fan from the corner; a
// rail's end is a corner whose other side is open, so where the regular rulings do not reach it the
// end's fan does.  The panel's outside is the side `T`'s normal points to as the rails' order
// orients it: for a duct whose lower rim runs counter-clockwise seen from above, with `b` the upper
// rim, the outside is out.  A distance from it is measured along its normal from a foot on it, so
// its band (`q distance(<= d, side: outside) panel`) is square at its edges.
//
// Closed form, a frustum: coaxial circles of radii 50 and 30, 100 apart — every ruling a
// generator, slant ℓ = √(20² + 100²) = 101.980, the panel's area π (50 + 30) ℓ = 25630.47.
component Developable(a: path, b: path) := { p |
  m := point
  n := point
  T := plane
  m coincident a
  n coincident b
  T tangent(at: m) a                        // one plane touches a at m ...
  T tangent(at: n, sense: same) b           // ... and b at n, b running through it the way a does
  p coincident(t in (0, 1)) line(m, n)      // ... and p is on the ruling between them
}

// The panel `S` laid flat: its layer `d` along the normal of `S` (a `Developable`) laid in the
// plane the instance is drawn in, outside up, from its home — the ruling through `s` on `S.a`, the
// seam of a band — whose ends `f` and `g` are drawn points of that plane.  An isometry along
// rulings, stated and nothing else: the strip between two neighbouring rulings of a developable is
// planar, so it lays flat congruent, and given its four sides and its angle at `a` a planar
// quadrilateral is determined — so each ruling keeps its length, each rail its length, and `a`
// meets each ruling at its angle.  The instance is a figure with three freedoms (four coordinates,
// one length): relations place it.  It makes `a` and `b`, the laid rails — curves of its plane over
// `u` from 0 to 1, the panel's start to its finish — and the end rulings `start` and `end`; a band
// cut at its seam, `seam` and `seam_end` too.  The other face up is the rails given the other way
// round.
//
// Closed form, the frustum above: an annular sector of radii 50ℓ/20 = 254.951 and 30ℓ/20 =
// 152.971 about one centre, through 2π·20/ℓ = 1.23224 rad (70.60°); its layer `d` out, radii
// 5·d more each, through the same angle.
component Development(S: set, s: point, d: Length, u: Scalar) {
  private m := point in space               // the ruling of `S` through `s` ...
  private n := point in space
  private T := plane
  m coincident S.a
  n coincident S.b
  T tangent(at: m) S.a
  T tangent(at: n, sense: same) S.b
  private mo := point in space              // ... moved `d` along `T`'s normal: the layer
  private no := point in space
  mo distance(d, along: n) T
  no distance(d, along: n) T
  line(m, mo) perpendicular T
  line(n, no) perpendicular T
  private ruling := line(mo, no)
  f := point                                // its image in the plane
  g := point
  image := line(f, g)
  image equal ruling                                    // each ruling keeps its length
  rate(f over u) equal rate(mo over u)                  // a keeps its length
  rate(g over u) equal rate(no over u)                  // b keeps its length
  rate(f over u) angle(rate(mo over u), ruling) image   // a meets each ruling at its angle
  initially {
    m coincident s                          // the home: the ruling through s
  }
}
