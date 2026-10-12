// Superseded by `rust/lib/std.sv` (#183 steps 1–7): what follows is the paper it was written
// from; docs/fillets/README.md lists the differences.
//
// X3 (#183): rounds and chamfers as they would stand in `rust/lib/std.sv`, written in Solvent over
// words the language does not have yet. Does not parse. Each line that needs a missing feature is
// marked [F1]–[F9]; docs/fillets/README.md (X3) lists the features, each with a use that has
// nothing to do with fillets.
//
// The meaning is X2's (a + v): one term per named edge, and one per corner where three named
// edges meet. The reach is said in the point's own distances (X3's `Reach::Edge`, held to every
// closed form X2 checked by `rust/gcs-core/tests/fillet_definition.rs`). Nothing here is a
// Rust construction: an evaluator reads these sets (the field), and lowering entries build exact
// surfaces for the shapes they recognise, never changing what the sets say.

// -- one edge ----------------------------------------------------------------------------------

// Where the surfaces of faces `a` and `b` meet, and the part of that which is the body's edge.
// A face is its patch (`q coincident a`: on the face itself) and has a surface (`a.surface`: the
// whole plane, cylinder or cone it lies on); `p inside a` is the surface's material side.
component Meeting(a: face, b: face) := { q |                                       // [F1]
  q coincident a.surface
  q coincident b.surface
}
component Edge(a: face, b: face) := { q |                                          // [F1]
  q coincident a
  q coincident b
}

// What a ball of radius `r` rolled along the convex edge of `a` and `b` cannot reach: the points
// within both faces' material sides that no such ball, lying within both, covers; and only where
// the point's own nearest point of the faces' meeting is on the edge, so carriers meeting where
// the body has no edge (two cylinders' far crossing, a cap a cut ate, a wall's plane past its
// tangent seam) make nothing.
component Run(a: face, b: face, r: Length) := { p |
  p inside a                                                                       // [F1]
  p inside b
  // a convex edge: the material's angle at `p`'s nearest point of the edge
  angle(a, b, at: p) < 180deg                                                      // [F9]
  // the balls' centres, r or more within both faces
  private centres := { c |
    c inside a
    c distance(>= r) a.surface                                                     // [F1]
    c inside b
    c distance(>= r) b.surface
  }
  // no ball covers `p`: it is farther than `r` from every centre
  p outside std.Offset(centres, d: r)                                              // [F2]
  // the ball is the ball of a point of the edge
  distance(p, Edge(a, b)) <= distance(p, Meeting(a, b))                            // [F2, F3]
  // a run that ends on a face that is not rounded is cut off by it
  p inside Edge(a, b).ends                                                         // [F7]
}

// The same at a concave edge, every side turned round: what the ball rolled outside both cannot
// reach is added.
component Fill(a: face, b: face, r: Length) := { p |
  p outside a
  p outside b
  angle(a, b, at: p) > 180deg                                                      // [F9]
  private centres := { c |
    c outside a
    c distance(>= r) a.surface
    c outside b
    c distance(>= r) b.surface
  }
  p outside std.Offset(centres, d: r)                                              // [F2]
  distance(p, Edge(a, b)) <= distance(p, Meeting(a, b))                            // [F2, F3]
  p inside Edge(a, b).ends                                                         // [F7]
}

// -- a corner ----------------------------------------------------------------------------------

// Where three rounded edges meet: the ball touching all three faces, a sphere's patch (X2's
// finding 3, a term of its own). Only at a corner of the body: the point of the three faces is
// one of its points.
component Corner(a: face, b: face, c: face, r: Length) := { p |
  p inside a
  p inside b
  p inside c
  angle(a, b, at: p) < 180deg                                                      // [F9]
  private centres := { o |
    o inside a
    o distance(>= r) a.surface
    o inside b
    o distance(>= r) b.surface
    o inside c
    o distance(>= r) c.surface
  }
  p outside std.Offset(centres, d: r)                                              // [F2]
  private vertex := { q | q coincident a; q coincident b; q coincident c }
  distance(p, vertex) <= distance(p, { q | q coincident a.surface; q coincident b.surface;
                                           q coincident c.surface })               // [F2, F3]
}
// `CornerFill` is `Corner` with every side turned round, as `Fill` is `Run`'s.

// -- the words ---------------------------------------------------------------------------------

// `A round(r) B`: every edge where a face of `A` meets a face of `B`, rounded by a ball of radius
// `r`, taken away where it is convex and filled where it is concave. A pair of faces that do not
// meet makes nothing (`Run`'s reach), so the word need not ask which do. Its material is a
// solid of the body the faces are faces of, cut from it or added to it, acting on the body as made
// by the terms that make those faces.
A round(r) B := {
  repeat a in A {                                                                  // [F4]
    repeat b in B {
      taken := solid(Run(a, b, r))                                                 // [F5]
      taken cut A.body                                                             // [F5, F6]
      filled := solid(Fill(a, b, r))
      filled union A.body
    }
  }
  // corners: three faces each pair of which the word names
  repeat a in A {
    repeat b in B {
      repeat c in A, B if named(a, c) and named(b, c) {                            // [F8]
        corner := solid(Corner(a, b, c, r))
        corner cut A.body
      }
    }
  }
}

// `round(r) X`: every edge among `X`'s faces.
round(r) X := X round(r) X

// -- chamfers ----------------------------------------------------------------------------------

// A chamfer's flat: the points within both faces nearer them than the flat through the point of
// each face its leg from the other face (X1's finding 4) — a bound on a sum of two distances. Between
// planes it is the flat; about an axis (a boss's foot, a turned rim) the cone; on a sphere it
// bows by the sphere's curvature (7e-5 mm on the hypoid's 0.2 mm end relief). Three chamfered
// edges at a corner meet in a point: no corner term.
component Flat(a: face, b: face, la: Length, lb: Length) := { p |
  p inside a
  p inside b
  angle(a, b, at: p) < 180deg                                                      // [F9]
  distance(p, a.surface) / lb + distance(p, b.surface) / la < 1                    // [F3]
  distance(p, Edge(a, b)) <= distance(p, Meeting(a, b))                            // [F2, F3]
  p inside Edge(a, b).ends                                                         // [F7]
}
// `FlatFill` is `Flat` with every side turned round.

// `A chamfer(d) B`: `d` each way; `A chamfer(la, lb) B`: `la` on `A`'s faces, `lb` on `B`'s.
A chamfer(la, lb) B := {
  repeat a in A {                                                                  // [F4]
    repeat b in B {
      taken := solid(Flat(a, b, la, lb))                                           // [F5]
      taken cut A.body                                                             // [F5, F6]
      filled := solid(FlatFill(a, b, la, lb))
      filled union A.body
    }
  }
}
A chamfer(d) B := A chamfer(d, d) B

// -- in a region's body ---------------------------------------------------------------------------
//
// A region is a conjunction, so every edge where two of its bounds meet is convex (an
// intersection's material angle is under a half turn): only `Run` and `Flat` apply, and the words
// stand for one more bound on the region's point — no solid, no body.
//
//   blank := { p |
//     p inside tip; p inside heel; p outside toe
//     tip chamfer(design.end_relief) toe     // p outside Flat(tip, toe, d, d)
//     tip chamfer(design.end_relief) heel
//   }
//
// Here the faces are the region's own bounding sets (a set is its own surface), as X1 lowered them.

// -- what the X0 programs then say, unchanged ----------------------------------------------------
//
//   pad.boss round(3mm) pad.plate          // x0/pad.sv: the boss's root
//   round(3mm) block                       // x0/block.sv: every edge, corners spherical
//   block.near round(3mm) block            // x0/lid.sv: the top rim, mitred at sharp corners
//   tip chamfer(design.end_relief) toe     // x0/hypoid_blank.sv: in a region's body
