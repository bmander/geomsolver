// `std.rounds`: rounds and chamfers along a solid's edges (`use std.rounds (round, chamfer)`),
// a solid grown, a solid shelled (`use std.rounds (shell)`), and a round or a chamfer as one more
// bound of a region.

// -- rounds and chamfers (#183) -----------------------------------------------------------------
//
// What a round or a chamfer means is said here, over the language's own words: faces as sets (a
// face is its patch, `q coincident a`; `a.surface` the whole surface it lies on; `p inside a` the
// surface's material side; `a.smooth` that surface carried on across every seam where a face of
// the body goes on from it tangent), the distance from a set, a bound on measurements, where an
// edge ends, the material's angle at an edge, words that repeat over faces and make solids.  An
// evaluator reads the sets (the field); a lowering may build an exact surface for a shape it
// recognises, and says nothing these do not.  Each is `symmetric` in its faces (#415): the edge of
// `a` and `b` is the edge of `b` and `a`, so a word that reaches it both ways states it once.
//
// The ball and the flat stand against each face's smooth surface (0.149): where a face goes on
// tangent into another — a round's own face, a fillet's run onto a bullnose — a ball rolled
// against it, or a leg measured along it, follows it across the seam rather than its plane run on
// into the air.  The edge itself is still the faces' own (`crease`, `meeting`).

// What a ball of radius `r` rolled along the convex edge of faces `a` and `b` cannot reach: the
// points within both faces' sides that no such ball within both covers; only where the point's
// nearest point of the faces' meeting, short of where the edge ends, is on the edge (so surfaces
// meeting where the body has no edge make nothing); so carried on past each end of the edge and
// cut off there by the faces it ends on, however they stand.
component Run(a: face, b: face, r: Length) symmetric(a, b) := { p |
  p inside a.smooth
  p inside b.smooth
  angle(a, b, at: p) <= 180deg
  centres := { c | c inside a.smooth; c distance(>= r) a.smooth; c inside b.smooth
               c distance(>= r) b.smooth }
  crease := { q | q coincident a; q coincident b }
  meeting := { q | q coincident a.surface; q coincident b.surface; q inside crease.ends }
  distance(p, centres) >= r
  distance(p, crease) <= distance(p, meeting)
  p inside crease.ends
}

// The same at a concave edge, every side turned round: what the ball rolled outside both faces
// cannot reach, added.
component Fill(a: face, b: face, r: Length) symmetric(a, b) := { p |
  p outside a.smooth
  p outside b.smooth
  angle(a, b, at: p) >= 180deg
  centres := { c | c outside a.smooth; c distance(>= r) a.smooth; c outside b.smooth
               c distance(>= r) b.smooth }
  crease := { q | q coincident a; q coincident b }
  meeting := { q | q coincident a.surface; q coincident b.surface; q inside crease.ends }
  distance(p, centres) >= r
  distance(p, crease) <= distance(p, meeting)
  p inside crease.ends
}

// Where three rounded convex edges meet at a corner of the body: what the ball touching all three
// faces cannot reach, within the three edges' extents.
component Corner(a: face, b: face, c: face, r: Length) symmetric(a, b, c) := { p |
  p inside a
  p inside b
  p inside c
  angle(a, b, at: p) <= 180deg
  angle(b, c, at: p) <= 180deg
  angle(c, a, at: p) <= 180deg
  centres := { o | o inside a; o distance(>= r) a.surface; o inside b; o distance(>= r) b.surface
               o inside c; o distance(>= r) c.surface }
  vertex := { q | q coincident a; q coincident b; q coincident c }
  meeting := { q | q coincident a.surface; q coincident b.surface; q coincident c.surface }
  ab := { q | q coincident a; q coincident b }
  bc := { q | q coincident b; q coincident c }
  ca := { q | q coincident c; q coincident a }
  distance(p, centres) >= r
  distance(p, vertex) <= distance(p, meeting)
  p inside ab.ends
  p inside bc.ends
  p inside ca.ends
}

// `Corner` at a concave corner, every side turned round.
component CornerFill(a: face, b: face, c: face, r: Length) symmetric(a, b, c) := { p |
  p outside a
  p outside b
  p outside c
  angle(a, b, at: p) >= 180deg
  angle(b, c, at: p) >= 180deg
  angle(c, a, at: p) >= 180deg
  centres := { o | o outside a; o distance(>= r) a.surface; o outside b; o distance(>= r) b.surface
               o outside c; o distance(>= r) c.surface }
  vertex := { q | q coincident a; q coincident b; q coincident c }
  meeting := { q | q coincident a.surface; q coincident b.surface; q coincident c.surface }
  ab := { q | q coincident a; q coincident b }
  bc := { q | q coincident b; q coincident c }
  ca := { q | q coincident c; q coincident a }
  distance(p, centres) >= r
  distance(p, vertex) <= distance(p, meeting)
  p inside ab.ends
  p inside bc.ends
  p inside ca.ends
}

// A chamfer's flat at a convex edge: the points within both faces nearer them than the flat
// through each face's point its leg from the other face — `la` on `a`, `lb` on `b` (#183 X1's
// finding 4).  Between planes the flat, about an axis a cone.
component Flat(a: face, b: face, la: Length, lb: Length) symmetric((a, la), (b, lb)) := { p |
  p inside a.smooth
  p inside b.smooth
  angle(a, b, at: p) <= 180deg
  distance(p, a.smooth) / lb + distance(p, b.smooth) / la <= 1
  crease := { q | q coincident a; q coincident b }
  meeting := { q | q coincident a.surface; q coincident b.surface; q inside crease.ends }
  distance(p, crease) <= distance(p, meeting)
  p inside crease.ends
}

// `Flat` at a concave edge, every side turned round: what the flat adds.
component FlatFill(a: face, b: face, la: Length, lb: Length) symmetric((a, la), (b, lb)) := { p |
  p outside a.smooth
  p outside b.smooth
  angle(a, b, at: p) >= 180deg
  distance(p, a.smooth) / lb + distance(p, b.smooth) / la <= 1
  crease := { q | q coincident a; q coincident b }
  meeting := { q | q coincident a.surface; q coincident b.surface; q inside crease.ends }
  distance(p, crease) <= distance(p, meeting)
  p inside crease.ends
}

// `A round(r: 3mm) B`: every edge where a face of `A` meets a face of `B` rounded by a ball of
// radius `r` — taken away where it is convex, filled where it is concave, read off the geometry —
// and where three such edges meet, the ball touching all three.  Faces that do not meet make
// nothing, so only faces that meet are paired (`meets`, #625): a body's faces are many, and
// three at a time they are more.  Its material acts on the body the faces are of, as made by
// the terms that do not read it.  `round(r) X` is every edge among `X`'s faces.
A round(r) B := {
  repeat a in A {
    repeat b in B {
      when b != a and b meets a {
        run := Run(a, b, r: r)
        taken := solid(run)
        taken cut A.body
        fill := Fill(a, b, r: r)
        filled := solid(fill)
        filled union A.body
        repeat c in A {
          when c in B and c != a and c != b and c meets a and c meets b {
            corner := Corner(a, b, c, r: r)
            cornered := solid(corner)
            cornered cut A.body
            corner_fill := CornerFill(a, b, c, r: r)
            corner_filled := solid(corner_fill)
            corner_filled union A.body
          }
        }
      }
    }
  }
}
round(r) X := X round(r: r) X

// `A chamfer(d) B`: every edge where a face of `A` meets one of `B` cut by a flat `d` along each
// face, measured from the other; where three meet, their flats meet in a point.
A chamfer(d) B := {
  repeat a in A {
    repeat b in B {
      when b != a and b meets a {
        flat := Flat(a, b, la: d, lb: d)
        taken := solid(flat)
        taken cut A.body
        flat_fill := FlatFill(a, b, la: d, lb: d)
        filled := solid(flat_fill)
        filled union A.body
      }
    }
  }
}
chamfer(d) X := X chamfer(d: d) X

// -- a solid grown ------------------------------------------------------------------------------
//
// `B` grown by `d` all round, as a solid of its own (`.body`): the points within `d` of it, so a
// clearance or a print's gap about a part is the part's own body grown, not its sizes said again.
// Its edges and corners are rounded by `d`, a ball rolled over `B`.  Where `d` is below nothing,
// `B` shrunk by it: the points of `B` at least `-d` from everything outside it.  At nothing, `B`
// itself.  `B` is left as it is.  Grown, it is built exactly where `B`'s faces are planes and
// cylinders meeting in convex edges and prisms' corners (a box, a post, a hex bar), or planes in
// concave ones (an L, a step, a slot, a pocket), or at a foot (a boss or a post standing on a
// plate, a bore's floor); shrunk, or any other body, it is read point by point, and an exact export
// says so.
//
//   window := std.rounds.Grown(uno.usb_plug, d: gap)
//   window.body cut frame_part
component Grown(B: solid, d: Length) {
  when d > 0mm {
    within := { c | c inside B }
    near := { p | distance(p, within) <= d }
    body := solid(near)
  }
  when d < 0mm {
    beyond := { c | c outside B }
    kept := { p | p inside B; distance(p, beyond) >= -d }
    body := solid(kept)
  }
  when d == 0mm {
    body := solid(B)
  }
}

// -- a solid shelled ------------------------------------------------------------------------------
//
// `A shell(t: 2mm) B`: the body `B`'s faces are of hollowed to walls `t` thick, open through the
// faces of `A` — of its material only what lies within `t` of one of its faces not among `A`'s is
// kept.  `shell(t: …) X` leaves every face closed: `X` keeps what lies within `t` of everything
// outside it, a sealed hollow.  Inside, each wall is its face moved in by `t`: at a convex edge
// the walls' inner faces meet sharp, at a concave one they are rounded by `t`, a ball rolled within
// both.  A wall runs on to an open face's rim, and round it by `t` wherever the hollow reaches past
// the wall's own edge there: a boss standing on an open top keeps a quarter round about its foot, a
// wall leaning in over the opening a rounded lip.  Like a round it acts on the body after its other
// terms, so a boss the body carries is shelled with it.  Where `B` is a prism of straight edges,
// with what is cut from it — a pocket, a bore — and nothing put on it, the shell is built exactly;
// any other body is read point by point, and an exact export says so.
//
//   tray.top shell(t: 2mm) tray
//   shell(t: 1.5mm) float
A shell(t) B := {
  within := { c | c inside B }
  cavity := solid(within)
  repeat w in B {
    when not w in A {
      wall := { p | p distance(<= t) w }
      walled := solid(wall)
      walled cut cavity
    }
  }
  cavity cut B.body
}
shell(t) X := {
  beyond := { c | c outside X }
  kept := { p | p inside X; distance(p, beyond) >= t }
  cavity := solid(kept)
  cavity cut X.body
}

// -- in a region's body ---------------------------------------------------------------------------
//
// A region is a conjunction, so every edge where two of its bounds meet is convex, and a round or
// a chamfer there is one more bound on its point: no solid, no body.  `a` and `b` are two of the
// sets the region is within, as it states them (`{ q | q outside toe }` for the side outside a
// toe it bounds).
//
//   disc := { p | p inside wall; p inside top; p inside bottom
//                 p inside std.rounds.Rounded(wall, top, r: 2mm) }

// Within reach of a ball of radius `r` lying within both `a` and `b`.
component Rounded(a: set, b: set, r: Length) := { p |
  centres := { c | c distance(>= r, side: inside) a; c distance(>= r, side: inside) b }
  distance(p, centres) <= r
}

// Short of the flat through each set's point `la` (on `a`) and `lb` (on `b`) from the other's
// surface: the points whose distances from the two surfaces, each over the other's leg, sum to
// one or more — for a point within both, its distance from each surface is its distance from
// what is outside that set.  Between planes the flat, about an axis a cone; where a surface is
// curved in section it bows with it (on a sphere beside a cone, a parabola's arc turned).
// Multiplied through by both legs, so a leg of nothing cuts nothing.
component Chamfered(a: set, b: set, la: Length, lb: Length) := { p |
  out_a := { q | q outside a }
  out_b := { q | q outside b }
  la * distance(p, out_a) + lb * distance(p, out_b) >= la * lb
}
