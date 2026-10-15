# Lofts and sweeps

A loft or a sweep is a face of a component as one of its numbers runs:
`solid(Station(…).f over s in (0, 1))`, the solid the face `f` sweeps as `s` runs from 0 to 1.
The station is the writer's own component — where the section stands, how it turns, how it
scales, which rails its corners follow — and the standard library's words say the common parts
of it. Nothing about a loft is in the kernel but the face family itself; every wall here is built
exactly, and every file's volume is a closed form its comments give. Issue #243;
[docs/lofts/README.md](../../../docs/lofts/README.md) is the record of how each part was made.

Read them in this order:

| File | What it shows |
|---|---|
| `pipe.sv` | **A chain is one path.** A round pipe along a U-bend of lines and tangent arcs, `std.On` standing square to whichever link it is on; the wall keeps one name round the bends. Pappus. |
| `twisted_duct.sv` | **A twist is a law.** A square drawn once and carried whole onto each station (`sec := square mapped(o -> st.n.origin, x -> tw.t) in st.n`), turned a quarter, three and a half turns (an auger), and a quarter eased in and out, `std.Twisted`'s point given another fraction of `s`. Cavalieri. |
| `tapered.sv` | **A scale is a law too.** A square shrinking to half as it turns, a disc shrinking to a cone and a disc swelling on a sine, each drawn once and carried by the similarity that scales it, a round's radius with it. Frusta, and an integral. |
| `thread.sv` | **A section carried along a screw.** A square thread drawn through the axis and an auger's flight drawn square to it, on `std.Helix`. Pappus and Cavalieri. |
| `spring.sv` | **A section square to a helix.** A round wire on `std.OnHelix`, leaned back by the lead angle at every station. Area times the helix's length. |
| `ribbon.sv` | **A frame that does not twist.** A flat ribbon on a helix in `std.OnHelix`'s plane (Frenet's frame, facing the axis) and in `std.Minimal`'s, the rotation-minimising frame stated as a rate and integrated from where the family is drawn: it falls behind Frenet's by the torsion. Area times the helix's length. |
| `bottle.sv` | **Several sections, on rails.** A bottle's body, shoulder and neck joined ruled (a chain of lines through the rims, an edge at the shoulder) and smooth (the spline through the rims, square to the base and the neck). |
| `squares.sv` | **A loft with no guide.** Three squares in space, each corner on a rail of its own, the station the plane through three rails (`std.Across`). Prismatoids. |
| `two_faces.sv` | **Two sketches drawn apart.** A square and a diamond in their own planes joined corner to corner by `std.Ruled`, and again with the diamond's walk started at its second corner (a connector). Prismatoids. |
| `triangle_to_square.sv` | **Faces of different corner counts.** A square lofted to a triangle given a fourth corner by naming a point on one of its edges. |
| `pyramid.sv` | **A loft to a point.** A square to an off-centre tip by `std.Apex`, ending in an apex. A third of base by height. |
| `blade.sv` | **A section held to two rails.** A profile drawn once and carried whole by the similarity of two pairs (`mapped`, or `std.Similar`) onto two rails that close in and turn it. Its area times its scale squared. |
| `three_rails.sv` | **And to three.** A profile carried whole by the affine map of three pairs (`mapped`, or `std.Affine`) onto three rails in space, sheared as they lean. Its area times the rails' triangle's ratio. |
| `square_to_circle.sv` | **One section, one edge vanishing.** A square to a circle drawn as a rounded square whose flats shrink to points: no pairing of lines with an arc. |

The case library has four more: `solid_loft.sv` (a hollow reducer) and `solid_elbow.sv` (a duct
round a bend), in the menu, and `solid_bend.sv` and `solid_twist.sv` beside them.

What the words are (all in `std`):

- **Stations** — a plane `n` square to a path where its parameter is `s`, its `v` along `up`:
  `OnLine`, `OnArc`, `OnSpline`, and `On` over any path, a named chain of lines and arcs among
  them; `Helix(base, pitch:, turns:, s:)` (`n` carried by a screw, `axial` through its axis) and
  `OnHelix`; `Across(a, b, c, s:)`, the plane through three rails at `s`; `Minimal(n, up:, s:)`,
  the rotation-minimising frame of a station `n`, integrated along the running number.
- **Laws** — a section drawn once is carried whole by the map taking its frame to the
  station's (`sec := prof mapped(o -> st.n.origin, x -> tw.t) in st.n`, #357), the law in where
  its second point stands — `Twisted(n, twist:, reach:, s:)`'s point `t`, turned `twist · s` —
  and a section written as coordinates in the station's plane is scaled by
  `p scaled(du:, dv:, scale:, s:) P` and `scaled_radius(r:, scale:, s:) k`. Any other law is
  another fraction of `s`.
- **Sections from faces drawn apart** — `Ruled(a, b, s:)`, `Apex(a, tip, s:)`, and a profile held
  to rails, `Similar(prof, from, to, n, a, b)` and `Affine(prof, from, to, third, n, a, b, c)`,
  each one `prof mapped(…) in n`.
- **Rails** — a chain of lines through corresponding corners (ruled), or
  `spline(through: chain, start: l, end: m)` (smooth).

Each file's test is in `rust/gcs-core/tests/loft_examples.rs`, its volume off the exact solid
against the closed form in its comments.
