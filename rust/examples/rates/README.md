# Rates along a running formal

A traced body may say how its points *move* as the formal it runs over runs: `rate(p over s)` is
`p`'s velocity, and it stands where a direction is read — `parallel`, `perpendicular`, a directed
`angle`, or alone before `distance(k, along: l)` for its component along `l`. Such a body is not
solved again at each place but **integrated from its home**: a drawn instance as it stands on the
sheet, or, written in place, where `initially { … }` puts it. The rows that fix its rates are a
rank checked at every place, and a curve stops where they lose it. Issue #339, item D2;
[docs/rates/README.md](../../../docs/rates/README.md) is the record of how it was made.

Read them in this order:

| File | What it shows |
|---|---|
| `tractrix.sv` | **One rate row.** A point dragged by a string moves only along the string. Drawn, the home is the cusp and the sheet holds the freedom the rate leaves; written in place, `initially` says it and the march runs both ways, the running formal handed to a nested component as itself. Away from the home the rate row is a claim, and a theorem. |
| `clothoid.sv` | **A law along a drawn direction.** Unit speed (a rate alone before `distance`) and a curvature growing with arc length: a railway's transition, eased into the bend that osculates its end. The Fresnel integrals. |
| `chain.sv` | **A two-point problem is a drawing.** A hanging chain's pulls left as unknowns, shot to its far end through the flow's sensitivities — beside the free curve of the same length, which hangs the same. The catenary. |
| `pursuit.sv` | **Rates between moving points, and where they stop.** A dog runs at a hare twice as fast; the line between them is the direction, and where it vanishes the curve is refused: the capture. |
| `geodesic.sv` | **In space, a rank and not a count.** A filament wound on a cone bends only toward the surface's normal; a `parallel` in space is two rows and the cone already implies one. Written in place, set off by `initially`. A straight line unrolled. |
| `front.sv` | **Two formals: a first-order PDE.** An ellipse's points move inward at unit speed, square to the front — a law and a gauge — marched along how far it has moved, sampled along the ellipse by the method of lines; `initially` is the Cauchy data. The front is the ellipse offset, and where its least radius of curvature runs out it folds and is refused: `folded` runs it there, and `solventc` says where. A point held on the major axis is found on the surface at its place, and a claim says the surface stays in the ellipse's plane. Issue #339, item D4. |
| `geodesic_circles.sv` | **A front on a surface.** A row over the moved point alone holds it to a sphere or a cone, and the front's normal is placed in the tangent plane. On the sphere the circles of latitude close on the antipode, where the front focuses and is refused; on the cone, unrolled, the fronts are circles about the apex. The starting circles are written by their own ordinates, the seeds are plain numbers that pick each root. Issue #370. |
| `housing.sv` | **An offset with a design rule.** A Wankel bore is the epitrochoid's parallel curve at the apex seal's radius: the front moves outward from the curve the seal's centre runs on. Outward a front folds only where its curve is concave — at the trochoid's waist — so the seal's radius must stay below the waist's radius of curvature; run past it the march is refused there. Issue #370. |
| `fisheye_fronts.sv` | **A speed that reads where the point is.** Light from a point source spreading through Maxwell's fish-eye, whose index falls off from its centre: the front moves at 1/n, and n is read at the moving point's own coordinates (`p.x`) — the eikonal equation in a graded medium. The fronts are circles (the lens is a sphere seen through a stereographic projection), and they close again on the point opposite the source, where the march is refused: the lens images perfectly. `../fisheye.sv`'s ray, a free curve, crosses every front square to it. Issue #378. |

What the words are:

- **`rate(x over s)`** — a point's velocity as `s` runs, `s` always named: a component does not know
  which of its formals the call runs. A component handed the formal as itself (`Drag(…, u: u)`)
  rates over the caller's. Along a `Scalar` or a `Length` formal.
- **Where it stands** — `rate(p over s) parallel l`, `perpendicular l`, a directed `angle(…) l`, and
  `rate(p over s) distance(k, along: l)`. Never a place (`coincident`) and never a magnitude
  (`length(k) rate(…)`), one rate to a relation.
- **`initially { … }`** — what holds at the home alone, for an instance written in place. On the
  sheet a drawn instance is its own home, and the block's relations are ordinary ones.
- **Enough rows** — the body's other rows and its rate rows must fix every rate: in the plane as
  many rows as it places numbers. One short is refused naming the points it leaves free.
- **Over two formals** — `over (d in (…), u in (…))`, the first marched and the second sampled at
  nodes, a rate along either. A rate along the second is read off the nodes, of a point the rates
  move, in a relation that places a point they do not (the front's normal) or a rate row. Along an
  angle a rate is per radian.
- **A number that reads the body's own points** — in a body traced in space a dimension may read
  `p.x`, `p.y`, `p.z` of a point the body declares, so a speed may depend on where the point is
  (`fisheye_fronts.sv`). Over two formals, in a rate row or a relation that places a point the
  rates do not move, never in `initially`.

- **What a front cannot say yet** — its Cauchy data are written in `initially`, by relations and
  expressions, never read off a curve the drawing has (an envelope, a spline, another traced
  curve) nor a point a motion carries, so a starting circle is written by its ordinates; and a surface is
  sampled on 32 pieces whatever its length, so a whole turn is resolved three times as coarsely
  as a third of one (`housing.sv` marches half its bore, the other half its mirror).

`solventc` says of each such curve how far it is integrated, or where it is refused. Each file's
test is in `rust/gcs-core/tests/rate_examples.rs`, its curves against the closed forms in its
comments; `front.sv`'s is `rust/gcs-core/tests/lines.rs`, the method of lines' gate.
