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
| `front.sv` | **Two formals: a first-order PDE.** An ellipse's points move inward at unit speed, square to the front — a law and a gauge — marched along how far it has moved, sampled along the ellipse by the method of lines; `initially` is the Cauchy data. The front is the ellipse offset, and where its least radius of curvature runs out it folds and is refused. Issue #339, item D4. |

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

`solventc` says of each such curve how far it is integrated, or where it is refused. Each file's
test is in `rust/gcs-core/tests/rate_examples.rs`, its curves against the closed forms in its
comments; `front.sv`'s is `rust/gcs-core/tests/lines.rs`, the method of lines' gate.
