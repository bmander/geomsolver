// The engine's connecting rod, its web sized by its sag (#567): the rod imported as the engine
// designs it (`engine.conrod`, its pin and its bore placed as its own preview places them, at the
// table's crank angle), its web's thickness `wt` left to the drawing — an unknown — and the
// firing load's sinking of its small end driving it.
//
// The rod is a filled body: no prism, but the end view's outline swept along the crank axis in
// common with the side view's, less its pockets, its oil passage and its bolts' holes, split at
// its parting line.  Its mesh is its exact boundary refined and filled once, and each node of its
// boundary moves with the faces it lies on — the web's two faces, the side view's lines swept
// across the engine, move apart as `wt` grows, and every node of the pockets' walls and floors
// with them — so the solve reads how the small end's corner sinks as the web thickens, and finds
// the web at which it sinks `sag`.  `solventc` drives it on a coarse mesh first and finishes on
// the reading's own; the page judges it, and says so.
unit mm
use std
use engine.conrod
use engine.parts
use engine.dims

sag := 0.29mm
param wt: Length hint(5mm)

// the rod's pin and bore, as the engine's crankshaft and block put them
in std.side {
  top := engine.parts.At(std.side.origin, dx: 0mm, dy: 300mm)
  bore := line(std.side.origin, top.p)
  pe := engine.parts.At(std.side.origin, dx: engine.dims.R * sin(engine.dims.theta),
                        dy: engine.dims.R * cos(engine.dims.theta))
}
in std.front {
  ps := engine.parts.At(std.origin, dx: engine.dims.front + 25mm + engine.dims.P / 2,
                        dy: engine.dims.R * cos(engine.dims.theta))
}
pin := {end: pe.p, side: ps.p}
rod := engine.conrod.ConRod(pin, bore, wt: wt)

// down the bore, toward the crank
down := axis
fix(dir == (0, 0, -1), origin == (0, 0, 0)) down

// a firing stroke's peak: 30kN pressed through the small end's bore toward the crank pin, the
// big end's bore held on it, and the small end's corner sinking `sag`
scenario fired {
  fixed rod.body.whole.bigbore
  load rod.body.whole.smallbore 30kN along: down
  deflection(rod.sb, along: down) == sag
}
