// The engine's connecting rod, stressed (#510): the rod imported as the engine designs it
// (`engine.conrod`, its pin and its bore placed as its own preview places them, at the table's
// crank angle) and read under the gas load of a firing stroke — the big end's bore held on its
// crank pin, 25kN pressed through the small end's bore toward the crank, along the bore.
//
// The rod is forged with sharp pockets and sharp steps where its shank meets its eyes, and stress
// at a sharp re-entrant edge is unbounded, so a factor of safety over the whole rod has no answer.
// Stress is unbounded only near such an edge: over faces clear of every one — the small end's
// eye, its bore, the round into the big end on the near side — it has a greatest stress a finer
// mesh converges to, and the factor of safety is read there, each with its error beside it.
unit mm
use std
use engine.conrod
use engine.parts
use engine.dims

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
rod := engine.conrod.ConRod(pin, bore)

// a firing stroke's peak: the rod pressed toward its crank pin
scenario fired {
  fixed rod.body.whole.bigbore
  load rod.body.whole.smallbore -25kN along: bore
  claim deflection(rod.body, along: bore) <= 0mm
  claim safety(rod.body.whole.eyeS) >= 1
  claim safety(rod.body.whole.smallbore) >= 1
  claim safety(rod.body.whole.fbl) >= 1
}
