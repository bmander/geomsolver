// The planetary's meshes, checked as they roll: in the carrier's frame the sun turns, each planet
// turns back on its pin at `Zs / Zp` the sun's rate, and the ring turns back at `Zs / Zr`.  Each
// frame's value is written over one parameter, so a claim over it stands the train at every
// sample with every joint turned at its ratio (#904 A11).  A sun pitch is 12°, a whole cycle of
// the sun's mesh; the claims say the teeth never close their backlash.
unit mm
use std
use std.place (right_of)
use std.motion (spins)
use planetary
use dims (Zs, Zp, Zr, a)

box := planetary.Planetary()
in std.top {
  pin := point
  pin right_of(d: a) std.top.origin
}

param t: Angle over (0deg, 12deg)
sun_turn := frame
sun_turn spins(1) std.top.origin
fix(value == t) sun_turn
planet_turn := frame
planet_turn spins(-Zs / Zp) pin
fix(value == t) planet_turn
ring_turn := frame
ring_turn spins(-Zs / Zr) std.top.origin
fix(value == t) ring_turn

sun_at := box.turns.sun at sun_turn
planet_at := box.planet[0].body at planet_turn
ring_at := box.turns.ring at ring_turn
claim over t {
  sun_at clear(0.005mm) planet_at
  ring_at clear(0.005mm) planet_at
}
