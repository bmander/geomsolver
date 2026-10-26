// A Wankel bore read off the apex's path: `housing.sv`'s front, its Cauchy data a curve the
// drawing has — the apex's envelope — rather than the apex the motion carries at each node (#376).
//
// The motion is `wankel/wankel.sv`'s, drawn the same way: the rotor's centre `hub` rides an
// eccentric `e` about the shaft, an apex stands `R` beyond it, and phasing gears turn the rotor a
// third as fast as the shaft. The apex's envelope under the rotor's turn is its path — the
// epitrochoid e cos 3u + R cos u, e sin 3u + R sin u — and the envelope's parameter is the shaft's
// roll, three times that eccentric angle, as `housing.sv`'s carried apex's is.
//
// `Bore` is `housing.sv`'s body with its Cauchy data read off the curve it is handed:
//
//   initially { p coincident(t == 3 * u) k }
//
// at every node of the sampled formal the moved point starts on `k` where its parameter is
// `3u`. Its seeds are plain numbers, as `housing.sv`'s are: the row places the point whatever
// they say, and `m`'s seed at the centre picks the normal toward the shaft. The march's
// sensitivities to the body's columns run through the envelope's: move the apex, or the
// eccentric, and the bore moves with it.
//
// `bore` is the front run to the seal's tip radius `s`, regular, and `housing.sv`'s bore to the
// march's tolerance; `past` runs on to 20 and is refused at the waist (u = 90°) short of its
// radius of curvature, (R − 3e)²/(9e − R) = 15, where `housing.sv`'s `past` is refused.
unit mm
use std
use std.place (right_of)
use std.motion (spins)

R := 75mm                 // the generating radius, the rotor's centre to an apex
e := 15mm                 // the eccentricity, the shaft's crank
s := 3mm                  // the apex seal's tip radius

// -- the motion, as `wankel.sv` draws it ------------------------------------------------------
in std.top {
  centre := point
  centre coincident std.top.origin
  hub := point hint((15, 0))
  hub right_of(d: e) centre
  apex := point hint((90, 0))
  apex right_of(d: R) hub
}
counter := frame
counter spins(-1) centre
spin := frame
spin spins(-2 / 3) hub                                   // the rotor, seen from the housing

// the curve the seal's centre runs on: the apex's path over half a turn of the rotor
trochoid := envelope(apex over t in (0deg, 540deg), under: spin, seen_from: counter)

// -- the bore, marched from it ----------------------------------------------------------------
component Bore(c: point, z1: point, k: curve, d: Length, u: Angle) {
  za := line(c, z1)
  p := point hint((0, 0, 0))
  m := point hint((0, 0, 0))
  nl := line(p, m)
  length(10mm) nl
  c distance(0mm, along: za) m               // the normal in the rotor's plane...
  nl perpendicular rate(p over u)            // ...square to the front
  rate(p over d) parallel nl                 // the gauge: square to itself
  rate(p over d) distance(-1, along: nl)     // the law: at unit speed, away from `nl`
  initially {
    p coincident(t == 3 * u) k               // the Cauchy data: the apex's path
  }
}

c := point hint((0, 0, 0))
fix((0, 0, 0)) c
z1 := point hint((0, 0, 10))
fix((0, 0, 10)) z1

bore := trace(Bore(c, z1, trochoid).p over (d in (0mm, s), u in (0deg, 180deg)))
past := trace(Bore(c, z1, trochoid).p over (d in (0mm, 20mm), u in (0deg, 180deg)))
