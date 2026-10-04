// A Wankel engine's rotor and housing from one planetary motion. The rotor's centre rides an
// eccentric about the shaft, and phasing gears turn the rotor a third as fast as the shaft: a
// turn about the rotor's centre, seen by an observer turning the other way about the shaft.
//
// The housing's bore is no typed curve: it is an apex's envelope under that motion — a point's
// envelope is its path — the epitrochoid. The rotor is the other half of envelope theory: what
// the housing never reaches as it moves about the rotor, a blank cut by the housing swept over a
// whole period — the largest rotor that turns in the bore. Its flank is the bore's envelope under
// the housing's motion, drawn on the page as `flank`. The numbers are in `configuration.sv`.
unit mm
use std
use configuration

// -- the motion -------------------------------------------------------------------------------
// Drawn in the top view, the rotor at the start of its turn: the shaft's centre, the rotor's `e`
// from it, an apex `R` beyond that on the line of centres.
in std.top {
  private centre := point
  centre distance(0mm, along: u) std.top
  centre distance(0mm, along: v) std.top
  private hub := point
  hub distance(configuration.eccentricity, along: u) std.top
  hub distance(0mm, along: v) std.top
  apex := point
  apex distance(configuration.eccentricity + configuration.generating_radius, along: u) std.top
  apex distance(0mm, along: v) std.top
}

// The shaft's turn seen from the housing, backwards, and the rotor's turn about its centre
// against it: the phasing gears, a pinion fixed to the housing twice the eccentric and the rotor's
// ring gear three times it, turn the rotor two thirds of the way back.
counter := motion(about: centre, ratio: -1)
spin := motion(about: hub, ratio: -2 / 3)
rotor_turn := motion(spin, relative_to: counter)      // the rotor, seen from the housing
housing_turn := motion(counter, relative_to: spin)    // the housing, seen from the rotor

// -- the curves -------------------------------------------------------------------------------
// The bore is where an apex goes over three turns of the shaft, one of the rotor; a flank is
// where the bore touches the rotor as the housing turns about it, from one apex to the next.
bore := envelope(apex, under: rotor_turn, from: 0deg, to: 1080deg)
flank := envelope(bore, under: housing_turn, from: 10deg, to: 170deg)

// -- the parts --------------------------------------------------------------------------------
// The housing: a disc with the bore through it, a hair proud of the rotor at each face.
in std.top {
  private rim := circle(center: centre) hint(r: 165)
  radius(configuration.generating_radius + 4 * configuration.eccentricity) rim
}
private case := solid(face(rim), from: -1mm, to: configuration.width + 1mm)
private chamber := solid(face(bore), from: -2mm, to: configuration.width + 2mm)
housing := solid(case)
chamber cut housing

// The rotor: a blank reaching past the apexes, less everything the housing passes through as it
// turns about the rotor over a whole period.
in std.top {
  private blank_rim := circle(center: hub) hint(r: 120)
  radius(configuration.generating_radius + configuration.eccentricity) blank_rim
}
private blank := solid(face(blank_rim), from: 0mm, to: configuration.width)
construction swept := solid(housing, under: housing_turn, from: 0deg, to: 1080deg)
rotor := solid(blank)
swept cut rotor

// -- a rotor to make --------------------------------------------------------------------------
// The envelope is the largest rotor that turns in the bore. One to machine has three arcs for
// flanks, each through two apexes drawn `clearance` in from the envelope's and through the
// middle of the flank at the envelope's own depth there less the clearance: R − 2e from the
// centre, where the flank faces the bore's waist (R − e from the shaft, the centre e nearer). An
// arc through those three points lies within the flank, and the claim says this rotor clears the
// housing by half the clearance at every pose of a whole turn.
in std.top {
  cycle 3 as k {
    private tip := point
    tip distance(configuration.eccentricity + (configuration.generating_radius - configuration.clearance) * cos(k * 120deg), along: u) std.top
    tip distance((configuration.generating_radius - configuration.clearance) * sin(k * 120deg), along: v) std.top
    private crown := point
    crown distance(configuration.eccentricity + (configuration.generating_radius - 2 * configuration.eccentricity - configuration.clearance) * cos(k * 120deg + 60deg), along: u) std.top
    crown distance((configuration.generating_radius - 2 * configuration.eccentricity - configuration.clearance) * sin(k * 120deg + 60deg), along: v) std.top
    // its centre on the far side of the rotor's, about R from it
    flank := arc(center: hint(x: configuration.eccentricity - configuration.generating_radius * cos(k * 120deg + 60deg),
      y: -configuration.generating_radius * sin(k * 120deg + 60deg)), start: tip, end: next.tip)
    crown on flank
  }
}
arc_rotor := solid(face(flank[0], flank[1], flank[2]), from: 0mm, to: configuration.width)
arc_rotor_at := solid(arc_rotor, under: rotor_turn, at: 0deg)
claim over rotor_turn in (0deg, 1080deg) {
  arc_rotor_at clear(configuration.clearance / 2) housing
}
