// A cam's pitch curve marched out from its profile by the roller's radius (#376): the path a
// roller's centre takes is the profile's parallel curve, a front moving away from the cam at unit
// speed, square to itself, its Cauchy data the profile the drawing has.
//
// The cam is `generation/heart_cam.sv`'s rising half: a follower sliding out `lift` along its line
// over half a turn, seen from the turning cam. The profile `cam_rise` is the envelope of the
// roller under that motion (`side: near`, the side facing the cam's centre), and the pitch curve
// `pitch_rise` the envelope of the roller's centre. `Pitch` marches the first into the second:
//
//   initially { p coincident(t == u) k }
//
// at every node of the sampled formal the front starts on the profile where its roll is `u`, and
// moves out along its normal — `nl`, placed in the cam's plane square to the front, its seed at
// the cam's centre — at unit speed. Marched the roller's radius it stands on `pitch_rise` at the
// same roll: `pitch`, over the rise less a little at each end, where the profile's halves part.
//
// **The undercut is the fold.** Marching out from a profile, a front folds where the profile is
// concave and its radius of curvature runs out: a roller larger than that cannot reach into the
// hollow, and its centre's path would cross itself. A generated profile never does this — it is
// the roller's own envelope — but a drawn one may. `lobed` is a three-lobed cam drawn by its
// polar formula, r = 30 + 6 cos 3u; at the bottom of each hollow (u = 60°) it is concave with
// radius r²/(r″ − r) = 19.2. A roller of 15 follows it (`follows`); one of 22 cannot
// (`undercut`), and its march is refused at the hollow, short of 19.2, where a node's area
// |x_d × x_u| falls to a hundredth of its home's. `solventc` says how far each is marched.

use std (right_of)

base := 25    // the roller centre's nearest approach to the cam's centre
lift := 15    // how far the follower rises
rr := 6       // the roller

in std.front {
  o := point
  fix((0, 0)) o
  s0 := point
  s1 := point
  fix((0, 0)) s0
  fix((10, 0)) s1
  path := line(s0, s1)                    // the follower's line, through the cam's centre
}

cam := motion(about: o, ratio: 1)
out := motion(along: path, advance: 2 * lift)
rise := motion(out, relative_to: cam)

in std.front {
  c_rise := point hint((25, 0))
  c_rise right_of(d: base) o
  roller := circle(center: c_rise) hint(r: 6)
  radius(rr) roller
}

pitch_rise := envelope(c_rise, under: rise, from: 0deg, to: 180deg)
cam_rise := envelope(roller, under: rise, from: 0deg, to: 180deg, side: near)

// a profile drawn by its formula, three lobes and three hollows
component Lobed(c: point, u: Angle) {
  p := point(x: c.x + (30 + 6 * cos(3 * u)) * cos(u), y: c.y + (30 + 6 * cos(3 * u)) * sin(u))
}
lobed := Lobed(o).p over u in (0deg, 120deg)

// -- the pitch curves, marched ----------------------------------------------------------------
component Pitch(c: point, y1: point, k: curve, d: Length, u: Angle) {
  ya := line(c, y1)
  p := point hint((0, 0, 0))
  m := point hint((0, 0, 0))
  nl := line(p, m)
  length(10) nl
  c distance(0, along: ya) m                 // the normal in the cam's plane...
  nl perpendicular rate(p over u)            // ...square to the front
  rate(p over d) parallel nl                 // the gauge: square to itself
  rate(p over d) distance(-1, along: nl)     // the law: at unit speed, away from the centre
  initially {
    p coincident(t == u) k                   // the Cauchy data: the profile
  }
}

cc := point hint((0, 0, 0))
fix((0, 0, 0)) cc
y1 := point hint((0, 10, 0))
fix((0, 10, 0)) y1

pitch := Pitch(cc, y1, cam_rise).p over (d in (0, rr), u in (10deg, 170deg))
follows := Pitch(cc, y1, lobed).p over (d in (0, 15), u in (30deg, 90deg))
undercut := Pitch(cc, y1, lobed).p over (d in (0, 22), u in (30deg, 90deg))
