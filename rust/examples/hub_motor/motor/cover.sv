// A side cover: the hub's end wall, its bearing housing and its spoke flange in one casting.
//
// One profile turned about the axle, drawn as the right-hand cover's: a spigot entering the
// rotor ring's bore, which centres it; the web that closes the end, its outer face's rim thinned
// to the spoke flange; the bearing boss, its pocket open inboard and the lip outboard that the
// bearing's outer race stands against; and a boss beyond the lip `boss_to` from the mid-plane and
// `boss_r` in radius — the freewheel's thread on the right, the brake disc's mount on the left.
// The left cover is this one mirrored in the mid-plane (#578): `Cover(…) mirrored(front)`.
//
// A cover is closed: a function of its numbers alone, drawn about the frame it stands in — its
// own datum and axle drawn from its `std`, as the assembly draws the frame's.  The holes are
// drilled along the axle, from the end view: the spoke holes round the flange, staggered half a
// pitch between the two flanges (`spoke_phase`), and the cap screws' clearance holes on the very
// circle and pitch the rotor ring's tapped holes are drawn on, from the same numbers of
// `motor.dims` (the assembly claims they meet).  The disc side (`disc: 1`) is also tapped for the
// brake disc's six M5 bolts.

use std
use std.place (horizontal)
use motor.dims (brg_in, brg_od, brg_w, clear4, disc_bcd, disc_bolts, disc_depth, flange_r, flange_t,
                half, lip, lip_r, pocket_in, Ro, Rr, screw_r, screws, spigot, spigot_r, spoke_d,
                spoke_r, tap5, web)
use motor.parts (Datums)

component Cover(boss_to: Length, boss_r: Length, spoke_phase: Angle, disc: Int) {
  // the frame it stands in, as the assembly draws it: the end view's datum and the axle's
  // centreline in the section
  in std.front {
    construction centerline datum := line(std.origin, hint((150, 0)))
    horizontal datum
    length(150mm) datum
  }
  in std.side {
    construction centerline ax := line(std.side.origin, hint((100, 0)))
    horizontal ax
    length(100mm) ax
  }
  f := {o: std.origin, datum: datum, so: std.side.origin, ax: ax}

  lip_in := brg_in + brg_w
  lip_out := lip_in + lip
  outer := half + web
  // each run one number: a station from the mid-plane or a radius from the axle, measured in
  // the section from its origin, the other ordinate kept where the run began
  in f.so.plane {
    profile := step(u: boss_to) (lip_bore := line) -> step(v: boss_r) (boss_end := line) ->
               step(u: lip_out) (boss := line) -> step(v: motor.dims.boss_r) (boss_step := line) ->
               step(u: outer) (housing := line) -> step(v: Ro) (face_o := line) ->
               step(u: half + flange_t) (flange_step := line) ->
               step(v: flange_r) (flange_o := line) ->
               step(u: half) (flange_rim := line) ->
               step(v: Rr) (flange_i := line) ->
               step(u: half - spigot) (spigot_o := line) ->
               step(v: spigot_r) (spigot_end := line) ->
               step(u: half) (spigot_i := line) ->
               step(v: motor.dims.boss_r) (face_i := line) ->
               step(u: pocket_in) (housing_i := line) ->
               step(v: brg_od / 2) (pocket_end := line) ->
               step(u: lip_in) (pocket := line) ->
               step(v: lip_r) (lip_face := line) -> close
  }
  body := solid(profile, about: f.ax)

  // the spoke holes, through the flange
  spokes := motor.parts.HoleCircle(f, n: motor.dims.spokes, r: spoke_r, d: spoke_d,
                                   phase: spoke_phase)
  repeat motor.dims.spokes as i {
    private spoke := solid(face(spokes.hole[i]), through: body)
    spoke cut body
  }

  // the cap screws' clearance holes, on the ring's tapped holes' circle, half a pitch off the datum
  in f.o.plane {
    ring screws about f.o {
      c := point
      f.o distance(screw_r) c
      clear := circle(center: c) hint(r: clear4 / 2)
      radius(clear4 / 2) clear
      private construction clear_radial := line(f.o, c)
    }
  }
  f.datum angle(180deg / screws) clear_radial[0]
  repeat screws as i {
    private screw := solid(face(clear[i]), through: body)
    screw cut body
  }

  // the brake disc's six M5 holes, `disc_depth` into the boss's face: the end view's normal runs
  // toward −y, so the face `boss_to` out stands at `−boss_to` along it, and the holes run inboard
  when disc == 1 {
    disc_holes := motor.parts.HoleCircle(f, n: disc_bolts, r: disc_bcd / 2, d: tap5, phase: 0deg)
    repeat disc_bolts as i {
      private tapped := solid(face(disc_holes.hole[i]), from: -boss_to, to: disc_depth - boss_to)
      tapped cut body
    }
  }
}

// Open this file to preview the brake-side cover alone: the right-hand cover's profile with the
// disc's boss, mirrored in the mid-plane.
preview {
  unit mm
  left := Cover(boss_to: motor.dims.disc_face, boss_r: motor.dims.disc_r, spoke_phase: 0deg,
                disc: 1) mirrored(front)
}
