// A side cover: the hub's end wall, its bearing housing and its spoke flange in one casting.
//
// One profile turned about the axle, drawn once as the right-hand cover's and carried onto the
// left (`sg`: +1 right, −1 left) as its mirror image in the mid-plane (#357): a spigot entering
// the rotor ring's bore, which centres it; the web that closes the end, its outer face's rim
// thinned to the spoke flange; the bearing boss, its pocket open inboard and the lip outboard
// that the bearing's outer race stands against; and a boss beyond the lip `boss_to` from the
// mid-plane and `boss_r` in radius — the brake disc's mount on the left, the freewheel's thread
// on the right.
//
// The holes are drilled along the axle, from the end view: the spoke holes round the flange,
// staggered half a pitch between the two flanges (`spoke_phase`), and the cap screws' clearance
// holes on the very centres the rotor ring's tapped holes stand on, which are handed in.  The
// disc side (`disc: 1`) is also tapped for the brake disc's six M5 bolts.

use std (horizontal)
use motor.dims (brg_in, brg_od, brg_w, clear4, disc_bcd, disc_bolts, disc_depth, flange_r, flange_t,
                half, lip, lip_r, pocket_in, Ro, Rr, spigot, spigot_r, spoke_d, spoke_r, tap5, web)
use motor.parts (Datums)

part Cover(f: Datums, screws: group, sg: Scalar, boss_to: Length, boss_r: Length,
           spoke_phase: Angle, disc: Int) {
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
  // the left cover's profile is the right's mirrored in the mid-plane, the side view's `v` through
  // its origin: the map taking that line's points to themselves, reversing the turn
  when sg == 1 {
    body := solid(profile, about: f.ax)
  }
  when sg == -1 {
    in f.so.plane {
      private rim := point
      rim level(u) f.so
      f.so distance(Ro, along: v) rim
    }
    mirrored := profile mapped(f.so -> f.so, rim -> rim, mirror: true) in f.so.plane
    body := solid(mirrored, about: f.ax)
  }

  // the spoke holes, through the flange
  spokes := motor.parts.HoleCircle(f, n: motor.dims.spokes, r: spoke_r, d: spoke_d,
                                   phase: spoke_phase)
  repeat motor.dims.spokes as i {
    private spoke := solid(face(spokes.hole[i]), through: body)
    spoke cut body
  }

  // the cap screws' clearance holes, each on a tapped hole's centre in the ring
  in f.o.plane {
    repeat motor.dims.screws as i {
      clear := circle(center: screws.c[i]) hint(r: clear4 / 2)
      radius(clear4 / 2) clear
    }
  }
  repeat motor.dims.screws as i {
    private screw := solid(face(clear[i]), through: body)
    screw cut body
  }

  // the brake disc's six M5 holes, `disc_depth` into the boss's face
  when disc == 1 {
    disc_holes := motor.parts.HoleCircle(f, n: disc_bolts, r: disc_bcd / 2, d: tap5, phase: 0deg)
    // the boss's face along the end view's normal (−y), and the holes run inboard from it
    face_h := -sg * boss_to
    repeat disc_bolts as i {
      private tapped := solid(face(disc_holes.hole[i]),
                              from: min(face_h, face_h + sg * disc_depth),
                              to: max(face_h, face_h + sg * disc_depth))
      tapped cut body
    }
  }
}

// Open this file to preview the brake-side cover alone.  The ring's tapped holes it is drilled
// on are drawn as `motor.rotor` draws them, half a pitch off the datum.
preview {
  unit mm
  // the frame, as the assembly draws it: the end view's datum and the axle's centreline
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
  frame := {o: std.origin, datum: datum, so: std.side.origin, ax: ax}

  screw := motor.parts.HoleCircle(frame, n: motor.dims.screws, r: motor.dims.screw_r,
                                  d: motor.dims.tap4, phase: 180deg / motor.dims.screws)
  left := Cover(frame, screw, sg: -1, boss_to: motor.dims.disc_face, boss_r: motor.dims.disc_r,
                spoke_phase: 0deg, disc: 1)
}
