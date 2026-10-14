// A side cover: the hub's end wall, its bearing housing and its spoke flange in one casting.
//
// One profile turned about the axle, written for the right-hand cover and mirrored by `sg` (+1
// right, −1 left): a spigot entering the rotor ring's bore, which centres it; the web that
// closes the end, its outer face's rim thinned to the spoke flange; the bearing boss, its pocket
// open inboard and the lip outboard that the bearing's outer race stands against; and a boss
// beyond the lip `boss_to` from the mid-plane and `boss_r` in radius — the brake disc's mount
// on the left, the freewheel's thread on the right.
//
// The holes are drilled along the axle, from the end view: the spoke holes round the flange,
// staggered half a pitch between the two flanges (`spoke_phase`), and the cap screws' clearance
// holes on the very centres the rotor ring's tapped holes stand on, which are handed in.  The
// disc side (`disc: 1`) is also tapped for the brake disc's six M5 bolts.

use std (horizontal)
use motor.dims
use motor.parts

component Cover(f: group, dims: group, screws: group, sg: Scalar, boss_to: Length,
                boss_r: Length, spoke_phase: Angle, disc: Int) {
  lip_in := dims.brg_in + dims.brg_w
  lip_out := lip_in + dims.lip
  outer := dims.half + dims.web
  // each run one number: a station from the mid-plane or a radius from the axle, measured in
  // the end view from its origin, the other ordinate kept where the run began
  in f.side {
    profile := step(u: sg * boss_to) (lip_bore := line) -> step(v: boss_r) (boss_end := line) ->
               step(u: sg * lip_out) (boss := line) -> step(v: dims.boss_r) (boss_step := line) ->
               step(u: sg * outer) (housing := line) -> step(v: dims.Ro) (face_o := line) ->
               step(u: sg * (dims.half + dims.flange_t)) (flange_step := line) ->
               step(v: dims.flange_r) (flange_o := line) ->
               step(u: sg * dims.half) (flange_rim := line) ->
               step(v: dims.Rr) (flange_i := line) ->
               step(u: sg * (dims.half - dims.spigot)) (spigot_o := line) ->
               step(v: dims.spigot_r) (spigot_end := line) ->
               step(u: sg * dims.half) (spigot_i := line) ->
               step(v: dims.boss_r) (face_i := line) ->
               step(u: sg * dims.pocket_in) (housing_i := line) ->
               step(v: dims.brg_od / 2) (pocket_end := line) ->
               step(u: sg * lip_in) (pocket := line) ->
               step(v: dims.lip_r) (lip_face := line) -> close
  }
  body := solid(profile, about: f.ax)

  // the spoke holes, through the flange
  spokes := motor.parts.HoleCircle(f, n: dims.spokes, r: dims.spoke_r, d: dims.spoke_d,
                                   phase: spoke_phase)
  repeat dims.spokes as i {
    private spoke := solid(face(spokes.hole[i]), through: body)
    spoke cut body
  }

  // the cap screws' clearance holes, each on a tapped hole's centre in the ring
  in f.front {
    repeat dims.screws as i {
      clear := circle(center: screws.c[i]) hint(r: dims.clear4 / 2)
      radius(dims.clear4 / 2) clear
    }
  }
  repeat dims.screws as i {
    private screw := solid(face(clear[i]), through: body)
    screw cut body
  }

  // the brake disc's six M5 holes, `disc_depth` into the boss's face
  when disc == 1 {
    disc_holes := motor.parts.HoleCircle(f, n: dims.disc_bolts, r: dims.disc_bcd / 2,
                                         d: dims.tap5, phase: 0deg)
    // the boss's face along the end view's normal (−y), and the holes run inboard from it
    face_h := -sg * boss_to
    repeat dims.disc_bolts as i {
      private tapped := solid(face(disc_holes.hole[i]),
                              from: min(face_h, face_h + sg * dims.disc_depth),
                              to: max(face_h, face_h + sg * dims.disc_depth))
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
  frame := {front: std.front, side: std.side, top: std.top, o: std.origin, datum: datum,
            so: std.side.origin, ax: ax, to: std.top.origin}

  screw := motor.parts.HoleCircle(frame, n: motor.dims.screws, r: motor.dims.screw_r,
                                  d: motor.dims.tap4, phase: 180deg / motor.dims.screws)
  left := Cover(frame, dims: motor.dims.motor_dims, screws: screw, sg: -1,
                boss_to: motor.dims.disc_face, boss_r: motor.dims.disc_r, spoke_phase: 0deg,
                disc: 1)
}
