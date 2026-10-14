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

use motor.parts

component Cover(f: group, dims: group, screws: group, sg: Scalar, boss_to: Length,
                boss_r: Length, spoke_phase: Angle, disc: Int) {
  lip_in := dims.brg_in + dims.brg_w
  lip_out := lip_in + dims.lip
  outer := dims.half + dims.web
  in f.side {
    p1 := motor.parts.At(f.so, dx: sg * lip_in, dy: dims.lip_r)
    p2 := motor.parts.At(f.so, dx: sg * boss_to, dy: dims.lip_r)
    p3 := motor.parts.At(f.so, dx: sg * boss_to, dy: boss_r)
    p4 := motor.parts.At(f.so, dx: sg * lip_out, dy: boss_r)
    p5 := motor.parts.At(f.so, dx: sg * lip_out, dy: dims.boss_r)
    p6 := motor.parts.At(f.so, dx: sg * outer, dy: dims.boss_r)
    p7 := motor.parts.At(f.so, dx: sg * outer, dy: dims.Ro)
    p8 := motor.parts.At(f.so, dx: sg * (dims.half + dims.flange_t), dy: dims.Ro)
    p9 := motor.parts.At(f.so, dx: sg * (dims.half + dims.flange_t), dy: dims.flange_r)
    p10 := motor.parts.At(f.so, dx: sg * dims.half, dy: dims.flange_r)
    p11 := motor.parts.At(f.so, dx: sg * dims.half, dy: dims.Rr)
    p12 := motor.parts.At(f.so, dx: sg * (dims.half - dims.spigot), dy: dims.Rr)
    p13 := motor.parts.At(f.so, dx: sg * (dims.half - dims.spigot), dy: dims.spigot_r)
    p14 := motor.parts.At(f.so, dx: sg * dims.half, dy: dims.spigot_r)
    p15 := motor.parts.At(f.so, dx: sg * dims.half, dy: dims.boss_r)
    p16 := motor.parts.At(f.so, dx: sg * dims.pocket_in, dy: dims.boss_r)
    p17 := motor.parts.At(f.so, dx: sg * dims.pocket_in, dy: dims.brg_od / 2)
    p18 := motor.parts.At(f.so, dx: sg * lip_in, dy: dims.brg_od / 2)
    profile := (lip_bore := line(p1.p, p2.p)) -> (boss_end := line(p2.p, p3.p)) ->
               (boss := line(p3.p, p4.p)) -> (boss_step := line(p4.p, p5.p)) ->
               (housing := line(p5.p, p6.p)) -> (face_o := line(p6.p, p7.p)) ->
               (flange_step := line(p7.p, p8.p)) -> (flange_o := line(p8.p, p9.p)) ->
               (flange_rim := line(p9.p, p10.p)) -> (flange_i := line(p10.p, p11.p)) ->
               (spigot_o := line(p11.p, p12.p)) -> (spigot_end := line(p12.p, p13.p)) ->
               (spigot_i := line(p13.p, p14.p)) -> (face_i := line(p14.p, p15.p)) ->
               (housing_i := line(p15.p, p16.p)) -> (pocket_end := line(p16.p, p17.p)) ->
               (pocket := line(p17.p, p18.p)) -> (lip_face := line(p18.p, p1.p)) -> close
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
