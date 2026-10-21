// The axle, and the key that carries the stator's reaction into it.
//
// A turned 4140 shaft, stepped from the carrier's seat at its middle to the bearings' seats and
// down again to the threaded ends the dropouts clamp.  Its section is one profile turned about
// its centreline; the features that are not turns are prisms cut from it: the flats the
// dropouts' slots bear on (from the section, sideways through it), the keyway (from the end
// view, along it), and the cable's way out — a bore up the left end that turns out radially
// beside the carrier's web, a hole drawn in the plan and swept straight up.

use std (below, horizontal)
use motor.dims (axle_half, cable_d, cable_to, d_core, d_end, d_seat, exit_at, exit_d, flats,
                flats_from, hub_r, key_h, key_l, key_t1, key_w, step_end, step_seat)
use motor.parts (Datums)

part Axle(f: Datums, to: point) {
  ah := axle_half
  rc := d_core / 2
  rs := d_seat / 2
  re := d_end / 2
  in f.so.plane {
    a := motor.parts.At(f.so, dx: -ah, dy: cable_d / 2)
    b := motor.parts.At(f.so, dx: -cable_to, dy: cable_d / 2)
    // the bore's end and the right end's centre, on the centreline
    c := point
    d := point
    c coincident f.ax
    d coincident f.ax
    f.so distance(-cable_to, along: u) c
    f.so distance(ah, along: u) d
    e := motor.parts.At(f.so, dx: ah, dy: re)
    g := motor.parts.At(f.so, dx: step_end, dy: re)
    h := motor.parts.At(f.so, dx: step_end, dy: rs)
    k := motor.parts.At(f.so, dx: step_seat, dy: rs)
    m := motor.parts.At(f.so, dx: step_seat, dy: rc)
    n := motor.parts.At(f.so, dx: -step_seat, dy: rc)
    q := motor.parts.At(f.so, dx: -step_seat, dy: rs)
    s := motor.parts.At(f.so, dx: -step_end, dy: rs)
    t := motor.parts.At(f.so, dx: -step_end, dy: re)
    w := motor.parts.At(f.so, dx: -ah, dy: re)
    profile := (cable := line(a.p, b.p)) -> (cable_end := line(b.p, c)) ->
               (centre := line(c, d)) -> (end_r := line(d, e.p)) ->
               (thread_r := line(e.p, g.p)) -> (step_r := line(g.p, h.p)) ->
               (seat_r := line(h.p, k.p)) -> (shoulder_r := line(k.p, m.p)) ->
               (core := line(m.p, n.p)) -> (shoulder_l := line(n.p, q.p)) ->
               (seat_l := line(q.p, s.p)) -> (step_l := line(s.p, t.p)) ->
               (thread_l := line(t.p, w.p)) -> (end_l := line(w.p, a.p)) -> close

    // the flats, each a slab beyond `flats / 2` from the centreline, cut sideways through
    fr_up := motor.parts.Box(f.so, x0: flats_from, y0: flats / 2, x1: ah + 1mm, y1: re + 1mm)
    fr_dn := motor.parts.Box(f.so, x0: flats_from, y0: -re - 1mm, x1: ah + 1mm, y1: -flats / 2)
    fl_up := motor.parts.Box(f.so, x0: -ah - 1mm, y0: flats / 2, x1: -flats_from, y1: re + 1mm)
    fl_dn := motor.parts.Box(f.so, x0: -ah - 1mm, y0: -re - 1mm, x1: -flats_from, y1: -flats / 2)
  }
  body := solid(profile, about: f.ax)
  private flat_ru := solid(fr_up.profile, through: body)
  private flat_rd := solid(fr_dn.profile, through: body)
  private flat_lu := solid(fl_up.profile, through: body)
  private flat_ld := solid(fl_dn.profile, through: body)
  flat_ru cut body
  flat_rd cut body
  flat_lu cut body
  flat_ld cut body

  // the keyway, under the carrier's seat, and the key in it: in the end view, straight down
  in f.o.plane {
    seat := motor.parts.Box(f.o, x0: -key_w / 2, y0: -rc - 1mm, x1: key_w / 2, y1: -rc + key_t1)
    k_sec := motor.parts.Box(f.o, x0: -key_w / 2, y0: -rc + key_t1 - key_h,
                             x1: key_w / 2, y1: -rc + key_t1)
    private keyway := solid(seat.profile, from: -key_l / 2, to: key_l / 2)
    key := solid(k_sec.profile, from: -key_l / 2, to: key_l / 2)
  }
  keyway cut body

  // the cable's way out: up the bore from the left end, out through the top beside the web
  in to.plane {
    outlet := point
    outlet below(d: exit_at) to
    exit_c := circle(center: outlet) hint(r: exit_d / 2)
    radius(exit_d / 2) exit_c
    exit := solid(face(exit_c), from: 0mm, to: hub_r + 1mm)
  }
  exit cut body
}

// Open this file to preview the axle and its key alone, in the assembly's frame.
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

  axle := Axle(frame, std.top.origin)
}
