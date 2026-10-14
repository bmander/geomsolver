// The carrier: the cast-and-turned aluminium spider the stator is pressed on.
//
// A hub keyed to the axle, a web, and a rim the stack's bore is pressed over: one stepped
// profile turned about the axle.  Six windows through the web lighten it, let air through and
// give the phase wires their way from the axle's port out to the windings.  The key's slot is
// cut through the hub's whole length; the axle's cable port, handed in, is cut through the hub
// where it passes, so the wires leave the axle into the left-hand bay.

use std (horizontal)
use motor.dims
use motor.parts
use motor.axle

component Carrier(f: group, dims: group, exit: solid) {
  hh := dims.hub_half
  wh := dims.web_half
  rl := dims.L / 2
  in f.side {
    a := motor.parts.At(f.so, dx: -hh, dy: dims.d_core / 2)
    b := motor.parts.At(f.so, dx: hh, dy: dims.d_core / 2)
    c := motor.parts.At(f.so, dx: hh, dy: dims.hub_r)
    d := motor.parts.At(f.so, dx: wh, dy: dims.hub_r)
    e := motor.parts.At(f.so, dx: wh, dy: dims.rim_r)
    g := motor.parts.At(f.so, dx: rl, dy: dims.rim_r)
    h := motor.parts.At(f.so, dx: rl, dy: dims.Rb)
    k := motor.parts.At(f.so, dx: -rl, dy: dims.Rb)
    m := motor.parts.At(f.so, dx: -rl, dy: dims.rim_r)
    n := motor.parts.At(f.so, dx: -wh, dy: dims.rim_r)
    q := motor.parts.At(f.so, dx: -wh, dy: dims.hub_r)
    s := motor.parts.At(f.so, dx: -hh, dy: dims.hub_r)
    profile := (bore := line(a.p, b.p)) -> (hub_r := line(b.p, c.p)) ->
               (hub_o := line(c.p, d.p)) -> (web_r := line(d.p, e.p)) ->
               (rim_i := line(e.p, g.p)) -> (rim_r := line(g.p, h.p)) ->
               (seat := line(h.p, k.p)) -> (rim_l := line(k.p, m.p)) ->
               (rim_il := line(m.p, n.p)) -> (web_l := line(n.p, q.p)) ->
               (hub_ol := line(q.p, s.p)) -> (hub_l := line(s.p, a.p)) -> close
  }
  body := solid(profile, about: f.ax)

  windows := motor.parts.HoleCircle(f, n: dims.windows, r: dims.window_r, d: dims.window_d,
                                    phase: 30deg)
  repeat dims.windows as i {
    private window := solid(face(windows.hole[i]), through: body)
    window cut body
  }

  // the hub's keyway, `key_t2` above the seat over the hub's length, and the cable port
  in f.front {
    slot := motor.parts.Box(f.o, x0: -dims.key_w / 2, y0: -dims.d_core / 2 - dims.key_t2,
                            x1: dims.key_w / 2, y1: -dims.d_core / 2 + 1mm)
  }
  private keyway := solid(slot.profile, through: body)
  keyway cut body
  exit cut body
}

// Open this file to preview the carrier on the axle whose cable port it is cut by.
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

  axle := motor.axle.Axle(frame, dims: motor.dims.motor_dims)
  carrier := Carrier(frame, dims: motor.dims.motor_dims, exit: axle.exit)
}
