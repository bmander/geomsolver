// A torus tumbled through space: it turns about an upright axis beside it while being watched
// from a frame that turns about a tilted axis, and the body is everything it passes through
// between -`roll` and +`roll`. Change a number and apply (⌘↵) to watch it refine again; ⌘B shows
// it in the glass box. Keep `tube` below `ring`, so the torus has a hole.
unit mm
use std (above, coords, on_u, on_v)

ring := 1mm        // the torus's centre circle
tube := 0.5mm      // the torus's tube
roll := 75deg      // how far it tumbles each way

// The torus: a circle turned about an upright axis `ring` away from its centre.
in std.front {
  private ta := point
  private tb := point
  fix((3, 1)) ta
  fix((3, 3)) tb
  private taxis := line(ta, tb)
  private tc := point hint((3mm + ring, 2))
  tc distance(ring, along: right) ta
  tc distance(2mm, along: v) std.front
  private section_c := circle(center: tc) hint(r: tube)
  radius(tube) section_c
  construction torus := solid(face(section_c), about: taxis)

  // The motion: a quarter turn about an upright axis for every turn of a frame about a tilted one.
  private hub := point hint((2, 0))
  hub on_u(d: 2mm) std.front
  private hub_up := point hint((2, 5))
  hub_up above(d: 5mm) hub
  construction centerline cradle := line(hub, hub_up)
}
private spin := motion(about: cradle, ratio: 0.25)
private rise := axis hint(dir: (0, 1, 1))
fix(dir == (0, sqrt(0.5), sqrt(0.5))) rise
private tilted := plane(u: std.x, v: rise)
fix(origin == (0, 0, 0)) tilted
in tilted {
  private k0 := point hint((0, 0.7071))
  private k1 := point hint((5, 0.7071))
  k0 on_v(d: 0.7071mm) tilted
  k1 coords(du: 5mm, dv: 0.7071mm) tilted
  construction centerline kaxis := line(k0, k1)
}
private observer := motion(about: kaxis)
tumble := motion(spin, relative_to: observer)
tumbled := solid(torus, under: tumble, from: -roll, to: roll)
