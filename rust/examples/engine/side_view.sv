// The side view: a longitudinal section along the crankshaft, all four cylinders.
//
// This view owns the engine's *lengths* — where the block ends, where each bore sits on the
// pitch, where the main bearings are — and states nothing about a height: every height here is
// left for the end view to give it by projection.  The points the document projects onto are the
// ports of the assemblies below; the rest is built from them by the parts' own dimensions.

use engine.dims
use engine.parts

// The block and head, edge on: a rectangle from the pan rail to the deck, another from the deck
// to the head's top, and the camshaft centreline — the two shafts one behind the other here.
// Every height is a port for the end view to place.
component BlockSide(o: point) {
  port bfl: point hint(x: o.x + front, y: o.y + deck)        // deck, front
  point bfr hint(x: o.x + back, y: o.y + deck)
  port rfl: point hint(x: o.x + front, y: o.y + rail)        // pan rail, front
  point rfr hint(x: o.x + back, y: o.y + rail)
  port deckline = dl
  line dl(bfl, bfr)
  line blockfront(bfl, rfl)
  line blockback(bfr, rfr)
  port railline = rl
  line rl(rfl, rfr)
  o distance(front, along: x) bfl
  o distance(front, along: x) rfl
  o distance(back, along: x) rfr
  horizontal dl
  horizontal rl
  bfl distance(back - front) bfr class shown
  port htl: point hint(x: o.x + front, y: o.y + deck + head)  // head top, front
  point htr hint(x: o.x + back, y: o.y + deck + head)
  line headtop(htl, htr)
  line headfront(bfl, htl)
  line headback(bfr, htr)
  o distance(front, along: x) htl
  o distance(back, along: x) htr
  horizontal headtop
  port cam: point hint(x: o.x + front, y: o.y + camh)         // the camshaft
  point camb hint(x: o.x + back, y: o.y + camh)
  line camline(cam, camb) class axis
  o distance(front, along: x) cam
  o distance(back, along: x) camb
  horizontal camline
}

// The sump, hung from the pan rail: shallow at the front, deep at the back; `sd` is the bottom
// the end view places.
component SumpSide(o: point, rail: line) {
  point sa hint(x: o.x + front + 15mm, y: o.y + rail)
  point sb hint(x: o.x + front + 45mm, y: o.y + sump + 45mm)
  point sc hint(x: o.x + front + 150mm, y: o.y + sump)
  port sd: point hint(x: o.x + back - 40mm, y: o.y + sump)
  point se hint(x: o.x + back - 10mm, y: o.y + rail)
  line s1(sa, sb) -> line s2(sb, sc) -> line s3(sc, sd) -> line s4(sd, se)
  sa on rail
  se on rail
  o distance(front + 15mm, along: x) sa
  o distance(back - 10mm, along: x) se
  o distance(front + 45mm, along: x) sb
  sd distance(45, along: y) sb
  o distance(front + 150mm, along: x) sc
  o distance(back - 40mm, along: x) sd
  horizontal s3
}

// One cylinder edge on, about its axis point `ax` on the crank axis: the bore walls down from
// the deck, the piston on the bore axis at the height the end view gives its small end, and the
// two cam lobes over it.  The crankshaft below and the rod are parts of their own
// (`engine.crankshaft`, `engine.conrod`), drawn by the assembly.
component CylinderSide(ax: point, deckline: line) {
  point wl0 hint(x: ax.x - D / 2, y: ax.y + deck)
  point wr0 hint(x: ax.x + D / 2, y: ax.y + deck)
  point wl1 hint(x: ax.x - D / 2, y: ax.y + deck - wall)
  point wr1 hint(x: ax.x + D / 2, y: ax.y + deck - wall)
  line wall_l(wl0, wl1)
  line wall_r(wr0, wr1)
  wl0 on deckline
  wr0 on deckline
  ax distance(-D / 2, along: x) wl0
  ax distance(D / 2, along: x) wr0
  ax distance(-D / 2, along: x) wl1
  ax distance(D / 2, along: x) wr1
  wl0 distance(-wall, along: y) wl1
  wr0 distance(-wall, along: y) wr1
  port small: point hint(x: ax.x, y: ax.y + R + L)
  ax distance(0, along: x) small
  piston: Piston(small, pin: 0)
  lobe_i: Box(ax, x0: 14mm, y0: camh - rb, x1: 26mm, y1: camh + rb)
  lobe_e: Box(ax, x0: -26mm, y0: camh - rb, x1: -14mm, y1: camh + rb)
}

// The timing drive on the front face, edge on: the pulleys are rectangles, the belt two lines.
component DriveSide(o: point, cam: point) {
  crankpulley: Box(o, x0: front - 55mm, y0: -rcp, x1: front - 30mm, y1: rcp)
  campulley: Box(cam, x0: -55mm, y0: -rcam, x1: -30mm, y1: rcam)
  line beltf(crankpulley.d, campulley.a) class belt
  line beltb(crankpulley.c, campulley.b) class belt
}

component SideSection(o: point) {
  // the crank axis, front to back
  a0: At(o, dx: front - 70mm, dy: 0mm)
  a1: At(o, dx: back + 20mm, dy: 0mm)
  line axisline(a0.p, a1.p) class axis
  block: BlockSide(o)
  sump: SumpSide(o, block.railline)
  drive: DriveSide(o, block.cam)
  // the four cylinders on the pitch
  repeat 4 as i {
    ax: At(o, dx: front + 25mm + P / 2 + i * P, dy: 0mm)
    cyl: CylinderSide(ax.p, block.deckline)
  }
  // the pitch, as a reference dimension: every bore is already on it
  claim ax[0].p distance(P) ax[1].p class shown
}
