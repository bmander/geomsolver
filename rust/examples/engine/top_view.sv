// The plan view: the engine from above.
//
// The plan states almost nothing of its own.  Its outline's corners are placed by projection —
// along the axis from the side view, across from the end view — and so are the bores and the
// camshafts; what it adds is the valve layout, four valves a cylinder, which no section shows.

use engine.dims
use engine.parts

// One bore from above: the cylinder, its plug, and its two intake and two exhaust valves.
component BoreTop(c: point) {
  circle bore(center: c) hint(r: D / 2)
  radius(D / 2) bore
  circle plug(center: c) hint(r: 7mm)
  radius(7) plug
  repeat 2 as k {
    vi: At(c, dx: -16mm + k * 32mm, dy: vs)
    ve: At(c, dx: -16mm + k * 32mm, dy: -vs)
    circle intake(center: vi.p) hint(r: div / 2)
    circle exhaust(center: ve.p) hint(r: dev / 2)
    radius(div / 2) intake
    radius(dev / 2) exhaust
  }
}

component PlanView(o: point) {
  // the outline of the block at the deck: a rectangle whose corners the document projects
  port fl: point hint(x: o.x + front, y: o.y - hw)
  point fr hint(x: o.x + back, y: o.y - hw)
  port br: point hint(x: o.x + back, y: o.y + hw)
  point bl hint(x: o.x + front, y: o.y + hw)
  line e1(fl, fr) -> line e2(fr, br) -> line e3(br, bl) -> line e4(bl, fl) -> close
  horizontal e1
  vertical e2
  horizontal e3
  vertical e4
  // the crank axis, seen from above
  a0: At(o, dx: front - 70mm, dy: 0mm)
  a1: At(o, dx: back + 20mm, dy: 0mm)
  line axisline(a0.p, a1.p) class axis
  // the two camshafts, across the block at the offsets the end view gives their centres
  port ci: point hint(x: o.x + front, y: o.y + camx)
  port ce: point hint(x: o.x + front, y: o.y - camx)
  line cam_i(ci, hint(x: o.x + back, y: o.y + camx)) class axis
  line cam_e(ce, hint(x: o.x + back, y: o.y - camx)) class axis
  ci on e4
  ce on e4
  cam_i.p2 on e2
  cam_e.p2 on e2
  horizontal cam_i
  horizontal cam_e
  // the bores, on the axis; where along it, the side view says
  repeat 4 as i {
    port c: point hint(x: o.x + front + 25mm + P / 2 + i * P, y: o.y)
    o distance(0, along: y) c
    bore: BoreTop(c)
  }
}
