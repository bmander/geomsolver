// A hand-cranked right-angle drive to print and glue together: the hypoid pair of
// `spiral_bevel/hypoid.sv`, imported whole, each member on a shaft running in a plain sleeve in
// one frame, and a crank on each shaft. Nothing of the pair is restated: `HypoidPair` lays it out
// and cuts both members, and everything here stands on what it names — each member's axis and
// the foot of the mean point M on it (`.ax`, `.foot`), the back cone its blank is bounded by
// (`.back`), which the member's printed hub fills, and the member itself (`.body`). The frame
// stands inside the pair, where the shafts cross: a post carrying a sleeve along each shaft into
// the hollow of its member, on a floor under both. Each shaft goes into its sleeve from the
// post, its head stopping it there, and its splined end is pressed into its member's hub and
// its crank's. The shafts are square and offset, and their common perpendicular (`Crossing`) is
// the frame's third direction. Every part is placed by the pair's own lines; no point is seeded
// and no coordinate is written. See README.md.
unit mm
use std
use spiral_bevel.hypoid (HypoidPair)

// -- the pair, as `hypoid.sv` states it --------------------------------------------------------
// The tooth counts, the module at M, the shafts, the spiral and the allowances; the face mill's
// two settings left to the pair's own balance, as there.
param pinion_teeth := 24
param gear_teeth := 48
param mean_module := 2mm
param shaft_angle := 90deg
param offset := 25mm
param spiral_angle := 25deg
param backlash := 0.05mm
param tip_relief := 0.2mm
param end_relief := 0.2mm
pair := HypoidPair(std.top, std.y, std.z, pinion_teeth: pinion_teeth, gear_teeth: gear_teeth,
  mean_module: mean_module, shaft_angle: shaft_angle, offset: offset,
  spiral_angle: spiral_angle, backlash: backlash, tip_relief: tip_relief, end_relief: end_relief,
  pressure_shift: hint(10deg), cutter_radius: hint(80mm))

// -- the drive ---------------------------------------------------------------------------------
// One set of numbers serves both shafts. Along each, ordinates are from the foot of M on it, or
// from where the other shaft crosses it for what stands at the post: the post `post` either way
// from the crossing; the sleeve from the post to just short of the member's hub (`front`, less
// the axial `play`), bored `clearance` over the journal; the shaft's head against the post's
// face, `play` clear; the member's hub from `front` to `hub`, filling the blank's core to `web`
// with a boss, a rim `rim` thick under the blank's back and `spokes` bars across between them,
// each two spokes `spoke_w` wide (and long enough, `spoke_reach`, to meet any rim); the crank's
// mid-plane at `crank`, the shaft ending at `end`, flush with the crank. From the hub on,
// the shaft is splined: `keys` bars across it, each two keys `key_w` wide from the `root` to the
// journal's radius, so the splined length passes through the sleeve; each hub's splined bore is
// that spline less `press` all round. The shaft's own spline is cut `shrink` inside it all round,
// so as drawn it stands `shrink - press` (0.05 mm) clear of each bore: a printer dilates what it
// lays down, an outer wall printing proud and a hole closing by some 0.05 to 0.1 mm each on a
// Prusa, so the printed spline goes into its bore a light press fit, to be glued. The crank is a
// plate `plate` thick: `boss_w` about the shaft, tapering to `eye_r` about the grip `throw` out,
// and below the shaft a fan `fan` either side of straight down, its reach and its pad sized by
// `Crank` to balance the rest.
// The grip is a pin `pin_r` about, pressed and glued into the eye, with a handle over it, `loose`
// all round and at each end, kept on by the pin's head. Printers differ: `shrink`, `press`,
// `clearance` and `loose` are the numbers to tune.
mount := {journal: 5mm, clearance: 0.2mm, play: 0.15mm, root: 4mm, keys: 3, key_w: 2.5mm,
  press: 0.05mm, shrink: 0.1mm, head_r: 8mm, head_t: 3mm, post: 10mm, sleeve_r: 9mm,
  front: -3mm, web: 6mm, hub: 12mm, hub_r: 10mm, rim: 2.5mm, spokes: 3, spoke_w: 5mm,
  spoke_reach: 80mm, crank: 23mm, end: 28mm, throw: 34mm, plate: 10mm, boss_w: 12mm, eye_r: 8mm,
  fan: 65deg, grip: 40mm, pin_r: 4mm, handle_r: 7mm, loose: 0.3mm, head_w: 3mm}

// The pair's size, for the frame's floor: nothing of either member stands farther from the
// gear's axis than its heel, half the face width (0.1 R) past the mean cone distance R, and a
// module beyond for the tip. The floor's top is a clearance below that: a round floor `plate`
// thick and `floor_r` about the post, spoked — a hub `hub_r` about the post's foot, a rim
// `rim` wide, and `spokes` bars across between them, each two spokes `spoke_w` wide. The cranks'
// reach is less than the gear's height over the floor, so a crank turns clear of the floor
// and of the table the drive stands on.
R := mean_module * sqrt(pinion_teeth^2 + gear_teeth^2) / 2
reach := 1.1 * R + 2 * mean_module
sizes := {below: reach + 12mm, plate: 8mm, floor_r: 50mm, hub_r: 16mm, rim: 6mm, spokes: 3,
  spoke_w: 8mm}

cross := Crossing(pair.gear_cone.ax, pair.pinion_cone.ax)
gear_side := Mount(pair.gear_cone.ax, pair.gear_cone.foot, cross.on_a, pair.gear_blank.back,
  pair.gear.body, cross.span, pair.pinion_cone.ax, mount)
pinion_side := Mount(pair.pinion_cone.ax, pair.pinion_cone.foot, cross.on_b,
  pair.pinion_blank.back, pair.pinion.body, cross.span, pair.gear_cone.ax, mount)
frame := Frame(pair.gear_cone.ax, pair.gear_cone.foot, pair.pinion_cone.ax,
  pair.pinion_cone.foot, cross, mount, sizes)

// -- what the drive must be ----------------------------------------------------------------------
// Each member's blank bounds it (a member is its blank less its cuts), so a part clear of the
// blank is clear of the member. The frame clears both members, and each shaft and hub runs in
// it with its clearance and play; each shaft clears the other member and the other shaft; each
// handle turns loose on its pin, between the crank and the pin's head. Each
// crank, turned with its shaft through a whole turn, clears the frame and the other member; the
// two cranks turn in slabs beyond their members, and the slabs are apart.
claim frame.body clear(2mm) pair.gear_blank.material
claim frame.body clear(2mm) pair.pinion_blank.material
claim gear_side.shaft clear(0.1mm) frame.body
claim pinion_side.shaft clear(0.1mm) frame.body
claim gear_side.hub clear(0.1mm) frame.body
claim pinion_side.hub clear(0.1mm) frame.body
claim gear_side.shaft clear(1mm) pair.pinion_blank.material
claim pinion_side.shaft clear(1mm) pair.gear_blank.material
claim gear_side.shaft clear(1mm) pinion_side.shaft
claim gear_side.crank.handle clear(0.2mm) gear_side.crank.body
claim gear_side.crank.handle clear(0.2mm) gear_side.crank.pin
claim pinion_side.crank.handle clear(0.2mm) pinion_side.crank.body
claim pinion_side.crank.handle clear(0.2mm) pinion_side.crank.pin
gear_turn := frame
std.Turn(gear_turn, about: pair.gear_cone.ax, ratio: 1)
pinion_turn := frame
std.Turn(pinion_turn, about: pair.pinion_cone.ax, ratio: 1)
construction gear_crank_turned := solid(gear_side.crank.body, under: gear_turn, at: 0deg)
construction pinion_crank_turned := solid(pinion_side.crank.body, under: pinion_turn, at: 0deg)
claim over gear_turn in (0deg, 360deg) {
  gear_crank_turned clear(2mm) frame.body
  gear_crank_turned clear(2mm) pair.pinion_blank.material
}
claim over pinion_turn in (0deg, 360deg) {
  pinion_crank_turned clear(2mm) frame.body
  pinion_crank_turned clear(2mm) pair.gear_blank.material
}

// -- the parts ---------------------------------------------------------------------------------
// The common perpendicular of two skew lines: a point on each, the line between them square to
// both. It runs from `a` to `b`, so with the gear's axis first it points from the gear's shaft to
// the pinion's — up, from the frame's floor. `Perpendicular` is what the frame reads of it.
interface Perpendicular { on_a: point, on_b: point, span: line }
component Crossing(a: line, b: line) {
  on_a := point
  on_b := point
  on_a coincident a
  on_b coincident b
  span := line(on_a, on_b)
  span perpendicular a
  span perpendicular b
}

// One member's drive train along its axis `ax`, where the other shaft crosses it at `inner`:
// the shaft, the member made a part with its hub, and the crank, its arm along `toward` and as
// wide across `across`. The hub is spoked within the member's core — the points deeper inside
// its pitch cone than the blank's back, `outside seat` — its rim the core's points within `rim`
// of the back, so the blank's back face lies on it; spokes from the rim to a boss, which runs on
// behind. The part is the member and its hub, bored for the shaft's spline.
component Mount(ax: line, foot: point, inner: point, seat: set, member: solid, toward: line,
    across: line, m: group) {
  journal := { q | q inside std.Cylinder(ax, r: m.journal)
                   inner distance(>= -m.post - m.play, along: ax) q
                   foot distance(<= m.front, along: ax) q }
  head := { q | q inside std.Cylinder(ax, r: m.head_r)
                inner distance(in: (-m.post - m.play - m.head_t, -m.post - m.play), along: ax) q }
  private construction journal_solid := solid(journal)
  private construction head_solid := solid(head)
  private splines := Splined(ax, foot, toward, lo: m.front, hi: m.end, root: m.root - m.shrink,
    tip: m.journal - m.shrink, width: m.key_w - 2 * m.shrink, keys: m.keys)
  shaft := solid()
  journal_solid union shaft
  head_solid union shaft
  splines.body union shaft

  core := { q | q outside seat
                foot distance(in: (m.front, m.web), along: ax) q }
  rim := { q | q outside seat; q distance(<= m.rim, side: outside) seat
               foot distance(in: (m.front, m.web), along: ax) q }
  boss := { q | q inside std.Cylinder(ax, r: m.hub_r)
                foot distance(in: (m.front, m.hub), along: ax) q }
  private construction core_solid := solid(core)
  private construction rim_solid := solid(rim)
  private construction boss_solid := solid(boss)
  private spoked := Spokes(ax, foot, across, lo: m.front, hi: m.web, width: m.spoke_w,
    reach: m.spoke_reach, bars: m.spokes)
  private construction wheel := solid()
  rim_solid union wheel
  spoked.body union wheel
  core_solid common wheel
  construction hub := solid()
  wheel union hub
  boss_solid union hub
  private hub_bore := Splined(ax, foot, toward, lo: m.front - 1mm, hi: m.hub + 1mm,
    root: m.root - m.press, tip: m.journal - m.press, width: m.key_w - 2 * m.press, keys: m.keys)
  part := solid(member)
  hub union part
  hub_bore.body cut part

  crank := Crank(ax, foot, toward, across, m)
}

// A splined length of shaft about `ax`, `lo` to `hi` along it from `o` (on the axis): a core of
// radius `root`, and `keys` bars across it within radius `tip`, each `width` wide, the first
// square to `across` and each turned on half a turn over `keys`, so `keys` bars make twice as
// many keys.
component Splined(ax: line, o: point, across: line, lo: Length, hi: Length, root: Length,
    tip: Length, width: Length, keys: Int) {
  turn := frame
  std.Turn(turn, about: ax, ratio: 1)
  core := { q | q inside std.Cylinder(ax, r: root); o distance(in: (lo, hi), along: ax) q }
  bar := { q | q inside std.Cylinder(ax, r: tip); o distance(in: (lo, hi), along: ax) q
               o distance(in: (-width / 2, width / 2), along: across) q }
  private construction core_solid := solid(core)
  private construction bar_solid := solid(bar)
  construction body := solid()
  core_solid union body
  repeat keys as i {
    private construction turned := solid(bar_solid, under: turn, at: i * 180deg / keys)
    turned union body
  }
}

// Spokes about `ax`, `lo` to `hi` along it from `o` (on the axis): `bars` bars across the axis,
// each `width` wide and `reach` out each way, the first square to `across` and each turned on
// half a turn over `bars`, so each bar is two spokes. A hub keeps what of them its core holds.
component Spokes(ax: line, o: point, across: line, lo: Length, hi: Length, width: Length,
    reach: Length, bars: Int) {
  turn := frame
  std.Turn(turn, about: ax, ratio: 1)
  bar := { q | q inside std.Cylinder(ax, r: reach); o distance(in: (lo, hi), along: ax) q
               o distance(in: (-width / 2, width / 2), along: across) q }
  private construction bar_solid := solid(bar)
  construction body := solid()
  repeat bars as i {
    private construction turned := solid(bar_solid, under: turn, at: i * 180deg / bars)
    turned union body
  }
}

// A crank on the shaft about `ax`, a plate `plate` thick at `crank` from `foot`, as a crank is
// drawn face on: a boss `boss_w` about the shaft, bored for the spline, tapering on lines
// tangent to both to an eye `eye_r` about the grip's axis, `throw` along `toward` and parallel
// to the shaft; and below the shaft, the plate's own lower side fanning out, its counterweight —
// the sector of a disc about the shaft `fan` either side of straight down, padded on the
// handle's side. The grip's pin (`pin`, a part of its own) is pressed into the eye from the
// member's side and stands out away from it, a handle (`handle`) loose on it between the plate
// and the pin's head.
//
// The fan balances what turns with the crank, so its centre of mass is on the shaft and the
// shaft is a principal axis of its inertia: it spins without shaking or wobbling. Each piece is a
// plain shape at a stated size, so its first moment about the shaft (its area or volume times
// its distance off the axis) is a number worked out here, and where along the shaft it stands:
// the plate above the shaft — the boss, the trapezoid between the tangent points and the eye,
// less the boss's segment beyond the trapezoid and with the eye's beyond it — in the plate's
// mid-plane, and the pin, handle and head out at the throw, standing out from it. The fan must
// carry their whole first moment (static balance) and their moment along the shaft (the couple
// that would wobble it). The plate's part of the fan stands in its mid-plane, so the couple is
// the pad's alone: a pad as deep as `pad` on the plate's face, over the fan beyond the boss,
// stands `(plate + pad) / 2` out, and its share of the fan's first moment is `pad / (plate +
// pad)`, which says how deep it is; the fan's first moment, `2/3 (R³ - r³) sin(fan)` a unit
// thick beyond the boss, then says its radius R. All of it assumes one density, as one print
// is.
component Crank(ax: line, foot: point, toward: line, across: line, m: group) {
  center := point
  center coincident ax
  foot distance(m.crank, along: ax) center
  pin_at := point
  construction arm := line(center, pin_at)
  arm parallel toward
  center distance(m.throw, along: toward) pin_at
  tip := point
  construction grip_ax := line(pin_at, tip)
  grip_ax parallel ax
  pin_at distance(m.grip, along: ax) tip

  // the plate's own frame: `arm` up it, `side` across it, the shaft square to both
  side_end := point
  construction side := line(center, side_end)
  side parallel across
  center distance(1mm, along: across) side_end
  face_on := plane(u: arm, v: side)

  a := m.eye_r
  b := m.boss_w
  t := m.plate
  L := m.throw
  // the tangent lines: their normals at `theta` either side of the arm, where the sine of their
  // lean off it is (b - a) / L; the trapezoid between the tangent points, `u0` to `u1` up the arm
  s := (b - a) / L
  c := sqrt(1 - s^2)
  theta := acos(s)
  u0 := b * s
  u1 := L + a * s
  h0 := b * c
  h1 := a * c
  leaned := (u1^3 - u0^3) / 3 - u0 * (u1^2 - u0^2) / 2
  trapezoid := h0 * (u1^2 - u0^2) + 2 * (h1 - h0) / (u1 - u0) * leaned
  segments := 2 / 3 * (a^3 - b^3) * c^3 + L * a^2 * (theta / 1rad - c * s)
  upper := trapezoid + segments
  pinned := pi * m.pin_r^2 * (m.grip - t / 2)
  bore_r := m.pin_r + m.loose
  sleeved := pi * (m.handle_r^2 - bore_r^2) * (m.grip - t / 2 - 2 * m.loose)
  headed := pi * m.handle_r^2 * m.head_w
  standing := (pinned + sleeved + headed) * L
  couple := (pinned + sleeved) * L * (t / 2 + m.grip) / 2 + headed * L * (m.grip + m.head_w / 2)
  moment := upper * t + standing
  pad := 2 * couple / moment
  fan_moment := moment / (t + pad)
  reach := ((b^3 + fan_moment / (2 / 3 * sin(m.fan))) / 1mm^3)^(1 / 3) * 1mm

  // the directions square to the tangent lines and to the fan's edges, in the plate
  normal_l := point
  normal_r := point
  edge_l := point
  edge_r := point
  normal_l coincident face_on
  normal_r coincident face_on
  edge_l coincident face_on
  edge_r coincident face_on
  center distance(1mm) normal_l
  center distance(1mm) normal_r
  center distance(1mm) edge_l
  center distance(1mm) edge_r
  construction to_normal_l := line(center, normal_l)
  construction to_normal_r := line(center, normal_r)
  construction to_edge_l := line(center, edge_l)
  construction to_edge_r := line(center, edge_r)
  arm angle(theta) to_normal_l
  arm angle(theta) to_normal_r
  arm angle(90deg - m.fan) to_edge_l
  arm angle(90deg - m.fan) to_edge_r
  center distance(>= 0mm, along: side) normal_l
  center distance(<= 0mm, along: side) normal_r
  center distance(>= 0mm, along: side) edge_l
  center distance(<= 0mm, along: side) edge_r

  slab := { q | center distance(in: (-t / 2, t / 2), along: ax) q }
  boss := { q | q inside slab; q inside std.Cylinder(ax, r: b) }
  eye := { q | q inside slab; q inside std.Cylinder(grip_ax, r: a) }
  taper := { q | q inside slab; center distance(in: (u0, u1), along: arm) q
                 center distance(<= b, along: to_normal_l) q
                 center distance(<= b, along: to_normal_r) q }
  weight := { q | q inside slab; q inside std.Cylinder(ax, r: reach)
                  center distance(<= 0mm, along: to_edge_l) q
                  center distance(<= 0mm, along: to_edge_r) q }
  padding := { q | center distance(in: (t / 2 - 1mm, t / 2 + pad), along: ax) q
                   q inside std.Cylinder(ax, r: reach); q outside std.Cylinder(ax, r: b)
                   center distance(<= 0mm, along: to_edge_l) q
                   center distance(<= 0mm, along: to_edge_r) q }
  hole := { q | q inside std.Cylinder(grip_ax, r: m.pin_r - m.press)
                center distance(in: (-t, t), along: ax) q }
  private construction boss_solid := solid(boss)
  private construction eye_solid := solid(eye)
  private construction taper_solid := solid(taper)
  private construction weight_solid := solid(weight)
  private construction padding_solid := solid(padding)
  private construction hole_solid := solid(hole)
  private bore := Splined(ax, foot, toward, lo: m.crank - t / 2 - 1mm, hi: m.crank + t / 2 + 1mm,
    root: m.root - m.press, tip: m.journal - m.press, width: m.key_w - 2 * m.press, keys: m.keys)
  body := solid()
  boss_solid union body
  eye_solid union body
  taper_solid union body
  weight_solid union body
  padding_solid union body
  hole_solid cut body
  bore.body cut body

  shank := { q | q inside std.Cylinder(grip_ax, r: m.pin_r)
                 center distance(in: (-t / 2, m.grip), along: ax) q }
  cap := { q | q inside std.Cylinder(grip_ax, r: m.handle_r)
               center distance(in: (m.grip, m.grip + m.head_w), along: ax) q }
  sleeve := { q | q inside std.Cylinder(grip_ax, r: m.handle_r)
                  q outside std.Cylinder(grip_ax, r: bore_r)
                  center distance(in: (t / 2 + m.loose, m.grip - m.loose), along: ax) q }
  private construction shank_solid := solid(shank)
  private construction cap_solid := solid(cap)
  pin := solid()
  shank_solid union pin
  cap_solid union pin
  handle := solid(sleeve)
}

// The frame, inside the pair: a post where the shafts cross, down to a round floor under both,
// spoked about the post's own axis — the shafts' common perpendicular — and a sleeve along each
// shaft from the post into the hollow of its member, bored for the journal. Its three directions
// are the two axes and their common perpendicular; each piece is the points between levels
// along them, or within a radius of one, each level an ordinate from a point the pair places —
// along a shaft from the foot of M on it or from the crossing, along the perpendicular from
// either axis.
component Frame(g: line, gf: point, p: line, pf: point, cross: Perpendicular, m: group,
    f: group) {
  post := { q | cross.on_a distance(in: (-m.post, m.post), along: g) q
                cross.on_a distance(in: (-m.post, m.post), along: p) q
                cross.on_a distance(>= -f.below, along: cross.span) q
                cross.on_b distance(<= 0mm, along: cross.span) q }
  floor_at := f.plate - f.below
  hub := { q | q inside std.Cylinder(cross.span, r: f.hub_r)
               cross.on_a distance(in: (-f.below, floor_at), along: cross.span) q }
  rim := { q | q inside std.Cylinder(cross.span, r: f.floor_r)
               q outside std.Cylinder(cross.span, r: f.floor_r - f.rim)
               cross.on_a distance(in: (-f.below, floor_at), along: cross.span) q }
  private spoked := Spokes(cross.span, cross.on_a, g, lo: -f.below, hi: floor_at,
    width: f.spoke_w, reach: f.floor_r - 1mm, bars: f.spokes)
  gear_sleeve := { q | q inside std.Cylinder(g, r: m.sleeve_r)
                       cross.on_a distance(>= -m.post, along: g) q
                       gf distance(<= m.front - m.play, along: g) q }
  pinion_sleeve := { q | q inside std.Cylinder(p, r: m.sleeve_r)
                         cross.on_b distance(>= -m.post, along: p) q
                         pf distance(<= m.front - m.play, along: p) q }
  gear_bore := { q | q inside std.Cylinder(g, r: m.journal + m.clearance)
                     cross.on_a distance(>= -m.post - 1mm, along: g) q
                     gf distance(<= m.front, along: g) q }
  pinion_bore := { q | q inside std.Cylinder(p, r: m.journal + m.clearance)
                       cross.on_b distance(>= -m.post - 1mm, along: p) q
                       pf distance(<= m.front, along: p) q }
  private construction post_solid := solid(post)
  private construction hub_solid := solid(hub)
  private construction rim_solid := solid(rim)
  private construction gear_sleeve_solid := solid(gear_sleeve)
  private construction pinion_sleeve_solid := solid(pinion_sleeve)
  private construction gear_bore_solid := solid(gear_bore)
  private construction pinion_bore_solid := solid(pinion_bore)
  body := solid()
  post_solid union body
  hub_solid union body
  rim_solid union body
  spoked.body union body
  gear_sleeve_solid union body
  pinion_sleeve_solid union body
  gear_bore_solid cut body
  pinion_bore_solid cut body
}
