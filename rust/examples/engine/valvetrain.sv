// The valvetrain, seen along the camshafts: a tangent cam on a flat-faced bucket follower over an
// inclined valve.  A cam lobe here is what a draughtsman draws — a base circle, a nose circle and
// two straight flanks tangent to both — and the follower face is tangent to whichever circle it
// rides, so the valve's lift is the solver's answer and not a number anyone typed.

use engine.dims (rb, rn, stem)
use engine.parts
use std

// a valve's own sizes: its stem, the margin round its head and the seat face's width at 45°, the
// neck where the head's tulip meets the stem, and the bucket — its radius and depth, the follower's
// face on top, and its skirt and crown
rstem := 3.5mm
margin := 1.5mm
seatw := 2.5mm
neck := 14mm
rbucket := 15mm
dbucket := 20mm
skirt := 1.5mm
crown := 3mm
// and what the head is bored for one: the bucket's bore, from inside the cam's pocket down to its
// floor (both measured down the axis from the cam's centre), the stem's guide, and the throat the
// valve's head lifts out of, this far up from its seat
bucket_top := 20mm
bucket_floor := 52mm
rbore := rbucket + 0.2mm
rguide := 4mm
throat := 16mm

// A tangent cam about `c`, its nose `dn` out at `phi` from the line `ref`, measured
// counter-clockwise: the outline a draughtsman draws, an arc of the base circle round the back,
// an arc of the nose circle round the front, and the two flanks tangent to both where they meet
// them.  Walked counter-clockwise, the right flank runs up from the base to the nose and the left
// one back down, both circles' centres on the walk's left; a flank leaves the base circle `90° -
// lean` either side of the nose, `lean` the angle the flanks close in by.  The outline is
// `profile`, a face, which the camshaft sweeps along the axis.
component Lobe(c: point, ref: line, phi: Angle, dn: Length) {
  lean := asin((rb - rn) / dn)
  n := point hint(x: c.x + dn * cos(phi + atan2(ref.p2.y - ref.p1.y, ref.p2.x - ref.p1.x)),
               y: c.y + dn * sin(phi + atan2(ref.p2.y - ref.p1.y, ref.p2.x - ref.p1.x)))
  spine := line(c, n)
  c distance(dn) n
  ref angle(phi) spine
  // where the flanks touch the base circle and the nose circle, right and left of the spine
  br := point hint(at: c, toward: n, by: rb / dn, turn: lean - 90deg)
  bl := point hint(at: c, toward: n, by: rb / dn, turn: 90deg - lean)
  nr := point hint(at: n, toward: c, by: -rn / dn, turn: lean - 90deg)
  nl := point hint(at: n, toward: c, by: -rn / dn, turn: 90deg - lean)
  base := arc(center: c, start: bl, end: br) hint(r: rb)
  nose := arc(center: n, start: nr, end: nl) hint(r: rn)
  radius(rb) base
  radius(rn) nose
  fr := line(br, nr)
  fl := line(nl, bl)
  fr tangent(at: p1) base
  fr tangent(at: p2) nose
  fl tangent(at: p1) nose
  fl tangent(at: p2) base
  ccw(br, nr, c)
  ccw(br, nr, n)
  ccw(nl, bl, c)
  ccw(nl, bl, n)
  profile := face(fr, nose, fl, base)
}

// A valve on the axis from its seat centre `seat` toward the cam centre at `axis.p2`, lifted
// `lift` off its seat by the lobe: the flat follower face stands `rb + lift` from the cam's
// centre, which is where the lobe's outline reaches at this moment (see `dims.sv`) — so the face
// is tangent to whichever of base circle, flank or nose is under it, without saying which.
component Valve(seat: point, ax: line, lift: Length, head: Length) {
  // the follower face: on the axis and square to it, `rb + lift` short of the cam's centre
  fc := point hint((ax.p2.x - (rb + lift) * (ax.p2.x - ax.p1.x) / stem, ax.p2.y - (rb + lift) * (ax.p2.y - ax.p1.y) / stem))
  fc coincident ax
  ax.p2 distance(rb + lift) fc
  f1 := point hint((ax.p2.x - rbucket, ax.p2.y - rb))
  f2 := point hint((ax.p2.x + rbucket, ax.p2.y - rb))
  flat := line(f1, f2)
  fc midpoint flat
  flat perpendicular ax
  f1 distance(2 * rbucket) f2
  // the bucket under the face, `2 rbucket` wide and `dbucket` deep
  b1 := point hint((ax.p2.x - rbucket, ax.p2.y - rb - dbucket))
  b2 := point hint((ax.p2.x + rbucket, ax.p2.y - rb - dbucket))
  bl := line(f1, b1)
  br := line(f2, b2)
  bb := line(b1, b2)
  bl parallel ax
  br parallel ax
  f1 distance(dbucket) b1
  f2 distance(dbucket) b2
  // the stem, `stem` down the axis to the head, which the lobe lifts off its seat or does not
  hc := point hint(at: seat)
  hc coincident ax
  hc distance(stem) fc
  st := line(hc, fc)
  h1 := point hint((seat.x - head / 2, seat.y))
  h2 := point hint((seat.x + head / 2, seat.y))
  hd := line(h1, h2)
  hc midpoint hd
  hd perpendicular ax
  h1 distance(head) h2
  // which way each piece stands, the axis running from the seat up to the cam: the face's `f1`
  // and the head's `h1` on its left, the cam above the face and the bucket and the stem under it
  ccw(ax.p1, ax.p2, f1)
  ccw(ax.p1, ax.p2, h1)
  ccw(f1, f2, ax.p2)
  cw(f1, f2, b1)
  cw(f1, f2, b2)
  cw(f1, f2, hc)
}

// The valve `v` draws (a `Valve`, drawn as construction where the solids are the picture) and its
// bucket as solids, each the half section left of the axis turned about it: the valve from its
// head's face over the chamber up the margin and the 45° seat face to the neck, then the stem up to
// the bucket's crown; the bucket a cup over the stem, open below, its face the follower's.  Each
// corner is two dimensions, from the axis and up it from the head's face — or down it from the
// follower's — so the sections turn with the axis however it leans, and lift with the valve.  A
// corner is seeded where those two put it: `by` the fraction of the run up the axis to the cam's
// centre (`up`), or down it to the seat (`down`), turned off the axis by `turn`.
component Poppet(v: group) {
  up := stem + rb + v.lift
  down := stem - v.lift
  // the valve, up the axis from the head's face `v.hd`
  construction m1 := point hint(at: v.hc, toward: v.ax.p2,
                                by: sqrt(v.head ^ 2 / 4 + margin ^ 2) / up,
                                turn: atan2(v.head / 2, margin))
  construction m2 := point hint(at: v.hc, toward: v.ax.p2,
                                by: sqrt((v.head / 2 - seatw) ^ 2 + (margin + seatw) ^ 2) / up,
                                turn: atan2(v.head / 2 - seatw, margin + seatw))
  construction m3 := point hint(at: v.hc, toward: v.ax.p2, by: sqrt(rstem ^ 2 + neck ^ 2) / up,
                                turn: atan2(rstem, neck))
  construction m4 := point hint(at: v.hc, toward: v.ax.p2, by: (stem - crown) / up,
                                turn: atan2(rstem, stem - crown))
  construction m5 := point hint(at: v.hc, toward: v.ax.p2, by: (stem - crown) / up)
  m1 distance(v.head / 2, side: left) v.ax
  m1 distance(margin, side: left) v.hd
  m2 distance(v.head / 2 - seatw, side: left) v.ax
  m2 distance(margin + seatw, side: left) v.hd
  m3 distance(rstem, side: left) v.ax
  m3 distance(neck, side: left) v.hd
  m4 distance(rstem, side: left) v.ax
  m4 distance(stem - crown, side: left) v.hd
  m5 coincident v.ax
  m5 distance(stem - crown, side: left) v.hd
  valve := solid(face(v.hc, v.h1, m1, m2, m3, m4, m5, -> close), about: v.ax)
  // the bucket, down the axis from its face `v.flat`
  construction c1 := point hint(at: v.fc, toward: v.ax.p1,
                                by: sqrt((rbucket - skirt) ^ 2 + dbucket ^ 2) / down,
                                turn: -atan2(rbucket - skirt, dbucket))
  construction c2 := point hint(at: v.fc, toward: v.ax.p1,
                                by: sqrt((rbucket - skirt) ^ 2 + crown ^ 2) / down,
                                turn: -atan2(rbucket - skirt, crown))
  construction c3 := point hint(at: v.fc, toward: v.ax.p1, by: crown / down)
  c1 distance(rbucket - skirt, side: left) v.ax
  c1 distance(dbucket, side: right) v.flat
  c2 distance(rbucket - skirt, side: left) v.ax
  c2 distance(crown, side: right) v.flat
  c3 coincident v.ax
  c3 distance(crown, side: right) v.flat
  bucket := solid(face(v.fc, v.f1, v.b1, c1, c2, c3, -> close), about: v.ax)
}

// What the head is bored for one valve on the axis `ax` (its seat up to the cam's centre), its
// head `head` across, as one turned cutter: the bucket's bore, open into the cam's pocket above,
// the stem's guide below it, and the throat the valve's head lifts out of, open into the chamber.
// Each corner is measured from the axis and down it from the cam's centre, which no lift moves:
// the line `cx` square to the axis there, `q` on its left.  The corners are seeded as the valve's
// are, down the run `reach` from the cam's centre to the seat.
component Bore(ax: line, head: Length) {
  reach := stem + rb
  rthroat := head / 2 + 0.5mm
  q := point hint(at: ax.p2, toward: ax.p1, by: 10mm / reach, turn: -90deg)
  cx := line(ax.p2, q)
  cx perpendicular ax
  ax.p2 distance(10) q
  ccw(ax.p1, ax.p2, q)
  t0 := point hint(at: ax.p2, toward: ax.p1, by: bucket_top / reach)
  t1 := point hint(at: ax.p2, toward: ax.p1, by: sqrt(rbore ^ 2 + bucket_top ^ 2) / reach,
                   turn: -atan2(rbore, bucket_top))
  t2 := point hint(at: ax.p2, toward: ax.p1, by: sqrt(rbore ^ 2 + bucket_floor ^ 2) / reach,
                   turn: -atan2(rbore, bucket_floor))
  t3 := point hint(at: ax.p2, toward: ax.p1, by: sqrt(rguide ^ 2 + bucket_floor ^ 2) / reach,
                   turn: -atan2(rguide, bucket_floor))
  t4 := point hint(at: ax.p2, toward: ax.p1, by: sqrt(rguide ^ 2 + (reach - throat) ^ 2) / reach,
                   turn: -atan2(rguide, reach - throat))
  t5 := point hint(at: ax.p2, toward: ax.p1, by: sqrt(rthroat ^ 2 + (reach - throat) ^ 2) / reach,
                   turn: -atan2(rthroat, reach - throat))
  t6 := point hint(at: ax.p2, toward: ax.p1, by: sqrt(rthroat ^ 2 + (reach + 1mm) ^ 2) / reach,
                   turn: -atan2(rthroat, reach + 1mm))
  t7 := point hint(at: ax.p2, toward: ax.p1, by: (reach + 1mm) / reach)
  t0 coincident ax
  t0 distance(bucket_top, side: left) cx
  t1 distance(rbore, side: left) ax
  t1 distance(bucket_top, side: left) cx
  t2 distance(rbore, side: left) ax
  t2 distance(bucket_floor, side: left) cx
  t3 distance(rguide, side: left) ax
  t3 distance(bucket_floor, side: left) cx
  t4 distance(rguide, side: left) ax
  t4 distance(reach - throat, side: left) cx
  t5 distance(rthroat, side: left) ax
  t5 distance(reach - throat, side: left) cx
  t6 distance(rthroat, side: left) ax
  t6 distance(reach + 1mm, side: left) cx
  t7 coincident ax
  t7 distance(reach + 1mm, side: left) cx
  cutter := solid(face(t0, t1, t2, t3, t4, t5, t6, t7, -> close), about: ax)
}

// Open this file to preview cylinder 1's intake valve and its lobe where the table's cycle puts
// them, stood upright: the valve's seat at the origin of the end view, the cam's centre `stem +
// rb` above it.
preview {
  unit mm
  in std.side {
    cam := engine.parts.At(std.side.origin, dx: 0mm, dy: stem + rb)
    vaxis := line(std.side.origin, cam.p)
    ai := (engine.dims.cycle - engine.dims.icenter) / 2
    lobe := Lobe(cam.p, vaxis, phi: 180deg + ai, dn: engine.dims.dn_i)
    valve := Valve(std.side.origin, vaxis, lift: max(rb, engine.dims.dn_i * cos(ai) + rn) - rb,
                   head: engine.dims.div)
    poppet := Poppet(valve)
    bore := Bore(vaxis, head: engine.dims.div)
  }
}
