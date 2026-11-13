// The frame: a 56 cm road frame in 7005 aluminium, drawn from the numbers a frame is specified by
// (stack, reach, the seat and head tube angles, the chainstay and the bottom bracket's drop) and
// built of hollow tubes about the lines between its nodes.  This is all of it but the drive side's
// chainstay, the bottom bracket's shell and the drive-side dropout, which house the drive and are
// drawn about its shaft and axles in `drive.sv`.
//
// The side view is the world's `std.front` (`std.x` forward, `std.z` up), the bottom bracket's
// centre at its origin; the frame's mid-plane is `y = 0`, the drive side toward −y.
unit mm
use std
use std.sets
use std.solids (cut, union)
use std.place (horizontal)
use dims (seat_angle, seat_length, stack, reach, head_angle, head_length, cluster, stay_spread,
          old, head_od, head_id, down_od, down_id, seat_od, seat_id, top_od, top_id, stay_od,
          stay_id, ns_stay_od, ns_stay_id, ns_stay_y, dropout_t, dropout_r, stay_short, stay_below)

// A point `dy` along `std.y` from `q`, beside it.
component Beside(q: point, dy: Length) {
  p := point
  q distance(0mm, along: std.x) p
  q distance(0mm, along: std.z) p
  q distance(dy, along: std.y) p
}

// A tube about the line from `a` to `b`, `od` across and `id` bored, running `past_a` beyond `a`
// and `past_b` beyond `b`; `plug` is the tube as a bar, what a tube joining it is coped to: a
// mitre is one statement, `receiving.plug cut joining.body`, the joining tube's end cut to the
// receiving tube's outside, its wall gone from inside the receiving tube's bore.
component Tube(a: point, b: point, od: Length, id: Length, past_a: Length, past_b: Length) {
  construction ax := line(a, b)
  wall := { q | q in std.sets.Cylinder(ax, r: od / 2)
                q not in std.sets.Cylinder(ax, r: id / 2)
                a distance(>= -past_a, along: ax) q
                b distance(<= past_b, along: ax) q }
  body := solid(wall)
  construction plug := solid({ q | q in std.sets.Cylinder(ax, r: od / 2)
                                   a distance(>= -past_a, along: ax) q
                                   b distance(<= past_b, along: ax) q })
}

// The frame about the bottom bracket's centre `bb` and the rear axle's centre `axle`, both drawn
// in the side view.
component Frame(bb: point, axle: point) {
  in std.front {
    // the seat tube: `seat_angle` from level, `seat_length` long
    ahead := point
    construction ground := line(bb, ahead)
    horizontal ground
    length(100mm) ground
    bb distance(>= 0mm, along: u) ahead
    seat_top := point
    construction seat_ax := line(bb, seat_top)
    length(seat_length) seat_ax
    ground angle(180deg - seat_angle) seat_ax
    // the head tube's top, `stack` over and `reach` ahead of the bracket; its bottom
    // `head_length` down its axis, `head_angle` from level
    head_top := point
    bb distance(reach, along: u) head_top
    bb distance(stack, along: v) head_top
    head_bottom := point
    construction head_ax := line(head_top, head_bottom)
    length(head_length) head_ax
    ground angle(-head_angle) head_ax
    // where the top tube and the seat stays meet the seat tube, `cluster` below its top, and
    // where the top and down tubes meet the head tube, 25 mm in from its ends
    cluster_c := point
    cluster_c coincident seat_ax
    seat_top distance(cluster) cluster_c
    top_head := point
    top_head coincident head_ax
    head_top distance(25mm) top_head
    down_head := point
    down_head coincident head_ax
    head_bottom distance(25mm) down_head
    bb distance(>= 0mm, along: seat_ax) cluster_c
    head_top distance(>= 0mm, along: head_ax) top_head
    head_bottom distance(<= 0mm, along: head_ax) down_head
    // where the seat stays meet the seat tube, `stay_below` under the top tube
    stay_c := point
    stay_c coincident seat_ax
    cluster_c distance(stay_below) stay_c
    bb distance(>= 0mm, along: seat_ax) stay_c
    cluster_c distance(<= 0mm, along: seat_ax) stay_c
  }
  // the stays' ends: either side of the seat tube under the cluster, at each dropout's inner face,
  // and the non-drive chainstay's foot on the shell
  stay_l := Beside(stay_c, dy: stay_spread)
  stay_r := Beside(stay_c, dy: -stay_spread)
  drop_l := Beside(axle, dy: old / 2 - dropout_t)
  drop_r := Beside(axle, dy: -old / 2 + dropout_t)
  cs_foot := Beside(bb, dy: ns_stay_y)

  seat := Tube(bb, seat_top, od: seat_od, id: seat_id, past_a: 0mm, past_b: 0mm)
  head := Tube(head_top, head_bottom, od: head_od, id: head_id, past_a: 0mm, past_b: 0mm)
  top := Tube(cluster_c, top_head, od: top_od, id: top_id, past_a: 0mm, past_b: 0mm)
  down := Tube(bb, down_head, od: down_od, id: down_id, past_a: 0mm, past_b: 0mm)
  seat_stay_l := Tube(stay_l.p, drop_l.p, od: stay_od, id: stay_id, past_a: 0mm, past_b: -stay_short)
  seat_stay_r := Tube(stay_r.p, drop_r.p, od: stay_od, id: stay_id, past_a: 0mm, past_b: -stay_short)
  chainstay_l := Tube(cs_foot.p, drop_l.p, od: ns_stay_od, id: ns_stay_id, past_a: 0mm,
    past_b: -stay_short)

  // the non-drive dropout: a plate about the axle, `dropout_t` thick, inboard of its face, that the
  // stays end on, short of the axle
  axle_end := Beside(axle, dy: 10mm)
  construction axle_ax := line(axle, axle_end.p)
  dropout_l := solid({ q | q in std.sets.Cylinder(axle_ax, r: dropout_r)
                           q not in std.sets.Cylinder(axle_ax, r: 6mm)
                           axle distance(in: (old / 2 - dropout_t, old / 2), along: axle_ax) q })

  // the mitres: the top tube coped to the seat and head tubes, the down tube to the head tube and
  // the seat stays to the seat tube (the tubes meeting the bottom bracket are coped to its shell,
  // in `drive.sv`)
  seat.plug cut top.body
  head.plug cut top.body
  head.plug cut down.body
  seat.plug cut seat_stay_l.body
  seat.plug cut seat_stay_r.body

  body := solid(seat.body)
  head.body union body
  top.body union body
  down.body union body
  seat_stay_l.body union body
  seat_stay_r.body union body
  chainstay_l.body union body
  dropout_l union body
}
