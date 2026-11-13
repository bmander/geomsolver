//! **Two faces never draw one diagonal** (`brep::mesh`, shaft_drive README finding 22).  Where a
//! frame's seat stays end beside its seat tube, just below its top tube, the seat tube's bore and
//! the top tube's wall share a short traced edge, sampled so its last chord leaves a sample a
//! hair off the line between its neighbours.  Each face's triangulation cut that sample off with
//! the same diagonal — two triangles of each face on one side of the written mesh, which a strict
//! reader of an STL refuses ("EdgeUseCount { …, count: 3 }") though the B-rep is sound.  An inner
//! side joining two points another face's boundary holds too is split by a point of the face's
//! own, so the four tubes' union writes.
use crate::common::{ent, parsed::determined};
use gcs_core::brep::{export, props, sweep::Say};

const CLUSTER: &str = "unit mm
use std
use std.sets
use std.solids (union)

// A point at `x`, `y`, `z` from the origin.
component At(x: Length, y: Length, z: Length) {
  p := point
  std.origin distance(x, along: std.x) p
  std.origin distance(y, along: std.y) p
  std.origin distance(z, along: std.z) p
}

// A tube about the line from `a` to `b`, ending `short` before `b`.
component Tube(a: point, b: point, od: Length, id: Length, short: Length) {
  construction ax := line(a, b)
  wall := { q | q in std.sets.Cylinder(ax, r: od / 2)
                q not in std.sets.Cylinder(ax, r: id / 2)
                a distance(>= 0mm, along: ax) q
                b distance(<= -short, along: ax) q }
  body := solid(wall)
}

bb := At(x: 0mm, y: 0mm, z: 0mm)
seat_top := At(x: -153.368mm, y: 0mm, z: 517.763mm)
cluster := At(x: -144.848mm, y: 0mm, z: 488.998mm)
top_head := At(x: 392.725mm, y: 0mm, z: 551.224mm)
stay_l := At(x: -144.848mm, y: 14mm, z: 488.998mm)
stay_r := At(x: -144.848mm, y: -14mm, z: 488.998mm)
drop_l := At(x: -424.264mm, y: 64mm, z: 70mm)
drop_r := At(x: -424.264mm, y: -64mm, z: 70mm)

seat := Tube(bb.p, seat_top.p, od: 34.9mm, id: 31.6mm, short: 0mm)
top := Tube(cluster.p, top_head.p, od: 35mm, id: 32mm, short: 0mm)
left := Tube(stay_l.p, drop_l.p, od: 16mm, id: 13mm, short: 26mm)
right := Tube(stay_r.p, drop_r.p, od: 16mm, id: 13mm, short: 26mm)

frame := solid(seat.body)
top.body union frame
left.body union frame
right.body union frame
";

#[test]
fn a_seat_tube_its_top_tube_and_both_stays_write_one_closed_mesh() {
    let e = determined(CLUSTER);
    let s = ent(&e, "frame").i();
    let say = Say { stage: &|_: &str| {}, mark: &|_| {} };
    let exact = export::exact(&e.sketch, s, None, None, &say).unwrap();
    // every edge of the written mesh two triangles meet at, and the volume the B-rep's
    let stl = export::stl(&e.sketch, s, &exact, None, &say).unwrap_or_else(|r| panic!("{r:?}"));
    gcs_core::mesh::stl_shells(&stl).unwrap();
    let v = props::volume(&exact.solid);
    assert!((v - 243298.3).abs() < 1., "{v}");
}
