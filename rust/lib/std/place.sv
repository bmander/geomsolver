// `std.place`: where points, lines and planes stand from one another, as relation words over
// the language's own dimensions — a file says `use std.place (horizontal, right_of)` to write
// them.

// Two points at one height, and one above the other, in the view they are drawn in: the
// ordinate's zero along the view's up and along its right (§9.9).  A file says `use std.place
// (horizontal, vertical)` to write them; `horizontal l` of a line is the language's own.
a horizontal b := a level(up) b
a vertical b := a level(right) b

// Where one point stands from another in the view both are drawn in (§9.9, #103): `b offset(dx:
// 30, dy: 12) a` — `b` 30 to the right of `a` and 12 above it, each a dimension; and the four
// steps along one of the view's axes, `b right_of(d: 30) a` — `b` 30 to the right of `a`, level
// with it — `left_of`, `above` and `below`.
b offset(dx, dy) a := {
  a distance(dx, along: u) b
  a distance(dy, along: v) b
}
b right_of(d) a := {
  a distance(d, along: right) b
  a level(up) b
}
b left_of(d) a := {
  a distance(d, along: left) b
  a level(up) b
}
b above(d) a := {
  a distance(d, along: up) b
  a level(right) b
}
b below(d) a := {
  a distance(d, along: down) b
  a level(right) b
}

// Where a point stands in a plane's own frame, from its origin (§9.9, #103): `p coords(du: 20,
// dv: 5) P` — 20 along `P.u` and 5 along its `v` — and on one of its axes, `p on_u(d: 20) P`,
// `p on_v(d: 5) P`.
p coords(du, dv) P := {
  p distance(du, along: u) P
  p distance(dv, along: v) P
}
p on_u(d) P := {
  p distance(d, along: u) P
  p level(v) P
}
p on_v(d) P := {
  p level(u) P
  p distance(d, along: v) P
}

// Two lines crossed at an angle and a distance apart — two shafts, the angle between them and
// their common perpendicular, in space: `gax skew(theta: 90deg, e: 20mm) pax`.
a skew(theta, e) b := {
  a angle(theta) b
  a distance(e) b
}

// A line the length of another, turned from it by an angle — `ax turned(theta: 180deg) t.ax`
// is `t.ax` reversed, `turned(theta: 0deg)` a copy carried across.
a turned(theta) b := {
  b angle(theta) a
  a equal b
}

// A plane square to a line or an axis, its `v` along a direction (#250): `n square(up: std.z) g`
// — `n`'s `v` along `up`, its `u` square to `up` and to `g`, so with `up` square to `g` its
// normal runs along `g`.  A plane's own axes are not held square to one another (§6.7), so it
// takes all three to say.  `up` square to `g` is the drawing's to keep: where it is not, the
// normal is `g`'s part square to `up`.
P square(up) l := {
  P.v parallel up
  P.u perpendicular l
  P.u perpendicular up
}

// A part seated (#595): an instance drawn about its own origin, placed by a mate on its frame
// said once — its origin on `p`, its z running along `up` and its x turned `turn` from `east`
// about `up`, right handed: `gear seats(turn: 15deg, up: std.z, east: std.x) hub`.  The mate of
// 1.12 written out is these three relations on `g.std`.
g seats(turn, up, east) p := {
  g.std.origin coincident p
  g.std.z parallel(sense: same) up
  east angle(turn, about: up) g.std.x
}

// A point raised from another (#595): `q` stands `d` from `p` along `along`, on the axis through
// `p` that runs that way — `q raised(d: 12mm, along: std.z) p`, a station above a point drawn in
// a plane, where a part's origin seats at its height.  An axis rather than a line from `p` to `q`,
// so the direction stays held at any `d`.
q raised(d, along) p := {
  private l := axis
  l parallel along
  p coincident l
  q coincident l
  p distance(d, along: along) q
}
