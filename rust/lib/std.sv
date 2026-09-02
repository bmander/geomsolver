// The standard library: what most drawings start from, written once.
//
// `use std` brings it in — from the library compiled into the core, so it is there in the browser
// as in the terminal; a `std.sv` beside a document would win over it, as any module does.

// The three principal views of third-angle projection (§6.7), laid out on one sheet: the page is
// the **front** view, its origin at `o`; the **right** view is folded from the front's z-axis and
// drawn `right` along the page from `o`, turned so up stays up; the **top** view is folded up from
// the front's x-axis and drawn `up` the page from `o`.  Every origin is one corner of the object,
// as each view sees it, so the views need no projection between their origins — a drawing states
// its projections between the points it draws.  Ground `o` and the sheet is placed; the spacing
// is stated here, so dragging `o` moves the whole sheet.
//
//   use std
//   point O hint(x: 0, y: 0)
//   ground O
//   views: ThreeViews(O, right: 620, up: 620)
//   part: Something(views.right_origin) in views.right
//
// The views are `front`, `right` and `top`, reached as `views.right`; `right` the length and
// `right` the view do not meet, since a number is read in an expression and an entity in a
// reference.
component ThreeViews(o: point, right: Length, up: Length) {
  // each view's `toward` point sets which way it is turned on the page; a hand's breadth away
  point qf
  o distance(40, along: x) qf
  o distance(0, along: y) qf
  plane front(origin: o, toward: qf)

  point right_origin
  point qr
  o distance(right, along: x) right_origin
  o distance(0, along: y) right_origin
  right_origin distance(0, along: x) qr
  right_origin distance(-40, along: y) qr
  plane right(origin: right_origin, toward: qr, from: front, fold: -90deg)

  point top_origin
  point qt
  o distance(0, along: x) top_origin
  o distance(up, along: y) top_origin
  top_origin distance(40, along: x) qt
  top_origin distance(0, along: y) qt
  plane top(origin: top_origin, toward: qt, from: front, fold: 0deg)
}
