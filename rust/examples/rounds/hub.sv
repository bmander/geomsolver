// A beam running into a hub (#763): a bar 3 deep, drawn in plan, its end swallowed by a round
// post that rises past the bar's top. The bar's sides are filled into the post's wall, where they
// stand up along it: the edges in plan. Where the post rises from the bar's top the two meet in an
// arc too, and that edge is left sharp, for whatever is stacked on the bar sits there.
//
// `A round(r) B` rounds every edge where a face of `A` meets one of `B`, so the statement below
// runs over the faces itself and keeps the ones that run along the post's axis: two faces that
// both do (`f parallel std.z`, a plane holding the direction or a cylinder about it) meet in
// lines along it, so these are the edges along the axis and no others. The bar's top and its
// floor run square to it, and its end lies inside the post.
//
// Drawn on the top plane, standing 3 (the bar) and 10 (the post) up from it. Orbit (right-drag)
// to see the arc left sharp. Edit `r` and apply (⌘↵), or round the bar against the post outright
// (`hub.bar round(r: r) hub.post`) to see the arc filled as well.
unit mm
use std
use std.solids (union)
use std.rounds (round)

r := 3mm   // the fillets' radius

in std.top {
  a := point
  b := point
  c := point
  d := point
  fix((0, -4)) a
  fix((40, -4)) b
  fix((40, 4)) c
  fix((0, 4)) d
  (ab := line(a, b)) -> (bc := line(b, c)) -> (cd := line(c, d)) -> (da := line(d, a)) -> close
  o := point
  fix((40, 0)) o
  k := circle(center: o) hint(r: 6)
  radius(6mm) k
}
bar := solid(std.solids.Prism(face(ab, bc, cd, da), from: 0mm, to: 3mm))
post := solid(std.solids.Prism(face(k), from: 0mm, to: 10mm))
hub := solid(bar)
post union hub

// the edges where the bar meets the post along the post's axis
repeat f in hub.bar {
  repeat g in hub.post {
    when f parallel std.z and g parallel std.z {
      f round(r: r) g
    }
  }
}
