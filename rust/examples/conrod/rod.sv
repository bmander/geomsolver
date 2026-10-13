// A forged connecting rod, rounded in Solvent: #66's closing case and #183's L8, after X0's paper
// program (docs/fillets/x0/conrod.sv). Two eyes joined by a tapering I-section shank, the shank
// filled into both eyes, a pocket each side of the web rounded inside, the bores chamfered, and
// the outside's edges rounded. Draft from the parting plane is left out (#66).
//
// `engine/conrod.sv` draws a rod in its plane of swing, where the shank-to-eye fillets are 42
// statement lines of 2D geometry. Here the sketch is the sharp outline, and every round and
// chamfer is one statement of `std.sv`'s words over the faces the solids gave the rod: the
// rounds act on the rod as its other terms leave it (the stratum), so it does not matter that the
// edges they round are where an eye meets the shank or a pocket was cut.
//
// The plane of swing is the front plane: the big end's pin at its origin, the rod along z, its
// width across the engine along y. Orbit (right-drag) to see the pockets and the rounds. Edit a
// number and apply (⌘↵); rod.svd is the drawing.
unit mm
use std (round, chamfer)

L := 140mm       // centre to centre
eB := 30mm       // big-end eye, outside
eS := 16mm       // small-end eye, outside
rB := 23.5mm     // big-end bore: the crank pin and a bearing shell
rS := 11mm       // small-end bore: the piston pin and a bush
hB := 12mm       // the shank's half-width where it leaves the big end …
hS := 9mm        // … and where it meets the small end
wB := 28mm       // the big end's width across the engine
wS := 22mm       // the small end's
fl := 18mm       // the shank's width across: its flanges
ft := 4mm        // flange thickness, and the pockets' clearance from each eye
wt := 5mm        // web thickness
rf := 6mm        // the shank into the eyes
rp := 2mm        // the pockets, inside
re := 1mm        // the outside's edges
ch := 1mm        // the bores' chamfers

// The plane of swing: the eyes, the bores, the shank's outline and the pockets', sharp.
in std.front {
  pin := point
  sm := point
  fix((0, 0)) pin
  fix((0, L)) sm
  big := circle(center: pin) hint(r: 30)
  small := circle(center: sm) hint(r: 16)
  radius(eB) big
  radius(eS) small
  big_bore := circle(center: pin) hint(r: 23.5)
  small_bore := circle(center: sm) hint(r: 11)
  radius(rB) big_bore
  radius(rS) small_bore

  s0 := point
  s1 := point
  s2 := point
  s3 := point
  fix((-hB, 0)) s0
  fix((hB, 0)) s1
  fix((hS, L)) s2
  fix((-hS, L)) s3
  (foot := line(s0, s1)) -> (right := line(s1, s2)) -> (head := line(s2, s3)) ->
    (left := line(s3, s0)) -> close

  // a pocket's outline: a flange's thickness inside the shank's sides, as far clear of each eye
  q1 := point hint((7.3, 33.2))
  q2 := point hint((5.4, 120.7))
  q3 := point hint((-5.4, 120.7))
  q4 := point hint((-7.3, 33.2))
  pr := line(q1, q2)
  pl := line(q3, q4)
  q1 distance(ft) right
  q2 distance(ft) right
  q3 distance(ft) left
  q4 distance(ft) left
  kb := arc(center: pin, start: q1, end: q4) hint(r: 34)
  kt := arc(center: sm, start: q3, end: q2) hint(r: 20)
  radius(eB + ft) kb
  radius(eS + ft) kt
  pocket := face(pr, kt, pl, kb)
}

big_end := solid(face(big), from: -wB / 2, to: wB / 2)
small_end := solid(face(small), from: -wS / 2, to: wS / 2)
shank := solid(face(foot, right, head, left), from: -fl / 2, to: fl / 2)
big_hole := solid(face(big_bore), from: -wB, to: wB)
small_hole := solid(face(small_bore), from: -wS, to: wS)
// beyond half the web from the plane of swing, each side: a prism of the outline, so its walls
// are planes and cylinders the rounds are built exactly against
front_pocket := solid(pocket, from: wt / 2, to: fl)
back_pocket := solid(pocket, from: -fl, to: -wt / 2)

// square to the rod halfway along: where the drawing cuts its section through the I
across := plane
fix(origin == (0, 0, L / 2)) across
fix(dir == (1, 0, 0)) across.u
fix(dir == (0, 1, 0)) across.v

rod := solid(shank)
big_end union rod
small_end union rod
big_hole cut rod
small_hole cut rod
front_pocket cut rod
back_pocket cut rod

// The shank's sides into the eyes: the four upright edges in the plane of swing, concave, so
// filled. Side by side rather than `rod.shank round(r: rf) rod.big_end`, which would also take the
// edges where the shank's flat faces meet the eye's wall, where a 6 mm ball does not fit beside
// the pockets.
rod.shank.left round(r: rf) rod.big_end
rod.shank.right round(r: rf) rod.big_end
rod.shank.left round(r: rf) rod.small_end
rod.shank.right round(r: rf) rod.small_end
// The pockets inside: each floor's edges, its upright corners, and where three meet a ball's.
round(r: rp) rod.front_pocket
round(r: rp) rod.back_pocket
// Both ends of each bore.
rod.big_end chamfer(d: ch) rod.big_hole
rod.small_end chamfer(d: ch) rod.small_hole
// The eyes' rims and the flanges' four long edges, each run cut off by the eye it ends on.
round(r: re) rod.big_end
round(r: re) rod.small_end
round(r: re) rod.shank

// Two sets of edges are left sharp: each round reads the rod as its other terms leave it, never
// another round, so where rounds of separate statements meet they cannot blend. Add either and
// the field and the exact build read the rod as written, but the material the rounds leave
// touches itself along a line, which no manifold file describes: the export is refused, naming
// where (#218):
//   rod.shank.near round(r: re) rod.big_end   (and `far`, and `small_end`: the shank's faces into
//                                              the eyes, meeting the flanges' runs and the 6 mm
//                                              fills at one vertex)
//   rod.front_pocket round(r: re) rod.shank   (and `back_pocket`: the pockets' rims, whose runs
//                                              cut down past the corners' fills on both sides)
