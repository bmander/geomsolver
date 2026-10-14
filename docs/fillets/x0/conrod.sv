// X0 (#183): #66's closing case, a forged connecting rod. Two eyes joined by a tapering I-section
// shank, the shank filled into both eyes, a pocket each side of the web rounded inside, the
// bores chamfered, and every other edge rounded. Draft from the parting plane is left out (#66).
// Does not parse yet: rounds and chamfers on solids, face groups, distances from faces and solids
// in a region's body. As it parses now, and built exactly: rust/examples/conrod/rod.sv (#197).
//
// `engine/conrod.sv` draws the same rod in its plane of swing, where the shank-to-eye fillets are
// 42 statement lines of 2D geometry: four arcs, their centres, rays, contact points and seeds, each
// tangency stated at its point. Here they are a face group and two rounds, and the sketch is the
// sharp outline.
unit mm
use std
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
re := 1mm        // every other edge
ch := 1mm        // the bores' chamfers

// The plane of swing: the eyes, the bores and the shank's outline, sharp.
in std.front {
  pin := point
  sm := point
  fix((0, 0)) pin
  fix((0, L)) sm
  big := circle(center: pin) hint(r: eB)
  small := circle(center: sm) hint(r: eS)
  radius(eB) big
  radius(eS) small
  big_bore := circle(center: pin) hint(r: rB)
  small_bore := circle(center: sm) hint(r: rS)
  radius(rB) big_bore
  radius(rS) small_bore
  axis := line(pin, sm)
  s0 := point hint((-hB, 0))
  s1 := point hint((hB, 0))
  s2 := point hint((hS, L))
  s3 := point hint((-hS, L))
  (foot := line(s0, s1)) -> (right := line(s1, s2)) -> (head := line(s2, s3)) ->
    (left := line(s3, s0)) -> close
  pin midpoint foot
  sm midpoint head
  foot perpendicular axis
  head perpendicular axis
  length(2 * hB) foot
  length(2 * hS) head
}

big_end := solid(face(big), from: -wB / 2, to: wB / 2)
small_end := solid(face(small), from: -wS / 2, to: wS / 2)
shank := solid(face(foot, right, head, left), from: -fl / 2, to: fl / 2)
big_hole := solid(face(big_bore), from: -wB, to: wB)
small_hole := solid(face(small_bore), from: -wS, to: wS)

// A pocket each side of the web: a flange's thickness inside the shank's sides, as far clear of
// each eye, and beyond half the web from the plane of swing. A region over the operands' own faces
// and solids, not the rod's, so it reads nothing the rod's body makes (X4).
front_pocket := solid({ p |
  p distance(>= ft, side: inside) shank.left
  p distance(>= ft, side: inside) shank.right
  p distance(>= ft, side: outside) big_end
  p distance(>= ft, side: outside) small_end
  p distance(>= wt / 2, side: outside) std.front
})
back_pocket := solid({ p |
  p distance(>= ft, side: inside) shank.left
  p distance(>= ft, side: inside) shank.right
  p distance(>= ft, side: outside) big_end
  p distance(>= ft, side: outside) small_end
  p distance(>= wt / 2, side: inside) std.front
})

rod := solid(shank)
big_end union rod
small_end union rod
big_hole cut rod
small_hole cut rod
front_pocket cut rod
back_pocket cut rod

// The shank's sides into the eyes: the four upright edges in the plane of swing, concave. A face
// group (finding 4), since `rod.shank round(rf) rod.big_end` would also take the edges where the
// shank's flat faces meet the eye's wall, which a 6 mm ball cannot fit beside the pockets.
sides := {left: rod.shank.left, right: rod.shank.right}
sides round(rf) rod.big_end
sides round(rf) rod.small_end
// The pockets inside: each floor's edges and its upright corners.
round(rp) rod.front_pocket
round(rp) rod.back_pocket
// Both ends of each bore.
rod.big_end chamfer(ch) rod.big_hole
rod.small_end chamfer(ch) rod.small_hole
// Every other edge (finding 5: which edges "other" means is the open question).
round(re) rod
