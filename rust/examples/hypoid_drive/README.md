# Hypoid drive

A hand-cranked right-angle drive to 3D print as a demonstration, built on the hypoid pair of
[`spiral_bevel/hypoid.sv`](../spiral_bevel/hypoid.sv): the 24-tooth pinion and the 48-tooth gear on
square shafts 25 mm apart. Each member has a shaft running in a plain sleeve in one frame, which
stands inside the pair, and each shaft carries a counterweighted crank with a loose handle. The
parts go together with glued press-fit splines.

The point is reuse: **`drive.sv` imports the pair and restates none of it.** One line,
`use spiral_bevel.hypoid (HypoidPair)`, and one call lay out the pitch cones, find the face mill
and cut both members. `hypoid.sv` and its tests stay the pair's single source. Every part here is
placed by lines and sets the pair already names:

| the pair names | the drive reads it as |
|---|---|
| `pair.gear_cone.ax`, `pair.pinion_cone.ax` | each shaft's axis: shaft, sleeve, hub, spline and crank all turn about it |
| `pair.gear_cone.foot`, `pair.pinion_cone.foot` | the foot of the mean point M on each axis, where every ordinate along a shaft is measured from |
| `pair.gear_blank.back`, `pair.pinion_blank.back` | the set bounding each blank's back: each hub lies within the points deeper than it (`q outside seat`), and its rim is those within 2.5 mm of it (`q distance(<= m.rim, side: outside) seat`), so the member sits on its hub exactly |
| `pair.gear.body`, `pair.pinion.body` | the toothed members, each made a printed part with its hub (`gear_side.part`, `pinion_side.part`) |
| `pair.gear_blank.material`, `pair.pinion_blank.material` | each member's blank, which bounds the member: what the clearance claims are judged against |

The only number taken from the pair's design rather than its geometry is the depth of the frame's
floor, `reach`. It is read from the stated tooth counts and module, because a value may not read
the solved drawing.

## The parts to print

| part | in the document | what it is |
|---|---|---|
| frame (1) | `frame.body` | a post where the shafts cross, standing on a floor under both, and a sleeve along each shaft from the post into the hollow of its member |
| shafts (2) | `gear_side.shaft`, `pinion_side.shaft` | a 10 mm journal that runs in its sleeve, a head that stops it at the post, and, from the member's hub out, six splines that pass through the sleeve |
| gear and pinion (2) | `gear_side.part`, `pinion_side.part` | the imported member and its spoked hub as one part: a rim on the blank's back cone, six spokes and a boss, bored for the spline |
| cranks (2) | `gear_side.crank.body`, `pinion_side.crank.body` | a 10 mm plate, as a crank is drawn face on: a boss about the shaft bored for the spline, tapering to an eye 34 mm out, and below the shaft a fan, padded on the handle's side, its counterweight |
| grip pins (2) | `….crank.pin` | an 8 mm pin pressed into the eye from behind, its head keeping the handle on |
| handles (2) | `….crank.handle` | a sleeve that turns loose on the grip pin |

Each part is a component used twice, once per member, with one set of numbers (`mount`). Along a
shaft, every ordinate is measured from the foot of M on it, or from where the other shaft crosses
it for what stands at the post. So the two drive trains read alike, though one member's blank
stands 63° to its axis and the other's 24°. Every part but the two members is built as an exact
B-rep: each plate as the intersection of its half-spaces, everything about one axis as a
revolution, and the splines and spokes as slabs placed round the axis by a motion. The members
are their swept cuts, as in `hypoid.sv`, meshed from their material fields.

## Assembly

1. Slide each shaft into its sleeve from the post, until its head stops at the post's face.
2. Press each member onto its shaft's spline, against the sleeve's end, and glue it.
3. Slip a handle over each grip pin, press the pin into its crank's eye from the side that faces
   the member, and glue it. Then press the crank onto the shaft's end, flush, and glue it.

The head and the member's hub hold each shaft in its sleeve, with 0.15 mm of play at each end.
The fits are numbers in `mount`, and printers differ, so tune them there:

| fit | number | stated |
|---|---|---|
| journal in its sleeve, a running fit | `clearance` | 0.2 mm over the radius |
| shaft endwise, at each end of the sleeve | `play` | 0.15 mm |
| each spline in its hub, a press fit to glue | `press` | the hub's bore 0.05 mm inside the spline all round |
| handle on its pin, loose, and at each end | `loose` | 0.3 mm |
| grip pin in the crank's eye, a press fit to glue | `press` | 0.05 mm |

## The counterweights

Each crank spins without shaking or wobbling: its centre of mass is on the shaft, and the shaft is
a principal axis of its inertia. Everything that goes round with it is a plain shape at a stated
size: the plate above the shaft (the boss, the trapezoid between the tangent lines and the eye),
the grip pin, and the handle, which rides round on the pin. So `Crank` works out their first
moment about the shaft (area or volume times distance off the axis) and their moment along the
shaft, as values from `mount`.

The counterweight is the plate's own lower side, a fan of a disc about the shaft 65° either side
of straight down. A flat plate puts all its mass in one plane, and the pin and handle stand out to
one side of it. Their couple would tilt the axis of inertia, and no part of the plate's plane can
cancel it. So the fan, and only the fan, is padded on the handle's side, and the couple is the
pad's alone. The pad's depth is twice the handle group's couple over the total first moment
(28 mm). The fan's radius then follows from static balance (25 mm). The arithmetic assumes one
density, as a single print has.

`gcs-core/tests/hypoid_drive.rs` holds the balance against the built meshes, not the arithmetic.
With its pin and handle, each crank's centre of mass is 0.01 mm off its shaft. Most of that is the
press fit, a ring both meshes count, though a pressed pin holds it once. Its product of inertia
between the shaft and a direction square to it is about 6,000 mm⁵, against 2.5 million for the
plate alone. Read off the meshes, the crank's principal axis lies 0.03° off the shaft, and with its
member and shaft the rotor's 0.008°. As a flat plate with no pad, the rotor's would tilt about 9°.

## What it claims

`solventc rust/examples/hypoid_drive/drive.sv` solves the pair and the drive together (DOF 0) and
judges seventeen claims, about three minutes in all:

- the frame is 2 mm clear of each member's blank;
- each shaft and hub runs in the frame with its clearance and play (0.15 mm, the least);
- each shaft is 1 mm clear of the other member's blank, and the two shafts are 1 mm clear of each
  other;
- each handle is 0.2 mm clear of its crank and its pin (0.3 mm by design);
- each crank, turned with its shaft through a whole turn (`claim over gear_turn in (0deg,
  360deg)`), is 2 mm clear of the frame and of the other member.

The cranks turn in slabs beyond their members, and the slabs are apart. Each crank's reach,
42 mm, is less than its shaft's height over the floor's underside (75 mm for the gear's), so it
turns clear of the table.

The swept claims read solids of instances, `frame.body` and `pair.pinion_blank.material`, and
this design is what made them possible. Until it, a name inside `claim over … { … }` was not
resolved the way the same claim outside the block is (`flatten/resolve.rs`). The gate is
`gcs-core/tests/solid_claim.rs`'s `a_claim_over_a_motion_reads_the_solids_of_instances`.
`gcs-core/tests/hypoid_drive.rs` holds the drive: DOF 0, each part of its own built exactly, each
crank balanced, and, in the slow tier, every claim held with each hub touching its blank.

## Changing it

The pair's inputs head `drive.sv`, as they head `hypoid.sv`, so the drive can carry a pair of its
own. The tooth counts, module and spiral may change. The offset may not go to zero: a bevel pair's
shafts meet, and then there is no common perpendicular for the frame to stand on. The drive's own
sizes are in `mount` (shaft, sleeve, splines, hub and spokes, crank plate, grip, handle) and `sizes` (the
frame's floor). The counterweight follows the crank's sizes, and the claims say
whether a change still fits.

Print files, one STL a part (each member takes about half a minute):

```sh
make solventc
mkdir -p build/exports
for s in frame.body gear_side.part pinion_side.part gear_side.shaft pinion_side.shaft \
    gear_side.crank.body pinion_side.crank.body gear_side.crank.pin pinion_side.crank.pin \
    gear_side.crank.handle pinion_side.crank.handle; do
  build/solventc rust/examples/hypoid_drive/drive.sv --no-diagnose --solid $s \
    --stl build/exports/drive-$s.stl
done
```
