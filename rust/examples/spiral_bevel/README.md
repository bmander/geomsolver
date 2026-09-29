# Spiral bevel and hypoid pair

A 24-tooth pinion and a 48-tooth gear, laid out the way a gear designer lays out a hypoid
pair: the pitch cones and the tooth trace through one design point, the mean point M, then
the blanks, the generating crown and the motions that roll it through each blank. Positions
follow from lines, circles, angles, incidence and projection; no constraint and no `param`
that feeds one computes a point with `sin` and `cos`. Every step's module carries a
`preview { … }`, so opening it in the app draws that step alone. The plan the files follow is
[docs/spiral-bevel-layout-plan.md](../../../docs/spiral-bevel-layout-plan.md).

The layout is drawn in four views through M, each a fold of the pitch plane about a line
through M, so no view needs an angle of its own. **`views.sv`**: `PitchView` is the pitch
plane P, the gear apex O at its datum origin; `FoldedView` folds a view square to it about a
hinge drawn there: G, the gear's axial view, along O → M; Q, the pinion's, along M → the
pinion apex; N, the normal section, along the trace normal C → M.

**Step 1, the requirements: `design.sv`** over **`configuration.sv`.** The configuration
states the tooth counts, the mean module, the shafts (at 90°, and the offset between them, the
length of their common perpendicular), the pressure shift and the crown's spiral angle; the
design group adds the pressure angle, the cutter radius and face width in proportion to the
mean cone distance, the depths in normal modules (addendum, dedendum, base, tip rounding,
back), the two members' generating rolls and the gear space cutter's reach. The normal
module is not stated: the trace constructs it, and every depth reads it. Three fabrication
allowances close the configuration: the normal **backlash** (0.05 mm), the **tip relief**
(0.2 mm) and the **end relief** (0.2 mm), each zero for the conjugate, sharp-edged pair. Zero offset is a
bevel pair with a common apex; the configured pair is a hypoid 25 mm off, whose 12.5-degree
shift and 25-degree spiral design out the undercut a symmetric rack develops past about 15 mm
of offset, chosen inside the generating-sweep class with margin
(`docs/spiral-bevel-layout-plan.md`, the true hypoid). `fixtures::gear` rewrites these
parameters for the suites.

**Step 2, the gear's pitch cone: `pitch/gear.sv`.** O and M lie in P; in G the right
triangle O–M–F has the two pitch radii at M for its legs, F the foot of M on the gear axis.
The pitch angle and the mean cone distance follow; nothing states them. The crown's axis
stands square to P at O, and the generator opposite M, across the axis, is where the gear's
blank is drawn.

**Step 2, the tooth trace: `pitch/trace.sv`.** The cutter centre C stands at the cutter
radius from M, with MC at 90° less the spiral angle to MO. The trace is the circle about C
through M; its heading, the tangent at M, meets the square dropped from O at H. The normal
module is constructed here: it is how far from MC a point one module from M along the
generator stands. `HypoidLayout` leaves its `normal_module` unbound, so it is one unknown of
the solve, which the trace settles and every depth reads.

**Step 2, the pinion's pitch cone: `pitch/pinion.sv`.** Solved against the gear's: its axis is
square to the gear's and the offset from it, drawn in Q from the image of its apex A, a point
of P that turns about M by an offset angle the solve answers, so P touches the pinion's cone
along AM. The equal normal pitch sizes it with no cosine written: V, where MA meets the square
from O to the heading, falls with O on one point of the heading, a virtual axis leaves V at the
bevel pinion's pitch angle (the angle at M in the gear's triangle), and the pinion's pitch
radius is one circle about M that both axes touch. The pinion's own pitch
angle and its spiral, the crown's plus the offset angle, follow. At no offset A and V are O.

**Step 3, the ends: `blank/sphere.sv`.** The face width, centred on M along the pitch
generator, and the toe and heel spheres about the apex through its ends, poles square to the
generator so each turns clear of the cones.

**Step 3, the cones: `blank/cone.sv`.** `ConeSpan` marks half and one and a half cone
distances along the pitch generator, each with a rib square to it that meets the axis;
`ConeBoundary` stands a meridian a stated offset off the generator along those ribs, toward the
axis or away, its caps square to the axis and its spine on the axis. Revolved, it is the tip,
root or back cone.

**Step 3, the end relief: `blank/ends.sv`.** Where the tip cone meets the toe sphere and the
heel sphere, `EndChamfer` chamfers the corner the end relief each way: its end on the tip cone
that far along the cone distance toward the tooth (a face span narrower by the relief at each
end), its end on the sphere that far down from the tip cone. Revolved about the axis the chamfer
is a cone band, and the ring `EndCut` takes from the blank is the chamfer's triangle with a
point beyond the corner, its other sides outside the blank. Every tooth's tip land and flanks
end on the band at both ends. It relieves the tips' toe and heel edges; the edges where a
flank meets a sphere down the tooth's depth are not relieved (a revolved face cannot follow
them, and a per-tooth chamfer is no blank face nor a generating sweep).

**Step 3, the blank: `blank/member.sv`.** `MemberLimits` draws a member's five limits in its
axial view, and its two end chamfers where the design has them; `MemberBlank` is the blank
term: the heel sphere within the tip cone, less the toe and the back. The root cone bounds
only the tooth regions the checks declare.

**Step 4, the tooth's thickness: `crown/thickness.sv`.** Two points on the mean pitch
circle a quarter of the crown's circular pitch either side of M (arcs of length π m / 4 about
O); the inner and outer trace circles about C pass through them, so the tooth is their radial
gap, and its pitch points are where they cross the trace normal beyond C.

**Step 4, the rack section: `crown/section.sv`** with **`crown/rounding.sv`.** Straight
flanks at the pressure angle, split by the shift, each a quarter of the backlash outside the
line through its pitch point that the tooth shares with its mate; a base and a tip at their
depths; two tangent tip roundings of one radius, which `TipRounding` states for every crown
section. The preview is the 24:48 crown at module 2, which **`crown.svd`** draws.

**Step 4, the pinion's generator: `crown/tooth.sv`.** The rack section in N, its pitch points
the images of the thickness's, and the cutter's axis standing at C's image square to P,
pointing out of the tooth's tip. Revolved, the flanks are cones and the roundings tori.

**Step 4, the gear's generator: `crown/mate.sv`** with **`crown/mate_section.sv`.** The
tooth's mate: two sections on the tooth's own flank lines, one tooth's width outward and one
inward, each with its own tip and roundings, revolved about the cutter's axis turned to point
the other way. The shift is stated once, on the tooth, so the two crowns are complementary by
construction (`gcs-core/tests/gear_crowns.rs`). A mate section walks its edges the other way
round from the tooth's; both name their corners by side. With backlash, each stands its flanks a
quarter of it outside the shared lines, so the two cutters overlap by half the backlash across
each, and each generated flank lies a quarter of it inside its conjugate one: the pair's normal
clearance is the backlash once one flank pair touches. At zero the sections are the conjugate
ones, their pitch points on their flanks.

**Step 4, the gear's space cutter: `crown/space.sv`** with **`crown/reach.sv`.** The stretch
between two neighbouring mate teeth, each active flank closed far from the working blank: the
outer mate within the inner one indexed a crown pitch round, closed at the reach's cap, which
stands clear of every blank point.

**Step 4, the tip relief: `crown/relief.sv`.** A semi-topping cut beside each crown. A
member's tip edges are generated by its crown's flanks near their base, an addendum from the
pitch line; `Kink` marks where, and the tip relief short of it. A chamfer leaves the flank there,
turned the relief angle (30°) so the cut widens toward the base, and rounds into a top inside
the crown; `ToothRelief` (the pinion's) and `SpaceRelief` (the gear's) close each chamfer as the
space cutter closes its flanks and bound one side by the other. Swept with the crown, it adds
only the sliver between chamfer and flank below the kink: each tooth's tip edges chamfered the
tip relief down its flanks. It is a cut of its own, not a kink in the crown's section, because a
concave corner inside one sweep trims two envelopes against each other (the generating class's
row T2); two sweeps meet in the kernel's Boolean instead. A tip relief of zero builds none of it.

**Step 5, generation: `generation.sv`.** Every roll shares the crown's angle, and each member
turns at the crown's tooth count over its own, `N_c / N` with `N_c = 2R / m`, measured off the
gear's triangle after the solve (its hypotenuse R over the gear's pitch radius, and over the
short leg `N_p m / 2` for the pinion, whose own solved cone rolls at another ratio off the
bevel), so both members stay conjugate through the common crown; the generating motions are
the crown roll relative to each member's. Indexing is one member angle; the crown's neighbour
is one crown pitch round.

**The layout: `layout.sv`.** `HypoidLayout(front, design, normal_module)` composes steps 2–5
in the four views and publishes each member's limits and motions as a group; its callers leave
`normal_module` unbound, for the trace to construct.

**The members: `members.sv`.** `GeneratedMember` is a blank less one continuous generating
sweep of its crown at every tooth index; `HypoidPair` generates the pinion from the crown tooth
and the gear from its space cutter, `ReliefCut` sweeps each member's tip relief the same way,
and `EndCut` takes its end relief's rings.

**The entry: `gears.sv`.** `pair: HypoidPair(std.front, hypoid_design)`. In the app it is the
example `spiral_bevel` (`?example=spiral_bevel`): the glass box (⌘B) shows both members
refining from their material fields. The public bodies are `pair.pinion.body` and
`pair.gear.body`; the layout is `pair.reference`.

**The checks: `pair.sv`** with **`verification.sv`.** The layout alone, `pair: HypoidLayout(…)`,
with `ReferenceFaces(pair)`: each generated flank as an envelope of its crown surface trimmed to
its member's limits, and the seams, corners, edges and faces its face loop is built from. The
independent generating-system checks (`gcs-core/tests/envelope/paired.rs`) read these; the
exported bodies do not.

## Making the pair

The fabrication files, each member's STEP (the master: exact cones, spheres and end bands, fitted
generated faces) and its STL, both held to 10 µm of the exact surface:

```sh
make solventc OCCT=1
mkdir -p build/exports
for m in pinion gear; do
  build/solventc rust/examples/spiral_bevel/gears.sv --solid pair.$m.body --tolerance \
    --step build/exports/hypoid-$m.step --stl build/exports/hypoid-$m.stl --no-diagnose
done
# every exact face within 10 µm, or exit 1
for f in pinion.step pinion.stl gear.step gear.stl; do
  build/solventc rust/examples/spiral_bevel/gears.sv --solid pair.${f%%.*}.body --tolerance \
    --measure build/exports/hypoid-$f --no-diagnose
done
# the pair from the two STLs: shafts, overlap, flank clearance, backlash, contact pattern
cargo test --manifest-path rust/Cargo.toml -p gcs-cli --test pair_check -- --ignored --nocapture
```

Nothing is written unless the shell checks and the field agreement pass. The exports take about
14 and 19 seconds (pinion, gear) on 12 cores (docs/native-speed-plan.md); measuring a file takes
longer than making it
(`--measure-samples` sets how finely). As configured (backlash 0.05 mm, tip and end relief 0.2 mm;
docs/native-hypoid-plan.md, phases 5 and 6):

| | STEP | STL | worst exact face, STEP / STL |
|---|---|---|---|
| pinion | 26.7 MB, 147 faces | 45.8 MB, 915,028 triangles | 2.75 / 3.94 µm (a fillet / the tip cone) |
| gear | 48.7 MB, 291 faces | 17.3 MB, 345,970 triangles | 1.44 / 6.16 µm (a root / a fillet) |

The blank's cones, spheres and end bands are exact in the STEP files. Read from the STLs alone,
the shafts stand at 89.9997° and 25.0001 mm; through one gear pitch the members stay at least
21 µm apart, each flank pair 21–26 µm (half the backlash by design); with one flank pair turned
into touch the other opens to 42–48 µm, the stated backlash within the files' tolerance, and the
bearing runs the whole face width, toe to heel.

The native path builds a member with swept cuts only for the generating-sweep class
(`docs/generating-sweeps.md`): admission asks each swept cut its rows and refuses with the row
and a witness, and nothing is written unless the mesh agrees with the material field. Both
members export natively, each with two sweeps (its crown's and its tip relief's); `--tolerance`
holds the STEP and the STL within 10 µm of the exact surface and `--measure` reads each file
against it (docs/native-hypoid-plan.md). `--stl-backend mesh` meshes a member from its material
field instead. Progress is reported on stderr. The stationary gear space cutter can be inspected
on its own:

```sh
build/solventc rust/examples/spiral_bevel/gears.sv --no-diagnose \
  --solid pair.reference.gear_space.body --step build/exports/solvent-gear-space.step
```

The explicit CLI selector can inspect private construction geometry; it does not grant access
from other source components. The source is a common-crown construction with a stated backlash
and tip and end relief, not a production acceptance result; see
[the roadmap](../../../docs/spiral-bevel-roadmap.md) for the generating-system evidence and the
outstanding engagement checks.
