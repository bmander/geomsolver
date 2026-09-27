# Spiral bevel and hypoid pair

A 24-tooth pinion and a 48-tooth gear, laid out the way a gear designer lays out a hypoid
pair: the pitch cones and the tooth trace through one design point, the mean point M, then
the blanks, the generating crown and the motions that roll it through each blank. Positions
follow from lines, circles, angles, incidence and projection; no constraint and no `param`
that feeds one computes a point with `sin` and `cos`. Every file carries a `preview { … }`, so
opening it in the app draws that step alone. The plan the files follow is
[docs/spiral-bevel-layout-plan.md](../../../docs/spiral-bevel-layout-plan.md).

The layout is drawn in four views through M, each a fold of the pitch plane about a line
through M, so no view needs an angle of its own. **`views.sv`**: `PitchView` is the pitch
plane P, the gear apex O at its datum origin; `FoldedView` folds a view square to it about a
hinge drawn there: G, the gear's axial view, along O → M; Q, the pinion's, along M → the
pinion apex; N, the normal section, along the trace normal C → M.

**Step 1, the requirements: `design.sv`** over **`configuration.sv`.** The configuration
states the tooth counts, the mean module, the offset angle, the pressure shift and the crown's
spiral angle; the design group adds the pressure angle, the cutter radius and face width in
proportion to the mean cone distance, the depths in normal modules (addendum, dedendum, base,
tip rounding, back), the two members' generating rolls and the gear space cutter's reach. The
normal module is the one number with a cosine in it, and the trace checks it. Zero offset is a
bevel pair with a common apex; the configured pair is a 25-degree hypoid whose 10-degree shift
and 25-degree spiral design out the undercut a symmetric rack develops past about 16 degrees
of offset (`docs/generating-sweeps-plan.md`). `fixtures::gear` rewrites these parameters for
the suites.

**Step 2, the gear's pitch cone: `pitch/gear.sv`.** O and M lie in P; in G the right
triangle O–M–F has the two pitch radii at M for its legs, F the foot of M on the gear axis.
The pitch angle and the mean cone distance follow; nothing states them. The crown's axis
stands square to P at O, and the generator opposite M, across the axis, is where the gear's
blank is drawn.

**Step 2, the tooth trace: `pitch/trace.sv`.** The cutter centre C stands at the cutter
radius from M, with MC at 90° less the spiral angle to MO. The trace is the circle about C
through M; its heading, the tangent at M, meets the square dropped from O at H. A claim checks
the normal module: a point one module from M along the generator stands that far from MC.

**Step 2, the pinion's pitch cone: `pitch/pinion.sv`.** Its apex A is where the line through
M at the offset angle from MO meets the square from O to the heading, so O and A fall on one
point of the heading and the equal normal pitch holds with no cosine written. At no offset A
is O. In Q the pinion's axis leaves A at its pitch angle, which `layout.sv` ties to the angle
at M in the gear's triangle.

**Step 3, the ends: `blank/sphere.sv`.** The face width, centred on M along the pitch
generator, and the toe and heel spheres about the apex through its ends, poles square to the
generator so each turns clear of the cones.

**Step 3, the cones: `blank/cone.sv`.** A meridian parallel to the pitch generator at a
stated offset square to it, its caps square to the axis and its spine on the axis; revolved,
it is the tip, root or back cone.

**Step 3, the blank: `blank/member.sv`.** `MemberLimits` draws a member's five limits in its
axial view; `MemberBlank` is the blank term: the heel sphere within the tip cone, less the
toe and the back. The root cone bounds only the tooth regions the checks declare.

**Step 4, the tooth's thickness: `crown/thickness.sv`.** Two points on the mean pitch
circle a quarter of the crown's circular pitch either side of M (arcs of length π m / 4 about
O); the inner and outer trace circles about C pass through them, so the tooth is their radial
gap, and its pitch points are where they cross the trace normal beyond C.

**Step 4, the rack section: `crown/section.sv`** with **`crown/rounding.sv`.** Straight
flanks through the two pitch points at the pressure angle, split by the shift; a base and a
tip at their depths; two tangent tip roundings of one radius, which `TipRounding` states for
every crown section. The preview is the 24:48 crown at module 2, which **`crown.svd`** draws.

**Step 4, the pinion's generator: `crown/tooth.sv`.** The rack section in N, its pitch points
the images of the thickness's, and the cutter's axis standing at C's image square to P,
pointing out of the tooth's tip. Revolved, the flanks are cones and the roundings tori.

**Step 4, the gear's generator: `crown/mate.sv`.** The tooth's mate: two sections on the
tooth's own flank lines, one tooth's width outward and one inward, each with its own tip and
roundings, revolved about the cutter's axis turned to point the other way. The shift is stated
once, on the tooth, so the two crowns are complementary by construction
(`gcs-core/tests/gear_crowns.rs`). The mate walks its edges the other way round and names its
corners by side.

**Step 4, the gear's space cutter: `crown/space.sv`.** The stretch between two neighbouring
mate teeth, each active flank closed far from the working blank: the outer mate within the
inner one indexed a crown pitch round, closed at a cap standing clear of every blank point.

**Step 5, generation: `generation.sv`.** Every roll shares the crown's angle, and each member
turns at the ratio its pitch cone rolls on the crown, measured off the pitch triangles after
the solve; the generating motions are the crown roll relative to each member's. Indexing is one
member angle; the crown's neighbour is one crown pitch round.

**The layout: `layout.sv`.** `HypoidLayout(front, design)` composes steps 2–5 in the four
views and publishes each member's limits and motions as a group.

**The members: `members.sv`.** `GeneratedMember` is a blank less one continuous generating
sweep of its crown at every tooth index; `HypoidPair` generates the pinion from the crown tooth
and the gear from its space cutter.

**The entry: `gears.sv`.** `pair: HypoidPair(std.front, hypoid_design)`. In the app it is the
example `spiral_bevel` (`?example=spiral_bevel`): the glass box (⌘B) shows both members
refining from their material fields. The public bodies are `pair.pinion.body` and
`pair.gear.body`; the layout is `pair.reference`.

**The checks: `pair.sv`** with **`verification.sv`.** The layout alone, `pair: HypoidLayout(…)`,
with `ReferenceFaces(pair)`: each generated flank as an envelope of its crown surface trimmed to
its member's limits, and the seams, corners, edges and faces its face loop is built from. The
independent generating-system checks (`gcs-core/tests/envelope/paired.rs`) read these; the
exported bodies do not.

## Export

```sh
make solventc OCCT=1
build/solventc rust/examples/spiral_bevel/gears.sv --no-diagnose
mkdir -p build/exports
build/solventc rust/examples/spiral_bevel/gears.sv --no-diagnose --solid pair.pinion.body \
  --step build/exports/solvent-pinion.step --stl build/exports/solvent-pinion.stl
build/solventc rust/examples/spiral_bevel/gears.sv --no-diagnose --solid pair.gear.body \
  --stl build/exports/solvent-gear.stl --stl-backend mesh
```

The native path builds a member with swept cuts only for the generating-sweep class
(`docs/generating-sweeps.md`): admission asks each swept cut its rows and refuses with the row
and a witness, and nothing is written unless the mesh agrees with the material field. The
configured pinion exports natively; the configured gear is refused there, and
`--stl-backend mesh` meshes it from its material field instead. Progress is
reported on stderr. The stationary gear space cutter can be inspected on its own:

```sh
build/solventc rust/examples/spiral_bevel/gears.sv --no-diagnose \
  --solid pair.reference.gear_space.body --step build/exports/solvent-gear-space.step
```

The explicit CLI selector can inspect private construction geometry; it does not grant access
from other source components. The source is a mathematical zero-backlash, common-crown
construction, not a production acceptance result; see
[the roadmap](../../../docs/spiral-bevel-roadmap.md) for the generating-system evidence and the
outstanding export-accuracy and engagement checks.
