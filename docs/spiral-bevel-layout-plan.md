# The spiral-bevel example as a hypoid layout: plan

This plan rewrites `rust/examples/spiral_bevel/` so that its files follow the steps a gear
designer lays a hypoid pair out in, and so that the source reads as geometry rather than as
arithmetic. Three principles govern it:

- **Geometry, not trigonometry.** Positions follow from lines, circles, angles, incidence and
  projection. A `param` may do arithmetic on stated numbers (a tooth count, a module); it does
  not compute a point with `sin` and `cos`. Seeds may be rough.
- **Declarative.** A file says what is true of the layout. The requirements are stated once, as
  numbers a designer chooses, and everything else follows from constraints.
- **Small modules, with submodules.** Each component has one idea and stays under about forty
  lines. Every file carries a `preview { … }`, so opening it in the app draws that step alone,
  as the V-twin parts do.

It depends on one language project, [spatial constraints with solved attitudes](#language-changes),
which comes first; see [Order of work](#order-of-work).

## Where the example stands

About 700 lines in ten files. The steps are all there, but tangled:

- `paired_references.sv` (193 lines) is one component, `MatchedReferences`, holding the
  configuration arithmetic, every plane, the pitch-cone placement, the tooth traces, the crown
  sections and the motions.
- About thirty values are computed with `sin`, `cos`, `tan`, `atan2` and `hypot`: `center_x`,
  `pitch_x`, the turned plane's `u: (cos(offset), -sin(offset), 0)`, and the stand-off
  `mean_distance * sin(offset_angle)`. `ConeBoundary` places eight point ordinates by
  trigonometry.
- The gear's generating crown is a second, separately parameterised copy of the rack section.
  That duplication let the pressure-shift interference in (fixed in `46da5c3`; guarded by
  `gcs-core/tests/gear_crowns.rs`).
- `verification.sv` writes out eleven surfaces, eleven envelopes, eleven regions, eight seams
  and so on, one statement per profile edge.
- The pinion is placed by turning its axis by `offset_angle` in the pitch plane while keeping
  the bevel pair's pitch angles, so the shaft angle comes out at 87.85°, not 90°. A true hypoid
  fixes the shaft angle and the offset distance and solves the pitch cones.

## The idea: four views through the mean point

A hypoid's geometry passes through one design point, the mean point M on the pitch plane. Four
views through M carry the whole layout, and each is a fold of the pitch plane about a line at a
design angle, so no plane needs trigonometry:

| View | Fold of the pitch plane P | What it shows in true shape |
|---|---|---|
| **P**, the pitch plane | — | the gear apex O, the pinion apex Aₚ, the tooth trace circle and the cutter centre C |
| **G**, gear axial | along M→O | the gear axis, gear pitch cone, gear blank cones and the crown axis |
| **Q**, pinion axial | at the offset angle ε | the pinion axis, pinion pitch cone and pinion blank cones |
| **N**, normal section | along the trace normal M→C | the rack section with its pressure angles, and the cutter axis |

Every revolution then has its profile and its axis in one view: the cutter in N, the gear blank
in G, the pinion blank in Q. N is the textbook normal section, so the normal pressure angles
and the normal tooth thickness live where a gear designer expects them. `project` ties each
shared point between views, since each shares a fold line with P.

A fold today turns about a line through its parent's origin (`Basis::fold` keeps `o`), so either
M becomes the world origin or the language gains `through:` (language change 2).

## The steps as geometry

1. **Requirements** (`design.sv`). One `group`: the tooth counts, the module, the offset, the
   spiral angle ψ, the pressure angle, the pressure shift, the cutter radius, the face width,
   and the addendum and dedendum factors. Only numbers a designer states.
2. **Pitch cones** (`pitch/`).
   - *Gear cone, in G.* A right triangle O–M–F, F the foot of M on the gear axis:
     `M distance(Ng·m/2) F`, `O distance(Np·m/2) F`, a right angle at F. The pitch angle and the
     mean cone distance R follow; nothing states them.
   - *Tooth trace, in P.* C at the stated cutter radius from M, with MC at 90° − ψ to MO. The
     trace is the circle about C through M.
   - *Pinion apex, in P.* Aₚ is where the line through M at the offset angle ε from MO meets
     the line through O parallel to MC. O and Aₚ then project to one point on the trace tangent,
     so `R·cos ψ = Rₚ·cos(ψ + ε)`, the equal-normal-pitch rule, holds with no cosine written. It
     is what `pinion_distance` computes today.
   - *Pinion axis, in Q.* Through Aₚ at the pitch angle, tied to the gear side by one shared
     angle rather than a repeated number.
   - With spatial constraints (below), the true hypoid replaces the last two items: the shaft
     angle and the offset distance E are stated, and the pinion cone is solved to touch the
     gear cone at M with a common pitch plane.
3. **Blanks** (`blank/`). A cone boundary is a line parallel to the pitch generator at the
   addendum, dedendum or back offset, its ends perpendicular to the generator, its end points
   `on` the axis. Toe and heel are arcs about the apex at R ± face width/2, as the spheres are
   now. `MemberBlank` is the same `bound`/`cut` term as today.
4. **The crown** (`crown/`).
   - *Tooth thickness, in P.* Two points on the mean pitch circle a quarter crown pitch either
     side of M (`angle(90deg / crown_teeth)`), the inner and outer trace circles concentric
     about C through them. The tooth width is their radial gap, which the existing `distance`
     between concentric circles states.
   - *Rack section, in N.* Its pitch points are the images of where those circles cross MC. The
     flanks are `angle` constraints at φ ± shift, then the tip roundings.
   - *The gear's crown lies on the tooth's own flank lines* (`parallel` and a point `on`), with
     its own tips and roundings. The two crowns are complementary by construction, the shift is
     stated once, and the interference bug of `46da5c3` cannot be written.
5. **Generation** (`generation.sv`). The crown roll, the two member rolls, the relative
   generating motions and the indexing, as now. The ratios are `crown_teeth / Np` and
   `crown_teeth / Ng`, with `crown_teeth = sqrt(Np² + Ng²)`: arithmetic, not trigonometry
   (language change 5 removes even that). `members.sv` keeps `GeneratedMember` as it is.
6. **Checks** (`checks/`). Claims readable in the model: the shaft angle, the offset, and the
   pinion's spiral angle, each measured and reported. The analytic faces, written per member
   side (language change 3).
7. **Manufacture** — machine settings — stays out of scope.

## Files

```
spiral_bevel/          (the app key stays; the file pane lists the steps)
  README.md            steps 1–6, one paragraph and one file each
  design.sv            step 1: the requirements group
  views.sv             the four views through M
  pitch/gear.sv        GearCone: the O–M–F triangle, gear axis, crown axis
  pitch/trace.sv       ToothTrace: C, the trace circle, the tangent and normal at M
  pitch/pinion.sv      PinionCone: Aₚ, the pinion axis, the tied pitch angle
  blank/cone.sv        ConeBoundary, from parallel and perpendicular lines
  blank/sphere.sv      SphericalBoundary (moved as is)
  blank/member.sv      MemberBlank
  crown/thickness.sv   the quarter-pitch points and the inner and outer trace circles
  crown/section.sv     RackSection: flanks, pitch points and tip in N
  crown/rounding.sv    TipRounding: the two tangent arcs, shared by both sections
  crown/tooth.sv       CrownTooth: the pinion's generator
  crown/mate.sv        CrownMate: the gear's generator, on the tooth's flank lines
  generation.sv        motions and indexing
  members.sv           GeneratedMember (as now)
  pair.sv              HypoidPair(design): composes steps 2–5 (about 40 lines)
  gears.sv             the entry point: pair: HypoidPair(…)
  checks/faces.sv      ToothFaces(member, crown): one tooth side's faces, used four times
  checks/pair.sv       the verification entry point (today's pair.sv)
```

## Migration and verification

1. **Baseline.** Record body volumes, interference (the Boolean sweep through one pitch), the
   admission verdicts, the `gear_cells` volumes and the paired-envelope test outputs, for the
   bevel, hypoid6 and configured designs and the nine size and ratio cases.
2. **Build beside the old files, a step at a time.** Each step gets a Rust test that
   elaborates the old and new documents at all nine cases and compares the corresponding solved
   quantities to 1e-9: R, the pitch angles, the apex and axis points, C, the section width, and
   the pinion's distance and spiral angle. The constructions above are exact equivalents of
   today's formulas, so they should agree to rounding.
3. **Swap the bodies.** Byte-identical STL where the world origin is unchanged; equal volumes
   and interference where it moves.
4. **Port the consumers**, then delete the old files:
   - the tests reading `pair.reference.*` and other pair names: `envelope/paired*`,
     `gear_cells`, `native_boundary`, `cli.rs`, `functional_solids/document.rs`, `gear_crowns`;
   - `fixtures::gear` (its parameter names);
   - the app catalog entry, and `web/src/test/example-drawing.test.ts`;
   - `docs/spiral-bevel-design.md` and the example's README.
5. **The true hypoid**, once spatial constraints exist: state the shaft angle and E, solve the
   pitch cones, and re-record every number the configured design pins. Designs pinned at zero
   offset (`fixtures::gear::bevel`) must not change.

## Language changes

Ordered by what they unlock.

1. **Spatial constraints with solved attitudes** — its own project, first. Spatial points and
   lines, point-to-line distance and angles between lines in space, and a plane whose attitude
   is an unknown rotor slaved by intrinsic rows, as the 2D datum's already is. Today's model
   cannot state a 90° shaft angle and an offset distance and solve the cones, because the view
   that shows both axes in true shape has an attitude that depends on the solve. This is the
   only change that alters what the layout can express rather than how cleanly it reads.
2. **`plane q(from: P, fold: ε, through: M)`**: a fold through a solved point. `project` never
   reads a plane's normal offset; the offset matters only when points are lifted to 3D, after
   the solve, as `against` placement already does. It lets every view pass through M while the
   gear apex stays at the origin.
3. **Iterating over a chain's edges**: `repeat e in rack.profile { surface s(crown, edge: e) … }`.
   Collapses most of `checks/faces.sv`.
4. **An arc-length dimension**: `length(pi * m / 2) arc`, the circular pitch along the pitch
   circle, replacing the quarter-pitch angle and `crown_teeth`.
5. **Motion ratios from solved geometry**, or post-solve contexts (motions, extents) reading
   measured lengths: safe for the same reason as change 2, and removes `crown_teeth` from the
   ratios.
6. **Angle equality as a word**, `a equal angle b`, where a shared free variable does it now
   but reads as a trick.

## Order of work

1. The spatial-constraints language project (change 1), with its own plan and gates.
2. `through:` (change 2), if it is not subsumed by change 1.
3. The rewrite, steps 1–4 of the migration, reproducing today's pair.
4. The true 90° hypoid (migration step 5).
5. Changes 3–6 as the rewrite shows where they pay.

## Open decisions

- Whether the world origin moves to M or stays at the gear apex (`through:`).
- Whether the verification suite is ported in the same change, or keeps running against the old
  files until the new geometry has proven equivalent.
