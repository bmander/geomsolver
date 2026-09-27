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
- `verification.sv` wrote out eleven surfaces, eleven envelopes and eleven regions, one
  statement per profile edge; it now makes them one `repeat e in … { … }` block per rack section
  (language change 3). Its seams, vertices and edges are still written one by one.
- The pinion is placed by turning its axis by `offset_angle` in the pitch plane while keeping
  the bevel pair's pitch angles, so the shaft angle comes out at 87.85°, not 90°. A true hypoid
  fixes the shaft angle and the offset distance and solves the pitch cones. *(Done: migration
  step 5.)*

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

A fold turns about a line through its parent's origin (`Basis::fold` keeps `o`); `through: M`
(language change 2, done) stands a folded view through M instead, so the world origin need not
move.

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
     gear cone at M with a common pitch plane. *Done* (migration step 5): Aₚ is solved, and the
     point the old construction made the apex is V, the apex of a *virtual* bevel pinion that
     sizes the real one.
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
   (language change 5 removes even that). Measured, the gear's is `R / r_g` off its triangle and
   the pinion's `R / (Np·m/2)`, the same triangle's hypotenuse over its short leg; at a nonzero
   offset the pinion's own cone would roll at `1 / sin γ`, which is not that ratio. `members.sv` keeps `GeneratedMember` as it is.
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

   *Steps 2–4 done.* `gears.sv` is `HypoidPair` and `pair.sv` the layout with
   `ReferenceFaces`; the old modules are gone. The step-2 gate became a regression:
   `tests/hypoid_layout.rs` holds the layout to the old pair's quantities and material,
   recorded at the twelve designs while both still elaborated
   (`tests/fixtures/hypoid_layout*.tsv`). The mate's crown surfaces take the tooth's span
   (its trace at 180°), and its flanks run base to join, so the checks read which end of a
   flank meets its round, and which way a chart normal points, off the geometry.
5. **The true hypoid**, once spatial constraints exist: state the shaft angle and E, solve the
   pitch cones, and re-record every number the configured design pins. Designs pinned at zero
   offset (`fixtures::gear::bevel`) must not change.

   *Done.* See [the true hypoid](#the-true-hypoid) below.

## The true hypoid

`configuration.sv` states `shaft_angle = 90deg` and `offset`, the length of the common
perpendicular between the shafts, in place of the offset angle; E = 0 is the bevel pair.

**What is stated** (`pitch/pinion.sv`). The gear's cone is as before: its triangle O–M–F with
legs `Ng·m/2` and `Np·m/2`, so its pitch angle is `atan(Ng/Np)` from the tooth counts. That is
the one condition beyond the radii, the shafts and the common pitch plane that the cones need
(primer 2.12 chooses the gear's pitch angle for the same reason); stated from the tooth counts,
it keeps E = 0 exactly the bevel pair. The pinion's apex A is a point of P, turned about M by an
angle ε nobody states; Q is folded square to P along M → A, and the pinion's axis is drawn in Q
from A's image, so P is the pinion cone's tangent plane along AM by construction:

```
gear_axis angle(design.shaft) axis
gear_axis distance(design.offset) axis
```

**What sizes it** is the equal normal pitch, `r_p cos(ψ + ε) = Np·m_n / 2`: the relative
velocity of the crown and the pinion at M is along the tooth trace when the pinion turns
`N_c / N_p` per crown turn. It is written as geometry. V is where the line MA crosses the
square from O to the trace's heading, so `|MV| cos(ψ + ε) = R cos ψ` (the old construction's
apex); in Q a virtual axis leaves V's image at the bevel pinion's pitch angle (tied to the angle
at M in the gear's triangle, `sin = Np / N_c`), and the pinion's pitch radius at M is one circle
about M that both axes touch:

```
V on hinge
V on foot
virtual_line angle(pinion_angle) virtual_axis      // in layout.sv
axis tangent(side: right) pitch_radius
virtual_axis tangent(side: right) pitch_radius
```

Then `r_p = |MV| Np / N_c = Np·m cos ψ / (2 cos(ψ + ε))`. Unknowns A (2), the pinion's pitch
angle and `r_p`; conditions the shaft angle, the offset and the two tangencies: DOF 0, and the
pinion's spiral `ψ + ε` follows from the trace, measured off M → A. At E = 0, A = V = O and the
two axes are one.

**Generation.** Conjugacy runs through the common crown: each member is the envelope of one
crown section turning about the crown axis, the gear at `N_c / N_g` and the pinion at
`N_c / N_p`, `N_c = 2R / m`. Both ratios are exactly the tooth-count ratio when measured off the
gear's triangle (`generation.sv`): `R / r_g` and `R / (Np·m/2)`. The pinion's own cone,
`|MA| / r_p = 1 / sin γ`, is 2.425 at the configured design against 2.236.
`tests/hypoid_layout.rs::the_true_hypoid_is_square_offset_on_one_pitch_plane_and_rolls_at_the_tooth_ratio`
holds, at five designs from 0 to 100 mm of offset: the shaft angle 90° and the common
perpendicular E to 1e-9, both apexes and M on P, each axis in the plane square to P through its
generator, each member's `r cos ψ = N m_n / 2`, the rolls `N_c / N` to 1e-12 and the crown pitch
`2π / N_c`.

**The configured design** is E = 25 mm (the turned layout's 25° was 24.56 mm), pressure shift
12.5° and spiral 25°, chosen inside the generating-sweep class with margin by
`tests/admission.rs::the_admission_grid` (least area factor J; E2 a double contact, E3 a
fold; the gear's verdict does not depend on E):

| E (mm) | shift | spiral | pinion | gear |
|---:|---:|---:|---|---|
| 20 | 10 | 25 | J 0.720 | J 0.580 |
| 22.5 | 10 | 25 | J 0.619 | J 0.580 |
| 25 | 10 | 25 | J 0.375 | J 0.580 |
| 27.5 | 10 | 25 | E3 | J 0.580 |
| 30 | 10 | 25 | E3 | J 0.580 |
| 25 | 5 | 25 | E3 | J 0.785 |
| 25 | 7.5 | 25 | E3 | J 0.709 |
| **25** | **12.5** | **25** | **J 0.694** | **J 0.338** |
| 22.5 / 27.5 | 12.5 | 25 | J 0.790 / J 0.424 | J 0.338 |
| 25 | 11 / 14 | 25 | J 0.532 / J 0.807 | J 0.503 / J 0.079 |
| 25 | 12.5 | 20 / 22.5 / 27.5 | J 0.621 / 0.660 / 0.724 | J 0.241 / 0.290 / 0.386 |
| 20 | 15 | 25 | J 0.919 | E3 |
| 30 | 15 | 25 | J 0.441 | E3 |
| 25 | 10 | 30 | J 0.339 | J 0.638 |
| 25 | 11 | 30 | J 0.547 | J 0.572 |
| 22.5 / 27.5 | 11 | 30 | J 0.765 / E3 | J 0.572 |
| 25 | 9.5 | 30 | J 0.194 (314 near tangent) | J 0.665 |
| 20 | 12.5 | 30 | J 0.908 | J 0.432 |
| 22.5 | 12.5 | 30 | J 0.861 | J 0.432 |
| 25 | 12.5 | 30 | J 0.744 | J 0.432 |
| 27.5 | 12.5 | 30 | J 0.284 (66 near tangent) | J 0.432 |
| 30 | 12.5 | 30 | E3 | J 0.432 |
| 25 | 14 | 30 | J 0.871 | J 0.213 |
| 20 | 15 | 30 | J 0.990 | E3 |
| 25 | 10 | 35 | J 0.018 (14401 near tangent) | J 0.693 |
| 25 | 12.5 | 35 | J 0.690 | J 0.522 |
| 27.5 | 12.5 | 35 | E3 | J 0.522 |
| 15 / 17.5 / 20 | 0 | 35 | J 0.182 / E3 / E3 | J 0.864 |
| 30 / 35 | 0 | 35 | E2 / E2 | J 0.864 |

Both members are admitted at every neighbour of the configured design within 2.5 mm of offset,
2.5° of shift and 5° of spiral. A symmetric rack (shift 0) leaves the class past about 15 mm.
Of the admitted designs, 25 / 12.5 / 30 has the widest margins, but its pinion's native export
(below) is refused at the fitted sheet's normal check, where its neighbours 20 and 22.5 mm,
25 / 11 / 30 and 25 / 12.5 / 25 all export; 25 / 12.5 / 25 keeps the native pinion today's pair
had, at the old spiral.

**Verified on the exports.** Measured off the field-meshed members' inertia axes, the shafts
are 89.999° apart and 25.005 mm off. Turned through one gear pitch of conjugate rotation (gear
7.5°, pinion 15°, from the exported phase, which is the best one), the two field meshes
overlap by 0.44–1.10 mm³ (the turned layout's pair: 0.30–0.92), at most 0.22 mm deep, at the
tips; the pinion exported natively against the same gear mesh overlaps by 0.006–0.071 mm³,
at most 0.021 mm deep, so what is left is the field mesh's resolution and not the generation.
At the exported pose the crown tooth and the gear's space cutter overlap by 0.48 mm³ and the
members' overlap outside both is 0.41 mm³ (the fixed old configured pair: 0.56 and 0.20; the
mate written at the tooth's own pressure angles, the bug `gear_crowns.rs` guards: 101 and 19).
Natively the configured pinion exports in 98 s with the field agreeing at every probe; the gear
is refused at the fitted sheet's normal check (83.3° at a crease, against 20°), as the turned
layout's gear was (89.3°).

**Re-recorded.** `tests/fixtures/hypoid_layout.tsv`'s configured and `hypoid6` columns and the
configured rows of `hypoid_layout_material.tsv` are now read off the layout (`record`); the
bevel pair's and the nine sizes' still hold it to the old pair to 1e-9. `fixtures::gear::hypoid6`
is 5.7 mm, the six-degree hypoid's axis offset (5.705 mm). The configured quantities, old
(turned, 25°/10°/25°) → new (true, 25 mm/12.5°/25°): shaft angle 92.148° → 90°, offset 24.557 →
25 mm, pinion pitch angle 26.565° → 24.357°, |MA| 75.667 → 82.249 mm, pinion spiral 50° →
50.117° (ε 25.117°), pinion roll ratio 2.2361 → 2.2361. At `hypoid6`: shaft 90.126° → 90°, offset 5.705 →
5.7 mm, pinion pitch angle 26.565° → 26.440°, |MA| 58.248 → 58.494 mm, spiral 41° → 40.989°.

## Language changes

Ordered by what they unlock. All six are done; the table gives the spelling that shipped.

| # | change | status | shipped spelling | where |
|---|---|---|---|---|
| 1 | spatial constraints, solved attitudes | done | `fold: beta`, `fold: along l`, `through: M`, `attitude: free`, `offset: free`; relations across views read in space; `sphere`, `cone`, `cylinder`; `against` with solved views | primer 1.13, 2.11–2.13; [spatial-constraints-plan.md](spatial-constraints-plan.md) |
| 2 | a fold through a solved point | done, as planned | `plane q(…, from: P, fold: ε, through: M)` | primer 1.13 |
| 3 | iterating over a chain's edges | done | `repeat e in CHAIN [as i] { … }`, and `cycle e in CHAIN` for a closed one | primer 1.7; `verification.sv` |
| 4 | an arc-length dimension | done, as planned | `length(L) a` | primer 1.5, 2.14; `belt_wrap.sv` |
| 5 | measurements after the solve | done for motions only | `length(l)`, `radius(c)`, `distance(a, b)`, `angle(l1, l2)` in `ratio:`, `phase:`, `advance:` | primer 1.6, 1.14; `lantern_generation.sv` |
| 6 | angle equality as a word | done, spelled differently | `l1 angle(l3, l4) l2` | primer 1.5, 2.15; `reflection.sv` |

1. **Spatial constraints with solved attitudes** — its own project, first. Spatial points and
   lines, point-to-line distance and angles between lines in space, and a plane whose attitude
   is an unknown rotor slaved by intrinsic rows, as the 2D datum's already is. Without it the
   model cannot state a 90° shaft angle and an offset distance and solve the cones, because the
   view that shows both axes in true shape has an attitude that depends on the solve. This is the
   only change that alters what the layout can express rather than how cleanly it reads.
   *Done.* A hypoid's pitch cones solve both ways — each axial view folded `along` its pitch
   generator (`gcs-core/tests/fixtures/hypoid_pitch_cones.sv`), or the cones named and
   `gc tangent(M) pc` stated (`rust/examples/hypoid_pitch_cones.sv`) — and agree to 2e-11
   (`tests/spatial_surfaces.rs`). Beyond the plan: cones, cylinders and spheres are entities with
   their own words, and the relations across views are the ordinary words read in space rather
   than new ones.
2. **`plane q(from: P, fold: ε, through: M)`**: a fold through a solved point. `project` never
   reads a plane's normal offset; the offset matters only when points are lifted to 3D, after
   the solve, as `against` placement already does. It lets every view pass through M while the
   gear apex stays at the origin. *Done* as written; where a mate bears on a view whose offset is
   `through:` a point, the mate's gap is a row of the solve.
3. **Iterating over a chain's edges**: `repeat e in rack.profile { surface s(crown, edge: e) … }`.
   Collapses most of `checks/faces.sv`. *Done*, as `repeat e in CHAIN [as i] { … }` with `cycle
   e in CHAIN` beside it (`next` is the following edge's copy). Today's `verification.sv` already
   uses it: each rack section's surfaces, envelopes and material regions are one block per
   section, so the per-edge declarations (and their names) changed there before the file split.
4. **An arc-length dimension**: `length(pi * m / 2) arc`, the circular pitch along the pitch
   circle, replacing the quarter-pitch angle and `crown_teeth`. *Done:* `length(L) a` on an
   arc, its radius times its counter-clockwise sweep (primer 1.5, `belt_wrap.sv`).
5. **Motion ratios from solved geometry**, or post-solve contexts (motions, extents) reading
   measured lengths: safe for the same reason as change 2, and removes `crown_teeth` from the
   ratios. *Done for motions*: `ratio: length(a) / distance(p, l)` and the rest (primer 1.14);
   `tests/measurements.rs` checks the measured rolls against `1 / sin(pinion_angle)` and
   `-1 / sin(gear_angle)` on the bevel pair. **Deviation:** a solid's extent and a placement's
   `at:` angle are settled at elaboration, before there is a solve to measure, so a measurement
   there is refused (E107), as it is in a `param`, a seed and a constraint's own number.
6. **Angle equality as a word**, `a equal angle b`, where a shared free variable does it now
   but reads as a trick. *Done*, **spelled differently**: `l1 angle(l3, l4) l2` — the angle from
   `l1` to `l2` equals the angle from `l3` to `l4`, directed, `sense: cw` for the mirror image —
   since a joint's `equal angle` already means two statements (primer 1.5, `reflection.sv`).

## Order of work

1. The spatial-constraints language project (change 1), with its own plan and gates. *Done.*
2. `through:` (change 2), if it is not subsumed by change 1. *Done.*
3. The rewrite, steps 1–4 of the migration, reproducing today's pair.
4. The true 90° hypoid (migration step 5). *Done.*
5. Changes 3–6 as the rewrite shows where they pay. *All four exist*; the rewrite decides where
   each is used.

## Open decisions

- Whether the world origin moves to M or stays at the gear apex. `through:` makes either
  possible.
- Whether the verification suite is ported in the same change, or keeps running against the old
  files until the new geometry has proven equivalent.
