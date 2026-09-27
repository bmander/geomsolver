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
  lines. Every step's module carries a `preview { … }`, so opening it in the app draws that
  step alone, as the V-twin parts do.

**Status.** Done: the example is this layout (`rust/examples/spiral_bevel/`, whose README walks
the files), a true hypoid with its shafts stated ([the true hypoid](#the-true-hypoid)), and all
six language changes it asked for exist. Open: step 6's per-side checks module (`checks/`); the
analytic faces are still written out one by one in `verification.sv`.

## Where the example stood

About 700 lines in ten files, the steps all there but tangled: one component,
`MatchedReferences` (193 lines), held the configuration arithmetic, every plane, the pitch-cone
placement, the tooth traces, the crown sections and the motions; about thirty values were
computed with `sin`, `cos`, `tan`, `atan2` and `hypot`; the gear's generating crown was a
second, separately parameterised copy of the rack section, which let the pressure-shift
interference in (fixed in `46da5c3`; guarded by `gcs-core/tests/gear_crowns.rs`); and the
pinion was placed by turning its axis by an offset angle in the pitch plane, so the shaft angle
came out at 87.85°, not 90°.

## The idea: four views through the mean point

A hypoid's geometry passes through one design point, the mean point M on the pitch plane. Four
views through M carry the whole layout, and each is a fold of the pitch plane about a line at a
design angle, so no plane needs trigonometry:

| View | Fold of the pitch plane P | What it shows in true shape |
|---|---|---|
| **P**, the pitch plane | — | the gear apex O, the pinion apex Aₚ, the tooth trace circle and the cutter centre C |
| **G**, gear axial | along M→O | the gear axis, gear pitch cone, gear blank cones and the crown axis |
| **Q**, pinion axial | along M→A, turned from MO by the offset angle ε | the pinion axis, pinion pitch cone and pinion blank cones |
| **N**, normal section | along the trace normal M→C | the rack section with its pressure angles, and the cutter axis |

Every revolution then has its profile and its axis in one view: the cutter in N, the gear blank
in G, the pinion blank in Q. N is the textbook normal section, so the normal pressure angles
and the normal tooth thickness live where a gear designer expects them. `project` ties each
shared point between views, since each shares a fold line with P.

Each view is folded square to P `along` a line drawn in P through M (`views.sv`,
`FoldedView`), which the solve places, so it stands through M with no angle of its own and the
world origin stays at the gear apex. (`through: M`, language change 2, would stand a view
folded by a stated angle through M; the layout does not need it.)

## The steps as geometry

1. **Requirements** (`design.sv` over `configuration.sv`). One `group`: the tooth counts, the
   module and the normal module, the shaft angle and offset, the spiral angle ψ, the pressure
   angle and shift, the cutter radius, the face width, the depth factors (addendum, dedendum,
   base, tip rounding, back), the generating rolls and the space cutter's reach. Only numbers a
   designer states.
2. **Pitch cones** (`pitch/`).
   - *Gear cone, in G.* A right triangle O–M–F, F the foot of M on the gear axis:
     `M distance(Ng·m/2) F`, `O distance(Np·m/2) F`, a right angle at F. The pitch angle and the
     mean cone distance R follow; nothing states them.
   - *Tooth trace, in P.* C at the stated cutter radius from M, with MC at 90° − ψ to MO. The
     trace is the circle about C through M.
   - *Pinion cone, in P and Q.* The shaft angle and the offset distance E are stated, and the
     pinion's cone is solved to touch the gear's pitch plane along its generator through M
     ([the true hypoid](#the-true-hypoid)). What sizes it is the equal normal pitch,
     `R·cos ψ = |MV|·cos(ψ + ε)`, held with no cosine written: V, where MA meets the square
     from O to the trace's heading, is the apex of a *virtual* bevel pinion whose pitch radius
     at M the real one shares.
3. **Blanks** (`blank/`). A cone boundary's meridian stands off the pitch generator by the
   addendum, dedendum or back depth, along two ribs square to it at half and one and a half
   cone distances, toward the axis or away; its caps are square to the axis and end on it. Toe
   and heel are arcs about the apex at R ± face width/2. `MemberBlank` is the heel within the
   tip cone, less the toe and the back.
4. **The crown** (`crown/`).
   - *Tooth thickness, in P.* Two points on the mean pitch circle a quarter crown pitch either
     side of M (arcs about O of `length(pi * module / 4)`), the inner and outer trace circles
     concentric about C through them. The tooth width is their radial gap.
   - *Rack section, in N.* Its pitch points are the images of where those circles cross MC. The
     flanks are `angle` constraints at φ ± shift, then the tip roundings.
   - *The gear's crown lies on the tooth's own flank lines* (`angle(180deg)` to each and a
     pitch point `on` it), with its own tips and roundings. The two crowns are complementary by
     construction, the shift is stated once, and the interference bug of `46da5c3` cannot be
     written.
5. **Generation** (`generation.sv`). The crown roll, the two member rolls, the relative
   generating motions and the indexing. Each ratio is the crown's tooth count over the
   member's, measured after the solve (language change 5): the gear's is `R / r_g` off its
   triangle and the pinion's `R / (Np·m/2)`, the same triangle's hypotenuse over its short leg;
   at a nonzero offset the pinion's own cone would roll at `1 / sin γ`, which is not that ratio.
   `members.sv` holds `GeneratedMember` and `HypoidPair`.
6. **Checks** (`checks/`, open). Claims readable in the model: the shaft angle, the offset,
   and the pinion's spiral angle, each measured and reported. The analytic faces, written per
   member side (language change 3); today they are `verification.sv`'s `ReferenceFaces`, one
   declaration per seam, corner and edge, read through `pair.sv`.
7. **Manufacture** — machine settings — stays out of scope.

## Files

```
spiral_bevel/          (the app key; the file pane lists the steps)
  README.md            the steps, one paragraph and one file each
  configuration.sv     what a designer states: teeth, module, shafts, shift, spiral
  design.sv            step 1: the requirements group over the configuration
  views.sv             PitchView and FoldedView: the views through M
  pitch/gear.sv        GearCone: the O–M–F triangle, gear axis, crown axis
  pitch/trace.sv       ToothTrace: C, the trace circle, its heading and normal at M
  pitch/pinion.sv      PinionCone: A, V, the pinion's axis solved against the gear's
  blank/sphere.sv      FaceSpan and SphericalBoundary: the toe and heel
  blank/cone.sv        ConeSpan and ConeBoundary: the tip, root and back cones
  blank/member.sv      MemberLimits and MemberBlank
  crown/thickness.sv   CrownThickness: the quarter-pitch points, inner and outer trace circles
  crown/rounding.sv    TipRounding: base, tip and roundings every crown section shares
  crown/section.sv     RackSection: flanks, pitch points and tip in N
  crown/tooth.sv       CrownTooth: the pinion's generator
  crown/mate_section.sv MateSection: a section on the tooth's flank lines
  crown/mate.sv        CrownMate: the gear's generator
  crown/reach.sv       CutterReach: the space cutter's cap
  crown/space.sv       ComplementarySpace: the gear's space cutter
  crown.svd            the crown section's sheet
  generation.sv        Generation: motions and indexing
  layout.sv            HypoidLayout(front, design): steps 2–5
  members.sv           GeneratedMember and HypoidPair
  gears.sv             the entry point: pair: HypoidPair(…)
  verification.sv      ReferenceFaces: the analytic faces (the plan's checks/faces.sv)
  pair.sv              the checks' entry point: the layout and its faces
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

   *Steps 1–4 done.* The step-2 gate became a regression: `tests/hypoid_layout.rs` holds the
   layout to the old pair's quantities and material, recorded at the twelve designs while both
   still elaborated (`tests/fixtures/hypoid_layout*.tsv`). The mate's crown surfaces take the
   tooth's span (its trace at 180°), and its flanks run base to join, so the checks read which
   end of a flank meets its round, and which way a chart normal points, off the geometry.
5. **The true hypoid**, once spatial constraints exist: state the shaft angle and E, solve the
   pitch cones, and re-record every number the configured design pins. Designs pinned at zero
   offset (`fixtures::gear::bevel`) must not change.

   *Done.* See [the true hypoid](#the-true-hypoid) below.

## The true hypoid

`configuration.sv` states `shaft_angle = 90deg` and `offset`, the length of the common
perpendicular between the shafts; E = 0 is the bevel pair.

**What is stated** (`pitch/pinion.sv`). The gear's cone is the bevel pair's: its triangle O–M–F with
legs `Ng·m/2` and `Np·m/2`, so its pitch angle is `atan(Ng/Np)` from the tooth counts. That is
the one condition beyond the radii, the shafts and the common pitch plane that the cones need
(primer 2.12 chooses the gear's pitch angle for the same reason); stated from the tooth counts,
it keeps E = 0 exactly the bevel pair. The pinion's apex A is a point of P, turned about M by an
angle ε nobody states; Q is folded square to P along M → A, and the pinion's axis is drawn in Q
from A's image, so P is the pinion cone's tangent plane along AM by construction:

```
gear.axis angle(design.shaft) axis
gear.axis distance(design.offset) axis
```

**What sizes it** is the equal normal pitch, `r_p cos(ψ + ε) = Np·m_n / 2`: the relative
velocity of the crown and the pinion at M is along the tooth trace when the pinion turns
`N_c / N_p` per crown turn. It is written as geometry. V is where the line MA crosses the
square from O to the trace's heading, so `|MV| cos(ψ + ε) = R cos ψ` (the turned layout's
apex); in Q a virtual axis leaves V's image at the bevel pinion's pitch angle (tied to the angle
at M in the gear's triangle, `sin = Np / N_c`), and the pinion's pitch radius at M is one circle
about M that both axes touch:

```
V on hinge
V on foot
gear.to_apex angle(pinion_angle, sense: cw) gear.to_foot
virtual_line angle(pinion_angle) virtual_axis
axis tangent(side: right) pitch_radius
virtual_axis tangent(side: right) pitch_radius
```

The two angles share the free `pinion_angle` because the angle-equality word (language change
6) relates lines in one view, and these are in G and Q.

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
fold; the gear's verdict does not depend on E), by spiral, shift and offset:

| spiral | shift | E (mm) | pinion | gear |
|---:|---:|---:|---|---|
| 20 | 12.5 | 25 | J 0.621 | J 0.241 |
| 22.5 | 12.5 | 25 | J 0.660 | J 0.290 |
| 25 | 5 | 25 | E3 | J 0.785 |
| 25 | 7.5 | 25 | E3 | J 0.709 |
| 25 | 10 | 20 | J 0.720 | J 0.580 |
| 25 | 10 | 22.5 | J 0.619 | J 0.580 |
| 25 | 10 | 25 | J 0.375 | J 0.580 |
| 25 | 10 | 27.5 | E3 | J 0.580 |
| 25 | 10 | 30 | E3 | J 0.580 |
| 25 | 11 | 25 | J 0.532 | J 0.503 |
| 25 | 12.5 | 22.5 | J 0.790 | J 0.338 |
| **25** | **12.5** | **25** | **J 0.694** | **J 0.338** |
| 25 | 12.5 | 27.5 | J 0.424 | J 0.338 |
| 25 | 14 | 25 | J 0.807 | J 0.079 |
| 25 | 15 | 20 | J 0.919 | E3 |
| 25 | 15 | 30 | J 0.441 | E3 |
| 27.5 | 12.5 | 25 | J 0.724 | J 0.386 |
| 30 | 9.5 | 25 | J 0.194 (314 near tangent) | J 0.665 |
| 30 | 10 | 25 | J 0.339 | J 0.638 |
| 30 | 11 | 22.5 | J 0.765 | J 0.572 |
| 30 | 11 | 25 | J 0.547 | J 0.572 |
| 30 | 11 | 27.5 | E3 | J 0.572 |
| 30 | 12.5 | 20 | J 0.908 | J 0.432 |
| 30 | 12.5 | 22.5 | J 0.861 | J 0.432 |
| 30 | 12.5 | 25 | J 0.744 | J 0.432 |
| 30 | 12.5 | 27.5 | J 0.284 (66 near tangent) | J 0.432 |
| 30 | 12.5 | 30 | E3 | J 0.432 |
| 30 | 14 | 25 | J 0.871 | J 0.213 |
| 30 | 15 | 20 | J 0.990 | E3 |
| 35 | 0 | 15 | J 0.182 | J 0.864 |
| 35 | 0 | 17.5 | E3 | J 0.864 |
| 35 | 0 | 20 | E3 | J 0.864 |
| 35 | 0 | 30 | E2 | J 0.864 |
| 35 | 0 | 35 | E2 | J 0.864 |
| 35 | 10 | 25 | J 0.018 (14401 near tangent) | J 0.693 |
| 35 | 12.5 | 25 | J 0.690 | J 0.522 |
| 35 | 12.5 | 27.5 | E3 | J 0.522 |

Both members are admitted at every neighbour of the configured design within 2.5 mm of offset,
2.5° of shift and 5° of spiral. A symmetric rack (shift 0) leaves the class past about 15 mm.
Of the admitted designs, 25 / 12.5 / 30 has the widest margins, but its pinion's native export
(below) is refused at the fitted sheet's normal check, where its neighbours 20 and 22.5 mm,
25 / 11 / 30 and 25 / 12.5 / 25 all export; 25 / 12.5 / 25 keeps a native pinion, at the turned
layout's spiral.

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

**Recorded.** `tests/fixtures/hypoid_layout.tsv`'s configured and `hypoid6` columns and the
configured rows of `hypoid_layout_material.tsv` are read off the layout (`record`); the bevel
pair's and the nine sizes' still hold it to the old pair to 1e-9. `fixtures::gear::hypoid6` is
5.7 mm, the six-degree turned hypoid's axis offset (5.705 mm). The configured quantities, turned
(25°/10°/25°) → true (25 mm/12.5°/25°): shaft angle 92.148° → 90°, offset 24.557 → 25 mm,
pinion pitch angle 26.565° → 24.357°, |MA| 75.667 → 82.249 mm, pinion spiral 50° → 50.117°
(ε 25.117°), pinion roll ratio 2.2361 → 2.2361. At `hypoid6`: shaft 90.126° → 90°, offset
5.705 → 5.7 mm, pinion pitch angle 26.565° → 26.440°, |MA| 58.248 → 58.494 mm, spiral 41° →
40.989°.

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
   It relates lines in one view, so the layout's one angle tie, between G and Q, keeps a
   shared free variable (`pitch/pinion.sv`).

## Order of work

1. The spatial-constraints language project (change 1), with its own plan and gates. *Done.*
2. `through:` (change 2), if it is not subsumed by change 1. *Done.*
3. The rewrite, steps 1–4 of the migration, reproducing the old pair. *Done.*
4. The true 90° hypoid (migration step 5). *Done.*
5. Changes 3–6 as the rewrite shows where they pay. *All four exist*; the layout uses 3
   (`verification.sv`), 4 (`crown/thickness.sv`) and 5 (`generation.sv`).

## Decisions

- The world origin stays at the gear apex; every view is folded square to the pitch plane about
  a line through M, so none needs `through:`.
- The verification suite was ported with the swap, once the equivalence gate held.
