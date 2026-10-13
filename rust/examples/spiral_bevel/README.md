# Spiral bevel and hypoid pair

A 24-tooth pinion and a 48-tooth gear, laid out the way a gear designer lays out a hypoid pair:
the pitch cones and the tooth trace through one design point, the mean point M, then the
blanks, the generating crown and the motions that roll it through each blank. Every position
follows from lines, angles, incidence and the stated numbers; no constraint, and no value that
feeds one, computes a point with `sin` and `cos`.

**One file, `hypoid.sv`.** Its head is the pair as a designer states it — the `param`s — and
one call, `pair := HypoidPair(std.top, std.y, std.z, …)`, passing them on by name. The component
`HypoidPair` reads as the steps a designer takes, each a component of its own below it, so every
name the pair makes is under `pair` (`pair.pinion.body`, `pair.gear_cone.M`, `pair.mill.ax`).
**No point is seeded.** Every root the layout could take is chosen by a word — a bound (`foot
inside P` in the gear's cone, `foot outside P` in the pinion's, `A distance(>= 0, along: v) P`),
a sense (`sense: cw`), an ordinate's sign — and the layout solves from those words alone: the
block-triangular order constructs it a block at a time, searching each block's roots for the one
its bounds say (#217). Nothing is `hint`ed at all; three numbers are unknown — `HypoidPair`'s
formals `mn`, `rp` and `pa`, which the call leaves out — and the solve finds them.

The layout stands on the pitch plane P, `std.top`, with the gear apex O at its origin. The gear's
right triangle O–M–F and the pinion's axes stand in space square to P on their generators; each
blank is a region about its axis; the crown draws nothing in a view: its tooth and the gear's space
cutter are regions about the cutter's axis (§6.21), sets of the flank cones and the levels along
it, each region's meridian turned once into its solid. The tip relief's kinks are drawn in N, the
normal section, square to P on the trace normal C → M.

The steps read one another through interfaces (§8.1): `Gear` is what the trace, the pinion's cone
and the face mill read of the gear's cone (O, M, its generator and axis, the triangle's legs, the
crown's axis), `Trace` what the pinion's cone and the mill read of the trace (C, the normal, the
line to the heading), and `BlankDesign` the four numbers a blank is cut to. Each step is handed
the instance and may read nothing else of it.

| Step | In `hypoid.sv` | Constructs |
|---|---|---|
| 1 | the `param`s; `HypoidPair`'s proportions | the stated pair: teeth, module, shafts, shift, spiral, allowances; the cone distance R and the depths in normal modules |
| 2 | `GearCone` (`pair.gear_cone`): O, M, `foot` | the gear's pitch cone: the right triangle O–M–F, standing square to P on O → M; the crown's axis up at O |
| 2 | `ToothTrace` (`pair.trace`): C, H | the tooth trace about the cutter centre C, and the normal module, O's distance from MC over R / m |
| 2 | `PinionCone` (`pair.pinion_cone`): A, V | the pinion's cone, solved against the gear's by the shafts and the equal normal pitch |
| 3 | `Blank` (`pair.gear_blank`, `pair.pinion_blank`) | each member's blank, one region: within its pitch cone grown by the addendum (`std.Offset`) and its heel sphere, outside its toe sphere and its pitch cone shrunk by the back depth; the end relief its tip's toe and heel corners chamfered in the same region (`std.Chamfered`); its faces `tip`, `heel`, `toe` and `back` |
| 4 | `FaceMill` (`pair.mill`) | the crown tooth's pitch points a quarter pitch either side of M; the face mill, its axis `ax` through C, the two flank cones through the pitch points, and the levels the crowns' tips and bases stand at; the space cutter's reach |
| 4 | `Tooth`, `Space` (`pair.tooth`, `pair.gear_space`) | the pinion's generator, the crown tooth, a region between the flank cones and two levels, its tip's edges rounded (`std.Rounded`), each flank grown a quarter of the backlash; the gear's, its space cutter, the tooth's mate on the same cones |
| 5 | the motions, `Generated`, `Member` (`pair.pinion`, `pair.gear`) | the rolls and the generating motions; each member its blank less its crown swept through it (`through: body`: the whole roll during which the crown meets the blank, found after the solve, so no roll limit is stated) and indexed at every tooth |
| — | `Topping`, `when tip_relief > 0mm` | the tip relief: a topping beside each crown flank (a kink on the flank in N, the cone through it turned by the relief angle), cut the same way |

The normal module is stated nowhere: the trace constructs it, and every depth reads it. As
configured the pair is a hypoid 25 mm off, whose 12.5° pressure shift and 25° spiral design out
the undercut a symmetric rack develops past about 15 mm of offset; zero offset is a bevel pair with
a common apex. The allowances — 0.05 mm normal backlash, 0.2 mm tip relief and 0.2 mm end relief
— are each zero for the conjugate, sharp-edged pair, which is what the test suites record
(`fixtures::gear` sets the document's own inputs for them).

The app opens `hypoid.sv` as the example `spiral_bevel` (the members are `pair.pinion.body` and
`pair.gear.body`), and the glass box (⌘B) shows them refining from their material fields; `crown.svd`
draws the crown tooth. The layout's checks read the same file
(`gcs-core/tests/hypoid_layout.rs`, `region_blank.rs`, which holds each member's region blank to
its own faces).

A built member's faces are named, as any body's are: its blank's by the sets its region's body
uses (`pair.pinion.body`'s `material.tip`, `material.heel`, `material.toe`, `material.back`), and
each face a tooth space leaves by the crown face that made it (`indexed.outer`,
`indexed.outer_round`, `indexed.tip`; the gear's by its space cutter's, `indexed.outer_crown.inner`
and `indexed.neighbor.outer`). The generating-system checks (`gcs-core/tests/envelope/paired.rs`)
read those faces off the members built at one tooth space each, and hold each flank to the
analytic envelope of the crown face its name says — with the seams, corners and edges of its face
loop solved on the analytic faces where the built member says they meet, never on the kernel's
fit.

## Making the pair

The fabrication files, each member's STEP (the master: exact cones, spheres and end bands,
fitted generated faces) and its STL, both held to 10 µm of the exact surface:

```sh
make solventc OCCT=1
mkdir -p build/exports
for m in pinion gear; do
  build/solventc rust/examples/spiral_bevel/hypoid.sv --solid pair.$m.body --tolerance \
    --step build/exports/hypoid-$m.step --stl build/exports/hypoid-$m.stl --no-diagnose
done
# every exact face within 10 µm, or exit 1
for f in pinion.step pinion.stl gear.step gear.stl; do
  build/solventc rust/examples/spiral_bevel/hypoid.sv --solid pair.${f%%.*}.body --tolerance \
    --measure build/exports/hypoid-$f --no-diagnose
done
# the pair from the two STLs: shafts, overlap, flank clearance, backlash, contact pattern
cargo test --manifest-path rust/Cargo.toml -p gcs-cli --test pair_check -- --ignored --nocapture
```

A member with swept cuts is built only inside the generating-sweep class
(`docs/generating-sweeps.md`): admission asks each swept cut its rows and refuses with the row
and a witness, and nothing is written unless the shell checks pass and the mesh agrees with the
material field. Each member has two sweeps, its crown's and its tip relief's. The exports take
about 14 and 19 seconds (pinion, gear) on 12 cores (docs/native-speed-plan.md); measuring a file
takes longer than making it (`--measure-samples` sets how finely). `--stl-backend mesh` meshes a
member from its material field instead; progress is reported on stderr. As configured
(docs/native-hypoid-plan.md, phases 5 and 6):

| | STEP | STL | worst exact face, STEP / STL |
|---|---|---|---|
| pinion | 26.7 MB, 147 faces | 45.8 MB, 915,028 triangles | 2.75 / 3.94 µm (a fillet / the tip cone) |
| gear | 48.7 MB, 291 faces | 17.3 MB, 345,970 triangles | 1.44 / 6.16 µm (a root / a fillet) |

Read from the STLs alone, the shafts stand at 89.9997° and 25.0001 mm; through one gear pitch
the members stay at least 21 µm apart, each flank pair 21–26 µm (half the backlash by design);
with one flank pair turned into touch the other opens to 42–48 µm, the stated backlash within
the files' tolerance, and the bearing runs the whole face width, toe to heel.

The stationary gear space cutter can be inspected on its own (an explicit `--solid` may name
private construction geometry; it grants no access from other source):

```sh
build/solventc rust/examples/spiral_bevel/hypoid.sv --no-diagnose \
  --solid pair.gear_space.body --step build/exports/solvent-gear-space.step
```

## The Formate pair: the pinion generated by the gear

`hypoid.sv` cuts both members with a third body, the generating crown: each is conjugate to the
crown, so the two are conjugate to each other only approximately once the shafts are offset.
**`formate.sv`** (issue #283, step 1) takes the crown out of the pinion. It imports the layout,
the blanks and the face mill from `hypoid.sv` and cuts the members differently:

- **The gear is formed** (Gleason's Formate): every flank of its tooth spaces is one of the face
  mill's two cones, copied round the gear's axis with no roll. Its tooth is therefore the ring
  between those cones about the cutter's axis (`hypoid.sv`'s `Tooth`, the region the crown tooth
  was, now standing in the gear), and a space (`Space`) is what lies
  between one tooth's outer cone and the next tooth's inner cone, a gear pitch round. The space's
  floor is its own cutter's only; the tooth ahead bounds it by its flank alone.
- **The pinion is generated by the gear**: its blank less the gear's tooth swept under the mesh,
  `motion(gear_turn, relative_to: pinion_turn)`, the gear turning about its shaft at the tooth
  ratio (`-pinion_teeth / gear_teeth`) as the pinion turns about its own. No cone distance and no
  roll ratio is read, and no crown enters the pinion's definition (`gcs-core/tests/formate.rs`).

Each pinion flank is the envelope of a gear flank under the motion the pair runs at, so the pair
is conjugate by construction at any offset. The backlash and tip relief are carried as before:
the gear's teeth stand a quarter of the backlash inside the ring, and the pinion's cutter is the
ring grown by the other quarter. The pressure shift is still the crown pair's 12.5°; the
generated pinion takes it without undercut, admitted with a least area factor of 0.72 for the
conjugate design.

```sh
for m in pinion gear; do
  build/solventc rust/examples/spiral_bevel/formate.sv --solid pair.$m.body --tolerance \
    --step build/exports/formate-$m.step --stl build/exports/formate-$m.stl --no-diagnose
done
cargo test --manifest-path rust/Cargo.toml -p gcs-cli --test pair_check formate -- --ignored --nocapture
```

The pinion's two sweeps, the gear tooth's and its tip relief's, are admitted and built as one
sector (243 faces, 125,376 triangles, about 14 s on 4 cores). The gear has no sweep: it is its blank less 48 spaces and 48 tip reliefs, each
Boolean in turn (436 faces, 95,100 triangles, about 95 s on one core). Read from the two STLs:

| | crown pair (`hypoid.sv`) | Formate pair (`formate.sv`) |
|---|---|---|
| each flank pair over the pitch, as laid out | 22.7–26.2 / 21.7–24.4 µm | 21.6–23.9 / 20.9–22.5 µm |
| one side closed: the touching flanks over the pitch | 0–4.1 / 0–2.7 µm | 0–3.1 / 0–2.1 µm |
| one side closed, at 4 µm export: the clearance's spread / the touching flanks | 1.4 / 1.7 µm, 0–1.7 / 0–1.8 µm | 1.0 / 0.5 µm, 0–1.6 / 0–1.1 µm |
| the film (6.35 µm) at one pose, at 4 µm export | 11.8 / 14.1 mm² | 43.8 / 26.3 mm² |

At this design the crown pair's departure from conjugacy is already within a few microns, near
what the files resolve. What separates the two is the film at a single pose: the band within
6.35 µm of the gear runs the whole face, toe to heel through the mean section, for both, and is
two to four times as wide for the Formate pair, whose flanks conform along the contact line.
`pair_check` holds the Formate pair to that: each side's clearance varies over the pitch by less
than the STL's sag, closed it touches all through the pitch, and at one pose the touching band
spans at least 80% of the face through the mean section. The normal backlash it reads is half
the backlash on each side, the closing turn opening the other by the ratio of the two flanks'
levers (17.7 and 22.0 mm here): 38–46 µm for the stated 50 µm, the files' sag taken off.

This is a common-crown construction with a stated backlash and tip and end relief, not a
production acceptance result; [the roadmap](../../../docs/spiral-bevel-roadmap.md) holds the
generating-system evidence and the outstanding engagement checks.
