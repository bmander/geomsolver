# Spiral bevel and hypoid pair

A 24-tooth pinion and a 48-tooth gear, laid out the way a gear designer lays out a hypoid pair:
the pitch cones and the tooth trace through one design point, the mean point M, then the
blanks, the generating crown and the motions that roll it through each blank. Every position
follows from lines, circles, angles, incidence and projection; no constraint, and no `param`
that feeds one, computes a point with `sin` and `cos`. Each step is a module whose
`preview { … }` draws that step alone when it is opened in the app, and whose header comment
says what it constructs. The plan is
[docs/spiral-bevel-layout-plan.md](../../../docs/spiral-bevel-layout-plan.md).

The layout is drawn in four views through M (`views.sv`): the pitch plane P, `std.top`, with the
gear apex O at its origin, and three views folded square to it about lines through M, so none needs
an angle of its own — G, the gear's axial view, along O → M; Q, the pinion's, along M → its apex A;
and N, the normal section, along the trace normal C → M.

| Step | Module | Constructs |
|---|---|---|
| 1 | `configuration.sv`, `design.sv` | the stated pair: teeth, module, shafts, shift, spiral, allowances; the proportions |
| 2 | `pitch/gear.sv` | the gear's pitch cone: the right triangle O–M–F in G |
| 2 | `pitch/trace.sv` | the tooth trace about the cutter centre C, and the normal module |
| 2 | `pitch/pinion.sv` | the pinion's cone, solved against the gear's by the shafts and equal normal pitch |
| 3 | `blank/sphere.sv`, `blank/cone.sv` | the face width, the toe and heel spheres, the tip, root and back cones |
| 3 | `blank/ends.sv` | the end relief: the tip's toe and heel corners chamfered |
| 3 | `blank/member.sv` | a member's limits and its blank: the heel within the tip, less the toe and the back |
| 4 | `crown/thickness.sv` | the crown tooth's pitch points, a quarter pitch either side of M |
| 4 | `crown/section.sv`, `crown/rounding.sv` | the rack section: straight flanks, base, tip and tip roundings |
| 4 | `crown/tooth.sv` | the pinion's generator: the section in N, revolved about the cutter's axis |
| 4 | `crown/mate.sv`, `crown/mate_section.sv` | the gear's generator: the tooth's mate on its own flank lines |
| 4 | `crown/reach.sv`, `crown/space.sv` | the gear's space cutter, closed clear of the blank |
| 4 | `crown/relief.sv` | the tip relief: a semi-topping cut beside each crown |
| 5 | `generation.sv` | the rolls, the generating motions and the indexing |
| — | `layout.sv` | `HypoidLayout`: steps 2–5 in the four views |
| — | `members.sv` | `HypoidPair`: each blank less its crown's sweep at every tooth, and its reliefs |

The normal module is stated nowhere: `ToothTrace` leaves it unbound and constructs it, and every
depth reads it as `trace.normal_module`. As configured the pair is a hypoid 25 mm off, whose 12.5°
pressure shift and 25° spiral design out the undercut a symmetric rack develops past about 15 mm of
offset; zero offset is a bevel pair with a common apex. The allowances — 0.05 mm normal backlash,
0.2 mm tip relief and 0.2 mm end relief — are each zero for the conjugate, sharp-edged pair, which
is what the test suites record (`fixtures::gear` rewrites the configuration for them).

Two entry points: **`gears.sv`**, the pair (`pair.pinion.body`, `pair.gear.body`; the layout
is `pair.reference`), which the app opens as the example `spiral_bevel` and the glass box (⌘B)
shows refining from its material fields; and **`pair.sv`**, the layout alone with
`verification.sv`'s analytic faces — each generated flank as an envelope of its crown surface
trimmed to its member's limits, with the seams, corners and edges of its face loop — which the
generating-system checks (`gcs-core/tests/envelope/paired.rs`) read and the exports do not.

## Making the pair

The fabrication files, each member's STEP (the master: exact cones, spheres and end bands,
fitted generated faces) and its STL, both held to 10 µm of the exact surface:

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
build/solventc rust/examples/spiral_bevel/gears.sv --no-diagnose \
  --solid pair.reference.gear_space.body --step build/exports/solvent-gear-space.step
```

This is a common-crown construction with a stated backlash and tip and end relief, not a
production acceptance result; [the roadmap](../../../docs/spiral-bevel-roadmap.md) holds the
generating-system evidence and the outstanding engagement checks.
