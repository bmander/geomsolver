# Spiral bevel source examples

`configuration.sv` sets the tooth counts, the mean module and the offset angle: zero is a
bevel pair with a common apex; anything else slides the pinion around the crown by that angle,
turning its axis about the crown's normal at the mean point, and the pair is a hypoid whose
axes stand about `mean_distance * sin(offset_angle)` apart. The pinion sees the tooth trace at
`spiral_angle + offset_angle` and is enlarged by the ratio of the cosines so the tooth-count
ratio holds at the mean point. It is still one crown tooth swept through its blank at every
index; the generating roll about the turned axis is a screw, which the contact equation reads
like any other motion. `gears.sv` instantiates the matched pair using ordinary components,
named motions, revolutions and repeated cuts:

```sh
make solventc OCCT=1
build/solventc rust/examples/spiral_bevel/gears.sv --no-diagnose
```

The public material bodies are `pair.pinion.body` and `pair.gear.body`. Each subtracts
one continuous generating sweep at every tooth index. Either exports as STEP and STL:

```sh
mkdir -p build/exports
build/solventc rust/examples/spiral_bevel/gears.sv --no-diagnose --solid pair.pinion.body \
  --step build/exports/solvent-pinion.step --stl build/exports/solvent-pinion.stl
build/solventc rust/examples/spiral_bevel/gears.sv --no-diagnose --solid pair.gear.body \
  --step build/exports/solvent-gear.step --stl build/exports/solvent-gear.stl
```

Built with `make solventc OCCT=1 MANIFOLD=1`, an STL takes about eight seconds for the
pinion and twenty-five for the gear: the generating sheet comes from exact contacts on
sections of the declared cutter, the blank and the sheets are arranged as meshes, and every
cell is classified by the declared material field. STEP, or `--stl-backend occt`, takes the
kernel path instead (six and eleven minutes). Progress is reported on stderr; see the
roadmap for what each path does and does not certify. The source is a mathematical
zero-backlash, 90-degree common-crown construction, not a production acceptance result.

The stationary gear-space cutter can already be inspected through the native backend:

```sh
build/solventc rust/examples/spiral_bevel/gears.sv --no-diagnose \
  --solid pair.reference.gear_space.body \
  --step build/exports/solvent-gear-space.step \
  --stl build/exports/solvent-gear-space.stl
```

Create the output directory first if it does not exist. The explicit CLI selector can
inspect private construction geometry; it does not grant access from other source
components. `pair.sv` is the entry point the independent generating-system checks read: the
reference geometry plus the analytic faces `verification.sv` declares over it. See [the roadmap](../../../docs/spiral-bevel-roadmap.md)
for the generating-system evidence and the outstanding 0.0254 mm export-accuracy and
engagement checks.
