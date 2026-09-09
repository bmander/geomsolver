# Spiral bevel source examples

`configuration.sv` sets the tooth counts and mean module. `gears.sv` instantiates the
matched pair using ordinary components, named motions, revolutions and repeated cuts:

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

The pinion takes about six minutes and the gear about eleven on a laptop; progress is
reported on stderr. The construction sections the declared cutter, fits its generating
sheet from exact contacts, indexes it by the declared motion, splits the blank in the
kernel and classifies every cell by the declared material field; see the roadmap for what
that does and does not certify. The source is a mathematical zero-backlash, 90-degree
common-crown construction, not a production acceptance result.

The stationary gear-space cutter can already be inspected through the native backend:

```sh
build/solventc rust/examples/spiral_bevel/gears.sv --no-diagnose \
  --solid pair.reference.gear_space.body \
  --step build/exports/solvent-gear-space.step \
  --stl build/exports/solvent-gear-space.stl
```

Create the output directory first if it does not exist. The explicit CLI selector can
inspect private construction geometry; it does not grant access from other source
components. `pair.sv` retains the separate reference-surface entry point used by the
independent generating-system checks. See [the roadmap](../../../docs/spiral-bevel-roadmap.md)
for the generating-system evidence and the outstanding 0.0254 mm export-accuracy and
engagement checks.
