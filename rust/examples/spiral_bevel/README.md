# Spiral bevel source examples

`configuration.sv` sets the tooth counts and mean module. `gears.sv` instantiates the
matched pair using ordinary components, named motions, revolutions and repeated cuts:

```sh
make solventc OCCT=1
build/solventc rust/examples/spiral_bevel/gears.sv --no-diagnose
```

The public material bodies are `pair.pinion.body` and `pair.gear.body`. Each subtracts
one continuous generating sweep at every tooth index. **Native boundary construction
for those sweeps is still pending**, so exporting either full body currently reports
an unsupported-operation error. The source is a mathematical zero-backlash, 90-degree
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
