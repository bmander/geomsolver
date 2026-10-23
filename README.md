# geomsolver

A geometric constraint solver and **Solvent**, the declarative language its drawings are written
in. You declare points, lines, arcs, planes and solids, state what must be true of them, and the
solver finds the geometry, then tells you whether the drawing is under-, well- or over-constrained
and which statements are to blame. It's one dependency-free Rust core, compiled to a command-line
compiler (`solventc`) and to WebAssembly for a browser sketcher.

**[Try the sketcher in your browser →](https://bmander.github.io/geomsolver/)**

## Quickstart

To check documents without building anything, install a prebuilt `solventc` (Linux and macOS, x86-64
and arm64; Windows from the [releases page](https://github.com/bmander/geomsolver/releases)):

```sh
curl -fsSL https://raw.githubusercontent.com/bmander/geomsolver/main/scripts/install-solventc.sh | sh
solventc drawing.sv                        # ~/.local/bin/solventc, the primer beside it
```

To build everything, you need Rust and Node.

```sh
make solventc                              # build/solventc
build/solventc rust/examples/truss.sv      # parse, solve, diagnose
make wasm && cd web && npm install && npm run build && npm run serve   # http://localhost:8123/
```

`make test` runs the Rust and web suites.

## Examples

**1. One line.** The seed in `hint(…)` only says where the solve begins. Every other number is a
constraint.

```solvent
use std

in std.front {
  a := point
  b := point hint((30, 10))
  horizontal line(a, b)
  a distance(40) b
  fix((0, 0)) a
}
```

```
$ build/solventc line.sv
line.sv: solved
  2 params, 2 equations, structural rank 2; DOF 0; 1 rigid cluster(s) in the distance graph
```

**2. A rectangle with an input.** A chain (`->`) joins edges end to end. Change `w` and the
height follows.

```solvent
use std

param w := 60

in std.front {
  p0 := point
  p1 := point hint((60, 0))
  p2 := point hint((60, 30))
  p3 := point hint((0, 30))

  horizontal line(p0, p1) -> vertical line(p1, p2) ->
  horizontal line(p2, p3) -> vertical line(p3, p0) -> close

  p0 distance(w) p1
  p1 distance(w / 2) p2
  fix((0, 0)) p0
}
```

**3. A plate with a hole.** The solid comes from the solved sketch: the outline is extruded, and a
bore through it is cut away.

```solvent
unit mm
use std

param w := 60mm

in std.front {
  a := point
  b := point hint((60, 0))
  c := point hint((60, 30))
  d := point hint((0, 30))
  outline := horizontal line(a, b) -> vertical line(b, c) ->
             horizontal line(c, d) -> vertical line(d, a) -> close
  a distance(w) b
  b distance(w / 2) c
  fix((0, 0)) a

  hole := circle hint(r: 5)
  radius(8mm) hole
  a distance(w / 2, along: right) hole.center
  a distance(w / 4, along: up) hole.center
}

plate := solid(outline, depth: 10mm)
bore := solid(face(hole), through: plate)
bore cut plate
```

```
$ build/solventc plate.sv --where plate.volume --solid plate --stl plate.stl
plate.sv: solved
  9 params, 9 equations, structural rank 9; DOF 0; 3 components: DOF 0, 0, 0; …
  plate.volume = 15989.4
```

There are many more examples in [`rust/examples/`](rust/examples/), from linkages and gears to a
V-twin engine and a spiral bevel pair. You can also open them in the sketcher (**File → Open
examples…**).

## Documentation

- [Solvent primer](docs/solvent-primer.md): the language as implemented. Start here.
- [Solvent drawings](docs/solvent-drawing.md): `.svd` sheets, views and dimensions.
- [Usage](docs/usage.md): `solventc`, solid export (STL/STEP), the TypeScript binding and
  hosting the web app.
- [Architecture](docs/contract/architecture.md) and the rest of the
  [contributor contract](CLAUDE.md): the rules each subsystem is held to, one file per area.
- [Implementation status](docs/implementation-status.md): how the build program's stages were
  met, with benchmarks.
- [`solvent-spec.md`](solvent-spec.md) (normative language spec, with its
  [changelog](docs/solvent-changelog.md)) and [`gcs-solver-program.md`](gcs-solver-program.md)
  (the staged build program).
- [Bibliography](docs/bibliography.md): the methods the core implements.

## License

[MIT](LICENSE).
