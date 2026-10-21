# Using geomsolver

How to drive the solver from outside the language: the `solventc` compiler, solid export, the
TypeScript binding and the web app. The language itself is in the
[primer](solvent-primer.md).

## Building

You need Rust and Node, nothing else.

```sh
make            # build/libgcs.{so,dylib}: the native C ABI
make solventc   # build/solventc: the command-line compiler
make wasm       # web/src/wasm/gcs.wasm: the browser core (adds the wasm32 target if needed)
cd web && npm install
make test       # cargo and the web suite
make test-slow  # the slow tier as well
```

## `solventc`

```sh
build/solventc rust/examples/*.sv            # parse, elaborate, solve, diagnose, report
build/solventc --json rust/examples/gear.sv  # the same numbers, structured
build/solventc drawing.sv --where hinge      # the numbers under one name
build/solventc --components drawing.sv       # each component against the boundary contract
build/solventc --lint drawing.sv             # also where its text drifts from the style (advisory)
build/solventc --help                        # every option
```

The exit codes are `0` (elaborated and solved), `1` (did not parse or elaborate) and `2` (did not
solve, unless `--allow-unsolved`), so you can check a document in CI. A diagnostic carries its
span, for example `gear.sv:31:14: error[E040]: …`. The wording of the report is the core's, so the
CLI and the app describe a drawing the same way.

`--lint` adds the drift lint's warnings, `W201`–`W206`: a pun (`teeth: teeth`) in a call, a
component call standing as a statement, a component that only states relations, a path reaching
through one instance into another, an `Int` read only by `when`, and a `group` handed on unread
([the contract](contract/language.md)). They are advisory and never change the exit code.

### Models and drawings

A `.sv` file is a model: geometry, constraints, hints and claims. A `.svd` file imports models and
lays out sheets, views, dimensions and styles ([Solvent drawings](solvent-drawing.md)).

```sh
build/solventc rust/examples/vtwin/components/piston.sv
build/solventc rust/examples/vtwin/piston.svd --output piston.svg
build/solventc drawing.svd --sheet assembly --output assembly.svg
```

`File ▸ Export SVG` in the sketcher writes the same file through the same function
(`gcs_core::svg`). `File ▸ Open drawing folder…` opens drawings together with the models they
depend on. A model opened on its own gets an automatic preview: each V-twin part's component file
has a `preview { … }` block that sets it up on the standard datums, and a `use` of that file leaves
the preview out.

## Solids and export

Each of these models has its editable parameters near the top and a companion `.svd` with three
views. Open them from `File ▸ Open examples…`, or render a sheet with
`build/solventc rust/examples/solid_flange.svd --output flange.svg`.

| Example | Shows |
| --- | --- |
| [Mounting flange](../rust/examples/solid_flange.sv) | a revolved stepped section, a bolt pattern of through cuts |
| [V-belt pulley](../rust/examples/solid_pulley.sv) | a full revolution, a revolved groove cut away |
| [Hollow duct elbow](../rust/examples/solid_elbow.sv) | a hollow section carried round a constrained arc |
| [Hollow reducer](../rust/examples/solid_loft.sv) | a hollow section run along a line, its size read from the station |
| [Pocketed tray](../rust/examples/solid_tray.sv) | named contours, a blind pocket, nested bodies, repeated bosses |
| [Indexed pattern](../rust/examples/solid_indexed_pattern.sv) | one cutter placed at the poses of a frame's turn |
| [Generating sweep](../rust/examples/solid_generating_sweep.sv) | material removed over a continuous motion |

```sh
build/solventc rust/examples/solid_flange.sv --solid flange --stl flange.stl
build/solventc rust/examples/solid_flange.sv --solid flange --step flange.step --stl flange.stl
build/solventc part.sv --step part.step --tolerance 0.01mm   # within a stated fabrication bar
build/solventc part.sv --gltf part.glb                       # every face a named node
```

STEP and STL are built by the core's own B-rep kernel. The same code runs in the browser, so native
and browser exports are identical byte for byte. Building with `make solventc OCCT=1` (set
`OCCT_ROOT` for a custom prefix) adds Open CASCADE as an oracle: `--kernel occt`, and
`--verify-step full` to read the written STEP back. An export is checked before it replaces
anything: a failed check leaves the old files in place. Export needs a solved model; a document
with no `unit` line is in millimetres.

A placed solid takes one pose of a motion, and a swept solid takes all of them over an interval:

```solvent
repeat teeth as i {
  indexed := tool at(under: indexing) i * 360deg / teeth
  indexed cut body
}

removal := sweep(tool over t in (-30deg, 30deg), under: generating)
removal cut body
```

The swept bodies that can be exported exactly are covered in
[generating sweeps](generating-sweeps.md). The spiral bevel pair
([`rust/examples/spiral_bevel/`](../rust/examples/spiral_bevel/), with its own README) is the
largest case.

## TypeScript binding

The binding in [`web/src/core/`](../web/src/core/) is a thin proxy over the WebAssembly core.

```ts
import { initCore } from './core/wasm.js';
import { Sketch } from './core/model.js';
import { solve } from './core/system.js';
import * as C from './core/constraints.js';

await initCore();                       // loads gcs.wasm once
const sk = new Sketch();
const p = sk.point(0, 0), q = sk.point(12, 0);
sk.add(new C.Distance(p, q, 10));
solve(sk);
```

## The web app

The sketcher is live at [bmander.github.io/geomsolver](https://bmander.github.io/geomsolver/),
built from `main` by [`.github/workflows/pages.yml`](../.github/workflows/pages.yml). To run it
locally:

```sh
make wasm && cd web && npm install && npm run build && npm run serve   # http://localhost:8123/
```

`…/example/<slug>` opens a case from the library (`/example/pythagoras`, `/example/truss:50`) by
redirecting to `?example=<slug>`. The dev server does this itself, and on GitHub Pages (or any host
that serves `404.html` for a missing path) `web/404.html` does it in the page. The app is static
files with relative paths, so any static host can serve `web/` after `npm run build`, from a
subdirectory as well as from a domain root.
