# gcs — geometric constraint solver

A 2D geometric constraint solver — points, lines, circles and arcs under dimensional and
relational constraints — with structural diagnosis, decomposition into cached solve plans, and
robust dragging, packaged for the browser.

**[Try the sketcher in your browser →](https://bmander.github.io/geomsolver/)**

## Models and drawings

`.sv` files specify geometry, constraints, solver hints, and assertions. `.svd` files import
models and specify sheets, views, dimensions, and styles. Components group statements; they
have no designated body. Their model inputs are explicit arguments; `group` bundles related
dimensions or references to layout geometry without adding solver state. See [Solvent Drawing](docs/solvent-drawing.md).

```sh
build/solventc rust/examples/vtwin/components/piston.sv
build/solventc rust/examples/vtwin/piston.svd --output piston.svg
build/solventc drawing.svd --sheet assembly --output assembly.svg
```

The browser's **File → Open drawing folder…** opens drawings with their model dependencies.
A bare model still has an automatic editor preview. The six V-twin part sheets demonstrate
paper layout independent of geometry. Each part's component file includes a `preview { … }`
setup using standard datums; the part sheet loads that preview, while `use` omits the preview
setup.

## Solid creation examples

Open these in **File → Open examples…**, or compile their drawing sheets with
`build/solventc rust/examples/solid_flange.svd --output flange.svg`.
Each model has editable parameters near the top and a companion `.svd` with three solid views.
Profiles use geometric constraints—dimensions, alignments, symmetry, and incidences—with
coordinates confined to starting hints.

| Example | Capabilities |
| --- | --- |
| [Mounting flange](rust/examples/solid_flange.sv) | Profiles with holes, depth and offset extrusions, union, repeated through cuts |
| [V-belt pulley](rust/examples/solid_pulley.sv) | Full revolution of a stepped profile, revolved groove subtraction |
| [Hollow duct elbow](rust/examples/solid_elbow.sv) | Hollow square profile swept along a constrained circular arc |
| [Hollow reducer](rust/examples/solid_loft.sv) | Two hollow component sections lofted along a dimensioned line |
| [Pocketed tray](rust/examples/solid_tray.sv) | Named contours, blind pocket, nested bodies, repeated annular bosses |

Export any finished part with
`build/solventc rust/examples/solid_flange.sv --solid body --stl flange.stl`.

`--step flange.step` calls Open CASCADE through a native C++/C ABI bridge.
Install OCCT development files (for example, `brew install opencascade`) and build with
`make solventc OCCT=1`. Set `OCCT_ROOT` for a custom installation prefix. No Python
runtime is involved; the Rust core and browser remain dependency-free. Current support covers line/circular profiles,
extrusion, revolution, rigid motion placement, holes, additive bodies and cuts including `through:` cutters;
along-guide lofts and generating-motion sweeps are not yet connected to this host.
In a native build, `--stl` also uses OCCT. Request `--step part.step --stl part.stl`
together to construct the native solid once and export both formats. Native STL coordinates
are millimetres; its tessellator uses 0.01 mm absolute linear deflection and 0.2 rad angular
control. These settings are not a bound on the complete exported geometry's error.
Each native face must be meshed, and the encoded float32 STL must have closed, oriented,
manifold shells before any requested output is replaced. Geometric export failures preserve
both old files. Files are replaced individually after all checks; filesystem failures during
replacement are not a transaction across multiple files.
`--stl-backend mesh` explicitly selects the existing dependency-free mesh path, which is
also the default in builds without OCCT. That path retains model-unit STL coordinates.

The [indexed pattern](rust/examples/solid_indexed_pattern.sv) places a shared cutter
using an ordinary component and a named motion. Both STL and native STEP support:

```solvent
repeat teeth as i {
  solid indexed(tool, under: indexing, at: i * 360deg / teeth)
  indexed cut body
}
```

`at:` selects one pose of the motion; the source solid stays available for other instances.
The motion's ratio, phase and relative frame apply as usual. This is indexing; a continuous
generating sweep still needs its own operation.

The six-hole indexed example passes native STEP and encoded STL checks. Its legacy
`--stl-backend mesh` triangulation still leaves unpaired edges; that separate regression
is retained in `solid_motion.rs`.

The [declarative bevel blank](rust/examples/spiral_bevel/blank.sv) uses the same export path:

```sh
build/solventc rust/examples/spiral_bevel/blank.sv --solid blank.body --step blank.step
```

STEP export requires a solved model with explicit length units, checks native validity
and a STEP round trip, and preserves an existing output on failure. These are construction
checks, not an end-to-end gear accuracy certificate.

## Implementation

The whole engine is one dependency-free Rust crate ([`rust/gcs-core/`](rust/gcs-core/)) behind a
flat C ABI ([`rust/gcs-ffi/`](rust/gcs-ffi/)), built as WebAssembly for the TypeScript binding
([`web/src/core/`](web/src/core/)) and as a native shared library for anything else that speaks
C.  The binding is a thin proxy with no algorithms of its own;
[`web/src/app/`](web/src/app/) is an HTML5-canvas sketcher on top.  Stages 0–5 of
[`gcs-solver-program.md`](gcs-solver-program.md) are done — see
[`docs/implementation-status.md`](docs/implementation-status.md) for what that covers, the module
map, benchmarks and per-stage status.

## Building

Rust and Node, and nothing else.

```sh
make            # build/libgcs.dylib (the native C ABI)
make solventc   # build/solventc (the command-line compiler)
make wasm       # web/src/wasm/gcs.wasm (browser); adds the wasm32 target if needed
cd web && npm install
make test       # cargo + web suite
```

## `solventc` — checking a drawing without a browser

```sh
solventc rust/examples/*.sv          # parse, elaborate, solve, diagnose, report
solventc --json gear.sv              # the same numbers, structured
solventc --output gear.svg gear.sv   # and an SVG of the drawing
```

The sketcher's `File ▸ Export SVG` writes the same file through the same function
(`gcs_core::svg`), so the button and the command line cannot draw one drawing differently.

Exit codes are `0` (elaborated and solved), `1` (did not parse or elaborate) and `2` (did not
solve, unless `--allow-unsolved`), so a document can be checked in CI. A diagnostic carries its
span: `gear.sv:31:14: error[E040]: \`c\` is a circle, and a line is built from points`.

The report's wording is the core's — `diagnose::summary` for the per-document line, `io::describe`
for a culprit — so the CLI and the app cannot come to describe the same drawing differently.

## TypeScript quickstart

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

## Running the web app

The sketcher is live at **[bmander.github.io/geomsolver](https://bmander.github.io/geomsolver/)**,
built from `main` by [`.github/workflows/pages.yml`](.github/workflows/pages.yml).  To run it
locally:

```sh
make wasm && cd web && npm run serve    # http://localhost:8123/
```

`…/example/<slug>` opens the sketcher on a case from the library — `/example/pythagoras`,
`/example/truss:50` — by redirecting to `/?example=<slug>`, which the page reads at boot; under
a mount such as `/geomsolver/`, `/geomsolver/example/pythagoras` lands on
`/geomsolver/?example=pythagoras`.  The dev server redirects itself; on GitHub Pages (and any
static host that serves `404.html` for a missing path) `web/404.html` does it client-side.

```
```

Static files only — any static host serves `web/` after `npm run build`, and every path the page
loads is relative, so it works from a subdirectory as happily as from a domain root.

## Bibliography

The methods the core implements:

- Owen, *Algebraic solution for geometry from dimensional constraints*, SMA 1991.
- Bouma, Fudos, Hoffmann, Cai, Paige, *Geometric constraint solver*, CAD 27(6), 1995.
- Fudos & Hoffmann, *A graph-constructive approach to solving systems of geometric constraints*, ACM TOG 16(2), 1997.
- Hoffmann, Lomonosov, Sitharam, *Decomposition plans for geometric constraint systems*, I & II, J. Symbolic Computation 31, 2001.
- Pothen & Fan, *Computing the block triangular form of a sparse matrix*, ACM TOMS 16(4), 1990.
- Jacobs & Hendrickson, *An algorithm for two-dimensional rigidity percolation: the pebble game*, J. Comput. Phys. 137, 1997.
- Michelucci & Foufou, *Geometric constraint solving: the witness configuration method*, CAD 38(4), 2006.
- Durand & Hoffmann, *A systematic framework for solving geometric constraints analytically*, J. Symbolic Computation 30, 2000.
- Sitharam, Arbree, Zhou, Kohareswaran, *Solution space navigation for geometric constraint systems*, ACM TOG 25(2), 2006.
- Zou et al., *A review on geometric constraint solving*, arXiv:2202.13795, 2022.

## License

[MIT](LICENSE).
