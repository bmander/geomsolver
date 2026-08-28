# gcs — geometric constraint solver

A 2D geometric constraint solver — points, lines, circles and arcs under dimensional and
relational constraints — with structural diagnosis, decomposition into cached solve plans, and
robust dragging, packaged for the browser.

**[Try the sketcher in your browser →](https://bmander.github.io/geomsolver/)**

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
make wasm       # web/src/wasm/gcs.wasm (browser); adds the wasm32 target if needed
cd web && npm install
make test       # cargo + web suite
```

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
