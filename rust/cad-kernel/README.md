# cad-kernel

A boundary-representation (B-rep) solid-modelling kernel in Rust. It builds solids from exact
analytic and NURBS geometry, combines them with Booleans, meshes them to a stated tolerance and
writes STEP. It has no dependencies, and native and WebAssembly builds produce identical bits.

**Status: extracted, not yet shaped for general use.** The kernel was built inside
[geomsolver](https://github.com/bmander/geomsolver), a geometric constraint solver, to make
fabrication-grade STEP and STL for every solid that solver's language defines. There it is the
default for every export, held to Open CASCADE as an oracle. It now stands as its own crate so it can
be measured against other kernels on [brep-bench](https://github.com/bmander/brep-bench) and become
a common kernel others can build on. It lives in geomsolver (`rust/cad-kernel/`), which releases
it to crates.io; what changes in each release is in [CHANGELOG.md](CHANGELOG.md).

## What it does

- **Geometry:** planes, cylinders, cones, spheres, tori, extrusions and revolutions of curves,
  rational and polynomial B-spline curves and surfaces, and traced intersection curves. Surfaces are
  parameterised as STEP and OCCT do, with closed-form inverses and signed distances.
- **Topology:** oriented edge uses with p-curves, seams, degenerate poles, and a validity check.
- **Construction:** spheres, cones (frustums and cylinders too) and tori by their numbers,
  prisms (drafted too), revolutions, and canals (a ball swept along a helix). Profiles are made of
  lines, arcs and splines.
- **Booleans:** union, cut and common. Intersections between analytic surfaces are closed forms
  wherever one exists, and traced otherwise. A meeting the kernel cannot resolve is refused, never
  read as none.
- **Measures:** exact volume (by Green's theorem over each face) and area.
- **Meshing:** a constrained Delaunay mesh refined until it is within a measured sag of the surface.
- **Output:** STEP AP214 with p-curves and seam curves, and STL.
- **Errors** carry a kind (`NotManifold`, `Open`, `Refused`, …). A caller can act on the kind
  without parsing the message.
- **Determinism:** the elementary functions are the crate's own (`util::fmath`), so the same input
  gives the same bits on every target.

It is clean-room: Open CASCADE's behaviour and conventions were studied, never its code.

## On brep-bench

The first run passed 29 of 33 cases. The 4 others are refusals, none wrong: edge fillets and a
flat-ended helical sweep, which the public API does not offer yet. See the live scoreboard at
https://bmander.github.io/brep-bench/.

## Using it

```toml
[dependencies]
cad-kernel = "0.1"
```

Until the public API settles, a 0.x minor release may break it (a patch release never does), so
pin the minor version you build against.

```rust
use cad_kernel::boolean::{combined, Op};
use cad_kernel::build::{disc, prism, square};
use cad_kernel::props::volume;

// a 10 mm block with a 3 mm hole through it
let block = prism(&square([0., 0., 0.], [0., 0., 1.], 5.), 0., 10.)?;
let pin = prism(&disc([0., 0., -1.], [0., 0., 1.], 3.), 0., 12.)?;
let drilled = combined(&block, &pin, Op::Cut, 0.)?;
assert!((volume(&drilled) - (1000. - 90. * std::f64::consts::PI)).abs() < 1e-6);
```

## What comes next

- A public API designed for strangers: builders, and the recipe JSON kept as one front end among
  several.
- Edge fillets and chamfers, shelling and offsets, and a sweep of any section along any path.
- STEP import.
- The kernel's own test suite moved into the crate, out of geomsolver's.

## Licence

MIT OR Apache-2.0, at your option. The elementary functions in `util::fmath` follow the
FreeBSD/musl algorithms (permissive licences).
