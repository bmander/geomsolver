# cad-kernel

A boundary-representation (B-rep) solid-modelling kernel in Rust. It builds solids from exact
analytic and NURBS geometry, combines them with Booleans, meshes them to a stated tolerance and
writes STEP. It has no dependencies, and native and WebAssembly builds produce identical bits.

**Status: extracted, not yet shaped for general use.** The kernel was built inside
[geomsolver](https://github.com/bmander/geomsolver), a geometric constraint solver, to make
fabrication-grade STEP and STL for every solid that solver's language defines. There it is the
default for every export, held to Open CASCADE as an oracle. It now stands as its own crate so it can
be measured against other kernels on [brep-bench](https://github.com/bmander/brep-bench) and become
a common kernel others can build on. Development happens in geomsolver (`rust/cad-kernel/`), and
this repository is synced from there. Once the public API below settles, this repository becomes
the home and geomsolver depends on it.

## What it does

- **Geometry:** planes, cylinders, cones, spheres, tori, extrusions and revolutions of curves,
  rational and polynomial B-spline curves and surfaces, and traced intersection curves. Surfaces are
  parameterised as STEP and OCCT do, with closed-form inverses and signed distances.
- **Topology:** oriented edge uses with p-curves, seams, degenerate poles, and a validity check.
- **Construction:** prisms (drafted too), revolutions, and canals (a ball swept along a helix).
  Profiles are made of lines, arcs and splines.
- **Booleans:** union, cut and common. Intersections between analytic surfaces are closed forms
  wherever one exists, and traced otherwise. A meeting the kernel cannot resolve is refused, never
  read as none.
- **Solid sweeps** (`sweep`): a tool — revolutions and prisms of lines and arcs, their unions and
  intersections, placed — swept under a rigid motion (a slide, a turn, a screw, or one seen from
  another: a rack seen from its blank) over a roll and cut from a blank, exactly (`sweep::cut`).
  Underneath: families of rigid motions with their poses, rates and interval enclosures; a tool's
  faces and their contacts; the sampled rows a sweep must pass to be built (`sweep::admit`), each
  refusal naming its row and a point; the contact tracer and sheet fitter of a generating sweep,
  and the characteristic of a screw's, a turn's or a slide's; the blank split by the sheets and
  each piece judged (`sweep::cells`).
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
cad-kernel = { git = "https://github.com/bmander/cad-kernel" }
```

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

A solid sweep: a rod grooved by a torus slid along it. Lengths are millimetres (the sheet
fitter's bars are), the roll radians of the motion's parameter (a slide moves `advance` a whole
turn of it), and the tolerance the sheet is held to millimetres (`None`: 0.01 mm).

```rust
use cad_kernel::build::{disc, prism};
use cad_kernel::sweep::cut::{swept_cut, Motion, Tool};

let rod = prism(&disc([0., 0., 0.], [0., 0., 1.], 5.), 0., 20.)?;
// a torus about the line through (11.5, 0, -9) along y: its tube's circle, radius 2, turned once
let wheel = Tool::Revolved { name: "wheel".into(), profile: disc([5.5, 0., -9.], [0., 0., 1.], 2.),
    origin: [11.5, 0., -9.], axis: [0., 1., 0.] };
let slide = Motion::Slide { direction: [0., 0., 1.], advance: 40. };
let grooved = swept_cut(&rod, &wheel, &slide, [0., std::f64::consts::TAU], Some(1e-6))?;
// a sweep outside the classes is refused: `refusal.condition` its row, `refusal.witness` a point
```

## What comes next

- A public API designed for strangers: builders, and the recipe JSON kept as one front end among
  several. The planar sweeps (a prism under a motion keeping its plane, cut over a whole period)
  through `sweep::cut` too.
- Edge fillets and chamfers, shelling and offsets, and a sweep of any section along any path.
- STEP import.
- The kernel's own test suite moved here from geomsolver.

## Licence

MIT OR Apache-2.0, at your option. The elementary functions in `util::fmath` follow the
FreeBSD/musl algorithms (permissive licences).
