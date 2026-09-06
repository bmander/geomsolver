# Projected drawing renderer

`renderer::Renderer` projects an immutable `EvaluatedSolid` into a vector drawing. It has no
`Sketch`, editor, browser, serialization, or style dependency. The first implementation uses
our existing polygonal hidden-line algorithms. No external renderer dependency is introduced.

```rust,ignore
let solid = sketch.evaluated_solid(index, ApproximationPolicy::View { unit })?;
let renderer = Renderer::prepare(&solid);
let bounds = renderer.bounds(frame); // conservative; no visibility work
let drawing = renderer.project(View { frame, section: None });
```

Preparation obtains the solid's cached edges. A prepared renderer borrows that immutable
snapshot and can render multiple views. `View` specifies a world-space view plane and its page
placement through `PageFrame`, plus an optional section plane. Tessellation accuracy belongs to
the evaluated solid's approximation policy; changing it requires another evaluation.

`Drawing` contains strokes, hidden/silhouette flags, source face paths, and tight bounds.
`Renderer::bounds` projects the eight local bounding-box corners and returns conservative page
bounds, including for a section. It still requires an evaluated solid and preparation; it avoids
projection and visibility work, not solid construction. Neither bounds query includes dimensions.
Empty drawings have no tight bounds.

## Ownership

- The solid kernel owns construction, booleans, material classification, and cached geometry.
- `renderer/projection.rs` owns silhouette selection, section edges, projected crossings, and
  stroke joining. `renderer/visibility.rs` owns occlusion and is also used by the 3D overview.
- `renderer/document.rs` adapts document views and applies source identities and styles. Its
  `layout` function feeds both the JSON ABI used by the browser and SVG export. Generated
  dimension measurements live in `renderer/dimensions.rs`; label placement stays in `callout`.
- The browser's `DerivedDrawing` retains the displayed result and schedules refinement. That
  scheduling remains outside the synchronous Rust renderer. The C ABI and JSON format are unchanged.
- `hidden` only re-exports the old Rust entry points for compatibility.

This extraction establishes preparation and projection boundaries. It does not yet cache
projections across page placements, change the camera-fit flow, or replace the visibility and
crossing algorithms. There is one concrete renderer; a backend trait can follow an actual second
implementation. New algorithms can be compared through the same view requests and stroke output.

## Verification and measurement

`tests/renderer.rs` exercises the renderer directly, including multiple views of one snapshot,
page placement, sections, source paths, and conservative/tight bounds. The existing derived-view,
section, SVG and browser tests continue to exercise the document adapters.

Run native phase timings over the example library (documents without derived views are skipped):

```sh
cargo run --manifest-path rust/Cargo.toml --release -p gcs-core --example render_cost
```

Append `-- vtwin_throttle vtwin_cylinder` to narrow the run. Each number is the median of three
runs at a pixel length of 0.15. Evaluation starts with an empty solid cache and includes boolean
construction; preparation populates shared edge caches; projection measures document layout
using those prepared evaluations. Parsing, solving, serialization, and browser startup are
excluded. These timings complement, rather than replace, measurements at the browser URL.
