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
snapshot and can render multiple views, retaining a lazily built visibility index across them.
Document layout prepares one renderer per distinct solid and shares it across all its views.
`View` specifies a world-space view plane and its page placement through `PageFrame`, plus an optional section plane. Tessellation accuracy belongs to
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
  `renderer/spatial.rs` supplies flat bounding-volume trees: a 3D tree rejects boundary faces
  outside a finite visibility ray, and a 2D tree rejects projected edges whose bounds cannot
  cross. Polygon and segment tests still determine each surviving candidate's exact result.
  Face bounds include the tolerance expansion at acute corners; slivers fall back to unbounded
  boxes. Segment bounds include endpoint tolerance. Tree traversal allocates no per-query stack.
- `renderer/document.rs` adapts document views and applies source identities and styles. Its
  `layout` function feeds both the JSON ABI used by the browser and SVG export. Generated
  dimension measurements live in `renderer/dimensions.rs`; label placement stays in `callout`.
- The browser's `DerivedDrawing` retains the displayed result and schedules refinement. That
  scheduling remains outside the synchronous Rust renderer. The C ABI and JSON format are unchanged.
- `hidden` only re-exports the old Rust entry points for compatibility.

The browser groups display tessellation into half-octave detail levels. Each level is at least
as fine as the requested pixel tolerance, with at most a sqrt(2) refinement in pixel length.
Small fit/zoom changes within the retained level reuse its drawing. A zoom that needs more detail
still defers refinement until the gesture rests. SVG/export callers bypass display levels and
request their exact output tolerance. The renderer does not yet cache projections across page
placements. There is one concrete implementation; a backend trait can follow an actual second one.

`Renderer::stats()` reports cumulative visibility-ray and candidate counts for a prepared solid.
`layout_with_stats` returns the document drawing and totals across its renderers, including the
number of face/edge candidates an exhaustive walk would have tested. These counters are separate
from the serialized drawing and make culling effectiveness testable without wall-clock gates.

## Verification and measurement

`tests/renderer.rs` exercises the renderer directly, including multiple views of one snapshot,
page placement, sections, source paths, and conservative/tight bounds. It also compares visibility
against an exhaustive reference near faces, vertices and section limits, and checks that spatial
queries skip most unrelated geometry in the throttle. The existing derived-view,
section, SVG and browser tests continue to exercise the document adapters.

Run native phase timings over the example library (documents without derived views are skipped):

```sh
cargo run --manifest-path rust/Cargo.toml --release -p gcs-core --example render_cost
```

Append `-- vtwin_throttle vtwin_cylinder` to narrow the run. Each number is the median of three
runs at a pixel length of 0.15. Evaluation starts with an empty solid cache and includes boolean
construction; preparation populates shared edge caches; projection measures document layout
using those prepared evaluations, including the first visibility-index build. Candidate counts
are printed alongside timings. Parsing, solving, serialization, and browser startup are
excluded. These timings complement, rather than replace, measurements at the browser URL.
