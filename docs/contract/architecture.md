# Architecture

Part of the [contract](../../CLAUDE.md) the code is held to. Read it when you need the map: which
layer a change belongs in. An issue number (#n) or a spec version ([0.n], see the [language
changelog](../solvent-changelog.md)) says where a rule came from.

Currently: **Stage 5 done, and Stage 7a/7b — solids**, in **one** implementation —

* **core** (`rust/gcs-core/`): the whole engine in Rust, no dependencies.  Model and constraints
  (`model.rs`, `constraints.rs`), vectorized kernels and the compile-to-plan `System` (`kernels.rs`,
  `system.rs`), our own DogLeg/LM plus dense and sparse linear algebra (`newton.rs`, `linalg.rs`,
  `sparse.rs`), structural diagnosis (`graph.rs`, `diagnose.rs`), decomposition into solve plans
  (`cgraph.rs`, `decompose.rs`), witness analysis (`witness.rs`), presentation (`style.rs`),
  drag/solution management (`solve.rs`, `homotopy.rs`), dimension callouts (`callout.rs`), dimension
  expressions (`expr.rs`), parametric curves (`curve.rs`), **solids** (`solid.rs`, `csg.rs`,
  `mesh.rs`, `hidden.rs`), the **Solvent** language (`syntax.rs`, `flatten.rs`, `program.rs`,
  `edit.rs`, `tape.rs`), JSON export (`io.rs`, `json.rs`), the rebuild walk behind deletion, copy,
  paste and a drag's part (`graft.rs`) and the reference sketches (`examples.rs`, over
  `rust/examples/`).  **The solver is usable without the language** (#409): a sketch is built
  from entities and constraint records, solved and diagnosed through the ABI with no document
  (`gcs-ffi/tests/without_a_document.rs`), and the solver's modules import neither the parser nor
  `io`: the words and types the model shares with the language (`Cmp`, `BodyWord`, `Span`,
  `num`, `snake`, `one_of`) are defined below both in `vocab.rs`, which depends on nothing, and
  `syntax` re-exports them.
* **ABI** (`rust/gcs-ffi/`): one flat C ABI over the core, built twice — a self-contained
  `wasm32-unknown-unknown` module and a native `cdylib`, where the *panic boundary* is checked
  (`gcs-ffi/tests/panic_boundary.rs`): `guard`'s `catch_unwind` only catches on native (wasm
  aborts), hence `panic = "unwind"` in release.
* **binding**: `web/src/core/` (TypeScript, WebAssembly) — *thin*: proxies over handles, buffers for
  hot-path numbers, JSON for ragged results.  It contains no algorithm and re-derives no number the
  report carries (a motion's `movingParams`).  **A `use` resolves against what the host handed over,
  then the library**: `gcs_module_set` is the browser's "beside the document" — `core/modules.ts`
  asks `gcs_program_uses`, fetches each through the app and hands it over, transitively; linking
  stays the core's (`modules::link`, in the CLI's order).  The dev server serves `rust/examples/` as
  `examples/…` and `app/remote.ts` asks it first (edits show on refresh, no wasm rebuild), else the
  compiled-in copy is read.
* **CLI** (`rust/gcs-cli/`): `solventc` parses, elaborates, solves, diagnoses and reports on a
  document, and resolves modules (`engine/parts.sv` beside the document or an ancestor, then
  `library::MODULES`). It **invents no wording**: a per-document line is `diagnose::summary`, a
  culprit is `io::describe_with` (`SourceMap` names, never `P0`; the app uses `gcs_elab_describe`),
  `--json` is `report::*_json`.
  **Where a name landed is part of the report**: `report::positions` zips `EntKind::scalar_names`
  against `Sketch::entity_params` (`hinge.x`, `base.r`); `--where NAME` filters it (a name and
  everything under it), `--json` publishes it, narrowed by the same flag. Exit codes 0/1/2 are
  `Diag::severity` and `SolveResult::success`.  The importer seam is a `Source { name, text }` list
  in, a report per source out — **the core takes text and has no filesystem** (it runs in wasm).
  `--stl` writes binary STL through `gcs_core::mesh` (`--solid` says which).  `--output` writes SVG
  through `gcs_core::svg`, as `File ▸ Export SVG` does; it **chooses a `unit`** from the page width
  (`--width`) and every constant size follows.  The *camera* is never consulted.
  **A long run says what it is doing** (#320): the core tells whoever listens on its thread
  (`progress::listen`) when work begins and ends — the elaboration, the solve and the diagnosis,
  which the CLI wraps in `progress::during`, and inside them each solid built
  (`Sketch::exact_solid`) or evaluated (`Sketch::evaluated_solid`) and each claim about solids
  judged — timed by `clock::Instant` and worded by `progress::Work`; with nobody listening the
  work is not even named.  `solventc --progress` says each on stderr when it is done (the same work
  again, a solid at each pose of a sweep, the first time only), and work still going past 5 s is
  said again, then each time it doubles, so a slow boolean is told from a hang; on a terminal,
  unasked, the stages of a second or more are said (`--no-progress` silences it).  `--timings`
  lists the solids and claims by their own time after the report, each at its statement — a
  module's solid at the document's instance of it (`Program::document_stmt`).
* **app** (`web/src/app/`): a 3D-workspace sketcher (see **The workspace** in [app.md](app.md)), the only front
  end.  The *view* is the canvas:
  `view.ts` holds the state and modules take that view as first argument (`paint`, `gesture`,
  `tools`, `dimension`, `edit`, `camera`); `underlay` is the picture traced over — **view state,
  never the document** (never saved, exported, solved or undone), handled by the ordinary select
  tool, with two rules: **the drawing outranks it**, and **only its frame is clickable, never its
  interior**.  It is not a `Primitive` and never joins `selected`, but the two selections are
  exclusive, so Delete is unambiguous.  Each direction is **one place**: `pickImage` clears
  `selected`, and `selected` is a *setter* that lets the picture go, so paste, a rubber band and the
  constraint list inherit the rule.  Pick precedence is stated once, in `gesture::whatIsAt`.
  `SketchView` keeps a one-line delegator for each verb the shell calls.  The *shell*: `shell.ts`,
  `commands` (constraints bar), `dialogs`, `lists`, `dimbox` (a dimension's number), `program` over
  `editor` (knows nothing of Solvent), `ui`, `main.ts` (only wiring). `index.html` is structure and
  `app.css` the whole of the styling.  **A phone**: a bar's buttons keep their size, and where they do not fit
  fold under one head (`ui::addTopic`/`fitBars`: every element tool and Construction under
  Elements, every constraint under Constraints; a bar wraps only past that), and bars folded to
  their heads share one row (`header.compact`); under 720px the program is a drawer above the footer, opened folded
  (`program.ts::bindDrawer`), its head carrying Compile (`applyProgram`, what ⌘↵ does); a finger
  picks within `TOUCH_PICK_PX` (`SketchView::pickPx`, set per press by `pointerType`).
