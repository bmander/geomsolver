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
  `num`, `snake`, `one_of`, `public_path`) are defined below both in `vocab.rs`, which depends on
  nothing, and `syntax` (or `program`, for `public_path`) re-exports them.  **The solver does
  not import the language**, and a gate holds the build to it (`tests/layering.rs`): no
  `crate::` path in a *solver* module (`model`,
  `constraints`, `kernels`, `system`, `newton`, `linalg`, `sparse`, `graph`, `cgraph`,
  `diagnose`, `decompose`, `witness`, `solve`, `homotopy`, `rng`, `fmath`, `graft`, `vocab`)
  names a *language* module (`syntax`, `flatten`, `program`, `modules`, `edit`, `style`,
  `library`, `examples`, `overview`, `callout`, `svg`, `drawing`, `report`, `io`).  Its list of
  edges still standing is empty (step 6) and stays a ratchet: a new edge fails the gate.  **Nor
  does the solver reach the language**, through any module in between: the gate follows every
  route from the solver set, through the curves, the solids, the motions and the B-rep that the
  model evaluates, and fails with the route, a `file:line` a hop, if one arrives at a language
  module.  So what the language and the solids share is defined below both: which solids are
  *objects* is the model's (`model::objects`, re-exported by `overview`), and an instance's
  public name the vocabulary's (`vocab::public_path`, re-exported by `program`).  The solver
  still reaches the solids, and **the solids are the solver's** (#409, step 6, decided): solid
  evaluation, motions and the B-rep are what the model evaluates, so they sit on the solver's
  side of the line, and the language above it.  The crate split (step 7) cuts there: the solver
  set and everything it reaches below, the language and what is built on it above.
  **Presentation rides on the model as tags and an annex** (#409, step 5): an entity and a
  constraint carry their classes as names (`vocab::Classes`, re-exported by `style`), which
  nothing that solves reads; the document's sheet is on the sketch as `Sketch::presentation`, a
  `model::Annex` the model clones and grafts and never reads, and `style` alone puts it there and
  resolves it (`Sketch::sheet`, `set_sheet`, `style_of`, `style_named` are an `impl Sketch` in
  `style.rs`).  The sketch stays the one document a handle names, so the ABI is unchanged.
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
  **`--components` classifies each component against the boundary contract** (#452,
  `boundary::classify`, worded by `boundary::text`/`json`): read through its first instance at the
  solved pose (`SourceMap::instances`, each with its path and definition; `holds`/`chosen`, whose
  `fix` and whose `ccw`), its own unknowns ranked with its inputs held and at inputs moved from
  them, its roots searched from scattered starts, and what it reads of the world said.  Advisory:
  nothing is refused on its word, and the search is labelled the heuristic it is.
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
