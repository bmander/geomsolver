# geomsolver

A geometric constraint solver, and **Solvent**, the language a drawing in it is written as.

**Start here.** Asked to *draw* something — write or edit a sketch, add constraints, work out why
a figure will not solve — read [`docs/solvent-primer.md`](docs/solvent-primer.md) first.  It is
the language as the implementation actually accepts it, with every example run through the solver
and its degrees of freedom quoted from what the solver said.  `solvent-spec.md` is the normative
specification and is the authority on what the language *should* be, but it specifies constructs
that do not parse yet (`hint` as a statement, `path`), so write from the primer and reach
for the spec when the question is what a rule ought to be.  Asked to work on the *solver* —
kernels, diagnosis, decomposition, the bindings, the app — the rest of this file is the contract,
and `gcs-solver-program.md` is the staged program it is built to.

Currently: **Stage 5 done, and Stage 7a/7b — solids** (issue #48, items 9 and 10), in **one**
implementation —

* **core** (`rust/gcs-core/`): the whole engine in Rust, no dependencies.  Model and constraints
  (`model.rs`, `constraints.rs`), vectorized kernels and the compile-to-plan `System`
  (`kernels.rs`, `system.rs`), our own DogLeg/LM plus the dense and sparse linear algebra
  (`newton.rs`, `linalg.rs`, `sparse.rs`), structural diagnosis (`graph.rs`, `diagnose.rs`),
  decomposition into cached solve plans (`cgraph.rs`, `decompose.rs`), witness analysis
  (`witness.rs`), presentation (`style.rs`), drag/solution management (`solve.rs`,
  `homotopy.rs`), dimension callouts (`callout.rs`), dimension expressions (`expr.rs`),
  parametric curves (`curve.rs`), **solids** — the term, its evaluation and what is drawn of it
  (`solid.rs`, `csg.rs`, `mesh.rs`, `hidden.rs`) — the **Solvent** language the document is
  written in
  (`syntax.rs`, `flatten.rs`, `program.rs`, `edit.rs`, `tape.rs`), JSON export
  (`io.rs`, `json.rs`) and the reference sketches
  (`examples.rs`, over the documents in `rust/examples/`).
* **ABI** (`rust/gcs-ffi/`): one flat C ABI over the core, built twice — a self-contained
  `wasm32-unknown-unknown` module for the browser and a native `cdylib` for anything else that
  speaks C.  The native build is also where the *panic boundary* is checked
  (`gcs-ffi/tests/panic_boundary.rs`): `guard`'s `catch_unwind` only ever catches on native,
  since `wasm32-unknown-unknown` aborts whatever the profile says, which is why the release
  profile carries `panic = "unwind"`.
* **binding**: `web/src/core/` (TypeScript, WebAssembly) — the only one.  It is *thin*: proxies
  over handles, buffers for hot-path numbers, JSON for ragged results.  It contains no
  algorithm — and re-derives no number the report already carries (a motion's `movingParams` is
  the core's reading of its own velocities, not a threshold the binding picks), since two copies
  of a rule are two rules the moment one of them is edited.  **A `use` resolves against what
  the host handed over, then the library**: the browser has no filesystem, so `gcs_module_set`
  is its "beside the document" — `core/modules.ts` asks `gcs_program_uses` what a text uses,
  fetches each through a function the app supplies, hands it over, and follows the fetched
  texts' uses too; the linking itself stays the core's (`modules::link`, in the CLI's order).
  The dev server serves `rust/examples/` as `examples/…` and `app/remote.ts` asks it first, so
  with `npm run serve` up a case is the file on disk and an edit shows on refresh with no wasm
  rebuilt; without a server nothing fetches and the compiled-in copy is read as before.
* **CLI** (`rust/gcs-cli/`): `solventc`, which parses, elaborates, solves, diagnoses and reports
  on a document from a terminal — the first way to check a drawing without a browser, and where
  module resolution lives: `use engine.parts` is `engine/parts.sv` beside the document, then the
  library compiled into the core (`library::MODULES`).  It **invents no wording**: a per-document
  line is `diagnose::summary`, a culprit is `io::describe_with` (the core's wording over the
  `SourceMap`'s names — `corner distance(60) along`, never `P0`; the app reaches the same through
  `gcs_elab_describe`), `--json` is `report::*_json`, so it and the app cannot come to describe
  one drawing differently.  **Where a name landed is part of the report** (issue #48, item 3):
  `report::positions` zips `EntKind::scalar_names` against `Sketch::entity_params` — one list
  twice, as names and as parameters — so `hinge.x`, `base.r` and `view.angle` are the source map's
  names against the sketch's numbers and nothing there learns what a circle is made of.  One walk,
  two readers: `--where NAME` filters it for a terminal (a name matches its own numbers and
  everything written under it) and `--json` publishes the whole table, narrowed by the same flag.
  Exit codes 0/1/2 are what
  `Diag::severity` and `SolveResult::success` already say, given a process to say it to.  The
  seam an importer will need is there already — a `Source { name, text }` list in, a report per
  source out — because **the core takes text and has no filesystem**: it runs in wasm and must
  not learn how to open a file, so the thing with a working directory is this binary.
  `--stl` writes a solid as binary STL through `gcs_core::mesh` — the one output of a drawing
  that is not a picture, and the reason a printer can be given a part at all; `--solid` says
  which, and a document with one part need not.
  `--output` writes an SVG through `gcs_core::svg`, in the core for the reason callout layout
  is — and the app's `File ▸ Export SVG` calls the same function, so the button and the command
  line are one picture of one drawing and not two.  An SVG has no screen, so the export
  **chooses a `unit`** from the page width (`--width`, or the canvas's own) and every constant
  size follows: callout text and arrowheads, a curve's flatness, a sheet's dashes and weights.
  The *camera* is deliberately not consulted — an export is of the drawing, not of the view, so
  the file is the same whatever the pan and zoom were.
* **app** (`web/src/app/`): an HTML5-canvas sketcher, the only front end.  Two halves, each
  a handful of modules rather than one slab.  The *view* is the canvas: `view.ts` holds the
  state — the camera, selection, tool, plan, diagnosis — and the modules beside it take that
  view as their first argument and do the work (`paint`, `gesture`, `tools`, `dimension`,
  `edit`), with `camera` the one that holds where the drawing sits on the canvas and
  `underlay` the picture traced over — **view state, never the document**: a photograph is
  scaffolding for the person drawing, so nothing about it is saved, exported, solved or
  undone.  It is nonetheless handled the way everything else on the canvas is — clicked,
  dragged, deleted, under the ordinary select tool and with no mode of its own — and two
  rules keep that from spoiling the tracing it exists for: **the drawing outranks it** (the
  geometry is offered a press first), and **only its frame is clickable, never its
  interior** — nothing in this drawing is picked by an area, a circle being picked by its rim
  and not by the disc inside it, so the edge is where you take hold of it and the middle is
  where you draw.  Selected, the whole of it drags, so placing it is not a fight with a
  two-pixel border.  It is not a `Primitive` and so never joins `selected` — it would have to
  be answered for at every seam that reads one — but the two selections are exclusive, which
  is what leaves Delete unambiguous.  Each direction of that is **one place**: `pickImage`
  clears `selected`, and `selected` is a *setter* that lets the picture go, so paste, a rubber
  band and the constraint list inherit the rule rather than each remembering it — enforced at
  the call sites instead, the one that forgets fails silently, by deleting the photograph
  instead of what was just pasted.  Which pick outranks which is likewise stated once, in
  `gesture::whatIsAt`, since a press and the cursor that promises what it will do must not walk
  two copies of the order.  Its placement is a similarity in world coordinates and
  reaches the screen only through `camera`, so it pans and zooms with the drawing and this
  file writes no minus sign in front of a y either;
  `SketchView` keeps a one-line delegator for each verb the shell calls, so a caller holds one
  object.  The *shell* is the page around it: `shell.ts` (the elements, the view, the focused
  constraint, and where the core is started), `commands` (the constraints bar), `dialogs` (what
  the menus open), `lists` (the constraints window, the banner and the status line), `dimbox`
  (a dimension's number, edited on the drawing), `program` (the source, beside the drawing it
  makes) over `editor` (the code box it is typed in, which knows nothing of Solvent),
  `ui` (dialogs and bar widgets), and `main.ts`, which is only wiring.  `index.html` is structure and `app.css` is the whole of the styling.

Commands:
`make` (native `build/libgcs.dylib`), `make solventc` (`build/solventc`),
`make wasm` (`web/src/wasm/gcs.wasm`),
`make test` (both released artefacts, cargo and the web suite),
`cargo test --manifest-path rust/Cargo.toml` (**never `--release`**: the suite runs under
`[profile.test]` — optimised, no LTO, no debuginfo — and `rust/Cargo.toml` says what each of
the three would cost; one file of the core suite is filtered as `cargo test chain::`),
`cd web && npm test`, `make bench` (the native `bench` binary and `npm run bench`, meant to be
read side by side), `cd web && npm run serve`.

`npm run build` is `tsc` (module per file, which is what `node --test` and the test suite run
against) then esbuild, which rolls `dist/app/main.js` and everything it reaches into one
`dist/app/bundle.js` — what the page loads, so splitting a module further costs the browser
nothing.  The bundle stands in for the entry module, so it is written *beside* it: `core/wasm.ts`
finds the core at `../wasm/gcs.wasm` relative to `import.meta.url`, which under bundling is the
bundle's own URL, and a bundle written anywhere else would look for it somewhere else.  The
node-only fallback imports in `wasm.ts` are left external; a browser never evaluates them.
**three.js is the project's one runtime dependency** and the bundle carries it: 208 kB became
1.3 MB, which is what a depth buffer for the glass box costs and is paid on the sheet too, since
one bundle is what the page loads.  It is bundled rather than fetched from a CDN for the reason
the wasm is beside the bundle — the app opens from a file as readily as from a server.

Conventions:
- **Every seed is written in one `hint(…)` clause, and nothing else is** (Solvent §4.3, §6.4):
  `point p hint(x: 0, y: 0)`, `circle c(center: o) hint(r: 25)`,
  `point_on_spline(p, s) hint(t: 0.4)`.  Keys in any order, an omitted coordinate is 0 — an
  omitted *radius* is computed from the geometry (`UNSEEDED_RADIUS` where it gives none), since
  0 is a stationary point of every on-circle row in `r` (#45.6) — and the
  clause joins the trailing-clause loop beside `knots` and `class`.  **The brackets after
  the name are what the thing is made of; the `hint(…)` after them is where the solve begins** —
  which is what `circle c(center: o, r: 25)` got wrong, putting a number the solver will move
  inside the same brackets as the structure it may not.  §4.3's rule is then lexical and exact:
  *a number inside a `hint(…)` is a seed, and every other number is not* — which `=` never was,
  since `param w = 100` is written with one and is not a seed.  The four retired spellings
  (`at (0, 0)`, `hint at (0, 0)`, a scalar in a constructor arg, and `hint at REF [bearing (…)]`)
  do **not** parse, and each errors saying where the number belongs.
  **A place is two keys of the same clause** (issue #47, item 2): `point b hint(at: orbit,
  bearing: u + f.angle)`, `point p hint(at: pin)` — a seed given as geometry rather than as a
  pair of numbers, on the sheet and inside a traced component alike.  `hint_body` reads `at:` as
  a reference (`Hint::place`) wherever the clause stands, and the declaration's trailer loop is
  the one table that takes it, into `Decl::seed_at` (an `AtRef`: the place and its bearing
  text); a clause with `at:` and a scalar, or `bearing:` without `at:`, is refused at the key.
  Both lower to the same tapes the coordinate spelling would, so nothing below the parser knows
  there are two — and there is no second grammar, no `bearing` keyword and no second print arm.
  One exception, and it is the rule rather than against it.  A **pin** stays in the argument
  list — `point_on_spline(p, s, t == 0.4)` — because `hint` marks what a solve revises and a pin
  is precisely what it does not; it is a stated number, beside every other stated number.
  **What `hint` marks is that a solve revises the number, not that the number is seed-class** —
  the two are different sets, and that is why a **callout placement keeps its bare `at`**.  A
  placement is every bit as inert (delete it and the drawing is the same), but nothing in the
  solve path ever writes one: `callout::drag` and `callout::reset` are a person acting, and the
  layout derives it until they do.  A coordinate seed is an input the solver overwrites; a
  placement is a preference it never touches.  Ask §4.3, not the keyword, for what may be
  deleted without changing the drawing.
- **A seed may read geometry, and reads its seed** (Solvent §6.4).  `hint(x: k.center.x + k.r,
  y: pin.y)`, `hint(at: pin)`, `hint(at: k, bearing: b)` — on the sheet and in a child slot alike.
  The flattener settles a seed text over the parameters in scope and, where that
  fails and the text names a dotted scalar, keeps it (`settle_seed`, `reads_geometry`) with every
  dotted name resolved to the entity's absolute name in `Decl::seed_names` (`rescope_seeds`, the
  same `lookup` every reference goes through — so a formal reads as its actual and a block copy as
  itself, and never rewritten into the text, since `side.#282.0.small` is no name the expression
  language can spell); `program::build` records each as a `Deferred` and `settle_deferred` works
  them out after every kind is built, in statement order, off the built seeds (`seed_read`,
  `place_of`).  A read is a `Length` where the document names a unit and a bare number where it
  does not (`seed_eval`).  A `param` may not read geometry — it feeds constraints — and
  `commit_seeds` never writes an expression back, so P3 holds.  The bearing of a sheet
  `hint(at: …)` is `substitute`d over the scope's numbers, which print **with their unit** (`of_vals`: an
  `Angle` as `(180deg)`, a `Length` as `(150mm)`), or `phi + atan2(…)` reads as a plain number
  added to an angle.  A file's top-level `param`s are in scope in its components (`Walk::file_vals`,
  under the formals in `bind`) and so are the params of the modules it `use`s
  (`module_params`, memoised, cycle-safe) — the used modules' params are merged **before** a
  file's own are worked out, so `param rB = rp + 1.5mm` reads the table.  `tests/seeds.rs` is
  the gate.
- **A part is one component carrying `in view { … }` blocks** (Solvent §6.7, `P::in_comp`): the
  block form is allowed inside a component body — the plane is a formal, and nothing the
  document deletes reaches the header — and still refused inside a root block.  With
  `repeat flag { … }` over a 0/1 `Int` formal for the views an instance does not show in, a
  part's whole design is one module (`engine/block.sv`, `engine/head.sv`,
  `engine/crankshaft.sv`, `engine/conrod.sv`), the castings included; the view modules hold only
  what the assembly adds (the bore axis, the pistons on it, the timing drive).  Instances inside a block copy are indexed like declarations
  (`cyl[0].small`: `copy_of` returns the copy's prefix and `lookup` reads the rest under it).
- **Which way is a word, not a sign** (Solvent §9.2, §9.4; issue #48, item 4).  A distance
  measured *from a line* (`PointLineDistance`, `ParallelDistance`) is a **magnitude**: its kernel
  is `|g| − d` (`kernels::point_line_magnitude`, degree 1 — **not** the squared form, whose
  gradient vanishes at `distance(0)`, an idiom a drawing writes thirty times in one cylinder),
  both sides are solutions, and the seed picks between them, which is what P3 already says a seed
  may do and what every other sketcher does.  `side: left|right` pins one, and then the *signed*
  kernel runs with the word's sign — so a pinned statement costs no new kernel and an unpinned one
  is the new one.  `CKind::side_words` is the one table of "the words a slot takes and what each
  means as a sign": `left` is +1 of a line and −1 along the page, opposite numbers and the same
  English, which is exactly why the word is what a document writes.  A negative magnitude is E040
  **by value**, so `Loc(v: -hw)` is caught at the call; where a sign is *arithmetic* rather than a
  convention (a coordinate a caller signs) a document writes `abs(v)` and lets the seed — the
  point, worked out — say which side.  The run, the rise and the directed angle keep their signs
  (a component computes those, and by settling time the flattener has folded the text into a
  number that no longer says how it was written) and gain `along: right|left|up|down` and
  `sense: cw|ccw` as the spelling a drawing should use.  `io::dimension_text` draws the number the
  statement **makes**, so a `sense: cw` label and the arc beside it agree.  A component takes a
  side as `Ty::Side` — a word in `Scope::sides`, never a ±1 in `vals`, since encoding it as a
  number would put the unreadable idiom back inside every helper.  `cgraph`'s PL edge asks
  `Constraint::signed_gap`, which is the word where one is pinned and the *pose* where none is,
  because a plan moves a figure that already satisfies its constraints rather than choosing among
  their solutions.  `tests/refusals.rs` is the gate, and the corpus is the proof: every drawing
  renders byte-identical to before the change.
- **A selector says what it means, or it is refused** (Solvent §9.2; issue #48, item 4).
  `CKind::words(slot)` is the vocabulary a `Str` slot takes (`at: start|end`, `at: p1|p2`) and
  `constraints::ALONG` is the one table `along:` is read by — the choice *and* the message, since
  a second list is a second answer.  Three silences went with them: a key naming no slot was
  dropped and the statement settled without it (`Written::assemble` checked only `Slot` keys), a
  word outside a slot's set fell through `contact_point`'s `s == "start"` and silently meant the
  other end, and `along: z` came back as "`distance` does not relate a point to a point" — an
  error about the operands for a mistake in the selector.  All three are E040 **at the key**
  (`Written::key_span`, since a selector's value carries no span of its own), and
  `report::registry_json` publishes each slot's words so a front end offers what the core accepts.
  `tests/refusals.rs` is the gate.
- **A recorded root choice is one record of one triangle** (`decompose::branch_record`; issue #48,
  item 4).  Three points can be named six ways, and `ccw(a, b, c)` ("c left of a→b") is the same
  fact as `ccw(a, c, b)` with the sign turned — so a record is **canonical**: the point indices
  ascending, the sign read against that order, and the sorting permutation's parity folded into
  the sign.  Written in whichever order each writer used, the document's choice and the plan's
  were two records that never met: `apply_gauge` wrote `ppp:a|b|c` and the plan looked up
  `ppp:a|c|b` (`Step::stated` — it builds the corner as "y left of x→z"), so a document's `ccw`
  matched no step and decided nothing, and a step's own choice lifted back into source named a
  different triple.  Every writer goes through `branch_record` — the elaborator, the plan's
  `branches`/`apply_branches`, `io::graft`, `Part::branches_out` — and `io::from_json`
  **re-records** what it reads, so a document written before the rule migrates on load, the
  `"construction": true` bargain again.  `tests/decompose.rs` and `tests/order.rs` are the gates.
- **A name declared over a built-in is said** (Solvent §3.3, §5; `program::shadowing`, W112).
  `expr::eval` knows `expr::CONSTANTS` and `FUNCTIONS` before it knows the document and
  `flatten::substitute_with` knows only the document, so a `param`, a formal or a block's index
  called `tau` does not shadow the constant — it reads 35° where a text is substituted and a full
  turn where a number is worked out, which is how a lever came to stand at 360° with nothing said
  (issue #48, item 2).  A *named dimension* of a built-in name is already refused where it is
  parsed (`expr::parse_in`); the other three are the **warning**, since what is wrong is the name
  and not the drawing.  Asked of the text, like every question about how a statement is written:
  every component the program holds, instantiated or not, and every module's own body, each
  declaration once.  `expr::builtin` is the one table — "is this built in" and "what is it" are
  one question — and `tests/names.rs` is the gate.
- **A call is the entities by position and the numbers by label** (Solvent §4.1,
  `flatten::check_call`).  `Cylinder(swing, side, top, piv, rod, across, dir: dir, fw: fw,
  o_s: o_s, o_t: o_t)`: an argument bound by position must fill an *entity* formal and must stand
  before every label, and either mistake is **E004** at the argument.  Position is a count, and a
  count is the one thing a reader of a long formal list cannot check — an argument written a
  place off lands on the formal beside the one it was meant for, and what comes back is a
  complaint about something else entirely (issue #48, item 1: `views` "is not a number here",
  `n` is Scalar and this is an Angle, a hex whose `phase` had arrived as its side count).  The
  question is about the **text**, so it is asked of the text: once per written call, before
  anything is bound, however many times the walk binds it — thirty copies of a `cycle` are one
  mistake.  `tests/components.rs` is the gate.
- **Modules** (Solvent §14.4, `modules.rs`, `library.rs`).  `use engine.parts` is parsed into
  `Program::uses`; `modules::link(prog, resolver)` resolves each once, transitively, parsing the
  module with `syntax::parse_from(text, base, first_id)` — **every span is one integer into one
  virtual text**: the document, then each module after a one-byte gap, so no consumer learns a
  second coordinate and a splice (root body only) never meets a module span.  `Program::source_at`
  says which text an offset is in; `modules::localize` (run by `link` and by `elaborate`) shows a
  module's diagnostic at the `use` that brought it in (`Module::via`) with `name:line:col` in
  front of the message.  A module contributes its components (`Component::module` says which)
  and its top-level params; its own drawing is not drawn.  **The core has no filesystem**: the
  resolver is the host's — `solventc` reads `engine.parts` as `engine/parts.sv` beside the
  document, then `library::resolve`; the FFI and `examples::document` use `library::parse_linked`,
  over `library::MODULES` (compiled in with `include_str!`, which is how the app opens the
  engine).  **`rust/lib/` is the standard library** — `std` (`ThreeViews`: the three principal
  views laid out from one grounded point) — and is in the Makefile's `RUST_SRC` so editing it
  rebuilds what compiled it in.  `program::reparse` relinks from the texts already in hand (`modules::relink`), so an
  edit never asks the host.  E070 no module, E071 defined twice.  `tests/modules.rs` is the gate.
- **`port` is retired** (issue #47, item 1).  Everything an instance makes is reached by its
  dotted name (`five.s[0].p1`, `t0.mid`), so a port was a second name for a thing that had one:
  its declaring form is a `point` of the body, its alias form is the caller writing the entity's
  own name, and the one real construct under the keyword — the **computed point** — is
  `point p = (xexpr, yexpr)` (`Decl::computed`, the same brackets-say-what-it-is-made-of rule as
  every other declaration), refused on the sheet by the flattener and compiled to two tapes when
  traced.  The parser keeps the word in `OPENERS` only to refuse it naming the three forms.
  Aliasing is untouched: it is a property of argument passing (`bind_instance`), not of ports.
- **A class stands on a relation and on an instance, and `display: none` hides** (Solvent
  §13.2).  `Relation::class` is parsed in both trailing-clause loops (a chain's and a lone
  relation's, before `at`), set on `Constraint::class` by `constrain`, written by `io::dumps`,
  read by `from_json`, carried by `graft`, printed by `write_relation`, and **not** in the
  binding's constraint record (identity and arguments only).  `callout::style_of` resolves
  `.dimension` (`.reference` for a claim) under the statement's classes, and `layout` skips one
  that is not `shown()`, so neither front end lays out or picks a hidden dimension.
  `Instance::class` is carried down the expansion as `Scope::in_class` and stamped **over** each
  emitted declaration's and relation's own (`stamp_scope_plane`, the Relation arm of `body`) —
  the assembly's word is the stronger — the way `in` is.  `Style::display` is
  `display: none | inline | geometry` (`style::Display`): `shown()` is what an entity asks,
  `dimensioned()` what `callout::layout` asks, and `geometry` is drawn but never dimensioned —
  a phantom position, `style .phantom { …; display: geometry }` on a ghosted instance of a
  dimensioned part.  `svg::render` and `paint.ts` skip what is not shown, and every point is
  drawn under the implicit class `.point` (`EntKind::implicit_class`), read once per repaint
  through `styleNamed('point')`.  The idiom for a drawing dense with dimensions is
  `style .dimension { display: none }` and `class shown` on the few to draw.
- **A declaration need not name its children** (Solvent §6.1, §6.2).  `line l` mints two points,
  `circle c` one, `arc a` three; a child slot may hold a `hint(…)` instead of a reference
  (`line alt_a(A, hint(x: 15, y: 5))`), which is the same clause standing in for a child rather
  than qualifying the declaration it follows.  `Decl::children` is therefore `Vec<Vec<Kid>>` —
  a name *or* a seed, and no third form, since "anonymous and unseeded" is spelled by an *empty
  slot*: a slot the list leaves out is an **implicit child**, minted by `program::build` exactly
  as a wholly-unwritten list's are, which is what lets a chain's marker fill only the ends it
  speaks for (`line l1 -> line l2` is three points, one shared).  E103 now refuses only a list
  with *more* children than the kind has slots.  A joint threads a *name*, so a seeded slot
  reads as unfilled there and the other side may say where the two meet — and between two
  declarations where neither does, `thread` mints the name itself (the earlier-built side's
  dotted boundary, `l1.p2`), refusing only when a side is a name-link whose kind no boundary
  field can be read off.
  **The dotted path is the name.**  An anonymous child has no name in the source, so `l.p1` *is*
  its name: `program::build` mints the point *with* that name, binds it in `map.names`, and
  records it against the parent's statement — which is what makes it resolve, constrain, drag,
  be picked, be dimensioned (`edit`'s `name_of` reads the map), survive a re-elaboration
  (`Document.entity`) and read as `l.p1` in the status line.  Nothing in the bindings changed.
  Writeback follows it down: an anonymous child's seed lives in the *parent's* statement, in a
  slot, so `commit_seeds` walks the parent's children and splices inside `Kid::Hint`'s spans —
  and where the source wrote no list at all, writes the whole argument list at `hint_span` in
  one edit, since two splices at one offset are two insertions racing for it.
  **The element's own name is optional too** (issue #33), independently of everything after it:
  `line`, `line(p1, p2)`, `circle hint(r: 25)` and `arc(center: c)` are all anonymous forms
  (a line owns no scalar, so its ends are seeded in the slots: `line(hint(x: 0, y: 0), hint(x: 60, y: 20))`),
  and the token after the kind keyword decides — `syntax::names_decl` is the one predicate, asked
  by `decl()` and the colouring alike, so a trailing-clause word, an operator word or an element
  keyword can no longer be a declaration's name (`curve` keeps requiring one; contacts address
  it — the colouring carries the same exception).  A word declined there is remembered
  (`P::declined`) and named in a note when the line then fails to parse, since the reservation
  is the cause no other error can see.  An anonymous declaration still carries a `Decl::name`:
  a key the source cannot write — `#a` and its own offset, the flattener's block-prefix device
  marked apart — with an **empty span at the point a real name would go** (`hint_span`'s
  idiom).  **A name is three questions, not one, and each is known where the name is minted and
  told — never sniffed back out of the characters** (issue #39).  The three: does it
  **resolve**, does the source **call the thing that** (so: shown, published, selected by), and
  may a statement be **written** with it.  `Decl::name` is a **`DeclName`** — the name fused
  with the three-question answer (issue #40): `Written(Name)` (`l0`, `s1.p0`), `Copy(Name)`
  (`#3.0.p`, one copy of a block) and `Key(Name)` (`#a41`, an anonymous declaration's minted
  key).  Fused, not a `bool` beside a string, so no reader can take the text without picking the
  accessor that says which question it asks — `key()` (resolution, every declaration's),
  `shown()` and `written()` (each an `Option` the compiler makes a new reader answer), `span()`
  (where a name is or would go) — where eleven guard sites used to be a convention policed by
  memory, four of which were not written.  It is stamped by the two mints that know: the parser
  took an identifier or declined to, and **the flattener knows whether the prefix it is putting
  on the front is an instance's own name or a block's id** (`Scope::copies`, and
  `DeclName::prefixed` — a key is prefixed like any name, since two copies of one block hold two
  entities the resolver must tell apart).  `SourceMap::bind` is then told (`DeclName::named`,
  the bare answer, is its vocabulary), and files each name into as many of its three tables as
  apply: `by_name` always, `names` when `shown()`, `writable` when `writable()`.
  So `map.names` is exactly "what the source calls the thing" — `name_of` is `names.first()`, an
  anonymous entity has no entry — and `writable_name` is the narrower one `edit::reconcile`'s
  gate asks, refusing a gesture on one copy of a block *with the cause* instead of writing the
  prefix out for `adopt` to fail on.  Two answers would not do: a predicate over characters can
  separate `Copy` from `Key` only by the `#a` marker the anonymous mint happens to use, which is
  the fragility the issue names, and it is `Copy` — unwritable but shown — that makes the two
  spellings disagree.  `syntax::hidden` survives only where the question really is about
  characters and only a `Ref` is in hand: a thread-filled slot's `Kid::Ref`, whose root names
  *another* link's boundary.
  The key is what a chain's corner welds by and what `res` resolves, and it must
  never reach the source: `write_decl` spells the statement without it,
  `commit_seeds` leaves a thread-filled slot holding one *empty* — forcing labels on the kept
  children when a gap precedes them (`decl_args`), since a line's slots count by position — and
  diagnostics spell the kind instead.  `edit::reconcile` **mints on demand**: the moment an
  appended statement must reference an anonymous element (a constraint from the app, a gauge on
  a fixed point — `held_refs` is the one walk `gauges` shares), a real name is spliced into the
  declaration at that empty span, **every entity the statement made** renamed with it — a child
  by the dotted path `program::child_names` would have given it, read off its *position* among
  the parent's children, which is where the path came from in the first place — and `bind`ed
  `Named::Written`, so the next gesture in the same elaboration reads a name where it read
  nothing.  What a statement made, in the order `build` made it, is `SourceMap::ents_made_by`,
  which `commit_seeds` and `reconcile` share so the ordering invariant is stated once.
  Its four guards are then one question, "does the source call this anything": named, no
  statement of the root's to name it on (refused with the cause, before anything is written),
  named since the map was made, or mint.
  Insertions racing for one offset are ordered by `splice`'s stable sort, so reconcile pushes
  appends before flags before names; `tests/anonymous.rs` is the gate.
  Where an unseeded point *starts* — an implicit child, a declared `point a` with no `hint(…)`
  clause, inside a component or not — is `program::scatter` and is an implementation choice
  the spec must not carry — but it may not be the origin (two endpoints there is a zero-length
  line, with no direction for `horizontal(l)` and a singular row for any tangency; two points
  there put a `distance` at a stationary point of its own residual, and the first document
  anybody writes solved as a conflict), and minted points may not pile up or seed a contour as a
  self-crossing quad: a collapsed side satisfies every direction constraint on it, so that basin
  must not be where a solve begins.  `scatter` therefore walks the bearing a fixed irrational
  step per minted point, in creation order — which for a chain is traversal order, so a contour
  seeds as a simple polygon.  "No clause" is the empty `hint_span` the parser leaves where one
  would go (a lifted declaration has `None` and keeps its numbers), and `commit_seeds` then
  writes the solved pose in as the clause.  A component's points take the same clause, and
  `gear.sv` / `gear_trace.sv` state theirs: from the
  centre, where every circle's row is flat, a flank's first step lands on the involute at the
  roll its contact names, where a start a unit off reached the mirror branch.
- **Presentation is a separate statement from what the drawing is** (`style.rs`, Solvent §13.2).
  A declaration carries a **class** (`line datum(o, q) class construction`) and a top-level
  `style .NAME { dash: 7 4; width: 0.5; color: #888888 }` says what a class looks like.
  **No algorithm in the core consults a class** — nothing in the model, the kernels, diagnosis or
  decomposition reads one, and that is the whole point: `construction` was a `bool` on seven
  entity structs, serialized, grafted, exported, published and given a toggle, all to reach one
  arm in `paint.ts`, and each new look cost the same again.  A class is one string in the same
  places, once, and the count goes into the sheet instead.
  `construction` is therefore no longer a word in the language: it is a class, and
  `style .construction { dash: 7 4 }` is the one rule in `style::base()`, which a document may
  override.  **The core resolves and the front end strokes** — the same seam `callout.rs` and
  `curve::tessellate` sit on, so `paint.ts` reads `ent.style` (dash, width, colour) and knows
  what a class is nowhere; `app/edit.ts`'s toggle sets and clears the *name*.  A sheet's lengths
  are **screen pixels**, never world units: a dashed line does not change its pattern when you
  zoom.  An unmatched class is not a diagnostic — it has no rule, as in CSS, which is also what
  makes paste work.  The cascade is **two layers, not one interleaved pass**: the whole base
  sheet under the whole document's, each in written order, so what a document says beats what
  the implementation ships whichever class it is written on.  Resolved a class at a time, a
  later class's shipped rule would override an earlier class's *stated* one.  A base rule
  therefore states only what its class **adds** — `.reference` is the lighter ink and nothing
  else, because a reference dimension *is* a dimension and is drawn `class dimension reference`;
  restating the shared weight there would make it a complete rule, and one
  `style .dimension { width: 2 }` would come out thick on half the callouts.  A value the sheet
  cannot read (`color:` with nothing after it) is **dropped**, exactly as an unknown property is
  — `Some("")` is not nullish and reached `ctx.fillStyle`, which ignores what it cannot parse.
  `Sketch::style_epoch` is bumped by the one write path (`set_class`,
  `set_sheet`) so a binding may cache a resolved table against it.  JSON writes `"class"` and
  **reads `"construction": true`** and never writes it, the same bargain `from_json` already
  strikes with the pre-§13.1 placements table.
- A constraint may own *unknowns* of its own: a `SpecKind::Param` slot in its `spec`, allocated
  by `Sketch::add` and moved by the solver like any other parameter.  The slot holds a seed
  number on the way in — which is what a document stores, what `graft` copies, and what
  `constraints::seed_param` supplies when a caller omits it (the `Param` counterpart of
  `infers_arg`) — and an index into `Sketch::params` once added.  It is not a value anyone
  states: `describe` leaves it out, both bindings publish it read-only, and `same_constraint`
  ignores it, since two contacts of the same point on the same curve say the same thing however
  far apart their seeds started.  `Sketch::remove` retires an orphaned one to `fixed` (a free
  parameter no equation mentions is a DOF the sketch does not have); the rebuild walk reclaims
  the slot outright.
- **The core owns every algorithm.**  A change to the model, a constraint type, diagnosis,
  decomposition or the solvers lands in `rust/gcs-core/` with a Rust test in
  `rust/gcs-core/tests/` — **a new file there is listed in `tests/main.rs`**, since the core
  suite is one binary (`autotests = false`; the note at the top of `main.rs` says why) and a
  file nobody lists is a test nobody runs, which `every_file_is_a_module` fails the suite
  over.  A binding changes only when the *surface* changes.  If you find
  yourself writing geometry or numerics in TypeScript, it belongs in Rust instead.
- A new *entity* kind stops the build in the exhaustive `match e.kind` arms — `model.rs`
  (`entity_params`, `own_params`, `own_length_params`, `children`, `min_children`, `count`,
  `bounds`, `distance_between`, `point_to_drawn`, `class_of`, `set_class`, `spatial`), `io::graft`'s
  remap, `overview::drawable`, `svg::entity`, `program`'s build and `set_class`,
  `syntax::kind_initial` and the FFI's `ent`/`kind_id`.  Give it an arm in each; `primitives()`
  and `topology_key` are where it joins the document.  A kind that is **evaluated rather than
  drawn** answers `spatial()` and owns no parameter, which is the whole of how `Face` and `Solid`
  sit in the same enum as a line without being on the sheet.
- Every new constraint type = a vectorized kernel in `kernels.rs` (added to `KERNELS`; the
  registration order **is** the kernel id) declaring its `degree` — the power of length its
  residual carries, 1 for a signed distance and 2 for a squared one — a `CKind` variant in
  `constraints.rs` declaring its `spec` (constructor args as (attr, kind) pairs), `params()`,
  `consts()` and `default_arg()`, and a row in `rust/gcs-core/tests/jacobians.rs` (FD check,
  spec round-trip).  Both bindings generate their classes from `report::registry_json`, so
  neither needs touching.
- A type whose spec carries a `Length` or an `Angle` is a *dimension*, and dimensions are drawn.
  `callout.rs` matches `CKind` exhaustively in two places, so adding any type stops the build
  there: give it a `Pen` arm (its drafting figure) and a `frame` arm (the `Frame` its placement
  is written in), or list it in `undrawn!`.  `every_dimension_is_drawn` then checks the arm you
  wrote actually produces a figure.
- A parametric curve (`curve.rs`) is one that is *linear in its control points*, `C(t) = Σ Bᵢ(t) Pᵢ`
  — every B-spline, so every Bézier.  It has no usable implicit form, so a contact with one
  carries its own curve parameter as a `Param` slot and says `p − C(t) = 0`: two residuals, one
  new unknown, the net one equation the contact is worth.  A contact kernel needs the basis
  values and their first two t-derivatives and nothing else about the curve, which is the whole
  extension point — a second curve family is a second basis, not a second constraint family.
  A line against a curve is a tangency; a *circle* against one is a curvature constraint —
  `SplineCurvature` makes it the curve's osculating circle, which is the circle a draughtsman
  would call the radius there.  It is written as "the centre is the centre of curvature", which
  says touching, tangent and equally-bent all at once and needs no `side` to infer.  Dividing by
  the turning rather than multiplying by it is load-bearing: multiplied through, every row would
  vanish as `C'` did and the solver would satisfy the constraint by bunching the control points
  until the parameterisation collapsed, which it promptly does given the freedom.
  Control points are ordinary `Point`s, so they drag, snap and constrain with the tools that
  already exist, the same trick as an arc being a centre and two real points.  Local support is
  what keeps the plan's fixed-width blocks: only `DEGREE + 1` control points are non-zero at any
  t, so a contact addresses one *span*, whichever span t is in.  The span is derived from t, not
  stored, and `Sketch::topology_key` carries it — a contact walking past a knot is a recompile,
  the same event as any other topology change.
- A control polygon is edited by three operations, and they are not the same shape of thing.
  *Inserting* a control point is `curve::insert_control` — Boehm's knot insertion, so C(t) is
  identical afterwards and every contact keeps both its parameter and its place on the drawing;
  `DEGREE - 1` neighbours move, keeping their identity, and if one of them is constrained the
  next solve honours that instead, which is a stronger thing than "keep the shape".  *Deleting*
  one shortens the curve rather than destroying it: `Sketch::min_children` is the general rule
  (an entity survives while enough children do — for a line or an arc that is all of them), and
  `curve::knots_without` gives up one interior knot per lost control point, so deletion is very
  nearly the inverse of insertion.  *Interpolating* is `Sketch::spline_through_held`: chord-length
  parameters, averaged knots and one collocation solve give a control polygon whose curve passes
  through the given places.  A place that came from empty space is construction input and leaves
  nothing behind — the same bargain `arc_through` strikes with its third click; a place that came
  from a Point is *held*, by a `PointOnSpline` whose parameter is **pinned** at the value the fit
  chose.  The pin is what makes the answer determinate: a contact with a free parameter says only
  "the curve meets this point somewhere along its length", so without it a curve through m points
  keeps m degrees of freedom and could slide along itself, and a fit to fully constrained points
  would come out under-constrained.  The fit knows where along, so that is knowledge and not an
  unknown.  A pin travels in the seed: `Arg::Seed { value, pinned }` is what a `Param` slot holds on the
  way in, and `Sketch::add` consumes both halves at the one seam that turns a number into a
  Param — so a document, a paste, a rebuild and a constructor all carry it without knowing pins
  exist.  `clamp_contacts` leaves a pinned parameter alone: somebody said where along, and the
  solver is not to argue.
- A curve parameter is bounded (`t0 <= t <= t1`) and a least-squares problem cannot say so: left
  alone the solver puts a tangency on the phantom polynomial past the end of the drawn curve.
  `System::solve` says it instead — clamp, compare `curve::contact_spans` against the spans it was
  compiled from, rebuild itself when one moved — so *every* caller gets it: the one-shot `solve`,
  the plan solver's fallback and a front end that compiled a system for itself alike.  A clamped
  parameter is *pinned* for the retry: free, the next solve walks straight back off the end.  A
  contact *seeded* off the end is the other case and is clamped **before** the first solve and
  left free — a seed says only where the search begins (P3), and pinned, `hint(t: 2)` nailed the
  point to the curve's last control point and a solvable document read UNSOLVED (#45.7).
  `SolveOpts::rehome` turns it off for the one caller owning a *pair* of systems that must stay
  in step — `PullPolish`, which re-homes both together, lifting its drag target out to rebuild.
  All of it is behind an empty span map, so a sketch with no curves pays nothing.
- A block's columns and its constants are ONE compile-time choice: which span of a spline a
  contact sits on.  `System::new` makes it once and passes it to both `params_on` and
  `consts_on`, and remembers it — so `refresh_consts` skips curve contacts outright (their knots
  are document data no solve moves) rather than re-deriving a span that may since have walked.
- `Param::scale` is the world length one unit of a parameter is worth — 1 for a coordinate or a
  radius, the curve's mean speed |C'| for a curve parameter, which `System::new` reads off the
  curve itself so it is a fact about the compile and cannot go stale.  `System` gathers it into
  `col_scale` and solves in `z = x * col_scale`, so the trust region and the minimum-norm step
  measure motion in world units.  It is not a nicety: unscaled, a tangency that converges in nine
  iterations at one size stalls at ten times the size, because the t column is wrong by a factor
  of the curve's length.  Systems where every scale is 1 take the untouched path.
- "Solved" is `System::max_relative_residual <= 1e-6`: each row's residual over its own units
  (`extent^degree`).  Never one absolute threshold for the whole system — half the kernels are
  linear in length and half quadratic, so one threshold is wrong for one of the halves.  **And
  the iteration sees the same vector**: `residuals_into` and `compute_csr` divide every row by
  `row_scale` where it is produced, so the dogleg's merit function, its ratio test and its
  Cauchy step weigh an `angle` row (radians, O(1)) and a `distance` row (a squared length,
  O(L²)) alike.  Minimised raw, a four-bar linkage with its crank angle stated turned about a
  degree an iteration and did not solve at all past forty units across (issue #43).  Anything
  reading a residual off a `System` — a test comparing against a kernel — multiplies by
  `row_scale` to get the raw one back.  A pose that is *not* a solution is one of two things,
  and `System::stationary` tells them apart: at a stationary point the residual left is what the
  constraints cannot agree on, and the diagnosis may call it a conflict; anywhere else the solver
  merely stopped, the diagnosis solves a scratch copy to find out which, and a consistent
  drawing is `State::Unsolved` — no conflict set, no culprits, the unsatisfied rows listed as
  what they are.
- Its Jacobian twin: a rank or a null space is judged on `System::conditioned` — the hard rows
  over `extent^(degree−1)` (a degree-`d` residual's derivative with respect to a length carries
  one power fewer), columns already in world length — against one absolute, dimensionless
  `system::RANK_TOL`.  Never a raw `J`, and never relative to `σ₀`: `σ₀` belongs to whichever
  row is largest, and that row may be a dimension in another figure entirely, so a relative
  rule lets an unrelated part of the drawing decide whether two constraints on a circle are
  dependent.  `Conditioned` is the only matrix the *diagnosis* will judge, and its methods take
  only an absolute tolerance, so the comparison cannot be written the other way — the `Tol`-taking
  factorisations are `pub(crate)` for the same reason.  (`decompose::deficiency` and `homotopy`
  keep their own relative rank on their own synthetic matrices, which are neither the sketch's
  Jacobian nor in the sketch's units.)  `jacobian_dense` stays the raw `∂r/∂z` for the solvers
  and the finite-difference checks.
- A tangency whose contact is a point the drawing already holds to the curve is stated *at*
  that point, or it is a double root: `PointOnCircle(p, C)` + `TangentLineCircle(L, C)` with p
  an endpoint of L say "tangent at p" with a Jacobian that is rank-deficient at *every*
  solution (the contact "swims" along the line to first order, blocked at second), which no
  rank tolerance can read correctly.  `TangentLineCircleAt(line, circle, at)` — the
  `tangent_arc_line` kernel reused, the radius perpendicular to the line at the named endpoint
  — is regular, and `commands::cTangent` states it whenever the picked line has an end already
  on the circle, exactly as it already did for an arc's own endpoints.  The on-circle stays the
  user's constraint; the pair is the statement.  For the degenerate pairs that still arise (an
  old document, a tangency applied before the endpoint was snapped on), the diagnosis and the
  witness settle-test the surplus motions: step along a null direction, let the solver settle,
  and a motion that walks back (a double root has no solution out there) is `shaky` — counted
  out of the DOF, reported as "blocked at second order", never painted as under- or
  over-constrained.  `numeric_rank` already includes them, so nothing downstream adds `shaky`
  back.  Every guard is inside `witness::screen`, because a caller that writes one itself will
  write it differently: the sketch must be at a solution (or the settle measures the solve, not
  the geometry), it must contain a tangency at all (the only thing a double root is made of —
  which is what keeps the solves off the ordinary theorem-type dependency, `polygon_chain`
  paying eight solves an edit for a freedom that is perfectly real), the removals are capped at
  what the matching cannot account for (so a settle that lands short can never eat a genuine
  DOF), and there are never more than `SCREEN_MAX` of them.  A candidate cannot be recognised
  more cheaply than by trying it: reached exactly, a double root's singular value is as small as
  a real freedom's, so there is nothing in the spectrum to sort on.
- `Horizontal`/`Vertical` level a line; `HorizontalPoints`/`VerticalPoints` level a *pair of
  points*, which is the same statement about the segment between them and needs no line drawn
  there.  They reuse the line kernels unchanged — those four columns were always two points'
  coordinates — and `cgraph` gives them a `virtual_line` in the ground x-axis's direction class,
  the same trick arc-endpoint tangency uses, so a levelled pair decomposes rather than falling to
  the numeric residue.
- **The datum is a `plane`** (`plane f(origin: o, toward: q)`, spec §3.2 [0.6]; issue #47,
  item 6 folded `frame` into it — the two were one construct with the attitude optional, and a
  plane with no attitude written is a view of the page, which is what a datum on the sheet is;
  the parser keeps the word `frame` only to refuse it, at a declaration and at a formal).  The
  two points alias, and the attitude is a **unit rotor** `(c, s)` — two owned scalars slaved to
  the chord by two intrinsic constraints minted in `Sketch::plane` and nowhere else (the arc's
  bargain, since
  intrinsics are never serialized): `frame_unit` is `c² + s² − 1`, **degree 0** (dimensionless,
  judged absolute — the `angle` kernel's rationale), and `frame_align` is `(t − o) − r·(c, s)`
  with the chord's length `r` a `Param` slot of its own — two rows, net one equation, and
  *directed*, where a bare cross-product row would admit the reversed frame.  Net **0 DOF**
  beyond the two points; `c`,`s` get `Param::scale` = the chord's length.  A rotor rather than a
  stored angle because it has no mod-2π seam, its rows are polynomial, and it is the 2D unit
  quaternion — a 3D workplane changes the component count, not the construct.  `.angle` is
  **derived, never stored**: `Tape::compile`'s one exception to the misspelling rule turns
  `f.angle` into `atan2(f.s, f.c)` (degrees) wherever the table holds the rotor — which is what
  lets a traced seed say `bearing: u + f.angle` and follow a tilted datum (issue #10;
  `tests/frame.rs` holds the mirror-elbow document that page-fixed seeds quietly get wrong).
  A datum is also the shortest formal list a traced component can be written over — it *is*
  an origin, a second point and a bearing, so passing those beside it states one datum three
  times: the Peaucellier cell went from 20 variable-table columns to 12 by taking `(orbit, f)`
  where it had taken `(o, q, datum, orbit, f)`, which is what kept it under `tape::MAX_VARS`.  Raising that constant is the wrong reflex — `get`/`map`/
  `zip` zero a `[f64; MAX_VARS]` per operand, so its width is a cost every tape pays whatever it
  reads, and 16 → 24 measured +7–14% on tapes as narrow as four variables.  Both intrinsics
  are `unsupported` in `cgraph` for now, so a document with a datum drags on the numeric path;
  the direction-class promotion is a follow-up.  `FrameE` is the datum half of a `PlaneE`, and
  `SpecKind::Plane` is what the two intrinsics take.  A document written before the fold loads:
  `io::from_json` reads a `"frames"` table as planes with the page's attitude and never writes
  one — the `"construction": true` bargain again.
- A **`plane`** (Solvent §6.7) is also a **view**: the datum's origin, toward point and rotor
  (the two intrinsics, minted by `Sketch::plane` through `datum`/`slave`), plus a constant
  attitude in space, `plane::Basis` `(u, v)`, with
  `n = u × v` toward the viewer.  **Nothing three-dimensional is solved for**: the basis is
  document data like a spline's knots, resolved at elaboration (`program::plane_bases`, a
  memoised walk over the `from` chain — the page, `from: P, fold: θ` as `Basis::fold`, or
  `u:`/`v:` orthonormalised by `Basis::explicit`) and stored on `PlaneE`; it is written in the
  brackets with the children because it is what the plane is *made of* and no solve moves it.
  A point's **membership** is `PointE.plane`, set by `point a in top` — a trailer applying to
  every point the declaration mints or names, filled in by `program::memberships` after every
  kind is built and before any constraint — and it moves nothing: only `Project` reads it.
  `a project b` is one row over 12 columns (`kernels::project`: both points, both planes'
  origins and rotors) with the fold line the planes share as consts (`plane::fold_line`).  The
  two plane slots are real entity slots — so `io::Part`, `topology_key`, the graft and a
  deletion follow the planes with no new code — and **inferred**: `infers_arg` marks them, the
  registry publishes null, the source and the bindings write two points, and
  **`io::seed_omitted` is the one seam** that fills them (`constraints::infer_entity`) and
  refuses what the model refuses (`constraints::validate`: no plane, one plane, parallel
  planes) — returning `Result`, so the elaborator (E061 at the statement), `from_json`, the
  FFI's `gcs_constraint_add` and `Constraint::project` are refused by one rule.  `operator_text`
  skips an inferred entity slot, so `describe` and a lifted statement both say `a project b`.
  A plane's minted label starts with `v` (`syntax::kind_initial`, now exhaustive): `p` is the
  point's, and a plane on the page is a view — as a curve's starts with `k`, `c` being the
  circle's.  It draws its chord as a datum glyph — the kind's
  *implicit class* `.plane` (`EntKind::implicit_class`, resolved under the declaration's own in
  `style_of`, so a document's `style .plane` rule wins and the JSON never writes it) — and is
  picked by that chord, its points outranking it as everywhere.  Deleting a plane from the source
  (`edit::remove`) dooms a plane folded from it (`mentions` counts `Attitude::From`), splices
  the `in` clause out of every surviving declaration (a membership is a label, not a
  dependency), and dooms every statement whose *elaborated* constraint named it — a projection
  never spells its planes.  `commit_seeds` replaces the bracket list at `Decl::list_span` rather
  than inserting a second one beside a list that stated an attitude and no children.
  **`in top { … }` is the clause written once**: the parser **hoists** the body's statements
  into the enclosing body, stamping each declaration (`Decl::plane_from_block`, `stamp_plane`
  recursing into `repeat`/`cycle` bodies so a contour drawn round a cycle is drawn in
  the view) — so writeback, carets, the DOF ledger and `edit::in_root` see ordinary root
  statements, and only the header and brace are the block's (`Program::in_blocks`), which is
  what `remove` splices when the plane goes.  The printers spell no clause a statement did not
  write, and a membership edit on a block-stamped declaration is refused with the cause (the
  clause is the header's, not the statement's).  Top level only — inside a body the clause
  says it per declaration.  **An instance joins a view whole** (`t: Tooth(…) in top`, or an
  instance inside the block): that stamping is the *flattener's* (`Scope::in_plane`, carried
  down the expansion and applied in `stamp_scope_plane` — the ref as written *with the prefixes
  of the scope it was written in*, which is what `rewrite` resolves it against: resolved through
  the emitted statement's own chain, a body declaration called `top` took the caller's view,
  #45.4), skipping datum and curve kinds,
  refusing a plane given twice, and reaching an aliased argument point through any body
  declaration that names it.  `add_rectangle` takes the plane, which is how the rect tool
  joins the current view.
  Not commutative (`same_args` swaps only the first two entity slots).  `cgraph` leaves it
  unsupported, so a multiview drawing drags on the numeric path.  `bracket.sv` is the case;
  `tests/plane.rs` and `tests/plane_lang.rs` are the gates.
- A **`claim`** (Solvent §9.7) is a constraint-shaped statement that is *judged, never solved
  for*: **no** `System` compiles a row for it, and decomposition (`cgraph`), the drag-part walk
  (`io::Part`) and the witness's jitter all skip it, so a claim can never move geometry, weld two
  figures into one drag part, or paint a sketch Over or Conflict.  The exclusion is written twice
  and deliberately: `Constraint::acts()` is the named half, which `hard_constraints`/`hard_ids`
  ask so a consumer added later inherits it rather than having to remember; the seams that need a
  row index or an enumeration spell it out inline, exactly as `soft` already is.  Anything that
  reads a constraint list to learn what determines the drawing — `known_radii`, the entity
  colouring, the conflict candidates, `duplicated` — must go through the named half, or a claim
  silently acts.  The diagnosis alone reads it, at the *end* of `diagnose_with` and through
  `System::conditioned_with`, which stacks a claim's rows onto the system already compiled:
  *theorem* (holds, and adds no rank), *violated* (does not hold), or *consuming* (holds only by
  the pose — enforcing it would have taken a freedom).  Judging it by compiling a second `System`
  is the thing not to do — a compile calls `locus::forget`, so a system built beside a live one
  throws that one's remembered trace poses away and every contact re-walks its march from the
  home: 834 µs a diagnosis on `peaucellier` against 69 µs now.  A claim may not own a `Param`
  slot and may not bind a free variable, since its unknown would sit in no equation:
  `CKind::claimable` is that rule, elaboration turns it into an E040 with a span, the document
  readers (untrusted input) drop the flag, and `expr::evaluate` refuses the free binding.  The
  flag travels like any other: `graft`, the document (`"claim"` in JSON, written only when set)
  and both bindings' records.  `peaucellier.sv` is the case, `tests/claim.rs` the gate.
  **What the front end shows is the core's wording, not its own**: `io::describe` prefixes
  `claim `, so every constraint list says which statements are claims, and `callout.rs` draws a
  claimed dimension **parenthesised** — the draughtsman's *reference dimension*, which says "this
  is what it measures, and it is not what controls it", a claim exactly.  The parentheses wrap
  the whole label (`(R50)`, never `R(50)`) and wrap *before* the text is measured, so the lane it
  is given and the box it is picked by are the size of what is drawn.
  **A verdict is shown where the claim is written, and quietly.**  The app is for *sketching*;
  proving is a remark in the margin, so there is no banner and no status — just a wash of colour
  behind the statement in the program panel (`app/program.ts::marks`, `.claim-proved` /
  `.claim-refuted` / `.claim-independent`), legible when looked at and invisible when not.
  `proved` is the faintest, being the expected answer; `refuted` the strongest, being the only
  one that is news.  The words are the classical trichotomy and are meant exactly: *proved* (the
  document entails it), *refuted* (this drawing is a counterexample), *independent* (true here,
  but not implied — stating it would have cost a freedom).  Not "inconsistent": under-constrained,
  a refuted claim may still hold at some other solution, so what is known is the counterexample
  and not the contradiction.
  `CodeEditor` therefore carries a *set* of marks rather than one lit range, and they may overlap
  (a picked statement that is also judged) — a mark tints and **never changes the face**, and
  background/box-shadow/outline are the safe properties while border/padding are not.  The
  splitting that composes overlapping marks is what `npm run overlay` guards: it drives the real
  `CodeEditor` in headless Chrome, and it is the only check that can see a dropped or repeated
  character moving every glyph after it.  Run it when you touch `editor.ts` — `make test` cannot,
  since it needs Chrome.
- `same_constraint` is "says exactly the same thing"; `same_relation` is the same *without* the
  numbers — same type, same entities, same flags.  A repeated *relation* is refused by the app
  (`edit::applyConstraints`): it says nothing the sketch does not already say and adds equations
  without adding rank, which the structural check cannot see.  A **dimension is never deduped by
  the UI** — not a run on a pair that already has a length, and not the same number stated twice.
  Whether a second number is redundant or a contradiction is the solve and the diagnosis's
  reading, and it comes back as `over` naming both, which is something the drawing can show and
  the user can act on; a button that guessed would instead decide it silently, and would have to
  decide from the type alone that the run somebody asked for is an edit of the length they had.
  So `gcs_constraint_stating` is a question the core answers and the front end no longer asks:
  `commands::dimension` states, `commands::editDimension` opens one already on the drawing (the
  constraint list's and the callout's double-click), and that is the whole of the distinction.
- An argument the core reads off the geometry (a tangency's side or sense) declares
  `CKind::infers_arg`; the registry publishes a null default for it so a binding leaves it
  omitted and the core fills it in.  A binding that substitutes a constant picks the branch.
- Mutating a constraint's constants (drag target, edited dimension) must be followed by
  `System::update_consts` / `refresh_consts` on any compiled system, or a recompile.
- `System` is the compile-once / evaluate-many seam; keep the object model out of the hot loop.
  The bindings own a handle — call `dispose()` when you drop one (the drags, the plan solver and
  diagnosis all do).
- Diagnosis is structural (matching/DM); numeric rank cross-check is the only thing that sees
  theorem-type dependencies — that residue is Stage 4's corpus, keep logging it
  (`Diagnosis.warnings`).  It is skipped above `NUMERIC_MAX` (300) free parameters because it
  runs after every edit.  A consistent dependency is `over` ("remove one") only when a
  dimension takes part in it — editing that dimension is the next conflict; one among pure
  relations is a theorem that nothing can break, so its wholly-implied constraints are
  `implied`: noted, never painted as an error.
- **Every case in the library is a Solvent document.**  Each is a `.sv` file in `rust/examples/`,
  and its builder is a one-liner that elaborates the text (`examples::document`).
  A case that takes arguments is still one implementation: `with_params` gives the document's own
  named numbers — a `param w = 100` line, or a `== a = 30` dimension — the caller's values, since
  a drawing written as a document already names what it is drawn from, and a second copy in Rust
  is a second drawing the moment one is edited.  A start that is off the solution is `jitter`, a
  function of the sketch: the document says what the figure *is*, not where a solve begins.  A
  document's **seeds must track its parameters** (`pythagoras.sv` places its square from `la`/`lb`)
  — a case is asked for unsolved, so seeds frozen at one size are a wrong drawing at another.
  `tests/examples_sv.rs` holds each document to what the library advertises about it.
- **What is not a drawing is not a case** (`fixtures.rs`).  `laman` and `henneberg_edges` make a
  *random* graph and then measure the positions they happened to place: there is no statement
  behind any of their numbers and no document that could express one.  They are the generator all
  three language test suites run their property tests over, so they stay in the core — the core
  owns algorithms — but in their own module, out of the case library and off the app's menu, so
  that nothing mistakes a fixture for an example.  The rule that sorts them: a case belongs in
  Solvent when its numbers are things a person would state, and stays a generator when they are
  things a program measured.
- **The document is the Solvent source (`gear.sv`, `syntax.rs`, `program.rs`, `edit.rs`).**  The
  drawing is what elaborating it produces, and drawing is a way to edit it.  So *every* edit is a
  **splice** — a few characters replaced in the text somebody wrote — and **never a reprint**: a
  reprint would flatten a hand-written `component` into the entities it elaborates to, throw away
  every comment and reflow every line, on the first drag.  A gear written as thirty instances of
  one tooth stays written that way while its hundred and twenty points are dragged.
  `edit.rs` is the whole of it: `commit_seeds` (a solve), `reconcile` (a gesture), `remove`,
  `set_dimension`, `add_point`/`add_entity`/`add_relation`, and `mint` for a name.
  Three edit classes, and **the core says which — the front end never guesses**: `Structural`
  (statements added or removed), `Numeric` (only numbers a solve may move, so a compiled plan
  survives — which is what keeps editing a dimension instant), `None`.
  Writeback is one lexical rule: *a seed is writable iff it is inside a `hint(…)` clause, is a
  literal and not an expression, and is reached by exactly one instance path.*  "Inside
  `hint(…)`" is the stronger test the `=`/`==` one was reaching for — it cannot be confused with
  a `param`.
  A seed the source **never wrote** has no span to splice and a solve moves it all the same (a
  radius, a frame's rotor), so `Decl::hint_span` carries both cases at once: where the clause
  *is*, and — an empty span — where it *would go*.  `commit_seeds` splices each seed in place
  when every one it needs has a span, and writes the whole clause at that point when one does
  not.  Appending, rather than leaving it, because otherwise a drawing has a pose its source
  cannot express; a decl seeded by *place* (`hint(at: t)`) is skipped, having no coordinates to
  write.
  `reconcile` and `retext` **apply themselves to the `Elaborated` and do not rebuild the
  drawing** — nothing about the drawing changed, the source is only catching up, and rebuilding
  would invalidate every proxy a half-finished tool is holding.  `retext` re-parses (same
  statements, same ids); `adopt` extends the map onto the statements just written.
  `reconcile` reads only the *append*: entities and constraints are appended to their vectors, so
  past the map's high-water mark is new and anything the map names that is gone was removed.  A
  mutation that **renumbers** is not something it can follow, which is why deletion is
  `edit::remove` and not `io::without`.
- Deletion, copy and paste are one rebuild walk (`io::graft`): every surviving entity is
  renumbered into the destination and every reference follows, and a constraint comes along
  exactly when all its entities did.  `without` keeps what is not deleted, `copy` keeps the
  selection (so a clipboard is an ordinary sketch document), `paste` grafts one sketch onto
  another at an offset.  A new thing that travels with a constraint or an entity — a flag, a
  placement — belongs in `graft`, or the three will disagree.  JSON is now the *export* format,
  derived rather than canonical; `io::dumps` is still what the Rust tests and the benchmarks
  compare against.
- **Every constraint is written as a prefix or an infix operator** (Solvent §9.2); `name(args…)`
  is retired.  `radius(25) c`, `p1 distance(80) p2`, `horizontal l`, `ground p`, `a symmetry(l) b`
  — the word, its one or two operands, and everything else in the parentheses on the word: the
  number, a selector (`side: -1`, `at: start`, `along: x`), a third entity, a pin (`t == 0.4`).
  A *seed* for an owned slot stays the trailing `hint(t: 0.4)`, where every seed is.
  **A pin and a seed are one `OpArg::Slot { key: Name, arg: Arg }`** — the word is the whole of
  the difference between them, and all three of the things that went wrong when they were three
  variants of two fields were the *same* omission.  A pin kept only its parsed number, and
  `value_text` parses no expression, so `t == t0` inside a component pinned the contact at 0 with
  nothing reported; the key was thrown away, so one naming no slot at all filled whichever
  `Param` slot the settled kind had; and `write_written` guessed the name `t`, which is right on
  a spline and was wrong on a curve (`PointOnCurve`'s was `u` until issue #47, item 6, made every
  contact's `t`), so it disagreed with `operator_text`
  about one statement.  The value is **`syntax::Arg` and not a second encoding of it**: `assemble`
  hands it straight on, and `flatten::settle_arg` is the one walk that reads a component's
  parameters out of an argument — a statement carries its arguments twice, as the operator was
  written and in spec order, and both halves are the same type.  `Written::assemble` matches the
  key against the spec; an unknown one is an E040 **at the key's own span** and in the word the
  writer typed, which is why the key is a `Name` and why `assemble` returns a `Result` and
  `settle` carries a `Span` beside its message, in the `(Span, String)` order every other
  fallible path in `program.rs` uses.  `syntax::slot_text` is the one place a slot is spelled,
  read by both printers, as `hint_of` is for the clause around it.
  **The shape is the library's, not a rule imposed on it**: every user-facing kind has one or two
  entity slots, always first in spec order, with `Symmetric` the single three-slot exception the
  parentheses absorb.  Several kinds share a word and that is the saving — **`on` is five,
  `distance` is six, `tangent` is six** — and `horizontal`/`vertical` are two each with the
  *fixity* doing the work, which is exactly the distinction `HorizontalPoints` was added to draw.
  **What a word means is the kinds of its operands, and a name does not carry its kind until
  elaboration** — so the parser resolves *nothing*: it produces a `syntax::Written` (the word, the
  operands, what was in the parentheses) and `program::settle` turns it into a `CKind` plus
  arguments in spec order, through `constraints::infix_op` / `prefix_op` and `Written::assemble`.
  One path, where 0.6 had a longhand and a chain: a **lone infix statement is a one-joint chain**,
  and what a chain adds is the corner — which end two links meet at — that an operator between two
  names cannot know.  `CKind::operator()` is the inverse and is matched exhaustively, so a new
  kind stops the build there; `syntax::operator_text` is the one printer, read by `write_relation`
  and by `io::describe`, so the drawing, the constraint list and the program panel cannot spell
  one constraint three ways.  **The surface word and the wire name are separate**: the registry
  goes on publishing snake_case `name`, which the binding and the JSON export key on, and
  `operator`/`fixity`/`operands` are new information beside it — **the binding is untouched**.
  `ccw`/`cw` keep a call — `Fixity::Call`, every operand in the parentheses: under the general
  rule they would be `a ccw(c) b`, which reorders three points that are symmetric, and the
  predicate is about the *triangle*.
  **The gauges and the orientation predicates are entries of the same table** (issue #47,
  item 5): `CKind::Ground`, `Fix`, `Ccw`, `Cw`, read by the one relation parser (so a class, a
  placement and the chain's lookahead treat them as any word) and settled by the word alone
  (`constraints::gauge_op`, before the operands' kinds are asked — `fix c.r` names a number, a
  `SpecKind::Scalar` slot, and `ccw(a, b, c)` has no operand outside its parentheses).  They
  are **applied, not added**: `program::apply_gauge` marks the parameters fixed or records the
  root choice, `constrain` returns no id, and no `Constraint` the sketch holds is one — so they
  are not in `ALL_KINDS`, the registry never publishes them, the binding is untouched, and
  `CKind::gauge` is the question every table that would reach for a kernel asks first.  A
  `claim` on one is refused (E040): a claim is judged by rank, and a gauge adds no row.
  `edit::reconcile` reads a held parameter's statement off the word (`gauge_key`) and appends
  one built by `program::lift_gauge`; a root choice under a key no triple spells stays the
  `branch(KEY, ±1)` statement (`StmtKind::Branch`).
  Operand order carries meaning now — `arc tangent line` is `TangentArcLine` and `line tangent
  circle` is `TangentLineCircle` — and a name that is also an element keyword can no longer lead
  a statement (`spline_follower.sv`'s spline is `cam`, not `curve`).
- A **chain** (Solvent §6.6) is parser-level sugar and nothing else: `horizontal line bottom(b1,
  b2) -> tangent arc a(center: c) hint(r: r) -> tangent …` desugars in `syntax.rs` into the
  ordinary statements it stands for — a prefix word (any `CKind` whose spec is one entity slot)
  becomes that unary relation, and a joint's word becomes the relation between its two
  neighbours: `tangent` maps per pair of kinds, and any binary `CKind` whose spec is two entity
  slots (`perpendicular`, `equal_length`, `equal_radius`, …) is an infix spelling of itself,
  type-checked against the pair — and no other module knows the construct exists.
  **Threading is a statement, not an inference** (issue #31): the `->` marker on a joint says
  its two links share a boundary point, threaded left-to-right (`p1 → p2`; `start → end`, CCW),
  and its absence says they do not — so `->` alone is the plain corner (`to` is retired into
  it), `-> tangent` is a corner that is also tangent there, and a bare word states the relation
  and welds nothing.  A joint may state *several* relations — `-> equal angle(30deg)`
  states each word as a statement of its own at that corner, and the marker may stand on
  either side of the words or both.  The run ends at the first word that opens the next link
  — an element keyword, or a prefix word standing before one — so fixity sorts a joint's
  infix words from the next link's prefix words with no punctuation.  A doomed word splices
  out where it stands (`Chained::Member`), leaving the corner and the rest; the whole joint
  doomed at once — an entity deletion dooms every relation naming it — has no word left to
  hold the line, so each member carries the joint's written word count and its one-word doom
  (`of`, `fall`, `out_of`) and `edit::doomed_splices` composes that single splice for
  `remove` and `reconcile` alike — counted against the words as *written*, so a word the
  desugarer refused holds its joint's text — and `remove` refuses outright a doom set whose
  splices leave text that no longer parses (a name link dangling between two doomed joints).
  A trailing placement attaches to the line's **one** relation and is refused on a line
  stating several; where none is written the parser records the spot one *would* take
  (`place_span` as an empty span — `hint_span`'s device), so the callout writeback splices
  where the parser said and re-derives nothing, and a line with no spot (it ends in a
  declaration) leaves the callout's pose to the layout.  At a threaded joint the shared point is named by exactly one side (or
  both, in agreement) and fills the boundary field a declared side left out, so a threaded
  `tangent` always desugars to the regular At-form (`TangentArcLine`/`TangentLineCircleAt`, or
  `Parallel` between two lines, which over the shared corner is collinearity) — never the bare
  pair that is rank-deficient at every solution; the `at:` argument is only ever supplied by a
  threaded joint, and an *unthreaded* `tangent` is the plain pair, correct exactly when the two
  are separate.  A chain may mix declarations and names, because each joint states its own
  threading: a link that only names an element offers no list to read or fill, so at a corner
  with one the declared side names the shared point, usually by the existing element's own
  child (`line t(p3, k.start) -> tangent k` — `follow_building` resolves such a child through
  the declaration when the entity's kind builds later).  Only lines and arcs are threaded; a
  circle has no ends — which is the radius-as-Param discussion again — but may stand in a chain
  no marker reaches.  `equal` is the second polymorphic word beside `tangent`
  (`syntax::equal_kind`): a length between lines, a radius between circles or arcs, an error
  between one of each; a name may be declared further down the file or come from a component,
  so `Relation::poly` carries the word and `program::constrain` settles it once the entities
  resolve, **before** reading the spec, since the spec is what the arguments are type-checked
  against.  Each desugared statement keeps an id of its own and a
  span into the chain's text (a chain is several statements from one *line*, where a `cycle` is
  many instances of one *statement*), so writeback, culprits and carets need nothing new.
  **How a statement is spelled is recorded, never sniffed back out of the characters**:
  `Stmt::chained` is a `Chained` (`No`/`Link`/`Prefix`/`Joint`/`Infix`/`Member`/`Stuck`/`Close`) written
  by the desugarer, and `edit::doom_splice` matches on it — a doomed threaded joint steps down
  to the bare corner `->`, a doomed unthreaded joint becomes a statement break (its span grown
  at desugar time over a terminal name-link a break would leave dangling; inside a closed chain
  there is no safe break, which is `Stuck` and refused), a doomed prefix word goes where it
  stands, and a link has *no* splice, which is how deleting one is refused (no splice takes a
  link out and leaves a chain behind).  Reading it back off the text instead would
  rest on "a longhand relation always carries a `(`", which nothing states and a qualified joint
  would quietly break.  Which slots a chain threads through is `EntKind::ends()` in `model.rs`,
  beside the `fields()` table it indexes and exhaustive, so a new kind with ends stops the build
  rather than silently threading the wrong children.  A line ending in a joint — the marker or
  a word — continues its chain onto the next; `-> close` seals a loop back to the first link's
  entry, and a `close` with no marker is an error, since a loop is a thread.
  The colouring asks `opens_link` — the *same* predicate `chain_starts` asks — since written
  twice the two drifted at once on whether `horizontal(bottom)` is a prefix.  Both reach it
  through `past_args`, the lookahead **stepping over the operator's own parentheses**: now that a
  word carries its number, the token after `radius` is `(` and not the element keyword, so
  reading `i + 1` had `radius(25) circle base(…)` open no chain — it fell to `relation()`, whose
  `refr()` swallowed the keyword `circle` as an operand — while the identical form parsed
  mid-chain, where `link` reads the arguments itself.  The same miss left every parenthesised
  infix uncoloured, which is most of the constraints in a document.  It returns an **index**, the
  shape `past_ref` already uses, because a caller asks two things at that position — what word is
  there, and whether the line ends there — and a lookahead answering only the first left
  `p distance(80)` at the end of a line reading as neither.  What it must *not* cost is the guard
  that keeps a bare name plain: a name in an argument list is followed by `,` or `)`, neither a
  word nor the end of a line, so `tangent` used as a point stays untinted.
  It runs per keystroke, so every guard there puts the pointer test before the registry lookup
  (`chain_kind` allocates); reversing those two operands cost 65% of a highlight pass — which is
  also why `tint_word` takes the token slice and computes the lookahead in the one arm that
  reads it, rather than once per token above the match.
  **A block body may end mid-joint** (issue #38): a *threaded* trailing joint at the body's `}`
  threads the chain onto the next copy's first link — every pair in a `cycle` (the wrap
  seals the loop: `cycle 4 { line s -> perpendicular equal }` is the square), all but the last
  in a `repeat`, whose final corner is simply unstated.  The parser records it on the block
  (`Block::joint`, an `OpenJoint`): the word statements are minted at parse through the same
  `joint_relation` — both links' kinds are known, being the body's own declarations, so a
  tangency is the regular At-form across the copy seam — with the right operand spelled
  `next.<leaf>`, which `flatten::lookup`'s own `next` arm resolves per pair under a scope given
  the block's `cyc` whatever the kind (the joint is the *block's* statement, so a `repeat` body
  still may not say `next`).  The weld is the flattener's (`weld`/`fill`): the earlier-built
  side's boundary name is written into the later-built side's slot — `builds_first` generalized
  to (kind, copy, statement), because `follow_building` rightly refuses a reach into an unbuilt
  entity's implicit child, and at a cycle's wrap the earlier side is the *first* copy.  At most
  one boundary slot may name its point (both named are two different points across the seam,
  refused); a name-link boundary is refused until #35 teaches `thread` to read one.  A statement
  inside a braced body ends at the `}` as at a line break (`end_of_stmt`), so the one-line
  spelling reads, and the colouring reads the brace the same way.  `square.sv` and `ngon.sv`
  (a component taking `n`) are the cases; `tests/open_joint.rs` is the gate.
  `tests/chain.rs` holds the gate: the chain spelling of `rect_fillets` states exactly what the
  shipped longhand does.
- A statement expanded by `flatten` **keeps the id of the statement it came from**: a `cycle` of
  thirty makes thirty things from one line, and the line is what a span points at, a caret lands
  on and a splice edits.  What tells the thirty apart is the `path` every `Site` carries.  Minting
  an id per copy made each look like a statement of its own, so every consumer that turns an id
  back into source found nothing there — and the multiplicity a fresh id hid is exactly what
  `commit_seeds` needs to see.  `Program::stmts` walks into block bodies for the same reason;
  whether a statement is one the *root* may splice on its own is a different question, asked
  against `root().body` (`edit::in_root`).
- **`ring` is refused by name** (issue #47, item 3): the parser reports the word once, saying
  `cycle N { … }` is the spelling whose copies are congruent by the numbers each is given, and
  consumes the block (`skip_block`) so its body is not read as loose lines.  It had been
  unrolled into exactly that cycle plus three rules and a warning (W112 on every run, E021,
  E022, a mandatory `about`) guarding a symmetry no solve held; the spec keeps §12.3–12.5 as
  the target and the word comes back with the fundamental-domain solve, for which
  `tests/ring.rs` is the gate.  A diagnostic that carries its own code (E041) goes through
  `Expansion::coded`, since the plain `errors` are sorted into E101/E103 by message.
  Likewise an expression's failure is an `expr::ExprError` with a `Fault`
  — `Dimension` (E103, §3.3: `distance(45deg)` is an error, never a coercion), `ClaimFree`
  (E040, §9.7) or `Uncomputable` (W110, the last number stands) — because three different things
  were one warning.  A point-to-point distance and a radius are `CKind::magnitude`, and a
  negative literal in one is E040 where it is written: the kernel would square the sign away.
- Decomposition maps constraints onto F–H elements in `cgraph::build`; a new constraint type is
  either an edge (PP/PL), a direction relation, or `unsupported` (numeric residual).  Merge
  decisions use generic-rank at witness poses; chirality of PPP merges is the triangle
  orientation sign from the current sketch.
- Replays are warm-started on the current geometry (leaves re-derived each frame), so the root a
  sketch is on is "nearest the identity"; alternatives are applied by writing geometry, not by
  caching transforms.
- **A solid is a term, and a verb is a noun that has not been given a name** (Solvent §6.9,
  `solid.rs`; issue #48, items 9 and 10).  What makes a CAD feature tree imperative is not that it
  is ordered but that it is *stateful*: step *n* acts on "the body as of step *n − 1*", an
  anonymous thing, and names faces by the order they were made in.  Solvent names everything —
  which is why `port` was retired — so a solid is its **stock, plus everything `on` it, minus
  everything `through` it**, over primitives that are faces swept.  Both groups are *sets*, so
  the statements filling them may be written anywhere in any order (P2), and the order that does
  exist lives inside one term over names, exactly as it lives inside `h = w / 2`.  A design that
  needs the other order (a pocket with a boss standing in it) **names the intermediate**, which
  is honest: there are two things there.
  **Nothing three-dimensional is ever solved for.**  `EntKind::Face` and `EntKind::Solid` own no
  `Param` — `entity_params` returns nothing for either, so no column of the Jacobian is one —
  and every extent is an `Extent`: the text a person wrote and the number the *flattener* settled
  it to, the `fold:` bargain.  The strata run one way with no edge back: the sketch solves, the
  depths are worked out, the terms are ordered (`solid::resolve`), the outputs are read.  No
  `SpecKind` takes a face or a solid, so a 2D constraint cannot name one — the stratification is
  a *type* fact rather than a rule anybody has to remember.  Built after every other kind
  including `Curve` (`program::solids`), since a face is written over edges and a solid over
  faces and solids.
  **The kernel is the term and nothing is built or stored**: a view, a section, a volume, a mesh
  and a clearance are all questions asked of `Csg` by classification (Requicha & Voelcker's
  boundary evaluation), memoised against `solid::reads` — every scalar of every edge the term
  reaches, each plane's pose and basis, every extent, and `unit` — which is `curve_polyline`'s
  bargain.  There is no B-rep, which is why the crate still has no dependency, and STEP is
  therefore deferred while STL is not.
  Two findings are load-bearing and each is written where the rule is.  **The classifier must
  read the facets the candidates are cut from**: classify against the true circle while cutting
  facets and every facet centroid of a bore's wall sits inside the true bore by the sagitta, both
  its samples read *outside*, and the wall silently vanishes.  And **a BSP prunes what a split
  loop cannot** (`csg.rs`): cutting every facet by every plane that reaches it is exact on a
  square hole, but the pieces go as the *square* of the facets — a block's cap against a
  six-hundred-facet bore is the arrangement of six hundred lines, twenty-five thousand cells for
  the six hundred the drawing needs.  Descending a tree, a piece wholly in front of a node is
  decided and goes no further; same planes, same answer, and it stops asking once it knows.
  There is **no coordinate snap grid**: a grid fine enough to leave the drawing's numbers alone
  is finer than the noise it was meant to collapse, and one coarse enough to collapse the noise
  moves every vertex (a block sixty across came out `72000.0036`).  What keeps the classifier out
  of the gap is `EPS`, four orders coarser than the solve's own noise, and `same_plane`, which
  reads two near-coincident planes as one and never splits a facet by its own plane.
  **A face is a loop and a loop is walked in order**, so an arc in one is entered by whichever end
  the walk arrives at — and *how far* it goes is the arc's own fact while *which way* is the
  walk's.  Entered by its `end` it is walked backwards over the same stretch of circle, never
  forwards over the rest of it: normalising `a0 - a1` gave `TAU - extent` there, so a channel
  between two concentric arcs — the V-twin plate's plenum, and the shape any annular duct is —
  closed as a bowtie of twelve times its area and meshed with seventy-six unpaired edges.  No
  drawing in the corpus had entered an arc by its end until a part was written as a solid, which
  is the whole reason the migration found it.
  `tests/solid.rs` is the gate and it checks against **arithmetic, not against a second kernel**:
  a block is `w·h·d` exactly, a bore takes exactly the polygon it is faceted into, a flush bore
  and one drilled past are one solid, a boss adds and the shared face is counted once, a
  revolution is Pappus.  Two bugs only that could have caught: a prism's caps wound against their
  declared normals (invisible wherever a cap sits on the origin and contributes no flux), and a
  revolution's walls facing inward, which reads as a negative volume and nothing else.
  `tests/solid_lang.rs` holds the language, `tests/derived.rs` the pictures.
- **The sheet is a report** (Solvent §6.12, `hidden::generated`; issue #48, item 10).
  `dimensions(body) in views.right` asks for the callouts that *follow from the object*: the
  part's overall extents in that view, and the diameter of every round feature the view sees
  square on.  They go through `callout::layout`'s own pen and lanes, which is the whole of what
  "laid out by the engine that already exists" means — a generated dimension stands off a stated
  one because neither knows the other is different — and their ids start at `callout::GENERATED`,
  past any constraint, so a front end resolving one back to a statement finds nothing there.
  That is the truth: it is a reading of the drawing, not a statement in it, and it reads the
  *solved* pose.
  **The boundary is the feature.**  What a machine can decide is what the object says; which
  datum a stack is measured from, which fit is critical, what is a reference and what controls —
  those are the design, and a machine that chose them would be guessing.  A sheet states the rest
  as it always did.  `tests/sheet.rs` asserts both halves: the three it makes, and that it makes
  no fourth.
- **A claim about a solid is judged, and can never act** (Solvent §9.8, `clear.rs`; issue #48,
  items 6 and 7).  §9.7's bargain one stratum out: `disc clear(2mm) cyl`, `head fits(0.15mm)
  trap`, `piston inside bore` compile **no row**, so a solid claim cannot move geometry, take a
  freedom or paint a sketch Over — checked by `tests/solid_claim.rs`, which adds two and asserts
  the parameter count, the equation count and the DOF are all unchanged.  The three words are in
  `constraints::OPERATORS` so a statement reads the way every other statement does, and they
  settle to no `CKind` because a `CKind` is a thing with a kernel; `constraints::solid_word` is
  where the word is read, asked before the spec is, the way `gauge_op` is.
  **What a reader is owed is the measurement**, not a yes or no — `clear(4mm)` failing by a
  millimetre and by a metre are different drawings — and the verdict carries the *sagitta* beside
  it: the faceting is honest about being faceting, and a claim decided within it comes back
  **undecided**, which is a third answer and not a failure.  The measurement is exact on the
  faceted solids: the implicit min/max reading of a term is only a lower bound for a difference,
  so it culls, and the answer is piece against piece with the boxes doing the work.
  **`claim over crank.theta in (0deg, 360deg) { … }`** (item 6) judges its body as the drawing
  runs along one of its own **free variables** — a `param` is a number the document already
  fixed, and sweeping a constant is not a question — reporting the *worst* pose, since a fact
  about a cycle is not a fact about one angle.  It is **sampling** at `SWEEP_STEPS` and says so.
  Two orderings are load-bearing.  The claims are read in a pass of their own **after phase 4**,
  because a free variable is what `expr::evaluate` allocates and a claim read beside the solids
  would find `theta` declared nowhere.  And the interval is read **in the unknown's own units** —
  `(0deg, 360deg)` is radians to the kernels and `(0mm, 20mm)` is a length unchanged — where
  converting everything as an angle made a sweep of millimetres sixty times too small and
  reported a claim that held over almost none of its interval.
- **A solid leaves as glTF, and that is the format this kernel's data already is** (`gltf.rs`).
  STL carries triangles and nothing else — no face named, no unit recorded, the grouping thrown
  away at the door.  STEP carries both and is a *boundary representation*, which this kernel
  deliberately is not.  glTF is positions, normals and a named group per face, which is precisely
  `mesh::Mesh`, so the mapping is nearly an identity — and its container is a twelve-byte header
  and two chunks, one of them JSON that `json.rs` already writes, so it needs no ZIP and no
  dependency where a `.3mf` would have wanted both.
  **Every face of every object is a named node**, so a viewer's outliner is the document's own
  tree of names — and the path is carried **twice**, as `name` for a person and in `extras` for a
  program, because a loader may sanitise one: three.js strips the dots out of `body.bore.wall`,
  its own animation paths being written with them, and passes `extras` through untouched as
  `userData`.  Verified by loading the file in three.js, which is the only test of an
  interchange format worth having.
  **Metres, because the spec says so normatively**: a document naming `mm` is scaled on the way
  out so a forty-millimetre part opens as one, with its own unit in `asset.extras` so the scaling
  loses nothing; one naming no unit is written as it stands.  What is exported is the document's
  *objects* (`overview::objects`, now public) — a bore is a hole in a part and not a part beside
  it — and glTF holds a scene, so unlike an STL it need not be told which part of an assembly to
  be.  `solventc --gltf`, `File ▸ Export solid (glTF)`, and `File ▸ Export solid (STL)` beside it
  for a printer.
- **A mesh is welded, and grouped by face** (`mesh.rs`) — the two things a boundary evaluation
  does not give on its own, and between them the whole of what a mesh was said to need a B-rep
  for.  A viewer and a slicer both take *triangles*; what they want of them is that the edges pair
  up and that the triangles say which face they belong to.
  **Welding** is a T-junction fix and not a vertex merge.  Neighbouring facets are cut by
  different planes, so one leaves a vertex partway along an edge the other still spans whole —
  and no amount of merging fixes that, since the neighbour has no vertex there to merge with.
  `weld` finds the vertices lying on an edge's interior and puts them in it: the V-twin cylinder
  went from 4,474 unpaired directed edges of 141,468 to **zero**, with the volume unchanged to
  the digit.  Two sizes are independent and conflating them cost a hundredfold: a hash **cell**
  need only be at least the weld tolerance, so it is sized to the object (`scale / 128`) while
  the tolerance stays at `1e-9` — sized at the tolerance, an edge walked its own length in cells.
  And `triangles` fans **from a corner where a piece has only corners, and from the centroid where
  it does not**: a vertex fan covers every boundary edge except the sub-edges of the two it stands
  on, which come out zero-area, and dropping those (which a printer's validator wants) re-opens
  exactly the T-junctions the weld had just closed.  Most pieces the weld never touched, so the
  test is per piece and is the condition itself — has this polygon a vertex that is not a corner?
  Always fanning from the centroid cost 78% more triangles than the mesh needed.
  **Grouping** is `mesh::grouped`: the same triangles in face-path order, with a normal per
  vertex — the facet's own where the face is flat, and the average of the facets meeting there
  where the face is `smooth`, so a bore's wall shades round and the rim where it meets a cap
  stays a corner.  Across the ABI as **buffers plus a small table** (`gcs_solid_mesh`,
  `gcs_solid_normals`, `gcs_solid_faces_json`), the division `gcs_entity_params` already draws;
  all three read one memoised `Want::Mesh`.  `Document.solids()` names them, since a solid has no
  proxy — it is on no sheet and owns no parameter.  `tests/mesh.rs` is the gate and asserts the
  *before* as well as the after, so a weld that stopped working could not pass.
- **A mesh is cut to the object and a volume to the report** (`solid::mesh_unit`, `MESH_SAGITTA`).
  Two requirements, and giving them one number cost an order of magnitude: `REPORT_UNIT` is
  chosen so a *volume* is good to one part in ten thousand, and a mesh inheriting it cut the
  V-twin cylinder's 16 mm bore into 257 flats — a six ten-thousandths of a millimetre sagitta,
  a hundred times under what a printer resolves — for 98,000 triangles where 8,000 are
  indistinguishable.  A mesh is cut to a sagitta that fraction of the *solid's own diagonal*,
  which is scale-free and so says the same thing in millimetres and in inches, and is asked of
  the primitives' boxes rather than an evaluated boundary: paying for a fine boundary to decide
  how fine a boundary to build is the tail wagging the dog.  The cylinder's STL went from
  97,772 triangles and 2.3 MB to 8,092 and 395 KB, and from 3.1 s to 0.37 s, still with zero
  unpaired edges.  `gcs_solid_mesh_unit` publishes the number so a viewer may take it or pass its
  own — and **a unit at or below zero *is* that choice**, resolved at the one seam every
  evaluation goes through (`Sketch::cut_unit`, read by `solid_boundary`, `solid_edges` and
  `solid_mesh`).  It had been written into `gcs_solid_glb` and `gcs_solid_stl` and nowhere else,
  so a caller that asked for a *mesh* at 0 got a sagitta of zero — an arc cut into an unbounded
  number of facets, which is not a coarse answer or a slow one but no answer at all: it took the
  browser tab with it.  `tests/mesh.rs` asks it of all three walks, since one of the three having
  the rule is exactly the state that was wrong.
  **The glass box asks with 0, and that is why zooming is free.**  `unit` is the world length of
  one screen pixel — the right refinement for strokes on a page being looked at, and the wrong
  one for a scene handed to a renderer with a camera of its own.  Cut by it, every wheel tick
  re-evaluated the term (158 ms for the V-twin cylinder's edges and 332 ms for its mesh,
  natively, and finer without bound as you zoomed *in*, since a pixel is a smaller world length
  the closer you get), while an orbit moved only the camera — which is exactly what one felt
  like against the other.  `overview::scene3d` asks the same way (`SCENE_PX` for its drawn
  polylines, `0` for the object's edges so they share the mesh's one evaluation), so nothing in
  the box is a function of the zoom and `Box3D`'s rebuild key does not mention it.
- **A face is one loop and a solid's faces are named by path** (§6.8).  A face is a closed loop of
  edges the drawing already has, on the one plane every point of every edge agrees about — *read*
  off the memberships and never written on the face, so a face inside `in swing { … }` is on the
  plane the block stamped.  There are **no holes**: a hole is a solid `through` the body, and that
  is the body rule saying it already, one construct fewer.  An `in` block leaves a face and a
  solid alone the way it leaves a datum alone, but for a different reason — they bear no points,
  *and* they are written over the geometry the block just stamped, so refusing them would put the
  design and the solid it is a section of in two different blocks.
  A boolean **never renames**, so the topological-naming problem every history-based kernel has
  does not arise: `block.near`, `block.far`, `block.side_l` (a prism), `bore.axis`, `bore.start`
  (a revolution), and a body's through its operands — `body.bore.wall`, `part.base.pocket.floor`.
  A name whose face a boolean ate is a *fact the report carries* (its surviving area, or nothing),
  never an error: the name was true of the operand and remains so.
  E080 a face that is not a loop on one plane, E081 a revolution's axis, E082 a face a body no
  longer has, E083 a stack that contradicts itself, E084 a section cut across its own view.
- **`from:` says which plane a plane is derived from, and the clause beside it says how**
  (§6.7).  `fold:` turns it; **`offset:` stands it off along the normal**, which is what a stack
  of parts is written in.  0.10's rule that an omitted fold meant `fold: 0deg` is withdrawn — no
  document in the corpus used it, and a plane naming another and folding nothing most plainly
  says *the same plane, moved*.  `plane::Basis` therefore carries the point its origin stands at,
  and **only along the normal**: the fold line is perpendicular to both normals, so `d·o = 0` and
  `fold_line` — the whole of what `Project` reads — cannot see the move.  Every plane in every
  document written before solids has `o = 0`, which is why no existing test changed.
- **Where a part stands is what it bears against** (Solvent §6.10, `program::place`; issue #48,
  item 8).  `cylB.block.far against plate.body.near` says the two faces touch, and the offset of
  the plane cylinder B is drawn in *follows* — which is what `zA = fwA + D / 2` and the chain of
  subtractions under it were, three files keeping one number in step by hand.  Two faces in
  contact are at the same point along the normal they share, so `offset(P) = offset(Q) + ord(G) −
  ord(F)`: one equation a statement, worked out in dependency order the way `expr::evaluate`
  works out a dimension, and nothing here is solved for either.
  **A mate is between the caps a sweep makes**: a side face is not at one ordinate — it runs the
  whole depth — and a revolution's walls are not flat, so neither is something a stack can bear
  on (E082, with the reason).  A *placed* plane is one written `from: P` with neither `fold:` nor
  `offset:`, recorded in `Sketch::placed_planes` where the attitude as **written** is still in
  hand; exactly one `against` places one, and none or two is E083.
  The delta is computed **when the mate is applied and not when it is collected**: a washer
  between two parts stands on the first before the second stands on it, and an offset worked out
  up front reads a zero the walk was about to fill in.
  `hardware.Groove` is item 5 in the same commit: the O-ring rule — 10–20% squeeze on the ring's
  section, a groove a third wider than it — stated once in `hardware` and cut by a component that
  reads it, so `dims.sv` derives `grooveb` and `groovew` where it used to type 12.9 and 2.4.
  **A component contributes a `through` to a body it was handed**, which is the body rule being a
  set and not a sequence: the feature owns the void it cuts.  `Loc` moved to `std` on the way,
  having been written out in two project files.
- **A part carries no views; a sheet asks for them** (§6.11, `hidden.rs`).  `view(body) in
  views.right` and `section(body, at: swing) in views.front` are *outputs*: no `Int` draw flag, no
  `repeat draw_side { … }`, no second copy of the geometry and no `project` to keep in step.
  Three draughtsman's rules, each written where it is enforced.  **A corner is drawn and a
  tessellation seam is not** — a `smooth` seam is drawn only where it is a *silhouette*, which is
  the two lines a cylinder is drawn as and not the sixty-four its facets would give.  **What the
  material covers is dashed, not dropped** — the eye's ray is classified against the term and the
  piece carries `.hidden`, the class every part sheet already styles.  And **coincident page lines
  are drawn once, visible winning** — which has to be an *interval* rule and not a segment one: a
  block seen square on puts its far corners exactly behind its near ones and the two agree segment
  for segment, but a cylinder's rim seen edge-on folds in half onto its own image and the visible
  and hidden halves split at different places, so nothing matches end to end.  Every stroke is laid
  on the line it belongs to, the visible stretches are unioned and the hidden are what is left
  over; drawn any other way a solid outline gets a dashed one laid under it, which at a printer's
  resolution reads as neither.
  Everything comes back in **page coordinates** through `plane::on_page` (written beside
  `in_view` for that function's own reason), so a derived view sits at its plane's own origin and
  rotor — exactly where a hand-drawn one tied by `project` would, which is `tests/derived.rs`'s
  strongest gate: extrude a face along its own normal, look at it square on, and the outline that
  comes back is the outline that was drawn.  **The core projects and the front end strokes**, the
  seam `callout.rs` sits on: `hidden::layout` resolves the ink through the sheet, `svg::render`
  and `paint.ts` stroke what they are handed, and neither owns a line of 3D arithmetic or a rule
  about what a hidden line looks like.  **Every part of the V-twin is written this way** — `vtwin/cylinder.sv`, `piston.sv`, `disc.sv`,
  `flywheel.sv`, `throttle.sv` and the plate in `frame.sv` — one section and the solid it is a
  section of, with the other two views asked for.  The cylinder went from 144 lines and 12 formals
  to 119 and 6 and its sheet from 69 points and 50 lines to 30 and 22; the plate's sheet from 166
  and 141 to 89 and 77, the piston's from 56 to 26, the disc's 59 to 20, the throttle's 63 to 22.
  **Where a part's turned features are is where its section has to be**, and that is the one rule
  the migration keeps teaching: a turn about a line lying in the section puts what it makes *on*
  that plane whatever else is written, so the crank disc's section is its mid-plane because the
  set screw runs through it, and the plate's is its mid-plane because the plenum, the boss, the
  vents and the coupling's hole are all centred there — which `tp`'s own comment said before any
  of this ("thick enough to carry the plenum on its mid-plane").
  Two things do not survive the crossing and are named where they are: a **hex pocket about a
  radial line** (the set screw's nut, `parts.Grub`) is neither a sweep along the plane's normal
  nor a turn about a line in it, so it stays four hidden lines a printer reads; and a feature the
  drawing carries as a *centreline* (the plate's exhaust vents) has to be told how wide it is,
  which is `wch`, the channel width the plenum and the passage already use.
- The **overview** (`overview.rs`) is the drawing folded back into the glass box it was unfolded
  from: each view standing on its own plane in space, and the object the views are *of*
  reconstructed between them.  **Nothing is solved for and nothing is stored** — a point drawn in
  view P has view coordinates `plane::in_view` (the same reading `project`'s residual takes, and
  the same function, so the two cannot drift), sits in space at `a·u_P + b·v_P` (`Basis::lift`),
  and a corner tied by `project` into two non-parallel views is over-determined and exact: four
  rows in three unknowns, consistent *because* the projection holds.  "Non-parallel" is
  `overview::RCOND`, about a degree: past that the rank test passes and the residual is amplified
  by 1/σ₃, which flings a corner across the page — `validate` refuses only the exactly-parallel
  pair, and an explicit `u:`/`v:` basis can be as close as it likes.  A corner is a **pair of
  images and never a transitive class** — in the front view the near and far ends of a vertical
  edge coincide, so merging `Ff project Fa` with `Ff project F2a` collapses the object along every
  edge that runs away from a view — and its images are **ordered by plane**, never by the order
  the statement named them, which is what lets the edge walk compare image to image.  An object
  edge is one both views draw, deduped by its **3D segment to a tolerance** (`SAME_POINT`), since
  one stroke in one view is two edges of the part and two pairs of views agree on a corner only
  to the solve.  Where a point *stands* is `overview::view_of` — its membership, or, for a datum's
  own origin and `toward` (members of nothing, since they place the view rather than being drawn
  in it), that plane: every origin is the one shared origin, and `Insert ▸ Three views` stamps
  nothing.  A line stands with each end where that end is, so a projector between two views is
  neither a stray stroke on the page nor anyone's.
  The **core projects and the front end strokes**, the seam `callout.rs` sits on: the scene comes
  out in 2D world coordinates already orbited and flattened, so `camera.ts` stays the whole of the
  app's linear algebra and no 3D arithmetic exists above the ABI.  `Part` names what an item is
  (`Face`/`Axis`/`Drawn`/`Solid`) and `Item::in_plane` names the view it belongs to, so a front end
  never works out a second time and in its own words which view a thing is in; `overview::drawable`
  is the per-kind polyline walk `svg::entity` and `paint.ts` each make for their own output, said
  once as geometry, refined to `curve::flatness`.
  **Every plane is a pane** — drawn in or not, because a view is a place to draw and one that did
  not show until something was in it could not be gone to — and `overview::pane` is the *one* rule
  for how far one reaches, so its face and its axes cannot disagree: the geometry standing on it
  and its origin, grown a little, and never thinner than `LEAST_SIDE` either way (a view holding
  only its origin, or points along one line, is a pane like any other).  Its **x and y run right
  across it**, crossing at the origin: the sheet's own axes folded up, since a pane is a little
  sheet and what makes it read as one is its axes.  A screen-constant tick would be truer to a
  datum glyph and is what this was; at the size a box is looked at it disappeared into whatever
  the view had drawn near its corner.
  The mode is **read-only, and that is two gates**: the pointer, once, in `gesture::onPointerDown`
  (a press picks and then orbits, nothing else), and `SketchView.mayEdit` at every verb that
  reaches the document *without* a pointer — `apply`, the constraints bar, paste, the class and
  fix toggles, a dimension, a branch flip — which says why when it refuses.  `setOverview`
  abandons whatever is in flight (a gesture, a carried dimension, an animation) before it refits,
  since the fit changes what a screen position means.  Hovering a pane's **edge** bolds it —
  never its interior, the rule everything on this canvas is picked by — but a *click* on one
  selects nothing: selecting a plane arms it as the view the next thing is drawn in, and that is
  the double-click's meaning.  A double-click on anything belonging to a view (its pane, its axes,
  its geometry) leaves the box and arms that plane without selecting it, so no constraints window
  opens over the drawing you came to make.  A drag orbits with the y inverted: the pointer pushes
  the box about as if it were held.  The scene is memoised against the drawing, the zoom and the
  orbit (`SketchView.scene`), because a pointer move asks for it twice and the box is read-only.
  **The box shows the objects, not the features they are made of** — a solid is the object exactly
  when nothing else is made of it (`overview::objects`).  A part is a stock, the holes cut out of
  it, and the body that is the term over them: four names for one thing and three voids, and
  drawn whole each void was an object in its own right, hidden-line tested against *itself*, so a
  bore floated in front of the face it is drilled through.
  **The box is drawn by three.js** (`app/box3d.ts`), and it is the one place in the app where a
  renderer of somebody else's is used.  The reason is a single problem the 2D canvas could not
  solve: a painter's order compares *centroids*, and ordering is only ever right between polygons
  that do not overlap in the picture — which a part with a bore through it is exactly not.  A
  depth buffer settles it per pixel.  **The seam did not move**: the core still says what is in
  the box and where (`overview::scene3d` — the same walk `scene` is, handing over the panes, their
  axes and the views' geometry in *space* rather than flattened — and `mesh::grouped` for the
  object), and this file turns that into three.js objects and works out no coordinate of its own.
  Its camera is set from `v.orbit` and `v.camera` through the same `overview::eye` basis the core
  flattens with, which is what keeps a click landing on the thing under the cursor; the WebGL
  canvas sits *under* the 2D one with `pointer-events: none`, so **not one line of gesture code
  changed** — the picking, the hover and the double-click still run against `overview::scene`'s
  flat projection, which is now what that projection is *for* (and is never asked `shaded`).
  Selection and the hovered pane are a **material write per frame** and never a rebuild, so a
  pointer move never re-uploads a mesh; the ink rule itself is `paint.ts`'s `chromeOf`, so a line
  picked on the sheet and the same line in the box light the same colour.
  Of the object, `scene3d` carries the **creases and nothing else** — a `smooth` seam is dropped
  outright rather than tested against an eye direction, because a silhouette is a fact about a
  viewpoint and this scene has none: what draws the round of a cylinder here is the shading, and
  what is left for a line to say is where the surface actually breaks.  Nothing is hidden-line
  removed either.
  **`⇧⌘B` shows the solid's surfaces** as well as its creases — view state like the orbit, never
  saved or undone — and it is **on by default**, which it was not while the box was strokes on a
  flat canvas: there a surface cost the boundary of every solid *and* the painter's order above.
  Off is still worth having, a wireframe being how you see the far side of a part.  The flat
  shaded path (`Part::Shell`, `overview::shell`, `Item.shade`) is still what `overview_json`
  offers a front end without a depth buffer, and its rule is that **a piece is kept when nothing
  stands between it and the eye**, a ray from its centroid against every other piece, with a face
  *partly* covered still all-or-nothing — the honest limit of a schematic with no depth buffer.
    **Where the document has a solid, that is the object and there is nothing to reconstruct**:
  the box shows the term's own edges, classified against the orbit's eye rather than against a
  view's normal, so a box of a part shows what a part *is* and not what two pictures of it happen
  to agree about.  The corner-and-edge walk is skipped outright then; it is what a drawing with
  no solid in it can still be shown as — several views of an object nothing in the document names.
  The mode, the orbit and the hovered pane are *view state*, `underlay`'s rule: never saved,
  exported, solved or undone — and **the box exists only where there are views**: a document
  with no plane has nothing to fold, so a load or an edit that leaves none returns to the sheet
  (`swap`) and ⌘B on one says why it stays, since shown in the box such a drawing is a tilted,
  read-only, empty-looking sheet on which every tool click silently does nothing.
- Slow tests are gated by `#[ignore]` (cargo).
- **Test time is measured, not guessed, and three findings decide the shape of `make test`**
  (each with its numbers where the rule is written): `cargo test --release` fat-LTO-linked the
  engine once per test binary (`rust/Cargo.toml`, `[profile.test]`); macOS assesses every fresh
  executable on its first launch and keeps unpacked debuginfo objects forever in
  `target/debug/deps`, so the core suite is one binary built without debuginfo
  (`gcs-core/tests/main.rs`); and the two released links overlap only inside one cargo
  invocation (`Makefile`, `release`).  A `target/debug/deps` that has grown to hundreds of
  thousands of files makes every binary in it take seconds to launch — `cargo clean --profile
  dev` is the cure, and `debug = 0` is what stops it recurring.
- Benchmark on a quiet machine (`uptime`); this box often has a JVM indexer at 300% CPU.  The
  native half is `rust/gcs-core/src/bin/bench.rs` (`cargo run --release -p gcs-core --bin bench`)
  and the wasm half is `npm run bench`; `make bench` runs both.  Wall-clock medians and nothing
  else — a benchmark harness would be the core's first dependency.
- The front end is two layers and they are kept orthogonal.  *Geometry* is the core's, and it
  is asked in world coordinates: what a click picks is `model::pick` (which measures what is
  *drawn* — a line's segment, an arc's sweep, the curve itself — as against `point_to`, which
  measures the idealised entity a dimension means, an infinite line and a whole circle), a
  callout is laid out and hit-tested by `callout.rs`, a curve is tessellated by `curve.rs`.
  *Linear algebra* is the front end's, and the whole of it is `app/camera.ts`: a similarity —
  uniform scale, translation, and the flip that comes of the canvas putting y downwards — so it
  carries lengths and angles faithfully, which is exactly what lets every geometric question be
  asked out where the geometry is.  A tolerance therefore travels as a world length
  (`PICK_PX * unit`, the same `unit` the callouts are sized through) and never as pixels.
  Nothing outside `camera.ts` multiplies by `scale` or writes a minus sign in front of a y, and
  nothing in `app/` measures a distance to an entity itself.
- Dimension callouts (`callout.rs`) are geometry, so the whole figure — extension lines, heads,
  radial leaders, angular arcs, the label's box and the hit test — is laid out in the core and
  the front end only strokes what it is handed.  Sizes are screen-constant through `unit`, the
  world length of one screen pixel — as is the pick tolerance, so a front end never converts.
  Where a callout sits is a *placement*: two numbers in a frame that follows the geometry,
  automatic until someone drags it and then `Sketch.placements` document state, **saved on the
  statement it qualifies** — `at (t, r)` after the dimension in Solvent, `"place"` inside the
  constraint object in JSON.  **It stays there, and the sheet takes everything it shares**
  (issue #16, spec §13.1): a class is a rule many statements share and a placement is a fact
  about one, so `style .dimension` / `.reference` / `.extension` own the ink, the weight and the
  dash — `paint.ts` asks `styleNamed` three times a repaint and holds no callout ink of its own
  — and the statement keeps the one pair of numbers that is about that statement alone.  The
  alternatives all fail a rule §13.1 already states: a position or an index fails *silently*, a
  selector on type-and-arguments cannot always name one dimension (the app deliberately never
  dedups them), and a minted id collides the first time somebody copies a block of text.  Naming
  the dimension is the one that works, and costs every dimension anyone has dragged a name
  nobody asked for.  A placement whose dimension is gone is gone with it — `Sketch::remove`
  drops it, and in the source it rides on the statement the splice takes.  Never by position in a list and never by entity index (Solvent
  §13.1): both follow the position rather than the thing, and both fail silently — a callout
  reappears on another dimension, a recorded root choice goes inert while the document still
  carries it.  `io::from_json` still *reads* the old position-keyed `placements` table, so an
  older document loads; it never writes one.  The number a dimension states
  comes from `io::dimension_text`, so the drawing and the constraint list cannot print it
  differently — and a bare number is read through `io::reading`, one constant
  (`READING_SIG`, six digits) for the callout, `arg_text` and `describe` alike, since
  `syntax::num` is the *source* printer and prints every digit a double has.  A written literal
  that names its unit (`60deg`) is drawn as written and given no second sign
  (`expr::names_unit`).  A callout is painted *over* the geometry, so `callout::pick` also owns what
  outranks it: a point within the same tolerance beats the figure's lines — a radius runs its
  leader out of the centre it measures from, and the one point a circle has has to stay
  clickable once it is dimensioned — but not the number's own box, which is filled solid, and
  picking through a thing the drawing covers up would be a lie about the drawing.  The rule is
  the core's because the figure is, so a front end asks once and every front end agrees.
- One straight dimension figure, `Pen::linear`, draws them all: it measures along a *given*
  direction, puts a head where each point falls on that line and runs an extension line out to
  each point from wherever it is.  `Pen::aligned` is the case where the direction is the pair's
  own (a length: both extension lines come out the same); a run or a rise passes a page axis
  instead, and everything else follows.
- A dimension between two points is three dimensions — `Distance`, `HorizontalDistance`,
  `VerticalDistance` — and which one is *stated by where the number is put*.
  `callout::pair_dimension` picks the nearest of the three lines a dimension line could lie
  along (the pair's own, the page's x, the page's y) to the direction from the middle of the
  pair out to the placement.  Nearest, so the borders are the bisectors and there is no
  threshold; a tie goes to the length.  It lives beside the frames because it has to agree
  with the figure that then gets drawn.  The front end asks (`gcs_dimension_pair_kind` → a
  registry index) and swaps the constraint for another as the pointer moves — see
  `SketchView.startDimension`, which is also the seam where a dimension is written at all: it
  is stated at once, at what it measures, and its number is edited on the drawing where it
  will be read.  Nothing reaches the undo stack until the number is accepted, and Escape takes
  the constraint back out.  *Nothing is solved while it is being carried* either — `afterEdit`
  skips the solve while `liveDim.placing`, so stating it, swapping it for another kind and
  moving it about leave the geometry alone; `placeDimension` (the click that plants it, and
  the release that ends a later drag of it) is the one solve, and the editor stays open past
  it because where it sits and what it says are settled separately.
  The run and the rise are signed from the first point to the second
  (`(qx - px) - d`, a constant Jacobian — the best-conditioned row there is, and still so with
  the two points one above the other, which is the pose someone reaches for a run in).  So
  they do not commute, and a front end orders the pair to make the number read positive.  In
  `cgraph` they are `unsupported` on purpose: the cluster vocabulary has no element for the
  line they are really measured from.
- A dimension's number may be an *expression* (`expr.rs`): `Arg::Expr { text, value }` in a
  `Length`/`Angle` slot, where `value` is what the kernels read (arg units — radians for an
  angle) and `text` is what a person wrote, in the units they read (degrees).  `w = 80` names
  its value, `h = w / 2` reads one; the names make a graph over the document's dimensions and
  `expr::evaluate` is a Kahn walk of it (earliest in the document first among the ready ones),
  writing every value and reporting, per expression, its name, deps and error — a name defined
  twice, one nothing defines, a cycle, a non-number.  An expression that cannot be computed
  keeps its last number, so the solver always has a constant.  Trigonometry is in degrees.
  A number may be written as a mixed fraction — `3 1/2` is three and a half, the way a drawing
  writes it — which the tokenizer folds into one `Num`.  The space is what tells the readings
  apart, so `31/2` and a bare `1/2` are still divisions, and the fraction itself is written
  tight.  `expr::literal` deliberately does *not* claim it, so it is kept as text with the value
  it came to: `expr::notation` says that text is a number written a particular way rather than a
  computation, so `arg_text` — the constraint list — prints it as written where a *formula* there
  prints the text and what it came to (`h = w * 2 = 80`).  A **callout carries the expression**:
  `io::dimension_text` draws every written dimension as written, since `h = w / 2` and `3 1/8`
  each tell a reader what 40 and 3.125 do not, and what a dimension came to is the one thing a
  reader can measure off the drawing.
  `expr::set_dimension` is the one write path for text (a bare number becomes `Arg::Num`, with
  the angle conversion — the app converts nothing); `Sketch::add` and `io::from_json` evaluate;
  `Sketch::set_constraint_num` is the write path for a number, and re-evaluates when it dropped
  an expression.  Documents save `{"expr", "value"}` and accept a bare
  string; the bindings' records keep the number in `args` and put the text under `exprs`, and
  their proxies `sync()` before handing out a value, since an edit elsewhere can move it.
- **Every number has a dimension, and it is checked** (`units.rs`, Solvent §3.3).  Two bases — a
  **length** and an **angle** — with *rational* exponents, because `sqrt` halves one.  `*` and
  `/` derive, `+` and `-` demand agreement, `^` takes a whole power on a dimensioned base, and
  what an expression comes to is checked against its slot (`SpecKind::dim()`, so nothing is
  written per constraint type).  `Aff` carries the dimension beside the value: one walk, because
  the dimension of `a * b` is a fact about the same two operands the number came from.
  **The asymmetry between `Dim::fits` and `Dim::agree` is the design.**  A *context* — a slot, a
  function's argument — may take a bare number, which is the whole of what "a document with no
  `unit` line is in drawing units" means.  Two *operands* are not a context, so mixing one that
  said what it was with one that did not is a question the language asks rather than answers:
  `90 / N + ivp` is an error, and `90deg / N + ivp` is the answer.  A **name** is worth a number
  and where it is used decides what it is (`w = 80` in a Length slot does not make `w` a length);
  a unit on the literal and a component formal's declared `Ty` are what *do* travel, and the
  formal is what catches `param x = w + phi` — `flatten::settle` substitutes a parameter away, so
  a dimension that did not travel with the number would leave nothing to check.
  A literal may carry a unit, converted **to the document's own** by the tokenizer (which is why
  `expr::parse_in` takes `Units`): `unit mm` names it, and without one a suffix is refused rather
  than guessed.  **Feet-and-inches is one literal** — `1' 6 3/16"` — by the rule the language
  already had, that *a space tells the readings apart*; so the language has **no string literal**
  at all (`"` is the inch mark, a `Str` argument is a bare word, a raw branch key is bare).
  `pi` is dimensionless and `tau`/`turn` are a **turn**, so `tau == 2 * pi * 1rad` holds; the nine
  `* 180 / pi` conversions are `* 1rad`, which is not noise but the statement that
  `inv φ = tan φ − φ` holds only in radians.  Storing the unit costs the solve nothing (every
  kernel is homogeneous in length), and `io::paste` converts a figure between two documents that
  named different ones — `Sketch::rescale`, which is written out by kind because "is this
  parameter a length?" is not a question a `Param` can answer.
- **A number's three names are one namespace** (issue #47, item 7; Solvent §5, §6.3).  A named
  dimension (`a distance(w = 60) b`) declares `w` in its body exactly as `param w = 60` does:
  `flatten::params` collects both as `Def`s and works them out in one dependency order, so a
  `param` may read a named dimension, a second `w` of either kind is "declared twice", and
  `pythagoras.sv`'s `distance(a = la)` is a param feeding a dimension whose name the sheet then
  reads.  A named dimension is two things in scope — its **number** in `vals`, for a `param`,
  a seed or a count, and its **name** in `Scope::graph` (written name → absolute name), for a
  dimension's text — because a dimension reading it must keep the *name*, or the tie the
  expression graph makes and the callout shows would be folded away.  `settle_text` is the one
  pass over a dimension's text: a name in `graph` reads as its absolute name (`w` → `t1.w`),
  a formal or a `param` as its number, and **a name nothing in scope declares as the
  instance's own unknown** (`Scope::instance_prefix`: `t1.w`, the name an unbound formal
  already gets — a block copy's prefix is skipped, so a `cycle` shares its body's unknowns),
  which is what stops a component reading the document it is drawn in by writing a name the
  document happens to define.  One pass, because a second would find a formal's name inside
  the absolute name the first had just written — which is also why `substitute_with` reads a
  dotted path as one word.  A name declared inside a block copy is `#3.0.w`, so the expression
  lexer reads a `#`-led key as an identifier.  The file's named dimensions reach its components
  through `Walk::file_graph` beside `file_vals`, formals shadowing both; a module's are numbers
  only (`module_params`), its drawing never being elaborated.  `tests/names.rs` is the gate.
- A name **nothing defines is a free variable** (`expr::Free`): an unknown of the sketch rather
  than an error, so the dimensions reading it are tied to each other and what they come to is
  left to the solver — one degree of freedom where two stated numbers would have been none.
  `expr::evaluate` owns them: it allocates one Param per free name into `Sketch::free_vars`,
  retires it to `fixed` when the last reader stops reading it — keeping the *slot*, so reading the
  name again reuses the unknown rather than leaking a new one and moving the parameter count that
  `topology_key` ends with — and rewrites every binding (`Constraint::free`, at most one, which is
  why it is on the constraint and not the argument) from scratch each run, so a document, a paste
  and a rebuild carry only the text and the number and let the next evaluation work the rest out
  again.  It runs on every edit that can touch one: `Sketch::add`, `remove`, `set_constraint_num`,
  `set_dimension`, `from_json`, `report::exprs_json`.  A caller adding a *whole document* one
  constraint at a time uses `Sketch::add_quiet` and evaluates once at the end (`io::graft`,
  `io::from_json`): per-add evaluation is quadratic in the expression count, and would make a
  dimension whose definition has not arrived yet briefly a free variable.
  The tie is **affine in one free name** — `a`, `a / 2`, `2 * a + 5` — because `value = m*a + c`
  is the whole of what a fixed-width block can carry: `expr::eval` works in `Aff`, so ordinary
  evaluation is the `free: None` case and "a free name may only be scaled and offset" falls out
  of the arithmetic rather than being checked for.  `a * a`, `sin(a)` and two free names in one
  dimension are errors, and an erroring dimension keeps its last number like any other.
  Every type carrying a `Length` or an `Angle` therefore needs a *second* kernel, its free twin
  — the same rows and one more column, the unknown where the constant was, with (m, c) as the
  constants — declared in `CKind::free_kernel`, which matches `CKind` exhaustively so a new
  dimension type stops the build there; `every_dimension_can_be_written_free` checks the shape.
  Nothing else in the solve path is per-type: `params_on` appends the free column (it is always
  last), `consts_on` returns `[m, c]`, `kernel_id` picks the twin.  A fresh free variable seeds
  from the number the dimension already stated, or — when it states none — from the geometry, by
  Newton on that one row (`expr::settle`), which asks the kernel and no table.  A free dimension
  is `unsupported` in `cgraph` (the cluster vocabulary has no element for a relation *between*
  dimensions), is never jittered by the witness (it states no number to make generic), is part of
  `topology_key` (which unknown it names is a column), and joins `io::Part`'s walk (two dimensions
  sharing an unknown move together, which is as real a tie as a shared point).  `expr::sync_free`
  brings the numbers they *show* back into step with the unknown, from every seam that writes
  parameters without going through the others — `Sketch::set_x`, `io::Part::write_back` and the
  wave's direct writes — since a solve moves the unknown and a stale callout is a wrong drawing.
- The page is the drawing *and the source it is written as*: the program panel is a permanent
  second child of `<main>`, because the source is not a remark about the drawing — it is what the
  drawing **is**.  Everything else the shell has to *say* is still said beside what is picked, and
  there is no other sidebar.  A component selected names itself in the status line (`describeEntity`) and
  brings up one floating window listing the constraints that reach it, and only those.  The
  window is open on a `subject` — the selection while there is one, and otherwise whatever it
  was last opened on, so focusing a constraint (which empties the selection) does not pull the
  window out from under the pointer that clicked the row.  Nothing infers where a pick came
  from: `openPanel` is how the two things that pick without selecting say so — a callout
  clicked on the drawing, the banner's culprits — and `closePanel`, wired to `onSelect`, is a
  press that hit nothing.
- **The colouring is the parser's own scan** (`syntax::highlight`), not a second lexer in
  TypeScript: told to keep the comments and to say what each token turned out to be, so what a
  colour says a word is and what the parser makes of it cannot disagree — and a regex highlighter
  would part company on the first thing the language learned (`==` against `=`, a mixed fraction,
  a block comment, a new constraint's name, which `CKind::from_name` supplies here for free).
  `Tint` names the classes and the stylesheet says what they look like; the front end writes one
  element per run with the gaps between them plain, so it describes no whitespace and parses
  nothing.  A function of the *text* and not of an `Elaborated`, since the program being looked at
  is usually the one half-typed.
- Offsets cross the ABI in **UTF-8 bytes** and index a **UTF-16 string** on the other side, and
  `gear.sv` has an em dash in its second line — so this is not a corner case but the ordinary one.
  `core/program.ts::Offsets` is the conversion and `Document.adopt` is the **one seam** every
  report crosses: the diagnostics, the source map and the coloured runs are all string indices by
  the time anyone sees them, so no consumer holds the wrong unit and no two of them disagree.  A
  wholly-ASCII program builds nothing and answers in a comparison; otherwise it is a binary search,
  since a source map's offsets do not arrive in order.  Nothing sends an offset back the other way.
- The box it is typed in is `app/editor.ts` — a `CodeEditor`, which knows nothing of Solvent and is
  handed the text and a function saying which runs of it are what.  It is a `<textarea>` over a
  `<pre>` of the same text, because a textarea cannot colour its contents and a `contenteditable`
  gets the caret, undo and the platform's keys wrong.  Everything hard about that is one sentence:
  **the two layers must put every character in the same place**, or the caret sits somewhere other
  than the text it is in front of — and sharing the CSS is necessary and not sufficient, because
  the box is one run per line where the copy is one element per colour.  Four rules, each of which
  has already been the bug: the line height is a whole number of pixels; kerning and ligatures are
  off; a run may change the colour and **never the face** (an italic or a bold is a different font,
  and the box has no spans to match it); and the copy is **translated** to follow the box, never
  scrolled — the box carries scrollbars and the copy does not, so its scroll *range* is a
  scrollbar shorter and an assignment clamps at the bottom of a long file, which is precisely a
  caret that is fine at the top and a line out at the end.  None of it is visible to a unit test,
  so `npm run overlay` (`web/tools/overlay.mjs`) drives headless Chrome against the *real*
  `CodeEditor` and checks the three things that can break: the metrics agree, the colouring moves
  no glyph off where the plain text puts it, and the copy follows the box to both ends.  It is not
  in `npm test` because it needs Chrome, and `make test` must not.
- `SketchView` holds a `Document` (`core/program.ts`), not a `Sketch`: `view.sketch` is a getter
  for what the source came to and `view.source` is the document.  **A new document is one
  verb**, `SketchView.load` (File ▸ New is `newDocument`, Open and a test case reach it too):
  the outgoing text goes on the undo stack as one step, so a load is undoable and ⌘Z after one
  cannot land on an older state of a drawing that is gone; `settle()` is the one list of what
  is in flight — a gesture, an animation, a carried dimension, a tool's half-collected clicks,
  the remembered scene — cleared before any swap and before the box.  `setProgram` is the
  *edit* of the same shape, the panel replacing the text, and keeps the history its own way.
  Undo is program text, so it
  restores what somebody wrote, comments and all.  A selection crosses a re-elaboration **by
  name** (`Document.nameOf` / `Document.entity`) — a proxy is interned per `Sketch` and dies with
  it; a name is what the source calls the thing.  `swap` is the one seam that replaces the
  drawing, and it disposes the outgoing document.
  The source catches up at exactly two seams and **never per frame**: `syncSeeds` at the end of a
  drag (`gesture::endGesture`, guaranteed numeric) and `syncSource` at the end of `afterEdit`.
  The panel is wired to `onProgram`, never to `onDragFrame`.
- A drag is an operation on the dragged point's *part* of the document (`io::Part`): what is
  reached from it through shared points and constraints, stopping at fixed entities.  `PlanDrag`
  builds its plan, systems and numeric fallback on that part alone and writes each frame back, so
  a drag costs the figure, never the document, and separate figures cost it nothing; anything a
  drag exchanges by point index (guards, flips, branch keys) crosses through the part's maps.
- A drag made `PlanDrag::on` the document's own `PlanSolver` — the app passes `view.plan()`,
  cached per topology and pinned for the gesture — starts with one pass over the residuals
  (`PlanSolver::ensure_solved`) and runs on the document itself; `PlanDrag::new` is the
  self-contained form, which builds a plan over the part.  The plan comes back with every
  `move_to`/`guard_triangles`/`branches` (`None` for a drag of its own), and `part` is `None`
  exactly while the drag moves the document directly.
- Within the part, a frame costs the *region*, not the figure: `decompose::Wave` moves the plan's
  roots as rigid bodies — the ones holding the dragged point, growing by shared elements only
  while the cursor is out of reach — with every element shared with the rest held as an anchor
  and every direction class the rest carries pinning a rotation.  Pull (cursor row) then polish
  (anchors only, min-norm from the pulled pose) on the tiny merge system, rotations about each
  body's centroid scaled by `TURN_COST` × its gyration radius so least-norm is least motion and a
  free body slides rather than spins.  The wave keeps its bodies' and anchors' poses and never
  re-reads what it moved, so nothing compounds over a long gesture; it starts from a solved
  configuration (`PlanDrag::new` solves first) and hands over to the numeric `Drag` only for
  unsupported constraints, a region past `WAVE_MAX`, or a body solve that will not converge.
- In `decompose`, a direction class is a *relation*, not an *adjacency*: merge candidates, the
  worklist refresh and the core frontier come from shared elements (`neighbours`), and the class
  counts toward a candidate's rank once it is on the table (`pair_rel`, `relation_bound`).
  `Horizontal`/`Vertical` put every levelled line in one class, so counting it as adjacency made
  every cluster a neighbour of every other.  `relation_bound` must stay an upper bound on the
  merge rank (an under-count loses a determined merge to the numeric fallback): validate a change
  by forcing the factorisation on every call and asserting `rank <= bound` across the cases.
- **The ellipse is a library component** (issue #47, item 4): `Ellipse(f: plane, a, b, u)` in
  `rust/lib/std.sv`, a computed point at eccentric angle `u` on a datum, traced as a curve — so
  `p on e`, `e tangent l` and `e curvature k` are the curve contacts, exact to third order, and
  the entity kind with its three kernels and three `CKind`s is gone from every exhaustive arm,
  the FFI, the binding and the app (the ellipse *tool* went with it: a tool that writes a
  `use std`, a datum and a curve statement is a follow-up).  The parser keeps the word only to
  refuse it, naming the spelling; `io::from_json` refuses a document carrying the old
  `"ellipses"` table rather than reading it short.  `tests/ellipse.rs` holds the rim, the
  tangent and the osculating circle against the closed forms the document never states, and
  the rim turning with its datum.  An axis is a value the curve takes — stated or a `param` —
  since a curve written in place is given every value and a component of one computed point
  cannot be drawn as an instance whose formal is left free.
- **A curve is a point of a component, as one of its numeric formals runs** (Solvent §6.5).
  There is no curve family: `curve path = leg.toe over theta in (0, 360)` asks a *drawn*
  instance where one of its points goes as one of its formals runs, and
  `curve e = Involute(base, phase: a0).p over u in (u0, u1)` asks the same of an instance
  written in place and never drawn.  `syntax::CurveSpec` is the statement (`CurveTarget`
  `Drawn`/`Anon`, the swept formal, the interval); the flattener records every instance it
  binds (`flatten::InstanceInfo` — prefix, component, the actuals resolved to absolute names,
  the numbers) and resolves a drawn target onto the instance owning the longest prefix, so
  `build_curve` never re-derives which instance a point belongs to.  A component's point is
  placed one of two ways, and `program::compile_curve` picks the body from that: a **computed**
  point, `point p = (xexpr, yexpr)` (`Decl::computed`), compiles to two `tape.rs` tapes — the
  formula an involute has — and a component with one is refused on the sheet, since nothing
  there holds a point to a formula; any other point is a **locus**, lowered by `compile_trace`
  from the body's statements.  Either way the tapes are differentiated forward in the swept
  formal *and* in every coordinate they read: `∂C/∂u` is which way a contact may slide, `∂C/∂θ`
  is how the curve moves when its geometry does, and without it a point solves once and falls
  off the moment the circle is dragged.
  **The body is expanded by the real flattener, symbolically** (`flatten::expand_component`,
  `Sym`): the entity formals are names the body may reach, and every numeric formal is bound
  as a *free value named after itself* — the same `Aff` an unbound formal becomes on the sheet
  — so the ordinary machinery carries it: `substitute` writes a free value back out by name (or
  as `(m * name + c)` for a `param` affine in it), `settle` keeps a dimension that comes to no
  number, and the mode adds one policy, `Walk::keep_text`: a text nothing can work out at all
  (a `param` or an argument over `sin(u)`) is kept as text and written in where it is read
  (`Sym::texts`, keyed by absolute name and looked up through the scope's prefixes like any
  name), where the sheet would report it.  A nested instance, a `repeat` and a formal's alias
  are the flattener's as they are on the sheet — and a nested instance's *own* unbound formal is
  its own unknown there too (`#c12.i.u`), no column of the curve, and reported rather than
  captured by an outer formal of the same name (`tests/curve_of.rs`).  One expansion per
  `(component, point, formal)` — `CurveDef::key` — and every instance asked for the same
  shares the definition.  Which instance a drawn point belongs to is `Walk::owner_of`: the
  innermost drawn instance owning the name's prefix *whose component has the swept formal*,
  handed to the elaborator as `CurveSpec::of` rather than re-derived there.
  The variable table is the swept formal, then every scalar the entity formals contribute
  **in `entity_params` order** (`EntKind::scalar_names`), then the other numeric formals —
  which is also `params_on`'s column order, so a tape's gradient *is* a row of the Jacobian.
  `EntKind::Curve` is the one kind whose children need not be points, and the one that must be
  built and grafted **last**, since its arguments may be of any other kind.
  A curve's kernel belongs to its **definition**, not its type: two definitions read different
  numbers of coordinates and cannot share a fixed-width block.  So `CKind::kernel()` panics for
  the three curve kinds, `kernel_id_in(sk)` returns `N_KERNELS + 3·def + slot`, `System` owns a
  table of the static kernels plus **three per definition** — the contact `PointOnCurve`, the
  tangency `CurveTangentLine` (`inv tangent l`: the spline tangency's two rows over the curve's
  frame) and the curvature `CurveCurvature` (`inv curvature k`: the spline curvature's three) —
  and the registry publishes `kernel: -1` for all three, which the bindings' exhaustive tests key
  on rather than on a name.  The tapes ride in the constraint's `consts`, so no kernel signature
  learns about curves and `KERNELS` stays `'static`.  What a tangency and a curvature need is the
  curve's **frame** (`kernels::CurveFrame`): `C` to `C'''` in the parameter and the gradient of
  the first three orders in `[u, θ…]`.  A formula gives all of it exactly from
  `tape::eval_series_flat` — truncated Taylor arithmetic to third order with the gradient
  carried through the same recurrences (`tape::Series`, checked against finite differences of
  the first-order evaluator in `tests/tape.rs`).  A trace gives `C` and `C'` exactly (the implicit
  function theorem, as the contact already had) and `C'`'s gradient by **forward difference** of
  that exact velocity from the memoised centre (`locus::kernel_frame`: one warm block solve per
  column, from the contact's remembered pose, so the branch cannot change under it) — accurate
  enough for a Jacobian, which is all it is used for, since the residual is exact.  **A residual
  never builds the frame**: `curve_value` gives the derivatives alone (for a trace, the memoised
  contact evaluation), and only a Jacobian pays for the gradient — the `EllFrame` bargain, since
  a rejected trust-region step evaluates residuals without ever asking for one, and for a trace
  the gradient is a sweep of block solves.  It gives no `C''`: that would need second derivatives
  of the block's kernels, and a residual by difference would solve to a slightly wrong circle and
  call it right, so `constraints::validate` refuses a curvature against a traced curve and its
  slot in the table is the `refused` kernel (every row NaN, which `System` reads as not
  converged).  Which kernel a kind runs through is `CKind::family_kernel` — `FamilyKernel`, whose
  discriminant is the slot and which knows its row count — read by `kernel_id_in`,
  `n_residuals`, the registry's `kernel: -1` and `kernel_table` alike, so a fourth per-definition
  kind is one arm.  `Sketch::curve_polyline` is memoised against everything it reads (the
  variables at the interval's start, the interval, the anchor and its pose), because a pick
  walks every drawn curve on every pointer move.  `tests/curve_contact.rs` holds both contacts
  against the involute's closed forms (the tangent is the string; the radius of curvature is
  the string unwound); `tests/common` is the finite-difference Jacobian check the curve tests
  share.
  **An unbound numeric formal is an unknown of the drawing.**  `leg: Leg(axle, pivot)` with
  `theta` not given binds it to a *free* `Aff` named under the instance — `leg.theta`, so two
  legs have two cranks — and every reader carries it: `value_aff` passes a free value the scope
  bound (refusing, as it always did, a name nothing binds), so a `param` over it is affine in
  it, an argument handing it to a nested instance binds that formal to the same unknown, and
  `substitute` writes the name into every dimension that reads it, where the language already
  makes a name nothing defines a free variable (`expr::Free`).  A drawn mechanism is therefore
  drawn once with its crank free, and the curve's anchor follows the unknown: `CurveE::home`
  is `Home::Free(name)` and `Sketch::curve_home` reads it, since the unknown is allocated after
  the curve is built and moves with every solve.
- A **locus** (`locus.rs`) is what a traced point is: the curve is wherever the body's
  constraints put `p`, which is how a person states an involute ("the end of a taut string as
  it unwinds") without ever deriving it.  The body is lowered once, when the curve is first
  built (`program::compile_trace`), through a scratch sketch and `Constraint::params_on` — the
  real column mapping, never a second copy — into rows of the static kernels over one variable
  table `[u, θ, values, q, w]`: `q` the body's own coordinates, `w` a dimension written over `u`
  and the geometry, computed by a tape and read by the dimension's **free twin** kernel with
  `(m, c)` the unit conversion, so no new derivative code exists anywhere.  Evaluating `C(u)`
  is a small damped Newton solve of the rows and its derivatives are the implicit function
  theorem at the solution — `∂C/∂u` and `∂C/∂θ` from one factorisation of the inner Jacobian —
  which is what keeps a contact on the curve when the geometry it is written over is dragged.
  The whole compiled body encodes to flat `f64` and rides in the contact's consts exactly as
  the tapes do (`locus::eval_at` is the one evaluator — kernel, tessellation and tests all run
  the flat form — with `eval_flat` its cold entry), so `System` only picks `trace_kernel` over
  `curve_kernel` per definition and the bindings are untouched.  A trace contact's constants
  are `[anchor, n_values, values…, has_pose, flat…, pose…]` (`kernel_eval` reads them,
  `consts_on` writes them, `kernel_table` sizes them; the flat's own header says how wide the
  pose is, and `view` ignores what trails it): the anchor parameter, the numbers, and — for a
  curve of a **drawn** instance — the **pose on the sheet**, read off the instance's own points
  at every compile and refresh (`CurveDef::pose_of` names each inner unknown's owner by its own
  scalars, `CurveE::pose` the resolved entities, `Sketch::curve_pose` the one reader, and
  `model::whole` the one statement that a pose with a hole is no pose), which is where the
  anchor solve starts (`locus::Anchor`).  The polyline sweep walks its samples **outward from
  the anchor** — down to `u0`, then from the anchor's pose again up to `u1` — so every solve
  is a sample; marching to `u0` first paid for that stretch twice on every repaint.  Chirality —
  which way the string unwinds — is a *branch*, and no regular residual can state one (a
  residual zero at exactly one direction has a vanishing gradient there).  A body states its
  way onto a branch by three instruments, in order of strength (spec §6.5): a **signed
  constraint** where the vocabulary has one (`point_line_distance` is signed, so "taut" written
  against the radius line makes the winding algebraic in the sign of the roll); an
  **orientation predicate** — `ccw(a, b, x)` contributes no residual and selects the solution
  component, its third point one the body places, enforced only at the **anchor** (the drawn
  pose, or the value an instance written in place gave the swept formal) by
  reflect-and-resolve, with deterministic restarts (fixed-seed `rng::Rng`, scaled by the entity
  formals' coordinates and by *nothing else* — scaled by every outer value, a tooth's `phase`
  of 129° threw the restarts eight radii from the circle, and the target `u` made the anchor
  solve a lottery per evaluation) when there is no pose and the seeds leave it nowhere to
  start; and a **seed** for what neither can say.  Continuity carries the branch from the
  anchor everywhere else — evaluation is one warm-started march, the same walk the polyline
  sweep does, and a body with predicates never trusts a direct solve *from the seeds* at the
  target (it could land in a forbidden component).  A branch once carried is **kept**: the
  outer solver moves `(u, θ)` a little and asks the same contact again, which is a
  continuation step like any other, so each contact's pose is remembered and the next
  evaluation *resumes* from it instead of re-walking the march.  Replaying it cost thirty-four
  block solves for every one it needed — 92% of a traced gear's solve.  A resumed step is
  trusted only as far as `locus::continues` can check it, against the tangent the pose's own
  `∂C/∂(u, θ)` predicted (a correction second order in the step is the same branch; one the
  size of the gap between branches is not), and what fails falls back to the anchor and the
  full march, so the doctrine above still decides every branch.  A pose is addressed by *where
  its contact's constants live* — `refresh_consts` rewrites a trace contact's in place, so the
  address is its own for the life of the system, and the values that rewrite may change ride in
  `outer` too and so miss rather than read stale.  Only a recompile can put another contact at
  that address, which is why `System::new` calls `locus::forget`.  The kernel path therefore
  carries a history where the drawing path is always cold, which is the intended reading: a
  contact is a point that reached its `u` by a road, and the two agree because every step of
  that road was checked against the curve's own tangent.
  A seed is a *place*, named geometrically where it can be: `hint(at: c, bearing: u + phase)`
  is the point at the edge of the circle, `hint(at: t)` is where another point starts
  (`program::at_seed` lowers both to the tapes the coordinate spelling would be), and
  `hint(x: xexpr, y: yexpr)` remains for a place with no name — in a component that is only
  ever traced, since on the sheet a seed is a number a solve writes back (`build` refuses a
  drawn one).
  A traced body must be square — as many rows as inner coordinates — or elaboration refuses it.
  `tests/trace.rs` holds the taut-string involute checked against the closed form it never
  states (with seeds wrong by 3× on purpose), a gear run on traced flanks, and — the guard on
  the resume — the same parameters asked in three orders, since an answer that depended on what
  was evaluated before it would be a warm start that had changed one; `tests/jansen.rs` holds
  the drawn-instance form against an independent circle-intersection model at 24 crank angles.
- A curve is *geometry*, so like a dimension callout it is laid out in the core and the front end
  only strokes what it is handed: `curve::tessellate` refines to `FLATNESS_PX` screen pixels
  through `unit` (the world length of one screen pixel), and `curve::closest` is the pick test
  and the seed for a fresh contact, so the two agree about where "on the curve" is.  No binding
  evaluates a basis function, and none writes the degree down: `report::registry_json` publishes
  `curve.minCtrl`, so a front end's tool and its messages cannot drift from what
  `Sketch::spline_with` will accept.
- `solve::Drag` is the one point-drag implementation (pull + polish), `RadiusDrag` its scalar
  counterpart for circle/arc radii (a `Radius` with `soft` set — its residual is already
  r − target, so no kernel of its own); the front end only translates coordinates.
- The ABI is a panic boundary: every entry point runs inside `guard`, so a core panic becomes
  `gcs_last_error()` and a neutral return.  That needs `panic = "unwind"` in the release profile.
  `wasm32-unknown-unknown` aborts whatever the profile says, so untrusted input (a document) is
  bounds-checked in the core as well.
- Determinism: ordered containers only (`Vec`, `BTreeMap`/`BTreeSet`), never `HashMap` iteration
  in the solve path.  Every random draw comes from the seeded `rng::Rng`.
- The trust-region loop is `newton::dogleg` over a `TrustRegion`; a new thing to minimise
  implements the trait rather than copying the loop.
- Nothing in the project is auto-formatted: there is no `rustfmt.toml`, and `cargo fmt` would
  reformat every file.  Match the surrounding style by hand (100 columns).
- No LAPACK/BLAS: the QR, complete-orthogonal, SVD and LDLᵀ routines are ours, and
  `rust/gcs-core/tests/linalg.rs` checks them against `nalgebra` — the one place two
  implementations are still compared, on purpose.  **The library has no dependencies; its tests
  have one reference implementation**, and it is a `[dev-dependencies]` entry precisely so that
  nothing it brings links into the cdylib or the wasm.  Each test also states the property the
  reference cannot (`A ≈ QR` on the pivots, `NᵀN ≈ I`, a minimum-norm solution orthogonal to the
  null space): a reference agreeing is evidence, a property holding is the contract.
