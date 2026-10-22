# Seeing a load scenario in the workspace: plan

**Status (2026-10-22): proposed.** It belongs to #510 (design intent as structural performance) and
covers what [analysis.md](contract/analysis.md) leaves as "the drawing's to say": "a requirement's
figure is the deformed frame". Nothing here is built yet. The [open questions](#open-questions) are
the decisions this plan needs from a person before the first stage starts.

A document with a `scenario` gets a second chooser beside **Sketch on**, in the viewport's upper
right. Choosing a scenario draws it over the workspace:

* **its supports**: what is `fixed` or `pinned`, and the direction held where it says `along:`;
* **its loads**: forces at points, along lines and beams and over faces, pressures on faces, and
  `accelerate` as a body load;
* **its deflection**: each body it reads, deformed by a stated factor and coloured by how far it
  moves, with a legend, the undeformed shape left as a ghost.

The page already learns what a scenario *answers* (`gcs_elab_scenarios_json`: the footer's
`#scenarios` line and the Diagnosis report). It learns nothing about where a scenario *falls* on
the drawing, or how the drawing *moves* under it. This plan adds those two readings to the core,
carries them over the ABI and draws them in the box.

## What the page has today

* **`#plane-select`** (`web/index.html`, `shell.ts`, `main.ts::refreshPlanes`) is a `<select>` in
  `#plane-pick`, rebuilt only when its list changes. Choosing from it is view state: it writes
  nothing and solves nothing.
* **`Document.scenarios()`** returns `[{ name, words, answers }]`. `SketchView.afterEdit` reads
  it again after every solve, *on the page's thread*. For a solid read in three dimensions this
  is seconds of meshing and factoring (see analysis.md, "What S3 leaves", Speed), and it is paid
  twice wherever anything is answered, since `scenario_read` makes a second reading twice as
  coarse for the error.
* **The core reads a body only where a requirement names it.** `beams::read_at` analyses the
  frame of each requirement's target. `bodies::read_at` analyses the solids that requirements
  name. A scenario with supports and loads and no requirement reads no body at all. Each reader's
  `Analysed` holds the displacements (`State::u`) and drops them when the read ends.
* **The box** (`box3d.ts`) draws objects' meshes, creases, panes and axes from what the core
  gives it, and computes no coordinate. `ghost()` already greys an object out. **The mesh worker**
  (`mesh-worker.ts`) runs three jobs, field meshing, exact surfaces and claims. Each job has its
  own copy of the core and is keyed by the text and the page's parameters. A newer job ends the
  one in hand at its next step.

## Decisions proposed

1. **Choosing a scenario is view state.** It writes nothing to the source or the `.svd`, solves
   nothing, and is not undone. The drawing's solve already holds every driving row whichever
   scenario is chosen, because a scenario adds no row of its own beyond its driving deflections
   (analysis.md, "A scenario solves nothing"). The choice is kept across an edit by the
   scenario's name, as `selectedSolids` keeps an object. It falls back to *None* when that name is
   gone. A file opened fresh shows *None*.
2. **The chooser exists only where there is something to choose.** `#scenario-pick` is hidden
   for a document with no scenario. It offers *None*, then each scenario in the order written.
   An extended one is written as `heavy : shelf`. A scenario with a requirement that fails, or a
   word said against it, is marked as the footer marks it.
3. **"Constraints" means the scenario's supports.** These are the boundary conditions the
   analysis holds. The drawing's geometric constraints are drawn as they always are and are not
   changed by the chooser. (See [open question 1](#open-questions).)
4. **The core says where and which way, and the page says what it looks like.** Every anchor,
   direction, face sample, inward normal, rest position and move comes from the core. Glyph
   shapes (a cone, an arrow, a hatch), their size in pixels and the colour ramp are chrome, kept
   with `INK` in `box3d.ts`. This is the line `paint.ts` already draws between geometry and ink.
5. **The deformation is a morph target.** The core hands each body's rest positions and its
   moves as two buffers, plus a default factor. three.js draws `rest + s·move` on the GPU
   (`morphAttributes.position`, `morphTargetInfluences[0] = s`). The scale slider then needs no
   round trip to the core, and the page still works out no coordinate. Glyphs are sized constant
   on screen through `unit`, as callouts are. The page sets each glyph's matrix scale from it,
   the way `placement` stands a solid.
6. **What is drawn deformed is what was analysed.** A frame is drawn as its beam elements, sampled
   along the cubic. A region read in its plane is drawn as its triangle mesh, stood through its
   depth. A solid is drawn as its tetrahedral mesh's boundary. The display mesh is never morphed.
   That would claim a resolution the analysis does not have, and the analysis mesh's edges can be
   shown on request.
7. **The body is read wherever the scenario touches it, not only where a requirement asks.**
   A scenario that only states supports and loads still shows its deflection. The picture reads
   every frame, region and solid that any of its lines falls on. "Falls on" is the rule the
   readers already use: `beams::of_frame`, `prisms::falls_on` and `bodies::solid_of`.
8. **The picture is the answers' reading, not a second analysis that might disagree.** The move
   it draws at a requirement's point is the answer `report::scenario_answers` gives for it, to
   rounding. Both read the same mesh at the same size. A body that cannot be read (left free, no
   material, a no-answer) is not drawn deformed. Its reason is said in the core's words where the
   footer says it today, and the body keeps its rest shape with its marks.
9. **Analysis runs in the worker.** A fourth job kind, `scenario`, reads the chosen scenario's
   picture off the page's thread for every kind of body, so frames take the same path as solids.
   The marks are cheap (no analysis), are read on the page and appear at once. The deformation
   arrives when the job posts it. Moving the *answers* into the same job is
   [open question 3](#open-questions).

## The core

### Marks: where a scenario falls (no analysis)

New in `gcs-solve`, beside the readers: `scenario_marks(sk, sc) -> Vec<Mark>`, one mark per line
of the scenario in the order written, so each mark carries the `Line` it was written on.

```text
Mark {
  line:   which support / load / requirement / reading (its index among the scenario's)
  what:   Fixed | Pinned | Force | Pressure | Accelerate | Output(Output) | Plane(InPlane)
  at:     [V]           the anchors: a point; a line's or beam's samples along it; a face's
                        samples (its triangles' centroids, thinned to a spacing)
  dir:    [V] | V | —   per anchor: a force's direction; a pressure's inward normal; a support's
                        held direction (`along:`), none where it holds every direction
  faces:  [(solid, face path)]   faces to tint: a pressure's, a face support's
  size:   0..1          a force's or a pressure's magnitude against the scenario's largest of its kind
  text:   "600N along down", "2MPa", "fixed", "9.81m/s²"   the core's words, `beams::value_text`
}
```

The anchors come from one place: the points `diagnose::analysis::points` already reads for a
support, and the same rule for a load's target. A face's samples come from the mesh the box
already evaluated (`mesh::grouped` by path), so marks on faces evaluate no second solid. A
target that cannot be read (a face path naming no face) produces no mark and is said as the
scenario's words already say it. **A mark adds nothing to the drawing** (gate below).

### Picture: how the drawing moves (one analysis per body)

`scenario_picture(sk, sc) -> Picture`, its readers split per body kind and each the reader
already there with its `State` kept:

* **A frame** (`beams.rs`): the mesh at `PIECES`. For each element, `SAMPLES` points along it,
  each with its rest place and its move under the cubic, Hermite in its ends' moves and turns.
  The cubic is the beam's exact deflection under end loads, so the curve drawn between nodes is
  the analysis's and not a straight line between nodes. Drawn as polylines.
* **A region read in its plane** (`prisms.rs`): each `Tri6` cut into four flat triangles at its
  middle nodes. Both caps at `mid ± depth/2` and the walls along the boundary sides are
  triangles, with the same in-plane move at each level.
* **A solid read in three dimensions** (`bodies.rs`): `Mesh::facets`, each six-node facet cut into
  four, and each node's move.

Per body:

```text
Body { kind, solid | beam, name, rest: [f32; 3n], moved: [f32; 3n], size: [f32; n] (|u| / max),
       triangles | polylines, max: f64, max_at: V, max_text: "0.412mm" }
Picture { bodies, scale: f64, words: [String] }
```

`scale` is the default factor. It is chosen so the largest move over the scenario's bodies
draws as `SHOWN` (a tenth) of their extent, and rounded to a 1–2–5 step so the legend reads
`×200`, not `×187.3`. A scenario whose largest move is already that size or larger is drawn at
×1 and says so. The field is the displacement's magnitude. Its range and the legend's ticks come
in the core's words.

The reading is made once, at the readers' own size, **with no coarser second reading**. The
error estimate belongs to the answers, so the picture costs half of what `scenario_read` costs.
A `what` argument (`Static`, later `Mode(n)` and `Buckled(n)`) leaves room for
[later stages](#later-and-not-in-this-plan) without changing the shape.

### Over the ABI

Following `mesh.ts`'s split, the small ragged part is JSON and the bulk is buffers:

* `gcs_elab_scenario_marks_json(h, sk, i)`, scenario `i`'s marks;
* `gcs_scenario_picture_json(h, sk, i)`, the picture's header (bodies, counts, scale, legend,
  words), memoised so the buffer calls below read the same answer;
* `gcs_scenario_picture_rest`, `_moved`, `_size`, `_index(h, sk, i, body, ptr, cap)`, the
  buffers.

`web/src/core/scenario.ts` holds the thin proxies (`marks`, `picture`) and no algorithm. The
`gcs-ffi/tests/` suite checks the buffers against the JSON's counts.

## The page

* **`#scenario-pick`** sits beside `#plane-pick`, labelled *Scenario*. `main.ts::refreshScenarios`
  is rebuilt from `view.scenarios` as `refreshPlanes` is rebuilt from `planeChoices`, only when
  the list changes. A change sets `view.scenario` (a name, or `''`) and blurs the chooser so the
  keys go back to the drawing.
* **`SketchView.scenario`** is the choice. `afterEdit` re-reads the marks, which is cheap, and
  restarts the worker's `scenario` job keyed by text, modules, `x` and name, as `claims.start` is
  keyed. The first drag frame hides the deformation, since a drag moves the drawing it was read
  of. Marks on points and lines follow every frame. Marks on faces and the deformation come back
  once the drag settles.
* **The box** gains a `scenario` group drawn above the objects:
  * the objects the picture deforms are `ghost()`ed, so their rest shape stays visible;
  * each body is a `THREE.Mesh` (or `LineSegments2` for a frame) with a morph target, coloured by
    vertex from `size` through the ramp, with the analysis mesh's edges as an optional overlay;
  * **supports**: `fixed` is a hatched plate square to the held face or at the point; `pinned` is
    a cone; `along:` is a cone on the held direction with a roller bar. Faces held are tinted in
    the support ink;
  * **loads**: an arrow ending at each anchor (its length from `size`, with a floor so the least
    load reads); a load spread along a line is a row of arrows with their tails joined; a pressure
    is arrows into the face along its normals with the face tinted; `accelerate` is one hollow
    arrow at the body's middle;
  * **labels** (a load's `text`, a support's word, the legend's largest move at `max_at`) are
    drawn on the 2D canvas at anchors the core projects each frame, as points in space are
    (`workspace::space_points`). Nothing in `app/` projects a point.
* **The scenario card**, a small panel under the choosers, reads what the page already holds
  (`view.scenarios`): each requirement's sentence and verdict ink, the scale slider (0 to 4× the
  default, a ×1 stop, an *animate* toggle that swings `s` from 0 to the factor), the colour
  legend, and *Show mesh*. Hovering a row lights its mark. Clicking a mark or a row puts the
  editor's caret on the line it was written on (`Stated`'s `Line`), as the program panel does
  for a statement.
* **Progress** shows in the same line the claims' judging uses (`judgingLine`), e.g. "reading
  `shaft`: 18,204 unknowns". A job is cancelled as the others are.

## Stages

Each stage lands on its own, with its gate, its contract rule and its app test.

### V0: the chooser, the marks and the card

Covers supports, forces and pressures, with no analysis: the core's `scenario_marks`, its ABI
call and proxy, `#scenario-pick`, the glyphs and labels, and the card reading the answers the
page already has. Any document in the library with a scenario shows where it is held and loaded.
**Gates:** `tests/scenario_view.rs` (new, listed in `tests/main.rs`):
`every_line_of_a_scenario_is_a_mark` (each of the library's scenario documents),
`a_mark_adds_nothing_to_the_drawing`, `a_pressure_points_into_its_face`,
`a_face_mark_samples_the_mesh_the_box_draws`. `core.test.ts`: `a document says where its
scenarios fall`. `app.test.ts`: `the scenario chooser lists the document's scenarios and None`,
`choosing a scenario writes nothing`, `the chooser is hidden for a document with none`.

### V1: a frame's deflection

Covers `beams.rs` keeping its `State` and sampling the cubic, `scenario_picture` for frames, the
buffers, the worker's `scenario` job, morph targets, the legend and the slider.
**Gates:** `a_cantilevers_picture_is_its_cubic` (the tip's move is `P L³/3EI`, and a sample
between nodes is the closed form's curve to 1e-9), `the_picture_moves_a_requirements_point_by_its_answer`
(`shelf_bracket.sv`, `portal_frame.sv`), `a_frame_left_free_draws_no_deflection_and_says_why`,
`a_scenario_of_supports_and_loads_alone_is_read` (decision 7).

### V2: regions and solids

Covers `prisms.rs` and `bodies.rs` keeping their states and giving their boundaries, the region's
prism stood through its depth, and progress words for a long read.
**Gates:** `pressure_pipe.sv`'s bore moving by Lamé's radial displacement,
`sized_plank.sv`'s corner moving by its driven 0.9mm, `cantilever_plate.sv`'s and
`pressed_panel.sv`'s pictures matching their answers, and a cancelled job leaving no picture of an
older drawing (`app.test.ts`).

### V3: what the requirements say, where they say it

A *probe* at each requirement's point: its value and verdict as a label in the verdict's ink (a
refuted claim red), and the worst point of a face or a beam marked where the reader found it.
Hovering a mark or a row lights the source line in the editor.

## Later, and not in this plan

* **Colour by stress.** von Mises at nodes exists for `Tri6` and `Tet10`, and the frame's normal
  stress at its fibres. A *Colour by* choice in the card needs each reader to give it per vertex,
  and a factor of safety reads it already.
* **Modes and buckling shapes.** `Picture`'s `what` as `Mode(n)` (`eigen::lowest`'s vector,
  animated) and `Buckled(n)`, offered in the card where the scenario asks for `mode(…)` or
  `buckling(…)`.
* **A scenario in a sheet.** An `.svd` view that draws one scenario's marks and deflection on
  paper, which is where a scenario's picture would be *saved*, since the model file never holds
  presentation.
* **Picking a mark in the workspace** (`workspace::pick`), which needs the marks in the core's pick
  order.

## Open questions

1. **"Constraints".** This plan reads it as the scenario's supports (decision 3). If it should
   also dim or highlight the drawing's own geometric constraints while a scenario is shown, that
   is a separate rule for `paint.ts`.
2. **Glyph size.** Glyphs are proposed constant on screen, as callouts are (decision 5). The
   other choice is a fraction of the model's extent, which keeps loads in proportion to the part
   when zoomed in, and is what most FE post-processors do.
3. **Answers off the page's thread.** Today `doc.scenarios()` holds the page for seconds after
   every edit of a document with a solid read in 3D. Folding the answers into V2's worker job
   would share one analysis per body between the answers and the picture, but the footer's line
   would then arrive a moment after the solve rather than with it.
   The proposal is to do it in V2.
4. **The default factor.** A tenth of the extent, rounded to 1–2–5, is a common
   convention. A true-scale default would be more honest and usually invisible.
5. **Remembering the choice.** View state only, as proposed, or kept per document in the page's
   local storage so a reload comes back to the scenario being looked at.
