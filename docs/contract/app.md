# Documents, edits and the app

Part of the [contract](../../CLAUDE.md) the code is held to. Read it when you touch `edit.rs`,
`io.rs`, `overview/`, `callout.rs`, or `web/src/app/`. An issue number (#n) or a spec version
([0.n], see the [language changelog](../solvent-changelog.md)) says where a rule came from.

- **The document is the Solvent source (`gear.sv`, `syntax.rs`, `program.rs`, `edit.rs`).**  The
  drawing is what elaborating it produces; *every* edit is a **splice** of the text somebody
  wrote and **never a reprint** (which would flatten components and drop comments).
  `edit.rs` is the whole of it: `commit_seeds` (a solve), `reconcile` (a gesture), `remove`,
  `set_dimension`, `add_point`/`add_entity`/`add_relation`, and `mint` for a name.  Three edit
  classes, and **the core says which — the front end never guesses**: `Structural`, `Numeric`
  (only numbers a solve may move, so a compiled plan survives), `None`.  Writeback is one
  lexical rule: *a seed is writable iff it is inside a `hint(…)` clause, is a literal and not an
  expression, and is reached by exactly one instance path.*  `Decl::hint_span` carries where the
  clause *is* or — empty — where it *would go*; `commit_seeds` splices each seed in place when
  all have spans, else writes the whole clause there; a decl seeded by *place* is skipped.  A
  statement in one of the document's own components is written back when everything it made
  shares **one instance path** with no block copy on it (`edit::one_path`), and only by splicing
  a seed the definition already writes — no clause is written into a definition, so the Rect
  tool's bare `Rectangle` stays bare; drawn twice, it is two poses of one statement and left
  alone.
  `reconcile` and `retext` **apply themselves to the `Elaborated` and do not rebuild the
  drawing** (that would invalidate a half-finished tool's proxies).  `retext` re-parses (same
  ids); `adopt` extends the map onto new statements.  `reconcile` reads only the *append* (past
  the map's high-water mark); it cannot follow a **renumbering**, so deletion is `edit::remove`
  and not `io::without`.
- Deletion, copy and paste are one rebuild walk (`io::graft`): every surviving entity is
  renumbered into the destination and every reference follows, and a constraint comes along
  exactly when all its entities did.  `without` keeps what is not deleted, `copy` keeps the
  selection (so a clipboard is an ordinary sketch document), `paste` grafts one sketch onto
  another at an offset.  A new thing that travels with a constraint or an entity — a flag, a
  placement — belongs in `graft`, or the three will disagree.  JSON is now the *export* format,
  derived rather than canonical; `io::dumps` is still what the Rust tests and the benchmarks
  compare against.
- **Delete takes everything selected** that has a statement of its own (`edit::remove`): what a
  component or a block made stays, counted in `Edit::kept`, and the rest goes — so a band swept
  over the drawing that also caught `std`'s datums deletes the drawing, and the status line says
  what stayed.  Refused, saying why, only when nothing would go.  The key and the workspace's
  Delete button are one function (`main.ts`'s `deleteSelection`): the focused constraint, else the
  drawing's selection, the solids or the picture.  Gate:
  `deleting_beside_what_a_component_made_takes_the_rest`, and the app test beside it.
- **Make component** (`edit::make_component`; FFI `gcs_elab_make_component`,
  `Document.makeComponent`; the Component button `⇧G`, and Insert ▸ Component… to name it) is
  the selection's statements moved, **as written**, into `component Part(…) { … }` and one
  `part0 := Part(…)` standing where the first was — a splice, comments kept, held to the
  elaborator (an error the document did not have refuses it in its words).  What moves is the
  root statement each selected thing came from (an instance or a block whole, by its path's
  first step), then every relation — and every anonymous declaration, the `line(o, q)` one is
  written over — whose root names all moved (`remove`'s rule).  A line moves whole or not at
  all (a chain, a `;`); an `in P { … }` block's run is an `in` block inside the definition, and
  the block goes when nothing in it stays.  **What the moved text reads from outside is a formal
  of the name it had**, so the text reads unchanged: a declaration by its kind, an instance or
  group as `group`, a value by label typed by its dimension (`Length`, `Angle`, else `Scalar`);
  a module's datum has no bare name, so `std.front` is handed in as `front` and rewritten.
  **What stays outside names what moved through the instance** (`part0.a`), found by
  `syntax::name_uses`, the lexer's own reading of a text's names.  What a module made stays,
  counted in `Edit::kept`; `names` are the component and the instance, and the app carries the
  selection across by them.  `tests/make_component.rs` and `app.test.ts` are the gates.

- **A gesture writes a solid** (#162 F0): `edit::add_face`, `add_solid` (`SolidSweep`, extents
  as text into `Arg::Dim`, mixtures refused by the parser's own `sweep_of`) and `add_body_word`
  append over names (FFI `gcs_elab_add_face`/`_solid`/`_body_word`, `Document.addFace`/
  `addSolid`/`addBodyWord`), each **held to the elaborator before it is handed back**
  (`append_checked`: an error inside the appended text refuses the edit in its words; one
  elsewhere was there before the gesture).  A hole loop that is not one circle is written as a
  face first: **a face is a hole as it stands** (`holes: h`, its outer loop; one holed itself, or
  declared after, is E080).  `edit::remove` dooms a body word or a sweep naming a gone solid.
  Fresh names pass every bound name (`taken_names`).  `tests/edit_solids.rs` is the gate.
  **The region under the cursor** is `overview::region::region_at` (FFI `gcs_elab_region_json`,
  `Document.regionAt`): the graph of a plane's drawn edges joined **by shared point identity**
  (what a face's walk accepts), dangling edges pruned, faces walked turning least right; the
  smallest bounded loop holding the place is the outer, other pieces' outlines inside it (not
  inside one another) its holes; a loop along an edge twice is refused.  `region::written` names
  it by the source.  `tests/regions.rs` is the gate.
- **Extrude** (#162 F2): the `extrude` tool (`u`) washes the region under the pointer
  (`SketchView.hoverRegion`, `regionAt`) and a click writes the face, the solid at a round
  default depth (`set_off`) and — with a body selected — `union`, or with ⌥ `cut`, or with ⌥⇧
  `through:` + `cut`, as one splice and one undo step (`edit::extrude`).  The new solid is then
  **sized** (`SketchView.liveExtent`, carried across an edit by the solid's name): typed in the
  number box at its arrow (`dimbox`'s `NumberBox`, which dimensions use too) or dragged by the
  arrow the eye sees along its face's normal (`workspace::extrude_handle`, no tip seen down the
  normal; `extent_at` reading the drag).  **Which way it runs is the core's** (`model::Way`:
  behind, `depth: d`; front, `from: 0, to: d`; both, `from: -d/2, to: d/2`, ⇧): the drag previews
  by `Sketch::set_prism(i, way, d)`, which bumps `shape_epoch` (the box re-cuts objects on it,
  gesture or no), and writes once at release through `edit::set_sweep` with
  `SolidSweep::thick(way, text)` (the bracket list replaced, `checked` as every gesture edit),
  taken without an undo push so it joins the click's step.  `tests/extrude.rs` is the gate.
- **Revolve** (#162 F3): `about:` takes an **axis** lying in the face's plane as well as a line
  drawn there (`SolidDef::Revolve::axis` an `EntRef`; every reader goes through
  `Sketch::revolve_axis`, the axis in the profile plane's coordinates, `None` off the plane, within
  `AXIS_IN_PLANE`).  `build_solid` refuses E081 only what it can judge (`axis_may_lie_in`: the
  plane's own axes pass; an axis and plane wholly held are read now); one the solve places is
  refused where evaluated.  The `revolve` tool (`q`) washes a region; its click waits
  (`SketchView.pendingRevolve`) for a click on a line or axis (lit under the pointer), or takes one
  already selected, and writes face, `solid(f, about: …)` (a whole turn) and with a body `union`
  (⌥ `cut`) as one splice (`edit::revolve`, shared with `extrude` by `region_solid`).  Its turn is
  sized as an extrusion's thickness (`LiveExtent` kind `turn`, signed degrees, negative `cw`): the
  arc its profile's middle turns through (`workspace::turn_handle`, no tip seen along the turning
  plane), dragged by `turn_at` (the whole turns nearest the last, so back past the start turns it
  the other way).  Signed degrees end to end, read once by `model::turn_of` (sense, and a whole
  turn as no `sweep:`): previewed by `Sketch::set_turn`, written by `edit::set_turn` (the
  statement's own `about:` kept; an expression written whole the way it already turns).
  `tests/revolve.rs` is the gate.
- **Union, Cut, Intersect** (#162 F4): Boolean tools (`m`, `z`, `y`; `view.ts::combines`, Intersect
  is `common`) over **the solid selection, in click order**: the first body clicked is the target,
  each after it a tool (clicked again, let go), a click back on the target or Enter
  (`finishCombine`) writes `tool word target` for every tool in one splice
  (`edit::add_body_word` takes a list), the target left picked; Escape lets the tools go, then the
  target.  **One step of the body rule**, `Sketch::join_body` (`as_body` moves a sweep to a
  stock past the last, the tool joins `word`'s set, `solid::reaches` asked once it is in, since a
  `through:` reads only what its body adds; a cycle popped and refused, E041 **at that
  statement**), states each statement in `program/solids.rs` and each tool of the **preview**:
  `Sketch::preview_body` joins them on a clone (the caches' operands kept) and evaluates the
  target there, so it is what the statements make and refused where they are; a body with a
  swept operand is never previewed (FFI `gcs_solid_preview_json`, one JSON a pick, faces by the
  shared `face_groups`; `core/mesh.ts::preview`).  `box3d` draws it in the target's place, the
  tools as ghosts in their word's ink (`SketchView.combining` is the key).  `tests/booleans.rs`
  is the gate.
- **Fillet, Chamfer** (`⇧F`, `⇧C`): an edge is where two faces of an object meet, read off its
  mesh (`mesh::face_edges`: triangle sides two faces share, matched to the weld's tolerance,
  turning past `EDGE_TURN` — a tangent seam is no edge; two stretches between one pair are one
  edge, as the words read them), cached on `EvaluatedSolid::face_edges`.  `workspace::pick_edge`
  picks one where the eye sees it, never through the material (a ray at it meeting a face nearer
  the eye); `solid_edges` gives them in space for the box to light (`box3d`'s fat lines, drawn
  over everything).  The tool's clicks toggle edges, or a face (every edge of it), on one object
  (`SketchView.rounding`; another object starts over), the number asked beside the last click
  (`dimbox`'s `roundBox`, focused, the pointer-down's default prevented so it keeps the
  keyboard); Enter writes `edit::add_rounds`: the mesh's paths mapped to the document's through
  the operand routes (`EvaluatedSolid::face_route`, the report's too; `plate.near` on `pad` is `pad.plate.near`, checked against `face_paths`; a
  round's own faces have no name to write), `F round(r: d) body` per face, `a round(r: d) b` per
  edge no face covers, `round(r: d) body` for every face, `use std` made to name the word — one
  splice, held to the elaborator over both places (the library's errors are said at the `use`).
  Corners where three rounded edges meet are a ball's only in that last form.  The offered size
  is the last written, else `edit::round_size`.  `tests/edit_rounds.rs` and `app.test.ts` are the
  gates.

- **The workspace** (`overview/workspace.rs`, `app/view.ts`) is the only model canvas: one scene
  in space, every sketch on its own plane, solids and panes drawn beneath by three.js
  (`app/box3d.ts`).  A view's page reaches the eye's picture plane by **one affine map**
  (`workspace::Projection`: `Basis::lift`, the orthographic `overview::eye`), so
  `camera.ts` composes it with the eye's similarity (`Camera::through` → `ViewCam`) and no 3D
  arithmetic exists above the ABI.  **What is under the pointer is asked where the eye sees it**
  (`workspace::pick`, `nearest_point`, `overlapping`, `callout::pick_seen`): two views' coordinates
  overlap but are apart in space, so nothing is picked in view coordinates.  `w2s`/`s2w` read the
  view being worked in — the painter's (`inView`), else the current plane's; a drag reads the
  dragged point's view, a callout its dimension's (`calloutView`).  A view seen edge on refuses a
  press (`ViewCam::readable`).  **A gesture writes the digits its zoom justifies**: what a tool
  puts down — a fresh point, a place a curve passes through, a circle's radius, a rectangle's
  sides — and where a point or a radius is dragged are read through `place`/`placeIn`/
  `placeLength`, onto the decade grid no coarser than the page length one pixel stands for there
  (`ViewCam.pixel`, along the least foreshortened way; the rule is `edit::placed`, FFI
  `gcs_placed`).  The drawing takes the rounded place, so the source still says where it is, and
  nothing moves more than half a pixel.  What the core derives from those places (an arc's
  circumcentre, a fitted control polygon) and what a solve settles keep their digits.  Gates:
  `a_click_writes_the_digits_its_pixel_justifies`, and the app test beside it.  **An object's face is picked where the eye's ray meets it**
  (`workspace::pick_solid`, nearest the viewer over each object's mesh; the path is where the face
  was made, so the object index rides beside it), beneath the drawing in `whatIsAt`'s order.  A
  solid picked is a **third selection** (`SketchView.selectedSolids`, `pickSolid`), exclusive
  with `selected` and the picture, carried across an edit by name; Delete removes it, `box3d`
  lights the face and washes the rest.  **Axes are picked where the box draws them**
  (`overview::axis_segment`, one rule for both), beneath every drawn figure and never seen end
  on; the TS `Axis` proxy is kind id 16 (`kindOf` decodes a pick).  **A plane is picked by its
  pane's frame** (`workspace::nearest_pane`), beneath the axes: the place's one pane, the frame
  `overview::pane` draws, never seen edge on — and never its inside, where a press lands on what
  is drawn or starts a band (a double-click on the frame chooses the plane, as one inside it
  does).  The frame runs just outside what is drawn, so `whatIsAt` offers it last — beneath an
  object's face and the traced picture — and a press on the frame of a plane that cannot move
  selects it but, moved, is the band it would have been on bare canvas.  **A plane that can move
  is dragged as a point in space is**: the press takes hold of its origin (`PlanDrag` seen, the
  pointer offset by where the frame was grabbed), and `DragSeen` on a plane's origin moves where
  the plane stands, its own `o`.  `Drag::seen` holds the plane's axis directions while it lasts
  (its axes stand through its origin, so a step would spread into turning them), so the plane is
  **carried, not turned**, and what is drawn on it goes with it; `DragSeen`'s weak third row
  keeps the depth the drag took hold at, so it is not slid along the line of sight, while a point
  held to a line or a sphere still slides.  The seeds land in `hint(origin: …)`.  Gates:
  `a_plane_follows_the_pointer_by_its_origin_where_the_eye_sees_it` (`tests/drag.rs`),
  `a_plane_is_picked_by_its_panes_frame` (`tests/workspace.rs`), `app.test.ts`.  **Plane (`n`)
  and Axis (`x`) are buttons that drop a datum at once** (`SketchView.addDatum` →
  `edit::add_datum`, one checked splice): free with nothing selected (set off the plane being
  drawn on), else
  constrained to the selection by the table on `add_datum` — a plane through a point, three
  points, a point square to a ray, two rays, parallel to a plane; an axis through a point or
  two, along a line, a plane's normal, where two planes meet — every relation an ordinary
  statement, seeds read off the live drawing; a line square to a plane is said against the
  plane's own axes (`v0.u`, `v0.v`).  The constraint bar relates them as anything else:
  `app/relate.ts` sorts the selection into bins (`axes` beside `planes`; a `direction` slot is a
  ray, a line or an axis) and asks the core what the button's word means over the pair
  (`constraints::word_between`, `infix_op` either way round; `relate::meant`).  With colour by
  state on, the box inks the drawing's own axes and planes by state
  (`paint::inkOf`, the one ink rule: selected, highlighted, state), std's keeping their quiet ink.
  Points in space stand in no view: seen where they are (`Projection::point`; per frame
  `workspace::space_points`, `gcs_workspace_space_points`, stroked through the eye's own
  camera), drawn and picked, and dragged where the eye sees them (`PlanDrag::seen`, numeric:
  `CKind::DragSeen`, two soft rows across the picture plane and a third a thousandth as strong
  holding the depth it was taken hold at, so depth is the constraints' and a free point keeps
  its own).  A view opens drawing on `std.front`
  (`drawOnFront`), and File ▸ New is `use std`.  `workspace::Views` is what does not depend on
  the eye — each view placed in space (slot 0 a 2D sketch's front plane, `Basis::page`), the view
  each point and entity stands in (a curve, owning no point, its tool's: `Sketch::curve_view`), and each view's **place** (the first view on the same plane in
  space: `std.front`, `std.up`)
  — read once an edit (`gcs_workspace_json`); the maps are per frame (`gcs_workspace_maps`, a
  buffer).  The front end compares place ids, never maps.  The chooser in the viewport's upper right (`#plane-select`) lists `std.front`,
  `std.side`, `std.top`, then the document's planes; `choosePlane` sets `v.plane` and swings the
  eye square on (`workspace::look_at`).  A standard plane the document lacks is `pendingPlane`
  until a tool's first press adds `use std` (`edit::add_use`, `ensurePlane`) — choosing writes
  nothing.  The chooser's first entry, **None** (value `''`, `drawInSpace`), is drawing in space:
  `v.openSpace`, no plane current and the eye left where it is; a click is read on the eye's
  picture plane (`toolCam`, the eye's own camera) and mints a point in space where it was seen,
  nought deep (`workspace::unseen`, `gcs_sketch_point_seen`, `Sketch.pointSeen`), written
  `pN := point hint((x, y, z))` with no `in`.  Only Point and Line draw there (`drawsInSpace`; a
  circle, arc or curve over a point in space is E060, a region is bounded on a plane); the rest
  say to choose a plane.  It outlasts an edit and an undo, and a load, a chosen plane or a lone
  plane selected ends it.  A double-click on a pane (select tool, nothing drawn under the pointer) chooses its
  plane without turning the eye (`choosePlane(name, false)`): `workspace::panes_at` lists them
  nearest the eye first, `paneAt` takes the first the chooser offers.  Callouts are drawn for the
  current plane's place only (`showsCallouts`), and the one being written or focused.  Right-drag
  orbits, middle or ⇧right-drag pans, the wheel zooms; a trackpad's pinch (ctrl-wheel, Safari's
  `gesturechange`) zooms and its scroll pans (`gesture::scrolled`), and two touch fingers pinch
  and pan (`touchNavigation`: a lone finger's press waits `TOUCH_HOLD_MS` or `TOUCH_SLOP_PX`, so
  a pinch neither clicks a tool nor drags).  **Orbit mode** (`SketchView.orbitMode`, the Orbit
  button `o` in the workspace's lower-right corner, beside Delete) makes a primary-button press —
  one finger on a phone — an orbit, whatever tool is down; a pinch still zooms and pans.  A flat
  document opens square on to the front, one with a solid or off-front geometry from three
  quarters (`homeOrbit`);
  `workspace::bounds` frames figures and solids.  The orbit is view state (never saved, exported,
  solved or undone).  `tests/workspace.rs` and `app.test.ts` are the gates.
- The **glass box scene** (`overview.rs`) folds a multiview drawing into space: each view on its
  own plane, the object reconstructed between them.  **Nothing is solved for and nothing is
  stored** — a point in view P has view coordinates `(a, b)` (what `project`'s residual reads),
  sits at `a·u_P + b·v_P` (`Basis::lift`), and a corner tied by `project` into
  two non-parallel views is four rows in three unknowns, exact *because* the projection holds.
  "Non-parallel" is `overview::RCOND`, about a degree (past it 1/σ₃ flings a corner); `validate`
  refuses only the exactly-parallel pair.  A corner is a **pair of images and never a transitive
  class** (merging `Ff project Fa` with `Ff project F2a` collapses the object), its images
  **ordered by plane**, never by the statement.  An object edge is one both views draw, deduped by
  its **3D segment to a tolerance** (`SAME_POINT`).  Where a point *stands* is `overview::view_of`
  — its membership.  A line stands with each end where that end is.  `overview::drawable` is the
  per-kind polyline walk `svg::entity` and the workspace share, refined to `curve::flatness`.
  **Every plane is a pane**, drawn in or not; `overview::pane` is the *one* rule for its reach
  (geometry and origin, grown a little, never thinner than `LEAST_SIDE`), so face and axes agree.
  **The workspace is at least `WORKSPACE_MM` (125 mm, about five inches) across**: no pane is
  narrower, an axis reaches half of it either side (`axis_reach`), and a drawing with nothing
  drawn but at one place — a blank sheet — opens framed on its panes (`workspace::bounds`).
  **A place is one pane** (`overview::places`, `Basis::coplanar`): planes standing on one plane
  in space (`std.up` on `std.front`) are drawn as the first one's pane, reaching over what is
  drawn on any of them, so drawing on the front leaves no second frame behind
  (`planes_in_one_place_are_one_pane`); the pane that bolds is the current plane's place.
  **The box shows the objects, not the features they are made of** — a solid is the object exactly
  when nothing else is made of it (`overview::objects`).  `box3d.ts` draws what `scene3d` says
  stands in space — panes, axes, the object's creases (a `smooth` seam dropped: shading draws the
  round) and the axes — and every object's mesh
  (`mesh::grouped`), computing no coordinate; an object that does not evaluate is left out, its
  `mesh()` throwing the core's reason rather than stopping every paint, and **the banner says
  which and why** in the core's words (`overview::unbuilt`, after a conflict or an over-block;
  a swept object still being meshed is not one —
  `an_object_that_does_not_evaluate_is_reported_with_its_reason`); sketches on planes are the 2D canvas's, stroked over
  it.  Its camera is set from `v.orbit` and the eye's camera; the current plane's pane is bold, a
  material write per frame.  **`⇧⌘B` toggles the solid's surfaces** (on by default; off is a
  wireframe).  The flat shaded path (`Part::Shell`, `overview::shell`, `Item.shade`) is what
  `overview_json` offers a front end without a depth buffer.

- The front end is two layers and they are kept orthogonal.  *Geometry* is the core's: a click
  picks by `workspace::pick` (what is *drawn*, where the eye sees it, as against `point_to`, the
  idealised entity a dimension means), callouts by `callout.rs`, curves by `curve.rs`.  *Linear
  algebra* is the front end's, and the whole of it is `app/camera.ts`: the eye's similarity
  (uniform scale, translation, the y flip) and a view's affine composed with it (`ViewCam`).  A
  tolerance travels as an eye length (`PICK_PX * unit`), never pixels.  Nothing outside
  `camera.ts` multiplies by `scale` or writes a minus sign in front of a y, and nothing in `app/`
  measures a distance to an entity itself.
- Dimension callouts (`callout.rs`) are geometry: extension lines, heads, leaders, arcs, the
  label's box and the hit test are laid out in the core, the front end only strokes.  Sizes are
  screen-constant through `unit` (world length of one pixel), as is the pick tolerance.  Where a
  callout sits is a *placement*: two numbers in a frame following the geometry, automatic until
  dragged, then `Sketch.placements` document state **saved on the statement it qualifies** —
  `at (t, r)` after the dimension, `"place"` in the constraint's JSON.  **The sheet takes
  everything it shares** (issue #16, §13.1): `style .dimension` / `.reference` / `.extension` own
  ink, weight and dash (`paint.ts` asks `styleNamed`, holding no callout ink).  Never by position
  in a list, entity index, type-and-arguments selector or minted id — each fails silently or
  collides.  A placement goes with its dimension (`Sketch::remove`; in the source, the splice).
  `io::from_json` *reads* the old position-keyed `placements` table and never writes one.
  The stated number comes from `io::dimension_text`; a bare number is read through `io::reading`
  (`READING_SIG`, six digits) for the callout, `arg_text` and `describe`, since `syntax::num` is
  the *source* printer.  A literal naming its unit (`60deg`) is drawn as written with no second
  sign (`expr::names_unit`), but one past six significant digits reads at six in printed text
  (`io::read_literals`): the flattener wrote it (`flatten::fold`: `13.333333333333334mm`).
  `callout::pick` owns what outranks a callout: a point within the tolerance beats its lines, but
  not the number's box, filled solid.
- One straight dimension figure, `Pen::linear`, draws them all: it measures along a *given*
  direction, puts a head where each point falls on that line and runs an extension line out to
  each point.  `Pen::aligned` is the case where the direction is the pair's own; a run or a rise
  passes a page axis instead.
- A dimension between two points is three — `Distance`, and an `Ordinate` `along: u` or `v` —
  *stated by where the number is put*: `callout::pair_dimension` (`PairDimension`) picks the
  line (pair's own, the view's u, its v) nearest the direction from the pair's middle to the
  placement; bisector borders, no threshold, a tie to the length.  The front end asks
  (`gcs_dimension_pair_kind` → 0 length, 1 run, 2 rise) and swaps the constraint as the pointer moves
  (`SketchView.startDimension`, where a dimension is written: stated at once, number edited on
  the drawing).  Nothing reaches the undo stack until the number is accepted; Escape takes it out.
  *Nothing is solved while it is carried*: `afterEdit` skips the solve while `liveDim.placing`;
  `placeDimension` (planting click, or release of a later drag) is the one solve.
  The run and the rise are signed from the first point to the second (`(qx - px) - d`, a constant
  Jacobian), so a front end orders the pair to read positive.  In `cgraph` they are `unsupported`
  on purpose: no cluster element for the line they are measured from.

- The page is the drawing *and the source it is written as*: the program panel is a permanent
  second child of `<main>`; there is no other sidebar.  A component selected names itself in the
  status line (`describeEntity`) and brings up one floating window listing the constraints that
  reach it, and only those, open on a `subject` — the selection, else what it was last opened on,
  so focusing a constraint does not pull it away.  `openPanel` is how a clicked callout or the
  banner's culprits pick without selecting; `closePanel`, wired to `onSelect`, is a press on
  nothing.
- **The colouring is the parser's own scan** (`syntax::highlight`), not a second lexer in
  TypeScript, so a colour and the parser cannot disagree.  `Tint` names the classes and the
  stylesheet says what they look like; the front end writes one element per run, parsing nothing.
  A function of the *text*, not of an `Elaborated`, since the program is usually half-typed.
  A `.svd` is coloured by `drawing::highlight`, the drawing lexer's own scan (`gcs_drawing_highlight`,
  `core/drawing.ts`); the panel picks one by the file shown (`CodeEditor.colouring`).  Relation
  words are known by the text: those it defines or names in a `use (…)` (`relation_words`); units
  join their number's run; `expr`'s functions, constants and measures are `builtin`.
- Offsets cross the ABI in **UTF-8 bytes** and index a **UTF-16 string** on the other side
  (`gear.sv` has an em dash).  `core/program.ts::Offsets` is the conversion and `Document.adopt`
  is the **one seam** every report crosses: diagnostics, source map and coloured runs are string
  indices by the time anyone sees them.  A wholly-ASCII program builds nothing; otherwise it is a
  binary search, since offsets do not arrive in order.  Nothing sends an offset back.
- The box it is typed in is `app/editor.ts` — a `CodeEditor`, which knows nothing of Solvent and is
  handed the text and a function saying which runs are what: a `<textarea>` over a `<pre>` of the
  same text.  **The two layers must put every character in the same place**; shared CSS is not
  sufficient.  Four rules: the line height is a whole number of pixels; kerning and ligatures are
  off; a run may change the colour and **never the face**; and the copy is **translated** to
  follow the box, never scrolled (the box's scrollbars shorten its range).  `npm run overlay`
  (`web/tools/overlay.mjs`) drives headless Chrome against the *real* `CodeEditor`: metrics agree,
  colouring moves no glyph, the copy follows to both ends.  Not in `npm test`: it needs Chrome,
  and `make test` must not.
- `SketchView` holds a `Document` (`core/program.ts`), not a `Sketch`: `view.sketch` is what the
  source came to and `view.source` the document.  **A new document is one verb**,
  `SketchView.load` (File ▸ New is `newDocument`; Open and a test case too): the outgoing text is
  one undo step; `settle()` is the one list of what is in flight — a gesture, an animation, a
  carried dimension, a tool's half-collected clicks, the remembered scene — cleared before any
  swap and before the box.  `setProgram` is the *edit* of the same shape.  Undo is program text,
  comments and all.  A selection crosses a re-elaboration **by name** (`Document.nameOf` /
  `Document.entity`); a proxy dies with its `Sketch`.  `swap` is the one seam that replaces the
  drawing, and disposes the outgoing document.
  **An opening says which object it is building** (`SketchView.hold`, `buildObjects`): an
  object is evaluated the first time anything asks — the banner's `unbuilt`, `fit`'s bounds, the
  box, all at `ApproximationPolicy::Mesh` — and a rounded part is seconds, so a load held paints
  nothing and tells the shell nothing, then evaluates each object (not a swept one, the worker's)
  in turn, the loading modal saying `building NAME (k of n)` with its bar, the page painting
  between, and lets go: the shell told once, the fit it owed taken.  An example and a project's
  file are opened so (`openExample`, `chooseFile`); `loading.ts` keeps the modal through the
  building and then waits on the swept surfaces as before.  The app test beside `buildObjects` is
  the gate.
  The source catches up at exactly two seams and **never per frame**: `syncSeeds` at the end of a
  drag (`gesture::endGesture`, guaranteed numeric) and `syncSource` at the end of `afterEdit`.
  The panel is wired to `onProgram`, never to `onDragFrame`.
