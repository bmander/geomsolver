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
  the map's high-water mark), but for what a plane owns and no gesture draws: its origin, and
  its normal, which `o.plane.normal` mints after every axis a statement made
  (`a_normal_read_writes_nothing_back`).  So a reconcile after a plain solve writes seeds and
  nothing structural, and the text it leaves elaborates to the parameters the drawing holds —
  which the page's workers rely on, reading that text and writing the page's values over it
  (`set_x` refuses a vector of another length).  It cannot follow a **renumbering**, so
  deletion is `edit::remove` and not `graft::without`.
- Deletion, copy and paste are one rebuild walk (`graft`): every surviving entity is
  renumbered into the destination and every reference follows, and a constraint comes along
  exactly when all its entities did.  `without` keeps what is not deleted, `copy` keeps the
  selection (so a clipboard is an ordinary sketch document), `paste` grafts one sketch onto
  another at an offset, and a drag's `Part` is the same walk.  JSON is now the *export* format,
  derived rather than canonical; `io::dumps` is still what the Rust tests and the benchmarks
  compare against.
  - **The walk's answer is a `Renumbering`**: each source entity to the one it became, each
    stated constraint to its copy, and what was made in the order it was made.  `rebuild` (a
    sketch of its own: the source's unit, presentation and root choices) and `graft_onto` (added
    to a document, in that document's unit, onto the planes it already has) say which they
    are; the destination's being empty says nothing, so a paste into a new document stays a
    paste (`a_paste_into_an_empty_document_lands_in_its_unit`).  `Part` pairs the two sketches'
    points, entities and numbers from it — a constraint's own unknowns by the constraint table,
    whatever kinds it names (`a_part_pairs_a_contacts_place_on_a_traced_body`).
  - **What a value refers to is the value's own to say**: each kind the walk copies whole has a
    `remapped` beside it (`FaceE`, `SolidDef`, `MotionDef`, `Held`, `CurveE`, `Constraint`, …),
    naming every field, `None` where something it cannot be without did not come along (`Held`
    degrades to `Held::Not` instead).  A new field stops the build there until it is decided;
    a new flag on a constraint travels unless `Constraint::remapped` says otherwise.  Every
    field of `Sketch` is named in `graft`'s ledger — carried by a step, minted again by the
    destination, or left behind and said so — so a new one stops the build there too.
  - A rebuild that removes nothing keeps every entity where it was, every stated constraint
    and every picture, in every document of the corpus
    (`a_rebuild_that_removes_nothing_keeps_everything`).  What a removal takes with it reaches
    the removed entity through `Sketch::children`, or through one of the three things the walk
    reads that `children` does not list — a face's plane, a curve's trim, a region term's set
    probe (`what_a_removal_takes_is_what_depends_on_it`); `copy` and `Part` walk `children`
    alone, so closing one of those is a change to `children`.
- **Delete takes everything selected** that has a statement of its own (`edit::remove`): what a
  component or a block made stays, counted in `Edit::kept`, and the rest goes — so a band swept
  over the drawing that also caught `std`'s datums deletes the drawing, and the status line says
  what stayed.  Refused, saying why, only when nothing would go.  The key and Edit ▸ Delete are
  one function (`main.ts`'s `deleteSelection`): the focused constraint, else the drawing's
  selection, the solids or the picture.  Gate:
  `deleting_beside_what_a_component_made_takes_the_rest`, and the app test beside it.
- **Space selects nothing** (`SketchView.selectNone`): the drawing's selection, the solids and the
  picture let go, and with them the focused constraint (`onSelect` drops it); the tool stays down,
  an edge tool's number put away.  Gate: `app.test.ts`'s Space test.
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
  stock past the last, the tool joins `word`'s set — a tool already in it joins nothing more, #415 —
  `solid::reaches` asked once it is in, since a
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
  over everything).  **The tool's picks are the selection's** (`SketchView.rounding` is read off
  `selectedSolids`, `solids::roundingOf`): edges and faces selected before it was taken up are
  where it starts, and the number is asked at once; its clicks toggle edges, or a face (every
  edge of it), on one object (another object starts over; taken up, the selection keeps the
  object picked last, `onePicked`), and a body picked whole is its every edge (`add_rounds`'s
  `whole`, the word over the body — so a body already rounded, whose round faces have no name,
  is still a body).  The number is
  asked beside the last click (`dimbox`'s `roundBox`, focused, the pointer-down's default
  prevented so it keeps the keyboard); Escape lets the picks go, the tool still down; a write
  lets them go too.  Enter writes `edit::add_rounds`: the mesh's paths mapped to the document's
  through the operand routes (`EvaluatedSolid::face_route`, the report's too; `plate.near` on `pad` is `pad.plate.near`, checked against `face_paths`; a
  round's own faces have no name to write), `F round(r: d) body` per face, `a round(r: d) b` per
  edge no face covers, `round(r: d) body` for every face, `use std.rounds` made to name the word — one
  splice, held to the elaborator over both places (the library's errors are said at the `use`).
  Corners where three rounded edges meet are a ball's only in that last form.  The offered size
  is the last written, else `edit::round_size`.  `tests/edit_rounds.rs` and `app.test.ts` are the
  gates.

- **The workspace** (`overview/workspace.rs`, `app/view.ts`) is the only model canvas: one scene
  in space, every sketch on its own plane, solids and panes drawn beneath by three.js
  (`app/box3d.ts`).  A view's page reaches the eye's picture plane by **one affine map**
  (`workspace::Projection`: `Basis::lift`, the orthographic `space::eye`), so
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
  was made, so the object index rides beside it), beneath the drawing in `whatIsAt`'s order, an
  edge the eye sees within the pick's reach (`pick_edge`) before the face it bounds.  **A part of
  an object picked** — a face, an edge (`SolidPick.edge`, its two faces), or the body whole (no
  face, no edge; `partOf`) — is a **third selection** (`SketchView.selectedSolids`,
  `pickSolid`, toggled part by part), exclusive with `selected` and the picture, carried across
  an edit by the object's name; Delete removes the objects it is of (a face or an edge has no
  statement of its own), a Boolean tool takes each object once whole (`bodiesPicked`), and
  `box3d` lights a face or an edge selected in the selection's ink, a body all over a little
  less.  **The pointer lights what a click would take, as it moves** (`gesture::hover` →
  `SketchView.hovered`, a `Hit`): `whatIsAt` asked on every move, a figure or point lit in the
  highlight's ink (`SketchView.lit`), an object's edge, face or body in the box — an edge tool
  its edge or face, a Boolean its body — and put out when the pointer leaves.  **⌘-click (Ctrl
  off a Mac) lists everything under the pointer** (`workspace::under`, FFI
  `gcs_workspace_under_json`, `underSeen`; `SketchView.hitsAt`): what is drawn first (points,
  figures, axes, panes' frames, each by how near), then the objects' edges and faces nearest
  the eye first — an edge behind the material listed too, `seen: false`, said hidden — then the
  objects whole; an edge tool lists only objects' parts, a Boolean only objects.  What a tool
  picks is said once (`view.ts::picksAs`), and the hover, the list and the selection a tool is
  taken up over all read it.  One is taken
  at once; several are handed to the shell (`onChoose`, `app/choose.ts`), each row lit as the
  pointer or the arrow keys are on it (`hoverHit`), the one chosen taken as a click on it would
  be (`takeHit`; shift adds).  `tests/edit_rounds.rs`'s
  `everything_under_the_pointer_is_listed_the_drawing_first_then_nearest_the_eye` and the app
  tests beside the Fillet tool's are the gates.  **Axes are picked where the box draws them**
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
  `a_plane_is_picked_by_its_panes_frame` (`tests/workspace.rs`), `app.test.ts`.  **A shift-drag
  turns the body** (`gesture::toggleOrTurn`, `solve::TurnDrag`): a shift-press on a figure, a
  point or a plane's frame let go within `TURN_SLOP_PX` is the click that toggles it in the
  selection; moved, it turns the selection (the figure alone, selected, where it was not in it)
  rigidly — in the plane it is drawn in, as the pointer sweeps round its middle, or, with a
  plane, a frame, an axis or a point in space in it, in space, pushed round as the workspace is
  orbited (`ORBIT_PER_PX` about the eye's up and right) about its middle or a lone plane's
  origin.  What is drawn on a turned plane goes with it; the seeds land as a drag's do (a plane's
  in `u: hint(dir: …)`, `v: hint(dir: …)`).  Shift is asked before whether a plane can be
  dragged, so a plane held at its origin still turns there; on the frame of a plane nothing of
  which turns, a shift-press that moves is the band (the plane toggled in its base), and on
  anything else that cannot turn, the status line says so.  **Plane (`n`)
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
  a pinch neither clicks a tool nor drags).  **Orbit mode** (`SketchView.orbitMode`, key `o`, and
  the Orbit button in the workspace's lower-right corner, shown only under `pointer: coarse` — a
  mouse orbits with its right button; Delete is Edit ▸ Delete) makes a primary-button press —
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
  stands in space — panes, axes, the axes and the curves traced in space (`IN_SPACE`: `axis`,
  `traced`; a spline over points in space is one, #363 S3) — and every object's creases (a
  `smooth` seam dropped: shading draws the round) and mesh (`mesh::grouped`) in its own
  coordinates, placed by the matrix the core gives (`core/mesh.ts::own`, `placement`; #412),
  computing no coordinate; an object that does not evaluate is left out, its
  `mesh()` throwing the core's reason rather than stopping every paint, and **the banner says
  which and why** in the core's words (`overview::unbuilt`, after a conflict or an over-block;
  a swept object still being meshed is not one —
  `an_object_that_does_not_evaluate_is_reported_with_its_reason`); sketches on planes are the 2D canvas's, stroked over
  it.  Its camera is set from `v.orbit` and the eye's camera; the current plane's pane is bold, a
  material write per frame.  **A section** (`⇧X`, View ▸ Section; `SketchView.section`) cuts
  every object by a plane — the one being drawn on when it is turned on, then whichever the
  section bar (`section-bar.ts`, the viewport's upper left) chooses among `planeChoices`, a
  standard plane the document lacks read from the library's — moved along that plane's normal
  over the range the objects reach (`workspace::reach_along`, `gcs_workspace_reach_along`) and
  flipped to keep the other side.  View state as `hidden` is: **set on the core's sketch**
  (`Sketch::cut`, `gcs_sketch_set_cut`, `core/section.ts`) on every drawing the view is handed and
  again as the pose moves its plane, carried across an edit by the plane's name, let go by a load
  or a name that no longer reaches a plane.  The box clips every object's materials (faces,
  creases, lit edges, a Boolean's preview) by the one plane the core gives
  (`gcs_sketch_cut`, three's `Plane`), and hangs each object's cut face and outline
  (`gcs_solid_section`, one buffer: the face's count, the face, the outline), in its own
  coordinates, in a group its matrix places; the face is hatched in device pixels and drawn a hair
  toward the eye.  **A pick passes by what the cut took away** (`workspace::crossings`,
  `edges_near`): a face hit in front of the plane is not met, an edge is offered only where the
  cut keeps it, and a ray that meets the plane inside the material (`EvaluatedSolid::contains`)
  meets the object whole — its cut face is no face of the document, so `under` lists the object
  and not a face.  `app.test.ts` ("a section cuts the objects") is the gate.
  **`⇧⌘B` toggles the solid's surfaces** (View ▸ Show solid; on by default; off is a
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
  The stated number comes from `io::dimension_text`; a bare number is read through `expr::reading`
  (`READING_SIG`, six digits) for the callout, `arg_text` and `describe`, since `syntax::num` is
  the *source* printer.  A literal naming its unit (`60deg`) is drawn as written with no second
  sign (`expr::names_unit`), but one past six significant digits reads at six in printed text
  (`io::read_literals`): the flattener wrote it (`flatten::fold`: `13.333333333333334mm`).
  `callout::pick` owns what outranks a callout: a point within the tolerance beats its lines, but
  not the number's box, filled solid.
- **A part's dimensions are drawn in the part's own file** (§14.4): a constraint stated in a
  module with a `preview` (`Module::previewed`), or reached through an instance written in one, is
  `Constraint::elsewhere`, set in `program::elaborate`'s `finish`, and the full callout layout
  leaves it out as it does a `repeated` copy — asked for by id (focused) it is still drawn, and a
  sheet's annotations and `dimensions` draw what they name.  The V-twin's assembly draws its own
  two numbers, not the ~290 of the parts it imports; opening `components/cylinder.sv` draws the
  cylinder's.  A module with no drawing of its own is vocabulary (`std.shapes.Hex`,
  `hardware.BoltPattern`), drawn where the document calls it.  `tests/callout.rs`'s
  `a_part_is_dimensioned_in_its_own_file` is the gate.
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
  second child of `<main>`; there is no other sidebar.  **The panel shows one pane at a time**,
  chosen on its head (`program.ts::showSide`, the `#ppanel-side` chooser): the program, the model
  outline, or — offered only while a folder is open (`offerProject`) — the project's files; the
  stylesheet hides the others by `#ppanel[data-side]`, and a head button belongs to the pane it
  acts on (Compile, Isolate/Show all).  The outline reads its tree only while shown
  (`OutlinePane.show`).  Turning from the program applies a draft as a blur does; a way to
  source (`goToSource`) and a file picked in the project's tree turn the panel to the program.
  On a narrow screen the panel is a drawer above the footer: the head's chevron folds it, and
  choosing a pane opens it (`bindDrawer`).  A component selected names itself in the
  status line (`describeEntity`) and brings up one floating window listing the constraints that
  reach it, and only those, open on a `subject` — the selection, else what it was last opened on,
  so focusing a constraint does not pull it away.  `openPanel` is how a clicked callout or the
  banner's culprits pick without selecting; `closePanel`, wired to `onSelect`, is a press on
  nothing.  The banner names a conflict's culprits to remove (`lists.ts::refreshBanner`); a
  requirement out of reach of what drives it (`Diagnosis.outOfReach`, #510 P7) it says with the
  core's words for why — `…: out of reach of the dimensions that drive it: 4.79mm is the least
  they reach here` — since what is wrong is the number stated, not a statement to delete.
- **The colouring is the parser's own scan** (`syntax::highlight`), not a second lexer in
  TypeScript, so a colour and the parser cannot disagree.  `Tint` names the classes and the
  stylesheet says what they look like; the front end writes one element per run, parsing nothing.
  A function of the *text*, not of an `Elaborated`, since the program is usually half-typed.
  A `.svd` is coloured by `drawing::highlight`, the drawing lexer's own scan (`gcs_drawing_highlight`,
  `core/drawing.ts`); the panel picks one by the file shown (`CodeEditor.colouring`).  Relation
  words are known by the text: those it defines or names in a `use (…)` (`relation_words`); units
  join their number's run; `expr`'s functions, constants and measures are `builtin`.
- **The names a text gives and reads are the core's** (`syntax::symbols`, FFI
  `gcs_program_symbols`, `core/program.ts::symbols` → `Symbols`): every name, the declaration it
  names, and each place it is named, given (`def`) or read — the index an editor feature stands
  on, the occurrences under the caret now and going to a definition or renaming next.  A function
  of the *text*, as the colouring is, and **standing on the colouring's own scan**: a word tinted
  `Def` gives a name, a plain word reads one, a `Type` before its `(` calls a component, a
  `Relation` the file defines or imports reads that word, a `Label` in a component's or a word's
  call names the formal it fills; so a colour and the index cannot disagree about a word.  On that
  the language's scopes: a component's, part's, interface's or relation word's body holds its
  formals and declarations, a set's, a `claim { … }`'s and a `preview`'s keep theirs, a repetition
  or an `in` block shares the scope around it but for its header's binders (as `integral(… over p)`
  keeps its own), and a name is the innermost declaration of it read outwards.  **A member is
  reached through what it is read of**: through an instance (`t0.mid`, `Rung(…).mid`) or a formal
  typed by an interface it is the declaration in that body; where the text declares none (`l.p1`,
  `std.front`) it is one symbol per name per thing read of; a group's label is its member;
  `next.v` is `v`.  A name nothing gives is one `unknown` symbol per name per body, so a half-typed
  name lights its other spellings.  `tests/symbols.rs` (the corpus indexed end to end, each
  occurrence its symbol's word) and `web/src/test/occurrences.test.ts` are the gates.
- **The name under the caret is lit wherever else it is named** (`app/occurrences.ts`), as every
  editor does: the caret in a name or just past it (`rim|`), or a selection of exactly the name;
  given in a stronger ink (`.occurrence-def`) than read (`.occurrence`); after a short pause
  (`PAUSE`), let go at once on an edit; F7 and ⇧F7 step to the next and the previous place, round
  the ends.  On a program only — a drawing or a text file lights nothing (`Occurrences.names`).
  `CodeEditor` stays ignorant of all of it: it keeps **marks in layers** (`setMarks(marks,
  layer)`; the default layer is the panel's picked statement and verdicts, `occurrences` this) and
  tells **the caret** (`onCaret(edited)`, once per move or edit however many events report it),
  which is all a feature reading the word under the caret needs.
- **The text and the drawing are one thing, read two ways** (`app/names.ts`): what is selected in
  the workspace — an entity, a body, by a press, a band or the outline — is lit wherever the text
  names it, in the occurrences' inks and a layer of its own (`selected`, which an edit lets go
  of); and the caret *put* on a name — a click, an arrow, F7, never typing (`Occurrences.onName`)
  — selects what it names, as a press there would (`onSelect`, `onChanged`), a hidden row shown
  first.  One join of two core answers, the outline's rows and the text's names: a row is the
  symbol its statement gives under the last part of its path (a module's datum, written nowhere
  here, the member its whole path reads, `std.front`), named where it is written as a part of
  that path — `t0.mid` and the bare `mid` inside `Rung` are `t0`'s, `t1.mid` is not, and the
  `mid` `Rung` declares is every instance's.  An instance named selects every member it made; a
  name that makes nothing (a number, a formal, a module's datums whole) leaves the selection
  where it was.  Only while the box holds the document's own text (`names`): a draft's offsets
  are its own, which is why an apply reprints the box with the seeds its solve committed.
  `web/src/test/names.test.ts` is the gate.
- Offsets cross the ABI in **UTF-8 bytes** and index a **UTF-16 string** on the other side
  (`gear.sv` has an em dash).  `core/program.ts::Offsets` is the conversion and `Document.adopt`
  is the **one seam** every report crosses: diagnostics, source map, coloured runs and names are
  string indices by the time anyone sees them.  A wholly-ASCII program builds nothing; otherwise
  it is a binary search, since offsets do not arrive in order.  Nothing sends an offset back.
- The box it is typed in is `app/editor.ts` — a `CodeEditor`, which knows nothing of Solvent and is
  handed the text and a function saying which runs are what: a `<textarea>` over a `<pre>` of the
  same text.  **The two layers must put every character in the same place**; shared CSS is not
  sufficient.  Four rules: the line height is a whole number of pixels; kerning and ligatures are
  off; a run may change the colour and **never the face**; and the copy is **translated** to
  follow the box, never scrolled (the box's scrollbars shorten its range).  `npm run overlay`
  (`web/tools/overlay.mjs`) drives headless Chrome against the *real* `CodeEditor`: metrics agree,
  colouring moves no glyph, the copy follows to both ends.  Not in `npm test`: it needs Chrome,
  and `make test` must not.
- **The page says it is arriving, and asks for the core first** (`index.html`, `core/wasm.ts`,
  `app/shell.ts`).  The core is most of what a visit waits for (megabytes, against the bundle's
  hundreds of kilobytes), so `index.html` *preloads* `dist/wasm/gcs.wasm` beside its own script
  rather than after it has run, and `initCore` fetches that same address and compiles the
  module **as it arrives** (`fromResponse`, `instantiateStreaming`; a host not saying
  `application/wasm` gets it compiled whole).  Its `progress` hears bytes and no words — the
  bytes counted on a clone of the body, against the module's size, written into the bundle as
  `GCS_WASM_BYTES` by `tools/bundle.js` since a compressing host's Content-Length is the
  compressed size; told no size, it counts nothing (`wasm.test.ts`).  The words are the shell's:
  the modal is in the page before any script, sweeping (`loading the sketcher…`); the download
  is 90% of the bar, rewritten only when its label changes, then `starting the solver…` and
  `starting the sketcher…`, painted (in a visible tab) before the page wires itself and reads
  its first document.  A page opening on an example asks for its sources while the core arrives
  (`remote.prefetch`); the packed sources are asked for once a page, and only a file the host
  lists at `examples/index.json` (the demo server) is asked for — a static host lists none, and
  asking would be a refusal per file down the chain of uses.
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
- **The scenario chooser** (#510; `app/scenario-card.ts`, `app/scenario-box.ts`,
  `SketchView.scenario`, `chooseScenario`).  Beside **Sketch on**, in the viewport's upper right
  (`#view-picks`), `#scenario-pick` lists the document's load scenarios by name (*None*, then each
  in the order written, an extended one `heavy : shelf`; `Document.scenarioNames`), hidden for a
  document with none and inked as failing while one of them fails.  **Choosing is view state**,
  as the plane chosen is: it writes nothing and solves nothing, is kept across an edit — the
  program panel's too — by its name, let go where the name is gone, and by a load.  Chosen, its
  **card** unfolds beneath the chooser: in its head the **exaggeration** the picture is drawn at
  (`#scenario-scale`, by equal ratios from true scale at its left to ten times the picture's own
  factor at its right, opening at the picture's own — `SketchView.scenarioScale`, null for it),
  *Animate* (the picture swings from rest to its factor and back) and *Mesh* (the edges of the
  mesh that was solved); the colour ramp the moves are read by, 0 to the largest in the core's
  words; what the reading says (still reading, why a body is not drawn, the scenario's words);
  and a row per line of the scenario — the line as written, by its span, a load's value, a
  requirement's sentence in the verdict's ink — which puts the editor's caret on its line
  (`goToSource`).  **The reading arrives a level at a time** (analysis.md, "A scenario read
  progressively"; `ScenarioReading.receive`, `Reading.levelText`): each level's answers and
  picture replace the last's at once, and while a finer one is read the card says which level
  stands (the core's `levelText`), its rows' sentences dimmed (`provisional`) and a requirement
  a coarser level could not answer still *reading…*; the footer's verdict and the chooser's
  failing ink wait for the last level, so a coarse reading never flashes a failure the reading
  does not make (`a scenario read in a worker arrives a level at a time, the last its
  reading`).  **A filled body's drive is `solventc`'s** (#567, analysis.md, "The page does not
  drive a filled body"): the page solves on its own thread, so its core compiles no row for a
  driving deflection of a solid that is no prism (`bodies::DRIVES_FILLED`) and its card's
  sentence is the core's, judged as a claim and saying `` `solventc` drives it, the page does
  not ``; nothing in the page decides it.  **In the box** (`ScenarioScene`) every coordinate is the core's (the marks,
  the picture: analysis.md, "A scenario drawn"); a glyph is chrome made in pixels about its
  anchor and scaled by `unit` every frame, so it keeps its size as the eye zooms — a plate for
  `fixed` on a face (a block at a point or along a line), a cone for `pinned` pointing at its
  place (on a roller where it holds one direction), an arrow ending at its place for a load and
  for a pressure, faces tinted, a probe at a deflection's point inked by its verdict, the words
  as sprites beside their place.  The deformed bodies are **morph targets**: three.js draws
  `rest + s·moved`, the factor written per frame, so the slider needs no round trip to the core
  and no coordinate is worked out in the page; each vertex coloured on the ramp by its move as a
  fraction of the largest.  A solid the picture deforms is ghosted beneath it.  A gesture moving
  the drawing hides the scenario until the edit settles and the worker reads it again — a drag
  too, read again where it is let go (`endGesture` asks `readBehind`, as `afterEdit` does), so
  the picture never comes back as it stood before the drag.
- **The timeline** (#398 L, [0.146]; `app/timeline.ts`, `SketchView.instant`, `showInstant`):
  a row per declared parameter in the viewport's lower left, its interval a slider whose solid
  part is what the march reached, hatched past it, the singular instant marked with the core's
  words (`March.singular.text`); the readout takes an instant as written (`90deg`, read by
  `follow::instant`); Run steps the drawing along the reach and round; Home puts it back.  **The
  instant is presentation, as a wobble is**: view state (`instant: { name, at }`, `at` null at
  home), carried across an edit by the parameter's name and let go by a load, never saved.
  `showInstant` clamps to the reach and stands the drawing (`follow::stand`).  **An edit is made
  at home**: `afterEdit` sends the drawing home before it solves, writes the seeds there
  (`syncSource`) and stands the drawing back at the instant shown, following each parameter
  again to do it; and the core writes no seed from a drawing standing at an instant, whoever
  asks.  **The march is off the edit path** (#412): an edit at home follows nothing before it
  returns (`followLater`); the march is taken when it is needed — `SketchView.marches` and
  `marchOf` follow then: an instant shown, the timeline scrubbed or run, a point dragged along
  it — or once the page is idle after the edit (`requestIdleCallback`, else a timer).  Until
  then the timeline reads `followedMarch` and shows the interval, live, without its reach or
  singular instant.  **Dragging a driven drawing scrubs it**: a press on a
  point of a drawing with a followed parameter takes hold of its march (`gesture::takeScrub`,
  the parameter shown first; `Scrub` over `gcs_scrub_*`), and each move stands the drawing at
  the instant its point comes nearest the pointer — the crank turned by its pin.  A scrub is
  `transient`: nothing is written back and nothing goes on the undo stack.  A point that does not
  move as the parameter runs is dragged as before at home and, shown at an instant, not at all
  (the status line says it does not move).  A constraint reading the parameter is listed at the
  value it has at the instant shown (`onTimeline` refreshes the lists).  The gate is
  `web/src/test/timeline.test.ts`: `mechanisms/four_bar.sv` scrubbed to a quarter turn on its
  closed form and on the `.svd` stating that instant, home again with the source untouched; an
  edit at an instant stood back there with its seed written at home; the pin dragged round with
  `Scrub.live` back to zero; the triple rocker's timeline clamped at its limit; an edit at home
  followed only once idle or asked, and one at an instant stood back there; the V-twin scrubbed
  with nothing cut again in the box.
- **The model outline** (#435; `overview::outline`, FFI `gcs_elab_outline_json`,
  `Document.outline`, `core/outline.ts`; the pane `app/outline.ts`, its verbs `app/hidden.ts`):
  every instance the document makes and its members, as a tree — an *outline*, not a feature
  tree, since Solvent has no ordered history.  **The tree is the core's**, read off the source
  map: the root body's statements in source order; an instance `crank : Crank` with its members
  under it; a block or ring `cyl[0‥2]` with a row per copy (a copy that made one row is that
  row); what a statement made beside its own entity — a line's implicit ends, a plane's origin, a
  sweep's face — under it; an object where it was made with its operands (features, cutters, a
  sweep's stock) under it, `operand`, listed for navigation only and taken from where they were
  written; and a module's datums (`use std`) one group, last.  A row's `path` is what it is
  carried by — the source map's name, `public_path` for a copy — empty for an anonymous row
  (`line in pin`), which is listed and not carried, as a selection of it is not; its `span` is
  its statement's, none for a module's.  **Hidden is view state**, the orbit's kind: never
  written to the source or the `.svd`, never saved, solved or undone; carried across an edit
  and an undo **by path** (`SketchView.hiddenRows`, re-applied in `swap`: `hidden.carryHidden`)
  and let go by a load.  Hiding a row takes out what it *made*, its subtree — a point declared
  at the root and read by a hidden component's line stays — and still solves.  The core is told
  whole (`Sketch::set_hidden`, `gcs_sketch_set_hidden`) and **hidden means not drawn, not lit and
  not picked**: `scene3d_of` (panes, axes, traced curves, objects), `workspace::pick`,
  `pick_solid`, `pick_edge`, `under`, `overlapping` (so a band or Delete never takes it),
  `nearest_pane`, `nearest_point`, `panes_at` and `bounds` pass it by (`Sketch::seen`,
  `overview::shown_objects`; `objects` stays the document's, what is exported), `paint.ts` skips
  it, the box leaves out its objects, and a callout goes once every entity its constraint reads
  is hidden.  Beside the rows, the **kinds** — objects, planes, axes, sketches, points, fixed
  marks and dimensions — are each taken out everywhere by View ▸ Show, and a closed part's
  insides by the assembly's rule (both below); the core is told all three together.  Only
  objects are hidden of the solids; an operand has no eye.  Hiding what is selected deselects
  it; selecting a hidden row shows it (and only it: the rows above it shown,
  their other children hidden in their stead); choosing a hidden plane, or drawing on it, shows
  it.  **A right-click** that does not turn the workspace (let go within `MENU_SLOP_PX` of its
  press, `gesture::orbitGesture`) opens a menu on what it is on (`SketchView.contextAt`, as a
  press finds it; `choose.ts::onContextMenu`; an object's face or edge stands for the object):
  **Show definition**, the editor's caret on the span of the row that made it
  (`hidden.rowOf`, `program.ts::goToSource` — a member's statement in its definition, offered
  only where the row has a span); **Show in model viewer**, the panel turned to the outline
  (`program.ts::openSide`) and the tree opened to that row, brought on screen, focused and lit a
  moment (`OutlinePane.showRow`), the selection left as it is; **Hide**, which hides that row
  (`hidden.hide` — something no row with an eye made is said so); and **Show all** while anything
  is hidden.  The pane: an eye per row (dimmed under a hidden row, half where some row under it is
  hidden); an icon of what the row is (`outline.iconOf`: its row kind, or for an entity the
  core's kind word for what it stands for — a point, a line, a plane, a face); a click selects (⇧ toggles), a hover lights as the ⌘-click list's rows do, a
  double-click frames the row's subtree (`workspace::bounds_of`, hidden or not, FFI
  `gcs_workspace_bounds_of`), ↗ puts the editor's caret on its span (`program.ts::goToSource`);
  **Isolate** hides all but the selection and the rows that made it and **Show all** clears it
  — `I` and `⇧I` while the tree has the keys (`I` is Coincident's elsewhere), Edit ▸ Isolate, and
  `⇧I` everywhere.  A selection made in the workspace opens the tree to its row
  (`hidden.selectionRow`).  The tree is rebuilt only when the outline's shape changes (spans
  aside, which every splice moves), never on a drag, and a row's children are made when it is
  first opened.  Gates: `tests/outline.rs` and `web/src/test/outline.test.ts`.
- **What the workspace shows, by kind** (#678; `model::Kinds`, `Sketch::set_kinds`, FFI
  `gcs_sketch_set_shown_kinds`; `SketchView.kinds`, View ▸ Show in `main.ts`): beside the rows the
  outline hides, the **kinds** — objects `⇧O` (the whole object, where `⇧⌘B` is only its surfaces),
  planes `⇧P` (a pane and its frame), axes `⇧A`, sketches `⇧S` (lines, circles, arcs, curves,
  regions and traces, and their points), points `⇧T`, fixed marks `⇧K` (off, a fixed point is drawn
  as any other) and dimensions `⇧D` (the callouts; `showDimensions` is this kind) — each turned off
  everywhere at once.  An entity is shown while every kind it is drawn as is (`Kinds::of`: a point
  is a sketch's and a point) or it is shown whatever its kind (`Sketch::shown_anyway`), and a kind
  off is held to hidden's rule — **not drawn, not lit, not picked**: `Sketch::shown` answers for
  both, so `seen`, `shown_objects`, every pick, `under`, `overlapping`, the panes and the bounds
  pass it by, `callout::layout_selected` lays out (and so picks) no dimension unless asked (the one
  being edited), a callout goes with the entities it reads, and the painter skips what the core says
  it passes by (`Sketch::unseen`, `gcs_sketch_unseen_json`, read into `hiddenEnts`).  View state, as
  hidden is: never written to the source or the `.svd`, saved, solved or undone, carried across an
  edit and kept across a load.  **A tool shows the kinds it needs while it is down**
  (`hidden.toolKinds`: a drawing tool the sketches and points it snaps to, Extrude the sketches,
  Revolve those and the axes, a Boolean or an edge tool the objects); a plane chosen to draw on, or
  drawn on, and a row chosen from the outline are shown whatever their kind
  (`SketchView.shownAnyway`, which a kind turned on or off lets go).  Gates:
  `a_kind_turned_off_is_not_picked_under_or_framed` (`tests/workspace.rs`) and
  `web/src/test/shown.test.ts`.
- **An assembly opens on its parts** (#679; `hidden.ts`): a *part* is an instance that makes an
  object, at any depth — the core's outline marks it (`outline::Node::part`, the row's `part`;
  `an_instance_that_makes_an_object_is_a_part`) — and **what a closed part makes, other than its
  objects, is not drawn** — its planes, axes, sketches, points, fixed marks and callouts — told the
  core as hidden is.  The line is *made inside the instance*, not Onshape's *used by an object*:
  what the root draws is the assembly's own layout and is drawn, a profile the root makes a solid of
  included, and a document with no part opens as it always did.  **Opening a part** — a double-click
  on its object (`hidden.partToOpen`; a double-click on the part open, or on an object the root
  made, draws on its face as before), *Open* on its right-click menu or the `open` on its outline
  row, or selecting one of its members' rows — draws its insides and only its own (a part inside it
  stays closed), and **dims the rest of the assembly** rather than hiding it (`DIMMED`: the box's
  other objects, the canvas's other figures); Esc, a click on nothing, *Close part* or its row's
  `close` closes it.  The part open is carried by path across an edit and let go by a load.  The
  outline's eye overrides the rule either way: a member of a closed part reads shut, the part half,
  and shown by its eye it is drawn (`SketchView.shownRows`, which Inside parts turned on or off lets
  go); View ▸ Show ▸ Inside parts draws every part's insides.  **Shown on its parts**
  (`Sketch::on_parts`), the datums left are drawn small and quiet: a pane is the box of what is
  shown on it, no narrower than half the objects' largest side (`overview::panes`, `PART_DATUM`)
  rather than the workspace, an axis reaches as far, the box draws them faint (`QUIET`) and labels
  each pane with its plane's name.  Gates:
  `an_assembly_on_its_parts_draws_its_datums_the_size_of_its_objects` (`tests/workspace.rs`) and
  `web/src/test/shown.test.ts`.
