# Solids

Part of the [contract](../../CLAUDE.md) the code is held to. Read it when you touch `solid.rs`,
`csg.rs`, `mesh.rs`, `hidden.rs`, `gltf.rs`, or `hypoid.sv`. An issue number (#n) or a spec version
([0.n], see the [language changelog](../solvent-changelog.md)) says where a rule came from.

**The body rule's third side (0.22):** `tip common body` keeps what the body has in common with
`tip`: a solid is its stock, plus `union`, minus what `cut`s it, within what it is in `common`
with. The word was `bound` through 0.84 (#270: a bound is a dimension choosing a root); `X bound B`
is refused naming `common` (`parser/statements.rs`), and `bound` stays reserved. The Rust names
keep the old word: `SolidDef::Body` carries `bound`; `Term::Inter`; facets, fields and the CAD
recipe (`"bound"`, `BRepAlgoAPI_Common`) all evaluate it. Union first; `cut` and `common` commute;
a swept solid may only be `cut`. `common` and `union` (0.32) are body words, not names;
`coincident` between two solids is refused. Spiral bevel (**one file, `spiral_bevel/hypoid.sv`**, #217: no `project`, no `hint` — every root chosen by a
word, the layout solved from them alone; README walks it). **The pair is one component**,
`pair := HypoidPair(std.top, std.y, std.z, …)` over the root's `param`s (std's datums its
arguments, its unknowns `mn`, `rp`, `pa` formals the call leaves out), its steps components below
it — `GearCone`, `ToothTrace`, `PinionCone` (each cone's axis `.ax`; `fixtures::gear::axis`),
`Blank`, `FaceMill` (`mill.ax`, the flank cones, levels and reach), `Tooth`, `Space`, `Member`,
`Topping` — so every name is under `pair.` (`pair.pinion.body`, `pair.gear_cone.M`); a test reads
the recorded rows under their old labels. Each
member's blank is a region (`Blank`: the pitch cone, a `std.Cone` about the member's axis, its
half left unknown so it settles from the pose, grown by the addendum (`std.Offset`) and its
outside by the back depth (`distance(<= d, side: inside)`), `std.Sphere`s about the apex the half face width either
side of M (`rim.p2 distance(w / 2, side: outside) toe`), sized by the solved layout — a region term
reads an unknown, `model::TermNumber`; the end relief two `std.Chamfered` bounds in the same
region, lowered as conics); its faces are `tip`, `heel`, `toe`, `back` (the sets its body uses),
and `tests/region_blank.rs` holds the region to their closed forms read off the layout's points
(faces, volume, material). The depths are values of the unknown normal module (`dedendum := 1.25 *
mn`); the normal module is O's distance from the trace normal over R / m. `tests/envelope/paired.rs`
reads the members built at one tooth space each (`fixtures::gear::one_space`; their topology read
at 2 mm by tooth counts, the design being proportional to its module) and holds each face to the
analytic envelope its name says (`face_source`), its seams, corners, edges and face loops made of
their parts where the built member says the faces meet; the gear's inner mate is read unplaced, one
tooth over (its space cutter's `neighbor` is the inner crown a crown pitch round). Its inputs are
`param`s at its head (`shaft_angle`, `offset`, E = 0 is bevel; `fixtures::gear::configure` sets
them where the parser says their values stand); the pinion's cone solves against the gear's;
members roll at `N_c / N` off the gear's triangle, not `1 / sin`. **No axial views** (#215): the
gear's triangle O–M–F and the pinion's two axes stand in space square to P on their generators
(`O level(across) foot`; the pinion's axes square to `square`, each running from its apex to the
foot of M on it, `gear.M level(ax) foot`), each side a bound (`foot inside P`, `foot outside P`
in the pinion's cone), the offset's hand and the apex behind M bounds too; only P is drawn
in, and N for the tip relief's kinks (beyond C as their pitch points: `distance(>= 0mm, along:
to_inner)`). `tests/hypoid_layout.rs` is the gate (the recorded pair to 1e-9 from the words alone,
from rough starts and from the wrong side of every bound). The mesh export refuses a sheet a
placement carries inside the blank; CLI test readers zero the offset.

- **A solid is a term, and a verb is a noun that has not been given a name** (Solvent §6.9,
  `solid.rs`; issue #48).  A solid is its **stock, plus everything in `union` with it, minus
  everything that `cut`s it**, over primitives that are faces swept — no stateful feature tree.
  Both groups are *sets*, filled anywhere in any order (P2).  A design needing the other order
  (a pocket with a boss in it) **names the intermediate**.
  **A swept solid takes features too** (§6.9, [0.31]): the first `union`/`cut`/`common` naming one
  makes it the body over its own sweep (`program/solids.rs`'s body pass) — the name keeps its
  index, and the sweep moves to a stock of the same name (`SourceMap::also_made`), which
  `operand_paths` gives no step, so `plate.near` stays its name.
  **Nothing three-dimensional is solved for past the sketch** (axes and planes are, with the
  sketch).  `EntKind::Face` and `EntKind::Solid` own no `Param`, and every extent is an `Extent`:
  the text written and the number the *flattener* settled.  The strata run one way: the sketch
  solves, depths are worked out, terms are ordered (`solid::resolve`), outputs are read.  No
  `SpecKind` takes a face or a solid, so a 2D constraint cannot name one.  Built after every other
  kind including `Curve` (`program::solids`).
  **The kernel is the term and nothing is built or stored**: a view, section, volume, mesh and
  clearance are questions asked of `Csg` by classification (Requicha & Voelcker), memoised against
  `solid::reads` — every scalar of every edge the term reaches, each plane's pose and basis,
  every extent, and `unit`.  Every memo lives in `Sketch::caches` (`model::Caches`, one
  `clear()`); a clone keeps every entry, sharing the `Rc`/`Arc` results, and then remembers apart
  — what a copy read slightly changed (`preview_body`, a body's stratum) reuses for its operands.
  **The classifier must read the facets the candidates are cut from** (against the true circle a
  bore's wall vanishes).  **A BSP prunes what a split loop cannot** (`csg.rs`): splitting every
  facet by every plane goes as the square of the facets.  **No coordinate snap grid**; `EPS`, four
  orders coarser than the solve's noise, and `same_plane` (near-coincident planes read as one; a
  facet is never split by its own plane) keep the classifier out of the gap.
  **A face is a loop walked in order**: an arc entered by its `end` is walked backwards over the
  same stretch of circle, never forwards over the rest (*how far* is the arc's, *which way* the
  walk's).
  `tests/solid.rs` is the gate and checks against **arithmetic, not a second kernel**: a block is
  `w·h·d`, a bore takes exactly its faceted polygon, a flush bore and one drilled past are one
  solid, a boss's shared face counts once, a revolution is Pappus.  `tests/solid_lang.rs` holds
  the language, `tests/derived.rs` the pictures.
- **The sheet is a report** (Solvent §6.12, `hidden::generated`; issue #48).
  `dimensions(body) in views.right` asks for the callouts that *follow from the object*: overall
  extents in that view and the diameter of every round feature seen square on.  They go through
  `callout::layout`'s pen and lanes, and their ids start at `callout::GENERATED`, past any
  constraint: a reading of the *solved* drawing, not a statement in it.
  **The boundary is the feature.**  Which datum, which fit, what is reference or controlling is
  design, which the sheet states.  `tests/sheet.rs` asserts the three it makes and no fourth.
- **A claim about a solid is judged, and can never act** (Solvent §9.8, `clear.rs`; issue #48).
  `disc clear(2mm) cyl`, `head fits(0.15mm) trap`, `piston inside bore` compile **no row** —
  `tests/solid_claim.rs` asserts parameter count, equation count and DOF unchanged.  The words
  are in `constraints::OPERATORS` and settle to no `CKind`; `constraints::solid_word` reads the
  word before the spec, like `gauge_op`.
  **What a reader is owed is the measurement**, not a yes or no, with the *sagitta* beside it: a
  claim decided within the faceting is **undecided**, a third answer.  The implicit min/max
  reading of a term is only a lower bound for a difference, so it culls; the answer is piece
  against piece.
  **`claim over crank.theta in (0deg, 360deg) { … }`** sweeps one of the drawing's **free
  variables** (never a `param`), reporting the *worst* pose; it is **sampling** at `SWEEP_STEPS`
  and says so.  Claims are read **after phase 4** (a free variable is what `expr::evaluate`
  allocates), and the interval **in the unknown's own units**, never converted as an angle.
- **A solid leaves as glTF, and that is the format this kernel's data already is** (`gltf.rs`):
  positions, normals and a named group per face — `mesh::Mesh` — in a header and two chunks, one
  JSON from `json.rs`; no ZIP, no dependency.  STEP is a B-rep, which this kernel is not.
  **Every face of every object is a named node**, the path carried **twice**, as `name` and in
  `extras`, because a loader may sanitise one (three.js strips dots; `extras` becomes `userData`).
  **Metres, because the spec says so normatively**: a `mm` document is scaled out, its unit in
  `asset.extras`; one naming no unit is written as it stands.  Exported are the document's
  *objects* (`overview::objects`) — a bore is a hole in a part, not a part — and glTF holds a
  scene, so it need not be told which part of an assembly to be.  `solventc --gltf`,
  `File ▸ Export solid (glTF)` / `(STL)`.
- **A mesh is welded, and grouped by face** (`mesh.rs`).
  **Welding** is a T-junction fix and not a vertex merge: `weld` puts the vertices lying on an
  edge's interior into it.  A hash **cell** is sized to the object (`scale / 128`) while the
  tolerance stays `1e-9`; never conflate them.  `triangles` fans **from a corner where a piece has
  only corners, and from the centroid where it does not** (a vertex fan's zero-area sub-edges,
  dropped, re-open the T-junctions).  The test is per piece: has this polygon a vertex that is
  not a corner?
  **Grouping** is `mesh::grouped`: triangles in face-path order, a normal per vertex — the
  facet's own where flat, averaged where the face is `smooth`.  Across the ABI as **buffers plus a
  small table** (`gcs_solid_mesh`, `gcs_solid_normals`, `gcs_solid_faces_json`), all reading one
  memoised `Want::Mesh`.  `Document.solids()` names them (a solid has no proxy).  `tests/mesh.rs`
  is the gate and asserts the *before* as well as the after.
- **A mesh is cut to the object and a volume to the report** (`solid::mesh_unit`, `MESH_SAGITTA`).
  `REPORT_UNIT` makes a *volume* good to 1e-4; a mesh is cut to a sagitta that fraction of the
  *solid's own diagonal* (scale-free), asked of the primitives' boxes.  `gcs_solid_mesh_unit`
  publishes it, and **a unit at or below zero *is* that choice**, resolved at the one seam
  (`Sketch::cut_unit`, read by `solid_boundary`, `solid_edges`, `solid_mesh`) — never per entry
  point (a sagitta of zero hangs the tab).  `tests/mesh.rs` asks it of all three.
  **The glass box asks with 0, and that is why zooming is free**: `unit` (a screen pixel) is
  wrong for a scene with its own camera.  `overview::scene3d` asks the same way (`SCENE_PX` for
  drawn polylines, `0` for the object's edges), so `Box3D`'s rebuild key omits the zoom.  **It
  includes the pose** (a fingerprint of the parameters): a drag moves numbers inside one sketch,
  so the datums (`overview::Layer::Datums`: panes, their axes, axes in space) are rebuilt every
  frame the pose moves, and the objects (`Layer::Objects`, creases and meshes) once it settles —
  no gesture or wobble in flight.
- **A face is one loop and a solid's faces are named by path** (§6.8).  A face is a closed loop of
  edges on the one plane every point agrees about — *read* off the memberships, never written on
  the face.  There are **no holes**: a hole is a solid that `cut`s the body.  An `in` block leaves
  a face and a solid alone, as it does a datum — they bear no points, and refusing them would
  split the design and the solid it is a section of across blocks.
  A boolean **never renames**: `block.near`, `block.far`, `block.side_l` (a prism), `bore.axis`,
  `bore.start` (a revolution), and through operands — `body.bore.wall`, `part.base.pocket.floor`.
  A name whose face a boolean ate is a *fact the report carries*, never an error.
  E080 a face that is not a loop on one plane, E081 a revolution's axis, E082 a face a body no
  longer has, E084 a section cut across its own view.

- **A face family is a face that runs** (§6.9 [0.93], `solid/family.rs`; #243 L1).
  `solid(I(…).f over s in (a, b))` parses to `Sweep::Family(FamilySpec)` (`parser::family`, by
  an `over` inside `solid(`'s own brackets, `family_ahead`); the flattener walks the instance
  **drawn**, under a key the source cannot write (`#f<offset>.`, so its declarations are
  anonymous), the running formal left unbound so `bind` makes it the free value `#f12.s` — and
  **held** (`flatten::Held`): a text affine in it carries it as any unknown's, its coefficient
  written with the dimension the two differ by (`of_vals_in`: `(-12mm * #f12.s + 24mm)`), a text
  that is not is kept and written in where it is read, as a trace keeps one; a pin `t == s` ties
  to it, a seed reads it at the middle.  The elaborator holds each (`Sketch::hold`): a **fixed**
  parameter, which `expr::evaluate` reads as a number (no free variable, any arithmetic), which a
  contact pinned to it owns (`Sketch::add`), and which `owned_arg`/`graft` carry by name.
  `SolidDef::Family { face, station, held, from, to }`: `station` is every entity the instance
  made (by key prefix, the face's edges and their children) — `Sketch::stations`, which
  `drawn()`, the panes, the pane picks and `callout::layout` leave out.  **The drawn station is
  solved with the drawing at the middle**, so the document's DOF and diagnosis are its too.
  **Evaluation re-solves it** (`family::built`, memoised in `Caches::family` against
  `MemoKey::at`, which reads every parameter and every constraint's number): the station's part
  (`io::Part::around_all` from the face, every primitive, plane pose, lift and outer free
  variable not the station's held fixed — a plane's origin is no wall while its plane moves, and
  a plane held along drawn lines keeps its intrinsic rows across `graft`), then a run from the
  middle to each end: the running number moved, `expr::evaluate`, a solve started where the
  last two stations carry it, and **a station must be regular** (`System::rank` = `n_free`).
  Steps are as long as a cubic through four solved stations stays within the sheet's flatness
  (the extrapolation's miss × 9/384); the facets between are filled in by that cubic
  (`densify`): as the chord rule asks of every sample's run (owed, to `MAX_STATIONS`), and as a
  wall's twist asks of its quads within `FACETED` samples × stations.  **An end that will not
  solve is read as its limit**: the stations close in to `LIMIT_GAP` of it and the samples are
  carried there quadratically (`limit: true`, never solved); an interior station that will not
  solve is halved to `FINEST` and refused there.  Each loop is walked once (`walk_face`, the
  loop's rule), each edge sampled at fixed fractions of its own run at every station (a line its
  start, an arc by angle, a circle from its plane's `u`, a spline by parameter), the counts the
  finest station's.  **Judged** at every solved station: each loop valid on its plane and wound
  as at the middle, holes inside and apart, no edge lost (its run under the flatness) or turned
  over between stations; an end may lose an edge or the whole section (its samples run together
  within the flatness).  **And ahead**: every sample of each station on the far side of the one
  before's plane, the way the family moves, else the swept volume crosses itself.  Faceted
  (`Family::primitive`): caps by `loft::cap`'s strips, none at a vanished end; walls the quads
  between stations, flat as one facet, gently bent as two, twisted past the flatness **fanned
  from the middle** (four triangles, which hold a ruled patch's volume where a diagonal errs to
  one side); a wall is `smooth` where its facets do not lie in one plane; the caps' strips welded
  into the end rows only.  Walls are the face's edge names under the path of the face's scope in
  the instance (`body.sec.ab`, `body.sec.inner.loop.ab`), the caps `start` and `end`.  A station's
  dimension may read a number of the geometry it was given (`rim.r`, `a.x`): the flattener
  records it (`Expansion::reads`) where it would refuse one outside a trace, the elaborator holds
  it reading that scalar (`HeldNumber::reads`, refreshed by `expr::evaluate`), and the part
  brings the read geometry along; one of the station's own is E103.  No exact construction, no
  material field and no OCCT recipe yet (each refuses naming the family).  `tests/face_family.rs`
  is the gate: the L0 programs (`docs/lofts/l0/`) against their closed forms.
- **A face closes itself** (§6.8, `program::build_face`; issue #49).  The brackets hold a *walk*,
  and an item may be a **point**, with `-> close` sealing the run back to the first item.  Where
  two neighbours do not meet, `build_face` mints the straight line between them, class
  `.closure` (hidden by the base sheet): `pist_f := face(crown, pL0, pL1, pL2, pL3, pL4, s0.p,
  -> close)`, `hole_f := face(x0.p, x1.p, x2.p, x3.p, -> close)`.  A minted run names a face
  of what is swept (`close0`, `close1`, … in mint order, skipping existing edge names).
  **The shorthand may not swallow a mistake**: an interior gap between two *edges* is still E080
  (`face(ab, cd, bc, da)` must not become a bowtie); **an edge takes its direction from a
  neighbour it actually meets**, so one between two gaps is refused (`bad := face(a, bc, d, ->
  close)`; name the corner: `bore_f := face(m0.p, b_br.p, bore_r, hx.p, -> close)`); and the wrap
  is minted only under `-> close`, so "the loop closes" stays something the source states (on a
  loop that already meets it mints nothing).  `tests/solid_lang.rs` is the gate.

- **A part carries no views; a sheet asks for them** (§6.11, `hidden.rs`).  `view(body) in
  views.right` and `section(body, at: swing) in views.front` are *outputs*: no `Int` draw flag, no
  `when draw_side == 1 { … }`, no second copy of the geometry and no `project` to keep in step.
  Three draughtsman's rules.  **A corner is drawn and a tessellation seam is not** — a `smooth`
  seam is drawn only where it is a *silhouette*.  **What the material covers is dashed, not
  dropped** — the eye's ray is classified against the term and the piece carries `.hidden`.
  **Coincident page lines are drawn once, visible winning** — an *interval* rule, not a segment
  one (a rim seen edge-on folds onto itself, splitting at different places): every stroke is laid
  on its line, the visible stretches unioned, the hidden what is left over.
  Everything comes back in **the plane's own coordinates** (`PageFrame` over `Basis::view_coords`),
  so a derived view sits where a hand-drawn one tied by `project` would — `tests/derived.rs`'s
  strongest gate.  **The core projects and the front end strokes**:
  `hidden::layout` resolves the ink through the sheet, `svg::render` and `paint.ts` stroke what
  they are handed, and neither owns 3D arithmetic or a rule about hidden lines.
  Derived views are the `.svd` paper's; the model workspace shows a solid as its mesh.
  **Every part of the V-twin is written this way** — `vtwin/components/cylinder.sv`, `piston.sv`,
  `disc.sv`, `flywheel.sv`, `throttle.sv` and the plate in `frame.sv`: one section and its solid.
  **Where a part's turned features are is where its section has to be**: a turn about a line in
  the section puts what it makes *on* that plane (the disc's and plate's sections are mid-planes).
  A **hex pocket about a radial line** (`parts.Grub`) is neither sweep nor turn, so it stays four
  hidden lines; a feature drawn as a *centreline* (the exhaust vents) is told its width, `wch`.
