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
members roll at `N_c / N`, written as tooth counts (`crown_teeth / pinion_teeth`), not `1 / sin`.
**The face mill is found, not stated** (#283): `pressure_shift` and `cutter_radius` are formals of
`HypoidPair` the call leaves to the solve, seeded (`hint(10deg)`, `hint(80mm)`), and `Balanced`
finds them as a designer would — the generated flanks' undercut margins (each flank's radius of
curvature in a section square to the trace, a module inside and outside the pitch, `Margins`)
equal at the mean point, which sets the shift, and at the toe, which sets the cutter; a cutter no
smaller than R (`gear.M distance(>= R) trace.C`), the root nearest the seeds (11.2527°, 75.6122
mm; seeds far off reach another, 260 mm from 14°, 60 mm).  Each blade (`Blade`) is
`std.ConeThrough` with the side its cone opens to said, so its half reads the unknown shift as a
number times it plus a number.  At the found cutter the pinion's swept cut is built and agrees
with its field, but its sheet is fitted as one face, not named by the cutter's (a follow-up);
`brep_export.rs` holds the sectors at the recorded mill.  `fixtures::gear::configure`
given a `pressure_shift` states it with the recorded cutter, 0.8 R, and drops the balance
(`recorded_machine`): every recorded number was taken of a pair so cut.
`tests/hypoid_machine.rs` is the gate. **No axial views** (#215): the
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
  **An empty body** (§6.9 [0.99], #313): `solid()` is built its own stock (`build::body`), and the
  body pass, having resolved every statement first, gives it the first `union` naming it in that
  place before any is joined; with none it is E080 at its declaration, and no statement joins it
  (nor does `made_of_itself` call it E041).  A solid swept in a `ring` is one solid a copy
  (`peg[3]`), so a pattern is `repeat n as i { peg[i] union pegs }`.  `tests/solid_lang.rs`.
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
  **A boolean takes its operands whole and gives back what it did not cut whole** (`csg.rs`,
  #313).  Each operand's polygons are clipped against the *other's* tree, never handed back as
  their own tree cut them (a ring's inner wall's planes cross its caps: 3,600 facets came back as
  140,000 pieces and fed every later boolean); a polygon whose pieces still cover its area is put
  back whole (`rejoin`); two operands whose boxes lie apart are concatenated.  A run of unions is
  one n-ary union (`union_all`: the operands lying apart grouped first, then the two smallest), and
  `s − a − b − …` is `s` less their union, so a pattern on a large solid builds its tree once.
  `tests/solid.rs`'s `a_pattern_on_two_rings_is_put_together_once` holds the volume to arithmetic
  and the pieces to fewer than twice the facets.
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
  **A swept claim's names resolve as a lone claim's do** (`flatten/resolve.rs`'s `rewrite`): the
  motion it runs along and the solids its body judges are found through instances
  (`frame.body`) and, inside a component, under the instance's prefix; a free variable is left as
  written for `sweep_of_claim` to find.  `tests/solid_claim.rs`'s
  `a_claim_over_a_motion_reads_the_solids_of_instances`; `hypoid_drive/drive.sv` is the case.
  **Claims are judged a pose at a time, and a page's are judged off its thread**
  (`diagnose::ClaimJudge`): `step` judges the next pose (a claim over a sweep is `SWEEP_STEPS + 1`,
  one at a pose is one) and `progress` says which claim of how many and which pose in the core's
  words (`ClaimProgress::doing`); `judge_solids` is the same judge run to the end, each claim in
  `progress::during`, so the CLI and a page cannot disagree.  A claim reads the solved pose and
  enters nothing above it, so `DiagnoseOptions::solid_claims: false` leaves `solid_claims` empty
  and every other word of the diagnosis as it was.  The page's diagnosis does so (`solidClaims:
  false`) and `app/claim-judging.ts` runs a `claims` job in its own instance of the mesh worker —
  the drawing's text, its pose and the claims as the page's core names them
  (`gcs_sketch_solid_claims_json`; a worker that names others refuses) — a newer drawing or pose
  ending it at its next pose.  The footer says where it stands (a spinner, the core's words, a bar,
  a clock); the verdict washes the claim's statement as a claim's does (`.claim-held`,
  `.claim-refuted`, `.claim-undecided`; `Document.spanOfStatement`).  Gates:
  `a_judge_stepped_a_pose_at_a_time_comes_to_the_verdicts`,
  `a_diagnosis_asked_not_to_judge_solids_says_the_rest_unchanged`, `claim-judging.test.ts`.
- **A solid leaves as glTF, and that is the format this kernel's data already is** (`gltf.rs`):
  positions, normals and a named group per face — `mesh::Mesh` — in a header and two chunks, one
  JSON from `json.rs`; no ZIP, no dependency.  STEP is a B-rep, which this kernel is not.
  **Every face of every object is a named node**, the path carried **twice**, as `name` and in
  `extras`, because a loader may sanitise one (three.js strips dots; `extras` becomes `userData`).
  **Metres, because the spec says so normatively**: a document's lengths are scaled out, its
  unit in `asset.extras` — millimetres where it names none.  Exported are the document's
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

- **A face family is a face that runs** (§6.9 [0.97], `solid/family.rs`; #243 L1).
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
  (`Family::primitive`): caps by `profile::cap`'s strips, none at a vanished end; walls the quads
  between stations, flat as one facet, gently bent as two, twisted past the flatness **fanned
  from the middle** (four triangles, which hold a ruled patch's volume where a diagonal errs to
  one side); a wall is `smooth` where its facets do not lie in one plane; the caps' strips welded
  into the end rows only.  Walls are the face's edge names under the path of the face's scope in
  the instance (`body.sec.ab`, `body.sec.inner.loop.ab`), the caps `start` and `end`.  A station's
  dimension may read a number of the geometry it was given (`rim.r`, `a.x`): the flattener
  records it (`Expansion::reads`) where it would refuse one outside a trace, the elaborator holds
  it reading that scalar (`HeldNumber::reads`, refreshed by `expr::evaluate`), and the part
  brings the read geometry along; one of the station's own is E103.  **Its walls are exact** (#243
  L2, `family::walls`, `brep::family`): once the faceted family is judged, the station is run from
  the middle to each end in **pieces**, each solved at its six Gauss–Lobatto points (warm from the
  last two) and **checked against a station solved between its last two** — the polynomials
  through the six must pass every edge of it within `EXACT` (1e-10) of the section's size, else
  the piece is cut (as the miss's sixth root says) — an end that will not solve read as its
  limit, the last piece's nodes stopping `LIMIT_GAP` short.  Each edge is **numbers** (a line its
  ends; an arc its centre, its radius vectors to its start and a quarter turn on, and its signed
  sweep; a spline its poles and parameter span), and the recipe's `family` node carries them at
  every node (points about `Writer`'s origin, directions scaled), so the solid is a recipe node
  like any other: Booleans, meshing, STEP and the evaluated solid read it (`exact_solid`, rung 3).
  A traced curve in the section has no exact wall yet (refused there; the facets answer).  The
  native OCCT host refuses the node (its lofts are the oracle at L10).  `tests/face_family.rs` is
  the gate: the L0 programs (`docs/lofts/l0/`) against their closed forms, every one to 1e-9 off
  the B-rep (`props::volume`, `Brep::check`), a slab cut by the twisted duct, STEP read back.
- **A loft is library words over the family** (§6.9 [0.100], `rust/lib/std.sv`; #243 L3).  The
  station words say only where a station stands: `std.OnLine` (a fraction of a line's run),
  `std.OnArc` (of an arc's turn), `std.OnSpline` (a spline's parameter), `std.On` (any `path`: a
  line, an arc, a spline, a traced curve — L4, a line touching a round at a point of it drawn in
  its plane, `planes::incidences`), `std.Helix` and `std.OnHelix` (L6, below) — each a component
  holding a plane `n` square to its path, its `v` along `up` (`On`'s at its home: [0.120], its
  frame integrated, below).  `On` and `OnSpline` seed their tangent ahead
  along the guide (`hint(at: m, along: g)`, `entities::rate_along`) and their plane off it as
  `OnLine` does, so the drawn station starts on the branch whose `v` is `up`'s and whose normal
  runs the way the guide does — never the mirror, which a sign-free `parallel` would otherwise
  leave to wherever the solve started (`tests/paths.rs`, `the_frame_keeps_up_up_on_every_guide`).  The section is the writer's component drawn `in st.n`, reading `s`; two
  sections correspond by being one.  **No Rust reads a word's name**, and there is no loft
  constructor: `SolidDef::Loft`, `Sweep::Along` and `solid/loft.rs` are gone, and `along:` in a
  solid's brackets is refused at the key saying what to write (`parser::declarations::sweep_arg`).
  `brep::build::ruled` joins a section only to its own image (a drafted prism's offset, a canal's
  screwed disc), edge for edge; the recipe has no `loft` node.  The two-face word (pairing two
  sketches drawn apart) returns at L8 (finding 8).  `tests/loft.rs` is the gate: `solid_loft`,
  `solid_elbow`, `solid_bend` and `solid_twist` at their closed forms off the B-rep to 1e-9, their
  walls by name and kind, `std.On` agreeing with each kind's word.
- **A guide in space** (#363 S1, [0.119]): a curve traced in space is a `path`, so a station
  stands on it by `m coincident(t == s) g`, `g tangent(at: m) tl` and a plane square to `tl`
  turned by `Minimal`'s rate from a home square to `up` — the frame integrated, as along
  `std.OnHelix` — which is `std.On` itself since [0.120] (S2: a frame holding `v` along `up` at
  every station leaned its plane off square to a twisted tangent, silently).  `On` over a planar
  guide is `OnSpline`'s algebraic frame, corner for corner (`tests/rates.rs`).
  `tests/space_paths.rs` is the gate: a 2 × 2 square along two turns of a traced helix (r 10, rise 10) at 4·√((40π)² + 10²)
  off the exact B-rep to 1e-9 and corner for corner `std.OnHelix`'s under `Minimal`; the frame
  turning −28.5576° a turn against Frenet's (r 10, pitch 5), to 1e-9; a traced surface refused;
  `std.On` on the traced helix `OnHelix`'s solid corner for corner; an off-centre square round
  `pipe.sv`'s U at 4·(60 + 23π), its frame carried across the joints.
- **Frenet's frame** (#363 S5, [0.122]): `std.Frenet(g, s:)` holds its plane square to the
  guide, `u` along `line(m, c)` to the centre of curvature (`g curvature(at: m) c`, `curves.md`)
  and `v` the binormal (seeded `tl` turned −90° about it, the one sign a seed picks, and kept by
  warm starts), an algebraic station — no rate.  Where the curve has no curvature the centre's
  rows lose their rank and the station is refused there ("not held"), never followed.
  `tests/space_paths.rs`: along `ribbon.sv`'s helix as a traced curve it is `std.OnHelix`'s
  `facing` solid corner for corner; a curve straight for a third of its run is refused at a
  station; a surface's centre is E040.
- **A chain is one path** (§6.9 [0.102], #243 L4, finding 6).  A `path` formal of a family's
  instance given a named open chain (`flatten::chain_argument`, the chain found among the
  statements walked so far) makes the family **legs**, one per link its interval reaches
  (`walk_legs`): the instance walked again under `#f12_k` with the formal given link `k`, its own
  running number held at the middle of its part of the interval, the formal read as `k` past it
  (`bind`: the free value's constant), so a law reads the chain's `s` everywhere; a contact pinned
  to `s` on the link itself is pinned to the leg's own number (`settle_relation`'s `leg_named`,
  the operand matched through the alias table), and one on anything else past the first link is
  E103.  The running formal must be a `Scalar`, the interval within `(0, n)`, the chain open
  (E080).  The elaborator gives `SolidDef::Family` its `legs` (`FamilyLeg`: face, running number,
  link index, and the station's entities in the order of their names under the leg, so legs
  correspond entry for entry); `face`/`held` are the first leg's and `station` every leg's.
  `SolidDef::legs` is the one reading: a family over one path is one leg.  `family::march` steps
  each leg over its own part (`leg_part`) — the first from its drawn station, **each later one
  from its first station, its part started where the last leg ended** (`carry_over`: points put
  at the same place in space, planes in the same pose, radii the same), so the drawing's branch
  on the first link is carried along the chain — settles the samples across every leg, reads
  only the family's own ends as limits (a joint that will not solve is refused), shifts each
  leg's stations by its index, judges the whole run and densifies leg by leg; the joint station,
  solved on both legs, is kept once.  `family::walls` runs each leg's pieces the same way and
  joins them, so a wall is one sheet, one name, over every link.  `tests/paths.rs` is the gate:
  `docs/lofts/l0/pipe.sv` (a U of three lines and two quarter turns) at the sum of its pieces'
  Pappus to 1e-9, a radius law in `s` over the chain, part of a chain, the refusals.
  **A chain across planes** (#363 S4, [0.121]): a straight between bends in two planes touches
  each by `TangentCircleAt3` (`space.md`), so the bent pipe is one chain.  **A station that
  states a rate on a link past the first is carried** (`program::solids::carry_legs`): its rows
  `Constraint::carried` (no row of the drawing: `states_rows`) and its own parameters — entities',
  points' lifts, planes' normals, contacts' places — held where seeded (`Sketch::carried`), since
  only the last leg's flow knows its frame (a guide along `up` there left the drawn station's
  turn a freedom); `part_of` gives both back to the leg's part.  `tests/space_paths.rs`: the pipe
  of #363 (line, R20 in `std.top`, line, R20 in x = 20, line) a 4 mm tube at 16π(60 + 20π), a
  2 × 2 square at 4(60 + 20π), a bar off its centre line at its centroid's path, and the end
  cap's frame (tangent +z, `u` −x, `v` −y) to 1e-9.  **A chain drawn in space turns either way at
  an arc** ([0.124], S6): an arc over points in space faces its minted plane as its points do, so
  L0's S-bend — a left turn then a right — is one chain (`in_space_a_chain_turns_either_way_at_an_arc`,
  Pappus, its second bend's plane facing −z); drawn in a plane's block a chain still turns left at
  every arc (L4: an arc there runs counter-clockwise from its start, and reading one walked end to
  start would need a leg's place pinned to `1 − s`, which no pin says yet).
- **A law is an expression in `s`** (§6.9 [0.103], #243 L5, finding 10).  A twist or a scale is a
  dimension of the section reading the running number; no word takes a `law:` (no function values).
  `std.sv`'s `Twisted` (the point `reach` along the station's `u` turned `twist · s` — [0.118],
  the second point a section drawn once is carried to, `f := prof mapped(o -> n.origin, x ->
  tw.t) in n`; through 0.117 a plane of axes a section was drawn on corner by corner), `scaled` (a point at `(du, dv)` scaled `1 + (scale − 1) · s` about the
  plane's origin) and `scaled_radius` take a total and run it linearly; any other law is the word
  given another fraction (`s: 3 * s^2 - 2 * s^3`) or the writer's own expression.  **A number given
  over `s` that is not affine in it is kept as text** (`Walk::held_text`), whichever formal it is
  given to: a component's (`keep_text`, into `held.texts`; `bind` then makes it no unknown, and
  `settle_text` reads the kept text before the closed scope's reach) or a relation word's
  (`apply_word`, written into the body's dimensions); the running number written in is no geometry's
  read (`note_dim_reads`).  Through 0.102 the first became an unknown of the drawing (the station
  "not held: 1 degree of freedom") and the second E103.  A `when` reading the running number is E040
  in its own words (`values::Names::running`, the walk's `held.names`): a `when` is made once, the
  station one shape over the run, and `min`/`max`/`abs` write a law in pieces.  **A station's own
  parameters are never held** (`family::part_of`): a primitive outside the station — a closing edge
  another face of the instance minted, named by nothing under it — holds only what is not the
  station's, else sweeping the instance's other face froze its corners where the drawing stood and
  no station past it solved.  `tests/laws.rs` is the gate: twisted extrusions at A·L (0°, 90°,
  −450°, eased), a twisted square round a bend at Pappus, square and round frusta (twisted too), a
  sine swell (closed form) and an eased scale (a quadrature exact for its degree), each given its
  word as a fraction, a law in pieces (`max`), the `when` refusal and two faces of one station, each
  off the exact B-rep to 1e-9.
- **A screw is constraints** (§6.9 [0.110], #243 L6).  `std.Helix(base, pitch:, turns:, s:)` is
  `base`'s frame turned `360deg · turns · s` about `base.normal` and risen `pitch · turns · s` along
  it: `n` the base so carried (a section in it sweeps a twisted prism), `axial` the plane through
  the axis, `u` out along `n.u`, `v` up the axis (a thread's).  `std.OnHelix(base, r:, …)`'s `n`
  stands square to the helix of radius `r`, `v` up the axis leaned by the lead angle (a coil's
  wire).  Both are points placed by ordinates in `base`'s frame (`coords`, and `distance(…, along:
  base.normal)` for the rise) and planes over the lines between them, so **no direction is held
  by a seed** — a `parallel` would leave its sign to wherever the solve started — and a negative
  pitch winds left-handed, the right-handed solid mirrored in the base plane.  The axis is
  `base`'s normal through its origin, never an axis given apart (`Screwed`'s, through 0.109,
  could disagree with `base`).  No motion is involved (#64's motions stay for swept solids).
  `tests/helix.rs` is the gate: square and V threads at A · 2πr̄ · turns (Pappus, r̄ the
  centroid's), flights square to the axis at A × the rise (Cavalieri), a spring at A × the helix's
  length, left-handed mirrors, each off the exact B-rep to 1e-9, and the fast path (`brep.md`).
- **Several sections are drawn on rails** (§6.9 [0.111], #243 L7).  A corner's rail is its
  places in order, one a section; the station's corner stands where its rail meets the station's
  plane.  **Chains run in step**: `flatten::chain_arguments` finds every `path` formal given a
  named open chain, `walk_legs` gives each its own link `k` on leg `k` (refusing chains of unequal
  counts, E080 naming both), and `Held::legs` keeps every link a leg runs, so a contact pinned to
  `s` on any of them reads the leg's own number (`leg_named`).  A guide cut at the sections and a
  chain of lines through each corner is the **ruled** loft, piecewise in `s`; its walls meet at an
  angle at each section, which the exact build makes an edge (`brep.md`, creases).  The **smooth**
  loft's rail is `spline(through: …)` (`curves.md`), one span a link, so with the guide cut the
  same way its sections still stand at `s = k`.  **No guide**: `std.Across(a, b, c, s:)` is the
  plane over the lines from the place at `s` on rail `a` to those on `b` and `c` (Helix's way, a
  plane over two lines sharing an end).  A rail drawn in a plane is reached by a corner drawn in the
  station's plane and that one (`point in st.n, front`): a point another plane already draws is not
  drawn in a spline's plane by its contact (#249's rule is for a point in space).
  `tests/sections.rs` is the gate: the spline through a chain's corners against control points
  solved apart from its end conditions on the curve's derivatives (tangent and natural), a ruled bottle at the sum of its frusta with an edge at
  the shoulder (two cones, one name), a loft with no guide at its prismatoids and, on three splines, by quadrature, a smooth bottle by
  quadrature on every span, its wall square to the base cap and tangent to a turned neck's cone,
  and the refusals; `face_family`'s bottle is `docs/lofts/l0/bottle.sv` over the spline through
  its rims.
- **Two faces drawn apart, and a section held to its rails, are library words** (#243 L8,
  [0.113]): no Rust names them.  `std.Ruled(a, b, s:)` walks the two faces in step (`cycle (ea,
  eb) in (a, b)`, `language.md`), joins corner `k` of each by a straight rail, stands each
  corner `point in st.n` on its rail with `st := Across(rail[0], rail[1], rail[2], s:)`, and is
  `face(side)` of its copies, so its walls are `body.side[k]` from corner `k` and its ends the
  faces themselves; `std.Apex(a, tip, s:)` the same to a point (no `end` cap).  Correspondence is
  the written walks: a connector is where a walk starts, unequal counts a point named on an edge;
  the first three corners must not stand in a line (`Across`).  `std.Similar` and `std.Affine`
  are each one statement, the face carried whole ([0.118], below): `f := prof mapped(from -> a,
  to -> b, third -> c) in n`, the profile's corners and frame columns of each corner's `mapped`,
  held where the profile is (`Framed` and `Spans`, two `offset`s over hidden spans of the frame,
  are retired, [0.117]); `lofts/blade.sv` and `three_rails.sv` write the word itself.  `tests/correspondence.rs` is the gate: `two_faces.sv` at
  40/6 · 1856, a connector and a split triangle at their prismatoids, a pyramid to an oblique tip,
  a blade on two rails (1560; turning, 1650), a square on three rails at its area ratio's
  prismatoid, a face walked, a face of a cycle's copies, and the refusals.  An arc edge lofted
  with a line edge is not said: it wants a rational edge whose weight reads the arc's sweep and a
  word choosing by an edge's kind.
- **A station that states a rate is integrated from its home** (§6.9 [0.116], #339 D3,
  `solid/rated.rs`).  `rate(x over s) perpendicular y` (or `parallel`) is no constraint: the
  elaborator keeps it in `Sketch::rates` (`model::RateRow`: whose rate — an axis's direction or a
  point in space — the word, what it is held against — an axis, a line or a plane — and the held
  running number its formal is given), grafted with the entities it names, so a station's part
  carries it.  `initially { … }` rows are ordinary constraints flagged `Constraint::initially`
  (graft and JSON carry it): the drawing solves them with the drawn station, the **home**.  A leg
  whose part states a rate over its running number (`rated::states_rates`) is solved at the home as
  any (regular there, with its `initially` rows), then **marched** (`rated::Flow::march`): the part
  cloned, its `initially` rows made claims, its running number unfixed and made the column every
  number written over it leans on — each such dimension's text a tape over the held numbers
  (`tape::derivatives_along`, exact), its constraint made its free twin at `m·s + c` with `m` the
  derivative (`Levers::lean`), so the running number's column of the conditioned Jacobian is `g_s`;
  a number over it that is no dimension, or a second on one statement, is refused.  The rate rows
  are linear rows over the state (`Rated::rate_rows`: an axis's rate is its unit direction's,
  `(I − d̂d̂ᵀ)d′/|d|`; square to a direction one row, along it two, a plane the other way round).
  `[G; R]·z′ = [−g_s; 0]`, rows of unit length: solved square over the rows found independent at
  the first evaluation (LU), and where that fails by the minimum-norm least squares, whose rank
  short of the state's width is the **index check** — refused there, "is short of rates: k
  degrees of freedom along its running number" (at the home, before any march), and a stack that
  is inconsistent is refused too.  `ode::march` integrates from the home to each end at
  `ode::TOL`, `rated::COLUMNS` (4) columns; `Reading::settle` reads a station off the flow
  (`Flow::put`, `ode::Path::at`: one more step from the node before) where it would solve one, so
  the faceted stepping, its judging and the exact walls (L2's pieces, each checked against one
  more integration between its nodes) are unchanged; the flow is marched once and shared with the
  walls (`Family::flows`, one a leg).  No regularity is asked of an integrated station: its rank is
  the flow's.  **A station read off its flow meets its rows** (`Rated::meet`, #363 S2):
  minimum-norm Newton steps from the integrated state, the running number held, to `MET` in each
  row's units — what the rows determine is theirs, only the freedom the rates hold the flow's, so a
  law with a kink (`max(1 − s, 0.7)`) is not smeared by the integration across it.  A contact on a
  spline that runs onto another span compiles the flow's system again (`Rated::respan`, as a solve
  does).  **A rate over a chain** (#363 S4): each leg its own flow; a leg after the first has its
  home at its start, the state carried across the joint (`carry_over`), its `initially` rows made
  claims and its rows met from there with no regularity asked (`Reading::first`), and its walls
  read off its own flow.  A station with `initially` rows and no rate is refused.  A
  station's part holds **no plane's normal of its own** (`part_of`): a normal is minted where read
  (`Sketch::plane_normal`), under no name of the station's, and moves with its plane.
  `std.Minimal(n, up:, s:)` (#243 L9) is the word.  `tests/rates.rs` is the gate: the minimal
  frame along a helix of radius 10 and pitch 5 turning −28.558° a turn against `std.OnHelix`'s
  (Frenet's) to 1e-9, read off the caps (and three quarters of a turn, and the home square to the
  axis); a tube on four turns at A·L to 1e-9 in either frame; along a planar spline the word and
  `std.On` the same solid, corner for corner; the refusals.
- **A face carried whole** (#357, [0.118]): `sec := prof mapped(from -> a, to -> b) in n` parses
  to a `Block` (`Block::image`, a `FaceImage`) — a `cycle` over `prof`'s loop under a binder no
  source writes, set aside and resolved as every block over a walk is — which makes no copies:
  `flatten::image::image_face` builds the statements a person would write, walked in the
  definition's scope and spliced where it stood.  Each link's source is an alias `sec#src{k}`
  (the link's own scope, as a copy's `e` is); a corner of a face of corners is `sec.<name> :=
  point in n` held by `mapped`, and the image `face(sec.k0, …, -> close)`, whose minted runs name
  the walls `close0`, … as the source's; an edge's start (`p1`, an arc's `start`) is `sec#q{k}`,
  a line `sec.<name> := line(sec#q{k}, sec#q{k+1})`, an arc about `sec#c{k}` (mapped from its
  centre) and a lone circle `sec.<name>` held by `CKind::MappedCircle` — each in `n`, each image
  point seeded `hint(at:)` its source, apart from the others (an arc's image at radius zero has
  no slope in its radius), and an arc's or a circle's radius left unseeded, read off its points.
  An arc's image is one row over its unknowns, as an arc held at both ends is.  So the walls
  keep the source's edge names (`body.ab`, `body.dome`).  `mirror: true` reverses the loop and
  each arc's ends, so the image winds as the source does; three pairs whose targets turn the
  other way reflect with the loop as it is, every reader taking a line loop either way.  Under
  three pairs an arc or a circle is E080 at the word naming it (an ellipse), as is a spline, a
  curve and a face with holes (`face_walk`).  Printed as written (`print::write_stmt`).
  `tests/mapped.rs` is the gate (a face of corners and of edges with their wall names, an arc's
  and a circle's radii, the mirror's winding, the refusals, the printer), with
  `lofts/tapered.sv` and `lofts/twisted_duct.sv` (each section drawn once, at their closed forms)
  and `hub_motor`'s left cover, the right's profile mirrored in the mid-plane.
- **The loft series** (`rust/examples/lofts/`, its README the reading order): one document per
  capability of #243 L1–L8, each solid at the closed form its comments give, held there off the
  exact B-rep to 1e-9 by `tests/loft_examples.rs`; the app lists the folder as one project
  (`example-catalog.ts`, *Lofts and sweeps*).
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
