# Solids

Part of the [contract](../../CLAUDE.md) the code is held to. Read it when you touch `solid.rs`,
`csg.rs`, `mesh.rs`, `hidden.rs`, `gltf.rs`, `program/refused.rs`, `solid/defined.rs`,
`rust/lib/std/solids.sv`, or `hypoid.sv`. An issue number
(#n) or a spec version ([0.n], see the [language changelog](../solvent-changelog.md)) says where a
rule came from.

**The body rule's third side (0.22):** `tip common body` keeps what the body has in common with
`tip`: a solid is its stock, plus `union`, minus what `cut`s it, within what it is in `common`
with. The word was `bound` through 0.84 (#270: a bound is a dimension choosing a root); `X bound B`
is refused naming `common` (`parser/statements.rs`), and `bound` stays reserved. The Rust names
keep the old word: `SolidDef::Body` carries `bound`; `Term::Inter`; facets, fields and the CAD
recipe (`"bound"`, `BRepAlgoAPI_Common`) all evaluate it. Union first; `cut` and `common` commute;
a swept solid may only be `cut`. `coincident` between two solids is refused.

**The body words are words of `std.solids`, over a primitive** (#783 P3, [0.206]): `B or { p | p in
x }` adds a disjunct to the body `B`'s stock and `B and { p | p not in x }` / `B and { p | p in x
}` a conjunct, said where `B` is not declared; the parser reads one membership of a solid in the
set (`P::joined`, the notation's `inside`/`outside` it was lowered to, `BodyWord::of_joined`) into
the body rule's term, `SolidRel`, and refuses any other set naming the three forms; the lookahead
is `words::joined_at`, which the notation asks too.  `std.solids` defines `x cut B := B and { p |
p not in x }`, `x union B` and `x common B` over it (`rust/lib/std/solids.sv`), so `bore cut body` is a word applied
(`flatten::words`), its statement identity the use's, reaching the same `SolidRel`, and everything
past the parser — `program/solids.rs`'s body pass, the stratum, `as_body`, the empty body, face
paths — is as it was.  The words are no longer reserved (`words::is_name`, `reserved_word`), so a
file may name a thing `cut`; they are imported by name (`use std.solids (cut, union)`), unimported
E102 naming the import.  `std.rounds` says its material of the body by the primitive itself
(`A.body and { p | p not in taken }`), its contributions repeated over pairs and triples of faces,
where a word applied would add an application each; `hardware` imports the words it uses.  The
Boolean gesture (`edit::add_body_word`) writes the word, as the relation it is, and names it in the
file's `use std.solids` (`edit::append::imported`), grown or a line of its own; the parser writes
`SolidRel` only from the primitive, which the printer writes back.  One solid within another is
claimed `claim every q in a: q in b` (`notation::every`, lowered to the claim word `inside` and
marked `Lowered::Every`, judged as before; the printer and the report write it as written,
`print::notation_text`, `describe_solid_claim`); `claim a inside b` is refused naming it, and
`every` outside a claim is refused.
`tests/body_words.rs` is the gate. Spiral bevel (**one file, `spiral_bevel/hypoid.sv`**, #217: no `project`, no `hint` — every root chosen by a
word, the layout solved from them alone; README walks it). **The pair is one component**,
`pair := HypoidPair(std.top, std.y, std.z, …)` over the root's `param`s (std's datums its
arguments, its unknowns `mn`, `rp`, `pa` formals the call leaves out), its steps components below
it — `GearCone`, `ToothTrace`, `PinionCone` (each cone's axis `.ax`; `fixtures::gear::axis`),
`Blank`, `FaceMill` (`mill.ax`, the flank cones, levels and reach), `Tooth`, `Space`, `Member`,
`Topping` — so every name is under `pair.` (`pair.pinion.body`, `pair.gear_cone.M`); a test reads
the recorded rows under their old labels. Each
member's blank is a region (`Blank`: the pitch cone, a `std.sets.Cone` about the member's axis, its
half left unknown so it settles from the pose, grown by the addendum
(`{ q | q distance(<= addendum) pitch }`) and its
outside by the back depth (`distance(<= d, side: inside)`), `std.sets.Sphere`s about the apex the half face width either
side of M (`rim.p2 distance(w / 2, side: outside) toe`), sized by the solved layout — a region term
reads an unknown, `model::TermNumber`; the end relief two `std.rounds.Chamfered` bounds in the same
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
**The face mill's shift is found, not stated** (#283, #904): `pressure_shift` is a formal of
`HypoidPair` the call leaves to the solve, seeded (`hint(10deg)`); the cutter's radius is a stated
input (`param cutter_radius := 75.6122mm`). `Balanced` reads the four flanks the two blades cut on
the two members — each the envelope its blade generates, `pair.balance.pinion_inner`,
`pinion_outer`, `gear_outer`, `gear_inner` — by their least area factors within their blanks
(`regular`, what E3 judges), on the conjugate crowns ungrown and within the blanks before their
end relief, each held from its fold (`regular(by: …) within …`, `>= 0`). It hands them out with
their crowns and blanks as `pair.margins`, and the caller ties the two that bind by holding both
to one unknown, the pair's formal `least` (`regular(pair.least, …)`, a margin reading an unknown,
[0.215]): a hypoid's are the pinion's inner-cut flank and the gear's outer-cut (8.8549°, the least
0.6767 at 24/48, 25 mm, the cutter as stated); with no offset it is the pinion's two flanks
(−1.195°, 0.4231), and the hypoid's tie there leaves the pinion's outer-cut flank folded, a bound
broken (`hypoid_machine::at_no_offset_the_pinions_two_flanks_tie`). Which two bind is the caller's
to say: the maximin of four margins over the shift ties the least of each blade's two, which no
statement can say yet. The cutter is not found: every margin grows with it from R to 4 R at every
design measured, so a balance of them has no root in it. The 12/24, 13/27, 16/32 (1.5 mm) and
10/48 pairs balance (`hypoid_machine::the_pair_retargeted_finds_its_shift`). Each blade (`Blade`) is
`std.sets.ConeThrough` with the side its cone opens to said, so its half reads the unknown shift as
a number times it plus a number.  At the shift found the configured pinion is built by sectors,
243 faces, its cut's sheet 69 × 39 (`hypoid_mill_sheet.rs`, the mill stated as
`fixtures::gear::FOUND_MILL`; `brep_export::the_pinion_as_written_is_built_by_sectors`, solved as
written), and its gear with 291 faces; at 12/24 the pinion builds (111 faces) and the gear's
fitted sheet is refused (0.105 mm and 62.9° from its contacts); the zero-offset pinion of the
pinion's own tie builds (243 faces); `brep_export.rs` holds the sectors at the recorded mill too.
`fixtures::gear::configure`
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
  **An evaluated solid is keyed on its shape, and a solid that only moved is moved** (#412):
  `Caches::solid` keeps each evaluation against a `solid::ShapeKey` — what `reads` reads, but
  every plane's pose rewritten relative to a reference plane (the first face's, found as
  `frame_origin` finds the origin), and that plane's world pose apart.  Two keys of one shape
  (`same_shape`: every number within the solve's step tolerance, `solve::XTOL`, of the larger or
  of the key's reach — the largest world number a pose in it was worked from — #498: a solve
  that settles a part it does not move, a drag's polish, leaves it anywhere a step it would not
  take away, and a nested frame's relative pose is worked through the world; a stand leaves what
  the parameter does not reach to the bit, `follow::Still`) are one solid moved rigidly:
  `Sketch::evaluated_solid` hands back the same `Shape` (`Rc`'d: boundary, mesh, edges,
  classifier) under the placement `solid::Rigid::between` the two reference poses makes, never
  building it again; the same pose within that tolerance is the evaluation itself.  **A solid
  reading anything in the world itself has its pose in its shape** (`Poses::world`): a region's
  points, a face family's stations, a body placed or swept under a motion, a revolution about an
  axis in space or a line in another plane, a face's point in space or another plane, any curve
  — conservative, so such a solid is reused only where it stands where it stood.
  `EvaluatedSolid`'s placement is a rotation and a translation (`to_world = R·p + origin`; `None`
  for a fresh one, bit for bit what it was): its own outputs (`boundary`, `bounds`, `edges`, `mesh`,
  `face_edges`, `round_features`) are the shape's; `local_basis` turns a view's plane in, so the
  renderer and the dimensions need nothing else; the world outputs (`world_*`, `world_bounds` — the
  turned boundary's own tight box — `stl`, a glTF node through `oriented_mesh`, the ABI's
  `gcs_solid_mesh`/`gcs_solid_normals`) apply it, and a pick or a scene's edges turn the eye in
  (`vector_to_local`).  `Sketch::built_solid` is the lookup without the build.
  `tests/rigid_solids.rs` is the gate: the V-twin stood round its crank reuses the disc's,
  cylinders' and pistons' evaluations, each equal to a fresh one where it stands (volume, faces,
  bounds and round features, its boundary on the fresh one's within the two sags), the plate the
  very solid it was, home again the evaluations built there, and a rim made larger built again.
  `bench scrub` (in `make bench`) times a step: the stand against the objects asked for where it
  leaves them and against the same built afresh — 5 ms, 1.5 ms and 507 ms on the V-twin when this
  landed, so a step is the stand.
  **Solids of one binding are one evaluation placed** (#566, [0.177]): a closed instance's solids
  are a function of its definition and its arguments, in the frame it stands in, so the language
  stamps each solid one makes with an opaque key and where the instance stands
  (`model::Binding`: the key, the instance's prefix `at`, its frame, and the turn of the `ring`
  copy it is in; `program::bound::stamp`, from `flatten::Bound`) — the core reads nothing else of
  Solvent.  Of the solids sharing a key, the first by index is the representative; every other
  is it placed (`Sketch::bound`: the motion `Rigid::between` the two instances' poses,
  `Sketch::binding_pose` — the frame's basis, the world's axes for none, turned about the ring's
  centre as the copy's entities are, `Turn`).  `evaluated_solid` and `built_solid` ask
  `placed_solid` before the per-index memo: the representative's `Shape` renamed once from its
  instance's names to the copy's (`EvaluatedSolid::renamed`, `Renamed`: a serial of its own,
  kept while the representative is that shape) and moved by the motion (`placed`), remembered in
  `Caches::placed` as a `Placement`: the representative's evaluation by identity and the motion.
  `exact_solid` places the representative's B-rep likewise (`Exact::placed`, `Caches::placed_exact`),
  and a recipe reaching a solid of a binding several share takes its B-rep from `exact_solid`,
  moved into its place (`exact::build`, `Sketch::shared`), so a union of a ring's instances builds
  one — its faces named in their two halves again (`Exact::halved`, as the export's mirrored
  blank takes one too), so the body's
  faces read by their routes (`joint[1].pad.cd`), and a round over a closed instance's faces in a
  frame rounds them (#760: through it they read as their keys, `#41.1.joint.pad.cd`, which no path
  names).  A key holds only while it does (`Sketch::shares`): a solid reaching one its instance did
  not make (a tool joined from outside) or a swept one is its own — but to the exact export, where
  a closed instance's swept body whose motions are the instance's own is its binding's first
  unmirrored solid's build placed, built once a drawing and tolerance (`Sketch::bound_where`,
  `Caches::swept_exact`, `brep::export::exact`;
  `swept_speed::two_closed_instances_of_a_swept_gear_are_built_once_and_placed`) — a solid redefined where it
  stands (`Sketch::redefine`: `join_body`, `set_prism`, `set_turn`, a stratum's body) loses its
  stamp, a refusal of the representative's is asked of the copy itself, and
  `graft` drops the stamp.  Keyed by provenance, never by content: equal content is the tests'
  claim, not the lookup's.  `tests/bindings.rs` is the gate
  (`two_closed_instances_with_equal_arguments_in_two_frames_are_built_once`,
  `a_copy_whose_frame_moves_is_placed_again_and_not_built`,
  `unequal_arguments_are_two_builds_and_an_open_instance_is_built_as_its_own`,
  `a_ring_of_instances_is_built_once_and_each_copy_stands_turned`,
  `a_union_of_a_rings_instances_builds_one_of_them`): one build said to a listener, each placed
  copy what a fresh evaluation of its own solid makes (volume, faces by name, every mesh corner and
  the bounds within 10⁻⁹ mm).  The hub motor's magnets and slot cutters are two
  bindings under 95 placements: 243 B-rep nodes built for the assembly's run before, 97 after, its
  wall time unchanged (the leaf prisms were cheap).
  **A mirrored instance's solid is its body's evaluation reflected** (#578, [0.178]): the
  language stamps each solid a mirrored instance makes, however deep in its body, with that
  instance's binding — the key it shares with the same binding unmirrored — and `Binding::mirror`,
  the axis of the frame it stands in its image reverses (0 its `u`, 1 its `v`, 2 its normal).  The
  solid's own entities are the preimage's, solved where the body stands unmirrored, so it is its
  binding's representative's own evaluation (`Sketch::own_solid`, the memo, its binding unread)
  carried where its preimage stands and reflected in the frame's plane square to the axis
  (`Sketch::bound`: `Bound::raw` the motion between the two poses, `Bound::motion` that after
  its `mirror`, a `Rigid::reflection`; a mirrored solid is always bound, `Bound::tried` falling
  back to its own preimage reflected where the representative does not build).  So a left and a right of one binding are one build (`placed_solid`), whichever
  comes first, and `exact_solid` places the representative's own B-rep by the improper motion
  (`Exact::placed` reflects it, `Brep::reflected`); `exact_preimage` is the B-rep where the preimage
  stands, which the solid's own evaluation, its built field and its faces read, and a recipe of one
  of the instance's solids takes an operand of its own instance shared with others as that; a
  mirrored operand of another instance is taken as it is published (`exact::build`).  The
  placement's rotation is improper (`EvaluatedSolid::improper`): the shape is shared and its own
  outputs are unchanged, and every world output written as polygons or triangles turns its winding
  so a face still faces out — `world_boundary`, `oriented_mesh` (STL, glTF, `world_mesh`, and the
  ABI's `gcs_solid_mesh`/`gcs_solid_normals`), a claim's `relative` read of one against another
  (wound back where exactly one is reflected); a host placing the shape by `matrix()` reads a
  determinant −1, which three.js and glTF wind by.  The native export takes a body that reaches a
  mirrored solid among its operands as the core publishes it (`export::exact` reads
  `exact_solid`, the body's own or an outside one built of it; a sweep in it is refused).  Volume, mass
  and `clear` read the shape, unchanged.  `tests/mirrored.rs` is the gate
  (`every_image_vertex_is_its_preimages_reflection_and_the_volume_is_equal`: in each of the three
  planes, every exact vertex the right's reflected, the volume equal, the check holding, the mesh
  wound outward; `a_left_and_a_right_of_one_binding_build_once`, either order;
  `a_body_built_of_a_mirrored_solid_reads_it_reflected`, in the core and the export alike).  The hub motor's
  covers and its bearings, spacers and locknuts are mirrored: 17 builds for its objects before, 14
  after (three figures under six placements), every one of its 226 solids' vertices the same to
  5·10⁻¹² mm.
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
  walk's).  **A radius is read by its magnitude**: an arc's and a circle's rows read their radius
  squared or as a point's distance, so a solve may leave it negative, and every reader of a
  solid — the CAD recipe's circle (`solid/cad.rs`), a revolved patch's meridian
  (`solid/surface.rs`), admission's ball — takes its absolute value, the arc's angles being its
  own ends'.  **A triangle with no plane stands off the surface by its distance from its sides**
  (`brep::mesh::off_at`): a sphere's first triangles join its seam's samples either side of it,
  two corners one point, and read by its plane's distance such a strip stood off by nothing, was
  never refined and was dropped, the whole face with it.  `tests/seated_ball.rs` is the gate: a
  solved radius flipped builds the solid of the positive radius, and a ball seated in another
  frame (finding 19 of `shaft_drive`'s README, a bearing's balls) meshes whole.
  `tests/solid.rs` is the gate and checks against **arithmetic, not a second kernel**: a block is
  `w·h·d`, a bore takes exactly its faceted polygon, a flush bore and one drilled past are one
  solid, a boss's shared face counts once, a revolution is Pappus.  `tests/solid_lang.rs` holds
  the language, `tests/derived.rs` the pictures.
- **Solids as sets** (#737, [0.199], Solvent §6.9).  `solid(S)` is the one constructor, the points
  inside a set, and `solid()` the empty body; the language has no prism, draft or revolution of its
  own.  A set may be written in place in the brackets — a set literal, or a family's call, its
  argument a `face(…)` written in place (`std.solids.Through(face(hole))`) — made its own
  statement under a key the source cannot write (`parser/statements.rs::hoist_in_place`, #745),
  read where it is written: no token is rewritten.  Each such statement names the statement whose
  text holds it, `Chained::InPlace(host)` — the solid's, or the call a face is given to — and every
  reader asks that, never the key's spelling: the flat printer writes each where its host reads
  it (`syntax/print.rs::in_place`, the program's statements kept), the outline puts a face's row
  under its solid's (`overview/outline.rs::solid_written_in`), the gestures find the call and its
  face (`edit/solids.rs::reworded`), and W202 tells a bare call from one held in place.  The
  old labels (`depth: from: to: draft: about: sweep: sense: through:`) are E001, the message
  naming the replacement written from the document's own arguments, and `sweep(…)` is retired the
  same way (`syntax::print::solid_word`/`sweep_text` spell them).  **The words are
  `rust/lib/std/solids.sv`** (`use std.solids`): `Prism(section, from, to)`, `Drafted(section,
  from, to, draft)`, `Revolution(section, about, from = 0deg, to = 360deg)` (`p coincident(under: about,
  over t in (from, to)) section`: a line named `under:` is the turn about it,
  `motions::turn_about`, a motion minted once a line and holding nothing, frames.md),
  `Through(section)` (`{ p | p inside section }`) and `Countersink(rim, angle)` (#765, [0.201]:
  `Drafted`'s lean at `tan(angle / 2)` and the one ordinate `<= 0mm`, the cone under a rim to its
  point; no prism for want of a far ordinate, it stays a region the meridian folds about a whole
  circle, [sets.md](sets.md), its faces `near` and the rim's edge).  **They are recognised by their terms, never by
  a word's name** (`gcs-solve/src/solid/defined.rs`, `recognised`, called from
  `program/solids/build.rs`'s `region`): a section between two stated ordinates along its own
  plane's normal is `SolidDef::Prism`; with the lean term a drafted prism (the angle read to a
  billionth of a degree); a face's patch swept by a turn about a line in its plane
  (`MotionDef::Rotation`/`Joint`, no advance) over an interval one end of which is nothing is
  `SolidDef::Revolve` (an end below nothing `Sense::Cw`; a whole turn exactly `TAU`); an unbounded
  section alone is `SolidDef::Through { face }` (by hand, `defined.rs`'s `through`;
  `std.solids.Through` read so by `flatten/said.rs` below), which **names
  no body** (#746): the evaluator — the faceted term (`solid::document`), the field
  (`solid/field/document.rs`), the exact recipe's `through` node and the native host — bounds it
  by its context, spanning along its normal the material of every body it, or a copy of it placed
  at a roll, is cut from or kept in common with (`solid::uses`), each read from where that copy's
  placement puts the section (`document::through_span`; the recipe's `uses`, a `matrix` each),
  so one pin goes through any number of bodies and a pattern's copies each through their own;
  cut from none it is E080 at its declaration.  So the same set written by hand builds identically — the same exactness,
  face names, volume, STEP and STL bytes.  A number a term reads from an unknown is no stated
  ordinate.  A set none of these recognises stays a region (`solid::region`, [sets.md](sets.md)),
  read point by point — shown, meshed, measured — its exact export refused by the region's own
  routes.  **The faces are named by what bounds them**, one rule (`defined.rs`'s module doc) for
  `std`'s words and a set written by hand: a section's sides by the edges they are swept from; of
  two ordinates the greater bounds `near`, the lesser `far`; a turn short of a whole one is capped
  `start` and `end`.  **Before the solid is built, the flattener reads the same sets as the sweep
  they say** (`flatten/said.rs`, the precedent the developables' `sheet_body` set): a set's body —
  a family's with its formals bound to the call's arguments as written (`said::binding`), or one
  written by hand in the solid's own scope, its names what they name there (`said::Bound::Own`,
  `Walk::said_by_hand`, #744) — that is a section between two ordinates, the drafted lean, a face's
  patch turned about a line, or a section alone (naming no body, #746) is written out
  as `syntax::Sweep::Prism`/`Revolve`/`Through`, so every reader
  of a solid's text before it is built reads it as one: a prism's sides for an envelope
  (`generated.rs`), a revolution's named side (`envelopes.rs`, `surface(S, e)`), a sketch's
  carriers on a face (`flatten/faces.rs`), the gestures' resizing (`edit::solids::reworded`, which
  reads the call's family's set through `said` and rewrites the call as the `std.solids` word
  saying the new sweep); and no probe is made, so the solve is the old spelling's to the bit,
  whichever is written.  **A revolution carries its interval as written** (#747), `Sweep::Revolve
  { axis, from, to }`: which end is nothing, how far and which way are what the ends evaluate to,
  decided in `program/solids/build.rs`'s `revolve` by `model::turn_between` (the reading
  `defined.rs` shares), so `to: 2 * pi * 1rad` is `to: 360deg` and `from: 0rad` is no end at all; a
  turn from the greater roll to the lesser is E040, one of nothing E080.  The flattener decides
  nothing on text: it keeps the set only where the same number evaluator reads both ends and
  neither is nothing (`Walk::off_its_face`) — a turn off its face, which no sweep of the
  evaluator's is, so it stays the region it was; the turn gesture takes a number for it and sends
  an expression to where it is written.  `tests/revolve.rs`'s
  `an_interval_is_read_by_its_numbers_not_its_text` is the gate.  (An evaluator's revolution that
  carries the interval itself, turned `from` and swept `to − from`, would build such a turn exactly
  and retire the exception.)  **Nothing recognises a sweep from a word's name or a statement's
  printed text**: a section is within its face by its side (`Written::side`, so `inside` and its lowered
  `coincident(<=)` alike), and the draft by the bound's tree — `Bin(Sub, distance(p, x),
  distance(p, y))` at least a product, either way round, of `distance(p, F)` and a number whose
  expression is the one call `tan(θ)` (`said::lean_of`).  A number that reads a formal under
  arithmetic (`-t`, `2 * t`) is no number of the call's, so such a set is left to the elaborator,
  which reads it in its own scope.  `defined.rs` is what recognises any other set saying the same
  (a turn under a frame's motion, `under: turn`, among them).  `tests/solids_as_sets.rs` is the
  gate (each word against its set by hand, a set by hand adding nothing to the solve and sketched
  on, a revolution by hand's named side, a draft read off its tree, a through prism bounded by
  what it cuts — two bodies at different heights, a copy through its own — the retired labels naming the set), and `tests/extrude.rs`'s
  `a_family_saying_a_prism_is_resized_by_its_set` the gestures'.
- **The sheet is a report** (Solvent §6.12, `hidden::generated`; issue #48).
  `dimensions(body) in views.right` asks for the callouts that *follow from the object*: overall
  extents in that view and the diameter of every round feature seen square on.  They go through
  `callout::layout`'s pen and lanes, and their ids start at `callout::GENERATED`, past any
  constraint: a reading of the *solved* drawing, not a statement in it.
  **The boundary is the feature.**  Which datum, which fit, what is reference or controlling is
  design, which the sheet states.  `tests/sheet.rs` asserts the three it makes and no fourth.
- **A claim about a solid is judged, and can never act** (Solvent §9.8, `clear.rs`; issue #48).
  `disc clear(2mm) cyl`, `head fits(0.15mm) trap`, `every q in piston: q in bore` ([0.206])
  compile **no row** —
  `tests/solid_claim.rs` asserts parameter count, equation count and DOF unchanged.  The words
  are in `constraints::OPERATORS` and settle to no `CKind`; `constraints::solid_word` reads the
  word before the spec, like `gauge_op`.
  **What a reader is owed is the measurement**, not a yes or no, with the *sagitta* beside it: a
  claim decided within the faceting is **undecided**, a third answer.  The implicit min/max
  reading of a term is only a lower bound for a difference, so it culls; the answer is piece
  against piece.
  **And where** (`clear::Nearest`): the face of each solid by its public path
  (`vocab::public_path`: `near_screw[1].post.rim`) and a world point on each — where two apart
  come nearest, of pairs as near the two facing most squarely; one point where boundaries touch or
  cross; and where material is common, the centre of the deepest ball `overlap` found
  (`deepest`), with the face of each nearest it.  A sweep that held the moving operand still says
  its points where the motion stands them (`GeometricEvidence::moved`).  The text says it under a
  claim refuted or undecided, the JSON (`nearest`) always.  `solid_claim.rs`'s
  `a_refuted_claim_names_the_faces_and_where`, `a_refuted_sweep_says_where_at_its_counterexample`.
  **A swept operand is measured from its exact B-rep** where this kernel builds one, at every
  pose a claim reads (#615; #904 A11 for the last three): one pose, a roll moving an operand
  rigidly, a copy re-placed along a roll, the drawing stood at an instant of a declared parameter
  or solved at a sample of a free one.  The judge supplies it (`brep::export::supply_exact_of`,
  `diagnose::solid_claims::geometry`) before reading, as a drawing reads an object's — a
  placement's build its source's, built once and moved — and meshes the field only for a build
  refused: a gear train stood at each sample meshed both gears' fields, minutes a pose.
  `planar_subtraction.rs`'s
  `a_claim_over_a_parameter_reads_the_cut_wheels_exact_build_at_every_sample`.  A lone claim that cannot be measured
  says why on its line (`— undecided: <reason>`, the solid named where its field would not
  mesh); "no solved valid poses" is a sweep's, whose poses are solved.  `planar_subtraction.rs`'s
  `a_claim_about_a_cut_wheel_reads_its_exact_build`, `solid_claim.rs`'s
  `a_claim_not_measured_at_the_drawing_pose_says_why`.
  **`claim over crank.theta in (0deg, 360deg) { … }`** sweeps one of the drawing's **free
  variables** (never a `param` that is a number), reporting the *worst* pose — of a refuted one
  the counterexample with the least room (`SolidVerdict::counterexample`), not the first; it is
  **sampling** at `SWEEP_STEPS` and says so.  **Over a declared parameter** (#633, [0.197]),
  `claim over t in (a, b) { … }` or `claim over t { … }` (`syntax::ClaimOver::declared`, its ends
  written in by `walk_claim_over`; a stretch outside the parameter's interval, or `over n { … }`
  naming none, E103): a copy of the drawing stood at each sample (`Poses::Stood`,
  `follow::stand`), which places a solid drawn in a frame whose value reads `t` where that value
  puts it (#697, parameters.md's #398 L); a march refused or an instant not reached is a pose
  failed, saying why.  `solid_claim.rs`'s
  `a_claim_over_a_declared_parameter_reads_the_drawing_where_it_stands` (the arm into the post
  through its depth from 42.5° to 47.5°, measured -5, and the closed form's room at every sample
  clear),
  `a_claim_over_a_declared_parameter_follows_its_march` (a slider-crank against its closed form)
  and `a_claim_over_a_declared_parameter_is_refused_outside_it`.
  Claims are read **after phase 4** (a free variable is what `expr::evaluate`
  allocates), and the interval **in the unknown's own units**, never converted as an angle.
  **A claim over a frame's roll turns only what is placed under that frame** (#904 A11,
  [0.214]): a solid it names placed at an instant of another motion stands where it was read
  through the sweep, whatever ratios tie the frames, and elaboration says so at the claim's
  operand (W117, `program/solids/claims.rs`'s `solid_claim`), naming that frame and how to run
  frames together: their values written over one declared parameter, each solid drawn at its
  frame, the claim over the parameter.  Only a solid it names: a copy placed at an instant inside
  a body it names is how a pattern is drawn (`hypoid_drive`'s spokes) and stands still because it
  is drawn still.  `solid_claim.rs`'s `a_claim_over_one_frame_says_what_it_leaves_standing` (a
  peg against a peg held still, against its closed form) and
  `a_train_tied_to_one_parameter_is_judged_as_it_rolls` (the same pegs turned at their ratio).
  **A refuted pose costs its depth's search, indexed** (#904 A11): where two solids share
  material, `clear::overlap` searches the deepest ball in the common region at up to 8192
  points, each the nearest piece of the common boundary (a BVH, `bvh::Bvh::query_nearest`) and
  whether both solids hold it (`Csg::inside_indexed` over each primitive's `RayIndex`) — the
  same answers as every piece and every facet read, in their logarithm: a refuted sweep of two
  gears read every facet of both at every point of every pose, minutes a pose.
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
  **A refused solid is said by whoever builds it**: the diagnosis says one among its warnings
  (`solid::validate` of each solid that is no body) unless asked not to
  (`DiagnoseOptions::refusals`, true by default, as `solventc` says it), and nothing else it
  says changes.  A page with a pool asks not to (`refusals: false`): validating a solid that
  rolls finds its roll, a march along its motion, seconds an edit on the page's thread, and the
  pool's workers validate each object they build and say its refusal in the export's words —
  the outline's *not built*, the build bar's list, and Tools ▸ Diagnose, which lists each object
  the pool refused and how many are still being built.  Gate:
  `a_refused_solid_is_said_among_the_warnings_unless_the_host_says_it`.
- **One builder serves every object, and the page draws only what it builds** (#670, #671):
  `brep::export::Builder` builds any object — a static body as `export::exact` builds it (one stage,
  then its mesh), a swept one by the generating class's stages — to a `Display`: its mesh, each face
  named as `solid::exact::named` names it, the B-rep's edges (`solid::ExactEdge`, sharp or smooth),
  its volume and what the solid read; a refusal is the export's.  That is all a page draws of an
  object, one build a binding, supplied to its sketch (fields.md, "The page never does"), and
  `exact_solid` builds a body a generating sweep cuts as the export does (#536, brep.md).
  `object_builder.rs` holds the builder to the export over the case library (the whole of it in
  the slow tier), and `object-builds.test.ts` the page to it.
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
- **The objects leave as one STEP assembly** (`brep::export::assembly`, `step::write_assembly`):
  AP214's product structure, a root product named for the document, **a part per object** and a
  sub-assembly per step of the objects' names as a reader writes them (`vocab::public_path`:
  `wheel.hub` is `hub` in `wheel`, `p[0].body` a ring's copy), each sub-assembly placed at its
  parent's axes.  **The objects of one binding are one part, built once** and used where each
  instance stands: a solid a closed instance made is written in its instance's frame
  (`Sketch::binding_pose`, orthonormal, in millimetres) and used placed by it, so the part is the
  representative's build (`Sketch::bound`) moved into that frame, named for the first object it
  is (the representative may not be in the file).  A representative that is
  mirrored publishes its build reflected, which is not the binding's own evaluation, so the
  part is then the first unmirrored object of the binding's; **a mirrored object is a part of its
  own**, its reflection in its solid and its placement proper (STEP places by right-handed axes).
  Any other object is written where it stands and used at the root's axes.  A part is written
  once however often it is used: each use a next assembly usage occurrence, its shape the
  transformation carrying the child's axes onto its placement, each an item of its
  representation.  The file is parsed back against the structure it was written from
  (`step_check::verify_assembly`): each product by name, a part's one solid walked as a
  single-solid file's is, every face, edge and vertex one part's, and every use one of the
  structure's with its placement.  One part refused refuses the file, said at that solid's
  statement.  The core's kernel writes it (`solventc --step-assembly`, `--solid` narrowing it to
  the objects under a name); OCCT, which writes one solid, is refused it.  `tests/step_assembly.rs`
  is the gate (`every_object_is_its_part_carried_where_it_stands`: each object's part, carried
  down the tree, has the object's own build's volume and bounds, and each one's every vertex lies
  on the other's boundary).  **A part is written in its colour** (#677): `step::Product::colour`,
  sRGB in 0..=1, the first object's it is built for (`export::assembly`'s `colour`, by solid
  index), written as AP214's presentation on its solid — `STYLED_ITEM` → presentation style
  assignment → surface style usage (`.BOTH.`) → side style → fill area → fill colour →
  `COLOUR_RGB`, one colour's chain written once and shared — the styled items listed by one
  `MECHANICAL_DESIGN_GEOMETRIC_PRESENTATION_REPRESENTATION` in the shapes' context; only a part is
  coloured (`Assembly::check`).  `verify_assembly` reads them back: a coloured part's every solid
  styled once, in its colour within `NEAR`, an uncoloured one's by nothing, every styled item a
  part's solid's and presented; `step_check::colours` reads each part's colour from a file
  (`each_part_is_written_in_its_colour_and_read_back_so`, `a_colour_not_the_parts_is_refused`).
  **A part's colour is one rule for the page and the file** (`colours.rs`, #676/#677): a part is
  a solid's binding (`model::Binding`'s key), else the solid; its colour is the drawing's `fill`
  (`drawing::fills`/`drawing::colours`: a rule outside every sheet naming an object, a scope of
  objects or a solid one is made of, every object of that part taking it, later rules over
  earlier) and otherwise the palette's (`colours::palette`: twelve muted colours, indexed by a
  digest of the part less its numbers — `model::Binding::part` — and moved on to the next one no
  earlier part, in object order, has taken; so a reload and a numeric edit keep it, and parts
  differ while there are colours to go round).  `solventc --step-assembly` writes them, the
  drawing `--drawing PATH` names or the `.svd` beside the model with its stem (`bot.svd` beside
  `bot.sv`), and `--png` shades in them (`part_colours.rs`; the corpus case
  `examples/plates/`, `a_step_assembly_is_written_in_the_drawings_colours`).
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
  memoised `Want::Mesh`; the same mesh as a renderer draws it, in the solid's own coordinates —
  over vertices shared within a face, with its sharp and tangent edges (`mesh::drawn`,
  `gcs_solid_drawn`, `core/mesh.ts::drawn`, #674) — and where it stands (`gcs_solid_placement`:
  the shape's serial, its geometry's — one for the solids of one binding — and a column-major
  4×4, `placement`) are what the box builds from (#412; `tests/drawn_mesh.rs`).
  `Document.solids()` names them (a solid has no proxy).  `tests/mesh.rs`
  is the gate and asserts the *before* as well as the after.
- **A mesh is cut to the object and a volume to the report** (`solid::mesh_unit`, `MESH_SAGITTA`).
  An exact solid reports its B-rep's own volume and area, its faces' fluxes and areas summed
  (`EvaluatedSolid::volume`, `area`; `solid_lang::a_reported_area_is_the_exact_solids`); for any
  other, `REPORT_UNIT` makes a *volume* good to 1e-4; a mesh is cut to a sagitta that fraction of the
  *solid's own diagonal* (scale-free), asked of the primitives' boxes.  `gcs_solid_mesh_unit`
  publishes it, and **a unit at or below zero *is* that choice**, resolved at the one seam
  (`Sketch::cut_unit`, read by `solid_boundary`, `solid_edges`, `solid_mesh`) — never per entry
  point (a sagitta of zero hangs the tab).  `tests/mesh.rs` asks it of all three.
  **The glass box asks with 0, and that is why zooming is free**: `unit` (a screen pixel) is
  wrong for a scene with its own camera.  `overview::scene3d` asks the same way (`SCENE_PX` for
  drawn polylines, `0` for the object's edges), so `Box3D`'s rebuild key omits the zoom.  **It
  includes the pose** (a fingerprint of the parameters): a drag moves numbers inside one sketch,
  so the datums (`overview::Layer::Datums`: panes, their axes, axes in space) are rebuilt every
  frame the pose moves.  **An object is meshed once and moved by its matrix** (#412): each is a
  group of its creases and faces in its own coordinates (`own`), kept while its shape's serial
  holds (`placement`) and given its new matrix every frame the pose moves, a drag or a scrub
  included; one whose shape changed is cut again, under a gesture or a wobble only where the core
  has it already (`placement(…, built)`, else it waits for the gesture to be let go) or its shape
  was changed in place (`shapeEpoch`).  `Box3D.sync` brings the scene up without rendering, and
  `stats` counts what it cut and what it only moved: `timeline.test.ts` scrubs
  `vtwin/assembly.sv` cutting nothing.
- **A solid is cut by a plane off the mesh it is drawn as** (`mesh/section.rs`,
  `EvaluatedSolid::section`).  The plane takes away the side its normal points to, and **a point
  on it goes with what stands in front of it** (`Section::height` snaps within `1e-9` of the
  mesh's extent, the weld's resolution, to nought, and nought is taken away): a face lying in the
  plane is cut along its rim and nowhere else.  **Each crossing is a point of a mesh edge,
  computed once** — filed by the edge's corners' bits, lower first, then by its own bits, so
  crossings at one corner on the plane are one point — and since the mesh is welded, a closed
  surface's crossings chain into closed loops with no tolerance (an odd-degree point starts an
  open chain, drawn and never filled).  **Which loop is a hole is how deep it is nested**, never
  which way the triangles wound it: even depth bounds material and is turned counter-clockwise
  about the normal, odd depth is a hole in the smallest loop one level out and turned clockwise.
  The face the cut leaves is each outer loop with its holes bridged in (Eberly, from each hole's
  rightmost corner) and its ears clipped by the exact `orient2`, a turn within `1e-12` of
  straight dropped as no corner (three points along one side made a sliver either way); its
  triangles are counter-clockwise about the normal, the face facing the side taken away.
  `Section::keeps`/`clip` are **the one rule** every consumer of a cut reads — `solventc --png`
  leaves out each sample and edge the cut takes away by it, the workspace's picks pass by them
  (`Sketch::cut`, app.md) — so what is drawn ends where the face begins.  `section_plane` is the
  rule with nothing sliced, for a pick.  Gates: `tests/cut_planes.rs` — the plate cut across its
  depth (outline counter-clockwise, hole clockwise) and through its hole (two pieces, either
  normal), a plane on a face or past the solid, a turned part across its axis (a ring), and
  library parts (flange, pulley, tray, elbow, a pattern) cut through their middles along each axis
  both ways, every cut closed, on its plane, facing the side taken away and **exactly its loops'
  area** (outer less holes, to `1e-9`).
- **A face is one loop and a solid's faces are named by path** (§6.8).  A face is a closed loop of
  edges on the one plane every point agrees about — *read* off the memberships, never written on
  the face.  There are **no holes**: a hole is a solid that `cut`s the body.  An `in` block leaves
  a face and a solid alone, as it does a datum — they bear no points, and refusing them would
  split the design and the solid it is a section of across blocks.
  A boolean **never renames**: `block.near`, `block.far`, `block.side_l` (a prism), `bore.axis`,
  `bore.start` (a revolution), and through operands — `body.bore.wall`, `part.base.pocket.floor`,
  each operand by its whole path from the body's scope (`part.hub.body.ab` for an instance's
  `hub.body`, `part.joint[1].pad.near` for a copy's, [0.203], #760, [sets.md](sets.md)).
  A name whose face a boolean ate is a *fact the report carries*, never an error.
  E080 a face that is not a loop on one plane, E081 a revolution's axis, E082 a face a body no
  longer has, E084 a section cut across its own view.
- **A solid is called by the name the language reads it by** (#904 A14, `Sketch::solid_name`):
  a block's copy by its index on the declaration it holds (`vocab::public_path`) — the `pin` copy
  1 of a `repeat` in an instance `box` makes is `box.pin[1]`, the `body` of an instance `planet`
  made there `box.planet[1].body` — never the key it is held by (`box.#15.1.pin`), whose
  statement id moves whenever the file's statements do.  The key stays `SolidE::name`, what
  faces, layers, uses and a binding's renaming route by (statement identity under expansion,
  [relations.md](relations.md)); what is *said* reads `solid_name`: the timings, the reports, a
  refusal of export, glTF nodes, an STL's header and a STEP product, the objects the page is
  handed — and a refusal is placed by the solid it so names (`program::refusal_site` reads the
  name or the key).  **A name picks a solid by one rule** (`Sketch::solid_called`, `solventc --solid`):
  the name a solid is called, or its key; a name a block's copies go by without their index
  (`box.pin`) is refused saying each copy, one two solids go by (a copy of a block within a
  block's copy, its index written once) saying their keys, and one nothing goes by saying the
  objects there are, so the name to write is learnt from the refusal.
  `a_solid_a_blocks_copy_makes_is_called_by_its_index` (`tests/face_names.rs`).
- **One cause is said once** (#572, `program::refused`).  After the solve a solid is refused
  (E080) in its own words only where nothing it is built on is refused (`solid::built_on`: its
  operands, and the solids a region's terms read — the body whose faces a round reads; a body is
  never refused itself, its operands are).  A solid refused only because such a root is, and a
  face borne on a body built on one (E082), is said by its statement **once**, as not built:
  **N080**, a note naming the roots, where each is said and the named solid between ("not built:
  it is built on `body`, made of `slot` (21:1), which could not be built"), `Diag::cause` the
  first root's span (`cause` under `solventc --json`).  So a round is one line however many
  `taken`s and `filled`s it made, and no piece of a region is a diagnostic.  Each is said where
  the document holds it (`refused::Placing`): a site in a module at the deepest instance on its
  path the document holds (the round that used `std.rounds.round`), else at the `use`
  (`modules::localize`).  `tests/refused_once.rs` is the gate.
  An export refused in a solid's words (`` `b.#31.0.#…filled`: its exact construction disagrees
  with the field ``) is said the same way (#881, `program::refusal_site`): at the document's
  statement that made the first solid it names, placed by `Placing` — a round's piece at its round
  — `solventc` printing it `file:line:col: error[E080]` as every diagnostic; at the exported
  solid's statement where it names none.
  `round_lowering::a_refused_rounds_piece_is_said_at_its_round` is the gate.
  A continuous sweep is drawn by its material field alone, so one whose field cannot be read is
  refused too (`solid::refusal`, what `refused` asks of every solid), at its statement and in a solved document as in any other (#588: "`space` sweeps
  `pyramid`, whose material the field does not read: …"), and what is built on it follows as
  N080 — never a solved report and a solid that silently names nothing
  (`planar_subtraction::a_sweep_the_field_cannot_read_is_said_at_its_statement`).
- **A solid drawn and not built says so** (#762, `EvaluatedSolid::unbuilt`).  An evaluation that
  is not its solid's exact build — its material field's surface, where `sweeps_among` finds an
  operand no exact construction builds, or the facet term, where `exact_preimage` is refused —
  carries why (`solid::Unbuilt`: what it is drawn from, the first refusal in the core's words,
  how many more): a region's refusal by the entry that recognised it (a round's gate: "its exact
  construction disagrees with the field at …"), or that none builds it; a sweep no ball's tube
  builds; a shell's cavity refused (`solid::field_only_reasons`, `sweeps_among` read for its
  reasons, building nothing it did not).  An exact build's mesh, a host's included, a solid
  built with no material, and a developable panel's band (whose faceting from its rulings is its
  build, #645; a body built on one carries its refusal) carry none, and a placement of a binding's representative carries its
  representative's.  A field the mesher cannot close (`FieldMeshing::Now`) is refused with the
  build's refusal beside its own, in `Unbuilt`'s one wording ("…; the field's to mesh, not
  built: …"), the cause to fix.  Whatever writes a look at a solid from an evaluation — `solventc --png`,
  `--gltf`, the faceted `--stl` — says it against the file it wrote, in `report::unbuilt_text`'s
  words ("`keeper` is drawn from its material field, not built: …"), the file still written and
  the exit unchanged, since what was asked is all there: a quick look is never read as "this
  builds" when the export would refuse it
  (`shell::a_shell_not_built_is_drawn_from_its_field_and_says_why`,
  `cli::a_picture_of_a_solid_not_built_says_so`).  The other way round (#904 A15): an exact
  export refused never writes the field in its place — its file is the build held to the
  tolerance, and a field's mesh is neither — but where the solid's boundary is its field's
  (`Sketch::is_swept`) an exact STL refused is followed by what writes that reading, in the core's
  words with the host's name for its faceted output (`report::field_read_instead`: "`tumbled` is
  read by its material field, and `--stl-backend mesh` writes that reading instead: …").  The
  faceted STL is the evaluation's, asked of no class: admission gates the exact outputs only
  (`kernel_blank::a_field_read_body_is_offered_its_fields_writing`,
  `cli::a_body_refused_exactly_names_what_writes_its_field`).

- **A face family is a face that runs** (§6.9 [0.97], `solid/family.rs`; #243 L1).
  `solid({ p | some s: p in I(…).f at s })` ([0.199], #737, [0.205]; through 0.198 `sweep(I(…).f
  over s)`) is read by the parser as the family's tokens (`parser/statements.rs::swept_set`) and
  parses
  to `Sweep::Family(FamilySpec)` (`parser::family`, by an `over` inside the brackets,
  `family_ahead`), `s` a declared parameter whose interval it
  runs over (#398 B, [0.145]: `Walk::family_parameter`, the formal the call gives it or the one
  named as it, `Held::formals`; `over s in (a, b)` is refused); the flattener walks the instance
  **drawn**, under a key the source cannot write (`#f<offset>.`, so its declarations are
  anonymous), the running formal left unbound so `bind` makes it the free value `#f12.s` — and
  **held** (`flatten::Held`): a text affine in it carries it as any unknown's, its coefficient
  written with the dimension the two differ by (`of_vals_in`: `(-12mm * #f12.s + 24mm)`), a text
  that is not is kept and written in where it is read, as a trace keeps one; a pin `t == s` ties
  to it, a seed reads it at the middle.  The elaborator holds each (`Sketch::hold`): a **fixed**
  parameter, which `expr::evaluate` reads as a number (no free variable, any arithmetic), which a
  contact pinned to it owns (`Sketch::add`), and which `owned_arg`/`graft` carry by name.
  `SolidDef::Family { face, station, over }`, `over` its running number as a parameter axis
  (`model::Running`, [parameters.md](parameters.md): its name the key in `Sketch::held`, its
  dimension the formal's, its ends as written): `station` is every entity the instance
  made (by key prefix, the face's edges and their children) — `Sketch::stations`, which
  `drawn()`, the panes, the pane picks and `callout::layout` leave out.  **The drawn station is
  solved with the drawing at the middle**, so the document's DOF and diagnosis are its too.
  **Evaluation re-solves it** (`family::built`, memoised in `Caches::family` against
  `MemoKey::at`, which reads every parameter and every constraint's number): the station's part
  (`graft::Part::around_all` from the face, every primitive, plane pose, lift and outer free
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
- **A loft is library words over the family** (§6.9 [0.100], `rust/lib/std/stations.sv`; #243 L3).
  The station words say only where a station stands: `std.stations.OnLine` (a fraction of a line's
  run), `std.stations.OnArc` (of an arc's turn), `std.stations.OnSpline` (a spline's parameter),
  `std.stations.On` (any `path`: a line, an arc, a spline, a traced curve — L4, a line touching a
  round at a point of it drawn in its plane, `planes::incidences`), `std.stations.Helix` and
  `std.stations.OnHelix` (L6, below) — each a component holding a plane `n` square to its path, its
  `v` along `up` (`On`'s at its home: [0.123], its frame integrated, below).  `On` and `OnSpline`
  seed their tangent ahead along the guide (`hint(at: m, along: g)`, `entities::rate_along`) and
  their plane off it as `OnLine` does, so the drawn station starts on the branch whose `v` is `up`'s
  and whose normal runs the way the guide does — never the mirror, which a sign-free `parallel`
  would otherwise leave to wherever the solve started (`tests/paths.rs`,
  `the_frame_keeps_up_up_on_every_guide`).  The section is the writer's component drawn `in st.n`,
  reading `s`; two sections correspond by being one.  **No Rust reads a word's name**, and there is
  no loft constructor: `SolidDef::Loft`, `Sweep::Along` and `solid/loft.rs` are gone, and `along:`
  in a solid's brackets is refused at the key saying what to write
  (`parser::declarations::sweep_arg`). `brep::build::ruled` joins a section only to its own image (a
  drafted prism's offset, a canal's screwed disc), edge for edge; the recipe has no `loft` node.
  The two-face word (pairing two sketches drawn apart) returns at L8 (finding 8).  `tests/loft.rs`
  is the gate: `solid_loft`, `solid_elbow`, `solid_bend` and `solid_twist` at their closed forms off
  the B-rep to 1e-9, their walls by name and kind, `std.stations.On` agreeing with each kind's word.
- **A guide in space** (#363 S1, [0.122]): a curve traced in space is a `path`, so a station
  stands on it by `m coincident(t == s) g`, `g tangent(at: m) tl` and a plane square to `tl`
  turned by `Minimal`'s rate from a home square to `up` — the frame integrated, as along
  `std.stations.OnHelix` — which is `std.stations.On` itself since [0.123] (S2: a frame holding `v`
  along `up` at every station leaned its plane off square to a twisted tangent, silently).  `On`
  over a planar guide is `OnSpline`'s algebraic frame, corner for corner (`tests/rates.rs`).
  `tests/space_paths.rs` is the gate: a 2 × 2 square along two turns of a traced helix (r 10, rise 10) at 4·√((40π)² + 10²)
  off the exact B-rep to 1e-9 and corner for corner `std.stations.OnHelix`'s under `Minimal`; the
  frame turning −28.5576° a turn against Frenet's (r 10, pitch 5), to 1e-9; a traced surface
  refused; `std.stations.On` on the traced helix `OnHelix`'s solid corner for corner; an off-centre
  square round `pipe.sv`'s U at 4·(60 + 23π), its frame carried across the joints.
- **Frenet's frame** (#363 S5, [0.125]): `std.stations.Frenet(g, s:)` holds its plane square to the
  guide, `u` along `line(m, c)` to the centre of curvature (`g curvature(at: m) c`, `curves.md`)
  and `v` the binormal (seeded `tl` turned −90° about it, the one sign a seed picks, and kept by
  warm starts), an algebraic station — no rate.  Where the curve has no curvature the centre's
  rows lose their rank and the station is refused there ("not held"), never followed.
  `tests/space_paths.rs`: along `ribbon.sv`'s helix as a traced curve it is `std.stations.OnHelix`'s
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
  correspond entry for entry); `face`/`over.name` are the first leg's and `station` every leg's.
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
  **A chain across planes** (#363 S4, [0.124]): a straight between bends in two planes touches
  each by `TangentCircleAt3` (`space.md`), so the bent pipe is one chain.  **A station that
  states a rate on a link past the first is carried** (`program::solids::carry_legs`): its rows
  `Constraint::carried` (no row of the drawing: `states_rows`) and its own parameters — entities',
  points' lifts, planes' normals, contacts' places — held where seeded (`Sketch::carried`), since
  only the last leg's flow knows its frame (a guide along `up` there left the drawn station's
  turn a freedom); `part_of` gives both back to the leg's part.  `tests/space_paths.rs`: the pipe
  of #363 (line, R20 in `std.top`, line, R20 in x = 20, line) a 4 mm tube at 16π(60 + 20π), a
  2 × 2 square at 4(60 + 20π), a bar off its centre line at its centroid's path, and the end
  cap's frame (tangent +z, `u` −x, `v` −y) to 1e-9.  **A chain drawn in space turns either way at
  an arc** ([0.127], S6): an arc over points in space faces its minted plane as its points do, so
  L0's S-bend — a left turn then a right — is one chain (`in_space_a_chain_turns_either_way_at_an_arc`,
  Pappus, its second bend's plane facing −z).  **In a plane, an arc may turn clockwise** ([0.128],
  `Decl::sense`, parsed for an arc as `sense: cw`, read as `Deferred::Clockwise`).  **One no
  chain enters at its start is the counter-clockwise arc from its end** (#573): `arcs_in_space`
  swaps its start and end in its own plane and files it in `Sketch::reversed`, so it owns what
  `arc(center: k, start: b, end: a)` owns — no minted plane, whose attitude and origin cost 36
  unknowns an arc (each `ring` copy its own) and took the blame for the drawing's freedom — and
  everything that reads an end or a fraction reads it as written: `resolve::follow` and
  `Sketch::scalar_names` (`--where`, a seed's read) through `drawn_from_end`, a tangency's `at:
  start|end` (`Constraint::contact_point`), a fraction's `t` (`point_along_arc`'s constant, its
  seed in `infer`), a frame along it (`guide::Path::read`), a seed's step along it; `graft`, the
  JSON (`"reversed"`) and the lifted text (`sense: cw`, ends as written) keep it.  A clockwise
  half turn is E060 still.  `tests/clockwise_arc.rs`: the issue's arc and its ring at 9 unknowns
  and no plane more, a free radius named as the arc's, the names, a tangency at `start` and
  `t == 0.25` against the counter-clockwise arc's `end` and `0.75`, a copy, the JSON and the
  lifted text read back.  One a chain enters at its start is drawn by `arcs_in_space` in its
  plane turned over (normal reversed), counter-clockwise there from its start, so a chain still
  enters every link at its start and its legs run forward — no leg is walked end to start — and
  the S-bend in `std.top` is one chain
  (`a_chain_in_a_plane_turns_right_at_an_arc_said_to_turn_clockwise`).  **A chain walks an arc it
  enters at its end from there** ([0.129]): `program::entities::walked_back` reads each named
  chain before the arcs are drawn — an arc whose end the link before leaves at, or, first, whose
  start the link after enters at — and `arcs_in_space` draws it from its end: start and end
  swapped and its turn reversed (in a plane `sense:` toggled, so it is drawn in its plane turned
  over or not; in space the plane through its points faces the other way), the same arc, so
  `validate_chain`, the joints' `at: start|end` and a family running it forward read it entered
  at its start.  `Sketch::walked_back` keeps the names: `resolve::follow` reads `start`/`end` of
  such an arc swapped, so `k.start` is the point written.  The parser threads a joint whose point
  is an arc's written end (`thread`, both ends written); an arc two chains walk opposite ways is
  E080.  `a_chain_walks_an_arc_it_enters_at_its_end_from_there`: the S-bend in `std.top` with its
  right turn written from `f` to `e` (declared in the chain, or named), Pappus, the names claimed;
  the bent pipe's second bend written clockwise from `f`, both tubes; the S-bend in space.
- **A law is an expression in `s`** (§6.9 [0.103], #243 L5, finding 10).  A twist or a scale is a
  dimension of the section reading the running number; no word takes a `law:` (no function values).
  `std.stations`' `Twisted` (the point `reach` along the station's `u` turned `twist · s` — [0.118],
  the second point a section drawn once is carried to, `f := prof mapped(o -> n.origin, x ->
  tw.t) in n`; through 0.117 a plane of axes a section was drawn on corner by corner; since
  [0.138] a turn of `n` about its origin read at that roll, and since [0.141] `x` drawn in that
  turn standing at `twist * s`, `t := x in turn`), `scaled` (a point at `(du, dv)` scaled `1 +
  (scale − 1) · s` about the plane's origin) and `scaled_radius` take a total and run it linearly;
  any other law is the word given another fraction (`s: 3 * s^2 - 2 * s^3`) or the writer's own
  expression.  **A number given
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
- **A screw is a motion read at a roll** (§6.9 [0.110], [0.138]; #243 L6, #366).
  `std.stations.Helix(base, pitch:, turns:, s:)` is `base` carried by the screw about its normal
  through its origin (a frame's `screws(pitch)` about `base.normal`) to `360deg · turns · s`: `n`
  the base so carried (a section in it sweeps a twisted prism), `axial` `n` turned a quarter about
  its `u` — `u` out along `n.u`, `v` up the axis (a thread's).  `std.stations.OnHelix(base, r:,
  …)`'s `n` stands square to the helix of radius `r`, `v` up the axis leaned by the lead angle (a
  coil's wire): `base` tilted about `base.u` by `90deg + atan2(pitch, 2πr)` and slid `r` out (one
  screw about that axis, read at its turn), then carried by the helix's screw.  Every frame is a
  carried one — since [0.141] drawn in a frame whose value is the roll, `n := base in screw` with
  `fix(value == 360deg * turns * s) screw` ([parameters.md](parameters.md)), through 0.140 `n :=
  base at(under: m) τ` ([frames.md](frames.md)) — about its frames' own axes, so **no direction
  is held by a seed** and no plane stands against its lines — which the ordinates of 0.110–0.137 did
  not ensure: where the solve started decided whether a station's axes ran with their lines, and
  `lofts/spring.sv`'s, `ribbon.sv`'s and `l0/spring.sv`'s thread's drawn stations stood turned (the
  solids alike to 1e-13, the sections' frames not).  A negative pitch winds left-handed, the
  right-handed solid mirrored in the base plane.  The axis is `base`'s normal through its origin.  A
  family such a station sweeps is the screw's sweep (`stated_carry`, `generating.md`): its walls the
  first rim carried by the motion's own screw. `tests/helix.rs` is the gate: square and V threads at
  A · 2πr̄ · turns (Pappus, r̄ the centroid's), flights square to the axis at A × the rise
  (Cavalieri), a spring at A × the helix's length, left-handed mirrors (corner for corner, matched
  by place), each off the exact B-rep to 1e-9, and the fast path (`brep.md`);
  `tests/carried_frame.rs` holds each station to its closed form.
- **Several sections are drawn on rails** (§6.9 [0.111], #243 L7).  A corner's rail is its
  places in order, one a section; the station's corner stands where its rail meets the station's
  plane.  **Chains run in step**: `flatten::chain_arguments` finds every `path` formal given a
  named open chain, `walk_legs` gives each its own link `k` on leg `k` (refusing chains of unequal
  counts, E080 naming both), and `Held::legs` keeps every link a leg runs, so a contact pinned to
  `s` on any of them reads the leg's own number (`leg_named`).  A guide cut at the sections and a
  chain of lines through each corner is the **ruled** loft, piecewise in `s`; its walls meet at an
  angle at each section, which the exact build makes an edge (`brep.md`, creases).  The **smooth**
  loft's rail is `spline(through: …)` (`curves.md`), one span a link, so with the guide cut the
  same way its sections still stand at `s = k`.  **No guide**: `std.stations.Across(a, b, c, s:)` is
  the plane over the lines from the place at `s` on rail `a` to those on `b` and `c` (Helix's way, a
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
  [0.113]): no Rust names them.  `std.stations.Ruled(a, b, s:)` walks the two faces in step (`cycle
  (ea, eb) in (a, b)`, `language.md`), joins corner `k` of each by a straight rail, stands each
  corner `point in st.n` on its rail with `st := Across(rail[0], rail[1], rail[2], s:)`, and is
  `face(side)` of its copies, so its walls are `body.side[k]` from corner `k` and its ends the
  faces themselves; `std.stations.Apex(a, tip, s:)` the same to a point (no `end` cap).
  Correspondence is the written walks: a connector is where a walk starts, unequal counts a point
  named on an edge; the first three corners must not stand in a line (`Across`).
  `std.stations.Similar` and `std.stations.Affine` are each one statement, the face carried whole
  ([0.118], below): `f := prof mapped(from -> a, to -> b, third -> c) in n`, the profile's corners
  and frame columns of each corner's `mapped`, held where the profile is (`Framed` and `Spans`, two
  `offset`s over hidden spans of the frame, are retired, [0.117]); `lofts/blade.sv` and
  `three_rails.sv` write the word itself.  `tests/correspondence.rs` is the gate: `two_faces.sv` at
  40/6 · 1856, a connector and a split triangle at their prismatoids, a pyramid to an oblique tip, a
  blade on two rails (1560; turning, 1650), a square on three rails at its area ratio's prismatoid,
  a face walked, a face of a cycle's copies, and the refusals.  An arc edge lofted with a line edge
  is not said: it wants a rational edge whose weight reads the arc's sweep and a word choosing by an
  edge's kind.
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
  `std.stations.Minimal(n, up:, s:)` (#243 L9) is the word.  `tests/rates.rs` is the gate: the
  minimal frame along a helix of radius 10 and pitch 5 turning −28.558° a turn against
  `std.stations.OnHelix`'s (Frenet's) to 1e-9, read off the caps (and three quarters of a turn, and
  the home square to the axis); a tube on four turns at A·L to 1e-9 in either frame; along a planar
  spline the word and `std.stations.On` the same solid, corner for corner; the refusals.
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

**A sheet's band is a solid of its own route** (#645, [developables.md](developables.md)).
`region::form` reads a band term first (`Form::Sheet`), before a meridian or general position, and
`Route::Sheet` is the band faceted from its panel's rulings (`solid::sheet::Band::primitive`,
`Caches::sheet`), so volume, area, faces, bounds, glTF and STL read it as any evaluated solid; the
recipe refuses it once (`cad.rs`: its exact ruled walls are not yet written to STEP), and the field
refuses it (`field/document.rs`). Its refusals are the panel's, in the core's words, said at the
solid once (`solid::refusal`); the messages carry no solid name, which `refusal`'s callers add.
