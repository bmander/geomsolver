# Sets and regions

Part of the [contract](../../CLAUDE.md) the code is held to. Read it when you touch
`flatten/sets.rs`, `solid/region.rs`, `solid/faces.rs`, or a set in `std.sv`. An issue number (#n)
or a spec version ([0.n], see the [language changelog](../solvent-changelog.md)) says where a rule
came from.

**Spheres, cones and cylinders are library sets** (`rust/lib/std.sv`, 2026-10-06; sets #101): a
centre or a line and one number (`std.Sphere(c, r)`, `std.Cylinder(about, r)`, `std.Cone(about,
half)`, apex at the line's start; the line is `.about`, since `axis` is an element's word a body
cannot name bare), nothing drawn, no entity kind, kernel or `CKind` of their own.  A point on one
is `p coincident S`, a line touching one `l tangent S` (below); two touching at a point with one
tangent plane is `k1 tangent(at: m) k2` (below, #104; `std.TangentCones` is retired, #265),
`std.Offset(S, d)` is the set `d` outside one (F4, below) and `std.CircleOnSphere(k, s, view)`
stays a component.  An unbound number is the instance's unknown
(`pc.half`, an angle in degrees); `edit::unknown_seeds` writes it back into the call's `hint(…)`
(`InstVal::Hint` carries the span).  A lifted program declares it under a name it can write
(`lift::declarable`: `pc_half`, apart from every other unknown's) and every dimension reading
it says so (`flatten::substitute_with`).  `tests/spatial_surfaces.rs` and `tests/spatial_lang.rs` are the
gates.

**Crown regions (#209, 0.76):** a family's body is split by its point at the instance
(`flatten::sets::instance_part`, called where `comp.set` makes the set): declarations and written
relations reading nothing of the bound point, directly or through a declaration that does, are
walked once as the instance's body, the rest is the set — so `std.ConeThrough(about, q, lean)`
owns `.apex`/`.spine` (the apex held by the angle between `about` and the generator through `q`,
`lean` or its supplement by which way it opens, and the spine from it to `q`'s foot on `about`, so
the cone is held at any size, #214; half `abs(lean)`), one per instance, and a region reading it
shares the `Cone` term's line.
`std.HalfSpace(o, along, d)` (`std.Level` through 0.84, #270) is an ordinate along a line:
`RegionShape::Plane { along: EntRef }` takes a plane (its `n`), a line or an axis
(`RegionShape::normal`). A reach to `{ q | q outside S }`
measures from `S`, sides turned (`Walk::side_of_set`, in `expand_reach`), so `std.Rounded(beyond,
tip, r)` lowers. A region tool's faces are its meridian's steps (`RevolvedSurface::meridian_loops`,
`SweepContacts::read`'s `Region` arm); the cutter reads its `revolve` recipe as any. A region's tip
land is a bound between its land's ends drawn on the tip, the flanks through them.
`tests/crown_regions.rs` is the gate (a crown tooth region cutting a region blank within 10 µm).

**The hypoid's crown is regions** (#214, 0.77): `hypoid.sv`'s cutter (the axis up through C
parallel to the crown's, the flank cones through the pitch points, the four levels), its
`Tooth` and its `Space` (beyond one cone grown, within the other, between two levels,
rounded by `std.Rounded`; the space's outer mate to the reach's cylinder, its inner one turned a
crown pitch), the relief's toppings the same over cones through kinks on the grown flanks
(`kink coincident tooth.inner.surface`). What it needed: `S.surface` as any set use's operand
(`set_use`, `surface_of`), a distance a number comes to nothing is the side or `coincident`
(`expand_reach`), an offset's grow added as numbers so an unknown keeps its dimension, a bound on
measurements reading one unknown (`linear`'s `Aff`, `TermNumber`) and named through its sets
(`Measured::through`), `face_source` finding a region cutter's face under the cutter's own name,
a seed's step in space (`place_of`), `RevolvedSurface::reversed`/`meridian_loops` for the checks.
`hypoid_layout.rs` reads each crown's corners off its meridian (the recorded sections' to 1e-9);
`envelope/paired.rs` reads a cutter's faces off its meridian in its walk (`cutter_faces`).

**Sets (§6.21, 0.49, #101):** `S := { p | BODY }` (`StmtKind::Set`, `syntax::SetLit`; the lexer
reads `{ NAME |` after `:=` as a body, not a group) and `component Name(F) := { p | … }`
(`Component::set`, an empty body: the instance *is* the set).  Making one makes nothing
(`flatten::sets::Site`, `Walk::set_made`).  Uses are expanded once every name is known
(`Walk::expand_sets`, rounds, E003 past `MAX_DEPTH`): `q coincident S` walks the body under the
use's own prefix (`{use scope prefix}#{stmt}.0.`, path + `Instance(stmt)`), the bound name an
alias of `q`; `l tangent S` declares the contact at the bound name (in space, seeded at `l`'s
middle), states it `coincident l`, walks the body, then walks it again with `Scope::twin` set —
relations only, each with `Relation::along` (`syntax::Along { point, toward: AlongBy::Line,
key, made }`, `key` the use's derivative, `made` the prefix its geometry is under; refs resolved
in `rewrite`).  Every relation made carries
`Worded { sets: [(k, name)], span }`, so `describe` reads `l tangent shaft` (`WordUse::sets`).

**Differentiated at elaboration (#104):** `program::relations::constrain` registers one
`model::Dual` per key (`Sketch::add_dual`: the point, `Toward::Line(l)` or `Chart`, `owned` =
`SourceMap::ents_under(made)` but the point) and sets `Constraint::along` to its index, dropping a
row that reads nothing the use moves (`Constraint::differentiable`, entity-level, the one rule
`io::from_json` shares; refuses a kernel with no Taylor form).  The kernel is
`kernels::dual_kernel(inner)` (ids past the curve families, `kernel_id_in`; in `kernel_table`
when any row has one): columns `[x, ẋ, a, b]`, consts `[kid, row consts, mask]` (`Move`: 0 held,
1–3 the line's direction `b − a`, `TANGENT` its own tangent column), residual `J·ẋ` and Jacobian
— the Hessian along `ẋ` by polarisation of the ε² coefficient, then `J` routed by the mask — all
read from `taylor::residual`, exact.  The use's own geometry (its `own_params` and lifts) moves by
**tangent unknowns** `Sketch::mint_tangents` mints in `add` (`Dual::tangent`, `{name}.d`, seeded
0, never saved; `retire_tangents` on `remove`); its intrinsic rows (a lift) are stated as
derivatives too (`twin_intrinsics`, at `add_dual` and at each later intrinsic add) — never the
point's own.  **`S1 tangent(at: m) S2`** (`expand_pair`): each body applied under
`begin_nth` (`#{stmt}.{n}.`), walked `Pass::Made` (declarations only; `Scope::pass` is the walk's `Pass`) then
`Pass::Along` along `Chart(0)` and `Chart(1)` (keys `#{stmt}.t{k}.`, shared by both sets); a
chart's point tangent is `e_a + s·e_c`, two components held (`set_chart`), `c` the world axis
the rows' gradient at the seed runs most along (`Sketch::choose_charts`, after phase 3 and at
`from_json`).  Refused there: a body making geometry (`sets::makes_points`), no `at:`, a claim,
inside a derivative.  `along` and `duals` travel through `graft` (duals first, remapped; one
missing its point or line is dropped with its rows), JSON (`"duals"`, `"along": index`) and
`topology_key`; `cgraph` leaves it to the residual, the witness never jitters it, callouts skip
it, `to_program` drops it (a flat program cannot spell it).  Every static kernel but the drags
has a Taylor form (the spline contacts' exact to every order, `taylor::span_frame`:
the basis about `t₀` a cubic in the jet `τ`, over `Σ wB` by the quotient rule): the spatial `*_rows` are generic over
`kernels::Num` (`Dual<N>` for res/jac, `Jet` for the form, `Kernel::form`, `num_kernel!`), the
hand-written spatial kernels have generic twins held to them by `tests/taylor.rs`.  Refused: a
set named as an entity (E040 in `rewrite`), `coincident` between two sets, parentheses, a
non-point `coincident`, a non-line or claimed `tangent`.  `tests/sets.rs` is the gate.

**Regions (§6.21, §9.6, 0.61, #145 F1):** a set whose body bounds a number is a region
(`sets::BodyShape::region`, over `Written::states_bound`). `q inside S` / `q outside S` are set
uses (`may_use`): `expand_use` rewrites the body in place before the walk (`region_body`) — its
one number (a relation with an `OpArg::Dim`) gains `OpArg::Bound` `Le`/`Ge`, a region's own is
kept or turned round (`Written::turn_bound`), a body of only uses (a conjunction) is applied as
written by `inside` and refused by `outside` (a union) but for one side of another set, turned
round, as are an interval `outside`, several numbers and none (E040). **A side is a bound**
(#192): `Written::lower_side` says `p inside X` as `p coincident(<=) X`, `p outside X` as `(>=)`,
beside a side grown, `p distance(<= d, side: outside|inside) X` (`inside(d)`/`outside(d)` are
refused by the parser naming it and `std.Offset`, #266: one spelling, `op_args`); every
set's body is held so (`sets::lowered`, at `Site::new`) and each use read so, and the
elaborator's readers (`relations::shape`, `side_of_plane`, `face_term`, `ends_term`) take either spelling
through `Written::side` (the bound on the signed distance from the surface, and its level). A tangency to a region is E040, once, in
`expand_set_uses`; the derivative pass drops whatever `states_bound`. `boundable` adds `PointLine3` (a cylinder in space) and `Angle3` (a cone), the latter
read straight off its cosine (`Constraint::reading`); an angle in a view is refused (signed: a
half-plane), but a set's body never reads one (below). `Constraint::reading_range` (`magnitude()` without a `side:`, or `Angle3`'s half
turn) keeps `Bound::aims` — the mirror, then two, four and eight times as far past the edge,
stopping where one clamps; an interval's middle alone — within what the reading can be; `across`
finds the measured point among the operands' own points too (a cone's angle is between two lines)
and nudges a point off a stationary reading (the cone's axis). `tests/set_regions.rs` is the gate.

**A set's body is read in space wherever its point is drawn** (§6.21, 0.65): `apply` marks every
relation a set's application makes `Relation::space` (`Application::of_set`, every set use in
`sets.rs`; no later stamp clears it, so a set inside a word's body keeps it), and `reading::in_space` reads it by `Reader::Set` — in space even with
every operand in one plane where the plane's kind says which way (`Angle`, directed, becomes
`Angle3`; a `side:`/`sense:` refused as across views), the plane's kind kept where it says the same
(drags and `cgraph` keep their edges) — or `Reader::Moved`, a derivative along a chart, always in
space (`Constraint::differentiable` refuses only a row with no kind there). So `std.Cone(ax)` about
a line drawn in a view is both generators; a region term's cone apex is compared by
`Sketch::twinned`; `lift::in_plane_angle` writes such an `Angle3` as the plane's `angle` with the
pose's `sense:`, so a lifted program keeps the root.

**A region is a solid (§6.9, 0.62, #145 F2/F3):** `blank := solid(R)`, R a set, is `SolidDef::Region
{ probe, terms }`. The flattener (`Walk::region_of`) rewrites `solid(R)`'s `Sweep::Body` over a set
to `Sweep::Region { probe }`, declares the probe `{solid}#probe` (in space, held at a fixed
irrational place; `Expansion::probes` → elaboration's `regions::Probes`) and states `probe inside
R`; `constrain` takes every bound landing on a probe as a `RegionTerm` instead of a row
(`regions::probe_of`, `region_term`: by kind — `Distance(3)` a ball, `PointLine(Distance|3)` a cylinder,
`Angle3` from a line's start a cone, an `Ordinate` along a plane's `n` a half-space; numbers by
`term_number`: stated, or affine in one unknown read when evaluated; an angle unknown read only
as unsigned, a cone's half, is kept within a half turn by `expr::sync_free`), and `build_solid`
moves them into the solid and holds the probe. `solid::region::meridian` folds the terms' (r, z)
regions in a box by `brep::planar::boolean` (coaxial, bounded, nonempty, else refused where
evaluated), memoised against `solid::reads` (`Sketch::meridian_cache`); every evaluator reads it —
the recipe a `revolve`
(`recipe::region_profile`, `Profile::to_json`), the field `RevolvedRegion::of_meridian` (also
`read`, so patches trim by it), facets `region::face_polys` turned by `primitive::revolve`,
admission `Turns::Line`/`Ball` and `tool()` — its loops `Region::outer_first`, a ball's
`Region::half_disc`. A set used in a predicate's body is applied under the statement it was
written as there (`Scope::written`, kept when `apply` stamps the use's over it; `begin_nth`), so
two uses in one body stand apart.
`tests/region_solids.rs` and `tests/region_blank.rs` (the blank as a region against today's, both
designs, both members) are the gates.

**A region in general position** (#183): one whose terms share no axis (the meridian refuses it,
`region::readable`/`in_general`) is read point by point by `solid::field::RegionField`, a
`SpatialField` leaf (each term its surface's exact signed distance, the largest; a box's enclosure
the centre's value within its half-diagonal), its support the box of the planes every term lies
within (`polytope_box`, refused unbounded). Evaluated as sweeps are (`sweeps_among`: the field's
surface) unless the lowering table builds it ([rounds.md](rounds.md)) or **the common of its terms** does (#238,
`solid::bounded`: every term a ball, a cylinder, a cone narrower than square or a half-space, no
offset — each a primitive bounded past the support, kept within in common, kept outside taken
away; the recipe's `"region"` node, `"pieces"` each with `"keep"`; gated by agreement with the
field, `Sketch::bounded_cache`); else no exact construction (`cad.rs` says so, naming the region).

**A sum of distances the meridian folds** (#238, `region::foci`): from one point, or two alike in
weight, on its axis (the line through them chooses it when nothing else does) — a ball, an
ellipsoid's half ellipse (two `Seg::Conic` quarters), the plane midway, or a hyperboloid's lobe
(`lobe`: hyperbolic conic arcs, `w = cosh`, a unit of `u` each); held to the field as rounds are
(`agrees`). A conic's `Seg::param` tells a hyperbola's other branch off its arc (both ends'
barycentric coordinates below nothing). `tests/region_general.rs` and `tests/region_exact.rs`
(closed forms to 1e-9) are the gates; `brep_census.rs::exact_census` (ignored) sorts every corpus
object by its route.

**A bound on measurements** (#183 F3): `distance(p, f1) + distance(p, f2) <= 10mm` in a set's
body is `StmtKind::Measured`, **parsed once** (#190, `parser/measured.rs`, where a statement calls
a measurement and compares outside every bracket, `compared_ahead`): each side a
`syntax::Measuring` tree whose `Measurement`s name what they measure by `Ref`s (`at:` a labelled
one), resolved by `rewrite` as every operand is and rewritten structurally by `measured_sets`;
whatever measures nothing is one `Number` text the walk writes the scope's numbers into, read by
`expr` (whose lexer reads no hidden key mid-name: `s#probe` is no name a source writes). A
region (`BodyShape`); `inside` keeps it, `outside` turns it, and a body of bounds only is all
of them under `inside`. Elaborated
by `regions::measured_term` into `RegionShape::Measured` (Σ cᵢ·distance from a point, line,
axis or plane; `linear`: affine only, one probe) on the probe, else E040; read by the field
(`region::measured`, divided by Σ|cᵢ|), never the meridian.

**Faces as sets** (#183 F1, F9): in a set's body a solid and the fields past it are a face
reference (`model::FaceRef { solid, path, surface }`: `pad.plate.near`, `pad.boss` every face
`boss` gave `pad`, `block`, `.surface` its whole surface).

**Face references as resolved operands** (#188, [0.70]): a face's name is a
`solid::faces::FacePath` (route through the operands, `document::operand_primitives`, the walk
`exact::build` names faces by, placed copies included — `Exact::named` keeps each face's two
halves — then the face's own name); `FacePath::matches` (segments from the first, never a
suffix) is the one rule, read by `BodyFaces::faces` and the flattener's `face_links`, and
`face_paths` enumerates the same walk with `SolidE::caps`, the one cap table. The members
`ends`, `surface`, `body` are `syntax::Member`, written as `Seg::Member` by the flattener only
past a set or a solid where no declared name swallows the field (`rewrite`'s `fix` over
`Known::solids`; `sets::measured_sets` for `S.ends`; in a measurement's text the segment
`Member::key`, `#surface`); the elaborator never matches the word. `Resolver::operand` resolves
an operand once into `resolve::Operand` (`Ent`, `Face`, `Ends`, `Body`; `entity` for the
entity-only readers, `operand_named` for text), so `follow` never meets a solid's fields, and
`relations::shape` (`Shape::Face`/`Ends`/`PlaneSide`/`Other`) is the one classifier `constrain`
and the claims about solids ask. `tests/face_names.rs` is the gate.

**One meaning, two representations** ([0.68]): a face path is the face, bounded by its edges,
and `.surface` the surface it lies on, whoever reads it. A region's point reads either here,
off the solid after the solve; a drawn point or plane reads `.surface` as #162 F5's carrier rows
(below), made under a hidden key only its statement is pointed at, so neither ever captures the
other (`tests/region_faces.rs::a_sketch_on_a_face_leaves_a_regions_reading_of_it`,
`tests/rounds_examples.rs::a_sketch_on_a_rounded_face_leaves_the_round`).
`regions::face_term` makes `p inside/outside[(d)] F` a `RegionShape::FaceSide` (signed
distance from the surface, positive away from the material), `p distance(… d) F` and `p coincident
F` a `Measured` over `Target::Face`; `distance(p, F)` in a sum the same; `angle(a, b, at: p)` (the
one labelled measurement, `syntax::Measurement::at`) a `RegionShape::Angle`, the material's angle at
the nearest point of the edges `a` and `b` share, read as a **gate** (`region::GATE`: ±1e30 by
whether it holds; convexity per edge at its middle by `solid_place`). Claims skip a point against a
solid. Read by `solid::faces::BodyFaces` off `brep::recipe::build(cad::recipe(…))` (millimetres,
unshifted; face names matched by `faces`), **the stratum**: a body read by regions that are its
own terms (`readers_of`) is read with them taken out of every body (`without`, a cloned sketch).
A region reading faces may have no support (`RegionField` with `None`): a cut needs none.
`reads` pushes a read solid. `tests/region_faces.rs` is the gate.

**The distance from a set** (#183 F2) and **where an edge ends** (F7): a set a measurement names,
or `p inside S.ends`, gets a hidden point of its own (`sets::measured_sets`: key
`{prefix}#{stmt}.m{k}`, a probe, `inner coincident S` applied under a prefix of its own so two
sets of one statement bind apart); the measurement then names that point, elaborated as
`Target::Set { probe, terms }` (`regions::ends_term`: `RegionShape::Ends`), the terms filled
and the point held when the solid is built (`regions::fill_sets`). Read by
`solid::conditions::SetDistance`: points on two faces' patches are the boundary's shared edges, on
three its vertices, on one its patch; any other conjunction (a face's side, `|side|` bounds, `on
F.surface`, a distance from a point) is the nearest point by sequential projection onto the
conditions linearised (each step a ≤3-active QP, exact). `d(p, E) <= d(p, M)` with `E` on the
patches of the faces `M` is on the surfaces of is read as `d(p, E) − d(p, rest)` (`Rest`: `M`'s
nearest point when off `E`, and where `E`'s run ends), the same points with a negative inside.
`ends` is `max σ·side` over the faces at each end of the run (σ the side it runs on), a face
carrying one of the run's on across a tangent seam none (`BodyFaces::continues`); a closed run
ends nowhere. In a set measured from, `q inside S.ends` is one condition per such face, and `Rest`
keeps an end of `E` only where `M` carries on past it within them: std's `meeting` is cut so, and
a run is carried past its edge's end and cut off by the face it ends on (#195). `edge` and `seam` are element keywords: a set may not be called so.

**Distance from a set, offsets (§6.21, 0.64, #145 F4):** `p distance(d, side: outside|inside) S`
(`distance` is a set use, `may_use`), its bound `p distance(<= d, side: outside) S` and
`std.Offset(S, d)` (the set, where one is passed or used) are one bound on the signed distance from `S`'s surface (`Reach`: cmp toward the outside, whether
the offset grows it, `reach_of` over `Written::lower_side`'s form; only a `distance` with `side:
outside|inside` is a set use, `sets::reaches`), applied by `sets::expand_reach` in three representations: a body whose
one relation is a `distance` gets its number moved (`(r) + #grow`, the grow settled in the use's
scope and given to the application, `Application::give`); otherwise, at a region solid's probe
(`Walk::probes` through the alias table), the bound carries it (`OpArg::Bound::grow`, `Written::grow`,
read by `constrain` into `RegionTerm::grow`, a `TermNumber`; a grow off a probe is E040) and
`solid/region.rs::fold` builds the exact offset (`within(t, v, g)` is `{sd ≤ g}`; `narrow`: a cone
shrunk is its apex moved `-g / sin`, grown is the shifted generator plus an arc about the apex, one
loop; wider than square the opposite cone shrunk); otherwise a drawn point's **foot**, declared at
the body's point under the application (`hidden_point`, as a tangency's contact; `hint(at: p)`),
the body walked `Itself` then along
`Chart(0)`/`Chart(1)` (keys `{prefix}t{k}.`), with `p distance(#reach) foot` (its derivative is the
sphere about `p`'s), `p outside S` (the side) and `foot inside S` (so the seed crossing puts the
foot across the surface from `p`) appended to the body under synthetic ids (`FOOT_STATEMENTS`
down, so `Scope::written` keeps their uses apart). `region_body` counts no bounded side of
another set among a body's numbers (a distance from one is `std.Offset`'s). `tests/offsets.rs` is the gate.

**`P tangent(at: m) S`, a plane tangent to a set (#145, #148, 0.56):** `sets::expand_touch`
applies the body at `m` and walks it `Pass::Made` then `Pass::Along` toward `P.u` and `P.v`
(`model::Toward::Axis`: the axis's `d` columns fed where a line's ends go, `dual_kernel`
unchanged), plus `m coincident P`, dropped in `relations::constrain` where `m` is drawn in `P`.
**No shape in Rust:** where the drawing already says what the body says along `P` (a cone's apex
drawn on it, a cylinder's axis parallel to it), the two derivatives are dependent, and the
diagnosis finds it by rank — a derivative row counts as a relation, and a plane touch's own
derivative rows found removable are `Diagnosis::expected`, out of `implied`/`over`, never
painted. A non-plane or no `at:`, E040; a labelled reference fills its slot
(`Written::assemble`). `tests/plane_tangent.rs` is the gate.

- **A face of a solid is a set** (#162 F5, [0.66], [0.68]): `b0.near`, `b0.far`, a straight side
  `b0.ab` and `body.b0.near` name the face, and `.surface` its carrier — the one a drawn point or
  plane reads, made by the flattener from the solid's own text before the uses are expanded
  (`flatten/faces.rs`, `Walk::face_sets`; a face written in place too) under the hidden key
  `{face}#surface`, the asking operand rewritten to it (`surface_of`); a bare face from a drawn
  use is E040 naming `.surface` (a region's point reads it, #183 above): a cap `{ p | p
  distance(D, along: n) f0#plane }` (`coincident` at 0), a side `{ p | p
  level(M) e.p1 }` over a hidden axis `M` square to the edge, parallel to the plane.  `f0#plane`
  is a name the source cannot write, bound by the elaborator to the plane the face's edges are
  drawn in before the constraints (`planes::face_planes`, `Expansion::face_planes`, which seeds
  `M` too).  Refused at the use (E040): a curved side, a `through:` solid's caps, a non-prism.
  **Sketching on a face** is `v1.origin coincident b0.near.surface` + `v1 tangent(at:
  v1.origin) b0.near.surface` (`at:` takes a path), written by `edit::sketch_on_face` seeded on the numeric twin
  `solid::carrier` where the eye's ray met it (`SolidHit::at`); `edit::face_plane` finds a plane
  already lying there, which the app chooses instead (a double-click on a face, or Solids ▸
  Sketch on face, `k`).  A plane dropped from a plane states `P distance(d) v` and is sized as an
  extrusion is (`LiveExtent` kind `offset`, `workspace::offset_handle`/`offset_at`; previewed by
  the constraint's number and a solve, written once by `set_dimension`).  `tests/face_sets.rs`
  and `tests/edit_datums.rs` are the gates.
