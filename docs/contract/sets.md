# Sets and regions

Part of the [contract](../../CLAUDE.md) the code is held to. Read it when you touch
`flatten/sets.rs`, `solid/region.rs`, `solid/faces.rs`, a set in `std.sets`, or a drawn face read
as a set (`RegionShape::Section`, an `over` in a relation's parentheses). An issue number (#n)
or a spec version ([0.n], see the [language changelog](../solvent-changelog.md)) says where a rule
came from.

**Spheres, cones and cylinders are library sets** (`rust/lib/std/sets.sv`, 2026-10-06; sets #101): a
centre or a line and one number (`std.sets.Sphere(c, r)`, `std.sets.Cylinder(about, r)`,
`std.sets.Cone(about, half)`, apex at the line's start; the line is `.about`, since `axis` is an
element's word a body cannot name bare), nothing drawn, no entity kind, kernel or `CKind` of their
own.  A point on one is `p coincident S`, a line touching one `l tangent S` (below); two touching at
a point with one tangent plane is `k1 tangent(at: m) k2` (below, #104; `std.TangentCones` is
retired, #265), a set grown by `d` is `{ q | q distance(<= d) S }` (F4, below; `std.sets.Offset`
retired, [0.205]) and `k
on_sphere(view: v) s` is a relation word (a component until #562, which held a circle it was given).
An unbound number is the instance's unknown (`pc.half`, an angle in degrees); `edit::unknown_seeds`
writes it back into the call's `hint(…)` (`InstVal::Hint` carries the span).  A lifted program
declares it under a name it can write (`lift::declarable`: `pc_half`, apart from every other
unknown's) and every dimension reading it says so (`flatten::substitute_with`).
`tests/spatial_surfaces.rs` and `tests/spatial_lang.rs` are the gates.

**The notation of sets** (#783, [0.205]): the parser lowers it, token by token, to the forms the
rest of this file reads (`syntax/notation.rs`, run on every text `parse_at` reads, a module's
included): `p in S` as `p inside S`, `p not in S` as `p outside S`, `p in a.side` as `p inside a`
(a face's material side), `p in a` with `a` a face — a formal `a: face` of the definition the
statement stands in, or one drawn `a := face(…)` — as `p coincident a`, `boundary(S)` anywhere as
`S.surface` (`a.side`'s `a.surface`, `a.smooth`'s itself), `p in boundary(S)` as `p coincident
S.surface`, `p in A or p not in B` in a set's body as `p outside { p | p outside A; p inside B }`,
`some t [in (a, b)]: p in X at(under: m, seen_from: g) t` as `p inside(under: m, seen_from: g[,
over t in (a, b)]) X` (`over t` where `t` names a declared `param`, none for the whole roll;
`coincident` where `X` is a face, or a family's member read at a declared parameter, `some s: p in
F(…).f at s`), `some h: p in F at(along: n) h` as `p inside F` and `not some …` as `p outside F`,
a distance from `{ q | q not in S }` as one with `side: inside` from `S`, and `n := x at f`, `f`
declared `:= frame` or a `frame` formal, as `n := x in f`; `when w not in A` is the condition
reader's own (`expr::parser`). A lowered token carries the span of the word it stands for, so
every diagnosis lands on the source, and the relation read from a lowered word is marked
(`Written::lowered`, `Lowered::Set` or, from `.side`, `Lowered::Side`; the parser's `lowered_word`
matches the word's start, its span widened over its parentheses). The retired spellings are
refused there, naming their replacements: `inside` and `outside` as a relation's word (but `claim
a inside b` between solids), `.surface`, `over` in `coincident`'s parentheses, `n := x in f`, and
in the condition reader `not w in A`. **What needs the operand's kind** is read once it is known.
In the flattener, before a round of set uses (`sets::notation`): a lowered `in` against a set whose
body bounds no number is `coincident` (no number is read as a bound unasked — the one-number rule
of **Regions** below stays for the internal `inside` a probe or a foot is applied with), a lowered
`not in` against one is **E040**, a `distance` with no `side:` against a region is the ordinary
distance (`side: outside`, marked lowered) and against a set with no inside **E040**, and a
`coincident` written against a set, its first operand no hidden key, is **E040** naming `in`;
`expand_reach` refuses `side:` written on a distance from a set that is no recognised sheet. In
the elaborator (`program/relations.rs::constrain`): a lowered `in` against one face of a solid (a
`Shape::Face` with a path, no `.surface`, no `.smooth`) is `coincident`, on the face, and `not in`
it **E040**; a lowered `in` against a plane is **E040** naming `coincident` and the plane's side as
an ordinate. An envelope's or a meshing's operand may be `boundary(S)` (`flatten::expand`'s
`set_operand`, the probe applied to `S.surface`). The printer prints the notation back
(`syntax::print::notation_text`, and `write_ref` a surface as `boundary(…)`), so a statement the
app writes parses to what it read; a sketch on a face writes `in boundary(F.side)`.
`tests/notation.rs` is the gate.

**A set is an envelope's operand** (#283, [0.90]): `flank := envelope(S, under: m, …)` over a set
is applied to a hidden point keyed `{flank}#probe` (`Walk::envelope_of`, beside `region_of`, both
through `set_operand`), `probe coincident S` stated with it; the planar pass skips it, and
`program::envelopes` reads what the body states at the probe as the surface (a cone, a cylinder) —
never a row (`analytic.md`).

**Crown regions (#209, 0.76):** a family's body is split by its point at the instance
(`flatten::sets::instance_part`, called where `comp.set` makes the set): declarations and written
relations reading nothing of the bound point, directly or through a declaration that does, are
walked once as the instance's body, the rest is the set — so `std.sets.ConeThrough(about, q, lean)`
owns `.apex`/`.spine` (the apex held by the angle between `about` and the generator through `q`,
`lean` or its supplement by which way it opens, and the spine from it to `q`'s foot on `about`, so
the cone is held at any size, #214; half `abs(lean)`), one per instance, and a region reading it
shares the `Cone` term's line.
`std.sets.HalfSpace(o, along, d)` (`std.Level` through 0.84, #270) is an ordinate along a line:
`RegionShape::Plane { along: EntRef }` takes a plane (its `n`), a line or an axis
(`RegionShape::normal`). A reach to `{ q | q outside S }`
measures from `S`, sides turned (`Walk::side_of_set`, in `expand_reach`), so
`std.rounds.Rounded(beyond, tip, r)` lowers. A region tool's faces are its meridian's steps
(`RevolvedSurface::meridian_loops`, `SweepContacts::read`'s `Region` arm); the cutter reads its
`revolve` recipe as any. A region's tip land is a bound between its land's ends drawn on the tip,
the flanks through them. `tests/crown_regions.rs` is the gate (a crown tooth region cutting a region
blank within 10 µm).

**The hypoid's crown is regions** (#214, 0.77): `hypoid.sv`'s cutter (the axis up through C
parallel to the crown's, the flank cones through the pitch points, the four levels), its
`Tooth` and its `Space` (beyond one cone grown, within the other, between two levels,
rounded by `std.rounds.Rounded`; the space's outer mate to the reach's cylinder, its inner one
turned a crown pitch), the relief's toppings the same over cones through kinks on the grown flanks
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
`begin_nth` (`#{stmt}.{n}.`), walked `Pass::Made` (declarations only; `PredWalk::pass` is the walk's `Pass`) then
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
non-point `coincident`, a non-line or claimed `tangent`.  **A plane's membership a body states of
what is drawn in the plane already** (or twinned there) states no row (`relations::drawn_on`,
beside a plane touch's point, [0.95]): the body is true of that point, where the same words said
directly are E061.  An instance of a family given no entity for an entity formal is E103 at the
call (`Walk::bind`, as a set or group formal left out), never silently nothing — but a point
formal drawn in a plane formal (`center: point in on`), which the instance owns
([language.md](language.md)), so a family's parameters are its formals, given or not.
`tests/sets.rs` is the gate.

**Regions (§6.21, §9.6, 0.61, #145 F1):** a set whose body bounds a number is a region
(`sets::BodyShape::region`, over `Written::states_bound`). `q inside S` / `q outside S` are set
uses (`may_use`): `expand_use` rewrites the body in place before the walk (`region_body`) — its
one number (a relation with an `OpArg::Dim`) gains `OpArg::Bound` `Le`/`Ge`, a region's own is
kept or turned round (`Written::turn_bound`), a body of only uses (a conjunction) is applied as
written by `inside` and refused by `outside` (a union) but for one side of another set, turned
round, as are an interval `outside`, several numbers and none (E040). **A side is a bound**
(#192): `Written::lower_side` says `p inside X` as `p coincident(<=) X`, `p outside X` as `(>=)`,
beside a side grown, `p distance(<= d, side: outside|inside) X` (`inside(d)`/`outside(d)` are
refused by the parser naming the distance and the set grown, #266: one spelling, `op_args`); every
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
space (`Constraint::differentiable` refuses only a row with no kind there). So `std.sets.Cone(ax)`
about a line drawn in a view is both generators; a region term's cone apex is compared by
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
evaluated; coaxial to the fold's own tolerance, 1e-7 of the region's size, a plane's tilt too —
two lines a solve holds on one axis hold to its acceptance, not to the bit, #283), memoised
against `solid::reads` (`Caches::meridian`); every evaluator reads it —
the recipe a `revolve`
(`recipe::region_profile`, `Profile::section`), the field `RevolvedRegion::of_meridian` (also
`read`, so patches trim by it), facets `region::face_polys` turned by `primitive::revolve`,
admission `Turns::Line`/`Ball` and `tool()` — its loops `Region::outer_first`, a ball's
`Region::half_disc`. A set used in a predicate's body is applied under the statement it was
written as there (`PredWalk::written`, kept when `apply` stamps the use's over it; `begin_nth`), so
two uses in one body stand apart.
`tests/region_solids.rs` and `tests/region_blank.rs` (the blank as a region against today's, both
designs, both members) are the gates.

**A region in general position** (#183): one whose terms share no axis (the meridian refuses it,
`region::readable`/`in_general`) is read point by point by `solid::field::RegionField`, a
`SpatialField` leaf (each term its surface's exact signed distance, the largest; a box's enclosure
the centre's value within its half-diagonal), its support the box of the planes every term lies
within (`polytope_box`, refused unbounded). Evaluated as sweeps are (`sweeps_among` over `region::route`: the field's
surface) unless the lowering table builds it ([rounds.md](rounds.md)) or **the common of its terms** does (#238,
`solid::bounded`: every term a ball, a cylinder, a cone narrower than square or a half-space, no
offset — each a primitive bounded past the support, kept within in common, kept outside taken
away; the recipe's `"region"` node, `"pieces"` each with `"keep"`; gated by agreement with the
field, `Caches::bounded`); else no exact construction (`cad.rs` says so, naming the region).

**A face deleted, said as a set** (`p in B.near.side; p in B.rim.side; …`, the faces round a
deleted face run on along their own surfaces until they meet): where the lowering table builds
none of it, the common of its terms builds it too (`solid::bounded`, after the table and a body
grown in `region::route`).  A face's side at a number is the solid its surface bounds grown by it
(`faces::BodyFaces::level`, a `faces::Level`): a plane's half-space, a cylinder, a ball, a cone
(its apex where the radius is nothing) or a torus (its tube short of its axis), kept within where
the material is within the surface, taken away where it is outside (a hole's wall, a
countersink); several faces on one surface as that surface, on several whose creases are all
convex as each (`Reading::levels`, at the surfaces only).  A term outside a set — a union, `p in a
or p in b`, the set of each turned — takes away that set's own region built the same way
(`bounded::gather`, a `"region"` node as a piece), from a block holding the support where it keeps
within nothing; a region of nothing but such terms is that block less them.  Its support is the
field's, which a face kept within a ball, a torus, a cylinder, a cone or its convex planes closes
as the shapes' own do (`region::general`), else the box of the bodies it reads grown by its own
diagonal — so a region reaching past that is cut off there and the gate refuses it.
`tests/face_edits.rs` (brep-bench's `edit/` cases, to closed forms) is the gate.

**A sum of distances the meridian folds** (#238, `region::meridian::foci`): from one point, or two alike in
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
(`region::general::measured`, divided by Σ|cᵢ|), never the meridian.

**A drawn face is a set operand** (#737 P1, [0.199]): at a region solid's point, `p inside sec`
/ `p outside sec`, `sec` a drawn `face(…)` (or one written in place as a family's argument), is
`RegionShape::Section { face }` (`model/region.rs`), elaborated by `program::regions::section_term`
through `relations::Shape::Section`: the region of the face's plane within its loops, read at the
point's foot on that plane, so off the plane the prism over it, unbounded along the normal; `p
distance(<= d, side: outside) sec` grows it in its plane (the term's `grow`).  `p distance(>= a,
along: n) sec` is the point's ordinate along the face's normal from the plane it lies in: written
along the face, and made the plane's ordinate by `program/solids/build.rs::face_planes` once the
faces are built.  The field reads a section by `solid::field::section` (the face's `PlanarField`
in its plane's basis), a `Signed` leaf in `solid/region/general.rs`; a distance from `{ q | q
inside sec }` or `{ q | q outside sec }` is `solid::conditions::SetDistance::Section`, the distance
in the plane from the point's foot to the region (what `std.solids.Drafted`'s lean reads).  A
section is not a solid's face (`FaceRef`, below): it is the region the face bounds, never a side of
material.  A drawn point against a face is E040 — a face is read only at a region solid's point.
`tests/solids_as_sets.rs` (`a_face_is_a_set_where_no_sweep_says_it`) is the gate.

**A draft's lean about a whole circle is two lines of a meridian** (#765, [0.201]): the lean
`a (distance(p, beyond) − distance(p, within)) + c · distance(p, P)` (`defined::lean`) over a face
that is one circle of radius `r` about `o` and no hole, its own plane `P`, reads `a (r − ρ) + c |h|`
— `ρ` off the line through `o` along `P`'s normal, `h` along it — so a region whose terms are such
a lean and half-spaces square to that line (`std.solids.Countersink`'s set, not a prism for want of
a second ordinate) is folded by the meridian (`meridian::leaning`, `Fold::lean_within`: all but a
diamond where it leans in, the outside of two cones where it leans out, each clipped convex,
`clipped`), its axis the circle's (`axis_of`), and every evaluator reads it as any meridian — the
recipe a `revolve` of its triangle, the field and the facets its region turned.  Its faces are named
as a prism's, on the terms themselves before any reader sees them (`defined::name_terms`, called
by `program/solids/build.rs::region` after `face_planes`, for any region no entry recognises): a
section's or a lean's term by its face's edge where the face is one edge and no hole, an ordinate
along that face's plane `near` bounding from above and `far` from below — so its paths
(`face_paths`, `repeat a in S`), the field and the exact solid agree.  A lean grown, about a circle not square to
the axis, or about any other face is no meridian's and stays the field's (`Route::Field`, its exact
export refused).  `tests/countersink.rs` is the gate.

**There is a roll at which** (#737 P4, [0.199]; written `some t in (a, b): p in X at(under: m) t`
since [0.205], and lowered to the forms below — `syntax/notation.rs`): `over t in (a, b)` (the
interval written in place) or `over s` (a declared `param`) in a relation's parentheses is
`syntax::OpArg::Over`,
parsed by `parser/relations.rs::op_args` for `coincident` and `inside` and settled by the
flattener like any dimension.  `p coincident(under: m, over t in (a, b)) sec` at a region solid's
point is what the face occupies under the motion at some roll of the interval:
`relations::shape` names it `Shape::Swept` and `face_or_ends` reads it with every other region
term — `relations::swept_term` makes it `RegionShape::Swept { motion, roll, face }`, `m` a frame or
a line it turns about (`motions::reading`, [0.200], frames.md).  **A face carried** (#846,
[0.208], `solid/carried.rs`): where `m` is a constant twist (`motion::Family::twist`: a turn, a
screw or a slide), the region whose one term that is is the face's profile where the motion stands
it at `a` carried to `b` — `region::route` gives it `Route::Carried` ahead of every other entry,
the recipe's `carried` node (`brep::build::carried`: a screw's `screwed`, a turn's `revolve`, a
slide's `ruled`), held to the region's field by the agreement gate before it stands
(`carried::built`, remembered in `Caches::carried`, as `bounded`'s is).  The field reads a screw's
or a slide's point by point (`carried::Reading`, from `region/general.rs`): a level crossing —
a slide's face crossed by its direction, a screw's square to its axis — is passed once by each
point's path, so one roll carries the point back to the face's plane; an axial one — a screw's face
in a plane holding its axis, on one side of it — once a turn into the face's half-plane, the least
over the rolls a turn apart that could carry it back within the heights the face spans.  A turn's
is read about a line in the face's plane as before.  Refused, by `Carried::checked` and so by the
field and the route alike, with its reason: a screw's face leaning on its axis, an axial face that
reaches the axis or — carried more than a turn — spans as much as the screw rises a turn (the sweep
meets itself), a slide along the face's plane, a turn about a line off it; a motion that is no
constant twist (several steps, a law) is refused by the field.  It names no running parameter, so a
component's body writes it as the document's does, where a face family cannot stand.  Its faces are
named off the definition alone (`carried::face_names`, read by `face_paths`): the section's sides
by their edges, then `start` and `end` (none for a turn through a whole one), as the kernel names
the build.  `tests/screwed_sets.rs` is the gate.  Where such a set is written in place in `solid(…)` and its body is one
statement saying a sweep, the parser reads it as the sweep of the same points
(`syntax/parser/statements.rs::swept_ahead`/`swept_set`): `solid({ p | p inside(under: m,
seen_from: g, over t in (a, b)) tool })` the motion sweep (`SolidDef::Swept`, a stated roll), `over
t` a declared parameter's, and no `over` at all the whole roll, `Sweep::Rolled`, naming no body
(#746) — bounded by every body the solid, or a copy of it placed at a roll (`tool at(under: m)
θ`), is cut from or kept in common with (`solid::uses`); cut from none it is E080 at its
declaration; `solid({ p | p coincident(over s) Station(…).f })` a face family and `solid({ p | p
coincident(over s) sec })` a face of the drawing over a declared parameter.  The developables'
`coincident(t in (0, 1))` (a contact's place on a curve, [developables.md](developables.md)) is
another clause, unchanged.  `tests/solids_as_sets.rs` is the gate.

**Some points of sets** ([0.212]; written `some q in A, r in B: p in X` in a set's body): the
points `p` that some point of each set puts in `X` — a projection, each bound point ranging over
its set as a sweep's roll ranges over its interval.  The notation lowers each binder's `in` as a
membership's (`inside`, or `coincident` with a face or a boundary) and the body as a statement
(`syntax::notation::points`); the parser (`P::points`) declares each binder a private construction
point of the body with its membership of its set beside it, both held in the text of the relation
they bind (`Chained::InPlace`), which carries them (`OpArg::Some`) so the printer writes the
quantifier back; the quantifier runs on past a line break after a binder's `,` or its `:`, as a list
in parentheses does (the notation drops the breaks).  The flattener registers each binder, in every round before any statement is
read (`sets::register_binders`: a binder's membership stands before the relation binding it), as
a probe with its quantifier and the set's point its relation reads (`Expansion::binders`; where
sets nest, another quantifier's binder) — a named instance of a component binding any keeps its
body whole as the set (`instance_part`), since the binders and their memberships, which read
nothing of the set's point, are each use's — and applies the binders' memberships and the
relation **held** (`binding`), as a set measured from is (#192); a `some` whose set's point is no
probe — a drawn point's, outside a solid's set — is E040.  At a binder a point on a line with no
number is a term at nothing (`PointOnLine3`, a segment's own), since the set a binder ranges over
may be no solid's.  The elaborator routes each constraint reading a binder (`regions::bound_by`):
one reading a binder alone is that binder's term, as any probe's; one reading a quantifier's set's
point and nothing else but what it binds is the quantifier's, never a row (so a join nested in
another's set is read as its own); any other is E040.  Once every relation is stated (`settle_bound`) each
quantifier's constraints are read by their kinds, never a word's name: a point on the line through
two points, its ordinate along it from the first at least nothing and from the second at most
nothing — the segment between them — each a binder or a point of the drawing, is **the join** of
what they range over (`RegionShape::Join`, its operands `Target::Set` for a binder, filled and
held as a set measured from is, `fill_sets`); a distance from one binder at most a number is the
distance from its set (`RegionShape::Measured`), the set grown; anything else is E040 saying so,
once: the solid it was refused in is not refused again for want of a term (`Probes::refused`).
A join is read where each operand is the hull of finitely many points (`solid::joined`: a point of
the drawing, a set of one point, a segment — on a line, its ordinate along it bounded at both ends
— a drawn polygon of lines with no hole whose corners turn one way, a solid every face of which is
a plane and every edge convex (its corners), or another join), the hull of
them all by the kernel's quickhull (`brep::build::hull`): its field the most any of the hull's
planes stands below the point — the signed distance within, a lower bound without, one-Lipschitz
— each plane a support plane of the region; anything else is refused by name, for the hull of its
corners would be another set.  The `Bounded` route builds a join exactly as that hull, kept within
or taken away at nothing (the recipe's `hull` node, its faces named by the term's set), gated by
the field as every route is; grown or shrunk by a number it is the field's.
Within a union said with `or` (the outside of a set of outsides), a join's set — several
statements, so outside it is a complement — is bounded, turned round, by its own terms
(`RegionTerm::turned`, a term's complement as terms taken together).  `tests/joins.rs` is the gate: a tetrahedron the join of two opposite edges (its volume, area and
field, and grown by 1 to brep-bench's offset/tetrahedron-grown), brep-bench's square-to-octagon
transition a join of joins to its closed forms, a pyramid the join of its apex and a drawn face,
a capsule a ball about some point of a segment, the join as a component making an octahedron, a
frustum and an oblique prism, a house the join of a box and a point (an L refused), a box rounded
by a ball about some point of it, two joins united and said with `or`, the bench's grown tetrahedron
as a ball about some point of the join, the case library's documents of it (`rust/examples/
joins.sv`, `tetrahedron_grown.sv`, `house.sv`, `pyramid.sv`) to their closed forms, the pyramid
re-dimensioned, and each refusal.  The experiment the syntax
waited for — the cases before and after, what a join's field costs against its half-spaces', and
what is left (a relation read by searching every bound point; joins of curved sets, by support
functions) — is [some-points-of-sets.md](../some-points-of-sets.md).

**Faces as sets** (#183 F1, F9): in a set's body a solid and the fields past it are a face
reference (`model::FaceRef { solid, path, surface }`: `pad.plate.near`, `pad.boss` every face
`boss` gave `pad`, `block`, `.surface` its whole surface).

**Face references as resolved operands** (#188, [0.70]): a face's name is a
`solid::faces::FacePath` (route through the operands, `document::operand_primitives`, the walk
`exact::build` names faces by, placed copies included — `Exact::named` keeps each face's two
halves — then the face's own name); `FacePath::matches` (segments from the first, never a
suffix; [0.157] a step written without an index naming a copy's, `drill` for `drill[1]`,
`same_step`) is the one rule, read by `BodyFaces::faces` and the flattener's `face_links`, and
`face_paths` enumerates the same walk with `SolidE::caps`, the one cap table. **An operand's step
is its path as the body's scope writes it** ([0.203], #760; `document::step_through`, over two
written names `step_between`, the one reading — `operand_primitives`, `past_step`, the layers'
labels, the recipe's per-body `steps` the kernel names faces by (a node has no step of its own),
and the flattener's carrier of a face set, `flatten::faces`): the operand's `SolidE::written`
name (its `public_path`, each block copy on the way an index on the declaration it holds,
`#7.1.joint.pad` → `joint[1].pad`, a copy in a copy the inner's)
past the body's scope, whole, so `hub.body` and `spine.body` unioned into `part` are
`part.hub.body` and `part.spine.body`; from its first hidden segment on (a use's `#17`, `rim#17`)
it is `route_step`'s, labels and leaf (`rim.taken`), and an operand outside the scope is
`route_step` of its name. The members
`ends`, `surface`, `body` are `syntax::Member`, written as `Seg::Member` by the flattener only
past a set or a solid where no declared name swallows the field (`rewrite`'s `fix` over
`Known::solids`; `sets::measured_sets` for `S.ends`; in a measurement's text the segment
`Member::key`, `#surface`); the elaborator never matches the word. `Resolver::operand` resolves
an operand once into `resolve::Operand` (`Ent`, `Face`, `Ends`, `Body`; `entity` for the
entity-only readers, `operand_named` for text), so `follow` never meets a solid's fields, and
`relations::shape` (`Shape::Face`/`Ends`/`PlaneSide`/`Other`) is the one classifier `constrain`
and the claims about solids ask. **A copy the document wrote is a step with its index**
([0.157]): `SolidE::copy`, set where the program builds a solid the document wrote in a block's
body itself (`copy_written`: a `DeclName::Copy` whose statement's last path step is the copy; a
word's solids are `DeclName::Key`, and an instance's in a copy end on the instance), makes its step `drill[1]` (`SolidE::step`, read by `operand_primitives`, `past_step`
and the recipe's `step`), and a face's edges are named from the scope it is declared in
(`boundary_name`), so an edge of its own copy is `hole`, not `hole[1]`. Where `p[k]` picks no
copy of a declaration, `Lookup::face_of_copy` reads the index into its segment (`flange.drill[1]`)
and keeps the reading only where it reaches a solid (`Walk::solid_names`), so a word's argument
and a block over faces read it as written. `tests/face_names.rs` is the gate.

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
conditions linearised (each step a ≤3-active QP, exact). Within faces and at least their one
surface's radius off it (`q inside F` with `q distance(>= d) F.surface`, a round's ball centres
where the ball is as wide as a cylinder: `rounds/dome.sv`) is what the surface's offset closes
onto (`Surface::offset`'s `Offset`), held as conditions with a gradient everywhere on it
(`conditions::closed_onto`): two planes through an axis, three through a centre, a plane and a
cylinder about a torus's middle circle — the side's own turns over on the axis, and the steps
toward it would never settle (the axis alone is exercised: no closed loop rounds with its
centres on a sphere's centre or a torus's middle circle). **Within or outside faces whose side is
convex toward the point, and at least a distance off them** (#962, `BodyFaces::bulging`): one
face a plane, or a cylinder, a sphere or a cone with that side inside it — read by its whole
surface's side, that convex solid's (a round pocket from its air, a round boss from its
material); or a smooth chain — `F.smooth` gone on across a seam — every face of which is so and
every edge between them a seam or a crease convex from that side, and each face's patch within
every other's convex solid — one sheet (#994: a block's side flush with a deck's whose corner arc
runs tangent into it is no sheet, the arc carried down through the block by its tangent planes
where the block's side stands flat) — each face its own condition,
its side carried on past its patch by its tangent planes at its seams
(`BodyFaces::carried_side_grad`, the signed distance from the convex solid the patch and those
planes bound), so the side is their common part and a point at least `d` inside it is at least `d`
inside each — a pocket's walls seen from its air, a boss's from its material. **And every face of
the chain within each other's side carried on** (#996's fifth item), read over each edge of the
chain's faces once, against each face of the chain but those it bounds: a plane's side in closed
form over a line or a circle (as `BodyFaces::goes_across` reads it), any other face's carried side
at nine points of the edge, past by more than the body's own `past` (ten times its tolerance):
convex at every seam and crease between its faces, a chain may still turn the other way along a
face's own outline — a top in two flush pieces, the flange's rim rounded and the block's side
meeting the flange's at a reflex corner of the top — where one face's carried side cuts off
another's patch and no convex solid is theirs: it bulges no way, and is read through the nearest
face (`rim_rounds::a_run_beside_a_chain_turning_back_along_its_outline_is_built`). This is the one
sheet above, one reading of it (#994's and #996's, `BodyFaces::bulging`). **Past any edge but a
seam a face's side is its own surface's** (#994): `F.smooth` is `F`'s side carried on across seams
only, so past a crease or where another face cuts the face across, its whole surface's side goes on
— an arm's rounded end half inside a hub, its cylinder carried past the hub's face by the plane
tangent there, read the centres of a ball as wide as the end off its axis, the round a few tenths
short of the ball beside the face (`tests/standing_ends.rs`). `BodyFaces::carried_past` carries the
tangent plane past a seam only, a seam as near as the nearest edge (a patch's corner where a seam
and a crease meet) still the one carried past. **Two faces lie on one surface where their surfaces
are one and their outward normals one** (#994, `BodyFaces::same_surface`: the kernel's
`fillet::one_surface`, `ssi::same` at the boundary's tolerance, read once a body, `surface_of`) —
one rule for the field's pieces of a surface (`BodyFaces::pieces`, `lowering::pieces`), a face going
on along a piece of a run's own (`carries_on`), the plane's pieces below, the surfaces a reading
lies on (`Reading::surfaces`), a face the ball stands on (`piece::held`) and the sides the kernel
rolls a ball on (`fillet`'s `meeting`); five readings of it (normals to `1e-9`, a sample of five
points an edge, `Surf::same`, `reversed` alike) are gone.  **Pieces of one surface are its convex
solid** (#994, `BodyFaces::bulging`): a face left in pieces by an operand crossing it (an arm's
bottom either side of a hub) has no seam between its pieces, but is one surface's side — a plane's
half-space, a cylinder's, a cone's or a sphere's solid where its face bulges that way (a torus's
whole side is none) — read as no convex side, a projection onto it and a cylinder's offset as thin
as a ball standing still alternated and failed, and the field read no centre there.  **Faces of the
chain on one plane are one condition** (#994, `BodyFaces::one_per_plane`): a plane's side carried on
past its patch is its own, whichever of its faces says it (a curved surface's carried past a piece
by its tangent planes is that piece's own, so its pieces each say theirs), so a plate's top and the
caps the fills of its pockets' uprights leave on it, a chain of one face and four for each pocket,
is one condition, not one a
piece — each projection's active rows are chosen among every three (`project`), so a chain
growing with the body made each reading of a round on that top cost the cube of the body's
pockets (a plate of nine pockets rounded all over, the pockets' own edges first, built in a
twentieth of the time, its volume the same to every digit); the rounds' gates are unchanged
(`tests/tight_corners.rs`, `round_lowering`, `round_onto_round`). A torus's patch is so in a chain
where it lies on the outer half of its tube with that side inside the tube, both its curvatures'
centres there (a pocket's floor filled round its plan corners, seen from the air;
`BodyFaces::outer_half`), never alone (its whole surface's side winds round the axis): a ball
standing on a run's axis beside those stretches was read through the nearest face and the gate
refused it (`round_onto_round::a_boss_across_a_trays_filled_wall_is_its_closed_form`).
Read through the nearest face alone, a corner of the offset (an arc's offset closed onto its axis, a
pocket's corner rounded to its own radius, or past it) is where the nearest face turns, and the
steps never settled there. **A smooth chain turning both ways is read through its nearest face and
face by face besides** (#994, `conditions::faces_apart`, `BodyFaces::carried_past`): a pocket's
walls with a reflex corner (an L, six walls, a chevron) seen from its air, a block's outline with a
tab, its every upright rounded, seen from its material — sides, convex rounds and the reflex
corners' fills in one chain bulging neither way, so no face's side carried past its patch is the
chain's (a block's side carried on past a tab's fill runs through the tab). Besides the nearest
face's side, each face of the chain is a condition: its side where the point's foot lies on its
patch (the patch's distance); past a seam into a face bulging toward the point
(`BodyFaces::bulges`) where that face is nearer than the distance, the plane tangent to that next
face at the seam (`BodyFaces::tangent_side`) — the chain read as a convex one about the corner. The
next face's plane, not this one's: they are one where the seam is tangent, but at a seam tangent
only to the drawing's numbers (a turn of 1e-7 rad, `rim_seams`) this one's plane, carried 13 mm
along the next face, read a ball's centre on that face's offset 1.35e-6 under the distance once
the face's own distance rounded a hair under it, past `nearest`'s bar: no centre, material read
where the construction has none ("leaves out what the field reads beside its edge"); nothing
past a seam into any other face (a reflex corner's arc or fill, whose
tangent plane runs on through the pocket or the tab beyond it), which is nearer and says it; past
any other edge — a crease, a face cutting it across — its own surface's side (`carried_past`
carries a tangent plane past a seam only, below) only where the patch is nearer than the distance,
and nothing where not. A face whose box in space (`BodyFaces::patch_gap`), and every face it goes on
into across a seam, is farther than twice the distance from the point says nothing without reading
its patch (#994): where the set's point is, a face holding it back is within the distance, so the
set is the same, and a chain grown with a body (a plate's top and its pockets' caps) is read only
near the point. Each is under the distance only where the point is that near a patch of the chain,
and every patch is of the chain's surface, so each says only what the distance from that surface
says — however the nearest face is read, past the chain's rim too (`BodyFaces::nearest_face`, its
nearest patch found among those whose box in space comes as near, `nearest_patch`, so a chain that
grows with a body's pockets is measured where the point is, #994; a path's faces, caps taken in,
read once a body, `BodyFaces::faces`) — and the points kept are the same; a chain that is no one
sheet (`bulging`, a block's side flush with a deck's) is read this way as well; what the conditions
add is every wall meeting at a corner of the offset in each step, where the closing axis (a floor or
a rim rounded to a convex corner's own radius) is met as a corner of conditions, as in a convex
chain. A condition holding with no gradient is no row of the step's program (`nearest`), so a step
chooses among the faces near the point, not the whole chain. **Not yet one rule** (#994, the
second review's altitude 9): how far is "near" is `faces_apart`'s `SEEN`, twice the distance on the
face's box, a reach and no bound the set states. The set needs only the faces within the distance,
and every reading past the box test says nothing past it but one: a face whose foot lies on its
patch says its own distance, a holding row, however far. Read as nothing past the distance there
too, and the box test at the distance (one rule, the set's), four `tests/round_onto_round.rs`
cases are refused by the gate ("`filled[0]` … leaves out what the field reads beside its edge",
the boss across a filled edge or a tray's filled wall): the step lands past a face whose row it no
longer sees. The box test at the distance alone, the foot's row kept, passes `round_onto_round`
(the rest of the suite not run), but is no rule either — which faces keep a row then depends on
their boxes' shapes. What a step needs is the faces within the distance of where it can land, a
reach the projection would have to bound from its own step; until then the reach stays `SEEN`.
Read about the nearest face alone —
that face and its bulging neighbours, slots changing with the face — a 56° corner as tight as the
ball was still refused by the gate (`tests/tight_corners.rs`, six walls): which walls a step sees
there hangs on which face the step stands nearest. A chain that bulges one way is read whole, as
above. `tests/tight_corners.rs`, `tests/rim_over_rounds.rs` and `tests/rim_seams.rs` are the gate.
**The steps stop at the first point keeping every condition where every condition is convex**
(`Cond::convex`: a plane's side, a face's carried side held toward its solid): each step's
linearisation then holds the whole set, so a point of it is the nearest; their own fixed point is
not steady where a surface curves tightly about a point far off — a ball's centres a hair short of
closing onto a corner's axis, seen from the corner, moved along it by the distance over the radius
times any error in where the last step stood. A QP's active rows are chosen among every inequality,
not the first ten (`project`). `tests/tight_corners.rs` is the gate. **Where the steps do not
settle, the nearest point stood on that keeps every condition** (#994, `conditions::nearest`): a
condition holding with no gradient is no row of a step, so a step from a point keeping it can land
back where it fails, and the two alternate — a ball's centres beside a blend ended by a side an eye
stands out past, the eye's patch beyond the blend's end holding a centre back only near it. The set
was then read as empty, its distance past every number, so `Run` read every point there as one no
ball covers. A point stood on that keeps every condition is of the set, and its distance no less
than the nearest's; and where the conditions read where a step landed have no point in common (a
chain read through another face a hair inside the body), the step is taken back half way, a few
times, first. **A set stated as another's terms and more, measured less that one** (`d(p, E) − d(p,
M)`, `E` within `M`: `std.rounds`' `rolled` within `centres`, [rounds.md](rounds.md); #994,
`region::general`'s `measured`) is read as written, `d(p, E)` the distance to the nearer `p` of
two points of `E` found: its point nearest `p` sought from `M`'s point nearest `p`
(`SetDistance::distance_from`, `conditions::nearest`'s `start`), and its point nearest that point of
`M` — each a point of `E`, so `d(p, E)` at most, the second found where steps toward a far `p` do
not settle (`E` a point, the centre of a ball a round pocket's floor closes onto: sought from `p`
alone, half `tests/tight_corners.rs`'s probes read `E` as empty). So nothing where `M`'s point is
of `E` (a ball rolled along the edge), and nothing to say where `M`'s is found nowhere.
Sought from `p` itself, `E` a set of no inside — the centres of a ball touching both faces, two
points where a boss crosses a filled edge — its steps did not settle under a chain read through its
faces' own surfaces past their creases (a fill's round cut across by the boss), and half the gate's
probes of the boss's fill went wrong (`tests/round_onto_round.rs`); from `M`'s point they start on
the set `E` lies in. (Read instead as the distance from `M`'s point to `E`, an upper bound of the
difference by the triangle inequality, the field read a smaller set than the definition states.)
The lowering reads the term by the section it builds (`recognise::rolled_by`: within a rolled
ball's section the nearest centre is that ball's, the difference nothing). **A face standing between the point and
the chain's nearest patch is gone round only where it bounds the chain nearer the point than any
face not of the chain does** (#994, `BodyFaces::nearest_face`): past a band's floor's rim at its
bowed side, the side bounds the clevis's eye 36 mm off at its far end, its own end and top nearer,
so the floor's plane is read there, not the eye's cylinder, whose side, 36 off where the field is a
hair past nothing, kept the gate's probe of the run's tangent line from being withdrawn ("disagrees
with the field", `tests/resting_touches.rs`); a band's side on its plate bounds only faces of the
chain, and reads as before (`tests/tapered_bands.rs`). `d(p, E) <= d(p, M)` with `E` on the patches
of the faces `M` is on the surfaces of is read as `d(p, E) − d(p, rest)` (`Rest`: `M`'s
nearest point when off `E`, and where `E`'s run ends), the same points with a negative inside.
`ends` is `max σ·side` over the faces at each end of the run (σ the side it runs on), a face
carrying the run on none: one going on tangent from one of the run's faces across a seam
(`BodyFaces::tangent_at`) that meets another of them, or a face one goes on into across a seam (a
fill's cap flush with a side), there in an edge of the run's own turn, along which the round goes
on, or that is a piece of the surface of the run's own face there (a floor two cuts leave in two
pieces) (`BodyFaces::carries_on`, #994) — a face going on tangent that meets them there only in an
edge of the other turn carries nothing on, and the run ends on it (one meeting them there in no
edge at all carries it on, as any face going on tangent does: a block's end flush under a flange's
side, tangent to its corner arc at the vertex alone, #996's `flush_ends`) (a deck's corner
arc running tangent into its side, flush with the block's, the block's side meeting its top in a
convex edge: the foot's fill kept within the block, not hung past its wall); nor one whose
surface the run itself goes across (`BodyFaces::goes_across`: no side of it holds the run), nor one
whose surface holds the run's edge there (a face along the run, its patch past the end, #996), nor
one back to back with one of the run's faces there (#996), nor a plane of the run's own body turning back over it at a reflex corner of less than a right angle,
meeting a face of the run in an edge there (one met at the vertex alone makes no corner, #996) —
the run ends there square; another body's wall so is its end (`BodyFaces::another_body`, #994) —
and with them where an edge a run-out at that end runs along ends, a face or the plane
square to it (`BodyFaces::end_faces`, `faces::EndBy`, #952, [rounds.md](rounds.md)); a closed run
ends nowhere. A run going on across a seam ends where its smooth chain ends, on each face there
whose solid it is kept out of or that is another body's (`BodyFaces::chain_ends`, #994). Where `S`'s
edges are in several runs (`BodyFaces::runs`; a crease a slot cuts across, #881, [0.209]), a point
is held to the ends of the run nearest it (`EndRuns`: the run of the edge `nearest_edge` names), the
run `lowering`'s piece beside it is built along: within every run's ends at once is within none
where the runs either side of a slot end on its two sides, and within any run's (their union) is too
much where a run ending on one post's side goes on past the next post. In a set measured from, `q
inside S.ends` is one condition per such face (the `k`th the nearest run's `k`th face, nothing where
it ends on fewer), and `Rest` keeps an end of `E` only where `M` carries on past it within them:
std's `meeting` is cut so, and a run is carried past its edge's end and cut off by the face it ends
on (#195). `edge` and `seam` are element keywords: a set may not be called so.

**Distance from a set, offsets (§6.21, 0.64, #145 F4):** `p distance(d, side: outside|inside) S`
(`distance` is a set use, `may_use`), its bound `p distance(<= d, side: outside) S` and
the set grown, `{ q | q distance(<= d) S }` (where one is passed or used), are one bound on the
signed distance from `S`'s surface (`Reach`: cmp toward the outside, whether
the offset grows it, `reach_of` over `Written::lower_side`'s form; only a `distance` with `side:
outside|inside` is a set use, `sets::reaches`), applied by `sets::expand_reach` in three representations: a body whose
one relation is a `distance` gets its number moved (`(r) + #grow`, the grow settled in the use's
scope and given to the application, `Application::give`); otherwise, at a region solid's probe
(`SetState::probes` through the alias table), the bound carries it (`OpArg::Bound::grow`, `Written::grow`,
read by `constrain` into `RegionTerm::grow`, a `TermNumber`; a grow off a probe is E040) and
`solid/region.rs::fold` builds the exact offset (`within(t, v, g)` is `{sd ≤ g}`; `narrow`: a cone
shrunk is its apex moved `-g / sin`, grown is the shifted generator plus an arc about the apex, one
loop; wider than square the opposite cone shrunk); otherwise a drawn point's **foot**, declared at
the body's point under the application (`hidden_point`, as a tangency's contact; `hint(at: p)`),
the body walked `Itself` then along
`Chart(0)`/`Chart(1)` (keys `{prefix}t{k}.`), with `p distance(#reach) foot` (its derivative is the
sphere about `p`'s), `p outside S` (the side) and `foot inside S` (so the seed crossing puts the
foot across the surface from `p`) appended to the body under synthetic ids (`FOOT_STATEMENTS`
down, so `PredWalk::written` keeps their uses apart). `region_body` counts no bounded side of
another set among a body's numbers (a distance from one is the set grown's). `tests/offsets.rs` is
the gate.

**A body grown, built exactly** (`std.rounds.Grown`, [0.194]): `B` grown by `d` is a set of the
language's own words, `near := { p | distance(p, within) <= d }` over `within := { c | c inside B }`
(B shrunk, `{ p | p inside B; distance(p, beyond) >= -d }` over `beyond := { c | c outside B }`),
and its field reads it as any distance from a set.  **Within a body in pieces** the set is read
shell by shell (`SetDistance::Nearest`, `BodyFaces::shells`): within the faces of a body's several
shells at once is within none — each face's surface's side taken in common
(`Reading::convex_pieces`) held nowhere for two blocks apart — so each shell's conditions stand
apart and the nearest is read (`conditions(…, shell)`, each reading of a term's faces kept to the
shell), and `convex_pieces` refuses faces on several shells.  `solid/grown.rs` builds the grown
region exactly, a route of its own (`region::Route::Grown`, after the lowering table, before
`bounded`): one term, the distance at most a stated number from a set whose one term is the inside
of every face of a body none of whose terms reads its faces (`faces::stratum_is_whole`); the recipe
walk asks the route which node it is built on (`Route::built_on`).  Outside `B` a point's nearest
point is inside a face, an edge or a corner, so that partitions what `B` gains: each face's prism
`d` out along its normal (a plane face's loops; a cylinder's band between two circles about its
axis, its radius out of a shaft or into a bore, `band`), each convex edge's (`BodyFaces::convex`; a
tangent one none) wedge of radius `d` between its faces' normals (along a line a prism; about a
circle a torus's stretch, turned), and each convex corner's sector of the ball: a prism's corner —
three faces, one square to the other two — the quarter disc turned about that one; **any other
convex corner** where planes meet along lines (a tetrahedron's, an octahedron's four), the ball cut
back to the plane each edge's wedge ends on there — spanned by its two faces' normals, square to
the edge — so its sector is the ball within the cone the normals span and meets each wedge face to
face (`ball_sector`; the ball turned about an axis square to the way out of the body from a half
disc square to it too, so its poles and the meridians its turn is read from stand outside the cone
and the sector is one face of the sphere — the axis and the seam's start set midway across the
widest gap between the lines the cuts' planes meet that plane in, read round a quarter turn, so no
cut passes through a pole or the seam, as an octahedron's would).  A vertex is a corner only where
its faces' normals are three apart to the body's reading of tangent (`faces::TANGENT`, #962): two
faces going on from one another across a seam tangent only to the drawing's stated numbers (a lobe
and a waist a ten-millionth of a radian off) stand no corner, and their prisms meet in the seam's
section (`rim_seams::a_body_grown_over_stated_seams_stands_no_corner_there`, Steiner's to 1e-9);
read to a billionth, a corner's sector was cut there and no piece met it.
`grown::convex_corners_of_any_faces_grown_are_built_exactly` is
the gate (brep-bench's offset/tetrahedron-grown and an octahedron, to Steiner's).  **A concave
edge** (two planes along a line) has no wedge: its faces' prisms overlap in a `d × d` column there,
so each is cut back to the edge's bisector, the plane through it halfway between them
(`beside_concave`: a face beside one a convex polygon, its section's concave sides pushed out past
it so the bisector crosses the prism clear of its edges, along the face's own side at its foot;
refused where the sides' lines moved in by their cuts bound no polygon of as many sides, a face
narrower than the reach).  At a corner where one concave edge meets a face square to both its faces
(a step's end, a pocket's rim) the ball has no sector and the two wedges along that face are cut
back to the same bisector (each begun `d` past the corner, so the cut crosses it clear; its
cylinder's trace there one ellipse, the other wedge's the same), meeting in it; where three concave
edges meet (a pocket's floor) the three prisms meet in their bisectors, and nothing else stands
there.  A piece cut back is a `region` in the recipe: its sweep in common with a block standing on
each bisector (`square`, past every piece).  **At a foot the bisector is a cone** (#693).  Where a
convex edge stands square to a face between two concave ones (a boss's foot), a point over the face
nearer it than the edge (`z < r`, `r` its distance from the edge) is the face's: the edge's wedge
is turned about it (a `revolve`) from its section cut back along the cone `z = r`, its apex the
corner, at half a right angle about the edge, and the face's prism keeps the rest; the cone meets
each foot edge's bisector plane tangent along the line they share.  Where a whole circle is concave,
a plane meeting a cylinder square to it about its axis (a post's foot, a bore's floor: `foot`), the
band's rectangle is cut back along the cone `z = |ρ − R|` about the axis, its apex `R` beyond the
plane (opening away from the plane about a post, toward it in a bore).  A face about a foot
(`about_feet`) — a hole whose edges are all concave, a convex polygon each corner of which such an
edge stands on with each wall square to the face (`about_polygon`) or one circle (`about_circle`),
or a rim that is one concave circle — is its prism with the loop moved `d` across the face (a
polygon's sides square to themselves, an arc of radius `d` about each corner; a circle's radius) and
the pieces between, each its own sweep with nothing cut: along each side the triangle under its
bisector (`z = h`) swept along it, at each corner that triangle turned about the edge standing there
from one wall's normal to the next, about a circle the triangle under the cone turned once from
where the circle starts (where the band's seam and the body's vertex are).  No Boolean meets a cone
here: at a boss's foot its apex lies on the wedge's axis.  The face's pieces are a `glued` node:
glued to one another (`boolean::glued`) and their feet, faces on one plane meeting edge to edge,
made one (`boolean::planes_joined`: the shared edges dropped, the rest walked as one face's loops
read onto the first's plane, outer first), so they meet the body's face as one face, as glueing
asks.  Each piece shares its faces exactly with the next, so the recipe's `grown` node (`body`, the
body's own node, a dependency of the region's in `cad::recipe_with`; `pieces`, each a `prism`, a
`revolve`, such a `region` or a `glued` node of them) glues them to the body in turn
(`boolean::glued`), nothing intersected, and drops the edges and vertices the glued faces took with
them (`Brep::compacted`); the native host fuses them, a `glued` node's piece by piece.  Unioned, a ball tangent to a
cylinder along a circle is refused by the Booleans; glued, a block grown is Steiner's volume to
1e-15.  Gated by the region's field (`agreement::gate`, built about the body's boundary), which
reads within (or beyond) every face of a body, and nothing more, by the body's own signed distance
(`SetDistance::Body`, `BodyFaces::signed_distance`, where `distance_slack` says it is exact) — no
one surface's side bounds a body with a concave edge, and the projection onto the nearest face's
side runs on past its patch at a convex corner.  Refused by name, the field's: a concave edge on a
curved face but a band's foot, a corner where a convex edge stands between two concave ones not
square to the face they bound, a concave corner on a curved face or of more than three faces, a
face beside a concave edge that is no convex polygon or narrower than the reach (a hole's pieces
reaching another loop), a face about a foot with a concave rim and a hole, an edge or a band
shorter than the reach between its concave corners or its feet, a face on a cone, a sphere or a
torus, an edge no line or circle carries, a convex corner on a curved face or edge no prism's, a
bore no wider than the reach, a body a term of which reads its faces (a round).  `tests/grown.rs` is the gate (a block, a
post, a hex bar and a bored plate to their closed forms to 1e-9; a body in two pieces; an L-block's
step and a pocketed tray to Steiner's with its concave edges' terms negative, `−d² L` each, a step's
end `−2d³/3` and a pocket's floor corner `+d³`; a plinth's boss to Steiner's, each foot corner
`−πd³/4`, its wedge losing `πd³/6` and the plate's prism `πd³/12`; a base's post and a cup's blind
bore to Pappus's, the square `d × d` about the foot once, `2π(R ± d/2) d²` less; the field of a
body with a concave edge, grown and shrunk).

**A body shelled** (`std.rounds`' `shell`, [0.198]): `A shell(t) B` is a word of the language's own
words, nothing in Rust of its own: the cavity `solid({ c | c inside B })` with each face `w` of `B`
not among `A`'s (`repeat w in B { when not w in A { … } }`) cutting its wall `solid({ p | p
distance(<= t) w })` from it, and the cavity cut from `B.body`; `shell(t) X` the one region `{ p |
p inside X; distance(p, beyond) >= t }` over `beyond := { c | c outside X }` (`Grown`'s shrunk),
cut from `X.body`.  What it needed is the stratum's: **a body standing on a reader reads too**
(`faces::readers_of`, `reads_faces_of`: a region whose terms read the body's faces, or any other
body whose stock, walked down, is one — never the body itself nor a layer, whose stock is the layer
beneath), so the cavity is one of `B`'s late terms, and `without` leaves a reader as it is, since
nothing of it is in the stratum.  **Built exactly where `B` is a prism less what is cut from it**
(`solid/shelled.rs`): the cavity is recognised by what its sets say (`shelled::recognised`, either
form) and built as a prism — within a prism the distance from its boundary is the lesser of the
section's distance from its edges and the distance along the axis from its caps, so the points at
least `t` from its kept faces are its section eroded by `t` between its kept caps moved in by `t` —
less each solid cut from `B` grown by `t` (the erosion of `A` less `P` is `A`'s less `P` grown:
`grown::around`, a body grown's construction for a solid no region names).  The section eroded
(`shelled::eroded`): each kept edge moved in by `t` — a line along its normal, an arc about its
centre, its radius less or more as the material is inside its circle or out (`Carried`, read off
the cap's circles by their own normal; a whole circle one edge) — two meeting at a convex corner
where their carriers do (the meeting nearest the corner moved, `plane2::meet`), going on tangent at
the point both move to, and joined by an arc of radius `t` about a reflex one; an open edge, or an open cap, moved
*out* by `t/2`, so the cavity crosses the body's faces there and lies on none of them; where a kept
edge meets an open one at more than a right angle the arc about the corner runs on to the open
edge's line (a wall leaning in keeps a rounded lip; over an open arc it is refused).  Where the
prism's construction refuses, the walls' is asked (below), and a refusal says both.  A body's open faces are matched to the
stock's faces by their planes.  Its recipe node is the cavity's (`cad.rs` asks `shelled::built`
once a solid, before anything else of it, and writes it for the cavity in place of the regions it
is made of, depending on what its construction reads, `Shelled::reads`): a `prism`, or a `region` keeping within it and outside each cut's
`grown` node.  **Any other body by its walls** (`shelled::walls`, a boss put on it): the walls are
what lies within `t` of what is outside the body, so they are the body's faces turned inside out
(`BodyFaces::inverted`: each face's side turned, each edge's convexity the other way, read off the
body's own and not classified again) grown by `t` in `Grown`'s pieces (`grown::pieces`) — each
face's prism cut back to a convex edge's bisector, a wedge about each concave edge (a boss's foot),
cut back at its corners as a step's end; a face whose rim is all such bisectors may now hold holes
(`grown::about_feet` cutting its rim back by `cut_back`, `beside_concave`'s, a plate's top about a
boss's foot), and a side running on in one line past a vertex is one side (`cut_back`: two faces
glued and made one leave the vertex).  An open face's prism `2t` out from it is glued on first
(`section_of`), so the walls run out through it, and a wall standing on an open face in a concave
edge is refused (its foot's quarter round hangs below the opening).  The cavity's node: a block
holding the body clear less one past it less the body (an inline `body` of its other terms) with
the openings' prisms (`glued`) and the walls glued to that (`glued`), so every Boolean either glues
faces shared exactly or nests a solid inside another; it depends on the body's other terms.  The
construction reads the body's stratum through a `placed` node of id `STRATUM`, the recipe the same
node of the body's terms.  The native host builds any node nested so (`Session::solid`).
`sweeps_among` reads the solids a built cavity stands for as built (`shelled::stands_for`, by what
its sets say; the cavity built only where nothing else decides), `exact::build` names the faces of
a solid whose node is no `body` under it as a region's — so the cavity's — each wall's inner face for the face it lines (`south`),
the arc `round`, a cut's grown faces `round` — and a body's stratum is gated: the stratum less the
cavity, built, against the body's field (`shelled::gate`).  Refused by name and read by the field:
a body rounded as well, a face left open that is none of the stock's (a cut's) or only part of
one, walls that meet or cross (`shelled::simple`, by `planar::meets`), a prism no longer than twice the thickness, a
cut no body grown builds, walls no body grown builds, a wall standing on a face left open.  `tests/shell.rs` is the gate: a block sealed and open through a face, an L's
concave edge rounded within by `t`, a boss shelled with its plinth, the same open through the
slab's top (its walls running on round their foot, a quarter round), a post added to a pan shelled
and named, each at a lattice of points to its closed form; built exactly, the block sealed and open,
the L (`8 (160 − π)` taken), a tray's pocket and a plate's bore grown out of the cavity, a hopper's
leaning wall's lip, the plinth sealed, as a chimney and with a pit beside its boss (its foot
`100 − 10π + 8/3` under the boss), each to its closed form to 1e-9; the refusals said, and the
faces' names.  `rust/examples/shells/` is the series (`tests/shell_examples.rs`: every case's
closed form to 1e-9, a rounded enclosure, a cup, a turned can, two bosses square and round, walls
leaning in and out, 120° and acute corners; `gcs-cli`'s
`the_shells_export_and_measure_within_their_tolerance`: each written as STEP and STL within
10 µm).

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
  `M` too).  Refused at the use (E040): a curved side, a through prism's caps, a non-prism.
  **Sketching on a face** is `v1.origin coincident b0.near.surface` + `v1 tangent(at:
  v1.origin) b0.near.surface` (`at:` takes a path), written by `edit::sketch_on_face` seeded on the numeric twin
  `solid::carrier` where the eye's ray met it (`SolidHit::at`); `edit::face_plane` finds a plane
  already lying there.  The app no longer offers it: everything is drawn in space, and there is
  no plane to choose (app.md).  A plane dropped from a plane states `P distance(d) v` and is sized as an
  extrusion is (`LiveExtent` kind `offset`, `workspace::offset_handle`/`offset_at`; previewed by
  the constraint's number and a solve, written once by `set_dimension`).  `tests/face_sets.rs`
  and `tests/edit_datums.rs` are the gates.

**Sheets, their sides and the square band** (#645, [0.192], [developables.md](developables.md)).
A set whose body says what `std.panels.Developable`'s does is a sheet
(`flatten::developable::sheet_body`), and a contact over an interval in a set's body makes no region
of it: under a set's derivative the flattener keeps the incidence and emits no bound
(`Walk::within`), and a sheet's body is read whole by the recogniser. At a region solid's probe `q
distance(<= d, side: outside|inside) S` of a sheet is intercepted in `expand_reach` (`reach_sheet`):
the side said, `<=`, `d` a stated length more than nothing, else E040; its term is
`RegionShape::Sheet`, the band along the normal from a foot on the panel, so square at its edges. A
sheet used as a set any other way — a distance off a probe, an equality — is E040 naming the band's
spelling. Every set the document makes that is a sheet is recorded for the report
(`Expansion::panels`, `Sketch::panels`).

**A closed shell, and the solid it encloses** (#953, [0.221]). A set whose body is a
disjunction of memberships of panels and drawn faces, `shell := { q | q in bottom or q in port or
q in deck }`, is the union of those surfaces: `syntax::notation`'s `disjunction` lowers a drawn
face's membership to `coincident` (a panel's to `outside`, as any set's), where through 0.220 a
face there was refused. Its **side** at a region solid's probe, `hull := solid({ p | p in
shell.side })`, is what it encloses: the flattener recognises the shell by its body as written
(`Walk::shell_members` — one `q outside #s`, `#s`'s every statement a sheet's `outside` or a
face's `coincident`; a member that is a set and no sheet leaves it the union of regions it has
always been) and, at the probe, records it (`Walk::enclose`, `EnclosedInfo`, `Expansion::shells`)
rather than applying its body; `program::developed::enclosed` makes the probe's term,
`RegionShape::Enclosed { members }`, each a panel by its rails or a face by its entity, named by
the set it is. Refused (**E040**): a shell read from a drawn point, or as anything but its side.
A member that is neither is refused where the shell is used. Built by `solid::enclosed` (below,
[developables.md](developables.md)); its faces are its members' names (`hull.port`).

**Outside a set, and Booleans as sets** (#737 P3, [0.199]): `p outside S`, `S` a set no one bound
of which turns round in place (a conjunction, an interval, several numbers: `region_body` refuses
it), is outside the set as its own point says it — `sets::complement` gives `S` a hidden point,
keyed `#{stmt}.c` (`keys::outside`), applies `inner coincident S` there under a prefix of its own,
and rewrites the statement `p outside inner`, which `relations::shape` names `Shape::Outside` and
`face_or_ends` reads (`relations::outside_term`) at a region solid's point as
`RegionShape::Outside { set: Target::Set { probe: inner } }`, filled when the
solid is built (`regions::fill_sets`).  The field reads it as the negated field of the set's terms
taken together (`region::general::field_of`, unbounded allowed), its support the union of the
boxes of each term turned round; so the outside of a conjunction is the union of its terms'
outsides.  A set whose terms are solids named whole (`FaceSide` of a bare solid at nothing) —
within one, within or outside others, or one `Outside` of several outsides — is built as
`SolidDef::Body` over them (`solid::defined`'s `boolean`): the stock, what is in union with it,
what cuts it, what it is in common with — so `{ p | p inside plate; p outside bore }` is the body
`bore cut` makes, to the byte.  A union of regions is read point by point (the meridian fold takes
no complement yet).  `tests/solids_as_sets.rs::a_boolean_of_solids_is_the_body_term` and
`outside_a_conjunction_is_a_union` are the gates.
