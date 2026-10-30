# Motions, generation and swept export

Part of the [contract](../../CLAUDE.md) the code is held to. Read it when you touch `motion.rs`,
`generate.rs`, `solid/admission.rs`, `brep/sweep`, or the native export. An issue number (#n) or a
spec version ([0.n], see the [language changelog](../solvent-changelog.md)) says where a rule came
from.

**A swept cut's faces are named by the cutter's faces that made them** (#212): a traced sheet's
contacts each know the cutter piece they came from (`contact_trace::Station::starts`, `Loop::starts`,
a convex corner's fan `cutter::FAN`, unnamed), the rows are laid band by band so every column that
reaches the blank crosses from one cutter face to the next on one row (`Anchors`, `Rows::Walk` only;
a margin column crossing otherwise keeps walk-length rows), and the fitted net is cut there
(`Fitted::faces`, `build::sheets`, edges shared) — `Fitted::face` stays the one judged fit. Each
band is named by its face's path in the cutter (`Cutter::face_of` → `Sectioned::face_of`, a step
tagged by its profile edge; routes `SweptCut::routes`) and each placed copy keyed by its placement
(`cad::SweptCut::key`, `brep::recipe::NAMED`); a boolean never renames. The meridian blank and
`recipe::build` name faces by node too. `BodyFaces::read` reads a swept body (`solid::exact::swept`,
`exact::named`), `face_paths` lists a sweep's cutter faces and a region's terms, a region's term is
named by the set its body used (`Worded::through`), and `faces::face_source` reads a path back to its
analytic surface: a term's or a revolution's face (`RevolvedSurface`, `moved` by placements), or a
cutter face's envelope under the sweep (`FaceSource::Swept`, `within` the cutter's own placement).
The analytic snapshots are made of parts too (`GeneratedEnvelope::new`, `TrimmedPatch::of_envelope`,
`EnvelopeSeam::new` with structural ends, `BoundarySeam::new`, `SurfaceSeam::new`,
`BoundaryVertex::new`, `JunctionVertex::new`, `SpatialEdge::new`, `SpatialFaceBoundary::new`).
`tests/swept_faces.rs` is the gate.

**A frame's joint is its motion** (#383 stage 3, [frames plan](../frames-plan.md)): a frame's
`axis`, `ratio` and `advance` are minted where a reference names them (`Expansion::joints`, read in
`program::entities` by `Sketch::mint_joint`: an `AxisE`, its direction seeded along z then by
`seed_joints` from what holds it, and two parameters, ratio 1 and advance 0) and held by
ordinary relations: `std.motion`'s relation words `turns`, `screws`, `spins` (about the axis through
`o` square to `o.plane`) and `slides` ([0.154], #464).  The components `std.Turn`, `std.Spin` and
`std.Slide` they replaced are retired ([0.176], #562): each stated a fact about the frame it was
given, which no definition may (`tests/joint_words.rs`).  A `fix` of a joint holds its numbers to
what they are — a ratio a plain number, an advance a length (E103) — whoever states it, so a word
handing its own on (`f turns(2mm) l`) is refused as the components' typed formals refused it.
A `fix` in a word's body is held within, as a component's is (`ResolvedRelation::worded`), so a reconcile does not write it again at the root.  Before
the solve `program::motions` reads each joint's kind off its holding statements — `fix`'s
`ratio`/`advance` slots, `coincident` with a line or point, `parallel` with a view's normal — and
pushes the `MotionDef` the readers know: a line with a ratio stated 0 `Translation`, a line
`Rotation`, a
view's point with the axis square to it `Turn`, anything else `Joint { axis, ratio, advance }`,
which `Family::read` reads (ratio 0 a translation), the held steps read about the joint's own axis
as the solve places it (#366, [frames.md](frames.md)) and the planar program refuses.
A joint's number no `fix` states is the solve's (stage 5); a measured one is E107.  The joint's
`motion` is that node; a frame without a joint is the world.  `motions::reading_of` is every
reading's one entry (`under:` on a solid, envelopes, meshings, held and planar and extruded
envelopes, sweep claims, `under:` relations): the frame `under:` names (`reading`), or with
`under:` left out what the tool rides on ([frames.md](frames.md)), seen from another frame as
`relative(source, observer)`, memoized as `"m seen from g"`; a frame with no joint read `under:`
"does not move".  A traced body takes a frame formal as its motion's steps (no columns at
rest), `seen_from` composing them, and `traced::{lifted, columns, theta}` read a frame argument's
joint.  `motion::evaluate` takes radians, returns exact pose and derivative via
`Sketch::world_point`; relative is observer inverse times source.  Cycles and depth over 64
refused.  `gcs_frame_sample` (browser `core/frame.ts`, six doubles) samples a frame's joint and
`gcs_frame_moves` says whether it has one.  `motion(…)`, `m: motion`, `relative_to:` and `phase:`
are refused naming what they became, and no motion has a phase: `MotionDef`, the steps
`motion::Family` and `envelope::held` read and the planar tape (`generate::Op::Turn`) turn by the
ratio times the roll alone.  `tests/frames.rs` is the gate.
**A chain** (#383 stage 1, [frames plan](../frames-plan.md)): `Sketch::chain(outer, inner)` is the
pose `outer(τ)·inner(τ)`, lowered as it is pushed and never evaluated as a new step: two joints
over one axis entity (one line, one centre) whose numbers are stated fold to one joint, ratios
and advances adding (so `Family::screw`/`turning` and the generating classes read a train on one
shaft as its one turn); any other pair is `Relative { source: inner, observer: inv }` over a
minted `MotionDef::Inverse { of: outer }` — a joint's inverse runs back (numbers negated), a
relative's swaps its sides — read so by `Family::read` (`Step::inverse`), the planar program
(`generated::program_signed`) and the held steps (`envelope::held::motion`, the same axis columns).
`Family`'s shape queries read the root step, so an inverse's leftover steps hide no joint. Heights
count the minted nodes (an inverse is as deep as what it undoes). `tests/frames.rs` is the gate
(a planet against `R(τ)·(c + R(−3τ)(x − c))`, three turns against the hand product, a turn on a
slide against the cycloid, coaxial screws read as one screw, 64 levels), with a `CarriedPoint`
under a chain FD-checked over both axes in `tests/jacobians.rs`.

**A face family its statements say a frame carries is that motion's sweep** (#366, [0.138];
what a frame read at a roll is, and the station words written so, are [frames.md](frames.md)'s):
`solid::family::stated_carry` reads the rows of the station that read its running number, and
where every one is a `CarriedPoint` at a stated roll under one motion — one turn or screw about one
axis (`Family::turning`) or one slide (`Family::sliding`) — at a roll affine in it, the recipe's
`family` node carries that motion over the interval (`carry`, `Carry::Screw`/`Line`, written in
millimetres) and `brep::family` builds the walls on the first rim so carried, every wall still held
to the general sheets; read off the stations (`brep::family::carried`) only where no statement
says.  Recognised by what is stated, never by a word's name.  So `lofts/spring.sv`'s wire and
`swept_spring.sv`'s ball are one tube on one screw (`Blend`), the canal's half balls the only
difference, and both export alike.  A part walked for a station follows a motion to what it is
written over (`graft::Part::around_all`: a motion owns no number, so is never a wall).  The gate:
`tests/carried_family.rs` (the springs, the screw's walls, a station read otherwise).

**A frame's numbers are stated or solved** (#383 stages 3 and 5): a ratio or advance measured
off the solved drawing (`ratio: -length(a) / length(b)`) is E107, as every measurement in a
number is, and the measured motion numbers of 0.27 are gone (`measure.rs`, `MotionE::measured`).
One no `fix` states is an unknown of the solve — [frames.md](frames.md)'s joints in the solve —
read after the solve where the solve left it (`MotionE::rotation`, `advance`, `Family::read`'s
`Joint`, `solid::reads`, so a cache key moves with it), and refused by the planar program
(`generated::program_signed`), whose numbers are constants of its tape.

**Solid placement:** `indexed := source at(under: indexing) 90deg` places a solid at a
motion's pose; `at:` is a constant Angle, not a hint or sweep. Repeatable, nestable, Boolean
operands; copy keeps source and motions. `make solventc OCCT=1` enables native STEP via a C ABI
over OCCT; the core supplies construction data, the CLI owns shapes and catches exceptions. A
written STEP is parsed back by `native::step_check`; `--verify-step full`
(`SOLVENT_STEP_VERIFY=full`, slow tier) also reads it through the kernel. Native `--stl` defaults
to OCCT (`--stl-backend mesh` legacy). STEP+STL build once, check both, then replace each. Native
units are mm; STL 0.01 mm deflection, 0.2 rad — settings, not an error bound. Every CAD face must
be meshed; `mesh::stl_shells` checks every component, cavities included, unwelded. Failure keeps
old outputs; renames are atomic per file, not across files. Native-path tests must not hide legacy
mesh defects.

**Planar generation (§6.15.1, #61):** `envelope(tool over t in (a, b), under: m, side:)` over a tool
of the sheet (point/line/circle/arc/formula curve) and a planar motion (`spins`, a turn about a
point of a view, is `MotionDef::Turn`, a rotation square to its view in space) is a **curve**
(`CurveBody::Envelope`, `program/generated.rs`, built with motions after the primitives, before
constraints): `F = X_s × X_t = 0` in the tool parameter, `C`..`C''` exact by `taylor::Jet`s, `C'''`
and θ gradients by difference (`generate.rs`); tool and motion geometry are its columns (shared ones
once; the motion's numbers ride past `tape::MAX_VARS` in `OUTER_MAX`).  Analytic tools are exact to
`C'''`, so a profile they cut may cut in turn (`generate::Tool::Envelope`, read as its Taylor
series); the encoding is `Generated::new` and `view` only.  Kernels are generic over
`kernels::{FORMULA,TRACE,ENVELOPE}` and ask a body for the orders they `need`. `tests/generation.rs`
is the gate.

**One engine for a prism's side (#70):** `side := surface(prism, edge: e)` names a prism's side
too; its `envelope` under a motion keeping the prism's view is built with the drawing
(`generated::extruded_envelopes`, after memberships, before relations) as `e`'s planar envelope
marked `CurveE::extrusion`, the face in a view that stands where drawn. `p coincident
flank` from any view is `CKind::PointOnExtrusion` (`FamilyKernel::Extrusion`, consts
`kernels::EXTRUSION_FRAME` then the contact's; `Sketch::extrusion_frame`). The export builds a
prism cutter's sheet the same way: `brep::sweep::extruded` (edges as lines/arcs, convex corners as
points over their fans, `generate::cut_at`, Hermite spans, extruded; `Family::in_frame`), else
traced (`sweep::traced` kept to compare). `tests/extruded_envelope.rs`, `rack_cut.rs`.

**A ball swept is its tube** (#238, #684 D, `solid::canal`): a sweep whose tool's recipe is one
ball (`brep::recipe::ball_of`), under any motion, is the points within the radius of its centre's
path — the ball's disc square to the path carried along it, united with the half ball beyond each
end: exact, its end poses the construction's own, so no clearance (E1) is asked and a sweep may
stand alone. Under one turn, screw or slide (`Family::twist`) the path is closed form
(`brep::build::canal`): a partial `revolve`, a whole torus past a whole turn, the disc carried to
its screwed image by `build::ruled` along `Carry::Screw` (a `Blend` carried by a turn and a step),
or to its slid image along `Carry::Line` (a cylinder; a slide is a screw of no turn), refused (left
the constant twist's or the field's) where the centre stands within the radius of the axis or a
helix past a turn climbs no more than the diameter a turn. Under any other motion — a relative
roll, a frame along a path or one the drawing moves, a driven joint — the path is read off the
motion (`canal::along`): the disc a face family's one closed `Run::Round` wall over the roll, its
stations the centre's point and the radius vectors of the frame square to the path, carried from
the start by double reflection (`motion::reflected`) once, over 4096 steps of the roll, any other
roll one step from its nearest (`canal::Carried`), its pieces of the roll (Gauss–Lobatto,
`family::weights`) each held to the disc read near both its ends within a face family's `EXACT` of
the diameter, so the wall is fitted, and lies on a cylinder or a torus where its
numbers do (`brep::family`'s `analytic`); built as the family and the end half balls united
(`build::capped`). Its rows, sampled at points an eighth of the radius apart along the path
(1024 to 32768) and said so, refuse it by name with a point of the path, in the canal class
(`admission::Asked::Canal`, asked first by `admit_body` where no other class reads the motion):
**B1** the path never stops and turns nowhere tighter than the radius; **B2** it comes back within
the diameter (and the points' spacing) of itself nowhere half a turn of a circle of the radius
along it from (`canal::refusal`). The recipe's `"canal"` node (`centre`, `radius`, the ball's
`face` naming every face, and either the axis `origin`/`axis`, `angle`, `advance` or the face
family's node as `tube`; native OCCT refuses it by name); gated by agreement with the sweep's
field (`Caches::canal`, which remembers the refusal too); `cad::contains_sweep`, `swept_cut` and
`sweeps_among` read such a sweep as static material. Seams run off the path's plane (`build::SEAM`)
and each half ball is a ball turned about an axis leaning off the path (`half_ball`, kept by
`build::kept`): a plane through a sphere's parameter pole is a meeting this kernel's Booleans do
not split. `swept_groove.sv`, `swept_spring.sv`, `solid_generating_sweep.sv`,
`solid_sweeps/path.sv`; `tests/canal.rs` (Pappus, to 1e-9) and `tests/ball_paths.rs` (a slide's
capsule, a spline in space, a roll's trochoid, an arc's torus, B1 and B2, each length integrated
apart) are the gate. Field-path tests sweep a cylinder along its axis instead
(`solid_sweep.rs::slid`).

**A sweep's roll through what it cuts** (#271, §6.9 [0.88]): `sweep(tool, through: body, under: m)`
is `SolidDef::Swept { roll: model::Roll::Through { body } }` (`Roll::Stated` is `over t in (a, b)`,
an interval of the shared roll, `Running::roll`: [parameters.md](parameters.md)), the roll about 0
during which the tool meets the body's material (`document::material_sources`: stock and additions,
never cuts, so `removal` cutting `body` is no cycle; `evaluation_operands` reads them). **Every
reader asks `solid::roll::interval`**, never the extents (admission's `SweepContacts`, the field's
`SweptField`, `SweptCut` here and in the CLI, `faces::face_source`, `canal`, `planar_class`),
memoised against `solid::reads` (`Caches::roll`). Found by marching out from 0 a `STEP` (1°) at a
time, both ways and a few steps ahead on every core, to the first step the tool is proved clear over
(`roll::clear`: the tool's support and the step split by branch and bound, a box dropped where the
tool's field encloses it empty or the step's `Family::bounds` carry it where the body's field
encloses it clear, the step halved where poses smear a box past its size; a centre in both at the
middle pose ends a step early): conservative, a few degrees past first clearance, never short, so E1
holds by construction. The boxes are one tree whatever order they are walked in, so the answer is
the order's to choose freely: each step walks first the halves toward where the way's step before
ended (`roll::march`), and a step the tool has not cleared ends in a few dozen boxes, not hundreds
(the same rolls, a fraction of the time; a document's workers each find them before their first
surface). Refused naming why (no meeting at 0, not clear within two turns, a tool with no bounded
support). The spiral bevel, the rack cuts and the lantern state no roll; the drill's flute and the
Wankel's period are design. `fixtures::gear::roll` restates one member's roll (negative controls).
`tests/roll_through.rs` is the gate (two balls parting at `2 asin(1/3)`; the pair at 10:48 admitted;
in the slow tier each member built as its stated roll built it).

**Generating sweeps and the native export** ([docs/generating-sweeps.md](../generating-sweeps.md)):
`solventc --step/--stl` builds swept cuts only for the bevel/hypoid generating class, the
constant-twist class (screws, turns and slides) and the planar classes (below).
`solid::admission::admit_body` checks rows T1–E4 (revolved line/arc tools, rotation with changing
contact, clear at both limits, one contact per point, no fold, no self-crossing) and T3 where a
prism's cap reaches the blank, sampled, refusing
with row and witness; it checks a placement once when the blank reads alike at every point
checked. A rack is in the class (#61): a prism tool with its caps clear of the blank
(`ToolSurface::Extruded`, sides only; a cap in the blank below, #684 F), a translation seen from a rotation (contact affine in the
roll, `NormalVelocity::Affine`), sectioned along its extrusion (`brep::sweep::cutter`, a station a
distance), flat caps joined across sector sides (`brep::pattern`, `Kept::Flat`); a blank reaching
its axis is built whole. `tests/rack_cut.rs`, `examples/generation/rack_cut_gear.sv`. A fold is
the sign of the area factor times the contact condition's rate. Built in
`gcs-cli/src/cad/native/sweep_boundary.rs`: cutters sectioned (`backend/sections.cpp`), contact
times from `SweepContacts::at_point_normal_over` (`PointContactError`), sheets traced and
resampled by the core (`solid::contact_trace`, fed section samples and nothing else; rows stop
outside the blank before a time leaps: `charted`; fall by walk length, or by length in space where
that fit misses or folds: `Rows`), indexed by the declared motion, blank split (fuzzy 1e-5 mm),
cells judged by `MaterialEvaluator::probe`. The construction is in mm, a field in the document's
units: every field it asks is a `solid::Millimetres` (points, values, a probe's radius in mm; its
constructors read the scale), never a bare `SpatialField`/`MaterialField` and a hand `x/scale`
(`tests/millimetres.rs`).
`UnifySameDomain` widens shared tolerances: unify a copy. `solid::agreement` probes 0.1 mm each
side (a one-sided disagreement withdrawn only where the centroid reads on the boundary); nothing
is written unless it agrees: outputs go through `cad::output::Staged`; native swept
construction needs `admission::Admission` (only `admit_body` makes one). Contracts in
`solid::contracts`; every refusal is one `solid::export::ExportRefusal` (`Stage`,
`SOLVENT_STAGE_TRACE`), reported at the solid's statement. Refine features:
`solid::blank_features` (`BlankTopology`). The configured pair is a 25 mm hypoid
(`spiral_angle`, and `pressure_shift` stated through `fixtures::gear::recorded_machine`,
`admission.rs::the_admission_grid`); recorded tests pin
`fixtures::gear::bevel` / `hypoid6` (`rust/fixtures`, dev-only);
`tests/native_surfaces/gear_cells.rs` holds the recorded tooth-space volumes;
`tests/generating_harness.rs` (ignored) locates refusals by stage.
The pair carries **backlash** (`Tooth`, `Space`: each flank cone grown a quarter of it), **tip
relief** (`Topping`s and the crown's regions over their cones, cut by `Generated`, under `when`)
and **end relief** (`Blank`'s `std.rounds.Chamfered`); `fixtures::gear::design` pins all to zero,
`fixtures::gear::fabricated` sets them. Files:
`build/exports/hypoid-{pinion,gear}.{step,stl}`; `gcs-cli/tests/pair_check.rs` (ignored) checks
the pair from the STLs.

**The Formate pair (#283, step 1):** `spiral_bevel/formate.sv` imports `hypoid.sv`'s layout,
blanks and face mill and drops the crown from the pinion. The gear is formed: its blank less a
space copied at every tooth with no roll (`Copied`, static placements, no sweep), every space
flank one of the mill's cones. The pinion is its blank less the gear's tooth (`gear_tooth`,
`hypoid.sv`'s `Tooth` region, the ring between those cones) swept `under: gear_turn, seen_from: pinion_turn`, the ratio
the teeth — exactly conjugate at any offset, and in the generating class as the crown's sweep
is. Two limits shaped it: the core sections a cutter only of revolutions about parallel lines
(`brep::section`), so the pinion's cutter is one ring about one axis and the space, not the
tooth, takes its two flanks from two cutter placements; and a space bounds the tooth ahead by
its flank alone, since a second floor a gear pitch round (3.35° off the first) broke the
kernel's Boolean at one copy of 48 (`a_formed_gear_with_two_floors_a_space_builds`, an ignored
reproduction). `tests/formate.rs` (the cutter and motion, admission);
`pair_check`'s `the_exported_formate_pair_is_conjugate` (ignored) reads
`build/exports/formate-{pinion,gear}.stl` and holds the pair to an unvarying clearance and to
line contact through the mean section.

**Screws, the constant-twist class (#64):** a sweep under a screw (`f screws(a) l`) is admitted by
rows S1–S4 (`solid::constant_twist`, docs/generating-sweeps.md): the tool's characteristic (each
ring's `A cos + B sin + C` roots under the screw's twist, fixed on the tool) walked through the blank,
a coarse pass then `rows` across the reach; S3 is signed. Its sheet is exact, S(s, t) = M(t)·σ(s), rows
along its section square to the axis (`Characteristic::on_section`: the characteristic has corners
where the profile's curvature jumps, the section does not), fitted by the shared `sheet::settle`
(`brep::sweep::helical`). A body built from swept material (`drill := solid(fluted)`, `shank union
drill`) is built by the body rule over exact operands (`brep::export::compose`; the page's `Builder`
too) and measured by it over operand meters (`accuracy::Meter`). A drawing reads a swept object's
exact B-rep (`brep::export::supply_exact`, `Sketch::supply_exact_solid`); one built with no faces is
a body with no material, read as a static solid's empty Boolean is (`overlap.volume = 0`), never as a
solid that failed to build, whose name would read nothing (#618,
`planar_subtraction::a_swept_bodys_empty_boolean_reads_volume_zero`). Kernel rules it found: every
vertex cuts the edges it lies inside (`boolean`, after the faces' meetings), a traced fit is laid and
graded by length and never crowds a vertex it must pass, a cone at its apex writes radius 0.
`rust/examples/twist_drill/`, `fixtures::drill`, `tests/twist_drill.rs`.

**Turns and slides, the constant twist's two limits (#684):** a sweep under one turn with no
advance (`f turns(r) l`) or one slide (`f slides(a) l`) is asked the screw's rows, S1–S4
(`Family::twist`, which reads a slide as a screw of no ratio about the line along it through the
origin). Its sheet is built by `brep::sweep::helical` along its section (`Screw::station`): a
slide's square to it, extruded across the blank's heights; a turn's a half-plane through its axis,
its points carried round at their own heights and radii (`Screw::time_to_station`, each node's
delay the one nearest the one before), revolved over the roll, its S4 read by radius and height;
the stretch of the roll each reader asks is `Screw::passing`. A turn of a whole turn or more is
read as one whole turn (below), and S1 also refuses
a ring in contact all round between two walked stations whose path enters the blank
(`constant_twist::stationary`: the amplitude turning right round, its three coefficients' least
sought by `roots::brent`) — a turn about an axis the tool's meets, or a slide along the tool's
axis — where the boundary leaves the walked branch with no root moving. A turn or a slide of one
ball is still its tube. The accuracy meter reads a turn's characteristic over the whole roll.
`tests/turns_and_slides.rs` and `tests/solid_sweeps_examples.rs` are the gate (volumes computed
apart: the lens, a circular segment, Pappus, and a tilted torus's shadow from its centre circle;
the STL held to the field), with
`rust/examples/twist_drill/straight_flute.sv` and `rust/examples/solid_sweeps/`'s `slide.sv` and `turn.sv`.

**A whole turn (#684 N):** a sweep is its path's, so a turn carried round a whole turn or more
sweeps what one whole turn does, a solid of revolution. Its contacts are read once round from the
roll's start where they are read (`SweepContacts::read`, by `constant_twist::once_round`), so every
reader of them — the walk, admission, the accuracy meter — asks one turn (`domain`), and
`SweepContacts::whole` and `Characteristic::whole` say so; the declared roll (`roll::interval`, which
the material field reads) is left as stated. E1 is not asked of it (`admission::ends`): the tool at
either end lies within what it sweeps, so the roll may start and end inside the blank. Its
sheet (`brep::sweep::helical`) runs its columns once round, the last column's pose the first's to
the bit; only its rows' ends must leave the blank. Fitted as any, it is cut into faces both ways
(`Fitted::named`'s `around`, `AROUND` faces round the turn, and a band each side of a convex
corner's fan) by `brep::build::grid_sheet`, neighbours sharing edges and the last round closing on
the first's edges: one closed face would carry a seam across the blank, which a split does not
finish. The native path refuses it by name (`--kernel rust` builds it). The gate is
`turns_and_slides::a_whole_turn_grooves_the_whole_ring` (`turn.sv`'s torus round the whole ring,
Pappus over its shadow; two turns read as the one from their start) and `solid_sweeps_examples::
the_wheel_turned_once_leaves_its_shadow_all_round` (`solid_sweeps/whole_turn.sv`, a V wheel leaning
across a rod turned once round it: twelve faces, three bands each four round; Pappus over its
shadow read by the signed distance from the nearer flank), each STL agreeing with the field,
`object_builder::a_whole_turn_builds_at_the_displays_samples`, and for the ring alone
`brep::a_net_closed_round_an_axis_is_a_ring_of_faces` (a cylinder's net cut in two along and four
round, its edges shared and closed, splitting a block into its core, π r² h).

**A screw with sharp edges: a convex corner's fan (#684 E):** a convex corner of a revolved tool's
profile — two faces whose outward normals there turn more than `LEAST_TURN` apart, the chord just
inside them in the material (`constant_twist::corners`, by the probe T2 reads too,
`SweepContacts::corner`) — is a piece of the walk of its own
(`Piece::Corner`): its one ring the circle the corner turns to (`RevolvedSurface::ring`), its
stations the fan of normals turned from the first face's to the second's, each station's roots
those of the same `A cos θ + B sin θ + C = 0` with the fan normal's coefficients
(`RingFrame::coefficients`, which a face's ring is read by too), solved as a face's ring is
(`RingFrame::roots`, `sinusoid_roots`). Under any constant twist (a screw,
a turn or a slide) the walk, at a face's end where the next face's ring has no root at the same
point, runs onto the fan and round it to the next face (`step`); the rows S1–S4 are asked of the
fan's nodes as of any. Where both faces' characteristics meet a convex corner at one point, every
fan normal is in contact there and the sweep has a sharp edge along that point's path: S1 refuses
it, saying so, as it does a concave corner's jump. The sheet's rows are laid by length along the
section plus each turn of its normal counted as the section's length over a half turn, so a fan
(the rim of a 60° V under a thread's screw: 0.06 mm turning 100°) is as many rows as its turn asks;
the rows where the walk runs onto and off a fan are exact rows of the grid (`Characteristic::fan_ends`,
kept by refinement), and the fitted sheet is cut there into faces (`Fitted::named`, the bands
unnamed: a fan is no face of the tool). Kernel rule it found: a near-crease in one fitted net lets
a point of one flank be inverted onto the other, a pcurve off its edge by a tenth of a millimetre
and a split with no cell; cut into faces, each inverts on its own. And a second: a plane meets the
fan in a curve shorter than a march's step (0.09 mm against `h0`'s 0.22), which a trace keeps
round the rim's tip and traces once ([brep.md](brep.md);
`object_builder::a_sharp_rims_fan_builds_at_the_displays_samples`). `Characteristic::fans` names
the corners walked. `solid_sweeps_examples.rs` (`vthread.sv`, a V wheel ground along a helix, its
section computed apart from the core) and `turns_and_slides.rs` (a V wheel slid tilted, its shadow;
slid square, refused at S1 for its sharp edge) are the gate, each STL held to the field.

**A drafted prism tool (#684 G):** a drafted prism is in the generating class where each corner
joins two lines or goes on tangent: then each side (each half of it, either side of the parting
plane, where the prism runs both ways from it: below) is
the ruled surface between its edge's two end sections, a plane or a cone (`ExtrudedSurface::drafted`,
from the field's `Sections`; `Sections::linear`), its points' normals their own, so its contacts are
read as an undrafted side's and every row is asked of it. T1 refuses the rest by name: a prism
drafted outward both ways from its plane (its sides meet there in a concave crease), an arc meeting
a side at an angle, a side no line, arc or circle. The cutter sections it at a station as the blend of its two end sections
(`cutter::Half::ends`, from `build::offset`), walked by the profile's lengths so a walk length
is a ruling at every station (by its own, a shrinking section moved where contacts leave the roll's
window and ended the chart in the blank); the sheet is traced, never the planar envelope extruded
(`Cutter::prism_profile` is undrafted only). A drafted tool the generating rows refuse under a
motion keeping its plane is still the planar subtraction class's; the native path refuses a drafted
prism by name (`--kernel rust` builds it). `tests/drafted_tools.rs` and
`solid_sweeps_examples.rs` (`helical.sv`, a rack tooth leaned 15° across the blank and drafted 2°,
which no planar class reads) are the gate: the drafted rack's and the helical rack's spaces within
2e-4 of arithmetic made apart (each section of the blank cut by the tooth's quadrilateral there,
swept by `rack_cut`'s arithmetic), each STL agreeing with the field; a drafted slot's planes and
cones on the tool's field; both T1 refusals.

**A drafted prism running both ways from its parting plane (#684 P):** its sides fold at the plane,
each side's two halves meeting in a convex crease across it at the profile itself, where every
normal between theirs is the tool's: a fan, as a convex corner's is, that runs along the parting
section. `ExtrudedSurface::drafted` reads it half by half (each half its section at the plane and
at its own end, `caps` saying which of a side's ends lie on a cap), and `SweepContacts::creases`
pairs each side's halves. Admission asks each half as a side; a half's end at the plane is no rim
(T1 asks caps by their rims, `ToolSurface::meets_cap`); and the crease is asked as a rim is: each
half's envelope stepped onto it and its fan stepped off running the same way (E3; a fan whose
normals all touch at the half's instant, as under a motion keeping the profile's plane, carries
the contact nowhere and steps no way), every fan normal touching the blank once (E2), the fans'
contacts joining E4's. The cutter's stations cross the crease between the halves
(`cutter::Drafted`, `Ordinate`): a station is the ordinate below the plane, its fan's turning laid
as `CREASE_SCALE` (a millimetre a radian) of stations, and the ordinate above it that much
further, so the sheet's columns are laid through the fan as through the extrusion; a fan laid much
longer than the contacts it carries move in space (20 mm a radian) left columns on one curve and
folded the fit. Drafted outward both ways, the crease is concave and T1 refuses it, as does the
cutter. `drafted_tools.rs` and `solid_sweeps_examples.rs` (`parted.sv`, the helical tooth drafted
2° both ways from the blank's mid-plane) are the gate: each space within 2e-4 of arithmetic made
apart (each section of the blank cut by the tooth's eight half-planes, the material below the plane
and above it of each side, swept by `rack_cut`'s arithmetic over pieces of the blank split where the
crease passes the profile's corners), the rack's unleaned tooth too (a motion keeping its plane,
the fan sweeping nothing, the halves' envelopes meeting in a ridge), each STL agreeing with the
field; `object_builder::a_tooth_drafted_both_ways_builds_at_the_displays_samples` builds it in the
page.

**A union tool is its members (#684 C, #586):** a body cutting the sweep of a union — its tool a
body that only puts solids on its stock, a union within one, or a placement of one — is admitted
(`admit_body`), built (`brep::export::own`, the page's `Builder`) and metered (`accuracy::Meter`)
from a copy of the drawing with that cut written as each member's (`solid::distributed::lowered`):
one sweep a member, minted once however many placements cut the union's, under its motion, the
placements between minted again over each, every minted solid keeping the one it copies' name.
`Sketch::distributed` sends each member's sweep back to the union's, so its roll is the union's
(`roll::interval`) and its cut faces are named by their routes in the union
(`sheet::SweptCut::read`); a refusal says the member (``its tool's member `bead2`: …``), and a
union whose every member is a ball's tube admits nothing and builds as static material, held to
the union's field as a swept body is. Not distributed: a union within an intersection or a cut
(T1 refuses it, saying so), and a union of prisms under a motion keeping their plane, which
`planar_subtraction::asks` reads whole. The native path refuses a distributed cut by name.
`tests/union_tools.rs` and `solid_sweeps_examples.rs` (`union.sv`) are the gate: the beads' groove
against rows across the rod computed apart, placed and turned, against the long form, two balls'
tubes by Pappus, two tori rolled in the generating class, and the refusals.

**A prism's cap in the blank (#684 F):** a prism standing alone as the tool (its solid the prism
itself, undrafted, its profile one loop of lines and arcs: `admission::Lone`) whose cap passes through the
blank is no longer refused at T1; any other tool's cap is, saying so. Its caps are faces of the
tool (`ToolSurface::Cap`, read over the box its profile lies in, a point off the profile passed
over as the field says), asked M2–E4 and E1 with its sides only where one reaches the blank; M2
refuses only a point in contact at every time (a constant nonzero condition touches nothing — a
side square to a rack's line of centres). At each point of a reaching cap's rim, where a face's
contact in the blank meets the rim, the face's envelope stepped onto the rim and the rim's fan
stepped off it must point the same way, or E3 refuses the fold (a flat end swung so its corners
dig past it), and the fans' contacts join E4's. **T3** then chooses the direction across the
profile, of 36, whose sections through the extrusion keep the band the sampled contacts in the
blank reach farthest from the profile's ends (where a section closes to a ruling) and its corners
(where a section's side leaps from one face's normal to the next's, a crease of the sheet), and
refuses where that is less than a quarter of the band and two of the reach's 96 stations either
side (`Cutter::spread`, `Lone::sections`); the evidence is `Class::Capped` (`Capped`: the
direction, the band, the clearance). The sheet (`sheet::capped_sheet`) is traced at those
sections (`Cutter::across`, `Kind::Across`): each the profile's chord carried along the extrusion
— the low cap's chord, the far ruling, the high cap's chord, the near ruling — its four corners
the caps' rims, convex, a radian of their fan walked as a millimetre (`RIM_SCALE`, so the rows
step through a rim a radian of the motion at a time and `charted` keeps them), every chord
walked as long as the band's middle one so a tapering profile's stations start each piece at the
same walk length. The rest is the generating class's: fit, split, cells, agreement. The native
path refuses a capped sweep by name. `tests/prism_caps.rs` and `solid_sweeps_examples.rs`
(`bar_end.sv`, a tapered bar's flat end cutting a tooth space's floor under a rack's roll, each
height's space read apart from the core by `rack_cut`'s arithmetic and held to 1e-6 of the part,
2e-6 of the space at 0.002 mm) are the gate, with E3's, T3's and T1's refusals;
`object_builder::a_bars_end_builds_at_the_displays_samples` builds it in the page.

**Every class has its simplest case in `rust/examples/solid_sweeps/`** (#684): one short document
a class — the canal (turned, and along a spline), a slide, a turn, a screw, a screw with sharp edges, the generating class's rack, its
drafted helical rack, a tooth drafted both ways (`parted.sv`) and a prism's cap (`bar_end.sv`), the planar subtraction class's conjugate wheel, a union's members — its README row naming the class, and `tests/solid_sweeps_examples.rs`
holding it to its class, to an exact build whose STL agrees with the field and to a volume made
apart from the code under test. A class admitted later adds its file, its row and its test there;
`every_example_is_held_and_named` fails a file the README does not name or no test reads. The
planar generating class's simplest case is the Wankel (`rust/examples/wankel/`).

**A swept body placed is the body moved (#619):** a chain of placements (`shown := wheel in held`,
`x at(under: f) τ`) ending in a body with swept cuts is that body built once and its boundary carried
by the chain's pose (`cad::placed_swept`): `brep::export::exact` moves the B-rep, its pattern
(`pattern::Built::moved`) and the turn its field agreement folds by, the page's `Builder` moves its
build (`sweep::Built::moved`) before the body rule takes it as the operand it stands for, and
`admit_body` admits the body placed. Its volume is the body's to rounding, and a body made from it
(`copy := solid(shown)`) is composed over it. A report asked of a swept solid supplies its exact
B-rep whether or not it is an object (`supply_exact_where`; a drawing's `supply_exact` keeps to
objects), so the body and its placements read one B-rep and no field: a placement's is the body's
own supplied B-rep moved (`brep::export::supply_one`), never a build of its own, so a claim that
places a wheel at thirteen poses builds it once (the radiator bot's full check, 156 s to 55 s).
`tests/planar_subtraction.rs` (`a_wheel_a_frame_stands_is_the_wheel_moved`,
`a_placed_wheel_is_supplied_as_the_wheel_moved`).

**The Wankel, the planar class (#65):** a point's planar envelope is its path (`bore :=
envelope(apex, under: rotor_turn, …)`); `generate::chosen` skips a stationary root. A generated
curve over whole turns of its motion is closed (`Sketch::curve_closed`), a face's lone loop, split
at its middle for the recipe. The rotor is a cut: its blank less the housing's wall swept over the
period (`common` with a sweep keeps the union's, another set). A pocketed prism under a planar motion
is admitted by rows P1–P5 (`solid::planar_class`): the pocket's inner envelope over a whole period
(`envelope::planar::InnerEnvelope`, over a formula or generated source: contact roots per sample,
kept where inside every pose, corners where an image recurs), built as the blank in common with
its prism (`brep::sweep::planar`), metered as envelope × slab (`accuracy::Planar`). `claim over
MOTION` advances the placements under it (`Sweep::motion`). A profile field's walls are boxed
(`Boxes`); `ssi` cuts an extrusion square in closed form and `boolean` gives its iso edges straight
pcurves. `rust/examples/wankel/`, `fixtures::wankel`, `tests/{bore,planar_envelope,wankel}.rs`.

**Planar subtraction (#606, #607, #590):** a sweep of prisms under a motion that turns their
plane in itself is the blank less the region its section sweeps, slab by slab
(`solid::planar_subtraction`, rows P1, Q1–Q4 in [docs/generating-sweeps.md](../generating-sweeps.md)).
`admit_body` asks it of a prism tool the generating rows refuse at T1, T2, E1 or E2 — a union, or a
draft the generating class does not read, in the tool, a concave corner, a roll never clear of the
blank, a second contact — so a rack
is still built by its sheet (`extruded`), and a rack tooth with a cap in the blank, or united with
itself, by its region (`rack_cut::a_rack_with_a_cap_in_the_blank_cuts_the_slab_it_spans`,
`the_tooth_united_with_itself_cuts_as_the_tooth_does`: the two classes agree to 1e-5). The slabs are the blank cut at every member's cap within
it, two next to each other with one section (every member undrafted, the same faces joined alike,
`Subtraction::key`) read as one. In each, `envelope::swept_region` reads the region `R = ∪ₜ
poseₜ(section)` within a box about the blank's shadow at every placement: its candidates are
each line's contact (the foot of the perpendicular from the instant centre, `ν·(ωJx + b) = 0` in the
tool's frame), each side of an arc's (`e ∥ ωc − Jb`), each corner's path while the line from the
instant centre lies within its fan of normals (its sides read off the section, so a corner where
members meet is the section's), and — over a roll that is no whole period — every edge held at each
end pose; a point is a candidate only where it lies on the section's boundary at its own pose
(`Kind::at`), and kept only where it lies outside the section at every pose (`Keep::outside`,
`envelope::planar`'s test turned round). Kept stretches are found on a grid of a quarter degree,
their ends bisected, and one of no length (a sample kept where its candidate's domain ends, its
ends and middle one point) dropped (#612); walked with `R` on the left they join where they cross (solved together,
`meet`), run on into each other at the same roll, meet at an end pose (the contact on the held
edge, exactly: tangent there, so the keeping test finds the edge's end up to `√(2 tol ρ)` off it,
`ρ` the envelope's radius, and the two join within that for `ρ` up to eight times the section's
reach) or leave the box, which carries the loop round to the next stretch entering. Where
several meet at one point (an end pose's corner: the held edge, the corner's path, and a hair of the
next edge the keeping test's tolerance keeps), the union's boundary takes the rightmost turn, and a
hair joined at neither end, or only to itself, is dropped (#613: a placement turned 30° boxes the
blank past a rack tooth's tip, which no quarter turn does), an edge held at an end pose as long as
the join's reach there (a flank whose contact left it at its corner just before the roll ended, kept
from the corner as far as the tangency lets the keeping test). A
whole period is where the motion takes the section onto itself (`Setting::alike`: the tool's
symmetry counted, the star's 540° at 4:6), and a roll that is neither one nor clear of the blank at
both ends is refused naming the period. `brep::sweep::subtraction` builds each
counter-clockwise loop and the clockwise ones within it as a profile (each stretch a cubic B-spline
fitted through its exact points within `FIT_MM` or the tolerance's fit, a ring split at its middle)
carried through its slab, a hair past the blank where the slab ends at its face, and cuts it from
the blank at every placement; a drafted slab (#607) is read at `STATIONS` heights and lofted
(`build::lofted`, a B-spline surface a stretch, cubic along it and across the stations) through
every other one, held to the ones between within the fit's bar, its loops matched station for
station (`SweptRegion::matching`). Admission reads its stations within the box grown by each of
`GROWN` in turn until they are alike (#612): the box's sides lie outside the blank, so a change of
shape only where they cut the region is the box's — the measured handle's hub ring, narrowing
with the draft, crosses them 48 mm about the wheel's centre. A change in every box is the blank's
own, refused at Q2 naming the drafted member's depth to step the tool at, found between the two
stations that differ. `accuracy::Subtracted` meters it, reading each sweep by the class's own
`planar_subtraction::admit`; a `through:` roll never clear of its body is refused saying the period
to state. Every period is found by one search (`roll::first_alike`: a degree at a time, none for a
tool the motion does not move) and said in one form (`roll::advice`). `tests/planar_subtraction.rs` is the gate
(the #585 wheel against arithmetic made apart, the long form, two bands as one, a stepped tool,
one arm through, Q3, the drafted wheel at 10 µm, the measured drafted handle above six bands
stepped down it and the square arms, a drafted disc whose islands change refused at their depth,
#613's pinion cut at a copy turned 30° and whole, and at module 1.5 over a roll ending as its flank
grazes the held one or its contact leaves it at its corner just before, stated and `through:`).

**Export tolerance ([plan](../native-hypoid-plan.md)):** `solventc --tolerance [LENGTH]` (default
0.01 mm; bare number in document units); `solid::export::Tolerance` states every bar once. Sheets
pass within half of it of withheld contacts (`contact_trace::Withheld::Sides`), normals within
`Tolerance::turn`, refined where they miss (`marked`, `Grid::refined`, ≤4 times, 480×400). Fit:
`cad/native/sweep_boundary/fit.rs`, `surface_feet_near`. STL meshed until `mesh_sag` is within
half (OCCT's deflection is no bound), float32 rounding counted with it (`mesh::stl_rounding`,
`brep::export::written_within`: far from the origin refused, #60); the mesh contract counts clusters
(`TinyTriangles::clustered`); the field is probed at max(2 tol, 2 µm).
`--measure FILE` checks an export (`solid::accuracy`) and exits 1 over tolerance.
**An empty body is refused as one** (#617): a body built with no face is what the document drew,
not a limit of the encoding, so `brep::export::exact` refuses it, whatever path built it, saying
which solid is empty and which statement emptied it (`` `shared` is empty: `b common shared` leaves
no material, so there is nothing to write ``, or its stock holding none) — `combine` checks after
each Boolean and builds the body empty with the statement that emptied it (`Exact::emptied`), and a
static body found empty is combined again a statement at a time to name it (`emptied`) — never a
float32 rounding read off its infinite bounds, nor a STEP file of no faces.  The refusal is the
file's alone: a drawing given the same build (`supply_exact`) reads a body with no material, its
volume 0 (#618).
`an_empty_body_is_refused_naming_the_statement_that_emptied_it` (`tests/brep_export.rs`).

**One sector, sewn round ([plan](../native-speed-plan.md)):** a body whose swept cuts are turns
of one placement about one axis (`solid::sector::indexing`), from a blank alike under the turn, is
built as one sector (`cad/native/sweep_boundary/sector.rs`): its side runs midway across the gaps
between neighbours' contacts (`sector::Boundary`, sliced by spheres about the blank's centres or
planes square to the axis — a spiral tooth space turns most of a pitch, so no flat half-plane
clears it), is read back against gaps and field, and with its pitch turn splits the blank (remade
about the axis, `solvent_cad_revolved`). Its material is turned and sewn (`solvent_cad_pattern`, no
face intersected); a ring's pieces are moved into one period and split at the seam so rings close
on an iso line. A failed premise builds whole and says why (`SOLVENT_SECTOR=off` forces it,
`SOLVENT_SECTOR_DEBUG` narrates). Faces and volume are the whole construction's; bytes are not.

**Speed (phases 3–4):** the STL is the sector's mesh turned, seam points shared bit for bit
(`solvent_cad_sector_stl`; `SOLVENT_SECTOR_STL=off`); pattern and STEP are measured as copies of
one (`SOLVENT_SECTOR_CHECK=full`, `SOLVENT_STEP_CHECK` measure whole). Volumes, admission,
sections, splits, cells, probes and agreement run on every core (`gcs_core::par`, serial on wasm,
answers in order; `progress::side_by_side`/`beside`/`under`). A revolved cutter is sectioned once
and turned. OCCT's parallel `BRepCheck_Analyzer` is unreliable: `valid_solid` checks faces in
separate analyzers. **Phase 5:** admission proves placements alike from the solid graph when every
blank operand is a full revolution about the indexing line or a ball on it
(`admission::Equivalence::Revolved`, 1e-12 of the blank; else `Sampled`); such a blank is its
meridian section turned once (`Session::construct_meridian`, `solvent_cad_revolve_region`;
`SOLVENT_BLANK=booleans`); field agreement probes one sector, reading cuts not proved positive
(`agreement::Sector`, `MaterialField::without_cuts`; `SOLVENT_AGREEMENT=whole`); STEP text is
formatted on every core, the kernel's to the byte (`SOLVENT_STEP_TEXT_CHECK`).

**Speed of the Rust kernel's swept cuts:** every shortcut reads what the long way read. A placement
is compared with the first by `SpatialField::below` (`value(p) < t`, a Boolean's second operand
unread where its first decides, a loop's nearest edge sought only within the level, the parity of
its ray first). A body built whole is split by its placed sheets a batch at a time, each sheet in
the first batch whose boxes stand clear of its own (`sector::apart`): sheets that meet none of each
other split as one. A cell's interior points (`query::interior`) are the deepest first by a
hierarchy of its mesh's triangles, the winding number asked only of the points the choice reaches.
Under these, the class's Booleans are seeded and traced as any are (brep.md), a cone's step read at
its radius in the pair's box (`Surface::feature_within`, not at its frame's origin, which may be
its apex), and a spline's foot found as `brep.md` says. A blank's prism of whole circles about the
line, along it, is turned as its section (`Recipe::meridian`), so a chamfered gear's blank is made
again about the axis and built as one sector; a side running in to its bore is clear of the cuts
by its gap's angle times its radius, or by how far nearer the axis than every contact of its slice
it runs where that is more (`Boundary::clearance_of`). `tests/swept_speed.rs` and
`sector::a_side_reaching_its_bore_is_clear_by_its_radial_gap` are the gates.
