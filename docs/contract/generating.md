# Motions, generation and swept export

Part of the [contract](../../CLAUDE.md) the code is held to. Read it when you touch `motion.rs`,
`generate.rs`, `solid/admission.rs`, `brep/sweep`, the kernel's `trajectory.rs` or `contact.rs`,
or the native export. An issue number (#n) or a
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
be meshed; `mesh::stl_shells` checks every component, cavities included, unwelded. Where it refuses
triangles with no area as written (`mesh::flat_triangles`), the core's export says where they are
(`brep::export::flattened`, #821): the solid, the B-rep faces its flat triangles are of and the faces beside the first, a point
near it (the refusal's witness), how long and thin it is, whether the mesh or float32 flattened it,
and, under the bar, the likely sliver of two cuts ending all but flush. Nothing is dropped: a triangle
dropped opens the shell (`brep_export.rs::a_sliver_float32_flattens_is_refused_naming_its_faces_and_where`).
The reader's own refusal names the triangle's number and where it is. Failure keeps
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
prism cutter's sheet the same way: `brep::sweep::extruded` (the kernel's `brep::swept::extruded`,
#721 F2c, its cuts in closed form, `brep::swept::planar_cut`, held to `generate::cut_at`'s to
rounding by `generation::the_kernels_planar_cut_is_the_generated_profiles`; edges as lines/arcs, convex corners as
points over their fans, `generate::cut_at`, Hermite spans, extruded; `Family::in_frame`), else
traced (`sweep::traced` kept to compare). `tests/extruded_envelope.rs`, `rack_cut.rs`.

**A ball swept is its tube** (#238, #684 D, `solid::canal`; the tube is the kernel's,
`brep::swept::canal`, #721 F2c): a sweep whose tool's recipe is one
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
motion (`brep::swept::canal::along`): the disc a face family's one closed `Run::Round` wall over the roll, its
stations the centre's point and the radius vectors of the frame square to the path, carried from
the start by double reflection (`brep::trajectory::reflected`) once, over 4096 steps of the roll, any other
roll one step from its nearest (`Carried`), its pieces of the roll (Gauss–Lobatto,
`brep::family::weights`) each held to the disc read near both its ends within a face family's `EXACT` (`brep::family::EXACT`) of
the diameter, so the wall is fitted, and lies on a cylinder or a torus where its
numbers do (`brep::family`'s `analytic`); built as the family and the end half balls united
(`build::capped`). Its rows, sampled at points an eighth of the radius apart along the path
(1024 to 32768) and said so, refuse it by name with a point of the path, in the canal class
(`admission::Asked::Canal`, asked first by `admit_body` where no other class reads the motion):
**B1** the path never stops and turns nowhere tighter than the radius; **B2** it comes back within
the diameter (and the points' spacing) of itself nowhere half a turn of a circle of the radius
along it from (`canal::refusal`).  The kernel refuses by `Row` (B1 `Curvature`, B2 `Return`) and
what it saw (`Seen`: where the path stops, the radius it turns on, how near it comes back), with a
point of the path and how many points it read; `solid::canal` says that as the rows always said it
and reads the tool's recipe as a ball, lowers the motion and gates what the kernel builds. The recipe's `"canal"` node (`centre`, `radius`, the ball's
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

**A motion's sweep is a set** (#737 P4, [0.199], [sets.md](sets.md)): `solid({ p | some t in (a,
b): p in tool at(under: m, seen_from: g) t })` ([0.205], lowered to `p inside(under: m,
seen_from: g, over t in (a, b)) tool`) — what the solid `tool` occupies under `m` at some roll
of the interval — is read by the parser as the motion sweep of the same points
(`syntax/parser/statements.rs::swept_ahead`/`swept_set`, its body one statement saying a sweep,
its parts read where they stand, no token rewritten)
and built as `SolidDef::Swept` with a stated roll; `over t` reads a declared parameter's interval.
With no `over` at all, `solid({ p | some t: p in tool at(under: m) t })` is what the tool occupies
at *any*
roll, the whole roll (`Sweep::Rolled`, `Roll::Through`), which **names no body** (#746): the
evaluator bounds it by its context, the material of every body it — or a copy of it placed at a
roll, `tool at(under: m) θ` — is cut from or kept in common with (`solid::applied_to`, the walk
`solid::uses`), so one sweep rolls through any number of bodies, and a pattern's copies cut from
one body read it once; cut from none, it is E080 at its declaration (`program/solids.rs`'s body
rule), and a tool always in mesh with the bodies is E080 asking for a whole period.
`sweep(…)` is retired (E001), the message naming the set it sweeps; the canonical printer
(`syntax/print.rs::sweep_text`) writes the set.  `tests/solids_as_sets.rs` is the gate.

**A sweep's roll through what it cuts** (#271, §6.9 [0.88]): the whole roll (through 0.198
`sweep(tool, through: body, under: m)`) is `SolidDef::Swept { roll: model::Roll::Through }`
(`Roll::Stated` is `over t in (a, b)`, an interval of the shared roll, `Running::roll`:
[parameters.md](parameters.md)), the roll about 0 during which the tool meets the material of the
bodies it is applied to, as they stand, one blank of them all (`document::applied_material`:
their stock and additions, never cuts, so `removal` cutting `body` is no cycle;
`evaluation_operands` reads them; a body cutting a member's sweep a union's is distributed into
cuts the union's, `distributed::union_of`, so the lowered copy reads the roll the drawing found).
The bodies are read as they stand, not carried back through a copy's placement as a prism through
everything's span is: right for a pattern turned about an axis its blank is round about, and a
gap for a copy placed elsewhere (the field has no inverse placement yet). **Every
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
surface). The march goes two turns each way (`roll::REACH`); under a screw or a slide it goes
instead as far as carries the tool's support wholly past the blank's along the axis, and a way it
has not proved clear sooner ends there (`roll::escape`: clear by the supports alone, so never
short), so a blank however long along a screw needs no roll stated (`square_thread.sv`). Refused naming why
(no meeting at 0, not clear within two turns, a tool with no bounded support). The spiral bevel,
the rack cuts and the lantern state no roll; the drill's flute and the Wankel's period are design.
`fixtures::gear::roll` restates one member's roll (negative controls).
`tests/roll_through.rs` is the gate (two balls parting at `2 asin(1/3)`; the square thread's bar
clearing its rod three and a half turns up; the pair at 10:48 admitted;
in the slow tier each member built as its stated roll built it).

**Generating sweeps and the native export** ([docs/generating-sweeps.md](../generating-sweeps.md)):
`solventc --step/--stl` builds swept cuts only for the bevel/hypoid generating class, the
constant-twist class (screws, turns and slides) and the planar classes (below).
`solid::admission::admit_body` checks rows T1–E4 (asked of each placement by the kernel,
`brep::swept::admission::check` and `check_screw`, #721 F2c; the solver says each `Row` as a
`Condition`) (revolved line/arc tools, rotation with changing
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

**The trajectory and the contact are the kernel's (#721 F2):** a motion reaches the kernel as a
`brep::trajectory::Trajectory` that keeps its structure as data — a constant twist exactly
(`Twist`, a `Screw`: a screw, a turn or a slide), one motion seen from another (`Relative`,
`observer⁻¹ · source`), a path taken at a pace (`Paced`, whose sweep is its path's over the pace's
range, `Trajectory::unpaced`, #684 L), a path of pieces each over its stretch of the parameter
(`Pieces`, refused by `Trajectory::pieces` where two do not meet, K1), and any other rigid motion
evaluated as a pose and its rate (`General`, K2).  `Family::trajectory` lowers a family step by
step: a turn, a screw or a slide to its constant twist, a driven joint to that twist taken at its
law (`Pace`), a relative to its two, a frame along a path or one the drawing moves to what it
evaluates to; its pose and rate are `Family::at`'s at every time, and its constant twist is
`Family::twist`'s wherever that reads one (`tests/trajectories.rs`,
`a_motion_lowers_to_the_kernels_trajectory`, over every motion of the sweeps', the generation's,
the mechanisms' and the paths' examples).  At each time a trajectory's twist is `Twist`, the field
`v(x) = v₀ + ω × x`, read in the world (`Twist::spatial`) or on what it carries (`Twist::body`, in
which a carried surface's contact is the world's envelope equation,
`a_carried_surfaces_contact_is_read_on_the_surface_as_written`); a screw's is the same at every
time (`a_screws_twist_is_its_poses_at_every_time`).  **Every kernel `Surface` answers its contact
under a twist** (`brep::contact`): the contact function `n̂ · v` (`Surface::normal_velocity`), and
in closed form where one exists (`Surface::contact_form`) — on each ring of a surface turned about
its frame's `z` (cylinder, cone, sphere, torus, revolution) `A cos u + B sin u + C`
(`ring_contact`, two roots, a double one, none or all round: `RingRoots`), along each ruling whose
normal stands still (plane, cylinder, cone, extrusion, a revolved line) `a + b v` (`ruling_contact`,
`RulingRoot`), and across a plane `a + b u + c v` (`plane_contact`); a blend, a B-spline and a face
family's wall are read numerically, the contact function's crossings along a line of parameters
(`contact_along`).  Each closed form is the contact function rearranged, held to it at sampled
parameters under seeded twists, each ring's roots to the numerical reading of the ring, and the
degenerate contacts the walk is built round (a ring in contact all round, a ruling all along, a flat
kept in its plane) to the root kind that names them (`tests/surface_contact.rs`).  Sampled.

**The constant twist is the kernel's (#721 F2b):** the walk of the characteristic and its rows
(`brep::swept::constant_twist`), the sheet fitted and judged (`brep::swept::sheet`: `Sheet`, `Grid`,
`settle`, `Fitted`, `Tolerance`) and the screw's sheet carried along (`brep::swept::helical`) are
in `cad-kernel`, on the tool as the kernel reads it (`brep::swept::tool::Tool`: its faces, a face of
revolution being `brep::swept::surface::RevolvedSurface`, its edges, creases and `Material`) and the
lowered trajectory; `constant_twist::…`, `helical::…` and `sheet::settle` below name them there.
`solid::constant_twist::walk` hands the kernel the tool and the motion `SweepContacts` read off the
drawing and says the kernel's `Row` as admission's `Condition`; `brep::sweep::helical` hands it the
cut's name, scale, trajectory and roll.  The readers of a tool's faces off a drawing are the
solver's, as extension traits (`ReadRevolved`, `ReadExtruded`, `ReadCap`, `ReadProjector`).  With
the move the solver's duplicates went: `Family::at`, `Family::twist` and `Family::instantaneous`
read the lowered trajectory (`Family::path`, lowered once when the family is read; `twist` its
`Trajectory::screw`, a turn of no ratio being the slide it is and one that moves nothing none), and
a ring's contact and its roots are `brep::contact`'s (`RingFrame::coefficients`, a `Sinusoid`; its
roots on a chart of the ring `Sinusoid::roots_over`), read off the motion's velocity field as the
walk always read them.  `Screw::at` reads its unit axis as it is, so a constant twist's pose is
`Family::pose_at`'s to the bit (`a_motion_lowers_to_the_kernels_trajectory`).  The pose alone and
the speed bound are the trajectory's too (#721 F2c): `Trajectory::pose` (a pace's `Pace::value`, a
path's `Path::pose`) and `Trajectory::inverse_point_speed_bound(_over)` (a pace enclosed,
`Pace::enclose`; a path by its `Stretch`, `Path::stretch` and `Path::speed_over`), each read off
the tree the motion lowers to with the step-by-step reading's arithmetic, so `Family::pose_at` and
`Family::inverse_point_speed_bound` read the lowered trajectory and their step-by-step copies went.
Held as the moves are: every pose (2,862,428 distinct readings) and every point's speed bound
(1,763,988) the core suite reads, before and after, the same to the bit.  The move was held
walk by walk, as F3 was: every walk and every screw's sheet the suite makes written down at full
precision before and after, the same to the bit.  (Read through `Twist::body` and `Sinusoid::roots`
instead, the walk moves by an ulp, and that alone turns three triangles of the whole turn's mesh over
(`the_wheel_turned_once_leaves_its_shadow_all_round`): the mesher's, F4's to mend.)

**The tool read from its B-rep (#721 F1, #828 F1a; `brep::swept::read`, `Tool::read(&Brep, scale)`):**
the kernel reads a sweep's tool off the boundary it builds, by its faces' surfaces and the edges
they share, never by how it was drawn.  A face of revolution spanning a whole turn is read about
its own axis (`Cylinder`, `Cone`, `Sphere`, `Torus`, a `Revolution` of a line or a circle), and so
is a plane bounded by circles about one centre (#684 O).  A face whose one loop is two lines or two
coaxial arcs joined end to end by two lines is a strip (`ToolSurface::Extruded`, `Side::Straight`
where the far edge is the near one carried, `Side::Leaning` where not).  Strips sharing a ruling
are read one way round, each the way its neighbour reads the ruling, so a box is a prism one way of
three.  A face of revolution trimmed to part of a turn is its surface's whole turn, and a plane
sharing a circle with a revolution is a flat of it, one flat for each axis it shares a circle
with; in both cases the material tells the face from the rest, as the tool keeps the faces a
Boolean hides.  Any other plane is a cap, at the strips' far end where it meets a far rim.
Corners (`PatchEdge`) are two readings sharing an edge at an end of each one's meridian or edge,
and a meridian closed on itself meets itself at its seam.  Creases are two strips sharing a rim,
the one's far and the other's near, their rulings going on across it.  Each revolution about an
axis starts from the half-plane the first face about it does: its frame's `x`, the profile's
half-plane for a face a revolution built.  The walk reads two faces' corner there and starts round
a ring there.  The material is the boundary's (`read::Boundary`): the distance to the nearest
face's foot or edge, a seam bounding nothing.  It is signed by that face's outward normal, by the
edge's faces' summed normals inside an edge, and at a corner by a mesh of the boundary, or by its
point location where the point is within the mesh's sag.  A face no reading takes is refused by its
geometry with a witness on it (`Unread::FreeForm`, G10: a B-spline, blend, face family, extrusion
or affine image; `Unread::Meridian`, G8: a revolution of another curve; `Unread::Unfit`).  Nothing
switches to it yet: the classes still read the tool off the drawing (`SweepContacts::read`), and
`tests/tool_reading.rs` holds the two readings to one tool over every sweep the drawing's reading
reads, as an oracle until F1b switches.  The gates are these:
- `the_two_readings_bound_one_tool`: each point of either reading's faces on the tool lies on a face
  of the other of one kind, square to one normal.
- `the_two_readings_are_one_material`: the two materials agree in sign off the boundary.
- `the_two_readings_turn_their_corners_alike`: sharp corners and creases are each other's.
- `the_two_readings_walk_alike`: under a constant twist the walks through the blank agree node for
  node to rounding, or are refused by one row.

These run over the sweep catalogue.  `every_examples_tools_read_alike` is the slow tier's, over
every example: 67 tools, the hypoid crowns standing about two axes on one plane, the twist drills.
The B-rep reading reads more than the drawing's in two places.  It reads a drafted prism's caps,
which the drawing's reading leaves to admission (`swept::admission::ends`).  It also reads the
faces a Boolean of the tool splits, and their corners: a union of two beads is refused at the
concave corner where they meet, where the drawing's whole tori are refused by a ring in contact all
round.  `a_face_no_closed_form_reads_is_refused_by_name` holds the Wankel's epitrochoid bore to
`FreeForm`, its witness on the face.

**The swept solid closed (#721 X0–X6) is designed in [docs/swept-solid-design.md](../swept-solid-design.md)**:
every candidate sheet a patch over an explicit domain cut at its branch and fold loci, a box holding
the sweep split by all of them, each cell classified by the kernel's membership test
(`brep::swept::member`, the definition read without the drawing: the point carried back lies in the
tool at some roll, by branch and bound over the trajectory's speed bound, `Unsettled` near the
boundary rather than guessed), the kept cells the closed solid; X1–X5 its cases.  Built
(`brep::swept::closed`): a polyhedral tool slid (a box aslant its zonotope to 1e-9, an L prism its
blocks' sweeps united), or turned where its edges run along the axis or square to it (a bar a quarter
turn beside the axis its polar integral to 2e-15), by the arrangement; and a **convex polyhedral tool
turned whole, however it leans, as its solid of revolution with no Boolean** — each slice convex, so
its radii one span `[ρ_min(z), ρ_max(z)]`, `ρ_max` reached on an edge (a hyperbola, its meridian a
rational quadratic) and `ρ_min` on the axis, a face's contact line or an edge, which one fixed
between the heights where two reach as far (`ρ²` quadratics equal or touching) or the axis crosses a
face; the profile cut there and turned once (`tests/swept_closed.rs`:
`a_block_leaning_to_the_axis_turned_whole_…`,
`a_block_straddling_the_axis_tipped_two_ways_turned_whole_…`, each the integral of its slices' radial
spans to 1e-6 and agreeing with membership).  **A leaning tool turned part of a turn** builds by the
arrangement, its edges' hyperboloids and faces' contact cones among the candidates: **one
arrangement of the box and every candidate, one selection by membership** (#735,
`brep::boolean::arrangement_within`, `Complex::select`) — no separator, no order of cuts, no Boolean
a cell, no union. A plane meets a hyperboloid of revolution in closed form (`brep::ssi`: an
ellipse or a hyperbola's branches held to the meridian's heights, as rational quadratics, or the two
rulings a plane holding one meets it in; `brep::surfaces_meet_in_curves_on_both`), and the
candidates' touches are the arrangement's (brep.md's tolerant rules: a touch laid as an edge, a
crossing where two meetings are tangent made one vertex, a piece with no inside merged into its
edge) (`a_block_leaning_to_the_axis_turned_part_of_a_turn_is_its_slices_swept`, slow tier: the volume
its slices sweep, each circle's arcs within a slice carried through the turn and united, to 1e-6, and
membership; built at 1e-6, and at 1e-7 in
`a_block_leaning_to_the_axis_turned_part_of_a_turn_holds_at_a_tenth_of_the_tolerance`).  X1–X5 are the pipeline's cases, not rows of their own: under a turn, a bar turned
through all but its own width meets itself, its end poses' faces and its edges' cylinders burying
one another where they meet (X5, X1; `a_bar_turned_through_all_but_its_width_meets_itself`, its
slices' arcs carried and united round the circle, to 1e-8), and an L-shaped prism's concave edge's
sheets cross and are trimmed (X3; `an_l_shaped_prism_turned_is_its_blocks_sweeps_united`, its two
blocks' arcs united, to 1e-7), each agreeing with membership.  **A bar under a screw** builds by the
same arrangement where its edges run along the axis or square to it: its faces at the end poses,
its edges along the axis and its side faces' contact lines as coaxial cylinders over the heights the
roll lifts them through, and its level edges' helicoidal sheets as open faces reaching past a box
square to the axis and as tall as the sweep (`swept::closed::screwed_sheet`). Its inner face at the
end pose holds a ruling of its inner level edge's sheet and touches it at the edge's foot, where
the cylinder of the foot's radius touches both: each pair met in closed form (brep.md's screwed
meetings), so the touch is one vertex of three exact curves.  Held to its volume in helical
coordinates `(ρ, ψ, ζ = z − k ψ)`, where the screw is a shift in `ψ`: each helix meets the bar in one
interval, lengthened by the turn `Θ`, so the sweep is `V + Θ ∫ ρ (h + k Δψ(ρ)) dρ`, for a unit cube
two out `1 + Θ (3 + k)` — built at 1e-6 and 1e-7, to 1e-9, agreeing with membership
(`a_bar_under_a_screw_closes_into_its_helical_sweep`, slow tier).  **A curved tool under a slide**
builds by the same arrangement where its faces are planes, cylinders, cones and balls and its edges
lines and circles: each round face at both end poses (its solid cylinder, frustum or ball over the
stretch it spans), each of its rulings whose normal is square to the slide (on a cone, the roots of
`A cos u + B sin u = C`) swept into the plane tangent along it — a cylinder along the slide swept
along itself, one cylinder — a ball's great circle square to the slide swept into the cylinder of
its radius along its centre's path, and each circle edge where its faces meet at an angle (a
smooth one, a ball on a cylinder, fans one normal) swept into an oblique cylinder, its disc's
cylinder sheared along the slide (`Brep::mapped`), whose meetings are the preimage's closed forms
(brep.md's affine images). A candidate whose solid stands clear of the domain's box is left out
of the arrangement. Held to a convex tool's sweep `V + |D| A`, `A` its shadow along the slide: a
drum slid aslant (`π r² cos θ + 2 r L sin θ`), a frustum slid aslant (two rims' ellipses' hull, one
stretch from two circles' hull), a capsule slid aslant (`π r² + 2 r h sin θ`) and a drum slid along
its axis, each to 1e-9 and agreeing with membership
(`swept_closed::a_drum_slid_aslant_closes_into_its_shadow_carried`,
`a_frustum_slid_aslant_closes_into_its_shadow_carried`,
`a_capsule_slid_aslant_closes_into_its_shadow_carried`, `a_drum_slid_along_its_axis_is_a_longer_drum`).
The constant-twist class's curved refusals it covers under a slide are its admitted cases now: a
hollow mill's ring bottom (G6, `flat_faces::a_hollow_mill_slid_across_the_block_cuts_a_whole_slot`,
a whole slot exactly), an engraving bit's apex on its axis (G5,
`turns_and_slides::an_engraving_bit_slid_square_to_its_axis_grooves_its_shadow`, its shadow's
triangle within the rod's disc in closed form) and a drill beside a post it does not pass wholly
through the heights of (`rings_all_round::a_drill_beside_a_post_bores_its_hole`) and a lens
stopping in a post at both ends (`admission::a_slide_that_ends_in_the_blank_is_the_closed_classs`),
each to 1e-9; and under a turn a corner holding the axis and a cap of a tool no lone prism, refused
at C3 until a plane touching a cone narrowing along its axis was met along their generator
(`prism_twists::a_corner_holding_the_turns_axis_is_the_closed_classs`,
`a_cap_of_a_tool_no_lone_prism_turned_is_the_closed_classs`).
**A torus under a slide** builds where the slide runs along its axis or square to it, its contacts
whole circles: along the axis its equators, swept into coaxial cylinders; across it its top and
bottom rings, within their level planes, and its meridians square to the slide, swept into
cylinders along it, each touching the torus at its end poses round that meridian (`ssi`'s
`tube_along_torus`, in closed form). A bull-nose mill slid to a stop inside a block keeps of each
level its strip and its half disc (`flat_faces::a_bull_nose_mill_slid_to_a_stop_inside_the_block`,
X1, the constant-twist class refusing it, to 1e-9).

**A curved tool under a turn** builds by the same arrangement where its round faces stand about
axes along the turn's, or are balls: a cylinder's or a cone's two generators in the plane through
both axes turned (a cylinder or a cone about the turn's axis), a ball's great circle in the plane
through the axis and its centre turned (a torus), a circle square to the axis its own plane, and
each round face at both end poses. The touches they make with the tool's end poses are met in
closed form (brep.md: parallel cylinders and cones touching along a line, a ball in a torus's
tube), and a face's pieces leaving one point tangent are ordered by their curvature
(`boolean::pieces_of_face`, ways within `TANGENT_TIE` read again at one length). Held to Pappus: a
drum, a frustum and a ball turned 100° (`swept_closed::round_tools_turned_part_of_a_turn_sweep_their_sections_by_pappus`),
and cut from a ring sector the drum turned whole (`closed_characteristics::the_drum_turned_a_whole_turn_bores_the_whole_ring`,
the constant-twist class's S1) and turned to a stop partway (`closed_class::a_drum_turned_partway_into_a_ring_sector_stops_there`),
each to 1e-9.
**A closed build is never returned wrong, and fails in seconds (#777 V).** Every closed or cut
build is judged by its definition before it is returned (`swept::closed::judge`): at a jittered
grid over its domain and at each cell's own points past the one that classified it, the solid's
exact point location is read against the sweep's membership (cut from a blank, against "in the
blank and no member"), and a boundary turned inside out (volume below nothing) is caught too; a
disagreement is `Unbuilt::Wrong {at, held}`, the witness point, never a solid
(`swept_closed::a_solid_built_wrong_is_refused_at_a_witness`). Point location is certified for
every surface the class emits (`query::curve_surface`'s certified readings: a plane, a cone, a
torus and a conic turned as polynomials along the line, a level rim's sheet on one unknown, any
other swept or B-spline sheet by the three-unknown search, `util::isolate`;
`swept_closed::every_surface_the_closed_class_emits_locates_points_exactly`, no point wrong and no
search sampled), and a Boolean's edges by the same readings, settled by one rule (#780 R2,
brep.md's `query::settled`; a conic edge and a spline edge against a plane certified too). Each
build's work is counted in steps per stage (`util::meter`: arrangement, trace, ray, outline,
inverse, membership, judge) against `closed::BUDGET`; past it every metered
loop winds down and the build is refused as `Unbuilt::Budget {stage, at, what}`, the same way at
every run (`swept_closed::a_build_past_its_budget_refuses_by_name_within_it`) — a curve's
crossings' search among them (`meeting::crossings`, #780), whose polish charges the inverse map
and once walked every pair of near chords on, 23 minutes, after the build was stopped; it
polishes only from a pair of chords no neighbouring pair stands closer than, since every pair of
two curves running within reach of one another walks to the same place (#780 C1: a short arc
beside a traced meeting, 26,000 pairs and no crossing among them, ran the build's inverse budget
down); it polishes first from each curve's own ends, where an end on the other curve is the
meeting exactly, keeps the nearest of the polishes landing on one crossing, and leaves a pair
farther apart than the reach and its chords' sags (#780 C2: a cone's contact ruling's foot, which a
polish from chords crawled toward along two tangent curves and stopped 4e-4 short of). A stopped
build is told by its type, never its words (#780 R3): every phase between its
stages asks the one check `util::meter::go_on`, whose `Stopped` names the stage and becomes
`Unbuilt::Budget`; a bounded search says why it gave up (`util::isolate::GaveUp`), so a certified
reading stopped by the build reads nothing more where one past its most boxes, or a pair with no
form, is sampled; and a sheet's inverse map searches no further
(`swept_closed::a_build_past_its_budget_stops_by_one_typed_check`,
`roots::a_search_says_why_it_gave_up`). The run #776 saw stall for hours, a ball-end mill screwed
through a ring sector (a ball beside a side about an axis along the screw's, admitted), is cut to
the volume it removes apart from the kernel, in helical coordinates to rounding, within a
millionth, and to a Monte Carlo of the sweep's definition within four of its standard errors
(`swept_closed::the_ball_end_mill_screwed_through_a_ring_sector_is_cut_right`, slow tier, about
ten seconds of build; until #780 C1 refused by its inverse stage after three minutes); its inverse
stage given a thousandth of `BUDGET`'s, every other stage its whole, it is refused there by name in
seconds, the same way twice
(`swept_closed::the_ball_end_mill_screwed_through_a_ring_sector_is_refused_by_its_inverse_maps_budget`).
`SOLVENT_SWEEP_PROFILE` prints a build's counts, its meetings in closed form against traced per
pair of surface kinds and the slowest, its curve–surface searches certified against sampled, and
its deepest outline: a performance change cites it before and after. `SOLVENT_SWEEP_DUMP=<dir>`
writes a failed build's domain and candidates whole (`swept::dump`), replayed alone and reduced
to the fewest candidates that still fail (`swept_closed::a_failed_builds_arrangement_is_dumped_replayed_and_reduced`;
the loop is docs/testing.md's).

**A tool leaning to a screw's axis is built (#777 M5).** Each leaning edge, and each leaning
face's contact line, screws into a patch of a ruled helicoid over the segment and the roll a
twentieth past each (`closed::Cutter::Patch`), its rim standing inside a cell; two sheets of one
screw meet in exact helices (`ssi::screwed_lines_meeting`: the line's points the screw carries onto
the other sheet, isolated with a bound), never traced ones, whose ends leaning a few degrees off
once ordered the curves leaving a corner wrongly. The gate is a block leaning 0.35 rad, 3 out from
the axis, screwed a radian at 2 a turn, held to 1e-6 of its volume read in helical coordinates
apart from the kernel and to membership
(`swept_closed::a_block_leaning_to_a_screws_axis_closes_into_its_helical_sweep`, slow tier).

**A cone under a screw is built (#780 C2)**, about an axis along the screw's: an end mill's
chamfer, a countersink. Its normal `cos α ρ̂ − sin α z` is square to
the screw's velocity where `A cos u + B sin u = C`, whatever the height along the ruling: two
rulings turned off the plane through both axes by `asin(κ tan α / (ω |d|))`, or none (a cone about
the screw's own axis, or a climb too steep for its lean). Each leans to the axis and screws into
M5's patch (`Cutter::Patch`), which the cone at each end pose touches along that ruling; its rims'
discs screw into their ramps (`Cutter::ScrewedDisc`). Each pair meets in closed form: a rim's
sheet and the cone at an end pose where the two circles of each level meet, the cone's radius read
at the height the rim has climbed to, its branches ending at the coincidence on the roots of the
coefficients' rates there — on a cone the feet of its contact rulings, where the sheet touches it
(`ssi::screwed_circle_meeting`); a patch and a rim's sheet in the helices of the ruling's points
the screw carries onto the rim (`ssi::screwed_line_circle`); a patch and the cone where each ruling
meets it, a quadratic along the ruling, the ruling the cone holds returned whole and the cut ended
on it at its limit, the roots of the coefficients' first differences across it or, touching, their
second (`ssi::screwed_line_cone`, on the branches a screwed circle's meetings are read by). Each is held point by point on both surfaces
(`swept_closed::a_frustums_screwed_rim_and_ruling_meet_what_they_touch_in_closed_form`). The gate
is `build::cone`'s frustum, radii 1 and 0.4, 3 out from the axis, screwed a radian at 2 a roll,
held to 1e-6 of its volume read in helical coordinates by the leaning block's method, run by run
along `ψ` (`swept_closed::screwed_stadium_volume`, itself held to the frustum's own volume unscrewed
and to the drum's closed form), which a Monte Carlo of the definition read directly over the roll
holds in turn, and to membership; it builds in about seven seconds
(`swept_closed::a_frustum_screwed_beside_the_axis_closes_into_its_helical_sweep`). Not built yet,
each refused by name, never returned wrong (#780 C2, open): a patch cut from a blank — crossing a
plane through the axis a contact ruling, turned only a little off it, nearly lies in it, and one of
a patch's pieces in a plate is read as having no inside — so no drawn example cuts a cone screwed
from a blank yet; and a drafted prism under a screw, its round ends cones and its flat sides
leaning planes ("a plane's pieces do not close"). The native path refuses it by name, as every
closed sweep.

**Which road a body takes (#699 D, decided):** where a class admits a sweep, its blank-scoped sheet
builds it (the fast path); where every class refuses, the closed class cuts it from the blank on one
arrangement (`cut_from`, itself blank-scoped: the blank the arrangement's domain). The two agree
where both build — `punch.sv`'s drum, its class's sheet and the one arrangement, each 672 and the
same to 1e-9, the sheet the faster by about an order
(`closed_class::the_fast_path_and_the_one_arrangement_agree_where_both_build`).

**A path of pieces** (#721 K1, `Trajectory::Pieces`) of slides and turns builds on the same one
arrangement: a sweep is its pieces' sweeps united, so every piece's candidates — read on the tool
where that piece takes it up (`Trajectory::Placed`, a constant twist after a placement: a turn
where a slide left the tool, its pose `twist(t) · at`, its speed bound its twist's, a rigid
placement moving nothing) over its stretch of the roll — split one box at once, and each cell is
read in the sweep of any piece (`swept::member`, piece by piece). Held to a toolpath: a unit cube
slid 4 then turned a quarter turn about the edge it stops beside, `5 + π/2` to 1e-9, agreeing with
membership along the whole path
(`swept_closed::a_toolpath_of_a_slide_and_a_turn_is_its_pieces_sweeps_united`;
`trajectories::a_twist_after_a_placement_is_the_placement_then_the_twist` holds `Placed`'s pose,
rate and speed bound by differences). No drawing lowers to pieces yet (a toolpath written as one
is the language's `path`), so the closed class's admission reads one constant twist.
A curved tool past these (`Unbuilt::Curved`: under a screw a round face about an axis leaning to it, a torus, or a sharp
circle leaning to it (#780 L2); under a turn a ball across its axis, a torus leaning to it or a free-form face; under a slide a
torus's face leaning to it that is no whole ring, or a free-form face; an edge no line or circle) are refused by
what they are, read before any build
(`closed::reads`); under a turn a cylinder or a cone about a leaning axis whose contact is no curve
over the face, or whose meridian folds back along the axis, at the build's first step. A sharp
circle through the axis, or whose normal lies in the plane through the axis and its centre (a pin
along a radius), turns into no ring (`Rim::simple`, #780 L2): refused as it is read, so the sweep is
not admitted to the closed class and the class asked first keeps its refusal
(`rings_all_round::what_a_turn_about_an_axis_the_tools_meets_still_refuses`).

**A round face about an axis leaning to a turn's (#780 C3, `brep::turned`).** On the ring at slant
`v` of a cylinder or a cone about the axis `a` through `o`, its normal `cos α ρ̂ − sin α a` is
square to the turn's velocity where `ρ̂ · W = C`, `W = cos α (Z × q) − R sin α (a × Z)` in the ring's
plane and `C = sin α a · (Z × q)`, `q = m(v) − c`: `A cos u + B sin u = C`, two roots, written without
an angle as `ρ̂ = (C W ± √(|W|² − C²) a × W)/|W|²`. Each sign is a curve along the face, and turned it
sweeps the surface of revolution of its meridian — where its points go in `(ρ, z)` — held as that
closed form in `v` with its derivatives, not fitted (`turned::Turned`, `Curve::Turned`; its jets one
formula read in floats and in outward-rounded intervals, `turned::Num`). Its candidate (`Cutter::Turned`)
is that meridian a twentieth past the face's span each way, closed down the axis at both ends and
turned once (`ProfileEdge::Exact`, `build::revolve`); refused (`Curved`) unless the contact is a
curve all along — its discriminant held above nought and it off the axis, proved over the span by the
intervals (`Turned::defined`) — and its height climbs one way (`Turned::climbs`). The face at each end
pose (its own surface a fortieth past its span) touches that surface along its contact turned there,
`Turned::posed` (the contact itself, in the turn's frame turned by the pose), in closed form (`ssi`'s `own_contact`, held only where the face stands on one side of
the surface over its heights; traced, the touch came apart in two). A line meets the surface
certified (`query::line_turned`: `G = a N² + b D N + c D² − D² ρ(v)²` isolated with `G'` and `G''`
bounded by the interval jets — any meridian with bounded derivatives, not only a conic); two coaxial
such surfaces, or one and a plane, a cylinder, a cone or a torus about the axis, meet in circles where
their meridians cross (`ssi`'s `Meridian::Curve` reads a turned contact's meridian as a spline's). An
axis in one plane with the turn's touches along its two generators there, turned into hyperboloids,
and one crossing the turn's axis within the face is refused. A rim lying in a plane square to the
axis — an ellipse a level plane cuts from a leaning drum, or from a cone (a plane cutting every
generator of a cone meets it in an exact ellipse, `ssi`'s `plane_cone_ellipse`, nothing where that
is the other nappe) — turns within its plane; a coplanar circle and ellipse cross where the ellipse
meets the circle's cylinder, certified, so a touch on that plane is one vertex
(`query::curve_curve`). An edge where a round face runs on smoothly — a line into a plane (a rounded
key's flat into its end), a circle into a ball, however it leans — bounds nothing, and nor does a
concave edge (a stem entering a ball): neither is a candidate. **A whole turn takes no face at its end
poses**: its sweep is the tool's solid of revolution, bounded by its contacts, edges and rims turned
alone, which those faces only touch (a leaning stem at an end pose, touching its contact's surface,
crossed there by its ball's torus, was a triple touch). Gates (`tests/swept_turned.rs`), each against
a volume worked apart from the build and against membership and a Monte Carlo of the definition read
by turning points back over the roll: a drum and a frustum leaning 25° from the turn's axis and 1.2
beside the plane through it, cut by level planes, turned whole — `∫ π (ρ_max² − ρ_min²) dz` over
their slices' radial spans — and 100°, their own volume and `Θ/2π` of that (each slice is convex and
clear of the axis, so every circle about the axis meets it in one arc and the slice sweeps `∫ ρ (w + Θ)
dρ`), each to 1e-9
(`swept_turned::a_leaning_drum_turned_closes_into_its_sweep`, `…::a_leaning_frustum_turned_closes_into_its_sweep`);
a rounded key so (`…::a_rounded_key_leaning_to_the_axis_turned_whole_closes_into_its_solid_of_revolution`,
its part turn slow tier); a ball off the axis tilted on a thinner stem, turned whole, its slices'
spans two apart near the ball's top (`…::a_ball_tilted_on_a_stem_turned_whole_closes_into_its_solid_of_revolution`, to
1e-11) — turned 100° it is refused by name (`Unsettled`: a cell at its concave seam the mesh's sag
cannot settle), never built wrong. Not yet: a drawn example agreeing with the field, a torus about a
leaning axis under a turn (a sharp circle leaning to it: #780 L2, below).  The result is the selection's boundary with the candidates that bound nothing healed out of
its faces (`boolean::selection`): each face as whole as the candidate's it lies on.

**A ring slid at a lean to its axis (#805, #780 L1's X2 under a constant twist, `brep::slid`).** A
constant twist keeps each tool point in contact for the whole roll or never, so what X2 leaves under
one is a ring's two roots meeting on a face. On a torus slid along `D`, its normal `cos v ρ̂ + sin v z`
is square to `D` where `cos v g(u) + sin v D_z = 0`, `g = D_x cos u + D_y sin u`: in the height chart
two roots on each latitude that meet at the silhouette's top and bottom, the branch locus
`A² + B² = C²` lying on the face; in the angle chart two smooth closed curves, `n = ±(D_z ρ̂ − g z)/
√(D_z² + g²)`, the outer and the inner half of the tube. So the patch is read across the locus, not
cut: each curve rational in `s = tan(u/2)` but for that root, in two halves `s ∈ [−1, 1]`, its jets
one formula read in floats and in outward intervals (`turned::Num`; `slid::Slid`, `Curve::Slid` —
the contact itself, or its apparent contour along `D` — its point, box, inverse and speed read off
its jets as a turned contact's are, `turned::Exact`). Each contact swept along `D` is the cylinder
over its contour (`Surface::Extrusion` of the contour, `ProfileEdge::Exact`), its candidate the
contour's region extruded from short of the ring at the start pose to past it at the end
(`Cutter::Contour`). The inner contour **folds** — cusps where the contact runs along `D`, an
asymptotic direction of the tube's saddle — from `tan² β = (R − r)/r` (`β` the lean, first at
`u = ±π/2`): the fold locus, refused by name (`Unbuilt::Folded` at a point of the inner contact,
read before any build, `closed::reads`, by the contour's tangent held off nought in intervals,
`Slid::folded`). Short of it every line along `D` through a contact point meets the ring there alone
— the quartic's other two roots `t² + a t + b`, discriminant `8 (2R² |D_xy|² − |p|² − R² + r²)`, held
below nought (`Slid::meets_again`) — so the ring at each end pose touches each cylinder along its
contact there and nowhere else, in closed form (`ssi`'s `slid_meeting`, `Slid::at`); a plane
square to `D` meets it in the contour carried onto it, one along `D` in lines through the
contour's crossings (`Exact::crossings`, isolated with the jets' interval bounds), one leaning to `D` in the contour
sheared along `D` onto it (`affine::MappedCurve`); the two contours' cylinders stand apart; and a
line meets it certified where it meets the contour in the plane square to `D` (`query::line_slid`).
A whole ring's face is admitted at a lean, and since #810 a band of one bounded by smooth seams
(below). Gates
(`tests/swept_slid.rs`), against a volume worked apart along lines parallel to the slide — every
such line meets the ring once or not at all short of the fold
(`a_line_along_the_slide_meets_the_leaning_ring_at_most_once`), so each line's chord lengthened by
`|D|` sums to `V + |D| A`, `A` the annulus between the contours, `½ ∮ (x y′ − y x′) du` of the
contact read here in angles (exact to rounding) — and against membership and a Monte Carlo of the
definition (each point's line back along the slide read against the ring): a ring of middle radius
3 and tube 1 leaning 35°, slid 6 and 2 (its end poses apart and overlapping), and at 20° slid aslant
its lean, each to 1e-9 (`a_ring_slid_at_a_lean_closes_into_its_sweep`,
`…_by_less_than_its_reach_closes_into_its_sweep`, `a_ring_slid_aslant_its_lean_closes_into_its_sweep`);
pushed into a block until wholly in it, drawn (`solid_sweeps/leaning_ring.sv`), the block less the
centre's depth times `A` and half the ring, `π² R r²` (the ring's symmetry about its centre,
`swept_slid::plunged_away`), to 1e-9 with its STL agreeing with the field
(`solid_sweeps_examples::the_ring_plunged_at_a_lean_bores_round_its_core`). The curve is held to the
contact read apart in angles, its jets to its points' differences and its contour to the contact
less its part along `D` (`the_slid_contact_is_where_the_ring_grazes_the_slide`), and the fold to
`tan² β = (R − r)/r` for three proportions, the contour's speed nought at its cusp
(`the_inner_contour_folds_where_the_slide_runs_along_an_asymptote`). Negative control: at 60°, past
the fold, refused as `Folded` by the build and the admission alike
(`a_ring_slid_past_its_fold_is_refused_by_name`); a ball's pipe that would fold is refused by the same
name (`swept_loci::a_ball_screwed_tighter_than_its_radius_is_refused_by_name`). Profiled
(`SOLVENT_SWEEP_PROFILE`): before, the ring slid 6 was refused as `Curved` in 47 µs; now it builds in
2.9 s (slid 2, 1.8 s), every meeting of a contour's cylinder in closed form (four with the other
contour, eight with a plane, eight with the posed ring) and every line read against it certified;
the ring's seam circles and the contacts' own curves are read against it by the sampled search, and
the ring at its two poses meets the planes and the other pose by the trace. Not yet: the fold, cut
where it lies.

**Touches given by construction, and a bull-nose slid at a lean (#810, #780 R5's first gate).** A
sweep knows where its candidates touch before any meeting is sought, and hands those touches to the
arrangement as given edges (`boolean::Given`, `arrangement_given`; `meeting::Shared::give`): curves
that are two candidates' surfaces' whole meeting, laid as an edge of both with no search for that
pair, each counted in the profile beside the meetings found in closed form and traced
(`… / given`). Each touch names its two candidates, and is given between the faces of their solids
that hold its curves, where both are carried (`closed`'s `Touch`). The closed class gives:
- **a band's contact where it crosses a smooth seam** — the path along the slide of the point
  where a torus face's contact runs on into its neighbour's, on which the sheet over the contact's
  contour and the plane the neighbour's ruling sweeps touch;
- **the torus at each end pose against its contact's cylinder or sheet** (#805's slid touch, `Slid::at`);
- **a leaning round face at each end pose of a turn against its contact turned** (C3, the curve
  `ssi`'s `own_contact` writes, given where it certifies the face on one side of the surface);
- **a face holding a sharp circle at each end pose against the ring the circle turns into** (L2,
  `plane_holding_rim`, `side_holding_rim`).

Each pair-by-pair recognition in `ssi::intersect` can be bypassed (`ssi::bypassing`,
`Recognised::{OwnContact, SlidTouch, RimHeld}`, carried to `util::par`'s workers by `util::bypass`), and each gate
rebuilt through its given touches alone holds to the same closed form
(`swept_slid::the_slid_touches_build_through_their_given_touches`,
`swept_turned::a_leaning_drum_turned_builds_through_its_given_touches`, and in the slow tier
`…::a_flat_ended_drum_turned_builds_through_its_given_touches`); with nothing given and the
recognition bypassed each is refused ("a hole in no piece"). A failed build's dump carries its given
meetings, and its replay meets them (`swept::dump::dump`).

Under a slide at a lean a **band** of a torus's tube is admitted (`closed`'s `Band`): a face that is
no whole ring, bounded by two latitudes of the tube's outer half (`v ∈ [−π/2, π/2]`), each a smooth
seam into its neighbour, curving out (a bull-nose's round, a rounded drum's corners). Its contact is
the outer contact's part on the band, `tan v_lo ≤ −(D_x cos u + D_y sin u)/D_z ≤ tan v_hi`:
nowhere, the whole contact (the ring's cylinder, `Cutter::Contour`, outer side only), or one arc
about the least or greatest `w`, crossing one seam at its two ends, where the contact runs on into
the neighbour's — a cylinder's or a cone's ruling whose normal is square to the slide, swept into
its tangent plane. That arc swept is the **sheet over its contour** (`Cutter::Sheet`), cut at the
seams exactly and run along the slide past the domain both ways: its only rims within the domain
are the seam points' paths, which lie on those planes, given as the sheet's touch with each (a
sheet run past the seams, as a patch's rims are, left a strip inside the sweep whose meetings with
the side's cylinders at the end poses stopped partway across them, and the judge refused the build,
never returned, at four of the six ways it was slid). A solid of revolution among the candidates is turned about its own axis
so its seam stands in the widest gap between the angles where the touches pass (`seam_away`): a
slide across the tool's own seam meridian laid a touch along the cylinder's, the torus's and the
top rim's seams at once. Refused by name (`Unbuilt::Curved`, at a point of the rim): a band meeting
a neighbour in a sharp rim (its fan not entered yet), a contact crossing both seams (two arcs), and a
crossed seam's neighbour no cylinder or cone. Membership reads the stretches of the roll best
first, the deepest bound first (`member::Swept::member`): taken depth first, a point on the tool's top
at the middle pose spent its reads halving the stretch before it, where it stood within the mesh's
sag, and never read the one after, where it lay deep inside — the deepest points of the cell between
the mill's top at its two end poses, slid 1.5 up a leaning way (`membership_reads_the_deepest_pose_first`).

Gates (`tests/swept_slid.rs`), against `V + |D| A` — the mill convex, so every line along the slide
meets it once — `A` its shadow along the slide read apart from the kernel as `½ ∮ (x y′ − y x′)`
over its outline (the bottom round's contact arc, the side's two rulings, the top rim's or the top
round's arc; Gauss–Legendre, exact to rounding, `swept_slid::Mill::shadow`), and against membership
and a Monte Carlo of the definition: a bull-nose of radius 2, its corner 0.5 and 3 tall, slid 5 down
a way leaning 29° (`a_bull_nose_slid_at_a_lean_closes_into_its_sweep`), 1.5 up aslant the other way
(`…_slid_up_aslant_by_less_than_its_reach_…`), across its seam meridian
(`…_slid_across_its_seam_meridian_…`), and a drum rounded at both corners, two sheets and four
touches (`a_drum_rounded_at_both_corners_slid_at_a_lean_closes_into_its_sweep`), each to 1e-9. Cut
from a block, the bull-nose leaning 30° pushed 8 straight down until wholly in it
(`a_bull_nose_pushed_at_a_lean_into_a_block_stops_inside_it`, and drawn,
`solid_sweeps/leaning_bullnose.sv`, its STL agreeing with the field,
`solid_sweeps_examples::the_bull_nose_plunged_at_a_lean_stops_inside_the_block`): every line down the
push meets the mill once, so the block loses `−∫ z_low dA` over its shadow, by the divergence theorem
`V − ∫ z n_z dS` over the faces turned up (`Mill::plunged_away`), to 1e-9. Negative control: a round
spanning only the eighth of the tube above `v = −π/4`, so the flat end meets it in a sharp rim,
refused as `Curved` at that rim by the build and the admission
(`a_round_meeting_a_sharp_rim_at_a_lean_is_refused_by_name`). Profiled (`SOLVENT_SWEEP_PROFILE`):
before, the bull-nose slid 5 at a lean was refused as `Curved` at its round in 53 µs; now it builds
in 3.4 s, its meetings 107 in closed form, 10 traced (the side's cylinders at the end poses against
the sheet, where they touch at the seam points; the end poses' tori against the planes and the top
rim's sheared cylinder) and 4 given (the sheet against each end pose's torus and against each seam's
plane); pushed into the block, 2.1 s, 139 / 6 / 3.

**A sweep cut from a blank on the same arrangement (#699 C, #735 A4, `swept::closed::cut_from`):**
the blank, not a box, is the domain the candidates split, and its cells are kept where they are no
member of the sweep — one arrangement and one classification, with no closed solid built and then
subtracted by a Boolean. The box the candidates are sized by holds the blank as well as the sweep,
so each reaches across both (a screwed sheet's ends and rims past the blank). A bar slid along a
block, sunk into its top, cuts a groove of `4 × 1 × 1` from `24`, its result's ten faces whole
(`a_bar_slid_along_a_block_cuts_a_groove_on_one_arrangement`); a bar beside the axis turned a quarter
turn through a disc takes its section's polar integral times the disc's thickness
(`a_bar_turned_through_a_disc_is_cut_from_it_on_one_arrangement`), each to 1e-9 and agreeing with the
definition (in the blank and no member).

**The closed class, dispatched from `admit_body` (#721 F2c5; `solid::closed_class`,
`brep::sweep::closed`):** a sweep every class it is asked refuses — the generating, constant-twist
or planar subtraction class, whichever its motion and tool asked first — is asked the closed
class's rows, and its refusal stands where they refuse too. C1 (every face of the tool a plane,
every edge a line; under a slide, a face a cylinder, a cone, a ball or a torus too and an edge a
circle; under a turn, a round face (a cylinder, a cone, a torus) about an axis along the turn's or
a ball clear of it — a torus touching round its meridians in the plane through both axes, turned
into tori about the turn's, and round its top and bottom rings, kept in their planes
(`swept_closed::a_ring_turned_about_an_axis_along_its_own_closes_into_its_sweep`,
`a_bull_nose_turned_about_an_axis_along_its_own_closes_into_its_sweep`, each to its sweep's
volume by levels and the membership test; `solid_sweeps_examples::the_bull_nose_turned_partway_stops_inside_the_ring`,
`solid_sweeps/stopped_bullnose.sv`); under a
screw, a cylinder or a cone about an axis along the screw's (a cone's contact rulings screwed into
patches, #780 C2) and a circle square to it, whose disc screwed is
its candidate — a round tool's rim under a screw, helical milling:
`swept_closed::a_drum_screwed_beside_the_axis_closes_into_its_helical_sweep`,
`a_drum_screwed_a_short_lead_through_most_of_a_turn_closes`,
`a_drum_screwed_across_a_ring_sector_bores_its_helical_groove`, each to its helical-coordinate
volume and the membership test; through the drawing,
`closed_characteristics::the_drum_screwed_bores_its_helical_groove`, its STL agreeing with the
field; or balls, alone or beside such a side (a ball-end mill), each touching round its great
circle square to its centre's velocity, screwed into the pipe about the centre's helix,
`Cutter::Pipe`: `swept_closed::a_ball_screwed_closes_into_the_pipe_about_its_centres_helix`,
Weyl's tube to 1e-9; `the_ball_end_mill_screwed_closes_into_its_helical_sweep`, its helical volume
to 1e-6, Weyl's tube and a Monte Carlo of the definition, slow tier; drawn, `solid_sweeps/ball_end_mill.sv`,
3 mm a turn, to `π/3 (40 + ∫ ρ M dρ)` within 1e-8, its STL agreeing with the field and the page's
stepped build held to the export, and at 2 and 4 mm a turn too
(`solid_sweeps_examples::the_ball_end_mill_screwed_mills_its_helical_groove`,
`…_at_other_leads_mills_its_groove`, slow tier; #791). There the pipe's meeting with the plane of
the side's screwed foot, the foot's rim and the touch helix touch at one point at the pipe's start,
and the cylinder's seam touches the pipe on that helix: read alike whatever the rounding — the
meeting ended at the vertex, the node's tangent ways read at a length along each, the seam met in
closed form — and held reduced at three leads, each cell to its closed form
(`swept_closed::the_drawn_ball_end_mills_pipe_beside_its_screwed_side_is_arranged_at_every_lead`). The pipe is inverted
off its spine, #780 C1: each foot of a point on the patch lies on the circle about a foot of it on
the centre's helix or on an end circle, those feet isolated with a bound and the nearest taken,
`geom::Blend::pipe`, `Pipe::feet`, no warm start and no search;
`swept_closed::a_balls_pipe_is_inverted_off_its_spine`. Only the spine's stretches that can hold
the nearest are searched (`Pipe::nearest_feet`, #798): the spine cut a sixteenth of a turn a
stretch, one is passed by where every circle about it stands farther from the point, by its ends'
reach and the spine's speed, than the circle about an end of some stretch — a coil of six turns was
searched whole at every inverse, every projection onto its traced meetings, and the same test holds
the nearest of these to the nearest of every foot. The two failures its gate met on the way,
a traced curve's inverse stepping off along the whole curve where its points crowd and a ball's
great circle tangent to its side's cylinder read as four helices, are kept reduced:
`swept_closed::the_ball_end_mills_pipe_beside_its_side_is_arranged`)
and C2 (one slide, turn or screw; under a screw a face or an edge leaning to its axis screws
into a patch of a ruled helicoid, `Cutter::Patch`, #777 M5) read the tool's B-rep and the lowered
trajectory in millimetres, before anything is built (`brep::swept::closed::reads`, its refusal
`Unbuilt::Curved {at}` with a point of the face or edge); C3 (each cell settled) is the build's.
Admitted, `Class::Closed` carries the tool and the screw, and the body is built by cutting each
sweep at each placement from the blank on one arrangement (`cut_from`, the tool and its screw placed
where the body cuts it, `Screw::moved`, the blank left where it stands), with no sheet, plan or
sector: the export's and the page's stepped build alike, through one dispatch (`sweep::whole`;
`object_builder::a_closed_sweep_builds_at_the_displays_samples`), and the native path refuses it by
name. A body cutting sweeps of this class beside another's is built by the other's road first —
the body less its closed sweeps, whole or by sector and pattern — and each closed sweep then cut
from what that leaves on one arrangement (`sweep::whole`'s `mixed_cut`, `closed::cut_on`), on both
roads (`closed_class::a_closed_sweep_is_cut_from_what_another_classs_sweep_leaves`, `punch.sv`'s
drum and `plunge.sv`'s bar in one plate, to 1e-9; a union's members alike, each its own sweep). A fitted sheet's face lying in a plane a
closed sweep's candidate holds is one surface with it (#780 C4, A2's coincidence): `punch.sv`'s
slot's end wall, a strip of the plane x = 6 its drum's flat end sweeps, fitted, and `plunge.sv`'s
bar's end face at its start pose in that plane, which had been two faces coinciding round the
slot's corner edges (`complex`'s "coincide along it"), are one face naming both, and the plate
beside it is split by that plane over the sheet's patch alone (brep.md's A2; the reduced
Boolean, `closed_class::a_fitted_sheet_lying_in_a_candidates_plane_is_one_surface_with_it`, the
plate and one block, 240 and 432 to 1e-9; the case,
`a_closed_sweep_is_cut_from_what_another_classs_sweep_leaves`, its bar's end at x = 6).
Each gate is a negative control of another class, held to a volume apart from
the build and its STL to the field with no disagreement (`tests/closed_class.rs`: a bar pushed
aslant wholly through a plate, and `punch.sv`'s drum wholly through it and partway into it, its end
pose's cylinder in the plate touching its side's tangent planes along lines and its rims' oblique
cylinders at points, each to 1e-9; `solid_sweeps_examples::the_bar_plunged_aslant_stops_inside_the_plate`,
`the_drum_plunged_aslant_stops_inside_the_plate`,
`the_bar_turned_partway_stops_inside_the_ring`, each X1 to 1e-9; slow tier, a leaning bar turned
wholly through a ring sector and `the_bar_screwed_partway_stops_inside_the_rod`, each to 1e-9). Two
coaxial surfaces of revolution with spline meridians — two edges' hyperboloids — meet in circles
in closed form (`ssi::spline_spline`, brep.md), where tracing them made the leaning bar's cut 300 s
of crossing searches against 2400-point traces; it is 12 s.

**A sheet meeting itself (#780 L1, X5; docs/swept-solid-design.md step 6).** A ball's pipe under a
screw comes back onto itself where its roll passes a turn at a lead short of its diameter. In the
screw's helical coordinates the pipe is its circle's points' helices, so it meets itself exactly
where the circle holds two points at one distance from the axis whose `z − κψ` differ by a whole
number of turns' rise, each such pair a helix (`geom::Pipe::returns`, isolated with a bound;
`swept_loci::a_pipe_comes_back_onto_itself_along_helices`). Such a pipe is entered in stretches of
the roll of at most half a turn, each meeting itself nowhere, no two joints, nor a joint and an
end, a return's turn apart (`closed::pipe_stretches`); one that does not come back is
entered whole, as before. Two stretches of one pipe meet in closed form, round the circle where
they abut and along those helices, and a plane holding a circle of the pipe meets it in that circle
and its level-by-level cut (`ssi`'s `pipes_meeting`, `pipe_plane`;
`swept_loci::two_stretches_of_one_pipe_meet_round_their_circle_and_along_helices`). A ball reaching
its centre's helix's radius of curvature, where the pipe folds, or the axis, is refused by what it
is (`Unbuilt::Folded` where it folds, by name, `Curved` where it reaches the axis, each at its centre;
`a_ball_screwed_tighter_than_its_radius_is_refused_by_name`).
The gate, `swept_loci::a_ball_screwed_past_a_turn_meets_its_own_pipe_and_closes_into_its_sweep`
(slow tier): a ball of radius 1 three out from the axis screwed a turn and a half at 0.2 a radian and
two and a half turns at 0.12, each to 1e-9 of a volume worked apart in helical coordinates (each
circle's angles covered, `min(2π, σ_max − σ_min)`, integrated with its corners cut; held first to
Weyl's tube below a turn, `the_worked_volume_of_a_ball_screwed_short_of_a_turn_is_weyls_tube`), to a
Monte Carlo of the definition and to membership; before L1 the judge refused it (`Unbuilt::Wrong`).
The failure met on the way, kept reduced: a point past a patch's rim, on its surface's extension,
is in none of its face (`query::Located::face_place`, its foot off the patch read from the
parameters in hand and held to the surface's inverse; `a_point_past_a_patchs_rim_is_in_none_of_its_face`)
— beside the circle where two stretches abut, the one whose rim it was had read it in, and a ray
crossing there was counted twice.

**Fans and cones (#780 L2, G1; docs/swept-solid-design.md step 5).** A sharp edge's fan is the
arc of normals between its two faces', square to its tangent `T`; the normals in that plane square
to the velocity `v` are `±T × v`, so a point of the edge is on the fan's contact exactly where one of
them lies within the arc, and the fan's sheet is the edge's own points swept: it lies on the
surface the edge sweeps, entered whole as the candidate, and membership keeps what carries
boundary. Its rim, where the arc's end normal is square to `v`, is where a face's own contact
reaches the edge, and there the two sheets are tangent. A vertex's cone of normals sweeps no
sheet: whatever the normal, its image is the vertex's own path, a curve its edges' sheets meet in,
so no candidate is entered for it. **Under a turn, a sharp circle or arc of one leaning to the axis**
— convex, the faces meeting at an angle — **turns into its circle's ring**, whose meridian is read
exactly (`turned::Rim`, `Turned::rim`, `Cutter::Turned` with a `Source::Rim`), its sites the arc's
own points; two arcs of one circle enter one ring. Measured from a point of the axis, `ρ² + z²` and
`z` are affine in `(cos w, sin w)`, so in `(ρ² + z², z)` the meridian is an ellipse and in `(ρ, z)`
a simple closed curve, refused by name (`Curved` at the circle's centre) unless proved so — the
ellipse's axes parting and the circle clear of the axis over 64 interval pieces (`Rim::simple`).
Its candidate is that meridian turned once, a ring (`build::revolve` of one closed
`ProfileEdge::Turned`), periodic round its meridian as a torus is (`Turned::period`, one rule for
every turned curve read over a whole period of its source's angle: its
slant read round it in Newton, so a trace across the meridian's start does not stall there); a line
meets it certified as any turned meridian (`query::line_turned`, the circle's jets read through
`Num::sin_cos`, in floats and outward-rounded intervals), and coaxial surfaces meet it in circles
where meridians cross or touch. **At an end pose** of a part turn, every meeting where the tool's
faces hold the rim is exact (`ssi`): the end's plane meets the rim's ring in the rim and a second
circle of its radius (`plane_holding_rim`); the side's cylinder in the rim and the closed loop
crossing it where the two touch (`turned::Crossing`, read round `τ` with `σ(h) = −cos τ`,
`side_holding_rim`), the side's surface there reaching past each rim beyond its loop; and any
plane not level against a contact's surface of revolution (#780 C3) in its cut, read round `τ` with
`C(v) = −cos τ` where `C` runs one way over the span (`turned::Section`, `plane_contact_section`).
A circle and a plane's cut in its plane (`Turned::plane`) cross where the cut meets the circle's
cylinder (`query::curve_curve`), so their tangency where the side's contact reaches the rim is one
vertex; a turned loop over a whole turn has its period, so its edge is laid round from its start,
and it crosses the rim where it crosses the rim's plane, as two curves of one meeting are crossed.
Part of a turn, a rim beside a round face other than a cylinder (a cone, a ball) is refused by name
(`Curved` at its centre), that face's meeting with the ring not yet in closed form. A plane face
with an edge no line has its sites and its contact line's stretch read off its outline (each edge's
start, and 32 points along an edge no line, its grid as much finer as its fan has more triangles),
not off its corners, of which a drum's end has one. Gates
(`tests/swept_turned.rs`), each against a volume worked apart from the build, membership and a Monte
Carlo of the definition: a flat-ended drum (X4: a flat end whose corners dig past it), radius 1 and
3 long, leaning 25° and 1.2 beside the plane through the axis, turned whole, to 1e-9 of `∫ π (ρ_max²
− ρ_min²) dz` with each slice's radii in closed form (it agrees to about 3e-12;
`…::a_flat_ended_drum_leaning_to_the_axis_turned_whole_closes_into_its_solid_of_revolution`), and
turned 100°, its own volume and `Θ/2π` of that, to 1e-9 (`…_turned_part_of_a_turn_closes_into_its_sweep`,
slow tier, about 15 s); a half drum (its rims half circles) turned whole, its slices' spans an
ellipse cut by half-planes, the integrand cut where the point holding an end changes
(`…::a_half_drum_leaning_to_the_axis_turned_whole_closes_into_its_solid_of_revolution`; that
reading held to the drum's, `the_worked_volume_of_a_cut_drum_is_the_flat_ended_drums`); drawn,
`solid_sweeps/leaning_mill.sv`, the drum swung 100° inside a ring, to 1e-9 and its STL agreeing with
the field, the page's stepped build held to the export
(`solid_sweeps_examples::the_leaning_mill_turned_partway_bores_its_channel`, slow tier). The ring and
the closed forms are held apart (`…::a_sharp_circle_turned_is_a_ring_read_exactly`: jets, intervals,
simplicity, its volume `2π² r² |n · (c × z)|`, the dump, a line's crossings against the ring's
implicit form; `…::the_meetings_where_a_flat_end_at_an_end_pose_holds_its_rim_are_exact`), and the
two failures the part turn met kept reduced (`…::a_flat_ends_side_at_an_end_pose_and_its_rims_ring_are_arranged`,
the loop dropped as no edge; `…::a_plane_cutting_a_rims_ring_across_its_meridians_start_closes`, the
trace stalled at the meridian's start). **Refused by name, never built wrong**: the half drum turned
part of a turn (`Unsettled`: where its arcs end at its cut, five candidates meet and a cell is
thinner than the mesh's sag; `…_is_cut_right_or_refused_by_name`, slow tier); a sharp circle
leaning to a screw's axis (`Curved` at the circle, `swept_closed::a_ball_with_a_flat_screwed_is_refused_by_name`:
its sheet meets the ball at an end pose and the ball's pipe in curves not yet given in closed form,
design step 5).

**K2's chart, contacts read at each instant (#721 K2, `brep::swept::instant`):** under a motion no
constant twist takes, a face of revolution's contact on each ring is still `A cos θ + B sin θ + C = 0`
against the instant's twist read on the tool (`Twist::body`, `Surface::ring_contact`), so it sweeps a
chart over `(t, h, ±)` (`ring_chart`): each ring's two roots at each roll, told apart by the sign of
the contact's rate round the ring, each carried by the pose; and the branch locus where they meet,
`A² + B² = C²` (`branch_locus`), the curve in `(t, h)` the patch is cut at.  Held to what the chart
does not compute (`tests/swept_instant.rs`): every root grazes in the world, read by the world's twist
(`Twist::spatial`, `every_root_of_the_chart_grazes_in_the_world`); a screw's roots stand still on the
tool; the roots meet on the locus; and a ball's contacts under a tumble (a turn seen from a turn
across it) bound what it sweeps, a hair inside a member and a hair outside not, by the kernel's
membership (`a_balls_contacts_bound_what_it_sweeps`).  Sampled where it samples.

**K2's patches: the closed class under any rigid motion (#780 L3, `brep::swept::patches`):** a
sweep under a motion no constant twist takes, nor a path of pieces (`Trajectory::General`, a frame
along a path #398; a `Relative`), is built on the closed class's one arrangement
(`swept::closed::closed`, `cut_from`) from three candidates: the tool where it starts and where it
stops, whole, and the sheet its contact sweeps over the roll. The tool must be a **convex solid of
revolution** (`Revolved::read`: every face a plane square to one axis, a cylinder, a cone or a torus
about it, or a ball centred on it, its meridian one chain from the axis round to the axis turning one
way); anything else is refused by name before anything is built (`Unbuilt::Curved` at a point of the
face, `swept_patches::a_tool_no_convex_revolution_under_any_motion_is_refused_by_name`: a box, a
whole ring). **The contact is read per tool, at each instant, in a pencil of half-planes** about the
line through the tool's centre along its velocity: each half-plane cuts the convex tool in an arc
along which `n · v` changes sign once, its root the contact there (on a sharp edge where the edge's
fan holds it). A half-plane holding no one root, a contact crossing a sharp edge straight from one
face onto the other (a fan of no width, between a fan's piece and none), or a contact whose pieces
(each on one face or one sharp edge, joined where it crosses a seam or reaches an edge) change over
the roll, is refused by name (`Curved`). The chart is transverse to the branch locus, so the patch is not cut there; a
piece on an edge is read along the edge's circle, where the pencil's angle runs as a square root.
The sheet is a quintic net over (the fraction round the loop, the roll) fitted until halfway
points stand within a tenth of the arrangement's tolerance (`K2_FIT`), cut into a tube of faces at
the pieces' joins, its rims the contact at the end rolls. **Every meeting of the tool at an end
pose with the sheet is given** (`boolean::Given`, R5): the piece's iso-curve at that roll on the
faces holding it (an edge's piece on both its faces), nothing on the others — the sheet stands
nowhere inside the tool at an end pose — so nothing is sought between them (a bull-nose slid at a
lean, its end poses' touches sought, took 20 s; given, 0.8 s). The tool at an end pose is turned
about its own axis so its seams stand clear of its contact there (#810's `seam_away`), and a ball
laid anew with its poles along its centre's velocity, so its touch circle passes no pole
(`patches::posed`). The
sheet stops at its rims: carried on past them, its flap's section on a blank's face the tool stands
across where it starts was tangent to the tool's section there, and the cell between them misread
(refused by the judge, never returned wrong); stopping there, the sheet's section ends on the
tool's. Held (`tests/swept_patches.rs`): **a ball along a
spline frame is its fitted tube** — `path.sv`'s ball, to 1.2e-9 of the canal class's own fitted
tube built apart (`swept::canal::tube`) and of `π r² L + 4/3 π r³`, to a Monte Carlo of the definition
and to membership (`a_ball_along_a_spline_frame_is_its_fitted_tube`); a bull-nose slid at a lean
read instant by instant is the constant twist's build to 1e-8
(`a_bull_nose_slid_at_a_lean_instant_by_instant_is_the_slides_sweep`); a bull-nose screwed beside
the axis, which the constant twist's road refuses (`Curved`), to 1e-6 of a volume worked apart
(`a_bull_nose_screwed_instant_by_instant_closes_into_its_sweep`); and **a bull-nose along a spline
frame** leaning 30° toward where it goes, to 1e-6
(`a_bull_nose_along_a_spline_frame_closes_into_the_volume_its_faces_advance_through`). The volume
worked apart (`swept_patches::advanced`) holds where the tool enters each point once and leaves it
once: what the tool holds where it starts and `∫ ∫ (n · v)⁺ dA dt` over its faces, each ring's angle
cut where `n · v` or a clipping height changes sign and the meridian where either first reaches a
ring, read in a cosine substitution that takes the square root out; each such volume is itself held
to a Monte Carlo of the definition (the tool's poses over the roll carrying each point back).
**Dispatch** (`solid::closed_class::admit`, `Closed::general`): a sweep every other class refuses,
whose motion is no constant twist and which `closed::reads_general` takes — the tool a convex solid
of revolution, its contact read round the pencil at 65 rolls with the same pieces at each, its
centre's path coming back nowhere (`patches::admits`) — is the closed class's ("any rigid motion");
otherwise the first class's refusal stands (a lens rolled with its pole in contact at every time,
its contact crossing its sharp rim with no fan, keeps `Stationary`; a drafted disc whose path loops
three times keeps the planar subtraction's `Slabs`); the build carries the blank back into the tool's placing, cuts it there and
carries it out (`brep::sweep::closed::cut_on`); a model unit other than the millimetre is the
trajectory scaled, part by part (`Trajectory::scaled`). Drawn: `solid_sweeps/spline_mill.sv`, the bull-nose riding
`path.sv`'s spline with its round sunk in a block, the block less what the mill holds below its top
where it starts and `∫ ∫ (n · v)⁺ dA dt` over its faces below the top, to 1e-8, its STL agreeing with
the field and the page's stepped build held to the export
(`solid_sweeps_examples::the_bull_nose_along_a_spline_cuts_what_its_faces_advance_through`). A sheet
that folds (its area factor `|Σ_u × Σ_t|` below a thousandth of its rates', or its normal turning
against the way out from the tool's centre: `Folded`) or comes back onto itself (the tool's centre
back within twice its reach after running half a turn of a circle of it, the canal's B2 for any
convex tool: `Returns`) is refused by name at the fitted points, sampled, the judge seeing what falls
between (`a_sheet_folding_or_coming_back_under_any_motion_is_refused_by_name`). **Left open**: a tool
no convex solid of revolution; a contact whose pieces change over the roll (a contact reaching an
edge or a seam partway: the patch cut there, step 6); a fold cut where it lies and a returning sheet
entered in stretches (L1's, under such a motion); and the sheet carried past its rims.

**The generating class's reading is the kernel's (#721 F2c):** a tool point's contact chart under
the trajectory, its form read once of the trajectory's structure (`brep::swept::generating::Form`:
a turn, or a turn seen from a turn, a sinusoid in the roll; a slide seen from a turn, a rack, a
line in it — `Form::normal_velocity`, a `NormalVelocity`, its roots `ContactTime`) and its
contact times over the roll (`Chart::new`, the lowered trajectory, its form and the roll:
`Chart::at_point_normal` for a point carrying a normal, `Chart::at_surface` for a surface's point,
each a `TimedContact`), the cutter sectioned at its stations (`brep::swept::cutter`) and its
stations' contact curves traced into the candidate sheet (`brep::swept::trace`) are in
`cad-kernel`; `contact_trace::…` and `cutter::…` below name `brep::swept::trace` and
`brep::swept::cutter`.  The chart reads each screw's axis as the unit axis the trajectory holds.
A trajectory with no chart the kernel refuses by what it is (`Uncharted`: a motion that is no turn
and no turn or slide seen from one, one at a pace, one evaluated), and `Family::normal_velocity` and
`SweepContacts::chart` say that as the drawing's steps say it (`Family::uncharted`), the words
unchanged; a point's own failures (no normal, coefficients that overflow) are its
`PointContactError`.  The chart is held on trajectories built by hand, seeded, to the
contact function of the trajectory's own pose and rate, each contact time it finds to being in
contact, and each refusal to its kind (`tests/surface_contact.rs`,
`a_generating_sweeps_chart_is_read_off_the_trajectory`; sampled).  A turn of no ratio is the
slide it is, so seen from a turn it is read as a rack, and an observer of no ratio is a frame at
rest.  `SweepContacts::at_point_normal_over` and `at_source_over` read the kernel's `Chart`
(`SweepContacts::chart`), and the tracer is handed it (`Tracer::chart`).  The
move was held as F2b's was: a temporary hook wrote down, at full precision, every traced sheet the
core suite makes (50 distinct), every surface point's contact times (37,570,280 distinct readings)
and every chart and point's contact times read through the solver, before the move and after —
every sheet and every surface point's times the same to the bit, and every chart and point read
after the same as one read before, none new.

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
sought by `roots::brent`) — a slide square to the tool's axis or a turn about one parallel to it,
its ring along it, or a turn about an axis the tool's meets whose ring `rounds` missed — where the
boundary leaves the walked branch with no root moving (a slide along the tool's axis is read whole,
and a turn about an axis the tool's meets walked onto its ring, #684 X below). A turn or a slide of one
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

**A joint driven by a law is its joint's sweep over the law's reach (#684 L, `solid::paced`):** a
sweep is its path's, not its pace's, so a sweep whose motion is one joint driven by a law
(`MotionDef::Driven`, read where the drive is the sweep's own motion and not one seen from another)
is the sweep of the joint it drives (`of`) over the least to the greatest value the law takes over
the sweep's roll (`roll::interval`): the law is continuous where its enclosure is carried, so it
takes every value between. The reach is `motion::Law::range`, `interval::minimum::enclose` each way
over the law's enclosure (`Law::enclose`) narrowed by the mean value form, each end a value the law
takes at the search's witness, within `law::RANGE` (1e-12 rad, or of the values at the roll's ends
past a radian) of the true one; a law with no enclosure or left unsettled, or a reach under
`paced::STILL`, is not read so. `paced::restated` is remembered against the sweep's reads
(`Caches::paced`), a law left unread too. A body reaching such a sweep is built from
`paced::lowered`, a copy of the drawing with each such sweep's motion and roll rewritten in place,
keeping its name, as the joint's over a roll stated; `solid::lowered` applies it before
`distributed::lowered` (over its copy, so a union under a drive is its members' sweeps under the
joint), and is what `admission::admit_body`, `accuracy::Meter::read`, `brep::export`'s `own` and
the page's stepped `Build` read; the build is held to the drawing's own field of the driven
sweep. A ball's tube
under a drive is the canal's closed form so. M1's refusal of a drive seen from another motion says
so (`NormalVelocity`). The native path refuses it by name (`--kernel rust` builds it).
`tests/paced_sweeps.rs` (`Law::range` against closed forms, 1e-12; a slide risen, dwelt and
returned, the lens; a ball rocked out and back, Pappus; a drive seen from a turn refused at M1),
`solid_sweeps_examples::the_rocked_cutter_leaves_the_turns_groove` (`solid_sweeps/rocked.sv`, held
to `turn.sv`'s figure) and `object_builder::a_rocked_turn_builds_at_the_displays_samples` are the
gate, each STL agreeing with the driven sweep's field.

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
point, runs onto the fan and round it to the next face (`Walker::step`); the rows S1–S4 are asked of the
fan's nodes as of any. Where each piece meets the next is read once from the pieces, before any
pass (`constant_twist::Junctions`, #721 F3): at each end of each piece the pieces it may go on
onto and the end of each (`Junction`: the next face across an edge with the fan between, a rim's
side or a cap's line, a flat's line, the arms of a ring in contact all round, or why the walk ends),
and along each piece the rings in contact all round of it and the rims its side's root crosses
onto (`Along`). `Walker::step` goes on along its piece or across a junction, a pass's rings saying
only which root beyond is the point it stands at. A ring with no roots says why by its kind
(`Unrooted`: in contact all round, an apex, a pole, a double point, …), which the walk reads (a
smooth join to a flat, #684 O) and words only where it ends. This changes no admission: the whole
suite holds it. Where both faces' characteristics meet a convex corner at one point, every
fan normal is in contact there and the sweep has a sharp edge along that point's path: the walk
steps straight across and the sheet is creased there (#684 V, below); S1 refuses a concave
corner's jump. The sheet's rows are laid by length along the
section plus each turn of its normal counted as the section's length over a half turn, so a fan
(the rim of a 60° V under a thread's screw: 0.06 mm turning 100°) is as many rows as its turn asks;
the rows where the walk runs onto and off a fan are exact rows of the grid (`Characteristic::fan_ends`,
kept by refinement), and the fitted sheet is cut there into faces (`Fitted::named`, the bands
unnamed: a fan is no face of the tool) where the fan turns more than a tenth of a radian (a
drafted crease's few degrees stay in their neighbours' faces, #684 AG). Kernel rule it found: a near-crease in one fitted net lets
a point of one flank be inverted onto the other, a pcurve off its edge by a tenth of a millimetre
and a split with no cell; cut into faces, each inverts on its own. And a second: a plane meets the
fan in a curve shorter than a march's step (0.09 mm against `h0`'s 0.22), which a trace keeps
round the rim's tip and traces once ([brep.md](brep.md);
`object_builder::a_sharp_rims_fan_builds_at_the_displays_samples`). `Characteristic::fans` names
the corners walked. `solid_sweeps_examples.rs` (`vthread.sv`, a V wheel ground along a helix, its
section computed apart from the core) and `turns_and_slides.rs` (a V wheel slid tilted, its shadow)
are the gate, each STL held to the field.

**Prisms under a constant twist (#684 M):** a prism tool (drafted too) under a screw, a turn or a
slide is walked along its rulings (`constant_twist::prism`). A screw moves a ruling's points as one
affine field and its normal is one, so its contact is linear along it: a side is a piece whose
stations run along its edge, its ring the ruling and its one root read from the contact at the
ruling's two ends, none where that lies past a cap (`Piece::Ruled`, `Ruling::Side`); a convex
corner — normals more than `LEAST_TURN` apart, the chord of `SweepContacts::corner`'s probe in the
material — is a piece whose stations are its fan's normals and whose ring is the ruling the sides
share (`Ruling::Rib`), crossed onto and off as a revolution's corner is (`step`, by the piece's
`patch` and `fan`, never by its index). A piece's stations are spread by the characteristic's length
(`Ruled::lengths`), so a root running out along the rulings toward a cap is sampled as finely as
anywhere; a contact within `NOUGHT` of the tool's speed reads nought, one bar for every reading.
Where the motion moves each ruling's points alike (a slide, a screw along the extrusion) the
rulings in contact are whole: each root of `n·v` across a side or a fan is a piece of its own,
its stations along the ruling (`Piece::Line`). S1 also refuses a ruling in contact all along it
whose path enters the blank (`constant_twist::rulings`, both ends' contacts changing sign between
two coarse stations and their least sought by `roots::brent`; under a slide, a piece in contact at
every parameter read) — under a slide, a side the motion keeps in its own plane, along the slide
— but a lined side's, which is walked (#684 AF, below), and a plane side a turn keeps in its own
plane, walked across its line (#684 O, below).
A cap reaching the blank under a constant twist is walked where the tool is a prism standing alone
(#684 AE, below), any other tool's refused at T1 (`admission::ends`), and a drafted prism running
both ways from its parting plane is walked across its crease (#684 AG, below); a twist
that keeps the plane of the tool's prisms is dispatched to the planar subtraction class
(`planar_subtraction::asks`, `Asked::Subtraction`), which reads it as it did. The corner's probe
(`SweepContacts::corner`, which T2 reads too) stands halfway along a prism's sides: at a cap its
chord lies on the tool's boundary, where the material reads nought, and a convex corner read
concave. `solid_sweeps_examples::the_square_tool_leaves_a_groove_widening_outward`
(`solid_sweeps/square_thread.sv`, a bar screwed across a rod, the groove in closed form) and
`tests/prism_twists.rs` (a bar leaning across a turn and the same drafted, each groove its corner's
line turned; a bar slid aslant, its silhouette's plane; each STL agreeing with the field; the
refusals and the upright bar still planar subtraction's) are the gate, with
`object_builder::a_square_thread_builds_at_the_displays_samples`. The accuracy meter reads no
prism tool's cut, in any class.

**A prism's cap under a constant twist (#684 AE):** where the tool is a prism standing alone or
placed, undrafted, its profile one loop (`admission::Lone`, which `check_screw` hands `ends`, so
T1 lets its caps reach the blank, and the walk as `Ask::caps`), `constant_twist::prism` adds a
piece for each plane side's rim on each cap (`Ruling::Rim`, `Across::Cap`: the side's edge at its
`v` 0 or 1, its stations the fan's normals turned from the side's to the cap's, one root a station
as a corner's) and each cap's contact line (`cap_lines`: `n·v` is linear across the cap, nought
along a line; each stretch of it within the profile, between its crossings of the sides' edges
there, in order along it and paired where the middle lies on the tool, a `Piece::Line` with its
`cap`). A side's stations take in those where the contact at a ruling's capped end is nought
(`Ruled::crossing`, `crossed`), so the walk steps from a side's root onto its rim's fan partway
along the side (`step`, `Piece::rim_of`), off the fan's far end onto the cap's line and off the
line's end onto the next rim, and off a rim's first end onto its side at the station whose root
it is (`side_at`). S1 refuses where a corner's, a rim's or a crease's root runs off its line
(`VERTEX`: the walk reaches a vertex of the prism, a sharp edge of the sweep), where a side's root
runs over a cap with no rim to cross (`OVER_CAP`: a round side's), where a cap's line leaves it
onto none (`OFF_CAP`), and where a silhouette's ruling runs over a cap under a slide or a screw
along the extrusion (`SILHOUETTE_OVER_CAP`), each within the walk's margin of the blank. The sheet
is `brep::sweep::helical`'s, cut into faces at each fan's ends. The gate is
`solid_sweeps_examples::the_flat_end_cuts_a_floor` (`solid_sweeps/flat_end.sv`, a bar's flat end
facing down toward the axis turned across `turn.sv`'s block: the end's line turned is the floor, the
inner side's the wall, the rim's fan between; held to the bar's shadow read apart, each height's
slice of its half-spaces bounding what the turn removes there, within 2e-5 at 0.001 mm, 9e-5 at
0.01 by the fit's bias) and `prism_twists::a_bars_flat_end_screwed_cuts_a_floor` (the bar screwed
8 mm a turn, held to arithmetic over `z − kθ`, which the screw keeps, within 2e-5 at 0.002 mm), each
STL agreeing with the field; with the refusals
`prism_twists::a_corner_reaching_a_cap_under_a_screw_is_refused` (S1, `VERTEX`),
`a_cap_in_the_blank_under_a_slide_is_refused` (S1) and
`a_cap_of_a_tool_no_lone_prism_turned_is_the_closed_classs` (T1 here; the closed class's), and
`object_builder::a_flat_end_turned_builds_at_the_displays_samples`.

**A drafted prism running both ways under a constant twist (#684 AG):** each plane side's crease
at the parting plane (`SweepContacts::creases`) is a piece read as a rim is (`Ruling::Rim`,
`Across::Half`: the half below's edge at its `v = 1`, its fan turned from that half's normal to the
half above's), stepped onto from the half below where its root reaches the plane and onto the half
above at the station whose root leaves it, both by `Ruled::crossing`; `constant_twist::prism` no
longer refuses it. Its few degrees' fan stays in the faces of the halves either side
(`helical::SHALLOW_FAN`, a tenth of a radian, `Characteristic::fan_turn`; its ends still exact
rows): cut out, a band 0.08 mm wide met the block's side plane in the split with a pcurve 3e-5 off
its edge (first seen at the display's samples, then at a 15° lean in the export). Under a slide, or
a screw along the extrusion (`silhouette`), it is still
refused at T1, the silhouette turning at the crease; a corner of a half meeting the plane is a
vertex (S1), a round side's crease is not walked (S1, `OVER_CREASE`). The gate is
`solid_sweeps_examples::the_creased_bar_crosses_its_crease` (`solid_sweeps/creased.sv`, a bar
leaning across the turn drafted 2° both ways, its inner side's halves joined round the crease;
its shadow from its ten half-spaces within 2e-5 at 0.001 mm) and
`prism_twists::a_bar_drafted_both_ways_screwed_crosses_its_crease` (screwed 6 mm a turn, by `z − kθ`,
within 2e-5 at 0.001 mm), each STL agreeing with the field, with the refusal
`prism_twists::a_corner_meeting_its_crease_turned_is_refused` (S1) and
`object_builder::a_creased_bar_turned_builds_at_the_displays_samples`.

**A ruling in contact all along it under a constant twist (#684 AF):** a plane side each of whose
rulings the motion moves square to its normal alike (`n·(ω × e)` nought: each ruling's two ends'
contacts within `NOUGHT` of the tool's speed of each other at all 256 parameters read, and not
nought all over, which is a flat face, #684 O) is lined (`Ruled::lined`): no ruling of it has one root
(`Ruled::root` gives none), and each ruling where its contact is nought (`noughts`) is a
`Piece::Line` from cap to cap with its `side`, the section reading a silhouette gets.
`constant_twist::prism` builds them; `step` steps off a line's end onto the fan of the rim its side
meets there (`Piece::rim_of`, the fan's first station, the side's normal) and off a rim's first end
onto the lined side's line (`lined`), and a walk whose reach lies within a line starts and ends
on it. S1 `rulings` skips a lined side and still refuses any other ruling in contact all along it
whose path enters the blank, saying whether the characteristic branches there (a ruling `BESIDE`
either side with a root on the tool, as at a corner whose fan holds a turn's axis) or runs along
it. A fan the walk runs round whose nodes, from the one it runs onto it at, all stand within 1e-7
of the tool's size of one point (under a turn a lined side's ruling lies in a plane through the
axis, and its rim at a cap runs along the turn's velocity there) was refused at S1; it is a crease
since #684 V (below). `Characteristic::lined` names the lined sides walked. The gate is
`solid_sweeps_examples::the_facing_side_cuts_a_cone` (`solid_sweeps/facing.sv`, a bar leaning
out in a plane through the axis, its inner side facing it squarely, turned: the wall that side's
ruling turned, a cone, in closed form within 1e-5) and
`prism_twists::a_side_facing_the_axis_screwed_runs_along_its_ruling` (`flat_end.sv`'s bar at 45°
with its section square, screwed 8 mm a turn, from the end's line round the rim onto the ruling;
by `z − kθ` within 2e-5 at 0.002 mm), each STL agreeing with the field, both refused at S1 before;
with the refusal of `prism_twists::a_corner_holding_the_turns_axis_is_the_closed_classs` (S1, a
branch; the closed class builds it), and
`object_builder::a_ruling_in_contact_all_along_it_turned_builds_at_the_displays_samples`.

**Flat faces a turn or a slide keeps in their own planes (#684 O):** a face whose contact `n·v`
is nought at three of its points against `NOUGHT` of the tool's speed (`constant_twist::kept`) — a
revolution's face whose meridian is a line square to its axis (`RevolvedSurface::flat`), or under a
turn or a screw about any line but the extrusion a prism's plane side (`prism`) — is a
`Piece::Flat`, every point of it in contact. Its stations run along the straight line `Flat::line`
from where it stands least across the motion's sections to where it stands most (`across`: its
distance from a turn's axis, its offset square to a slide): a disc's diameter square to the slide
or along the line from the turn's axis through its centre (`revolved_flat`), a side from its
point nearest the turn's axis to its corner farthest (`side_line`). No fan beside a flat is a
piece (`corners`, `prism`'s ribs, rims and creases): it is in contact at one point all round or
nowhere. `step` steps from a face's root at its end onto the flat's line where the root is one of
its ends (`onto_flat`; elsewhere, or onto a flat with no line, it ends), and off the line's other
end onto a face beside it whose root there is the point, or another flat's line (`OFF_FLAT`
otherwise). A revolution's face joined smoothly to a flat disc has its ring at the join, in contact
all round, replaced by the flat's line's two ends with the flat's normal (`Walker::pass`). S1
refuses a kept face with no line — a ring, a partial revolution, a face a turn's axis passes
through (`THROUGH`) — where any of a 17 × 17 net of its points enters the blank (`unread_clear`),
and under a slide (`silhouette`) a prism's side kept in its plane, saying the class reads one only
under a turn; S2 lets two roots of one ring reach the blank only where both are on the walk with a
flat's line between them; S3 is not read at a crease's node. `Characteristic::creases` gives each
node where the walk runs onto or off a flat, and `Characteristic::flats` names the flats walked.
`helical_sheet` lays a row at each crease (each stretch between creases at least four rows), counts
no turn of the normal across one, withholds no side contact on one, and cuts the sheet into faces
there; `Sheet::creases` carries the rows, and `sheet::fit` interpolates by
`nurbs::interpolate_creased` (the rows along the columns, then each stretch along the rows at its
own `net_parameters`, joined by a knot three times over: continuous across a crease, each side its
stretch's fit, a flat strip its plane to rounding), and `judged`'s fold check compares no readings
across a knot standing that often. The gate is `solid_sweeps_examples::the_end_mill_cuts_a_slot`
(`solid_sweeps/slot.sv`, an end mill slid across a block, the slot `4 × 3 × 20` exactly) and
`tests/flat_faces.rs` (the mill along the block's edge, a step, exactly; a bull-nose mill, within
1e-6 at 0.001 mm; the mill turned across `turn.sv`'s block, Pappus over the annulus, and turned a
whole turn, each within 1e-5; a tapered mill turned, its walls cones, within 1e-5), with
`prism_twists::a_side_square_to_the_turn_cuts_a_floor` (a bar lying along the turn, its underside
the floor, within 1e-5, an S1 refusal before), each STL agreeing with the field; what this class refuses that the closed class builds,
`flat_faces::a_mill_turned_whole_about_an_axis_through_its_bottom_bores_a_disc` (S1, a flat bottom
the turn's axis passes through, G6's `THROUGH`),
`flat_faces::a_hollow_mill_slid_across_the_block_cuts_a_whole_slot` (a ring bottom kept in its
plane, G6) and the side along a slide
(`prism_twists::a_side_along_the_slide_is_the_closed_classs`); the creased fit's
`brep::a_grid_creased_along_a_row_is_its_stretches_fits_joined_there` (through every point, each
side its plane to 1e-12); and `object_builder::a_flat_face_kept_in_its_plane_builds_at_the_displays_samples`.

**A sharp edge along one point's path in a constant-twist sheet (#684 V):** where both faces'
characteristics meet a convex corner at one point (a face's root at its end within 1e-7 of the
tool's size of the next face's there, the corner's fan a piece: `corners`, `prism`'s ribs), every
fan normal, a positive mix of the faces', is in contact there, and `Walker::step` steps straight onto the
next face (`Step::Sharp`, naming the fan) rather than round the fan, and the nodes read that step as
the fan standing at the point in no steps; a fan the walk runs round whose nodes all stand within
1e-7 of the tool's size of the point it runs onto it at (a rim's, a crease's, a corner's) is dropped
between the faces' nodes either side, by the one rule for both. Each is a crease at the node at that
point, a mark on the node naming the fan that moves with it wherever nodes are dropped or begun
again (`Node::sharp`; `Characteristic::sharp`, which names the fans; `Characteristic::creases`, with
#684 O's), built as O's is: `helical_sheet`'s exact row, no turn counted, no withheld side contact, the
creased fit (`nurbs::interpolate_creased`), no fold read across its knot, the sheet cut into faces
there. S2 counts a reaching root of a fan stepped across at the crease's point as on the walk; S3
is not read at the crease's node. A standing fan among the walk's last `MARGIN` rings is where the
walk ends, one node at its point (S1 refuses one there that reaches the blank, which the margin
leaves none). S1 refuses, naming the face, a revolved face reaching the tool's axis with its
normal off the axis (`APEX`, `constant_twist::apex`: a cone to a point, whose corner turns to no
circle; a smooth pole is none); a vertex of a prism stays refused (`VERTEX`). The gate is
`solid_sweeps_examples::the_v_wheel_slid_square_cuts_a_v_groove` (`solid_sweeps/vgroove.sv`, a V
wheel slid across a block square to its axis, the groove `2² tan 30° × 20` exactly) and
`turns_and_slides::a_v_wheel_slid_square_leaves_its_profile_creased_at_the_rim` (up the rod, the
wedge its profile cuts from the rod's circle in closed form, within 1e-6),
`an_upright_v_wheel_turned_once_leaves_its_v_turned` (`whole_turn.sv` upright, Pappus, within
1e-6), `prism_twists::a_bar_lying_across_the_turn_creases_at_its_corner` (its section turned in
closed form, which `Bar::removed` matches to 1e-9, within 1e-5) and
`a_side_facing_the_axis_turned_onto_its_rim_creases_there` (`Bar::removed`, within 2e-5 at 0.001
mm; S1 before, #684 AF), each STL agreeing with the field; the refusal in
`turns_and_slides::an_engraving_bit_slid_square_to_its_axis_grooves_its_shadow` (an engraving bit
slid square, S1, apex; the closed class builds it); and `object_builder::a_sharp_edge_along_one_points_path_builds_at_the_displays_samples`.

**A characteristic that closes on itself (#684 W):** where the walk comes round to the root it
began at (`next == seed`, the walk back taken first as before), or the walk ahead comes round onto
a root the walk back took (`met`, the loop leaving the blank for fewer rings than two `MARGIN`s),
the characteristic closes on itself within the blank's reach, a tool passed wholly through it:
`Pass::order` is the loop, its last root its first, and `Characteristic::closed` says so. The loop
is begun at a root within one piece's stretch on no fan, so the nodes neither start on a crossing
nor at a fan they drop, and the nodes are then begun again at their first crease (`creases`), the
last node that node again, each node's crease mark moving with it (the first's carried on the first
alone); `reach` is every node. S1 refuses a
loop with no crease (`SMOOTH_LOOP`: its sheet would close where it is smooth; no case reaches it)
and one under a whole turn (`WHOLE_LOOP`: a sheet closed both ways, no edge). S2's ring rule and
every-root rule, S1's spread and S3 are asked as of any stretch (S3 not at the crease node 0);
S4's section is read as a closed polygon (`crossing_of`, its first and last segments neighbours);
`fans`, `lined` and `flats` read the nodes after the first (`walked`). `helical_sheet` lays twice
`FIRST_ROWS`, forces its last row to its first, withholds no side contact on either, asks only the
columns' ends clear of the blank, adds a cut at its middle row where the loop has no crease but
the first, and cuts the sheet by `Fitted::named`'s `closed` into bands that close on each other
(`brep::build::grid_sheet` with `closed: [true, _]`; N's ring is `[_, true]`). The native path
refuses it by name. The gate is `solid_sweeps_examples::the_drum_punches_its_shadow_through_a_plate`
(`solid_sweeps/punch.sv`, a drum pushed edge first through a plate, `8 × 4 × 4` exactly) and
`tests/closed_characteristics.rs` (`a_drum_turned_through_a_ring_sector_bores_a_tunnel`, Pappus within
1e-5; `a_bull_nosed_drum_punches_its_shadow_rounded`, within 2e-6 at 0.001 mm), each STL agreeing
with the field; `brep::a_net_closed_along_its_rows_is_a_tube_of_faces` (a cylinder's net closed along
its rows cut four round and two along, its core π r² h); the refusals
`closed_characteristics::a_square_bar_pushed_square_to_its_length_is_planar_subtractions` (the
drum screwed, S2, is now the closed class's: `the_drum_screwed_bores_its_helical_groove`; a prism
aslant wholly through, S1 over a cap, and a prism turned wholly within, S1 a rim's ruling, are the
closed class's too, `closed_class`) and
`turns_and_slides::the_turn_and_slide_rows_refuse_by_name`; what it refuses that the closed class
builds, `turns_and_slides::a_torus_slid_through_the_rods_middle_grooves_its_shadow` (its rings at the
tube's top and bottom in contact all round, S1); and
`object_builder::a_closed_characteristic_builds_at_the_displays_samples`.

**A ring in contact all round (#684 X):** under a slide along the tool's own axis, or a screw about
it, where every face of the tool is a revolution about that one line (`constant_twist::along_axis`,
asked before the walk, which would find no root: each ring touches nowhere or all round), the
characteristic is the ring of greatest radius R alone, a `Piece::Round` read round it, closed
(`closed`, `reach` every node, no crease, `Characteristic::rounds` names it). R is the greatest of
each face's points on the tool at `ROUND_READS` stations, refined by `roots::brent` between the
stations either side. S2 refuses a shadow along the slide that is no whole disc (the radii of
consecutive points on one face, each stretch covering at least its ends', leaving a gap between the
axis and R: a torus's inner wall); E1 refuses a tool not passing wholly through the blank's heights
(`Screw::passing` of every point within the roll). S3 and S4 are asked of the ring. Then the sweep
within the blank is the blank within the cylinder, and `helical_sheet` writes it exactly
(`Characteristic::cylinder`; `brep::build::tube`, four faces of one `Surface::Cylinder` from the
first column's height to the last), no fit. The native path refuses it as any closed
characteristic. The gate is `solid_sweeps_examples::the_drill_bores_its_radius_through_a_plate`
(`solid_sweeps/drill.sv`, `π 2² 4` within 1e-12) and `tests/rings_all_round.rs` (a drill fed as it
turns, a screw; a lemon whose greatest radius lies within its arc; each within 1e-9), each STL
agreeing with the field; what it refuses that the closed class builds,
`rings_all_round::a_torus_slid_along_its_axis_bores_an_annulus` (a torus, S2) and
`a_drill_beside_a_post_bores_its_hole` (beside a taller post, E1; the closed
class builds it); and
`object_builder::a_ring_in_contact_all_round_builds_at_the_displays_samples`. Under a turn about an
axis a revolved face's own meets at P, `constant_twist::rounds` reads each face and convex corner
at `ROUND_READS` stations for a ring with its three coefficients nought (the amplitude turning
right round, the least sought by `roots::brent`) whose normals meet the tool's axis at a point of
the turn's, each a `Piece::Round` walked half round from its highest point about the turn's axis
to its lowest (both in the plane of the two axes, where every other ring's two roots lie), with
`support` (the tool within the sphere about P there, without, or neither: a face read 1e-3 along
it either side, a corner's fan by its normal). `Walker::sides` reads, in the coarse pass, which way
from each end the boundary goes on (`arm`, `moved`, `Round::lean`: the branch leaving the arc's
angles from the turn's axis about P), the fine pass taking it again; `Walker::arms` gives the root
that way and walks each branch the other way, which the ring trims, and the walk is not begun on
those. `step` turns onto a ring between two stations of its piece at the end it is at (S1 refuses
`CROSSED`, a ring with no support; `TRIMMED`, coming along the trimmed branch; `UNREAD_ARMS`), off
its far end by `arms`, and walks a fan holding one rather than stepping across it (dropped as V's
standing fan, a crease). S2's ring rule is deferred where the axes meet (`Walker::meets`) and lets
two roots stand either side of a ring in contact all round or on a trimmed branch; its every-root
rule passes trimmed roots; `Walker::under` asks each trimmed root in the blank to lie within the
zone of a ring trimming it, its normal facing it and the line to the zone crossing nothing of the
section. S3 is not read at a ring's ends. A ring of no radius is a pole (`POLE`) or an apex
(`at_axis`), never `ALL_ROUND`. The native path refuses a characteristic with a ring by name. The
gate is `solid_sweeps_examples::the_spoke_swings_a_slot_ended_by_a_zone_of_a_sphere`
(`solid_sweeps/spoke.sv`, `π/6 (57.25 − 1/12)` within 1e-5) and
`rings_all_round::a_torus_leaning_toward_the_turns_axis_grooves_its_shadow` (Pappus over the shadow,
2e-5, over a sector and a whole turn), each STL agreeing with the field; the refusals
`rings_all_round::what_a_turn_about_an_axis_the_tools_meets_still_refuses` (a domed pin and a pin
wholly within, S1 `POLE`; an upright torus across a taller block, S1, its ring kept in a plane; a
small leaning torus wholly within, S1 `SMOOTH_LOOP`); and
`object_builder::a_ring_in_contact_all_round_turned_builds_at_the_displays_samples`. A slide square
to the tool's axis, or a turn about an axis parallel to it, stays refused at S1.

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
at its own end), and `SweepContacts::creases`
pairs each side's halves. Admission asks each half as a side; a half's end at the plane is no rim
(T1 asks caps by their rims, and a half's end in `creases` is none); and the crease is asked as a rim is, by one
pass over both (`admission::Fan`): each
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
(T1 refuses it, saying so; the closed class reads such a tool by its B-rep where it takes its faces,
`union_tools::a_union_within_an_intersection_is_the_closed_classs`; members falling to the closed
class and another each built by its road, `union_tools::a_member_its_class_refuses_is_the_closed_classs`), and a union of prisms under a motion keeping their plane, which
`planar_subtraction::asks` reads whole. The native path refuses a distributed cut by name.
`tests/union_tools.rs` and `solid_sweeps_examples.rs` (`union.sv`) are the gate: the beads' groove
against rows across the rod computed apart, placed and turned, against the long form, two balls'
tubes by Pappus, two tori rolled in the generating class, and the refusals.

**A prism's cap in the blank (#684 F):** a prism standing alone as the tool (its solid the prism
itself, or the prism placed, #684 AC; undrafted, its profile one loop of lines and arcs:
`admission::Lone`) whose cap passes through the blank is no longer refused at T1; any other tool's
cap is, saying so. A placed prism is read through its placements: its faces carried by their pose
(`SweepContacts::read`), its cutter the prism's with its profile moved by them (`Cutter::prism`
follows the recipe's `placed` nodes to the prism). The same reading builds a placed prism whose
caps are clear, which was admitted but whose cutter went unread, so its export was refused
(`rack_cut::a_placed_tooth_cuts_as_the_tooth_does`: the rack's tooth placed along the rack cuts as
the tooth does, to 1e-5). Its caps are faces of the
tool (`ToolSurface::Cap`, read over the box its profile lies in, a point off the profile passed
over as the field says), asked M2–E4 and E1 with its sides only where one reaches the blank; M2
refuses only a point in contact at every time (a constant nonzero condition touches nothing — a
side square to a rack's line of centres). At each point of a reaching cap's rim, where a face's
contact in the blank meets the rim, the face's envelope stepped onto the rim and the rim's fan
stepped off it must point the same way, or E3 refuses the fold (a flat end swung so its corners
dig past it), each fan normal touches the blank once (E2), and the fans' contacts join E4's. **T3** then chooses the direction across the
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
`object_builder::a_bars_end_builds_at_the_displays_samples` builds it in the page. A placed prism's
gate is `next_tooth.sv` (`solid_sweeps_examples::the_next_tooth_leaves_the_bars_space_turned`): the
bar placed a quarter turn of the roll along the rack and rolled a quarter turn earlier, cutting
the bar's space turned round the disc, within 1e-6 of its volume, its STL agreeing with the field;
`object_builder::a_placed_bars_end_builds_at_the_displays_samples` builds it in the page.

**Every class has its simplest case in `rust/examples/solid_sweeps/`** (#684): one short document
a class — the canal (turned, and along a spline), a slide, a turn, a screw, a screw with sharp edges, the generating class's rack, its
drafted helical rack, a tooth drafted both ways (`parted.sv`) a prism's cap (`bar_end.sv`) and a placed one's (`next_tooth.sv`), a prism's cap under a turn (`flat_end.sv`) and a prism drafted both ways under one (`creased.sv`), a sharp edge along one point's path slid (`vgroove.sv`), the planar subtraction class's conjugate wheel, a union's members — its README row naming the class, and `tests/solid_sweeps_examples.rs`
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
(`envelope::planar::InnerEnvelope`, the kernel's `brep::swept::planar`, #721 F2c, its curve a
`Section` and its material a `Region` the solver reads off the drawing, its refusals typed
`Seen`/`Row` and said by `envelope::planar`; over a formula or generated source: contact roots per sample,
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
`Subtraction::key`) read as one. In each, `envelope::swept_region` reads the region (the kernel's `brep::swept::region`, #721
F2c: the section's material its members' `PlanarField`s joined, `swept_region::section`, its
refusals typed and said by `swept_region::said`) `R = ∪ₜ
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
`planar_subtraction::admit`; a whole roll never clear of its body is refused saying the period
to state. Every period is found by one search (`roll::first_alike`: a degree at a time, none for a
tool the motion does not move) and said in one form (`roll::advice`). `tests/planar_subtraction.rs` is the gate
(the #585 wheel against arithmetic made apart, the long form, two bands as one, a stepped tool,
one arm through, Q3, the drafted wheel at 10 µm, the measured drafted handle above six bands
stepped down it and the square arms, a drafted disc whose islands change refused at their depth,
#613's pinion cut at a copy turned 30° and whole, and at module 1.5 over a roll ending as its flank
grazes the held one or its contact leaves it at its corner just before, stated and whole).

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
