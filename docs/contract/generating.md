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
`tests/carried_family.rs` (the springs, the screw's walls, a station read otherwise).  A face
swept over an interval written in place under a screw, a slide or a turn (#846, [0.208]) is the
same carry with no station to solve: the recipe's `carried` node, the face's profile placed where
the interval begins and carried by `brep::build::carried` (`screwed`, `ruled` or `revolve`), its
`carry` written by the one writer a family's is (`cad::carry`) — [sets.md](sets.md)'s.

**A frame's numbers are stated or solved** (#383 stages 3 and 5): a ratio or advance measured
off the solved drawing (`ratio: -length(a) / length(b)`) is E107, as every measurement in a
number is, and the measured motion numbers of 0.27 are gone (`measure.rs`, `MotionE::measured`).
One no `fix` states is an unknown of the solve — [frames.md](frames.md)'s joints in the solve —
read after the solve where the solve left it (`MotionE::rotation`, `advance`, `Family::read`'s
`Joint`, `solid::reads`, so a cache key moves with it), and refused by the planar program
(`generated::program_signed`), whose numbers are constants of its tape.

**Solid placement:** `indexed := source at(under: indexing) 90deg` places a solid at a
motion's pose; `at:` is a constant Angle, not a hint or sweep. Repeatable, nestable, Boolean
operands; copy keeps source and motions, and the source stays an object of its own unless it
is `construction` or something else is made of it ([app.md](app.md), #904 A10). `make
solventc OCCT=1` enables native STEP via a C ABI over OCCT; the core supplies construction data, the CLI owns shapes and catches exceptions. A
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
**A sliver the B-rep holds is said wherever it stands** (#836, `brep::export::slivers`): a face of
the built solid thinner than the export's tolerance somewhere (the fabrication tolerance where none
is named), or an edge shorter than it (`brep::sliver::slivers`), is said beside every file written of
it — the STEP, a written STL, `--step-assembly` for each part, and `--png` of a solid drawn from
its exact build (`slivers_drawn`) — as `solventc: PATH: …`, in the core's words: the solid, how many
faces and edges, the first by name and how thin, the faces it is thin between, a point on it (the
witness), and what to move. A warning, not a refusal: the B-rep is exact and holds what was drawn,
the ends nearer than the Booleans' reach having been made one (brep.md, `brep::flush`), so what is
said is a step drawn under what can be made
(`brep_export::a_sliver_the_brep_holds_is_said_wherever_it_stands`).
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

**A helical rack's flank is its transverse envelope carried up a helix (#826):** an undrafted prism
of lines and arcs whose extrusion leans along the rack's slide, under a slide seen from a turn (a
helical rack against the blank it cuts), is the same set slid along its extrusion, so its section
by a plane square to the turn's axis at height `h` is the base section slid `h·shift` along the
rack's line, and the roll carries a section so slid to its own envelope turned about the axis:
`E_h(t − lag·(h − b)) = R(rate·(h − b))·E_b(t)`, `rate` = `tan β / r` for a rack on a pitch circle
`r` (`brep::swept::extruded::Screwed`, read where `planar_sheet` reads no plane the motion keeps;
anything else — another curve in the profile, a drafted prism, a lean across the slide, any other
motion — is traced). An arc of the profile, a hob's tip round (#904 A6), is sectioned as an
ellipse, its circle's affine image stretched along the lean, on its principal axes; an ellipse
cuts in closed form as a circle does (`planar_cut::Cutting::Ellipse`: the root of `F` Newton
finds from the ellipse's support point toward the instant centre, refused unless its outward
normal faces that way on the side asked), so the round's trochoidal fillet is carried up the
helix with the flanks, not traced and fitted (it was 1.7e-2 mm and 4° off its contacts). A
corner where the section runs on smoothly, a round's end, is convex with an empty fan. Its
heights are an arc's at its ends and where its height turns within its span.
`generation::an_ellipse_cuts_as_its_circle_does`, `helical_rack::a_helical_rack_with_rounded_tips_is_screwed`
and `…_cuts_its_space` (the space against the rounded transverse tooth's arithmetic). The sheet (`brep::sweep::extruded::screwed_sheet`) is the base section's
envelope read as a prism's is, its Hermite spans joined at knots of multiplicity two within a piece,
carried by the helix's cubic fitted within an eighth of the bar (`helix_fit`): each section pole
turned and raised by each helix pole, so the net's height runs with `v` alone. The identity is
held at both ends of the band to `1e-8 (1 + reach)` mm, the sheet's rims at those heights out of
the blank, and its contacts' normals are the helicoid's. A plane cutting a net whose height over it
runs with `v` alone, strictly, is cut along that iso line exactly (`ssi::level_iso`, a net of one
linear span across left to the case after it), so the blank's faces meet a flank in a pcurve that is
a line. `tests/helical_rack.rs` is the gate: every exact contact the trace finds within the
sheet's fit, the space's volume against the transverse tooth's arithmetic, and the 20-tooth gear's
STEP within twice its spur twin's entities (radiatorbot's 19-tooth, 25° gear: 107 623 entities
and 9.7 MB traced, 32 155 and 2.7 MB screwed, its spur twin 17 901).

**A ball swept is its tube** (#238, #684 D, `solid::canal`; the tube is the kernel's,
`brep::swept::canal`, #721 F2c): a sweep whose tool's B-rep is one
ball (`brep::swept::canal::ball`: every face on one sphere, however it was drawn, #721 F1c), under any motion, is the points within the radius of its centre's
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
(`dispatch::Asked::Canal`, asked first by the kernel's dispatch, `brep::swept::dispatch::sweep`):
**B1** the path never stops and turns nowhere tighter than the radius; **B2** it comes back within
the diameter (and the points' spacing) of itself nowhere half a turn of a circle of the radius
along it from (`canal::refusal`).  **A path that closes on itself** (`brep::swept::canal::ring`,
an offset's tube about a closed edge) is a ring with no end: the frame carried round comes back
turned about the tangent by the path's holonomy (none on a plane), unwound evenly along the roll —
the disc is round, so the same tube — its last station its first to the bit, B2 read the shorter
way round; built by `brep::family::ring`, two faces on one sheet whose `s` runs round its interval
(`Sheet::ring_period`, so a foot past the seam is read across it, never held at the end).  The
kernel refuses by `Row` (B1 `Curvature`, B2 `Return`) and
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
the sign of the area factor times the contact condition's rate. A fold on a flat face under a
rack's roll (a slide square to the turn's axis) whose contacts in the blank reach past `R sin² αₜ`
inside the pitch line — R the slide's advance a radian over the turn's rate, αₜ the face normal's
slant from the slide with its part along the axis dropped — is an undercut, and E3 says it so
(`brep::swept::admission::Undercut`, #822): the flank, the base circle `R cos αₜ`, the reach and
the depth allowed, in millimetres, and the rack's shift, the teeth and the flank's end that clear
it; a fold that is not one keeps E3's own words, which name the tool face whose generated surface
folds (a crown's flank that undercuts its member: which blade to change, #904 A2;
`hypoid_machine::at_no_offset_the_pinions_two_flanks_tie`). `a_helical_racks_undercut_is_said_with_its_numbers`
(14 teeth at 25° refused so, 17 admitted). Building a leaned rack's undercut, as the planar
subtraction class builds a spur rack's, is left open: its tooth is no prism square to the plane. Built in
`gcs-cli/src/cad/native/sweep_boundary.rs`: cutters sectioned (`backend/sections.cpp`), contact
times from `SweepContacts::at_point_normal_over` (`PointContactError`), sheets traced and
resampled by the core (`solid::contact_trace`, fed section samples and nothing else; rows stop
outside the blank before a time leaps: `charted`, leaned across the stations where a leap pins
them in it: `sheared`; fall by walk length, or by length in space where
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
evaluated as a pose and its rate (`General`, K2) — among them a frame carried along a B-spline
from its start (`trajectory::Along`: the rotation-minimising frame by double reflection on a grid,
from a home anywhere on it, or the curve's own (Frenet) frame, over any curve that reads its jet
(`trajectory::Guide`: the kernel's B-spline, the drawing's path — `motion::guide` holds no copy),
its stretches enclosed by the curve's hodographs, a rational curve's and the Frenet frame's failing
closed;
`a_frame_along_a_curve_turns_only_as_its_tangent_does`), so a caller of the kernel sweeping along
a curve writes no path of its own.  `Family::trajectory` lowers a family step by
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
revolution being `brep::swept::surface::RevolvedSurface`, its edges, creases and `Material`; whether a
point is on the tool, `Tool::on`, is `Material::near`, which a B-rep's `read::Boundary` reads off
the faces and edges within the threshold alone, the first within it deciding, where `value` reads
every face and edge nearer than the best found — a traced edge's foot by projections onto both
its surfaces, asked at every sample: a hypoid gear's admission 9.6 s, now 3.8 s) and the
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
edge's faces' summed normals inside an edge, and at a corner by the corner's angle-weighted
pseudo-normal (each face's normal there weighted by the angle its loop turns through at the corner,
F1b), or by the boundary's point location where none is read (an apex, a pole).  A face no reading takes is refused by its
geometry with a witness on it (`Unread::FreeForm`, G10: a B-spline, blend, face family, extrusion
or affine image; `Unread::Meridian`, G8: a revolution of another curve; `Unread::Unfit`).  The
classes read the tool this way since F1b (below).  The drawing's reading, the oracle the B-rep
reading was held to while the classes switched (F1a, F1b: the two readings one boundary, one
material, one set of corners and one walk over every example), is retired (F1c), and
`tests/tool_reading.rs` holds the reading to the B-rep it reads.  The gates are these:
- `the_catalogues_tools_read_from_their_boundaries`: every tool of the sweep catalogue is read, of
  every kind, and a refusal names a face of the B-rep with a point on it.
- `the_reading_bounds_its_brep`: each point of a read face on the tool lies on the B-rep's boundary
  by its point location, and each point of a B-rep face clear of its edges lies on a read face
  square to its normal.
- `the_reading_is_its_tools_material`: the reading's material agrees in sign with the drawing's field
  of the tool, the semantic reference, off the boundary, and reads nought on it.

These run over the sweep catalogue; `every_examples_tools_read_from_their_boundaries` is the slow
tier's, over every example.  `a_face_no_closed_form_reads_is_refused_by_name` holds the Wankel's
epitrochoid bore to `FreeForm`, its witness on the face.

**The classes switched onto the B-rep reading (#721 F1b):** `SweepContacts::read` builds the tool's
recipe into a B-rep and reads it (`Tool::read`); its material is the boundary's.  The recipe gates
`admission::tool` asked of the drawing are retired: T1 is now the reading's refusal, a face no
reading takes or a tool the kernel builds no boundary of, named with a point of the face where the
reading gives one (`dispatch::sweep`'s T1, #721 F1c).  A tool is one prism standing alone where its
B-rep is (`admission::Lone::read`): every face it reads a side running square along one direction,
no crease, and two caps, the boundary's planes square to that direction, each one loop of lines and
arcs; its cutter is that direction's prism of the nearer cap's loop (`Cutter::extruded`), however
the tool was drawn — a prism, one placed, a box a Boolean leaves — and the generating class's
prism sheets, across a cap (T3) or not, and the planar envelope's, are sectioned by that cutter
(`SweptCut::lone`), so admission's band and the build's stations are measured from one origin.  A
cutter of revolutions is still sectioned off its recipe (`Cutter::read`), F1d's.  So G9's
negative controls are admitted where the B-rep is ordinary faces: a bar a `common` leaves whole is
the bar, its cap cut as the lone bar's (`prism_caps::a_cap_of_a_prism_a_common_leaves_whole_cuts_as_the_bars`,
`prism_twists::a_cap_of_a_prism_a_common_leaves_whole_turned_cuts_as_the_bars`), and the rack's
tooth united with itself is the tooth (`rack_cut::the_tooth_united_with_itself_cuts_as_the_tooth_does`).
What the class still refuses it refuses by the geometry it reads: a plane that is no side and no
lone prism's cap passing through the blank (T1, asked by its own points where no strip's rim bounds
it: a partial revolution's end, a slanted cut, `prism_caps::a_cap_of_a_tool_no_lone_prism_is_refused`;
under a constant twist only where the motion's contact lies on it, #721 F1d below);
a crease where a drafted prism's halves fold, concave where it is drafted outward both ways (T2, as
a concave corner is, by the chord between points of the two halves beside it); a drafted corner
where an arc meets a side at an angle, which the kernel builds as the conic its plane and cone meet
in (#962) and the class pairs no corner of (T1, G8, `swept::read`'s `Tool::read`; and the cutter,
whose sections are no blends of its ends: `drafted_tools::a_drafted_tool_the_class_does_not_read_is_refused`).
The faces are named as the B-rep names them (`read::named`: the solid's key and the face's own, a
dot between), so a prism's caps are `far` (at its least height) and `near`, not the drawing's
`start` and `end`, and a corner names its faces in the B-rep's order.  Three notes F1a's review
left are settled here: a corner is probed where it lies on the tool (`Tool::corner`, round the turn
or out along the sides from the first place), not at a chart's start; a corner's side is its
pseudo-normal's, which retires the material's mesh; and faces a Boolean cuts from one surface whose
readings are one surface's alike are kept once (`read::once_a_surface`).  The two notes it left —
faces of one surface over different spans of its meridian, and a concave corner in the walk — are
settled in F1d, below.

**The dispatch is the kernel's (#721 F1c; `brep::swept::dispatch::sweep`):** one `sweep` takes the
tool's B-rep (millimetres), its trajectory and its roll (model units), the names a refusal says,
the blank at its placements (`dispatch::Blank`: membership, its support's box, its own boundary's
box) and how finely to sample (`dispatch::Ask`), and asks the classes in order, building by the
first that admits: the canal (a B-rep every face of which lies on one sphere, `canal::ball`; its
rows' refusal stands); the planar class (a stock cut by one pocket, every placement asked, its
refusal standing — the pocket read from the tool's B-rep, `Pocket::read`, F1d below); the planar subtraction class outright under a constant
twist (its tool read from the B-rep, `swept::subtraction`, below); the constant twist's rows under
a screw, a turn or a slide and the generating class's under any other motion (the tool read by
`Tool::read`, its refusal T1 with a witness; the motion charted, M1; the first placement asked and
each later one read alike to an earlier, by the drawing's proof where the blank is the same under
every turn about one line, `Ask::revolved`, else at every point the first's rows read, or asked
itself); the planar subtraction class in their place where they refuse and it reads the tool; and
the closed class (`closed::reads`, `closed::reads_general`) where every class asked refuses, its
own refusal never standing for theirs.  Every refusal is typed (`dispatch::Row`, which the solver's
`admission::Condition` is; `dispatch::Asked`; `dispatch::Seen`: words, a ball's path the canal
saw, or a motion with no chart) with a witness in the body.  `admission::admit_body` keeps the
drawing's reading — each sweep's tool built into its B-rep once (`cad::tool_brep`, remembered
against what the sweep reads), its motion lowered, its roll, the blank's field and boxes, the
placements and the proof that they read the blank alike, the pocket — asks the kernel for each
sweep side by side, and says the refusals in the drawing's words (a ball's path, an uncharted
motion, a member of a union tool).  A ball's tube the dispatch admits is static material the
drawing has already lowered; one asked here is one whose tube its gate refused, and admission says
the gate's refusal.  The recipe reading of the tool is retired (F1 decided): the drawing's reading
of a sweep's faces (`SweepContacts::drawn`, `ReadCap`, `ReadExtruded`) and its oracle tests are
gone, the canal reads its ball and the planar subtraction class its section from the B-rep.  A
sweep depends on the set: an end mill slid square to its axis, a drum turned about a line along
its own and a bar lying along a turn stand along the normal of a plane their motion keeps, and are
the planar subtraction class's now, before the constant twist's rows, each to the closed form it
was cut to before; a box rolled by two turns about parallel axes and a stepped disc's concave
corner, which the generating rows refuse at M2 and T2, are that class's in their place
(`admission::a_prism_whose_cap_passes_through_the_blank_is_planar_subtractions`,
`a_concave_corner_carried_through_the_blank_is_planar_subtractions`).  What F1d keeps open is below.  The gate is
`tests/swept_dispatch.rs`, in the kernel alone: a drum built as a disc's prism and as a rectangle's
revolution dispatched alike, the region each sweeps through a block one to 1e-9
(`a_drum_drawn_two_ways_is_one_tool_to_the_dispatch`); a ball the canal's, turned or slid; a tool
with no B-rep refused at T1 in the drawing's words.

**What F1b left (#721 F1d).**  *The drawing's field:* `SweepContacts::read` reads no field of the
drawing: the tool's material is its B-rep's (`read::Boundary`), and the accuracy meter trims a
generated foot to the cutter's own boundary, and reads which way it faces, by that material
(`SweepContacts::source_material`), not by the source's `SpatialField`.  It takes no axis tolerance
for that reason, and the planar class's stock is the B-rep's too (its pocket, below).
*The chart's start:* `Tool::read` starts each revolution's chart where its own surface's frame
does (`Reader::start`), no longer in a half-plane shared by every face about one axis, a convention
kept only so the walks could be held node for node to the drawing's.  Nothing the sweep reads
depends on where: a corner is probed with its second face at the first's half-plane
(`RevolvedSurface::turn_of`, `Tool::corner`); a convex corner's fan reads both faces' normals in the
half-plane of its ring's start (`constant_twist::corners`, which before read each at its own chart's
start and saw a fan at a smooth join where two charts started apart); faces a Boolean cuts from one
surface are told by their meridians about one axis (`read::once_a_surface`); and the walk crosses
from face to face by position.  The gate is `tests/swept_charts.rs`
(`a_walk_is_the_same_curve_wherever_its_charts_start`): a bull-nosed drum pushed through a plate,
drawn in three half-planes and dispatched, and its reading walked again with each face's chart
started further round (`RevolvedSurface::started`), every walk one curve to rounding.
*Prisms drafted and with holes:* `admission::Lone::read` reads one prism standing alone from its
B-rep drafted or not and of any loops: two caps, the boundary's planes square to the prism's
direction, every side's rulings along it or leaning to it alike — one draft one way, or out to a
parting plane and in past it, the halves meeting in creases (#684 G, P) — each side's draft read
as how far its ruling runs out across its outward normal square to the axis per unit along it.
Its cutter is that direction's prism of its widest section, the nearer cap's loops undrafted,
the wider cap's drafted one way, the parting plane's (the low cap's moved out to it) drafted both
ways (`Cutter::extruded`), laid from the cap the sides' rulings leave so its stations are measured
as before.  A cylinder or cone about a line along the prism is a side of it too (a round hole's
wall), and a cap a round hole passes through, read as flats of its wall, is a cap.  So a drafted
prism's sheets are sectioned by the cutter its B-rep reads (`SweptCut::cutter`), not the recipe's,
and the drafted racks' spaces are cut to the same volumes (`drafted_tools`).  Its caps are cut
too (G9): the constant twist walks the cap of any prism standing alone, drafted or with holes (a
drafted plane side's rim is a line, `ExtrudedSurface::straight`; a cap's contact line is cut where
the material leaves the cap, `cap_lines`), and T3 sections across a prism undrafted or drafted
with plane sides, of any loops (`Lone::sectioned_across`, `Cutter::across`): each stretch of it
along its extrusion whose sides run straight between its end sections (`Ends`: the undrafted
prism's one, a drafted prism's halves) carries each chord's ends up a ruling of each side, the
halves creased where they meet; a hole parts a section into chords, one loop each, and its ends
along `across` bound the stations as a corner does (`Spread::corners`, the stations run between the
nearest ends of holes either side of the band).  A drafted arc's point moves along a conic, whose
sections across are not built: such a prism's cap is refused at T1 as before.  The gates are
`drafted_tools::a_drafted_prisms_cutter_is_read_from_its_brep`,
`prism_twists::a_drafted_bars_flat_end_turned_cuts_a_floor` and
`a_bar_with_a_hole_turned_cuts_as_the_bar` (a hole clear of the end's contact line; one across it
parts that line, refused at S2, and the closed class builds it),
`prism_caps::a_bar_with_a_hole_cuts_the_floor_of_its_space` and
`a_drafted_bars_end_cuts_the_floor_of_its_space`, and
`swept_dispatch::a_prism_with_a_hole_is_one_prism`.  The helical rack's screwed sheet
(`brep::sweep::extruded::screwed_sheet`) takes the B-rep's prism too: its section and band are the
recipe's to rounding (2e-15), and the blank's faces cut both nets along iso lines alike; only a
traced meeting's sample count moves with how the tracer stepped (the outer rim's with a flank, 139
poles where the recipe's took 100), so
`helical_rack::a_helical_gears_step_is_about_its_spur_twins_size` bars a curve of 200 poles, still
far short of the many hundred #826 removed.
*Cutters of revolution off the recipe:* a tool whose B-rep is revolutions about parallel lines —
every face a cylinder, cone, sphere, torus or revolution of a line or circle about a line parallel
to the first face's, or a plane square to them — is sectioned from its B-rep
(`Sectioned::of_boundary`, `Cutter::of_boundary`, read once in `SweepContacts::read` as
`SweepContacts::revolved`), each face read as its surface's meridian in its own line's half-plane
over the span of it the face reaches, its ends on the circles its edges lie on, each step naming
its face.  Where every face is a whole turn about one line, every half-plane's section is the same,
the meridians joined end to end and closed along the line; else each half-plane is sectioned as it
is asked: each face's meridian carried into it (a `Chart`), or a plane's line across it, split
where another meets it, and the pieces whose middles lie on their faces of the B-rep kept and
joined into loops, so a crown bounded by its neighbour (the spiral bevel's and the hypoid's
cutters) is sectioned by its faces as its recipe's Boolean was.  `SweptCut::cutter` takes the
B-rep's prism, else its cutter of revolutions, and reads no recipe: `Cutter::read` and its recipe
prism are retired, and `Sectioned::of` a recipe stays only as the oracle the B-rep's sections
are held to.  The gate is
`tool_reading::the_catalogues_cutters_of_revolution_section_from_their_boundaries`: every catalogue
sweep the recipe sections as revolutions (23) the B-rep sections alike, the two sections at five
half-planes one region by their exact areas and a grid of points read in space; the slow tier's
`every_examples_cutters_of_revolution_section_from_their_boundaries` holds every example's 43,
the hypoid's and the spiral bevel's among them, none refused.
*Concave corners in the walk:* T2 is the generating class's row alone; the constant twist meets a
concave corner in its walk.  Where both faces' points meet the corner (`Junction::Edge` with no fan,
the roots one point), the walk crosses straight from one face to the next and marks the node a
crease (`Node::crease`), the normals turning there by more than `LEAST_TURN`: the sheet is fitted
creased along that point's path, as at a fan standing at one point (`Characteristic::creases`,
`Characteristic::concave`).  So a tool drawn with a concave corner — `union.sv`'s beads as one
revolution of their two discs' outline, or their union within an intersection, which no union
distributes — is the constant twist's, the rod losing what either bead's shadow covers to 1e-6
(`union_tools::a_tool_creased_at_a_concave_corner_cuts_as_its_members`,
`a_union_within_an_intersection_is_the_constant_twists`).  So a union that is one solid under a
constant twist is swept whole, as its set (`distributed::whole`: its B-rep's faces joined into one
solid and read by the kernel), not member by member: `union.sv`'s beads are one sweep, the same
groove as their members' to 2e-7 (`union_tools::the_union_cuts_what_its_members_cut_in_turn`), and
the grown bead its class refuses alone is built whole with its twin
(`a_member_its_class_refuses_alone_is_built_whole`).  A union of separate solids, or one under a
turn seen from a turn, is still its members' sweeps (#684 C).  Where the faces' points part at the
corner, the walk ends a stretch there (`Step::Concave`) and walks the others from the roots in the
blank no stretch holds yet (`Pass::others`, `Characteristic::others`), each ending at a concave
corner or past the blank; S2 takes a ring's two points in two such stretches.  Each stretch's sheet
is fitted on its own, its edge at a concave corner left in the blank (`Characteristic::concave`,
the edge check passing it over), and the blank is split by all of a sweep's sheets at once on one
arrangement (`Fitted::parted`, each sheet an operand of its own, `sector::whole`), each cell its
own selection so a sheet's end within a cell bounds nothing, the cells judged by the body's field
as a split's: so each stretch is trimmed against the next where they cross.  The gate is
`union_tools::a_characteristic_parted_at_a_concave_corner_cuts_as_its_swept_set`: a drum stepped
down through a cone, slid aslant past a plate's strip, the contact on the cone leaning off the
drums' line so the stretches part at the concave corner, built to 6e-7 of what the closed class
cuts from the plate by the tool's set alone (`closed::cut_from`).
*One reading a surface:* the faces a Boolean cuts from one surface of revolution are read once,
over the span of its meridian they reach together (`read::once_a_surface`: a reading lies on
another's surface where its meridian lies on the other's line or circle about one axis, the charts
outward alike, `one_surface`; the first's meridian carried past its ends over the hull,
`RevolvedSurface::meridian_param`, `RevolvedSurface::over`), and a corner where another face meets
a face of revolution along a ring partway along its meridian is read there (`PatchEdge::ends`
strictly between nought and one).  A part-cylinder's kind is no longer its loop's: a face over part
of a turn that shares a circle with a face read about that circle's axis — a part of a drum's,
cone's, ball's or ring's surface beside it, or a plane square to the axis — is read about the same
axis before any face is read as a strip (`Reader::beside`), so the faces of one turn read alike and
meet at corners along their meridians; a part-cylinder no face read whole round meets is a prism's
side, a strip, as before, and T1 asks a strip's rim only where it lies in a cap's plane (or a
lone prism's flat's: a cap a round hole passes through).  The walk reads a corner partway along a face: its stations hold the
corner's (`cut_at`), and where the walk stands on it and the face goes no further its way at that
point (`Walker::partway`, the material read just past it), it crosses the corner as at a face's
end (`Along::cuts`: onto the face beyond, round the corner's fan, onto a flat's line); onto a face
partway along it, it goes on the way the face is the tool's from that point (`Walker::onward`).
`Tool::corner` probes such a corner on the side its face goes on, and first at its own point
(`PatchEdge::at`, the middle of the B-rep's edge it is read off, within both faces), so not at a
partial revolution's chart start, where the chord would lie in its end plane.  No strip meets a face of revolution at a ring: a strip's
rim lies along a ring only about the strip's own axis, so a face over part of a turn that no strip
reading takes (a ball's, a ring's, a revolution of an arc) is read as a revolution first, and the
faces sharing its circles are read about its axis with it; the corner between them is a corner of
two revolutions, walked straight across where their points meet and round its fan where they
part.  A plane bounded by circles is a flat from its centre where its loops wind about it,
whatever radius they reach (a notched disc), and a corner the B-rep's edges split is read once.
The gates are `tests/swept_partway.rs`: a drum with a window ground down through its side and top
by a quarter-turn groove, its side read once whole round, the window's floor a flat and its back
wall a part-cylinder about its axis, the floor's corner partway along the side's meridian and the
back wall's partway along the top's (`a_windowed_drum_is_read_a_surface_at_a_time`), and turned
about a line square to its axis through a blank, its characteristic walked up the side, across the
floor and up the back wall, every node on that path to 1e-15
(`the_walk_crosses_a_corner_partway_along_a_face`); and the bull-nosed drum turned three quarters
of a turn, its side read about its axis with its rounds, the push's silhouette in what the three quarters
keep, pushed through the plate, its characteristic the whole drum's curve to 2e-15
(`a_partial_revolution_is_walked_as_its_whole_turn`); and a barrel whose side meets its ball
ends at a sharp corner, slid aslant so the faces' points part there, turned three quarters, its
characteristic run round the corner's fan and the whole barrel's curve to 2e-15
(`a_corner_where_the_points_part_is_walked_round_on_a_partial_revolution`).
*A partial revolution's ends:* the drawing's field reads a revolution over part of a turn
(`RevolvedRegion::spatial`: its whole turn's material in common with the wedge its turn spans, a
sector about the axis carried along it; `RevolvedRegion::classify` the same), where before it
refused any but a whole turn; and the kernel reads it by its faces, its ends planes through its
axis.  A plane that is no lone prism's cap is asked at T1, under either class, only where the
motion's contact lies on it at some pose of the roll sampled (`admission::holds_none`: `n·v` read
over its points on the tool at every pose, one sign throughout a plane that holds none): an end
the motion's normals never lie in is nothing of the sweep's boundary but at its end poses, and
passes through the blank unrefused (the three-quarter drum above).  A plane the contact crosses is
still refused at T1, no class reading a contact line across it but a lone prism's cap's; and a
turn of more than half a turn meets itself along its axis at a concave edge, which the generating
class refuses at T2 as any concave corner.  The gate is
`partial_turns::a_pin_turned_part_of_a_turn_is_the_generating_classs`: the lantern pinion's pin
turned 160° is admitted by the generating class as the whole pin is
(`revolved_regions::material_queries_are_snapshots_and_refuse_unsupported_or_nonfinite_geometry`
holds the field's half turn).
*The planar class's pocket:* read from the tool's B-rep (`pocket::Pocket::read`): two caps, planes
square to one direction each bounded by an outline and one hole, every other face standing along
it (a plane, a cylinder or a sweep of a curve along it); the hole the loop enclosing the less
(`Brep::loop_area`), its edges' curves walked as the loop walks them and read as one curve's parameter where they are
stretches of one curve end to end (`Hole`, a `planar::Section`).  The B-rep carries a point's path
under turns and slides at steady rates exactly: `generate::point_path` reads the drawing's envelope
of a point as terms `(a + b t) e^{iωt}` off its turn and slide ops (`brep::trochoid::Trochoid`,
its points and derivatives in closed form at every order, its speed bounded over any stretch, an
`Exact` curve), the recipe's profile writes its two halves as `trochoid` edges
(`cad::trochoid_halves`), and the extrusion's side over it is cut by a plane in closed form
(`ssi`).  So the Wankel's bore is its apex's path in the B-rep to rounding, the housing's volume
its closed form's to 1e-9 (`wankel::the_housing_is_its_case_less_the_bore`), and the rotor its
inner envelope read off that curve (`InnerEnvelope::new`), built to 1e-6 of its closed form
(`wankel::the_rotor_is_admitted_to_the_planar_class_and_built_as_its_closed_form`).  The stock's
material is its outline's prism (the outline's lines and circles read as themselves), so the bore
is no boundary of it, where the rotor's apex touches it; a loop's material in the plane is
`subtraction::Loops`, its edges held in a tree so a reading visits only those that may be nearest
and those the ray from the point can cross.  The dispatch is told only
whether the drawing writes the tool as a stock cut by one pocket (`Sweep::pocketed`), for which
the class's refusal stands; the drawing's `planar_class::read` and `Sweep::pocket` are retired, and
a refusal of the envelope is said in the motion's words (`Seen::Envelope`).

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
a whole slot exactly), an engraving bit slid square to its axis (G5,
`turns_and_slides::an_engraving_bit_slid_square_to_its_axis_grooves_its_shadow`, its shadow's
triangle within the rod's disc in closed form; the dispatch asks no walk of it, the bit standing
along the normal of the planes its slide keeps) and a drill beside a post it does not pass wholly
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
and turned to a stop partway (`closed_class::a_drum_turned_partway_into_a_ring_sector_stops_there`),
each to 1e-9.  (The drum turned whole through a ring sector, the constant-twist class's S1, stands
along the turn's axis read from its B-rep, and is the planar subtraction class's since #721 F1c,
`closed_characteristics::the_drum_turned_a_whole_turn_bores_the_whole_ring`, within 1e-6.)
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
the loop is docs/testing.md's). A build refused in its trim writes what the trim was given
(`dump::dump_trim`, #942 F5): the tool, the trajectory by its structure, the roll, the bar, the trim's
reading, the candidates with their face families' walls, rails and rims, and the given meetings;
`load_trim` reads them back, every candidate and meeting bit for bit
(`swept_general::the_general_roads_candidates_are_written_and_read_back_whole`), and its replay
refuses as the build did (`swept_general::a_refused_trim_replays_from_its_dump_to_the_same_refusal`,
and in the slow tier the bench's tumbling cube). A frame carried along a B-spline
(`trajectory::Along`) is written as its curve exactly, its home, which frame and its name, and made
again from them (`Path::along`), its poses and rates read back bit for bit
(`swept_rings::a_frame_carried_along_a_spline_is_written_and_read_back_whole`, #942 T1d-ring);
any other path read only by evaluating it, a pace and a path of pieces stay refused by name.

**A piece is one verdict all over, or a meeting is missing, and the trim completes it** (#942 F2,
`trim`'s `sided`, `missing_across` and `completed`): each piece is read at up to eight of its inner
points, the first that settles its verdict and as many as seven more that hold it there. Two that
differ, the one read off the boundary proved clear of it by `CLEAR` hairs (`Swept::beyond`), are a
witness: a meeting the trim was not given parts them. Where the result does not close, each edge it
uses once is read across, `ACROSS` hairs either way on every candidate face through its middle: the
face kept there read clear of the boundary, or a face not kept read as the boundary, is a witness
with the point its piece was read at. From a witness the face is walked between its two points in
its own parameters (`Located::uv`, the branch of a folded wall the face is on), and wherever another
candidate's face crosses the walk where no given curve passes, the two surfaces' meeting is traced
from the crossing (`ssi::trace`, else marched in their parameters), added to their given meeting or
given as a new one, and the trim run again, up to `ROUNDS` times. The gate has no sweep's road in
it (`swept_trim::a_meeting_given_with_a_branch_left_out_is_completed_from_its_witness`): the bar
screwed a half turn, its candidates entered by hand, each end pose's face given its closed-form
meeting with a helicoid patch less the branch across the patch, completed to the helical integral
apart from the build. On the general road's own candidates
(`swept_general::a_meeting_left_out_of_the_trim_is_completed_from_its_witness`): the cube tumbling
5°, its fans' branches beside their rails taken out of what it is given, completed to the volume the
whole given builds. A witness nothing missing crosses is the refusal, naming what parts it where it
can: a pair met by search that missed the crossing, or another face of the same surface crossing the
walk — the surface passing back through itself, which no given meeting of two surfaces lays (the
tumbling cube at 75°, its edge 5's fan crossing itself, until the general road cut such a fan into
runs, below). A mixed piece is never kept whole or dropped whole.
Faces whose surfaces a `Given` names are always met, where their boxes do (`trim`'s pairing): the
given curves are the meeting laid with no search, and near balls alone left a thin stretch's face
and the plane it crosses unpaired.

**Trimming by certified membership, face by face (#880 E1, `swept::trim`; docs/swept-solid-design.md
§12).** A closed swept solid is built from candidates given to it — the tool's faces at the end
poses, the sheets its contacts and edges sweep — and the meetings known by construction, with no
blank: each candidate meshed and zoned by the sweep's depth (`member::Swept::beyond`: a ball about a
triangle proved interior where `m < −r` at its middle, exterior where `m > r`, `m` being
1-Lipschitz, else halved to the resolution and left near; `None` the cautious answer past `L h ≤
r/4` or 256 stretches); two faces of two candidates met only where near balls of each stand within
reach (`boolean::trimmed`, the arrangement's phases with `boolean::Meet` filtering its edge–face
crossings and face pairs; no cell built); each piece kept where membership a hair to either side of
it differs — the hair at least four times the membership mesh's sag — turned so its member side
lies behind it; the kept pieces assembled and healed as a selection's are; and the result checked
and judged by its definition (`closed::judge`) before it is returned. With no filter the
arrangement is unchanged. The bar of `square-bar-screwed-through-rod`, screwed its four turns — the
arrangement refusing it ("a cylinder's pieces do not close") — is trimmed from hand-entered
candidates (each end-pose face a sheet past its edges, `trim::planes_past`; each edge along the axis
and each long face's contact line its cylinder; each level edge its helicoid's patch,
`trim::screwed_segment`) to the helical integral worked apart from the build to 1e-9, its STEP file
reading back, and cut from the rod by the ordinary Boolean it is the bench's settled solid to 1e-10
(`swept_trim::the_bar_screwed_through_the_rod_is_trimmed_over_its_whole_roll`; two turns,
`the_bar_screwed_two_turns_is_trimmed_from_hand_entered_candidates`). `closed`'s own candidates
trimmed (`closed::trimmed`) are the arrangement's solid from fewer meetings
(`swept_trim::closeds_candidates_trimmed_are_their_arrangements_solid`). The ring pushed past its fold,
its inner contact entered in arcs cut at its cusps and each sheet's second meeting with each end
pose given (`t = −2q ± √(2(2R²|D_xy|² − |p|² − R² + r²))` along the slide), is trimmed as far as its
cusps and refused there, never built wrong (`swept_trim::a_ring_past_its_fold_is_trimmed_up_to_its_cusps`,
slow tier): the open box is #880 T1's fold cut.

**A face's contact under a constant twist, in pieces (#880 Q1b S1, `swept::contacts`;
docs/swept-solid-design.md §13).** Under a slide, a turn or a screw the points of a tool in contact are
fixed on it, and a whole tool's contact is a graph: `contacts::face_contact` reads a face's as the zero
set of `g = n · v` over its parameters, for any surface. It samples `g` on a grid over the face's
parameter box (`Located::parameter_box`, its loops' outline's, as many cells each way as the side's
length in space asks, `CELLS` at least) and closes each change of sign on a cell's edge by bisection.
It links the roots cell by cell, a cell crossed four times through its double point where `∇g = 0`
there and `g` is nought (the pieces meeting there end `Singular`), else by its saddle's own value.
It cuts each polyline where it leaves the face (`Located::holds_parameters`, the outline's winding),
each end placed on the boundary by bisection along the curve, and settles each point onto `g = 0` by
Newton across the curve. Points are added until every chord is within the tolerance of the curve,
and pieces ending at one point either side of a seam are joined, a loop closed. A face the motion keeps
in its own surface (`g ≡ 0` over it) is `Whole`. A ring's two roots meeting is no branch of the set,
only where the curve turns back along the ring: the leaning torus of `torus-screwed-through-rod` is in
contact along one closed curve through both its seams, where the constant-twist class's walk refuses
it ("a double point on a ring"). Held on that torus, the V wheel of `v-wheel-screwed-through-rod`, a
box leaning to a screw and a drum turned about its own axis: every point within `1e-9` of `g`'s
greatest of nought and in its face; no change of sign on a grid four times finer without a piece
within a cell; each ring's two closed-form roots (`Surface::ring_contact`) on a piece within the
tolerance; a plane's contact a line across it ending on its boundary; the drum's side `Whole`
(`swept_contacts`).

**A piece carried over the roll, and a sweep's candidates under a constant twist (#880 Q1b S3, S4).**
A piece is fitted as the cubic B-spline through points of the contact by chord length
(`contacts::piece_curve`), within a tenth of the tolerance of the contact itself: from seventeen of
the piece's points spread along it, its ends among them, every span whose middle stands farther off
`g = 0` than that given a point between its two, the parameters' midpoint settled (a seam crossed
unwrapped by the surface's period). Every node is a point of the curve, so the fit closes at a
cubic's rate; read along the piece's polyline by its own chord length, whose speed jumps at each
vertex, it closed at a line's, doubled its spans to four thousand, and every inverse on the sheet
read them all (750 µs an inverse on the leaning torus's sheets, 28 µs now). It is carried exactly (`contacts::carried`):
placed at the roll's start and carried to its end as one face of a blend between the curve and its
image (`Surface::Blend`, `Carry::Line` under a slide, `Carry::Screw` under a turn or a screw), its
rails the paths of its ends (`Curve::Iso`), a closed piece's one rail its seam; the carry is held to
the path's own pose at the roll's end. `contacts::candidates` gives each face's pieces carried and
the tool at the roll's two ends; a face read whole is refused by name (`Curved`) before any face is
read. **A sharp edge's fan is a contact of its own** (S2, `contacts::fan_runs`): a point of an edge
between two faces not smooth there bounds the sweep where its velocity is square to a normal of the
fan between the faces' — where the two faces' `n · v` differ in sign — so each stretch of the edge
where they do, its ends roots of one face's `n · v` (bracketed over 512 points of the edge, the face
whose `n · v` turns there the one solved), a closed edge's stretch through its seam one, is a piece:
its section the edge itself, fitted through points of it, carried as any other, crossing each of its
two faces at an end pose simply (order one in `pose_meeting`, where a face's own contact touches it,
order two). On the V wheel's rim and the drum's (`swept_contacts::a_sharp_edges_fan_is_in_contact_between_its_faces_contacts`,
S2's gate): every stretch ends where `n · v` of one face is nought within the tolerance, and on a
grid a hundred times finer the faces' `n · v` differ in sign inside every stretch and agree between.

Each piece is cut into stretches, each a candidate of its own (`contacts::stretches`): the trim keeps
or drops a candidate's piece by membership and meets a candidate only with others, so the leaning
torus's one closed contact — its outer branch bounding the sweep and its inner one buried — was read at
a buried point and dropped whole. A closed piece is cut in four at least, no stretch longer than a
quarter of the tool, each fitted through its ends exactly; neighbouring stretches meet smoothly along
the rail they share, which is given (P4, `boolean::Given`), not searched for. **Every two sheets are
given their whole meeting** (`shared_paths`): two sheets of one carry meet only along whole paths,
where their sections cross or touch in its invariants, so each pair is given those paths (each the
one sheet's `Iso` there, laid where both faces hold it), and a pair sharing none is given none and
not searched. Found from the invariant images' chords that cross or come within a thousandth of the
tool, closed on by damped Gauss–Newton, kept within the tolerance, and each valley of points whose
gap stays within it taken once, where the gap is least (golden section along one section of the
least over the other). At a double point of the contact — a saddle of `g`, the closed piece passing
it twice and each stretch turning a corner through it — the two stretches' sections touch in the
invariants, as every two curves through one point of the tool do (the invariants' differential has
the carry's velocity, which lies in the surface there, in its kernel), and their sheets touch along
that point's path: Newton alone, on that kinked double root, read five paths a hair apart, and
searched, the march along the one path crossed to the other where the sheets nearly touch between.
**A contact is cut at its double points** (`face_contact`): it is smooth but where two branches
cross, so a piece whose polyline turns by more than 30° there has passed one the grid did not see as
a saddle — the leaning torus's stands on the crossing of its two seams, a node of the grid, where its
one closed piece ran through it twice and turned a corner each time, and every sheet over such a
stretch is creased along that point's path. Cut there, the corner placed on the double point where
Newton finds `∇g = 0` within a cell, each end `Singular`, a closed piece opened at its first corner:
four stretches then end on the point's path. Every two of those part so slowly from it that the
sections' own fit can cross them again beside it (7e-3 along, on the torus, where laid as a path it
was a slit in the sheet), so a valley holding an exact root at an end of both sections is placed
there (`swept_contacts::every_two_sheets_of_one_carry_are_given_the_paths_they_share`). A rail is one point's
path and a sheet a union of paths, and two paths of one carry are one or apart, so a rail meets
another stretch's sheet nowhere or lies in it (`Blend::carried_gap` reads how far apart, in the
carry's invariants): each cut is moved, within a quarter of its stretch, to the point whose path
stands clearest of the piece's other points, by a thousandth of the tool where one does — a cut at an
even length on the torus stood 7e-6 from another stretch's paths, and its rail ran beside that sheet
its whole length, which no search settles. **Each sheet is given its whole meeting with the tool at
each end** (`pose_meeting`): with every face of the posed tool, the zero set of `F(u, v)`, the face's
signed distance at the sheet's point, read in the sheet's own parameters; and with the face its
section lies on, at the end its rim is carried to (`v = e`), where the sheet touches the face along
the rim and `F = (v − e)² h`, the zero set of `h` alone — what of a buried stretch passes back into
the tool a little way into the roll (on the torus, 9° on), reaching the rim where `h` changes sign
along it, and at a double point of the contact leaving the rim's corner. The rim itself is the
sheet's own edge, which the arrangement reads lying on the face, and is not given again (given, it
was laid a second time beside that edge, the two cut a hair apart). `h` is `F` less its value and
rate at the rim over `(v − e)²`, and within a hundred-thousandth of the roll of the rim its Taylor
series there, the one passing smoothly into the other, so the two readings are one function (one
jumping to the other made a sign change of its own along the rail, and at the kernel's tolerance
set a branch's points between two curves). **The rim's value and rate are read exactly** — `F` there,
and its gradient along the carry's own velocity (`Carry::rate`) — and the series' second and third
terms by differences of that rate: by five-point differences of `F` a thousandth of the roll apart,
on the leaning torus, whose carry moves a point a tenth of a millimetre in that step past a tube of
radius 2, the rate took the series' fifth-order truncation, 2.7e-5 for its own 1e-8, and the branch
beside the rim stood that much times `w` off the tool (4e-8, past the kernel's tolerance, refused
by the check). A branch is settled on `h` within a thousandth of the roll of the rim and on `F`
past four, and between on `F` less a share of the rim's value and rate that falls smoothly from all
of it to none, so its points lie on one curve passing from `h`'s zero set into `F`'s: settled on
the one or the other across a single edge, they stood between two curves a tenth of the kernel's
tolerance apart, and the fit chased them until four thousand nodes stood in a span. Each branch is marched over a grid of the
parameters, its crossings of the cells' edges exact, a branch's end beside a double point's corner
by no more than `h`'s rounding there puts it set on the corner, and fitted (`fitted`) from seventeen
of its points, a span split where its middle stands off the curve, measured across its chord from
the point it would be split at. Sought, a branch reaching the rim was marched to a stop a step
short of it, and at the double point laid beside the rim; given, the arrangement holds a given
meeting to the tolerance (`boolean`'s `triple_points`), not to the tolerance over the sine of its
surfaces' angle, a trace's, which at a double point, where every sheet and the pose are tangent,
grew until two given curves leaving the point a few degrees apart were cut a centimetre along
(`swept_contacts::each_sheet_is_given_its_meeting_with_the_tool_at_each_end`). The rim is given too:
the sheet's own edge and the rim given are one curve, merged where the meetings crossing them are
cut alike, and without it nothing lays the rim on the pose's face, which then stood whole.

**The trim of a screwed torus** (`swept_contacts::a_rod_grooved_by_a_leaning_torus_screwed_along_it_is_its_volume`,
S4's gate for `torus-screwed-through-rod`): the closed sweep trimmed from those candidates, judged,
and cut from the rod is the groove's volume computed apart from it (1460.6300031715, by the
helical heights the torus reaches at each radius) within ten tolerances, built to 1e-5 and to 1e-6.
Four things it took.
**A tool that is where each of its faces' surfaces has it inside is read exactly** (`member::Solid`,
`exactly`): its faces planes, cylinders, cones, balls or tori, the greatest of their signed
distances, each turned to read outward as its face does, is 1-Lipschitz, nought on the boundary and
negative inside, so membership has no sag; taken only where every point of the tool's mesh stands on
each surface's inner side (a concave tool is not that greatest: the solid a turned ring builds, read
so, stood 12 inside where it was out) and held to the tool's own point location at 512 points over
its box. A mesh's sag — a mesh's (a hundredth of the tool) set the hair a
piece's sides are read at, and two sheets touching along a path stand within a thousandth of each
other a centimetre off it, which no side read at that hair tells apart. **The hair is the
tolerance's**, a few of them (`Trim::hair`): above the sections' fit, below the thinnest piece
beside a touch; the tool's 1e-4 read those wrong. **A piece proved inside or outside the sweep at a
point of it bounds nothing** (`trim`'s `side`, `Swept::beyond` at `ON_CURVE` tolerances), read
before its sides. **A fit merges nodes within the tolerance** (`fitted`): a cubic through two
nodes a hair apart swung 160 off its branch between them, unread by a span too short to check,
and the flux and the mesh of the face it bounded read it so; and a branch two of whose roots
stood at one place was refused whole. Each piece of contact is cut at every double point it
passes (S1's gate: a piece ends on the face's boundary or where `∇g = 0`), where `|∇g|` falls to
a least and Newton holds one, as well as where it turns sharply. **At a finer tolerance**: a
branch of a sheet's meeting with a pose is settled on `h` and then onto `F` itself off the rim, a
move of a few tolerances (`h`'s zero set is where `F` is the rim's own value and rate carried out,
the section's fit, and a branch settled on it alone stood 2.5e-6 off the tool at 1e-6; Newton on
`F` alone wandered near a fold, where both its factors are small). The arrangement holds a given
meeting to the tolerance throughout (`brep.md`): where an edge meets a face whose surface a face it
bounds is given a meeting with and the search is refused, it is read off the given curves; a
touch run reaching an edge's own end is that end; two given meetings' crossing is kept where their
curves cross; and no vertex is taken onto a given meeting's curve from farther than `ON_CURVE`
tolerances. At a double point's corner, where `h` is nought but for its rounding, its zero line
clips the corner and leaves a root of rounding on each of the corner's two sides besides the
branch's own, three in one cell, which no pairing joins: the corner is the root those two stand
for, joined to the third (`pose_meeting`).

The build's tolerance must be one a fit can hold: a contact fitted within a tenth of
`geom::tolerance` (5e-10 of the size) takes thousands of spans, and its rim then stands off its face
past the tolerance (3.7e-8 against 2.45e-8 on the leaning torus), so the edge reads as crossing it.
**The front door** (`closed::closed`, `cut_from`): under a constant twist, what the closed class
refuses as `Curved` goes the contact's road (`closed::contacts_road`: the candidates, trimmed at a
hair of four tolerances or a millionth of the tool, judged, cut from the blank), and where that road
refuses too the closed class's refusal stands. S4's three cases are built
through it at 1e-5, each written as a STEP file that reads back as the solid
(`swept_contacts::…_through_the_front_door`): the torus screwed through the rod, to 1460.6300031715;
the V wheel, its rim's fan a sheet, to 1554.3809694 (the rod cut where the helical invariants
`(ρ, z − c φ)` lie in the wheel's shadow, the radii by Gauss–Legendre, moving 1e-5 a doubling at
forty points; built, 1554.3809657); and the drum leaning 25° to a screw through the ring sector,
both rims' fans sheets, to 50.2500186377 (the strip of helical heights its shadow covers at each
radius, integrated by Gauss–Legendre between the strip's kinks — where the shadow splits in two and
where its ends cross the window's — over sixteen pieces of each panel, eight giving 2e-10 less;
built, 50.2500191). At the
kernel's own tolerance (`geom::tolerance`, a billionth of the size, what the bench builds them at)
the three are built alike, to their volumes and their STEP files, in about a minute
(`swept_contacts::the_three_cases_are_cut_at_the_kernels_own_tolerance`, slow tier): the torus to
1460.6300031967, 2.5e-8 off. The sharp-rimmed round slid at a lean, #810's negative control,
builds through its fan to the meridian's own reading within 1e-5 of the volume
(`swept_slid::a_round_meeting_a_sharp_rim_at_a_lean_builds_through_its_fan`), its admission
(`closed::reads`) still refusing it as `Curved` at the rim.

**Poles: a turn or a screw past the axis, and rings in contact all round (#880 Q1b S5).** Under a
turn every point of a section runs round a circle about the axis, so **each sheet lies on the
surface of revolution of its section's meridian image** `(ρ, h)` (`contacts::meridian_surface`): a
plane, cylinder or cone where the image is a line within the tolerance, a ball or torus where it is a
circle (fitted by quarters where its ends meet), else a `Surface::Revolution` of the fitted
meridian, one surface reused for coincident images (a piece's mirror halves); the sheet's rails are
circles about the axis, its rims' parameter curves interpolated by angle about the axis (a point near
the axis takes its neighbour's angle, a torus's `v` its place on its own meridian, the end rim read
ahead by the turn so a turn past a whole one stands where the walk left it) and refined until they
stand within a tenth of the tolerance of the rim. Two sheets about one axis meet in circles, which the
arrangement reads in closed form: the turn's givens are the sheets' closed-form meetings less the
circles both sections end on (shared rails). **A section is cut where its meridian image folds back
and where it crosses the axis** (`folds`, `folded`): each side its own sheet, the axis point settled
onto the surface along the axis and the end `Singular`; an end on the axis does not move, its rail a
degenerate edge, and a pose meeting's branch from one pole to another meets nothing. **A part is
buried where its path dips into the tool on both sides** (`buried`): read at seven inner points to
second order — the face's normal curvature along the motion against the motion's own — where the
greatest of the faces' signed distances at the point moved a step either way falls below a
thousandth of the tool times the step's square; a point that does not move abstains. A ball is
redrawn with its poles off its contact (its contact the great circle through the axis). **A contact
along a face's own edge is the edge's** — read with the edge's fan, `fan_runs` taking a run where one
face's `n · v` is nought along at least three samples — **and one along its seam the face's own**
(the cone spun about a line through it, in contact along its seam's generator). **A piece reaching a
pole of the face's parameters runs on to it** (`face_contact`): a cone's apex, where no node of the
grid reads `g`, a branch ended a cell short; pieces meeting at the pole are not joined as across a
seam. The rod spun about a line through it (41.685706292608, the slices' areas closed in `ρ` and
Gauss–Legendre in height), the ball swung across its own centre line (59.333194348871), the coin
turned about a line through its rim (46.547001074918), the pin swung through a ring sector
(`80π/3 − (π/6)(57 + 1/6)`, closed) and the cone spun about a line through it (288.49505, the arcs
of each circle the cone holds swept on by the turn, by adaptive Gauss–Kronrod, known to 1e-5) are
each held to a volume made apart from the build and to a STEP file at 1e-5, and at the kernel's tolerance in the
slow tier (`swept_contacts::the_turned_cases_are_cut_at_the_kernels_own_tolerance`). Beside the
circle where the cone's two branches of contact cross, its sheet is covered by both end poses less
deeply than membership's sag: **a trimmed piece inside the tool at either end of the roll is inside
the sweep**, read exactly (`Located::solid_place`) before its sides.

**Screwed through whole turns** (`drum-end-screwed-through-rod`, `cone-point-screwed-through-rod`,
four turns rising 6, `swept_contacts::a_drum_end_and_a_cone_point_screwed_through_a_rod_are_cut_through_the_front_door`,
slow tier, each within its boundary's area times the kernel's tolerance of the volume made apart: the
rod less `20 ∫ ρ W(ρ) dρ`, `W` the width of the tool's section at radius `ρ` across the screw, the
roll passing the whole section at every height of the rod — 1339.548902340676 and 1513.245603582067).
**A sheet's meeting with a posed face is read only where the face is** (`pose_meeting`, its box grown
by two cells) **and not along its rim lifted whole turns on**, where the rim lies on the face's surface
again (a disc whose plane holds the screw's axis, touched by its rim's fan each turn): a touch, `F` a
square there, crossing nothing, whose rounding the grid read as crumbs of branches. Two fans of one
rim share the path of the rim's points a tenth of a turn apart and cross along it at under a degree,
so **an edge's crossing with a surface whose meeting with a face the edge bounds is given is where the
edge crosses the given curves** (`boolean`): the search's root, that tolerance over the sine off the
path, is moved onto the given crossing within a thousand tolerances of it, and a flat root lying on a
given curve is not moved onto another cut. Where membership a hair off **a piece of the tool's own
face at an end of the roll** cannot settle near that end (the start cone along its base rim, a band
narrower than the hair), behind it is the tool and ahead of it is swept where the face leads the
motion there (`trim`). A cycle of a face's pieces no outer cycle holds and smaller than a millionth
of the face's parameters — three faces of other operands all but tangent at its corner — bounds
nothing and is left out (`boolean`'s `pieces_of_face`). **A branch of a sheet's meeting with a posed
face the section does not lie on is clipped to that face** (`pose_meeting`, #880 T1e): where the
branch leaves the face (`query::Located::face_place` read along it, the crossing bisected) it ends,
and an end within the root of the tolerance of a corner of the sheet is put on the corner. Read
whole, a branch ran on over the face's surface past its rim, where nothing of the tool stands. With these the contact's road also builds
the screwed bull-nose (`swept_patches`) to the volume worked apart.

**Free-form faces (#880 Q1b S6).** A face's contact is read numerically on any surface, so S6 is
what the faces' shapes leave to the road's later steps. **A piece ending on the face's boundary ends
at `g`'s root along that edge** (`face_contact`): placed by halving across the boundary in the face's
own parameters, the coil spring's tube's contact and its ball cap's met the rim between them 4.3e-5
apart, and the two sheets' rails along that path did not close; read along the edge with the face's
normal, the two roots are one where the faces are tangent, each its own across a crease. **A face closed
by its own seam is unwrapped across it** (`piece_curve`, given the tool's `Located`): a parameter the
surface gives no period, whose box's two sides are one curve in space — an extrusion of a closed
spline — is read as one a period the box's width, so a piece joined across that seam is fitted
through points beside one another (fitted across the bar, the spline bar's inner contact took a cubic
61 long for its 17). **Membership's mesh is held to the sag measured over every triangle** (`member::Solid::new`): the
mesher reads a triangle's sag at its middle and its inner sides' alone, and on the drafted slot a point
of the tool's own cone stood 1.1e-3 off a mesh measured 9.5e-4 — past `Swept::beyond`'s margin, so a
piece of the end pose's face it bounds was proved outside and dropped; each triangle's sides' and
middle's distance from its face is measured too, the greater taken. The spheroid wheel screwed through a rod
(1559.464104558205: unrolled, the rod less `20 ∫ ρ W(ρ) dρ`, `W` the section's width across the
screw), the drafted slot screwed through it (1401.313962654651, the same way, its half height drafted)
and the coil spring slid aslant through a plate (3541.9379700: the plate less its thickness times the
spring's shadow along the slide, by scanlines). **A face's contact is cut where its burial changes**
(`cut_at_burial`, under a screw or a slide): where the path of its points passes from dipping inside
the tool to rising off it — read at 64 points along the piece and bisected to a point of it — the
sheet passes from sweeping only the inside to bounding, and its meeting with the posed tool leaves its
rim there; cut there as a corner (`Singular`), the meeting's branch ends on the corner. Cut at a length
instead, the spline bar's far contact left a buried stretch beside a bounding one and a sliver where
they and the pose met that no membership read settled. Two stretches with a buried one left out
between them are not neighbours and are given no rail: given the one's end as both's, the leaning
torus's sheets were given a path 0.48 off one of them. A closed spline's nearest point found at its
seam is refined from the seam's other end too and the nearer kept (`nurbs::BSpline`): held at the
seam, an extrusion's point beside it read a foot 0.07 off, and a spurious burial. **A meeting's end at a
vertex the face's boundary holds takes that node's parameters** (`boolean`'s `FaceCut::internal`),
where both are that point: at a sheet's corner where its two rates nearly agree the walked inverse
stood 0.017 along the rim from the corner's own, and the meeting hung there, parting nothing. The
spline bar screwed through the rod (1335.319593047173, unrolled, the most of `acos(x/ρ) − z/a` over
the section's lower left boundary) joins the other three, each cut at the kernel's tolerance within
its boundary's area times it (`swept_contacts::s6s_free_form_faces_are_cut_through_the_front_door`,
slow tier, S6's gate): all of Q1b's fourteen cases build (docs/swept-solid-design.md §13).

**A drawing reaches the contact's road through the closed class's admission** (`swept::dispatch`,
#880 Q1b): under a slide, a turn or a screw a tool the class reads no candidate for
(`closed::reads` refusing it `Curved`), or whose sheet folds (`Folded`), is admitted where the
class asked first refused it by a row the road answers (`road_answers`: the tool unread, T1 and T2;
a contact no regular curve, S1; an end of the roll in the blank, E1; a fold, E3 and S3, since #880
T1c), and its build takes the road. A stock reached twice on a ring (S2) and a sheet's crossing (E4,
S4) keep their first refusal. **Folds in drawings** (#880 T1c): `folded_ring.sv`, `leaning_ring.sv`'s
ring leaning past its fold, refused at E1 by the walk and read `Folded` by the class, is admitted
past both and built by the road to brep-bench's `ring-pushed-in-past-its-fold` within 1e-9
(`solid_sweeps_examples::the_ring_pushed_past_its_fold_bores_round_its_cusped_core`); `tight_ball.sv`,
brep-bench's ball round a tight helix inside a block, which the class takes as a ball under a screw
and whose build sends the fold to the road, to its ball within 1e-8
(`solid_sweeps_examples::the_ball_screwed_round_a_tight_helix_hollows_the_block`). Under a turn a
sheet lies on its meridian's surface of revolution, so a fold's two leaves lie on one surface and
overlap, and the trim keeps them once: `tangent_rod.sv`, a rod lying level along the way it turns,
the closed class reading its side curved past its candidates at the fold, is built by the road to
its arcs' closed form within 1e-6
(`solid_sweeps_examples::the_rod_turned_along_itself_hollows_the_block_past_its_fold`). **Two
sections whose images about a turn's axis are one curve sweep one surface** (`contacts::meridian_surface`):
the rod's two end rims, a tool's mirror pieces; the second is matched to the first's meridian
within twice the tolerance, measured on the fitted meridian itself (its polyline finds the stretch,
a golden section settles it), so their coincident pieces are one surface to `ssi::same` and the
trim keeps the lower operand's. The twist drill's
flute wheel too steep for its helix is admitted past S3 for the road
(`twist_drill::the_constant_twist_rows_refuse_by_name`) and built by it
(`twist_drill::the_steep_wheel_cuts_its_folded_flute_by_the_contacts_road`, slow tier: per
millimetre its 3 mm and 6 mm drills within 1e-6 of each other, the field's section area within
2e-3, each STL agreeing with the field). **A body mixing a closed sweep
with sheets** cuts the sheets' sweeps first (`brep::sweep::mixed_cut`), their cells judged by the
body's field less the closed sweeps (`MaterialField::read_less`, `sector::left_out`): with them, a
cell of the body clearance's the flute would split read both material and removed. (Cutting the
closed sweeps first instead loses them: the sheets' road rebuilds its blank from the recipe.)
**Where the whole sweep's trim does not close, the road trims it within the blank's reach**
(`closed::contacts_road`, `trim::Trim::reach`): cut from a blank, the sweep is wanted only inside a
box holding the blank, grown by 0.0937 of its diagonal so its planes stand clear of the blank's;
the candidates' pieces outside it are dropped, they are paired only near it, and the box's own
faces are kept where the sweep holds them. The judge reads membership inside the box, and the cut
is the same. The domed pin's whole trim does not close at its pole; within the reach it does
(`rings_all_round::a_turn_about_an_axis_the_tools_meets_builds_within_the_blanks_reach`, to its
closed form within 1e-9). The whole sweep is asked first: a torus screwed through a rod, its
stretches' rails crossing the box, is split there unlike its neighbours. The narrow wheel stays
S2's. **A tool of revolution has its seams turned off its contact's folds** (`contacts::seated`,
#880 T1e): turned about its own axis, the same solid, every face's seam turned with it. Where its
contact folds (its image in the motion's invariants turning back) and a seam stands within a tenth
of a radian of a fold, the tool is turned to put the seams in the middle of the widest gap between
the folds' angles about the axis. A seam beside a fold put an edge of the tool's own through the
cusp's meetings on the tool posed at an end of the roll, every corner of the pieces there a
tangency: the twist drill's steep flute wheel, drawn in the plane where it grinds, its seams 0.056
rad from its fold, which the whole sweep's trim did not close. Seated, the flute's whole trim closes
cut from the stock (`twist_drill::the_steep_wheel_cuts_its_folded_flute_by_the_contacts_road`) and
alone, with no blank, as drawn and turned a quarter turn first within its area times the tolerance
of each other (`twist_drill::the_steep_wheels_flute_builds_alone`, 8e-6 of 0.028). The leaning rings'
seams stand a third of a radian off their folds and are left: turned, they were refused.
The admission keeps that first refusal (`dispatch::Closed::first`, `closed_class::Closed::first`),
and where the road refuses at the build the drawing is refused by it, not by the closed class's
`Curved` (the domed pin's pole, S1, refused so until the road trimmed within the blank's reach,
#880 T1c). The series' three drawings of the road, one a motion (`arched_bar.sv`, `coin.sv`,
`drum_end.sv`), are held to volumes made apart from the build (`solid_sweeps_examples`), and four
negative controls the walk refused by S1 or E1 are built and held to their definitions by a Monte
Carlo (`closed_class::kept_by_its_definition`). **A fold is read from steps past the tolerance**
(`contacts::folds`): a step of a section's meridian image within the tolerance its points hold says
no way, and where a section runs in square to the turn's velocity at its end its image barely moved
(1e-10 a step) and its points' own error turned it back and forth, the coin drawn on its rim cut
twice a hair from its end and its plane's pieces left open.

**A sheet that folds goes the contact's road too** (`closed::closed`, #880 T1): under a constant
twist the closed class's `Folded` — a ring slid leaning past `tan² β = (R − r)/r`, its inner contour
cusping seen down the push; a ball screwed round a helix tighter than its radius, its pipe folding —
is asked of the road as `Curved` is, and the trim keeps of each folded sheet what bounds the sweep.
brep-bench's `ring-pushed-in-past-its-fold` and `ball-screwed-round-a-tight-helix` build through
the front door to volumes made apart from the build — the ring line by line down the push, each
line's intervals a quartic's roots with the outline's crossings as breakpoints
(`swept_slid::torus_pushed_by_lines`), the ball circle by circle about the axis, each covered
`min(L, 2π)` (`swept_loci::swept_by_circles`, any ball, its pipe folding or not) — within ten
tolerances at 1e-5 and with STEP files that read back
(`swept_contacts::a_sheet_that_folds_is_cut_through_the_front_door`), and #805's two negative
controls, the ring slid at 60° and the ball screwed tighter than its radius, build within 1e-6
(`swept_slid::a_ring_slid_past_its_fold_is_swept_by_the_contacts_road`,
`swept_loci::a_ball_screwed_tighter_than_its_radius_is_swept_by_the_contacts_road`). The ring
stands 5e-10 off at the kernel's tolerance as at 1e-5: its reference reads each line's length
across `y` adaptively, to its share of the tolerance, where a fixed rule of 128 points a piece read
it 4.6e-7 of itself over and moved by 4e-7 when doubled — near the cusps a line's length runs
nearly square-root-like inside a piece. The balls' reference cuts each of its integrals where the
integrand's make-up changes — over `z` where the rolls reach a circle's height, the end poses stop
clamping them and whole circles start and stop; over `ρ` where a stretch of circles covered all
round is born or dies — where uncut it read the bench's ball 3.5e-6 low; and its halving is taken
only past three halvings (`swept_slid::kronrod`), the two rules having agreed on a piece both
misread by 1.65e-4. Read so, every folded case the road builds stands within its tolerance: at the
kernel's, the bench's ring 5e-10 off and its ball 4e-10, the ball screwed tighter than its radius
2.4e-10, its faces across the pipe's fold read by columns (`props`)
(`swept_contacts::the_fold_cases_hold_at_the_kernels_tolerance`,
`swept_loci::the_tight_ball_holds_at_the_kernels_tolerance`), and the ring slid at 60° within 1e-9
(`swept_slid::a_ring_slid_past_its_fold_is_swept_by_the_contacts_road`). **A stretch is cut
clear of the folds** (`contacts::stretches`, #880 T1b): not within half its reach of where its
image in the motion's invariants turns back, or barely moves against the piece — where the contact
runs within a tenth of a radian of the flow. Drawn to the clearest point, a cut found the cusp,
the swallowtail's tip, and the stretch ended on the fold's path, where the sheet's two rates run
together: its meeting with an end pose was traced near the cusp on either leaf, and the pieces did
not close (the ring slid at 60°, refused finer than 1e-6; pushed in at 60°, at every tolerance).
**At a node where a face's rates nearly run together** (`geom::NEAR_FOLD`) — a fold's path
through the middle of a stretch — a half's way is first read a fiftieth along it, not a
ten-thousandth (`boolean`'s `leave` and `arrive_back`; ways tied there are read again at one length):
there a point beside the node stands in the parameters at the square root of its distance in
space, and a curve leaving the cusp at an end pose read 0.26 radians for −2.3, its pieces open (the
ring at 65°, its cut held clear, at 1e-5). **A swallowtail is cut at its double point**
(`contacts::double_points`, `contacts::stretches`, #880 T1e): where a piece's image turns back
twice, its outer leaves cross over one path, and a stretch holding both folds held both leaves of
that crossing — one sheet crossing itself, which the trim, meeting each candidate only with others,
never parts. Each double point is found exactly: seeded where two chords of the image cross with a
fold between, and settled by Newton's method on its four equations (two points of the face on the
contact, their images one) to the rounding, its two points apart; round a closed piece either way,
so a double point whose two points straddle where the piece is read from is found too. Under a screw
the image's height is read unwrapped along the piece, so only crossings within one turn are double
points: a sheet carried past a turn meets its next turn as any two sheets meet (the ball screwed round
a helix tighter than itself, whose image crosses itself at nine places modulo the rise). Two double
points whose spans nest or interleave are not cut (the ring at 70°, half a degree before its two
double points meet and vanish). The piece is cut at both of its
points, so its outer leaves are stretches of their own meeting along the path, and the tail between
(both folds and the middle leaf) is a stretch the trim reads buried. No cut is drawn onto a fold.
**Two splines of one meeting are crossed by their enclosures** (`boolean::crossings_among`,
`query::enclosed_crossings`): where `crossings_in` has no closed form for two curves given or found
for one pair of surfaces, their crossings are sought by their stretches' enclosures, each held back
a hundredth of itself from an end lying on the other (which `ends_on_one_another` lays, and where the
two may leave one another tangent); a meeting `enclosed_crossings` settles outside the stretches
asked about is not theirs, however shallow; and a crossing is kept only where it parts the two, each
standing more than eight tolerances off the other a thousand tolerances along it either way — two
fits of one touch wander about one another within the tolerance where they run together (a turned
coin's sheets on the torus its rim turns into, at the kernel's tolerance, read several crossings
there, `swept_contacts::the_turned_cases_are_cut_at_the_kernels_own_tolerance`). On the tool posed at the roll's start the rim of one
leaf crosses the other's branch over the double point, which no vertex parted, and the face's
pieces read a hole in no piece. The ring pushed in at 56°, its swallowtail 0.02 across, builds cut
and alone within ten tolerances (`swept_contacts::a_narrow_swallowtail_is_cut_at_its_double_point`,
1.5e-6 and 1.2e-6 as built), as do 58° and 60°
(`swept_contacts::swallowtails_cut_at_their_double_points_hold_past_the_fold`; 60° stood 1.6e-4 and
4.7e-4 off before). **A crossing of a surface read by inversion is placed on it**
(`boolean::on_surface`): where an edge crosses a swept sheet, a fitted net or a family's wall at a
shallow angle, the reading's root stands off the surface by what that angle makes of its own
tolerance, and is settled onto the crossing by Newton's method on `c(t) − S(u, v)` from the root and
its foot, taken where it settles to a thousandth of the tolerance within ten thousand of the root —
at 80° the torus's seam circle crossed a slid contact's sheet with its root 4e-5 off it, read
outside the sheet's face and dropped, and the branch the sheet meets the torus in crossed the seam
with no vertex: a hole in no piece. Every whole lean from 55° to 85° builds cut and alone within ten
tolerances of its volume (`swallowtails_cut_at_their_double_points_hold_past_the_fold` holds 58°,
60°, 70° and 80°); at exactly 57° the line integral reads 4.7e-4 low, the build smooth through it to
4e-8 at 1e-6 and the reference not, its second differences jumping there. **Not yet:** a swallowtail
being born, within a third of a degree of the fold (the ring at 54.75°, 54.8° and 54.9°, its tail
1e-7 to 5e-4 across), refused: its double point is found and cut, but its leaves cross there at a
vanishing angle, the two rails of the double point's path overlap most of their length, and pieces
of a thousandth of the ring stand beside them. (brep-bench's cadrs builds the ball to 282.091, 11% over the sweep's 253.4494.)

**A point's side off the nearest feature's pseudo-normal** (`member::Solid::signed_near`, #880
Q1b): a point whose nearest point of the tool's mesh lies on a vertex or an edge is read by that
feature's pseudo-normal — a vertex's, its triangles' unit normals weighted by their angles there; an
edge's, its two triangles' summed (Bærentzen & Aanæs 2005) — exact for a closed mesh turned
consistently outward, where it fell to the generalised winding number over every triangle. A vertex
with an edge not bounding exactly two triangles, and such an edge, has none, and is read by the
winding. Zoning the leaning torus's sheet, where most of what is read is far from any pose of the
tool, fell from over twenty-five minutes to thirty-six seconds. Held at four thousand points about a
torus, a leaning V wheel and a box, every one farther than the mesh's sag from it, to the winding's
side, on meshes 99% of whose vertices have a pseudo-normal
(`swept_member::a_points_side_off_its_nearest_feature_is_the_windings`).

**A tool leaning to a screw's axis is built (#777 M5).** Each leaning edge, and each leaning
face's contact line, screws into a patch of a ruled helicoid over the segment and the roll a
twentieth past each (`closed::Cutter::Patch`), its rim standing inside a cell; two sheets of one
screw meet in exact helices (`ssi::screwed_lines_meeting`: the line's points the screw carries onto
the other sheet, isolated by the Krawczyk operator, #814), never traced ones, whose ends leaning a few degrees off
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

**A tool of revolution's path of pieces cut from a blank is cut piece by piece**
(`swept::pieces`): the tool read as a convex solid of revolution (`patches::Revolved`, its cone's
apex a point of its meridian), each piece reaching the blank's box swept alone — slid along its
axis, its meridian drawn out where it reaches farthest and turned once; slid square to it, its half
behind, its axial section carried and its half ahead, sewn; turned about a line along its axis that
misses it, the section turned about that line between its halves (a whole turn the ring alone); a
ball-end mill at a slant as a run (below); any other piece by the closed class alone —
and cut in the path's order, then judged. **Its junctions are laid by construction where they are
smooth** (#931): the next move's sides touch what the last left round the section they share, a
touch the Boolean reads within the tolerance of three surfaces at once. So moves square to the axis,
each starting where the last ends with the axis moving on the same way, are **one chain**, their
sides sewn along their shared sections between the tool's half behind at the start and its half
ahead at the end — a chain closing on itself its sides alone, a ring; a slide along the axis into a
chain's start (a plunge) or out of its end (a lift), or into or out of a whole turn, is the column
its flat top sweeps alone (`lift`), the chain or the ring holding the tool there, its start then
taken with its half behind; and slides along one axis line one after another (a drill's pecks) are
one slide over every place they stand; and a ball-end mill's slides at a slant, one after another
in one plane holding its axis, are a run, its corners laid by construction. Elsewhere each piece after the first is shut where it starts
by its section, the piece before holding the tool there; and a chain ending where a sweep cut before
it holds the whole tool (a closed path come back to where it began) is shut there by its section
too. A section's arcs are drawn with their circle's seam opposite the arc's middle, so no
cylinder or torus a section's arc is carried or turned into has its seam along an edge; and
`build::prism` reads an arc facing against the profile's normal as the same arc facing it
(`Seg::facing`) — it wrote a mirrored arc's cylinder's parameters along the normal, and a
bull-nose's corner mirrored across the axis stood 80 off an edge it bounds. A ball's quarter in a half (its meridian's
arc from the axis to the equator) is the ball's part on that side of the section and of the equator,
its poles off both planes (`raster::lune`): turned about the axis, its pole stands on the section,
and at a corner the next move's side touches the half along a great circle through that pole — a
meeting through a pole, which the Booleans do not split ("a sphere has a hole in no piece"). A piece
the road cannot sweep or cut leaves
the whole path to the arrangement. Held to a flat end mill plunged, run round a square and lifted,
`2 (76 + π)` to 1e-9 (`swept_closed::a_flat_end_mill_run_round_a_square_takes_its_band`), a
bull-nose run round a triangle and back to its band at every height
(`swept_closed::a_bull_nose_mill_run_round_a_triangle_and_back_takes_its_band`), a ball-end mill
round a slot narrower than itself and back to the slot and its band
(`swept_closed::a_ball_end_mill_run_round_a_slot_narrower_than_itself_takes_the_slot_and_its_band`), a
bull-nose run along two smooth arcs to Pappus's volume
(`swept_closed::a_bull_nose_mill_run_along_two_smooth_arcs_takes_its_section_turned`), and a drill's
pecks to one hole (`swept_closed::a_drill_pecking_a_hole_takes_one_hole`); brep-bench's
`cnc/flat-end-mill-profile`, `cnc/bull-nose-profile` and `cnc/pocket-with-an-island` to their closed
forms within 2e-15, each in about two seconds. **A run of slides square to the axis turning at
sharp corners is one sweep too where it is laid** (`mitred`): each slide's two halves carried along
it, each cut square at the section where it starts and ends — or, on the side a corner turns
toward, slanted at the plane through the axis bisecting the corner, where the halves before and
after meet in one curve (a line on a plane, an ellipse on a cylinder) — and at each corner the
tool's wedge on the side it turns from, turned through the corner about the axis. A slide goes on
with the run before it only while each slide is longer than the mitres either side take back on
the side each turns toward (the tool's reach times the tangent of half the turn, past a thousandth
of the reach spare), and the run turns through a right angle at most: it then runs on along one
way, and no two of its slides but those that join come within reach of one another (a polygon's
inward offset whose every side keeps a length is simple). A curve a CAM program writes as a fine
polyline is a sweep a quarter turn, not one a side (a 36-gon's 27 sides: five sweeps; 135 sides of
a 180-gon in 1.4 s, not 15,
`swept_closed::a_bull_nose_mill_run_along_a_polygons_sides_takes_its_band_with_its_corners_mitred`).
Still met in the Boolean: a corner no run takes (a zigzag's turn back, its stepover shorter than
the mitres), or between two runs at a slant (a raster's passes are off the blank between) — whose
sweeps touch round the section at the corner, read there within the tolerance; a slide turning
back along the last within a tenth of a degree is refused there. Refused: a turn about a line square to the axis (a five-axis tilt:
the top rim's sphere, which no section carries; the closed class refuses it as `Curved`).

**A ball-end mill's run at a slant** (`swept::raster`, the pieces' road's sweep of slides at a slant
one after another; brep-bench's `cnc/ball-end-surface-raster`, 944 moves whose one arrangement ran
past its membership budget after nine minutes). The mill is read off its
B-rep (`raster::Mill::read`: a half ball, a cylinder of its radius about the axis through its
centre, a flat top square to it, and the volume theirs to 1e-9). Its moves fall into runs, each a
polyline of the ball's centre in one plane holding the axis, every step forward along one way across
it (a step along the axis a run alone). A run's sweep is built from its surfaces: under the centre's
path each step's half cylinder, at a corner turning up (a valley) the ball's lune between the two
steps' great circles, at one turning down (a hill) the two half cylinders' meeting in the plane
bisecting the turn — two cylinders of one radius whose axes meet are each other's mirror there, so
each cuts the same ellipse — a corner turning by less than a tolerance's lune (`r δ²/8` under the
tolerance) a hill's, and steps along one line one step; at its ends the ball's quarters; above it the
two walls the shank's sides sweep, the shank's halves at its ends and the stadium its section sweeps
in the plane the sweep is cut off at, a hundredth of the blank's size beyond the blank and every
ball centre of a run reaching it along the axis — which takes nothing from the blank, every pose's
top standing past it (a run ends where it would turn at a hill so sharply that its half cylinders'
ends would cross, `raster::laid`; a pose whose top does not stand past it leaves the path to the
arrangement). So a corner is laid by
construction, never found by a Boolean where two moves' balls coincide and the next step's cylinder
touches the ball along a great circle (which ordered the faces round that edge wrongly and joined two
cells). A sphere's poles stand off every plane a run's piece is cut at. The runs are cut with the path's other pieces, in its
order. Held to the prism a run along a line takes, `2000 − 20 (2π + 8)`
to 1e-9 (`swept_closed::a_ball_end_mill_run_along_a_line_takes_its_sections_prism`), and to what the
lower envelope of a run over two valleys and a hill leaves, integrated slice by slice apart from the
build, to 1e-6
(`swept_closed::a_ball_end_mill_run_over_valleys_and_a_hill_takes_what_its_lower_envelope_leaves`).
The raster's 41 passes cut in about 7 s, judged, and agree with cadrs's solid to 1e-10. Membership
along a path of pieces reads only the pieces near the point: a slide's by the tool's box carried to its
two ends (`member::Swept`), so the judge reads a raster's 512 points against a few moves each, not
all 944.
A curved tool past these (`Unbuilt::Curved`: under a screw a round face about an axis leaning to it, a torus, or a sharp
circle leaning to it (#780 L2); under a turn a ball across its axis, a torus leaning to it or a free-form face; under a slide a
torus's face leaning to it that is no whole ring, or a free-form face; an edge no line or circle) are refused by
what they are, read before any build
(`closed::reads`); under a turn a cylinder or a cone about a leaning axis whose contact is no curve
over the face, or whose meridian folds back along the axis, at the build's first step. A sharp
circle through the axis, or whose normal lies in the plane through the axis and its centre (a pin
along a radius), turns into no ring (`Rim::simple`, #780 L2): refused as it is read (`Curved`), so
the sweep is admitted to the closed class only for the contact's road, where the class asked first
refused it by a row the road answers, and where the road refuses it too that first refusal stands
(#880 Q1b; the domed pin, refused so until #880 T1c, is built within the blank's reach,
`rings_all_round::a_turn_about_an_axis_the_tools_meets_builds_within_the_blanks_reach`).

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

**A whole turn of any tool is built by its shadow (#880 Q1, `brep::swept::shadow`).** Over a whole
turn the sweep holds each circle about the axis whole or not at all, so it is the solid of
revolution of the tool's shadow in a half-plane through the axis. `swept::closed::closed` asks this
road first whenever the roll covers a whole turn (no blank: #880 P1), and its arrangement only where
it is refused. The candidates are exact curves of the half-plane — each face's contact under the turn
(a plane's line, a hyperbola's arc; a cylinder's or a cone's along, beside or leaning to the axis,
`turned::Contact`; a ball's great circle through the axis, mirrored where it reaches across), a face
kept in itself its own meridian, each sharp edge's image (a line's hyperbola, a circle's
`turned::Rim`, a level curve's stretch of height), and the axis — each face's contact read over the
face alone, its ends held onto the edges it crosses by Gauss–Newton. They are split where they cross
and where one's end stands on another (a contact meets the edge it ends on tangentially there, given
by construction), each piece kept where membership a hair to each side differs (the hair under a
third of the way to the nearest other candidate; the tool meshed finer where the sag leaves a reading
unsettled), the kept pieces linked with the shadow on their left, each region revolved with those
curves as its meridians, and the solid judged by membership before it is returned (`closed::judge`).
Refused by what it is (`Unbuilt::Curved`) where a face is no plane, cylinder, cone, ball or torus
about the axis, an edge no line, circle or level curve, or a leaning round face's contact no curve
over it, or judged wrong or past its budget, which are the build's refusals. A convex polyhedral
tool keeps its closed form, asked first (its slices' radial spans, no mesh, about fifty times
faster). Every other whole turn the arrangement built builds so, to the same gates (the drum, the
frustum, the rounded key and the ball on its stem to 1e-9 or finer), and brep-bench's `swept/v-wheel-turned-once-round-rod`, which the arrangement
refused (a turned curve meeting a revolution "was not proved"), builds in a tenth of a second: gate
`swept_shadow::a_leaning_v_wheel_turned_whole_is_its_shadow_revolved` — the wheel's sweep to the
volume of its shadow worked apart from the build (each height's radii whose circle meets the wheel's
analytic margin, by adaptive Simpson) to 1e-6, to membership, to a closed STEP file, and the rod cut by
it to the same integral within the rod's radius.

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
the ring's seam circles and the contacts' own curves are read against it by the general reading (#814), and
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
  `plane_holding_rim`, `side_holding_rim`);
- **the neighbour across a band's seam at each end pose against the band's sheet, at the seam
  points alone** (#942 F3, `slide`'s `touched_at_seams`; `Touch`'s points, `Given::touches`).
  - The neighbour's face posed is the tool's boundary there and the sheet is the sweep's, so the
    two meet only where the face's contact lies on the sheet: where the band's contact crosses the
    seam into it. So does every face of the solid the face's candidate bounds, which lies within
    the tool posed.
  - Each such face is given those points against the sheet, and nothing else.
  - The circle where the neighbour meets the band touches the sheet there: a double root, which no
    search proves by a change of sign. It is read off the given points as a touch, with no search
    (`boolean`'s `on_known`).
  - Gate: brep-bench's bull-nose pushed in at a lean, its mill turned from a meridian that puts
    the touches mid-span on its side's rim. It was refused ("a circle … meets a extrusion was not
    proved"), and is now cut from the block to `swept_slid::Mill::plunged_away`'s closed form to
    1e-9 (`swept_slid::a_bull_nose_whose_rim_touches_its_contacts_sheet_mid_span_is_cut_from_the_block`).

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
a neighbour in a sharp rim (its fan not entered here; the contact's road builds it, #880 Q1b S2), a contact crossing both seams (two arcs), and a
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
`V − ∫ z n_z dS` over the faces turned up (`Mill::plunged_away`), to 1e-9. A round spanning only
the eighth of the tube above `v = −π/4`, so the flat end meets it in a sharp rim, is refused as
`Curved` at that rim by the admission, and built by the contact's road through its fan (#880 Q1b
S2), to the meridian's reading (`a_round_meeting_a_sharp_rim_at_a_lean_builds_through_its_fan`). Profiled (`SOLVENT_SWEEP_PROFILE`):
before, the bull-nose slid 5 at a lean was refused as `Curved` at its round in 53 µs; now it builds
in 3.4 s, its meetings 107 in closed form, 10 traced (the side's cylinders at the end poses against
the sheet, where they touch at the seam points; the end poses' tori against the planes and the top
rim's sheared cylinder) and 4 given (the sheet against each end pose's torus and against each seam's
plane); pushed into the block, 2.1 s, 139 / 6 / 3.

**A body built of sweeps cuts as any solid does (#818):** a cut operand that is a body built of
sweeps, or a chain of placements ending in one (`cad::swept_body`) — a gear rack-cut, cut from a
ring's blank so that the ring's teeth are the gear's spaces — is swept material
(`cad::swept_operands`), not a swept cut: built by its own road and subtracted by the kernel's
Boolean (`export::compose`), in the export and the page's stepped build alike.  Only a sweep, or
a chain of placements ending in one, is a swept cut a class reads.  A sweep cut directly from a
body so composed is still refused by name (`no_swept_cut`): the body it is cut from is named
apart.  The native path builds no such body and refuses it by name, `--kernel rust` asked.
Gates: `object_builder::a_body_cut_by_a_gear_whose_space_is_a_sweep_keeps_the_space`, the housing
and the same sweep cut from the ring they overlap summing to the housing's annulus to 1e-9;
`solid_sweeps_examples::the_ring_gear_keeps_the_gears_spaces_as_its_teeth`
(`solid_sweeps/ring_gear.sv`), a ring cut by a 20-tooth rack-cut gear, held to `rack_cut`'s
arithmetic for each tooth and to the page's stepped build.

**A sweep cut from a blank on the same arrangement (#699 C, #735 A4, `swept::closed::cut_from`):**
the blank, not a box, is the domain the candidates split, and its cells are kept where they are no
member of the sweep — one arrangement and one classification, with no closed solid built and then
subtracted by a Boolean. **The blank is split within the sweep's reach alone** (#847): the box the
candidates are sized by holds the sweep, as `closed`'s does, and is the reach of
`boolean::arrangement_reached` — the blank parted by its boundary too, every candidate's piece
outside it left out, and a cell a face of it bounds from outside kept with nothing read (the box
holds the sweep, so nothing outside it is a member). A blank's earlier cuts outside the reach are
neither split into cells nor read, and what the candidates lay on the blank's faces there (each is
sized by the box, a plane's block four times as wide) is healed out of its faces and its edges
(`boolean::rejoined`), so turned copies of one sweep cost about the same each, a little more as the
blank near the box gains faces, and leave the blank's edges whole: the issue's tube, its slab
screwed through it and cut at three placements, 141 s (23, 42 and 76 s a cut), 26 faces and 2 446
edges, 30 under 10 µm, is now 8 s (2.1, 2.5 and 3.0 s), 26 faces and 72 edges, none under 10 µm;
twelve copies a thirtieth of a turn apart, overlapping, some 8 s each. Gates:
`screwed_copies::a_turned_copy_of_a_screwed_slot_is_cut_as_the_first` (two copies remove twice
what one does to 1e-9, adding their own edges alone) and
`one_screwed_slot_leaves_the_tubes_edges_whole`. A bar slid along a
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
the centre's helix or on an end circle, those feet isolated by their enclosures (#814) and the nearest taken,
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
number of turns' rise, each such pair a helix (`geom::Pipe::returns`, isolated by the Krawczyk operator, #814;
`swept_loci::a_pipe_comes_back_onto_itself_along_helices`). Such a pipe is entered in stretches of
the roll of at most half a turn, each meeting itself nowhere, no two joints, nor a joint and an
end, a return's turn apart (`closed::twist::pipe_stretches`); one that does not come back is
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
thinner than the mesh's sag; `…_is_cut_right_or_refused_by_name`, slow tier). A sharp circle
leaning to a screw's axis the closed class refuses (`Curved` at the circle: its sheet meets the ball
at an end pose and the ball's pipe in curves not given in closed form), and the contact's road builds
it (`swept_closed::a_ball_with_a_flat_screwed_is_swept_through_the_contacts_road`, #880 Q1b).

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

**The contact's road under any motion (#880 T1d, `brep::swept::general`):** the candidates
of a polyhedral tool under a motion no constant twist takes, read per tool at each instant against
its twist read on the tool (`Twist::body`). Each face's `n · v` is affine over its plane, so its
contact is a chord of its characteristic line from edge to edge; each sharp edge's contact is where
its two faces' `n · v` differ in sign, between their roots along it or a vertex. So an instant's
contact is closed polygons, and its shape changes only at instants where a root passes a vertex or
two meet, found by halving the roll between readings that differ (a shape over less than a
billionth of the roll dropped as the passing state between its neighbours), and each stretch cut
again to a quarter turn and the tool's size of travel. Each piece lies on a **bearer**, a face
family's wall (`family::Sheet`) moving with the roll: an edge's line carried over the whole roll, or
a face's characteristic line over each stretch it comes near the face; the pieces of one bearer are
the faces of one candidate, strip by strip, sharing the rims between them, their moving sides exact
walls in space and in the bearer's parameters (`Pcurve::Curve`). What is known is given: two edges'
bearers with a vertex in common its path; an edge's and a face's of it the path where the face's
chord reaches the edge, over each stretch holding that end, where the two touch; each bearer's whole
meeting with the tool at either end (`contacts::pose_meeting`, read on any `Rolled` sheet, its
branches ending on the strips' corners there). Held: every given rail lies on both its surfaces
(`swept_general`). A bearer whose every line stands on one cylinder is that cylinder
(`on_a_cylinder`, within a few times its sheet's fit; its faces' parameters its edges inverted onto
it), so two such are one surface to the trim. **`closed` asks it where K2 cannot read the tool**
(`closed::general_built`, its candidates trimmed by membership and judged; a tool it cannot read
either keeps K2's `Curved`). **Built**: brep-bench's rod threaded at a growing lead, cut from the rod
to the volume and area OCCT and cadrs agree on at 7.5e-11
(`a_rod_threaded_at_a_growing_lead_is_its_bench_volume`, slow tier), and a cube tumbling two degrees
(`a_cube_tumbling_two_degrees_is_built_by_the_general_road`), and five
(`a_cube_tumbling_five_degrees_is_built_by_the_general_road`), each held to membership; a branch of a
sheet's meeting with the tool at an end pose takes each end to its own nearest corner within two
cells (`pose_meeting`), where both ends of the bottom edge's strip's branch had gone to one. **A given
meeting is the whole of it**: the arrangement seeks nothing more of a pair it is given, so two edges'
fans sharing a vertex are given, beside its path, every other branch of their meeting
(`beside_rails`, seeded where an edge of either crosses the other, each seed traced alone); where
the march in space refuses a branch (the two under a degree apart) it is marched in the two sheets'
own parameters (`marched`: the 3 × 4 Jacobian's null vector, minimum-norm Newton, landed on the
patch's edge, a cubic or, where it turns within a few points, a polyline within the tolerance).
**A fan that passes back through itself** (#942 T1d-cube, `general::crossed`): an edge's fan is one
wall over the whole roll, its strips its faces, and where the edge's line passes back through where
it stood a while before (the cube tumbling 15° and 75°: edge 5's strip over `[0.39, 0.50]`
crossing its strip over `[0.23, 0.29]`) two faces of one surface cross, which no given meeting
describes. Each two of a bearer's stretches are met as two surfaces, its wall restricted to each
(`Sheet::restricted`: its pieces over the stretch, the end ones cut short, each still the polynomial
through its own nodes), from where an edge of either crosses the other away from the rim they share
(`beside_rails`); a fan crossing itself nowhere stays one surface, as it was. One that does is cut
into the fewest runs of stretches that part every two crossing ones, each run its wall restricted to
it, its strips' surface and a candidate of its own (the trim meets no two faces of one candidate).
Each meeting given of the whole wall is given of each run, each curve as much of it as lies on the
run — a rail over the run's span, any other where its points stand within it — and each two runs
are given the rim they share and where they cross, met run against run, so a branch is one curve
across a run's stretches (traced a pair of stretches at a time, its pieces ended some tolerances
apart and closed nothing). The fan's double curve ends at the pinch where it is born, on the rim
two runs share. A seed lies on a branch traced already within the reach a point of two surfaces
meeting at their angle there has (`tolerant::on_meeting`): two fans meeting shallowly traced one
branch twice from two seeds, 4e-7 apart, laid side by side. A dump writes each wall with the number
it is shared by (`dump::shared`), so a trim read back shares its walls as it was built: a wall's
inverse starts from the last foot found on it, and walls read back apart moved a refusal's points in
their last digits.
**A branch beside a rail ends where the two fans touch** (#942 T1d-cube,
`general::ended_on_rails`): two edges' fans from one corner meet in the corner's path and in a
second branch beside it, and two branches of one meeting cross only where its surfaces are
tangent; the two fans are tangent on their path where the corner's velocity lies in the face
holding both edges, an instant that face turns, where the road cuts its stretches. There the trace,
its surfaces meeting ever more shallowly, stops short or runs on along the path (the cube tumbling
75°, edges 1 and 9: 3.95e-6 past the corner, two thousand tolerances, at a sine of 7e-7), and the
branch hung in the face as a slit, the piece holding both its sides. An end within its meeting's
reach of the path (`tolerant::on_meeting`) is carried to where the walls' normals turn past each
other along it (`(nₐ × n_b) · r′` changing sign), and to the corner of either wall there, the road's
own, where one stands nearer that root than the end did (the root, read through two numerical
feet, stood 5e-9 off it, past the pool's tolerance); the curve is cut there (`BSpline::segment`).
Not for one fan's runs meeting each other: their walls are one sheet across the rim they share, the
sine along it nought throughout.
**A fan's runs are found before the bearers are met** (#942 T1d-cube, `general::crossed` then
`given_to_runs`): the branches beside a rail two fans share are traced on each run's own wall,
run by run (`walls_of`), not on the whole wall a fan crossing itself folds over: there the trace's
feet fell on the fold its other run lays over it, and its branches ran 1e-5 off both walls, one out
and back along itself 3e-7 apart, a crossing 5.7e-6 off any of them left a piece unparted. A
meeting found run by run joins what its two runs are given. Any other curve given of the whole
wall is cut to the stretches of it that lie on a run's wall (`on_wall`: read at 64 places, on it
within what a patch is read to, `1e3 tol`, each change halved to the rounding), never given whole
where one point stood within the run (`shadow::stretches`). The cube tumbled 15° is built (`swept_general`'s gate, held to membership through its box: a
count of 40000 points gives 3.463 ± 0.018 against its 3.4739).
**A fan's crossing with itself is built from the pinch it is born at** (#942 T1d-cube,
`general::pinches`, `general::handle`): where two runs of a fan share a rim, the fan's sheet is
singular on it at each point its rate across the rim (`Sₛ` less its part along the rim) turns over
— a pinch, the instant the edge's two faces' contacts reach it together, where the road cuts its
stretches, so the runs' corner there. About it the sheet is a Whitney umbrella: with `Sₛ = λ Sᵤ`
there, the points `(u_p ± λt, at ∓ t)`, one on each run, meet to the third order, and the crossing
is the pairs `X(u₁, at − t) = X(u₂, s₂)`, each solved by Newton from the last, marched out in `t`
until a run's span or the edge ends, interpolated from the corner and refined until it lies within
half the tolerance of the pairs solved between. Traced toward the pinch instead, the runs meeting
ever more shallowly there, the crossing lost itself on the rim they share (the 15° cube: its edge
11's crossing wandered 2e-3 along the rim either way of its pinch); a branch traced to a pinch is
carried onto it (`ended_at_pinch`, an end lying on the rim carried to the nearest pinch along it)
and given no more where the pinch's crossing is built. **A wall's foot** (`family::Sheet::newton`,
`Sheet::inverse`): Levenberg–Marquardt over the wall's domain, each step taken only where it comes
nearer, one leaving the domain held at its bound and the other parameter stepped alone along it —
beside a pinch the wall's two derivatives all but parallel, a full Gauss–Newton step ran along the
way the wall barely moves, onto a run's rim, and stalled there 1.4e-6 off a point lying on the
wall. On a run of a fan that passes back through itself (`Sheet::layered`, set by `crossed`) the
last foot found is trusted only where it settles on the point itself, to the rounding, and
otherwise the nearest four of a grid are each settled and the nearest kept (the 75° cube's edge 5
fan passes within 5e-3 of itself, a foot from the last one settled on its other layer, and a face
was kept facing the wrong way); on any other wall the last foot is trusted where it settles within
a thousandth, and the search settles the grid's nearest point alone (asked of every wall, the care
doubled the hypoid gear's trim, 125 s to 269 s). A curve on a face read by its inverse takes the chord of its feet for
its rate where the face's two derivatives are parallel (`topo::inverse_derivative`: the volume
read NaN at a pinch). The bench's 75° cube is built (`swept_general`'s slow gate, held to
membership: a count of 40000 points gives 11.029 ± 0.072 against its 11.0993). **Not yet built**:
the cube tumbled 10° and 12° is refused in its trim, a piece the boundary at one point and off it at
another (candidate 6 near `(2.99, 0.77, 2.59)`), with no fan crossing itself. **A ring's fold pair born partway**
(`brep::swept::rings`): a pair narrower than its band read by its least `s''` along the band, on the
curve's own branch; the roll cut where the contact changes shape (`rings::changes`), each stretch
fitted alone, its tubes sewn across the cuts and trimmed; a sheet refused as folding only where it
turns over. The stretches are sewn row by row (`sew::sewn_open`): a vertex merged only with other
stretches' vertices on its own row, a row's rim cut only at that row's vertices — beside a
swallowtail's point a stretch two-hundredths of a radian wide has its two rows within the tolerance
of one another. A row is the cut it sews (#942 F3, by construction): `across` names which cut's row
a face's parameters lie on, and the two stretches either side of a cut, fitted half the roll's grain
clear of it (`closed::grain`; a billionth of the roll before #942 F6), have one row there — told apart by their parameters' values, 6.3e-9 of the roll
apart beside the half turn's cut, no vertex of one was merged with the other's, nothing was sewn,
and each stretch's row, lying along the next stretch's sheet, was searched for where it met it
and not proved (gated with no sweep by
`brep::two_stretches_of_a_sheet_fitted_either_side_of_a_cut_are_sewn_along_it_as_one_row`, and on
the half turn's own sheets by
`swept_rings::the_half_turns_stretches_are_sewn_across_its_cut_into_one_tube_a_contact_curve`).
A double curve's preimage on a tube is unwrapped along it before the swallowtail's tail above it is
read (#942 F6), as it already was where it is cut at the seams between faces. One crossing the seam
where the tube's faces close round, read in `[0, pieces)`, ran from one end of the tube to the other:
all but a sliver of the thin torus's middle stretch was dropped as tail, and its tube was sewn into
two candidates where it is one
(`swept_rings::a_double_curve_crossing_its_tubes_seam_bounds_a_tail_beside_it_not_the_whole_tube`,
slow tier). `closed::ring_entered` is what the road enters before it trims: the tool posed at either
end, the sheets sewn, their meetings given.
**A double curve's end inside a face is the tube's own vertex** (#942 T1d-tori,
`rings::split_chords`, by construction): where a double curve's preimage reaches the roll's end on
one leaf, at the rim touching the tool, its other preimage ends where the leaf it passes through
crosses the tool — a chord ending on another inside the face's square. That end stays where it is
and the crossing is cut at it; a trace's end, placed only to its step, is put on the chord it ends
on, as before. Put on the crossing's polyline instead, the thin torus's double curve ended 1e-6
(3.5 tolerances) from the rim's corner it is, two vertices of the tube on the tool, and the
touching rim lay in the tool's face as a slit, its piece read as the boundary on one side and
buried on the other (`swept_rings::a_double_curve_ending_on_a_tubes_crossing_with_the_tool_is_one_vertex_with_it`,
slow tier). **A grid seed is a root** (`boolean::grid_seeds`): each grid line's change of sign in
the other surface's signed distance is halved until it stands within the tolerance of nought, and
seeds nothing where it does not come to it — a jump, the other's nearest foot leaping across (the
thin torus's "two tubes meet at 0.00°" was seeded 0.03, the tube's diameter, off the other tube,
its distance read across a net's seam where its inverse fell on a far corner of the net).
**A double curve traced from a row's seed is joined to its swallowtail point** (#942 T1d-tori,
`rings::joined`, `Fitted::ends`): beside the point the curve's two leaves stand within the
tolerance of one another over a band (the thin torus's tail covered less than the tolerance deep
some 2e-3 of the roll about its points), so the curve is fixed there to the tolerance alone; marched
toward the pair, the thin torus's preimages slid onto a fold's inner leaf 2e-3 short of its points
(read by `depth::cover`: `s''` below nought, then covered by nothing) and the roll was cut at false
apexes 1.3e-3 short. The swallowtail points are sought in every stretch, and a pair of the march
within its parting of one is joined to it along the swallowtail's normal form — along the kernel
linearly, across it quadratically — where every point of the join stands within the tolerance of
its partner and on an outer leaf. Every point of both thin curves then agrees with the far cover
to 1.7e-8 (tolerance 3e-7), and the roll is cut at the true points, 0.8371692 and 1.1404747. Built
outward from the point instead and stepped by arclength (the pairs `(θ₁, t₁, θ₂, t₂)` continued),
the march settled among pairs mirrored across the fold, missing one another by the cube of their
parting, within any residual the tolerance allows. **A tube's tail is read from the depth**: a place
is tail where an instant other than its own covers it deeper than its fit (`depth::cover`), not above
the curve over its first preimage's angle, which the joined curve turns back over.
**`tumbling-torus-thin`, not yet built** (after the join): refused on the row sewn at its first
pair's birth (near `(3.9578, 1.5333, 1.6132)`, face `(2, 6)` of the dump), beside where the second
preimage, dipping below the row into the stretch before, comes back across it. Before the join:
past both, its trim is refused beside the swallowtail
point where its fold pair is born (near `(3.961, 1.480, 1.663)`). The roll is cut there twice,
where `changes` sees the pair (0.8356864) and at the double curve's swallowtail point (0.8358228),
and the double curve's preimage on the tube dips between them to 0.835783, so a stretch 1.4e-4 of
the roll wide holds a chord of it; the tail it bounds stands within the tolerance of the boundary
(the sweep's depth there 2.5e-8 to 2.5e-7, the tolerance 3e-7), so membership reads the pieces
there, a ten-thousandth wide, both ways. Which of the three is the swallowtail point is the
question left: the fold pair's birth, its double curve's lowest point and the meeting of its
preimages are one point of the sheet.
A curve a tube is cut along where it crosses the tool at an end pose carries, as its pcurve, the
places on the face it was traced through and the meeting's own point settled between each two, each
at the curve's own parameter there — the coarsest cubic through every so many whose image holds the
curve's bar, a quarter of the tolerance, at each and at 64 places along it — and is inverted onto
the face only where none holds (#942 F3, by construction: beside a fold pair's birth the leaf folds
back on itself within 2e-4, and the half turn's cut, inverted from the line between its ends,
landed on the other fold; gated in
`swept_rings::the_half_turns_stretches_are_sewn_across_its_cut_into_one_tube_a_contact_curve`).
A trace of a face's meeting with the tool of no length — its points all at one place on the
face's square, within `ON_SIDE` — bounds nothing and is no chord; one along a side is kept, a rim
lying on the tool (#942 F6: the near-orthogonal tori's, two points 2.4e-13 apart on the rim where a
crossing ends, put two chords' ends at one place; `swept_rings::a_trace_of_no_length_bounds_nothing`),
nor is one its ends' places put at one place when they are snapped to the seams.
**A given curve a face holds as several edges is laid as each** (#942 T1d-tori, `boolean`'s
`edges_that_are`): F3's class B rule lays a given curve that is a face's own edge as that edge, and
a face may hold one curve as several, cut where another meeting reaches it — the full turn's tube
held its row at the start, the rim it touches the start torus along, as two edges, the end pose's
chord between them where the tool posed at the end holds the tube beside it. The first alone laid,
the start torus was parted along half its rim, and a piece of it reached round the other half from
the side the tool trails to the side it leads, refused as read both ways (gated with no sweep by
`arrangement::a_given_curve_a_face_holds_as_two_edges_is_laid_as_both`). **A place above a double curve that lies below the stretch there is tail only where a pose
holds it** (#942 T1d-tori, `Ring::held`, read with the tail over the angle below): a curve that dips
into the stretch before and comes back across the tube's seam has every place of the stretch at the
angles between above it, and a tail ends where its curve meets the tool at an end pose — so such a
place is tail where a pose holds it more than the tolerance deep (read off the ring's exact signed
distance, best first over the roll as membership reads; a sheet on the boundary stands within its
fit, a tenth of the tolerance, of it). The full turn's last stretch kept a face between its double
curve and the seam, 3e-3 inside the sweep, its crossing of the tool at the end hung open on the
seam; taken whole, the column dropped the corners above the end pose's chord, which stand on the
boundary to 1e-8; and a probe proved inside taken alone, with no curve below it, dropped a region of
the tumbling torus's tube that another tube's sweep covers over part of it only, its solid then
wrong at a point.
**A double curve's end stays where it is**
(`rings::split_chords`): it ends where one leaf reaches the end of the roll, on the tool posed
there, so on the other leaf's meeting with that tool, which is cut there; put on that trace's
polyline, the full turn's stood 2.4e-6 inside the end torus, two tolerances from the same point on
the leaf whose rim it is, and the two never met.
**A fold pair's double curve is built from its swallowtail point** (#942 T1d-tori,
`rings::swallowtails`, `rings::double_curve_from_apex`): where a pair is born at an instant its two
folds are the branches of one curve of `(θ, t)`, `s'' = 0`, tangent at the birth to the instant's own
contact, whose rate `Σ_θ` is no kernel of the sheet's — so the birth is an ordinary point of a
cuspidal edge, and no double curve begins there. One begins where a fold runs along the kernel of the
sheet's rates, `k = (−Σ_θ · Σ_t / |Σ_θ|², 1)` read where `Σ_θ` and `Σ_t` are one way: the swallowtail
point, where `∇s'' · k` passes nought along a branch (the half turn's left fold 0.0072 of the roll
after its birth at 1.4766, the kernel's `dθ/dt` −2.06; a coverage of the leaf — each point's least
signed distance from the ring over the roll — put the double curve's bottom there, and its other
preimage running back across the cut into the stretch before, to 1.463). The rows' search finds no
pair there: one preimage lies in the stretch before the cut, which the stretch's net does not reach,
and beside the point both lie within the bridge. So each branch is read at the stretch's ends and
halved to the rounding, and the curve built outward from the point: its first pair from the
swallowtail's normal form — the preimages `(x, ±τ)` along the kernel with `x = −2τ²`, the folds at
`x = −6τ²` — settled by Newton's method at `θ ∓ s`, `s` halved from half the double curve's greatest
step until the chord from the point sags within the tolerance; then marched out, the point itself
on the curve between its preimages. Away from a pair, a pair whose preimages have met or crossed
within the least step is none (beside the point the march slid onto the pair of no distance every
point of the sheet is, and ran along the sheet), nor, on a curve built from its swallowtail point,
one whose second preimage comes back over the angle (beside the half turn's point, where the two
sheets cross at an angle of the pair's width, the pair settled within its residual 1e-6 of the
angle back; a curve traced from a row's seed is not yet held so, the turned torus's coming back
over its angle); and where the last pair alone does not settle the next it is sought from the line
through the last two, the point the first of them. Not yet in place
of the rows' seeds, which are traced toward the point as before: beside the thin torus's second
pair, born 1.2e-3 of the roll before its swallowtail point, the first preimage's angle turns back
within 2e-4 of it, and the curve, stepped by that angle, is no graph over it there. **The tail is
above the curve over the angle round the axis**, the place's angle read round the tool's axis where
it stands at its roll, against the whole curve: it is traced a graph over its first preimage's angle,
not over `u`, which the faces' seams shear as the pieces they share move (the half turn's curve
between its birth and its swallowtail point ran back over `u`, and the tail beside it was kept), nor
over the stretch's part of it alone, past whose ends it runs on (a piece of the tail beyond the part
in the stretch between the cut and the point was kept). *Found, not yet fixed:* a tube's seam edge
is shared by the faces either side keyed by its lower place alone, so where one face walks the seam
to a place the other does not have — the half turn's end pose meets its sheet along a seam 3.3e-5 of
the roll from its corner, a trace of a face's lying along the seam — the two are given one edge for
two stretches of it, and the stretch's tube enters with a loop broken there. Laid instead as edges
between each two places, the trace along the seam the seam's own, the ring along a spline's end-pose
meetings, given as curves of their own, lay along the seam's edges twice and its arrangement did not
close: the fix gives a trace along a seam as the seam's own curve.
**Every threshold the roads under any motion read is the tolerance's, a feature size's, or the
rounding's** (#942 F6), none a number of its own:
- *the rounding* — a value read by cancellation is nought where half its digits are its rounding's,
  `HALF_READ = √ε` of its scale: a meridian's normal velocity (a meridian touched whole, read in
  closed form), a narrow pair's least `s''`, a root's jump at a piece's end, a fit row's area
  element; `G` is carried, not divided, within `ε^(1/(order+2))` of a rim it touches to `order`,
  where what its division leaves to the rounding is what carrying it on straight leaves of its
  curvature;
- *the tolerance* — the roll's grain, over which no corner of the tool moves it (`closed::grain`):
  two shape changes nearer than it are one instant, and a ring's stretches are fitted half of it
  clear of a cut, their rows so within the tolerance and sewn as one, a row read within twice that; a
  double curve stepped so its own chord sags within it at its middle, halved from at most the angle
  over which the meridian's chord sags it (`√(8 tol / r)`) down to the angle over which the meridian
  moves it (`tol / r`, nearer two points are one) — at `√(8 tol / r)` alone the turned and the thin
  tori's double curves were traced too coarsely and their trims refused an edge against a face — a
  far apex within the step it stopped at either side, a seed on a traced curve where its point
  stands within twice it of the curve's polyline; a tube's own branch within it of its net; seeds off a rail and on a traced
  branch, within twice it; two corners at an instant one within the fit's exactness;
- *the normal form and the trace's step* — a narrow pair's crossing sought from the bridge's ends,
  its preimages within `√3 − 1` of the bridge's half-width past them (`θ³ − 3h²θ` crosses at
  `±√3 h`); a double curve's first pair from its swallowtail point either way along the kernel, at
  half its greatest step and halved by its own chord's sag; a pair whose preimages stand within
  its least step one point; a swallowtail point's rates read by central differences at `ε^(1/3)` of
  the angle and of the roll, where their rounding and their truncation balance; a chord's end within a trace's step (`MET_STEP`) of another chord ends on it, and two
  traces whose ends stand that near are one chord;
- *the structure* — a side of a face's square is straight, its way its own; a halving stops where
  its middle stands between its ends; a face's characteristic line read as far as the face's reach;
  feet within the slack of a line make no circle; the end pose a stuck instant is nearer is the one
  that holds it, and the way the tool's point moves there is the motion's own velocity (no step);
  every trim's hair is #880 Q1b's one rule (`closed::road_trim`), which the roads under any motion had
  copied. (The screw's domain's room, a twentieth of its extent, and the trim's grown box's, a
  tenth, are #806's and E1's, older than these roads, and stand. The domain's room decides how long
  a screwed sweep takes — the fine thread with sharp crests 24 s at a twentieth, 70 s at the tool's
  size, 118 s at the box's own extent — and no longer whether its solid is right: the drum screwed a
  short lead through most of a turn, wrong at a tenth, the tool's size or the extent while the
  complex read tied faces a thousandth of the box in, is right at a twentieth, a tenth and half the
  extent since it reads them nearest first, `brep.md`'s A1.)
**The tool at the roll's two ends is met where both bound the sweep** (#942 T1d-tori,
`depth::ends_met`): over a full turn the end pose's tool stands beside the start's and meets it.
Most of that meeting is buried, one tool or both covered by the instants between (where their tubes
touch: no cut is needed there). Along two arcs, each some 0.02 long, both are uncovered except by
one another, and no edge crosses those arcs, so the trim's search missed them. Read under the depth
(`cover`), an arc on the boundary ends where its cover changes hands, at a corner on a seam the road
gives on one pose (a tube's trace with it, its rim) where the other pose's signed distance changes
sign. So the corners are found along the given seams by a certified march (steps no longer than
the larger of that distance and the tolerance) and kept where neither pose is covered there by more
than the tolerance. The meeting is traced through each corner (`meeting::through`, within a box about it grown until
the branch through it reaches another corner on each side where it is uncovered), each point
settled on both surfaces by alternating feet; a corner whose branch is not read gives nothing (the
torus turned across a block: a zero-set search there asked four million cells for 30 s). It is cut at the corners, and each
stretch whose middle neither pose covers is given (gated in
`swept_rings::the_full_turns_end_poses_meet_uncovered_along_two_short_arcs_given`). A witness's
missing meeting is completed at no crossing that membership proves the sweep holds or leaves
(`trim::completed`): there it is buried and bounds nothing. Before this, the full turn's tori were
given a meeting traced through where their tubes touch, and its trim was refused across the instants
after the start.
**The torus turned whole** (#942, built below; the history of its refusals kept): the tumbling tori
fit, are cut and sewn into one tube a contact curve, and were refused past the meetings their sewing
settles — where its two end poses' tori meet (#942 T1d-tori: past its start and end tori's pieces,
above, the boundary left open along the start torus's seam circle beside a piece the end torus
crosses): over a full turn the end pose is
the start turned 0.75π about a line parallel to the torus's axis, two congruent tori in one plane
whose tubes touch at two points beside each place their core circles cross. Their meeting is given
where it bounds the sweep (above). The start torus's uncovered island at angles (2.745, −0.887)
lies between the two strands, 0.13–0.16 deep, that the tube near the end (`t` 3.03–3.13) covers,
bounded by that tube's trace with the start pose, the arc above, and the start pose's rim. That
trace crosses the rim at `(2.5497, 3.1166, −0.1932)`, both curves edges of the one tube; the
arrangement now cuts them there (`boolean`'s `given_crossings`, brep.md), where the island's piece
reached round the rim's end to the start torus's seam. The trim is refused next along the rim
past that crossing, `(2.5295, 3.0460, −0.1999)`, buried (membership the same 0.05 either way along
each axis): the tube's piece beside it is kept, mixed — the boundary beyond and what the tube late
in the roll covers — since the tube's crossing with itself over a whole turn, no fold pair's double
curve, is not cut into it; it leaves the rim at that crossing, and a seam read from the depth
(`depth::seams` over the tube's early sheet, seeded there) is what would cut it. That crossing is
read (`depth::passed_back`): each end row where its far cover changes sign, the cover's boundary
traced through there with no search (`depth::seams_through`), the runs an instant inside the roll
covers kept and ended at their corners where the covering instant reaches the roll's end
(`depth::corner`), each with its partner on the covering sheet, as a double curve is laid
(`swept_rings::the_full_turns_tube_passing_back_over_its_start_is_read_from_the_start_row`; 29 s,
nearly all of it one long run). It is entered (`closed::ring_entered`), cut into the stretches
with the double curves: the start torus closes, and the run on the end tube's row at θ 0.48–1.17
(covered by the tube at τ ≈ −1.94…−2.09) has its partner end on the end torus's trace across the
mid-roll tube, a closed curve inside a face. **A closed trace inside a face is read through a place
the caller knows** (`rings::traces`' seeds): where a curve cut into the face ends inside it on the
tool, settled onto the zero set and traced round until it closes on it. **A component of a face's
chords touching no side is a hole** (`rings::with_holes`, `loops_of_square`): the planar graph's
walks go round each piece with it on their left, so such a component is walked once clockwise
round its outside, and that walk is given as an inner loop to the least counter-clockwise piece
holding it; a piece touching no side is judged just left of its longest chord's middle (gated in
`swept_rings::a_closed_chord_inside_a_faces_square_is_a_hole_in_the_piece_about_it` and
`the_full_turns_tube_is_holed_where_the_end_torus_crosses_it`: the end torus's loop and the
partner's chord bound the hole, the loop's inside and what the late tube covers beside it dropped).
**The full turn is then built** (`swept_rings::the_full_turn_is_built_closed`: 35 faces, volume
23.6738854, cadrs's 23.6739, held to membership), past two last refusals at the double curve that
begins at the swallowtail point where the road parts the roll (2.2318). Its join to the point
(`rings::joined`) is laid in chords doubled until each leaf's chord straight in its own `(θ, t)` —
what a tube's face draws between two of the curve's points — stands at its middle within the
tolerance of the curve's own middle: across the kernel each preimage is the parabola `x = −2τ²`
and the leaves bend opposite ways, so their drawings parted by 3.8e-4 beside the point (3 chords
then, 24 now). And the stretch below the cut's drawing, merged by the arrangement into the face
above it, keeps that face's own pcurve carried onto it (`brep.md`, `carried_pcurve`): inverted,
it landed on the leaf past the fold, 1.3e-5 off at 5/8 of it. It is built wherever either end
pose's seam stands (`swept_rings::the_full_turn_is_built_wherever_its_end_poses_seams_stand`: each
torus turned to eight seams, each built, checked and its volume the one within the tolerance's
worth), and with its observer's origin moved by an ulp (the volume within 3e-9) or by 1e-6
(`ring_profile full-at`); each face's deepest inner place is covered by no other instant beyond the
tolerance (`the_full_turn_is_built_closed`). (The refusal named the start torus's piece, the tube tangent to it along the rim: a point a hair
across on the torus read as on the kept tube. A point read across an open edge is now on a kept face
where a face of the solid on its own candidate's surface holds it, `trim`'s `missing_across`; the
refusal names the early tube's mixed piece.) The island's read point is uncovered, 0.122 clear
of the end pose; the thin one beside the swallowtail point where its fold pair is born
(above: its two tubes' meeting at under a hundredth of a degree was a seed off the other tube, no
root). (The half turn, refused here where the trim found the boundary open 8e-4 from its
swallowtail point, is built: its pair parted at the point, below.) A double curve's
points are placed on a stretch by the seams' angles of the stretch's own net (`rings::cut_along`),
not by an instant read for each (#942 F4: the turned torus refused in 32 s where it took 106, 74 of
them reading instants; the thin one in 58 s where it took over forty minutes); what is left is the
shape changes found (6.6 s) and, in the trim's Boolean, its fitted tubes' edges read against their
faces (693 searches, 81 s of the cores' time). **A ring along a spline** (the whole meridian touched at every instant, `rings::root`
in closed form there) closes its sheets — a bridge holding the join its label turns over at, a
trace leaving beside a corner ending at a nearer crossing, a cap both faces of a seam trace into one
corner dropped, a chord refined by the meeting's own points where its cubic strays — and its tubes
are arranged with the torus at both ends (#942 T1d-ring, at the bench's tolerance:
`swept_rings::a_rings_tubes_beside_its_whole_touched_meridians_are_arranged_with_it_at_both_ends`).
**A boundary read from the depth enters as a seam** (#942, `swept::depth::{Covered,seams,Seam}`):
`depth::Covered` is the far cover's depth over a ring's sheet in its `(θ, t)` on one contact — not
defined where the sheet folds away, its neighbours hold it or nothing else covers it — with its
gradient by the envelope theorem (the tool's normal at the covering instant against the sheet's
rates; differences at `zero_set`'s half digits read the sheet's root's noise), and `depth::seams`
traces its zero set over a box with `zero_set::trace` from the caller's seeds (events it knows),
each piece's chords held to the tolerance in space by pulling every chord's middle onto it (beside a
swallowtail point the sheet's rates are nearly one way, and a first-order distance through the
inverse metric took a chord 3.6e-5 off as near), a piece traced twice from a seed dropped, a cell
left unresolved refused. Each `Seam` carries its sheet's points `(θ, t, σ)` and the covering
instant's preimages `(θ′, τ, σ′)` where `τ` is inside the roll — `Seam::curves` in the form
`rings::cut_along` lays a double curve's, so a caller pushes them among the curves cut into the
stretches. On the half turn it traces the double curve from the cut where the pair is born to where
the end pose covers it, one piece rim to rim, each partner point covered at its own point's instant
(`swept_depth::the_cover_s_boundary_traced_from_the_depth_is_the_half_turn_s_double_curve`, 4 s).
The half turn's refusal was no missing seam: the cut is where the pair's least `s''` passes nought,
to 3e-7 of the roll; the band its far cover holds deeper than the tolerance before the cut is
bounded by the double curve; and from its swallowtail point to where it crosses the cut the double
curve stood 0.2e-6 to 3.03e-6 from the cut's contact in space (the tolerance 3.2e-6), so the
arrangement cut the cut's edge at the swallowtail point and left edges used four to six times.
**A fold pair born partway is parted at its swallowtail point, not its birth** (#942 T1d-tori,
`closed::ring_fitted`, `rings::fitted_unborn`): the birth's row is a construction line, the double
curve geometry, and the line is moved off it. Where the stretch holding a swallowtail point begins at
the cut where its pair is born (the stretch before has fewer bridges), the two are fitted as one up to
the point, the pair read as not yet born — the stretch's pieces those at its start, and an instant
whose shape is not theirs read with no narrow pair put in, one touching piece across the bridge,
the folds check passing over the newborn pair's bridge as over any bridge — and the rest as one from
the point; the row there crosses the double curve only where it ends. The sheet over the bridge is
the sheet itself, folded back between the folds as a bridge's is, and that sliver lies wholly in the
tail above the double curve, which the trim drops: at eight rolls between the half turn's birth and
its point and 64 places across each bridge, every place between the folds (360) is held by a later
instant (to 2.3e-4 deep) and every place the tail rule keeps is held no deeper than the rounding
(`swept_depth::the_half_turns_fold_pair_bounds_nothing_the_trim_keeps_before_its_swallowtail_point`).
The fold itself, held by the instants beside its own, is 4.9e-7 to 1.6e-6 thick there (`depth_probe
sliver`), under the tolerance. Read unborn is the newborn pair alone: each curve's pair whose least
`s''` stands nearest the swallowtail point's angle (`Fitted::unborn` carries it). Every pair read so
dropped both of the thin and the full turn's second pairs' stretches, which begin past the first's
point with the first already born, and their curves no longer turned onto the start's ("T|T against
FT|T"). brep-bench's
`tumbling-torus-half-turn` is built, checked and judged as the bench calls it: 20 faces, volume
12.2377154 (cadrs 12.2377), held to membership, 8 s in the test profile
(`swept_rings::the_half_turn_is_built_closed`). A Monte Carlo count of the definition read 12.198 ±
0.021, 1.9 of its deviations below; read point by point instead (`ring_profile half-turn 90 agree`),
the solid and the definition (`Swept::member`) differ at 6 of 200 000 points of its box, none
unsettled, each within 4.7e-5 of the boundary by the ring's own depth, inside the 1e-4 sag of the
mesh the solid was read by, and both ways. **A double curve's point past a stretch's end ends its
line** (`rings::cut_along`), though the point before stood on the end: a swallowtail point stands on
its row, and a preimage leaving the stretch there and coming back across the row further on was
laid as one line with it, joined along the row — the thin torus's second preimage dips below its
point's row by up to 4.5e-3 of the roll and crosses it again 0.0316 of an angle along, standing up
to 2.3e-5 off the row's contact in space between, a lens of the sheet, and the double curve was laid
on the row's own edge. **A far cover is read from the top of its own contact's climb**
(`depth::cover`): the own contact ends where the point's distance from the tool first falls a
rounding below the greatest sample so far. Read against the sample before, a climb barely past the
rounding followed by a slow fall had no two neighbours a rounding apart, and the point was read
`Alone`: the lens the thin torus's second double curve's dip cuts off below its swallowtail point's
row climbed to 3.1e-8 against the fit's 3e-8 and is held 2.2e-6 deep 0.018 of the roll on, and was
kept as boundary where the tail above the row was dropped — the trim then refused with the dip's
edge used by one face (`swept_depth::a_far_cover_past_an_own_contact_that_barely_climbs_is_read_from_its_climbs_top`).
So read, the lens is wholly tail, bounded by the dip and the row, as the swallowtail's normal form
puts it (the double curve's parabola, `x = −2τ²`, dips three times as far below the point as the
folds', `x = −6τ²`, whose lowest is the birth: 3.46e-3 of the roll against the 1.15e-3 from birth to
point, the dip read to 1.1370). The thin's folds are thicker than the tolerance, unlike the half
turn's — held by the instants beside their own 6.3e-7 to 1.4e-6 deep at the first pair, 2.5e-7 to
8.1e-7 at the second, from an eighth of the way from birth to point up to it (`depth_probe
sliver`), against 3e-7 — but lie wholly in that tail, held by the neighbours (`Near`) or by a later
instant, so a pair read unborn still bounds nothing kept.

**A bridge's end gives way to a double curve within the arrangement's band of it** (#942 T1d-tori,
`rings::give_way`): the end is a construction line — where the contact's `s''` leaves the band a
fold is read in, at the depth of the pair's least — and the double curve is geometry, so where a
preimage comes within the arrangement's band of an end (`tolerant::ON_CURVE` tolerances, within
which it takes two curves for one), the end is not put there: over the stretch it is the preimage
itself, the face before the bridge ending on the double curve and the bridge beginning there, one
seam by construction, laid as no chord (`cut_along`). A preimage beside its swallowtail point starts
on the fold (`s''` nought) and crosses the end as `s''` climbs past the least's depth — the thin
torus's first curve's first preimage reads `s''` at 0.08 of that depth by its point, crosses it 0.036
of the roll above (at (3.96004, 1.50408, 1.64131)) and stands at three times it where the roll ends,
the normal form's 2 between; where it crosses, the sheet's rates all but one way, it ran within
8.8e-7 of the end in space (three tolerances) over 9.9e-4 of its length, its row vertex (3.96007,
1.50335, 1.64197) inside the band of that seam, and the arrangement laid the strip of the bridge's
face between the two as one edge on both faces, a pcurve 8.47e-7 off it. Each branch of each curve
(its points between swallowtail points) is read over each stretch: its points there one run that
only climbs or only falls in the roll, beside the bridge's end nearest its middle point, its
nearness each point's distance from that end's polyline. A branch within the band that spans the
stretch is the end over it, and over every other stretch it spans, so the rows the stretches are
sewn along meet; one not spanning the stretch gives no way. The end is read on the branch as a cubic
spline of its angle over the roll through the traced points (settled row by row beside the
swallowtail point the pair is ill posed — two families 1.5e-6 of an angle apart 6e-4 of the roll
above the thin's first point, the end zig-zagging 2e-6 in space, which no fit closes — and read
straight between points 5e-4 of the roll apart, the angle's second derivative 130 there, it misses
by 4e-6). On the thin the first curve's branch gives the first pair's bridge's start way over both
stretches above its point, and the second's rising branch (crossing the end 0.03 above its point)
the second pair's bridge's end over the last; the branches clear of every end (3.6 to 18 times the
depth) give none, nor does the half turn's
(`swept_rings::a_bridges_end_gives_way_to_a_double_curve_within_the_arrangements_band_of_it`). The
half turn's birth row and the thin's site A (a preimage laid along a row) are the rule's other form
— the construction line moved clear rather than put on the curve — applied to rows by hand: a row is
where a stretch's fit begins, not a seam a face's end can be moved along, so the half turn's is moved
to the swallowtail point, where it meets the curve only at its end, and a line leaving a stretch at
its row ends there. `give_way` reads bridges' ends alone; the rows keep those handlings.
`tumbling-torus-thin` is built: 28 faces, checked at the call's tolerance and judged, held to
membership, the solid and the sweep's definition agreeing at all 20 000 points of its box
(`ring_profile thin 75 agree 20000 2e-5`), its mesh within 1e-5 enclosing 0.6962961 (cadrs 0.69630;
`swept_rings::the_thin_tumbling_torus_is_built_closed` holds its mesh within 1e-4 at 0.6962402),
about two minutes in release, 19.5 s of it the Boolean (18.3 s its grid seeds). **A sheet enters with
its seams first** (`rings::seams_first`): where the arrangement takes two of a sheet's edges for one
it keeps the first (`boolean::merged`) and reads every other use as the surface's inverse of it, and
the face given way ends on the fold at the swallowtail point, where that inverse is ill posed. With
the chord on the other leaf kept, the bench's input passed the check by luck (its seam's rate in `v`
read −0.997 against its feet's −5e-6) and with the observer's origin one ulp off, or 1e-6 either way,
it was refused, a pcurve 1.2e-5 off its edge beside the second swallowtail point, its foot found on
the leaf folded back. Kept so, the seam is the face's own line and the chord's leaf, clear of the
fold, takes the inverse; all four build and check, their meshes' volumes within 1.2e-6 of one
another. With `boolean::carried_pcurve` (the full turn's: a piece merged into another edge carries
its own pcurve, not the surface's inverse) every input builds without the seams first too, but the
face given way then carries the far leaf's chord as its seam on its folded corner and
`props::volume` reads 0.054; the two together read 0.6963028 (cadrs 0.69630), the same to 1e-9 one
ulp off and in every run (three builds alike to the last digit). The check at the tolerance passed all
of these: it holds each pcurve's points on the surface, not on which leaf of a fold — a seam carried
on the far side of the fold is on the surface to the tolerance, and only the face's region in its
parameters, which the flux reads, is wrong; `agrees_over` and `ring_profile agree` read the mesh,
which follows the edges in space, and could not see it either. To refuse it the check would have to
read each loop's chart consistent along it — the area factor's sign on the face's side of each
pcurve the same as at the loop's other uses. The Boolean's 19.5 s is its grid seeds over the two
tubes' face pairs: the ring road's trim resolution is the tool's size over 32, 0.063, coarser than the tube (0.03), so
every near-boundary ball of one tube reaches the other's.

**A seam's crossing within the tolerance band of the rim is the tube's vertex** (`rings::traces`):
where a touching piece's rim meets a dipped one's the relative curvature across the rim is nought
at the vertex, and beside it the tube and the tool stand within the fit's error of one another all
along the seam between them; where along it `G` changes sign is the fit's slope across the rim
(2.7e-5 there), not the meeting's. A crossing of a seam whose seam down to the rim the tool stands
on stays within the tolerance of the tool (read at the bracketing grid's points, `F` itself, which
both faces of the seam read alike) is no crossing: the trace reaching it runs into the tube's own
vertex, as it does where the face's corner is nearer. Found 4e-4 of the roll up the seam, the
chord traced from it zigzagged 1.1e-5 off both surfaces round a sliver face laid over its
neighbour ("faces … coincide along it"). Two chords run into one vertex so (the ring's end pose,
two crossings of the torus converging on its corner) part the face as the planar graph of its
chords does (`pieces_among`), not round its square alone. **A grid's seed is a root on the other
face's patch** (`boolean::grid_seeds`, `off_patch`, `kept`'s reading): a sheet's distance along the
normal at a foot clamped to its patch has zeros of its own past the patch, and a seed laid 1.0 off
the other tube there was traced from and refused as the two meeting at 0.19°. **Refused by name**:
along a path in a plane the ring's axis lies in, its top and bottom meridians are touched whole at
every instant (`Ring::touched_whole`, `root`'s closed form: `A` and `B` nought at one angle), and
the sweep of each is a sheet of its own — the groove's floor, 1.5 below the path, where the deeper
contact curve's tube reaches 1.098 — which no contact curve's tube holds: arranged from the tubes
alone the groove and the block beside it were one cell, and the block came back whole with two
sheets in it. A meridian touched whole at two of the closed class's poses (`K2_POSES`) or more is
`Curved` at its farthest point (`ring_entered`, before anything is fitted;
`swept_rings::a_ring_carried_along_a_spline_touching_its_meridians_whole_is_refused_by_name`); an
instant alone touching one whole (an end pose) is no sheet. **What builds it, measured**: along this path the contact
is exact — the two parallels `ψ₀(t)`, `ψ₀ + π` (`s cos ψ + ωR sin ψ = 0`) and the two meridians
`θ* = ±π/2` — and on a meridian `s''` changes sign exactly where a parallel crosses it, as it does on
the parallel there, without its sheet folding: the parallel is touching on one side of the crossing
right up to the meridian (`A`), and on the other a buried sliver runs out to a true fold `d_F` past it
(0.0017 to 0.012 of a turn over the roll), past which the far leaf `B'` lies over the meridian's
canal, crossing it at 0.03° and standing up to 5.8e-8 above it until `2 d_F` (`d_c = 2 d_F` to three
digits: the section is a parabola through the crossing). So the bridge's old ends lay on the canal
only to 1e-10 – 1e-8 and parted from it near the roll's ends; the crossing itself lies on it to the
rounding. **The touching side is the sheet** (`rings`' reader, #942 T1d-ring): of a bridge holding
the join where the label turns over, the piece on the side clear of the sliver's least is touching,
its rim on the meridian the crossing, and the sliver's piece alone is dipped (`narrow` now asks a
narrow pair's bridge to share its neighbours' branch). **The meridian's touching half is a sheet of
its own** (`rings::Strip`, `fitted`'s `strips`): the arc between the two tubes' crossings on the
side where its `s''` stands above nought, read at each roll (`Ring::touched_near`, the arc's roots a
hair off the meridian, as the tubes read their seams), fitted as a tube is, its sides the tubes'
seams within the fits
(`swept_rings::a_whole_touched_meridians_touching_half_is_a_sheet_between_the_tubes_seams`). Entered
(`rings::strip_entered`), its sides are the tubes' very seam curves, and what it meets is given by
construction: the ring at either end in its rim, each tube's touching face in the seam, its dipped
face in the seam and its far seam (the sliver's far side, on the strip within 2e-9), the leaf beyond
in the far seam, every other face of the tube nothing; a far seam leaving the strip across its rim is
cut there, and where it ends on the strip the dipped face's rim on the ring closes the strip's
corner. **Not yet**: arranged so, the bench's case fits in 11 s and arranges in 41 s, and the
groove and the block are still one cell, at 1e-5 as at 5.3e-8: at the end poses the ring where it
stops lies on both sides of a tube's rim beside the strip's corner (its face's loop walking the
touching rim, the dipped one and the leaf's out and back), where the ring, the strip, the tube's
touching side and its dipped one all touch one another, and a far seam's crossing of the strip's rim
3e-5 off the corner is taken for the corner by the reach of a meeting so shallow. The refusal stands
before anything is fitted. **A ring is the ring road's under any motion
K2 and the general road refuse it** (#942 T1d-ring, `closed::k2`): a tool of revolution clear of its
axis whose contact the road reads at the instants `rings::admits` asks is built on the ring road
from `closed`/`cut_from`, judged as every road is, its own refusal standing where it cannot build
it — the bench's ring along a spline, built by its runner through `cut_from`, ended at K2's
`Curved` at the torus's rim, and was refused by name at its top meridian until the depth trim
entered it (below); any other tool keeps K2's
refusal (`swept_patches::a_tool_no_convex_revolution_under_any_motion_is_refused_by_name`'s ring,
its two meridians square to the motion touched whole, is `Curved` so).

**A meridian touched whole for an instant, refused by name** (#880 T1f, `Ring::touched_once`):
on an arc the contact's two branches are `ψ = atan2(∓A, ±B)`, each one continuous function of the
angle round the axis and the roll wherever `A` and `B` are not both nought. Where they are, at one
angle `θ*` and one roll `t*`, the instant touches that meridian all round; on the contact surface
`A cos ψ + B sin ψ = 0` in `(θ, ψ, t)` the roll has two saddles at once, on that meridian half a
turn apart (forced: the surface is odd under `ψ → ψ + π`), and the two branches meet there and
exchange the meridian's halves — before it one tube's contact runs along one half and the other's
along the other, after it the other way about — no curve's kinds changing, so `changes` does not
see it. Over the angle and the roll each tube's sheet blows its point up into the whole meridian,
and no net over `(u, t)` holds it: brep-bench's crossed tumbling tori (`swept/tumbling-torus-crossed-
forward` at `t*` 1.3264280599, `θ*` 5.97116466; `-reverse` at 0.9003349192, 2.10298891) refined
the fit toward the instant without end. Found exactly — `A` vanishes at two angles, each followed
along the roll by continuity, and `B` there, as a part of the terms' scale, read at `SHAPE_LOOKS`
rolls and halved to the rounding where it changes sign clearly beyond `HALF_READ` (a meridian
touched whole at every instant reads `B` nought throughout and has none) — and refused before
anything is fitted, `Curved` at the meridian's outer point (`closed::ring`'s road and `admits`).
The meridian's touching half is the sweep's boundary (`depth::cover` reads it `Alone`), each branch
arriving on it touching on one side of `θ*` and leaving buried on the other (`s''` ±0.611 forward,
±0.470 reverse), a fold through the instant on each tube
(`swept_rings::a_meridian_touched_whole_for_an_instant_is_found_exactly_and_refused_by_name`).
**Not yet**: the instant built — the roll cut there, the cut's row four arcs (each tube's curve
and the meridian's two halves) meeting at the two saddles, the tubes' faces beside it read over a
chart the saddle does not blow up, and the fold and the double curve that run through it. **A
contact is followed from angle to angle by its label first** (`Ring::instant`): on an arc each
label holds one root at an angle, continuous where `A` and `B` are not both nought, so a contact
keeping its label is the same one, and its place round the meridian decides only where the label
changes. Followed by its place alone, the crossed tori's contact beside such a meridian — `ψ`
turning half a turn within a 256th of a turn of the angle — was taken for the other branch and the
instant refused (forward at 1.32689, reverse at 0.47042: the bench's `Curved` before the road
began), or read as one curve round twice; the reverse's near miss (`B` within 7e-4 of the scale
over 0.44–0.52, never through nought) reads the same changes with the observer's origin moved by an
ulp or a millionth (`swept_rings::the_crossed_tori_are_read_at_every_roll_by_their_contacts_labels`,
slow tier). The cases already built read alike: none touches a meridian whole for an instant. **Not
built either side of the instant yet**: the forward is built over the roll up to 72° (40 faces,
checked at the call's tolerance) and refused `Folded` at 74.5°, its rows 0.026 short of the instant
already bending too sharply; the reverse is refused before its near miss and before its first fold
pair is born (`Wrong` at a roll's end of 5°, a face's boundary through its own point at 6.5°,
`Folded` from 24°), refusals of the trim and the arrangement of their own: the meridian touched
whole (0.9003) lies outside those rolls. At 5° (one stretch, each tube in four faces, 12 s) the
arrangement finds three cells and keeps only the start pose's torus (a cell of 5 faces); the cell of
37 faces the tubes bound is classified out, and the judge's witness (2.4408, −1.0338, 2.1265) lies
0.058 deep in the sweep (the tool at roll −0.0707 holds it) and 2.95 from the solid kept.
**The collar about a meridian touched whole for an instant** (#880 T1f, `rings::Collar`): where the
Jacobian of `(A, B)` by `(θ, t)` is regular at the instant (forward det −1.06, reverse 0.37),
`(A, B)(θ, t) = ρ (−sin ψ, cos ψ)` is solved for `(θ, t)` by Newton's method given `(ψ, ρ)` and the
contact there is the arc's point at `ψ`: one smooth annulus holding both tubes and the meridian about
the instant (`ρ > 0` the branch `atan2(−A, B)`, `ρ < 0` the other, `ρ = 0` the meridian at `t*`, each
place `(θ, t)` met once from either branch, its `ψ` turning once round as the place goes round the
instant). It is cadrs's pinch chart (`pinch_at`). S, the surface the ring's core circle sweeps, is
singular there (`S_t = μ S_θ`, μ 1.929 and 1.122) and a Whitney umbrella (`det(S_θ, g_uv, g_vv)` over
its norms −0.278 and 0.412, its double line leaving the pinch toward a saddle's `ψ`). **Measured on it
(`ring_profile collar`, and independently by the least over the roll of the core circle's
closed-form distance), the boundary is one strip across the meridian**: on the meridian the arc
between `ψ` 257.127° and 72.883° forward (175.333° to 337.210° reverse) where the far cover passes the
tolerance below nought; each end a double curve's preimage crossing the meridian square-on, its `ψ`
within a degree over `|ρ| ≤ 0.4`, covered beyond by an instant 0.06–0.4 of the roll away, the fold
following it 2e-3 to 4e-3 further on and buried. Each double curve's two preimages are both on the
collar, one curve through a swallowtail point off the meridian, where its partner's instant passes
its own (forward `ρ` ≈ +0.07 and +0.09, the roll ≈ t* ± 0.03; reverse ≈ −0.22 and −0.135), the fold
tangent to it there alone. So nothing of the boundary's structure stands at the event: the
meridian's touching half, the two saddles and the instant lie inside one smooth piece of the collar,
and the "fold and double curve through `(θ*, t*)`" a tube's `(θ, t)` shows are these two curves
crossing the meridian, which that chart blows down to the point
(`swept_rings::the_collar_about_a_meridian_touched_whole_charts_both_tubes_and_holds_one_strip_of_boundary`).
**The face structure it asks** (not yet built): the collar a candidate sheet over
`(ψ, ρ) ∈ [0, 2π) × [−ρ₀, ρ₀]`, `ρ₀` clear of the swallowtail points, its rims `ρ = ±ρ₀` closed
curves inside one stretch's tubes, holes in them; the strip cut from it by its two double curves,
each laid as a swallowtail's is and running out across the rims into the tubes' own; vertices at
those crossings and at the swallowtail points; the meridian no edge.
**The tubes fitted through the instant with a hole** (#880 T1f, `rings::Hole`,
`closed::ring_fitted_holed`; not yet the road's, which refuses the instant by name until the collar
fills the hole): the roll is cut at `t*` — the pieces' kinds do not change there, but the bridges'
ends, read from the sign of `s''` along the curve, which the branches exchange, jump across it (tube
`−`'s bridge [5.4478, 6.3607] → [5.6925, 5.9700], tube `+`'s touching end 5.9723 → 5.5436, forward)
— and each tube's net over `(u, t)` is fitted over its whole rectangle, read inside the hole
`|(A, B)| < ρ₀` about the instant (its reach the rim's box doubled, so a near miss elsewhere is no
hole) as its stand-in `c + r (B, −A)/h(ρ)`, `h² = ρ² + (ρ₀/2)² β(2ρ/ρ₀)`, `β(s) = e^(1 − 1/(1 − s²))`:
smooth through the instant, the contact itself from `ρ₀/2` out, so the net fits the very sheet
across the rim the trim is to cut it along; its folds are not read in the hole. **A pair's end
through the swing**: at the instant `s''` is odd about the meridian (±0.611 forward), and tube
`+`'s pair has its least 0.044 of a turn past the meridian at a depth (0.614) all but that `s''`
beside it, so its end, where `s''` climbs back to the depth, was found from roll to roll at the
swing (5.9719) or along the flat stretch beside it (5.9282, 5.9025), and the fit refined to 1e-11
of the roll there. Within the hole an end is pulled onto the swing — the row's least `|(A, B)|`
(`Hole::least`), one smooth line through the instant — by the smooth step's weight: wholly over the
hole's inner half and nothing at its rim, wholly where it stands beyond the swing from the least and
nothing from halfway to the least on, and only where the least stands clear of the swing by twice
its width (tube `−`'s least lies in the swing, and its ends are its own); no decision on a
threshold, each weight `C^∞`. **A pair born or vanishing at a cut is read unborn up to its
swallowtail point before the stretch is fitted** (`rings::pairs_over`, on a road with a hole): at
the cut the pair has no width and the sheet goes as the square root of the roll from it, so the
stretch beside the cut fits only by luck — the forward's `Folded` at a roll's end of 74.5° was tube
`+`'s pair born at 1.2240, its rows refined to a millionth of the roll and flipped there, not the
instant. Its point is the earliest (latest) on the curve whose pairs change in number, matched by
the curve's labels, the pair dropped by its label (`Apex::label`): forward tube `+`'s at (6.0508,
1.28159), `ρ` 0.087, and (5.9103, 1.35616), `ρ` 0.071 — the predecessor's two. In a stretch with a
hole the gauge is read at the fit's first rows and not at a fold in the hole: tube `−`'s fold runs
into the instant, and its change of sign 2.3e-4 short of it hid tube `+`'s point between the
stretch's ends. **Measured, ρ₀ = 0.03 forward**: fitted from −100° through 1.28159 to `t*` (the
stretch holding tube `+`'s pair from its swallowtail point 219 rows); refused `Folded` in
[`t*`, 1.38326] at its last rows: tube `−`'s pair vanishing at 1.3833 has no swallowtail point
outside the hole in that stretch — its fold is the one through the instant — so nothing yet reads it
unborn toward the cut. Not yet: that, `ρ₀`'s rule, the rim as a chord and the hole's piece dropped
by construction, the collar's net and its trim.

**The tube road: a torus's sweep as the offset of its core circle's sweep** (#880 T1f,
`swept::tube`; it builds every ring case the old road builds, not yet called by `closed::ring`): a
torus is the set of points within its tube `r` of its core circle, so its sweep's boundary lies on
the offset at `r` of the surface `S(θ, t)` the circle sweeps, its two sheets `X± = S ± r N`
(`Tube::point`, `S`'s rates in closed form but `S_tt`, one central difference) each one map over
the whole rectangle wherever `S` is regular, plus the end poses' tori (`Chart::End`, over `(θ,
ψ)`). A sheet's fold is a cuspidal edge of the map, the zero set of its Jacobian against the
spine's `J = 1 − 2rH + r²K`
(`tube::edges`, its metric the spine's), and `J`'s sign is the far cover's: `J ≤ 0` is `Near`
(the half turn, every point of a 32 × 15 grid on both sheets,
`swept_tube::the_half_turns_sheets_are_maps_with_one_cuspidal_edge_and_the_ring_roads_swallowtail_point`).
An end pose's own edge is its contact circles, where `n · v` passes nought (`Tube::side`). The
swallowtail points are where the sheet's kernel runs along its cuspidal edge (`tube::swallowtails`,
Newton on `(J, ∇J · k)`): the half turn's one, on the sheet `−`, is the ring road's apex to 1e-6.
**Measured on the half turn** (`tube_profile half-turn`): each sheet fitted as a map by
`nurbs::fit_net` (quintic, no fold check) to 6.3e-8 against a bar of 3.2e-7, 257 × 65 poles, in
0.15 s; the sheet `+` has no cuspidal edge, the sheet `−` one, a sliver from the roll's end across
`θ ≡ 0` (from `θ` 6.2518 to 0.1386 at `t = b`, down to 1.4839 at the swallowtail point
(0.0656162, 1.4839292)); each end pose's contact circles traced in 0.3 s. The far cover's depth
over a chart (`tube::Depth`, `tube::seams`) is traced by `zero_set::trace` over the rectangle cut in
two across each whole turn (`tube::round`: the search halves a cell across its longer way read
between its sides' ends, and over a whole turn those are one point, so it halved one way only and
never settled; `zero_set` now halves such a cell across the turn, below). Its bound reads the cell's
middle alone: each instant's depth is 1-Lipschitz in space, so where the same basins of the roll
cover the cell the far cover's depth lies within the cell's reach of its middle's — one certified
cover a cell where nine were read, the half turn's sheet `−` traced in 13.6 s, not 32.7, and the
half turn built in 19 s, not 39. Where nothing else covers the middle (`Alone`) the nine are read
and a cell whose nine are all `Alone` is read clear: sampled evidence, not proof. **What the
trim's completeness is:** a curve through a point found exactly — a swallowtail point (its disc's
rim), a crossing of a rim or a cut, a corner, an end pose's contact circle — is certain; a closed
double curve touching none of them is found where the search does not clear its cell, so one born
and dying in a region nothing else covers, between a cell's nine points, is missed, and one in a
covered region is found wherever its basins hold over the cell. What remains of the time is the
march: `zero_set` reads the certified least at each step and chord's quarters (a third instant
undercutting the branch, a triple point), and `depth::cover`'s certified march creeps where the far
depth is near nought — all along a seam — its steps the larger of the depth and the tolerance over
the point's speed (some 2.7 ms a cover on the sheet `−`, 0.36 ms on the sheet `+`). Not yet: the
least read only at checkpoints with a switch bisected where one is found, or a second-order step
(the depth's rate in the roll bounded) in the cover's march. **`zero_set`'s whole turn** (#880 T1f):
a cell's longer way is read along its path through its middle (both half-chords), so a cell spanning
a whole turn, whose sides' ends are one point, is halved across it
(`zero_set::a_cell_spanning_a_whole_turn_is_halved_across_it`, a loop on a cylinder: without it
the search never settles); `tube::round`'s halves stay, for its seeds across the cuts. `Depth::along`, continuing a cover held at an end of the
roll, reads that end too: past the point's own contact `d` is no single basin over the stretch, and
the golden section settled on the climb, 7e-6 off. A piece leaving one part across a cut seeds
the part beside it (`round`): beside a seam the far cover may be read on one side only, so the cut's
own sign changes missed both curves' continuations across `θ ≡ 0`. **About a swallowtail point a
disc** (`tube::disc`, `Disc`) is cut from the depth's domain, its rim an inner edge, its radius (in
the spine's metric) halved from the feature until every far cover read on its rim is within the
tolerance: there the far cover is all but invisible (points read `Alone`, the few `Far` covered
1e-9 deep), the double curve ends tangent to the cuspidal edge where the sheet's metric is singular,
and the march stalled 5e-4 from the point. Measured (`tube_profile half-turn … trace`, gated in the
slow tier by `swept_tube::the_half_turns_seams_are_traced_from_the_depth_on_every_chart`): no seam
on the sheet `+` nor on the start pose; on the sheet `−` a disc of 0.01 and the double curve's two
preimages, one from the disc's rim across `θ ≡ 0` (at `t` 1.52384) to the roll's end at `θ`
6.22418, covered at τ 1.4626–1.4726, the other from the disc's rim to (0.13929, 1.56981), its cover
passing to the end pose; on the end pose one seam from its contact circle at `θ` 6.22420 across
`θ ≡ 0` to its contact circle at 0.13633. The two charts' crossings of their shared rim are one
crossing read twice, 1.64e-5 apart (five tolerances: the end pose touches the sheet along that rim,
so a crossing of it is ill-conditioned along it) — such a vertex is to be found once, on the rim.
**The faces sewn** (`tube::faces::solid`): each sheet's zero set is read as runs by what covers
it — a double curve of the sheets is traced on both its preimages and kept on the later one (its
own instant past its cover's), its partner read off the cover and settled on the sheet; a double
curve with an end pose is traced on the sheet alone, the end pose's seam its image — and every end
of a run is an event found once, one vertex of everything meeting there: a **rim crossing**,
solved on the rim as the root along it of the far cover's depth (one curve read in two charts:
`Tube::on_end` gives the end pose's place in closed form), its partner on the covering instant the
**switch** where the partner preimage's cover passes to the end pose; a **swallowtail point**, the
run joined to it from its disc's rim along the normal form by the ring road's rule
(`rings::joined_by_normal_form`, lifted out of `rings::joined`, which calls it); a **corner**, where
the cuspidal edge reaches the roll's end (`Tube::corner`, `J`'s root along the rim, (0.1386336,
`b`) on the half turn), the run joined to it straight in the sheet's parameters, every point of the
join within the tolerance of the end pose (about it every far cover is within the rounding, and the
trace stops short). Each run and join is fitted as a curve with both pcurves on one parameter, a
traced run's points settled on the double curve (`X₁(u₁) = X₂(u₂)`), the joins held to the
tolerance alone on their partner. The charts are cut across their angle at two angles clear of
every trim (the windows) and from each swallowtail point to the rim it opens away from (so no face
meets itself along its double curve); each window is walked as a planar graph whose nodes are a
vertex at each of its places on the chart (a double curve's point has two on one sheet), and each
part kept where its inner point bounds the sweep; a sheet's face lies on a net fitted over its
window, an end pose's on its torus; sewn by `sew::sewn`. **The half turn builds**: 9 faces, `check`
at the call's tolerance, volume 12.2377154879 against the old road's 12.2377154283 (4.9e-9 of
itself), its mesh's volume within its bar, and 19 119 of 20 000 seeded points read clear of the
boundary each a member of the solid exactly where the depth is below nought, in some 40 s, nearly
all of it the sheet `−`'s trace (`swept_tube::the_half_turn_builds_by_the_tube_road`, slow tier).
**Each edge's bar:** a traced run, a rim and a cut are fitted to a tenth of the tolerance on both
their charts (their curve within it of each chart's point through its pcurve); a join's partner
pcurve is held to the tolerance itself. That is sound because the join is what the two charts agree
to only to the tolerance: inside a swallowtail point's disc every far cover is within the tolerance
(the disc's own definition), so the double curve's two leaves stand within it of one another and
no reading places the curve closer; the corner's join is checked within the tolerance of the end
pose at every chord's point and middle. The sewing measures each edge's gap as its tolerance, and
`check` allows it. On the half turn the normal-form join reads 2.29e-6 (of 3.2e-6), the corner's
straight join 7.0e-9, the windows' cuts 1.06e-7 (the nets' fit), every other edge under 6.1e-8;
`Brep::misfits` lists nothing at the call's tolerance nor at the size's (`geom::tolerance`, 7.9e-9
over its diagonal): each edge's own measured tolerance is what `check` holds it to.
**The two end poses' meeting** is the depth's zero set on the start pose where the cover is
the end pose, as every other seam — `seams` on `Chart::End(0)` reads it, the certified cover
marching the whole roll to its far end — read as trims between the poses (below, `faces::met`).
`closed::ring` calls the old road, and the tube road where the old refuses a meridian touched whole for an instant (below, the crossed pair). **The next cases, as checks:** the crossed reverse
member with its roll's end at 5° (no pinch; the old road misclassifies a cell and refuses `Wrong`)
builds in 0.4 s — 8 faces, `check`, volume 1.5608532821, no volume known apart, held to its mesh and
to membership by the depth at 18 988 of 20 000 points
(`swept_tube::the_crossed_reverse_member_at_five_degrees_builds_by_the_tube_road`). The 80° torus,
the thin and the full turn are not yet built in a time worth a road: the 80° torus's sheet `−` has
fold pairs running the whole roll and no swallowtail point, and its depth's trace had not ended in
twenty minutes; the full turn's sheet `+` alone asked 57 000 sampled cells of its edges' search
(8 s) before its depth's; the thin's edges had not ended — each the cover's creeping march
and the march's least at every step (above).
**The tube's own cover** (#880 T1f, `swept::tube::cover`, `Tube::cover`): at the roll `τ` a
point stands at `q(τ) = M(τ)⁻¹ x` in the tool's frame and the torus's depth there is
`d = g(q) − r`, `g` the distance from the core circle, so `d'' = ∇g · q'' + q'ᵀ H q'` and, `H`'s
eigenvalues in the ring's cylindrical frame being `0`, `1/g` and `g_ρ/ρ`,
`|d''| ≤ A + V² max(1/g, 1/ρ)` — `V` and `A` the motion's bounds on `q`'s speed and acceleration
(`Trajectory::point_jet_bound_over`: a constant twist's `|ratio|ᵏ ρ`, a relative motion's by
Leibniz's rule through its second node's spin, `spin_bound`; gated by
`swept_tube::the_motions_rate_bounds_hold`), `g` and `ρ` their least over the stretch read, the
speed local (`|q'| + A w`). The point's own contact is marched to its top by steps over which the
bound keeps `d` above the greatest read less the tolerance (the steps no feature size caps), the
least past each top found best-first over halved stretches, each floored by
`d(m) − |d'(m)| w − M w²/2` or the Lipschitz floor, until every floor stands within the tolerance
of the least read, then settled by the secant (Illinois) on `d'` in closed form. It reads as
`depth::cover` does to the tolerance (`swept_tube::the_tubes_cover_reads_as_the_depths`), but where
the sampled march stepped over a fall past the own contact's top it reads the far cover (above
nought) the march read `Alone`; some 80 reads and 40 µs a cover against 0.5 ms (2.7 ms along a
seam). Where the motion gives no bound (a pace, a path evaluated) `depth::cover` is read.
**The charts' sides proved over cells** (`Tube::jacobian_bound`, `Tube::contact_bound`, the bounds
of `tube::edges`'s search, gated by `swept_tube::the_charts_sides_are_bounded_over_cells`): on a
sheet `J = det Q / w⁴`, `Q = w I − s r II'` (`n = S_θ × S_t`, `w = |n|`, `II'` read against `n`),
read at the cell's middle in the parameters scaled by `diag(1/R, 1/|S_t|)`; each entry's move over
the cell is bounded by its rates from the circle's (`Tube::rates`: `|S_θ…| = R`, `|S_θt| ≤ ΩR`,
`|S_θtt| ≤ Ω²R`, `|S_t|, |S_tt|, |S_ttt|` the circle's speed, acceleration and jerk, each from the
middle's own), and `Q`'s eigenvalues by that move's Frobenius norm (Weyl): definite proves `J > 0`,
indefinite `J < 0`; or, where the tube is thin beside the spine's bending, `λ_min(I) ≥ W/(E + G)`
standing above `r |II'|/w`. The middle's `S_tt`, a central difference, is off by its truncation
`h² J₄/6` and its rounding, both counted. An end pose's side `± n · v` moves by at most
`(|v| + Ω(R + r)) Δθ + (|v| + Ω r) Δψ`. What no bound proves is sampled as before. **Measured**
(`tube_profile … build`): the half turn builds in 3.9 s where it took 28 s (on this machine; 19 s
on the last), 9 faces, volume 12.2377154879 to the digit, its edges' search 0.7 and 1.9 s where it
sampled 29 000 cells a sheet for 3 and 4.5 s; the 80° torus (`tumbling(80)`) builds in 2.2 s,
8 faces, volume 10.9393141712 against the old road's 10.9393141414 (2.7e-9 of itself).
**Read beyond the half turn** (each change keeping it at 12.2377154879): the depth's bound has a
top where the middle is covered deeper than the reach (its covering instant holds the whole cell,
so it lies past each point's own contact), so a rim covered for a stretch is cleared, where the
full turn's rim at its start was halved to the tolerance and every leaf of it touched (some
32 000 leaves, 64 covers each); a swallowtail point every far cover about whose feature's disc is
covered deeper than the tolerance is held inside by another instant and given no disc; a disc's
rim counts a cover clear above nought only from an instant inside the disc's span of the roll (an
end pose's, far off, is another basin's: the full turn's second swallowtail point, read 0.85 above
by the start pose, had no disc); and two runs' ends on one cut are one point where they stand within
the tolerance in space (each the root along the cut its own part's trace read: 2e-6 apart in `t` on
the full turn). **Every ring case the old road builds, built** (#880 T1f; each change keeping the half turn at
12.2377154879 to 1e-10, the 80° torus at 10.9393141712, and the half turn's time):
* **A far cover continued into the point's own contact ends** (`Depth::along`; gated by
  `swept_tube::a_far_cover_continued_into_the_points_own_contact_ends`): its stretch is cut half
  the way toward the point's own instant, and a least found at that cut, to the rounding of the
  roll, is the own contact's slope — `d` falling there toward the own instant — no basin of another
  instant: the double curve's two instants have met (its end, beside a swallowtail point), and the
  cover ends. Read on, each corrector's pass halved the way again and the cover slid into the own
  contact, nought everywhere on the sheet: on the full turn's sheet `−` a phantom branch ran along
  `t` 2.24599 to the cut `θ = π`, where the depth is 1.28. It costs nothing: the half turn's ordinary
  branches never settle at that cut (reading the climb between the instants instead slowed it
  fivefold).
* **A point is traced where it is so in the chart** (`zero_set`'s near test; gated by
  `zero_set::a_loop_whose_image_crosses_a_traced_curve_is_found`): a seed, or a search cell's
  middle, is answered as on a traced curve where a traced chord stands within the radius of it in
  space *and*, by the metric at the point, in the parameters. A sheet meets itself along its double
  curve, so another place of the chart comes to the same point: on the full turn the preimage
  seeded across the cut `θ ≡ 0` stood in space on its other preimage, traced already in that part,
  and was never traced.
* **A switch solved where it falls** (`faces::switched`): where a run's cover passes from the sheet
  `B`'s point at an instant inside the roll to the end pose `k`, the event is where `B`'s preimage
  reaches its rim at `k` — a rim crossing on `B`. Where the run covered by `B` is the one kept (its
  own instant the later), `B`'s run is not, and the crossing is solved there, on `B`'s rim, from
  `B`'s point covered at the run's own instant; else it is `B`'s own run's rim crossing, read once
  every run's ends are (the half turn's). Both runs beside a switch end at its one vertex.
* **The two end poses' meeting** (`faces::met`, the old road's `depth::ends_met`): the start
  pose's seams (`seams` on `Chart::End(0)`, its contact circles inner edges) where the end pose
  covers it are trims of the start pose against the end pose, the partner of each point the end
  pose's place in closed form (`Tube::on_end`). Each end of such a run is an event already found, a
  vertex standing on both poses: on the start pose's contact circle the sheet's rim crossing at
  the start covered by the end pose; where the start pose's cover passes to the end pose from the
  instants before it, the sheet's rim crossing at the end covered by the start pose — the nearest
  such vertex, held within the trace's longest step of the run's end, its parameters put there.
  The full turn's are two short arcs, as the old road's; tracing the start pose costs the half turn
  some 0.4 s (it builds in 4.3 s, the old road in 6.6; the 80° torus in 2.3 against 6.2).
* **The cut from a swallowtail point ends at the first trim it meets** (`faces::met_by_cut`): it
  runs at the point's angle toward the rim the swallowtail opens away from, and a trim's polyline
  crossed before the rim (its own or its partner's on the chart) is met there — the point settled
  on the double curve with its angle held, a vertex of both its charts, and the trim split there
  into two. On the full turn's sheet `−` the cut from the second swallowtail point ran to the start
  and crossed a double curve with the end pose with no vertex there, and one face used a trim on
  both its sides.
* **A window's island is a hole** (`faces::faces`): a loop of a window's graph turning clockwise is
  the outside of its part of the graph — the window's own, or, where it meets no node of a loop
  turning counter-clockwise and stands inside it, an island in that part, a hole of the least
  such; a face is its outer loop and its holes, its inner point read clear of them (the full turn's
  sheet `+` holds a lens bounded by a double curve with the sheet `−` and one with the end pose).
* **Each chart its own windows** (`faces::windows`): the cuts split the largest gap of angle the
  trims' uses *on that chart* leave free, in order within the turn; a sheet's rim is split at its
  own cuts and its end pose's (one vertex where they coincide). On the bench's tumbling torus the
  sheet `−`'s double curves with the end pose run near its rim over 2.5 of a turn's angle, their
  partners on the end pose over the rest: no angle was free on every chart at once.
* **A shallow crossing of a side read twice is no piece** (`zero_set`; gated by
  `zero_set::a_rim_crossed_at_two_readings_of_one_place_is_no_piece`): a piece every point of which
  lies on the one side it started on (on an inner edge, of two points) is that side's crossing read
  twice. The far cover's depth is read to the cover's tolerance, and where the zero set crosses a
  rim shallowly that changes sign twice about the one root: on the thin's sheet `−` two rim seeds
  2.5 tolerances apart, the march from one arriving at the other, a piece along the rim the run
  joined and doubled back on past its crossing (the misfit of 3.3e-7 against 3e-8 at its start).
  Neither settling the trace's first point with the crossing nor holding that end to the tolerance
  is true to it: the curve meets the rim once, at the crossing, which replaces the trace's own
  reading of it.
* **A curve run to the edge of its field's domain ends there** (`zero_set::End::Undefined`; gated
  by `zero_set::a_curve_running_to_where_its_field_is_undefined_ends_there`): a march that can
  step no farther — its step halved under the rounding — and finds the field undefined within a
  tolerance of where it stands (ahead, or across its curve either way) ends there, where it was
  refused as stalled. On the bench's tumbling torus the sheet `−`'s double curve with the end pose
  runs into a corner of the cuspidal edge and the roll's end where the two stand within 1e-12 of
  one another: there its zero set is the very edge of where the end pose is a cover past the point's
  own contact (beyond, its basin has merged into the own contact and nothing covers the point), so
  no chord keeps inside the field; the run ends there and is joined straight to the corner, every
  point of the join within the tolerance of the end pose (`cornered`, the half turn's).

**Measured** (`tube_profile … build` against `… old`, on a quiet machine): the full turn builds in
25.8 s where the old road takes 36.6 (14 faces, volume 23.6738858153 against the old road's
23.6738852805, 2.3e-8 of itself), the thin in 63.0 s against 45.7 (10 faces, 0.6963027945 against
0.6963027896, 6.5e-9; the sheet `−`'s seams 33 s of it, its edges' search most of the rest, where
`jacobian_bound` is weak with `S_θ` and `S_t` near parallel), the bench's tumbling torus in 9.5 s
against 5.9 (8 faces, 25.2750727786 against the volume made apart, 25.2750727653 —
`swept_rings::the_tumbling_torus_as_the_bench_spells_it_is_built_closed` — 5e-10; the old road at
this spelling of the motion reads 25.2750392926, 1.3e-6 below, and its solid's faces carry surface
covered by other instants: nine points of its mesh at 0.003 covered 1e-5 to 3.4e-3 deep, on three
faces, the tube road's none: gated in `swept_tube::the_tumbling_torus_builds_by_the_tube_road`). Each passes `check` at the call's tolerance with no misfit at the
size's tolerance, and each is held to its mesh and to membership by the depth in the slow tier
(`swept_tube::the_full_turn_builds_by_the_tube_road`, `…the_thin_…` at 160 000 points, a tube of
0.015 filling a part in a hundred of its box, `…the_tumbling_torus_…`). **Read again at moved inputs**
(the observer's origin moved by an ulp and by 1e-6): every case builds by both roads with the same
faces; at the ulp the tube road's volumes are the same to the digit, at 1e-6 both roads' move
alike (the tumbling torus by 1.07e-5 each), and the tube road's misfits stay none where the old
road's are 10 to 43 at the size's tolerance. **Not yet answered:** on the half turn and the thin
both roads' faces reach past a double curve beside a swallowtail point — mesh points 1.5e-5 to
7e-5 past an edge whose own points stand on the depth's zero set (to 6e-11), covered 0.6e-6 to
1.4e-5 deep (the tube road's one on the half turn at 1.35e-5, seven on the thin; the old road's six
and eight, to 1.2e-4), where the sheet's map is near singular — and on the thin's end pose one
point 3.7e-4 inside its contact circle held 2.5e-6 deep by the instants beside its own; the full turn, the 80° torus and
the tumbling torus have none on the tube road. **What `closed::ring`
would need to call it first** (not switched): the tool a torus (`Tube::new`, else the old road);
no meridian touched whole over the roll until the collar chart is built (`Ring::touched_once`; the
tube road has no pinch); the motion's rate bounds given (`Tube::rates`, else every cover is
`depth::cover`'s and every edge's cell sampled, at a time unmeasured); any refusal by name read as
a fall back to the old road, never a wrong solid; and its time no worse — today the thin (63 s
against 45.7) and the tumbling torus (9.5 against 5.9) are slower, the sheet `−`'s seams and the
edges' search where `S_θ` and `S_t` near parallel weaken `jacobian_bound`. Gates pinned to the old
road's digits through `closed::ring` move with it: the tumbling torus by 1.3e-6, toward the volume
made apart. **The old march's step
over a shallow fall** (asked of the tube's cover, `swept_tube::the_tubes_cover_reads_as_the_depths`):
on the half turn's sheet `+` at (θ = 2.5π/12, t = −0.9163) the own contact climbs to 2.2120598518 at τ
1.568359 and falls 2.96e-6 — under the tolerance, over the rounding — to the end pose's
2.2120568897: `depth::cover` steps from below the top to the roll's end and reads `Alone`, the
tube's cover the far cover at the end. Within `depth::cover`'s contract (no stretch held deeper
than the tolerance is stepped over, so the sign of a cover is never misread), not #998's fall
(which read a covered point `Alone`); the old road's `covered`, `ends_met` and `passed_back` decide
on that sign alone.
**The collar as the tube road's third chart** (#880 T1f, `Tube::pinched`, `Chart::Collar`,
`tube::Pinch`; gated by `swept_tube::the_collar_charts_the_crossed_members_pinch`): where an
instant inside the roll touches a meridian whole (`Ring::touched_once`) the spine is singular —
`n = S_θ × S_t = R (B e_r − A e_z)`, so the sheets' normal is `(B e_r − A e_z)/|(A, B)|` and both
sheets turn half round the meridian across `(θ*, t*)` — and the collar `(ψ, ρ)`,
`(A, B)(θ, t) = ρ (−sin ψ, cos ψ)`, charts them as one annulus: `ρ > 0` the sheet `+`, `ρ < 0` the
sheet `−` (each collar point the sheet's point at its place to 1e-15), `ρ = 0` the meridian. Its
point is the tool's `c + r (cos ψ, sin ψ)` at the place posed, its rates by the chain rule through
the place's (`D ∂(θ, t)/∂ψ = ρ (−cos ψ, −sin ψ)`, `D ∂(θ, t)/∂ρ = (−sin ψ, cos ψ)`, `D` the
Jacobian of `(A, B)`); its side `−sgn(D) (X_ψ × X_ρ) · n` over the rates' lengths has the sheets'
`J`'s sign at every place and is smooth through the meridian, where `J` is not read; its cuspidal
edges are traced on a regular metric (the tool's own way round its tube posed and the rate in `ρ`:
the collar's own rates are parallel on its edges, and the march stalled there).
**`ρ₀` by its rule**: the collar's reach is the terms' scale halved until its rim is read whole
(beyond it `(A, B)` folds over the roll: the reverse's rim is not read all round past 0.135);
within it the collar's cuspidal edges are traced and their swallowtail points read
(`swallowtails_on`), and `ρ₀` is half the least `|ρ|` of those and of the reach, so the collar
holds no swallowtail point and its rim stands at most halfway out to each. Forward: the reach
0.1742 holds the two at `ρ` 0.0710 and 0.087, so `ρ₀` 0.0355; reverse: the reach 0.1091 holds
none (its nearest, −0.135, lies past it), so 0.0545. **The hole**: the sheets' points with
`|(A, B)| < ρ₀` about the instant (within the rim's box doubled) are the collar's; its rim is one
curve read in three charts (the collar's `ρ = ±ρ₀`, each sheet's `place(ψ, ±ρ₀)`), an inner edge of
the sheets' edges and depth, which read on across it to `ρ₀/2` (`Pinch::deep`) so the tracer's
exit settles on the field; inside, the sheet is filled by a smooth stand-in, `S + s r (B e_r −
A e_z)/h`, `h² = ρ² + (ρ₀/2)² β(2ρ/ρ₀)` (`Tube::net_point`, the old road's `Hole::stand_in` on the
tube), which is also the depth's metric there: the sheet's own map is a cone point at the pinch,
and a cell holding it kept a reach of the tube's size however halved (the forward's search asked
four million cells). **The sheets' nets about the hole**: a uniform net over a window's whole
roll holds its bar outside the hole only from `ρ₀` 0.14 forward and 0.12 reverse (at 1024 × 512 and
3.5 s a sheet) — past the forward's swallowtail points and at the reverse's reach — so a window
holding the hole is fitted with its parameters graded toward where it misses (`faces::sheet_net`,
each interval over the bar halved, neighbours within a factor two), read against the sheet
outside the hole: forward 241 × 230 poles in 1.3 s. **Measured on the collar**: the reverse's
depth gives two trims rim to rim at `ψ` 175.5° and 337.9° (the meridian's 175.333° and
337.210°), the sheet `+`'s runs ending on the hole's rim at the same own instants (0.867, 0.970);
the forward's gives its trim at 72.83°, its other (257.1°) and the sheets' runs into the hole not
yet found: there each double curve runs within 0.6° of the cuspidal edge it folds beside (a
covered sliver 2.3e-3 across, at most 6.5e-5 deep), between the rim's samples; the collar's rim is
read beside each edge's end (`beside_edges`, evidence). **The faces' side** (`faces::solid`, in place, not yet
building either member): the collar is traced first and its runs' ends on the rim seed the sheets';
every run's end on the rim, kept or not, is one vertex in three charts (`faces::rim_vertex`,
solved on the rim as the root in `ψ` of the far cover's depth on the collar, `hole_crossing`), so a
kept trim whose partner passes into the collar switches there; a cover whose place lies in the hole
is the collar's (`Tube::in_collar`, `ψ` in closed form from the tool's point, `ρ` square to
`(A, B)`, settled); the collar's windows are cut in `ψ` clear of its trims and the sheets' clear of
the hole too (`chart_cuts`); the rim is an edge on each sign between its vertices (`Kind::Hole`,
the collar's use and the sheet's). **The reverse's next refusal**: a run of the sheet `+` ends at
the cut `θ = 2π` (t 1.4597, covered at −0.072) with no piece across it: at `θ = 0` its seed lies
within the tolerance in space of the run from (0, 0.6988) (covered at 1.4613, own 0.699), a triple
point of the sheet with itself, and `zero_set`'s duplicate test (`near_traced`) read in space alone,
so it dropped a second preimage there (`zero_set::through` from the same seed traces it to the
roll's end at (0.9046, b)); it now reads the parameters too, by the metric (the tube road's sibling's
change, merged), and the sheet `+` traces 7 pieces. The build
then stops at the sheet `−`'s stall below. **Not yet**: three readings off the pinch — the reverse's near miss of a second whole-touch
(`|(A, B)|` 6.3e-4, 3.6e-4 of the terms' scale, at (1.0764, 0.4611), no root, where the sheets fold
through within a few ten-thousandths and a graded net asks 866 × 855 poles), and the sheet `−`'s
stalls of each member (forward at (2.9903, 0.6355), 0.02 from its swallowtail point (2.9730,
0.6227), where the far cover is born within the tolerance of nought; reverse at (1.4390, 0.1605)
on a cuspidal edge with nothing covering either side).

**The crossed pair built, and `closed::ring` dispatching to it** (#880 T1f; gated by
`swept_tube::the_crossed_forward_member_builds_by_the_tube_road` and
`swept_tube::the_crossed_reverse_member_builds_by_the_tube_road`, both slow): where the old road
refuses `Curved` at a meridian touched whole for an instant (`Ring::touched_once`) and no two
instants touch one, `closed::ring` builds the tube road's solid (`Tube::pinched`,
`tube::faces::solid`); nowhere else, and cut from a blank (`ring_cut_from`) it stays refused. The
forward: 14 faces, volume 16.6109998153 against cadrs's 16.6109997999, about 15 s; the reverse: 15
faces, 13.7898471917 against 13.7898472784, about a minute, most of it the two graded nets about
its near miss of a second whole-touch. Each is held to `check`, no misfit at the size's tolerance,
membership by the depth at 20 000 points, none of its mesh's points covered deeper than the
tolerance, its boundary audited by the depth (`audited`: each point of its mesh at 1e-3 stepped
`δ` out along its face's normal stands outside by the depth, stepped in, inside, at `δ` 2e-5 and
twice the tolerance, none apart), and built again with the observer's origin one ulp and 1e-6 off
to the same volume (forward 16.6109998154 and 16.6110018019; reverse 13.7898471715 and
13.7898510440). **The forward's 3.8e-5 was the volume's reading, not the solid**: no point of its
boundary stood under another instant, and its mesh's volume, extrapolated to no sag, read
16.6109994 — but the face holding the pinch's hole read its flux 6.6e-4 off at any bar. `G`, read
along a use by Green's theorem, is smooth only between the use's crossings of its net's knot lines,
and the graded net crowds hundreds of them across one stretch of a trim: each use on a net is now cut
at its crossings too (`props::crossings`). An audit by a mesh's membership (`member::Solid`, its
sag the bar's) cannot read a step below its bar: at 2e-5 against a mesh at 1e-4 it read points
apart both ways where the solid was right. **The reverse one ulp off**: its trim against the end
pose had stopped at the cuspidal edge 6.9e-4 short of the corner and been joined straight; one ulp
off it ran on to the rim 2.2e-5 beside the corner and fitted only to 1.2e-5. Beside the rim the end
pose stands from the sheet's point by half its depth's second rate in the instant times the square
of the instant's way to the end, nought only on the cuspidal edge: a run against the end pose `k`
reaches the rim at `k` at the corner alone, and about it the two touch, so its points there stand on
the double curve to the rounding only. Such a run's tail is the straight join to the corner
(`cornered`, `join_holds`), from the farthest of its points the join holds the end pose from.
**The forward one ulp off**: the collar's trim began with the tracer's seed on the pinch's rim and
its exit settled on the rim 7e-15 from it, one crossing read twice; the walk read the way out of the
vertex from that segment, lost the collar's face and was not sewn. A trim does not run along the
rim: of a run's points on the rim at an end only the end is kept. **The pair's Booleans** (union,
difference and intersection of the two members, the ordinary `boolean::boolean`; probe:
`tube_profile crossed-forward pair`) **are not yet built**: each refuses after about 13 minutes,
"a bspline has a hole in no piece" (`boolean.rs`, a face's hole held by no outer cycle once split
— most likely the pinch's hole, an island on a sheet's graded net).

**A ring's sweep read for depth by sampling** (#942, the depth trim, `swept::depth::Sampled`:
the reading for a path that encloses nothing, no proof; where the path encloses its motion,
`depth::cover`'s certified march is the reading): the sweep is the union of
the tool's poses, so a point lies in it by the least over the roll of the tool's signed distance at
it (`Ring::signed` at `pose(t)⁻¹ x`), each basin the samples show settled by golden section, and a
candidate sheet is on the boundary exactly where no instant but the point's own holds it
(`depth::Sampled::cover`: the own basin taken out — the own instant a sample, with samples halving in toward
it until the roll reads no difference, so a partner closing on it is parted from it; a least at
nought to the depth's rounding is the own touch read again). The instant holding a point names the
tool's point there, so a seam — the edge of what is kept (`Sampled::edge_on`, halved to a quarter of
the tolerance; its kept end may be held by under the tolerance, on the boundary to it), traced as a
graph over one parameter with its chord sagging within the tolerance (`Sampled::seam_over`) — carries
both faces' parameters by construction. On the ring along a spline it reads each tube kept on one
side of the bottom meridian only (tube 0 in front of θ*, tube 1 behind), the stretch beside θ*
held by under 1e-7 (the sliver, its fold and B′ within that of one another), the strip's touching
half held by no other basin at all, and the start pose keeping, besides the quadrant behind θ*
below the rim, a wedge in front of θ* beside the rim, held from beyond by tube 0 at an instant 0.12
of its angle on, its width falling to nought at the corner — the corner one point all five pieces
meet at (`swept_depth::a_rings_kept_pieces_bound_its_sweep_by_depth`,
`swept_depth::the_start_wedges_seam_is_traced_into_its_corner_with_its_partner`).

**A ring touching a meridian whole, entered by depth** (#942, the depth trim, `swept::touched`):
cut from a blank its other whole-touched meridian sweeps clear of (`closed::ring_cut_from`, and so
`cut_from`), a torus whose meridian the instants touch whole is entered over exactly what bounds
its sweep, and never through the road's fitted tubes. There the contact off the meridian is one
parallel each side, `n · v = 0` solved round the tube at each angle (divided by `sin(α − θ)`, its
rate by `α` on the meridian itself), and each parallel's sheet folds exactly at the meridian — the
road's reader labels a stretch 0.008 round it a fold by its `s''` threshold, `s''` being cubic
there, which put one tube's touching faces on the wrong leaf and its kept leaf's edge 0.0114 off
the strip's corner. Read for depth (`depth::Sampled::cover`, the roll sampled at the bar's grain — a cover
whose basin is narrower than that can be missed, where a certified march along the roll could not):
of each parallel's sheet one leaf is kept whole and taken from the meridian (its seam 6.9e-8 off
the true edge of what is kept, against the bar); the meridian's half between the parallels that
nothing beyond holds is the strip; and at each end of the roll the other leaf, where it stands
outside every instant from its rim on the end pose's tool, is kept up to where it crosses that tool
again (read as a graph over the angle: the crossing's partner is the end pose itself, measured at
t = 0.01 – 0.05, so that curve is also the edge of the wedge the end pose keeps). Each is its own
sheet over its closed form, fitted quintic to `K2_FIT` of the tolerance: a leaf over the angle from the
meridian to its equator and the roll, the strip over the fraction round the half and the roll —
the leaves and the strip on one set of instants, so the strip's sides are the leaves' seams — and
an end piece over the fraction from its rim to its crossing and the angle, its corner one point.
Toward the corner the piece stands within the rounding of the tool all across and no sign places
the crossing; below the angle where it stands within the fit of the tool all across, the crossing
is taken on a quadratic in the angle through nought, each place of it held to the tool within the
fit — where the seam lies there is the bar's question, its points on both faces the tolerance's.
Every meeting is given: each sheet with the tool at either end in its rim, the strip with each leaf
in its seam, an end piece with its tool in its rim and its crossing, and touching its leaf and the
strip at its corner; so the arrangement seeks none between two sheets. **Of the tool at either end
only what depth keeps is entered** (#942, the ring's ends kept): the quadrant on the side of the
meridian the roll does not go — between the strip's rim on the meridian, the two branches' rims on
that side (a kept leaf's, or where the leaf kept is the other one, its end piece's) and the
meridian at the equator — and beside it, for each end piece, the wedge between its own leaf's rim
and its crossing, closed by a meridian where the farther of the two reaches the other, clear of
the blank; the torus's own frame turned so the meridian stands at `u = π`, its seam round the axis
at the far meridian, and the quadrant parted at the outer equator, the seam round the tube, so no
face crosses one. The part beyond the meridian beside the strip is not entered: the strip leaves
its rim tangent to the tool there, and a whole torus put the two on one side of the edge to
second order, standing 1.3e-9 rad apart at the edge and 6.5e-6 a full step in — ordered only by
the fitted tie's step to its farthest depth, where they stood 1.3e-8 the other way (with the tie at
1e-12 the build was `Wrong`). Kept alone, the quadrant leaves the edge on the far side from the
strip, half a turn apart, and no order rests on a rounding. A branch whose kept leaf ends on the
side beyond the meridian with no end piece there, or an end piece on the quadrant's side, is
refused by name. The cells are classified by membership as before; the candidates no longer close
on themselves, but every place of the sweep's boundary in the blank lies in one of them, so no
cell straddles it (the judge would refuse one that did). Moved by an ulp or a millionth — a control
point, the radii, the centre's height, the block's top, the roll's ends within the curve's domain
— it builds the same seventeen faces, each volume within what the move sweeps
(`swept_rings::the_ring_along_a_spline_builds_at_its_inputs_moved_by_an_ulp_and_a_millionth`, slow
tier). **The symmetric limit.** A meridian is touched whole where its normal velocity reads nought
within the rounding (`Ring::touched_whole`, `HALF_READ` of its scale), not the tolerance: the ring
moved along the path's tangent by 1e-6 touches none and the road refuses at its outer equator; by
1e-9 or 1e-10 it is still read touched whole, and the residual `n · v` on the meridian, divided by
`sin(α − θ)`, is a pole no fit holds — refused by name at the meridian (`Curved`), not `Failed`;
by 1e-12 or an ulp it builds. A build there would read the residual as the rounding it is (the
sweep stands within 1e-9 of the symmetric one, under the tolerance): not done. Refused by name, `Curved`
where it reads so, wherever this is not what depth says (the tool no torus, the far meridian or a
leaf's equator in the blank, the contact not two parallels, a leaf kept on both sides or neither,
any kept sheet's place held from beyond by more than the tolerance); with no blank, the road's
refusal at a meridian touched whole stands. **To serve `closed::ring`** (no blank, the box its
domain) the same construction needs the far meridian too: its strip, entered as the near one's
(which half nothing beyond holds read there on its own); each leaf over the whole half turn from
one meridian to the other rather than to its equator, which leaf is kept read near each meridian
— the leaf kept beside one may be held beside the other, and the seam where it changes is then a
cover's edge across the leaf, which no leaf is cut along yet; and the end pieces bounded where
they reach either meridian. Not entered: `ring` keeps its refusal
(`swept_rings::a_ring_carried_along_a_spline_touching_its_meridians_whole_is_cut_from_a_block`
asserts it). The bench's ring along a spline, its runner's frame,
a bar of 0.005 and the tolerance of its size: built in 2.8 s (entered in 1.2 s, arranged and
judged in 1.5 s; its sheets quintic, so a reading span by span — its area, 2.8 s — has a few
spans to read where the cubic fit gave hundreds), closed, volume 10696.461766 (the peer kernel's 10696.4618), five B-spline faces and the
block's six planes, membership (sampled, above) agreeing at seeded points over the groove and its volume counted
by forty thousand of them within three standard errors (56.49 ± 1.28 against 55.54)
(`swept_rings::a_ring_carried_along_a_spline_touching_its_meridians_whole_is_cut_from_a_block`,
`swept_rings::the_rings_groove_counted_is_the_blocks_volume_less_the_cuts`). **Membership along a
path that encloses nothing** (`Trajectory::General` whose `Path` gives no `stretch`, as the bench's
runner's does — it gives poses alone, and nothing bounds their rates) stays `Unbounded` for every
reader of `member::Swept`, and is read at the carried point's greatest sampled speed,
`|v(M(t)⁻¹ x)|` over 4096 equal steps, only where it is asked for by name (`Swept::sampled`, set
by this entry alone for its cells and its judge): no proof, as the depth trim's own reading is
none, and the gate's membership agreement is read so. The end pose's kept quadrant comes out as
two faces either side of the torus's own seam parallel (`ψ = 0`, its outer equator, which runs
through the quadrant and which turning the seam round the axis cannot move): six torus faces where
the peer kernel has four.

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
**Dispatch** (the kernel's, `brep::swept::dispatch`, #721 F1c; `Closed::general`): a sweep every other class refuses,
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

**A prism under a motion that keeps a plane, swept closed (#880 Q2, `brep::swept::planar_solid`):**
a sweep with no blank (`swept::closed::closed`) whose motion is no constant twist nor a path of
pieces, but every pose of which turns one plane in itself, the same face up (`region::keeps`, at 17
rolls and then on the region's own grid), of a tool standing along that plane's normal — every face
a cap square to it or a side along it, read from the B-rep (`subtraction::Prismatic`), no side
leaning — is asked this before K2's road: the swept solid is, slab by slab between the tool's caps
(`Prismatic::slabs`), the region its section sweeps in the plane carried through the slab. The
region is the planar classes' (`region::SweptRegion`): each side's contact, each corner's path,
each edge at an end pose where the roll is no whole period (`Setting::alike`), kept where it stands
outside the section at every pose and linked into loops. **It reads no blank**: its box holds every
pose of the section (sampled four a degree) with a quarter of the section's size and the farthest a
point moved between two samples to spare, and a loop that runs along the box is refused (`Failed`),
never built. Each stretch is a cubic B-spline from corner to corner fitted within a tenth of the
tolerance (`region::profiles`), each loop's prism extruded through its slab (`build::prism`), the
slabs and pieces united, and the solid judged by membership before it is returned
(`closed::judge`). A region whose boundary folds is `Folded` with its point; any other refusal of the
region's is `Failed` saying what it saw and where. Two stretches crossing at a grazing angle — two
corners' traces at a lobe's apex, each kept past their crossing as far as the keeping test's
tolerance reaches along so shallow a meeting — are paired where their ends lie within `grazing` of
each other and their crossing is found (`region::joined`), where before only `loose` was. Held
(`tests/swept_planar_solid.rs`) on brep-bench's three cases of the kind: **the square on a planet
carrier** (a whole turn, 39 faces) and **the plus in mesh with a wheel** (a whole period of 540°, an
annulus whose hole is the wheel left standing, 47 faces) each to 1e-6 of the area the definition
gives in the plane, read line by line on the exact section — the polygon at 9000 poses and each
edge's quadrilateral between two, so each corner's trace is filled to the square of the step — the
square's surface also to 1e-7 of twice its region's area and its boundary's length through the
height, both read ray by ray from the axis (`ring_by_rays`: on each ray the poses' intervals run
together into one, each extreme refined at every local extreme of a grid; #880 E4, where cadrs's
surface is 0.0705 larger with the same volume) — and
**the rack's tooth space**, its swept solid cut from the blank by the ordinary Boolean, to 1e-6 of
the volume next.BREP and cadrs agree on, at a billionth (4e-11 off); each written as a closed STEP B-rep.
**Cut from a blank too** (`cut_from`): the same road, its solid cut from the blank by the Boolean —
**the key carried along a spline** (`swept/key-carried-along-a-spline`, a frame along a cubic
Bézier, `trajectory::Along`), once K2's `Curved` at the key's corner, its swept solid held to the
definition's area in the plane and the grooved block to the volume OCCT and cadrs agree on and
their area, at a billionth (2e-12 off; `a_key_carried_along_a_spline_grooves_a_block`).  The
keeping test reads a stretch of the roll at its ends alone below `KEEP_STEP` of the roll, or, for a
point moving faster than the tool's reach a unit (a roll that is no angle: a curve's parameter),
below as far as `KEEP_STEP` of a turn at that reach moves it — at a roll's floor the key's corner
sliver, covered for 5e-4 of it, was kept and the region left open.  The drawing's sweeps still
reach these cases by the blank-scoped planar classes (`reads_general` does not ask this, #699). **Left open**: a drafted tool (its stations lofted, as the
subtraction class lofts them); a tool standing along the plane's normal under a constant twist,
which the arrangement builds exactly. **A box along a spline frame** (#880 T1d, once `Folded`):
`path.sv`'s frame keeps the box's instant centre on the line of its side through the frame's
origin, a corner, so three rules hold of the region's candidates. A side's contact within a
billionth of its length of an end is no point of the side's (`Kind::Line`) but the corner's, which
carries it — read with the rounding of the side's parameter, it stood either side of the end and its
walk turned about. A corner's side whose `n · v` is nought there — the corner moving along it — is
read by its rate along the side away from the corner, which says which way the side turns off the
corner's path (`Kind::Corner`), its normals the sides' own where the fan reads one of them (a part in
1e11 off, a speed of 40 reads as motion across it). And where the turn changes sense within the roll
(`region::turning_back`, the spline's inflection) every edge is held at that pose as at an end pose:
there the motion is a slide for an instant, a side along it in contact all along, and the corners
carrying the boundary before and after stand that side apart. Held
(`swept_patches::a_box_along_a_spline_frame_is_its_sections_planar_sweep`) to membership and to
1e-7 of the area the definition sweeps in the plane, read on lines turned 0.3 rad off every end
pose's side (1.3e-9 off).

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

**A traced sheet is laid in its cut's own scale (#904 A3):** the reach (`Cutter::reach`) asks every
piece of a section loop at the same number of places (`REACH_SAMPLES`, and each convex corner's fan
at its middle), however long, and its walk is the complement of the longest stretch of faces missed
between two hit, by their length — at a fixed 0.3 mm the pair at 0.625 missed the crown tooth's
0.24 mm tip, and a tie of face counts turned the walk the wrong way round the tooth. A cutter of
revolutions' cut has a unit (`Tracer::unit`): the length of the faces that walk runs over at the
band's middle station (`Loop::walked`) over `REFERENCE_WALK`, 13.7855 mm, the configured pinion's
crown tooth's at the mill its balance of profile radii found, where the construction's lengths
were set (at the mill its balance finds since #904 it walks 13.702 mm, a unit of 0.994). The rows' and columns'
spacings and the trace's step, how far a curve first runs out of the blank, a fan's walk
(`Loop::scaled`, `TURN_SCALE`), and the gross fit's distance, its local trust and how near the
blank a withheld contact is held, are each that many times their constant. A stated tolerance stays
a physical length (`Tolerance`), as the sector's side clearance (`sector::MARGIN`) does; a screw's
sheet keeps its own spacings, and a prism's cut stays at unit one (open): its stations are
millimetres along it and a drafted prism's crease fan is laid when the cutter is read
(`CREASE_SCALE`). Moving the bar's cap off unit one moves `prism_caps`' and `bar_end.sv`'s volumes
by up to 5e-6 either way (from 0.5 to 2 of it), past the 1e-6 and 2e-6 they are held to, so its
scale waits on the fit's own scatter there. So the hypoid
pair with every length scaled (the cutter stated with the rest, the shift its balance finds the
same) is traced as the pair is, and built with the same faces
(`brep_export::the_pinion_scaled_is_built_as_the_pinion_is`, at 0.625;
`each_member_of_the_pair_is_built_alike_at_every_scale`, slow;
`hypoid_machine::the_face_mill_scaled_is_found_scaled`). What scaling no longer moves, resolution
still does, and the configured pinion's cut holds only near its own unit (open): traced at 0.97 or
1.02 of it a station beside the blank finds no contact near its neighbour's, and at 0.95 or 1.05
neither row placement fits within the gross bars — by walk length the fit turns 21° from contacts
a quarter millimetre outside the blank by its first rows, and by length in space a column whose
curve is short in space crowds its rows and the fit's normal turns 90°. The gear's cut and both
reliefs hold over 0.85 to 1.2.

**A traced sheet does not depend on where its cut stands (#904 A4):** a cutter of revolutions'
reach (`Cutter::reach`) reads its `STATIONS` stations a step apart from the half-plane
`Cutter::of_boundary` counts from, which a world axis picks, and then again from where its
contacts first reach the blank — the station before the band's first and the first bisected to it,
to `START` (1e-12 rad) — so the band, the walk and the cut's unit, and the columns laid across the
band, stand where the cutter and the blank stand, turned together. The sheet's rows are as many as
keep them `ROW_SPACING` apart, and where its curves end on the trace's grid of walk lengths its
span is a whole number of spacings: one to rounding (`WHOLE`) is counted so, not one more as a
placement's rounding falls. Counted from the world's half-plane, a turn of the pair moved the grid
against the cutter, and the gear at 0.625 was built in its own frame and refused in `drive.sv`'s,
"the candidate sheet cannot be widened out of the blank"
(`hypoid_machine::the_gear_builds_alike_in_a_frame_turned_to_the_pair`, slow: the same faces, the
volume to 1e-9; `the_gears_sheet_is_traced_alike_in_a_frame_turned_to_the_pair`, slow: the cut's
sheet, each point the other's carried, to 1e-6 mm). That refusal was the chart's, at four of eleven phases of the grid within one
step, in either frame: a margin column's curve leaps a turn round into a far corner's fan a row
nearer the blank with each column further out, and with the rows trimmed back to before any
column's leap (`charted`), wherever a column leapt on the row past the blank's last every column's
rows stopped in the blank, however far the margins grew. Outside the columns holding the blank
(their stations reached it, or a contact of theirs lies in it) a column leaping anywhere from the
blank's first row to the row past its last now goes, with those beyond it, where one holding none
is left at that side; the rows are then trimmed at the kept columns' leaps as before
(`contact_trace::a_margin_column_leaping_beside_the_blank_is_dropped_rather_than_the_rows`;
`hypoid_machine::the_gears_sheet_is_traced_at_every_phase_of_its_stations`, slow, the grid turned
past the band's start by `Cutter::turned`, `sweep::traced_turned`). So are the configured pair's
gear cut with a 90.37 mm cutter and both members of the 25-tooth pair built, where that refusal
was seen too.

**A sheet's rows lean where a leap pins them in the blank (#904):** a column holding the blank
leaps where its contact curve runs into a convex corner's fan of the cutter (the neighbour's flank
meeting the crown's floor, `Space`), a fixed walk past where it leaves the blank. Where the blank's
edge crosses the stations obliquely (a bevel gear's face 0.3 R wide, on a drive), the corner does
too, and rows at one walk length in every column cannot be past the blank in one column and before
the corner in another: trimmed at the first leap (`charted`) the kept rows stopped in the blank
(`pinned`), and no margin moved the corner, "the candidate sheet cannot be widened out of the
blank". The rows are then leaned (`Shear`: row `x` at station angle `a` is walk length
`x + slope·(a − at)`, `at` the band's middle) by the slope among `SLOPES` each side of none that
keeps the nearest leap furthest outside the rows the blank holds once leaned, where that is further
than none keeps it and at least a row and a half (`sheared`), up to `SHEARS` times; the columns are
traced again over the leaned span, and every station a finer grid or a withheld contact asks for
is read leaned alike (`Layout::station`, `Layout::at`, `Anchors` in row coordinates). Rows never
pinned are not leaned, and their sheet is traced as it was; rows a widened margin once carried past
their pin (the drive's gear relief, at its fourth margin) are leaned at the first instead. A leaned
sheet is said so (`its rows leaned … to leave the blank before a leap`). A leap within the rows a
column itself holds in the blank is no lean's to clear, and stays the chart contract's
(`contact_trace::rows_pinned_by_an_oblique_corner_are_leaned_clear_of_it`;
`hypoid_machine::a_gear_whose_blanks_edge_crosses_the_stations_obliquely_is_built`, slow: the
18/54 pair at no offset on 0.3 R faces, the same solid with the crown's floor 2 and 3 normal
modules down, to the fit). Left open: the 24/48 hypoid of `hypoid_machine::scaled_pair` on 0.3 R
faces is leaned out of the blank, but its fitted sheet then folds in the blank under either row
placement (by walk length it folds there on its own 0.2 R faces too, and by length in space holds).

**A fold's gap outside the blank is crossed, not turned at (#904 A1/A2):** where a station's root
meets the other in a fold of the time chart outside the blank and comes back further along the walk
(within what is left of the curve's extent, the chord across the gap outside the blank), the walk
crosses the gap and runs on along its own root (`Tracer::trace`), and a row in the gap is the chord's
point (`Tracer::along`); a fold in the blank, or one past which the root does not come back, is
turned at as before. Where the fold's locus ends in a cusp among the stations, a station before it
folds and the one after does not; turned onto the other root, the folding station's rows ran on in
that root's part of the envelope while its neighbour's stayed on the root they share, and the sheet
tore between them. At the face mill the configured pair's balance finds (8.8549°, the cutter
75.6122 mm) the inner flank's fold near its round ends so at the start of the pinion's band, whose
last station before the cusp holds the blank on both sides of its gap: turned there, its rows ran a
turn round the pinion, and the fit left its contacts by 0.155 mm and turned 89.7° from them. Its
cut is now one chart, 69 × 39 (`hypoid_mill_sheet::the_pinions_cut_at_the_found_mill_is_one_chart`),
and the pinion is built by sectors with 243 faces, 14664.2385 mm³
(`the_pinion_at_the_found_mill_is_built_by_sectors`); the tip relief its inner flank keeps there, a
hundredth of a millimetre thick, is sampled as `query::interior` says below.

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

**A finite screw's roll need not carry its section past the blank (#721 F4):** a point of
the section stands for the characteristic's at its own delay, so the section's columns end
where the roll has carried the latest of them. Where an end of the columns the roll cut short
stands in the blank, read at the characteristic's nodes, or no column is left,
`helical_sheet` carries the original characteristic through the actual poses instead:
`S(s,t) = M(t)·c(s)`, rows by its length and normal turn, columns over exactly that roll,
whose ends E1 holds clear of the blank.
E1 and S1–S4, the sheet's edge-clearance check and the fit's withheld-position and normal
bars still hold. No pose outside the roll is added. The gate is
`prism_twists::a_screws_section_can_span_more_than_its_roll`: a bar leaning 40° screwed
±3 mm per turn through 280°, its section's delays spanning over 315°, and 3 mm per turn
through 400°, where the columns left are 77° of the 240° that pass the block's heights, each
cut said to be laid over the stated roll and held to
an independent half-space volume within 3e-5 relative and zero STL/field disagreement at
0.01 mm. `solid_sweeps_examples::the_short_screw_carries_its_characteristic_over_the_stated_roll`
holds `short_screw.sv` and its stepped build; the native path's temporal-contact chart
still refuses a screw (`Uncharted::Motion`). A genuine crossing stays refused at S4
(`swept_vertex::a_short_screw_whose_section_crosses_still_fails_s4`). A stretch of the
characteristic whose two nodes are one point, a closed face's seam, is that point at every `s`
between (`Characteristic::at`), not the rings between its last station and its first:
`turns_and_slides::a_torus_screwed_at_a_short_lead_is_carried_across_its_seam`. This does not
settle F4's crossed sections, rim fits or inverted triangles at other leads.

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
it is (`side_at`). S1 refuses where a corner's, a rim's or a crease's root runs off its line at a
vertex some edge of which is no fan the walk reads (`VERTEX`; at any other it goes round the
vertex, G1 below), where a side's root
runs over a cap with no rim to cross (`OVER_CAP`: a round side's), where a cap's line leaves it
onto none (`OFF_CAP`), each within the walk's margin of the blank. Whole silhouette ribs under
a slide or a screw along the extrusion join straight cap rims through their vertices (G1b,
below). The sheet
is `brep::sweep::helical`'s, cut into faces at each fan's ends. The gate is
`solid_sweeps_examples::the_flat_end_cuts_a_floor` (`solid_sweeps/flat_end.sv`, a bar's flat end
facing down toward the axis turned across `turn.sv`'s block: the end's line turned is the floor, the
inner side's the wall, the rim's fan between; held to the bar's shadow read apart, each height's
slice of its half-spaces bounding what the turn removes there, within 2e-5 at 0.001 mm, 9e-5 at
0.01 by the fit's bias) and `prism_twists::a_bars_flat_end_screwed_cuts_a_floor` (the bar screwed
8 mm a turn, held to arithmetic over `z − kθ`, which the screw keeps, within 2e-5 at 0.002 mm), each
STL agreeing with the field; with the refusals
`solid_sweeps_examples::the_capped_thread_goes_round_its_bars_vertices` (S1, `VERTEX`, before G1),
`a_cap_in_the_blank_under_a_slide_turns_round_its_vertices` (G1b) and
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
a screw along the extrusion, the silhouette is read by the whole-edge reading of G1b below. A corner of a half meeting the plane is a vertex, and a
round side's crease is still not walked (S1, `OVER_CREASE`). The gate is
`solid_sweeps_examples::the_creased_bar_crosses_its_crease` (`solid_sweeps/creased.sv`, a bar
leaning across the turn drafted 2° both ways, its inner side's halves joined round the crease;
its shadow from its ten half-spaces within 2e-5 at 0.001 mm) and
`prism_twists::a_bar_drafted_both_ways_screwed_crosses_its_crease` (screwed 6 mm a turn, by `z − kθ`,
within 2e-5 at 0.001 mm), each STL agreeing with the field, with the refusal
`prism_twists::a_corner_meeting_its_crease_goes_round_the_vertex` (S1 before G1) and
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
(`prism_twists::a_side_along_the_slide_is_planar_subtractions`) — each of these, and the end mill,
the step, the turned and tapered mills and the bar lying along the turn, standing along the normal
of a plane the motion keeps read from its B-rep, is the planar subtraction class's since #721 F1c,
asked before these rows, to the same closed forms; the bull-nose mill, its round a torus, keeps the
walk gated; the creased fit's
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
leaves none). An apex on the tool's axis is walked by its own junction (G5, below), and so is a
vertex of a prism every edge of which is a fan (G1, below). The gate is
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
slid square, stood along the normal of the planes its slide keeps: the planar subtraction class
asked first, the closed class building it); and `object_builder::a_sharp_edge_along_one_points_path_builds_at_the_displays_samples`.

**An apex on the tool's axis (#721 G5):** where a face of revolution's meridian ends on its axis
with no face beyond and its normal there off the axis (`constant_twist::apex`: a cone to a point,
an ogive; a smooth pole is none), `Junctions::new` reads `Junction::Apex` at that end, and `step`
crosses it to the apex's ring's other root at the same point, going back the way it came. The
apex's ring is a ring of no radius read as any ring is (`apex_roots`: `RingFrame::coefficients`
with the face's normal there `nr e + nz a`, `nr = −α` and `nz = β` off the meridian's tangent
`α a + β e` at the apex, and `Sinusoid::roots`), its two roots the normals the walk turns between,
each turned outward as the face's a thousandth along the meridian is (`outward`); none where the
apex is in contact nowhere, `DoublePoint` where its roots meet, `AllRound` where it stands still.
A step takes the next ring's nearest root and, of roots as near as each other (within 1e-7 of the
tool's size), the one whose normal is nearest the one it stands on: onto or off an apex every root
there is the one point, or as far from it. The nodes hold the apex once, `Crease::Apex` on it, and
the next node's step starts at its station (`turns`, one ring's two roots in a row); it is a crease
(`creases`, `Characteristic::apexes`), built as V's is. S2's ring rule lets a ring's two roots
stand either side of an apex the walk turns at between them; `stationary` reads no pair at one
station. `APEX` is said where the walk meets an apex it does not turn at (short of a face's end, at
an end another face meets, or whose ring has no two roots). The native path refuses a characteristic that turns at an apex by name (`--kernel rust`). The
gate is `swept_apex` (the walk in the kernel under a screw, a turn and a slide, cones of
30° and 60° and an ogive, each meridian either way: every node in contact within 1e-9 of `|v|`,
its normal outward, the apex once, the blank reached either side, the sheet creased there alone)
and `solid_sweeps_examples::the_pointed_tool_chases_a_thread_its_root_the_apexs_helix`
(`chased_point.sv`, the rod's loss in helical coordinates within 1e-6, the page's stepped build
held to the export's), `the_pointed_tool_engraves_an_arc_its_root_the_apexs_circle`
(`engraved_arc.sv`, the cone's shadow turned within 1e-6) and
`the_raked_pointed_tool_cuts_a_vee_its_root_the_apexs_path` (`raked_vee.sv`, exactly), each STL
agreeing with the field. Its point read right is the kernel's and the field's (`brep.md`'s
inverse across the axis and point location near a pole, `fields.md`'s convex arc at the axis), held
by `swept_apex::behind_a_point_is_outside_the_tool` and
`the_drawings_field_reads_behind_an_ogives_point_as_outside`.

**A vertex of a prism (#721 G1):** a corner's, a rim's or a crease's fan (`Ruling::Rib`,
`Ruling::Rim`) takes among its stations those where its root reaches either end of its ruling
(`Ruled::crossing` at both ends, as a side's at its rims), each end a vertex of the prism. For each
such end `Junctions::new` reads the vertex's fans (`Junction::Vertex` in `Along::off`).
The vertex is keyed by each side's profile corner and ruling end (`PrismVertices`, #721 G1b):
adjacent sides are joined by `Tool::edges`, drafted halves by `Tool::creases`. No points are
matched and no faces counted. Every incident rib and rim must have a read fan, and at least
three fans must meet; a missing concave rib or round rim still refuses the junction. Where a
fan's next ring has no root, `step` crosses the junction at the ruling's end its root stands
nearer. It enters the other fan whose root is there in the direction its roots continue.
`Step::Vertex` records the crossing and its topology key; `Crossed::Vertex` marks the node as
`Crease::Vertex` instead of inferring a vertex from coincident fan points. Other incident fan
roots at that vertex are covered by the same normal-cone crossing (S2), not a second stretch.

**Whole edges and cap-line vertices (#721 G1b, with G4):** a side, rib or rim whose endpoint
contacts agree all along its parameter is read as whole rulings at the contact's roots
(`silhouette`, `Line::of`). Rims are constructed before this reading, including under a slide
or a screw along the extrusion. A whole rib or rim retains its topology and joins the vertex
at each end; a whole side ruling joins its rims. A fixed edge whose endpoint contact functions
are positive multiples is read the same way (`Ruling::isolated`): it has no ordinary root on
the edge, only whole-edge contacts where both functions vanish. Opposite multiples have a
root inside the edge and branch there, so S1 still refuses them. A cap line ending at a profile
vertex retains the side-chart endpoint and crosses its vertex junction too (`Line::vertices`).
Consecutive crossings at the same vertex keep one node and the last piece's starting station;
a repeated endpoint must not create a zero-length section edge that S4 reads as an intersection.
The gates are `swept_vertex` (slides at three inclinations and two paces, a screw along the
extrusion, and a cap contact line ending at a vertex; every node in contact, outward and on the
tool, every vertex crossing creased; the cap-line cut also held to the bar's half-spaces and its
STL to the field) and
`prism_twists::a_cap_in_the_blank_under_a_slide_turns_round_its_vertices` (the exact cut to
`capped_slide_removed`, the exported STL agreeing with the field).

**A flat's line leaving at a vertex (#721 G1b):** a prism side kept in its own plane
retains the side-chart corners at the endpoints of its contact line (`Flat::vertices`,
`side_line`). These join the same `PrismVertices` classes as ribs and rims, without
matching positions. At a vertex, an edge beside a flat can replace a read fan only
when its corner's chord is in the tool's material; its cap rim can do so only where
the lone prism's caps are walked. An unread parting crease still refuses. The
junction enters the other incident fan or flat line, records `Crossed::Vertex`,
and creases the sheet along the vertex's path. A line endpoint inside an edge
keeps the existing `Beside` junction. `swept_vertex::a_flats_line_leaves_through_a_vertex`
holds the walk to contact within 1e-9 of speed, outward normals and exactly the
vertex creases, with the profile drawn either way and the turn at two paces and
either direction. It holds the cut to its annular band's closed form within
1e-5 relative volume, with zero STL/field disagreement. A concave corner beside
a flat stays refused (`swept_vertex::a_flat_at_a_concave_vertex_still_ends_the_walk`).

G1b remains open for concave vertices, round rims and creases, caps of tools no lone prism,
flat lines ending at unread parting creases, and the generating class's corner in a cap band (T3). A plane
no lone prism's cap whose contact reaches the blank remains T1 (G9).
The native path refuses a characteristic that goes round a vertex by
name (`--kernel rust`). The gate is `swept_vertex` (in the kernel, `flat_end`'s bar with its section
turned 30°, 45°, 60° and −30° so its corner runs to its far end, and a bar drafted both ways whose
corner runs to its crease, each turned and screwed: every node in contact within 1e-9 of `|v|`, its
normal outward, the sheet creased at a vertex of the prism and nowhere else, the blank reached
either side of one; a bar whose inner side bulges to an arc still refused, `VERTEX` turned and
`OVER_CAP` screwed; and drawn, the bar turned ±30° and screwed to `Bar::lost` within 2e-5, its STL
agreeing with the field, `a_bars_corner_running_to_its_end_screwed_cuts_as_the_bar`),
`prism_twists::a_corner_meeting_its_crease_goes_round_the_vertex` (turned and screwed, within 2e-5)
and `solid_sweeps_examples::the_square_ended_bar_goes_round_its_vertex` (`square_end.sv`, within
2e-5) and `the_capped_thread_goes_round_its_bars_vertices` (`capped_thread.sv`, within 2e-5 of
`closed_class::screwed_partway_away`), each STL agreeing with the field and the page's stepped
build held to the export's. The fitted sheets come out a few millionths low, as every turned or
screwed bar's do (F4).

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
refuses it by name. The gate is `closed_characteristics::a_bull_nosed_drum_punches_its_shadow_rounded`
(within 2e-6 at 0.001 mm), its STL agreeing with the field; a plain drum — `solid_sweeps/punch.sv`'s
pushed edge first through a plate (`solid_sweeps_examples::the_drum_punches_its_shadow_through_a_plate`,
`8 × 4 × 4` exactly) and one turned through a ring sector
(`closed_characteristics::a_drum_turned_through_a_ring_sector_bores_a_tunnel`, Pappus within 1e-5) —
stands along its axis read from its B-rep and is the planar subtraction class's since #721 F1c,
asked before these rows; `brep::a_net_closed_along_its_rows_is_a_tube_of_faces` (a cylinder's net closed along
its rows cut four round and two along, its core π r² h); the refusals
`closed_characteristics::a_square_bar_pushed_square_to_its_length_is_planar_subtractions` (the
drum screwed, S2, is now the closed class's: `the_drum_screwed_bores_its_helical_groove`; a prism
aslant wholly through, S1 over a cap, and a prism turned wholly within, S1 a rim's ruling, are the
closed class's too, `closed_class`); what it refuses that the contact's road builds, the leaning
torus turned to a stop in the ring (E1,
`turns_and_slides::a_leaning_torus_turned_to_a_stop_inside_the_ring_is_built_by_the_contacts_road`,
#880 Q1b); what it refuses that the closed class builds, `turns_and_slides::a_torus_slid_through_the_rods_middle_grooves_its_shadow` (its rings at the
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
2e-5, over a sector and a whole turn), each STL agreeing with the field; the refusals of a domed
pin (S1 `POLE`, which the contact's road builds within the blank's reach since #880 T1c,
`rings_all_round::a_turn_about_an_axis_the_tools_meets_builds_within_the_blanks_reach`) and the pin wholly within (S1 `POLE`) and small leaning torus
wholly within (S1 `SMOOTH_LOOP`) the road builds since
(`rings_all_round::what_a_turn_about_an_axis_the_tools_meets_is_built_by_the_contacts_road`, #880
Q1b); and
`object_builder::a_ring_in_contact_all_round_turned_builds_at_the_displays_samples`. A slide square
to the tool's axis, or a turn about an axis parallel to it, stays refused at S1.

**A drafted prism tool (#684 G):** a drafted prism is in the generating class where each corner
joins two lines or goes on tangent: then each side (each half of it, either side of the parting
plane, where the prism runs both ways from it: below) is
the ruled surface between its edge's two end sections, a plane or a cone (`ExtrudedSurface::drafted`,
from the field's `Sections`; `Sections::linear`), its points' normals their own, so its contacts are
read as an undrafted side's and every row is asked of it. The rest is refused by name: a prism
drafted outward both ways from its plane (its sides meet there in a concave crease, T2 since #721
F1b), an arc meeting a side at an angle (T1, G8: the kernel builds its corner as a conic, #962, which
the class pairs no corner of), a side no
line, arc or circle (T1). The cutter sections it at a station as the blend of its two end sections
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
folded the fit. Drafted outward both ways, the crease is concave and T2 refuses it (#721 F1b), as
the cutter refuses to section it. `drafted_tools.rs` and `solid_sweeps_examples.rs` (`parted.sv`, the helical tooth drafted
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
(read from its B-rep whole, #721 F1b: the generating rows refuse its concave corner at T2, the
constant twist crosses it where both faces' points meet it, creased there, #721 F1d,
`union_tools::a_union_within_an_intersection_is_the_constant_twists`, and the closed class reads
such a tool by its B-rep where it takes its faces; members falling to the closed
class and another each built by its road, where the union is distributed,
`union_tools::a_member_its_class_refuses_is_the_closed_classs`); a union of prisms under a motion
keeping their plane, which `planar_subtraction::asks` reads whole; and a union that is one solid
under a constant twist, swept whole as its set (`distributed::whole`, #721 F1d). The native path refuses a distributed cut by name.
`tests/union_tools.rs` and `solid_sweeps_examples.rs` (`union.sv`) are the gate: the beads' groove
against rows across the rod computed apart, placed and turned, against the long form, two balls'
tubes by Pappus, two tori rolled in the generating class, and the refusals.

**A prism's cap in the blank (#684 F):** a prism standing alone as the tool (its B-rep one undrafted
prism, its profile one loop of lines and arcs, however it was drawn: `admission::Lone::read`, #721
F1b; the prism itself, placed, #684 AC, or what a Boolean leaves of it) whose cap passes through the
blank is no longer refused at T1; any other tool's cap is, saying so. A placed prism is read through its placements: its faces carried by their pose
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
`Section` and its material a `Region` the kernel reads off the tool's B-rep, #721 F1d, its refusals typed
`Seen`/`Row` and said by `envelope::planar`; over a formula or generated source: contact roots per sample,
kept where inside every pose, corners where an image recurs), built as the blank in common with
its prism (`brep::sweep::planar`), metered as envelope × slab (`accuracy::Planar`). `claim over
MOTION` advances the placements under it (`Sweep::motion`). A profile field's walls are boxed
(`Boxes`); `ssi` cuts an extrusion square in closed form and `boolean` gives its iso edges straight
pcurves. `rust/examples/wankel/`, `fixtures::wankel`, `tests/{bore,planar_envelope,wankel}.rs`.

**Planar subtraction (#606, #607, #590):** a sweep of a tool standing along the normal of a plane
its motion turns in itself is the blank less the region its section sweeps, slab by slab (the
kernel's `brep::swept::subtraction`, #721 F1c; `solid::planar_subtraction` the drawing's reading;
rows P1, Q1–Q4 in [docs/generating-sweeps.md](../generating-sweeps.md)).  **The tool is read from its
B-rep** (`Subtraction::read`, `Prismatic::along`): the plane is square to the first of its plane
faces' normals the motion keeps at five rolls (P1), and every face is a cap (a plane square to it),
a side (a plane or a cylinder along it) or a leaning side (a plane, or a cone about a line along
it), the sides across any one height leaning alike — by one draft, or none (Q1, refused naming the
face and a point of it: a slanted cut, a side of a prism lying across the plane, a torus).  Its
section at a height is the face the B-rep, cut there by a half-space (`build::kept`), leaves on the
plane, read from the slab's side of a cap at a cap's own height; each loop's lines and arcs walked
with the material on the left from the step on the tool's first face (each face told apart by its
surface and its box, so a plus's opposite arms' coplanar sides are two), each corner one point, so
a drafted slab's stations are matched piece for piece; its material is its loops' signed distance
(`subtraction::Loops`).  The kernel's dispatch asks it outright under a constant twist, and of a
tool the generating rows refuse — a concave corner, a roll never clear of the blank, a second
contact — in their place, so a rack
is still built by its sheet (`extruded`), and a rack tooth with a cap in the blank, or united with
itself, by its region (`rack_cut::a_rack_with_a_cap_in_the_blank_cuts_the_slab_it_spans`,
`the_tooth_united_with_itself_cuts_as_the_tooth_does`: the two classes agree to 1e-5). The slabs are the blank cut at every cap within
it; a union's faces a Boolean merges leave no cap between bands of one section. In each the
kernel's `brep::swept::region` reads the region (its refusals typed, their words
`region::Refusal::words`; two kept stretches walking one curve,
two corners at one distance from a turn's centre, one piece of its boundary) `R = ∪ₜ
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
own, refused at Q2 naming the height to step the tool at, found between the two stations that
differ. `accuracy::Subtracted` meters it, reading each sweep by the class's own
`planar_subtraction::found`; a whole roll never clear of its body is refused saying the period
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
**A STEP's fitted curves follow a stated tolerance and only a stated one** (#961,
`brep::export::step_accuracy`): with `--tolerance` the file states a twentieth of it as its
uncertainty and holds every curve it fits (a traced intersection, a pcurve) within half that of the
exact meeting **on each face it trims**, measured along the face (`step::Fit::stated`) — at a
grazing meeting nearer in space by the angle's sine — so a coarser tolerance writes a smaller file.
Without it the file states the gross 1e-4 mm and its curves are held to `step::Fit::Exact`, 1e-9
mm: a document exported with no tolerance asked reads as exactly as the kernel builds
it (brep.md, "A traced intersection is written as a cubic").
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
meridian section turned once (`Session::built_meridian`, `solvent_cad_revolve_region`;
`SOLVENT_BLANK=booleans`); field agreement probes one sector, reading cuts not proved positive
(`agreement::Sector`, `MaterialField::without_cuts`; `SOLVENT_AGREEMENT=whole`); STEP text is
formatted on every core, the kernel's to the byte (`SOLVENT_STEP_TEXT_CHECK`).

**Speed of the Rust kernel's swept cuts:** every shortcut reads what the long way read. A placement
is compared with the first by `SpatialField::below` (`value(p) < t`, a Boolean's second operand
unread where its first decides, a loop's nearest edge sought only within the level, the parity of
its ray first). A body built whole is split by its placed sheets a batch at a time, each sheet in
the first batch whose boxes stand clear of its own (`sector::apart`): sheets that meet none of each
other split as one. A cell's interior points (`query::interior`) are the deepest first by a
hierarchy of its mesh's triangles, the winding number asked only of the points the choice reaches,
and one read settling every candidate in the ball about it no triangle enters (a closed mesh's
winding is constant off it): the deepest candidates of a thin cell, its box's grid, lie mostly
outside it, and two thousand windings of every triangle were read a cell for its four points. A
cell thinner than its first mesh's sag (5e-4 of its box's diagonal) everywhere has no point that
mesh vouches for, and is read again on one ten times finer (`query::SAGS`, #904 A1/A2:
`brep::a_cell_thinner_than_its_first_mesh_is_sampled_on_a_finer_one`).
Under these, the class's Booleans are seeded and traced as any are (brep.md), a cone's step read at
its radius in the pair's box (`Surface::feature_within`, not at its frame's origin, which may be
its apex), and a spline's foot found as `brep.md` says. A blank's prism of whole circles about the
line, along it, is turned as its section (`Recipe::meridian`), so a chamfered gear's blank is made
again about the axis and built as one sector; a side running in to its bore is clear of the cuts
by its gap's angle times its radius, or by how far nearer the axis than every contact of its slice
it runs where that is more (`Boundary::clearance_of`). `tests/swept_speed.rs` and
`sector::a_side_reaching_its_bore_is_clear_by_its_radial_gap` are the gates.
