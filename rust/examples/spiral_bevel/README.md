# Spiral bevel and hypoid pair

A 24-tooth pinion and a 48-tooth gear, laid out the way a gear designer lays out a hypoid pair:
the pitch cones and the tooth trace through one design point, the mean point M, then the
blanks, the generating crown and the motions that roll it through each blank. Every position
follows from lines, angles, incidence and the stated numbers; no constraint, and no value that
feeds one, computes a point with `sin` and `cos`.

**One file, `hypoid.sv`.** Its head is the pair as a designer states it — the `param`s — and
one call, `pair := HypoidPair(std.top, std.y, std.z, …)`, passing them on by name. The component
`HypoidPair` reads as the steps a designer takes, each a component of its own below it, so every
name the pair makes is under `pair` (`pair.pinion.body`, `pair.gear_cone.M`, `pair.mill.ax`).
**No point is seeded** but each blade's apex and foot, started on the side its cone opens to as
`std.ConeThrough`'s are. Every root the layout could take is chosen by a word — a bound (`foot
inside P` in the gear's cone, `foot outside P` in the pinion's, `A distance(>= 0, along: v) P`),
a sense (`sense: cw`), an ordinate's sign — and the layout solves from those words alone: the
block-triangular order constructs it a block at a time, searching each block's roots for the one
its bounds say (#217). Five numbers are unknown and the solve finds them: `HypoidPair`'s formals
`mn`, `rp` and `pa`, which the call leaves out, and the face mill's `pressure_shift` and
`cutter_radius`, which it leaves to the solve with a seed each (`hint(10deg)`, `hint(80mm)`) — the
only hints in the file.

The layout stands on the pitch plane P, `std.top`, with the gear apex O at its origin. The gear's
right triangle O–M–F and the pinion's axes stand in space square to P on their generators; each
blank is a region about its axis; the crown draws nothing in a view: its tooth and the gear's space
cutter are regions about the cutter's axis (§6.21), sets of the flank cones and the levels along
it, each region's meridian turned once into its solid. The tip relief's kinks are drawn in N, the
normal section, square to P on the trace normal C → M.

The steps read one another through interfaces (§8.1): `Gear` is what the trace, the pinion's cone
and the face mill read of the gear's cone (O, M, its generator and axis, the triangle's legs, the
crown's axis), `Trace` what the pinion's cone and the mill read of the trace (C, the normal, the
line to the heading), and `BlankDesign` the four numbers a blank is cut to. Each step is handed
the instance and may read nothing else of it.

| Step | In `hypoid.sv` | Constructs |
|---|---|---|
| 1 | the `param`s; `HypoidPair`'s proportions | the stated pair: teeth, module, shafts, spiral, allowances; the crown's tooth count, the cone distance R and the depths in normal modules |
| 2 | `GearCone` (`pair.gear_cone`): O, M, `foot` | the gear's pitch cone: the right triangle O–M–F, standing square to P on O → M; the crown's axis up at O |
| 2 | `ToothTrace` (`pair.trace`): C, H | the tooth trace about the cutter centre C, and the normal module, O's distance from MC over R / m |
| 2 | `PinionCone` (`pair.pinion_cone`): A, V | the pinion's cone, solved against the gear's by the shafts and the equal normal pitch |
| 3 | `Blank` (`pair.gear_blank`, `pair.pinion_blank`) | each member's blank, one region: within its pitch cone grown by the addendum (`std.Offset`) and its heel sphere, outside its toe sphere and its pitch cone shrunk by the back depth; the end relief its tip's toe and heel corners chamfered in the same region (`std.Chamfered`); its faces `tip`, `heel`, `toe` and `back` |
| 4 | `FaceMill` (`pair.mill`) | the crown tooth's pitch points a quarter pitch either side of M; the face mill, its axis `ax` through C, the cutter's radius long, the two blades (`Blade`: each a cone about the axis through a pitch point, its half the pressure angle split by the shift), and the levels the crowns' tips and bases stand at; the space cutter's reach |
| 4 | `Tooth`, `Space` (`pair.tooth`, `pair.gear_space`) | the pinion's generator, the crown tooth, a region between the flank cones and two levels, its tip's edges rounded (`std.Rounded`), each flank grown a quarter of the backlash; the gear's, its space cutter, the tooth's mate on the same cones |
| 4 | `Balanced`, `Margins` (`pair.balance`) | the face mill's pressure shift and cutter radius, found: the members' undercut margins equal at M and at the toe (below) |
| 5 | the motions, `Generated`, `Member` (`pair.pinion`, `pair.gear`) | the rolls (each member at the crown's tooth count over its own) and the generating motions; each member its blank less its crown swept through it (`through: body`: the whole roll during which the crown meets the blank, found after the solve, so no roll limit is stated) and indexed at every tooth |
| — | `Topping`, `when tip_relief > 0mm` | the tip relief: a topping beside each crown flank (a kink on the flank in N, the cone through it turned by the relief angle), cut the same way |

The normal module is stated nowhere: the trace constructs it, and every depth reads it. As
configured the pair is a hypoid 25 mm off, whose 25° spiral and pressure shift design out the
undercut a symmetric rack develops past about 15 mm of offset — the shift and the cutter found
by balancing it, 11.25° and 75.6 mm; zero offset is a bevel pair with
a common apex. The allowances — 0.05 mm normal backlash, 0.2 mm tip relief and 0.2 mm end relief
— are each zero for the conjugate, sharp-edged pair, which is what the test suites record
(`fixtures::gear` sets the document's own inputs for them).

The app opens `hypoid.sv` as the example `spiral_bevel` (the members are `pair.pinion.body` and
`pair.gear.body`), and the glass box (⌘B) shows them refining from their material fields; `crown.svd`
draws the crown tooth. The layout's checks read the same file
(`gcs-core/tests/hypoid_layout.rs`, `region_blank.rs`, which holds each member's region blank to
its own faces).

A built member's faces are named, as any body's are: its blank's by the sets its region's body
uses (`pair.pinion.body`'s `material.tip`, `material.heel`, `material.toe`, `material.back`), and
each face a tooth space leaves by the crown face that made it (`indexed.outer`,
`indexed.outer_round`, `indexed.tip`; the gear's by its space cutter's, `indexed.outer_crown.inner`
and `indexed.neighbor.outer`). The generating-system checks (`gcs-core/tests/envelope/paired.rs`)
read those faces off the members built at one tooth space each, and hold each flank to the
analytic envelope of the crown face its name says — with the seams, corners and edges of its face
loop solved on the analytic faces where the built member says they meet, never on the kernel's
fit.

## Making the pair

The fabrication files, each member's STEP (the master: exact cones, spheres and end bands,
fitted generated faces) and its STL, both held to 10 µm of the exact surface:

```sh
make solventc OCCT=1
mkdir -p build/exports
for m in pinion gear; do
  build/solventc rust/examples/spiral_bevel/hypoid.sv --solid pair.$m.body --tolerance \
    --step build/exports/hypoid-$m.step --stl build/exports/hypoid-$m.stl --no-diagnose
done
# every exact face within 10 µm, or exit 1
for f in pinion.step pinion.stl gear.step gear.stl; do
  build/solventc rust/examples/spiral_bevel/hypoid.sv --solid pair.${f%%.*}.body --tolerance \
    --measure build/exports/hypoid-$f --no-diagnose
done
# the pair from the two STLs: shafts, overlap, flank clearance, backlash, contact pattern
cargo test --manifest-path rust/Cargo.toml -p gcs-cli --test pair_check -- --ignored --nocapture
```

A member with swept cuts is built only inside the generating-sweep class
(`docs/generating-sweeps.md`): admission asks each swept cut its rows and refuses with the row
and a witness, and nothing is written unless the shell checks pass and the mesh agrees with the
material field. Each member has two sweeps, its crown's and its tip relief's. The exports take
about 14 and 19 seconds (pinion, gear) on 12 cores (docs/native-speed-plan.md); measuring a file
takes longer than making it (`--measure-samples` sets how finely). `--stl-backend mesh` meshes a
member from its material field instead; progress is reported on stderr. As configured before
the face mill was found (a 12.5° shift and a cutter of 0.8 R; docs/native-hypoid-plan.md, phases 5
and 6):

| | STEP | STL | worst exact face, STEP / STL |
|---|---|---|---|
| pinion | 26.7 MB, 147 faces | 45.8 MB, 915,028 triangles | 2.75 / 3.94 µm (a fillet / the tip cone) |
| gear | 48.7 MB, 291 faces | 17.3 MB, 345,970 triangles | 1.44 / 6.16 µm (a root / a fillet) |

Read from the STLs alone, the shafts stand at 89.9997° and 25.0001 mm; through one gear pitch
the members stay at least 21 µm apart, each flank pair 21–26 µm (half the backlash by design);
with one flank pair turned into touch the other opens to 42–48 µm, the stated backlash within
the files' tolerance, and the bearing runs the whole face width, toe to heel.

The stationary gear space cutter can be inspected on its own (an explicit `--solid` may name
private construction geometry; it grants no access from other source):

```sh
build/solventc rust/examples/spiral_bevel/hypoid.sv --no-diagnose \
  --solid pair.gear_space.body --step build/exports/solvent-gear-space.step
```

## The Formate pair: the pinion generated by the gear

`hypoid.sv` cuts both members with a third body, the generating crown: each is conjugate to the
crown, so the two are conjugate to each other only approximately once the shafts are offset.
**`formate.sv`** (issue #283, step 1) takes the crown out of the pinion. It imports the layout,
the blanks and the face mill from `hypoid.sv` and cuts the members differently:

- **The gear is formed** (Gleason's Formate): every flank of its tooth spaces is one of the face
  mill's two cones, copied round the gear's axis with no roll. Its tooth is therefore the ring
  between those cones about the cutter's axis (`hypoid.sv`'s `Tooth`, the region the crown tooth
  was, now standing in the gear), and a space (`Space`) is what lies
  between one tooth's outer cone and the next tooth's inner cone, a gear pitch round. The space's
  floor is its own cutter's only; the tooth ahead bounds it by its flank alone.
- **The pinion is generated by the gear**: its blank less the gear's tooth swept under the mesh,
  `motion(gear_turn, relative_to: pinion_turn)`, the gear turning about its shaft at the tooth
  ratio (`-pinion_teeth / gear_teeth`) as the pinion turns about its own. No cone distance and no
  roll ratio is read, and no crown enters the pinion's definition (`gcs-core/tests/formate.rs`).

Each pinion flank is the envelope of a gear flank under the motion the pair runs at, so the pair
is conjugate by construction at any offset. The backlash and tip relief are carried as before:
the gear's teeth stand a quarter of the backlash inside the ring, and the pinion's cutter is the
ring grown by the other quarter. The pressure shift is stated here, 12.5°, the cutter 0.8 R; the
generated pinion takes it without undercut, admitted with a least area factor of 0.72 for the
conjugate design.

```sh
for m in pinion gear; do
  build/solventc rust/examples/spiral_bevel/formate.sv --solid pair.$m.body --tolerance \
    --step build/exports/formate-$m.step --stl build/exports/formate-$m.stl --no-diagnose
done
cargo test --manifest-path rust/Cargo.toml -p gcs-cli --test pair_check formate -- --ignored --nocapture
```

The pinion's two sweeps, the gear tooth's and its tip relief's, are admitted and built as one
sector (243 faces, 125,376 triangles, about 14 s on 4 cores). The gear has no sweep: it is its blank less 48 spaces and 48 tip reliefs, each
Boolean in turn (436 faces, 95,100 triangles, about 95 s on one core). Read from the two STLs:

| | crown pair (`hypoid.sv`) | Formate pair (`formate.sv`) |
|---|---|---|
| each flank pair over the pitch, as laid out | 22.7–26.2 / 21.7–24.4 µm | 21.6–23.9 / 20.9–22.5 µm |
| one side closed: the touching flanks over the pitch | 0–4.1 / 0–2.7 µm | 0–3.1 / 0–2.1 µm |
| one side closed, at 4 µm export: the clearance's spread / the touching flanks | 1.4 / 1.7 µm, 0–1.7 / 0–1.8 µm | 1.0 / 0.5 µm, 0–1.6 / 0–1.1 µm |
| the film (6.35 µm) at one pose, at 4 µm export | 11.8 / 14.1 mm² | 43.8 / 26.3 mm² |

At this design the crown pair's departure from conjugacy is already within a few microns, near
what the files resolve. What separates the two is the film at a single pose: the band within
6.35 µm of the gear runs the whole face, toe to heel through the mean section, for both, and is
two to four times as wide for the Formate pair, whose flanks conform along the contact line.
`pair_check` holds the Formate pair to that: each side's clearance varies over the pitch by less
than the STL's sag, closed it touches all through the pitch, and at one pose the touching band
spans at least 80% of the face through the mean section. The normal backlash it reads is half
the backlash on each side, the closing turn opening the other by the ratio of the two flanks'
levers (17.7 and 22.0 mm here): 38–46 µm for the stated 50 µm, the files' sag taken off.

This is a common-crown construction with a stated backlash and tip and end relief, not a
production acceptance result; [the roadmap](../../../docs/spiral-bevel-roadmap.md) holds the
generating-system evidence and the outstanding engagement checks.

## The lean at the mean point: an envelope the solve sees

Step 2 of #283 brings envelopes in space into the solve. The envelope of a cone or a cylinder set
(`std.Cone`, `std.ConeThrough` with its lean stated, `std.Cylinder`), or of a revolution's side,
under a motion of turns about lines, screws and `relative_to:`, is no longer only read after the
solve: a point is held to it (`M coincident flank`), and at that point a line along its normal
(`perpendicular(at: M)`), a line or a plane touching it (`tangent(at: M)`) and the circle
osculating its section by a plane (`curvature(at: M)`). The source's geometry, its number where
that is unknown and the motion's axes are columns of each, so a cutter's numbers solve from a
contact a designer states (the specification's §6.15.3).

**`mean_point.sv`** states that contact for a face mill. The mesh is the hypoid's at its mean point
reduced to its kinematics — crossed helical gears on square shafts, the pitch surfaces touching
at M and the teeth running along the sliding there — with the face mill's axis leaning `tilt` from
the pitch plane's normal. The pinion's flank is the gear's cone generated under the mesh, held to
M with its normal at the pressure angle to the pitch plane; nothing states the lean, which is
`flank.half`, an unknown. It solves to the pressure angle less the tilt, the closed form at this
mesh (20° less 5° is 15°, at every design `held_envelopes.rs` tries), the contact at the roll the
pair is drawn at; and the pinion flank's profile circle at M — the osculating circle of its section
by the normal plane — is the circle through three points of that section a micron apart (21.17 mm
at the stated design). `hypoid.sv` now finds its own face mill this way (below).

## The face mill found: balancing the undercut

`hypoid.sv` states no machine setting. The rolls are tooth counts: each member turns at the
crown's tooth count over its own (`crown_teeth / pinion_teeth`, `-crown_teeth / gear_teeth`), the
crown's being `sqrt(pinion_teeth^2 + gear_teeth^2)`. The pressure shift and the cutter's radius
are unknowns, found by `Balanced` as a designer finds them: each member's flank nearest undercut —
the pinion's, generated by the inner blade, and the gear's, by the outer — is an envelope the
solve sees, and in a section through the cutter's axis (`Margins`) the point where the mate's tip
reaches it, a module from the pitch plane, has a profile circle (`curvature(at:)`) whose radius is
how far the profile stands from the cusp undercut would make. The two members' margins are held
equal at the mean point, which sets the shift, and at the toe of the trace, which sets the
cutter. Of the cutter's two roots the larger is the one wanted (`gear.M distance(>= R) trace.C`).
As configured: 11.2527° and 75.6122 mm, the margins 15.68 mm at M and 12.76 mm at the toe. The
seeds choose among the balance's roots: from 8°–12° and 70–100 mm the solve finds this one; from
14° and 60 mm another, a 260 mm cutter. Both members export at 10 µm with no disagreement
against their fields, but at this cutter the pinion's swept cut is fitted as one face, not named
by the crown's faces.

The solve finds them by tearing (`docs/contract/solver.md`): the two numbers held, the rest of
their block — the blades, the generated flanks, the probes and their circles — is solved block by
block, and the two balances are brought to zero by Newton steps over the numbers alone, from their
seeds. The suites recorded the pair cut by a stated mill, so `fixtures::gear` states one at every
recorded design (`recorded_machine`: the shift each design names, the cutter 0.8 R) and drops the
balance; `gcs-core/tests/hypoid_machine.rs` holds the found mill.

## Contact analysis: the meshing of two flanks

Issue #283's step 3 makes the contact of a pair a locus. `meshing(pinion_flank, gear_flank, under:
mesh, from:, to:)` turns the first flank with the pinion and the second with the gear, the mesh's
observer and source. Each flank is an envelope in space or a source standing still (a formed
gear's cone). At each phase of the pinion it finds the turn of the gear at which the two share a
point and a normal: the gear's **lag** behind the ratio is the transmission error. A point held to
it owns the phase and the lag, both pinnable (`Q coincident(t == 0deg, lag == 0) contact`), so a
stated transmission error is a row of the drawing. After the solve the contact is traced over the
phase into the transmission error, the path of contact and the bearing on each flank; `solventc`
reports it (the specification's §6.15.4).

**`contact.sv`** analyses a pair at the mean point. The mesh is `mean_point.sv`'s and the gear is
formed by the face mill. The pinion is cut by a cutter `grown` times the gear's about M (the
lengthwise mismatch that localises a bearing) at a roll `roll` times the mesh's. The two cones
share the generator through M and their normal along it and nothing else, so:

- the pair touches at a point, and the bearing runs along that generator;
- the pinion is conjugate to the gear at the modified ratio, so the gear lags `(roll − 1)` times
  the phase, exactly.

```sh
build/solventc rust/examples/spiral_bevel/contact.sv
#   bearing: point contact; transmission error (the gear's angle) -0.0375° … 0.0375° over the
#   phase -15° … 15°, the path of contact 12.6455 long
```

`gcs-core/tests/meshing.rs` holds the meshing to closed forms:

- a conjugate pair transmits without error, along a line;
- a pair cut at another ratio transmits at that ratio;
- `contact.sv`'s bearing lies on the generator, at three designs.

For a cutter turned 2° about the contact normal at M, which touches the gear at M only, it holds
the meshing to a check computed apart from it. The lag at each phase is the turn of the gear that
closes the gap between the two members: the pinion's flank's nearest approach to the gear's cone,
from the cone's equation and the poses alone. The contact found by Newton is local. A pair a
little off conjugate holds its contact only weakly along the bearing, so it is found only from
near it; a failed solve seeds the contact again with the flanks where the solve left them.

## The pair stated as its meshing problem (#345)

**`meshing_problem.sv`** is `formate.sv`'s pair written as the problem a gear engineer states:
the shafts and the ratio, the blanks, and the contact. The gear is formed and the pinion is its
blank less the gear's tooth swept under the mesh, so the pinion is conjugate to the face mill's
cones, as in `formate.sv`. A conjugate pair touches along a line. Here the gear's concave flank
is **eased** off the inner cone (#345 S2) — relieved along its normal by `½ κₗ s² + ½ κₚ r²`
about the mean contact T — so the pair touches at a point and runs with a transmission error.
The designer states:

- the mean contact: T on the inner cone in the normal section through M, where the cone's
  normal is a line of action of the mesh (`normal reciprocal mesh`), the contact at phase 0;
- `pressure`, 30°, the lean of the common normal there (the inner blades' half angle);
- `path_angle`, 70°, the angle the path of contact makes there with the trace's heading
  (`rate(Q over t) parallel path`: Q's rate along the phase);
- `lag`, −0.03°, how far the pinion stands behind the ratio at the start of the phase, half a
  pinion pitch before the pair is drawn;
- no undercut: `pinion_flank regular(by: gear_tooth.crown) within pinion_blank.material`.

The ease's two curvatures are unknown, seeded and found: κₚ = −0.018005 and κₗ = −0.0028544 a
millimetre (negative: away from the mill's axis, into the gear's tooth). The meshing is read in
the gear's frame, `drive := motion(pinion_turn, relative_to: gear_turn)`, since the ease relieves
the first flank, and the phase is the pinion's. The block holding the two contacts has no
tearing at both numbers (the lag's contact is ten rows), so the solve tears it at `κₗ` alone,
the path's angle left over, and finds `κₚ` inside from the lag's contact (`solve::blocks`,
`torn`). It comes to the same ease from seeds on either side of it.

The gear is cut to the ease: its concave blade (`EasedBlade`) is an arc in the normal section,
tangent to the cone's generator at T and bent by the parabola, turned about an axis parallel to
the mill's, moved out so its circle through T bends by the parabola along the trace. The profile
radius comes to 55.56 mm and the blade's cutter radius at T to 48.55 mm (the face mill's: 41.86
mm). A surface of revolution through T with the cone's normal there, it matches the eased flank
to second order: within 0.02 µm along the whole bearing, 1.5 µm 1.25 mm up the profile and 4 mm
along the tooth, 13 µm at the tooth's corners, where the pair never touches. The backlash is all
the gear's, half of it off each flank.

```sh
build/solventc rust/examples/spiral_bevel/meshing_problem.sv
#   pair.contact: point contact; transmission error (the gear's angle) -0.03° … -5.00183e-13°
#   over the phase -7.5° … 7.5°, the path of contact 5.38159 long
for m in pinion gear; do
  build/solventc rust/examples/spiral_bevel/meshing_problem.sv --solid pair.$m.body \
    --tolerance 0.003mm --stl build/exports/meshing_problem-$m.stl --no-diagnose
done
cargo test --manifest-path rust/Cargo.toml -p gcs-cli --test pair_check meshing_problem -- --ignored --nocapture
```

The pinion is one sweep, admitted with a least area factor of 0.486 (147 faces, about 8 s); the
gear is its blank less 48 spaces, each a torus and a cone (292 faces, about 30 s). `pair_check`
reads the two STLs, poses the pinion in the gear's frame at each phase of the meshing the model
traces and holds them to it. On the pinion's convex side the clearance grows from the drawn
pose by the gap the meshing's lag leaves, within 1.0 µm of it over the phase, and the members
come nearest within 0.64 mm of the meshing's contact on the gear's flank (at the phase's ends the
next pair, a pitch on, is the nearer). On its concave side, conjugate, the clearance stays at
23.6–24.6 µm, half the backlash. `gcs-core/tests/meshing_problem.rs` records the solved ease and
reads each stated condition again apart from the relation that states it: the contact at phase 0
at T, the pressure angle of the cone's normal there, the path's angle by differences of the
traced contact, the lag at the start of the phase, a point contact throughout, and the least
area factor against the sweep's admission.

## Phillips' spatial involute gearing (#326)

**`phillips.sv`** is a pair on the same skew shafts made from its contact alone: no face mill, no
crown. The design is one line of action `L` through the pitch point. A force along `L` does no
work on the members' relative motion when the two move as fast along it: `L reciprocal mesh`,
which fixes its helix angle once its pressure angle is stated. `screw := std.Screw(isa, mesh)`
draws the mesh's instantaneous screw axis, on the shafts' common perpendicular, and `screw.pitch`
is its advance a radian of the mesh's roll. Both are the standard library's, rates of points the
mesh carries (#365): a motion's twist has no words of its own. Each member's flank is then the involute helicoid a plane
square to `L` sweeps sliding along it at that speed, seen from the member: its rack,
`std.HalfSpace(P, L, d: 0mm)` under `motion(slide, relative_to: pinion_turn)`. The two flanks
touch where the rack meets `L`, with `L` for their normal, at the ratio's roll:

```sh
build/solventc rust/examples/spiral_bevel/phillips.sv --where screw.pitch --where isa
#   bearing: point contact; transmission error (the gear's angle) 0° … 8.1e-14° over the phase
#   -15° … 15°, the path of contact 8.34989 long
#   screw.pitch = -32.1994
#   isa.px = 0, isa.py = 57.6, isa.pz = 0; isa.x = -0.894427, isa.y = 0, isa.z = -0.447214
```

Ball's pitch is −28.8 a radian of turn about the axis; the mesh turns √1.25 about it a radian of
its roll.

`gcs-core/tests/phillips.rs` holds each flank to the helicoid's own equations and the pair to a
point contact running along `L` with no lag, the screw to Ball's closed form, and `L` to the
members' equal speeds along it. A gear whose rack slides 2 % faster lags by what
closes the gap between the racks, `(σ/σ' − 1)` times the phase. And Phillips' claim holds: moved
after its flanks are made — half a millimetre further off, or its shaft turned half a degree — the
pair runs at a constant lag, so with no transmission error.

**`phillips_pure.sv`** is the same pair with no rack: each flank is stated by the contact alone.
The contact element is a line of the member's plane of action moving along `L` at the members'
speed; the member, turned by the phase, sees it carried back (`X coincident(under: turn, t ==
-phi) x`). Over the phase and a place along the element that is a surface traced in space, and it
meshes as the rack-cut pair does:

```sh
build/solventc rust/examples/spiral_bevel/phillips_pure.sv
#   bearing: point contact; transmission error (the gear's angle) 0° … 3.1e-13° over the phase
#   -15° … 15°, the path of contact 8.34989 long
#   pinion_flank: a surface traced in space, regular over its domain
#   gear_flank: a surface traced in space, regular over its domain
```

`gcs-core/tests/phillips.rs` holds each traced flank to the rack's envelope it agrees with, to
1e-9. Line contact on skew axes stated by its contact alone is a first-order PDE, marched from one
contact line: #339, item D5.
