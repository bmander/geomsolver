# Frames: joints, riding and readings

The rules for frames that move (#383, [frames plan](../frames-plan.md)): a frame's joint, the
`in` blocks that make a declaration ride on frames, how a reading at a roll finds what it carries,
and a frame or a point read at a roll — owned or stated — with the station words written so
(#366).  A frame as a place to draw — a plane, a spatial chart, `f := frame` — is
[space.md](space.md)'s; the motion graph the readings lower to, and the generating classes that
read it, are [generating.md](generating.md)'s.

**A frame rides on the frames of the blocks around its declaration** (stage 4): `in (arm :=
frame) { in (planet := frame) { … } }` declares `planet` in `arm`'s block, so `FrameE::rides` is
`[arm]`, outermost first, set by `planes::memberships` from the declaration's membership path.
`Sketch::carriers_of(path)` is every jointed frame a path carries, each after the frames it rides
on, each once; `Sketch::carriers(e)` is a point's path's (`Sketch::rides`), a frame's own path
and itself, and anything drawn (a line, a circle, an arc, a spline, a face, a solid, a surface)
what its parts ride on, which must agree — `Err` names two parts riding on different paths, and
`motions::carriers` says it in the document's names.  A motion, an axis, a datum and a curve of
expressions ride on nothing unless `rides` says so.  `motions::frame_motion` of a frame is the
chain of its carriers' joints, so `under: planet` is `arm·planet`.  `tests/frames.rs` is the gate
(`a_planet_drawn_in_its_arm_rides_on_both`).

**`in` nests in `in`** (stage 4): the parser counts the braced bodies that are `in` blocks
(`P::in_in`), so a block inside another is no nested root block, and records every header in
`Program::in_blocks` for the editor.  `stamp_frame` stamps the innermost block first: a
declaration already in a frame — an inner block's or its own clause — rides on the outer one
(`Membership::ride_in`, `rides`, outermost first), and a frame declared in a block takes the
block's frame the same way.  An instance's frame reaches its body's declarations through
`Stamps::{plane, rides}`, each resolved where it was written (`rewrite_decl`).  The elaborator
draws a point in the **innermost frame of its path with axes** (a plane, or a spatial frame with
a chart), and the path itself is what it rides on; a frame with axes further out says nothing of
where, which is W115 — but for a plane standing at the outer frame's own place, one of its
standard datums (#542, [0.168], `Sketch::plane_at`), which is where in it a definition standing
in the frame draws.  So a body standing in `f` that draws `in std.front { … }` is drawn in `f`'s front and
rides `f` (and whatever `f` rides) by this rule alone: the datums stand at the frame's rest pose,
built over its chart and riding nothing themselves, and an instance in a frame with a joint is
carried whole at a roll with no `under:` (`tests/frame_datums.rs`,
`a_definition_in_a_frame_with_a_joint_rides_it_at_a_roll`,
`an_instance_stands_in_the_innermost_frame_which_rides_those_outside_it`).  At rest a riding point
is drawn where it stands: `Sketch::world_point` reads no joint, so no relation naming no roll sees
riding at all.  A point already riding keeps its path: drawn again on another is E060.

**A reading at a roll infers its carrier** (stage 4): where a reading leaves `under:` out, the
motion is what its tool rides on, seen from the frame `seen_from:` names (`motions::riding`,
`reading_of`), and a tool riding on nothing is refused saying so:
- `X coincident(t == τ) x`, both points: the carried point over `path(X)⁻¹·path(x)` — `x`'s
  carriers seen from `X`'s, or from `seen_from:` — with `under: #motion{n}` put on the relation
  (`relations::riding_point`); both riding alike, it is read as written;
- an envelope in the plane, a prism's side, an envelope in space (its source a drawn tool, a
  revolution's side read by its edge, `envelopes::ridden`; a set applied to its hidden point rides
  on nothing);
- a meshing: the second flank's carriers seen from the first's (`motions::meshing`), an
  envelope flank riding on its observer;
- a solid read at an instant (`tool at θ`), swept over an interval (`sweep(tool over t in (a,
  b))`) or rolled through a body (`sweep(tool, through: body)`) — over a solid that rides, a
  `solid(…)`'s `from:`/`to:` and `through:` are refused naming those (`build_solid`, #391 S2).

`under:` names the frame of an operand that rides on none: naming one for a tool drawn riding is
refused (`motions::under_riding`, over `Sketch::drawn_carriers`, its points' paths alone) — what a
reading made rides on its observer and may still be read under another frame, a generating cut
indexed `under:` its blank's turn.  **What a reading makes rides on its observer**: a profile
generated in the plane, an envelope in space and a swept solid get `rides` of the `seen_from:`
frame's path (`generated::ride_on`) — the profile a rack cuts rides on the blank.  The parser
takes an envelope's, a meshing's and a solid's `under:` as optional (`Sweep::{Placed, Swept,
Rolled}::motion` is an `Option`).  `tests/frames.rs` holds the rack and the swept tool against
the same readings written `under:` (`a_rack_riding_its_frame_cuts_what_the_reading_under_it_cuts`,
`a_tool_riding_its_frame_is_placed_and_swept_as_under_it`) and the refusals
(`riding_refuses_what_it_cannot_mean`).

**Joints in the solve** (stage 5): a joint's ratio or advance no `fix` states (`f turns(k) l`
over a declared unknown `k`, `program::motions` reading a `Tie` slot or none) stays a free
parameter of the frame's, seeded where its unknown is, and the motion node carries both as
`MotionE::numbers`.
The kind is still read off the holding statements: an axis on a line with its ratio unknown is a
turn.  `envelope::held::motion` puts the two numbers among the motion's columns past its axis's
(`held::Free`, encoded in a step's constants as where they start, one past, and whether the step
runs back), and `held::Step::numbers` reads them in place of the constants — so `CarriedPoint`,
the rates and the rate of a contact, `PointOnEnvelope` and its words, and `PointOnMeshing` hold
them as columns (`Meshing::new` reads the source's ratio off its parts).  `tests/jacobians.rs`
FD-checks every such row over the joints' columns
(`a_joints_free_numbers_agree_with_finite_differences`).  `std.Rolls(at, a, b, along: d)` holds
the rate of `at` carried by `a` seen from `b`, at roll 0, to nothing along `d` — one row, the
other parts of the rate zero by construction for parallel or meeting shafts with `at` on the line
of centres.  `tests/frames.rs` is the gate: a pair's ratio solved from rolling against `−R/r`,
the centre distance solved from a stated ratio, a joint nothing holds reported free with its six
numbers.  A joint stating no number is no longer refused (E080 through 0.136).

**A frame that moves as a number runs is a frame read at a roll** (#366, [0.138]) — one notion,
wherever a frame moves.  `X coincident x at τ` ([0.140]; `X coincident(at: τ) x` through 0.139) is
`x` carried to the roll `τ` by what it rides on (seen from what `X` rides on, or from `seen_from:`),
or by the frame `under: m` names where `x` rides on none — the same reading as `t == τ`, one kind,
`CKind::CarriedPoint`, whose roll slot is read two ways (`Constraint::stated_roll`): **owned**, `t
== τ`, the relation's own unknown in degrees, a column, pinned or solved, which a rate runs over;
**stated**, `at: τ`, the text the statement wrote, worked out as a dimension is
(`Constraint::arg_kind` an angle, `expr::evaluate`), no unknown (`Sketch::add` allocates none), so
it may be written over a face family's running number and is read again at every station, and a free
variable it names is its twin's last column (`envelope::screw::Reading::At`, constants `[free, m, c,
steps…]`, the roll `m·a + c`).  Saying both is refused (`Written::assemble`).  At a stated roll a
point drawn in a plane is held where that plane sees it, two rows along the plane's axes
(`CarriedPointIn`, `program::reading`, as a relation in space has its kind in a view) — as many rows
as it has numbers.  A roll read where nothing rides and no frame is named is refused saying so
(`relations::riding_point`).

**A frame carried** is `n := base at τ` ([0.140]; `syntax::Decl::carried`, `at(under: m, seen_from:
g)` as on a point), the reading at an instant a frame's source makes of it — a solid's is a solid
placed, a point's a point held ([parameters.md](parameters.md)): the flattener writes it out where
it walks the body (`flatten::carried`) — three points drawn in `base` and held there (its origin and
ten along each axis), riding as `base` does; each carried to `τ` into a point riding on nothing; the
plane over the lines from the carried origin — so its carrier is found as a point's is, its `u`, `v`
and normal are `base`'s carried, and the source prints the one statement.  `under:` on a `base` that
rides is refused (`motions::under_riding`).  Before the solve each joint held to an axis is seeded
on it and each point carried to a stated roll where its motion carries it, round by round with the
planes placed over them (`Sketch::seed_joints`, `Sketch::seed_carried`, `program::place_planes`), so
a plane drawn over carried points stands along its lines however far round the roll is.

**A joint about a frame's own axis** (#366): `f turns(r) P.normal`, or `P.u`, holds
`f.axis` coincident with an axis no drawn line is — `Joint { axis }` — which the held steps read as
a turn (a slide, its ratio stated 0) about the joint's axis as the solve places it, its point and
its direction (`held::Step::Rotation { dir: true }`, codes 5 and 6), seeded where the axis it is
held to stands.  **The station words are frames whose value is written over `s`** (`std.sv`,
#391 S3 — a joint's value and `n := x in f`, drawn in a jointed frame, are
[parameters.md](parameters.md)'s): `Helix`'s screw about `base.normal` (`advance: pitch`) stands
at `360deg · turns · s`, `n` is `base` drawn in it, and `axial` is `n` drawn in a quarter turn
about its `u`; `OnHelix` is `base` drawn in a screw about `base.u` standing at the helix's start,
then in the helix's screw; `Twisted`'s point is `reach` along `n.u` drawn in a turn in `n` about
its origin standing at `twist · s` (`spins`, `CarriedPointIn`) — no `cos`/`sin` of a running
number and no geometry drawn to hold an axis.  Each lowers to the reading at an instant it was
written as through [0.140], row for row.  A family a station so carried sweeps is
its motion's sweep ([generating.md](generating.md)).  The gates: `tests/carried_frame.rs` (frames,
points and the station words against `Family::at` and their closed forms to `EXACT`, a roll owned
and stated read alike, a frame carried by what its source rides on), `jacobians.rs`'s
`carried_at_rows_agree_with_finite_differences` (a joint about a plane's normal among them).

**A frame driven by a parameter** (#391 S5): a jointed frame whose value reads a declared
parameter stands where its joint carries it at the law's value of the roll, for every reading of
its motion over the roll — the joint's node and a `MotionDef::Driven` beside it, `x in f` reading
the one and every other reading the other; a relation the solve holds reads a drive only as a
point carried to a roll and its rate, the law carried in the row's constants (#398 G,
[parameters.md](parameters.md)).

**Drawn in a frame with a value** (#398 B, [0.145]): what a valued frame's `in` block draws
stands where the value puts it — the planes it draws in read as those planes drawn in the frame
(`flatten::valued`, a twin `P in f` per plane), so it rides on the valued frame no more; the rules
are [parameters.md](parameters.md)'s.  A frame declared in such a block still rides on it and is
refused (`motions::values`).

**A frame along a path** (#398 C, [0.145]): a joint held along a path, `g coincident f.axis`
(`std.Along`, `std.AlongFrenet`), is no turn about a line but `MotionDef::Guided`, its pose read
off the path after the solve (`motion::Guide`) — the rotation-minimising frame for ratio 0, the
path's own for ratio 1, from where the frame is drawn.  What is drawn in it rides on it, and a face
so drawn is swept as a declared parameter runs; a row reads no such motion.  The rules are
[parameters.md](parameters.md)'s.

**Not built in stage 4**: the app offers only planar frames as places to draw (a spatial chart is
drawn in text), and the printer writes no `in` blocks of its own, so the plan's "jointed frame
outermost" is the author's order.
