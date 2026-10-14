# Relations: operators, chains, sides and ordinates

Part of the [contract](../../CLAUDE.md) the code is held to. Read it when you touch
`constraints::infix_op`/`prefix_op`, `Written`, chains, or add a relation word. An issue number (#n)
or a spec version ([0.n], see the [language changelog](../solvent-changelog.md)) says where a rule
came from.

- **Which way is a word, not a sign** (Solvent §9.2, §9.4; issue #48, item 4).  A distance measured
  *from a line* (`PointLineDistance`, `ParallelDistance`) is a **magnitude**: its kernel is `|g| −
  d` (`kernels::point_line_magnitude`, degree 1 — **not** the squared form, whose gradient vanishes
  at `distance(0)`), both sides are solutions, and the seed picks.  `side: left|right` pins one, and
  then the *signed* kernel runs with the word's sign. `CKind::side_words` is the one table of words
  and signs (`left`: +1 of a line, −1 along the page).  A negative magnitude is E040 **by value**,
  so `p distance(-hw) axis` is refused; where a sign is arithmetic, use a signed datum ordinate. The
  run, the rise and the directed angle keep their signs and gain `along: right|left|up|down` and
  `sense: cw|ccw`.  `io::dimension_text` draws the number the statement **makes**.  A component
  takes a side as `Ty::Side` — a word in `Scope::sides`, never a ±1 in `vals`.  `cgraph`'s PL edge
  asks `Constraint::signed_gap`: the word where pinned, the *pose* where not. `tests/refusals.rs` is
  the gate.
- **A selector says what it means, or it is refused** (Solvent §9.2). `CKind::words(slot)` is the
  vocabulary a `Str` slot takes (`at: start|end`, `at: p1|p2`) and `constraints::ALONG` the one
  table `along:` is read by (choice and message).  A key naming no slot, a word outside a slot's
  set, and a bad `along:` (e.g. `along: z`) are all E040 **at the key** (`Written::key_span`), never
  silently dropped or misread; `report::registry_json` publishes each slot's words.
  `tests/refusals.rs` is the gate.  `along:` also takes a reference (an axis, a line): see the
  ordinate below.
- **An ordinate is one kind along any direction, and `level` its zero**
  ([plan](../ordinate-plan.md), Solvent [0.46]).  `a distance(d, along: t) b` is `CKind::Ordinate`,
  `(q − p)·t̂ − d`, signed along `t`; `a level(t) b` is `CKind::Level`, no number, no callout (the
  direction positional, as `symmetry(l)`'s line).  Slots `(p, q, t: SpecKind::Along, [d], along:
  Str, form: Int)`: `t` an axis, a line, or a plane standing for its normal (only via the word
  `n`); the word (`constraints::ALONG`, `Toward`) keeps the spelling and the sign
  (`right`/`left`/`up`/`down`), and `infer_entity` reads a page word's axis off the view both points
  are drawn in (none in a planeless sketch: refused).  `form` (`OrdinateForm`: `PageU`, `PageV`,
  `InView`, `Space`, `CoordU`/`CoordV` — from a view's own origin, the point's one coordinate, the
  radius kernel's one column — and `FrameU`/`FrameV`/`FrameN` — a plane's own word from its origin
  for a point drawn elsewhere, over the plane's columns, so `v` is its frame's `v̂`, square to `u`,
  which an axis `P.v` written outright need not be) is set by `Sketch::add_quiet` from `constraints::ordinate_form` and picks the
  kernel (`Constraint::kernel`; the run and the rise keep their constant-Jacobian kernels,
  `K::OrdinateU`/`V`); `Constraint::reads_space` asks it where `CKind::spatial` would.
  The plane forms (`q distance(d, along: u) P`) are rewritten from `P.origin` along `P.u` in
  `program::relations::ordinate_operands`, which also refuses a plane as a direction, a word that
  also names a direction in scope, and a literal zero (E040, naming the level); `a horizontal b`
  is `std`'s word for `level(up)` (`use std (horizontal)`, §9.9), and the printer
  (`print::ordinate_text`) spells each case back.  `tests/ordinate.rs` is the gate.

- **`distance` relates two things; `length` measures one** (#269, Solvent 0.90).  A path's
  length is `length(d) g` for every kind of path: `prefix_op` settles `("length", Line)` to
  `Distance` between the line's own ends (`Written::assemble` expands the one operand to its
  `p1`/`p2`), and an arc, a spline and a free curve to `ArcLength`, `SplineLength` and
  `CurveLength` — so a word over `g: path` states `length(L) g` whatever it is given.  The prefix
  `distance(d) l` is retired: `constraints::retired_prefix` keeps it a prefix word to the grammar
  (`prefix_word`, so a statement or a chain link written that way still parses, and
  `builtin_word`, so no definition takes it) and `program::settle` refuses it (E040) naming
  `length(…)` and not the operand, which in a block's copy is a minted name.  The infix `a distance(d) b` is unchanged.  `tests/refusals.rs` and
  `tests/tangent_at.rs` (`length_of_any_path`) are the gate.

- **Every constraint is written as a prefix or an infix operator** (Solvent §9.2); `name(args…)`
  is retired.  `radius(25) c`, `p1 distance(80) p2`, `horizontal l`, `fix((0, 0)) p`,
  `a symmetry(l) b` — the word, its one or two operands, and everything else in the parentheses on the word: the
  number, a selector (`side: -1`, `at: start`, `along: x`), a third entity, a pin (`t == 0.4`).
  A *seed* for an owned slot stays the trailing `hint(t: 0.4)`, where every seed is.
  **A pin and a seed are one `OpArg::Slot { key: Name, arg: Arg }`** — the word is the only
  difference (as separate variants a pin lost its expression, a key naming no slot filled any
  `Param` slot, and `write_written` disagreed with `operator_text`).  The value is
  **`syntax::Arg` and not a second encoding of it**: `assemble` hands it straight on, and
  `flatten::settle_arg` is the one walk that reads a component's parameters out of an argument.
  `Written::assemble` matches the key against the spec; an unknown one is an E040 **at the key's
  own span**, so the key is a `Name`, `assemble` returns a `Result` and `settle` carries a `Span`
  beside its message, in `program.rs`'s `(Span, String)` order.  `syntax::slot_text` is the one
  place a slot is spelled, read by both printers, as `hint_of` is for the clause around it.
  **The shape is the library's**: every user-facing kind has one or two entity slots, always first
  in spec order, with `Symmetric` the single three-slot exception.  Several kinds share a word —
  **`coincident` is fifteen kinds, `distance` nine, `tangent` nine** (twins in space included; a set's uses besides) — and `horizontal`/`vertical` are
  two each with the *fixity* doing the work.  **Over an envelope in space** (#283, [0.90]) a point
  is `coincident`, and at a point held there a line or an axis `tangent` (`EnvelopeTangentAt`) or
  `perpendicular` (`EnvelopeNormalAt`), a plane `tangent` (`EnvelopePlaneAt`) and a circle or arc
  `curvature` (`EnvelopeCurvatureAt`): the parser reads `at:` after `perpendicular` and `curvature`
  as a point, as after `tangent` (`op_args`), and `settle` names the envelope first whatever was
  written first (`curve_last`, the symmetric swap).
  **What a word means is the kinds of its operands, and a name does not carry its kind until
  elaboration** — so the parser resolves *nothing*: it produces a `syntax::Written` and
  `program::settle` turns it into a `CKind` plus arguments in spec order, through
  `constraints::infix_op` / `prefix_op` and `Written::assemble`.  A **lone infix statement is a
  one-joint chain**.  `CKind::operator()` is the inverse and is matched exhaustively, so a new
  kind stops the build there; `syntax::operator_text` is the one printer, read by
  `write_relation` and `io::describe`.  **The surface word and the wire name are separate**: the
  registry goes on publishing snake_case `name`, which the binding and the JSON export key on,
  with `operator`/`fixity`/`operands` beside it — **the binding is untouched**.  `ccw`/`cw` keep a
  call (`Fixity::Call`, every operand in the parentheses): the predicate is about the *triangle*.
  **The gauges and the orientation predicates are entries of the same table** (issue #47):
  `CKind::Fix`, `Ccw`, `Cw`, read by the one relation parser and settled by the word alone
  (`constraints::gauge_op`, before the operands' kinds are asked — which numbers `fix` holds is
  the entity's, and `ccw(a, b, c)` has no operand outside its parentheses).  They are
  **applied, not added**: `program::gauges::apply_gauge` marks the parameters fixed or records the root
  choice, `constrain` returns no id, and no `Constraint` is one — so they are not in `ALL_KINDS`,
  the registry never publishes them, and `CKind::gauge` is the question every table that would
  reach for a kernel asks first.  A `claim` on one is refused (E040): a gauge adds no row.
  **A `fix` states what it holds** ([0.34]): `fix((0, 0)) p`, `fix(x == 5) p` (partial),
  `fix(r == 25) c`, `fix(half == 30deg) k`, `fix(dir == (0, 0, 1), origin == (0, 0, 0)) t` —
  pinned slots named by the entity's members (`EntKind::members`: `x`, `dir.x`, `origin.z`;
  `Fix`'s spec is their union; `apply_gauge` checks the kind's).  **A vector** ([0.45]) is `(a,
  b[, c])`: `OpArg::Vector` keeps it whole so `apply_gauge` refuses one shorter or longer than
  what it holds (a point in space is three, so `(0, 0)` cannot leave `z` free unsaid; a partial
  hold is by member); `Written::slots` expands it into member slots, and `hint_body` expands a
  seed's at parse.  Writers say each vector whole where all of it is given, else by member
  (`syntax::said`; `lift_gauge` writes a `Written` fix).  `o := (10mm, 20mm)` is a group of
  members `x`, `y` (`o.x`).  `ground`, `fix c.r` and a bare
  `fix p` are gone (`gauges::fix_spelling` refuses the others where written).  **A held number
  is its own seed**: fixes are applied once points have their places (`entities::places`), before
  `axes_along`, motions, envelopes and `settle_deferred` (`gauges::is_fix`), and
  `views::stand_axes` then stands each plane's free axes through its held origin, so no `hint`
  repeats a `fix`.  Holds and root choices are gathered (`gauges::Gauges`) and applied once
  all are in (`hold_all` after the fix pass, `choose_all` after phase 5's `branch`es): alike, one
  gauge; differing, E031 at each statement and nothing applied, so no order decides (#113).
  `commit_seeds` takes a held number out of a written clause (the whole clause
  where all are held) and writes it back when the hold goes.  `tests/seeds.rs::a_fix_is_its_own_seed`.
  `edit::reconcile` diffs holds per entity and field (`gauge_key`, `held_refs` over
  `program::holds`) and appends a statement built by `program::lift_gauge` with the numbers; a root choice under a key no triple spells stays the
  `branch(KEY, ±1)` statement (`StmtKind::Branch`).
  Operand order carries meaning — `arc tangent line` is `TangentArcLine`, `line tangent circle`
  is `TangentLineCircle` — and a name that is also an element keyword cannot lead a statement
  (`spline_follower.sv`'s spline is `cam`, not `curve`).
- A **chain** (Solvent §6.6) is parser-level sugar and nothing else: `horizontal line bottom(b1,
  b2) -> tangent arc a(center: c) hint(r: r) -> tangent …` desugars in `syntax.rs` into the
  ordinary statements it stands for — a prefix word (any `CKind` whose spec is one entity slot)
  becomes that unary relation, and a joint's word the relation between its two neighbours:
  `tangent` maps per pair of kinds, and any binary `CKind` (`perpendicular`, `equal_length`, …)
  is an infix spelling of itself, type-checked against the pair.  No other module knows chains.
  **Threading is a statement, not an inference** (issue #31): the `->` marker on a joint says
  its two links share a boundary point, threaded left-to-right (`p1 → p2`; `start → end`, CCW),
  and its absence says they do not — `->` alone is the plain corner (`to` is retired into it),
  `-> tangent` a corner also tangent there, and a bare word states the relation and welds nothing.
  A joint may state *several* relations (`-> equal angle(30deg)`), the marker on either side or
  both; the run ends at the first word opening the next link, so fixity sorts them.  A doomed word
  splices out where it stands (`Chained::Member`); for a whole joint doomed at once each member
  carries the joint's written word count and one-word doom (`of`, `fall`, `out_of`) and
  `edit::doomed_splices` composes the single splice for `remove` and `reconcile` alike, and
  `remove` refuses a doom set whose splices leave text that no longer parses.
  A trailing placement attaches to the line's **one** relation and is refused on a line stating
  several; where none is written the parser records the spot (`place_span`, an empty span), so
  callout writeback splices where the parser said and re-derives nothing.
  At a threaded joint the shared point is named by exactly one side (or both, in agreement), so a
  threaded `tangent` always desugars to the regular At-form (`TangentArcLine`/
  `TangentLineCircleAt`, or `Parallel` between lines) — never the bare pair, rank-deficient at
  every solution; `at:` is only supplied by a threaded joint, and an *unthreaded* `tangent` is the
  plain pair, correct exactly when the two are separate.  At a corner with a name-link the
  declared side names the shared point (`(t := line(p3, k.start)) -> tangent k`;
  `follow_building` resolves such a child when the entity's kind builds later).  Only lines and
  arcs are threaded.  `equal` is the second polymorphic word (`syntax::equal_kind`): length
  between lines, radius between circles/arcs; `Relation::poly` carries it and
  `program::constrain` settles it once the entities resolve, **before** reading the spec.  Each
  desugared statement keeps its own id and a span into the chain's text.
  **How a statement is spelled is recorded, never sniffed back out of the characters**:
  `Stmt::chained` is a `Chained` (`No`/`Link`/`Prefix`/`Joint`/`Infix`/`Member`/`Stuck`/`Close`)
  and `edit::doom_splice` matches on it — a doomed threaded joint steps down to `->`, a doomed
  unthreaded joint becomes a statement break (inside a closed chain there is no safe break:
  `Stuck`, refused), a doomed prefix word goes where it stands, and a link has *no* splice, which
  is how deleting one is refused.  Which slots a chain threads is `EntKind::ends()` in `model.rs`,
  exhaustive, so a new kind with ends stops the build.  A line ending in a joint continues its
  chain onto the next; `-> close` seals a loop, and a `close` with no marker is an error.
  The colouring asks `opens_link` — the *same* predicate `chain_starts` asks.  Both reach it
  through `past_args`, the lookahead **stepping over the operator's own parentheses**, returning
  an **index** (like `past_ref`) since callers ask both what word is there and whether the line
  ends there.  A bare name in an argument list (followed by `,` or `)`) stays untinted.  It runs
  per keystroke: put the pointer test before the registry lookup (`chain_kind` allocates), and
  `tint_word` computes the lookahead only in the arm that reads it.
  **A block body may end mid-joint** (issue #38): a *threaded* trailing joint at the body's `}`
  threads onto the next copy's first link — every pair in a `cycle` (`cycle 4 { (s := line) ->
  perpendicular equal }` is the square), all but the last in a `repeat`.  Recorded as
  `Block::joint` (an `OpenJoint`), its word statements minted through `joint_relation` with the
  right operand `next.<leaf>`, which `flatten::lookup`'s `next` arm resolves under the block's
  `cyc` (a `repeat` body still may not say `next`).  The weld is the flattener's (`weld`/`fill`):
  `builds_first` generalized to (kind, copy, statement), since `follow_building` refuses a reach
  into an unbuilt entity's implicit child.  At most one boundary slot may name its point; a
  name-link boundary is refused until #35.  A statement in a braced body ends at the `}`
  (`end_of_stmt`).  `square.sv`, `ngon.sv` are the cases; `tests/open_joint.rs` is the gate, and
  `tests/chain.rs` holds `rect_fillets`'s chain spelling to the shipped longhand.
- A statement expanded by `flatten` **keeps the id of the statement it came from**: the line is
  what a span points at, a caret lands on and a splice edits; the `path` every `Site` carries
  tells copies apart, and `commit_seeds` needs to see the multiplicity.  `Program::stmts` walks
  into block bodies; whether the *root* may splice a statement on its own is asked against
  `root().body` (`edit::in_root`).
- **`ring` solves one copy** (#96, the `ring` paragraph in [language.md](language.md); it was refused from #47 until it
  could).  A diagnostic carrying its own code (E041) goes
  through `Expansion::coded`, since plain `errors` are sorted into E101/E103 by message.  An
  expression's failure is an `expr::ExprError` with a `Fault` — `Dimension` (E103, §3.3:
  `distance(45deg)` is an error, never a coercion), `ClaimFree` (E040, §9.7) or `Uncomputable`
  (W110, the last number stands).  A point-to-point distance and a radius are
  `CKind::magnitude`; a negative literal in one is E040 where written.
