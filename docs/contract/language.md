# Solvent: names, seeds, components, modules, words and blocks

Part of the [contract](../../CLAUDE.md) the code is held to. Read it when you touch `syntax/`,
`flatten/`, `program/`, `modules.rs`, `style.rs`, or write `.sv`. An issue number (#n) or a spec
version ([0.n], see the [language changelog](../solvent-changelog.md)) says where a rule came from.

**Model/drawing split (0.20):** `.sv` is geometry, constraints, hints and assertions only;
presentation belongs in `.svd` ([docs/solvent-drawing.md](../solvent-drawing.md)). This
supersedes older presentation instructions below: never write `class`, `style`, `view`, `section`,
`dimensions` or callout placements in model source. Model serialization and reconciliation omit
presentation. The drawing compiler and renderer stay in the core; hosts supply texts. Browser
examples are file or directory entries in `app/example-catalog.ts`, loaded via `app/remote.ts`;
V-twin is `rust/examples/vtwin/` (shared modules in `components/`). `.svd` shows the paper preview,
`.sv` the model canvas; `app/program.ts` keeps each file's source across switches; the `file` URL
parameter keeps the selected file. Every parsed file has an anonymous root; component definitions
are not instances. A file-level `preview { … }` adds root statements when opened as a model (also
via `.svd`) and is omitted entirely on `use` (units and params included). Each V-twin part's setup
is its component's `preview` block (the plate's in `vtwin/components/frame.sv`); only the assembly
has a top-level `.sv`. Source edits add geometry inside the preview, reusable definitions outside.
Hosts look for modules beside the model, then its ancestors, then the library.
`web/tools/copy-examples.js` packages sources for static hosts; live files override them.

**Nothing is imported bare unless the `use` names it (Solvent §14.4, [0.30], [0.48], #94):** a
used module's component, relation word, param or group is written by full path —
`engine.parts.Rod(…)`, `hardware.nut14_af`, `components.dims.vtwin_dims` — only through the
file's own `use`, or bare where it lists the name: `use std (horizontal, vertical)` (`Use::names`;
`Module::uses` keeps each module's `Use`s). `Program::imported` is the one lookup;
`resolve_component`/`resolve_word` consult it after the file's own; `Walk::imported_params` puts
values and groups under their bare names at a root (a group as an alias of the module's). Per
file, not transitive, no prelude. `program/words.rs` checks every import as written: undefined
E101, twice / two modules / over the file's own top-level name / over a built-in word E071; a
module names its own bare.
`Program::resolve_component(name, from)` resolves from the calling file (`Scope::module`);
`component_id` is program-wide identity (keys `CurveDef`s). `Walk::used_params` qualifies module
numbers (`module_params` keeps the module's own); groups register under their path.

**Relation words (§9.9, #94):** `a horizontal b := a level(up) b`, `flat(d) l := …`, `a above(d)
b := b distance(d, along: up) a` at a file's top level (`syntax::WordDef`, `Program::words`,
parsed by `parser/definitions.rs`: `word_definition_ahead`, `defined_prefix_ahead` before a call,
`word_args` keeping labelled texts). The flattener applies a use (`flatten/words.rs`,
`Walk::apply_word`, over `flatten/apply.rs`, #103): params by label (E004/E040), a number
parameter bound in the body's scope, a word parameter where a word stands, an **entity parameter**
(`Param::Entity`, read where an entity stands: `n square(up: std.z) g`, #250) aliased as an operand
is — its text read as a reference at the use (`syntax::reference_at`), a direction word put in
place unless a name there is spelt so (`names_here`: a formal `up` given on) — and one only passed
whole to another word (`Param::Passed`) decided by what it is given, operands aliased,
every relation's span moved to the use's word (a parameter's dimension to its argument, so
`written` and `edit::set_dimension`'s `word_dimension` read it); nested words resolve from the
definition's file, a cycle is E003, an unknown word E102 naming the import. `Relation::word` (`Worded`) rides to
`Constraint::word` (`WordUse`, entities; `graft` keeps it while its operands survive) and
`io::describe_with` reads it (`p horizontal q`). `flatten::word_faults` (closure E101, a
reserved word or a builtin of the fixity E071 — `constraints::builtin_word` — names twice E001)
is asked of every definition (`program/words.rs`) and blocks its expansion. A body is one relation
or several with their declarations (#103); none is defined in a body. `horizontal`/`vertical` between points live in `rust/lib/std.sv`; the printer
writes `level(up)` (`print::ordinate_text`). `tests/relation_words.rs` is the gate. E071 is a
component defined twice in one file, the document's own included (a value twice is E001). `std.front` needs the document's own `use std`. A drawn callout drops a
module path (`relations::unqualified`).

**Predicates, applied (§9.9, §6.21, 0.50, #103):** a relation word and a set are one predicate —
parameters given at the call, **bound variables** (a word's operands, a set's point) filled at
the use — applied by one routine, `flatten/apply.rs`: `begin` (an `Application`: the closure
under the use's prefix `{scope}#{stmt}.0.`, and where its output starts), `bind_to_use` (each
bound variable an alias of the use's operand; one naming nothing is E101 once at the operand,
`use_aliases`/`failed` in `resolve`), `inherited_twin` (differentiated already where the use is),
`apply` (a dimension that is a parameter put at its argument for `written` and
`edit::word_dimension`, every other at an empty span; the body walked per `walks`; every relation
the application made stamped with the use's id/span/path, claim, placement when it makes one
relation, word spans at the use, and `Worded`, the outermost use's).  `syntax::Relation::of`
builds a relation with nothing but its form.  Words: `WordDef::body` is `Vec<Stmt>` (one relation, or `{ … }` of relations and
declarations), applied synchronously in the walk's Relation arm (`Walk::apply_word`; numbers
bound in the closure's `vals`, selector words substituted; `applying_words` for E003); sets:
`expand_use`.  A set may be written in place on the right of `coincident`/`tangent`
(`P::inline_set_use`, hoisted as a `Chained::Link` statement keyed `#s…`/`#i…`, described by its
text).  The lexer reads a brace after `:=` as a group only before `}` or `name:` (`group_ahead`).
`tests/applied.rs` is the gate.

**`ring N about C { … }`, solved over one copy (0.47, #96):** copies that are turns of the first
(the representative) about a point (in its view) or an axis held in its direction (in space).
The flattener expands it as a `cycle` (`BlockKind::Ring`, `Block::about`, `InBlock::ring`) and marks
every statement of copy k ≥ 1 `ir::Statement::turned` (`Site::turned`): its declarations are
built, its relations, holds, claims and branches are not (`program.rs`'s `stating`).
`program/rings.rs` pairs each turned entity with the representative's by key
(`prefix#id.k.rest` ↔ `prefix#id.0.rest`) into `Sketch::turns` (`model/turns.rs`): a point's,
circle's or arc's numbers are then **derived** (`Sketch::derived`, linear in the
representative's and the centre's params), owning no column — `System::col_of` is `-2 - k` for
one, its Jacobian folded into its bases by weights (`ent_w`, 1 elsewhere so every other bit
stands), its value settled in `apply_z`/`full_x`, at `expr::sync_free` and as each seed is written
(`settle_deferred`'s `Seeds`, so a seed reads a copy turned, #225); a turned arc's
intrinsic rows are not compiled; a turned curve evaluates as its representative, turned
(`curve_turn`). Readers outside `System` are told: `cgraph` leaves rows on copies to the
numeric residual, `decompose` writes none, the diagnosis reads a copy as free as its bases,
`Part`/`graft`/JSON (`"turns"`)/`topology_key` carry turns. E015 the index read in the body
(parser finds the reads, `Block::index_reads`), E021 a reference the turn would move or a copy
by index (`judge_rings`; an instance's entity argument or a use's operand is judged at the call as
written, `next.u` a turn, and a body reading the formal is not judged again: `WordState::arguments`,
#226), E022 a ring in a ring, E023
what a ring cannot turn (`rings.rs`). The ring's own turn is gauged outside (`hub horizontal
tip[0]`). `std.Polygon`, `gear.sv`, `gear_trace.sv`, `ngon.sv` and the Wankel rotor are rings;
`tests/ring.rs` and `tests/turns.rs` are the gates.

**`when COND { … }`, a block made or not (§12.9, 0.72, #210):** `BlockKind::When`, the condition
in `Block::count` (`head` its span). The flattener decides it over the scope's `vals`
(`flatten::values::condition`: E101 a name nothing gives a number, E040 one that is or reads an
unknown, E103 two dimensions, E107 a measurement) and walks the body once in the enclosing scope
with its own copy of `vals` — no `PathStep`, no prefix, so names are the scope's (`q`, `t.tip`).
`expr::Cond` (`Cmp` `<`…`!=`, `and`/`or`/`not`) is made only by `expr::parse_condition_in` and
read by `expr::decide`; no `Ast` holds one, and its sides agree as `+`'s (`> 0mm`, never `> 0`).
The parser refuses `as` and a trailing joint; the measured-bound sniff skips a statement an
opener word starts. `commit_seeds` writes into a root `when`'s body. The hypoid's tip relief is
one (`hypoid.sv`). `tests/when.rs` is the gate.

**`when` is the one conditional (0.91, #267).** Two names that are not numbers compare by
**identity**: `a == b`/`a != b` (`Cond::Cmp` over two `Ast::Var`, a side not in `vals`) holds as
`Walk::stands_for` resolves them — the absolute name through every alias, so a formal is what
its call gave and a face repeat's binder its face — and `c in B` (`Cond::In`, made by
`parse_condition_in` only) as `Walk::faces_named` lists `B`'s faces. The walk answers through
`values::Names` (`expand::Asked`), building the alias table only if a condition asks. A number
against something drawn is E103, something drawn compared as a number E103, `in` over what
names no faces E103, a name standing for nothing E101; `expr::decide` refuses `Cond::In`. So a
block over faces has no filter of its own: `repeat b in B if …` is a syntax error naming
`repeat b in B { when b != a { … } }` (`std.sv`'s `round`, `chamfer`), and a switch is `when
flag == 1`, never `repeat flag` — the count of zero or one named its copies (`sm[0]`), a `when`
keeps the name (`sm`).

**`rate(x over s)` and `initially { … }` (§6.5 [0.119], §6.9 [0.116], #339 D2, D3):** both
reserved (`syntax::words::OPENERS`).  A rate is an **operand** of a relation (`P::operand`,
`rate_ahead`), recorded as `syntax::RateOf { op, over }` on the `Written` — the reference stays a
plain `Ref`, so every rewrite of operands carries it, two rates fit one relation and an option
fits a rate — and printed back as written (`write_written`); `Relation::reads_rate` (syntax and
`ir`) asks whether one is.  The formal is always named (a syntax error without `over`), and the
word must read a direction, `parser::relations::RATE_WORDS` (`parallel`, `perpendicular`,
`angle`, `distance`): any other is a syntax error.  A rate alone before an infix word
(`rate(p over s) distance(1, along: l)`) is a one-operand `Infix`; `rate(` never starts a
component call.  The flattener settles each rate's formal in `settle_relation`
(`Walk::rate_over`), through the numbers in force: given a number it is a drawn instance at its
home, where a rate is no row, and the relation is dropped; given a face family's running number,
or a nonzero multiple of it, it is that held name; where a component is expanded to be traced
(`sym`) a formal handed on as itself is the caller's (`Drag(…, u: u)`) and one given an
expression in it E103; anything else, or a name no formal is, E040.  It passes no plane
membership (`drawn_incidences`, `planes::incidences`).  In a trace the lowering takes it
(`program::curves`, `program::traced`, curves.md's "Rates along a running formal"); on the sheet
`program::relations::rate_row` keeps a face family's in `Sketch::rates` (one rate, of the first
operand, before `parallel` or `perpendicular`, else E040; E040 a rate of anything but an axis or
a point, or against anything but an axis, a line or a plane) and states no constraint (solids.md).
`initially { … }` is hoisted as an `in` body is (`parser::statements::initially`), each relation
in it, through a `when`, marked `Relation::initially` (anything else in it a syntax error); a
defined word's body takes the flag as it takes `claim`, and it reaches `ir::Relation::initially`
and `Constraint::initially`: on the sheet a row of the drawn home, in a trace the home's alone.
`tests/rates.rs` (a face family's) and `tests/traced_rates.rs` (a traced body's) are the gates.

**A component may instantiate itself** (§8, 0.75): a call back into one still being expanded
is walked like any other (`Walk::instantiating` keeps each with the numbers it was given), each
level an instance under the last (`d.s.s.m`), ended by a `when` over its numbers. E003 at the
call that closes the cycle is only what never ends: the same numbers again (`sub_vals` equal, so
every `when` comes out alike), or nesting past `MAX_DEPTH`; one branching at every level
meets the instances' cap (`MAX_FLAT`, E103), since a body of calls emits no statement.
`tests/recursion.rs` is the gate
(Penrose's P3 by Robinson triangles, two components each calling the other, held to the
subdivision worked out directly).

**Interfaces (§8.1, 0.84):** `interface Gear { O: point, M: point, pitch: Length }` at a file's
top level (`syntax::Interface`, `Program::interfaces`, `parser/statements.rs::interface`; linked
from modules as words are; `resolve_interface` shares `resolve_component`'s `resolve_named`). A
formal typed by one (`gear: Gear`) is `Ty::Group` with `Formal::interface` set.
`flatten/interfaces.rs` resolves every interface once per walk (`shapes`, a cycle read as an
untyped group) and `bind` makes the formal **a group of its own** (`Walk::typed`: its key in
`names`/`group_names`/`typed`, each declared entity, set or group member an alias of the
argument's, a nested interface a group again; the numbers by `interface_numbers`, each of its
dimension), the whole argument kept under `{key}#given` for `conform` (from `resolve`: members
present, sets and groups of their sort; an entity's kind by `relations::kinds` via
`Expansion::kinds`, as a `path` formal's). So an undeclared member is no name: `Lookup::find`
refuses a field left past a typed group (`Scope::typed`, given with `access`), `Lookup::missing`
names the interface, and nothing handed on carries more. A member missing at the call is `failed`, its
reads silenced (`read_through`); a relation reading one is not stated. Text faults are
`interfaces::check` (E071/E001/E101/E003). `hypoid.sv`'s `Gear`, `Trace`, `BlankDesign`;
`tests/interfaces.rs` is the gate.

**Closed components:** model dependencies enter through arguments, standard datums included.
Definitions and built-ins stay callable. A component scope holds only its formals and
declarations; repetitions share it. `dims := {width: 20mm, origin: o}` bundles values and
geometry aliases; `dims: group` is a required formal. Groups nest, by name or in place
(`{bar: {at: o}}`, `InstVal::Group`, flattened to dotted members by `members_of`, each nested
group registered so `Bar(design.bar)` binds); an instance may pass as a layout group. A **`set`
formal** (`Ty::Set`, #191) takes a set — named, or a family's instance, which is both a set and an
instance — and a `group` formal no named set; each refusal is E103 at the argument
(`Checks::groups`, checked in `resolve` against `sets` / `group_names`). Only a number may be
left out of a call ([0.95]): an entity, path, group or set formal left out is E103 at the call
(`Walk::bind`) — but for **a point formal drawn in a plane formal**, `center: point in on`
(`Formal::within`), which the instance then owns: `Walk::owned_points` declares it as its body
would, `center := point in on`, under the call's own statement (so the call deletes and
describes it), in the formal's plane whatever plane the instance stands in, seeded by
`center: hint((x, y))` (`seed_point`), which `edit::commit_seeds` writes back
(`owned_splice`). A `within` naming no `plane` formal is E103 at the call (`check_call`); `in` on
another type, or on an interface's member, does not parse. `tests/sets.rs` is the gate
(`a_family_owns_a_point_formal_left_out`). **An
instance's numbers are read by its name**: `Walk::instance_numbers` puts every numeric formal
of every instance a body writes into its `vals` as `inst.formal` (the number bound, else the
unknown `bind` names), before the statements, so `p distance(ball.r) ball.center` reads it and a
group argument carries it (`bind`'s group arm copies `{actual}.…`); body values are not. No
solver state; member units survive substitution; missing members are errors.
Curves need fixed scalar/entity formals. The V-twin and inline-four pass
`components.dims.vtwin_dims` / `engine.dims.engine_dims` explicitly; `Frame(layout, dims: …)`
takes a layout naming the front datum, origin and reference axis.

- **A name is defined one way, `NAME := VALUE`** ([plan](../definitions-plan.md), Solvent §5,
  [0.29]).  `w := 100` is a value (`StmtKind::Param`), `c := circle(…)` a declaration, `t :=
  Tooth(…)` an instance, `dims := {…}` a group (a brace after `:=` is a list, lexed across lines),
  `profile := (ab := line(a, b)) -> line -> close` a chain whose link is named in place, `k :=
  leg.toe over u in (a, b)` a curve (`S := C(…).p over (phi in (a, b), s in (c, d))`, two formals
  running, a surface traced in space, #326 — `CurveSpec::second`, parsed by `curve_spec` after an
  `over (`, printed by `write_curve_spec`, its ends settled by the flattener with the first's),
  `p := point(x: e, y: e)` a computed point.  A dimension's
  number defines no name (named dimensions are gone, [0.41]).  `:=` is `Tok::Define`; a lone `=` is
  no token.  `:=` binds loosest, so a link is named in parentheses; with no `->` in the statement
  the name goes to the one declaration (`l := horizontal line(a, b)`: a prefix word's value is its
  operand).  `P::definition` lowers to the existing `StmtKind`s (`Param`, `Group`, `Instance`,
  `Decl`, `Chain`), so nothing below the parser knows; `syntax::words::named_link_at` tells `(l :=
  line)` from an operator's own parentheses.  `label:` never defines: it fills a slot.  A name in a
  child slot (`line(a, q := hint(…))`) is not implemented yet.
- **Every seed is written in one `hint(…)` clause, and nothing else is** (Solvent §4.3, §6.4): `p :=
  point hint((0, 0))`, `c := circle(center: o) hint(r: 25)`, `point_on_spline(p, s) hint(t:
  0.4)`.  Keys in any order, an omitted coordinate is 0 — an omitted *radius* is computed from the
  geometry (`UNSEEDED_RADIUS` where it gives none; 0 is stationary for on-circle rows); the clause
  sits in the trailer loop beside `knots`, `class`.  **The brackets after the name are what the
  thing is made of; the `hint(…)` is where the solve begins.**  §4.3's rule is lexical: *a number
  inside a `hint(…)` is a seed, and every other number is not*.  The four retired spellings (`at (0,
  0)`, `hint at (0, 0)`, a scalar in a constructor arg, and `hint at REF [bearing (…)]`) do **not**
  parse, and each errors saying where the number belongs.
  **A place is two keys of the same clause** (issue #47, item 2): `point b hint(at: orbit, bearing:
  u + f.angle)`, `point p hint(at: pin)`. `hint_body` reads `at:` as a reference (`Hint::place`),
  and the declaration's trailer loop is the one table that takes it, into `Decl::seed_at` (an
  `AtRef`); a clause with `at:` and a scalar, or `bearing:` without `at:`, is refused at the key.
  Both lower to the coordinate spelling's tapes — no second grammar or `bearing` keyword.
  A **pin** stays in the argument list — `point_on_spline(p, s, t == 0.4)` — because `hint` marks
  what a solve revises and a pin is what it does not.  **What `hint` marks is that a solve revises
  the number, not that the number is seed-class** — so a **callout placement keeps its bare `at`**:
  inert, but nothing in the solve path writes one (`callout::drag`/`callout::reset` are a person
  acting).
- **A seed may read geometry, and reads its seed** (Solvent §6.4).  `hint((k.center.x + k.r,
  pin.y))`, `hint(at: k, bearing: b)`, on the sheet and in a child slot. The flattener settles a seed
  text over scope parameters; failing that, a text naming a dotted scalar is kept (`settle_seed`,
  `reads_geometry`) with every dotted name resolved to its absolute name in `Decl::seed_names`
  (`rescope_seeds`, via `lookup`; never rewritten into the text); `program::build` records each as a
  `Deferred` and `settle_deferred` works them out after every kind is built, in statement order
  (`seed_read`, `place_of`).  A read is a `Length` where the document names a unit, else a bare
  number (`seed_eval`).  A `param` may not read geometry, and `commit_seeds` never writes an
  expression back, so P3 holds.  A sheet `hint(at: …)` bearing is `substitute`d over the scope's
  numbers, printed **with their unit** (`of_vals`: `(180deg)`, `(150mm)`).
  **A place may be a step** ([0.33]): `hint(at: a, toward: b, by: f, turn: θ)`, or `along: l` for
  `toward:` (`AtRef::{toward, along, by, turn}`, `entities::place_of`), not coordinate arithmetic. A
  place in another view is read in space and projected into the seeded point's (`seed_in`), so
  `program::build` settles seeds a second time after `solve_planes` when one crosses
  (`crosses_views`), and only then.  An unwritten arc radius settles with them (`Deferred::Radius`).
  A traced point refuses a step.  Components are closed over their arguments: only their own
  parameters and formals are available.  An entity argument naming nothing is said at the
  argument (`check_aliases`; `unseen` names the component that cannot see it and the formal it
  was for, #253), and not where the callee reads the formal: a relation reading it is not
  stated, and a declaration's reference to it is `Expansion::quiet`, which the elaborator's E101
  there yields to. `module_params` exports values to importing root bodies, to
  be passed explicitly. `tests/seeds.rs` and `tests/closed_scopes.rs` are the gates.
- **A part is one component carrying `in view { … }` blocks** (Solvent §6.7, `P::in_comp`): the
  block form is allowed inside a component body (the plane is a formal) and still refused inside a
  root block.  With `when flag == 1 { … }` over a 0/1 `Int` formal for the views an instance does
  not show in, a part's whole design is one module (`engine/block.sv`, `engine/head.sv`,
  `engine/crankshaft.sv`, `engine/conrod.sv`); view modules hold only what the assembly adds.
  Instances inside a block copy are indexed like declarations (`cyl[0].small`, `copy_of`).

- **A name declared over a built-in is said** (Solvent §3.3, §5; `program::shadowing`, W112).
  `expr::eval` knows `expr::CONSTANTS` and `FUNCTIONS` before the document and
  `flatten::substitute_with` only the document, so a value, input, formal or block index called
  `tau` reads differently in the two: the **warning**. Asked of the text (every component,
  instantiated or not, and every module body).  `expr::builtin` is the one table; `tests/names.rs`
  is the gate.
- **A call is the entities by position and the numbers by label** (Solvent §4.1,
  `flatten::check_call`).  `Cylinder(swing, side, top, piv, rod, across, dir: dir, fw: fw, o_s: o_s,
  o_t: o_t)`: an argument bound by position must fill an *entity* formal and must stand before every
  label; either mistake is **E004** at the argument.  Asked of the **text**: once per written call,
  before anything is bound.  `tests/components.rs` is the gate.
- **Modules** (Solvent §14.4, `modules.rs`, `library.rs`).  `use engine.parts` is parsed into
  `Program::uses`; `modules::link(prog, resolver)` resolves each once, transitively, via
  `syntax::parse_from(text, base, first_id)` — **every span is one integer into one virtual text**:
  the document, then each module after a one-byte gap (a splice, root body only, never meets a
  module span).  Module statements are numbered from `modules::MODULE_STMTS` (2³⁰), so a statement
  appended to the document never renumbers the library's ones the source map names.
  `Program::source_at` maps an offset to its text; `modules::localize` shows a
  module's diagnostic at its `use` (`Module::via`) with `name:line:col` in front.  A module
  contributes its components (`Component::module` says which) and its top-level params and groups,
  each reached by the module's full path (`engine.parts.Rod`, [0.30]); its own drawing is not
  drawn.  **The core has no filesystem**: the resolver is the host's — `solventc` reads
  `engine/parts.sv` beside the document, then `library::resolve`; the FFI and `examples::document`
  use `library::parse_linked`, over `library::MODULES`.  **`rust/lib/` is the standard library**
  (`std`: `std.StandardDatums`, `std.Turned`), in the Makefile's `RUST_SRC`.  `program::reparse` relinks from the
  texts in hand (`modules::relink`), so an edit never asks the host.  E070 no module, E071 defined
  twice in one file.  `tests/modules.rs` is the gate.
- **`port` is retired** (issue #47, item 1).  Everything an instance makes is reached by its dotted
  name (`five.s[0].p1`).  The **computed point** is `p := point(x: xexpr, y: yexpr)`
  (`Decl::computed`), refused on the sheet by the flattener and compiled to two tapes when traced.
  The parser keeps the word in `OPENERS` only to refuse it.  Aliasing is untouched: it is argument
  passing (`bind_instance`).
- **A class stands on a relation and on an instance, and `display: none` hides** (Solvent §13.2).
  `Relation::class` is parsed in both trailing-clause loops (a chain's and a lone relation's, before
  `at`) into `Constraint::class`, through `io::dumps`, `from_json`, `graft` and `write_relation`,
  and **not** in the binding's record.  `callout::style_of` resolves `.dimension` (`.reference` for
  a claim), and `layout` skips one not `shown()`. `Instance::class` (`Stamps::class`) is stamped
  **over** each emitted declaration's and relation's own (`stamp_scope_plane`), the way `in` is.
  `Style::display` is `display: none | inline | geometry` (`style::Display`): `shown()` is what an
  entity asks, `dimensioned()` what `callout::layout` asks; `geometry` is drawn but never
  dimensioned (`style .phantom { …; display: geometry }`).  `svg::render` and `paint.ts` skip what
  is not shown, and every point is drawn under the implicit class `.point`
  (`EntKind::implicit_class`, `styleNamed('point')`).  The idiom for a dense drawing is `style
  .dimension { display: none }` and `class shown` on the few to draw.
- **A declaration need not name its children** (Solvent §6.1, §6.2).  `l := line` mints two points,
  `c := circle` one, `a := arc` three; a child slot may hold a `hint(…)` instead of a reference
  (`alt_a := line(A, hint((15, 5)))`).  `Decl::children` is `Vec<Vec<Kid>>` — a name *or* a
  seed; "anonymous and unseeded" is an *empty slot*: an **implicit child**, minted by
  `program::build` (`(l1 := line) -> (l2 := line)` is three points).  E103 refuses only a list with
  *more* children than slots.  A joint threads a *name*, so a seeded slot reads as unfilled there;
  where neither side names it, `thread` mints the earlier-built side's dotted boundary (`l1.p2`),
  refusing only a name-link whose kind has no boundary field.
  **The dotted path is the name.**  `program::build` mints an anonymous child *with* `l.p1`, binds
  it in `map.names`, and records it against the parent's statement, so it resolves, constrains,
  drags, picks and survives re-elaboration (`Document.entity`).  Its seed lives in the *parent's*
  statement, so `commit_seeds` splices inside `Kid::Hint`'s spans — and where the source wrote no
  list, writes the whole argument list at `hint_span` in one edit (two splices at one offset race).
  **The element's own name is optional too**: `line`, `line(p1, p2)`, `circle hint(r: 25)` and
  `arc(center: c)` are anonymous forms, and a name is written *before* the value (`l := line(p1,
  p2)`), never after the keyword. An anonymous declaration carries a `Decl::name` key the source
  cannot write — `#a` and its keyword's offset — with an **empty span where `name := ` would go**
  (`hint_span`'s idiom); in a chain `Decl::mint_close` says where the `)` of `(name := …)` goes.
  **A name is three questions, each known where the name is minted — never sniffed back out of the
  characters**: does it **resolve**, does the source **call the thing that** (shown, selected by),
  and may a statement be **written** with it.  `Decl::name` is a **`DeclName`** fusing the answer:
  `Written(Name)` (`l0`, `s1.p0`), `Copy(Name)` (`#3.0.p`, a block copy) and `Key(Name)` (`#a41`);
  accessors `key()`, `shown()`, `written()`, `span()`.  Stamped by the parser and by **the
  flattener, which knows whether a prefix is an instance's name or a block's id** (`InBlock::copies`,
  `DeclName::prefixed`).  `SourceMap::bind` (vocabulary `DeclName::named`) files each name into
  `by_name` always, `names` when `shown()`, `writable` when `writable()`.  So `map.names` is "what
  the source calls the thing" (`name_of` is `names.first()`); `writable_name` is `edit::reconcile`'s
  gate, refusing a gesture on a block copy *with the cause*.  `syntax::hidden` survives only for a
  thread-filled slot's `Kid::Ref`.
  The key (what a corner welds by, what `res` resolves) must never reach the source: `write_decl`
  omits it, `commit_seeds` leaves a thread-filled slot *empty* — forcing labels on kept children
  after a gap (`decl_args`) — and diagnostics spell the kind.
  `edit::reconcile` **mints on demand**: when an appended statement must reference an anonymous
  element (a constraint, a gauge on a fixed point — `held_refs` is the one walk `gauges` shares), a
  real name is spliced in at that empty span, **every entity the statement made** renamed with it (a
  child by the path `program::child_names` gives its *position*) and `bind`ed `Named::Written`.
  `SourceMap::ents_made_by` (build order) is shared by `commit_seeds` and `reconcile`.  The guards
  are one question, "does the source call this anything": named, no root statement to name it on
  (refused with the cause), named since the map was made, or mint.  Insertions racing for one offset
  are ordered by `splice`'s stable sort, so reconcile pushes appends before flags before names;
  `tests/anonymous.rs` is the gate.
  Where an unseeded point *starts* (an implicit child, `a := point` with no `hint(…)`) is
  `program::scatter`, an implementation choice the spec must not carry — never the origin (a
  zero-length line, a `distance` at a stationary point), and minted points may not pile up or seed
  a self-crossing contour (a collapsed side satisfies every direction constraint).  `scatter` walks
  the bearing a fixed irrational step per minted point in creation order (a chain's traversal
  order: a simple polygon).
  "No clause" is the empty `hint_span` (a lifted declaration has `None`), and `commit_seeds` writes
  the solved pose in as the clause.  A component's points take the same clause; `gear.sv` /
  `gear_trace.sv` state theirs (else a flank's first step can reach the mirror branch).
- **Presentation is a separate statement from what the drawing is** (`style.rs`, Solvent §13.2).
  A declaration carries a **class** (`datum := line(o, q) class construction`) and a top-level
  `style .NAME { dash: 7 4; width: 0.5; color: #888888 }` says what a class looks like.
  **No algorithm in the core consults a class**.  `construction` is a class, not a word:
  `style .construction { dash: 7 4 }` is the one rule in `style::base()`, which a document may
  override.  **The core resolves and the front end strokes**: `paint.ts` reads `ent.style` (dash,
  width, colour) and knows what a class is nowhere; `app/edit.ts`'s toggle sets and clears the
  *name*.  A sheet's lengths are **screen pixels**, never world units.  An unmatched class is not
  a diagnostic (no rule, as in CSS).  The cascade is **two layers, not one interleaved pass**: the
  whole base sheet under the whole document's, each in written order, so a document beats what
  ships whichever class it is written on.  A base rule states only what its class **adds**
  (`.reference` is the lighter ink only; a reference dimension is `class dimension reference`).
  A value the sheet cannot read (`color:` with nothing after it) is **dropped**, as an unknown
  property is.  `Sketch::style_epoch` is bumped by the one write path (`set_class`, `set_sheet`)
  so a binding may cache against it.  JSON writes `"class"` and **reads `"construction": true`**
  and never writes it.

- **A chain is a named traversal** (§6.6; issue #49, item 3). `profile := line -> … -> close`
  records a `NamedChain` beside the usual desugared declarations and constraints. The flattener
  scopes the binding and its links like ordinary component members; edges stay `boss.ab`, the
  grouping is `boss.profile`. Closed traversals build ordinary faces and can be swept directly.
  Open traversals retain their ordered references for `face(trail, -> close)` and are refused
  as direct sweep sections. All geometry validation stays in the existing face/solid code.
  Source edits preserve the expression; deleting its geometry is refused under the existing
  chain rule. Canonical flat printing refuses named chains atomically and retains source,
  as it does for components. `tests/solid_lang.rs` and the browser binding test cover this.
  `repeat e in CHAIN { … }` (and `cycle e in`, refused on an open chain) makes a copy per link,
  `e` an alias to that link in each copy (`#<id>.<k>.e`); the count is the chain's, so the block
  is set aside where it stands and expanded once the chain resolves (`flatten::expand_pending`),
  through instances and group formals alike. `tests/chain_edges.rs` is the gate.
  **A block walks a face too** (#243 L8, [0.113]): `repeat e in f` over a face by name or a
  `face` formal runs its loop (`flatten::resolve::face_walk`) — a closed chain it is made of or
  the edges it names, each `e` an alias as above, or for a face of corners the straight run from
  each corner to the next (`Link::Run`), a hidden `.closure` line keyed under the face
  (`{face}#run{k}`, `keys::run`), drawn once whatever walks it, so its corners read as the face's
  geometry and not a copy's.  Holes, an anonymous corner, a trimmed stretch and corners beside
  edges are E080 at the reference.  **Walks run in step**: `cycle (ea, eb) in (a, b)`
  (`EdgesOf::zip`, `words::over_walks`) resolves every walk before it expands, copy `k` aliasing
  link `k` of each; different lengths are E080 naming what each walk stands for (a word's formal
  names it only as the word's own), a `cycle` over an open one E103.  **A face of a block's
  copies**: a face's item naming nothing as it stands, whose copies `e[0]`, `e[1]`, … exist, is
  spliced into them in order (`flatten::resolve::faces_of_copies`, before references resolve),
  and each such edge names its swept face `side[k]` (`program::solids::faces::boundary_name`
  spells a copy's public path).  `tests/correspondence.rs` is the gate.  **A face carried
  whole** (`sec := prof mapped(…) in n`, #357) is such a block that makes no copies
  (`Block::image`): see `solids.md`.
