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
has a top-level `.sv`. Each of the engine's part files (`engine/*.sv` but its table) previews its
part about the world's origin as `engine.sv` places it, solid included (`tests/engine.rs`).
Source edits add geometry inside the preview, reusable definitions outside.
Hosts look for modules beside the model, then its ancestors, then the library.
`web/tools/copy-examples.js` packages sources for static hosts; live files override them.

**Nothing is imported bare unless the `use` names it (Solvent §14.4, [0.30], [0.48], #94):** a
used module's component, relation word, param or group is written by full path —
`engine.parts.Rod(…)`, `hardware.nut14_af`, `components.dims.D` — only through the
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
`Walk::apply_word`, over `flatten/apply.rs`, #103): params by label (E004/E040) — but a word of
one parameter takes it bare too (`Walk::word_args`, [0.152], #463: `r above(7) q`; bare to a word
of two or more E004, to one of none E040; a bare argument inside a body passed on to a defined word
as a labelled one is, `pass_on`, and `word_dimension` edits it), a number
parameter bound in the body's scope, a word parameter where a word stands, an **entity parameter**
(`Param::Entity`, read where an entity stands: `n square(up: std.z) g`, #250) aliased as an operand
is — its text read as a reference at the use (`syntax::reference_at`), a direction word put in
place unless a name there is spelt so (`names_here`: a formal `up` given on) — and one only passed
whole to another word (`Param::Passed`) decided by what it is given, operands aliased,
every relation's span moved to the use's word (a parameter's dimension to its argument, so
`written` and `edit::set_dimension`'s `word_dimension` read it); nested words resolve from the
definition's file, a cycle is E003, an unknown word E102 naming the import. **A point's plane**
([0.152], #463): `o.plane` is the frame a point is drawn in (`program::resolve::follow`, `PLANE`,
`drawn_in`), so a word about a point in a view reads the view from its operand and stays closed;
`o.plane.normal` is minted once the memberships are in (`planes::drawn_normals`, over the
flattener's `normals`). **An `in` reads it too** ([0.159], #461): `in pin.plane { … }`, through any
path to a point (`in pin.end.plane`), is that point's plane, so a body draws where its operand is
drawn with no plane formal. `planes::memberships` reads every other membership first and then
those reading a point's plane round by round, each once its point is drawn (`waits`, `frame_of`);
one whose point is drawn nowhere after the last round is E040 at the reference, as a point in space
is. `tests/relation_words.rs` (`a_body_draws_in_the_plane_its_operand_is_drawn_in`) is the gate. A point standing in space is E040 at the operand, or at the use where the
operand is a word's body's — a relation that cannot settle says first why an operand whose name is
found names nothing past it (`relations::constrain`), rather than that it has no kind; before the
memberships (a declaration's child) it is refused as not yet known. `Relation::word` (`Worded`) rides to
`Constraint::word` (`WordUse`, entities; `graft` keeps it while its operands survive) and
`io::describe_with` reads it (`p horizontal q`). `flatten::word_faults` (closure E101, a
reserved word or a builtin of the fixity E071 — `constraints::builtin_word` — names twice E001)
is asked of every definition (`program/words.rs`) and blocks its expansion. A body is one relation
or several with their declarations (#103); none is defined in a body. `horizontal`/`vertical` between points live in `rust/lib/std.sv`; the printer
writes `level(up)` (`print::ordinate_text`). `tests/relation_words.rs` is the gate. E071 is a
component defined twice in one file, the document's own included (a value twice is E001). `std.front` needs the document's own `use std` — a definition's body, its file's ([0.167]). A drawn callout drops a
module path (`relations::unqualified`), and a module with a `preview` draws its own dimensions
there and not in a document using it (`Constraint::elsewhere`, [app.md](app.md)).

**A use named (§9.9, [0.149]):** `rim := A round(r) B` (and `rim := round(r) X`) parses as the
relation with `Relation::name` (`parser/definitions.rs`' `named_relation_ahead`, asked in
`definition` after the words read past a reference and ahead of the instance; a word of the
language's own — `is_operator`, `reserved_word` — is no such use, read as the declaration it
would name and refused there; `print::write_relation` writes the name back). Its application's
prefix is `{scope}{label}#{statement}.{n}.` (`flatten::keys::labelled`, `Walk::begin_nth`): the
label one segment with the statement, which no source name spells, so the steps a face's route is
read by — `solid::route_step`, the one reading `operand_primitives`, `brep::recipe`,
`faces::face_reach` and `carrier` share (`past_step`) — put each named use's label before its
solid's own leaf (`plate.rim.taken.round`) and two named uses' faces stay apart; the recipe lumps
lowered regions by their labels (`cad::Walk::lump`), so a named round's pieces keep its names. The
label is the body's (`Walk::body`: a declaration, a param, a group, an instance, a set or another
named use of it is E001), and a named relation no defined word expands is E001 at the name
(`walk_relation`). `tests/composed_rounds.rs` is the gate.

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

**`claim { … }`, a claim with witnesses of its own (§9.7, [0.151], #460):** hoisted as an `in` body
is (`parser::statements::witnessed`), the block minting a statement id of its own; `stamp_witness`
marks each declaration in it (through a `when`) with the block's id as `GeometryRoles::witness` and
each relation `claim` with it as `Relation::witness`, and refuses anything else (an instance, a set,
another block: a syntax error).  A defined word applied there stamps what its body makes a witness
and every row it states the block's — the `coincident` placing a point it declares included
(`flatten::apply`).  `ir::Relation::witness` reaches `Constraint::witness`, and the role
`Sketch::roles`, as the block's index per copy (`Elab::witness_block`, keyed by id and path: a
word's witness past its applications), and `program::witnessed::hold` holds every number a witness
owns (`fixed`): no row of the drawing reads one, so the drawing's solve, counts and diagnosis are
those of the document with the block deleted.  A statement outside the block reading a witness — a
row, a plain claim, another block's relation — is E040 there; an entity's own intrinsic rows are the
witness's.  The role prints as no modifier (`Annotations::write`) but is a style selector,
`.witness`; the flag travels through `graft`, JSON (`"witness"` on the role and the row, written
only when set) and `to_program`, whose printer writes each block back as `claim { … }` after the
body, grouped by that index (`print::claim_blocks`), and refuses a block's statement printed alone
(`write_stmt_to`).  What the diagnosis does with one is solver.md's; `tests/claim_witness.rs` is the
gate.

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

**`scenario NAME { … }`, a load case (§9.11 [0.164], #510 P4):** a block like `when` — no prefix,
the enclosing body's names — at the top of a document alone, whose body is lines (supports, loads,
requirements) and no statements; it adds nothing to the drawing.  Its rules are
[analysis.md](analysis.md)'s.

**`when` is the one conditional (0.91, #267).** Two names that are not numbers compare by
**identity**: `a == b`/`a != b` (`Cond::Cmp` over two `Ast::Var`, a side not in `vals`) holds as
`Walk::stands_for` resolves them — the absolute name through every alias, so a formal is what
its call gave and a face repeat's binder its face — and `c in B` (`Cond::In`, made by
`parse_condition_in` only) as `Walk::faces_named` lists `B`'s faces. The walk answers through
`values::Names` (`expand::Asked`), reading only the aliases a condition names (`resolve::Lazy`,
as the whole table would resolve them, and held to it under debug assertions on a small walk). A number
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
expression in it E103; a name in scope that is no formal, E040.  **A name nothing in scope is**,
outside a trace and a face family's station, is the slot of the contact holding the rate's point
(#365, [0.130]): `rate_over` returns the empty name and the rate is marked `RateOf::slot`, kept
with the name as written, and `constrain` settles it as a row like any other (`settle_rate`,
relations.md's "The rate of a contact") rather than `rate_row`'s.  It passes no plane
membership (`drawn_incidences`, `planes::incidences`).  In a trace the lowering takes it
(`program::curves`, `program::traced`, curves.md's "Rates along a running formal"); on the sheet
`program::relations::rate_row` keeps a face family's in `Sketch::rates` (one rate, of the first
operand, before `parallel` or `perpendicular`, else E040; E040 a rate of anything but an axis or
a point, or against anything but an axis, a line or a plane) and states no constraint (solids.md).
`initially { … }` is hoisted as an `in` body is (`parser::statements::initially`), each relation
in it, through a `when`, marked `Relation::initially` (anything else in it a syntax error); a
defined word's body takes the flag as it takes `claim` — but a claimed word's `coincident`
placing a point its body declares is stated, never claimed, so a claim judges what it says of
geometry it still makes (#365, `flatten::apply`) — and it reaches `ir::Relation::initially`
and `Constraint::initially`: on the sheet a row of the drawn home, in a trace the home's alone.
`tests/rates.rs` (a face family's) and `tests/traced_rates.rs` (a traced body's) are the gates.
**Over two formals** (§6.5 [0.121], #339 D4) a rate may be along either, at most one along the
first (marched) in a relation — `rate(x over d) perpendicular rate(x over u)` is a rate row — and
a relation reading only rates along the second is algebraic at each step, lowered with the body's
other rows (`program::traced::Lowering::rate_relation`, `rated` keyed by point and formal);
curves.md's "The method of lines" says what it may read, and its "Cauchy data on a drawn curve"
(#376, [0.133]) what `initially` may read of a curve the body is written over.  A rate to the right of its word, `nl
perpendicular rate(p over u)`, is a relation, not a chain's link (`parser::chains::chain_starts`
looks past the word for `rate(`).
**A dimension that reads the body's own points** (§6.5 [0.132], #378) is written as any other:
the symbolic expansion keeps a dotted name in a trace as text (`note_dim_reads` sets nothing aside
there), and a body traced in space resolves `p.x` against the points it declares where it lowers
the dimension (`program::traced::Lowering::dimension`); curves.md's "A number the body places"
says what it becomes.  A trace in a plane compiles its dimensions over the formals and the geometry
alone, so there the name is still E103 ("not something this curve can read").

**Every braced definition is a function of its arguments and of the frame its instance stands in**
(§8.2, [0.162], [0.175], #461, #562, [plan](../parts-plan.md)).  There is one definition form:
`Component::braced` (named, no set family) is what `parts::check` reads, and relation words and set
families are untouched.  `part Name(…) {` is refused by the parser (E100, `PART_RETIRED`, read
through `words::part_at` so `part := solid(…)` and `part above(7) q` stay statements), and parsed on
as `component` so what follows is still checked; `highlight` and `symbols` no longer read the word.
`parts::check` runs once `program::elaborate` has a document that elaborated with no error.  It
reads each drawn instance as the classifier reads one (`boundary::split`).  Its own unknowns are
what it made, the numbers no entity owns that its rows read, and the free variables under its
prefix that are no formal of its own; the free variables under its prefix that *are* its formals
— a number the call left unbound — are columns its rows may determine and need not (`Read::unbound`:
in the over-determined block's matching, not in the matching E108 asks for).  Its rows are the
acting ones among those kept.  Its inputs, held, are every other column (`boundary::reduced`).
**The frame is an input** (#562): an intrinsic row places one thing, its *subject*
(`boundary::subject`: a lift's point, the origin a plane stands at, an axis held through a point or
along a line, a twin held on a plane or folded onto its point), and is the instance's exactly when
that subject is something it made — so the lift of its point into the frame it stands in is its
own and the frame's placement an input, and the lift of a point another instance draws in a plane
this one made, or a plane another stands at this one's origin, is not this one's.  An intrinsic
row with no single subject (an arc's ends on it, a unit direction) is the instance's when it names
a non-point the instance made.  **What a lift reads is its point's and its plane's** (#501,
`World::with`): a plane's axes are the plane's numbers and a point's hidden point (`Sketch::lifts`)
the point's; and a curve's points no name reaches (an arc's centre and ends) and the plane an arc
in space is drawn in (`arcs_in_space`) are the curve's.  Before, those numbers were no entity's,
so whichever instance read one first took it, and with it every row over them.  Two rules are
judged over the compiled structure of those rows (`System::structure`,
`graph::dulmage_mendelsohn`):
- **E108** at the definition's name: an own column the matching leaves unmatched, named by its
  entity or its free variable;
- **E109** at the row's site: a row of no own column or unbound formal, or one of the
  over-determined block, worded by whether it reads either.

Each fault is said once a definition (`said`), against the first instance it is found in.  The
structure only, so it is deterministic and decided before the solve; nothing changes a row.  A
flow's home is the body's own (`initially`, from formals); a joint is a relation word (`turns`,
`spins`, `slides`, `screws`, `rolls`), the components `std.Turn`/`Spin`/`Slide` retired.
**PT5 is advice** (`parts::roots`, **W207**): `solventc --lint` asks, of the solved sketch, the
classifier's root search for each definition's first instance (`boundary::classify`), and names a part
classed *locally determined, branch not chosen* — its scattered starts came to another pose its
stated `ccw`/`cw` and its bounds admit (a bound sited under the instance is among the rows `split`
keeps, read by `bounds_hold` on each probe's root, as the orientations are by `orientation`).  A
`sense:` needs no reading: its row's only root is the one it states (#535), so a probe turned the
other way finds none, and an axis a plane holds along a drawn line is sensed so by its row.  Not in `DRIFT`, which reads text alone;
`parts::roots` is its gate's.
**The diagnosis is read through the parts** (#461 D1–D3, the diagnostic half of #455, from the flat
solve): `parts::read` takes the solved sketch and its `Diagnosis`, and `solventc` prints
`parts::text` after the diagnosis's own lines (and `--json` writes `"parts"` where the document
has any).
- **D1:** a conflict (`Diagnosis::conflicts`, else `over`) holding a part's rows and a row outside
  it over the part's own unknowns names that row as the one to remove, the part's rows and its
  inputs as why.
- **D2:** a part whose rows `violated` holds is named with its rows and its inputs' places, the
  innermost only, and not where D1 already said why.
- **D3:** a settled solve's part whose rows, alone with its inputs held, have numeric rank
  (`System::conditioned`, `RANK_TOL`) short of its own unknowns is said singular with the rank.

`tests/parts.rs` is the gate.

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

**Closed components:** model dependencies enter through arguments. Definitions and built-ins stay
callable. **A definition's standard datums are those of the frame it stands in** ([0.167], #524):
the one name a body reads unhanded. `walk_instance` gives the body's scope `Scope::stands`, every
`in` around the call and around each call enclosing it (`Stamps::{rides, plane}`, each an
`InPlane` carrying the file and the place it was written in), outermost first and empty at the
root; a relation word's closure, a traced body and the document's own scope have none. Where
nothing else names it, `Lookup::raw` reads `std.front` in such a body through `Lookup::datum`: a
standard datum's name (`Datums`), where the body's file says `use std` (`Datums::files`, the
document's `None` or a module's index), and the drawing has them — made whenever any file read
says `use std` (`Walk::new`, over `Program::uses_of`), while `std.front` read by its own spelling
is a datum only to a file that says it (`Lookup::unread`). Where a place it stands resolves, read
where it was written (`Scope::written_at`), to a frame (`Datums::frames`: a declaration written
`frame`, or a reading at an instant whose source is one, `walk_reading_at`, `Lookup::frame`), the
innermost such names **the frame's own datums** ([0.169], #542; through 0.167 E101): `std.x`,
`std.y`, `std.z` read as its `u`, `v`, `normal` (minting its chart as `f.u` does), and the rest as
`std`'s `FrameDatums` over it under `keys::frame_datums` (`{frame}#std.`, keys a source cannot
write, walked `Stamps::anonymous` so no reader is shown them),
`std.back` as `behind.normal`, `f.u` reversed. Their names are made with the frame
(`Walk::declare_frame`), so an alias of one resolves as any name does (a call or a word handed
`std.front`), and the frame is kept in `Datums::read`; `Walk::finish` walks `FrameDatums` once for
each frame read, after the world's, its formal an alias of the frame and sited where the frame was
declared (`Frame::path`), so a part that declares the frame makes them (`boundary::split`). Each plane
stands at the frame's own place (`Sketch::plane_at`, chosen in `entities::build_plane` by
`flatten::frame_datum`): no unknown and no row for an axis already through it, so a frame's datums
add no freedom and no dependency, and `diagnose::view_freedoms` says a chart sharing an earlier
one's place no more. Drawn in a plane standing where the frame around it does is no W115 (`planes::member`), and
`drawn_incidences` twins a point onto one as onto the world's. A plane is no frame, so `Tooth(…)
in top` reads the world's. `tests/frame_datums.rs` is the gate
(`a_body_reading_its_datums_at_the_root_stands_where_one_handed_them_does`,
`a_frame_at_the_worlds_pose_gives_the_roots_solve`,
`a_frame_turned_and_moved_moves_the_body_with_it`,
`two_instances_of_one_definition_stand_in_two_frames`,
`a_bare_frame_placed_by_relations_on_its_datums_is_fully_held`,
`a_frame_left_unplaced_reports_its_freedoms`,
`a_definition_in_a_frame_with_a_joint_rides_it_at_a_roll`,
`an_instance_stands_in_the_innermost_frame_which_rides_those_outside_it`,
`a_frames_datum_is_handed_on_as_any_name_is`, `nested_instances_read_the_same_datums`,
`a_modules_definition_reads_its_datums_by_the_modules_own_use`,
`a_body_reads_the_datums_of_the_frame_around_its_call_wherever_that_is_written`,
`a_frames_own_datums_are_read_by_no_name_outside_a_body`, `a_plane_is_no_frame`,
`a_relation_word_still_reads_only_its_operands`). **An instance is a frame** ([0.171], #548): read
from outside, `b.std.D` names the datum `D` of a frame of the instance's own, `keys::instance_frame`
(`{b}#frame`). What decides it is lazy and syntactic: each time a body is walked, before its
instances, `Walk::read_outside` (`flatten/framed.rs`) reads its statements — their text by
`syntax::name_uses`, and the frames they are drawn in, which a hoisted `in` header is no part of —
for a path `X.std.D`, `X` not empty and `D` a datum's name, and keeps `{prefix}X` in
`Datums::outside`, so a later statement or an enclosing body reads it as well as the one declaring
the instance. `Walk::own_frame`, at the call, declares that frame bare as `in (f := frame) { b := …
}` would — in the `in`s around the call, `Stamps::plane` stamping the parent's, so it rides the
frame its call stands in — put in `Expansion::charts` so it has axes, and stamps it as the body's
innermost `Stamps::plane`, the call's own `in` ridden, so `Scope::stands` ends in it and the body's
`std` are its datums (#542 as above). `Lookup::raw` reads the path through `Lookup::instance_datum`,
one rule with the body's (`Lookup::frame_datum`). The frame is **square**: `entities::build_plane`
gives an instance's own frame `Sketch::square_chart`, an intrinsic `Perpendicular3` of its `u` and
`v`, so it has six freedoms, and names it after the instance (`Sketch::plane_names`), so
`diagnose::view_freedoms` says `cyl.attitude` and `cyl.origin`. It **starts where the frame around
its call stands**: `views::instance_frames`, after the holds, seeds each from the innermost frame
with axes it rides (`Sketch::seed_at_frame`), a held number left as held. An instance whose `std`
nothing outside reads makes no frame and stands as before. One whose body reads no datum, nor any
body it calls (`Walk::reads_datums`), is kept in `Datums::refused`: its `b.std.D` names nothing and
`Lookup::missing` says **E111**, since it is placed by its arguments. A path past `std` naming no
datum is left as written. `tests/frame_datums.rs` is the gate
(`an_instance_whose_std_nothing_outside_reads_stands_as_before`,
`an_instances_datums_place_it_as_a_frame_written_around_it_does`,
`a_mate_on_an_instances_datums_holds_it_wholly`,
`an_unplaced_instance_reports_six_freedoms_under_its_own_name`,
`two_instances_of_one_definition_are_placed_by_two_mates`,
`an_instance_in_a_frame_with_a_joint_rides_it`,
`an_instances_frame_starts_where_the_frame_around_its_call_stands`,
`an_instance_nested_in_a_body_is_read_as_a_frame_by_its_path`,
`an_instance_whose_body_reads_no_datum_is_refused_its_frame`); the V-twin's bank mates its cylinder
so. **A definition reads the constants of its file**
([0.160], #461 PC1, PC2): `Walk::file_numbers` works out, once a file, the numbers the file reads at
its top — a module's own and its `use`s' (`used_params`, `imported_params`, `module_params`), the
document's from its root table less its root instances' numbers and its `preview`'s — and splits
them into constants (no unknown, `Aff::free` none) and the rest. Every instance's table is its
call's numbers over the constants of its component's file (`walk_instance`), and every word's
closure its parameters over its word's file's (`flatten/words.rs`), so a formal hides a constant;
the rest join `Outside::unknowns`, out of reach with their own E103 (`values::out_of_reach`: "an
unknown of the drawing, and a definition reads only constants"). A word's definition is judged as
written against its file's numbers by name (`flatten::file_number_names`, `faults`). The gates are
`tests/closed_scopes.rs` (`a_definition_reads_the_constants_of_its_file`, `a_formal_hides_a_constant`,
`a_definition_reads_no_unknown_no_instance_number_and_no_preview_value`) and `tests/modules.rs`
(`a_definition_reads_the_constants_of_the_modules_its_file_uses`,
`a_definition_reads_no_number_of_its_caller`). A component scope holds only its formals and
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
(`a_family_owns_a_point_formal_left_out`). **A number formal may state its default** (#383,
0.135), `advance: Length = 0mm` (`Formal::default`, a number type only, else a parse error), which
`Walk::bind` binds where the call leaves the formal out; and **a `frame` formal left out**
(`Formal::frame`) is the instance's own bare frame, declared by `owned_points` as `f := frame`
would be: a frame with no joint, the world, so `seen_from:` left out of a call reads from rest.
`m: motion` is refused naming `m: frame` (`formal_or_interface` keeps `motion` from being read as
an interface's name). `tests/frames.rs` is the gate
(`a_left_out_advance_is_none_and_a_left_out_observer_is_the_world`). **An
instance's numbers are read by its name**: `Walk::instance_numbers` puts every numeric formal
of every instance a body writes into its `vals` as `inst.formal` (the number bound, else the
unknown `bind` names), before the statements, so `p distance(ball.r) ball.center` reads it and a
group argument carries it (`bind`'s group arm copies `{actual}.…`); body values are not. No
solver state; member units survive substitution; missing members are errors.
**A family the same set in any order** (#415, [0.148]): `component Run(a: face, b: face, r:
Length) symmetric(a, b) := { p | … }` (`Component::symmetric`, items of one or more formals,
`parser::statements::symmetric`; on no family, one item, a formal twice, items of unequal length
or places of unequal type, a parse error).  `flatten::same`: `Walk::same_instance`, asked in
`walk_instance` before anything is made, keys each such instance on its component, what each
argument stands for (`stands_for` through the instance's own formal aliases, a number's value, a
side's word; any of them unknown, or the instance drawn `in` a plane or riding a frame, or a
curve's compile: no key) with the items in the order that sorts them (`canonical`, any number of
items), and the roles and classes its parts would be stamped with (an instance stamped otherwise
is its own); an instance whose key was made before is a name for that one (its name out of
`names`, an alias to the first, its `InstanceInfo` popped; its formals' aliases and the call's
checks stay) and its body is not walked.  A **copy's** region solid of such a name is the first
copy's solid of that set (`Walk::same_solid`, through `set_named`, read once with the region
`region_of` makes; only copies' solids are remembered, so a document's own solid is never taken
for a copy's nor a copy's for it, and a document's own two solids of one set stay two).  Every
body statement is emitted; one over such a solid saying what one over it said joins nothing more
(`Sketch::join_body`: a tool a word already joins to a body is not joined again).
`tests/symmetric.rs` is the gate, beside the closed forms of
`tests/round_lowering.rs`.
Curves need fixed scalar/entity formals. The V-twin's parts, as the inline four's and the hub
motor's, read `components.dims` as constants of their definitions ([0.160], #501), each drawing
in the plane its operands are drawn in: `Frame(O, ref)`, `Bank(crank.pin, plate.pivots.r, z:
components.dims.zB)`.

- **A name is defined one way, `NAME := VALUE`** ([plan](../definitions-plan.md), Solvent §5,
  [0.29]).  `w := 100` is a value (`StmtKind::Param`), `c := circle(…)` a declaration, `t :=
  Tooth(…)` an instance, `dims := {…}` a group (a brace after `:=` is a list, lexed across lines),
  `profile := (ab := line(a, b)) -> line -> close` a chain whose link is named in place, `k :=
  trace(leg.toe over u in (a, b))` a curve ([0.140]; `S := trace(C(…).p over (phi in (a, b), s in
  (c, d)))`, two formals running, a surface traced in space, #326 — `CurveSpec::second`, parsed by
  `curve_spec` after an `over (`, printed by `write_curve_spec`, its ends settled by the flattener
  with the first's), `p := point(x: e, y: e)` a computed point.  A dimension's number defines no
  name (named dimensions are gone, [0.41]).  `:=` is `Tok::Define`; a lone `=` is no token.  `:=`
  binds loosest, so a link is named in parentheses; with no `->` in the statement the name goes to
  the one declaration (`l := horizontal line(a, b)`: a prefix word's value is its operand).
  `P::definition` lowers to the existing `StmtKind`s (`Param`, `Group`, `Instance`, `Decl`,
  `Chain`), so nothing below the parser knows; `syntax::words::named_link_at` tells `(l := line)`
  from an operator's own parentheses.  `label:` never defines: it fills a slot.  A name in a child
  slot (`line(a, q := hint(…))`) is not implemented yet.
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
  there yields to. `module_params` exports values to importing root bodies, and they
  are constants of the importing file's definitions ([0.160]). `tests/seeds.rs` and `tests/closed_scopes.rs` are the gates.
- **Elaboration is linear in the instances** (#403).  The root's numbers in force hold every
  instance's by name (`P0.w`), and every statement emitted carries the table of its body, so
  `Scope::vals` is shared (`Rc`) and never copied per statement; what a closed body may not
  capture (`Scope::forbidden`) is the outer tables themselves, asked at lookup (`Outside`), not a
  set of their names built per instance.  `tests/many_instances.rs` is the gate: the bytes
  elaborating four times the plates allocates are about four times as many, counted by the
  allocator it installs for the whole core suite (a per-thread tally beside `System`'s).
- **The numbers in force are frames, read outward** (#405, `flatten::table::Vals`).  A body, a
  block's copy, a `when` that holds, a word's application and a number an application gives
  (`Application::give`) each add a frame of only the names they make over the table around them,
  shared by `Rc` and never copied: a copy of the root's (every instance's numbers) per block copy
  or per `when` is the instances times the copies.  An inner frame's name hides an outer one's,
  as an insert into a copy would.  An instance's table starts afresh with its call's numbers, so
  the depth is the blocks nested inside one instance.  Nothing reads the whole table per
  statement: a member is read under its prefix (`Table::under`), whether a value is affine in an
  unknown is asked of each frame's index of them (`Table::binds`), and a seed reads the table
  through `Seeded`, which answers a free value with where its unknown starts at the moment it
  is asked, where it was a table of every number built per declaration as soon as one was free.
  `groups` and `sides` are shared as `vals` is, `groups` copied only by a body declaring a group
  of its own.  Outside the walk, the elaborator reads where an instance on a constraint's path is
  written from the statements' spans gathered once (`source_map::spans`), asked only where some
  module is previewed.  `tests/many_instances.rs` holds a free root number
  and a root `repeat` and `when`s beside the plates to the same fourfold bound.
- **A part carries its views in `in view { … }` blocks** (Solvent §6.7, `P::in_comp`): the block
  form is allowed inside a component body (the plane a formal, or since [0.159] the plane an
  operand is drawn in, `in pin.end.plane`) and still refused inside a root block.  A part makes
  every view it defines, every time; which a sheet shows, and how, is the drawing's (`style
  m.rod2 in end { … }`, [solvent-drawing.md](../solvent-drawing.md)).  So a part's whole design is
  one module (`engine/block.sv`, `engine/head.sv`, `engine/crankshaft.sv`, `engine/conrod.sv`,
  every definition of the engine a `part` reading `engine.dims` as constants, #461, and every one
  of the V-twin's, `vtwin/components/`, #501); view modules
  hold only what the assembly adds.  The hub motor's parts (`hub_motor/motor/`, #502) take no
  plane either: each is given `motor.parts.Datums`, the axle's centre and centreline in its two
  views, and draws `in f.o.plane` and `in f.so.plane`.  Instances inside a block copy are indexed like declarations
  (`cyl[0].small`, `copy_of`).

- **A name declared over a built-in is said** (Solvent §3.3, §5; `program::shadowing`, W112).
  `expr::eval` knows `expr::CONSTANTS` and `FUNCTIONS` before the document and
  `flatten::substitute_with` only the document, so a value, input, formal or block index called
  `tau` reads differently in the two: the **warning**. Asked of the text (every component,
  instantiated or not, and every module body).  `expr::builtin` is the one table; `tests/names.rs`
  is the gate.
- **The drift lint is asked for, and refuses nothing** (#462; `program::drift`, codes
  `W201`–`W206`, the `W2xx` block ours as `E1xx` is).  It names six patterns that are drift from
  the language's style by definition (R10 of #448), not
  counts of an idiom: **W201** a label given the name it labels in a call (`teeth: teeth` to a
  component, `d: d` to a word a file defines — a built-in's slot, `plane(u: u, v: v)`, and a seed
  are no call); **W202** a component call standing as a statement; **W203** a component that makes
  nothing a parent can read (everything it declares is `private` or a claim's witness) and states
  relations; **W204** a path reading through an instance into its instance (`pair.gear_cone.ax`,
  the instances known by their statements, a block's next copy's too, `next.t.r.lo`; a path
  through an index or a formal is not followed);
  **W205** an `Int` formal read only by `when` conditions; **W206** a `group` formal handed on whole
  and never read through.  Asked of the **document's own text** (its components and root, not a
  module it reads, which is linted as a document of its own), and never by `elaborate`: only
  `solventc --lint` adds the warnings, which fail nothing.  `tests/drift.rs` holds each pattern to
  one document that trips it and one that does not, and holds the corpus (`rust/examples`,
  `rust/lib`) to `ALLOWED`, a list that only shrinks, for the patterns in `GATED` (W204, W206),
  each of which the language can say another way today: an instance names what a parent reads of
  its instance as a group of aliases (`HypoidPair`'s `gear_member := {ax: gear_cone.ax, …}`,
  `Tooth`'s `left` and `right`), or the parent reads it off a member one level down
  (`inlet.tbore.center`, a set's `flank.apex`).  The list is empty.  The rest are said, not held:
  W201 waits on a word's unlabelled measure (#463), W202 on `Standoff` (the motion components are
  words since #464, `rolls` since #562), W203 on a word that may leave a number to the solve
  (`std.Screw`'s pitch, a joint's ratio: a word's parameter left out is E040); W205 is a
  heuristic, since a flag chooses a model as often as a picture.
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
  (`std`: `std.StandardDatums`, `std.FrameDatums`, `std.Turned`), in the Makefile's `RUST_SRC`.  `program::reparse` relinks from the
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
  order: a simple polygon).  A contour scattered at the size of a unit whose stated sides are
  thirty no longer collapses one on the way: the direction rows read unit directions (#479,
  [solver.md](solver.md)).
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
  (`Block::image`): see `solids.md`.  **A frame carried** (`n := base at τ`, #366,
  [0.138]) is a declaration of kind `Frame` the parser marks (`Decl::carried`,
  `statements::carried_frame`: `at:`, and `under:` and `seen_from:` if given, read as a carried
  point's arguments) and the flattener writes out where it walks the body (`flatten::carried`), so
  the source keeps and prints the one statement (`print::write_decl`); see
  [frames.md](frames.md).
