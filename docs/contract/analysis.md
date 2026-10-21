# Analysis: load scenarios and requirements

Part of the [contract](../../CLAUDE.md) the code is held to. Read it when you touch
`model/scenario.rs`, `syntax/scenario.rs`, `syntax/parser/scenario.rs`, `program/scenarios.rs`, or
write a `scenario`. The program these rules are built to is issue #510 (P1–P8, S0–S4); a tag
(P4, S0, …) is its name there. The gate is `rust/gcs-core/tests/scenarios.rs` unless a rule names
another.

**A scenario, parsed (P4, §9.11 [0.164]).** `scenario NAME [: BASE] { … }` is
`syntax::StmtKind::Scenario` (`syntax::Scenario`, its lines `Item`/`ItemKind`: a support, a load
or a `RequirementText`), read by `parser/scenario.rs` alone — its body is lines, not statements,
each told by its first word (`Hold::parse`, `LoadKind::parse`, `claim`, `Output::parse` before
`(`), so the words are contextual and only `scenario` is reserved (`words::OPENERS`). It stands at
the top of a document or in its `preview` (`unannotated_chain_or_one` asks `at_document_top`, as
a `param` does: anywhere else is a syntax error); a declaration in it is a syntax error ("adds no
geometry"), and so is a load vector of two components. `print`'s `write_stmt` writes it back as
written (`Scenario::write`), and `highlight` tints its words inside a scenario alone, as the
parser reads them (`scenario_word`).

**Read as the drawing's names (P4).** The flattener writes the scope's numbers into every number
(`Scenario::numbers_mut`, `substitute_in`) and rescopes every reference (`refs_mut` in
`resolve::rewrite`): a scenario is a block like `when`, no prefix and no `PathStep`.
`program::scenarios` reads each in `finish`, once every solid is built, into a
`model::Scenario` (gcs-solve, pure data): each target by `Resolver::operand` — `Applied::Ent` (a
point, an edge, a face, a solid, `material`; `B.body` its solid) or `Applied::Face` (a face path,
`FaceRef`), anything else **E040**, an unresolved name **E101**; a pressure on a face alone
(**E040**); `along:` an axis or a line (**E040**); each number by `flatten::value_read` against
`Sketch::declared`, held to `LoadKind::dim`/`Output::dim` (**E103**) and refused where it reads an
unknown (**E040**); `mode`'s count a whole number from 1 (**E103**). Whether a face path names
faces is not judged here: the analysis reads the solid's boundary after the solve, as a region
reading a face does. Values are in the units the arithmetic is done in (`units.rs`).

**Scoping, decided (P4).** Names: a scenario's is its own namespace (**E001** twice), colliding
with no geometry. Extension: `scenario B: A` begins with `A`'s supports and loads (copied, with
the lines they were written on), then its own; requirements are never inherited; `A` is written
above `B` (**E101**), so there is no cycle. A load combination is written out, no construct. A
scenario adds no geometry of its own: a load patch is drawn in the body. `Elaborated::scenarios`
holds each as a `program::Stated` beside the `Line` (statement, index) of every support, load and
requirement, so a later report says it where it is written and survives `retext`.

**Nothing is solved (P4).** A scenario states no row and makes no entity, parameter or unknown:
the sketch, its JSON and its solve are the document's with every scenario deleted
(`a_scenario_adds_nothing_to_the_drawing`). A requirement's grade
(`Requirement::grade` over its `expr::Cmp`: `Claim`, `Driving`, `Bound`) is recorded, not acted
on, but the rule S1–S3 hold is kept now: one scenario's requirements drive dimensions, a driving
row in a second **E106** until S4's optimiser. An edit
removing what a scenario names never takes the scenario with it (`edit::remove::mentions`): the
name left dangling is said (**E101**).
