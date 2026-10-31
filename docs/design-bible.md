# Design Bible

This document is the authority on design principles across the project. A contract file, the
specification or an epic is bound by it. Where a rule there conflicts with a principle here, the
principle governs, and the rule is the defect.

Most of these principles were first stated to redirect a piece of work that was going wrong. The
issue numbers and commits in parentheses point at where that happened, so a reader can see the
case that taught the rule.

## Project purpose

Solvent is a language for describing physical geometry in terms of design intent.

### Examples

* Driving 2D and 3D constraints, as traditional sketch solvers do
* A beam sized according to deflection requirements
* An electric motor driven by current and power requirements
* Driving targets for magnetic fields, fluid pressure, etc.
* A rocket assembly with a particular payload delta-v
* A building of particular size and shape, compliant with building code

## Components

Solvent has four principal components. Each has its own requirements, and principles govern how
they interact.

* **The Solvent language.** It is specified in [solvent-spec.md](../solvent-spec.md), which has
  its own principles, P1–P5 (§1.1): aliasing is not constraint, bodies are unordered, hints are
  inert, components are decomposition, symmetry is a claim. Geometry and constraints live in
  `.sv`. How they are shown lives in `.svd`, views are derived and never drawn into the model
  (#523), and a flag that only chooses what is drawn is presentation (#462, #677).
* **The Solvent compiler, `solventc`.** It parses, elaborates, solves, diagnoses, reports and
  exports, over the engine in `rust/`. It invents no wording: every line it prints is the core's.
  A fault is said once, at the author's statement and in the author's terms, never naming an
  internal id (#387).
* **The Solvent IDE.** The workspace in `web/src/app/`. The text is the design: the document is
  the source, every edit is a splice of the author's text and never a reprint, and a gesture
  writes Solvent words ([app.md](contract/app.md)). It adds no geometry.
* **The CAD kernel.** `rust/cad-kernel/` holds the exact B-rep, its Booleans, mesher and STEP. It
  is clean-room, licensed MIT OR Apache-2.0, and measured beside other kernels
  ([brep.md](contract/brep.md)). It knows nothing of sets and nothing of Solvent.

How they interact:

- **The engine is ours and whole.** One dependency-free core owns every algorithm. The layers
  above it (the ABI, the bindings, `solventc` and the IDE) add no geometry and invent no wording
  ([architecture.md](contract/architecture.md)).
- **The same input gives the same output.** Ordered containers and seeded randomness only.
- **Solvent is lowered to the kernel, never the reverse.** The kernel serves the language, and
  what it cannot build yet limits exact output, never what a document can say (principle 2).

## Guiding principles

### 1. Do it right even if it's hard

Solvent and its kernel are designed to be general and robust. This means doing the hard thing,
even if it takes work, even if it would entail years of engineering labour. If a goal is
theoretically possible and its blocker is engineering labour, we commit that labour. We do not
build what the principled design will have to throw away. Scope is a separate question: work
may be deferred or declined (#151, #154), but whatever is built is built the principled way.

- **What cannot be built yet is said, never guessed.** The evaluator either builds exactly or
  refuses by a named reason with a witness, and every such refusal is an open item of work. It
  never approximates silently, and it never moves the refusal into the language.
- **"Solved", "holds" and "built" are true, or they are not said.** An unsettled solve is no
  answer, and nothing is judged on its pose (#449, #492). A silent wrong drawing is the worst
  failure, because every measurement downstream is of the wrong thing (#43, #588, #629). A part
  left out of a picture or a file is an error, not a footnote (#614).
- **Correctness is shown against references that owe nothing to what they check.** These are
  closed forms, NAFEMS benchmarks, and OCCT and CalculiX as oracles in the tests alone
  ([analysis.md](contract/analysis.md), [brep.md](contract/brep.md)).
- **Measure, do not assume.** A claim about speed, cost or scaling is settled by a measurement
  before it is built on (#459, `b6d7d0f5`). The syntax follows what an experiment shows is worth
  exposing (#461, #465).

### 2. Solvent borrows the generality of math

#### Solvent's primitives are sets, from which geometry is derived in a standard library

Solvent expresses geometric concepts as set math, and CAD concepts in those terms. A statement
means its set. In Solvent, `distance` is the primitive and `sphere` is a derived term, a one-line
definition in `std.sets`.

- **A result depends on the set, not on how it was written.** Two spellings of one set behave
  alike: a tool drawn as a `common` or as a prism, faces split or merged, a motion written as one
  joint or as a frame along a path. No author restates a drawing to suit a method.
- **The definition is the reference.** The set read directly (a field, a sampled reading) is the
  truth that exact constructions are measured against. Exactness is how we evaluate, not what the
  statement means.
- **A drawing is a function of its parameters: trajectories, not snapshots** (#391). A relation
  that reads a parameter holds for every value of it. A motion is a map of a body into space over
  its parameters, and rigid motion is its special case.
- **A physical principle is a statement.** A catenary is said by least energy, a ray by its
  medium, a front by its rate along a normal: a variational problem, an ODE, a first-order PDE
  (#121, #151, #154, #339).

#### The CAD kernel never informs the shape or capabilities of Solvent

The shape of a CAD kernel never drives Solvent's goals or aesthetics. That shape means its surface
types, its operations, its methods and their limits. A kernel is one way to evaluate a set, and
what it cannot build yet is the kernel's gap, not a rule of the language.

- **The kernel and the language have different primitives.** In the kernel, `sphere` is a
  primitive and `distance` is a query. `cad-kernel`'s `sphere()` does not give Solvent a `sphere`
  keyword, and no kernel feature enters the language because the kernel has it.
- **The language does not know the kernel's methods.** Classes, fast paths and special cases live
  in the evaluator and never surface as syntax or as elaboration rules.
- **Lowering is evaluation, not translation.** Most sets have no finite B-rep with closed-form
  faces (a gyroid, or the envelope of a moving tool). A lowering recognises a set by its
  definition, never by the name of the word that wrote it: a round written by hand, bounded by the
  same two faces and the same ball of centres as `std.rounds`' `round`, is built the same way. Its
  B-rep is checked against direct evaluation of the set. Lowering changes how a set is computed,
  never which points belong to it.
- **A gap in the kernel limits exact output, never the shapes a document can define.** A region
  the kernel cannot build is still displayed, meshed, measured and analysed by direct evaluation,
  and that path gets the engineering work to stay a practical choice. Only its STEP export is
  refused.
- **The B-rep stays for topology, exact geometry, exchange formats such as STEP, and fast
  repeated queries.** Topology here means named faces and edges, such as the ones a round refers
  to. The kernel is required, but it does not decide what a statement means.

#### Solvent shrinks, it doesn't grow

The aim of math and physics is unification. Likewise, the aim of Solvent is to express more and
more physical concepts in terms of mathematical primitives, moving their consequences to the
standard library.

- **A capability arrives as a word in the library**, written in Solvent, not as grammar, a
  constructor label or an option bag (#243, #383, #645). Rust grows only general features, each
  with a use outside the case that motivated it, and general evaluators and lowerings (#183). The
  preferred change gives a meaning to something the language already refuses.
- **One meaning, one spelling.** Two names for one idea make a worse language than one (#11,
  #76, #266). Two mechanisms that are one concept seen twice become one: one notion of a moving
  frame, one derivative primitive, one parameter axis, one definition form (#365, #366, #391,
  #562).
- **A relation is a word, not a procedure.** A component that only asserts is a relation word,
  and a bare component call standing as a statement is a lint (#464, W202, W203).
- **Entities are declared bare and constrained after.** Relation words say what holds, and no
  growing set of constructor words builds things already constrained (#383).
- **Each fact is said once, and the rest is derived.** A declaration that repeats what a body
  already says can be wrong (#148). Redundancy and regularity are found by the system's rank or
  remaining freedom, not by flags or shape-specific code (#87, #130, #145). The kernel follows
  the same rule: a vertex is computed once and shared, not recomputed and matched by tolerance
  (#711).
- **No compatibility while nothing is deployed.** A retired spelling stops parsing and is refused
  naming its replacement, the corpus is rewritten in the same change, and no solved value moves
  (#243, #383, #391).

#### Solvent is not beholden to CAD traditions

No separate assembly level, no operation timeline, no features, construction geometry is first
class, and 2D is a special case of 3D constraints.

- **There is no assembly level.** Mass and forces know nothing of assemblies, and programming
  languages have no special assembly level of functions. Composition replaces it: a tractor is a
  definition whose body applies others in frames it relates, and the designer chooses where the
  boundaries fall (#523). Physical quantities such as mass, volume and cost are additive over
  matter, so they sum over whatever grouping was chosen. A bill of materials is a query, a report
  beside the drawing, not a structure in the model.
- **There is no operation timeline.** A document is a set of statements, not a history
  (principle 3).
- **There are no features.** A round, a chamfer or a loft is a word of the standard library that
  names a set, not an operation recorded against a model (#183, #243).
- **Construction geometry is first class.** A line drawn only to state a relation is an entity
  like any other, related and solved as any other. Geometry made only to state a claim does not
  restrict the model (#460).
- **2D is a special case of 3D** (#523, #558). A 2D statement is the 3D one in a plane, read
  about that plane's normal.

### 3. Solvent encodes design intent

Design intent usually lives in a corpus of documents outside CAD. An analysis phase solves the
geometry before the CAD phase begins, and CAD then implements the output of the analysis. In
Solvent the design is encoded in the document, and the solver outputs the geometry.

- **Every number is one a person would dimension.** It is a station, a diameter, a ratio or a
  lead, as the part is dimensioned and as the shop states it (#309, #383). A value tuned until the
  picture looked right is an output written as an input (#283). Evaluation scaffolding, such as a
  roll limit or a mesh tolerance, is not design data (#271). A measurement of the solved drawing
  is never an input. It belongs in a claim (#312, #366).
- **State the property, not its answer.** A curve is said by what defines it: a taut string
  unwinding, a least energy, an envelope, a meshing law. It is never said by the closed form
  someone derived on paper (#3, #121, #283).
- **Hold by construction, not by a check that can fail.** Where the defining relation can be
  stated, the property follows from it. A flank cut by an envelope is conjugate by definition,
  and a roll that follows from the blank it cuts needs no stated limit (#271, #283).
- **Which way is a word, not a sign.** A root, branch, side or sense that the design means is
  stated (`ccw`, `sense:`, a bound, a `Side` input) and then held. A seed never chooses it,
  since a root chosen by a word survives a change of numerics that a seed does not (#145, #217,
  #157, #503, `857b1bd8`).
- **Intent to be illustrated is a claim.** A theorem, a clearance or a symmetry the design is
  meant to have is judged against the drawing and never joins its solve (#7, `d7dae876`).

#### Solvent is declarative, not imperative

Solvent has no timeline of operations, and it does not describe a sequence of imperative
operations in emulation of a machining or assembly plan. Fabrication and assembly are left as a
separate concern. Solvent documents are sets of facts stated in arbitrary order, and the solver's
job is to find the geometry.

- **Statements are unordered** (P2). Meaning never comes from the shape, the order or the
  position of the source. A joint is marked (#31), correspondence is stated (#243), and document
  state is keyed by the statement it qualifies (#2, #141).
- **State the function, not the manufacture.** The functional problem is the document: the
  meshing a gear engineer specifies, not the machine settings that cut it. A manufacturable
  family is a second layer, and the solver picks its settings, not the author (#283).

#### Trigonometry and cartesian constraints are lints

Express ideas in terms of geometric constraints, not trigonometry.

- `cos` and `sin` of an angle are a turn spelled out in one frame, and the relation they come
  from is lost (#96, #366).
- A position typed in ties the part to the page. Parts are placed by mates, not by offsets: a
  chain such as `zA = fwA + D / 2` is a mate kept in step by hand (`817e61ed`).
- A seed names a place, `hint(at: …, toward: …, turn: …)`. A seed computed as `pin.y + 26mm` is
  arithmetic standing in for a place (#523).
- A formula may stay where it is measurably cheaper or better behaved than the property, and it
  then says in a comment which property it computes.
- A lint names drift by meaning, not by counts, and it is advisory (#462). Today's drift lint
  (`program/drift.rs`) names none of these yet: that is open work, not a licence.

### 4. Solvent is to be useful to industry

The aim is real machines, carried from text to a checked fabrication file and into the tools
engineers already use.

- **The pilot is a real machine.** It is something well established, large in scope and
  universally useful, a tractor (#523). Its numbers are ones a person would state.
- **The whole path is certified, from source to file.** A short text is solved, swept and
  written to a checked fabrication file (#61). An export honours its stated tolerance, rounding
  included (#60, `fac61264`), and a reported number comes from the exact solid (`ed9c19db`). A
  successful export of incorrect geometry is a defect of the first priority (#58).
- **The output goes where engineers already work.** STEP carries the assembly with its product
  structure and colours (`77959aa0`, #677), and it opens in commercial CAD. The IDE builds
  assemblies as the export builds them and draws them as a CAD viewer draws them (#668).
- **Engineering knowledge lives in the library.** Standards are tables, such as an ISO 730 hitch
  category or an O-ring groove, and the design says "a groove for a #014" while the arithmetic
  is the library's (#48, #523). Quantities beyond length are first class and checked, such as a
  torque, a power or a flux (#311, #323). The everyday checks are matched, such as interference
  across a whole assembly (#539).

#### Solvent describes assemblies of arbitrary size

A tractor is as much a Solvent document as a bracket (#523).

##### Functional components

Functional in the functional programming language sense: a component is a function of its
definition and its inputs only, and a function of particular inputs can be cached.

- **A definition is a function of its arguments and its frame** (#523, #562). It is closed: it
  reads its formals, its own declarations and its file's constants, and nothing else. Its rows
  place everything it makes (E108), and it states no fact about its operands alone (E109).
  Inside, Solvent stays relational among a scope's unknowns. At the boundary it is functional,
  and an argument a call leaves unbound is a freedom the parent solves.
- **Caching follows from that.** Referential transparency gives reuse: the same definition with
  the same arguments is the same figure, evaluated once and placed many times (#566).
  Parametricity gives equivariance as a scope rule, not as a check. A solid that has only moved
  rigidly is the same solid under a new placement (#412). So the cost of a document grows with
  its distinct bindings, not with its instances, and anything that grows faster is a defect
  (#403).
- **Boundaries exist so a design can be read and trusted a part at a time.** Speed is a
  consequence to be measured, never the reason: solving through boundaries was measured no faster
  than the flat solve (#459, #465).

##### Frames and frame composition are fundamental concepts

- **Frames are the unit of placement and of motion.** A plane is a frame, a motion is a frame
  that moves, and an instance is a frame (#383, #548).
- **Frames compose by nesting.** A nested frame's pose is the product of its joints. A mate is a
  relation between frames' datums.
- **A frame stays right-handed.** Mirroring is an operation on a figure (#558).

#### The CAD kernel should be robust to all target geometries

Tangents, NURBS, solid sweeps, lofts, offsets: the hardest of the hard. Blends are where
commercial kernels fail most (#66), and that is the reason to get them right. The kernel serves
any rigid trajectory, so no author writes a constant-rate joint to get exact geometry (#721).

#### Mechanisms and motion are in scope

Solvent relates a mechanism to its movement. Gear meshing, cams, linkages, screws and the Wankel
are cases of one construct, a body read under a motion, and each new mechanism should cost a
definition, not a project (#64, #70). A conjugate wheel is `blank − sweep(star)`, one statement
(#590). The meshing law is a relation among rates, and tooth contact is a locus (#283, #365,
`6df000ab`). Freedom is counted the way a mechanism designer counts it: `1 DOF over t` for a
four-bar's crank (#391).

#### Elastic assemblies are in scope

Deformation is the general case of motion. A spring, a flexure or a snap fit works by deforming,
and its deformation is a field over its body just as a motion is. Structural performance is
design intent (#510):

- What a body is made of belongs to the body.
- A load scenario is drawn.
- An analysis output is a row of the solve, with its gradient from the adjoint.
- Its discretisation error is part of the answer.
- An ill-posed question is a diagnosis, not a number.

Today that is the linear, small-strain case. The elastica (#136), large deflection, contact and
compliant mechanisms are the same idea's next cases, not a separate program.

#### Electromagnetic and fluid mechanics constraints are in scope

A motor is sized by its current, power and torque, and a magnetic field or a fluid pressure is a
target the geometry is driven to meet. These follow the pattern structural analysis set: a field
is solved over the drawing's geometry, its output is a row the solve can be driven by, and what a
body is made of is stated on the body. The quantities are first class and checked by unit, such
as a voltage, a current or a flux (#311, #323).
