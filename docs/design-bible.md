# Design Bible

This document is the authority on design principles across the project. The contract files, the
specification and every epic are bound by it. If a rule in one of them conflicts with a principle
here, the rule is wrong and gets fixed.

Most of these principles were first stated to correct a piece of work that was going wrong. The
issue numbers and commits in parentheses point to where that happened, so a reader can see the
case that prompted each rule.

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

* **The Solvent language.** It is specified in [solvent-spec.md](../solvent-spec.md), whose §1.1
  sets out five language principles of its own, P1–P5:
  * Passing an entity to a component gives one entity two names; it adds no constraint.
  * Reordering the statements in a body never changes its meaning.
  * Deleting every seed (`hint`) never changes the set of solutions.
  * Component boundaries are a decomposition the solver may exploit.
  * `ring` asserts a symmetry; it does not copy statements.

  Geometry and constraints live in `.sv` files, and how they are displayed lives in `.svd`
  files. Views are derived from the model and never written into it (#523). A flag whose only
  effect is to show or hide geometry, or to set a colour, belongs in `.svd` (#462, #677).
* **The Solvent compiler, `solventc`.** It parses, elaborates, solves, diagnoses, reports and
  exports, using the engine in `rust/`. It writes no messages of its own: every line it prints
  comes from the core. An error is reported once, against the source statement that caused it,
  using the names the author wrote and never an internal id (#387).
* **The Solvent IDE.** The workspace in `web/src/app/`. The Solvent text is the design. A gesture
  in the IDE edits that text in place by inserting or changing Solvent statements, and never
  regenerates the file ([app.md](contract/app.md)). The IDE contains no geometry code.
* **The CAD kernel.** `rust/cad-kernel/` holds the exact boundary representation (B-rep), its
  Boolean operations, its mesher and its STEP export. It is written from scratch, licensed MIT OR
  Apache-2.0, and benchmarked against other kernels ([brep.md](contract/brep.md)). It has no
  knowledge of Solvent or of sets.

How they interact:

- **One dependency-free core holds every algorithm.** The layers above it (the C ABI, the
  TypeScript bindings, `solventc` and the IDE) add no geometry and write no messages
  ([architecture.md](contract/architecture.md)).
- **The same input always gives the same output.** The implementation uses only ordered
  containers and seeded random numbers.
- **Solvent is translated down to the kernel, never shaped by it.** A shape the kernel cannot
  build yet loses only its exact B-rep output. A document can still define it (principle 2).

## Guiding principles

### 1. Do it right even if it's hard

Solvent and its kernel are designed to be general and robust. This means doing the hard thing,
even if it takes work, even if it would entail years of engineering labour. If a goal is
theoretically possible and its blocker is engineering labour, we commit that labour. We do not
build a shortcut that the principled design will later have to tear out. Scope is a separate
question: work may be deferred or declined (#151, #154), but whatever is built is built the
principled way.

- **A shape that cannot be built yet is refused with a reason, never approximated.** The
  evaluator either builds a shape exactly or refuses it. A refusal names the reason and gives a
  witness, a specific point or entity that shows the problem, and every refusal is an open item
  of work. The evaluator never substitutes an approximation without saying so, and a limitation
  of the evaluator is never written into the language as a rule.
- **Success is reported only when it happened.** A solve that stopped before converging is
  reported as unsolved, and no claim is judged on the geometry it left behind (#449, #492). A
  silently wrong drawing is the worst failure, because every measurement taken from it is then
  wrong (#43, #588, #629). A solid missing from a rendered image or an exported file is an error,
  not a warning (#614).
- **Correctness is checked against independent references.** These are closed-form solutions,
  NAFEMS benchmarks, and OCCT and CalculiX, which run as oracles in the tests only
  ([analysis.md](contract/analysis.md), [brep.md](contract/brep.md)).
- **Measure, do not assume.** A belief about speed, cost or scaling is tested by measurement
  before anything is built on it (#459, `b6d7d0f5`). New syntax is added only after an experiment
  shows that the capability is worth exposing (#461, #465).

### 2. Solvent borrows the generality of math

Borrowing math's generality also brings its aesthetics, and one of those is terseness, which is
a reason in itself to adopt them. In set notation an offset is one line:
`component Offset(S: set, d: Length) := { p | p distance(d, side: outside) S }`. A document should
read the same way, saying what the thing is and little else. Length that carries no design is
ceremony, and ceremony is a defect (#46, #145).

#### Solvent's primitives are sets, from which geometry is derived in a standard library

Solvent expresses geometric concepts, and CAD concepts, as set math. A statement means the set of
points it defines. In Solvent, `distance` is a primitive and `sphere` is a derived term: a
one-line definition in `std.sets` (the points at a given distance from a centre).

- **Two spellings of one set give the same result.** A cutting tool written as a `common` or as a
  prism, faces split or merged, a motion written as one joint or as a frame along a path: each
  pair behaves identically. An author never has to rewrite a drawing to suit an algorithm.
- **The set's definition is the ground truth.** Exact constructions are tested against the
  definition evaluated directly, point by point (as a field, or by sampling). An exact B-rep is
  one way to evaluate a statement. It does not define the statement's meaning.
- **A drawing is a function of its parameters: trajectories, not snapshots** (#391). A constraint
  that depends on a parameter holds for every value of that parameter. A motion is a map of a
  body into space as its parameters vary, and rigid motion is the special case of that map.
- **A physical principle is stated, not solved by hand.** A hanging chain is stated as the curve
  of least potential energy. A light ray is stated by its path through a refractive medium, and
  #154 even solves for the medium. An advancing front is stated by its speed along its normal.
  These are variational problems, ordinary differential equations and first-order partial
  differential equations, and the document states the problem rather than its solution (#121,
  #151, #154, #339).

#### The CAD kernel never informs the shape or capabilities of Solvent

The design of a CAD kernel never drives Solvent's goals or aesthetics. That design includes its
surface types, its operations, its algorithms and their limits. A kernel is one way to evaluate a
set. A shape the kernel cannot build yet is a gap in the kernel, not a restriction on the
language.

- **The kernel and the language have different primitives.** In the kernel, `sphere` is a
  primitive and `distance` is a query. The kernel's `sphere()` does not give Solvent a `sphere`
  keyword, and no kernel capability enters the language just because the kernel has it.
- **The language knows nothing of the kernel's algorithms.** Algorithm-specific
  classifications, fast paths and special cases live in the evaluator. They never appear as
  syntax or as elaboration rules.
- **Lowering is evaluation, not translation.** "Lowering" means building a B-rep for a set. Most
  sets have no finite B-rep with closed-form faces: a gyroid, for example, or the envelope of a
  moving tool. A lowering identifies a set from its definition, never from the name of the
  library word used to write it. A round written out by hand, bounded by the same two faces and
  the same ball of centres as `std.rounds`' `round`, is built the same way as the library's. Each
  B-rep is checked against direct evaluation of the set. Lowering changes how a set is computed,
  never which points belong to it.
- **A gap in the kernel limits exact output, never the shapes a document can define.** A region
  the kernel cannot build is still displayed, meshed, measured and analysed by evaluating its
  definition directly. That direct-evaluation path gets the engineering effort it needs to stay
  fast enough for real use. Only the region's STEP export is refused.
- **The B-rep exists for topology, exact geometry, exchange formats such as STEP, and fast
  repeated queries.** Topology here means named faces and edges, such as the edge a round is
  applied to. The kernel is required, but it never decides a statement's meaning.

#### Solvent shrinks, it doesn't grow

The aim of math and physics is unification. Likewise, the aim of Solvent is to express more and
more physical concepts in terms of mathematical primitives, defining each specific concept in the
standard library.

- **A new capability arrives as a library definition written in Solvent.** It does not arrive as
  new grammar, a new constructor label or a new set of options (#243, #383, #645). The Rust core
  gains only general features, each with a use beyond the case that motivated it, plus general
  evaluators and lowerings (#183). The preferred language change gives a meaning to a construct
  the compiler currently rejects, rather than adding syntax.
- **One meaning, one spelling.** Two names for one idea make a worse language than one (#11,
  #76, #266). Two mechanisms that turn out to be one concept are merged: one notion of a moving
  frame, one derivative primitive, one kind of running parameter, one form of definition (#365,
  #366, #391, #562).
- **A constraint is written as a relation word, not called as a procedure.** A component that
  creates no geometry and only constrains its arguments should be a relation word, written
  `counter spins(-1) centre` rather than `std.Spin(counter, …)`. A bare component call standing
  alone as a statement gets a lint warning (#464, W202, W203).
- **Entities are declared plain and constrained afterwards.** An entity is declared with no
  options, and relation words then state the facts about it. The language does not grow a set of
  constructors that build entities already constrained (#383).
- **Each fact is stated once, and everything else is derived.** A declaration that restates a
  fact the body already implies can contradict it (#148). Redundant and regular structure is
  found from the rank of the system or the freedoms left in it, not from flags or code written
  for one shape (#87, #130, #145). The kernel follows the same rule: a vertex is computed once
  and shared, not computed twice and matched by tolerance (#711).
- **No backward compatibility while nothing is deployed.** When a spelling is retired, the old
  form becomes an error whose message names the replacement. The example corpus is rewritten in
  the same change, and no solved coordinate moves (#243, #383, #391).

#### Solvent is not beholden to CAD traditions

Solvent has no separate assembly level, no operation timeline and no features. Construction
geometry is first class, and 2D is a special case of 3D constraints.

- **There is no assembly level.** Mass and forces know nothing of assemblies, and programming
  languages have no special "assembly" level of functions. Composition replaces it. A tractor is
  a component whose body places other components in frames related to one another, and the
  designer decides where the component boundaries fall (#523). Mass, volume and cost add up over
  bodies, so their totals come out the same however the designer groups them. A bill of
  materials is a report computed from the document, not a structure declared in it.
- **There is no operation timeline.** A document is a set of statements, not a history of
  operations (principle 3).
- **There are no features.** A round, a chamfer or a loft is a standard-library word that defines
  a set. It is not an operation applied to a model and recorded in a history (#183, #243).
- **Construction geometry is first class.** A line drawn only to state a constraint is an entity
  like any other, constrained and solved like any other. Geometry added only to state a claim
  never adds constraints to the model (#460).
- **2D is a special case of 3D** (#523, #558). A 2D statement is the 3D statement made inside a
  plane, with angles measured about that plane's normal.

### 3. Solvent encodes design intent

Design intent usually lives in documents outside CAD. An analysis phase works out the geometry
before the CAD phase begins, and CAD then implements the results of that analysis. In Solvent the
design is encoded in the document, and the solver produces the geometry.

- **Every number in a document is one a designer would put on a drawing.** It is a position
  along an axis, a diameter, a ratio or a lead, dimensioned as the shop would dimension the part
  (#309, #383). A value tuned until the picture looked right is a result passed off as an input
  (#283). Numbers that only steer the evaluation, such as a roll limit or a mesh tolerance, are
  not design data (#271). A measurement of the solved geometry is never an input. It belongs in a
  claim, which checks it (#312, #366).
- **State the defining property, not its closed-form solution.** A curve is stated by its
  definition: a taut string unwinding from a circle, a least energy, an envelope, a meshing law.
  It is never stated by a formula someone derived on paper (#3, #121, #283).
- **Prefer properties that hold by construction over checks that can fail.** When the defining
  relation is stated, the property follows from it. A gear flank cut as the envelope of its mate
  is conjugate by definition. A generating roll whose extent follows from the blank it cuts needs
  no stated limit (#271, #283).
- **Choose solution branches with words, not with seeds or signs.** When a constraint has more
  than one solution (two roots, two sides, two directions of turn), the intended one is stated,
  with `ccw`, `sense:`, a bound or a `Side` argument, and the solver holds it. A seed never makes
  that choice. A stated choice still holds after the numbers change, whereas a seed may then land
  on a different root (#145, #217, #157, #503, `857b1bd8`).
- **Properties the design should have are claims.** A theorem, a clearance or a symmetry that the
  design is expected to satisfy is checked against the solved geometry. It is never added as a
  constraint (#7, `d7dae876`).
- **Requirements are stated in the document.** A deflection limit, a payload delta-v or a rule of
  a building code is written beside the geometry it governs. Whether the implementation checks a
  requirement as a claim, drives the geometry to meet it, or holds it as a bound is a choice of
  mechanism, and the designer never restates a requirement to suit one (#510).
- **Dragging geometry explores the design without changing it.** Dragging a point, a curve or a
  frame in the IDE moves the drawing to another solution of the same statements. It edits no
  statement, and the only text it writes back is seeds, which do not affect the solution set
  (#2, #12). Changing a dimension is an edit, and the IDE makes it as one.

#### Solvent is declarative, not imperative

Solvent has no timeline of operations, and it does not describe a sequence of imperative
operations in emulation of a machining or assembly plan. Fabrication and assembly are a separate
concern. Solvent documents are sets of facts stated in arbitrary order, and the solver's job is
to find the geometry.

- **Statements are unordered** (P2). Meaning never comes from the layout, order or position of
  the source text. Where two chain links join is marked explicitly with `->` rather than inferred
  from adjacency (#31). Which points of two cross-sections correspond is stated, not guessed
  (#243). Any state the document stores is attached to the statement it belongs to, never to a
  position or an index (#2, #141).
- **State the function, not the manufacturing method.** The document states the functional
  problem: the meshing a gear engineer specifies, not the machine settings that cut the gear. The
  choice of a manufacturable surface family, such as face-milled teeth, is a second layer, and
  the solver picks its machine settings, not the author (#283).

#### Trigonometry and cartesian constraints are lints

Express ideas in terms of geometric constraints, not trigonometry.

- `cos` and `sin` of an angle compute a rotation by hand in one particular frame. The rotation
  itself, which is the intent, is lost (#96, #366).
- A coordinate typed in places a part relative to the world origin instead of the parts it
  relates to. Parts are placed by mates, not by offsets. An offset chain such as
  `zA = fwA + D / 2` is a mate that has to be updated by hand whenever `D` changes (`817e61ed`).
- A seed is written relative to existing geometry, as in `hint(at: …, toward: …, turn: …)`. It
  is not computed by coordinate arithmetic such as `pin.y + 26mm` (#523).
- A formula may stay where it is measurably cheaper or better behaved than the property it
  computes. A comment beside it then names that property.
- Lints flag specific patterns, not counts of constructs, and they are warnings, not errors
  (#462). The current lint pass (`program/drift.rs`) flags none of these patterns yet. That is
  open work, not permission to write them.

### 4. Solvent is to be useful to industry

The aim is real machines, taken from text to a checked fabrication file and into the tools
engineers already use.

- **The pilot project is a real machine.** It should be well established, large in scope and
  universally useful, and the choice is a tractor (#523). Its dimensions are ones a designer
  would state.
- **The whole path from source to file is checked.** A short text is solved, swept and written to
  a fabrication file that has been checked (#61). An export stays within its stated tolerance,
  including floating-point rounding in the file format (#60, `fac61264`). A reported area or
  volume is computed from the exact solid (`ed9c19db`). Successfully exporting incorrect geometry
  is a top-priority defect (#58).
- **The output goes where engineers already work.** A STEP export carries the assembly's product
  structure and colours (`77959aa0`, #677), and it opens in commercial CAD. The IDE builds
  assemblies the same way the export does and displays them the way a CAD viewer does (#668).
- **Engineering knowledge lives in the library.** Standards are tables, such as the ISO 730
  hitch categories or O-ring groove sizes. The design says "a groove for a #014 O-ring" and the
  library does the arithmetic (#48, #523). Quantities beyond length, such as torque, power and
  magnetic flux, are first class and their units are checked (#311, #323). The checks engineers
  rely on in commercial CAD are provided, such as interference across a whole assembly (#539).

#### Solvent describes assemblies of arbitrary size

A tractor is as much a Solvent document as a bracket (#523).

##### Functional components

Functional in the functional programming language sense: a component is a function of its
definition and its inputs only, and a function of particular inputs can be cached.

- **A component is a function of its arguments and of the frame it is placed in** (#523, #562).
  Its scope is closed: it reads only its arguments, its own declarations and its file's
  constants. Its constraints fully determine every entity it creates (E108), and it states no
  constraint among its arguments alone (E109). Inside a component, constraints relate its
  unknowns to one another as usual. At its boundary it behaves as a function: an argument the
  call leaves unbound becomes an unknown the parent solves for.
- **Caching follows from that.** The same definition with the same arguments always produces the
  same figure, so it is evaluated once and placed many times (#566). A body can reach its frame
  only through that frame's standard datums, so moving the frame moves the result rigidly with
  it. That follows from the scoping rule and needs no check. A solid that has only been moved
  rigidly is the same solid in a new place (#412). The cost of a document therefore grows with
  its number of distinct (definition, arguments) pairs, not with its number of instances, and
  anything that grows faster is a defect (#403).
- **Component boundaries exist so a design can be read and trusted one part at a time.** Speed is
  a side effect to be measured, never the reason for a boundary. Solving across boundaries was
  measured to be no faster than solving the whole system at once (#459, #465).

##### Frames and frame composition are fundamental concepts

- **Frames are the unit of placement and of motion.** A plane is a frame, a motion is a frame
  that moves, and a component instance is a frame (#383, #548).
- **Frames compose by nesting.** The pose of a nested frame is the composition of the joint
  transforms above it. A mate is a constraint between the datums (origin, axes and planes) of two
  frames.
- **Every frame is right-handed.** A mirror image is made by reflecting the figure, not by using a
  left-handed frame (#558).

#### The CAD kernel should be robust to all target geometries

Tangents, NURBS, solid sweeps, lofts, offsets: the hardest of the hard. Blends are where
commercial kernels fail most often (#66), which is why we have to get them right. The kernel
sweeps a solid exactly along any rigid trajectory, so an author never has to rewrite a motion as
constant-rate joints to get exact geometry (#721).

#### Mechanisms and motion are in scope

Solvent relates a mechanism to its movement. Gear meshing, cams, linkages, screws and the Wankel
engine are all cases of one construct: a body swept or traced as it moves. Supporting a new kind
of mechanism should take a library definition, not a new engine project (#64, #70). A gear
generated by its mating cutter is the blank minus the cutter's sweep, written as one statement:
`blank − sweep(star)` (#590). The law of gearing is a constraint on velocities (`rate(…)`), and
the path of tooth contact is the locus of the contact points (#283, #365, `6df000ab`). Degrees
of freedom are counted the way a mechanism designer counts them: a four-bar linkage has
`1 DOF over t`, its crank angle (#391).

#### Elastic assemblies are in scope

Deformation is the general case of motion. A spring, a flexure or a snap fit works by deforming,
and its deformation is a field over its body, just as a rigid motion is. Structural performance
is design intent (#510):

- A body's material is declared on the body.
- Load cases (supports and loads) are stated in the document.
- An analysis result, such as a deflection, is a constraint equation the solver can drive, with
  its gradient computed by the adjoint method.
- The estimated discretisation error is reported as part of every result.
- A question with no meaningful answer, such as the peak stress at a sharp re-entrant corner, is
  reported as a diagnosis rather than answered with a number that depends on the mesh.

Today this covers linear, small-strain analysis. The elastica (#136), large deflection, contact
and compliant mechanisms are extensions of the same approach, not a separate system.

#### Electromagnetic and fluid mechanics constraints are in scope

A motor is sized by its current, power and torque, and a magnetic field strength or a fluid
pressure can be a target the geometry is driven to meet. These follow the pattern that structural
analysis set: a field is solved over the drawing's geometry, its results are constraint equations
the solver can drive, and material properties are declared on the body. These quantities, such
as voltage, current and magnetic flux, are first class and their units are checked (#311, #323).
