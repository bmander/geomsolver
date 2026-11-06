# Design Bible

This document is the constitution of Solvent's design. The specification, the contract files and
every epic are bound by it. Where a rule in one of them conflicts with a principle here, the rule
is wrong and is fixed.

The headings and the bold sentences are binding; the text after each bold sentence explains it.
This document states principles and their main consequences, not the language, which the
[primer](solvent-primer.md) and the [specification](../solvent-spec.md) describe. Issue numbers and
commit hashes in parentheses point to the work whose correction first prompted a principle.

## Project purpose

Solvent is a language for describing physical geometry in terms of design intent. Design intent
means the requirements and relations that explain why an object has its shape, such as a
deflection limit, a fit with another part or a gear ratio.

### Examples

* Driving 2D and 3D constraints, as traditional sketch solvers do
* A beam sized according to deflection requirements
* An electric motor driven by current and power requirements
* Driving targets for magnetic fields, fluid pressure, etc.
* A rocket assembly with a particular payload delta-v
* A building of particular size and shape, compliant with building code

## Terms

Each term below has one meaning throughout.

- **Document**: a Solvent source file (`.sv`), the canonical form of a design. Presentation lives
  in a separate drawing file (`.svd`).
- **Statement**: one declaration or relation in a document.
- **Entity**: a point, line, curve, plane, frame, face or solid that a document declares.
- **Set**: the points or values that satisfy a condition, written `{ p | condition }`.
- **Dimension**: a number the designer states. An **unknown** is a number the solver finds.
- **Running parameter**: a number declared over an interval, such as a crank angle. The design
  holds at every value in the interval.
- **Constraint**: a statement that restricts the solution set. A constraint *drives* the geometry:
  the solver changes unknowns to satisfy it.
- **Solution set**: every assignment of the unknowns that satisfies all the constraints. A
  **solution** is one member; the **solved geometry** is the solution the solver reports.
- **Degree of freedom**: an independent direction in which the solution set lets the geometry vary.
- **Claim**: a statement checked against the solved geometry. It never drives.
- **Requirement**: a statement of performance the design must meet, such as a deflection limit.
- **Seed**: a starting value for an unknown, written with `hint`.
- **Component**: a named definition with arguments. An **instance** is one use of a component,
  placed in a frame.
- **Frame**: an origin with three right-handed axes and the planes between them. A **mate** is a
  constraint between the origins, axes or planes of two frames.
- **Standard library** (`std`): definitions written in Solvent, available to every document.
- **B-rep** (boundary representation): a solid stored as faces, edges and vertices with exact
  geometry. **Lowering** is building a B-rep from a set's definition.
- **Field**: a quantity defined at every point of a body, such as displacement or temperature.
- **Diagnosis**: an explanation, in terms of a document's statements, of why the document has no
  solution or why a question about it has no answer.
- **Witness**: a specific point, entity or value that demonstrates a failure.
- **Lint**: a warning about how a document is written.

## Components

Solvent has five principal components.

- **The Solvent language**, specified in [solvent-spec.md](../solvent-spec.md) and implemented in
  `rust/gcs-core/`. Views, colors and visibility live in drawing files, never in documents (#462,
  #523, #677).
- **The solver** (`rust/gcs-solve/`), which finds solutions and diagnoses documents that have none.
  Its **evaluator** computes meshes, B-reps, measurements and analyses from definitions. The solver
  knows nothing of Solvent's syntax.
- **The CAD kernel** (`rust/cad-kernel/`): the B-rep, Boolean operations, meshing and export to
  STEP, the standard CAD exchange format. It is written from scratch, to be published on its own,
  and knows nothing of Solvent or of sets.
- **The Solvent compiler, `solventc`**, which parses, elaborates, solves, diagnoses, reports and
  exports. Every message it prints comes from the core, and an error is reported once, against the
  statement that caused it, in the names the author wrote (#387).
- **The Solvent IDE** (`web/src/app/`), which edits the document in place, inserting or changing
  statements and never regenerating the file ([app.md](contract/app.md)). It contains no geometry
  code.

How they interact:

- **One dependency-free core holds every algorithm.** The core is the language implementation, the
  solver and the kernel. The layers above it add no geometry and write no messages
  ([architecture.md](contract/architecture.md)).
- **The same input always gives the same output.** The implementation uses only ordered containers
  and seeded random numbers.
- **Solvent is lowered to the kernel, never shaped by it** (principle 2).

## Guiding principles

### 1. Do it right even if it's hard

Solvent and its kernel are meant to be general and robust. When a goal is possible in principle
and the obstacle is engineering labor, even years of it, the project commits the labor instead of
building a shortcut that the principled design would later remove. Work may be deferred or declined
(#151, #154), but whatever is built is built the principled way.

- **A shape that cannot be built yet is refused with a reason, never approximated.** The evaluator
  computes a shape exactly or to a stated tolerance, or refuses it with a reason and a witness.
  Approximating means substituting a different shape; a mesh computed to a stated tolerance is not
  an approximation. Each refusal is an open item of work, and no limitation of the evaluator
  becomes a rule of the language.
- **Success is reported only when it happened.** A solve that stopped before converging is reported
  as unsolved, and no claim is judged on its geometry (#449, #492). Silently wrong geometry is the
  worst failure, because every measurement taken from it is wrong (#43, #588, #629). A solid
  missing from an image or an export is an error (#614).
- **Correctness is checked against independent references:** closed-form solutions, published
  benchmarks such as NAFEMS's, and other implementations (OCCT, CalculiX) used only in tests.
- **Measure, do not assume.** A belief about performance or scaling is measured before anything is
  built on it (#459, `b6d7d0f5`). New syntax waits for an experiment showing that it is worth
  exposing (#461, #465).

### 2. Solvent borrows the generality of math

Solvent adopts the generality and the terseness of mathematical notation. An offset is one line:
`component Offset(S: set, d: Length) := { p | p distance(d, side: outside) S }`. Text that carries
no design is ceremony, and ceremony is a defect (#46, #145). Terseness never leaves a design
decision to inference (principle 3).

#### Solvent's primitives are sets, from which geometry is derived in a standard library

Each constraint defines a set: the points or values that satisfy it. `distance` is a primitive,
and a sphere is a one-line library definition, the points at a given distance from a center.

- **Two spellings of one set give the same result.** A cutting tool written as an intersection or
  as a prism, faces split or merged, a motion written as one joint or as a frame along a path:
  each pair gives identical results, so an author never rewrites a document to suit an algorithm.
- **The set's definition is the ground truth.** Every computed form of a set, such as a B-rep or a
  mesh, is tested against the definition evaluated directly at sample points.
- **A design is a function of its parameters: trajectories, not snapshots** (#391). Solved geometry
  is a function of the document's dimensions and running parameters. A constraint that depends on a
  running parameter holds at every value of it, so a solution is a path rather than one
  configuration, and a motion maps a body into space as the parameter varies.
- **A physical principle is stated, not solved by hand.** A hanging chain is the curve of least
  potential energy; a light ray is the path of least time, and #154 solves for the medium it
  crosses. The document states the variational problem or the differential equation, not its
  solution (#121, #151, #339).

#### The CAD kernel never informs the shape or capabilities of Solvent

The kernel's surface types, operations, algorithms and limits never set Solvent's goals or
aesthetics.

- **The kernel and the language have different primitives.** In the kernel a sphere is a primitive
  and distance is a query. No kernel capability enters the language because the kernel has it.
- **The language knows nothing of the kernel's algorithms.** Special cases and fast paths for
  particular shapes live in the evaluator, never in the language.
- **Lowering is evaluation, not translation.** Most sets, such as a gyroid or the envelope of a
  moving tool (the surface the tool touches at every position), have no B-rep with closed-form
  faces. The evaluator recognizes a set by its definition, never by the name of the definition used
  to write it: a round written out by hand is built the same way as the library's round. Lowering
  changes how a set is computed, never which points belong to it.
- **A gap in the kernel limits exact output, never the shapes a document can define.** A set the
  kernel cannot build is still displayed, meshed, measured and analyzed by evaluating its
  definition directly, and that evaluation is engineered to stay fast. Only output that requires an
  exact B-rep, such as STEP, is refused.
- **The B-rep exists for topology, exact geometry, exchange formats such as STEP, and fast
  repeated queries.** Topology means named faces and edges, such as the edge a round is applied to.

#### Solvent shrinks, it doesn't grow

Mathematics and physics progress by unification. Solvent likewise expresses ever more physical
concepts with the same few primitives, defining each specific concept in the standard library.

- **A new capability arrives as a library definition written in Solvent,** not as new grammar,
  constructor arguments or options (#243, #383, #645). The core gains only general features and
  methods of evaluation, each useful beyond the case that motivated it (#183). The preferred
  language change gives a meaning to a construct the compiler currently rejects.
- **One meaning, one spelling.** Two names for one concept make a worse language (#11, #76, #266).
  Mechanisms that turn out to be one concept are merged: one notion of a moving frame, one
  derivative, one kind of running parameter, one form of definition (#365, #366, #391, #562).
- **A constraint is written as a relation word, not called as a procedure.** A relation word is
  written between its operands. A component that only constrains its arguments becomes one:
  `counter spins(-1) center`, not `std.Spin(counter, …)` (#464).
- **Entities are declared plain and constrained afterwards.** A declaration takes no options;
  relation words then state the facts about the entity (#383).
- **Each fact is stated once, and everything else is derived.** A restated fact can contradict the
  original (#148). Redundancy and regular structure are found from the rank of the equations, not
  from flags or shape-specific code (#87, #130, #145). The kernel computes a shared vertex once,
  rather than twice with the results matched within a tolerance (#711).
- **No backward compatibility while nothing is deployed.** A retired spelling becomes an error that
  names its replacement, and the examples are rewritten in the same change without moving a solved
  coordinate (#243, #383, #391).

#### Physics is built from primitives, as geometry is built from sets

Solvent states physical behavior with a few primitives, and the standard library builds the
familiar forms from them, as it builds a sphere and an offset from `distance`.

- **Each familiar form is a short library definition.** An electric circuit, a truss, heat
  conduction, elasticity, magnetostatics and viscous flow each take a few lines, and so does each
  material law.
- **The primitives cover physics that minimizes no energy.** Elasticity minimizes an energy;
  viscous flow and friction do not, and the primitives state both.
- **One formulation serves networks and continuous bodies.** A truss and a solid, or a magnetic
  circuit and a motor's magnetic field, are one physics at two levels of detail: a design is sized
  with the network and checked with the continuous body.
- **The evaluator recognizes structure from the definition.** As with sets, the evaluator
  recognizes a problem's structure, such as a symmetric positive-definite system, from the stated
  physics, never from a definition's name. A beam element is one way to evaluate the
  primitives.
- **A primitive is adopted by experiment.** Before a primitive enters the language, the familiar
  forms are written with it and checked against independent references.

#### Solvent is not beholden to CAD traditions

- **There is no assembly level.** Mass and forces know nothing of assemblies, and a programming
  language needs no special construct for a large program, because functions compose. A machine is
  a component whose instances are placed in frames related to one another, with component
  boundaries where the designer chooses (#523). Totals such as mass and cost do not depend on how
  bodies are grouped, and a bill of materials is a report computed from the document.
- **There is no operation timeline** (principle 3).
- **There are no features.** A round, a chamfer or a loft is a library definition of a set, not an
  operation recorded in a history (#183, #243).
- **Construction geometry is first class.** An entity drawn only to state a constraint is solved
  like any other. Geometry declared inside a claim adds no constraint (#460).
- **2D is a special case of 3D** (#523, #558). A 2D statement is the 3D statement made in a plane,
  with angles measured about the plane's normal.

### 3. Solvent encodes design intent

Conventionally, design intent lives in reports and spreadsheets, and CAD records geometry worked
out elsewhere. In Solvent the document states the intent, and the solver produces the geometry.

- **Every number in a document is one a designer would state.** Positions along axes, diameters,
  ratios, leads and loads are given as the shop would dimension the part (#309, #383). A value
  tuned until the picture looked right is a result disguised as an input (#283). A number that only
  controls evaluation, such as a mesh tolerance, is not design data (#271), and a measurement of the
  solved geometry belongs in a claim (#312, #366). Seeds guide the solver and state no design.
- **State the defining property, not its closed-form solution.** A curve is stated by its
  definition (a taut string unwinding from a circle, a least energy, an envelope, a meshing law),
  never by a formula derived on paper (#3, #121, #283). The exception is a formula measurably
  cheaper or better behaved than the property, with a comment naming the property.
- **Prefer properties that hold by construction over checks that can fail.** A gear flank cut as
  the envelope of its partner's flank meshes correctly by definition. A generating motion whose
  extent follows from the blank it cuts needs no stated limit (#271, #283).
- **Choose solution branches with words, not with seeds or signs.** When constraints admit
  discrete alternatives (two roots, two sides, two directions of turn), the intended one is stated
  with a word such as `ccw`, a bound or a side, never chosen by a seed or by a number's sign. A
  stated choice survives a change of dimensions; a seed may then land on another root (#145, #157,
  #217, #503, `857b1bd8`).
- **Properties the design should have are claims.** A theorem, a clearance or a symmetry the design
  is expected to satisfy is checked, never added as a constraint (#7, `d7dae876`). A property that
  is the reason for the shape is stated as a constraint; a property expected to follow from those
  reasons is claimed.
- **Requirements are stated in the document.** A deflection limit, a payload delta-v or a
  building-code rule is written beside the geometry it governs. Checking it, driving dimensions
  with it or holding it as a bound is the implementation's choice, and the designer never rewrites
  a requirement to suit one (#510).
- **Dragging geometry explores the design without changing it.** A drag moves the solved geometry
  to another solution along degrees of freedom the document leaves open, writing back only seeds
  (#2, #12). Changing a dimension is an edit, and the IDE makes it as one.

#### Solvent is declarative, not imperative

A document is an unordered collection of facts, not a machining or assembly sequence; fabrication
planning is a separate concern.

- **Statements are unordered.** Meaning never depends on the order or position of text: where two
  segments join is marked with `->` (#31), and which points of two cross-sections correspond is
  stated (#243). Data a document stores, such as a seed, is attached to its statement, never to a
  position or an index (#2, #141).
- **Deleting every seed never changes the solution set.** Seeds may change which solution the
  solver finds, or whether it converges. Anything that selects among solutions is a constraint.
- **State the function, not the manufacturing method.** The document states the meshing a gear
  engineer specifies, not the machine settings that cut the gear; for a manufacturable surface
  family, such as face-milled teeth, the solver finds the settings (#283).

#### A design is a set of acceptable parts

A manufactured part never matches its nominal geometry exactly. The tolerances and fits that decide
which parts are acceptable are design intent, as dimensions are.

- **A tolerance is a set.** A toleranced dimension is an interval, and a geometric tolerance zone is
  a set of points built from `distance`, such as the band around a nominal surface. The solver
  finds and displays the nominal geometry, one member of the set.
- **Claims and requirements hold for every acceptable part.** Worst case is the default; a
  statistical claim states its distribution and confidence. A tolerance, like a running parameter,
  ranges over an interval, but a moving part takes every value in turn and a manufactured part
  takes one unknown value.
- **Variation in conditions is stated the same way.** A range of loads, a material within its
  specification or a range of temperatures is a set, and a requirement holds over all of it.
- **An assembly must go together for every combination of acceptable parts.** For each
  combination, a placement satisfying the mates must exist, and the claims must hold in it.
- **A failed claim gives its witness:** the part, or combination of parts, that breaks it.

#### Trigonometry and cartesian constraints are lints

Ideas are expressed as geometric relations, not as trigonometry or coordinates.

- `cos` and `sin` compute a rotation by hand in one frame, and the rotation, which is the intent,
  is lost (#96, #366).
- A literal coordinate places a part relative to the world origin instead of the parts it relates
  to; a sum of offsets such as `zA = fwA + D / 2` is a mate maintained by hand (`817e61ed`).
- A seed is written relative to existing geometry, as in `hint(at: …, toward: …)`, not by
  arithmetic such as `pin.y + 26mm` (#523).
- Lints flag specific patterns, not counts of constructs, and are warnings, not errors (#462).

### 4. Solvent is to be useful to industry

The aim is real machines, carried from text to a checked fabrication file and into the tools
engineers already use.

- **The whole path from source to file is checked.** A document is solved, built into solids and
  written to a fabrication file that is itself checked (#61). The file stays within its stated
  tolerance, including the format's rounding (#60, `fac61264`). Areas and volumes are computed from
  the exact solid (`ed9c19db`). Exporting wrong geometry without an error is a top-priority defect
  (#58).
- **The output goes where engineers already work.** A STEP export carries the assembly's product
  structure and colors and opens in commercial CAD (`77959aa0`, #677). The IDE displays assemblies
  as a CAD viewer does (#668).
- **Engineering knowledge lives in the library.** Standards are tables, such as ISO limits and
  fits or O-ring groove sizes: a design says "a groove for a #014 O-ring", and the library does the
  arithmetic (#48). Quantities such as torque, power, voltage and magnetic flux are
  first class, and their units are checked (#311, #323). The checks engineers rely on in commercial
  CAD, such as interference across an assembly, are provided (#539).

#### Solvent describes assemblies of arbitrary size

A machine of thousands of parts is as much a Solvent document as a bracket (#523).

##### Functional components

"Functional" is meant in the functional-programming sense: a component's result depends only on
its definition and its inputs, so the result for given inputs can be cached.

- **A component is a function of its arguments and of the frame it is placed in** (#523, #562). Its
  statements refer only to its arguments, its own declarations and its file's constants. Its
  constraints determine every entity it creates, and it states no constraint among its arguments
  alone. Passing an entity as an argument gives it a second name and adds no constraint; an
  argument left unbound becomes an unknown of the caller.
- **Caching follows from that.** The same definition with the same arguments gives the same
  geometry, so it is evaluated once and placed many times (#566). A component refers to its frame
  only through the frame's origin, axes and planes, so moving the frame moves the result rigidly,
  without evaluating it again (#412). Evaluation time grows with a document's distinct pairs of
  definition and arguments, not with its instances; faster growth is a defect (#403).
- **Component boundaries exist so a design can be read and trusted one part at a time.** The
  solver may exploit them, but speed is never the reason for a boundary (#459, #465).

##### Frames and frame composition are fundamental concepts

- **Frames are the unit of placement and of motion.** A plane is a frame, an instance is placed in
  a frame, and a rigid motion is a frame whose placement depends on a running parameter (#383,
  #548).
- **Frames compose by nesting.** A nested frame's placement is the composition of the placements of
  the frames that contain it.
- **Every frame is right-handed.** A mirror image is made by reflecting the geometry, not by using a
  left-handed frame (#558).

#### The CAD kernel should be robust to all target geometries

The targets include tangent blends, NURBS surfaces, swept solids, lofts and offsets, the hardest
cases for any kernel. Robust means each is built correctly or refused with a witness, never built
wrong. Blends are where commercial kernels fail most often (#66), so Solvent's must get them right.
The kernel sweeps a solid exactly along any rigid motion, so no author rewrites a motion to get
exact geometry (#721).

#### Mechanisms and motion are in scope

Solvent relates a mechanism to its movement. Gears, cams, linkages, screws and the Wankel engine
are cases of one construct: the volume a body sweeps, or the curve a point traces, as the body
moves. A new kind of mechanism takes a library definition, not a new engine project (#64, #70). A
gear generated by its cutter is one statement, the blank minus the volume the cutter sweeps:
`blank − sweep(star)` (#590). The law of gearing is a constraint on velocities (#283, #365,
`6df000ab`). Degrees of freedom are counted as a mechanism designer counts them: a four-bar linkage
has one (#391).

#### Elastic assemblies are in scope

Deformation is the general case of motion. A spring, a flexure or a snap fit works by deforming,
and its deformation, like a rigid motion, is a field over its body. Structural performance is
design intent (#510):

- A body's material is declared on the body.
- Supports and loads are stated in the document.
- An analysis result, such as a deflection, can drive dimensions, with its gradient computed by the
  adjoint method.
- Every result reports its estimated discretization error.
- A question with no meaningful answer, such as the peak stress at a sharp re-entrant corner, gets a
  diagnosis, not a number that depends on the mesh.

Large deflection, contact, the elastica (#136) and compliant mechanisms extend this approach.

#### Electromagnetic and fluid mechanics constraints are in scope

A motor is sized by its current, power and torque, and a magnetic field strength or a fluid
pressure can be a target the geometry is driven to meet. These follow the elastic case, with
their fields stated through the physics primitives.
