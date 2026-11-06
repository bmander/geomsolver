# Design Bible

This document is the constitution of Solvent's design. The specification, the contract files and
every epic are bound by it. Where a rule in one of them conflicts with a principle here, the rule
is wrong and is fixed.

**It states values, not definitions or rules.** The headings and the bold sentences are binding;
the text after each bold sentence explains it. What Solvent's words mean is the
[specification](../solvent-spec.md)'s to say, the language as the implementation accepts it is the
[primer](solvent-primer.md)'s, and how the implementation keeps a principle is the
[contract files](contract/)'. A sentence here that could be false of the mathematics or of the code
belongs in one of those instead. Issue numbers and commit hashes in parentheses point to the work
whose correction first prompted a principle.

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

## The parts

Solvent is a language, a solver, a CAD kernel, a compiler (`solventc`) and an IDE.
[architecture.md](contract/architecture.md) maps them and says what each may hold.

- **One dependency-free core holds every algorithm.** The layers above it add no geometry and
  write no messages.
- **The same input always gives the same output.**
- **An error is reported once, against the statement that caused it, in the names the author
  wrote** (#387).
- **The document is the design.** Presentation lives apart from it (#462, #523, #677), and a tool
  edits the document in place, never regenerating it.
- **The kernel knows nothing of Solvent, and Solvent is lowered to the kernel, never shaped by it**
  (principle 2).

## Guiding principles

### 1. Do it right even if it's hard

Solvent and its kernel are meant to be general and robust. When a goal is possible in principle
and the obstacle is engineering labor, even years of it, the project commits the labor instead of
building a shortcut that the principled design would later remove. Work may be deferred or declined
(#151, #154), but whatever is built is built the principled way.

- **A shape that cannot be built yet is refused with a reason, never approximated.** A shape is
  computed exactly, or to a tolerance that is stated, or it is refused with a reason and a witness.
  Each refusal is an open item of work, and no limitation of the evaluator becomes a rule of the
  language.
- **Success is reported only when it happened.** A solve that stopped short is reported as
  unsolved, and nothing is judged on its geometry (#449, #492). Silently wrong geometry is the
  worst failure, because everything measured from it is wrong (#43, #588, #629). Something missing
  from an output is an error (#614).
- **A result says how far to trust it, or why there is no answer.** A question with no meaningful
  answer gets a diagnosis, not a number (#510).
- **Correctness is checked against independent references,** never only against the
  implementation's own earlier answers.
- **Measure, do not assume.** A belief about performance or scaling is measured before anything is
  built on it (#459, `b6d7d0f5`). New syntax waits for an experiment showing that it is worth
  exposing (#461, #465).

### 2. Solvent borrows the generality of math

Solvent adopts the generality and the terseness of mathematical notation. Text that carries no
design is ceremony, and ceremony is a defect (#46, #145). Terseness never leaves a design decision
to inference (principle 3).

#### Solvent's primitives are sets, from which geometry is derived in a standard library

A constraint defines a set. Geometry is built from a few such primitives by definitions in the
standard library, not by the core.

- **Two spellings of one set give the same result.** What a document means is the set, however it
  is written, so an author never rewrites a document to suit an algorithm.
- **The set's definition is the ground truth.** Every computed form of a set is checked against
  its definition.
- **A design is a function of its parameters: trajectories, not snapshots** (#391). A design is
  stated over every value its running parameters take, not at one of them.
- **A physical principle is stated, not solved by hand.** The document states the principle, not
  a solution worked out from it on paper (#121, #151, #339).

#### The CAD kernel never informs the shape or capabilities of Solvent

The kernel's surface types, operations, algorithms and limits never set Solvent's goals or
aesthetics.

- **No kernel capability enters the language because the kernel has it.**
- **The language knows nothing of the kernel's algorithms.** Special cases and fast paths live in
  the evaluator, never in the language.
- **Lowering is evaluation, not translation.** The evaluator recognizes a set by its definition,
  never by the name of the definition used to write it. Lowering changes how a set is computed,
  not what the set is.
- **A gap in the kernel limits exact output, never the shapes a document can define.** A set the
  kernel cannot build is still shown, measured and analyzed from its definition.

#### Solvent shrinks, it doesn't grow

Mathematics and physics progress by unification. Solvent likewise expresses ever more physical
concepts with the same few primitives, defining each specific concept in the standard library.

- **A new capability arrives as a library definition written in Solvent,** not as new grammar or
  options (#243, #383, #645). The core gains only general features and methods of evaluation, each
  useful beyond the case that motivated it (#183). The preferred language change gives a meaning
  to a construct the compiler currently rejects.
- **One meaning, one spelling.** Two names for one concept make a worse language (#11, #76, #266).
  Mechanisms that turn out to be one concept are merged (#365, #366, #391, #562).
- **A constraint is written as a relation, not called as a procedure** (#464).
- **Entities are declared plain and constrained afterwards.** A fact about an entity is a
  statement of its own, not an option of its declaration (#383).
- **Each fact is stated once, and everything else is derived.** A restated fact can contradict the
  original (#148). Redundancy and structure are found from the facts themselves, not from flags or
  special cases (#87, #130, #145, #711).
- **No backward compatibility while nothing is deployed.** A retired spelling becomes an error that
  names its replacement, and the examples are rewritten in the same change (#243, #383, #391).

#### Physics is built from primitives, as geometry is built from sets

- **Each familiar form of physics is a short library definition,** as each material law is.
- **Physics is not limited to what minimizes an energy.**
- **A coarse model and a fine one of the same design are stated alike,** so a design can be sized
  with one and checked with the other.
- **The evaluator recognizes structure from the definition,** never from a definition's name.
- **A primitive is adopted by experiment.** Before a primitive enters the language, the familiar
  forms are written with it and checked against independent references.

#### Solvent is not beholden to CAD traditions

- **There is no assembly level.** A large design composes as a program does, with boundaries
  where the designer chooses them, and totals do not depend on how bodies are grouped (#523).
- **There is no operation timeline** (principle 3).
- **There are no features.** A round or a loft is a definition of a set, not an operation recorded
  in a history (#183, #243).
- **Construction geometry is first class** (#460).
- **2D is a special case of 3D** (#523, #558).

### 3. Solvent encodes design intent

Conventionally, design intent lives in reports and spreadsheets, and CAD records geometry worked
out elsewhere. In Solvent the document states the intent, and the solver produces the geometry.

- **Every number in a document is one a designer would state.** A value tuned until the picture
  looked right is a result disguised as an input (#283, #309, #383). A number that only controls
  evaluation is not design data (#271), a measurement of the result belongs in a claim (#312,
  #366), and seeds state no design.
- **State the defining property, not its closed-form solution** (#3, #121, #283). The exception is
  a formula measurably cheaper or better behaved than the property, with a comment naming the
  property.
- **Prefer properties that hold by construction over checks that can fail** (#271, #283).
- **Choose solution branches with words, not with seeds or signs.** Where the constraints admit
  discrete alternatives, the intended one is stated; a seed only says where the solver started
  (#145, #157, #217, #503, `857b1bd8`).
- **Properties the design should have are claims.** A property that is the reason for the shape is
  a constraint; a property expected to follow from those reasons is claimed (#7, `d7dae876`).
- **Requirements are stated in the document,** beside the geometry they govern, and the designer
  never rewrites a requirement to suit how it is evaluated (#510).
- **Exploring a design does not change it.** Moving the geometry within the freedoms the document
  leaves open is exploration; changing a dimension is an edit (#2, #12).

#### Solvent is declarative, not imperative

A document is an unordered collection of facts, not a machining or assembly sequence; fabrication
planning is a separate concern.

- **Statements are unordered.** Meaning never depends on the order or position of text, and what a
  document stores is attached to its statement, never to a position (#2, #31, #141, #243).
- **Deleting every seed never changes the solution set.** Anything that selects among solutions is
  a constraint.
- **State the function, not the manufacturing method** (#283).

#### A design is a set of acceptable parts

A manufactured part never matches its nominal geometry exactly. The tolerances and fits that decide
which parts are acceptable are design intent, as dimensions are.

- **A tolerance is stated as a set of acceptable values,** as a dimension is stated as one value.
- **Claims and requirements hold for every acceptable part,** and for every combination of
  acceptable parts in an assembly, unless the document states otherwise.
- **Variation in conditions is stated the same way.**
- **A failed claim gives its witness.**

#### Trigonometry and cartesian constraints are lints

Ideas are expressed as relations between the things they relate, not as trigonometry or
coordinates worked out by hand in one frame (#96, #366, #523, `817e61ed`). A document that does
otherwise is warned, not refused (#462).

### 4. Solvent is to be useful to industry

The aim is real machines, carried from text to a checked fabrication file and into the tools
engineers already use.

- **The whole path from source to file is checked.** Exporting wrong geometry without an error is a
  top-priority defect (#58, #60, #61, `fac61264`).
- **The output goes where engineers already work** (`77959aa0`, #668, #677).
- **Engineering knowledge lives in the library.** Standards are data the library reads, quantities
  carry units that are checked, and the checks engineers rely on in their tools are provided (#48,
  #311, #323, #539).

#### Solvent describes assemblies of arbitrary size

A machine of thousands of parts is as much a Solvent document as a bracket (#523).

##### Functional components

"Functional" is meant in the functional-programming sense: a component's result depends only on
its definition and its inputs, so the result for given inputs can be cached.

- **A component is a function of its arguments and of where it is placed** (#523, #562).
- **Caching follows from that.** The same definition with the same arguments gives the same
  geometry, so it is evaluated once and placed many times (#412, #566). Evaluation time grows with
  a document's distinct pairs of definition and arguments, not with its instances; faster growth
  is a defect (#403).
- **Component boundaries exist so a design can be read and trusted one part at a time.** The
  solver may exploit them, but speed is never the reason for a boundary (#459, #465).

##### Frames and frame composition are fundamental concepts

- **Placement and motion are expressed with frames** (#383, #548).
- **Frames compose by nesting.**

#### The CAD kernel should be robust to all target geometries

The targets include the hardest cases for any kernel, such as blends, where commercial kernels fail
most often (#66). Robust means each is built correctly or refused with a witness, never built wrong,
and no author rewrites a document to get exact geometry (#721).

#### Mechanisms and motion are in scope

Solvent relates a mechanism to its movement. Kinds of mechanism are cases of a few general
constructs, such as the volume a body sweeps as it moves, so a new kind takes a library definition,
not a new engine project (#64, #70, #590). Freedoms are counted as a mechanism designer counts them
(#391).

#### Elastic assemblies are in scope

Deformation is the general case of motion: a spring, a flexure or a snap fit works by deforming.
Structural performance is design intent (#510): what a body is made of, and how it is held and
loaded, are stated in the document, and a structural result can drive the geometry as any other
constraint does. Large deflection, contact, the elastica (#136) and compliant mechanisms extend
this approach.

#### Electromagnetic and fluid mechanics constraints are in scope

A motor is sized by its current, power and torque, and a magnetic field strength or a fluid
pressure can be a target the geometry is driven to meet.
