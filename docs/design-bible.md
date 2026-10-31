# Design Bible

The principles under every decision in this project. A contract file, the specification or an
epic is bound by them. Where a rule there conflicts with one, the principle governs, and the rule
is the defect.

## The two principles

**1. Take the correct, principled move, whatever it costs.** We are building a robust and generic
kernel. When a principled design and an expedient one part, we take the principled one, even if
it means extra work, even engineer-years of it. We do not build what the principled design will
have to throw away.

**2. Solvent expresses geometry; kernels serve it.** Solvent is fundamentally a way to express
geometric concepts as set math, and CAD concepts in those terms. The shape of a CAD kernel never
drives Solvent's goals or aesthetics. That shape means its surface types, its operations, its
methods and their limits. A statement means its set. A kernel is one way to evaluate that set,
and what it cannot build yet is the kernel's gap, not a rule of the language.

## What follows from them

- **A result depends on the set, not on how it was written.** Two spellings of one set behave
  alike: a tool drawn as a `common` or as a prism, faces split or merged, a motion written as one
  joint or as a frame along a path. No author restates a drawing to suit a method.
- **The definition is the reference.** The set read directly (a field, a sampled reading) is the
  truth that exact constructions are measured against. Exactness is how we evaluate, not what the
  statement means.
- **The language does not know the kernel's methods.** Classes, fast paths and special cases live
  in the evaluator and never surface as syntax or as elaboration rules.
- **What cannot be built yet is said, never guessed.** The evaluator either builds exactly or
  refuses by a named reason with a witness, and every such refusal is an open item of work. It
  never approximates silently, and it never moves the refusal into the language.
- **The engine is ours and whole.** One dependency-free core owns every algorithm. The layers
  above it (the ABI, the bindings, the CLI and the app) add no geometry and invent no wording.
  ([architecture.md](contract/architecture.md))
- **The same input gives the same output.** Ordered containers and seeded randomness only.
- **The model is not its presentation.** Geometry and constraints live in `.sv`, and how they
  are shown lives in `.svd`.
- **The language has its own principles.** See [solvent-spec.md §1.1](../solvent-spec.md), P1–P5:
  aliasing is not constraint, bodies are unordered, hints are inert, components are
  decomposition, symmetry is a claim.

## Solvent and its kernels

- **Lowering is evaluation, not translation.** Every B-rep defines a set, but most sets have no
  finite B-rep with closed-form faces (a gyroid, or the envelope of a moving tool). A lowering
  recognises a set by its definition, never by the name of the word that wrote it: a round
  written by hand, bounded by the same two faces and the same ball of centres as `std.rounds`'
  `round`, is built the same way. Its B-rep is checked against direct evaluation of the set, which
  tests membership point by point. Lowering changes how a set is computed, never which points
  belong to it.
- **The kernel and the language have different primitives.** In the kernel, `sphere` is a
  primitive and `distance` is a query; in Solvent, `distance` is the primitive and `sphere` is a
  derived term, a one-line definition in `std.sets`. Solvent is lowered to the kernel, never the
  reverse: `cad-kernel`'s `sphere()` does not give Solvent a `sphere` keyword, and no kernel
  feature enters the language because the kernel has it.
- **The kernel knows nothing of sets.** A gap in the kernel limits exact B-rep output, never the
  shapes a Solvent document can define: a region the kernel cannot build is still displayed,
  meshed and measured, and only its STEP export is refused.
- **The B-rep stays for topology, exact geometry, exchange formats such as STEP, and fast
  repeated queries.** Topology here means named faces and edges, such as the ones a round refers
  to. The kernel is required, but it does not decide what a statement means.
- **A shape with no lowering is fully supported.** It is displayed, meshed, measured and
  analysed by direct evaluation, and that path gets the engineering work to stay a practical
  choice.
