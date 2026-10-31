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
