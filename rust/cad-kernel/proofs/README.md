# What the certified layers claim, written to be proved

The kernel's robustness rests on three small layers of `src/util/`: intervals that enclose
(`interval.rs`), exact predicates (`expansion.rs`, `predicates.rs`) and certified root isolation
(`isolate.rs`). Their module documentation says what each guarantees. This directory says it
exactly, in [Rocq](https://rocq-prover.org) over [Flocq](https://flocq.gitlabpages.inria.fr)'s
IEEE-754, so that each guarantee is a statement a proof assistant can check a proof of.

**This is the first step: the statements, and four small proofs.** Everything else is stated and
unproved (`Admitted`), or stated as a property awaiting the function it is about. A statement not
yet proved may still be wrong in a detail; proving it is what finds out.

```bash
make        # check every statement and proof (Rocq 8.18+, Flocq 4: apt install coq libcoq-flocq)
```

```bash
make gaps   # run the kernel where it departs from a statement
```

Nothing here is linked into the kernel, built by its suite or shipped in the crate.

## The statements

| file | what it states | of |
|---|---|---|
| `Float.v` | the model of `f64`: binary64, round to nearest even | — |
| `Interval.v` | every operation that answers `Ok` holds the real result for every real its operands hold | the arithmetic, transcribed |
| `Predicates.v` | each error-free transformation's pair sums to the exact result | `two_sum` … `two_product`, transcribed |
| | `orient`, `power`, `orient2`, `in_circle` each return the sign of their determinant, over a stated range | a property |
| `Isolate.v` | a stretch `Slope` reads off holds no root; one read monotone runs one way | `Slope::read`, transcribed |
| | a box `Krawczyk` reads off holds no root; one read as one holds exactly one | a property |
| | `solve` answering no root means there is none; each crossing it returns lies in a box holding exactly one root | properties |
| | `solve` misses no root; `roots` answering no root means there is none | properties **the code does not meet** |

Three kinds of statement, told apart in the files:

- **Proved** (`Qed`): `rounded_is_bracketed` (a real lies between the floats either side of the
  float it rounds to, which is why `Interval::outward` is sound), `new_valid`, `ineg_encloses`, and
  a helper. They depend on nothing but the axioms of Coq's real numbers.
- **Stated of a transcription** (`Admitted`, 22): the Rust function is written out in Gallina by
  hand and the theorem is about that. The transcription is trusted until a translator replaces it.
- **A property** (`Definition … : Prop`, 10): what any function standing for the Rust one must
  satisfy. The filter and expansions of the predicates, `sin_cos`, the Krawczyk test and the two
  searches are too long to transcribe by hand with confidence.

## What writing them down found

Each is reproduced by `make gaps` against the crate as it stands.

1. **The predicates are exact only in range.** `predicates::orient` answers 0 (coplanar) for a
   positively oriented tetrahedron of side 1e-108, and of side 1e103: the determinant underflows,
   or overflows, in the filter and in the expansions alike. `expansion.rs` says it assumes neither
   happens; nothing checks. No drawing is near either scale, so the statements carry a range
   (`tame`: whole multiples of 2⁻¹⁵⁰ up to 2¹⁵⁰) and nothing else changes.
2. **`isolate::solve` drops a box whole for a run that covers part of it.** When a run is proved
   through the middle of the box being read, the box is not read again, though it may reach past
   the run's ends. A function nought over `[0.4, 0.6]` with a crossing at 0.9 is answered, over
   `[0, 1]`, by the run's two ends and no crossing. `solve_misses_none` is the statement this
   breaks. Whether a curve and a surface of the kernel's own reach it is not known.
3. **`isolate::roots` drops a stretch nothing proved, and answers `Ok`.** In several unknowns a
   box too narrow to halve that the test left unsure and `near` does not accept is refused
   (`GaveUp::Unsettled`: "not proved means refused"). In one unknown it is dropped. A crossing
   whose rate is enclosed by `Iv::ANY`, which `Function::slope` may answer, is lost and the search
   answers no root. `roots_empty_means_none` is the statement this breaks.

Two more places where the documentation says more than a proof could, neither shown to lose a
root, are noted at `solve_misses_none`: a box 200 halvings deep is read as narrow whatever its
width, and two crossings are merged where the floats they were polished to stand within two
widths of one another.

## What is trusted, and what is not stated yet

Trusted: that each statement says what is meant (read them); Rocq's checker and Flocq's model of
IEEE-754; that `rustc` and the hardware compute as that model does, natively and on wasm32; and
the hand transcriptions.

Not stated yet: the one-unknown search's own guarantees beyond the empty answer (its ends and
its merging of neighbouring roots read signs in floats, and what it certifies of a returned root
needs working out); `Jet`'s derivatives; `interval/minimum.rs`; `Kantorovich` and `Bounded`,
which no reading uses; and the enclosures each kind of curve and surface gives of itself
(`enclose.rs`), which every search here takes as its hypothesis.

## Next

Prove the `Admitted` statements of `Interval.v`, the layer the others stand on; then decide what
turns the Rust into Gallina, so the properties become theorems about the code and not about a
copy of it.
