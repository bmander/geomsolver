# What the certified layers claim, written to be proved

The kernel's robustness rests on three small layers of `src/util/`: intervals that enclose
(`interval.rs`), exact predicates (`expansion.rs`, `predicates.rs`) and certified root isolation
(`isolate.rs`). Their module documentation says what each guarantees. This directory says it
exactly, in [Rocq](https://rocq-prover.org) over [Flocq](https://flocq.gitlabpages.inria.fr)'s
IEEE-754, so that each guarantee is a statement a proof assistant can check a proof of.

**The interval arithmetic is proved; the rest is stated.** Every theorem of `Interval.v` is
checked, on the transcription of the Rust. The other layers are stated and unproved
(`Admitted`), or stated as a property awaiting the function it is about. A statement not yet
proved may still be wrong in a detail; proving it is what finds out.

```bash
make        # check every statement and proof (Rocq 8.18+, Flocq 4: apt install coq libcoq-flocq)
```

```bash
make gaps   # run the kernel outside the range the predicates are stated over
```

Nothing here is linked into the kernel, built by its suite or shipped in the crate.

## The statements

| file | what it states | of |
|---|---|---|
| `Float.v` | the model of `f64`: binary64, round to nearest even | — |
| `Rounding.v` | a rounded operation is its exact result rounded, or the infinity of its sign; one float outward brackets it | the floats (**proved**) |
| `Interval.v` | every operation that answers `Ok` holds the real result for every real its operands hold | the arithmetic, transcribed (**proved**) |
| `Predicates.v` | each error-free transformation's pair sums to the exact result | `two_sum` … `two_product`, transcribed |
| | `orient`, `power`, `orient2`, `in_circle` each return the sign of their determinant, over a stated range | a property |
| `Isolate.v` | a stretch `Slope` reads off holds no root; one read monotone runs one way | `Slope::read`, transcribed |
| | a box `Krawczyk` reads off holds no root; one read as one holds exactly one | a property |
| | `solve` answering no root means there is none; each crossing it returns lies in a box holding exactly one root | properties |
| | `solve` misses no root; `roots` answering no root means there is none, where the tolerance resolves | properties not true when first written, and not yet shown to hold |

Three kinds of statement, told apart in the files:

- **Proved** (`Qed`): all of `Rounding.v` and `Interval.v`. `add`, `sub`, `mul`, `div`,
  `square`, `sqrt` and `neg` each hold the real result, a result that overflows to an infinity
  included (the operation then answers `Err`, or an end no real it must hold is past); `Iv`'s
  `new`, `point`, `hull`, `meet`, `has_zero` and `mag` say what the tests read them for. They
  depend on nothing but the axioms of Coq's real numbers (`Print Assumptions`).
- **Stated of a transcription** (`Admitted`, 5): the error-free transformations of
  `Predicates.v` and `Slope::read` of `Isolate.v`. The Rust function is written out in Gallina
  by hand and the theorem is about that; the transcription is trusted until a translator
  replaces it.
- **A property** (`Definition … : Prop`, 10): what any function standing for the Rust one must
  satisfy. The filter and expansions of the predicates, `sin_cos`, the Krawczyk test and the two
  searches are too long to transcribe by hand with confidence.

## What writing them down found

1. **The predicates are exact only in range.** `predicates::orient` answers 0 (coplanar) for a
   positively oriented tetrahedron of side 1e-108, and of side 1e103: the determinant underflows,
   or overflows, in the filter and in the expansions alike (`make gaps`). `expansion.rs` says it
   assumes neither happens; nothing checks. No drawing is near either scale, so the statements
   carry a range (`tame`: whole multiples of 2⁻¹⁵⁰ up to 2¹⁵⁰) and nothing else changes.
2. **`isolate::solve` dropped a box whole for a run that covered part of it.** A function nought
   over `[0.4, 0.6]` with a crossing at 0.9 was answered, over `[0, 1]`, by the run's two ends and
   no crossing. **Mended**: the box is read on past the run's ends
   (`roots::a_crossing_beside_a_run_in_the_box_it_was_found_from_is_found`).
3. **`isolate::roots` reads a stretch nothing proved by the tolerance alone, and nothing said
   what the tolerance must then guarantee.** A stretch too narrow to halve that the test left
   unsure is a touch where `near` accepts its middle and rootless where not. That is right for
   the kernel's own readings, where `near` is a distance within the tolerance and the width a
   thousandth of it, and it is needed: a line passing a cone's apex is nought to rounding there
   and no enclosure tells it from a meeting. It is wrong for a tolerance tighter than the width
   resolves: a crossing whose rate was enclosed by `Iv::ANY`, or only bounded in magnitude, was
   lost and the search answered no root. **Mended** two ways: the statement and the
   documentation carry the hypothesis (`near_resolves`), and a stretch whose ends are proved
   either side of nought answers its crossing whatever the tolerance says
   (`roots::a_crossing_proved_across_a_stretch_too_narrow_to_halve_is_found`). Refusing such a
   stretch outright, as several unknowns do, was tried first and refused that line and cone.

What still stands between the code and the two search statements, none of it shown to lose a
root, is said at each in `Isolate.v`: in one unknown, three readings of a monotone stretch that
rest on a sign no enclosure proved; in several, a box 200 halvings deep read as narrow, and two
crossings merged where their polished floats stand within two widths.

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

`slope_read_sound` now stands on proved ground and is the next to prove; then the error-free
transformations, which Flocq has the theory for. After that, decide what turns the Rust into
Gallina, so the properties (the trigonometry among them) become theorems about the code and not
about a copy of it.
