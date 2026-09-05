# Solid claim evidence

Claims are parsed into `SolidRequirement::Inside`, `Fits { gap: Length }`, or
`Clear { gap: Length }`. `Length` accepts only finite values. The enclosing
`SolidClaim` retains validated solid references and, for a sweep, its free
parameter, dimension and finite endpoints. These fields have read-only accessors.
Lengths stay in drawing units; angular free parameters stay in degrees.

`clear::evaluate_pair` uses the evaluated-solid interface and preserves a required
containment or disjointness predicate separately from the requested spacing.
The boundary-distance measurement is a checked finite interval, or explicitly
unbounded when an empty boundary has no finite distance. Failed evaluation has
an explicit reason and no measurement. Containment uses a signed boundary
distance, rather than an invented ±1 length. Curved contact can leave the required
predicate unresolved even when a negative requested gap is satisfied.

`clear::GeometricEvidence::holds()` derives the geometric answer: both the required predicate
and requested spacing must be satisfied. A refuted condition refutes the claim;
otherwise an unresolved condition prevents success. The interval must lie outside
the uncertainty around a spacing threshold; exact planar equality is accepted.
These rules do not introduce a new collision algorithm or guarantee the accuracy
of numerical evidence supplied by the existing geometry routines.

`diagnose::SolidVerdict` keeps every attempted `SolidPose`. Each pose contains its
parameter value and either solved, valid geometric evidence or a failure reason.
Single-pose claims check the current drawing's constraint residuals without
re-solving it; sweeps solve a scratch drawing at each parameter value.
The outcome is derived from the complete sequence:

- `Holds`: a single solved pose satisfies the claim.
- `SampledSuccess`: all 37 attempted poses satisfy it. This is not a proof over
  the continuous interval.
- `Refuted`: at least one solved, valid pose refutes it, including when other
  poses fail or remain uncertain.
- `Indeterminate`: no counterexample was found, but some evidence is unresolved.

A refuted report selects an actual counterexample. Otherwise it selects the
least measured valid pose. An all-failed sweep has no representative pose,
measurement, tolerance or counterexample. No missing value uses NaN or infinity.
Results and their constituent poses expose no public mutable fields or unchecked
constructors.

## Reports and API migration

`report::solid_claim_text` and `report::solid_claim_json` consume the same result.
The CLI uses the core text directly, and JSON includes it in `text`. JSON keeps
`measured`, `tolerance`, `samples`, `failedSamples` and `worst` for existing readers,
with absent measurements and tolerances represented by `null`. Swept success now
has verdict `sampled-success`; single-pose success remains `holds`. Refutation is
`refuted`; indeterminate results retain the serialized spelling `undecided`.

The structured evidence includes:

- `measurement`: `bounded` with `lower`/`upper`, `unbounded`, or `absent`.
- `poses`: every attempted parameter value, a `valid` or `failed` status, and
  either its geometric evidence and derived verdict or its failure reason.
- `requiredPredicate`: containment/disjointness status and unresolved reason,
  inside each valid pose.
- `coverage`: sweep parameter, dimension, user units, endpoints, uniform inclusive
  sampling method and attempted count.
- `validSamples`, `counterexample` (a complete pose or `null`), and
  `continuousProof: false`.

Rust consumers use result accessors instead of fields. `measured()` and
`tolerance()` return `Option<f64>`. The old word/gap evaluation entry points remain
as checked adapters; nonfinite gaps and gaps attached to `inside` return explicit
failed evidence before geometric evaluation.
