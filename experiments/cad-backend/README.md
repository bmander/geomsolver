# Conventional CAD backend trial

This isolated Open CASCADE experiment tests an alternative export path for the documented
spiral-bevel pair. Solvent continues to define the geometry; a mature CAD kernel constructs
surface/solid topology, performs subtraction, and writes STEP. It does not require completing
a general F-rep mesher first. No runtime dependency or language construct changes here.

## Input and construction

The ignored Rust test `export_tooth_space_sections_for_cad_backend` reads and solves the
existing `paired_references.sv` default 24:48 pair, module 2 mm. It samples both generated
flanks and root fillets at 8, 16 and 32 subdivisions in each patch direction. Numerical
envelope samples are checked against the independent closed-form characteristic and the
declared trims. Sampling follows the cutter meridian parameter within each patch and
spherical distance across the face width. The gear's second side is indexed by one pitch
to bound a tooth space; the pinion's sides already bound one space.

`tooth_space.py` interpolates these grids into four B-spline side faces. Root/tip closures
and split toe/heel closures add six faces. The end closures interpolate spherical angles;
their fitted surfaces are approximations, not exact spherical/conical supports. Uniform
interpolation parameters preserve common edge curves between independently fitted patches.
The kernel sews at 0.000001 mm and forms an oriented solid. The candidate tooth space is
clipped to the nominal tip and toe/heel extent; it is not an unbounded physical cutter.

The blank is built independently as a full revolution of a meridian bounded by straight
cone generators and exact circular toe/heel arcs. Its endpoints come from the solved
Solvent cone/sphere intersections. The kernel subtracts one candidate tooth space and
exports/reimports both the tool space and the cut blank as STEP.

## Checks and limits

- Kernel validity, one shell, no free or multiple edges, orientation and positive volume.
- Comparison of the 8/16-subdivision fitted flank/fillet surfaces against the next finer
  source grids, including points not used in fitting; sample target 0.001 mm.
- Native and STEP solid counts/validity, positive material removal, and volume agreement
  after STEP round trip. Volume integration uses the kernel's adaptive Gauss integration
  with a relative target of 1e-10. Default non-adaptive integration gave misleading apparent
  STEP volume changes and is not used for these checks.

These are local CAD checks, not full acceptance of a matched pair. Kernel validity is not
an independent proof of embedding. The initial source checks are sampled; the subsequent
closure-support audit below adds whole-patch bounds. Local envelope validity does not
establish globally exposed material.
The next gates are whole-sweep material comparison, robust blank clipping/indexed cuts,
surface-error control, and independent pair/contact verification. The prototype does not
implement an arbitrary moving-solid sweep API or integrate finished solid outputs in Solvent.

An initial axial-height parameterization produced 10–16 micron sample errors, because that
coordinate becomes singular at the tangent root. Following the source cutter meridian
reduced the finer-grid flank/fillet errors to approximately 0.08 microns (pinion) and
0.023 microns (gear). This is a parameterization correction, not a tolerance relaxation.
The coarse pinion subtraction remains a recorded failure even though its tooth-space solid
itself is valid. All candidates and failed intermediates are retained under `/private/tmp`.

The completed [trial report](results.json) records:

| Member | Subdivisions | Sample error (mm) | Tooth-space / blank subtraction / STEP checks |
|---|---:|---:|---|
| Pinion | 8 | 0.00041246 | Space passes; subtraction produces invalid/multiple solids and is refused |
| Pinion | 16 | 0.00008000 | Local checks pass; one cut blank solid survives STEP round trip |
| Gear | 8 | 0.00010261 | Local checks pass |
| Gear | 16 | 0.00002342 | Local checks pass; one cut blank solid survives STEP round trip |

The finer cut pinion and gear volumes are approximately 11918.28069694 and 25789.53472155
mm³. STEP round-trip differences are below 0.0000001 mm³ with adaptive integration. The
reported subtraction stage includes validity and adaptive volume evaluation: approximately
5.8 and 7.2 seconds for those two candidates. This is evidence for the backend interface,
not a guarantee that every indexed cut or parameter choice will succeed.

## CAD material and indexed-cut follow-up

`material_probes.py` reads the actual finer STEP tooth-space and single-cut blank exports.
It samples the midpoint of each of ten faces, offsets 0.01 mm along both surface-normal
directions, and records the cut blank's CAD classification. Tool closures at tip/toe/heel
can correctly have both offsets outside the remaining material. The resulting 40 probes
include flanks, fillets, root floors and closures for both members.

The ignored Rust test `cad_export_material_probes_match_continuous_indexed_sweeps` transforms
those positions into the solved model frame and evaluates the complete finite blank minus
all continuous indexed generating sweeps. All 40 classifications agree, with every roll
minimization converged. `check_material.py` binds the evidence to the exact input probe
hash, labels, expectations and coordinate transform before invoking the independent
rational checker. That audit passed 1,440 indexed attained witnesses and 360 full-roll
positive covers comprising 14,564 cells. The minimum retained field margin is 0.0049000 mm;
the independently recomputed frame differs by at most 8.9e-16 mm. See
[material-results.json](material-results.json). The raw evidence remains in
`/private/tmp/solvent-cad-material.json`; this proves these point signs, not whole-surface
coverage, source-solve accuracy or contact correctness. The binding's rejection tests cover
changed expectations, missing points, wrong frames, unresolved native queries and altered
input hashes.

`indexed_cuts.py` applies all selected rotated tools in one Boolean operation. Two adjacent
cuts passed on both blanks, taking about nine seconds each for the Boolean stage. The full
24-cut pinion then passed native validity, single-solid and STEP round-trip checks; its
Boolean stage took 94.3 seconds. `check_indexed.py` checks actual STEP classifications at
every rotated copy of the independently audited probes: all 480 pinion positions agree.
This adds coverage around the circumference, not an independent full-roll proof at every
rotated location. [indexed-results.json](indexed-results.json) records these observations.

The complete 48-cut gear also passes native/STEP checks and all 960 rotated probe
classifications. Its Boolean stage took 219.2 seconds. The final CAD volumes are
9141.94525542 mm³ (pinion) and 20284.08432528 mm³ (gear), with STEP round-trip differences
below 0.000003 mm³. These files are complete nominal rim candidates, still pending
assembly-level contact/interference and whole-surface accuracy acceptance.

The kernel tessellated the full pinion STEP into 26,988 triangles in about 0.6 seconds at
0.05 mm linear and 0.2 rad angular settings. The actual binary STL passes the independent
exact topology/embedding audit: closed, outward, genus one, no improper triangle
intersections. The audit took about 24 seconds. Tessellation settings do not certify source
error; the mesh volume differs from the CAD volume by about 0.15%. It is an inspection
artifact, not a machining-accuracy mesh. Both full-pinion files are in
`/private/tmp/solvent-cad-full-pinion`.

The gear STL has 45,586 triangles and takes about 0.87 seconds to tessellate with the same
settings. Its independent exact embedding audit also passes (approximately 27 seconds),
with closed outward genus-one topology. Its mesh volume differs from the CAD volume by
about 0.11%. Files are in `/private/tmp/solvent-cad-full-gear`. A rendering of the actual
exported meshes in the documented zero-roll assembly frames is saved as
`/private/tmp/solvent-cad-pair-preview.png`; it is visual inspection, not a contact test.

```sh
/private/tmp/solvent-occt-env/bin/python experiments/cad-backend/material_probes.py /private/tmp/solvent-cad-one-cut-checked /private/tmp/solvent-cad-probes.txt
SOLVENT_CAD_PROBES_INPUT=/private/tmp/solvent-cad-probes.txt SOLVENT_CAD_MATERIAL_OUTPUT=/private/tmp/solvent-cad-material.json cargo test --manifest-path rust/Cargo.toml cad_export_material_probes_match_continuous_indexed_sweeps -- --ignored --nocapture
python3 experiments/cad-backend/check_material.py /private/tmp/solvent-cad-probes.txt /private/tmp/solvent-cad-material.json /private/tmp/solvent-cad-material-audit.json
/private/tmp/solvent-occt-env/bin/python experiments/cad-backend/indexed_cuts.py /private/tmp/solvent-cad-sections.json /private/tmp/solvent-cad-one-cut-checked /private/tmp/solvent-cad-full-pinion --full --member 0
/private/tmp/solvent-occt-env/bin/python experiments/cad-backend/check_indexed.py /private/tmp/solvent-cad-probes.txt /private/tmp/solvent-cad-material-audit.json /private/tmp/solvent-cad-full-pinion/report.json /private/tmp/solvent-cad-full-pinion/material-audit.json
```

For the adjacent-cut gate, omit `--full`; omit `--member` to process both members. To
tessellate use `export_stl.py SOURCE.step OUTPUT.stl`, then independently check with
`python3 rust/gcs-core/tests/verification/stl_embedding.py OUTPUT.stl`. The assembly checks
below use these exact full-member STEP files.

## Assembly and contact follow-up

`assembly.py` reads the actual full-member STEP files, verifies their hashes against the
indexed-cut reports and binds both reports to the same source. A tooth-period fraction
`f` rotates the pinion by `+2πf/24` and the gear by `−2πf/48` about their respective local
z axes, then applies the documented shaft tilts about y. The shared apex is the origin;
the shaft angle is 90 degrees. No extra half-tooth phase is added: the source's
complementary crown profiles already specify the zero-backlash assembly phase.

At tooth-period fractions 0, 0.25, 0.5, 0.75 and 1, Open CASCADE's intersection returns a
valid result with no solids and zero integrated volume. The final fraction checks the
index-period repeat. A negative control rotates only the gear an additional 0.001 radian:
the kernel then finds three intersection solids totaling approximately 0.71536 mm³.
These are kernel observations at five poses, not a bound on between-pose interference
or overlap below the kernel's numerical resolution. The command measures and records
interference; its exit status reports operation validity, not absence of overlap.

The existing Rust contact-window test can now export its independent analytical contacts
with `SOLVENT_CAD_CONTACTS_OUTPUT`. It covers both flanks, five spherical face stations and
25 contact samples per station, including tooth indexing into a shared assembly period.
`check_contacts.py` checks all 250 contact positions against both actual STEP shells.
Shell distance is used deliberately: containment in a solid must not produce a spurious
zero boundary distance. Independently recomputed assembly frames agree with the reference
within 5.8e-14 mm. Maximum sampled boundary distances are 5.4e-7 mm for the pinion and
2.2e-7 mm for the gear, against a 0.001 mm target. These very small residuals describe
contact-position agreement, not the accuracy of the entire CAD surface or original solve.

At 138 interior contact locations, the checker additionally classifies both solids at
the same world-space points offset by ±0.001 mm along the analytical contact normal.
All pairs have complementary material sides, reversing inside/outside across the contact.
The toe/heel stations and nearly trimmed tip endpoints are retained for distance checks
but excluded from offset checks because an offset can cross a second boundary there.
This is 500 boundary-distance queries and 552 solid classifications; the nominal contact
audit took about 62 seconds. [assembly-results.json](assembly-results.json) records the
sampled evidence, source and STEP hashes. Whole-surface/material coverage, continuous
interference, source-solve accuracy and public solid integration remain acceptance work.

```sh
SOLVENT_CAD_CONTACTS_OUTPUT=/private/tmp/solvent-cad-contacts.json cargo test --manifest-path rust/Cargo.toml both_flanks_have_overlapping_contact_windows_covering_a_full_tooth_period -- --nocapture
/private/tmp/solvent-occt-env/bin/python experiments/cad-backend/assembly.py /private/tmp/solvent-cad-sections.json /private/tmp/solvent-cad-full-pinion/report.json /private/tmp/solvent-cad-full-gear/report.json /private/tmp/solvent-cad-assembly.json --fractions 0 0.25 0.5 0.75 1
/private/tmp/solvent-occt-env/bin/python experiments/cad-backend/check_contacts.py /private/tmp/solvent-cad-sections.json /private/tmp/solvent-cad-full-pinion/report.json /private/tmp/solvent-cad-full-gear/report.json /private/tmp/solvent-cad-contacts.json /private/tmp/solvent-cad-contact-audit.json
```

Both scripts accept `--gear-offset 0.001` for the misphased control. Contact-check failures
are persisted and return a nonzero exit status; never substitute a negative-control report
for the nominal assembly evidence. This control fails the contact audit as intended, with
a maximum gear boundary miss of 0.0481 mm and failures of both distance and complementary
material-side checks.

## Whole-patch closure accuracy

The [independent rational support audit](SUPPORTS.md) now bounds all twelve tooth-space
closure surfaces against their nominal spheres and cones across their entire parameter
domains. All 3,072 Bézier patches pass the 0.001 mm target; the largest distance bound is
8.2e-6 mm. Exact knot insertion and Bernstein polynomial bounds operate on coefficients
extracted from the actual STEP files. This is a whole-patch result for closure support
distance, with explicit limits concerning trimmed coverage, generated flanks, source
accuracy and later indexed CAD operations. It does not establish full gear accuracy.

The [generated-fillet audit](FILLETS.md) also now bounds all four complete fillet support
surfaces within 0.001 mm of their common-crown parameterizations. It uses quadratic Taylor
bounds with interval third derivatives, exact Bernstein subdivision, and independent exact
parameter-cover checks. Each full audit takes approximately 81–86 seconds in a concurrent
four-member/side run; a deliberately distorted coefficient net is rejected after one cell.
Working-flank accuracy and the remaining whole-solid checks are still open.

## Initial trial reproduction

The frozen Python environment uses `cadquery-ocp==7.9.3.1.1` (OCCT 7.9.3) on CPython 3.12,
macOS x86-64. Its distribution also installs VTK and dependencies; they do not enter Solvent.

```sh
python3.12 -m venv /private/tmp/solvent-occt-env
/private/tmp/solvent-occt-env/bin/python -m pip install --only-binary=:all: -r experiments/cad-backend/requirements.txt
SOLVENT_CAD_SECTIONS_OUTPUT=/private/tmp/solvent-cad-sections.json cargo test --manifest-path rust/Cargo.toml export_tooth_space_sections_for_cad_backend -- --ignored --nocapture
/private/tmp/solvent-occt-env/bin/python experiments/cad-backend/tooth_space.py /private/tmp/solvent-cad-sections.json /private/tmp/solvent-cad-one-cut-checked
```

The script records failures and exits nonzero if any configuration fails. Generated STEP
files are inspection candidates even when the report refuses them. Setup, compilation,
sampling, kernel construction, Booleans and exchange have separate costs; no comprehensive
end-to-end performance claim is made.

## Why this route

[HyGEARS](https://www.hygears.com/hygears_4.0-february2020_019.htm) documents STEP export using
B-spline surfaces, separating flanks and fillets. [Open CASCADE's modeling documentation](https://occt3d.com/dev/doc/overview/html/occt_user_guides__modeling_algos.html)
provides the surface/solid operations used here, but its ordinary profile sweep explicitly
excludes solid inputs. A generating-envelope adapter remains our responsibility unless an
appropriate moving-solid sweep backend is identified. F-rep material queries and previous
independent gear equations remain useful verification assets alongside this CAD route.
