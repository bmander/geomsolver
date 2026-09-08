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
an independent proof of embedding. Source checks are sampled, closure approximation error
is not certified, and local envelope validity does not establish globally exposed material.
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

## Reproduce

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
