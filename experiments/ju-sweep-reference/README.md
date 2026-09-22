# Ju et al. sweep reference experiment

This runs the public implementation of **Lifted Surfacing of Generalized Sweep
Volumes**, separate from Solvent. It does not alter the sweep algorithm or repair
its output. See [the experiment report](../../docs/ju-sweep-reference-experiment.md)
for measured results and limitations.

Upstream: <https://github.com/Jurwen/Swept-Volume>, commit
`0bb260118443545029c98db2e98278fac27ba460`.

## Build

Commands below assume the Solvent repository is the current directory and use
new temporary paths. The example repository is about 1.6 GB; the source build
downloads further dependencies. CMake >= 3.28 and a C++20 compiler are required.
The measured machine used AppleClang 17, CMake 4.4.3, and Python 3.14.2 on Intel macOS.

```sh
git clone https://github.com/Jurwen/Swept-Volume.git /private/tmp/ju-swept-volume
git -C /private/tmp/ju-swept-volume checkout 0bb260118443545029c98db2e98278fac27ba460
git -C /private/tmp/ju-swept-volume apply "$PWD/experiments/ju-sweep-reference/macos-fmt.patch"
cmake -S /private/tmp/ju-swept-volume -B /private/tmp/ju-swept-volume/build \
  -DCMAKE_BUILD_TYPE=Release -DGEN_SWEEP_TESTS=ON \
  -DCPM_SOURCE_CACHE=/private/tmp/ju-sweep-cpm-cache
cmake --build /private/tmp/ju-swept-volume/build --target generalized_sweep -j 4
python3 -m venv /private/tmp/ju-sweep-python
/private/tmp/ju-sweep-python/bin/python -m pip install -r experiments/ju-sweep-reference/requirements.txt
```

The small `macos-fmt.patch` disables optional strong-type formatting integration
to avoid mixing Homebrew fmt headers with spdlog's bundled fmt. It changes build
definitions only. It was necessary on the measured machine; other environments
may build without it.

The upstream commit does **not** pin every dependency: `mtet` tracks `main`.
`dependencies.json` records the dependency checkouts used in this experiment.
For a matched rebuild, check out its recorded `mtet` commit
`a798adb85558584364f04474bc4fd54df66e1c26` in `build/_deps/mtet-src` after initial
configuration, then reconfigure with `-DFETCHCONTENT_UPDATES_DISCONNECTED=ON`
before building. Check the remaining dependency revisions against the manifest.

## Run and inspect

```sh
/private/tmp/ju-sweep-python/bin/python experiments/ju-sweep-reference/run.py \
  --source /private/tmp/ju-swept-volume --output /private/tmp/ju-sweep-results
/private/tmp/ju-sweep-python/bin/python experiments/ju-sweep-reference/preview.py \
  /private/tmp/ju-sweep-results /private/tmp/ju-sweep-preview.png
/private/tmp/ju-sweep-python/bin/python experiments/ju-sweep-reference/export_stls.py \
  --source /private/tmp/ju-swept-volume --results /private/tmp/ju-sweep-results \
  --output /private/tmp/ju-sweep-stls --case simple-stock \
  --case letter_L-stock --case flipping_torus-stock
/private/tmp/ju-sweep-python/bin/python experiments/ju-sweep-reference/collect_results.py \
  /private/tmp/ju-sweep-results /private/tmp/ju-sweep-stls
```

Use repeated `--case NAME` arguments to select cases. Names and transformations
are in `run.py`. Default generation timeout is 300 seconds per case; each exact
intersection scan has a 120-second soft budget, checked between candidate pairs.
Existing case directories are skipped to preserve evidence. Use a fresh output
directory for a complete repeat; interrupted cases are not silently rerun.

Each case retains the exact YAML inputs, process log, duration, return code,
native `.msh` meshes, audit JSON, decoded NumPy mesh, and an explicitly secondary
float32 STL. Environment metadata includes the executable hash and upstream
source diff. `results.json` collects the case records. Audit errors and timeouts
are recorded as incomplete evidence, not passes.

The audit checks original indexed incidence, connected components, orientation,
vertex links, exact degeneracy and triangle contacts using integer/rational
predicates on the represented coordinates. It reuses the independent verifier
in `rust/gcs-core/tests/verification/stl_embedding.py`. No proximity welding is
performed. Coordinate aliases are reported separately because the pair predicate
recognizes a shared simplex by coordinates. The legacy STL topology check assumes
one shell; its rejection of disconnected shells is not by itself a sweep failure.

For translated spheres, the audit also measures volume and sampled distances
against the analytic capsule. Reverse distances search nearby triangles and are
upper bounds **at those samples**, not a continuous Hausdorff bound. A completed
intersection scan certifies represented triangle contacts, not correspondence
to the continuous swept solid or correctness of cavity classification.

## Audit controls

```sh
/private/tmp/ju-sweep-python/bin/python -m unittest discover \
  -s rust/gcs-core/tests/verification -p test_stl_embedding.py -v
/private/tmp/ju-sweep-python/bin/python -m unittest discover \
  -s experiments/ju-sweep-reference -p test_audit.py -v
```

Controls include valid closed surfaces, holes, intersections, contacts, and a
float64-valid surface whose float32 export collapses. They validate the checks;
they are not successes of the reference sweep implementation.

## Sharp-prism follow-up

`run_prisms.py` generates sharp rectangular and triangular prism OBJ tools and
uses the existing CLI's mesh-distance input path. Its predefined OBJ motion is
translation from `(0.14, 0.51, 0.5)` to `(0.86, 0.51, 0.51)` and z rotation;
the upstream `-r 2` flag means 360 degrees, not two full turns. Subdivision adds
coplanar triangles without rounding the input edges.

```sh
/private/tmp/ju-sweep-python/bin/python experiments/ju-sweep-reference/run_prisms.py \
  --source /private/tmp/ju-swept-volume --output /private/tmp/ju-sweep-prisms
```

An additional adapter, `prism_probe.cpp`, passes the exact rectangular-box signed
distance and its spatial/time derivatives to the public sweep API. It runs a
sampled finite-difference derivative check first. It does not modify the sweep
engine. To build it, include `prism-probe.cmake` **once at the end** of the
upstream CMakeLists.txt, then build its target:

```sh
printf '\ninclude("%s/experiments/ju-sweep-reference/prism-probe.cmake")\n' "$PWD" >> /private/tmp/ju-swept-volume/CMakeLists.txt
cmake -S /private/tmp/ju-swept-volume -B /private/tmp/ju-swept-volume/build \
  -DFETCHCONTENT_UPDATES_DISCONNECTED=ON
cmake --build /private/tmp/ju-swept-volume/build --target ju_prism_probe -j 4
/private/tmp/ju-sweep-python/bin/python experiments/ju-sweep-reference/run_prisms.py \
  --source /private/tmp/ju-swept-volume --output /private/tmp/ju-sweep-prisms --analytic
```

The analytic probe limits refinement to 200,000 splits; an emitted mesh does not
by itself show that refinement converged to the requested epsilon. Its input
JSON and logs retain the settings. The OBJ trials use the supplied default split
limit, with a five-minute process timeout. Use `--case prism-tumble` to select a
single rectangular-prism rotation trial.

New runs use a separate working directory for each case: upstream refinement
temporarily saves and reloads a fixed filename, `init.msh`. Independent processes
must not share that working directory during this step.
