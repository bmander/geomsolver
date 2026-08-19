# geomsolver

Geometric constraint solver built in stages per `gcs-solver-program.md` (read it first).
Currently: **Stage 0** — pure Python, scipy least-squares. Python ≥ 3.14 in `.venv/`.

Commands: `.venv/bin/pytest` (Qt tests run offscreen), `.venv/bin/mypy` (strict, must stay clean),
`.venv/bin/python -m gcs.bench`, `.venv/bin/python -m gcs.app` (PySide6 sketcher), `.venv/bin/python -m gcs.canvas <example>`.

Conventions:
- Every new constraint type declares `spec` (constructor args as (attr, kind) pairs — drives JSON I/O, the app's
  constraint list, value editing and the toolbar applier), has an analytic Jacobian, and gets a row in
  `tests/test_jacobians.py::all_constraints` (which also checks spec round-trips).
- `gcs.solve.Drag` is the one drag implementation (pull + polish); frontends only translate coordinates.
- Determinism: ordered lists only, no set/dict-order-dependent iteration in the solve path.
- `System` is the compile-once / evaluate-many seam; keep Python object model out of the hot loop.
- Go to C/Cython only when profiling shows the bottleneck (user's call).
