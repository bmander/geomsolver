"""Benchmark: solve time per method on the example sketches (perturbed warm start).

    python -m gcs.bench
"""

from __future__ import annotations

import time
from collections.abc import Callable

import numpy as np

from gcs.examples import EXAMPLES, perturb, truss
from gcs.model import Sketch
from gcs.solve import METHODS, Method, solve

MakeSketch = Callable[[], Sketch]


def bench(make: MakeSketch, method: Method, reps: int = 20, sigma: float = 1.0) -> tuple[float, bool, float]:
    ts, ok, nfev = [], True, 0
    for i in range(reps):
        sk = make()
        perturb(sk, sigma, seed=i)
        t0 = time.perf_counter()
        r = solve(sk, method=method)
        ts.append(time.perf_counter() - t0)
        ok &= r.success
        nfev += r.nfev
    return float(np.median(ts)) * 1e3, ok, nfev / reps


def main() -> None:
    cases: dict[str, MakeSketch] = dict(EXAMPLES)
    cases["truss_50"] = lambda: truss(bays=50)
    cases["truss_200_free"] = lambda: truss(bays=200, dims=False)
    print(f"{'sketch':<16}{'params':>7}{'res':>5} | " + " | ".join(f"{m:^22}" for m in METHODS))
    for name, make in cases.items():
        sk = make()
        row = f"{name:<16}{len(sk.params):>7}{sk.n_residuals():>5} | "
        cells = []
        for m in METHODS:
            ms, ok, nf = bench(make, m, reps=5 if len(sk.params) > 300 else 20)
            cells.append(f"{ms:8.2f} ms {'ok ' if ok else 'BAD'} nfev={nf:4.1f}")
        print(row + " | ".join(cells))


if __name__ == "__main__":
    main()
