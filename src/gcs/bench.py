"""Benchmark: our DogLeg/LM vs scipy references, plus drag frame rates.

    python -m gcs.bench

Solve times are for a *compiled* System (what dragging pays per frame) and
for compile+solve (one-shot).  Kill background CPU hogs before trusting numbers.
"""

from __future__ import annotations

import time
from collections.abc import Callable

import numpy as np

from gcs.examples import EXAMPLES, perturb, truss
from gcs.model import Sketch
from gcs.solve import METHODS, Drag, Method, System

MakeSketch = Callable[[], Sketch]


def bench_solve(make: MakeSketch, method: Method, reps: int = 20, sigma: float = 1.0) -> tuple[float, float, bool, float]:
    """(median compiled-solve ms, median compile ms, all ok, mean iterations)."""
    ts, tc, ok, its = [], [], True, 0.0
    for i in range(reps):
        sk = make()
        perturb(sk, sigma, seed=i)
        x0 = sk.get_x()
        t0 = time.perf_counter()
        s = System(sk)
        tc.append(time.perf_counter() - t0)
        sk.set_x(x0)
        t0 = time.perf_counter()
        r = s.solve(method=method)
        ts.append(time.perf_counter() - t0)
        ok &= r.success
        its += r.iterations
    return float(np.median(ts)) * 1e3, float(np.median(tc)) * 1e3, ok, its / reps


def bench_drag(sk: Sketch, frames: int = 20) -> tuple[float, bool]:
    p = sk.points[len(sk.points) // 2]
    d = Drag(sk, p, *p.xy)
    ts = []
    for _ in range(frames):
        res = d.move(p.xy[0] + 1.0, p.xy[1] + 0.5)
        ts.append(res.time_s)
    d.end()
    return float(np.median(ts)) * 1e3, res.success


def floating(sk: Sketch) -> Sketch:
    """Unfix everything and drop the orientation constraint → rigid body with 3 DOF."""
    for prm in sk.params:
        prm.fixed = False
    sk.constraints = [c for c in sk.constraints if type(c).__name__ != "Horizontal"]
    return sk


def main() -> None:
    cases: dict[str, MakeSketch] = dict(EXAMPLES)
    cases["truss_50"] = lambda: truss(bays=50)
    cases["truss_100"] = lambda: truss(bays=100)
    print("== solve (perturbed warm start): compiled-solve ms / iterations ==")
    print(f"{'sketch':<14}{'free':>5}{'res':>5} | " + " | ".join(f"{m:^16}" for m in METHODS) + " | compile")
    for name, make in cases.items():
        sk = make()
        cells = []
        comp = 0.0
        for m in METHODS:
            if m.startswith("scipy") and len(sk.params) > 100:
                cells.append(f"{'(skipped)':^24}")   # scipy's dense LM/exact TR are O(n³) and very slow here
                continue
            ms, comp, ok, it = bench_solve(make, m, reps=5 if len(sk.params) > 100 else 20)
            cells.append(f"{ms:7.2f} ms {'' if ok else 'BAD'} it={it:4.1f}")
        print(f"{name:<14}{len(sk.free_indices()):>5}{sk.n_residuals():>5} | " + " | ".join(cells) + f" | {comp:5.2f} ms", flush=True)

    print("\n== drag frame (pull + polish), dogleg ==")
    for bays in (30, 50, 100, 200):
        sk = truss(bays)
        ents = len(sk.points) + len(sk.lines)
        ms, ok = bench_drag(sk)
        ms2, ok2 = bench_drag(floating(truss(bays)))
        print(f"truss({bays:3d}) {ents:5d} entities: fully constrained {ms:6.1f} ms ({1e3 / ms:4.0f} fps) | "
              f"floating rigid {ms2:6.1f} ms ({1e3 / ms2:4.0f} fps) {'ok' if ok and ok2 else 'BAD'}", flush=True)


if __name__ == "__main__":
    main()
