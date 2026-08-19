"""Benchmark: our DogLeg/LM vs scipy references, plus drag frame rates.

    python -m gcs.bench

Solve times are for a *compiled* System (what dragging pays per frame) and
for compile+solve (one-shot).  Kill background CPU hogs before trusting numbers.
"""

from __future__ import annotations

import time
from collections.abc import Callable

import numpy as np

from gcs.constraints import Horizontal
from gcs.decompose import PlanSolver, execute
from gcs.examples import EXAMPLES, perturb, truss
from gcs.model import Sketch
from gcs.solve import METHODS, Drag, Method, System

MakeSketch = Callable[[], Sketch]


def bench_solve(make: MakeSketch, method: Method, reps: int = 20, sigma: float = 1.0) -> tuple[float, bool, float]:
    """(median compiled-solve ms, all ok, mean iterations)."""
    ts, ok, its = [], True, 0.0
    for i in range(reps):
        sk = make()
        perturb(sk, sigma, seed=i)
        s = System(sk)
        t0 = time.perf_counter()
        r = s.solve(method=method)
        ts.append(time.perf_counter() - t0)
        ok &= r.success
        its += r.iterations
    return float(np.median(ts)) * 1e3, ok, its / reps


def bench_compile(make: MakeSketch, reps: int = 20) -> float:
    sk = make()
    ts = []
    for _ in range(reps):
        t0 = time.perf_counter()
        System(sk)
        ts.append(time.perf_counter() - t0)
    return float(np.median(ts)) * 1e3


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
    sk.constraints = [c for c in sk.constraints if not isinstance(c, Horizontal)]
    return sk


def main() -> None:
    cases: dict[str, MakeSketch] = dict(EXAMPLES)
    cases["truss_50"] = lambda: truss(bays=50)
    cases["truss_100"] = lambda: truss(bays=100)
    print("== solve (perturbed warm start): compiled-solve ms / iterations ==")
    print(f"{'sketch':<14}{'free':>5}{'res':>5} | " + " | ".join(f"{m:^16}" for m in METHODS) + " | compile")
    for name, make in cases.items():
        sk = make()
        big = len(sk.params) > 100
        cells = []
        for m in METHODS:
            if m.startswith("scipy") and big:
                cells.append(f"{'(skipped)':^24}")   # scipy's dense LM/exact TR are O(n³) and very slow here
                continue
            ms, ok, it = bench_solve(make, m, reps=5 if big else 20)
            cells.append(f"{ms:7.2f} ms {'' if ok else 'BAD'} it={it:4.1f}")
        print(f"{name:<14}{len(sk.free_indices()):>5}{sk.n_residuals():>5} | " + " | ".join(cells)
              + f" | {bench_compile(make):5.2f} ms", flush=True)

    print("\n== decomposition plan (Stage 3): compile once, replay per solve ==")
    for name, make in cases.items():
        sk = make()
        t0 = time.perf_counter()
        ps = PlanSolver(sk)
        tc = time.perf_counter() - t0
        te = []
        for i in range(5):
            perturb(sk, 1.0, seed=i)
            t0 = time.perf_counter()
            execute(ps.plan)
            te.append(time.perf_counter() - t0)
        r = ps.solve(fallback=False)
        print(f"{name:<14} compile {tc * 1e3:7.1f} ms | replay {float(np.median(te)) * 1e3:7.2f} ms | "
              f"{ps.plan.summary()} | {'exact' if r.success else 'needs fallback'}", flush=True)

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
