"""Benchmark: solve times, plan replay and drag frame rates.

    python -m gcs.bench

Solve times are for a *compiled* System (what dragging pays per frame) and for compile+solve
(one-shot).  Kill background CPU hogs before trusting numbers.
"""

from __future__ import annotations

import time
from collections.abc import Callable

import numpy as np

from gcs.decompose import PlanDrag, PlanSolver
from gcs.examples import EXAMPLES, perturb, truss, truss_floating, zigzag
from gcs.model import Sketch
from gcs.solve import METHODS, Drag, Method, System

MakeSketch = Callable[[], Sketch]


def bench_solve(make: MakeSketch, method: Method, reps: int = 20,
                sigma: float = 1.0) -> tuple[float, bool, float]:
    """(median compiled-solve ms, all ok, mean iterations)."""
    ts, ok, its = [], True, 0.0
    for i in range(reps):
        sk = make()
        perturb(sk, sigma, seed=i)
        s = System(sk)
        t0 = time.perf_counter()
        r = s.solve(method=method)
        ts.append(time.perf_counter() - t0)
        s.dispose()
        ok &= r.success
        its += r.iterations
    return float(np.median(ts)) * 1e3, ok, its / reps


def bench_compile(make: MakeSketch, reps: int = 20) -> float:
    sk = make()
    ts = []
    for _ in range(reps):
        t0 = time.perf_counter()
        System(sk).dispose()
        ts.append(time.perf_counter() - t0)
    return float(np.median(ts)) * 1e3


def bench_drag(sk: Sketch, frames: int = 20) -> tuple[float, bool]:
    p = sk.points[len(sk.points) // 2]
    d = Drag(sk, p, *p.xy)
    ts = []
    res = None
    for _ in range(frames):
        res = d.move(p.xy[0] + 1.0, p.xy[1] + 0.5)
        ts.append(res.time_s)
    d.end()
    return float(np.median(ts)) * 1e3, bool(res and res.success)


def bench_plan_drag(sk: Sketch, point_index: int, frames: int = 20,
                    plan: PlanSolver | None = None) -> tuple[float, float]:
    """(drag start ms, median frame ms) of the app's drag on `point_index` — on the sketch's
    cached plan when given one, as the app drags, else with a plan of its own."""
    p = sk.points[point_index]
    x, y = p.xy
    t0 = time.perf_counter()
    d = PlanDrag(sk, p, x, y, plan=plan)
    start = time.perf_counter() - t0
    ts = []
    for i in range(frames):
        t0 = time.perf_counter()
        d.move(x + 2.0 * np.cos(0.3 * i), y + 2.0 * np.sin(0.3 * i))
        ts.append(time.perf_counter() - t0)
    d.end()
    return start * 1e3, float(np.median(ts)) * 1e3


def main() -> None:
    cases: dict[str, MakeSketch] = dict(EXAMPLES)
    cases["truss_50"] = lambda: truss(bays=50)
    cases["truss_100"] = lambda: truss(bays=100)
    print("== solve (perturbed warm start): compiled-solve ms / iterations ==")
    print(f"{'sketch':<14}{'free':>5}{'res':>5} | " + " | ".join(f"{m:^16}" for m in METHODS)
          + " | compile")
    for name, make in cases.items():
        sk = make()
        big = len(sk.params) > 100
        cells = []
        for m in METHODS:
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
            ps.execute()
            te.append(time.perf_counter() - t0)
        r = ps.solve(fallback=False)
        print(f"{name:<14} compile {tc * 1e3:7.1f} ms | replay "
              f"{float(np.median(te)) * 1e3:7.2f} ms | {ps.plan.summary} | "
              f"{'exact' if r.success else 'needs fallback'}", flush=True)
        ps.dispose()

    print("\n== drag frame (pull + polish), dogleg ==")
    for bays in (30, 50, 100, 200):
        sk = truss(bays)
        ents = len(sk.points) + len(sk.lines)
        ms, ok = bench_drag(sk)
        ms2, ok2 = bench_drag(truss_floating(bays))
        print(f"truss({bays:3d}) {ents:5d} entities: fully constrained {ms:6.1f} ms "
              f"({1e3 / ms:4.0f} fps) | floating rigid {ms2:6.1f} ms ({1e3 / ms2:4.0f} fps) "
              f"{'ok' if ok and ok2 else 'BAD'}", flush=True)

    print("\n== drag of one figure among many (PlanDrag start + frame): the cost of the region ==")
    print("   own plan: the drag decomposes the figure | cached plan: as the app drags")
    for n, copies in ((32, 1), (32, 3), (32, 30), (128, 1), (2048, 1)):
        sk = zigzag(n, copies)
        start, frame = bench_plan_drag(sk, n // 2)   # a point of the first staircase
        ps = PlanSolver(sk, sticky=True)
        ps.solve()
        start2, frame2 = bench_plan_drag(sk, n // 2, plan=ps)
        ps.dispose()
        print(f"zigzag {n:4d} x {copies:2d} ({len(sk.points):5d} points): own plan start {start:7.2f} ms "
              f"frame {frame:6.3f} ms | cached plan start {start2:6.2f} ms frame {frame2:6.3f} ms",
              flush=True)


if __name__ == "__main__":
    main()
